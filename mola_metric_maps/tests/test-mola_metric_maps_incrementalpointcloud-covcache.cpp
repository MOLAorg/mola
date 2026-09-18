/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   test-mola_metric_maps_incrementalpointcloud-covcache.cpp
 * @brief  Behavior and cost of the per-point covariance cache admission gate
 *         (TCreationOptions::min_neighbors_to_cache_cov)
 * @author Jose Luis Blanco Claraco
 * @date   Sep 18, 2026
 *
 * The map caches one plane-regularized covariance per point and reuses it for
 * the rest of that point's life. A covariance estimated while the neighborhood
 * was still thin is the one worth not keeping: the map only ever gains points
 * around an existing one, so that estimate is provisional, and the isotropic
 * fallback returned when too few neighbors were found would otherwise never be
 * upgraded. The gate leaves such a point uncached, at the price of one
 * neighbor search per query until its neighborhood fills in.
 *
 * The tests below pin the direction of that trade: raising the gate must make
 * the covariances the map hands to ICP measurably closer to what a map built
 * from scratch over the same live points would hand out, while caching fewer
 * of them. The benchmark part (longer, gated behind argv) puts a cost on it.
 *
 * Usage: [<benchmark frames>] [<seed>]. With no arguments only the checks run,
 * so the plain `ctest` run stays short.
 */

#include <mola_metric_maps/IncrementalPointCloud.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/bits_math.h>  // RAD2DEG
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "incrementalpointcloud_test_scans.h"

namespace
{
using mola::IncrementalPointCloud;
using mrpt::math::TPoint3Df;

constexpr double kMaxDistForCov = 2.0;  // as in the shipped LO pipelines
constexpr double kKeepHalfSide  = 40.0;
constexpr double kStepPerFrame  = 1.5;
constexpr size_t kPointsPerScan = 12000;

/// 1 caches every estimate, i.e. the behavior before the gate existed. The
/// other values bracket the two natural settings: the fallback threshold
/// (`min_correspondences_for_cov`) and the full neighborhood
/// (`k_correspondences_for_cov`, which is what 0 resolves to).
const std::vector<uint32_t> kThresholds = {1, 5, 10, 20};

constexpr uint32_t kCacheEverything = 1;

using CovMap = std::map<uint32_t, Eigen::Matrix3f>;

// -------------------------------------------------------------------------
// Reading the cached covariances back out
// -------------------------------------------------------------------------

/** Greedily picks points that are mutually farther apart than `minSeparation`,
 *  so that each of them is alone in its own covariance neighborhood.
 */
std::vector<TPoint3Df> pickSpreadPoints(
    const std::vector<TPoint3Df>& candidates, double minSeparation, size_t maxCount)
{
  std::vector<TPoint3Df> out;
  const double           minSqr = minSeparation * minSeparation;

  for (const auto& c : candidates)
  {
    if (out.size() >= maxCount) break;

    bool tooClose = false;
    for (const auto& o : out)
    {
      const double d =
          std::pow(c.x - o.x, 2.0) + std::pow(c.y - o.y, 2.0) + std::pow(c.z - o.z, 2.0);
      if (d < minSqr)
      {
        tooClose = true;
        break;
      }
    }
    if (!tooClose) out.push_back(c);
  }
  return out;
}

std::vector<TPoint3Df> livePointsOf(const IncrementalPointCloud& map)
{
  const auto  live = map.liveCompactedCopy();
  const auto& xs   = live->getPointsBufferRef_x();
  const auto& ys   = live->getPointsBufferRef_y();
  const auto& zs   = live->getPointsBufferRef_z();

  std::vector<TPoint3Df> out;
  out.reserve(live->size());
  for (size_t i = 0; i < live->size(); i++) out.push_back({xs[i], ys[i], zs[i]});
  return out;
}

/** A query map whose own per-point covariances are all the isotropic identity
 *  fallback: its points are mutually farther apart than `max_distance_for_cov`,
 *  so none of them sees a single neighbor. The GICP weight of a pairing then
 *  reduces to `(COV_global + I)^-1`, which inverts back into the map's own
 *  cached covariance. That is the only way to read the cache from outside.
 */
IncrementalPointCloud::Ptr makeProbeMap(const std::vector<TPoint3Df>& probes)
{
  ASSERT_(probes.size() >= 3);  // below that, computeCovariance() bails out early

  auto probeMap                                  = IncrementalPointCloud::Create();
  probeMap->creationOptions.max_distance_for_cov = kMaxDistForCov;

  for (const auto& p : probes) probeMap->insertPoint(p.x, p.y, p.z);

  return probeMap;
}

/** The covariances the map hands out for the points the probes fall on, keyed
 *  by the probe's own index. Reads the cache and computes whatever is not in
 *  it, exactly as an ICP iteration would.
 */
CovMap probeCovariances(const IncrementalPointCloud& map, const IncrementalPointCloud& probeMap)
{
  mp2p_icp::MatchedPointWithCovList pairings;
  map.nn_search_cov2cov(probeMap, mrpt::poses::CPose3D::Identity(), 0.05f, pairings);

  CovMap out;
  for (const auto& p : pairings)
  {
    const Eigen::Matrix3f w = p.cov_inv.asEigen().inverse();
    out[p.local_idx]        = (w - Eigen::Matrix3f::Identity()).eval();
  }
  return out;
}

/// A map holding exactly the live points of `src`, with every covariance
/// computed once against the final geometry, i.e. the from-scratch reference.
IncrementalPointCloud::Ptr referenceMapOf(const IncrementalPointCloud& src)
{
  auto ref                                  = IncrementalPointCloud::Create();
  ref->creationOptions.max_distance_for_cov = kMaxDistForCov;

  const auto live = src.liveCompactedCopy();
  ref->insertAnotherMap(live.get(), mrpt::poses::CPose3D::Identity());

  return ref;
}

// -------------------------------------------------------------------------
// Comparing covariances
// -------------------------------------------------------------------------

/// The estimated surface normal, i.e. the eigenvector of the smallest
/// eigenvalue. `isPlane` is false for the isotropic fallback, which has none.
Eigen::Vector3f surfaceNormal(const Eigen::Matrix3f& cov, bool& isPlane)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);

  // Eigenvalues come out ascending:
  isPlane = es.eigenvalues()(0) < 0.5f * es.eigenvalues()(2);
  return es.eigenvectors().col(0);
}

/// Angle [deg] between the two surface normals, or a negative value when either
/// covariance is the isotropic fallback.
double normalAngleDeg(const Eigen::Matrix3f& a, const Eigen::Matrix3f& b)
{
  bool                  aIsPlane = false;
  bool                  bIsPlane = false;
  const Eigen::Vector3f na       = surfaceNormal(a, aIsPlane);
  const Eigen::Vector3f nb       = surfaceNormal(b, bIsPlane);

  if (!aIsPlane || !bIsPlane) return -1.0;

  const double c = std::min(1.0, std::abs(static_cast<double>(na.dot(nb))));
  return mrpt::RAD2DEG(std::acos(c));
}

struct CovDiff
{
  size_t compared      = 0;
  size_t bothIsotropic = 0;
  size_t planeMismatch = 0;  ///< one is a plane, the other the fallback
  double maxFrobenius  = 0;

  std::vector<double> angles;  ///< [deg], plane-vs-plane pairs only

  [[nodiscard]] double meanAngle() const
  {
    if (angles.empty()) return 0;
    double s = 0;
    for (const double a : angles) s += a;
    return s / static_cast<double>(angles.size());
  }

  [[nodiscard]] double percentileAngle(double p) const
  {
    if (angles.empty()) return 0;
    std::vector<double> v = angles;
    std::sort(v.begin(), v.end());
    const auto i = static_cast<size_t>(p * static_cast<double>(v.size() - 1));
    return v[i];
  }

  [[nodiscard]] double maxAngle() const
  {
    if (angles.empty()) return 0;
    return *std::max_element(angles.begin(), angles.end());
  }

  /// Fraction of the compared pairs whose normal is off by more than 5 deg, or
  /// whose planarity verdict does not even agree. The headline staleness
  /// number: 0 means the map answers exactly like a fresh one.
  [[nodiscard]] double fractionOff5deg() const
  {
    if (compared == 0) return 0;
    size_t n = planeMismatch;
    for (const double a : angles)
    {
      if (a > 5.0) n++;
    }
    return static_cast<double>(n) / static_cast<double>(compared);
  }
};

CovDiff compareCovariances(const CovMap& got, const CovMap& expected)
{
  CovDiff d;

  for (const auto& [key, covGot] : got)
  {
    const auto it = expected.find(key);
    if (it == expected.end()) continue;

    const Eigen::Matrix3f& covExp = it->second;

    d.compared++;
    d.maxFrobenius = std::max<double>(d.maxFrobenius, (covGot - covExp).norm());

    bool gotIsPlane = false;
    bool expIsPlane = false;
    (void)surfaceNormal(covGot, gotIsPlane);
    (void)surfaceNormal(covExp, expIsPlane);

    if (!gotIsPlane && !expIsPlane)
    {
      d.bothIsotropic++;
      continue;
    }
    if (gotIsPlane != expIsPlane)
    {
      d.planeMismatch++;
      continue;
    }
    d.angles.push_back(normalAngleDeg(covGot, covExp));
  }
  return d;
}

// -------------------------------------------------------------------------
// Behavior
// -------------------------------------------------------------------------

IncrementalPointCloud::Ptr makeMap(uint32_t threshold, bool async = false)
{
  auto m                                        = IncrementalPointCloud::Create();
  m->creationOptions.max_distance_for_cov       = kMaxDistForCov;
  m->creationOptions.min_neighbors_to_cache_cov = threshold;
  m->creationOptions.async_rebuild              = async;
  return m;
}

void insertCloud(IncrementalPointCloud& m, const mrpt::maps::CSimplePointsMap& pts)
{
  m.insertAnotherMap(&pts, mrpt::poses::CPose3D::Identity());
}

/// Covariances of `probeMap`'s points as `m` hands them out, against a map
/// built from scratch over `m`'s current live points.
CovDiff stalenessOf(const IncrementalPointCloud& m, const IncrementalPointCloud& probeMap)
{
  const CovMap got      = probeCovariances(m, probeMap);
  const auto   ref      = referenceMapOf(m);
  const CovMap expected = probeCovariances(*ref, probeMap);
  return compareCovariances(got, expected);
}

/// 0 must behave exactly like `k_correspondences_for_cov`, which is what the
/// shipped default relies on.
void test_auto_threshold_resolves_to_k()
{
  std::cout << "[test] a threshold of 0 means k_correspondences_for_cov\n";

  mrpt::random::CRandomGenerator rng;
  rng.randomize(11U);
  const auto cloud = mola_test::makeScan(rng, 6000);

  std::vector<size_t> cached;

  for (const uint32_t t : {0U, 20U, 1U})
  {
    auto m = makeMap(t);
    ASSERT_EQUAL_(m->creationOptions.k_correspondences_for_cov, 20U);
    insertCloud(*m, *cloud);

    const auto probes   = pickSpreadPoints(livePointsOf(*m), 1.5 * kMaxDistForCov, 300);
    const auto probeMap = makeProbeMap(probes);
    (void)probeCovariances(*m, *probeMap);

    cached.push_back(m->cachedCovarianceCount());
  }

  ASSERT_EQUAL_(cached[0], cached[1]);

  // ...and it is a real gate, i.e. it does keep entries out:
  ASSERT_(cached[0] < cached[2]);

  std::cout << mrpt::format(
      "        cached: auto=%zu k=%zu everything=%zu\n", cached[0], cached[1], cached[2]);
}

/** The property the gate exists for: after points are inserted around
 *  already-queried ones, the covariances the map hands out must be closer to a
 *  from-scratch map's than they are with everything cached.
 */
void test_densification_improves_the_cache()
{
  std::cout << "[test] densification: a higher gate tracks the map better\n";

  mrpt::random::CRandomGenerator rng;
  rng.randomize(1234U);

  const auto sparse = mola_test::makeScan(rng, 3000);
  const auto extra  = mola_test::makeScan(rng, 15000);

  // One probe set, shared by every threshold, taken from the sparse cloud so
  // that every probe lands on a point queried while the map was still thin.
  std::vector<TPoint3Df> sparsePts;
  {
    const auto& xs = sparse->getPointsBufferRef_x();
    const auto& ys = sparse->getPointsBufferRef_y();
    const auto& zs = sparse->getPointsBufferRef_z();
    for (size_t i = 0; i < sparse->size(); i++) sparsePts.push_back({xs[i], ys[i], zs[i]});
  }
  const auto probes   = pickSpreadPoints(sparsePts, 1.5 * kMaxDistForCov, 200);
  const auto probeMap = makeProbeMap(probes);
  std::cout << "        probes: " << probes.size() << "\n";
  ASSERT_(probes.size() > 50);

  std::vector<double> off5;
  std::vector<size_t> mismatches;

  for (const uint32_t t : kThresholds)
  {
    auto m = makeMap(t);
    insertCloud(*m, *sparse);

    const CovMap before = probeCovariances(*m, *probeMap);
    ASSERT_(!before.empty());

    insertCloud(*m, *extra);

    const CovDiff d = stalenessOf(*m, *probeMap);
    ASSERT_(d.compared > 50);

    std::cout << mrpt::format(
        "        gate>=%-3u compared=%zu maxFrobenius=%.2e planeMismatch=%zu meanAngle=%.3f deg "
        "off5deg=%.1f%% cached=%zu\n",
        t, d.compared, d.maxFrobenius, d.planeMismatch, d.meanAngle(), 100.0 * d.fractionOff5deg(),
        m->cachedCovarianceCount());

    off5.push_back(d.fractionOff5deg());
    mismatches.push_back(d.planeMismatch);

    if (t == kCacheEverything)
    {
      // The cache was frozen: nothing moved despite the densification. If this
      // ever stops holding, the test data no longer exercises the problem.
      const CovMap  after  = probeCovariances(*m, *probeMap);
      const CovDiff frozen = compareCovariances(after, before);
      ASSERTMSG_(
          frozen.maxFrobenius < 1e-5,
          "a gate of 1 must keep the very first covariance estimate for every point");
      ASSERTMSG_(
          d.fractionOff5deg() > 0.05,
          mrpt::format(
              "with everything cached the map was expected to be measurably stale, but only "
              "%.2f%% of the normals are off by more than 5 deg",
              100.0 * d.fractionOff5deg()));
    }
  }

  // Monotonic in the gate, and the full-neighborhood setting must clear the
  // bulk of the staleness rather than a sliver of it.
  for (size_t i = 1; i < off5.size(); i++)
  {
    ASSERTMSG_(
        off5[i] <= off5[i - 1] + 1e-9,
        mrpt::format(
            "raising the gate from %u to %u made the staleness worse (%.3f -> %.3f)",
            kThresholds[i - 1], kThresholds[i], off5[i - 1], off5[i]));
  }
  ASSERTMSG_(
      off5.back() < 0.25 * off5.front(),
      mrpt::format(
          "the full-neighborhood gate only cut the staleness from %.3f to %.3f", off5.front(),
          off5.back()));

  // The isotropic fallback is the case the gate removes outright:
  ASSERT_(mismatches.back() <= mismatches.front());
}

/** Removing a neighbor changes a covariance just as inserting one does, but a
 *  point that was already well populated stays cached, so an eviction next to
 *  it is NOT tracked. Pinned here so the limitation is explicit and measured:
 *  the gate must at least never do worse than caching everything.
 */
void test_eviction_is_not_tracked()
{
  std::cout << "[test] eviction is outside the gate's reach\n";

  mrpt::random::CRandomGenerator rng;
  rng.randomize(4321U);

  const auto cloud = mola_test::makeScan(rng, 20000);

  std::vector<double> off5;

  for (const uint32_t t : {kCacheEverything, 20U})
  {
    auto m = makeMap(t);
    insertCloud(*m, *cloud);

    // Cache the covariances of a band of points that the trim below keeps, but
    // whose neighbors on the other side of the cut it drops.
    std::vector<TPoint3Df> nearTheCut;
    for (const auto& p : livePointsOf(*m))
    {
      // Just inside the cube of half side 10 m centered at the origin:
      const double cheb = std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)});
      if (cheb < 10.0 && cheb > 10.0 - kMaxDistForCov) nearTheCut.push_back(p);
    }

    const auto probes   = pickSpreadPoints(nearTheCut, 1.5 * kMaxDistForCov, 200);
    const auto probeMap = makeProbeMap(probes);
    ASSERT_(probes.size() > 20);

    (void)probeCovariances(*m, *probeMap);  // caches what qualifies

    m->keepOnlyPointsNear({0.f, 0.f, 0.f}, 10.0);

    const CovDiff d = stalenessOf(*m, *probeMap);
    ASSERT_(d.compared > 20);

    std::cout << mrpt::format(
        "        gate>=%-3u compared=%zu maxFrobenius=%.2e meanAngle=%.3f deg off5deg=%.1f%%\n", t,
        d.compared, d.maxFrobenius, d.meanAngle(), 100.0 * d.fractionOff5deg());

    off5.push_back(d.fractionOff5deg());
  }

  ASSERTMSG_(
      off5.back() <= off5.front() + 1e-9,
      "the gate made the post-eviction covariances worse than caching everything");
}

/** The odometry usage pattern: a sliding window that continuously evicts and
 *  recycles slots, background rebuilds in flight, and a cov2cov query per
 *  frame. Checks that nothing dangles, and that the gate still pays off once
 *  eviction is in the mix.
 */
void test_sliding_window_churn()
{
  std::cout << "[test] sliding-window churn\n";

  std::vector<double> off5;

  for (const uint32_t t : {kCacheEverything, 20U})
  {
    mrpt::random::CRandomGenerator rng;
    rng.randomize(7U);

    auto m                                        = makeMap(t, /*async*/ true);
    m->creationOptions.remove_points_farther_than = kKeepHalfSide;
    m->compact();  // apply the structural options

    auto obs        = mrpt::obs::CObservationPointCloud::Create();
    obs->sensorPose = mrpt::poses::CPose3D::Identity();

    constexpr size_t kFrames = 12;

    for (size_t f = 0; f < kFrames; f++)
    {
      const auto pose = mola_test::poseOfFrame(f, kStepPerFrame);
      const auto scan = mola_test::makeScan(rng, 4000);

      if (m->livePointCount() > 100)
      {
        auto local                                  = IncrementalPointCloud::Create();
        local->creationOptions.max_distance_for_cov = kMaxDistForCov;
        local->insertAnotherMap(scan.get(), mrpt::poses::CPose3D::Identity());

        mp2p_icp::MatchedPointWithCovList pairings;
        m->nn_search_cov2cov(*local, pose, 1.0f, pairings);

        for (const auto& p : pairings)
        {
          ASSERT_(p.global_idx < m->size());
          for (int r = 0; r < 3; r++)
          {
            for (int c = 0; c < 3; c++) ASSERT_(std::isfinite(p.cov_inv(r, c)));
          }
        }
      }

      obs->pointcloud = scan;
      m->insertObservation(*obs, pose);
    }

    // Probed around the current robot pose, where the live points are.
    const auto             pose = mola_test::poseOfFrame(kFrames - 1, kStepPerFrame);
    std::vector<TPoint3Df> near;
    for (const auto& p : livePointsOf(*m))
    {
      const double d =
          std::max({std::abs(p.x - pose.x()), std::abs(p.y - pose.y()), std::abs(p.z - pose.z())});
      if (d < 20.0) near.push_back(p);
    }

    const auto probes   = pickSpreadPoints(near, 1.5 * kMaxDistForCov, 200);
    const auto probeMap = makeProbeMap(probes);
    ASSERT_(probes.size() > 20);

    const CovDiff d = stalenessOf(*m, *probeMap);
    ASSERT_(d.compared > 20);

    std::cout << mrpt::format(
        "        gate>=%-3u live=%zu storage=%zu cached=%zu compared=%zu meanAngle=%.3f deg "
        "off5deg=%.1f%% computed=%llu\n",
        t, m->livePointCount(), m->size(), m->cachedCovarianceCount(), d.compared, d.meanAngle(),
        100.0 * d.fractionOff5deg(), static_cast<unsigned long long>(m->covarianceComputations()));

    off5.push_back(d.fractionOff5deg());
  }

  ASSERTMSG_(
      off5.back() <= off5.front() + 1e-9,
      "the gate lost ground to caching everything across the sliding-window churn");
}

/// The option must survive a round trip through every serialization path.
void test_options_round_trip()
{
  std::cout << "[test] options round trip\n";

  IncrementalPointCloud::TCreationOptions o;
  ASSERT_EQUAL_(o.min_neighbors_to_cache_cov, 0U);  // the shipped default
  o.min_neighbors_to_cache_cov = 7;

  mrpt::io::CMemoryStream buf;
  auto                    arch = mrpt::serialization::archiveFrom(buf);
  o.writeToStream(arch);

  buf.Seek(0);
  IncrementalPointCloud::TCreationOptions r;
  r.readFromStream(arch);
  ASSERT_EQUAL_(r.min_neighbors_to_cache_cov, 7U);

  // The text dump the map-layer exporter reads back:
  std::ostringstream dump;
  o.dumpToTextStream(dump);
  ASSERT_(dump.str().find("min_neighbors_to_cache_cov") != std::string::npos);

  // The .ini path:
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("map", "min_neighbors_to_cache_cov", 12);

  IncrementalPointCloud::TCreationOptions c;
  c.loadFromConfigFile(cfg, "map");
  ASSERT_EQUAL_(c.min_neighbors_to_cache_cov, 12U);

  // ...and an absent key keeps the current value:
  mrpt::config::CConfigFileMemory         empty;
  IncrementalPointCloud::TCreationOptions d;
  d.min_neighbors_to_cache_cov = 9;
  d.loadFromConfigFile(empty, "map");
  ASSERT_EQUAL_(d.min_neighbors_to_cache_cov, 9U);

  // A stream written before this option existed carries no value for it, so
  // reading one onto an existing map must fall back to the default instead of
  // keeping whatever that map happened to hold. The v2 layout, by hand:
  mrpt::io::CMemoryStream legacyBuf;
  auto                    legacy = mrpt::serialization::archiveFrom(legacyBuf);

  legacy << static_cast<int8_t>(2);
  legacy << static_cast<double>(30.0) << true << 0.75f << 0.5f << static_cast<uint64_t>(0)
         << static_cast<uint32_t>(20) << static_cast<uint32_t>(5) << static_cast<double>(1.0);
  legacy << false;  // v1
  legacy << static_cast<double>(0) << static_cast<double>(1e-3);  // v2

  legacyBuf.Seek(0);

  IncrementalPointCloud::TCreationOptions stale;
  stale.min_neighbors_to_cache_cov = 13;
  stale.readFromStream(legacy);

  ASSERT_EQUAL_(stale.min_neighbors_to_cache_cov, 0U);
  ASSERT_EQUAL_(stale.k_correspondences_for_cov, 20U);
  ASSERT_NEAR_(stale.remove_points_farther_than, 30.0, 1e-12);
}

/** Retuning anything that changes what a covariance would be must drop the
 *  ones already cached, or the map keeps handing out values computed with the
 *  previous settings for the rest of each point's life.
 */
void test_option_change_drops_the_cache()
{
  std::cout << "[test] retuning a covariance option drops the cache\n";

  mrpt::random::CRandomGenerator rng;
  rng.randomize(31U);
  const auto cloud = mola_test::makeScan(rng, 6000);

  const char* keys[] = {"k_correspondences_for_cov",   "min_correspondences_for_cov",
                        "max_distance_for_cov",        "min_neighbors_to_cache_cov",
                        "max_plane_deviation_for_cov", "plane_regularization_lambda"};

  for (const char* key : keys)
  {
    auto m = makeMap(kCacheEverything);
    insertCloud(*m, *cloud);

    const auto probes   = pickSpreadPoints(livePointsOf(*m), 1.5 * kMaxDistForCov, 200);
    const auto probeMap = makeProbeMap(probes);
    (void)probeCovariances(*m, *probeMap);

    ASSERT_(m->cachedCovarianceCount() > 0);

    // A value that differs from the current one, whatever the field means:
    mrpt::config::CConfigFileMemory cfg;
    cfg.write("map", key, 0.125);

    m->trySetCreationOptions(cfg, "map");

    ASSERTMSG_(
        m->cachedCovarianceCount() == 0,
        mrpt::format("changing '%s' left covariances cached from the previous value", key));
  }
}

/** With fewer than three live points there is no neighborhood to search, so
 *  nothing is computed from neighbors. The setting that keeps whatever was
 *  computed must still cache it, or it would not be the exact compatibility
 *  mode it is documented to be.
 */
void test_degenerate_map_admission()
{
  std::cout << "[test] a map too small to search\n";

  const std::vector<TPoint3Df> probes   = {{0.f, 0.f, 0.f}, {50.f, 0.f, 0.f}, {100.f, 0.f, 0.f}};
  const auto                   probeMap = makeProbeMap(probes);

  for (const uint32_t t : {kCacheEverything, 20U})
  {
    auto m = makeMap(t);
    m->insertPoint(0.f, 0.f, 0.f);
    m->insertPoint(0.01f, 0.f, 0.f);
    ASSERT_EQUAL_(m->livePointCount(), 2UL);

    (void)probeCovariances(*m, *probeMap);

    const std::size_t cached = m->cachedCovarianceCount();
    std::cout << mrpt::format("        gate>=%-3u cached=%zu\n", t, cached);

    if (t == kCacheEverything)
    {
      ASSERTMSG_(cached > 0, "the compatibility setting must keep the degenerate estimate");
    }
    else
    {
      ASSERTMSG_(cached == 0, "a real gate must not keep an estimate made from no neighbors");
    }
  }
}

// -------------------------------------------------------------------------
// Real data (optional dependency on mola_test_datasets)
// -------------------------------------------------------------------------

#if defined(TEST_DATASETS_ROOT)
void test_kitti_extract()
{
  const auto fil =
      mrpt::system::pathJoin({std::string(TEST_DATASETS_ROOT), "kitti", "kitti_00_extract.rawlog"});

  if (!mrpt::system::fileExists(fil))
  {
    std::cout << "[test] KITTI extract not found, skipping: " << fil << "\n";
    return;
  }

  std::cout << "[test] KITTI extract: " << fil << "\n";

  mrpt::obs::CRawlog dataset;
  ASSERT_(dataset.loadFromRawLogFile(fil));

  // Decimated: the point here is real LiDAR geometry, not scale.
  constexpr size_t kDecimation = 6;

  std::vector<mrpt::maps::CSimplePointsMap::Ptr> scans;
  for (size_t i = 0; i < dataset.size(); i++)
  {
    auto o =
        std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(dataset.getAsObservation(i));
    if (!o || o->sensorLabel != "lidar") continue;

    const auto& src = *o->pointcloud;
    const auto& xs  = src.getPointsBufferRef_x();
    const auto& ys  = src.getPointsBufferRef_y();
    const auto& zs  = src.getPointsBufferRef_z();

    auto pts = mrpt::maps::CSimplePointsMap::Create();
    pts->reserve(src.size() / kDecimation + 1);
    for (size_t k = 0; k < src.size(); k += kDecimation) pts->insertPointFast(xs[k], ys[k], zs[k]);
    pts->mark_as_modified();

    scans.push_back(pts);
  }
  ASSERT_(scans.size() >= 2);
  std::cout << "        scans: " << scans.size() << " x " << scans[0]->size()
            << " points (decimated by " << kDecimation << ")\n";

  std::vector<double> off5;

  for (const uint32_t t : kThresholds)
  {
    auto m = makeMap(t);
    m->insertAnotherMap(scans[0].get(), mrpt::poses::CPose3D::Identity());

    const auto probes   = pickSpreadPoints(livePointsOf(*m), 1.5 * kMaxDistForCov, 300);
    const auto probeMap = makeProbeMap(probes);
    ASSERT_(probes.size() > 50);

    (void)probeCovariances(*m, *probeMap);  // caches what qualifies

    // The following scans overlap the first one heavily, so most queried
    // covariances see their neighborhood change.
    for (size_t i = 1; i < scans.size(); i++)
    {
      m->insertAnotherMap(scans[i].get(), mrpt::poses::CPose3D::Identity());
    }

    const CovDiff d = stalenessOf(*m, *probeMap);
    ASSERT_(d.compared > 50);

    std::cout << mrpt::format(
        "        gate>=%-3u compared=%zu maxFrobenius=%.2e meanAngle=%.3f deg off5deg=%.1f%% "
        "cached=%zu\n",
        t, d.compared, d.maxFrobenius, d.meanAngle(), 100.0 * d.fractionOff5deg(),
        m->cachedCovarianceCount());

    off5.push_back(d.fractionOff5deg());
  }

  ASSERTMSG_(
      off5.back() < off5.front(),
      "on real LiDAR geometry the gate did not reduce the covariance staleness");
}
#endif

// -------------------------------------------------------------------------
// Benchmark
// -------------------------------------------------------------------------

/** A fixed world the vehicle drives through, sampled anew on every scan.
 *
 *  Drawing independent random points per frame, as the behavior tests do,
 *  would make the benchmark measure nothing: no map point is ever matched
 *  twice, so every covariance is computed once and none can go stale. Real
 *  consecutive scans re-measure the same surfaces from slightly different
 *  sample positions, which is what both fills the neighborhood of an
 *  already-cached point and asks for its covariance again.
 */
struct SyntheticWorld
{
  std::vector<TPoint3Df> pts;  ///< world frame
};

SyntheticWorld makeWorld(mrpt::random::CRandomGenerator& rng, size_t n, double pathLength)
{
  SyntheticWorld w;
  w.pts.reserve(n);

  const double x0 = -30.0;
  const double x1 = pathLength + 30.0;

  for (size_t i = 0; i < n; i++)
  {
    const double x = rng.drawUniform(x0, x1);

    // The trajectory's own lateral offset, so the walls follow the path:
    const double yPath = 20.0 * std::sin(x / 60.0);

    const double u = rng.drawUniform(0.0, 1.0);
    if (u < 0.5)
    {
      // Ground plane:
      w.pts.push_back(
          {static_cast<float>(x), static_cast<float>(yPath + rng.drawUniform(-50.0, 50.0)),
           static_cast<float>(-1.7 + rng.drawGaussian1D(0, 0.01))});
    }
    else if (u < 0.9)
    {
      // Two walls flanking the path:
      const double side = (rng.drawUniform(0.0, 1.0) < 0.5) ? -8.0 : 8.0;
      w.pts.push_back(
          {static_cast<float>(x), static_cast<float>(yPath + side + rng.drawGaussian1D(0, 0.01)),
           static_cast<float>(rng.drawUniform(-1.7, 6.0))});
    }
    else
    {
      // Clutter:
      w.pts.push_back(
          {static_cast<float>(x), static_cast<float>(yPath + rng.drawUniform(-55.0, 55.0)),
           static_cast<float>(rng.drawUniform(-1.7, 8.0))});
    }
  }
  return w;
}

/** The world points within sensor range of `pose`, subsampled to `n` and
 *  jittered by the sensor noise, in the sensor's own frame.
 */
mrpt::maps::CSimplePointsMap::Ptr scanFromWorld(
    const SyntheticWorld& w, const mrpt::poses::CPose3D& pose, size_t n,
    mrpt::random::CRandomGenerator& rng)
{
  constexpr double kMaxRange   = 60.0;
  constexpr double kRangeNoise = 0.02;

  const auto c = pose.translation();

  std::vector<uint32_t> visible;
  visible.reserve(w.pts.size());
  for (size_t i = 0; i < w.pts.size(); i++)
  {
    const auto&  p = w.pts[i];
    const double d = std::pow(p.x - c.x, 2.0) + std::pow(p.y - c.y, 2.0) + std::pow(p.z - c.z, 2.0);
    if (d < kMaxRange * kMaxRange) visible.push_back(static_cast<uint32_t>(i));
  }

  // A different random subset every scan, as a moving sensor produces:
  for (size_t i = visible.size(); i > 1; i--)
  {
    const auto j = static_cast<size_t>(rng.drawUniform(0.0, static_cast<double>(i)));
    std::swap(visible[i - 1], visible[std::min(j, i - 1)]);
  }
  if (visible.size() > n) visible.resize(n);

  const auto poseInv = -pose;

  auto pts = mrpt::maps::CSimplePointsMap::Create();
  pts->reserve(visible.size());
  for (const uint32_t i : visible)
  {
    const auto& p = w.pts[i];

    const auto local = poseInv.composePoint(mrpt::math::TPoint3D(
        p.x + rng.drawGaussian1D(0, kRangeNoise), p.y + rng.drawGaussian1D(0, kRangeNoise),
        p.z + rng.drawGaussian1D(0, kRangeNoise)));

    pts->insertPointFast(
        static_cast<float>(local.x), static_cast<float>(local.y), static_cast<float>(local.z));
  }
  pts->mark_as_modified();
  return pts;
}

struct BenchRow
{
  uint32_t threshold = 0;

  double insertMs   = 0;  ///< per scan
  double firstCovMs = 0;  ///< per scan
  double laterCovMs = 0;  ///< per scan, per extra ICP iteration
  double computes   = 0;  ///< covariance estimations per scan
  double cachedFrac = 0;  ///< of the live points

  CovDiff accuracy;
};

BenchRow runBenchmarkConfig(uint32_t threshold, size_t nFrames, uint32_t seed)
{
  constexpr int kIcpIterations = 4;

  BenchRow row;
  row.threshold = threshold;

  mrpt::random::CRandomGenerator rng;
  rng.randomize(seed);

  const SyntheticWorld world = makeWorld(rng, 120000, static_cast<double>(nFrames) * kStepPerFrame);

  auto m                                        = makeMap(threshold);
  m->creationOptions.remove_points_farther_than = kKeepHalfSide;
  m->compact();

  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorPose = mrpt::poses::CPose3D::Identity();

  const auto fN = static_cast<double>(nFrames);

  double tInsert      = 0;
  double tFirst       = 0;
  double tLater       = 0;
  size_t scoredFrames = 0;

  const auto now = []() { return std::chrono::steady_clock::now(); };
  const auto ms  = [](auto a, auto b)
  { return std::chrono::duration<double, std::milli>(b - a).count(); };

  for (size_t f = 0; f < nFrames; f++)
  {
    const auto pose = mola_test::poseOfFrame(f, kStepPerFrame);
    const auto scan = scanFromWorld(world, pose, kPointsPerScan, rng);

    // The odometry order: match the incoming scan against the map built from
    // the PREVIOUS scans, then insert it. Inserting first would let every
    // local point match the exact copy of itself just added, so no map point
    // would ever be matched twice and no covariance could go stale.
    auto local                                  = IncrementalPointCloud::Create();
    local->creationOptions.max_distance_for_cov = kMaxDistForCov;
    local->insertAnotherMap(scan.get(), mrpt::poses::CPose3D::Identity());

    if (m->livePointCount() > 100)
    {
      mp2p_icp::MatchedPointWithCovList pairings;

      for (int it = 0; it < kIcpIterations; it++)
      {
        pairings.clear();

        const auto t2 = now();
        m->nn_search_cov2cov(*local, pose, 1.0f, pairings);
        const auto t3 = now();

        if (it == 0)
        {
          tFirst += ms(t2, t3);
        }
        else
        {
          tLater += ms(t2, t3);
        }
      }
      scoredFrames++;

      // Everything that describes the state ICP just ran against, sampled on
      // the last frame and BEFORE its insertion, so it is the cache as the
      // solver saw it rather than one refreshed by the insertion that follows.
      if (f + 1 == nFrames)
      {
        row.computes   = static_cast<double>(m->covarianceComputations()) / fN;
        row.cachedFrac = static_cast<double>(m->cachedCovarianceCount()) /
                         std::max<double>(1.0, static_cast<double>(m->livePointCount()));

        std::vector<TPoint3Df> matched;
        matched.reserve(pairings.size());
        for (const auto& p : pairings) matched.push_back(p.global);

        const auto probes = pickSpreadPoints(matched, 1.5 * kMaxDistForCov, 400);
        if (probes.size() >= 3)
        {
          const auto probeMap = makeProbeMap(probes);
          row.accuracy        = stalenessOf(*m, *probeMap);
        }
      }
    }

    obs->pointcloud = scan;

    const auto t0 = now();
    m->insertObservation(*obs, pose);
    const auto t1 = now();
    tInsert += ms(t0, t1);
  }

  row.insertMs   = tInsert / fN;
  row.firstCovMs = tFirst / std::max(1.0, static_cast<double>(scoredFrames));
  row.laterCovMs =
      tLater / (std::max(1.0, static_cast<double>(scoredFrames)) * (kIcpIterations - 1));

  return row;
}

void runBenchmark(size_t nFrames, uint32_t seed)
{
  std::cout << "\n=== min_neighbors_to_cache_cov benchmark: " << nFrames << " frames of "
            << kPointsPerScan << " points, max_distance_for_cov=" << kMaxDistForCov << " m, seed "
            << seed << " ===\n\n";

  // Plus the degenerate end of the range: above `k_correspondences_for_cov`
  // nothing can ever qualify, so every query recomputes and the map is always
  // exact. That is the upper bound the cheaper settings are traded against.
  std::vector<uint32_t> sweep = kThresholds;
  sweep.push_back(21);

  std::vector<BenchRow> rows;
  for (const uint32_t t : sweep)
  {
    std::cout << "  running gate>=" << t << " ...\n" << std::flush;
    rows.push_back(runBenchmarkConfig(t, nFrames, seed));
  }

  std::cout << "\n";
  std::cout << "| gate | insert | 1st cov2cov | later cov2cov | cov/scan | cached | mean ang | p95 "
               "ang | max ang | >5 deg |\n";
  std::cout << "|------|--------|-------------|---------------|----------|--------|----------|----"
               "-----|---------|--------|\n";

  for (const auto& r : rows)
  {
    std::cout << mrpt::format(
        "| %4u | %6.1f | %11.1f | %13.1f | %8.0f | %5.1f%% | %8.2f | %7.2f | %7.2f | %5.1f%% |\n",
        r.threshold, r.insertMs, r.firstCovMs, r.laterCovMs, r.computes, 100.0 * r.cachedFrac,
        r.accuracy.meanAngle(), r.accuracy.percentileAngle(0.95), r.accuracy.maxAngle(),
        100.0 * r.accuracy.fractionOff5deg());
  }
  std::cout << "\nTimes are milliseconds per scan. The angle columns are the error of the "
               "estimated\nsurface normal against a map built from scratch over the same live "
               "points.\n\n";
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    test_options_round_trip();
    test_degenerate_map_admission();
    test_option_change_drops_the_cache();
    test_auto_threshold_resolves_to_k();
    test_densification_improves_the_cache();
    test_eviction_is_not_tracked();
    test_sliding_window_churn();
#if defined(TEST_DATASETS_ROOT)
    test_kitti_extract();
#endif

    if (argc > 1)
    {
      const auto nFrames = static_cast<size_t>(std::atoi(argv[1]));
      const auto seed    = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 1234U;
      runBenchmark(nFrames, seed);
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
  std::cout << "Test passed.\n";
  return 0;
}
