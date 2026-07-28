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
 * @file   test-mola_metric_maps_incrementalpointcloud-stress.cpp
 * @brief  Long-running, LiDAR-odometry-like stress test of IncrementalPointCloud
 * @author Jose Luis Blanco Claraco
 * @date   Jul 28, 2026
 *
 * The functional test (test-mola_metric_maps_incrementalpointcloud.cpp) checks
 * results on small maps. This one instead reproduces the *usage pattern* of
 * mola_lidar_odometry, which is where non-deterministic failures have been
 * observed: full-size scans, a sliding window that continuously evicts and
 * recycles storage slots, background k-d tree rebuilds in flight, cov2cov
 * queries running the TBB paths, map copy-outs for publishing, and a mapping
 * thread concurrent with an ICP thread.
 *
 * It is meant to be run under a sanitizer or valgrind. See
 * `tests/README-sanitizers.md`.
 *
 * Usage: [<frames>] [<seed>]. The default frame count keeps the plain `ctest`
 * run short; chasing a non-deterministic failure wants a few hundred frames and
 * several seeds, which is what tests/run-sanitizers.sh is for.
 */

#include <mola_metric_maps/IncrementalPointCloud.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

namespace
{
// Sized after a decimated KITTI/Oxford local map: big enough for the k-d tree's
// background rebuilds to actually trigger (nanoflann's MT index does not
// rebuild below 10k nodes).
constexpr size_t kPointsPerScan = 12000;
constexpr double kKeepHalfSide  = 40.0;  // [m], the sliding-window trim
constexpr double kStepPerFrame  = 1.5;  // [m], ~15 m/s at 10 Hz

mrpt::random::CRandomGenerator rng;

/** A vehicle-like scan: ground plane, two side walls and some clutter, so the
 *  plane-regularized covariances have real structure to fit.
 */
mrpt::maps::CSimplePointsMap::Ptr makeScan(size_t n)
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  pts->reserve(n);

  for (size_t i = 0; i < n; i++)
  {
    const double u = rng.drawUniform(0.0, 1.0);
    if (u < 0.5)
    {
      // Ground plane, up to 50 m away:
      const double r  = rng.drawUniform(1.0, 50.0);
      const double th = rng.drawUniform(-M_PI, M_PI);
      pts->insertPointFast(
          static_cast<float>(r * std::cos(th)), static_cast<float>(r * std::sin(th)),
          static_cast<float>(-1.7 + rng.drawGaussian1D(0, 0.01)));
    }
    else if (u < 0.9)
    {
      // Two walls parallel to the driving direction:
      const double side = (rng.drawUniform(0.0, 1.0) < 0.5) ? -8.0 : 8.0;
      pts->insertPointFast(
          static_cast<float>(rng.drawUniform(-40.0, 40.0)),
          static_cast<float>(side + rng.drawGaussian1D(0, 0.01)),
          static_cast<float>(rng.drawUniform(-1.7, 6.0)));
    }
    else
    {
      // Clutter:
      const double r  = rng.drawUniform(2.0, 60.0);
      const double th = rng.drawUniform(-M_PI, M_PI);
      pts->insertPointFast(
          static_cast<float>(r * std::cos(th)), static_cast<float>(r * std::sin(th)),
          static_cast<float>(rng.drawUniform(-1.7, 8.0)));
    }
  }
  pts->mark_as_modified();
  return pts;
}

/// The scan as a map of the class the odometry pipeline uses for the
/// "observation" layer, i.e. the local side of every cov2cov query.
mola::IncrementalPointCloud::Ptr asLocalMap(const mrpt::maps::CSimplePointsMap& scan)
{
  auto local = mola::IncrementalPointCloud::Create();
  local->insertAnotherMap(&scan, mrpt::poses::CPose3D::Identity());
  return local;
}

mrpt::poses::CPose3D poseOfFrame(size_t f)
{
  // A gentle curve, so the keep box sweeps in both x and y:
  const double s = static_cast<double>(f) * kStepPerFrame;
  return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
      s, 20.0 * std::sin(s / 60.0), 0.02 * std::sin(s / 13.0), 0.3 * std::sin(s / 60.0), 0, 0);
}

/** Every invariant that a corrupted index or a desynchronized slot bookkeeping
 *  would break. Checked often, so a failure is reported near its cause instead
 *  of as a crash hundreds of frames later.
 */
void checkInvariants(
    const mola::IncrementalPointCloud& map, const mrpt::poses::CPose3D& robotPose, size_t frame)
{
  const auto live = map.liveCompactedCopy();

  ASSERTMSG_(
      live->size() == map.livePointCount(),
      mrpt::format(
          "frame %zu: livePointCount()=%zu but the compacted copy has %zu points", frame,
          map.livePointCount(), live->size()));

  ASSERTMSG_(
      map.livePointCount() <= map.size(),
      mrpt::format("frame %zu: more live points than storage slots", frame));

  const auto& xs = live->getPointsBufferRef_x();
  const auto& ys = live->getPointsBufferRef_y();
  const auto& zs = live->getPointsBufferRef_z();

  const auto c = robotPose.translation();

  for (size_t i = 0; i < live->size(); i++)
  {
    // A blanked (recycled) slot leaking into the live set:
    ASSERTMSG_(
        std::isfinite(xs[i]) && std::isfinite(ys[i]) && std::isfinite(zs[i]),
        mrpt::format("frame %zu: non-finite live point at index %zu", frame, i));

    // A live point far outside the keep cube means the tree is returning slots
    // that were evicted, i.e. the slot bookkeeping desynchronized. The margin
    // covers the points inserted after the last trim of this very frame.
    const double d =
        std::max({std::abs(xs[i] - c.x), std::abs(ys[i] - c.y), std::abs(zs[i] - c.z)});
    ASSERTMSG_(
        d < kKeepHalfSide + 65.0,
        mrpt::format(
            "frame %zu: live point %zu at Chebyshev distance %.1f m from the robot", frame, i, d));
  }
}

/// The nn_* indices must address the very point the query returned.
void checkQueryIndicesAddressTheirPoints(const mola::IncrementalPointCloud& map, size_t frame)
{
  for (size_t trial = 0; trial < 50; trial++)
  {
    const mrpt::math::TPoint3Df q = {
        static_cast<float>(rng.drawUniform(-60.0, 60.0)),
        static_cast<float>(rng.drawUniform(-60.0, 60.0)),
        static_cast<float>(rng.drawUniform(-5.0, 10.0))};

    mrpt::math::TPoint3Df p;
    float                 d  = 0;
    uint64_t              id = 0;
    if (!map.nn_single_search(q, p, d, id)) continue;

    ASSERTMSG_(
        id < map.size(), mrpt::format(
                             "frame %zu: nn_single_search returned slot %zu >= storage %zu", frame,
                             static_cast<size_t>(id), map.size()));

    float sx = 0;
    float sy = 0;
    float sz = 0;
    map.getPointFast(id, sx, sy, sz);

    ASSERTMSG_(
        std::isfinite(sx) && std::isfinite(sy) && std::isfinite(sz),
        mrpt::format(
            "frame %zu: nn_single_search returned a blanked slot %zu", frame,
            static_cast<size_t>(id)));

    ASSERTMSG_(
        std::abs(sx - p.x) < 1e-5f && std::abs(sy - p.y) < 1e-5f && std::abs(sz - p.z) < 1e-5f,
        mrpt::format(
            "frame %zu: slot %zu holds (%.3f %.3f %.3f) but the query returned "
            "(%.3f %.3f %.3f)",
            frame, static_cast<size_t>(id), sx, sy, sz, p.x, p.y, p.z));
  }
}

// -------------------------------------------------------------------------
/** The mola_lidar_odometry / icp_bench loop: trim, insert, match, publish.
 *  @param reservePoints as in TCreationOptions; 0 exercises the storage growth
 *         path, a large value the production configuration.
 */
void stress_odometry_like(size_t nFrames, bool asyncRebuild, uint64_t reservePoints)
{
  std::cout << "[stress] odometry-like: frames=" << nFrames << " async=" << asyncRebuild
            << " reserve=" << reservePoints << "\n";

  auto map = mola::IncrementalPointCloud::Create();

  map->creationOptions.remove_points_farther_than = kKeepHalfSide;
  map->creationOptions.async_rebuild              = asyncRebuild;
  map->creationOptions.reserve_points             = reservePoints;
  map->compact();  // apply the structural options above

  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorPose = mrpt::poses::CPose3D::Identity();

  for (size_t f = 0; f < nFrames; f++)
  {
    const auto robotPose = poseOfFrame(f);
    const auto scan      = makeScan(kPointsPerScan);

    // 1) Match this scan against the map, as ICP does on every frame. The
    //    local map is in the sensor frame; the pose is the current estimate.
    if (map->livePointCount() > 100)
    {
      const auto                        local = asLocalMap(*scan);
      mp2p_icp::MatchedPointWithCovList pairings;
      map->nn_search_cov2cov(*local, robotPose, 1.0f, pairings);

      for (const auto& p : pairings)
      {
        ASSERTMSG_(
            p.global_idx < map->size(),
            mrpt::format(
                "frame %zu: pairing global slot %u >= storage %zu", f, p.global_idx, map->size()));

        float gx = 0;
        float gy = 0;
        float gz = 0;
        map->getPointFast(p.global_idx, gx, gy, gz);
        ASSERTMSG_(
            std::abs(gx - p.global.x) < 1e-4f && std::abs(gy - p.global.y) < 1e-4f &&
                std::abs(gz - p.global.z) < 1e-4f,
            mrpt::format(
                "frame %zu: pairing global slot %u does not hold its own point", f, p.global_idx));

        for (int r = 0; r < 3; r++)
        {
          for (int c = 0; c < 3; c++)
          {
            ASSERTMSG_(
                std::isfinite(p.cov_inv(r, c)),
                mrpt::format("frame %zu: non-finite cov_inv in a pairing", f));
          }
        }
      }
    }

    // 2) Insert it into the local map (this trims the sliding window first):
    obs->pointcloud = scan;
    map->insertObservation(*obs, robotPose);

    // 3) Every few frames, the things the odometry does around the map:
    if (f % 7 == 3)
    {
      // Copy the layer out for publishing/visualization:
      mola::IncrementalPointCloud copy;
      copy.insertAnotherMap(map.get(), mrpt::poses::CPose3D::Identity());
      (void)copy.boundingBox();

      // And the compacted views:
      (void)map->getAsSimplePointsMap();
      (void)map->asString();
    }

    if (f % 5 == 0)
    {
      checkInvariants(*map, robotPose, f);
      checkQueryIndicesAddressTheirPoints(*map, f);
    }
  }

  std::cout << "[stress]   done: live=" << map->livePointCount() << " storage=" << map->size()
            << " free slots=" << map->recyclableSlotCount() << "\n";

  // A run that ends with the map cleared: internal_clear() drops the very
  // coordinate buffers a background rebuild may still be reading.
  map->clear();
  ASSERT_EQUAL_(map->livePointCount(), 0UL);
}

// -------------------------------------------------------------------------
/** The documented threading contract: "a mapping thread and an ICP thread may
 *  use the map concurrently".
 */
void stress_concurrent_mapping_and_icp(size_t nFrames)
{
  std::cout << "[stress] concurrent mapping + ICP threads: frames=" << nFrames << "\n";

  auto map = mola::IncrementalPointCloud::Create();

  map->creationOptions.remove_points_farther_than = kKeepHalfSide;
  map->creationOptions.async_rebuild              = true;
  map->compact();

  std::atomic_bool           done{false};
  std::atomic<size_t>        frame{0};
  std::atomic<unsigned long> queries{0};

  std::thread icpThread(
      [&]()
      {
        mrpt::random::CRandomGenerator localRng;
        localRng.randomize(77U);

        while (!done)
        {
          const auto pose = poseOfFrame(frame.load());

          mrpt::math::TPoint3Df q = {
              static_cast<float>(pose.x() + localRng.drawUniform(-30.0, 30.0)),
              static_cast<float>(pose.y() + localRng.drawUniform(-30.0, 30.0)),
              static_cast<float>(localRng.drawUniform(-2.0, 6.0))};

          mrpt::math::TPoint3Df p;
          float                 d  = 0;
          uint64_t              id = 0;
          if (map->nn_single_search(q, p, d, id))
          {
            ASSERT_(std::isfinite(p.x));
          }

          std::vector<mrpt::math::TPoint3Df> res;
          std::vector<float>                 dists;
          std::vector<uint64_t>              ids;
          map->nn_multiple_search(q, 20, res, dists, ids);
          for (const auto& r : res) ASSERT_(std::isfinite(r.x));

          queries++;
        }
      });

  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorPose = mrpt::poses::CPose3D::Identity();

  for (size_t f = 0; f < nFrames; f++)
  {
    frame.store(f);
    obs->pointcloud = makeScan(kPointsPerScan);
    map->insertObservation(*obs, poseOfFrame(f));
  }

  done = true;
  icpThread.join();

  std::cout << "[stress]   done: live=" << map->livePointCount()
            << " concurrent queries=" << queries.load() << "\n";
}

// -------------------------------------------------------------------------
/** Non-finite coordinates must never reach the k-d tree: the incremental index
 *  splits with std::nth_element on the coordinate, and a NaN there makes the
 *  comparator a non-strict-weak ordering, which is undefined behavior (in
 *  practice, out-of-bounds accesses). Real datasets do carry invalid returns,
 *  and the map itself writes NaN into recycled storage slots.
 */
void stress_nonfinite_input()
{
  std::cout << "[stress] non-finite input coordinates\n";

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  mola::IncrementalPointCloud map;

  // Enough points for the tree to actually be built and rebuilt:
  for (size_t i = 0; i < 30000; i++)
  {
    map.insertPoint(
        static_cast<float>(rng.drawUniform(-50, 50)), static_cast<float>(rng.drawUniform(-50, 50)),
        static_cast<float>(rng.drawUniform(-5, 5)));

    // A sprinkle of invalid returns, in each coordinate:
    if (i % 1000 == 500) map.insertPoint(nan, 1.0f, 2.0f);
    if (i % 1000 == 700) map.insertPoint(1.0f, nan, 2.0f);
    if (i % 1000 == 900) map.insertPoint(1.0f, 2.0f, inf);
  }

  for (size_t t = 0; t < 500; t++)
  {
    const mrpt::math::TPoint3Df q = {
        static_cast<float>(rng.drawUniform(-50, 50)), static_cast<float>(rng.drawUniform(-50, 50)),
        static_cast<float>(rng.drawUniform(-5, 5))};

    mrpt::math::TPoint3Df p;
    float                 d  = 0;
    uint64_t              id = 0;
    if (map.nn_single_search(q, p, d, id))
    {
      ASSERTMSG_(
          std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z),
          "a non-finite point was returned as a nearest neighbor");
    }
  }

  map.keepOnlyPointsNear({0, 0, 0}, 20.0);
  (void)map.liveCompactedCopy();

  std::cout << "[stress]   done: live=" << map.livePointCount() << " storage=" << map.size()
            << "\n";
}

// -------------------------------------------------------------------------
/** The class is a `mrpt::maps::CPointsMap`, so any generic MRPT code may reach
 *  its inherited storage buffers directly, including the `KDTreeCapable`
 *  interface every points map carries (`kdTreeNClosestPoint3DIdx()` and
 *  friends). That static k-d tree is built over the raw storage, holes
 *  included, and must not be corrupted by them.
 */
void stress_generic_pointsmap_kdtree()
{
  std::cout << "[stress] generic CPointsMap k-d tree over a map with recycled slots\n";

  mola::IncrementalPointCloud map;
  map.creationOptions.remove_points_farther_than = kKeepHalfSide;
  map.creationOptions.async_rebuild              = true;
  map.compact();

  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorPose = mrpt::poses::CPose3D::Identity();

  // Enough frames for the sliding window to evict, reclaim and recycle slots:
  for (size_t f = 0; f < 60; f++)
  {
    obs->pointcloud = makeScan(kPointsPerScan);
    map.insertObservation(*obs, poseOfFrame(f));
  }

  std::cout << "[stress]   live=" << map.livePointCount() << " storage=" << map.size()
            << " free slots=" << map.recyclableSlotCount() << "\n";
  ASSERTMSG_(
      map.size() > map.livePointCount(),
      "the scenario is void unless the storage actually has holes in it");

  // This is what a consumer holding it as a plain CPointsMap does (icp_bench's
  // map-quality metric, for one): walk every storage slot and ask the inherited
  // static k-d tree for its neighbors.
  const auto& pts = static_cast<const mrpt::maps::CPointsMap&>(map);

  std::vector<float>  distsSq;
  std::vector<size_t> idxs;

  const size_t step = std::max<size_t>(1, pts.size() / 2000);
  for (size_t i = 0; i < pts.size(); i += step)
  {
    mrpt::math::TPoint3Df q;
    pts.getPointFast(i, q.x, q.y, q.z);

    idxs.clear();
    distsSq.clear();
    pts.kdTreeNClosestPoint3DIdx(q.x, q.y, q.z, 12, idxs, distsSq);
  }

  std::cout << "[stress]   done\n";
}

// -------------------------------------------------------------------------
/** Dropping / replacing / shrinking the point storage while a background
 *  rebuild is reading it. The worker holds raw pointers into the inherited
 *  coordinate buffers, so every path that can free or move them must wait for
 *  it first.
 */
void stress_teardown_during_background_rebuild(size_t rounds)
{
  std::cout << "[stress] storage teardown while a background rebuild is in flight\n";

  // Big enough that the background build is still running when the teardown
  // below starts; the scan is made once and reused, so nothing slows the window
  // between triggering the rebuild and dropping the storage under it.
  const auto scan = makeScan(400000);

  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorPose = mrpt::poses::CPose3D::Identity();
  obs->pointcloud = scan;

  for (size_t r = 0; r < rounds; r++)
  {
    auto map                           = mola::IncrementalPointCloud::Create();
    map->creationOptions.async_rebuild = true;
    map->compact();

    // Returns with a freshly triggered background rebuild in flight: measured
    // with this scan size, the worker still has >150 ms of work left here.
    map->insertObservation(*obs, mrpt::poses::CPose3D::Identity());

    switch (r % 3)
    {
      case 0:
        // clear(): frees the buffers outright.
        map->clear();
        break;
      case 1:
        // Copy-assignment: clears and refills the storage wholesale.
        *map = mola::IncrementalPointCloud();
        break;
      case 2:
        // Destruction while the worker runs.
        break;
    }
  }

  std::cout << "[stress]   done: " << rounds << " rounds\n";
}

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    const size_t   nFrames = (argc > 1) ? std::strtoul(argv[1], nullptr, 10) : 60;
    const unsigned seed    = (argc > 2) ? std::strtoul(argv[2], nullptr, 10) : 1234U;

    rng.randomize(seed);

    stress_nonfinite_input();
    stress_generic_pointsmap_kdtree();
    stress_teardown_during_background_rebuild(30);

    stress_odometry_like(nFrames, true /*async rebuilds*/, 0 /*no pre-reserve*/);
    stress_odometry_like(nFrames, true /*async rebuilds*/, 2000000 /*as in the LO pipelines*/);
    stress_odometry_like(nFrames, false /*synchronous rebuilds*/, 0);

    stress_concurrent_mapping_and_icp(nFrames);

    std::cout << "All tests passed.\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "TEST FAILED: " << e.what() << "\n";
    return 1;
  }
}
