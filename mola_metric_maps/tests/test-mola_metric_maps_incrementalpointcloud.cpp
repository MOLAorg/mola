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
 * @file   test-mola_metric_maps_incrementalpointcloud.cpp
 * @brief  Test the incremental (self-balancing k-d tree) point cloud map
 * @author Jose Luis Blanco Claraco
 * @date   Jul 25, 2026
 */

#include <mola_metric_maps/IncrementalPointCloud.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
mrpt::random::CRandomGenerator rng;

/// Points uniformly spread in a cube of the given half side around `center`.
mrpt::maps::CSimplePointsMap::Ptr randomCloud(
    size_t n, const mrpt::math::TPoint3Df& center, double halfSide)
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  pts->reserve(n);
  for (size_t i = 0; i < n; i++)
  {
    pts->insertPointFast(
        static_cast<float>(center.x + rng.drawUniform(-halfSide, halfSide)),
        static_cast<float>(center.y + rng.drawUniform(-halfSide, halfSide)),
        static_cast<float>(center.z + rng.drawUniform(-halfSide, halfSide)));
  }
  pts->mark_as_modified();
  return pts;
}

/// An independent (MRPT static k-d tree) view of the live points, used as oracle.
mrpt::maps::CSimplePointsMap::Ptr bruteForceReference(const mola::IncrementalPointCloud& map)
{
  const auto live = map.liveCompactedCopy();

  auto ref = mrpt::maps::CSimplePointsMap::Create();
  ref->reserve(live->size());
  for (size_t i = 0; i < live->size(); i++)
  {
    float x = 0;
    float y = 0;
    float z = 0;
    live->getPointFast(i, x, y, z);
    ref->insertPointFast(x, y, z);
  }
  ref->mark_as_modified();
  return ref;
}

void insertPoints(mola::IncrementalPointCloud& map, const mrpt::maps::CSimplePointsMap& pts)
{
  for (size_t i = 0; i < pts.size(); i++)
  {
    float x = 0;
    float y = 0;
    float z = 0;
    pts.getPointFast(i, x, y, z);
    map.insertPoint(x, y, z);
  }
}

// -------------------------------------------------------------------------
void test_queries_against_brute_force(bool async)
{
  mola::IncrementalPointCloud map;
  map.creationOptions.async_rebuild = async;
  map.compact();  // apply the structural option above
  ASSERT_(map.isEmpty());

  // A first batch, then a trim, then a second batch: the index must be correct
  // across recycled slots and tombstones.
  insertPoints(map, *randomCloud(2000, {0, 0, 0}, 20.0));
  ASSERT_EQUAL_(map.livePointCount(), 2000UL);
  ASSERT_(!map.isEmpty());

  map.keepOnlyPointsNear({5.0f, 0.0f, 0.0f}, 10.0);
  ASSERT_(map.livePointCount() < 2000);

  insertPoints(map, *randomCloud(2000, {5, 0, 0}, 12.0));

  const auto ref = bruteForceReference(map);
  ASSERT_EQUAL_(map.livePointCount(), ref->size());

  for (size_t trial = 0; trial < 200; trial++)
  {
    const mrpt::math::TPoint3Df q = {
        static_cast<float>(rng.drawUniform(-20.0, 20.0)),
        static_cast<float>(rng.drawUniform(-20.0, 20.0)),
        static_cast<float>(rng.drawUniform(-20.0, 20.0))};

    // 1) single NN
    {
      mrpt::math::TPoint3Df p;
      float                 d  = 0;
      uint64_t              id = 0;
      const bool            ok = map.nn_single_search(q, p, d, id);
      mrpt::math::TPoint3Df gt_p;
      float                 gt_d  = 0;
      uint64_t              gt_id = 0;
      const bool            gt_ok = ref->nn_single_search(q, gt_p, gt_d, gt_id);

      ASSERT_EQUAL_(ok, gt_ok);
      if (!ok) continue;

      ASSERT_NEAR_(d, gt_d, 1e-4f);

      // The returned index must address the live point in our own storage:
      float sx = 0;
      float sy = 0;
      float sz = 0;
      map.getPointFast(id, sx, sy, sz);
      ASSERT_NEAR_(sx, p.x, 1e-6f);
      ASSERT_NEAR_(sy, p.y, 1e-6f);
      ASSERT_NEAR_(sz, p.z, 1e-6f);
    }

    // 2) k-NN
    {
      constexpr size_t                   knn = 5;
      std::vector<mrpt::math::TPoint3Df> res;
      std::vector<float>                 dists;
      std::vector<uint64_t>              ids;
      map.nn_multiple_search(q, knn, res, dists, ids);

      std::vector<mrpt::math::TPoint3Df> gt_res;
      std::vector<float>                 gt_dists;
      std::vector<uint64_t>              gt_ids;
      ref->nn_multiple_search(q, knn, gt_res, gt_dists, gt_ids);

      ASSERT_EQUAL_(res.size(), gt_res.size());
      for (size_t k = 0; k < res.size(); k++)
      {
        ASSERT_NEAR_(dists[k], gt_dists[k], 1e-4f);
      }
    }

    // 3) radius search (unlimited count): same set of neighbors
    {
      constexpr float                    radiusSqr = 4.0f;
      std::vector<mrpt::math::TPoint3Df> res;
      std::vector<float>                 dists;
      std::vector<uint64_t>              ids;
      map.nn_radius_search(q, radiusSqr, res, dists, ids, 0 /*maxPoints*/);

      std::vector<mrpt::math::TPoint3Df> gt_res;
      std::vector<float>                 gt_dists;
      std::vector<uint64_t>              gt_ids;
      ref->nn_radius_search(q, radiusSqr, gt_res, gt_dists, gt_ids, 0 /*maxPoints*/);

      ASSERT_EQUAL_(res.size(), gt_res.size());

      std::sort(dists.begin(), dists.end());
      std::sort(gt_dists.begin(), gt_dists.end());
      for (size_t k = 0; k < dists.size(); k++)
      {
        ASSERT_NEAR_(dists[k], gt_dists[k], 1e-4f);
      }
    }
  }
}

// -------------------------------------------------------------------------
void test_bounded_memory_under_churn()
{
  mola::IncrementalPointCloud map;
  map.creationOptions.remove_points_farther_than = 15.0;

  constexpr size_t nFrames     = 120;
  constexpr size_t ptsPerFrame = 500;

  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorPose = mrpt::poses::CPose3D::Identity();

  for (size_t f = 0; f < nFrames; f++)
  {
    // The sensor sweeps along +x, one meter per frame:
    const auto robotPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(static_cast<double>(f), 0, 0, 0, 0, 0);

    obs->pointcloud = randomCloud(ptsPerFrame, {0, 0, 0}, 5.0);

    map.insertObservation(*obs, robotPose);
  }

  const size_t totalInserted = nFrames * ptsPerFrame;

  std::cout << "[churn] storage=" << map.size() << " live=" << map.livePointCount()
            << " free slots=" << map.recyclableSlotCount() << " total inserted=" << totalInserted
            << "\n";

  ASSERT_(map.livePointCount() > 0);
  ASSERT_(map.livePointCount() <= map.size());

  // Slot recycling must keep the storage bounded by the sliding window instead
  // of growing with the total number of inserted points.
  ASSERT_LT_(map.size(), totalInserted / 2);

  // Every *live* point is still inside the keep cube around the last robot
  // pose (note map.boundingBox() is only a conservative superset, since the
  // not-yet-reclaimed tombstones also contribute to it).
  const auto bb = map.liveCompactedCopy()->boundingBox();
  ASSERT_GT_(bb.min.x, static_cast<float>(nFrames - 1) - 15.1f);
  ASSERT_LT_(bb.max.x, static_cast<float>(nFrames - 1) + 15.1f);

  // A generic copy through the inherited CPointsMap storage (this is how
  // mola_lidar_odometry copies map layers out for publishing) must NOT
  // resurrect evicted points: reclaimed slots are blanked, so it can only pick
  // up live points plus the bounded set of not-yet-reclaimed tombstones.
  {
    mola::IncrementalPointCloud rawCopy;
    rawCopy.insertAnotherMap(&map, mrpt::poses::CPose3D::Identity());

    std::cout << "[churn] raw insertAnotherMap copy=" << rawCopy.size() << " (live was "
              << map.livePointCount() << ")\n";

    ASSERT_GE_(rawCopy.size(), map.livePointCount());
    // Anything much beyond the live set would be stale geometry showing up in
    // the published/visualized map:
    ASSERT_LT_(rawCopy.size(), map.livePointCount() * 6 / 5);

    // Points tombstoned but not reclaimed yet do keep their coordinates, so
    // the copy may reach slightly past the keep cube. What it must never
    // contain is geometry from the beginning of the run: that would mean
    // reclaimed slots are still holding their evicted point.
    const auto rawBb = rawCopy.boundingBox();
    ASSERT_GT_(rawBb.min.x, static_cast<float>(nFrames) * 0.5f);
  }

  // ... and compacting must drop exactly the tombstoned slots:
  map.compact();
  ASSERT_EQUAL_(map.size(), map.livePointCount());
  ASSERT_EQUAL_(map.recyclableSlotCount(), 0UL);
}

// -------------------------------------------------------------------------
void test_serialization_and_copy()
{
  mola::IncrementalPointCloud map;
  map.creationOptions.remove_points_farther_than = 7.0;
  map.creationOptions.k_correspondences_for_cov  = 12;

  insertPoints(map, *randomCloud(3000, {0, 0, 0}, 20.0));
  map.keepOnlyPointsNear({0, 0, 0}, 8.0);

  const size_t nLive = map.livePointCount();
  ASSERT_(nLive > 0);
  ASSERT_(nLive < 3000);

  const auto lambdaSameContents = [&](const mola::IncrementalPointCloud& other)
  {
    ASSERT_EQUAL_(other.livePointCount(), nLive);
    // A deserialized/copied map is compacted, so there is nothing tombstoned:
    ASSERT_EQUAL_(other.size(), nLive);
    ASSERT_EQUAL_(other.creationOptions.k_correspondences_for_cov, 12U);
    ASSERT_NEAR_(other.creationOptions.remove_points_farther_than, 7.0, 1e-9);

    for (size_t trial = 0; trial < 100; trial++)
    {
      const mrpt::math::TPoint3Df q = {
          static_cast<float>(rng.drawUniform(-10.0, 10.0)),
          static_cast<float>(rng.drawUniform(-10.0, 10.0)),
          static_cast<float>(rng.drawUniform(-10.0, 10.0))};

      mrpt::math::TPoint3Df p1;
      mrpt::math::TPoint3Df p2;
      float                 d1 = 0;
      float                 d2 = 0;
      uint64_t              i1 = 0;
      uint64_t              i2 = 0;

      ASSERT_(map.nn_single_search(q, p1, d1, i1));
      ASSERT_(other.nn_single_search(q, p2, d2, i2));
      ASSERT_NEAR_(d1, d2, 1e-5f);
      ASSERT_NEAR_(p1.x, p2.x, 1e-5f);
      ASSERT_NEAR_(p1.y, p2.y, 1e-5f);
      ASSERT_NEAR_(p1.z, p2.z, 1e-5f);
    }
  };

  // Copy:
  {
    const mola::IncrementalPointCloud copy = map;
    lambdaSameContents(copy);
  }

  // Serialization round trip:
  {
    mrpt::io::CMemoryStream buf;
    auto                    arch = mrpt::serialization::archiveFrom(buf);
    arch << map;

    buf.Seek(0);
    mola::IncrementalPointCloud restored;
    arch >> restored;

    lambdaSameContents(restored);
  }
}

// -------------------------------------------------------------------------
void test_cov2cov()
{
  // A locally-planar surface, so the plane-regularized covariances are well
  // defined:
  auto surface = mrpt::maps::CSimplePointsMap::Create();
  for (int ix = -30; ix <= 30; ix++)
  {
    for (int iy = -30; iy <= 30; iy++)
    {
      surface->insertPointFast(
          static_cast<float>(ix * 0.2), static_cast<float>(iy * 0.2),
          static_cast<float>(rng.drawGaussian1D(0, 0.005)));
    }
  }
  surface->mark_as_modified();

  mola::IncrementalPointCloud global;
  insertPoints(global, *surface);

  // The local map holds the very same points, but expressed in a frame such
  // that composing them with `localPose` lands exactly on the global ones.
  const auto localPose =
      mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, -0.5, 0.2, 0.15, 0.02, -0.03);
  const auto localPoseInv = mrpt::poses::CPose3D::Identity() - localPose;

  mola::IncrementalPointCloud local;
  for (size_t i = 0; i < surface->size(); i++)
  {
    float x = 0;
    float y = 0;
    float z = 0;
    surface->getPointFast(i, x, y, z);
    const auto pl = localPoseInv.composePoint(mrpt::math::TPoint3D(x, y, z));
    local.insertPoint(static_cast<float>(pl.x), static_cast<float>(pl.y), static_cast<float>(pl.z));
  }

  ASSERT_EQUAL_(global.point_count(), surface->size());
  ASSERT_EQUAL_(local.point_count(), surface->size());

  mp2p_icp::MatchedPointWithCovList pairings;
  global.nn_search_cov2cov(local, localPose, 0.5f /*max search distance*/, pairings);

  ASSERT_EQUAL_(pairings.size(), surface->size());

  for (const auto& p : pairings)
  {
    // Each local point, once composed, must land on its own global twin:
    const auto lg = localPose.composePoint(mrpt::math::TPoint3D(p.local.x, p.local.y, p.local.z));
    ASSERT_NEAR_(lg.x, p.global.x, 1e-4);
    ASSERT_NEAR_(lg.y, p.global.y, 1e-4);
    ASSERT_NEAR_(lg.z, p.global.z, 1e-4);

    // The GICP weight must be a usable, finite matrix:
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        ASSERT_(std::isfinite(p.cov_inv(r, c)));
      }
      ASSERT_GT_(p.cov_inv(r, r), .0f);
    }
  }
}

// -------------------------------------------------------------------------
// Pairings must come back in a canonical order, every run. The parallel path
// accumulates into thread-local vectors and merges them in an unspecified
// order, and the snapshot of live local points is in k-d tree traversal order
// rather than in slot order, so neither path was canonical on its own. The
// resulting permutation reaches the solver's summation order and therefore the
// optimized pose. Ascending local_idx is asserted because it is a total order
// on a unique key, which also pins the parallel and sequential paths together.
void test_pairing_order_is_canonical()
{
  // Big enough that TBB really splits the range across workers: on a small
  // cloud the defect this guards against does not show up at all.
  auto surface = mrpt::maps::CSimplePointsMap::Create();
  for (int ix = -100; ix <= 100; ix++)
  {
    for (int iy = -100; iy <= 100; iy++)
    {
      surface->insertPointFast(
          static_cast<float>(ix * 0.1), static_cast<float>(iy * 0.1),
          static_cast<float>(rng.drawGaussian1D(0, 0.005)));
    }
  }
  surface->mark_as_modified();

  mola::IncrementalPointCloud global;
  insertPoints(global, *surface);

  mola::IncrementalPointCloud local;
  insertPoints(local, *surface);

  // Tombstones make the tree's traversal order diverge from slot order, which
  // is the case the sort has to survive:
  local.keepOnlyPointsNear({0, 0, 0}, 8.0);

  std::vector<uint32_t> reference;
  for (int pass = 0; pass < 4; pass++)
  {
    mp2p_icp::MatchedPointWithCovList pairings;
    global.nn_search_cov2cov(
        local, mrpt::poses::CPose3D::Identity(), 0.5f /*max search distance*/, pairings);
    ASSERT_(!pairings.empty());

    std::vector<uint32_t> order;
    order.reserve(pairings.size());
    for (const auto& p : pairings) order.push_back(static_cast<uint32_t>(p.local_idx));

    ASSERTMSG_(
        std::is_sorted(order.begin(), order.end()),
        "Pairings are not in canonical (ascending local_idx) order");

    if (pass == 0)
      reference = order;
    else
      ASSERTMSG_(order == reference, "Pairing order differs between identical calls");
  }
}

// -------------------------------------------------------------------------
// Compares NN query results between two maps expected to hold the same set of
// points (used by the k-d tree baking tests below).
void assertSameNNResults(
    const mola::IncrementalPointCloud& a, const mola::IncrementalPointCloud& b, size_t nTrials,
    double halfRange)
{
  ASSERT_EQUAL_(a.livePointCount(), b.livePointCount());
  for (size_t trial = 0; trial < nTrials; trial++)
  {
    const mrpt::math::TPoint3Df q = {
        static_cast<float>(rng.drawUniform(-halfRange, halfRange)),
        static_cast<float>(rng.drawUniform(-halfRange, halfRange)),
        static_cast<float>(rng.drawUniform(-halfRange, halfRange))};

    mrpt::math::TPoint3Df pa;
    mrpt::math::TPoint3Df pb;
    float                 da  = 0;
    float                 db  = 0;
    uint64_t              ia  = 0;
    uint64_t              ib  = 0;
    const bool            oka = a.nn_single_search(q, pa, da, ia);
    const bool            okb = b.nn_single_search(q, pb, db, ib);
    ASSERT_EQUAL_(oka, okb);
    if (!oka) continue;
    ASSERT_NEAR_(da, db, 1e-4f);
    ASSERT_NEAR_(pa.x, pb.x, 1e-4f);
    ASSERT_NEAR_(pa.y, pb.y, 1e-4f);
    ASSERT_NEAR_(pa.z, pb.z, 1e-4f);
  }
}

// -------------------------------------------------------------------------
// Baking the k-d tree (creationOptions.serialize_kdtree=true) must produce a
// map that, once (de)serialized through memory, answers NN queries exactly
// like a map serialized without baking. This must hold whether or not this
// build was compiled against a nanoflann providing the incremental index's
// saveIndex()/loadIndex() (MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE): the
// option is a documented no-op when unavailable, not a correctness hazard.
void test_kdtree_bake_roundtrip_memory()
{
  mola::IncrementalPointCloud map;
  map.creationOptions.serialize_kdtree = true;

  insertPoints(map, *randomCloud(4000, {0, 0, 0}, 15.0));
  map.keepOnlyPointsNear({0, 0, 0}, 9.0);  // introduce tombstones before baking
  ASSERT_(map.livePointCount() > 0);
  ASSERT_(map.livePointCount() < 4000);

  mrpt::io::CMemoryStream buf;
  auto                    arch = mrpt::serialization::archiveFrom(buf);
  arch << map;

  buf.Seek(0);
  mola::IncrementalPointCloud loaded;
  arch >> loaded;

  ASSERT_EQUAL_(loaded.livePointCount(), map.livePointCount());
  ASSERT_EQUAL_(loaded.size(), loaded.livePointCount());  // deserialized: always compacted
  ASSERT_(loaded.creationOptions.serialize_kdtree);

  assertSameNNResults(map, loaded, 200, 15.0);

#if defined(MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE)
  // Positive check that baking actually wrote something: the same map
  // serialized without the option must yield a strictly smaller stream.
  {
    mola::IncrementalPointCloud noBake      = map;
    noBake.creationOptions.serialize_kdtree = false;

    mrpt::io::CMemoryStream bufNoBake;
    auto                    archNoBake = mrpt::serialization::archiveFrom(bufNoBake);
    archNoBake << noBake;

    ASSERT_GT_(buf.getTotalBytesCount(), bufNoBake.getTotalBytesCount());
  }
#endif
}

// -------------------------------------------------------------------------
// Same as above, but through a real temporary file on disk (as the
// mm-ipc-bake-kdtree CLI tool does), and with the k-d tree parameters
// intentionally different between the baking map and the loaded one
// (async_rebuild toggled): a baked index must be usable under either live
// k-d tree configuration.
void test_kdtree_bake_roundtrip_file()
{
  mola::IncrementalPointCloud map;
  map.creationOptions.serialize_kdtree = true;
  map.creationOptions.async_rebuild    = false;

  insertPoints(map, *randomCloud(3000, {2, -1, 0}, 12.0));

  const std::string tmpFile = mrpt::system::getTempFileName();
  {
    mrpt::io::CFileOutputStream fo(tmpFile);
    auto                        arch = mrpt::serialization::archiveFrom(fo);
    arch << map;
  }

  mola::IncrementalPointCloud loaded;
  loaded.creationOptions.async_rebuild = true;  // deliberately different from `map`
  {
    mrpt::io::CFileInputStream fi(tmpFile);
    auto                       arch = mrpt::serialization::archiveFrom(fi);
    arch >> loaded;
  }
  mrpt::system::deleteFile(tmpFile);

  ASSERT_EQUAL_(loaded.livePointCount(), map.livePointCount());
  assertSameNNResults(map, loaded, 200, 12.0);
}

// -------------------------------------------------------------------------
// A map loaded from a baked file must behave like any other map afterwards:
// clear() must reset it to empty and usable, and new insertions must be
// correctly reflected by subsequent queries (the k-d tree must not be left
// frozen in the state it was baked in).
void test_kdtree_bake_then_clear_and_reinsert()
{
  mola::IncrementalPointCloud map;
  map.creationOptions.serialize_kdtree = true;
  insertPoints(map, *randomCloud(1500, {0, 0, 0}, 10.0));

  mrpt::io::CMemoryStream buf;
  {
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch << map;
  }
  buf.Seek(0);
  mola::IncrementalPointCloud loaded;
  {
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch >> loaded;
  }
  ASSERT_(loaded.livePointCount() > 0);

  // Clearing a just-loaded (possibly baked-index) map must leave it empty and
  // in a state where the index can be rebuilt from scratch:
  loaded.clear();
  ASSERT_(loaded.isEmpty());
  ASSERT_EQUAL_(loaded.livePointCount(), 0UL);
  {
    mrpt::math::TPoint3Df p;
    float                 d  = 0;
    uint64_t              id = 0;
    ASSERT_(!loaded.nn_single_search({0, 0, 0}, p, d, id));
  }

  // Re-populating it afterwards must work exactly like a fresh map:
  insertPoints(loaded, *randomCloud(1200, {3, 3, 3}, 8.0));
  const auto ref = bruteForceReference(loaded);
  ASSERT_EQUAL_(loaded.livePointCount(), ref->size());

  for (size_t trial = 0; trial < 100; trial++)
  {
    const mrpt::math::TPoint3Df q = {
        static_cast<float>(rng.drawUniform(-5.0, 11.0)),
        static_cast<float>(rng.drawUniform(-5.0, 11.0)),
        static_cast<float>(rng.drawUniform(-5.0, 11.0))};
    mrpt::math::TPoint3Df p;
    mrpt::math::TPoint3Df gt_p;
    float                 d     = 0;
    float                 gt_d  = 0;
    uint64_t              id    = 0;
    uint64_t              gt_id = 0;
    const bool            ok    = loaded.nn_single_search(q, p, d, id);
    const bool            gt_ok = ref->nn_single_search(q, gt_p, gt_d, gt_id);
    ASSERT_EQUAL_(ok, gt_ok);
    if (ok) ASSERT_NEAR_(d, gt_d, 1e-4f);
  }
}

// -------------------------------------------------------------------------
// A map loaded from a baked file must also correctly absorb further
// modifications (insertions AND a box trim), not just fresh re-insertion
// after clear(): the loaded index must be a fully-functional, live index.
void test_kdtree_bake_then_modify()
{
  mola::IncrementalPointCloud map;
  map.creationOptions.serialize_kdtree = true;
  insertPoints(map, *randomCloud(2500, {0, 0, 0}, 12.0));

  mrpt::io::CMemoryStream buf;
  {
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch << map;
  }
  buf.Seek(0);
  mola::IncrementalPointCloud loaded;
  {
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch >> loaded;
  }
  const size_t nBaked = loaded.livePointCount();
  ASSERT_(nBaked > 0);

  // Insert more points on top of the baked index:
  insertPoints(loaded, *randomCloud(800, {6, 6, 0}, 5.0));
  ASSERT_EQUAL_(loaded.livePointCount(), nBaked + 800);

  // ... and trim a region, exercising removal on a baked-then-grown index:
  loaded.keepOnlyPointsNear({0, 0, 0}, 7.0);
  ASSERT_(loaded.livePointCount() > 0);
  ASSERT_(loaded.livePointCount() < nBaked + 800);

  const auto ref = bruteForceReference(loaded);
  ASSERT_EQUAL_(loaded.livePointCount(), ref->size());

  for (size_t trial = 0; trial < 150; trial++)
  {
    const mrpt::math::TPoint3Df q = {
        static_cast<float>(rng.drawUniform(-10.0, 10.0)),
        static_cast<float>(rng.drawUniform(-10.0, 10.0)),
        static_cast<float>(rng.drawUniform(-10.0, 10.0))};
    mrpt::math::TPoint3Df p;
    mrpt::math::TPoint3Df gt_p;
    float                 d     = 0;
    float                 gt_d  = 0;
    uint64_t              id    = 0;
    uint64_t              gt_id = 0;
    const bool            ok    = loaded.nn_single_search(q, p, d, id);
    const bool            gt_ok = ref->nn_single_search(q, gt_p, gt_d, gt_id);
    ASSERT_EQUAL_(ok, gt_ok);
    if (ok) ASSERT_NEAR_(d, gt_d, 1e-4f);
  }
}

// -------------------------------------------------------------------------
// serialize_kdtree=false (the default) must remain unaffected: no blob is
// written, and the loaded map is a plain, freshly-rebuilt index.
void test_kdtree_bake_disabled_by_default()
{
  mola::IncrementalPointCloud map;
  ASSERT_(!map.creationOptions.serialize_kdtree);
  insertPoints(map, *randomCloud(1000, {0, 0, 0}, 10.0));

  mrpt::io::CMemoryStream buf;
  {
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch << map;
  }
  buf.Seek(0);
  mola::IncrementalPointCloud loaded;
  {
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch >> loaded;
  }
  ASSERT_(!loaded.creationOptions.serialize_kdtree);
  assertSameNNResults(map, loaded, 100, 10.0);
}

// -------------------------------------------------------------------------
/// Compares NN queries of `map` against an independent, static-k-d-tree oracle.
void assertMatchesReference(
    const mola::IncrementalPointCloud& map, const mrpt::maps::CSimplePointsMap& ref, size_t nTrials,
    const mrpt::math::TPoint3Df& center, double halfRange)
{
  ASSERT_EQUAL_(map.livePointCount(), ref.size());

  for (size_t trial = 0; trial < nTrials; trial++)
  {
    const mrpt::math::TPoint3Df q = {
        static_cast<float>(center.x + rng.drawUniform(-halfRange, halfRange)),
        static_cast<float>(center.y + rng.drawUniform(-halfRange, halfRange)),
        static_cast<float>(center.z + rng.drawUniform(-halfRange, halfRange))};

    mrpt::math::TPoint3Df p;
    mrpt::math::TPoint3Df gt_p;
    float                 d     = 0;
    float                 gt_d  = 0;
    uint64_t              id    = 0;
    uint64_t              gt_id = 0;

    const bool ok    = map.nn_single_search(q, p, d, id);
    const bool gt_ok = ref.nn_single_search(q, gt_p, gt_d, gt_id);

    ASSERT_EQUAL_(ok, gt_ok);
    if (!ok) continue;

    ASSERT_NEAR_(d, gt_d, 1e-3f);
    ASSERT_NEAR_(p.x, gt_p.x, 1e-3f);
    ASSERT_NEAR_(p.y, gt_p.y, 1e-3f);
    ASSERT_NEAR_(p.z, gt_p.z, 1e-3f);
  }
}

// -------------------------------------------------------------------------
// A global SE(3) re-map must move every point and leave the index consistent
// with the new coordinates. Since CPointsMap::changeCoordinatesReference() is
// not virtual, this is checked both on the concrete type (intercepted right
// away) and through a base-class pointer (only detectable on the next query).
void test_change_coordinates_reference(bool viaBasePointer, bool async)
{
  mola::IncrementalPointCloud map;
  map.creationOptions.async_rebuild = async;
  map.compact();  // apply the structural option above

  insertPoints(map, *randomCloud(3000, {0, 0, 0}, 20.0));

  // Leave tombstoned and recycled slots behind, so the rebuild has to preserve
  // the live set instead of blindly re-indexing the whole storage:
  map.keepOnlyPointsNear({5.0f, 0.0f, 0.0f}, 10.0);
  insertPoints(map, *randomCloud(1500, {5, 0, 0}, 8.0));

  const size_t liveBefore = map.livePointCount();
  ASSERT_(liveBefore > 100);

  const mrpt::poses::CPose3D T(1.0, -2.0, 0.5, 0.3, 0.15, -0.2);

  // Oracle: the very same live points, transformed with plain MRPT:
  const auto ref = bruteForceReference(map);
  ref->changeCoordinatesReference(T);

  if (viaBasePointer)
  {
    auto* asBase = static_cast<mrpt::maps::CPointsMap*>(&map);
    asBase->changeCoordinatesReference(T);
  }
  else
  {
    map.changeCoordinatesReference(T);
  }

  ASSERT_EQUAL_(map.livePointCount(), liveBefore);
  assertMatchesReference(map, *ref, 300, {6.0f, -2.0f, 0.5f}, 20.0);

  // The map must stay usable afterwards: insertion, slot recycling and trimming
  // all have to keep working on the rebuilt index.
  insertPoints(map, *randomCloud(1000, {6, -2, 0}, 6.0));
  map.keepOnlyPointsNear({6.0f, -2.0f, 0.0f}, 9.0);
  ASSERT_(map.livePointCount() > 0);

  assertMatchesReference(map, *bruteForceReference(map), 200, {6.0f, -2.0f, 0.0f}, 12.0);
}

// -------------------------------------------------------------------------
// The other two changeCoordinatesReference() overloads must leave the map in
// the same state as the CPose3D one they delegate to.
void test_change_coordinates_reference_overloads()
{
  const auto cloud = randomCloud(2000, {0, 0, 0}, 15.0);

  // a) The CPose2D overload:
  {
    mola::IncrementalPointCloud map;
    insertPoints(map, *cloud);
    map.keepOnlyPointsNear({3.0f, 0.0f, 0.0f}, 9.0);  // leave dead slots behind

    const mrpt::poses::CPose2D p2d(2.0, -1.0, 0.7);

    const auto ref = bruteForceReference(map);
    ref->changeCoordinatesReference(mrpt::poses::CPose3D(p2d));

    map.changeCoordinatesReference(p2d);
    assertMatchesReference(map, *ref, 200, {5.0f, -1.0f, 0.0f}, 15.0);
  }

  // b) The (other, pose) overload: our previous contents must be replaced by
  //    `other`'s, transformed.
  {
    mola::IncrementalPointCloud map;
    insertPoints(map, *randomCloud(500, {50, 50, 50}, 5.0));  // to be discarded

    const mrpt::poses::CPose3D T(-3.0, 4.0, 1.0, -0.4, 0.2, 0.1);

    const auto ref = mrpt::maps::CSimplePointsMap::Create();
    ref->changeCoordinatesReference(*cloud, T);

    map.changeCoordinatesReference(*cloud, T);
    assertMatchesReference(map, *ref, 200, {-3.0f, 4.0f, 1.0f}, 20.0);
  }
}

// -------------------------------------------------------------------------
// View-direction fields arrive in the sensor frame of each insertion. The map
// must keep them in its own frame, turn them with the coordinates, and use them
// to reject cov-to-cov pairs seen from opposite sides of a surface.

/// A 4x4 m planar patch as a point cloud observation, every point carrying the
/// same view vector (toward the sensor), in the cloud's frame: a wall on the
/// plane x=0, or ground on the plane z=0.
mrpt::obs::CObservationPointCloud surfaceObservation(
    const mrpt::math::TVector3D& view, bool ground = false)
{
  auto pc = mrpt::maps::CGenericPointsMap::Create();
  pc->registerField_float("view_x");
  pc->registerField_float("view_y");
  pc->registerField_float("view_z");
  for (int i = -20; i <= 20; i++)
  {
    for (int j = -20; j <= 20; j++)
    {
      const auto a = static_cast<float>(i * 0.1);
      const auto b = static_cast<float>(j * 0.1);
      if (ground)
      {
        pc->insertPointFast(a, b, 0.0f);
      }
      else
      {
        pc->insertPointFast(0.0f, a, b);
      }
      pc->insertPointField_float("view_x", static_cast<float>(view.x));
      pc->insertPointField_float("view_y", static_cast<float>(view.y));
      pc->insertPointField_float("view_z", static_cast<float>(view.z));
    }
  }
  pc->mark_as_modified();

  mrpt::obs::CObservationPointCloud obs;
  obs.pointcloud = pc;
  return obs;
}

mrpt::obs::CObservationPointCloud wallObservation(const mrpt::math::TVector3D& view)
{
  return surfaceObservation(view);
}

constexpr size_t WALL_POINTS = 41 * 41;

/// The view vector stored in storage slot `i`.
mrpt::math::TVector3D storedView(const mola::IncrementalPointCloud& map, size_t i)
{
  return {
      map.getPointField_float(i, "view_x"), map.getPointField_float(i, "view_y"),
      map.getPointField_float(i, "view_z")};
}

void assertSameVector(const mrpt::math::TVector3D& a, const mrpt::math::TVector3D& b)
{
  ASSERT_NEAR_(a.x, b.x, 1e-5);
  ASSERT_NEAR_(a.y, b.y, 1e-5);
  ASSERT_NEAR_(a.z, b.z, 1e-5);
}

void test_view_fields_in_map_frame()
{
  // Inserted with a robot pose, the stored views must be rotated like the points:
  const mrpt::poses::CPose3D  robotPose(1.0, 2.0, 0.3, mrpt::DEG2RAD(90.0), 0, 0);
  const mrpt::math::TVector3D viewInSensor(1.0, 0.0, 0.0);

  mola::IncrementalPointCloud map;
  map.insertObservation(wallObservation(viewInSensor), robotPose);
  ASSERT_(map.hasPointField("view_x"));
  ASSERT_EQUAL_(map.size(), WALL_POINTS);

  const auto expected = robotPose.rotateVector(viewInSensor);  // (0, 1, 0)
  for (size_t i = 0; i < map.size(); i++)
  {
    assertSameVector(storedView(map, i), expected);
  }

  // A global re-map turns them again:
  const mrpt::poses::CPose3D T(-2.0, 0.5, 0.0, 0.3, 0.2, -0.1);
  map.changeCoordinatesReference(T);
  const auto expectedAfter = T.rotateVector(expected);
  for (size_t i = 0; i < map.size(); i++)
  {
    assertSameVector(storedView(map, i), expectedAfter);
  }

  // And they survive a serialization round trip, still in the map frame:
  mrpt::io::CMemoryStream buf;
  auto                    arch = mrpt::serialization::archiveFrom(buf);
  arch << map;
  buf.Seek(0);
  mola::IncrementalPointCloud reloaded;
  arch >> reloaded;
  ASSERT_(reloaded.hasPointField("view_x"));
  ASSERT_EQUAL_(reloaded.size(), WALL_POINTS);
  assertSameVector(storedView(reloaded, 0), expectedAfter);
}

// Points appended into recycled storage slots are moved there by the map, and
// their view directions must still end up rotated into the map frame.
void test_view_fields_in_recycled_slots()
{
  const mrpt::math::TVector3D viewInSensor(1.0, 0.0, 0.0);

  mola::IncrementalPointCloud map;
  map.insertObservation(
      wallObservation(viewInSensor), mrpt::poses::CPose3D(0, 0, 0, mrpt::DEG2RAD(30.0), 0, 0));

  // Evict everything, so the next insertion reuses the freed slots:
  map.keepOnlyPointsNear({100.0f, 100.0f, 100.0f}, 1.0);
  ASSERT_EQUAL_(map.livePointCount(), 0U);
  ASSERT_(map.recyclableSlotCount() > 0);

  const mrpt::poses::CPose3D robotPose(-1.0, 0.5, 0.2, mrpt::DEG2RAD(-70.0), 0.1, 0.05);
  map.insertObservation(wallObservation(viewInSensor), robotPose);
  ASSERT_EQUAL_(map.livePointCount(), WALL_POINTS);

  const auto expected = robotPose.rotateVector(viewInSensor);
  const auto live     = map.liveCompactedCopy();
  for (size_t i = 0; i < live->size(); i++)
  {
    assertSameVector(
        {live->getPointField_float(i, "view_x"), live->getPointField_float(i, "view_y"),
         live->getPointField_float(i, "view_z")},
        expected);
  }
}

struct ViewFilterSetup
{
  mola::ViewDirectionFilter mode        = mola::ViewDirectionFilter::MaxAngle;
  double                    maxAngleDeg = 120.0;
  bool                      enabled     = true;  // the master switch
};

/// Number of cov-to-cov pairings between `global` and its surface observed
/// with `queryViewInMap` (in the MAP frame). The query cloud is expressed in its
/// own frame, which `queryPose` maps onto the map.
size_t queryPairings(
    const mola::IncrementalPointCloud& global, const mrpt::math::TVector3D& queryViewInMap,
    bool ground)
{
  // Build the query cloud in its own frame by inserting the wall with the
  // inverse of the pose later given to the search. Insertion turns the view
  // vectors along with the points, so the search has to turn them back.
  const mrpt::poses::CPose3D queryPose(0.2, -0.1, 0.05, mrpt::DEG2RAD(160.0), 0.1, -0.05);
  const auto                 queryPoseInv = mrpt::poses::CPose3D::Identity() - queryPose;

  mola::IncrementalPointCloud local;
  local.insertObservation(surfaceObservation(queryViewInMap, ground), queryPoseInv);

  mp2p_icp::MatchedPointWithCovList pairings;
  global.nn_search_cov2cov(local, queryPose, 0.5f /*max search distance*/, pairings);
  return pairings.size();
}

/// Number of cov-to-cov pairings between a surface mapped as seen with
/// `mapView` and the same surface observed with `queryViewInMap` (both in the
/// MAP frame).
size_t surfacePairings(
    const mrpt::math::TVector3D& mapView, const mrpt::math::TVector3D& queryViewInMap, bool ground,
    const ViewFilterSetup& setup)
{
  mola::IncrementalPointCloud global;
  global.creationOptions.view_direction_filter     = setup.mode;
  global.creationOptions.max_view_angle_deg        = setup.maxAngleDeg;
  global.creationOptions.use_view_direction_filter = setup.enabled;
  global.insertObservation(surfaceObservation(mapView, ground), mrpt::poses::CPose3D::Identity());

  return queryPairings(global, queryViewInMap, ground);
}

/// A wall mapped as seen from +x, observed with `queryViewInMap`.
size_t wallPairings(const mrpt::math::TVector3D& queryViewInMap, const ViewFilterSetup& setup)
{
  return surfacePairings({1.0, 0.0, 0.0}, queryViewInMap, false /*wall*/, setup);
}

void test_view_direction_filter()
{
  const ViewFilterSetup angle;  // the keyframe map's test, 120 deg
  ViewFilterSetup       off;
  off.mode = mola::ViewDirectionFilter::None;
  ViewFilterSetup open180;
  open180.maxAngleDeg = 180.0;

  // Same side: every point pairs.
  ASSERT_EQUAL_(wallPairings({1.0, 0.0, 0.0}, angle), WALL_POINTS);

  // 60 deg apart, inside the default 120 deg: still paired.
  const double a60 = mrpt::DEG2RAD(60.0);
  ASSERT_EQUAL_(wallPairings({std::cos(a60), std::sin(a60), 0.0}, angle), WALL_POINTS);

  // Opposite side: every pair is rejected...
  ASSERT_EQUAL_(wallPairings({-1.0, 0.0, 0.0}, angle), 0U);

  // ...unless the filter is off, or its threshold opened to 180 deg:
  ASSERT_EQUAL_(wallPairings({-1.0, 0.0, 0.0}, off), WALL_POINTS);
  ASSERT_EQUAL_(wallPairings({-1.0, 0.0, 0.0}, open180), WALL_POINTS);
}

// The surface-side test must reject the two faces of a wall like the angle
// test does, but keep the same surface seen from very different directions on
// the same side, which the angle test rejects: ground observed from opposite
// azimuths, at a low elevation.
void test_view_filter_surface_side()
{
  ViewFilterSetup side;
  side.mode = mola::ViewDirectionFilter::SurfaceSide;
  const ViewFilterSetup angle;

  // The two faces of a wall: rejected by both tests.
  ASSERT_EQUAL_(wallPairings({-1.0, 0.0, 0.0}, side), 0U);

  // ...unless the master switch is off:
  ViewFilterSetup sideDisabled = side;
  sideDisabled.enabled         = false;
  ASSERT_EQUAL_(wallPairings({-1.0, 0.0, 0.0}, sideDisabled), WALL_POINTS);
  ASSERT_EQUAL_(wallPairings({1.0, 0.0, 0.0}, side), WALL_POINTS);

  // Ground seen at 20 deg of elevation from +x and from -x: 140 deg apart.
  const double                elev = mrpt::DEG2RAD(20.0);
  const mrpt::math::TVector3D fromPlusX(std::cos(elev), 0.0, std::sin(elev));
  const mrpt::math::TVector3D fromMinusX(-std::cos(elev), 0.0, std::sin(elev));

  ASSERT_EQUAL_(surfacePairings(fromPlusX, fromMinusX, true /*ground*/, angle), 0U);
  ASSERT_EQUAL_(surfacePairings(fromPlusX, fromMinusX, true /*ground*/, side), WALL_POINTS);

  // Views near grazing (under ~14.5 deg from the plane) are never judged, from
  // either side:
  for (const double deg : {3.0, 10.0})
  {
    const double                grazing = mrpt::DEG2RAD(deg);
    const mrpt::math::TVector3D below(-std::cos(grazing), 0.0, -std::sin(grazing));
    ASSERT_EQUAL_(surfacePairings(fromPlusX, below, true /*ground*/, side), WALL_POINTS);
  }

  // A cloud without a flat neighborhood (e.g. foliage) seen from opposite
  // sides: its regularized covariance still looks like a plane, but the side
  // test must not trust that normal.
  const auto lambdaBlob = [](const mrpt::math::TVector3D& view)
  {
    mrpt::random::CRandomGenerator blobRng(4321U);
    auto                           pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float("view_x");
    pc->registerField_float("view_y");
    pc->registerField_float("view_z");
    for (size_t i = 0; i < WALL_POINTS; i++)
    {
      pc->insertPointFast(
          static_cast<float>(blobRng.drawUniform(-1.0, 1.0)),
          static_cast<float>(blobRng.drawUniform(-1.0, 1.0)),
          static_cast<float>(blobRng.drawUniform(-1.0, 1.0)));
      pc->insertPointField_float("view_x", static_cast<float>(view.x));
      pc->insertPointField_float("view_y", static_cast<float>(view.y));
      pc->insertPointField_float("view_z", static_cast<float>(view.z));
    }
    pc->mark_as_modified();
    mrpt::obs::CObservationPointCloud obs;
    obs.pointcloud = pc;
    return obs;
  };
  for (const auto& [setup, expected] :
       {std::make_pair(side, WALL_POINTS), std::make_pair(angle, size_t(0))})
  {
    mola::IncrementalPointCloud global;
    global.creationOptions.view_direction_filter = setup.mode;
    global.insertObservation(lambdaBlob({1.0, 0.0, 0.0}), mrpt::poses::CPose3D::Identity());
    mola::IncrementalPointCloud local;
    local.insertObservation(lambdaBlob({-1.0, 0.0, 0.0}), mrpt::poses::CPose3D::Identity());

    mp2p_icp::MatchedPointWithCovList pairings;
    global.nn_search_cov2cov(local, mrpt::poses::CPose3D::Identity(), 0.5f, pairings);
    ASSERT_EQUAL_(pairings.size(), expected);
  }
}

// The filter is on by default (SurfaceSide): a default map does not pair the two
// faces of a wall, and the option loads from its enum name, as a pipeline would.
void test_view_filter_default_and_config()
{
  {
    mola::IncrementalPointCloud global;
    ASSERT_(global.creationOptions.view_direction_filter == mola::ViewDirectionFilter::SurfaceSide);
  }
  ViewFilterSetup defaults;
  defaults.mode = mola::IncrementalPointCloud::TCreationOptions().view_direction_filter;
  ASSERT_EQUAL_(wallPairings({-1.0, 0.0, 0.0}, defaults), 0U);  // the two faces of a wall
  ASSERT_EQUAL_(wallPairings({1.0, 0.0, 0.0}, defaults), WALL_POINTS);

  mrpt::config::CConfigFileMemory cfg;
  cfg.write("opts", "view_direction_filter", "ViewDirectionFilter::SurfaceSide");
  cfg.write("opts", "use_view_direction_filter", false);
  cfg.write("opts", "max_view_angle_deg", 95.0);
  mola::IncrementalPointCloud::TCreationOptions opts;
  opts.loadFromConfigFile(cfg, "opts");
  ASSERT_(opts.view_direction_filter == mola::ViewDirectionFilter::SurfaceSide);
  ASSERT_(!opts.use_view_direction_filter);

  // An empty value, as a pipeline leaving the mode unset writes, keeps the default:
  mrpt::config::CConfigFileMemory cfgEmpty;
  cfgEmpty.write("opts", "view_direction_filter", "");
  mola::IncrementalPointCloud::TCreationOptions optsEmpty;
  optsEmpty.loadFromConfigFile(cfgEmpty, "opts");
  ASSERT_(optsEmpty.view_direction_filter == mola::ViewDirectionFilter::SurfaceSide);
  ASSERT_NEAR_(opts.max_view_angle_deg, 95.0, 1e-9);
}

// A re-map through the non-virtual base-class method moves the points without
// telling the map by how much, so their view directions cannot be turned: they
// must be dropped, and the filter must then stop rejecting pairs.
void test_view_fields_cleared_on_base_class_remap()
{
  const ViewFilterSetup       angle;
  mola::IncrementalPointCloud global;
  global.creationOptions.view_direction_filter = angle.mode;
  global.insertObservation(wallObservation({1.0, 0.0, 0.0}), mrpt::poses::CPose3D::Identity());
  ASSERT_EQUAL_(queryPairings(global, {-1.0, 0.0, 0.0}, false /*wall*/), 0U);

  // Out through the base class, back through the map's own method: the
  // geometry is restored, but the first move could not turn the directions.
  const mrpt::poses::CPose3D T(1.0, -2.0, 0.5, 0.3, 0.15, -0.2);
  static_cast<mrpt::maps::CPointsMap&>(global).changeCoordinatesReference(T);
  global.changeCoordinatesReference(mrpt::poses::CPose3D::Identity() - T);

  ASSERT_(global.hasPointField("view_x"));
  for (size_t i = 0; i < global.size(); i++)
  {
    assertSameVector(storedView(global, i), {0.0, 0.0, 0.0});
  }
  ASSERT_EQUAL_(queryPairings(global, {-1.0, 0.0, 0.0}, false /*wall*/), WALL_POINTS);
}

// A base-class re-map followed by an append before the next query: the rewrite
// must still be noticed, so that both the moved and the appended points are
// indexed at their current coordinates, and the view directions (now in an
// unknown frame) are dropped.
void test_base_class_remap_then_append()
{
  mola::IncrementalPointCloud map;
  map.insertObservation(wallObservation({1.0, 0.0, 0.0}), mrpt::poses::CPose3D::Identity());

  // Leave dead slots behind, so that the rebuild has to preserve them:
  map.keepOnlyPointsNear({0.0f, 0.0f, 0.0f}, 1.05);
  ASSERT_(map.livePointCount() < WALL_POINTS);

  const mrpt::poses::CPose3D T(1.0, -2.0, 0.5, 0.3, 0.15, -0.2);

  const auto ref = bruteForceReference(map);
  ref->changeCoordinatesReference(T);

  static_cast<mrpt::maps::CPointsMap&>(map).changeCoordinatesReference(T);

  // Appended through the inherited API, before any query:
  for (int i = 0; i < 50; i++)
  {
    const auto p = T.composePoint(mrpt::math::TPoint3D(
        rng.drawUniform(-1.0, 1.0), rng.drawUniform(-1.0, 1.0), rng.drawUniform(0.5, 1.5)));
    map.insertPoint(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    ref->insertPoint(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
  }
  ref->mark_as_modified();

  assertMatchesReference(map, *ref, 300, T.translation().cast<float>(), 2.5);

  for (const char* f : {"view_x", "view_y", "view_z"})
  {
    const auto* v = map.getPointsBufferRef_float_field(f);
    ASSERT_(v != nullptr);
    for (const float x : *v)
    {
      ASSERT_EQUAL_(x, 0.0f);
    }
  }
}

// A corrupted filter mode in a stream must be rejected, not read as a mode that
// no filter branch recognizes.
void test_invalid_view_filter_mode_rejected()
{
  mrpt::io::CMemoryStream buf;
  auto                    arch = mrpt::serialization::archiveFrom(buf);
  mola::IncrementalPointCloud::TCreationOptions().writeToStream(arch);

  // The mode byte is the last field but one, the angle (a double):
  const auto modeOffset = buf.getTotalBytesCount() - sizeof(double) - 1;
  static_cast<uint8_t*>(buf.getRawBufferData())[modeOffset] = 3;
  buf.Seek(0);

  bool rejected = false;
  try
  {
    mola::IncrementalPointCloud::TCreationOptions loaded;
    loaded.readFromStream(arch);
  }
  catch (const std::exception&)
  {
    rejected = true;
  }
  ASSERT_(rejected);
}

/// Gives the tests access to the (protected) versioned serialization entry points.
struct SerializationProbe : public mola::IncrementalPointCloud
{
  using mola::IncrementalPointCloud::serializeFrom;
  using mola::IncrementalPointCloud::serializeTo;
};

// A map saved before view directions were kept in the map frame cannot tell
// which frame its stored ones are in, so loading it must drop them.
void test_legacy_view_fields_dropped_on_load()
{
  SerializationProbe map;
  map.insertObservation(wallObservation({1.0, 0.0, 0.0}), mrpt::poses::CPose3D::Identity());
  ASSERT_(map.hasPointField("view_x"));

  // A version-1 stream has the same layout, so the current writer can emulate it:
  mrpt::io::CMemoryStream buf;
  auto                    arch = mrpt::serialization::archiveFrom(buf);
  map.serializeTo(arch);
  buf.Seek(0);

  SerializationProbe reloaded;
  reloaded.serializeFrom(arch, 1 /*version*/);

  ASSERT_EQUAL_(reloaded.livePointCount(), map.livePointCount());
  ASSERT_(!reloaded.hasPointField("view_x"));
  ASSERT_(!reloaded.hasPointField("view_y"));
  ASSERT_(!reloaded.hasPointField("view_z"));
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    rng.randomize(1234U);

    test_queries_against_brute_force(false /*sync index*/);
    test_queries_against_brute_force(true /*background rebuilds*/);
    test_bounded_memory_under_churn();
    test_serialization_and_copy();
    test_cov2cov();
    test_pairing_order_is_canonical();
    test_kdtree_bake_roundtrip_memory();
    test_kdtree_bake_roundtrip_file();
    test_kdtree_bake_then_clear_and_reinsert();
    test_kdtree_bake_then_modify();
    test_kdtree_bake_disabled_by_default();
    test_change_coordinates_reference(false /*concrete type*/, false /*sync index*/);
    test_change_coordinates_reference(false /*concrete type*/, true /*background rebuilds*/);
    // A base-class call cannot join a pending background rebuild before it
    // rewrites the coordinate buffers, so that combination is a data race by
    // construction and is deliberately not exercised here.
    test_change_coordinates_reference(true /*base-class pointer*/, false /*sync index*/);
    test_change_coordinates_reference_overloads();
    test_view_fields_in_map_frame();
    test_view_fields_in_recycled_slots();
    test_view_direction_filter();
    test_view_filter_surface_side();
    test_view_filter_default_and_config();
    test_legacy_view_fields_dropped_on_load();
    test_view_fields_cleared_on_base_class_remap();
    test_base_class_remap_then_append();
    test_invalid_view_filter_mode_rejected();

    std::cout << "All tests passed.\n";
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
