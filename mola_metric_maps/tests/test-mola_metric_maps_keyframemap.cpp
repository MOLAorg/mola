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
 * @file   test-mola_metric_maps_keyframemap.cpp
 * @brief  Unit tests for KeyframePointCloudMap: basic API, cov-to-cov NN
 *         matching, and the view-direction pairing filter.
 * @author Jose Luis Blanco Claraco
 * @date   Mar 2026
 */

#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mp2p_icp/NearestPointWithCovCapable.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <set>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace
{

/**
 * Build a CGenericPointsMap populated with the provided points.
 * Each entry is {x, y, z, view_x, view_y, view_z}; the view vector
 * must be a unit vector pointing FROM the point TOWARD the sensor.
 */
mrpt::maps::CGenericPointsMap::Ptr makeCloudWithViews(const std::vector<std::array<float, 6>>& pts)
{
  auto pc = mrpt::maps::CGenericPointsMap::Create();
  pc->registerField_float("view_x");
  pc->registerField_float("view_y");
  pc->registerField_float("view_z");

  for (const auto& p : pts)
  {
    pc->insertPointFast(p[0], p[1], p[2]);
    pc->insertPointField_float("view_x", p[3]);
    pc->insertPointField_float("view_y", p[4]);
    pc->insertPointField_float("view_z", p[5]);
  }
  pc->mark_as_modified();
  return pc;
}

/**
 * Return a flat grid of nx*ny points at height z0, all sharing the same
 * view direction (vx, vy, vz).  Points are spaced 1 m apart, centred near
 * the origin: x in [-(nx/2)..(nx/2-1)], y in [-(ny/2)..(ny/2-1)].
 *
 * Default nx=6, ny=5 → 30 points, enough for k_correspondences_for_cov=5.
 */
std::vector<std::array<float, 6>> makeGridPts(
    float z0 = 0.f, float vx = 0.f, float vy = 0.f, float vz = 1.f, size_t nx = 6, size_t ny = 5)
{
  std::vector<std::array<float, 6>> pts;
  pts.reserve(nx * ny);
  for (size_t i = 0; i < nx; ++i)
  {
    for (size_t j = 0; j < ny; ++j)
    {
      pts.push_back(
          {static_cast<float>(i) - static_cast<float>(nx) / 2,
           static_cast<float>(j) - static_cast<float>(ny) / 2, z0, vx, vy, vz});
    }
  }
  return pts;
}

/// Append a single point to an existing pts vector.
void appendPt(
    std::vector<std::array<float, 6>>& pts, float x, float y, float z, float vx, float vy, float vz)
{
  pts.push_back({x, y, z, vx, vy, vz});
}

/**
 * Build a single-keyframe KeyframePointCloudMap from the given pointcloud.
 *
 * @param pc        Source cloud (may carry view_{x,y,z} fields).
 * @param kf_pose   Pose of the keyframe in the map frame (default: identity).
 * @param k_cov     k_correspondences_for_cov (default: 5, small for tests).
 * @param max_kfs   max_search_keyframes (default: 1).
 */
mola::KeyframePointCloudMap makeMapFromCloud(
    const mrpt::maps::CPointsMap::Ptr& pc,
    const mrpt::poses::CPose3D& kf_pose = mrpt::poses::CPose3D::Identity(), uint32_t k_cov = 5,
    uint32_t max_kfs = 1)
{
  mola::KeyframePointCloudMap m;
  m.creationOptions.k_correspondences_for_cov = k_cov;
  m.creationOptions.max_search_keyframes      = max_kfs;

  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud = pc;
  m.insertObservation(*obs, kf_pose);
  return m;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// ── 1. Basic map lifecycle ──────────────────────────────────────────────────
void test_basic_ops()
{
  mola::KeyframePointCloudMap m;
  m.creationOptions.k_correspondences_for_cov = 5;

  ASSERT_(m.isEmpty());
  ASSERT_EQUAL_(m.point_count(), 0UL);

  // Insert a flat grid
  auto pts        = makeGridPts();  // 30 points
  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud = makeCloudWithViews(pts);
  m.insertObservation(*obs);

  ASSERT_(!m.isEmpty());
  ASSERT_EQUAL_(m.point_count(), pts.size());

  const std::string s = m.asString();
  ASSERT_(!s.empty());
}

// ── 2. clear() empties the map ─────────────────────────────────────────────
void test_clear()
{
  auto m = makeMapFromCloud(makeCloudWithViews(makeGridPts()));
  ASSERT_(!m.isEmpty());
  m.clear();
  ASSERT_(m.isEmpty());
  ASSERT_EQUAL_(m.point_count(), 0UL);
}

// ── 3. Bounding box is sensible ────────────────────────────────────────────
void test_bounding_box()
{
  // Grid: x in [-3..2], y in [-2..2], z = 0
  auto m = makeMapFromCloud(makeCloudWithViews(makeGridPts(0.f)));

  const auto bb = m.boundingBox();
  ASSERT_GT_(bb.max.x, bb.min.x);
  ASSERT_GT_(bb.max.y, bb.min.y);
  // All points at z=0 → flat bounding box
  ASSERT_NEAR_(bb.min.z, 0.f, 1e-3f);
  ASSERT_NEAR_(bb.max.z, 0.f, 1e-3f);
}

// ── 4. point_count() sums across all keyframes ────────────────────────────
void test_point_count_multi_kf()
{
  using mrpt::literals::operator""_deg;

  const size_t                N = 30;
  mola::KeyframePointCloudMap m;
  m.creationOptions.k_correspondences_for_cov = 5;
  m.creationOptions.max_search_keyframes      = 3;

  for (int kf = 0; kf < 3; ++kf)
  {
    auto obs        = mrpt::obs::CObservationPointCloud::Create();
    obs->pointcloud = makeCloudWithViews(makeGridPts(static_cast<float>(kf) * 10.f));
    // Each KF is 10 m apart on z so they stay distinct
    m.insertObservation(
        *obs,
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, kf * 10.0, 0.0_deg, 0.0_deg, 0.0_deg));
  }

  ASSERT_EQUAL_(m.point_count(), 3 * N);
}

// ── 5. merge_with() joins two maps ────────────────────────────────────────
void test_merge_with()
{
  auto pts1 = makeGridPts(0.f);  // 30 pts at z=0
  auto pts2 = makeGridPts(50.f);  // 30 pts at z=50 m (no overlap)

  auto m1 = makeMapFromCloud(makeCloudWithViews(pts1));
  auto m2 = makeMapFromCloud(makeCloudWithViews(pts2));

  m1.merge_with(m2);

  ASSERT_EQUAL_(m1.point_count(), pts1.size() + pts2.size());
  ASSERT_(!m1.isEmpty());
}

// ── 6. TCreationOptions round-trip through serialization ──────────────────
void test_creation_options_roundtrip()
{
  mola::KeyframePointCloudMap m;
  m.creationOptions.use_view_direction_filter = false;
  m.creationOptions.max_view_angle_deg        = 75.0;
  m.creationOptions.k_correspondences_for_cov = 12;
  m.creationOptions.max_search_keyframes      = 4;
  m.creationOptions.rotation_distance_weight  = 3.5;
  m.creationOptions.num_diverse_keyframes     = 2;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << m;
  }
  buf.Seek(0);

  mola::KeyframePointCloudMap m2;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> m2;
  }

  ASSERT_EQUAL_(m2.creationOptions.use_view_direction_filter, false);
  ASSERT_NEAR_(m2.creationOptions.max_view_angle_deg, 75.0, 1e-9);
  ASSERT_EQUAL_(m2.creationOptions.k_correspondences_for_cov, 12U);
  ASSERT_EQUAL_(m2.creationOptions.max_search_keyframes, 4U);
  ASSERT_NEAR_(m2.creationOptions.rotation_distance_weight, 3.5, 1e-9);
  ASSERT_EQUAL_(m2.creationOptions.num_diverse_keyframes, 2U);
}

// ── 7. Full map serialization preserves points and options ─────────────────
void test_serialization_roundtrip()
{
  auto pts = makeGridPts(0.f, 0.f, 0.f, 1.f);  // 30 pts, view=(0,0,1)
  auto m1  = makeMapFromCloud(makeCloudWithViews(pts));
  m1.creationOptions.use_view_direction_filter = true;
  m1.creationOptions.max_view_angle_deg        = 90.0;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << m1;
  }
  buf.Seek(0);

  mola::KeyframePointCloudMap m2;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> m2;
  }

  ASSERT_EQUAL_(m1.point_count(), m2.point_count());
  ASSERT_EQUAL_(m2.creationOptions.use_view_direction_filter, true);
  ASSERT_NEAR_(m2.creationOptions.max_view_angle_deg, 90.0, 1e-9);
}

// ── 8. NN pairings are formed (no view filter) ────────────────────────────
// Both clouds have identical xy positions but local is lifted by dz.
// All 30 local points should find their counterpart in the global cloud.
void test_nn_pairings_formed_no_view_filter()
{
  constexpr float kDz      = 0.05f;  // 5 cm lift
  constexpr float kMaxDist = 1.0f;

  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);  // z=0
  auto local_pts  = makeGridPts(kDz, 0.f, 0.f, 1.f);  // z=0.05

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto local_m  = makeMapFromCloud(makeCloudWithViews(local_pts));

  // Disable filter so every within-radius pair passes
  global_m.creationOptions.use_view_direction_filter = false;

  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  mp2p_icp::MatchedPointWithCovList pairings;
  global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);

  ASSERT_EQUAL_(pairings.size(), global_pts.size());

  // Every pairing should have distance² ≈ dz²
  for (const auto& p : pairings)
  {
    const auto  diff = p.global - p.local;
    const float d2   = diff.sqrNorm();
    ASSERT_NEAR_(d2, kDz * kDz, 1e-4f);
  }
}

// ── 9. Pairing local/global indices are self-consistent ───────────────────
void test_pairing_indices_consistent()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);
  auto local_pts  = makeGridPts(kDz, 0.f, 0.f, 1.f);

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto local_m  = makeMapFromCloud(makeCloudWithViews(local_pts));

  global_m.creationOptions.use_view_direction_filter = false;
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  mp2p_icp::MatchedPointWithCovList pairings;
  global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);

  ASSERT_(!pairings.empty());

  for (const auto& p : pairings)
  {
    // cov_inv must be a valid (non-zero) matrix
    ASSERT_GT_(p.cov_inv.norm(), 0.f);

    // Coordinates stored in p.local/p.global must be finite
    for (int i = 0; i < 3; ++i)
    {
      ASSERT_(std::isfinite(p.local[i]));
      ASSERT_(std::isfinite(p.global[i]));
    }
  }
}

// ── 9b. Pairings come back in a canonical order, every run ────────────────
// The parallel path accumulates into thread-local vectors and merges them, and
// the merge order is unspecified. The resulting permutation reaches the
// solver's summation order and therefore the optimized pose, so it must not
// depend on which workers happened to run. Ascending local_idx is the order the
// sequential path produces, so asserting it also pins the parallel path to the
// sequential result.
void test_pairing_order_is_canonical()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  // Big enough that TBB really splits the range across workers: on a small
  // cloud the defect this guards against does not show up at all.
  constexpr size_t kNx = 200, kNy = 200;

  const auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f, kNx, kNy);
  const auto local_pts  = makeGridPts(kDz, 0.f, 0.f, 1.f, kNx, kNy);

  for (const bool approximate : {false, true})
  {
    auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
    global_m.creationOptions.use_view_direction_filter = false;
    global_m.creationOptions.approximate_cov           = approximate;
    global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

    std::vector<std::pair<uint32_t, uint32_t>> reference;

    for (int rep = 0; rep < 5; ++rep)
    {
      auto local_m = makeMapFromCloud(makeCloudWithViews(local_pts));

      mp2p_icp::MatchedPointWithCovList pairings;
      global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);

      ASSERT_(!pairings.empty());

      // Each local point yields at most one pairing, so the key is unique and
      // the order is strict:
      for (size_t i = 1; i < pairings.size(); ++i)
      {
        ASSERTMSG_(
            pairings[i - 1].local_idx < pairings[i].local_idx,
            "Pairings are not in canonical (ascending local_idx) order");
      }

      std::vector<std::pair<uint32_t, uint32_t>> idx;
      idx.reserve(pairings.size());
      for (const auto& p : pairings)
      {
        idx.emplace_back(p.local_idx, p.global_idx);
      }

      if (rep == 0)
      {
        reference = std::move(idx);
      }
      else
      {
        ASSERTMSG_(idx == reference, "The pairing list changed between identical runs");
      }
    }
  }
}

// ── 10. Points beyond max_search_distance are never paired ────────────────
void test_distance_threshold_respected()
{
  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);
  // Local cloud displaced 5 m in z → well beyond any reasonable threshold
  auto local_pts = makeGridPts(5.f, 0.f, 0.f, 1.f);

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto local_m  = makeMapFromCloud(makeCloudWithViews(local_pts));

  global_m.creationOptions.use_view_direction_filter = false;
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  mp2p_icp::MatchedPointWithCovList pairings;
  global_m.nn_search_cov2cov(
      local_m, mrpt::poses::CPose3D::Identity(),
      /*max_search_distance=*/0.5f,  // smaller than the 5 m gap
      pairings);

  ASSERT_EQUAL_(pairings.size(), 0UL);
}

// ── 11. View filter: opposite views reject exactly one bad pair ────────────
//
// Scenario
// --------
// Global KF  (pose = identity, 30 background pts + P_bad + P_good)
//   • 30 background points at z=0,  view=(0,0,+1)   [front-face: seen from above]
//   • P_bad  at (kTrapX, 0, 0),     view=(0,0,−1)   [back-face: seen from below]
//   • P_good at (kGoodX, 0, 0),     view=(0,0,+1)   [front-face]
//
// Local KF  (same layout, z-offset by kDz, all view=(0,0,+1))
//   • 30 background counterparts at z=kDz
//   • Q_trap at (kTrapX, 0, kDz)  → NN in global = P_bad  (view angle 180°)
//   • Q_good at (kGoodX, 0, kDz)  → NN in global = P_good (view angle   0°)
//
// With filter ON (threshold 120°):
//   Q_trap → P_bad  rejected  (180° > 120°)     → 31 pairings total
// With filter OFF:
//   all 32 pairings pass
void test_view_filter_rejects_opposite_view_pairs()
{
  constexpr float kDz      = 0.02f;  // z offset between global and local
  constexpr float kMaxDist = 1.0f;  // generous search radius
  constexpr float kTrapX   = 10.0f;  // x of the "trap" (back-face) pair
  constexpr float kGoodX   = 20.0f;  // x of the "good" pair

  // ------ Global reference cloud ------
  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);  // 30 bg, view=(0,0,+1)
  appendPt(global_pts, kTrapX, 0.f, 0.f, 0.f, 0.f, -1.f);  // P_bad:  back-face
  appendPt(global_pts, kGoodX, 0.f, 0.f, 0.f, 0.f, +1.f);  // P_good: front-face
  const size_t totalPts = global_pts.size();  // 32

  // ------ Local query cloud ------
  auto local_pts = makeGridPts(kDz, 0.f, 0.f, 1.f);  // 30 bg at z=kDz
  appendPt(local_pts, kTrapX, 0.f, kDz, 0.f, 0.f, +1.f);  // Q_trap: front-face
  appendPt(local_pts, kGoodX, 0.f, kDz, 0.f, 0.f, +1.f);  // Q_good: front-face
  ASSERT_EQUAL_(local_pts.size(), totalPts);

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto local_m  = makeMapFromCloud(makeCloudWithViews(local_pts));

  // Prepare reference submap once - it will carry view fields after the fix
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  // ---- filter ON (default 120°) ----
  global_m.creationOptions.use_view_direction_filter = true;
  global_m.creationOptions.max_view_angle_deg        = 120.0;
  {
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);
    // Q_trap→P_bad rejected (180° > 120°); all other 31 pairs pass
    ASSERT_EQUAL_(pairings.size(), totalPts - 1);
  }

  // ---- filter OFF ----
  global_m.creationOptions.use_view_direction_filter = false;
  {
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);
    // All 32 pairs pass
    ASSERT_EQUAL_(pairings.size(), totalPts);
  }
}

// ── 12. View filter: similar views are never rejected ─────────────────────
// All view vectors are (0,0,+1) in both clouds.  Even with a very tight
// threshold of 10°, no pair should be rejected since angle = 0°.
void test_view_filter_accepts_aligned_views()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);
  auto local_pts  = makeGridPts(kDz, 0.f, 0.f, 1.f);

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto local_m  = makeMapFromCloud(makeCloudWithViews(local_pts));

  global_m.creationOptions.use_view_direction_filter = true;
  global_m.creationOptions.max_view_angle_deg        = 10.0;  // very tight
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  mp2p_icp::MatchedPointWithCovList pairings;
  global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);

  // 0° < 10° → all 30 pairs pass
  ASSERT_EQUAL_(pairings.size(), global_pts.size());
}

// ── 13. View filter: angle threshold controls exactly which pairs survive ──
//
// This test isolates the threshold arithmetic.  One pair has a view angle
// of 90° (dot = 0); the other pairs have angle = 0°.
//
//   threshold 60°  → cos = 0.5  → dot=0  <  0.5  → 90°-pair REJECTED
//   threshold 100° → cos ≈ −0.17 → dot=0 > −0.17 → 90°-pair ACCEPTED
void test_view_filter_angle_threshold_boundary()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  // Background: 30 pts all with matching view=(0,0,1) → always pass
  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);
  auto local_pts  = makeGridPts(kDz, 0.f, 0.f, 1.f);

  // Add a "90-degree pair":
  //   Global P90 at (100, 0, 0)  with view = (1, 0, 0)  [+x direction]
  //   Local  Q90 at (100, 0, dz) with view = (0, 1, 0)  [+y direction]
  //   dot((1,0,0),(0,1,0)) = 0  → view angle = 90°
  appendPt(global_pts, 100.f, 0.f, 0.f, 1.f, 0.f, 0.f);  // P90
  appendPt(local_pts, 100.f, 0.f, kDz, 0.f, 1.f, 0.f);  // Q90

  const size_t totalPts = global_pts.size();  // 31

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto local_m  = makeMapFromCloud(makeCloudWithViews(local_pts));

  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  // ---- threshold = 60°: cos(60°)=0.5 → dot=0 < 0.5 → 90°-pair REJECTED ----
  global_m.creationOptions.use_view_direction_filter = true;
  global_m.creationOptions.max_view_angle_deg        = 60.0;
  {
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);
    ASSERT_EQUAL_(pairings.size(), totalPts - 1);  // 90°-pair excluded
  }

  // ---- threshold = 100°: cos(100°)≈−0.174 → dot=0 > −0.174 → ACCEPTED ----
  global_m.creationOptions.max_view_angle_deg = 100.0;
  {
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);
    ASSERT_EQUAL_(pairings.size(), totalPts);  // 90°-pair now included
  }
}

// ── 14. View filter: graceful no-op when clouds have no view fields ─────────
// If neither cloud carries view_{x,y,z}, do_view_filter is false even when
// use_view_direction_filter=true.  Result must equal the no-filter case.
void test_view_filter_graceful_when_no_view_fields()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  // Use plain CSimplePointsMap (no custom fields)
  auto make_simple = [](float z) -> mrpt::maps::CPointsMap::Ptr
  {
    auto pts = makeGridPts(z);
    auto pc  = mrpt::maps::CSimplePointsMap::Create();
    for (const auto& p : pts)
    {
      pc->insertPoint(p[0], p[1], p[2]);
    }
    return pc;
  };

  auto global_m = makeMapFromCloud(make_simple(0.f));
  auto local_m  = makeMapFromCloud(make_simple(kDz));

  // Turn filter ON - but because there are no view fields, it cannot activate
  global_m.creationOptions.use_view_direction_filter = true;
  global_m.creationOptions.max_view_angle_deg        = 120.0;
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  mp2p_icp::MatchedPointWithCovList pairings;
  global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);

  // All 30 pairs should pass because the filter had no data to work with
  ASSERT_EQUAL_(pairings.size(), 30UL);
}

// ── 15. View filter: local KF has non-trivial rotation ─────────────────────
//
// The local KF is inserted with a 90° CCW yaw rotation around z.
// View vectors stored in the local cloud are in the *local KF frame*; the
// filter must rotate them by R_local before comparing against the global
// reference vectors.
//
// Coordinate conventions (90° CCW yaw: R maps +x→+y, +y→−x, +z→+z):
//   R = [[0,−1,0],[+1,0,0],[0,0,1]]     R^T = [[0,+1,0],[−1,0,0],[0,0,1]]
//
// Setup
// -----
// Global (identity pose):
//   background grid for covariance + P at (50, 0, 0) with view=(0,+1,0)
//
// Local KF (90° CCW yaw, no translation):
//   background grid + Q whose LOCAL coords map to GLOBAL (50, 0, dz):
//     global (50, 0, dz) → local = R^T*(50,0,dz) = (0, −50, dz)
//
// Case A: local view q=(+1,0,0) → R*q=(0,+1,0) [matches P's global view]
//         dot=1, angle=0° → always ACCEPTED
//
// Case B: local view q=(0,−1,0) → R*q=(+1,0,0) [⊥ to P's global view (0,1,0)]
//         dot=0, angle=90°
//         threshold=60°  → rejected  (cos60°=0.5, 0 < 0.5)
//         threshold=100° → accepted  (cos100°≈−0.17, 0 > −0.17)

/// Search pairings for one where local coords ≈ lp and global coords ≈ gp.
bool hasPairing(
    const mp2p_icp::MatchedPointWithCovList& ps, const mrpt::math::TPoint3Df& lp,
    const mrpt::math::TPoint3Df& gp, float tol = 0.05f)
{
  for (const auto& p : ps)  // NOLINT
  {
    const auto dl  = p.local - lp;
    const auto dg  = p.global - gp;
    const bool lOk = dl.x * dl.x + dl.y * dl.y + dl.z * dl.z < tol * tol;
    const bool gOk = dg.x * dg.x + dg.y * dg.y + dg.z * dg.z < tol * tol;
    if (lOk && gOk)
    {
      return true;
    }
  }
  return false;
}

void test_view_filter_with_rotated_local_frame()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  // Global reference: background grid at z=0 + P with view=(0,+1,0)
  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);  // bg: view=(0,0,+1)
  appendPt(global_pts, 50.f, 0.f, 0.f, 0.f, +1.f, 0.f);  // P: view=(0,+1,0)
  // const size_t kGlobalSize = global_pts.size();

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  // 90° CCW yaw around z
  const auto localPose =
      mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.0, 0.0, 0.0, mrpt::DEG2RAD(90.0), 0.0, 0.0);

  // Q in LOCAL frame: R^T * (50, 0, kDz) = (0, -50, kDz)
  const mrpt::math::TPoint3Df Q_local{0.f, -50.f, kDz};
  const mrpt::math::TPoint3Df P_global{50.f, 0.f, 0.f};

  // ---- Case A: local view (1,0,0) → R*(1,0,0)=(0,+1,0) ≡ P's global view ----
  {
    auto local_pts = makeGridPts(kDz, 0.f, 0.f, 1.f);  // bg
    appendPt(local_pts, Q_local.x, Q_local.y, Q_local.z, +1.f, 0.f, 0.f);

    auto local_m = makeMapFromCloud(makeCloudWithViews(local_pts), localPose);

    global_m.creationOptions.use_view_direction_filter = true;
    global_m.creationOptions.max_view_angle_deg        = 60.0;  // tight

    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, localPose, kMaxDist, pairings);

    // angle=0° < 60° → Q→P must be ACCEPTED
    ASSERT_(hasPairing(pairings, Q_local, P_global));
  }

  // ---- Case B: local view (0,−1,0) → R*(0,−1,0)=(+1,0,0) ⊥ P's view ----
  {
    auto local_pts = makeGridPts(kDz, 0.f, 0.f, 1.f);
    appendPt(local_pts, Q_local.x, Q_local.y, Q_local.z, 0.f, -1.f, 0.f);

    auto local_m = makeMapFromCloud(makeCloudWithViews(local_pts), localPose);

    // threshold=60° → cos(60°)=0.5, dot=0 < 0.5 → REJECTED
    global_m.creationOptions.max_view_angle_deg = 60.0;
    {
      mp2p_icp::MatchedPointWithCovList pairings;
      global_m.nn_search_cov2cov(local_m, localPose, kMaxDist, pairings);
      ASSERT_(!hasPairing(pairings, Q_local, P_global));
    }

    // threshold=100° → cos(100°)≈−0.174, dot=0 > −0.174 → ACCEPTED
    global_m.creationOptions.max_view_angle_deg = 100.0;
    {
      mp2p_icp::MatchedPointWithCovList pairings;
      global_m.nn_search_cov2cov(local_m, localPose, kMaxDist, pairings);
      ASSERT_(hasPairing(pairings, Q_local, P_global));
    }
  }
}

// ── 17. View filter pairing must be accepted regardless of KF heading ─────
//
// Regression test for the FilterMerge double-rotation bug: a keyframe's
// view_{x,y,z} fields must be expressed in the *local KF frame* regardless
// of the keyframe's absolute heading (see the contract documented at
// `TCreationOptions::use_view_direction_filter`). If view vectors were
// rotated twice (once by the inserter, once by `updatePointsGlobal()`), the
// effective residual rotation error would grow with the keyframe heading,
// eventually flipping an aligned-view pair into a "rejected" one. Sweeping
// heading in {0, 90, 180, 270} deg must accept the very same Q->P pair at
// every heading, since the underlying geometry (Q and P seen from the same
// direction) does not change with heading.
void test_view_filter_heading_sweep_pairing_accepted()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  // Global reference: background grid at z=0 + P with view=(0,+1,0).
  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);  // bg: view=(0,0,+1)
  appendPt(global_pts, 50.f, 0.f, 0.f, 0.f, +1.f, 0.f);  // P: view=(0,+1,0)

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());
  global_m.creationOptions.use_view_direction_filter = true;
  global_m.creationOptions.max_view_angle_deg        = 60.0;  // tight

  for (const double heading_deg : {0.0, 90.0, 180.0, 270.0})
  {
    const auto kfPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        0.0, 0.0, 0.0, mrpt::DEG2RAD(heading_deg), 0.0, 0.0);

    // Q sits near P=(50,0,0) in the global frame, regardless of heading.
    // Express both its position and its view vector (which globally must
    // match P's view (0,+1,0)) in the LOCAL KF frame.
    const mrpt::math::TPoint3D  Q_global_pt{50.0, 0.0, kDz};
    const mrpt::math::TPoint3D  Q_local_pt = kfPose.inverseComposePoint(Q_global_pt);
    const mrpt::math::TVector3D localView  = kfPose.inverseRotateVector({0.0, 1.0, 0.0});

    auto local_pts = makeGridPts(kDz, 0.f, 0.f, 1.f);  // bg, local view=(0,0,+1)
    appendPt(
        local_pts, static_cast<float>(Q_local_pt.x), static_cast<float>(Q_local_pt.y),
        static_cast<float>(Q_local_pt.z), static_cast<float>(localView.x),
        static_cast<float>(localView.y), static_cast<float>(localView.z));

    auto local_m = makeMapFromCloud(makeCloudWithViews(local_pts), kfPose);

    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, kfPose, kMaxDist, pairings);

    const mrpt::math::TPoint3Df Q_local{
        static_cast<float>(Q_local_pt.x), static_cast<float>(Q_local_pt.y),
        static_cast<float>(Q_local_pt.z)};
    const mrpt::math::TPoint3Df P_global{50.f, 0.f, 0.f};
    ASSERT_(hasPairing(pairings, Q_local, P_global));
  }
}

// ── 19. Per-KF pose plumbing ──────────────────────────────────────────────
//
// Covers:
//   - keyframePoses
//   - setKeyframePose (existing id, missing id is a no-op)
//   - lastInsertedKeyFrameID (none / after insert / after clear)
//   - drainEvictedKeyFrameIDs (returns then clears)
//   - nextFreeKeyFrameID_public
//   - eviction tracking via insertionOptions.remove_frames_farther_than
void test_kf_pose_plumbing()
{
  using mrpt::literals::operator""_deg;
  using mrpt::poses::CPose3D;
  using KFID = mola::KeyframePointCloudMap::KeyFrameID;

  mola::KeyframePointCloudMap m;
  m.creationOptions.k_correspondences_for_cov = 5;
  m.creationOptions.max_search_keyframes      = 3;

  // Fresh map: no last id, next id is 0.
  ASSERT_(!m.lastInsertedKeyFrameID().has_value());
  ASSERT_EQUAL_(m.nextFreeKeyFrameID_public(), KFID{0});
  ASSERT_(m.keyframePoses().empty());
  ASSERT_(m.drainEvictedKeyFrameIDs().empty());

  // Insert 3 KFs at distinct positions.
  std::vector<CPose3D> seedPoses = {
      CPose3D::FromXYZYawPitchRoll(0.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg),
      CPose3D::FromXYZYawPitchRoll(2.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg),
      CPose3D::FromXYZYawPitchRoll(4.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg)};

  for (size_t i = 0; i < seedPoses.size(); ++i)
  {
    auto obs        = mrpt::obs::CObservationPointCloud::Create();
    obs->pointcloud = makeCloudWithViews(makeGridPts(static_cast<float>(i) * 10.f));
    const auto next = m.nextFreeKeyFrameID_public();
    ASSERT_EQUAL_(next, static_cast<KFID>(i));
    m.insertObservation(*obs, seedPoses[i]);
    ASSERT_(m.lastInsertedKeyFrameID().has_value());
    ASSERT_EQUAL_(*m.lastInsertedKeyFrameID(), static_cast<KFID>(i));
  }

  // keyframePoses returns a snapshot keyed by id.
  auto snap = m.keyframePoses();
  ASSERT_EQUAL_(snap.size(), seedPoses.size());
  for (size_t i = 0; i < seedPoses.size(); ++i)
  {
    ASSERT_NEAR_(snap.at(static_cast<KFID>(i)).x(), seedPoses[i].x(), 1e-9);
  }

  // Mutating the snapshot does not affect the map.
  snap.at(KFID{0}) = CPose3D::FromXYZYawPitchRoll(99.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg);
  ASSERT_NEAR_(m.keyframePoses().at(KFID{0}).x(), 0.0, 1e-9);

  // setKeyframePose updates the stored pose.
  const auto newPose1 = CPose3D::FromXYZYawPitchRoll(2.0, 0.0, 1.5, 0.0_deg, 5.0_deg, 0.0_deg);
  m.setKeyframePose(KFID{1}, newPose1);
  const auto poseAfter = m.keyframePoses().at(KFID{1});
  ASSERT_NEAR_(poseAfter.x(), newPose1.x(), 1e-9);
  ASSERT_NEAR_(poseAfter.z(), newPose1.z(), 1e-9);
  double y, p, r;
  poseAfter.getYawPitchRoll(y, p, r);
  ASSERT_NEAR_(p, 5.0_deg, 1e-9);

  // setKeyframePose on a missing id is a no-op (does not throw, does not add).
  m.setKeyframePose(KFID{999}, CPose3D::Identity());
  ASSERT_EQUAL_(m.keyframePoses().size(), seedPoses.size());

  // No evictions yet.
  ASSERT_(m.drainEvictedKeyFrameIDs().empty());

  // Trigger partial eviction: insert a new KF at x=5, threshold=2.5 m.
  // KF 0 (dist 5) and KF 1 (dist ~3.35, since z was raised to 1.5) fall
  // outside the threshold; KF 2 at x=4 (dist 1) survives. Keeping at least
  // one KF in place preserves id monotonicity (next id = 3).
  m.insertionOptions.remove_frames_farther_than = 2.5;  // metres

  auto obs            = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud     = makeCloudWithViews(makeGridPts(40.f));
  const auto poseNear = CPose3D::FromXYZYawPitchRoll(5.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg);
  m.insertObservation(*obs, poseNear);

  ASSERT_(m.lastInsertedKeyFrameID().has_value());
  ASSERT_EQUAL_(*m.lastInsertedKeyFrameID(), KFID{3});

  // drainEvictedKeyFrameIDs returns the ids of KFs dropped during the
  // last insertion(s), then clears its internal list.
  const auto evicted1 = m.drainEvictedKeyFrameIDs();
  ASSERT_EQUAL_(evicted1.size(), 2UL);
  std::set<KFID> evictedSet(evicted1.begin(), evicted1.end());
  ASSERT_(evictedSet.count(KFID{0}) == 1);
  ASSERT_(evictedSet.count(KFID{1}) == 1);

  // A second drain returns an empty vector.
  ASSERT_(m.drainEvictedKeyFrameIDs().empty());

  // After clear(), bookkeeping is reset.
  m.clear();
  ASSERT_(!m.lastInsertedKeyFrameID().has_value());
  ASSERT_(m.drainEvictedKeyFrameIDs().empty());
  ASSERT_EQUAL_(m.nextFreeKeyFrameID_public(), KFID{0});
}

// ── 21. Regroup: builds a multi-KF map along a line ───────────────────────
/// Inserts `nkf` keyframes of large overlapping grid clouds, spaced `spacing`
/// metres apart along +x. Returns the assembled map.
mola::KeyframePointCloudMap makeLinearMap(size_t nkf, double spacing, bool withViews = false)
{
  using mrpt::literals::      operator""_deg;
  mola::KeyframePointCloudMap m;
  m.creationOptions.k_correspondences_for_cov = 5;
  m.creationOptions.max_search_keyframes      = 3;

  for (size_t i = 0; i < nkf; ++i)
  {
    // Large grid (21x21, ~20 m span) so consecutive KFs overlap heavily.
    auto                        pts = makeGridPts(0.f, 0.f, 0.f, 1.f, 21, 21);
    mrpt::maps::CPointsMap::Ptr pc;
    if (withViews)
    {
      pc = makeCloudWithViews(pts);
    }
    else
    {
      auto simple = mrpt::maps::CSimplePointsMap::Create();
      for (const auto& p : pts)
      {
        simple->insertPoint(p[0], p[1], p[2]);
      }
      pc = simple;
    }
    auto obs        = mrpt::obs::CObservationPointCloud::Create();
    obs->pointcloud = pc;
    m.insertObservation(
        *obs, mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                  static_cast<double>(i) * spacing, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg));
  }
  return m;
}

// ── 21. Regroup reduces KF count and stays deterministic ──────────────────
void test_regroup_reduces_keyframes()
{
  auto m = makeLinearMap(/*nkf=*/15, /*spacing=*/3.0);
  ASSERT_EQUAL_(m.keyframePoses().size(), 15UL);

  mola::KeyframePointCloudMap::RegroupParams params;  // defaults
  auto                                       g1 = m.regroupKeyframes(params);
  ASSERT_(g1);
  ASSERT_(!g1->isEmpty());

  // Heavy overlap along the line must collapse 15 KFs into strictly fewer
  // super-keyframes.
  ASSERT_LT_(g1->keyframePoses().size(), 15UL);
  ASSERT_GT_(g1->keyframePoses().size(), 0UL);

  // Options are carried over from the source map.
  ASSERT_EQUAL_(g1->creationOptions.max_search_keyframes, m.creationOptions.max_search_keyframes);

  // Deterministic: same input + params -> same number of super-keyframes.
  auto g2 = m.regroupKeyframes(params);
  ASSERT_EQUAL_(g1->keyframePoses().size(), g2->keyframePoses().size());

  // Global bounding box is approximately preserved (points are not lost).
  const auto bbIn  = m.boundingBox();
  const auto bbOut = g1->boundingBox();
  ASSERT_NEAR_(bbIn.min.x, bbOut.min.x, 1.0f);
  ASSERT_NEAR_(bbIn.max.x, bbOut.max.x, 1.0f);
  ASSERT_NEAR_(bbIn.min.y, bbOut.min.y, 1.0f);
  ASSERT_NEAR_(bbIn.max.y, bbOut.max.y, 1.0f);
}

// ── 22. Regroup: every original pose is covered by ONE super-keyframe ──────
void test_regroup_single_kf_coverage()
{
  auto       m         = makeLinearMap(/*nkf=*/15, /*spacing=*/3.0);
  const auto origPoses = m.keyframePoses();

  mola::KeyframePointCloudMap::RegroupParams params;
  auto                                       g = m.regroupKeyframes(params);

  // Force single-active-KF localization behavior.
  g->creationOptions.max_search_keyframes = 1;

  for (const auto& [id, pose] : origPoses)
  {
    g->icp_get_prepared_as_global(pose);

    // With only ONE active super-keyframe selected, a point at the robot
    // location must still find a close neighbor: the group covers this pose.
    mrpt::math::TPoint3Df query{static_cast<float>(pose.x()), static_cast<float>(pose.y()), 0.f};
    mrpt::math::TPoint3Df result;
    float                 dsqr = 0;
    uint64_t              idx  = 0;
    const bool            ok   = g->nn_single_search(query, result, dsqr, idx);
    ASSERT_(ok);
    // The grids are 1 m spaced and centered on each KF, so the nearest point
    // to the robot location is well under 2 m if the pose is truly covered.
    ASSERT_LT_(dsqr, 4.0f);
  }
}

// ── 23. Regroup: empty input yields an empty (but valid) map ───────────────
void test_regroup_empty()
{
  mola::KeyframePointCloudMap                m;
  mola::KeyframePointCloudMap::RegroupParams params;
  auto                                       g = m.regroupKeyframes(params);
  ASSERT_(g);
  ASSERT_(g->isEmpty());
}

// ── 24. Regroup: output survives serialization round-trip ──────────────────
void test_regroup_serialization_roundtrip()
{
  auto m = makeLinearMap(/*nkf=*/12, /*spacing=*/3.0, /*withViews=*/true);

  mola::KeyframePointCloudMap::RegroupParams params;
  auto                                       g = m.regroupKeyframes(params);
  ASSERT_(!g->isEmpty());

  const auto nKF  = g->keyframePoses().size();
  const auto nPts = g->point_count();

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << *g;
  }
  buf.Seek(0);

  mola::KeyframePointCloudMap g2;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> g2;
  }

  ASSERT_EQUAL_(g2.keyframePoses().size(), nKF);
  ASSERT_EQUAL_(g2.point_count(), nPts);
}

// ── 26. Regroup: voxel decimation shrinks the cloud and keeps view fields ──
// Two identical, fully-overlapping keyframes are merged into one super-keyframe
// with merge_decimate_voxel>0. The duplicated points must collapse (fewer points
// than the 2x input), and the per-point view-direction fields must survive the
// decimation - verified functionally: with the view filter ON, an opposite-view
// pair is rejected ONLY if the regrouped+decimated global cloud still carries
// view fields, so ON must yield fewer pairings than OFF.
void test_regroup_decimation_preserves_views()
{
  using mrpt::poses::CPose3D;
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;
  constexpr float kBackX   = 20.0f;  // outside the grid, so it is an isolated NN

  const auto makeKfPts = []
  {
    auto pts = makeGridPts(0.f, 0.f, 0.f, 1.f, 10, 10);  // 100 pts, view=(0,0,+1)
    appendPt(pts, kBackX, 0.f, 0.f, 0.f, 0.f, -1.f);  // back-face pt, view=(0,0,-1)
    return pts;
  };

  mola::KeyframePointCloudMap m;
  m.creationOptions.k_correspondences_for_cov = 5;
  m.creationOptions.max_search_keyframes      = 1;
  for (int k = 0; k < 2; ++k)
  {
    auto obs        = mrpt::obs::CObservationPointCloud::Create();
    obs->pointcloud = makeCloudWithViews(makeKfPts());
    m.insertObservation(*obs, CPose3D::Identity());  // identical -> full overlap
  }
  const auto inPts = m.point_count();  // 2 * 101 = 202

  mola::KeyframePointCloudMap::RegroupParams params;
  params.merge_decimate_voxel = 0.5;  // dedup the co-located duplicate points
  auto g                      = m.regroupKeyframes(params);
  ASSERT_(!g->isEmpty());

  // Decimation must collapse the two identical clouds.
  ASSERT_LT_(g->point_count(), inPts);
  ASSERT_LT_(g->point_count(), inPts * 3 / 4);

  // Local query cloud: grid (for covariances) + a FRONT-face point whose nearest
  // neighbor in the global cloud is the back-face point.
  auto localPts = makeGridPts(kDz, 0.f, 0.f, 1.f, 10, 10);
  appendPt(localPts, kBackX, 0.f, kDz, 0.f, 0.f, 1.f);  // front-face, view=(0,0,+1)
  auto local_m = makeMapFromCloud(makeCloudWithViews(localPts));

  g->creationOptions.max_search_keyframes = 1;
  g->icp_get_prepared_as_global(CPose3D::Identity());

  g->creationOptions.use_view_direction_filter = true;
  g->creationOptions.max_view_angle_deg        = 120.0;
  mp2p_icp::MatchedPointWithCovList pairingsOn;
  g->nn_search_cov2cov(local_m, CPose3D::Identity(), kMaxDist, pairingsOn);

  g->creationOptions.use_view_direction_filter = false;
  mp2p_icp::MatchedPointWithCovList pairingsOff;
  g->nn_search_cov2cov(local_m, CPose3D::Identity(), kMaxDist, pairingsOff);

  // View fields survived decimation -> the opposite-view pair was rejected.
  ASSERT_LT_(pairingsOn.size(), pairingsOff.size());
}

// ── 25. serialize_kdtrees round-trips the stream correctly ─────────────────
// Whether or not the MRPT build provides the KD-tree save/load API, the map
// serialization must stay stream-aligned with serialize_kdtrees enabled, and
// the loaded map must remain fully functional (points + NN queries).
void test_kdtree_serialization_roundtrip()
{
  auto m = makeMapFromCloud(makeCloudWithViews(makeGridPts(0.f, 0.f, 0.f, 1.f, 10, 10)));
  m.creationOptions.serialize_kdtrees = true;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << m;
  }
  buf.Seek(0);

  mola::KeyframePointCloudMap m2;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> m2;
  }

  ASSERT_EQUAL_(m2.point_count(), m.point_count());
  ASSERT_(m2.creationOptions.serialize_kdtrees == true);

  // Functional: prepare a single-KF submap and run a NN query on the loaded map.
  m2.creationOptions.max_search_keyframes = 1;
  m2.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());
  mrpt::math::TPoint3Df query{0.f, 0.f, 0.f};
  mrpt::math::TPoint3Df result;
  float                 dsqr = 0;
  uint64_t              idx  = 0;
  ASSERT_(m2.nn_single_search(query, result, dsqr, idx));
  ASSERT_LT_(dsqr, 4.0f);
}

// ── 34. serialize_covariances round-trips and yields identical covariances ──
// A map baked with serialize_covariances must, after load, produce the exact same
// cov-to-cov pairings (including cov_inv) as one that computed its covariances at
// runtime. This proves both that the persisted covariances are installed and used
// (not silently recomputed), and that the approximate_cov path querying each KF's
// own local (baked) KD-tree yields the same result as before.
void test_covariance_serialization_roundtrip()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f, 10, 10);
  auto local_pts  = makeGridPts(kDz, 0.f, 0.f, 1.f, 10, 10);

  // Reference map: covariances computed at runtime.
  auto ref_m                            = makeMapFromCloud(makeCloudWithViews(global_pts));
  ref_m.creationOptions.approximate_cov = true;
  ref_m.creationOptions.use_view_direction_filter = false;

  // Baked map: same points, covariances serialized into the stream.
  auto bake_m                            = makeMapFromCloud(makeCloudWithViews(global_pts));
  bake_m.creationOptions.approximate_cov = true;
  bake_m.creationOptions.use_view_direction_filter = false;
  bake_m.creationOptions.serialize_covariances     = true;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << bake_m;
  }
  buf.Seek(0);
  mola::KeyframePointCloudMap loaded_m;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> loaded_m;
  }

  ASSERT_(loaded_m.creationOptions.serialize_covariances == true);
  ASSERT_EQUAL_(loaded_m.point_count(), global_pts.size());

  const auto run = [&](mola::KeyframePointCloudMap& m)
  {
    auto local_m = makeMapFromCloud(makeCloudWithViews(local_pts));
    m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());
    mp2p_icp::MatchedPointWithCovList ps;
    m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, ps);
    // Sort indices, not the pairings themselves, to avoid heapsort/std::sort
    // passing the (over-aligned) point_with_cov_pair_t by value.
    std::vector<size_t> order(ps.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(
        order.begin(), order.end(),
        [&ps](size_t i, size_t j) { return ps[i].local_idx < ps[j].local_idx; });
    mp2p_icp::MatchedPointWithCovList sorted;
    sorted.reserve(ps.size());
    for (size_t idx : order)
    {
      sorted.push_back(ps[idx]);
    }
    return sorted;
  };

  auto refPs    = run(ref_m);
  auto loadedPs = run(loaded_m);

  ASSERT_EQUAL_(refPs.size(), loadedPs.size());
  ASSERT_GT_(refPs.size(), 0UL);
  for (size_t i = 0; i < refPs.size(); i++)
  {
    ASSERT_EQUAL_(refPs[i].local_idx, loadedPs[i].local_idx);
    ASSERT_NEAR_((refPs[i].global - loadedPs[i].global).norm(), 0.f, 1e-5f);
    ASSERT_NEAR_((refPs[i].cov_inv - loadedPs[i].cov_inv).norm(), 0.0f, 1e-4f);
  }
}

// ── 22. Debug per-keyframe coloring env var ───────────────────────────────
/// With MOLA_KEYFRAME_MAP_VIZ_COLOR_BY_KF=1 every keyframe must be rendered
/// with a single, uniform color, and different keyframes must get different
/// colors (so regrouped clusters are visually distinguishable).
void test_viz_color_by_kf()
{
  constexpr size_t nkf = 3;
  auto             m   = makeLinearMap(nkf, 5.0);

  // Note: getVisualizationInto() caches the env var value in a thread_local,
  // so this test must be the only one that reads it (it is).
  setenv("MOLA_KEYFRAME_MAP_VIZ_COLOR_BY_KF", "1", 1);
  auto glObj = m.getVisualization();
  unsetenv("MOLA_KEYFRAME_MAP_VIZ_COLOR_BY_KF");

  ASSERT_(glObj);

  // Collect the (uniform) color of each point-cloud child object.
  std::set<uint32_t> distinctColors;
  size_t             cloudObjs = 0;
  for (const auto& child : *glObj)
  {
    auto pc = std::dynamic_pointer_cast<mrpt::opengl::CPointCloudColoured>(child);
    if (!pc || pc->size() == 0)
    {
      continue;
    }
    ++cloudObjs;

    const auto c0 = pc->getPointColor(0);
    for (size_t i = 1; i < pc->size(); ++i)
    {
      const auto ci = pc->getPointColor(i);
      ASSERT_EQUAL_(ci.R, c0.R);
      ASSERT_EQUAL_(ci.G, c0.G);
      ASSERT_EQUAL_(ci.B, c0.B);
    }
    distinctColors.insert(
        (static_cast<uint32_t>(c0.R) << 16) | (static_cast<uint32_t>(c0.G) << 8) |
        static_cast<uint32_t>(c0.B));
  }

  ASSERT_EQUAL_(cloudObjs, nkf);
  ASSERT_EQUAL_(distinctColors.size(), nkf);
}

// ── 20. Default filter parameters are sane ────────────────────────────────
void test_default_creation_options()
{
  mola::KeyframePointCloudMap m;
  ASSERT_(m.creationOptions.use_view_direction_filter == true);
  ASSERT_NEAR_(m.creationOptions.max_view_angle_deg, 120.0, 1e-9);
  ASSERT_GT_(m.creationOptions.k_correspondences_for_cov, 0U);
  ASSERT_GT_(m.creationOptions.max_search_keyframes, 0U);
  ASSERT_(m.creationOptions.approximate_cov == false);
  ASSERT_(m.creationOptions.serialize_kdtrees == false);
  ASSERT_(m.creationOptions.serialize_covariances == false);
}

// ---------------------------------------------------------------------------
// approximate_cov: per-active-KF KD-tree queries instead of a merged submap
// (see TCreationOptions::approximate_cov / nn_search_cov2cov_approximate()).
// ---------------------------------------------------------------------------

// ── 27. approximate_cov round-trips through serialization ─────────────────
void test_approximate_cov_option_roundtrip()
{
  mola::KeyframePointCloudMap m;
  m.creationOptions.approximate_cov = true;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << m;
  }
  buf.Seek(0);

  mola::KeyframePointCloudMap m2;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> m2;
  }
  ASSERT_(m2.creationOptions.approximate_cov == true);
}

// ── 28. approximate_cov requires icp_get_prepared_as_global() first ───────
// Mirrors the exact-mode contract: calling nn_search_cov2cov() without a prior
// icp_get_prepared_as_global() must fail loudly rather than silently no-op.
void test_approximate_cov_requires_prepared_call()
{
  auto global_m                            = makeMapFromCloud(makeCloudWithViews(makeGridPts()));
  auto local_m                             = makeMapFromCloud(makeCloudWithViews(makeGridPts()));
  global_m.creationOptions.approximate_cov = true;

  mp2p_icp::MatchedPointWithCovList pairings;
  bool                              threw = false;
  try
  {
    global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), 1.0f, pairings);
  }
  catch (const std::exception&)
  {
    threw = true;
  }
  ASSERT_(threw);
}

// ── 29. approximate_cov (single active KF) matches exact mode ─────────────
// With exactly one active KF, the "merged submap" is identical to that KF's own
// global cloud, and covariances are computed from the very same within-KF
// neighborhood in both modes. So the two modes must produce numerically
// equivalent pairings (same count, same points, same covariances).
void test_approximate_cov_matches_exact_single_kf()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);
  auto local_pts  = makeGridPts(kDz, 0.f, 0.f, 1.f);

  auto exact_m                             = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto approx_m                            = makeMapFromCloud(makeCloudWithViews(global_pts));
  approx_m.creationOptions.approximate_cov = true;

  auto local_m1 = makeMapFromCloud(makeCloudWithViews(local_pts));
  auto local_m2 = makeMapFromCloud(makeCloudWithViews(local_pts));

  exact_m.creationOptions.use_view_direction_filter  = false;
  approx_m.creationOptions.use_view_direction_filter = false;

  exact_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());
  approx_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  mp2p_icp::MatchedPointWithCovList exactPairings, approxPairings;
  exact_m.nn_search_cov2cov(local_m1, mrpt::poses::CPose3D::Identity(), kMaxDist, exactPairings);
  approx_m.nn_search_cov2cov(local_m2, mrpt::poses::CPose3D::Identity(), kMaxDist, approxPairings);

  ASSERT_EQUAL_(exactPairings.size(), approxPairings.size());
  ASSERT_EQUAL_(exactPairings.size(), global_pts.size());

  // Sort indices by local_idx so entries line up (TBB may reorder them),
  // rather than sorting the pairings themselves.
  std::vector<size_t> exactOrder(exactPairings.size());
  std::vector<size_t> approxOrder(approxPairings.size());
  std::iota(exactOrder.begin(), exactOrder.end(), 0);
  std::iota(approxOrder.begin(), approxOrder.end(), 0);

  const auto byLocalIdx = [](const auto& list)
  { return [&list](size_t i, size_t j) { return list[i].local_idx < list[j].local_idx; }; };
  std::sort(exactOrder.begin(), exactOrder.end(), byLocalIdx(exactPairings));
  std::sort(approxOrder.begin(), approxOrder.end(), byLocalIdx(approxPairings));

  for (size_t i = 0; i < exactPairings.size(); i++)
  {
    const auto& e = exactPairings[exactOrder[i]];
    const auto& a = approxPairings[approxOrder[i]];
    ASSERT_EQUAL_(e.local_idx, a.local_idx);
    ASSERT_NEAR_((e.local - a.local).norm(), 0.f, 1e-5f);
    ASSERT_NEAR_((e.global - a.global).norm(), 0.f, 1e-5f);
    ASSERT_NEAR_((e.cov_inv - a.cov_inv).norm(), 0.0f, 1e-3f);
  }
}

// ── 30. approximate_cov: distance threshold is still respected ────────────
void test_approximate_cov_distance_threshold_respected()
{
  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);
  auto local_pts  = makeGridPts(5.f, 0.f, 0.f, 1.f);  // 5 m away: beyond any small threshold

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto local_m  = makeMapFromCloud(makeCloudWithViews(local_pts));

  global_m.creationOptions.approximate_cov           = true;
  global_m.creationOptions.use_view_direction_filter = false;
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  mp2p_icp::MatchedPointWithCovList pairings;
  global_m.nn_search_cov2cov(
      local_m, mrpt::poses::CPose3D::Identity(), /*max_search_distance=*/0.5f, pairings);

  ASSERT_EQUAL_(pairings.size(), 0UL);
}

// ── 31. approximate_cov with multiple active KFs picks the true closest ────
// Two well-separated keyframes (no overlap) are both in the active set. Each
// query point must pair with a point from ITS OWN nearby keyframe, proving the
// "N kd-tree queries, keep the best" logic correctly picks across keyframes
// rather than only ever using the first (or last) one queried.
void test_approximate_cov_multi_kf_picks_closest()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;
  constexpr float kKf1X    = 100.0f;  // far enough that the two grids never overlap

  mola::KeyframePointCloudMap m;
  m.creationOptions.k_correspondences_for_cov = 5;
  m.creationOptions.max_search_keyframes      = 2;
  m.creationOptions.approximate_cov           = true;
  m.creationOptions.use_view_direction_filter = false;

  auto obs0        = mrpt::obs::CObservationPointCloud::Create();
  obs0->pointcloud = makeCloudWithViews(makeGridPts(0.f));
  m.insertObservation(*obs0, mrpt::poses::CPose3D::Identity());

  auto obs1        = mrpt::obs::CObservationPointCloud::Create();
  obs1->pointcloud = makeCloudWithViews(makeGridPts(0.f));
  m.insertObservation(*obs1, mrpt::poses::CPose3D::FromXYZYawPitchRoll(kKf1X, 0, 0, 0, 0, 0));

  m.icp_get_prepared_as_global(mrpt::poses::CPose3D::FromXYZYawPitchRoll(kKf1X / 2, 0, 0, 0, 0, 0));

  // Local query cloud: one grid near KF0, one grid near KF1 (both offset by kDz).
  auto local_pts = makeGridPts(kDz, 0.f, 0.f, 1.f);
  for (const auto& p : makeGridPts(kDz, 0.f, 0.f, 1.f))
  {
    appendPt(local_pts, p[0] + kKf1X, p[1], p[2], p[3], p[4], p[5]);
  }
  auto local_m = makeMapFromCloud(makeCloudWithViews(local_pts));

  mp2p_icp::MatchedPointWithCovList pairings;
  m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);

  // Every local point (near either KF) must find a close pairing in its own KF.
  ASSERT_EQUAL_(pairings.size(), local_pts.size());
  for (const auto& p : pairings)
  {
    const auto  diff = p.global - p.local;
    const float d2   = diff.sqrNorm();
    ASSERT_NEAR_(d2, kDz * kDz, 1e-4f);
  }
}

// ── 32. approximate_cov: view-direction filter still applies ──────────────
// Same scenario as test_view_filter_rejects_opposite_view_pairs(), but with
// approximate_cov=true and a single active KF, confirming the filter logic is
// preserved by the per-KF code path.
void test_approximate_cov_view_filter_rejects_opposite_view_pairs()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;
  constexpr float kTrapX   = 10.0f;
  constexpr float kGoodX   = 20.0f;

  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);
  appendPt(global_pts, kTrapX, 0.f, 0.f, 0.f, 0.f, -1.f);  // P_bad: back-face
  appendPt(global_pts, kGoodX, 0.f, 0.f, 0.f, 0.f, +1.f);  // P_good: front-face
  const size_t totalPts = global_pts.size();  // 32

  auto local_pts = makeGridPts(kDz, 0.f, 0.f, 1.f);
  appendPt(local_pts, kTrapX, 0.f, kDz, 0.f, 0.f, +1.f);  // Q_trap: front-face
  appendPt(local_pts, kGoodX, 0.f, kDz, 0.f, 0.f, +1.f);  // Q_good: front-face
  ASSERT_EQUAL_(local_pts.size(), totalPts);

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  auto local_m  = makeMapFromCloud(makeCloudWithViews(local_pts));

  global_m.creationOptions.approximate_cov = true;
  global_m.icp_get_prepared_as_global(mrpt::poses::CPose3D::Identity());

  // ---- filter ON (default 120°) ----
  global_m.creationOptions.use_view_direction_filter = true;
  global_m.creationOptions.max_view_angle_deg        = 120.0;
  {
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);
    ASSERT_EQUAL_(pairings.size(), totalPts - 1);
  }

  // ---- filter OFF ----
  global_m.creationOptions.use_view_direction_filter = false;
  {
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, mrpt::poses::CPose3D::Identity(), kMaxDist, pairings);
    ASSERT_EQUAL_(pairings.size(), totalPts);
  }
}

// ── 33. Toggling approximate_cov forces a rebuild even if the KF selection
//        (active set) does not change ────────────────────────────────────
// icp_get_prepared_as_global() early-returns when the active-KF-set selection
// is unchanged. This must NOT short-circuit a switch between exact/approximate
// mode, or nn_search_cov2cov() would use a stale cache built under the other
// mode. Toggle the flag with the very same query pose (so the KF selection is
// identical) and check both modes still work right after the switch.
void test_approximate_cov_mode_switch_invalidates_cache()
{
  constexpr float kDz      = 0.02f;
  constexpr float kMaxDist = 1.0f;

  auto global_pts = makeGridPts(0.f, 0.f, 0.f, 1.f);
  auto local_pts  = makeGridPts(kDz, 0.f, 0.f, 1.f);

  auto global_m = makeMapFromCloud(makeCloudWithViews(global_pts));
  global_m.creationOptions.use_view_direction_filter = false;

  const auto pose = mrpt::poses::CPose3D::Identity();

  // Start in exact mode.
  global_m.icp_get_prepared_as_global(pose);
  {
    auto                              local_m = makeMapFromCloud(makeCloudWithViews(local_pts));
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, pose, kMaxDist, pairings);
    ASSERT_EQUAL_(pairings.size(), global_pts.size());
  }

  // Switch to approximate mode with the SAME pose (same KF selection).
  global_m.creationOptions.approximate_cov = true;
  global_m.icp_get_prepared_as_global(pose);
  {
    auto                              local_m = makeMapFromCloud(makeCloudWithViews(local_pts));
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, pose, kMaxDist, pairings);
    ASSERT_EQUAL_(pairings.size(), global_pts.size());
  }

  // Switch back to exact mode, again with the same pose.
  global_m.creationOptions.approximate_cov = false;
  global_m.icp_get_prepared_as_global(pose);
  {
    auto                              local_m = makeMapFromCloud(makeCloudWithViews(local_pts));
    mp2p_icp::MatchedPointWithCovList pairings;
    global_m.nn_search_cov2cov(local_m, pose, kMaxDist, pairings);
    ASSERT_EQUAL_(pairings.size(), global_pts.size());
  }
}
}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    std::cout << "test_basic_ops ...\n";
    test_basic_ops();

    std::cout << "test_clear ...\n";
    test_clear();

    std::cout << "test_bounding_box ...\n";
    test_bounding_box();

    std::cout << "test_point_count_multi_kf ...\n";
    test_point_count_multi_kf();

    std::cout << "test_merge_with ...\n";
    test_merge_with();

    std::cout << "test_creation_options_roundtrip ...\n";
    test_creation_options_roundtrip();

    std::cout << "test_serialization_roundtrip ...\n";
    test_serialization_roundtrip();

    std::cout << "test_nn_pairings_formed_no_view_filter ...\n";
    test_nn_pairings_formed_no_view_filter();

    std::cout << "test_pairing_indices_consistent ...\n";
    test_pairing_indices_consistent();

    std::cout << "test_pairing_order_is_canonical ...\n";
    test_pairing_order_is_canonical();

    std::cout << "test_distance_threshold_respected ...\n";
    test_distance_threshold_respected();

    std::cout << "test_view_filter_rejects_opposite_view_pairs ...\n";
    test_view_filter_rejects_opposite_view_pairs();

    std::cout << "test_view_filter_accepts_aligned_views ...\n";
    test_view_filter_accepts_aligned_views();

    std::cout << "test_view_filter_angle_threshold_boundary ...\n";
    test_view_filter_angle_threshold_boundary();

    std::cout << "test_view_filter_graceful_when_no_view_fields ...\n";
    test_view_filter_graceful_when_no_view_fields();

    std::cout << "test_view_filter_with_rotated_local_frame ...\n";
    test_view_filter_with_rotated_local_frame();

    std::cout << "test_view_filter_heading_sweep_pairing_accepted ...\n";
    test_view_filter_heading_sweep_pairing_accepted();

    std::cout << "test_kf_pose_plumbing ...\n";
    test_kf_pose_plumbing();

    std::cout << "test_default_creation_options ...\n";
    test_default_creation_options();

    std::cout << "test_regroup_reduces_keyframes ...\n";
    test_regroup_reduces_keyframes();

    std::cout << "test_regroup_single_kf_coverage ...\n";
    test_regroup_single_kf_coverage();

    std::cout << "test_regroup_empty ...\n";
    test_regroup_empty();

    std::cout << "test_regroup_serialization_roundtrip ...\n";
    test_regroup_serialization_roundtrip();

    std::cout << "test_regroup_decimation_preserves_views ...\n";
    test_regroup_decimation_preserves_views();

    std::cout << "test_kdtree_serialization_roundtrip ...\n";
    test_kdtree_serialization_roundtrip();

    std::cout << "test_covariance_serialization_roundtrip ...\n";
    test_covariance_serialization_roundtrip();

    std::cout << "test_viz_color_by_kf ...\n";
    test_viz_color_by_kf();

    std::cout << "test_approximate_cov_option_roundtrip ...\n";
    test_approximate_cov_option_roundtrip();

    std::cout << "test_approximate_cov_requires_prepared_call ...\n";
    test_approximate_cov_requires_prepared_call();

    std::cout << "test_approximate_cov_matches_exact_single_kf ...\n";
    test_approximate_cov_matches_exact_single_kf();

    std::cout << "test_approximate_cov_distance_threshold_respected ...\n";
    test_approximate_cov_distance_threshold_respected();

    std::cout << "test_approximate_cov_multi_kf_picks_closest ...\n";
    test_approximate_cov_multi_kf_picks_closest();

    std::cout << "test_approximate_cov_view_filter_rejects_opposite_view_pairs ...\n";
    test_approximate_cov_view_filter_rejects_opposite_view_pairs();

    std::cout << "test_approximate_cov_mode_switch_invalidates_cache ...\n";
    test_approximate_cov_mode_switch_invalidates_cache();

    std::cout << "\nAll tests passed.\n";
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
