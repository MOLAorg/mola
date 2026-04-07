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
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <cmath>
#include <iostream>

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

// ── 16. Default filter parameters are sane ────────────────────────────────
void test_default_creation_options()
{
  mola::KeyframePointCloudMap m;
  ASSERT_(m.creationOptions.use_view_direction_filter == true);
  ASSERT_NEAR_(m.creationOptions.max_view_angle_deg, 120.0, 1e-9);
  ASSERT_GT_(m.creationOptions.k_correspondences_for_cov, 0U);
  ASSERT_GT_(m.creationOptions.max_search_keyframes, 0U);
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

    std::cout << "test_default_creation_options ...\n";
    test_default_creation_options();

    std::cout << "\nAll tests passed.\n";
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
