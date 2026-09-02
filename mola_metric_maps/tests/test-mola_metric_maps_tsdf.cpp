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
 * @file   test-mola_metric_maps_tsdf.cpp
 * @brief  Test the TSDF map class
 * @author Jose Luis Blanco Claraco
 * @date   Sep 02, 2026
 */

#include <mola_metric_maps/TSDF.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <array>
#include <cmath>
#include <iostream>
#include <random>

namespace
{
constexpr float PLANE_SPAN     = 6.0f;
constexpr float PLANE_STEP     = 0.05f;
constexpr float SENSOR_HEIGHT  = 2.0f;
constexpr float TEST_VOXEL     = 0.25f;
constexpr float QUERY_TOLERANC = 0.02f;

/** Fills a horizontal plane z=0, observed from a sensor above it. */
void fill_ground_plane(mola::TSDF& map, float zPlane = .0f, float noiseSigma = .0f)
{
  std::mt19937                    rng(1234);
  std::normal_distribution<float> noise(.0f, noiseSigma);
  const mrpt::math::TPoint3Df     sensor(.0f, .0f, SENSOR_HEIGHT);

  for (float x = -PLANE_SPAN; x <= PLANE_SPAN; x += PLANE_STEP)
  {
    for (float y = -PLANE_SPAN; y <= PLANE_SPAN; y += PLANE_STEP)
    {
      const float z = zPlane + (noiseSigma > 0 ? noise(rng) : .0f);
      map.insertPoint({x, y, z}, sensor);
    }
  }
}

void test_field_of_a_plane()
{
  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.truncation_voxels    = 4.0;
  map.insertionOptions.min_weight_for_query = 1.0;

  fill_ground_plane(map);
  ASSERT_(!map.isEmpty());

  // Above the plane (free space, sensor side) the field must be positive and
  // equal to the height; below it, negative. Samples are stored as distances
  // along their own ray, so the field is a projective distance and the metric
  // one is recovered by dividing by the gradient norm.
  for (const float zq : {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f})
  {
    const auto q = map.queryField({0.3f, -0.2f, zq});
    ASSERTMSG_(q.valid, mrpt::format("field query invalid at z=%.2f", zq));

    // The gradient must be the surface normal, pointing up (towards the
    // sensor), and of norm 1/cos(incidence), which is close to 1 here.
    const float gn = std::sqrt(
        q.gradient.x * q.gradient.x + q.gradient.y * q.gradient.y + q.gradient.z * q.gradient.z);
    ASSERTMSG_(gn > 0.9f && gn < 1.2f, mrpt::format("gradient norm %.4f out of range", gn));
    ASSERT_(q.gradient.z / gn > 0.99f);

    const float metricDist = q.dist / gn;
    ASSERTMSG_(
        std::abs(metricDist - zq) < QUERY_TOLERANC,
        mrpt::format("field %.4f != %.4f at z=%.2f", metricDist, zq, zq));
  }

  std::cout << "test_field_of_a_plane: OK" << std::endl;
}

/** The zero level set is what registration is steered by, so a systematic
 *  offset in it is a range bias on every scan. It is produced by off-axis
 *  samples, whose along-ray distance over-estimates the distance to the
 *  surface, so it grows with the ray tube radius: guard the shipped default.
 */
double surface_offset(float tubeSigmaVoxels)
{
  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.truncation_voxels = 4.0;
  map.insertionOptions.tube_sigma_voxels = tubeSigmaVoxels;
  map.insertionOptions.max_weight        = 1e9;

  fill_ground_plane(map);

  const auto q = map.queryField({0.3f, -0.2f, .0f});
  ASSERT_(q.valid);
  const float gn = std::sqrt(
      q.gradient.x * q.gradient.x + q.gradient.y * q.gradient.y + q.gradient.z * q.gradient.z);
  ASSERT_GT_(gn, 1e-3f);

  // Height of the estimated surface under a query sitting on the true plane:
  return -q.dist / gn;
}

/** The zero level set is what registration is steered by, so a systematic
 *  offset in it is a range bias on every scan. It is produced by the samples
 *  the ray passes to one side of, whose along-ray distance over-estimates the
 *  distance to the surface, so it is controlled by the tube profile. Guard both
 *  the shipped default and the fact that the knob still works.
 */
void test_surface_is_not_displaced()
{
  const double atDefault = surface_offset(mola::TSDF::TInsertionOptions().tube_sigma_voxels);
  const double atNarrow  = surface_offset(0.15);

  std::cout << "test_surface_is_not_displaced: default = " << atDefault * 1000
            << " mm, narrow profile = " << atNarrow * 1000 << " mm" << std::endl;

  ASSERTMSG_(
      std::abs(atDefault) < 0.015,
      mrpt::format("default profile displaces the surface by %.2f mm", atDefault * 1000));

  // Narrowing the profile has to remove most of it, or the knob is not live:
  ASSERTMSG_(
      std::abs(atNarrow) < 0.003,
      mrpt::format("narrow profile still displaces the surface by %.2f mm", atNarrow * 1000));
  ASSERT_LT_(std::abs(atNarrow), std::abs(atDefault));
}

/** The gradient the matcher uses as a surface normal is the analytical
 *  gradient of the trilinear interpolant, not a difference of neighbors, so it
 *  is worth checking against finite differences of the field itself: a sign or
 *  a scale error there would tilt every plane the solver is given, and would
 *  not show up on a horizontal plane where two of the three components are
 *  zero.
 */
void test_gradient_against_finite_differences()
{
  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.truncation_voxels = 4.0;

  // A tilted plane, so all three gradient components are exercised:
  // z = 0.3*x - 0.2*y, seen from above.
  const mrpt::math::TPoint3Df sensor(.0f, .0f, 4.0f);
  for (float x = -PLANE_SPAN; x <= PLANE_SPAN; x += PLANE_STEP)
  {
    for (float y = -PLANE_SPAN; y <= PLANE_SPAN; y += PLANE_STEP)
    {
      map.insertPoint({x, y, 0.3f * x - 0.2f * y}, sensor);
    }
  }

  constexpr float H = 0.02f;
  size_t          n = 0;
  for (const float qx : {-1.0f, 0.4f, 1.3f})
  {
    for (const float qy : {-0.7f, 0.6f})
    {
      const mrpt::math::TPoint3Df q{qx, qy, 0.3f * qx - 0.2f * qy + 0.2f};
      const auto                  f = map.queryField(q);
      if (!f.valid)
      {
        continue;
      }
      const std::array<mrpt::math::TVector3Df, 3> axes = {
          mrpt::math::TVector3Df{H, 0, 0}, mrpt::math::TVector3Df{0, H, 0},
          mrpt::math::TVector3Df{0, 0, H}};
      const std::array<float, 3> analytic = {f.gradient.x, f.gradient.y, f.gradient.z};

      for (int k = 0; k < 3; k++)
      {
        const auto& d  = axes[k];
        const auto  fp = map.queryField({q.x + d.x, q.y + d.y, q.z + d.z});
        const auto  fm = map.queryField({q.x - d.x, q.y - d.y, q.z - d.z});
        if (!fp.valid || !fm.valid)
        {
          continue;
        }
        const float numeric = (fp.dist - fm.dist) / (2 * H);
        ASSERTMSG_(
            std::abs(numeric - analytic[k]) < 0.05f,
            mrpt::format("gradient[%i] analytic %.4f != numeric %.4f", k, analytic[k], numeric));
        n++;
      }
    }
  }
  ASSERT_GT_(n, 10U);
  std::cout << "test_gradient_against_finite_differences: OK (" << n << " checks)" << std::endl;
}

void test_pt2pl_pairing_of_a_plane()
{
  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.truncation_voxels = 4.0;

  fill_ground_plane(map);

  // A query point 0.3 m above the ground must pair against a plane at z=0
  // whose normal is vertical, and report 0.3 m of distance.
  const mrpt::math::TPoint3Df query(0.5f, 0.5f, 0.3f);

  const auto r = map.nn_search_pt2pl(query, 1.0f);
  ASSERT_(r.pairing.has_value());
  ASSERTMSG_(
      std::abs(r.distance - 0.3f) < QUERY_TOLERANC,
      mrpt::format("pt2pl distance %.4f != 0.3", r.distance));

  const auto& c = r.pairing->pl_global.centroid;
  ASSERTMSG_(std::abs(c.z) < QUERY_TOLERANC, mrpt::format("plane foot z=%.4f != 0", c.z));
  ASSERTMSG_(
      std::abs(c.x - query.x) < QUERY_TOLERANC && std::abs(c.y - query.y) < QUERY_TOLERANC,
      "plane foot is not the query's own projection");

  // Beyond the truncation there is no field, hence no pairing:
  const auto rFar = map.nn_search_pt2pl({0.5f, 0.5f, 3.0f}, 5.0f);
  ASSERT_(!rFar.pairing.has_value());

  std::cout << "test_pt2pl_pairing_of_a_plane: OK" << std::endl;
}

/** The point of the map class: fusing many noisy views of one surface must
 *  estimate it better than any single measurement does.
 */
/** The solver's point-to-plane residual is signed: it is the vector
 *  -n/|n|^2 * (n.g + d), which points from the transformed query towards the
 *  plane and reverses on the far side. So the plane this map emits has to carry
 *  the field's own sign convention through to the optimizer, not just its
 *  magnitude -- a normal flipped between the two sides would leave the squared
 *  cost unchanged while sending the Gauss-Newton step the wrong way.
 *
 *  Unlike a plane fitted by PCA, whose normal sign is arbitrary, the gradient
 *  of a signed field always points towards free space. Pin that.
 */
void test_plane_sign_is_carried_through()
{
  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.truncation_voxels = 4.0;

  fill_ground_plane(map);

  // The same lateral position, once in free space above the ground and once
  // inside it below.
  for (const float zq : {0.3f, -0.3f})
  {
    const mrpt::math::TPoint3Df q(0.5f, 0.5f, zq);
    const auto                  r = map.nn_search_pt2pl(q, 1.0f);
    ASSERTMSG_(r.pairing.has_value(), mrpt::format("no pairing at z=%.2f", zq));

    const auto& pl = r.pairing->pl_global.plane;

    // The normal must point towards the sensor, i.e. up, on BOTH sides.
    const double nz = pl.coefs[2];
    ASSERTMSG_(nz > 0.9, mrpt::format("normal points down at z=%.2f (nz=%.3f)", zq, nz));

    // And the plane's own signed evaluation must reproduce the query's height,
    // with the sign, so the residual handed to the solver reverses across the
    // surface.
    const double modN = std::sqrt(
        pl.coefs[0] * pl.coefs[0] + pl.coefs[1] * pl.coefs[1] + pl.coefs[2] * pl.coefs[2]);
    const double signedDist =
        (pl.coefs[0] * q.x + pl.coefs[1] * q.y + pl.coefs[2] * q.z + pl.coefs[3]) / modN;

    ASSERTMSG_(
        std::abs(signedDist - zq) < 0.02,
        mrpt::format("signed plane distance %.4f != %.2f", signedDist, zq));
  }

  std::cout << "test_plane_sign_is_carried_through: OK" << std::endl;
}

void test_noise_averaging()
{
  constexpr float SIGMA = 0.05f;

  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.truncation_voxels = 4.0;
  map.insertionOptions.max_weight        = 1e6;  // no forgetting, for the test

  fill_ground_plane(map, .0f, SIGMA);

  // Sample the recovered zero level set over the plane and check that its
  // spread is well below the per-measurement noise.
  double sum  = 0;
  double sum2 = 0;
  size_t n    = 0;
  for (float x = -2.0f; x <= 2.0f; x += 0.13f)
  {
    for (float y = -2.0f; y <= 2.0f; y += 0.13f)
    {
      const auto q = map.queryField({x, y, 0.1f});
      if (!q.valid)
      {
        continue;
      }
      const float gn = std::sqrt(
          q.gradient.x * q.gradient.x + q.gradient.y * q.gradient.y + q.gradient.z * q.gradient.z);
      if (gn < 1e-3f)
      {
        continue;
      }
      // Height of the zero crossing under this query:
      const double zSurf = 0.1 - q.dist / gn;
      sum += zSurf;
      sum2 += zSurf * zSurf;
      n++;
    }
  }
  ASSERT_GT_(n, 100U);

  const double mean = sum / n;
  const double var  = sum2 / n - mean * mean;
  const double rms  = std::sqrt(std::max(.0, var));

  std::cout << "test_noise_averaging: n=" << n << " mean=" << mean << " rms=" << rms << std::endl;

  // The claim under test is about the SPREAD: fusing many views of one surface
  // has to estimate it far better than any single measurement does.
  ASSERTMSG_(
      rms < 0.25 * SIGMA,
      mrpt::format(
          "fused surface spread %.4f is not well below the %.4f measurement noise", rms,
          static_cast<double>(SIGMA)));

  // The mean is a different quantity, and it does not go to zero: it carries
  // the tube profile's own displacement (see test_surface_is_not_displaced),
  // which grows with the roughness of the surface being fused.
  ASSERTMSG_(std::abs(mean) < 0.03, mrpt::format("fused surface is biased: mean=%.4f", mean));
}

/** One scan of a horizontal plane z=0, as an observation, so that insertion
 *  goes through the path that drives the warm-up. `step` sets how far apart
 *  the measured points are, which is what decides whether one scan alone can
 *  fill the eight corners a trilinear query needs.
 */
mrpt::obs::CObservationPointCloud::Ptr plane_scan(float step, float zPlane = .0f)
{
  auto obs                                        = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorPose                                 = mrpt::poses::CPose3D::Identity();
  auto pts                                        = mrpt::maps::CSimplePointsMap::Create();
  pts->insertionOptions.minDistBetweenLaserPoints = .0f;

  for (float x = -PLANE_SPAN; x <= PLANE_SPAN; x += step)
  {
    for (float y = -PLANE_SPAN; y <= PLANE_SPAN; y += step)
    {
      // Coordinates in the sensor frame: the sensor sits SENSOR_HEIGHT above.
      pts->insertPointFast(x, y, zPlane - SENSOR_HEIGHT);
    }
  }
  pts->mark_as_modified();
  obs->pointcloud = pts;
  return obs;
}

const mrpt::poses::CPose3D SENSOR_POSE = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
    .0, .0, static_cast<double>(SENSOR_HEIGHT), .0, .0, .0);

/** The warm-up must produce a pairing where the field cannot yet, and the two
 *  must agree once the field can, or the switch would move the residual under
 *  the optimizer.
 */
void test_bootstrap_answers_before_the_field_can()
{
  const mrpt::math::TPoint3Df query{0.3f, -0.2f, 0.15f};

  // A scan whose points are far enough apart that one of them alone leaves
  // most cells with fewer than eight populated corners.
  auto sparseScan = plane_scan(4.0f * TEST_VOXEL);

  {
    mola::TSDF map(TEST_VOXEL);
    map.insertionOptions.ray_tube_voxels = 0.5;  // no coverage help from the tube
    map.insertObservation(*sparseScan, SENSOR_POSE);

    ASSERT_(map.isBootstrapping());
    ASSERT_(!map.queryField(query).valid);

    // ... and yet the map answers, because the warm-up cloud does.
    const auto np = map.nn_search_pt2pl(query, 1.0f);
    ASSERT_(np.pairing.has_value());
    std::cout << "test_bootstrap_answers_before_the_field_can: warm-up distance = " << np.distance
              << " m, field ready = " << map.lastFieldReadyFraction() << "\n";
    ASSERT_NEAR_(np.distance, 0.15f, 0.02f);
  }

  // With a dense scan the field is ready at once, and the two answers agree.
  {
    mola::TSDF map(TEST_VOXEL);
    map.insertionOptions.bootstrap_min_scans = 1;
    auto denseScan                           = plane_scan(0.05f);

    map.insertObservation(*denseScan, SENSOR_POSE);
    ASSERT_(!map.isBootstrapping());
    ASSERT_(map.queryField(query).valid);

    const auto fromField = map.nn_search_pt2pl(query, 1.0f);
    ASSERT_(fromField.pairing.has_value());

    mola::TSDF pointsOnly(TEST_VOXEL);
    pointsOnly.insertionOptions.bootstrap_min_scans = 100;  // never leaves the warm-up
    pointsOnly.insertionOptions.bootstrap_max_scans = 100;
    pointsOnly.insertObservation(*denseScan, SENSOR_POSE);
    ASSERT_(pointsOnly.isBootstrapping());

    const auto fromPoints = pointsOnly.nn_search_pt2pl(query, 1.0f);
    ASSERT_(fromPoints.pairing.has_value());

    std::cout << "test_bootstrap_answers_before_the_field_can: field = " << fromField.distance
              << " m, points = " << fromPoints.distance << " m\n";

    // Both must find the same surface. They are not the same estimator, so the
    // tolerance is the field's own tube displacement, not zero.
    ASSERT_NEAR_(fromField.distance, fromPoints.distance, 0.03f);
  }
}

/** A neighborhood with no plane in it must yield no pairing, not an exception.
 *
 *  On the first scans a k-NN neighborhood can easily be a single scan line, and
 *  a plane fit that throws on that case escapes the matcher and is treated by
 *  the caller as a fatal error, which stops the run outright.
 */
void test_bootstrap_survives_a_degenerate_neighborhood()
{
  auto obs                                        = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorPose                                 = mrpt::poses::CPose3D::Identity();
  auto pts                                        = mrpt::maps::CSimplePointsMap::Create();
  pts->insertionOptions.minDistBetweenLaserPoints = .0f;

  // A single straight line of points: collinear, so there is no tangent plane.
  for (int i = 0; i < 200; i++)
  {
    pts->insertPointFast(0.05f * static_cast<float>(i) - 5.0f, .0f, -SENSOR_HEIGHT);
  }
  pts->mark_as_modified();
  obs->pointcloud = pts;

  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.bootstrap_min_scans = 100;  // stay in the warm-up
  map.insertionOptions.bootstrap_max_scans = 100;
  map.insertObservation(*obs, SENSOR_POSE);
  ASSERT_(map.isBootstrapping());

  const auto np = map.nn_search_pt2pl({0.1f, 0.02f, 0.1f}, 1.0f);
  ASSERT_(!np.pairing.has_value());

  std::cout << "test_bootstrap_survives_a_degenerate_neighborhood: OK" << std::endl;
}

/** The warm-up must be bounded: a field that never becomes ready must not
 *  leave the map silently answering as a point cloud for the whole run.
 */
void test_bootstrap_is_bounded()
{
  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.ray_tube_voxels = 0.5;
  // A readiness that can never be reached, so only the bound can end it.
  map.insertionOptions.bootstrap_field_ready_fraction = 2.0;
  map.insertionOptions.bootstrap_min_scans            = 1;
  map.insertionOptions.bootstrap_max_scans            = 3;

  auto scan = plane_scan(4.0f * TEST_VOXEL);

  map.insertObservation(*scan, SENSOR_POSE);
  ASSERT_(map.isBootstrapping());
  map.insertObservation(*scan, SENSOR_POSE);
  ASSERT_(map.isBootstrapping());
  map.insertObservation(*scan, SENSOR_POSE);
  ASSERT_(!map.isBootstrapping());

  std::cout << "test_bootstrap_is_bounded: OK" << std::endl;
}

void test_serialization_roundtrip()
{
  mola::TSDF map(TEST_VOXEL);
  fill_ground_plane(map);

  mrpt::io::CMemoryStream buf;
  auto                    arch = mrpt::serialization::archiveFrom(buf);
  arch << map;

  buf.Seek(0);
  mola::TSDF map2;
  arch >> map2;

  ASSERT_EQUAL_(map.size(), map2.size());

  const auto q1 = map.queryField({0.3f, -0.2f, 0.2f});
  const auto q2 = map2.queryField({0.3f, -0.2f, 0.2f});
  ASSERT_(q1.valid && q2.valid);
  ASSERT_(std::abs(q1.dist - q2.dist) < 1e-6f);

  std::cout << "test_serialization_roundtrip: OK" << std::endl;
}

void test_pruning()
{
  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.remove_voxels_farther_than = 2.0;

  fill_ground_plane(map);
  const size_t nBefore = map.size();
  ASSERT_GT_(nBefore, 0U);

  // Inserting an observation from the origin prunes everything beyond 2 m:
  mrpt::obs::CObservationPointCloud obs;
  obs.pointcloud = mrpt::maps::CSimplePointsMap::Create();
  obs.pointcloud->insertPointFast(0.1f, 0.1f, -2.0f);
  map.insertObservation(obs, mrpt::poses::CPose3D::Identity());

  ASSERT_LT_(map.size(), nBefore);

  // ...and nothing far away survived:
  const auto qFar = map.queryField({5.0f, 5.0f, .0f});
  ASSERT_(!qFar.valid);

  std::cout << "test_pruning: OK" << std::endl;
}
/** The extent bound alone does not bound anything usefully: at a fixed extent
 *  the voxel count still grows as the inverse cube of the voxel size and with
 *  how much surface the scene puts inside it. The count ceiling has to hold
 *  whatever the extent says.
 */
void test_voxel_budget_is_a_ceiling()
{
  constexpr uint64_t CAP = 2000;

  mola::TSDF map(TEST_VOXEL);
  map.insertionOptions.truncation_voxels          = 4.0;
  map.insertionOptions.remove_voxels_farther_than = 1000.0;  // deliberately useless
  map.insertionOptions.max_voxels                 = CAP;

  fill_ground_plane(map);

  // Nothing prunes until an observation is inserted, which is where the
  // sensor pose the eviction is measured from comes from.
  mrpt::obs::CObservationPointCloud obs;
  obs.pointcloud = mrpt::maps::CSimplePointsMap::Create();
  obs.pointcloud->insertPointFast(0.1f, 0.1f, -2.0f);
  map.insertObservation(obs, mrpt::poses::CPose3D::Identity());

  std::cout << "test_voxel_budget_is_a_ceiling: " << map.size() << " voxels under a cap of " << CAP
            << std::endl;

  // Ties at the cut radius are all kept, so the cap is approached from above by
  // at most one shell; a factor of two is a generous allowance for that.
  ASSERTMSG_(
      map.size() <= 2 * CAP, mrpt::format(
                                 "voxel budget not enforced: %zu voxels against a cap of %lu",
                                 map.size(), static_cast<unsigned long>(CAP)));

  // What survives has to be what is nearest the sensor, not an arbitrary
  // subset: the field around the origin must still answer.
  const auto q = map.queryField({.0f, .0f, .0f});
  ASSERT_(q.valid);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_field_of_a_plane();
    test_surface_is_not_displaced();
    test_gradient_against_finite_differences();
    test_pt2pl_pairing_of_a_plane();
    test_plane_sign_is_carried_through();
    test_noise_averaging();
    test_bootstrap_answers_before_the_field_can();
    test_bootstrap_is_bounded();
    test_bootstrap_survives_a_degenerate_neighborhood();
    test_serialization_roundtrip();
    test_pruning();
    test_voxel_budget_is_a_ceiling();
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
  return 0;
}
