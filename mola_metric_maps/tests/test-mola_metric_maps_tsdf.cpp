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
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/serialization/CArchive.h>

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
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_field_of_a_plane();
    test_surface_is_not_displaced();
    test_pt2pl_pairing_of_a_plane();
    test_noise_averaging();
    test_serialization_roundtrip();
    test_pruning();
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
  return 0;
}
