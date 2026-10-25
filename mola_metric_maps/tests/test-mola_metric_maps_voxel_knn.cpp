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
 * @file   test-mola_metric_maps_voxel_knn.cpp
 * @brief  k-NN and radius searches of HashedVoxelPointCloud and NDT against
 *         brute force, for any k.
 * @author Jose Luis Blanco Claraco
 * @date   Oct 24, 2026
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mola_metric_maps/NDT.h>

#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <vector>

namespace
{
const mrpt::math::TPoint3Df SENSOR(0, 0, 0);

void insert(mola::HashedVoxelPointCloud& m, const mrpt::math::TPoint3Df& p) { m.insertPoint(p); }
void insert(mola::NDT& m, const mrpt::math::TPoint3Df& p) { m.insertPoint(p, SENSOR); }

template <class MAP>
std::vector<mrpt::math::TPoint3Df> storedPoints(const MAP& map)
{
  std::vector<mrpt::math::TPoint3Df> pts;
  map.visitAllPoints([&](const mrpt::math::TPoint3Df& p) { pts.push_back(p); });
  return pts;
}

/// Squared distances from `query` to all `pts`, ascending.
std::vector<float> sortedSqrDists(
    const std::vector<mrpt::math::TPoint3Df>& pts, const mrpt::math::TPoint3Df& query)
{
  std::vector<float> d;
  for (const auto& p : pts)
  {
    d.push_back((p - query).sqrNorm());
  }
  std::sort(d.begin(), d.end());
  return d;
}

/// The results are `expected.size()` distinct points, at the expected
/// distances, closest first.
void checkResults(
    const mrpt::math::TPoint3Df& query, const std::vector<float>& expected,
    const std::vector<mrpt::math::TPoint3Df>& pts, const std::vector<float>& sqrDists,
    const std::vector<uint64_t>& ids)
{
  ASSERT_EQUAL_(pts.size(), expected.size());
  ASSERT_EQUAL_(sqrDists.size(), expected.size());
  ASSERT_EQUAL_(ids.size(), expected.size());
  ASSERT_EQUAL_(std::set<uint64_t>(ids.begin(), ids.end()).size(), ids.size());
  for (size_t k = 0; k < pts.size(); k++)
  {
    ASSERT_NEAR_(sqrDists[k], expected[k], 1e-6f);
    ASSERT_NEAR_(sqrDists[k], (pts[k] - query).sqrNorm(), 1e-6f);
  }
}

/// Points inserted in decreasing distance to the query, so that each one goes
/// to the front of the list: shifting the rest from the front would copy the
/// same match over all of them.
template <class MAP>
void test_closer_points_last()
{
  MAP map(1.0f);
  for (const float x : {0.9f, 0.7f, 0.5f, 0.3f, 0.1f})
  {
    insert(map, {x, 0.5f, 0.5f});
  }
  const mrpt::math::TPoint3Df query(0.0f, 0.5f, 0.5f);

  std::vector<mrpt::math::TPoint3Df> pts;
  std::vector<float>                 sqrDists;
  std::vector<uint64_t>              ids;
  map.nn_multiple_search(query, 4, pts, sqrDists, ids);
  checkResults(query, {0.01f, 0.09f, 0.25f, 0.49f}, pts, sqrDists, ids);

  map.nn_radius_search(query, 1.0f, pts, sqrDists, ids, 4);
  checkResults(query, {0.01f, 0.09f, 0.25f, 0.49f}, pts, sqrDists, ids);
}

/// Random points and queries in [0,2)^3 with unit voxels: the voxels around
/// any query then hold every point, so the k-NN result is exact.
template <class MAP>
void test_against_brute_force()
{
  std::mt19937                          rng(1234);
  std::uniform_real_distribution<float> uniform(0.0f, 1.999f);
  auto                                  randomPoint = [&]()
  { return mrpt::math::TPoint3Df(uniform(rng), uniform(rng), uniform(rng)); };

  MAP map(1.0f);
  for (int i = 0; i < 100; i++)
  {
    insert(map, randomPoint());
  }
  const auto stored = storedPoints(map);
  ASSERT_GT_(stored.size(), 40UL);

  for (int q = 0; q < 50; q++)
  {
    const auto query = randomPoint();
    const auto all   = sortedSqrDists(stored, query);

    for (const size_t k : {1UL, 2UL, 3UL, 4UL, 8UL, 20UL, 40UL})
    {
      const std::vector<float> expected(all.begin(), all.begin() + k);

      std::vector<mrpt::math::TPoint3Df> pts;
      std::vector<float>                 sqrDists;
      std::vector<uint64_t>              ids;

      map.nn_multiple_search(query, k, pts, sqrDists, ids);
      checkResults(query, expected, pts, sqrDists, ids);

      // A radius covering everything, capped at k:
      map.nn_radius_search(query, 100.0f, pts, sqrDists, ids, k);
      checkResults(query, expected, pts, sqrDists, ids);
    }

    // Uncapped: every point within the radius, in any order.
    const float                        radiusSqr = 0.5f;
    std::vector<mrpt::math::TPoint3Df> pts;
    std::vector<float>                 sqrDists;
    std::vector<uint64_t>              ids;
    map.nn_radius_search(query, radiusSqr, pts, sqrDists, ids, 0);
    std::sort(sqrDists.begin(), sqrDists.end());
    const auto inRadius =
        static_cast<size_t>(std::upper_bound(all.begin(), all.end(), radiusSqr) - all.begin());
    ASSERT_EQUAL_(sqrDists.size(), inRadius);
  }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_closer_points_last<mola::HashedVoxelPointCloud>();
    test_against_brute_force<mola::HashedVoxelPointCloud>();
    std::cout << "[voxel_knn] HashedVoxelPointCloud: OK\n";

    test_closer_points_last<mola::NDT>();
    test_against_brute_force<mola::NDT>();
    std::cout << "[voxel_knn] NDT: OK\n";
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
