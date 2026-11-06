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
 * @file   test-voxelmaps-likelihood.cpp
 * @brief  Observation likelihood of the voxel maps: unmatched points and empty
 *         maps score as the worst match.
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mola_metric_maps/SparseTreesPointCloud.h>
#include <mola_metric_maps/SparseVoxelPointCloud.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
/// Points of a 5x5 m plane at z=0, 0.1 m apart, shifted by `dx` in x.
mrpt::obs::CObservationPointCloud planeScan(double dx)
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  for (int i = 0; i < 50; i++)
  {
    for (int j = 0; j < 50; j++)
    {
      pts->insertPoint(static_cast<float>(i * 0.1 + dx), static_cast<float>(j * 0.1), 0.0f);
    }
  }
  mrpt::obs::CObservationPointCloud obs;
  obs.pointcloud = pts;
  return obs;
}

template <class MAP>
void test_map(const std::string& name)
{
  const auto full  = planeScan(0.0);
  const auto other = planeScan(100.0);  // nowhere near the map

  // Half the points match, half do not:
  mrpt::obs::CObservationPointCloud half = planeScan(0.0);
  {
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    for (size_t k = 0; k < full.pointcloud->size(); k++)
    {
      mrpt::math::TPoint3Df p;
      full.pointcloud->getPoint(k, p.x, p.y, p.z);
      if (k % 2 == 1)
      {
        p.x += 100.0f;
      }
      pts->insertPoint(p);
    }
    half.pointcloud = pts;
  }

  MAP empty;
  MAP map;
  // Every point counts, so the half-matching scan is really half matching:
  empty.likelihoodOptions.decimation = 1;
  map.likelihoodOptions.decimation   = 1;
  for (size_t k = 0; k < full.pointcloud->size(); k++)
  {
    mrpt::math::TPoint3Df p;
    full.pointcloud->getPoint(k, p.x, p.y, p.z);
    map.insertPoint(p);
  }

  const mrpt::poses::CPose3D origin;
  const double               lFull  = map.computeObservationLikelihood(full, origin);
  const double               lHalf  = map.computeObservationLikelihood(half, origin);
  const double               lNone  = map.computeObservationLikelihood(other, origin);
  const double               lEmpty = empty.computeObservationLikelihood(full, origin);

  std::cout << "[" << name << "] full=" << lFull << " half=" << lHalf << " none=" << lNone
            << " empty map=" << lEmpty << "\n";

  ASSERT_(std::isfinite(lFull) && std::isfinite(lHalf) && std::isfinite(lNone));
  ASSERT_GT_(lFull, lHalf);
  ASSERT_GT_(lHalf, lNone);
  // An empty map must not beat a partially matching one:
  ASSERT_LT_(lEmpty, lHalf);
  ASSERT_NEAR_(lEmpty, lNone, 1e-9);
}
}  // namespace

int main()
{
  try
  {
    test_map<mola::HashedVoxelPointCloud>("HashedVoxelPointCloud");
    test_map<mola::SparseTreesPointCloud>("SparseTreesPointCloud");
    test_map<mola::SparseVoxelPointCloud>("SparseVoxelPointCloud");
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
  return 0;
}
