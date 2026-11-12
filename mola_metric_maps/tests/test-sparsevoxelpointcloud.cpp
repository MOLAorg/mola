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
 * @file   test-sparsevoxelpointcloud.cpp
 * @brief  Test the voxel map class
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */

#include <mola_metric_maps/SparseVoxelPointCloud.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/viz/Scene.h>

#include <iostream>
#include <memory>

void test_voxelmap_basic_ops()
{
  mola::SparseVoxelPointCloud map;
  ASSERT_(map.isEmpty());

  map.insertPoint({1.0f, 2.0f, 3.0f});
  ASSERT_(!map.isEmpty());
}

void test_voxelmap_insert_2d_scan()
{
  mola::SparseVoxelPointCloud map(0.2 /*decim*/);

  // The numbers in this test would change if this param changes:
  static_assert(mola::SparseVoxelPointCloud::HARDLIMIT_MAX_POINTS_PER_VOXEL == 16);

  mrpt::obs::CObservation2DRangeScan scan2D;
  mrpt::obs::stock_observations::example2DRangeScan(scan2D);

  map.insertObservation(scan2D);
#if 0
    mrpt::viz::Scene scene;
    map.renderOptions.show_inner_grid_boxes = true;
    map.renderOptions.show_mean_only        = true;
    map.renderOptions.point_size            = 5.0f;
    scene.insert(map.getVisualization());
    scene.saveToFile("sparsevoxelmap_scan2d.3Dscene");
#endif

  {
    size_t nPts = 0;

    const auto lambdaVisitPoints = [&nPts](const mrpt::math::TPoint3Df&) { nPts++; };

    map.visitAllPoints(lambdaVisitPoints);

    ASSERT_EQUAL_(nPts, 267UL);
  }

  {
    size_t nVoxels = 0;

    const auto lambdaVisitVoxels = [&nVoxels](
                                       const mola::SparseVoxelPointCloud::outer_index3d_t&,
                                       const mola::SparseVoxelPointCloud::inner_plain_index_t,
                                       const mola::SparseVoxelPointCloud::VoxelData& v,
                                       const mola::SparseVoxelPointCloud::InnerGrid& grid)
    {
      // count them:
      if (!v.points(grid).empty()) nVoxels++;
    };
    map.visitAllVoxels(lambdaVisitVoxels);

    ASSERT_EQUAL_(nVoxels, 96UL);
  }
}

size_t count_points(const mola::SparseVoxelPointCloud& map)
{
  size_t nPts = 0;
  map.visitAllPoints([&nPts](const mrpt::math::TPoint3Df&) { nPts++; });
  return nPts;
}

// Copies must be independent of the source map, which may be gone:
void test_voxelmap_copies()
{
  auto original = std::make_unique<mola::SparseVoxelPointCloud>(0.2f);
  original->insertPoint({0.05f, 0.05f, 0.05f});

  mola::SparseVoxelPointCloud copy(*original);
  mola::SparseVoxelPointCloud assigned(0.2f);
  assigned = *original;
  original.reset();

  copy.insertPoint({0.06f, 0.05f, 0.05f});
  ASSERT_EQUAL_(count_points(copy), 2UL);

  assigned.insertPoint({0.06f, 0.05f, 0.05f});
  assigned.insertPoint({0.07f, 0.05f, 0.05f});
  ASSERT_EQUAL_(count_points(assigned), 3UL);
  ASSERT_EQUAL_(count_points(copy), 2UL);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_voxelmap_basic_ops();
    test_voxelmap_insert_2d_scan();
    test_voxelmap_copies();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
