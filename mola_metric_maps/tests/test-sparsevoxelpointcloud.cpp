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
#include <mrpt/opengl/Scene.h>

#include <iostream>

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
    mrpt::opengl::Scene scene;
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

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_voxelmap_basic_ops();
    test_voxelmap_insert_2d_scan();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
