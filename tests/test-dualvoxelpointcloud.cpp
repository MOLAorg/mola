/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-dualvoxelpointcloud.cpp
 * @brief  Test the OccGrid class
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */

#include <mola_metric_maps/DualVoxelPointCloud.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/opengl/Scene.h>

#include <iostream>

void test_voxelmap_basic_ops()
{
    mola::DualVoxelPointCloud map;
    ASSERT_(map.isEmpty());

    map.insertPoint({1.0f, 2.0f, 3.0f});
    ASSERT_(!map.isEmpty());
}

void test_voxelmap_insert_2d_scan()
{
    mola::DualVoxelPointCloud map(0.2 /*decim*/);

    mrpt::obs::CObservation2DRangeScan scan2D;
    mrpt::obs::stock_observations::example2DRangeScan(scan2D);

    map.insertObservation(scan2D);
#if 1
    mrpt::opengl::Scene scene;
    map.renderOptions.show_inner_grid_boxes = true;
    map.renderOptions.show_mean_only        = true;
    map.renderOptions.point_size            = 5.0f;
    scene.insert(map.getVisualization());
    scene.saveToFile("dualvoxelmap_scan2d.3Dscene");
#endif
    std::cout << "numGrids: " << map.grids().size() << std::endl;

    {
        size_t nPts = 0;

        const auto lambdaVisitPoints = [&nPts](const mrpt::math::TPoint3Df&) {
            nPts++;
        };

        map.visitAllPoints(lambdaVisitPoints);

        ASSERT_EQUAL_(nPts, 267UL);
    }

    {
        size_t nVoxels = 0;

        const auto lambdaVisitVoxels =
            [&nVoxels](
                const mola::DualVoxelPointCloud::outer_index3d_t&,
                const mola::DualVoxelPointCloud::inner_plain_index_t,
                const mola::DualVoxelPointCloud::VoxelData& v) {
                // count them:
                if (!v.points().empty()) nVoxels++;
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
