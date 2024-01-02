/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
 * @file   test-mola_metric_maps_hashedvoxels.cpp
 * @brief  Test the voxel map class
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/opengl/Scene.h>

#include <iostream>

void test_voxelmap_basic_ops()
{
    mola::HashedVoxelPointCloud map;
    ASSERT_(map.isEmpty());

    map.insertPoint({1.0f, 2.0f, 3.0f});
    ASSERT_(!map.isEmpty());
}

void test_voxelmap_insert_2d_scan()
{
    mola::HashedVoxelPointCloud map(0.2 /*voxel size*/);

    mrpt::obs::CObservation2DRangeScan scan2D;
    mrpt::obs::stock_observations::example2DRangeScan(scan2D);

    map.insertObservation(scan2D);
#if 0
    mrpt::opengl::Scene scene;
    map.renderOptions.point_size = 5.0f;
    scene.insert(map.getVisualization());
    scene.saveToFile("sparsevoxelmap_scan2d.3Dscene");
#endif

    {
        size_t nPts = 0;

        const auto lambdaVisitPoints = [&nPts](const mrpt::math::TPoint3Df&) {
            nPts++;
        };

        map.visitAllPoints(lambdaVisitPoints);

        ASSERT_EQUAL_(nPts, 267UL);
    }

    // test NN search:
    mrpt::maps::CSimplePointsMap refPts;
    map.visitAllPoints(
        [&](const mrpt::math::TPoint3Df& pt) { refPts.insertPoint(pt); });

    mrpt::maps::CSimplePointsMap queryPts;
    queryPts.insertObservation(
        scan2D, mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                    -0.05, -0.01, -0.01, 0, 0, 0));

    for (size_t knn = 2; knn < 3; knn++)
    {
        for (size_t i = 0; i < queryPts.size(); i++)
        {
            mrpt::math::TPoint3D query;
            queryPts.getPoint(i, query);

            std::vector<mrpt::math::TPoint3Df> results;
            std::vector<float>                 dists_sqr;
            std::vector<uint64_t>              IDs;
            map.nn_multiple_search(query, knn, results, dists_sqr, IDs);

            std::vector<mrpt::math::TPoint3Df> gt_results;
            std::vector<float>                 gt_dists_sqr;
            std::vector<uint64_t>              gt_IDs;
            refPts.nn_multiple_search(
                query, knn, gt_results, gt_dists_sqr, gt_IDs);

#if 0
            for (size_t k = 0; k < results.size(); k++)
            {
                std::cout << "k: " << k << "/" << knn << " query: " << query
                          << " nn: " << results.at(k) << " ("
                          << std::sqrt(dists_sqr.at(k)) << ") "
                          << " gt: " << gt_results.at(k) << " ("
                          << std::sqrt(gt_dists_sqr.at(k)) << ")" << std::endl;
            }
#endif
            for (size_t k = 0; k < results.size(); k++)
            {
                ASSERT_NEAR_(results[k].x, gt_results[k].x, 1e-3f);
                ASSERT_NEAR_(results[k].y, gt_results[k].y, 1e-3f);
                ASSERT_NEAR_(results[k].z, gt_results[k].z, 1e-3f);

                ASSERT_NEAR_(dists_sqr[k], gt_dists_sqr[k], 1e-3f);
            }
        }
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
