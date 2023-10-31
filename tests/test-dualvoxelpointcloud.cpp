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

#include <iostream>

void test_voxelmap_basic_ops()
{
    mola::DualVoxelPointCloud map;
    ASSERT_(map.isEmpty());

    map.insertPoint({1.0f, 2.0f, 3.0f});
    ASSERT_(!map.isEmpty());

    ASSERT_EQUAL_(map.voxels().size(), 1UL);
    ASSERT_EQUAL_(map.voxels().begin()->second.points().size(), 1UL);

    const auto& p = map.voxels().begin()->second.points().at(0);
    ASSERT_EQUAL_(p.x, 1.0f);
    ASSERT_EQUAL_(p.y, 2.0f);
    ASSERT_EQUAL_(p.z, 3.0f);

    // NN:
    {
        mrpt::math::TPoint3Df pt;

        float      ptSqrDist = 0;
        const bool nn_ok =
            map.nn_find_nearest({1.10f, 1.9f, 2.9f}, pt, ptSqrDist);

        ASSERT_(nn_ok);
        ASSERT_EQUAL_(pt.x, 1.0f);
        ASSERT_EQUAL_(pt.y, 2.0f);
        ASSERT_EQUAL_(pt.z, 3.0f);
    }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_voxelmap_basic_ops();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
