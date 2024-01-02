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
 * @file   register.cpp
 * @brief  Register MOLA modules in the factory
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2018
 */

/** \defgroup mola_metric_maps_grp mola_metric_maps
 * C++ library: Advanced metric map classes
 *
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mola_metric_maps/OccGrid.h>
#include <mola_metric_maps/SparseTreesPointCloud.h>
#include <mola_metric_maps/SparseVoxelPointCloud.h>
#include <mrpt/core/initializer.h>
#include <mrpt/rtti/CObject.h>

using namespace mola;

MRPT_INITIALIZER(do_register_mola_metric_maps)
{
    using mrpt::rtti::registerClass;

    // and register RTTI info:
    registerClass(CLASS_ID(mola::OccGrid));
    registerClass(CLASS_ID(mola::SparseVoxelPointCloud));
    registerClass(CLASS_ID(mola::SparseTreesPointCloud));
    registerClass(CLASS_ID(mola::HashedVoxelPointCloud));
}
