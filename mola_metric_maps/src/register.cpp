/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

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
#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mola_metric_maps/NDT.h>
#include <mola_metric_maps/OccGrid.h>
#include <mola_metric_maps/SparseTreesPointCloud.h>
#include <mola_metric_maps/SparseVoxelPointCloud.h>
#include <mrpt/core/initializer.h>
#include <mrpt/rtti/CObject.h>

using namespace mola;

MRPT_INITIALIZER(do_register_mola_metric_maps)  // NOLINT(misc-use-anonymous-namespace)
{
  using mrpt::rtti::registerClass;

  // and register RTTI info:
  registerClass(CLASS_ID(mola::HashedVoxelPointCloud));
  registerClass(CLASS_ID(mola::KeyframePointCloudMap));
  registerClass(CLASS_ID(mola::NDT));
  registerClass(CLASS_ID(mola::OccGrid));
  registerClass(CLASS_ID(mola::SparseTreesPointCloud));
  registerClass(CLASS_ID(mola::SparseVoxelPointCloud));
}
