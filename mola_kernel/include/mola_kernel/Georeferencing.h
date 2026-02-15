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
 * @file   Georeferencing.h
 * @brief  Defines a common georeferencing data structure, similar to mp2p_icp
 * but independent
 * @author Jose Luis Blanco Claraco
 * @date   Jan 31, 2025
 */
#pragma once

#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/topography/data_types.h>

namespace mola
{
/** Defines a common georeferencing data structure, similar to mp2p_icp but
 * independent of that library.
 *
 * \ingroup mola_kernel_grp */
struct Georeferencing
{
  Georeferencing() = default;

  /** The geodetic coordinates (on WGS-84) of the metric map ENU frame of
   * reference. */
  mrpt::topography::TGeodeticCoords geo_coord;

  /** The SE(3) transformation from the ENU (earth-north-up) frame
   * to the metric map local frame of reference.
   * If this is the identity (default) it means the map is already in
   * ENU coordinates (i.e. +X is East, +Y is North, +Z is up) and
   * the point (0,0,0) is the one having the geodetic coordinates
   * geo_coord
   */
  mrpt::poses::CPose3DPDFGaussian T_enu_to_map;
};
}  // namespace mola
