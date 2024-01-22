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
 * @file   NavStateFuseParams.h
 * @brief  Parameters for NavStateFuse
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>

namespace mola
{
/** Parameters needed by NavStateFuse.
 *
 * \ingroup mola_imu_preintegration_grp
 */
class NavStateFuseParams
{
   public:
    NavStateFuseParams()  = default;
    ~NavStateFuseParams() = default;

    /// Loads all parameters from a YAML map node.
    void loadFrom(const mrpt::containers::yaml& cfg);

    /** Valid estimations will be extrapolated only up to this time since the
     * last incorporated observation. */
    double max_time_to_use_velocity_model = 2.0;  // [s]

    double sigma_random_walk_acceleration_linear  = 1.0;  // [m/s²]
    double sigma_random_walk_acceleration_angular = 1.0;  // [rad/s²]
};

}  // namespace mola
