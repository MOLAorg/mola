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
 * @file   IMUIntegrationParams.h
 * @brief  Parameters for IMU preintegration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */

#pragma once

#include <mola_imu_preintegration/RotationIntegrationParams.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>

namespace mola
{
/** Parameters needed by IMU preintegration classes, when integrating both,
 * acceleration and rotation.
 *
 *  Refer to:
 *  - Crassidis, J. L. (2006). Sigma-point Kalman filtering for integrated GPS
 * and inertial navigation. IEEE Transactions on Aerospace and Electronic
 * Systems, 42(2), 750-756.
 *  - Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2015). IMU
 * preintegration on manifold for efficient visual-inertial maximum-a-posteriori
 * estimation. Georgia Institute of Technology.
 *  - Nikolic, J. (2016). Characterisation, calibration, and design of
 * visual-inertial sensor systems for robot navigation (Doctoral dissertation,
 * ETH Zurich).
 *
 * \ingroup mola_imu_preintegration_grp
 */
class IMUIntegrationParams
{
   public:
    IMUIntegrationParams()  = default;
    ~IMUIntegrationParams() = default;

    /// Loads all parameters from a YAML map node.
    void loadFrom(const mrpt::containers::yaml& cfg);

    /// Parameters for gyroscope integration:
    RotationIntegrationParams rotationParams;

    /// Gravity vector (units are m/s²), in the global frame of coordinates.
    mrpt::math::TVector3D gravityVector = {0, 0, -9.81};

    /// Accelerometer covariance (units of sigma are m/s²/√Hz )
    mrpt::math::CMatrixDouble33 accCov =
        mrpt::math::CMatrixDouble33::Identity();

    /// Integration covariance: jerk, that is, how much acceleration can change
    /// over time:
    mrpt::math::CMatrixDouble33 integrationCov =
        mrpt::math::CMatrixDouble33::Identity();
};

}  // namespace mola
