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
 * @file   RotationIntegrationParams.h
 * @brief  Parameters for angular velocity integration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <optional>

namespace mola
{
/** Parameters needed when integrating IMU rotation only.
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
class RotationIntegrationParams
{
   public:
    RotationIntegrationParams()  = default;
    ~RotationIntegrationParams() = default;

    /// Loads all parameters from a YAML map node.
    void load_from(const mrpt::containers::yaml& cfg);

    /// Gyroscope (initial or constant) bias, in the local IMU frame of
    /// reference (units: rad/s).
    mrpt::math::TVector3D gyroBias = {.0, .0, .0};

    /// Gyroscope covariance (units of sigma are rad/s/√Hz )
    mrpt::math::CMatrixDouble33 gyroCov =
        mrpt::math::CMatrixDouble33::Identity();

    /// If provided, defines an IMU placed at a pose different than the
    /// vehicle origin of coordinates (Default: IMU used as reference of the
    /// vehicle frame, i.e. sensorPose = SE(3) identity I_{4x4}).
    std::optional<mrpt::poses::CPose3D> sensorPose;
};

}  // namespace mola
