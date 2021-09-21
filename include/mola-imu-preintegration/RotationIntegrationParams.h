/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RotationIntegrationParams.h
 * @brief  Parameters for angular velocity integration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */

#pragma once

#include <mola-kernel/Yaml.h>
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
    void load_from(const Yaml& cfg);

    /// Gyroscope (initial or constant) bias, in the local IMU frame of
    /// reference (units: rad/s).
    mrpt::math::TVector3D gyroBias = {.0, .0, .0};

    /// Gyroscope covariance (units of sigma are rad/s/√Hz )
    mrpt::math::CMatrixDouble33 gyroCov =
        mrpt::math::CMatrixDouble33::Identity();

    /// If provided, defines an IMU placed at a pose different than the
    /// vehicle origin of coordinates (Default: IMU used as reference of the
    /// vehicle frame).
    std::optional<mrpt::poses::CPose3D> sensorLocationInVehicle;
};

}  // namespace mola
