/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   IMUIntegrationParams.h
 * @brief  Parameters for IMU preintegration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */

#pragma once

#include <mola-imu-preintegration/RotationIntegrationParams.h>
#include <mola-kernel/Yaml.h>
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
    void loadFrom(const Yaml& cfg);

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
