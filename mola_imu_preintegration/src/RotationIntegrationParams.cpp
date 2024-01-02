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
 * @file   RotationIntegrationParams.cpp
 * @brief  Parameters for angular velocity integration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */

#include <mola_imu_preintegration/RotationIntegrationParams.h>

using namespace mola;

void RotationIntegrationParams::load_from(const mrpt::containers::yaml& cfg)
{
    gyroBias = mrpt::math::TVector3D::FromVector(
        cfg["gyroBias"].toStdVector<double>());

    const auto poseQuat =
        cfg["sensorLocationInVehicle"]["quaternion"].toStdVector<double>();
    const auto poseTrans =
        cfg["sensorLocationInVehicle"]["translation"].toStdVector<double>();
    ASSERT_EQUAL_(poseQuat.size(), 4U);
    ASSERT_EQUAL_(poseTrans.size(), 3U);

    auto pose = mrpt::poses::CPose3D::FromQuaternionAndTranslation(
        mrpt::math::CQuaternionDouble(
            poseQuat[3], poseQuat[0], poseQuat[1], poseQuat[2]),
        mrpt::math::TPoint3D::FromVector(poseTrans));

    if (pose != mrpt::poses::CPose3D::Identity())
    {
        // Store:
        sensorPose = pose;
    }
    else
    {
        // Leave as unasigned to reflect it's just I_{4,4}
    }
}
