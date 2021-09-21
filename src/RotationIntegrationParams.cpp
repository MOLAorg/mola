/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RotationIntegrationParams.cpp
 * @brief  Parameters for angular velocity integration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */

#include <mola-imu-preintegration/RotationIntegrationParams.h>

using namespace mola;

void RotationIntegrationParams::load_from(const Yaml& cfg)
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
