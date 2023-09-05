/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   IMUIntegrationPublisher.cpp
 * @brief  A mola node that takes IMU observations, integrate them since the
 * last KF, and publishes the preintegrated rotation.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 18, 2021
 */

#include <mola_imu_preintegration/IMUIntegrationPublisher.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(IMUIntegrationPublisher, ExecutableBase, mola)

void IMUIntegrationPublisher::initialize(const mrpt::containers::yaml& cfg)
{
    //
}

void IMUIntegrationPublisher::onNewObservation(CObservation::Ptr& o)
{
    //
}

void IMUIntegrationPublisher::spinOnce()
{
    //
}
