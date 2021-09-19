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

#include <mola-imu-preintegration/IMUIntegrationPublisher.h>
//#include <mrpt/containers/yaml.h>
//#include <mrpt/core/initializer.h>

using namespace mola;

void IMUIntegrationPublisher::initialize(const std::string& cfg_block)
{
    //
}

void IMUIntegrationPublisher::onNewObservation(CObservation::Ptr& o)
{
    //
}
