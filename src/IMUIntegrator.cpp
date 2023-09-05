/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   IMUIntegrator.cpp
 * @brief  Integrator of IMU accelerations and angular velocity readings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */

/** \defgroup mola_imu_preintegration_grp mola-imu-preintegration
 * This repository provides:
 * - mola::IMUIntegrator and mola::RotationIntegrator: C++ classes to integrate
 * IMU accelerations and angular velocities.
 * - mola::IMUIntegrationPublisher: A MOLA module to process CObservationIMU
 * readings and emit relative pose updates with respect to the last keyframe.
 */

#include <mola_imu_preintegration/IMUIntegrator.h>
//#include <mrpt/containers/yaml.h>
//#include <mrpt/core/initializer.h>

using namespace mola;

void IMUIntegrator::initialize(const mrpt::containers::yaml& cfg)
{
    //
}
