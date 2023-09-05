/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   IMUIntegrator.h
 * @brief  Integrator of IMU accelerations and angular velocity readings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>

namespace mola
{
/** Integrates accelerometer and gyroscope readings.
 *
 * See IMUIntegrationParams for a list of related papers explaining the methods
 * and parameters.
 *
 * \ingroup mola_imu_preintegration_grp
 */
class IMUIntegrator
{
   public:
    IMUIntegrator()  = default;
    ~IMUIntegrator() = default;

    /**
     * @brief initialize
     * @param cfg
     */
    void initialize(const mrpt::containers::yaml& cfg);
    //    void spinOnce() override;

   private:
};

}  // namespace mola
