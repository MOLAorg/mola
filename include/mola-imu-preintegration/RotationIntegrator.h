/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RotationIntegrator.h
 * @brief  Integrator of IMU angular velocity readings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */
#pragma once

#include <mola-kernel/Yaml.h>

namespace mola
{
/** Write me!
 *
 * \ingroup mola_imu_preintegration_grp
 */
class RotationIntegrator
{
   public:
    RotationIntegrator()  = default;
    ~RotationIntegrator() = default;

    void initialize(const Yaml& cfg);
    //    void spinOnce() override;
};

}  // namespace mola
