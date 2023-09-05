/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   register.cpp
 * @brief  Register RTTI classes
 * @author Jose Luis Blanco Claraco
 * @date   Sep 18, 2021
 */

//#include <mola_imu_preintegration/IMUIntegrationPublisher.h>
#include <mrpt/core/initializer.h>

// using namespace mola;

MRPT_INITIALIZER(do_register_imu_preintegration)
{
    //  MOLA_REGISTER_MODULE(IMUIntegrationPublisher);
}
