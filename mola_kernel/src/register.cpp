/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   register.cpp
 * @brief  RTTI & CSerializable registry
 * @author Jose Luis Blanco Claraco
 * @date   Jun 22, 2019
 */

#include <mola_kernel/Factor.h>
#include <mola_kernel/WorldModel.h>
#include <mrpt/core/initializer.h>

using namespace mola;

// To be run at .so/.dll load:
MRPT_INITIALIZER(do_register_mrpt_kernel)
{
    // Register module:
    MOLA_REGISTER_MODULE(WorldModel);

    // Register serializable classes:
    mrpt::rtti::registerClass(CLASS_ID(mola::WorldModelData));

    mrpt::rtti::registerClass(CLASS_ID(mola::FactorDynamicsConstVel));
    mrpt::rtti::registerClass(CLASS_ID(mola::FactorRelativePose3));
    mrpt::rtti::registerClass(CLASS_ID(mola::FactorStereoProjectionPose));
    mrpt::rtti::registerClass(CLASS_ID(mola::SmartFactorIMU));
    mrpt::rtti::registerClass(CLASS_ID(mola::SmartFactorStereoProjectionPose));
}
