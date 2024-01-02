/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   factors-common.h
 * @brief  Includes all headers for common types of world-model factors
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/factors/FactorDynamicsConstVel.h>
#include <mola_kernel/factors/FactorRelativePose3.h>
#include <mola_kernel/factors/FactorStereoProjectionPose.h>
#include <mola_kernel/factors/SmartFactorIMU.h>
#include <mola_kernel/factors/SmartFactorStereoProjectionPose.h>

#include <memory>

namespace mola
{
/** Placeholder for a generic factor of user-defined types */
using FactorOther = std::shared_ptr<FactorBase>;

}  // namespace mola
