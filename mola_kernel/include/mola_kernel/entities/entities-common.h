/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   entities-common.h
 * @brief  Includes all headers for common types of world-model entities
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/entities/LandmarkPoint3.h>
#include <mola_kernel/entities/RefPose3.h>
#include <mola_kernel/entities/RelDynPose3KF.h>
#include <mola_kernel/entities/RelPose3.h>
#include <mola_kernel/entities/RelPose3KF.h>

#include <memory>

namespace mola
{
/** Placeholder for generic entity of user-defined types */
using EntityOther = std::shared_ptr<EntityBase>;

}  // namespace mola
