/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Entity.h
 * @brief  Variant type for "entities" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola-kernel/entities/entities-common.h>
#include <variant>

namespace mola
{
/** Variant type for "entities" in the world model.
 *
 * \ingroup mola_kernel_grp
 */
using Entity = std::variant<RefPose3, RelPose3, RelDynPose3, EntityOther>;

}  // namespace mola
