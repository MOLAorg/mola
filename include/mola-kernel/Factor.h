/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Factor.h
 * @brief  Variant type for "factors" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola-kernel/factors/factors-common.h>
#include <variant>

namespace mola
{
/** Variant type for "factors" in the world model.
 *
 * \ingroup mola_kernel_grp
 */
using Factor = std::variant<std::monostate, FactorRelativePose3, FactorOther>;

}  // namespace mola
