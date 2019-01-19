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
 * It's guaranteed that the type is either:
 * - `std::monostate`: Not initialized, or
 * - A class derived from `FactorBase` (stored by value), or
 * - `FactorOther`, which is a wrapper around `FactorBase::Ptr` (an object
 * allocated in the heap).
 *
 * In this way, `Factor` can be handled as a polymorphic class, without the cost
 * of dynamic memory allocation for the most-common classes, while still
 * allowing using any user-derived class via dynamic memory.
 *
 * \ingroup mola_kernel_grp
 */
using Factor = std::variant<std::monostate, FactorRelativePose3, FactorOther>;

}  // namespace mola
