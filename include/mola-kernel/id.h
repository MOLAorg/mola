/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   id.h
 * @brief  Defines world entities ID types and values.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <cstdint>
#include <limits>

namespace mola
{
/** Unique ID for each Entity in a WorldModel. \ingroup mola_kernel_grp */
using id_t = std::uint64_t;
/** A numeric value for invalid IDs. \ingroup mola_kernel_grp */
constexpr id_t INVALID_ID = std::numeric_limits<id_t>::max();

}  // namespace mola
