/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/
/**
 * @file   index_se3_t.h
 * @brief  Discrete index type for SE(3) lattices, suitable for std::map
 *         and std::unordered_map
 * @author Jose Luis Blanco Claraco
 * @date   Apr 16, 2024
 */
#pragma once

#include <cstdint>
#include <functional>  // hash<>
#include <iostream>

namespace mola
{
/** Discrete index type for SE(3) lattices, suitable for std::map and
 * std::unordered_map. The SO(3) part is parameterized as yaw/pitch/roll angles.
 */
template <typename cell_coord_t = int32_t>
struct index_se3_t
{
  index_se3_t() = default;

  index_se3_t(
      cell_coord_t Cx, cell_coord_t Cy, cell_coord_t Cz, cell_coord_t Cyaw, cell_coord_t Cpitch,
      cell_coord_t Croll) noexcept
      : cx(Cx), cy(Cy), cz(Cz), cyaw(Cyaw), cpitch(Cpitch), croll(Croll)
  {
  }

  cell_coord_t cx = 0, cy = 0, cz = 0;
  cell_coord_t cyaw = 0, cpitch = 0, croll = 0;

  bool operator==(const index_se3_t<cell_coord_t>& o) const noexcept
  {
    return cx == o.cx && cy == o.cy && cz == o.cz && cyaw == o.cyaw && cpitch == o.cpitch &&
           croll == o.croll;
  }
  bool operator!=(const index_se3_t<cell_coord_t>& o) const noexcept { return !operator==(o); }
};

template <typename cell_coord_t>
std::ostream& operator<<(std::ostream& o, const index_se3_t<cell_coord_t>& idx)
{
  o << "(" << idx.cx << "," << idx.cy << "," << idx.cz << "," << idx.cyaw << "," << idx.cpitch
    << "," << idx.croll << ")";
  return o;
}

/** This implements a hash for index_se3_t
 *
 */
template <typename cell_coord_t = int32_t>
struct index_se3_t_hash
{
  /// Hash operator for unordered maps:
  std::size_t operator()(const index_se3_t<cell_coord_t>& k) const noexcept
  {
    std::hash<cell_coord_t> h;
    std::size_t             ret = 0;
    ret ^= h(k.cx) | 1;
    ret ^= h(k.cy) | 2;
    ret ^= h(k.cz) | 3;
    ret ^= h(k.cyaw) | 4;
    ret ^= h(k.cpitch) | 5;
    ret ^= h(k.croll) | 6;
    return ret;
  }
};

}  // namespace mola
