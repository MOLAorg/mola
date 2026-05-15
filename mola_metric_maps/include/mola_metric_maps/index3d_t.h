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
 * @file   index3d_t.h
 * @brief  Discrete index type for voxel or 3D grid maps, suitable for std::map
 *         and std::unordered_map
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */
#pragma once

#include <cstdint>
#include <iostream>

namespace mola
{
/** Discrete 3D voxel index: a plain (cx, cy, cz) triple of integer coordinates.
 *
 * Suitable for both `std::map` (using `index3d_hash` as comparator) and
 * `std::unordered_map` / `tsl::robin_map` (using `index3d_hash` as hash).
 *
 * The default coordinate type is `int32_t`, which covers roughly ±2×10⁹ voxels
 * per axis. Use `uint32_t` for inner (non-negative) indices in dense grids.
 *
 * @see index3d_hash for the associated hash / comparator functor.
 */
template <typename cell_coord_t = int32_t>
struct index3d_t
{
  index3d_t() = default;

  index3d_t(cell_coord_t Cx, cell_coord_t Cy, cell_coord_t Cz) noexcept : cx(Cx), cy(Cy), cz(Cz) {}

  cell_coord_t cx = 0, cy = 0, cz = 0;

  bool operator==(const index3d_t<cell_coord_t>& o) const noexcept
  {
    return cx == o.cx && cy == o.cy && cz == o.cz;
  }
  bool operator!=(const index3d_t<cell_coord_t>& o) const noexcept { return !operator==(o); }

  index3d_t operator+(const index3d_t& o) const noexcept
  {
    return {cx + o.cx, cy + o.cy, cz + o.cz};
  }
  index3d_t operator-(const index3d_t& o) const noexcept
  {
    return {cx - o.cx, cy - o.cy, cz - o.cz};
  }
};

template <typename cell_coord_t>
std::ostream& operator<<(std::ostream& o, const index3d_t<cell_coord_t>& idx)
{
  o << "(" << idx.cx << "," << idx.cy << "," << idx.cz << ")";
  return o;
}

/** Hash functor and comparator for `index3d_t`, usable with both
 * `std::unordered_map` / `tsl::robin_map` (hash) and `std::map` (comparator).
 *
 * The hash function is the optimized spatial hash from:
 *   Teschner et al., "Optimized spatial hashing for collision detection of
 *   deformable objects", VMV 2003.
 * It mixes the three integer coordinates with large prime multipliers and
 * truncates to 20 bits, giving good distribution for typical voxel grids.
 *
 * The `operator()(k1,k2)` overload provides a strict weak ordering on
 * `index3d_t` (X-primary, Y-secondary, Z-tertiary) for `std::map`.
 */
template <typename cell_coord_t = int32_t>
struct index3d_hash
{
  /// Hash operator for unordered maps:
  std::size_t operator()(const index3d_t<cell_coord_t>& k) const noexcept
  {
    // These are the implicit assumptions of the reinterpret cast below:
    static_assert(sizeof(cell_coord_t) == sizeof(uint32_t));
    static_assert(offsetof(index3d_t<cell_coord_t>, cx) == 0 * sizeof(uint32_t));
    static_assert(offsetof(index3d_t<cell_coord_t>, cy) == 1 * sizeof(uint32_t));
    static_assert(offsetof(index3d_t<cell_coord_t>, cz) == 2 * sizeof(uint32_t));

    const uint32_t* vec = reinterpret_cast<const uint32_t*>(&k);
    return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
  }

  /// k1 < k2? for std::map containers
  bool operator()(
      const index3d_t<cell_coord_t>& k1, const index3d_t<cell_coord_t>& k2) const noexcept
  {
    if (k1.cx != k2.cx)
    {
      return k1.cx < k2.cx;
    }
    if (k1.cy != k2.cy)
    {
      return k1.cy < k2.cy;
    }
    return k1.cz < k2.cz;
  }
};

}  // namespace mola
