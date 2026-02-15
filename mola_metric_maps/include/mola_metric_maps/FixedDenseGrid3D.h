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
 * @file   FixedDenseGrid3D.h
 * @brief  An efficient 3D grid template class with compile-time fixed size.
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */
#pragma once

#include <mola_metric_maps/index3d_t.h>
#include <mrpt/core/exceptions.h>

#include <cstdlib>
#include <type_traits>

namespace mola
{
/** A dense 3D grid holding cells of type "T" of fixed size
 *  NxNxN cells, with N=2^SIDE_NUM_BITS.
 *
 *  Used internally in SparseVoxelPointCloud
 */
template <typename T, size_t SIDE_NUM_BITS, typename inner_coord_t>
class FixedDenseGrid3D
{
 public:
  constexpr static size_t CELLS_PER_DIM    = 1 << SIDE_NUM_BITS;
  constexpr static size_t TOTAL_CELL_COUNT = 1 << (3 * SIDE_NUM_BITS);

  // The use of "calloc()" for super fast allocation needs this:
  static_assert(std::is_trivially_copyable_v<T>);

  FixedDenseGrid3D() { cells_ = reinterpret_cast<T*>(calloc(sizeof(T), TOTAL_CELL_COUNT)); }
  ~FixedDenseGrid3D() { free(cells_); }

  // Delete copy and move constructors and assignment operators
  FixedDenseGrid3D(const FixedDenseGrid3D&)            = default;
  FixedDenseGrid3D& operator=(const FixedDenseGrid3D&) = default;
  FixedDenseGrid3D(FixedDenseGrid3D&&)                 = default;
  FixedDenseGrid3D& operator=(FixedDenseGrid3D&&)      = default;

  T& cellByIndex(const index3d_t<inner_coord_t>& idx)
  {
    ASSERTDEB_LT_(idx.cx, CELLS_PER_DIM);
    ASSERTDEB_LT_(idx.cy, CELLS_PER_DIM);
    ASSERTDEB_LT_(idx.cz, CELLS_PER_DIM);

    return cells_[idx.cx + (idx.cy << SIDE_NUM_BITS) + (idx.cz << (2 * SIDE_NUM_BITS))];
  }
  const T& cellByIndex(const index3d_t<inner_coord_t>& idx) const
  {
    ASSERTDEB_LT_(idx.cx, CELLS_PER_DIM);
    ASSERTDEB_LT_(idx.cy, CELLS_PER_DIM);
    ASSERTDEB_LT_(idx.cz, CELLS_PER_DIM);

    return cells_[idx.cx + (idx.cy << SIDE_NUM_BITS) + (idx.cz << (2 * SIDE_NUM_BITS))];
  }

  const T* cells() const { return cells_; }
  T*       cells() { return cells_; }

 private:
  T* cells_;
};

}  // namespace mola
