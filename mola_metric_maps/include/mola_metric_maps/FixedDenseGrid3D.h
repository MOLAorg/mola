/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   FixedDenseGrid3D.h
 * @brief  An efficient 3D grid template class with compile-time fixed size.
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */
#pragma once

#include <mola_metric_maps/index3d_t.h>
#include <mrpt/core/exceptions.h>

#include <cstdint>
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

    FixedDenseGrid3D()
    {
        cells_ = reinterpret_cast<T*>(calloc(sizeof(T), TOTAL_CELL_COUNT));
    }
    ~FixedDenseGrid3D() { free(cells_); }

    T& cellByIndex(const index3d_t<inner_coord_t>& idx)
    {
        ASSERTDEB_LT_(idx.cx, CELLS_PER_DIM);
        ASSERTDEB_LT_(idx.cy, CELLS_PER_DIM);
        ASSERTDEB_LT_(idx.cz, CELLS_PER_DIM);

        return cells_
            [idx.cx + (idx.cy << SIDE_NUM_BITS) +
             (idx.cz << (2 * SIDE_NUM_BITS))];
    }
    const T& cellByIndex(const index3d_t<inner_coord_t>& idx) const
    {
        ASSERTDEB_LT_(idx.cx, CELLS_PER_DIM);
        ASSERTDEB_LT_(idx.cy, CELLS_PER_DIM);
        ASSERTDEB_LT_(idx.cz, CELLS_PER_DIM);

        return cells_
            [idx.cx + (idx.cy << SIDE_NUM_BITS) +
             (idx.cz << (2 * SIDE_NUM_BITS))];
    }

    const T* cells() const { return cells_; }
    T*       cells() { return cells_; }

   private:
    T* cells_;
};

}  // namespace mola
