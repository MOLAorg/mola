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
 * @file   index3d_t.h
 * @brief  Discrete index type for voxel or 3D grid maps, suitable for std::map
 *         and std::unordered_map
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */
#pragma once

#include <cstdint>
#include <functional>  // hash<>
#include <iostream>

namespace mola
{
/** Discrete index type for voxel or 3D grid maps, suitable for std::map and
 * std::unordered_map, using mola::index3d_hash as hash type.
 */
template <typename cell_coord_t = int32_t>
struct index3d_t
{
    index3d_t() = default;

    index3d_t(cell_coord_t Cx, cell_coord_t Cy, cell_coord_t Cz) noexcept
        : cx(Cx), cy(Cy), cz(Cz)
    {
    }

    cell_coord_t cx = 0, cy = 0, cz = 0;

    bool operator==(const index3d_t<cell_coord_t>& o) const noexcept
    {
        return cx == o.cx && cy == o.cy && cz == o.cz;
    }
    bool operator!=(const index3d_t<cell_coord_t>& o) const noexcept
    {
        return !operator==(o);
    }

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

/** This implement the optimized hash from this paper:
 *
 *  Teschner, M., Heidelberger, B., Müller, M., Pomerantes, D., & Gross, M. H.
 * (2003, November). Optimized spatial hashing for collision detection of
 * deformable objects. In Vmv (Vol. 3, pp. 47-54).
 *
 */
template <typename cell_coord_t = int32_t>
struct index3d_hash
{
    /// Hash operator for unordered maps:
    std::size_t operator()(const index3d_t<cell_coord_t>& k) const noexcept
    {
        // These are the implicit assumptions of the reinterpret cast below:
        static_assert(sizeof(cell_coord_t) == sizeof(uint32_t));
        static_assert(
            offsetof(index3d_t<cell_coord_t>, cx) == 0 * sizeof(uint32_t));
        static_assert(
            offsetof(index3d_t<cell_coord_t>, cy) == 1 * sizeof(uint32_t));
        static_assert(
            offsetof(index3d_t<cell_coord_t>, cz) == 2 * sizeof(uint32_t));

        const uint32_t* vec = reinterpret_cast<const uint32_t*>(&k);
        return ((1 << 20) - 1) &
               (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }

    /// k1 < k2? for std::map containers
    bool operator()(
        const index3d_t<cell_coord_t>& k1,
        const index3d_t<cell_coord_t>& k2) const noexcept
    {
        if (k1.cx != k2.cx) return k1.cx < k2.cx;
        if (k1.cy != k2.cy) return k1.cy < k2.cy;
        return k1.cz < k2.cz;
    }
};

}  // namespace mola
