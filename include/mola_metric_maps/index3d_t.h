/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
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

namespace mola
{
/** Discrete index type for voxel or 3D grid maps, suitable for std::map and
 * std::unordered_map, using mola::index3d_hash as hash type.
 */
struct index3d_t
{
    index3d_t() = default;

    index3d_t(int32_t cx, int32_t cy, int32_t cz) noexcept
        : cx_(cx), cy_(cy), cz_(cz)
    {
    }

    int32_t cx_ = 0, cy_ = 0, cz_ = 0;

    bool operator==(const index3d_t& o) const noexcept
    {
        return cx_ == o.cx_ && cy_ == o.cy_ && cz_ == o.cz_;
    }
};

struct index3d_hash
{
    std::size_t operator()(const index3d_t& k) const noexcept
    {
        std::size_t res = 17;

        res = res * 31 + std::hash<int32_t>()(k.cx_);
        res = res * 31 + std::hash<int32_t>()(k.cy_);
        res = res * 31 + std::hash<int32_t>()(k.cz_);
        return res;
    }
    // k1 < k2?
    bool operator()(const index3d_t& k1, const index3d_t& k2) const noexcept
    {
        if (k1.cx_ != k2.cx_) return k1.cx_ < k2.cx_;
        if (k1.cy_ != k2.cy_) return k1.cy_ < k2.cy_;
        return k1.cz_ < k2.cz_;
    }
};

}  // namespace mola
