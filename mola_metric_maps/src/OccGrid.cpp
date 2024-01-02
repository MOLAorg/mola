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
 * @file   OccGrid.cpp
 * @brief  Class extending MRPT occupancy grid with additional functionalities
 * @author Jose Luis Blanco Claraco
 * @date   Feb 21, 2021
 */

#include <mola_metric_maps/OccGrid.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

IMPLEMENTS_SERIALIZABLE(OccGrid, CSerializable, mola)

OccGrid::OccGrid()
{
    // reset:
    resetLikelihoodCache();
}

// =====================================
// Serialization
// =====================================

uint8_t OccGrid::serializeGetVersion() const { return 0; }
void    OccGrid::serializeTo(mrpt::serialization::CArchive& out) const
{
    // The data
    out << grid_;
}
void OccGrid::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            in >> grid_;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };

    // cache reset
    resetLikelihoodCache();
}

// =====================================
// Size API
// =====================================

void OccGrid::setSize(
    const mrpt::math::TPoint2Df& minCorner,
    const mrpt::math::TPoint2Df& maxCorner, float resolution,
    float occupancyValue)
{
    grid_.setSize(
        minCorner.x, maxCorner.x, minCorner.y, maxCorner.y, resolution,
        occupancyValue);

    resetLikelihoodCache();
}

void OccGrid::resizeGrid(
    const mrpt::math::TPoint2Df& minCorner,
    const mrpt::math::TPoint2Df& maxCorner, float newCellsOccupancy) noexcept
{
    grid_.resizeGrid(
        minCorner.x, maxCorner.x, minCorner.y, maxCorner.y, newCellsOccupancy);

    const auto defValue = CACHE_MISS_VALUE;
    likelihoodCacheGrid_.resize(
        grid_.getXMin(), grid_.getXMax(), grid_.getYMin(), grid_.getYMax(),
        defValue, 0.5 /* extra margin [meters] */);
}

// =====================================
//   Read and update API
// =====================================
void OccGrid::insertObservation(
    const mrpt::obs::CObservation2DRangeScan& obs,
    const mrpt::math::TPose2D&                robotPose)
{
    // Update the grid:
    auto&       gio = grid_.insertionOptions;
    const auto& ip  = insertionParameters_;

    gio.maxDistanceInsertion        = insertionParameters_.maxDistanceInsertion;
    gio.maxOccupancyUpdateCertainty = ip.maxOccupancyUpdateCertainty;
    gio.maxFreenessUpdateCertainty  = ip.maxFreenessUpdateCertainty;
    gio.maxFreenessInvalidRanges    = ip.maxFreenessInvalidRanges;
    gio.decimation                  = ip.decimation;
    gio.wideningBeamsWithDistance   = ip.wideningBeamsWithDistance;
    gio.considerInvalidRangesAsFreeSpace = ip.considerInvalidRangesAsFreeSpace;

    const auto p = mrpt::poses::CPose3D(robotPose);
    grid_.insertObservation(obs, p);

    // Enqueue areas as pending of likelihood recalculation:
    // TODO;
}

// =====================================
// Likelihood API
// =====================================
void OccGrid::resetLikelihoodCache()
{
    const auto defValue = CACHE_MISS_VALUE;
    likelihoodCacheGrid_.setSize(
        grid_.getXMin(), grid_.getXMax(), grid_.getYMin(), grid_.getYMax(),
        grid_.getResolution() * likelihoodParameters_.superResolutionFactor,
        &defValue);
}
