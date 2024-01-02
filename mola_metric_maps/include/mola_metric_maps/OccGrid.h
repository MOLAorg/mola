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
 * @file   OccGrid.h
 * @brief  Class extending MRPT occupancy grid with additional functionalities
 * @author Jose Luis Blanco Claraco
 * @date   Feb 21, 2021
 */
#pragma once

#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/serialization/CSerializable.h>

namespace mola
{
/**
 * @brief Extends MRPT's occupancy grid with super-resolution likelihood field.
 *
 */
class OccGrid : public mrpt::serialization::CSerializable
{
    DEFINE_SERIALIZABLE(OccGrid, mola)

   public:
    OccGrid();
    ~OccGrid() = default;

    /** @name Grid size API
     * @{ */

    /** Change the size of gridmap, erasing all its previous contents.
     * \param minCorner The (x,y) coordinates of left-bottom grid corner.
     * \param maxCorner The (x,y) coordinates of right-top grid corner.
     * \param resolution The new size of square cells [meters]
     * \param occupancyValue The occupancy value of cells, tipically 0.5.
     */
    void setSize(
        const mrpt::math::TPoint2Df& minCorner,
        const mrpt::math::TPoint2Df& maxCorner, float resolution,
        float occupancyValue = 0.5f);

    /** Change the size of gridmap, maintaining previous contents.
     * \param minCorner The (x,y) coordinates of left-bottom grid corner.
     * \param maxCorner The (x,y) coordinates of right-top grid corner.
     * \param newCellsOccupancyValue Occupancy of new cells, tipically 0.5.
     * \sa setSize
     */
    void resizeGrid(
        const mrpt::math::TPoint2Df& minCorner,
        const mrpt::math::TPoint2Df& maxCorner,
        float                        newCellsOccupancy = 0.5f) noexcept;

    /** @} */

    /** @name Read and update API
     * @{ */

    struct InsertionParameters
    {
        InsertionParameters() = default;

        /** The largest distance at which cells will be updated [m] */
        float maxDistanceInsertion = 50.0;

        /** A value in the range [0.5,1] used for updating cell with a bayesian
         * approach (default 0.8) */
        float maxOccupancyUpdateCertainty{0.65f};

        /** A value in the range [0.5,1] for updating a free cell. (default=0
         * means use the same than maxOccupancyUpdateCertainty) */
        float maxFreenessUpdateCertainty{.0f};

        /** Like maxFreenessUpdateCertainty, but for invalid ranges (no echo).
         * (default=0 means same than maxOccupancyUpdateCertainty)*/
        float maxFreenessInvalidRanges{.0f};

        /** If set to true (default), invalid range values (no echo rays) as
         * consider as free space until "maxOccupancyUpdateCertainty", but ONLY
         * when the previous and next rays are also an invalid ray. */
        bool considerInvalidRangesAsFreeSpace{true};

        /** Specify the decimation of the range scan (default=1 : take all the
         * range values!) */
        uint16_t decimation{1};

        /** Enabled: Rays widen with distance to approximate the real behavior
         * of lasers, disabled: insert rays as simple lines (Default=false) */
        bool wideningBeamsWithDistance{false};
    };

    InsertionParameters insertionParameters_;

    void insertObservation(
        const mrpt::obs::CObservation2DRangeScan& obs,
        const mrpt::math::TPose2D&                robotPose);

    const mrpt::maps::COccupancyGridMap2D& grid() const { return grid_; }

    /** @} */

    /** @name Likelihood API
     * @{ */
    struct LikelihoodParameters
    {
        LikelihoodParameters() = default;

        double superResolutionFactor = 4;
    };

    LikelihoodParameters likelihoodParameters_;

    /** Invalidates all cached likelihood values (normally users do not need to
     * call this) */
    void resetLikelihoodCache();

    /** For use in likelihoodCacheGrid_ */
    static constexpr float CACHE_MISS_VALUE = std::numeric_limits<float>::min();

    /** @} */

   private:
    mrpt::maps::COccupancyGridMap2D grid_;

    /** This grid has a higher resolution than grid_ and holds cached likelihood
     * log-likelihood values for it.
     *
     * Cells without cached values hold the invalid value CACHE_MISS_VALUE.
     */
    mrpt::containers::CDynamicGrid<float> likelihoodCacheGrid_;
};

}  // namespace mola
