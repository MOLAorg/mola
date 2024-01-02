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
 * @file   SparseTreesPointCloud.h
 * @brief  Point cloud stored as a 3D grid of KD-trees/pointclouds
 * @author Jose Luis Blanco Claraco
 * @date   Nov 11, 2023
 */
#pragma once

#include <mola_metric_maps/FixedDenseGrid3D.h>
#include <mola_metric_maps/index3d_t.h>
#include <mrpt/core/round.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <shared_mutex>

namespace mola
{
/** SparseTreesPointCloud: Point cloud stored as a 3D grid of
 * KD-trees/pointclouds. Efficient for storing point clouds and running nearest
 * nearest-neighbor search.
 */
class SparseTreesPointCloud : public mrpt::maps::CMetricMap,
                              public mrpt::maps::NearestNeighborsCapable
{
    DEFINE_SERIALIZABLE(SparseTreesPointCloud, mola)
   public:
    /** @name Compile-time parameters
     *  @{ */
    constexpr static uint8_t GLOBAL_ID_SUBVOXEL_BITCOUNT = 20;

    using outer_index3d_t     = index3d_t<int32_t>;
    using inner_plain_index_t = uint32_t;
    /// collapsed plain unique ID for global indices
    using global_plain_index_t = uint64_t;

    /** @} */

    /** @name Indices and coordinates
     *  @{ */

    inline outer_index3d_t coordToOuterIdx(
        const mrpt::math::TPoint3Df& pt) const
    {
        return outer_index3d_t(
            static_cast<int32_t>((pt.x + grid_size_half_) * grid_size_inv_),  //
            static_cast<int32_t>((pt.y + grid_size_half_) * grid_size_inv_),  //
            static_cast<int32_t>((pt.z + grid_size_half_) * grid_size_inv_));
    }

    /// returns the coordinate of the voxel center
    inline mrpt::math::TPoint3Df outerIdxToCoord(
        const outer_index3d_t idx) const
    {
        return {
            (idx.cx - 0.5f) * grid_size_,  //
            (idx.cy - 0.5f) * grid_size_,  //
            (idx.cz - 0.5f) * grid_size_};
    }

    static inline global_plain_index_t g2plain(
        const outer_index3d_t& g, int subVoxelIndex = 0)
    {
        constexpr uint64_t SUBVOXEL_MASK =
            ((1 << GLOBAL_ID_SUBVOXEL_BITCOUNT) - 1);
        constexpr auto     OFF   = GLOBAL_ID_SUBVOXEL_BITCOUNT;
        constexpr int      FBITS = (64 - OFF) / 3;
        constexpr uint64_t FMASK = (1 << FBITS) - 1;

        return (static_cast<uint64_t>(subVoxelIndex) & SUBVOXEL_MASK) |
               (static_cast<uint64_t>(g.cx & FMASK) << (OFF + FBITS * 0)) |
               (static_cast<uint64_t>(g.cy & FMASK) << (OFF + FBITS * 1)) |
               (static_cast<uint64_t>(g.cz & FMASK) << (OFF + FBITS * 2));
    }

    /** @} */

    /** @name Basic API for construction and main parameters
     *  @{ */

    /**
     * @brief Constructor / default ctor
     * @param voxel_size Voxel size [meters] for decimation purposes.
     */
    SparseTreesPointCloud(float grid_size = 10.0f);

    ~SparseTreesPointCloud();

    /** Reset the main voxel parameters, and *clears* all current map contents
     */
    void setGridProperties(float grid_size);

    /** @} */

    /** @name Data structures
     *  @{ */

    struct GridData
    {
       public:
        GridData() = default;

        auto&       points() { return points_; }
        const auto& points() const { return points_; }

        void insertPoint(const mrpt::math::TPoint3Df& p)
        {
            points_.insertPoint(p);
        }

       private:
        mrpt::maps::CSimplePointsMap points_;
    };

    using grids_map_t =
        std::map<outer_index3d_t, GridData, index3d_hash<int32_t>>;

    /** @} */

    /** @name Data access API
     *  @{ */
    // clear(): available in base class

    /** returns the voxeldata by global index coordinates, creating it or not if
     * not found depending on createIfNew.
     * Returns nullptr if not found and createIfNew is false
     *
     * Function defined in the header file so compilers can optimize
     * for literals "createIfNew"
     */
    inline GridData* gridByOuterIdxs(
        const outer_index3d_t& oIdx, bool createIfNew)
    {
        // 1) Insert into grid map:
        GridData* grid = nullptr;
        cachedMtx_.lock_shared();

        for (int i = 0; i < CachedData::NUM_CACHED_IDXS; i++)
        {
            if (cached_.lastAccessGrid[i] && cached_.lastAccessIdx[i] == oIdx)
            {
                // Cache hit:
#ifdef USE_DEBUG_PROFILER
                mrpt::system::CTimeLoggerEntry tle(
                    profiler, "insertPoint.cache_hit");
#endif
                grid = cached_.lastAccessGrid[i];
                break;
            }
        }
        cachedMtx_.unlock_shared();

        if (!grid)
        {
            // Cache miss:
#ifdef USE_DEBUG_PROFILER
            mrpt::system::CTimeLoggerEntry tle(
                profiler, "insertPoint.cache_misss");
#endif

            auto it = grids_.find(oIdx);
            if (it == grids_.end())
            {
                if (!createIfNew)
                    return nullptr;
                else
                    grid = &grids_[oIdx];  // Create it
            }
            else
            {
                grid = &it->second;  // Use the found grid
            }
            // Add to cache:
            cachedMtx_.lock();

            cached_.lastAccessIdx[cached_.lastAccessNextWrite]  = oIdx;
            cached_.lastAccessGrid[cached_.lastAccessNextWrite] = grid;
            cached_.lastAccessNextWrite++;
            cached_.lastAccessNextWrite &= CachedData::NUM_CACHED_IDX_MASK;

            cachedMtx_.unlock();
        }

        return grid;
    }

    // const version:
    inline const GridData* gridByOuterIdxs(
        const outer_index3d_t& oIdx, bool createIfNew) const
    {  // reuse the non-const method:
        return const_cast<SparseTreesPointCloud*>(this)->gridByOuterIdxs(
            oIdx, createIfNew);
    }

    /** Insert one point into the sparse grid map */
    void insertPoint(const mrpt::math::TPoint3Df& pt)
    {
        auto* g = gridByOuterIdxs(coordToOuterIdx(pt), true);
        g->insertPoint(pt);

        // Also, update bbox:
        if (!cached_.boundingBox_.has_value())
            cached_.boundingBox_.emplace(pt, pt);
        else
            cached_.boundingBox_->updateWithPoint(pt);
    }

    const grids_map_t& grids() const { return grids_; }

    /** Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if
     * there are no points. Results are cached unless the map is somehow
     * modified to avoid repeated calculations.
     */
    mrpt::math::TBoundingBoxf boundingBox() const override;

    void visitAllPoints(
        const std::function<void(const mrpt::math::TPoint3Df&)>& f) const;

    void visitAllGrids(
        const std::function<void(const outer_index3d_t&, const GridData&)>& f)
        const;

    /** Save to a text file. Each line contains "X Y Z" point coordinates.
     *  Returns false if any error occured, true elsewere.
     */
    bool saveToTextFile(const std::string& file) const;

    /** Erase submap blocks entirely farther away than the given distance
     * threshold. */
    void eraseGridsFartherThan(
        const mrpt::math::TPoint3Df& pt, const float distanceMeters);

    /** @} */

    /** @name API of the NearestNeighborsCapable virtual interface
    @{ */
    void                 nn_prepare_for_2d_queries() const override;
    void                 nn_prepare_for_3d_queries() const override;
    [[nodiscard]] bool   nn_has_indices_or_ids() const override;
    [[nodiscard]] size_t nn_index_count() const override;
    [[nodiscard]] bool   nn_single_search(
          const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result,
          float& out_dist_sqr, uint64_t& resultIndexOrID) const override;
    [[nodiscard]] bool nn_single_search(
        const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result,
        float& out_dist_sqr, uint64_t& resultIndexOrID) const override;
    void nn_multiple_search(
        const mrpt::math::TPoint3Df& query, const size_t N,
        std::vector<mrpt::math::TPoint3Df>& results,
        std::vector<float>&                 out_dists_sqr,
        std::vector<uint64_t>&              resultIndicesOrIDs) const override;
    void nn_multiple_search(
        const mrpt::math::TPoint2Df& query, const size_t N,
        std::vector<mrpt::math::TPoint2Df>& results,
        std::vector<float>&                 out_dists_sqr,
        std::vector<uint64_t>&              resultIndicesOrIDs) const override;
    void nn_radius_search(
        const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
        std::vector<mrpt::math::TPoint3Df>& results,
        std::vector<float>&                 out_dists_sqr,
        std::vector<uint64_t>&              resultIndicesOrIDs,
        size_t                              maxPoints) const override;
    void nn_radius_search(
        const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
        std::vector<mrpt::math::TPoint2Df>& results,
        std::vector<float>&                 out_dists_sqr,
        std::vector<uint64_t>&              resultIndicesOrIDs,
        size_t                              maxPoints) const override;
    /** @} */

    /** @name Public virtual methods implementation for CMetricMap
     *  @{ */

    /** Returns a short description of the map. */
    std::string asString() const override;

    void getVisualizationInto(
        mrpt::opengl::CSetOfObjects& outObj) const override;

    /** Returns true if the map is empty */
    bool isEmpty() const override;

    /** This virtual method saves the map to a file "filNamePrefix"+<
     * some_file_extension >, as an image or in any other applicable way (Notice
     * that other methods to save the map may be implemented in classes
     * implementing this virtual interface).  */
    void saveMetricMapRepresentationToFile(
        const std::string& filNamePrefix) const override;

    /** @} */

    /** Options for insertObservation()
     */
    struct TInsertionOptions : public mrpt::config::CLoadableOptions
    {
        TInsertionOptions() = default;

        void loadFromConfigFile(
            const mrpt::config::CConfigFileBase& source,
            const std::string& section) override;  // See base docs
        void dumpToTextStream(
            std::ostream& out) const override;  // See base docs

        void writeToStream(mrpt::serialization::CArchive& out) const;
        void readFromStream(mrpt::serialization::CArchive& in);

        /** Minimum distance between an inserted point and the existing ones in
         * the map for it to be actually inserted. */
        float minimum_points_clearance = 0.20f;

        /** If !=0, remove the submap blocks farther (L1 distance) than this
         * distance [meters] */
        float remove_submaps_farther_than = .0f;
    };
    TInsertionOptions insertionOptions;

    /** Options used when evaluating "computeObservationLikelihood" in the
     * derived classes.
     * \sa CObservation::computeObservationLikelihood
     */
    struct TLikelihoodOptions : public mrpt::config::CLoadableOptions
    {
        TLikelihoodOptions() = default;

        void loadFromConfigFile(
            const mrpt::config::CConfigFileBase& source,
            const std::string& section) override;  // See base docs
        void dumpToTextStream(
            std::ostream& out) const override;  // See base docs

        void writeToStream(mrpt::serialization::CArchive& out) const;
        void readFromStream(mrpt::serialization::CArchive& in);

        /** Sigma (standard deviation, in meters) of the Gaussian observation
         *  model used to model the likelihood (default= 0.5 m) */
        double sigma_dist = 0.5;

        /** Maximum distance in meters to consider for the numerator divided by
         * "sigma_dist", so that each point has a minimum (but very small)
         * likelihood to avoid underflows (default=1.0 meters) */
        double max_corr_distance = 1.0;

        /** Speed up the likelihood computation by considering only one out of N
         * rays (default=10) */
        uint32_t decimation = 10;
    };
    TLikelihoodOptions likelihoodOptions;

    /** Rendering options, used in getAs3DObject()
     */
    struct TRenderOptions : public mrpt::config::CLoadableOptions
    {
        void loadFromConfigFile(
            const mrpt::config::CConfigFileBase& source,
            const std::string& section) override;  // See base docs
        void dumpToTextStream(
            std::ostream& out) const override;  // See base docs

        /** Binary dump to stream - used in derived classes' serialization */
        void writeToStream(mrpt::serialization::CArchive& out) const;
        /** Binary dump to stream - used in derived classes' serialization */
        void readFromStream(mrpt::serialization::CArchive& in);

        float point_size = 1.0f;

        bool show_inner_grid_boxes = false;

        /** Color of points. Superseded by colormap if the latter is set. */
        mrpt::img::TColorf color{.0f, .0f, 1.0f};

        /** Colormap for points (index is "z" coordinates) */
        mrpt::img::TColormap colormap = mrpt::img::cmHOT;

        /** If colormap!=mrpt::img::cmNONE, use this coordinate
         *  as color index: 0=x  1=y  2=z
         */
        uint8_t recolorizeByCoordinateIndex = 2;
    };
    TRenderOptions renderOptions;

   public:
    // Interface for use within a mrpt::maps::CMultiMetricMap:
    MAP_DEFINITION_START(SparseTreesPointCloud)
    float grid_size = 10.0f;

    mola::SparseTreesPointCloud::TInsertionOptions  insertionOpts;
    mola::SparseTreesPointCloud::TLikelihoodOptions likelihoodOpts;
    mola::SparseTreesPointCloud::TRenderOptions     renderOpts;
    MAP_DEFINITION_END(SparseTreesPointCloud)

   private:
    float grid_size_ = 10.0f;

    // Calculated from the above, in setVoxelProperties()
    float                 grid_size_inv_  = 1.0f / grid_size_;
    float                 grid_size_half_ = 0.5f * grid_size_;
    mrpt::math::TVector3D gridVector_;

    /** Voxel map as a set of fixed-size grids */
    grids_map_t grids_;

    struct CachedData
    {
        CachedData() = default;

        void reset() { *this = CachedData(); }

        mutable std::optional<mrpt::math::TBoundingBoxf> boundingBox_;

        // 2 bits seems to be the optimum for typical cases:
        constexpr static int CBITS               = 2;
        constexpr static int NUM_CACHED_IDXS     = 1 << CBITS;
        constexpr static int NUM_CACHED_IDX_MASK = NUM_CACHED_IDXS - 1;

        int             lastAccessNextWrite = 0;
        outer_index3d_t lastAccessIdx[NUM_CACHED_IDXS];
        GridData*       lastAccessGrid[NUM_CACHED_IDXS] = {nullptr};
    };

    CachedData        cached_;
    std::shared_mutex cachedMtx_;

   protected:
    // See docs in base CMetricMap class:
    void internal_clear() override;

   private:
    // See docs in base CMetricMap class:
    bool internal_insertObservation(
        const mrpt::obs::CObservation&                   obs,
        const std::optional<const mrpt::poses::CPose3D>& robotPose =
            std::nullopt) override;
    // See docs in base class
    double internal_computeObservationLikelihood(
        const mrpt::obs::CObservation& obs,
        const mrpt::poses::CPose3D&    takenFrom) const override;

    double internal_computeObservationLikelihoodPointCloud3D(
        const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys,
        const float* zs, const std::size_t num_pts) const;

    /** - (xs,ys,zs): Sensed point local coordinates in the robot frame.
     *  - pc_in_map: SE(3) pose of the robot in the map frame.
     */
    void internal_insertPointCloud3D(
        const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys,
        const float* zs, const std::size_t num_pts);

    // See docs in base class
    bool internal_canComputeObservationLikelihood(
        const mrpt::obs::CObservation& obs) const override;
};

}  // namespace mola
