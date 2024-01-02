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
 * @file   SparseVoxelPointCloud.h
 * @brief  Point cloud stored as a dual-resolution voxel map
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
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

namespace mola
{
/** SparseVoxelPointCloud: a pointcloud stored as a sparse-dense set of
 *  cubic voxel maps. Efficient for storing point clouds, decimating them,
 *  and running nearest nearest-neighbor search.
 */
class SparseVoxelPointCloud : public mrpt::maps::CMetricMap,
                              public mrpt::maps::NearestNeighborsCapable
{
    DEFINE_SERIALIZABLE(SparseVoxelPointCloud, mola)
   public:
    /** @name Compile-time parameters
     *  @{ */

    /// Size of the std::array for the small-size optimization container in each
    /// voxel, defining the maximum number of points that can be stored without
    /// heap allocation.
    constexpr static std::size_t HARDLIMIT_MAX_POINTS_PER_VOXEL = 16;
    constexpr static uint32_t    INNER_GRID_BIT_COUNT           = 5;
    constexpr static std::size_t GLOBAL_ID_SUBVOXEL_BITCOUNT    = 4;
    static_assert(
        HARDLIMIT_MAX_POINTS_PER_VOXEL <= (1 << GLOBAL_ID_SUBVOXEL_BITCOUNT));

    constexpr static uint32_t INNER_GRID_SIDE   = 1 << INNER_GRID_BIT_COUNT;
    constexpr static uint32_t INNER_COORDS_MASK = INNER_GRID_SIDE - 1;
    constexpr static uint32_t OUTER_COORDS_MASK = ~INNER_COORDS_MASK;

    using global_index3d_t    = index3d_t<int32_t>;
    using outer_index3d_t     = index3d_t<int32_t>;
    using inner_index3d_t     = index3d_t<uint32_t>;
    using inner_plain_index_t = uint32_t;

    /// collapsed plain unique ID for global indices
    using global_plain_index_t = uint64_t;

    /** @} */

    /** @name Indices and coordinates
     *  @{ */

    static inline outer_index3d_t g2o(const global_index3d_t& g)
    {
        return outer_index3d_t(
            g.cx & OUTER_COORDS_MASK,  //
            g.cy & OUTER_COORDS_MASK,  //
            g.cz & OUTER_COORDS_MASK);
    }

    static inline inner_index3d_t g2i(const global_index3d_t& g)
    {
        return inner_index3d_t(
            g.cx & INNER_COORDS_MASK,  //
            g.cy & INNER_COORDS_MASK,  //
            g.cz & INNER_COORDS_MASK);
    }

    static inline inner_index3d_t plain2i(const inner_plain_index_t& p)
    {
        return inner_index3d_t(
            p & INNER_COORDS_MASK,  //
            (p >> INNER_GRID_BIT_COUNT) & INNER_COORDS_MASK,  //
            (p >> (2 * INNER_GRID_BIT_COUNT)) & INNER_COORDS_MASK);
    }

    inline global_index3d_t coordToGlobalIdx(
        const mrpt::math::TPoint3Df& pt) const
    {
        return global_index3d_t(
            mrpt::round(pt.x * voxel_size_inv_),  //
            mrpt::round(pt.y * voxel_size_inv_),  //
            mrpt::round(pt.z * voxel_size_inv_));
    }

    /// returns the coordinate of the voxel center
    inline mrpt::math::TPoint3Df globalIdxToCoord(
        const global_index3d_t idx) const
    {
        return {
            idx.cx * voxel_size_,  //
            idx.cy * voxel_size_,  //
            idx.cz * voxel_size_};
    }

    static inline global_plain_index_t g2plain(
        const global_index3d_t& g, int subVoxelIndex = 0)
    {
        constexpr uint64_t SUBVOXEL_MASK =
            ((1 << GLOBAL_ID_SUBVOXEL_BITCOUNT) - 1);
        constexpr auto     OFF   = GLOBAL_ID_SUBVOXEL_BITCOUNT;
        constexpr int      FBITS = 20;  // (64 - OFF)/3, rounded if needed
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
    SparseVoxelPointCloud(float voxel_size = 0.20f);

    ~SparseVoxelPointCloud();

    /** Reset the main voxel parameters, and *clears* all current map contents
     */
    void setVoxelProperties(float voxel_size);

    /** @} */

    /** @name Data structures
     *  @{ */

    struct InnerGrid;

    struct VoxelData
    {
       public:
        VoxelData() = default;

        struct PointSpan
        {
            PointSpan(
                const mrpt::maps::CSimplePointsMap& data,
                const uint32_t* pointIndices, size_t n)
                : data_(data), pointIndices_(pointIndices), n_(n)
            {
            }

            size_t size() const { return n_; }
            bool   empty() const { return n_ == 0; }

            mrpt::math::TPoint3Df operator[](int i) const
            {
                float x, y, z;
                data_.getPointFast(pointIndices_[i], x, y, z);
                return {x, y, z};
            }

           private:
            const mrpt::maps::CSimplePointsMap& data_;
            const uint32_t*                     pointIndices_;
            const size_t                        n_;
        };

        auto points(const InnerGrid& parent) const -> PointSpan;

        void insertPoint(const mrpt::math::TPoint3Df& p, InnerGrid& parent);

        /** Gets the mean of all points in the voxel. Throws if empty. */
        const mrpt::math::TPoint3Df& mean() const { return mean_; }

        // for serialization, do not use in normal use:
        size_t   size() const { return numPoints_; }
        void     resize(size_t n) { numPoints_ = n; }
        void     setIndex(size_t i, uint32_t idx) { pointIndices_[i] = idx; }
        uint32_t getIndex(size_t i) const { return pointIndices_[i]; }

       private:
        uint32_t pointIndices_[HARDLIMIT_MAX_POINTS_PER_VOXEL] = {0};

        uint8_t                       numPoints_ = 0;
        mutable mrpt::math::TPoint3Df mean_      = {0, 0, 0};
    };

    struct InnerGrid
    {
        FixedDenseGrid3D<VoxelData, INNER_GRID_BIT_COUNT, uint32_t> gridData;
        mrpt::maps::CSimplePointsMap                                points;
    };

    using grids_map_t =
        std::map<outer_index3d_t, InnerGrid, index3d_hash<int32_t>>;

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
    inline std::tuple<VoxelData*, InnerGrid*> voxelByGlobalIdxs(
        const global_index3d_t& idx, bool createIfNew)
    {
        // Get voxel indices:
        const outer_index3d_t oIdx = g2o(idx);
        const inner_index3d_t iIdx = g2i(idx);

        // 1) Insert into decimation voxel map:
        InnerGrid* grid = nullptr;
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
                    return {nullptr, nullptr};
                else
                    grid = &grids_[oIdx];  // Create it
            }
            else
            {
                // Use the found grid
                grid = const_cast<InnerGrid*>(&it->second);
            }
            // Add to cache:
            cached_.lastAccessIdx[cached_.lastAccessNextWrite]  = oIdx;
            cached_.lastAccessGrid[cached_.lastAccessNextWrite] = grid;
            cached_.lastAccessNextWrite++;
            cached_.lastAccessNextWrite &= CachedData::NUM_CACHED_IDX_MASK;
        }

        // Now, look for the cell withi the grid block:
        return {&grid->gridData.cellByIndex(iIdx), grid};
    }

    std::tuple<const VoxelData*, const InnerGrid*> voxelByGlobalIdxs(
        const global_index3d_t& idx, bool createIfNew) const
    {  // reuse the non-const method:
        return const_cast<SparseVoxelPointCloud*>(this)->voxelByGlobalIdxs(
            idx, createIfNew);
    }

    /** Get a voxeldata by (x,y,z) coordinates, creating the container grid if
     * needed. */
    std::tuple<VoxelData&, InnerGrid&> voxelByCoords(
        const mrpt::math::TPoint3Df& pt)
    {
        auto [voxel, grid] =
            voxelByGlobalIdxs(coordToGlobalIdx(pt), true /*create*/);
        return {*voxel, *grid};
    }
    std::tuple<const VoxelData&, const InnerGrid&> voxelByCoords(
        const mrpt::math::TPoint3Df& pt) const
    {  // reuse the non-const method
        return const_cast<SparseVoxelPointCloud*>(this)->voxelByCoords(pt);
    }

    /** Insert one point into the dual voxel map */
    void insertPoint(const mrpt::math::TPoint3Df& pt);

    const grids_map_t& grids() const { return grids_; }

    /** Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if
     * there are no points. Results are cached unless the map is somehow
     * modified to avoid repeated calculations.
     */
    mrpt::math::TBoundingBoxf boundingBox() const override;

    void visitAllPoints(
        const std::function<void(const mrpt::math::TPoint3Df&)>& f) const;

    void visitAllVoxels(const std::function<void(
                            const outer_index3d_t&, const inner_plain_index_t,
                            const VoxelData&, const InnerGrid&)>& f) const;

    void visitAllGrids(
        const std::function<void(const outer_index3d_t&, const InnerGrid&)>& f)
        const;

    /** Save to a text file. Each line contains "X Y Z" point coordinates.
     *  Returns false if any error occured, true elsewere.
     */
    bool saveToTextFile(const std::string& file) const;

    /** @} */

    /** @name API of the NearestNeighborsCapable virtual interface
    @{ */
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

        /** Maximum number of points per voxel. 0 means no limit (up to the
         * compile-time limit HARDLIMIT_MAX_POINTS_PER_VOXEL).
         */
        uint32_t max_points_per_voxel = 0;
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

        /** If true, search for pairings only against the voxel mean point,
         * instead of the contained points.
         *
         * This parameters affects both, likelihood, and
         * the NN (nearest-neighbors) API methods.
         */
        bool match_mean = false;
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

        /** If true, when rendering a voxel map only the mean point per voxel
         * will be rendered instead of all contained points. */
        bool show_mean_only = true;

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
    MAP_DEFINITION_START(SparseVoxelPointCloud)
    float voxel_size = 0.20f;

    mola::SparseVoxelPointCloud::TInsertionOptions  insertionOpts;
    mola::SparseVoxelPointCloud::TLikelihoodOptions likelihoodOpts;
    mola::SparseVoxelPointCloud::TRenderOptions     renderOpts;
    MAP_DEFINITION_END(SparseVoxelPointCloud)

   private:
    float voxel_size_ = 0.20f;

    // Calculated from the above, in setVoxelProperties()
    float                 voxel_size_inv_ = 1.0f / voxel_size_;
    float                 voxel_size_sqr_ = voxel_size_ * voxel_size_;
    mrpt::math::TPoint3Df halfVoxel_;
    mrpt::math::TPoint3Df gridSizeMinusHalf_;

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
        InnerGrid*      lastAccessGrid[NUM_CACHED_IDXS] = {nullptr};
    };

    CachedData cached_;

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
