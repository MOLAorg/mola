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
 * @file   SparseVoxelPointCloud.h
 * @brief  Point cloud stored as a dual-resolution voxel map
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */
#pragma once

#include <mola_metric_maps/index3d_t.h>
#include <mrpt/core/round.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>

#include <array>
#include <functional>
#include <map>
#include <optional>

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

/** SparseVoxelPointCloud: a pointcloud stored as a sparse-dense set of
 *  cubic voxel maps. Efficient for storing point clouds, decimating them,
 *  and running nearest nearest-neighbor search.
 */
class SparseVoxelPointCloud : public mrpt::maps::CMetricMap
{
    DEFINE_SERIALIZABLE(SparseVoxelPointCloud, mola)
   public:
    /** @name Compile-time parameters
     *  @{ */

    /// Size of the std::array for the small-size optimization container in each
    /// voxel, defining the maximum number of points that can be stored without
    /// heap allocation.
    constexpr static std::size_t SSO_LENGTH           = 8;
    constexpr static uint32_t    INNER_GRID_BIT_COUNT = 5;

    constexpr static uint32_t INNER_GRID_SIDE   = 1 << INNER_GRID_BIT_COUNT;
    constexpr static uint32_t INNER_COORDS_MASK = INNER_GRID_SIDE - 1;
    constexpr static uint32_t OUTER_COORDS_MASK = ~INNER_COORDS_MASK;

    using global_index3d_t    = index3d_t<int32_t>;
    using outer_index3d_t     = index3d_t<int32_t>;
    using inner_index3d_t     = index3d_t<uint32_t>;
    using inner_plain_index_t = uint32_t;

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

    struct VoxelData
    {
       public:
        VoxelData() = default;

        struct PointSpan
        {
            PointSpan(const mrpt::math::TPoint3Df* data, size_t n)
                : data_(data), n_(n)
            {
            }

            auto begin() const { return data_; }
            auto end() const { return data_ + n_; }

            auto begin() { return data_; }
            auto end() { return data_ + n_; }

            size_t size() const { return n_; }
            bool   empty() const { return n_ == 0; }

           private:
            const mrpt::math::TPoint3Df* data_;
            const size_t                 n_;
        };

        auto points() const { return PointSpan(points_.data(), numPoints_); }

        void insertPoint(const mrpt::math::TPoint3Df& p);

        /** Gets the mean of all points in the voxel. Throws if empty. */
        const mrpt::math::TPoint3Df& mean() const { return mean_; }

       private:
        std::array<mrpt::math::TPoint3Df, SSO_LENGTH> points_;
        uint8_t                                       numPoints_ = 0;
        mutable mrpt::math::TPoint3Df                 mean_      = {0, 0, 0};
    };

    using InnerGrid =
        FixedDenseGrid3D<VoxelData, INNER_GRID_BIT_COUNT, uint32_t>;

    using grids_map_t =
        std::map<outer_index3d_t, InnerGrid, index3d_hash<int32_t>>;

    /** @} */

    /** @name Data access API
     *  @{ */
    // clear(): available in base class

    /** Insert one point into the dual voxel map */
    void insertPoint(const mrpt::math::TPoint3Df& pt);

    /** Query for the closest neighbor of a given point.
     *  \return true if nearest neighbor was found.
     */
    bool nn_find_nearest(
        const mrpt::math::TPoint3Df& queryPoint,
        mrpt::math::TPoint3Df& outNearest, float& outDistanceSquared) const;

    const grids_map_t& grids() const { return grids_; }

    /** Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if
     * there are no points. Results are cached unless the map is somehow
     * modified to avoid repeated calculations.
     */
    mrpt::math::TBoundingBoxf boundingBox() const;

    void visitAllPoints(
        const std::function<void(const mrpt::math::TPoint3Df&)>& f) const;

    void visitAllVoxels(const std::function<void(
                            const outer_index3d_t&, const inner_plain_index_t,
                            const VoxelData&)>& f) const;

    void visitAllGrids(
        const std::function<void(const outer_index3d_t&, const InnerGrid&)>& f)
        const;

    /** Save to a text file. Each line contains "X Y Z" point coordinates.
     *  Returns false if any error occured, true elsewere.
     */
    bool saveToTextFile(const std::string& file) const;

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

        /** Maximum insertion distance for points, wrt the sensor location
         *  , in meters. Default=0 means no filtering.
         */
        void max_distance(float v)
        {
            max_distance_     = v;
            max_distance_sqr_ = v * v;
        }
        float max_distance() const { return max_distance_; }
        float max_distance_sqr() const { return max_distance_sqr_; }

        /** Speed up the insertion by skipping points and only inserting
         *  one out of "decimation" points. Default=1 means insert them all.
         */
        uint32_t decimation = 1;

        /** Maximum number of points per voxel. 0 means no limit (up to the
         * compile-time limit SSO_LENGTH).
         */
        uint32_t max_points_per_voxel = 0;

       private:
        float max_distance_ = .0f, max_distance_sqr_ = .0f;
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
    mrpt::math::TPoint3Df halfVoxel_;
    mrpt::math::TPoint3Df gridSizeMinusHalf_;

    /** Voxel map as a set of fixed-size grids */
    grids_map_t grids_;

    struct CachedData
    {
        CachedData() = default;

        void reset() { *this = CachedData(); }

        mutable std::optional<mrpt::math::TBoundingBoxf> boundingBox_;

        outer_index3d_t lastAccessIdx;
        InnerGrid*      lastAccessGrid = nullptr;
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
