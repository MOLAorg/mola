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
 * @file   DualVoxelPointCloud.h
 * @brief  Point cloud stored as a dual-resolution voxel map
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */
#pragma once

#include <mola_metric_maps/index3d_t.h>
#include <mrpt/containers/vector_with_small_size_optimization.h>
#include <mrpt/core/round.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/TPoint3D.h>

#include <tuple>

namespace mola
{
/** DualVoxelPointCloud: a pointcloud stored in two dual hash'ed voxel maps,
 *  one for decimation purposes only, and another for nearest-neighbor search.
 *
 */
class DualVoxelPointCloud : public mrpt::maps::CMetricMap
{
    DEFINE_SERIALIZABLE(DualVoxelPointCloud, mola)
   public:
    /** @name Basic API for construction and main parameters
     *  @{ */

    /**
     * @brief Constructor / default ctor
     * @param decimation_size Voxel size [meters] for decimation purposes.
     * @param max_nn_radius Maximum radius [meters] for nearest-neighbor search.
     * @param max_points_per_voxel If !=0, defines a maximum number of points
     * per voxel.
     */
    DualVoxelPointCloud(
        float decimation_size = 0.20f, float max_nn_radius = 0.60f,
        uint32_t max_points_per_voxel = 0);

    ~DualVoxelPointCloud();

    /** Reset the main voxel parameters, and *clears* all current map contents
     */
    void setVoxelProperties(
        float decimation_size, float max_nn_radius,
        uint32_t max_points_per_voxel = 0);

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
        mrpt::math::TPoint3Df& outNearest, float& outDistanceSquared);

    /** @} */

    /** @name Data structures and compile-time parameters
     *  @{ */

    /// Size of the std::array for the small-size optimization container:
    constexpr static std::size_t SSO_LENGTH    = 16;
    constexpr static std::size_t SSO_LENGTH_NN = 4 * 4;

    /// shortcut to save typing:
    template <typename T, std::size_t LEN>
    using vector_sso =
        mrpt::containers::vector_with_small_size_optimization<T, LEN>;

    struct VoxelData
    {
       public:
        const auto& points() const { return points_; }

        void insertPoint(const mrpt::math::TPoint3Df& p);

        /** Gets the mean of all points in the voxel. Throws if empty. */
        const mrpt::math::TPoint3Df& mean() const;

       private:
        vector_sso<mrpt::math::TPoint3Df, SSO_LENGTH> points_;
        mutable std::optional<mrpt::math::TPoint3Df>  mean_;
    };

    struct VoxelNNData
    {
        // We can store pointers safely, since the unordered_map container
        // does not invalidate them.
        std::unordered_map<
            index3d_t, std::optional<std::reference_wrapper<const VoxelData>>,
            index3d_hash>
            nodes;
    };
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

    using voxel_map_t = std::unordered_map<index3d_t, VoxelData, index3d_hash>;

    const voxel_map_t& voxels() const { return voxels_; }

   public:
    // Interface for use within a mrpt::maps::CMultiMetricMap:
    MAP_DEFINITION_START(DualVoxelPointCloud)
    float  decimation_size      = 0.20f;
    float  max_nn_radius        = 0.60f;
    size_t max_points_per_voxel = 0;
    // mola::DualVoxelPointCloud::TInsertionOptions insertionOpts;
    // mola::DualVoxelPointCloud::TLikelihoodOptions likelihoodOpts;
    MAP_DEFINITION_END(DualVoxelPointCloud)

   private:
    float    decimation_size_      = 0.20f;
    float    max_nn_radius_        = 0.60f;
    float    max_nn_radius_sqr_    = max_nn_radius_ * max_nn_radius_;
    uint32_t max_points_per_voxel_ = 0;

    int32_t nn_to_decim_ratio_ = 3;  // ceiling of nn_radius / decim_size

    /** Decimation voxel map */
    voxel_map_t voxels_;

    /** Nearest-neighbor voxel map */
    std::unordered_map<index3d_t, VoxelNNData, index3d_hash> voxelsNN_;

    inline int32_t coord2idx(float xyz) const
    {
        return mrpt::round(xyz / decimation_size_);
    }

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
    // See docs in base class
    bool internal_canComputeObservationLikelihood(
        const mrpt::obs::CObservation& obs) const override;
};

}  // namespace mola
