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
 * @file   KeyframePointCloudMap.h
 * @brief  Key-frames, each keeping point cloud layers with their own KD-tree
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */
#pragma once

#include <mp2p_icp/IcpPrepareCapable.h>
#include <mp2p_icp/MetricMapMergeCapable.h>
#include <mp2p_icp/NearestPointWithCovCapable.h>
#include <mrpt/containers/NonCopiableData.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/opengl/opengl_frwds.h>

#include <map>
#include <optional>

namespace mola
{
/** An efficient storage class for large point clouds built as keyframes, each having an associated
 * local cloud.
 *
 * The user of the class is responsible for processing raw observations into
 * mrpt::obs::CObservationPointCloud observations, the only ones allowed as input to the insert*()
 * methods, with points already transformed from the sensor frame to the vehicle (`base_link`)
 * frame. This can be easily done with mp2p_icp::Generator, plus an optional filtering pipeline.
 *
 * Each key-frame is responsible of keeping its own KD-tree for NN searches and keeping up-to-date
 * covariances for each point local vicinity.
 */
class KeyframePointCloudMap : public mrpt::maps::CMetricMap,
                              public mrpt::maps::NearestNeighborsCapable,
                              public mp2p_icp::IcpPrepareCapable,
                              public mp2p_icp::NearestPointWithCovCapable,
                              public mp2p_icp::MetricMapMergeCapable
{
  DEFINE_SERIALIZABLE(KeyframePointCloudMap, mola)
 public:
  // Prevent copying and moving
  KeyframePointCloudMap(const KeyframePointCloudMap&)            = default;
  KeyframePointCloudMap& operator=(const KeyframePointCloudMap&) = default;
  KeyframePointCloudMap(KeyframePointCloudMap&&)                 = default;
  KeyframePointCloudMap& operator=(KeyframePointCloudMap&&)      = default;

  /** @name Basic API for construction and main parameters
   *  @{ */

  using KeyFrameID = uint64_t;  ///< key-frame ID, index in the keyframes_ map.

  /**
   * @brief Constructor / default ctor
   */
  KeyframePointCloudMap() = default;

  ~KeyframePointCloudMap();

  /** @} */

  /** @name Data structures
   *  @{ */

  /** @} */

  /** @name Data access API
   *  @{ */
  // clear(): available in base class

  /** Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if
   * there are no points. Results are cached unless the map is somehow
   * modified to avoid repeated calculations.
   */
  mrpt::math::TBoundingBoxf boundingBox() const override;

  /** @} */

  /** @name API of the NearestNeighborsCapable virtual interface
  @{ */
  [[nodiscard]] bool nn_has_indices_or_ids() const override
  {
    return true;  // Yes, but only via the IcpPrepareCapable interface
  }
  [[nodiscard]] size_t nn_index_count() const override
  {
    // icp_search_submap is rebuilt (potentially from another thread) inside
    // icp_get_prepared_as_global() while holding state_mtx_.  We must acquire
    // the same lock before reading it to avoid a data race on the
    // partially-constructed optional<KeyFrame>.
    auto lck = mrpt::lockHelper(*state_mtx_);
    if (cached_.icp_search_submap && cached_.icp_search_submap->pointcloud())
    {
      return cached_.icp_search_submap->pointcloud()->size();
    }
    return 0;
  }

  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override;
  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result, float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override;
  void nn_multiple_search(
      const mrpt::math::TPoint3Df& query, size_t N, std::vector<mrpt::math::TPoint3Df>& results,
      std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const override;
  void nn_multiple_search(
      const mrpt::math::TPoint2Df& query, size_t N, std::vector<mrpt::math::TPoint2Df>& results,
      std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const override;
  void nn_radius_search(
      const mrpt::math::TPoint3Df& query, float search_radius_sqr,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const override;
  void nn_radius_search(
      const mrpt::math::TPoint2Df& query, float search_radius_sqr,
      std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const override;

  template <size_t MAX_KNN>
  void nn_multiple_search_impl(
      const mrpt::math::TPoint3Df& query, size_t N, std::vector<mrpt::math::TPoint3Df>& results,
      std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const;

  /** @} */

  /** @name Public virtual methods implementation for IcpPrepareCapable
   *  @{ */

  /// Prepare the map for ICP with a given point as reference.
  void icp_get_prepared_as_global(
      const mrpt::poses::CPose3D&                     icp_ref_point,
      const std::optional<mrpt::math::TBoundingBoxf>& local_map_roi = std::nullopt) const override;

  /// Optionally, clean up after ICP is done.
  void icp_cleanup() const override;

  /** @} */

  /** @name Public virtual methods implementation for MetricMapMergeCapable
   *  @{ */

  void merge_with(
      const MetricMapMergeCapable&               source,
      const std::optional<mrpt::poses::CPose3D>& otherRelativePose = std::nullopt) override;

  void transform_map_left_multiply(const mrpt::poses::CPose3D& b) override;

  /** @} */

  /** @name Public virtual methods implementation for NearestPointWithCovCapable
   *  @{ */
  void nn_search_cov2cov(
      const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
      float max_search_distance, mp2p_icp::MatchedPointWithCovList& outPairings) const override;

  [[nodiscard]] std::size_t point_count() const override;

  /** @} */

  /** @name Public virtual methods implementation for CMetricMap
   *  @{ */

  /** Returns a short description of the map. */
  std::string asString() const override;

  void getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const override;

  /** Returns true if the map is empty */
  bool isEmpty() const override;

  /** This virtual method saves the map to a file "filNamePrefix"+<
   * some_file_extension >, as an image or in any other applicable way (Notice
   * that other methods to save the map may be implemented in classes
   * implementing this virtual interface).  */
  void saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const override;

  /// Returns a cached point cloud view of the entire map.
  /// Not efficient at all. Only for MOLA->ROS2 bridge.
  const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() const override;

  /** @} */

  /** Options for insertObservation()
   */
  struct TInsertionOptions : public mrpt::config::CLoadableOptions
  {
    TInsertionOptions() = default;

    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& c,
        const std::string&                   s) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);

    /// If >0, remove old key-frames farther than this (meters)
    double remove_frames_farther_than = 0;
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
        const std::string&                   section) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);
  };
  TLikelihoodOptions likelihoodOptions;

  /** Rendering options, used in getAs3DObject()
   */
  struct TRenderOptions : public mrpt::config::CLoadableOptions
  {
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& c,
        const std::string&                   s) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    /** Binary dump to stream - used in derived classes' serialization */
    void writeToStream(mrpt::serialization::CArchive& out) const;
    /** Binary dump to stream - used in derived classes' serialization */
    void readFromStream(mrpt::serialization::CArchive& in);

    float point_size = 1.0f;

    /** Color of points. Superseded by colormap if the latter is set. */
    mrpt::img::TColorf color{.0f, .0f, 1.0f};

    /** Colormap for points (index is "z" coordinates) */
    mrpt::img::TColormap colormap = mrpt::img::cmHOT;

    /** If colormap!=mrpt::img::cmNONE, use this channel as color index */
    std::string recolorByPointField = "intensity";

    uint64_t max_points_per_kf = 10000;  //!< Max points to render per key-frame

    uint64_t max_overall_points = 1000000;  //!< Max points to render in global maps

    float keyframes_axes_length = .0f;  //!< If >0, draw XYZ frames per key-frame in the map
  };
  TRenderOptions renderOptions;

  struct TCreationOptions : public mrpt::config::CLoadableOptions
  {
    TCreationOptions() = default;
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& c,
        const std::string&                   s) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);

    uint32_t max_search_keyframes      = 3;  //!< Maximum number of key-frames to search for NN
    uint32_t k_correspondences_for_cov = 20;

    /** Weight converting angular distance [rad] to equivalent linear
     *  distance [m] for keyframe proximity ranking. Higher values favor
     *  angularly-close (similar orientation) frames. */
    double rotation_distance_weight = 2.0;

    /** Number of keyframe slots (out of max_search_keyframes) reserved
     *  for angularly diverse and/or more distant keyframes.
     *  Must be < max_search_keyframes. */
    uint32_t num_diverse_keyframes = 1;

    /** If true (default), and if both the reference and query point clouds contain
     *  per-point view-direction fields ("view_x", "view_y", "view_z" - unit vectors
     *  pointing FROM the point TOWARD the sensor at acquisition time), then a
     *  cov-to-cov pairing is accepted only when the angle between the two view
     *  directions is at most `max_view_angle_deg`.
     *
     *  The rationale is that two points on opposite sides of a thin surface (e.g.
     *  a wall seen from the front vs. the back, or a thin pole) will have nearly
     *  anti-parallel view vectors.  Pairing them would produce a badly conditioned
     *  or outright wrong ICP residual.  120° is a reasonable default: it rejects
     *  pairs whose view directions differ by more than 120° (cos < -0.5) while
     *  keeping pairs seen from "similar enough" directions.
     *
     *  Setting this to `false`, or to a threshold ≥ 180°, effectively disables
     *  the filter even when view fields are present.
     */
    bool use_view_direction_filter = true;

    /** Maximum allowed angle [degrees] between the view-direction vectors of a
     *  candidate cov-to-cov pair.  Only used when `use_view_direction_filter`
     *  is `true` and the view fields are present.  Default: 120°.
     */
    double max_view_angle_deg = 120.0;
  };
  TCreationOptions creationOptions;

  // Interface for use within a mrpt::maps::CMultiMetricMap:
  MAP_DEFINITION_START(KeyframePointCloudMap)
  mola::KeyframePointCloudMap::TCreationOptions   creationOptions;
  mola::KeyframePointCloudMap::TInsertionOptions  insertionOpts;
  mola::KeyframePointCloudMap::TLikelihoodOptions likelihoodOpts;
  mola::KeyframePointCloudMap::TRenderOptions     renderOpts;
  MAP_DEFINITION_END(KeyframePointCloudMap)

 private:
  /** Key-frame data */
  class KeyFrame
  {
   public:
    KeyFrame(std::size_t k_correspondences_for_cov)
        : k_correspondences_for_cov_(k_correspondences_for_cov)
    {
    }

    // Copy constructor
    KeyFrame(const KeyFrame& other)
        : timestamp(other.timestamp),
          k_correspondences_for_cov_(other.k_correspondences_for_cov_),
          pointcloud_(other.pointcloud_),
          pose_(other.pose_)
    {
      // Do not copy cached data -> force recomputation
      invalidateCache();
    }

    // Copy assignment operator (safe self-assignment)
    KeyFrame& operator=(const KeyFrame& other)
    {
      if (this != &other)
      {
        k_correspondences_for_cov_ = other.k_correspondences_for_cov_;
        pointcloud_                = other.pointcloud_;
        pose_                      = other.pose_;
        timestamp                  = other.timestamp;

        invalidateCache();
      }
      return *this;
    }

    // Rule of 5: default is fine
    ~KeyFrame()                              = default;
    KeyFrame(KeyFrame&&) noexcept            = default;
    KeyFrame& operator=(KeyFrame&&) noexcept = default;

    /** Local metric map for this key-frame. Points are already transformed from the sensor frame
     * to the vehicle ("base_link") frame.
     */
    void pointcloud(const mrpt::maps::CPointsMap::Ptr& pc)
    {
      pointcloud_ = pc;
      invalidateCache();
    }

    [[nodiscard]] const mrpt::maps::CPointsMap::Ptr& pointcloud() const { return pointcloud_; }

    [[nodiscard]] const mrpt::maps::CPointsMap::Ptr& pointcloud_global() const
    {
      ASSERT_(pointcloud_);
      if (!pointcloud_global_)
      {
        updatePointsGlobal();
      }
      return pointcloud_global_;
    }

    mrpt::Clock::time_point timestamp;  //!< Timestamp of the key-frame (from observation)

    /// Sets the pose of the key-frame in the map reference frame
    void pose(const mrpt::poses::CPose3D& pose)
    {
      pose_ = pose;
      cached_cov_global_.clear();
      pointcloud_global_.reset();
    }
    /// Gets the pose of the key-frame in the map reference frame
    [[nodiscard]] const mrpt::poses::CPose3D& pose() const { return pose_; }

    [[nodiscard]] mrpt::math::TBoundingBoxf localBoundingBox() const;

    /// Ensures the bbox, the kd-tree, and the covariances are built
    void buildCache() const;

    void invalidateCache()
    {
      cached_bbox_local_.reset();
      cached_cov_local_.clear();
      cached_cov_global_.clear();
      cloud_density_.reset();
      pointcloud_global_.reset();
    }

    const auto& covariancesGlobal() const
    {
      computeCovariancesAndDensity();  // will reuse cached if possible
      updateCovariancesGlobal();  // (idem)
      return cached_cov_global_;
    }

    /** Builds (or get cached) visualization of the cloud in this KF, already transformed to its
     * global pose.
     */
    std::shared_ptr<mrpt::opengl::CPointCloudColoured> getViz(const TRenderOptions& ro) const;

   private:
    std::size_t k_correspondences_for_cov_;

    void updateBBox() const;
    void computeCovariancesAndDensity() const;
    void updateCovariancesGlobal() const;
    void updatePointsGlobal() const;

    mrpt::maps::CPointsMap::Ptr pointcloud_;
    mrpt::poses::CPose3D        pose_;  //!< Pose of the key-frame in the map reference frame

    /// Bounding box in local KF coordinates. Filled by updateBBox()
    mutable std::optional<mrpt::math::TBoundingBoxf> cached_bbox_local_;
    mutable std::optional<float>                     cloud_density_;
    /** One cov per point in local KF coordinates (empty: not computed). Filled by
     * computeCovariancesAndDensity()
     */
    mutable std::vector<mrpt::math::CMatrixFloat33> cached_cov_local_;

    /** One cov per point in global map coordinates (empty: not computed). Updated by
     *  updateCovariancesGlobal() */
    mutable std::vector<mrpt::math::CMatrixFloat33> cached_cov_global_;

    /** The cloud, converted to the "global" frame using "pose" (empty: not computed).
     * Updated by updatePointsGlobal() */
    mutable mrpt::maps::CPointsMap::Ptr pointcloud_global_;

    /** Cached visualization, created/getted by getViz() */
    mutable std::shared_ptr<mrpt::opengl::CPointCloudColoured> cached_viz_;
  };

  std::map<KeyFrameID, KeyFrame> keyframes_;

  /// for cached_ and _keyframes
  mutable mrpt::containers::NonCopiableData<std::recursive_mutex> state_mtx_;

  struct CachedData
  {
    CachedData() = default;

    void reset() { *this = CachedData(); }

    mutable std::optional<mrpt::math::TBoundingBoxf> boundingBox;
    mutable std::optional<std::set<KeyFrameID>>      icp_search_kfs;
    mutable std::optional<KeyFrame>                  icp_search_submap;

    /// Used for getAsSimplePointsMap only.
    mutable mrpt::maps::CSimplePointsMap::Ptr cachedPoints;
  };

  CachedData cached_;

  /** This will be a copy of cachedPoints, but kept here as long as possible until
   * getAsSimplePointsMap() is called again, to extend the life of the returned pointer.
   * A better solution would be for MRPT-3.0 to have shared_ptr return values, when that happens,
   * remove this one.
   */
  mutable mrpt::maps::CSimplePointsMap::Ptr cachedPointsLastReturned_;

 protected:
  // See docs in base CMetricMap class:
  void internal_clear() override;

 private:
  // See docs in base CMetricMap class:
  bool internal_insertObservation(
      const mrpt::obs::CObservation&                   obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;
  // See docs in base class
  double internal_computeObservationLikelihood(
      const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const override;

  double internal_computeObservationLikelihoodPointCloud3D(
      const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
      std::size_t num_pts) const;

  // See docs in base class
  bool internal_canComputeObservationLikelihood(const mrpt::obs::CObservation& obs) const override;

  /// Convert a KF index and a local point index into a global index:
  static uint64_t ToGlobalIndex(const KeyFrameID kf_idx, const size_t local_pt_idx)
  {
    // Build 64 bits from 32bit kf_idx and 32bit local_pt_idx:
    return (kf_idx << 32) | static_cast<uint64_t>(local_pt_idx);
  }
  /// Inverse of ToGlobalIndex(), returning kf_idx and local_pt_idx as a tuple:
  static std::tuple<size_t, size_t> FromGlobalIndex(const uint64_t global_idx)
  {
    const auto kf_idx       = static_cast<size_t>(global_idx >> 32);
    const auto local_pt_idx = static_cast<size_t>(global_idx & 0xFFFFFFFF);
    return {kf_idx, local_pt_idx};
  }

  /// Return the next key-frame ID, which is the size of the map:
  [[nodiscard]] KeyFrameID nextFreeKeyFrameID() const
  {
    if (keyframes_.empty())
    {
      return 0;  // First key-frame
    }
    return keyframes_.rbegin()->first + 1;  // Next ID
  }
};

}  // namespace mola
