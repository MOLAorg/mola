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

#include <mola_kernel/interfaces/KeyframeMapCapable.h>
#include <mola_metric_maps/OptionsCapable.h>
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
#include <vector>

namespace mola
{
/** Keyframe-based map: a collection of local point clouds, each anchored to a
 * known SE(3) pose (the keyframe pose).
 *
 * Unlike voxel maps that merge all points into a single global grid, this class
 * keeps each keyframe's point cloud in its **local coordinate frame**. Only the
 * SE(3) pose of each keyframe is stored globally, so correcting the map after a
 * loop closure only requires updating poses, not re-inserting any points.
 *
 * ## Input
 * The caller inserts `mrpt::obs::CObservationPointCloud` observations, with
 * points already expressed in the vehicle `base_link` frame and already motion
 * compensated (deskewed). This is typically produced by an `mp2p_icp::Generator`
 * plus optional filter pipeline.
 *
 * ## Per-keyframe data
 * Each `KeyFrame` holds:
 * - A multi-layer point cloud (`mp2p_icp::metric_map_t`) in local coordinates.
 * - The global SE(3) pose (`mrpt::poses::CPose3D`).
 * - A KD-tree (built lazily) for nearest-neighbor searches within that KF.
 * - Per-point local covariance estimates (used by point-to-plane ICP).
 * - A cached local AABB for fast global bounding-box queries.
 *
 * ## ICP integration
 * For registration queries the class assembles a "search submap" from the
 * keyframes nearest to the current pose (`IcpPrepareCapable`). The
 * `NearestPointWithCovCapable` interface exposes per-point covariances to
 * plane-aware ICP solvers.
 *
 * ## Interfaces implemented
 * - `mrpt::maps::CMetricMap` — standard MRPT map (serialization, visualization).
 * - `mrpt::maps::NearestNeighborsCapable` — KNN queries over the active submap.
 * - `mp2p_icp::IcpPrepareCapable` — prepare a local submap for ICP.
 * - `mp2p_icp::NearestPointWithCovCapable` — NN with local covariance.
 * - `mp2p_icp::MetricMapMergeCapable` — merge another map into this one.
 * - `mola::KeyframeMapCapable` — keyframe pose management (add, update, query).
 *
 * @note For single-frame global voxel maps, see `mola::SparseVoxelPointCloud`,
 *       `mola::HashedVoxelPointCloud`, or `mola::NDT`.
 */
class KeyframePointCloudMap : public mrpt::maps::CMetricMap,
                              public mrpt::maps::NearestNeighborsCapable,
                              public mp2p_icp::IcpPrepareCapable,
                              public mp2p_icp::NearestPointWithCovCapable,
                              public mp2p_icp::MetricMapMergeCapable,
                              public mola::KeyframeMapCapable,
                              public mola::OptionsCapable
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

  /** The KF id that will be assigned to the next inserted key-frame. */
  [[nodiscard]] KeyFrameID nextFreeKeyFrameID_public() const;

  /** Snapshot of all key-frame poses, keyed by KF id. Cheap. Thread-safe. */
  [[nodiscard]] std::map<KeyFrameID, mrpt::poses::CPose3D> cloneKFPoses() const;

  /** Overwrites the pose of one KF in the map, invalidating its caches.
   *  No-op if `id` is not present. Thread-safe.
   */
  void setKeyframePose(KeyFrameID id, const mrpt::poses::CPose3D& new_pose) override;

  // mola::KeyframeMapCapable overrides:
  [[nodiscard]] std::map<KeyFrameID, mrpt::poses::CPose3D> keyframePoses() const override;
  [[nodiscard]] std::optional<KeyFrameID>                  oldestActiveKeyframeID() const override;
  void                                                     applyPivotTransform(
                                                          KeyFrameID pivot_id, const mrpt::poses::CPose3D& delta_at_pivot) override;

  /** The id of the last key-frame successfully inserted, or nullopt if none
   *  has been inserted since the map was created/cleared.
   */
  [[nodiscard]] std::optional<KeyFrameID> lastInsertedKeyFrameID() const;

  /** Returns and clears the list of KF ids evicted since the last call (or
   *  since map construction). Used by callers that maintain per-KF
   *  auxiliary state and need to drop it on eviction.
   */
  [[nodiscard]] std::vector<KeyFrameID> drainEvictedKeyFrameIDs();

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

  /** Transform all KF poses left-multiplying with this transform. Thread safe. */
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

    bool show_covariances = false;  //!< If true, draw per-point covariances as ellipsoids
    mrpt::img::TColorf cov_color{.0f, 1.0f, .0f};
    uint32_t           show_cov_decimation = 100;
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

    /** Minimum number of neighbors (including the query point itself) that must
     *  actually be found to compute a per-point covariance via plane regularization.
     *  Points whose key-frame has fewer than this many neighbors available fall back
     *  to an isotropic (unregularized) covariance, since a plane/line fit from too
     *  few samples is unreliable and noise-sensitive. Must be >= 3 and
     *  <= k_correspondences_for_cov.
     */
    uint32_t min_correspondences_for_cov = 5;

    /** Maximum distance [meters] allowed between a query point and a
     *  candidate neighbor for it to be used in the per-point covariance
     *  estimation. Without this bound, sparse/low-density regions would pull
     *  in far-away "neighbors" that do not represent the local surface,
     *  producing meaningless covariance ellipsoids. Points with fewer than
     *  `min_correspondences_for_cov` neighbors within this radius fall back
     *  to an isotropic covariance (see above).
     */
    double max_distance_for_cov = 1.0;

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
     *
     *  Contract: the "view_x/y/z" fields stored in a keyframe's point cloud
     *  (`KeyFrame::pointcloud_`) MUST be expressed in the *local KF frame*,
     *  not in the global/map frame. `KeyFrame::updatePointsGlobal()` rotates
     *  them to the global frame using `mp2p_icp::rotateViewDirectionFields()`
     *  (see `mp2p_icp/pointcloud_field_utils.h`). Any code that inserts
     *  points carrying these fields into a keyframe (e.g.
     *  `mp2p_icp_filters::FilterMerge`) must call the same helper to rotate
     *  the fields alongside the point coordinates, or this filter will
     *  silently compare vectors expressed in inconsistent frames.
     */
    bool use_view_direction_filter = true;

    /** Maximum allowed angle [degrees] between the view-direction vectors of a
     *  candidate cov-to-cov pair.  Only used when `use_view_direction_filter`
     *  is `true` and the view fields are present.  Default: 120°.
     */
    double max_view_angle_deg = 120.0;
  };
  TCreationOptions creationOptions;

  // mola::OptionsCapable interface:
  [[nodiscard]] std::map<std::string, mrpt::config::CLoadableOptions*> optionsByName() override;
  bool                                                                 trySetCreationOptions(
                                                                      const mrpt::config::CConfigFileBase& cfg, const std::string& section) override;

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
    KeyFrame(
        std::size_t k_correspondences_for_cov, std::size_t min_correspondences_for_cov,
        double max_distance_for_cov)
        : k_correspondences_for_cov_(k_correspondences_for_cov),
          min_correspondences_for_cov_(min_correspondences_for_cov),
          max_distance_for_cov_(max_distance_for_cov)
    {
    }

    // Copy constructor
    KeyFrame(const KeyFrame& other)
        : timestamp(other.timestamp),
          k_correspondences_for_cov_(other.k_correspondences_for_cov_),
          min_correspondences_for_cov_(other.min_correspondences_for_cov_),
          max_distance_for_cov_(other.max_distance_for_cov_),
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
        k_correspondences_for_cov_   = other.k_correspondences_for_cov_;
        min_correspondences_for_cov_ = other.min_correspondences_for_cov_;
        max_distance_for_cov_        = other.max_distance_for_cov_;
        pointcloud_                  = other.pointcloud_;
        pose_                        = other.pose_;
        timestamp                    = other.timestamp;

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

    /** Updates the per-point covariance estimation parameters (normally frozen at construction
     *  time from `TCreationOptions`) and invalidates any cached covariance, so it gets
     *  recomputed with the new parameters next time it is queried.
     */
    void updateCovarianceParams(
        std::size_t k_correspondences_for_cov, std::size_t min_correspondences_for_cov,
        double max_distance_for_cov)
    {
      k_correspondences_for_cov_   = k_correspondences_for_cov;
      min_correspondences_for_cov_ = min_correspondences_for_cov;
      max_distance_for_cov_        = max_distance_for_cov;
      invalidateCache();
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

    std::shared_ptr<mrpt::opengl::CSetOfObjects> getCovarianceEllipsoidViz(
        const TRenderOptions& ro) const;

   private:
    std::size_t k_correspondences_for_cov_;
    std::size_t min_correspondences_for_cov_;
    double      max_distance_for_cov_;

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

    /** Cached cov visualization, created/getted by getCovarianceEllipsoidViz() */
    mutable std::shared_ptr<mrpt::opengl::CSetOfObjects> cachez_viz_covs_;
  };

  std::map<KeyFrameID, KeyFrame> keyframes_;

  /// Last key-frame id successfully inserted (nullopt if none since creation/clear).
  std::optional<KeyFrameID> last_inserted_kf_id_;

  /// KF ids evicted since the last call to drainEvictedKeyFrameIDs().
  std::vector<KeyFrameID> evicted_kf_ids_;

  /// Monotonically increasing KF id allocator. Incremented on every
  /// insertion, never decremented on eviction, so ids are never reused.
  /// Reset to 0 on internal_clear(); restored to max-loaded-id+1 in
  /// serializeFrom().
  KeyFrameID next_free_kf_id_ = 0;

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

  /// Returns the next monotonically-allocated key-frame ID. The id is NOT
  /// consumed by this call; callers must use it as the key for the new KF
  /// and then bump `next_free_kf_id_`.
  [[nodiscard]] KeyFrameID nextFreeKeyFrameID() const { return next_free_kf_id_; }

  /** Non-thread safe version of transform_map_left_multiply() */
  void transform_map_left_multiply_impl(const mrpt::poses::CPose3D& b);
};

}  // namespace mola
