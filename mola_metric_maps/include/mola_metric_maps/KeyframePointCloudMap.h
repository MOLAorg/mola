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

#include <mola_metric_maps/MatchingDistanceProfileCompat.h>
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
#include <mrpt/viz/viz_frwds.h>

#include <functional>
#include <map>
#include <optional>
#include <set>
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
 *
 * @note For single-frame global voxel maps, see `mola::SparseVoxelPointCloud`,
 *       `mola::HashedVoxelPointCloud`, or `mola::NDT`.
 */
class KeyframePointCloudMap : public mrpt::maps::CMetricMap,
                              public mrpt::maps::NearestNeighborsCapable,
                              public mp2p_icp::IcpPrepareCapable,
                              public mp2p_icp::NearestPointWithCovCapable,
                              public mp2p_icp::MetricMapMergeCapable,
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

  /** Snapshot of all key-frame poses, keyed by KF id. The returned map is a
   *  deep copy and is safe to hold across subsequent map mutations.
   *  Thread-safe.
   */
  [[nodiscard]] std::map<KeyFrameID, mrpt::poses::CPose3D> keyframePoses() const;

  /** Overwrites the pose of one KF in the map, invalidating its caches.
   *  No-op if `id` is not present. Thread-safe.
   */
  void setKeyframePose(KeyFrameID id, const mrpt::poses::CPose3D& new_pose);

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

#if defined(MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE)
  void nn_search_cov2cov(
      const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
      const mp2p_icp::MatchingDistanceProfile& matchingDistance,
      mp2p_icp::MatchedPointWithCovList&       outPairings) const override;
#endif

  [[nodiscard]] std::size_t point_count() const override;

  /** @} */

  /** @name Public virtual methods implementation for CMetricMap
   *  @{ */

  /** Returns a short description of the map. */
  std::string asString() const override;

  void getVisualizationInto(mrpt::viz::CSetOfObjects& outObj) const override;

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

  /** @name Offline keyframe regrouping (localization-oriented map optimization)
   *  @{ */

  /** Parameters for regroupKeyframes(). */
  struct RegroupParams
  {
    /** Voxel resolution [m] used to compute pairwise cloud overlaps. If <=0, it
     *  is auto-derived from the median per-keyframe sensing radius. */
    double voxel_size = 0;

    /** Minimum pairwise voxel-overlap (Jaccard-min, in [0,1]) required to create
     *  an edge in the keyframe adjacency graph, i.e. for two keyframes to be
     *  candidates for merging into the same super-keyframe. */
    double edge_overlap = 0.75;

    /** Inner-core fraction, in (0,1], of a super-keyframe's spatial extent. A member
     *  keyframe whose center lies within `core_fraction * extent` of the seed is
     *  marked "covered" (deep inside, single-KF localization is reliable there).
     *  Members in the outer band stay available to seed/join neighboring
     *  super-keyframes, which is what produces the deliberate inter-group overlap.
     *  The outer (overlap) band is `(1 - core_fraction)` of the extent, so the
     *  default of 0.85 yields ~15% overlap between neighboring super-keyframes. */
    double core_fraction = 0.85;

    /** Spatial extent cap of a super-keyframe, as a multiple of the seed keyframe's
     *  own sensing radius (half its cloud bounding-box diagonal). This auto-sizes
     *  groups non-uniformly: large outdoors (long range), small indoors. */
    double extent_factor = 2.0;

    /** If >0, voxel-downsample each merged super-keyframe cloud at this resolution
     *  [m] to bound the point-count blow-up from the deliberate overlap. This is
     *  strongly recommended for large maps. Per-point view-direction fields, when
     *  present, are carried over (first point kept per voxel). */
    double merge_decimate_voxel = 0;

    /** Any cluster whose total point count falls below this fraction of the
     *  largest cluster's point count is treated as an "island": too small to
     *  usefully stand on its own as a super-keyframe (it would mislead
     *  proximity-based nearest-keyframe search at runtime -- see
     *  `TCreationOptions::density_penalty_min_points` -- while contributing
     *  little real geometry). Islands are absorbed into their nearest
     *  neighboring cluster (by center distance) instead of being emitted as
     *  their own super-keyframe. Set to 0 to disable (old behavior: every
     *  cluster becomes its own super-keyframe, however small). Default 0.025 = 2.5%. */
    double island_merge_fraction = 0.025;

    /** If `true`, bypass the overlap-graph clustering entirely and merge ALL
     *  keyframes into a single super-keyframe, anchored at the pose of the
     *  first (oldest) keyframe. Useful to turn a whole session's keyframe map
     *  into one wide, self-contained keyframe -- e.g. so a subsequent
     *  session's localization always sees the full previous map instead of
     *  only the `max_search_keyframes` nearest of many small keyframes, which
     *  under-covers stop-and-rotate scanning patterns. `merge_decimate_voxel`
     *  still applies to bound the resulting cloud size; all other clustering
     *  parameters above are ignored. Default `false`. */
    bool unify_all = false;
  };

  /** Builds a NEW map whose keyframes are "super-keyframes": spatially coherent
   *  groups of the original keyframes, merged into single larger clouds with
   *  deliberate overlap, so that localization only needs ONE active keyframe at a
   *  time (see \ref RegroupParams and the graph-theoretic grouping in the .cpp).
   *  All options (creation/insertion/likelihood/render) are copied from `*this`.
   *  Thread-safe (reads `*this` under its lock). `logCb`, if set, receives
   *  human-readable progress lines.
   */
  [[nodiscard]] std::shared_ptr<KeyframePointCloudMap> regroupKeyframes(
      const RegroupParams& params, const std::function<void(const std::string&)>& logCb = {}) const;

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

    /** Maximum distance [meters] any neighbor may sit from the least-squares
     *  plane through the whole neighborhood for that neighborhood to be given
     *  the plane regularization below. 0 (default) disables the test, which is
     *  the historical behavior: every neighborhood with enough neighbors is
     *  regularized to a 1000:1 plane covariance whether or not it is planar.
     *
     *  This is Fast-LIO2's `esti_plane` gate. A neighborhood that fails it
     *  falls back to an isotropic covariance, the same fallback the
     *  too-few-neighbors case uses, so the pairing still constrains the
     *  solution -- it just stops asserting a surface normal it cannot support.
     *  The local density estimate is deliberately left untouched by a
     *  rejection, so this knob moves the covariance and nothing else.
     */
    double max_plane_deviation_for_cov = 0;

    /** The smallest of the three regularized singular values of a per-point
     *  covariance, i.e. the variance asserted along the estimated surface
     *  normal. The other two are 1, so this IS the plane confidence ratio:
     *  the shipped 1e-3 asserts 1000:1.
     *
     *  Exposed because that ratio is not free. Information the whitening puts
     *  into the normal direction it takes, relatively, from the two directions
     *  in the surface, and on a ground vehicle the normal of the dominant
     *  surface is the vertical -- the axis that is already best determined.
     *  Raising this softens the assertion without changing which direction is
     *  asserted. Must be in (0, 1].
     */
    double plane_regularization_lambda = 1e-3;

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

    /** If `true`, each keyframe's per-cloud 3D KD-tree index is serialized
     *  alongside its points (see `KeyframePointCloudMap` serialization), so it
     *  does NOT have to be rebuilt when the map is loaded. This trades a larger
     *  `.mm` file for faster startup in localization-only use. Requires an MRPT
     *  build providing the KD-tree save/load API (feature-detected at compile
     *  time); when unavailable this option is silently a no-op on write. Default
     *  `false`. Typically enabled offline by the `mm-kf-bake-kdtrees` tool.
     */
    bool serialize_kdtrees = false;

    /** If `true`, each keyframe's per-point covariances (the `cached_cov_local_`
     *  produced by the plane-regularized SVD in `computeCovariancesAndDensity()`)
     *  are serialized alongside its points, so they do NOT have to be recomputed
     *  when the map is loaded. Covariance computation (one K-NN query + 3×3 SVD
     *  per point) is the single most expensive part of warming a freshly-loaded
     *  keyframe for ICP, so persisting it removes the multi-second stall paid the
     *  first time each keyframe becomes active in localization-only operation.
     *
     *  Unlike `serialize_kdtrees`, this needs no special MRPT API and works on
     *  any build. The stored covariances are the *local-frame* ones; the cheap
     *  per-pose global rotation (`updateCovariancesGlobal()`) is still done at
     *  runtime. Trades a larger `.mm` file (one 3×3 float matrix per point) for
     *  faster startup. Default `false`. Typically enabled offline by the
     *  `mm-kf-bake-kdtrees` tool.
     */
    bool serialize_covariances = false;

    /** If `true`, `nn_search_cov2cov()` (used by `mp2p_icp::Matcher_Cov2Cov`, i.e.
     *  GICP-style pipelines) skips building the merged, multi-keyframe submap that
     *  `icp_get_prepared_as_global()` otherwise assembles for the active KF set.
     *  Instead, each active keyframe's own already-built KD-tree and per-point
     *  covariances (both cached at insertion time, in the KF's *local* neighborhood
     *  only) are queried directly: for every query point, one KD-tree lookup is done
     *  per active keyframe ("N" lookups instead of 1 on a merged cloud), and the
     *  overall closest one is kept, together with that keyframe's own cached
     *  covariance at that point.
     *
     *  This trades exactness for speed: the reference covariance at the matched point
     *  is estimated only from neighbors *within the same source keyframe*, whereas the
     *  exact (default) mode computes it from neighbors in the merged cloud, which may
     *  include points contributed by other active keyframes. It also avoids the
     *  merged-cloud allocation, copy and KD-tree (re)build entirely, which can be a
     *  significant fraction of `icp_get_prepared_as_global()`'s cost when the active
     *  set has several sizeable keyframes.
     *
     *  Only affects `nn_search_cov2cov()`. The generic `NearestNeighborsCapable`
     *  entry points (`nn_single_search()`, `nn_multiple_search()`, `nn_radius_search()`)
     *  still require the merged submap and are not supported in this mode. Default
     *  `false` (exact, merged-cloud behavior, unchanged).
     */
    bool approximate_cov = false;

    /** Below this point count, a keyframe candidate is considered "sparse" for
     *  proximity ranking in `icp_get_prepared_as_global()`, and gets a distance
     *  penalty (see `density_penalty_max_m`) linearly scaled by how far below
     *  this floor its point count is (0 penalty at/above this count, maximum
     *  penalty at 0 points). Guards against small/leftover keyframes (e.g. an
     *  under-merged cluster from `regroupKeyframes()`) outranking a much
     *  better-populated keyframe merely because their pose happens to be
     *  closer to the query. Default 200000; set to 0 to disable. */
    uint32_t density_penalty_min_points = 200000;

    /** Maximum proximity-ranking penalty [m] added for a keyframe with (close
     *  to) zero points; see `density_penalty_min_points`. Default 20.0. */
    double density_penalty_max_m = 20.0;
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
        double max_distance_for_cov, double max_plane_deviation_for_cov,
        double plane_regularization_lambda)
        : k_correspondences_for_cov_(k_correspondences_for_cov),
          min_correspondences_for_cov_(min_correspondences_for_cov),
          max_distance_for_cov_(max_distance_for_cov),
          max_plane_deviation_for_cov_(max_plane_deviation_for_cov),
          plane_regularization_lambda_(plane_regularization_lambda)
    {
    }

    // Copy constructor
    KeyFrame(const KeyFrame& other)
        : timestamp(other.timestamp),
          k_correspondences_for_cov_(other.k_correspondences_for_cov_),
          min_correspondences_for_cov_(other.min_correspondences_for_cov_),
          max_distance_for_cov_(other.max_distance_for_cov_),
          max_plane_deviation_for_cov_(other.max_plane_deviation_for_cov_),
          plane_regularization_lambda_(other.plane_regularization_lambda_),
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
        max_plane_deviation_for_cov_ = other.max_plane_deviation_for_cov_;
        plane_regularization_lambda_ = other.plane_regularization_lambda_;
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
        double max_distance_for_cov, double max_plane_deviation_for_cov,
        double plane_regularization_lambda)
    {
      k_correspondences_for_cov_   = k_correspondences_for_cov;
      min_correspondences_for_cov_ = min_correspondences_for_cov;
      max_distance_for_cov_        = max_distance_for_cov;
      max_plane_deviation_for_cov_ = max_plane_deviation_for_cov;
      plane_regularization_lambda_ = plane_regularization_lambda;
      invalidateCache();
    }

    const auto& covariancesGlobal() const
    {
      computeCovariancesAndDensity();  // will reuse cached if possible
      updateCovariancesGlobal();  // (idem)
      return cached_cov_global_;
    }

    /** Ensures the per-point *local-frame* covariances are computed and returns
     *  them. Used to bake them into the serialized map (see
     *  `TCreationOptions::serialize_covariances`). */
    const std::vector<mrpt::math::CMatrixFloat33>& covariancesLocal() const
    {
      computeCovariancesAndDensity();
      return cached_cov_local_;
    }

    /** Installs precomputed per-point *local-frame* covariances loaded from a
     *  baked `.mm`, so `computeCovariancesAndDensity()` becomes a no-op (it
     *  early-returns when `cached_cov_local_.size() == pointcloud size`). The
     *  global-frame rotation is invalidated so it is recomputed for the current
     *  pose. No-op (covariances left to be computed lazily) if `covs` does not
     *  match the current point count. Must be called after `pointcloud()`, which
     *  clears all caches. */
    void installCovariancesLocal(std::vector<mrpt::math::CMatrixFloat33>&& covs) const
    {
      if (!pointcloud_ || covs.size() != pointcloud_->size())
      {
        return;  // size mismatch: fall back to lazy recomputation
      }
      cached_cov_local_ = std::move(covs);
      cached_cov_global_.clear();  // invalidate: rotated lazily for current pose
    }

    /** Builds (or get cached) visualization of the cloud in this KF, already transformed to its
     * global pose.
     *
     * If `overrideColor` is set, ALL points are painted that single color instead of the
     * normal per-field colormap, and the result is NOT cached (used by the per-keyframe
     * debug coloring, see `MOLA_KEYFRAME_MAP_VIZ_COLOR_BY_KF`).
     */
    std::shared_ptr<mrpt::viz::CPointCloudColoured> getViz(
        const TRenderOptions&                   ro,
        const std::optional<mrpt::img::TColor>& overrideColor = std::nullopt) const;

    std::shared_ptr<mrpt::viz::CSetOfObjects> getCovarianceEllipsoidViz(
        const TRenderOptions& ro) const;

   private:
    std::size_t k_correspondences_for_cov_;
    std::size_t min_correspondences_for_cov_;
    double      max_distance_for_cov_;
    double      max_plane_deviation_for_cov_;
    double      plane_regularization_lambda_;

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
    mutable std::shared_ptr<mrpt::viz::CPointCloudColoured> cached_viz_;

    /** Cached cov visualization, created/getted by getCovarianceEllipsoidViz() */
    mutable std::shared_ptr<mrpt::viz::CSetOfObjects> cachez_viz_covs_;
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

    /// Value of creationOptions.approximate_cov used to build the cache above. Compared
    /// against the live option in icp_get_prepared_as_global() so that toggling the flag
    /// (even with an unchanged active KF set) forces a rebuild instead of silently reusing
    /// a cache built under the other mode.
    mutable bool icp_search_built_approximate = false;

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

  /** Implements `nn_search_cov2cov()` for `creationOptions.approximate_cov == true`: queries
   *  each keyframe in `activeKfs` with its own cached KD-tree instead of a merged submap.
   *  `localKf`'s pose must already have been set to the query pose by the caller.
   *  \sa TCreationOptions::approximate_cov
   */
  void nn_search_cov2cov_approximate(
      const KeyFrame& localKf, const std::set<KeyFrameID>& activeKfs,
      const MatchingDistanceProfile&     matchingDistance,
      mp2p_icp::MatchedPointWithCovList& outPairings) const;

  /** The actual cov2cov search. Both public overloads forward here, so the
   *  implementation stays free of preprocessor branches. */
  void nn_search_cov2cov_impl(
      const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
      const MatchingDistanceProfile&     matchingDistance,
      mp2p_icp::MatchedPointWithCovList& outPairings) const;
};

}  // namespace mola
