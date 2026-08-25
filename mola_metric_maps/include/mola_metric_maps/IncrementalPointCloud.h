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
 * @file   IncrementalPointCloud.h
 * @brief  Sliding-window point cloud with an incremental (self-balancing) k-d tree
 * @author Jose Luis Blanco Claraco
 * @date   Jul 25, 2026
 */
#pragma once

/** \def MOLA_METRIC_MAPS_HAS_INCREMENTAL_POINT_CLOUD
 *  Defined by the build system when nanoflann >= 1.10.0 (the version that
 *  introduced the incremental k-d tree index) was available, i.e. when
 *  mola::IncrementalPointCloud is actually functional.
 *
 *  The class is declared, compiled and registered in the class factory either
 *  way, so this header is always usable; without a suitable nanoflann,
 *  *constructing* the map throws an explanatory std::runtime_error instead of
 *  failing earlier with a confusing "no such registered CMetricMap class".
 */

#include <mola_metric_maps/MatchingDistanceProfileCompat.h>
#include <mola_metric_maps/OptionsCapable.h>
#include <mp2p_icp/NearestPointWithCovCapable.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace mola
{
namespace internal
{
class IncrementalKDTree;
}

/** A single-global-frame, sliding-window point map for LiDAR (inertial)
 *  odometry, backed by **one incremental, self-balancing nanoflann k-d tree**
 *  instead of a static tree rebuilt on every scan.
 *
 * ## Why
 * `mrpt::maps::CPointsMap` is a `mrpt::math::KDTreeCapable`: every insertion
 * marks the index dirty and the **entire** tree is rebuilt on the next query,
 * i.e. O(N log N) per scan. Here insertion is amortized O(log N) per point plus
 * bounded (optionally backgrounded) partial rebuilds.
 *
 * ## Role
 * This is the *odometry local map*, playing the same role as
 * `mola::HashedVoxelPointCloud` / `mola::SparseTreesPointCloud`, but exact
 * (per-point) rather than voxelized:
 * - points are added by the usual `insertObservation()` / `insertAnotherMap()`
 *   / `insertPoint()` entry points inherited from `CGenericPointsMap`, so all
 *   per-point channels (`t`, `intensity`, `view_x/y/z`, ...) are supported;
 * - points far from the robot are evicted on insertion, see
 *   `TCreationOptions::remove_points_farther_than`;
 * - nearest-neighbor queries go through `mrpt::maps::NearestNeighborsCapable`;
 * - GICP-style matching goes through `mp2p_icp::NearestPointWithCovCapable`
 *   (per-point, plane-regularized covariances).
 *
 * ## Not for loop closure
 * A single global tree cannot cheaply absorb a global SE(3) re-map: every
 * coordinate moves, forcing a full rebuild and a full covariance recompute. For
 * loop-closing maps use `mola::KeyframePointCloudMap`, which keeps points in
 * per-keyframe local frames.
 *
 * ## Point indices are storage slots
 * Removal from the tree is lazy (tombstones reclaimed by later rebuilds), and
 * reclaimed slots are recycled by subsequent insertions to keep memory bounded.
 * As a consequence, the inherited `size()` reports the **storage** size (live,
 * plus reclaimed-but-not-yet-reused, plus not-yet-reclaimed slots); use
 * `livePointCount()` for the number of points actually in the map, or
 * `compact()` to make the two equal. The indices reported by the `nn_*` methods
 * are storage slots, valid for `getPointFast()` and stable until the slot is
 * recycled. Views of the live set only (visualization, serialization,
 * `getAsSimplePointsMap()`) are compacted on the fly.
 *
 * Reclaimed slots are blanked to NaN rather than left holding the evicted
 * point's coordinates, so that generic code walking the inherited `CPointsMap`
 * buffers (notably `insertAnotherMap()`, used to copy map layers out for
 * publishing/visualization) does not resurrect long-evicted geometry. Slots of
 * points that are tombstoned but not reclaimed yet do still hold their
 * coordinates; that fraction is bounded by
 * `TCreationOptions::alpha_deleted`.
 *
 * @note Thread-safety: all index access is serialized internally, so a mapping
 *       thread and an ICP thread may use the map concurrently through the
 *       `nn_*` and `insert*` APIs.
 *       The one exception is the *generic* `mrpt::math::KDTreeCapable` API
 *       inherited from `CPointsMap` (`kdTreeNClosestPoint3D*()` and friends),
 *       which builds its own static tree over the raw storage: that tree's
 *       construction calls back into `boundingBox()`, so it takes MRPT's k-d
 *       tree mutex and then ours, while insertion takes them the other way
 *       round. Use those methods only from a single thread, and prefer
 *       `liveCompactedCopy()` anyway, since the raw storage they see includes
 *       the tombstoned and blanked slots.
 */
class IncrementalPointCloud : public mrpt::maps::CGenericPointsMap,
                              public mp2p_icp::NearestPointWithCovCapable,
                              public mola::OptionsCapable
{
  DEFINE_SERIALIZABLE(IncrementalPointCloud, mola)

 public:
  IncrementalPointCloud();
  ~IncrementalPointCloud() override;

  IncrementalPointCloud(const IncrementalPointCloud& o);
  IncrementalPointCloud& operator=(const IncrementalPointCloud& o);

  /** @name Data access API
   *  @{ */

  /** Number of points actually in the map, i.e. excluding the tombstoned
   *  storage slots still counted by the inherited `size()`.
   */
  [[nodiscard]] std::size_t livePointCount() const;

  /** Number of storage slots ready to be reused by the next insertion. */
  [[nodiscard]] std::size_t recyclableSlotCount() const;

  /** Physically drops the tombstoned storage slots (so `size()` becomes
   *  `livePointCount()`) and rebuilds the k-d tree with the current
   *  `creationOptions`. Point indices previously returned by the `nn_*`
   *  methods are invalidated.
   *
   *  Call it after changing any structural option (`async_rebuild`, `alpha_*`,
   *  `reserve_points`) on an already-populated map; `trySetCreationOptions()`
   *  does it for you.
   */
  void compact();

  /** Removes every point outside the axis-aligned cube of half side
   *  `half_side` centered at `center`. This is what insertions do
   *  automatically when `TCreationOptions::remove_points_farther_than` is set.
   */
  void keepOnlyPointsNear(const mrpt::math::TPoint3Df& center, double half_side);

  /** A copy holding only the live points, in ascending slot order, with all
   *  per-point fields preserved.
   */
  [[nodiscard]] mrpt::maps::CGenericPointsMap::Ptr liveCompactedCopy() const;

  /** O(1): the AABB maintained by the k-d tree. It is a conservative superset
   *  of the live points, since not-yet-reclaimed points still contribute.
   */
  mrpt::math::TBoundingBoxf boundingBox() const override;

  /** @} */

  /** @name CPointsMap storage interface
   *  Overridden so that growing the coordinate buffers can never happen while a
   *  background k-d tree rebuild is reading them (see `async_rebuild`).
   *  @{ */
  void reserve(std::size_t newLength) override;
  void resize(std::size_t newLength) override;
  void setSize(std::size_t newLength) override;
  /** @} */

  /** @name Global re-map of all points
   *  These shadow the (non-virtual) `mrpt::maps::CPointsMap` methods of the
   *  same name: moving every point invalidates the whole incremental k-d tree
   *  (its split planes and bounding boxes were built from the old
   *  coordinates), so they apply the transform and then rebuild the index and
   *  drop the cached per-point covariances. Point indices previously returned
   *  by the `nn_*` methods are invalidated.
   *
   *  Being non-virtual in the base class, a call made through a
   *  `mrpt::maps::CPointsMap*` cannot be routed here; that case is still
   *  handled, but lazily: the next query notices that the coordinates moved
   *  and pays the same full rebuild then. That fallback samples a bounded
   *  number of points, so it is reliable for a **global** re-map (which moves
   *  all of them) but not for a mutator rewriting only a few points, nor for a
   *  caller writing through the inherited coordinate buffer references: those
   *  can still leave the index stale, exactly as before. Prefer calling the
   *  methods below, which are also the only ones that wait for a pending
   *  background rebuild (see `async_rebuild`) before the coordinate buffers
   *  are rewritten. \sa compact()
   *
   *  @note This is an O(N log N) rebuild plus a full covariance recompute, so
   *  it is meant for one-shot re-mappings (e.g. re-leveling the odometry frame
   *  against gravity), not for loop closure. \sa KeyframePointCloudMap
   *  @{ */
  void changeCoordinatesReference(const mrpt::poses::CPose2D& b);
  void changeCoordinatesReference(const mrpt::poses::CPose3D& b);
  void changeCoordinatesReference(
      const mrpt::maps::CPointsMap& other, const mrpt::poses::CPose3D& b);
  /** @} */

  /** @name API of the NearestNeighborsCapable virtual interface
   *  @{ */
  [[nodiscard]] bool   nn_has_indices_or_ids() const override;
  [[nodiscard]] size_t nn_index_count() const override;

  void nn_prepare_for_3d_queries() const override;

  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override;
  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result, float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override;
  void nn_multiple_search(
      const mrpt::math::TPoint3Df& query, const size_t N,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const override;
  void nn_multiple_search(
      const mrpt::math::TPoint2Df& query, const size_t N,
      std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const override;
  void nn_radius_search(
      const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const override;
  void nn_radius_search(
      const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
      std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const override;
  /** @} */

  /** @name API of the mp2p_icp::NearestPointWithCovCapable virtual interface
   *  @{ */

  /** Pairs each point of `localMap` (which must also be an
   *  `IncrementalPointCloud`) with its nearest point in this map, weighting the
   *  pair by the GICP matrix `(COV_global + R*COV_local*R^T)^-1`.
   */
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
  std::string asString() const override;
  bool        isEmpty() const override;
  void        getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const override;
  void        saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const override;
  const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() const override;
  /** @} */

  /** All parameters specific to this class. The standard `insertionOptions`,
   *  `likelihoodOptions` and `renderOptions` groups inherited from
   *  `mrpt::maps::CPointsMap` apply as usual.
   *
   *  Changing any of the k-d tree parameters (`async_rebuild`, `alpha_*`,
   *  `reserve_points`) on an already-populated map is fine:
   *  `trySetCreationOptions()` compacts and rebuilds the index with the new
   *  values, so no point is lost, but the point indices previously returned by
   *  the `nn_*` methods are invalidated. \sa compact()
   */
  struct TCreationOptions : public mrpt::config::CLoadableOptions
  {
    TCreationOptions() = default;

    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source,
        const std::string&                   section) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);

    /** If !=0, on each insertion with a known robot pose, keep only the points
     *  inside the axis-aligned cube of this half side [m] around it. This is
     *  the sliding-window trim of FAST-LIO-style local maps, the analogous of
     *  `HashedVoxelPointCloud::remove_voxels_farther_than`.
     */
    double remove_points_farther_than = .0;

    /** Perform the k-d tree balancing rebuilds on a background thread. Bounds
     *  the mapping-thread tail latency (on a LiDAR-odometry local map this
     *  turns ~300 ms insertion spikes into a flat ~10 ms), at the cost of
     *  roughly twice the index memory.
     *
     *  @note The worker reads the inherited coordinate buffers, so they must
     *  not be reallocated while it runs. `reserve()`, `resize()` and
     *  `setSize()` wait for it, which covers `insertObservation()` and
     *  `insertAnotherMap()`; a hand-written `insertPointFast()` loop bypasses
     *  them, so either reserve the storage up front (`reserve_points`) or grow
     *  it through `reserve()`.
     */
    bool async_rebuild = false;

    /** Weight-balance rebuild threshold: a subtree is rebuilt when its larger
     *  child holds more than this fraction of its points. Lower keeps queries
     *  faster and rebuilds more frequent.
     */
    float alpha_balance = 0.75f;

    /** Tombstone rebuild threshold: a subtree is rebuilt, physically dropping
     *  removed points, once this fraction of it is dead.
     */
    float alpha_deleted = 0.5f;

    /** If !=0, pre-allocate storage for this many points. Recommended together
     *  with `async_rebuild`, since it avoids reallocating the coordinate
     *  buffers the background worker reads.
     */
    uint64_t reserve_points = 0;

    /** Number of neighbors used to estimate each point covariance for
     *  `nn_search_cov2cov()`. \sa mola::KeyframePointCloudMap
     */
    uint32_t k_correspondences_for_cov = 20;

    /** Below this number of neighbors actually found, a plane fit is deemed
     *  unreliable and an isotropic covariance is used instead.
     */
    uint32_t min_correspondences_for_cov = 5;

    /** Maximum distance [m] to search neighbors for the covariance estimate. */
    double max_distance_for_cov = 1.0;

    /** Maximum distance [m] any neighbor may sit from the least-squares plane
     *  through the neighborhood for that neighborhood to receive the plane
     *  regularization below. 0 (default) disables the test. Same semantics as
     *  the option of the same name on KeyframePointCloudMap. */
    double max_plane_deviation_for_cov = 0;

    /** Variance asserted along the estimated surface normal, the other two
     *  being 1, i.e. the plane confidence ratio written as its reciprocal. The
     *  shipped 1e-3 asserts 1000:1. Must be in (0, 1]. */
    double plane_regularization_lambda = 1e-3;

    /** If `true`, the k-d tree index is serialized alongside the points (see
     *  `IncrementalPointCloud` serialization), so it does NOT have to be
     *  rebuilt (an O(N log N) bulk build) when the map is loaded. Requires an
     *  MRPT/nanoflann build providing the incremental index's save/load API
     *  (feature-detected at compile time via
     *  `MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE`); when unavailable this
     *  option is silently a no-op on write, and a reader without the feature
     *  skips any baked blob found in the file. Default `false`. Typically
     *  enabled offline by the `mm-ipc-bake-kdtree` tool: the map is always
     *  serialized compacted (no tombstoned slots, see
     *  `IncrementalPointCloud::serializeTo()`), so baking rebuilds the index
     *  once over that compacted point order at save time instead of leaving
     *  the O(N log N) build for every subsequent load.
     */
    bool serialize_kdtree = false;
  };
  TCreationOptions creationOptions;

  // mola::OptionsCapable interface:
  [[nodiscard]] std::map<std::string, mrpt::config::CLoadableOptions*> optionsByName() override;
  bool                                                                 trySetCreationOptions(
                                                                      const mrpt::config::CConfigFileBase& cfg, const std::string& section) override;

 public:
  // Interface for use within a mrpt::maps::CMultiMetricMap:
  MAP_DEFINITION_START(IncrementalPointCloud)
  mola::IncrementalPointCloud::TCreationOptions creationOpts;
  mrpt::maps::CPointsMap::TInsertionOptions     insertionOpts;
  mrpt::maps::CPointsMap::TLikelihoodOptions    likelihoodOpts;
  mrpt::maps::CPointsMap::TRenderOptions        renderOpts;
  MAP_DEFINITION_END(IncrementalPointCloud)

 protected:
  // See docs in base CMetricMap class:
  void internal_clear() override;

  bool internal_insertObservation(
      const mrpt::obs::CObservation&                   obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;

 private:
  /// The actual cov2cov search. Both public overloads forward here, so the
  /// implementation stays free of preprocessor branches.
  void nn_search_cov2cov_impl(
      const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
      const MatchingDistanceProfile&     matchingDistance,
      mp2p_icp::MatchedPointWithCovList& outPairings) const;

  /// Serializes all access to index_ and to the caches below.
  mutable std::recursive_mutex mtx_;

  mutable std::unique_ptr<internal::IncrementalKDTree> index_;

  /// Storage slots reclaimed by the index, ready to be reused.
  mutable std::vector<uint32_t> free_slots_;

  /// Number of leading storage slots already handed to the index.
  mutable std::size_t indexed_up_to_ = 0;

  /// A few (slot, coordinates) samples taken the last time the index was left
  /// consistent. \sa coordinatesChangedExternally()
  mutable std::vector<std::pair<uint32_t, mrpt::math::TPoint3Df>> coordinates_watch_;

  /// Size of the last appended batch, used to keep spare capacity when the
  /// rebuilds run on a background thread (see internal_insertObservation()).
  std::size_t last_insert_batch_ = 0;

  /// Per-point, plane-regularized covariance cache, in this map's frame.
  mutable std::vector<mrpt::math::CMatrixFloat33> cov_;
  mutable std::vector<uint8_t>                    cov_valid_;

  /// Used for getAsSimplePointsMap() only.
  mutable mrpt::maps::CSimplePointsMap::Ptr cachedPoints_;

  /** Creates a fresh, empty index_ from `creationOptions`, sized/reserved as
   *  configured, and points it at the current coordinate buffers. Leaves
   *  indexed_up_to_ at 0: the caller is responsible for indexing the current
   *  storage afterwards (either the bulk `addPoints()` done by resetIndex(),
   *  or `internal::IncrementalKDTree::loadIndex()` when reconstructing a
   *  baked index from a serialized stream).
   */
  void createEmptyIndex();

  /** (Re)creates the index from `creationOptions` and bulk-loads **every**
   *  storage slot into it. Only valid when no slot is tombstoned, i.e. right
   *  after the storage was cleared, replaced wholesale, or compacted;
   *  otherwise use compact(), which drops the dead slots first.
   */
  void resetIndex();

  /** Indexes anything appended through the inherited `CPointsMap` API since
   *  the last call, and rebuilds from scratch if the storage shrank. Must be
   *  called under `mtx_` before any query or eviction.
   */
  void ensureIndexUpToDate() const;

  /// Points the index at the current (possibly reallocated) coordinate buffers.
  void refreshPointBuffers() const;

  /** Rebuilds the index over the current coordinates, keeping the live/dead
   *  status of every storage slot (unlike resetIndex(), which assumes all of
   *  them are live). Used after the coordinates were rewritten in place.
   */
  void rebuildIndexInPlace() const;

  /** Samples the coordinates of a few storage slots, so that a later in-place
   *  rewrite made through the inherited (non-virtual) `CPointsMap` mutators
   *  can be noticed. Called whenever the index is left consistent.
   */
  void refreshCoordinatesWatch() const;

  /** Whether any sampled slot no longer holds the coordinates it had when
   *  refreshCoordinatesWatch() was last called. Only a bounded number of slots
   *  is sampled, so this detects a global re-map (every point moves), not a
   *  rewrite touching just a few points. \sa coordinates_watch_
   */
  [[nodiscard]] bool coordinatesChangedExternally() const;

  /// Moves storage slot `from` (coordinates and every field) onto slot `to`.
  void relocatePoint(std::size_t from, std::size_t to);

  /** Moves the just-appended points in [firstAppended, size()) into recycled
   *  slots where possible, shrinking the storage accordingly, and hands every
   *  resulting slot to the index.
   */
  void indexAppendedPointsRecycling(std::size_t firstAppended);

  /// Collects the slots the index reclaimed, blanks them and marks them reusable.
  void harvestRemovedSlots();

  /// Computes and caches the covariance of one live point, if not cached yet.
  void computeCovariance(uint32_t slot) const;

  /** Fills the covariance cache for the given slots, in parallel when TBB is
   *  available. The caller must pass each slot at most once, so that threads
   *  never write to the same cache entry.
   */
  void ensureCovariancesFor(const std::vector<uint32_t>& slots) const;
};

}  // namespace mola
