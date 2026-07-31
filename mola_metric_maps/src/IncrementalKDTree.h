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
 * @file   IncrementalKDTree.h
 * @brief  Opaque wrapper over nanoflann's incremental (self-balancing) k-d tree
 * @author Jose Luis Blanco Claraco
 * @date   Jul 25, 2026
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <vector>

namespace mola::internal
{
/** Abstract facade over the nanoflann incremental k-d tree index
 *  (`KDTreeSingleIndexIncrementalAdaptor` and its multi-threaded variant),
 *  used by mola::IncrementalPointCloud.
 *
 *  This header intentionally exposes no nanoflann type: the whole library is
 *  included by a single implementation file that pulls in no MRPT header.
 *  MRPT bundles (and its binaries are compiled against) its own copy of
 *  nanoflann, which is typically older than the one providing the incremental
 *  index. Both copies declare the same non-template entities, so letting them
 *  be visible to the same translation unit -- or exporting either of them
 *  through a public header -- would be an ODR violation with real
 *  memory-corruption potential. See IncrementalKDTree.cpp.
 *
 *  Point coordinates are *not* owned here: the index only stores point indices
 *  ("slots") into externally-owned x/y/z buffers, see setPointBuffers().
 */
class IncrementalKDTree
{
 public:
  struct Params
  {
    /// Run the balancing rebuilds on a background thread.
    bool async_rebuild = false;

    /// Weight-balance and tombstone-fraction rebuild thresholds.
    float alpha_balance = 0.75f;
    float alpha_deleted = 0.5f;
  };

  struct Neighbor
  {
    uint32_t index    = 0;
    float    dist_sqr = 0;
  };

  [[nodiscard]] static std::unique_ptr<IncrementalKDTree> Create(const Params& p);

  virtual ~IncrementalKDTree();

  IncrementalKDTree(const IncrementalKDTree&)            = delete;
  IncrementalKDTree& operator=(const IncrementalKDTree&) = delete;

  /** @name Dataset binding
   *  @{ */

  /** Rebinds the coordinate buffers the index reads point coordinates from.
   *  Must be called again after anything that may have reallocated them
   *  (i.e. after growing the owner point cloud).
   */
  virtual void setPointBuffers(
      const float* xs, const float* ys, const float* zs, std::size_t count) = 0;

  /** @} */

  /** @name Modifiers
   *  @{ */

  /// Pre-sizes internal structures for an expected point count.
  virtual void reserve(std::size_t n) = 0;

  /// Drops the whole tree (and any pending background rebuild).
  virtual void clear() = 0;

  virtual void addPoint(uint32_t index)                           = 0;
  virtual void addPoints(uint32_t firstIndex, uint32_t lastIndex) = 0;
  virtual void removePoint(uint32_t index)                        = 0;

  /** Removes every point outside the given axis-aligned box: the sliding
   *  window map-trimming primitive.
   */
  virtual void keepOnlyPointsInsideBox(const float minCorner[3], const float maxCorner[3]) = 0;

  /** Point slots that became physically free since the last call, so the
   *  owner can recycle them. Removal is lazy (tombstones), so slots only show
   *  up here once an actual rebuild reclaimed them.
   */
  [[nodiscard]] virtual std::vector<uint32_t> acquireRemovedPoints() = 0;

  /// Blocks until any in-flight background rebuild has been integrated.
  virtual void waitForPendingRebuilds() = 0;

  /** @} */

  /** @name Persistence (index topology only; point coordinates are NOT
   *  stored, see setPointBuffers())
   *  @{ */

  /** Serializes the tree topology to a binary stream, so it does not have to
   *  be rebuilt (an O(N log N) bulk build) the next time this same point set
   *  is indexed. Requires nanoflann >= 1.11.0 (the release that added
   *  saveIndex()/loadIndex() to the incremental index); throws
   *  std::runtime_error on older builds. Blocks on any in-flight background
   *  rebuild first (see waitForPendingRebuilds()).
   */
  virtual void saveIndex(std::ostream& stream) = 0;

  /** Loads a topology previously written by saveIndex(). Must be called right
   *  after construction (or clear()), on an index already bound (via
   *  setPointBuffers()) to the same point coordinates that were indexed when
   *  saveIndex() was called. Requires nanoflann >= 1.11.0; throws
   *  std::runtime_error on older builds.
   */
  virtual void loadIndex(std::istream& stream) = 0;

  /** @} */

  /** @name Observers
   *  @{ */

  /// Number of live (non-removed) points.
  [[nodiscard]] virtual std::size_t size() const = 0;

  /// Number of nodes physically stored (live + not-yet-reclaimed tombstones).
  [[nodiscard]] virtual std::size_t physicalSize() const = 0;

  /// Appends the live point slots into `out`.
  virtual void snapshotLiveIndices(std::vector<uint32_t>& out) const = 0;

  /** O(1) AABB of the points in the index (a conservative superset of the live
   *  set, since not-yet-reclaimed tombstones still contribute).
   *  \return false if the index is empty, in which case the outputs are unset.
   */
  [[nodiscard]] virtual bool boundingBox(float minCorner[3], float maxCorner[3]) const = 0;

  /** @} */

  /** @name Queries
   *  @{ */

  /// \return the number of neighbors actually found (<= k).
  [[nodiscard]] virtual std::size_t knnSearch(
      const float query[3], std::size_t k, uint32_t* outIndices, float* outDistsSqr) const = 0;

  /// As knnSearch(), but discarding neighbors farther than `maxDistSqr`.
  [[nodiscard]] virtual std::size_t knnSearchWithinRadius(
      const float query[3], std::size_t k, float maxDistSqr, uint32_t* outIndices,
      float* outDistsSqr) const = 0;

  /// Appends every neighbor closer than `radiusSqr` (squared distance).
  virtual void radiusSearch(
      const float query[3], float radiusSqr, std::vector<Neighbor>& out) const = 0;

  /** @} */

 protected:
  IncrementalKDTree() = default;
};

}  // namespace mola::internal
