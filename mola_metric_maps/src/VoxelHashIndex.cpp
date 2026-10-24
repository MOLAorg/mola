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
 * @file   VoxelHashIndex.cpp
 * @brief  Exact nearest-neighbor index over a hashed voxel grid
 * @author Jose Luis Blanco Claraco
 * @date   Oct 24, 2026
 *
 * Every voxel keeps a contiguous copy of the coordinates of its points next to
 * their storage slots, so a query reads a few short arrays instead of chasing
 * one pointer per point down a tree. Queries are exact:
 * - the voxels around the query are visited in Chebyshev rings of growing
 *   radius, each one skipped, without even a hash lookup, when its box lies
 *   farther than the current k-th distance;
 * - the expansion stops as soon as no unvisited voxel can hold a closer point;
 * - a query far from the map, or with a radius covering a large part of it,
 *   would examine many empty cells before reaching the points: once the cells
 *   examined outnumber the occupied voxels, the query scans those instead.
 *
 * Insertion and removal are O(1) per point and there are no rebuilds, so there
 * is no latency spike to hide in a background thread, and every result is a
 * deterministic function of the sequence of operations.
 */

#include "VoxelHashIndex.h"

#include <mola_metric_maps/index3d_t.h>
#include <tsl/robin_map.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using mola::internal::IncrementalKDTree;
using voxel_key_t = mola::index3d_t<int32_t>;

/** Points farther than this many voxels from the origin along any axis are not
 *  indexed (2^30 voxels: a million kilometers with 1 m voxels), so that no
 *  voxel arithmetic can overflow.
 */
constexpr double MAX_ABS_CELL = 1073741824.0;

/** A point is accepted when its squared distance, computed in float, is below
 *  the current bound, so a voxel whose exact distance lies a few float ulps
 *  beyond the bound may still hold an acceptable point: its lower bound is
 *  shrunk by this factor before pruning.
 */
constexpr double PRUNE_SHRINK = 1.0 - 1e-6;

class VoxelHashIndex : public IncrementalKDTree
{
 public:
  explicit VoxelHashIndex(float voxelSize)
      : size_(static_cast<double>(voxelSize)), slack_(1e-6 * static_cast<double>(voxelSize))
  {
    if (!(voxelSize > 0) || !std::isfinite(voxelSize))
    {
      throw std::invalid_argument(
          "VoxelHashIndex: the voxel size must be positive and finite, got " +
          std::to_string(voxelSize));
    }
  }

  void setPointBuffers(
      const float* xs, const float* ys, const float* zs, std::size_t count) override
  {
    xs_    = xs;
    ys_    = ys;
    zs_    = zs;
    count_ = count;
  }

  void reserve(std::size_t n) override { key2voxel_.reserve(n / 8); }

  void clear() override
  {
    key2voxel_.clear();
    voxels_.clear();
    freeVoxels_.clear();
    removed_.clear();
    live_      = 0;
    hasPoints_ = false;
    markContentChanged();
  }

  void addPoint(uint32_t index) override
  {
    insertOne(index);
    markContentChanged();
  }

  void addPoints(uint32_t firstIndex, uint32_t lastIndex) override
  {
    for (uint32_t i = firstIndex; i <= lastIndex; i++)
    {
      insertOne(i);
      if (i == std::numeric_limits<uint32_t>::max())
      {
        break;
      }
    }
    markContentChanged();
  }

  void removePoint(uint32_t index) override
  {
    checkSlot(index);
    const float p[3] = {xs_[index], ys_[index], zs_[index]};
    int64_t     c[3];
    if (!cellOf(p, c))
    {
      return;  // never indexed
    }
    const auto it = key2voxel_.find(keyOf(c));
    if (it == key2voxel_.end())
    {
      return;
    }
    const uint32_t vi  = it->second;
    auto&          pts = voxels_[vi].pts;
    for (std::size_t k = 0; k < pts.size(); k++)
    {
      if (pts[k].slot != index)
      {
        continue;
      }
      pts[k] = pts.back();
      pts.pop_back();
      removed_.push_back(index);
      live_--;
      if (pts.empty())
      {
        dropVoxel(it->first, vi);
      }
      markContentChanged();
      return;
    }
  }

  void keepOnlyPointsInsideBox(const float minCorner[3], const float maxCorner[3]) override
  {
    std::vector<voxel_key_t> emptied;
    for (const auto& [key, vi] : key2voxel_)
    {
      auto&         pts  = voxels_[vi].pts;
      const int64_t c[3] = {key.cx, key.cy, key.cz};

      // Any point of the voxel lies within its box widened by the slack:
      bool allIn  = true;
      bool allOut = false;
      for (int a = 0; a < 3; a++)
      {
        const double lo = static_cast<double>(c[a]) * size_ - slack_;
        const double hi = static_cast<double>(c[a] + 1) * size_ + slack_;
        allIn           = allIn && lo >= minCorner[a] && hi <= maxCorner[a];
        allOut          = allOut || hi < minCorner[a] || lo > maxCorner[a];
      }
      if (allIn)
      {
        continue;
      }

      // Same rule as the k-d tree: keep minCorner <= p <= maxCorner.
      std::size_t kept = 0;
      for (const Entry& e : pts)
      {
        const bool inside = !allOut && e.x >= minCorner[0] && e.x <= maxCorner[0] &&
                            e.y >= minCorner[1] && e.y <= maxCorner[1] && e.z >= minCorner[2] &&
                            e.z <= maxCorner[2];
        if (inside)
        {
          pts[kept++] = e;
        }
        else
        {
          removed_.push_back(e.slot);
          live_--;
        }
      }
      pts.resize(kept);
      if (kept == 0)
      {
        emptied.push_back(key);
      }
    }
    for (const auto& key : emptied)
    {
      dropVoxel(key, key2voxel_.at(key));
    }

    // Still a superset of the live points, as the interface asks:
    if (live_ == 0)
    {
      hasPoints_ = false;
    }
    else
    {
      int64_t cMin[3];
      int64_t cMax[3];
      for (int a = 0; a < 3; a++)
      {
        bbMin_[a] = std::max(bbMin_[a], minCorner[a]);
        bbMax_[a] = std::min(bbMax_[a], maxCorner[a]);
      }
      if (cellOf(bbMin_, cMin) && cellOf(bbMax_, cMax))
      {
        for (int a = 0; a < 3; a++)
        {
          cellMin_[a] = std::max(cellMin_[a], cMin[a]);
          cellMax_[a] = std::min(cellMax_[a], cMax[a]);
        }
      }
    }
    markContentChanged();
  }

  [[nodiscard]] std::vector<uint32_t> acquireRemovedPoints() override
  {
    std::vector<uint32_t> out;
    out.swap(removed_);
    return out;
  }

  void waitForPendingRebuilds() override {}

  void saveIndex([[maybe_unused]] std::ostream& stream) override
  {
    throw std::logic_error("VoxelHashIndex: only the k-d tree index can be serialized");
  }

  void loadIndex([[maybe_unused]] std::istream& stream) override
  {
    throw std::logic_error("VoxelHashIndex: only the k-d tree index can be serialized");
  }

  [[nodiscard]] std::size_t size() const override { return live_; }

  // Removal is immediate, so there are no tombstones:
  [[nodiscard]] std::size_t physicalSize() const override { return live_; }

  void snapshotLiveIndices(std::vector<uint32_t>& out) const override
  {
    out.reserve(out.size() + live_);
    for (const Voxel& v : voxels_)
    {
      for (const Entry& e : v.pts)
      {
        out.push_back(e.slot);
      }
    }
  }

  [[nodiscard]] bool boundingBox(float minCorner[3], float maxCorner[3]) const override
  {
    if (live_ == 0 || !hasPoints_)
    {
      return false;
    }
    for (int a = 0; a < 3; a++)
    {
      minCorner[a] = bbMin_[a];
      maxCorner[a] = bbMax_[a];
    }
    return true;
  }

  [[nodiscard]] std::size_t knnSearch(
      const float query[3], std::size_t k, uint32_t* outIndices, float* outDistsSqr) const override
  {
    return knn(query, k, std::numeric_limits<float>::infinity(), outIndices, outDistsSqr);
  }

  [[nodiscard]] std::size_t knnSearchWithinRadius(
      const float query[3], std::size_t k, float maxDistSqr, uint32_t* outIndices,
      float* outDistsSqr) const override
  {
    return knn(query, k, maxDistSqr, outIndices, outDistsSqr);
  }

  void radiusSearch(
      const float query[3], float radiusSqr, std::vector<Neighbor>& out) const override
  {
    if (live_ == 0 || !(radiusSqr > 0) || !isFinite(query))
    {
      return;
    }

    const std::size_t first = out.size();

    const auto visit = [&](const Voxel& v)
    {
      for (const Entry& e : v.pts)
      {
        const float d2 = sqrDist(e, query);
        if (d2 < radiusSqr)
        {
          out.push_back({e.slot, d2});
        }
      }
    };
    const auto restart = [&]() { out.resize(first); };

    search(query, std::sqrt(static_cast<double>(radiusSqr)), radiusSqr, visit, restart);

    // Closest first, as nanoflann returns them:
    std::sort(
        out.begin() + static_cast<std::ptrdiff_t>(first), out.end(),
        [](const Neighbor& a, const Neighbor& b)
        { return a.dist_sqr < b.dist_sqr || (a.dist_sqr == b.dist_sqr && a.index < b.index); });
  }

 private:
  struct Entry
  {
    float    x, y, z;
    uint32_t slot;
  };

  struct Voxel
  {
    voxel_key_t        key;
    std::vector<Entry> pts;
  };

  const double size_;
  /// Bucketing a point divides in double, so it can land in a voxel whose
  /// exact box misses it by far less than this; every voxel box is widened by
  /// it.
  const double slack_;

  const float* xs_    = nullptr;
  const float* ys_    = nullptr;
  const float* zs_    = nullptr;
  std::size_t  count_ = 0;

  tsl::robin_map<voxel_key_t, uint32_t, mola::index3d_hash<int32_t>> key2voxel_;
  std::vector<Voxel>                                                 voxels_;
  std::vector<uint32_t>                                              freeVoxels_;
  std::vector<uint32_t>                                              removed_;
  std::size_t                                                        live_ = 0;

  /// Conservative bounds of the points ever inserted, and of their voxels,
  /// since the last clear(), trimmed by keepOnlyPointsInsideBox():
  bool    hasPoints_  = false;
  float   bbMin_[3]   = {0, 0, 0};
  float   bbMax_[3]   = {0, 0, 0};
  int64_t cellMin_[3] = {0, 0, 0};
  int64_t cellMax_[3] = {0, 0, 0};

  /// The coordinates of a slot are read from the bound buffers.
  void checkSlot(uint32_t i) const
  {
    if (i >= count_)
    {
      throw std::out_of_range(
          "VoxelHashIndex: slot " + std::to_string(i) + " is past the " + std::to_string(count_) +
          " points of the bound buffers (missing setPointBuffers() call?)");
    }
  }

  static bool isFinite(const float p[3])
  {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
  }

  static float sqrDist(const Entry& e, const float q[3])
  {
    const float dx = e.x - q[0];
    const float dy = e.y - q[1];
    const float dz = e.z - q[2];
    return dx * dx + dy * dy + dz * dz;
  }

  /** Voxel of a point: voxel `i` spans `[i*size, (i+1)*size)` along each axis.
   *  \return false if the point cannot be indexed: non-finite, or farther than
   *  MAX_ABS_CELL voxels from the origin.
   */
  bool cellOf(const float p[3], int64_t c[3]) const
  {
    for (int a = 0; a < 3; a++)
    {
      const double v = std::floor(static_cast<double>(p[a]) / size_);
      if (!(std::abs(v) <= MAX_ABS_CELL))
      {
        return false;  // also rejects NaN
      }
      c[a] = static_cast<int64_t>(v);
    }
    return true;
  }

  static voxel_key_t keyOf(const int64_t c[3])
  {
    return {static_cast<int32_t>(c[0]), static_cast<int32_t>(c[1]), static_cast<int32_t>(c[2])};
  }

  const Voxel* findVoxel(const int64_t c[3]) const
  {
    for (int a = 0; a < 3; a++)
    {
      if (static_cast<double>(std::abs(c[a])) > MAX_ABS_CELL)
      {
        return nullptr;
      }
    }
    const auto it = key2voxel_.find(keyOf(c));
    return it == key2voxel_.end() ? nullptr : &voxels_[it->second];
  }

  /// Lower bound of the squared distance from `q` to any point stored in voxel
  /// `c`: that to its box, widened by the slack.
  double voxelDistSqr(const double q[3], const int64_t c[3]) const
  {
    double d2 = 0;
    for (int a = 0; a < 3; a++)
    {
      const double lo = static_cast<double>(c[a]) * size_ - slack_;
      const double hi = static_cast<double>(c[a] + 1) * size_ + slack_;
      if (q[a] < lo)
      {
        d2 += (lo - q[a]) * (lo - q[a]);
      }
      else if (q[a] > hi)
      {
        d2 += (q[a] - hi) * (q[a] - hi);
      }
    }
    return d2;
  }

  /// Whether a voxel at (at least) squared distance `lowerBound2` may hold a
  /// point accepted by a bound `bound2`.
  static bool reachable(double lowerBound2, float bound2)
  {
    return lowerBound2 * PRUNE_SHRINK <= static_cast<double>(bound2);
  }

  void insertOne(uint32_t i)
  {
    checkSlot(i);
    const float p[3] = {xs_[i], ys_[i], zs_[i]};
    int64_t     c[3];
    if (!cellOf(p, c))
    {
      return;  // non-finite (e.g. a blanked free slot) or absurdly far
    }

    const voxel_key_t key = keyOf(c);
    uint32_t          vi  = 0;
    if (const auto it = key2voxel_.find(key); it != key2voxel_.end())
    {
      vi = it->second;
    }
    else
    {
      if (!freeVoxels_.empty())
      {
        vi = freeVoxels_.back();
        freeVoxels_.pop_back();
      }
      else
      {
        vi = static_cast<uint32_t>(voxels_.size());
        voxels_.emplace_back();
      }
      voxels_[vi].key = key;
      key2voxel_.emplace(key, vi);
    }
    voxels_[vi].pts.push_back({p[0], p[1], p[2], i});
    live_++;

    if (!hasPoints_)
    {
      for (int a = 0; a < 3; a++)
      {
        bbMin_[a] = bbMax_[a] = p[a];
        cellMin_[a] = cellMax_[a] = c[a];
      }
      hasPoints_ = true;
    }
    else
    {
      for (int a = 0; a < 3; a++)
      {
        bbMin_[a]   = std::min(bbMin_[a], p[a]);
        bbMax_[a]   = std::max(bbMax_[a], p[a]);
        cellMin_[a] = std::min(cellMin_[a], c[a]);
        cellMax_[a] = std::max(cellMax_[a], c[a]);
      }
    }
  }

  /// Unlinks an emptied voxel; its storage, and the capacity of its point
  /// vector, are reused by the next new voxel.
  void dropVoxel(const voxel_key_t& key, uint32_t vi)
  {
    key2voxel_.erase(key);
    voxels_[vi].pts.clear();
    freeVoxels_.push_back(vi);
  }

  /// Cells a ring search may examine before a scan of the occupied voxels is
  /// cheaper.
  std::size_t cellBudget() const { return 64 + 4 * key2voxel_.size(); }

  /** Runs `visit` on every occupied voxel that may hold a point within
   *  `bound2` of the query (a bound `visit` may shrink as it goes), stopping
   *  early once no remaining voxel can: ring by ring around the query voxel,
   *  up to `radius` if finite, unless that would examine more cells than a
   *  scan of all occupied voxels, which is then done instead (after calling
   *  `restart`, which must undo whatever `visit` already recorded).
   */
  template <class VISIT, class RESTART>
  void search(
      const float q[3], double radius, const float& bound2, VISIT&& visit, RESTART&& restart) const
  {
    const double qd[3] = {q[0], q[1], q[2]};

    int64_t c[3];
    if (cellOf(q, c))
    {
      // Rings needed to reach the nearest occupied voxel, and to cover them all:
      int64_t reach = 0;
      int64_t cover = 0;
      for (int a = 0; a < 3; a++)
      {
        reach = std::max({reach, cellMin_[a] - c[a], c[a] - cellMax_[a]});
        cover = std::max({cover, c[a] - cellMin_[a], cellMax_[a] - c[a]});
      }
      if (std::isfinite(radius) && radius / size_ < MAX_ABS_CELL)
      {
        cover = std::min(cover, static_cast<int64_t>(std::ceil(radius / size_)) + 1);
      }
      // Every cell of the rings short of `reach` is examined in vain:
      const double side = 2.0 * static_cast<double>(reach) + 1.0;
      if (side * side * side <= static_cast<double>(cellBudget()))
      {
        if (visitRings(qd, c, cover, bound2, visit))
        {
          return;
        }
        restart();
      }
    }
    for (const Voxel& v : voxels_)
    {
      if (v.pts.empty())
      {
        continue;
      }
      const int64_t vc[3] = {v.key.cx, v.key.cy, v.key.cz};
      if (reachable(voxelDistSqr(qd, vc), bound2))
      {
        visit(v);
      }
    }
  }

  /** The ring part of search(). Ring `r` is the shell of voxels at Chebyshev
   *  distance `r` from the query voxel `c`.
   *  \return false if it gave up after examining cellBudget() cells.
   */
  template <class VISIT>
  bool visitRings(
      const double q[3], const int64_t c[3], int64_t maxRing, const float& bound2,
      VISIT&& visit) const
  {
    const std::size_t budget   = cellBudget();
    std::size_t       examined = 0;
    for (int64_t r = 0; r <= maxRing; r++)
    {
      if (r > 0)
      {
        // Distance from q to the outside of the block of rings below r, which
        // bounds that of every voxel left:
        double toOutside = std::numeric_limits<double>::max();
        for (int a = 0; a < 3; a++)
        {
          const double lo = static_cast<double>(c[a] - (r - 1)) * size_ - slack_;
          const double hi = static_cast<double>(c[a] + r) * size_ + slack_;
          toOutside       = std::min({toOutside, q[a] - lo, hi - q[a]});
        }
        toOutside = std::max(toOutside, 0.0);
        if (!reachable(toOutside * toOutside, bound2))
        {
          return true;
        }
      }
      for (int64_t dx = -r; dx <= r; dx++)
      {
        const bool xEdge = dx == -r || dx == r;
        for (int64_t dy = -r; dy <= r; dy++)
        {
          // Only the shell: every z on its x and y faces, else its two z faces.
          const bool    xyEdge = xEdge || dy == -r || dy == r;
          const int64_t dzStep = (xyEdge || r == 0) ? 1 : 2 * r;
          for (int64_t dz = -r; dz <= r; dz += dzStep)
          {
            if (++examined > budget)
            {
              return false;
            }
            const int64_t v[3] = {c[0] + dx, c[1] + dy, c[2] + dz};
            // The box bound needs no hash lookup, so check it first:
            if (!reachable(voxelDistSqr(q, v), bound2))
            {
              continue;
            }
            if (const Voxel* vox = findVoxel(v); vox != nullptr)
            {
              visit(*vox);
            }
          }
        }
      }
    }
    return true;
  }

  std::size_t knn(
      const float q[3], std::size_t k, float maxDistSqr, uint32_t* outIndices,
      float* outDistsSqr) const
  {
    if (k == 0 || live_ == 0 || !isFinite(q))
    {
      return 0;
    }

    std::size_t found = 0;
    float       bound = maxDistSqr;  // the k-th distance once k are found

    const auto visit = [&](const Voxel& v)
    {
      for (const Entry& e : v.pts)
      {
        const float d2 = sqrDist(e, q);
        if (!(d2 < bound))
        {
          continue;
        }
        // Sorted insertion, after any equal distance (as nanoflann does). When
        // full, the last one drops out.
        std::size_t j = found < k ? found++ : k - 1;
        while (j > 0 && outDistsSqr[j - 1] > d2)
        {
          outDistsSqr[j] = outDistsSqr[j - 1];
          outIndices[j]  = outIndices[j - 1];
          j--;
        }
        outDistsSqr[j] = d2;
        outIndices[j]  = e.slot;
        if (found == k)
        {
          bound = outDistsSqr[k - 1];
        }
      }
    };
    const auto restart = [&]()
    {
      found = 0;
      bound = maxDistSqr;
    };

    search(q, std::sqrt(static_cast<double>(maxDistSqr)), bound, visit, restart);
    return found;
  }
};
}  // namespace

namespace mola::internal
{
std::unique_ptr<IncrementalKDTree> CreateVoxelHashIndex(float voxelSize)
{
  return std::make_unique<VoxelHashIndex>(voxelSize);
}
}  // namespace mola::internal
