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
 * @brief  Exact nearest-neighbor index over a hashed voxel grid, as an
 *         alternative backend of mola::internal::IncrementalKDTree.
 *
 * EXPERIMENTAL (performance study). Every voxel keeps a copy of the
 * coordinates of its points, contiguously, plus their storage slots, so a
 * query touches a few short arrays instead of chasing one pointer per point as
 * the one-point-per-node incremental k-d tree does. Queries are exact: voxels
 * are visited in Chebyshev rings around the query voxel, each one skipped when
 * its box lies farther than the current k-th distance, and the expansion stops
 * as soon as no unvisited voxel can hold a closer point.
 *
 * Insertions and box evictions are O(1) per point, with no rebuild at all, so
 * there is no tail latency to hide in a background thread.
 */

#include <tsl/robin_map.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "IncrementalKDTree.h"

namespace
{
struct MixHash
{
  std::size_t operator()(uint64_t x) const noexcept
  {
    // splitmix64 finalizer: the robin map uses power-of-two bucket counts, so
    // the packed voxel coordinates must be mixed before masking.
    x ^= x >> 30U;
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27U;
    x *= 0x94d049bb133111ebULL;
    x ^= x >> 31U;
    return static_cast<std::size_t>(x);
  }
};

class VoxelHashIndex : public mola::internal::IncrementalKDTree
{
 public:
  explicit VoxelHashIndex(float voxelSize) : s_(voxelSize), inv_(1.0f / voxelSize) {}

  void setPointBuffers(const float* xs, const float* ys, const float* zs, std::size_t count) override
  {
    xs_    = xs;
    ys_    = ys;
    zs_    = zs;
    count_ = count;
  }

  void reserve(std::size_t n) override { key2v_.reserve(n / 8); }

  void clear() override
  {
    key2v_.clear();
    vox_.clear();
    freeVox_.clear();
    removed_.clear();
    live_ = 0;
    bbValid_ = false;
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
      if (i == std::numeric_limits<uint32_t>::max()) break;
    }
    markContentChanged();
  }

  void removePoint(uint32_t index) override
  {
    const float p[3] = {xs_[index], ys_[index], zs_[index]};
    auto        it   = key2v_.find(keyOf(p));
    if (it == key2v_.end()) return;
    Voxel& v = vox_[it->second];
    for (std::size_t k = 0; k < v.slot.size(); k++)
    {
      if (v.slot[k] != index) continue;
      eraseAt(v, k);
      removed_.push_back(index);
      live_--;
      if (v.slot.empty()) dropVoxel(it->first, it->second);
      markContentChanged();
      return;
    }
  }

  void keepOnlyPointsInsideBox(const float mn[3], const float mx[3]) override
  {
    std::vector<uint64_t> emptied;
    for (auto it = key2v_.begin(); it != key2v_.end(); ++it)
    {
      Voxel&      v       = vox_[it->second];
      const float lo[3]   = {v.ix * s_, v.iy * s_, v.iz * s_};
      bool        allIn   = true;
      bool        allOut  = false;
      for (int a = 0; a < 3; a++)
      {
        if (lo[a] + s_ < mn[a] || lo[a] > mx[a]) allOut = true;
        if (lo[a] < mn[a] || lo[a] + s_ > mx[a]) allIn = false;
      }
      if (allIn) continue;

      std::size_t w = 0;
      for (std::size_t k = 0; k < v.slot.size(); k++)
      {
        const float* p  = &v.xyz[3 * k];
        const bool   in = !allOut && p[0] >= mn[0] && p[0] <= mx[0] && p[1] >= mn[1] &&
                        p[1] <= mx[1] && p[2] >= mn[2] && p[2] <= mx[2];
        if (!in)
        {
          removed_.push_back(v.slot[k]);
          live_--;
          continue;
        }
        if (w != k)
        {
          v.slot[w]          = v.slot[k];
          v.xyz[3 * w + 0] = p[0];
          v.xyz[3 * w + 1] = p[1];
          v.xyz[3 * w + 2] = p[2];
        }
        w++;
      }
      if (w != v.slot.size())
      {
        v.slot.resize(w);
        v.xyz.resize(3 * w);
      }
      if (w == 0) emptied.push_back(it->first);
    }
    for (const uint64_t k : emptied)
    {
      auto it = key2v_.find(k);
      dropVoxel(k, it->second);
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

  void saveIndex(std::ostream&) override
  {
    throw std::runtime_error("VoxelHashIndex: saveIndex() not supported");
  }
  void loadIndex(std::istream&) override
  {
    throw std::runtime_error("VoxelHashIndex: loadIndex() not supported");
  }

  [[nodiscard]] std::size_t size() const override { return live_; }
  [[nodiscard]] std::size_t physicalSize() const override { return live_; }

  void snapshotLiveIndices(std::vector<uint32_t>& out) const override
  {
    out.reserve(out.size() + live_);
    for (const auto& kv : key2v_)
    {
      const Voxel& v = vox_[kv.second];
      out.insert(out.end(), v.slot.begin(), v.slot.end());
    }
  }

  [[nodiscard]] bool boundingBox(float mn[3], float mx[3]) const override
  {
    if (live_ == 0 || !bbValid_) return false;
    for (int a = 0; a < 3; a++)
    {
      mn[a] = bbMin_[a];
      mx[a] = bbMax_[a];
    }
    return true;
  }

  [[nodiscard]] std::size_t knnSearch(
      const float q[3], std::size_t k, uint32_t* outIdx, float* outD2) const override
  {
    return knn(q, k, std::numeric_limits<float>::max(), outIdx, outD2, maxRingForMap(q));
  }

  [[nodiscard]] std::size_t knnSearchWithinRadius(
      const float q[3], std::size_t k, float maxD2, uint32_t* outIdx, float* outD2) const override
  {
    const int maxRing = std::min(
        maxRingForMap(q), static_cast<int>(std::ceil(std::sqrt(maxD2) * inv_)) + 1);
    return knn(q, k, maxD2, outIdx, outD2, maxRing);
  }

  void radiusSearch(const float q[3], float r2, std::vector<Neighbor>& out) const override
  {
    float     bound   = r2;
    const int maxRing = std::min(
        maxRingForMap(q), static_cast<int>(std::ceil(std::sqrt(r2) * inv_)) + 1);
    visitRings(
        q, maxRing, bound,
        [&](const Voxel& v)
        {
          const std::size_t n = v.slot.size();
          for (std::size_t k = 0; k < n; k++)
          {
            const float dx = v.xyz[3 * k] - q[0];
            const float dy = v.xyz[3 * k + 1] - q[1];
            const float dz = v.xyz[3 * k + 2] - q[2];
            const float d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < r2) out.push_back({v.slot[k], d2});
          }
        });
  }

 private:
  struct Voxel
  {
    std::vector<float>    xyz;  ///< interleaved copies of the coordinates
    std::vector<uint32_t> slot;  ///< storage slot of each point
    int32_t               ix = 0, iy = 0, iz = 0;
  };

  float s_, inv_;

  const float* xs_    = nullptr;
  const float* ys_    = nullptr;
  const float* zs_    = nullptr;
  std::size_t  count_ = 0;

  tsl::robin_map<uint64_t, uint32_t, MixHash> key2v_;
  std::vector<Voxel>                          vox_;
  std::vector<uint32_t>                       freeVox_;
  std::vector<uint32_t>                       removed_;
  std::size_t                                 live_ = 0;

  bool  bbValid_  = false;
  float bbMin_[3] = {0, 0, 0}, bbMax_[3] = {0, 0, 0};

  [[nodiscard]] int32_t cell(float v) const { return static_cast<int32_t>(std::floor(v * inv_)); }

  static uint64_t pack(int32_t x, int32_t y, int32_t z)
  {
    constexpr uint32_t OFF  = 1U << 20U;
    constexpr uint64_t MASK = 0x1FFFFF;
    return ((static_cast<uint64_t>(static_cast<uint32_t>(x) + OFF) & MASK) << 42U) |
           ((static_cast<uint64_t>(static_cast<uint32_t>(y) + OFF) & MASK) << 21U) |
           (static_cast<uint64_t>(static_cast<uint32_t>(z) + OFF) & MASK);
  }
  [[nodiscard]] uint64_t keyOf(const float p[3]) const
  {
    return pack(cell(p[0]), cell(p[1]), cell(p[2]));
  }

  void insertOne(uint32_t i)
  {
    const float p[3] = {xs_[i], ys_[i], zs_[i]};
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) return;

    const int32_t  ix = cell(p[0]), iy = cell(p[1]), iz = cell(p[2]);
    const uint64_t k  = pack(ix, iy, iz);
    auto           it = key2v_.find(k);
    uint32_t       vi = 0;
    if (it == key2v_.end())
    {
      if (!freeVox_.empty())
      {
        vi = freeVox_.back();
        freeVox_.pop_back();
      }
      else
      {
        vi = static_cast<uint32_t>(vox_.size());
        vox_.emplace_back();
      }
      Voxel& v = vox_[vi];
      v.ix     = ix;
      v.iy     = iy;
      v.iz     = iz;
      key2v_.emplace(k, vi);
    }
    else
    {
      vi = it->second;
    }
    Voxel& v = vox_[vi];
    v.slot.push_back(i);
    v.xyz.push_back(p[0]);
    v.xyz.push_back(p[1]);
    v.xyz.push_back(p[2]);
    live_++;

    if (!bbValid_)
    {
      for (int a = 0; a < 3; a++) bbMin_[a] = bbMax_[a] = p[a];
      bbValid_ = true;
    }
    else
    {
      for (int a = 0; a < 3; a++)
      {
        bbMin_[a] = std::min(bbMin_[a], p[a]);
        bbMax_[a] = std::max(bbMax_[a], p[a]);
      }
    }
  }

  static void eraseAt(Voxel& v, std::size_t k)
  {
    const std::size_t last = v.slot.size() - 1;
    v.slot[k]              = v.slot[last];
    v.xyz[3 * k + 0]       = v.xyz[3 * last + 0];
    v.xyz[3 * k + 1]       = v.xyz[3 * last + 1];
    v.xyz[3 * k + 2]       = v.xyz[3 * last + 2];
    v.slot.pop_back();
    v.xyz.resize(3 * last);
  }

  void dropVoxel(uint64_t key, uint32_t vi)
  {
    key2v_.erase(key);
    Voxel& v = vox_[vi];
    v.slot.clear();
    v.xyz.clear();
    freeVox_.push_back(vi);
  }

  /// Upper bound on the rings worth visiting: enough to cover the whole map
  /// AABB from the query voxel.
  [[nodiscard]] int maxRingForMap(const float q[3]) const
  {
    if (!bbValid_) return 0;
    float far = 0;
    for (int a = 0; a < 3; a++)
    {
      far = std::max({far, std::abs(q[a] - bbMin_[a]), std::abs(q[a] - bbMax_[a])});
    }
    return static_cast<int>(std::ceil(far * inv_)) + 1;
  }

  /** Visits the voxels in Chebyshev rings of growing radius around the query
   *  voxel, skipping those whose box is farther than `bound2` (which the
   *  visitor may shrink), and stopping once a whole ring is out of reach.
   */
  template <class F>
  void visitRings(const float q[3], int maxRing, float& bound2, F&& visit) const
  {
    const int32_t c[3] = {cell(q[0]), cell(q[1]), cell(q[2])};
    for (int r = 0; r <= maxRing; r++)
    {
      if (r > 0)
      {
        // Distance from q to the boundary of the block of rings < r:
        float lb = std::numeric_limits<float>::max();
        for (int a = 0; a < 3; a++)
        {
          const float lo = static_cast<float>(c[a] - (r - 1)) * s_;
          const float hi = static_cast<float>(c[a] + r) * s_;
          lb             = std::min({lb, q[a] - lo, hi - q[a]});
        }
        if (lb * lb > bound2) return;
      }
      for (int dx = -r; dx <= r; dx++)
      {
        const bool xEdge = dx == -r || dx == r;
        for (int dy = -r; dy <= r; dy++)
        {
          const bool xyEdge = xEdge || dy == -r || dy == r;
          // Only the shell of the ring: full z range on the x/y faces, else
          // just its two z caps.
          const int dzStep = xyEdge ? 1 : (r == 0 ? 1 : 2 * r);
          for (int dz = -r; dz <= r; dz += dzStep)
          {
            // Lower bound from q to this voxel box, from its integer
            // coordinates alone, so that out-of-reach voxels cost no hash
            // lookup at all:
            const int32_t vi[3] = {c[0] + dx, c[1] + dy, c[2] + dz};
            float         d2    = 0;
            for (int a = 0; a < 3; a++)
            {
              const float lo = static_cast<float>(vi[a]) * s_;
              const float hi = lo + s_;
              if (q[a] < lo)
                d2 += (lo - q[a]) * (lo - q[a]);
              else if (q[a] > hi)
                d2 += (q[a] - hi) * (q[a] - hi);
            }
            if (d2 > bound2) continue;

            auto it = key2v_.find(pack(vi[0], vi[1], vi[2]));
            if (it == key2v_.end()) continue;
            visit(vox_[it->second]);
          }
        }
      }
    }
  }

  std::size_t knn(
      const float q[3], std::size_t K, float maxD2, uint32_t* idxs, float* d2s, int maxRing) const
  {
    if (K == 0 || live_ == 0) return 0;
    std::size_t cnt   = 0;
    float       worst = maxD2;
    visitRings(
        q, maxRing, worst,
        [&](const Voxel& v)
        {
          const std::size_t n = v.slot.size();
          const float*      p = v.xyz.data();
          for (std::size_t k = 0; k < n; k++)
          {
            const float dx = p[3 * k] - q[0];
            const float dy = p[3 * k + 1] - q[1];
            const float dz = p[3 * k + 2] - q[2];
            const float dd = dx * dx + dy * dy + dz * dz;
            if (dd >= worst) continue;
            std::size_t j = cnt < K ? cnt++ : K - 1;
            while (j > 0 && d2s[j - 1] > dd)
            {
              d2s[j]  = d2s[j - 1];
              idxs[j] = idxs[j - 1];
              j--;
            }
            d2s[j]  = dd;
            idxs[j] = v.slot[k];
            if (cnt == K) worst = d2s[K - 1];
          }
        });
    return cnt;
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
