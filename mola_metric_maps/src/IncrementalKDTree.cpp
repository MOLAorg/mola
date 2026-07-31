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
 * @file   IncrementalKDTree.cpp
 * @brief  Opaque wrapper over nanoflann's incremental (self-balancing) k-d tree
 * @author Jose Luis Blanco Claraco
 * @date   Jul 25, 2026
 */

// ---------------------------------------------------------------------------
//  This translation unit is the ONLY place in the library where nanoflann is
//  included, and it deliberately includes no MRPT header.
//
//  MRPT bundles its own copy of nanoflann and its shared libraries are compiled
//  against it. That copy is usually older than the one providing the
//  incremental index used here, and the two versions differ in non-template
//  entities (e.g. the pooled allocator) that would end up sharing a mangled
//  name at link time. Mixing them is an ODR violation with real
//  memory-corruption potential, so the namespace is renamed for our private
//  copy, which makes every symbol distinct.
//
//  For the same reason the header is included by ABSOLUTE path (MOLA_NANOFLANN_HEADER,
//  set by CMake): MRPT exports its own copy through an -isystem directory, and a
//  directory given both as -I and -isystem is deduplicated by GCC in favor of the
//  system one, so no include-path ordering can reliably select ours.
//
//  Consequences to preserve when editing this file:
//   - do not include any MRPT (or other nanoflann-using) header here;
//   - do not expose any nanoflann type through IncrementalKDTree.h.
// ---------------------------------------------------------------------------
#if !defined(MOLA_NANOFLANN_HEADER)
#define MOLA_NANOFLANN_HEADER <nanoflann.hpp>
#endif

#define nanoflann mola_nanoflann
#include MOLA_NANOFLANN_HEADER
#undef nanoflann

#if !defined(NANOFLANN_VERSION) || NANOFLANN_VERSION < 0x010A00
#error "mola::IncrementalPointCloud requires nanoflann >= 1.10.0 (incremental index)"
#endif

#include "IncrementalKDTree.h"

#include <algorithm>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <type_traits>

namespace
{
/** nanoflann dataset adaptor over three externally-owned coordinate arrays. */
struct PointBuffers
{
  const float* coords[3] = {nullptr, nullptr, nullptr};
  std::size_t  count     = 0;

  [[nodiscard]] inline std::size_t kdtree_get_point_count() const { return count; }

  [[nodiscard]] inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const
  {
    return coords[dim][idx];
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const
  {
    return false;
  }
};

using metric_t = mola_nanoflann::L2_Simple_Adaptor<float, PointBuffers>;

using sync_index_t =
    mola_nanoflann::KDTreeSingleIndexIncrementalAdaptor<metric_t, PointBuffers, 3, uint32_t>;

using async_index_t =
    mola_nanoflann::KDTreeSingleIndexIncrementalAdaptorMT<metric_t, PointBuffers, 3, uint32_t>;

template <class INDEX>
class IncrementalKDTreeImpl : public mola::internal::IncrementalKDTree
{
 public:
  explicit IncrementalKDTreeImpl(const Params& p) : params_(p.alpha_balance, p.alpha_deleted)
  {
    createIndex();
  }

  void setPointBuffers(
      const float* xs, const float* ys, const float* zs, std::size_t count) override
  {
    // Written only when something actually changed: the MT worker reads these
    // while it rebuilds, and the owner calls this on every query/insertion.
    // The owner guarantees the pointers cannot change while a rebuild is in
    // flight (it blocks before any reallocation); `count` is not consulted by
    // the rebuild path, which walks an explicit index list.
    if (data_.coords[0] != xs || data_.coords[1] != ys || data_.coords[2] != zs)
    {
      data_.coords[0] = xs;
      data_.coords[1] = ys;
      data_.coords[2] = zs;
    }
    if (data_.count != count) data_.count = count;
  }

  void reserve(std::size_t n) override { index_->reserve(n); }

  void clear() override
  {
    index_.reset();  // joins the background worker, if any
    createIndex();
  }

  void addPoint(uint32_t index) override { index_->addPoint(index); }

  void addPoints(uint32_t firstIndex, uint32_t lastIndex) override
  {
    index_->addPoints(firstIndex, lastIndex);
  }

  void removePoint(uint32_t index) override { index_->removePoint(index); }

  void keepOnlyPointsInsideBox(const float minCorner[3], const float maxCorner[3]) override
  {
    typename INDEX::BoundingBox keep;
    mola_nanoflann::resize(keep, 3);
    for (int d = 0; d < 3; d++)
    {
      keep[d].low  = minCorner[d];
      keep[d].high = maxCorner[d];
    }
    index_->removeOutsideBox(keep);
  }

  std::vector<uint32_t> acquireRemovedPoints() override { return index_->acquireRemovedPoints(); }

  void waitForPendingRebuilds() override
  {
    if constexpr (std::is_same_v<INDEX, async_index_t>)
    {
      index_->sync();
    }
  }

  void saveIndex(std::ostream& stream) override
  {
#if NANOFLANN_VERSION >= 0x010B00
    // A background rebuild may be replacing the tree topology right now:
    waitForPendingRebuilds();
    index_->saveIndex(stream);
#else
    (void)stream;
    throw std::runtime_error(
        "mola::IncrementalPointCloud: baking the k-d tree requires nanoflann >= 1.11.0 "
        "(saveIndex()/loadIndex() support for the incremental index); this build was "
        "compiled against an older version.");
#endif
  }

  void loadIndex(std::istream& stream) override
  {
#if NANOFLANN_VERSION >= 0x010B00
    index_->loadIndex(stream);
#else
    (void)stream;
    throw std::runtime_error(
        "mola::IncrementalPointCloud: loading a baked k-d tree requires nanoflann >= 1.11.0; "
        "this build was compiled against an older version.");
#endif
  }

  std::size_t size() const override { return index_->size(); }
  std::size_t physicalSize() const override { return index_->physicalSize(); }

  void snapshotLiveIndices(std::vector<uint32_t>& out) const override
  {
    index_->snapshotLiveIndices(out);
  }

  bool boundingBox(float minCorner[3], float maxCorner[3]) const override
  {
    if (index_->empty()) return false;

    const auto bb = index_->boundingBox();
    for (int d = 0; d < 3; d++)
    {
      minCorner[d] = bb[d].low;
      maxCorner[d] = bb[d].high;
    }
    return true;
  }

  std::size_t knnSearch(
      const float query[3], std::size_t k, uint32_t* outIndices, float* outDistsSqr) const override
  {
    return index_->knnSearch(query, k, outIndices, outDistsSqr);
  }

  std::size_t knnSearchWithinRadius(
      const float query[3], std::size_t k, float maxDistSqr, uint32_t* outIndices,
      float* outDistsSqr) const override
  {
    return index_->rknnSearch(query, k, outIndices, outDistsSqr, maxDistSqr);
  }

  void radiusSearch(
      const float query[3], float radiusSqr, std::vector<Neighbor>& out) const override
  {
    std::vector<mola_nanoflann::ResultItem<uint32_t, float>> matches;
    index_->radiusSearch(query, radiusSqr, matches);

    out.reserve(out.size() + matches.size());
    for (const auto& m : matches)
    {
      out.push_back({m.first, m.second});
    }
  }

 private:
  void createIndex()
  {
    index_ = std::make_unique<INDEX>(3, data_, params_);
    // Always on: the owner relies on it to recycle point slots and keep its
    // storage bounded under churn.
    index_->setCollectRemovedPoints(true);
  }

  /// Referenced (by const&) from the index, hence declared before it.
  PointBuffers data_;

  mola_nanoflann::KDTreeIncrementalIndexParams params_;

  std::unique_ptr<INDEX> index_;
};

}  // namespace

namespace mola::internal
{
IncrementalKDTree::~IncrementalKDTree() = default;

std::unique_ptr<IncrementalKDTree> IncrementalKDTree::Create(const Params& p)
{
  if (p.async_rebuild)
  {
    return std::make_unique<IncrementalKDTreeImpl<async_index_t>>(p);
  }
  return std::make_unique<IncrementalKDTreeImpl<sync_index_t>>(p);
}

}  // namespace mola::internal
