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
 * @file   IncrementalPointCloud.cpp
 * @brief  Sliding-window point cloud with an incremental (self-balancing) k-d tree
 * @author Jose Luis Blanco Claraco
 * @date   Jul 25, 2026
 */

#include <mola_metric_maps/IncrementalPointCloud.h>
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR
#include <mrpt/core/bits_math.h>  // mrpt::square()
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include "IncrementalKDTree.h"

#if defined(MOLA_METRIC_MAPS_USE_TBB)
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#endif

using namespace mola;

namespace
{
/// Trace live/storage/free-slot counts and the map bbox on every insertion.
const bool ENV_DEBUG_STATS = mrpt::get_env<bool>("MOLA_INCREMENTAL_MAP_DEBUG_STATS", false);

/// How many storage slots are sampled to detect in-place coordinate rewrites.
constexpr std::size_t COORDINATES_WATCH_SAMPLES = 16;

/// How many slots each of those samples may probe looking for a non-blanked one.
constexpr std::size_t MAX_COORDINATE_PROBES = 8;

/// Equality that also holds for the NaNs marking blanked (free) storage slots.
bool sameCoordinate(float a, float b) { return a == b || (std::isnan(a) && std::isnan(b)); }

/// The value written into blanked (free) storage slots.
constexpr float NO_POINT = std::numeric_limits<float>::quiet_NaN();
}  // namespace

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
    "mola::IncrementalPointCloud,IncrementalPointCloud", mola::IncrementalPointCloud)

IncrementalPointCloud::TMapDefinition::TMapDefinition() = default;

void IncrementalPointCloud::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& s, const std::string& sectionPrefix)
{
  using namespace std::string_literals;

  if (s.sectionExists(sectionPrefix + "_creationOpts"s))
  {
    creationOpts.loadFromConfigFile(s, sectionPrefix + "_creationOpts"s);
  }

  if (s.sectionExists(sectionPrefix + "_insertOpts"s))
  {
    insertionOpts.loadFromConfigFile(s, sectionPrefix + "_insertOpts"s);
  }

  if (s.sectionExists(sectionPrefix + "_likelihoodOpts"s))
  {
    likelihoodOpts.loadFromConfigFile(s, sectionPrefix + "_likelihoodOpts"s);
  }

  if (s.sectionExists(sectionPrefix + "_renderOpts"s))
  {
    renderOpts.loadFromConfigFile(s, sectionPrefix + "_renderOpts"s);
  }
}

void IncrementalPointCloud::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  creationOpts.dumpToTextStream(out);
  insertionOpts.dumpToTextStream(out);
  likelihoodOpts.dumpToTextStream(out);
  renderOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr IncrementalPointCloud::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const IncrementalPointCloud::TMapDefinition* def =
      dynamic_cast<const IncrementalPointCloud::TMapDefinition*>(&_def);
  ASSERT_(def);

  auto obj = IncrementalPointCloud::Create();

  obj->creationOptions   = def->creationOpts;
  obj->insertionOptions  = def->insertionOpts;
  obj->likelihoodOptions = def->likelihoodOpts;
  obj->renderOptions     = def->renderOpts;

  // Apply the (possibly changed) k-d tree parameters:
  obj->resetIndex();

  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(IncrementalPointCloud, CGenericPointsMap, mola)

// =====================================
// Construction / copy
// =====================================

IncrementalPointCloud::IncrementalPointCloud() { resetIndex(); }

IncrementalPointCloud::~IncrementalPointCloud() = default;

IncrementalPointCloud::IncrementalPointCloud(const IncrementalPointCloud& o) : CGenericPointsMap()
{
  *this = o;
}

IncrementalPointCloud& IncrementalPointCloud::operator=(const IncrementalPointCloud& o)
{
  if (this == &o) return *this;

  std::scoped_lock lck(mtx_, o.mtx_);

  // Copy the live points only: tombstoned storage slots must not be resurrected
  // by the index rebuild below.
  const auto compacted = o.liveCompactedCopy();

  // Copies point coordinates and all per-point fields (and clears us first):
  CPointsMap::operator=(*compacted);

  creationOptions   = o.creationOptions;
  insertionOptions  = o.insertionOptions;
  likelihoodOptions = o.likelihoodOptions;
  renderOptions     = o.renderOptions;
  genericMapParams  = o.genericMapParams;

  resetIndex();

  return *this;
}

// =====================================
// Index maintenance
// =====================================

void IncrementalPointCloud::createEmptyIndex()
{
  internal::IncrementalKDTree::Params p;
  p.async_rebuild = creationOptions.async_rebuild;
  p.alpha_balance = creationOptions.alpha_balance;
  p.alpha_deleted = creationOptions.alpha_deleted;

  index_ = internal::IncrementalKDTree::Create(p);

  free_slots_.clear();
  indexed_up_to_ = 0;

  const std::size_t n = m_x.size();
  cov_.assign(n, mrpt::math::CMatrixFloat33());
  cov_valid_.assign(n, 0);

  if (creationOptions.reserve_points != 0)
  {
    const auto n_reserve = static_cast<std::size_t>(creationOptions.reserve_points);
    index_->reserve(n_reserve);
    // Also the coordinate buffers: with a background rebuild worker reading
    // them, avoiding reallocations is what makes insertion cheap (see
    // internal_insertObservation()).
    this->reserve(n_reserve);
  }

  refreshPointBuffers();
  refreshCoordinatesWatch();
}

void IncrementalPointCloud::resetIndex()
{
  auto lck = mrpt::lockHelper(mtx_);

  createEmptyIndex();

  const std::size_t n = m_x.size();
  if (n != 0)
  {
    index_->addPoints(0, static_cast<uint32_t>(n - 1));
    indexed_up_to_ = n;
  }
}

void IncrementalPointCloud::refreshPointBuffers() const
{
  index_->setPointBuffers(m_x.data(), m_y.data(), m_z.data(), m_x.size());
}

void IncrementalPointCloud::rebuildIndexInPlace() const
{
  auto* me = const_cast<IncrementalPointCloud*>(this);

  std::vector<uint32_t> live;
  index_->snapshotLiveIndices(live);

  const std::size_t n = m_x.size();

  if (live.size() == n)
  {
    // No dead slot to preserve: the cheaper bulk path indexes everything.
    me->resetIndex();
    return;
  }

  std::vector<uint8_t> isLive(n, 0);
  for (const uint32_t slot : live)
  {
    if (slot < n) isLive[slot] = 1;
  }

  me->createEmptyIndex();

  for (std::size_t i = 0; i < n; i++)
  {
    if (isLive[i] != 0)
    {
      index_->addPoint(static_cast<uint32_t>(i));
      continue;
    }
    // Dead storage: blank it (see harvestRemovedSlots()) and offer it for reuse.
    me->m_x[i] = NO_POINT;
    me->m_y[i] = NO_POINT;
    me->m_z[i] = NO_POINT;
    free_slots_.push_back(static_cast<uint32_t>(i));
  }
  indexed_up_to_ = n;

  refreshCoordinatesWatch();
}

void IncrementalPointCloud::refreshCoordinatesWatch() const
{
  coordinates_watch_.clear();

  const std::size_t n = m_x.size();
  if (n == 0) return;

  const std::size_t stride = std::max<std::size_t>(1, n / COORDINATES_WATCH_SAMPLES);

  for (std::size_t base = 0; base < n; base += stride)
  {
    // Prefer a finite slot: a blanked one is invariant under a rigid transform,
    // so it could not witness one. The probe is bounded because this runs on
    // the insertion path, where a mostly-blanked storage would otherwise make
    // it O(N).
    const std::size_t last = std::min(n, base + std::min(stride, MAX_COORDINATE_PROBES));

    std::size_t i = base;
    while (i < last && !std::isfinite(m_x[i])) i++;
    if (i >= last) continue;  // all-blanked window: contributes no sample

    coordinates_watch_.emplace_back(
        static_cast<uint32_t>(i), mrpt::math::TPoint3Df(m_x[i], m_y[i], m_z[i]));
  }
}

bool IncrementalPointCloud::coordinatesChangedExternally() const
{
  const std::size_t n = m_x.size();

  for (const auto& [slot, p] : coordinates_watch_)
  {
    if (slot >= n) return true;

    if (!sameCoordinate(m_x[slot], p.x) || !sameCoordinate(m_y[slot], p.y) ||
        !sameCoordinate(m_z[slot], p.z))
    {
      return true;
    }
  }
  return false;
}

void IncrementalPointCloud::reserve(std::size_t newLength)
{
  auto lck = mrpt::lockHelper(mtx_);

  // A background rebuild reads the coordinate buffers, so it must be finished
  // before they can move. Reserving enough storage up front (`reserve_points`)
  // is what keeps this from ever triggering in steady state, where slot
  // recycling stops the storage from growing at all.
  if (index_ && creationOptions.async_rebuild && newLength > m_x.capacity())
  {
    index_->waitForPendingRebuilds();
  }

  CGenericPointsMap::reserve(newLength);

  if (index_) refreshPointBuffers();
}

void IncrementalPointCloud::resize(std::size_t newLength)
{
  auto lck = mrpt::lockHelper(mtx_);

  if (index_ && creationOptions.async_rebuild && newLength > m_x.capacity())
  {
    index_->waitForPendingRebuilds();
  }

  CGenericPointsMap::resize(newLength);

  if (index_) refreshPointBuffers();
}

void IncrementalPointCloud::setSize(std::size_t newLength)
{
  auto lck = mrpt::lockHelper(mtx_);

  if (index_ && creationOptions.async_rebuild && newLength > m_x.capacity())
  {
    index_->waitForPendingRebuilds();
  }

  CGenericPointsMap::setSize(newLength);

  // Unlike resize(), this erases the previous contents, so every slot index the
  // tree holds became meaningless:
  if (index_) resetIndex();
}

void IncrementalPointCloud::ensureIndexUpToDate() const
{
  const std::size_t n = m_x.size();

  if (n < indexed_up_to_)
  {
    // The storage was replaced or truncated behind our back: the slot
    // bookkeeping is no longer valid, so start over.
    const_cast<IncrementalPointCloud*>(this)->resetIndex();
    return;
  }

  if (n == indexed_up_to_)
  {
    // The point count alone cannot tell an untouched map from one whose
    // coordinates were rewritten in place by an inherited (non-virtual)
    // CPointsMap mutator, most notably changeCoordinatesReference() called
    // through a base-class pointer. Those rewrites leave the tree indexing the
    // old coordinates, so they must force a rebuild instead of going unnoticed.
    if (coordinatesChangedExternally())
    {
      rebuildIndexInPlace();
      return;
    }

    refreshPointBuffers();
    return;
  }

  // Points appended through the inherited CPointsMap API: index the new tail.
  cov_.resize(n);
  cov_valid_.resize(n, 0);
  for (std::size_t i = indexed_up_to_; i < n; i++)
  {
    cov_valid_[i] = 0;
  }

  refreshPointBuffers();
  index_->addPoints(static_cast<uint32_t>(indexed_up_to_), static_cast<uint32_t>(n - 1));
  indexed_up_to_ = n;

  refreshCoordinatesWatch();
}

void IncrementalPointCloud::relocatePoint(std::size_t from, std::size_t to)
{
  m_x[to] = m_x[from];
  m_y[to] = m_y[from];
  m_z[to] = m_z[from];

  for (auto& [name, v] : m_float_fields) v[to] = v[from];
  for (auto& [name, v] : m_double_fields) v[to] = v[from];
  for (auto& [name, v] : m_uint16_fields) v[to] = v[from];
  for (auto& [name, v] : m_uint8_fields) v[to] = v[from];
}

void IncrementalPointCloud::indexAppendedPointsRecycling(std::size_t firstAppended)
{
  const std::size_t n = m_x.size();

  if (firstAppended != indexed_up_to_ || n < firstAppended)
  {
    // Contents were replaced rather than appended: rebuild everything.
    resetIndex();
    return;
  }

  if (n == firstAppended) return;  // nothing was appended

  last_insert_batch_ = n - firstAppended;

  cov_.resize(n);
  cov_valid_.resize(n, 0);

  refreshPointBuffers();

  // Move the appended points, starting from the last one, into previously
  // freed slots. This keeps the storage bounded by the size of the sliding
  // window instead of growing with the total number of inserted points.
  std::size_t moved = 0;
  while (moved < n - firstAppended && !free_slots_.empty())
  {
    const uint32_t slot = free_slots_.back();
    free_slots_.pop_back();

    relocatePoint(n - 1 - moved, slot);
    cov_valid_[slot] = 0;
    index_->addPoint(slot);

    moved++;
  }

  if (moved != 0)
  {
    // Drop the relocated tail. Shrinking never reallocates, but the point
    // count seen by the index must be updated.
    resize(n - moved);
    cov_.resize(m_x.size());
    cov_valid_.resize(m_x.size());
    refreshPointBuffers();
  }

  const std::size_t newN = m_x.size();
  if (newN > firstAppended)
  {
    index_->addPoints(static_cast<uint32_t>(firstAppended), static_cast<uint32_t>(newN - 1));
  }
  indexed_up_to_ = newN;

  refreshCoordinatesWatch();
}

void IncrementalPointCloud::harvestRemovedSlots()
{
  // A freed slot keeps the coordinates of the evicted point until some later
  // insertion overwrites it, and a big eviction can free far more slots than
  // the next few scans consume. Those stale coordinates are invisible to the
  // index (and hence to ICP), but NOT to code that walks the inherited
  // CPointsMap storage directly -- most visibly `insertAnotherMap()`, which is
  // how mola_lidar_odometry copies map layers out for publishing, and which
  // would therefore render long-evicted geometry.
  //
  // Blanking them as NaN is what makes a free slot hold no point at all:
  // `insertAnotherMap()` and the other MRPT copy paths skip non-finite points,
  // and a recycled slot is fully overwritten anyway.
  for (const uint32_t slot : index_->acquireRemovedPoints())
  {
    // A slot past the current storage would mean the index and the storage
    // disagree; recycling it would write out of bounds in relocatePoint(), so
    // drop it instead (it costs one leaked slot until the next compaction).
    if (slot >= m_x.size()) continue;

    if (slot < cov_valid_.size()) cov_valid_[slot] = 0;

    m_x[slot] = NO_POINT;
    m_y[slot] = NO_POINT;
    m_z[slot] = NO_POINT;

    free_slots_.push_back(slot);
  }

  refreshCoordinatesWatch();
}

void IncrementalPointCloud::keepOnlyPointsNear(
    const mrpt::math::TPoint3Df& center, double half_side)
{
  auto lck = mrpt::lockHelper(mtx_);

  ensureIndexUpToDate();

  const auto  h     = static_cast<float>(half_side);
  const float mn[3] = {center.x - h, center.y - h, center.z - h};
  const float mx[3] = {center.x + h, center.y + h, center.z + h};

  index_->keepOnlyPointsInsideBox(mn, mx);
  harvestRemovedSlots();

  mark_as_modified();
}

// =====================================
// Global re-map of all points
// =====================================

void IncrementalPointCloud::changeCoordinatesReference(const mrpt::poses::CPose3D& b)
{
  auto lck = mrpt::lockHelper(mtx_);

  ensureIndexUpToDate();

  // A background rebuild reads the coordinate buffers, which are about to be
  // rewritten under it:
  if (creationOptions.async_rebuild) index_->waitForPendingRebuilds();

  // Moves every stored coordinate, blanked free slots included (they stay NaN,
  // since composePoint() propagates it), and marks the inherited caches dirty:
  CPointsMap::changeCoordinatesReference(b);

  // The tree's split planes and bounding boxes were built from the previous
  // coordinates, so none of them survives a global re-map. This also drops the
  // cached per-point covariances, which are expressed in this map's frame.
  rebuildIndexInPlace();
}

void IncrementalPointCloud::changeCoordinatesReference(const mrpt::poses::CPose2D& b)
{
  changeCoordinatesReference(mrpt::poses::CPose3D(b));
}

void IncrementalPointCloud::changeCoordinatesReference(
    const mrpt::maps::CPointsMap& other, const mrpt::poses::CPose3D& b)
{
  auto lck = mrpt::lockHelper(mtx_);

  // Copies points and all per-point fields (clearing us first):
  CPointsMap::operator=(other);
  resetIndex();

  changeCoordinatesReference(b);
}

// =====================================
// Insertion
// =====================================

bool IncrementalPointCloud::internal_insertObservation(
    const mrpt::obs::CObservation& obs, const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  auto lck = mrpt::lockHelper(mtx_);

  const auto tStart = std::chrono::steady_clock::now();

  ensureIndexUpToDate();

  // Evict first, so this very insertion can already recycle the freed slots:
  if (creationOptions.remove_points_farther_than > 0 && robotPose.has_value())
  {
    keepOnlyPointsNear(
        robotPose->translation().cast<float>(), creationOptions.remove_points_farther_than);
  }

  // A background rebuild reads the coordinate buffers, but blocking on it here
  // would defeat the point of running it in the background. Instead the buffers
  // are only ever allowed to move from reserve()/resize()/setSize(), which wait
  // for the worker themselves.
  //
  // `insertAnotherMap()` (the path taken by mp2p_icp_filters::FilterMerge)
  // reserves up front, so it is covered. A hand-written `insertPoint()` loop is
  // not, since push_back can reallocate without going through reserve(), so
  // keep enough spare capacity for a batch of the size we have been seeing.
  if (creationOptions.async_rebuild)
  {
    const std::size_t margin = std::max<std::size_t>(2 * last_insert_batch_, 8192);
    if (m_x.capacity() - m_x.size() < margin) this->reserve(m_x.size() + margin);
  }

  const std::size_t before = m_x.size();

  // Let the base class do the observation-type-specific work (it also honors
  // the standard insertionOptions, e.g. minDistBetweenLaserPoints):
  const bool ok = CPointsMap::internal_insertObservation(obs, robotPose);

  if (insertionOptions.addToExistingPointsMap)
  {
    indexAppendedPointsRecycling(before);
  }
  else
  {
    // The base class replaced the map contents instead of appending, so every
    // slot index changed meaning: rebuild.
    resetIndex();
  }

  if (ENV_DEBUG_STATS)
  {
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tStart)
            .count();
    const auto bb = boundingBox();
    printf(
        "[IncrementalPointCloud] insert_ms=%.1f live=%zu storage=%zu free_slots=%zu "
        "bbox=[%.1f %.1f %.1f]-[%.1f %.1f %.1f]\n",
        ms, index_->size(), m_x.size(), free_slots_.size(), static_cast<double>(bb.min.x),
        static_cast<double>(bb.min.y), static_cast<double>(bb.min.z), static_cast<double>(bb.max.x),
        static_cast<double>(bb.max.y), static_cast<double>(bb.max.z));
  }

  return ok;
}

void IncrementalPointCloud::internal_clear()
{
  auto lck = mrpt::lockHelper(mtx_);

  // A background rebuild reads the coordinate buffers through raw pointers, and
  // the shrink_to_fit() below frees them. resetIndex() at the end of this method
  // would join the worker, but only long after the memory is gone.
  // This is also the path taken by CPointsMap::operator=() (hence by compact()
  // and by deserialization), which clears before refilling.
  if (index_) index_->waitForPendingRebuilds();

  m_x.clear();
  m_y.clear();
  m_z.clear();
  m_x.shrink_to_fit();
  m_y.shrink_to_fit();
  m_z.shrink_to_fit();

  m_float_fields.clear();
  m_double_fields.clear();
  m_uint16_fields.clear();
  m_uint8_fields.clear();

  cachedPoints_.reset();

  resetIndex();

  mark_as_modified();
}

// =====================================
// Data access
// =====================================

std::size_t IncrementalPointCloud::livePointCount() const
{
  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();
  return index_->size();
}

std::size_t IncrementalPointCloud::recyclableSlotCount() const
{
  auto lck = mrpt::lockHelper(mtx_);
  return free_slots_.size();
}

void IncrementalPointCloud::compact()
{
  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  if (index_->size() != m_x.size())
  {
    // Copies points and all per-point fields back onto ourselves, dropping the
    // tombstoned slots on the way (it clears us first, which already resets
    // the index):
    const auto  live = liveCompactedCopy();
    CPointsMap::operator=(*live);
  }

  resetIndex();
}

mrpt::maps::CGenericPointsMap::Ptr IncrementalPointCloud::liveCompactedCopy() const
{
  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  std::vector<uint32_t> live;
  index_->snapshotLiveIndices(live);
  std::sort(live.begin(), live.end());

  auto out = mrpt::maps::CGenericPointsMap::Create();
  out->reserve(live.size());
  out->registerPointFieldsFrom(*this);

  const auto ctx = out->prepareForInsertPointsFrom(*this);
  for (const uint32_t slot : live)
  {
    out->insertPointFrom(slot, ctx);
  }

  return out;
}

mrpt::math::TBoundingBoxf IncrementalPointCloud::boundingBox() const
{
  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  float mn[3];
  float mx[3];
  if (!index_->boundingBox(mn, mx))
  {
    return mrpt::math::TBoundingBoxf(
        mrpt::math::TPoint3Df(0, 0, 0), mrpt::math::TPoint3Df(0, 0, 0));
  }

  return mrpt::math::TBoundingBoxf(
      mrpt::math::TPoint3Df(mn[0], mn[1], mn[2]), mrpt::math::TPoint3Df(mx[0], mx[1], mx[2]));
}

bool IncrementalPointCloud::isEmpty() const { return livePointCount() == 0; }

std::string IncrementalPointCloud::asString() const
{
  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  return mrpt::format(
      "IncrementalPointCloud, points=%zu (storage=%zu, free slots=%zu) bbox=%s", index_->size(),
      m_x.size(), free_slots_.size(), boundingBox().asString().c_str());
}

// =====================================
// NearestNeighborsCapable
// =====================================

bool IncrementalPointCloud::nn_has_indices_or_ids() const
{
  return true;  // storage slot indices, usable with getPointFast()
}

size_t IncrementalPointCloud::nn_index_count() const
{
  auto lck = mrpt::lockHelper(mtx_);
  return m_x.size();
}

void IncrementalPointCloud::nn_prepare_for_3d_queries() const
{
  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();
}

bool IncrementalPointCloud::nn_single_search(
    const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  const float q[3] = {query.x, query.y, query.z};

  uint32_t idx  = 0;
  float    dist = 0;
  if (index_->knnSearch(q, 1, &idx, &dist) == 0) return false;

  result          = {m_x[idx], m_y[idx], m_z[idx]};
  out_dist_sqr    = dist;
  resultIndexOrID = idx;
  return true;
}

void IncrementalPointCloud::nn_multiple_search(
    const mrpt::math::TPoint3Df& query, const size_t N, std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  results.clear();
  out_dists_sqr.clear();
  resultIndicesOrIDs.clear();
  if (N == 0) return;

  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  const float q[3] = {query.x, query.y, query.z};

  std::vector<uint32_t> idxs(N);
  std::vector<float>    dists(N);

  const std::size_t found = index_->knnSearch(q, N, idxs.data(), dists.data());

  results.reserve(found);
  out_dists_sqr.reserve(found);
  resultIndicesOrIDs.reserve(found);
  for (std::size_t i = 0; i < found; i++)
  {
    const uint32_t idx = idxs[i];
    results.push_back({m_x[idx], m_y[idx], m_z[idx]});
    out_dists_sqr.push_back(dists[i]);
    resultIndicesOrIDs.push_back(idx);
  }
}

void IncrementalPointCloud::nn_radius_search(
    const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  results.clear();
  out_dists_sqr.clear();
  resultIndicesOrIDs.clear();
  if (search_radius_sqr <= 0) return;

  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  const float q[3] = {query.x, query.y, query.z};

  const auto lambdaAddResult = [&](uint32_t idx, float distSqr)
  {
    results.push_back({m_x[idx], m_y[idx], m_z[idx]});
    out_dists_sqr.push_back(distSqr);
    resultIndicesOrIDs.push_back(idx);
  };

  if (maxPoints != 0)
  {
    std::vector<uint32_t> idxs(maxPoints);
    std::vector<float>    dists(maxPoints);

    const std::size_t found =
        index_->knnSearchWithinRadius(q, maxPoints, search_radius_sqr, idxs.data(), dists.data());

    for (std::size_t i = 0; i < found; i++) lambdaAddResult(idxs[i], dists[i]);
  }
  else
  {
    std::vector<internal::IncrementalKDTree::Neighbor> matches;
    index_->radiusSearch(q, search_radius_sqr, matches);

    for (const auto& m : matches) lambdaAddResult(m.index, m.dist_sqr);
  }
}

bool IncrementalPointCloud::nn_single_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df& query,
    [[maybe_unused]] mrpt::math::TPoint2Df& result, [[maybe_unused]] float& out_dist_sqr,
    [[maybe_unused]] uint64_t& resultIndexOrID) const
{
  THROW_EXCEPTION("Cannot run a 2D search on an IncrementalPointCloud");
}

void IncrementalPointCloud::nn_multiple_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df& query, [[maybe_unused]] const size_t N,
    [[maybe_unused]] std::vector<mrpt::math::TPoint2Df>& results,
    [[maybe_unused]] std::vector<float>&                 out_dists_sqr,
    [[maybe_unused]] std::vector<uint64_t>&              resultIndicesOrIDs) const
{
  THROW_EXCEPTION("Cannot run a 2D search on an IncrementalPointCloud");
}

void IncrementalPointCloud::nn_radius_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df&        query,
    [[maybe_unused]] const float                         search_radius_sqr,
    [[maybe_unused]] std::vector<mrpt::math::TPoint2Df>& results,
    [[maybe_unused]] std::vector<float>&                 out_dists_sqr,
    [[maybe_unused]] std::vector<uint64_t>&              resultIndicesOrIDs,
    [[maybe_unused]] size_t                              maxPoints) const
{
  THROW_EXCEPTION("Cannot run a 2D search on an IncrementalPointCloud");
}

// =====================================
// Per-point covariances (cov2cov)
// =====================================

void IncrementalPointCloud::computeCovariance(uint32_t slot) const
{
  if (cov_valid_[slot] != 0) return;

  cov_[slot]       = mrpt::math::CMatrixFloat33::Identity();
  cov_valid_[slot] = 1;

  const std::size_t liveCount = index_->size();
  if (liveCount < 3) return;

  const std::size_t K = std::min<std::size_t>(creationOptions.k_correspondences_for_cov, liveCount);
  const std::size_t minK = std::min<std::size_t>(creationOptions.min_correspondences_for_cov, K);

  const auto maxDistSqr = static_cast<float>(
      creationOptions.max_distance_for_cov * creationOptions.max_distance_for_cov);

  const float q[3] = {m_x[slot], m_y[slot], m_z[slot]};

  std::vector<uint32_t> idxs(K);
  std::vector<float>    dists(K);

  const std::size_t found =
      index_->knnSearchWithinRadius(q, K, maxDistSqr, idxs.data(), dists.data());

  // Too few neighbors actually found: a plane fit from this few samples is
  // unreliable, so keep the isotropic covariance set above instead of an
  // over-confident, possibly degenerate regularized one.
  if (found < minK) return;

  Eigen::Matrix<double, 3, -1> neighbors(3, static_cast<Eigen::Index>(found));
  for (std::size_t j = 0; j < found; j++)
  {
    const uint32_t nIdx                        = idxs[j];
    neighbors(0, static_cast<Eigen::Index>(j)) = static_cast<double>(m_x[nIdx]);
    neighbors(1, static_cast<Eigen::Index>(j)) = static_cast<double>(m_y[nIdx]);
    neighbors(2, static_cast<Eigen::Index>(j)) = static_cast<double>(m_z[nIdx]);
  }
  neighbors.colwise() -= Eigen::Vector3d(m_x[slot], m_y[slot], m_z[slot]);

  const Eigen::Matrix3d cov = neighbors * neighbors.transpose() / static_cast<double>(found);

  // Plane regularization of the singular values (see DLIO'2023, or Segal's
  // GICP paper): SVD sorts them in decreasing order, so the last one is the
  // plane normal direction.
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  const Eigen::Vector3d values(1.0, 1.0, 1e-3);
  cov_[slot] = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
}

void IncrementalPointCloud::ensureCovariancesFor(const std::vector<uint32_t>& slots) const
{
#if defined(MOLA_METRIC_MAPS_USE_TBB)
  tbb::parallel_for(
      static_cast<std::size_t>(0), slots.size(),
      [&](std::size_t i) { computeCovariance(slots[i]); });
#else
  for (const uint32_t slot : slots) computeCovariance(slot);
#endif
}

std::size_t IncrementalPointCloud::point_count() const { return livePointCount(); }

void IncrementalPointCloud::nn_search_cov2cov(
    const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
    const float max_search_distance, mp2p_icp::MatchedPointWithCovList& outPairings) const
{
  nn_search_cov2cov_impl(
      localMap, localMapPose, MatchingDistanceProfile(max_search_distance), outPairings);
}

#if defined(MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE)
void IncrementalPointCloud::nn_search_cov2cov(
    const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
    const mp2p_icp::MatchingDistanceProfile& matchingDistance,
    mp2p_icp::MatchedPointWithCovList&       outPairings) const
{
  nn_search_cov2cov_impl(localMap, localMapPose, matchingDistance, outPairings);
}
#endif

void IncrementalPointCloud::nn_search_cov2cov_impl(
    const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
    const MatchingDistanceProfile&     matchingDistance,
    mp2p_icp::MatchedPointWithCovList& outPairings) const
{
  const auto* localPc = dynamic_cast<const IncrementalPointCloud*>(&localMap);
  ASSERTMSG_(
      localPc,
      "Implementation expects the local map to be also of type mola::IncrementalPointCloud");

  // std::scoped_lock takes both without risking a lock-order deadlock, but it
  // must not be given the same mutex twice:
  std::unique_lock<std::recursive_mutex> lckGlobal(mtx_, std::defer_lock);
  std::unique_lock<std::recursive_mutex> lckLocal(localPc->mtx_, std::defer_lock);
  if (localPc == this)
  {
    lckGlobal.lock();
  }
  else
  {
    std::lock(lckGlobal, lckLocal);
  }

  ensureIndexUpToDate();
  localPc->ensureIndexUpToDate();

  // The local points, in the local map own frame:
  std::vector<uint32_t> localLive;
  localPc->index_->snapshotLiveIndices(localLive);
  if (localLive.empty()) return;

  // Local covariances are needed for every local point, and each slot appears
  // exactly once in the snapshot:
  localPc->ensureCovariancesFor(localLive);

  // Fast path: a flat threshold (the common case, and the only one before this
  // profile existed) needs no per-point range computation.
  const bool  matchDistIsFlat  = matchingDistance.isFlat();
  const float matchDistFlatSqr = mrpt::square(matchingDistance.near);

  const bool needsQueryRange = matchingDistance.needsRange();

  const auto& l_xs = localPc->m_x;
  const auto& l_ys = localPc->m_y;
  const auto& l_zs = localPc->m_z;

  // --- Pass 1: nearest-neighbor search (no covariance access) --------------
  struct Match
  {
    uint32_t local_slot  = 0;
    uint32_t global_slot = 0;
  };
  std::vector<Match> matches;
  matches.reserve(localLive.size());

  const auto lambdaMatchOne = [&](uint32_t ls, std::vector<Match>& out)
  {
    const auto gq = localMapPose.composePoint(mrpt::math::TPoint3D(l_xs[ls], l_ys[ls], l_zs[ls]));

    const float q[3] = {
        static_cast<float>(gq.x), static_cast<float>(gq.y), static_cast<float>(gq.z)};

    // Range from the sensor, i.e. in the query's own (untransformed) local frame -
    // matches how the range-adaptive threshold was measured and validated (see
    // ~/plans/icp-bench-range-adaptive-matching.md).
    float range = 0;
    if (needsQueryRange)
    {
      range = std::sqrt(mrpt::square(l_xs[ls]) + mrpt::square(l_ys[ls]) + mrpt::square(l_zs[ls]));
    }

    const float max_sqr_dist =
        matchDistIsFlat ? matchDistFlatSqr : mrpt::square(matchingDistance(range));

    uint32_t gIdx = 0;
    float    d    = 0;
    if (index_->knnSearchWithinRadius(q, 1, max_sqr_dist, &gIdx, &d) == 0) return;

    out.push_back({ls, gIdx});
  };

#if defined(MOLA_METRIC_MAPS_USE_TBB)
  tbb::enumerable_thread_specific<std::vector<Match>> tls;

  tbb::parallel_for(
      static_cast<std::size_t>(0), localLive.size(),
      [&](std::size_t i) { lambdaMatchOne(localLive[i], tls.local()); });

  for (auto& perThread : tls)
  {
    matches.insert(matches.end(), perThread.begin(), perThread.end());
  }
#else
  for (const uint32_t ls : localLive) lambdaMatchOne(ls, matches);
#endif

  if (matches.empty()) return;

  // Give the pairing list a canonical order. Pass 3 below emits `outPairings`
  // in this order, and that order reaches the solver's summation order and
  // therefore the optimized pose, so it must not depend on which worker threads
  // happened to take part. Each local point yields at most one match, so its
  // slot is a unique key and the order is total.
  //
  // Unlike the equivalent in KeyframePointCloudMap, this sorts both the
  // parallel and the sequential path, because neither was canonical here:
  // `snapshotLiveIndices()` walks the k-d tree depth-first, so `localLive` is
  // in tree-topology order rather than in slot order. That also makes the two
  // paths agree by construction, and makes the result independent of the tree
  // shape a rebuild happened to leave behind.
  std::sort(
      matches.begin(), matches.end(),
      [](const Match& a, const Match& b) { return a.local_slot < b.local_slot; });

  // --- Pass 2: covariances of the matched global points -------------------
  // Deduplicated, so no two threads write to the same cache entry.
  std::vector<uint32_t> globalSlots;
  globalSlots.reserve(matches.size());
  for (const auto& m : matches) globalSlots.push_back(m.global_slot);
  std::sort(globalSlots.begin(), globalSlots.end());
  globalSlots.erase(std::unique(globalSlots.begin(), globalSlots.end()), globalSlots.end());

  ensureCovariancesFor(globalSlots);

  // --- Pass 3: assemble the pairings --------------------------------------
  const Eigen::Matrix3f R = localMapPose.getRotationMatrix().cast_float().asEigen();

  outPairings.reserve(outPairings.size() + matches.size());
  for (const auto& m : matches)
  {
    auto& p = outPairings.emplace_back();

    p.local_idx  = m.local_slot;
    p.global_idx = m.global_slot;
    p.local      = {l_xs[m.local_slot], l_ys[m.local_slot], l_zs[m.local_slot]};
    p.global     = {m_x[m.global_slot], m_y[m.global_slot], m_z[m.global_slot]};

    // GICP \cite segal2009gicp : (COV_global + R*COV_local*R^T)^-1
    const Eigen::Matrix3f covLocalGlobalFrame =
        R * localPc->cov_[m.local_slot].asEigen() * R.transpose();

    const Eigen::Matrix3f w = cov_[m.global_slot].asEigen() + covLocalGlobalFrame;

    p.cov_inv.asEigen() = w.inverse();
  }
}

// =====================================
// Visualization / export
// =====================================

void IncrementalPointCloud::getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const
{
  MRPT_START
  if (!genericMapParams.enableSaveAs3DObject) return;

  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  if (index_->size() == m_x.size())
  {
    // No tombstoned points: the base class can render our own storage directly.
    CGenericPointsMap::getVisualizationInto(outObj);
    return;
  }

  const auto compacted        = liveCompactedCopy();
  compacted->renderOptions    = renderOptions;
  compacted->genericMapParams = genericMapParams;
  compacted->getVisualizationInto(outObj);

  MRPT_END
}

void IncrementalPointCloud::saveMetricMapRepresentationToFile(
    const std::string& filNamePrefix) const
{
  using namespace std::string_literals;
  liveCompactedCopy()->save3D_to_text_file(filNamePrefix + ".txt"s);
}

const mrpt::maps::CSimplePointsMap* IncrementalPointCloud::getAsSimplePointsMap() const
{
  auto lck = mrpt::lockHelper(mtx_);
  ensureIndexUpToDate();

  if (!cachedPoints_) cachedPoints_ = mrpt::maps::CSimplePointsMap::Create();

  cachedPoints_->clear();

  std::vector<uint32_t> live;
  index_->snapshotLiveIndices(live);
  std::sort(live.begin(), live.end());

  cachedPoints_->reserve(live.size());
  for (const uint32_t slot : live)
  {
    cachedPoints_->insertPointFast(m_x[slot], m_y[slot], m_z[slot]);
  }
  cachedPoints_->mark_as_modified();

  return cachedPoints_.get();
}

// =====================================
// Serialization
// =====================================

uint8_t IncrementalPointCloud::serializeGetVersion() const { return 1; }

void IncrementalPointCloud::serializeTo(mrpt::serialization::CArchive& out) const
{
  auto lck = mrpt::lockHelper(mtx_);

  // Only the live points, compacted, delegating the per-point field encoding
  // to the base class:
  const auto compacted = liveCompactedCopy();
  out << *compacted;

  creationOptions.writeToStream(out);
  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
  renderOptions.writeToStream(out);

  // v1: optionally cache the k-d tree index built over the very same
  // (compacted) point order just written above, so it does not have to be
  // rebuilt on load. Self-describing (a flag byte precedes any blob), so
  // readers stay stream-aligned regardless of the write-time option or the
  // nanoflann version this build was compiled against.
  uint8_t     hasKdTree = 0;
  std::string kdBlob;
#if defined(MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE)
  if (creationOptions.serialize_kdtree)
  {
    // A throwaway index bound to `compacted`'s own buffers: `index_` (the
    // live one) was built over the possibly-uncompacted storage, whose slot
    // numbers do not match the renumbered order just serialized.
    internal::IncrementalKDTree::Params p;
    p.async_rebuild = false;  // baking is synchronous, one-shot offline work
    p.alpha_balance = creationOptions.alpha_balance;
    p.alpha_deleted = creationOptions.alpha_deleted;

    const auto bakeIndex = internal::IncrementalKDTree::Create(p);
    bakeIndex->setPointBuffers(
        compacted->getPointsBufferRef_x().data(), compacted->getPointsBufferRef_y().data(),
        compacted->getPointsBufferRef_z().data(), compacted->size());
    if (compacted->size() != 0)
    {
      bakeIndex->addPoints(0, static_cast<uint32_t>(compacted->size() - 1));
    }

    std::ostringstream ss(std::ios::binary);
    bakeIndex->saveIndex(ss);
    kdBlob    = ss.str();
    hasKdTree = 1;
  }
#endif
  out.WriteAs<uint8_t>(hasKdTree);
  if (hasKdTree != 0)
  {
    out << kdBlob;
  }
}

void IncrementalPointCloud::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  auto lck = mrpt::lockHelper(mtx_);

  bool loadedBakedIndex = false;

  switch (version)
  {
    case 0:
    case 1:
    {
      auto tmp = mrpt::maps::CGenericPointsMap::Create();
      in >> *tmp;

      // Copies points and all per-point fields (clearing us first):
      CPointsMap::operator=(*tmp);

      creationOptions.readFromStream(in);
      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
      renderOptions.readFromStream(in);

      // v1: optional cached k-d tree index (see serializeTo). Always consume
      // the flag byte + blob so the stream stays aligned even when this
      // build cannot install the index.
      if (version >= 1)
      {
        const auto hasKdTree = in.ReadAs<uint8_t>();
        if (hasKdTree != 0)
        {
          std::string kdBlob;
          in >> kdBlob;
#if defined(MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE)
          // Points were just installed above (in the very same, compacted
          // order the blob was baked over): build an empty index over them
          // and install the baked topology instead of bulk-rebuilding it.
          try
          {
            createEmptyIndex();
            std::istringstream ss(kdBlob, std::ios::binary);
            index_->loadIndex(ss);
            indexed_up_to_   = m_x.size();
            loadedBakedIndex = true;
          }
          catch (const std::exception&)
          {
            // A corrupt or incompatible blob must not lose the map: the points
            // are already installed, so fall back to a normal bulk rebuild
            // below (loadedBakedIndex stays false).
          }
#endif
        }
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  if (!loadedBakedIndex)
  {
    resetIndex();
  }
}

// =====================================
// Options
// =====================================

std::map<std::string, mrpt::config::CLoadableOptions*> IncrementalPointCloud::optionsByName()
{
  return {
      {"creationOptions", &creationOptions},
      {"insertionOptions", &insertionOptions},
      {"likelihoodOptions", &likelihoodOptions},
      {"renderOptions", &renderOptions},
  };
}

bool IncrementalPointCloud::trySetCreationOptions(
    const mrpt::config::CConfigFileBase& cfg, const std::string& section)
{
  auto lck = mrpt::lockHelper(mtx_);

  TCreationOptions newOpts = creationOptions;
  newOpts.loadFromConfigFile(cfg, section);

  const bool treeParamsChanged = newOpts.async_rebuild != creationOptions.async_rebuild ||
                                 newOpts.alpha_balance != creationOptions.alpha_balance ||
                                 newOpts.alpha_deleted != creationOptions.alpha_deleted ||
                                 newOpts.reserve_points != creationOptions.reserve_points;

  const bool covParamsChanged =
      newOpts.k_correspondences_for_cov != creationOptions.k_correspondences_for_cov ||
      newOpts.min_correspondences_for_cov != creationOptions.min_correspondences_for_cov ||
      newOpts.max_distance_for_cov != creationOptions.max_distance_for_cov;

  creationOptions = newOpts;

  if (treeParamsChanged)
  {
    // The tree is rebuilt with the new parameters; no map content is lost.
    compact();
  }
  else if (covParamsChanged)
  {
    std::fill(cov_valid_.begin(), cov_valid_.end(), 0);
  }

  return true;
}

void IncrementalPointCloud::TCreationOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(remove_points_farther_than, double, c, s);
  MRPT_LOAD_CONFIG_VAR(async_rebuild, bool, c, s);
  MRPT_LOAD_CONFIG_VAR(alpha_balance, float, c, s);
  MRPT_LOAD_CONFIG_VAR(alpha_deleted, float, c, s);
  MRPT_LOAD_CONFIG_VAR(reserve_points, uint64_t, c, s);
  MRPT_LOAD_CONFIG_VAR(k_correspondences_for_cov, uint64_t, c, s);
  MRPT_LOAD_CONFIG_VAR(min_correspondences_for_cov, uint64_t, c, s);
  MRPT_LOAD_CONFIG_VAR(max_distance_for_cov, double, c, s);
  MRPT_LOAD_CONFIG_VAR(serialize_kdtree, bool, c, s);
}

void IncrementalPointCloud::TCreationOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [IncrementalPointCloud::TCreationOptions] ------- \n\n";

  LOADABLEOPTS_DUMP_VAR(remove_points_farther_than, double);
  LOADABLEOPTS_DUMP_VAR(async_rebuild, bool);
  LOADABLEOPTS_DUMP_VAR(alpha_balance, float);
  LOADABLEOPTS_DUMP_VAR(alpha_deleted, float);
  LOADABLEOPTS_DUMP_VAR(reserve_points, int);
  LOADABLEOPTS_DUMP_VAR(k_correspondences_for_cov, int);
  LOADABLEOPTS_DUMP_VAR(min_correspondences_for_cov, int);
  LOADABLEOPTS_DUMP_VAR(max_distance_for_cov, double);
  LOADABLEOPTS_DUMP_VAR(serialize_kdtree, bool);
}

void IncrementalPointCloud::TCreationOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  const int8_t version = 1;
  out << version;
  out << remove_points_farther_than << async_rebuild << alpha_balance << alpha_deleted
      << reserve_points << k_correspondences_for_cov << min_correspondences_for_cov
      << max_distance_for_cov;
  out << serialize_kdtree;  // v1
}

void IncrementalPointCloud::TCreationOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version;
  in >> version;
  switch (version)
  {
    case 0:
    case 1:
    {
      in >> remove_points_farther_than >> async_rebuild >> alpha_balance >> alpha_deleted >>
          reserve_points >> k_correspondences_for_cov >> min_correspondences_for_cov >>
          max_distance_for_cov;
      if (version >= 1)
      {
        in >> serialize_kdtree;
      }
      else
      {
        serialize_kdtree = false;
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}
