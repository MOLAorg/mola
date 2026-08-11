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
 * @file   KeyframePointCloudMap.cpp
 * @brief  Key-frames, each keeping point cloud layers with their own KD-tree
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */

#include <mola_metric_maps/KeyframePointCloudMap.h>
#if __has_include(<mp2p_icp/pointcloud_field_utils.h>)
#include <mp2p_icp/pointcloud_field_utils.h>
#define MOLA_MM_HAS_ROTATE_VIEW_HEADER 1
#endif
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR
#include <mrpt/core/get_env.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/math/TOrientedBox.h>
#include <mrpt/math/matrix_serialization.h>  // CArchive << CMatrixFloat33 (cov baking)
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/serialization/CArchive.h>  // serialization
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/string_utils.h>  // unitsFormat()
#include <mrpt/version.h>  // For MRPT_VERSION

#include <algorithm>  // std::lower_bound
#include <cmath>
#include <cstdint>
#include <iterator>  // std::distance
#include <numeric>  // std::accumulate
#include <sstream>
#include <unordered_set>

#if defined(MOLA_METRIC_MAPS_USE_TBB)
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#endif

#include <atomic>
#include <type_traits>

// Radius-limited kNN (RKNN), which MRPT exposes as the optional
// maximum-search-distance argument of kdTreeNClosestPoint3DIdx(), needs
// nanoflann >= 1.5.1; older versions throw at runtime from that overload.
// NANOFLANN_VERSION comes in transitively from the MRPT points-map headers
// above, i.e. it describes exactly the copy MRPT's own templates were built
// against, which is the one that decides here. (Do NOT include nanoflann
// directly: see the note at the top of IncrementalKDTree.cpp.)
//
// The version varies across ROS distributions, so this cannot be assumed:
// Ubuntu jammy still ships 1.4.2.
#if defined(NANOFLANN_VERSION) && NANOFLANN_VERSION >= 0x151
#define MOLA_MM_HAS_RKNN_SEARCH 1
#endif

#if defined(MOLA_MM_HAS_ROTATE_VIEW_HEADER)
// Feature detection for mp2p_icp::rotateViewDirectionFields(): the function
// was added to an already-existing header (pointcloud_field_utils.h), so
// __has_include() alone cannot tell an old mp2p_icp_map checkout (header
// present, function absent) apart from a new one.
//
// A plain decltype/void_t probe on the *qualified* name doesn't work: looking
// up a name that simply does not exist in a fixed, non-dependent namespace is
// a hard compile error, not a SFINAE-friendly substitution failure (confirmed
// against mp2p_icp 2.9.0, which lacks the function). A using-directive-based
// "ADL barrier" doesn't work either: a using-directive only injects the used
// namespace's members at the *nearest common ancestor* of the directive's
// own namespace and the used one (here, the global namespace, since
// mp2p_icp and any namespace we declare are unrelated siblings) -- so a
// fallback declared in our own namespace is found first and always wins,
// regardless of whether the real overload exists.
//
// What does work: reopening mp2p_icp itself to add a low-priority ellipsis
// fallback as a genuine member of that same namespace. Now both candidates
// (real declaration, if any, and our fallback) live in the exact same
// namespace, so ordinary overload resolution picks the real one whenever
// it exists (exact match beats the ellipsis conversion), and falls back
// to ours otherwise -- with no lookup error in either case.
namespace mp2p_icp
{
struct rotate_view_fields_unavailable_tag
{
};
[[maybe_unused]] rotate_view_fields_unavailable_tag rotateViewDirectionFields(...);
}  // namespace mp2p_icp

namespace
{
template <typename Pts, typename Pose>
constexpr bool mp2p_icp_has_rotate_view_fields()
{
  return !std::is_same_v<
      decltype(mp2p_icp::rotateViewDirectionFields(
          std::declval<Pts&>(), std::declval<const Pose&>())),
      mp2p_icp::rotate_view_fields_unavailable_tag>;
}
}  // namespace
#endif

namespace
{
// MRPT_TODO: this is the pre-fix, open-coded rotation loop, kept only as a
// fallback for mp2p_icp_map checkouts older than the one introducing
// mp2p_icp::rotateViewDirectionFields() (see MOLAorg/mp2p_icp#70). Delete
// this function (and its call sites below) once the minimum required
// mp2p_icp_map version provides the helper.
[[maybe_unused]] void rotateViewDirectionFieldsLegacy(
    mrpt::maps::CPointsMap& pts, const mrpt::poses::CPose3D& tf)
{
  auto* vx = pts.getPointsBufferRef_float_field("view_x");
  auto* vy = pts.getPointsBufferRef_float_field("view_y");
  auto* vz = pts.getPointsBufferRef_float_field("view_z");

  if (vx == nullptr || vy == nullptr || vz == nullptr)
  {
    // One or more view fields are missing in the destination map even
    // though the source had all three.
    // TODO: log a warning here once a logger is available.
    return;
  }

  const size_t n = pts.size();

  // TODO: Write an AVX2 version of this rotation loop.
  for (size_t i = 0; i < n; ++i)
  {
    const auto vg = tf.rotateVector({(*vx)[i], (*vy)[i], (*vz)[i]}).cast<float>();
    (*vx)[i]      = vg.x;
    (*vy)[i]      = vg.y;
    (*vz)[i]      = vg.z;
  }
}

#if defined(MOLA_MM_HAS_ROTATE_VIEW_HEADER)
template <typename Pts, typename Pose>
void rotateViewDirectionFieldsOrFallback(Pts& pts, const Pose& tf)
{
  // This dispatch must live in a template so that if constexpr genuinely
  // discards (without type-checking) the untaken branch: outside of a
  // template, both branches of "if constexpr" are still fully compiled,
  // which would try to actually call the ellipsis fallback declared above
  // when the real helper is absent -- an ill-formed call, since it'd
  // require copying a non-copyable mrpt::maps::CPointsMap.
  if constexpr (mp2p_icp_has_rotate_view_fields<Pts, Pose>())
  {
    mp2p_icp::rotateViewDirectionFields(pts, tf);
  }
  else
  {
    rotateViewDirectionFieldsLegacy(pts, tf);
  }
}
#endif
}  // namespace

const thread_local auto ENV_DO_PROFILE_COV =
    mrpt::get_env<bool>("MOLA_KEYFRAME_MAP_PROFILE_COV", false);

const thread_local auto ENV_DEBUG_ACTIVE_KFS =
    mrpt::get_env<bool>("MOLA_KEYFRAME_MAP_DEBUG_ACTIVE_KFS", false);

// #define DO_VIZ_DEBUG 1

#if DO_VIZ_DEBUG
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/Scene.h>

#include <fstream>
#endif

static_assert(
    std::is_copy_constructible_v<mola::KeyframePointCloudMap>,
    "KeyframePointCloudMap must be copy constructible");

using namespace mola;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
    "mola::KeyframePointCloudMap,KeyframePointCloudMap", mola::KeyframePointCloudMap)

KeyframePointCloudMap::TMapDefinition::TMapDefinition() = default;
void KeyframePointCloudMap::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& s, const std::string& sectionPrefix)
{
  using namespace std::string_literals;

  if (s.sectionExists(sectionPrefix + "_creationOpts"s))
  {
    creationOptions.loadFromConfigFile(s, sectionPrefix + "_creationOpts"s);
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

void KeyframePointCloudMap::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  creationOptions.dumpToTextStream(out);
  insertionOpts.dumpToTextStream(out);
  likelihoodOpts.dumpToTextStream(out);
  renderOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr KeyframePointCloudMap::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const auto* def = dynamic_cast<const KeyframePointCloudMap::TMapDefinition*>(&_def);
  ASSERT_(def);
  auto obj = KeyframePointCloudMap::Create();

  obj->creationOptions   = def->creationOptions;
  obj->insertionOptions  = def->insertionOpts;
  obj->likelihoodOptions = def->likelihoodOpts;
  obj->renderOptions     = def->renderOpts;

  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(KeyframePointCloudMap, CMetricMap, mola)

// =====================================
// Serialization
// =====================================

uint8_t KeyframePointCloudMap::serializeGetVersion() const { return 3; }
void    KeyframePointCloudMap::serializeTo(mrpt::serialization::CArchive& out) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  // params:
  // out << params_;
  creationOptions.writeToStream(out);
  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
  renderOptions.writeToStream(out);

  // data:
  out.WriteAs<uint32_t>(keyframes_.size());
  for (const auto& [kf_id, kf] : keyframes_)
  {
    out << kf_id;
    out << kf.timestamp;
    out << kf.pose();
    if (kf.pointcloud())
    {
      out.WriteAs<uint8_t>(1);  // has point cloud
      out << *kf.pointcloud();

      // v2: optionally cache the per-cloud 3D KD-tree index so it does not have
      // to be rebuilt on load. Self-describing (a flag byte precedes any blob),
      // so readers can skip it regardless of the write-time option or MRPT build.
      uint8_t     hasKdTree = 0;
      std::string kdBlob;
#if defined(MRPT_HAS_KDTREE_SAVE_LOAD_INDEX)
      if (creationOptions.serialize_kdtrees)
      {
        std::ostringstream ss(std::ios::binary);
        if (kf.pointcloud()->kdtree_save_index_3D(ss))
        {
          kdBlob    = ss.str();
          hasKdTree = 1;
        }
      }
#endif
      out.WriteAs<uint8_t>(hasKdTree);
      if (hasKdTree != 0)
      {
        out << kdBlob;
      }

      // v3: optionally cache the per-point local-frame covariances so they do not
      // have to be recomputed (K-NN + SVD per point) on load. Self-describing: a
      // flag byte precedes any data, so readers stay stream-aligned regardless of
      // the write-time option. See TCreationOptions::serialize_covariances.
      uint8_t hasCov = 0;
      if (creationOptions.serialize_covariances)
      {
        hasCov = 1;
      }
      out.WriteAs<uint8_t>(hasCov);
      if (hasCov != 0)
      {
        // Ensures cached_cov_local_ is populated (computes it if needed).
        const auto& covs = kf.covariancesLocal();
        out.WriteAs<uint32_t>(covs.size());
        for (const auto& c : covs)
        {
          out << c;
        }
      }
    }
    else
    {
      out.WriteAs<uint8_t>(0);  // no point cloud
    }
  }

  // v1: add active set, to debug ICP states
  out << cached_.icp_search_kfs;
}

void KeyframePointCloudMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  // clear contents (including cache_)
  this->clear();

  auto lck = mrpt::lockHelper(*state_mtx_);

  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    {
      // params:
      creationOptions.readFromStream(in);
      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
      renderOptions.readFromStream(in);

      // data:
      const auto n_kfs = in.ReadAs<uint32_t>();
      for (uint32_t i = 0; i < n_kfs; i++)
      {
        uint64_t kf_id = 0;
        in >> kf_id;

        auto [it, isNew] = keyframes_.try_emplace(
            kf_id, creationOptions.k_correspondences_for_cov,
            creationOptions.min_correspondences_for_cov, creationOptions.max_distance_for_cov);
        KeyFrame& kf = it->second;

        in >> kf.timestamp;
        mrpt::poses::CPose3D pose;
        in >> pose;
        kf.pose(pose);
        const auto has_pointcloud = in.ReadAs<uint8_t>();
        if (has_pointcloud > 0)
        {
          auto obj = in.ReadObject();
          auto pc  = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obj);
          ASSERT_(pc);
          kf.pointcloud(pc);

          // v2: optional cached KD-tree index (see serializeTo). Always consume
          // the flag byte + blob so the stream stays aligned even when this
          // build cannot install the index.
          if (version >= 2)
          {
            const auto hasKdTree = in.ReadAs<uint8_t>();
            if (hasKdTree != 0)
            {
              std::string kdBlob;
              in >> kdBlob;
#if defined(MRPT_HAS_KDTREE_SAVE_LOAD_INDEX)
              std::istringstream ss(kdBlob, std::ios::binary);
              // Install the precomputed index so the first query does not rebuild
              // it. Points were just deserialized above (which marks the tree
              // outdated), so this must come after kf.pointcloud(pc).
              pc->kdtree_load_index_3D(ss);
#endif
            }
          }

          // v3: optional cached per-point local-frame covariances (see
          // serializeTo). Always consume the flag byte + data so the stream stays
          // aligned. Install them (after kf.pointcloud(pc), which cleared caches)
          // so computeCovariancesAndDensity() becomes a no-op.
          if (version >= 3)
          {
            const auto hasCov = in.ReadAs<uint8_t>();
            if (hasCov != 0)
            {
              const auto                              nCov = in.ReadAs<uint32_t>();
              std::vector<mrpt::math::CMatrixFloat33> covs(nCov);
              for (uint32_t c = 0; c < nCov; c++)
              {
                in >> covs[c];
              }
              kf.installCovariancesLocal(std::move(covs));
            }
          }
        }
      }

      if (version >= 1)
      {
        // Only kept in the stream for post-mortem debugging of ICP states;
        // it must not be restored into the live cache. icp_search_submap (the
        // actual prepared search structure it would otherwise validate) is
        // never serialized, so leaving this set here would let
        // icp_get_prepared_as_global()'s "already up to date" fast path
        // wrongly skip rebuilding it whenever the freshly-computed active-KF
        // selection happens to match this stale, reloaded set.
        std::optional<std::set<KeyFrameID>> debugOnlyPriorActiveKfs;
        in >> debugOnlyPriorActiveKfs;
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  // Restore the monotonic id counter so future insertions don't collide
  // with already-loaded ids.
  next_free_kf_id_ = keyframes_.empty() ? 0 : (keyframes_.rbegin()->first + 1);

  if (mrpt::get_env<bool>("MOLA_KEYFRAME_MAP_DEBUG_DUMP_KFS_ON_LOAD", false))
  {
    for (const auto& [kf_id, kf] : keyframes_)
    {
      const auto& p    = kf.pose();
      const auto  bbox = kf.localBoundingBox();
      const auto  diag = (bbox.max - bbox.min).norm();
      printf(
          "[KeyframePointCloudMap] loaded KF id=%llu pose=[x=%.3f y=%.3f z=%.3f yaw=%.2f "
          "pitch=%.2f roll=%.2f] points=%zu local_bbox_diag=%.2f local_bbox=[%.2f,%.2f,%.2f]-["
          "%.2f,%.2f,%.2f]\n",
          static_cast<unsigned long long>(kf_id), p.x(), p.y(), p.z(), mrpt::RAD2DEG(p.yaw()),
          mrpt::RAD2DEG(p.pitch()), mrpt::RAD2DEG(p.roll()),
          kf.pointcloud() ? kf.pointcloud()->size() : 0, diag, bbox.min.x, bbox.min.y, bbox.min.z,
          bbox.max.x, bbox.max.y, bbox.max.z);
    }
  }
}

///  === KeyframePointCloudMap ===

KeyframePointCloudMap::~KeyframePointCloudMap() = default;

mrpt::math::TBoundingBoxf KeyframePointCloudMap::boundingBox() const
{
  if (cached_.boundingBox)
  {
    return *cached_.boundingBox;
  }

  cached_.boundingBox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
  for (const auto& [kf_id, kf] : keyframes_)
  {
#if MRPT_VERSION >= 0x020e0d
    // Use TOrientedBox to correctly transform all 8 corners of the local AABB
    // before computing the global axis-aligned envelope.
    // TBoundingBox::compose() only transforms min/max, which is inaccurate under rotation.
    const auto                  lbbox        = kf.localBoundingBox();
    const mrpt::math::TPoint3Df center_local = {
        (lbbox.min.x + lbbox.max.x) * 0.5f, (lbbox.min.y + lbbox.max.y) * 0.5f,
        (lbbox.min.z + lbbox.max.z) * 0.5f};
    const mrpt::math::TPoint3Df size = {
        lbbox.max.x - lbbox.min.x, lbbox.max.y - lbbox.min.y, lbbox.max.z - lbbox.min.z};
    const auto          global_center = kf.pose().composePoint(mrpt::math::TPoint3D(center_local));
    mrpt::math::TPose3D obb_pose      = kf.pose().asTPose();
    obb_pose.x                        = global_center.x;
    obb_pose.y                        = global_center.y;
    obb_pose.z                        = global_center.z;
    cached_.boundingBox               = cached_.boundingBox->unionWith(
                      mrpt::math::TOrientedBoxf(obb_pose, size).getAxisAlignedBox());
#else
    cached_.boundingBox = cached_.boundingBox->unionWith(kf.localBoundingBox().compose(kf.pose()));
#endif
  }

  return *cached_.boundingBox;
}

bool KeyframePointCloudMap::nn_single_search(
    const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  ASSERT_(cached_.icp_search_submap);
  return cached_.icp_search_submap->pointcloud()->nn_single_search(
      query, result, out_dist_sqr, resultIndexOrID);
}

bool KeyframePointCloudMap::nn_single_search(
    const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  ASSERT_(cached_.icp_search_submap);
  return cached_.icp_search_submap->pointcloud()->nn_single_search(
      query, result, out_dist_sqr, resultIndexOrID);
}

void KeyframePointCloudMap::nn_multiple_search(
    const mrpt::math::TPoint3Df& query, const size_t N, std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_multiple_search(
      query, N, results, out_dists_sqr, resultIndicesOrIDs);
}

void KeyframePointCloudMap::nn_multiple_search(
    const mrpt::math::TPoint2Df& query, const size_t N, std::vector<mrpt::math::TPoint2Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_multiple_search(
      query, N, results, out_dists_sqr, resultIndicesOrIDs);
}

void KeyframePointCloudMap::nn_radius_search(
    const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_radius_search(
      query, search_radius_sqr, results, out_dists_sqr, resultIndicesOrIDs, maxPoints);
}

void KeyframePointCloudMap::nn_radius_search(
    const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_radius_search(
      query, search_radius_sqr, results, out_dists_sqr, resultIndicesOrIDs, maxPoints);
}

void KeyframePointCloudMap::icp_get_prepared_as_global(  // NOLINT
    const mrpt::poses::CPose3D&                                      icp_ref_point,
    [[maybe_unused]] const std::optional<mrpt::math::TBoundingBoxf>& local_map_roi) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  std::set<KeyFrameID> kfs_to_search_limited;

  // ---------------------------------------------------------------
  // 1) Score every keyframe with an additive proximity metric
  // ---------------------------------------------------------------
  struct KFCandidate
  {
    KeyFrameID kfId;
    double     dist;  // Euclidean distance
    double     angle;  // SO(3) log-norm (rad)
    double     metric;  // combined score (lower is better)
  };
  std::vector<KFCandidate> candidates;
  candidates.reserve(keyframes_.size());

  const double rotW = creationOptions.rotation_distance_weight;

  for (const auto& [kf_id, kf] : keyframes_)
  {
    if (!kf.pointcloud())
    {
      continue;
    }

    const auto query_local = icp_ref_point - kf.pose();

    const double dist_to_kf  = query_local.norm();
    const double angle_to_kf = mrpt::poses::Lie::SO<3>::log(query_local.getRotationMatrix()).norm();

    // Density penalty: a keyframe with few points (e.g. an under-merged, leftover
    // cluster from regroupKeyframes()) can sit geometrically closer to the query
    // than a much better-populated keyframe, yet contribute almost no usable
    // geometry for ICP correspondences. Without this term, pure pose-distance
    // ranking can pick that sparse keyframe over one that would actually match,
    // starving the ICP submap of real points. Penalty ramps linearly from 0 (at
    // or above density_penalty_min_points) to density_penalty_max_m (at 0 points).
    double densityPenalty = 0.0;
    if (creationOptions.density_penalty_min_points > 0)
    {
      const auto minPts = static_cast<double>(creationOptions.density_penalty_min_points);
      const auto nPts   = static_cast<double>(kf.pointcloud()->size());
      if (nPts < minPts)
      {
        densityPenalty = (1.0 - nPts / minPts) * creationOptions.density_penalty_max_m;
      }
    }

    // Additive metric: prevents the zero-distance degeneracy of multiplicative forms,
    // and gives a clean meters-equivalent score that is easy to reason about.
    const double m = dist_to_kf + rotW * angle_to_kf + densityPenalty;

    candidates.push_back({kf_id, dist_to_kf, angle_to_kf, m});
  }

  // Sort ascending by metric (best first):
  std::sort(
      candidates.begin(), candidates.end(),
      [](const KFCandidate& a, const KFCandidate& b) { return a.metric < b.metric; });

  // ---------------------------------------------------------------
  // 2) Fill primary slots (proximity-ranked)
  // ---------------------------------------------------------------
  const uint32_t totalSlots = creationOptions.max_search_keyframes;
  const uint32_t diverseSlots =
      std::min(creationOptions.num_diverse_keyframes, totalSlots > 1 ? totalSlots - 1 : 0u);
  const uint32_t primarySlots = totalSlots - diverseSlots;

  // Track which KFs are already selected and their orientations:
  std::set<KeyFrameID> selectedIds;
  std::vector<double>  selectedAngles;  // angle_to_kf for diversity calc

  for (const auto& c : candidates)
  {
    if (selectedIds.size() >= primarySlots)
    {
      break;
    }
    selectedIds.insert(c.kfId);
    selectedAngles.push_back(c.angle);
  }

  // ---------------------------------------------------------------
  // 3) Fill diverse slots: pick remaining candidates that maximise
  //    the minimum angular difference to any already-selected frame,
  //    while keeping a reasonable distance (within 3× the best
  //    candidate's distance, or the closest unselected).
  // ---------------------------------------------------------------
  if (diverseSlots > 0 && candidates.size() > primarySlots)
  {
    // Distance threshold for the diverse pool: at most 3× the
    // farthest primary KF distance, but never smaller than the
    // closest unselected candidate.
    double maxPrimaryDist = 0.0;
    for (const auto& c : candidates)
    {
      if (selectedIds.count(c.kfId) != 0)
      {
        maxPrimaryDist = std::max(maxPrimaryDist, c.dist);
      }
    }
    const double diverseDistLimit = std::max(maxPrimaryDist * 3.0, 1.0);

    for (uint32_t d = 0; d < diverseSlots; ++d)
    {
      double             bestDiversityScore = -1.0;
      const KFCandidate* bestCandidate      = nullptr;

      // Two passes: first restricted to diverseDistLimit (the common case, when
      // enough nearby candidates exist); if that finds nothing (e.g. only a
      // handful of widely-spaced super-keyframes cover the map, as produced by
      // regroupKeyframes()), fall back to the best-diversity candidate regardless
      // of distance rather than leaving this slot -- and thus the submap -- short
      // a keyframe. An unfilled diverse slot means ICP runs against a single
      // keyframe only, which can permanently starve it of correspondences at the
      // edge of that keyframe's own coverage.
      for (const bool enforceDistLimit : {true, false})
      {
        if (bestCandidate != nullptr)
        {
          break;
        }

        for (const auto& c : candidates)
        {
          if (selectedIds.count(c.kfId) != 0)
          {
            continue;
          }
          if (enforceDistLimit && c.dist > diverseDistLimit)
          {
            continue;
          }

          // Diversity score: minimum angular difference to any
          // already-selected frame's angle_to_kf.  We actually
          // want the frame whose *orientation* (kf.pose()) differs
          // most from the selected set, so compute pairwise SO(3)
          // differences would be ideal but expensive; as a cheaper
          // proxy, use the absolute angle_to_kf difference, which
          // works well because frames at similar positions but
          // different orientations will have very different
          // angle_to_kf values.
          double minAngDiff = std::numeric_limits<double>::max();
          for (const double selAngle : selectedAngles)
          {
            minAngDiff = std::min(minAngDiff, std::abs(c.angle - selAngle));
          }

          if (minAngDiff > bestDiversityScore)
          {
            bestDiversityScore = minAngDiff;
            bestCandidate      = &c;
          }
        }
      }

      if (bestCandidate != nullptr)
      {
        selectedIds.insert(bestCandidate->kfId);
        selectedAngles.push_back(bestCandidate->angle);
      }
    }
  }

  kfs_to_search_limited = selectedIds;

  if (ENV_DEBUG_ACTIVE_KFS)
  {
    std::string s;
    for (const auto kf_id : kfs_to_search_limited)
    {
      s += std::to_string(kf_id) + " ";
    }
    printf(
        "[KeyframePointCloudMap] ICP active KFs (%zu): %s | query_pose=[x=%.3f y=%.3f z=%.3f "
        "yaw=%.2f]\n",
        kfs_to_search_limited.size(), s.c_str(), icp_ref_point.x(), icp_ref_point.y(),
        icp_ref_point.z(), mrpt::RAD2DEG(icp_ref_point.yaw()));
    for (const auto& c : candidates)
    {
      printf(
          "[KeyframePointCloudMap]   candidate KF id=%llu dist=%.2f angle_deg=%.2f metric=%.2f "
          "%s\n",
          static_cast<unsigned long long>(c.kfId), c.dist, mrpt::RAD2DEG(c.angle), c.metric,
          kfs_to_search_limited.count(c.kfId) ? "[SELECTED]" : "");
    }
  }

  // ---------------------------------------------------------------
  // 4) Rebuild merged submap if the selection (or the exact/approximate mode) changed
  // ---------------------------------------------------------------
  if (cached_.icp_search_kfs && *cached_.icp_search_kfs == kfs_to_search_limited &&
      cached_.icp_search_built_approximate == creationOptions.approximate_cov)
  {
    return;  // Already up to date.
  }

  cached_.icp_search_kfs               = kfs_to_search_limited;
  cached_.icp_search_built_approximate = creationOptions.approximate_cov;

  // NOTE: Do NOT unlock 'lck' here. The mutex must be held for the entire submap
  // rebuild below, because cached_.icp_search_submap and keyframes_ are both
  // shared mutable state that can be read concurrently by nn_* methods and
  // icp_get_prepared_as_global() itself.

  if (creationOptions.approximate_cov)
  {
    // Approximate mode: skip the merged-cloud submap entirely. nn_search_cov2cov()
    // will instead query each active KF's own cached, *local*-frame KD-tree
    // directly. A KD-tree's structure is invariant under the rigid keyframe pose,
    // so rather than materializing a per-KF global-frame cloud and rebuilding a
    // KD-tree on it (which the baked, on-disk index cannot accelerate, since the
    // baked index lives on the local cloud), the matcher transforms the query
    // point into each KF's local frame and queries the local (baked) index.
    //
    // Here we only warm each active KF's per-KF cache so the first ICP align()
    // does not pay that cost on the caller's thread:
    //   - buildCache(): local bbox, local KD-tree (installed from disk when
    //     serialize_kdtrees was baked; otherwise built once), and the per-point
    //     local covariances (installed from disk when serialize_covariances was
    //     baked; otherwise computed once).
    //   - covariancesGlobal(): the cheap per-pose rotation of those covariances.
    // No global-frame cloud or KD-tree is built at all. See
    // TCreationOptions::approximate_cov / serialize_covariances.
    cached_.icp_search_submap.reset();

    for (const auto kf_id : kfs_to_search_limited)
    {
      const auto& kf = keyframes_.at(kf_id);
      if (!kf.pointcloud())
      {
        continue;
      }
      kf.buildCache();  // local bbox, KD-tree, per-point local covariances
      kf.covariancesGlobal();  // rotate covariances to the current global pose
    }
    return;
  }

  cached_.icp_search_submap.reset();
  cached_.icp_search_submap.emplace(
      creationOptions.k_correspondences_for_cov, creationOptions.min_correspondences_for_cov,
      creationOptions.max_distance_for_cov);

  for (const auto kf_id : kfs_to_search_limited)
  {
    const auto& kf = keyframes_.at(kf_id);

    if (!kf.pointcloud())
    {
      continue;
    }

    // pointcloud_global() carries view_{x,y,z} in global frame when the
    // source cloud has those fields (see updatePointsGlobal).
    const auto& kf_global = kf.pointcloud_global();

    if (!cached_.icp_search_submap->pointcloud())
    {
      // Use CGenericPointsMap for the merged submap when the source carries
      // view-direction fields, so they are preserved for nn_search_cov2cov.
      const bool src_has_view = (kf_global->getPointsBufferRef_float_field("view_x") != nullptr) &&
                                (kf_global->getPointsBufferRef_float_field("view_y") != nullptr) &&
                                (kf_global->getPointsBufferRef_float_field("view_z") != nullptr);

      if (src_has_view)
      {
        auto gpc = mrpt::maps::CGenericPointsMap::Create();
        // The fact that the map is CGenericPointsMap will make insertAnotherMap() below to
        // copy the "view_{x,y,z}" fields.
        // It will also copy all other fields, unless we set the (mrpt>=3.0.0) param
        // `autoRegisterAllSourceFields` to false.
        gpc->registerField_float("view_x");
        gpc->registerField_float("view_y");
        gpc->registerField_float("view_z");
        cached_.icp_search_submap->pointcloud(gpc);
      }
      else
      {
        cached_.icp_search_submap->pointcloud(mrpt::maps::CSimplePointsMap::Create());
      }
    }

#if MRPT_VERSION >= 0x020f0b  // 2.15.11
    cached_.icp_search_submap->pointcloud()->insertAnotherMap(
        kf_global.get(), mrpt::poses::CPose3D::Identity(), false /*filterOutPointsAtZero*/,
        false /*autoRegisterAllSourceFields*/);
#else
    cached_.icp_search_submap->pointcloud()->insertAnotherMap(
        kf_global.get(), mrpt::poses::CPose3D::Identity());
#endif
  }

  // If no keyframe contributed any points (the selection was empty, or every
  // selected keyframe had a null/empty cloud), fall back to a valid empty
  // cloud. Otherwise buildCache() and the global-frame warm-up below would
  // dereference a null pointcloud. An empty cloud yields zero ICP
  // correspondences (the map is simply rejected as a match), which keeps a
  // single degenerate submap from aborting a whole background loop-closure scan.
  if (!cached_.icp_search_submap->pointcloud())
  {
    cached_.icp_search_submap->pointcloud(mrpt::maps::CSimplePointsMap::Create());
  }

  cached_.icp_search_submap->buildCache();

  // Pre-warm the *global-frame* structures used by NearestPointWithCovCapable
  // matchers (nn_search_cov2cov).  buildCache() above only builds:
  //   - bbox, KD-tree on pointcloud_  (local frame)
  //   - per-point covariances in local frame
  //
  // But the matcher accesses, on the submap, all of these instead:
  //   - pointcloud_global()      -> deep copy of pointcloud_ rotated to global
  //   - kdTreeEnsureIndexBuilt3D -> KD-tree on the *global* cloud
  //   - covariancesGlobal()      -> rotated covariances
  //
  // Without warming these here, the first ICP align() pays the cost on the
  // lidar worker thread, which can stall the very first scan for several
  // seconds on large maps - even when icp_get_prepared_as_global() has
  // already been called.
  const auto& globalPoints = cached_.icp_search_submap->pointcloud_global();
  globalPoints->kdTreeEnsureIndexBuilt3D();
  cached_.icp_search_submap->covariancesGlobal();
}

void KeyframePointCloudMap::icp_cleanup() const
{
  // Do NOT free the map, we might reuse it for next ICP call.
}

// =============== MetricMapMergeCapable ===============
void KeyframePointCloudMap::merge_with(
    const MetricMapMergeCapable&               source,
    const std::optional<mrpt::poses::CPose3D>& otherRelativePose)
{
  const auto* sourceMapKF = dynamic_cast<const KeyframePointCloudMap*>(&source);
  ASSERTMSG_(
      sourceMapKF, "Implementation expects source map to be also of type KeyframePointCloudMap");

  ASSERT_(sourceMapKF != this);

  auto lck = mrpt::lockHelper(*state_mtx_);

  for (const auto& [srcKfId, srcKf] : sourceMapKF->keyframes_)
  {
    const auto& srcPc = srcKf.pointcloud();
    if (!srcPc)
    {
      continue;
    }
    auto [it, isNew] = keyframes_.try_emplace(
        next_free_kf_id_++, creationOptions.k_correspondences_for_cov,
        creationOptions.min_correspondences_for_cov, creationOptions.max_distance_for_cov);
    auto& new_kf = it->second;

    // copy
    new_kf = srcKf;
    // and optionally, transform
    if (otherRelativePose)
    {
      new_kf.pose(*otherRelativePose + new_kf.pose());
    }
    last_inserted_kf_id_ = it->first;
  }
}

void KeyframePointCloudMap::transform_map_left_multiply(const mrpt::poses::CPose3D& b)
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  transform_map_left_multiply_impl(b);
}

void KeyframePointCloudMap::transform_map_left_multiply_impl(const mrpt::poses::CPose3D& b)
{
  for (auto& [id, kf] : keyframes_)
  {
    kf.pose(b + kf.pose());
  }

  cached_.reset();
}

void KeyframePointCloudMap::nn_search_cov2cov(
    const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
    const float max_search_distance, mp2p_icp::MatchedPointWithCovList& outPairings) const
{
  nn_search_cov2cov_impl(
      localMap, localMapPose, MatchingDistanceProfile(max_search_distance), outPairings);
}

#if defined(MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE)
void KeyframePointCloudMap::nn_search_cov2cov(
    const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
    const mp2p_icp::MatchingDistanceProfile& matchingDistance,
    mp2p_icp::MatchedPointWithCovList&       outPairings) const
{
  nn_search_cov2cov_impl(localMap, localMapPose, matchingDistance, outPairings);
}
#endif

void KeyframePointCloudMap::nn_search_cov2cov_impl(
    const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
    const MatchingDistanceProfile&     matchingDistance,
    mp2p_icp::MatchedPointWithCovList& outPairings) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  // Enforce local map to recompute its covariances to the new pose:
  const auto* localMapKF = dynamic_cast<const KeyframePointCloudMap*>(&localMap);
  ASSERTMSG_(
      localMapKF, "Implementation expects local map to be also of type KeyframePointCloudMap");

  ASSERT_EQUAL_(localMapKF->keyframes_.size(), 1U);
  auto&      localKf             = const_cast<KeyFrame&>(localMapKF->keyframes_.at(0));
  const auto originalLocalKfPose = localKf.pose();
  localKf.pose(localMapPose);

  if (creationOptions.approximate_cov)
  {
    ASSERTMSG_(
        cached_.icp_search_kfs,
        "Using this method requires calling icp_get_prepared_as_global() first");
    nn_search_cov2cov_approximate(localKf, *cached_.icp_search_kfs, matchingDistance, outPairings);
    localKf.pose(originalLocalKfPose);
    return;
  }

  ASSERTMSG_(
      cached_.icp_search_submap,
      "Using this method requires calling icp_get_prepared_as_global() first");

  const auto& localKfCov        = localKf.covariancesGlobal();
  const auto& localPointsTransf = localKf.pointcloud_global();
  const auto& localPoints       = localKf.pointcloud();

  const auto& globalKfCov  = cached_.icp_search_submap->covariancesGlobal();
  const auto& globalPoints = cached_.icp_search_submap->pointcloud_global();

  const auto localPointCount = localPointsTransf->size();

  // Fast path: a flat threshold (the common case, and the only one before this
  // profile existed) needs no per-point range computation.
  const bool  matchDistIsFlat  = matchingDistance.isFlat();
  const float matchDistFlatSqr = mrpt::square(matchingDistance.near);

  const bool needsQueryRange = matchingDistance.needsRange();

  const auto& xs_tf = localPointsTransf->getPointsBufferRef_x();
  const auto& ys_tf = localPointsTransf->getPointsBufferRef_y();
  const auto& zs_tf = localPointsTransf->getPointsBufferRef_z();

  const auto& xs = localPoints->getPointsBufferRef_x();
  const auto& ys = localPoints->getPointsBufferRef_y();
  const auto& zs = localPoints->getPointsBufferRef_z();

  const auto& g_xs = globalPoints->getPointsBufferRef_x();
  const auto& g_ys = globalPoints->getPointsBufferRef_y();
  const auto& g_zs = globalPoints->getPointsBufferRef_z();

  globalPoints->kdTreeEnsureIndexBuilt3D();

  // ------------------------------------------------------------------
  // View-direction filter setup
  // ------------------------------------------------------------------
  // "view_x/y/z" are unit vectors in the *local KF frame* of each cloud,
  // pointing FROM the point TOWARD the sensor at acquisition time.
  //
  // The merged submap (globalPoints) and localPointsTransf are plain
  // CSimplePointsMap objects that carry only XYZ - custom fields such as
  // view_x/y/z are not propagated by insertAnotherMap().  We therefore
  // read view vectors directly from the original local-frame clouds and
  // rotate them to the global frame on the fly.
  //
  // For the global reference submap we need the view vectors from each
  // contributing KF.  Because the submap was built by sequentially
  // appending KF point clouds (see icp_get_prepared_as_global), the
  // global point index `nn_global_idx` maps directly into that merged
  // buffer - but we cannot recover which KF it came from without extra
  // bookkeeping.  As a pragmatic solution we instead use the view
  // vectors stored in the *reference keyframe's global pointcloud*.
  //
  // The reference submap is cached_.icp_search_submap, which is itself
  // a KeyFrame object.  Its `pointcloud()` holds the merged local cloud
  // (without view fields) while the individual contributing KFs hold the
  // original clouds.  Since the submap KF is a synthetic merge, the only
  // robust approach that does not require index-bookkeeping is to obtain
  // the global-frame view vectors from the *original* KFs stored in
  // keyframes_.
  //
  // For simplicity - and because this filter is a best-effort heuristic -
  // we only activate the filter when the *local* query cloud carries view
  // fields AND the global reference submap's merged pointcloud also does.
  // The merged submap won't have them unless explicitly populated, so the
  // effective activation path today is:
  //
  //   local KF cloud has view_x/y/z   AND
  //   globalPoints (the icp_search_submap pointcloud_global()) has view_x/y/z
  //
  // If only one side has them the filter is silently skipped.
  const bool try_view_filter = creationOptions.use_view_direction_filter;

  // Local cloud view buffers - in the *local KF frame* of the query map.
  // We need to rotate these by localMapPose.R to get global-frame vectors.
  const mrpt::aligned_std_vector<float>* local_view_x = nullptr;
  const mrpt::aligned_std_vector<float>* local_view_y = nullptr;
  const mrpt::aligned_std_vector<float>* local_view_z = nullptr;

  // Global (reference submap) view buffers - already in global frame if the
  // submap pointcloud carries them (currently only if the upstream pipeline
  // populated them via e.g. insertAnotherMap on a CGenericPointsMap).
  const mrpt::aligned_std_vector<float>* global_view_x = nullptr;
  const mrpt::aligned_std_vector<float>* global_view_y = nullptr;
  const mrpt::aligned_std_vector<float>* global_view_z = nullptr;

  if (try_view_filter)
  {
    local_view_x = localPoints->getPointsBufferRef_float_field("view_x");
    local_view_y = localPoints->getPointsBufferRef_float_field("view_y");
    local_view_z = localPoints->getPointsBufferRef_float_field("view_z");

    global_view_x = globalPoints->getPointsBufferRef_float_field("view_x");
    global_view_y = globalPoints->getPointsBufferRef_float_field("view_y");
    global_view_z = globalPoints->getPointsBufferRef_float_field("view_z");
  }

  // Both clouds must expose all three channels for the filter to activate.
  const bool have_view_fields = (local_view_x != nullptr) && (local_view_y != nullptr) &&
                                (local_view_z != nullptr) && (global_view_x != nullptr) &&
                                (global_view_y != nullptr) && (global_view_z != nullptr);

  const double max_view_angle_deg = std::clamp(creationOptions.max_view_angle_deg, 0.0, 180.0);
  const bool   do_view_filter = try_view_filter && have_view_fields && max_view_angle_deg < 180.0;

  // Pre-compute the cosine threshold once (cos is monotonically decreasing
  // on [0°, 180°], so angle > threshold  <=>  dot < cos(threshold)).
  const float view_cos_threshold =
      do_view_filter ? static_cast<float>(std::cos(mrpt::DEG2RAD(max_view_angle_deg)))
                     : -2.0f;  // sentinel: never reached when filter is disabled

#if defined(MOLA_METRIC_MAPS_USE_TBB)
  // Pairings are appended, so only the ones added below get reordered:
  const size_t firstNewPairing = outPairings.size();

  tbb::enumerable_thread_specific<mp2p_icp::MatchedPointWithCovList> tls;

  tbb::parallel_for(
      static_cast<size_t>(0), localPointCount,
      [&](size_t local_idx)
#else
  for (size_t local_idx = 0; local_idx < localPointCount; local_idx++)
#endif
      {
        // Range from the sensor, i.e. in the query's own (untransformed) local frame -
        // matches how the range-adaptive threshold was measured and validated (see
        // ~/plans/icp-bench-range-adaptive-matching.md).
        float range = 0;
        if (needsQueryRange)
        {
          range = std::sqrt(
              mrpt::square(xs[local_idx]) + mrpt::square(ys[local_idx]) +
              mrpt::square(zs[local_idx]));
        }

        const float max_sqr_dist =
            matchDistIsFlat ? matchDistFlatSqr : mrpt::square(matchingDistance(range));

        float nn_dist_sqr = std::numeric_limits<float>::max();
        const size_t nn_global_idx = globalPoints->kdTreeClosestPoint3D(
            xs_tf[local_idx], ys_tf[local_idx], zs_tf[local_idx], nn_dist_sqr);

        if (nn_dist_sqr > max_sqr_dist)
        {
#if defined(MOLA_METRIC_MAPS_USE_TBB)
          return;  // exit TBB lambda for this index
#else
      continue;  // skip to next iteration of the for loop
#endif
        }

        // ----------------------------------------------------------
        // View-direction angle filter
        // ----------------------------------------------------------
        if (do_view_filter)
        {
          // Rotate the local-frame view vector to the global frame.
          const auto v_local_global =
              localMapPose
                  .rotateVector(
                      {(*local_view_x)[local_idx], (*local_view_y)[local_idx],
                       (*local_view_z)[local_idx]})
                  .cast<float>();

          // dot product with the reference cloud's (already global-frame) view vector
          const float dot = v_local_global.x * (*global_view_x)[nn_global_idx] +
                            v_local_global.y * (*global_view_y)[nn_global_idx] +
                            v_local_global.z * (*global_view_z)[nn_global_idx];

          // dot < cos(max_angle)  =>  angle > max_angle  =>  reject
          if (dot < view_cos_threshold)
          {
#if defined(MOLA_METRIC_MAPS_USE_TBB)
            return;  // exit TBB lambda for this index
#else
        continue;  // skip to next iteration of the for loop
#endif
          }
        }

    // Add pairing:
#if defined(MOLA_METRIC_MAPS_USE_TBB)
        auto& p = tls.local().emplace_back();
#else
    auto& p = outPairings.emplace_back();
#endif

        p.global_idx = nn_global_idx;
        p.local_idx  = local_idx;
        p.local      = {xs[local_idx], ys[local_idx], zs[local_idx]};
        p.global     = {g_xs[nn_global_idx], g_ys[nn_global_idx], g_zs[nn_global_idx]};

        /* Following GICP \cite segal2009gicp this should be:
         *  `(COV_{global} + R*COV_{local}*R^T)^{-1}`
         *  But localKfCov already incorporate R*C*R^T from localKf.pose(p)
         */
        p.cov_inv = (globalKfCov.at(nn_global_idx) + localKfCov.at(local_idx)).inverse();
      }
#if defined(MOLA_METRIC_MAPS_USE_TBB)
  );
  // Merge from all threads:
  for (auto& localVec : tls)
  {
    outPairings.insert(
        outPairings.end(), std::make_move_iterator(localVec.begin()),
        std::make_move_iterator(localVec.end()));
  }

  // Thread-local vectors are visited in an unspecified order, so restore the
  // order the sequential path produces. Each local point yields at most one
  // pairing, so its index is a unique key. The pairing list order reaches the
  // solver's summation order, and from there the optimized pose.
  std::sort(
      outPairings.begin() + firstNewPairing, outPairings.end(),
      [](const mp2p_icp::point_with_cov_pair_t& a, const mp2p_icp::point_with_cov_pair_t& b)
      { return a.local_idx < b.local_idx; });
#endif

  // Recover original:
  localKf.pose(originalLocalKfPose);
}

namespace
{
/// Best-effort, packed identifier for a (active-KF ordinal, local point index) pair, used as
/// point_with_cov_pair_t::global_idx in approximate cov2cov mode (see
/// KeyframePointCloudMap::nn_search_cov2cov_approximate()). Unlike the exact (merged-cloud)
/// mode, there is no single flat index space to draw from, so 8 bits are reserved for the
/// active-KF ordinal (creationOptions.max_search_keyframes is never more than a few dozen) and
/// 24 bits for the local index (post-decimation KF clouds are always far below 16M points).
/// Collisions beyond those bounds only affect Matcher_Cov2Cov's optional
/// "allowMatchAlreadyMatchedGlobalPoints" bookkeeping, not correctness of the pairing itself.
uint32_t packApproxGlobalIdx(uint32_t kf_ordinal, uint32_t local_idx)
{
  return ((kf_ordinal & 0xFFu) << 24) | (local_idx & 0x00FFFFFFu);
}
}  // namespace

void KeyframePointCloudMap::nn_search_cov2cov_approximate(
    const KeyFrame& localKf, const std::set<KeyFrameID>& activeKfs,
    const MatchingDistanceProfile&     matchingDistance,
    mp2p_icp::MatchedPointWithCovList& outPairings) const
{
  const auto& localKfCov        = localKf.covariancesGlobal();
  const auto& localPointsTransf = localKf.pointcloud_global();
  const auto& localPoints       = localKf.pointcloud();

  const auto localPointCount = localPointsTransf->size();

  // See the exact-mode nn_search_cov2cov() above for the rationale.
  const bool  matchDistIsFlat  = matchingDistance.isFlat();
  const float matchDistFlatSqr = mrpt::square(matchingDistance.near);
  const bool  needsQueryRange  = matchingDistance.needsRange();

  const auto& xs_tf = localPointsTransf->getPointsBufferRef_x();
  const auto& ys_tf = localPointsTransf->getPointsBufferRef_y();
  const auto& zs_tf = localPointsTransf->getPointsBufferRef_z();

  const auto& xs = localPoints->getPointsBufferRef_x();
  const auto& ys = localPoints->getPointsBufferRef_y();
  const auto& zs = localPoints->getPointsBufferRef_z();

  // View-direction filter setup: see the exact-mode nn_search_cov2cov() above for the full
  // rationale. Here the same per-pair test is applied against whichever active KF ends up
  // being the closest one for a given query point.
  const bool try_view_filter = creationOptions.use_view_direction_filter;

  const mrpt::aligned_std_vector<float>* local_view_x = nullptr;
  const mrpt::aligned_std_vector<float>* local_view_y = nullptr;
  const mrpt::aligned_std_vector<float>* local_view_z = nullptr;
  if (try_view_filter)
  {
    local_view_x = localPoints->getPointsBufferRef_float_field("view_x");
    local_view_y = localPoints->getPointsBufferRef_float_field("view_y");
    local_view_z = localPoints->getPointsBufferRef_float_field("view_z");
  }
  const bool have_local_view_fields =
      (local_view_x != nullptr) && (local_view_y != nullptr) && (local_view_z != nullptr);

  const double max_view_angle_deg = std::clamp(creationOptions.max_view_angle_deg, 0.0, 180.0);
  const bool   do_view_filter =
      try_view_filter && have_local_view_fields && max_view_angle_deg < 180.0;
  const float view_cos_threshold =
      do_view_filter ? static_cast<float>(std::cos(mrpt::DEG2RAD(max_view_angle_deg))) : -2.0f;

  // Per-active-KF lookup tables, built once (not per query point).
  //
  // Each active KF is queried through its OWN local-frame cloud and KD-tree
  // (`kf.pointcloud()`), which is the one baked on disk by mm-kf-bake-kdtrees. The
  // query point (in the global frame) is transformed into the KF's local frame via
  // `poseInv` before the KD-tree lookup, and the matched local point is composed
  // back to the global frame via `pose` for the output pairing. This avoids ever
  // materializing a per-KF global-frame cloud or rebuilding a KD-tree on it.
  struct ActiveKfEntry
  {
    const mrpt::maps::CPointsMap*                  localPoints = nullptr;  // baked KD-tree
    mrpt::poses::CPose3D                           pose;  // KF pose (local->global)
    mrpt::poses::CPose3D                           poseInv;  // its inverse (global->local)
    const std::vector<mrpt::math::CMatrixFloat33>* globalCov = nullptr;  // per-pose rotated
    // Local-frame coordinate buffers, parallel to the KD-tree points:
    const mrpt::aligned_std_vector<float>* xs = nullptr;
    const mrpt::aligned_std_vector<float>* ys = nullptr;
    const mrpt::aligned_std_vector<float>* zs = nullptr;
    // Local-frame view fields (rotated to global on the fly when filtering):
    const mrpt::aligned_std_vector<float>* view_x = nullptr;
    const mrpt::aligned_std_vector<float>* view_y = nullptr;
    const mrpt::aligned_std_vector<float>* view_z = nullptr;
  };
  const bool debugMatchStats = mrpt::get_env<bool>("MOLA_KEYFRAME_MAP_DEBUG_MATCH_STATS", false);
  std::atomic<size_t> statsNoCandidateInRange{0};
  std::atomic<size_t> statsRejectedByViewFilter{0};
  std::atomic<size_t> statsAccepted{0};

  std::vector<ActiveKfEntry> entries;
  entries.reserve(activeKfs.size());
  for (const auto kf_id : activeKfs)
  {
    // activeKfs was snapshotted by an earlier icp_get_prepared_as_global() call, under its
    // own lock acquisition. A concurrent insertObservation() may have evicted this id in the
    // meantime (see insertionOptions.remove_frames_farther_than), so look it up defensively
    // instead of keyframes_.at(), which would throw.
    const auto it = keyframes_.find(kf_id);
    if (it == keyframes_.end())
    {
      continue;
    }
    const auto& kf = it->second;
    const auto& lp = kf.pointcloud();
    if (!lp || lp->empty())
    {
      continue;
    }
    lp->kdTreeEnsureIndexBuilt3D();  // baked on disk when serialize_kdtrees was used
    ActiveKfEntry e;
    e.localPoints = lp.get();
    e.pose        = kf.pose();
    e.poseInv     = mrpt::poses::CPose3D::Identity() - kf.pose();  // == pose^{-1}
    e.globalCov   = &kf.covariancesGlobal();
    e.xs          = &lp->getPointsBufferRef_x();
    e.ys          = &lp->getPointsBufferRef_y();
    e.zs          = &lp->getPointsBufferRef_z();
    if (do_view_filter)
    {
      e.view_x = lp->getPointsBufferRef_float_field("view_x");
      e.view_y = lp->getPointsBufferRef_float_field("view_y");
      e.view_z = lp->getPointsBufferRef_float_field("view_z");
    }
    entries.push_back(e);
  }

#if defined(MOLA_METRIC_MAPS_USE_TBB)
  // Pairings are appended, so only the ones added below get reordered:
  const size_t firstNewPairing = outPairings.size();

  tbb::enumerable_thread_specific<mp2p_icp::MatchedPointWithCovList> tls;

  tbb::parallel_for(
      static_cast<size_t>(0), localPointCount,
      [&](size_t local_idx)
#else
  for (size_t local_idx = 0; local_idx < localPointCount; local_idx++)
#endif
      {
        float range = 0;
        if (needsQueryRange)
        {
          range = std::sqrt(
              mrpt::square(xs[local_idx]) + mrpt::square(ys[local_idx]) +
              mrpt::square(zs[local_idx]));
        }

        const float max_sqr_dist =
            matchDistIsFlat ? matchDistFlatSqr : mrpt::square(matchingDistance(range));

        // "N" KD-tree queries (one per active KF) instead of one on a merged cloud:
        float  best_dist_sqr = std::numeric_limits<float>::max();
        size_t best_entry    = 0;
        size_t best_idx      = 0;
        bool   found         = false;

        // The winner may live in a different keyframe than the previous best, so
        // candidates are folded across all of them.
        const auto lambdaFoldCandidate = [&](size_t e, size_t idx, float d)
        {
          if (d < best_dist_sqr)
          {
            best_dist_sqr = d;
            best_entry    = e;
            best_idx      = idx;
            found         = true;
          }
        };

        for (size_t e = 0; e < entries.size(); e++)
        {
          // Transform the (global-frame) query point into this KF's local frame,
          // then query its baked local KD-tree. Rigid transforms preserve
          // distances, so best_dist_sqr remains comparable across keyframes.
          const auto ql = entries[e].poseInv.composePoint(
              mrpt::math::TPoint3D(xs_tf[local_idx], ys_tf[local_idx], zs_tf[local_idx]));

          float      d   = std::numeric_limits<float>::max();
          const auto idx = entries[e].localPoints->kdTreeClosestPoint3D(
              static_cast<float>(ql.x), static_cast<float>(ql.y), static_cast<float>(ql.z), d);
          lambdaFoldCandidate(e, idx, d);
        }

        if (!found || best_dist_sqr > max_sqr_dist)
        {
          if (debugMatchStats)
          {
            statsNoCandidateInRange++;
          }
#if defined(MOLA_METRIC_MAPS_USE_TBB)
          return;  // exit TBB lambda for this index
#else
      continue;  // skip to next iteration of the for loop
#endif
        }

        const auto& entry = entries[best_entry];

        if (do_view_filter && entry.view_x != nullptr && entry.view_y != nullptr &&
            entry.view_z != nullptr)
        {
          const auto v_local_global =
              localKf.pose()
                  .rotateVector(
                      {(*local_view_x)[local_idx], (*local_view_y)[local_idx],
                       (*local_view_z)[local_idx]})
                  .cast<float>();

          // The reference view fields are in the KF's *local* frame; rotate them
          // to the global frame by the KF pose before comparing (mirrors what the
          // pre-baked global cloud used to store).
          const auto ref_view_global = entry.pose.rotateVector(mrpt::math::TVector3D(
              (*entry.view_x)[best_idx], (*entry.view_y)[best_idx], (*entry.view_z)[best_idx]));

          const float dot = v_local_global.x * static_cast<float>(ref_view_global.x) +
                            v_local_global.y * static_cast<float>(ref_view_global.y) +
                            v_local_global.z * static_cast<float>(ref_view_global.z);

          if (dot < view_cos_threshold)
          {
            if (debugMatchStats)
            {
              statsRejectedByViewFilter++;
            }
#if defined(MOLA_METRIC_MAPS_USE_TBB)
            return;  // exit TBB lambda for this index
#else
        continue;  // skip to next iteration of the for loop
#endif
          }
        }

        if (debugMatchStats)
        {
          statsAccepted++;
        }

    // Add pairing:
#if defined(MOLA_METRIC_MAPS_USE_TBB)
        auto& p = tls.local().emplace_back();
#else
    auto& p = outPairings.emplace_back();
#endif

        // Compose the matched local-frame reference point back to the global frame.
        const auto g_pt = entry.pose.composePoint(mrpt::math::TPoint3D(
            (*entry.xs)[best_idx], (*entry.ys)[best_idx], (*entry.zs)[best_idx]));

        p.global_idx =
            packApproxGlobalIdx(static_cast<uint32_t>(best_entry), static_cast<uint32_t>(best_idx));
        p.local_idx = static_cast<uint32_t>(local_idx);
        p.local     = {xs[local_idx], ys[local_idx], zs[local_idx]};
        p.global    = {
               static_cast<float>(g_pt.x), static_cast<float>(g_pt.y), static_cast<float>(g_pt.z)};

        /* Following GICP \cite segal2009gicp this should be:
         *  `(COV_{global} + R*COV_{local}*R^T)^{-1}`
         *  But localKfCov already incorporate R*C*R^T from localKf.pose(p)
         */
        p.cov_inv = (entry.globalCov->at(best_idx) + localKfCov.at(local_idx)).inverse();
      }
#if defined(MOLA_METRIC_MAPS_USE_TBB)
  );
  // Merge from all threads:
  for (auto& localVec : tls)
  {
    outPairings.insert(
        outPairings.end(), std::make_move_iterator(localVec.begin()),
        std::make_move_iterator(localVec.end()));
  }

  // Thread-local vectors are visited in an unspecified order, so restore the
  // order the sequential path produces. Each local point yields at most one
  // pairing, so its index is a unique key. The pairing list order reaches the
  // solver's summation order, and from there the optimized pose.
  std::sort(
      outPairings.begin() + firstNewPairing, outPairings.end(),
      [](const mp2p_icp::point_with_cov_pair_t& a, const mp2p_icp::point_with_cov_pair_t& b)
      { return a.local_idx < b.local_idx; });
#endif

  if (debugMatchStats)
  {
    printf(
        "[KeyframePointCloudMap] nn_search_cov2cov_approximate: query_points=%zu "
        "active_kfs=%zu accepted=%zu no_candidate_in_range=%zu rejected_by_view_filter=%zu "
        "matching_distance_near=%.3f matching_distance_far=%.3f\n",
        localPointCount, entries.size(), statsAccepted.load(), statsNoCandidateInRange.load(),
        statsRejectedByViewFilter.load(), static_cast<double>(matchingDistance.near),
        static_cast<double>(matchingDistance.far));
  }
}

std::size_t KeyframePointCloudMap::point_count() const
{
  std::size_t total = 0;
  for (const auto& [id, kf] : keyframes_)
  {
    if (kf.pointcloud())
    {
      total += kf.pointcloud()->size();
    }
  }
  return total;
}

std::string KeyframePointCloudMap::asString() const
{
  // Returns a short description of the map:
  std::ostringstream o;
  std::size_t        total_points = 0;
  for (const auto& [kf_id, kf] : keyframes_)
  {
    total_points += kf.pointcloud() ? kf.pointcloud()->size() : 0;
  }

  o << "KeyframePointCloudMap: " << keyframes_.size() << " keyframes, "
    << mrpt::system::unitsFormat(static_cast<double>(total_points)) << " points.";
  return o.str();
}

namespace
{
/// Maps HSV (h,s,v all in [0,1]) to an 8-bit RGB color (switch-free formulation).
mrpt::img::TColor hsvToColor(double h, double s, double v)
{
  const auto chan = [&](double n)
  {
    const double k = std::fmod(n + h * 6.0, 6.0);
    return v - v * s * std::max(0.0, std::min({k, 4.0 - k, 1.0}));
  };
  const auto u8 = [](double x) { return static_cast<uint8_t>(std::clamp(x, 0.0, 1.0) * 255.0); };
  return mrpt::img::TColor(u8(chan(5.0)), u8(chan(3.0)), u8(chan(1.0)));
}

/// Deterministic, well-separated color for the i-th key-frame (golden-ratio hue
/// spacing keeps consecutive key-frames visually distinct).
mrpt::img::TColor distinctKfColor(size_t i)
{
  const double hue = std::fmod(static_cast<double>(i) * 0.618033988749895, 1.0);
  return hsvToColor(hue, 0.75, 0.98);
}
}  // namespace

void KeyframePointCloudMap::getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const
{
  MRPT_START
  if (!genericMapParams.enableSaveAs3DObject)
  {
    return;
  }

  const thread_local auto ENV_KEYFRAMES_SHOW_ACTIVE_FRAMES =
      mrpt::get_env<bool>("MOLA_KEYFRAME_MAP_VIZ_SHOW_ACTIVE_SUBMAP", false);
  const thread_local auto ENV_KEYFRAMES_AXES_LENGTH =
      mrpt::get_env<float>("MOLA_KEYFRAME_MAP_VIZ_OVERRIDE_AXES_LENGTH", .0f);

  const thread_local auto ENV_KEYFRAMES_SHOW_COV =
      mrpt::get_env<bool>("MOLA_KEYFRAME_MAP_VIZ_SHOW_COV", false);

  // Debug aid: paint each key-frame a distinct color so regrouped
  // super-keyframes / clusters are easy to tell apart visually.
  const thread_local auto ENV_KEYFRAMES_COLOR_BY_KF =
      mrpt::get_env<bool>("MOLA_KEYFRAME_MAP_VIZ_COLOR_BY_KF", false);

  auto lck = mrpt::lockHelper(*state_mtx_);

  // Create one visualization object per KF:
  for (const auto& [kf_id, kf] : keyframes_)
  {
    std::optional<mrpt::img::TColor> overrideColor;
    if (ENV_KEYFRAMES_COLOR_BY_KF)
    {
      // Use the stable kf_id (not iteration order) so a given KF keeps the
      // same color across frames, regardless of evictions elsewhere in the map.
      overrideColor = distinctKfColor(kf_id);
    }

    auto obj = kf.getViz(renderOptions, overrideColor);

    float      pointSize  = renderOptions.point_size;
    const bool isActiveKF = (cached_.icp_search_kfs && cached_.icp_search_kfs->count(kf_id) != 0);
    if (ENV_KEYFRAMES_SHOW_ACTIVE_FRAMES)
    {
      if (isActiveKF)
      {
        pointSize *= 4;
      }
    }
    obj->setPointSize(pointSize);

    outObj.insert(obj);

    const auto nominalAxesLength =
        std::max(renderOptions.keyframes_axes_length, ENV_KEYFRAMES_AXES_LENGTH);

    if (nominalAxesLength > 0)
    {
      const float axesLength =
          (ENV_KEYFRAMES_SHOW_ACTIVE_FRAMES && isActiveKF ? 3.0f : 1.0f) * nominalAxesLength;
      auto glAxes = mrpt::opengl::stock_objects::CornerXYZSimple(axesLength);
      glAxes->setPose(kf.pose());
      outObj.insert(glAxes);
    }

    if (ENV_KEYFRAMES_SHOW_COV || renderOptions.show_covariances)
    {
      auto glCov = kf.getCovarianceEllipsoidViz(renderOptions);
      if (glCov)
      {
        outObj.insert(glCov);
      }
    }
  }

  MRPT_END
}

bool KeyframePointCloudMap::isEmpty() const { return keyframes_.empty(); }

std::map<std::string, mrpt::config::CLoadableOptions*> KeyframePointCloudMap::optionsByName()
{
  return {
      {"creationOptions", &creationOptions},
      {"insertionOptions", &insertionOptions},
      {"likelihoodOptions", &likelihoodOptions},
      {"renderOptions", &renderOptions},
  };
}

bool KeyframePointCloudMap::trySetCreationOptions(
    const mrpt::config::CConfigFileBase& cfg, const std::string& section)
{
  // All TCreationOptions fields are runtime search/threshold parameters -- none of them affect
  // already-built internal structures (each keyframe's own KD-tree, point clouds, etc.), so it
  // is always safe to apply them in place, regardless of whether the map already holds data.
  //
  // Caveat: k_correspondences_for_cov/min_correspondences_for_cov/max_distance_for_cov are
  // copied into each KeyFrame at construction time (used to lazily compute per-point
  // covariances), instead of being read live from `creationOptions`. Propagate the new values
  // to all existing keyframes and invalidate their cached covariances, so they get recomputed
  // with the new parameters next time they are queried.
  // Note on approximate_cov: icp_get_prepared_as_global() compares
  // cached_.icp_search_built_approximate against the live option, so a change here (even
  // with an unchanged active KF set) is picked up and forces a rebuild on the next call.
  TCreationOptions newOpts = creationOptions;
  newOpts.loadFromConfigFile(cfg, section);
  creationOptions = newOpts;

  auto lck = mrpt::lockHelper(*state_mtx_);
  for (auto& kv : keyframes_)
  {
    kv.second.updateCovarianceParams(
        creationOptions.k_correspondences_for_cov, creationOptions.min_correspondences_for_cov,
        creationOptions.max_distance_for_cov);
  }
  return true;
}

void KeyframePointCloudMap::saveMetricMapRepresentationToFile(
    const std::string& filNamePrefix) const
{
  using namespace std::string_literals;

  mrpt::opengl::Scene scene;
  scene.insert(getVisualization());
  scene.saveToFile(filNamePrefix + ".3Dscene"s);
}

const mrpt::maps::CSimplePointsMap* KeyframePointCloudMap::getAsSimplePointsMap() const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  // Return cachedPoints_ or recompute it:
  if (cached_.cachedPoints && cachedPointsLastReturned_ == cached_.cachedPoints)
  {
    return cachedPointsLastReturned_.get();
  }

  // rebuild global point cloud (quite inefficient, but this is only for MOLA->ROS2 bridge).
  cached_.cachedPoints = mrpt::maps::CSimplePointsMap::Create();

  std::optional<std::size_t> estimated_total_points;

  for (const auto& [kf_id, kf] : keyframes_)
  {
    if (!kf.pointcloud())
    {
      continue;
    }

    const auto& kf_pts = *kf.pointcloud_global().get();

    if (!estimated_total_points)
    {
      estimated_total_points = kf_pts.size() * keyframes_.size();
    }

    // Use renderOptions.max_points_per_kf to limit points per KF and
    // predicted total size < renderOptions.max_overall_points
    if (renderOptions.max_points_per_kf > 0 || renderOptions.max_overall_points > 0)
    {
      const float ratio_kf = renderOptions.max_points_per_kf > 0
                                 ? std::min(
                                       1.0f, static_cast<float>(renderOptions.max_points_per_kf) /
                                                 static_cast<float>(kf_pts.size()))
                                 : 1.0f;

      float ratio_overall = 1.0f;
      if (renderOptions.max_overall_points > 0 && estimated_total_points)
      {
        const float predicted_total_after_this_kf = static_cast<float>(*estimated_total_points) +
                                                    static_cast<float>(kf_pts.size()) * ratio_kf;
        ratio_overall = std::min(
            1.0f,
            static_cast<float>(renderOptions.max_overall_points) / predicted_total_after_this_kf);
      }

      const float final_ratio = std::min(ratio_kf, ratio_overall);
      if (final_ratio < 1.0f)
      {
        const auto n_points_to_take =
            static_cast<std::size_t>(final_ratio * static_cast<float>(kf_pts.size()));

        // go by steps to subsample:
        const float step = static_cast<float>(kf_pts.size()) / static_cast<float>(n_points_to_take);
        for (size_t i = 0; i < n_points_to_take; i++)
        {
          const auto            idx = static_cast<size_t>(static_cast<float>(i) * step);
          mrpt::math::TPoint3Df pt;
          kf_pts.getPoint(idx, pt.x, pt.y, pt.z);
          cached_.cachedPoints->insertPointFast(pt.x, pt.y, pt.z);
        }
        continue;
      }
    }
    // else: insert all points:
    cached_.cachedPoints->insertAnotherMap(&kf_pts, mrpt::poses::CPose3D::Identity());
  }
  cachedPointsLastReturned_ = cached_.cachedPoints;

  return cachedPointsLastReturned_.get();
}

// ==========================
//   Keyframe regrouping
// ==========================

namespace
{
/// Packs a voxel coordinate triplet into a single 64-bit key. Uses 21 bits per
/// axis with a large centering offset, covering ~ +/-1e6 voxels per axis.
inline int64_t voxelKey(float x, float y, float z, double inv_voxel)
{
  constexpr int64_t kOffset = 1 << 20;  // center the range around 0
  const auto        q       = [inv_voxel](float c)
  { return static_cast<int64_t>(std::floor(static_cast<double>(c) * inv_voxel)) + kOffset; };
  const int64_t ix = q(x);
  const int64_t iy = q(y);
  const int64_t iz = q(z);
  return (ix << 42) | (iy << 21) | iz;
}

using VoxelSet = std::unordered_set<int64_t>;

VoxelSet cloudToVoxelSet(const mrpt::maps::CPointsMap& pc, double inv_voxel)
{
  VoxelSet    s;
  const auto& xs = pc.getPointsBufferRef_x();
  const auto& ys = pc.getPointsBufferRef_y();
  const auto& zs = pc.getPointsBufferRef_z();
  s.reserve(xs.size());
  for (size_t i = 0; i < xs.size(); i++)
  {
    s.insert(voxelKey(xs[i], ys[i], zs[i], inv_voxel));
  }
  return s;
}

/// Jaccard-min overlap: |A ∩ B| / min(|A|,|B|), in [0,1].
double voxelOverlap(const VoxelSet& a, const VoxelSet& b)
{
  if (a.empty() || b.empty())
  {
    return 0;
  }
  const VoxelSet& small = a.size() <= b.size() ? a : b;
  const VoxelSet& large = a.size() <= b.size() ? b : a;
  size_t          inter = 0;
  for (const auto k : small)
  {
    if (large.count(k) != 0)
    {
      inter++;
    }
  }
  return static_cast<double>(inter) / static_cast<double>(small.size());
}

/// Builds one super-keyframe's cloud, expressed in the anchor (seed) LOCAL
/// frame, by merging the GLOBAL clouds of all `members` (indices into
/// `globals`), optionally voxel-decimating to bound the overlap-induced point
/// blow-up, and finally rotating into the anchor frame. View-direction fields
/// are carried and re-rotated when present in the seed cloud.
mrpt::maps::CPointsMap::Ptr buildSuperKeyframeCloud(
    const std::vector<size_t>& members, const std::vector<mrpt::maps::CPointsMap::Ptr>& globals,
    const mrpt::poses::CPose3D& anchorPose, double decimateVoxel)
{
  const auto& seedGlobal = globals[members.front()];
  const bool  hasView    = (seedGlobal->getPointsBufferRef_float_field("view_x") != nullptr) &&
                       (seedGlobal->getPointsBufferRef_float_field("view_y") != nullptr) &&
                       (seedGlobal->getPointsBufferRef_float_field("view_z") != nullptr);

  const auto makeCloud = [hasView]() -> mrpt::maps::CPointsMap::Ptr
  {
    if (hasView)
    {
      auto gpc = mrpt::maps::CGenericPointsMap::Create();
      gpc->registerField_float("view_x");
      gpc->registerField_float("view_y");
      gpc->registerField_float("view_z");
      return gpc;
    }
    return mrpt::maps::CSimplePointsMap::Create();
  };

  // Merge all member clouds in the GLOBAL frame (view fields already global).
  // When decimation is requested and the clouds carry no custom fields, fold the
  // voxel-dedup INTO the merge, so we never materialize the (potentially
  // enormous) fully-overlapped intermediate cloud. This is what keeps the tool
  // tractable on large maps where each super-keyframe absorbs hundreds of
  // heavily-overlapping keyframes.
  mrpt::maps::CPointsMap::Ptr mergedGlobal = makeCloud();

  const bool decimate = decimateVoxel > 0;
  if (decimate)
  {
    // Voxel-dedup folded into the merge, carrying view_{x,y,z} (global frame)
    // for the first point kept in each voxel when present.
    const double                dinv = 1.0 / decimateVoxel;
    std::unordered_set<int64_t> seen;
    for (const size_t mIdx : members)
    {
      const auto& g  = globals[mIdx];
      const auto& xs = g->getPointsBufferRef_x();
      const auto& ys = g->getPointsBufferRef_y();
      const auto& zs = g->getPointsBufferRef_z();

      const mrpt::aligned_std_vector<float>* vx = nullptr;
      const mrpt::aligned_std_vector<float>* vy = nullptr;
      const mrpt::aligned_std_vector<float>* vz = nullptr;
      if (hasView)
      {
        vx = g->getPointsBufferRef_float_field("view_x");
        vy = g->getPointsBufferRef_float_field("view_y");
        vz = g->getPointsBufferRef_float_field("view_z");
      }

      for (size_t i = 0; i < xs.size(); i++)
      {
        if (!seen.insert(voxelKey(xs[i], ys[i], zs[i], dinv)).second)
        {
          continue;
        }
        mergedGlobal->insertPointFast(xs[i], ys[i], zs[i]);
        if (hasView && vx != nullptr && vy != nullptr && vz != nullptr)
        {
          mergedGlobal->insertPointField_float("view_x", (*vx)[i]);
          mergedGlobal->insertPointField_float("view_y", (*vy)[i]);
          mergedGlobal->insertPointField_float("view_z", (*vz)[i]);
        }
      }
    }
    mergedGlobal->mark_as_modified();
  }
  else
  {
    for (const size_t mIdx : members)
    {
      const auto& g = globals[mIdx];
#if MRPT_VERSION >= 0x020f0b  // 2.15.11
      mergedGlobal->insertAnotherMap(
          g.get(), mrpt::poses::CPose3D::Identity(), false /*filterOutPointsAtZero*/,
          false /*autoRegisterAllSourceFields*/);
#else
      mergedGlobal->insertAnotherMap(g.get(), mrpt::poses::CPose3D::Identity());
#endif
    }
  }

  // Express the merged cloud in the anchor (seed) local frame.
  // (Identity - anchorPose) == anchorPose^{-1}.
  const mrpt::poses::CPose3D anchorInv = mrpt::poses::CPose3D::Identity() - anchorPose;

  mrpt::maps::CPointsMap::Ptr localCloud = makeCloud();
#if MRPT_VERSION >= 0x020f0b  // 2.15.11
  localCloud->insertAnotherMap(
      mergedGlobal.get(), anchorInv, false /*filterOutPointsAtZero*/,
      false /*autoRegisterAllSourceFields*/);
#else
  localCloud->insertAnotherMap(mergedGlobal.get(), anchorInv);
#endif
  // insertAnotherMap copies the (still global-frame) view vectors verbatim;
  // rotate them into the anchor-local frame to satisfy the KF contract.
  if (hasView)
  {
#if defined(MOLA_MM_HAS_ROTATE_VIEW_HEADER)
    rotateViewDirectionFieldsOrFallback(*localCloud, anchorInv);
#else
    rotateViewDirectionFieldsLegacy(*localCloud, anchorInv);
#endif
  }

  return localCloud;
}
}  // namespace

std::shared_ptr<KeyframePointCloudMap> KeyframePointCloudMap::regroupKeyframes(
    const RegroupParams& params, const std::function<void(const std::string&)>& logCb) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  const auto log = [&](const std::string& s)
  {
    if (logCb)
    {
      logCb(s);
    }
  };

  // ---- 1) Collect keyframes that actually hold a (non-empty) cloud ----
  struct KFInfo
  {
    KeyFrameID              id = 0;
    mrpt::math::TPoint3D    center;  // global sensing-sphere center
    double                  radius = 0;  // global sensing radius (half bbox diagonal)
    mrpt::poses::CPose3D    pose;  // keyframe pose in the map frame
    mrpt::Clock::time_point timestamp;
  };
  std::vector<KFInfo>                      kfs;
  std::vector<mrpt::maps::CPointsMap::Ptr> globals;  // parallel to kfs: global-frame clouds

  for (const auto& [id, kf] : keyframes_)
  {
    if (!kf.pointcloud() || kf.pointcloud()->empty())
    {
      continue;
    }
    const auto&                g    = kf.pointcloud_global();
    const auto                 bbox = g->boundingBox();
    const mrpt::math::TPoint3D center{
        0.5 * (bbox.min.x + bbox.max.x), 0.5 * (bbox.min.y + bbox.max.y),
        0.5 * (bbox.min.z + bbox.max.z)};
    const double diag = std::sqrt(
        mrpt::square(bbox.max.x - bbox.min.x) + mrpt::square(bbox.max.y - bbox.min.y) +
        mrpt::square(bbox.max.z - bbox.min.z));
    kfs.push_back({id, center, 0.5 * diag, kf.pose(), kf.timestamp});
    globals.push_back(g);
  }

  const size_t n = kfs.size();

  auto out               = KeyframePointCloudMap::Create();
  out->creationOptions   = creationOptions;
  out->insertionOptions  = insertionOptions;
  out->likelihoodOptions = likelihoodOptions;
  out->renderOptions     = renderOptions;

  if (n == 0)
  {
    log("[regroup] Input map has no keyframes with clouds; returning empty map.");
    return out;
  }

  // ---- 1.5) unify_all: skip clustering, merge everything into one super-keyframe ----
  if (params.unify_all)
  {
    log(mrpt::format(
        "[regroup] unify_all: merging all %zu keyframes into a single super-keyframe", n));

    const auto& seed = kfs.front();

    std::vector<size_t> allMembers(n);
    std::iota(allMembers.begin(), allMembers.end(), size_t(0));

    mrpt::maps::CPointsMap::Ptr cloud =
        buildSuperKeyframeCloud(allMembers, globals, seed.pose, params.merge_decimate_voxel);

    auto [it, isNew] = out->keyframes_.try_emplace(
        KeyFrameID{0}, creationOptions.k_correspondences_for_cov,
        creationOptions.min_correspondences_for_cov, creationOptions.max_distance_for_cov);
    KeyFrame& nkf = it->second;
    nkf.timestamp = seed.timestamp;
    nkf.pose(seed.pose);
    nkf.pointcloud(cloud);
    out->last_inserted_kf_id_ = 0;
    out->next_free_kf_id_     = 1;

    log("[regroup] unify_all: done, 1 super-keyframe produced.");
    return out;
  }

  // ---- 2) Determine the overlap voxel size ----
  double voxelSize = params.voxel_size;
  if (voxelSize <= 0)
  {
    std::vector<double> radii;
    radii.reserve(n);
    for (const auto& k : kfs)
    {
      radii.push_back(k.radius);
    }
    const auto mid = static_cast<std::ptrdiff_t>(radii.size() / 2);
    std::nth_element(radii.begin(), radii.begin() + mid, radii.end());
    const double medianR = radii[radii.size() / 2];
    voxelSize            = std::clamp(medianR * 0.02, 0.1, 2.0);
  }
  const double invVoxel = 1.0 / voxelSize;
  log(mrpt::format(
      "[regroup] %zu keyframes with clouds, overlap voxel size = %.3f m", n, voxelSize));

  // ---- 3) Voxelize all clouds (global frame) ----
  std::vector<VoxelSet> voxels(n);
  for (size_t i = 0; i < n; i++)
  {
    voxels[i] = cloudToVoxelSet(*globals[i], invVoxel);
  }

  // ---- 4) Build the keyframe adjacency graph (overlap edges) ----
  // Nodes = keyframes; edge weight = voxel Jaccard-min overlap. Only pairs whose
  // sensing spheres intersect are considered (broad phase); edges are kept when
  // their overlap is >= edge_overlap.
  std::vector<std::vector<std::pair<size_t, double>>> adj(n);
  size_t                                              numEdges = 0;
  for (size_t i = 0; i < n; i++)
  {
    for (size_t j = i + 1; j < n; j++)
    {
      const double d = (kfs[i].center - kfs[j].center).norm();
      if (d > kfs[i].radius + kfs[j].radius)
      {
        continue;  // spheres do not intersect: no possible overlap
      }
      const double ov = voxelOverlap(voxels[i], voxels[j]);
      if (ov < params.edge_overlap)
      {
        continue;
      }
      adj[i].emplace_back(j, ov);
      adj[j].emplace_back(i, ov);
      numEdges++;
    }
  }
  log(mrpt::format(
      "[regroup] adjacency graph: %zu edges (avg degree %.1f)", numEdges,
      2.0 * static_cast<double>(numEdges) / static_cast<double>(n)));

  // ---- 5) Greedy overlapping set-cover clustering ----
  // Seeds are chosen from the densest / most-connected keyframes first. Each
  // super-keyframe grows by absorbing graph neighbors (highest overlap first)
  // while staying within the seed's spatial extent cap. A member is marked
  // "covered" only if it lies within the inner core; boundary members remain
  // uncovered and thus seed/join neighboring groups -> deliberate overlap.
  std::vector<bool> covered(n, false);

  std::vector<size_t> order(n);
  std::iota(order.begin(), order.end(), size_t(0));
  std::sort(
      order.begin(), order.end(),
      [&](size_t a, size_t b)
      {
        const double sa =
            static_cast<double>(voxels[a].size()) * (1.0 + static_cast<double>(adj[a].size()));
        const double sb =
            static_cast<double>(voxels[b].size()) * (1.0 + static_cast<double>(adj[b].size()));
        if (sa != sb)
        {
          return sa > sb;
        }
        return kfs[a].id < kfs[b].id;
      });

  std::vector<std::vector<size_t>> clusters;  // each: member indices, seed first

  for (const size_t seed : order)
  {
    if (covered[seed])
    {
      continue;
    }

    const double extentCap = std::max(params.extent_factor * kfs[seed].radius, voxelSize);

    std::vector<size_t>        members   = {seed};
    std::unordered_set<size_t> memberSet = {seed};
    VoxelSet                   Vc        = voxels[seed];

    // Grow: repeatedly add the best eligible neighbor of any current member.
    for (;;)
    {
      double bestOv = -1.0;
      size_t bestN  = 0;
      bool   found  = false;
      for (const size_t m : members)
      {
        for (const auto& [nb, w] : adj[m])
        {
          if (memberSet.count(nb) != 0)
          {
            continue;
          }
          if ((kfs[nb].center - kfs[seed].center).norm() > extentCap)
          {
            continue;
          }
          const double ov = voxelOverlap(voxels[nb], Vc);
          if (ov < params.edge_overlap)
          {
            continue;
          }
          if (ov > bestOv)
          {
            bestOv = ov;
            bestN  = nb;
            found  = true;
          }
        }
      }
      if (!found)
      {
        break;
      }
      members.push_back(bestN);
      memberSet.insert(bestN);
      Vc.insert(voxels[bestN].begin(), voxels[bestN].end());
    }

    // Mark inner-core members as covered (boundary members stay available).
    const double coreRadius = params.core_fraction * extentCap;
    for (const size_t m : members)
    {
      if ((kfs[m].center - kfs[seed].center).norm() <= coreRadius)
      {
        covered[m] = true;
      }
    }
    covered[seed] = true;  // always: guarantees the loop makes progress

    clusters.push_back(std::move(members));
  }

  log(mrpt::format(
      "[regroup] %zu keyframes -> %zu super-keyframes (%.2fx reduction)", n, clusters.size(),
      clusters.empty() ? 0.0 : static_cast<double>(n) / static_cast<double>(clusters.size())));

  // ---- 5.5) Absorb tiny "island" clusters into their nearest neighbor ----
  // A cluster whose total point count is a small fraction of the largest cluster's
  // is too sparse to usefully stand on its own: at runtime, its pose can still win
  // proximity-based nearest-keyframe search (see TCreationOptions::density_penalty_*)
  // over a much better-populated neighbor, starving ICP of real geometry. Rather than
  // leave it as a standalone super-keyframe, fold it into its nearest neighbor cluster.
  if (params.island_merge_fraction > 0 && clusters.size() > 1)
  {
    const auto clusterPoints = [&](const std::vector<size_t>& members) -> size_t
    {
      size_t total = 0;
      for (const size_t m : members)
      {
        total += globals[m]->size();
      }
      return total;
    };
    // The seed (first member) center is used as the cluster's representative position.
    const auto clusterCenter = [&](const std::vector<size_t>& members)
    { return kfs[members.front()].center; };

    // Clusters whose nearest neighbor turns out to be farther than a reasonable bound (see
    // below) are left standalone rather than glued to a spatially disjoint cluster; track
    // them by their (stable) seed index so the smallest-first search does not retry them.
    std::unordered_set<size_t> unmergeableSeeds;

    size_t numIslandsAbsorbed = 0;
    for (;;)
    {
      if (clusters.size() <= 1)
      {
        break;
      }

      size_t maxPoints = 0;
      for (const auto& c : clusters)
      {
        maxPoints = std::max(maxPoints, clusterPoints(c));
      }
      const double minPoints = params.island_merge_fraction * static_cast<double>(maxPoints);

      // Find the smallest cluster below threshold (process smallest-first so a
      // chain of tiny islands absorbs into progressively larger neighbors).
      size_t smallestIdx = clusters.size();
      size_t smallestPts = 0;
      for (size_t i = 0; i < clusters.size(); i++)
      {
        if (unmergeableSeeds.count(clusters[i].front()) != 0)
        {
          continue;
        }
        const size_t pts = clusterPoints(clusters[i]);
        if (static_cast<double>(pts) < minPoints &&
            (smallestIdx == clusters.size() || pts < smallestPts))
        {
          smallestIdx = i;
          smallestPts = pts;
        }
      }
      if (smallestIdx == clusters.size())
      {
        break;  // no (mergeable) island left
      }

      // Find the nearest other cluster by center distance:
      const auto islandCenter = clusterCenter(clusters[smallestIdx]);
      size_t     nearestIdx   = clusters.size();
      double     nearestDist  = std::numeric_limits<double>::max();
      for (size_t j = 0; j < clusters.size(); j++)
      {
        if (j == smallestIdx)
        {
          continue;
        }
        const double d = (clusterCenter(clusters[j]) - islandCenter).norm();
        if (d < nearestDist)
        {
          nearestDist = d;
          nearestIdx  = j;
        }
      }
      ASSERT_(nearestIdx != clusters.size());

      // Reject absorption into a spatially disjoint neighbor: cap the merge distance at
      // extent_factor times the sum of both clusters' (seed) sensing radii, the same
      // scale used to cap a super-keyframe's own extent during growth (step 5 above).
      const double mergeDistCap =
          params.extent_factor *
          (kfs[clusters[smallestIdx].front()].radius + kfs[clusters[nearestIdx].front()].radius);
      if (nearestDist > mergeDistCap)
      {
        unmergeableSeeds.insert(clusters[smallestIdx].front());
        continue;
      }

      // Absorb: append the island's members into its nearest neighbor, then drop it.
      clusters[nearestIdx].insert(
          clusters[nearestIdx].end(), clusters[smallestIdx].begin(), clusters[smallestIdx].end());
      clusters.erase(clusters.begin() + static_cast<std::ptrdiff_t>(smallestIdx));

      numIslandsAbsorbed++;
    }

    if (numIslandsAbsorbed > 0)
    {
      log(mrpt::format(
          "[regroup] absorbed %zu isolated/undersized cluster(s) into their nearest neighbor "
          "(island_merge_fraction=%.2f) -> %zu super-keyframes remain",
          numIslandsAbsorbed, params.island_merge_fraction, clusters.size()));
    }
  }

  // ---- 6) Build the output map: one super-keyframe per cluster ----
  // Building each super-keyframe cloud (merge + voxel-decimate) is by far the
  // most expensive part of this function and fully independent across
  // clusters, so it is parallelized; the actual insertion into `out` (which
  // assigns sequential KF ids) is kept single-threaded below.
  std::vector<mrpt::maps::CPointsMap::Ptr> superKfClouds(clusters.size());

#if defined(MOLA_METRIC_MAPS_USE_TBB)
  tbb::parallel_for(
      static_cast<size_t>(0), clusters.size(),
      [&](size_t i)
#else
  for (size_t i = 0; i < clusters.size(); i++)
#endif
      {
        const auto& members = clusters[i];
        const auto& seed    = kfs[members.front()];
        superKfClouds[i] =
            buildSuperKeyframeCloud(members, globals, seed.pose, params.merge_decimate_voxel);
      }
#if defined(MOLA_METRIC_MAPS_USE_TBB)
  );
#endif

  KeyFrameID nextId = 0;
  for (size_t i = 0; i < clusters.size(); i++)
  {
    const auto& seed = kfs[clusters[i].front()];

    // Insert as a new super-keyframe (caches build lazily on first use / load).
    auto [it, isNew] = out->keyframes_.try_emplace(
        nextId, creationOptions.k_correspondences_for_cov,
        creationOptions.min_correspondences_for_cov, creationOptions.max_distance_for_cov);
    KeyFrame& nkf = it->second;
    nkf.timestamp = seed.timestamp;
    nkf.pose(seed.pose);
    nkf.pointcloud(superKfClouds[i]);
    out->last_inserted_kf_id_ = nextId;
    nextId++;
  }
  out->next_free_kf_id_ = nextId;

  return out;
}

// ==========================
//   Options
// ==========================

void KeyframePointCloudMap::TInsertionOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(remove_frames_farther_than, double, c, s);
}

void KeyframePointCloudMap::TInsertionOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [KeyframePointCloudMap::TInsertionOptions] ------- \n\n";
  LOADABLEOPTS_DUMP_VAR(remove_frames_farther_than, double);
}

void KeyframePointCloudMap::TInsertionOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  const int8_t version = 0;
  out << version;
  out << remove_frames_farther_than;
}

void KeyframePointCloudMap::TInsertionOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version = 0;
  in >> version;
  switch (version)
  {
    case 0:
    {
      in >> remove_frames_farther_than;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void KeyframePointCloudMap::TLikelihoodOptions::loadFromConfigFile(
    [[maybe_unused]] const mrpt::config::CConfigFileBase& source,
    [[maybe_unused]] const std::string&                   section)
{
}

void KeyframePointCloudMap::TLikelihoodOptions::dumpToTextStream(
    [[maybe_unused]] std::ostream& out) const
{
  out << "\n------ [KeyframePointCloudMap::TLikelihoodOptions] ------- \n\n";
}

void KeyframePointCloudMap::TLikelihoodOptions::writeToStream(  // NOLINT
    mrpt::serialization::CArchive& out) const
{
  out.WriteAs<uint8_t>(0);
}

void KeyframePointCloudMap::TLikelihoodOptions::readFromStream(  // NOLINT
    mrpt::serialization::CArchive& in)
{
  const auto version = in.ReadAs<uint8_t>();
  (void)version;
}

void KeyframePointCloudMap::TRenderOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(point_size, float, c, s);
  MRPT_LOAD_CONFIG_VAR(color.R, float, c, s);
  MRPT_LOAD_CONFIG_VAR(color.G, float, c, s);
  MRPT_LOAD_CONFIG_VAR(color.B, float, c, s);
  MRPT_LOAD_CONFIG_VAR(color.A, float, c, s);
  MRPT_LOAD_CONFIG_VAR(max_points_per_kf, uint64_t, c, s);
  MRPT_LOAD_CONFIG_VAR(max_overall_points, uint64_t, c, s);
  colormap = c.read_enum(s, "colormap", this->colormap);
  MRPT_LOAD_CONFIG_VAR(recolorByPointField, string, c, s);
  MRPT_LOAD_CONFIG_VAR(keyframes_axes_length, float, c, s);
  MRPT_LOAD_CONFIG_VAR(show_covariances, bool, c, s);
  MRPT_LOAD_CONFIG_VAR(cov_color.R, float, c, s);
  MRPT_LOAD_CONFIG_VAR(cov_color.G, float, c, s);
  MRPT_LOAD_CONFIG_VAR(cov_color.B, float, c, s);
  MRPT_LOAD_CONFIG_VAR(cov_color.A, float, c, s);
  MRPT_LOAD_CONFIG_VAR(show_cov_decimation, uint64_t, c, s);
}

void KeyframePointCloudMap::TRenderOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [KeyframePointCloudMap::TRenderOptions] ------- \n\n";

  LOADABLEOPTS_DUMP_VAR(point_size, float);
  LOADABLEOPTS_DUMP_VAR(color.R, float);
  LOADABLEOPTS_DUMP_VAR(color.G, float);
  LOADABLEOPTS_DUMP_VAR(color.B, float);
  LOADABLEOPTS_DUMP_VAR(color.A, float);
  LOADABLEOPTS_DUMP_VAR(colormap, int);
  using std::string;
  LOADABLEOPTS_DUMP_VAR(recolorByPointField, string);
  LOADABLEOPTS_DUMP_VAR(max_points_per_kf, int);
  LOADABLEOPTS_DUMP_VAR(max_overall_points, int);
  LOADABLEOPTS_DUMP_VAR(keyframes_axes_length, float);
  LOADABLEOPTS_DUMP_VAR(show_covariances, bool);
  LOADABLEOPTS_DUMP_VAR(show_cov_decimation, int);
}

void KeyframePointCloudMap::TRenderOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const int8_t version = 4;
  out << version;
  out << point_size << color << int8_t(colormap) << recolorByPointField;  // v2
  out << max_points_per_kf << max_overall_points;  // v1
  out << keyframes_axes_length;  // v3
  out << show_covariances << show_cov_decimation;  // v4
}

void KeyframePointCloudMap::TRenderOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version = 0;
  in >> version;
  *this = {};
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    {
      in >> point_size;
      in >> this->color;
      in.ReadAsAndCastTo<int8_t>(this->colormap);

      if (version >= 2)
      {
        in >> recolorByPointField;
      }
      else
      {
        switch (in.ReadAs<uint8_t>())
        {
          case 0:
            recolorByPointField = "x";
            break;
          case 1:
            recolorByPointField = "y";
            break;
          default:
            recolorByPointField = "z";
            break;
        }
      }

      if (version >= 1)
      {
        in >> max_points_per_kf >> max_overall_points;
      }
      if (version >= 3)
      {
        in >> keyframes_axes_length;
      }
      if (version >= 4)
      {
        in >> show_covariances >> show_cov_decimation;
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void KeyframePointCloudMap::TCreationOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR_REQUIRED_CS(max_search_keyframes, uint64_t);
  MRPT_LOAD_CONFIG_VAR_REQUIRED_CS(k_correspondences_for_cov, uint64_t);
  MRPT_LOAD_CONFIG_VAR_CS(min_correspondences_for_cov, uint64_t);
  MRPT_LOAD_CONFIG_VAR_CS(max_distance_for_cov, double);
  MRPT_LOAD_CONFIG_VAR_CS(rotation_distance_weight, double);
  MRPT_LOAD_CONFIG_VAR_CS(num_diverse_keyframes, uint64_t);
  MRPT_LOAD_CONFIG_VAR_CS(use_view_direction_filter, bool);
  MRPT_LOAD_CONFIG_VAR_CS(max_view_angle_deg, double);
  MRPT_LOAD_CONFIG_VAR_CS(serialize_kdtrees, bool);
  MRPT_LOAD_CONFIG_VAR_CS(serialize_covariances, bool);
  MRPT_LOAD_CONFIG_VAR_CS(approximate_cov, bool);
  MRPT_LOAD_CONFIG_VAR_CS(density_penalty_min_points, uint64_t);
  MRPT_LOAD_CONFIG_VAR_CS(density_penalty_max_m, double);
}

void KeyframePointCloudMap::TCreationOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [KeyframePointCloudMap::TCreationOptions] ------- \n\n";
  LOADABLEOPTS_DUMP_VAR(max_search_keyframes, int);
  LOADABLEOPTS_DUMP_VAR(k_correspondences_for_cov, int);
  LOADABLEOPTS_DUMP_VAR(min_correspondences_for_cov, int);
  LOADABLEOPTS_DUMP_VAR(max_distance_for_cov, double);
  LOADABLEOPTS_DUMP_VAR(rotation_distance_weight, double);
  LOADABLEOPTS_DUMP_VAR(num_diverse_keyframes, int);
  LOADABLEOPTS_DUMP_VAR(use_view_direction_filter, bool);
  LOADABLEOPTS_DUMP_VAR(max_view_angle_deg, double);
  LOADABLEOPTS_DUMP_VAR(serialize_kdtrees, bool);
  LOADABLEOPTS_DUMP_VAR(serialize_covariances, bool);
  LOADABLEOPTS_DUMP_VAR(approximate_cov, bool);
  LOADABLEOPTS_DUMP_VAR(density_penalty_min_points, int);
  LOADABLEOPTS_DUMP_VAR(density_penalty_max_m, double);
}

void KeyframePointCloudMap::TCreationOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  out.WriteAs<uint8_t>(7);  // version
  out << max_search_keyframes << k_correspondences_for_cov;
  out << rotation_distance_weight << num_diverse_keyframes;  // v1
  out << use_view_direction_filter << max_view_angle_deg;  // v2
  out << min_correspondences_for_cov << max_distance_for_cov;  // v3
  out << serialize_kdtrees;  // v4
  out << approximate_cov;  // v5
  out << serialize_covariances;  // v6
  out << density_penalty_min_points << density_penalty_max_m;  // v7
}

void KeyframePointCloudMap::TCreationOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  *this = {};

  const auto version = in.ReadAs<uint8_t>();
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    {
      in >> max_search_keyframes >> k_correspondences_for_cov;
      if (version >= 1)
      {
        in >> rotation_distance_weight >> num_diverse_keyframes;
      }
      if (version >= 2)
      {
        in >> use_view_direction_filter >> max_view_angle_deg;
      }
      if (version >= 3)
      {
        in >> min_correspondences_for_cov;
        in >> max_distance_for_cov;
      }
      if (version >= 4)
      {
        in >> serialize_kdtrees;
      }
      if (version >= 5)
      {
        in >> approximate_cov;
      }
      if (version >= 6)
      {
        in >> serialize_covariances;
      }
      if (version >= 7)
      {
        in >> density_penalty_min_points >> density_penalty_max_m;
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

// ==========================
//   Protected / Private
// ==========================

void KeyframePointCloudMap::internal_clear()
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  keyframes_.clear();
  last_inserted_kf_id_.reset();
  evicted_kf_ids_.clear();
  next_free_kf_id_ = 0;
  cached_.reset();
}

KeyframePointCloudMap::KeyFrameID KeyframePointCloudMap::nextFreeKeyFrameID_public() const
{
  auto lck = mrpt::lockHelper(*state_mtx_);
  return nextFreeKeyFrameID();
}

std::map<KeyframePointCloudMap::KeyFrameID, mrpt::poses::CPose3D>
    KeyframePointCloudMap::keyframePoses() const
{
  auto                                       lck = mrpt::lockHelper(*state_mtx_);
  std::map<KeyFrameID, mrpt::poses::CPose3D> out;
  for (const auto& [id, kf] : keyframes_) out.emplace(id, kf.pose());
  return out;
}

void KeyframePointCloudMap::setKeyframePose(KeyFrameID id, const mrpt::poses::CPose3D& new_pose)
{
  auto lck = mrpt::lockHelper(*state_mtx_);
  auto it  = keyframes_.find(id);
  if (it == keyframes_.end()) return;
  it->second.pose(new_pose);
  // KF-local caches are invalidated by KeyFrame::pose(). Top-level caches
  // (bounding box, icp submap) also depend on KF poses, so invalidate them:
  cached_.reset();
}

std::optional<KeyframePointCloudMap::KeyFrameID> KeyframePointCloudMap::lastInsertedKeyFrameID()
    const
{
  auto lck = mrpt::lockHelper(*state_mtx_);
  return last_inserted_kf_id_;
}

std::vector<KeyframePointCloudMap::KeyFrameID> KeyframePointCloudMap::drainEvictedKeyFrameIDs()
{
  auto                    lck = mrpt::lockHelper(*state_mtx_);
  std::vector<KeyFrameID> out;
  out.swap(evicted_kf_ids_);
  return out;
}

bool KeyframePointCloudMap::internal_insertObservation(
    const mrpt::obs::CObservation& obs, const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  // Get robot pose for insertion pose:
  mrpt::poses::CPose3D pc_in_map;
  if (robotPose)
  {
    pc_in_map = *robotPose;
  }

  // Remove old key-frames if requested:
  if (insertionOptions.remove_frames_farther_than > 0)
  {
    for (auto it = keyframes_.begin(); it != keyframes_.end();)
    {
      const double dist = pc_in_map.distanceTo(it->second.pose());
      if (dist > insertionOptions.remove_frames_farther_than)
      {
        evicted_kf_ids_.push_back(it->first);
        it = keyframes_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  // Observation must be a point cloud:
  if (const auto* obsPC = dynamic_cast<const mrpt::obs::CObservationPointCloud*>(&obs); obsPC)
  {
    ASSERT_(obsPC->pointcloud);

    // Add KF: allocate a fresh monotonic id (never reused, even after eviction).
    auto [it, isNew] = keyframes_.try_emplace(
        next_free_kf_id_++, creationOptions.k_correspondences_for_cov,
        creationOptions.min_correspondences_for_cov, creationOptions.max_distance_for_cov);
    auto& new_kf = it->second;

    new_kf.timestamp = obs.timestamp;
    new_kf.pose(pc_in_map);
    new_kf.pointcloud(obsPC->pointcloud);

    new_kf.buildCache();
    cached_.reset();

    last_inserted_kf_id_ = it->first;

    return true;
  }

  // Not of supported type, we cannot insert into our map:
  return false;
}

double KeyframePointCloudMap::internal_computeObservationLikelihood(
    [[maybe_unused]] const mrpt::obs::CObservation& obs,
    [[maybe_unused]] const mrpt::poses::CPose3D&    takenFrom) const
{
  return .0;
}

double KeyframePointCloudMap::internal_computeObservationLikelihoodPointCloud3D(  // NOLINT
    [[maybe_unused]] const mrpt::poses::CPose3D&   pc_in_map,
    [[maybe_unused]] [[maybe_unused]] const float* xs, [[maybe_unused]] const float* ys,
    [[maybe_unused]] const float* zs, [[maybe_unused]] const std::size_t num_pts) const
{
  return .0;
}

bool KeyframePointCloudMap::internal_canComputeObservationLikelihood(
    [[maybe_unused]] const mrpt::obs::CObservation& obs) const
{
  return false;
}

//  =========== KeyFrame ============

void KeyframePointCloudMap::KeyFrame::updateBBox() const
{
  if (!pointcloud_)
  {
    cached_bbox_local_ = mrpt::math::TBoundingBoxf({0, 0, 0}, {0, 0, 0});
  }
  else
  {
    cached_bbox_local_ = pointcloud_->boundingBox();
  }
}

mrpt::math::TBoundingBoxf KeyframePointCloudMap::KeyFrame::localBoundingBox() const
{
  if (!cached_bbox_local_)
  {
    updateBBox();
  }
  return *cached_bbox_local_;
}

void KeyframePointCloudMap::KeyFrame::buildCache() const
{
  // Compute bbox:
  updateBBox();

  // Build KD-tree:
  ASSERT_(pointcloud_);
  pointcloud_->kdTreeEnsureIndexBuilt3D();

  // Build per-point covariances:
  computeCovariancesAndDensity();
}

void KeyframePointCloudMap::KeyFrame::updatePointsGlobal() const
{
  if (!pointcloud_)
  {
    return;
  }

  // Check whether the source cloud carries view-direction fields.
  // view_{x,y,z} are unit vectors in the *local KF frame*; they must be
  // rotated to the global frame alongside the point coordinates.
  const auto* src_vx   = pointcloud_->getPointsBufferRef_float_field("view_x");
  const auto* src_vy   = pointcloud_->getPointsBufferRef_float_field("view_y");
  const auto* src_vz   = pointcloud_->getPointsBufferRef_float_field("view_z");
  const bool  has_view = (src_vx != nullptr) && (src_vy != nullptr) && (src_vz != nullptr);

  // Choose the global-cloud type: CGenericPointsMap when we need to carry
  // custom fields, plain CSimplePointsMap otherwise (cheaper).
  if (!pointcloud_global_ ||
      (has_view && pointcloud_global_->getPointsBufferRef_float_field("view_x") == nullptr))
  {
    if (has_view)
    {
      auto gpc = mrpt::maps::CGenericPointsMap::Create();
      gpc->registerField_float("view_x");
      gpc->registerField_float("view_y");
      gpc->registerField_float("view_z");
      pointcloud_global_ = gpc;
    }
    else
    {
      pointcloud_global_ = mrpt::maps::CSimplePointsMap::Create();
    }
  }

  // preserve existing fields but remove their data:
  pointcloud_global_->resize(0);

  // XYZ coordinates are rotated+translated; custom fields are copied verbatim.
  // This automatically registers all source fields
  pointcloud_global_->insertAnotherMap(pointcloud_.get(), pose());

  // insertAnotherMap() does not know that view_{x,y,z} are direction vectors,
  // so it copies local-frame values as-is.  Rotate them to the global frame now.
  if (has_view)
  {
#if defined(MOLA_MM_HAS_ROTATE_VIEW_HEADER)
    rotateViewDirectionFieldsOrFallback(*pointcloud_global_, pose_);
#else
    rotateViewDirectionFieldsLegacy(*pointcloud_global_, pose_);
#endif
  }
}

void KeyframePointCloudMap::KeyFrame::computeCovariancesAndDensity() const
{
  ASSERT_(pointcloud_);
  const auto point_count = pointcloud_->size();

  if (cached_cov_local_.size() == point_count)
  {
    return;  // Already computed
  }

  std::chrono::high_resolution_clock::time_point start;
  if (ENV_DO_PROFILE_COV)
  {
    start = std::chrono::high_resolution_clock::now();
  }
#if DO_VIZ_DEBUG
  static int call_counter = 0;
  call_counter++;
#endif

  // Resize:
  cached_cov_local_.resize(point_count);
  cached_cov_global_.clear();  // invalidate

  if (point_count < 3)
  {
    // Nothing to do
    cloud_density_ = 0;
    return;
  }

  // Compute using KD-tree:
  std::vector<float> sum_k_sq_distances(point_count);

  // Never request more neighbors than points actually available: requesting
  // more than the kd-tree holds leaves the trailing entries of out_idx/
  // out_dist_sqr unfilled (stale/zero) inside MRPT's
  // kdTreeNClosestPoint3DIdx(), which would silently corrupt the covariance
  // below with bogus duplicated-point-0 entries.
  const size_t K_CORRESPONDENCES = std::min<size_t>(k_correspondences_for_cov_, point_count);
  const size_t MIN_CORRESPONDENCES =
      std::min<size_t>(min_correspondences_for_cov_, K_CORRESPONDENCES);
  // nanoflann's RKNNResultSet expects the maximum SEARCH DISTANCE SQUARED:
  const float MAX_DIST_SQR_FOR_COV =
      static_cast<float>(max_distance_for_cov_ * max_distance_for_cov_);
  const auto normalization =
      static_cast<float>(((K_CORRESPONDENCES - 1) * (2 + K_CORRESPONDENCES))) / 2;

  const auto& xs = pointcloud_->getPointsBufferRef_x();
  const auto& ys = pointcloud_->getPointsBufferRef_y();
  const auto& zs = pointcloud_->getPointsBufferRef_z();

#if defined(MOLA_METRIC_MAPS_USE_TBB)
  tbb::parallel_for(
      static_cast<size_t>(0), point_count,
      [&](size_t i)
#else
  for (size_t i = 0; i < point_count; i++)
#endif
      {
        std::vector<size_t> k_indices;
        std::vector<float>  k_sq_distances;

#if defined(MOLA_MM_HAS_RKNN_SEARCH)
        pointcloud_->kdTreeNClosestPoint3DIdx(
            xs[i], ys[i], zs[i], K_CORRESPONDENCES, k_indices, k_sq_distances,
            MAX_DIST_SQR_FOR_COV);
#else
    // Equivalent to the RKNN search above: a plain kNN returns its
    // neighbors sorted by ascending squared distance, so cutting the
    // result at the first entry that reaches the limit leaves exactly the
    // ones an RKNN search would have accepted (it keeps distances strictly
    // below the limit too). The only cost is visiting all K neighbors
    // before dropping the far ones.
    pointcloud_->kdTreeNClosestPoint3DIdx(
        xs[i], ys[i], zs[i], K_CORRESPONDENCES, k_indices, k_sq_distances);

    const auto within_radius = static_cast<size_t>(std::distance(
        k_sq_distances.begin(),
        std::lower_bound(k_sq_distances.begin(), k_sq_distances.end(), MAX_DIST_SQR_FOR_COV)));
    k_indices.resize(within_radius);
    k_sq_distances.resize(within_radius);
#endif

        // Too few neighbors actually found: a plane/line fit from this few
        // samples is unreliable, so fall back to an isotropic covariance
        // instead of an (over-confident, possibly degenerate) regularized one.
        if (k_indices.size() < MIN_CORRESPONDENCES)
        {
          sum_k_sq_distances[i] = 0;
          cached_cov_local_[i]  = mrpt::math::CMatrixFloat33::Identity();
#if defined(MOLA_METRIC_MAPS_USE_TBB)
          return;
#else
      continue;
#endif
        }

        sum_k_sq_distances[i] =
            std::accumulate(k_sq_distances.begin() + 1, k_sq_distances.end(), 0.0f) / normalization;

        Eigen::Matrix<double, 3, -1> neighbors(3, k_indices.size());
        for (size_t j = 0; j < k_indices.size(); j++)
        {
          neighbors(0, static_cast<Eigen::Index>(j)) = static_cast<double>(xs[k_indices[j]]);
          neighbors(1, static_cast<Eigen::Index>(j)) = static_cast<double>(ys[k_indices[j]]);
          neighbors(2, static_cast<Eigen::Index>(j)) = static_cast<double>(zs[k_indices[j]]);
        }

        // neighbors.colwise() -= neighbors.rowwise().mean().eval();
        neighbors.colwise() -= Eigen::Vector3d(xs[i], ys[i], zs[i]);
        const Eigen::Matrix3d cov =
            neighbors * neighbors.transpose() / static_cast<double>(k_indices.size());

        // Plane regularization (see DLIO'2023 or Thrun's GICP paper)
        // ------------------------------------------------------------
        // Regularization of singular values.
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // SVD sorts eigenvalues in decreasing order, so the last one
        // is the smallest (normal direction of a plane):
        const Eigen::Vector3d values = Eigen::Vector3d(1.0, 1.0, 1e-3);
        cached_cov_local_[i] = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();

#if DO_VIZ_DEBUG
        if (i % 100 == 0)
        {
          mrpt::opengl::Scene scene;

          scene.insert(mrpt::opengl::CAxis::Create());
          scene.insert(mrpt::opengl::CGridPlaneXY::Create(-100, 100, -100, 100, 0, 5));

          {
            auto glPts = mrpt::opengl::CPointCloud::Create();

            glPts->loadFromPointsMap(this->pointcloud().get());
            glPts->setPointSize(2.5f);
            glPts->setColor_u8(0x00, 0x00, 0x00, 0x90);
            scene.insert(glPts);
          }

          {
            auto glPts = mrpt::opengl::CPointCloud::Create();

            for (size_t j = 0; j < k_indices.size(); j++)
            {
              glPts->insertPoint(xs[k_indices[j]], ys[k_indices[j]], zs[k_indices[j]]);
            }
            glPts->setPointSize(7.0f);
            glPts->setColor_u8(0xff, 0x00, 0x00, 0xff);
            scene.insert(glPts);
          }

          {
            auto glPts = mrpt::opengl::CPointCloud::Create();

            glPts->insertPoint(xs[i], ys[i], zs[i]);

            glPts->setPointSize(19.0f);
            glPts->setColor_u8(0x00, 0x00, 0xff, 0xff);
            scene.insert(glPts);
          }

          {
            auto glElli = mrpt::opengl::CEllipsoid3D::Create();
            glElli->setLocation(xs[i], ys[i], zs[i]);
            glElli->enableDrawSolid3D(false);
            glElli->setCovMatrix(cached_cov_local_[i] * 0.05);
            glElli->setColor_u8(0x00, 0xff, 0x00, 0xff);
            scene.insert(glElli);
          }

          {
            auto glEigs = mrpt::opengl::CSetOfLines::Create();
            glEigs->setColor_u8(0x00, 0x00, 0x00, 0xff);

            const auto c = mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]).cast<double>();

            glEigs->appendLine(c, c + eigData.eigVectors[0]);
            glEigs->appendLine(c, c + eigData.eigVectors[1] * 2);
            glEigs->appendLine(c, c + eigData.eigVectors[2] * 2);

            scene.insert(glEigs);
          }

          scene.saveToFile(mrpt::format("debug_kf_cov_%05i_%05zu.3Dscene", call_counter, i));
          std::ofstream f(mrpt::format("debug_kf_cov_%05i_%05zu.cov", call_counter, i));
          f << cached_cov_local_[i].inMatlabFormat();
        }
#endif
      }
#if defined(MOLA_METRIC_MAPS_USE_TBB)
  );
#endif

  cloud_density_ = std::sqrt(
      [&]()
      {
        float sum = 0;
        for (const auto d : sum_k_sq_distances)
        {
          sum += d;
        }
        return sum;
      }() /
      static_cast<float>(point_count));

  // done.

  if (ENV_DO_PROFILE_COV)
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start);
    std::cout << "[KeyframePointCloudMap] Compute covs: N=" << point_count << " in "
              << static_cast<double>(duration.count()) * 1e-3 << " ms d=" << *cloud_density_
              << "\n";
  }
}

void KeyframePointCloudMap::KeyFrame::updateCovariancesGlobal() const
{
  if (cached_cov_global_.size() == cached_cov_local_.size())
  {
    return;  // Already computed
  }

  ASSERT_EQUAL_(cached_cov_local_.size(), pointcloud_->size());

  cached_cov_global_.resize(cached_cov_local_.size());

  const Eigen::Matrix3f R = pose_.getRotationMatrix().cast_float().asEigen();

  for (size_t i = 0; i < cached_cov_local_.size(); i++)
  {
    cached_cov_global_[i] = R * cached_cov_local_[i].asEigen() * R.transpose();
  }
}

std::shared_ptr<mrpt::opengl::CPointCloudColoured> KeyframePointCloudMap::KeyFrame::getViz(
    const TRenderOptions& ro, const std::optional<mrpt::img::TColor>& overrideColor) const
{
  // The cache only holds the normal (non-overridden) visualization.
  if (!overrideColor && cached_viz_)
  {
    return cached_viz_;
  }

  const uint8_t alpha_u8 = mrpt::f2u8(ro.color.A);
  auto          obj      = mrpt::opengl::CPointCloudColoured::Create();

  obj->loadFromPointsMap(pointcloud().get());

  obj->setPose(pose());

  // Debug path: paint the whole key-frame a single, uniform color so different
  // key-frames (e.g. regrouped super-keyframes / clusters) are easy to tell
  // apart. Not cached, so toggling the env var takes effect on the next render.
  if (overrideColor)
  {
    const auto&  c = *overrideColor;
    const size_t n = obj->size();
    for (size_t i = 0; i < n; i++)
    {
      obj->setPointColor_u8_fast(i, c.R, c.G, c.B, alpha_u8);
    }
    return obj;
  }

  if (ro.color.A != 1.0f)
  {
    obj->setAllPointsAlpha(alpha_u8);
  }

  mrpt::obs::PointCloudRecoloringParameters pcdCol;
  pcdCol.colorMap        = ro.colormap;
  pcdCol.colorizeByField = ro.recolorByPointField;

  mrpt::obs::recolorize3Dpc(obj, pointcloud().get(), pcdCol);

  cached_viz_ = obj;
  return cached_viz_;
}

std::shared_ptr<mrpt::opengl::CSetOfObjects>
    KeyframePointCloudMap::KeyFrame::getCovarianceEllipsoidViz(const TRenderOptions& ro) const
{
  buildCache();
  updatePointsGlobal();
  updateCovariancesGlobal();

  if (cached_cov_global_.empty())
  {
    return {};
  }

  const thread_local auto ENV_KEYFRAMES_SHOW_COV_DECIMATION =
      mrpt::get_env<uint32_t>("MOLA_KEYFRAME_MAP_VIZ_SHOW_COV_DECIMATION", 0);

  auto cov_decimation = ENV_KEYFRAMES_SHOW_COV_DECIMATION > 0 ? ENV_KEYFRAMES_SHOW_COV_DECIMATION
                                                              : ro.show_cov_decimation;
  cov_decimation      = std::max<uint32_t>(1u, cov_decimation);
  if (cov_decimation >= cached_cov_global_.size())
  {
    cov_decimation = 1;
  }

  auto obj = mrpt::opengl::CSetOfObjects::Create();

  ASSERT_(pointcloud_global_);
  ASSERT_EQUAL_(cached_cov_global_.size(), pointcloud_global_->size());
  const auto& xs = pointcloud_global_->getPointsBufferRef_x();
  const auto& ys = pointcloud_global_->getPointsBufferRef_y();
  const auto& zs = pointcloud_global_->getPointsBufferRef_z();

  for (size_t i = 0; i < cached_cov_global_.size(); i++)
  {
    if ((i % cov_decimation) != 0)
    {
      continue;
    }

    const auto& cov = cached_cov_global_[i];

    auto elli = mrpt::opengl::CEllipsoid3D::Create();
    elli->setLocation(xs[i], ys[i], zs[i]);
    elli->enableDrawSolid3D(false);
    elli->setCovMatrix(cov * 0.05);
    elli->setColor(ro.cov_color);

    obj->insert(elli);
  }

  return obj;
}
