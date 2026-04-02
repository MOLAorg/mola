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
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/math/TOrientedBox.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/serialization/CArchive.h>  // serialization
#include <mrpt/system/string_utils.h>  // unitsFormat()
#include <mrpt/version.h>

#include <numeric>  // std::accumulate

#if defined(MOLA_METRIC_MAPS_USE_TBB)
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#endif

#include <type_traits>

// #define DO_PROFILE_COV 1
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

uint8_t KeyframePointCloudMap::serializeGetVersion() const { return 0; }
void    KeyframePointCloudMap::serializeTo(mrpt::serialization::CArchive& out) const
{
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
    }
    else
    {
      out.WriteAs<uint8_t>(0);  // no point cloud
    }
  }
}

void KeyframePointCloudMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  // clear contents
  this->clear();

  switch (version)
  {
    case 0:
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

        auto [it, isNew] = keyframes_.emplace(kf_id, creationOptions.k_correspondences_for_cov);
        KeyFrame& kf     = it->second;

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
        }
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  // cache reset:
  cached_.reset();
}

///  === KeyframePointCloudMap ===

KeyframePointCloudMap::~KeyframePointCloudMap() = default;

mrpt::math::TBoundingBoxf KeyframePointCloudMap::boundingBox() const
{
  if (cached_.boundingBox)
  {
    return *cached_.boundingBox;
  }

  // TODO(jlbc): To be refined with new mrpt implementation of Oriented Bounding Boxes
#if MRPT_VERSION >= 0x020e0d && 0
  mrpt::math::TOrientedBox ob;
#else
  // Pessimistic bounding box:
  cached_.boundingBox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
  for (const auto& [kf_id, kf] : keyframes_)
  {
    cached_.boundingBox = cached_.boundingBox->unionWith(kf.localBoundingBox().compose(kf.pose()));
  }
#endif

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

    // Additive metric: prevents the zero-distance degeneracy of multiplicative forms,
    // and gives a clean meters-equivalent score that is easy to reason about.
    const double m = dist_to_kf + rotW * angle_to_kf;

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

      for (const auto& c : candidates)
      {
        if (selectedIds.count(c.kfId) != 0)
        {
          continue;
        }
        if (c.dist > diverseDistLimit)
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

      if (bestCandidate != nullptr)
      {
        selectedIds.insert(bestCandidate->kfId);
        selectedAngles.push_back(bestCandidate->angle);
      }
    }
  }

  kfs_to_search_limited = selectedIds;

  // ---------------------------------------------------------------
  // 4) Rebuild merged submap if the selection changed
  // ---------------------------------------------------------------
  if (cached_.icp_search_kfs && *cached_.icp_search_kfs == kfs_to_search_limited)
  {
    return;  // Already up to date.
  }

  cached_.icp_search_kfs = kfs_to_search_limited;

  // NOTE: Do NOT unlock 'lck' here. The mutex must be held for the entire submap
  // rebuild below, because cached_.icp_search_submap and keyframes_ are both
  // shared mutable state that can be read concurrently by nn_* methods and
  // icp_get_prepared_as_global() itself.

  cached_.icp_search_submap.reset();
  cached_.icp_search_submap.emplace(creationOptions.k_correspondences_for_cov);

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

  cached_.icp_search_submap->buildCache();
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

  for (const auto& [srcKfId, srcKf] : sourceMapKF->keyframes_)
  {
    const auto& srcPc = srcKf.pointcloud();
    if (!srcPc)
    {
      continue;
    }
    auto [it, isNew] =
        keyframes_.emplace(nextFreeKeyFrameID(), creationOptions.k_correspondences_for_cov);
    auto& new_kf = it->second;

    // copy
    new_kf = srcKf;
    // and optionally, transform
    if (otherRelativePose)
    {
      new_kf.pose(*otherRelativePose + new_kf.pose());
    }
  }
}

void KeyframePointCloudMap::transform_map_left_multiply(const mrpt::poses::CPose3D& b)
{
  auto lck = mrpt::lockHelper(*state_mtx_);

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
  auto lck = mrpt::lockHelper(*state_mtx_);

  ASSERTMSG_(
      cached_.icp_search_submap,
      "Using this method requires calling icp_get_prepared_as_global() first");

  // Enforce local map to recompute its covariances to the new pose:
  const auto* localMapKF = dynamic_cast<const KeyframePointCloudMap*>(&localMap);
  ASSERTMSG_(
      localMapKF, "Implementation expects local map to be also of type KeyframePointCloudMap");

  ASSERT_EQUAL_(localMapKF->keyframes_.size(), 1U);
  auto&      localKf             = const_cast<KeyFrame&>(localMapKF->keyframes_.at(0));
  const auto originalLocalKfPose = localKf.pose();
  localKf.pose(localMapPose);

  const auto& localKfCov        = localKf.covariancesGlobal();
  const auto& localPointsTransf = localKf.pointcloud_global();
  const auto& localPoints       = localKf.pointcloud();

  const auto& globalKfCov  = cached_.icp_search_submap->covariancesGlobal();
  const auto& globalPoints = cached_.icp_search_submap->pointcloud_global();

  const auto localPointCount = localPointsTransf->size();

  const float max_sqr_dist = mrpt::square(max_search_distance);

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
  tbb::enumerable_thread_specific<mp2p_icp::MatchedPointWithCovList> tls;

  tbb::parallel_for(
      static_cast<size_t>(0), localPointCount,
      [&](size_t local_idx)
#else
  for (size_t local_idx = 0; local_idx < localPointCount; local_idx++)
#endif
      {
        float nn_dist_sqr = std::numeric_limits<float>::max();

        const auto nn_global_idx = globalPoints->kdTreeClosestPoint3D(
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
#endif

  // Recover original:
  localKf.pose(originalLocalKfPose);
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

void KeyframePointCloudMap::getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const
{
  MRPT_START
  if (!genericMapParams.enableSaveAs3DObject)
  {
    return;
  }
  auto lck = mrpt::lockHelper(*state_mtx_);

  // Create one visualization object per KF:
  for (const auto& [kf_id, kf] : keyframes_)
  {
    auto obj = kf.getViz(renderOptions);

    const static auto ENV_KEYFRAMES_SHOW_ACTIVE_FRAMES =
        mrpt::get_env<bool>("MOLA_KEYFRAME_MAP_VIZ_SHOW_ACTIVE_SUBMAP", false);

    if (ENV_KEYFRAMES_SHOW_ACTIVE_FRAMES)
    {
      float pointSize = renderOptions.point_size;
      if (cached_.icp_search_kfs && cached_.icp_search_kfs->count(kf_id) != 0)
      {
        pointSize *= 4;
      }
      obj->setPointSize(pointSize);
    }

    outObj.insert(obj);

    const static auto ENV_KEYFRAMES_AXES_LENGTH =
        mrpt::get_env<float>("MOLA_KEYFRAME_MAP_VIZ_OVERRIDE_AXES_LENGTH", .0f);
    const auto activeAxesLength =
        std::max(renderOptions.keyframes_axes_length, ENV_KEYFRAMES_AXES_LENGTH);

    if (activeAxesLength > 0)
    {
      auto glAxes = mrpt::opengl::stock_objects::CornerXYZSimple(activeAxesLength);
      glAxes->setPose(kf.pose());
      outObj.insert(glAxes);
    }
  }

  MRPT_END
}

bool KeyframePointCloudMap::isEmpty() const { return keyframes_.empty(); }

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
}

void KeyframePointCloudMap::TRenderOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const int8_t version = 3;
  out << version;
  out << point_size << color << int8_t(colormap) << recolorByPointField;  // v2
  out << max_points_per_kf << max_overall_points;  // v1
  out << keyframes_axes_length;  // v3
}

void KeyframePointCloudMap::TRenderOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version = 0;
  in >> version;
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
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
  MRPT_LOAD_CONFIG_VAR_CS(rotation_distance_weight, double);
  MRPT_LOAD_CONFIG_VAR_CS(num_diverse_keyframes, uint64_t);
  MRPT_LOAD_CONFIG_VAR_CS(use_view_direction_filter, bool);
  MRPT_LOAD_CONFIG_VAR_CS(max_view_angle_deg, double);
}

void KeyframePointCloudMap::TCreationOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [KeyframePointCloudMap::TCreationOptions] ------- \n\n";
  LOADABLEOPTS_DUMP_VAR(max_search_keyframes, int);
  LOADABLEOPTS_DUMP_VAR(k_correspondences_for_cov, int);
  LOADABLEOPTS_DUMP_VAR(rotation_distance_weight, double);
  LOADABLEOPTS_DUMP_VAR(num_diverse_keyframes, int);
  LOADABLEOPTS_DUMP_VAR(use_view_direction_filter, bool);
  LOADABLEOPTS_DUMP_VAR(max_view_angle_deg, double);
}

void KeyframePointCloudMap::TCreationOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  out.WriteAs<uint8_t>(2);  // version
  out << max_search_keyframes << k_correspondences_for_cov;
  out << rotation_distance_weight << num_diverse_keyframes;  // v1
  out << use_view_direction_filter << max_view_angle_deg;  // v2
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
  cached_.reset();
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

    // Add KF:
    auto [it, isNew] =
        keyframes_.emplace(nextFreeKeyFrameID(), creationOptions.k_correspondences_for_cov);
    auto& new_kf = it->second;

    new_kf.timestamp = obs.timestamp;
    new_kf.pose(pc_in_map);
    new_kf.pointcloud(obsPC->pointcloud);

    new_kf.buildCache();
    cached_.reset();

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
    auto* vx = pointcloud_global_->getPointsBufferRef_float_field("view_x");
    auto* vy = pointcloud_global_->getPointsBufferRef_float_field("view_y");
    auto* vz = pointcloud_global_->getPointsBufferRef_float_field("view_z");

    if (vx == nullptr || vy == nullptr || vz == nullptr)
    {
      // One or more view fields are missing in the destination map even
      // though the source had all three.
      // TODO: log a warning here once a logger is available.
    }
    else
    {
      const size_t n = pointcloud_global_->size();

      // TODO: Write an AVX2 version of this rotation loop.
      for (size_t i = 0; i < n; ++i)
      {
        const auto vg = pose_.rotateVector({(*vx)[i], (*vy)[i], (*vz)[i]}).cast<float>();
        (*vx)[i]      = vg.x;
        (*vy)[i]      = vg.y;
        (*vz)[i]      = vg.z;
      }
    }
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

#if DO_PROFILE_COV
  auto start = std::chrono::high_resolution_clock::now();
#endif
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

  const size_t K_CORRESPONDENCES = k_correspondences_for_cov_;
  const auto   normalization =
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

        pointcloud_->kdTreeNClosestPoint3DIdx(
            xs[i], ys[i], zs[i], K_CORRESPONDENCES, k_indices, k_sq_distances);

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
        const Eigen::Matrix3d cov = neighbors * neighbors.transpose() / K_CORRESPONDENCES;

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
#if DO_PROFILE_COV
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start);
  std::cout << "Compute covs: N=" << point_count << " in "
            << static_cast<double>(duration.count()) * 1e-3 << " ms d=" << *cloud_density_ << "\n";
#endif
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
    const TRenderOptions& ro) const
{
  if (cached_viz_)
  {
    return cached_viz_;
  }

  const uint8_t alpha_u8 = mrpt::f2u8(ro.color.A);
  auto          obj      = mrpt::opengl::CPointCloudColoured::Create();

  obj->loadFromPointsMap(pointcloud().get());

  obj->setPose(pose());

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
