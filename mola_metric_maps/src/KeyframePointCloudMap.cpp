/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
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
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/serialization/CArchive.h>  // serialization
#include <mrpt/system/string_utils.h>  // unitsFormat()
#include <mrpt/version.h>

#if MRPT_VERSION >= 0x020e0d
#include <mrpt/math/TOrientedBox.h>
#endif

#include <Eigen/Dense>
#include <numeric>  // std::accumulate

#if defined(MOLA_METRIC_MAPS_USE_TBB)
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#endif

#include <type_traits>

// #define DO_PROFILE_COVS 1
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
      uint32_t n_kfs = in.ReadAs<uint32_t>();
      for (uint32_t i = 0; i < n_kfs; i++)
      {
        uint64_t kf_id;
        in >> kf_id;

        auto [it, isNew] = keyframes_.emplace(kf_id, creationOptions.k_correspondences_for_cov);
        KeyFrame& kf     = it->second;

        in >> kf.timestamp;
        mrpt::poses::CPose3D pose;
        in >> pose;
        kf.pose(pose);
        const auto has_pointcloud = in.ReadAs<uint8_t>();
        if (has_pointcloud)
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
  ASSERT_(cached_.icp_search_submap);
  return cached_.icp_search_submap->pointcloud()->nn_single_search(
      query, result, out_dist_sqr, resultIndexOrID);
}

bool KeyframePointCloudMap::nn_single_search(
    const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  ASSERT_(cached_.icp_search_submap);
  return cached_.icp_search_submap->pointcloud()->nn_single_search(
      query, result, out_dist_sqr, resultIndexOrID);
}

void KeyframePointCloudMap::nn_multiple_search(
    const mrpt::math::TPoint3Df& query, const size_t N, std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_multiple_search(
      query, N, results, out_dists_sqr, resultIndicesOrIDs);
}

void KeyframePointCloudMap::nn_multiple_search(
    const mrpt::math::TPoint2Df& query, const size_t N, std::vector<mrpt::math::TPoint2Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_multiple_search(
      query, N, results, out_dists_sqr, resultIndicesOrIDs);
}

void KeyframePointCloudMap::nn_radius_search(
    const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_radius_search(
      query, search_radius_sqr, results, out_dists_sqr, resultIndicesOrIDs, maxPoints);
}

void KeyframePointCloudMap::nn_radius_search(
    const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_radius_search(
      query, search_radius_sqr, results, out_dists_sqr, resultIndicesOrIDs, maxPoints);
}

void KeyframePointCloudMap::icp_get_prepared_as_global(
    const mrpt::poses::CPose3D&                                      icp_ref_point,
    [[maybe_unused]] const std::optional<mrpt::math::TBoundingBoxf>& local_map_roi) const
{
  auto lck = mrpt::lockHelper(*state_mtx_);

  std::set<KeyFrameID> kfs_to_search_limited;

  //  max_search_keyframes
  // First, build a list of candidate key-frames to search into:
  std::map<double, KeyFrameID> kfs_to_search;  // key: distance to KF center
  for (auto& [kf_id, kf] : keyframes_)
  {
    if (!kf.pointcloud())
    {
      continue;
    }

    // convert query to local coordinates of the keyframe:
    const auto query_local = icp_ref_point - kf.pose();

    // Heuristic mix of Euclidean and angular distances, to favor nearby frames, but only if their
    // orientation is similar:
    const double sigma_rot = mrpt::DEG2RAD(90.0);

    const auto dist_to_kf  = query_local.norm();
    const auto angle_to_kf = mrpt::poses::Lie::SO<3>::log(query_local.getRotationMatrix()).norm();

    const double metric = dist_to_kf * std::exp(angle_to_kf / sigma_rot);

    kfs_to_search[metric] = kf_id;
  }

  // TODO: Explore other criteria here so more distant frames are used too?

  for (const auto& [dist, kf_id] : kfs_to_search)
  {
    kfs_to_search_limited.insert(kf_id);
    if (kfs_to_search_limited.size() >= creationOptions.max_search_keyframes)
    {
      break;
    }
  }

  // For selected KFs, build the submap, if it's different from the previous one:

  if (cached_.icp_search_kfs && *cached_.icp_search_kfs == kfs_to_search_limited)
  {
    // We are already up to date.
    return;
  }

  cached_.icp_search_kfs = kfs_to_search_limited;
  lck.unlock();

  // Rebuild "cached merged KF":

  cached_.icp_search_submap.reset();
  cached_.icp_search_submap.emplace(creationOptions.k_correspondences_for_cov);

  for (const auto kf_id : kfs_to_search_limited)
  {
    const auto& kf = keyframes_.at(kf_id);

    if (!kf.pointcloud())
    {
      continue;  // Should never happen!
    }

    if (!cached_.icp_search_submap->pointcloud())
    {
      cached_.icp_search_submap->pointcloud(mrpt::maps::CSimplePointsMap::Create());
    }

    // Use cached global pointcloud inside KF:
    cached_.icp_search_submap->pointcloud()->insertAnotherMap(
        kf.pointcloud_global().get(), mrpt::poses::CPose3D::Identity());
  }

  // This builds the KD-tree and covariances
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
  for (auto& [id, kf] : keyframes_)
  {
    kf.pose(b + kf.pose());
  }
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

  tbb::enumerable_thread_specific<mp2p_icp::MatchedPointWithCovList> tls;

#if defined(MOLA_METRIC_MAPS_USE_TBB)
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

        if (nn_dist_sqr <= max_sqr_dist)
        {
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

#if 0
    float pointSize = ro.point_size;
    if (cached_.icp_search_kfs->count(kf_id))
    {
      pointSize *= 3;
    }
    obj->setPointSize(pointSize);
#endif

    outObj.insert(obj);

    if (renderOptions.keyframes_axes_length > 0)
    {
      auto glAxes =
          mrpt::opengl::stock_objects::CornerXYZSimple(renderOptions.keyframes_axes_length);
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
        const std::size_t n_points_to_take =
            static_cast<std::size_t>(final_ratio * static_cast<float>(kf_pts.size()));

        // go by steps to subsample:
        const float step = static_cast<float>(kf_pts.size()) / static_cast<float>(n_points_to_take);
        for (size_t i = 0; i < n_points_to_take; i++)
        {
          const size_t idx = static_cast<size_t>(static_cast<float>(i) * step);
          float        x, y, z;
          kf_pts.getPoint(idx, x, y, z);
          cached_.cachedPoints->insertPointFast(x, y, z);
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
  int8_t version;
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

void KeyframePointCloudMap::TLikelihoodOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  out.WriteAs<uint8_t>(0);
}

void KeyframePointCloudMap::TLikelihoodOptions::readFromStream(mrpt::serialization::CArchive& in)
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
  int8_t version;
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
}

void KeyframePointCloudMap::TCreationOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [KeyframePointCloudMap::TCreationOptions] ------- \n\n";
  LOADABLEOPTS_DUMP_VAR(max_search_keyframes, int);
  LOADABLEOPTS_DUMP_VAR(k_correspondences_for_cov, int);
}

void KeyframePointCloudMap::TCreationOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  out.WriteAs<uint8_t>(0);  // version
  out << max_search_keyframes << k_correspondences_for_cov;
}

void KeyframePointCloudMap::TCreationOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  const auto version = in.ReadAs<uint8_t>();
  switch (version)
  {
    case 0:
    {
      in >> max_search_keyframes >> k_correspondences_for_cov;
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
  if (auto obsPC = dynamic_cast<const mrpt::obs::CObservationPointCloud*>(&obs); obsPC)
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

double KeyframePointCloudMap::internal_computeObservationLikelihoodPointCloud3D(
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
  if (!pointcloud_global_)
  {
    pointcloud_global_ = mrpt::maps::CSimplePointsMap::Create();
  }
  pointcloud_global_->clear();
  pointcloud_global_->insertAnotherMap(pointcloud_.get(), pose());
}

void KeyframePointCloudMap::KeyFrame::computeCovariancesAndDensity() const
{
  ASSERT_(pointcloud_);
  const auto point_count = pointcloud_->size();

  if (cached_cov_local_.size() == point_count)
  {
    return;  // Already computed
  }

#if DO_PROFILE_COVS
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
#if DO_PROFILE_COVS
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
