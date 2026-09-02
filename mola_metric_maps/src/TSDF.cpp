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
 * @file   TSDF.cpp
 * @brief  Truncated signed distance field (TSDF) 3D map representation
 * @author Jose Luis Blanco Claraco
 * @date   Sep 02, 2026
 */

/* A voxel-hashed truncated signed distance field in the line of:
    Curless and Levoy, "A volumetric method for building complex models from
    range images", SIGGRAPH 1996;
    Niessner et al., "Real-time 3D reconstruction at scale using voxel
    hashing", ACM TOG 2013.
   The registration side follows the correspondence-free formulation used by
   distance-field odometry: the residual of a point is the field value at it.
*/

#include <mola_metric_maps/TSDF.h>
#include <mp2p_icp/estimate_points_eigen.h>
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace mola;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("mola::TSDF,TSDF", mola::TSDF)

TSDF::TMapDefinition::TMapDefinition() = default;

void TSDF::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& s, const std::string& sectionPrefix)
{
  using namespace std::string_literals;

  // [<sectionNamePrefix>+"_creationOpts"]
  const std::string sSectCreation = sectionPrefix + "_creationOpts"s;
  MRPT_LOAD_CONFIG_VAR(voxel_size, float, s, sSectCreation);

  ASSERT_(s.sectionExists(sectionPrefix + "_insertOpts"s));
  insertionOpts.loadFromConfigFile(s, sectionPrefix + "_insertOpts"s);

  if (s.sectionExists(sectionPrefix + "_renderOpts"s))
  {
    renderOpts.loadFromConfigFile(s, sectionPrefix + "_renderOpts"s);
  }
}

void TSDF::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  LOADABLEOPTS_DUMP_VAR(voxel_size, float);

  insertionOpts.dumpToTextStream(out);
  renderOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr TSDF::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const TSDF::TMapDefinition* def = dynamic_cast<const TSDF::TMapDefinition*>(&_def);
  ASSERT_(def);
  auto obj = TSDF::Create(def->voxel_size);

  obj->insertionOptions = def->insertionOpts;
  obj->renderOptions    = def->renderOpts;

  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(TSDF, CMetricMap, mola)

// =====================================
// Serialization
// =====================================

uint8_t TSDF::serializeGetVersion() const { return 1; }

void TSDF::serializeTo(mrpt::serialization::CArchive& out) const
{
  // params:
  out << voxel_size_;
  insertionOptions.writeToStream(out);
  renderOptions.writeToStream(out);

  // data:
  out.WriteAs<uint32_t>(voxels_.size());
  for (const auto& [idx, voxel] : voxels_)
  {
    out << idx.cx << idx.cy << idx.cz;
    out << voxel.dist << voxel.weight;
  }

  // Warm-up state (v1). A map saved mid-warm-up would otherwise come back
  // answering from a field that cannot yet answer.
  out << bootstrapping_ << bootstrapScans_ << fieldReadyFraction_;
  out << bootstrapPoints_.has_value();
  if (bootstrapPoints_)
  {
    out << *bootstrapPoints_;
  }
}

void TSDF::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    {
      float vxSize = 0;
      in >> vxSize;
      setVoxelProperties(vxSize);

      insertionOptions.readFromStream(in);
      renderOptions.readFromStream(in);

      const auto nVoxels = in.ReadAs<uint32_t>();
      voxels_.clear();
      voxels_.reserve(nVoxels);
      for (uint32_t i = 0; i < nVoxels; i++)
      {
        global_index3d_t idx;
        in >> idx.cx >> idx.cy >> idx.cz;

        VoxelData v;
        in >> v.dist >> v.weight;
        voxels_[idx] = v;
      }

      if (version >= 1)
      {
        in >> bootstrapping_ >> bootstrapScans_ >> fieldReadyFraction_;
        bool hasPoints = false;
        in >> hasPoints;
        if (hasPoints)
        {
          in >> bootstrapPoints_.emplace();
        }
        else
        {
          bootstrapPoints_.reset();
        }
      }
      else
      {
        // A map written before the warm-up existed is a finished map.
        endBootstrap();
      }

      invalidateCaches();
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

// =====================================
// Construction
// =====================================

TSDF::TSDF(float voxel_size) { setVoxelProperties(voxel_size); }

TSDF::~TSDF() = default;

void TSDF::setVoxelProperties(float voxel_size)
{
  ASSERT_GT_(voxel_size, .0f);

  voxel_size_     = voxel_size;
  voxel_size_inv_ = 1.0f / voxel_size_;

  creationOptions.voxel_size = voxel_size;

  // Clear contents, which are tied to the discretization:
  this->clear();
}

void TSDF::internal_clear()
{
  voxels_.clear();

  bootstrapping_      = true;
  bootstrapWarned_    = false;
  bootstrapScans_     = 0;
  fieldReadyFraction_ = 0;
  bootstrapPoints_.reset();
  bootstrapProbe_.clear();

  invalidateCaches();
}

void TSDF::invalidateCaches()
{
  cachedBoundingBox_.reset();
  cachedPoints_.reset();
}

// =====================================
// The field itself
// =====================================

void TSDF::insertPoint(const mrpt::math::TPoint3Df& pt, const mrpt::math::TPoint3Df& sensorPt)
{
  const float rayX  = pt.x - sensorPt.x;
  const float rayY  = pt.y - sensorPt.y;
  const float rayZ  = pt.z - sensorPt.z;
  const float range = std::sqrt(rayX * rayX + rayY * rayY + rayZ * rayZ);

  if (range < 1e-3f)
  {
    return;  // degenerate: no ray direction to define a sign with
  }

  // Unit ray direction, pointing away from the sensor:
  const float ux = rayX / range;
  const float uy = rayY / range;
  const float uz = rayZ / range;

  const float trunc    = truncation();
  const float tubeR    = static_cast<float>(insertionOptions.ray_tube_voxels) * voxel_size_;
  const float tubeRSqr = tubeR * tubeR;

  const float tubeSigma = static_cast<float>(insertionOptions.tube_sigma_voxels) * voxel_size_;
  const float tubeInvTwoSigmaSqr = tubeSigma > 0 ? 0.5f / (tubeSigma * tubeSigma) : .0f;

  float sampleWeight = 1.0f;
  if (insertionOptions.weight_by_range)
  {
    const float refR = static_cast<float>(insertionOptions.weight_range_ref);
    sampleWeight *= std::min(1.0f, (refR * refR) / (range * range));
  }
  if (insertionOptions.weight_by_incidence)
  {
    // The gradient already accumulated at the surface is the best available
    // estimate of the local normal at insertion time.
    const auto q = queryField(pt);
    if (q.valid)
    {
      const float gn = std::sqrt(
          q.gradient.x * q.gradient.x + q.gradient.y * q.gradient.y + q.gradient.z * q.gradient.z);
      if (gn > 1e-6f)
      {
        const float cosInc =
            std::abs((q.gradient.x * ux + q.gradient.y * uy + q.gradient.z * uz) / gn);
        sampleWeight *= std::max(0.05f, cosInc);
      }
    }
  }

  const float maxW = static_cast<float>(insertionOptions.max_weight);

  // The updated region is a cylinder of radius `tubeR` around the ray segment
  // [pt - trunc*u, pt + trunc*u]. Its axis-aligned box is much tighter than a
  // cube of side 2*trunc whenever the ray is close to an axis, which is the
  // common case for the ground returns that dominate a lidar scan.
  const float hx = trunc * std::abs(ux) + tubeR;
  const float hy = trunc * std::abs(uy) + tubeR;
  const float hz = trunc * std::abs(uz) + tubeR;

  const global_index3d_t i0 = coordToGlobalIdx({pt.x - hx, pt.y - hy, pt.z - hz});
  const global_index3d_t i1 = coordToGlobalIdx({pt.x + hx, pt.y + hy, pt.z + hz});

  for (int32_t cx = i0.cx; cx <= i1.cx; cx++)
  {
    for (int32_t cy = i0.cy; cy <= i1.cy; cy++)
    {
      for (int32_t cz = i0.cz; cz <= i1.cz; cz++)
      {
        const auto c = globalIdxToCenter({cx, cy, cz});

        // Vector from the voxel center to the measured point:
        const float vx = pt.x - c.x;
        const float vy = pt.y - c.y;
        const float vz = pt.z - c.z;

        // Signed distance along the ray. Positive means the voxel sits between
        // the sensor and the surface, i.e. on the free-space side.
        const float sd = vx * ux + vy * uy + vz * uz;
        if (std::abs(sd) > trunc)
        {
          continue;
        }

        // Perpendicular offset from the ray:
        const float px      = vx - sd * ux;
        const float py      = vy - sd * uy;
        const float pz      = vz - sd * uz;
        const float perpSqr = px * px + py * py + pz * pz;
        if (perpSqr > tubeRSqr)
        {
          continue;
        }

        float w = sampleWeight;
        if (tubeInvTwoSigmaSqr > 0)
        {
          w *= std::exp(-perpSqr * tubeInvTwoSigmaSqr);
          if (w < 1e-6f)
          {
            continue;
          }
        }

        auto& v = voxels_[{cx, cy, cz}];

        const float wSum = v.weight + w;
        v.dist           = (v.dist * v.weight + sd * w) / wSum;
        v.weight         = std::min(wSum, maxW);
      }
    }
  }

  invalidateCaches();
}

TSDF::FieldQuery TSDF::queryField(const mrpt::math::TPoint3Df& p) const
{
  FieldQuery ret;

  // Continuous voxel-center coordinates: the center of voxel i sits at
  // (i + 0.5) * voxel_size, so shifting by half a voxel puts the interpolation
  // lattice on integer coordinates.
  const float tx = p.x * voxel_size_inv_ - 0.5f;
  const float ty = p.y * voxel_size_inv_ - 0.5f;
  const float tz = p.z * voxel_size_inv_ - 0.5f;

  const float fx = std::floor(tx);
  const float fy = std::floor(ty);
  const float fz = std::floor(tz);

  const auto ix = static_cast<int32_t>(fx);
  const auto iy = static_cast<int32_t>(fy);
  const auto iz = static_cast<int32_t>(fz);

  const float ax = tx - fx;
  const float ay = ty - fy;
  const float az = tz - fz;

  const float minW = static_cast<float>(insertionOptions.min_weight_for_query);

  // Field values at the 8 surrounding voxel centers. A partially observed
  // neighborhood is rejected rather than extrapolated: a surface inferred from
  // one corner is not a measurement.
  float d[2][2][2];
  for (int dx = 0; dx < 2; dx++)
  {
    for (int dy = 0; dy < 2; dy++)
    {
      for (int dz = 0; dz < 2; dz++)
      {
        const VoxelData* v = voxelByGlobalIdxs({ix + dx, iy + dy, iz + dz});
        if (!v || v->weight < minW)
        {
          return ret;  // invalid
        }
        d[dx][dy][dz] = v->dist;
      }
    }
  }

  const float bx = 1.0f - ax;
  const float by = 1.0f - ay;
  const float bz = 1.0f - az;

  // Trilinear interpolation:
  const float c00 = d[0][0][0] * bx + d[1][0][0] * ax;
  const float c01 = d[0][0][1] * bx + d[1][0][1] * ax;
  const float c10 = d[0][1][0] * bx + d[1][1][0] * ax;
  const float c11 = d[0][1][1] * bx + d[1][1][1] * ax;

  const float c0 = c00 * by + c10 * ay;
  const float c1 = c01 * by + c11 * ay;

  ret.dist = c0 * bz + c1 * az;

  // Analytical gradient of the same trilinear interpolant, in meters per
  // meter. Using the interpolant's own gradient, rather than central
  // differences over neighboring cells, keeps the normal consistent with the
  // residual the solver is given.
  const float ddx = ((d[1][0][0] - d[0][0][0]) * by + (d[1][1][0] - d[0][1][0]) * ay) * bz +
                    ((d[1][0][1] - d[0][0][1]) * by + (d[1][1][1] - d[0][1][1]) * ay) * az;

  const float ddy = ((d[0][1][0] - d[0][0][0]) * bx + (d[1][1][0] - d[1][0][0]) * ax) * bz +
                    ((d[0][1][1] - d[0][0][1]) * bx + (d[1][1][1] - d[1][0][1]) * ax) * az;

  const float ddz = (c01 - c00) * by + (c11 - c10) * ay;

  ret.gradient = {ddx * voxel_size_inv_, ddy * voxel_size_inv_, ddz * voxel_size_inv_};
  ret.valid    = true;

  return ret;
}

mp2p_icp::NearestPlaneCapable::NearestPlaneResult TSDF::nn_search_pt2pl(
    const mrpt::math::TPoint3Df& query, const float max_search_distance) const
{
  NearestPlaneCapable::NearestPlaneResult ret;

  // While the field is still filling in, the same pairing comes from a plane
  // fitted to the accumulated points. A map built through the low-level
  // insertPoint() API has no warm-up cloud and goes straight to the field.
  if (bootstrapping_ && insertionOptions.bootstrap_with_points && bootstrapPoints_ &&
      !bootstrapPoints_->empty())
  {
    return bootstrapSearch(query, max_search_distance);
  }

  const auto q = queryField(query);
  if (!q.valid)
  {
    return ret;
  }

  const float dist = std::abs(q.dist);
  if (dist > max_search_distance)
  {
    return ret;
  }

  const float gn = std::sqrt(
      q.gradient.x * q.gradient.x + q.gradient.y * q.gradient.y + q.gradient.z * q.gradient.z);
  if (gn < 1e-3f)
  {
    // A flat gradient means the field carries no surface direction here, which
    // happens where measurements from opposite sides average out.
    return ret;
  }

  const mrpt::math::TVector3D normal = {q.gradient.x / gn, q.gradient.y / gn, q.gradient.z / gn};

  // The point's own projection onto the zero level set. The field is a
  // projective distance, so its magnitude is an over-estimate at grazing
  // incidence; dividing by the gradient norm converts it back to a metric
  // distance, which is what the point-to-plane residual expects.
  const float step = q.dist / gn;

  const mrpt::math::TPoint3D foot = {
      query.x - step * normal.x, query.y - step * normal.y, query.z - step * normal.z};

  auto& pa              = ret.pairing.emplace();
  pa.pt_local           = query;
  pa.pl_global.centroid = foot;
  pa.pl_global.plane    = mrpt::math::TPlane(foot, normal);

  ret.distance = std::abs(step);

  return ret;
}

// =====================================
// Warm-up
// =====================================

mp2p_icp::NearestPlaneCapable::NearestPlaneResult TSDF::bootstrapSearch(
    const mrpt::math::TPoint3Df& query, const float max_search_distance) const
{
  NearestPlaneCapable::NearestPlaneResult ret;

  const size_t knn = insertionOptions.bootstrap_knn;
  ASSERT_GE_(knn, 3U);

  if (!bootstrapPoints_ || bootstrapPoints_->size() < knn)
  {
    return ret;
  }

  // The neighbor search is deliberately not capped at `max_search_distance`.
  // That bound is on the distance to the *surface*, which is what the field
  // applies too, while the patch needed to fit a plane is necessarily wider
  // than it. What bounds the search instead is the map's own declared reach.
  std::vector<size_t> idxs;
  std::vector<float>  distsSqr;
  bootstrapPoints_->kdTreeNClosestPoint3DIdx(query.x, query.y, query.z, knn, idxs, distsSqr);

  if (idxs.size() < knn)
  {
    return ret;
  }

  const float reach = std::max(max_search_distance, truncation());
  if (distsSqr.front() > reach * reach)
  {
    return ret;
  }

  const auto& xs = bootstrapPoints_->getPointsBufferRef_x();
  const auto& ys = bootstrapPoints_->getPointsBufferRef_y();
  const auto& zs = bootstrapPoints_->getPointsBufferRef_z();

  // The principal components of the neighborhood, which is how every other map
  // class in this library fits a local plane. It also reports the degenerate
  // cases instead of throwing on them, and a neighborhood of collinear points
  // is not rare here: on the first scans it can easily be a single scan line.
  const mp2p_icp::PointCloudEigen pca =
      mp2p_icp::estimate_points_eigen(xs.data(), ys.data(), zs.data(), idxs);

  // Eigenvalues ascending. A line or a point has no second spread direction and
  // therefore no tangent plane; taking one anyway yields an arbitrary normal.
  if (pca.eigVals[1] < 1e-3 * pca.eigVals[2])
  {
    return ret;
  }

  // The smallest eigenvalue is the variance across the fitted plane, so its
  // square root is the neighborhood's RMS distance from that plane: a
  // neighborhood that is not locally planar has no plane worth pairing against.
  const auto maxDev = static_cast<double>(insertionOptions.bootstrap_max_plane_deviation);
  if (maxDev > 0 && std::sqrt(std::max(.0, pca.eigVals[0])) > maxDev)
  {
    return ret;
  }

  const mrpt::math::TVector3D normal   = pca.eigVectors[0];
  const mrpt::math::TPoint3D  centroid = pca.meanCov.mean.asTPoint();
  const mrpt::math::TPoint3D  q3d(query.x, query.y, query.z);

  // Signed distance along the plane normal. Its sign depends on the arbitrary
  // orientation the decomposition gave the normal, which the point-to-plane
  // residual is invariant to: flipping the normal flips the residual too, and
  // the product is unchanged.
  const double signedDist = normal.x * (q3d.x - centroid.x) +  //
                            normal.y * (q3d.y - centroid.y) +  //
                            normal.z * (q3d.z - centroid.z);

  if (std::abs(signedDist) > max_search_distance)
  {
    return ret;
  }

  const mrpt::math::TPoint3D foot = {
      q3d.x - signedDist * normal.x,  //
      q3d.y - signedDist * normal.y,  //
      q3d.z - signedDist * normal.z};

  auto& pa              = ret.pairing.emplace();
  pa.pt_local           = query;
  pa.pl_global.centroid = foot;
  pa.pl_global.plane    = mrpt::math::TPlane(foot, normal);

  ret.distance = static_cast<float>(std::abs(signedDist));

  return ret;
}

void TSDF::updateBootstrapState()
{
  if (!bootstrapping_)
  {
    return;
  }

  if (!insertionOptions.bootstrap_with_points)
  {
    endBootstrap();
    return;
  }

  bootstrapScans_++;

  size_t valid = 0;
  for (const auto& p : bootstrapProbe_)
  {
    if (queryField(p).valid)
    {
      valid++;
    }
  }
  fieldReadyFraction_ =
      bootstrapProbe_.empty() ? .0 : double(valid) / double(bootstrapProbe_.size());
  bootstrapProbe_.clear();

  const bool ready = bootstrapScans_ >= insertionOptions.bootstrap_min_scans &&
                     fieldReadyFraction_ >= insertionOptions.bootstrap_field_ready_fraction;

  if (ready)
  {
    endBootstrap();
    return;
  }

  if (bootstrapScans_ >= insertionOptions.bootstrap_max_scans)
  {
    if (!bootstrapWarned_)
    {
      bootstrapWarned_ = true;
      std::cerr << "[mola::TSDF] Warning: the field could only answer "
                << mrpt::format("%.1f%%", 100.0 * fieldReadyFraction_) << " of the last scan after "
                << bootstrapScans_
                << " scans, below the configured bootstrap_field_ready_fraction. Switching to the "
                   "field anyway. A coarser voxel_size, a wider ray_tube_voxels or a lower "
                   "min_weight_for_query would each let it fill in sooner.\n";
    }
    endBootstrap();
  }
}

void TSDF::endBootstrap()
{
  bootstrapping_ = false;
  bootstrapPoints_.reset();
  bootstrapProbe_.clear();
  bootstrapProbe_.shrink_to_fit();
}

// =====================================
// Insertion
// =====================================

void TSDF::pruneFarVoxels(const mrpt::math::TPoint3Df& sensorPt)
{
  const bool byExtent = insertionOptions.remove_voxels_farther_than > 0;
  const bool byCount =
      insertionOptions.max_voxels > 0 && voxels_.size() > insertionOptions.max_voxels;

  if (!byExtent && !byCount)
  {
    return;
  }

  const auto idxCurObs = coordToGlobalIdx(sensorPt);

  const auto chebyshev = [&idxCurObs](const global_index3d_t& i)
  {
    return mrpt::max3(
        std::abs(i.cx - idxCurObs.cx), std::abs(i.cy - idxCurObs.cy),
        std::abs(i.cz - idxCurObs.cz));
  };

  int distInGrid = std::numeric_limits<int>::max();
  if (byExtent)
  {
    distInGrid =
        static_cast<int>(std::ceil(insertionOptions.remove_voxels_farther_than * voxel_size_inv_));
  }

  if (byCount)
  {
    // Shrink the effective radius until the map fits under the ceiling. Taking
    // the n-th smallest distance keeps this a pure function of the contents and
    // the sensor pose, so it does not depend on insertion order.
    std::vector<int> dists;
    dists.reserve(voxels_.size());
    for (const auto& [idx, v] : voxels_)
    {
      dists.push_back(chebyshev(idx));
    }
    const size_t keep = insertionOptions.max_voxels;
    std::nth_element(dists.begin(), dists.begin() + keep, dists.end());
    distInGrid = std::min(distInGrid, dists[keep]);
  }

  for (auto it = voxels_.begin(); it != voxels_.end();)
  {
    if (chebyshev(it->first) > distInGrid)
    {
      it = voxels_.erase(it);
    }
    else
    {
      ++it;
    }
  }
  invalidateCaches();
}

void TSDF::internal_insertPointCloud3D(
    const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
    const std::size_t num_pts)
{
  const auto sensorPt = pc_in_map.translation().cast<float>();

  const bool warmingUp = bootstrapping_ && insertionOptions.bootstrap_with_points;
  if (warmingUp)
  {
    if (!bootstrapPoints_)
    {
      bootstrapPoints_.emplace();
      bootstrapPoints_->insertionOptions.minDistBetweenLaserPoints = .0f;
    }
    bootstrapPoints_->reserve(bootstrapPoints_->size() + num_pts);
  }

  // One point in this many is kept to probe the field's readiness, so the test
  // costs a fixed fraction of the insertion whatever the scan size.
  constexpr std::size_t PROBE_DECIMATION = 20;

  for (std::size_t i = 0; i < num_pts; i++)
  {
    const auto gPt = pc_in_map.composePoint(mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]));
    insertPoint(gPt, sensorPt);

    if (warmingUp)
    {
      bootstrapPoints_->insertPointFast(gPt.x, gPt.y, gPt.z);
      if (i % PROBE_DECIMATION == 0)
      {
        bootstrapProbe_.push_back(gPt);
      }
    }
  }

  if (warmingUp)
  {
    bootstrapPoints_->mark_as_modified();
  }

  // Called here, and not per observation, because this is the one path every
  // observation type funnels through.
  updateBootstrapState();
}

bool TSDF::internal_insertObservation(
    const mrpt::obs::CObservation& obs, const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  MRPT_START

  using namespace mrpt::obs;

  mrpt::poses::CPose3D robotPose3D;
  if (robotPose)
  {
    robotPose3D = (*robotPose);
  }

  pruneFarVoxels(robotPose3D.translation().cast<float>());

  if (IS_CLASS(obs, CObservationPointCloud))
  {
    const auto& o = dynamic_cast<const CObservationPointCloud&>(obs);
    if (!o.pointcloud)
    {
      return false;
    }

    const auto& xs = o.pointcloud->getPointsBufferRef_x();
    const auto& ys = o.pointcloud->getPointsBufferRef_y();
    const auto& zs = o.pointcloud->getPointsBufferRef_z();

    // The sensor pose on the robot is already applied by whoever built the
    // cloud in the observation, as elsewhere in this library.
    internal_insertPointCloud3D(robotPose3D, xs.data(), ys.data(), zs.data(), xs.size());
    return true;
  }

  if (IS_CLASS(obs, CObservation2DRangeScan))
  {
    const auto& o = dynamic_cast<const CObservation2DRangeScan&>(obs);

    const auto* scanPoints = o.buildAuxPointsMap<mrpt::maps::CPointsMap>();
    if (scanPoints->empty())
    {
      return false;
    }

    const auto& xs = scanPoints->getPointsBufferRef_x();
    const auto& ys = scanPoints->getPointsBufferRef_y();
    const auto& zs = scanPoints->getPointsBufferRef_z();

    internal_insertPointCloud3D(robotPose3D, xs.data(), ys.data(), zs.data(), xs.size());
    return true;
  }

  if (IS_CLASS(obs, CObservation3DRangeScan))
  {
    const auto& o = dynamic_cast<const CObservation3DRangeScan&>(obs);

    mrpt::obs::T3DPointsProjectionParams pp;
    pp.takeIntoAccountSensorPoseOnRobot = true;

    mrpt::maps::CSimplePointsMap pointMap;

    // The two sources do not deliver points in the same frame: the stored
    // points3D_* are in the sensor frame, while unprojectInto() with the
    // parameter above already brings them into the robot frame.
    mrpt::poses::CPose3D cloudPose = robotPose3D;

    if (o.hasPoints3D)
    {
      pointMap.insertionOptions.minDistBetweenLaserPoints = .0f;
      for (size_t i = 0; i < o.points3D_x.size(); i++)
      {
        pointMap.insertPointFast(o.points3D_x[i], o.points3D_y[i], o.points3D_z[i]);
      }
      cloudPose = robotPose3D + o.sensorPose;
    }
    else if (o.hasRangeImage)
    {
      const_cast<CObservation3DRangeScan&>(o).unprojectInto(pointMap, pp);
    }
    else
    {
      return false;
    }

    const auto& xs = pointMap.getPointsBufferRef_x();
    const auto& ys = pointMap.getPointsBufferRef_y();
    const auto& zs = pointMap.getPointsBufferRef_z();

    internal_insertPointCloud3D(cloudPose, xs.data(), ys.data(), zs.data(), xs.size());
    return true;
  }

  if (IS_CLASS(obs, CObservationVelodyneScan))
  {
    const auto& o = dynamic_cast<const CObservationVelodyneScan&>(obs);

    // Automatically generate the point cloud if it was not done before:
    const_cast<CObservationVelodyneScan&>(o).generatePointCloud();

    mrpt::maps::CSimplePointsMap pointMap;
    for (size_t i = 0; i < o.point_cloud.x.size(); i++)
    {
      pointMap.insertPointFast(o.point_cloud.x[i], o.point_cloud.y[i], o.point_cloud.z[i]);
    }

    const auto& xs = pointMap.getPointsBufferRef_x();
    const auto& ys = pointMap.getPointsBufferRef_y();
    const auto& zs = pointMap.getPointsBufferRef_z();

    internal_insertPointCloud3D(
        robotPose3D + o.sensorPose, xs.data(), ys.data(), zs.data(), xs.size());
    return true;
  }

  return false;

  MRPT_END
}

// =====================================
// CMetricMap interface
// =====================================

bool TSDF::isEmpty() const { return voxels_.empty(); }

std::string TSDF::asString() const
{
  std::string ret = mrpt::format(
      "TSDF, voxel_size=%.03f truncation=%.03f voxels=%u", voxel_size_, truncation(),
      static_cast<unsigned int>(voxels_.size()));

  if (bootstrapping_)
  {
    ret += mrpt::format(
        " [warming up: %u scans, field answers %.1f%% of the last one]", bootstrapScans_,
        100.0 * fieldReadyFraction_);
  }
  return ret;
}

void TSDF::visitAllVoxels(
    const std::function<void(const global_index3d_t&, const VoxelData&)>& f) const
{
  for (const auto& [idx, v] : voxels_)
  {
    f(idx, v);
  }
}

mrpt::math::TBoundingBoxf TSDF::boundingBox() const
{
  if (!cachedBoundingBox_)
  {
    if (voxels_.empty())
    {
      cachedBoundingBox_.emplace();
      cachedBoundingBox_->min = {0, 0, 0};
      cachedBoundingBox_->max = {0, 0, 0};
    }
    else
    {
      cachedBoundingBox_ = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
      const float h      = 0.5f * voxel_size_;
      for (const auto& [idx, v] : voxels_)
      {
        const auto c = globalIdxToCenter(idx);
        cachedBoundingBox_->updateWithPoint(mrpt::math::TPoint3Df(c.x - h, c.y - h, c.z - h));
        cachedBoundingBox_->updateWithPoint(mrpt::math::TPoint3Df(c.x + h, c.y + h, c.z + h));
      }
    }
  }
  return *cachedBoundingBox_;
}

const mrpt::maps::CSimplePointsMap* TSDF::getAsSimplePointsMap() const
{
  if (!cachedPoints_)
  {
    cachedPoints_ = mrpt::maps::CSimplePointsMap::Create();

    // Report the voxel centers whose field is close to the zero level set, as
    // a stand-in surface sampling for visualization and for the ROS bridge.
    const float minW = static_cast<float>(insertionOptions.min_weight_for_query);
    for (const auto& [idx, v] : voxels_)
    {
      if (v.weight < minW || std::abs(v.dist) > 0.5f * voxel_size_)
      {
        continue;
      }
      const auto c = globalIdxToCenter(idx);
      cachedPoints_->insertPointFast(c.x, c.y, c.z);
    }
    cachedPoints_->mark_as_modified();
  }
  return cachedPoints_.get();
}

void TSDF::getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const
{
  auto pts = mrpt::opengl::CPointCloud::Create();
  pts->setPointSize(renderOptions.point_size);
  pts->setColor(renderOptions.points_color);

  const auto* surface = getAsSimplePointsMap();
  for (size_t i = 0; i < surface->size(); i++)
  {
    float x = 0;
    float y = 0;
    float z = 0;
    surface->getPointFast(i, x, y, z);
    pts->insertPoint(x, y, z);
  }

  outObj.insert(pts);
}

void TSDF::saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const
{
  using namespace std::string_literals;

  const auto fil = filNamePrefix + ".txt"s;

  FILE* f = mrpt::system::os::fopen(fil.c_str(), "wt");
  if (!f)
  {
    return;
  }
  for (const auto& [idx, v] : voxels_)
  {
    const auto c = globalIdxToCenter(idx);
    mrpt::system::os::fprintf(f, "%f %f %f %f %f\n", c.x, c.y, c.z, v.dist, v.weight);
  }
  mrpt::system::os::fclose(f);
}

double TSDF::internal_computeObservationLikelihood(
    [[maybe_unused]] const mrpt::obs::CObservation& obs,
    [[maybe_unused]] const mrpt::poses::CPose3D&    takenFrom) const
{
  THROW_EXCEPTION("Observation likelihood not implemented for mola::TSDF");
}

bool TSDF::internal_canComputeObservationLikelihood(
    [[maybe_unused]] const mrpt::obs::CObservation& obs) const
{
  return false;
}

// =====================================
// Options
// =====================================

std::map<std::string, mrpt::config::CLoadableOptions*> TSDF::optionsByName()
{
  return {
      {"creationOptions", &creationOptions},
      {"insertionOptions", &insertionOptions},
      {"renderOptions", &renderOptions},
  };
}

void TSDF::TCreationOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(voxel_size, float, c, s);
}

void TSDF::TCreationOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [TSDF::TCreationOptions] ------- \n\n";
  LOADABLEOPTS_DUMP_VAR(voxel_size, float);
}

bool TSDF::trySetCreationOptions(
    const mrpt::config::CConfigFileBase& cfg, const std::string& section)
{
  TCreationOptions newOpts = creationOptions;
  newOpts.loadFromConfigFile(cfg, section);

  if (newOpts.voxel_size != creationOptions.voxel_size)
  {
    if (!isEmpty())
    {
      return false;  // would require discarding existing voxels
    }
    setVoxelProperties(newOpts.voxel_size);
  }
  return true;
}

void TSDF::TInsertionOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(truncation_distance, double, c, s);
  MRPT_LOAD_CONFIG_VAR(truncation_voxels, double, c, s);
  MRPT_LOAD_CONFIG_VAR(ray_tube_voxels, double, c, s);
  MRPT_LOAD_CONFIG_VAR(tube_sigma_voxels, double, c, s);
  MRPT_LOAD_CONFIG_VAR(max_weight, double, c, s);
  MRPT_LOAD_CONFIG_VAR(min_weight_for_query, double, c, s);
  MRPT_LOAD_CONFIG_VAR(remove_voxels_farther_than, double, c, s);
  max_voxels =
      static_cast<uint64_t>(c.read_double(s, "max_voxels", static_cast<double>(max_voxels)));
  MRPT_LOAD_CONFIG_VAR(weight_by_range, bool, c, s);
  MRPT_LOAD_CONFIG_VAR(weight_range_ref, double, c, s);
  MRPT_LOAD_CONFIG_VAR(weight_by_incidence, bool, c, s);

  MRPT_LOAD_CONFIG_VAR(bootstrap_with_points, bool, c, s);
  bootstrap_min_scans = static_cast<uint32_t>(
      c.read_int(s, "bootstrap_min_scans", static_cast<int>(bootstrap_min_scans)));
  bootstrap_max_scans = static_cast<uint32_t>(
      c.read_int(s, "bootstrap_max_scans", static_cast<int>(bootstrap_max_scans)));
  MRPT_LOAD_CONFIG_VAR(bootstrap_field_ready_fraction, double, c, s);
  bootstrap_knn =
      static_cast<uint32_t>(c.read_int(s, "bootstrap_knn", static_cast<int>(bootstrap_knn)));
  MRPT_LOAD_CONFIG_VAR(bootstrap_max_plane_deviation, double, c, s);
}

void TSDF::TInsertionOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [TSDF::TInsertionOptions] ------- \n\n";
  LOADABLEOPTS_DUMP_VAR(truncation_distance, double);
  LOADABLEOPTS_DUMP_VAR(truncation_voxels, double);
  LOADABLEOPTS_DUMP_VAR(ray_tube_voxels, double);
  LOADABLEOPTS_DUMP_VAR(tube_sigma_voxels, double);
  LOADABLEOPTS_DUMP_VAR(max_weight, double);
  LOADABLEOPTS_DUMP_VAR(min_weight_for_query, double);
  LOADABLEOPTS_DUMP_VAR(remove_voxels_farther_than, double);
  out << mrpt::format("max_voxels = %lu\n", static_cast<unsigned long>(max_voxels));
  LOADABLEOPTS_DUMP_VAR(weight_by_range, bool);
  LOADABLEOPTS_DUMP_VAR(weight_range_ref, double);
  LOADABLEOPTS_DUMP_VAR(weight_by_incidence, bool);
  LOADABLEOPTS_DUMP_VAR(bootstrap_with_points, bool);
  out << mrpt::format("bootstrap_min_scans = %u\n", bootstrap_min_scans);
  out << mrpt::format("bootstrap_max_scans = %u\n", bootstrap_max_scans);
  LOADABLEOPTS_DUMP_VAR(bootstrap_field_ready_fraction, double);
  out << mrpt::format("bootstrap_knn = %u\n", bootstrap_knn);
  LOADABLEOPTS_DUMP_VAR(bootstrap_max_plane_deviation, double);
}

void TSDF::TInsertionOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const int8_t version = 1;
  out << version;
  out << truncation_distance << truncation_voxels << ray_tube_voxels << tube_sigma_voxels
      << max_weight << min_weight_for_query << remove_voxels_farther_than << max_voxels
      << weight_by_range << weight_range_ref << weight_by_incidence;
  // v1:
  out << bootstrap_with_points << bootstrap_min_scans << bootstrap_max_scans
      << bootstrap_field_ready_fraction << bootstrap_knn << bootstrap_max_plane_deviation;
}

void TSDF::TInsertionOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version = 0;
  in >> version;
  switch (version)
  {
    case 0:
    case 1:
      in >> truncation_distance >> truncation_voxels >> ray_tube_voxels >> tube_sigma_voxels >>
          max_weight >> min_weight_for_query >> remove_voxels_farther_than >> max_voxels >>
          weight_by_range >> weight_range_ref >> weight_by_incidence;
      if (version >= 1)
      {
        in >> bootstrap_with_points >> bootstrap_min_scans >> bootstrap_max_scans >>
            bootstrap_field_ready_fraction >> bootstrap_knn >> bootstrap_max_plane_deviation;
      }
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void TSDF::TRenderOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(point_size, float, c, s);
  MRPT_LOAD_CONFIG_VAR(points_color.R, float, c, s);
  MRPT_LOAD_CONFIG_VAR(points_color.G, float, c, s);
  MRPT_LOAD_CONFIG_VAR(points_color.B, float, c, s);
}

void TSDF::TRenderOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [TSDF::TRenderOptions] ------- \n\n";
  LOADABLEOPTS_DUMP_VAR(point_size, float);
  LOADABLEOPTS_DUMP_VAR(points_color.R, float);
  LOADABLEOPTS_DUMP_VAR(points_color.G, float);
  LOADABLEOPTS_DUMP_VAR(points_color.B, float);
}

void TSDF::TRenderOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const int8_t version = 0;
  out << version;
  out << point_size << points_color.R << points_color.G << points_color.B;
}

void TSDF::TRenderOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version = 0;
  in >> version;
  switch (version)
  {
    case 0:
      in >> point_size >> points_color.R >> points_color.G >> points_color.B;
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}
