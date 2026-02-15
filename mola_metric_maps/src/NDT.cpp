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
 * @file   NDT.cpp
 * @brief  NDT 3D map representation
 * @author Jose Luis Blanco Claraco
 * @date   Aug 22, 2024
 */

/* An implementation of:
    Scan registration for autonomous mining vehicles using 3D-NDT,
    Magnusson, Martin and Lilienthal, Achim and Duckett, Tom,
    Journal of Field Robotics, 2007.
*/

#include <mola_metric_maps/NDT.h>
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CHistogram.h>
#include <mrpt/math/distributions.h>  // confidenceIntervals()
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/serialization/CArchive.h>  // serialization
#include <mrpt/system/os.h>

#include <cmath>

// #define USE_DEBUG_PROFILER

#ifdef USE_DEBUG_PROFILER
#include <mrpt/system/CTimeLogger.h>
static mrpt::system::CTimeLogger profiler(true, "NDT");
#endif

constexpr size_t HARD_MAX_MATCHES = 3;

using namespace mola;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("mola::NDT,NDT", mola::NDT)

NDT::TMapDefinition::TMapDefinition() = default;
void NDT::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& s, const std::string& sectionPrefix)
{
  using namespace std::string_literals;

  // [<sectionNamePrefix>+"_creationOpts"]
  const std::string sSectCreation = sectionPrefix + "_creationOpts"s;
  MRPT_LOAD_CONFIG_VAR(voxel_size, float, s, sSectCreation);

  ASSERT_(s.sectionExists(sectionPrefix + "_insertOpts"s));
  insertionOpts.loadFromConfigFile(s, sectionPrefix + "_insertOpts"s);

  if (s.sectionExists(sectionPrefix + "_likelihoodOpts"s))
    likelihoodOpts.loadFromConfigFile(s, sectionPrefix + "_likelihoodOpts"s);

  if (s.sectionExists(sectionPrefix + "_renderOpts"s))
    renderOpts.loadFromConfigFile(s, sectionPrefix + "_renderOpts"s);
}

void NDT::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  LOADABLEOPTS_DUMP_VAR(voxel_size, float);

  insertionOpts.dumpToTextStream(out);
  likelihoodOpts.dumpToTextStream(out);
  renderOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr NDT::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const NDT::TMapDefinition* def = dynamic_cast<const NDT::TMapDefinition*>(&_def);
  ASSERT_(def);
  auto obj = NDT::Create(def->voxel_size);

  obj->insertionOptions  = def->insertionOpts;
  obj->likelihoodOptions = def->likelihoodOpts;
  obj->renderOptions     = def->renderOpts;

  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(NDT, CMetricMap, mola)

// =====================================
// Serialization
// =====================================

uint8_t NDT::serializeGetVersion() const { return 0; }
void    NDT::serializeTo(mrpt::serialization::CArchive& out) const
{
  // params:
  out << voxel_size_;
  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
  renderOptions.writeToStream(out);

  // data:
  out.WriteAs<uint32_t>(voxels_.size());
  for (const auto& [idx, voxel] : voxels_)
  {
    out << idx.cx << idx.cy << idx.cz;
    const auto&  pts = voxel.points();
    const size_t N   = pts.size();
    out.WriteAs<uint16_t>(N);
    out.WriteAs<bool>(voxel.was_seen_from().has_value());
    if (voxel.was_seen_from()) out << *voxel.was_seen_from();

    for (size_t j = 0; j < N; j++) out << pts[j];
  }
}
void NDT::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      // params:
      in >> voxel_size_;

      // clear contents and compute computed fields:
      this->setVoxelProperties(voxel_size_);

      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
      renderOptions.readFromStream(in);

      // data:
      const auto nGrids = in.ReadAs<uint32_t>();
      for (uint32_t i = 0; i < nGrids; i++)
      {
        global_index3d_t idx;
        in >> idx.cx >> idx.cy >> idx.cz;

        auto& grid = voxels_[idx];

        const auto            nPts = in.ReadAs<uint16_t>();
        mrpt::math::TPoint3Df sensorPose(0, 0, 0);
        if (in.ReadAs<bool>()) in >> sensorPose;

        for (size_t j = 0; j < nPts; j++)
        {
          mrpt::math::TPoint3Df pt;
          in >> pt;
          grid.insertPoint(pt, sensorPose);
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

// VoxelData

// Ctor:
NDT::NDT(float voxel_size) { setVoxelProperties(voxel_size); }

NDT::~NDT() = default;

void NDT::setVoxelProperties(float voxel_size)
{
  voxel_size_ = voxel_size;

  // calculated fields:
  voxel_size_inv_ = 1.0f / voxel_size_;
  voxel_size_sqr_ = voxel_size_ * voxel_size_;
  voxelDiagonal_  = mrpt::math::TPoint3Df(1.0f, 1.0f, 1.0f) * voxel_size_;

  // clear all:
  NDT::internal_clear();
}

bool NDT::ndt_is_plane(const mp2p_icp::PointCloudEigen& ndt) const
{
  // e0/e1 (and hence, e0/e2) must be < planeEigenThreshold:
  return (ndt.eigVals[0] < insertionOptions.max_eigen_ratio_for_planes * ndt.eigVals[1]);
}

std::string NDT::asString() const
{
  return mrpt::format(
      "NDT, resolution=%.03f bbox=%s", voxel_size_, boundingBox().asString().c_str());
}

void NDT::getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const
{
  MRPT_START
  if (!genericMapParams.enableSaveAs3DObject)
  {
    return;
  }

  // Calculate histograms / stats:
  auto bb = this->boundingBox();

  // handle planar maps (avoids error in histogram below):
  for (int i = 0; i < 3; i++)
  {
    if (bb.max[i] - bb.min[i] < 0.1f)
    {
      bb.max[i] = bb.min[i] + 0.1f;
    }
  }

  // Use a histogram to discard outliers from the colormap extremes:
  constexpr size_t nBins = 100;
  // for x,y,z
  std::array<mrpt::math::CHistogram, 3> hists = {
      mrpt::math::CHistogram(bb.min.x, bb.max.x, nBins),
      mrpt::math::CHistogram(bb.min.y, bb.max.y, nBins),
      mrpt::math::CHistogram(bb.min.z, bb.max.z, nBins)};

  size_t nPoints = 0;

  const auto lambdaVisitPointsForHist = [&hists, &nPoints](const mrpt::math::TPoint3Df& pt)
  {
    for (int i = 0; i < 3; i++)
    {
      hists[i].add(pt[i]);
    }
    nPoints++;
  };

  this->visitAllPoints(lambdaVisitPointsForHist);

  const auto recolor_idx_from_field = [](const std::string& field) -> int
  {
    if (field == "x")
    {
      return 0;
    }
    if (field == "y")
    {
      return 1;
    }
    // Default to "z" (2)
    return 2;
  };
  const int recolorIdx = recolor_idx_from_field(renderOptions.recolorByPointField);
  ASSERT_(recolorIdx >= 0 && recolorIdx < 3);

  float recolorMin = .0, recolorMax = 1.f;
  if (nPoints)
  {
    // Analyze the histograms and get confidence intervals:
    std::vector<double> coords;
    std::vector<double> hits;

    constexpr double confidenceInterval = 0.02;

    hists[recolorIdx].getHistogramNormalized(coords, hits);
    mrpt::math::confidenceIntervalsFromHistogram(
        coords, hits, recolorMin, recolorMax, confidenceInterval);
  }
  const float recolorK = recolorMax != recolorMin ? 1.0f / (recolorMax - recolorMin) : 1.0f;

  // points:
  if (renderOptions.points_visible)
  {
    auto obj = mrpt::opengl::CPointCloudColoured::Create();

    const auto lambdaVisitPoints = [&obj](const mrpt::math::TPoint3Df& pt) {
      obj->insertPoint({pt.x, pt.y, pt.z, 0, 0, 0});
    };
    this->visitAllPoints(lambdaVisitPoints);

    if (renderOptions.points_colormap == mrpt::img::cmNONE)
    {
      obj->setColor(renderOptions.points_color);
    }
    else
    {
      if (!obj->empty())
      {
        obj->recolorizeByCoordinate(
            recolorMin, recolorMax, recolorIdx, renderOptions.points_colormap);

        // Set alpha:
        if (renderOptions.points_color.A != 1.0f)
        {
          const uint8_t alpha = static_cast<uint8_t>(255 * renderOptions.points_color.A);
          for (size_t i = 0; i < nPoints; i++)
          {
            uint8_t r, g, b;
            obj->getPointColor_fast(i, r, g, b);
            obj->setPointColor_u8_fast(i, r, g, b, alpha);
          }
        }
      }
    }

    obj->setPointSize(renderOptions.point_size);
    outObj.insert(obj);
  }

  // planes:
  if (renderOptions.planes_visible)
  {
    auto obj = mrpt::opengl::CSetOfTriangles::Create();

    const auto lambdaVisitVoxel =
        [&obj, recolorK, recolorMin, recolorIdx, this](
            [[maybe_unused]] const global_index3d_t& idx, const VoxelData& v)
    {
      // get or evaluate its NDT:
      const auto& ndt = v.ndt();
      if (!ndt.has_value())
      {
        return;
      }

      // flat enough to be a plane?
      if (!ndt_is_plane(*ndt))
      {
        return;
      }

      const auto center = ndt->meanCov.mean.asTPoint().cast<float>();

      // eigenVector[0] is the plane normal.
      // [1] and [2] are parallel to the plane surface:
      const float s  = voxel_size_ * 0.5f;
      const auto  vx = ndt->eigVectors.at(1).cast<float>() * s;
      const auto  vy = ndt->eigVectors.at(2).cast<float>() * s;

      mrpt::opengl::TTriangle t;

      if (renderOptions.planes_colormap == mrpt::img::cmNONE)
      {
        t.setColor(renderOptions.planes_color);
      }
      else
      {
        t.setColor(mrpt::img::colormap(
            renderOptions.planes_colormap, recolorK * (center[recolorIdx] - recolorMin)));
      }

      t.vertices[0].xyzrgba.pt = center + vx + vy;
      t.vertices[1].xyzrgba.pt = center - vx - vy;
      t.vertices[2].xyzrgba.pt = center + vx - vy;
      t.computeNormals();
      obj->insertTriangle(t);

      t.vertices[0].xyzrgba.pt = center + vx + vy;
      t.vertices[1].xyzrgba.pt = center - vx + vy;
      t.vertices[2].xyzrgba.pt = center - vx - vy;
      t.computeNormals();
      obj->insertTriangle(t);
    };

    this->visitAllVoxels(lambdaVisitVoxel);
    outObj.insert(obj);
  }

  // normals:
  if (renderOptions.normals_visible)
  {
    auto obj = mrpt::opengl::CSetOfLines::Create();

    const auto lambdaVisitVoxel =
        [&obj, this]([[maybe_unused]] const global_index3d_t& idx, const VoxelData& v)
    {
      // get or evaluate its NDT:
      const auto& ndt = v.ndt();
      if (!ndt.has_value())
      {
        return;
      }

      // flat enough to be a plane?
      if (!ndt_is_plane(*ndt))
      {
        return;
      }

      const auto center = ndt->meanCov.mean.asTPoint().cast<float>();

      // eigenVector[0] is the plane normal.
      // [1] and [2] are parallel to the plane surface:
      const float s  = voxel_size_ * 0.2f;
      const auto  vz = ndt->eigVectors.at(0).cast<float>() * s;

      const auto p1 = center;
      const auto p2 = center + vz;

      obj->appendLine(p1, p2);
    };

    this->visitAllVoxels(lambdaVisitVoxel);
    outObj.insert(obj);
  }

  MRPT_END
}

void NDT::internal_clear() { voxels_.clear(); }

bool NDT::internal_insertObservation(
    const mrpt::obs::CObservation& obs, const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  MRPT_START

  using namespace mrpt::obs;

  mrpt::poses::CPose2D robotPose2D;
  mrpt::poses::CPose3D robotPose3D;

  if (robotPose)
  {
    robotPose2D = mrpt::poses::CPose2D(*robotPose);
    robotPose3D = (*robotPose);
  }
  else
  {
    // Default values are (0,0,0)
  }
  const auto sensorPt = robotPose3D.translation().cast<float>();

  if (insertionOptions.remove_voxels_farther_than > 0)
  {
    const int distInGrid =
        static_cast<int>(std::ceil(insertionOptions.remove_voxels_farther_than * voxel_size_inv_));

    const auto idxCurObs = coordToGlobalIdx(robotPose3D.translation().cast<float>());

    for (auto it = voxels_.begin(); it != voxels_.end();)
    {
      // manhattan distance:
      const int dist = mrpt::max3(
          std::abs(it->first.cx - idxCurObs.cx), std::abs(it->first.cy - idxCurObs.cy),
          std::abs(it->first.cz - idxCurObs.cz));

      if (dist > distInGrid)
      {
        it = voxels_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  if (IS_CLASS(obs, CObservation2DRangeScan))
  {
    /********************************************************************
                OBSERVATION TYPE: CObservation2DRangeScan
     ********************************************************************/
    const auto& o = dynamic_cast<const CObservation2DRangeScan&>(obs);

    // Build (if not done before) the points map representation of this
    // observation:
    const auto* scanPoints = o.buildAuxPointsMap<mrpt::maps::CPointsMap>();

    if (scanPoints->empty())
    {
      return 0;
    }

    const auto& xs = scanPoints->getPointsBufferRef_x();
    const auto& ys = scanPoints->getPointsBufferRef_y();
    const auto& zs = scanPoints->getPointsBufferRef_z();

    internal_insertPointCloud3D(robotPose3D, xs.data(), ys.data(), zs.data(), xs.size());

    return true;
  }

  if (IS_CLASS(obs, CObservation3DRangeScan))
  {
    /********************************************************************
                OBSERVATION TYPE: CObservation3DRangeScan
     ********************************************************************/
    const auto& o = dynamic_cast<const CObservation3DRangeScan&>(obs);

    mrpt::obs::T3DPointsProjectionParams pp;
    pp.takeIntoAccountSensorPoseOnRobot = true;

    // Empty point set, or load from XYZ in observation:
    if (o.hasPoints3D)
    {
      for (size_t i = 0; i < o.points3D_x.size(); i++)
        this->insertPoint(
            robotPose3D.composePoint({o.points3D_x[i], o.points3D_y[i], o.points3D_z[i]}),
            sensorPt);

      return true;
    }

    if (o.hasRangeImage)
    {
      mrpt::maps::CSimplePointsMap pointMap;
      const_cast<CObservation3DRangeScan&>(o).unprojectInto(pointMap, pp);

      const auto& xs = pointMap.getPointsBufferRef_x();
      const auto& ys = pointMap.getPointsBufferRef_y();
      const auto& zs = pointMap.getPointsBufferRef_z();

      internal_insertPointCloud3D(robotPose3D, xs.data(), ys.data(), zs.data(), xs.size());

      return true;
    }

    return false;
  }

  if (IS_CLASS(obs, CObservationVelodyneScan))
  {
    /********************************************************************
                OBSERVATION TYPE: CObservationVelodyneScan
     ********************************************************************/
    const auto& o = dynamic_cast<const CObservationVelodyneScan&>(obs);

    // Automatically generate pointcloud if needed:
    if (!o.point_cloud.size())
    {
      const_cast<CObservationVelodyneScan&>(o).generatePointCloud();
    }

    for (size_t i = 0; i < o.point_cloud.x.size(); i++)
    {
      insertPoint(
          robotPose3D.composePoint({o.point_cloud.x[i], o.point_cloud.y[i], o.point_cloud.z[i]}),
          sensorPt);
    }

    return true;
  }

  if (IS_CLASS(obs, CObservationPointCloud))
  {
    const auto& o = dynamic_cast<const CObservationPointCloud&>(obs);
    ASSERT_(o.pointcloud);

    const auto& xs = o.pointcloud->getPointsBufferRef_x();
    const auto& ys = o.pointcloud->getPointsBufferRef_y();
    const auto& zs = o.pointcloud->getPointsBufferRef_z();

    for (size_t i = 0; i < xs.size(); i++)
    {
      insertPoint(robotPose3D.composePoint({xs[i], ys[i], zs[i]}), sensorPt);
    }

    return true;
  }

  /********************************************************************
              OBSERVATION TYPE: Unknown
  ********************************************************************/
  return false;

  MRPT_END
}

double NDT::internal_computeObservationLikelihood(
    const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const
{
  using namespace mrpt::obs;
  using namespace mrpt::poses;
  using namespace mrpt::maps;

  if (isEmpty())
  {
    return 0;
  }

  // This function depends on the observation type:
  // -----------------------------------------------------
  if (IS_CLASS(obs, CObservation2DRangeScan))
  {
    // Observation is a laser range scan:
    // -------------------------------------------
    const auto& o = static_cast<const CObservation2DRangeScan&>(obs);

    // Build (if not done before) the points map representation of this
    // observation:
    const auto* scanPoints = o.buildAuxPointsMap<CPointsMap>();

    const size_t N = scanPoints->size();
    if (!N)
    {
      return 0;
    }

    const auto& xs = scanPoints->getPointsBufferRef_x();
    const auto& ys = scanPoints->getPointsBufferRef_y();
    const auto& zs = scanPoints->getPointsBufferRef_z();

    return internal_computeObservationLikelihoodPointCloud3D(
        takenFrom, xs.data(), ys.data(), zs.data(), N);
  }

  if (IS_CLASS(obs, CObservationVelodyneScan))
  {
    const auto& o = dynamic_cast<const CObservationVelodyneScan&>(obs);

    // Automatically generate pointcloud if needed:
    if (!o.point_cloud.size()) const_cast<CObservationVelodyneScan&>(o).generatePointCloud();

    const size_t N = o.point_cloud.size();
    if (!N)
    {
      return 0;
    }

    const CPose3D sensorAbsPose = takenFrom + o.sensorPose;

    const auto& xs = o.point_cloud.x;
    const auto& ys = o.point_cloud.y;
    const auto& zs = o.point_cloud.z;

    return internal_computeObservationLikelihoodPointCloud3D(
        sensorAbsPose, xs.data(), ys.data(), zs.data(), N);
  }

  if (IS_CLASS(obs, CObservationPointCloud))
  {
    const auto& o = dynamic_cast<const CObservationPointCloud&>(obs);

    const size_t N = o.pointcloud->size();
    if (!N)
    {
      return 0;
    }

    const CPose3D sensorAbsPose = takenFrom + o.sensorPose;

    auto xs = o.pointcloud->getPointsBufferRef_x();
    auto ys = o.pointcloud->getPointsBufferRef_y();
    auto zs = o.pointcloud->getPointsBufferRef_z();

    return internal_computeObservationLikelihoodPointCloud3D(
        sensorAbsPose, xs.data(), ys.data(), zs.data(), N);
  }

  return .0;
}

double NDT::internal_computeObservationLikelihoodPointCloud3D(
    [[maybe_unused]] const mrpt::poses::CPose3D& pc_in_map,
    [[maybe_unused]] const float*                xs,  //
    [[maybe_unused]] const float*                ys,  //
    [[maybe_unused]] const float*                zs,  //
    [[maybe_unused]] const std::size_t           num_pts) const
{
  MRPT_TRY_START

  ASSERT_GT_(likelihoodOptions.sigma_dist, .0);

  THROW_EXCEPTION("to-do");
#if 0
    mrpt::math::TPoint3Df closest;
    float                 closest_err;
    uint64_t              closest_id;
    const float max_sqr_err = mrpt::square(likelihoodOptions.max_corr_distance);
    double      sumSqrDist  = .0;

    std::size_t nPtsForAverage = 0;
    for (std::size_t i = 0; i < num_pts;
         i += likelihoodOptions.decimation, nPtsForAverage++)
    {
        // Transform the point from the scan reference to its global 3D
        // position:
        const auto gPt = pc_in_map.composePoint({xs[i], ys[i], zs[i]});

        const bool found =
            nn_single_search(gPt, closest, closest_err, closest_id);
        if (!found)
        continue;

        // Put a limit:
        mrpt::keep_min(closest_err, max_sqr_err);

        sumSqrDist += static_cast<double>(closest_err);
    }
    if (nPtsForAverage) sumSqrDist /= nPtsForAverage;

    // Log-likelihood:
    return -sumSqrDist / likelihoodOptions.sigma_dist;
#endif

  MRPT_TRY_END
}

bool NDT::internal_canComputeObservationLikelihood(const mrpt::obs::CObservation& obs) const
{
  using namespace mrpt::obs;

  return IS_CLASS(obs, CObservation2DRangeScan) || IS_CLASS(obs, CObservationVelodyneScan) ||
         IS_CLASS(obs, CObservationPointCloud);
}

bool NDT::isEmpty() const
{
  // empty if no voxels exist:
  return voxels_.empty();
}

void NDT::saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const
{
  using namespace std::string_literals;

  const auto fil = filNamePrefix + ".txt"s;
  saveToTextFile(fil);
}

bool NDT::saveToTextFile(const std::string& file) const
{
  FILE* f = mrpt::system::os::fopen(file.c_str(), "wt");
  if (!f) return false;

  const auto lambdaVisitPoints = [f](const mrpt::math::TPoint3Df& pt)
  { mrpt::system::os::fprintf(f, "%f %f %f\n", pt.x, pt.y, pt.z); };

  this->visitAllPoints(lambdaVisitPoints);

  mrpt::system::os::fclose(f);
  return true;
}

void NDT::insertPoint(const mrpt::math::TPoint3Df& pt, const mrpt::math::TPoint3Df& sensorPose)
{
  auto& v = *voxelByCoords(pt, true /*create if new*/);

  const auto nPreviousPoints = v.points().size();

  if (insertionOptions.max_points_per_voxel != 0 &&
      nPreviousPoints >= insertionOptions.max_points_per_voxel)
    return;  // skip, voxel is full

  if (insertionOptions.min_distance_between_points > 0 && nPreviousPoints != 0)
  {
    // Look for the closest existing point in the voxel:
    std::optional<float> curClosestDistSqr;

    const auto& pts = v.points();
    for (size_t i = 0; i < pts.size(); i++)
    {
      const float distSqr = pts[i].sqrDistanceTo(pt);
      if (!curClosestDistSqr.has_value() || distSqr < curClosestDistSqr.value())
        curClosestDistSqr = distSqr;
    }
    const float minDistSqr = mrpt::square(insertionOptions.min_distance_between_points);

    // Skip if the point is too close to existing ones:
    if (curClosestDistSqr.value() < minDistSqr)
    {
      return;
    }
  }

  v.insertPoint(pt, sensorPose);

  // Also, update bbox:
  if (!cached_.boundingBox_.has_value())
    cached_.boundingBox_.emplace(pt, pt);
  else
    cached_.boundingBox_->updateWithPoint(pt);
}

mrpt::math::TBoundingBoxf NDT::boundingBox() const
{
  if (!cached_.boundingBox_)
  {
    cached_.boundingBox_.emplace();
    if (this->isEmpty())
    {
      cached_.boundingBox_->min = {0, 0, 0};
      cached_.boundingBox_->max = {0, 0, 0};
    }
    else
    {
      cached_.boundingBox_ = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

      auto lambdaForEachVoxel = [this](const global_index3d_t& idxs, const VoxelData&)
      {
        const mrpt::math::TPoint3Df voxelCorner = globalIdxToCoord(idxs);

        cached_.boundingBox_->updateWithPoint(voxelCorner);
        cached_.boundingBox_->updateWithPoint(voxelCorner + voxelDiagonal_);
      };

      this->visitAllVoxels(lambdaForEachVoxel);
    }
  }

  return cached_.boundingBox_.value();
}

void NDT::visitAllPoints(const std::function<void(const mrpt::math::TPoint3Df&)>& f) const
{
  for (const auto& [idx, v] : voxels_)
  {
    const auto& pts = v.points();
    for (size_t i = 0; i < pts.size(); i++) f(pts[i]);
  }
}

void NDT::visitAllVoxels(
    const std::function<void(const global_index3d_t&, const VoxelData&)>& f) const
{
  for (const auto& [idx, v] : voxels_) f(idx, v);
}

// ========== Option structures ==========
void NDT::TInsertionOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const int8_t version = 0;
  out << version;
  out << max_points_per_voxel << remove_voxels_farther_than << min_distance_between_points
      << max_eigen_ratio_for_planes;
}

void NDT::TInsertionOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version;
  in >> version;
  switch (version)
  {
    case 0:
    {
      in >> max_points_per_voxel >> remove_voxels_farther_than >> min_distance_between_points >>
          max_eigen_ratio_for_planes;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void NDT::TLikelihoodOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const int8_t version = 0;
  out << version;
  out << sigma_dist << max_corr_distance << decimation;
}

void NDT::TLikelihoodOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version;
  in >> version;
  switch (version)
  {
    case 0:
    {
      in >> sigma_dist >> max_corr_distance >> decimation;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void NDT::TRenderOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const int8_t version = 2;
  out << version;
  out << points_visible << point_size << points_color;
  out << planes_visible << planes_color;
  out << normals_visible << normals_color;
  out << static_cast<int8_t>(points_colormap) << static_cast<int8_t>(planes_colormap)
      << recolorByPointField;  // v1, v2
}

void NDT::TRenderOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version;
  in >> version;
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    {
      in >> points_visible >> point_size >> points_color;
      in >> planes_visible >> planes_color;
      in >> normals_visible >> normals_color;
      if (version >= 1)
      {
        in.ReadAsAndCastTo<int8_t>(this->points_colormap);
        in.ReadAsAndCastTo<int8_t>(this->planes_colormap);

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
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void NDT::TInsertionOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [NDT::TInsertionOptions] ------- "
         "\n\n";

  LOADABLEOPTS_DUMP_VAR(max_points_per_voxel, int);
  LOADABLEOPTS_DUMP_VAR(remove_voxels_farther_than, double);
  LOADABLEOPTS_DUMP_VAR(min_distance_between_points, float)
  LOADABLEOPTS_DUMP_VAR(max_eigen_ratio_for_planes, double);
}

void NDT::TLikelihoodOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [NDT::TLikelihoodOptions] ------- "
         "\n\n";

  LOADABLEOPTS_DUMP_VAR(sigma_dist, double);
  LOADABLEOPTS_DUMP_VAR(max_corr_distance, double);
  LOADABLEOPTS_DUMP_VAR(decimation, int);
}

void NDT::TRenderOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n------ [NDT::TRenderOptions] ------- \n\n";

  LOADABLEOPTS_DUMP_VAR(points_visible, bool);
  LOADABLEOPTS_DUMP_VAR(point_size, float);
  LOADABLEOPTS_DUMP_VAR(points_color.R, float);
  LOADABLEOPTS_DUMP_VAR(points_color.G, float);
  LOADABLEOPTS_DUMP_VAR(points_color.B, float);

  LOADABLEOPTS_DUMP_VAR(planes_visible, bool);
  LOADABLEOPTS_DUMP_VAR(planes_color.R, float);
  LOADABLEOPTS_DUMP_VAR(planes_color.G, float);
  LOADABLEOPTS_DUMP_VAR(planes_color.B, float);

  LOADABLEOPTS_DUMP_VAR(normals_visible, bool);
  LOADABLEOPTS_DUMP_VAR(normals_color.R, float);
  LOADABLEOPTS_DUMP_VAR(normals_color.G, float);
  LOADABLEOPTS_DUMP_VAR(normals_color.B, float);

  LOADABLEOPTS_DUMP_VAR(points_colormap, int);
  LOADABLEOPTS_DUMP_VAR(planes_colormap, int);
  using std::string;
  LOADABLEOPTS_DUMP_VAR(recolorByPointField, string);
}

void NDT::TInsertionOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(max_points_per_voxel, int, c, s);
  MRPT_LOAD_CONFIG_VAR(remove_voxels_farther_than, double, c, s);
  MRPT_LOAD_CONFIG_VAR(min_distance_between_points, float, c, s);
  MRPT_LOAD_CONFIG_VAR(max_eigen_ratio_for_planes, double, c, s);
}

void NDT::TLikelihoodOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(sigma_dist, double, c, s);
  MRPT_LOAD_CONFIG_VAR(max_corr_distance, double, c, s);
  MRPT_LOAD_CONFIG_VAR(decimation, int, c, s);
}

void NDT::TRenderOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(points_visible, bool, c, s);
  MRPT_LOAD_CONFIG_VAR(point_size, float, c, s);
  MRPT_LOAD_CONFIG_VAR(points_color.R, float, c, s);
  MRPT_LOAD_CONFIG_VAR(points_color.G, float, c, s);
  MRPT_LOAD_CONFIG_VAR(points_color.B, float, c, s);

  MRPT_LOAD_CONFIG_VAR(planes_visible, bool, c, s);
  MRPT_LOAD_CONFIG_VAR(planes_color.R, float, c, s);
  MRPT_LOAD_CONFIG_VAR(planes_color.G, float, c, s);
  MRPT_LOAD_CONFIG_VAR(planes_color.B, float, c, s);

  MRPT_LOAD_CONFIG_VAR(normals_visible, bool, c, s);
  MRPT_LOAD_CONFIG_VAR(normals_color.R, float, c, s);
  MRPT_LOAD_CONFIG_VAR(normals_color.G, float, c, s);
  MRPT_LOAD_CONFIG_VAR(normals_color.B, float, c, s);

  points_colormap = c.read_enum(s, "points_colormap", this->points_colormap);
  planes_colormap = c.read_enum(s, "planes_colormap", this->planes_colormap);
  MRPT_LOAD_CONFIG_VAR(recolorByPointField, string, c, s);
}

void NDT::internal_insertPointCloud3D(
    const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
    const std::size_t num_pts)
{
  MRPT_TRY_START

  voxels_.reserve(voxels_.size() + num_pts);

  const auto sensorPose = pc_in_map.translation().cast<float>();

  for (std::size_t i = 0; i < num_pts; i++)
  {
    // Transform the point from the scan reference to its global 3D
    // position:
    const auto gPt = pc_in_map.composePoint({xs[i], ys[i], zs[i]});
    insertPoint(gPt, sensorPose);
  }

  MRPT_TRY_END
}

const mrpt::maps::CSimplePointsMap* NDT::getAsSimplePointsMap() const
{
  if (!cachedPoints_)
  {
    cachedPoints_ = mrpt::maps::CSimplePointsMap::Create();
  }

  cachedPoints_->clear();

  this->visitAllPoints([this](const mrpt::math::TPoint3Df& p)
                       { cachedPoints_->insertPointFast(p.x, p.y, p.z); });

  return cachedPoints_.get();
}

const std::optional<mp2p_icp::PointCloudEigen>& NDT::VoxelData::ndt() const
{
  if (has_ndt() || size() < 4)
  {
    return ndt_;
  }

  ndt_.emplace();
  *ndt_ = mp2p_icp::estimate_points_eigen(
      points_.xs.data(), points_.ys.data(), points_.zs.data(), std::nullopt, size());

  // Fix z normal such that it points towards the observer (outwards):
  ASSERT_(was_seen_from_);
  auto&       ev = ndt_->eigVectors;
  const float r =
      mrpt::math::dotProduct<3, float>(ev.at(0), *was_seen_from_ - ndt_->meanCov.mean.asTPoint());

  // +z must point outwards:
  if (r < 0)
  {
    ev.at(0) = -ev.at(0);
  }

  // Ensure CCW axes:
  mrpt::math::crossProduct3D(ev[0], ev[1], ev[2]);

  return ndt_;
}

bool NDT::nn_has_indices_or_ids() const
{  // false: IDs, not contiguous indices
  return false;
}

size_t NDT::nn_index_count() const
{  // Not used.
  return 0;
}

bool NDT::nn_single_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df& query,
    [[maybe_unused]] mrpt::math::TPoint2Df& result, [[maybe_unused]] float& out_dist_sqr,
    [[maybe_unused]] uint64_t& resultIndexOrID) const
{
  THROW_EXCEPTION("Cannot run a 2D search on a NDT");
}
void NDT::nn_multiple_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df& query, [[maybe_unused]] const size_t N,
    [[maybe_unused]] std::vector<mrpt::math::TPoint2Df>& results,
    [[maybe_unused]] std::vector<float>&                 out_dists_sqr,
    [[maybe_unused]] std::vector<uint64_t>&              resultIndicesOrIDs) const
{
  THROW_EXCEPTION("Cannot run a 2D search on a NDT");
}
void NDT::nn_radius_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df&        query,
    [[maybe_unused]] const float                         search_radius_sqr,
    [[maybe_unused]] std::vector<mrpt::math::TPoint2Df>& results,
    [[maybe_unused]] std::vector<float>&                 out_dists_sqr,
    [[maybe_unused]] std::vector<uint64_t>&              resultIndicesOrIDs,
    [[maybe_unused]] size_t                              maxPoints) const
{
  THROW_EXCEPTION("Cannot run a 2D search on a NDT");
}

mp2p_icp::NearestPlaneCapable::NearestPlaneResult NDT::nn_search_pt2pl(
    const mrpt::math::TPoint3Df& query, const float max_search_distance) const
{
  NearestPlaneCapable::NearestPlaneResult ret;

  const int nn_search_margin = static_cast<int>(std::ceil(max_search_distance * voxel_size_inv_));

  const global_index3d_t idxs0 =
      coordToGlobalIdx(query) -
      global_index3d_t(nn_search_margin, nn_search_margin, nn_search_margin);
  const global_index3d_t idxs1 =
      coordToGlobalIdx(query) +
      global_index3d_t(nn_search_margin, nn_search_margin, nn_search_margin);

  auto lambdaCheckCell = [&](const global_index3d_t& p)
  {
    auto* v = voxelByGlobalIdxs(p);
    if (!v)
    {
      return;
    }
    const auto& ndt = v->ndt();
    if (!ndt)
    {
      return;
    }
    if (!ndt_is_plane(*ndt))
    {
      return;
    }

    const auto&                normal        = ndt->eigVectors[0];
    const mrpt::math::TPoint3D planeCentroid = ndt->meanCov.mean.asTPoint();
    const auto                 thePlane      = mrpt::math::TPlane(planeCentroid, normal);
    const double               ptPlaneDist   = std::abs(thePlane.distance(query));

    // Better than the current one? replace:
    if (!ret.pairing || ptPlaneDist < ret.distance)
    {
      auto& pa              = ret.pairing.emplace();
      pa.pt_local           = query;
      pa.pl_global.centroid = planeCentroid;
      pa.pl_global.plane    = thePlane;

      ret.distance = static_cast<float>(ptPlaneDist);
    }
  };

  for (int32_t cx = idxs0.cx; cx <= idxs1.cx; cx++)
  {
    for (int32_t cy = idxs0.cy; cy <= idxs1.cy; cy++)
    {
      for (int32_t cz = idxs0.cz; cz <= idxs1.cz; cz++)
      {
        lambdaCheckCell({cx, cy, cz});
      }
    }
  }

  return ret;
}

bool NDT::nn_single_search(
    const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  std::vector<mrpt::math::TPoint3Df> r;
  std::vector<float>                 dist_sqr;
  std::vector<uint64_t>              resultIndices;
  nn_multiple_search_impl<1>(query, 1, r, dist_sqr, resultIndices);
  if (r.empty())
  {
    return false;  // none found
  }
  result          = r[0];
  out_dist_sqr    = dist_sqr[0];
  resultIndexOrID = resultIndices[0];
  return true;
}

void NDT::nn_multiple_search(
    const mrpt::math::TPoint3Df& query, const size_t N, std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  nn_multiple_search_impl<HARD_MAX_MATCHES>(query, N, results, out_dists_sqr, resultIndicesOrIDs);
}

template <size_t MAX_KNN>
void NDT::nn_multiple_search_impl(
    const mrpt::math::TPoint3Df& query, const size_t N, std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  results.clear();
  out_dists_sqr.clear();
  resultIndicesOrIDs.clear();

  ASSERT_(N > 0);

  const global_index3d_t idxs0 = coordToGlobalIdx(query) - global_index3d_t(1, 1, 1);
  const global_index3d_t idxs1 = coordToGlobalIdx(query) + global_index3d_t(1, 1, 1);

  // Data structures to avoid ANY heap memory allocation and keep working
  // on the stack at all time:
  struct Match
  {
    mrpt::math::TPoint3Df globalPt;
    float                 sqrDist = 0;
    uint64_t              id      = 0;
  };
  std::array<Match, MAX_KNN> matches;  // sorted by sqrDist!
  size_t                     foundMatches = 0;

  auto lambdaProcessCandidate =
      [&](const float sqrDist, const mrpt::math::TPoint3Df& pt, const global_plain_index_t& id)
  {
    // bubble sort (yes, really!)
    // found its position in the list:
    size_t i = 0;
    for (i = 0; i < foundMatches; i++)
    {
      if (sqrDist < matches[i].sqrDist)
      {
        break;
      }
    }
    if (i >= MAX_KNN)
    {
      return;
    }

    // insert new one at [i], shift [i+1:end] one position.
    const size_t last = std::min(foundMatches + 1, MAX_KNN);
    for (size_t j = i + 1; j < last; j++)
    {
      matches[j] = matches[j - 1];
    }

    matches[i].globalPt = pt;
    matches[i].id       = id;
    matches[i].sqrDist  = sqrDist;

    if (foundMatches < MAX_KNN)
    {
      foundMatches++;
    }
  };

  auto lambdaCheckCell = [&](const global_index3d_t& p)
  {
    if (auto* v = voxelByGlobalIdxs(p); v && !v->points().empty())
    {
      const auto& pts = v->points();
      for (size_t i = 0; i < pts.size(); i++)
      {
        const auto& pt      = pts[i];
        float       distSqr = (pt - query).sqrNorm();
        const auto  id      = g2plain(p, i);

        lambdaProcessCandidate(distSqr, pt, id);
      }
    }
  };

  for (int32_t cx = idxs0.cx; cx <= idxs1.cx; cx++)
  {
    for (int32_t cy = idxs0.cy; cy <= idxs1.cy; cy++)
    {
      for (int32_t cz = idxs0.cz; cz <= idxs1.cz; cz++)
      {
        lambdaCheckCell({cx, cy, cz});
      }
    }
  }

  for (size_t i = 0; i < std::min<size_t>(N, foundMatches); i++)
  {
    const auto& m = matches[i];

    out_dists_sqr.push_back(m.sqrDist);
    results.push_back(m.globalPt);
    resultIndicesOrIDs.push_back(m.id);
  }
}

void NDT::nn_radius_search(
    const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  results.clear();
  out_dists_sqr.clear();
  resultIndicesOrIDs.clear();

  if (search_radius_sqr <= 0)
  {
    return;
  }

  const float radius   = std::sqrt(search_radius_sqr);
  const auto  diagonal = mrpt::math::TPoint3Df(1.0f, 1.0f, 1.0f) * radius;

  const global_index3d_t idxs0 = coordToGlobalIdx(query - diagonal);
  const global_index3d_t idxs1 = coordToGlobalIdx(query + diagonal);

  // Data structures to avoid ANY heap memory allocation and keep working
  // on the stack at all time:
  struct Match
  {
    mrpt::math::TPoint3Df globalPt;
    float                 sqrDist = 0;
    uint64_t              id      = 0;
  };
  std::array<Match, HARD_MAX_MATCHES> matches;  // sorted by sqrDist!
  size_t                              foundMatches = 0;

  auto lambdaProcessCandidate =
      [&](const float sqrDist, const mrpt::math::TPoint3Df& pt, const global_plain_index_t& id)
  {
    // bubble sort (yes, really!)
    // found its position in the list:
    size_t i = 0;
    for (i = 0; i < foundMatches; i++)
    {
      if (sqrDist < matches[i].sqrDist)
      {
        break;
      }
    }
    if (i >= HARD_MAX_MATCHES)
    {
      return;
    }

    // insert new one at [i], shift [i+1:end] one position.
    const size_t last = std::min(foundMatches + 1, HARD_MAX_MATCHES);
    for (size_t j = i + 1; j < last; j++)
    {
      matches[j] = matches[j - 1];
    }

    matches[i].globalPt = pt;
    matches[i].id       = id;
    matches[i].sqrDist  = sqrDist;

    if (foundMatches < HARD_MAX_MATCHES)
    {
      foundMatches++;
    }
  };

  auto lambdaCheckCell = [&](const global_index3d_t& p)
  {
    if (auto* v = voxelByGlobalIdxs(p); v && !v->points().empty())
    {
      const auto& pts = v->points();
      for (size_t i = 0; i < pts.size(); i++)
      {
        const auto& pt      = pts[i];
        float       distSqr = (pt - query).sqrNorm();
        if (distSqr > search_radius_sqr)
        {
          continue;
        }

        const auto id = g2plain(p, i);

        if (maxPoints != 0)
        {
          // temporary list:
          lambdaProcessCandidate(distSqr, pt, id);
        }
        else
        {
          // directly save output:
          out_dists_sqr.push_back(distSqr);
          results.push_back(pt);
          resultIndicesOrIDs.push_back(id);
        }
      }
    }
  };

  for (int32_t cx = idxs0.cx; cx <= idxs1.cx; cx++)
  {
    for (int32_t cy = idxs0.cy; cy <= idxs1.cy; cy++)
    {
      for (int32_t cz = idxs0.cz; cz <= idxs1.cz; cz++)
      {
        lambdaCheckCell({cx, cy, cz});
      }
    }
  }

  if (maxPoints != 0)
  {
    // we saved results in a temporary buffer, save them out now:
    for (size_t i = 0; i < std::min<size_t>(maxPoints, foundMatches); i++)
    {
      const auto& m = matches[i];

      out_dists_sqr.push_back(m.sqrDist);
      results.push_back(m.globalPt);
      resultIndicesOrIDs.push_back(m.id);
    }
  }
}
