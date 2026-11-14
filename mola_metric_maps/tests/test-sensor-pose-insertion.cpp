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
 * @file   test-sensor-pose-insertion.cpp
 * @brief  insertObservation() places points given in the sensor frame at
 *         robotPose + sensorPose, as MRPT's own point maps do.
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mola_metric_maps/NDT.h>
#include <mola_metric_maps/SparseTreesPointCloud.h>
#include <mola_metric_maps/SparseVoxelPointCloud.h>
#include <mola_metric_maps/TSDF.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>

#include <functional>
#include <iostream>
#include <string>
#include <type_traits>
#include <utility>

namespace
{
using mrpt::DEG2RAD;

const mrpt::poses::CPose3D robotPose(3.0, 2.0, 0.1, DEG2RAD(45.0), 0.0, 0.0);
const mrpt::poses::CPose3D sensorPose(1.0, -0.5, 0.8, DEG2RAD(30.0), DEG2RAD(10.0), DEG2RAD(-5.0));

/// A 4x2 m wall, 5 m in front of the sensor, in the sensor frame.
mrpt::maps::CSimplePointsMap::Ptr wallInSensorFrame()
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  for (int i = -40; i <= 40; i++)
  {
    for (int j = -20; j <= 20; j++)
    {
      pts->insertPoint(5.0f, static_cast<float>(i * 0.05), static_cast<float>(j * 0.05));
    }
  }
  return pts;
}

mrpt::obs::CObservationPointCloud::Ptr pointCloudObs()
{
  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud = wallInSensorFrame();
  obs->sensorPose = sensorPose;
  return obs;
}

/// The same wall, already transformed to the robot frame, with the sensor at
/// the robot origin.
mrpt::obs::CObservationPointCloud::Ptr referenceObs()
{
  const auto inSensor = wallInSensorFrame();
  auto       inRobot  = mrpt::maps::CSimplePointsMap::Create();
  for (size_t k = 0; k < inSensor->size(); k++)
  {
    mrpt::math::TPoint3Df p;
    inSensor->getPoint(k, p.x, p.y, p.z);
    inRobot->insertPoint(sensorPose.composePoint(p));
  }
  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud = inRobot;
  return obs;
}

mrpt::obs::CObservation3DRangeScan::Ptr rangeScanObs()
{
  const auto pts   = wallInSensorFrame();
  auto       obs   = mrpt::obs::CObservation3DRangeScan::Create();
  obs->hasPoints3D = true;
  obs->resizePoints3DVectors(pts->size());
  for (size_t k = 0; k < pts->size(); k++)
  {
    pts->getPoint(k, obs->points3D_x[k], obs->points3D_y[k], obs->points3D_z[k]);
  }
  obs->sensorPose = sensorPose;
  return obs;
}

template <class MAP, class = void>
struct HasVisitAllPoints : std::false_type
{
};
template <class MAP>
struct HasVisitAllPoints<
    MAP, std::void_t<decltype(std::declval<const MAP&>().visitAllPoints(
             std::function<void(const mrpt::math::TPoint3Df&)>()))>> : std::true_type
{
};

/// Centroid of all the points in a map.
template <class MAP>
mrpt::math::TPoint3D mapCentroid(const MAP& map)
{
  mrpt::math::TPoint3D c(0, 0, 0);
  size_t               n = 0;

  const auto add = [&](const mrpt::math::TPoint3Df& p)
  {
    c += mrpt::math::TPoint3D(p);
    n++;
  };

  if constexpr (HasVisitAllPoints<MAP>::value)
  {
    map.visitAllPoints(add);
  }
  else
  {
    const auto* pts = map.getAsSimplePointsMap();
    ASSERT_(pts);
    for (size_t k = 0; k < pts->size(); k++)
    {
      mrpt::math::TPoint3Df p;
      pts->getPoint(k, p.x, p.y, p.z);
      add(p);
    }
  }
  ASSERT_(n > 0);
  return c * (1.0 / static_cast<double>(n));
}

template <class MAP>
void test_map(const mrpt::obs::CObservation& obs, const std::string& name)
{
  MAP map;
  ASSERT_(map.insertObservation(obs, robotPose));

  // Same points, given in the robot frame:
  MAP ref;
  ASSERT_(ref.insertObservation(*referenceObs(), robotPose));

  const auto c        = mapCentroid(map);
  const auto expected = mapCentroid(ref);
  const auto err      = (c - expected).norm();

  // The sensor offset is over 1 m, so ignoring it would give a much larger
  // error. The tolerance covers TSDF, whose rays start at a different point in
  // the reference:
  ASSERTMSG_(
      err < 0.05, mrpt::format(
                      "%s: map centroid %s, expected %s", name.c_str(), c.asString().c_str(),
                      expected.asString().c_str()));

  std::cout << "[Test Passed] " << name << " (err=" << err << " m)\n";
}

}  // namespace

int main()
{
  try
  {
    const auto pc = pointCloudObs();
    test_map<mola::HashedVoxelPointCloud>(*pc, "HashedVoxelPointCloud, point cloud");
    test_map<mola::SparseVoxelPointCloud>(*pc, "SparseVoxelPointCloud, point cloud");
    test_map<mola::SparseTreesPointCloud>(*pc, "SparseTreesPointCloud, point cloud");
    test_map<mola::NDT>(*pc, "NDT, point cloud");
    test_map<mola::TSDF>(*pc, "TSDF, point cloud");
    test_map<mola::KeyframePointCloudMap>(*pc, "KeyframePointCloudMap, point cloud");

    const auto rs = rangeScanObs();
    test_map<mola::HashedVoxelPointCloud>(*rs, "HashedVoxelPointCloud, 3D range scan");
    test_map<mola::SparseVoxelPointCloud>(*rs, "SparseVoxelPointCloud, 3D range scan");
    test_map<mola::SparseTreesPointCloud>(*rs, "SparseTreesPointCloud, 3D range scan");
    test_map<mola::NDT>(*rs, "NDT, 3D range scan");
    test_map<mola::TSDF>(*rs, "TSDF, 3D range scan");
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
  return 0;
}
