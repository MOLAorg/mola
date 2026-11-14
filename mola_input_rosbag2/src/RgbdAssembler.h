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
#pragma once

#include <mrpt/img/TCamera.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/poses/CPose3D.h>

#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <utility>
#include <vector>

namespace tf2
{
class BufferCore;
}

namespace mola
{
/** Builds mrpt::obs::CObservation3DRangeScan observations out of the ROS
 *  topics of an RGB-D camera: a depth image, an optional intensity image
 *  (color or IR), and the camera_info of each.
 *
 *  Depth and intensity frames are paired by header stamp. The observation
 *  keeps the raw sensor geometry, following the MRPT conventions:
 *  - `rangeImage` and `cameraParams`: the depth image and its intrinsics.
 *  - `intensityImage` and `cameraParamsIntensity`: the intensity image and
 *    its intrinsics, which may differ from the depth ones.
 *  - `relativePoseIntensityWRTDepth`: from /tf (depth -> intensity optical
 *    frames). Depth that is already registered to the intensity camera (e.g.
 *    a RealSense "aligned_depth_to_color" topic) yields
 *    doDepthAndIntensityCamerasCoincide() == true.
 *  - `sensorPose`: the depth camera on the robot, in the MRPT +X-forward
 *    convention, from /tf or from a fixed pose.
 *
 *  Depth encodings: 16UC1/mono16 (millimeters, REP 118) and 32FC1 (meters).
 */
class RgbdAssembler
{
 public:
  struct Parameters
  {
    std::string sensorLabel;
    bool        withIntensity = true;
    /** Maximum stamp difference between paired depth/intensity images [s] */
    double maxTimeOffset = 0.005;
    float  maxRange      = 10.0f;  //!< [m]
    /** Unpaired images kept waiting for their counterpart, per stream */
    size_t maxQueueLength = 10;
    /** If set, used as `sensorPose` instead of /tf (depth camera, +X forward) */
    std::optional<mrpt::poses::CPose3D> fixedSensorPose;
  };

  using Obs = std::vector<mrpt::obs::CObservation::Ptr>;

  RgbdAssembler(Parameters p, std::shared_ptr<const tf2::BufferCore> tf, std::string baseLinkFrame);

  void onDepthCameraInfo(const sensor_msgs::msg::CameraInfo& info);
  void onIntensityCameraInfo(const sensor_msgs::msg::CameraInfo& info);

  /** Returns the assembled observation, if this image completes a pair. */
  Obs onDepthImage(sensor_msgs::msg::Image&& img);
  Obs onIntensityImage(sensor_msgs::msg::Image&& img);

  /** Decodes a ROS depth image into `obs.rangeImage` and `obs.rangeUnits`. */
  static void decodeDepth(
      const sensor_msgs::msg::Image& img, mrpt::obs::CObservation3DRangeScan& obs);

  /** Pose of a camera optical frame (+Z forward, ROS REP 103) in the MRPT
   *  3D range scan sensor frame (+X forward): (0,0,0,-90deg,0,-90deg). */
  static const mrpt::poses::CPose3D& opticalFrameInSensorFrame();

 private:
  struct Pending
  {
    double                  stamp = 0;
    sensor_msgs::msg::Image msg;
  };

  Parameters                             params_;
  std::shared_ptr<const tf2::BufferCore> tf_;
  std::string                            baseLinkFrame_;

  std::optional<mrpt::img::TCamera> depthCamera_;
  std::optional<mrpt::img::TCamera> intensityCamera_;
  std::deque<Pending>               depthQueue_;
  std::deque<Pending>               intensityQueue_;

  /** Cached intensity-wrt-depth poses, by (depth frame, intensity frame) */
  std::map<std::pair<std::string, std::string>, mrpt::poses::CPose3D> intensityWrtDepth_;

  Obs pairWith(
      sensor_msgs::msg::Image&& img, std::deque<Pending>& own, std::deque<Pending>& other,
      bool imgIsDepth);

  mrpt::obs::CObservation3DRangeScan::Ptr build(
      const sensor_msgs::msg::Image& depth, const sensor_msgs::msg::Image* intensity);

  mrpt::poses::CPose3D lookupPose(const std::string& target, const std::string& source) const;
};

}  // namespace mola
