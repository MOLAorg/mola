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

#include "RgbdAssembler.h"

#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

using namespace mola;

namespace
{
double stampOf(const sensor_msgs::msg::Image& img)
{
  return static_cast<double>(img.header.stamp.sec) +
         1e-9 * static_cast<double>(img.header.stamp.nanosec);
}
}  // namespace

const mrpt::poses::CPose3D& RgbdAssembler::opticalFrameInSensorFrame()
{
  using namespace mrpt;  // _deg
  static const mrpt::poses::CPose3D p(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);
  return p;
}

RgbdAssembler::RgbdAssembler(
    Parameters p, std::shared_ptr<const tf2::BufferCore> tf, std::string baseLinkFrame)
    : params_(std::move(p)), tf_(std::move(tf)), baseLinkFrame_(std::move(baseLinkFrame))
{
  ASSERT_(tf_);
  ASSERT_GT_(params_.maxQueueLength, 0U);
}

mrpt::img::TCamera RgbdAssembler::cameraFromROS(const sensor_msgs::msg::CameraInfo& info)
{
  // Same as mrpt::ros2bridge::fromROS(CameraInfo), which released versions of
  // mrpt_ros_bridge do not provide yet.
  namespace dm = sensor_msgs::distortion_models;

  mrpt::img::TCamera cam;
  cam.ncols = info.width;
  cam.nrows = info.height;
  cam.setIntrinsicParamsFromValues(info.k[0], info.k[4], info.k[2], info.k[5]);

  const auto d = [&info](size_t i) { return i < info.d.size() ? info.d[i] : 0.0; };
  if (info.distortion_model == dm::PLUMB_BOB || info.distortion_model == dm::RATIONAL_POLYNOMIAL)
  {
    cam.setDistortionPlumbBob(d(0), d(1), d(2), d(3), d(4));
    cam.k4(d(5));
    cam.k5(d(6));
    cam.k6(d(7));
  }
  else if (info.distortion_model == dm::EQUIDISTANT || info.distortion_model == "kannala_brandt")
  {
    cam.setDistortionKannalaBrandt(d(0), d(1), d(2), d(3));
  }
  else
  {
    cam.distortion = mrpt::img::DistortionModel::none;
  }
  return cam;
}

void RgbdAssembler::onDepthCameraInfo(const sensor_msgs::msg::CameraInfo& info)
{
  depthCamera_ = cameraFromROS(info);
}

void RgbdAssembler::onIntensityCameraInfo(const sensor_msgs::msg::CameraInfo& info)
{
  intensityCamera_ = cameraFromROS(info);
}

RgbdAssembler::Obs RgbdAssembler::onDepthImage(sensor_msgs::msg::Image&& img)
{
  if (!params_.withIntensity)
  {
    auto obs = build(img, nullptr);
    return obs ? Obs{obs} : Obs{};
  }
  return pairWith(std::move(img), depthQueue_, intensityQueue_, true);
}

RgbdAssembler::Obs RgbdAssembler::onIntensityImage(sensor_msgs::msg::Image&& img)
{
  return pairWith(std::move(img), intensityQueue_, depthQueue_, false);
}

RgbdAssembler::Obs RgbdAssembler::pairWith(
    sensor_msgs::msg::Image&& img, std::deque<Pending>& own, std::deque<Pending>& other,
    bool imgIsDepth)
{
  const double t = stampOf(img);

  auto   best   = other.end();
  double bestDt = params_.maxTimeOffset;
  for (auto it = other.begin(); it != other.end(); ++it)
  {
    const double dt = std::abs(it->stamp - t);
    if (dt <= bestDt)
    {
      best   = it;
      bestDt = dt;
    }
  }

  if (best == other.end())
  {
    own.push_back({t, std::move(img)});
    while (own.size() > params_.maxQueueLength)
    {
      // Report how close its best counterpart was, to tell a too strict
      // max_time_offset from a stream that is not arriving at all:
      double closest = std::numeric_limits<double>::infinity();
      for (const auto& o : other)
      {
        closest = std::min(closest, std::abs(o.stamp - own.front().stamp));
      }
      lastEvictedClosestDt_ = closest;
      droppedUnpaired_++;
      own.pop_front();
    }
    return {};
  }

  const Pending match = std::move(*best);
  // Streams arrive in time order: older entries can no longer be paired.
  other.erase(other.begin(), best + 1);
  while (!own.empty() && own.front().stamp <= t)
  {
    own.pop_front();
  }

  const auto& depth     = imgIsDepth ? img : match.msg;
  const auto& intensity = imgIsDepth ? match.msg : img;
  auto        obs       = build(depth, &intensity);
  return obs ? Obs{obs} : Obs{};
}

void RgbdAssembler::decodeDepth(
    const sensor_msgs::msg::Image& img, mrpt::obs::CObservation3DRangeScan& obs)
{
  namespace enc = sensor_msgs::image_encodings;

  const auto h = static_cast<int>(img.height);
  const auto w = static_cast<int>(img.width);
  ASSERTMSG_(!img.is_bigendian, "Big-endian depth images are not supported");

  obs.rangeImage_setSize(h, w);
  obs.hasRangeImage = true;

  if (img.encoding == enc::TYPE_16UC1 || img.encoding == enc::MONO16)
  {
    ASSERT_GE_(img.step, img.width * sizeof(uint16_t));
    ASSERT_GE_(img.data.size(), static_cast<size_t>(img.step) * img.height);
    obs.rangeUnits = 1e-3f;
    for (int r = 0; r < h; r++)
    {
      std::memcpy(
          &obs.rangeImage(r, 0), img.data.data() + static_cast<size_t>(r) * img.step,
          sizeof(uint16_t) * img.width);
    }
  }
  else if (img.encoding == enc::TYPE_32FC1)
  {
    ASSERT_GE_(img.step, img.width * sizeof(float));
    ASSERT_GE_(img.data.size(), static_cast<size_t>(img.step) * img.height);
    obs.rangeUnits             = 1e-3f;
    constexpr float kMaxMeters = std::numeric_limits<uint16_t>::max() * 1e-3f;
    for (int r = 0; r < h; r++)
    {
      const uint8_t* row = img.data.data() + static_cast<size_t>(r) * img.step;
      for (int c = 0; c < w; c++)
      {
        float d = 0;
        std::memcpy(&d, row + sizeof(float) * static_cast<size_t>(c), sizeof(float));
        // NaN, +-inf and non-positive values mean "no measurement":
        obs.rangeImage(r, c) = (std::isfinite(d) && d > 0 && d < kMaxMeters)
                                   ? static_cast<uint16_t>(std::lround(d * 1e3f))
                                   : 0;
      }
    }
  }
  else
  {
    THROW_EXCEPTION_FMT(
        "Unsupported depth image encoding '%s' (expected 16UC1, mono16 or 32FC1)",
        img.encoding.c_str());
  }
}

mrpt::poses::CPose3D RgbdAssembler::lookupPose(
    const std::string& target, const std::string& source) const
{
  try
  {
    const auto     tfMsg = tf_->lookupTransform(target, source, tf2::TimePointZero);
    tf2::Transform t;
    tf2::fromMsg(tfMsg.transform, t);
    return mrpt::ros2bridge::fromROS(t);
  }
  catch (const tf2::TransformException& ex)
  {
    THROW_EXCEPTION_FMT(
        "Could not find /tf '%s' -> '%s': %s", target.c_str(), source.c_str(), ex.what());
  }
}

mrpt::obs::CObservation3DRangeScan::Ptr RgbdAssembler::build(
    const sensor_msgs::msg::Image& depth, const sensor_msgs::msg::Image* intensity)
{
  // Drivers usually publish camera_info after the first images:
  if (!depthCamera_ || (intensity != nullptr && !intensityCamera_))
  {
    droppedNoInfo_++;
    return {};
  }

  auto obs            = mrpt::obs::CObservation3DRangeScan::Create();
  obs->sensorLabel    = params_.sensorLabel;
  obs->timestamp      = mrpt::ros2bridge::fromROS(depth.header.stamp);
  obs->range_is_depth = true;
  obs->maxRange       = params_.maxRange;

  decodeDepth(depth, *obs);
  obs->cameraParams = *depthCamera_;
  ASSERT_EQUAL_(obs->cameraParams.ncols, depth.width);
  ASSERT_EQUAL_(obs->cameraParams.nrows, depth.height);

  // ROS camera images are referenced to an optical frame (+Z forward):
  const auto& depthFrame = depth.header.frame_id;
  obs->sensorPose        = params_.fixedSensorPose
                               ? *params_.fixedSensorPose
                               : lookupPose(baseLinkFrame_, depthFrame) + (-opticalFrameInSensorFrame());

  if (intensity == nullptr)
  {
    obs->cameraParamsIntensity = obs->cameraParams;
    return obs;
  }

  obs->intensityImage        = mrpt::ros2bridge::fromROS(*intensity);
  obs->hasIntensityImage     = true;
  obs->cameraParamsIntensity = *intensityCamera_;
  ASSERT_EQUAL_(obs->cameraParamsIntensity.ncols, intensity->width);
  ASSERT_EQUAL_(obs->cameraParamsIntensity.nrows, intensity->height);

  // Camera extrinsics are static: look them up only once.
  const auto key = std::make_pair(depthFrame, intensity->header.frame_id);
  auto       it  = intensityWrtDepth_.find(key);
  if (it == intensityWrtDepth_.end())
  {
    const mrpt::poses::CPose3D depthToIntensity = key.first == key.second
                                                      ? mrpt::poses::CPose3D::Identity()
                                                      : lookupPose(key.first, key.second);
    it = intensityWrtDepth_.emplace(key, opticalFrameInSensorFrame() + depthToIntensity).first;
  }
  obs->relativePoseIntensityWRTDepth = it->second;

  // MRPT reads the intensity image at depth pixel coordinates when both
  // cameras coincide, so they must then share the image size too:
  ASSERTMSG_(
      !obs->doDepthAndIntensityCamerasCoincide() ||
          (depth.width == intensity->width && depth.height == intensity->height),
      "Depth and intensity cameras of sensor '" + params_.sensorLabel +
          "' coincide but their image sizes differ");

  return obs;
}
