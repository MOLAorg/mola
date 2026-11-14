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

// Unit tests for the RGB-D observation assembler of Rosbag2Dataset.

#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/poses/CPose3D.h>

#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <tf2/buffer_core.hpp>
#include <utility>

#include "../src/RgbdAssembler.h"

using mola::RgbdAssembler;

namespace
{
constexpr const char* kDepthFrame = "camera_depth_optical_frame";
constexpr const char* kColorFrame = "camera_color_optical_frame";

builtin_interfaces::msg::Time rosTime(double t)
{
  builtin_interfaces::msg::Time s;
  s.sec     = static_cast<int32_t>(std::floor(t));
  s.nanosec = static_cast<uint32_t>(std::lround((t - std::floor(t)) * 1e9));
  return s;
}

sensor_msgs::msg::Image depthImage16(double t, int w, int h, uint16_t mm)
{
  sensor_msgs::msg::Image img;
  img.header.stamp    = rosTime(t);
  img.header.frame_id = kDepthFrame;
  img.width           = static_cast<uint32_t>(w);
  img.height          = static_cast<uint32_t>(h);
  img.encoding        = "16UC1";
  img.step            = static_cast<uint32_t>(w * 2 + 4);  // padded rows
  img.data.assign(static_cast<size_t>(img.step) * img.height, 0);
  for (int r = 0; r < h; r++)
  {
    for (int c = 0; c < w; c++)
    {
      std::memcpy(
          &img.data[static_cast<size_t>(r) * img.step + 2U * static_cast<size_t>(c)], &mm, 2);
    }
  }
  return img;
}

sensor_msgs::msg::Image colorImage(double t, int w, int h)
{
  sensor_msgs::msg::Image img;
  img.header.stamp    = rosTime(t);
  img.header.frame_id = kColorFrame;
  img.width           = static_cast<uint32_t>(w);
  img.height          = static_cast<uint32_t>(h);
  img.encoding        = "rgb8";
  img.step            = static_cast<uint32_t>(w * 3);
  img.data.assign(static_cast<size_t>(img.step) * img.height, 128);
  return img;
}

sensor_msgs::msg::CameraInfo cameraInfo(const char* frame, int w, int h, double f)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.header.frame_id  = frame;
  ci.width            = static_cast<uint32_t>(w);
  ci.height           = static_cast<uint32_t>(h);
  ci.distortion_model = "plumb_bob";
  ci.d                = {0, 0, 0, 0, 0};
  ci.k                = {f, 0, w / 2.0, 0, f, h / 2.0, 0, 0, 1};
  return ci;
}

void addStaticTf(
    tf2::BufferCore& tf, const std::string& parent, const std::string& child, double x, double y,
    double z, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id         = parent;
  t.child_frame_id          = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.x    = qx;
  t.transform.rotation.y    = qy;
  t.transform.rotation.z    = qz;
  t.transform.rotation.w    = qw;
  tf.setTransform(t, "test", true);
}

std::shared_ptr<tf2::BufferCore> makeTf()
{
  auto tf = std::make_shared<tf2::BufferCore>();
  // A forward-looking camera 0.5 m above base_link, and its usual optical
  // frames (as published by realsense2_camera): the color camera is 15 mm
  // to the left of the depth camera.
  addStaticTf(*tf, "base_link", "camera_link", 0.1, 0, 0.5, 0, 0, 0, 1);
  addStaticTf(*tf, "camera_link", kDepthFrame, 0, 0, 0, -0.5, 0.5, -0.5, 0.5);
  addStaticTf(*tf, "camera_link", "camera_color_frame", 0, 0.015, 0, 0, 0, 0, 1);
  addStaticTf(*tf, "camera_color_frame", kColorFrame, 0, 0, 0, -0.5, 0.5, -0.5, 0.5);
  return tf;
}

void test_decode_depth()
{
  mrpt::obs::CObservation3DRangeScan obs;

  RgbdAssembler::decodeDepth(depthImage16(0, 5, 3, 1234), obs);
  ASSERT_EQUAL_(obs.rangeImage.rows(), 3);
  ASSERT_EQUAL_(obs.rangeImage.cols(), 5);
  ASSERT_EQUAL_(obs.rangeImage(2, 4), 1234);
  ASSERT_NEAR_(obs.rangeUnits, 1e-3f, 1e-9f);

  sensor_msgs::msg::Image f;
  f.width                         = 4;
  f.height                        = 1;
  f.encoding                      = "32FC1";
  f.step                          = 16;
  const std::array<float, 4> vals = {1.5f, std::numeric_limits<float>::quiet_NaN(), -1.f, 0.f};
  f.data.resize(16);
  std::memcpy(f.data.data(), vals.data(), sizeof(float) * vals.size());
  RgbdAssembler::decodeDepth(f, obs);
  ASSERT_EQUAL_(obs.rangeImage(0, 0), 1500);
  ASSERT_EQUAL_(obs.rangeImage(0, 1), 0);
  ASSERT_EQUAL_(obs.rangeImage(0, 2), 0);
  ASSERT_EQUAL_(obs.rangeImage(0, 3), 0);
}

void test_pairing_and_geometry()
{
  RgbdAssembler::Parameters p;
  p.sensorLabel = "rgbd";
  RgbdAssembler a(p, makeTf(), "base_link");

  // Images before their camera_info cannot be assembled:
  a.onIntensityImage(colorImage(1.0, 8, 6));
  bool threw = false;
  try
  {
    a.onDepthImage(depthImage16(1.0, 10, 6, 1000));
  }
  catch (const std::exception&)
  {
    threw = true;
  }
  ASSERT_(threw);

  a.onDepthCameraInfo(cameraInfo(kDepthFrame, 10, 6, 5.0));
  a.onIntensityCameraInfo(cameraInfo(kColorFrame, 8, 6, 7.0));

  // Intensity first, then its depth (within the time tolerance):
  ASSERT_(a.onIntensityImage(colorImage(2.0, 8, 6)).empty());
  auto out = a.onDepthImage(depthImage16(2.002, 10, 6, 1000));
  ASSERT_EQUAL_(out.size(), 1U);
  auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(out.at(0));
  ASSERT_(obs);
  ASSERT_EQUAL_(obs->sensorLabel, "rgbd");
  ASSERT_(obs->hasRangeImage && obs->hasIntensityImage);
  ASSERT_EQUAL_(obs->cameraParams.ncols, 10U);
  ASSERT_EQUAL_(obs->cameraParamsIntensity.ncols, 8U);
  ASSERT_EQUAL_(obs->intensityImage.getWidth(), 8);

  // Depth sensor frame (+X forward) on the robot == camera_link:
  ASSERT_NEAR_(
      (obs->sensorPose.asVectorVal() - mrpt::poses::CPose3D(0.1, 0, 0.5, 0, 0, 0).asVectorVal())
          .norm(),
      0.0, 1e-6);

  // A point 1 m in front of the color camera, seen from the depth camera:
  ASSERT_(!obs->doDepthAndIntensityCamerasCoincide());
  const auto pt = obs->relativePoseIntensityWRTDepth.composePoint({0, 0, 1});
  ASSERT_NEAR_(pt.x, 1.0, 1e-6);
  ASSERT_NEAR_(pt.y, 0.015, 1e-6);
  ASSERT_NEAR_(pt.z, 0.0, 1e-6);

  // Depth first, then an intensity frame too far in time: no pair.
  ASSERT_(a.onDepthImage(depthImage16(3.0, 10, 6, 1000)).empty());
  ASSERT_(a.onIntensityImage(colorImage(3.1, 8, 6)).empty());
  // ...until the depth frame matching the pending intensity arrives:
  ASSERT_EQUAL_(a.onDepthImage(depthImage16(3.1, 10, 6, 1000)).size(), 1U);
}

void test_registered_and_depth_only()
{
  // Depth already registered to the intensity camera (same optical frame):
  {
    const RgbdAssembler::Parameters p;
    RgbdAssembler                   a(p, makeTf(), "base_link");
    a.onDepthCameraInfo(cameraInfo(kDepthFrame, 10, 6, 5.0));
    a.onIntensityCameraInfo(cameraInfo(kDepthFrame, 10, 6, 5.0));
    auto img            = colorImage(1.0, 10, 6);
    img.header.frame_id = kDepthFrame;
    ASSERT_(a.onIntensityImage(std::move(img)).empty());
    auto out = a.onDepthImage(depthImage16(1.0, 10, 6, 1000));
    ASSERT_EQUAL_(out.size(), 1U);
    auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(out.at(0));
    ASSERT_(obs->doDepthAndIntensityCamerasCoincide());
  }
  // No intensity stream:
  {
    RgbdAssembler::Parameters p;
    p.withIntensity   = false;
    p.fixedSensorPose = mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
    RgbdAssembler a(p, makeTf(), "base_link");
    a.onDepthCameraInfo(cameraInfo(kDepthFrame, 10, 6, 5.0));
    auto out = a.onDepthImage(depthImage16(1.0, 10, 6, 1000));
    ASSERT_EQUAL_(out.size(), 1U);
    auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(out.at(0));
    ASSERT_(!obs->hasIntensityImage);
    ASSERT_NEAR_(obs->sensorPose.z(), 3.0, 1e-9);
  }
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_decode_depth();
    test_pairing_and_geometry();
    test_registered_and_depth_only();
    std::cout << "All tests passed.\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
