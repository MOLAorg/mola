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
 * @file   MolaViz.cpp
 * @brief  Main C++ class for MOLA GUI
 * @author Jose Luis Blanco Claraco
 * @date   May  11, 2019
 */

/** \defgroup mola_viz_grp mola-viz
 * C++ library for main MOLA GUI
 */

#include <mola_viz/MolaViz.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/thread_name.h>
#include <mrpt/version.h>

#include <array>
#include <cinttypes>

#include "mola_icon_64x64.h"

using namespace mola;

IMPLEMENTS_MRPT_OBJECT(MolaViz, ExecutableBase, mola)

// ---------------------------------------------------------------------------
// Handler registry
// ---------------------------------------------------------------------------

struct HandlersContainer
{
  static HandlersContainer& Instance()
  {
    static HandlersContainer o;
    return o;
  }

  std::multimap<MolaViz::class_name_t, MolaViz::update_handler_t> guiHandlers_;
  std::mutex                                                      guiHandlersMtx_;

 private:
  HandlersContainer() = default;
};

// ---------------------------------------------------------------------------
// Anonymous-namespace helpers (sensor handlers)
// ---------------------------------------------------------------------------

namespace
{

constexpr const char* DECAY_CLOUDS_NAME = "__viz_decaying_clouds";

// Ignore NaN and Inf
template <typename Iter>
std::pair<Iter, Iter> minmax_ignore_nan(Iter begin, Iter end)
{
  Iter first = std::find_if(begin, end, [](auto x) { return !std::isnan(x) && !std::isinf(x); });
  if (first == end)
  {
    return {end, end};
  }

  Iter itMin = first;
  Iter itMax = first;
  for (Iter it = std::next(first); it != end; ++it)
  {
    if (!std::isnan(*it) && !std::isinf(*it))
    {
      if (*it < *itMin)
      {
        itMin = it;
      }
      if (*it > *itMax)
      {
        itMax = it;
      }
    }
  }
  return {itMin, itMax};
}

template <typename Iter>
std::string minmax_ignore_nan_str(Iter begin, Iter end)
{
  const auto [itMin, itMax] = minmax_ignore_nan(begin, end);
  if (itMin == end)
  {
    return "[N/A,N/A]";
  }
  return "[" + std::to_string(*itMin) + "," + std::to_string(*itMax) + "]";
}

void gui_handler_show_common_sensor_info(
    const mrpt::obs::CObservation& obs, nanogui::Window* w,
    const double sensor_rate_decimation = 1.0, const std::vector<std::string>& additionalMsgs = {})
{
  auto* glControl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
  if (glControl == nullptr)
  {
    return;
  }
  if (!glControl->scene)
  {
    return;
  }

  auto glView = glControl->scene->getViewport();
  if (!glView)
  {
    return;
  }

  constexpr unsigned int TXT_ID_TIMESTAMP   = 0;
  constexpr unsigned int TXT_ID_RATE        = 1;
  constexpr unsigned int TXT_ID_SENSOR_POSE = 2;
  constexpr unsigned int TXT_ID_ADDITIONALS = 3;

  mrpt::opengl::TFontParams fp;
  fp.color        = {1.0f, 1.0f, 1.0f};
  fp.draw_shadow  = true;
  fp.shadow_color = {0.0f, 0.0f, 0.0f};
  fp.vfont_scale  = 9;

  const auto line_y = [&fp](const int line)
  { return 2 + static_cast<float>(line) * (2 + fp.vfont_scale); };

  glView->addTextMessage(
      2, line_y(TXT_ID_TIMESTAMP),
      mrpt::format("Timestamp: %s", mrpt::system::dateTimeToString(obs.timestamp).c_str()),
      TXT_ID_TIMESTAMP, fp);

  mrpt::poses::CPose3D sensorPose;
  obs.getSensorPose(sensorPose);

  glView->addTextMessage(
      2, line_y(TXT_ID_SENSOR_POSE), mrpt::format("Sensor pose: %s", sensorPose.asString().c_str()),
      TXT_ID_SENSOR_POSE, fp);

  thread_local std::map<nanogui::Window*, double> estimatedHzs;
  thread_local std::map<nanogui::Window*, double> lastTimestamp;

  const double curTim = mrpt::Clock::toDouble(obs.timestamp);
  if (lastTimestamp.count(w) != 0)
  {
    const double At    = curTim - lastTimestamp[w];
    const double curHz = (At > 0 ? (1.0 / At) : 1.0) * sensor_rate_decimation;
    const double alpha = 0.9;

    double showHz = 0;
    if (estimatedHzs.count(w) == 0)
    {
      estimatedHzs[w] = curHz;
      showHz          = curHz;
    }
    else
    {
      double& estimatedHz = estimatedHzs[w];
      estimatedHz         = alpha * estimatedHz + (1.0 - alpha) * curHz;
      showHz              = estimatedHz;
    }

    glView->addTextMessage(
        2, line_y(TXT_ID_RATE),
        mrpt::format("Rate: %7.03f Hz Class: %s", showHz, obs.GetRuntimeClass()->className),
        TXT_ID_RATE, fp);
  }
  lastTimestamp[w] = curTim;

  for (size_t i = 0; i < additionalMsgs.size(); i++)
  {
    const auto id = TXT_ID_ADDITIONALS + i;
    glView->addTextMessage(2, line_y(static_cast<int>(id)), additionalMsgs.at(i), id, fp);
  }
}

// CObservationImage
void gui_handler_images(
    const mrpt::rtti::CObject::Ptr& o, nanogui::Window* w, const MolaViz::window_name_t& parentWin,
    MolaViz* instance, [[maybe_unused]] const mrpt::containers::yaml* extra_parameters)
{
  mrpt::img::CImage imgToShow;

  if (auto obj = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o); obj)
  {
    obj->load();
    imgToShow = obj->image.makeShallowCopy();
  }
  else if (auto obj3D = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(o);
           obj3D && obj3D->hasIntensityImage)
  {
    imgToShow = obj3D->intensityImage.makeShallowCopy();
  }
  else
  {
    return;
  }

  mrpt::gui::MRPT2NanoguiGLCanvas* glControl = nullptr;
  if (w->children().size() == 1)
  {
    auto winW = static_cast<int>(imgToShow.getWidth());
    auto winH = static_cast<int>(imgToShow.getHeight());
    while (winW > 512 || winH > 512)
    {
      winW /= 2;
      winH /= 2;
    }

    glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
    glControl->setSize({winW, winH});
    glControl->setFixedSize({winW, winH});

    auto lck         = mrpt::lockHelper(glControl->scene_mtx);
    glControl->scene = mrpt::opengl::COpenGLScene::Create();
    instance->markWindowForReLayout(parentWin);
  }
  else
  {
    glControl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
  }
  ASSERT_(glControl != nullptr);

  const double sensorDecimation = [&]()
  {
    if (extra_parameters)
    {
      return extra_parameters->getOrDefault("sensor_rate_decimation", 1.0);
    }
    return 1.0;
  }();

  const auto imgW        = static_cast<int>(imgToShow.getWidth());
  const auto imgH        = static_cast<int>(imgToShow.getHeight());
  const int  imgChannels = imgToShow.channelCount();

  auto lck = mrpt::lockHelper(glControl->scene_mtx);
  glControl->scene->getViewport()->setImageView(imgToShow);

  gui_handler_show_common_sensor_info(
      *std::dynamic_pointer_cast<mrpt::obs::CObservation>(o), w, sensorDecimation,
      {mrpt::format("Size: %ix%ix%i", imgW, imgH, imgChannels)});
}

// ---- Helper: set up or reuse the GL canvas + point cloud objects ----
struct PointCloudGLObjects
{
  mrpt::gui::MRPT2NanoguiGLCanvas*            glControl = nullptr;
  mrpt::opengl::CPointCloudColoured::Ptr      glPc;
  mrpt::opengl::CSetOfObjects::Ptr            glCornerRef;
  mrpt::opengl::CSetOfObjects::Ptr            glCornerSensor;
  std::optional<mrpt::LockHelper<std::mutex>> lck;
};

PointCloudGLObjects setup_or_reuse_point_cloud_gl(
    nanogui::Window* w, float point_size, const MolaViz::window_name_t& parentWin,
    MolaViz* instance)
{
  PointCloudGLObjects gl;

  if (w->children().size() == 1)
  {
    gl.glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
    gl.lck.emplace(&gl.glControl->scene_mtx);
    gl.glControl->scene = mrpt::opengl::COpenGLScene::Create();
    gl.glPc             = mrpt::opengl::CPointCloudColoured::Create();
    gl.glControl->scene->insert(gl.glPc);
    gl.glCornerRef    = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    gl.glCornerSensor = mrpt::opengl::stock_objects::CornerXYZ(0.5f);
    gl.glControl->scene->insert(gl.glCornerRef);
    gl.glControl->scene->insert(gl.glCornerSensor);
    gl.glPc->setPointSize(point_size);
    instance->markWindowForReLayout(parentWin);
  }
  else
  {
    gl.glControl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
    gl.lck.emplace(&gl.glControl->scene_mtx);
    gl.glPc           = gl.glControl->scene->getByClass<mrpt::opengl::CPointCloudColoured>();
    gl.glCornerRef    = gl.glControl->scene->getByClass<mrpt::opengl::CSetOfObjects>(0);
    gl.glCornerSensor = gl.glControl->scene->getByClass<mrpt::opengl::CSetOfObjects>(1);
  }
  ASSERT_(gl.glControl != nullptr);
  ASSERT_(gl.glPc);
  ASSERT_(gl.glCornerRef);
  ASSERT_(gl.glCornerSensor);

  return gl;
}

bool populate_from_observation_point_cloud(
    const mrpt::obs::CObservationPointCloud&      objPc,
    const mrpt::opengl::CPointCloudColoured::Ptr& glPc, nanogui::Window* w, double sensorDecimation)
{
  auto& objPcMut = const_cast<mrpt::obs::CObservationPointCloud&>(objPc);
  objPcMut.load();
  if (!objPc.pointcloud)
  {
    return false;
  }
  glPc->loadFromPointsMap(objPc.pointcloud.get());
  glPc->setPose(objPc.sensorPose);

  std::vector<std::string> additionalMsgs = {
      mrpt::format("Point count: %zu", objPc.pointcloud->size()),
      mrpt::format("Type: %s", objPc.pointcloud->GetRuntimeClass()->className),
  };

  for (const auto& field : objPc.pointcloud->getPointFieldNames_float())
  {
    if (const auto* buf = objPc.pointcloud->getPointsBufferRef_float_field(field);
        buf != nullptr && !buf->empty())
    {
      additionalMsgs.push_back(mrpt::format(
          "%.*s range: %s", static_cast<int>(field.size()), field.data(),
          minmax_ignore_nan_str(buf->begin(), buf->end()).c_str()));
    }
  }
  for (const auto& field : objPc.pointcloud->getPointFieldNames_uint16())
  {
    if (const auto* buf = objPc.pointcloud->getPointsBufferRef_uint16_field(field);
        buf != nullptr && !buf->empty())
    {
      additionalMsgs.push_back(mrpt::format(
          "%.*s range: %s", static_cast<int>(field.size()), field.data(),
          minmax_ignore_nan_str(buf->begin(), buf->end()).c_str()));
    }
  }
  for (const auto& field : objPc.pointcloud->getPointFieldNames_double())
  {
    if (const auto* buf = objPc.pointcloud->getPointsBufferRef_double_field(field);
        buf != nullptr && !buf->empty())
    {
      additionalMsgs.push_back(mrpt::format(
          "%.*s range: %s", static_cast<int>(field.size()), field.data(),
          minmax_ignore_nan_str(buf->begin(), buf->end()).c_str()));
    }
  }
  for (const auto& field : objPc.pointcloud->getPointFieldNames_uint8())
  {
    if (const auto* buf = objPc.pointcloud->getPointsBufferRef_uint8_field(field);
        buf != nullptr && !buf->empty())
    {
      additionalMsgs.push_back(mrpt::format(
          "%.*s range: %s", static_cast<int>(field.size()), field.data(),
          minmax_ignore_nan_str(buf->begin(), buf->end()).c_str()));
    }
  }

  gui_handler_show_common_sensor_info(objPc, w, sensorDecimation, additionalMsgs);
  return true;
}

void populate_from_rotating_scan(
    const mrpt::obs::CObservationRotatingScan&    objRS,
    const mrpt::opengl::CPointCloudColoured::Ptr& glPc, nanogui::Window* w, double sensorDecimation)
{
  auto& objRSMut = const_cast<mrpt::obs::CObservationRotatingScan&>(objRS);
  objRSMut.load();
  glPc->clear();
  mrpt::math::TBoundingBoxf bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

  for (size_t r = 0; r < objRS.rowCount; r++)
  {
    for (size_t c = 0; c < objRS.columnCount; c++)
    {
      const auto range = objRS.rangeImage(r, c);
      if (range == 0)
      {
        continue;
      }
      const auto& pt = objRS.organizedPoints(r, c);
      glPc->insertPoint({pt.x, pt.y, pt.z, 0, 0, 0});
      bbox.updateWithPoint(pt);
    }
  }
  glPc->recolorizeByCoordinate(bbox.min.z, bbox.max.z);
  gui_handler_show_common_sensor_info(objRS, w, sensorDecimation);
}

void populate_from_3d_range_scan(
    const mrpt::obs::CObservation3DRangeScan&     obj3D,
    const mrpt::opengl::CPointCloudColoured::Ptr& glPc, nanogui::Window* w, double sensorDecimation,
    bool& color_from_z)
{
  if (obj3D.hasPoints3D)
  {
    auto& obj3DMut = const_cast<mrpt::obs::CObservation3DRangeScan&>(obj3D);
    if (obj3D.points3D_isExternallyStored())
    {
      obj3DMut.load();
    }
    for (size_t i = 0; i < obj3D.points3D_x.size(); i++)
    {
      glPc->insertPoint({obj3D.points3D_x[i], obj3D.points3D_y[i], obj3D.points3D_z[i], 0, 0, 0});
    }
  }
  else
  {
    auto& obj3DMut = const_cast<mrpt::obs::CObservation3DRangeScan&>(obj3D);
    obj3DMut.load();
    mrpt::obs::T3DPointsProjectionParams pp;
    pp.takeIntoAccountSensorPoseOnRobot = true;
    if (obj3D.hasRangeImage && obj3D.hasIntensityImage)
    {
      auto pointMapCol                = mrpt::maps::CColouredPointsMap::Create();
      pointMapCol->colorScheme.scheme = mrpt::maps::CColouredPointsMap::cmFromIntensityImage;
      obj3DMut.unprojectInto(*pointMapCol, pp);
      glPc->loadFromPointsMap(pointMapCol.get());
      color_from_z = false;
    }
    else
    {
      obj3DMut.unprojectInto(*glPc, pp);
    }
  }
  gui_handler_show_common_sensor_info(obj3D, w, sensorDecimation);
}

void populate_from_2d_range_scan(
    const mrpt::obs::CObservation2DRangeScan&     obj2D,
    const mrpt::opengl::CPointCloudColoured::Ptr& glPc, nanogui::Window* w, double sensorDecimation)
{
  mrpt::maps::CSimplePointsMap auxMap;
  auxMap.insertObservationPtr(std::make_shared<mrpt::obs::CObservation2DRangeScan>(obj2D));
  glPc->loadFromPointsMap(&auxMap);
  gui_handler_show_common_sensor_info(obj2D, w, sensorDecimation);
}

bool populate_from_velodyne_scan(
    const mrpt::obs::CObservationVelodyneScan&    objVel,
    const mrpt::opengl::CPointCloudColoured::Ptr& glPc, nanogui::Window* w, double sensorDecimation)
{
  if (objVel.point_cloud.size() == 0)
  {
    return false;
  }

  const auto&                   pc = objVel.point_cloud;
  const size_t                  N  = pc.size();
  mrpt::maps::CGenericPointsMap pts;
  pts.registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  pts.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    pts.setPoint(i, pc.x[i], pc.y[i], pc.z[i]);
    pts.setPointField_float(
        i, mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY,
        static_cast<float>(pc.intensity[i]) / 255.0f);
  }

  glPc->loadFromPointsMap(&pts);
  gui_handler_show_common_sensor_info(
      objVel, w, sensorDecimation, {mrpt::format("Point count: %zu", N)});
  return true;
}

void gui_handler_point_cloud(
    const mrpt::rtti::CObject::Ptr& o, nanogui::Window* w, const MolaViz::window_name_t& parentWin,
    MolaViz* instance, const mrpt::containers::yaml* extra_parameters)
{
  using namespace mrpt::obs;

  const double sensorDecimation = [&]()
  {
    if (extra_parameters)
    {
      return extra_parameters->getOrDefault("sensor_rate_decimation", 1.0);
    }
    return 1.0;
  }();

  bool  color_from_z = true;
  float point_size   = 3.0f;
  if (extra_parameters != nullptr)
  {
    point_size   = extra_parameters->getOrDefault("point_size", point_size);
    color_from_z = extra_parameters->getOrDefault("color_from_z", color_from_z);
  }

  auto gl = setup_or_reuse_point_cloud_gl(w, point_size, parentWin, instance);
  gl.glPc->setPose(mrpt::poses::CPose3D::Identity());

  if (auto obs = std::dynamic_pointer_cast<CObservation>(o); obs)
  {
    mrpt::poses::CPose3D p;
    obs->getSensorPose(p);
    gl.glCornerSensor->setPose(p);
  }

  if (auto objPc = std::dynamic_pointer_cast<CObservationPointCloud>(o); objPc)
  {
    if (!populate_from_observation_point_cloud(*objPc, gl.glPc, w, sensorDecimation))
    {
      return;
    }
  }
  else if (auto objRS = std::dynamic_pointer_cast<CObservationRotatingScan>(o); objRS)
  {
    populate_from_rotating_scan(*objRS, gl.glPc, w, sensorDecimation);
    color_from_z = false;
  }
  else if (auto obj3D = std::dynamic_pointer_cast<CObservation3DRangeScan>(o);
           instance->show_rgbd_as_point_cloud_ && obj3D)
  {
    populate_from_3d_range_scan(*obj3D, gl.glPc, w, sensorDecimation, color_from_z);
  }
  else if (auto obj2D = std::dynamic_pointer_cast<CObservation2DRangeScan>(o); obj2D)
  {
    populate_from_2d_range_scan(*obj2D, gl.glPc, w, sensorDecimation);
  }
  else if (auto objVel = std::dynamic_pointer_cast<CObservationVelodyneScan>(o); objVel)
  {
    if (!populate_from_velodyne_scan(*objVel, gl.glPc, w, sensorDecimation))
    {
      return;
    }
  }
  else
  {
    return;
  }

  if (color_from_z)
  {
    const auto bb = gl.glPc->getBoundingBox();
    gl.glPc->recolorizeByCoordinate(static_cast<float>(bb.min.z), static_cast<float>(bb.max.z));
  }
}

void gui_handler_gps(
    const mrpt::rtti::CObject::Ptr& o, nanogui::Window* w, const MolaViz::window_name_t& parentWin,
    MolaViz* instance, [[maybe_unused]] const mrpt::containers::yaml* extra_parameters)
{
  auto obj = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o);
  if (!obj)
  {
    return;
  }

  std::array<nanogui::Label*, 6> labels{};
  labels.fill(nullptr);
  if (w->children().size() == 1)
  {
    w->setLayout(new nanogui::GridLayout(
        nanogui::Orientation::Horizontal, 1, nanogui::Alignment::Fill, 2, 2));
    for (size_t i = 0; i < labels.size(); i++)
    {
      labels[i] = w->add<nanogui::Label>(" ");
    }
    const int winW = 250;
    w->setSize({winW, 0});
    w->setFixedSize({winW, 0});
    instance->markWindowForReLayout(parentWin);
  }
  else
  {
    for (size_t i = 0; i < labels.size(); i++)
    {
      labels[i] = dynamic_cast<nanogui::Label*>(w->children().at(1 + i));
    }
  }
  for (const auto& label : labels)
  {
    ASSERT_(label);
  }

  if (auto* gga = obj->getMsgByClassPtr<mrpt::obs::gnss::Message_NMEA_GGA>(); gga)
  {
    labels[0]->setCaption(mrpt::format("Latitude: %.06f deg", gga->fields.latitude_degrees));
    labels[1]->setCaption(mrpt::format("Longitude: %.06f deg", gga->fields.longitude_degrees));
    labels[2]->setCaption(mrpt::format("Altitude: %.02f m", gga->fields.altitude_meters));
    labels[3]->setCaption(mrpt::format("HDOP: %.02f", gga->fields.HDOP));
    labels[4]->setCaption(mrpt::format(
        "GGA UTC time: %02u:%02u:%02.03f", static_cast<unsigned int>(gga->fields.UTCTime.hour),
        static_cast<unsigned int>(gga->fields.UTCTime.minute), gga->fields.UTCTime.sec));
  }
  if (obj->covariance_enu.has_value())
  {
    const auto&  cov   = obj->covariance_enu.value();
    const double std_x = std::sqrt(cov(0, 0));
    const double std_y = std::sqrt(cov(1, 1));
    const double std_z = std::sqrt(cov(2, 2));
    labels[5]->setCaption(
        mrpt::format("sigmas [m]: x=%.02f  y=%.02f  z=%.02f", std_x, std_y, std_z));
  }
}

void gui_handler_imu(
    const mrpt::rtti::CObject::Ptr& o, nanogui::Window* w, const MolaViz::window_name_t& parentWin,
    MolaViz* instance, [[maybe_unused]] const mrpt::containers::yaml* extra_parameters)
{
  auto obj = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(o);
  if (!obj)
  {
    return;
  }

  const double sensorDecimation = [&]()
  {
    if (extra_parameters)
    {
      return extra_parameters->getOrDefault("sensor_rate_decimation", 1.0);
    }
    return 1.0;
  }();

  mrpt::gui::MRPT2NanoguiGLCanvas*            glControl = nullptr;
  std::optional<mrpt::LockHelper<std::mutex>> lck;

  if (w->children().size() == 1)
  {
    w->setLayout(new nanogui::GridLayout(
        nanogui::Orientation::Horizontal, 1, nanogui::Alignment::Fill, 2, 2));
    glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
    lck.emplace(&glControl->scene_mtx);
    glControl->scene = mrpt::opengl::COpenGLScene::Create();
    const int winW   = 400;
    const int winH   = 125;
    glControl->setSize({winW, winH});
    glControl->setFixedSize({winW, winH});
    instance->markWindowForReLayout(parentWin);
  }
  else
  {
    glControl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
    lck.emplace(&glControl->scene_mtx);
  }
  ASSERT_(glControl != nullptr);

  std::vector<std::string> txts;
  if (obj->has(mrpt::obs::IMU_WX))
  {
    txts.push_back(mrpt::format(
        "omega=(%7.04f,%7.04f,%7.04f)", obj->get(mrpt::obs::IMU_WX), obj->get(mrpt::obs::IMU_WY),
        obj->get(mrpt::obs::IMU_WZ)));
  }
  else
  {
    txts.emplace_back("omega=None");
  }

  if (obj->has(mrpt::obs::IMU_X_ACC))
  {
    txts.push_back(mrpt::format(
        "acc=(%7.04f,%7.04f,%7.04f)", obj->get(mrpt::obs::IMU_X_ACC),
        obj->get(mrpt::obs::IMU_Y_ACC), obj->get(mrpt::obs::IMU_Z_ACC)));
  }
  else
  {
    txts.emplace_back("acc=None");
  }

  gui_handler_show_common_sensor_info(*obj, w, sensorDecimation, txts);
}

// ---------------------------------------------------------------------------
// Widget description builder helpers - called from the GUI thread
// ---------------------------------------------------------------------------

/** Recursively builds nanogui widgets from a LeafWidget variant into `parent`.
 *  Must be called from the GUI thread. */
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void build_leaf_widget(
    nanogui::Widget* parent, const mola::gui::LeafWidget& w,
    std::map<uint64_t, mola::gui::LiveString::Ptr>& liveStringRegistry)
{
  std::visit(
      // NOLINTNEXTLINE(readability-function-cognitive-complexity)
      [&](auto&& widget)
      {
        using T = std::decay_t<decltype(widget)>;

        if constexpr (std::is_same_v<T, mola::gui::Label>)
        {
          auto* lb = parent->add<nanogui::Label>(" ");
          if (widget.font_size > 0)
          {
            lb->setFontSize(widget.font_size);
          }
          if (widget.fixed_width > 0)
          {
            lb->setFixedWidth(widget.fixed_width);
          }
          // Use the LiveString's intrinsic unique ID as registry key.
          const uint64_t lsId      = widget.text->id();
          liveStringRegistry[lsId] = widget.text;
          lb->setId(mrpt::format("_live_%" PRIu64, lsId));
          // Set initial caption from the display member (which the constructor
          // populates).  poll() would return false here because dirty_ starts
          // as false - the initial value only lives in `display`.
          lb->setCaption(widget.text->display);
        }
        else if constexpr (std::is_same_v<T, mola::gui::Separator>)
        {
          // A zero-height widget acts as a visual spacer in nanogui.
          auto* sep = parent->add<nanogui::Widget>();
          sep->setHeight(4);
        }
        else if constexpr (std::is_same_v<T, mola::gui::CheckBox>)
        {
          auto* cb = parent->add<nanogui::CheckBox>(widget.label);
          cb->setChecked(widget.initial_value);
          if (widget.on_change)
          {
            cb->setCallback(widget.on_change);
          }
        }
        else if constexpr (std::is_same_v<T, mola::gui::Button>)
        {
          auto* btn = parent->add<nanogui::Button>(widget.label, widget.icon_entypo);
          if (widget.font_size > 0)
          {
            btn->setFontSize(widget.font_size);
          }
          if (widget.on_click)
          {
            btn->setCallback(widget.on_click);
          }
        }
        else if constexpr (std::is_same_v<T, mola::gui::TextBox>)
        {
          // Optional heading label above the field:
          if (!widget.label.empty())
          {
            parent->add<nanogui::Label>(widget.label);
          }

          auto* tb = parent->add<nanogui::TextBox>();
          if (widget.font_size > 0)
          {
            tb->setFontSize(widget.font_size);
          }
          tb->setEditable(true);
          tb->setAlignment(nanogui::TextBox::Alignment::Left);
          tb->setValue(widget.initial_value);
          if (widget.on_change)
          {
            tb->setCallback(widget.on_change);
          }
        }
        else if constexpr (std::is_same_v<T, mola::gui::TextPanel>)
        {
          if (!widget.label.empty())
          {
            parent->add<nanogui::Label>(widget.label);
          }

          auto* tb = parent->add<nanogui::TextBox>();
          if (widget.size_pixels[0] > 0 || widget.size_pixels[1] > 0)
          {
            tb->setFixedSize({widget.size_pixels[0], widget.size_pixels[1]});
          }
          tb->setEditable(widget.editable);
          tb->setAlignment(nanogui::TextBox::Alignment::Left);
          // Use the LiveString's intrinsic unique ID as registry key.
          const uint64_t lsId      = widget.live_text->id();
          liveStringRegistry[lsId] = widget.live_text;
          tb->setId(mrpt::format("_live_%" PRIu64, lsId));
          // Set initial value from the display member.
          std::string tmp;
          if (widget.live_text->poll(tmp))
          {
            tb->setValue(tmp);
          }
          if (widget.editable && widget.on_change)
          {
            tb->setCallback(
                [cb = widget.on_change](const std::string& s)
                {
                  cb(s);
                  return true;
                });
          }
        }
        else if constexpr (std::is_same_v<T, mola::gui::SliderFloat>)
        {
          auto* row = parent->add<nanogui::Widget>();
          row->setLayout(new nanogui::BoxLayout(
              nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 4));

          if (!widget.label.empty())
          {
            row->add<nanogui::Label>(widget.label);
          }

          auto* sl = row->add<nanogui::Slider>();
          if (widget.fixed_width > 0)
          {
            sl->setFixedWidth(widget.fixed_width);
          }
          // Normalise to [0,1]:
          const float range = widget.max_value - widget.min_value;
          sl->setValue(range > 0.0f ? (widget.initial_value - widget.min_value) / range : 0.0f);

          sl->setCallback(
              [min = widget.min_value, max = widget.max_value, fmt = widget.format_string,
               cb = widget.on_change](float v)
              {
                const float real = min + v * (max - min);
                if (cb)
                {
                  cb(real);
                }
              });
        }
        else if constexpr (std::is_same_v<T, mola::gui::SliderInt>)
        {
          if (!widget.label.empty())
          {
            parent->add<nanogui::Label>(widget.label);
          }

          auto* row = parent->add<nanogui::Widget>();
          row->setLayout(new nanogui::BoxLayout(
              nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 4));

          auto* sl = row->add<nanogui::Slider>();
          if (widget.fixed_width > 0)
          {
            sl->setFixedWidth(widget.fixed_width);
          }
          auto* valLbl = row->add<nanogui::Label>(std::to_string(widget.initial_value));
          valLbl->setFixedWidth(50);

          const float range = static_cast<float>(widget.max_value - widget.min_value);
          sl->setValue(
              range > 0.0f ? static_cast<float>(widget.initial_value - widget.min_value) / range
                           : 0.0f);

          sl->setCallback(
              [min = widget.min_value, max = widget.max_value, valLbl,
               cb = widget.on_change](float v)
              {
                const int real =
                    min + static_cast<int>(std::round(v * static_cast<float>(max - min)));
                valLbl->setCaption(std::to_string(real));
                if (cb)
                {
                  cb(real);
                }
              });
        }
        else if constexpr (std::is_same_v<T, mola::gui::ComboBox>)
        {
          if (!widget.label.empty())
          {
            parent->add<nanogui::Label>(widget.label);
          }

          auto* cb = parent->add<nanogui::ComboBox>();
          cb->setItems(widget.items);
          cb->setSelectedIndex(widget.initial_index);
          if (widget.on_change)
          {
            cb->setCallback(widget.on_change);
          }
        }
      },
      w);
}

/** Builds all widgets for one Tab into `tabPage`.  Must run on GUI thread. */
void build_tab_widgets(
    nanogui::Widget* tabPage, const mola::gui::Tab& tab,
    std::map<uint64_t, mola::gui::LiveString::Ptr>& liveStringRegistry)
{
  for (const auto& anyW : tab.widgets)
  {
    std::visit(
        [&](auto&& widget)
        {
          using T = std::decay_t<decltype(widget)>;

          if constexpr (std::is_same_v<T, mola::gui::Row>)
          {
            // Horizontal row: create a child panel with horizontal layout.
            auto*     row     = tabPage->add<nanogui::Widget>();
            const int spacing = widget.item_spacing > 0 ? widget.item_spacing : 1;
            row->setLayout(new nanogui::BoxLayout(
                nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, spacing, spacing));
            for (const auto& leaf : widget.widgets)
            {
              build_leaf_widget(row, leaf, liveStringRegistry);
            }
          }
          else
          {
            // All leaf types are also valid AnyWidget alternatives:
            build_leaf_widget(tabPage, widget, liveStringRegistry);
          }
        },
        anyW);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// MRPT initializer: register module + default GUI handlers
// ---------------------------------------------------------------------------

MRPT_INITIALIZER(do_register_MolaViz)  // NOLINT(misc-use-anonymous-namespace)
{
  MOLA_REGISTER_MODULE(MolaViz);

  // clang-format off
  MolaViz::register_gui_handler("mrpt::obs::CObservationImage",        &gui_handler_images);
  MolaViz::register_gui_handler("mrpt::obs::CObservationGPS",          &gui_handler_gps);
  MolaViz::register_gui_handler("mrpt::obs::CObservationIMU",          &gui_handler_imu);
  MolaViz::register_gui_handler("mrpt::obs::CObservationPointCloud",   &gui_handler_point_cloud);
  MolaViz::register_gui_handler("mrpt::obs::CObservation3DRangeScan",  &gui_handler_point_cloud);
  MolaViz::register_gui_handler("mrpt::obs::CObservation3DRangeScan",  &gui_handler_images);
  MolaViz::register_gui_handler("mrpt::obs::CObservation2DRangeScan",  &gui_handler_point_cloud);
  MolaViz::register_gui_handler("mrpt::obs::CObservationRotatingScan", &gui_handler_point_cloud);
  MolaViz::register_gui_handler("mrpt::obs::CObservationVelodyneScan", &gui_handler_point_cloud);
  // clang-format on
}

// ---------------------------------------------------------------------------
// Static members
// ---------------------------------------------------------------------------

MolaViz*                     MolaViz::instance_ = nullptr;
std::shared_mutex            MolaViz::instanceMtx_;
const MolaViz::window_name_t MolaViz::DEFAULT_WINDOW_NAME = "main";

// ---------------------------------------------------------------------------
// Handler registry
// ---------------------------------------------------------------------------

void MolaViz::register_gui_handler(const class_name_t& name, const update_handler_t& handler)
{
  auto& hc  = HandlersContainer::Instance();
  auto  lck = mrpt::lockHelper(hc.guiHandlersMtx_);
  hc.guiHandlers_.emplace(name, handler);
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

MolaViz::MolaViz() = default;

MolaViz::~MolaViz()
{
  instanceMtx_.lock();
  instance_ = nullptr;
  instanceMtx_.unlock();

  nanogui::leave();
  if (guiThread_.joinable())
  {
    guiThread_.join();
  }
}

// ---------------------------------------------------------------------------
// Instance management
// ---------------------------------------------------------------------------

bool     MolaViz::IsRunning() { return Instance() != nullptr; }
MolaViz* MolaViz::Instance()
{
  instanceMtx_.lock_shared();
  auto* ret = instance_;
  instanceMtx_.unlock_shared();
  return ret;
}

// ---------------------------------------------------------------------------
// Backend identity
// ---------------------------------------------------------------------------

const std::string& MolaViz::gui_backend() const noexcept { return VizInterface::BACKEND_NANOGUI; }

// ---------------------------------------------------------------------------
// initialize / spinOnce
// ---------------------------------------------------------------------------

void MolaViz::initialize(const Yaml& c)
{
  MRPT_START

  auto cfg = c["params"];
  MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

  YAML_LOAD_MEMBER_OPT(max_console_lines, unsigned int);
  YAML_LOAD_MEMBER_OPT(console_text_font_size, double);
  YAML_LOAD_MEMBER_OPT(show_rgbd_as_point_cloud, bool);
  YAML_LOAD_MEMBER_OPT(assumed_sensor_rate_hz, double);

  instanceMtx_.lock();
  instance_ = this;
  instanceMtx_.unlock();

  guiThread_ = std::thread(&MolaViz::gui_thread, this);

  MRPT_END
}

void MolaViz::spinOnce()
{
  const double PERIOD_CHECK_NEW_MODS    = 2.0;
  const double PERIOD_UPDATE_DATASET_UI = 0.25;

  const double tNow = mrpt::Clock::nowDouble();
  if (tNow - lastTimeCheckForNewModules_ > PERIOD_CHECK_NEW_MODS)
  {
    dataset_ui_check_new_modules();
    lastTimeCheckForNewModules_ = tNow;
  }
  if (tNow - lastTimeUpdateDatasetUIs_ > PERIOD_UPDATE_DATASET_UI)
  {
    dataset_ui_update();
    lastTimeUpdateDatasetUIs_ = tNow;
  }
}

// ---------------------------------------------------------------------------
// Dataset UI - ported to create_subwindow_from_description
// ---------------------------------------------------------------------------

void MolaViz::dataset_ui_check_new_modules()
{
  auto datasetUIs = findService<Dataset_UI>();

  for (auto& module : datasetUIs)
  {
    const auto modUI = std::dynamic_pointer_cast<Dataset_UI>(module);
    ASSERT_(modUI);

    auto& e = datasetUIs_[module->getModuleInstanceName()];
    if (!e.first_time_seen)
    {
      continue;
    }

    e.first_time_seen = false;
    e.module          = modUI;

    // Allocate LiveStrings that will be updated by dataset_ui_update():
    e.lbPlaybackPosition = std::make_shared<mola::gui::LiveString>("Progress: ");

    // Build rates combobox items and find initial selection index:
    const std::vector<float>       rates  = {0.1, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0};
    const std::vector<std::string> labels = {"x0.1",  "x0.25", "x0.5", "x0.75", "x1.0",
                                             "x1.25", "x1.5",  "x2.0", "x3.0",  "x5.0"};
    int                            selIdx = 4;
    const double                   initialRate = modUI->datasetUI_playback_speed();
    for (size_t i = 0; i < rates.size(); i++)
    {
      if (rates[i] == static_cast<float>(initialRate))
      {
        selIdx = static_cast<int>(i);
        break;
      }
    }

    // Capture weak_ptr copies for callbacks (avoids storing dangling refs):
    std::weak_ptr<Dataset_UI> weakMod = modUI;

    // ---- Window description ----
    mola::gui::WindowDescription desc;
    desc.title         = module->getModuleInstanceName();
    desc.position      = {300, 5};
    desc.size          = {650, 75};
    desc.starts_hidden = false;

    mola::gui::Tab tab{"Controls", {}};

    // Single horizontal row: [Paused CB] [Progress label] [Slider] [Rate label] [ComboBox]
    mola::gui::Row row;

    row.widgets.emplace_back(mola::gui::CheckBox{
        "Paused", modUI->datasetUI_paused(),
        [weakMod](bool checked)
        {
          if (auto mod = weakMod.lock())
          {
            mod->datasetUI_paused(checked);
          }
        }});

    row.widgets.emplace_back(mola::gui::Label{e.lbPlaybackPosition, 0, 100});

    // Slider - uses a dedicated SliderFloat spanning dataset range.
    // Initial max will be overwritten by dataset_ui_update() on the first tick.
    row.widgets.emplace_back(mola::gui::SliderFloat{
        "", static_cast<float>(modUI->datasetUI_lastQueriedTimestep()), 0.0f,
        static_cast<float>(std::max<size_t>(1u, modUI->datasetUI_size())), "%.0f",
        [weakMod](float pos)
        {
          if (auto mod = weakMod.lock())
          {
            mod->datasetUI_teleport(static_cast<size_t>(pos));
          }
        },
        270});

    row.widgets.emplace_back(
        mola::gui::Label{std::make_shared<mola::gui::LiveString>("Playback rate:")});

    row.widgets.emplace_back(mola::gui::ComboBox{
        "", labels, selIdx,
        [weakMod, rates](int idx)
        {
          if (auto mod = weakMod.lock())
          {
            mod->datasetUI_playback_speed(static_cast<double>(rates.at(idx)));
          }
        }});

    tab.widgets.emplace_back(std::move(row));
    desc.tabs.emplace_back(std::move(tab));

    // Fire and forget - we don't need to .get() since dataset_ui_update()
    // only uses LiveStrings (no retained widget pointers needed).
    create_subwindow_from_description(desc);

    markWindowForReLayout(DEFAULT_WINDOW_NAME);
  }
}

void MolaViz::dataset_ui_update()
{
  for (auto& kv : datasetUIs_)
  {
    auto& e = kv.second;
    if (e.module.expired())
    {
      continue;
    }

    auto mod = e.module.lock();
    if (!mod)
    {
      continue;
    }

    const size_t pos = mod->datasetUI_lastQueriedTimestep();
    const size_t N   = mod->datasetUI_size();

    // Update the label via LiveString - thread-safe, picked up next frame:
    if (e.lbPlaybackPosition)
    {
      e.lbPlaybackPosition->set(mrpt::format("%zu / %zu", pos, N));
    }
  }
}

// ---------------------------------------------------------------------------
// GUI thread
// ---------------------------------------------------------------------------

mrpt::gui::CDisplayWindowGUI::Ptr MolaViz::create_and_add_window(const window_name_t& name)
{
  using namespace std::string_literals;

  MRPT_LOG_DEBUG_FMT("Creating new window `%s`", name.c_str());

  mrpt::gui::CDisplayWindowGUI_Params cp;
  cp.maximized   = true;
  windows_[name] = {
      mrpt::gui::CDisplayWindowGUI::Create("MOLAViz - "s + name, 1000, 800, cp), {}, {}};

  {
    std::unique_lock<std::shared_mutex> lck(subWindowsMtx_);
    subWindows_[name];
  }

  auto& win = windows_[name].win;

  win->setIconFromData(mola_icon_data, mola_icon_width, mola_icon_height, 0xff);

  auto scene = mrpt::opengl::COpenGLScene::Create();
  {
    std::lock_guard<std::mutex> lck(win->background_scene_mtx);
    win->background_scene = std::move(scene);
  }

  win->performLayout();
  auto& cam = win->camera();
  cam.setCameraPointing(8.0f, .0f, .0f);
  cam.setAzimuthDegrees(110.0f);
  cam.setElevationDegrees(15.0f);
  cam.setZoomDistance(20.0f);
  win->drawAll();
  win->setVisible(true);

  return win;
}

void MolaViz::gui_thread()
{
  MRPT_LOG_DEBUG("gui_thread() started.");
  mrpt::system::thread_name("MolaViz::gui_thread");

  nanogui::init();

  auto w = create_and_add_window(DEFAULT_WINDOW_NAME);

  w->setLoopCallback(
      [this]()
      {
        ProfilerEntry pe(profiler_, "loopCallback lambda");

        // Drain the task queue:
        task_queue_t tasks;
        {
          auto lck               = mrpt::lockHelper(guiThreadPendingTasksMtx_);
          tasks                  = std::move(guiThreadPendingTasks_);
          guiThreadPendingTasks_ = task_queue_t();
          auto winsToReLayout    = guiThreadMustReLayoutTheseWindows_;
          guiThreadMustReLayoutTheseWindows_.clear();

          // Unlock before running tasks to avoid re-entrant deadlock:
          lck.unlock();

          auto& hc          = HandlersContainer::Instance();
          auto  lckHandlers = mrpt::lockHelper(hc.guiHandlersMtx_);
          for (auto& t : tasks)
          {
            try
            {
              t();
            }
            catch (const std::exception& e)
            {
              MRPT_LOG_ERROR_STREAM("Exception in task sent to GUI thread:\n" << e.what());
            }
          }
          lckHandlers.unlock();

          for (const auto& winName : winsToReLayout)
          {
            windows_.at(winName).win->performLayout();
          }
        }

        // Poll all LiveStrings registered with managed subwindows:
        poll_live_strings_in_subwindows_();

        // Handle decaying point clouds:
        internal_handle_decaying_clouds();
      });

  nanogui::mainloop(25 /*ms*/);
  nanogui::shutdown();

  windows_.clear();
  subWindows_.clear();

  MRPT_LOG_DEBUG("gui_thread() quitted.");
}

// ---------------------------------------------------------------------------
// poll_live_strings_in_subwindows_ - called every GUI frame
// ---------------------------------------------------------------------------

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void MolaViz::poll_live_strings_in_subwindows_()
{
  // Walk every widget in every managed subwindow.  Widgets that were created
  // from a Label or TextPanel description carry an id tag of the form
  // "_live_<uint64_id>" encoding the registry key of their LiveString.
  // We decode that, call poll(), and update the caption / value if dirty.
  //
  // This is O(total widgets), but only the dirty ones do any real work.

  for (auto& [parentName, subMap] : subWindows_)
  {
    for (auto& [subName, subWin] : subMap)
    {
      if (subWin == nullptr)
      {
        continue;
      }

      // Walk all children recursively via a small lambda:
      std::function<void(nanogui::Widget*)> visit = [&](nanogui::Widget* widget)
      {
        const auto& id = widget->id();
        if (id.substr(0, 6) == "_live_")
        {
          // Decode the LiveString unique ID from the widget tag:
          const uint64_t lsId = std::stoull(id.substr(6));
          auto           it   = liveStringRegistry_.find(lsId);
          if (it != liveStringRegistry_.end())
          {
            std::string tmp;
            if (it->second->poll(tmp))
            {
              if (auto* lb = dynamic_cast<nanogui::Label*>(widget))
              {
                lb->setCaption(tmp);
              }
              if (auto* tb = dynamic_cast<nanogui::TextBox*>(widget))
              {
                tb->setValue(tmp);
              }
            }
          }
        }
        for (auto* child : widget->children())
        {
          visit(child);
        }
      };
      visit(subWin);
    }
  }
}

// ---------------------------------------------------------------------------
// VizInterface - new backend-agnostic API
// ---------------------------------------------------------------------------

/** Core implementation: enqueue a callable to run on the GUI thread. */
std::future<void> MolaViz::enqueue_custom_gui_code(const std::function<void()>& userCode)
{
  auto task = std::make_shared<std::packaged_task<void()>>([=]() { userCode(); });
  auto lck  = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

void* MolaViz::get_subwindow_handle(
    const std::string& subWindowTitle, const std::string& parentWindow)
{
  // This is called from arbitrary threads.  The map is only written from the
  // GUI thread, so we take a shared lock for the read here.
  std::shared_lock<std::shared_mutex> lck(subWindowsMtx_);

  auto itParent = subWindows_.find(parentWindow);
  if (itParent == subWindows_.end())
  {
    return nullptr;
  }
  auto itSub = itParent->second.find(subWindowTitle);
  if (itSub == itParent->second.end())
  {
    return nullptr;
  }
  return static_cast<void*>(itSub->second);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::future<void> MolaViz::create_subwindow_from_description(
    const mola::gui::WindowDescription& desc, const std::string& parentWindow)
{
  // Copy the description by value so the lambda owns it safely:
  return enqueue_custom_gui_code(
      [this, desc, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM("create_subwindow_from_description() title='" << desc.title << "'");

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);

        auto* subwin = topWin->createManagedSubWindow(desc.title);

        {
          std::unique_lock<std::shared_mutex> lck(subWindowsMtx_);
          subWindows_[parentWindow][desc.title] = subwin;
        }

        subwin->setVisible(!desc.starts_hidden);
        subwin->setPosition({desc.position[0], desc.position[1]});

        // Resize/enlarge buttons in the title-bar button panel:
        subwin->buttonPanel()
            ->add<nanogui::Button>("", ENTYPO_ICON_RESIZE_100_PERCENT)
            ->setCallback(
                [subwin, topWin]()
                {
                  if (subwin->children().size() > 1)
                  {
                    if (auto* gl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(
                            subwin->children().at(1));
                        gl)
                    {
                      auto s = gl->size();
                      s.x()  = mrpt::round(s.x() * 0.75);
                      s.y()  = mrpt::round(s.y() * 0.75);
                      gl->setSize(s);
                      gl->setFixedSize(s);
                    }
                  }
                  topWin->performLayout();
                });
        subwin->buttonPanel()
            ->add<nanogui::Button>("", ENTYPO_ICON_RESIZE_FULL_SCREEN)
            ->setCallback(
                [subwin, topWin]()
                {
                  if (subwin->children().size() > 1)
                  {
                    if (auto* gl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(
                            subwin->children().at(1));
                        gl)
                    {
                      auto s = gl->size();
                      s.x()  = mrpt::round(s.x() * 1.25);
                      s.y()  = mrpt::round(s.y() * 1.25);
                      gl->setSize(s);
                      gl->setFixedSize(s);
                    }
                  }
                  topWin->performLayout();
                });

        if (desc.tabs.empty())
        {
          // Bare subwindow (no widget description).  Set a default vertical
          // single-column GridLayout so that children added directly to the
          // window (e.g. MRPT2NanoguiGLCanvas added by gui handlers) are
          // positioned inside the subwindow frame rather than floating.
          // This matches the old subwindow_grid_layout(title, true, 1) call.
          subwin->setLayout(new nanogui::GridLayout(
              nanogui::Orientation::Vertical, 1, nanogui::Alignment::Fill, 2, 2));
        }
        else
        {
          // Apply size constraints only for description-based windows:
          if (desc.size[0] > 0)
          {
            subwin->setFixedWidth(desc.size[0]);
          }
          if (desc.size[1] > 0)
          {
            subwin->setFixedHeight(desc.size[1]);
          }

          if (desc.tabs.size() == 1)
          {
            // Single-tab optimisation: skip the TabWidget, render content
            // directly.  This saves vertical space for simple tool panels.
            subwin->setLayout(new nanogui::GroupLayout());
            build_tab_widgets(subwin, desc.tabs.front(), liveStringRegistry_);
          }
          else
          {
            subwin->setLayout(new nanogui::BoxLayout(
                nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
            auto* tabWidget = subwin->add<nanogui::TabWidget>();
            for (const auto& tab : desc.tabs)
            {
              auto* page = tabWidget->createTab(tab.title);
              page->setLayout(new nanogui::GroupLayout());
              build_tab_widgets(page, tab, liveStringRegistry_);
            }
            tabWidget->setActiveTab(0);
          }
        }

        markWindowForReLayout(parentWindow);
      });
}

std::future<std::optional<std::string>> MolaViz::open_file_dialog(
    [[maybe_unused]] const std::string& title, bool save,
    const std::vector<std::pair<std::string, std::string>>& filters,
    const std::string& default_path, [[maybe_unused]] const std::string& parentWindow)
{
  // nanogui::file_dialog() blocks the GUI thread while the OS dialog is open.
  // We wrap it in a packaged_task that resolves when the user dismisses it.
  // Callers must NOT call .get() from the GUI thread to avoid deadlock.

  using return_type = std::optional<std::string>;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [save, filters, default_path]() -> return_type
      {
        // nanogui::file_dialog takes vector<pair<string,string>>
        // where each pair is {extension, description}.
        // Our API uses {description, extensions} so we swap:
        std::vector<std::pair<std::string, std::string>> ngFilters;
        ngFilters.reserve(filters.size());
        for (const auto& [desc, exts] : filters)
        {
          // nanogui expects a single extension per entry; split on comma
          // and add one entry per extension:
          std::vector<std::string> extList;
          mrpt::system::tokenize(exts, ",", extList);
          for (const auto& ext : extList)
          {
            ngFilters.emplace_back(ext, desc);
          }
        }

        // Fall back to "all files" if no filters given:
        if (ngFilters.empty())
        {
          ngFilters.emplace_back("*", "All files");
        }

        auto result = nanogui::file_dialog(ngFilters, save);
        if (result.empty())
        {
          return std::nullopt;
        }
        return result;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// VizInterface - deprecated shims (delegate to new API, one line each)
// ---------------------------------------------------------------------------

std::future<nanogui::Window*> MolaViz::create_subwindow(
    const std::string& subWindowTitle, const std::string& parentWindow)
{
  // Build a minimal description that matches the legacy behaviour:
  // bare window, no tabs, no widgets - caller populates it via
  // enqueue_custom_nanogui_code() as before.
  mola::gui::WindowDescription desc;
  desc.title         = subWindowTitle;
  desc.starts_hidden = false;

  // Schedule creation and then return the raw pointer via a chained task.
  // We use a shared promise so the inner lambda can set its value after
  // create_subwindow_from_description has run.
  auto promise = std::make_shared<std::promise<nanogui::Window*>>();
  auto future  = promise->get_future();

  auto descFut = create_subwindow_from_description(desc, parentWindow);

  // Chain: once the description task completes, extract the raw pointer.
  enqueue_custom_gui_code(
      [this, subWindowTitle, parentWindow, p = std::move(promise)]() mutable
      {
        std::shared_lock<std::shared_mutex> lck(subWindowsMtx_);

        auto itParent = subWindows_.find(parentWindow);
        if (itParent != subWindows_.end())
        {
          auto itSub = itParent->second.find(subWindowTitle);
          if (itSub != itParent->second.end())
          {
            p->set_value(itSub->second);
            return;
          }
        }
        p->set_value(nullptr);
      });

  (void)descFut;
  return future;
}

std::future<void> MolaViz::enqueue_custom_nanogui_code(const std::function<void()>& userCode)
{
  return enqueue_custom_gui_code(userCode);
}

std::future<void> MolaViz::subwindow_grid_layout(
    const std::string& subWindowTitle, const bool orientationVertical, int resolution,
    const std::string& parentWindow)
{
  return enqueue_custom_gui_code(
      [this, subWindowTitle, orientationVertical, resolution, parentWindow]()
      {
        std::shared_lock<std::shared_mutex> lck(subWindowsMtx_);

        auto itWin = subWindows_.find(parentWindow);
        ASSERTMSG_(itWin != subWindows_.end(), "Unknown GUI top-level window");
        auto itSubWin = itWin->second.find(subWindowTitle);
        ASSERTMSG_(itSubWin != itWin->second.end(), "Unknown subwindow");

        itSubWin->second->setLayout(new nanogui::GridLayout(
            orientationVertical ? nanogui::Orientation::Vertical : nanogui::Orientation::Horizontal,
            resolution, nanogui::Alignment::Fill, 2, 2));
      });
}

std::future<void> MolaViz::subwindow_move_resize(
    const std::string& subWindowTitle, const mrpt::math::TPoint2D_<int>& location,
    const mrpt::math::TPoint2D_<int>& size, const std::string& parentWindow)
{
  return enqueue_custom_gui_code(
      [this, subWindowTitle, location, size, parentWindow]()
      {
        std::shared_lock<std::shared_mutex> lck(subWindowsMtx_);

        auto itWin = subWindows_.find(parentWindow);
        ASSERTMSG_(itWin != subWindows_.end(), "Unknown GUI top-level window");
        auto itSubWin = itWin->second.find(subWindowTitle);
        ASSERTMSG_(itSubWin != itWin->second.end(), "Unknown subwindow");

        itSubWin->second->setPosition({location.x, location.y});
        itSubWin->second->setSize({size.x, size.y});
      });
}

// ---------------------------------------------------------------------------
// Observation / RTTI handler dispatch
// ---------------------------------------------------------------------------

std::future<bool> MolaViz::subwindow_update_visualization(
    const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
    const mrpt::containers::yaml* extra_parameters, const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, obj, subWindowTitle, parentWindow, extra_parameters]()
      {
        try
        {
          const char* objClassName = obj->GetRuntimeClass()->className;
          MRPT_LOG_DEBUG_STREAM(
              "subwindow_update_visualization() title='" << subWindowTitle << "' class: '"
                                                         << objClassName << "'");

          std::shared_lock<std::shared_mutex> lck(subWindowsMtx_);

          ASSERTMSG_(
              subWindows_.count(parentWindow),
              mrpt::format("parentWindow not found: '%s'", parentWindow.c_str()));
          auto topWin = subWindows_.at(parentWindow);

          ASSERTMSG_(
              topWin.count(subWindowTitle),
              mrpt::format("subWindow not found: '%s'", subWindowTitle.c_str()));

          auto* subWin = topWin.at(subWindowTitle);
          ASSERT_(subWin != nullptr);

          auto& hc  = HandlersContainer::Instance();
          bool  any = false;
          for (auto [it, rangeEnd] = hc.guiHandlers_.equal_range(objClassName); it != rangeEnd;
               ++it)
          {
            it->second(obj, subWin, parentWindow, this, extra_parameters);
            any = true;
          }
          if (!any)
          {
            MRPT_LOG_DEBUG_STREAM(
                "subwindow_update_visualization(): no handler for '" << objClassName << "'");
          }
          return any;
        }
        catch (const std::exception& e)
        {
          MRPT_LOG_ERROR_STREAM("subwindow_update_visualization(): exception:\n" << e.what());
          return false;
        }
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// 3D scene API
// ---------------------------------------------------------------------------

std::future<bool> MolaViz::update_3d_object(
    const std::string& objName, const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
    const std::string& viewportName, const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, objName, obj, viewportName, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM("update_3d_object() objName='" << objName << "'");

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);
        ASSERT_(topWin->background_scene);

        mrpt::opengl::CSetOfObjects::Ptr glContainer;
        if (auto o = topWin->background_scene->getByName(objName, viewportName); o)
        {
          glContainer = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o);
          ASSERT_(glContainer);
        }
        else
        {
          glContainer = mrpt::opengl::CSetOfObjects::Create();
          topWin->background_scene->insert(glContainer, viewportName);
        }

        *glContainer = *obj;
        glContainer->setName(objName);
        return true;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

std::future<bool> MolaViz::insert_point_cloud_with_decay(
    const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud,
    const double decay_time_seconds, const std::string& viewportName,
    const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, cloud, decay_time_seconds, viewportName, parentWindow]()
      {
        if (!cloud || cloud->empty())
        {
          return true;
        }

        ASSERT_(windows_.count(parentWindow));
        auto& winData = windows_.at(parentWindow);
        auto  topWin  = winData.win;
        ASSERT_(topWin);
        ASSERT_(topWin->background_scene);

        mrpt::opengl::CSetOfObjects::Ptr glContainer;
        if (auto o = topWin->background_scene->getByName(DECAY_CLOUDS_NAME, viewportName); o)
        {
          glContainer = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o);
          ASSERT_(glContainer);
        }
        else
        {
          glContainer = mrpt::opengl::CSetOfObjects::Create();
          topWin->background_scene->insert(glContainer, viewportName);
          glContainer->setName(DECAY_CLOUDS_NAME);
        }

        glContainer->insert(cloud);

        const size_t maxScans = std::max<size_t>(
            1u, static_cast<size_t>(std::round(decay_time_seconds * assumed_sensor_rate_hz_)));
        winData.max_decaying_clouds = maxScans;

        const float initial_alpha = mrpt::u8tof(cloud->shaderPointsVertexColorBuffer().at(0).A);
        winData.decaying_clouds.emplace_back(viewportName, cloud, initial_alpha);

        while (winData.decaying_clouds.size() > maxScans)
        {
          auto& oldest = winData.decaying_clouds.front();
          glContainer->removeObject(oldest.cloud);
          winData.decaying_clouds.pop_front();
        }
        return true;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

std::future<bool> MolaViz::clear_all_point_clouds_with_decay(
    const std::string& viewportName, const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, viewportName, parentWindow]()
      {
        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);
        ASSERT_(topWin->background_scene);

        mrpt::opengl::CSetOfObjects::Ptr glContainer;
        if (auto o = topWin->background_scene->getByName(DECAY_CLOUDS_NAME, viewportName); o)
        {
          glContainer = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o);
          ASSERT_(glContainer);
        }
        else
        {
          glContainer = mrpt::opengl::CSetOfObjects::Create();
          topWin->background_scene->insert(glContainer, viewportName);
          glContainer->setName(DECAY_CLOUDS_NAME);
        }

        glContainer->clear();
        windows_.at(parentWindow).decaying_clouds.clear();
        return true;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

// ---------------------------------------------------------------------------
// Viewport / camera API
// ---------------------------------------------------------------------------

std::future<bool> MolaViz::update_viewport_look_at(
    const mrpt::math::TPoint3Df& lookAt, [[maybe_unused]] const std::string& viewportName,
    const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, lookAt, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM("update_viewport_look_at() lookAt=" << lookAt.asString());
        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);
        ASSERT_(topWin->background_scene);
        topWin->camera().setCameraPointing(lookAt.x, lookAt.y, lookAt.z);
        return true;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

std::future<bool> MolaViz::update_viewport_camera_azimuth(
    const double azimuth, bool absolute_falseForRelative,
    [[maybe_unused]] const std::string& viewportName,
    [[maybe_unused]] const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, azimuth, absolute_falseForRelative, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM("update_viewport_camera_azimuth() azimuth=" << azimuth);
        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);
        ASSERT_(topWin->background_scene);
        if (absolute_falseForRelative)
        {
          topWin->camera().setAzimuthDegrees(static_cast<float>(mrpt::RAD2DEG(azimuth)));
        }
        else
        {
          topWin->camera().setAzimuthDegrees(
              static_cast<float>(mrpt::RAD2DEG(azimuth)) + topWin->camera().getAzimuthDegrees());
        }
        return true;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

std::future<bool> MolaViz::update_viewport_camera_orthographic(
    const bool orthographic, [[maybe_unused]] const std::string& viewportName,
    const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, orthographic, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM(
            "update_viewport_camera_orthographic() orthographic=" << orthographic);
        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);
        ASSERT_(topWin->background_scene);
        topWin->camera().setCameraProjective(!orthographic);
        return true;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

std::future<bool> MolaViz::execute_custom_code_on_background_scene(
    const std::function<void(mrpt::opengl::Scene&)>& userCode, const std::string& parentWindow)
{
  using return_type = bool;

  const auto userCodeCopy = userCode;
  auto       task         = std::make_shared<std::packaged_task<return_type()>>(
      [this, userCodeCopy, parentWindow]()
      {
        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);
        ASSERT_(topWin->background_scene);
        try
        {
          userCodeCopy(*topWin->background_scene);
          return true;
        }
        catch (const std::exception& e)
        {
          MRPT_LOG_ERROR_STREAM(
                            "Exception in execute_custom_code_on_background_scene():\n"
                            << e.what());
          return false;
        }
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// Console output
// ---------------------------------------------------------------------------

std::future<bool> MolaViz::output_console_message(
    const std::string& message, const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, message, parentWindow]()
      {
        ASSERT_(windows_.count(parentWindow));
        auto& winData = windows_.at(parentWindow);

        std::vector<std::string> lines;
        mrpt::system::tokenize(message, "\r\n", lines);
        for (const auto& msg : lines)
        {
          winData.console_messages.push_back(msg);
          while (winData.console_messages.size() > max_console_lines_)
          {
            winData.console_messages.erase(winData.console_messages.begin());
          }
        }

        mrpt::gui::CDisplayWindowGUI::Ptr topWin = winData.win;
        ASSERT_(topWin);
        ASSERT_(topWin->background_scene);

        const double              LINE_HEIGHT  = console_text_font_size_;
        const double              LINE_SPACING = 3.0;
        mrpt::opengl::TFontParams fp;
        fp.vfont_scale = static_cast<float>(LINE_HEIGHT);

        for (size_t i = 0; i < winData.console_messages.size(); i++)
        {
          const size_t invIdx = (winData.console_messages.size() - 1 - i);
          fp.color.A          = 1.0f;
          if (invIdx > 1 && invIdx + 3 >= max_console_lines_)
          {
            fp.color.A = 1.0f - (static_cast<float>(invIdx) -
                                 (static_cast<float>(max_console_lines_) - 3.5f)) /
                                    3.5f;
          }
          topWin->background_scene->getViewport()->addTextMessage(
              3.0, LINE_SPACING + (LINE_SPACING + LINE_HEIGHT) * static_cast<float>(invIdx),
              winData.console_messages.at(i), i, fp);
        }
        return true;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

// ---------------------------------------------------------------------------
// Decaying clouds
// ---------------------------------------------------------------------------

void MolaViz::internal_handle_decaying_clouds()
{
  constexpr float FADE_OUT_FRACTION = 0.1f;

  for (auto& [winName, winData] : windows_)
  {
    const size_t queueSize = winData.decaying_clouds.size();
    if (queueSize == 0)
    {
      continue;
    }

    const size_t maxScans  = winData.max_decaying_clouds;
    const size_t fadeCount = std::max<size_t>(
        1u, static_cast<size_t>(std::round(static_cast<float>(maxScans) * FADE_OUT_FRACTION)));

    for (size_t i = 0; i < queueSize; i++)
    {
      auto&        dc  = winData.decaying_clouds[i];
      const size_t age = queueSize - 1u - i;

      float alpha = dc.initial_alpha;
      if (age >= (maxScans - fadeCount))
      {
        const float t =
            static_cast<float>(age - (maxScans - fadeCount)) / static_cast<float>(fadeCount);
        alpha = dc.initial_alpha * mrpt::saturate_val(1.0f - t, 0.0f, 1.0f);
      }
      dc.cloud->setAllPointsAlpha(mrpt::f2u8(alpha));
    }
  }
}

std::future<void> MolaViz::set_menu_bar(
    const mola::gui::MenuBar& /*bar*/, const std::string& /*parentWindow*/)
{
  // Note: nanogui backend does not support custom menu bars.
  MRPT_LOG_DEBUG_STREAM(
      "Ignoring request to add a menu bar since nanogui backend does not support menus.");

  std::promise<void> p;
  p.set_value();
  return p.get_future();
}
