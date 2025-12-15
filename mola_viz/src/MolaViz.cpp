/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
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
#include <mrpt/maps/CPointsMapXYZI.h>
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

#include "mola_icon_64x64.h"

using namespace mola;

IMPLEMENTS_MRPT_OBJECT(MolaViz, ExecutableBase, mola)

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

namespace
{

constexpr const char* DECAY_CLOUDS_NAME = "__viz_decaying_clouds";

template <typename Iter>
std::pair<Iter, Iter> minmax_ignore_nan(Iter begin, Iter end)
{
  // Find first non-NaN element
  Iter first = std::find_if(begin, end, [](auto x) { return !std::isnan(x); });

  if (first == end)
  {
    return {end, end};  // no valid elements
  }

  Iter itMin = first;
  Iter itMax = first;

  for (Iter it = std::next(first); it != end; ++it)
  {
    if (!std::isnan(*it))
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

void gui_handler_show_common_sensor_info(
    const mrpt::obs::CObservation& obs, nanogui::Window* w,
    const double sensor_rate_decimation = 1.0, const std::vector<std::string>& additionalMsgs = {})
{
  auto glControl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
  if (!glControl)
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

  // Computes the "y" coordinate of a text line by index
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

  // Estimate the sensor rate: one mean rate value stored per subwindow
  // (1 subwindow = 1 sensor stream)
  thread_local std::map<nanogui::Window*, double> estimatedHzs;
  thread_local std::map<nanogui::Window*, double> lastTimestamp;

  const double curTim = mrpt::Clock::toDouble(obs.timestamp);
  if (lastTimestamp.count(w) == 0)
  {
    // first time: do nothing
  }
  else
  {
    const double At    = curTim - lastTimestamp[w];
    const double curHz = (At > 0 ? (1.0 / At) : 1.0) * sensor_rate_decimation;
    const double alpha = 0.9;

    double showHz;

    if (estimatedHzs.count(w) == 0)
    {
      estimatedHzs[w] = curHz;
      showHz          = curHz;
    }
    else
    {
      // low-pass filtering:
      double& estimatedHz = estimatedHzs[w];

      estimatedHz = alpha * estimatedHz + (1.0 - alpha) * curHz;
      showHz      = estimatedHz;
    }

    glView->addTextMessage(
        2, line_y(TXT_ID_RATE),
        mrpt::format("Rate: %7.03f Hz Class: %s", showHz, obs.GetRuntimeClass()->className),
        TXT_ID_RATE, fp);
  }
  // store for the next iter:
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
    // dont know how to handle this class?
    return;
  }

  mrpt::gui::MRPT2NanoguiGLCanvas* glControl;
  if (w->children().size() == 1)
  {
    // Guess window size:
    auto winW = static_cast<int>(imgToShow.getWidth());
    auto winH = static_cast<int>(imgToShow.getHeight());

    // Guess if we need to decimate subwindow size:
    while (winW > 512 || winH > 512)
    {
      winW /= 2;
      winH /= 2;
    }

    glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
    glControl->setSize({winW, winH});
    glControl->setFixedSize({winW, winH});

    auto lck = mrpt::lockHelper(glControl->scene_mtx);

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

// CObservationPointCloud
// CObservation2DRangeScan
// CObservation3DRangeScan
// CObservationRotatingScan
// CObservationVelodyneScan
void gui_handler_point_cloud(
    const mrpt::rtti::CObject::Ptr& o, nanogui::Window* w, const MolaViz::window_name_t& parentWin,
    MolaViz* instance, const mrpt::containers::yaml* extra_parameters)
{
  using namespace mrpt::obs;

  mrpt::gui::MRPT2NanoguiGLCanvas*            glControl;
  mrpt::opengl::CPointCloudColoured::Ptr      glPc;
  mrpt::opengl::CSetOfObjects::Ptr            glCornerRef, glCornerSensor;
  std::optional<mrpt::LockHelper<std::mutex>> lck;

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
  if (extra_parameters)
  {
    point_size   = extra_parameters->getOrDefault("point_size", point_size);
    color_from_z = extra_parameters->getOrDefault("color_from_z", color_from_z);
  }

  if (w->children().size() == 1)
  {
    // Create on first use:
    glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();

    lck.emplace(&glControl->scene_mtx);

    glControl->scene = mrpt::opengl::COpenGLScene::Create();

    glPc = mrpt::opengl::CPointCloudColoured::Create();
    glControl->scene->insert(glPc);

    glCornerRef    = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    glCornerSensor = mrpt::opengl::stock_objects::CornerXYZ(0.5f);
    glControl->scene->insert(glCornerRef);
    glControl->scene->insert(glCornerSensor);

    glPc->setPointSize(point_size);
    instance->markWindowForReLayout(parentWin);
  }
  else
  {
    // Reuse from past iterations:
    glControl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
    lck.emplace(&glControl->scene_mtx);

    glPc = glControl->scene->getByClass<mrpt::opengl::CPointCloudColoured>();

    glCornerRef    = glControl->scene->getByClass<mrpt::opengl::CSetOfObjects>(0);
    glCornerSensor = glControl->scene->getByClass<mrpt::opengl::CSetOfObjects>(1);
  }
  ASSERT_(glControl != nullptr);
  ASSERT_(glPc);
  ASSERT_(glCornerRef);
  ASSERT_(glCornerSensor);

  glPc->setPose(mrpt::poses::CPose3D::Identity());

  if (auto obs = std::dynamic_pointer_cast<CObservation>(o); obs)
  {
    mrpt::poses::CPose3D p;
    obs->getSensorPose(p);
    glCornerSensor->setPose(p);
  }

  if (auto objPc = std::dynamic_pointer_cast<CObservationPointCloud>(o); objPc)
  {
    objPc->load();
    if (!objPc->pointcloud)
    {
      return;
    }
    glPc->loadFromPointsMap(objPc->pointcloud.get());
    glPc->setPose(objPc->sensorPose);

    std::vector<std::string> additionalMsgs = {
        mrpt::format("Point count: %zu", objPc->pointcloud->size()),
        mrpt::format("Type: %s", objPc->pointcloud->GetRuntimeClass()->className),
    };

    // Collect optional stats:
#if MRPT_VERSION >= 0x020f00  // 2.15.0

    for (const auto& field : objPc->pointcloud->getPointFieldNames_float())
    {
      if (const auto* buf = objPc->pointcloud->getPointsBufferRef_float_field(field);
          buf && !buf->empty())
      {
        const auto [itMin, itMax] = minmax_ignore_nan(buf->begin(), buf->end());
        additionalMsgs.push_back(mrpt::format(
            "%.*s range: [%.02f,%.02f]", static_cast<int>(field.size()), field.data(), *itMin,
            *itMax));
      }
    }
    for (const auto& field : objPc->pointcloud->getPointFieldNames_uint16())
    {
      if (const auto* buf = objPc->pointcloud->getPointsBufferRef_uint_field(field);
          buf && !buf->empty())
      {
        const auto [itMin, itMax] = minmax_ignore_nan(buf->begin(), buf->end());
        additionalMsgs.push_back(mrpt::format(
            "%.*s range: [%hu,%hu]", static_cast<int>(field.size()), field.data(), *itMin, *itMax));
      }
    }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
    for (const auto& field : objPc->pointcloud->getPointFieldNames_double())
    {
      if (const auto* buf = objPc->pointcloud->getPointsBufferRef_double_field(field);
          buf && !buf->empty())
      {
        const auto [itMin, itMax] = minmax_ignore_nan(buf->begin(), buf->end());
        additionalMsgs.push_back(mrpt::format(
            "%.*s range: [%.02lf,%.02lf]", static_cast<int>(field.size()), field.data(), *itMin,
            *itMax));
      }
    }
#endif

#else
    if (const auto* Is = objPc->pointcloud->getPointsBufferRef_intensity(); Is && !Is->empty())
    {
      const auto [itMin, itMax] = std::minmax_element(Is->begin(), Is->end());
      additionalMsgs.push_back(mrpt::format("Intensity range: [%.02f,%.02f]", *itMin, *itMax));
    }
#endif
    gui_handler_show_common_sensor_info(*objPc, w, sensorDecimation, additionalMsgs);
  }
  else if (auto objRS = std::dynamic_pointer_cast<CObservationRotatingScan>(o); objRS)
  {
    objRS->load();
    glPc->clear();
    mrpt::math::TBoundingBoxf bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

    for (size_t r = 0; r < objRS->rowCount; r++)
    {
      for (size_t c = 0; c < objRS->columnCount; c++)
      {
        const auto range = objRS->rangeImage(r, c);
        if (!range)
        {
          continue;  // invalid pt
        }

        const auto& pt = objRS->organizedPoints(r, c);

        glPc->insertPoint({pt.x, pt.y, pt.z, 0, 0, 0});
        bbox.updateWithPoint(pt);
      }
    }
    glPc->recolorizeByCoordinate(bbox.min.z, bbox.max.z);

    gui_handler_show_common_sensor_info(*objRS, w, sensorDecimation);
  }
  else if (auto obj3D = std::dynamic_pointer_cast<CObservation3DRangeScan>(o);
           instance->show_rgbd_as_point_cloud_ && obj3D)
  {
    if (obj3D->hasPoints3D)
    {
      if (obj3D->points3D_isExternallyStored())
      {
        obj3D->load();
      }

      for (size_t i = 0; i < obj3D->points3D_x.size(); i++)
      {
        glPc->insertPoint(
            {obj3D->points3D_x[i], obj3D->points3D_y[i], obj3D->points3D_z[i], 0, 0, 0});
      }
    }
    else
    {
      obj3D->load();

      mrpt::obs::T3DPointsProjectionParams pp;
      pp.takeIntoAccountSensorPoseOnRobot = true;

      if (obj3D->hasRangeImage && obj3D->hasIntensityImage)
      {
        auto pointMapCol                = mrpt::maps::CColouredPointsMap::Create();
        pointMapCol->colorScheme.scheme = mrpt::maps::CColouredPointsMap::cmFromIntensityImage;

        obj3D->unprojectInto(*pointMapCol, pp);
        glPc->loadFromPointsMap(pointMapCol.get());
        color_from_z = false;
      }
      else
      {
        obj3D->unprojectInto(*glPc, pp);
      }
    }
    gui_handler_show_common_sensor_info(*obj3D, w, sensorDecimation);
  }
  else if (auto obj2D = std::dynamic_pointer_cast<CObservation2DRangeScan>(o); obj2D)
  {
    mrpt::maps::CSimplePointsMap auxMap;
    auxMap.insertObservationPtr(obj2D);
    glPc->loadFromPointsMap(&auxMap);

    gui_handler_show_common_sensor_info(*obj2D, w, sensorDecimation);
  }
  else if (auto objVel = std::dynamic_pointer_cast<CObservationVelodyneScan>(o); objVel)
  {
    if (objVel->point_cloud.size() == 0)
    {
      return;
    }

    mrpt::maps::CPointsMapXYZI pts;
    const auto&                pc = objVel->point_cloud;
    const size_t               N  = pc.size();
    pts.resize(N);
    for (size_t i = 0; i < N; i++)
    {
      pts.setPoint(i, pc.x[i], pc.y[i], pc.z[i]);
      pts.setPointIntensity(i, static_cast<float>(pc.intensity[i]) / 255.0f);
    }
    glPc->loadFromPointsMap(&pts);

    gui_handler_show_common_sensor_info(
        *objVel, w, sensorDecimation,
        {
            mrpt::format("Point count: %zu", N),
        });
  }
  else
  {
    return;
  }

  // viz options:
  if (color_from_z)
  {
    const auto bb = glPc->getBoundingBox();
    glPc->recolorizeByCoordinate(static_cast<float>(bb.min.z), static_cast<float>(bb.max.z));
  }
}

// CObservationGPS
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
  for (size_t i = 0; i < labels.size(); i++)
  {
    ASSERT_(labels[i]);
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

// CObservationIMU
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

  mrpt::gui::MRPT2NanoguiGLCanvas* glControl;
  // mrpt::opengl::CSetOfObjects::Ptr            glCornerRef, glCornerSensor;
  std::optional<mrpt::LockHelper<std::mutex>> lck;

  if (w->children().size() == 1)
  {
    // Create on first use:
    w->setLayout(new nanogui::GridLayout(
        nanogui::Orientation::Horizontal, 1, nanogui::Alignment::Fill, 2, 2));

    glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();

    lck.emplace(&glControl->scene_mtx);

    glControl->scene = mrpt::opengl::COpenGLScene::Create();

    // glCornerRef    = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    // glControl->scene->insert(glCornerRef);

    const int winW = 400, winH = 125;
    glControl->setSize({winW, winH});
    glControl->setFixedSize({winW, winH});

    instance->markWindowForReLayout(parentWin);
  }
  else
  {
    // Reuse from past iterations:
    glControl = dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
    lck.emplace(&glControl->scene_mtx);

    // glCornerRef
    // =glControl->scene->getByClass<mrpt::opengl::CSetOfObjects>(0);
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
    txts.push_back("omega=None");
  }

  if (obj->has(mrpt::obs::IMU_X_ACC))
  {
    txts.push_back(mrpt::format(
        "acc=(%7.04f,%7.04f,%7.04f)", obj->get(mrpt::obs::IMU_X_ACC),
        obj->get(mrpt::obs::IMU_Y_ACC), obj->get(mrpt::obs::IMU_Z_ACC)));
  }
  else
  {
    txts.push_back("acc=None");
  }

  gui_handler_show_common_sensor_info(*obj, w, sensorDecimation, txts);
}

}  // namespace

MRPT_INITIALIZER(do_register_MolaViz)
{
  // Register MOLA module:
  MOLA_REGISTER_MODULE(MolaViz);

  // Register GUI handlers for common sensor types:
  // clang-format off
    MolaViz::register_gui_handler("mrpt::obs::CObservationImage", &gui_handler_images);
    MolaViz::register_gui_handler("mrpt::obs::CObservationGPS", &gui_handler_gps);
    MolaViz::register_gui_handler("mrpt::obs::CObservationIMU", &gui_handler_imu);
    MolaViz::register_gui_handler("mrpt::obs::CObservationPointCloud",   &gui_handler_point_cloud);
    MolaViz::register_gui_handler("mrpt::obs::CObservation3DRangeScan",  &gui_handler_point_cloud);
    MolaViz::register_gui_handler("mrpt::obs::CObservation3DRangeScan",  &gui_handler_images);
    MolaViz::register_gui_handler("mrpt::obs::CObservation2DRangeScan",  &gui_handler_point_cloud);
    MolaViz::register_gui_handler("mrpt::obs::CObservationRotatingScan", &gui_handler_point_cloud);
    MolaViz::register_gui_handler("mrpt::obs::CObservationVelodyneScan", &gui_handler_point_cloud);
  // clang-format on
}

MolaViz*                     MolaViz::instance_ = nullptr;
std::shared_mutex            MolaViz::instanceMtx_;
const MolaViz::window_name_t MolaViz::DEFAULT_WINDOW_NAME = "main";

void MolaViz::register_gui_handler(const class_name_t& name, const update_handler_t& handler)
{
  auto& hc  = HandlersContainer::Instance();
  auto  lck = mrpt::lockHelper(hc.guiHandlersMtx_);
  hc.guiHandlers_.emplace(name, handler);
}

MolaViz::MolaViz() {}

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

bool     MolaViz::IsRunning() { return Instance() != nullptr; }
MolaViz* MolaViz::Instance()
{
  instanceMtx_.lock_shared();
  auto ret = instance_;
  instanceMtx_.unlock_shared();
  return ret;
}

void MolaViz::initialize(const Yaml& c)
{
  MRPT_START

  // Load:
  auto cfg = c["params"];
  MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

  YAML_LOAD_MEMBER_OPT(max_console_lines, unsigned int);
  YAML_LOAD_MEMBER_OPT(console_text_font_size, double);
  YAML_LOAD_MEMBER_OPT(show_rgbd_as_point_cloud, bool);

  // Mark as initialized and up:
  instanceMtx_.lock();
  instance_ = this;
  instanceMtx_.unlock();

  guiThread_ = std::thread(&MolaViz::gui_thread, this);

  MRPT_END
}

void MolaViz::spinOnce()
{
  // once every X seconds, check for modules with automatically generated
  // UI by this MolaViz:
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
      continue;  // an already known one
    }

    e.first_time_seen = false;

    e.module = modUI;

    // Create UI:
    auto fut = this->create_subwindow(module->getModuleInstanceName());
    e.ui     = fut.get();
    e.ui->requestFocus();
    e.ui->setVisible(true);
    e.ui->setPosition({300, 5});

    e.ui->setLayout(
        new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 5, 2));
    e.ui->setFixedWidth(650);
    e.ui->setFixedHeight(60);

    e.cbPaused = e.ui->add<nanogui::CheckBox>("Paused");
    e.cbPaused->setChecked(modUI->datasetUI_paused());
    e.cbPaused->setCallback(
        [e](bool checked)
        {
          auto mod = e.module.lock();
          if (mod)
          {
            mod->datasetUI_paused(checked);
          }
        });

    e.lbPlaybackPosition = e.ui->add<nanogui::Label>("Progress: ");
    e.lbPlaybackPosition->setFixedWidth(100);
    e.slider = e.ui->add<nanogui::Slider>();
    e.slider->setFixedWidth(300);
    e.slider->setCallback(
        [e](float pos)
        {
          auto mod = e.module.lock();
          if (mod)
          {
            mod->datasetUI_teleport(static_cast<size_t>(pos));
          }
        });

    e.ui->add<nanogui::Label>("Playback rate:");
    e.cmRate                       = e.ui->add<nanogui::ComboBox>();
    const std::vector<float> rates = {0.1, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0};
    e.cmRate->setItems(
        {"x0.1", "x0.25", "x0.5", "x0.75", "x1.0", "x1.25", "x1.5", "x2.0", "x3.0", "x5.0"});
    int          selIdx      = 4;
    const double initialRate = modUI->datasetUI_playback_speed();
    for (size_t i = 0; i < rates.size(); i++)
    {
      if (rates[i] == initialRate)
      {
        selIdx = static_cast<int>(i);
        break;
      }
    }
    e.cmRate->setSelectedIndex(selIdx);
    e.cmRate->setCallback(
        [rates, e](int idx)
        {
          const double rate = rates.at(idx);
          auto         mod  = e.module.lock();
          mod->datasetUI_playback_speed(rate);
        });

    markWindowForReLayout(DEFAULT_WINDOW_NAME);
  }
}

void MolaViz::dataset_ui_update()
{
  for (auto& kv : datasetUIs_)
  {
    this->enqueue_custom_nanogui_code(
        [&kv]()
        {
          auto& e = kv.second;  // lambda capture structured bind is >C++20
          if (e.module.expired())
          {
            return;
          }

          auto mod = e.module.lock();
          if (!mod)
          {
            return;
          }

          const size_t pos = mod->datasetUI_lastQueriedTimestep();
          const size_t N   = mod->datasetUI_size();

          e.lbPlaybackPosition->setCaption(mrpt::format("%zu / %zu", pos, N));
          e.slider->setRange(std::make_pair<float>(0, N));
          e.slider->setValue(static_cast<float>(pos));
          e.slider->setHighlightedRange(
              std::make_pair<float>(0.f, static_cast<float>(pos) / static_cast<float>(N)));
        });
  }
}

mrpt::gui::CDisplayWindowGUI::Ptr MolaViz::create_and_add_window(const window_name_t& name)
{
  using namespace std::string_literals;

  MRPT_LOG_DEBUG_FMT("Creating new window `%s`", name.c_str());

  mrpt::gui::CDisplayWindowGUI_Params cp;
  cp.maximized   = true;
  windows_[name] = {
      mrpt::gui::CDisplayWindowGUI::Create("MOLAViz - "s + name, 1000, 800, cp), {}, {}};

  // create empty list of subwindows too:
  subWindows_[name];

  auto& win = windows_[name].win;

  // Apply custom MOLA icon:
  win->setIconFromData(
      mola_icon_data, mola_icon_width, mola_icon_height, 0xff /*transparent color*/);

  // Add a background scene:
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

  // Open first GUI window:
  auto w = create_and_add_window(DEFAULT_WINDOW_NAME);

  // Tasks pending to be run before each refresh:
  w->setLoopCallback(
      [this]()
      {
        ProfilerEntry pe(profiler_, "loopCallback lambda");

        // Get a copy of the tasks:
        task_queue_t tasks;
        auto         lck       = mrpt::lockHelper(guiThreadPendingTasksMtx_);
        tasks                  = std::move(guiThreadPendingTasks_);
        guiThreadPendingTasks_ = task_queue_t();
        auto winsToReLayout    = guiThreadMustReLayoutTheseWindows_;
        guiThreadMustReLayoutTheseWindows_.clear();
        lck.unlock();

        // Run them:
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

        // Pending GUI re-layouts
        for (const auto& winName : winsToReLayout)
        {
          windows_.at(winName).win->performLayout();
        }

        // Handle decaying point clouds
        internal_handle_decaying_clouds();
      });

  // A call to "nanogui::leave()" is required to end the infinite loop
  // in mainloop:
  nanogui::mainloop(25 /*refresh milliseconds*/);

  // Tidy up:
  nanogui::shutdown();

  // Delete all OpenGL memory from this same thread:
  windows_.clear();
  subWindows_.clear();

  MRPT_LOG_DEBUG("gui_thread() quitted.");
}

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
              "subwindow_update_visualization() title='" << subWindowTitle << "' obj of class: '"
                                                         << objClassName << "'");

          // Get subwindow:
          ASSERTMSG_(
              subWindows_.count(parentWindow),
              mrpt::format("parentWindow not found: '%s'", parentWindow.c_str()));
          auto topWin = subWindows_.at(parentWindow);

          ASSERTMSG_(
              topWin.count(subWindowTitle),
              mrpt::format("subWindow not found: '%s'", subWindowTitle.c_str()));

          auto subWin = topWin.at(subWindowTitle);
          ASSERT_(subWin != nullptr);

          // Get object GUI handler:
          // (Note: guiHandlersMtx_ is already locked by main render thread calling me)
          auto& hc = HandlersContainer::Instance();

          bool any = false;
          for (auto [it, rangeEnd] = hc.guiHandlers_.equal_range(objClassName); it != rangeEnd;
               ++it)
          {
            // Update GUI with object:
            it->second(obj, subWin, parentWindow, this, extra_parameters);
            any = true;
          }
          if (any)
          {  // done
            return true;
          }

          // No handler for this class:
          MRPT_LOG_DEBUG_STREAM(
              "subwindow_update_visualization() No known handler for "
              "obj of "
              "class: '"
              << objClassName << "'");

          return false;
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

std::future<nanogui::Window*> MolaViz::create_subwindow(
    const std::string& subWindowTitle, const std::string& parentWindow)
{
  using return_type = nanogui::Window*;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, subWindowTitle, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM(
            "create_subwindow() title='" << subWindowTitle << "' inside toplevel '" << parentWindow
                                         << "'");

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);

        auto subwin = topWin->createManagedSubWindow(subWindowTitle);
        // add to list of subwindows too:
        subWindows_[parentWindow][subWindowTitle] = subwin;

        // Reduce size button:
        subwin->buttonPanel()
            ->add<nanogui::Button>("", ENTYPO_ICON_RESIZE_100_PERCENT)
            ->setCallback(
                [subwin, topWin]()
                {
                  if (auto glControl =
                          dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(subwin->children().at(1));
                      glControl)
                  {
                    auto s = glControl->size();
                    s.x()  = mrpt::round(s.x() * 0.75);
                    s.y()  = mrpt::round(s.y() * 0.75);
                    glControl->setSize(s);
                    glControl->setFixedSize(s);
                  }
                  topWin->performLayout();
                });

        // Enlarge button:
        subwin->buttonPanel()
            ->add<nanogui::Button>("", ENTYPO_ICON_RESIZE_FULL_SCREEN)
            ->setCallback(
                [subwin, topWin]()
                {
                  if (auto glControl =
                          dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(subwin->children().at(1));
                      glControl)
                  {
                    auto s = glControl->size();
                    s.x()  = mrpt::round(s.x() * 1.25);
                    s.y()  = mrpt::round(s.y() * 1.25);
                    glControl->setSize(s);
                    glControl->setFixedSize(s);
                  }

                  topWin->performLayout();
                });

        return subwin;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

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

        // No need to acquire the mutex, since this task will be run
        // in the proper moment in the proper thread:
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

        // Move user contents and container properties (pose, scale,
        // etc.) via the "operator=":
        *glContainer = *obj;

        // (except the name! which we need to re-use in the next call)
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
        MRPT_LOG_DEBUG_STREAM(
            "insert_point_cloud_with_decay() called with decay_time_seconds='" << decay_time_seconds
                                                                               << "'");

        if (!cloud || cloud->empty())
        {
          return true;
        }

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);

        // No need to acquire the mutex, since this task will be run
        // in the proper moment in the proper thread:
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

        // Insert into the gl viz container:
        glContainer->insert(cloud);

        // and in our own struct to make it fade out later on:
        // (Assumption: all points have same alpha)
        const auto initial_alpha = mrpt::u8tof(cloud->shaderPointsVertexColorBuffer().at(0).A);

        windows_.at(parentWindow)
            .decaying_clouds.emplace_back(
                viewportName, mrpt::Clock::now(), cloud, decay_time_seconds, initial_alpha);

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
        MRPT_LOG_DEBUG("clear_all_point_clouds_with_decay() called");

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);

        // No need to acquire the mutex, since this task will be run
        // in the proper moment in the proper thread:
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

        // Clear the container:
        glContainer->clear();

        // and our own struct:
        windows_.at(parentWindow).decaying_clouds.clear();

        return true;
      });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
  return task->get_future();
}

std::future<bool> MolaViz::update_viewport_look_at(
    const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName,
    const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, lookAt, viewportName, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM("update_viewport_look_at() lookAt=" << lookAt.asString());

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);

        // No need to acquire the mutex, since this task will be run
        // in the proper moment in the proper thread:
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
    const double azimuth, bool absolute_falseForRelative, const std::string& viewportName,
    const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, azimuth, absolute_falseForRelative, viewportName, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM(
            "update_viewport_camera_azimuth() azimuth=" << azimuth << " absolute_falseForRelative:"
                                                        << absolute_falseForRelative);

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);

        // No need to acquire the mutex, since this task will be run
        // in the proper moment in the proper thread:
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
    const bool orthographic, const std::string& viewportName, const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, orthographic, viewportName, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM(
            "update_viewport_camera_orthographic() orthographic=" << orthographic);

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);

        // No need to acquire the mutex, since this task will be run
        // in the proper moment in the proper thread:
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

  // Make a copy of the const ref object:
  const auto userCodeCopy = userCode;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, userCodeCopy, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM("execute_custom_code_on_background_scene()");

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow).win;
        ASSERT_(topWin);

        // No need to acquire the mutex, since this task will be run
        // in the proper moment in the proper thread:
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

std::future<bool> MolaViz::output_console_message(
    const std::string& message, const std::string& parentWindow)
{
  using return_type = bool;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, message, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM("output_console_message() msg=" << message);

        ASSERT_(windows_.count(parentWindow));
        auto& winData = windows_.at(parentWindow);

        // Split multiline messages:
        std::vector<std::string> lines;
        mrpt::system::tokenize(message, "\r\n", lines);
        for (const auto& msg : lines)
        {
          // Append msg:
          winData.console_messages.push_back(msg);
          // remove older ones:
          while (winData.console_messages.size() > max_console_lines_)
          {
            winData.console_messages.erase(winData.console_messages.begin());
          }
        }

        mrpt::gui::CDisplayWindowGUI::Ptr topWin = winData.win;
        ASSERT_(topWin);

        // No need to acquire the mutex, since this task will be run
        // in the proper moment in the proper thread:
        ASSERT_(topWin->background_scene);

        const double              LINE_HEIGHT  = console_text_font_size_;
        const double              LINE_SPACING = 3.0;
        mrpt::opengl::TFontParams fp;
        fp.vfont_scale = static_cast<float>(LINE_HEIGHT);

        for (size_t i = 0; i < winData.console_messages.size(); i++)
        {
          const size_t invIdx = (winData.console_messages.size() - 1 - i);

          fp.color.A = 1.0f;
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

std::future<void> MolaViz::enqueue_custom_nanogui_code(const std::function<void(void)>& userCode)
{
  using return_type = void;

  auto task = std::make_shared<std::packaged_task<return_type()>>([=]() { userCode(); });

  auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

#if 0
// Visualize GT:
if (1)
{
    auto vizMods = this->findService<mola::VizInterface>();
    ASSERTMSG_(!vizMods.empty(), "Could not find a running MolaViz module");

    auto viz = std::dynamic_pointer_cast<VizInterface>(vizMods.at(0));

    auto glObjs   = mrpt::opengl::CSetOfObjects::Create();
    auto glCorner = mrpt::opengl::stock_objects::CornerXYZSimple(2.0);
    glCorner->enableShowName();
    glCorner->setName("GT");
    glCorner->setPose(it->second);
    glObjs->insert(glCorner);

    viz->update_3d_object("ground_truth", glObjs);
}
#endif

std::future<void> MolaViz::subwindow_grid_layout(
    const std::string& subWindowTitle, const bool orientationVertical, int resolution,
    const std::string& parentWindow)
{
  return enqueue_custom_nanogui_code(
      [this, subWindowTitle, orientationVertical, resolution, parentWindow]()
      {
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
  return enqueue_custom_nanogui_code(
      [this, subWindowTitle, location, size, parentWindow]()
      {
        auto itWin = subWindows_.find(parentWindow);
        ASSERTMSG_(itWin != subWindows_.end(), "Unknown GUI top-level window");
        auto itSubWin = itWin->second.find(subWindowTitle);
        ASSERTMSG_(itSubWin != itWin->second.end(), "Unknown subwindow");

        itSubWin->second->setPosition({location.x, location.y});
        itSubWin->second->setSize({size.x, size.y});
      });
}

void MolaViz::internal_handle_decaying_clouds()
{
  // This will be always called from the OpenGL thread, so no need to worry about mutexes.

  constexpr float DECAY_FADE_OUT_TIME = 1.0f;

  const auto tNow = mrpt::Clock::now();

  for (auto& [winName, winData] : windows_)
  {
    for (auto it = winData.decaying_clouds.begin(); it != winData.decaying_clouds.end();)
    {
      auto& decay_cloud = *it;

      const float delta_time =
          static_cast<float>(mrpt::system::timeDifference(decay_cloud.insertion_stamp, tNow));

      const float threshold_time =
          static_cast<float>(decay_cloud.decay_time_seconds) - DECAY_FADE_OUT_TIME;

#if MRPT_VERSION >= 0x20e0c  // v2.14.12 (setAllPointsAlpha)
      if (delta_time > threshold_time && decay_cloud.decay_time_seconds > 0)
      {
        const auto  decay_time = static_cast<float>(decay_cloud.decay_time_seconds);
        const float new_alpha  = mrpt::saturate_val(
             decay_cloud.initial_alpha *
                 (1.0f - (decay_time - threshold_time) / DECAY_FADE_OUT_TIME),
             0.0f, 1.0f);
        decay_cloud.cloud->setAllPointsAlpha(mrpt::f2u8(new_alpha));
      }
#endif

      // clouds to be deleted?
      if (delta_time > static_cast<float>(decay_cloud.decay_time_seconds))
      {
        // Delete clouds from the actual GL container, otherwise they will keep consuming
        // rendering resources forever!
        mrpt::opengl::CSetOfObjects::Ptr glContainer;
        if (auto o = winData.win->background_scene->getByName(
                DECAY_CLOUDS_NAME, decay_cloud.opengl_viewport_name);
            o)
        {
          glContainer = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o);
        }
        if (glContainer)
        {
          glContainer->removeObject(it->cloud);
        }

        // and delete from this list:
        it = winData.decaying_clouds.erase(it);
      }
      else
      {
        ++it;
      }
    }  // end for each decaying cloud in this window

  }  // end for each window
}
