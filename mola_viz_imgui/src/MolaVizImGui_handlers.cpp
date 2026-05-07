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
 * @file   MolaVizImGui_handlers.cpp
 * @brief  Sensor-observation GUI handlers for the Dear ImGui backend.
 *
 * Each handler renders its observation type into a named ImGui sub-window.
 *
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_viz_imgui/MolaVizImGui.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/maps/CColouredPointsMap.h>
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

#include <mutex>

using namespace mola;

// ---------------------------------------------------------------------------
// Helper: drain any stale OpenGL errors so MRPT's CHECK_OPENGL_ERROR()
// doesn't pick up errors left by ImGui or our texture upload code.
// ---------------------------------------------------------------------------
namespace
{

void drain_gl_errors()
{
  while (glGetError() != GL_NO_ERROR)
  {
  }
}

// ---------------------------------------------------------------------------
// Helper: derive a unique ImGui ID from the observation's sensorLabel and
// the parent window name, avoiding collisions when multiple sensors exist.
// ---------------------------------------------------------------------------

std::string window_id_for(const std::string& subWindowTitle, const char* suffix)
{
  // "<title>##<suffix>": the visible title is subWindowTitle; the suffix
  // only participates in ImGui's id hash, so two handlers targeting the
  // same title (e.g. image + point-cloud views on a 3DRangeScan) produce
  // distinct ImGui windows while both match the caller-provided title.
  return subWindowTitle + "##" + suffix;
}

// ---------------------------------------------------------------------------
// Helper: show common sensor metadata as ImGui::Text lines.
// ---------------------------------------------------------------------------

// Must only be called from the GUI thread.  The static maps below are not
// mutex-protected; thread safety relies on all callers running on the GUI thread.
void show_common_sensor_info(const mrpt::obs::CObservation& obs, const std::string& key)
{
  // Rate estimation — one low-pass filter per key:
  static std::map<std::string, double> lastTimestamp;
  static std::map<std::string, double> estimatedHz;

  const double     curTim = mrpt::Clock::toDouble(obs.timestamp);
  constexpr double alpha  = 0.9;

  double showHz = 0.0;
  if (lastTimestamp.count(key))
  {
    const double At    = curTim - lastTimestamp[key];
    const double curHz = At > 0.0 ? 1.0 / At : 0.0;
    auto&        est   = estimatedHz[key];
    est                = alpha * est + (1.0 - alpha) * curHz;
    showHz             = est;
  }
  lastTimestamp[key] = curTim;

  const auto msg = mrpt::format(
      "Sensor: %s | Rate: %.2f Hz | t=%s", obs.sensorLabel.c_str(), showHz,
      mrpt::system::dateTimeToString(obs.timestamp).c_str());

  ImGui::Text("%s", msg.c_str());
  // too verbose? ImGui::Text("Class: %s", , obs.GetRuntimeClass()->className);

  mrpt::poses::CPose3D sensorPose;
  obs.getSensorPose(sensorPose);
  ImGui::Text("Sensor pose: %s", sensorPose.asString().c_str());
}

// ---------------------------------------------------------------------------
// CObservationImage / CObservation3DRangeScan (intensity channel)
//
// Uses mrpt::imgui::CImGuiSceneView with Viewport::setImageView() to render
// the image as a textured quad inside an FBO, matching the nanogui backend.
// ---------------------------------------------------------------------------

struct ImageViewState
{
  mrpt::imgui::CImGuiSceneView sceneView;
  bool                         initialized = false;
};

void handler_images(
    const mrpt::rtti::CObject::Ptr& o, void* /*handle*/,
    const MolaVizImGui::window_name_t& /*parentWin*/, const std::string& subWindowTitle,
    MolaVizImGui* /*instance*/, const mrpt::containers::yaml* /*extra*/)
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

  const std::string winId = window_id_for(subWindowTitle, "image");

  // Per-window persistent state.  Cleared from the GUI thread on shutdown
  // (while the GL context is still current) via register_gui_cleanup, to
  // avoid ~CImGuiSceneView calling glDelete* on a dead context.
  static std::map<std::string, ImageViewState> stateMap;
  static std::once_flag                        cleanupReg;
  std::call_once(
      cleanupReg, []() { MolaVizImGui::register_gui_cleanup([]() { stateMap.clear(); }); });
  auto& st = stateMap[winId];

  if (!st.initialized)
  {
    auto scene = mrpt::opengl::COpenGLScene::Create();
    st.sceneView.setScene(scene);
    st.initialized = true;
  }

  // Update the viewport's image (MRPT handles texture upload internally):
  st.sceneView.scene()->getViewport()->setImageView(imgToShow);

  if (ImGui::Begin(winId.c_str()))
  {
    if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(o); obs)
      show_common_sensor_info(*obs, winId);

    const auto imgW = static_cast<int>(imgToShow.getWidth());
    const auto imgH = static_cast<int>(imgToShow.getHeight());
    ImGui::Text("Size: %dx%dx%d", imgW, imgH, imgToShow.channelCount());

    // Drain stale GL errors before MRPT rendering:
    drain_gl_errors();

    st.sceneView.render();
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Point cloud observations — render via mrpt::imgui::CImGuiSceneView
// ---------------------------------------------------------------------------

struct PointCloudViewState
{
  mrpt::imgui::CImGuiSceneView           sceneView;
  mrpt::opengl::CPointCloudColoured::Ptr glPc;
  mrpt::opengl::CSetOfObjects::Ptr       glCornerRef;
  mrpt::opengl::CSetOfObjects::Ptr       glCornerSensor;
  bool                                   initialized = false;
};

void handler_point_cloud(
    const mrpt::rtti::CObject::Ptr& o, void* /*handle*/,
    const MolaVizImGui::window_name_t& /*parentWin*/, const std::string& subWindowTitle,
    MolaVizImGui* instance, const mrpt::containers::yaml* extra)
{
  using namespace mrpt::obs;

  auto obs = std::dynamic_pointer_cast<CObservation>(o);
  if (!obs) return;

  // CObservation3DRangeScan is dual-registered (also for handler_images).
  // Skip the point-cloud view when the config flag is off so we don't
  // allocate GL state for a window that will never be shown.
  if (std::dynamic_pointer_cast<CObservation3DRangeScan>(o) &&
      (!instance || !instance->show_rgbd_as_point_cloud_))
  {
    return;
  }

  const std::string winId = window_id_for(subWindowTitle, "pointcloud");

  // Per-window persistent state.  Cleared on GUI-thread shutdown via
  // register_gui_cleanup so CImGuiSceneView's GL resources don't outlive
  // the context.
  static std::map<std::string, PointCloudViewState> stateMap;
  static std::once_flag                             cleanupReg;
  std::call_once(
      cleanupReg, []() { MolaVizImGui::register_gui_cleanup([]() { stateMap.clear(); }); });
  auto& st = stateMap[winId];

  if (!st.initialized)
  {
    auto scene        = mrpt::opengl::COpenGLScene::Create();
    st.glPc           = mrpt::opengl::CPointCloudColoured::Create();
    st.glCornerRef    = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    st.glCornerSensor = mrpt::opengl::stock_objects::CornerXYZ(0.5f);
    st.glPc->setPointSize(3.0f);
    scene->insert(st.glPc);
    scene->insert(st.glCornerRef);
    scene->insert(st.glCornerSensor);
    st.sceneView.setScene(scene);
    st.sceneView.setBackgroundColor(0.15f, 0.15f, 0.18f);
    st.sceneView.camera().setZoomDistance(20.0f);
    st.sceneView.camera().setAzimuthDegrees(-140.0f);
    st.sceneView.camera().setElevationDegrees(30.0f);
    st.initialized = true;
  }

  // Read extra parameters:
  bool  color_from_z = true;
  float point_size   = 3.0f;
  if (extra)
  {
    point_size   = extra->getOrDefault("point_size", point_size);
    color_from_z = extra->getOrDefault("color_from_z", color_from_z);
  }
  st.glPc->setPointSize(point_size);
  st.glPc->setPose(mrpt::poses::CPose3D::Identity());
  st.glPc->clear();

  // Set sensor pose on corner marker:
  {
    mrpt::poses::CPose3D p;
    obs->getSensorPose(p);
    st.glCornerSensor->setPose(p);
  }

  // Populate point cloud from observation:
  bool populated = false;

  if (auto objPc = std::dynamic_pointer_cast<CObservationPointCloud>(o); objPc)
  {
    objPc->load();
    if (objPc->pointcloud)
    {
      st.glPc->loadFromPointsMap(objPc->pointcloud.get());
      st.glPc->setPose(objPc->sensorPose);
      populated = true;
    }
  }
  else if (auto objRS = std::dynamic_pointer_cast<CObservationRotatingScan>(o); objRS)
  {
    objRS->load();
    mrpt::math::TBoundingBoxf bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
    for (size_t r = 0; r < objRS->rowCount; r++)
    {
      for (size_t c = 0; c < objRS->columnCount; c++)
      {
        if (objRS->rangeImage(r, c) == 0) continue;
        const auto& pt = objRS->organizedPoints(r, c);
        st.glPc->insertPoint({pt.x, pt.y, pt.z, 0, 0, 0});
        bbox.updateWithPoint(pt);
      }
    }
    st.glPc->recolorizeByCoordinate(bbox.min.z, bbox.max.z);
    color_from_z = false;
    populated    = true;
  }
  else if (auto obj3D = std::dynamic_pointer_cast<CObservation3DRangeScan>(o); obj3D)
  {
    if (instance->show_rgbd_as_point_cloud_)
    {
      obj3D->load();
      if (obj3D->hasPoints3D)
      {
        for (size_t i = 0; i < obj3D->points3D_x.size(); i++)
          st.glPc->insertPoint(
              {obj3D->points3D_x[i], obj3D->points3D_y[i], obj3D->points3D_z[i], 0, 0, 0});
      }
      else if (obj3D->hasRangeImage && obj3D->hasIntensityImage)
      {
        mrpt::obs::T3DPointsProjectionParams pp;
        pp.takeIntoAccountSensorPoseOnRobot = true;
        auto pointMapCol                    = mrpt::maps::CColouredPointsMap::Create();
        pointMapCol->colorScheme.scheme     = mrpt::maps::CColouredPointsMap::cmFromIntensityImage;
        obj3D->unprojectInto(*pointMapCol, pp);
        st.glPc->loadFromPointsMap(pointMapCol.get());
        color_from_z = false;
      }
      else if (obj3D->hasRangeImage)
      {
        mrpt::obs::T3DPointsProjectionParams pp;
        pp.takeIntoAccountSensorPoseOnRobot = true;
        obj3D->unprojectInto(*st.glPc, pp);
      }
      populated = true;
    }
  }
  else if (auto obj2D = std::dynamic_pointer_cast<CObservation2DRangeScan>(o); obj2D)
  {
    mrpt::maps::CSimplePointsMap auxMap;
    auxMap.insertObservationPtr(std::make_shared<CObservation2DRangeScan>(*obj2D));
    st.glPc->loadFromPointsMap(&auxMap);
    populated = true;
  }
  else if (auto objVel = std::dynamic_pointer_cast<CObservationVelodyneScan>(o); objVel)
  {
    if (objVel->point_cloud.size() > 0)
    {
      const auto&  pc = objVel->point_cloud;
      const size_t N  = pc.size();
      for (size_t i = 0; i < N; i++) st.glPc->insertPoint({pc.x[i], pc.y[i], pc.z[i], 0, 0, 0});
      populated = true;
    }
  }

  if (!populated) return;

  if (color_from_z)
  {
    const auto bb = st.glPc->getBoundingBox();
    st.glPc->recolorizeByCoordinate(static_cast<float>(bb.min.z), static_cast<float>(bb.max.z));
  }

  if (ImGui::Begin(winId.c_str()))
  {
    show_common_sensor_info(*obs, winId);

    if (auto objPc = std::dynamic_pointer_cast<CObservationPointCloud>(o);
        objPc && objPc->pointcloud)
      ImGui::Text(
          "Points: %zu  |  Type: %s", objPc->pointcloud->size(),
          objPc->pointcloud->GetRuntimeClass()->className);

    // Drain stale GL errors before MRPT rendering:
    drain_gl_errors();

    // Render the 3D scene view:
    st.sceneView.render();
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// CObservationGPS
// ---------------------------------------------------------------------------

void handler_gps(
    const mrpt::rtti::CObject::Ptr& o, void* /*handle*/,
    const MolaVizImGui::window_name_t& /*parentWin*/, const std::string& subWindowTitle,
    MolaVizImGui* /*instance*/, const mrpt::containers::yaml* /*extra*/)
{
  auto obj = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o);
  if (!obj) return;

  const std::string winId = window_id_for(subWindowTitle, "GPS");

  if (ImGui::Begin(winId.c_str()))
  {
    show_common_sensor_info(*obj, winId);

    if (auto* gga = obj->getMsgByClassPtr<mrpt::obs::gnss::Message_NMEA_GGA>(); gga)
    {
      ImGui::Text("Latitude:  %.6f deg", gga->fields.latitude_degrees);
      ImGui::Text("Longitude: %.6f deg", gga->fields.longitude_degrees);
      ImGui::Text("Altitude:  %.2f m", gga->fields.altitude_meters);
      ImGui::Text("HDOP:      %.2f", gga->fields.HDOP);
      ImGui::Text(
          "UTC: %02u:%02u:%05.2f", static_cast<unsigned>(gga->fields.UTCTime.hour),
          static_cast<unsigned>(gga->fields.UTCTime.minute), gga->fields.UTCTime.sec);
    }
    if (obj->covariance_enu.has_value())
    {
      const auto& cov = obj->covariance_enu.value();
      ImGui::Text(
          "sigma [m]: x=%.2f  y=%.2f  z=%.2f", std::sqrt(cov(0, 0)), std::sqrt(cov(1, 1)),
          std::sqrt(cov(2, 2)));
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// CObservationIMU
// ---------------------------------------------------------------------------

void handler_imu(
    const mrpt::rtti::CObject::Ptr& o, void* /*handle*/,
    const MolaVizImGui::window_name_t& /*parentWin*/, const std::string& subWindowTitle,
    MolaVizImGui* /*instance*/, const mrpt::containers::yaml* /*extra*/)
{
  auto obj = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(o);
  if (!obj) return;

  const std::string winId = window_id_for(subWindowTitle, "IMU");

  if (ImGui::Begin(winId.c_str()))
  {
    show_common_sensor_info(*obj, winId);

    if (obj->has(mrpt::obs::IMU_WX))
      ImGui::Text(
          "omega: (%.4f, %.4f, %.4f)", obj->get(mrpt::obs::IMU_WX), obj->get(mrpt::obs::IMU_WY),
          obj->get(mrpt::obs::IMU_WZ));
    else
      ImGui::TextDisabled("omega: N/A");

    if (obj->has(mrpt::obs::IMU_X_ACC))
      ImGui::Text(
          "accel: (%.4f, %.4f, %.4f)", obj->get(mrpt::obs::IMU_X_ACC),
          obj->get(mrpt::obs::IMU_Y_ACC), obj->get(mrpt::obs::IMU_Z_ACC));
    else
      ImGui::TextDisabled("accel: N/A");
  }
  ImGui::End();
}

}  // namespace

// ---------------------------------------------------------------------------
// Registration — called from MRPT_INITIALIZER in MolaVizImGui.cpp
// ---------------------------------------------------------------------------

void mola_viz_imgui_register_default_handlers()
{
  // clang-format off
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservationImage",        &handler_images);
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservation3DRangeScan",  &handler_images);
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservationGPS",          &handler_gps);
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservationIMU",          &handler_imu);
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservationPointCloud",   &handler_point_cloud);
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservation3DRangeScan",  &handler_point_cloud);
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservation2DRangeScan",  &handler_point_cloud);
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservationRotatingScan", &handler_point_cloud);
  MolaVizImGui::register_gui_handler("mrpt::obs::CObservationVelodyneScan", &handler_point_cloud);
  // clang-format on
}
