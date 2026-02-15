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
 * @file   VizInterface.h
 * @brief  Virtual visualization interface (see MolaViz)
 * @author Jose Luis Blanco Claraco
 * @date   Sep 9, 2020
 */
#pragma once

#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/rtti/CObject.h>

#include <future>
#include <memory>

// Fwrd decl to avoid pushing gui dependencies to all clients of mola-kernel
// Was: #include <mrpt/gui/CDisplayWindowGUI.h>  // nanogui
// clang-format off
namespace nanogui { class Window; }
// clang-format on

namespace mola
{
/** Virtual visualization interface (see MolaViz)
 *
 * \ingroup mola_kernel_interfaces_grp */
class VizInterface
{
 public:
  VizInterface() = default;
  virtual ~VizInterface();

  VizInterface(const VizInterface&)            = default;
  VizInterface& operator=(const VizInterface&) = default;
  VizInterface(VizInterface&&)                 = default;
  VizInterface& operator=(VizInterface&&)      = default;

  using Ptr = std::shared_ptr<VizInterface>;

  // ===============================
  // See class MolaViz for docs
  // ===============================

  virtual std::future<nanogui::Window*> create_subwindow(
      const std::string& title, const std::string& parentWindow = "main") = 0;

  virtual std::future<void> subwindow_grid_layout(
      const std::string& subWindowTitle, bool orientationVertical, int resolution,
      const std::string& parentWindow = "main") = 0;

  virtual std::future<void> subwindow_move_resize(
      const std::string& subWindowTitle, const mrpt::math::TPoint2D_<int>& location,
      const mrpt::math::TPoint2D_<int>& size, const std::string& parentWindow = "main") = 0;

  /** Updates the contents of a subwindow from a given object, typically a
   * mrpt::obs::CObservation, but custom handlers can be installed for
   * arbitrary classes.
   *
   * Depending on the object class RTTI, the corresponding handler is called.
   *
   * \return false if no handler is found for the given object.
   *
   * \sa mola::MolaViz
   */
  virtual std::future<bool> subwindow_update_visualization(
      const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
      const mrpt::containers::yaml* extra_parameters = nullptr,
      const std::string&            parentWindow     = "main") = 0;

  /**
   * \brief Updates or inserts a 3D object in the background scene of a specified window.
   *
   * This method schedules a task to update (or insert if not present) a named 3D object
   * (`objName`) in the background scene of the given parent window (`parentWindow`),
   * optionally within a specific viewport (`viewportName`). The object is provided as a
   * shared pointer to a `mrpt::opengl::CSetOfObjects`.
   *
   * The update is performed in the GUI thread. If the object already exists, its contents
   * and properties are updated by **copying** the shared pointers contained into `obj`.
   *
   * \param objName        The name of the 3D object to update or insert.
   * \param obj            Shared pointer to the 3D object (`CSetOfObjects`) to be displayed.
   * \param viewportName   The name of the viewport where the object should be placed.
   * \param parentWindow   The name of the parent window containing the background scene.
   * \return               A future<bool>, will return `true` when the task is run in its thread.
   */
  virtual std::future<bool> update_3d_object(
      const std::string& objName, const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
      const std::string& viewportName = "main", const std::string& parentWindow = "main") = 0;

  /**
   * @brief Inserts a temporary point cloud, which will be visible only during `decay_time_seconds`
   *
   * \param obj            Shared pointer to the cloud (`CPointCloudColoured`) to be added.
   * \param decay_time_seconds Seconds that the cloud will be visible before disappearing.
   * \param viewportName   The name of the viewport where the object should be placed.
   * \param parentWindow   The name of the parent window containing the background scene.
   * \return               A future<bool>, will return `true` when the task is run in its thread.
   */
  virtual std::future<bool> insert_point_cloud_with_decay(
      const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud, double decay_time_seconds,
      const std::string& viewportName = "main", const std::string& parentWindow = "main") = 0;

  /**
   * @brief Removes from the visualization all clouds inserted with insert_point_cloud_with_decay()
   *
   * \param viewportName   The name of the viewport where the object should be placed.
   * \param parentWindow   The name of the parent window containing the background scene.
   * \return               A future<bool>, will return `true` when the task is run in its thread.
   */
  virtual std::future<bool> clear_all_point_clouds_with_decay(
      const std::string& viewportName = "main", const std::string& parentWindow = "main") = 0;

  virtual std::future<bool> update_viewport_look_at(
      const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName = "main",
      const std::string& parentWindow = "main") = 0;

  virtual std::future<bool> update_viewport_camera_azimuth(
      double azimuth, bool absolute_falseForRelative = true,
      const std::string& viewportName = "main", const std::string& parentWindow = "main") = 0;

  virtual std::future<bool> update_viewport_camera_orthographic(
      bool orthographic, const std::string& viewportName = "main",
      const std::string& parentWindow = "main") = 0;

  /**
   * \brief Executes arbitrary user code on the 3D Scene in the background of the main window space.
   *
   * This can be used to modify the viewport, create new sub-viewports, etc.
   * \note The user-provided code will be executed in the main GUI thread, so mutexes must be used
   *       as needed.
   */
  virtual std::future<bool> execute_custom_code_on_background_scene(
      const std::function<void(mrpt::opengl::Scene&)>& userCode,
      const std::string&                               parentWindow = "main") = 0;

  virtual std::future<void> enqueue_custom_nanogui_code(
      const std::function<void(void)>& userCode) = 0;

  virtual std::future<bool> output_console_message(
      const std::string& msg, const std::string& parentWindow = "main") = 0;
};

}  // namespace mola
