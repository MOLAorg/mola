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
 * @file   MolaVizImGui.h
 * @brief  Dear ImGui (docking branch) backend for the MOLA visualization API.
 *         MOLA module shell: owns the GLFW window + ImGui context + render
 *         thread, and delegates all VizInterface calls to MolaVizImGuiCore.
 * @author Jose Luis Blanco Claraco
 * @date   Mar 5th, 2026
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_viz_imgui/MolaVizImGuiCore.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mola
{

/** MOLA visualization module — Dear ImGui docking-branch backend.
 *
 * Wraps `MolaVizImGuiCore` with the MOLA `ExecutableBase` lifecycle (module
 * registry, `initialize()`, `spinOnce()`), owns the GLFW window and the
 * dedicated GUI thread, and provides the Dataset_UI control panels.
 *
 * For embedding the visualizer into a host application that already owns an
 * ImGui context, use `MolaVizImGuiCore` directly — this class is only needed
 * when running as a standalone MOLA module.
 *
 * \ingroup mola_viz_imgui_grp
 */
class MolaVizImGui : public ExecutableBase, public VizInterface
{
  DEFINE_MRPT_OBJECT(MolaVizImGui, mola)

 public:
  MolaVizImGui();
  ~MolaVizImGui() override;

  MolaVizImGui(const MolaVizImGui&)            = delete;
  MolaVizImGui& operator=(const MolaVizImGui&) = delete;
  MolaVizImGui(MolaVizImGui&&)                 = delete;
  MolaVizImGui& operator=(MolaVizImGui&&)      = delete;

  // ExecutableBase
  void initialize(const Yaml& cfg) override;
  void spinOnce() override;

  // =========================================================================
  /** @name mola-viz-imgui main API
   * @{ */

  using window_name_t    = MolaVizImGuiCore::window_name_t;
  using subwindow_name_t = MolaVizImGuiCore::subwindow_name_t;

  static const window_name_t DEFAULT_WINDOW_NAME;

  static bool          IsRunning();
  static MolaVizImGui* Instance();

  /** @} */

  // =========================================================================
  /** @name VizInterface — backend identity
   * @{ */

  [[nodiscard]] const std::string& gui_backend() const noexcept override
  {
    return core_.gui_backend();
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — sub-window API (delegated to core_)
   * @{ */

  std::future<void> create_subwindow_from_description(
      const mola::gui::WindowDescription& desc,
      const std::string&                  parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.create_subwindow_from_description(desc, parentWindow);
  }

  std::future<void> enqueue_custom_gui_code(const std::function<void()>& userCode) override
  {
    return core_.enqueue_custom_gui_code(userCode);
  }

  void* get_subwindow_handle(
      const std::string& subWindowTitle,
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.get_subwindow_handle(subWindowTitle, parentWindow);
  }

  std::future<std::optional<std::string>> open_file_dialog(
      const std::string& title, bool save,
      const std::vector<std::pair<std::string, std::string>>& filters      = {},
      const std::string&                                      default_path = "",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.open_file_dialog(title, save, filters, default_path, parentWindow);
  }

  std::future<void> set_menu_bar(
      const mola::gui::MenuBar& bar, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.set_menu_bar(bar, parentWindow);
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — 3-D scene API (delegated to core_)
   * @{ */

  std::future<bool> update_3d_object(
      const std::string& objName, const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.update_3d_object(objName, obj, viewportName, parentWindow);
  }

  std::future<bool> insert_point_cloud_with_decay(
      const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud, double decay_time_seconds,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.insert_point_cloud_with_decay(
        cloud, decay_time_seconds, viewportName, parentWindow);
  }

  std::future<bool> clear_all_point_clouds_with_decay(
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.clear_all_point_clouds_with_decay(viewportName, parentWindow);
  }

  std::future<bool> update_viewport_look_at(
      const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.update_viewport_look_at(lookAt, viewportName, parentWindow);
  }

  std::future<bool> update_viewport_camera_azimuth(
      double azimuth, bool absolute_falseForRelative = true,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.update_viewport_camera_azimuth(
        azimuth, absolute_falseForRelative, viewportName, parentWindow);
  }

  std::future<bool> update_viewport_camera_orthographic(
      bool orthographic, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.update_viewport_camera_orthographic(orthographic, viewportName, parentWindow);
  }

  std::future<bool> execute_custom_code_on_background_scene(
      const std::function<void(mrpt::opengl::Scene&)>& userCode,
      const std::string&                               parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.execute_custom_code_on_background_scene(userCode, parentWindow);
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — observation handler API (delegated to core_)
   * @{ */

  std::future<bool> subwindow_update_visualization(
      const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
      const mrpt::containers::yaml* extra_parameters = nullptr,
      const std::string&            parentWindow     = DEFAULT_WINDOW_NAME) override
  {
    return core_.subwindow_update_visualization(
        obj, subWindowTitle, extra_parameters, parentWindow);
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — console output (delegated to core_)
   * @{ */

  std::future<bool> output_console_message(
      const std::string& message, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.output_console_message(message, parentWindow);
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — deprecated nanogui-specific stubs
   * @{ */

  [[deprecated]] std::future<nanogui::Window*> create_subwindow(
      const std::string& title, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.create_subwindow(title, parentWindow);
  }

  [[deprecated]] std::future<void> enqueue_custom_nanogui_code(
      const std::function<void()>& userCode) override
  {
    return core_.enqueue_custom_nanogui_code(userCode);
  }

  [[deprecated]] std::future<void> subwindow_grid_layout(
      const std::string& subWindowTitle, bool orientationVertical, int resolution,
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.subwindow_grid_layout(
        subWindowTitle, orientationVertical, resolution, parentWindow);
  }

  [[deprecated]] std::future<void> subwindow_move_resize(
      const std::string& subWindowTitle, const mrpt::math::TPoint2D_<int>& location,
      const mrpt::math::TPoint2D_<int>& size,
      const std::string&                parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_.subwindow_move_resize(subWindowTitle, location, size, parentWindow);
  }

  /** @} */

  // =========================================================================
  /** @name Handler registry (forwarded to MolaVizImGuiCore statics)
   * @{ */

  using update_handler_t = MolaVizImGuiCore::update_handler_t;
  using class_name_t     = MolaVizImGuiCore::class_name_t;

  static void register_gui_handler(const class_name_t& name, const update_handler_t& handler)
  {
    MolaVizImGuiCore::register_gui_handler(name, handler);
  }

  static void register_gui_cleanup(const std::function<void()>& cleanup)
  {
    MolaVizImGuiCore::register_gui_cleanup(cleanup);
  }

  /** @} */

  // Module parameters are configured via YAML in initialize(); they live in core_.

 private:
  MolaVizImGuiCore core_;

  // ---------------------------------------------------------------------------
  // Singleton
  // ---------------------------------------------------------------------------
  static MolaVizImGui* instance_;
  static std::mutex    instanceMtx_;

  // ---------------------------------------------------------------------------
  // GUI thread
  // ---------------------------------------------------------------------------
  std::thread       guiThread_;
  std::atomic<bool> guiThreadShutdown_{false};
  ImGuiContext*     imgui_ctx_ = nullptr;

  void                             gui_thread();
  MolaVizImGuiCore::PerWindowData& create_and_add_window(const window_name_t& name);

  // ---------------------------------------------------------------------------
  // Dataset_UI auto-generated panels
  // ---------------------------------------------------------------------------
  double lastTimeCheckForNewModules_ = 0;
  double lastTimeUpdateDatasetUIs_   = 0;

  struct DataPerDatasetUI
  {
    std::weak_ptr<mola::Dataset_UI> module;
    bool                            first_time_seen = true;
    mola::gui::LiveString::Ptr      lbPlaybackPosition;
  };
  std::map<std::string, DataPerDatasetUI> datasetUIs_;

  void dataset_ui_check_new_modules();
  void dataset_ui_update();
};

}  // namespace mola
