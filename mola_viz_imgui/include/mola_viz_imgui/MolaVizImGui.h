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
#include <set>
#include <string>
#include <thread>

// Forward declaration: implot.h is only pulled in by the .cpp files that
// need it, so it is not forced on every consumer of this public header.
struct ImPlotContext;

namespace mola
{

/** MOLA visualization module — Dear ImGui docking-branch backend.
 *
 * Wraps `MolaVizImGuiCore` with the MOLA `ExecutableBase` lifecycle (module
 * registry, `initialize()`, `spinOnce()`), owns the GLFW window and the
 * dedicated GUI thread, and provides the Dataset_UI control panels.
 *
 * **Embed mode**: when a host application already owns an
 * ImGui context, call `MolaVizImGui::install_embed_core(core)` with its
 * pre-initialized `MolaVizImGuiCore` **before** calling
 * `MolaLauncherApp::setup()`.  This module will then skip GLFW/window
 * creation and use the provided core instead, so all VizInterface calls are
 * routed through the host's render loop.
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

  /** Embed mode: install a pre-initialized `MolaVizImGuiCore` that was
   *  created by the host application.  Must be called **before** the first
   *  `MolaLauncherApp::setup()` call that includes a `mola::MolaVizImGui`
   *  module.  The install is consumed (cleared) by the first
   *  `initialize()` that picks it up, so subsequent setups create a fresh
   *  GLFW-owned core as usual.
   */
  static void install_embed_core(std::shared_ptr<MolaVizImGuiCore> core);

  /** @} */

  // =========================================================================
  /** @name VizInterface — backend identity
   * @{ */

  [[nodiscard]] const std::string& gui_backend() const noexcept override
  {
    return core_ptr_->gui_backend();
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — sub-window API (delegated to core_ptr_)
   * @{ */

  std::future<void> create_subwindow_from_description(
      const mola::gui::WindowDescription& desc,
      const std::string&                  parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->create_subwindow_from_description(desc, parentWindow);
  }

  std::future<void> enqueue_custom_gui_code(const std::function<void()>& userCode) override
  {
    return core_ptr_->enqueue_custom_gui_code(userCode);
  }

  void* get_subwindow_handle(
      const std::string& subWindowTitle,
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->get_subwindow_handle(subWindowTitle, parentWindow);
  }

  std::future<std::optional<std::string>> open_file_dialog(
      const std::string& title, bool save,
      const std::vector<std::pair<std::string, std::string>>& filters      = {},
      const std::string&                                      default_path = "",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->open_file_dialog(title, save, filters, default_path, parentWindow);
  }

  std::future<void> set_menu_bar(
      const mola::gui::MenuBar& bar, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->set_menu_bar(bar, parentWindow);
  }

  MetricChannel::Ptr register_metric(const std::string& name, const std::string& unit = "") override
  {
    return core_ptr_->register_metric(name, unit);
  }

  void push_metric(const std::string& name, double t, double value) override
  {
    core_ptr_->push_metric(name, t, value);
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — 3-D scene API (delegated to core_ptr_)
   * @{ */

  std::future<bool> update_3d_object(
      const std::string& objName, const std::shared_ptr<mrpt::viz::CSetOfObjects>& obj,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME,
      const std::string& parentFrame  = "") override
  {
    return core_ptr_->update_3d_object(objName, obj, viewportName, parentWindow, parentFrame);
  }

  std::future<bool> update_3d_object_frame(
      const std::string& frameName, const mrpt::math::TPose3D& pose,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->update_3d_object_frame(frameName, pose, viewportName, parentWindow);
  }

  std::future<bool> insert_point_cloud_with_decay(
      const std::shared_ptr<mrpt::viz::CPointCloudColoured>& cloud, double decay_time_seconds,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME,
      const std::string& parentFrame  = "") override
  {
    return core_ptr_->insert_point_cloud_with_decay(
        cloud, decay_time_seconds, viewportName, parentWindow, parentFrame);
  }

  std::future<bool> clear_all_point_clouds_with_decay(
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->clear_all_point_clouds_with_decay(viewportName, parentWindow);
  }

  std::future<bool> update_viewport_look_at(
      const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME,
      const std::string& parentFrame  = "") override
  {
    return core_ptr_->update_viewport_look_at(lookAt, viewportName, parentWindow, parentFrame);
  }

  std::future<bool> update_viewport_camera_azimuth(
      double azimuth, bool absolute_falseForRelative = true,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->update_viewport_camera_azimuth(
        azimuth, absolute_falseForRelative, viewportName, parentWindow);
  }

  std::future<bool> update_viewport_camera_orthographic(
      bool orthographic, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->update_viewport_camera_orthographic(orthographic, viewportName, parentWindow);
  }

  std::future<bool> execute_custom_code_on_background_scene(
      const std::function<void(mrpt::viz::Scene&)>& userCode,
      const std::string&                            parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->execute_custom_code_on_background_scene(userCode, parentWindow);
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — observation handler API (delegated to core_ptr_)
   * @{ */

  std::future<bool> subwindow_update_visualization(
      const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
      const mrpt::containers::yaml* extra_parameters = nullptr,
      const std::string&            parentWindow     = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->subwindow_update_visualization(
        obj, subWindowTitle, extra_parameters, parentWindow);
  }

  /** @} */

  // =========================================================================
  /** @name VizInterface — console output (delegated to core_ptr_)
   * @{ */

  std::future<bool> output_console_message(
      const std::string& message, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override
  {
    return core_ptr_->output_console_message(message, parentWindow);
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

  void register_gui_cleanup(const std::function<void()>& cleanup)
  {
    core_ptr_->register_gui_cleanup(cleanup);
  }

  /** @} */

 private:
  // Active core — points to the host-provided instance (embed mode) or the
  // locally owned one (host mode).  Always non-null after initialize().
  std::shared_ptr<MolaVizImGuiCore> core_ptr_;

  bool embed_mode_ = false;

  // ---------------------------------------------------------------------------
  // Embed mode: one-shot install, consumed by initialize().
  // ---------------------------------------------------------------------------
  static std::shared_ptr<MolaVizImGuiCore> s_embed_core_;
  static std::mutex                        s_embed_core_mtx_;

  // ---------------------------------------------------------------------------
  // Singleton
  // ---------------------------------------------------------------------------
  static MolaVizImGui* instance_;
  static std::mutex    instanceMtx_;

  // ---------------------------------------------------------------------------
  // GUI thread (host mode only)
  // ---------------------------------------------------------------------------
  std::thread       guiThread_;
  std::atomic<bool> guiThreadShutdown_{false};
  ImGuiContext*     imgui_ctx_  = nullptr;
  ImPlotContext*    implot_ctx_ = nullptr;

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
    mola::gui::LiveFloat::Ptr       liveSliderPos;
  };
  std::map<std::string, DataPerDatasetUI> datasetUIs_;

  void dataset_ui_check_new_modules();
  void dataset_ui_update();

  // ---------------------------------------------------------------------------
  // Console window: log capture from all discovered ExecutableBase modules
  // ---------------------------------------------------------------------------
  double                lastTimeCheckForConsoleModules_ = 0;
  std::set<std::string> consoleHookedModules_;  // instance names already hooked
  // child_loggers() entries already hooked, keyed by logger identity (a
  // module may create/destroy these, e.g. a background-thread engine, so
  // they cannot be tracked by instance name alone).
  std::set<const mrpt::system::COutputLogger*> consoleHookedChildLoggers_;

  void console_check_new_modules();
};

}  // namespace mola
