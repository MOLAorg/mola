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
 * @brief  Dear ImGui (docking branch) backend for the MOLA visualization API
 * @author Jose Luis Blanco Claraco
 * @date   Mar 5th, 2026
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/opengl/COpenGLScene.h>

// Dear ImGui — included here because MolaVizImGui.h is only ever included by
// ImGui-aware translation units (the module itself + nanogui-free clients that
// explicitly opted in).  Clients that only use VizInterface::Ptr never see this.
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// GLFW (docking branch ImGui uses GLFW + OpenGL3 backend)
#include <GLFW/glfw3.h>

#include <atomic>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace mola
{

/** MOLA visualization — Dear ImGui docking-branch backend.
 *
 * Implements VizInterface using Dear ImGui + GLFW + OpenGL 3.
 * For the nanogui backend see MolaViz.
 *
 * Window management
 * -----------------
 * Each "parentWindow" maps to one GLFW window.  Sub-windows created via
 * create_subwindow_from_description() become ImGui windows rendered inside
 * the GLFW host; the ImGui docking system handles placement and persistence
 * via imgui.ini automatically.
 *
 * Threading model
 * ---------------
 * All ImGui and GLFW calls happen on guiThread_.  Other threads enqueue
 * tasks via the same guiThreadPendingTasks_ / guiThreadPendingTasksMtx_
 * mechanism as MolaViz.  LiveString polling is done inline every frame.
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

  using window_name_t    = std::string;
  using subwindow_name_t = std::string;

  static const window_name_t DEFAULT_WINDOW_NAME;

  static bool          IsRunning();
  static MolaVizImGui* Instance();

  /** @} */

  // =========================================================================
  /** @name VizInterface — backend identity
   * @{ */

  [[nodiscard]] const std::string& gui_backend() const noexcept override;

  /** @} */

  // =========================================================================
  /** @name VizInterface — backend-agnostic sub-window API
   * @{ */

  std::future<void> create_subwindow_from_description(
      const mola::gui::WindowDescription& desc,
      const std::string&                  parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<void> enqueue_custom_gui_code(const std::function<void()>& userCode) override;

  void* get_subwindow_handle(
      const std::string& subWindowTitle,
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<std::optional<std::string>> open_file_dialog(
      const std::string& title, bool save,
      const std::vector<std::pair<std::string, std::string>>& filters      = {},
      const std::string&                                      default_path = "",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<void> set_menu_bar(
      const mola::gui::MenuBar& bar,
      const std::string&        parentWindow = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name VizInterface — 3-D scene API
   * @{ */

  std::future<bool> update_3d_object(
      const std::string& objName, const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> insert_point_cloud_with_decay(
      const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud, double decay_time_seconds,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> clear_all_point_clouds_with_decay(
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> update_viewport_look_at(
      const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> update_viewport_camera_azimuth(
      double azimuth, bool absolute_falseForRelative = true,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> update_viewport_camera_orthographic(
      bool orthographic, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> execute_custom_code_on_background_scene(
      const std::function<void(mrpt::opengl::Scene&)>& userCode,
      const std::string&                               parentWindow = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name VizInterface — observation / RTTI handler API
   * @{ */

  std::future<bool> subwindow_update_visualization(
      const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
      const mrpt::containers::yaml* extra_parameters = nullptr,
      const std::string&            parentWindow     = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name VizInterface — console output
   * @{ */

  std::future<bool> output_console_message(
      const std::string& message, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name VizInterface — deprecated nanogui-specific stubs
   *
   * These compile cleanly and resolve futures immediately (no-ops or
   * delegates).  nullptr is returned for create_subwindow().
   * @{ */

  [[deprecated("Use create_subwindow_from_description() instead")]] std::future<nanogui::Window*>
      create_subwindow(
          const std::string& /*title*/,
          const std::string& /*parentWindow*/ = DEFAULT_WINDOW_NAME) override;

  [[deprecated("Use enqueue_custom_gui_code() instead")]] std::future<void>
      enqueue_custom_nanogui_code(const std::function<void()>& userCode) override;

  [[deprecated("Encode layout in WindowDescription instead")]] std::future<void>
      subwindow_grid_layout(
          const std::string& /*subWindowTitle*/, bool /*orientationVertical*/, int /*resolution*/,
          const std::string& /*parentWindow*/ = DEFAULT_WINDOW_NAME) override;

  [[deprecated("Encode position/size in WindowDescription instead")]] std::future<void>
      subwindow_move_resize(
          const std::string& /*subWindowTitle*/, const mrpt::math::TPoint2D_<int>& /*location*/,
          const mrpt::math::TPoint2D_<int>& /*size*/,
          const std::string& /*parentWindow*/ = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name GUI update handler registry (shared with MolaViz)
   * @{ */

  /// Sensor-observation rendering handler signature.
  /// subWin is nullptr for the ImGui backend — use subWindowTitle to key
  /// ImGui state instead.
  using update_handler_t = std::function<void(
      const mrpt::rtti::CObject::Ptr&, void* subWinHandle, const window_name_t& parentWin,
      const std::string& subWindowTitle, MolaVizImGui* instance,
      const mrpt::containers::yaml* extra_parameters)>;
  using class_name_t     = std::string;

  static void register_gui_handler(const class_name_t& name, const update_handler_t& handler);

  /// Register a callback that will be invoked on the GUI thread during
  /// shutdown, **with the GL context still current**, before any GLFW
  /// window is destroyed. Handlers use this to release GL resources held
  /// in function-local static state (FBOs, textures, VAOs in
  /// CImGuiSceneView, …) without risking calls on a dead context.
  static void register_gui_cleanup(const std::function<void()>& cleanup);

  /** @} */

  // =========================================================================
  /** @name Module parameters
   * @{ */

  double       console_text_font_size_   = 13.0;
  unsigned int max_console_lines_        = 12;
  bool         show_rgbd_as_point_cloud_ = false;
  double       assumed_sensor_rate_hz_   = 10.0;
  int          target_fps_               = 60;

  /** Identifier used to persist ImGui window layout / docking state across
   *  runs: each distinct value maps to its own `imgui_<app_name>.ini` file
   *  under `$XDG_CONFIG_HOME/mola/` (or `$HOME/.config/mola/`).  Set a
   *  different name per launch config (e.g. "kitti_replay", "live_lio")
   *  to keep layouts separate.  Empty string disables persistence.
   */
  std::string imgui_app_name_ = "default";

  /** @} */

 private:
  // ---------------------------------------------------------------------------
  // Singleton
  // ---------------------------------------------------------------------------
  static MolaVizImGui* instance_;
  static std::mutex    instanceMtx_;

  // ---------------------------------------------------------------------------
  // Per-subwindow ImGui state
  // ---------------------------------------------------------------------------

  /// State built from WindowDescription and kept alive for the lifetime of
  /// the ImGui window.  ImGui windows are identified by their title string.
  struct SubWindowState
  {
    mola::gui::WindowDescription desc;

    /// Per-tab active index (ImGui tracks it, but we need it for first-use):
    int active_tab = 0;

    /// FBO handle for sensor-observation GL canvas (0 = not yet created).
    /// TODO: per-subwindow FBO for subwindow_update_visualization().
    unsigned int fbo_id      = 0;
    unsigned int fbo_texture = 0;
    int          fbo_w       = 0;
    int          fbo_h       = 0;
  };

  // ---------------------------------------------------------------------------
  // Per-host-window state
  // ---------------------------------------------------------------------------

  struct DecayingCloud
  {
    DecayingCloud() = default;
    DecayingCloud(
        std::string vp, const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud_,
        float alpha_)
        : viewport_name(std::move(vp)), cloud(cloud_), initial_alpha(alpha_)
    {
    }
    std::string                                        viewport_name;
    std::shared_ptr<mrpt::opengl::CPointCloudColoured> cloud;
    float                                              initial_alpha = 1.0f;
  };

  struct PerWindowData
  {
    GLFWwindow* glfw_window = nullptr;

    /// mrpt scene rendered in the background OpenGL viewport.
    std::shared_ptr<mrpt::opengl::COpenGLScene> background_scene;
    std::mutex                                  background_scene_mtx;

    /// Camera / FBO renderer for the background scene.
    /// Held by unique_ptr so we can release its GL resources (FBO/texture)
    /// explicitly while the GL context is still current, before
    /// glfwDestroyWindow(). CImGuiSceneView is non-copyable and non-movable.
    std::unique_ptr<mrpt::imgui::CImGuiSceneView> background_scene_view;

    /// Camera state mirrored from background_scene_view after each render.
    /// API calls (update_viewport_*) write here; render_background_scene
    /// pushes to the view on the first frame (cam_dirty=true) and reads
    /// back every frame so mouse interaction persists.
    float cam_azimuth_deg   = 110.0f;
    float cam_elevation_deg = 15.0f;
    float cam_zoom          = 20.0f;
    float cam_look_at[3]    = {0.0f, 0.0f, 0.0f};
    bool  cam_orthographic  = false;
    bool  cam_dirty         = true;  ///< push cam_* into view on next render

    /// Sub-windows registered for this host window.
    std::map<subwindow_name_t, SubWindowState> sub_windows;

    /// Latest observation stashed per sensor-subwindow title, updated by
    /// subwindow_update_visualization() and drawn every frame by
    /// render_sensor_windows() (ImGui is immediate-mode — we must re-draw
    /// every frame or the window disappears).
    struct PendingSensorObs
    {
      mrpt::rtti::CObject::Ptr                      obj;
      std::shared_ptr<const mrpt::containers::yaml> extra;
      std::string                                   class_name;
    };
    std::map<subwindow_name_t, PendingSensorObs> sensor_windows;

    /// Console overlay lines.
    std::deque<std::string> console_messages;

    /// Decaying point clouds.
    std::deque<DecayingCloud> decaying_clouds;
    size_t                    max_decaying_clouds = 100;

    /// Menu bar (empty = no menu bar rendered).
    mola::gui::MenuBar menu_bar;
  };

  std::map<window_name_t, PerWindowData> windows_;

  /// Resolved ImGui .ini path(s), one per GLFW host window.  Stored as a
  /// member because ImGui's `io.IniFilename` keeps the pointer (not a copy)
  /// for the lifetime of the context.
  std::map<window_name_t, std::string> imgui_ini_paths_;

  /// Resolve the .ini path for a given host window, based on
  /// `imgui_app_name_`.  Creates parent directories if needed.  Returns
  /// empty string if persistence is disabled (empty imgui_app_name_) or the
  /// directory couldn't be created.
  std::string resolve_imgui_ini_path(const window_name_t& windowName) const;

  // ---------------------------------------------------------------------------
  // GUI thread + task queue
  // ---------------------------------------------------------------------------

  std::thread       guiThread_;
  std::atomic<bool> guiThreadShutdown_{false};
  ImGuiContext*     imgui_ctx_ = nullptr;  // single context, owned by gui_thread
  void              gui_thread();

  using task_queue_t = std::vector<std::function<void()>>;
  task_queue_t guiThreadPendingTasks_;
  std::mutex   guiThreadPendingTasksMtx_;

  PerWindowData& create_and_add_window(const window_name_t& name);

  // ---------------------------------------------------------------------------
  // Per-frame rendering helpers (called from GUI thread)
  // ---------------------------------------------------------------------------

  void render_frame(const window_name_t& name, PerWindowData& win);
  void render_menu_bar(PerWindowData& win);
  void render_background_scene(PerWindowData& win);
  void render_subwindow(SubWindowState& sw);
  void render_sensor_windows(const window_name_t& parentName, PerWindowData& win);
  void render_console_overlay(PerWindowData& win);
  void render_widget_description(const mola::gui::WindowDescription& desc, SubWindowState& sw);
  void render_tab(const mola::gui::Tab& tab, const std::string& ctx);
  void render_any_widget(const mola::gui::AnyWidget& w, const std::string& ctx);
  void render_leaf_widget(const mola::gui::LeafWidget& w, const std::string& ctx);

  void internal_handle_decaying_clouds(PerWindowData& win);

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
