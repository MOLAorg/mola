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
 * @file   MolaVizImGuiCore.h
 * @brief  Core rendering state for the Dear ImGui MOLA viz backend.
 *         Implements VizInterface; usable in host-owns-window or embed mode.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/system/COutputLogger.h>

#include <atomic>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mola
{

/** Core rendering and state-management logic for the Dear ImGui MOLA viz backend.
 *
 * Implements `VizInterface` so it can be used anywhere a `VizInterface::Ptr` is
 * expected without going through the full `MolaVizImGui` MOLA module.
 *
 * Two operating modes:
 *
 *  - **Host mode** (`MolaVizImGui`): a companion class creates a GLFW window and
 *    ImGui context on a dedicated thread and calls `render_frame()` each frame.
 *
 *  - **Embed mode**: the caller already owns the GLFW window and ImGui context.
 *    After calling `init_for_embed()` once (GL context current, no ImGui frame
 *    open), the caller invokes `spin_one_frame()` each frame between
 *    `ImGui::NewFrame()` and `ImGui::Render()`.  The 3-D background scene is
 *    NOT rendered by the core in this mode; the caller renders it via its own
 *    `CImGuiSceneView` using the scene returned by `get_background_scene()`.
 *
 * \ingroup mola_viz_imgui_grp
 */
class MolaVizImGuiCore : public VizInterface, public mrpt::system::COutputLogger
{
 public:
  MolaVizImGuiCore();
  ~MolaVizImGuiCore() override;

  MolaVizImGuiCore(const MolaVizImGuiCore&)            = delete;
  MolaVizImGuiCore& operator=(const MolaVizImGuiCore&) = delete;
  MolaVizImGuiCore(MolaVizImGuiCore&&)                 = delete;
  MolaVizImGuiCore& operator=(MolaVizImGuiCore&&)      = delete;

  using window_name_t    = std::string;
  using subwindow_name_t = std::string;

  static const window_name_t DEFAULT_WINDOW_NAME;

  // =========================================================================
  /** @name Lifecycle
   * @{ */

  /** Embed mode: heap-allocate a core and return it as a `VizInterface::Ptr`.
   *  Preferred over constructing on the stack/as a member, because MOLA
   *  modules (e.g. `mola_lidar_odometry`) consume a `VizInterface::Ptr` and
   *  the host must guarantee the instance outlives them.  The returned
   *  pointer can also be `dynamic_pointer_cast<MolaVizImGuiCore>` to reach
   *  embed-specific APIs.
   *
   *  Caller must still invoke `init_for_embed()` once the GL context is
   *  current, and `shutdown_for_embed()` while it is still current.
   */
  static std::shared_ptr<MolaVizImGuiCore> make_for_embed();

  /** Embed mode: register a virtual window (no GLFW window) keyed by @p name.
   *  Must be called once per window-name with the GL context current, before
   *  `spin_one_frame()`.  Calling twice with the same @p name is a no-op
   *  (returns without overwriting existing sub-windows / sensor windows).
   */
  void init_for_embed(const window_name_t& name = DEFAULT_WINDOW_NAME);

  /** Embed mode: release GL resources held by registered handlers and clear
   *  internal scene state.  MUST be called by the host while the GL context
   *  is still current, before destroying the ImGui/GL context.  Safe to call
   *  more than once.  If the destructor runs without a prior call, a warning
   *  is logged (GL state may leak / crash on shutdown).
   */
  void shutdown_for_embed();

  /** Host mode: called after a GLFW window + ImGui context are fully
   *  initialised.  Returns a reference to the new `PerWindowData` entry.
   *  @p win is the GLFW window that owns this context.
   */
  struct PerWindowData;
  PerWindowData& init_window(const window_name_t& name, GLFWwindow* win);

  /** Embed mode: drain the task queue and render all MOLA sub-windows into
   *  the caller's active ImGui frame.  Background 3-D scene rendering is
   *  skipped — the caller is responsible for that.
   *  Must be called between `ImGui::NewFrame()` and `ImGui::Render()`.
   */
  void spin_one_frame(const window_name_t& name = DEFAULT_WINDOW_NAME);

  /** Host mode: full per-frame render for a GLFW-owned window.  Calls
   *  `glfwMakeContextCurrent`, begins a new ImGui frame, runs all sub-renderers
   *  (background scene, sub-windows, sensor windows, console overlay), and
   *  finishes with `ImGui::Render` + `glfwSwapBuffers`.
   */
  void render_frame(const window_name_t& name, PerWindowData& wd);

  /** Expose the shared OpenGL scene written to by MOLA observation handlers.
   *  An external `CImGuiSceneView` (embed mode) can render it directly.
   *  Returns nullptr if @p name is not registered.
   *
   *  Thread-safety: must be called from the GUI thread (the same thread that
   *  calls `spin_one_frame()` / `render_frame()`).  Reads of the returned
   *  scene contents on the GUI thread must hold `get_background_scene_mutex()`
   *  for the duration of access — VizInterface writers also lock it.
   */
  std::shared_ptr<mrpt::opengl::COpenGLScene> get_background_scene(
      const window_name_t& name = DEFAULT_WINDOW_NAME);

  /** Returns the mutex guarding `get_background_scene()`.
   *  Returns nullptr if @p name is not registered.
   *  Same thread-safety note as `get_background_scene()`.
   */
  std::mutex* get_background_scene_mutex(const window_name_t& name = DEFAULT_WINDOW_NAME);

  /** @} */
  // =========================================================================
  /** @name VizInterface — backend identity
   * @{ */

  [[nodiscard]] const std::string& gui_backend() const noexcept override;

  /** @} */
  // =========================================================================
  /** @name VizInterface — sub-window API
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
  /** @name VizInterface — observation handler API
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
  /** @name VizInterface — deprecated nanogui stubs
   * @{ */

  [[deprecated]] std::future<nanogui::Window*> create_subwindow(
      const std::string& title, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  [[deprecated]] std::future<void> enqueue_custom_nanogui_code(
      const std::function<void()>& userCode) override;

  [[deprecated]] std::future<void> subwindow_grid_layout(
      const std::string& subWindowTitle, bool orientationVertical, int resolution,
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  [[deprecated]] std::future<void> subwindow_move_resize(
      const std::string& subWindowTitle, const mrpt::math::TPoint2D_<int>& location,
      const mrpt::math::TPoint2D_<int>& size,
      const std::string&                parentWindow = DEFAULT_WINDOW_NAME) override;

  /** @} */
  // =========================================================================
  /** @name GUI update handler registry
   * @{ */

  using update_handler_t = std::function<void(
      const mrpt::rtti::CObject::Ptr&, void* subWinHandle, const window_name_t& parentWin,
      const std::string& subWindowTitle, MolaVizImGuiCore* instance,
      const mrpt::containers::yaml* extra_parameters)>;
  using class_name_t     = std::string;

  static void register_gui_handler(const class_name_t& name, const update_handler_t& handler);

  static void register_gui_cleanup(const std::function<void()>& cleanup);

  /** Run all registered cleanup callbacks.  Must be called with the GL
   *  context current (releases FBOs, textures, VAOs).
   */
  static void run_registered_cleanups();

  /** @} */
  // =========================================================================
  /** @name Module parameters
   * @{ */

  double       console_text_font_size_   = 13.0;
  unsigned int max_console_lines_        = 12;
  bool         show_rgbd_as_point_cloud_ = false;
  double       assumed_sensor_rate_hz_   = 10.0;
  int          target_fps_               = 60;

  /** Identifier used to key the imgui .ini persistence file.
   *  Empty string disables persistence.  See `MolaVizImGui` docs.
   */
  std::string imgui_app_name_ = "default";

  /** @} */

  // =========================================================================
  // Internal data structures — public so MolaVizImGui can access them from
  // its gui_thread without extra indirection.
  // =========================================================================

  struct SubWindowState
  {
    mola::gui::WindowDescription desc;
    int                          active_tab  = 0;
    unsigned int                 fbo_id      = 0;
    unsigned int                 fbo_texture = 0;
    int                          fbo_w       = 0;
    int                          fbo_h       = 0;
  };

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
    GLFWwindow* glfw_window = nullptr;  // nullptr in embed mode

    std::shared_ptr<mrpt::opengl::COpenGLScene> background_scene;
    std::mutex                                  background_scene_mtx;

    /// nullptr in embed mode (host renders via its own CImGuiSceneView).
    std::unique_ptr<mrpt::imgui::CImGuiSceneView> background_scene_view;

    float cam_azimuth_deg   = 110.0f;
    float cam_elevation_deg = 15.0f;
    float cam_zoom          = 20.0f;
    float cam_look_at[3]    = {0.0f, 0.0f, 0.0f};
    bool  cam_orthographic  = false;
    bool  cam_dirty         = true;

    std::map<subwindow_name_t, SubWindowState> sub_windows;

    struct PendingSensorObs
    {
      mrpt::rtti::CObject::Ptr                      obj;
      std::shared_ptr<const mrpt::containers::yaml> extra;
      std::string                                   class_name;
    };
    std::map<subwindow_name_t, PendingSensorObs> sensor_windows;

    std::deque<std::string>   console_messages;
    std::deque<DecayingCloud> decaying_clouds;
    size_t                    max_decaying_clouds = 100;

    mola::gui::MenuBar menu_bar;
  };

  std::map<window_name_t, PerWindowData> windows_;

  using task_queue_t = std::vector<std::function<void()>>;
  task_queue_t guiThreadPendingTasks_;
  std::mutex   guiThreadPendingTasksMtx_;

  // imgui.ini path storage (member keeps string alive for ImGui's IniFilename pointer)
  std::map<window_name_t, std::string> imgui_ini_paths_;
  std::string resolve_imgui_ini_path(const window_name_t& windowName) const;

 private:
  // True between init_for_embed() and shutdown_for_embed(); used by the
  // destructor to warn the caller if they forgot to release GL resources.
  bool embed_active_ = false;

  // Per-frame rendering helpers
  void render_menu_bar(PerWindowData& win);
  void render_background_scene(PerWindowData& win);
  void render_subwindow(SubWindowState& sw);
  void render_sensor_windows(const window_name_t& parentName, PerWindowData& win);
  void render_console_overlay(PerWindowData& win);
  void render_widget_description(const mola::gui::WindowDescription& desc, SubWindowState& sw);
  void render_tab(const mola::gui::Tab& tab, const std::string& ctx);
  void render_any_widget(const mola::gui::AnyWidget& w, const std::string& ctx);
  void render_leaf_widget(const mola::gui::LeafWidget& w, const std::string& ctx);

  void internal_drain_task_queue();
  void internal_handle_decaying_clouds(PerWindowData& win);
};

}  // namespace mola
