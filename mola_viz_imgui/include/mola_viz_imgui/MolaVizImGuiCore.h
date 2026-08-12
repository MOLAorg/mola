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

#include <imgui.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/system/COutputLogger.h>

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

// Forward declarations: only pointers to these types appear in this header,
// so the GLFW/OpenGL/ImPlot headers are pulled in by the .cpp implementation
// files instead of being forced on every consumer of the public API.
struct GLFWwindow;
struct ImPlotContext;

namespace mola
{

// Defined in MetricsRegistry.h (kept out of the public include dir; only
// this Core uses the concrete type). The public handle modules interact
// with is the kernel's mola::MetricChannel.
class MetricsRegistry;

/** One captured log line, tagged with its originating module and severity. */
struct ConsoleLogEntry
{
  mrpt::Clock::time_point      timestamp;
  mrpt::system::VerbosityLevel level = mrpt::system::LVL_INFO;
  std::string                  source;  // module instance name
  std::string                  text;  // single line (already split on \n)
};

/** Thread-safe rolling store of captured log lines, bounded by entry count.
 *  Shared (shared_ptr) between MolaVizImGuiCore and the per-module logger
 *  callbacks, so it outlives any module still holding a callback. */
class ConsoleLogSink
{
 public:
  void                        push(const ConsoleLogEntry& e);
  std::deque<ConsoleLogEntry> snapshot() const;
  void                        clear();
  void                        note_source(const std::string& s);
  std::vector<std::string>    sources() const;

  /** Bumped on every push()/clear(). Lock-free, so callers that re-render
   *  every frame (e.g. the Console window) can skip the snapshot()/lock/copy
   *  entirely when nothing changed since their last cached copy. */
  uint64_t version() const { return version_.load(std::memory_order_relaxed); }

  std::atomic<size_t> max_entries{5000};
  std::atomic<double> window_seconds{120.0};

 private:
  mutable std::mutex          mtx_;
  std::deque<ConsoleLogEntry> entries_;
  std::set<std::string>       sources_;
  std::atomic<uint64_t>       version_{0};
};

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

  /** Name of the default (single) host window, as a compile-time literal.
   *  Every `DEFAULT_WINDOW_NAME` string object must be initialized from THIS,
   *  never copied from another one of them: those objects live in different
   *  translation units, whose dynamic-initialization order is unspecified, so
   *  a copy silently ends up empty depending on the link order.
   */
  static constexpr const char* DEFAULT_WINDOW_NAME_LITERAL = "main";

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

  /** Removes every object from a viewport of the background scene.
   *
   *  Unlike `update_3d_object()` (which only touches the single named object
   *  it is given) or `clear_all_point_clouds_with_decay()` (limited to the
   *  decay container), this is a full reset of the viewport contents.
   */
  std::future<bool> clear_background_scene(
      const std::string&   viewportName = "main",
      const window_name_t& parentWindow = DEFAULT_WINDOW_NAME);

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

  MetricChannel::Ptr register_metric(
      const std::string& name, const std::string& unit = "") override;

  void push_metric(const std::string& name, double t, double value) override;

  /** @} */
  // =========================================================================
  /** @name VizInterface — 3-D scene API
   * @{ */

  std::future<bool> update_3d_object(
      const std::string& objName, const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME,
      const std::string& parentFrame  = "") override;

  std::future<bool> update_3d_object_frame(
      const std::string& frameName, const mrpt::math::TPose3D& pose,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> insert_point_cloud_with_decay(
      const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud, double decay_time_seconds,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME,
      const std::string& parentFrame  = "") override;

  std::future<bool> clear_all_point_clouds_with_decay(
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> update_viewport_look_at(
      const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME,
      const std::string& parentFrame  = "") override;

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

  /** No-op on this backend: superseded by the dockable Console subwindow,
   *  which aggregates mrpt-logger output directly. */
  std::future<bool> output_console_message(
      const std::string& message, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

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

  /** Register a cleanup callback on this instance.  The callback is called
   *  (with the GL context current) during shutdown_for_embed() / destruction
   *  and should release any GL handles this instance allocated.  Multiple
   *  callbacks may be registered; they are called in registration order.
   */
  void register_gui_cleanup(const std::function<void()>& cleanup);

  /** Run all cleanup callbacks registered on this instance.
   *  Must be called with the GL context current.
   */
  void run_registered_cleanups();

  /** @} */
  // =========================================================================
  /** @name Module parameters
   * @{ */

  bool   show_rgbd_as_point_cloud_ = false;
  double assumed_sensor_rate_hz_   = 10.0;
  int    target_fps_               = 60;

  /** Identifier used to key the imgui .ini persistence file.
   *  Empty string disables persistence.  See `MolaVizImGui` docs.
   */
  std::string imgui_app_name_ = "default";

  /** Console subwindow: master enable. When false, no log interception and
   *  the Console window is not shown. */
  bool console_enabled_ = true;

  /** Max number of captured log entries kept in the rolling buffer. */
  unsigned int console_max_entries_ = 5000;

  /** Rolling time window (seconds): entries older than this are dropped. */
  double console_window_seconds_ = 120.0;

  /** Verbosity threshold used when registering module log callbacks (i.e.
   *  what gets captured pipeline-wide, as opposed to `console_level_enabled_`
   *  below, which only filters what the already-captured entries display).
   *  Default INFO: capturing DEBUG from every module forces every
   *  MRPT_LOG_DEBUG_STREAM() callsite in the whole pipeline to always format
   *  its message just to feed the sink, even if the UI ends up hiding it.
   *  Runtime-adjustable from the Console window's "Capture" combo; atomic
   *  because it's written from the GUI thread and read from the module's
   *  own spin thread (see `MolaVizImGui::console_check_new_modules()`). */
  std::atomic<mrpt::system::VerbosityLevel> console_capture_level_{mrpt::system::LVL_INFO};

  /** Sink shared with the per-module logger callbacks; always allocated,
   *  populated only while `console_enabled_` is true. */
  std::shared_ptr<ConsoleLogSink> console_sink_ = std::make_shared<ConsoleLogSink>();

  /** Top main menu bar (host mode only): master enable. When false, no menu
   *  bar is created at all (including the "MOLA" and built-in "View" menus,
   *  and any module-installed menu via `set_menu_bar()`). Existing
   *  plot/console windows are unaffected; only the top strip and its menus
   *  disappear. */
  bool menu_bar_enabled_ = true;

  /** Invoked when the user clicks "Quit" in the "MOLA" menu (host mode only).
   *  Set by `MolaVizImGui::initialize()` to request shutdown of the whole
   *  MOLA application. Left unset (no-op) outside of that context, e.g. when
   *  the Core is driven directly in embed mode. */
  std::function<void()> quit_callback_;

  /** Plot windows: master enable for the plot-window items in the "View" menu
   *  and metric registration. When false, register_metric()/push_metric()
   *  still return valid (no-op) channels so callers never need to guard the
   *  call. */
  bool plots_enabled_ = true;

  /** Default rolling-history cap (seconds) for a newly-registered channel. */
  double plots_default_retention_seconds_ = 10.0;

  /** Default horizontal span (seconds) of a newly-created plot window. */
  double plots_default_span_seconds_ = 5.0;

  /** Registry of metric channels shared with the per-module producer handles;
   *  always allocated, like `console_sink_`. See MetricsRegistry.h. */
  std::shared_ptr<MetricsRegistry> metrics_;

  /** @} */

  // =========================================================================
  // Internal data structures — public so MolaVizImGui can access them from
  // its gui_thread without extra indirection.
  // =========================================================================

  struct SubWindowState
  {
    mola::gui::WindowDescription desc;
    int                          active_tab  = 0;
    bool                         open        = true;
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
    mrpt::opengl::CSetOfObjects::Ptr container;  // owning container at insert time
    float                            initial_alpha = 1.0f;
  };

  /** One live plot window: which channels it overlays and its display options.
   *  Opened from the built-in "View" menu; independently closable/reopenable
   *  via its native `[x]` button and the same menu, mirroring how the Console
   *  window's visibility is toggled. */
  struct PlotWindowState
  {
    std::string title;  // e.g. "Plot 1"; unique within a PerWindowData
    bool        open = true;  // drives ImGui::Begin(title, &open) -> [x] button

    std::vector<std::string> channels;  // subscribed channel names (overlaid)

    float span_seconds = 5.0f;  // horizontal window shown, one of {1,2,5,10}
    bool  show_grid_x  = true;
    bool  show_grid_y  = true;
    bool  show_legend  = true;
    bool  lines        = true;  // true=solid lines, false=ticks/markers only
    bool  y_autoscale  = true;
    bool  paused       = false;  // freeze the view for inspection
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

    std::deque<DecayingCloud> decaying_clouds;
    size_t                    max_decaying_clouds = 100;

    mola::gui::MenuBar menu_bar;

    /** Drives the Console window's native `[x]` button; the "View" menu's
     *  checklist re-opens it, same mechanism as the plot windows below. */
    bool console_open = true;

    /** Open plot windows for this parent window; created via the "View"
     *  menu, each independently closable/reopenable. */
    std::vector<PlotWindowState> plot_windows;
    int                          next_plot_id = 1;

    /** Set once `render_frame()` has attempted the Console's default
     *  bottom-dock, so it only runs once per session. The attempt itself is
     *  a no-op if the Console already has a saved imgui.ini entry (its own
     *  or a user-moved one). */
    bool console_dock_defaulted = false;

    /** ID of the dock node currently acting as the "remaining" passthrough
     *  area available for further default-dock splits (Console, then any
     *  `dock_top_by_default` window). Splitting off a slice always updates
     *  this to the new remainder, since after a node is split its own ID
     *  becomes a non-leaf parent and can no longer be split directly. Zero
     *  until the dockspace node has been resolved for the first time. */
    ImGuiID dock_central_id = 0;

    /** Dock node reserved at the top of the main window, lazily created the
     *  first time a window without a saved imgui.ini entry requests
     *  `WindowDescription::dock_top_by_default` (e.g. a Dataset_UI panel).
     *  Zero until then. */
    ImGuiID default_dock_top_id = 0;
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

  // Embed mode only: the host owns the ImGui context but has no reason to
  // know about ImPlot, so the Core creates/destroys its own ImPlot context
  // here. Host mode (MolaVizImGui) instead owns an ImPlot context alongside
  // its ImGui context (see MolaVizImGui::gui_thread()).
  ImPlotContext* embed_implot_ctx_ = nullptr;

  // Per-frame rendering helpers
  void render_menu_bar(PerWindowData& win);
  void render_background_scene(PerWindowData& win);
  void render_subwindow(SubWindowState& sw);
  void render_sensor_windows(const window_name_t& parentName, PerWindowData& win);
  void render_console_window(PerWindowData& win);
  void render_app_menu();
  void render_view_menu(PerWindowData& win);
  void render_plot_windows(PerWindowData& win);
  void render_plot_toolbar(PlotWindowState& st);
  void render_widget_description(const mola::gui::WindowDescription& desc, SubWindowState& sw);
  void render_tab(const mola::gui::Tab& tab, const std::string& ctx);
  void render_any_widget(const mola::gui::AnyWidget& w, const std::string& ctx);
  void render_leaf_widget(const mola::gui::LeafWidget& w, const std::string& ctx);

  void internal_drain_task_queue();
  void internal_handle_decaying_clouds(PerWindowData& win);

  // Console window: filter test shared between on-screen rendering and Save.
  bool console_entry_passes_filters(const ConsoleLogEntry& e) const;
  void console_save_to_file();

  // Cleanup callbacks registered via register_gui_cleanup().
  std::vector<std::function<void()>> instance_cleanups_;

  // Per-instance widget state, keyed by "<ctx>##<label>" (or LiveString id for
  // text panels).  Kept on the instance — not as function-local statics — so
  // multiple cores in one process don't share widget state.
  std::map<std::string, bool>        widget_checkbox_states_;
  std::map<std::string, std::string> widget_text_buffers_;
  std::map<std::string, float>       widget_slider_float_vals_;
  std::map<std::string, int>         widget_slider_int_vals_;
  std::map<std::string, int>         widget_combo_indices_;
  std::map<uint64_t, std::string>    widget_textpanel_bufs_;

  // Console window UI state (per instance).
  std::string console_filter_text_;
  bool        console_level_enabled_[4] = {false, true, true, true};  // D,I,W,E
  bool        console_autoscroll_       = true;

  // Cached copy of the sink's entries, refreshed only when `ConsoleLogSink::
  // version()` changes -- avoids a full deque-of-strings copy under the
  // sink's mutex on every rendered frame when nothing new was logged.
  std::deque<ConsoleLogEntry> console_cached_entries_;
  uint64_t                    console_cached_version_ = 0;

  // Selected source filter, keyed by name (empty == "all"), not by an index
  // into `ConsoleLogSink::sources()` -- that set is sorted and its order
  // shifts as new modules register mid-session, so an index alone would
  // silently start pointing at a different source.
  std::string console_selected_source_name_;

  // Plot windows: buffers reused across channels/frames to avoid a
  // heap allocation per PlotLine() call.
  std::vector<double> plot_scratch_xs_;
  std::vector<double> plot_scratch_ys_;
};

}  // namespace mola
