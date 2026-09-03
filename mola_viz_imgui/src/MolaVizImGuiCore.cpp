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
 * @file   MolaVizImGuiCore.cpp
 * @brief  Core VizInterface implementation: task queue, rendering helpers,
 *         scene management.  Works in both host-window and embed mode.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>
#include <implot.h>
#include <mola_viz_imgui/MolaVizImGuiCore.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "MetricsRegistry.h"

using namespace mola;

// ---------------------------------------------------------------------------
// Static constant
// ---------------------------------------------------------------------------

const MolaVizImGuiCore::window_name_t MolaVizImGuiCore::DEFAULT_WINDOW_NAME =
    MolaVizImGuiCore::DEFAULT_WINDOW_NAME_LITERAL;

// ---------------------------------------------------------------------------
// Handler registry
// ---------------------------------------------------------------------------

namespace
{
struct ImGuiHandlersContainer
{
  static ImGuiHandlersContainer& Instance()
  {
    static ImGuiHandlersContainer o;
    return o;
  }
  std::multimap<MolaVizImGuiCore::class_name_t, MolaVizImGuiCore::update_handler_t> handlers;
  std::mutex                                                                        mtx;

 private:
  ImGuiHandlersContainer() = default;
};

}  // namespace

void MolaVizImGuiCore::register_gui_handler(
    const class_name_t& name, const update_handler_t& handler)
{
  auto&           hc = ImGuiHandlersContainer::Instance();
  std::lock_guard lk(hc.mtx);
  hc.handlers.emplace(name, handler);
}

void MolaVizImGuiCore::register_gui_cleanup(const std::function<void()>& cleanup)
{
  instance_cleanups_.push_back(cleanup);
}

void MolaVizImGuiCore::run_registered_cleanups()
{
  for (auto& fn : instance_cleanups_)
  {
    try
    {
      fn();
    }
    catch (...)
    {
    }
  }
}

// ---------------------------------------------------------------------------
// Backend identity
// ---------------------------------------------------------------------------

const std::string& MolaVizImGuiCore::gui_backend() const noexcept
{
  return VizInterface::BACKEND_IMGUI;
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

MolaVizImGuiCore::MolaVizImGuiCore() : metrics_(std::make_shared<MetricsRegistry>())
{
  setLoggerName("MolaVizImGuiCore");
}

MolaVizImGuiCore::~MolaVizImGuiCore()
{
  if (embed_active_)
  {
    MRPT_LOG_WARN(
        "~MolaVizImGuiCore(): shutdown_for_embed() was never called; GL "
        "resources held by registered handlers may leak or crash at exit.");
  }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

std::shared_ptr<MolaVizImGuiCore> MolaVizImGuiCore::make_for_embed()
{
  return std::make_shared<MolaVizImGuiCore>();
}

void MolaVizImGuiCore::init_for_embed(const window_name_t& name)
{
  if (auto it = windows_.find(name); it != windows_.end())
  {
    MRPT_LOG_WARN_STREAM(
        "init_for_embed(): window '" << name << "' already registered; ignoring re-init.");
    return;
  }
  auto& wd            = windows_[name];
  wd.glfw_window      = nullptr;
  wd.background_scene = mrpt::viz::Scene::Create();
  // No background_scene_view — the embed host renders via its own CImGuiSceneView.
  embed_active_ = true;

  // The embed host owns the ImGui context but not ImPlot's; create our own,
  // once per Core instance, matching MolaVizImGui::gui_thread() in host mode.
  if (!embed_implot_ctx_) embed_implot_ctx_ = ImPlot::CreateContext();
}

void MolaVizImGuiCore::shutdown_for_embed()
{
  if (!embed_active_) return;

  // Discard any queued GUI tasks so we don't run user code against
  // half-torn-down state.  Destroying the packaged_tasks breaks their
  // promises, so callers blocked on the returned futures get a
  // broken_promise exception instead of hanging forever.
  {
    std::lock_guard lk(guiThreadPendingTasksMtx_);
    guiThreadPendingTasks_.clear();
  }

  // Release GL handles held by static state in handler files (CImGuiSceneView
  // FBOs, textures, VAOs).  Caller guarantees GL context is current.
  run_registered_cleanups();

  for (auto& [name, wd] : windows_)
  {
    wd.sensor_windows.clear();
    wd.decaying_clouds.clear();
    wd.background_scene.reset();
    wd.background_scene_view.reset();
  }
  windows_.clear();

  if (embed_implot_ctx_)
  {
    ImPlot::DestroyContext(embed_implot_ctx_);
    embed_implot_ctx_ = nullptr;
  }

  embed_active_ = false;
}

MolaVizImGuiCore::PerWindowData& MolaVizImGuiCore::init_window(
    const window_name_t& name, GLFWwindow* win)
{
  auto& wd                 = windows_[name];
  wd.glfw_window           = win;
  wd.background_scene      = mrpt::viz::Scene::Create();
  wd.background_scene_view = std::make_unique<mrpt::imgui::CImGuiSceneView>();
  return wd;
}

std::shared_ptr<mrpt::viz::Scene> MolaVizImGuiCore::get_background_scene(const window_name_t& name)
{
  auto it = windows_.find(name);
  if (it == windows_.end()) return {};
  return it->second.background_scene;
}

std::mutex* MolaVizImGuiCore::get_background_scene_mutex(const window_name_t& name)
{
  auto it = windows_.find(name);
  if (it == windows_.end()) return nullptr;
  return &it->second.background_scene_mtx;
}

// ---------------------------------------------------------------------------
// imgui.ini path resolution
// ---------------------------------------------------------------------------

std::string MolaVizImGuiCore::resolve_imgui_ini_path(const window_name_t& windowName) const
{
  if (imgui_app_name_.empty()) return {};

  namespace fs = std::filesystem;

  fs::path baseDir;
  if (const char* xdg = std::getenv("XDG_CONFIG_HOME"); xdg && *xdg)
    baseDir = fs::path(xdg) / "mola";
  else if (const char* home = std::getenv("HOME"); home && *home)
    baseDir = fs::path(home) / ".config" / "mola";
  else
    baseDir = fs::temp_directory_path() / "mola";

  std::error_code ec;
  fs::create_directories(baseDir, ec);
  if (ec)
  {
    MRPT_LOG_WARN_STREAM(
        "resolve_imgui_ini_path(): could not create '" << baseDir.string()
                                                       << "': " << ec.message());
    return {};
  }

  std::string fname = "imgui_" + imgui_app_name_;
  if (windowName != DEFAULT_WINDOW_NAME) fname += "_" + windowName;
  fname += ".ini";

  return (baseDir / fname).string();
}

// ---------------------------------------------------------------------------
// Embed mode: spin_one_frame
// ---------------------------------------------------------------------------

void MolaVizImGuiCore::spin_one_frame(const window_name_t& name)
{
  auto it = windows_.find(name);
  if (it == windows_.end()) return;
  auto& wd = it->second;

  internal_drain_task_queue();

  for (auto& [swName, sw] : wd.sub_windows)
  {
    if (sw.desc.tabs.empty() && wd.sensor_windows.count(swName)) continue;
    render_subwindow(sw);
  }

  render_sensor_windows(name, wd);
  if (console_enabled_)
  {
    render_console_window(wd);
  }
  if (plots_enabled_)
  {
    render_plot_windows(wd);
  }
  internal_handle_decaying_clouds(wd);
}

// ---------------------------------------------------------------------------
// Host mode: render_frame
// ---------------------------------------------------------------------------

void MolaVizImGuiCore::render_frame(const window_name_t& name, PerWindowData& wd)
{
  glfwMakeContextCurrent(wd.glfw_window);

  int display_w, display_h;
  glfwGetFramebufferSize(wd.glfw_window, &display_w, &display_h);

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // Full-screen dockspace:
  ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::SetNextWindowViewport(viewport->ID);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

  ImGuiWindowFlags dockspace_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                                     ImGuiWindowFlags_NoBackground;
  if (menu_bar_enabled_)
  {
    dockspace_flags |= ImGuiWindowFlags_MenuBar;
  }

  ImGui::Begin("##DockSpaceRoot", nullptr, dockspace_flags);
  ImGui::PopStyleVar(3);

  if (menu_bar_enabled_)
  {
    render_menu_bar(wd);
  }

  ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");

  // The dockspace node normally comes into existence lazily on the first
  // ImGui::DockSpace() call below, but the DockBuilder calls just after this
  // need it to already exist. Create it once, without touching a layout
  // that may already have been loaded from imgui.ini for this same ID. A
  // freshly-added node has no children yet, so it *is* the sole leaf/
  // passthrough area so far; track that in `dock_central_id` since, once
  // this ID is later split (below, or from a Dataset_UI panel), the root ID
  // itself becomes a non-leaf parent node and can no longer be split
  // directly. For a layout already loaded from imgui.ini, resolve the
  // actual passthrough leaf via DockBuilderGetCentralNode() instead (retried
  // every frame until it resolves, in case it isn't populated yet on the
  // very first frame).
  if (!ImGui::DockBuilderGetNode(dockspace_id))
  {
    ImGui::DockBuilderAddNode(
        dockspace_id, ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);
    wd.dock_central_id = dockspace_id;
  }
  else if (wd.dock_central_id == 0)
  {
    if (ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dockspace_id))
    {
      wd.dock_central_id = central->ID;
    }
  }

  // Dock the Console window at the bottom the first time it appears in this
  // profile: either a genuinely fresh imgui.ini, or an older saved layout
  // that predates the Console feature and therefore has no saved entry for
  // it yet. Runs once; if the window already has a saved entry (including
  // one left floating by the user on purpose), it is left untouched.
  if (console_enabled_ && !wd.console_dock_defaulted)
  {
    wd.console_dock_defaulted = true;
    if (wd.dock_central_id != 0 && !ImGui::FindWindowSettingsByID(ImHashStr("Console")))
    {
      ImGuiID dock_id_bottom = ImGui::DockBuilderSplitNode(
          wd.dock_central_id, ImGuiDir_Down, 0.25f, nullptr, &wd.dock_central_id);
      ImGui::DockBuilderDockWindow("Console", dock_id_bottom);
      ImGui::DockBuilderFinish(dockspace_id);
    }
  }

  ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
  ImGui::End();

  internal_drain_task_queue();

  render_background_scene(wd);

  for (auto& [swName, sw] : wd.sub_windows)
  {
    if (sw.desc.tabs.empty() && wd.sensor_windows.count(swName)) continue;
    render_subwindow(sw);
  }

  render_sensor_windows(name, wd);
  if (console_enabled_)
  {
    render_console_window(wd);
  }
  if (plots_enabled_)
  {
    render_plot_windows(wd);
  }
  internal_handle_decaying_clouds(wd);

  ImGui::Render();
  glViewport(0, 0, display_w, display_h);
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(wd.glfw_window);
}

// ---------------------------------------------------------------------------
// Task queue
// ---------------------------------------------------------------------------

std::future<void> MolaVizImGuiCore::enqueue_custom_gui_code(const std::function<void()>& userCode)
{
  auto            task = std::make_shared<std::packaged_task<void()>>([=]() { userCode(); });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

void MolaVizImGuiCore::internal_drain_task_queue()
{
  task_queue_t tasks;
  {
    std::lock_guard lk(guiThreadPendingTasksMtx_);
    tasks = std::move(guiThreadPendingTasks_);
    guiThreadPendingTasks_.clear();
  }
  for (auto& t : tasks)
  {
    try
    {
      t();
    }
    catch (const std::exception& e)
    {
      MRPT_LOG_ERROR_STREAM("Exception in GUI task:\n" << e.what());
    }
  }
}

// ---------------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------------

void MolaVizImGuiCore::render_app_menu()
{
  if (!ImGui::BeginMenu("MOLA"))
  {
    return;
  }

  if (ImGui::MenuItem("Quit") && quit_callback_)
  {
    quit_callback_();
  }

  ImGui::EndMenu();
}

void MolaVizImGuiCore::render_menu_bar(PerWindowData& wd)
{
  // The "MOLA" and built-in "View" menus are always available, even if no
  // module has installed a custom menu bar via set_menu_bar().
  if (ImGui::BeginMenuBar())
  {
    render_app_menu();

    if (plots_enabled_)
    {
      render_view_menu(wd);
    }

    for (const auto& menu : wd.menu_bar.menus)
    {
      if (ImGui::BeginMenu(menu.label.c_str()))
      {
        for (const auto& item : menu.items)
        {
          if (item.label.empty())
          {
            ImGui::Separator();
            continue;
          }
          const char* shortcut = item.shortcut.empty() ? nullptr : item.shortcut.c_str();
          if (ImGui::MenuItem(item.label.c_str(), shortcut, false, item.enabled) && item.on_click)
            item.on_click();
        }
        ImGui::EndMenu();
      }
    }
    ImGui::EndMenuBar();
  }
}

void MolaVizImGuiCore::render_view_menu(PerWindowData& wd)
{
  if (!ImGui::BeginMenu("View"))
  {
    return;
  }

  if (ImGui::MenuItem("New plot window"))
  {
    PlotWindowState st;
    st.title        = "Plot " + std::to_string(wd.next_plot_id++);
    st.span_seconds = static_cast<float>(plots_default_span_seconds_);
    wd.plot_windows.push_back(std::move(st));
  }

  if (!wd.plot_windows.empty() || console_enabled_)
  {
    ImGui::Separator();
  }

  if (console_enabled_)
  {
    ImGui::MenuItem("Console", nullptr, &wd.console_open);
  }

  for (auto& st : wd.plot_windows)
  {
    ImGui::MenuItem(st.title.c_str(), nullptr, &st.open);
  }

  ImGui::EndMenu();
}

void MolaVizImGuiCore::render_background_scene(PerWindowData& wd)
{
  std::lock_guard lk(wd.background_scene_mtx);

  if (!wd.background_scene || !wd.background_scene_view) return;

  if (wd.background_scene_view->scene() != wd.background_scene)
    wd.background_scene_view->setScene(wd.background_scene);

  if (wd.cam_dirty)
  {
    auto& cam = wd.background_scene_view->cameraController;
    cam.setAzimuthDegrees(wd.cam_azimuth_deg);
    cam.setElevationDegrees(wd.cam_elevation_deg);
    cam.setZoomDistance(wd.cam_zoom);
    cam.setCameraPointing(wd.cam_look_at[0], wd.cam_look_at[1], wd.cam_look_at[2]);
    cam.setProjectiveModel(!wd.cam_orthographic);
    wd.cam_dirty = false;
  }

  ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(viewport->WorkSize);

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                           ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
                           ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings;

  ImGui::SetNextWindowBgAlpha(0.0f);
  if (ImGui::Begin("##bg_scene", nullptr, flags))
  {
    while (glGetError() != GL_NO_ERROR)
    {
    }
    wd.background_scene_view->render();

    const auto& cam      = wd.background_scene_view->cameraController;
    wd.cam_azimuth_deg   = cam.getAzimuthDegrees();
    wd.cam_elevation_deg = cam.getElevationDegrees();
    wd.cam_zoom          = cam.getZoomDistance();
    wd.cam_look_at[0]    = cam.getCameraPointingX();
    wd.cam_look_at[1]    = cam.getCameraPointingY();
    wd.cam_look_at[2]    = cam.getCameraPointingZ();
  }
  ImGui::End();
}

namespace
{
ImVec4 color_for_level(mrpt::system::VerbosityLevel level)
{
  switch (level)
  {
    case mrpt::system::LVL_DEBUG:
      return ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
    case mrpt::system::LVL_WARN:
      return ImVec4(1.0f, 0.85f, 0.3f, 1.0f);
    case mrpt::system::LVL_ERROR:
      return ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
    case mrpt::system::LVL_INFO:
    default:
      return ImVec4(0.9f, 0.9f, 0.9f, 1.0f);
  }
}

bool contains_ci(std::string_view hay, std::string_view needle)
{
  auto to_lower = [](std::string_view s)
  {
    std::string r(s);
    std::transform(r.begin(), r.end(), r.begin(), [](unsigned char c) { return std::tolower(c); });
    return r;
  };
  return to_lower(hay).find(to_lower(needle)) != std::string::npos;
}

std::string format_console_line(const ConsoleLogEntry& e)
{
  mrpt::system::TTimeParts parts;
  mrpt::system::timestampToParts(e.timestamp, parts, /*localTime=*/true);
  const unsigned int millisec =
      static_cast<unsigned int>((parts.second - static_cast<unsigned int>(parts.second)) * 1000);
  return mrpt::format(
      "[%02u:%02u:%02u.%03u] [%s] %s", parts.hour, parts.minute,
      static_cast<unsigned int>(parts.second), millisec, e.source.c_str(), e.text.c_str());
}
}  // namespace

bool MolaVizImGuiCore::console_entry_passes_filters(const ConsoleLogEntry& e) const
{
  const double now    = mrpt::Clock::nowDouble();
  const double maxAge = console_window_seconds_;
  if (maxAge > 0 && now - mrpt::Clock::toDouble(e.timestamp) > maxAge) return false;

  if (!console_level_enabled_[static_cast<int>(e.level)]) return false;

  if (!console_selected_source_name_.empty() && e.source != console_selected_source_name_)
    return false;

  if (!console_filter_text_.empty() && !contains_ci(e.text, console_filter_text_) &&
      !contains_ci(e.source, console_filter_text_))
    return false;

  return true;
}

void MolaVizImGuiCore::console_save_to_file()
{
  const std::string fname = "mola_console_" +
                            mrpt::system::fileNameStripInvalidChars(
                                mrpt::system::dateTimeLocalToString(mrpt::Clock::now())) +
                            ".txt";

  std::ofstream f(fname);
  if (!f)
  {
    MRPT_LOG_ERROR_STREAM("Console: could not open '" << fname << "' for writing.");
    return;
  }

  for (const auto& e : console_sink_->snapshot())
  {
    if (!console_entry_passes_filters(e)) continue;
    f << format_console_line(e) << "\n";
  }
  MRPT_LOG_INFO_STREAM("Console: saved log to '" << fname << "'.");
}

void MolaVizImGuiCore::render_console_window(PerWindowData& wd)
{
  if (!console_sink_ || !wd.console_open)
  {
    return;
  }

  ImGui::SetNextWindowSize(ImVec2(700, 300), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Console", &wd.console_open))
  {
    ImGui::End();
    return;
  }

  // ---- Toolbar row --------------------------------------------------------
  if (ImGui::SmallButton("Clear")) console_sink_->clear();
  ImGui::SameLine();
  const bool wantSave = ImGui::SmallButton("Save");
  ImGui::SameLine();

  ImGui::SetNextItemWidth(180);
  ImGui::InputTextWithHint("##filter", "Filter (substring)", &console_filter_text_);
  ImGui::SameLine();

  const std::vector<std::string> srcs = console_sink_->sources();
  std::vector<const char*>       items;
  items.push_back("all");
  for (const auto& s : srcs) items.push_back(s.c_str());

  // Re-derive the combo index from the selected *name* every frame, since
  // `srcs` is a sorted set whose order shifts as new modules register --
  // an index alone would silently start pointing at a different source.
  int selCombo = 0;
  if (!console_selected_source_name_.empty())
  {
    const auto it = std::find(srcs.begin(), srcs.end(), console_selected_source_name_);
    if (it != srcs.end())
      selCombo = static_cast<int>(it - srcs.begin()) + 1;
    else
      console_selected_source_name_.clear();
  }

  ImGui::SetNextItemWidth(160);
  if (ImGui::Combo("Source", &selCombo, items.data(), static_cast<int>(items.size())))
    console_selected_source_name_ = selCombo > 0 ? srcs[static_cast<size_t>(selCombo - 1)] : "";
  ImGui::SameLine();

  static const char* kCaptureLevelNames[] = {"Debug", "Info", "Warn", "Error"};
  int                captureIdx           = static_cast<int>(console_capture_level_.load());
  ImGui::SetNextItemWidth(100);
  if (ImGui::Combo("Capture", &captureIdx, kCaptureLevelNames, IM_ARRAYSIZE(kCaptureLevelNames)))
    console_capture_level_.store(static_cast<mrpt::system::VerbosityLevel>(captureIdx));
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip(
        "Verbosity threshold sent by every module to this console (pipeline-wide).\n"
        "Raising it to Debug makes ALL modules pay the cost of formatting debug\n"
        "log lines, even ones the Levels filter below then hides.\n"
        "The Levels checkboxes only filter what's already been captured.");

  ImGui::TextUnformatted("Levels:");
  ImGui::SameLine();
  ImGui::Checkbox("D", &console_level_enabled_[0]);
  ImGui::SameLine();
  ImGui::Checkbox("I", &console_level_enabled_[1]);
  ImGui::SameLine();
  ImGui::Checkbox("W", &console_level_enabled_[2]);
  ImGui::SameLine();
  ImGui::Checkbox("E", &console_level_enabled_[3]);
  ImGui::SameLine();
  ImGui::Checkbox("Autoscroll", &console_autoscroll_);

  ImGui::Separator();

  // ---- Log area -------------------------------------------------------------
  // Re-snapshot the sink only when it actually changed (version bump on
  // push()/clear()). Producer threads (module loggers) and this GUI thread
  // both contend on the sink's mutex; a plain snapshot() on every rendered
  // frame -- regardless of whether new entries arrived -- copies the whole
  // (up to `max_entries`) deque under that lock at up to `target_fps_` Hz,
  // which is wasted work when logging is idle or between bursts, and briefly
  // blocks any module trying to push() a new line while the copy runs.
  // Read the version *before* copying: if a push() races in between and
  // lands in the copy anyway, we'll just re-snapshot once more than strictly
  // needed next frame -- reading it after the copy could instead record a
  // version newer than what was actually captured, permanently hiding that
  // entry until the next push happens to change the version again.
  if (const uint64_t v = console_sink_->version(); v != console_cached_version_)
  {
    console_cached_entries_ = console_sink_->snapshot();
    console_cached_version_ = v;
  }

  ImGui::BeginChild("##log", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

  for (const auto& e : console_cached_entries_)
  {
    if (!console_entry_passes_filters(e)) continue;

    const ImVec4 col = color_for_level(e.level);
    ImGui::PushStyleColor(ImGuiCol_Text, col);
    ImGui::TextUnformatted(format_console_line(e).c_str());
    ImGui::PopStyleColor();
  }

  if (console_autoscroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
    ImGui::SetScrollHereY(1.0f);

  ImGui::EndChild();
  ImGui::End();

  if (wantSave) console_save_to_file();
}

void MolaVizImGuiCore::render_plot_toolbar(PlotWindowState& st)
{
  static constexpr float       kSpans[]      = {1.0f, 2.0f, 5.0f, 10.0f};
  static constexpr const char* kSpanLabels[] = {"1s", "2s", "5s", "10s"};

  int spanIdx = 2;  // fall back to 5s if span_seconds doesn't match a preset
  for (size_t i = 0; i < IM_ARRAYSIZE(kSpans); i++)
  {
    if (std::abs(kSpans[i] - st.span_seconds) < 1e-3f) spanIdx = static_cast<int>(i);
  }
  ImGui::SetNextItemWidth(70);
  if (ImGui::Combo("Span", &spanIdx, kSpanLabels, IM_ARRAYSIZE(kSpanLabels)))
  {
    st.span_seconds = kSpans[spanIdx];
  }
  ImGui::SameLine();

  // "Add channel" combo, populated with channels not already shown here.
  const std::vector<std::string> allNames = metrics_->channel_names();
  std::vector<std::string>       avail;
  for (const auto& n : allNames)
  {
    if (std::find(st.channels.begin(), st.channels.end(), n) == st.channels.end())
    {
      avail.push_back(n);
    }
  }
  std::vector<const char*> availLabels;
  availLabels.reserve(avail.size());
  for (const auto& n : avail) availLabels.push_back(n.c_str());

  int addIdx = -1;
  ImGui::SetNextItemWidth(200);
  if (ImGui::Combo(
          "Add channel", &addIdx, availLabels.data(), static_cast<int>(availLabels.size())) &&
      addIdx >= 0)
  {
    st.channels.push_back(avail[static_cast<size_t>(addIdx)]);
  }

  ImGui::Checkbox("Grid X", &st.show_grid_x);
  ImGui::SameLine();
  ImGui::Checkbox("Grid Y", &st.show_grid_y);
  ImGui::SameLine();
  ImGui::Checkbox("Legend", &st.show_legend);
  ImGui::SameLine();
  ImGui::Checkbox("Lines", &st.lines);
  ImGui::SameLine();
  ImGui::Checkbox("Y autoscale", &st.y_autoscale);
  ImGui::SameLine();
  ImGui::Checkbox("Pause", &st.paused);

  // One removable "chip" per subscribed channel.
  for (size_t i = 0; i < st.channels.size();)
  {
    ImGui::SameLine();
    const std::string label = st.channels[i] + " x##rm_" + st.channels[i];
    if (ImGui::SmallButton(label.c_str()))
    {
      st.channels.erase(st.channels.begin() + static_cast<long>(i));
      continue;
    }
    ++i;
  }
}

void MolaVizImGuiCore::render_plot_windows(PerWindowData& wd)
{
  bool anyOpenWithChannels = false;

  for (auto& st : wd.plot_windows)
  {
    if (!st.open) continue;

    ImGui::SetNextWindowSize(ImVec2(480, 320), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(st.title.c_str(), &st.open))
    {
      render_plot_toolbar(st);
      if (!st.channels.empty()) anyOpenWithChannels = true;

      const double now  = mrpt::Clock::nowDouble();
      const double tmin = now - static_cast<double>(st.span_seconds);

      // Widen (never shrink) each shown channel's retention to cover this
      // window's span, so copy_span() below always finds the full history.
      const double wantRetention =
          std::max(plots_default_retention_seconds_, static_cast<double>(st.span_seconds));
      for (const auto& chName : st.channels)
      {
        auto ch = metrics_->find(chName);
        if (!ch) continue;
        if (wantRetention > ch->retention_seconds.load(std::memory_order_relaxed))
          ch->retention_seconds.store(wantRetention, std::memory_order_relaxed);
      }

      const ImPlotFlags plotFlags = st.show_legend ? ImPlotFlags_None : ImPlotFlags_NoLegend;
      if (ImPlot::BeginPlot(("##" + st.title).c_str(), ImVec2(-1, -1), plotFlags))
      {
        const ImPlotAxisFlags xFlags =
            st.show_grid_x ? ImPlotAxisFlags_None : ImPlotAxisFlags_NoGridLines;
        const ImPlotAxisFlags yFlags =
            (st.show_grid_y ? ImPlotAxisFlags_None : ImPlotAxisFlags_NoGridLines) |
            (st.y_autoscale ? ImPlotAxisFlags_AutoFit : ImPlotAxisFlags_None);
        ImPlot::SetupAxes("t [s]", nullptr, xFlags, yFlags);
        if (!st.paused) ImPlot::SetupAxisLimits(ImAxis_X1, tmin, now, ImPlotCond_Always);

        for (const auto& chName : st.channels)
        {
          auto ch = metrics_->find(chName);
          if (!ch) continue;

          ch->copy_span(tmin, plot_scratch_xs_, plot_scratch_ys_);
          if (plot_scratch_xs_.empty()) continue;

          ImPlotSpec spec;
          spec.LineColor      = ch->color;
          const auto        n = static_cast<int>(plot_scratch_xs_.size());
          const std::string label =
              ch->unit.empty() ? ch->name : (ch->name + " [" + ch->unit + "]");
          if (st.lines)
            ImPlot::PlotLine(
                label.c_str(), plot_scratch_xs_.data(), plot_scratch_ys_.data(), n, spec);
          else
            ImPlot::PlotScatter(
                label.c_str(), plot_scratch_xs_.data(), plot_scratch_ys_.data(), n, spec);
        }
        ImPlot::EndPlot();
      }
    }
    ImGui::End();
  }

  metrics_->any_window_open()->store(anyOpenWithChannels, std::memory_order_relaxed);
}

void MolaVizImGuiCore::render_subwindow(SubWindowState& sw)
{
  const auto& desc = sw.desc;

  ImGui::SetNextWindowPos(
      ImVec2(static_cast<float>(desc.position[0]), static_cast<float>(desc.position[1])),
      ImGuiCond_FirstUseEver);

  if (desc.size[0] > 0 || desc.size[1] > 0)
  {
    ImGui::SetNextWindowSize(
        ImVec2(
            static_cast<float>(desc.size[0]),
            static_cast<float>(desc.size[1] > 0 ? desc.size[1] : 300)),
        ImGuiCond_FirstUseEver);
  }

  if (desc.starts_hidden) ImGui::SetNextWindowCollapsed(true, ImGuiCond_FirstUseEver);

  if (!sw.open) return;

  // Persist the close state in SubWindowState so clicking the window's [x]
  // sticks across frames instead of the window reappearing each frame.
  if (ImGui::Begin(desc.title.c_str(), &sw.open)) render_widget_description(desc, sw);
  ImGui::End();
}

void MolaVizImGuiCore::render_sensor_windows(const window_name_t& parentName, PerWindowData& wd)
{
  auto&           hc = ImGuiHandlersContainer::Instance();
  std::lock_guard lk(hc.mtx);

  for (auto& [title, slot] : wd.sensor_windows)
  {
    if (!slot.obj) continue;
    try
    {
      if (auto swIt = wd.sub_windows.find(title); swIt != wd.sub_windows.end())
      {
        const auto& d = swIt->second.desc;
        ImGui::SetNextWindowPos(
            ImVec2(static_cast<float>(d.position[0]), static_cast<float>(d.position[1])),
            ImGuiCond_FirstUseEver);
        if (d.size[0] > 0 || d.size[1] > 0)
        {
          ImGui::SetNextWindowSize(
              ImVec2(
                  static_cast<float>(d.size[0] > 0 ? d.size[0] : 400),
                  static_cast<float>(d.size[1] > 0 ? d.size[1] : 300)),
              ImGuiCond_FirstUseEver);
        }
      }
      auto range = hc.handlers.equal_range(slot.class_name);
      for (auto it = range.first; it != range.second; ++it)
        it->second(slot.obj, nullptr, parentName, title, this, slot.extra.get());
    }
    catch (const std::exception& e)
    {
      MRPT_LOG_ERROR_STREAM("render_sensor_windows: handler threw: " << e.what());
    }
  }
}

void MolaVizImGuiCore::internal_handle_decaying_clouds(PerWindowData& wd)
{
  constexpr float FADE_OUT_FRACTION = 0.1f;

  // Mutate point-cloud alphas under the same mutex VizInterface writers use,
  // so an external scene renderer (embed-mode CImGuiSceneView) holding this
  // lock during render() sees consistent vertex buffers.
  std::lock_guard lk(wd.background_scene_mtx);

  const size_t queueSize = wd.decaying_clouds.size();
  if (queueSize == 0) return;

  const size_t maxScans  = wd.max_decaying_clouds;
  const size_t fadeCount = std::max<size_t>(
      1u, static_cast<size_t>(std::round(static_cast<float>(maxScans) * FADE_OUT_FRACTION)));

  for (size_t i = 0; i < queueSize; i++)
  {
    auto&        dc    = wd.decaying_clouds[i];
    const size_t age   = queueSize - 1u - i;
    float        alpha = dc.initial_alpha;
    if (age >= (maxScans - fadeCount))
    {
      const float t =
          static_cast<float>(age - (maxScans - fadeCount)) / static_cast<float>(fadeCount);
      alpha = dc.initial_alpha * std::max(0.0f, 1.0f - t);
    }
    dc.cloud->setAllPointsAlpha(mrpt::f2u8(alpha));
  }
}

// ---------------------------------------------------------------------------
// VizInterface: sub-window API
// ---------------------------------------------------------------------------

void* MolaVizImGuiCore::get_subwindow_handle(const std::string&, const std::string&)
{
  return nullptr;
}

std::future<void> MolaVizImGuiCore::create_subwindow_from_description(
    const mola::gui::WindowDescription& desc, const std::string& parentWindow)
{
  return enqueue_custom_gui_code(
      [this, desc, parentWindow]()
      {
        MRPT_LOG_DEBUG_STREAM("create_subwindow_from_description() title='" << desc.title << "'");

        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          MRPT_LOG_WARN_STREAM(
              "create_subwindow_from_description(): unknown parentWindow '" << parentWindow << "'");
          return;
        }
        auto&      wd          = it->second;
        const bool isNewWindow = wd.sub_windows.count(desc.title) == 0;

        auto& sw      = wd.sub_windows[desc.title];
        sw.desc       = desc;
        sw.active_tab = 0;

        // The first time a window that requests it (e.g. a Dataset_UI
        // panel) appears without a saved imgui.ini entry of its own, dock
        // it into a strip reserved at the top of the main window instead of
        // leaving it floating. Skipped if the window already has a saved
        // entry (including one the user has since moved elsewhere on
        // purpose). Must run before the window's first ImGui::Begin(),
        // which is guaranteed here since this task queue is drained before
        // sub-windows are rendered each frame.
        if (isNewWindow && desc.dock_top_by_default &&
            !ImGui::FindWindowSettingsByID(ImHashStr(desc.title.c_str())))
        {
          // Split off the *current* remaining passthrough leaf (tracked in
          // `dock_central_id`, kept up to date by render_frame() and by
          // every split below), not the dockspace root: once any split has
          // happened the root ID becomes a non-leaf parent node and can no
          // longer be split directly.
          if (wd.default_dock_top_id == 0 && wd.dock_central_id != 0)
          {
            wd.default_dock_top_id = ImGui::DockBuilderSplitNode(
                wd.dock_central_id, ImGuiDir_Up, 0.12f, nullptr, &wd.dock_central_id);
            ImGui::DockBuilderFinish(ImGui::GetID("MainDockSpace"));
          }
          if (wd.default_dock_top_id != 0)
          {
            ImGui::DockBuilderDockWindow(desc.title.c_str(), wd.default_dock_top_id);
          }
        }
      });
}

std::future<void> MolaVizImGuiCore::set_menu_bar(
    const mola::gui::MenuBar& bar, const std::string& parentWindow)
{
  return enqueue_custom_gui_code(
      [this, bar, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return;
        if (it->second.glfw_window == nullptr)
        {
          // Embed mode: the host owns the dockspace and decides whether to
          // render a menu bar.  We do not render one ourselves, so silently
          // accepting the bar would lose it.  Warn once so the host wires
          // its own menu integration.
          MRPT_LOG_WARN_STREAM(
              "set_menu_bar(): parentWindow '"
              << parentWindow
              << "' is in embed mode; menu bar will not be rendered by the core. "
                 "The host application is responsible for rendering menus.");
          return;
        }
        it->second.menu_bar = bar;
      });
}

MetricChannel::Ptr MolaVizImGuiCore::register_metric(
    const std::string& name, const std::string& unit)
{
  auto ch = metrics_->get_or_create(name, unit);
  ch->retention_seconds.store(plots_default_retention_seconds_, std::memory_order_relaxed);
  return ch;
}

void MolaVizImGuiCore::push_metric(const std::string& name, double t, double value)
{
  metrics_->get_or_create(name, "")->push(t, value);
}

std::future<std::optional<std::string>> MolaVizImGuiCore::open_file_dialog(
    const std::string& title, bool save,
    const std::vector<std::pair<std::string, std::string>>& filters,
    const std::string& default_path, const std::string& /*parentWindow*/)
{
  using return_type = std::optional<std::string>;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, title, save, filters, default_path]() -> return_type
      {
        // TODO: integrate ImGuiFileDialog or NFD.
        MRPT_LOG_WARN("open_file_dialog(): stub - no file dialog library integrated yet.");
        (void)title;
        (void)save;
        (void)filters;
        (void)default_path;
        return std::nullopt;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// VizInterface: 3-D scene API
// ---------------------------------------------------------------------------

namespace
{
/** Get-or-create a movable reference-frame node (a named CSetOfObjects at the
 *  viewport root). Caller must already hold the scene mutex. */
mrpt::viz::CSetOfObjects::Ptr getOrCreateFrameNode(
    mrpt::viz::Scene& scene, const std::string& frameName, const std::string& viewportName)
{
  mrpt::viz::CSetOfObjects::Ptr frameNode;
  if (auto o = scene.getByName(frameName, viewportName); o)
    frameNode = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(o);
  if (!frameNode)
  {
    frameNode = mrpt::viz::CSetOfObjects::Create();
    frameNode->setName(frameName);
    scene.insert(frameNode, viewportName);
  }
  return frameNode;
}
}  // namespace

std::future<bool> MolaVizImGuiCore::update_3d_object(
    const std::string& objName, const std::shared_ptr<mrpt::viz::CSetOfObjects>& obj,
    const std::string& viewportName, const std::string& parentWindow,
    const std::string& parentFrame)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, objName, obj, viewportName, parentWindow, parentFrame]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;

        auto&           scene = it->second.background_scene;
        std::lock_guard lk(it->second.background_scene_mtx);

        mrpt::viz::CSetOfObjects::Ptr container;
        if (parentFrame.empty())
        {
          // Non-recursive search: only direct children of the viewport root,
          // so objects nested inside frame nodes are not falsely matched.
          if (auto vp = scene->getViewport(viewportName); vp)
          {
            for (const auto& o : *vp)
            {
              if (o->getName() == objName)
              {
                container = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(o);
                break;
              }
            }
          }
          if (!container)
          {
            // Not at root. Evict from any frame node it may live in, then
            // create a fresh container at the viewport root.
            if (auto o = scene->getByName(objName, viewportName); o)
            {
              scene->removeObject(o, viewportName);
            }
            container = mrpt::viz::CSetOfObjects::Create();
            scene->insert(container, viewportName);
          }
        }
        else
        {
          // Attach as a child of the named movable frame node:
          auto frameNode = getOrCreateFrameNode(*scene, parentFrame, viewportName);
          if (auto o = frameNode->getByName(objName); o)
          {
            container = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(o);
          }
          if (!container)
          {
            // Not in this frame. Evict from root or another frame before
            // inserting here (removeObject searches recursively).
            if (auto o = scene->getByName(objName, viewportName); o)
            {
              scene->removeObject(o, viewportName);
            }
            container = mrpt::viz::CSetOfObjects::Create();
            frameNode->insert(container);
          }
        }
        *container = *obj;
        container->setName(objName);
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::update_3d_object_frame(
    const std::string& frameName, const mrpt::math::TPose3D& pose, const std::string& viewportName,
    const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, frameName, pose, viewportName, parentWindow]()
      {
        ASSERT_(!frameName.empty());
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;

        auto&           scene = it->second.background_scene;
        std::lock_guard lk(it->second.background_scene_mtx);

        auto frameNode = getOrCreateFrameNode(*scene, frameName, viewportName);
        frameNode->setPose(pose);
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::clear_background_scene(
    const std::string& viewportName, const window_name_t& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, viewportName, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }

        auto&           scene = it->second.background_scene;
        std::lock_guard lk(it->second.background_scene_mtx);

        auto viewport = scene->getViewport(viewportName);
        if (!viewport)
        {
          return false;
        }

        viewport->clear();

        // Also drop the decaying-clouds bookkeeping: their backing
        // CSetOfObjects container just got cleared above, so keeping these
        // around would only fade alphas on now-orphaned point clouds.
        it->second.decaying_clouds.clear();
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::insert_point_cloud_with_decay(
    const std::shared_ptr<mrpt::viz::CPointCloudColoured>& cloud, double decay_time_seconds,
    const std::string& viewportName, const std::string& parentWindow,
    const std::string& parentFrame)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, cloud, decay_time_seconds, viewportName, parentWindow, parentFrame]()
      {
        if (!cloud || cloud->empty()) return true;

        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;

        auto& wd = it->second;

        constexpr const char* DECAY_NAME = "__viz_decaying_clouds";
        std::lock_guard       lk(wd.background_scene_mtx);

        mrpt::viz::CSetOfObjects::Ptr container;
        if (parentFrame.empty())
        {
          if (auto o = wd.background_scene->getByName(DECAY_NAME, viewportName); o)
            container = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(o);
          if (!container)
          {
            container = mrpt::viz::CSetOfObjects::Create();
            wd.background_scene->insert(container, viewportName);
            container->setName(DECAY_NAME);
          }
        }
        else
        {
          auto frameNode = getOrCreateFrameNode(*wd.background_scene, parentFrame, viewportName);
          if (auto o = frameNode->getByName(DECAY_NAME); o)
            container = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(o);
          if (!container)
          {
            container = mrpt::viz::CSetOfObjects::Create();
            container->setName(DECAY_NAME);
            frameNode->insert(container);
          }
        }
        container->insert(cloud);

        // Clamp to >= 0 first: a negative decay_time_seconds would round to a
        // negative double and wrap around to a huge size_t, letting the decay
        // queue grow without bound.
        const double decaySecs = std::max(0.0, decay_time_seconds);
        const size_t maxScans  = std::max<size_t>(
            1u, static_cast<size_t>(std::round(decaySecs * assumed_sensor_rate_hz_)));
        wd.max_decaying_clouds = maxScans;

        const float alpha = mrpt::u8tof(cloud->shaderPointsVertexColorBuffer().at(0).A);
        wd.decaying_clouds.emplace_back(viewportName, cloud, alpha);
        wd.decaying_clouds.back().container = container;

        while (wd.decaying_clouds.size() > maxScans)
        {
          auto& oldest = wd.decaying_clouds.front();
          if (oldest.container)
          {
            oldest.container->removeObject(oldest.cloud);
          }
          wd.decaying_clouds.pop_front();
        }
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::clear_all_point_clouds_with_decay(
    const std::string& viewportName, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, viewportName, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;

        auto&                 wd         = it->second;
        constexpr const char* DECAY_NAME = "__viz_decaying_clouds";
        std::lock_guard       lk(wd.background_scene_mtx);
        if (auto o = wd.background_scene->getByName(DECAY_NAME, viewportName); o)
          if (auto c = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(o); c) c->clear();
        wd.decaying_clouds.clear();
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::update_viewport_look_at(
    const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName,
    const std::string& parentWindow, const std::string& parentFrame)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, lookAt, viewportName, parentWindow, parentFrame]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;
        auto& wd = it->second;

        mrpt::math::TPoint3Df worldLookAt = lookAt;
        if (!parentFrame.empty())
        {
          std::lock_guard lk(wd.background_scene_mtx);
          if (auto o = wd.background_scene->getByName(parentFrame, viewportName); o)
          {
            const auto worldPt =
                mrpt::poses::CPose3D(o->getPose())
                    .composePoint(mrpt::math::TPoint3D(lookAt.x, lookAt.y, lookAt.z));
            worldLookAt = {
                static_cast<float>(worldPt.x), static_cast<float>(worldPt.y),
                static_cast<float>(worldPt.z)};
          }
          else
          {
            return false;  // frame node not in scene yet; skip camera update
          }
        }
        wd.cam_look_at[0] = worldLookAt.x;
        wd.cam_look_at[1] = worldLookAt.y;
        wd.cam_look_at[2] = worldLookAt.z;
        wd.cam_dirty      = true;
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::update_viewport_camera_azimuth(
    double azimuth, bool absolute_falseForRelative, const std::string& /*viewportName*/,
    const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, azimuth, absolute_falseForRelative, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;
        auto& wd = it->second;
        if (absolute_falseForRelative)
          wd.cam_azimuth_deg = static_cast<float>(mrpt::RAD2DEG(azimuth));
        else
          wd.cam_azimuth_deg += static_cast<float>(mrpt::RAD2DEG(azimuth));
        wd.cam_dirty = true;
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::update_viewport_camera_orthographic(
    bool orthographic, const std::string& /*viewportName*/, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, orthographic, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;
        it->second.cam_orthographic = orthographic;
        it->second.cam_dirty        = true;
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::execute_custom_code_on_background_scene(
    const std::function<void(mrpt::viz::Scene&)>& userCode, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, userCode, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;
        auto&           wd = it->second;
        std::lock_guard lk(wd.background_scene_mtx);
        try
        {
          userCode(*wd.background_scene);
          return true;
        }
        catch (const std::exception& e)
        {
          MRPT_LOG_ERROR_STREAM("execute_custom_code_on_background_scene: " << e.what());
          return false;
        }
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// VizInterface: observation handler API
// ---------------------------------------------------------------------------

std::future<bool> MolaVizImGuiCore::subwindow_update_visualization(
    const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
    const mrpt::containers::yaml* extra_parameters, const std::string& parentWindow)
{
  auto safeExtra = extra_parameters
                       ? std::make_shared<const mrpt::containers::yaml>(*extra_parameters)
                       : std::shared_ptr<const mrpt::containers::yaml>();

  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, obj, subWindowTitle, safeExtra, parentWindow]() -> bool
      {
        if (!obj) return false;
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          MRPT_LOG_WARN_STREAM(
              "subwindow_update_visualization(): unknown parentWindow '" << parentWindow << "'");
          return false;
        }
        auto& slot      = it->second.sensor_windows[subWindowTitle];
        slot.obj        = obj;
        slot.extra      = safeExtra;
        slot.class_name = obj->GetRuntimeClass()->className;
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// VizInterface: console output
// ---------------------------------------------------------------------------

std::future<bool> MolaVizImGuiCore::output_console_message(
    const std::string& /*message*/, const std::string& /*parentWindow*/)
{
  // No-op: superseded by the dockable Console subwindow (see
  // render_console_window() / ConsoleLogSink), which aggregates
  // mrpt-logger output instead of requiring modules to push text here.
  std::promise<bool> prom;
  prom.set_value(true);
  return prom.get_future();
}
