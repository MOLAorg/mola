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
 * @file   MolaVizImGui.cpp
 * @brief  Dear ImGui (docking branch) backend - lifecycle, task queue,
 *         VizInterface API implementations.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

/** \defgroup mola_viz_imgui_grp mola-viz-imgui
 * C++ library for the Dear ImGui MOLA GUI backend
 */

#include <mola_viz_imgui/MolaVizImGui.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/system/thread_name.h>

// ImGui stdlib helper for std::string InputText:
#include <imgui_stdlib.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>

using namespace mola;

IMPLEMENTS_MRPT_OBJECT(MolaVizImGui, ExecutableBase, mola)

// ---------------------------------------------------------------------------
// Static constant definitions
// ---------------------------------------------------------------------------

const MolaVizImGui::window_name_t MolaVizImGui::DEFAULT_WINDOW_NAME = "main";

// ---------------------------------------------------------------------------
// Handler registry
// ---------------------------------------------------------------------------

struct ImGuiHandlersContainer
{
  static ImGuiHandlersContainer& Instance()
  {
    static ImGuiHandlersContainer o;
    return o;
  }
  std::multimap<MolaVizImGui::class_name_t, MolaVizImGui::update_handler_t> handlers;
  std::mutex                                                                mtx;

 private:
  ImGuiHandlersContainer() = default;
};

void MolaVizImGui::register_gui_handler(const class_name_t& name, const update_handler_t& handler)
{
  auto&           hc = ImGuiHandlersContainer::Instance();
  std::lock_guard lk(hc.mtx);
  hc.handlers.emplace(name, handler);
}

// ---------------------------------------------------------------------------
// GUI shutdown cleanup registry — see header doc
// ---------------------------------------------------------------------------

namespace
{
struct ImGuiCleanupContainer
{
  static ImGuiCleanupContainer& Instance()
  {
    static ImGuiCleanupContainer o;
    return o;
  }
  std::vector<std::function<void()>> cleanups;
  std::mutex                         mtx;
};
}  // namespace

void MolaVizImGui::register_gui_cleanup(const std::function<void()>& cleanup)
{
  auto&           c = ImGuiCleanupContainer::Instance();
  std::lock_guard lk(c.mtx);
  c.cleanups.push_back(cleanup);
}

namespace
{
void run_gui_cleanups()
{
  auto&                              c = ImGuiCleanupContainer::Instance();
  std::vector<std::function<void()>> local;
  {
    std::lock_guard lk(c.mtx);
    local = c.cleanups;  // copy so callbacks can't mutate under lock
  }
  for (auto& fn : local)
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
}  // namespace

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

MolaVizImGui* MolaVizImGui::instance_ = nullptr;
std::mutex    MolaVizImGui::instanceMtx_;

bool          MolaVizImGui::IsRunning() { return Instance() != nullptr; }
MolaVizImGui* MolaVizImGui::Instance()
{
  std::lock_guard lk(instanceMtx_);
  return instance_;
}

// ---------------------------------------------------------------------------
// MRPT initializer
// ---------------------------------------------------------------------------

extern void mola_viz_imgui_register_default_handlers();

MRPT_INITIALIZER(do_register_MolaVizImGui)  // NOLINT
{
  MOLA_REGISTER_MODULE(MolaVizImGui);
  mola_viz_imgui_register_default_handlers();
}

// ---------------------------------------------------------------------------
// Backend identity
// ---------------------------------------------------------------------------

const std::string& MolaVizImGui::gui_backend() const noexcept
{
  return VizInterface::BACKEND_IMGUI;
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

MolaVizImGui::MolaVizImGui() = default;

MolaVizImGui::~MolaVizImGui()
{
  guiThreadShutdown_.store(true);
  if (guiThread_.joinable()) guiThread_.join();

  std::lock_guard lk(instanceMtx_);
  instance_ = nullptr;
}

// ---------------------------------------------------------------------------
// initialize / spinOnce
// ---------------------------------------------------------------------------

void MolaVizImGui::initialize(const Yaml& c)
{
  MRPT_START

  auto cfg = c["params"];
  MRPT_LOG_DEBUG_STREAM("MolaVizImGui: loading params:\n" << cfg);

  YAML_LOAD_MEMBER_OPT(max_console_lines, unsigned int);
  YAML_LOAD_MEMBER_OPT(console_text_font_size, double);
  YAML_LOAD_MEMBER_OPT(show_rgbd_as_point_cloud, bool);
  YAML_LOAD_MEMBER_OPT(assumed_sensor_rate_hz, double);
  YAML_LOAD_MEMBER_OPT(target_fps, int);
  YAML_LOAD_MEMBER_OPT(imgui_app_name, std::string);

  {
    std::lock_guard lk(instanceMtx_);
    instance_ = this;
  }

  guiThread_ = std::thread(&MolaVizImGui::gui_thread, this);

  MRPT_END
}

void MolaVizImGui::spinOnce()
{
  const double PERIOD_CHECK_NEW_MODS    = 2.0;
  const double PERIOD_UPDATE_DATASET_UI = 0.25;
  const double tNow                     = mrpt::Clock::nowDouble();

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
// Core task-queue implementation
// ---------------------------------------------------------------------------

std::future<void> MolaVizImGui::enqueue_custom_gui_code(const std::function<void()>& userCode)
{
  auto            task = std::make_shared<std::packaged_task<void()>>([=]() { userCode(); });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// GUI thread
// ---------------------------------------------------------------------------

std::string MolaVizImGui::resolve_imgui_ini_path(const window_name_t& windowName) const
{
  if (imgui_app_name_.empty()) return {};  // persistence disabled

  namespace fs = std::filesystem;

  // Base directory: $XDG_CONFIG_HOME/mola/ or $HOME/.config/mola/
  fs::path baseDir;
  if (const char* xdg = std::getenv("XDG_CONFIG_HOME"); xdg && *xdg)
  {
    baseDir = fs::path(xdg) / "mola";
  }
  else if (const char* home = std::getenv("HOME"); home && *home)
  {
    baseDir = fs::path(home) / ".config" / "mola";
  }
  else
  {
    baseDir = fs::temp_directory_path() / "mola";
  }

  std::error_code ec;
  fs::create_directories(baseDir, ec);
  if (ec)
  {
    MRPT_LOG_WARN_STREAM(
        "resolve_imgui_ini_path(): could not create '" << baseDir.string()
                                                       << "': " << ec.message());
    return {};
  }

  // One file per (app, host-window) pair.  The default window is just
  // "<app>.ini"; additional windows get a suffix so they don't collide.
  std::string fname = "imgui_" + imgui_app_name_;
  if (windowName != DEFAULT_WINDOW_NAME) fname += "_" + windowName;
  fname += ".ini";

  return (baseDir / fname).string();
}

MolaVizImGui::PerWindowData& MolaVizImGui::create_and_add_window(const window_name_t& name)
{
  // Only one host GLFW window is supported.  Multi-OS-window setups should
  // use ImGuiConfigFlags_ViewportsEnable instead of multiple contexts.
  ASSERT_(windows_.empty());

  // Must be called from the GUI thread.
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

  const std::string title = "MOLAViz ImGui - " + name;
  GLFWwindow*       win   = glfwCreateWindow(1280, 800, title.c_str(), nullptr, nullptr);
  if (!win) throw std::runtime_error("MolaVizImGui: glfwCreateWindow failed");

  glfwMakeContextCurrent(win);
  glfwSwapInterval(0);  // vsync off; we cap via target_fps_

  auto& wd                 = windows_[name];
  wd.glfw_window           = win;
  wd.background_scene      = mrpt::opengl::COpenGLScene::Create();
  wd.background_scene_view = std::make_unique<mrpt::imgui::CImGuiSceneView>();

  // imgui_ctx_ was already created in gui_thread() before this call.
  // Configure IO flags and .ini persistence on that single context.
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  // Persist window / docking layout to a per-app .ini file.  The buffer
  // pointed to by io.IniFilename must outlive the ImGui context, so we
  // store it in imgui_ini_paths_ (a member map).  Empty path disables
  // persistence — ImGui then skips all load/save of .ini state.
  const std::string resolvedIni = resolve_imgui_ini_path(name);
  if (!resolvedIni.empty())
  {
    auto [it, inserted] = imgui_ini_paths_.emplace(name, resolvedIni);
    io.IniFilename      = it->second.c_str();
    MRPT_LOG_INFO_STREAM("ImGui layout .ini file: " << it->second);
  }
  else
  {
    io.IniFilename = nullptr;
  }

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(win, /*install_callbacks=*/true);
  ImGui_ImplOpenGL3_Init("#version 330");

  return wd;
}

void MolaVizImGui::gui_thread()
{
  MRPT_LOG_DEBUG("MolaVizImGui::gui_thread() started.");
  mrpt::system::thread_name("MolaVizImGui::gui_thread");

  if (!glfwInit()) throw std::runtime_error("MolaVizImGui: glfwInit() failed");

  // Single ImGui context for the lifetime of the GUI thread.  All GLFW
  // windows share it; multi-OS-window support (if ever needed) should use
  // ImGuiConfigFlags_ViewportsEnable instead of multiple contexts.
  imgui_ctx_ = ImGui::CreateContext();
  ImGui::SetCurrentContext(imgui_ctx_);

  create_and_add_window(DEFAULT_WINDOW_NAME);

  const double frame_period = 1.0 / static_cast<double>(std::max(1, target_fps_));

  while (!guiThreadShutdown_.load())
  {
    const double t0 = mrpt::Clock::nowDouble();

    glfwPollEvents();

    // Check if all windows were closed by the user:
    bool any_open = false;
    for (auto& [name, wd] : windows_)
      if (wd.glfw_window && !glfwWindowShouldClose(wd.glfw_window)) any_open = true;
    if (!any_open) break;

    // Render each window (pending tasks are drained inside render_frame
    // so they execute with a valid GL context and active ImGui frame):
    for (auto& [name, wd] : windows_)
    {
      if (!wd.glfw_window || glfwWindowShouldClose(wd.glfw_window)) continue;
      render_frame(name, wd);
    }

    // Cap frame rate:
    const double elapsed = mrpt::Clock::nowDouble() - t0;
    if (elapsed < frame_period)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(frame_period - elapsed));
    }
  }

  // Cleanup:
  // Critical: anything that owns GL resources (FBOs, textures, buffers)
  // must be released BEFORE the GL context is destroyed.  That means
  // clearing PerWindowData members that wrap GL state first, while the
  // context is still current, and only then shutting down the ImGui
  // backend and the GLFW window.
  for (auto& [name, wd] : windows_)
  {
    if (!wd.glfw_window) continue;
    glfwMakeContextCurrent(wd.glfw_window);

    // 1) Handler-owned state (function-local statics in handler files).
    //    These hold CImGuiSceneView instances whose destructors call
    //    glDeleteFramebuffers/Textures/etc.  Run with context current.
    run_gui_cleanups();

    // 2) This window's own GL-resource-holding members:
    wd.sensor_windows.clear();  // drops any per-slot GL state
    wd.decaying_clouds.clear();  // dropping CPointCloudColoured refs
    wd.background_scene.reset();  // drops scene (textures, VBOs)
    wd.background_scene_view.reset();  // drops FBO/RBO/texture in the view

    // 3) ImGui backend + GLFW teardown, still with context current.
    //    The ImGui context itself is destroyed once after the loop.
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwDestroyWindow(wd.glfw_window);
    wd.glfw_window = nullptr;
  }
  windows_.clear();

  ImGui::DestroyContext(imgui_ctx_);
  imgui_ctx_ = nullptr;

  glfwTerminate();

  MRPT_LOG_DEBUG("MolaVizImGui::gui_thread() done.");
}

// ---------------------------------------------------------------------------
// Per-frame rendering
// ---------------------------------------------------------------------------

void MolaVizImGui::render_frame(const window_name_t& name, PerWindowData& wd)
{
  glfwMakeContextCurrent(wd.glfw_window);

  int display_w, display_h;
  glfwGetFramebufferSize(wd.glfw_window, &display_w, &display_h);

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // Full-screen dockspace so every ImGui window can dock inside:
  ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::SetNextWindowViewport(viewport->ID);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

  constexpr ImGuiWindowFlags dockspace_flags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_MenuBar |
      ImGuiWindowFlags_NoBackground;  // PassthruCentralNode requires this

  ImGui::Begin("##DockSpaceRoot", nullptr, dockspace_flags);
  ImGui::PopStyleVar(3);

  // Menu bar inside the dockspace root:
  render_menu_bar(wd);

  ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
  ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
  ImGui::End();  // DockSpaceRoot

  // Drain pending tasks (they may call ImGui::Begin/End, GL rendering, etc.
  // so they must run inside an active GL context + ImGui frame):
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

  // Background 3D scene rendered into an ImGui image window:
  render_background_scene(wd);

  // Each registered sub-window:
  for (auto& [swName, sw] : wd.sub_windows)
  {
    // Skip "shell" subwindows that exist only to carry position/size hints
    // for a sensor handler (created by RawDataSourceBase::attach_sensor_viz).
    // The handler renders its own ImGui window keyed on the same title via
    // render_sensor_windows(), so drawing this one too would produce a
    // duplicate empty window next to the real content.
    if (sw.desc.tabs.empty() && wd.sensor_windows.count(swName))
    {
      continue;
    }
    render_subwindow(sw);
  }

  // Sensor observation windows — re-drawn every frame (immediate mode):
  render_sensor_windows(name, wd);

  // Console overlay (rendered as a transparent ImGui window):
  render_console_overlay(wd);

  // Decaying clouds alpha update:
  internal_handle_decaying_clouds(wd);

  // Compose and swap:
  ImGui::Render();
  glViewport(0, 0, display_w, display_h);
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(wd.glfw_window);
}

// ---------------------------------------------------------------------------
void MolaVizImGui::render_menu_bar(PerWindowData& wd)
{
  if (wd.menu_bar.menus.empty()) return;

  if (ImGui::BeginMenuBar())
  {
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
          {
            item.on_click();
          }
        }
        ImGui::EndMenu();
      }
    }
    ImGui::EndMenuBar();
  }
}

// ---------------------------------------------------------------------------
void MolaVizImGui::render_background_scene(PerWindowData& wd)
{
  std::lock_guard lk(wd.background_scene_mtx);

  if (!wd.background_scene || !wd.background_scene_view) return;

  // Sync scene pointer into the view (idempotent if already set).
  // Note: do NOT call setBackgroundColor() here — leaving it unset lets the
  // mrpt::opengl::Scene's natural viewport gradient show through; otherwise
  // CImGuiSceneView would override it with a flat dark fill on every frame.
  if (wd.background_scene_view->scene() != wd.background_scene)
  {
    wd.background_scene_view->setScene(wd.background_scene);
  }

  // Push cam_* into the view only when explicitly requested (initialization or
  // an API call such as update_viewport_look_at).  After every render we read
  // the camera back so mouse-driven orbit/pan/zoom persists across frames.
  if (wd.cam_dirty)
  {
    auto& cam = wd.background_scene_view->camera();
    cam.setAzimuthDegrees(wd.cam_azimuth_deg);
    cam.setElevationDegrees(wd.cam_elevation_deg);
    cam.setZoomDistance(wd.cam_zoom);
    cam.setPointingAt(wd.cam_look_at[0], wd.cam_look_at[1], wd.cam_look_at[2]);
    cam.setProjectiveModel(!wd.cam_orthographic);
    wd.cam_dirty = false;
  }

  // Size the background window to fill the entire OS window so the 3D scene
  // occupies the full viewport (dockable windows float on top of it).
  ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(viewport->WorkSize);

  // NoDocking: prevent this special full-screen window from accidentally
  // being docked into the dockspace (e.g. via a persisted .ini state).
  // NoSavedSettings: don't let imgui.ini persist position/size/collapse for
  // this window — those are meaningless for a full-screen background layer.
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                           ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
                           ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings;

  ImGui::SetNextWindowBgAlpha(0.0f);
  if (ImGui::Begin("##bg_scene", nullptr, flags))
  {
    // Drain stale GL errors so MRPT's CHECK_OPENGL_ERROR doesn't pick them up:
    while (glGetError() != GL_NO_ERROR)
    {
    }
    wd.background_scene_view->render();

    // Read back camera state so any mouse-driven orbit/pan/zoom made inside
    // render() (via CImGuiSceneView::handleMouseInteraction) persists to the
    // next frame.  Without this, cam_* would be pushed back unchanged next
    // frame and all user camera interaction would be silently discarded.
    const auto& cam      = wd.background_scene_view->camera();
    wd.cam_azimuth_deg   = cam.getAzimuthDegrees();
    wd.cam_elevation_deg = cam.getElevationDegrees();
    wd.cam_zoom          = cam.getZoomDistance();
    wd.cam_look_at[0]    = cam.getPointingAtX();
    wd.cam_look_at[1]    = cam.getPointingAtY();
    wd.cam_look_at[2]    = cam.getPointingAtZ();
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
void MolaVizImGui::render_console_overlay(PerWindowData& wd)
{
  if (wd.console_messages.empty())
  {
    return;
  }

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs |
                           ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoSavedSettings |
                           ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoBackground;

  ImGui::SetNextWindowPos(ImVec2(8.0f, 8.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(600.0f, 0.0f), ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(0.0f);
  ImGui::Begin("##console_overlay", nullptr, flags);

  const size_t N = wd.console_messages.size();
  for (size_t i = 0; i < N; i++)
  {
    // Fade oldest lines:
    const float t     = static_cast<float>(N - 1 - i) / static_cast<float>(std::max<size_t>(1u, N));
    const float alpha = 1.0f - t * t;
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, alpha));
    ImGui::TextUnformatted(wd.console_messages.at(i).c_str());
    ImGui::PopStyleColor();
  }

  ImGui::End();
}

// ---------------------------------------------------------------------------
void MolaVizImGui::render_subwindow(SubWindowState& sw)
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

  bool open = true;
  if (ImGui::Begin(desc.title.c_str(), &open)) render_widget_description(desc, sw);
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Decaying clouds
// ---------------------------------------------------------------------------

void MolaVizImGui::internal_handle_decaying_clouds(PerWindowData& wd)
{
  constexpr float FADE_OUT_FRACTION = 0.1f;

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
// get_subwindow_handle - returns nullptr (no retained ImGui window objects)
// ---------------------------------------------------------------------------

void* MolaVizImGui::get_subwindow_handle(
    const std::string& /*subWindowTitle*/, const std::string& /*parentWindow*/)
{
  // ImGui windows are identified by their title string, not by a pointer.
  // Use enqueue_custom_gui_code() + ImGui::Begin(title) for direct access.
  return nullptr;
}

// ---------------------------------------------------------------------------
// create_subwindow_from_description
// ---------------------------------------------------------------------------

std::future<void> MolaVizImGui::create_subwindow_from_description(
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

        auto& sw      = it->second.sub_windows[desc.title];
        sw.desc       = desc;
        sw.active_tab = 0;
        // FBO will be created lazily on first render.
      });
}

// ---------------------------------------------------------------------------
// set_menu_bar
// ---------------------------------------------------------------------------

std::future<void> MolaVizImGui::set_menu_bar(
    const mola::gui::MenuBar& bar, const std::string& parentWindow)
{
  return enqueue_custom_gui_code(
      [this, bar, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it != windows_.end()) it->second.menu_bar = bar;
      });
}

// ---------------------------------------------------------------------------
// open_file_dialog - delegates to NFD / tinyfiledialogs (stub)
// ---------------------------------------------------------------------------

std::future<std::optional<std::string>> MolaVizImGui::open_file_dialog(
    const std::string& title, bool save,
    const std::vector<std::pair<std::string, std::string>>& filters,
    const std::string& default_path, const std::string& /*parentWindow*/)
{
  using return_type = std::optional<std::string>;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      [this, title, save, filters, default_path]() -> return_type
      {
        // TODO: integrate ImGuiFileDialog or NFD here.
        // For now this is a stub that always returns nullopt.
        //
        // Example with NFD (https://github.com/btzy/nativefiledialog-extended):
        //   nfdchar_t* outPath = nullptr;
        //   nfdresult_t result = save
        //       ? NFD_SaveDialog(&outPath, nfdFilters.data(), nfdFilters.size(),
        //       default_path.c_str()) : NFD_OpenDialog(&outPath, nfdFilters.data(),
        //       nfdFilters.size(), default_path.c_str());
        //   if (result == NFD_OKAY) { std::string p(outPath); NFD_FreePath(outPath); return p; }
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
// 3-D scene API
// ---------------------------------------------------------------------------

std::future<bool> MolaVizImGui::update_3d_object(
    const std::string& objName, const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
    const std::string& viewportName, const std::string& parentWindow)
{
  using return_type = bool;
  auto task         = std::make_shared<std::packaged_task<return_type()>>(
      [this, objName, obj, viewportName, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }
        auto&           scene = it->second.background_scene;
        std::lock_guard lk(it->second.background_scene_mtx);

        mrpt::opengl::CSetOfObjects::Ptr container;
        if (auto o = scene->getByName(objName, viewportName); o)
        {
          container = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o);
        }
        else
        {
          container = mrpt::opengl::CSetOfObjects::Create();
          scene->insert(container, viewportName);
        }
        *container = *obj;
        container->setName(objName);
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGui::insert_point_cloud_with_decay(
    const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud, double decay_time_seconds,
    const std::string& viewportName, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, cloud, decay_time_seconds, viewportName, parentWindow]()
      {
        if (!cloud || cloud->empty())
        {
          return true;
        }
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }
        auto& wd = it->second;

        constexpr const char* DECAY_NAME = "__viz_decaying_clouds";
        std::lock_guard       lk(wd.background_scene_mtx);

        mrpt::opengl::CSetOfObjects::Ptr container;
        if (auto o = wd.background_scene->getByName(DECAY_NAME, viewportName); o)
        {
          container = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o);
        }
        else
        {
          container = mrpt::opengl::CSetOfObjects::Create();
          wd.background_scene->insert(container, viewportName);
          container->setName(DECAY_NAME);
        }
        container->insert(cloud);

        const size_t maxScans = std::max<size_t>(
            1u, static_cast<size_t>(std::round(decay_time_seconds * assumed_sensor_rate_hz_)));
        wd.max_decaying_clouds = maxScans;

        const float alpha = mrpt::u8tof(cloud->shaderPointsVertexColorBuffer().at(0).A);
        wd.decaying_clouds.emplace_back(viewportName, cloud, alpha);

        while (wd.decaying_clouds.size() > maxScans)
        {
          container->removeObject(wd.decaying_clouds.front().cloud);
          wd.decaying_clouds.pop_front();
        }
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGui::clear_all_point_clouds_with_decay(
    const std::string& viewportName, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, viewportName, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }
        auto&                 wd         = it->second;
        constexpr const char* DECAY_NAME = "__viz_decaying_clouds";
        std::lock_guard       lk(wd.background_scene_mtx);
        if (auto o = wd.background_scene->getByName(DECAY_NAME, viewportName); o)
        {
          std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o)->clear();
        }
        wd.decaying_clouds.clear();
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGui::update_viewport_look_at(
    const mrpt::math::TPoint3Df& lookAt, const std::string& /*viewportName*/,
    const std::string&           parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, lookAt, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }
        auto& wd          = it->second;
        wd.cam_look_at[0] = lookAt.x;
        wd.cam_look_at[1] = lookAt.y;
        wd.cam_look_at[2] = lookAt.z;
        wd.cam_dirty      = true;
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGui::update_viewport_camera_azimuth(
    double azimuth, bool absolute_falseForRelative, const std::string& /*viewportName*/,
    const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, azimuth, absolute_falseForRelative, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }
        auto& wd = it->second;
        if (absolute_falseForRelative)
        {
          wd.cam_azimuth_deg = static_cast<float>(mrpt::RAD2DEG(azimuth));
        }
        else
        {
          wd.cam_azimuth_deg += static_cast<float>(mrpt::RAD2DEG(azimuth));
        }
        wd.cam_dirty = true;
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGui::update_viewport_camera_orthographic(
    bool orthographic, const std::string& /*viewportName*/, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, orthographic, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }
        it->second.cam_orthographic = orthographic;
        it->second.cam_dirty        = true;
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGui::execute_custom_code_on_background_scene(
    const std::function<void(mrpt::opengl::Scene&)>& userCode, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, userCode, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }
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
// Console output
// ---------------------------------------------------------------------------

std::future<bool> MolaVizImGui::output_console_message(
    const std::string& message, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, message, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end())
        {
          return false;
        }
        auto& wd = it->second;

        std::vector<std::string> lines;
        mrpt::system::tokenize(message, "\r\n", lines);
        for (const auto& line : lines)
        {
          wd.console_messages.push_back(line);
          while (wd.console_messages.size() > max_console_lines_)
          {
            wd.console_messages.pop_front();
          }
        }
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// subwindow_update_visualization
// ---------------------------------------------------------------------------

std::future<bool> MolaVizImGui::subwindow_update_visualization(
    const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
    const mrpt::containers::yaml* extra_parameters, const std::string& parentWindow)
{
  // Safe copy of extra_parameters (caller's yaml may go out of scope).
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
        // Stash the latest observation for this title; the per-frame draw
        // pass (render_sensor_windows) is what actually invokes handlers,
        // so ImGui windows stay alive across frames.
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
// render_sensor_windows — called every frame from render_frame. Looks up the
// registered handler(s) for each stored observation's class and invokes them.
// Handlers carry their own per-window persistent state (FBOs, textures, …)
// in function-local statics keyed by ImGui window id, so calling them at
// display-rate with the same observation is idempotent and cheap enough for
// typical sensor rates.  This is the fix for the flicker: in immediate-mode
// ImGui, Begin/End must happen *every* frame or the window disappears.
// ---------------------------------------------------------------------------
void MolaVizImGui::render_sensor_windows(const window_name_t& parentName, PerWindowData& wd)
{
  auto&           hc = ImGuiHandlersContainer::Instance();
  std::lock_guard lk(hc.mtx);

  for (auto& [title, slot] : wd.sensor_windows)
  {
    if (!slot.obj) continue;
    try
    {
      // Propagate position/size hints from the paired description-subwindow
      // (if any) to the first Begin() the handler will issue this frame.
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
      {
        it->second(slot.obj, nullptr, parentName, title, this, slot.extra.get());
      }
    }
    catch (const std::exception& e)
    {
      MRPT_LOG_ERROR_STREAM("render_sensor_windows: handler threw: " << e.what());
    }
  }
}

// ---------------------------------------------------------------------------
// Dataset UI
// ---------------------------------------------------------------------------

void MolaVizImGui::dataset_ui_check_new_modules()
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

    e.first_time_seen    = false;
    e.module             = modUI;
    e.lbPlaybackPosition = std::make_shared<mola::gui::LiveString>("Progress: ");

    std::weak_ptr<Dataset_UI> weakMod = modUI;

    const std::vector<float>       rates       = {0.1f,  0.25f, 0.5f, 0.75f, 1.0f,
                                                  1.25f, 1.5f,  2.0f, 3.0f,  5.0f};
    const std::vector<std::string> labels      = {"x0.1",  "x0.25", "x0.5", "x0.75", "x1.0",
                                                  "x1.25", "x1.5",  "x2.0", "x3.0",  "x5.0"};
    int                            selIdx      = 4;
    double                         initialRate = modUI->datasetUI_playback_speed();
    for (size_t i = 0; i < rates.size(); i++)
    {
      if (std::abs(rates[i] - static_cast<float>(initialRate)) < 1e-3f)
      {
        selIdx = static_cast<int>(i);
        break;
      }
    }

    mola::gui::WindowDescription desc;
    desc.title    = module->getModuleInstanceName();
    desc.position = {300, 5};
    desc.size     = {650, 70};

    mola::gui::Tab tab{"Controls", {}};
    mola::gui::Row row;

    row.widgets.emplace_back(mola::gui::CheckBox{
        "Paused", modUI->datasetUI_paused(),
        [weakMod](bool v)
        {
          if (auto m = weakMod.lock()) m->datasetUI_paused(v);
        }});

    row.widgets.emplace_back(mola::gui::Label{e.lbPlaybackPosition});

    row.widgets.emplace_back(mola::gui::SliderFloat{
        "", static_cast<float>(modUI->datasetUI_lastQueriedTimestep()), 0.0f,
        static_cast<float>(std::max<size_t>(1u, modUI->datasetUI_size())), "%.0f",
        [weakMod](float v)
        {
          if (auto m = weakMod.lock()) m->datasetUI_teleport(static_cast<size_t>(v));
        }});

    row.widgets.emplace_back(
        mola::gui::Label{std::make_shared<mola::gui::LiveString>("Playback rate:")});

    row.widgets.emplace_back(mola::gui::ComboBox{
        "", labels, selIdx,
        [weakMod, rates](int idx)
        {
          if (auto m = weakMod.lock())
            m->datasetUI_playback_speed(static_cast<double>(rates.at(idx)));
        }});

    tab.widgets.emplace_back(std::move(row));
    desc.tabs.emplace_back(std::move(tab));

    create_subwindow_from_description(desc);
  }
}

void MolaVizImGui::dataset_ui_update()
{
  for (auto& [name, e] : datasetUIs_)
  {
    if (e.module.expired())
    {
      continue;
    }
    auto mod = e.module.lock();
    if (!mod || !e.lbPlaybackPosition)
    {
      continue;
    }
    const size_t pos = mod->datasetUI_lastQueriedTimestep();
    const size_t N   = mod->datasetUI_size();
    e.lbPlaybackPosition->set(mrpt::format("%zu / %zu", pos, N));
  }
}

// ---------------------------------------------------------------------------
// Deprecated stubs
// ---------------------------------------------------------------------------

std::future<nanogui::Window*> MolaVizImGui::create_subwindow(
    const std::string& title, const std::string& parentWindow)
{
  // Create the description-based window, then return nullptr as the "handle".
  mola::gui::WindowDescription desc;
  desc.title = title;
  create_subwindow_from_description(desc, parentWindow);

  std::promise<nanogui::Window*> p;
  p.set_value(nullptr);
  return p.get_future();
}

std::future<void> MolaVizImGui::enqueue_custom_nanogui_code(const std::function<void()>& userCode)
{
  return enqueue_custom_gui_code(userCode);
}

std::future<void> MolaVizImGui::subwindow_grid_layout(
    const std::string&, bool, int, const std::string&)
{
  // No-op: ImGui manages layout automatically.
  std::promise<void> p;
  p.set_value();
  return p.get_future();
}

std::future<void> MolaVizImGui::subwindow_move_resize(
    const std::string&, const mrpt::math::TPoint2D_<int>&, const mrpt::math::TPoint2D_<int>&,
    const std::string&)
{
  // No-op: position/size are first-use hints encoded in WindowDescription.
  std::promise<void> p;
  p.set_value();
  return p.get_future();
}
