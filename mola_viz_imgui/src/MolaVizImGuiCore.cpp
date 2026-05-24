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
#include <mola_viz_imgui/MolaVizImGuiCore.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>

using namespace mola;

// ---------------------------------------------------------------------------
// Static constant
// ---------------------------------------------------------------------------

const MolaVizImGuiCore::window_name_t MolaVizImGuiCore::DEFAULT_WINDOW_NAME = "main";

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

MolaVizImGuiCore::MolaVizImGuiCore() { setLoggerName("MolaVizImGuiCore"); }

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
  wd.background_scene = mrpt::opengl::COpenGLScene::Create();
  // No background_scene_view — the embed host renders via its own CImGuiSceneView.
  embed_active_ = true;
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

  embed_active_ = false;
}

MolaVizImGuiCore::PerWindowData& MolaVizImGuiCore::init_window(
    const window_name_t& name, GLFWwindow* win)
{
  auto& wd                 = windows_[name];
  wd.glfw_window           = win;
  wd.background_scene      = mrpt::opengl::COpenGLScene::Create();
  wd.background_scene_view = std::make_unique<mrpt::imgui::CImGuiSceneView>();
  return wd;
}

std::shared_ptr<mrpt::opengl::COpenGLScene> MolaVizImGuiCore::get_background_scene(
    const window_name_t& name)
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
  render_console_overlay(wd);
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

  constexpr ImGuiWindowFlags dockspace_flags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_MenuBar |
      ImGuiWindowFlags_NoBackground;

  ImGui::Begin("##DockSpaceRoot", nullptr, dockspace_flags);
  ImGui::PopStyleVar(3);

  render_menu_bar(wd);

  ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
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
  render_console_overlay(wd);
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

void MolaVizImGuiCore::render_menu_bar(PerWindowData& wd)
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
            item.on_click();
        }
        ImGui::EndMenu();
      }
    }
    ImGui::EndMenuBar();
  }
}

void MolaVizImGuiCore::render_background_scene(PerWindowData& wd)
{
  std::lock_guard lk(wd.background_scene_mtx);

  if (!wd.background_scene || !wd.background_scene_view) return;

  if (wd.background_scene_view->scene() != wd.background_scene)
    wd.background_scene_view->setScene(wd.background_scene);

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

void MolaVizImGuiCore::render_console_overlay(PerWindowData& wd)
{
  if (wd.console_messages.empty()) return;

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
    const float t     = static_cast<float>(N - 1 - i) / static_cast<float>(std::max<size_t>(1u, N));
    const float alpha = 1.0f - t * t;
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, alpha));
    ImGui::TextUnformatted(wd.console_messages.at(i).c_str());
    ImGui::PopStyleColor();
  }

  ImGui::End();
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
        auto& sw      = it->second.sub_windows[desc.title];
        sw.desc       = desc;
        sw.active_tab = 0;
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

std::future<bool> MolaVizImGuiCore::update_3d_object(
    const std::string& objName, const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
    const std::string& viewportName, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, objName, obj, viewportName, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;

        auto&           scene = it->second.background_scene;
        std::lock_guard lk(it->second.background_scene_mtx);

        mrpt::opengl::CSetOfObjects::Ptr container;
        if (auto o = scene->getByName(objName, viewportName); o)
          container = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o);
        if (!container)
        {
          // Either the name was unused, or an object of a different type is
          // stored under it; (re)create a CSetOfObjects (insert overwrites by
          // name) so we never dereference a null container below.
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

std::future<bool> MolaVizImGuiCore::insert_point_cloud_with_decay(
    const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud, double decay_time_seconds,
    const std::string& viewportName, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, cloud, decay_time_seconds, viewportName, parentWindow]()
      {
        if (!cloud || cloud->empty()) return true;

        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;

        auto& wd = it->second;

        constexpr const char* DECAY_NAME = "__viz_decaying_clouds";
        std::lock_guard       lk(wd.background_scene_mtx);

        mrpt::opengl::CSetOfObjects::Ptr container;
        if (auto o = wd.background_scene->getByName(DECAY_NAME, viewportName); o)
          container = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o);
        if (!container)
        {
          // Either the reserved name was never used, or someone inserted a
          // non-CSetOfObjects under it.  Recreate fresh (overwrites by name).
          container = mrpt::opengl::CSetOfObjects::Create();
          wd.background_scene->insert(container, viewportName);
          container->setName(DECAY_NAME);
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
          if (auto c = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(o); c) c->clear();
        wd.decaying_clouds.clear();
        return true;
      });

  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

std::future<bool> MolaVizImGuiCore::update_viewport_look_at(
    const mrpt::math::TPoint3Df& lookAt, const std::string& /*viewportName*/,
    const std::string&           parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, lookAt, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;
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
    const std::function<void(mrpt::opengl::Scene&)>& userCode, const std::string& parentWindow)
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
    const std::string& message, const std::string& parentWindow)
{
  auto task = std::make_shared<std::packaged_task<bool()>>(
      [this, message, parentWindow]()
      {
        auto it = windows_.find(parentWindow);
        if (it == windows_.end()) return false;
        auto& wd = it->second;

        std::vector<std::string> lines;
        mrpt::system::tokenize(message, "\r\n", lines);
        for (const auto& line : lines)
        {
          wd.console_messages.push_back(line);
          while (wd.console_messages.size() > max_console_lines_) wd.console_messages.pop_front();
        }
        return true;
      });
  std::lock_guard lk(guiThreadPendingTasksMtx_);
  guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
  return task->get_future();
}

// ---------------------------------------------------------------------------
// VizInterface: deprecated stubs
// ---------------------------------------------------------------------------

std::future<nanogui::Window*> MolaVizImGuiCore::create_subwindow(
    const std::string& title, const std::string& parentWindow)
{
  mola::gui::WindowDescription desc;
  desc.title = title;
  create_subwindow_from_description(desc, parentWindow);

  std::promise<nanogui::Window*> p;
  p.set_value(nullptr);
  return p.get_future();
}

std::future<void> MolaVizImGuiCore::enqueue_custom_nanogui_code(const std::function<void()>& fn)
{
  return enqueue_custom_gui_code(fn);
}

std::future<void> MolaVizImGuiCore::subwindow_grid_layout(
    const std::string&, bool, int, const std::string&)
{
  std::promise<void> p;
  p.set_value();
  return p.get_future();
}

std::future<void> MolaVizImGuiCore::subwindow_move_resize(
    const std::string&, const mrpt::math::TPoint2D_<int>&, const mrpt::math::TPoint2D_<int>&,
    const std::string&)
{
  std::promise<void> p;
  p.set_value();
  return p.get_future();
}
