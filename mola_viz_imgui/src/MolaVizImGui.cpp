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
 * @brief  MOLA module shell — lifecycle, GLFW window, Dataset_UI panels.
 *         All rendering and VizInterface logic lives in MolaVizImGuiCore.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

/** \defgroup mola_viz_imgui_grp mola-viz-imgui
 * C++ library for the Dear ImGui MOLA GUI backend
 */

#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <mola_kernel/assets/mola_icon_64x64.h>
#include <mola_viz_imgui/MolaVizImGui.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/thread_name.h>

#include <stdexcept>
#include <vector>

using namespace mola;

IMPLEMENTS_MRPT_OBJECT(MolaVizImGui, ExecutableBase, mola)

// ---------------------------------------------------------------------------
// Static constant definitions
// ---------------------------------------------------------------------------

// NOTE: initialized from the compile-time literal, NOT copied from
// MolaVizImGuiCore::DEFAULT_WINDOW_NAME: that object lives in another
// translation unit and the initialization order between them is unspecified.
const MolaVizImGui::window_name_t MolaVizImGui::DEFAULT_WINDOW_NAME =
    MolaVizImGuiCore::DEFAULT_WINDOW_NAME_LITERAL;

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
// Embed-mode install
// ---------------------------------------------------------------------------

std::shared_ptr<MolaVizImGuiCore> MolaVizImGui::s_embed_core_;
std::mutex                        MolaVizImGui::s_embed_core_mtx_;

void MolaVizImGui::install_embed_core(std::shared_ptr<MolaVizImGuiCore> core)
{
  std::lock_guard lk(s_embed_core_mtx_);
  s_embed_core_ = std::move(core);
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

MolaVizImGui::MolaVizImGui()
{
  // Allocate a default core; may be replaced by an embed core in initialize().
  core_ptr_ = std::make_shared<MolaVizImGuiCore>();
}

MolaVizImGui::~MolaVizImGui()
{
  if (!embed_mode_)
  {
    guiThreadShutdown_.store(true);
    if (guiThread_.joinable()) guiThread_.join();
  }

  std::lock_guard lk(instanceMtx_);
  instance_ = nullptr;
}

// ---------------------------------------------------------------------------
// initialize / spinOnce
// ---------------------------------------------------------------------------

void MolaVizImGui::initialize(const Yaml& c)
{
  MRPT_START

  // Consume any pre-installed embed core (one-shot).
  {
    std::lock_guard lk(s_embed_core_mtx_);
    if (s_embed_core_)
    {
      core_ptr_   = std::move(s_embed_core_);
      embed_mode_ = true;
      MRPT_LOG_INFO("MolaVizImGui: running in embed mode (no GLFW window created).");
    }
  }

  auto cfg = c["params"];
  MRPT_LOG_DEBUG_STREAM("MolaVizImGui: loading params:\n" << cfg);

  core_ptr_->show_rgbd_as_point_cloud_ =
      cfg.getOrDefault("show_rgbd_as_point_cloud", core_ptr_->show_rgbd_as_point_cloud_);
  core_ptr_->assumed_sensor_rate_hz_ =
      cfg.getOrDefault("assumed_sensor_rate_hz", core_ptr_->assumed_sensor_rate_hz_);
  core_ptr_->target_fps_     = cfg.getOrDefault("target_fps", core_ptr_->target_fps_);
  core_ptr_->imgui_app_name_ = cfg.getOrDefault("imgui_app_name", core_ptr_->imgui_app_name_);

  core_ptr_->console_enabled_ = cfg.getOrDefault("console_enabled", core_ptr_->console_enabled_);
  core_ptr_->console_max_entries_ =
      cfg.getOrDefault("console_max_entries", core_ptr_->console_max_entries_);
  core_ptr_->console_window_seconds_ =
      cfg.getOrDefault("console_window_seconds", core_ptr_->console_window_seconds_);

  core_ptr_->plots_enabled_ = cfg.getOrDefault("plots_enabled", core_ptr_->plots_enabled_);
  core_ptr_->plots_default_retention_seconds_ = cfg.getOrDefault(
      "plots_default_retention_seconds", core_ptr_->plots_default_retention_seconds_);
  core_ptr_->plots_default_span_seconds_ =
      cfg.getOrDefault("plots_default_span_seconds", core_ptr_->plots_default_span_seconds_);

  core_ptr_->menu_bar_enabled_ = cfg.getOrDefault("menu_bar_enabled", core_ptr_->menu_bar_enabled_);

  core_ptr_->console_sink_->max_entries    = core_ptr_->console_max_entries_;
  core_ptr_->console_sink_->window_seconds = core_ptr_->console_window_seconds_;

  {
    std::lock_guard lk(instanceMtx_);
    instance_ = this;
  }

  core_ptr_->quit_callback_ = [this]() { this->requestShutdown(); };

  if (!embed_mode_)
  {
    guiThread_ = std::thread(&MolaVizImGui::gui_thread, this);
  }

  MRPT_END
}

void MolaVizImGui::spinOnce()
{
  // Dataset UI updates run in both host and embed mode.
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

  if (core_ptr_->console_enabled_ && tNow - lastTimeCheckForConsoleModules_ > PERIOD_CHECK_NEW_MODS)
  {
    console_check_new_modules();
    lastTimeCheckForConsoleModules_ = tNow;
  }
}

// ---------------------------------------------------------------------------
// GUI thread (host mode only)
// ---------------------------------------------------------------------------

MolaVizImGuiCore::PerWindowData& MolaVizImGui::create_and_add_window(const window_name_t& name)
{
  ASSERT_(core_ptr_->windows_.empty());

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

  std::string title = "MOLAViz ImGui - " + name;
  if (core_ptr_->imgui_app_name_ != "default")
  {
    title += " [" + core_ptr_->imgui_app_name_ + "]";
  }
  GLFWwindow* win = glfwCreateWindow(1280, 800, title.c_str(), nullptr, nullptr);
  if (!win) throw std::runtime_error("MolaVizImGui: glfwCreateWindow failed");

  {
    // The GIMP header format used by mola_icon_data has no alpha channel, so
    // the (0,0) corner pixel color is used as the transparent color key.
    std::vector<unsigned char> rgba(mola_icon_width * mola_icon_height * 4);
    const char*                data = mola_icon_data;

    unsigned char transparentRgb[3];
    {
      const char* cornerData = mola_icon_data;
      HEADER_PIXEL(cornerData, transparentRgb);
    }

    for (unsigned int i = 0; i < mola_icon_width * mola_icon_height; i++)
    {
      unsigned char rgb[3];
      HEADER_PIXEL(data, rgb);
      rgba[4 * i + 0] = rgb[0];
      rgba[4 * i + 1] = rgb[1];
      rgba[4 * i + 2] = rgb[2];
      const bool isTransparent =
          rgb[0] == transparentRgb[0] && rgb[1] == transparentRgb[1] && rgb[2] == transparentRgb[2];
      rgba[4 * i + 3] = isTransparent ? 0x00 : 0xff;
    }
    const GLFWimage icon_image{
        static_cast<int>(mola_icon_width), static_cast<int>(mola_icon_height), rgba.data()};
    glfwSetWindowIcon(win, 1, &icon_image);
  }

  glfwMakeContextCurrent(win);
  glfwSwapInterval(0);

  auto& wd = core_ptr_->init_window(name, win);

  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  const std::string resolvedIni = core_ptr_->resolve_imgui_ini_path(name);
  if (!resolvedIni.empty())
  {
    auto [it, inserted] = core_ptr_->imgui_ini_paths_.emplace(name, resolvedIni);
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

  imgui_ctx_ = ImGui::CreateContext();
  ImGui::SetCurrentContext(imgui_ctx_);
  implot_ctx_ = ImPlot::CreateContext();

  create_and_add_window(DEFAULT_WINDOW_NAME);

  const double frame_period = 1.0 / static_cast<double>(std::max(1, core_ptr_->target_fps_));

  while (!guiThreadShutdown_.load())
  {
    const double t0 = mrpt::Clock::nowDouble();

    glfwPollEvents();

    bool any_open = false;
    for (auto& [name, wd] : core_ptr_->windows_)
      if (wd.glfw_window && !glfwWindowShouldClose(wd.glfw_window)) any_open = true;
    if (!any_open) break;

    for (auto& [name, wd] : core_ptr_->windows_)
    {
      if (!wd.glfw_window || glfwWindowShouldClose(wd.glfw_window)) continue;
      core_ptr_->render_frame(name, wd);
    }

    const double elapsed = mrpt::Clock::nowDouble() - t0;
    if (elapsed < frame_period)
      std::this_thread::sleep_for(std::chrono::duration<double>(frame_period - elapsed));
  }

  // Cleanup — GL resources must be released while the context is still current.
  for (auto& [name, wd] : core_ptr_->windows_)
  {
    if (!wd.glfw_window) continue;
    glfwMakeContextCurrent(wd.glfw_window);

    core_ptr_->run_registered_cleanups();

    wd.sensor_windows.clear();
    wd.decaying_clouds.clear();
    wd.background_scene.reset();
    wd.background_scene_view.reset();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwDestroyWindow(wd.glfw_window);
    wd.glfw_window = nullptr;
  }
  core_ptr_->windows_.clear();

  ImPlot::DestroyContext(implot_ctx_);
  implot_ctx_ = nullptr;

  ImGui::DestroyContext(imgui_ctx_);
  imgui_ctx_ = nullptr;

  glfwTerminate();

  MRPT_LOG_DEBUG("MolaVizImGui::gui_thread() done.");
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
    if (!e.first_time_seen) continue;

    e.first_time_seen    = false;
    e.module             = modUI;
    e.lbPlaybackPosition = std::make_shared<mola::gui::LiveString>("Progress: ");
    e.liveSliderPos      = std::make_shared<mola::gui::LiveFloat>(
        static_cast<float>(modUI->datasetUI_lastQueriedTimestep()));

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
    desc.title               = module->getModuleInstanceName();
    desc.position            = {300, 5};
    desc.size                = {650, 70};
    desc.dock_top_by_default = true;

    mola::gui::Tab tab{"Controls", {}};
    mola::gui::Row row;

    row.widgets.emplace_back(mola::gui::CheckBox{
        "Paused", modUI->datasetUI_paused(),
        [weakMod](bool v)
        {
          if (auto m = weakMod.lock()) m->datasetUI_paused(v);
        }});

    row.widgets.emplace_back(mola::gui::Label{e.lbPlaybackPosition});

    {
      mola::gui::SliderFloat sf;
      sf.label         = "";
      sf.initial_value = static_cast<float>(modUI->datasetUI_lastQueriedTimestep());
      sf.min_value     = 0.0f;
      sf.max_value     = static_cast<float>(std::max<size_t>(1u, modUI->datasetUI_size()));
      sf.format_string = "%.0f";
      sf.live_value    = e.liveSliderPos;
      sf.on_change     = [weakMod](float v)
      {
        if (auto m = weakMod.lock()) m->datasetUI_teleport(static_cast<size_t>(v));
      };
      row.widgets.emplace_back(std::move(sf));
    }

    row.widgets.emplace_back(
        mola::gui::Label{std::make_shared<mola::gui::LiveString>("Playback rate:")});

    {
      mola::gui::ComboBox cb;
      cb.label         = "";
      cb.items         = labels;
      cb.initial_index = selIdx;
      cb.on_change     = [weakMod, rates](int idx)
      {
        if (auto m = weakMod.lock())
          m->datasetUI_playback_speed(static_cast<double>(rates.at(idx)));
      };
      cb.fixed_width = 80;
      row.widgets.emplace_back(std::move(cb));
    }

    tab.widgets.emplace_back(std::move(row));
    desc.tabs.emplace_back(std::move(tab));

    core_ptr_->create_subwindow_from_description(desc);
  }
}

void MolaVizImGui::dataset_ui_update()
{
  for (auto& [name, e] : datasetUIs_)
  {
    if (e.module.expired()) continue;
    auto mod = e.module.lock();
    if (!mod || !e.lbPlaybackPosition) continue;
    const size_t pos = mod->datasetUI_lastQueriedTimestep();
    const size_t N   = mod->datasetUI_size();
    e.lbPlaybackPosition->set(mrpt::format("%zu / %zu", pos, N));
    if (e.liveSliderPos) e.liveSliderPos->set(static_cast<float>(pos));
  }
}

// ---------------------------------------------------------------------------
// Console window: log capture from all discovered ExecutableBase modules
// ---------------------------------------------------------------------------

void MolaVizImGui::console_check_new_modules()
{
  auto sink = core_ptr_->console_sink_;
  if (!sink) return;

  // Read once per tick: the user may have changed it at runtime via the
  // Console window's "Capture" combo (MolaVizImGuiCore::render_console_window).
  const auto captureLevel = core_ptr_->console_capture_level_.load();

  for (auto& module : findService<ExecutableBase>())
  {
    if (module.get() == this) continue;  // never hook ourselves (recursion)

    // Re-applied every tick (not just at hook time) so a runtime change to
    // the capture level propagates to already-hooked modules too. Does not
    // change what actually prints to the terminal, only what's forwarded to
    // the console callback -- see setVerbosityLevelForCallbacks() docs.
    module->setVerbosityLevelForCallbacks(captureLevel);

    const std::string name = module->getModuleInstanceName();
    if (!consoleHookedModules_.count(name))
    {
      consoleHookedModules_.insert(name);
      sink->note_source(name);

      // shared_ptr capture keeps the sink alive as long as the module's
      // logger holds this callback; 'name' (not loggerName) is captured for
      // a stable id.
      module->logRegisterCallback(
          [sink, name](
              std::string_view msg, mrpt::system::VerbosityLevel       level,
              std::string_view /*loggerName*/, mrpt::Clock::time_point timestamp)
          {
            std::vector<std::string> lines;
            mrpt::system::tokenize(std::string(msg), "\r\n", lines);
            if (lines.empty()) lines.push_back(std::string(msg));

            for (const auto& line : lines)
            {
              ConsoleLogEntry e;
              e.timestamp = timestamp;
              e.level     = level;
              e.source    = name;
              e.text      = line;
              sink->push(e);
            }
          });
    }

    // Sub-loggers the module owns but does not log through itself (e.g. a
    // library engine run on its own background thread). Hooked the same
    // way, tagged with their own source name, so their output shows up in
    // the console window without the module having to relay each message.
    for (const auto& [childName, childLogger] : module->child_loggers())
    {
      if (!childLogger) continue;

      // Re-applied every tick, same reasoning as for the module above.
      childLogger->setVerbosityLevelForCallbacks(captureLevel);

      if (consoleHookedChildLoggers_.count(childLogger)) continue;
      consoleHookedChildLoggers_.insert(childLogger);

      const std::string fullName = name + "/" + childName;
      sink->note_source(fullName);

      childLogger->logRegisterCallback(
          [sink, fullName](
              std::string_view msg, mrpt::system::VerbosityLevel       level,
              std::string_view /*loggerName*/, mrpt::Clock::time_point timestamp)
          {
            std::vector<std::string> lines;
            mrpt::system::tokenize(std::string(msg), "\r\n", lines);
            if (lines.empty()) lines.push_back(std::string(msg));

            for (const auto& line : lines)
            {
              ConsoleLogEntry e;
              e.timestamp = timestamp;
              e.level     = level;
              e.source    = fullName;
              e.text      = line;
              sink->push(e);
            }
          });
    }
  }
}
