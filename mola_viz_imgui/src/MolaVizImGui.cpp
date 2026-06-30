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
#include <mola_kernel/assets/mola_icon_64x64.h>
#include <mola_viz_imgui/MolaVizImGui.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/system/thread_name.h>

#include <stdexcept>
#include <vector>

using namespace mola;

IMPLEMENTS_MRPT_OBJECT(MolaVizImGui, ExecutableBase, mola)

// ---------------------------------------------------------------------------
// Static constant definitions
// ---------------------------------------------------------------------------

const MolaVizImGui::window_name_t MolaVizImGui::DEFAULT_WINDOW_NAME =
    MolaVizImGuiCore::DEFAULT_WINDOW_NAME;

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

  core_ptr_->max_console_lines_ =
      cfg.getOrDefault("max_console_lines", core_ptr_->max_console_lines_);
  core_ptr_->console_text_font_size_ =
      cfg.getOrDefault("console_text_font_size", core_ptr_->console_text_font_size_);
  core_ptr_->show_rgbd_as_point_cloud_ =
      cfg.getOrDefault("show_rgbd_as_point_cloud", core_ptr_->show_rgbd_as_point_cloud_);
  core_ptr_->assumed_sensor_rate_hz_ =
      cfg.getOrDefault("assumed_sensor_rate_hz", core_ptr_->assumed_sensor_rate_hz_);
  core_ptr_->target_fps_     = cfg.getOrDefault("target_fps", core_ptr_->target_fps_);
  core_ptr_->imgui_app_name_ = cfg.getOrDefault("imgui_app_name", core_ptr_->imgui_app_name_);

  {
    std::lock_guard lk(instanceMtx_);
    instance_ = this;
  }

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

  const std::string title = "MOLAViz ImGui - " + name;
  GLFWwindow*       win   = glfwCreateWindow(1280, 800, title.c_str(), nullptr, nullptr);
  if (!win) throw std::runtime_error("MolaVizImGui: glfwCreateWindow failed");

  {
    std::vector<unsigned char> rgba(mola_icon_width * mola_icon_height * 4);
    const char*                data = mola_icon_data;
    for (unsigned int i = 0; i < mola_icon_width * mola_icon_height; i++)
    {
      unsigned char rgb[3];
      HEADER_PIXEL(data, rgb);
      rgba[4 * i + 0] = rgb[0];
      rgba[4 * i + 1] = rgb[1];
      rgba[4 * i + 2] = rgb[2];
      rgba[4 * i + 3] = 0xff;
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

    row.widgets.emplace_back(mola::gui::ComboBox{
        "", labels, selIdx,
        [weakMod, rates](int idx)
        {
          if (auto m = weakMod.lock())
            m->datasetUI_playback_speed(static_cast<double>(rates.at(idx)));
        }});

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
