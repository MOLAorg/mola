/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MolaViz.cpp
 * @brief  Main C++ class for MOLA GUI
 * @author Jose Luis Blanco Claraco
 * @date   May  11, 2019
 */

/** \defgroup mola_viz_grp mola-viz
 * C++ library for main MOLA GUI
 */

#include <mola-viz/MolaViz.h>
#include <mola-yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/CGridPlaneXY.h>

using namespace mola;

IMPLEMENTS_MRPT_OBJECT(MolaViz, ExecutableBase, mola)

MRPT_INITIALIZER(do_register_MolaViz) { MOLA_REGISTER_MODULE(MolaViz); }

MolaViz*                     MolaViz::instance_ = nullptr;
std::shared_mutex            MolaViz::instanceMtx_;
const MolaViz::window_name_t MolaViz::DEFAULT_WINDOW_NAME = "main";

MolaViz::MolaViz() {}

MolaViz::~MolaViz()
{
    instanceMtx_.lock();
    instance_ = nullptr;
    instanceMtx_.unlock();

    nanogui::leave();
    if (guiThread_.joinable()) guiThread_.join();
}

bool     MolaViz::IsRunning() { return Instance() != nullptr; }
MolaViz* MolaViz::Instance()
{
    instanceMtx_.lock_shared();
    auto ret = instance_;
    instanceMtx_.unlock_shared();
    return ret;
}

void MolaViz::initialize(const std::string& cfg_block)
{
    MRPT_START

    // Load:
    auto c   = mrpt::containers::yaml::FromText(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    // Mark as initialized and up:
    instanceMtx_.lock();
    instance_ = this;
    instanceMtx_.unlock();

    guiThread_ = std::thread(&MolaViz::gui_thread, this);

    MRPT_END
}

void MolaViz::spinOnce()
{
    // MRPT_START
    // xx
    // MRPT_END
}

mrpt::gui::CDisplayWindowGUI::Ptr MolaViz::create_and_add_window(
    const window_name_t& name)
{
    MRPT_LOG_DEBUG_FMT("Creating new window `%s`", name.c_str());

    mrpt::gui::CDisplayWindowGUI_Params cp;
    cp.maximized   = true;
    windows_[name] = mrpt::gui::CDisplayWindowGUI::Create(name, 1000, 800, cp);

    auto& win = windows_[name];

    // Add a background scene:
    auto scene = mrpt::opengl::COpenGLScene::Create();

    scene->insert(mrpt::opengl::CGridPlaneXY::Create());

    {
        std::lock_guard<std::mutex> lck(win->background_scene_mtx);
        win->background_scene = std::move(scene);
    }

    win->performLayout();
    auto& cam = win->camera();
    cam.setCameraPointing(8.0f, .0f, .0f);
    cam.setAzimuthDegrees(110.0f);
    cam.setElevationDegrees(15.0f);
    cam.setZoomDistance(20.0f);

    win->drawAll();
    win->setVisible(true);

    return win;
}

void MolaViz::gui_thread()
{
    MRPT_LOG_DEBUG("gui_thread() started.");

    nanogui::init();

    // Open first GUI window:
    auto w = create_and_add_window(DEFAULT_WINDOW_NAME);

    // Tasks pending to be run before each refresh:
    w->setLoopCallback([this]() {
        // Get a copy of the tasks:
        task_queue_t tasks;
        auto         lck       = mrpt::lockHelper(guiThreadPendingTasksMtx_);
        tasks                  = std::move(guiThreadPendingTasks_);
        guiThreadPendingTasks_ = task_queue_t();
        lck.unlock();

        // Run them:
        for (auto& t : tasks)
        {
            try
            {
                t();
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM(
                    "Exception in tasks sent to GUI thread:\n"
                    << e.what());
            }
        }
    });

    nanogui::mainloop(100 /*refresh milliseconds*/);

    nanogui::shutdown();

    MRPT_LOG_DEBUG("gui_thread() quitted.");
}

bool MolaViz::subwindow_update_visualization(
    const std::string& subWindowTitle, const mrpt::rtti::CObject::Ptr& obj)
{
    return false;
}

void MolaViz::create_subwindow(
    const std::string& title, const std::string& parentWindow)
{
    auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
    guiThreadPendingTasks_.emplace_back([=]() {
        MRPT_LOG_DEBUG_STREAM(
            "create_subwindow() title='" << title << "' inside toplevel '"
                                         << parentWindow << "'");

        ASSERT_(windows_.count(parentWindow));
        auto topWin = windows_.at(parentWindow);
        ASSERT_(topWin);

        auto subw = topWin->createManagedSubWindow(title);
        subw->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical));
    });
}
