/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
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
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/thread_name.h>
#include <mrpt/version.h>

using namespace mola;

IMPLEMENTS_MRPT_OBJECT(MolaViz, ExecutableBase, mola)

struct HandlersContainer
{
    static HandlersContainer& Instance()
    {
        static HandlersContainer o;
        return o;
    }

    std::map<MolaViz::class_name_t, MolaViz::update_handler_t> guiHandlers_;
    std::mutex                                                 guiHandlersMtx_;

   private:
    HandlersContainer() = default;
};

// CObservationImage
void gui_handler_images(
    const mrpt::rtti::CObject::Ptr&, nanogui::Window*,
    MolaViz::window_name_t parentWin, MolaViz* instance);

// CObservationPointCloud
void gui_handler_point_cloud(
    const mrpt::rtti::CObject::Ptr&, nanogui::Window*,
    MolaViz::window_name_t parentWin, MolaViz* instance);

MRPT_INITIALIZER(do_register_MolaViz)
{
    // Register MOLA module:
    MOLA_REGISTER_MODULE(MolaViz);

    // Register GUI handlers for common sensor types:
    MolaViz::register_gui_handler(
        "mrpt::obs::CObservationImage", &gui_handler_images);
    MolaViz::register_gui_handler(
        "mrpt::obs::CObservationPointCloud", &gui_handler_point_cloud);
    MolaViz::register_gui_handler(
        "mrpt::obs::CObservation3DRangeScan", &gui_handler_point_cloud);
    MolaViz::register_gui_handler(
        "mrpt::obs::CObservation2DRangeScan", &gui_handler_point_cloud);
}

MolaViz*                     MolaViz::instance_ = nullptr;
std::shared_mutex            MolaViz::instanceMtx_;
const MolaViz::window_name_t MolaViz::DEFAULT_WINDOW_NAME = "main";

void MolaViz::register_gui_handler(class_name_t name, update_handler_t handler)
{
    auto& hc              = HandlersContainer::Instance();
    auto  lck             = mrpt::lockHelper(hc.guiHandlersMtx_);
    hc.guiHandlers_[name] = handler;
}

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

void MolaViz::initialize(const Yaml& c)
{
    MRPT_START

    // Load:
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
    // Nothing to do.
}

mrpt::gui::CDisplayWindowGUI::Ptr MolaViz::create_and_add_window(
    const window_name_t& name)
{
    MRPT_LOG_DEBUG_FMT("Creating new window `%s`", name.c_str());

    mrpt::gui::CDisplayWindowGUI_Params cp;
    cp.maximized   = true;
    windows_[name] = mrpt::gui::CDisplayWindowGUI::Create(name, 1000, 800, cp);

    // create empty list of subwindows too:
    subWindows_[name];

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

    mrpt::system::thread_name("MolaViz::gui_thread");

    nanogui::init();

    // Open first GUI window:
    auto w = create_and_add_window(DEFAULT_WINDOW_NAME);

    // Tasks pending to be run before each refresh:
    w->setLoopCallback([this]() {
        ProfilerEntry pe(profiler_, "loopCallback lambda");

        // Get a copy of the tasks:
        task_queue_t tasks;
        auto         lck       = mrpt::lockHelper(guiThreadPendingTasksMtx_);
        tasks                  = std::move(guiThreadPendingTasks_);
        guiThreadPendingTasks_ = task_queue_t();
        auto winsToReLayout    = guiThreadMustReLayoutTheseWindows_;
        guiThreadMustReLayoutTheseWindows_.clear();
        lck.unlock();

        // Run them:
        auto& hc          = HandlersContainer::Instance();
        auto  lckHandlers = mrpt::lockHelper(hc.guiHandlersMtx_);
        for (auto& t : tasks)
        {
            try
            {
                t();
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM(
                    "Exception in task sent to GUI thread:\n"
                    << e.what());
            }
        }
        lckHandlers.unlock();

        for (const auto& winName : winsToReLayout)
            windows_.at(winName)->performLayout();
    });

    // A call to "nanogui::leave()" is required to end the infinite loop
    // in mainloop:
    nanogui::mainloop(25 /*refresh milliseconds*/);

    // Tidy up:
    nanogui::shutdown();

    // Delete all OpenGL memory from this same thread:
    windows_.clear();
    subWindows_.clear();

    MRPT_LOG_DEBUG("gui_thread() quitted.");
}

std::future<bool> MolaViz::subwindow_update_visualization(
    const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
    const std::string& parentWindow)
{
    using return_type = bool;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        [this, obj, subWindowTitle, parentWindow]() {
            try
            {
                const char* objClassName = obj->GetRuntimeClass()->className;

                MRPT_LOG_DEBUG_STREAM(
                    "subwindow_update_visualization() title='"
                    << subWindowTitle << "' obj of class: '" << objClassName
                    << "'");

                // Get subwindow:
                ASSERTMSG_(
                    subWindows_.count(parentWindow),
                    mrpt::format(
                        "parentWindow not found: '%s'", parentWindow.c_str()));
                auto topWin = subWindows_.at(parentWindow);

                ASSERTMSG_(
                    topWin.count(subWindowTitle),
                    mrpt::format(
                        "subWindow not found: '%s'", subWindowTitle.c_str()));

                auto subWin = topWin.at(subWindowTitle);
                ASSERT_(subWin != nullptr);

                // Get object GUI handler:
                // (Note: guiHandlersMtx_ is already locked by main render
                // thread calling me)
                auto& hc = HandlersContainer::Instance();

                if (auto itHandler = hc.guiHandlers_.find(objClassName);
                    itHandler != hc.guiHandlers_.end())
                {
                    // Update GUI with object:
                    itHandler->second(obj, subWin, parentWindow, this);
                    return true;
                }
                else
                {
                    // No handler for this class:
                    MRPT_LOG_DEBUG_STREAM(
                        "subwindow_update_visualization() No known handler for "
                        "obj of "
                        "class: '"
                        << objClassName << "'");

                    return false;
                }
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM(
                    "subwindow_update_visualization(): exception:\n"
                    << e.what());
                return false;
            }
        });

    auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
    guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
    return task->get_future();
}

std::future<nanogui::Window*> MolaViz::create_subwindow(
    const std::string& subWindowTitle, const std::string& parentWindow)
{
    using return_type = nanogui::Window*;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        [this, subWindowTitle, parentWindow]() {
            MRPT_LOG_DEBUG_STREAM(
                "create_subwindow() title='"
                << subWindowTitle << "' inside toplevel '" << parentWindow
                << "'");

            ASSERT_(windows_.count(parentWindow));
            auto topWin = windows_.at(parentWindow);
            ASSERT_(topWin);

            auto subw = topWin->createManagedSubWindow(subWindowTitle);
            subw->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Vertical, 1, nanogui::Alignment::Fill, 2,
                2));

            // add to list of subwindows too:
            subWindows_[parentWindow][subWindowTitle] = subw;

            // Reduce size button:
            subw->buttonPanel()
                ->add<nanogui::Button>("", ENTYPO_ICON_RESIZE_100_PERCENT)
                ->setCallback([subw, topWin]() {
                    if (auto glControl =
                            dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(
                                subw->children().at(1));
                        glControl)
                    {
                        auto s = glControl->size();
                        s.x() *= 0.75;
                        s.y() *= 0.75;
                        glControl->setSize(s);
                        glControl->setFixedSize(s);
                    }
                    topWin->performLayout();
                });

            // Enlarge button:
            subw->buttonPanel()
                ->add<nanogui::Button>("", ENTYPO_ICON_RESIZE_FULL_SCREEN)
                ->setCallback([subw, topWin]() {
                    if (auto glControl =
                            dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(
                                subw->children().at(1));
                        glControl)
                    {
                        auto s = glControl->size();
                        s.x() *= 1.25;
                        s.y() *= 1.25;
                        glControl->setSize(s);
                        glControl->setFixedSize(s);
                    }

                    topWin->performLayout();
                });

            return subw;
        });

    auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
    guiThreadPendingTasks_.emplace_back([=]() { (*task)(); });
    guiThreadMustReLayoutTheseWindows_.insert(parentWindow);
    return task->get_future();
}

void gui_handler_show_common_sensor_info(
    const mrpt::obs::CObservation& obs, nanogui::Window* w)
{
    auto glControl =
        dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
    if (!glControl) return;
    if (!glControl->scene) return;

    auto glView = glControl->scene->getViewport();
    if (!glView) return;

    constexpr unsigned int TXT_ID_TIMESTAMP = 0;

    mrpt::opengl::TFontParams fp;
    fp.color        = {1.0f, 1.0f, 1.0f};
    fp.draw_shadow  = true;
    fp.shadow_color = {0.0f, 0.0f, 0.0f};
    fp.vfont_scale  = 8;

    glView->addTextMessage(
        2, 2,
        mrpt::format(
            "Timestamp: %s",
            mrpt::system::dateTimeToString(obs.timestamp).c_str()),
        TXT_ID_TIMESTAMP, fp);
}

// CObservationImage
void gui_handler_images(
    const mrpt::rtti::CObject::Ptr& o, nanogui::Window* w,
    MolaViz::window_name_t parentWin, MolaViz* instance)
{
    auto obj = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o);
    if (!obj) return;

    MRPT_TODO("Add more info abot the image: resolution, refresh rate, etc.");

    obj->load();

    mrpt::gui::MRPT2NanoguiGLCanvas* glControl;
    if (w->children().size() == 1)
    {
        // Guess window size:
        int winW = obj->image.getWidth(), winH = obj->image.getHeight();

        // Guess if we need to decimate subwindow size:
        while (winW > 512 || winH > 512)
        {
            winW /= 2;
            winH /= 2;
        }

        glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
        glControl->setSize({winW, winH});
        glControl->setFixedSize({winW, winH});

        auto lck = mrpt::lockHelper(glControl->scene_mtx);

        glControl->scene = mrpt::opengl::COpenGLScene::Create();
        instance->markWindowForReLayout(parentWin);
    }
    else
    {
        glControl =
            dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
    }
    ASSERT_(glControl != nullptr);

#if MRPT_VERSION <= 0x232
    // This overcomes a bug in MRPT 2.3.1, fixed in 2.3.2:
    obj->image.loadFromFile(obj->image.getExternalStorageFileAbsolutePath());
#endif

    auto lck = mrpt::lockHelper(glControl->scene_mtx);
    glControl->scene->getViewport()->setImageView(obj->image);

    gui_handler_show_common_sensor_info(*obj, w);
}

// CObservationPointCloud
// CObservation2DRangeScan
// CObservation3DRangeScan
void gui_handler_point_cloud(
    const mrpt::rtti::CObject::Ptr& o, nanogui::Window* w,
    MolaViz::window_name_t parentWin, MolaViz* instance)
{
    using namespace mrpt::obs;

    MRPT_TODO("Show more info abot the PC");

    mrpt::gui::MRPT2NanoguiGLCanvas*       glControl;
    mrpt::opengl::CPointCloudColoured::Ptr glPc;
    if (w->children().size() == 1)
    {
        // Reuse from past iterations:
        glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();

        auto lck = mrpt::lockHelper(glControl->scene_mtx);

        glControl->scene = mrpt::opengl::COpenGLScene::Create();

        glPc = mrpt::opengl::CPointCloudColoured::Create();
        glControl->scene->insert(glPc);

        glPc->setPointSize(3.0);
        instance->markWindowForReLayout(parentWin);
    }
    else
    {
        // Create on first use:
        glControl =
            dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
        auto lck = mrpt::lockHelper(glControl->scene_mtx);

        glPc =
            glControl->scene->getByClass<mrpt::opengl::CPointCloudColoured>();
    }
    ASSERT_(glControl != nullptr);
    ASSERT_(glPc);

    auto lck = mrpt::lockHelper(glControl->scene_mtx);

    if (auto objPc = std::dynamic_pointer_cast<CObservationPointCloud>(o);
        objPc)
    {
        if (!objPc->pointcloud) return;
        objPc->load();
        glPc->loadFromPointsMap(objPc->pointcloud.get());

        gui_handler_show_common_sensor_info(*objPc, w);
    }
    else if (auto obj3D = std::dynamic_pointer_cast<CObservation3DRangeScan>(o);
             obj3D)
    {
        obj3D->load();

        mrpt::obs::T3DPointsProjectionParams pp;
        pp.takeIntoAccountSensorPoseOnRobot = true;

        obj3D->unprojectInto(*glPc, pp);
        gui_handler_show_common_sensor_info(*obj3D, w);
    }
    else if (auto obj2D = std::dynamic_pointer_cast<CObservation2DRangeScan>(o);
             obj2D)
    {
        mrpt::maps::CSimplePointsMap auxMap;
        auxMap.insertObservationPtr(obj2D);
        glPc->loadFromPointsMap(&auxMap);

        gui_handler_show_common_sensor_info(*obj2D, w);
    }
    else
        return;

    // viz options:
    const auto bb = glPc->getBoundingBox();
    glPc->recolorizeByCoordinate(bb.min.z, bb.max.z);
}
