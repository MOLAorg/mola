/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MolaViz.h
 * @brief  Main C++ class for MOLA GUI
 * @author Jose Luis Blanco Claraco
 * @date   May  11, 2019
 */
#pragma once

#include <mola-kernel/interfaces/ExecutableBase.h>
#include <mola-kernel/interfaces/VizInterface.h>
#include <mrpt/gui/CDisplayWindowGUI.h>

#include <future>
#include <memory>
#include <shared_mutex>
#include <vector>

namespace mola
{
/** MOLA GUI and visualization API
 *
 */
class MolaViz : public ExecutableBase, public VizInterface
{
    DEFINE_MRPT_OBJECT(MolaViz, mola)

   public:
    MolaViz();
    ~MolaViz() override;

    // See docs in base class
    void initialize_common(const std::string&) override {}
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

    /** @name mola-viz main API
     * @{ */
    using window_name_t    = std::string;
    using subwindow_name_t = std::string;

    const static window_name_t DEFAULT_WINDOW_NAME;

    static bool     IsRunning();
    static MolaViz* Instance();

    /** Returned object is owned by the VizInterface, do NOT delete it. */
    std::future<nanogui::Window*> create_subwindow(
        const std::string& subWindowTitle,
        const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

    /** Updates the contents of a subwindow from a given object, typically a
     * mrpt::obs::CObservation, but custom handlers can be installed for
     * arbitrary classes.
     *
     * Depending on the object class RTTI, the corresponding handler is called.
     *
     * \return false if no handler is found for the given object.
     *
     * \sa
     */
    std::future<bool> subwindow_update_visualization(
        const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
        const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

    /** @} */

    /** @name mola-viz GUI update handlers registry
     * @{ */

    using update_handler_t = std::function<void(
        const mrpt::rtti::CObject::Ptr&, nanogui::Window* subWin,
        window_name_t parentWin, MolaViz* instance)>;
    using class_name_t     = std::string;

    static void register_gui_handler(
        class_name_t className, update_handler_t handler);

    /** @} */

    void markWindowForReLayout(const window_name_t& name)
    {
        guiThreadMustReLayoutTheseWindows_.insert(name);
    }

   private:
    static MolaViz*          instance_;
    static std::shared_mutex instanceMtx_;

    mrpt::gui::CDisplayWindowGUI::Ptr create_and_add_window(
        const window_name_t& name);

    std::map<window_name_t, mrpt::gui::CDisplayWindowGUI::Ptr> windows_;
    std::map<window_name_t, std::map<subwindow_name_t, nanogui::Window*>>
        subWindows_;

    std::thread guiThread_;
    void        gui_thread();

    using task_queue_t = std::vector<std::function<void()>>;
    task_queue_t            guiThreadPendingTasks_;
    std::set<window_name_t> guiThreadMustReLayoutTheseWindows_;
    std::mutex              guiThreadPendingTasksMtx_;
};

}  // namespace mola
