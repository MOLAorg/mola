/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   MolaViz.h
 * @brief  Main C++ class for MOLA GUI
 * @author Jose Luis Blanco Claraco
 * @date   May  11, 2019
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/VizInterface.h>
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
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;

    /** @name mola-viz main API
     * @{ */
    using window_name_t    = std::string;
    using subwindow_name_t = std::string;

    const static window_name_t DEFAULT_WINDOW_NAME;

    static bool     IsRunning();
    static MolaViz* Instance();

    /** Returned object is owned by the VizInterface, do NOT delete it. Updates
     * to it must be done via enqueue_custom_nanogui_code()*/
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

    /** Update (or adds if not found) a 3D object in the main 3D view area.
     */
    std::future<bool> update_3d_object(
        const std::string&                                  objName,
        const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
        const std::string& viewportName = "main",
        const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

    std::future<bool> update_viewport_look_at(
        const mrpt::math::TPoint3Df& lookAt,
        const std::string&           viewportName = "main",
        const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

    std::future<bool> output_console_message(
        const std::string& msg,
        const std::string& parentWindow = "main") override;

    /// Updates to nanogui window controls must happen via this method to ensure
    /// it is run by the correct thread, in the next available time slot.
    std::future<void> enqueue_custom_nanogui_code(
        const std::function<void(void)>& userCode) override;

    /** @} */

    /** @name mola-viz GUI update handlers registry
     * @{ */

    using update_handler_t = std::function<void(
        const mrpt::rtti::CObject::Ptr&, nanogui::Window* subWin,
        window_name_t parentWin, MolaViz* instance)>;
    using class_name_t     = std::string;

    static void register_gui_handler(
        class_name_t name, update_handler_t handler);

    /** @} */

    /** @name mola-viz module parameters
     * @{ */

    double       console_text_font_size_   = 9.0;
    unsigned int max_console_lines_        = 5;
    bool         show_rgbd_as_point_cloud_ = false;  // too CPU demanding!

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

    struct PerWindowData
    {
        mrpt::gui::CDisplayWindowGUI::Ptr win;
        std::vector<std::string>          console_messages = {};
    };

    std::map<window_name_t, PerWindowData> windows_;
    std::map<window_name_t, std::map<subwindow_name_t, nanogui::Window*>>
        subWindows_;

    std::thread guiThread_;
    void        gui_thread();

    using task_queue_t = std::vector<std::function<void()>>;
    task_queue_t            guiThreadPendingTasks_;
    std::set<window_name_t> guiThreadMustReLayoutTheseWindows_;
    std::mutex              guiThreadPendingTasksMtx_;

    double lastTimeCheckForNewModules_ = 0;
    double lastTimeUpdateDatasetUIs_   = 0;
    struct DataPerDatasetUI
    {
        std::shared_ptr<mola::Dataset_UI> module;

        nanogui::Window*   ui                 = nullptr;
        nanogui::Label*    lbPlaybackPosition = nullptr;
        nanogui::Slider*   slider             = nullptr;
        nanogui::CheckBox* cbPaused           = nullptr;
        nanogui::ComboBox* cmRate             = nullptr;
    };
    std::map<std::string, DataPerDatasetUI> datasetUIs_;

    void dataset_ui_check_new_modules();
    void dataset_ui_update();
};

}  // namespace mola
