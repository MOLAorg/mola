/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
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
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <shared_mutex>

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
    using window_name_t = std::string;

    const static window_name_t DEFAULT_WINDOW_NAME;

    static bool     IsRunning();
    static MolaViz* Instance();

    void create_subwindow(
        const std::string& title,
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
    bool subwindow_update_visualization(
        const std::string& subWindowTitle, const mrpt::rtti::CObject::Ptr& obj);

    /** @} */

   private:
    // mrpt::WorkerThreadsPool worker_pool_{1};
    static MolaViz*          instance_;
    static std::shared_mutex instanceMtx_;

    mrpt::gui::CDisplayWindowGUI::Ptr create_and_add_window(
        const window_name_t& name);

    std::map<window_name_t, mrpt::gui::CDisplayWindowGUI::Ptr> windows_;

    std::thread guiThread_;
    void        gui_thread();

    using task_queue_t = std::vector<std::function<void(void)>>;
    task_queue_t guiThreadPendingTasks_;
    std::mutex   guiThreadPendingTasksMtx_;
};

}  // namespace mola
