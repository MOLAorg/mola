/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MolaLauncherApp.h
 * @brief  Main launcher for MOLA systems
 * @author Jose Luis Blanco Claraco
 * @date   Nov 28, 2018
 */
#pragma once

#include <mola-kernel/interfaces/ExecutableBase.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <condition_variable>
#include <map>
#include <thread>
#include <vector>

namespace mola
{
/** Main launcher for a MOLA system. See mola-cli for a ready-to-use program.
 * \ingroup mola_launcher_grp */
class MolaLauncherApp : public mrpt::system::COutputLogger
{
   public:
    MolaLauncherApp();
    virtual ~MolaLauncherApp();

    /** @name SLAM system setup
     * @{ */

    /** Adds a directory to the list of paths to search for MOLA modules
     * (.so/.dll) when setup() is called
     * (Default=CMAKE_LIBRARY_OUTPUT_DIRECTORY).
     */
    void addModulesDirectory(const std::string& path);

    /** Returns a copy of the current list of paths in which this object will
     * try to look for MOLA modules.
     * (Default=CMAKE_LIBRARY_OUTPUT_DIRECTORY).
     * \sa addModulesDirectory
     */
    std::vector<std::string> getModulesPaths() const
    {
        return lib_search_paths_;
    }

    /** Returns the current list of loaded module dynamic libraries. */
    std::vector<std::string> getLoadedModules();

    /** Prepares the SLAM system based on a YAML configuration file.
     * See [mola]/demos/ for example YAML files.
     * At this point, MOLA module libraries are searched in a list of paths
     * and loaded for their classes to be available in name-based class
     * factories. Modules must be named "libmola*" to be loaded.
     * \sa addModulesDirectory, scanAndLoadLibraries
     */
    void setup(const YAML::Node& cfg);

    /** (Blocking call) Launch sensor and worker threads and enters into an
     * infinite loop executing the SLAM system, until shutdown() is called
     * (e.g. from another thread or a signal handler). */
    void spin();

    /** Attempts to do a clean shutdown of the system, giving all threads an
     * opportunity to end and save any pending data, etc. \sa spin() */
    void shutdown();

    /** @} */

    /** @name SLAM system control & monitoring
     * @{ */

    // TODO: set SLAM / localization mode?

    /** @} */

    /** Scans and loads MOLA module libraries. This is automatically called
     * within setup(), but it's provided here in case a user want to only load
     * modules for use the RTTI machinery on them without setting up a complete
     * SLAM system.
     * \sa addModulesDirectory, setup
     */
    void scanAndLoadLibraries();

    /** Time profiler. It's enabled/disabled status will be inherited (by
     * default, unless set otherwise in their YAML config files) by all modules
     * created upon the call to setup() */
    mrpt::system::CTimeLogger profiler_{"MolaLauncherApp"};

    /** Enabled from mola-cli with `--profiler-whole` to save full profile stats
     * to .m files at program end.
     */
    std::optional<ProfilerSaverAtDtor> profiler_dtor_save_stats_;

    struct Parameters
    {
        bool enforce_initialize_one_at_a_time{false};
    };

    Parameters launcher_params_;

   private:
    struct InfoPerRunningThread
    {
        std::string         yaml_cfg_block;
        ExecutableBase::Ptr impl;
        std::thread         executor;
        std::string         name;
        double              execution_rate{10.0};  //!< (Hz)
        int                 launch_priority{0};
        bool                initialization_done{false};
    };
    /** Indexed by `name` */
    std::map<std::string, InfoPerRunningThread> running_threads_;

    /** Used to enforce a one-by-one module initialization */
    std::condition_variable thread_launch_condition_;
    /** Used together with thread_launch_condition_ */
    std::mutex thread_launch_init_mtx_;

    /** Number of threads which are not done yet with initilize()
     * This is incremented before launching each thread from the main thread,
     * then decreased by each thread after initialization is done. The main
     * spinOnce() loop is not executed until this global variable is zero, i.e.
     * all modules are correctly initialized.
     */
    std::atomic<int> pending_initializations_{0};

    void executor_thread(InfoPerRunningThread& rds);  //!< Thread func.

    /** Set to true to command all running threads to exit */
    std::atomic_bool threads_must_end_{false};

    /** Used in setup(), can be added to via addModulesDirectory() */
    std::vector<std::string> lib_search_paths_{};

    /** Implementation for nameServer in the ExecutableBase interface */
    ExecutableBase::Ptr nameServerImpl(const std::string& name);

    /** Household tasks to be done while in the infinite main loop. */
    void internal_spin_tasks();
};

}  // namespace mola
