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

/** \defgroup mola_launcher_grp mola-launcher: Library to load and launch MOLA
 * systems
 */

#include <mola-kernel/interfaces/FrontEndBase.h>
#include <mola-kernel/interfaces/RawDataSourceBase.h>
#include <mola-kernel/yaml_helpers.h>
#include <mola-launcher/MolaLauncherApp.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/memory.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "MolaDLL_Loader.h"

using namespace mola;

MolaLauncherApp::MolaLauncherApp()
    : mrpt::system::COutputLogger("MolaLauncherApp")
{
    lib_search_paths_.emplace_back(MOLA_MODULES_DIR);
}

MolaLauncherApp::~MolaLauncherApp()
{
    this->shutdown();
    if (!running_threads_.empty())
    {
        MRPT_LOG_INFO_STREAM(
            "Shutting down " << running_threads_.size()
                             << " module threads...");
        for (auto& ds : running_threads_)
            if (ds.second.executor.joinable()) ds.second.executor.join();
        MRPT_LOG_INFO("Done.");
        running_threads_.clear();
    }

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
}

void MolaLauncherApp::shutdown()
{
    using namespace std::chrono_literals;

    // Ordered shut down:

    // Stop data sources first
    stopAllThreadsOfType<RawDataSourceBase>();
    std::this_thread::sleep_for(200ms);

    // Front ends next:
    stopAllThreadsOfType<FrontEndBase>();
    std::this_thread::sleep_for(300ms);

    // End all threads:
    threads_must_end_ = true;
}

void MolaLauncherApp::addModulesDirectory(const std::string& path)
{
    lib_search_paths_.push_back(path);
}

std::vector<std::string> MolaLauncherApp::getLoadedModules()
{
    std::vector<std::string> mods;

    const std::map<std::string, LoadedModules>& lst = get_loaded_modules();
    for (const auto& m : lst) mods.push_back(m.second.lib_path);
    return mods;
}

void MolaLauncherApp::scanAndLoadLibraries()
{
    MRPT_TRY_START

    // make sure all available modules are loaded and classes are registered.
    internal_load_lib_modules(*this, lib_search_paths_);

    MRPT_TRY_END
}

void MolaLauncherApp::setup(const YAML::Node& cfg_in)
{
    MRPT_TRY_START

    scanAndLoadLibraries();

    MRPT_LOG_INFO(
        "Setting up system from YAML config... (set DEBUG verbosity level to "
        "see full config)");

    // Enable save to stat files at dtor:
    if (profiler_.isEnabledKeepWholeHistory())
        profiler_dtor_save_stats_.emplace(profiler_);

    // Parse YAML env variables:
    YAML::Node cfg;
    {
        std::stringstream ss;
        ss << cfg_in;
        cfg = YAML::Load(mola::parseEnvVars(ss.str()));
    }

    MRPT_LOG_DEBUG_STREAM(
        "Using the following configuration:\n"
        "==================================================\n"
        << cfg << std::endl
        << "==================================================\n");

    // clang-format off
    const std::vector<
        std::pair<std::string, std::function<ExecutableBase::Ptr(const std::string &)>>>
        lstSections = {
            /* modules */
            {
              "modules", [](const std::string &type)
              {
               return mola::ExecutableBase::Factory(type);
              }
            }
    };
    // clang-format on

    // for each YAML file section type:
    //
    // Refer to docs:
    // https://docs.mola-slam.org/latest/concept-slam-configuration-file.html
    //
    for (const auto& section : lstSections)
    {
        const auto& sectName      = section.first;
        const auto& sectGenerator = section.second;

        ASSERTMSG_(
            cfg[sectName], "Missing YAML required entry: `" + sectName + "`");
        const auto& cfg_blk = cfg[sectName];

        // Create each module instance in this section:
        for (const auto& ds : cfg_blk)
        {
            ENSURE_YAML_ENTRY_EXISTS(ds, "type");
            ENSURE_YAML_ENTRY_EXISTS(ds, "name");
            ENSURE_YAML_ENTRY_EXISTS(ds, "params");

            // Allow quickly disabling sections:
            if (ds["launch_ignore"] && ds["launch_ignore"].as<bool>()) continue;

            const auto ds_label = ds["name"].as<std::string>();
            ASSERTMSG_(!ds_label.empty(), "`name` cannot be empty!");
            if (running_threads_.count(ds_label) != 0)
                THROW_EXCEPTION_FMT("Duplicated `name`: %s", ds_label.c_str());

            const auto ds_classname = ds["type"].as<std::string>();
            ASSERTMSG_(!ds_classname.empty(), "`type` cannot be empty!");

            InfoPerRunningThread& info = running_threads_[ds_label];
            // Make a copy of the YAML config block:
            info.yaml_cfg_block = mola::yaml2string(ds);
            info.name           = ds_label;
            MRPT_LOG_INFO_STREAM(
                "Instantiating module `" << ds_label << "` of type `"
                                         << ds_classname << "`");
            // Create object (needs to be registered):
            info.impl = sectGenerator(ds_classname);
            ASSERTMSG_(
                info.impl,
                mrpt::format(
                    "Error: object for module `%s` of type `%s` couldn't be "
                    "constructed from the Factory. Check class type spelling, "
                    "and whether the corresponding MOLA module is loaded.",
                    ds_label.c_str(), ds_classname.c_str()));

            // Inherit verbosity level:
            auto verb_level = this->getMinLoggingLevel();
            // override it if present:
            const auto verbLvl = ds["verbosity_level"].as<std::string>("");
            if (!verbLvl.empty())
            {
                verb_level = mrpt::typemeta::TEnumType<
                    mrpt::system::VerbosityLevel>::name2value(verbLvl);
            }
            info.impl->setMinLoggingLevel(verb_level);

            // Default logger name, can be changed in initilize() if desired
            const auto logName = ds_classname + std::string(":") + ds_label;
            info.impl->setLoggerName(logName);
            info.execution_rate  = ds["execution_rate"].as<double>(1.0);
            info.launch_priority = info.impl->launchOrderPriority();

            info.impl->setModuleInstanceName(logName);

            info.impl->profiler_.setName(logName);
            info.impl->profiler_.enable(profiler_.isEnabled());
            if (profiler_.isEnabledKeepWholeHistory())
            {
                info.impl->profiler_.enableKeepWholeHistory(true);
                // Enable save to CSV at dtor:
                info.impl->profiler_dtor_save_stats_.emplace(
                    info.impl->profiler_);
            }

            info.impl->nameServer_ = std::bind(
                &MolaLauncherApp::nameServerImpl, this, std::placeholders::_1);
        }
    }

    MRPT_LOG_DEBUG_STREAM("All modules have been created");

    MRPT_TRY_END
}

void MolaLauncherApp::spin()
{
    MRPT_TRY_START

    // Launch working threads:
    // ---------------------------------
    // Sort by launch priority:
    {
        std::multimap<int, std::string> lst;
        std::transform(
            running_threads_.begin(), running_threads_.end(),
            std::inserter(lst, lst.begin()), [](auto& ds) {
                return std::make_pair(
                    ds.second.launch_priority, ds.second.name);
            });

        for (auto& name : lst)
        {
            // atomic counter. See docs in the variable declaration (in the .h)
            pending_initializations_++;

            auto& ds    = running_threads_[name.second];
            ds.executor = std::thread(
                &MolaLauncherApp::executor_thread, this, std::ref(ds));

            // Wait until the new thread is done with its initialization():
            if (launcher_params_.enforce_initialize_one_at_a_time)
            {
                std::unique_lock<std::mutex> lock(thread_launch_init_mtx_);
                thread_launch_condition_.wait_for(
                    lock, std::chrono::milliseconds(5), [&ds, this] {
                        return !threads_must_end_ && ds.initialization_done;
                    });
            }
        }
    }

    // Main SLAM/Localization infinite loop
    // -------------------------------------------
    MRPT_LOG_INFO(
        "Entering main SLAM/localization loop..."
        "(CTRL+C from mola-cli to stop)");
    while (!threads_must_end_)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(500ms);

        internal_spin_tasks();
    }
    MRPT_LOG_INFO("Main SLAM/localization loop ended.");

    MRPT_TRY_END
}

void MolaLauncherApp::executor_thread(InfoPerRunningThread& rds)
{
    try
    {
        if (threads_must_end_) return;

        // Initilize:
        MRPT_LOG_DEBUG_STREAM(
            "Thread started for module named `"
            << rds.name << "` (launch priority=" << rds.launch_priority << ")");

        // Ensure that modules initialize one by one, if so defined:
        std::unique_ptr<std::unique_lock<std::mutex>> lock;
        if (launcher_params_.enforce_initialize_one_at_a_time)
            lock = std::make_unique<std::unique_lock<std::mutex>>(
                thread_launch_init_mtx_);

        rds.impl->initialize_common(rds.yaml_cfg_block);
        rds.impl->initialize(rds.yaml_cfg_block);

        // Notify that we are done with initialization:
        rds.initialization_done = true;
        thread_launch_condition_.notify_one();
        pending_initializations_--;

        lock.reset();  // unlock at dtor, if created

        mrpt::system::CRateTimer timer(rds.execution_rate);

        while (!threads_must_end_ && !rds.this_thread_must_end)
        {
            // Only if all modules are correctly initialized:
            if (pending_initializations_ == 0)
            {
                // Run the main module loop code:
                rds.impl->spinOnce();
            }

            // Done, cycle:
            const bool ontime = timer.sleep();
            if (!ontime)
                MRPT_LOG_THROTTLE_WARN_STREAM(
                    30.0,
                    "Could not achieve desired real-time execution rate ("
                        << rds.execution_rate
                        << " Hz) on thread for sensor named: " << rds.name);
        };

        // Give the module an opportunity to do any extra household tasks before
        // the actual dtors are invoked:
        rds.impl->onQuit();
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM(
            "Error: Will shutdown since thread for module named `"
            << rds.name << "` ended due to an exception:\n"
            << mrpt::exception_to_str(e));
        threads_must_end_ = true;
    }
    using namespace std::chrono_literals;
    // Give time for all threads to end:
    std::this_thread::sleep_for(250ms);
}

ExecutableBase::Ptr MolaLauncherApp::nameServerImpl(const std::string& name)
{
    // Special syntax to sequentially access all existing modules:
    // If the requested name has the format: "[" + <i>, return the i-th
    // module, or nullptr if out of range.
    // This is used by ExecutableBase::findService()
    if (name.size() >= 2 && name[0] == '[')
    {
        const auto idx = std::stoul(name.substr(1));
        if (idx >= running_threads_.size()) { return ExecutableBase::Ptr(); }
        else
        {
            auto it = running_threads_.begin();
            std::advance(it, idx);
            return it->second.impl;
        }
    }  // Regular name request:
    const auto it_th = running_threads_.find(name);
    if (it_th == running_threads_.end())
        return ExecutableBase::Ptr();
    else
        return it_th->second.impl;
}

void MolaLauncherApp::internal_spin_tasks()
{
    MRPT_START

    // Collect memory stats:
    {
        const auto mem_used = mrpt::system::getMemoryUsage();
        profiler_.registerUserMeasure("memory_used", mem_used);
    }

    MRPT_END
}
