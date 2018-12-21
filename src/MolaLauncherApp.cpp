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

#include <mola-kernel/yaml_helpers.h>
#include <mola-launcher/MolaLauncherApp.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/CRateTimer.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include "MolaDLL_Loader.h"

using namespace mola;

MolaLauncherApp::MolaLauncherApp()
    : mrpt::system::COutputLogger("MolaLauncherApp")
{
    lib_search_paths_.emplace_back(MOLA_MODULES_DIR);
}

MolaLauncherApp::~MolaLauncherApp() { this->shutdown(); }

void MolaLauncherApp::shutdown()
{
    // End all threads:
    threads_must_end_ = true;
    if (!running_threads_.empty())
    {
        MRPT_LOG_INFO_STREAM(
            "Shutting down " << running_threads_.size()
                             << " worker threads...");
        for (auto& ds : running_threads_)
            if (ds.second.executor.joinable()) ds.second.executor.join();
        MRPT_LOG_INFO("Done.");
        running_threads_.clear();
    }

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
}

void MolaLauncherApp::addModulesDirectory(const std::string& path)
{
    lib_search_paths_.push_back(path);
}

void MolaLauncherApp::setup(const YAML::Node& cfg_in)
{
    MRPT_TRY_START

    // make sure all available modules are loaded and classes are registered.
    internal_load_lib_modules(*this, lib_search_paths_);

    MRPT_LOG_INFO(
        "Setting up system from YAML config... (set DEBUG verbosity level to "
        "see full config)");

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

            const auto ds_label = ds["name"].as<std::string>();
            ASSERTMSG_(!ds_label.empty(), "`name` cannot be empty!");
            if (running_threads_.count(ds_label) != 0)
                THROW_EXCEPTION_FMT("Duplicated `name`: %s", ds_label.c_str());

            const auto ds_classname = ds["type"].as<std::string>();
            ASSERTMSG_(!ds_classname.empty(), "`type` cannot be empty!");

            InfoPerRunningThread& info = running_threads_[ds_label];
            {
                // Make a copy of the YAML config block:
                std::stringstream ss;
                ss << ds;
                info.yaml_cfg_block = ss.str();
            }
            info.name = ds_label;
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
            info.execution_rate = ds["execution_rate"].as<double>(1.0);

            info.impl->profiler_.setName(logName);
            info.impl->profiler_.enable(profiler_.isEnabled());

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
    for (auto& ds : running_threads_)
    {
        ds.second.executor = std::thread(
            &MolaLauncherApp::executor_thread, this, std::ref(ds.second));
    }

    // Main SLAM/Localization infinite loop
    // -------------------------------------------
    MRPT_LOG_INFO(
        "Entering main SLAM/localization loop..."
        "(CTRL+C from mola-cli to stop)");
    while (!threads_must_end_)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(200ms);
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
        rds.impl->initialize_common(rds.yaml_cfg_block);
        rds.impl->initialize(rds.yaml_cfg_block);

        mrpt::system::CRateTimer timer(rds.execution_rate);

        while (!threads_must_end_)
        {
            rds.impl->spinOnce();

            // Done, cycle:
            const bool ontime = timer.sleep();
            if (!ontime)
                MRPT_LOG_THROTTLE_WARN_STREAM(
                    1.0,
                    "Could not achieve desired real-time execution rate ("
                        << rds.execution_rate
                        << " Hz) on thread for sensor named: " << rds.name);
        };
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM(
            "Error: Will shutdown since thread for module named `"
            << rds.name << "` ended due to an exception:\n"
            << mrpt::exception_to_str(e));
        threads_must_end_ = true;
    }
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
