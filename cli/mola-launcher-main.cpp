/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-launcher-main.cpp
 * @brief  main() for mola-launcher app
 * @author Jose Luis Blanco Claraco
 * @date   Nov 28, 2018
 */

#include <mola-launcher/MolaLauncherApp.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <csignal>  // sigaction
#include <cstdlib>
#include <iostream>
#include <string>

#if defined(WIN32)
#include <windows.h>  // SetConsoleCtrlHandler
#endif
MRPT_TODO("win32: add SetConsoleCtrlHandler");

// Declare supported cli switches ===========
static TCLAP::CmdLine               cmd("mola-launcher");
static TCLAP::ValueArg<std::string> arg_yaml_cfg(
    "c", "config", "Input YAML config file (required) (*.yml)", true, "",
    "demo.yml", cmd);

static TCLAP::ValueArg<std::string> arg_verbosity_level(
    "v", "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
    false, "", "INFO", cmd);

static TCLAP::SwitchArg arg_enable_profiler(
    "p", "profiler",
    "Enable time profiler by default in all modules (Default: NO)", cmd);

static TCLAP::SwitchArg arg_enable_profiler_whole(
    "", "profiler-whole",
    "Enable whole-history time profiler in all modules (Default: NO). **DO "
    "NOT** use in production, only to benchmark short runs (unbounded memory "
    "usage)",
    cmd);

void mola_signal_handler(int s);
void mola_install_signal_handler();

static mola::MolaLauncherApp app;

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        mola_install_signal_handler();

        // Load YAML config file:
        const auto file_yml = arg_yaml_cfg.getValue();

        YAML::Node cfg = YAML::LoadFile(file_yml);

        if (arg_verbosity_level.isSet())
        {
            using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
            const auto v = vl::name2value(arg_verbosity_level.getValue());
            app.setVerbosityLevel(v);
        }
        app.profiler_.enable(
            arg_enable_profiler.isSet() || arg_enable_profiler_whole.isSet());
        app.profiler_.enableKeepWholeHistory(arg_enable_profiler_whole.isSet());

        // Create SLAM system:
        app.setup(cfg);

        // Run it:
        app.spin();

        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "[mola-launcher] Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}

void mola_signal_handler(int s)
{
    std::cerr << "Caught signal " << s << ". Shutting down..." << std::endl;
    app.shutdown();
    exit(0);
}

void mola_install_signal_handler()
{
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = &mola_signal_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, nullptr);
}
