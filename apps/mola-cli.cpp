/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-cli.cpp
 * @brief  main() for the mola-cli app, the CLI for mola-launcher
 * @author Jose Luis Blanco Claraco
 * @date   Nov 28, 2018
 */

#include <mola-launcher/MolaLauncherApp.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/rtti/CObject.h>
#include <csignal>  // sigaction
#include <cstdlib>
#include <iostream>
#include <string>

#if defined(WIN32)
#include <windows.h>  // SetConsoleCtrlHandler
#endif
MRPT_TODO("win32: add SetConsoleCtrlHandler");

// Declare supported cli switches ===========
static TCLAP::CmdLine               cmd("mola-cli");
static TCLAP::ValueArg<std::string> arg_yaml_cfg(
    "c", "config", "Input YAML config file (required) (*.yml)", false, "",
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

static TCLAP::SwitchArg arg_rtti_list_all(
    "", "rtti-list-all",
    "Loads all MOLA modules, then list all classes registered via mrpt::rtti, "
    "and exits.",
    cmd);

static TCLAP::ValueArg<std::string> arg_rtti_list_children(
    "", "rtti-children-of",
    "Loads all MOLA modules, then list all known classes that inherit from the "
    "given one, and exits.",
    false, "", "mp2p_icp::ICP_Base", cmd);

static TCLAP::SwitchArg arg_list_modules(
    "", "list-modules",
    "Loads all MOLA modules, then list them. It also shows the list of paths "
    "in which the program looks for module dynamic libraries, then exits.",
    cmd);

static TCLAP::SwitchArg arg_list_module_shared_dirs(
    "", "list-module-shared-dirs",
    "Finds all MOLA module source/shared directories, then list them. Paths "
    "can be added with the environment variable MOLA_MODULES_SHARED_PATH.",
    cmd);

void mola_signal_handler(int s);
void mola_install_signal_handler();

static mola::MolaLauncherApp app;

// Default task for mola-cli: launching a SLAM system
// -----------------------------------------------------
static int mola_cli_launch_slam()
{
    // Load YAML config file:
    if (!arg_yaml_cfg.isSet())
    {
        TCLAP::ArgException e(
            "-c xxx.yaml (or --config xxx.yml) is required to launch a SLAM "
            "system.");
        cmd.getOutput()->failure(cmd, e);
        return 1;
    }
    const auto file_yml = arg_yaml_cfg.getValue();

    YAML::Node cfg = YAML::LoadFile(file_yml);

    app.profiler_.enable(
        arg_enable_profiler.isSet() || arg_enable_profiler_whole.isSet());
    app.profiler_.enableKeepWholeHistory(arg_enable_profiler_whole.isSet());

    // Create SLAM system:
    app.setup(cfg);

    // Run it:
    app.spin();

    return 0;
}

// list all RTTI classes:
// -----------------------------------------------------
int mola_cli_rtti_list_all()
{
    app.scanAndLoadLibraries();

    std::vector<const mrpt::rtti::TRuntimeClassId*> lst =
        mrpt::rtti::getAllRegisteredClasses();

    for (const auto& c : lst) std::cout << c->className << "\n";

    return 0;
}

// list children of a given class:
// -----------------------------------------------------
int mola_cli_rtti_list_child()
{
    app.scanAndLoadLibraries();

    const auto parentName = arg_rtti_list_children.getValue();

    std::cout << "Listing children of class: " << parentName << "\n";

    const mrpt::rtti::TRuntimeClassId* id_parent =
        mrpt::rtti::findRegisteredClass(parentName);

    if (id_parent == nullptr)
        throw std::runtime_error(mrpt::format(
            "Cannot find any registered class named `%s`.\nTry using "
            "`mola-cli --rtti-list-all`",
            parentName.c_str()));

    const auto lst = mrpt::rtti::getAllRegisteredClassesChildrenOf(id_parent);
    for (const auto& c : lst) std::cout << c->className << "\n";
    return 0;
}

int mola_cli_list_modules()
{
    // show paths:
    std::vector<std::string> lst = app.getModuleLibPaths();
    std::cout << "MOLA_MODULES_LIB_PATH has " << lst.size()
              << " directories:\n";
    for (const auto& p : lst) std::cout << p << "\n";
    std::cout << "\n";

    // Load and show loaded modules:
    app.scanAndLoadLibraries();

    const std::vector<std::string> mods = app.getLoadedModules();
    std::cout << "Loaded MOLA modules: " << mods.size() << "\n";
    for (const auto& p : mods) std::cout << p << "\n";
    std::cout << "\n";

    return 0;
}

int mola_cli_list_module_shared_dirs()
{
    const auto mod2path_lst = app.scanForModuleSharedDirectories();
    for (const auto& p : mod2path_lst)
        std::cout << p.first << ": " << p.second << "\n";

    return 0;
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        mola_install_signal_handler();

        // Define the verbosity level here so it affects all possible commands
        // of mola-cli:
        if (arg_verbosity_level.isSet())
        {
            using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
            const auto v = vl::name2value(arg_verbosity_level.getValue());
            app.setVerbosityLevel(v);
        }

        // Different tasks that can be dine with mola-cli:
        if (arg_rtti_list_all.isSet()) return mola_cli_rtti_list_all();
        if (arg_rtti_list_children.isSet()) return mola_cli_rtti_list_child();
        if (arg_list_modules.isSet()) return mola_cli_list_modules();
        if (arg_list_module_shared_dirs.isSet())
            return mola_cli_list_module_shared_dirs();

        // Default task:
        return mola_cli_launch_slam();

        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "[mola-cli] Exit due to exception:\n"
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
