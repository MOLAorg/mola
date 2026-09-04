/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   mola-cli.cpp
 * @brief  main() for the mola-cli app, the CLI for mola-launcher
 * @author Jose Luis Blanco Claraco
 * @date   Nov 28, 2018
 */

#include <mola_kernel/pretty_print_exception.h>
#include <mola_launcher/MolaLauncherApp.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/filesystem.h>

#include <CLI/CLI.hpp>
#include <csignal>  // sigaction
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#if defined(WIN32)
#include <windows.h>  // SetConsoleCtrlHandler
#endif
// TODO(jlbc): win32: add SetConsoleCtrlHandler

namespace
{
mola::MolaLauncherApp* theApp = nullptr;

void mola_signal_handler(int s)
{
  std::cerr << "Caught signal " << s << ". Shutting down..."
            << "\n";
  if (theApp) theApp->shutdown();
  // exit(0);
}

void mola_install_signal_handler()
{
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = &mola_signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, nullptr);
  // SIGTERM too: it is what process supervisors (systemd, containers, `timeout`,
  // plain `kill`) send to request a shutdown. Without this, rclcpp's own handler
  // would be the only one to run: it stops the ROS node from being spun, while
  // the main MOLA loop keeps running, so the process never exits and lingers
  // with its ROS 2 endpoints still advertised but no longer served.
  sigaction(SIGTERM, &sigIntHandler, nullptr);
}

// Default task for mola-cli: launching a SLAM system
// -----------------------------------------------------
int mola_cli_launch_slam(
    const std::vector<std::string>& yamlFiles, const std::string& verbosity, bool enableProfiler,
    bool enableProfilerWhole, const std::optional<std::vector<std::string>>& rosArgs)
{
  using namespace std::string_literals;

  if (yamlFiles.empty())
  {
    std::cerr << "Error: at least one YAML config file (*.yaml) is required "
                 "to launch a SLAM system.\n";
    return 1;
  }

  // replace a special variable for ROS args:
  mola::YAMLParseOptions po;

  if (rosArgs)
  {
    std::string strRosArgs = "[";
    for (const auto& s : *rosArgs) strRosArgs += " '"s + s + "', "s;
    strRosArgs += "]";
    po.variables["ROS_ARGS"] = strRosArgs;
  }

  mola::MolaLauncherApp app;
  theApp = &app;  // for the signal handler

  if (!verbosity.empty())
  {
    using vl     = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    const auto v = vl::name2value(verbosity);
    app.setVerbosityLevel(v);
  }

  app.profiler_.enable(enableProfiler || enableProfilerWhole);
  app.profiler_.enableKeepWholeHistory(enableProfilerWhole);

  // Load and set up each YAML config file. All `modules` sections are merged
  // together into the same running system.
  for (const auto& file_yml : yamlFiles)
  {
    auto cfg = mola::load_yaml_file(file_yml, po);
    app.setup(cfg, mrpt::system::extractFileDirectory(file_yml));
  }

  // Run it:
  app.spin();

  return 0;
}

// list all RTTI classes:
// -----------------------------------------------------
int mola_cli_rtti_list_all()
{
  mola::MolaLauncherApp app;
  theApp = &app;  // for the signal handler

  app.scanAndLoadLibraries();

  std::vector<const mrpt::rtti::TRuntimeClassId*> lst = mrpt::rtti::getAllRegisteredClasses();

  for (const auto& c : lst) std::cout << c->className << "\n";

  return 0;
}

// list children of a given class:
// -----------------------------------------------------
int mola_cli_rtti_list_child(const std::string& parentName)
{
  mola::MolaLauncherApp app;
  theApp = &app;  // for the signal handler

  app.scanAndLoadLibraries();

  std::cout << "Listing children of class: " << parentName << "\n";

  const mrpt::rtti::TRuntimeClassId* id_parent = mrpt::rtti::findRegisteredClass(parentName);

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
  mola::MolaLauncherApp app;
  theApp = &app;  // for the signal handler

  // show paths:
  std::vector<std::string> lst = app.getModuleLibPaths();
  std::cout << "MOLA_MODULES_LIB_PATH has " << lst.size() << " directories:\n";
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
  mola::MolaLauncherApp app;
  theApp = &app;  // for the signal handler

  const auto mod2path_lst = app.scanForModuleSharedDirectories();

  int longestName = 1;
  for (const auto& p : mod2path_lst) mrpt::keep_max<int>(longestName, p.first.size());

  for (const auto& p : mod2path_lst)
    std::cout << mrpt::format("%-*s : %s\n", longestName, p.first.c_str(), p.second.c_str());

  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli{"mola-cli"};

    std::vector<std::string> argYamlCfg;
    cli.add_option(
        "config", argYamlCfg,
        "Input YAML config file(s) (*.yaml). If more than one file is "
        "given, all of them are loaded and their `modules` sections are "
        "merged together into a single running system.");

    std::string argVerbosityLevel;
    cli.add_option(
        "-v,--verbosity", argVerbosityLevel,
        "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");

    bool argEnableProfiler = false;
    cli.add_flag(
        "-p,--profiler", argEnableProfiler,
        "Enable time profiler by default in all modules (Default: NO)");

    bool argEnableProfilerWhole = false;
    cli.add_flag(
        "--profiler-whole", argEnableProfilerWhole,
        "Enable whole-history time profiler in all modules (Default: NO). "
        "**DO NOT** use in production, only to benchmark short runs "
        "(unbounded memory usage)");

    bool argRttiListAll = false;
    cli.add_flag(
        "--rtti-list-all", argRttiListAll,
        "Loads all MOLA modules, then list all classes registered via "
        "mrpt::rtti, and exits.");

    std::string argRttiListChildren;
    cli.add_option(
        "--rtti-children-of", argRttiListChildren,
        "Loads all MOLA modules, then list all known classes that inherit "
        "from the given one, and exits.");

    bool argListModules = false;
    cli.add_flag(
        "--list-modules", argListModules,
        "Loads all MOLA modules, then list them. It also shows the list of "
        "paths in which the program looks for module dynamic libraries, "
        "then exits.");

    bool argListModuleSharedDirs = false;
    cli.add_flag(
        "--list-module-shared-dirs", argListModuleSharedDirs,
        "Finds all MOLA module source/shared directories, then list them. "
        "Paths can be added with the environment variable "
        "MOLA_MODULES_SHARED_PATH.");

    // Handle special ROS arguments (if mola-cli is launched as a ROS node)
    // before handling (argc,argv) to CLI11:
    std::optional<std::vector<std::string>> rosArgs;
    std::vector<std::string>                otherArgs;
    for (int i = 0; i < argc; i++)
    {
      const auto sArg = std::string(argv[i]);
      if (sArg == "--ros-args" && !rosArgs) rosArgs.emplace();

      if (rosArgs)
        rosArgs->push_back(sArg);
      else
        otherArgs.push_back(sArg);
    }
    // handle the case "--ros-args\n"
    if (rosArgs && rosArgs->empty()) rosArgs.reset();

    std::vector<const char*> argvBis;
    for (const auto& s : otherArgs) argvBis.push_back(s.c_str());
    const int argcBis = static_cast<int>(argvBis.size());

    // Parse arguments:
    try
    {
      cli.parse(argcBis, argvBis.data());
    }
    catch (const CLI::ParseError& e)
    {
      return cli.exit(e);
    }

    mola_install_signal_handler();

    // Different tasks that can be done with mola-cli:
    if (argRttiListAll)  //
      return mola_cli_rtti_list_all();

    if (!argRttiListChildren.empty()) return mola_cli_rtti_list_child(argRttiListChildren);

    if (argListModules)  //
      return mola_cli_list_modules();

    if (argListModuleSharedDirs) return mola_cli_list_module_shared_dirs();

    // Default task:
    return mola_cli_launch_slam(
        argYamlCfg, argVerbosityLevel, argEnableProfiler, argEnableProfilerWhole, rosArgs);
  }
  catch (std::exception& e)
  {
    mola::pretty_print_exception(e, "[mola-cli] Exit due to exception:");
    return 1;
  }
}
