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
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/filesystem.h>

#include <csignal>  // sigaction
#include <iostream>
#include <string>

#if defined(WIN32)
#include <windows.h>  // SetConsoleCtrlHandler
#endif
// TODO(jlbc): win32: add SetConsoleCtrlHandler

// Declare supported cli switches ===========
struct Cli
{
  TCLAP::CmdLine                        cmd{"mola-cli"};
  TCLAP::UnlabeledValueArg<std::string> arg_yaml_cfg{
      "config", "Input YAML config file (required) (*.yaml)", false, "", "mola-system.yaml", cmd};

  TCLAP::ValueArg<std::string> arg_verbosity_level{
      "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)", false, "",
      "INFO", cmd};

  TCLAP::SwitchArg arg_enable_profiler{
      "p", "profiler", "Enable time profiler by default in all modules (Default: NO)", cmd};

  TCLAP::SwitchArg arg_enable_profiler_whole{
      "", "profiler-whole",
      "Enable whole-history time profiler in all modules (Default: NO). **DO "
      "NOT** use in production, only to benchmark short runs (unbounded "
      "memory "
      "usage)",
      cmd};

  TCLAP::SwitchArg arg_rtti_list_all{
      "", "rtti-list-all",
      "Loads all MOLA modules, then list all classes registered via "
      "mrpt::rtti, "
      "and exits.",
      cmd};

  TCLAP::ValueArg<std::string> arg_rtti_list_children{
      "",
      "rtti-children-of",
      "Loads all MOLA modules, then list all known classes that inherit from "
      "the "
      "given one, and exits.",
      false,
      "",
      "mp2p_icp::ICP_Base",
      cmd};

  TCLAP::SwitchArg arg_list_modules{
      "", "list-modules",
      "Loads all MOLA modules, then list them. It also shows the list of "
      "paths "
      "in which the program looks for module dynamic libraries, then exits.",
      cmd};

  TCLAP::SwitchArg arg_list_module_shared_dirs{
      "", "list-module-shared-dirs",
      "Finds all MOLA module source/shared directories, then list them. "
      "Paths "
      "can be added with the environment variable MOLA_MODULES_SHARED_PATH.",
      cmd};
};

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
}

// Default task for mola-cli: launching a SLAM system
// -----------------------------------------------------
int mola_cli_launch_slam(Cli& cli, const std::optional<std::vector<std::string>>& rosArgs)
{
  using namespace std::string_literals;

  // Load YAML config file:
  if (!cli.arg_yaml_cfg.isSet())
  {
    TCLAP::ArgException e("mola-system.yaml is required to launch a SLAM system.");
    cli.cmd.getOutput()->failure(cli.cmd, e);
    return 1;
  }
  const auto file_yml = cli.arg_yaml_cfg.getValue();

  // replace a special variable for ROS args:
  mola::YAMLParseOptions po;

  if (rosArgs)
  {
    std::string strRosArgs = "[";
    for (const auto& s : *rosArgs) strRosArgs += " '"s + s + "', "s;
    strRosArgs += "]";
    po.variables["ROS_ARGS"] = strRosArgs;
  }

  // Load YAML from file:
  auto cfg = mola::load_yaml_file(file_yml, po);

  mola::MolaLauncherApp app;
  theApp = &app;  // for the signal handler

  if (cli.arg_verbosity_level.isSet())
  {
    using vl     = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    const auto v = vl::name2value(cli.arg_verbosity_level.getValue());
    app.setVerbosityLevel(v);
  }

  app.profiler_.enable(cli.arg_enable_profiler.isSet() || cli.arg_enable_profiler_whole.isSet());
  app.profiler_.enableKeepWholeHistory(cli.arg_enable_profiler_whole.isSet());

  // Create SLAM system:
  app.setup(cfg, mrpt::system::extractFileDirectory(file_yml));

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
int mola_cli_rtti_list_child(Cli& cli)
{
  mola::MolaLauncherApp app;
  theApp = &app;  // for the signal handler

  app.scanAndLoadLibraries();

  const auto parentName = cli.arg_rtti_list_children.getValue();

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
    Cli cli;

    // Handle special ROS arguments (if mola-cli is launched as a ROS node)
    // before handling (argc,argv) to tclap:
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
    if (!cli.cmd.parse(argcBis, argvBis.data())) return 1;  // should exit.

    mola_install_signal_handler();

    // Different tasks that can be dine with mola-cli:
    if (cli.arg_rtti_list_all.isSet())  //
      return mola_cli_rtti_list_all();

    if (cli.arg_rtti_list_children.isSet()) return mola_cli_rtti_list_child(cli);

    if (cli.arg_list_modules.isSet())  //
      return mola_cli_list_modules();

    if (cli.arg_list_module_shared_dirs.isSet()) return mola_cli_list_module_shared_dirs();

    // Default task:
    return mola_cli_launch_slam(cli, rosArgs);

    return 0;
  }
  catch (std::exception& e)
  {
    mola::pretty_print_exception(e, "[mola-cli] Exit due to exception:");
    return 1;
  }
}
