/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-yaml-parser.cpp
 * @brief  main() for mola-yaml-parser: parses YAML files with MOLA extensions
 * @author Jose Luis Blanco Claraco
 * @date   Sep 9, 2019
 */

#include <mola-kernel/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/system/filesystem.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

// Declare supported cli switches ===========
static TCLAP::CmdLine   cmd("mola-yaml-parser");
static TCLAP::SwitchArg arg_no_includes(
    "", "no-includes", "Disables solving YAML `$include{}`s (Default: NO)",
    cmd);
static TCLAP::SwitchArg arg_no_cmd_runs(
    "", "no-cmd-runs", "Disables solving YAML `$(cmd)`s (Default: NO)", cmd);
static TCLAP::SwitchArg arg_no_env_vars(
    "", "no-env-vars", "Disables solving YAML `${xxx}`s (Default: NO)", cmd);
static TCLAP::UnlabeledValueArg<std::string> arg_input_files(
    "YAML_file", "Input YAML file (required) (*.yml)", true, "params.yml",
    "YAML files", cmd);

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        const auto filName = arg_input_files.getValue();
        ASSERT_FILE_EXISTS_(filName);

        // Load & parse YAML file:
        YAML::Node root = YAML::LoadFile(filName);

        // MOLA-specific parsing:
        mola::YAMLParseOptions options;
        if (arg_no_includes.isSet()) options.doIncludes = false;
        if (arg_no_cmd_runs.isSet()) options.doCmdRuns = false;
        if (arg_no_env_vars.isSet()) options.doEnvVars = false;

        const std::string parsedTxt =
            mola::parseYaml(mola::yaml2string(root), options);

        // Dump output:
        std::cout << parsedTxt << std::endl;

        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "[mola-yaml-parser] Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
