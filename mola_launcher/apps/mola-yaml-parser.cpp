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
 * @file   mola-yaml-parser.cpp
 * @brief  main() for mola-yaml-parser: parses YAML files with MOLA extensions
 * @author Jose Luis Blanco Claraco
 * @date   Sep 9, 2019
 */

#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli{"mola-yaml-parser"};

    bool argNoIncludes = false;
    cli.add_flag(
        "--no-includes", argNoIncludes, "Disables solving YAML `$include{}`s (Default: NO)");

    bool argNoCmdRuns = false;
    cli.add_flag("--no-cmd-runs", argNoCmdRuns, "Disables solving YAML `$(cmd)`s (Default: NO)");

    bool argNoEnvVars = false;
    cli.add_flag("--no-env-vars", argNoEnvVars, "Disables solving YAML `${xxx}`s (Default: NO)");

    std::string argInputFile;
    cli.add_option("YAML_file", argInputFile, "Input YAML file (required) (*.yml)")->required();

    try
    {
      cli.parse(argc, argv);
    }
    catch (const CLI::ParseError& e)
    {
      return cli.exit(e);
    }

    // MOLA-specific parsing:
    mola::YAMLParseOptions options;
    if (argNoIncludes) options.doIncludes = false;
    if (argNoCmdRuns) options.doCmdRuns = false;
    if (argNoEnvVars) options.doEnvVars = false;

    auto d = mola::load_yaml_file(argInputFile, options);

    // Dump output:
    d.printAsYAML(std::cout);
    std::cout << "\n";

    return 0;
  }
  catch (std::exception& e)
  {
    mola::pretty_print_exception(e, "[mola-yaml-parser] Exit due to exception:");

    return 1;
  }
}
