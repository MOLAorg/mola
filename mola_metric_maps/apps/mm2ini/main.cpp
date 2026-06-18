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
 * @file   mm2ini/main.cpp
 * @brief  CLI tool to export all CLoadableOptions of the layers in a .mm file into a .ini file
 * @author Jose Luis Blanco Claraco
 * @date   Jun 18, 2026
 */

#include <mola_metric_maps/OptionsIniIO.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <stdexcept>

void run_mm2ini(
    const std::string& filInput, const std::string& filOutput, const std::string& plugins)
{
  if (!plugins.empty())
  {
    std::string errMsg;
    std::cout << "Loading plugin(s): " << plugins << std::endl;
    if (!mrpt::system::loadPluginModules(plugins, errMsg))
    {
      throw std::runtime_error(errMsg);
    }
  }

  ASSERT_FILE_EXISTS_(filInput);

  std::cout << "[mm2ini] Reading input map from: '" << filInput << "'..." << std::endl;

  mp2p_icp::metric_map_t mm;
  mm.load_from_file(filInput);

  std::cout << "[mm2ini] Read " << mm.layers.size() << " layer(s). Exporting options to: '"
            << filOutput << "'" << std::endl;

  mrpt::config::CConfigFile cfg(filOutput);

  size_t numExported = 0;
  for (const auto& [layerName, layer] : mm.layers)
  {
    if (!layer)
    {
      continue;
    }
    if (!mola::exportMapLayerOptionsToIni(*layer, cfg, layerName))
    {
      std::cerr << "[mm2ini] WARNING: layer '" << layerName << "' has unsupported class '"
                << layer->GetRuntimeClass()->className << "', skipping it." << std::endl;
      continue;
    }
    std::cout << "[mm2ini]  - layer '" << layerName << "' (" << layer->GetRuntimeClass()->className
              << "): options exported." << std::endl;
    numExported++;
  }

  cfg.writeNow();

  std::cout << "[mm2ini] Done. Exported options for " << numExported << "/" << mm.layers.size()
            << " layer(s)." << std::endl;
}

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli{"mm2ini: export the options of all layers in a .mm file into a .ini file"};

    std::string argInput;
    cli.add_option("-i,--input", argInput, "Input metric map file (*.mm)")
        ->required()
        ->check(CLI::ExistingFile);

    std::string argOutput;
    cli.add_option("-o,--output", argOutput, "Output configuration file (*.ini)")->required();

    std::string argPlugins;
    cli.add_option(
        "-l,--load-plugins", argPlugins,
        "One or more (comma separated) *.so files to load as plugins");

    try
    {
      cli.parse(argc, argv);
    }
    catch (const CLI::ParseError& e)
    {
      return cli.exit(e);
    }

    run_mm2ini(argInput, argOutput, argPlugins);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
