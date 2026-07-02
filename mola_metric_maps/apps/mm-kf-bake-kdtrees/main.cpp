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
 * @file   mm-kf-bake-kdtrees/main.cpp
 * @brief  CLI tool to bake the per-keyframe KD-tree indices into a .mm file so
 *         they do not have to be rebuilt every time the map is loaded (fast
 *         localization-only startup).
 * @author Jose Luis Blanco Claraco
 * @date   Jul 2, 2026
 */

#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <stdexcept>

namespace
{
struct Args
{
  std::string input;
  std::string output;
  std::string layer;  // empty: process all KeyframePointCloudMap layers
  std::string plugins;
  bool        disable = false;  // if set, strip cached kd-trees instead of adding them
};

void run_bake(const Args& args)
{
  if (!args.plugins.empty())
  {
    std::string errMsg;
    std::cout << "Loading plugin(s): " << args.plugins << std::endl;
    if (!mrpt::system::loadPluginModules(args.plugins, errMsg))
    {
      throw std::runtime_error(errMsg);
    }
  }

  ASSERT_FILE_EXISTS_(args.input);

  std::cout << "[mm-kf-bake-kdtrees] Reading input map from: '" << args.input << "'..."
            << std::endl;

  mp2p_icp::metric_map_t mm;
  mm.load_from_file(args.input);

  size_t numProcessed = 0;
  for (auto& [layerName, layer] : mm.layers)
  {
    if (!layer)
    {
      continue;
    }
    if (!args.layer.empty() && layerName != args.layer)
    {
      continue;
    }

    auto kfMap = std::dynamic_pointer_cast<mola::KeyframePointCloudMap>(layer);
    if (!kfMap)
    {
      if (!args.layer.empty())
      {
        throw std::runtime_error(
            "Layer '" + layerName + "' is not a mola::KeyframePointCloudMap (it is a '" +
            layer->GetRuntimeClass()->className + "').");
      }
      continue;  // silently skip non-matching layers when processing all
    }

    kfMap->creationOptions.serialize_kdtrees = !args.disable;
    std::cout << "[mm-kf-bake-kdtrees] Layer '" << layerName << "': serialize_kdtrees -> "
              << (kfMap->creationOptions.serialize_kdtrees ? "true" : "false")
              << " (KD-trees are (re)built on save only if not already current)." << std::endl;
    numProcessed++;
  }

  if (numProcessed == 0)
  {
    throw std::runtime_error(
        "No mola::KeyframePointCloudMap layer was found/processed in the input map.");
  }

  std::cout << "[mm-kf-bake-kdtrees] Writing output map to: '" << args.output << "'..."
            << std::endl;
  if (!mm.save_to_file(args.output))
  {
    throw std::runtime_error("Error writing output map file: '" + args.output + "'");
  }

  std::cout << "[mm-kf-bake-kdtrees] Done. Processed " << numProcessed << " layer(s)." << std::endl;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli{
        "mm-kf-bake-kdtrees: cache the per-keyframe KD-tree indices inside a .mm file so they "
        "are not rebuilt on every load (fast localization-only startup)"};

    Args args;

    cli.add_option("-i,--input", args.input, "Input metric map file (*.mm)")
        ->required()
        ->check(CLI::ExistingFile);

    cli.add_option("-o,--output", args.output, "Output metric map file (*.mm)")->required();

    cli.add_option(
        "--layer", args.layer,
        "Name of the KeyframePointCloudMap layer to process. If omitted, all such layers are "
        "processed.");

    cli.add_flag(
        "--disable", args.disable,
        "Instead of baking, strip any cached KD-trees (sets serialize_kdtrees=false).");

    cli.add_option(
        "-l,--load-plugins", args.plugins,
        "One or more (comma separated) *.so files to load as plugins");

    try
    {
      cli.parse(argc, argv);
    }
    catch (const CLI::ParseError& e)
    {
      return cli.exit(e);
    }

    run_bake(args);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
