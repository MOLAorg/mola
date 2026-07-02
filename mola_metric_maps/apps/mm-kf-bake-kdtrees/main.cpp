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

#include <CLI/CLI.hpp>
#include <iostream>
#include <stdexcept>

#include "../kf_cli_utils.h"

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
#if !defined(MRPT_HAS_KDTREE_SAVE_LOAD_INDEX)
  if (!args.disable)
  {
    std::cout << "[mm-kf-bake-kdtrees] WARNING: this MRPT build lacks the KD-tree save/load "
                 "index API; baking is a no-op and no KD-tree data will be written."
              << std::endl;
  }
#endif

  mp2p_icp::metric_map_t mm = mola::kf_cli::loadMap(args.input, args.plugins, "mm-kf-bake-kdtrees");

  const size_t numProcessed = mola::kf_cli::forEachKeyframeMapLayer(
      mm, args.layer,
      [&](const std::string& layerName, const mola::KeyframePointCloudMap::Ptr& kfMap,
          mrpt::maps::CMetricMap::Ptr& /*layerSlot*/)
      {
        kfMap->creationOptions.serialize_kdtrees = !args.disable;
        std::cout << "[mm-kf-bake-kdtrees] Layer '" << layerName << "': serialize_kdtrees -> "
                  << (kfMap->creationOptions.serialize_kdtrees ? "true" : "false")
                  << " (KD-trees are (re)built on save only if not already current)." << std::endl;
      });

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
