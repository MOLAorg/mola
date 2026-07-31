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
 * @file   mm-ipc-bake-kdtree/main.cpp
 * @brief  CLI tool to bake the k-d tree index of IncrementalPointCloud layers
 *         into a .mm file so it does not have to be rebuilt (an O(N log N)
 *         bulk build) every time the map is loaded.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 31, 2026
 */

#include <mola_metric_maps/IncrementalPointCloud.h>
#include <mp2p_icp/metricmap.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <stdexcept>

#include "../mm_cli_utils.h"

namespace
{
struct Args
{
  std::string input;
  std::string output;
  std::string layer;  // empty: process all IncrementalPointCloud layers
  std::string plugins;
  bool        disable = false;  // if set, strip the cached k-d tree instead of adding it
  bool        kdtree  = true;  // bake the k-d tree index
};

void run_bake(const Args& args)
{
#if !defined(MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE)
  if (!args.disable && args.kdtree)
  {
    std::cout << "[mm-ipc-bake-kdtree] WARNING: this build's nanoflann lacks the incremental "
                 "index's save/load API (requires nanoflann >= 1.11.0); k-d tree baking is a "
                 "no-op and no k-d tree data will be written."
              << std::endl;
  }
#endif

  mp2p_icp::metric_map_t mm = mola::mm_cli::loadMap(args.input, args.plugins, "mm-ipc-bake-kdtree");

  const size_t numProcessed = mola::mm_cli::forEachMapLayerOfType<mola::IncrementalPointCloud>(
      mm, args.layer, "mola::IncrementalPointCloud",
      [&](const std::string& layerName, const mola::IncrementalPointCloud::Ptr& ipcMap,
          mrpt::maps::CMetricMap::Ptr& /*layerSlot*/)
      {
        ipcMap->creationOptions.serialize_kdtree = !args.disable && args.kdtree;
        std::cout << "[mm-ipc-bake-kdtree] Layer '" << layerName << "': serialize_kdtree -> "
                  << (ipcMap->creationOptions.serialize_kdtree ? "true" : "false")
                  << " (when enabled, a throwaway index is built during save over the "
                     "compacted points that are written)."
                  << std::endl;
      });

  std::cout << "[mm-ipc-bake-kdtree] Writing output map to: '" << args.output << "'..."
            << std::endl;
  if (!mm.save_to_file(args.output))
  {
    throw std::runtime_error("Error writing output map file: '" + args.output + "'");
  }

  std::cout << "[mm-ipc-bake-kdtree] Done. Processed " << numProcessed << " layer(s)." << std::endl;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli{
        "mm-ipc-bake-kdtree: cache the IncrementalPointCloud k-d tree index inside a .mm file so "
        "it is not rebuilt on every load"};

    Args args;

    cli.add_option("-i,--input", args.input, "Input metric map file (*.mm)")
        ->required()
        ->check(CLI::ExistingFile);

    cli.add_option("-o,--output", args.output, "Output metric map file (*.mm)")->required();

    cli.add_option(
        "--layer", args.layer,
        "Name of the IncrementalPointCloud layer to process. If omitted, all such layers are "
        "processed.");

    cli.add_flag(
        "--disable", args.disable,
        "Instead of baking, strip any cached data (sets serialize_kdtree=false).");

    cli.add_flag(
        "--kdtree,!--no-kdtree", args.kdtree,
        "Bake the k-d tree index (default: on). Requires a nanoflann build with the incremental "
        "index's save/load API.");

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
