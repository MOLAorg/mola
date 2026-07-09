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
 * @file   mm-kf-regroup/main.cpp
 * @brief  CLI tool to regroup the keyframes of a KeyframePointCloudMap layer
 *         into fewer, larger, overlapping "super-keyframes" for fast
 *         localization-only operation.
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

  mola::KeyframePointCloudMap::RegroupParams params;

  Args()
  {
    // 0 disables.
    params.merge_decimate_voxel = 0;
  }
};

void run_regroup(const Args& args)
{
  mp2p_icp::metric_map_t mm = mola::kf_cli::loadMap(args.input, args.plugins, "mm-kf-regroup");

  const auto logCb = [](const std::string& s) { std::cout << s << std::endl; };

  const size_t numProcessed = mola::kf_cli::forEachKeyframeMapLayer(
      mm, args.layer,
      [&](const std::string& layerName, const mola::KeyframePointCloudMap::Ptr& kfMap,
          mrpt::maps::CMetricMap::Ptr& layerSlot)
      {
        std::cout << "[mm-kf-regroup] Regrouping layer '" << layerName << "'..." << std::endl;
        layerSlot = kfMap->regroupKeyframes(args.params, logCb);  // replace the layer in-place
      });

  std::cout << "[mm-kf-regroup] Writing output map to: '" << args.output << "'..." << std::endl;
  if (!mm.save_to_file(args.output))
  {
    throw std::runtime_error("Error writing output map file: '" + args.output + "'");
  }

  std::cout << "[mm-kf-regroup] Done. Processed " << numProcessed << " layer(s)." << std::endl;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli{
        "mm-kf-regroup: group the keyframes of a KeyframePointCloudMap into fewer, larger, "
        "overlapping super-keyframes for fast localization-only operation"};

    Args args;

    cli.add_option("-i,--input", args.input, "Input metric map file (*.mm)")
        ->required()
        ->check(CLI::ExistingFile);

    cli.add_option("-o,--output", args.output, "Output metric map file (*.mm)")->required();

    cli.add_option(
        "--layer", args.layer,
        "Name of the KeyframePointCloudMap layer to process. If omitted, all such layers are "
        "processed.");

    cli.add_option(
        "--voxel-size", args.params.voxel_size,
        "Voxel resolution [m] used to compute keyframe overlaps. <=0: auto (default).");

    cli.add_option(
           "--edge-overlap", args.params.edge_overlap,
           "Min pairwise voxel-overlap [0..1] to link two keyframes in the graph.")
        ->check(CLI::Range(0.0, 1.0));

    cli.add_option(
           "--core-fraction", args.params.core_fraction,
           "Inner-core fraction (0..1] of a super-keyframe extent within which members are marked "
           "covered. Smaller => more inter-group overlap (default: 0.85, i.e. ~15% overlap).")
        ->check(CLI::Range(0.0, 1.0));

    cli.add_option(
           "--extent-factor", args.params.extent_factor,
           "Super-keyframe spatial extent cap, as a multiple of the seed keyframe sensing radius.")
        ->check(CLI::PositiveNumber);

    cli.add_option(
        "--decimate-voxel", args.params.merge_decimate_voxel,
        "If >0, voxel-downsample [m] each merged super-keyframe cloud to bound its size.");

    cli.add_flag(
        "--unify-all", args.params.unify_all,
        "Bypass overlap-graph clustering and merge ALL keyframes of each processed layer into a "
        "single super-keyframe (anchored at the first/oldest keyframe's pose). "
        "--decimate-voxel still applies; --edge-overlap/--core-fraction/--extent-factor/"
        "--island-merge-fraction are ignored.");

    cli.add_option(
           "--island-merge-fraction", args.params.island_merge_fraction,
           "Clusters whose point count falls below this fraction of the largest cluster's are "
           "absorbed into their nearest neighbor instead of being left as standalone, "
           "under-populated super-keyframes. 0 disables this (default: 0.025=2.5%).")
        ->check(CLI::Range(0.0, 1.0));

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

    run_regroup(args);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
