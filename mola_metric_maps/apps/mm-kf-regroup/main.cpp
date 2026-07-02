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

  mola::KeyframePointCloudMap::RegroupParams params;

  Args()
  {
    // Sensible default for the tool: decimate merged super-keyframes so the
    // heavy inter-keyframe overlap does not blow up the output size. 0 disables.
    params.merge_decimate_voxel = 0.2;
  }
};

void run_regroup(const Args& args)
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

  std::cout << "[mm-kf-regroup] Reading input map from: '" << args.input << "'..." << std::endl;

  mp2p_icp::metric_map_t mm;
  mm.load_from_file(args.input);

  const auto logCb = [](const std::string& s) { std::cout << s << std::endl; };

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

    std::cout << "[mm-kf-regroup] Regrouping layer '" << layerName << "'..." << std::endl;
    auto regrouped = kfMap->regroupKeyframes(args.params, logCb);
    layer          = regrouped;  // replace the layer in-place
    numProcessed++;
  }

  if (numProcessed == 0)
  {
    throw std::runtime_error(
        "No mola::KeyframePointCloudMap layer was found/processed in the input map.");
  }

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
           "covered. Smaller => more inter-group overlap.")
        ->check(CLI::Range(0.0, 1.0));

    cli.add_option(
           "--extent-factor", args.params.extent_factor,
           "Super-keyframe spatial extent cap, as a multiple of the seed keyframe sensing radius.")
        ->check(CLI::PositiveNumber);

    cli.add_option(
        "--decimate-voxel", args.params.merge_decimate_voxel,
        "If >0, voxel-downsample [m] each merged super-keyframe cloud to bound its size.");

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
