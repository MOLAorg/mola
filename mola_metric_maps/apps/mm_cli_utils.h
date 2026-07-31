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
 * @file   mm_cli_utils.h
 * @brief  Generic helpers shared by the .mm-file CLI tools: plugin loading,
 *         map loading, and iteration over layers of a given map type.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 31, 2026
 */
#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

namespace mola::mm_cli
{
/// Loads one or more comma-separated *.so plugin modules (no-op if empty).
inline void loadPlugins(const std::string& plugins)
{
  if (plugins.empty())
  {
    return;
  }
  std::string errMsg;
  std::cout << "Loading plugin(s): " << plugins << std::endl;
  if (!mrpt::system::loadPluginModules(plugins, errMsg))
  {
    throw std::runtime_error(errMsg);
  }
}

/// Loads plugins (if any) and reads a `.mm` metric map from disk. `tag` is a
/// short tool name used to prefix log lines.
inline mp2p_icp::metric_map_t loadMap(
    const std::string& input, const std::string& plugins, const std::string& tag)
{
  loadPlugins(plugins);

  ASSERT_FILE_EXISTS_(input);

  std::cout << "[" << tag << "] Reading input map from: '" << input << "'..." << std::endl;

  mp2p_icp::metric_map_t mm;
  mm.load_from_file(input);
  return mm;
}

/// Invokes `fn(layerName, typedLayer, layerSlot)` for every layer of `mm`
/// whose runtime type is `MapT`. `layerSlot` is a mutable reference to the
/// layer pointer in `mm.layers`, so callers can replace the layer in place if
/// needed.
///
/// If `layerName` is non-empty, only that layer is processed and it is an
/// error (throws) if it exists but is not a `MapT`. If `layerName` is empty,
/// non-matching layers are silently skipped. `typeName` is used only for
/// error messages (e.g. "mola::IncrementalPointCloud").
///
/// Returns the number of layers processed.
template <typename MapT>
inline size_t forEachMapLayerOfType(
    mp2p_icp::metric_map_t& mm, const std::string& layerName, const std::string& typeName,
    const std::function<
        void(const std::string&, const typename MapT::Ptr&, mrpt::maps::CMetricMap::Ptr&)>& fn)
{
  // Distinguish "requested layer absent" from "no layers of this type at all".
  if (!layerName.empty() && mm.layers.find(layerName) == mm.layers.end())
  {
    throw std::runtime_error("Layer '" + layerName + "' not found in the input map.");
  }

  size_t numProcessed = 0;
  for (auto& [name, layer] : mm.layers)
  {
    if (!layer)
    {
      continue;
    }
    if (!layerName.empty() && name != layerName)
    {
      continue;
    }

    auto typedLayer = std::dynamic_pointer_cast<MapT>(layer);
    if (!typedLayer)
    {
      if (!layerName.empty())
      {
        throw std::runtime_error(
            "Layer '" + name + "' is not a " + typeName + " (it is a '" +
            layer->GetRuntimeClass()->className + "').");
      }
      continue;  // silently skip non-matching layers when processing all
    }

    fn(name, typedLayer, layer);
    numProcessed++;
  }

  if (numProcessed == 0)
  {
    throw std::runtime_error("No " + typeName + " layer was found/processed in the input map.");
  }
  return numProcessed;
}
}  // namespace mola::mm_cli
