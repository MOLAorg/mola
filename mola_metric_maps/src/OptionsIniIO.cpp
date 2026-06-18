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
 * @file   OptionsIniIO.cpp
 * @brief  Helpers to export/import CLoadableOptions of metric map layers to/from .ini files
 * @author Jose Luis Blanco Claraco
 * @date   Jun 18, 2026
 */

#include <mola_metric_maps/OptionsCapable.h>
#include <mola_metric_maps/OptionsIniIO.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMap.h>

#include <sstream>

using namespace mola;

namespace
{
std::string trim(const std::string& s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
  {
    return {};
  }
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::string sectionFor(const std::string& layerName, const std::string& structName)
{
  return layerName + "." + structName;
}

}  // namespace

void mola::exportLoadableOptionsToIni(
    const mrpt::config::CLoadableOptions& opts, mrpt::config::CConfigFileBase& cfg,
    const std::string& section, const std::set<std::string>& excludeKeys)
{
  std::ostringstream ss;
  opts.dumpToTextStream(ss);

  std::istringstream iss(ss.str());
  std::string        line;
  while (std::getline(iss, line))
  {
    const auto eq = line.find('=');
    if (eq == std::string::npos)
    {
      continue;  // not a "key = value" line (e.g. banner/blank lines)
    }

    const std::string key   = trim(line.substr(0, eq));
    std::string       value = trim(line.substr(eq + 1));
    if (key.empty() || excludeKeys.count(key) != 0)
    {
      continue;
    }
    if (value.find("...") != std::string::npos || value.find('(') != std::string::npos)
    {
      // Some dumpToTextStream() implementations print human-readable, non-parseable summaries
      // for vector fields (e.g. COccupancyGridMap2D::TLikelihoodOptions::OWA_weights, truncated
      // with "..." and suffixed with "(size=N)"). Skip them, leaving that one field at whatever
      // value it had when later imported.
      continue;
    }
    if (value == "Y" || value == "N")
    {
      // Some dumpToTextStream() implementations print bools as a bare 'Y'/'N' character (e.g.
      // COccupancyGridMap2D::TLikelihoodOptions), but CConfigFileBase::read_bool() only
      // recognizes "true"/"false"/"yes"/"no"/"1"/"0" -- normalize to a value it understands.
      value = (value == "Y") ? "YES" : "NO";
    }

    cfg.write(section, key, value);
  }
}

bool mola::importLoadableOptionsFromIni(
    mrpt::config::CLoadableOptions& opts, const mrpt::config::CConfigFileBase& cfg,
    const std::string& section)
{
  if (!cfg.sectionExists(section))
  {
    return false;
  }
  opts.loadFromConfigFile(cfg, section);
  return true;
}

bool mola::exportMapLayerOptionsToIni(
    mrpt::maps::CMetricMap& map, mrpt::config::CConfigFileBase& cfg, const std::string& layerName)
{
  // Generic path: any class (present or future) implementing mola::OptionsCapable is handled
  // uniformly, without needing to know its concrete type or the names of its options structures.
  if (auto* capable = dynamic_cast<mola::OptionsCapable*>(&map))
  {
    cfg.write(layerName, "class", std::string(map.GetRuntimeClass()->className));
    for (const auto& [name, opts] : capable->optionsByName())
    {
      exportLoadableOptionsToIni(*opts, cfg, sectionFor(layerName, name));
    }
    return true;
  }

  // Fallback for classes from outside this library, which cannot implement
  // mola::OptionsCapable directly:
  if (const auto* m = dynamic_cast<const mrpt::maps::COccupancyGridMap2D*>(&map))
  {
    cfg.write(layerName, "class", std::string(map.GetRuntimeClass()->className));
    // "horizontalTolerance" is excluded: COccupancyGridMap2D::TInsertionOptions dumps it in
    // radians (plain LOADABLEOPTS_DUMP_VAR) but loads it as degrees (MRPT_LOAD_CONFIG_VAR_DEGREESf)
    // -- an inconsistency in MRPT itself that would silently scale the value by ~57x on import.
    exportLoadableOptionsToIni(
        m->insertionOptions, cfg, sectionFor(layerName, "insertionOptions"),
        {"horizontalTolerance"});
    // "rayTracing_decimation" is excluded: COccupancyGridMap2D::TLikelihoodOptions dumps it, but
    // loadFromConfigFile() never reads it back (a no-op, write-only field in MRPT's
    // implementation).
    exportLoadableOptionsToIni(
        m->likelihoodOptions, cfg, sectionFor(layerName, "likelihoodOptions"),
        {"rayTracing_decimation"});
    return true;
  }
  // Generic fallback: covers CSimplePointsMap, CPointsMapXYZI, CPointsMapXYZIRT,
  // CColouredPointsMap, CWeightedPointsMap, CGenericPointsMap, etc, since they all share
  // the same CPointsMap::T{Insertion,Likelihood,Render}Options structs.
  if (const auto* m = dynamic_cast<const mrpt::maps::CPointsMap*>(&map))
  {
    cfg.write(layerName, "class", std::string(map.GetRuntimeClass()->className));
    exportLoadableOptionsToIni(m->insertionOptions, cfg, sectionFor(layerName, "insertionOptions"));
    exportLoadableOptionsToIni(
        m->likelihoodOptions, cfg, sectionFor(layerName, "likelihoodOptions"));
    exportLoadableOptionsToIni(m->renderOptions, cfg, sectionFor(layerName, "renderOptions"));
    return true;
  }

  return false;  // unknown/unsupported map class
}

bool mola::importMapLayerOptionsFromIni(
    mrpt::maps::CMetricMap& map, const mrpt::config::CConfigFileBase& cfg,
    const std::string& layerName, std::vector<std::string>* appliedSections,
    std::vector<std::string>* rejectedSections)
{
  // Generic path, mirroring exportMapLayerOptionsToIni() above.
  if (auto* capable = dynamic_cast<mola::OptionsCapable*>(&map))
  {
    for (const auto& [name, opts] : capable->optionsByName())
    {
      const auto section = sectionFor(layerName, name);
      if (!cfg.sectionExists(section))
      {
        continue;
      }

      if (name == "creationOptions")
      {
        // Creation options may be unsafe to change in place (e.g. a voxel size): let the map
        // itself decide, instead of blindly overwriting the live struct through `opts`.
        if (capable->trySetCreationOptions(cfg, section))
        {
          if (appliedSections)
          {
            appliedSections->push_back(section);
          }
        }
        else if (rejectedSections)
        {
          rejectedSections->push_back(section);
        }
        continue;
      }

      opts->loadFromConfigFile(cfg, section);
      if (appliedSections)
      {
        appliedSections->push_back(section);
      }
    }
    return true;
  }

  // Fallback for classes from outside this library:
  const auto apply = [&](mrpt::config::CLoadableOptions& opts, const std::string& structName)
  {
    const auto section = sectionFor(layerName, structName);
    if (importLoadableOptionsFromIni(opts, cfg, section) && appliedSections)
    {
      appliedSections->push_back(section);
    }
  };

  if (auto* m = dynamic_cast<mrpt::maps::COccupancyGridMap2D*>(&map))
  {
    apply(m->insertionOptions, "insertionOptions");
    apply(m->likelihoodOptions, "likelihoodOptions");
    return true;
  }
  if (auto* m = dynamic_cast<mrpt::maps::CPointsMap*>(&map))
  {
    apply(m->insertionOptions, "insertionOptions");
    apply(m->likelihoodOptions, "likelihoodOptions");
    apply(m->renderOptions, "renderOptions");
    return true;
  }

  return false;  // unknown/unsupported map class
}
