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
 * @file   OptionsIniIO.h
 * @brief  Helpers to export/import CLoadableOptions of metric map layers to/from .ini files
 * @author Jose Luis Blanco Claraco
 * @date   Jun 18, 2026
 */
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CMetricMap.h>

#include <set>
#include <string>
#include <vector>

namespace mola
{
/** Dumps all parameters of `opts` (via its `dumpToTextStream()`, which all known
 * `CLoadableOptions` implement) as `key = value` entries under `section` in `cfg`.
 *
 * This indirect route (instead of calling `saveToConfigFile()` directly) is needed because
 * several `CLoadableOptions` subclasses in this library (and in MRPT itself) only override
 * `dumpToTextStream()`, not `saveToConfigFile()`, which would otherwise throw.
 *
 * Any key listed in `excludeKeys` is skipped entirely (neither written nor overwritten), which is
 * used to work around individual fields whose dumpToTextStream()/loadFromConfigFile() pair is
 * known to be inconsistent in some MRPT classes (e.g. unit mismatches, or values that get
 * truncated for display and can no longer be parsed back). Importing such a field then simply
 * leaves it at whatever value it already had.
 */
void exportLoadableOptionsToIni(
    const mrpt::config::CLoadableOptions& opts, mrpt::config::CConfigFileBase& cfg,
    const std::string& section, const std::set<std::string>& excludeKeys = {});

/** If `section` exists in `cfg`, calls `opts.loadFromConfigFile()` on it and returns true.
 * Otherwise leaves `opts` untouched and returns false.
 */
bool importLoadableOptionsFromIni(
    mrpt::config::CLoadableOptions& opts, const mrpt::config::CConfigFileBase& cfg,
    const std::string& section);

/** Exports all `CLoadableOptions` structures of `map` into `cfg`, one section per structure,
 * named `"<layerName>.<structName>"`. If `map` implements `mola::OptionsCapable`, the set of
 * structures (and their names) is obtained generically via `OptionsCapable::optionsByName()`
 * -- this covers all classes in this library, present and future, without needing per-class
 * code here. Otherwise, a fallback handles basic `mrpt::maps` classes (`CPointsMap` and
 * derivatives, `COccupancyGridMap2D`) via `dynamic_cast`.
 *
 * Also writes a `[<layerName>]` section with a `class` key holding the MRPT RTTI class name, used
 * by importMapLayerOptionsFromIni() as a sanity check.
 *
 * \return false if `map` is none of the known, supported classes (nothing is written).
 */
bool exportMapLayerOptionsToIni(
    mrpt::maps::CMetricMap& map, mrpt::config::CConfigFileBase& cfg, const std::string& layerName);

/** The reverse of exportMapLayerOptionsToIni(): for each of `map`'s `CLoadableOptions`
 * structures whose section is present in `cfg`, loads it from `cfg` into `map`.
 *
 * Sections absent from `cfg` are silently skipped, leaving the corresponding options untouched.
 * Names of the sections actually applied are appended to `appliedSections`, if provided.
 *
 * The "creationOptions" structure (if present) is handled specially: instead of being
 * overwritten directly, the new values are passed to `OptionsCapable::trySetCreationOptions()`,
 * which may refuse to apply them (e.g. a voxel size change that would require discarding the
 * map's current contents). Section names refused this way are appended to `rejectedSections`, if
 * provided, instead of `appliedSections`.
 *
 * \return false if `map` is none of the known, supported classes (nothing is read).
 */
bool importMapLayerOptionsFromIni(
    mrpt::maps::CMetricMap& map, const mrpt::config::CConfigFileBase& cfg,
    const std::string& layerName, std::vector<std::string>* appliedSections = nullptr,
    std::vector<std::string>* rejectedSections = nullptr);

}  // namespace mola
