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
 * @file   OptionsCapable.h
 * @brief  Virtual interface for metric maps that expose their CLoadableOptions generically.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 18, 2026
 */
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>

#include <map>
#include <string>

namespace mola
{
/** Mixin interface for metric map classes that expose one or more
 *  `mrpt::config::CLoadableOptions` structures (e.g. "creationOptions", "insertionOptions",
 *  "likelihoodOptions", "renderOptions") in a generic, name-indexed way, so that callers (e.g.
 *  the `mm2ini`/`ini2mm` CLI tools) never need to know the concrete map class nor the exact
 *  set/names of options structures it defines -- works uniformly for all present and future
 *  classes implementing this interface.
 *
 *  "Creation options" (those that configure a map's internal structure, e.g. a voxel/grid size)
 *  are handled separately via trySetCreationOptions(), since changing them may be incompatible
 *  with already-inserted map contents.
 *
 *  \note This interface is a temporary home for functionality that may eventually be proposed
 *  for inclusion in MRPT itself (mrpt::maps::CMetricMap). Until then, it lives here.
 */
class OptionsCapable
{
 public:
  OptionsCapable()                                 = default;
  OptionsCapable(const OptionsCapable&)            = default;
  OptionsCapable& operator=(const OptionsCapable&) = default;
  OptionsCapable(OptionsCapable&&)                 = default;
  OptionsCapable& operator=(OptionsCapable&&)      = default;
  virtual ~OptionsCapable()                        = default;

  /** Maps an options-group name (e.g. "insertionOptions") to a pointer to the corresponding,
   *  live `CLoadableOptions` member. Pointers remain valid as long as `this` is alive, and point
   *  directly to the map's actual option members (no copies), so writes through them (e.g. via
   *  `loadFromConfigFile()`) take effect immediately.
   *
   *  Implementations should list every `CLoadableOptions` member they define, INCLUDING
   *  "creationOptions" if present (so it can be discovered/exported generically); however,
   *  callers must use trySetCreationOptions(), not a direct write through this pointer, to
   *  safely *modify* creation options.
   */
  [[nodiscard]] virtual std::map<std::string, mrpt::config::CLoadableOptions*> optionsByName() = 0;

  /** Attempts to apply new "creation options" (i.e. those that configure a map's internal
   *  structure, such as a voxel/grid size), read from the given `section` of `cfg`.
   *
   *  Many creation-time parameters are just runtime thresholds with no effect on already-built
   *  internal structures, so applying them in place is often possible even after the map holds
   *  data. Others (e.g. a voxel size) cannot be changed without discarding existing contents.
   *
   *  \return true if the new options were applied; false if doing so would require destroying
   *  the map's current contents (in which case the map is left unmodified) -- or if this map
   *  class does not define any "creationOptions" group at all (the default implementation).
   */
  virtual bool trySetCreationOptions(
      [[maybe_unused]] const mrpt::config::CConfigFileBase& cfg,
      [[maybe_unused]] const std::string&                   section)
  {
    return false;
  }
};

}  // namespace mola
