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
 * @file   MapServer.h
 * @brief  Map saving/loading services offered by MOLA modules
 * @author Jose Luis Blanco Claraco
 * @date   Aug 18, 2024
 */
#pragma once

#include <cstdlib>
#include <string>

namespace mola
{
/** Virtual interface for map server services offered by MOLA modules
 * \ingroup mola_kernel_interfaces_grp */
class MapServer
{
 public:
  MapServer() = default;

  /** @name Virtual interface of MapServer
   *{ */

  struct ReturnStatus
  {
    ReturnStatus() = default;

    bool        success = false;
    std::string error_message;
  };

  /** Loads a map from file(s) and sets it as active current map.
   * Different implementations may use one or more files to store map as
   * files.
   *
   *  \param[in] path File name(s) prefix for the map to load. Do not add file
   * extension.
   */
  virtual ReturnStatus map_load([[maybe_unused]] const std::string& path) { return {}; }

  /** Saves a map from file(s) and sets it as active current map.
   * Different implementations may use one or more files to store map as
   * files.
   *
   *  \param[in] path File name(s) prefix for the map to save. Do not add file
   * extension.
   */
  virtual ReturnStatus map_save([[maybe_unused]] const std::string& path) { return {}; }

  /** @} */
};

}  // namespace mola
