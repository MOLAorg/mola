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
 * @file   Dataset_UI.h
 * @brief  Virtual interface for offline dataset sources to have a GUI
 * @author Jose Luis Blanco Claraco
 * @date   Feb 20, 2024
 */
#pragma once

#include <cstdlib>

namespace mola
{
/** Virtual base for offline dataset sources to have a GUI within MolaViz
 * \ingroup mola_kernel_interfaces_grp */
class Dataset_UI
{
 public:
  Dataset_UI() = default;

  /** @name Virtual interface of Dataset_UI
   *{ */

  /** Number of different time steps available to call getObservations() */
  virtual size_t datasetUI_size() const = 0;

  /** Returns the latest requested observation, range [0, datasetSize()] */
  virtual size_t datasetUI_lastQueriedTimestep() const = 0;

  virtual double datasetUI_playback_speed() const       = 0;
  virtual void   datasetUI_playback_speed(double speed) = 0;

  virtual bool datasetUI_paused() const      = 0;
  virtual void datasetUI_paused(bool paused) = 0;

  /** Forces continue replaying in this moment in time */
  virtual void datasetUI_teleport(size_t timestep) = 0;
  /** @} */
};

}  // namespace mola
