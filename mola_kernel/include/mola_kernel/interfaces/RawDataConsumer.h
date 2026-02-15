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
 * @file   RawDataConsumer.h
 * @brief  Virtual interface for raw data consumers, typically, SLAM front-ends
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */
#pragma once

#include <mrpt/obs/CObservation.h>

namespace mola
{
using CObservation = mrpt::obs::CObservation;

/** Virtual base for raw-observation consumers
 * \ingroup mola_kernel_interfaces_grp */
class RawDataConsumer
{
 public:
  RawDataConsumer() = default;

  /** @name Virtual interface of any RawDataConsumer
   *{ */

  /** To be called whenever a new observation arrives. It should return as
   * fast as possible, enqueuing the data for processing in another thread.
   */
  virtual void onNewObservation(const CObservation::ConstPtr& o) = 0;
  /** @} */
};

}  // namespace mola
