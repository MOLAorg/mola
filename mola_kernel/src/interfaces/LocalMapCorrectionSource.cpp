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
 * @file   LocalMapCorrectionSource.cpp
 * @brief  Virtual interface for modules that correct the keyframes of a front end.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_kernel/interfaces/LocalMapCorrectionSource.h>
#include <mrpt/core/lock_helper.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <utility>

using namespace mola;

LocalMapCorrectionSource::~LocalMapCorrectionSource() = default;

LocalMapCorrectionSource::subscription_id_t
    LocalMapCorrectionSource::subscribe_to_local_map_corrections(
        const correction_callback_t& callback)
{
  if (!callback)
  {
    throw std::invalid_argument("subscribe_to_local_map_corrections(): empty callback");
  }
  auto       lck = mrpt::lockHelper(subsMtx_);
  const auto id  = nextId_++;
  subs_.emplace(id, callback);
  return id;
}

void LocalMapCorrectionSource::unsubscribe_from_local_map_corrections(subscription_id_t id)
{
  // Waits for a dispatch in progress in another thread (a callback may also
  // unsubscribe from within itself: the mutex is recursive).
  std::lock_guard<std::recursive_mutex> dispatchLck(dispatchMtx_);
  correction_callback_t                 removed;
  {
    auto       lck = mrpt::lockHelper(subsMtx_);
    const auto it  = subs_.find(id);
    if (it == subs_.end())
    {
      return;
    }
    removed = std::move(it->second);
    subs_.erase(it);
  }
  // `removed` is destroyed here, without subsMtx_ held.
}

bool LocalMapCorrectionSource::any_local_map_correction_subscriber() const
{
  auto lck = mrpt::lockHelper(subsMtx_);
  return !subs_.empty();
}

void LocalMapCorrectionSource::advertise_local_map_correction(const LocalMapCorrection& c)
{
  // Callbacks run without subsMtx_, so they can (un)subscribe or query.
  std::lock_guard<std::recursive_mutex>                            dispatchLck(dispatchMtx_);
  std::vector<std::pair<subscription_id_t, correction_callback_t>> subs;
  {
    auto lck = mrpt::lockHelper(subsMtx_);
    subs.assign(subs_.begin(), subs_.end());
  }
  for (const auto& [id, callback] : subs)
  {
    {
      // Skip callbacks removed by an earlier one in this same loop:
      auto lck = mrpt::lockHelper(subsMtx_);
      if (subs_.count(id) == 0)
      {
        continue;
      }
    }
    try
    {
      callback(c);
    }
    catch (const std::exception& e)
    {
      std::cerr << "[LocalMapCorrectionSource] Exception in callback: " << e.what() << "\n";
    }
    catch (...)
    {
      std::cerr << "[LocalMapCorrectionSource] Unknown exception in callback\n";
    }
  }
}
