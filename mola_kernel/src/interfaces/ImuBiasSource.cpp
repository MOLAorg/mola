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

#include <mola_kernel/interfaces/ImuBiasSource.h>
#include <mrpt/core/lock_helper.h>

#include <exception>
#include <iostream>
#include <stdexcept>

using namespace mola;

ImuBiasSource::~ImuBiasSource() = default;

ImuBiasSource::subscription_id_t ImuBiasSource::subscribe_to_imu_bias_updates(
    const imu_bias_callback_t& callback)
{
  if (!callback)
  {
    throw std::invalid_argument("subscribe_to_imu_bias_updates(): empty callback");
  }
  auto       lck = mrpt::lockHelper(imuBiasSubsMtx_);
  const auto id  = imuBiasNextId_++;
  imuBiasSubs_.emplace(id, callback);
  return id;
}

void ImuBiasSource::unsubscribe_from_imu_bias_updates(subscription_id_t id)
{
  // Waits for a dispatch in progress in another thread (a callback may also
  // unsubscribe from within itself: the mutex is recursive).
  std::lock_guard<std::recursive_mutex> dispatchLck(imuBiasDispatchMtx_);
  imu_bias_callback_t                   removed;
  {
    auto       lck = mrpt::lockHelper(imuBiasSubsMtx_);
    const auto it  = imuBiasSubs_.find(id);
    if (it == imuBiasSubs_.end())
    {
      return;
    }
    removed = std::move(it->second);
    imuBiasSubs_.erase(it);
  }
  // `removed` is destroyed here, without imuBiasSubsMtx_ held.
}

bool ImuBiasSource::any_imu_bias_subscriber() const
{
  auto lck = mrpt::lockHelper(imuBiasSubsMtx_);
  return !imuBiasSubs_.empty();
}

void ImuBiasSource::advertise_updated_imu_bias(const ImuBias& b)
{
  // Callbacks run without imuBiasSubsMtx_, so they can (un)subscribe or query.
  std::lock_guard<std::recursive_mutex>                          dispatchLck(imuBiasDispatchMtx_);
  std::vector<std::pair<subscription_id_t, imu_bias_callback_t>> subs;
  {
    auto lck = mrpt::lockHelper(imuBiasSubsMtx_);
    subs.assign(imuBiasSubs_.begin(), imuBiasSubs_.end());
  }
  for (const auto& [id, callback] : subs)
  {
    {
      // Skip callbacks removed by an earlier one in this same loop:
      auto lck = mrpt::lockHelper(imuBiasSubsMtx_);
      if (imuBiasSubs_.count(id) == 0)
      {
        continue;
      }
    }
    try
    {
      callback(b);
    }
    catch (const std::exception& e)
    {
      std::cerr << "[ImuBiasSource] Exception in callback: " << e.what() << "\n";
    }
    catch (...)
    {
      std::cerr << "[ImuBiasSource] Unknown exception in callback\n";
    }
  }
}
