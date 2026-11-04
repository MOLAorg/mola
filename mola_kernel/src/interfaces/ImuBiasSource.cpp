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

using namespace mola;

ImuBiasSource::~ImuBiasSource() = default;

ImuBiasSource::subscription_id_t ImuBiasSource::subscribe_to_imu_bias_updates(
    const imu_bias_callback_t& callback)
{
  auto       lck = mrpt::lockHelper(imuBiasSubsMtx_);
  const auto id  = imuBiasNextId_++;
  imuBiasSubs_.emplace(id, callback);
  return id;
}

void ImuBiasSource::unsubscribe_from_imu_bias_updates(subscription_id_t id)
{
  auto lck = mrpt::lockHelper(imuBiasSubsMtx_);
  imuBiasSubs_.erase(id);
}

bool ImuBiasSource::any_imu_bias_subscriber() const
{
  auto lck = mrpt::lockHelper(imuBiasSubsMtx_);
  return !imuBiasSubs_.empty();
}

void ImuBiasSource::advertise_updated_imu_bias(const ImuBias& b)
{
  // Callbacks run without the lock, so they can (un)subscribe or query.
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
