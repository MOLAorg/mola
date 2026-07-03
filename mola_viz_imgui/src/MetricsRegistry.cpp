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
 * @file   MetricsRegistry.cpp
 * @brief  Thread-safe rolling store of metric channels for the ImGui plot windows.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include "MetricsRegistry.h"

namespace mola
{

void MetricChannelImpl::push(double t, double value)
{
  // Cheap, lock-free early-out: nobody is plotting anything at all.
  if (!any_window_open_->load(std::memory_order_relaxed))
  {
    return;
  }

  std::lock_guard<std::mutex> lk(mtx_);
  ring_.push_back({t, static_cast<float>(value)});
  prune_older_than_locked(t - retention_seconds.load(std::memory_order_relaxed));
  version_.fetch_add(1, std::memory_order_relaxed);
}

void MetricChannelImpl::prune_older_than_locked(double cutoff)
{
  while (!ring_.empty() && ring_.front().t < cutoff)
  {
    ring_.pop_front();
  }
}

void MetricChannelImpl::copy_span(
    double t_min, std::vector<double>& xs, std::vector<double>& ys) const
{
  xs.clear();
  ys.clear();

  std::lock_guard<std::mutex> lk(mtx_);
  for (const auto& s : ring_)
  {
    if (s.t < t_min)
    {
      continue;
    }
    xs.push_back(s.t);
    ys.push_back(s.v);
  }
}

namespace
{
/** Round-robin palette assigned to newly-registered channels. Matches
 *  ImPlot's default qualitative colormap so plot legends and any future
 *  per-channel swatch controls line up visually. */
constexpr ImVec4 kPalette[] = {
    {0.00f, 0.75f, 0.75f, 1.0f}, {1.00f, 0.49f, 0.06f, 1.0f}, {0.55f, 0.36f, 0.83f, 1.0f},
    {0.17f, 0.63f, 0.17f, 1.0f}, {0.84f, 0.15f, 0.16f, 1.0f}, {0.09f, 0.47f, 0.72f, 1.0f},
    {0.74f, 0.74f, 0.13f, 1.0f}, {0.89f, 0.47f, 0.76f, 1.0f},
};
}  // namespace

MetricChannelImpl::Ptr MetricsRegistry::get_or_create(
    const std::string& name, const std::string& unit)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (const auto it = channels_.find(name); it != channels_.end())
  {
    return it->second;
  }

  const ImVec4 color = kPalette[channels_.size() % (sizeof(kPalette) / sizeof(kPalette[0]))];
  auto         ch    = std::make_shared<MetricChannelImpl>(name, unit, color, any_window_open_);
  channels_.emplace(name, ch);
  return ch;
}

std::vector<std::string> MetricsRegistry::channel_names() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  std::vector<std::string>    names;
  names.reserve(channels_.size());
  for (const auto& [name, ch] : channels_) names.push_back(name);
  return names;
}

MetricChannelImpl::Ptr MetricsRegistry::find(const std::string& name) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  const auto                  it = channels_.find(name);
  return it != channels_.end() ? it->second : nullptr;
}

}  // namespace mola
