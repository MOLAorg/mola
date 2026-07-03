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
 * @file   MetricsRegistry.h
 * @brief  Thread-safe rolling store of metric channels for the ImGui plot windows.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <imgui.h>
#include <mola_kernel/interfaces/MetricChannel.h>

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace mola
{

/** One (t, value) sample. Value is stored as `float` to halve the memory
 *  footprint of the rolling buffer (the plotted span rarely needs more
 *  precision than that). */
struct MetricSample
{
  double t;
  float  v;
};

/** Concrete `MetricChannel`: a bounded, thread-safe rolling buffer of samples.
 *
 *  Modelled on `ConsoleLogSink`: a shared_ptr sink written to by producer
 *  threads and read by the GUI thread, a rolling buffer bounded by time, a
 *  lock-free `version()` change hint, and an atomic gate letting producers
 *  early-out cheaply while nobody is plotting.
 */
class MetricChannelImpl : public MetricChannel
{
 public:
  using Ptr = std::shared_ptr<MetricChannelImpl>;

  MetricChannelImpl(
      std::string channelName, std::string channelUnit, ImVec4 channelColor,
      std::shared_ptr<std::atomic<bool>> anyWindowOpen)
      : name(std::move(channelName)),
        unit(std::move(channelUnit)),
        color(channelColor),
        any_window_open_(std::move(anyWindowOpen))
  {
  }

  void push(double t, double value) override;

  /** Appends the samples with `t >= t_min` to the (caller-owned, reused)
   *  output buffers, oldest first. Both buffers use `double` -- ImPlot's
   *  PlotLine()/PlotScatter() require the x and y arrays to share one type. */
  void copy_span(double t_min, std::vector<double>& xs, std::vector<double>& ys) const;

  /** Bumped on every push(). Lock-free, so the renderer can skip
   *  copy_span() when nothing changed since the last rendered frame. */
  uint64_t version() const { return version_.load(std::memory_order_relaxed); }

  const std::string name;
  const std::string unit;
  const ImVec4      color;

  /** Rolling history length; the widest span among all plot windows
   *  currently showing this channel (capped by nothing here -- callers
   *  decide the cap; default 10 s, see `MolaVizImGuiCore::plots_default_retention_seconds_`). */
  std::atomic<double> retention_seconds{10.0};

 private:
  mutable std::mutex                 mtx_;
  std::deque<MetricSample>           ring_;
  std::atomic<uint64_t>              version_{0};
  std::shared_ptr<std::atomic<bool>> any_window_open_;

  void prune_older_than_locked(double cutoff);
};

/** Owns the set of named metric channels for one `MolaVizImGuiCore` instance. */
class MetricsRegistry
{
 public:
  /** Idempotent: returns the existing channel if `name` was already registered. */
  MetricChannelImpl::Ptr get_or_create(const std::string& name, const std::string& unit);

  /** Sorted list of registered channel names, for the "add channel" combo. */
  std::vector<std::string> channel_names() const;

  MetricChannelImpl::Ptr find(const std::string& name) const;

  /** Shared gate: producers early-out on push() when this is false. The
   *  renderer recomputes it once per frame from the set of open plot
   *  windows with >=1 channel. */
  std::shared_ptr<std::atomic<bool>> any_window_open() const { return any_window_open_; }

 private:
  mutable std::mutex                            mtx_;
  std::map<std::string, MetricChannelImpl::Ptr> channels_;
  std::shared_ptr<std::atomic<bool>> any_window_open_ = std::make_shared<std::atomic<bool>>(false);
};

}  // namespace mola
