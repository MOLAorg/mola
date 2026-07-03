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
 * @file   MetricChannel.h
 * @brief  Handle for streaming timestamped scalar values to the visualizer.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <memory>

/** Feature macro: when defined, VizInterface offers register_metric() /
 *  push_metric() and the mola::MetricChannel handle type, i.e. the streaming
 *  time-series "metrics" plotting mechanism. Lets out-of-repo modules detect
 *  the feature at compile time. */
#define MOLA_KERNEL_VIZ_HAS_METRICS 1

namespace mola
{

/** Opaque handle a module holds after registering a metric channel via
 *  `VizInterface::register_metric()`.
 *
 *  Thread-safe: `push()` may be called from any thread, at any rate, without
 *  external synchronization. On backends without plotting support (e.g. the
 *  nanogui `MolaViz`) the returned handle is a live no-op whose `push()`
 *  returns immediately. On backends with plotting support, `push()` is
 *  near-free whenever no plot window is currently displaying the channel.
 *
 * \ingroup mola_kernel_interfaces_grp
 */
class MetricChannel
{
 public:
  using Ptr = std::shared_ptr<MetricChannel>;

  MetricChannel()          = default;
  virtual ~MetricChannel() = default;

  MetricChannel(const MetricChannel&)            = default;
  MetricChannel& operator=(const MetricChannel&) = default;
  MetricChannel(MetricChannel&&)                 = default;
  MetricChannel& operator=(MetricChannel&&)      = default;

  /** Appends one sample.
   *  \param t     Timestamp in seconds, using any monotonic clock consistent
   *               across calls for this channel. Using wall-clock time
   *               (`mrpt::Clock::nowDouble()`) is recommended so multiple
   *               channels can share a common x-axis.
   *  \param value Sample value.
   */
  virtual void push(double t, double value) = 0;

  /** Convenience overload: uses `mrpt::Clock::nowDouble()` as the timestamp. */
  void push(double value);
};

}  // namespace mola
