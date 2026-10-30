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
 * @file   LogHub.h
 * @brief  Process-wide fan-out of the log messages of all MOLA modules
 * @author Jose Luis Blanco Claraco
 * @date   Oct 30, 2026
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/system/COutputLogger.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace mola
{
class ExecutableBase;

/** One log message, as delivered to LogHub sinks. The views are only valid
 *  during the sink call. \ingroup mola_kernel_grp */
struct LogRecord
{
  /** Module instance name, or "<module>/<child>" for child loggers. */
  std::string_view             source;
  std::string_view             text;  //!< As passed to the logger
  mrpt::system::VerbosityLevel level = mrpt::system::LVL_INFO;
  mrpt::Clock::time_point      timestamp;
};

/** Process-wide fan-out of the log messages of all MOLA modules to any number
 *  of consumers ("sinks"), e.g. the GUI console window or ROS 2 `/rosout`.
 *
 * Every hooked mrpt::system::COutputLogger gets exactly one forwarding
 * callback, which only refers to this (never destroyed) singleton, so loggers
 * may outlive any consumer, and consumers may come and go at any time.
 *
 * Usage from a consumer module:
 * - add_sink() once, keeping the returned handle alive while the sink is
 *   wanted. Releasing it unregisters the sink.
 * - attach_modules() periodically (e.g. every few seconds from spinOnce()),
 *   to hook modules (and their ExecutableBase::child_loggers()) created
 *   later, and to apply the current sink levels.
 *
 * Sink functions are called synchronously from whichever thread logs, so they
 * must be fast and must not block (e.g. just push into a queue). They may be
 * invoked once more right after their handle is released, so they must only
 * capture state they co-own (e.g. a shared_ptr). Messages logged from within a
 * sink function are not forwarded (no recursion), and exceptions thrown by a
 * sink are ignored.
 *
 * The callback verbosity threshold of each hooked logger is set to the lowest
 * level among the live sinks, but never above the logger's own console level,
 * so callbacks registered by modules themselves keep receiving what they did.
 *
 * \note Thread-safe hooking of loggers that are concurrently logging requires
 *       MRPT >= 3.3.1.
 * \ingroup mola_kernel_grp
 */
class LogHub
{
 public:
  /** The process-wide instance. It is never destroyed. */
  static LogHub& Instance();

  using SinkFunction = std::function<void(const LogRecord&)>;

  /** Handle to a registered sink: the sink is removed when it is destroyed.
   *  \sa add_sink() */
  class Sink
  {
   public:
    ~Sink();

    Sink(const Sink&)            = delete;
    Sink& operator=(const Sink&) = delete;
    Sink(Sink&&)                 = delete;
    Sink& operator=(Sink&&)      = delete;

    /** Changes the minimum level of messages sent to this sink. It reaches
     *  the module loggers on the next attach_modules() call. */
    void set_min_level(mrpt::system::VerbosityLevel level);

    [[nodiscard]] mrpt::system::VerbosityLevel min_level() const;

    struct Entry;  //!< Internal

   private:
    friend class LogHub;
    explicit Sink(std::shared_ptr<Entry> e) : entry_(std::move(e)) {}
    std::shared_ptr<Entry> entry_;
  };

  /** Registers a new sink, receiving messages with level >= minLevel.
   *  Messages logged before this call are not delivered. */
  [[nodiscard]] std::shared_ptr<Sink> add_sink(
      mrpt::system::VerbosityLevel minLevel, SinkFunction f);

  /** Hooks all modules reachable from \a self via its name server (including
   *  \a self), plus their ExecutableBase::child_loggers(), and updates their
   *  callback verbosity thresholds. Cheap for already-hooked loggers. */
  void attach_modules(const ExecutableBase& self);

  /** Hooks a single logger, with \a source as the LogRecord::source of its
   *  messages, and updates its callback verbosity threshold. Hooking the same
   *  logger again is a no-op, apart from updating the threshold. */
  void attach_logger(mrpt::system::COutputLogger& logger, const std::string& source);

 private:
  LogHub() = default;

  /** Guards the sink list, which is replaced (never modified) on changes so
   *  that dispatch() only holds the lock to copy a shared_ptr. */
  mutable std::mutex                                               sinksMtx_;
  std::shared_ptr<const std::vector<std::shared_ptr<Sink::Entry>>> sinks_;
  std::atomic<size_t>                                              sinkCount_{0};

  /** Serializes attach_logger(), so two consumers cannot both conclude that
   *  the same logger is not hooked yet. */
  std::mutex attachMtx_;

  void remove_sink(const Sink::Entry* e);
  void dispatch(const LogRecord& r) const;

  std::shared_ptr<const std::vector<std::shared_ptr<Sink::Entry>>> sinks() const;
};

}  // namespace mola
