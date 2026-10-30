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
 * @file   LogHub.cpp
 * @brief  Process-wide fan-out of the log messages of all MOLA modules
 * @author Jose Luis Blanco Claraco
 * @date   Oct 30, 2026
 */

#include <mola_kernel/LogHub.h>
#include <mola_kernel/interfaces/ExecutableBase.h>

#include <algorithm>
#include <optional>

using namespace mola;

struct LogHub::Sink::Entry
{
  Entry(mrpt::system::VerbosityLevel l, SinkFunction f) : minLevel(l), fn(std::move(f)) {}

  std::atomic<mrpt::system::VerbosityLevel> minLevel;
  std::atomic<bool>                         enabled{true};
  const SinkFunction                        fn;
};

namespace
{
// Set while a thread runs the sinks, so messages logged from within a sink
// are not fed back into the sinks.
thread_local bool tl_in_dispatch = false;

// A no-op callback that marks a logger as hooked. Being a plain function
// pointer, unlike the forwarding lambda, logDeregisterCallback() can find it,
// and since it lives inside the logger, a new logger later created at the
// address of a destroyed one is not mistaken for an already hooked one.
void hooked_marker(
    std::string_view /*msg*/, mrpt::system::VerbosityLevel /*level*/,
    std::string_view /*loggerName*/, mrpt::Clock::time_point /*timestamp*/)
{
}
}  // namespace

LogHub& LogHub::Instance()
{
  // Intentionally leaked: loggers holding a forwarding callback may still log
  // during static destruction.
  static auto* hub = new LogHub();
  return *hub;
}

LogHub::Sink::~Sink()
{
  entry_->enabled = false;
  LogHub::Instance().remove_sink(entry_.get());
}

void LogHub::Sink::set_min_level(mrpt::system::VerbosityLevel level) { entry_->minLevel = level; }

mrpt::system::VerbosityLevel LogHub::Sink::min_level() const { return entry_->minLevel; }

std::shared_ptr<LogHub::Sink> LogHub::add_sink(
    mrpt::system::VerbosityLevel minLevel, SinkFunction f)
{
  auto entry = std::make_shared<Sink::Entry>(minLevel, std::move(f));

  std::lock_guard<std::mutex> lck(sinksMtx_);
  auto newList = sinks_ ? std::make_shared<std::vector<std::shared_ptr<Sink::Entry>>>(*sinks_)
                        : std::make_shared<std::vector<std::shared_ptr<Sink::Entry>>>();
  newList->push_back(entry);
  sinks_     = std::move(newList);
  sinkCount_ = sinks_->size();

  // Not make_shared(): the constructor is private.
  return std::shared_ptr<Sink>(new Sink(std::move(entry)));
}

void LogHub::remove_sink(const Sink::Entry* e)
{
  std::lock_guard<std::mutex> lck(sinksMtx_);
  if (!sinks_)
  {
    return;
  }
  auto newList = std::make_shared<std::vector<std::shared_ptr<Sink::Entry>>>();
  for (const auto& s : *sinks_)
  {
    if (s.get() != e)
    {
      newList->push_back(s);
    }
  }
  sinks_     = std::move(newList);
  sinkCount_ = sinks_->size();
}

std::shared_ptr<const std::vector<std::shared_ptr<LogHub::Sink::Entry>>> LogHub::sinks() const
{
  std::lock_guard<std::mutex> lck(sinksMtx_);
  return sinks_;
}

void LogHub::dispatch(const LogRecord& r) const
{
  if (tl_in_dispatch || sinkCount_ == 0)
  {
    return;
  }

  struct Guard
  {
    Guard() { tl_in_dispatch = true; }
    ~Guard() { tl_in_dispatch = false; }
    Guard(const Guard&)            = delete;
    Guard& operator=(const Guard&) = delete;
    Guard(Guard&&)                 = delete;
    Guard& operator=(Guard&&)      = delete;
  };
  const Guard guard;

  const auto list = sinks();
  if (!list)
  {
    return;
  }
  for (const auto& s : *list)
  {
    if (!s->enabled || r.level < s->minLevel)
    {
      continue;
    }
    // A logger callback must never throw: it may run inside a destructor.
    try
    {
      s->fn(r);
    }
    catch (...)
    {
    }
  }
}

void LogHub::attach_logger(mrpt::system::COutputLogger& logger, const std::string& source)
{
  {
    std::lock_guard<std::mutex> lck(attachMtx_);

    const bool alreadyHooked = logger.logDeregisterCallback(&hooked_marker);
    logger.logRegisterCallback(&hooked_marker);

    if (!alreadyHooked)
    {
      // MRPT copies the callbacks for each message: a shared_ptr is cheaper to
      // copy than a (usually not short) string.
      logger.logRegisterCallback(
          [src = std::make_shared<const std::string>(source)](
              std::string_view msg, mrpt::system::VerbosityLevel       level,
              std::string_view /*loggerName*/, mrpt::Clock::time_point timestamp)
          {
            LogRecord r;
            r.source    = *src;
            r.text      = msg;
            r.level     = level;
            r.timestamp = timestamp;
            LogHub::Instance().dispatch(r);
          });
    }
  }

  // Callback threshold: the lowest level wanted by any sink, but never above
  // what the logger prints, which is what it gets by default.
  std::optional<mrpt::system::VerbosityLevel> sinksLevel;
  if (const auto list = sinks(); list)
  {
    for (const auto& s : *list)
    {
      const mrpt::system::VerbosityLevel l = s->minLevel;
      if (!sinksLevel || l < *sinksLevel)
      {
        sinksLevel = l;
      }
    }
  }
  const auto ownLevel = logger.getMinLoggingLevel();
  logger.setVerbosityLevelForCallbacks(sinksLevel ? std::min(*sinksLevel, ownLevel) : ownLevel);
}

void LogHub::attach_modules(ExecutableBase& self)
{
  const auto attachWithChildren = [this](ExecutableBase& module)
  {
    const std::string name = module.getModuleInstanceName();
    attach_logger(module, name);

    for (const auto& [childName, childLogger] : module.child_loggers())
    {
      if (childLogger != nullptr)
      {
        std::string childSource = name;
        childSource += "/";
        childSource += childName;
        attach_logger(*childLogger, childSource);
      }
    }
  };

  // Directly, in case it is not reachable via its own name server:
  attachWithChildren(self);

  for (const auto& module : self.findService<ExecutableBase>())
  {
    if (module.get() != &self)
    {
      attachWithChildren(*module);
    }
  }
}
