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
 * @file   test_log_hub.cpp
 * @brief  Unit tests for mola::LogHub
 * @author Jose Luis Blanco Claraco
 * @date   Oct 30, 2026
 */

#include <mola_kernel/LogHub.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mrpt/core/exceptions.h>

#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>

namespace mola
{
class MockLogModule : public ExecutableBase
{
  DEFINE_MRPT_OBJECT(MockLogModule, mola)
 public:
  MockLogModule() : child_("child") { child_.logging_enable_console_output = false; }

  void initialize(const mrpt::containers::yaml&) override {}
  void spinOnce() override {}

  std::vector<std::pair<ChildLoggerName, mrpt::system::COutputLogger*>> child_loggers()
      const override
  {
    return {{"engine", &child_}};
  }

  mutable mrpt::system::COutputLogger child_;
};
}  // namespace mola
IMPLEMENTS_MRPT_OBJECT(MockLogModule, mola::ExecutableBase, mola)

namespace
{
using mrpt::system::LVL_DEBUG;
using mrpt::system::LVL_ERROR;
using mrpt::system::LVL_INFO;
using mrpt::system::LVL_WARN;

/** Collects what a sink receives. Shared, as sinks must co-own their state. */
struct Collected
{
  std::mutex               mtx;
  std::vector<std::string> lines;  // "source|level|text"

  void add(const mola::LogRecord& r)
  {
    std::lock_guard<std::mutex> lck(mtx);
    lines.push_back(
        std::string(r.source) + "|" + std::to_string(static_cast<int>(r.level)) + "|" +
        std::string(r.text));
  }
  size_t size()
  {
    std::lock_guard<std::mutex> lck(mtx);
    return lines.size();
  }
};

std::shared_ptr<mola::LogHub::Sink> collect_into(
    const std::shared_ptr<Collected>& c, mrpt::system::VerbosityLevel level)
{
  return mola::LogHub::Instance().add_sink(level, [c](const mola::LogRecord& r) { c->add(r); });
}

mrpt::system::COutputLogger quiet_logger(const std::string& name)
{
  mrpt::system::COutputLogger lg(name);
  lg.logging_enable_console_output = false;
  return lg;
}

void test_delivery_and_levels()
{
  auto& hub = mola::LogHub::Instance();
  auto  lg  = quiet_logger("a");

  auto cInfo  = std::make_shared<Collected>();
  auto cDebug = std::make_shared<Collected>();
  auto sInfo  = collect_into(cInfo, LVL_INFO);
  auto sDebug = collect_into(cDebug, LVL_DEBUG);

  hub.attach_logger(lg, "srcA");
  hub.attach_logger(lg, "srcA");  // must not hook twice

  // The logger prints INFO+, but a sink wants DEBUG:
  ASSERT_EQUAL_(lg.getMinLoggingLevelForCallbacks(), LVL_DEBUG);

  lg.logStr(LVL_DEBUG, "dbg");
  lg.logStr(LVL_WARN, "wrn");

  ASSERT_EQUAL_(cInfo->size(), 1U);
  ASSERT_EQUAL_(cInfo->lines.at(0), std::string("srcA|2|wrn"));
  ASSERT_EQUAL_(cDebug->size(), 2U);
  ASSERT_EQUAL_(cDebug->lines.at(0), std::string("srcA|0|dbg"));

  // Without the DEBUG sink, the threshold goes back to the logger's own level:
  sDebug.reset();
  hub.attach_logger(lg, "srcA");
  ASSERT_EQUAL_(lg.getMinLoggingLevelForCallbacks(), LVL_INFO);

  // ...and it never goes above it, so other callbacks keep working:
  sInfo->set_min_level(LVL_ERROR);
  hub.attach_logger(lg, "srcA");
  ASSERT_EQUAL_(lg.getMinLoggingLevelForCallbacks(), LVL_INFO);

  // INFO is below the raised sink level, so the live sink must not receive it:
  lg.logStr(LVL_INFO, "info");
  ASSERT_EQUAL_(cInfo->size(), 1U);

  // A released sink receives nothing:
  sInfo.reset();
  lg.logStr(LVL_ERROR, "err");
  ASSERT_EQUAL_(cInfo->size(), 1U);
  ASSERT_EQUAL_(cDebug->size(), 2U);
}

void test_recursion_and_exceptions()
{
  auto& hub = mola::LogHub::Instance();
  auto  lg  = quiet_logger("b");

  std::atomic<int> calls{0};
  auto             sLoop = hub.add_sink(
                  LVL_INFO,
                  [&lg, &calls](const mola::LogRecord&)
                  {
        calls++;
        lg.logStr(LVL_ERROR, "from within a sink");  // must not recurse
        throw std::runtime_error("must be swallowed");
      });

  auto c  = std::make_shared<Collected>();
  auto sC = collect_into(c, LVL_INFO);

  hub.attach_logger(lg, "srcB");
  lg.logStr(LVL_INFO, "hello");

  ASSERT_EQUAL_(calls.load(), 1);
  // The second sink still runs after the first one threw, and does not get
  // the message logged from within the first sink:
  ASSERT_EQUAL_(c->size(), 1U);
  ASSERT_EQUAL_(c->lines.at(0), std::string("srcB|1|hello"));
}

void test_new_logger_at_reused_address()
{
  auto& hub = mola::LogHub::Instance();
  auto  c   = std::make_shared<Collected>();
  auto  s   = collect_into(c, LVL_INFO);

  alignas(mrpt::system::COutputLogger) unsigned char buf[sizeof(mrpt::system::COutputLogger)];

  for (int i = 0; i < 2; i++)
  {
    auto* lg                          = new (buf) mrpt::system::COutputLogger("reused");
    lg->logging_enable_console_output = false;
    hub.attach_logger(*lg, "src" + std::to_string(i));
    lg->logStr(LVL_INFO, "msg");
    lg->~COutputLogger();
  }
  ASSERT_EQUAL_(c->size(), 2U);
  ASSERT_EQUAL_(c->lines.at(0), std::string("src0|1|msg"));
  ASSERT_EQUAL_(c->lines.at(1), std::string("src1|1|msg"));
}

void test_attach_unenrolled_self()
{
  mola::MockLogModule mod;  // no name server
  mod.setModuleInstanceName("lonely");
  mod.logging_enable_console_output = false;

  auto c = std::make_shared<Collected>();
  auto s = collect_into(c, LVL_INFO);

  mola::LogHub::Instance().attach_modules(mod);
  mod.logStr(LVL_INFO, "m");
  mod.child_.logStr(LVL_INFO, "c");

  ASSERT_EQUAL_(c->size(), 2U);
  ASSERT_EQUAL_(c->lines.at(0), std::string("lonely|1|m"));
  ASSERT_EQUAL_(c->lines.at(1), std::string("lonely/engine|1|c"));
}

void test_attach_modules()
{
  auto mod = std::make_shared<mola::MockLogModule>();
  mod->setModuleInstanceName("mod1");
  mod->logging_enable_console_output = false;
  mola::MinimalModuleContainer container({mod});

  auto c = std::make_shared<Collected>();
  auto s = collect_into(c, LVL_INFO);

  mola::LogHub::Instance().attach_modules(*mod);
  mola::LogHub::Instance().attach_modules(*mod);

  mod->logStr(LVL_INFO, "m");
  mod->child_.logStr(LVL_WARN, "c");

  ASSERT_EQUAL_(c->size(), 2U);
  ASSERT_EQUAL_(c->lines.at(0), std::string("mod1|1|m"));
  ASSERT_EQUAL_(c->lines.at(1), std::string("mod1/engine|2|c"));
}

// Loggers logging from several threads while they are (re)hooked, and sinks
// are added and removed. Meant to be run under ASan/TSan too.
void test_concurrency()
{
  auto& hub = mola::LogHub::Instance();

  constexpr int NUM_LOGGERS  = 4;
  constexpr int NUM_MESSAGES = 20000;

  std::vector<std::unique_ptr<mrpt::system::COutputLogger>> loggers;
  for (int i = 0; i < NUM_LOGGERS; i++)
  {
    loggers.push_back(std::make_unique<mrpt::system::COutputLogger>("l" + std::to_string(i)));
    loggers.back()->logging_enable_console_output = false;
  }

  auto counter = std::make_shared<std::atomic<size_t>>(0);
  auto sKeep   = hub.add_sink(
        LVL_INFO,
        [counter](const mola::LogRecord& r)
        {
        if (r.source != "tmp")
        {
          (*counter)++;
        }
      });

  // Hooked before logging starts, so the count below is exact:
  for (int i = 0; i < NUM_LOGGERS; i++)
  {
    hub.attach_logger(*loggers[i], "l" + std::to_string(i));
  }

  std::atomic<bool>        done{false};
  std::vector<std::thread> threads;
  for (int i = 0; i < NUM_LOGGERS; i++)
  {
    threads.emplace_back(
        [&, i]()
        {
          for (int k = 0; k < NUM_MESSAGES; k++)
          {
            loggers[i]->logStr(k % 2 == 0 ? LVL_INFO : LVL_DEBUG, "x");
          }
        });
  }

  std::thread churn(
      [&]()
      {
        while (!done)
        {
          auto tmp = std::make_shared<std::atomic<size_t>>(0);
          auto s   = hub.add_sink(LVL_DEBUG, [tmp](const mola::LogRecord&) { (*tmp)++; });
          for (int i = 0; i < NUM_LOGGERS; i++)
          {
            hub.attach_logger(*loggers[i], "l" + std::to_string(i));
          }
          // A short-lived logger, hooked and destroyed while others log:
          auto lg = quiet_logger("tmp");
          hub.attach_logger(lg, "tmp");
          lg.logStr(LVL_INFO, "y");
        }
      });

  for (auto& t : threads)
  {
    t.join();
  }
  done = true;
  churn.join();

  // Exactly the INFO messages, none lost or duplicated by the re-hooking:
  ASSERT_EQUAL_(counter->load(), static_cast<size_t>(NUM_LOGGERS * NUM_MESSAGES / 2));
}
}  // namespace

int main()
{
  try
  {
    test_delivery_and_levels();
    test_recursion_and_exceptions();
    test_new_logger_at_reused_address();
    test_attach_unenrolled_self();
    test_attach_modules();
    test_concurrency();
    std::cout << "All LogHub tests passed.\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Test failed: " << e.what() << "\n";
    return 1;
  }
}
