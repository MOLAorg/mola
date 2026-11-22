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
 * @file   test_executable_base_diagnostics.cpp
 * @brief  Unit tests for the ExecutableBase diagnostics output queue
 * @author Jose Luis Blanco Claraco
 * @date   Nov 22, 2026
 */

#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mrpt/core/exceptions.h>

#include <iostream>
#include <memory>
#include <string>

namespace mola
{
class MockQueueModule : public ExecutableBase
{
  DEFINE_MRPT_OBJECT(MockQueueModule, mola)
 public:
  MockQueueModule()           = default;
  ~MockQueueModule() override = default;

  void initialize(const mrpt::containers::yaml&) override {}
  void spinOnce() override {}

  void publish(const std::string& label)
  {
    DiagnosticsOutput d;
    d.timestamp = mrpt::Clock::now();
    d.label     = label;
    d.value     = true;
    module_publish_diagnostics(d);
  }

  [[nodiscard]] bool is_time() const { return module_is_time_to_publish_diagnostics(); }
};
}  // namespace mola
IMPLEMENTS_MRPT_OBJECT(MockQueueModule, mola::ExecutableBase, mola)

namespace
{
void test_move_out()
{
  mola::MockQueueModule m;
  ASSERT_(m.module_move_out_diagnostics_messages().empty());

  m.publish("a");
  m.publish("b");

  const auto msgs = m.module_move_out_diagnostics_messages();
  ASSERT_EQUAL_(msgs.size(), 2u);
  ASSERT_EQUAL_(msgs[0].label, std::string("a"));
  ASSERT_EQUAL_(msgs[1].label, std::string("b"));
  ASSERT_(std::any_cast<bool>(msgs[1].value));

  ASSERT_(m.module_move_out_diagnostics_messages().empty());
}

void test_queue_cap_drops_oldest()
{
  constexpr std::size_t CAP   = mola::ExecutableBase::MAX_QUEUED_DIAGNOSTICS;
  constexpr std::size_t EXTRA = 10;

  mola::MockQueueModule m;
  for (std::size_t i = 0; i < CAP + EXTRA; i++)
  {
    m.publish(std::to_string(i));
  }

  const auto msgs = m.module_move_out_diagnostics_messages();
  ASSERT_EQUAL_(msgs.size(), CAP);
  ASSERT_EQUAL_(msgs.front().label, std::to_string(EXTRA));
  ASSERT_EQUAL_(msgs.back().label, std::to_string(CAP + EXTRA - 1));
}

void test_is_time_to_publish()
{
  mola::MockQueueModule m;
  // Never published yet:
  ASSERT_(m.is_time());

  // Just published, the default period is 1 s:
  m.publish("x");
  ASSERT_(!m.is_time());
}
}  // namespace

int main()
{
  try
  {
    test_move_out();
    test_queue_cap_drops_oldest();
    test_is_time_to_publish();
    std::cout << "All ExecutableBase diagnostics tests passed.\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Test failed: " << e.what() << "\n";
    return 1;
  }
}
