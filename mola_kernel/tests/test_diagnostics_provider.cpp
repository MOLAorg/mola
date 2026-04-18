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
 * @file   test_diagnostics_provider.cpp
 * @brief  Unit tests for DiagnosticsProvider interface
 * @author Jose Luis Blanco Claraco
 * @date   Apr 19, 2026
 */

#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/DiagnosticsProvider.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mrpt/core/exceptions.h>

#include <cstdint>
#include <iostream>
#include <memory>

namespace mola
{
class MockDiagModule : public ExecutableBase, public DiagnosticsProvider
{
  DEFINE_MRPT_OBJECT(MockDiagModule, mola)
 public:
  MockDiagModule()           = default;
  ~MockDiagModule() override = default;

  void initialize(const mrpt::containers::yaml&) override {}
  void spinOnce() override {}

  void getDiagnostics(std::vector<DiagnosticStatusMsg>& status) override
  {
    DiagnosticStatusMsg s;
    s.level   = DiagnosticLevel::OK;
    s.name    = "MockDiag";
    s.message = "nominal";
    s.values.push_back({"foo", "42"});
    status.push_back(s);
  }
};
}  // namespace mola
IMPLEMENTS_MRPT_OBJECT(MockDiagModule, mola::ExecutableBase, mola)

namespace
{
void test_enum_values_match_rep107()
{
  ASSERT_EQUAL_(static_cast<uint8_t>(mola::DiagnosticLevel::OK), 0u);
  ASSERT_EQUAL_(static_cast<uint8_t>(mola::DiagnosticLevel::WARN), 1u);
  ASSERT_EQUAL_(static_cast<uint8_t>(mola::DiagnosticLevel::ERROR), 2u);
  ASSERT_EQUAL_(static_cast<uint8_t>(mola::DiagnosticLevel::STALE), 3u);
}

void test_interface_discovery()
{
  auto                         mod = std::make_shared<mola::MockDiagModule>();
  mola::MinimalModuleContainer container({mod});

  const auto found = mod->findService<mola::DiagnosticsProvider>();
  ASSERT_EQUAL_(found.size(), 1u);

  auto prov = std::dynamic_pointer_cast<mola::DiagnosticsProvider>(found.front());
  ASSERT_(prov);

  std::vector<mola::DiagnosticStatusMsg> out;
  prov->getDiagnostics(out);
  ASSERT_EQUAL_(out.size(), 1u);
  ASSERT_EQUAL_(out[0].name, std::string("MockDiag"));
  ASSERT_EQUAL_(out[0].values.size(), 1u);
  ASSERT_EQUAL_(out[0].values[0].key, std::string("foo"));
}

struct EmptyMod : mola::DiagnosticsProvider
{
  void getDiagnostics(std::vector<mola::DiagnosticStatusMsg>&) override {}
};

void test_empty_diagnostics()
{
  EmptyMod                               m;
  std::vector<mola::DiagnosticStatusMsg> out;
  m.getDiagnostics(out);
  ASSERT_(out.empty());
}
}  // namespace

int main()
{
  try
  {
    test_enum_values_match_rep107();
    test_empty_diagnostics();
    test_interface_discovery();
    std::cout << "All DiagnosticsProvider tests passed.\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Test failed: " << e.what() << "\n";
    return 1;
  }
}
