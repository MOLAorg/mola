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
 * @file   test_parallelism.cpp
 * @brief  Unit tests for mola::physical_core_count() and max_parallel_threads()
 * @author Jose Luis Blanco Claraco
 * @date   Oct 30, 2026
 */

#include <mola_kernel/utils/parallelism.h>
#include <mrpt/core/exceptions.h>

#include <cstdlib>
#include <iostream>
#include <thread>

namespace
{
void test_physical_core_count()
{
  const unsigned int n  = mola::physical_core_count();
  const unsigned int hw = std::thread::hardware_concurrency();
  ASSERT_GE_(n, 1U);
  if (hw > 0)
  {
    ASSERT_LE_(n, hw);
  }
}

void test_env_var()
{
  ::setenv("MOLA_MAX_THREADS", "3", 1);
  ASSERT_EQUAL_(mola::max_parallel_threads(), 3U);

  // Invalid values fall back to the physical cores:
  for (const char* bad : {"0", "-2", "abc", "4x", ""})
  {
    ::setenv("MOLA_MAX_THREADS", bad, 1);
    ASSERT_EQUAL_(mola::max_parallel_threads(), mola::physical_core_count());
  }
  ::unsetenv("MOLA_MAX_THREADS");
  ASSERT_EQUAL_(mola::max_parallel_threads(), mola::physical_core_count());
}
}  // namespace

int main()
{
  try
  {
    test_physical_core_count();
    test_env_var();
    std::cout << "All parallelism tests passed (physical cores: " << mola::physical_core_count()
              << ").\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Test failed: " << e.what() << "\n";
    return 1;
  }
}
