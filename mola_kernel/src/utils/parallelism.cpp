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
 * @file   parallelism.cpp
 * @brief  Number of threads for the parallel algorithms of a MOLA process
 * @author Jose Luis Blanco Claraco
 * @date   Oct 30, 2026
 */

#include <mola_kernel/utils/parallelism.h>

#include <cstdlib>
#include <fstream>
#include <set>
#include <string>
#include <thread>
#include <utility>

#if defined(__linux__)
#include <sched.h>
#endif

namespace
{
unsigned int fallback_count()
{
  const unsigned int n = std::thread::hardware_concurrency();
  return n > 0 ? n : 1;
}

#if defined(__linux__)
bool read_int(const std::string& file, int& value)
{
  std::ifstream f(file);
  return static_cast<bool>(f >> value);
}
#endif
}  // namespace

unsigned int mola::physical_core_count()
{
#if defined(__linux__)
  cpu_set_t mask;
  CPU_ZERO(&mask);
  if (sched_getaffinity(0, sizeof(mask), &mask) != 0)
  {
    return fallback_count();
  }
  // One entry per (package, core) pair among the CPUs this process may use:
  std::set<std::pair<int, int>> cores;
  for (int cpu = 0; cpu < CPU_SETSIZE; cpu++)
  {
    if (!CPU_ISSET(cpu, &mask))
    {
      continue;
    }
    const std::string dir     = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/";
    int               package = 0;
    int               core    = 0;
    if (!read_int(dir + "physical_package_id", package) || !read_int(dir + "core_id", core))
    {
      return fallback_count();
    }
    cores.emplace(package, core);
  }
  return cores.empty() ? fallback_count() : static_cast<unsigned int>(cores.size());
#else
  return fallback_count();
#endif
}

unsigned int mola::max_parallel_threads()
{
  if (const char* env = std::getenv("MOLA_MAX_THREADS"); env != nullptr)
  {
    char*      end = nullptr;
    const long n   = std::strtol(env, &end, 10);
    if (end != env && *end == '\0' && n > 0)
    {
      return static_cast<unsigned int>(n);
    }
  }
  return physical_core_count();
}
