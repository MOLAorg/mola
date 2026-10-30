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
 * @file   parallelism.h
 * @brief  Number of threads for the parallel algorithms of a MOLA process
 * @author Jose Luis Blanco Claraco
 * @date   Oct 30, 2026
 */
#pragma once

namespace mola
{
/** Number of physical CPU cores this process may run on (its CPU affinity
 *  mask), counting the hardware threads of one core (SMT, Hyper-Threading)
 *  once. Where the CPU topology is unknown, the number of CPUs in the affinity
 *  mask, or std::thread::hardware_concurrency() without one. Always >= 1.
 */
unsigned int physical_core_count();

/** Maximum number of threads that MOLA executables let the parallel
 *  algorithms (TBB) use: the environment variable `MOLA_MAX_THREADS` if it
 *  holds a positive integer, otherwise physical_core_count().
 *
 * Extra hardware threads of the same cores do not make LiDAR odometry faster,
 * but the idle TBB workers spin while waiting for the next (short) parallel
 * loop, so each one burns CPU that other processes need.
 */
unsigned int max_parallel_threads();

}  // namespace mola
