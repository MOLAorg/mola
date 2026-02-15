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
 * @file   FastAllocator.h
 * @brief  Provides alternative STL allocators, if available, per CMake config
 * @author Jose Luis Blanco Claraco
 * @date   Jan 07, 2019
 */
#pragma once

#include <map>
#include <memory>  // std::allocator
#include <set>
#if MOLA_KERNEL_HAS_TBB
#include <tbb/tbb_allocator.h>
#endif

namespace mola
{
#if MOLA_KERNEL_HAS_TBB
template <typename T>
using FastAllocator = tbb::tbb_allocator<T>;
#else
template <typename T>
using FastAllocator = std::allocator<T>;
#endif

template <class T, class Compare = std::less<T>>
using fast_set = std::set<T, Compare, FastAllocator<T>>;

template <class Key, class T, class Compare = std::less<Key>>
using fast_map = std::map<Key, T, Compare, FastAllocator<std::pair<const Key, T>>>;

}  // namespace mola
