/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FastAllocator.h
 * @brief  Provides alternative STL allocators, if available, per CMake config
 * @author Jose Luis Blanco Claraco
 * @date   Jan 07, 2019
 */
#pragma once

#include <memory>  // std::allocator
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

}  // namespace mola
