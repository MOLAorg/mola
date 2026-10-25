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
 * @file   VoxelHashIndex.h
 * @brief  Hashed voxel grid implementation of mola::internal::IncrementalKDTree
 * @author Jose Luis Blanco Claraco
 * @date   Oct 24, 2026
 */
#pragma once

#include <memory>

#include "IncrementalKDTree.h"

namespace mola::internal
{
/** Creates the hashed voxel grid implementation of IncrementalKDTree, with
 *  voxels of side `voxelSize` (any positive value: queries are exact for all
 *  of them, it only changes their cost). See VoxelHashIndex.cpp.
 */
[[nodiscard]] std::unique_ptr<IncrementalKDTree> CreateVoxelHashIndex(float voxelSize);
}  // namespace mola::internal
