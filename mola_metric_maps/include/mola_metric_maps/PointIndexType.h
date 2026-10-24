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
 * @file   PointIndexType.h
 * @brief  Spatial index of mola::IncrementalPointCloud
 * @author Jose Luis Blanco Claraco
 * @date   Oct 24, 2026
 */
#pragma once

#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

namespace mola
{
/** The data structure answering the nearest-neighbor queries of
 *  `mola::IncrementalPointCloud`. Both are exact: they return the same
 *  neighbor distances, and may only differ in which of several exactly
 *  equidistant points they pick.
 */
enum class PointIndexType : uint8_t
{
  /** Incremental, self-balancing k-d tree (nanoflann). Points are removed
   *  lazily (tombstones) and the tree is kept balanced by partial rebuilds,
   *  optionally on a background thread. */
  KDTree = 0,
  /** Hashed grid of voxels, each one keeping a contiguous copy of its points.
   *  Queries visit the voxels around the query in rings of growing size,
   *  pruning those farther than the current result. Insertion and removal are
   *  O(1) per point, with no rebuilds at all. */
  VoxelHash
};
}  // namespace mola

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mola, mola::PointIndexType)
MRPT_FILL_ENUM(PointIndexType::KDTree);
MRPT_FILL_ENUM(PointIndexType::VoxelHash);
MRPT_ENUM_TYPE_END()
