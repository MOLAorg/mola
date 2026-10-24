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
 * @file   test-hashed-set-se3.cpp
 * @brief  Unit tests for mola_pose_list / HashedSetSE3 voxel indexing
 * @author Jose Luis Blanco Claraco
 * @date   Oct 24, 2026
 */

#include <mola_pose_list/HashedSetSE3.h>
#include <mrpt/core/exceptions.h>

#include <cmath>
#include <iostream>

namespace
{
size_t countVoxels(const mola::HashedSetSE3& s)
{
  size_t n = 0;
  s.visitAllVoxels([&n](const auto&, const auto&) { n++; });
  return n;
}

// Along each of the six dimensions, values from -2.4 to 2.4 every 0.2 with unit
// voxels: each voxel, centered at an integer, holds five of them. Truncating
// toward zero made the voxel around zero twice as large, holding nine.
void test_voxels_same_size()
{
  for (size_t dim = 0; dim < 6; dim++)
  {
    mola::HashedSetSE3 s(1.0, 1.0, 1.0, 1.0);
    for (int i = -24; i <= 24; i += 2)
    {
      mrpt::math::TPose3D p(0, 0, 0, 0, 0, 0);
      p[dim] = 0.1 * i;
      s.insertPose(p);
    }

    ASSERT_EQUAL_(countVoxels(s), 5UL);
    s.visitAllVoxels([](const auto&, const auto& v) { ASSERT_EQUAL_(v.poses().size(), 5UL); });

    // Each pose lies within half a voxel of the voxel center:
    s.visitAllVoxels(
        [&](const auto& idx, const auto& v)
        {
          const auto center = s.globalIdxToCoord(idx);
          for (const auto& p : v.poses())
          {
            ASSERT_LT_(std::abs(p[dim] - center[dim]), 0.5);
          }
        });
  }
  std::cout << "[hashed_set_se3] voxels of the same size: OK\n";
}

// z, pitch and roll of a planar robot only jitter around zero: such poses at
// the same x, y, yaw must not be split across voxels.
void test_jitter_around_zero()
{
  mola::HashedSetSE3 s;
  for (const double e : {-1e-3, 1e-3})
  {
    for (const double f : {-1e-3, 1e-3})
    {
      s.insertPose({0.3, -0.2, e, 0.05, f, -f});
    }
  }
  ASSERT_EQUAL_(countVoxels(s), 1UL);
  std::cout << "[hashed_set_se3] jitter around zero: OK\n";
}

}  // namespace

int main()
{
  try
  {
    test_voxels_same_size();
    test_jitter_around_zero();
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
  return 0;
}
