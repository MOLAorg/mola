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
 * @file   HashedSetSE3.cpp
 * @brief  Point cloud stored in voxels, in a sparse hash map
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */

#include <mola_pose_list/HashedSetSE3.h>
#include <mrpt/system/os.h>

using namespace mola;

// VoxelData

// Ctor:
HashedSetSE3::HashedSetSE3(
    double voxel_xyz_size, double voxel_yaw_size, double voxel_pitch_size, double voxel_roll_size)
{
  setVoxelProperties(voxel_xyz_size, voxel_yaw_size, voxel_pitch_size, voxel_roll_size);
}

HashedSetSE3::~HashedSetSE3() = default;

void HashedSetSE3::setVoxelProperties(
    double voxel_xyz_size, double voxel_yaw_size, double voxel_pitch_size, double voxel_roll_size)
{
  voxel_xyz_size_   = voxel_xyz_size;
  voxel_yaw_size_   = voxel_yaw_size;
  voxel_pitch_size_ = voxel_pitch_size;
  voxel_roll_size_  = voxel_roll_size;

  // calculated fields:
  voxel_xyz_size_inv_   = 1.0 / voxel_xyz_size_;
  voxel_yaw_size_inv_   = 1.0 / voxel_yaw_size_;
  voxel_pitch_size_inv_ = 1.0 / voxel_pitch_size_;
  voxel_roll_size_inv_  = 1.0 / voxel_roll_size_;

  // clear all:
  this->clear();
}

void HashedSetSE3::clear()
{
  //
  voxels_.clear();
}

bool HashedSetSE3::empty() const
{
  // empty if no voxels exist:
  return voxels_.empty();
}

bool HashedSetSE3::saveToTextFile(const std::string& file) const
{
  FILE* f = mrpt::system::os::fopen(file.c_str(), "wt");
  if (!f) return false;

  const auto lambdaVisitPoints = [f](const mrpt::math::TPose3D& p)
  { mrpt::system::os::fprintf(f, "%s\n", p.asString().c_str()); };

  this->visitAllPoses(lambdaVisitPoints);

  mrpt::system::os::fclose(f);
  return true;
}

void HashedSetSE3::insertPose(const mrpt::math::TPose3D& p)
{
  auto& v = *voxelByCoords(p, true /*create if new*/);
  v.insertPose(p);
}

void HashedSetSE3::visitAllPoses(const std::function<void(const mrpt::math::TPose3D&)>& f) const
{
  for (const auto& [idx, v] : voxels_)
    for (const auto& p : v.poses())  //
      f(p);
}

void HashedSetSE3::visitAllVoxels(
    const std::function<void(const global_index3d_t&, const VoxelData&)>& f) const
{
  for (const auto& [idx, v] : voxels_) f(idx, v);
}
