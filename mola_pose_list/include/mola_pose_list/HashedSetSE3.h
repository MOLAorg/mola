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
 * @file   HashedSetSE3.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Apr 16, 2024
 */
#pragma once

#include <mola_pose_list/index_se3_t.h>
#include <mrpt/core/round.h>
#include <mrpt/math/TPose3D.h>

#include <unordered_map>

namespace mola
{
/** HashedSetSE3: a sparse hashed lattice of SE(3) poses
 */
class HashedSetSE3
{
 public:
  using global_index3d_t = index_se3_t<int32_t>;

  /** @name Indices and coordinates
   *  @{ */

  inline global_index3d_t coordToGlobalIdx(const mrpt::math::TPose3D& p) const
  {
    return global_index3d_t(
        static_cast<int32_t>(p.x * voxel_xyz_size_inv_),  //
        static_cast<int32_t>(p.y * voxel_xyz_size_inv_),  //
        static_cast<int32_t>(p.z * voxel_xyz_size_inv_),  //
        static_cast<int32_t>(p.yaw * voxel_yaw_size_inv_),  //
        static_cast<int32_t>(p.pitch * voxel_pitch_size_inv_),  //
        static_cast<int32_t>(p.roll * voxel_roll_size_inv_)  //

    );
  }

  /// returns the coordinate of the voxel "bottom" corner
  inline mrpt::math::TPose3D globalIdxToCoord(const global_index3d_t idx) const
  {
    return {
        idx.cx * voxel_xyz_size_,  //
        idx.cy * voxel_xyz_size_,  //
        idx.cz * voxel_xyz_size_,  //
        idx.cyaw * voxel_yaw_size_,  //
        idx.cpitch * voxel_pitch_size_,  //
        idx.croll * voxel_roll_size_  //
    };
  }

  /** @} */

  /** @name Basic API for construction and main parameters
   *  @{ */

  /**
   * @brief Constructor / default ctor
   * @param voxel_size Voxel size [meters]
   */
  HashedSetSE3(
      double voxel_xyz_size = 1.0, double voxel_yaw_size = mrpt::DEG2RAD(10.0),
      double voxel_pitch_size = mrpt::DEG2RAD(10.0), double voxel_roll_size = mrpt::DEG2RAD(10.0));

  ~HashedSetSE3();

  /** Reset the main voxel parameters, and *clears* all current map contents
   */
  void setVoxelProperties(
      double voxel_xyz_size, double voxel_yaw_size, double voxel_pitch_size,
      double voxel_roll_size);

  /** @} */

  /** @name Data structures
   *  @{ */

  using pose_vector_t = std::vector<mrpt::math::TPose3D>;

  struct VoxelData
  {
   public:
    VoxelData() = default;

    auto poses() const { return poses_; }
    void insertPose(const mrpt::math::TPose3D& p) { poses_.push_back(p); }

   private:
    pose_vector_t poses_;
  };

  using grids_map_t = std::unordered_map<global_index3d_t, VoxelData, index_se3_t_hash<int32_t>>;

  /** @} */

  /** @name Data access API
   *  @{ */
  void clear();

  bool empty() const;

  /** returns the voxeldata by global index coordinates, creating it or not if
   * not found depending on createIfNew.
   * Returns nullptr if not found and createIfNew is false
   *
   * Function defined in the header file so compilers can optimize
   * for literals "createIfNew"
   */
  inline VoxelData* voxelByGlobalIdxs(const global_index3d_t& idx, bool createIfNew)
  {
    // 1) Insert into decimation voxel map:
    VoxelData* voxel = nullptr;

    auto it = voxels_.find(idx);
    if (it == voxels_.end())
    {
      if (!createIfNew)
        return nullptr;
      else
        voxel = &voxels_[idx];  // Create it
    }
    else
    {
      // Use the found grid
      voxel = const_cast<VoxelData*>(&it->second);
    }
    return voxel;
  }

  /// \overload (const version)
  const VoxelData* voxelByGlobalIdxs(const global_index3d_t& idx) const
  {  // reuse the non-const method:
    return const_cast<HashedSetSE3*>(this)->voxelByGlobalIdxs(idx, false);
  }

  /** Get a voxeldata by (x,y,z) coordinates, **creating** the voxel if
   * needed. */
  VoxelData* voxelByCoords(const mrpt::math::TPose3D& pt, bool createIfNew)
  {
    return voxelByGlobalIdxs(coordToGlobalIdx(pt), createIfNew);
  }

  /// \overload const version. Returns nullptr if voxel does not exist
  const VoxelData* voxelByCoords(const mrpt::math::TPose3D& pt) const
  {
    return const_cast<HashedSetSE3*>(this)->voxelByCoords(pt, false);
  }

  /** Insert one pose into the lattice */
  void insertPose(const mrpt::math::TPose3D& pt);

  const grids_map_t& voxels() const { return voxels_; }

  void visitAllPoses(const std::function<void(const mrpt::math::TPose3D&)>& f) const;

  void visitAllVoxels(
      const std::function<void(const global_index3d_t&, const VoxelData&)>& f) const;

  /** Save to a text file. Each line contains "X Y Z YAW PITCH ROLL".
   *  Returns false if any error ocurred, true elsewere.
   */
  bool saveToTextFile(const std::string& file) const;

  /** @} */

 private:
  double voxel_xyz_size_   = 1.00;
  double voxel_yaw_size_   = mrpt::DEG2RAD(10.0);
  double voxel_pitch_size_ = mrpt::DEG2RAD(10.0);
  double voxel_roll_size_  = mrpt::DEG2RAD(10.0);

  // Calculated from the above, in setVoxelProperties()
  double voxel_xyz_size_inv_   = 1.0 / voxel_xyz_size_;
  double voxel_yaw_size_inv_   = 1.0 / voxel_yaw_size_;
  double voxel_pitch_size_inv_ = 1.0 / voxel_pitch_size_;
  double voxel_roll_size_inv_  = 1.0 / voxel_roll_size_;

  /** Voxel map as a set of fixed-size grids */
  grids_map_t voxels_;
};

}  // namespace mola
