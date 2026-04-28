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
 * @file   SearchablePoseList.h
 * @brief  Data structure to search for nearby SE(3) poses
 * @author Jose Luis Blanco Claraco
 * @date   Mar 5, 2024
 */
#pragma once

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <vector>

namespace mola
{
/** Data structure to search for nearby SE(3) poses.
 *
 *  It uses a KD-tree for the search.
 *
 *  Optionally, each inserted pose can be tagged with an external ID
 *  (e.g. a keyframe ID) so that callers can later update its stored
 *  pose in place via setPoseById(). This is used by the online
 *  gravity-rebake feature to keep distance-checkers in sync with
 *  per-KF pose corrections without rebuilding from scratch.
 *
 * \ingroup mola_pose_list_grp
 */
class SearchablePoseList
{
 public:
  using KFID = uint64_t;

  SearchablePoseList() = default;

  SearchablePoseList(bool measure_from_last_kf_only) : from_last_only_(measure_from_last_kf_only) {}

  bool empty() const
  {
    if (from_last_only_)
    {
      return !has_last_kf_;
    }

    return kf_poses_.empty();
  }

  size_t size() const { return from_last_only_ ? (has_last_kf_ ? 1 : 0) : kf_poses_.size(); }

  void insert(const mrpt::poses::CPose3D& p)
  {
    if (from_last_only_)
    {
      last_kf_     = p;
      has_last_kf_ = true;
    }
    else
    {
      kf_points_.insertPoint(p.translation());
      kf_poses_.push_back(p);
      kf_ids_.push_back(std::nullopt);
    }
  }

  /** Same as insert(p), but tags the stored entry with `id` so that the
   *  pose can later be updated in place via setPoseById(). No-op in
   *  `from_last_only_` mode (the single tracked pose has no id).
   */
  void insert(const mrpt::poses::CPose3D& p, KFID id)
  {
    if (from_last_only_)
    {
      last_kf_     = p;
      has_last_kf_ = true;
      return;
    }
    ASSERTMSG_(
        id_to_idx_.find(id) == id_to_idx_.end(),
        "SearchablePoseList::insert: KFID already present");
    const size_t idx = kf_poses_.size();
    kf_points_.insertPoint(p.translation());
    kf_poses_.push_back(p);
    kf_ids_.push_back(id);
    id_to_idx_[id] = idx;
  }

  /** Updates the stored pose for an entry previously inserted with an id.
   *  No-op if `from_last_only_` is set or `id` is unknown.
   *  The internal KD-tree point is updated in place; subsequent NN queries
   *  reflect the new pose.
   */
  void setPoseById(KFID id, const mrpt::poses::CPose3D& new_pose);

  [[nodiscard]] std::tuple<bool /*isFirst*/, mrpt::poses::CPose3D /*distanceToClosest*/> check(
      const mrpt::poses::CPose3D& p) const;

  /** Returns the count of stored poses that are within both the given
   *  translation and rotation distance from \a p.
   *  The check is: translation(p - candidate).norm() <= maxTranslation
   *             && SO3_log(rotation(p - candidate)).norm() <= maxRotationRad
   */
  [[nodiscard]] uint32_t countNearby(
      const mrpt::poses::CPose3D& p, double maxTranslation, double maxRotationRad) const;

  void removeAllFartherThan(const mrpt::poses::CPose3D& p, double maxTranslation);

 private:
  // if from_last_only_==true
  mrpt::poses::CPose3D last_kf_     = mrpt::poses::CPose3D::Identity();
  bool                 has_last_kf_ = false;

  // if from_last_only_==false
  std::deque<mrpt::poses::CPose3D> kf_poses_;
  mrpt::maps::CSimplePointsMap     kf_points_;
  /// Optional KFID for each entry, parallel to kf_poses_.
  std::deque<std::optional<KFID>> kf_ids_;
  /// Inverse lookup: KFID -> index into kf_poses_/kf_points_/kf_ids_.
  std::map<KFID, size_t> id_to_idx_;

  bool from_last_only_ = false;
};
}  // namespace mola

/** Feature macro: SearchablePoseList exposes countNearby() and the
 *  sensor-pose-as-key plumbing used by mola_lidar_odometry.
 *  Downstream packages should guard usage with
 *  `#if defined(MOLA_POSE_LIST_HAS_KFM_POSE_PLUMBING)`.
 */
#define MOLA_POSE_LIST_HAS_KFM_POSE_PLUMBING 1

/** Feature macro: SearchablePoseList exposes the id-keyed API
 *  (`insert(pose, id)`, `setPoseById`) used by the online gravity-rebake
 *  feature to keep distance-checkers in sync with per-KF pose corrections.
 *  Downstream packages in separate repos should guard usage with
 *  `#if defined(MOLA_POSE_LIST_HAS_ID_KEYED_API)` (combined with
 *  `__has_include(<mola_pose_list/SearchablePoseList.h>)`) to remain
 *  buildable against older `mola_pose_list` checkouts.
 */
#define MOLA_POSE_LIST_HAS_ID_KEYED_API 1
