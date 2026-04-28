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
 * @file   SearchablePoseList.cpp
 * @brief  Data structure to search for nearby SE(3) poses
 * @author Jose Luis Blanco Claraco
 * @date   Mar 5, 2024
 */

#include <mola_pose_list/SearchablePoseList.h>
#include <mrpt/poses/Lie/SO.h>

using namespace mola;

std::tuple<bool /*isFirst*/, mrpt::poses::CPose3D /*distanceToClosest*/> SearchablePoseList::check(
    const mrpt::poses::CPose3D& p) const
{
  const bool           isFirst = empty();
  mrpt::poses::CPose3D distanceToClosest;
  if (isFirst)
  {
    return {isFirst, distanceToClosest};
  }

  if (from_last_only_)
  {
    distanceToClosest = p - last_kf_;
  }
  else
  {
    ASSERT_EQUAL_(kf_poses_.size(), kf_points_.size());

    std::vector<mrpt::math::TPoint3Df> closest;
    std::vector<float>                 closestSqrDist;
    std::vector<uint64_t>              closestID;

    // Cap k at the actual cloud size: nn_multiple_search resizes its output
    // vectors to k regardless of how many neighbours it finds, leaving
    // trailing entries uninitialized when fewer than k points exist. Reading
    // those garbage entries would corrupt the best-match selection below.
    const size_t k = std::min<size_t>(20, kf_points_.size());
    kf_points_.nn_multiple_search(
        p.translation().cast<float>(), k, closest, closestSqrDist, closestID);
    ASSERT_(!closest.empty());  // empty()==false from check above

    // Check for both, rotation and translation.
    // Use a heuristic SE(3) metric to merge both parts:
    constexpr double ROTATION_WEIGHT = 1.0;

    std::optional<size_t> bestIdx;

    for (size_t i = 0; i < closest.size(); i++)
    {
      const auto&  candidate = kf_poses_.at(closestID.at(i));
      const double rot = mrpt::poses::Lie::SO<3>::log((p - candidate).getRotationMatrix()).norm();

      closestSqrDist[i] += static_cast<float>(ROTATION_WEIGHT * mrpt::square(rot));

      if (!bestIdx || closestSqrDist[i] < closestSqrDist[*bestIdx])
      {
        bestIdx = i;
      }
    }

    const auto& closestPose = kf_poses_.at(closestID.at(*bestIdx));

    distanceToClosest = p - closestPose;
  }

  return {isFirst, distanceToClosest};
}

uint32_t SearchablePoseList::countNearby(
    const mrpt::poses::CPose3D& p, const double maxTranslation, const double maxRotationRad) const
{
  if (empty())
  {
    return 0;
  }

  if (from_last_only_)
  {
    if (!has_last_kf_) return 0;
    const mrpt::poses::CPose3D diff  = p - last_kf_;
    const double               trans = diff.translation().norm();
    if (trans > maxTranslation)
    {
      return 0;
    }
    const double rot = mrpt::poses::Lie::SO<3>::log(diff.getRotationMatrix()).norm();
    return (rot <= maxRotationRad) ? 1u : 0u;
  }

  ASSERT_EQUAL_(kf_poses_.size(), kf_points_.size());

  uint32_t count = 0;
  for (const auto& candidate : kf_poses_)
  {
    const mrpt::poses::CPose3D diff = p - candidate;
    if (diff.translation().norm() > maxTranslation)
    {
      continue;
    }
    const double rot = mrpt::poses::Lie::SO<3>::log(diff.getRotationMatrix()).norm();
    if (rot <= maxRotationRad)
    {
      ++count;
    }
  }
  return count;
}

void SearchablePoseList::removeAllFartherThan(
    const mrpt::poses::CPose3D& p, const double maxTranslation)
{
  if (from_last_only_)
  {
    return;  // not applicable
  }

  std::deque<mrpt::poses::CPose3D>                    new_kf_poses;
  mrpt::maps::CSimplePointsMap                        new_kf_points;
  std::deque<std::optional<SearchablePoseList::KFID>> new_kf_ids;
  std::map<SearchablePoseList::KFID, size_t>          new_id_to_idx;

  const double maxSqrDist = mrpt::square(maxTranslation);
  const auto   c          = p.translation();

  for (size_t i = 0; i < kf_poses_.size(); i++)
  {
    mrpt::math::TPoint3D pt;
    kf_points_.getPoint(i, pt.x, pt.y, pt.z);
    if ((pt - c).sqrNorm() > maxSqrDist)
    {
      continue;  // remove
    }
    // pass:
    const size_t newIdx = new_kf_poses.size();
    new_kf_points.insertPoint(pt);
    new_kf_poses.push_back(kf_poses_.at(i));
    new_kf_ids.push_back(kf_ids_.at(i));
    if (kf_ids_.at(i)) new_id_to_idx[*kf_ids_.at(i)] = newIdx;
  }
  // replace:
  kf_poses_  = std::move(new_kf_poses);
  kf_points_ = std::move(new_kf_points);  // NOLINT
  kf_ids_    = std::move(new_kf_ids);
  id_to_idx_ = std::move(new_id_to_idx);
  ASSERT_EQUAL_(kf_poses_.size(), kf_points_.size());
  ASSERT_EQUAL_(kf_poses_.size(), kf_ids_.size());
}

void SearchablePoseList::setPoseById(KFID id, const mrpt::poses::CPose3D& new_pose)
{
  if (from_last_only_) return;
  auto it = id_to_idx_.find(id);
  if (it == id_to_idx_.end()) return;
  const size_t idx  = it->second;
  kf_poses_.at(idx) = new_pose;
  const auto t      = new_pose.translation();
  // Update the kd-tree point in place. setPoint() invalidates the kd-tree.
  kf_points_.setPoint(idx, t.x, t.y, t.z);
}
