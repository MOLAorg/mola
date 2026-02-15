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

    kf_points_.nn_multiple_search(
        p.translation().cast<float>(), 20, closest, closestSqrDist, closestID);
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

void SearchablePoseList::removeAllFartherThan(
    const mrpt::poses::CPose3D& p, const double maxTranslation)
{
  if (from_last_only_)
  {
    return;  // not applicable
  }

  std::deque<mrpt::poses::CPose3D> new_kf_poses;
  mrpt::maps::CSimplePointsMap     new_kf_points;

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
    new_kf_points.insertPoint(pt);
    new_kf_poses.push_back(kf_poses_.at(i));
  }
  // replace:
  kf_poses_  = std::move(new_kf_poses);
  kf_points_ = std::move(new_kf_points);  // NOLINT
  ASSERT_EQUAL_(kf_poses_.size(), kf_points_.size());
}
