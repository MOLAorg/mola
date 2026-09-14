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

#include <algorithm>

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

std::vector<SearchablePoseList::NearbyPose> SearchablePoseList::findNearby(
    const mrpt::poses::CPose3D& p, const double maxTranslation, const double maxRotationRad,
    const size_t maxCount) const
{
  std::vector<NearbyPose> out;
  if (empty())
  {
    return out;
  }

  // Fills in the parts of an entry that are common to both branches, and says
  // whether the candidate is inside the neighborhood at all.
  const auto accept = [&](size_t index, const mrpt::poses::CPose3D& candidate,
                          std::optional<KFID> id) -> std::optional<NearbyPose>
  {
    NearbyPose e;
    e.relativePose = p - candidate;
    e.translation  = e.relativePose.translation().norm();
    if (e.translation > maxTranslation)
    {
      return std::nullopt;
    }
    e.rotation = mrpt::poses::Lie::SO<3>::log(e.relativePose.getRotationMatrix()).norm();
    if (e.rotation > maxRotationRad)
    {
      return std::nullopt;
    }
    e.index = index;
    e.id    = id;
    e.pose  = candidate;
    return e;
  };

  if (from_last_only_)
  {
    if (auto e = accept(0, last_kf_, std::nullopt); e.has_value())
    {
      out.push_back(*e);
    }
    return out;
  }

  ASSERT_EQUAL_(kf_poses_.size(), kf_points_.size());

  // The translation bound is a radius, so the k-d tree already holds the
  // answer: only the points it returns can pass, and the rotation test is
  // applied to those alone.
  std::vector<mrpt::math::TPoint3Df> closest;
  std::vector<float>                 closestSqrDist;
  std::vector<uint64_t>              closestID;
  kf_points_.nn_radius_search(
      p.translation().cast<float>(), static_cast<float>(mrpt::square(maxTranslation)), closest,
      closestSqrDist, closestID, 0 /* maxPoints: unlimited */);

  for (const auto id : closestID)
  {
    const size_t index = static_cast<size_t>(id);
    if (auto e = accept(index, kf_poses_.at(index), kf_ids_.at(index)); e.has_value())
    {
      out.push_back(*e);
    }
  }

  std::sort(
      out.begin(), out.end(),
      [](const NearbyPose& a, const NearbyPose& b) { return a.translation < b.translation; });

  if (maxCount != 0 && out.size() > maxCount)
  {
    out.resize(maxCount);
  }
  return out;
}

uint32_t SearchablePoseList::countNearby(
    const mrpt::poses::CPose3D& p, const double maxTranslation, const double maxRotationRad) const
{
  return static_cast<uint32_t>(findNearby(p, maxTranslation, maxRotationRad).size());
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

void SearchablePoseList::transform_left_multiply(const mrpt::poses::CPose3D& b)
{
  if (from_last_only_)
  {
    if (has_last_kf_)
    {
      last_kf_ = b + last_kf_;
    }
    return;
  }

  ASSERT_EQUAL_(kf_poses_.size(), kf_points_.size());

  for (size_t i = 0; i < kf_poses_.size(); i++)
  {
    kf_poses_.at(i) = b + kf_poses_.at(i);
    const auto t    = kf_poses_.at(i).translation();
    // setPoint() invalidates the kd-tree, which is rebuilt on the next query.
    kf_points_.setPoint(i, t.x, t.y, t.z);
  }
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
