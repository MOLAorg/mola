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

namespace mola
{
/** Data structure to search for nearby SE(3) poses.
 *
 *  It uses a KD-tree for the search.
 *
 * \ingroup mola_pose_list_grp
 */
class SearchablePoseList
{
 public:
  SearchablePoseList() = default;

  SearchablePoseList(bool measure_from_last_kf_only) : from_last_only_(measure_from_last_kf_only) {}

  bool empty() const
  {
    if (from_last_only_)
    {
      return last_kf_ == mrpt::poses::CPose3D::Identity();
    }

    return kf_poses_.empty();
  }

  size_t size() const { return from_last_only_ ? 1 : kf_poses_.size(); }

  void insert(const mrpt::poses::CPose3D& p)
  {
    if (from_last_only_)
    {
      last_kf_ = p;
    }
    else
    {
      kf_points_.insertPoint(p.translation());
      kf_poses_.push_back(p);
    }
  }

  [[nodiscard]] std::tuple<bool /*isFirst*/, mrpt::poses::CPose3D /*distanceToClosest*/> check(
      const mrpt::poses::CPose3D& p) const;

  void removeAllFartherThan(const mrpt::poses::CPose3D& p, double maxTranslation);

 private:
  // if from_last_only_==true
  mrpt::poses::CPose3D last_kf_ = mrpt::poses::CPose3D::Identity();

  // if from_last_only_==false
  std::deque<mrpt::poses::CPose3D> kf_poses_;
  mrpt::maps::CSimplePointsMap     kf_points_;

  bool from_last_only_ = false;
};
}  // namespace mola
