/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
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

    SearchablePoseList(bool measure_from_last_kf_only)
        : from_last_only_(measure_from_last_kf_only)
    {
    }

    bool empty() const
    {
        if (from_last_only_)  //
            return last_kf_ == mrpt::poses::CPose3D::Identity();
        else
            return kf_poses_.empty();
    }

    size_t size() const { return from_last_only_ ? 1 : kf_poses_.size(); }

    void insert(const mrpt::poses::CPose3D& p)
    {
        if (from_last_only_)
        {  //
            last_kf_ = p;
        }
        else
        {
            kf_points_.insertPoint(p.translation());
            kf_poses_.push_back(p);
        }
    }

    [[nodiscard]] std::tuple<
        bool /*isFirst*/, mrpt::poses::CPose3D /*distanceToClosest*/>
        check(const mrpt::poses::CPose3D& p) const;

    void removeAllFartherThan(
        const mrpt::poses::CPose3D& p, const double maxTranslation);

   private:
    // if from_last_only_==true
    mrpt::poses::CPose3D last_kf_ = mrpt::poses::CPose3D::Identity();

    // if from_last_only_==false
    std::deque<mrpt::poses::CPose3D> kf_poses_;
    mrpt::maps::CSimplePointsMap     kf_points_;

    bool from_last_only_ = false;
};
}  // namespace mola
