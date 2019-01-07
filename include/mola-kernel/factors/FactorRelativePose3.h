/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorRelativePose3.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola-kernel/factors/FactorBase.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mola
{
/**
 * \todo Handle covariance matrix
 *
 * \ingroup mola_kernel_grp
 */
class FactorRelativePose3 : public FactorBase
{
   public:
    /** Creates relative pose constraint of KF `to` as seem from `from`. */
    FactorRelativePose3(
        id_t kf_from, id_t kf_to, const mrpt::math::TPose3D& rel_pose)
        : from_kf_(kf_from), to_kf_{kf_to}, rel_pose_{rel_pose}
    {
    }

    id_t                from_kf_{INVALID_ID}, to_kf_{INVALID_ID};
    mrpt::math::TPose3D rel_pose_;
};

}  // namespace mola
