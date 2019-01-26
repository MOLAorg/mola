/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorDynamicsConstVel.h
 * @brief  Constant-velocity model factor
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola-kernel/factors/FactorBase.h>
#include <mrpt/core/exceptions.h>

namespace mola
{
/**
 *
 * \ingroup mola_kernel_grp
 */
class FactorDynamicsConstVel : public FactorBase
{
   public:
    /** Creates relative pose constraint of KF `to` as seem from `from`. */
    FactorDynamicsConstVel(id_t kf_from, id_t kf_to)
        : from_kf_(kf_from), to_kf_{kf_to}
    {
    }

    id_t from_kf_{INVALID_ID}, to_kf_{INVALID_ID};

    std::size_t edge_count() const override { return 2; }
    std::size_t edge_indices(const std::size_t i) const override
    {
        switch (i)
        {
            case 0:
                return from_kf_;
            case 1:
                return to_kf_;
            default:
                THROW_EXCEPTION("Out of bounds");
        }
    }
};

}  // namespace mola
