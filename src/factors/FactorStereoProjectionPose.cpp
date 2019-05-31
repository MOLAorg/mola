/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorStereoProjectionPose.cpp
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */

#include <mola-kernel/BackEndBase.h>
#include <mola-kernel/factors/FactorStereoProjectionPose.h>

// TODO: make serializable

using namespace mola;

std::size_t FactorStereoProjectionPose::edge_count() const { return 1; }
mola::id_t  FactorStereoProjectionPose::edge_indices(const std::size_t i) const
{
    ASSERT_EQUAL_(i, 0UL);
    return observing_kf_;
}
