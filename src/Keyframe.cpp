/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   KeyFrame.cpp
 * @brief  Information keep for each keyframe (except its "global" pose)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */

#include <mola-kernel/Keyframe.h>

using namespace mola;

mrpt::Clock::time_point Keyframe::timestamp() const
{
    ASSERT_(raw_observations_);
    ASSERT_(!raw_observations_->empty());
    return (*raw_observations_->begin())->timestamp;
}
