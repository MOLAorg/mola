/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   KeyFrame.h
 * @brief  Information keep for each keyframe (except its "global" pose)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mrpt/obs/CSensoryFrame.h>

#include <cstdint>

namespace mola
{
/** Keyframes (KFs) are the lowest-level entities in a World Model (a "map")
 * holding raw observation data. We keep raw observations as they were
 * originally observed in each KF, ideally, more than observations only if they
 * were collected exactly at the same timestamp.
 *
 * This class is used as a base class for all world-model entities that can be
 * used as robot/vehicle keyframes.
 *
 * \ingroup mola_kernel_grp
 */
class KeyFrameBase
{
   public:
    mrpt::obs::CSensoryFrame::Ptr raw_observations_;
};

}  // namespace mola
