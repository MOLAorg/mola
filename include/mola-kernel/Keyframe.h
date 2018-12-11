/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
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
/** Numeric, unique, ID for each mola::Keyframe. \ingroup mola_kernel_grp */
using keyframe_id_t = std::uint64_t;

/** Keyframes (KFs) are the lowest-level entities in a World Model (a "map").
 * We keep raw observations as they were originally observed in each KF,
 * ideally, more than observations only if they were collected exactly at the
 * same timestamp.
 * \ingroup mola_kernel_grp
 */
class Keyframe
{
public:
    Keyframe() = default;
    Keyframe(keyframe_id_t id) : kf_id_(id) {}

    /** @name Keyframe data fields
     * @{ */
    keyframe_id_t                 kf_id_{0};
    mrpt::obs::CSensoryFrame::Ptr raw_observations_;
    /** @} */

    /** Returns the timestamp of the first observation */
    mrpt::Clock::time_point timestamp() const;
};

}  // namespace mola
