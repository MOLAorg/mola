/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RefPose3.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/entities/EntityBase.h>

namespace mola
{
/** A Reference SE(3) keyframe.
 * Does not hold raw observations.
 * This kind of frame is used as "coordinate origin" for both, absolute and
 * relative maps (submaps). Global SLAM frameworks should have only one entity
 * of this class, submapping approaches will have several instances.
 *
 * \ingroup mola_kernel_grp
 */
class RefPose3 : public EntityBase
{
    DEFINE_SERIALIZABLE(RefPose3, mola)

   public:
};

}  // namespace mola
