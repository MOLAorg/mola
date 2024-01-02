/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RelPose3KF.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/entities/EntityRelativeBase.h>
#include <mola_kernel/entities/KeyFrameBase.h>
#include <mrpt/math/TPose3D.h>

namespace mola
{
/** A relative SE(3) keyframe, holding a relative SE(3) pose wrt to the base
 * frame.
 * This entity is also a key-frame.
 * \ingroup mola_kernel_grp
 */
class RelPose3KF : public EntityRelativeBase, public KeyFrameBase
{
    DEFINE_SERIALIZABLE(RelPose3KF, mola)

   public:
    /** The up-to-date value of this entity. */
    mrpt::math::TPose3D relpose_value;
};

}  // namespace mola
