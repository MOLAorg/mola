/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RelDynPose3KF.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola-kernel/entities/EntityRelativeBase.h>
#include <mola-kernel/entities/KeyFrameBase.h>

namespace mola
{
/** A relative "dynamic" pose: SE(3) pose + velocity vector.
 * Both the pose and the velocity vector are given in the frame of the base KF.
 * This entity is always a key-frame.
 * \ingroup mola_kernel_grp
 */
class RelDynPose3KF : public EntityRelativeBase, public KeyFrameBase
{
   public:
};

}  // namespace mola
