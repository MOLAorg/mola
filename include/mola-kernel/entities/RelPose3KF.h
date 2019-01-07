/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RelPose3KF.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola-kernel/entities/EntityRelativeBase.h>
#include <mola-kernel/entities/KeyFrameBase.h>

namespace mola
{
/** A relative SE(3) keyframe.
 *
 * \ingroup mola_kernel_grp
 */
class RelPose3KF : public EntityRelativeBase, public KeyFrameBase
{
   public:
};

}  // namespace mola
