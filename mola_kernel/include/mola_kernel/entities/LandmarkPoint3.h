/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LandmarkPoint3.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/entities/EntityBase.h>
#include <mrpt/math/TPoint3D.h>

namespace mola
{
/** A 3D point landmark.
 *
 * \ingroup mola_kernel_grp
 */
class LandmarkPoint3 : public EntityBase
{
    DEFINE_SERIALIZABLE(LandmarkPoint3, mola)

   public:
    mrpt::math::TPoint3D point;
};

}  // namespace mola
