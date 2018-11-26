/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WorldModel.h
 * @brief  The main class for a "map" or "world model".
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola-kernel/Keyframe.h>
#include <mola-kernel/EntityBase.h>
#include <map>

namespace mola
{
/** The main class for a "map" or "world model".
 *
 * \ingroup mola_kernel_grp
 */
class WorldModel
{
   public:
    /** @name Main data fields
     * @{ */

	std::map<keyframe_id_t, Keyframe> keyframes_;

    /** @} */
};

}  // namespace mola
