/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EntityBase.h
 * @brief  Virtual base class for all optimizable entities in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola-kernel/Keyframe.h>
#include <map>

namespace mola
{
/** Virtual base class for all optimizable entities in the world model
 *
 * \ingroup mola_kernel_grp
 */
class EntityBase
{
   public:
	EntityBase()          = default;
	virtual ~EntityBase() = default;
};

}  // namespace mola
