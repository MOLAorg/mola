/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EntityRelativeBase.h
 * @brief  World optimizable entities which are relative to some keyframe.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola_kernel/entities/EntityBase.h>

namespace mola
{
/** World optimizable entities which are relative to some keyframe.
 *
 * \ingroup mola_kernel_grp
 */
class EntityRelativeBase : public EntityBase
{
   public:
    /** The ID of the base keyframe (entity) */
    mola::id_t base_id_{mola::INVALID_ID};
};

}  // namespace mola
