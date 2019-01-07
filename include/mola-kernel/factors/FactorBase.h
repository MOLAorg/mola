/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorBase.h
 * @brief  Base class for all "factors" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola-kernel/id.h>

namespace mola
{
/** Base class for all "factors" in the world model
 *
 * \ingroup mola_kernel_grp
 */
class FactorBase
{
   public:
    FactorBase() = default;
    virtual ~FactorBase();

    /** The unique ID of this factor in the world model.
     * Stored here for convenience, notice that it is redundant since entities
     * are already stored in the WorldModel indexed by ID.
     */
    mola::fid_t my_id_{mola::INVALID_FID};
};

}  // namespace mola
