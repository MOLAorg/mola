/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EntityBase.h
 * @brief  Base class for all "entities" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola-kernel/LazyLoadResource.h>
#include <mola-kernel/id.h>
#include <map>

namespace mola
{
/** Arbitrary annotated data, indexed by "label" or "variable name" and
 *capable of offline storage on disk when not used.
 * \ingroup mola_kernel_grp
 */
using annotations_data_t = std::map<std::string, LazyLoadResource>;

/** Base class for all "entities" in the world model
 *
 * \ingroup mola_kernel_grp
 */
class EntityBase
{
   public:
    EntityBase() = default;
    virtual ~EntityBase();

    /** The unique ID of this entity in the world model.
     * Stored here for convenience, notice that it is redundant since entities
     * are already stored in the WorldModel indexed by ID.
     */
    mola::id_t my_id_{mola::INVALID_ID};

    /** See annotations_data_t */
    annotations_data_t annotations_;
};

}  // namespace mola
