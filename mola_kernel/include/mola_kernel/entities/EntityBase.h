/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EntityBase.h
 * @brief  Base class for all "entities" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola_kernel/LazyLoadResource.h>
#include <mola_kernel/id.h>
#include <mrpt/core/Clock.h>
#include <mrpt/serialization/CSerializable.h>

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
class EntityBase : public mrpt::serialization::CSerializable
{
    DEFINE_VIRTUAL_SERIALIZABLE(EntityBase);

   public:
    EntityBase();
    virtual ~EntityBase();

    /** The unique ID of this entity in the world model.
     * Stored here for convenience, notice that it is redundant since entities
     * are already stored in the WorldModel indexed by ID.
     */
    mola::id_t my_id_{mola::INVALID_ID};

    /** Entity creation timestamp */
    mrpt::Clock::time_point timestamp_{};

    /** See annotations_data_t */
    annotations_data_t annotations_;

    void load();
    void unload();
    bool is_unloaded() const;

   protected:
    // Derived classes mus call these methods to serialize the common data in
    // this base class:
    void baseSerializeTo(mrpt::serialization::CArchive& out) const;
    void baseSerializeFrom(mrpt::serialization::CArchive& in);
};

}  // namespace mola
