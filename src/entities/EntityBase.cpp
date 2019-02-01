/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EntityBase.cpp
 * @brief  Base class for all "entities" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */

#include <mola-kernel/Entity.h>
#include <mola-kernel/entities/EntityBase.h>
#include <mola-kernel/variant_helper.h>

// TODO: make serializable

using namespace mola;

EntityBase::EntityBase()  = default;
EntityBase::~EntityBase() = default;

void EntityBase::load()
{
    MRPT_TRY_START

    // Always: unload annotations:
    for (auto& a : annotations_)
    {
        a.second.setParentEntityID(my_id_);
        a.second.load();
    }

    // If I am a KeyFrame: unload observations:
    if (auto kf = dynamic_cast<KeyFrameBase*>(this); kf != nullptr)
    {
        // TO DO:
        MRPT_TODO("Reload kf->raw_observations_");
    }

    MRPT_TRY_END
}
void EntityBase::unload()
{
    MRPT_TRY_START

    // Always: unload annotations:
    for (auto& a : annotations_)
    {
        a.second.setParentEntityID(my_id_);
        a.second.unload();
    }

    // If I am a KeyFrame: unload observations:
    if (auto kf = dynamic_cast<KeyFrameBase*>(this); kf != nullptr)
    {
        // Unload heavy observation data back to disk:
        if (kf->raw_observations_)
        {
            for (auto& obs : *kf->raw_observations_) { obs->unload(); }

            kf->raw_observations_.reset();
        }
    }

    MRPT_TRY_END
}
