/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EntityBase.cpp
 * @brief  Base class for all "entities" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */

#include <mola_kernel/Entity.h>
#include <mola_kernel/entities/EntityBase.h>
#include <mola_kernel/variant_helper.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

// arguments: class, parent class, namespace
IMPLEMENTS_VIRTUAL_SERIALIZABLE(
    EntityBase, mrpt::serialization::CSerializable, mola);

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

bool EntityBase::is_unloaded() const
{
    MRPT_TRY_START

    bool is_unloaded = true;

    // Always: unload annotations:
    for (auto& a : annotations_)
        is_unloaded = is_unloaded && a.second.isUnloaded();

    /*    // If I am a KeyFrame: unload observations:
        if (auto kf = dynamic_cast<const KeyFrameBase*>(this); kf != nullptr)
            is_unloaded = is_unloaded && kf->raw_observations_.get() == nullptr;
    */
    return is_unloaded;

    MRPT_TRY_END
}

void EntityBase::baseSerializeTo(mrpt::serialization::CArchive& out) const
{
    const uint8_t base_serial_version = 0;
    out << base_serial_version;

    out << my_id_ << timestamp_;

    out.WriteAs<uint32_t>(annotations_.size());
    for (const auto& a : annotations_)
    {
        out << a.first;
        // this saves data to disk to independent file
        a.second.unload();
        // Save name of external file so we know what to load when
        // de-serializing:
        out << a.second.externalStorage();
    }
}
void EntityBase::baseSerializeFrom(mrpt::serialization::CArchive& in)
{
    uint8_t base_serial_version = in.ReadAs<uint8_t>();
    ASSERT_(base_serial_version == 0);

    in >> my_id_ >> timestamp_;

    const auto nAnnotations = in.ReadAs<uint32_t>();
    annotations_.clear();

    for (uint32_t i = 0; i < nAnnotations; i++)
    {
        std::string annotationName, annotationExternalFilename;
        in >> annotationName >> annotationExternalFilename;
        auto& a = annotations_[annotationName];
        a.setAsExternal(annotationExternalFilename);
    }
}
