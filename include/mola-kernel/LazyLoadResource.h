/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LazyLoadResource.h
 * @brief  Pointer-like object capable of being stored offline when not used.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 19, 2019
 */
#pragma once

#include <mola-kernel/id.h>
#include <mrpt/serialization/CSerializable.h>
#include <string>

namespace mola
{
/**
 * @brief Looks like a `std::shared_ptr`, but is stored on disk when not used.
 */
class LazyLoadResource
{
   public:
    LazyLoadResource() = default;

    LazyLoadResource(
        const mrpt::serialization::CSerializable::Ptr& source,
        const std::string&                             f)
        : data_(source), external_filename_(f)
    {
    }

    const std::string& externalStorage() const { return external_filename_; }

    /** Casts as (a copy of) the underlying smart pointer to the object */
    mrpt::rtti::CObject::Ptr operator()()
    {
        load_proxy();
        return data_;
    }
    /** Casts as (a const-ref to) the underlying smart pointer to the object */
    const mrpt::serialization::CSerializable::Ptr& operator()() const
    {
        load_proxy();
        return data_;
    }

    /** Ensure data is loaded from disk */
    void load() const;
    /** Unload data is loaded from disk */
    void unload() const;

    void reset() { data_.reset(); }

    /** Sets the contents of this container, including the desired external file
     * name */
    void set(
        const mrpt::serialization::CSerializable::Ptr& source,
        const std::string&                             f);

    void setParentEntityID(const mola::id_t id) { parent_entity_id_ = id; }

   private:
    void load_proxy() const
    {
        if (data_) return;
        load();
    }

    const std::string& buildAbsoluteFilePath() const;

    mutable mrpt::serialization::CSerializable::Ptr data_;

    std::string         external_filename_;
    mola::id_t          parent_entity_id_{INVALID_ID};
    mutable std::string cached_abs_fil;
    mutable bool        cached_file_ok{false};
};
}  // namespace mola
