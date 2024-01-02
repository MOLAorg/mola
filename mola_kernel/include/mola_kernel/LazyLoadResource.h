/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LazyLoadResource.h
 * @brief  Pointer-like object capable of being stored offline when not used.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 19, 2019
 */
#pragma once

#include <mola_kernel/id.h>
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

    /** Access (a copy of) the underlying smart pointer to the object */
    mrpt::serialization::CSerializable::Ptr value()
    {
        internalLoadProxy();
        return data_;
    }
    /** Access (a const-ref to) the underlying smart pointer to the object */
    const mrpt::serialization::CSerializable::Ptr& value() const
    {
        internalLoadProxy();
        return data_;
    }

    /** Ensure data is loaded from disk, if it was automatically swapped-off */
    void load() const;

    /** Unload data and save to disk now, if not already done before */
    void unload() const;
    bool isUnloaded() const;

    /** Empty this container. */
    void reset() { *this = LazyLoadResource(); }

    /** Sets the contents of this container, including the desired external file
     * name */
    void set(
        const mrpt::serialization::CSerializable::Ptr& source,
        const std::string&                             f);

    /** Sets this container as externally-stored, given an external filename.
     * The external content is *not* automatically loaded, for that, explicitly
     * call load() if needed.
     */
    void setAsExternal(const std::string& relativeFileName);

    void setParentEntityID(const mola::id_t id) { parent_entity_id_ = id; }

    static std::string EXTERNAL_BASE_DIR;

   private:
    inline void internalLoadProxy() const
    {
        if (data_) return;
        load();
    }

    const std::string& buildAbsoluteFilePath() const;

    mutable mrpt::serialization::CSerializable::Ptr data_;

    std::string         external_filename_;
    mola::id_t          parent_entity_id_{INVALID_ID};
    mutable std::string cached_abs_fil_;
    mutable bool        cached_file_ok_{false};
};
}  // namespace mola
