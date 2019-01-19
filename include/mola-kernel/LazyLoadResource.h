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

#include <mrpt/rtti/CObject.h>
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
    /** Sets the external filename where this resource should be loaded from
     * and/or stored to.
     * Path is relative to the storage directory of the parent WorldModel.*/
    void externalStorage(const std::string& f);

    const std::string& externalStorage() const { return external_filename_; }

    /** Casts as (a copy of) the underlying smart pointer to the object */
    mrpt::rtti::CObject::Ptr operator()()
    {
        load_proxy();
        return data_;
    }
    /** Casts as (a const-ref to) the underlying smart pointer to the object */
    const mrpt::rtti::CObject::Ptr& operator()() const
    {
        load_proxy();
        return data_;
    }

    /** Ensure data is loaded from disk */
    void load() const;
    /** Unload data is loaded from disk */
    void unload() const;

    void reset() { data_.reset(); }

   private:
    void load_proxy() const
    {
        if (data_) return;
        load();
    }

    mutable mrpt::rtti::CObject::Ptr data_;
    std::string                      external_filename_;
};
}  // namespace mola
