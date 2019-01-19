/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LazyLoadResource.cpp
 * @brief  Pointer-like object capable of being stored offline when not used.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 19, 2019
 */

#include <mola-kernel/LazyLoadResource.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

void LazyLoadResource::externalStorage(const std::string& f)
{
    external_filename_ = f;
}

void LazyLoadResource::load() const
{
    MRPT_START

    if (data_) return;
    if (external_filename_.empty())
        THROW_EXCEPTION(
            "Trying to load() a resource without associated external file!");

    mrpt::io::CFileGZInputStream f;
    if (!f.open(external_filename_))
        THROW_EXCEPTION_FMT(
            "Cannot read from file: `%s`", external_filename_.c_str());

    auto a = mrpt::serialization::archiveFrom(f);
    data_  = a.ReadObject();
    ASSERTMSG_(data_, "Could not load resource from external storage");

    if (auto obj = dynamic_cast<mrpt::obs::CObservation*>(data_.get());
        obj != nullptr)
    {
        // Recursive load from external storage:
        obj->load();
    }

    MRPT_END
}

void LazyLoadResource::unload() const
{
    if (data_ && external_filename_.empty())
        THROW_EXCEPTION(
            "Trying to unload() a resource without associated external file! "
            "Aborting, it would imply losing data.");

    if (auto obj = dynamic_cast<mrpt::obs::CObservation*>(data_.get());
        obj != nullptr)
    {
        // Recursive unload from external storage:
        obj->unload();
    }

    data_.reset();
}
