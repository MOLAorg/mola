/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LazyLoadResource.cpp
 * @brief  Pointer-like object capable of being stored offline when not used.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 19, 2019
 */

#include <mola_kernel/LazyLoadResource.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

using namespace mola;

static const int GZ_COMPRESS_LEVEL = 0;

std::string LazyLoadResource::EXTERNAL_BASE_DIR{""};

const std::string& LazyLoadResource::buildAbsoluteFilePath() const
{
    if (cached_file_ok_) return cached_abs_fil_;

    cached_abs_fil_ = mrpt::format(
        "ent_%06lu_%s", static_cast<long unsigned>(parent_entity_id_),
        external_filename_.c_str());

    cached_abs_fil_ = mrpt::system::fileNameStripInvalidChars(cached_abs_fil_);

    ASSERT_(!EXTERNAL_BASE_DIR.empty());
    cached_abs_fil_ = EXTERNAL_BASE_DIR + cached_abs_fil_;

    cached_file_ok_ = true;
    return cached_abs_fil_;
}

void LazyLoadResource::set(
    const mrpt::serialization::CSerializable::Ptr& source, const std::string& f)
{
    data_              = source;
    external_filename_ = f;
    cached_file_ok_    = false;
}

void LazyLoadResource::load() const
{
    MRPT_START
    const auto& fil = buildAbsoluteFilePath();

    if (data_) return;
    if (external_filename_.empty())
        THROW_EXCEPTION(
            "Trying to load() a swapped-off resource without an associated "
            "external file");

    mrpt::io::CFileGZInputStream f;
    if (!f.open(fil))
        THROW_EXCEPTION_FMT("Cannot read from file: `%s`", fil.c_str());

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

void LazyLoadResource::setAsExternal(const std::string& relativeFileName)
{
    reset();
    external_filename_ = relativeFileName;
}

bool LazyLoadResource::isUnloaded() const { return !data_; }

void LazyLoadResource::unload() const
{
    const auto& fil = buildAbsoluteFilePath();

    if (data_ && external_filename_.empty())
        THROW_EXCEPTION(
            "Trying to unload() a resource without associated external file. "
            "Aborting, it would imply losing data.");

    MRPT_TODO(
        "Add a setter for a 'modified' flag to re-write to disk if needed");

    // If the file already exists, assume it's up-to-date and dont overwrite.
    if (!mrpt::system::fileExists(fil))
    {
        mrpt::io::CFileGZOutputStream f;
        if (!f.open(fil, GZ_COMPRESS_LEVEL))
            THROW_EXCEPTION_FMT("Cannot write to file: `%s`", fil.c_str());

        auto a = mrpt::serialization::archiveFrom(f);
        a << data_;
    }

    if (auto obj = dynamic_cast<mrpt::obs::CObservation*>(data_.get());
        obj != nullptr)
    {
        // Recursive unload from external storage:
        obj->unload();
    }

    data_.reset();
}
