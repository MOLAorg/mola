/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawlogDataset.cpp
 * @brief  RawDataSource for datasets in MRPT rawlog format
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2019
 */

/** \defgroup mola_input_rawlog_grp mola_input_rawlog_grp.
 * RawDataSource for datasets in MRPT rawlog format
 *
 */

#include <mola_input_rawlog/RawlogDataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(RawlogDataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_RawlogDataset)
{
    MOLA_REGISTER_MODULE(RawlogDataset);
}

RawlogDataset::RawlogDataset() = default;

void RawlogDataset::initialize_rds(const Yaml& c)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    // Mandatory parameters:
    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

    YAML_LOAD_MEMBER_REQ(rawlog_filename, std::string);
    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);
    YAML_LOAD_MEMBER_OPT(read_all_first, bool);
    paused_ = cfg.getOrDefault<bool>("start_paused", paused_);

    ASSERT_FILE_EXISTS_(rawlog_filename_);

    // Detect the external files directory, if used:
    const auto imgsDir =
        mrpt::obs::CRawlog::detectImagesDirectory(rawlog_filename_);
    if (mrpt::system::directoryExists(imgsDir))
    {
        mrpt::io::setLazyLoadPathBase(imgsDir);
        MRPT_LOG_INFO_STREAM(
            "Setting rawlog external directory to: " << imgsDir);
    }

    if (read_all_first_)
    {
        MRPT_LOG_INFO_STREAM(
            "Reading the whole rawlog dataset: " << rawlog_filename_);

        rawlog_entire_.loadFromRawLogFile(rawlog_filename_);

        MRPT_LOG_INFO_STREAM(
            "Read ok, with " << rawlog_entire_.size() << " entries.");
    }
    else
    {
        if (!rawlog_in_.open(rawlog_filename_))
            throw std::runtime_error("Cannot open input rawlog!");
    }

    MRPT_END
}  // end initialize()

void RawlogDataset::spinOnce()
{
    using mrpt::system::timeDifference;

    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    const auto tNow = mrpt::Clock::now();

    if (!last_play_wallclock_time_) last_play_wallclock_time_ = tNow;

    doReadAhead();

    if (read_ahead_.empty())
    {
        onDatasetPlaybackEnds();  // notify base class

        MRPT_LOG_THROTTLE_INFO(
            10.0,
            "End of dataset reached! Nothing else to publish (CTRL+C to quit)");
        return;
    }
    else if (read_all_first_)
    {
        MRPT_LOG_THROTTLE_INFO_FMT(
            5.0, "Dataset replay progress: %lu / %lu  (%4.02f%%)",
            static_cast<unsigned long>(rawlog_next_idx_),
            static_cast<unsigned long>(rawlog_entire_.size()),
            (100.0 * rawlog_next_idx_) / (rawlog_entire_.size()));
    }

    // First rawlog timestamp?
    if (rawlog_begin_time_ == INVALID_TIMESTAMP)
        rawlog_begin_time_ = read_ahead_.begin()->first;

    auto         lckUIVars       = mrpt::lockHelper(dataset_ui_mtx_);
    const double time_warp_scale = time_warp_scale_;
    const bool   paused          = paused_;
    const auto   teleport_here   = teleport_here_;
    teleport_here_.reset();
    lckUIVars.unlock();

    double dt = mrpt::system::timeDifference(*last_play_wallclock_time_, tNow) *
                time_warp_scale;
    last_play_wallclock_time_ = tNow;

    // override by an special teleport order?
    if (read_all_first_ && teleport_here.has_value() &&
        *teleport_here < rawlog_entire_.size())
    {
        rawlog_next_idx_ = *teleport_here;
        if (auto obs = rawlog_entire_.getAsObservation(rawlog_next_idx_); obs)
            last_dataset_time_ = mrpt::system::timeDifference(
                rawlog_begin_time_, obs->timestamp);
    }
    else
    {
        if (paused) return;
        // move forward replayed dataset time:
        last_dataset_time_ += dt;
    }

    doReadAhead();

    // Publish observations up to current time:
    while (!read_ahead_.empty() &&
           last_dataset_time_ >=
               timeDifference(rawlog_begin_time_, read_ahead_.begin()->first))
    {
        //
        CObservation::Ptr obs = read_ahead_.begin()->second;
        this->sendObservationsToFrontEnds(obs);

        unload_queue_.emplace(obs->getTimeStamp(), obs);
        read_ahead_.erase(read_ahead_.begin());

        MRPT_LOG_DEBUG_STREAM(
            "Publishing " << obs->GetRuntimeClass()->className
                          << " sensorLabel: " << obs->sensorLabel << " for t="
                          << last_dataset_time_ << " observation timestamp="
                          << mrpt::system::dateTimeLocalToString(
                                 obs->timestamp));
    }

    if (read_all_first_)
    {
        auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
        last_used_tim_index_ = rawlog_next_idx_;
    }

    MRPT_END
}

constexpr size_t READ_AHEAD_LEN = 10;
constexpr size_t MAX_UNLOAD_LEN = 500;

void RawlogDataset::doReadAheadFromFile()
{
    MRPT_START

    auto rawlog_arch = mrpt::serialization::archiveFrom(rawlog_in_);

    while (read_ahead_.size() < READ_AHEAD_LEN)
    {
        try
        {
            const auto obj = rawlog_arch.ReadObject();
            if (auto obs =
                    std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj);
                obs)
            {  // Single observation:
                read_ahead_.emplace(obs->getTimeStamp(), std::move(obs));
            }
            else  //
                if (auto sf =
                        std::dynamic_pointer_cast<mrpt::obs::CSensoryFrame>(
                            obj);
                    sf)
            {
                for (const auto& o : *sf)
                    read_ahead_.emplace(o->getTimeStamp(), o);
            }
            else if (auto acts = std::dynamic_pointer_cast<
                         mrpt::obs::CActionCollection>(obj);
                     acts)
            {
                // odometry actions: ignore
            }
            else
                THROW_EXCEPTION_FMT(
                    "Rawlog file can contain classes: "
                    "CObservation|CSensoryFrame|CActionCollection, but class "
                    "'%s' found.",
                    obj->GetRuntimeClass()->className);
        }
        catch (const mrpt::serialization::CExceptionEOF&)
        {
            return;  // EOF reached.
        }
        catch (const std::exception&)
        {
            throw;
        }
    }

    MRPT_END
}

void RawlogDataset::doReadAheadFromEntireRawlog()
{
    while (read_ahead_.size() < READ_AHEAD_LEN &&
           rawlog_next_idx_ < rawlog_entire_.size())
    {
        const auto obj = rawlog_entire_.getAsGeneric(rawlog_next_idx_++);
        if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj);
            obs)
        {  // Single observation:
            read_ahead_.emplace(obs->getTimeStamp(), std::move(obs));
        }
        else  //
            if (auto sf =
                    std::dynamic_pointer_cast<mrpt::obs::CSensoryFrame>(obj);
                sf)
        {
            for (const auto& o : *sf) read_ahead_.emplace(o->getTimeStamp(), o);
        }
        else if (auto acts =
                     std::dynamic_pointer_cast<mrpt::obs::CActionCollection>(
                         obj);
                 acts)
        {
            // odometry actions: ignore
        }
        else
            THROW_EXCEPTION_FMT(
                "Rawlog file can contain classes: "
                "CObservation|CSensoryFrame|CActionCollection, but class "
                "'%s' found.",
                obj->GetRuntimeClass()->className);
    }
}

void RawlogDataset::doReadAhead()
{
    if (read_all_first_)  //
        doReadAheadFromEntireRawlog();
    else
        doReadAheadFromFile();

    // and also, unload() very old observations.
    autoUnloadOldEntries();
}

void RawlogDataset::autoUnloadOldEntries() const
{
    // unload() very old observations.
    // Motivation:
    // the lazy-load data may remain loaded in the rawlog objects
    // even if no shared_ptr is active in the client consumer anymore.
    while (unload_queue_.size() > MAX_UNLOAD_LEN)
    {
        unload_queue_.begin()->second->unload();
        unload_queue_.erase(unload_queue_.begin());
    }
}

// See docs in base class:
size_t RawlogDataset::datasetSize() const
{
    ASSERTMSG_(
        read_all_first_,
        "Using the OfflineDatasetSource API in this class requires setting "
        "'read_all_first' to 'true'");

    return rawlog_entire_.size();
}

mrpt::obs::CSensoryFrame::Ptr RawlogDataset::datasetGetObservations(
    size_t timestep) const
{
    ASSERTMSG_(
        read_all_first_,
        "Using the OfflineDatasetSource API in this class requires setting "
        "'read_all_first' to 'true'");

    autoUnloadOldEntries();  // see inside function comments for motivation

    {
        auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
        last_used_tim_index_ = timestep;
    }

    const auto obj = rawlog_entire_.getAsGeneric(timestep);

    auto sfRet = mrpt::obs::CSensoryFrame::Create();

    if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj); obs)
    {  // Single observation:

        sfRet->insert(obs);

        // enqueue for auto unload in the future:
        unload_queue_.emplace(obs->getTimeStamp(), obs);
    }
    else  //
        if (auto sf = std::dynamic_pointer_cast<mrpt::obs::CSensoryFrame>(obj);
            sf)
    {
        sfRet = sf;

        for (auto& o : *sf) unload_queue_.emplace(o->getTimeStamp(), o);
    }
    else if (auto acts =
                 std::dynamic_pointer_cast<mrpt::obs::CActionCollection>(obj);
             acts)
    {
        // odometry actions: ignore
    }

    return sfRet;
}
