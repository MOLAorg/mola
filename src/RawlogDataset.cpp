/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
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

void RawlogDataset::initialize(const Yaml& c)
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

    ASSERT_FILE_EXISTS_(rawlog_filename_);

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

    // Starting time:
    if (!replay_started_)
    {
        replay_begin_time_ = mrpt::Clock::now();
        replay_started_    = true;
    }

    // get current replay time:
    const double t = timeDifference(replay_begin_time_, mrpt::Clock::now()) *
                     time_warp_scale_;

    doReadAhead();

    if (read_ahead_.empty())
    {
        MRPT_LOG_THROTTLE_INFO(
            10.0,
            "End of dataset reached! Nothing else to publish (CTRL+C to quit)");
        return;
    }

    // First rawlog timestamp?
    if (rawlog_begin_time_ == INVALID_TIMESTAMP)
        rawlog_begin_time_ = read_ahead_.begin()->first;

    // Publish observations up to current time:
    while (!read_ahead_.empty() &&
           t >= timeDifference(rawlog_begin_time_, read_ahead_.begin()->first))
    {
        //
        CObservation::Ptr obs = read_ahead_.begin()->second;
        this->sendObservationsToFrontEnds(obs);

        read_ahead_.erase(read_ahead_.begin());

        MRPT_LOG_DEBUG_STREAM(
            "Publishing " << obs->GetRuntimeClass()->className
                          << " sensorLabel: " << obs->sensorLabel
                          << " for t=" << t << " observation timestamp="
                          << mrpt::system::dateTimeLocalToString(
                                 obs->timestamp));
    }
    MRPT_END
}

constexpr size_t READ_AHEAD_LEN = 10;

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
    if (read_all_first_)
        doReadAheadFromEntireRawlog();
    else
        doReadAheadFromFile();
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

    const auto obj = rawlog_entire_.getAsGeneric(timestep);

    auto sfRet = mrpt::obs::CSensoryFrame::Create();

    if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj); obs)
    {  // Single observation:

        sfRet->insert(obs);
    }
    else  //
        if (auto sf = std::dynamic_pointer_cast<mrpt::obs::CSensoryFrame>(obj);
            sf)
    {
        sfRet = sf;
    }
    else if (auto acts =
                 std::dynamic_pointer_cast<mrpt::obs::CActionCollection>(obj);
             acts)
    {
        // odometry actions: ignore
    }

    return sfRet;
}
