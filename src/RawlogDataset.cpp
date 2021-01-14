/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
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

#include <mola-input-rawlog/RawlogDataset.h>
#include <mola-yaml/yaml_helpers.h>
#include <mrpt/core/initializer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/containers/yaml.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(RawlogDataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_RawlogDataset)
{
    MOLA_REGISTER_MODULE(RawlogDataset);
}

RawlogDataset::RawlogDataset() = default;

void RawlogDataset::initialize(const std::string& cfg_block)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    // Mandatory parameters:
    auto c = mrpt::containers::yaml::FromText(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

    YAML_LOAD_MEMBER_REQ(rawlog_filename, std::string);
    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);

    ASSERT_FILE_EXISTS_(rawlog_filename_);
    if (!rawlog_in_.open(rawlog_filename_))
        throw std::runtime_error("Cannot open input rawlog!");

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

void RawlogDataset::doReadAhead()
{
    MRPT_START

    MRPT_TODO("do read ahead in a different thread");

    const size_t READ_AHEAD_LEN = 10;
    auto         rawlog_arch    = mrpt::serialization::archiveFrom(rawlog_in_);

    while (read_ahead_.size() < READ_AHEAD_LEN)
    {
        try
        {
            const auto obj = rawlog_arch.ReadObject();
            auto       obs = mrpt::ptr_cast<mrpt::obs::CObservation>::from(obj);
            if (!obs)
                throw std::runtime_error(
                    "Rawlog file can contain CObservation objects only.");

            read_ahead_[obs->getTimeStamp()] = std::move(obs);
        }
        catch (const mrpt::serialization::CExceptionEOF&)
        {
            return;  // EOF reached.
        }
        catch (...)
        {
            return;  // EOF reached?
        }
    }

    MRPT_END
}
