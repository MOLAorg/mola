/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawlogDataset.h
 * @brief  RawDataSource for datasets in MRPT rawlog format
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2019
 */
#pragma once

#include <mola-kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CArchive.h>

namespace mola
{
namespace sensor_rawlog_dataset
{
/**
 *
 * \ingroup mola_sensor_rawlog_grp */
class RawlogDataset : public RawDataSourceBase
{
   public:
    RawlogDataset();
    ~RawlogDataset() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

   private:
    std::string                  rawlog_filename_;
    mrpt::io::CFileGZInputStream rawlog_in_;

    mrpt::Clock::time_point replay_begin_time_{};
    mrpt::Clock::time_point rawlog_begin_time_{INVALID_TIMESTAMP};
    bool                    replay_started_{false};
    double                  time_warp_scale_{1.0};

    void doReadAhead();
    std::map<mrpt::Clock::time_point, mrpt::obs::CObservation::Ptr> read_ahead_;
};

}  // namespace sensor_rawlog_dataset
}  // namespace mola
