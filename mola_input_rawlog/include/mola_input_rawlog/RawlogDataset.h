/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawlogDataset.h
 * @brief  RawDataSource for datasets in MRPT rawlog format
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2019
 */
#pragma once

#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/serialization/CArchive.h>

namespace mola
{
/** RawDataSource for datasets in MRPT rawlog format
 *
 * \ingroup mola_input_rawlog_grp */
class RawlogDataset : public RawDataSourceBase, public OfflineDatasetSource
{
    DEFINE_MRPT_OBJECT(RawlogDataset, mola)

   public:
    RawlogDataset();
    ~RawlogDataset() override = default;

    // See docs in base class
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;

    // See docs in base class:
    size_t datasetSize() const override;

    mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(
        size_t timestep) const override;

   private:
    std::string                  rawlog_filename_;
    mrpt::io::CFileGZInputStream rawlog_in_;
    mrpt::obs::CRawlog           rawlog_entire_;  //!< if read_all_first_=true
    size_t                       rawlog_next_idx_ = 0;

    mrpt::Clock::time_point replay_begin_time_{};
    mrpt::Clock::time_point rawlog_begin_time_{INVALID_TIMESTAMP};
    bool                    replay_started_{false};
    double                  time_warp_scale_{1.0};
    bool                    read_all_first_ = true;

    void doReadAhead();
    void doReadAheadFromFile();
    void doReadAheadFromEntireRawlog();

    std::multimap<mrpt::Clock::time_point, mrpt::obs::CObservation::Ptr>
        read_ahead_;
};

}  // namespace mola
