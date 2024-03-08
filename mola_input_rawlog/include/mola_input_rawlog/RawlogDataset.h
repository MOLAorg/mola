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

#include <mola_kernel/interfaces/Dataset_UI.h>
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
class RawlogDataset : public RawDataSourceBase,
                      public OfflineDatasetSource,
                      public Dataset_UI
{
    DEFINE_MRPT_OBJECT(RawlogDataset, mola)

   public:
    RawlogDataset();
    ~RawlogDataset() override = default;

    void spinOnce() override;

    // See docs in base class:
    size_t datasetSize() const override;

    mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(
        size_t timestep) const override;

    // Virtual interface of Dataset_UI (see docs in derived class)
    size_t datasetUI_size() const override
    {
        if (read_all_first_)
            return datasetSize();
        else
            return 10000000;  // we just don't know...
    }
    size_t datasetUI_lastQueriedTimestep() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return last_used_tim_index_;
    }
    double datasetUI_playback_speed() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return time_warp_scale_;
    }
    void datasetUI_playback_speed(double speed) override
    {
        auto lck         = mrpt::lockHelper(dataset_ui_mtx_);
        time_warp_scale_ = speed;
    }
    bool datasetUI_paused() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return paused_;
    }
    void datasetUI_paused(bool paused) override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        paused_  = paused;
    }
    void datasetUI_teleport(size_t timestep) override
    {
        auto lck       = mrpt::lockHelper(dataset_ui_mtx_);
        teleport_here_ = timestep;
    }

   protected:
    // See docs in base class
    void initialize_rds(const Yaml& cfg) override;

   private:
    std::string                  rawlog_filename_;
    mrpt::io::CFileGZInputStream rawlog_in_;
    mrpt::obs::CRawlog           rawlog_entire_;  //!< if read_all_first_=true
    size_t                       rawlog_next_idx_ = 0;

    mrpt::Clock::time_point rawlog_begin_time_{INVALID_TIMESTAMP};
    bool                    read_all_first_ = true;

    std::optional<mrpt::Clock::time_point> last_play_wallclock_time_;
    double                                 last_dataset_time_ = 0;

    void doReadAhead();
    void doReadAheadFromFile();
    void doReadAheadFromEntireRawlog();

    std::multimap<mrpt::Clock::time_point, mrpt::obs::CObservation::Ptr>
        read_ahead_;

    void autoUnloadOldEntries() const;

    mutable std::multimap<mrpt::Clock::time_point, mrpt::obs::CObservation::Ptr>
        unload_queue_;

    mutable timestep_t    last_used_tim_index_ = 0;
    bool                  paused_              = false;
    double                time_warp_scale_     = 1.0;
    std::optional<size_t> teleport_here_;
    mutable std::mutex    dataset_ui_mtx_;
};

}  // namespace mola
