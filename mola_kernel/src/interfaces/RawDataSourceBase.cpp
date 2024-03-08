/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawDataSourceBase.cpp
 * @brief  Virtual interface for data sources, either real sensors or datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */

#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_VIRTUAL_MRPT_OBJECT(RawDataSourceBase, ExecutableBase, mola)

struct RawDataSourceBase::SensorViewerImpl
{
    unsigned int decimation{1}, decim_counter{0};
    std::string  sensor_label;
    std::string  win_pos;  //!< "[x,y,width,height]"

    nanogui::Window* win = nullptr;
};

RawDataSourceBase::RawDataSourceBase() = default;

RawDataSourceBase::~RawDataSourceBase()
{
    if (!gui_updater_threadpool_.pendingTasks()) return;

    MRPT_LOG_INFO_STREAM(
        "Dtor called while gui_updater_threadpool_ still has "
        << gui_updater_threadpool_.pendingTasks() << " tasks. Aborting them.");
    gui_updater_threadpool_.clear();
}

void RawDataSourceBase::initialize(const Yaml& cfg)
{
    MRPT_TRY_START

    // Handle optional sensor preview GUI:
    if (cfg.has("gui_preview_sensors"))
    {
        auto ds_preview = cfg["gui_preview_sensors"];
        for (const auto& s : ds_preview.asSequence())
        {
            const auto sensor = mrpt::containers::yaml(s);
            const auto label  = sensor["raw_sensor_label"].as<std::string>();
            const auto decim =
                sensor.getOrDefault<unsigned int>("decimation", 1);
            const auto win_pos =
                sensor.getOrDefault<std::string>("win_pos", "");

            ASSERTMSG_(
                sensor_preview_gui_.find(label) == sensor_preview_gui_.end(),
                mrpt::format(
                    "Duplicated sensor label in `gui_preview_sensors`: `%s`",
                    label.c_str()));
            auto& sv = sensor_preview_gui_[label] =
                mrpt::make_impl<RawDataSourceBase::SensorViewerImpl>();

            sv->decimation   = decim;
            sv->sensor_label = label;
            sv->win_pos      = win_pos;
            // sv->win: Create a window when the sensor actually publishes.
        }
    }

    // Handle optional export to rawlog file
    if (auto fil = cfg.getOrDefault<std::string>("export_to_rawlog", "");
        !fil.empty())
    {
        MRPT_LOG_INFO_STREAM("Exporting to rawlog file: " << fil);
        if (!export_to_rawlog_out_.open(fil))
            THROW_EXCEPTION_FMT("Error opening for write: `%s`", fil.c_str());
    }

    // Optional force load lazy-load observations:
    YAML_LOAD_MEMBER_OPT(force_load_lazy_load, bool);
    YAML_LOAD_MEMBER_OPT(quit_mola_app_on_dataset_end, bool);

    // children params:
    this->initialize_rds(cfg);

    MRPT_TRY_END
}

void RawDataSourceBase::sendObservationsToFrontEnds(
    const mrpt::obs::CObservation::Ptr& obs)
{
    MRPT_TRY_START

    ASSERT_(obs);
    // Forward the data to my associated consumer:
    if (!rdc_.empty())
    {
        // prepare observation before processing it:
        prepareObservationBeforeFrontEnds(obs);

        // Forward data:
        for (auto& subscriber : rdc_) subscriber->onNewObservation(obs);
    }
    else
    {
        // double POD zero initialized by default:
        thread_local std::map<std::string, double> lastWarnPerLabel;

        double&      lastWarn = lastWarnPerLabel[obs->sensorLabel];
        const double now      = mrpt::Clock::nowDouble();
        if (now - lastWarn >= 10.0)
        {
            lastWarn = now;
            MRPT_LOG_WARN_FMT(
                "[sendObservationsToFrontEnds] Dropping observation '%s': no "
                "consumer is attached.",
                obs->sensorLabel.c_str());
        }
    }

    // if we are storing data to .rawlog, enqueue it in the specific worker
    // thread:
    if (export_to_rawlog_out_.is_open())
    {
        auto fut = worker_pool_export_rawlog_.enqueue(
            [this](const mrpt::obs::CObservation::Ptr& o) {
                if (!o) return;
                auto a = mrpt::serialization::archiveFrom(
                    this->export_to_rawlog_out_);
                a << o;
            },
            obs);
    }

    // Send this observation for GUI preview, if enabled:
    const auto it_sen_gui = sensor_preview_gui_.find(obs->sensorLabel);
    if (it_sen_gui != sensor_preview_gui_.end())
    {
        // Create and enque the GUI update function, as a lambda:
        RawDataSourceBase::SensorViewerImpl* sv = &(*it_sen_gui->second);

        auto func = [this, sv, obs]() {
            try
            {
                ProfilerEntry pe(profiler_, "send to viz lambda");

                using namespace mrpt::opengl;

                // GUI update decimation:
                if (++sv->decim_counter < sv->decimation) return;
                sv->decim_counter = 0;

                // Create subwindow now:
                auto vizMods = this->findService<mola::VizInterface>();
                ASSERTMSG_(
                    !vizMods.empty(),
                    "Could not find a running MolaViz module");

                auto viz =
                    std::dynamic_pointer_cast<VizInterface>(vizMods.at(0));

                // Create GUI upon first call:
                if (!sv->win)
                {
                    // get std::future and wait for it:
                    auto fut = viz->create_subwindow(sv->sensor_label);
                    sv->win  = fut.get();

                    // Replace and resize, if user provided "win_pos":
                    if (sv->win && !sv->win_pos.empty())
                    {
                        int                x = 0, y = 0, w = 400, h = 300;
                        std::istringstream ss(sv->win_pos);
                        // parse: "x y w h"
                        if ((ss >> x) && (ss >> y) && (ss >> w) && (ss >> h))
                        {
                            sv->win->setPosition({x, y});
                            sv->win->setSize({w, h});
                        }
                    }
                }

                // Update the GUI:
                // (We don't need to wait for the future result, just move on)
                // auto fut =
                viz->subwindow_update_visualization(obs, sv->sensor_label);
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM(
                    "Error in GUI updater worker thread:\n"
                    << mrpt::exception_to_str(e));
            }
        };

        auto fut = gui_updater_threadpool_.enqueue(func);
    }

    MRPT_TRY_END
}

void RawDataSourceBase::attachToDataConsumer(RawDataConsumer& rdc)
{
    // TODO(jlbc) fix shared_from_this()??
    rdc_.push_back(&rdc);  // rdc.getAsPtr();
}

void RawDataSourceBase::prepareObservationBeforeFrontEnds(
    const CObservation::Ptr& obs) const
{
    MRPT_TRY_START

    using namespace mrpt::obs;

    // These operations are optional:
    if (!force_load_lazy_load_) { obs->load(); }

    // Sensor-specific:
    if (auto o_velo = mrpt::ptr_cast<CObservationVelodyneScan>::from(obs);
        o_velo)
    {
        if (!o_velo->point_cloud.size())
        {
            // Generate point timestamps & RING ID:
            mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters
                p;
            p.generatePerPointTimestamp = true;
            p.generatePointsForLaserID  = true;
            o_velo->generatePointCloud(p);
        }

        const auto& pc = o_velo->point_cloud;
        ASSERT_EQUAL_(pc.x.size(), pc.y.size());
        ASSERT_EQUAL_(pc.x.size(), pc.z.size());
        ASSERT_EQUAL_(pc.x.size(), pc.intensity.size());
        ASSERT_EQUAL_(pc.x.size(), pc.laser_id.size());
        ASSERT_EQUAL_(pc.x.size(), pc.timestamp.size());
    }
    MRPT_TRY_END
}

void RawDataSourceBase::onDatasetPlaybackEnds()
{
    if (!quit_mola_app_on_dataset_end_) return;  // do nothing

    this->requestShutdown();  // Quit mola app
}
