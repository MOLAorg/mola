/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawDataSourceBase.cpp
 * @brief  Virtual interface for data sources, either real sensors or datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */

#include <mola-kernel/interfaces/RawDataSourceBase.h>
#include <mola-kernel/interfaces/VizInterface.h>
#include <mola-yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

using namespace mola;

MRPT_TODO("Move sensor-specific rendering to MolaViz module");

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

void RawDataSourceBase::initialize_common(const std::string& cfg_block)
{
    MRPT_TRY_START
    const auto cfg = mrpt::containers::yaml::FromText(cfg_block);

    // Handle optional sensor preview GUI:
    if (cfg.has("gui_preview_sensors"))
    {
        auto ds_preview = cfg["gui_preview_sensors"];
        for (auto s : ds_preview.asSequence())
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

    MRPT_TRY_END
}

void RawDataSourceBase::sendObservationsToFrontEnds(
    mrpt::obs::CObservation::Ptr& obs)
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
        MRPT_LOG_THROTTLE_WARN_FMT(
            10.0,
            "[sendObservationsToFrontEnds] Dropping observation '%s': no "
            "consumer is attached.",
            obs->sensorLabel.c_str());
    }

    // if we are storing data to .rawlog, enqueue it in the specific worker
    // thread:
    if (export_to_rawlog_out_.is_open())
    {
        worker_pool_export_rawlog_.enqueue(
            [this](mrpt::obs::CObservation::Ptr& o) {
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

#if 0
                     mrpt::gui::CDisplayWindow3DLocker lck(*sv->win, scene);
                    scene->insert(stock_objects::CornerXYZSimple(1.0f, 4.0f));
                    auto o = CSetOfObjects::Create();
                    o->setName("pointcloud");
                    scene->insert(o);
#endif
                }

                // Update the GUI:
                // (We don't need to wait for the future result, just move on)
                // auto fut =
                viz->subwindow_update_visualization(obs, sv->sensor_label);

#if 0
                // temp code ----
                if (auto o_points =
                        mrpt::ptr_cast<mrpt::obs::CObservationPointCloud>::from(
                            obs);
                    o_points && o_points->pointcloud)
                {
                    mrpt::gui::CDisplayWindow3DLocker lck(*sv->win, scene);
                    auto o     = scene->getByName("pointcloud");
                    auto gl_pt = mrpt::ptr_cast<CSetOfObjects>::from(o);

                    // o_points
                    gl_pt->clear();
                    o_points->pointcloud->renderOptions.point_size = 1.0f;
                    o_points->pointcloud->getAs3DObject(gl_pt);
                    gl_pt->setPose(o_points->sensorPose);
                }

                if (auto o_velo = mrpt::ptr_cast<
                        mrpt::obs::CObservationVelodyneScan>::from(obs);
                    o_velo)
                {
                    mrpt::gui::CDisplayWindow3DLocker lck(*sv->win, scene);
                    auto o     = scene->getByName("pointcloud");
                    auto gl_pt = mrpt::ptr_cast<CSetOfObjects>::from(o);

                    // o_velo
                    gl_pt->clear();
                    mrpt::maps::CPointsMapXYZI xyzi;
                    xyzi.renderOptions.point_size = 1.0f;
                    if (o_velo->point_cloud.size() == 0)
                        o_velo->generatePointCloud();
                    xyzi.loadFromVelodyneScan(*o_velo);
                    xyzi.getAs3DObject(gl_pt);
                }

                if (auto o_2dscan = mrpt::ptr_cast<
                        mrpt::obs::CObservation2DRangeScan>::from(obs);
                    o_2dscan)
                {
                    mrpt::gui::CDisplayWindow3DLocker lck(*sv->win, scene);
                    auto o     = scene->getByName("pointcloud");
                    auto gl_pt = mrpt::ptr_cast<CSetOfObjects>::from(o);

                    // o_2dscan
                    gl_pt->clear();
                    auto gl_scan = mrpt::opengl::CPlanarLaserScan::Create();
                    gl_scan->setScan(*o_2dscan);
                    gl_pt->insert(gl_scan);
                }

                auto o_img =
                    mrpt::ptr_cast<mrpt::obs::CObservationImage>::from(obs);
                if (o_img) { sv->win->setImageView(o_img->image); }
                // temp code ----

                // Force repaint:
                sv->win->repaint();
#endif
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM(
                    "Error in GUI updater worker thread:\n"
                    << mrpt::exception_to_str(e));
            }
        };

        gui_updater_threadpool_.enqueue(func);
    }

    MRPT_TRY_END
}

void RawDataSourceBase::attachToDataConsumer(RawDataConsumer& rdc)
{
    MRPT_TODO("fix shared_from_this()");
    rdc_.push_back(&rdc);  // rdc.getAsPtr();
}

void RawDataSourceBase::prepareObservationBeforeFrontEnds(
    CObservation::Ptr& obs) const
{
    MRPT_TRY_START

    using namespace mrpt::obs;

    MRPT_TODO("Make these operations optional and disabled by default");

    // for delay-load data:
    obs->load();

    // Sensor-specific:
    if (auto o_velo = mrpt::ptr_cast<CObservationVelodyneScan>::from(obs);
        o_velo)
    {
        if (!o_velo->point_cloud.size()) o_velo->generatePointCloud();
        const auto& pc = o_velo->point_cloud;
        ASSERT_EQUAL_(pc.x.size(), pc.y.size());
        ASSERT_EQUAL_(pc.x.size(), pc.z.size());
    }
    MRPT_TRY_END
}
