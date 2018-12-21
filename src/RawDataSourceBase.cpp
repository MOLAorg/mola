/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawDataSourceBase.cpp
 * @brief  Virtual interface for data sources, either real sensors or datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */

#include <mola-kernel/RawDataSourceBase.h>
#include <mola-kernel/WorkerThreadsPool.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace mola;
using mrpt::gui::CDisplayWindow3D;

static WorkerThreadsPool gui_updater_threadpool(1 /* 1 thread */);

struct RawDataSourceBase::SensorViewerImpl
{
    unsigned int          decimation{1}, decim_counter{0};
    std::string           sensor_label;
    std::string           win_pos;  //!< "[x,y,width,height]"
    CDisplayWindow3D::Ptr win;
};

RawDataSourceBase::RawDataSourceBase() = default;

void RawDataSourceBase::initialize_common(const std::string& cfg_block)
{
    MRPT_TRY_START
    auto cfg = YAML::Load(cfg_block);

    // Handle optional sensor preview GUI:
    auto ds_preview = cfg["gui_preview_sensors"];
    if (ds_preview)
    {
        for (auto sensor : ds_preview)
        {
            ENSURE_YAML_ENTRY_EXISTS(sensor, "raw_sensor_label");
            const auto label   = sensor["raw_sensor_label"].as<std::string>();
            const auto decim   = sensor["decimation"].as<unsigned int>(1);
            const auto win_pos = sensor["win_pos"].as<std::string>("");

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
    MRPT_TRY_END
}

void RawDataSourceBase::sendObservationsToFrontEnds(
    mrpt::obs::CObservation::Ptr& obs)
{
    MRPT_TRY_START

    ASSERT_(obs);
    // Just forward the data to my associated consumer:
    if (rdc_) { rdc_->onNewObservation(obs); }
    else
    {
        MRPT_LOG_WARN(
            "[sendObservationsToFrontEnds] Dropping observation: no consumer "
            "is attached.");
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
                using namespace mrpt::opengl;

                // GUI update decimation:
                if (++sv->decim_counter < sv->decimation) return;
                sv->decim_counter = 0;

                // Create GUI upon first call:
                COpenGLScene::Ptr scene;
                if (!sv->win)
                {
                    sv->win = CDisplayWindow3D::Create(sv->sensor_label);

                    // Replace and resize, if user provided "win_pos":
                    if (!sv->win_pos.empty())
                    {
                        int                x = 0, y = 0, w = 400, h = 300;
                        std::istringstream ss(sv->win_pos);
                        // parse: "x y w h"
                        if ((ss >> x) && (ss >> y) && (ss >> w) && (ss >> h))
                        {
                            sv->win->setPos(x, y);
                            sv->win->resize(w, h);
                        }
                    }

                    mrpt::gui::CDisplayWindow3DLocker lck(*sv->win, scene);
                    scene->insert(stock_objects::CornerXYZSimple(1.0f, 4.0f));
                    scene->insert(CGridPlaneXY::Create());
                    auto o = CPointCloudColoured::Create();
                    o->setName("pointcloud");
                    scene->insert(o);
                }

                // Update rendered object:
                MRPT_TODO("Make new registry of renderizable objects,...");

                // temp code ----
                auto o_velo =
                    mrpt::ptr_cast<mrpt::obs::CObservationPointCloud>::from(
                        obs);
                if (o_velo && o_velo->pointcloud)
                {
                    mrpt::gui::CDisplayWindow3DLocker lck(*sv->win, scene);
                    auto o     = scene->getByName("pointcloud");
                    auto gl_pt = mrpt::ptr_cast<CPointCloudColoured>::from(o);

                    // o_velo
                    gl_pt->loadFromPointsMap(o_velo->pointcloud.get());
                }

                auto o_img =
                    mrpt::ptr_cast<mrpt::obs::CObservationImage>::from(obs);
                if (o_img) { sv->win->setImageView(o_img->image); }
                // temp code ----

                // Force repaint:
                sv->win->repaint();
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM(
                    "Error in GUI updater worker thread:\n"
                    << mrpt::exception_to_str(e));
            }
        };

        gui_updater_threadpool.enqueue(func);
    }

    MRPT_TRY_END
}

void RawDataSourceBase::attachToDataConsumer(RawDataConsumer& rdc)
{
    MRPT_TODO("fix shared_from_this()");
    rdc_ = &rdc;  // rdc.getAsPtr();
}
