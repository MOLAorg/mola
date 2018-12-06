/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawDataSourceBase.h
 * @brief  Virtual interface for data sources, either real sensors or datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */

#include <mola-kernel/RawDataSourceBase.h>
#include <mola-kernel/WorkerThreadsPool.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace mola;
using mrpt::gui::CDisplayWindow3D;

// Class factory:
static std::map<std::string, std::function<RawDataSourceBase*(void)>> registry;

static WorkerThreadsPool gui_updater_threadpool(1 /* 1 thread */);

struct RawDataSourceBase::SensorViewerImpl
{
    unsigned int          decimation{1}, decim_counter{0};
    std::string           sensor_label;
    CDisplayWindow3D::Ptr win;
};

RawDataSourceBase::RawDataSourceBase()
    : mrpt::system::COutputLogger("RawDataSourceBase")
{
}

RawDataSourceBase::Ptr RawDataSourceBase::Factory(const std::string& name)
{
    const auto f = registry.find(name);
    if (f == registry.end())
        THROW_EXCEPTION_FMT(
            "[RawDataSourceBase::Factory] Request for unregistered class: `%s`",
            name.c_str());
    return Ptr((f->second)());
}

void RawDataSourceBase::registerClass(
    const std::string_view&                 classname,
    std::function<RawDataSourceBase*(void)> func)
{
    registry.emplace(classname, func);
}

/** This should be reimplemented to read all the required parameters */
void RawDataSourceBase::initialize(const std::string& cfg_block)
{
    MRPT_LOG_WARN_STREAM(
        "`initialize()` not reimplemented by derived class. "
        "Ignoring YAML config block:\n"
        << cfg_block);
}

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
            const auto label = sensor["raw_sensor_label"].as<std::string>();
            const auto decim = sensor["decimation"].as<unsigned int>(1);

            ASSERTMSG_(
                sensor_preview_gui_.find(label) == sensor_preview_gui_.end(),
                mrpt::format(
                    "Duplicated sensor label in `gui_preview_sensors`: `%s`",
                    label.c_str()));
            auto& sv = sensor_preview_gui_[label] =
                mrpt::make_impl<RawDataSourceBase::SensorViewerImpl>();

            sv->decimation   = decim;
            sv->sensor_label = label;
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
                // Create GUI upon first call:
                COpenGLScene::Ptr scene;
                if (!sv->win)
                {
                    sv->win = CDisplayWindow3D::Create(sv->sensor_label);
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
                    mrpt::ptr_cast<mrpt::obs::CObservationVelodyneScan>::from(
                        obs);
                if (o_velo)
                {
                    mrpt::gui::CDisplayWindow3DLocker lck(*sv->win, scene);
                    auto o     = scene->getByName("pointcloud");
                    auto gl_pt = mrpt::ptr_cast<CPointCloudColoured>::from(o);

                    // o_velo
                    gl_pt->clear();
                    const auto N = o_velo->point_cloud.size();
                    gl_pt->reserve(N);
                    for (std::size_t i = 0; i < N; i++)
                    {
                        // XYZ-RGB:
                        gl_pt->push_back(
                            o_velo->point_cloud.x[i], o_velo->point_cloud.y[i],
                            o_velo->point_cloud.z[i],
                            o_velo->point_cloud.intensity[i],
                            o_velo->point_cloud.intensity[i],
                            o_velo->point_cloud.intensity[i]);
                    }
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
    rdc_ = rdc.getAsPtr();
}
