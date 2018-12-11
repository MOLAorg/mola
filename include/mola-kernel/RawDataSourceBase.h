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
#pragma once

#include <mola-kernel/RawDataConsumer.h>
#include <mrpt/core/initializer.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/system/COutputLogger.h>
#include <functional>
#include <memory>

namespace mola
{
/** Virtual base for raw-observation data sources (sensors, dataset parsers,...)
 * \ingroup mola_kernel_grp */
class RawDataSourceBase : public mrpt::system::COutputLogger
{
   public:
    RawDataSourceBase();
    virtual ~RawDataSourceBase() = default;

    using Ptr = std::shared_ptr<RawDataSourceBase>;

    /** Class factory. Register using MOLA_REGISTER_RAWDATASOURCE() */
    static Ptr Factory(const std::string& classname);

    /** @name Virtual interface of any RawDataSource
     *{ */

    /** This should be reimplemented to read all the required parameters */
    virtual void initialize(const std::string& cfg_block);

    /** This will be called in an infinite loop at the sensor execution
     * rate. New observations should be sent to the associated MOLA
     * front-end by calling \a sendObservationsToFrontEnds().
     */
    virtual void spinOnce() = 0;
    /** @} */

    static void registerClass(
        const std::string_view&                 classname,
        std::function<RawDataSourceBase*(void)> func);

    /** Attach this object to a consumer. A shared_ptr is created to keep a
     * reference to the object. */
    void attachToDataConsumer(RawDataConsumer& rdc);

    /** Loads common parameters for all RDS. Called by launcher just before
     * initialize(). */
    void initialize_common(const std::string& cfg_block);

   protected:
    /** Send an observation to the associated target front-ends */
    void sendObservationsToFrontEnds(CObservation::Ptr& obs);

   private:
    /** Target of captured data */
    std::shared_ptr<RawDataConsumer> rdc_;

    struct SensorViewerImpl;
    /** Optional real-time GUI view of sensor data. Viewers indexed by
     * sensor_label */
    std::map<std::string, mrpt::pimpl<SensorViewerImpl>> sensor_preview_gui_;
};

#define MOLA_REGISTER_RAWDATASOURCE(_classname)              \
    MRPT_INITIALIZER(do_register_class)                      \
    {                                                        \
        mola::RawDataSourceBase::registerClass(              \
            #_classname, []() { return new _classname(); }); \
    }

#define ENSURE_YAML_ENTRY_EXISTS(_c, _name) \
    ASSERTMSG_(_c[_name], "Missing YAML required entry: `" _name "`")

}  // namespace mola
