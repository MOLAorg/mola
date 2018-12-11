/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FrontEndBase.h
 * @brief  Virtual interface for SLAM front-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */
#pragma once

#include <mola-kernel/RawDataConsumer.h>
#include <mrpt/system/COutputLogger.h>

namespace mola
{
/** Virtual interface for SLAM front-ends.
 *
 * \ingroup mola_kernel_grp */
class FrontEndBase : public mrpt::system::COutputLogger, RawDataConsumer
{
   public:
    FrontEndBase();
    virtual ~FrontEndBase() = default;

    using Ptr = std::shared_ptr<FrontEndBase>;

    /** Class factory. Register using MOLA_REGISTER_FRONTEND() */
    static Ptr Factory(const std::string& classname);

    /** @name Virtual interface of any SLAM Front-End
     *{ */

    /** This should be reimplemented to read all the required parameters */
    virtual void initialize(const std::string& cfg_block);

    /** This will be called in an infinite loop at the execution rate specified
     * in the launch file. Output to the localization/SLAM back-end should to
     * sent out via...
     */
    virtual void spinOnce() = 0;
    /** @} */

    static void registerClass(
        const std::string_view&            classname,
        std::function<FrontEndBase*(void)> func);

    /** Loads common parameters for all RDS. Called by launcher just before
     * initialize(). */
    void initialize_common(const std::string& cfg_block);

   private:
};

#define MOLA_REGISTER_FRONTEND(_classname)                   \
    MRPT_INITIALIZER(do_register_class)                      \
    {                                                        \
        mola::FrontEndBase::registerClass(                   \
            #_classname, []() { return new _classname(); }); \
    }

}  // namespace mola
