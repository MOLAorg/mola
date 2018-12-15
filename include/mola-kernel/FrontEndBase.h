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

#include <mola-kernel/ExecutableBase.h>
#include <mola-kernel/RawDataConsumer.h>

namespace mola
{
/** Virtual interface for SLAM front-ends.
 *
 * Instructions for implementing new front-ends:
 * Raw observations arrive via calls to the virtual method `onNewObservation()`,
 * which must be implemented. Minimum time should be spent there, just copy the
 * incomming data (smart pointer). Actual processing can be normally done by any
 * of these two ways:
 * - Wait until `spinOnce()` is called, at the rate especified in the yaml file
 * (default=1 Hz), or
 * - Use your own logic to enque a task into a worker thread pool (preferred).
 *
 * \ingroup mola_kernel_grp */
class FrontEndBase : public ExecutableBase, RawDataConsumer
{
   public:
    FrontEndBase();
    virtual ~FrontEndBase() = default;

    using Ptr = std::shared_ptr<FrontEndBase>;

    /** Class factory. Register using MOLA_REGISTER_FRONTEND() */
    static Ptr Factory(const std::string& classname);

    static void registerClass(
        const std::string_view&            classname,
        std::function<FrontEndBase*(void)> func);

    /** Loads common parameters for all front-ends. Called by launcher just
     * before initialize(). */
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
