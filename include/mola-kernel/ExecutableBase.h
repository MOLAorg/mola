/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ExecutableBase.h
 * @brief  Virtual interface for objects that can be run into MOLA
 * @author Jose Luis Blanco Claraco
 * @date   Dec 14, 2018
 */
#pragma once

#include <mrpt/system/COutputLogger.h>
#include <functional>
#include <memory>
#include <string>

namespace mola
{
/**
 * \ingroup mola_kernel_grp */
class ExecutableBase : public mrpt::system::COutputLogger,
                       std::enable_shared_from_this<ExecutableBase>
{
   public:
    ExecutableBase();
    virtual ~ExecutableBase();

    using Ptr = std::shared_ptr<ExecutableBase>;

    /** Get as shared_ptr via enable_shared_from_this<> */
    Ptr getAsPtr() { return shared_from_this(); }

    /** @name Virtual interface of any ExecutableBase
     *{ */
    virtual void initialize_common(const std::string& cfg_block) = 0;
    virtual void initialize(const std::string& cfg_block);
    virtual void spinOnce() = 0;
    /** @} */

    /** A name server function to search for other ExecutableBase objects in my
     * running system. Empty during ctor, should be usable from
     * initialize_common() and initialize() */
    std::function<Ptr(const std::string&)> nameServer_;
};

}  // namespace mola
