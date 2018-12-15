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
#include <memory>
#include <string>

namespace mola
{
/**
 * \ingroup mola_kernel_grp */
class ExecutableBase : public mrpt::system::COutputLogger
{
   public:
    ExecutableBase();
    virtual ~ExecutableBase();

    using Ptr = std::shared_ptr<ExecutableBase>;

    /** @name Virtual interface of any ExecutableBase
     *{ */
    virtual void initialize_common(const std::string& cfg_block) = 0;
    virtual void initialize(const std::string& cfg_block);
    virtual void spinOnce() = 0;
    /** @} */
};

}  // namespace mola
