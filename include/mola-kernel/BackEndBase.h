/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   BackEndBase.h
 * @brief  Virtual interface for SLAM back-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 21, 2018
 */
#pragma once

#include <mola-kernel/ExecutableBase.h>

namespace mola
{
/** Virtual interface for SLAM back-ends.
 *
 * \ingroup mola_kernel_grp */
class BackEndBase : public ExecutableBase
{
   public:
    BackEndBase();
    virtual ~BackEndBase() = default;

    using Ptr = std::shared_ptr<BackEndBase>;

    /** Loads common parameters for all back-ends. Called by launcher just
     * before initialize(). */
    void initialize_common(const std::string& cfg_block);
};

}  // namespace mola
