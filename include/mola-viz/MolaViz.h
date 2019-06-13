/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MolaViz.h
 * @brief  Main C++ class for MOLA GUI
 * @author Jose Luis Blanco Claraco
 * @date   May  11, 2019
 */
#pragma once

#include <mola-kernel/interfaces/ExecutableBase.h>
#include <mola-kernel/WorkerThreadsPool.h>
#include <mrpt/obs/CObservationIMU.h>

namespace mola
{
/**
 *
 */
class MolaViz : public ExecutableBase
{
   public:
    MolaViz();
    virtual ~MolaViz() override = default;

    // See docs in base class
    void initialize_common(const std::string&) override {}
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

   private:
    // mola::WorkerThreadsPool worker_pool_{1};
};

}  // namespace mola
