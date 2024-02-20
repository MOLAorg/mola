/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Dataset_UI.h
 * @brief  Virtual interface for offline dataset sources to have a GUI
 * @author Jose Luis Blanco Claraco
 * @date   Feb 20, 2024
 */
#pragma once

#include <cstdlib>
#include <memory>

namespace mola
{
/** Virtual base for offline dataset sources to have a GUI within MolaViz
 * \ingroup mola_kernel_grp */
class Dataset_UI
{
   public:
    Dataset_UI()          = default;
    virtual ~Dataset_UI() = default;

    /** @name Virtual interface of Dataset_UI
     *{ */

    /** Number of different time steps available to call getObservations() */
    virtual size_t datasetUI_size() const = 0;

    /** Returns the latest requested observation, range [0, datasetSize()] */
    virtual size_t datasetUI_lastQueriedTimestep() const = 0;

    virtual double datasetUI_playback_speed() const       = 0;
    virtual void   datasetUI_playback_speed(double speed) = 0;

    virtual bool datasetUI_paused() const      = 0;
    virtual void datasetUI_paused(bool paused) = 0;

    /** Forces continue replaying in this moment in time */
    virtual void datasetUI_teleport(size_t timestep) = 0;
    /** @} */
};

}  // namespace mola
