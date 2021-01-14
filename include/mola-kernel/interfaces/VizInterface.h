/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   VizInterface.h
 * @brief  Virtual visualization interface (see MolaViz)
 * @author Jose Luis Blanco Claraco
 * @date   Sep 9, 2020
 */
#pragma once

#include <mrpt/gui/CDisplayWindowGUI.h>  // nanogui

namespace mola
{
/** Virtual visualization interface (see MolaViz)
 *
 * \ingroup mola_kernel_grp */
class VizInterface
{
   public:
    VizInterface()          = default;
    virtual ~VizInterface() = default;

    /** Returned object is owned by the VizInterface, do NOT delete it. */
    virtual void create_subwindow(
        const std::string& title, const std::string& parentWindow = "main") = 0;

   protected:
};

}  // namespace mola
