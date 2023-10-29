/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   register.cpp
 * @brief  Register MOLA modules in the factory
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2018
 */

/** \defgroup mola_metric_maps_grp mola_metric_maps
 * C++ library: Advanced metric map classes
 *
 */

#include <mola_metric_maps/OccGrid.h>
#include <mrpt/core/initializer.h>
#include <mrpt/rtti/CObject.h>

using namespace mola;

MRPT_INITIALIZER(do_register_mola_metric_maps)
{
    using mrpt::rtti::registerClass;

    // and register RTTI info:
    registerClass(CLASS_ID(mola::OccGrid));
}
