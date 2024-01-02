/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Entity.h
 * @brief  Variant type for "entities" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola_kernel/entities/entities-common.h>
#include <mrpt/math/TPose3D.h>

#include <array>
#include <variant>

namespace mola
{
/** Variant type for "entities" in the world model.
 * It's guaranteed that the type is either:
 * - `std::monostate`: Not initialized, or
 * - A class derived from `EntityBase` (stored by value), or
 * - `EntityOther`, which is a wrapper around `EntityBase::Ptr` (an object
 * allocated in the heap).
 *
 * In this way, `Entity` can be handled as a polymorphic class, without the cost
 * of dynamic memory allocation for the most-common classes, while still
 * allowing using any user-derived class via dynamic memory.
 *
 * \ingroup mola_kernel_grp
 */
using Entity = std::variant<
    std::monostate, RefPose3, RelPose3, RelPose3KF, RelDynPose3KF,
    LandmarkPoint3, EntityOther>;

/** \addtogroup entity_utils Entity handling utilities
 *  \ingroup mola_kernel_grp
 *  @{ */

/** Return a reference to the EntityBase associated to the variant e */
EntityBase& entity_get_base(Entity& e);
/** \overload */
const EntityBase& entity_get_base(const Entity& e);

mrpt::math::TPose3D  entity_get_pose(const Entity& e);
mrpt::math::TTwist3D entity_get_twist(const mola::Entity& e);
void entity_update_pose(Entity& e, const mrpt::math::TPose3D& p);
void entity_update_vel(Entity& e, const std::array<double, 3>& v);
mrpt::Clock::time_point entity_get_timestamp(const Entity& e);

/** @} */

}  // namespace mola
