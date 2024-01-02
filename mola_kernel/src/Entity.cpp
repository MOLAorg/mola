/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Entity.cpp
 * @brief  Utilities for entities variant types
 * @author Jose Luis Blanco Claraco
 * @date   Jan 28, 2019
 */

#include <mola_kernel/Entity.h>
#include <mola_kernel/variant_helper.h>

using namespace mola;

EntityBase& mola::entity_get_base(Entity& e)
{
    EntityBase* ret = nullptr;
    std::visit(
        overloaded{
            [&ret](EntityBase& b) { ret = &b; },
            [&ret](EntityOther& o) {
                ASSERT_(o);
                ret = o.get();
            },
            [](std::monostate) {}},
        e);

    if (!ret) THROW_EXCEPTION("entity_get_base(): Empty variant.");

    return *ret;
}

const EntityBase& mola::entity_get_base(const Entity& e)
{
    const EntityBase* ret = nullptr;
    std::visit(
        overloaded{
            [&ret](const EntityBase& b) { ret = &b; },
            [&ret](const EntityOther& o) {
                ASSERT_(o);
                ret = o.get();
            },
            [](std::monostate) {}},
        e);

    if (!ret) THROW_EXCEPTION("entity_get_base(): Empty variant.");
    return *ret;
}

void mola::entity_update_pose(mola::Entity& e, const mrpt::math::TPose3D& p)
{
    MRPT_TRY_START

    std::visit(
        overloaded{
            [&](RefPose3&) {
                ASSERTMSG_(
                    p == mrpt::math::TPose3D::Identity(),
                    "RefPose3 cannot be assigned a pose != Identity()");
            },
            [&](RelDynPose3KF& ee) { ee.relpose_value = p; },
            [&](RelPose3& ee) { ee.relpose_value = p; },
            [&](RelPose3KF& ee) { ee.relpose_value = p; },
            []([[maybe_unused]] auto ee) {
                throw std::runtime_error(
                    mrpt::format("[updateEntityPose] Unknown Entity type!"));
            },
        },
        e);
    MRPT_TRY_END
}

void mola::entity_update_vel(mola::Entity& e, const std::array<double, 3>& v)
{
    MRPT_TRY_START

    std::visit(
        overloaded{
            [&](RefPose3&) {
#if 0
                if (v[0] * v[0] + v[1] * v[1] + v[2] * v[2] > 1e-10)
                    THROW_EXCEPTION(
                        "RefPose3 cannot be assigned a velocity!=0");
#endif
            },
            [&](RelDynPose3KF& ee) {
                ee.twist_value.vx = v[0];
                ee.twist_value.vy = v[1];
                ee.twist_value.vz = v[2];
            },
            [&](RelPose3&) {},
            [&](RelPose3KF&) {},
            []([[maybe_unused]] auto ee) {
                throw std::runtime_error(
                    mrpt::format("[updateEntityPose] Unknown Entity type!"));
            },
        },
        e);
    MRPT_TRY_END
}

mrpt::math::TTwist3D mola::entity_get_twist(const mola::Entity& e)
{
    MRPT_TRY_START

    mrpt::math::TTwist3D ret;
    std::visit(
        overloaded{
            [&](const RefPose3&) {},
            [&](const RelDynPose3KF& ee) { ret = ee.twist_value; },
            [&](const RelPose3&) {},
            [&](const RelPose3KF&) {},
            []([[maybe_unused]] auto ee) {
                throw std::runtime_error(
                    mrpt::format("[updateEntityPose] Unknown Entity type!"));
            },
        },
        e);
    return ret;
    MRPT_TRY_END
}

mrpt::math::TPose3D mola::entity_get_pose(const mola::Entity& e)
{
    MRPT_TRY_START
    //
    mrpt::math::TPose3D ret;
    std::visit(
        overloaded{
            [&](const RefPose3&) { ret = mrpt::math::TPose3D::Identity(); },
            [&](const RelDynPose3KF& ee) { ret = ee.relpose_value; },
            [&](const RelPose3& ee) { ret = ee.relpose_value; },
            [&](const RelPose3KF& ee) { ret = ee.relpose_value; },
            []([[maybe_unused]] auto ee) {
                throw std::runtime_error(
                    mrpt::format("[getEntityPose] Unknown Entity type!"));
            },
        },
        e);

    return ret;
    MRPT_TRY_END
}

mrpt::Clock::time_point mola::entity_get_timestamp(const mola::Entity& e)
{
    MRPT_TRY_START
    //
    mrpt::Clock::time_point ret{};
    std::visit(
        overloaded{
            [&](const EntityBase& ee) { ret = ee.timestamp_; },
            [&](const EntityOther& ee) { ret = ee->timestamp_; },
            [](std::monostate) {
                throw std::runtime_error(
                    mrpt::format("[getEntityTimeStamp] Unknown Entity type!"));
            },
        },
        e);

    ASSERT_(ret != INVALID_TIMESTAMP);

    return ret;
    MRPT_TRY_END
}
