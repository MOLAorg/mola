/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EntityBase.cpp
 * @brief  Base class for all "entities" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */

#include <mola-kernel/Entity.h>
#include <mola-kernel/entities/EntityBase.h>
#include <mola-kernel/variant_helper.h>

// TODO: make serializable

using namespace mola;

EntityBase::~EntityBase() = default;

EntityBase& mola::entity_get_base(Entity& e)
{
    EntityBase* ret = nullptr;
    std::visit(
        overloaded{[&ret](EntityBase& b) { ret = &b; },
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
        overloaded{[&ret](const EntityBase& b) { ret = &b; },
                   [&ret](const EntityOther& o) {
                       ASSERT_(o);
                       ret = o.get();
                   },
                   [](std::monostate) {}},
        e);

    if (!ret) THROW_EXCEPTION("entity_get_base(): Empty variant.");
    return *ret;
}
