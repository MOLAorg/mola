/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WorldModel.h
 * @brief  The main class for a "map" or "world model".
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola-kernel/EntityBase.h>
#include <mola-kernel/ExecutableBase.h>
#include <mola-kernel/Keyframe.h>
#include <map>

namespace mola
{
/** Unique ID for each Entity in a WorldModel. \ingroup mola_kernel_grp */
using id_t = std::uint64_t;
/** A numeric value for invalid IDs. \ingroup mola_kernel_grp */
constexpr id_t INVALID_ID = std::numeric_limits<id_t>::max();

struct EntitiesContainer;

/** The main class for a "map" or "world model".
 *
 * \ingroup mola_kernel_grp
 */
class WorldModel : public ExecutableBase
{
   public:
    // Virtual interface of any ExecutableBase. See base docs:
    void initialize_common(const std::string&) override {}
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override {}

    using Ptr = std::shared_ptr<WorldModel>;

    /** @name Main data fields
     * @{ */

    /** All keyframes, relative and absolute poses, calibration parameter sets,
     * etc. that can be stored in a world model.
     * Indexed by a unique id_t; */
    std::shared_ptr<EntitiesContainer> entities_;

    /** @} */
};

/** Map container interface for Entities inside a WorldModel
 * \ingroup mola_kernel_grp */
struct EntitiesContainer
{
    EntitiesContainer() = default;
    virtual ~EntitiesContainer();

    virtual std::size_t     size() const                           = 0;
    virtual EntityBase::Ptr getByID(const id_t id) const           = 0;
    virtual id_t            emplace_back(const EntityBase::Ptr& e) = 0;
};

}  // namespace mola
