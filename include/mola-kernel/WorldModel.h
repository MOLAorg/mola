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

#include <mola-kernel/Entity.h>
#include <mola-kernel/ExecutableBase.h>
#include <mola-kernel/Factor.h>
#include <mola-kernel/id.h>
#include <map>

namespace mola
{
struct EntitiesContainer;
struct FactorsContainer;

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

    /** All observations, constraints, etc. as generic "factors".
     * Indexed by a unique fid_t; */
    std::shared_ptr<FactorsContainer> factors_;

    /** @} */
};

/** Map container interface for Entities inside a WorldModel
 * \ingroup mola_kernel_grp */
struct EntitiesContainer
{
    EntitiesContainer() = default;
    virtual ~EntitiesContainer();

    virtual std::size_t   size() const                 = 0;
    virtual const Entity& getByID(const id_t id) const = 0;
    virtual Entity&       getByID(const id_t id)       = 0;
    virtual id_t          emplace_back(Entity&& e)     = 0;
};

/** Map container interface for Factors inside a WorldModel
 * \ingroup mola_kernel_grp */
struct FactorsContainer
{
    FactorsContainer() = default;
    virtual ~FactorsContainer();

    virtual std::size_t   size() const                 = 0;
    virtual const Factor& getByID(const id_t id) const = 0;
    virtual Factor&       getByID(const id_t id)       = 0;
    virtual id_t          emplace_back(Factor&& e)     = 0;
};

}  // namespace mola
