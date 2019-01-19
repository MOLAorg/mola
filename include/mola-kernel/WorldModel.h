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
#include <mola-kernel/FastAllocator.h>
#include <mola-kernel/id.h>
#include <map>
#include <mutex>

namespace mola
{
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

    /** The WorldModel is launched first, before most other modules. */
    int launchOrderPriority() const override { return 10; }

    using Ptr = std::shared_ptr<WorldModel>;

    using entity_connected_factors_t =
        mola::fast_map<id_t, mola::fast_set<fid_t>>;

    /** @name Main API
     * @{ */
    void entities_lock() { entities_mtx_.lock(); }
    void entities_unlock() { entities_mtx_.unlock(); }

    void factors_lock() { factors_mtx_.lock(); }
    void factors_unlock() { factors_mtx_.unlock(); }

    const Entity& entity_by_id(const id_t id) const;
    Entity&       entity_by_id(const id_t id);

    id_t  entity_emplace_back(Entity&& e);
    fid_t factor_emplace_back(Factor&& f);
    id_t  entity_push_back(const Entity& e);
    fid_t factor_push_back(const Factor& f);

    annotations_data_t&       entity_annotations_by_id(const id_t id);
    const annotations_data_t& entity_annotations_by_id(const id_t id) const;

    /** Returns all entities that are connected to a given one by any common
     * factor.
     */
    std::set<id_t> entity_neighbors(const id_t id) const;

    /** @} */

    struct EntitiesContainer;
    struct FactorsContainer;

   private:
    /** All keyframes, relative and absolute poses, calibration parameter sets,
     * etc. that can be stored in a world model.
     * Indexed by a unique id_t; */
    std::unique_ptr<EntitiesContainer> entities_;
    entity_connected_factors_t         entity_connected_factors_;
    std::recursive_timed_mutex         entities_mtx_;

    /** All observations, constraints, etc. as generic "factors".
     * Indexed by a unique fid_t; */
    std::unique_ptr<FactorsContainer> factors_;
    std::recursive_timed_mutex        factors_mtx_;

    void internal_update_neighbors(const FactorBase& f);
};

}  // namespace mola
