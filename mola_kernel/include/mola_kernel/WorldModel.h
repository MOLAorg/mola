/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WorldModel.h
 * @brief  The main class for a "map" or "world model".
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */
#pragma once

#include <mola_kernel/Entity.h>
#include <mola_kernel/Factor.h>
#include <mola_kernel/FastAllocator.h>
#include <mola_kernel/Yaml.h>
#include <mola_kernel/id.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mrpt/serialization/CSerializable.h>

#include <map>
#include <shared_mutex>

namespace mola
{
/** \addtogroup mola_kernel_grp
 * @{ */

using entity_connected_factors_t = mola::fast_map<id_t, mola::fast_set<fid_t>>;

/** A serializable data container for all WorldModel data (i.e. a "map")
 */
class WorldModelData : public mrpt::serialization::CSerializable
{
    DEFINE_SERIALIZABLE(WorldModelData, mola)
   public:
    struct EntitiesContainer;
    struct FactorsContainer;

    /** Arbitrary map name, used as directory prefix for saving to disk, for
     * example. It is populated at ctor with the date and time. */
    std::string map_name_;

    /** All keyframes, relative and absolute poses, calibration parameter
     * sets, etc. that can be stored in a world model. Indexed by a unique
     * id_t; */
    std::unique_ptr<EntitiesContainer> entities_;
    entity_connected_factors_t         entity_connected_factors_;
    std::shared_mutex                  entities_mtx_;

    /** All observations, constraints, etc. as generic "factors".
     * Indexed by a unique fid_t; */
    std::unique_ptr<FactorsContainer> factors_;
    std::shared_mutex                 factors_mtx_;

    mutable mola::fast_map<id_t, mrpt::Clock::time_point> entity_last_access_;
    std::shared_mutex entity_last_access_mtx_;
};

/** The main class for a "map" or "world model".
 *
 * \note This class cannot be factored out into its independent module due to
 * its tight connection to "mola-kernel/interfaces".
 */
class WorldModel : public ExecutableBase
{
    DEFINE_MRPT_OBJECT(WorldModel, mola)

   public:
    WorldModel();

    // Virtual interface of any ExecutableBase. See base docs:
    void initialize(const Yaml&) override final;
    void spinOnce() override;

    /** The WorldModel is launched first, before most other modules. */
    int launchOrderPriority() const override { return 10; }

    struct Parameters
    {
        double age_to_unload_keyframes{15.0};  //!< [s]
    };

    Parameters params_;

    /** @name Map load/save API
     * @{ */

    /** Loads the map from a given source.
     * \exception std::runtime_error in case of any I/O error. */
    void map_load_from(mrpt::serialization::CArchive& in);

    /// overload
    void map_load_from(const std::string& fileName);

    /** Saves the map to the given data sink.
     * \exception std::runtime_error in case of any I/O error. */
    void map_save_to(mrpt::serialization::CArchive& out) const;

    void map_save_to(const std::string& fileName) const;

    /** Returns the directory where entities will be swapped-off to disk when
     * unloaded, i.e. the map database directory.
     * This directory is built at construction from `MOLA_MAP_STORAGE_DIR` and
     * the map name, which by default is the current date and time.
     */
    std::string map_base_directory() const { return map_base_dir_; }

    /** @} */

    /** @name Main map content API
     * @{ */
    void entities_lock_for_read() { data_.entities_mtx_.lock_shared(); }
    void entities_unlock_for_read() { data_.entities_mtx_.unlock_shared(); }
    void entities_lock_for_write() { data_.entities_mtx_.lock(); }
    void entities_unlock_for_write() { data_.entities_mtx_.unlock(); }

    void factors_lock_for_read() { data_.factors_mtx_.lock_shared(); }
    void factors_unlock_for_read() { data_.factors_mtx_.unlock_shared(); }
    void factors_lock_for_write() { data_.factors_mtx_.lock(); }
    void factors_unlock_for_write() { data_.factors_mtx_.unlock(); }

    const Entity& entity_by_id(const id_t id) const;
    Entity&       entity_by_id(const id_t id);

    const Factor& factor_by_id(const fid_t id) const;
    Factor&       factor_by_id(const fid_t id);

    id_t  entity_emplace_back(Entity&& e);
    fid_t factor_emplace_back(Factor&& f);

    id_t  entity_push_back(const Entity& e);
    fid_t factor_push_back(const Factor& f);

    std::vector<id_t>  entity_all_ids() const;
    std::vector<fid_t> factor_all_ids() const;

    annotations_data_t&       entity_annotations_by_id(const id_t id);
    const annotations_data_t& entity_annotations_by_id(const id_t id) const;

    /** Returns all entities that are connected to a given one by any common
     * factor.
     */
    std::set<id_t> entity_neighbors(const id_t id) const;

    /** @} */

   private:
    /** All map data */
    WorldModelData data_;

    std::string map_base_dir_;

    /** Returns a list with all those entities that have not been accessed in
     * `age_to_unload_keyframes`. Once an entity is reported as "aged", it's
     * removed from the list of entities to watch, so it will be not reported
     * again unless re-loaded. */
    std::vector<id_t> findEntitiesToSwapOff();

    /** Updates entity_connected_factors_ within each call to
     * factor_emplace_back() */
    void internal_update_neighbors(const FactorBase& f);
};

/** @} */

}  // namespace mola
