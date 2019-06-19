/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WorldModel.cpp
 * @brief  The main class for a "map" or "world model".
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2018
 */

/** \defgroup mola_kernel_grp mola-kernel: MOLA data types and interfaces
 */

#include <mola-kernel/FastAllocator.h>
#include <mola-kernel/WorldModel.h>
#include <mola-kernel/entities/KeyFrameBase.h>
#include <mola-kernel/lock_helper.h>
#include <mola-kernel/variant_helper.h>
#include <mola-kernel/yaml_helpers.h>
#include <mrpt/core/initializer.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <map>
#include <numeric>  // iota()
#include <type_traits>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(WorldModel, ExecutableBase, mola);

MRPT_INITIALIZER(do_register_WorldModel)
{
    // Register:
    MOLA_REGISTER_MODULE(WorldModel);
}

/** Map container interface for Entities inside a WorldModel
 * \ingroup mola_kernel_grp */
struct WorldModel::EntitiesContainer
{
    EntitiesContainer() = default;
    virtual ~EntitiesContainer();

    virtual std::size_t              size() const               = 0;
    virtual const Entity&            by_id(const id_t id) const = 0;
    virtual Entity&                  by_id(const id_t id)       = 0;
    virtual std::pair<id_t, Entity*> emplace_back(Entity&& e)   = 0;
    virtual std::vector<id_t>        all_ids() const            = 0;
};

WorldModel::EntitiesContainer::~EntitiesContainer() = default;

/** Map container interface for Factors inside a WorldModel
 * \ingroup mola_kernel_grp */
struct WorldModel::FactorsContainer
{
    FactorsContainer() = default;
    virtual ~FactorsContainer();

    virtual std::size_t               size() const                = 0;
    virtual const Factor&             by_id(const fid_t id) const = 0;
    virtual Factor&                   by_id(const fid_t id)       = 0;
    virtual std::pair<fid_t, Factor*> emplace_back(Factor&& e)    = 0;
    virtual std::vector<fid_t>        all_ids() const             = 0;
};
WorldModel::FactorsContainer::~FactorsContainer() = default;

namespace mola
{
template <class T, class BASE, class Tbase, class Tother, typename ID>
struct ContainerDeque : public BASE
{
    std::deque<T> data_;

    ContainerDeque()           = default;
    ~ContainerDeque() override = default;

    std::size_t       size() const override { return data_.size(); }
    std::pair<ID, T*> emplace_back(T&& e) override
    {
        const ID new_id = data_.size();
        std::visit(
            overloaded{
                [new_id](Tbase& b) { b.my_id_ = new_id; },
                [new_id](Tother& o) { o->my_id_ = new_id; },
                [](std::monostate) { THROW_EXCEPTION("Empty variant!"); }},
            e);
        auto& e_ref = data_.emplace_back(e);
        return std::make_pair(new_id, &e_ref);
    }
    const T& by_id(const ID id) const override
    {
        if (id >= data_.size()) THROW_EXCEPTION("by_id(): id out of range");
        return data_[id];
    }
    T& by_id(const ID id) override
    {
        if (id >= data_.size()) THROW_EXCEPTION("by_id(): id out of range");
        return data_[id];
    }
    std::vector<ID> all_ids() const override
    {
        MRPT_TODO("Keep a separate list of *actually* existing IDs?");
        std::vector<ID> ret(this->size());
        std::iota(ret.begin(), ret.end(), 1);
        return ret;
    }
};

template <
    class T, class BASE, class Tbase, class Tother, typename ID,
    const char* Tstr>
struct ContainerFastMap : public BASE
{
    using map_t = mola::fast_map<ID, T>;

    map_t data_;

    ContainerFastMap()           = default;
    ~ContainerFastMap() override = default;

    std::size_t       size() const override { return data_.size(); }
    std::pair<ID, T*> emplace_back(T&& e) override
    {
        const ID new_id = data_.size();
        std::visit(
            overloaded{
                [new_id](Tbase& b) { b.my_id_ = new_id; },
                [new_id](Tother& o) { o->my_id_ = new_id; },
                [](std::monostate) { THROW_EXCEPTION("Empty variant!"); }},
            e);
        T& e_ref = data_[new_id] = std::move(e);
        return std::make_pair(new_id, &e_ref);
    }
    const T& by_id(const ID id) const override
    {
        const auto it = data_.find(id);
        if (it == data_.end())
        {
            THROW_EXCEPTION(mrpt::format(
                "`%s` not found with id=`%lu`", Tstr,
                static_cast<unsigned long>(id)));
        }
        return it->second;
    }
    T& by_id(const ID id) override
    {
        const auto it = data_.find(id);
        if (it == data_.end())
        {
            THROW_EXCEPTION(mrpt::format(
                "`%s` not found with id=`%lu`", Tstr,
                static_cast<unsigned long>(id)));
        }
        return it->second;
    }
    std::vector<ID> all_ids() const override
    {
        std::vector<ID> ret;
        ret.reserve(data_.size());
        for (const auto& e : data_) ret.push_back(e.first);
        return ret;
    }
};

/** Implementation of EntitiesContainer using a std::deque.
 * Avoids pool allocation for each entry, but poorly supports discontinuous
 * ID numbers. \ingroup mola_kernel_grp */
using EntitiesContainerDeque = ContainerDeque<
    Entity, WorldModel::EntitiesContainer, EntityBase, EntityOther, id_t>;
using FactorsContainerDeque = ContainerDeque<
    Factor, WorldModel::FactorsContainer, FactorBase, FactorOther, fid_t>;

static const char e_str[] = "Entity", f_str[] = "Factor";

/** Implementation of EntitiesContainer using a std::map<> with
 * mola::FastAllocator.
 *
 * \ingroup mola_kernel_grp */
using EntitiesContainerFastMap = ContainerFastMap<
    Entity, WorldModel::EntitiesContainer, EntityBase, EntityOther, id_t,
    e_str>;

using FactorsContainerFastMap = ContainerFastMap<
    Factor, WorldModel::FactorsContainer, FactorBase, FactorOther, fid_t,
    f_str>;

}  // namespace mola

void WorldModel::initialize(const std::string& cfg_block)
{
    MRPT_TRY_START

    // Load params:
    auto c   = YAML::Load(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_OPT(params_, age_to_unload_keyframes, double);

    // Create map container:
    MRPT_TODO("Switch between container type per cfg");
    entities_ =
        std::unique_ptr<EntitiesContainer>(new EntitiesContainerFastMap);
    ASSERT_(entities_);

    factors_ = std::unique_ptr<FactorsContainer>(new FactorsContainerFastMap);
    ASSERT_(factors_);

    MRPT_TRY_END
}

const Entity& WorldModel::entity_by_id(const id_t id) const
{
    return const_cast<WorldModel&>(*this).entity_by_id(id);
}

Entity& WorldModel::entity_by_id(const id_t id)
{
    Entity& e = entities_->by_id(id);
    return e;
}

const Factor& WorldModel::factor_by_id(const fid_t id) const
{
    return factors_->by_id(id);
}
Factor& WorldModel::factor_by_id(const fid_t id) { return factors_->by_id(id); }

std::vector<mola::id_t> WorldModel::entity_all_ids() const
{
    return entities_->all_ids();
}

std::vector<mola::fid_t> WorldModel::factor_all_ids() const
{
    return factors_->all_ids();
}

mola::id_t WorldModel::entity_emplace_back(Entity&& e)
{
    const auto [id, eptr] = entities_->emplace_back(std::move(e));
    (void)eptr;

    entity_connected_factors_[id];  // Create empty entry

    {
        auto lock = mola::lockHelper(entity_last_access_mtx_);

        entity_last_access_[id] = mrpt::Clock::now();
    }

    return id;
}
mola::fid_t WorldModel::factor_emplace_back(Factor&& f)
{
    const auto [id, fptr] = factors_->emplace_back(std::move(f));

    std::visit(
        overloaded{
            [this](const FactorBase& b) { internal_update_neighbors(b); },
            [this](const FactorOther& o) { internal_update_neighbors(*o); },
            [](std::monostate) { THROW_EXCEPTION("Empty variant!"); }},
        *fptr);

    return id;
}

// These versions end up calling the two versions above:
mola::id_t WorldModel::entity_push_back(const Entity& e)
{
    Entity ee = e;
    return entity_emplace_back(std::move(ee));
}
mola::fid_t WorldModel::factor_push_back(const Factor& f)
{
    Factor ff = f;
    return factor_emplace_back(std::move(ff));
}

void WorldModel::internal_update_neighbors(const FactorBase& f)
{
    const auto n = f.edge_count();
    ASSERT_(f.my_id_ != mola::INVALID_FID);
    for (size_t i = 0; i < n; i++)
    {
        const auto id = f.edge_indices(i);
        ASSERT_(id != mola::INVALID_ID);
        entity_connected_factors_[id].insert(f.my_id_);
    }
}

std::set<mola::id_t> WorldModel::entity_neighbors(const mola::id_t id) const
{
    MRPT_START

    std::set<mola::id_t> ids;
    const auto           it_ns = entity_connected_factors_.find(id);
    ASSERTMSG_(it_ns != entity_connected_factors_.end(), "Unknown entity `id`");

    auto adder = [&ids](const FactorBase& b) {
        const auto n = b.edge_count();
        for (size_t i = 0; i < n; i++) ids.insert(b.edge_indices(i));
    };

    for (const auto fid : it_ns->second)
    {
        const auto& f = factors_->by_id(fid);
        std::visit(
            overloaded{
                [&adder](const FactorBase& b) { adder(b); },
                [&adder](const FactorOther& o) { adder(*o); },
                [](std::monostate) { THROW_EXCEPTION("Empty variant!"); }},
            f);
    }
    return ids;

    MRPT_END
}

annotations_data_t& WorldModel::entity_annotations_by_id(const id_t id)
{
    return const_cast<annotations_data_t&>(
        const_cast<const WorldModel*>(this)->entity_annotations_by_id(id));
}
const annotations_data_t& WorldModel::entity_annotations_by_id(
    const id_t id) const
{
    MRPT_START

    {
        auto lock               = mola::lockHelper(entity_last_access_mtx_);
        entity_last_access_[id] = mrpt::Clock::now();
    }

    const annotations_data_t* ret = nullptr;
    auto&                     e   = entities_->by_id(id);

    // Load on-the-fly if required:
    entity_get_base(e).load();

    std::visit(
        overloaded{[&ret](const EntityBase& b) { ret = &b.annotations_; },
                   [&ret](const EntityOther& o) {
                       ASSERT_(o);
                       ret = &o->annotations_;
                   },
                   [](std::monostate) {}},
        e);

    if (!ret) { THROW_EXCEPTION("Empty variant!"); }
    else
        return *ret;

    MRPT_END
}

std::vector<mola::id_t> WorldModel::findEntitiesToSwapOff()
{
    MRPT_START
    ProfilerEntry tle(profiler_, "findEntitiesToSwapOff");

    std::vector<id_t> aged_ids;

    auto       lk    = lockHelper(entity_last_access_mtx_);
    const auto t_now = mrpt::Clock::now();

    for (auto it_ent = entity_last_access_.begin();
         it_ent != entity_last_access_.end();)
    {
        const double age = mrpt::system::timeDifference(it_ent->second, t_now);

        if (age > params_.age_to_unload_keyframes)
        {
            const auto id = it_ent->first;
            // Remove from list:
            it_ent = entity_last_access_.erase(it_ent);
            // and report:
            aged_ids.push_back(id);
        }
        else
        {
            ++it_ent;
        }
    }

    return aged_ids;
    MRPT_END
}

void WorldModel::spinOnce()
{
    MRPT_START

    // Unload KeyFrames that have not been used in a while:
    const std::vector<id_t> aged_ids = findEntitiesToSwapOff();

    ProfilerEntry tle(profiler_, "unload_aged_entities");

    // and do unload()
    if (!aged_ids.empty())
    {
        entities_lock_for_write();

        for (auto id : aged_ids)
            mola::entity_get_base(entities_->by_id(id)).unload();

        entities_unlock_for_write();

        MRPT_LOG_DEBUG_STREAM(
            "Swapped-off to disk: " << aged_ids.size() << " map entities.");
    }

    profiler_.registerUserMeasure("unloaded_count", aged_ids.size());

    MRPT_END
}
