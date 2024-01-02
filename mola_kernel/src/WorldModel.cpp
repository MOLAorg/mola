/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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

#include <mola_kernel/FastAllocator.h>
#include <mola_kernel/WorldModel.h>
#include <mola_kernel/entities/KeyFrameBase.h>
#include <mola_kernel/variant_helper.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <deque>
#include <map>
#include <numeric>  // iota()
#include <type_traits>

using namespace mola;

std::string MOLA_MAP_STORAGE_DIR =
    mrpt::get_env<std::string>("MOLA_MAP_STORAGE_DIR", "/tmp");

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(WorldModel, ExecutableBase, mola)

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_SERIALIZABLE(
    WorldModelData, mrpt::serialization::CSerializable, mola)

// =============== WorldModel ===============
WorldModel::WorldModel()
{
    using namespace std::string_literals;
    MRPT_START

    this->setLoggerName("WorldModel");

    data_.map_name_ =
        "mola_map_" +
        mrpt::system::fileNameStripInvalidChars(
            mrpt::system::dateTimeLocalToString(mrpt::Clock::now()));

    map_base_dir_ = MOLA_MAP_STORAGE_DIR + "/"s + data_.map_name_ + "/"s;

    LazyLoadResource::EXTERNAL_BASE_DIR = map_base_dir_;

    MRPT_LOG_INFO_STREAM("=== Using map name: `" << data_.map_name_ << "` ===");
    MRPT_LOG_INFO_STREAM(
        "=== Setting map storage base directory: `" << map_base_dir_
                                                    << "` ===");

    if (!mrpt::system::createDirectory(map_base_dir_))
        THROW_EXCEPTION_FMT(
            "Error creating directory: `%s`", map_base_dir_.c_str());

    MRPT_END
}

/** Map container interface for Entities inside a WorldModel
 * \ingroup mola_kernel_grp */
struct WorldModelData::EntitiesContainer
{
    EntitiesContainer() = default;
    virtual ~EntitiesContainer();

    virtual std::size_t              size() const               = 0;
    virtual const Entity&            by_id(const id_t id) const = 0;
    virtual Entity&                  by_id(const id_t id)       = 0;
    virtual std::pair<id_t, Entity*> emplace_back(Entity&& e)   = 0;
    virtual std::vector<id_t>        all_ids() const            = 0;
    virtual void                     clear()                    = 0;
};

WorldModelData::EntitiesContainer::~EntitiesContainer() = default;

/** Map container interface for Factors inside a WorldModel
 * \ingroup mola_kernel_grp */
struct WorldModelData::FactorsContainer
{
    FactorsContainer() = default;
    virtual ~FactorsContainer();

    virtual std::size_t               size() const                = 0;
    virtual const Factor&             by_id(const fid_t id) const = 0;
    virtual Factor&                   by_id(const fid_t id)       = 0;
    virtual std::pair<fid_t, Factor*> emplace_back(Factor&& e)    = 0;
    virtual std::vector<fid_t>        all_ids() const             = 0;
    virtual void                      clear()                     = 0;
};
WorldModelData::FactorsContainer::~FactorsContainer() = default;

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
    void clear() override { data_.clear(); }
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
    void clear() override { data_.clear(); }
};

/** Implementation of EntitiesContainer using a std::deque.
 * Avoids pool allocation for each entry, but poorly supports discontinuous
 * ID numbers. \ingroup mola_kernel_grp */
using EntitiesContainerDeque = ContainerDeque<
    Entity, WorldModelData::EntitiesContainer, EntityBase, EntityOther, id_t>;
using FactorsContainerDeque = ContainerDeque<
    Factor, WorldModelData::FactorsContainer, FactorBase, FactorOther, fid_t>;

static const char e_str[] = "Entity", f_str[] = "Factor";

/** Implementation of EntitiesContainer using a std::map<> with
 * mola::FastAllocator.
 *
 * \ingroup mola_kernel_grp */
using EntitiesContainerFastMap = ContainerFastMap<
    Entity, WorldModelData::EntitiesContainer, EntityBase, EntityOther, id_t,
    e_str>;

using FactorsContainerFastMap = ContainerFastMap<
    Factor, WorldModelData::FactorsContainer, FactorBase, FactorOther, fid_t,
    f_str>;

}  // namespace mola

void WorldModel::initialize(const Yaml& c)
{
    MRPT_TRY_START

    // Load params:
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_OPT(params_, age_to_unload_keyframes, double);

    // Create map container:
    MRPT_TODO("Switch between container type per cfg");
    data_.entities_ = std::make_unique<EntitiesContainerFastMap>();
    data_.factors_  = std::unique_ptr<FactorsContainerFastMap>();

    ASSERT_(data_.entities_);
    ASSERT_(data_.factors_);

    MRPT_TRY_END
}

const Entity& WorldModel::entity_by_id(const id_t id) const
{
    return const_cast<WorldModel&>(*this).entity_by_id(id);
}

Entity& WorldModel::entity_by_id(const id_t id)
{
    Entity& e = data_.entities_->by_id(id);
    return e;
}

const Factor& WorldModel::factor_by_id(const fid_t id) const
{
    return data_.factors_->by_id(id);
}
Factor& WorldModel::factor_by_id(const fid_t id)
{
    return data_.factors_->by_id(id);
}

std::vector<mola::id_t> WorldModel::entity_all_ids() const
{
    return data_.entities_->all_ids();
}

std::vector<mola::fid_t> WorldModel::factor_all_ids() const
{
    return data_.factors_->all_ids();
}

mola::id_t WorldModel::entity_emplace_back(Entity&& e)
{
    const auto [id, eptr] = data_.entities_->emplace_back(std::move(e));
    (void)eptr;

    data_.entity_connected_factors_[id];  // Create empty entry

    {
        auto lock = mrpt::lockHelper(data_.entity_last_access_mtx_);

        data_.entity_last_access_[id] = mrpt::Clock::now();
    }

    return id;
}
mola::fid_t WorldModel::factor_emplace_back(Factor&& f)
{
    const auto [id, fptr] = data_.factors_->emplace_back(std::move(f));

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
        data_.entity_connected_factors_[id].insert(f.my_id_);
    }
}

std::set<mola::id_t> WorldModel::entity_neighbors(const mola::id_t id) const
{
    MRPT_START

    std::set<mola::id_t> ids;
    const auto           it_ns = data_.entity_connected_factors_.find(id);
    ASSERTMSG_(
        it_ns != data_.entity_connected_factors_.end(), "Unknown entity `id`");

    auto adder = [&ids](const FactorBase& b) {
        const auto n = b.edge_count();
        for (size_t i = 0; i < n; i++) ids.insert(b.edge_indices(i));
    };

    for (const auto fid : it_ns->second)
    {
        const auto& f = data_.factors_->by_id(fid);
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
        auto lock = mrpt::lockHelper(data_.entity_last_access_mtx_);
        data_.entity_last_access_[id] = mrpt::Clock::now();
    }

    const annotations_data_t* ret = nullptr;
    auto&                     e   = data_.entities_->by_id(id);

    // Load on-the-fly if required:
    entity_get_base(e).load();

    std::visit(
        overloaded{
            [&ret](const EntityBase& b) { ret = &b.annotations_; },
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

    auto       lk    = mrpt::lockHelper(data_.entity_last_access_mtx_);
    const auto t_now = mrpt::Clock::now();

    for (auto it_ent = data_.entity_last_access_.begin();
         it_ent != data_.entity_last_access_.end();)
    {
        const double age = mrpt::system::timeDifference(it_ent->second, t_now);

        if (age > params_.age_to_unload_keyframes)
        {
            const auto id = it_ent->first;
            // Remove from list:
            it_ent = data_.entity_last_access_.erase(it_ent);
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
            mola::entity_get_base(data_.entities_->by_id(id)).unload();

        entities_unlock_for_write();

        MRPT_LOG_DEBUG_STREAM(
            "Swapped-off to disk: " << aged_ids.size() << " map entities.");
    }

    profiler_.registerUserMeasure("unloaded_count", aged_ids.size());

    MRPT_END
}

void WorldModel::map_load_from(mrpt::serialization::CArchive& in)
{
    MRPT_START
    in >> data_;
    MRPT_END
}

void WorldModel::map_load_from(const std::string& fileName)
{
    MRPT_START
    mrpt::io::CFileGZInputStream f(fileName);

    auto in = mrpt::serialization::archiveFrom(f);
    map_load_from(in);
    MRPT_END
}

void WorldModel::map_save_to(mrpt::serialization::CArchive& out) const
{
    MRPT_START
    out << data_;
    MRPT_END
}

void WorldModel::map_save_to(const std::string& fileName) const
{
    MRPT_START
    mrpt::io::CFileGZOutputStream f(fileName);

    auto out = mrpt::serialization::archiveFrom(f);
    map_save_to(out);
    MRPT_END
}

// =============== WorldModelData ===============
// See docs for mrpt-serialization:
uint8_t WorldModelData::serializeGetVersion() const { return 0; }
void    WorldModelData::serializeTo(mrpt::serialization::CArchive& out) const
{
    // Ensure lock:
    {
        auto el = mrpt::lockHelper(entities_mtx_);
    }
    {
        auto ef = mrpt::lockHelper(factors_mtx_);
    }
    auto el = mrpt::lockHelper(entities_mtx_);
    auto ef = mrpt::lockHelper(factors_mtx_);

    out << map_name_;

    ASSERT_(entities_);
    ASSERT_(factors_);

    MRPT_TODO("Improvement: offer a visitor-like instead of call all_ids()");

    // Entities:
    std::vector<id_t> entity_ids = entities_->all_ids();
    out << entity_ids;
    for (auto eid : entity_ids)
    {
        const Entity&     ent = entities_->by_id(eid);
        const EntityBase& e   = mola::entity_get_base(ent);
        out << e;
    }

    // Factors:
    std::vector<fid_t> factor_ids = factors_->all_ids();
    out << factor_ids;
    for (auto fid : factor_ids)
    {
        const Factor&     fac = factors_->by_id(fid);
        const FactorBase& f   = mola::factor_get_base(fac);
        out << f;
    }
}
void WorldModelData::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    // Ensure lock:
    {
        auto el = mrpt::lockHelper(entities_mtx_);
    }
    {
        auto ef = mrpt::lockHelper(factors_mtx_);
    }
    auto el = mrpt::lockHelper(entities_mtx_);
    auto ef = mrpt::lockHelper(factors_mtx_);

    // Clear:
    this->entities_->clear();

    switch (version)
    {
        case 0:
        {
            // Entities:
            std::vector<id_t> entity_ids;
            in >> entity_ids;
            for (auto eid : entity_ids)
            {
                // TODO: Any way to avoid repeating the list of tparams here?
                Entity e = in.ReadVariant<
                    std::monostate, RefPose3, RelPose3, RelPose3KF,
                    RelDynPose3KF, LandmarkPoint3, EntityOther>();

                mola::entity_get_base(e).my_id_ = eid;
                entities_->emplace_back(std::move(e));
            }

            // Factors:
            std::vector<fid_t> factor_ids;
            in >> factor_ids;
            for (auto fid : factor_ids)
            {
                Factor f = in.ReadVariant<
                    std::monostate, FactorRelativePose3, FactorDynamicsConstVel,
                    FactorStereoProjectionPose, SmartFactorStereoProjectionPose,
                    SmartFactorIMU, FactorOther>();

                mola::factor_get_base(f).my_id_ = fid;
                factors_->emplace_back(std::move(f));
            }
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}
