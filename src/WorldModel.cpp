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
#include <mola-kernel/variant_helper.h>
#include <mrpt/core/initializer.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <map>
#include <type_traits>

using namespace mola;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(WorldModel)}

EntitiesContainer::~EntitiesContainer()
{
}

namespace mola
{
template <class T, class BASE, class Tbase, class Tother, typename ID>
struct ContainerDeque : public BASE
{
    std::deque<T> data_;

    ContainerDeque()           = default;
    ~ContainerDeque() override = default;

    std::size_t size() const override { return data_.size(); }
    ID          emplace_back(T&& e) override
    {
        const auto new_id = data_.size();
        std::visit(
            overloaded{[new_id](Tbase& b) { b.my_id_ = new_id; },
                       [new_id](Tother& o) { o->my_id_ = new_id; }},
            e);
        data_.emplace_back(e);
        return new_id;
    }
    const T& getByID(const ID id) const override
    {
        if (id >= data_.size()) THROW_EXCEPTION("getByID(): id out of range");
        return data_[id];
    }
    T& getByID(const ID id) override
    {
        if (id >= data_.size()) THROW_EXCEPTION("getByID(): id out of range");
        return data_[id];
    }
};

template <class T, class BASE, class Tbase, class Tother, typename ID>
struct ContainerFastMap : public BASE
{
    std::map<ID, T, std::less<ID>, mola::FastAllocator<std::pair<const ID, T>>>
        data_;

    ContainerFastMap()           = default;
    ~ContainerFastMap() override = default;

    std::size_t size() const override { return data_.size(); }
    ID          emplace_back(T&& e) override
    {
        const auto new_id = data_.size();
        std::visit(
            overloaded{[new_id](Tbase& b) { b.my_id_ = new_id; },
                       [new_id](Tother& o) { o->my_id_ = new_id; }},
            e);
        data_[new_id] = e;
        return new_id;
    }
    const T& getByID(const ID id) const override
    {
        const auto it = data_.find(id);
        ASSERTMSG_(it != data_.end(), "Attempt to access non-existing entity");
        return it->second;
    }
    T& getByID(const ID id) override
    {
        const auto it = data_.find(id);
        ASSERTMSG_(it != data_.end(), "Attempt to access non-existing entity");
        return it->second;
    }
};

/** Implementation of EntitiesContainer using a std::deque.
 * Avoids pool allocation for each entry, but poorly supports discontinuous ID
 * numbers.
 * \ingroup mola_kernel_grp */
using EntitiesContainerDeque =
    ContainerDeque<Entity, EntitiesContainer, EntityBase, EntityOther, id_t>;
using FactorsContainerDeque =
    ContainerDeque<Factor, FactorsContainer, FactorBase, FactorOther, fid_t>;

/** Implementation of EntitiesContainer using a std::map<> with
 * mola::FastAllocator.
 *
 * \ingroup mola_kernel_grp */
using EntitiesContainerFastMap =
    ContainerFastMap<Entity, EntitiesContainer, EntityBase, EntityOther, id_t>;

using FactorsContainerFastMap =
    ContainerFastMap<Factor, FactorsContainer, FactorBase, FactorOther, fid_t>;

}  // namespace mola

void WorldModel::initialize(const std::string& cfg_block)
{
    MRPT_TRY_START
    auto cfg = YAML::Load(cfg_block);

    // Create map container:
    MRPT_TODO("Switch between container type per cfg");

    auto c    = std::make_shared<EntitiesContainerDeque>();
    entities_ = std::dynamic_pointer_cast<EntitiesContainer>(c);
    ASSERT_(entities_);

    MRPT_TRY_END
}
