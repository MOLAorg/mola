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
#include <mrpt/core/initializer.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <map>

using namespace mola;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(WorldModel)}

EntitiesContainer::~EntitiesContainer()
{
}

namespace mola
{
/** Implementation of EntitiesContainer using a std::deque.
 * Avoids pool allocation for each entry, but poorly supports discontinuous ID
 * numbers.
 * \ingroup mola_kernel_grp */
struct EntitiesContainerDeque : public mola::EntitiesContainer
{
    std::deque<EntityBase::Ptr> data_;

    EntitiesContainerDeque() = default;
    ~EntitiesContainerDeque() override;

    std::size_t size() const override { return data_.size(); }
    id_t        emplace_back(const EntityBase::Ptr& e) override
    {
        const auto n = data_.size();
        data_.emplace_back(e);
        return n;
    }
    EntityBase::Ptr getByID(const id_t id) const override
    {
        if (id >= data_.size()) THROW_EXCEPTION("getByID(): id out of range");
        return data_[id];
    }
};

EntitiesContainerDeque::~EntitiesContainerDeque() = default;

/** Implementation of EntitiesContainer using a std::map<> with
 * mola::FastAllocator.
 *
 * \ingroup mola_kernel_grp */
struct EntitiesContainerFastMap : public mola::EntitiesContainer
{
    using T = EntityBase::Ptr;
    std::map<
        id_t, T, std::less<id_t>, mola::FastAllocator<std::pair<const id_t, T>>>
        data_;

    EntitiesContainerFastMap() = default;
    ~EntitiesContainerFastMap() override;

    std::size_t size() const override { return data_.size(); }
    id_t        emplace_back(const T& e) override
    {
        const auto n = data_.size();
        data_[n]     = e;
        return n;
    }
    T getByID(const id_t id) const override
    {
        const auto it = data_.find(id);
        ASSERTMSG_(it != data_.end(), "Attempt to access non-existing entity");
        return it->second;
    }
};

EntitiesContainerFastMap::~EntitiesContainerFastMap() = default;

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
