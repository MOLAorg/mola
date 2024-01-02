/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorBase.cpp
 * @brief  Base class for all "factors" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */

#include <mola_kernel/factors/FactorBase.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

// arguments: class, parent class, namespace
IMPLEMENTS_VIRTUAL_SERIALIZABLE(
    FactorBase, mrpt::serialization::CSerializable, mola)

FactorBase::~FactorBase() = default;

void FactorBase::baseSerializeTo(mrpt::serialization::CArchive& out) const
{
    out << my_id_ << robust_param_;
    out.WriteAs<uint8_t>(robust_type_);
}
void FactorBase::baseSerializeFrom(mrpt::serialization::CArchive& in)
{
    in >> my_id_ >> robust_param_;
    robust_type_ = static_cast<mola::Robust>(in.ReadAs<uint8_t>());
}
