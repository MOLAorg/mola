/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorRelativePose3.cpp
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */

#include <mola_kernel/factors/FactorRelativePose3.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

// arguments: classname, parent class, namespace
IMPLEMENTS_SERIALIZABLE(FactorRelativePose3, FactorBase, mola);

mola::id_t FactorRelativePose3::edge_indices(const std::size_t i) const
{
    switch (i)
    {
        case 0:
            return from_kf_;
        case 1:
            return to_kf_;
        default:
            THROW_EXCEPTION("Out of bounds");
    }
}

// Implementation of the CSerializable virtual interface:
uint8_t FactorRelativePose3::serializeGetVersion() const { return 0; }
void FactorRelativePose3::serializeTo(mrpt::serialization::CArchive& out) const
{
    baseSerializeTo(out);

    out << from_kf_ << to_kf_ << rel_pose_;
    out.WriteAs<bool>(noise_model_.has_value());
    if (noise_model_) out << mrpt::math::CMatrixD(*noise_model_);
    out << noise_model_diag_xyz_ << noise_model_diag_rot_;
}
void FactorRelativePose3::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    baseSerializeFrom(in);

    switch (version)
    {
        case 0:
        {
            in >> from_kf_ >> to_kf_ >> rel_pose_;
            if (in.ReadAs<bool>())
            {
                mrpt::math::CMatrixD m;
                in >> m;
                mrpt::math::CMatrixDouble66 m66;
                m66 = m;
                noise_model_.emplace(std::move(m66));
            }
            in >> noise_model_diag_xyz_ >> noise_model_diag_rot_;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}
