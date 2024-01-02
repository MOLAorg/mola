/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RelPose3KF.cpp
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */

#include <mola_kernel/entities/RelPose3KF.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

// arguments: classname, parent class, namespace
IMPLEMENTS_SERIALIZABLE(RelPose3KF, EntityBase, mola);

// Implementation of the CSerializable virtual interface:
uint8_t RelPose3KF::serializeGetVersion() const { return 0; }
void    RelPose3KF::serializeTo(mrpt::serialization::CArchive& out) const
{
    baseSerializeTo(out);

    out << relpose_value << raw_observations_;
}
void RelPose3KF::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    baseSerializeFrom(in);

    switch (version)
    {
        case 0:
        {
            in >> relpose_value >> raw_observations_;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}
