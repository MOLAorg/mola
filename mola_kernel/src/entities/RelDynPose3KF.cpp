/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RelDynPose3KF.cpp
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */

#include <mola_kernel/entities/RelDynPose3KF.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

// arguments: classname, parent class, namespace
IMPLEMENTS_SERIALIZABLE(RelDynPose3KF, EntityBase, mola);

// Implementation of the CSerializable virtual interface:
uint8_t RelDynPose3KF::serializeGetVersion() const { return 0; }
void    RelDynPose3KF::serializeTo(mrpt::serialization::CArchive& out) const
{
    baseSerializeTo(out);

    out << relpose_value << twist_value;
}
void RelDynPose3KF::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    baseSerializeFrom(in);

    switch (version)
    {
        case 0:
        {
            in >> relpose_value >> twist_value;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}
