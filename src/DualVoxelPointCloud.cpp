/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

/**
 * @file   DualVoxelPointCloud.cpp
 * @brief  Point cloud stored as a dual-resolution voxel map
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */

#include <mola_metric_maps/DualVoxelPointCloud.h>
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR
#include <mrpt/serialization/CArchive.h>  // serialization

#include <cmath>

using namespace mola;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
    "mola::DualVoxelPointCloud,DualVoxelPointCloud", mola::DualVoxelPointCloud)

DualVoxelPointCloud::TMapDefinition::TMapDefinition() = default;
void DualVoxelPointCloud::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& s, const std::string& sectionPrefix)
{
    using namespace std::string_literals;

    // [<sectionNamePrefix>+"_creationOpts"]
    const std::string sSectCreation = sectionPrefix + "_creationOpts"s;
    MRPT_LOAD_CONFIG_VAR(decimation_size, float, s, sSectCreation);
    MRPT_LOAD_CONFIG_VAR(max_nn_radius, float, s, sSectCreation);
    MRPT_LOAD_CONFIG_VAR(max_points_per_voxel, uint64_t, s, sSectCreation);

    // insertionOpts.loadFromConfigFile(s, sectionPrefix + "_insertOpts"s);
    // likelihoodOpts.loadFromConfigFile(s, sectionNamePrefix +
    // "_likelihoodOpts"s);
}

void DualVoxelPointCloud::TMapDefinition::dumpToTextStream_map_specific(
    std::ostream& out) const
{
    LOADABLEOPTS_DUMP_VAR(decimation_size, float);
    LOADABLEOPTS_DUMP_VAR(max_nn_radius, float);
    LOADABLEOPTS_DUMP_VAR(max_points_per_voxel, float);

    // insertionOpts.dumpToTextStream(out);
    // this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* DualVoxelPointCloud::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
    const DualVoxelPointCloud::TMapDefinition* def =
        dynamic_cast<const DualVoxelPointCloud::TMapDefinition*>(&_def);
    ASSERT_(def);
    auto* obj = new DualVoxelPointCloud(
        def->decimation_size, def->max_nn_radius, def->max_points_per_voxel);

    // obj->insertionOptions  = def.insertionOpts;
    // obj->likelihoodOptions = def.likelihoodOpts;
    return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(DualVoxelPointCloud, CMetricMap, mola)

// =====================================
// Serialization
// =====================================

uint8_t DualVoxelPointCloud::serializeGetVersion() const { return 0; }
void DualVoxelPointCloud::serializeTo(mrpt::serialization::CArchive& out) const
{
    // The data
    THROW_EXCEPTION("TODO");
}
void DualVoxelPointCloud::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            THROW_EXCEPTION("TODO");
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };

    // cache reset?
}

// Ctor:
DualVoxelPointCloud::DualVoxelPointCloud(
    float decimation_size, float max_nn_radius, uint32_t max_points_per_voxel)
{
    setVoxelProperties(decimation_size, max_nn_radius, max_points_per_voxel);
}

DualVoxelPointCloud::~DualVoxelPointCloud() = default;

void DualVoxelPointCloud::setVoxelProperties(
    float decimation_size, float max_nn_radius, uint32_t max_points_per_voxel)
{
    decimation_size_      = decimation_size;
    max_nn_radius_        = max_nn_radius;
    max_points_per_voxel_ = max_points_per_voxel;

    nn_to_decim_ratio_ =
        static_cast<int32_t>(std::ceil(max_nn_radius_ / decimation_size_));

    nn_to_decim_ratio_sqr_ = nn_to_decim_ratio_ * nn_to_decim_ratio_;

    // clear all:
    internal_clear();
}

std::string DualVoxelPointCloud::asString() const
{
    return mrpt::format(
        "DualVoxelPointCloud, decimation_size=%.03f max_nn_radius=%.03f",
        decimation_size_, max_nn_radius_);
}

void DualVoxelPointCloud::getVisualizationInto(
    mrpt::opengl::CSetOfObjects& outObj) const
{
    THROW_EXCEPTION("TODO");
}

void DualVoxelPointCloud::internal_clear()
{
    voxels_.clear();
    voxelsNN_.clear();
}

bool DualVoxelPointCloud::internal_insertObservation(
    const mrpt::obs::CObservation&                   obs,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
    THROW_EXCEPTION("TODO");
}

double DualVoxelPointCloud::internal_computeObservationLikelihood(
    const mrpt::obs::CObservation& obs,
    const mrpt::poses::CPose3D&    takenFrom) const
{
    THROW_EXCEPTION("TODO");
}

bool DualVoxelPointCloud::internal_canComputeObservationLikelihood(
    const mrpt::obs::CObservation& obs) const
{
    THROW_EXCEPTION("TODO");
}

bool DualVoxelPointCloud::isEmpty() const
{
    //
    return voxels_.empty();
}

void DualVoxelPointCloud::saveMetricMapRepresentationToFile(
    const std::string& filNamePrefix) const
{
    //
    THROW_EXCEPTION("TODO");
}
