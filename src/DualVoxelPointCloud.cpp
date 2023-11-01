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
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
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

    likelihoodOpts.loadFromConfigFile(s, sectionPrefix + "_likelihoodOpts"s);
    renderOpts.loadFromConfigFile(s, sectionPrefix + "_renderOpts"s);
}

void DualVoxelPointCloud::TMapDefinition::dumpToTextStream_map_specific(
    std::ostream& out) const
{
    LOADABLEOPTS_DUMP_VAR(decimation_size, float);
    LOADABLEOPTS_DUMP_VAR(max_nn_radius, float);
    LOADABLEOPTS_DUMP_VAR(max_points_per_voxel, float);

    likelihoodOpts.dumpToTextStream(out);
    renderOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* DualVoxelPointCloud::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
    const DualVoxelPointCloud::TMapDefinition* def =
        dynamic_cast<const DualVoxelPointCloud::TMapDefinition*>(&_def);
    ASSERT_(def);
    auto* obj = new DualVoxelPointCloud(
        def->decimation_size, def->max_nn_radius, def->max_points_per_voxel);

    obj->likelihoodOptions = def->likelihoodOpts;
    obj->renderOptions     = def->renderOpts;
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
    // params:
    out << decimation_size_ << max_nn_radius_ << max_points_per_voxel_;
    likelihoodOptions.writeToStream(out);
    renderOptions.writeToStream(out);

    // data:
    out.WriteAs<uint32_t>(voxels_.size());
    for (const auto& kv : voxels_)
    {
        out << kv.first.cx_ << kv.first.cy_ << kv.first.cz_;
        const auto& pts = kv.second.points();
        out.WriteAs<uint32_t>(pts.size());
        for (const auto& pt : pts) out << pt.x << pt.y << pt.z;
    }
}
void DualVoxelPointCloud::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            // params:
            in >> decimation_size_ >> max_nn_radius_ >> max_points_per_voxel_;

            // clear contents and compute computed fields:
            this->setVoxelProperties(
                decimation_size_, max_nn_radius_, max_points_per_voxel_);

            likelihoodOptions.readFromStream(in);
            renderOptions.readFromStream(in);

            // data:
            const auto nVoxels = in.ReadAs<uint32_t>();
            for (uint32_t i = 0; i < nVoxels; i++)
            {
                index3d_t idx;
                in >> idx.cx_ >> idx.cy_ >> idx.cz_;

                auto& v = voxels_[idx];

                const auto nPts = in.ReadAs<uint32_t>();
                for (uint32_t j = 0; j < nPts; j++)
                {
                    mrpt::math::TPoint3Df pt;
                    in >> pt.x >> pt.y >> pt.z;
                    v.insertPoint(pt);
                }
                // Insert this voxel into the list of neighbors for all
                // voxels in the nearby volume:
                internalUpdateNNs(idx, v);
            }
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };

    // any cache reset?
}

// VoxelData

const mrpt::math::TPoint3Df& DualVoxelPointCloud::VoxelData::mean() const
{
    if (!mean_)
    {
        ASSERT_(!points_.empty());

        mrpt::math::TPoint3Df m = {0, 0, 0};
        for (const auto& v : points_) m += v;
        m *= 1.0f / points_.size();
        mean_.emplace(m);
    }

    return mean_.value();
}

void DualVoxelPointCloud::VoxelData::insertPoint(const mrpt::math::TPoint3Df& p)
{
    mean_.reset();
    points_.push_back(p);
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

    // calculated fields:
    max_nn_radius_sqr_ = mrpt::square(max_nn_radius_);
    nn_to_decim_ratio_ =
        static_cast<int32_t>(std::ceil(max_nn_radius_ / decimation_size_));

    // clear all:
    DualVoxelPointCloud::internal_clear();
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
    using namespace mrpt::obs;

    // This function depends on the observation type:
    // -----------------------------------------------------
    if (IS_CLASS(obs, CObservation2DRangeScan))
    {
        // Observation is a laser range scan:
        // -------------------------------------------
        const auto& o = static_cast<const CObservation2DRangeScan&>(obs);

        // Build (if not done before) the points map representation of this
        // observation:
        const auto* scanPoints = o.buildAuxPointsMap<CPointsMap>();

        const size_t N = scanPoints->m_x.size();
        if (!N || !this->size()) return -100;

        const float* xs = &scanPoints->m_x[0];
        const float* ys = &scanPoints->m_y[0];
        const float* zs = &scanPoints->m_z[0];

        if (takenFrom.isHorizontal())
        {
            double      sumSqrDist = 0;
            float       closest_x, closest_y;
            float       closest_err;
            const float max_sqr_err =
                mrpt::square(likelihoodOptions.max_corr_distance);

            // optimized 2D version ---------------------------
            mrpt::math::TPose2D takenFrom2D =
                mrpt::poses::CPose2D(takenFrom).asTPose();

            const double ccos           = cos(takenFrom2D.phi);
            const double csin           = sin(takenFrom2D.phi);
            int          nPtsForAverage = 0;

            for (size_t i = 0; i < N;
                 i += likelihoodOptions.decimation, nPtsForAverage++)
            {
                // Transform the point from the scan reference to its global
                // 3D position:
                const float xg = takenFrom2D.x + ccos * xs[i] - csin * ys[i];
                const float yg = takenFrom2D.y + csin * xs[i] + ccos * ys[i];

                kdTreeClosestPoint2D(
                    xg, yg,  // Look for the closest to this guy
                    closest_x, closest_y,  // save here the closest match
                    closest_err  // save here the min. distance squared
                );

                // Put a limit:
                mrpt::keep_min(closest_err, max_sqr_err);

                sumSqrDist += static_cast<double>(closest_err);
            }
            sumSqrDist /= nPtsForAverage;
            // Log-likelihood:
            return -sumSqrDist / likelihoodOptions.sigma_dist;
        }
        else
        {
            // Generic 3D version ---------------------------
            return internal_computeObservationLikelihoodPointCloud3D(
                takenFrom, xs, ys, zs, N);
        }
    }
    else if (IS_CLASS(obs, CObservationVelodyneScan))
    {
        const auto& o = dynamic_cast<const CObservationVelodyneScan&>(obs);

        // Automatically generate pointcloud if needed:
        if (!o.point_cloud.size())
            const_cast<CObservationVelodyneScan&>(o).generatePointCloud();

        const size_t N = o.point_cloud.size();
        if (!N || !this->size()) return -100;

        const CPose3D sensorAbsPose = takenFrom + o.sensorPose;

        const float* xs = &o.point_cloud.x[0];
        const float* ys = &o.point_cloud.y[0];
        const float* zs = &o.point_cloud.z[0];

        return internal_computeObservationLikelihoodPointCloud3D(
            sensorAbsPose, xs, ys, zs, N);
    }
    else if (IS_CLASS(obs, CObservationPointCloud))
    {
        const auto& o = dynamic_cast<const CObservationPointCloud&>(obs);

        const size_t N = o.pointcloud->size();
        if (!N || !this->size()) return -100;

        const CPose3D sensorAbsPose = takenFrom + o.sensorPose;

        auto xs = o.pointcloud->getPointsBufferRef_x();
        auto ys = o.pointcloud->getPointsBufferRef_y();
        auto zs = o.pointcloud->getPointsBufferRef_z();

        return internal_computeObservationLikelihoodPointCloud3D(
            sensorAbsPose, &xs[0], &ys[0], &zs[0], N);
    }

    return .0;
}

double DualVoxelPointCloud::internal_computeObservationLikelihoodPointCloud3D(
    const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys,
    const float* zs, const std::size_t num_pts) const
{
    //
    xx;
}

bool DualVoxelPointCloud::internal_canComputeObservationLikelihood(
    const mrpt::obs::CObservation& obs) const
{
    THROW_EXCEPTION("TODO");
}

bool DualVoxelPointCloud::isEmpty() const
{
    // empty if no voxels exist:
    return voxels_.empty();
}

void DualVoxelPointCloud::saveMetricMapRepresentationToFile(
    const std::string& filNamePrefix) const
{
    //
    THROW_EXCEPTION("TODO");
}

void DualVoxelPointCloud::insertPoint(const mrpt::math::TPoint3Df& pt)
{
    // Get voxel indices:
    const index3d_t idxPoint = {
        coord2idx(pt.x), coord2idx(pt.y), coord2idx(pt.z)};

    // 1) Insert into decimation voxel map:
    auto& v = voxels_[idxPoint];
    v.insertPoint(pt);

    // 2) Insert this voxel into the list of neighbors for all
    //    voxels in the nearby volume:
    internalUpdateNNs(idxPoint, v);
}

void DualVoxelPointCloud::internalUpdateNNs(
    const index3d_t& voxelIdxs, const VoxelData& voxel)
{
    // Insert this voxel into the list of neighbors for all
    // voxels in the nearby volume:
    for (int32_t iz = -nn_to_decim_ratio_; iz <= nn_to_decim_ratio_; iz++)
    {
        for (int32_t iy = -nn_to_decim_ratio_; iy <= nn_to_decim_ratio_; iy++)
        {
            for (int32_t ix = -nn_to_decim_ratio_; ix <= nn_to_decim_ratio_;
                 ix++)
            {
                const index3d_t nnIdxs = {
                    voxelIdxs.cx_ + ix, voxelIdxs.cy_ + iy, voxelIdxs.cz_ + iz};

                VoxelNNData& nnNode = voxelsNN_[nnIdxs];
                nnNode.nodes[voxelIdxs].emplace(voxel);  // save reference
            }
        }
    }
}

bool DualVoxelPointCloud::nn_find_nearest(
    const mrpt::math::TPoint3Df& queryPoint, mrpt::math::TPoint3Df& outNearest,
    float& outDistanceSquared)
{
    // Get voxel indices:
    const index3d_t idxPoint = {
        coord2idx(queryPoint.x), coord2idx(queryPoint.y),
        coord2idx(queryPoint.z)};

    auto itNN = voxelsNN_.find(idxPoint);
    if (itNN == voxelsNN_.end()) return false;

    // Keep closest only:
    outDistanceSquared = max_nn_radius_sqr_ * 1.01;  // larger than maximum

    for (const auto& voxelRef : itNN->second.nodes)
    {
        const auto& node = voxelRef.second.value().get();

        float distSqr = .0f;
        for (const auto& pt : node.points())
        {
            distSqr += mrpt::square(queryPoint.x - pt.x);
            if (distSqr > outDistanceSquared) continue;
            distSqr += mrpt::square(queryPoint.y - pt.y);
            if (distSqr > outDistanceSquared) continue;
            distSqr += mrpt::square(queryPoint.z - pt.z);
            if (distSqr > outDistanceSquared) continue;

            // This is better:
            outDistanceSquared = distSqr;
            outNearest         = pt;
        }
    }

    return outDistanceSquared < max_nn_radius_sqr_;
}
