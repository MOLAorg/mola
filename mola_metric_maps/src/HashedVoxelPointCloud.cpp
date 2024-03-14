/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
 * @file   HashedVoxelPointCloud.cpp
 * @brief  Point cloud stored in voxels, in a sparse hash map
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CHistogram.h>
#include <mrpt/math/distributions.h>  // confidenceIntervals()
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/serialization/CArchive.h>  // serialization
#include <mrpt/system/os.h>

#include <cmath>

constexpr size_t HARD_MAX_MATCHES = 3;

//#define USE_DEBUG_PROFILER

#ifdef USE_DEBUG_PROFILER
#include <mrpt/system/CTimeLogger.h>
static mrpt::system::CTimeLogger profiler(true, "HashedVoxelPointCloud");
#endif

using namespace mola;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
    "mola::HashedVoxelPointCloud,HashedVoxelPointCloud",
    mola::HashedVoxelPointCloud)

HashedVoxelPointCloud::TMapDefinition::TMapDefinition() = default;
void HashedVoxelPointCloud::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& s, const std::string& sectionPrefix)
{
    using namespace std::string_literals;

    // [<sectionNamePrefix>+"_creationOpts"]
    const std::string sSectCreation = sectionPrefix + "_creationOpts"s;
    MRPT_LOAD_CONFIG_VAR(voxel_size, float, s, sSectCreation);

    ASSERT_(s.sectionExists(sectionPrefix + "_insertionOpts"s));
    insertionOpts.loadFromConfigFile(s, sectionPrefix + "_insertionOpts"s);

    if (s.sectionExists(sectionPrefix + "_likelihoodOpts"s))
        likelihoodOpts.loadFromConfigFile(
            s, sectionPrefix + "_likelihoodOpts"s);

    if (s.sectionExists(sectionPrefix + "_renderOpts"s))
        renderOpts.loadFromConfigFile(s, sectionPrefix + "_renderOpts"s);
}

void HashedVoxelPointCloud::TMapDefinition::dumpToTextStream_map_specific(
    std::ostream& out) const
{
    LOADABLEOPTS_DUMP_VAR(voxel_size, float);

    insertionOpts.dumpToTextStream(out);
    likelihoodOpts.dumpToTextStream(out);
    renderOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr
    HashedVoxelPointCloud::internal_CreateFromMapDefinition(
        const mrpt::maps::TMetricMapInitializer& _def)
{
    const HashedVoxelPointCloud::TMapDefinition* def =
        dynamic_cast<const HashedVoxelPointCloud::TMapDefinition*>(&_def);
    ASSERT_(def);
    auto obj = HashedVoxelPointCloud::Create(def->voxel_size);

    obj->insertionOptions  = def->insertionOpts;
    obj->likelihoodOptions = def->likelihoodOpts;
    obj->renderOptions     = def->renderOpts;

    return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(HashedVoxelPointCloud, CMetricMap, mola)

// =====================================
// Serialization
// =====================================

uint8_t HashedVoxelPointCloud::serializeGetVersion() const { return 0; }
void    HashedVoxelPointCloud::serializeTo(
    mrpt::serialization::CArchive& out) const
{
    // params:
    out << voxel_size_;
    insertionOptions.writeToStream(out);
    likelihoodOptions.writeToStream(out);
    renderOptions.writeToStream(out);

    // data:
    out.WriteAs<uint32_t>(voxels_.size());
    for (const auto& [idx, voxel] : voxels_)
    {
        out << idx.cx << idx.cy << idx.cz;
        const auto&  pts = voxel.points();
        const size_t N   = pts.size();
        out.WriteAs<uint16_t>(N);
        for (size_t j = 0; j < N; j++) out << pts[j];
    }
}
void HashedVoxelPointCloud::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            // params:
            in >> voxel_size_;

            // clear contents and compute computed fields:
            this->setVoxelProperties(voxel_size_);

            insertionOptions.readFromStream(in);
            likelihoodOptions.readFromStream(in);
            renderOptions.readFromStream(in);

            // data:
            const auto nGrids = in.ReadAs<uint32_t>();
            for (uint32_t i = 0; i < nGrids; i++)
            {
                global_index3d_t idx;
                in >> idx.cx >> idx.cy >> idx.cz;

                auto& grid = voxels_[idx];

                const auto nPts = in.ReadAs<uint16_t>();
                for (size_t j = 0; j < nPts; j++)
                {
                    mrpt::math::TPoint3Df pt;
                    in >> pt;
                    grid.insertPoint(pt);
                }
            }
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };

    // cache reset:
    cached_.reset();
}

// VoxelData

// Ctor:
HashedVoxelPointCloud::HashedVoxelPointCloud(float voxel_size)
{
    setVoxelProperties(voxel_size);
}

HashedVoxelPointCloud::~HashedVoxelPointCloud() = default;

void HashedVoxelPointCloud::setVoxelProperties(float voxel_size)
{
    voxel_size_ = voxel_size;

    // calculated fields:
    voxel_size_inv_ = 1.0f / voxel_size_;
    voxel_size_sqr_ = voxel_size_ * voxel_size_;
    voxelDiagonal_  = mrpt::math::TPoint3Df(1.0f, 1.0f, 1.0f) * voxel_size_;

    // clear all:
    HashedVoxelPointCloud::internal_clear();
}

std::string HashedVoxelPointCloud::asString() const
{
    return mrpt::format(
        "HashedVoxelPointCloud, resolution=%.03f bbox=%s", voxel_size_,
        boundingBox().asString().c_str());
}

void HashedVoxelPointCloud::getVisualizationInto(
    mrpt::opengl::CSetOfObjects& outObj) const
{
    MRPT_START
    if (!genericMapParams.enableSaveAs3DObject) return;

    if (renderOptions.colormap == mrpt::img::cmNONE)
    {
        // Single color:
        auto obj = mrpt::opengl::CPointCloud::Create();

        const auto lambdaVisitPoints = [&obj](const mrpt::math::TPoint3Df& pt) {
            obj->insertPoint(pt);
        };
        this->visitAllPoints(lambdaVisitPoints);

        obj->setColor(renderOptions.color);
        obj->setPointSize(renderOptions.point_size);
        obj->enableColorFromZ(false);
        outObj.insert(obj);
    }
    else
    {
        auto obj = mrpt::opengl::CPointCloudColoured::Create();

        auto bb = this->boundingBox();

        // handle planar maps (avoids error in histogram below):
        for (int i = 0; i < 3; i++)
            if (bb.max[i] - bb.min[i] < 0.1f) bb.max[i] = bb.min[i] + 0.1f;

        // Use a histogram to discard outliers from the colormap extremes:
        constexpr size_t nBins = 100;
        // for x,y,z
        std::array<mrpt::math::CHistogram, 3> hists = {
            mrpt::math::CHistogram(bb.min.x, bb.max.x, nBins),
            mrpt::math::CHistogram(bb.min.y, bb.max.y, nBins),
            mrpt::math::CHistogram(bb.min.z, bb.max.z, nBins)};

        size_t nPoints = 0;

        const auto lambdaVisitPoints =
            [&obj, &hists, &nPoints](const mrpt::math::TPoint3Df& pt) {
                // x y z R G B [A]
                obj->insertPoint({pt.x, pt.y, pt.z, 0, 0, 0});
                for (int i = 0; i < 3; i++) hists[i].add(pt[i]);
                nPoints++;
            };

        this->visitAllPoints(lambdaVisitPoints);

        obj->setPointSize(renderOptions.point_size);

        if (nPoints)
        {
            // Analyze the histograms and get confidence intervals:
            std::vector<double> coords;
            std::vector<double> hits;

            const int idx = renderOptions.recolorizeByCoordinateIndex;
            ASSERT_(idx >= 0 && idx < 3);

            float            min = .0, max = 1.f;
            constexpr double confidenceInterval = 0.02;

            hists[idx].getHistogramNormalized(coords, hits);
            mrpt::math::confidenceIntervalsFromHistogram(
                coords, hits, min, max, confidenceInterval);

            obj->recolorizeByCoordinate(
                min, max, renderOptions.recolorizeByCoordinateIndex,
                renderOptions.colormap);
        }
        outObj.insert(obj);
    }
    MRPT_END
}

void HashedVoxelPointCloud::internal_clear() { voxels_.clear(); }

bool HashedVoxelPointCloud::internal_insertObservation(
    const mrpt::obs::CObservation&                   obs,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
    MRPT_START

    using namespace mrpt::obs;

    mrpt::poses::CPose2D robotPose2D;
    mrpt::poses::CPose3D robotPose3D;

    if (robotPose)
    {
        robotPose2D = mrpt::poses::CPose2D(*robotPose);
        robotPose3D = (*robotPose);

        if (insertionOptions.remove_voxels_farther_than > 0)
        {
            const int distInGrid = static_cast<int>(std::ceil(
                insertionOptions.remove_voxels_farther_than * voxel_size_inv_));

            const auto idxCurObs =
                coordToGlobalIdx(robotPose3D.translation().cast<float>());

            for (auto it = voxels_.begin(); it != voxels_.end();)
            {
                // manhattan distance:
                const int dist = mrpt::max3(
                    std::abs(it->first.cx - idxCurObs.cx),
                    std::abs(it->first.cy - idxCurObs.cy),
                    std::abs(it->first.cz - idxCurObs.cz));

                if (dist > distInGrid)
                    it = voxels_.erase(it);
                else
                    ++it;
            }
        }
    }
    else
    {
        // Default values are (0,0,0)
    }

    if (IS_CLASS(obs, CObservation2DRangeScan))
    {
        /********************************************************************
                    OBSERVATION TYPE: CObservation2DRangeScan
         ********************************************************************/
        const auto& o = static_cast<const CObservation2DRangeScan&>(obs);

        // Build (if not done before) the points map representation of this
        // observation:
        const auto* scanPoints = o.buildAuxPointsMap<mrpt::maps::CPointsMap>();

        if (scanPoints->empty()) return 0;

        const auto& xs = scanPoints->getPointsBufferRef_x();
        const auto& ys = scanPoints->getPointsBufferRef_y();
        const auto& zs = scanPoints->getPointsBufferRef_z();

        internal_insertPointCloud3D(
            robotPose3D, xs.data(), ys.data(), zs.data(), xs.size());

        return true;
    }
    else if (IS_CLASS(obs, CObservation3DRangeScan))
    {
        /********************************************************************
                    OBSERVATION TYPE: CObservation3DRangeScan
         ********************************************************************/
        const auto& o = static_cast<const CObservation3DRangeScan&>(obs);

        mrpt::obs::T3DPointsProjectionParams pp;
        pp.takeIntoAccountSensorPoseOnRobot = true;

        // Empty point set, or load from XYZ in observation:
        if (o.hasPoints3D)
        {
            for (size_t i = 0; i < o.points3D_x.size(); i++)
                this->insertPoint(robotPose3D.composePoint(
                    {o.points3D_x[i], o.points3D_y[i], o.points3D_z[i]}));

            return true;
        }
        else if (o.hasRangeImage)
        {
            mrpt::maps::CSimplePointsMap pointMap;
            const_cast<CObservation3DRangeScan&>(o).unprojectInto(pointMap, pp);

            const auto& xs = pointMap.getPointsBufferRef_x();
            const auto& ys = pointMap.getPointsBufferRef_y();
            const auto& zs = pointMap.getPointsBufferRef_z();

            internal_insertPointCloud3D(
                robotPose3D, xs.data(), ys.data(), zs.data(), xs.size());

            return true;
        }
        else
            return false;
    }
    else if (IS_CLASS(obs, CObservationVelodyneScan))
    {
        /********************************************************************
                    OBSERVATION TYPE: CObservationVelodyneScan
         ********************************************************************/
        const auto& o = static_cast<const CObservationVelodyneScan&>(obs);

        // Automatically generate pointcloud if needed:
        if (!o.point_cloud.size())
            const_cast<CObservationVelodyneScan&>(o).generatePointCloud();

        for (size_t i = 0; i < o.point_cloud.x.size(); i++)
        {
            insertPoint(robotPose3D.composePoint(
                {o.point_cloud.x[i], o.point_cloud.y[i], o.point_cloud.z[i]}));
        }

        return true;
    }
    else if (IS_CLASS(obs, CObservationPointCloud))
    {
        const auto& o = static_cast<const CObservationPointCloud&>(obs);
        ASSERT_(o.pointcloud);

        const auto& xs = o.pointcloud->getPointsBufferRef_x();
        const auto& ys = o.pointcloud->getPointsBufferRef_y();
        const auto& zs = o.pointcloud->getPointsBufferRef_z();

        for (size_t i = 0; i < xs.size(); i++)
        {
            insertPoint(robotPose3D.composePoint({xs[i], ys[i], zs[i]}));
        }

        return true;
    }
    else
    {
        /********************************************************************
                    OBSERVATION TYPE: Unknown
        ********************************************************************/
        return false;
    }

    MRPT_END
}

double HashedVoxelPointCloud::internal_computeObservationLikelihood(
    const mrpt::obs::CObservation& obs,
    const mrpt::poses::CPose3D&    takenFrom) const
{
    using namespace mrpt::obs;
    using namespace mrpt::poses;
    using namespace mrpt::maps;

    if (isEmpty()) return 0;

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

        const size_t N = scanPoints->size();
        if (!N) return 0;

        const auto& xs = scanPoints->getPointsBufferRef_x();
        const auto& ys = scanPoints->getPointsBufferRef_y();
        const auto& zs = scanPoints->getPointsBufferRef_z();

        return internal_computeObservationLikelihoodPointCloud3D(
            takenFrom, xs.data(), ys.data(), zs.data(), N);
    }
    else if (IS_CLASS(obs, CObservationVelodyneScan))
    {
        const auto& o = dynamic_cast<const CObservationVelodyneScan&>(obs);

        // Automatically generate pointcloud if needed:
        if (!o.point_cloud.size())
            const_cast<CObservationVelodyneScan&>(o).generatePointCloud();

        const size_t N = o.point_cloud.size();
        if (!N) return 0;

        const CPose3D sensorAbsPose = takenFrom + o.sensorPose;

        const auto& xs = o.point_cloud.x;
        const auto& ys = o.point_cloud.y;
        const auto& zs = o.point_cloud.z;

        return internal_computeObservationLikelihoodPointCloud3D(
            sensorAbsPose, xs.data(), ys.data(), zs.data(), N);
    }
    else if (IS_CLASS(obs, CObservationPointCloud))
    {
        const auto& o = dynamic_cast<const CObservationPointCloud&>(obs);

        const size_t N = o.pointcloud->size();
        if (!N) return 0;

        const CPose3D sensorAbsPose = takenFrom + o.sensorPose;

        auto xs = o.pointcloud->getPointsBufferRef_x();
        auto ys = o.pointcloud->getPointsBufferRef_y();
        auto zs = o.pointcloud->getPointsBufferRef_z();

        return internal_computeObservationLikelihoodPointCloud3D(
            sensorAbsPose, xs.data(), ys.data(), zs.data(), N);
    }

    return .0;
}

double HashedVoxelPointCloud::internal_computeObservationLikelihoodPointCloud3D(
    const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys,
    const float* zs, const std::size_t num_pts) const
{
    MRPT_TRY_START

    ASSERT_GT_(likelihoodOptions.sigma_dist, .0);

    mrpt::math::TPoint3Df closest;
    float                 closest_err;
    uint64_t              closest_id;
    const float max_sqr_err = mrpt::square(likelihoodOptions.max_corr_distance);
    double      sumSqrDist  = .0;

    std::size_t nPtsForAverage = 0;
    for (std::size_t i = 0; i < num_pts;
         i += likelihoodOptions.decimation, nPtsForAverage++)
    {
        // Transform the point from the scan reference to its global 3D
        // position:
        const auto gPt = pc_in_map.composePoint({xs[i], ys[i], zs[i]});

        const bool found =
            nn_single_search(gPt, closest, closest_err, closest_id);
        if (!found) continue;

        // Put a limit:
        mrpt::keep_min(closest_err, max_sqr_err);

        sumSqrDist += static_cast<double>(closest_err);
    }
    if (nPtsForAverage) sumSqrDist /= nPtsForAverage;

    // Log-likelihood:
    return -sumSqrDist / likelihoodOptions.sigma_dist;

    MRPT_TRY_END
}

bool HashedVoxelPointCloud::internal_canComputeObservationLikelihood(
    const mrpt::obs::CObservation& obs) const
{
    using namespace mrpt::obs;

    return IS_CLASS(obs, CObservation2DRangeScan) ||
           IS_CLASS(obs, CObservationVelodyneScan) ||
           IS_CLASS(obs, CObservationPointCloud);
}

bool HashedVoxelPointCloud::isEmpty() const
{
    // empty if no voxels exist:
    return voxels_.empty();
}

void HashedVoxelPointCloud::saveMetricMapRepresentationToFile(
    const std::string& filNamePrefix) const
{
    using namespace std::string_literals;

    const auto fil = filNamePrefix + ".txt"s;
    saveToTextFile(fil);
}

bool HashedVoxelPointCloud::saveToTextFile(const std::string& file) const
{
    FILE* f = mrpt::system::os::fopen(file.c_str(), "wt");
    if (!f) return false;

    const auto lambdaVisitPoints = [f](const mrpt::math::TPoint3Df& pt) {
        mrpt::system::os::fprintf(f, "%f %f %f\n", pt.x, pt.y, pt.z);
    };

    this->visitAllPoints(lambdaVisitPoints);

    mrpt::system::os::fclose(f);
    return true;
}

void HashedVoxelPointCloud::insertPoint(const mrpt::math::TPoint3Df& pt)
{
    auto& v = *voxelByCoords(pt, true /*create if new*/);

    const auto nPreviousPoints = v.points().size();

    if (insertionOptions.max_points_per_voxel != 0 &&
        nPreviousPoints >= insertionOptions.max_points_per_voxel)
        return;  // skip, voxel is full

    if (insertionOptions.min_distance_between_points > 0 &&
        nPreviousPoints != 0)
    {
        // Look for the closest existing point in the voxel:
        std::optional<float> curClosestDistSqr;

        const auto& pts = v.points();
        for (size_t i = 0; i < pts.size(); i++)
        {
            const float distSqr = pts[i].sqrDistanceTo(pt);
            if (!curClosestDistSqr.has_value() ||
                distSqr < curClosestDistSqr.value())
                curClosestDistSqr = distSqr;
        }
        const float minDistSqr =
            mrpt::square(insertionOptions.min_distance_between_points);

        // Skip if the point is too close to existing ones:
        if (curClosestDistSqr.value() < minDistSqr) return;
    }

    v.insertPoint(pt);

    // Also, update bbox:
    if (!cached_.boundingBox_.has_value())
        cached_.boundingBox_.emplace(pt, pt);
    else
        cached_.boundingBox_->updateWithPoint(pt);
}

bool HashedVoxelPointCloud::nn_has_indices_or_ids() const
{  // false: IDs, not contiguous indices
    return false;
}

size_t HashedVoxelPointCloud::nn_index_count() const
{  // Not used.
    return 0;
}

bool HashedVoxelPointCloud::nn_single_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df& query,
    [[maybe_unused]] mrpt::math::TPoint2Df&       result,
    [[maybe_unused]] float&                       out_dist_sqr,
    [[maybe_unused]] uint64_t&                    resultIndexOrID) const
{
    THROW_EXCEPTION("Cannot run a 2D search on a HashedVoxelPointCloud");
}
void HashedVoxelPointCloud::nn_multiple_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df&        query,
    [[maybe_unused]] const size_t                        N,
    [[maybe_unused]] std::vector<mrpt::math::TPoint2Df>& results,
    [[maybe_unused]] std::vector<float>&                 out_dists_sqr,
    [[maybe_unused]] std::vector<uint64_t>& resultIndicesOrIDs) const
{
    THROW_EXCEPTION("Cannot run a 2D search on a HashedVoxelPointCloud");
}
void HashedVoxelPointCloud::nn_radius_search(
    [[maybe_unused]] const mrpt::math::TPoint2Df&        query,
    [[maybe_unused]] const float                         search_radius_sqr,
    [[maybe_unused]] std::vector<mrpt::math::TPoint2Df>& results,
    [[maybe_unused]] std::vector<float>&                 out_dists_sqr,
    [[maybe_unused]] std::vector<uint64_t>&              resultIndicesOrIDs,
    [[maybe_unused]] size_t                              maxPoints) const
{
    THROW_EXCEPTION("Cannot run a 2D search on a HashedVoxelPointCloud");
}

bool HashedVoxelPointCloud::nn_single_search(
    const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result,
    float& out_dist_sqr, uint64_t& resultIndexOrID) const
{
    std::vector<mrpt::math::TPoint3Df> r;
    std::vector<float>                 dist_sqr;
    std::vector<uint64_t>              resultIndices;
    nn_multiple_search_impl<1>(query, 1, r, dist_sqr, resultIndices);
    if (r.empty()) return false;  // none found
    result          = r[0];
    out_dist_sqr    = dist_sqr[0];
    resultIndexOrID = resultIndices[0];
    return true;
}

void HashedVoxelPointCloud::nn_multiple_search(
    const mrpt::math::TPoint3Df& query, const size_t N,
    std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>&                 out_dists_sqr,
    std::vector<uint64_t>&              resultIndicesOrIDs) const
{
    nn_multiple_search_impl<HARD_MAX_MATCHES>(
        query, N, results, out_dists_sqr, resultIndicesOrIDs);
}

template <size_t MAX_KNN>
void HashedVoxelPointCloud::nn_multiple_search_impl(
    const mrpt::math::TPoint3Df& query, const size_t N,
    std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>&                 out_dists_sqr,
    std::vector<uint64_t>&              resultIndicesOrIDs) const
{
    results.clear();
    out_dists_sqr.clear();
    resultIndicesOrIDs.clear();

    ASSERT_(N > 0);

    const global_index3d_t idxs0 =
        coordToGlobalIdx(query) - global_index3d_t(1, 1, 1);
    const global_index3d_t idxs1 =
        coordToGlobalIdx(query) + global_index3d_t(1, 1, 1);

    // Data structures to avoid ANY heap memory allocation and keep working
    // on the stack at all time:
    struct Match
    {
        mrpt::math::TPoint3Df globalPt;
        float                 sqrDist;
        uint64_t              id;
    };
    std::array<Match, MAX_KNN> matches;  // sorted by sqrDist!
    size_t                     foundMatches = 0;

    auto lambdaProcessCandidate = [&](const float                  sqrDist,
                                      const mrpt::math::TPoint3Df& pt,
                                      const global_plain_index_t&  id) {
        // bubble sort (yes, really!)
        // found its position in the list:
        size_t i = 0;
        for (i = 0; i < foundMatches; i++)
        {
            if (sqrDist < matches[i].sqrDist) break;
        }
        if (i >= MAX_KNN) return;

        // insert new one at [i], shift [i+1:end] one position.
        const size_t last = std::min(foundMatches + 1, MAX_KNN);
        for (size_t j = i + 1; j < last; j++) matches[j] = matches[j - 1];

        matches[i].globalPt = pt;
        matches[i].id       = id;
        matches[i].sqrDist  = sqrDist;

        if (foundMatches < MAX_KNN) foundMatches++;
    };

    auto lambdaCheckCell = [&](const global_index3d_t& p) {
        if (auto* v = voxelByGlobalIdxs(p); v && !v->points().empty())
        {
            const auto& pts = v->points();
            for (size_t i = 0; i < pts.size(); i++)
            {
                const auto& pt      = pts[i];
                float       distSqr = (pt - query).sqrNorm();
                const auto  id      = g2plain(p, i);

                lambdaProcessCandidate(distSqr, pt, id);
            }
        }
    };

    for (int32_t cx = idxs0.cx; cx <= idxs1.cx; cx++)
        for (int32_t cy = idxs0.cy; cy <= idxs1.cy; cy++)
            for (int32_t cz = idxs0.cz; cz <= idxs1.cz; cz++)
                lambdaCheckCell({cx, cy, cz});

    for (size_t i = 0; i < std::min<size_t>(N, foundMatches); i++)
    {
        const auto& m = matches[i];

        out_dists_sqr.push_back(m.sqrDist);
        results.push_back(m.globalPt);
        resultIndicesOrIDs.push_back(m.id);
    }
}

void HashedVoxelPointCloud::nn_radius_search(
    const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>&                 out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
    results.clear();
    out_dists_sqr.clear();
    resultIndicesOrIDs.clear();

    if (search_radius_sqr <= 0) return;

    const float radius   = std::sqrt(search_radius_sqr);
    const auto  diagonal = mrpt::math::TPoint3Df(1.0f, 1.0f, 1.0f) * radius;

    const global_index3d_t idxs0 = coordToGlobalIdx(query - diagonal);
    const global_index3d_t idxs1 = coordToGlobalIdx(query + diagonal);

    // Data structures to avoid ANY heap memory allocation and keep working
    // on the stack at all time:
    struct Match
    {
        mrpt::math::TPoint3Df globalPt;
        float                 sqrDist;
        uint64_t              id;
    };
    std::array<Match, HARD_MAX_MATCHES> matches;  // sorted by sqrDist!
    size_t                              foundMatches = 0;

    auto lambdaProcessCandidate = [&](const float                  sqrDist,
                                      const mrpt::math::TPoint3Df& pt,
                                      const global_plain_index_t&  id) {
        // bubble sort (yes, really!)
        // found its position in the list:
        size_t i = 0;
        for (i = 0; i < foundMatches; i++)
        {
            if (sqrDist < matches[i].sqrDist) break;
        }
        if (i >= HARD_MAX_MATCHES) return;

        // insert new one at [i], shift [i+1:end] one position.
        const size_t last = std::min(foundMatches + 1, HARD_MAX_MATCHES);
        for (size_t j = i + 1; j < last; j++) matches[j] = matches[j - 1];

        matches[i].globalPt = pt;
        matches[i].id       = id;
        matches[i].sqrDist  = sqrDist;

        if (foundMatches < HARD_MAX_MATCHES) foundMatches++;
    };

    auto lambdaCheckCell = [&](const global_index3d_t& p) {
        if (auto* v = voxelByGlobalIdxs(p); v && !v->points().empty())
        {
            const auto& pts = v->points();
            for (size_t i = 0; i < pts.size(); i++)
            {
                const auto& pt      = pts[i];
                float       distSqr = (pt - query).sqrNorm();
                if (distSqr > search_radius_sqr) continue;

                const auto id = g2plain(p, i);

                if (maxPoints != 0)
                {
                    // temporary list:
                    lambdaProcessCandidate(distSqr, pt, id);
                }
                else
                {
                    // directly save output:
                    out_dists_sqr.push_back(distSqr);
                    results.push_back(pt);
                    resultIndicesOrIDs.push_back(id);
                }
            }
        }
    };

    for (int32_t cx = idxs0.cx; cx <= idxs1.cx; cx++)
        for (int32_t cy = idxs0.cy; cy <= idxs1.cy; cy++)
            for (int32_t cz = idxs0.cz; cz <= idxs1.cz; cz++)
                lambdaCheckCell({cx, cy, cz});

    if (maxPoints != 0)
    {
        // we saved results in a temporary buffer, save them out now:
        for (size_t i = 0; i < std::min<size_t>(maxPoints, foundMatches); i++)
        {
            const auto& m = matches[i];

            out_dists_sqr.push_back(m.sqrDist);
            results.push_back(m.globalPt);
            resultIndicesOrIDs.push_back(m.id);
        }
    }
}

mrpt::math::TBoundingBoxf HashedVoxelPointCloud::boundingBox() const
{
    if (!cached_.boundingBox_)
    {
        cached_.boundingBox_.emplace();
        if (this->isEmpty())
        {
            cached_.boundingBox_->min = {0, 0, 0};
            cached_.boundingBox_->max = {0, 0, 0};
        }
        else
        {
            cached_.boundingBox_ =
                mrpt::math::TBoundingBoxf::PlusMinusInfinity();

            auto lambdaForEachVoxel =
                [this](const global_index3d_t& idxs, const VoxelData&) {
                    const mrpt::math::TPoint3Df voxelCorner =
                        globalIdxToCoord(idxs);

                    cached_.boundingBox_->updateWithPoint(voxelCorner);
                    cached_.boundingBox_->updateWithPoint(
                        voxelCorner + voxelDiagonal_);
                };

            this->visitAllVoxels(lambdaForEachVoxel);
        }
    }

    return cached_.boundingBox_.value();
}

void HashedVoxelPointCloud::visitAllPoints(
    const std::function<void(const mrpt::math::TPoint3Df&)>& f) const
{
    for (const auto& [idx, v] : voxels_)
    {
        const auto& pts = v.points();
        for (size_t i = 0; i < pts.size(); i++) f(pts[i]);
    }
}

void HashedVoxelPointCloud::visitAllVoxels(
    const std::function<void(const global_index3d_t&, const VoxelData&)>& f)
    const
{
    for (const auto& [idx, v] : voxels_) f(idx, v);
}

// ========== Option structures ==========
void HashedVoxelPointCloud::TInsertionOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
    const int8_t version = 0;
    out << version;
    out << max_points_per_voxel << remove_voxels_farther_than
        << min_distance_between_points;
}

void HashedVoxelPointCloud::TInsertionOptions::readFromStream(
    mrpt::serialization::CArchive& in)
{
    int8_t version;
    in >> version;
    switch (version)
    {
        case 0:
        {
            in >> max_points_per_voxel >> remove_voxels_farther_than >>
                min_distance_between_points;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    }
}

void HashedVoxelPointCloud::TLikelihoodOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
    const int8_t version = 0;
    out << version;
    out << sigma_dist << max_corr_distance << decimation;
}

void HashedVoxelPointCloud::TLikelihoodOptions::readFromStream(
    mrpt::serialization::CArchive& in)
{
    int8_t version;
    in >> version;
    switch (version)
    {
        case 0:
        {
            in >> sigma_dist >> max_corr_distance >> decimation;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    }
}

void HashedVoxelPointCloud::TRenderOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
    const int8_t version = 0;
    out << version;
    out << point_size << color << int8_t(colormap)
        << recolorizeByCoordinateIndex;
}

void HashedVoxelPointCloud::TRenderOptions::readFromStream(
    mrpt::serialization::CArchive& in)
{
    int8_t version;
    in >> version;
    switch (version)
    {
        case 0:
        {
            in >> point_size;
            in >> this->color;
            in.ReadAsAndCastTo<int8_t>(this->colormap);
            in >> recolorizeByCoordinateIndex;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    }
}

void HashedVoxelPointCloud::TInsertionOptions::dumpToTextStream(
    std::ostream& out) const
{
    out << "\n------ [HashedVoxelPointCloud::TInsertionOptions] ------- "
           "\n\n";

    LOADABLEOPTS_DUMP_VAR(max_points_per_voxel, int);
    LOADABLEOPTS_DUMP_VAR(remove_voxels_farther_than, double);
    LOADABLEOPTS_DUMP_VAR(min_distance_between_points, float)
}

void HashedVoxelPointCloud::TLikelihoodOptions::dumpToTextStream(
    std::ostream& out) const
{
    out << "\n------ [HashedVoxelPointCloud::TLikelihoodOptions] ------- "
           "\n\n";

    LOADABLEOPTS_DUMP_VAR(sigma_dist, double);
    LOADABLEOPTS_DUMP_VAR(max_corr_distance, double);
    LOADABLEOPTS_DUMP_VAR(decimation, int);
}

void HashedVoxelPointCloud::TRenderOptions::dumpToTextStream(
    std::ostream& out) const
{
    out << "\n------ [HashedVoxelPointCloud::TRenderOptions] ------- \n\n";

    LOADABLEOPTS_DUMP_VAR(point_size, float);
    LOADABLEOPTS_DUMP_VAR(color.R, float);
    LOADABLEOPTS_DUMP_VAR(color.G, float);
    LOADABLEOPTS_DUMP_VAR(color.B, float);
    LOADABLEOPTS_DUMP_VAR(colormap, int);
    LOADABLEOPTS_DUMP_VAR(recolorizeByCoordinateIndex, int);
}

void HashedVoxelPointCloud::TInsertionOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
    MRPT_LOAD_CONFIG_VAR(max_points_per_voxel, int, c, s);
    MRPT_LOAD_CONFIG_VAR(remove_voxels_farther_than, double, c, s);
    MRPT_LOAD_CONFIG_VAR(min_distance_between_points, float, c, s);
}

void HashedVoxelPointCloud::TLikelihoodOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
    MRPT_LOAD_CONFIG_VAR(sigma_dist, double, c, s);
    MRPT_LOAD_CONFIG_VAR(max_corr_distance, double, c, s);
    MRPT_LOAD_CONFIG_VAR(decimation, int, c, s);
}

void HashedVoxelPointCloud::TRenderOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
    MRPT_LOAD_CONFIG_VAR(point_size, float, c, s);
    MRPT_LOAD_CONFIG_VAR(color.R, float, c, s);
    MRPT_LOAD_CONFIG_VAR(color.G, float, c, s);
    MRPT_LOAD_CONFIG_VAR(color.B, float, c, s);
    colormap = c.read_enum(s, "colormap", this->colormap);
    MRPT_LOAD_CONFIG_VAR(recolorizeByCoordinateIndex, int, c, s);
}

void HashedVoxelPointCloud::internal_insertPointCloud3D(
    const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys,
    const float* zs, const std::size_t num_pts)
{
    MRPT_TRY_START

    voxels_.reserve(voxels_.size() + num_pts);

    for (std::size_t i = 0; i < num_pts; i++)
    {
        // Transform the point from the scan reference to its global 3D
        // position:
        const auto gPt = pc_in_map.composePoint({xs[i], ys[i], zs[i]});
        insertPoint(gPt);
    }

    MRPT_TRY_END
}

const mrpt::maps::CSimplePointsMap*
    HashedVoxelPointCloud::getAsSimplePointsMap() const
{
    if (!cachedPoints_) cachedPoints_ = mrpt::maps::CSimplePointsMap::Create();

    cachedPoints_->clear();

    this->visitAllPoints([this](const mrpt::math::TPoint3Df& p) {
        cachedPoints_->insertPointFast(p.x, p.y, p.z);
    });

    return cachedPoints_.get();
}
