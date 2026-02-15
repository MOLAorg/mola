/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   NDT.h
 * @brief  NDT 3D map representation
 * @author Jose Luis Blanco Claraco
 * @date   Aug 22, 2024
 */
#pragma once

#include <mola_metric_maps/index3d_t.h>
#include <mp2p_icp/NearestPlaneCapable.h>
#include <mp2p_icp/estimate_points_eigen.h>  // PointCloudEigen
#include <mrpt/core/round.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>
#include <tsl/robin_map.h>

#include <array>
#include <functional>
#include <optional>

// #define HASHED_VOXEL_POINT_CLOUD_WITH_CACHED_ACCESS

namespace mola
{
/** 3D NDT maps: Voxels with points approximated by Gaussians.
 * Implementation of \cite magnusson2007scan.
 *
 * This implements two nearest-neighbor virtual APIs:
 * - mrpt::maps::NearestNeighborsCapable: It will return pairings of local
 *   points against global (this map) points in voxels that do not conform
 *   planes.
 *
 * - mp2p_icp::NearestPlaneCapable: It will return points-to-plane pairings,
 *   if the local point falls within a voxel with points conforming a plane.
 *
 */
class NDT : public mrpt::maps::CMetricMap,
            public mrpt::maps::NearestNeighborsCapable,
            public mp2p_icp::NearestPlaneCapable
{
  DEFINE_SERIALIZABLE(NDT, mola)
 public:
  NDT(const NDT&)            = default;
  NDT& operator=(const NDT&) = default;
  NDT(NDT&&)                 = default;
  NDT& operator=(NDT&&)      = default;

  using global_index3d_t = index3d_t<int32_t>;

  /** @name Indices and coordinates
   *  @{ */

  constexpr static size_t      MAX_POINTS_PER_VOXEL        = 16;
  constexpr static std::size_t GLOBAL_ID_SUBVOXEL_BITCOUNT = 5;
  static_assert(MAX_POINTS_PER_VOXEL <= (1 << GLOBAL_ID_SUBVOXEL_BITCOUNT));

  inline global_index3d_t coordToGlobalIdx(const mrpt::math::TPoint3Df& pt) const
  {
    return global_index3d_t(
        static_cast<int32_t>(pt.x * voxel_size_inv_),  //
        static_cast<int32_t>(pt.y * voxel_size_inv_),  //
        static_cast<int32_t>(pt.z * voxel_size_inv_));
  }

  /// returns the coordinate of the voxel "bottom" corner
  inline mrpt::math::TPoint3Df globalIdxToCoord(const global_index3d_t idx) const
  {
    return {
        static_cast<float>(idx.cx) * voxel_size_,  //
        static_cast<float>(idx.cy) * voxel_size_,  //
        static_cast<float>(idx.cz) * voxel_size_};
  }

  /// collapsed plain unique ID for global indices
  using global_plain_index_t = uint64_t;

  static inline global_plain_index_t g2plain(const global_index3d_t& g, int subVoxelIndex = 0)
  {
    constexpr uint64_t SUBVOXEL_MASK = ((1 << GLOBAL_ID_SUBVOXEL_BITCOUNT) - 1);
    constexpr auto     OFF           = GLOBAL_ID_SUBVOXEL_BITCOUNT;
    constexpr int      FBITS         = 20;  // (64 - OFF)/3, rounded if needed
    constexpr uint64_t FMASK         = (1 << FBITS) - 1;

    return (static_cast<uint64_t>(subVoxelIndex) & SUBVOXEL_MASK) |
           (static_cast<uint64_t>(g.cx & FMASK) << (OFF + FBITS * 0)) |
           (static_cast<uint64_t>(g.cy & FMASK) << (OFF + FBITS * 1)) |
           (static_cast<uint64_t>(g.cz & FMASK) << (OFF + FBITS * 2));
  }
  /** @} */

  /** @name Basic API for construction and main parameters
   *  @{ */

  /**
   * @brief Constructor / default ctor
   * @param voxel_size Voxel size [meters]
   */
  NDT(float voxel_size = 5.00f);

  ~NDT();

  /** Reset the main voxel parameters, and *clears* all current map contents
   */
  void setVoxelProperties(float voxel_size);

  /** @} */

  /** @name Data structures
   *  @{ */

  struct point_vector_t
  {
    std::array<float, MAX_POINTS_PER_VOXEL> xs, ys, zs;
  };

  /// Returns true if the fitted Gaussian fulfills the requirements of being
  /// planar enough, according to the insertionOptions parameters
  bool ndt_is_plane(const mp2p_icp::PointCloudEigen& ndt) const;

  struct VoxelData
  {
   public:
    VoxelData() = default;

    struct PointSpan
    {
      PointSpan(const point_vector_t& points, uint32_t n) : points_(points), n_(n) {}

      size_t size() const { return n_; }
      bool   empty() const { return n_ == 0; }

      mrpt::math::TPoint3Df operator[](int i) const
      {
        return {points_.xs[i], points_.ys[i], points_.zs[i]};
      }

     private:
      const point_vector_t& points_;
      const uint32_t        n_;
    };

    auto points() const -> PointSpan { return PointSpan(points_, nPoints_); }

    void insertPoint(
        const mrpt::math::TPoint3Df& p, const mrpt::math::TPoint3Df& sensorPose)  // NOLINT
    {
      if (nPoints_ < MAX_POINTS_PER_VOXEL)
      {
        points_.xs[nPoints_] = p.x;
        points_.ys[nPoints_] = p.y;
        points_.zs[nPoints_] = p.z;
        nPoints_++;
        ndt_.reset();
      }
      was_seen_from_ = sensorPose;
    }

    /// Gets cached NDT for this voxel, or computes it right now.
    /// If there are no points enough, nullopt is returned
    const std::optional<mp2p_icp::PointCloudEigen>& ndt() const;

    bool has_ndt() const { return ndt_.has_value(); }

    // for serialization, do not use in normal use:
    size_t size() const { return nPoints_; }

    const auto& was_seen_from() const { return was_seen_from_; }

   private:
    mutable std::optional<mp2p_icp::PointCloudEigen> ndt_;
    point_vector_t                                   points_{};
    std::optional<mrpt::math::TPoint3Df>             was_seen_from_;
    uint32_t                                         nPoints_ = 0;
  };

  using grids_map_t = tsl::robin_map<global_index3d_t, VoxelData, index3d_hash<int32_t>>;

  /** @} */

  /** @name Data access API
   *  @{ */
  // clear(): available in base class

  /** returns the voxeldata by global index coordinates, creating it or not if
   * not found depending on createIfNew.
   * Returns nullptr if not found and createIfNew is false
   *
   * Function defined in the header file so compilers can optimize
   * for literals "createIfNew"
   */
  inline VoxelData* voxelByGlobalIdxs(const global_index3d_t& idx, bool createIfNew)
  {
    // 1) Insert into decimation voxel map:
    VoxelData* voxel = nullptr;

#if defined(HASHED_VOXEL_POINT_CLOUD_WITH_CACHED_ACCESS)
    for (int i = 0; i < CachedData::NUM_CACHED_IDXS; i++)
    {
      if (cached_.lastAccessVoxel[i] && cached_.lastAccessIdx[i] == idx)
      {
        // Cache hit:
#ifdef USE_DEBUG_PROFILER
        mrpt::system::CTimeLoggerEntry tle(profiler, "insertPoint.cache_hit");
#endif
        voxel = cached_.lastAccessVoxel[i];
        break;
      }
    }

    if (!voxel)
    {
#endif
#ifdef USE_DEBUG_PROFILER
      mrpt::system::CTimeLoggerEntry tle(profiler, "insertPoint.cache_misss");
#endif

      auto it = voxels_.find(idx);
      if (it == voxels_.end())
      {
        if (!createIfNew)
        {
          return nullptr;
        }

        voxel = &voxels_[idx];  // Create it
      }
      else
      {
        // Use the found grid
        voxel = &it.value();
      }
#if defined(HASHED_VOXEL_POINT_CLOUD_WITH_CACHED_ACCESS)
      // Add to cache:
      cached_.lastAccessIdx[cached_.lastAccessNextWrite]   = idx;
      cached_.lastAccessVoxel[cached_.lastAccessNextWrite] = voxel;
      cached_.lastAccessNextWrite++;
      cached_.lastAccessNextWrite &= CachedData::NUM_CACHED_IDX_MASK;
    }
#endif
    return voxel;
  }

  /// \overload (const version)
  const VoxelData* voxelByGlobalIdxs(const global_index3d_t& idx  //
                                     /*, bool createIfNew this must be false for const! */) const
  {  // reuse the non-const method:
    return const_cast<NDT*>(this)->voxelByGlobalIdxs(idx, false);
  }

  /** Get a voxeldata by (x,y,z) coordinates, **creating** the voxel if
   * needed. */
  VoxelData* voxelByCoords(const mrpt::math::TPoint3Df& pt, bool createIfNew)
  {
    return voxelByGlobalIdxs(coordToGlobalIdx(pt), createIfNew);
  }

  /// \overload const version. Returns nullptr if voxel does not exist
  const VoxelData* voxelByCoords(const mrpt::math::TPoint3Df& pt) const
  {
    return const_cast<NDT*>(this)->voxelByCoords(pt, false);
  }

  /** Insert one point into the map. The sensor pose is used to estimate the
   * outward surface normal */
  void insertPoint(const mrpt::math::TPoint3Df& pt, const mrpt::math::TPoint3Df& sensorPose);

  const grids_map_t& voxels() const { return voxels_; }

  /** Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if
   * there are no points. Results are cached unless the map is somehow
   * modified to avoid repeated calculations.
   */
  mrpt::math::TBoundingBoxf boundingBox() const override;

  void visitAllPoints(const std::function<void(const mrpt::math::TPoint3Df&)>& f) const;

  void visitAllVoxels(
      const std::function<void(const global_index3d_t&, const VoxelData&)>& f) const;

  /** Save to a text file. Each line contains "X Y Z" point coordinates.
   *  Returns false if any error ocurred, true elsewere.
   */
  bool saveToTextFile(const std::string& file) const;

  /** @} */

  /** @name API of the NearestNeighborsCapable virtual interface
  @{ */
  [[nodiscard]] bool   nn_has_indices_or_ids() const override;
  [[nodiscard]] size_t nn_index_count() const override;
  [[nodiscard]] bool   nn_single_search(
        const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
        uint64_t& resultIndexOrID) const override;
  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result, float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override;
  void nn_multiple_search(
      const mrpt::math::TPoint3Df& query, const size_t N,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const override;
  void nn_multiple_search(
      const mrpt::math::TPoint2Df& query, const size_t N,
      std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const override;
  void nn_radius_search(
      const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const override;
  void nn_radius_search(
      const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
      std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const override;

  template <size_t MAX_KNN>
  void nn_multiple_search_impl(
      const mrpt::math::TPoint3Df& query, const size_t N,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const;

  /** @} */

  /** @name Public virtual methods implementation for NearestPlaneCapable
   *  @{ */
  NearestPlaneResult nn_search_pt2pl(
      const mrpt::math::TPoint3Df& point, const float max_search_distance) const override;
  /** @} */

  /** @name Public virtual methods implementation for CMetricMap
   *  @{ */

  /** Returns a short description of the map. */
  std::string asString() const override;

  void getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const override;

  /** Returns true if the map is empty */
  bool isEmpty() const override;

  /** This virtual method saves the map to a file "filNamePrefix"+<
   * some_file_extension >, as an image or in any other applicable way (Notice
   * that other methods to save the map may be implemented in classes
   * implementing this virtual interface).  */
  void saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const override;

  /// Returns a cached point cloud view of the hash map.
  /// Not efficient at all. Only for MOLA->ROS2 bridge.
  const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() const override;

  /** @} */

  /** Options for insertObservation()
   */
  struct TInsertionOptions : public mrpt::config::CLoadableOptions
  {
    TInsertionOptions() = default;

    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source,
        const std::string&                   section) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);

    /** Maximum number of points per voxel. 0 means no limit further than
     * the hard compile-time limit.
     */
    uint32_t max_points_per_voxel = 0;

    /** If !=0, remove the voxels farther (L1 distance) than this
     * distance, in meters. */
    double remove_voxels_farther_than = .0;

    /** If !=0 skip the insertion of points that are closer than this
     * distance to any other already in the voxel. */
    float min_distance_between_points = .0f;

    double max_eigen_ratio_for_planes = 0.01;
  };
  TInsertionOptions insertionOptions;

  /** Options used when evaluating "computeObservationLikelihood" in the
   * derived classes.
   * \sa CObservation::computeObservationLikelihood
   */
  struct TLikelihoodOptions : public mrpt::config::CLoadableOptions
  {
    TLikelihoodOptions() = default;

    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source,
        const std::string&                   section) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);

    /** Sigma (standard deviation, in meters) of the Gaussian observation
     *  model used to model the likelihood (default= 0.5 m) */
    double sigma_dist = 0.5;

    /** Maximum distance in meters to consider for the numerator divided by
     * "sigma_dist", so that each point has a minimum (but very small)
     * likelihood to avoid underflows (default=1.0 meters) */
    double max_corr_distance = 1.0;

    /** Speed up the likelihood computation by considering only one out of N
     * rays (default=10) */
    uint32_t decimation = 10;
  };
  TLikelihoodOptions likelihoodOptions;

  /** Rendering options, used in getAs3DObject()
   */
  struct TRenderOptions : public mrpt::config::CLoadableOptions
  {
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source,
        const std::string&                   section) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    /** Binary dump to stream - used in derived classes' serialization */
    void writeToStream(mrpt::serialization::CArchive& out) const;
    /** Binary dump to stream - used in derived classes' serialization */
    void readFromStream(mrpt::serialization::CArchive& in);

    bool               points_visible = false;
    float              point_size     = 1.0f;
    mrpt::img::TColorf points_color{.0f, .0f, 1.0f};
    bool               planes_visible = true;
    mrpt::img::TColorf planes_color{1.0f, .0f, 1.0f};
    bool               normals_visible = true;
    mrpt::img::TColorf normals_color{1.0f, 0.0f, 0.0f};

    /** Colormap to use. Can be set to cmNone for uniform color */
    mrpt::img::TColormap points_colormap = mrpt::img::cmHOT;
    mrpt::img::TColormap planes_colormap = mrpt::img::cmHOT;

    /** If colormap!=mrpt::img::cmNONE, use this channel as color index */
    std::string recolorByPointField = "intensity";
  };
  TRenderOptions renderOptions;

 public:
  // Interface for use within a mrpt::maps::CMultiMetricMap:
  MAP_DEFINITION_START(NDT)
  float voxel_size = 1.00f;

  mola::NDT::TInsertionOptions  insertionOpts;
  mola::NDT::TLikelihoodOptions likelihoodOpts;
  mola::NDT::TRenderOptions     renderOpts;
  MAP_DEFINITION_END(NDT)

 private:
  float voxel_size_ = 1.00f;

  // Calculated from the above, in setVoxelProperties()
  float                 voxel_size_inv_ = 1.0f / voxel_size_;
  float                 voxel_size_sqr_ = voxel_size_ * voxel_size_;
  mrpt::math::TPoint3Df voxelDiagonal_;

  /** Voxel map as a set of fixed-size grids */
  grids_map_t voxels_;

  struct CachedData
  {
    CachedData() = default;

    void reset() { *this = CachedData(); }

    mutable std::optional<mrpt::math::TBoundingBoxf> boundingBox_;

#if defined(HASHED_VOXEL_POINT_CLOUD_WITH_CACHED_ACCESS)
    // 2 bits seems to be the optimum for typical cases:
    constexpr static int CBITS               = 2;
    constexpr static int NUM_CACHED_IDXS     = 1 << CBITS;
    constexpr static int NUM_CACHED_IDX_MASK = NUM_CACHED_IDXS - 1;

    int              lastAccessNextWrite = 0;
    global_index3d_t lastAccessIdx[NUM_CACHED_IDXS];
    VoxelData*       lastAccessVoxel[NUM_CACHED_IDXS] = {nullptr};
#endif
  };

  CachedData cached_;

 protected:
  // See docs in base CMetricMap class:
  void internal_clear() override;

 private:
  // See docs in base CMetricMap class:
  bool internal_insertObservation(
      const mrpt::obs::CObservation&                   obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;
  // See docs in base class
  double internal_computeObservationLikelihood(
      const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const override;

  double internal_computeObservationLikelihoodPointCloud3D(
      const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
      const std::size_t num_pts) const;

  /** - (xs,ys,zs): Sensed point local coordinates in the robot frame.
   *  - pc_in_map: SE(3) pose of the robot in the map frame.
   */
  void internal_insertPointCloud3D(
      const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
      const std::size_t num_pts);

  // See docs in base class
  bool internal_canComputeObservationLikelihood(const mrpt::obs::CObservation& obs) const override;

  /// Used for getAsSimplePointsMap only.
  mutable mrpt::maps::CSimplePointsMap::Ptr cachedPoints_;
};

}  // namespace mola
