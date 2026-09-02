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
 * @file   TSDF.h
 * @brief  Truncated signed distance field (TSDF) 3D map representation
 * @author Jose Luis Blanco Claraco
 * @date   Sep 02, 2026
 */
#pragma once

#include <mola_metric_maps/OptionsCapable.h>
#include <mola_metric_maps/index3d_t.h>
#include <mp2p_icp/NearestPlaneCapable.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>
#include <tsl/robin_map.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>

namespace mola
{
/** Truncated signed distance field (TSDF) map, stored as a voxel hash map.
 *
 * Each measured point contributes a signed distance sample to every voxel
 * inside a narrow cylinder around its measurement ray, within
 * `truncation_distance` of the surface. Samples are combined by a weighted
 * running average, so the surface a query sees is the *fused* estimate over
 * every measurement of it, rather than any one stored point.
 *
 * Three consequences make this a distinct map class rather than another point
 * container, and they are the reason it exists:
 *
 * - There is no representative point per cell, so the "which point stands for
 *   this cell" selection disappears entirely.
 * - There is no correspondence search: the residual of a query point is the
 *   interpolated field value at it, and the surface normal is the field
 *   gradient. What the ICP engine consumes is still a point-to-plane pairing,
 *   so no new matcher or solver is required.
 * - The field is continuous (trilinear) inside the mapped region, so the
 *   residual has no jumps as the state moves.
 *
 * The map exposes `mp2p_icp::NearestPlaneCapable` and is therefore usable by
 * `mp2p_icp::Matcher_Point2Plane` with no changes.
 *
 * @note For a per-voxel Gaussian instead of a distance field, see `mola::NDT`.
 */
class TSDF : public mrpt::maps::CMetricMap,
             public mp2p_icp::NearestPlaneCapable,
             public mola::OptionsCapable
{
  DEFINE_SERIALIZABLE(TSDF, mola)
 public:
  TSDF(const TSDF&)            = default;
  TSDF& operator=(const TSDF&) = default;
  TSDF(TSDF&&)                 = default;
  TSDF& operator=(TSDF&&)      = default;

  using global_index3d_t = index3d_t<int32_t>;

  /** @name Basic API for construction and main parameters
   *  @{ */

  /**
   * @brief Constructor
   * @param voxel_size Voxel size [meters]
   */
  TSDF(float voxel_size = 0.30f);

  ~TSDF() override;

  /** Resets the voxel size, and *clears* all current map contents. */
  void setVoxelProperties(float voxel_size);

  float voxelSize() const { return voxel_size_; }

  /** @} */

  /** @name Indices and coordinates
   *  @{ */

  /// Global index of the voxel containing a point.
  inline global_index3d_t coordToGlobalIdx(const mrpt::math::TPoint3Df& pt) const
  {
    return global_index3d_t(
        static_cast<int32_t>(std::floor(pt.x * voxel_size_inv_)),  //
        static_cast<int32_t>(std::floor(pt.y * voxel_size_inv_)),  //
        static_cast<int32_t>(std::floor(pt.z * voxel_size_inv_)));
  }

  /// Coordinates of the *center* of a voxel. The field is sampled at centers,
  /// so this, and not the corner, is what the interpolation is built on.
  inline mrpt::math::TPoint3Df globalIdxToCenter(const global_index3d_t idx) const
  {
    return {
        (static_cast<float>(idx.cx) + 0.5f) * voxel_size_,  //
        (static_cast<float>(idx.cy) + 0.5f) * voxel_size_,  //
        (static_cast<float>(idx.cz) + 0.5f) * voxel_size_};
  }

  /** @} */

  /** @name Data structures
   *  @{ */

  struct VoxelData
  {
    VoxelData() = default;

    /// Truncated signed distance to the surface, in meters. Positive on the
    /// free-space side, i.e. towards the sensor that measured it.
    float dist = 0;

    /// Accumulated weight of the samples averaged into `dist`.
    float weight = 0;
  };

  /** Spatial hash for the field's voxels.
   *
   *  `mola::index3d_hash` truncates to 20 bits, which caps it at 2^20 distinct
   *  values. That is ample for a point-based voxel map, which holds one cell
   *  per occupied voxel at a decimetre-to-metre cell size, and no map class in
   *  this library had ever come near it. A field map is the first that does: it
   *  fills a band around *every* surface at a fine voxel, so it passes a
   *  million voxels within seconds of driving, and past that point every new
   *  key collides with an existing hash and the container degrades from a
   *  measured 51 bytes per voxel to effectively unbounded -- 112 GB on a single
   *  KITTI sequence.
   *
   *  So this class uses the full 64-bit mix. The shared functor is deliberately
   *  left alone: changing it would alter the iteration order of every existing
   *  voxel map, and with it the summation order of anything that accumulates
   *  over one.
   */
  struct voxel_hash_t
  {
    std::size_t operator()(const global_index3d_t& k) const noexcept
    {
      return (static_cast<uint64_t>(static_cast<uint32_t>(k.cx)) * 73856093ULL) ^
             (static_cast<uint64_t>(static_cast<uint32_t>(k.cy)) * 19349663ULL) ^
             (static_cast<uint64_t>(static_cast<uint32_t>(k.cz)) * 83492791ULL);
    }
  };

  using grids_map_t = tsl::robin_map<global_index3d_t, VoxelData, voxel_hash_t>;

  /** @} */

  /** @name Data access API
   *  @{ */

  /// Returns the voxel by global index, or nullptr if it does not exist.
  inline const VoxelData* voxelByGlobalIdxs(const global_index3d_t& idx) const
  {
    auto it = voxels_.find(idx);
    return it == voxels_.end() ? nullptr : &it.value();
  }

  /** Fuses one measured point into the field.
   *  @param pt        Measured point, in map coordinates.
   *  @param sensorPt  Origin of the measurement ray, in map coordinates.
   */
  void insertPoint(const mrpt::math::TPoint3Df& pt, const mrpt::math::TPoint3Df& sensorPt);

  /** Result of a field query. */
  struct FieldQuery
  {
    bool                   valid = false;
    float                  dist  = 0;  //!< Interpolated signed distance [m]
    mrpt::math::TVector3Df gradient{};  //!< Interpolated gradient, unnormalized
  };

  /** Trilinear interpolation of the field, and its analytical gradient, at an
   *  arbitrary point. The query is `valid` only if all eight surrounding
   *  voxels exist and carry at least `min_weight_for_query` of evidence: a
   *  partially observed neighborhood would otherwise extrapolate a surface
   *  from one measurement.
   */
  FieldQuery queryField(const mrpt::math::TPoint3Df& p) const;

  const grids_map_t& voxels() const { return voxels_; }

  size_t size() const { return voxels_.size(); }

  mrpt::math::TBoundingBoxf boundingBox() const override;

  void visitAllVoxels(
      const std::function<void(const global_index3d_t&, const VoxelData&)>& f) const;

  /** @} */

  /** @name API of the NearestPlaneCapable virtual interface
   *  @{ */

  /** Returns the local tangent plane of the zero level set, as seen from the
   *  query point: the plane through the point's own projection onto the
   *  surface, with the field gradient as normal. There is no candidate set and
   *  no selection, so `nn_visit_pt2pl_candidates()` reports exactly this one
   *  pairing.
   */
  NearestPlaneResult nn_search_pt2pl(
      const mrpt::math::TPoint3Df& point, const float max_search_distance) const override;

  /** @} */

  /** @name Public virtual methods implementation for CMetricMap
   *  @{ */

  std::string asString() const override;

  void getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const override;

  bool isEmpty() const override;

  void saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const override;

  /// Zero-crossing points of the field, for visualization and for the
  /// MOLA->ROS2 bridge only. Not efficient.
  const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() const override;

  /** @} */

  /** Options that configure the map's internal structure. Changing
   *  `voxel_size` after the map holds data discards its contents.
   */
  struct TCreationOptions : public mrpt::config::CLoadableOptions
  {
    TCreationOptions() = default;

    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source, const std::string& section) override;
    void dumpToTextStream(std::ostream& out) const override;

    float voxel_size = 0.30f;
  };
  TCreationOptions creationOptions;

  /** Options for insertObservation() */
  struct TInsertionOptions : public mrpt::config::CLoadableOptions
  {
    TInsertionOptions() = default;

    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source, const std::string& section) override;
    void dumpToTextStream(std::ostream& out) const override;

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);

    /** Half-width of the band updated around each surface, in meters. If <=0,
     *  it defaults to `truncation_voxels` times the voxel size.
     *  This is also the maximum distance at which a query can find a surface,
     *  so it must comfortably exceed the expected registration prior error.
     */
    double truncation_distance = .0;

    /** Used only when `truncation_distance` is <= 0. */
    double truncation_voxels = 4.0;

    /** Radius of the cylinder around each measurement ray that receives
     *  samples, in units of the voxel size. This is the **coverage** knob: it
     *  has to reach at least half the spacing between adjacent beams at working
     *  range, or the field is undefined between them and most scan points find
     *  no surface to pair against.
     */
    double ray_tube_voxels = 1.5;

    /** Standard deviation of the Gaussian profile applied across the tube, in
     *  units of the voxel size, or <=0 for a flat profile. This is the **bias**
     *  knob, and it is separate from the coverage one on purpose.
     *
     *  A sample is stored as its distance *along its own ray*, which for an
     *  off-axis voxel over-estimates the distance to the surface, so averaging
     *  samples over a wide tube pushes the zero level set away from the sensor.
     *  Making the profile narrow keeps the average dominated by near-axis rays
     *  while the cutoff above still fills the gaps between beams.
     */
    double tube_sigma_voxels = 0.4;

    /** Weight cap of a voxel. It bounds how slowly the field can follow a
     *  change in the scene, and it is what makes the average a running one.
     */
    double max_weight = 64.0;

    /** Minimum accumulated weight for a voxel to take part in a query. */
    double min_weight_for_query = 1.0;

    /** If !=0, remove voxels farther (Chebyshev distance) than this, in
     *  meters, from the current sensor pose.
     *
     *  A field map must not be given a point map's extent. It allocates a band
     *  of voxels around every surface rather than one cell per measured point,
     *  so its footprint grows with the retained volume and with the inverse
     *  cube of the voxel size, not with the point count.
     */
    double remove_voxels_farther_than = .0;

    /** If !=0, a hard ceiling on the number of voxels. Once exceeded, the
     *  effective extent is shrunk until the map fits, so the footprint is
     *  bounded whatever the voxel size, the scene density or the trajectory.
     *
     *  This exists because `remove_voxels_farther_than` alone does not bound
     *  anything usefully: at a fixed extent the voxel count still scales as the
     *  inverse cube of the voxel size and with how much surface the scene puts
     *  inside that extent, and a run that diverges sweeps an arbitrary volume.
     *  Measured on kitti-05 at a 0.3 m voxel, a 155 m extent peaked at 110 GB.
     *
     *  The eviction is by distance, not by age, so it stays a pure function of
     *  the map contents and the sensor pose and does not depend on insertion
     *  order.
     */
    uint64_t max_voxels = 0;

    /** If true, each sample is weighted by `(ref_range/range)^2`, the usual
     *  range-dependent confidence of a range measurement. Default off, so the
     *  map class is one axis on its own.
     */
    bool weight_by_range = false;

    /** Reference range for `weight_by_range`, in meters. */
    double weight_range_ref = 10.0;

    /** If true, each sample is weighted by the cosine of the incidence angle
     *  between the ray and the local surface normal, approximated by the field
     *  gradient already present. Default off, for the same reason.
     */
    bool weight_by_incidence = false;
  };
  TInsertionOptions insertionOptions;

  /** Rendering options, used in getVisualizationInto() */
  struct TRenderOptions : public mrpt::config::CLoadableOptions
  {
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source, const std::string& section) override;
    void dumpToTextStream(std::ostream& out) const override;

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);

    float              point_size = 2.0f;
    mrpt::img::TColorf points_color{.0f, .8f, .0f};
  };
  TRenderOptions renderOptions;

  // mola::OptionsCapable interface:
  [[nodiscard]] std::map<std::string, mrpt::config::CLoadableOptions*> optionsByName() override;
  bool                                                                 trySetCreationOptions(
                                                                      const mrpt::config::CConfigFileBase& cfg, const std::string& section) override;

 public:
  // Interface for use within a mrpt::maps::CMultiMetricMap:
  MAP_DEFINITION_START(TSDF)
  float voxel_size = 0.30f;

  mola::TSDF::TInsertionOptions insertionOpts;
  mola::TSDF::TRenderOptions    renderOpts;
  MAP_DEFINITION_END(TSDF)

 private:
  float voxel_size_ = 0.30f;

  // Calculated from the above, in setVoxelProperties():
  float voxel_size_inv_ = 1.0f / voxel_size_;

  /// Effective truncation distance, resolved from insertionOptions.
  float truncation() const
  {
    return insertionOptions.truncation_distance > 0
               ? static_cast<float>(insertionOptions.truncation_distance)
               : static_cast<float>(insertionOptions.truncation_voxels) * voxel_size_;
  }

  grids_map_t voxels_;

  mutable std::optional<mrpt::math::TBoundingBoxf> cachedBoundingBox_;
  mutable mrpt::maps::CSimplePointsMap::Ptr        cachedPoints_;

  void invalidateCaches();

 protected:
  // See docs in base CMetricMap class:
  void internal_clear() override;

 private:
  // See docs in base CMetricMap class:
  bool internal_insertObservation(
      const mrpt::obs::CObservation&                   obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;

  double internal_computeObservationLikelihood(
      const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const override;

  bool internal_canComputeObservationLikelihood(const mrpt::obs::CObservation& obs) const override;

  /** Removes voxels far from the given sensor position. */
  void pruneFarVoxels(const mrpt::math::TPoint3Df& sensorPt);

  /** - (xs,ys,zs): sensed point coordinates in the sensor frame.
   *  - pc_in_map: SE(3) pose of the sensor in the map frame.
   */
  void internal_insertPointCloud3D(
      const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
      const std::size_t num_pts);
};

}  // namespace mola
