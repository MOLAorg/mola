^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_metric_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.0 (2026-04-29)
------------------
* Merge pull request `#139 <https://github.com/MOLAorg/mola/issues/139>`_ from MOLAorg/fix/monothonic-kf-ids
  Fix: ensure monothonic KF ids
* Fix: ensure monothonic KF ids
* Merge pull request `#138 <https://github.com/MOLAorg/mola/issues/138>`_ from MOLAorg/fix/kf-map
  fix: robustify edge cases from last API changes
* fix: robustify edge cases from last API changes
* Merge pull request `#137 <https://github.com/MOLAorg/mola/issues/137>`_ from MOLAorg/feat/metric-map-changes-for-lo-grav-align
  mola_metric_maps: per-KF pose plumbing for online gravity rebake
* feat(mola_metric_maps): per-KF pose plumbing for online gravity rebake
  Add the KeyframePointCloudMap APIs needed by mola_lidar_odometry's
  online gravity-rebake feature:
  - cloneKFPoses: snapshot of all KF poses keyed by id
  - setKeyframePose: per-KF pose overwrite with cache invalidation
  - lastInsertedKeyFrameID / nextFreeKeyFrameID_public: id introspection
  - drainEvictedKeyFrameIDs: pull-then-clear list of KFs dropped during
  remove_frames_farther_than-driven evictions inside insertObservation
  Also exports the MOLA_METRIC_MAPS_HAS_KFM_POSE_PLUMBING feature macro
  so downstream packages in separate repos (mola_lidar_odometry) can
  guard usage with __has_include + this macro and stay buildable
  against older mola_metric_maps checkouts.
  Adds unit-test coverage for the new APIs in
  test-mola_metric_maps_keyframemap.
  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
* Merge pull request `#134 <https://github.com/MOLAorg/mola/issues/134>`_ from Zeal-Robotics/perf/keyframe-prewarm-global-submap
  perf(mola_metric_maps): pre-warm global-frame submap data in icp_get_prepared_as_global
* perf(mola_metric_maps): pre-warm global-frame submap data in icp_get_prepared_as_global
  icp_get_prepared_as_global() was building, on the search submap, only the
  local-frame KD-tree and per-point covariances (via KeyFrame::buildCache):
  - bbox
  - kdTreeEnsureIndexBuilt3D() on pointcloud\_     (local frame)
  - computeCovariancesAndDensity()                 (local frame)
  The very first call to nn_search_cov2cov() then lazily materialized the
  global-frame counterparts on the caller thread:
  - pointcloud_global()      (deep copy of pointcloud\_ rotated by pose())
  - kdTreeEnsureIndexBuilt3D() on the global cloud
  - covariancesGlobal()      (rotated covariances)
  For a non-trivial preloaded local map this stalled the first ICP align()
  by several seconds (e.g. ~6 s on a typical localization map), defeating
  the purpose of having a separate "prepare global" hook. Front-ends that
  explicitly pre-warm via icp_get_prepared_as_global() at startup were
  hit hardest, since the remaining lazy work then showed up on the very
  first scan instead of being amortized across startup.
  Pre-trigger the same three calls at the end of
  icp_get_prepared_as_global() so the search submap is fully ready by the
  time ICP::align() / nn_search_cov2cov() runs.
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

2.7.0 (2026-04-22)
------------------
* Reorganize website
* Merge pull request `#121 <https://github.com/MOLAorg/mola/issues/121>`_ from MOLAorg/fix/clean-up-old-mrpt-version-checks
  Clean up: remove old mrpt version fallback code sections
* Contributors: Jose Luis Blanco-Claraco

2.6.1 (2026-04-02)
------------------
* Merge pull request `#119 <https://github.com/MOLAorg/mola/issues/119>`_ from MOLAorg/fix/thread-safety
  Fix/thread safety
* mola_metric_maps: FIX potential race conditions
  They didn't happen in practice but to be safe
  code clean up
  wip
* Fix potential race in KeyframePointCloudMap::icp_get_prepared_as_global()
* Fix MRPT version required for updated insertAnotherMap() API
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

2.6.0 (2026-03-12)
------------------
* Merge pull request `#113 <https://github.com/MOLAorg/mola/issues/113>`_ from MOLAorg/feat/kf-map-view-vectors
  Feat:kf map view vectors filter for NN search
* Honor the documented disable semantics for max_view_angle_deg
* NDT tests: fix clang-tidy warnings
* view-direction NN filter; add KF map unit tests
* Fix: possible invalid deref
* Better criterion to select active frames
* KeyFramePointCloudMap: Add two debug env vars to control visualization
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* BUG FIX: creationOptions were not loaded from ini file in KeyframePointCloudMap
* fix clang-tidy warning: avoid std::endl
* Update coyright notes
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------
* Merge pull request `#100 <https://github.com/MOLAorg/mola/issues/100>`_ from MOLAorg/fix/remove-mrpt-deprecated-maps
  Remove use of mrpt deprecated maps
* Avoid use of deprecated mrpt::maps classes
* Merge pull request `#99 <https://github.com/MOLAorg/mola/issues/99>`_ from MOLAorg/feat/ros2-bridge-pub-geographic
  ROS2 bridge: publish geographic poses too
* ros2 bridge: use geographic_msgs, store the last georeference info internally, and publish georef poses
  merge of these commits:
  - Enable many more clang-tidy checks
  - lint clean
  - implement publishing georeferenced poses
  - mola-viz: fix potential crash on edge case with all points having NaN value
  - FIX: potential crash if no MapServer is present and map services are called
* Contributors: Jose Luis Blanco-Claraco

2.4.0 (2025-12-28)
------------------
* KeyframePointCloudMap: viz now reuses mrpt function for better generalized per-field coloring and avoid code duplication
* Contributors: Jose Luis Blanco-Claraco

2.3.0 (2025-12-15)
------------------
* KF metric map: change heuristic to select nearby KFs for NN matching taking into account the orientation
* KeyFramePointCloudMap: new rendering option to show XYZ axes
* Contributors: Jose Luis Blanco-Claraco

2.2.1 (2025-11-08)
------------------
* Metric maps: implement missing loading 'color.A' from config files
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2025-10-28)
------------------
* format
* Upgrade to use the upcoming MRPT 2.15 API for CGenericsPointsMap
* KeyFrames metric map: new option to visualize (via ROS publish) with a maximum number of points, downsampling for better performance
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2025-10-20)
------------------
* Fix formatting
* Implement getAsSimplePointsMap()
* KeyframePointCloudMap: Fix class must be copy-constructible
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2025-10-13)
------------------
* Merge pull request `#93 <https://github.com/MOLAorg/mola/issues/93>`_ from MOLAorg/feature/better-lio
  Changes for new LIO
* add optional debug viz files; fix race conditions
* cov2cov pairings now saves the sqrt(cov_inv)
* Move to new mp2p_icp cov2cov matcher API
* Update missing copyright notices
* New KeyframePointCloudMap map
* Fix typos and clang-tidy hints
* Fix clang-tidy formatting tips
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------

1.8.1 (2025-05-28)
------------------
* Fix: Do not use the deprecated ament_target_dependencies()
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-05-25)
------------------
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------
* fix clang-format
* Metric maps can now be rendered as semitransparent pointclouds
* Contributors: Jose Luis Blanco-Claraco

1.6.4 (2025-04-23)
------------------
* robin-map: Update to v1.4.0
* modernize clang-format
* Contributors: Jose Luis Blanco-Claraco

1.6.3 (2025-03-15)
------------------

1.6.2 (2025-02-22)
------------------

1.6.1 (2025-02-13)
------------------

1.6.0 (2025-01-21)
------------------

1.5.1 (2024-12-29)
------------------

1.5.0 (2024-12-26)
------------------

1.4.1 (2024-12-20)
------------------

1.4.0 (2024-12-18)
------------------

1.3.0 (2024-12-11)
------------------
* NDT maps: more render options (enable colormaps,etc.)
* mola_metric_maps: robin-maps upgraded to latest version
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------
* gcc warning fix
* Avoid gcc warning
* Merge pull request `#69 <https://github.com/MOLAorg/mola/issues/69>`_ from MOLAorg/new-map-ndt
  New NDT-3D metric map
* Add NDT-3D map class
* Remove leftover dead .cpp file from MOLA package template
* FIX BUG: missing cmake dependency on robin_map in exported targets
* Contributors: Jose Luis Blanco-Claraco

1.1.3 (2024-08-28)
------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

1.1.2 (2024-08-26)
------------------

1.1.1 (2024-08-23)
------------------

1.1.0 (2024-08-18)
------------------
* Update clang-format style; add reformat bash script
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* Contributors: Jose Luis Blanco-Claraco

1.0.8 (2024-07-29)
------------------
* Update robin-map to latest version (Fix cmake < 3.5 compatibility warning)
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------

1.0.6 (2024-06-21)
------------------

1.0.5 (2024-05-28)
------------------

1.0.4 (2024-05-14)
------------------
* Metric maps: load insertion options from field 'insertOpts' instead of 'insertionOptions' for compatibility with all other MRPT maps
* disable clang-format in 3rdparty submodules
* Fix usage of const_cast<> with proper value() method
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Add macro HASHED_VOXEL_POINT_CLOUD_WITH_CACHED_ACCESS
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* implement cached conversion to pointcloud
* make cfg file section optional
* FIX: error on rendering empty voxel maps
* HashedVoxelPointCloud: add missing reserve()
* copyright update
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
