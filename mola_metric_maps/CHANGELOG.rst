^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_metric_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
