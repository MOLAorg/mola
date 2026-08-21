^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2026-08-21)
------------------
* Remove stray COLCON_IGNORE from mola_viz, mola_viz_imgui
  Both were accidentally swept into 2c5735f5 ("declare transform_frame()
  last..."), unrelated to that commit's actual content. With colcon no
  longer building them from source, any package depending on mola_viz
  (e.g. mola_mapper's exec_depend) falls back to the stale public
  ros-<distro>-mola-viz release, which pulls in a matching stale
  ros-<distro>-mola-kernel via apt -- and since /opt/ros/<distro>/include
  is searched before the workspace install prefix, that stale
  mola_kernel silently shadows the freshly-built one, breaking any
  translation unit that needs a recent mola_kernel API
  (ChildLoggerName/child_loggers()).
* Merge pull request `#192 <https://github.com/MOLAorg/mola/issues/192>`_ from MOLAorg/feat/map-frame-gauge-change
  Support re-expressing estimator and pose-list state in a new frame
* Merge remote-tracking branch 'origin/feat/map-frame-gauge-change' into feat/map-frame-gauge-change
* mola_kernel: declare transform_frame() last; test SearchablePoseList frame change
  Appending the new optional virtual only grows the vtable, whereas the
  previous mid-list position shifted the slot index of the three virtuals
  after it, breaking any prebuilt NavStateFilter plugin. A note asks that
  future optional virtuals also go at the end.
  Adds regression coverage for SearchablePoseList::transform_left_multiply()
  in both storage modes: that the kd-tree follows the transformed poses
  (verified to fail if the spatial index is left stale), that the empty and
  identity cases are no-ops, and that the id-keyed lookup survives.
* Merge branch 'develop' into feat/map-frame-gauge-change
* Contributors: Jose Luis Blanco-Claraco

3.1.1 (2026-08-10)
------------------

3.1.0 (2026-08-06)
------------------
* Merge pull request `#187 <https://github.com/MOLAorg/mola/issues/187>`_ from MOLAorg/feat/incremental-point-cloud-kdtree-bake
  Bake IncrementalPointCloud's k-d tree index (mm-ipc-bake-kdtree)
* changelog
* Merge pull request `#180 <https://github.com/MOLAorg/mola/issues/180>`_ from MOLAorg/feat/combobox-fixed-width
  GUI: add optional fixed_width to ComboBox (both backends)
* Add optional fixed_width to ComboBox, honored by both GUI backends
  Without it, ImGui::Combo defaults to an item width that scales with
  the window, so short option lists visibly balloon in wide sub-windows
  (seen when packing a ComboBox next to a checkbox in a Row). 0 keeps
  each backend's current auto-sizing behavior.
* Contributors: Jose Luis Blanco-Claraco

* feat: ComboBox GUI widget gained an optional fixed_width parameter (both
  backends).
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-07-17)
------------------
* Merge branch 'feat/imviz-metric-plots' into develop
* Remove deprecated nanogui-specific VizInterface API
  create_subwindow(), enqueue_custom_nanogui_code(), subwindow_grid_layout()
  and subwindow_move_resize() have no remaining callers now that
  create_subwindow_from_description() and enqueue_custom_gui_code() cover
  their use cases on both backends.
* Merge pull request `#172 <https://github.com/MOLAorg/mola/issues/172>`_ from MOLAorg/feat/imviz-metric-plots
  feat(mola_viz_imgui): add metric time-series plot windows
* feat(mola_viz_imgui): add metric time-series plot windows
  Adds a backend-agnostic register_metric()/push_metric() API to
  VizInterface (guarded by MOLA_KERNEL_VIZ_HAS_METRICS) so any MOLA
  module can stream timestamped scalar values to live, autoscrolling
  plot windows opened from a new "Plots" menu in the mola_viz_imgui
  (ImGui/ImPlot) backend. The nanogui MolaViz backend implements the
  API as a no-op, mirroring the set_menu_bar precedent.
  The Plots menu also gives the Console window its first working
  close/reopen toggle.
  Vendors ImPlot as a submodule under mola_viz_imgui/3rdparty/implot.
* Merge pull request `#165 <https://github.com/MOLAorg/mola/issues/165>`_ from MOLAorg/imgui-window-icon
  feat: mola viz imgui set window icon
* feat: mola viz imgui set window icon
* Merge pull request `#162 <https://github.com/MOLAorg/mola/issues/162>`_ from MOLAorg/feat/viz-decay-lookat-frame-aware
  feat: frame-aware parentFrame for decay clouds and camera look-at
* fix: decay eviction uses per-entry container; look-at fails on missing frame
  Four reviewer findings, all confirmed valid:
  1. VizInterface.h comment: add update_viewport_look_at() to the list of APIs
  covered by MOLA_KERNEL_VIZ_HAS_MOVABLE_FRAMES (it was omitted).
  2/3. Decay cloud eviction (both backends): the eviction path inside
  insert_point_cloud_with_decay() was removing the oldest cloud from the
  *current insertion's* container rather than the container that cloud was
  originally placed in.  If parentFrame ever changes between calls the wrong
  container would be used, leaking the cloud in the scene.  Fix: add a
  `container` member to DecayingCloud and store the owning container at insert
  time; use it at eviction time.
  4/5. Look-at frame not found (both backends): when parentFrame is non-empty but
  the frame node does not yet exist in the scene, the code was silently falling
  back to treating the raw local-frame coordinates as world-space and moving the
  camera to the wrong position.  Fix: return false so the camera is not moved
  until the frame node is present.
* feat: extend VizInterface for frame-aware decay clouds and camera look-at
  insert_point_cloud_with_decay() and update_viewport_look_at() now both
  accept an optional parentFrame argument (same semantics as the existing
  parentFrame on update_3d_object()): when non-empty, the backend looks up
  the named movable frame node and composes its current scene pose with the
  supplied point before inserting the cloud / moving the camera.  This lets
  callers (e.g. mola_lidar_odometry) pass a local-odom-frame point while
  the camera and transient clouds still end up at the correct world-space
  position when a central mapper (mola_mapper_3d) is continuously
  repositioning the frame node.
  Both backends (MolaViz / MolaVizImGuiCore) and the MolaVizImGui
  forwarder are updated.  The MOLA_KERNEL_VIZ_HAS_MOVABLE_FRAMES feature
  macro already covers both new parameters; no additional macro is added.
* Merge pull request `#160 <https://github.com/MOLAorg/mola/issues/160>`_ from MOLAorg/feat/viz-with-movable-frames
  feat: viz with movable frames
* feat: viz with movable frames
* Merge pull request `#151 <https://github.com/MOLAorg/mola/issues/151>`_ from MOLAorg/fix/imgui-slider
  fix: imgui slider
* fix: imgui slider
* Merge pull request `#146 <https://github.com/MOLAorg/mola/issues/146>`_ from MOLAorg/fix-upgrade-points-classes
  fix: upgrade not to use deprecated mrpt2 point cloud classes
* fix: upgrade not to use deprecated mrpt2 point cloud classes
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

2.9.0 (2026-05-11)
------------------
* Merge pull request `#112 <https://github.com/MOLAorg/mola/issues/112>`_ from MOLAorg/feat/dear-imgui-viz
  Introduce Dear ImGui alternative viz module
* chore: misc improvements
* Introduce Dear ImGui alternative viz module
* Merge pull request `#143 <https://github.com/MOLAorg/mola/issues/143>`_ from MOLAorg/bump-cmake-version
  bump min req cmake version to 3.22
* bump min req cmake version to 3.22
* Contributors: Jose Luis Blanco-Claraco

2.8.0 (2026-04-29)
------------------

2.7.0 (2026-04-22)
------------------

2.6.1 (2026-04-02)
------------------
* Merge pull request `#116 <https://github.com/MOLAorg/mola/issues/116>`_ from MOLAorg/update-rds-gui
  Update RDS gui creation to new backend agnostic API
* Add missing nanogui layout
* Contributors: Jose Luis Blanco-Claraco

2.6.0 (2026-03-12)
------------------
* Merge pull request `#113 <https://github.com/MOLAorg/mola/issues/113>`_ from MOLAorg/feat/kf-map-view-vectors
  Feat:kf map view vectors filter for NN search
* comments formatting
* Merge pull request `#109 <https://github.com/MOLAorg/mola/issues/109>`_ from MOLAorg/feat/support-imgui
  Feature: GUI backend agnostic API in mola_kernel (Step towards supporting imgui)
* safer multithread
* Several visual and safety fixes in the new GUI interface
* Use safer livestring registry
* minor fixes
* Add support for menu bars
* MolaViz implements the new backend agnostic API
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* Refactor decay-clouds so they are invariant to wallclock incoming speed
* Update coyright notes
* fix clang-tidy warnings
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------
* Fix: mola_viz lidar preview didn't filter Inf ranges in stats
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
* Fix build against upcoming mrpt 2.15.4
* Prepare for deprecated mrpt::maps classes towards mrpt 3.0.0
* viz pointclouds preview: fix missing uint8_t fields
* Contributors: Jose Luis Blanco-Claraco

2.3.0 (2025-12-15)
------------------
* Fix: missing uint fields stats in PCD preview GUI in mola_viz
* mola-viz: fix NaNs in range of each point channel preview window
* Viz: Fix handling multi-line messages in the UI terminal
* Contributors: Jose Luis Blanco-Claraco

2.2.1 (2025-11-08)
------------------

2.2.0 (2025-10-28)
------------------
* format
* Upgrade to use the upcoming MRPT 2.15 API for CGenericsPointsMap
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2025-10-20)
------------------
* FIX: Show correct sensor rate in Hz when visualizing with a decimation
* Fix clang-tidy warnings
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2025-10-13)
------------------
* Merge pull request `#93 <https://github.com/MOLAorg/mola/issues/93>`_ from MOLAorg/feature/better-lio
  Changes for new LIO
* Fix warnings
* fix build against old mrpt versions
* Implement removal of decayed clouds
* MolaViz: Add method clear_all_point_clouds_with_decay()
* MolaViz: Add support for inserting clouds with decay_time
* fix clang-format
* Allow extra parameters in mola_viz per-sensor preview windows
* MolaViz: show min/max intensity in input sensor point clouds
* Remove old code that was needed to support very old MRPT versions
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------

1.8.1 (2025-05-28)
------------------

1.8.0 (2025-05-25)
------------------
* Implement new virtual Viz methods
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------
* Metric maps can now be rendered as semitransparent pointclouds
* Contributors: Jose Luis Blanco-Claraco

1.6.4 (2025-04-23)
------------------
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
* Drop dependency on mrpt-gui in kernel by abstracting MolaViz subwindow layout operations
* MolaViz: show package name in GUI windows
* Contributors: Jose Luis Blanco-Claraco

1.4.1 (2024-12-20)
------------------

1.4.0 (2024-12-18)
------------------

1.3.0 (2024-12-11)
------------------
* mola_viz: Show IMU data in the GUI too
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------
* mola_viz: do not add a XY ground grid by default to all GUIs
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
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------
* Viz interface: add API for rotate camera
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2024-06-21)
------------------

1.0.5 (2024-05-28)
------------------
* viz: fix mismatched free/delete inside nanogui layout
* Contributors: Jose Luis Blanco-Claraco

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* MolaViz: BUGFIX: shared_ptr were captured by lambdas, delaying proper dtors. Replaced by weak_ptr's
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* ROS2 launch demos
* use new mrpt GPS covariance field
* visualize sensor pose
* mola_kernel: new UI interface for datasets
* mola-viz: show image channel of RGBD observations
* Fix sensorPose on lidar preview
* Viz: show GPS data
* mola_viz: add custom icon
* viz: more options to visualize RGBD camera observations
* viz API: add enqueue_custom_nanogui_code()
* viz console: add fading effect
* mola_viz: show console messages
* Correct usage of mola:: namespace in cmake targets
* copyright update
* mola_viz: support visualizing velodyne observations
* Add look_at() viz interface
* Fewer mutex locking()
* reorganize as monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Initial public release.
* Contributors: Jose Luis Blanco-Claraco


