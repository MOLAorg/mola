^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_kernel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2026-08-21)
------------------
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
* Support re-expressing estimator and pose-list state in a new frame
  Adds the two pieces a one-off gauge change of the map frame needs, so that
  leveling the frame once gravity is known does not have to throw state away:
  - NavStateFilter::transform_frame(), an optional virtual returning false by
  default, so existing estimators are unaffected and a caller that gets false
  falls back to reset(). Guarded by a feature macro for downstream detection.
  - SearchablePoseList::transform_left_multiply(), so keyframe-density
  bookkeeping moves with the frame instead of being invalidated by it.
  Both are pure left-multiplications, p -> b + p. Note that relative motion,
  and hence any velocity expressed in the vehicle's own frame, is invariant
  under this and must NOT be rotated.
* Contributors: Jose Luis Blanco-Claraco

3.1.1 (2026-08-10)
------------------
* Fix C++20 build: keep TransformTree/TransformTreeNode aggregates by
  dropping their defaulted default constructors, which made brace
  initialization stop compiling on ROS 2 Lyrical (C++20).
* Contributors: Jose Luis Blanco-Claraco

3.1.0 (2026-08-06)
------------------
* Add mola::TransformTreeSource: expose the /tf tree to other MOLA modules (`#188 <https://github.com/MOLAorg/mola/issues/188>`_)
  * Add mola::TransformTreeSource: expose the /tf tree to other modules
  New mola_kernel interface letting a data source publish its tree of
  coordinate frames, so consumers (e.g. a LiDAR-odometry viewer) can draw a
  robot's joints as it moves. transform_tree(root) returns the subtree below
  'root' with poses already resolved against it.
  The filtering is done in the source, which is what owns the transform
  buffer, so a consumer never receives nor walks unrelated subtrees. The
  interface carries no ROS/tf2 type, keeping mola_kernel ROS-independent.
  Implemented by Rosbag1Dataset (separate repo), Rosbag2Dataset and BridgeROS2.
  No locking is added around the subtree walk: tf2::BufferCore guards its own
  internals, which is also what lets BridgeROS2's TransformListener feed the
  buffer from its own thread.
  * Address review: depth-first comment, cycle guard in the tf walk
  * No need for include guards inside this same git repo
* fix formatting
* mola_kernel: don't flood ERROR logs when GUI preview has no viz module
  A launch YAML can populate gui_preview_sensors for a sensor without
  gating that entry behind MOLA_WITH_GUI (several mola_lidar_odometry
  launch files did exactly this). In that case every observation for that
  sensor label enqueued a task that threw and caught "Could not find a
  running MolaViz module" -- thousands of ERROR-level log lines per
  headless run, found via SLAM quality-eval sweeps producing 20k+ of them
  on a single KITTI sequence.
  Check once, outside the per-observation lambda, whether a VizInterface
  is actually running; if not, log a single throttled warning and skip
  enqueueing entirely instead of relying on every launch file getting its
  own MOLA_WITH_GUI gate right.
* Merge pull request `#187 <https://github.com/MOLAorg/mola/issues/187>`_ from MOLAorg/feat/incremental-point-cloud-kdtree-bake
  Bake IncrementalPointCloud's k-d tree index (mm-ipc-bake-kdtree)
* changelog
* fix: mola_kernel listed mrpt::gui as dependency but could be removed
* Merge pull request `#185 <https://github.com/MOLAorg/mola/issues/185>`_ from MOLAorg/chore/remove-keyframe-map-capable
  Remove the KeyframeMapCapable interface
* chore: remove the KeyframeMapCapable interface
  This mixin was introduced to expose per-KF pose plumbing to
  mola_lidar_odometry's trajectory-rebake experiment, which corrected
  accumulated tilt by re-integrating the keyframe chain. That experiment is
  being removed: it was never wired in, and rotating map keyframes without
  transforming the trajectory consistently leaks vertical position.
  The interface had exactly one implementation and no callers, so it is
  removed along with the two methods that existed only for the rebake path,
  `oldestActiveKeyframeID()` and `applyPivotTransform()`.
  `keyframePoses()` is kept, since the regroup tests already use it as
  ordinary map API, and the duplicate `cloneKFPoses()` (whose only difference
  was not being the virtual one) is folded into it.
* Merge pull request `#183 <https://github.com/MOLAorg/mola/issues/183>`_ from MOLAorg/feat/child-loggers-console-hook
  Let a module expose child loggers for console capture
* feat(kernel): let a module expose child loggers for console capture
  ExecutableBase gains child_loggers(), an optional hook returning
  additional mrpt::system::COutputLogger instances a module owns but does
  not log through itself (e.g. a library engine run on its own background
  thread). mola_viz_imgui's console window now discovers and hooks these
  the same way it already does for the module itself, tagged with a
  "module/child" source name, so their output is captured without the
  owning module having to relay each message through its own logger
  (which would otherwise re-apply the module's own verbosity gating).
  Motivated by mola_mapper's background loop-closure engine
  (mola_sm_loop_closure), which has its own independent logger whose
  output was previously invisible to the imgui console.
* Merge pull request `#180 <https://github.com/MOLAorg/mola/issues/180>`_ from MOLAorg/feat/combobox-fixed-width
  GUI: add optional fixed_width to ComboBox (both backends)
* Add optional fixed_width to ComboBox, honored by both GUI backends
  Without it, ImGui::Combo defaults to an item width that scales with
  the window, so short option lists visibly balloon in wide sub-windows
  (seen when packing a ComboBox next to a checkbox in a Row). 0 keeps
  each backend's current auto-sizing behavior.
* Contributors: Jose Luis Blanco-Claraco

* fix: removed unused mrpt::gui dependency and the unused KeyframeMapCapable
  interface (dead code from an abandoned trajectory-rebake experiment).
* feat(kernel): added child_loggers() hook to ExecutableBase so modules can
  expose background-thread loggers for console capture.
* feat: ComboBox GUI widget gained an optional fixed_width parameter (both
  backends).
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-07-17)
------------------
* fix: default-dock Dataset_UI panel at top; fix Console dock on old layouts
  Add WindowDescription::dock_top_by_default so a window without a saved
  imgui.ini entry gets docked into a strip reserved at the top of the main
  window instead of floating; enabled for the per-module Dataset_UI panel.
  Also make the Console's default bottom-dock (and the new top-dock) key off
  whether that specific window has a saved imgui.ini entry, instead of whether
  the whole ini file already existed. This fixes both never docking new
  window kinds added to an already-existing layout, and splitting a stale,
  no-longer-leaf dock node once more than one default dock has been applied.
* fix: avoid enque on shutdown
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
* NavStateFilter: introduce optional virtual georef setter/getter
* Merge pull request `#165 <https://github.com/MOLAorg/mola/issues/165>`_ from MOLAorg/imgui-window-icon
  feat: mola viz imgui set window icon
* feat: mola viz imgui set window icon
* Merge pull request `#163 <https://github.com/MOLAorg/mola/issues/163>`_ from MOLAorg/fix/race-conditions
  fix(mola_kernel): guard RawDataSourceBase consumer list against concu…
* fix(mola_kernel): guard RawDataSourceBase consumer list against concurrent attach
  mola_launcher initializes modules in parallel threads (executor_thread), so when
  two consumers subscribe to the SAME raw_data_source (e.g. both LidarOdometry and
  a mapper module use `raw_data_source: dataset_input`), both call
  attachToDataConsumer() concurrently and race on the unsynchronized
  std::vector<RawDataConsumer*> push_back. This corrupted the heap and caused an
  intermittent SIGSEGV at startup (caught via coredump: _M_realloc_insert into a
  garbage pointer under RawDataSourceBase::attachToDataConsumer <- FrontEndBase::
  initialize <- MolaLauncherApp::executor_thread).
  Add a mutex guarding rdc\_: lock the push_back, and snapshot the list under the
  lock in sendObservationsToFrontEnds() before dispatching (dispatch itself runs
  without the lock so onNewObservation() is not serialized). Validated: 16/16
  SharedMapOnly startups on MulRan DCC01 with two consumers, 0 crashes / 0 cores.
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
* Merge pull request `#159 <https://github.com/MOLAorg/mola/issues/159>`_ from MOLAorg/feat/central-map
  feat: Add virtual interface for central keyframe map for mapper-3d
* feat: Add virtual interface for central keyframe map for mapper-3d
* Merge pull request `#151 <https://github.com/MOLAorg/mola/issues/151>`_ from MOLAorg/fix/imgui-slider
  fix: imgui slider
* fix: imgui slider
* Contributors: Jose Luis Blanco-Claraco

2.9.0 (2026-05-11)
------------------
* Merge pull request `#143 <https://github.com/MOLAorg/mola/issues/143>`_ from MOLAorg/bump-cmake-version
  bump min req cmake version to 3.22
* bump min req cmake version to 3.22
* Merge pull request `#142 <https://github.com/MOLAorg/mola/issues/142>`_ from MOLAorg/feat/add-keyframe-map-capable-api
  feat: Add keyframe map capable API
* feat: Add keyframe map capable API
* kernel: utils RegexCache made thread-safe
* Contributors: Jose Luis Blanco-Claraco

2.8.0 (2026-04-29)
------------------
* Merge branch 'Zeal-Robotics-fix/map-source-latched-replay' into develop
* chore: transient callbacks done with a copy of last updates
* fix(mola_kernel): replay latched MapUpdates to late subscribers
  `MapSourceBase::subscribeToMapUpdates()` now caches the most recent
  `MapUpdate` per `map_name` whenever it was advertised with
  `keep_last_one_only=true`, and replays it once into any callback
  registered later. This mirrors ROS' `transient_local` durability at
  the MOLA-callback layer.
  Without this, a producer that advertises its initial map before a
  consumer has had a chance to subscribe (e.g. `mola_lidar_odometry`
  publishing the loaded `.mm` on its first scan, before
  `mola_bridge_ros2::doLookForNewMolaSubs()` has run for the first
  time) would silently lose that update. The race is more likely with
  small maps, where odometry initialization is fast enough to beat the
  bridge's first sub-discovery poll. With this change the late
  subscriber receives the cached update on registration, and the ROS
  publisher created by the bridge then carries the map onward via its
  existing `transient_local` QoS.
  Updates flagged with `keep_last_one_only=false` (e.g. deskewed
  scans) are intentionally not cached, matching their fire-and-forget
  semantics.
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

2.7.0 (2026-04-22)
------------------
* Merge pull request `#129 <https://github.com/MOLAorg/mola/issues/129>`_ from MOLAorg/feat/ros2-diagnostics
  Feature: ROS2 diagnostics
* feat(kernel): add DiagnosticsProvider interface for REP-107 diagnostics
  Introduce an opt-in mix-in interface for modules to publish structured,
  severity-tagged diagnostics, to be bridged to ROS 2 /diagnostics by
  mola_bridge_ros2 in a follow-up.
  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
* Merge pull request `#121 <https://github.com/MOLAorg/mola/issues/121>`_ from MOLAorg/fix/clean-up-old-mrpt-version-checks
  Clean up: remove old mrpt version fallback code sections
* Bump minimum required MRPT version to 2.15.0
* Clean up: remove old mrpt version fallback code sections
* Contributors: Jose Luis Blanco-Claraco

2.6.1 (2026-04-02)
------------------
* Merge pull request `#116 <https://github.com/MOLAorg/mola/issues/116>`_ from MOLAorg/update-rds-gui
  Update RDS gui creation to new backend agnostic API
* RDS: Fix parsing initial window pos and width
* Update RDS gui creation to new backend agnostic API
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
* Add support for menu bars
* VizInterface made backend agnostic (ImGUI and Nanogui)
* Merge pull request `#108 <https://github.com/MOLAorg/mola/issues/108>`_ from MOLAorg/feat/use-gps-msgs
  Support gps_msgs data types too for ROS2
* Support gps_msgs as alternative to NavSatFix
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* fix typo in comments
* Update coyright notes
* fix clang-tidy warnings
* Contributors: Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------
* Merge pull request `#104 <https://github.com/MOLAorg/mola/issues/104>`_ from MOLAorg/feat/better-regex
  Explain error on bad regex
* Explain error on bad regex
* Merge pull request `#103 <https://github.com/MOLAorg/mola/issues/103>`_ from MOLAorg/feat/RegexCache
  Add utility RegexCache
* Add utility RegexCache
* Add has_converged_localization() method for state estimation
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

2.3.0 (2025-12-15)
------------------

2.2.1 (2025-11-08)
------------------

2.2.0 (2025-10-28)
------------------

2.1.0 (2025-10-20)
------------------
* Send sensor_rate_decimation to Viz
* interfaces/MapSourceBase: Add keep_last_one_only property
* Make use of ConstPtr across API
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2025-10-13)
------------------
* Merge pull request `#93 <https://github.com/MOLAorg/mola/issues/93>`_ from MOLAorg/feature/better-lio
  Changes for new LIO
* MolaViz: Add method clear_all_point_clouds_with_decay()
* MolaViz: Add support for inserting clouds with decay_time
* Large clean up of unused code from older MOLA versions.
  In particular, all abstract definitions of factors, entities, and WorldModel have been removed.
  It seems more natural and efficient to keep them in the specific SLAM modules.
* Allow extra parameters in mola_viz per-sensor preview windows
* fix clang-format
* Modernize copyright notice
* Remove old code that was needed to support very old MRPT versions
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------
* MapSourceBase: add a new optional field "metadata"
* NavStateFilter interface: Now is a RawDataConsumer too
* Contributors: Jose Luis Blanco-Claraco

1.8.1 (2025-05-28)
------------------

1.8.0 (2025-05-25)
------------------
* Update Viz interface: add methods to run arbitrary Scene manipulation and camera orthographic mode
* Update copyright year
* fix reversed logic
* clang-format fix
* Add mola::Synchronizer for grouping observations
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------
* code clean up: remove useless dtors, and mark the required copy ctors as deleted
* Contributors: Jose Luis Blanco-Claraco

1.6.4 (2025-04-23)
------------------
* fix: Correctly handling Livox cloud timestamps ("double"s, but in nanoseconds) in BridgeROS2 and bag2 data sources. They are automatically detected, no need to change any parameter.
* modernize clang-format
* Contributors: Jose Luis Blanco-Claraco

1.6.3 (2025-03-15)
------------------

1.6.2 (2025-02-22)
------------------
* ExecutableBase inteface: added diagnostics API
* Contributors: Jose Luis Blanco Claraco

1.6.1 (2025-02-13)
------------------
* mola_kernel: Add Georeferencing structure and add it to map updates
* Contributors: Jose Luis Blanco-Claraco

1.6.0 (2025-01-21)
------------------
* Fix published /tf's: those from LocalizationSources now can explicitly define their parent and child frames
* LocalizationSources now can explicitly define both, their reference and child frames for each estimated pose
* docs: add state estimation images
* Contributors: Jose Luis Blanco-Claraco

1.5.1 (2024-12-29)
------------------
* NavStateFilter API: add estimated_trajectory()
* Contributors: Jose Luis Blanco-Claraco

1.5.0 (2024-12-26)
------------------
* NavStateFilter Interface now also inherits from ExecutableBase for convenience
* MinimalModuleContainer ctor should not be explicit
* Add mola::MinimalModuleContainer
* Drop dependency on mrpt-gui in kernel by abstracting MolaViz subwindow layout operations
* Contributors: Jose Luis Blanco-Claraco

1.4.1 (2024-12-20)
------------------

1.4.0 (2024-12-18)
------------------
* MOLA system yaml files: added "enabled" optional property for modules and rds visualizers
* Add field for localization quality
* cmake: remove duplicated info message
* ExecutableBase: Add support for runtime-configurable parameter API
* mola-kernel Doxygen docs: add groups
* Contributors: Jose Luis Blanco-Claraco

1.3.0 (2024-12-11)
------------------
* NavStateFilter interface: add API for merging GNSS observations
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------
* Update RTTI macros for upcoming MRPT 2.14.0
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
* add <mola_kernel/version.h> with a version-checking macro
* Merge pull request `#65 <https://github.com/MOLAorg/mola/issues/65>`_ from MOLAorg/add-more-srvs
  Add more Services
* Avoid cmake file glob expressions
* mola_kernel: add MapServer interface
* mola_kernel: add public symbols MOLA\_{MAJOR,MINOR,PATCH}_VERSION
* Update clang-format style; add reformat bash script
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* Contributors: Jose Luis Blanco-Claraco

1.0.8 (2024-07-29)
------------------
* mola_kernel: add C++ virtual interface for relocalization methods
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------
* Viz interface: add API for rotate camera
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2024-06-21)
------------------
* Create new NavStateFilter interface and separate the simple fuser and the factor-graph approach in two packages
* mola_kernel: renamed factor FactorConstVelKinematics
* Contributors: Jose Luis Blanco-Claraco

1.0.5 (2024-05-28)
------------------
* viz: fix mismatched free/delete inside nanogui layout
* Contributors: Jose Luis Blanco-Claraco

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Avoid global static objects
* remove useless #include's
* Define Dataset_UI dtor/ctor in a separate translation unit
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------
* Remove now-useless build dependencies and includes for mola-kernel
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* add methods to query for subscribers
* New interfaces
* Refactor initialize()
* mola_kernel: new UI interface for datasets
* New option to shutdown automatically mola-cli after dataset ends
* viz API: add enqueue_custom_nanogui_code()
* mola_viz: show console messages
* Correct usage of mola:: namespace in cmake targets
* copyright update
* mola_viz: support visualizing velodyne observations
* Add look_at() viz interface
* Fewer mutex locking()
* dont force by default load() lazy-load observations
* FrontEndBase: attach to VizInterface too
* Fix loss of yaml key/values when using import-from-file feature
* kitti eval cli moves to its own package
* port to mrpt::lockHelper()
* reorganize as monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Correct references to the license.
* viz interface: new service update_3d_object()
* Fix const-correctness of observations
* FIX missing dependency on mrpt::gui for public header
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-02)
------------------

* Add virtual interface for dataset groundtruth
* Update copyright date
* Update to new colcon ROS2 build system
* Contributors: Jose Luis Blanco-Claraco
