^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_bridge_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


3.2.0 (2026-08-21)
------------------
* Merge pull request `#198 <https://github.com/MOLAorg/mola/issues/198>`_ from Zeal-Robotics/fix/bridge_ros2-tf-buffer-staleness
  fix(bridge_ros2): keep the TF buffer current under bridge executor load
* fix(bridge_ros2): give the TF listener its own spin thread
  The listener was constructed with spin_thread=false, against tf2's own default,
  on the grounds that the bridge already spins rosNode\_ and a second spinner would
  be redundant. That holds only while the bridge executor keeps up with /tf, and
  makes buffer freshness a function of everything else the node is handling --
  sensor callbacks, timers, map publishing. When it does not keep up, the failure
  is silent: lookups still succeed, they just answer from a buffer that is behind,
  and the resulting poses look plausible.
  With spin_thread=true the /tf and /tf_static subscriptions move to a callback
  group and executor the listener owns, so ingestion is independent of the bridge
  executor's load.
  No new sharing is introduced. The buffer is already read from other threads --
  publishLocalizationTf() from the localization source's thread, transform_tree()
  from whichever module calls it -- and tf2::BufferCore serialises readers and
  writers on a single mutex. The listener's group is created with
  automatically_add_to_executor_with_node=false, so add_node() does not adopt it
  and no second executor can dispatch those subscriptions. Its destructor cancels
  and joins its thread, and it is declared after tf_buffer\_ and rosNode\_, so that
  happens while both are still alive.
* fix(bridge_ros2): drain the ROS queues instead of one message per topic
  The ROS thread polls `spin_some()` on a 10 ms sleep. That loop replaced a
  blocking `rclcpp::spin()` so the destructor could stop the thread via
  `shouldExit\_`, and the sleep was only ever there to keep the flag poll from
  busy-waiting. But `spin_some()` collects the ready set once and executes each
  ready entity at most once per call, so the pair caps every subscription at
  roughly one message per 10 ms and puts a 10 ms floor under handling anything.
  Any topic arriving faster than ~100 Hz therefore keeps its subscription queue
  permanently full, and every message the node sees is stale by the whole queue
  depth. `/tf` is the worst case: it aggregates every broadcaster on the graph,
  so on a robot publishing a few hundred transforms/s the listener's
  KeepLast(100) queue leaves the TF buffer several hundred ms behind. The visible
  symptom is REP-105 composition falling back to the stale odom transform with
  "extrapolation into the future" while the odom broadcaster is publishing on
  time and a normally-spun listener on the same graph reads it as current. A
  200 Hz IMU on the same node loses half its samples the same way.
  `spin_once(timeout)` blocks until there is work while still bounding how long
  `shouldExit\_` goes unchecked, and `spin_all()` then drains whatever else is
  ready. Both are needed: `spin_some()` and `spin_all()` each call
  `wait_for_work(0ms)`, so `spin_all()` alone returns immediately when idle and
  would busy-wait. With work pending `spin_once()` returns at once, so no
  iteration sleeps while a queue is non-empty.
  Cost is that the two bounds stack: worst case for observing `shouldExit\_` goes
  from 10 ms to 20 ms.
  Measured with two TF listeners on the same `/tf`, one per spin strategy:
  draining fully leaves the buffer 12.6 ms behind the broadcaster, taking one
  message per 10 ms tick leaves it 358 ms behind.
  A blocking `spin()` cancelled via `executor.cancel()` would remove the poll
  loop altogether and is the better end state, but `cancel()` racing the
  `spinning.exchange(true)` in `spin()` means the destructor has to retry until
  the thread joins. That is left as a separate change.
* Merge remote-tracking branch 'origin/feat/map-frame-gauge-change' into feat/map-frame-gauge-change
* Merge branch 'develop' into feat/map-frame-gauge-change
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

3.1.1 (2026-08-10)
------------------
* fix build in ros rolling (newer gcc)
* fix format
* mola_bridge_ros2: use tf2_ros .hpp headers instead of deprecated .h
  Avoids #warning deprecation spam on newer ROS2 distros while
  building unchanged from Humble to Rolling.
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

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
* Merge pull request `#187 <https://github.com/MOLAorg/mola/issues/187>`_ from MOLAorg/feat/incremental-point-cloud-kdtree-bake
  Bake IncrementalPointCloud's k-d tree index (mm-ipc-bake-kdtree)
* changelog
* fix: ros rolling changed arguments of tf2_ros tf listeners
* fix: GCC error in Rolling due to missing direct rclcpp/Node.hpp include
* totally drop mrpt2 point cloud classes
* Merge pull request `#177 <https://github.com/MOLAorg/mola/issues/177>`_ from MOLAorg/feat/rep105-loud-stale-odom-warning
  Make the REP-105 stale-odom fallback loud on first occurrence
* fix(bridge): make the REP-105 stale-odom fallback loud on first occurrence
  When publish_localization_following_rep105 is on and the exact sensor-stamp
  odom->base_link transform is unavailable, the bridge composes map->odom against
  the latest (stale) odom transform. That biases the published TF by the robot's
  motion over the stamp gap (motion-correlated jitter, worst mid-turn), and it is
  the normal path whenever the localizer publishes an estimate extrapolated to
  "now" while odom only exists in the past. The warning was throttled to 60 s,
  which hid this persistent timing violation.
  Emit a loud, explanatory warning the first time the fallback fires, naming the
  consequence and the recommended fix (have the state estimator publish map->odom
  directly via StateEstimationSmoother's publish_map_to_odom_tf, and route the
  bridge TF source filter to that /map_odom method), then continue with the
  throttled steady-state warning so logs are not flooded. No behavior change to
  the published transform itself.
* Contributors: Jose Luis Blanco-Claraco

* fix: ros rolling changed arguments of tf2_ros tf listeners; fixed GCC error in
  Rolling due to missing direct rclcpp/Node.hpp include; dropped MRPT2 point
  cloud classes.
* fix(bridge): REP-105 stale-odom TF fallback now logs a loud, explanatory
  warning the first time it fires (with the recommended fix), instead of only
  a throttled one.
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-07-17)
------------------
* chore: less verbose warning
* fix: rep105 mode made robust against lagging odom frame
* fix: replace rclcpp deprecated spin_some()
* Merge pull request `#153 <https://github.com/MOLAorg/mola/issues/153>`_ from Zeal-Robotics/fix/bridge_ros2-relocalize-frame
  fix(bridge_ros2): transform relocalization pose into reference frame
* fix(bridge_ros2): transform relocalization pose into reference frame
  The relocalization topic callback and the relocalize_near_pose service fed
  the incoming PoseWithCovarianceStamped straight to relocalize_near_pose_pdf(),
  ignoring its header.frame_id. The pose was therefore interpreted in the
  localization reference_frame regardless of the frame it was actually
  expressed in. When a tool publishes it in a different frame (e.g. RViz's
  "2D Pose Estimate" in the fixed frame) that differs from reference_frame,
  the relocalization lands in the wrong place.
  Compose reference_frame <- header.frame_id via the tf buffer before
  relocalizing, in both the topic callback and the service. The covariance is
  propagated through the rotation by CPose3DPDFGaussian::changeCoordinatesReference().
  Requests are skipped (topic) or rejected (service) when the transform is
  unavailable, instead of silently relocalizing to a wrong pose.
* fix: safer owned_rclcpp flag
* Merge pull request `#150 <https://github.com/MOLAorg/mola/issues/150>`_ from MOLAorg/fix/bridge-ros2-rclcpp-shutdown-ownership
  feat: bridge ros2 now autodetects rclpp shutdown ownership
* feat: bridge ros2 now autodetects rclpp shutdown ownership
* Merge pull request `#149 <https://github.com/MOLAorg/mola/issues/149>`_ from MOLAorg/fix/tf-listener-share-ros-node
  fix(mola_bridge_ros2): share rosNode\_ with TF listener for consistent clock/use_sim_time
* fix(mola_bridge_ros2): share rosNode\_ with TF listener for consistent clock/use_sim_time handling
  Pass rosNode\_ to TransformListener so its /tf and /tf_static subscriptions
  share the bridge's DDS participant, clock, and use_sim_time setting.
  Without this, the default constructor creates an anonymous node that
  ignores use_sim_time -- causing subtle inconsistencies when playing bags
  with --clock. spin_thread=false avoids a redundant second spinner since
  rosNode\_ is already spun by the bridge's main loop.
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

2.9.0 (2026-05-11)
------------------
* Merge pull request `#143 <https://github.com/MOLAorg/mola/issues/143>`_ from MOLAorg/bump-cmake-version
  bump min req cmake version to 3.22
* bump min req cmake version to 3.22
* Less verbose output: don't print covariance matrix for each geo-referenced solution
* Contributors: Jose Luis Blanco-Claraco

2.8.0 (2026-04-29)
------------------
* Merge pull request `#140 <https://github.com/MOLAorg/mola/issues/140>`_ from MOLAorg/feat/bridge-ros2-qos
  feat: BridgeROS2 now have configurable QoS
* fix: harden parameter parsing
* feat: BridgeROS2 now have configurable QoS
* Merge pull request `#135 <https://github.com/MOLAorg/mola/issues/135>`_ from MOLAorg/pr-132
  feat(bridge_ros2): align REP-105 TF with Nav2 conventions without regressing direct-publish mode
* feat(bridge_ros2): align REP-105 TF publishing with Nav2 conventions
  This brings the localization /tf publisher in line with the convention
  used by AMCL, slam_toolbox, RTAB-Map and Cartographer, addressing three
  issues observed when MOLA is consumed by Nav2-style downstream nodes:
  1. REP-105 composition bug (fix). The inner factor of
  map -> odom = (map -> base)(t_scan) * (base -> odom)(t)
  was sampled at "latest" via waitForTransform(), which only supports
  tf2::TimePoint{}. The two factors must be sampled at the same
  instant or the published correction is biased by the odom-frame
  motion accumulated during the localizer's processing latency. Switch
  to tf_buffer\_->lookupTransform(..., scan_tp). On lookup failure the
  publish is skipped (an empty default-constructed TransformStamped
  would inject TF_NO_FRAME_ID into every consumer's tf2 buffer).
  2. transform_tolerance (new param, default 0.1s). The published stamp
  is now node->now() + transform_tolerance so consumers can do
  lookupTransform(map, base_link, now()) without
  ExtrapolationException. Mirrors AMCL's transform_tolerance and
  RTAB-Map's tf_tolerance. Uses the ROS clock so use_sim_time is
  honored automatically.
  3. transform_publish_period (new param, default 0.05s = 20 Hz). A
  wall timer re-broadcasts the cached map -> odom independent of
  localization update rate, keeping the TF buffer continuously fresh
  even when the localizer runs slower than the consumer query rate.
  Set to 0 to disable. Mirrors slam_toolbox's transform_publish_period
  and RTAB-Map's tf_delay.
  Backwards compat: to exactly preserve the prior (post-bugfix) stamping
  behavior set transform_tolerance: 0 and transform_publish_period: 0.
  publish_in_sim_time retains its meaning for all other publishers
  (sensor TFs, odom messages, etc.); only the localization /tf stamp
  source changed.
* Merge branch 'develop' into perf/keyframe-prewarm-global-submap
* Merge pull request `#133 <https://github.com/MOLAorg/mola/issues/133>`_ from Zeal-Robotics/fix/bridge-ros2-throttle-tf-lookup-errors
  fix(bridge_ros2): throttle TF lookup error logging
* fix(bridge_ros2): throttle TF lookup error logging
  Failed sensor TF lookups (e.g. missing static URDF transforms during
  bring-up) flooded the console with two unthrottled ERROR lines per
  observation. With a Livox IMU at ~100 Hz this produced ~200 lines/s
  per affected topic, drowning out other diagnostics.
  - Throttle (5 s) every per-observation "Could not forward ROS2
  observation to MOLA due to timeout..." in the PointCloud2,
  LaserScan, Imu, NavSatFix and gps_msgs paths, plus the
  "Could not get /tf" path that uses lookupSensorPose, and the
  REP-105 odom->base_link recompute warning in publishLocalizationTf.
  - Drop the redundant printErrors plumbing from waitForTransform: the
  inner MRPT_LOG_ERROR(ex.what()) duplicated what callers already
  print with more context (frames + timestamp). The raw tf2 reason
  is still emitted at DEBUG for deep diagnostics.
  Per-callsite throttle state means independent sensors still surface
  "this stream stopped" promptly; we just stop spamming the same one.
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

2.7.0 (2026-04-22)
------------------
* Merge pull request `#131 <https://github.com/MOLAorg/mola/issues/131>`_ from MOLAorg/feat/actions-custom-runner
  CI actions: build for arm64 too
* Merge branch 'Zeal-Robotics-fix/bridge-ros2-skip-empty-tf-on-rep105-failure' into develop
* Fix formatting and clarify function arguments
* fix(bridge_ros2): skip TF publish when REP-105 odom lookup fails
  In `publishLocalizationTf`, when `publish_localization_following_rep105`
  is enabled, the bridge needs to look up `odom_frame -> base_link_frame`
  to compose `map -> odom`. If that lookup fails (e.g. wheel odometry
  hasn't started publishing yet during system startup), the previous code
  logged the error but still fell through to `tf_bc\_->sendTransform(tf)`
  with a default-constructed `TransformStamped` (empty `frame_id` and
  `child_frame_id`).
  This poisoned every subscriber's tf2 buffer at the localization rate,
  producing a continuous stream of `TF_NO_FRAME_ID`,
  `TF_NO_CHILD_FRAME_ID`, and `TF_SELF_TRANSFORM` errors across every
  node in the system until the odom TF became available.
  Fix: return early on lookup failure so no broadcast happens. While
  here, restructure the function with an early-return guard for
  `publish_tf_from_slam` to flatten the nesting, and include the actual
  frame names in the error message.
  Behavior unchanged in the success path.
* Merge pull request `#129 <https://github.com/MOLAorg/mola/issues/129>`_ from MOLAorg/feat/ros2-diagnostics
  Feature: ROS2 diagnostics
* feat(bridge_ros2): publish REP-107 /diagnostics DiagnosticArray
  Collect structured diagnostics from modules implementing the new
  mola::DiagnosticsProvider interface and publish them on the standard
  ROS 2 /diagnostics topic alongside the existing ad-hoc mola_diagnostics/
  topics. Adds the diagnostic_msgs dependency.
  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
* Merge pull request `#122 <https://github.com/MOLAorg/mola/issues/122>`_ from MOLAorg/feat/ros2-bridge-multiple-odometries
  Add param 'odometry_as_robot_pose_observation' to switch OdometryMsg …
* Skip entries with empty topic name
* Add param 'odometry_as_robot_pose_observation' to switch OdometryMsg mapping to MRPT types
* Merge pull request `#121 <https://github.com/MOLAorg/mola/issues/121>`_ from MOLAorg/fix/clean-up-old-mrpt-version-checks
  Clean up: remove old mrpt version fallback code sections
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

2.6.1 (2026-04-02)
------------------

2.6.0 (2026-03-12)
------------------
* Remove now obsolete version check macro
* Merge pull request `#108 <https://github.com/MOLAorg/mola/issues/108>`_ from MOLAorg/feat/use-gps-msgs
  Support gps_msgs data types too for ROS2
* Support gps_msgs as alternative to NavSatFix
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* Update coyright notes
* Contributors: Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------
* Merge pull request `#105 <https://github.com/MOLAorg/mola/issues/105>`_ from MOLAorg/feat/refactor-ros2-bridges
  Refactor to use external rosbag2 conversion in mrpt_ros_bridge
* Use refactored ros2mrpt bridge in live node too
* Fix: potential access to the ROS2 node before it's ready
* Merge pull request `#101 <https://github.com/MOLAorg/mola/issues/101>`_ from MOLAorg/fix/mola-bridge-no-pub-queue
  Fix: mola bridge no pub queue
* BridgeRos2: publish localization updates immediately, do not buffer them
* Add debug traces
* Merge pull request `#99 <https://github.com/MOLAorg/mola/issues/99>`_ from MOLAorg/feat/ros2-bridge-pub-geographic
  ROS2 bridge: publish geographic poses too
* ros2 bridge: use geographic_msgs, store the last georeference info internally, and publish georef poses
  merge of these commits:
  - Enable many more clang-tidy checks
  - lint clean
  - implement publishing georeferenced poses
  - mola-viz: fix potential crash on edge case with all points having NaN value
  - FIX: potential crash if no MapServer is present and map services are called
* fix clang-format
* FIX: potential deadlock in BridgeROS2 dtor due to early errors before ros2 is initialized
* Contributors: Jose Luis Blanco-Claraco

2.4.0 (2025-12-28)
------------------
* Prepare for not using deprecated mrpt_maps types starting for mrpt >=3.0.0
* FIX: Potential segfault if observations come before ROS2 /tf broadcasters are initialized
* Contributors: Jose Luis Blanco-Claraco

2.3.0 (2025-12-15)
------------------
* Import all pcd fields from ros2 messages using CGenericPointsMap
* Contributors: Jose Luis Blanco-Claraco

2.2.1 (2025-11-08)
------------------
* BridgeROS2: more debug traces in map publishing
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2025-10-28)
------------------
* format
* Fix build against upcoming mrpt v2.15.0
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2025-10-20)
------------------
* Publish to ROS all map types implementing getAsSimplePointsMap()
* format
* FIX: ROS2 bridge must use timestamps for map updates to publish maps and deskewed clouds
* Support publishing several maps from the same source per iteration to ROS
* clang-format
* Make use of ConstPtr across API
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2025-10-13)
------------------
* fix clang-format
* Modernize copyright notice
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------
* fix clang-format
* Implement publishing of optional "metadata" map field too
* Contributors: Jose Luis Blanco-Claraco

1.8.1 (2025-05-28)
------------------
* Fix: Do not use the deprecated ament_target_dependencies()
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-05-25)
------------------
* Update license tag to "BSD-3-Clause"
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------

1.6.4 (2025-04-23)
------------------
* fix: Correctly handling Livox cloud timestamps ("double"s, but in nanoseconds) in BridgeROS2 and bag2 data sources. They are automatically detected, no need to change any parameter.
* modernize clang-format
* Merge pull request `#82 <https://github.com/MOLAorg/mola/issues/82>`_ from ahpinder/develop
  Add Support for Voxel Map ROS2 Publishing Via Point Map Conversion
* fixed Clang formatting
* Clean up voxel map publishing code
* Added voxel map point cloud publishing
  Added code to timerPubMap to publish the occupied voxels of a mrpt::maps::CVoxelMap as a point cloud to ROS2, allowing for real-time ROS2 visualization of 2D map capture
* Contributors: Jose Luis Blanco-Claraco, ahpinder

1.6.3 (2025-03-15)
------------------
* clang-tidy: const correctness
* Service renamed: RelocalizeFromGNSS -> RelocalizeFromStateEstimator
* FIX: Potential deadlock in initialization
* Contributors: Jose Luis Blanco-Claraco

1.6.2 (2025-02-22)
------------------
* Implement publish Diagnostics per mola module & ROS2 publishers refactored (code clean up)
* BridgeROS2: add source filter for forwarding localization updates to ROS2
* ROS2: base_footprint_frame /tf is broadcasted now as base_link -> base_footprint to avoid /tf warnings (better as a child than as a second parent in the tf tree)
* FIX: In parsing base_footprint_to_base_link_tf
* Contributors: Jose Luis Blanco-Claraco

1.6.1 (2025-02-13)
------------------
* Add new option: publish_tf_from_slam; add better docs on the meaning of all parameters
* Publish georef /tf as /tf_static
* ROS2 bridge now publishes georeferenced map metadata as /tf's and as mrpt_nav_interfaces/GeoreferencingMetadata
* Revert "Feature: all MOLA modules got its MRPT logger to ROS console for easier debugging"
  This reverts commit 8a84611d85022f37b80d8bdcb7acaa1910669fc1.
* FIX: wrong variable in former commit
* Merge pull request `#75 <https://github.com/MOLAorg/mola/issues/75>`_ from MOLAorg/feature/mrpt-to-ros-console
  Feature: all MOLA modules got its MRPT logger to ROS console for easier debugging
* Feature: all MOLA modules got its MRPT logger to ROS console for easier debugging
* Contributors: Jose Luis Blanco-Claraco

1.6.0 (2025-01-21)
------------------
* Publish gridmaps too
* ros2 bridge: rep105 only for map->base_link tfs
* BridgeROS2: support forwarding more than one localization message per timer call
* Fix published /tf's: those from LocalizationSources now can explicitly define their parent and child frames
* Contributors: Jose Luis Blanco-Claraco

1.5.1 (2024-12-29)
------------------

1.5.0 (2024-12-26)
------------------

1.4.1 (2024-12-20)
------------------
* BridgeROS2: add option (now enabled by default) to publish /tfs following REP105 order
* BUG FIX: Published odometry msg lacked target frame_id
* Rename method for better reflecting its goal
* Contributors: Jose Luis Blanco-Claraco

1.4.0 (2024-12-18)
------------------
* Publish localization quality topic
* Forward --ros-args to BridgeROS2
* expose services for runtime parameters
* Load relocalize_from_topic from yaml file
* ros2bridge: handle /initialpose topic -> relocalize service
* Contributors: Jose Luis Blanco-Claraco

1.3.0 (2024-12-11)
------------------
* Support publishing IMU readings MOLA -> ROS2
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------
* BUGFIX: Prevent potential race condition
* Contributors: Jose Luis Blanco-Claraco

1.2.0 (2024-09-16)
------------------
* sort <depend> entries
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
* Merge pull request `#65 <https://github.com/MOLAorg/mola/issues/65>`_ from MOLAorg/add-more-srvs
  Add more Services
* Offer ROS2 services for the new MOLA MapServer interface
* clang-format: switch to 100 columns
* ros2bridge: offer ROS2 services for relocalization
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* change ament linters to apply in test builds
* Contributors: Jose Luis Blanco-Claraco

1.0.8 (2024-07-29)
------------------
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------
* Fix GNSS typo
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2024-06-21)
------------------

1.0.5 (2024-05-28)
------------------

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* BridgeROS2: more robust /tf find_transform by using tf2::BufferCore
* FIXBUG: inverse sensor poses in rosbag2 reader.
  Also: unify notation in C++ calls to lookupTransform()
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------
* update docs
* Contributors: Jose Luis Blanco-Claraco

1.0.1 (2024-03-28)
------------------
* BridgeROS2: do not quit on temporary /tf timeout
* mola_bridge_ros2: option to publish /tf_static for base_footprint
* mola_bridge_ros2: implement missing MOLA->ROS2 conversion for GNSS observations
* BUGFIX: Inverted value of "use_fixed_sensor_pose" was used
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* Comply with ROS2 REP-2003
* Merge ROS2 input and output in one module
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------