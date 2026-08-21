^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_viz_imgui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix mola_viz_imgui GUI rendering nothing depending on link order
  MolaVizImGui::DEFAULT_WINDOW_NAME was copy-initialized from
  MolaVizImGuiCore::DEFAULT_WINDOW_NAME, a std::string global in another
  translation unit, whose initialization order is unspecified. When the
  module TU is initialized first, the copy ends up empty, the host window
  gets registered under "" and every VizInterface call (which defaults to
  the literal "main" from VizInterface.h) misses, so the window opens but
  stays empty, with "unknown parentWindow 'main'" warnings.
  Both string objects are now initialized from a new constexpr literal,
  DEFAULT_WINDOW_NAME_LITERAL, so they no longer depend on each other.
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
* gui: limit width of the speed rate UI
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
* imgui windows: append profile name to window title
* Merge pull request `#179 <https://github.com/MOLAorg/mola/issues/179>`_ from MOLAorg/chore/clang-format-viz-handlers
  style: clang-format fix in MolaVizImGui_handlers.cpp
* perf: avoid a redundant image-buffer copy and heap alloc per frame
  Two per-frame savings in toCompressedImage():
  - The temporary sensor_msgs::msg::CompressedImage only needs to live for
  the duration of this call; a stack-local avoids a heap allocation
  cv_bridge::toCvCopy() doesn't require (it takes the message by const
  reference, unlike the raw-Image path's toCvShare(), which needs a
  shared_ptr).
  - cv_ptr (and the cv::Mat it owns) is likewise local to this call, so
  copying it into the CObservationImage with mrpt::img::SHALLOW_COPY
  (ref-counted) instead of DEEP_COPY avoids duplicating the decoded
  pixel buffer a second time.
  Re-verified end-to-end against the Oxford Spires dataset: no errors.
  (CodeRabbit finding on PR `#178 <https://github.com/MOLAorg/mola/issues/178>`_)
* style: clang-format fix in MolaVizImGui_handlers.cpp
  Pre-existing violation on develop (introduced by 3de93af2, whose title
  claims "+ clang-format" but didn't quite match clang-format-14's
  formatting for the multi-clause "else if (init; cond)" statement),
  currently failing CI clang-format-check on develop itself and blocking
  that check from going green on any PR against this repo.
* chore(viz): fixed-width Hz in sensor info overlay + clang-format
* Contributors: Jose Luis Blanco-Claraco

* gui: limited width of the speed-rate UI control; imgui windows now show the
  profile name in their title.
* feat(kernel): console window now captures child-module loggers (e.g.
  background loop-closure engine) via the new child_loggers() hook.
* feat: ComboBox GUI widget gained an optional fixed_width parameter (both
  backends).
* perf: avoided a redundant image-buffer copy and heap allocation per frame in
  toCompressedImage(); fixed a pre-existing clang-format violation blocking
  CI; fixed-width Hz display in the sensor info overlay.
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-07-17)
------------------
* fix format
* feat: remove the obsolete text messages overlay mechanism in mola_viz_imgui (superseded by console subwindow)
* fix: default-dock Dataset_UI panel at top; fix Console dock on old layouts
  Add WindowDescription::dock_top_by_default so a window without a saved
  imgui.ini entry gets docked into a strip reserved at the top of the main
  window instead of floating; enabled for the per-module Dataset_UI panel.
  Also make the Console's default bottom-dock (and the new top-dock) key off
  whether that specific window has a saved imgui.ini entry, instead of whether
  the whole ini file already existed. This fixes both never docking new
  window kinds added to an already-existing layout, and splitting a stale,
  no-longer-leaf dock node once more than one default dock has been applied.
* fix: dont use deprecated mrpt2.x point cloud classes
* feat: autodock log window; update menu item names
* transparent icon
* Merge branch 'feat/imviz-metric-plots' into develop
* Remove deprecated nanogui-specific VizInterface API
  create_subwindow(), enqueue_custom_nanogui_code(), subwindow_grid_layout()
  and subwindow_move_resize() have no remaining callers now that
  create_subwindow_from_description() and enqueue_custom_gui_code() cover
  their use cases on both backends.
* Merge pull request `#172 <https://github.com/MOLAorg/mola/issues/172>`_ from MOLAorg/feat/imviz-metric-plots
  feat(mola_viz_imgui): add metric time-series plot windows
* fix: load params from yaml
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
* Merge pull request `#170 <https://github.com/MOLAorg/mola/issues/170>`_ from MOLAorg/fix/console-log-window-hardening
  fix(mola_viz_imgui): Console log window perf/config follow-ups
* fix(mola_viz_imgui): Console log window perf/config follow-ups
  - Default the pipeline-wide log capture level to INFO instead of DEBUG:
  capturing DEBUG from every module forced every MRPT_LOG_DEBUG_STREAM()
  callsite in the whole pipeline to always format its message just to feed
  the sink, even though the UI hid Debug entries by default anyway. Now
  runtime-adjustable from a new "Capture" combo in the Console toolbar
  (with a tooltip explaining it's distinct from the Levels display filter),
  and re-applied to already-hooked modules on every discovery tick so a
  runtime change takes effect immediately.
  - Avoid a full deque-of-strings copy under ConsoleLogSink's mutex on every
  rendered frame: added a lock-free version() counter (bumped on
  push()/clear()) so the Console window only re-snapshots when the sink
  actually changed, instead of unconditionally at up to target_fps\_ Hz --
  that copy was contending with module threads trying to push() new lines.
  See `MRPT/mrpt#1375 <https://github.com/MRPT/mrpt/issues/1375>`_ for a related fix in COutputLogger itself (an
  unsynchronized concurrent access to its callback list, triggered by this
  window's pattern of registering a callback on an already-running module
  from the GUI thread).
* fix(mola_viz_imgui): sensor Hz readout underestimates real rate
  show_common_sensor_info() is invoked once per GUI frame, not once per
  new observation, since render_sensor_windows() redraws the last-seen
  observation every frame while waiting for the next one. The Hz
  low-pass filter was treating each of those repeat-frame calls (same
  timestamp, At=0) as a genuine 0 Hz sample, dragging the running
  estimate well below the sensor's real rate. Now the filter is only
  updated when the timestamp actually advances.
* Merge pull request `#166 <https://github.com/MOLAorg/mola/issues/166>`_ from MOLAorg/feat/console-log-window
  feat: add auto-exposed Console log window to MolaVizImGui
* fix: address coderabbitai review on Console log window
  - ConsoleLogSink::push() now also evicts entries past window_seconds,
  not just past max_entries, so the rolling buffer honors both bounds
  as documented.
  - Track the Console source filter by name instead of combo index: the
  underlying source set is sorted and its order shifts as new modules
  register, so an index alone could silently start pointing at a
  different module. The combo index is now re-derived from the stored
  name each frame.
  - agents.md: clarify that the Console subwindow was added after v2.6,
  not as part of it.
* feat: add auto-exposed Console log window to MolaVizImGui
  Aggregates mrpt-logger output from all running ExecutableBase modules
  into a dockable, filterable "Console" subwindow (level/source/text
  filters, color-by-level, Save-to-file), gated by a console_enabled
  param. Capture is done via logRegisterCallback() into a thread-safe
  rolling buffer bounded by entry count and age.
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
* feat: imgui now has a clear all method
* Merge pull request `#151 <https://github.com/MOLAorg/mola/issues/151>`_ from MOLAorg/fix/imgui-slider
  fix: imgui slider
* fix: imgui slider
* fix docs
* Merge pull request `#148 <https://github.com/MOLAorg/mola/issues/148>`_ from MOLAorg/refactor/imgui
  refactor: split into MolaVizImGuiCore and MolaVizImGui to enable 3rdparty GUIs
* fix(mola_viz_imgui): address review findings in ImGui viz backend
  - Move widget state from process-global statics to per-instance members
  - Tab-qualify widget context so same-labeled widgets across tabs don't collide
  - Persist sub-window open flag so closing sticks across frames
  - Clamp negative decay_time_seconds to avoid size_t wraparound
  - Guard null dynamic_pointer_cast in update_3d_object / clear_all_point_clouds_with_decay
  - Drain pending GUI tasks in shutdown_for_embed (broken_promise instead of hang)
  - Stop leaking GLFW/ImGui backend headers from the public header; forward-declare GLFWwindow
* refactor: split into MolaVizImGuiCore and MolaVizImGui to enable 3rdparty GUIs
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

2.9.0 (2026-05-11)
------------------
* fix: Imgui background scene color was washed out
* Merge pull request `#144 <https://github.com/MOLAorg/mola/issues/144>`_ from MOLAorg/imgui-improvements
  fix: background scene doesn't respond to mouse
* fix: background scene doesn't respond to mouse
* Merge pull request `#112 <https://github.com/MOLAorg/mola/issues/112>`_ from MOLAorg/feat/dear-imgui-viz
  Introduce Dear ImGui alternative viz module
* Cleaner shutdown order
* Introduce Dear ImGui alternative viz module
* Contributors: Jose Luis Blanco-Claraco

2.8.0 (2026-04-29)
------------------
