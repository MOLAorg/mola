^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_viz_imgui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
