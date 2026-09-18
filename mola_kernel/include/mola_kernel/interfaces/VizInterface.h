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
 * @file   VizInterface.h
 * @brief  Virtual visualization interface (see MolaViz)
 * @author Jose Luis Blanco Claraco
 * @date   Sep 9, 2020
 *
 * Changelog
 * ---------
 * 2026: Added create_subwindow_from_description() and
 *       enqueue_custom_gui_code() as backend-agnostic replacements for the
 *       old nanogui-specific create_subwindow() / enqueue_custom_nanogui_code()
 *       APIs (removed).
 * 2026: Added register_metric() / push_metric() for streaming time-series
 *       metrics to live plot windows (ImGui backend only; no-op on nanogui).
 */
#pragma once

#include <mola_kernel/GuiWidgetDescription.h>
#include <mola_kernel/interfaces/MetricChannel.h>
#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/viz/viz_frwds.h>

#include <future>
#include <memory>
#include <optional>

/** Feature macro: when defined, VizInterface offers update_3d_object_frame()
 *  and the `parentFrame` argument of update_3d_object(),
 *  insert_point_cloud_with_decay(), and update_viewport_look_at(), i.e. named
 *  movable reference-frame nodes that child objects / camera targets can be
 *  resolved against.  Lets out-of-repo modules (e.g. an older
 *  mola_lidar_odometry checkout) detect the feature at compile time. */
#define MOLA_KERNEL_VIZ_HAS_MOVABLE_FRAMES 1

namespace mola
{

/** Virtual visualization interface.
 *
 * Implemented by MolaViz (nanogui backend) and MolaVizImGui (Dear ImGui
 * docking-branch backend).  Clients should program against this interface
 * exclusively and avoid toolkit-specific types.
 *
 * \ingroup mola_kernel_interfaces_grp
 */
class VizInterface
{
 public:
  VizInterface() = default;
  virtual ~VizInterface();

  VizInterface(const VizInterface&)            = default;
  VizInterface& operator=(const VizInterface&) = default;
  VizInterface(VizInterface&&)                 = default;
  VizInterface& operator=(VizInterface&&)      = default;

  using Ptr = std::shared_ptr<VizInterface>;

  // =========================================================================
  /** @name Backend-agnostic sub-window API  (preferred)
   *
   * These methods work identically with all VizInterface backends.
   * New code should use these exclusively.
   * @{ */

  /**
   * \brief Creates a sub-window described by a GuiWidgetDescription.
   *
   * The description carries the window title, initial position/size hints,
   * tab structure, and all widget definitions including their initial values
   * and on_change callbacks.
   *
   * The returned future resolves once the window has been created in the GUI
   * thread and all widgets are live.  Callers that need to know when
   * construction is complete should call `.get()` on the future.
   *
   * Position and size in the description are treated as *first-use hints*:
   *  - nanogui backend  : applied unconditionally via setPosition/setFixedWidth.
   *  - ImGui backend    : passed as ImGuiCond_FirstUseEver, so the dock
   *                       manager and imgui.ini take over from the second run.
   *
   * Label text is updated at any time and from any thread by calling
   * LiveString::set() on the LiveString instances embedded in the description.
   * The backend polls them on each frame / spinOnce tick.
   *
   * \param desc         Full window and widget description.
   * \param parentWindow Name of the parent host window (default: "main").
   * \return             future<void> that resolves when the window is live.
   *
   * \sa mola::gui::WindowDescription, mola::gui::LiveString
   */
  virtual std::future<void> create_subwindow_from_description(
      const mola::gui::WindowDescription& desc, const std::string& parentWindow = "main") = 0;

  /**
   * \brief Enqueues arbitrary code to run on the GUI thread at the next
   *        available opportunity.
   *
   * Use this for any GUI-thread work that is not covered by the widget
   * description API: e.g. updating widget enable/disable state, changing
   * slider ranges at runtime, or interacting with toolkit-specific objects
   * via a cast of the opaque handle returned by get_subwindow_handle().
   *
   * The callable is executed in the GUI thread; do not call blocking
   * operations inside it.
   *
   * This replaces the deprecated enqueue_custom_nanogui_code().
   */
  virtual std::future<void> enqueue_custom_gui_code(const std::function<void()>& userCode) = 0;

  /**
   * \brief Returns a short identifier string for the active GUI backend.
   *
   * Clients may use this to conditionally tune their GUIs - for example to
   * choose between an Entypo icon (nanogui) and a FontAwesome codepoint
   * (ImGui), or to skip features not yet supported on a given backend.
   *
   * Canonical values are provided as static constants below:
   *   VizInterface::BACKEND_NANOGUI   →  "nanogui"
   *   VizInterface::BACKEND_IMGUI     →  "imgui"
   *
   * Comparison example:
   * \code
   *   if (visualizer_->gui_backend() == VizInterface::BACKEND_IMGUI) { ... }
   * \endcode
   *
   * \note The method is pure virtual so each concrete backend is forced to
   *       declare its identity explicitly.  It is noexcept and const so it
   *       is safe to call from any context, including hot loops.
   */
  [[nodiscard]] virtual const std::string& gui_backend() const noexcept = 0;

  /// Canonical backend name returned by MolaViz (nanogui).
  static const std::string BACKEND_NANOGUI;  // defined in VizInterface.cpp as "nanogui"

  /// Canonical backend name returned by MolaVizImGui (Dear ImGui).
  static const std::string BACKEND_IMGUI;  // defined in VizInterface.cpp as "imgui"

  /**
   * \brief Installs or replaces the menu bar of a parent window.
   *
   * Menu bars are a first-class feature of the Dear ImGui docking branch
   * (ImGui::BeginMainMenuBar / BeginMenuBar).  The nanogui backend has no
   * native equivalent and provides a no-op stub that resolves the future
   * immediately without rendering anything.  Callers should guard with
   * gui_backend() if the menus are essential to their workflow:
   *
   * \code
   *   if (visualizer_->gui_backend() == VizInterface::BACKEND_IMGUI)
   *       visualizer_->set_menu_bar(bar);
   * \endcode
   *
   * \param bar          Full menu bar description.
   * \param parentWindow Host window whose menu bar is replaced.
   * \return             future<void> resolving when the menu bar is live.
   */
  virtual std::future<void> set_menu_bar(
      const mola::gui::MenuBar& bar, const std::string& parentWindow = "main") = 0;

  /**
   * \brief Returns an opaque handle to a previously created sub-window.
   *
   * The handle type depends on the backend:
   *  - MolaViz (nanogui)   : cast to `nanogui::Window*`
   *  - MolaVizImGui        : the handle is not meaningful as a pointer;
   *                          use enqueue_custom_gui_code() with ImGui calls
   *                          keyed on the window title instead.
   *
   * Returns nullptr if the window does not exist yet or the backend does not
   * support retained window objects.
   *
   * \note This is an intentional "escape hatch" for toolkit-specific code that
   *       cannot be expressed through the description API.  Prefer
   *       enqueue_custom_gui_code() over casting this pointer wherever
   *       possible, as that keeps the calling module's cpp file free of GUI
   *       toolkit headers.
   */
  virtual void* get_subwindow_handle(
      const std::string& subWindowTitle, const std::string& parentWindow = "main") = 0;

  /** @} */

  // =========================================================================
  /** @name 3-D scene API  (backend-agnostic)
   * @{ */

  /**
   * \brief Updates or inserts a 3D object in the background scene.
   *
   * The update is performed in the GUI thread.  If the named object already
   * exists, its contents are replaced by copying the shared pointers inside
   * `obj`.
   *
   * \param objName        Name used to identify the object (upsert key).
   * \param obj            Object to display.
   * \param viewportName   Viewport inside the parent window.
   * \param parentWindow   Host window name.
   * \param parentFrame    Optional name of a movable reference-frame node
   *                       previously created via update_3d_object_frame().
   *                       When non-empty, the object is attached as a child of
   *                       that frame (so moving the frame moves the object
   *                       without re-rendering it); the frame node is
   *                       auto-created at the identity pose if it does not
   *                       exist yet.  Empty (default) attaches to the viewport
   *                       root, as before.
   * \return               future<bool>; resolves to true when executed.
   */
  virtual std::future<bool> update_3d_object(
      const std::string& objName, const std::shared_ptr<mrpt::viz::CSetOfObjects>& obj,
      const std::string& viewportName = "main", const std::string& parentWindow = "main",
      const std::string& parentFrame = "") = 0;

  /**
   * \brief Creates or repositions a named movable "reference frame" node.
   *
   * A frame node is an (initially empty) container placed at the viewport
   * root.  Objects can be attached to it via the `parentFrame` argument of
   * update_3d_object(); moving the frame then relocates all attached objects
   * at once, without re-uploading their geometry.  This is how a back end such
   * as mola_mapper_3d keeps a front end's dense clouds / local map (drawn once
   * in its own `{odom_i}` frame) correctly placed in `{map}` as it estimates
   * the `T_map_to_odom_i` transform online.
   *
   * Idempotent: the first call creates the node; later calls only update its
   * pose, preserving any already-attached children.
   *
   * \param frameName    Name of the frame node (upsert key). Must be non-empty.
   * \param pose         Pose of the frame in the viewport (i.e. in `{map}`).
   * \param viewportName Viewport inside the parent window.
   * \param parentWindow Host window name.
   * \return             future<bool>; resolves to true when executed.
   */
  virtual std::future<bool> update_3d_object_frame(
      const std::string& frameName, const mrpt::math::TPose3D& pose,
      const std::string& viewportName = "main", const std::string& parentWindow = "main") = 0;

  /**
   * \brief Inserts a temporary point cloud visible for `decay_time_seconds`.
   *
   * \param cloud              Cloud to display.
   * \param decay_time_seconds Lifetime in seconds before the cloud fades out.
   * \param viewportName       Target viewport.
   * \param parentWindow       Host window name.
   * \param parentFrame        If non-empty, the cloud is attached as a child of
   *                           the named movable frame node (same semantics as
   *                           the parentFrame argument of update_3d_object()).
   * \return                   future<bool> resolving to true when executed.
   */
  virtual std::future<bool> insert_point_cloud_with_decay(
      const std::shared_ptr<mrpt::viz::CPointCloudColoured>& cloud, double decay_time_seconds,
      const std::string& viewportName = "main", const std::string& parentWindow = "main",
      const std::string& parentFrame = "") = 0;

  /**
   * \brief Removes all clouds previously inserted with
   *        insert_point_cloud_with_decay().
   */
  virtual std::future<bool> clear_all_point_clouds_with_decay(
      const std::string& viewportName = "main", const std::string& parentWindow = "main") = 0;

  /**
   * \brief Moves the viewport camera look-at point.
   *
   * \param lookAt       Target point in the local coordinate frame of
   *                     `parentFrame` (if given) or in world/scene space.
   * \param parentFrame  If non-empty, the backend looks up the named movable
   *                     frame node, applies its current scene pose to `lookAt`,
   *                     and uses the resulting world-space point as the target.
   *                     This makes the camera follow a vehicle rendered under
   *                     a frame node rather than chasing its odom-frame coords.
   */
  virtual std::future<bool> update_viewport_look_at(
      const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName = "main",
      const std::string& parentWindow = "main", const std::string& parentFrame = "") = 0;

  /**
   * \brief Rotates the viewport camera around the vertical axis.
   *
   * \param azimuth                  Angle in radians.
   * \param absolute_falseForRelative true → set absolute azimuth;
   *                                  false → add increment to current value.
   */
  virtual std::future<bool> update_viewport_camera_azimuth(
      double azimuth, bool absolute_falseForRelative = true,
      const std::string& viewportName = "main", const std::string& parentWindow = "main") = 0;

  /** Switches the viewport camera between perspective and orthographic. */
  virtual std::future<bool> update_viewport_camera_orthographic(
      bool orthographic, const std::string& viewportName = "main",
      const std::string& parentWindow = "main") = 0;

  /**
   * \brief Executes arbitrary user code on the mrpt::viz::Scene of the
   *        background viewport.
   *
   * Use this to modify viewport properties, add sub-viewports, change
   * background colour, etc.  The callable runs in the GUI thread.
   *
   * This method operates on the mrpt OpenGL scene, not on GUI toolkit
   * widgets, so it is backend-agnostic: both nanogui and ImGui backends
   * embed an mrpt scene in their OpenGL canvas.
   */
  virtual std::future<bool> execute_custom_code_on_background_scene(
      const std::function<void(mrpt::viz::Scene&)>& userCode,
      const std::string&                            parentWindow = "main") = 0;

  /** @} */

  // =========================================================================
  /** @name Sensor observation handler API
   * @{ */

  /**
   * \brief Updates a sub-window's content from an MRPT object.
   *
   * The correct handler is selected based on the runtime type of `obj`.
   * Custom handlers can be registered via MolaViz::register_gui_handler().
   *
   * \return future<bool> resolving to false if no handler exists for the
   *         object's type.
   */
  virtual std::future<bool> subwindow_update_visualization(
      const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
      const mrpt::containers::yaml* extra_parameters = nullptr,
      const std::string&            parentWindow     = "main") = 0;

  /** @} */

  // =========================================================================
  /** @name Console / log output
   * @{ */

  /** Appends a line of text to the on-screen console overlay. */
  virtual std::future<bool> output_console_message(
      const std::string& msg, const std::string& parentWindow = "main") = 0;

  /** @} */

  // =========================================================================
  /** @name File dialog API  (backend-agnostic)
   * @{ */

  /**
   * \brief Opens a modal file-picker dialog on the GUI thread.
   *
   * The call is non-blocking: it enqueues the dialog to open on the GUI
   * thread and returns a future that resolves when the user dismisses it.
   *
   * \param title       Dialog window title, e.g. "Open point cloud".
   * \param save        true  → "Save as" dialog (prompts before overwrite).
   *                    false → "Open" dialog (validates file exists).
   * \param filters     List of {description, comma-separated-extensions} pairs.
   *                    Example: {{"LAS files","las,laz"},{"All files","*"}}
   *                    Pass an empty vector to show all files.
   * \param default_path  Initial directory or full path suggestion.
   *                      Empty string means the backend's default (usually
   *                      the current working directory).
   * \param parentWindow  Host window name.
   * \return            future resolving to the chosen absolute path, or
   *                    std::nullopt if the user cancelled.
   *
   * Backend notes
   * -------------
   *  nanogui : nanogui::file_dialog(), called from the GUI thread via
   *            enqueue_custom_gui_code().  Blocks the GUI thread while open
   *            (nanogui's native behaviour); the future resolves on return.
   *  ImGui   : delegates to a file-dialog library (e.g. ImGuiFileDialog or
   *            NFD).  The dialog is rendered over successive frames; the
   *            future resolves on the frame the user confirms or cancels.
   *            The main render loop is not blocked.
   *
   * \note Both backends resolve the future on the GUI thread.  Callers
   *       should not call .get() from the GUI thread itself to avoid
   *       deadlock; instead chain work in a separate thread or use
   *       .wait_for() with zero duration to poll.
   */
  virtual std::future<std::optional<std::string>> open_file_dialog(
      const std::string& title, bool save,
      const std::vector<std::pair<std::string, std::string>>& filters = {},
      const std::string& default_path = "", const std::string& parentWindow = "main") = 0;

  /** @} */

  // =========================================================================
  /** @name Metrics / time-series plotting
   *
   * Generic mechanism for any module to stream timestamped scalar values
   * ("metrics": CPU %, per-frame delay, ICP uncertainties, residuals, queue
   * depth, temperatures, ...) to the visualizer for display in live,
   * autoscrolling plot windows.
   *
   * This is an ImGui-backend feature (rendered via ImPlot). The nanogui
   * `MolaViz` backend implements both methods as no-ops: `register_metric()`
   * returns a live handle whose `push()` is a no-op, so callers can use the
   * API unconditionally without checking `gui_backend()`.
   * @{ */

  /**
   * \brief Registers (or returns the existing) metric channel by unique name.
   *
   * Idempotent: repeated calls with the same name return the same channel.
   *
   * \param name Unique id, also the default legend label. Use a namespaced
   *             form like "lidar_odom/icp_delay_ms" to avoid collisions.
   * \param unit Optional unit string shown on axis/legend (e.g. "ms", "%").
   * \return     A channel handle; never null (no-op handle on backends
   *             without plotting).
   */
  virtual MetricChannel::Ptr register_metric(
      const std::string& name, const std::string& unit = "") = 0;

  /**
   * \brief One-shot convenience: register-if-needed then push.
   *
   * For rarely-updated metrics; prefer holding the handle returned by
   * register_metric() on hot paths to avoid a name lookup per sample.
   */
  virtual void push_metric(const std::string& name, double t, double value) = 0;

  /** @} */
};

}  // namespace mola