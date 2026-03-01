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
 * @file   MolaViz.h
 * @brief  Main C++ class for MOLA GUI  (nanogui backend)
 * @author Jose Luis Blanco Claraco
 * @date   May  11, 2019
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mrpt/gui/CDisplayWindowGUI.h>

#include <deque>
#include <future>
#include <memory>
#include <optional>
#include <set>
#include <shared_mutex>
#include <vector>

namespace mola
{
/** MOLA GUI and visualization API - nanogui backend.
 *
 * Implements VizInterface using nanogui / CDisplayWindowGUI.
 * For the Dear ImGui backend see MolaVizImGui.
 *
 * \ingroup mola_viz_grp
 */
class MolaViz : public ExecutableBase, public VizInterface
{
  DEFINE_MRPT_OBJECT(MolaViz, mola)

 public:
  MolaViz();
  ~MolaViz() override;

  // Non-copyable, non-movable (owns a GUI thread and static singleton).
  MolaViz(const MolaViz&)            = delete;
  MolaViz& operator=(const MolaViz&) = delete;
  MolaViz(MolaViz&&)                 = delete;
  MolaViz& operator=(MolaViz&&)      = delete;

  // ExecutableBase overrides
  void initialize(const Yaml& cfg) override;
  void spinOnce() override;

  // =========================================================================
  /** @name mola-viz main API
   * @{ */

  using window_name_t    = std::string;
  using subwindow_name_t = std::string;

  static const window_name_t DEFAULT_WINDOW_NAME;

  static bool     IsRunning();
  static MolaViz* Instance();

  /** @} */

  // =========================================================================
  /** @name VizInterface - backend-agnostic API (preferred)
   * @{ */

  /** Returns "nanogui".  \sa VizInterface::BACKEND_NANOGUI */
  [[nodiscard]] const std::string& gui_backend() const noexcept override;

  std::future<void> create_subwindow_from_description(
      const mola::gui::WindowDescription& desc,
      const std::string&                  parentWindow = DEFAULT_WINDOW_NAME) override;

  /** Enqueues a callable to run on the GUI thread at the next frame.
   *  The callable must not block.  Replaces enqueue_custom_nanogui_code(). */
  std::future<void> enqueue_custom_gui_code(const std::function<void()>& userCode) override;

  /** Returns the underlying nanogui::Window* cast to void*, or nullptr if the
   *  sub-window does not exist yet.  Cast only inside nanogui-aware .cpp files. */
  void* get_subwindow_handle(
      const std::string& subWindowTitle,
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<std::optional<std::string>> open_file_dialog(
      const std::string& title, bool save,
      const std::vector<std::pair<std::string, std::string>>& filters      = {},
      const std::string&                                      default_path = "",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  /** Does nothing in nanogui backend. */
  std::future<void> set_menu_bar(
      const mola::gui::MenuBar& bar,
      const std::string&        parentWindow = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name VizInterface - 3D scene API
   * @{ */

  std::future<bool> update_3d_object(
      const std::string& objName, const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> insert_point_cloud_with_decay(
      const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud, double decay_time_seconds,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> clear_all_point_clouds_with_decay(
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> update_viewport_look_at(
      const mrpt::math::TPoint3Df& lookAt, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> update_viewport_camera_azimuth(
      double azimuth, bool absolute_falseForRelative = true,
      const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> update_viewport_camera_orthographic(
      bool orthographic, const std::string& viewportName = "main",
      const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  std::future<bool> execute_custom_code_on_background_scene(
      const std::function<void(mrpt::opengl::Scene&)>& userCode,
      const std::string&                               parentWindow = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name VizInterface - observation handler API
   * @{ */

  std::future<bool> subwindow_update_visualization(
      const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
      const mrpt::containers::yaml* extra_parameters = nullptr,
      const std::string&            parentWindow     = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name VizInterface - console output
   * @{ */

  std::future<bool> output_console_message(
      const std::string& message, const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name VizInterface - deprecated nanogui-specific API
   *
   * All three methods delegate to their replacements above.
   * They will be removed in a future release.
   * @{ */

  /// \deprecated Use create_subwindow_from_description() instead.
  [[deprecated("Use create_subwindow_from_description() instead")]] std::future<nanogui::Window*>
      create_subwindow(
          const std::string& subWindowTitle,
          const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  /// \deprecated Use enqueue_custom_gui_code() instead.
  [[deprecated("Use enqueue_custom_gui_code() instead")]] std::future<void>
      enqueue_custom_nanogui_code(const std::function<void()>& userCode) override;

  /// \deprecated Encode layout in WindowDescription instead.
  [[deprecated("Encode layout in WindowDescription instead")]] std::future<void>
      subwindow_grid_layout(
          const std::string& subWindowTitle, bool orientationVertical, int resolution,
          const std::string& parentWindow = DEFAULT_WINDOW_NAME) override;

  /// \deprecated Encode position/size in WindowDescription instead.
  [[deprecated("Encode position/size in WindowDescription instead")]] std::future<void>
      subwindow_move_resize(
          const std::string& subWindowTitle, const mrpt::math::TPoint2D_<int>& location,
          const mrpt::math::TPoint2D_<int>& size,
          const std::string&                parentWindow = DEFAULT_WINDOW_NAME) override;

  /** @} */

  // =========================================================================
  /** @name GUI update handler registry
   * @{ */

  using update_handler_t = std::function<void(
      const mrpt::rtti::CObject::Ptr&, nanogui::Window* subWin, const window_name_t& parentWin,
      MolaViz* instance, const mrpt::containers::yaml* extra_parameters)>;
  using class_name_t     = std::string;

  static void register_gui_handler(const class_name_t& name, const update_handler_t& handler);

  /** @} */

  // =========================================================================
  /** @name Module parameters
   * @{ */

  double       console_text_font_size_   = 9.0;
  unsigned int max_console_lines_        = 5;
  bool         show_rgbd_as_point_cloud_ = false;

  /// Assumed sensor rate [Hz] used to compute max number of decaying clouds
  /// from the decay_time_seconds parameter.
  double assumed_sensor_rate_hz_ = 10.0;

  /** @} */

  /** Schedules a nanogui performLayout() for `name` at the next frame.
   *  Called from GUI-thread helpers that add or resize widgets. */
  void markWindowForReLayout(const window_name_t& name)
  {
    auto lck = mrpt::lockHelper(guiThreadPendingTasksMtx_);
    guiThreadMustReLayoutTheseWindows_.insert(name);
  }

 private:
  // ---------------------------------------------------------------------------
  // Singleton
  // ---------------------------------------------------------------------------
  static MolaViz*          instance_;
  static std::shared_mutex instanceMtx_;

  // ---------------------------------------------------------------------------
  // Per-window state
  // ---------------------------------------------------------------------------

  struct DecayingCloud
  {
    DecayingCloud() = default;
    DecayingCloud(
        std::string                                               opengl_viewport_name_,
        const std::shared_ptr<mrpt::opengl::CPointCloudColoured>& cloud_, float initial_alpha_)
        : opengl_viewport_name(std::move(opengl_viewport_name_)),
          cloud(cloud_),
          initial_alpha(initial_alpha_)
    {
    }

    std::string                                        opengl_viewport_name;
    std::shared_ptr<mrpt::opengl::CPointCloudColoured> cloud;
    float                                              initial_alpha = 1.0f;
  };

  struct PerWindowData
  {
    mrpt::gui::CDisplayWindowGUI::Ptr win;
    std::vector<std::string>          console_messages;
    std::deque<DecayingCloud>         decaying_clouds;
    size_t                            max_decaying_clouds = 100;
  };

  std::map<window_name_t, PerWindowData>                                windows_;
  std::map<window_name_t, std::map<subwindow_name_t, nanogui::Window*>> subWindows_;

  mrpt::gui::CDisplayWindowGUI::Ptr create_and_add_window(const window_name_t& name);

  mutable std::shared_mutex subWindowsMtx_;

  // ---------------------------------------------------------------------------
  // GUI thread and task queue
  // ---------------------------------------------------------------------------

  std::thread guiThread_;
  void        gui_thread();

  using task_queue_t = std::vector<std::function<void()>>;
  task_queue_t            guiThreadPendingTasks_;
  std::set<window_name_t> guiThreadMustReLayoutTheseWindows_;
  std::mutex              guiThreadPendingTasksMtx_;

  // ---------------------------------------------------------------------------
  // LiveString polling (called every GUI frame from gui_thread loop callback)
  // ---------------------------------------------------------------------------

  /** Walks all managed sub-window widgets, finds those tagged with a
   *  LiveString pointer, polls them, and updates the widget caption/value
   *  if dirty.  Called from the GUI thread only. */
  void poll_live_strings_in_subwindows_();

  std::map<uint64_t, mola::gui::LiveString::Ptr> liveStringRegistry_;

  // ---------------------------------------------------------------------------
  // Dataset_UI auto-generated panels
  // ---------------------------------------------------------------------------

  double lastTimeCheckForNewModules_ = 0;
  double lastTimeUpdateDatasetUIs_   = 0;

  struct DataPerDatasetUI
  {
    std::weak_ptr<mola::Dataset_UI> module;

    bool first_time_seen = true;

    /// Live label text updated by dataset_ui_update() via LiveString::set().
    /// The nanogui widget polls this every frame via poll_live_strings_in_subwindows_().
    mola::gui::LiveString::Ptr lbPlaybackPosition;

    // Note: slider, cbPaused, cmRate are now owned by the nanogui widget tree
    // created via create_subwindow_from_description().  They are not held here
    // because the description callbacks capture weak_ptr<Dataset_UI> directly,
    // making raw widget pointers unnecessary.
  };
  std::map<std::string, DataPerDatasetUI> datasetUIs_;

  void dataset_ui_check_new_modules();
  void dataset_ui_update();

  // ---------------------------------------------------------------------------
  // Decaying clouds
  // ---------------------------------------------------------------------------

  void internal_handle_decaying_clouds();
};

}  // namespace mola