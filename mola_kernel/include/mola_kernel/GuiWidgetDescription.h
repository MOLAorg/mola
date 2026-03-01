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
 * @file   GuiWidgetDescription.h
 * @brief  Backend-agnostic description of a GUI sub-window and its widgets.
 *
 * Design goals
 * ------------
 *  - Pure data / std types - no GUI toolkit headers required by clients.
 *  - Compatible with both nanogui (retained mode) and Dear ImGui docking
 *    branch (immediate mode).
 *  - Thread-safe label updates via LiveString.
 *  - Widget values are bound to caller-owned variables (T*) so the module
 *    always holds ground truth; the GUI is a view of it.
 *
 * Nanogui mapping notes
 * ---------------------
 *  WindowDescription  →  nanogui::Window inside CDisplayWindowGUI
 *  Tab                →  nanogui::TabWidget + one tab per Tab entry
 *  Row                →  nanogui::Widget with BoxLayout(Horizontal)
 *  Label              →  nanogui::Label  (caption refreshed via LiveString::poll)
 *  Separator          →  zero-height nanogui::Widget used as spacer
 *  CheckBox           →  nanogui::CheckBox
 *  Button             →  nanogui::Button  (ENTYPO icon optional, ignored if 0)
 *  TextBox            →  nanogui::TextBox (editable)
 *  SliderFloat        →  nanogui::Slider + adjacent nanogui::Label showing value
 *  SliderInt          →  same, integer clamped
 *  ComboBox           →  nanogui::ComboBox
 *
 * Dear ImGui (docking branch) mapping notes
 * ------------------------------------------
 *  WindowDescription  →  ImGui::Begin / End.
 *                         position/size passed as ImGuiCond_FirstUseEver so
 *                         the dock manager can freely override them.
 *  Tab                →  ImGui::BeginTabBar / ImGui::BeginTabItem
 *  Row                →  consecutive widgets separated by ImGui::SameLine()
 *  Label              →  ImGui::TextUnformatted (reads LiveString each frame)
 *  Separator          →  ImGui::Separator()
 *  CheckBox           →  ImGui::Checkbox  (binds directly to bool*)
 *  Button             →  ImGui::Button; callback fired when returns true
 *  TextBox            →  ImGui::InputText (uses internal 256-byte buffer,
 *                         flushed to std::string* on every edit)
 *  SliderFloat        →  ImGui::SliderFloat
 *  SliderInt          →  ImGui::SliderInt
 *  ComboBox           →  ImGui::Combo
 *
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <variant>
#include <vector>

namespace mola::gui
{

// ---------------------------------------------------------------------------
// LiveString - thread-safe label text updated from any thread, consumed by
//              the GUI thread each frame / spinOnce tick.
// ---------------------------------------------------------------------------

/**
 * A string value that can be written from any thread and read (polled) from
 * the GUI thread without data races.
 *
 * Usage (writer side, any thread):
 *   liveStr.set(mrpt::format("ICP quality: %.1f%%", q));
 *
 * Usage (GUI thread, nanogui backend):
 *   std::string tmp;
 *   if (liveStr.poll(tmp)) label->setCaption(tmp);
 *
 * Usage (GUI thread, ImGui backend - called every frame):
 *   ImGui::TextUnformatted(liveStr.display.c_str());
 *   liveStr.pollIntoDisplay();   // updates display if dirty
 */
struct LiveString
{
  LiveString() : id_(nextId_.fetch_add(1, std::memory_order_relaxed)) {}
  explicit LiveString(std::string initial)
      : display(std::move(initial)), id_(nextId_.fetch_add(1, std::memory_order_relaxed))
  {
  }
  ~LiveString() = default;

  /** Globally unique ID for this instance (stable for its lifetime). */
  uint64_t id() const { return id_; }

  // Non-copyable (owns a mutex)
  LiveString(const LiveString&)            = delete;
  LiveString& operator=(const LiveString&) = delete;
  LiveString(LiveString&&)                 = delete;
  LiveString& operator=(LiveString&&)      = delete;

  using Ptr = std::shared_ptr<LiveString>;

  /** Write from any thread. */
  void set(std::string s)
  {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      pending_ = std::move(s);
    }
    dirty_.store(true, std::memory_order_release);
  }

  /**
   * Call from the GUI thread.
   * Copies pending value into `out` and clears the dirty flag.
   * Returns true if the value changed.
   */
  bool poll(std::string& out)
  {
    if (!dirty_.exchange(false, std::memory_order_acq_rel))
    {
      return false;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    out = pending_;
    return true;
  }

  /**
   * Convenience variant for the ImGui backend: updates the public `display`
   * member in-place.  Call this each frame; then read `display` directly.
   */
  void pollIntoDisplay()
  {
    if (!dirty_.exchange(false, std::memory_order_acq_rel))
    {
      return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    display = pending_;
  }

  /**
   * The current display value.  Written only by the GUI thread (via
   * pollIntoDisplay), so safe to read from the GUI thread without locking.
   */
  std::string display;

 private:
  std::string       pending_;
  std::mutex        mtx_;
  std::atomic<bool> dirty_{false};
  uint64_t          id_;

  static inline std::atomic<uint64_t> nextId_{0};
};

// ---------------------------------------------------------------------------
// Forward declarations of widget types
// ---------------------------------------------------------------------------

struct Label;
struct Separator;
struct CheckBox;
struct Button;
struct TextBox;
struct TextPanel;
struct SliderFloat;
struct SliderInt;
struct ComboBox;
struct Row;  // horizontal grouping of other widgets

// ---------------------------------------------------------------------------
// Widget variant
//
// A Row may not nest another Row (keep it flat; both toolkits handle one level
// of horizontal grouping trivially).
// ---------------------------------------------------------------------------

using LeafWidget = std::variant<
    Label, Separator, CheckBox, Button, TextBox, TextPanel, SliderFloat, SliderInt, ComboBox>;

using AnyWidget = std::variant<
    Label, Separator, CheckBox, Button, TextBox, TextPanel, SliderFloat, SliderInt, ComboBox, Row>;

// ---------------------------------------------------------------------------
// Widget definitions
// ---------------------------------------------------------------------------

/**
 * A read-only text label whose content is updated from any thread via
 * LiveString::set().
 *
 * nanogui : nanogui::Label; caption refreshed in spinOnce via poll().
 * ImGui   : ImGui::TextUnformatted(liveStr->display.c_str()) each frame,
 *           preceded by liveStr->pollIntoDisplay().
 *
 * font_size == 0  →  use toolkit default.
 */
struct Label
{
  LiveString::Ptr text;  ///< Shared with the module; never null.
  int             font_size   = 0;
  int             fixed_width = 0;  ///< 0 = auto (nanogui: setFixedWidth)
};

/**
 * A horizontal rule / empty spacer.
 *
 * nanogui : zero-height nanogui::Widget.
 * ImGui   : ImGui::Separator().
 */
struct Separator
{
};

/**
 * A boolean toggle.
 *
 * `value` is read at construction time to set the initial state.
 * `on_change` is called (in the GUI thread) when the user toggles.
 * The callback is responsible for any thread-safe propagation
 * (e.g. via enqueue_request).
 *
 * nanogui : nanogui::CheckBox.
 * ImGui   : ImGui::Checkbox.
 */
struct CheckBox
{
  std::string               label;
  bool                      initial_value = false;
  std::function<void(bool)> on_change;  ///< Called from GUI thread.
};

/**
 * A push button.
 *
 * Icons are specified independently per backend:
 *  - `icon_entypo`  : ENTYPO_ICON_* integer constant (nanogui).
 *  - `icon_imgui`   : UTF-8 string containing the icon codepoint, typically
 *                     a FontAwesome macro such as ICON_FA_SAVE (u8"\uf0c7").
 *                     The ImGui backend prepends this to the label string:
 *                     e.g. ICON_FA_SAVE " Save map now".
 *                     Requires the icon font to have been loaded into the
 *                     ImGui font atlas before the first frame.
 *
 * Either field may be left at its zero/empty default to omit the icon on
 * that backend.  Both can be set simultaneously if you support both backends
 * at runtime.
 */
struct Button
{
  std::string           label;
  int                   icon_entypo = 0;  ///< nanogui: ENTYPO_ICON_* constant
  std::string           icon_imgui;  ///< ImGui: UTF-8 codepoint string, e.g. ICON_FA_SAVE
  int                   font_size = 0;
  std::function<void()> on_click;
};

/**
 * An editable single-line text field.
 *
 * `initial_value` sets the displayed text at construction.
 * `on_change` receives the new string (from the GUI thread); return true to
 * accept the edit (nanogui convention; ImGui always accepts).
 *
 * nanogui : nanogui::TextBox (editable, left-aligned).
 * ImGui   : ImGui::InputText with a 512-byte internal buffer; on_change fired
 *           when the buffer differs from the last committed value.
 */
struct TextBox
{
  std::string                      label;  ///< shown beside the field
  std::string                      initial_value;
  int                              font_size = 0;
  std::function<bool(std::string)> on_change;  ///< Called from GUI thread.
};

/**
 * A multi-line text panel.
 *
 * Read-only mode:
 *  - nanogui : a non-editable nanogui::TextBox with vertical scroll, or a
 *              sequence of nanogui::Label lines inside a scroll panel.
 *              Content is refreshed from `live_text` each spinOnce tick.
 *  - ImGui   : ImGui::InputTextMultiline with ImGuiInputTextFlags_ReadOnly.
 *              live_text->pollIntoDisplay() is called each frame; the widget
 *              reads display directly.  No copy into a separate buffer needed
 *              because ReadOnly mode does not write back.
 *
 * Editable mode:
 *  - nanogui : nanogui::TextBox (multiline variant) with setEditable(true).
 *              on_change fires when the user commits (focus lost / Enter).
 *  - ImGui   : ImGui::InputTextMultiline with a caller-owned std::string
 *              buffer (ImGui 1.89+ supports std::string via InputTextCallback
 *              or the imgui_stdlib.h helper - use InputTextMultiline overload
 *              that takes std::string*).  on_change fires every frame the
 *              content differs from the last committed snapshot.
 *
 * `size_pixels` sets the widget's (width, height) in pixels.
 *  {0, 0} means "fill available width, auto height" for ImGui
 *  (ImVec2(0,0) default) and a reasonable fixed size for nanogui.
 *
 * When `editable` is false, `on_change` is ignored and may be left empty.
 * When `editable` is true,  `live_text` is still used for the *initial*
 * content (set it before calling create_subwindow_from_description); after
 * that the ImGui backend owns the buffer and `live_text` is not polled.
 * The nanogui backend follows the same convention.
 */
struct TextPanel
{
  std::string                             label;  ///< optional heading above the panel
  LiveString::Ptr                         live_text;  ///< content; never null
  bool                                    editable    = false;
  std::array<int, 2>                      size_pixels = {0, 0};  ///< {width, height}, 0 = auto
  std::function<void(const std::string&)> on_change;  ///< editable only
};

/**
 * A floating-point slider in [min_value, max_value].
 *
 * The displayed value beside the slider uses `format_string` (printf style,
 * default "%.2f").
 *
 * nanogui : nanogui::Slider (0–1 normalised internally) + adjacent Label.
 *           The backend maps [min_value, max_value] ↔ [0,1] transparently.
 * ImGui   : ImGui::SliderFloat.
 */
struct SliderFloat
{
  std::string                label;
  float                      initial_value = 0.0f;
  float                      min_value     = 0.0f;
  float                      max_value     = 1.0f;
  std::string                format_string = "%.2f";
  std::function<void(float)> on_change;
  int                        fixed_width = 0;  ///< 0 = auto (nanogui: setFixedWidth on slider)
};

/**
 * An integer slider in [min_value, max_value].
 *
 * nanogui : same as SliderFloat but with integer snapping.
 * ImGui   : ImGui::SliderInt.
 */
struct SliderInt
{
  std::string              label;
  int                      initial_value = 0;
  int                      min_value     = 0;
  int                      max_value     = 100;
  std::function<void(int)> on_change;
  int                      fixed_width = 0;  ///< 0 = auto (nanogui: setFixedWidth on slider)
};

/**
 * A drop-down / combo-box ("pick one of N").
 *
 * `items`         - display strings shown in the list.
 * `initial_index` - zero-based index of the initially selected item.
 * `on_change`     - called with the new index from the GUI thread.
 *
 * nanogui : nanogui::ComboBox.
 * ImGui   : ImGui::Combo.
 */
struct ComboBox
{
  std::string              label;
  std::vector<std::string> items;
  int                      initial_index = 0;
  std::function<void(int)> on_change;
};

/**
 * A horizontal row containing one or more leaf widgets.
 *
 * Rows may NOT be nested.
 *
 * nanogui : nanogui::Widget child with BoxLayout(Horizontal, Alignment::Middle).
 * ImGui   : widgets rendered consecutively separated by ImGui::SameLine().
 *
 * `item_spacing` overrides the inter-widget spacing in pixels (0 = default).
 */
struct Row
{
  std::vector<LeafWidget> widgets;
  int                     item_spacing = 0;  ///< 0 = toolkit default
};

// ---------------------------------------------------------------------------
// Tab and WindowDescription
// ---------------------------------------------------------------------------

/**
 * A named tab page containing a vertical list of widgets (or Rows).
 *
 * nanogui : one tab created via TabWidget::createTab(), with GroupLayout.
 * ImGui   : ImGui::BeginTabItem(title) / EndTabItem block.
 */
struct Tab
{
  std::string            title;
  std::vector<AnyWidget> widgets;
};

/// A single menu item. Separator if label is empty.
struct MenuItem
{
  std::string           label;  ///< Empty string → separator line
  std::string           shortcut;  ///< Display-only hint, e.g. "Ctrl+S"
  bool                  enabled = true;
  std::function<void()> on_click;
};

/// A top-level menu, e.g. "File", "View", "Help"
struct Menu
{
  std::string           label;  ///< e.g. "File"
  std::vector<MenuItem> items;
};

/// The full menu bar for a parent window.
struct MenuBar
{
  std::vector<Menu> menus;
};

/**
 * Full description of a floating sub-window with tabbed content.
 *
 * Position / size semantics
 * -------------------------
 * Both values are treated as *first-use hints*:
 *  - nanogui : setPosition / setFixedWidth called unconditionally (nanogui
 *              does not persist layouts between runs).
 *  - ImGui   : ImGui::SetNextWindowPos(pos, ImGuiCond_FirstUseEver) and
 *              ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver).
 *    This is correct for the docking branch: the dock manager can override
 *    position and size freely after the first frame, and imgui.ini persists
 *    the docked layout across sessions without any extra work.
 *
 * `starts_hidden`
 *  - nanogui : setVisible(false).
 *  - ImGui   : window is not rendered until the module sets a shared
 *              `bool visible` flag (stored alongside the description).
 *              Alternatively, ImGui::SetNextWindowCollapsed(true, FirstUseEver).
 */
struct WindowDescription
{
  std::string title;

  /// Initial position hint (pixels from top-left of the parent window).
  std::array<int, 2> position = {5, 700};

  /// Initial size hint.  height=0 means "auto" (let the toolkit decide).
  std::array<int, 2> size = {340, 0};

  /// If true, the window starts hidden / collapsed.
  bool starts_hidden = false;

  /// The tabs.  If only one tab is provided the backend may omit the tab
  /// bar and render the widgets directly (saves vertical space).
  std::vector<Tab> tabs;
};

}  // namespace mola::gui
