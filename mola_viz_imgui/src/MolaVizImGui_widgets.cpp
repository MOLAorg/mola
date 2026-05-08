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
 * @file   MolaVizImGui_widgets.cpp
 * @brief  Dear ImGui widget description renderer.
 *         Translates mola::gui::WindowDescription → ImGui draw calls.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <imgui.h>  // Must come before <imgui_stdlib.h>
#include <imgui_stdlib.h>  // ImGui::InputText(std::string*)
#include <mola_viz_imgui/MolaVizImGui.h>

using namespace mola;

// ---------------------------------------------------------------------------
// render_widget_description
// Renders all tabs (or flat content if single tab) for a SubWindowState.
// Called from the GUI thread inside an open ImGui::Begin/End block.
// ---------------------------------------------------------------------------

void MolaVizImGui::render_widget_description(
    const mola::gui::WindowDescription& desc, SubWindowState& /*sw*/)
{
  if (desc.tabs.empty())
  {
    return;
  }

  if (desc.tabs.size() == 1)
  {
    // Single-tab optimisation: skip the tab bar.
    render_tab(desc.tabs.front(), desc.title);
    return;
  }

  if (ImGui::BeginTabBar(("##tabs_" + desc.title).c_str()))
  {
    for (const auto& tab : desc.tabs)
    {
      if (ImGui::BeginTabItem(tab.title.c_str()))
      {
        render_tab(tab, desc.title);
        ImGui::EndTabItem();
      }
    }
    ImGui::EndTabBar();
  }
}

// ---------------------------------------------------------------------------
// render_tab
// ---------------------------------------------------------------------------

void MolaVizImGui::render_tab(const mola::gui::Tab& tab, const std::string& ctx)
{
  for (const auto& anyW : tab.widgets)
  {
    render_any_widget(anyW, ctx);
  }
}

// ---------------------------------------------------------------------------
// render_any_widget — dispatches Row vs leaf
// ---------------------------------------------------------------------------

void MolaVizImGui::render_any_widget(const mola::gui::AnyWidget& w, const std::string& ctx)
{
  std::visit(
      [this, &ctx](auto&& widget)
      {
        using T = std::decay_t<decltype(widget)>;
        if constexpr (std::is_same_v<T, mola::gui::Row>)
        {
          bool first = true;
          for (const auto& leaf : widget.widgets)
          {
            if (!first)
            {
              const float spacing = widget.item_spacing > 0
                                        ? static_cast<float>(widget.item_spacing)
                                        : ImGui::GetStyle().ItemSpacing.x;
              ImGui::SameLine(0.0f, spacing);
            }
            render_leaf_widget(leaf, ctx);
            first = false;
          }
        }
        else
        {
          render_leaf_widget(widget, ctx);
        }
      },
      w);
}

// ---------------------------------------------------------------------------
// render_leaf_widget — one ImGui call per widget type
// ---------------------------------------------------------------------------

void MolaVizImGui::render_leaf_widget(const mola::gui::LeafWidget& w, const std::string& ctx)
{
  std::visit(
      [&ctx](auto&& widget)
      {
        using T = std::decay_t<decltype(widget)>;

        // ── Label ──────────────────────────────────────────────────────────
        if constexpr (std::is_same_v<T, mola::gui::Label>)
        {
          if (!widget.text) return;
          // Poll LiveString every frame (cheap: atomic load + possible mutex).
          widget.text->pollIntoDisplay();
          ImGui::TextUnformatted(widget.text->display.c_str());
        }

        // ── Separator ──────────────────────────────────────────────────────
        else if constexpr (std::is_same_v<T, mola::gui::Separator>)
        {
          ImGui::Separator();
        }

        // ── CheckBox ───────────────────────────────────────────────────────
        else if constexpr (std::is_same_v<T, mola::gui::CheckBox>)
        {
          // Keyed by (ctx + "##" + label) so two checkboxes with the same
          // label in different sub-windows do not share state.
          static std::map<std::string, bool> states;
          const std::string                  key = ctx + "##" + widget.label;
          auto                               it  = states.find(key);
          if (it == states.end()) it = states.emplace(key, widget.initial_value).first;

          if (ImGui::Checkbox(widget.label.c_str(), &it->second))
          {
            if (widget.on_change)
            {
              widget.on_change(it->second);
            }
          }
        }

        // ── Button ─────────────────────────────────────────────────────────
        else if constexpr (std::is_same_v<T, mola::gui::Button>)
        {
          // Prepend ImGui icon codepoint if provided:
          const std::string label =
              widget.icon_imgui.empty() ? widget.label : (widget.icon_imgui + "  " + widget.label);

          if (ImGui::Button(label.c_str()) && widget.on_click)
          {
            widget.on_click();
          }
        }

        // ── TextBox (single-line editable) ─────────────────────────────────
        else if constexpr (std::is_same_v<T, mola::gui::TextBox>)
        {
          if (!widget.label.empty())
          {
            ImGui::TextUnformatted(widget.label.c_str());
          }

          static std::map<std::string, std::string> buffers;
          const std::string                         key = ctx + "##" + widget.label;
          auto                                      it  = buffers.find(key);
          if (it == buffers.end())
          {
            it = buffers.emplace(key, widget.initial_value).first;
          }

          const std::string id = "##tb_" + key;
          if (ImGui::InputText(id.c_str(), &it->second, ImGuiInputTextFlags_EnterReturnsTrue))
          {
            if (widget.on_change)
            {
              widget.on_change(it->second);
            }
          }
        }

        // ── TextPanel (multi-line) ─────────────────────────────────────────
        else if constexpr (std::is_same_v<T, mola::gui::TextPanel>)
        {
          if (!widget.label.empty())
          {
            ImGui::TextUnformatted(widget.label.c_str());
          }

          const ImVec2 size{
              static_cast<float>(widget.size_pixels[0]), static_cast<float>(widget.size_pixels[1])};

          // Key buffers by LiveString::id() (stable monotonic u64 per
          // instance) rather than by raw pointer — a destroyed LiveString
          // can have its address reused by a new instance, aliasing a
          // stale buffer into the new widget.
          const uint64_t lsId = widget.live_text->id();

          if (widget.editable)
          {
            // Editable: imgui_stdlib InputTextMultiline with std::string buffer.
            static std::map<uint64_t, std::string> bufs;
            auto                                   it = bufs.find(lsId);
            if (it == bufs.end())
            {
              // Seed from LiveString on first use only:
              widget.live_text->pollIntoDisplay();
              it = bufs.emplace(lsId, widget.live_text->display).first;
            }
            const std::string id = "##tpe_" + std::to_string(lsId);
            if (ImGui::InputTextMultiline(id.c_str(), &it->second, size) && widget.on_change)
            {
              widget.on_change(it->second);
            }
          }
          else
          {
            // Read-only: poll LiveString each frame, display with InputTextMultiline.
            widget.live_text->pollIntoDisplay();
            const std::string id = "##tpr_" + std::to_string(lsId);
            ImGui::InputTextMultiline(
                id.c_str(), const_cast<char*>(widget.live_text->display.c_str()),
                widget.live_text->display.size() + 1, size, ImGuiInputTextFlags_ReadOnly);
          }
        }

        // ── SliderFloat ────────────────────────────────────────────────────
        else if constexpr (std::is_same_v<T, mola::gui::SliderFloat>)
        {
          static std::map<std::string, float> vals;
          const std::string                   key = ctx + "##" + widget.label;
          auto                                it  = vals.find(key);
          if (it == vals.end()) it = vals.emplace(key, widget.initial_value).first;

          if (ImGui::SliderFloat(
                  widget.label.c_str(), &it->second, widget.min_value, widget.max_value,
                  widget.format_string.c_str()))
          {
            if (widget.on_change) widget.on_change(it->second);
          }
        }

        // ── SliderInt ──────────────────────────────────────────────────────
        else if constexpr (std::is_same_v<T, mola::gui::SliderInt>)
        {
          static std::map<std::string, int> vals;
          const std::string                 key = ctx + "##" + widget.label;
          auto                              it  = vals.find(key);
          if (it == vals.end()) it = vals.emplace(key, widget.initial_value).first;

          if (ImGui::SliderInt(
                  widget.label.c_str(), &it->second, widget.min_value, widget.max_value))
          {
            if (widget.on_change) widget.on_change(it->second);
          }
        }

        // ── ComboBox ───────────────────────────────────────────────────────
        else if constexpr (std::is_same_v<T, mola::gui::ComboBox>)
        {
          static std::map<std::string, int> indices;
          // Use ctx + "##" + label as the stable key.  When label is empty
          // we still get a unique-per-window key from ctx alone.
          const std::string key = ctx + "##" + widget.label;
          auto              it  = indices.find(key);
          if (it == indices.end()) it = indices.emplace(key, widget.initial_index).first;

          // ImGui::Combo needs a null-terminated item list:
          std::string items_str;
          for (const auto& s : widget.items)
          {
            items_str += s;
            items_str += '\0';
          }
          items_str += '\0';

          // The ImGui id uses "##key" so the visible label stays clean.
          const std::string id = widget.label.empty() ? ("##combo_" + key) : widget.label;

          if (ImGui::Combo(id.c_str(), &it->second, items_str.c_str()))
            if (widget.on_change) widget.on_change(it->second);
        }
      },
      w);
}
