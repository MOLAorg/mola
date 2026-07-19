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
 *         Translates mola::gui::WindowDescription => ImGui draw calls.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <imgui.h>
#include <imgui_stdlib.h>
#include <mola_viz_imgui/MolaVizImGuiCore.h>

using namespace mola;

void MolaVizImGuiCore::render_widget_description(
    const mola::gui::WindowDescription& desc, SubWindowState& /*sw*/)
{
  if (desc.tabs.empty()) return;

  if (desc.tabs.size() == 1)
  {
    render_tab(desc.tabs.front(), desc.title + "##" + desc.tabs.front().title);
    return;
  }

  if (ImGui::BeginTabBar(("##tabs_" + desc.title).c_str()))
  {
    for (const auto& tab : desc.tabs)
    {
      if (ImGui::BeginTabItem(tab.title.c_str()))
      {
        render_tab(tab, desc.title + "##" + tab.title);
        ImGui::EndTabItem();
      }
    }
    ImGui::EndTabBar();
  }
}

void MolaVizImGuiCore::render_tab(const mola::gui::Tab& tab, const std::string& ctx)
{
  for (size_t i = 0; i < tab.widgets.size(); ++i)
  {
    render_any_widget(tab.widgets[i], ctx + "|" + std::to_string(i));
  }
}

void MolaVizImGuiCore::render_any_widget(const mola::gui::AnyWidget& w, const std::string& ctx)
{
  std::visit(
      [this, &ctx](auto&& widget)
      {
        using T = std::decay_t<decltype(widget)>;
        if constexpr (std::is_same_v<T, mola::gui::Row>)
        {
          for (size_t i = 0; i < widget.widgets.size(); ++i)
          {
            if (i > 0)
            {
              const float spacing = widget.item_spacing > 0
                                        ? static_cast<float>(widget.item_spacing)
                                        : ImGui::GetStyle().ItemSpacing.x;
              ImGui::SameLine(0.0f, spacing);
            }
            render_leaf_widget(widget.widgets[i], ctx + "." + std::to_string(i));
          }
        }
        else
        {
          render_leaf_widget(widget, ctx);
        }
      },
      w);
}

void MolaVizImGuiCore::render_leaf_widget(const mola::gui::LeafWidget& w, const std::string& ctx)
{
  std::visit(
      [this, &ctx](auto&& widget)
      {
        using T = std::decay_t<decltype(widget)>;

        if constexpr (std::is_same_v<T, mola::gui::Label>)
        {
          if (!widget.text) return;
          widget.text->pollIntoDisplay();
          ImGui::TextUnformatted(widget.text->display.c_str());
        }
        else if constexpr (std::is_same_v<T, mola::gui::Separator>)
        {
          ImGui::Separator();
        }
        else if constexpr (std::is_same_v<T, mola::gui::CheckBox>)
        {
          auto&             states = widget_checkbox_states_;
          const std::string key    = ctx + "##" + widget.label;
          auto              it     = states.find(key);
          if (it == states.end()) it = states.emplace(key, widget.initial_value).first;
          if (ImGui::Checkbox(widget.label.c_str(), &it->second) && widget.on_change)
            widget.on_change(it->second);
        }
        else if constexpr (std::is_same_v<T, mola::gui::Button>)
        {
          const std::string label =
              widget.icon_imgui.empty() ? widget.label : (widget.icon_imgui + "  " + widget.label);
          if (ImGui::Button(label.c_str()) && widget.on_click) widget.on_click();
        }
        else if constexpr (std::is_same_v<T, mola::gui::TextBox>)
        {
          if (!widget.label.empty()) ImGui::TextUnformatted(widget.label.c_str());
          auto&             buffers = widget_text_buffers_;
          const std::string key     = ctx + "##" + widget.label;
          auto              it      = buffers.find(key);
          if (it == buffers.end()) it = buffers.emplace(key, widget.initial_value).first;
          const std::string id = "##tb_" + key;
          if (ImGui::InputText(id.c_str(), &it->second, ImGuiInputTextFlags_EnterReturnsTrue) &&
              widget.on_change)
            widget.on_change(it->second);
        }
        else if constexpr (std::is_same_v<T, mola::gui::TextPanel>)
        {
          if (!widget.label.empty()) ImGui::TextUnformatted(widget.label.c_str());
          const ImVec2 size{
              static_cast<float>(widget.size_pixels[0]), static_cast<float>(widget.size_pixels[1])};
          const uint64_t lsId = widget.live_text->id();
          if (widget.editable)
          {
            auto& bufs = widget_textpanel_bufs_;
            auto  it   = bufs.find(lsId);
            if (it == bufs.end())
            {
              widget.live_text->pollIntoDisplay();
              it = bufs.emplace(lsId, widget.live_text->display).first;
            }
            const std::string id = "##tpe_" + std::to_string(lsId);
            if (ImGui::InputTextMultiline(id.c_str(), &it->second, size) && widget.on_change)
              widget.on_change(it->second);
          }
          else
          {
            widget.live_text->pollIntoDisplay();
            const std::string id = "##tpr_" + std::to_string(lsId);
            ImGui::InputTextMultiline(
                id.c_str(), const_cast<char*>(widget.live_text->display.c_str()),
                widget.live_text->display.size() + 1, size, ImGuiInputTextFlags_ReadOnly);
          }
        }
        else if constexpr (std::is_same_v<T, mola::gui::SliderFloat>)
        {
          auto&             vals = widget_slider_float_vals_;
          const std::string key  = ctx + "##" + widget.label;
          auto              it   = vals.find(key);
          if (it == vals.end()) it = vals.emplace(key, widget.initial_value).first;
          const bool userChanged = ImGui::SliderFloat(
              widget.label.c_str(), &it->second, widget.min_value, widget.max_value,
              widget.format_string.c_str());
          if (userChanged)
          {
            if (widget.on_change) widget.on_change(it->second);
          }
          else if (widget.live_value && !ImGui::IsItemActive() && widget.live_value->dirty())
          {
            it->second = widget.live_value->poll();
          }
        }
        else if constexpr (std::is_same_v<T, mola::gui::SliderInt>)
        {
          auto&             vals = widget_slider_int_vals_;
          const std::string key  = ctx + "##" + widget.label;
          auto              it   = vals.find(key);
          if (it == vals.end()) it = vals.emplace(key, widget.initial_value).first;
          if (ImGui::SliderInt(
                  widget.label.c_str(), &it->second, widget.min_value, widget.max_value) &&
              widget.on_change)
            widget.on_change(it->second);
        }
        else if constexpr (std::is_same_v<T, mola::gui::ComboBox>)
        {
          auto&             indices = widget_combo_indices_;
          const std::string key     = ctx + "##" + widget.label;
          auto              it      = indices.find(key);
          if (it == indices.end()) it = indices.emplace(key, widget.initial_index).first;
          std::string items_str;
          for (const auto& s : widget.items)
          {
            items_str += s;
            items_str += '\0';
          }
          items_str += '\0';
          const std::string id = widget.label.empty() ? ("##combo_" + key) : widget.label;
          if (widget.fixed_width > 0)
            ImGui::SetNextItemWidth(static_cast<float>(widget.fixed_width));
          if (ImGui::Combo(id.c_str(), &it->second, items_str.c_str()) && widget.on_change)
            widget.on_change(it->second);
        }
      },
      w);
}
