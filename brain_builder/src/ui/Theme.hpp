#pragma once
#include <string>
#include <imgui.h>

namespace bb {

// Pin colour by payload type name: the Godot panel's PAYLOAD_TYPE_COLORS table
// (godot_host/project/scripts/ogma_graph_panel.gd) plus the types it lacked.
ImU32  payload_color(std::string const& payload);
// Node title tint by palette category.
ImU32  category_color(std::string const& category);
// Link colour by topic prefix (the panel's _classify_topic categories).
ImU32  topic_color(std::string const& topic);
void   apply_theme(float scale);

} // namespace bb
