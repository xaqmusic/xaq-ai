#pragma once
// The window: GLFW + Dear ImGui (docking) + imgui-node-editor.  Panels are
// free functions over one AppState so each file owns one panel.
#include <string>
#include <vector>

#include "Catalogue.hpp"

namespace ax { namespace NodeEditor { struct EditorContext; } }

namespace bb {

struct AppState {
    Catalogue   catalogue;
    std::string config_path;
    std::vector<std::string> log;

    // palette
    std::string     palette_search;
    TypeInfo const* palette_selected = nullptr;

    // canvas
    ax::NodeEditor::EditorContext* editor = nullptr;

    float ui_scale     = 1.0f;
    bool  layout_built = false;
    bool  log_scroll   = true;

    void logf(std::string s) { log.push_back(std::move(s)); log_scroll = true; }
};

int  run_app(AppState& st);

void draw_palette(AppState& st);
void draw_canvas(AppState& st);
void draw_log(AppState& st);

} // namespace bb
