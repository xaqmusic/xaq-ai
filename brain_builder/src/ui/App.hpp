#pragma once
// The window: GLFW + Dear ImGui (docking) + imgui-node-editor.  Panels are
// free functions over one AppState so each file owns one panel.
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "Body.hpp"
#include "DryRun.hpp"
#include "Order.hpp"
#include "Catalogue.hpp"
#include "Graph.hpp"
#include "Layout.hpp"
#include "Wiring.hpp"

namespace ax { namespace NodeEditor { struct EditorContext; } }

namespace bb {

struct AppState {
    Catalogue    catalogue;
    BodyRegistry bodies;
    Graph        graph;
    Body const*  body = nullptr;
    Wiring       wiring;
    TrialCache   cache;
    LayoutMap    layout;
    SizeMap      node_sizes;              // measured each frame by the canvas
    bool         relayout_after_measure = false;

    bool wiring_dirty = true;     // rebuild the wiring model next frame
    bool layout_apply = false;    // push `layout` into the editor next frame
    int  fit_frames   = 0;        // >0: NavigateToContent once positions and sizes exist

    std::string config_path;      // what was asked for on the command line
    std::vector<std::string> log;
    std::string status;

    // selection / panels
    std::string selected;         // module id, "" for none
    std::string palette_search;
    TypeInfo const* palette_selected = nullptr;
    std::string prop_filter;
    bool prop_required_only = false, prop_set_only = false, prop_sockets_only = false, prop_hot_only = false;
    std::unordered_map<std::string, std::string> text_bufs;   // per-field edit buffers
    std::string pending_key;      // numeric field being edited
    double      pending_num = 0;

    // dry run (on a worker; the report lands on the Validation panel)
    struct DryRunJob {
        std::thread       thread;
        std::atomic<bool> done{false};
        DryRunReport      report;
    };
    std::unique_ptr<DryRunJob> dry_job;
    DryRunReport dry_report;
    bool         dry_has_report = false;
    int          dry_ticks = 50;

    // execution-order suggestion awaiting apply
    OrderSuggestion order_suggestion;
    bool            order_preview = false;

    // the "+" pin chooser
    bool        plus_open = false;
    std::string plus_node, plus_topic, plus_payload;

    // dialogs
    bool   show_open = false, show_saveas = false, show_new = false;
    std::string dialog_path, dialog_search;

    // canvas
    ax::NodeEditor::EditorContext* editor = nullptr;
    float canvas_mouse_x = 0, canvas_mouse_y = 0;
    float ui_scale     = 1.0f;
    bool  layout_built = false;
    bool  log_scroll   = true;

    void logf(std::string s) { log.push_back(std::move(s)); log_scroll = true; }

    // document operations (App.cpp)
    void open(std::string const& path);
    void new_graph(std::string const& body_id);
    void save(std::string const& path);
    void select_body(std::string const& id);
    void rebuild();
    void add_module_at(std::string const& type, float x, float y);
    void start_dry_run();
    void poll_dry_run();
};

int  run_app(AppState& st);

void draw_palette(AppState& st);
void draw_canvas(AppState& st);
void draw_properties(AppState& st);
void draw_order(AppState& st);
void draw_validation(AppState& st);
void draw_log(AppState& st);

} // namespace bb
