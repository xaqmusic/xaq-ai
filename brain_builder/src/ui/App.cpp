#include "ui/App.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <filesystem>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_node_editor.h>

#include "TrialSetup.hpp"
#include "ui/Theme.hpp"

namespace ed = ax::NodeEditor;
namespace fs = std::filesystem;

namespace bb {

namespace {

void glfw_error(int code, char const* msg) { std::fprintf(stderr, "GLFW error %d: %s\n", code, msg); }

void build_dock_layout(ImGuiID dockspace) {
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, ImGui::GetMainViewport()->Size);
    ImGuiID centre = dockspace;
    ImGuiID left   = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left,  0.16f, nullptr, &centre);
    ImGuiID right  = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.32f, nullptr, &centre);
    ImGuiID bottom = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down,  0.22f, nullptr, &centre);
    ImGui::DockBuilderDockWindow("Palette",         left);
    ImGui::DockBuilderDockWindow("Properties",      right);
    ImGui::DockBuilderDockWindow("Validation",      bottom);
    ImGui::DockBuilderDockWindow("Execution Order", bottom);
    ImGui::DockBuilderDockWindow("Log",             bottom);
    ImGui::DockBuilderDockWindow("Canvas",          centre);
    ImGui::DockBuilderFinish(dockspace);
}

std::vector<std::string> list_configs(std::string const& dir) {
    std::vector<std::string> out;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return out;
    for (auto const& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".json") out.push_back(e.path().string());
    std::sort(out.begin(), out.end());
    return out;
}

bool icontains(std::string const& hay, std::string const& needle) {
    if (needle.empty()) return true;
    auto it = std::search(hay.begin(), hay.end(), needle.begin(), needle.end(),
        [](char a, char b) { return std::tolower((unsigned char)a) == std::tolower((unsigned char)b); });
    return it != hay.end();
}

} // namespace

// ---------------------------------------------------------------------------
// document operations
// ---------------------------------------------------------------------------
void AppState::rebuild() {
    if (!body) {
        std::vector<std::string> actions;
        for (size_t i = 0; i < graph.size(); ++i) {
            auto const& p = graph.params(i);
            if (p.contains("action_topics") && p["action_topics"].is_array())
                for (auto const& t : p["action_topics"]) if (t.is_string()) actions.push_back(t.get<std::string>());
        }
        body = bodies.guess(graph.body_manifest(), graph.env_target(), actions);
    }
    wiring = Wiring::build(graph, catalogue, body, cache);
    LayoutMap stored = read_layout(graph);
    for (auto const& [k, v] : layout) if (!stored.count(k)) stored[k] = v;   // moves not yet saved
    bool missing = false;
    for (auto const& n : wiring.nodes) if (!stored.count(n.name)) { missing = true; break; }
    if (missing) {
        LayoutMap a = auto_layout(wiring, catalogue, ui_scale, &node_sizes);
        if (stored.empty()) relayout_after_measure = true;   // a config with no layout: redo once sizes are known
        for (auto const& n : wiring.nodes) if (!stored.count(n.name)) stored[n.name] = a[n.name];
        layout_apply = true;
    }
    layout = std::move(stored);
    wiring_dirty = false;
}

void AppState::open(std::string const& path) {
    try {
        graph = Graph::load(path);
    } catch (std::exception const& e) {
        logf(std::string("open failed: ") + e.what());
        if (!graph.doc.is_object()) new_graph(bodies.find("microduck_joints") ? "microduck_joints" : "");
        return;
    }
    config_path = path;
    body = nullptr;
    if (graph.index_of(selected) < 0) selected.clear();
    text_bufs.clear();
    layout.clear();
    wiring_dirty = true;
    fit_frames = 3;
    logf("opened " + path + " (" + std::to_string(graph.size()) + " modules)");
}

void AppState::new_graph(std::string const& body_id) {
    Body const* b = bodies.find(body_id);
    graph = Graph::empty(b ? b->env_target : "");
    if (b) graph.set_body_manifest(b->id);
    graph.dirty = false;
    config_path.clear();
    body = b;
    selected.clear();
    text_bufs.clear();
    layout.clear();
    wiring_dirty = true;
    fit_frames = 3;
    logf(std::string("new graph") + (b ? " on " + b->title : ""));
}

void AppState::save(std::string const& path) {
    write_layout(graph, layout);
    try {
        graph.save(path);
        config_path = path;
        logf("saved " + path);
    } catch (std::exception const& e) {
        logf(std::string("save failed: ") + e.what());
    }
}

void AppState::select_body(std::string const& id) {
    body = bodies.find(id);
    if (body) graph.set_body_manifest(body->id);
    wiring_dirty = true;
}

void AppState::add_module_at(std::string const& type, float x, float y) {
    TypeInfo const* ti = catalogue.find(type);
    std::string id = graph.add_module(type, ti ? ti->id_prefix : "m", ojson::object());
    layout[id] = {x, y};
    layout_apply = true;
    selected = id;
    wiring_dirty = true;
    logf("added " + id + " (" + type + ")");
}

void AppState::start_dry_run() {
    if (dry_job) return;
    auto job = std::make_unique<DryRunJob>();
    Graph  g;  g.doc = graph.doc;      // a private copy: the operator keeps editing
    Wiring w = wiring;
    Body const* b = body;
    int ticks = dry_ticks;
    DryRunJob* raw = job.get();
    job->thread = std::thread([raw, g = std::move(g), w = std::move(w), b, ticks]() {
        raw->report = dry_run(g, w, b, ticks);
        raw->done = true;
    });
    dry_job = std::move(job);
    logf("dry run: " + std::to_string(ticks) + " ticks started");
}

void AppState::poll_dry_run() {
    if (!dry_job || !dry_job->done) return;
    dry_job->thread.join();
    dry_report = std::move(dry_job->report);
    dry_has_report = true;
    dry_job.reset();
    if (!dry_report.constructed) logf("dry run: " + dry_report.error);
    else logf("dry run: " + std::to_string(dry_report.ticks_done) + " ticks, " + std::to_string(dry_report.actions_seen.size()) +
              " of " + std::to_string(dry_report.actions_seen.size() + dry_report.actions_missing.size()) + " body sinks driven" +
              (dry_report.error.empty() ? "" : " - " + dry_report.error));
}

// ---------------------------------------------------------------------------
// the live link
// ---------------------------------------------------------------------------
bool AppState::live_connect(std::string const& endpoint) {
    std::string host; uint16_t port = 7400;
    if (!parse_endpoint(endpoint, host, port)) { logf("connect: bad endpoint '" + endpoint + "'"); return false; }
    std::string err;
    if (!live.client.connect(host, port, &err)) { logf("connect: " + err); return false; }
    nlohmann::json r = live.client.call({{"verb", "get_graph"}});
    if (r.value("status", "") != "ok") {
        logf("connect: the host answered '" + r.value("message", std::string("?")) + "' - it has no get_graph verb (rebuild mj_host / the Godot host)");
        live.client.close();
        return false;
    }
    live.connected   = true;
    live.source_path = r.value("source_path", "");
    live.version     = uint64_t(r.value("graph_version", int64_t(0)));
    live.synced      = r.value("config", nlohmann::json::object()).value("modules", nlohmann::json::array());
    live.out_of_sync = false;
    live.recreate.clear();

    // The document: the host's source file for metadata and layout when we
    // can read it, otherwise a fresh one; the modules are the host's.
    Graph g;
    bool from_file = false;
    if (!live.source_path.empty() && fs::exists(live.source_path)) {
        try { g = Graph::load(live.source_path); from_file = true; } catch (std::exception const& e) { logf(std::string("connect: ") + e.what()); }
    }
    if (!from_file) g = Graph::empty("");
    adopt_live_modules(g, r.value("config", nlohmann::json::object()));
    g.dirty = false;
    graph = std::move(g);
    config_path = from_file ? live.source_path : "";
    live.synced_revision = graph.revision;
    body = nullptr;
    selected.clear();
    text_bufs.clear();
    layout.clear();
    cache.clear();
    wiring_dirty = true;
    fit_frames = 3;
    logf("LIVE: connected to " + live.client.endpoint() + " - " + std::to_string(graph.size()) + " modules, graph v" +
         std::to_string(live.version) + (from_file ? ", file " + live.source_path : ", no readable source file"));
    return true;
}

void AppState::live_disconnect() {
    if (!live.connected) return;
    live.client.close();
    live.connected = false;
    live.recreate.clear();
    logf("LIVE: disconnected; the document stays for saving");
}

bool AppState::live_pull(bool adopt) {
    nlohmann::json r = live.client.call({{"verb", "get_graph"}});
    if (r.value("status", "") != "ok") { logf("LIVE: get_graph: " + r.value("message", std::string("?"))); return false; }
    live.version = uint64_t(r.value("graph_version", int64_t(0)));
    live.synced  = r.value("config", nlohmann::json::object()).value("modules", nlohmann::json::array());
    if (adopt) {
        graph.checkpoint();
        adopt_live_modules(graph, r.value("config", nlohmann::json::object()));
        live.synced_revision = graph.revision;
        live.out_of_sync = false;
        live.recreate.clear();
        cache.clear();
        wiring_dirty = true;
        logf("LIVE: pulled graph v" + std::to_string(live.version) + " (" + std::to_string(graph.size()) + " modules)");
    }
    return true;
}

void AppState::live_send(nlohmann::json ops, std::string const& what) {
    if (!live.connected || ops.empty()) return;
    nlohmann::json r = live.client.call({{"verb", "apply_patch"}, {"ops", ops}, {"source", "builder"}});
    if (r.value("status", "") == "ok") {
        live.version = uint64_t(r.value("graph_version", int64_t(live.version)));
        logf("LIVE: " + what + " → batch " + std::to_string(r.value("batch_id", int64_t(0))) + ", graph v" + std::to_string(live.version));
    } else {
        live.out_of_sync = true;
        logf("LIVE: " + what + " REFUSED: " + r.value("message", std::string("?")) + "  (Resync from host to realign)");
        if (r.contains("errors")) for (auto const& e : r["errors"]) logf("LIVE:   " + e.get<std::string>());
        if (!live.client.connected()) { live.connected = false; logf("LIVE: connection lost"); }
    }
}

void AppState::live_recreate(bool confirm) {
    if (confirm) {
        nlohmann::json ops = nlohmann::json::array();
        for (auto const& id : live.recreate) for (auto const& o : recreate_ops(graph, id)) ops.push_back(o);
        live_send(ops, "recreate " + std::to_string(live.recreate.size()) + " module(s)");
    } else {
        // Put the host's spec back for those modules.
        graph.checkpoint();
        for (auto const& id : live.recreate) {
            for (auto const& m : live.synced)
                if (m.value("id", "") == id) {
                    if (ojson* doc_m = graph.find(id)) {
                        (*doc_m)["type"]   = m.value("type", "");
                        (*doc_m)["params"] = ojson::parse(m.value("params", nlohmann::json::object()).dump());
                    }
                }
        }
        graph.dirty = true;
        wiring_dirty = true;
        logf("LIVE: reverted " + std::to_string(live.recreate.size()) + " module(s) to the host's spec");
    }
    live.recreate.clear();
    if (!live.out_of_sync) { live.synced = nlohmann::json::parse(graph.doc["modules"].dump()); live.synced_revision = graph.revision; }
}

void AppState::live_tick(double now) {
    if (!live.connected) return;
    // 1. local edits → patches (debounced, never mid-drag)
    if (graph.revision != live.synced_revision && !live.recreate_prompt && !live.out_of_sync &&
        !ImGui::IsAnyItemActive() && now - live.last_change > 0.3) {
        LiveOps d = diff_for_live(graph, catalogue, live.synced);
        for (auto const& n : d.notes) logf("LIVE: " + n);
        if (!d.ops.empty()) live_send(d.ops, std::to_string(d.ops.size()) + " op(s)");
        if (!d.recreate.empty()) { live.recreate = d.recreate; live.recreate_prompt = true; }
        if (!live.out_of_sync) {
            // The host now matches the document except for modules awaiting recreate.
            nlohmann::json synced = nlohmann::json::parse(graph.doc["modules"].dump());
            for (auto const& id : live.recreate)
                for (auto& m : synced) if (m.value("id", "") == id)
                    for (auto const& h : live.synced) if (h.value("id", "") == id) m = h;
            live.synced = std::move(synced);
        }
        live.synced_revision = graph.revision;
    }
    if (graph.revision != live.synced_revision) live.last_change = now;
    // 2. someone else changed the host (the Godot panel, another builder)
    if (now - live.last_poll > 1.0) {
        live.last_poll = now;
        nlohmann::json r = live.client.call({{"verb", "graph_version"}});
        if (r.value("status", "") != "ok") {
            if (!live.client.connected()) { live.connected = false; logf("LIVE: connection lost"); }
            return;
        }
        uint64_t v = uint64_t(r.value("graph_version", int64_t(0)));
        if (v != live.version && live.recreate.empty()) {
            logf("LIVE: the host's graph moved to v" + std::to_string(v) + " - pulling");
            live_pull(true);
        }
    }
}

// ---------------------------------------------------------------------------
void draw_log(AppState& st) {
    if (ImGui::Begin("Log")) {
        if (ImGui::SmallButton("clear")) st.log.clear();
        ImGui::Separator();
        ImGui::BeginChild("log_lines", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
        for (auto const& line : st.log) ImGui::TextUnformatted(line.c_str());
        if (st.log_scroll) ImGui::SetScrollHereY(1.0f);
        st.log_scroll = false;
        ImGui::EndChild();
    }
    ImGui::End();
}

namespace {

void draw_dialogs(AppState& st) {
    if (st.show_open) { ImGui::OpenPopup("Open config"); st.show_open = false; st.dialog_search.clear(); }
    if (st.show_saveas) { ImGui::OpenPopup("Save config as"); st.show_saveas = false; st.dialog_path = st.config_path; }
    if (st.show_new) { ImGui::OpenPopup("New graph"); st.show_new = false; }

    ImGui::SetNextWindowSize(ImVec2(760 * st.ui_scale, 520 * st.ui_scale), ImGuiCond_Appearing);
    if (ImGui::BeginPopupModal("Open config", nullptr, ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##search", "filter (name, or a path to open)", &st.dialog_search);
        ImGui::BeginChild("files", ImVec2(0, -ImGui::GetFrameHeightWithSpacing() * 1.5f), true);
        for (char const* dir : {BB_MJ_CONFIG_DIR, BB_GODOT_CONFIG_DIR}) {
            std::vector<std::string> files = list_configs(dir);
            std::string title = std::string(dir).find("mj_host") != std::string::npos ? "mj_host/configs" : "godot_host configs";
            if (!ImGui::CollapsingHeader(title.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) continue;
            for (auto const& f : files) {
                std::string name = fs::path(f).filename().string();
                if (!icontains(name, st.dialog_search)) continue;
                if (ImGui::Selectable(name.c_str())) { st.open(f); ImGui::CloseCurrentPopup(); }
            }
        }
        ImGui::EndChild();
        if (ImGui::Button("Open path") && fs::exists(st.dialog_search)) { st.open(st.dialog_search); ImGui::CloseCurrentPopup(); }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    ImGui::SetNextWindowSize(ImVec2(700 * st.ui_scale, 0), ImGuiCond_Appearing);
    if (ImGui::BeginPopupModal("Save config as", nullptr, ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::TextDisabled("mj_host configs live in %s", BB_MJ_CONFIG_DIR);
        ImGui::SetNextItemWidth(-1);
        bool enter = ImGui::InputText("##path", &st.dialog_path, ImGuiInputTextFlags_EnterReturnsTrue);
        if ((ImGui::Button("Save") || enter) && !st.dialog_path.empty()) { st.save(st.dialog_path); ImGui::CloseCurrentPopup(); }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (st.live.show_dialog) { ImGui::OpenPopup("Connect to a running brain"); st.live.show_dialog = false; }
    if (ImGui::BeginPopupModal("Connect to a running brain", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::TextDisabled("the host's control socket (mj_host or the Godot host; the inspector may stay connected)");
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 18);
        bool enter = ImGui::InputText("host:port", &st.live.dialog_endpoint, ImGuiInputTextFlags_EnterReturnsTrue);
        if (st.graph.dirty) ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "unsaved changes will be replaced by the host's graph");
        if (ImGui::Button("Connect") || enter) { if (st.live_connect(st.live.dialog_endpoint)) ImGui::CloseCurrentPopup(); }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (st.live.recreate_prompt) { ImGui::OpenPopup("Recreate on the host?"); st.live.recreate_prompt = false; }
    if (ImGui::BeginPopupModal("Recreate on the host?", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::TextUnformatted("These edits change construction-only params (topics, dimensions, seeds).");
        ImGui::TextUnformatted("A running module cannot change them: it must be removed and re-added,");
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "which loses everything it has learned.");
        ImGui::Separator();
        for (auto const& id : st.live.recreate) ImGui::BulletText("%s", id.c_str());
        ImGui::Separator();
        if (ImGui::Button("Recreate them")) { st.live_recreate(true); ImGui::CloseCurrentPopup(); }
        ImGui::SameLine();
        if (ImGui::Button("Revert my edits")) { st.live_recreate(false); ImGui::CloseCurrentPopup(); }
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopupModal("New graph", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::TextUnformatted("Body:");
        for (auto const& b : st.bodies.bodies)
            if (ImGui::Selectable(b.title.c_str())) { st.new_graph(b.id); ImGui::CloseCurrentPopup(); }
        if (ImGui::Selectable("(no body)")) { st.new_graph(""); ImGui::CloseCurrentPopup(); }
        ImGui::Separator();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void draw_menu(AppState& st, GLFWwindow* window) {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New...", "Ctrl+N")) st.show_new = true;
            if (ImGui::MenuItem("Open...", "Ctrl+O")) st.show_open = true;
            if (ImGui::MenuItem("Save", "Ctrl+S")) { if (st.config_path.empty()) st.show_saveas = true; else st.save(st.config_path); }
            if (ImGui::MenuItem("Save as...")) st.show_saveas = true;
            if (ImGui::MenuItem("Publish...", "Ctrl+P")) st.show_publish = true;
            ImGui::Separator();
            if (ImGui::MenuItem("Connect to a running brain...", nullptr, false, !st.live.connected)) st.live.show_dialog = true;
            if (ImGui::MenuItem("Resync from host", nullptr, false, st.live.connected)) st.live_pull(true);
            if (ImGui::MenuItem("Disconnect", nullptr, false, st.live.connected)) st.live_disconnect();
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "Ctrl+Q")) glfwSetWindowShouldClose(window, 1);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Undo", "Ctrl+Z", false, st.graph.can_undo())) { st.graph.undo(); st.wiring_dirty = true; }
            if (ImGui::MenuItem("Redo", "Ctrl+Y", false, st.graph.can_redo())) { st.graph.redo(); st.wiring_dirty = true; }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Body")) {
            for (auto const& b : st.bodies.bodies)
                if (ImGui::MenuItem(b.title.c_str(), nullptr, st.body == &b)) st.select_body(b.id);
            if (ImGui::MenuItem("(none)", nullptr, st.body == nullptr)) { st.body = nullptr; st.wiring_dirty = true; }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem("Auto layout")) { st.layout = auto_layout(st.wiring, st.catalogue, st.ui_scale, &st.node_sizes); st.layout_apply = true; st.fit_frames = 3; }
            if (ImGui::MenuItem("Fit to content", "F")) st.fit_frames = 3;
            if (ImGui::MenuItem("Reset panels")) st.layout_built = false;
            ImGui::EndMenu();
        }
        // status, right-aligned
        if (st.live.connected) {
            std::string live = "LIVE " + st.live.client.endpoint() + " v" + std::to_string(st.live.version) + (st.live.out_of_sync ? "  OUT OF SYNC" : "");
            ImGui::SameLine(0, 30);
            ImGui::TextColored(st.live.out_of_sync ? ImVec4(1.0f, 0.55f, 0.2f, 1.0f) : ImVec4(0.4f, 0.85f, 0.45f, 1.0f), "%s", live.c_str());
        }
        std::string status = st.config_path.empty() ? "(unsaved graph)" : fs::path(st.config_path).filename().string();
        if (st.graph.dirty) status += " *";
        status += "   body: " + std::string(st.body ? st.body->title : "none");
        status += "   " + std::to_string(st.wiring.errors) + " errors, " + std::to_string(st.wiring.warnings) + " warnings";
        float w = ImGui::CalcTextSize(status.c_str()).x;
        ImGui::SameLine(ImGui::GetWindowWidth() - w - 16 * st.ui_scale);
        if (st.wiring.errors) ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", status.c_str());
        else ImGui::TextDisabled("%s", status.c_str());
        ImGui::EndMainMenuBar();
    }
}

void handle_shortcuts(AppState& st, GLFWwindow* window) {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) return;
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Q, false)) glfwSetWindowShouldClose(window, 1);
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false)) { if (st.config_path.empty()) st.show_saveas = true; else st.save(st.config_path); }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false)) st.show_open = true;
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_N, false)) st.show_new = true;
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_P, false)) st.show_publish = true;
    if (io.KeyCtrl && !io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false) && st.graph.can_undo()) { st.graph.undo(); st.wiring_dirty = true; }
    if (io.KeyCtrl && (ImGui::IsKeyPressed(ImGuiKey_Y, false) || (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false))) && st.graph.can_redo()) { st.graph.redo(); st.wiring_dirty = true; }
}

} // namespace

int run_app(AppState& st) {
    warm_registry();
    glfwSetErrorCallback(glfw_error);
    if (!glfwInit()) { std::fprintf(stderr, "glfwInit failed\n"); return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(1600, 1000, "Brain Builder", nullptr, nullptr);
    if (!window) { std::fprintf(stderr, "glfwCreateWindow failed\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    float xs = 1.0f, ys = 1.0f;
    glfwGetWindowContentScale(window, &xs, &ys);
    st.ui_scale = xs > 0.0f ? xs : 1.0f;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable | ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
    // A real TTF when the system has one (the default bitmap font is ASCII-only).
    {
        char const* candidates[] = {"/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
                                    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
                                    "/usr/share/fonts/dejavu/DejaVuSansMono.ttf"};
        bool loaded = false;
        for (char const* f : candidates)
            if (!loaded && fs::exists(f)) {
                ImFontConfig fc; fc.OversampleH = 2; fc.OversampleV = 2;
                loaded = io.Fonts->AddFontFromFileTTF(f, std::round(14.0f * st.ui_scale), &fc, io.Fonts->GetGlyphRangesDefault()) != nullptr;
            }
        io.FontGlobalScale = loaded ? 1.0f : st.ui_scale;
    }
    apply_theme(st.ui_scale);
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    ed::Config cfg;
    cfg.SettingsFile = nullptr;
    st.editor = ed::CreateEditor(&cfg);

    st.logf("catalogue: " + std::to_string(st.catalogue.types.size()) + " module types; " +
            std::to_string(st.bodies.bodies.size()) + " body manifests");
    for (auto const& w : st.bodies.warnings) st.logf("bodies: " + w);
    if (st.live_on_start) { if (!st.live_connect(st.live.dialog_endpoint)) st.new_graph(""); }
    else if (!st.config_path.empty()) st.open(st.config_path);
    else st.new_graph(st.bodies.bodies.empty() ? "" : (st.bodies.find("microduck_joints") ? "microduck_joints" : st.bodies.bodies.front().id));

    std::string last_title;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) { ImGui_ImplGlfw_Sleep(10); continue; }
        if (st.wiring_dirty) st.rebuild();
        st.poll_dry_run();
        st.live_tick(glfwGetTime());

        std::string title = "Brain Builder - " + (st.config_path.empty() ? std::string("unsaved") : st.config_path) + (st.graph.dirty ? " *" : "");
        if (title != last_title) { glfwSetWindowTitle(window, title.c_str()); last_title = title; }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiID dockspace = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
        if (!st.layout_built) { build_dock_layout(dockspace); st.layout_built = true; }

        draw_menu(st, window);
        handle_shortcuts(st, window);
        draw_dialogs(st);
        draw_publish(st);
        draw_palette(st);
        draw_canvas(st);
        draw_properties(st);
        draw_order(st);
        draw_validation(st);
        draw_log(st);

        if (st.relayout_after_measure) {
            bool measured = true;
            for (auto const& n : st.wiring.nodes) if (!st.node_sizes.count(n.name)) measured = false;
            if (measured) {
                st.layout = auto_layout(st.wiring, st.catalogue, st.ui_scale, &st.node_sizes);
                st.layout_apply = true;
                st.fit_frames = 3;
                st.relayout_after_measure = false;
            }
        }

        ImGui::Render();
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.10f, 0.10f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    if (st.dry_job) { st.dry_job->thread.join(); st.dry_job.reset(); }
    st.live_disconnect();
    ed::DestroyEditor(st.editor);
    st.editor = nullptr;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace bb
