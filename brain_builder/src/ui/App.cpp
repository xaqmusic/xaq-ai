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
    if (!st.config_path.empty()) st.open(st.config_path);
    else st.new_graph(st.bodies.bodies.empty() ? "" : (st.bodies.find("microduck_joints") ? "microduck_joints" : st.bodies.bodies.front().id));

    std::string last_title;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) { ImGui_ImplGlfw_Sleep(10); continue; }
        if (st.wiring_dirty) st.rebuild();

        std::string title = "Brain Builder — " + (st.config_path.empty() ? std::string("unsaved") : st.config_path) + (st.graph.dirty ? " *" : "");
        if (title != last_title) { glfwSetWindowTitle(window, title.c_str()); last_title = title; }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiID dockspace = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
        if (!st.layout_built) { build_dock_layout(dockspace); st.layout_built = true; }

        draw_menu(st, window);
        handle_shortcuts(st, window);
        draw_dialogs(st);
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
