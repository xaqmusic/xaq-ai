#include "ui/App.hpp"

#include <cstdio>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_node_editor.h>

#include "TrialSetup.hpp"
#include "ui/Theme.hpp"

namespace ed = ax::NodeEditor;

namespace bb {

namespace {

void glfw_error(int code, char const* msg) {
    std::fprintf(stderr, "GLFW error %d: %s\n", code, msg);
}

void build_dock_layout(ImGuiID dockspace) {
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, ImGui::GetMainViewport()->Size);
    ImGuiID centre = dockspace;
    ImGuiID left   = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left,  0.18f, nullptr, &centre);
    ImGuiID right  = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.30f, nullptr, &centre);
    ImGuiID bottom = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down,  0.25f, nullptr, &centre);
    ImGui::DockBuilderDockWindow("Palette",         left);
    ImGui::DockBuilderDockWindow("Properties",      right);
    ImGui::DockBuilderDockWindow("Validation",      bottom);
    ImGui::DockBuilderDockWindow("Execution Order", bottom);
    ImGui::DockBuilderDockWindow("Log",             bottom);
    ImGui::DockBuilderDockWindow("Canvas",          centre);
    ImGui::DockBuilderFinish(dockspace);
}

} // namespace

void draw_log(AppState& st) {
    if (ImGui::Begin("Log")) {
        if (ImGui::SmallButton("clear")) st.log.clear();
        ImGui::Separator();
        ImGui::BeginChild("log_lines", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
        for (auto const& line : st.log) ImGui::TextUnformatted(line.c_str());
        if (st.log_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f) ImGui::SetScrollHereY(1.0f);
        st.log_scroll = false;
        ImGui::EndChild();
    }
    ImGui::End();
}

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
    io.IniFilename = nullptr;            // window layout is rebuilt each launch
    io.FontGlobalScale = st.ui_scale;
    apply_theme(st.ui_scale);
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    ed::Config cfg;
    cfg.SettingsFile = nullptr;          // node positions live in the config's metadata
    st.editor = ed::CreateEditor(&cfg);

    st.logf("catalogue: " + std::to_string(st.catalogue.types.size()) + " module types from " + st.catalogue.palette_path);
    if (!st.config_path.empty()) st.logf("open: " + st.config_path + " (loading arrives with the graph model)");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) { ImGui_ImplGlfw_Sleep(10); continue; }
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiID dockspace = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
        if (!st.layout_built) { build_dock_layout(dockspace); st.layout_built = true; }

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Quit", "Ctrl+Q")) glfwSetWindowShouldClose(window, 1);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                if (ImGui::MenuItem("Reset layout")) st.layout_built = false;
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Q)) glfwSetWindowShouldClose(window, 1);

        draw_palette(st);
        draw_canvas(st);
        draw_log(st);
        if (ImGui::Begin("Properties")) ImGui::TextDisabled("select a node");
        ImGui::End();
        if (ImGui::Begin("Validation")) ImGui::TextDisabled("no graph");
        ImGui::End();
        if (ImGui::Begin("Execution Order")) ImGui::TextDisabled("no graph");
        ImGui::End();

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
