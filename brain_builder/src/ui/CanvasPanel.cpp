#include <imgui.h>
#include <imgui_node_editor.h>

#include "ui/App.hpp"

namespace ed = ax::NodeEditor;

namespace bb {

void draw_canvas(AppState& st) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    bool open = ImGui::Begin("Canvas");
    ImGui::PopStyleVar();
    if (open) {
        ed::SetCurrentEditor(st.editor);
        ed::Begin("canvas", ImVec2(0, 0));
        ed::End();
        ed::SetCurrentEditor(nullptr);
        if (ImGui::BeginDragDropTarget()) {
            if (ImGuiPayload const* p = ImGui::AcceptDragDropPayload("BB_MODULE_TYPE"))
                st.logf(std::string("drop: ") + static_cast<char const*>(p->Data) + " (node creation arrives with the graph model)");
            ImGui::EndDragDropTarget();
        }
    }
    ImGui::End();
}

} // namespace bb
