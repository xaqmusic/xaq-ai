#include <algorithm>
#include <cctype>

#include <imgui.h>

#include "ui/App.hpp"
#include "ui/Theme.hpp"

namespace bb {

namespace {
bool icontains(std::string const& hay, std::string const& needle) {
    if (needle.empty()) return true;
    auto it = std::search(hay.begin(), hay.end(), needle.begin(), needle.end(),
        [](char a, char b) { return std::tolower((unsigned char)a) == std::tolower((unsigned char)b); });
    return it != hay.end();
}
}

void draw_palette(AppState& st) {
    if (ImGui::Begin("Palette")) {
        char buf[128];
        std::snprintf(buf, sizeof buf, "%s", st.palette_search.c_str());
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputTextWithHint("##search", "search modules", buf, sizeof buf)) st.palette_search = buf;
        ImGui::Separator();
        ImGui::BeginChild("palette_list");
        for (auto const& cat : st.catalogue.categories()) {
            std::vector<TypeInfo const*> items;
            for (auto const& t : st.catalogue.types)
                if (t.category == cat && (icontains(t.type, st.palette_search) || icontains(t.purpose, st.palette_search)))
                    items.push_back(&t);
            if (items.empty()) continue;
            ImGui::PushStyleColor(ImGuiCol_Header, category_color(cat));
            bool open = ImGui::CollapsingHeader(cat.c_str(), ImGuiTreeNodeFlags_DefaultOpen);
            ImGui::PopStyleColor();
            if (!open) continue;
            for (auto const* t : items) {
                if (t->deprecated) ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                bool selected = st.palette_selected == t;
                if (ImGui::Selectable(t->type.c_str(), selected)) st.palette_selected = t;
                if (t->deprecated) ImGui::PopStyleColor();
                if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
                    ImGui::SetDragDropPayload("BB_MODULE_TYPE", t->type.c_str(), t->type.size() + 1);
                    ImGui::TextUnformatted(t->type.c_str());
                    ImGui::EndDragDropSource();
                }
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip)) {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
                    ImGui::TextUnformatted(t->purpose.empty() ? "(no purpose recorded)" : t->purpose.c_str());
                    if (t->deprecated) ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "deprecated: %s", t->deprecated_note.c_str());
                    ImGui::TextDisabled("%zu params, %zu sockets, %zu fixed topics", t->params.size(), t->sockets.size(), t->fixed.size());
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
            }
        }
        ImGui::EndChild();
    }
    ImGui::End();
}

} // namespace bb
