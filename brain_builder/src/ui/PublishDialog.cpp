#include <imgui.h>
#include <imgui_stdlib.h>

#include "DryRun.hpp"
#include "Publish.hpp"
#include "ui/App.hpp"

namespace bb {

void draw_publish(AppState& st) {
    if (st.show_publish) {
        ImGui::OpenPopup("Publish");
        st.show_publish = false;
        if (st.pub_title.empty()) {
            std::string n = st.graph.metadata().value("name", "");
            size_t dot = n.find("\xC2\xB7");
            st.pub_title = dot == std::string::npos ? n : n.substr(dot + 2);
            while (!st.pub_title.empty() && st.pub_title.front() == ' ') st.pub_title.erase(0, 1);
        }
        if (st.pub_target.empty()) st.pub_target = (st.body && st.body->host == "godot") ? "godot" : "mj_host";
    }
    ImGui::SetNextWindowSize(ImVec2(760 * st.ui_scale, 0), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal("Publish", nullptr, ImGuiWindowFlags_NoSavedSettings)) return;

    ImGui::TextUnformatted("target");
    ImGui::SameLine();
    if (ImGui::RadioButton("mj_host/configs (duck launcher series)", st.pub_target == "mj_host")) st.pub_target = "mj_host";
    ImGui::SameLine();
    if (ImGui::RadioButton("godot_host configs", st.pub_target == "godot")) st.pub_target = "godot";
    ImGui::InputText("title", &st.pub_title);
    ImGui::InputText("slug", &st.pub_slug);
    ImGui::InputTextMultiline("why / description", &st.pub_why, ImVec2(-1, ImGui::GetTextLineHeight() * 4));
    ImGui::InputText("phase tag", &st.pub_phase);

    PublishPlan plan = plan_publish(st.graph, st.body, st.pub_target, st.pub_slug, st.pub_title, st.pub_why, st.pub_phase);
    ImGui::Separator();
    ImGui::TextDisabled("file   %s", plan.path.c_str());
    ImGui::TextDisabled("name   %s", plan.name.c_str());
    if (plan.target == "mj_host") ImGui::TextDisabled("rank   %d   env_target %s", plan.rank, plan.env_target.c_str());
    else ImGui::TextDisabled("env_target %s   phase_tag %s", plan.env_target.c_str(), plan.phase_tag.c_str());
    for (auto const& n : plan.notes) ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "%s", n.c_str());

    bool blocked = st.wiring.errors > 0 || st.pub_title.empty();
    if (st.wiring.errors > 0) ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%d validation errors: fix them first", st.wiring.errors);
    else if (st.wiring.warnings > 0) ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "%d warnings (see Validation)", st.wiring.warnings);
    ImGui::Checkbox("dry run 30 ticks before writing", &st.pub_dry_first);

    ImGui::Separator();
    ImGui::BeginDisabled(blocked);
    if (ImGui::Button("Publish")) {
        bool ok = true;
        if (st.pub_dry_first) {
            DryRunReport r = dry_run(st.graph, st.wiring, st.body, 30);
            if (!r.constructed || !r.error.empty()) { st.logf("publish refused: dry run failed: " + r.error); ok = false; }
            else if (!r.actions_missing.empty()) st.logf("publish: note " + std::to_string(r.actions_missing.size()) + " body sinks received no value in the dry run");
        }
        if (ok) {
            try {
                apply_publish(st.graph, plan);
                st.config_path = plan.path;
                st.logf("published " + plan.path + "  (" + plan.name + ")");
                for (auto const& n : plan.notes) st.logf("publish: " + n);
                ImGui::CloseCurrentPopup();
            } catch (std::exception const& e) { st.logf(std::string("publish failed: ") + e.what()); }
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

} // namespace bb
