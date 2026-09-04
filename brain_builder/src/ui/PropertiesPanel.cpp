#include <algorithm>
#include <cstdio>

#include <imgui.h>
#include <imgui_stdlib.h>

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

std::string short_json(ojson const& v) {
    std::string s = v.dump();
    if (s.size() > 40) s = s.substr(0, 37) + "...";
    return s;
}

void param_tooltip(ParamInfo const& p) {
    if (!ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled)) return;
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 36);
    ImGui::TextUnformatted(p.key.c_str());
    ImGui::TextDisabled("%s · %s%s", kind_name(p.kind), p.hot ? "hot-mutable" : "construction-only", p.required ? " · REQUIRED" : "");
    if (!p.required) ImGui::TextDisabled("default %s", short_json(p.def).c_str());
    if (!p.min.is_null() || !p.max.is_null()) ImGui::TextDisabled("range [%s, %s]", p.min.is_null() ? "-" : p.min.dump().c_str(), p.max.is_null() ? "-" : p.max.dump().c_str());
    if (!p.description.empty()) { ImGui::Separator(); ImGui::TextUnformatted(p.description.c_str()); }
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

// One editable value.  Text fields keep a buffer per (module, key) so the
// document only changes when the edit is committed.
void value_widget(AppState& st, std::string const& id, ParamInfo const& p, ojson const& params,
                  std::vector<std::string> const& topics, bool socket) {
    std::string key = id + "\x1f" + p.key;
    bool set = params.contains(p.key) && !params[p.key].is_null();
    ojson cur = set ? params[p.key] : ojson(p.def);
    ImGui::PushID(p.key.c_str());
    ImGui::SetNextItemWidth(-1);

    ParamKind kind = p.kind;
    if (set) kind = kind_of_json(cur, kind);   // the document's type wins over the schema guess

    switch (kind) {
        case ParamKind::Bool: {
            bool v = cur.is_boolean() ? cur.get<bool>() : false;
            if (ImGui::Checkbox("##v", &v)) st.graph.set_param(id, p.key, v);
            break;
        }
        case ParamKind::Int: {
            long long v = cur.is_number() ? cur.get<long long>() : 0;
            if (ImGui::InputScalar("##v", ImGuiDataType_S64, &v, nullptr, nullptr, "%lld")) { st.pending_key = key; st.pending_num = double(v); }
            if (ImGui::IsItemDeactivatedAfterEdit() && st.pending_key == key) {
                long long out = (long long)st.pending_num;
                if (!p.min.is_null() && p.min.is_number()) out = std::max(out, p.min.get<long long>());
                if (!p.max.is_null() && p.max.is_number()) out = std::min(out, p.max.get<long long>());
                st.graph.set_param(id, p.key, out); st.pending_key.clear();
            }
            break;
        }
        case ParamKind::Float: {
            double v = cur.is_number() ? cur.get<double>() : 0.0;
            if (ImGui::InputDouble("##v", &v, 0.0, 0.0, "%.6g")) { st.pending_key = key; st.pending_num = v; }
            if (ImGui::IsItemDeactivatedAfterEdit() && st.pending_key == key) {
                double out = st.pending_num;
                if (!p.min.is_null() && p.min.is_number()) out = std::max(out, p.min.get<double>());
                if (!p.max.is_null() && p.max.is_number()) out = std::min(out, p.max.get<double>());
                st.graph.set_param(id, p.key, out); st.pending_key.clear();
            }
            break;
        }
        case ParamKind::String: {
            std::string v = cur.is_string() ? cur.get<std::string>() : "";
            if (!p.enum_values.empty()) {
                if (ImGui::BeginCombo("##v", v.c_str())) {
                    for (auto const& e : p.enum_values)
                        if (ImGui::Selectable(e.c_str(), e == v)) st.graph.set_param(id, p.key, e);
                    ImGui::EndCombo();
                }
                break;
            }
            auto& buf = st.text_bufs[key];
            if (!ImGui::IsAnyItemActive() || buf.empty()) buf = v;
            float bw = socket ? ImGui::GetContentRegionAvail().x - ImGui::GetFrameHeight() - 4 : -1.0f;
            ImGui::SetNextItemWidth(bw);
            ImGui::InputText("##v", &buf);
            if (ImGui::IsItemDeactivatedAfterEdit() && buf != v) st.graph.set_param(id, p.key, buf);
            if (socket) {
                ImGui::SameLine(0, 4);
                if (ImGui::BeginCombo("##topics", "", ImGuiComboFlags_NoPreview)) {
                    for (auto const& t : topics)
                        if (ImGui::Selectable(t.c_str(), t == v)) { st.graph.set_param(id, p.key, t); buf = t; }
                    ImGui::EndCombo();
                }
            }
            break;
        }
        case ParamKind::ListFloat:
        case ParamKind::ListString: {
            ojson lst = cur.is_array() ? cur : ojson::array();
            bool strings = kind == ParamKind::ListString;
            ImGui::TextDisabled("[%zu]", lst.size());
            ImGui::SameLine();
            if (ImGui::SmallButton("+")) { lst.push_back(strings ? ojson("") : ojson(0.0)); st.graph.set_param(id, p.key, lst); }
            for (size_t i = 0; i < lst.size(); ++i) {
                ImGui::PushID(int(i));
                std::string ekey = key + "[" + std::to_string(i) + "]";
                if (strings) {
                    std::string v = lst[i].is_string() ? lst[i].get<std::string>() : "";
                    auto& buf = st.text_bufs[ekey];
                    if (!ImGui::IsAnyItemActive() || buf.empty()) buf = v;
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 2 * ImGui::GetFrameHeight() - 8);
                    ImGui::InputText("##e", &buf);
                    if (ImGui::IsItemDeactivatedAfterEdit() && buf != v) { ojson l2 = lst; l2[i] = buf; st.graph.set_param(id, p.key, l2); }
                    if (socket) {
                        ImGui::SameLine(0, 4);
                        if (ImGui::BeginCombo("##t", "", ImGuiComboFlags_NoPreview)) {
                            for (auto const& t : topics)
                                if (ImGui::Selectable(t.c_str(), t == v)) { ojson l2 = lst; l2[i] = t; st.graph.set_param(id, p.key, l2); buf = t; }
                            ImGui::EndCombo();
                        }
                    }
                } else {
                    double v = lst[i].is_number() ? lst[i].get<double>() : 0.0;
                    bool integral = lst[i].is_number_integer();
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - ImGui::GetFrameHeight() - 8);
                    if (ImGui::InputDouble("##e", &v, 0.0, 0.0, integral ? "%.0f" : "%.6g")) { st.pending_key = ekey; st.pending_num = v; }
                    if (ImGui::IsItemDeactivatedAfterEdit() && st.pending_key == ekey) {
                        ojson l2 = lst;
                        if (integral && st.pending_num == std::floor(st.pending_num)) l2[i] = (long long)st.pending_num; else l2[i] = st.pending_num;
                        st.graph.set_param(id, p.key, l2); st.pending_key.clear();
                    }
                }
                ImGui::SameLine(0, 4);
                if (ImGui::SmallButton("x")) { ojson l2 = lst; l2.erase(l2.begin() + long(i)); st.graph.set_param(id, p.key, l2); ImGui::PopID(); break; }
                ImGui::PopID();
            }
            break;
        }
    }
    ImGui::PopID();
}

} // namespace

void draw_properties(AppState& st) {
    if (!ImGui::Begin("Properties")) { ImGui::End(); return; }
    int idx = st.selected.empty() ? -1 : st.graph.index_of(st.selected);
    if (idx < 0) {
        ImGui::TextDisabled("select a module on the canvas");
        if (st.body) {
            ImGui::Separator();
            ImGui::TextUnformatted(st.body->title.c_str());
            ImGui::PushTextWrapPos();
            ImGui::TextDisabled("%s", st.body->description.c_str());
            if (!st.body->launch_hint.empty()) ImGui::TextDisabled("run: %s", st.body->launch_hint.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::End();
        return;
    }
    std::string id = st.selected;
    std::string type = st.graph.type_of(size_t(idx));
    TypeInfo const* ti = st.catalogue.find(type);
    Node const* node = st.wiring.node_named(id);
    ojson const& params = st.graph.params(size_t(idx));

    // header
    {
        auto& buf = st.text_bufs["@id"];
        if (!ImGui::IsAnyItemActive() || buf.empty()) buf = id;
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##id", &buf, ImGuiInputTextFlags_EnterReturnsTrue);
        if (ImGui::IsItemDeactivatedAfterEdit() && buf != id) {
            if (st.graph.rename_module(id, buf)) { st.selected = buf; st.wiring_dirty = true; st.logf("renamed " + id + " → " + buf); }
            else st.logf("rename refused: '" + buf + "' is empty or taken");
        }
    }
    ImGui::PushStyleColor(ImGuiCol_Text, category_color(ti ? ti->category : "other"));
    ImGui::TextUnformatted(type.c_str());
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::TextDisabled("#%d of %zu", idx, st.graph.size());
    if (ti && !ti->purpose.empty()) { ImGui::PushTextWrapPos(); ImGui::TextDisabled("%s", ti->purpose.c_str()); ImGui::PopTextWrapPos(); }
    if (ti && ti->deprecated) ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "deprecated: %s", ti->deprecated_note.c_str());
    if (node) {
        if (node->setup_ok) ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "setup OK · %zu in · %zu out", node->inputs.size(), node->outputs.size());
        else { ImGui::PushTextWrapPos(); ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.45f, 1.0f), "setup failed: %s", node->setup_error.c_str()); ImGui::PopTextWrapPos(); }
    }
    if (ImGui::SmallButton("remove")) { st.graph.remove_module(id); st.selected.clear(); st.wiring_dirty = true; ImGui::End(); return; }
    ImGui::SameLine();
    if (ImGui::SmallButton("move up") && st.graph.move_module(idx, idx - 1)) st.wiring_dirty = true;
    ImGui::SameLine();
    if (ImGui::SmallButton("move down") && st.graph.move_module(idx, idx + 1)) st.wiring_dirty = true;
    ImGui::Separator();

    if (!ti) { ImGui::TextDisabled("unknown type: params shown raw"); ImGui::TextWrapped("%s", params.dump(2).c_str()); ImGui::End(); return; }

    // filters
    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##filter", "filter params", &st.prop_filter);
    ImGui::Checkbox("required", &st.prop_required_only); ImGui::SameLine();
    ImGui::Checkbox("set", &st.prop_set_only); ImGui::SameLine();
    ImGui::Checkbox("sockets", &st.prop_sockets_only); ImGui::SameLine();
    ImGui::Checkbox("hot", &st.prop_hot_only);

    std::vector<std::string> topics = st.wiring.known_topics();
    std::vector<std::string> socket_params;
    for (auto const& s : ti->sockets) for (auto const& p : s.params) socket_params.push_back(p);
    auto is_socket = [&](std::string const& k) { return std::find(socket_params.begin(), socket_params.end(), k) != socket_params.end(); };

    std::vector<ParamInfo const*> rows;
    for (auto const& p : ti->params) {
        bool set = params.contains(p.key) && !params[p.key].is_null();
        if (st.prop_required_only && !p.required) continue;
        if (st.prop_set_only && !set) continue;
        if (st.prop_sockets_only && !is_socket(p.key)) continue;
        if (st.prop_hot_only && !p.hot) continue;
        if (!icontains(p.key, st.prop_filter) && !icontains(p.description, st.prop_filter)) continue;
        rows.push_back(&p);
    }
    std::stable_sort(rows.begin(), rows.end(), [&](ParamInfo const* a, ParamInfo const* b) {
        bool sa = is_socket(a->key), sb = is_socket(b->key);
        if (sa != sb) return sa;
        if (a->required != b->required) return a->required;
        return false;
    });

    // params the document has that the schema does not know
    std::vector<std::string> unknown;
    for (auto const& [k, v] : params.items()) if (!ti->param(k) && k != "_comment") unknown.push_back(k);

    if (ImGui::BeginTable("params", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("param", ImGuiTableColumnFlags_WidthStretch, 0.42f);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 0.58f);
        ImGui::TableSetupColumn("##x", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight() * 1.2f);
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableHeadersRow();
        for (auto const* p : rows) {
            bool set = params.contains(p->key) && !params[p->key].is_null();
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImVec4 col = ImGui::GetStyleColorVec4(set ? ImGuiCol_Text : ImGuiCol_TextDisabled);
            if (p->required && !set) col = ImVec4(1.0f, 0.45f, 0.4f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, col);
            ImGui::TextUnformatted(p->key.c_str());
            ImGui::PopStyleColor();
            param_tooltip(*p);
            if (p->hot) { ImGui::SameLine(); ImGui::TextDisabled("hot"); }
            ImGui::TableSetColumnIndex(1);
            value_widget(st, id, *p, params, topics, is_socket(p->key));
            ImGui::TableSetColumnIndex(2);
            if (set) {
                ImGui::PushID(p->key.c_str());
                if (ImGui::SmallButton("x")) st.graph.erase_param(id, p->key);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("reset to default (remove the key)");
                ImGui::PopID();
            }
        }
        for (auto const& k : unknown) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "%s", k.c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("not in this type's schema");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextDisabled("%s", short_json(params[k]).c_str());
            ImGui::TableSetColumnIndex(2);
            ImGui::PushID(k.c_str());
            if (ImGui::SmallButton("x")) st.graph.erase_param(id, k);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void draw_order(AppState& st) {
    if (!ImGui::Begin("Execution Order")) { ImGui::End(); return; }
    ImGui::TextDisabled("modules tick in this order, one level (the scheduler runs the array top to bottom)");
    int move_from = -1, move_to = -1;
    if (ImGui::BeginTable("order", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
        ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 2.5f);
        ImGui::TableSetupColumn("id", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        for (size_t i = 0; i < st.graph.size(); ++i) {
            std::string id = st.graph.id_of(i);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%zu", i);
            ImGui::TableSetColumnIndex(1);
            ImGui::PushID(int(i));
            if (ImGui::Selectable(id.c_str(), st.selected == id, ImGuiSelectableFlags_SpanAllColumns)) st.selected = id;
            if (ImGui::BeginDragDropSource()) { int from = int(i); ImGui::SetDragDropPayload("BB_ORDER", &from, sizeof from); ImGui::TextUnformatted(id.c_str()); ImGui::EndDragDropSource(); }
            if (ImGui::BeginDragDropTarget()) {
                if (ImGuiPayload const* p = ImGui::AcceptDragDropPayload("BB_ORDER")) { move_from = *static_cast<int const*>(p->Data); move_to = int(i); }
                ImGui::EndDragDropTarget();
            }
            ImGui::PopID();
            ImGui::TableSetColumnIndex(2);
            ImGui::TextDisabled("%s", st.graph.type_of(i).c_str());
        }
        ImGui::EndTable();
    }
    if (move_from >= 0 && st.graph.move_module(move_from, move_to)) st.wiring_dirty = true;
    ImGui::Separator();
    if (ImGui::Button("Sort topologically")) { st.order_suggestion = topological_order(st.graph, st.wiring); st.order_preview = true; }
    if (st.order_preview) {
        auto const& sg = st.order_suggestion;
        for (auto const& n : sg.notes) ImGui::TextDisabled("%s", n.c_str());
        if (sg.changed) {
            ImGui::TextUnformatted("proposed order:");
            for (size_t i = 0; i < sg.order.size(); ++i) {
                int from = sg.order[i];
                bool moved = from != int(i);
                if (moved) ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "  %zu  %s   (was #%d)", i, st.graph.id_of(size_t(from)).c_str(), from);
                else       ImGui::TextDisabled("  %zu  %s", i, st.graph.id_of(size_t(from)).c_str());
            }
            if (ImGui::Button("Apply")) { if (st.graph.reorder(sg.order)) { st.wiring_dirty = true; st.logf("execution order sorted topologically"); } st.order_preview = false; }
            ImGui::SameLine();
        }
        if (ImGui::Button("Close")) st.order_preview = false;
    }
    ImGui::End();
}

void draw_validation(AppState& st) {
    if (!ImGui::Begin("Validation")) { ImGui::End(); return; }
    ImGui::Text("%d errors, %d warnings", st.wiring.errors, st.wiring.warnings);
    ImGui::SameLine(0, 30);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
    ImGui::InputInt("ticks", &st.dry_ticks);
    if (st.dry_ticks < 1) st.dry_ticks = 1;
    ImGui::SameLine();
    ImGui::BeginDisabled(st.dry_job != nullptr);
    if (ImGui::Button(st.dry_job ? "running..." : "Dry run")) st.start_dry_run();
    ImGui::EndDisabled();
    if (st.dry_has_report) {
        auto const& r = st.dry_report;
        ImGui::SameLine(0, 20);
        if (!r.constructed) ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", r.error.c_str());
        else if (!r.error.empty()) ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%d ticks then: %s", r.ticks_done, r.error.c_str());
        else ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "%d ticks OK  (construct %.0f ms, %.2f ms/tick)  sinks driven %zu/%zu, silent outputs %zu",
                                r.ticks_done, r.construct_ms, r.ticks_done ? r.tick_ms / r.ticks_done : 0.0,
                                r.actions_seen.size(), r.actions_seen.size() + r.actions_missing.size(), r.silent.size());
        if (r.constructed && ImGui::TreeNode("dry run detail")) {
            for (auto const& f : r.fed) ImGui::TextDisabled("fed      %s", f.c_str());
            for (auto const& a : r.actions_missing) ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "no value  %s (body sink)", a.c_str());
            for (auto const& t : r.silent) ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "silent    %s", t.c_str());
            for (auto const& t : r.published) ImGui::TextDisabled("published %s", t.c_str());
            ImGui::TreePop();
        }
    }
    if (ImGui::BeginTable("diag", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("severity", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 5.0f);
        ImGui::TableSetupColumn("node", ImGuiTableColumnFlags_WidthStretch, 0.25f);
        ImGui::TableSetupColumn("message", ImGuiTableColumnFlags_WidthStretch, 0.75f);
        ImGui::TableHeadersRow();
        for (auto const& d : st.wiring.diagnostics) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImVec4 c = d.severity == Diagnostic::Error ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f)
                     : d.severity == Diagnostic::Warning ? ImVec4(0.95f, 0.75f, 0.3f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
            ImGui::TextColored(c, "%s", d.severity == Diagnostic::Error ? "error" : d.severity == Diagnostic::Warning ? "warning" : "info");
            ImGui::TableSetColumnIndex(1);
            if (ImGui::Selectable(d.node.c_str(), st.selected == d.node)) if (st.graph.index_of(d.node) >= 0) st.selected = d.node;
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted(d.message.c_str());
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace bb
