#include <algorithm>
#include <cmath>

#include <imgui.h>
#include <imgui_node_editor.h>

#include "ui/App.hpp"
#include "ui/Theme.hpp"

namespace ed = ax::NodeEditor;

namespace bb {

namespace {

// A pin's icon: filled circle for a live topic, hollow for a placeholder, a
// red ring when required and unresolved, a square for a prefix subscription.
void pin_icon(Pin const& p, bool linked, float scale) {
    float r = 5.0f * scale;
    ImVec2 c = ImGui::GetCursorScreenPos();
    c.x += r; c.y += ImGui::GetTextLineHeight() * 0.5f;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImU32 col = payload_color(p.payload);
    if (p.prefix) {
        if (p.placeholder) dl->AddRect(ImVec2(c.x - r, c.y - r), ImVec2(c.x + r, c.y + r), col, 1.0f, 0, 1.5f * scale);
        else dl->AddRectFilled(ImVec2(c.x - r, c.y - r), ImVec2(c.x + r, c.y + r), col, 1.0f);
    } else if (p.placeholder) {
        dl->AddCircle(c, r, col, 12, 1.5f * scale);
    } else {
        dl->AddCircleFilled(c, r, col, 12);
    }
    if (p.required && !linked && !p.feedback && !p.output)
        dl->AddCircle(c, r + 2.5f * scale, IM_COL32(230, 70, 70, 255), 12, 1.5f * scale);
    if (p.polled)
        dl->AddCircle(c, r + 2.5f * scale, IM_COL32(200, 200, 200, 120), 12, 1.0f * scale);
    if (p.feedback)
        dl->AddText(ImVec2(c.x - r * 0.45f, c.y - ImGui::GetTextLineHeight() * 0.5f), IM_COL32(255, 255, 255, 220), "F");
    ImGui::Dummy(ImVec2(2 * r, ImGui::GetTextLineHeight()));
    ed::PinPivotRect(ImVec2(c.x - r, c.y - r), ImVec2(c.x + r, c.y + r));
}

void pin_tooltip(Pin const& p) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted(p.placeholder ? ("unset: " + p.label).c_str() : p.topic.c_str());
    if (p.plus) { ImGui::TextDisabled("drop a producer here to set an optional input"); ImGui::EndTooltip(); return; }
    ImGui::TextDisabled("%s%s%s%s%s", p.payload.c_str(), p.feedback ? " · feedback" : "", p.prefix ? " · prefix" : "", p.required ? "" : " · optional", p.polled ? " · polled (not a declared port)" : "");
    if (!p.param.empty()) ImGui::TextDisabled("param: %s", p.param.c_str());
    if (p.dims) ImGui::TextDisabled("dims: %d", p.dims);
    if (!p.description.empty()) { ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32); ImGui::TextUnformatted(p.description.c_str()); ImGui::PopTextWrapPos(); }
    ImGui::EndTooltip();
}

void draw_node(AppState& st, Node const& n, std::vector<uint64_t> const& linked_pins) {
    float scale = st.ui_scale;
    auto is_linked = [&](uint64_t id) { return std::binary_search(linked_pins.begin(), linked_pins.end(), id); };
    ImU32 tint = n.kind == NodeKind::Module ? category_color(n.category) : IM_COL32(60, 110, 130, 255);
    if (!n.setup_ok) tint = IM_COL32(170, 50, 50, 255);
    ed::PushStyleColor(ed::StyleColor_NodeBorder, ImGui::ColorConvertU32ToFloat4(tint));
    ed::PushStyleVar(ed::StyleVar_NodeBorderWidth, (n.name == st.selected ? 3.0f : 1.5f) * scale);
    ed::BeginNode(n.id);

    // header
    ImGui::PushStyleColor(ImGuiCol_Text, tint);
    ImGui::TextUnformatted(n.kind == NodeKind::Module ? n.name.c_str()
                           : n.kind == NodeKind::Sources ? "BODY · sources"
                           : n.kind == NodeKind::Sinks ? "BODY · sinks" : "BODY · events");
    ImGui::PopStyleColor();
    ImGui::TextDisabled("%s%s", n.type.c_str(), n.kind == NodeKind::Module ? ("  #" + std::to_string(n.index)).c_str() : "");
    if (!n.setup_ok) ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.45f, 1.0f), "setup failed");
    ImGui::Dummy(ImVec2(0, 4 * scale));

    float out_w = 0;
    for (auto const& p : n.outputs) out_w = std::max(out_w, ImGui::CalcTextSize(p.label.c_str()).x);
    ImGui::BeginGroup();
    for (auto const& p : n.inputs) {
        ed::BeginPin(p.id, ed::PinKind::Input);
        pin_icon(p, is_linked(p.id), scale);
        ImGui::SameLine();
        if (p.placeholder) ImGui::TextDisabled("%s", p.label.c_str()); else ImGui::TextUnformatted(p.label.c_str());
        ed::EndPin();
    }
    if (n.inputs.empty()) ImGui::Dummy(ImVec2(10 * scale, 1));
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::Dummy(ImVec2(24 * scale, 1));
    ImGui::SameLine();
    ImGui::BeginGroup();
    for (auto const& p : n.outputs) {
        float w = ImGui::CalcTextSize(p.label.c_str()).x;
        ImGui::Dummy(ImVec2(out_w - w, 1));
        ImGui::SameLine(0, 0);
        if (p.placeholder) ImGui::TextDisabled("%s", p.label.c_str()); else ImGui::TextUnformatted(p.label.c_str());
        ImGui::SameLine();
        ed::BeginPin(p.id, ed::PinKind::Output);
        pin_icon(p, is_linked(p.id), scale);
        ed::EndPin();
    }
    ImGui::EndGroup();

    ed::EndNode();
    ed::PopStyleVar();
    ed::PopStyleColor();
}

} // namespace

void draw_canvas(AppState& st) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    bool open = ImGui::Begin("Canvas");
    ImGui::PopStyleVar();
    if (!open) { ImGui::End(); return; }

    Wiring& w = st.wiring;
    std::vector<uint64_t> linked;
    for (auto const& l : w.links) { linked.push_back(l.from_pin); linked.push_back(l.to_pin); }
    std::sort(linked.begin(), linked.end());

    ed::SetCurrentEditor(st.editor);
    ed::Begin("canvas", ImVec2(0, 0));

    if (st.layout_apply) {
        for (auto const& n : w.nodes) {
            auto it = st.layout.find(n.name);
            if (it != st.layout.end()) ed::SetNodePosition(n.id, ImVec2(it->second.x, it->second.y));
        }
        st.layout_apply = false;
    }

    for (auto const& n : w.nodes) draw_node(st, n, linked);
    for (auto const& n : w.nodes) { ImVec2 sz = ed::GetNodeSize(n.id); if (sz.x > 1) st.node_sizes[n.name] = {sz.x, sz.y}; }
    for (auto const& l : w.links) {
        ImVec4 col = ImGui::ColorConvertU32ToFloat4(topic_color(l.topic));
        if (l.feedback) col.w = 0.55f;
        if (!l.type_ok) col = ImVec4(1.0f, 0.6f, 0.2f, 1.0f);
        float thick = (l.topic.rfind("action.", 0) == 0 ? 2.5f : 1.5f) * st.ui_scale;
        ed::Link(l.id, l.from_pin, l.to_pin, col, thick);
    }

    // drag-connect
    if (ed::BeginCreate(ImVec4(1, 1, 1, 1), 2.0f * st.ui_scale)) {
        ed::PinId a, b;
        if (ed::QueryNewLink(&a, &b)) {
            Pin const* pa = w.pin(uint64_t(a.Get()));
            Pin const* pb = w.pin(uint64_t(b.Get()));
            bool ok = pa && pb && pa->output != pb->output && w.owner(pa->id) != w.owner(pb->id);
            if (!ok) ed::RejectNewItem(ImVec4(1, 0.3f, 0.3f, 1), 2.0f * st.ui_scale);
            else if (ed::AcceptNewItem()) {
                std::string err = w.connect(st.graph, st.catalogue, pa->id, pb->id);
                if (err == "choose") {
                    Pin const* src = pa->output ? pa : pb;
                    Pin const* dst = pa->output ? pb : pa;
                    st.plus_node = dst->node; st.plus_topic = src->topic; st.plus_payload = src->payload; st.plus_open = true;
                    err.clear();
                }
                if (!err.empty()) st.logf("connect: " + err);
                else { st.logf("connected " + (pa->output ? pa->node : pb->node) + " → " + (pa->output ? pb->node : pa->node)); st.wiring_dirty = true; }
            }
        }
    }
    ed::EndCreate();

    // delete links (= clear the param) and nodes
    if (ed::BeginDelete()) {
        ed::LinkId lid;
        while (ed::QueryDeletedLink(&lid)) {
            Link const* l = nullptr;
            for (auto const& x : w.links) if (x.id == uint64_t(lid.Get())) l = &x;
            if (l && ed::AcceptDeletedItem()) {
                std::string err = w.disconnect(st.graph, st.catalogue, *l);
                if (!err.empty()) st.logf("disconnect: " + err); else st.wiring_dirty = true;
            } else ed::RejectDeletedItem();
        }
        ed::NodeId nid;
        while (ed::QueryDeletedNode(&nid)) {
            Node const* n = w.node(uint64_t(nid.Get()));
            if (n && n->kind == NodeKind::Module && ed::AcceptDeletedItem(false)) {
                st.logf("removed " + n->name);
                if (st.selected == n->name) st.selected.clear();
                st.graph.remove_module(n->name);
                st.wiring_dirty = true;
            } else ed::RejectDeletedItem();
        }
    }
    ed::EndDelete();

    // selection → Properties
    if (ed::HasSelectionChanged()) {
        ed::NodeId ids[4];
        int c = ed::GetSelectedNodes(ids, 4);
        if (c >= 1) { Node const* n = w.node(uint64_t(ids[0].Get())); st.selected = (n && n->kind == NodeKind::Module) ? n->name : ""; }
        else st.selected.clear();
    }

    // positions back into the layout (moves are not undo steps)
    for (auto const& n : w.nodes) {
        ImVec2 p = ed::GetNodePosition(n.id);
        auto& q = st.layout[n.name];
        if (std::fabs(p.x - q.x) > 0.5f || std::fabs(p.y - q.y) > 0.5f) { q = {p.x, p.y}; st.graph.dirty = true; }
    }

    ImVec2 mouse_canvas = ed::ScreenToCanvas(ImGui::GetMousePos());
    st.canvas_mouse_x = mouse_canvas.x; st.canvas_mouse_y = mouse_canvas.y;

    // context menus and tooltips
    ed::Suspend();
    static ed::NodeId ctx_node;
    static ImVec2 ctx_pos;
    if (ed::ShowBackgroundContextMenu()) { ImGui::OpenPopup("canvas_bg"); ctx_pos = mouse_canvas; }
    if (ed::ShowNodeContextMenu(&ctx_node)) ImGui::OpenPopup("canvas_node");
    if (ImGui::BeginPopup("canvas_bg")) {
        ImGui::TextDisabled("add module");
        ImGui::Separator();
        for (auto const& cat : st.catalogue.categories()) {
            if (!ImGui::BeginMenu(cat.c_str())) continue;
            for (auto const& t : st.catalogue.types)
                if (t.category == cat && ImGui::MenuItem(t.type.c_str())) st.add_module_at(t.type, ctx_pos.x, ctx_pos.y);
            ImGui::EndMenu();
        }
        ImGui::EndPopup();
    }
    if (st.plus_open) { ImGui::OpenPopup("canvas_plus"); st.plus_open = false; }
    if (ImGui::BeginPopup("canvas_plus")) {
        Node const* n = w.node_named(st.plus_node);
        ImGui::TextDisabled("%s ← %s", st.plus_node.c_str(), st.plus_topic.c_str());
        ImGui::Separator();
        if (n) {
            auto socks = st.wiring.optional_unset_inputs(st.graph, st.catalogue, *n);
            std::stable_sort(socks.begin(), socks.end(), [&](SocketInfo const& a, SocketInfo const& b) {
                bool ma = a.payload == st.plus_payload, mb = b.payload == st.plus_payload;
                return ma && !mb;
            });
            for (auto const& s : socks) {
                std::string label = s.pattern + "  (" + s.payload + (s.polled ? ", polled" : "") + ")";
                bool match = s.payload == st.plus_payload || s.payload == "Unknown";
                if (!match) ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                if (ImGui::MenuItem(label.c_str())) {
                    if (s.list) {
                        ojson lst = ojson::array();
                        ojson const& params = st.graph.params(size_t(n->index));
                        if (params.contains(s.params[0]) && params[s.params[0]].is_array()) lst = params[s.params[0]];
                        lst.push_back(st.plus_topic);
                        st.graph.set_param(n->name, s.params[0], lst);
                    } else st.graph.set_param(n->name, s.params[0], st.plus_topic);
                    st.wiring_dirty = true;
                    st.logf("connected " + st.plus_topic + " → " + n->name + "." + s.params[0]);
                }
                if (!match) ImGui::PopStyleColor();
            }
            if (socks.empty()) ImGui::TextDisabled("no optional inputs left");
        }
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("canvas_node")) {
        Node const* n = w.node(uint64_t(ctx_node.Get()));
        if (n && n->kind == NodeKind::Module) {
            ImGui::TextDisabled("%s", n->name.c_str());
            if (ImGui::MenuItem("Remove")) { st.graph.remove_module(n->name); if (st.selected == n->name) st.selected.clear(); st.wiring_dirty = true; }
            if (ImGui::MenuItem("Duplicate")) {
                ojson params = st.graph.params(size_t(n->index));
                std::string id = st.graph.add_module(n->type, n->name, params, n->index + 1);
                st.layout[id] = {st.layout[n->name].x + 40, st.layout[n->name].y + 40};
                st.layout_apply = true; st.selected = id; st.wiring_dirty = true;
            }
        } else ImGui::TextDisabled("the body's node");
        ImGui::EndPopup();
    }
    {
        ed::PinId hp = ed::GetHoveredPin();
        ed::NodeId hn = ed::GetHoveredNode();
        if (hp) { if (Pin const* p = w.pin(uint64_t(hp.Get()))) pin_tooltip(*p); }
        else if (hn) {
            Node const* n = w.node(uint64_t(hn.Get()));
            if (n && !n->setup_ok) { ImGui::BeginTooltip(); ImGui::PushTextWrapPos(ImGui::GetFontSize() * 36); ImGui::TextUnformatted(n->setup_error.c_str()); ImGui::PopTextWrapPos(); ImGui::EndTooltip(); }
        }
    }
    ed::Resume();

    if (st.fit_frames > 0 && --st.fit_frames == 0) ed::NavigateToContent(0.0f);
    ed::End();
    ed::SetCurrentEditor(nullptr);

    if (ImGui::BeginDragDropTarget()) {
        if (ImGuiPayload const* p = ImGui::AcceptDragDropPayload("BB_MODULE_TYPE"))
            st.add_module_at(static_cast<char const*>(p->Data), st.canvas_mouse_x, st.canvas_mouse_y);
        ImGui::EndDragDropTarget();
    }
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && !ImGui::GetIO().WantTextInput && ImGui::IsKeyPressed(ImGuiKey_F, false)) st.fit_frames = 1;
    ImGui::End();
}

} // namespace bb
