#include "Layout.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace bb {

LayoutMap auto_layout(Wiring const& w, Catalogue const& cat, float scale, SizeMap const* sizes) {
    LayoutMap out;
    // Depth: 0 for a module fed only by the body (or nothing); else one more
    // than its deepest module producer, over links that run forward in array
    // order (a link from a later module is a feedback for layout purposes).
    std::map<std::string, int> depth, index;
    std::vector<Node const*> modules;
    for (auto const& n : w.nodes) if (n.kind == NodeKind::Module) { modules.push_back(&n); index[n.name] = n.index; depth[n.name] = 0; }
    for (int pass = 0; pass < 64; ++pass) {
        bool changed = false;
        for (auto const& l : w.links) {
            if (l.feedback) continue;
            if (!index.count(l.from_node) || !index.count(l.to_node)) continue;
            if (index[l.from_node] > index[l.to_node]) continue;
            int d = depth[l.from_node] + 1;
            if (d > depth[l.to_node]) { depth[l.to_node] = d; changed = true; }
        }
        if (!changed) break;
    }
    int max_depth = 0;
    for (auto const& [k, d] : depth) max_depth = std::max(max_depth, d);

    auto size_of = [&](Node const& n) -> Pos {
        if (sizes) { auto it = sizes->find(n.name); if (it != sizes->end() && it->second.x > 1) return it->second; }
        float longest = 0;
        for (auto const& p : n.inputs)  longest = std::max(longest, float(p.label.size()));
        for (auto const& p : n.outputs) longest = std::max(longest, float(p.label.size()));
        return {(120.0f + 7.5f * longest * 1.6f) * scale,
                (60.0f + 22.0f * float(std::max(n.inputs.size(), n.outputs.size()))) * scale};
    };
    const float gap_x = 90.0f * scale, gap_y = 30.0f * scale;

    // Columns: body sources at 0, modules from 1, sinks after the last.
    std::map<int, std::vector<Node const*>> columns;
    for (auto const* n : modules) columns[depth[n->name] + 1].push_back(n);
    if (auto const* s = w.node_named("@sources")) columns[0].push_back(s);
    if (auto const* e = w.node_named("@events"))  columns[0].push_back(e);
    if (auto const* k = w.node_named("@sinks"))   columns[max_depth + 2].push_back(k);
    for (auto& [c, v] : columns)
        std::sort(v.begin(), v.end(), [](Node const* a, Node const* b) { return a->index < b->index; });
    float x = 0;
    for (auto const& [c, v] : columns) {
        float y = 0, col_w = 0;
        for (auto const* n : v) { Pos sz = size_of(*n); out[n->name] = {x, y}; y += sz.y + gap_y; col_w = std::max(col_w, sz.x); }
        x += col_w + gap_x;
    }
    (void)cat;
    return out;
}

LayoutMap read_layout(Graph const& g) {
    LayoutMap out;
    auto const* md = g.metadata();
    if (!md || !md->contains("builder") || !(*md)["builder"].is_object()) return out;
    auto const& b = (*md)["builder"];
    if (!b.contains("layout") || !b["layout"].is_object()) return out;
    for (auto const& [k, v] : b["layout"].items())
        if (v.is_array() && v.size() == 2 && v[0].is_number() && v[1].is_number())
            out[k] = {v[0].get<float>(), v[1].get<float>()};
    return out;
}

bool write_layout(Graph& g, LayoutMap const& pos) {
    LayoutMap cur = read_layout(g);
    bool same = cur.size() == pos.size();
    if (same)
        for (auto const& [k, p] : pos) {
            auto it = cur.find(k);
            if (it == cur.end() || std::fabs(it->second.x - p.x) > 0.5f || std::fabs(it->second.y - p.y) > 0.5f) { same = false; break; }
        }
    if (same) return false;
    auto& md = g.metadata();
    if (!md.contains("builder") || !md["builder"].is_object()) md["builder"] = ojson::object();
    ojson layout = ojson::object();
    std::vector<std::string> keys;
    for (auto const& [k, p] : pos) keys.push_back(k);
    std::sort(keys.begin(), keys.end());
    for (auto const& k : keys) {
        Pos const& p = pos.at(k);
        layout[k] = ojson::array({std::round(p.x), std::round(p.y)});
    }
    md["builder"]["layout"] = std::move(layout);
    g.dirty = true;
    return true;
}

} // namespace bb
