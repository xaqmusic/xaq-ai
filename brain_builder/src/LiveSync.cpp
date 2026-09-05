#include "LiveSync.hpp"

#include <map>

namespace bb {

namespace {

std::map<std::string, nlohmann::json> by_id(nlohmann::json const& modules) {
    std::map<std::string, nlohmann::json> out;
    if (modules.is_array())
        for (auto const& m : modules) if (m.is_object() && m.contains("id")) out[m["id"].get<std::string>()] = m;
    return out;
}

} // namespace

LiveOps diff_for_live(Graph const& g, Catalogue const& cat, nlohmann::json const& synced) {
    LiveOps out;
    auto host = by_id(synced);
    std::map<std::string, nlohmann::json> local;
    std::vector<std::string> local_order;
    for (size_t i = 0; i < g.size(); ++i) {
        nlohmann::json m = g.module(i);
        local[g.id_of(i)] = m;
        local_order.push_back(g.id_of(i));
    }
    for (auto const& [id, m] : host)
        if (!local.count(id)) out.ops.push_back({{"op", "remove_node"}, {"id", id}});
    for (auto const& id : local_order) {
        nlohmann::json const& m = local[id];
        auto it = host.find(id);
        if (it == host.end()) {
            out.ops.push_back({{"op", "add_node"}, {"id", id}, {"type", m.value("type", "")}, {"params", m.value("params", nlohmann::json::object())}});
            continue;
        }
        nlohmann::json const& h = it->second;
        if (h.value("type", "") != m.value("type", "")) { out.recreate.push_back(id); continue; }
        TypeInfo const* ti = cat.find(m.value("type", ""));
        nlohmann::json const lp = m.value("params", nlohmann::json::object());
        nlohmann::json const hp = h.value("params", nlohmann::json::object());
        bool needs_recreate = false;
        std::vector<nlohmann::json> sets;
        auto consider = [&](std::string const& key) {
            bool in_l = lp.contains(key), in_h = hp.contains(key);
            if (in_l && in_h && lp[key] == hp[key]) return;
            if (!in_l && !in_h) return;
            ParamInfo const* pi = ti ? ti->param(key) : nullptr;
            if (in_l && pi && pi->hot) { sets.push_back({{"op", "set_param"}, {"id", id}, {"key", key}, {"value", lp[key]}}); return; }
            if (!in_l && pi && pi->hot && !pi->def.is_null()) { sets.push_back({{"op", "set_param"}, {"id", id}, {"key", key}, {"value", pi->def}}); return; }
            needs_recreate = true;
        };
        for (auto const& [k, v] : lp.items()) consider(k);
        for (auto const& [k, v] : hp.items()) if (!lp.contains(k)) consider(k);
        if (needs_recreate) out.recreate.push_back(id);
        else for (auto& s : sets) out.ops.push_back(std::move(s));
    }
    // Order: the host ticks in its own array order; a reorder needs a restart.
    std::vector<std::string> host_order;
    if (synced.is_array()) for (auto const& m : synced) if (m.contains("id")) host_order.push_back(m["id"].get<std::string>());
    std::vector<std::string> shared_local, shared_host;
    for (auto const& id : local_order) if (host.count(id)) shared_local.push_back(id);
    for (auto const& id : host_order) if (local.count(id)) shared_host.push_back(id);
    if (shared_local != shared_host) out.notes.push_back("execution order differs from the host; a reorder applies at the next restart (save the file)");
    return out;
}

nlohmann::json recreate_ops(Graph const& g, std::string const& id) {
    nlohmann::json ops = nlohmann::json::array();
    int i = g.index_of(id);
    if (i < 0) return ops;
    nlohmann::json m = g.module(size_t(i));
    ops.push_back({{"op", "remove_node"}, {"id", id}});
    ops.push_back({{"op", "add_node"}, {"id", id}, {"type", m.value("type", "")}, {"params", m.value("params", nlohmann::json::object())}});
    return ops;
}

void adopt_live_modules(Graph& g, nlohmann::json const& config) {
    if (!config.is_object()) return;
    ojson mods = ojson::array();
    if (config.contains("modules") && config["modules"].is_array())
        for (auto const& m : config["modules"]) {
            ojson o = ojson::object();
            o["id"]     = m.value("id", "");
            o["type"]   = m.value("type", "");
            o["params"] = ojson::parse(m.value("params", nlohmann::json::object()).dump());
            mods.push_back(std::move(o));
        }
    g.doc["modules"] = std::move(mods);
    if (config.contains("runtime") && config["runtime"].is_object()) g.doc["runtime"] = ojson::parse(config["runtime"].dump());
    ++g.revision;
}

} // namespace bb
