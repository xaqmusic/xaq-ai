#include "Graph.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "ogma/GraphConfig.hpp"

namespace bb {

void Graph::ensure_shape() {
    if (!doc.is_object()) doc = ojson::object();
    if (!doc.contains("version")) doc["version"] = 1;
    if (!doc.contains("runtime") || !doc["runtime"].is_object())
        doc["runtime"] = ojson{{"thread_pool", "per_instance"}, {"num_threads", 0}};
    if (!doc.contains("modules") || !doc["modules"].is_array()) doc["modules"] = ojson::array();
    if (!doc.contains("edges") || !doc["edges"].is_array()) doc["edges"] = ojson::array();
}

Graph Graph::empty(std::string const& env_target) {
    Graph g;
    g.doc = ojson::object();
    g.doc["metadata"] = ojson{{"name", ""}, {"env_target", env_target}, {"description", ""}};
    g.doc["version"] = 1;
    g.doc["runtime"] = ojson{{"thread_pool", "per_instance"}, {"num_threads", 0}};
    g.doc["modules"] = ojson::array();
    g.doc["edges"]   = ojson::array();
    g.doc["description"] = "";
    return g;
}

Graph Graph::load(std::string const& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    Graph g;
    try {
        g.doc = ojson::parse(text);
    } catch (std::exception const& e) {
        throw std::runtime_error(path + ": " + e.what());
    }
    g.path = path;
    g.ascii_escapes = text.find("\\u") != std::string::npos;
    g.ensure_shape();
    for (size_t i = 0; i < g.size(); ++i) {
        auto& m = g.module(i);
        if (!m.is_object()) throw std::runtime_error(path + ": modules[" + std::to_string(i) + "] is not an object");
        if (!m.contains("params") || !m["params"].is_object()) m["params"] = ojson::object();
    }
    return g;
}

std::string Graph::dump() const {
    return doc.dump(2, ' ', ascii_escapes) + "\n";
}

void Graph::save(std::string const& to_path) {
    std::ofstream f(to_path);
    if (!f) throw std::runtime_error("cannot write " + to_path);
    f << dump();
    path  = to_path;
    dirty = false;
}

size_t       Graph::size() const            { return doc["modules"].size(); }
ojson&       Graph::module(size_t i)        { return doc["modules"][i]; }
ojson const& Graph::module(size_t i) const  { return doc["modules"][i]; }
std::string  Graph::id_of(size_t i) const   { return module(i).value("id", ""); }
std::string  Graph::type_of(size_t i) const { return module(i).value("type", ""); }
ojson&       Graph::params(size_t i)        { return module(i)["params"]; }
ojson const& Graph::params(size_t i) const  { return module(i)["params"]; }

int Graph::index_of(std::string const& id) const {
    for (size_t i = 0; i < size(); ++i) if (id_of(i) == id) return int(i);
    return -1;
}
ojson*       Graph::find(std::string const& id)       { int i = index_of(id); return i < 0 ? nullptr : &module(size_t(i)); }
ojson const* Graph::find(std::string const& id) const { int i = index_of(id); return i < 0 ? nullptr : &module(size_t(i)); }

std::vector<std::string> Graph::ids() const {
    std::vector<std::string> out;
    for (size_t i = 0; i < size(); ++i) out.push_back(id_of(i));
    return out;
}

ojson& Graph::metadata() {
    if (!doc.contains("metadata") || !doc["metadata"].is_object()) doc["metadata"] = ojson::object();
    return doc["metadata"];
}
ojson const* Graph::metadata() const {
    if (!doc.contains("metadata") || !doc["metadata"].is_object()) return nullptr;
    return &doc["metadata"];
}
std::string Graph::env_target() const {
    auto const* m = metadata();
    return m ? m->value("env_target", "") : "";
}
std::string Graph::body_manifest() const {
    auto const* m = metadata();
    return m ? m->value("body_manifest", "") : "";
}
void Graph::set_body_manifest(std::string const& body) {
    metadata()["body_manifest"] = body;
    dirty = true;
}

std::string Graph::unique_id(std::string const& prefix) const {
    if (index_of(prefix) < 0) return prefix;
    for (int n = 2; n < 10000; ++n) {
        std::string cand = prefix + "_" + std::to_string(n);
        if (index_of(cand) < 0) return cand;
    }
    return prefix + "_x";
}

// --- undo ------------------------------------------------------------------
void Graph::checkpoint() {
    undo_.push_back(doc);
    if (undo_.size() > 200) undo_.erase(undo_.begin());
    redo_.clear();
}
bool Graph::undo() {
    if (undo_.empty()) return false;
    redo_.push_back(doc);
    doc = std::move(undo_.back());
    undo_.pop_back();
    dirty = true;
    return true;
}
bool Graph::redo() {
    if (redo_.empty()) return false;
    undo_.push_back(doc);
    doc = std::move(redo_.back());
    redo_.pop_back();
    dirty = true;
    return true;
}

// --- mutate ----------------------------------------------------------------
std::string Graph::add_module(std::string const& type, std::string const& id, ojson params, int at) {
    checkpoint();
    std::string uid = unique_id(id);
    ojson m = ojson::object();
    m["id"]     = uid;
    m["type"]   = type;
    m["params"] = params.is_object() ? params : ojson::object();
    auto& mods = doc["modules"];
    if (at < 0 || at >= int(mods.size())) mods.push_back(std::move(m));
    else mods.insert(mods.begin() + at, std::move(m));
    dirty = true;
    return uid;
}

bool Graph::remove_module(std::string const& id) {
    int i = index_of(id);
    if (i < 0) return false;
    checkpoint();
    auto& mods = doc["modules"];
    mods.erase(mods.begin() + i);
    if (auto* md = metadata().contains("builder") ? &metadata()["builder"] : nullptr)
        if (md->contains("layout") && (*md)["layout"].is_object()) (*md)["layout"].erase(id);
    dirty = true;
    return true;
}

bool Graph::move_module(int from, int to) {
    int n = int(size());
    if (from < 0 || from >= n || to < 0 || to >= n || from == to) return false;
    checkpoint();
    auto& mods = doc["modules"];
    ojson m = mods[size_t(from)];
    mods.erase(mods.begin() + from);
    mods.insert(mods.begin() + to, std::move(m));
    dirty = true;
    return true;
}

void Graph::set_param(std::string const& id, std::string const& key, ojson value, bool with_checkpoint) {
    ojson* m = find(id);
    if (!m) return;
    if (with_checkpoint) checkpoint();
    (*m)["params"][key] = std::move(value);
    dirty = true;
}

void Graph::erase_param(std::string const& id, std::string const& key) {
    ojson* m = find(id);
    if (!m || !(*m)["params"].contains(key)) return;
    checkpoint();
    (*m)["params"].erase(key);
    dirty = true;
}

bool Graph::rename_module(std::string const& old_id, std::string const& new_id) {
    if (new_id.empty() || old_id == new_id || index_of(new_id) >= 0) return false;
    ojson* m = find(old_id);
    if (!m) return false;
    checkpoint();
    (*m)["id"] = new_id;
    auto& md = metadata();
    if (md.contains("builder") && md["builder"].contains("layout") && md["builder"]["layout"].contains(old_id)) {
        md["builder"]["layout"][new_id] = md["builder"]["layout"][old_id];
        md["builder"]["layout"].erase(old_id);
    }
    dirty = true;
    return true;
}

ogma::GraphConfig Graph::to_graph_config() const {
    return ogma::GraphConfig::load_from_json(doc.dump());
}

} // namespace bb
