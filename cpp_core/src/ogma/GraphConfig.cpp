// =============================================================================
// GraphConfig.cpp  --  JSON serialization / deserialization
// =============================================================================
//
// Uses nlohmann/json (already in the dependency tree).
//
// ParamValue variant mapping:
//   bool            → JSON boolean
//   int64_t         → JSON integer
//   double          → JSON number
//   string          → JSON string
//   vector<double>  → JSON array of numbers
//   vector<string>  → JSON array of strings

#include "ogma/GraphConfig.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace ogma {

using json = nlohmann::json;

// --------------------------------------------------------------------------
// ParamValue  ↔  json
// --------------------------------------------------------------------------

namespace {

json param_to_json(ParamValue const& v) {
    return std::visit([](auto const& x) -> json {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, bool>)
            return x;
        else if constexpr (std::is_same_v<T, int64_t>)
            return x;
        else if constexpr (std::is_same_v<T, double>)
            return x;
        else if constexpr (std::is_same_v<T, std::string>)
            return x;
        else if constexpr (std::is_same_v<T, std::vector<double>>) {
            json arr = json::array();
            for (auto d : x) arr.push_back(d);
            return arr;
        } else {
            json arr = json::array();
            for (auto const& s : x) arr.push_back(s);
            return arr;
        }
    }, v);
}

ParamValue json_to_param(json const& j) {
    if (j.is_boolean())       return ParamValue{j.get<bool>()};
    if (j.is_number_integer()) return ParamValue{j.get<int64_t>()};
    if (j.is_number_float())  return ParamValue{j.get<double>()};
    if (j.is_string())         return ParamValue{j.get<std::string>()};
    if (j.is_array() && !j.empty()) {
        if (j[0].is_string()) {
            std::vector<std::string> v;
            for (auto const& e : j) v.push_back(e.get<std::string>());
            return ParamValue{v};
        } else {
            std::vector<double> v;
            for (auto const& e : j) v.push_back(e.get<double>());
            return ParamValue{v};
        }
    }
    if (j.is_array()) return ParamValue{std::vector<double>{}};
    // Fallback for null/object: treat as empty string.
    return ParamValue{std::string{}};
}

RuntimeSpec parse_runtime(json const& j) {
    RuntimeSpec rt;
    if (j.contains("thread_pool")) {
        auto tp = j["thread_pool"].get<std::string>();
        rt.thread_pool = (tp == "shared") ? ThreadPoolPolicy::Shared
                                           : ThreadPoolPolicy::PerInstance;
    }
    if (j.contains("num_threads"))
        rt.num_threads = j["num_threads"].get<int>();
    if (j.contains("auto_subscribe"))
        rt.auto_subscribe = j["auto_subscribe"].get<bool>();
    return rt;
}

json runtime_to_json(RuntimeSpec const& rt) {
    json j;
    j["thread_pool"]    = (rt.thread_pool == ThreadPoolPolicy::Shared) ? "shared" : "per_instance";
    j["num_threads"]    = rt.num_threads;
    j["auto_subscribe"] = rt.auto_subscribe;
    return j;
}

} // namespace

// --------------------------------------------------------------------------
// GraphConfig::load_from_json
// --------------------------------------------------------------------------

GraphConfig GraphConfig::load_from_json(std::string_view text) {
    json root;
    try { root = json::parse(text); }
    catch (json::parse_error const& e) {
        throw std::runtime_error(std::string("GraphConfig::load_from_json parse error: ") + e.what());
    }

    GraphConfig g;

    if (root.contains("version"))
        g.version = root["version"].get<int>();

    if (root.contains("runtime"))
        g.runtime = parse_runtime(root["runtime"]);

    if (root.contains("modules")) {
        for (auto const& m : root["modules"]) {
            ModuleSpec spec;
            spec.id   = m.value("id",   "");
            spec.type = m.value("type", "");
            if (spec.id.empty())
                throw std::runtime_error("GraphConfig: module has empty 'id'");
            if (spec.type.empty())
                throw std::runtime_error("GraphConfig: module '" + spec.id + "' has empty 'type'");
            if (m.contains("params")) {
                for (auto it = m["params"].begin(); it != m["params"].end(); ++it)
                    spec.params[it.key()] = json_to_param(it.value());
            }
            g.modules.push_back(std::move(spec));
        }
    }

    // Validate unique IDs.
    std::unordered_map<std::string, bool> seen;
    for (auto const& ms : g.modules) {
        if (seen.count(ms.id))
            throw std::runtime_error("GraphConfig: duplicate module id '" + ms.id + "'");
        seen[ms.id] = true;
    }

    if (root.contains("edges")) {
        for (auto const& e : root["edges"]) {
            EdgeSpec edge;
            edge.from     = e.value("from",     "");
            edge.to       = e.value("to",       "");
            edge.topic    = e.value("topic",    "");
            edge.feedback = e.value("feedback", false);
            g.edges.push_back(std::move(edge));
        }
    }

    return g;
}

GraphConfig GraphConfig::load_from_file(std::string_view path) {
    std::ifstream f{std::string(path)};
    if (!f) throw std::runtime_error("GraphConfig::load_from_file: cannot open " + std::string(path));
    std::ostringstream ss;
    ss << f.rdbuf();
    return load_from_json(ss.str());
}

// --------------------------------------------------------------------------
// GraphConfig::to_json
// --------------------------------------------------------------------------

std::string GraphConfig::to_json() const {
    json root;
    root["version"] = version;
    root["runtime"] = runtime_to_json(runtime);

    root["modules"] = json::array();
    for (auto const& ms : modules) {
        json m;
        m["id"]   = ms.id;
        m["type"] = ms.type;
        m["params"] = json::object();
        for (auto const& [k, v] : ms.params)
            m["params"][k] = param_to_json(v);
        root["modules"].push_back(std::move(m));
    }

    root["edges"] = json::array();
    for (auto const& e : edges) {
        json ej;
        ej["from"] = e.from;
        ej["to"]   = e.to;
        if (!e.topic.empty())    ej["topic"]    = e.topic;
        if (e.feedback)          ej["feedback"] = true;
        root["edges"].push_back(std::move(ej));
    }

    return root.dump(2);
}

} // namespace ogma
