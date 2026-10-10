#include "Catalogue.hpp"

#include <algorithm>
#include <fstream>
#include <regex>
#include <set>

#include "TrialSetup.hpp"
#include "ogma/Module.hpp"

namespace bb {

const std::vector<std::string> kCategoryOrder = {
    "sensory", "fusion", "neurochem", "drive", "predictor",
    "nav", "reflex", "motor", "meta", "instrument", "other"};

const std::vector<std::string> kPayloadTypes = {
    "RealityToken", "ConsensusToken", "NeuroState", "DriveErrors", "ActionOut",
    "FaderState", "PolicyToken", "IntentToken", "EpisodicChunkProposal",
    "PredictionToken", "SequenceMotif", "ExplorationDirective", "MotorChunk",
    "MotorChunks", "MotorPlayCmd", "MotorPlayStream", "RolloutQuery",
    "RolloutResult", "RawImageFrame", "RawAudioFrame", "ProprioToken", "EnvEvent",
    "ReflexGate", "HormoneState", "FitnessScore", "GainVector",
    "AdaptiveThreshold", "Unknown"};

const char* kind_name(ParamKind k) {
    switch (k) {
        case ParamKind::Bool:       return "bool";
        case ParamKind::Int:        return "int";
        case ParamKind::Float:      return "float";
        case ParamKind::String:     return "string";
        case ParamKind::ListFloat:  return "list_float";
        case ParamKind::ListString: return "list_string";
    }
    return "string";
}

ParamKind kind_from_name(std::string const& s, ParamKind fallback) {
    if (s == "bool")        return ParamKind::Bool;
    if (s == "int")         return ParamKind::Int;
    if (s == "float")       return ParamKind::Float;
    if (s == "string")      return ParamKind::String;
    if (s == "list_float")  return ParamKind::ListFloat;
    if (s == "list_string") return ParamKind::ListString;
    return fallback;
}

ParamKind kind_of_json(nlohmann::json const& v, ParamKind fallback) {
    if (v.is_boolean())        return ParamKind::Bool;
    if (v.is_number_integer()) return ParamKind::Int;
    if (v.is_number_float())   return ParamKind::Float;
    if (v.is_string())         return ParamKind::String;
    if (v.is_array()) {
        for (auto const& e : v) if (e.is_string()) return ParamKind::ListString;
        return ParamKind::ListFloat;
    }
    return fallback;
}

ParamInfo const* TypeInfo::param(std::string const& key) const {
    for (auto const& p : params) if (p.key == key) return &p;
    return nullptr;
}

TypeInfo const* Catalogue::find(std::string const& type) const {
    for (auto const& t : types) if (t.type == type) return &t;
    return nullptr;
}

std::vector<std::string> Catalogue::categories() const {
    std::vector<std::string> out;
    for (auto const& c : kCategoryOrder)
        for (auto const& t : types)
            if (t.category == c) { out.push_back(c); break; }
    return out;
}

// ---------------------------------------------------------------------------
// palette.json (de)serialisation of the pieces the generator also writes
// ---------------------------------------------------------------------------
nlohmann::ordered_json socket_to_json(SocketInfo const& s) {
    nlohmann::ordered_json j;
    j["pattern"]  = s.pattern;
    j["params"]   = s.params;
    j["output"]   = s.output;
    j["payload"]  = s.payload;
    if (s.feedback)  j["feedback"] = true;
    if (!s.required) j["required"] = false;
    if (s.list)      j["list"]     = true;
    if (s.prefix)    j["prefix"]   = true;
    if (s.dynamic)   j["dynamic"]  = true;
    if (s.polled)    j["polled"]   = true;
    return j;
}

nlohmann::ordered_json fixed_to_json(FixedTopic const& f) {
    nlohmann::ordered_json j;
    j["topic"]   = f.topic;
    j["output"]  = f.output;
    j["payload"] = f.payload;
    if (f.feedback)  j["feedback"] = true;
    if (!f.required) j["required"] = false;
    return j;
}

SocketInfo socket_from_json(nlohmann::json const& j) {
    SocketInfo s;
    s.pattern  = j.value("pattern", "");
    if (j.contains("params") && j["params"].is_array())
        for (auto const& p : j["params"]) s.params.push_back(p.get<std::string>());
    s.output   = j.value("output", false);
    s.payload  = j.value("payload", "Unknown");
    s.feedback = j.value("feedback", false);
    s.required = j.value("required", true);
    s.list     = j.value("list", false);
    s.prefix   = j.value("prefix", false);
    s.dynamic  = j.value("dynamic", false);
    s.polled   = j.value("polled", false);
    return s;
}

FixedTopic fixed_from_json(nlohmann::json const& j) {
    FixedTopic f;
    f.topic    = j.value("topic", "");
    f.output   = j.value("output", false);
    f.payload  = j.value("payload", "Unknown");
    f.feedback = j.value("feedback", false);
    f.required = j.value("required", true);
    return f;
}

// ---------------------------------------------------------------------------
// build
// ---------------------------------------------------------------------------
namespace {

std::vector<std::string> parse_enum(std::string const& description) {
    // "jl|stft|rbf|identity" as the whole description, or an "a|b|c" group in
    // quotes, parentheses, brackets or braces.  Tokens are identifiers, so prose
    // such as "[3 targets | 3 weights]" or "|error|" does not match.
    static const std::regex re(R"((?:^|["'(\[{])\s*([A-Za-z_][A-Za-z0-9_]*(?:\s*\|\s*[A-Za-z_][A-Za-z0-9_]*)+)\s*(?:$|["')\]}]))");
    std::smatch m;
    if (!std::regex_search(description, m, re)) return {};
    std::vector<std::string> out;
    std::string s = m[1];
    size_t start = 0;
    while (true) {
        size_t bar = s.find('|', start);
        std::string tok = s.substr(start, bar == std::string::npos ? std::string::npos : bar - start);
        tok.erase(0, tok.find_first_not_of(" \t"));
        tok.erase(tok.find_last_not_of(" \t") + 1);
        if (!tok.empty()) out.push_back(tok);
        if (bar == std::string::npos) break;
        start = bar + 1;
    }
    return out.size() >= 2 ? out : std::vector<std::string>{};
}

// For a required param (no default, so no type) when nothing better is known.
ParamKind kind_from_description(std::string const& key, std::string const& d) {
    if (key.size() > 7 && key.compare(key.size() - 7, 7, "_topics") == 0) return ParamKind::ListString;
    std::string l = d;
    std::transform(l.begin(), l.end(), l.begin(), [](unsigned char c) { return std::tolower(c); });
    bool arr = l.rfind("array", 0) == 0 || l.find("array of") != std::string::npos ||
               l.find("list of") != std::string::npos || l.find("json array") != std::string::npos;
    if (arr) {
        bool strings = l.find("topic") != std::string::npos || l.find("name") != std::string::npos ||
                       l.find("string") != std::string::npos || l.find("label") != std::string::npos;
        return strings ? ParamKind::ListString : ParamKind::ListFloat;
    }
    if (l.find("integer") != std::string::npos) return ParamKind::Int;
    if (l.find("true/false") != std::string::npos) return ParamKind::Bool;
    return ParamKind::String;
}

int category_rank(std::string const& c) {
    auto it = std::find(kCategoryOrder.begin(), kCategoryOrder.end(), c);
    return it == kCategoryOrder.end() ? int(kCategoryOrder.size()) : int(it - kCategoryOrder.begin());
}

} // namespace

Catalogue Catalogue::build(std::string const& palette_path) {
    Catalogue cat;
    cat.palette_path = palette_path;

    nlohmann::json palette = nlohmann::json::object();
    {
        std::ifstream f(palette_path);
        if (f) {
            palette = nlohmann::json::parse(f, nullptr, false);
            if (palette.is_discarded()) {
                cat.warnings.push_back("palette.json does not parse: " + palette_path);
                palette = nlohmann::json::object();
            }
        } else {
            cat.warnings.push_back("palette.json not found: " + palette_path);
        }
    }
    nlohmann::json const types_pal = palette.value("types", nlohmann::json::object());

    warm_registry();
    auto& reg = ogma::ModuleRegistry::instance();
    std::vector<std::string> names = reg.registered_types();
    std::sort(names.begin(), names.end());

    for (auto const& name : names) {
        TypeInfo t;
        t.type = name;
        ogma::ModulePtr m;
        try { m = reg.create(name); } catch (std::exception const& e) {
            cat.warnings.push_back(name + ": create() threw: " + e.what());
        }
        nlohmann::json pe = types_pal.contains(name) ? types_pal[name] : nlohmann::json::object();
        nlohmann::json kinds        = pe.value("kinds", nlohmann::json::object());
        std::set<std::string> tolerated;
        for (auto const& k : pe.value("tolerated_missing", nlohmann::json::array())) if (k.is_string()) tolerated.insert(k.get<std::string>());
        nlohmann::json kinds_probed = pe.value("kinds_probed", nlohmann::json::object());

        if (m) {
            for (auto const& spec : m->params_schema()) {
                ParamInfo p;
                p.key         = spec.key;
                p.hot         = spec.mutability == ogma::ParamMutability::HotMutable;
                p.description = spec.description;
                if (spec.default_value) {
                    p.def  = param_to_json(*spec.default_value);
                    p.kind = kind_of_json(p.def, ParamKind::String);
                } else {
                    p.required = true;
                    p.kind = kind_from_description(p.key, p.description);
                    if (spec.min_value) p.kind = kind_of_json(param_to_json(*spec.min_value), p.kind);
                    if (kinds_probed.contains(p.key) && kinds_probed[p.key].is_string())
                        p.kind = kind_from_name(kinds_probed[p.key].get<std::string>(), p.kind);
                }
                if (spec.min_value) p.min = param_to_json(*spec.min_value);
                if (spec.max_value) p.max = param_to_json(*spec.max_value);
                if (kinds.contains(p.key) && kinds[p.key].is_string())
                    p.kind = kind_from_name(kinds[p.key].get<std::string>(), p.kind);
                if (p.required && tolerated.count(p.key)) p.required = false;   // the module has a fallback
                if (p.kind == ParamKind::String) p.enum_values = parse_enum(p.description);
                t.params.push_back(std::move(p));
            }
        }

        if (pe.empty()) {
            cat.warnings.push_back(name + ": no palette entry (category 'other')");
        }
        t.category        = pe.value("category", std::string("other"));
        t.layer           = pe.value("layer", 3);
        t.purpose         = pe.value("purpose", std::string());
        t.deprecated      = pe.value("deprecated", false);
        t.deprecated_note = pe.value("deprecated_note", std::string());
        t.id_prefix       = pe.value("id_prefix", std::string());
        t.no_trial_setup  = pe.value("no_trial_setup", false);
        if (pe.contains("baseline") && pe["baseline"].is_string()) t.baseline = pe["baseline"];
        if (pe.contains("baseline_params")) t.baseline_params = pe["baseline_params"];
        if (t.id_prefix.empty()) {
            t.id_prefix = name;
            std::transform(t.id_prefix.begin(), t.id_prefix.end(), t.id_prefix.begin(),
                           [](unsigned char c) { return std::tolower(c); });
        }
        for (char const* key : {"sockets", "sockets_extra"})
            if (pe.contains(key) && pe[key].is_array())
                for (auto const& sj : pe[key]) t.sockets.push_back(socket_from_json(sj));
        if (pe.contains("fixed") && pe["fixed"].is_array())
            for (auto const& fj : pe["fixed"]) t.fixed.push_back(fixed_from_json(fj));

        cat.types.push_back(std::move(t));
    }

    std::stable_sort(cat.types.begin(), cat.types.end(), [](TypeInfo const& a, TypeInfo const& b) {
        int ra = category_rank(a.category), rb = category_rank(b.category);
        if (ra != rb) return ra < rb;
        return a.type < b.type;
    });
    return cat;
}

nlohmann::ordered_json Catalogue::to_json() const {
    nlohmann::ordered_json root;
    root["palette"] = palette_path;
    root["type_count"] = types.size();
    root["payload_types"] = kPayloadTypes;
    root["categories"] = kCategoryOrder;
    auto arr = nlohmann::ordered_json::array();
    for (auto const& t : types) {
        nlohmann::ordered_json j;
        j["type"]       = t.type;
        j["category"]   = t.category;
        j["layer"]      = t.layer;
        j["purpose"]    = t.purpose;
        j["id_prefix"]  = t.id_prefix;
        if (t.deprecated) { j["deprecated"] = true; j["deprecated_note"] = t.deprecated_note; }
        if (!t.baseline.empty()) j["baseline"] = t.baseline;
        auto params = nlohmann::ordered_json::array();
        for (auto const& p : t.params) {
            nlohmann::ordered_json pj;
            pj["key"] = p.key;
            pj["kind"] = kind_name(p.kind);
            pj["mutability"] = p.hot ? "hot" : "construction";
            if (p.required) pj["required"] = true; else pj["default"] = p.def;
            if (!p.min.is_null()) pj["min"] = p.min;
            if (!p.max.is_null()) pj["max"] = p.max;
            if (!p.enum_values.empty()) pj["enum"] = p.enum_values;
            pj["description"] = p.description;
            params.push_back(std::move(pj));
        }
        j["params"] = std::move(params);
        auto socks = nlohmann::ordered_json::array();
        for (auto const& s : t.sockets) socks.push_back(socket_to_json(s));
        j["sockets"] = std::move(socks);
        auto fixed = nlohmann::ordered_json::array();
        for (auto const& f : t.fixed) fixed.push_back(fixed_to_json(f));
        j["fixed"] = std::move(fixed);
        arr.push_back(std::move(j));
    }
    root["types"] = std::move(arr);
    root["warnings"] = warnings;
    return root;
}

} // namespace bb
