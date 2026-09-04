#include "PaletteGen.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <ostream>
#include <set>

#include "TrialSetup.hpp"
#include "ogma/PayloadTypeName.hpp"

namespace fs = std::filesystem;

namespace bb {
namespace {

struct Draft {
    SocketInfo  s;
    std::string param;
    std::string base_topic;   // the baseline topic this probe replaced ("" if none)
    std::string base_value;   // the baseline param value (string params only)
};

struct Diff {
    std::vector<std::string>     removed;
    std::vector<ogma::TopicSpec> added;
};

Diff diff_ports(std::vector<ogma::TopicSpec> const& before, std::vector<ogma::TopicSpec> const& after) {
    Diff d;
    std::set<std::string> b, a;
    for (auto const& t : before) b.insert(t.name);
    for (auto const& t : after)  a.insert(t.name);
    for (auto const& t : before) if (!a.count(t.name)) d.removed.push_back(t.name);
    for (auto const& t : after)  if (!b.count(t.name)) d.added.push_back(t);
    return d;
}

std::string relative_to_root(fs::path const& p) {
    std::error_code ec;
    auto rel = fs::relative(p, fs::path(BB_ROOT), ec);
    return ec ? p.string() : rel.string();
}

// First module of each type across the config dirs whose trial setup succeeds.
std::map<std::string, std::pair<nlohmann::json, std::string>>
harvest_baselines(std::vector<std::string> const& dirs, std::ostream& log) {
    std::map<std::string, std::pair<nlohmann::json, std::string>> out;
    std::map<std::string, int> failures;
    for (auto const& dir : dirs) {
        std::error_code ec;
        if (!fs::is_directory(dir, ec)) { log << "  skip (not a directory): " << dir << "\n"; continue; }
        std::vector<fs::path> files;
        for (auto const& e : fs::directory_iterator(dir))
            if (e.path().extension() == ".json") files.push_back(e.path());
        std::sort(files.begin(), files.end());
        for (auto const& f : files) {
            std::ifstream in(f);
            nlohmann::json doc = nlohmann::json::parse(in, nullptr, false);
            if (doc.is_discarded() || !doc.contains("modules") || !doc["modules"].is_array()) continue;
            for (auto const& m : doc["modules"]) {
                std::string type = m.value("type", "");
                if (type.empty() || out.count(type)) continue;
                nlohmann::json params = m.value("params", nlohmann::json::object());
                ogma::ParamMap pm;
                try { pm = param_map_from_json(params); } catch (...) { continue; }
                TrialResult r = trial_setup(type, "probe", pm);
                if (r.ok) out[type] = {params, relative_to_root(f)};
                else ++failures[type];
            }
        }
    }
    for (auto const& [type, n] : failures)
        if (!out.count(type)) log << "  no shipped config constructs " << type << " (" << n << " tried)\n";
    return out;
}

ParamKind kind_of_param(TypeInfo const& t, std::string const& key) {
    auto const* p = t.param(key);
    return p ? p->kind : ParamKind::String;
}

} // namespace

nlohmann::ordered_json gen_palette(Catalogue const& cat,
                                   std::vector<std::string> const& config_dirs,
                                   std::ostream& log) {
    StdoutSilencer quiet;
    log << "harvesting baselines from " << config_dirs.size() << " config dir(s)\n";
    auto baselines = harvest_baselines(config_dirs, log);
    log << "  " << baselines.size() << " types have a constructing baseline\n";

    nlohmann::ordered_json out;
    out["types"] = nlohmann::ordered_json::object();

    for (auto const& t : cat.types) {
        nlohmann::ordered_json entry;
        ogma::ParamMap bp;
        std::string src;
        if (baselines.count(t.type)) {
            bp  = param_map_from_json(baselines[t.type].first);
            src = baselines[t.type].second;
        } else {
            nlohmann::json defaults = nlohmann::json::object();
            for (auto const& p : t.params) if (!p.required) defaults[p.key] = p.def;
            src = "defaults";
            if (t.baseline_params.is_object()) {
                for (auto const& [k, v] : t.baseline_params.items()) defaults[k] = v;
                src = "palette:baseline_params";
            }
            bp = param_map_from_json(defaults);
        }
        TrialResult r0 = trial_setup(t.type, "probe", bp);
        if (!r0.ok) {
            log << "  " << t.type << ": baseline (" << src << ") fails setup: " << r0.error << "\n";
            entry["baseline"] = nullptr;
            entry["error"]    = r0.error;
            entry["sockets"]  = nlohmann::ordered_json::array();
            entry["fixed"]    = nlohmann::ordered_json::array();
            out["types"][t.type] = std::move(entry);
            continue;
        }
        entry["baseline"] = src;

        std::vector<Draft> drafts;
        std::set<std::string> ever_removed;
        int probes = 0, hits = 0;

        // A required param has no default and therefore no type in the schema:
        // the baseline value is the authority on how to probe it, and its kind
        // is written back (kinds_probed) so the Properties panel gets it right.
        nlohmann::ordered_json kinds_probed = nlohmann::ordered_json::object();
        for (auto const& p : t.params) {
            enum { Str, ListStr, Int, Skip } pk = Skip;
            auto bit = bp.find(p.key);
            if (bit != bp.end()) {
                switch (bit->second.index()) {
                    case 1: pk = Int;     if (p.required) kinds_probed[p.key] = "int";         break;
                    case 3: pk = Str;     if (p.required) kinds_probed[p.key] = "string";      break;
                    case 5: pk = ListStr; if (p.required) kinds_probed[p.key] = "list_string"; break;
                    case 0: if (p.required) kinds_probed[p.key] = "bool";       break;
                    case 2: if (p.required) kinds_probed[p.key] = "float";      break;
                    case 4: if (p.required) kinds_probed[p.key] = "list_float"; break;
                    default: break;
                }
            } else {
                ParamKind k = kind_of_param(t, p.key);
                pk = k == ParamKind::String ? Str : k == ParamKind::ListString ? ListStr : k == ParamKind::Int ? Int : Skip;
            }
            // Ints are probed only where a level index composes a topic
            // (consensus.<level>); a large probe value elsewhere can size a grid.
            if (pk == Int && p.key.find("level") == std::string::npos) pk = Skip;
            if (pk == Skip) continue;

            std::string base_value;
            if (bit != bp.end()) {
                if (auto const* sv = std::get_if<std::string>(&bit->second)) base_value = *sv;
                else if (auto const* iv = std::get_if<int64_t>(&bit->second)) base_value = std::to_string(*iv);
            }
            std::string probe = pk == Int ? "777" : "bb.probe";
            if (pk == Str && !base_value.empty() && base_value.back() == '.') probe += ".";

            ogma::ParamMap pm = bp;
            // A list keeps its length (a module may require it to match another
            // list or a dimension): only the first entry becomes the probe.
            if (pk == ListStr) {
                std::vector<std::string> lst;
                if (bit != bp.end())
                    if (auto const* lv = std::get_if<std::vector<std::string>>(&bit->second)) lst = *lv;
                if (lst.empty()) lst.push_back(probe); else lst[0] = probe;
                pm[p.key] = lst;
            }
            else if (pk == Int)     pm[p.key] = int64_t(777);
            else                    pm[p.key] = probe;
            ++probes;
            TrialResult r = trial_setup(t.type, "probe", pm);
            if (!r.ok) continue;

            for (int dir = 0; dir < 2; ++dir) {
                bool output = dir == 1;
                Diff d = output ? diff_ports(r0.ports.outputs, r.ports.outputs)
                                : diff_ports(r0.ports.inputs,  r.ports.inputs);
                for (auto const& rm : d.removed) ever_removed.insert(rm);
                for (auto const& a : d.added) {
                    Draft dr;
                    dr.param       = p.key;
                    dr.base_value  = base_value;
                    dr.base_topic  = d.removed.size() == 1 ? d.removed.front() : "";
                    dr.s.output    = output;
                    dr.s.feedback  = a.kind == ogma::SubscriptionKind::Feedback;
                    dr.s.required  = a.required;
                    dr.s.list      = pk == ListStr;
                    dr.s.prefix    = !a.name.empty() && a.name.back() == '.';
                    dr.s.payload   = ogma::payload_type_name(a.payload_type);
                    dr.s.params    = {p.key};
                    size_t pos = a.name.find(probe);
                    if (pos == std::string::npos) { dr.s.pattern = a.name; dr.s.dynamic = true; }
                    else dr.s.pattern = a.name.substr(0, pos) + "{" + p.key + "}" + a.name.substr(pos + probe.size());
                    // The other entries of a probed list are sockets of the
                    // same param, not fixed topics.
                    if (pk == ListStr && !dr.s.dynamic) {
                        if (auto const* lv = (bit != bp.end()) ? std::get_if<std::vector<std::string>>(&bit->second) : nullptr) {
                            std::string const ph = "{" + p.key + "}";
                            for (auto const& v : *lv) {
                                std::string topic = dr.s.pattern;
                                size_t at = topic.find(ph);
                                if (at != std::string::npos) topic.replace(at, ph.size(), v);
                                ever_removed.insert(topic);
                            }
                        }
                    }
                    drafts.push_back(std::move(dr));
                    ++hits;
                }
            }
        }

        // Merge probes that replaced the same baseline topic: a composed socket.
        std::vector<SocketInfo> sockets;
        std::vector<bool> used(drafts.size(), false);
        for (size_t i = 0; i < drafts.size(); ++i) {
            if (used[i]) continue;
            used[i] = true;
            Draft const& a = drafts[i];
            std::vector<size_t> group{i};
            if (!a.base_topic.empty())
                for (size_t j = i + 1; j < drafts.size(); ++j)
                    if (!used[j] && drafts[j].base_topic == a.base_topic &&
                        drafts[j].s.output == a.s.output && drafts[j].s.feedback == a.s.feedback &&
                        drafts[j].param != a.param) { used[j] = true; group.push_back(j); }
            SocketInfo s = a.s;
            if (group.size() > 1) {
                std::string pattern = a.base_topic;
                s.params.clear();
                for (size_t gi : group) {
                    Draft const& d = drafts[gi];
                    s.params.push_back(d.param);
                    if (d.base_value.empty()) continue;
                    size_t pos = pattern.find(d.base_value);
                    if (pos != std::string::npos)
                        pattern = pattern.substr(0, pos) + "{" + d.param + "}" + pattern.substr(pos + d.base_value.size());
                    s.list = s.list || d.s.list;
                }
                s.pattern = pattern;
                s.dynamic = false;
            }
            bool dup = false;
            for (auto const& e : sockets)
                if (e.pattern == s.pattern && e.output == s.output && e.feedback == s.feedback) dup = true;
            if (!dup) sockets.push_back(std::move(s));
        }

        std::vector<FixedTopic> fixed;
        for (int dir = 0; dir < 2; ++dir) {
            auto const& ports = dir ? r0.ports.outputs : r0.ports.inputs;
            for (auto const& sp : ports) {
                if (ever_removed.count(sp.name)) continue;
                FixedTopic f;
                f.topic    = sp.name;
                f.output   = dir == 1;
                f.feedback = sp.kind == ogma::SubscriptionKind::Feedback;
                f.required = sp.required;
                f.payload  = ogma::payload_type_name(sp.payload_type);
                fixed.push_back(std::move(f));
            }
        }

        log << "  " << t.type << ": " << probes << " probes, " << sockets.size() << " sockets, "
            << fixed.size() << " fixed (baseline " << src << ")\n";
        auto sj = nlohmann::ordered_json::array();
        for (auto const& s : sockets) sj.push_back(socket_to_json(s));
        auto fj = nlohmann::ordered_json::array();
        for (auto const& f : fixed) fj.push_back(fixed_to_json(f));
        entry["sockets"]      = std::move(sj);
        entry["fixed"]        = std::move(fj);
        entry["kinds_probed"] = std::move(kinds_probed);
        (void)hits;
        out["types"][t.type] = std::move(entry);
    }
    return out;
}

void merge_palette(std::string const& palette_path, nlohmann::ordered_json const& gen) {
    nlohmann::ordered_json pal = nlohmann::ordered_json::object();
    {
        std::ifstream f(palette_path);
        if (f) {
            pal = nlohmann::ordered_json::parse(f, nullptr, false);
            if (pal.is_discarded()) pal = nlohmann::ordered_json::object();
        }
    }
    if (!pal.contains("types") || !pal["types"].is_object()) pal["types"] = nlohmann::ordered_json::object();
    for (auto const& [type, entry] : gen["types"].items()) {
        auto& pe = pal["types"][type];
        if (!pe.is_object()) pe = nlohmann::ordered_json::object();
        pe["baseline"] = entry["baseline"];
        pe["sockets"]  = entry["sockets"];
        pe["fixed"]    = entry["fixed"];
        if (entry.contains("kinds_probed")) pe["kinds_probed"] = entry["kinds_probed"];
        if (entry.contains("error")) pe["gen_error"] = entry["error"]; else pe.erase("gen_error");
    }
    std::ofstream out(palette_path);
    out << pal.dump(2) << "\n";
}

} // namespace bb
