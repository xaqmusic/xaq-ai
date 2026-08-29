#include "patch.hpp"

#include <algorithm>
#include <cmath>

namespace xv {
namespace {

// Read tolerantly.  A patch file is hand-edited and a studio is a work in progress; a
// missing or wrong-typed field must fall back to the default, never take the tool down.
double      dnum (const json& j, const char* k, double d)             { auto it = j.find(k); return (it != j.end() && it->is_number())  ? it->get<double>()      : d; }
bool        dbool(const json& j, const char* k, bool d)               { auto it = j.find(k); return (it != j.end() && it->is_boolean()) ? it->get<bool>()        : d; }
int         dint (const json& j, const char* k, int d)                { auto it = j.find(k); return (it != j.end() && it->is_number())  ? it->get<int>()         : d; }
std::string dstr (const json& j, const char* k, const std::string& d) { auto it = j.find(k); return (it != j.end() && it->is_string())  ? it->get<std::string>() : d; }
json        dobj (const json& j, const char* k)                       { auto it = j.find(k); return (it != j.end() && it->is_object())  ? *it : json::object(); }
json        darr (const json& j, const char* k)                       { auto it = j.find(k); return (it != j.end() && it->is_array())   ? *it : json::array();  }

json src_json(const SourceRef& s) { return {{"module", s.module}, {"key", s.key}}; }
SourceRef src_from(const json& j) { return SourceRef{dstr(j, "module", ""), dstr(j, "key", "")}; }

json norm_json(const Norm& n) {
    return {{"mode", norm_mode_name(n.mode)}, {"z_lo", n.z_lo},   {"z_hi", n.z_hi},
            {"ref_key", n.ref_key},           {"gate", n.gate},   {"full", n.full},
            {"in_lo", n.in_lo},               {"in_hi", n.in_hi},
            {"smooth_ms", n.smooth_ms},       {"window_s", n.window_s}};
}
Norm norm_from(const json& j) {
    Norm n;
    norm_mode_from_name(dstr(j, "mode", norm_mode_name(n.mode)), n.mode);
    n.z_lo      = dnum(j, "z_lo", n.z_lo);
    n.z_hi      = dnum(j, "z_hi", n.z_hi);
    n.ref_key   = dstr(j, "ref_key", n.ref_key);
    n.gate      = dnum(j, "gate", n.gate);
    n.full      = dnum(j, "full", n.full);
    n.in_lo     = dnum(j, "in_lo", n.in_lo);
    n.in_hi     = dnum(j, "in_hi", n.in_hi);
    n.smooth_ms = dnum(j, "smooth_ms", n.smooth_ms);
    n.window_s  = dnum(j, "window_s", n.window_s);
    return n;
}

json filter_json(const FilterCfg& f) {
    return {{"enabled", f.enabled},   {"mode", filter_mode_name(f.mode)},
            {"cutoff_hz", f.cutoff_hz}, {"q", f.q}, {"mix", f.mix},
            {"vowel_a", f.vowel_a},   {"vowel_b", f.vowel_b}, {"morph", f.morph}};
}
FilterCfg filter_from(const json& j, FilterCfg f) {
    f.enabled = dbool(j, "enabled", f.enabled);
    filter_mode_from_name(dstr(j, "mode", filter_mode_name(f.mode)), f.mode);
    f.cutoff_hz = dnum(j, "cutoff_hz", f.cutoff_hz);
    f.q         = dnum(j, "q", f.q);
    f.mix       = dnum(j, "mix", f.mix);
    f.vowel_a   = dstr(j, "vowel_a", f.vowel_a);
    f.vowel_b   = dstr(j, "vowel_b", f.vowel_b);
    f.morph     = dnum(j, "morph", f.morph);
    return f;
}

json osc_json(const OscCfg& o) {
    return {{"waveform", wave_name(o.waveform)}, {"pulse_width", o.pulse_width},
            {"noise_mix", o.noise_mix},          {"base_hz", o.base_hz},
            {"level", o.level},                  {"pan", o.pan},
            {"glide_ms", o.glide_ms},            {"attack_ms", o.attack_ms},
            {"release_ms", o.release_ms},        {"quantize", o.quantize}};
}
OscCfg osc_from(const json& j) {
    OscCfg o;
    wave_from_name(dstr(j, "waveform", wave_name(o.waveform)), o.waveform);
    o.pulse_width = dnum(j, "pulse_width", o.pulse_width);
    o.noise_mix   = dnum(j, "noise_mix", o.noise_mix);
    // Accept a note name ("C3") as well as a number, since that is how the tool has always
    // talked about base pitch on the command line and in its status output.
    if (auto it = j.find("base_hz"); it != j.end()) {
        if (it->is_number())      o.base_hz = it->get<double>();
        else if (it->is_string()) { const float hz = note_to_hz(it->get<std::string>()); if (hz > 0) o.base_hz = hz; }
    }
    o.level      = dnum(j, "level", o.level);
    o.pan        = dnum(j, "pan", o.pan);
    o.glide_ms   = dnum(j, "glide_ms", o.glide_ms);
    o.attack_ms  = dnum(j, "attack_ms", o.attack_ms);
    o.release_ms = dnum(j, "release_ms", o.release_ms);
    o.quantize   = dint(j, "quantize", o.quantize);
    return o;
}

json route_json(const Route& r) {
    return {{"source", src_json(r.source)}, {"dest", dest_name(r.dest)},
            {"norm", norm_json(r.norm)},    {"depth", r.depth},
            {"curve", r.curve},             {"invert", r.invert}, {"enabled", r.enabled}};
}
Route route_from(const json& j) {
    Route r;
    r.source = src_from(dobj(j, "source"));
    dest_from_name(dstr(j, "dest", dest_name(r.dest)), r.dest);
    r.norm    = norm_from(dobj(j, "norm"));
    r.depth   = dnum(j, "depth", r.depth);
    r.curve   = dnum(j, "curve", r.curve);
    r.invert  = dbool(j, "invert", r.invert);
    r.enabled = dbool(j, "enabled", r.enabled);
    return r;
}

json event_json(const EventRoute& e) {
    return {{"source", src_json(e.source)}, {"trigger", trigger_name(e.trigger)},
            {"sound", e.sound},             {"enabled", e.enabled}};
}
EventRoute event_from(const json& j) {
    EventRoute e;
    e.source = src_from(dobj(j, "source"));
    trigger_from_name(dstr(j, "trigger", trigger_name(e.trigger)), e.trigger);
    e.sound   = dstr(j, "sound", e.sound);
    e.enabled = dbool(j, "enabled", e.enabled);
    return e;
}

}  // namespace

// ---------------------------------------------------------------------------- to/from
json to_json(const Patch& p) {
    json voices = json::array();
    for (const auto& v : p.voices) {
        json routes = json::array();
        for (const auto& r : v.routes) routes.push_back(route_json(r));
        json events = json::array();
        for (const auto& e : v.events) events.push_back(event_json(e));
        voices.push_back({{"id", v.id},
                          {"module", v.module},
                          {"enabled", v.enabled},
                          {"osc", osc_json(v.osc)},
                          {"filter", filter_json(v.filter)},
                          {"routes", std::move(routes)},
                          {"events", std::move(events)}});
    }
    return {{"version", p.version},
            {"engine", "xaq_voice"},
            {"brain", {{"host", p.brain.host}, {"port", p.brain.port}, {"hz", p.brain.hz}}},
            {"master",
             {{"volume", p.master.volume},
              {"quantize", p.master.quantize},
              {"scale", p.master.scale},
              {"span", p.master.span},
              {"filter", filter_json(p.master.filter)},
              {"routes", [&] {
                   json a = json::array();
                   for (const auto& r : p.master.routes) a.push_back(route_json(r));
                   return a;
               }()}}},
            {"voices", std::move(voices)}};
}

Patch from_json(const json& j) {
    Patch p;
    if (!j.is_object()) return p;
    p.version = dint(j, "version", p.version);

    const json b = dobj(j, "brain");
    p.brain.host = dstr(b, "host", p.brain.host);
    p.brain.port = dint(b, "port", p.brain.port);
    p.brain.hz   = dnum(b, "hz", p.brain.hz);

    const json m    = dobj(j, "master");
    p.master.volume   = dnum(m, "volume", p.master.volume);
    p.master.quantize = dbool(m, "quantize", p.master.quantize);
    p.master.scale    = dstr(m, "scale", p.master.scale);
    p.master.span     = dnum(m, "span", p.master.span);
    p.master.filter   = filter_from(dobj(m, "filter"), p.master.filter);
    for (const auto& rj : darr(m, "routes")) if (rj.is_object()) p.master.routes.push_back(route_from(rj));

    for (const auto& vj : darr(j, "voices")) {
        if (!vj.is_object()) continue;
        VoiceCfg v;
        v.module  = dstr(vj, "module", "");
        v.id      = dstr(vj, "id", v.module);
        v.enabled = dbool(vj, "enabled", true);
        v.osc     = osc_from(dobj(vj, "osc"));
        v.filter  = filter_from(dobj(vj, "filter"), v.filter);
        for (const auto& rj : darr(vj, "routes")) if (rj.is_object()) v.routes.push_back(route_from(rj));
        for (const auto& ej : darr(vj, "events")) if (ej.is_object()) v.events.push_back(event_from(ej));
        p.voices.push_back(std::move(v));
    }
    return p;
}

bool apply_ops(Patch& p, const json& ops, std::string& err) {
    if (!ops.is_array()) { err = "ops must be an array"; return false; }
    json j = to_json(p);
    for (const auto& op : ops) {
        if (!op.is_object() || !op.contains("path") || !op["path"].is_string()) {
            err = "each op needs a string 'path'";
            return false;
        }
        if (!op.contains("value")) { err = "each op needs a 'value'"; return false; }
        const std::string path = op["path"].get<std::string>();
        try {
            const auto ptr = json::json_pointer(path);
            // The location must already exist.  Assigning through a pointer would happily
            // grow an array to index 99 with nulls in between, or invent a misspelled key,
            // and both fail silently — the studio would report success on an op that
            // changed nothing an operator can hear.
            if (!j.contains(ptr)) { err = "no such path: " + path; return false; }
            j[ptr] = op["value"];
        } catch (const std::exception& e) {
            err = "bad path '" + path + "': " + e.what();
            return false;
        }
    }
    // Round-trip through the parser so a live edit can never reach a state the file
    // format cannot express — the studio and a hand-edited file get identical validation.
    p = from_json(j);
    return true;
}

// ---------------------------------------------------------------------------- discovery
namespace {

// The octave ladder the tool has always used, in the order modules arrive from
// list_modules.  Deliberately unchanged: the operator's ear is calibrated to it, and
// motor_epm being the LOWEST voice is load-bearing for telling the mix apart.
const double OCTAVES[] = {130.81, 261.63, 523.25, 1046.5, 65.41, 2093.0};   // C3 C4 C5 C6 C2 C7

// When several timbres are wanted, walk waveforms that differ in HARMONIC CONTENT rather
// than in name: square (odd only), saw (all), triangle (soft odd), pulse (bright, thin).
const Wave TIMBRES[] = {Wave::Square, Wave::Saw, Wave::Triangle, Wave::Pulse};

const ObservedSource* find_source(const ObservedModule& m, const std::string& key) {
    for (const auto& s : m.sources) if (s.key == key) return &s;
    return nullptr;
}

bool has_source(const ObservedModule& m, const std::string& key) { return find_source(m, key) != nullptr; }

// The signal a module is "about".  Preference order first, because a module that
// publishes a named error signal should be heard through it; otherwise the key that moves
// most, since a source that never varies makes no sound whatever it is routed to.
std::string primary_key(const ObservedModule& m) {
    for (const char* pref : {"motor_tle", "last_tle", "fused_tle", "ema_tle", "match_confidence", "urgency"})
        if (has_source(m, pref)) return pref;
    const ObservedSource* best = nullptr;
    for (const auto& s : m.sources) {
        if (s.is_bool || s.seen < 2) continue;
        // Compare spread relative to level, so a counter in the hundreds does not beat a
        // TLE in the hundredths purely by being large.
        const double rel  = s.var / (std::fabs(s.mean) + 1e-9);
        const double brel = best ? best->var / (std::fabs(best->mean) + 1e-9) : -1.0;
        if (!best || rel > brel) best = &s;
    }
    return best ? best->key : (m.sources.empty() ? std::string() : m.sources.front().key);
}

// A key that says "this is the level above which the module considers itself surprised".
std::string threshold_key(const ObservedModule& m) {
    for (const char* pref : {"novelty_threshold_now", "baking_threshold"})
        if (has_source(m, pref)) return pref;
    return {};
}

}  // namespace

Patch auto_patch(const std::vector<ObservedModule>& observed, const Patch& base, bool vary_timbre) {
    Patch p    = base;
    p.voices.clear();

    int nv = 0;
    for (const auto& m : observed) {
        // A module with no numeric source has nothing to say at rate.  Skipping it here is
        // what turns "silent oscillator" into "absent voice", which the studio can then
        // report honestly instead of the operator wondering why nothing sounds.
        bool any_numeric = false;
        for (const auto& s : m.sources) if (!s.is_bool) { any_numeric = true; break; }
        if (!any_numeric) continue;

        const std::string pk = primary_key(m);
        if (pk.empty()) continue;

        VoiceCfg v;
        v.id     = m.module;
        v.module = m.module;
        v.osc.base_hz  = OCTAVES[std::min(nv, 5)];
        v.osc.waveform = vary_timbre ? TIMBRES[nv % 4] : Wave::Square;

        // Pitch: the module's own signal, z-scored against its running median/MAD so the
        // mapping survives both the GNG baking and the scale of a source nobody has looked
        // at before.  Smoothed at ~60 ms — enough to kill tick jitter, short enough to keep
        // the spike that makes surprise audible as surprise.
        Route pitch;
        pitch.source        = {m.module, pk};
        pitch.dest          = Dest::Pitch;
        pitch.norm.mode     = NormMode::MedianMad;
        pitch.norm.z_lo     = 0.0;
        pitch.norm.z_hi     = 4.0;
        pitch.norm.smooth_ms = 60.0;
        pitch.depth         = p.master.span;
        pitch.curve         = 1.0;
        v.routes.push_back(pitch);

        // Volume: silence means "I know this".  Against a published novelty threshold when
        // the module has one; otherwise against its own median + 1 MAD, which is what
        // threshold_ratio falls back to when ref_key is empty.
        Route amp;
        amp.source          = {m.module, pk};
        amp.dest            = Dest::Amp;
        amp.norm.mode       = NormMode::ThresholdRatio;
        amp.norm.ref_key    = threshold_key(m);
        amp.norm.gate       = 1.4;
        amp.norm.full       = 2.0;
        amp.norm.smooth_ms  = 0.0;
        amp.depth           = 1.0;
        amp.curve           = 0.5;
        v.routes.push_back(amp);

        // The GNG's life, where there is one: a node earning its place, splitting, dying.
        if (has_source(m, "baked_now"))
            v.events.push_back({{m.module, "baked_now"}, Trigger::True, "chirp_up", true});
        if (has_source(m, "mitosis_count"))
            v.events.push_back({{m.module, "mitosis_count"}, Trigger::Increase, "two_notes", true});
        if (has_source(m, "nodes"))
            v.events.push_back({{m.module, "nodes"}, Trigger::Decrease, "blip_down", true});
        if (has_source(m, "just_baked"))
            v.events.push_back({{m.module, "just_baked"}, Trigger::True, "chirp_up", true});

        p.voices.push_back(std::move(v));
        ++nv;
    }
    return p;
}

}  // namespace xv
