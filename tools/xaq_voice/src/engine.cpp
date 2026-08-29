#include "engine.hpp"

#include <algorithm>
#include <cmath>

namespace xv {

// ---------------------------------------------------------------------------- stats
float RunningStats::median() const {
    if (win.empty()) return 0.f;
    std::vector<float> t(win.begin(), win.end());
    const size_t k = t.size() / 2;
    std::nth_element(t.begin(), t.begin() + k, t.end());
    return t[k];
}

float RunningStats::mad() const {
    if (win.empty()) return 1e-6f;
    const float m = median();
    std::vector<float> t;
    t.reserve(win.size());
    for (float v : win) t.push_back(std::fabs(v - m));
    const size_t k = t.size() / 2;
    std::nth_element(t.begin(), t.begin() + k, t.end());
    return std::max(t[k], 1e-6f);
}

// ---------------------------------------------------------------------------- registry
namespace {

// diag_lite() promises flat scalars with at most one level of nesting for a map the
// module cannot enumerate at compile time.  Honour exactly that: one level down, and
// anything deeper is ignored rather than walked, because walking it is how a tool ends up
// quietly serialising an unbounded container it was built to avoid.
template <typename F>
void flatten(const nlohmann::json& j, const std::string& prefix, int depth, F&& emit) {
    if (!j.is_object()) return;
    for (const auto& [k, v] : j.items()) {
        const std::string path = prefix.empty() ? k : prefix + "." + k;
        if (v.is_boolean())      emit(path, v.get<bool>() ? 1.0 : 0.0, true);
        else if (v.is_number())  emit(path, v.get<double>(), false);
        else if (v.is_object() && depth < 1) flatten(v, path, depth + 1, emit);
    }
}

}  // namespace

void SourceRegistry::note_module(const std::string& module, const std::string& type) {
    auto& m = mods_[module];
    if (m.type.empty()) m.type = type;
}

void SourceRegistry::observe(const std::string& module, const std::string& type,
                             const nlohmann::json& snapshot, double dt_s) {
    auto& m = mods_[module];
    if (m.type.empty()) m.type = type;
    ++m.frames;
    flatten(snapshot, "", 0, [&](const std::string& key, double val, bool is_bool) {
        SourceState& s = m.keys[key];
        s.is_bool = is_bool;
        if (s.seen > 0) {
            s.prev      = s.last;
            s.have_prev = true;
            s.delta     = dt_s > 1e-9 ? (val - s.prev) / dt_s : 0.0;
            s.dstats.push(float(s.delta));
        }
        s.last = val;
        ++s.seen;
        // Welford: what auto_patch reads to decide which key a module is "about" when it
        // publishes no named error signal.
        const double d = val - s.mean;
        s.mean += d / double(s.seen);
        s.m2   += d * (val - s.mean);
        s.stats.push(float(val));
        if (!s.have_ext) { s.lo = s.hi = val; s.have_ext = true; }
        else { s.lo = std::min(s.lo, val); s.hi = std::max(s.hi, val); }
    });
}

const SourceState* SourceRegistry::find(const SourceRef& s) const {
    auto mi = mods_.find(s.module);
    if (mi == mods_.end()) return nullptr;
    auto ki = mi->second.keys.find(s.key);
    return ki == mi->second.keys.end() ? nullptr : &ki->second;
}

SourceState* SourceRegistry::find(const SourceRef& s) {
    return const_cast<SourceState*>(static_cast<const SourceRegistry*>(this)->find(s));
}

std::vector<ObservedModule> SourceRegistry::observed() const {
    std::vector<ObservedModule> out;
    out.reserve(mods_.size());
    for (const auto& [id, m] : mods_) {
        ObservedModule om;
        om.module = id;
        om.type   = m.type;
        for (const auto& [k, s] : m.keys)
            om.sources.push_back(ObservedSource{k, s.is_bool, s.last, s.mean, s.variance(), s.seen});
        out.push_back(std::move(om));
    }
    return out;
}

nlohmann::json SourceRegistry::to_json() const {
    nlohmann::json mods = nlohmann::json::array();
    for (const auto& [id, m] : mods_) {
        nlohmann::json keys = nlohmann::json::array();
        for (const auto& [k, s] : m.keys) {
            keys.push_back({{"key", k},
                            {"is_bool", s.is_bool},
                            {"value", s.last},
                            {"delta", s.delta},
                            {"median", s.stats.ready() ? double(s.stats.median()) : s.mean},
                            {"mad", s.stats.ready() ? double(s.stats.mad()) : 0.0},
                            {"min", s.lo},
                            {"max", s.hi},
                            {"mean", s.mean},
                            {"var", s.variance()},
                            {"seen", s.seen}});
        }
        // `frames > 0 && keys empty` is the honest report of a module that has no
        // diag_lite() override: it is being listened to and it is saying nothing.
        mods.push_back({{"module", id},
                        {"type", m.type},
                        {"frames", m.frames},
                        {"keys", std::move(keys)}});
    }
    return mods;
}

// ---------------------------------------------------------------------------- engine
void Engine::set_patch(const Patch& p) {
    std::lock_guard<std::mutex> lk(mtx_);
    patch_ = p;
    resize_runtime_locked();
}

Patch Engine::patch() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return patch_;
}

bool Engine::apply_ops_locked(const nlohmann::json& ops, std::string& err) {
    if (!apply_ops(patch_, ops, err)) return false;
    resize_runtime_locked();
    return true;
}

void Engine::resize_runtime_locked() {
    // Preserve per-voice runtime across a patch edit, matched by voice id: an operator
    // dragging a slider must not hear the oscillator restart, and the running stats a
    // route depends on take ten seconds to refill.
    std::vector<VoiceRT> next(patch_.voices.size());
    for (size_t i = 0; i < patch_.voices.size(); ++i) {
        const auto& cfg = patch_.voices[i];
        for (size_t j = 0; j < voices_.size() && j < prev_ids_.size(); ++j) {
            if (prev_ids_[j] == cfg.id) { next[i] = std::move(voices_[j]); break; }
        }
        // Seed per voice so two noise oscillators are not the same noise; only meaningful
        // for a freshly created voice, since a re-matched one keeps its own generator.
        if (next[i].oscil.rng.s == Rng().s)
            next[i].oscil.rng = Rng(uint32_t(0x9e3779b9u + 2654435761u * uint32_t(i + 1)));
        next[i].routes.resize(cfg.routes.size());
        next[i].events.resize(cfg.events.size());
    }
    voices_ = std::move(next);
    prev_ids_.clear();
    for (const auto& v : patch_.voices) prev_ids_.push_back(v.id);
    master_routes_.resize(patch_.master.routes.size());
}

double Engine::normalise_locked(const Route& r, const SourceState& s, const SourceRef& ref) {
    const double x = s.last;
    switch (r.norm.mode) {
        case NormMode::MedianMad: {
            if (!s.stats.ready()) return 0.0;
            const double med = s.stats.median();
            const double mad = std::max(double(s.stats.mad()), 1e-9);
            const double z   = (x - med) / mad;
            const double lo = r.norm.z_lo, hi = r.norm.z_hi;
            if (hi - lo < 1e-9) return 0.0;
            return std::clamp((z - lo) / (hi - lo), 0.0, 1.0);
        }
        case NormMode::ThresholdRatio: {
            // Against a threshold the module publishes about itself where there is one —
            // that is the difference between "this number is big" and "this module
            // considers itself surprised", and only the second one deserves to be loud.
            double thr = 0.0;
            if (!r.norm.ref_key.empty()) {
                const SourceState* t = reg_.find(SourceRef{ref.module, r.norm.ref_key});
                thr = t ? t->last : 0.0;
            } else {
                if (!s.stats.ready()) return 0.0;
                thr = double(s.stats.median()) + double(s.stats.mad());
            }
            if (!(std::fabs(thr) > 1e-12)) return 0.0;
            const double ratio = x / thr;
            const double gate  = r.norm.gate;
            const double full  = std::max(r.norm.full, gate + 0.1);
            return std::clamp((ratio - gate) / (full - gate), 0.0, 1.0);
        }
        case NormMode::MinMax: {
            if (!s.have_ext || (s.hi - s.lo) < 1e-12) return 0.0;
            return std::clamp((x - s.lo) / (s.hi - s.lo), 0.0, 1.0);
        }
        case NormMode::Delta: {
            // A counter's VALUE is a ramp and makes a boring modulator; its RATE is where
            // the event is.  Same z-score knobs so the two modes read alike in the UI.
            if (!s.dstats.ready()) return 0.0;
            const double med = s.dstats.median();
            const double mad = std::max(double(s.dstats.mad()), 1e-9);
            const double z   = (s.delta - med) / mad;
            const double lo = r.norm.z_lo, hi = r.norm.z_hi;
            if (hi - lo < 1e-9) return 0.0;
            return std::clamp((z - lo) / (hi - lo), 0.0, 1.0);
        }
        case NormMode::Raw: {
            const double lo = r.norm.in_lo, hi = r.norm.in_hi;
            if (std::fabs(hi - lo) < 1e-12) return 0.0;
            return std::clamp((x - lo) / (hi - lo), 0.0, 1.0);
        }
    }
    return 0.0;
}

double Engine::eval_route_locked(const Route& r, RouteRT& rt, double dt_s) {
    if (!r.enabled || r.source.empty()) { rt.last_norm = rt.last_out = 0.0; return 0.0; }
    const SourceState* s = reg_.find(r.source);
    if (!s || s->seen == 0) { rt.last_norm = rt.last_out = 0.0; return 0.0; }

    double n = normalise_locked(r, *s, r.source);

    if (r.norm.smooth_ms > 0.5 && dt_s > 1e-9) {
        const double a = 1.0 - std::exp(-dt_s / (r.norm.smooth_ms / 1000.0));
        if (!rt.have) { rt.smoothed = n; rt.have = true; }
        else          { rt.smoothed += a * (n - rt.smoothed); }
        n = rt.smoothed;
    } else {
        rt.smoothed = n;
        rt.have     = true;
    }

    double shaped = std::pow(std::clamp(n, 0.0, 1.0), std::max(0.01, r.curve));
    if (r.invert) shaped = 1.0 - shaped;
    rt.last_norm = shaped;
    rt.last_out  = shaped * r.depth;
    return rt.last_out;
}

void Engine::eval_events_locked(const VoiceCfg& cfg, VoiceRT& rt) {
    for (size_t i = 0; i < cfg.events.size() && i < rt.events.size(); ++i) {
        const EventRoute& e  = cfg.events[i];
        EventRT&          er = rt.events[i];
        if (e.source.empty()) continue;
        const SourceState* s = reg_.find(e.source);
        if (!s || s->seen == 0) continue;
        const double x = s->last;

        bool fire = false;
        if (er.have) {
            switch (e.trigger) {
                case Trigger::Rise:     fire = (x > 0.5 && er.prev <= 0.5); break;
                case Trigger::Fall:     fire = (x < 0.5 && er.prev >= 0.5); break;
                // Level, not edge: baked_now is already a one-tick pulse, so an edge test
                // would need it to go false between two bakes to fire the second one.
                case Trigger::True:     fire = (x > 0.5); break;
                case Trigger::Increase: fire = (x > er.prev + 1e-9); break;
                case Trigger::Decrease: fire = (x < er.prev - 1e-9); break;
            }
        }
        er.prev = x;
        er.have = true;

        if (!fire || !e.enabled || e.sound == "none") continue;
        const auto& names = event_sound_names();
        for (size_t k = 0; k < names.size(); ++k)
            if (names[k] == e.sound) { rt.chirp_kind = int(k); rt.chirp_t = 0.0; break; }
    }
}

void Engine::on_frame(const std::string& module, const std::string& type,
                      const nlohmann::json& snapshot, double dt_s) {
    std::lock_guard<std::mutex> lk(mtx_);
    reg_.observe(module, type, snapshot, dt_s);

    if (voices_.size() != patch_.voices.size()) resize_runtime_locked();

    const auto& degrees = scale_degrees(patch_.master.scale);

    // Every route is re-evaluated on every frame regardless of which module it came from:
    // a route may read any module, and the cost is a few dozen scalars against a diag
    // stream that arrives tens of times a second.
    for (size_t vi = 0; vi < patch_.voices.size(); ++vi) {
        const VoiceCfg& cfg = patch_.voices[vi];
        VoiceRT&        rt  = voices_[vi];
        if (rt.routes.size() != cfg.routes.size()) rt.routes.resize(cfg.routes.size());
        if (rt.events.size() != cfg.events.size()) rt.events.resize(cfg.events.size());

        double semis = 0, detune = 0, amp = 0, level = 0, cut = 0, qq = 0, pw = 0, nz = 0,
               morph = 0, pan = 0;
        for (size_t ri = 0; ri < cfg.routes.size(); ++ri) {
            const double v = eval_route_locked(cfg.routes[ri], rt.routes[ri], dt_s);
            switch (cfg.routes[ri].dest) {
                case Dest::Pitch:      semis  += v; break;
                case Dest::Detune:     detune += v; break;
                case Dest::Amp:        amp    += v; break;
                case Dest::Level:      level  += v; break;
                case Dest::Cutoff:     cut    += v; break;
                case Dest::Resonance:  qq     += v; break;
                case Dest::PulseWidth: pw     += v; break;
                case Dest::NoiseMix:   nz     += v; break;
                case Dest::VowelMorph: morph  += v; break;
                case Dest::Pan:        pan    += v; break;
            }
        }

        // Quantise with hysteresis: a note may only change once the underlying pitch has
        // moved a clear half-step past where it sits, or the mix warbles on tick jitter.
        const bool quant = cfg.osc.quantize < 0 ? patch_.master.quantize : (cfg.osc.quantize > 0);
        float sem = float(semis);
        if (quant) {
            const float q = quantize_semis(sem, degrees);
            if (std::fabs(q - rt.note_semis) >= 1.f || std::fabs(sem - rt.note_semis) > 1.6f)
                rt.note_semis = q;
            sem = rt.note_semis;
        }
        sem += float(detune);

        rt.t_hz     = cfg.osc.base_hz * std::pow(2.0, sem / 12.0);
        rt.t_amp    = cfg.enabled ? std::clamp(amp, 0.0, 1.0) : 0.0;
        rt.t_level  = std::clamp(cfg.osc.level + level, 0.0, 4.0);
        rt.t_cutoff = std::clamp(cfg.filter.cutoff_hz * std::pow(2.0, cut / 12.0), 10.0, sr_ * 0.45);
        rt.t_q      = std::clamp(cfg.filter.q + qq, 0.35, 40.0);
        rt.t_pw     = std::clamp(cfg.osc.pulse_width + pw, 0.02, 0.98);
        rt.t_noise  = std::clamp(cfg.osc.noise_mix + nz, 0.0, 1.0);
        rt.t_morph  = std::clamp(cfg.filter.morph + morph, 0.0, 1.0);
        rt.t_pan    = std::clamp(cfg.osc.pan + pan, -1.0, 1.0);

        eval_events_locked(cfg, rt);
    }

    // The master rack: only the destinations that mean something on a summed bus.
    if (master_routes_.size() != patch_.master.routes.size())
        master_routes_.resize(patch_.master.routes.size());
    double mcut = 0, mq = 0, mmorph = 0, mlevel = 0;
    for (size_t i = 0; i < patch_.master.routes.size(); ++i) {
        const double v = eval_route_locked(patch_.master.routes[i], master_routes_[i], dt_s);
        switch (patch_.master.routes[i].dest) {
            case Dest::Cutoff:     mcut   += v; break;
            case Dest::Resonance:  mq     += v; break;
            case Dest::VowelMorph: mmorph += v; break;
            case Dest::Level:      mlevel += v; break;
            default: break;
        }
    }
    m_cutoff = std::clamp(patch_.master.filter.cutoff_hz * std::pow(2.0, mcut / 12.0), 10.0, sr_ * 0.45);
    m_q      = std::clamp(patch_.master.filter.q + mq, 0.35, 40.0);
    m_morph  = std::clamp(patch_.master.filter.morph + mmorph, 0.0, 1.0);
    m_level  = std::clamp(1.0 + mlevel, 0.0, 4.0);
}

// ---------------------------------------------------------------------------- audio
namespace {

// Each event sound is a short pitch gesture over the voice's own base, so it lands in the
// same register as the voice that produced it and the ear attributes it correctly.
struct ChirpShape { double dur; };
ChirpShape chirp_shape(int kind) {
    switch (kind) {
        case 0: return {0.08};    // chirp_up
        case 1: return {0.08};    // chirp_down
        case 2: return {0.16};    // two_notes
        case 3: return {0.06};    // blip_down
        case 4: return {0.012};   // click
        default: return {0.0};
    }
}
double chirp_hz(int kind, double base, double u) {
    switch (kind) {
        case 0: return base * 2.0 * std::pow(2.0, u);            // rising octave
        case 1: return base * 2.0 * std::pow(2.0, 1.0 - u);      // falling octave
        case 2: return base * (u < 0.5 ? 3.0 : 4.0);             // two notes
        case 3: return base * 0.5;                               // low blip
        case 4: return base * 8.0;                               // click
        default: return base;
    }
}

}  // namespace

void Engine::render(int16_t* out, int frames) {
    std::lock_guard<std::mutex> lk(mtx_);
    const double dt = 1.0 / double(sr_);

    if (int(mixL_.size()) < frames) { mixL_.resize(frames); mixR_.resize(frames); }
    std::fill(mixL_.begin(), mixL_.begin() + frames, 0.f);
    std::fill(mixR_.begin(), mixR_.begin() + frames, 0.f);

    const bool tone = tone_.load();

    for (size_t vi = 0; vi < patch_.voices.size() && vi < voices_.size(); ++vi) {
        const VoiceCfg& cfg = patch_.voices[vi];
        VoiceRT&        rt  = voices_[vi];
        if (!cfg.enabled) continue;

        const double glide   = std::exp(-dt / std::max(0.001, cfg.osc.glide_ms   / 1000.0));
        const double attack  = std::exp(-dt / std::max(0.001, cfg.osc.attack_ms  / 1000.0));
        const double release = std::exp(-dt / std::max(0.001, cfg.osc.release_ms / 1000.0));

        const FilterMode fmode = cfg.filter.enabled ? cfg.filter.mode : FilterMode::Bypass;
        const Formants   fm    = vowel_lerp(vowel_table(cfg.filter.vowel_a),
                                            vowel_table(cfg.filter.vowel_b), rt.t_morph);

        // Equal-power pan: a voice swept across the field keeps its apparent loudness.
        const double th = (rt.t_pan + 1.0) * 0.25 * M_PI;
        const double gl = std::cos(th), gr = std::sin(th);

        if (rt.hz_now <= 0.0) rt.hz_now = rt.t_hz;

        for (int i = 0; i < frames; ++i) {
            rt.hz_now = rt.t_hz + (rt.hz_now - rt.t_hz) * glide;
            const double ta = tone ? rt.t_amp : 0.0;
            rt.amp_now = ta > rt.amp_now ? ta + (rt.amp_now - ta) * attack
                                         : ta + (rt.amp_now - ta) * release;

            double hz  = rt.hz_now;
            double amp = rt.amp_now;

            if (rt.chirp_t >= 0.0) {
                const ChirpShape cs = chirp_shape(rt.chirp_kind);
                if (cs.dur <= 0.0) { rt.chirp_t = -1.0; }
                else {
                    const double u = rt.chirp_t / cs.dur;
                    hz  = chirp_hz(rt.chirp_kind, cfg.osc.base_hz, u);
                    amp = std::max(amp, 0.45 * (1.0 - u));
                    rt.chirp_t += dt;
                    if (rt.chirp_t > cs.dur) rt.chirp_t = -1.0;
                }
            }

            float s = rt.oscil.render(cfg.osc.waveform, hz, dt, float(rt.t_pw), float(rt.t_noise));
            s = rt.filt.process(s, fmode, rt.t_cutoff, rt.t_q, fm, double(sr_),
                                float(cfg.filter.mix));
            const float g = float(amp * rt.t_level);
            mixL_[i] += s * g * float(gl);
            mixR_[i] += s * g * float(gr);
        }
    }

    // Master bus: filter the sum, then level, then one soft clip at the very end so a
    // loud moment compresses rather than tearing.
    const FilterMode mmode = patch_.master.filter.enabled ? patch_.master.filter.mode
                                                          : FilterMode::Bypass;
    const Formants mfm = vowel_lerp(vowel_table(patch_.master.filter.vowel_a),
                                    vowel_table(patch_.master.filter.vowel_b), m_morph);
    const double vol = (mute_.load() ? 0.0 : patch_.master.volume) * m_level * 0.25;

    double peak = 0.0;
    for (int i = 0; i < frames; ++i) {
        float l = master_filt_.process(mixL_[i], mmode, m_cutoff, m_q, mfm, double(sr_),
                                       float(patch_.master.filter.mix));
        float r = master_filt_r_.process(mixR_[i], mmode, m_cutoff, m_q, mfm, double(sr_),
                                         float(patch_.master.filter.mix));
        l = std::tanh(l * float(vol));
        r = std::tanh(r * float(vol));
        peak = std::max(peak, double(std::max(std::fabs(l), std::fabs(r))));
        out[2 * i]     = int16_t(std::clamp(l, -1.f, 1.f) * 32767.f);
        out[2 * i + 1] = int16_t(std::clamp(r, -1.f, 1.f) * 32767.f);
    }
    // Decay rather than replace, so a meter polled at 15 Hz still catches a transient
    // that happened between polls.
    master_peak_ = std::max(peak, master_peak_ * 0.85);
}

// ---------------------------------------------------------------------------- studio
nlohmann::json Engine::sources_json() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return reg_.to_json();
}

std::vector<ObservedModule> Engine::observed() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return reg_.observed();
}

nlohmann::json Engine::state_json() const {
    std::lock_guard<std::mutex> lk(mtx_);

    nlohmann::json voices = nlohmann::json::array();
    for (size_t i = 0; i < patch_.voices.size() && i < voices_.size(); ++i) {
        const auto& cfg = patch_.voices[i];
        const auto& rt  = voices_[i];
        nlohmann::json routes = nlohmann::json::array();
        for (size_t r = 0; r < rt.routes.size(); ++r)
            routes.push_back({{"norm", rt.routes[r].last_norm}, {"out", rt.routes[r].last_out}});
        voices.push_back({{"id", cfg.id},
                          {"module", cfg.module},
                          {"hz", rt.t_hz},
                          {"note", hz_to_note(float(rt.t_hz))},
                          {"amp", rt.t_amp},
                          {"amp_now", rt.amp_now},
                          {"level", rt.t_level},
                          {"cutoff", rt.t_cutoff},
                          {"q", rt.t_q},
                          {"pulse_width", rt.t_pw},
                          {"noise_mix", rt.t_noise},
                          {"vowel_morph", rt.t_morph},
                          {"pan", rt.t_pan},
                          {"routes", std::move(routes)}});
    }

    nlohmann::json mroutes = nlohmann::json::array();
    for (const auto& r : master_routes_)
        mroutes.push_back({{"norm", r.last_norm}, {"out", r.last_out}});

    // The live value of every source rides along with the meters: the studio's tree wants
    // to update at the same rate as everything else, and a second request for it would
    // just be two round trips where one will do.
    nlohmann::json srcs = nlohmann::json::array();
    for (const auto& om : reg_.observed()) {
        nlohmann::json keys = nlohmann::json::object();
        for (const auto& s : om.sources) keys[s.key] = s.last;
        srcs.push_back({{"module", om.module}, {"type", om.type}, {"values", std::move(keys)}});
    }

    return {{"voices", std::move(voices)},
            {"master", {{"routes", std::move(mroutes)},
                        {"cutoff", m_cutoff},
                        {"q", m_q},
                        {"vowel_morph", m_morph},
                        {"level", m_level},
                        {"peak", master_peak_},
                        {"volume", patch_.master.volume},
                        {"muted", mute_.load()}}},
            {"sources", std::move(srcs)}};
}

}  // namespace xv
