// patch.hpp — what the synth IS, as data.
//
// A patch is the whole instrument: which brain signals are sources, which oscillator each
// drives, through what normalisation, into which synthesis destination.  It is the thing
// the studio edits, the thing `--config` loads, and the only thing that has to be right
// for a given brain to have a voice of its own.
//
// The design turns on one observation: every signal the brain publishes has a DIFFERENT
// SCALE, and most of them nobody has ever looked at.  `last_tle` lives near 0.1, `nodes`
// counts to a few hundred, `upright` sits at 1.0 and dips, `dopamine` wanders around a
// baseline.  A depth slider is meaningless until the source has been mapped to 0..1, so
// normalisation is a first-class part of every route rather than a global assumption —
// that is what lets the same studio tune a picrawler, a cell, and whatever comes next.
#pragma once

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

#include "dsp.hpp"

namespace xv {

using json = nlohmann::json;

// ---------------------------------------------------------------------------- enums
// A source is addressed by module id plus a DOTTED key, because a module may publish a
// map whose entries it cannot name at compile time (LateralVoter's per-modality trust
// weights, HomeostaticDrive's channels).  "voter" + "trust.video" is one source.
struct SourceRef {
    std::string module;
    std::string key;
    bool empty() const { return module.empty() || key.empty(); }
    std::string label() const { return module + "." + key; }
    bool operator==(const SourceRef& o) const { return module == o.module && key == o.key; }
};

enum class Dest { Pitch, Amp, Level, Cutoff, Resonance, PulseWidth, NoiseMix, VowelMorph, Pan, Detune };

inline const std::vector<std::string>& dest_names() {
    static const std::vector<std::string> n = {"pitch",      "amp",       "level",
                                               "cutoff",     "resonance", "pulse_width",
                                               "noise_mix",  "vowel_morph", "pan", "detune"};
    return n;
}
inline bool dest_from_name(const std::string& s, Dest& out) {
    const auto& n = dest_names();
    for (size_t i = 0; i < n.size(); ++i)
        if (n[i] == s) { out = Dest(i); return true; }
    return false;
}
inline const char* dest_name(Dest d) { return dest_names()[size_t(d)].c_str(); }

// The unit a route's `depth` is measured in, which is what makes one slider legible
// across ten destinations.
inline const char* dest_unit(Dest d) {
    switch (d) {
        case Dest::Pitch:  case Dest::Detune:     return "semitones";
        case Dest::Cutoff:                        return "semitones";
        case Dest::Resonance:                     return "Q";
        default:                                  return "0..1";
    }
}

enum class NormMode { MedianMad, ThresholdRatio, MinMax, Delta, Raw };

inline const std::vector<std::string>& norm_mode_names() {
    static const std::vector<std::string> n = {"median_mad", "threshold_ratio", "minmax",
                                               "delta", "raw"};
    return n;
}
inline bool norm_mode_from_name(const std::string& s, NormMode& out) {
    const auto& n = norm_mode_names();
    for (size_t i = 0; i < n.size(); ++i)
        if (n[i] == s) { out = NormMode(i); return true; }
    return false;
}
inline const char* norm_mode_name(NormMode m) { return norm_mode_names()[size_t(m)].c_str(); }

enum class Trigger { Rise, Fall, True, Increase, Decrease };

inline const std::vector<std::string>& trigger_names() {
    static const std::vector<std::string> n = {"rise", "fall", "true", "increase", "decrease"};
    return n;
}
inline bool trigger_from_name(const std::string& s, Trigger& out) {
    const auto& n = trigger_names();
    for (size_t i = 0; i < n.size(); ++i)
        if (n[i] == s) { out = Trigger(i); return true; }
    return false;
}
inline const char* trigger_name(Trigger t) { return trigger_names()[size_t(t)].c_str(); }

inline const std::vector<std::string>& event_sound_names() {
    static const std::vector<std::string> n = {"chirp_up", "chirp_down", "two_notes",
                                               "blip_down", "click", "none"};
    return n;
}

// ---------------------------------------------------------------------------- config
struct Norm {
    NormMode mode = NormMode::MedianMad;
    // median_mad: z = (x - median) / MAD, then z_lo..z_hi maps to 0..1.
    double z_lo = 0.0, z_hi = 4.0;
    // threshold_ratio: r = x / <ref_key>, then gate..full maps to 0..1.  ref_key is a key
    // on the SAME module (e.g. novelty_threshold_now); empty means median + 1 MAD.
    std::string ref_key;
    double gate = 1.4, full = 2.0;
    // raw: explicit input range.
    double in_lo = 0.0, in_hi = 1.0;
    // shared: smoothing on the normalised value, and the window minmax/delta run over.
    double smooth_ms = 60.0;
    double window_s  = 10.0;
};

struct Route {
    SourceRef source;
    Dest      dest    = Dest::Pitch;
    Norm      norm;
    double    depth   = 1.0;     // in dest_unit(dest)
    double    curve   = 1.0;     // x^curve on the normalised 0..1; <1 opens up the quiet end
    bool      invert  = false;
    bool      enabled = true;
};

struct EventRoute {
    SourceRef   source;
    Trigger     trigger = Trigger::Increase;
    std::string sound   = "chirp_up";
    bool        enabled = true;
};

struct FilterCfg {
    bool        enabled   = false;
    FilterMode  mode      = FilterMode::LowPass;
    double      cutoff_hz = 4000.0;
    double      q         = 0.7;
    double      mix       = 1.0;
    std::string vowel_a   = "A";
    std::string vowel_b   = "E";
    double      morph     = 0.0;
};

struct OscCfg {
    Wave   waveform    = Wave::Square;
    double pulse_width = 0.5;
    double noise_mix   = 0.0;
    double base_hz     = 261.63;
    double level       = 1.0;
    double pan         = 0.0;      // -1 L .. +1 R (mono output sums; kept for the Pi's stereo HAT)
    double glide_ms    = 30.0;
    double attack_ms   = 20.0;
    double release_ms  = 150.0;
    int    quantize    = -1;       // -1 follow master, 0 off, 1 on
};

struct VoiceCfg {
    std::string             id;
    std::string             module;
    bool                    enabled = true;
    OscCfg                  osc;
    FilterCfg               filter;
    std::vector<Route>      routes;
    std::vector<EventRoute> events;
};

struct MasterCfg {
    double      volume   = 0.5;
    bool        quantize = true;
    std::string scale    = "major_pentatonic";
    double      span     = 24.0;   // default pitch depth handed to auto-built routes
    // How fast the control-rate destinations (cutoff, resonance, vowel morph, pulse width,
    // noise mix, pan, level) chase their targets.  Diag frames land at ~30 Hz, so a target
    // used raw moves in visible stair-steps; this is what turns them into a slide.  Pitch
    // and amplitude are NOT smoothed here — they have glide and attack/release per voice.
    // 0 disables it and restores the stair, which is occasionally what a sample-and-hold
    // effect wants.
    double      mod_smooth_ms = 25.0;
    FilterCfg   filter;
    // The master bus gets its own mod rack, so the output filter is modulatable by any
    // source rather than being a static shape.  This is where `vowel` earns its keep: the
    // whole mix speaking one vowel, morphing with a signal that belongs to no single voice
    // — consensus surprise, urgency, dopamine — reads as the brain's mood rather than as
    // any one module's opinion.  Only the master-relevant destinations apply here
    // (cutoff, resonance, vowel_morph, level); the rest are ignored.
    std::vector<Route> routes;
};

struct BrainCfg {
    std::string host = "127.0.0.1";
    int         port = 7400;
    double      hz   = 30.0;
};

struct Patch {
    int                   version = 1;
    BrainCfg              brain;
    MasterCfg             master;
    std::vector<VoiceCfg> voices;
};

// ---------------------------------------------------------------------------- json
json  to_json(const Patch& p);
Patch from_json(const json& j);          // tolerant: any missing field keeps its default

// Partial update, addressed by JSON pointer ("/voices/2/osc/base_hz").  Implemented as
// serialise -> patch -> parse so a live edit can never reach a state the file format
// cannot express, and so there is exactly one place that knows the schema.
bool apply_ops(Patch& p, const json& ops, std::string& err);

// ---------------------------------------------------------------------------- discovery
// What the engine actually saw on the wire, which is the ONLY basis for building a patch:
// a hardcoded key list would go stale the moment a module's diag_lite() changes, and
// would silently mis-describe a brain config that does not have that module at all.
struct ObservedSource {
    std::string key;                       // dotted
    bool        is_bool = false;
    double      last    = 0.0;
    double      mean    = 0.0;
    double      var     = 0.0;
    uint64_t    seen    = 0;
};

struct ObservedModule {
    std::string                 module;
    std::string                 type;
    std::vector<ObservedSource> sources;   // empty => the module has no diag_lite()
};

// Build a complete patch from what was observed.  With the modules the picrawler runs
// today this reproduces the tool's original mapping — one voice per TLE-carrying module,
// TLE to pitch through median/MAD, TLE against the novelty threshold to volume — so
// launching with no --config sounds exactly as it always has.  `vary_timbre` is the one
// departure, and is off by default precisely so that parity holds: it walks the waveforms
// so several voices can be told apart by ear rather than by pitch alone.
Patch auto_patch(const std::vector<ObservedModule>& observed, const Patch& base,
                 bool vary_timbre = false);

}  // namespace xv
