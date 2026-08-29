// dsp.hpp — the synthesis primitives.  No brain headers, no JSON, no I/O: this layer
// is pure arithmetic so it can be unit-tested without a sim, a socket or a sound card.
//
// Everything here is written for MODULATION AT AUDIO RATE, because that is what the brain
// does to it — a TLE spike moves a cutoff in one block.  That constraint picks the
// algorithms: polyBLEP oscillators (a naive square aliases into a buzz the moment pitch
// moves), and a TPT/Zavalishin state-variable filter (a direct-form biquad recomputed
// per sample blows up when its coefficients jump; the TPT topology stays stable).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace xv {

// ---------------------------------------------------------------------------- waveforms
enum class Wave { Sine, Triangle, Saw, Square, Pulse, NoiseWhite, NoisePink };

inline const std::vector<std::string>& wave_names() {
    static const std::vector<std::string> n = {"sine",  "triangle",    "saw", "square",
                                               "pulse", "noise_white", "noise_pink"};
    return n;
}
inline const char* wave_name(Wave w) { return wave_names()[size_t(w)].c_str(); }
inline bool wave_from_name(const std::string& s, Wave& out) {
    const auto& n = wave_names();
    for (size_t i = 0; i < n.size(); ++i)
        if (n[i] == s) { out = Wave(i); return true; }
    return false;
}

// ---------------------------------------------------------------------------- noise
// xorshift32: deterministic per voice, and cheap enough to run unconditionally.
struct Rng {
    uint32_t s = 0x9e3779b9u;
    explicit Rng(uint32_t seed = 0x9e3779b9u) : s(seed ? seed : 0x9e3779b9u) {}
    uint32_t next_u32() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    float next_bipolar() { return float(next_u32()) * (2.0f / 4294967296.0f) - 1.0f; }
};

// Paul Kellet's economical pink filter: -3 dB/octave to well under a Hz, six poles.
// Pink reads as "rushing air" where white reads as "hiss" — with the brain's error
// signals driving noise_mix, pink sits under a tone without masking it.
struct PinkFilter {
    float b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
    float process(float white) {
        b0 = 0.99886f * b0 + white * 0.0555179f;
        b1 = 0.99332f * b1 + white * 0.0750759f;
        b2 = 0.96900f * b2 + white * 0.1538520f;
        b3 = 0.86650f * b3 + white * 0.3104856f;
        b4 = 0.55000f * b4 + white * 0.5329522f;
        b5 = -0.7616f * b5 - white * 0.0168980f;
        const float out = b0 + b1 + b2 + b3 + b4 + b5 + b6 + white * 0.5362f;
        b6 = white * 0.115926f;
        return out * 0.11f;                       // roughly unity RMS against the others
    }
};

// polyBLEP: one-sample correction that removes the worst of the aliasing at a hard edge.
inline float poly_blep(double t, double dt) {
    if (dt <= 0.0) return 0.f;
    if (t < dt) { t /= dt; return float(t + t - t * t - 1.0); }
    if (t > 1.0 - dt) { t = (t - 1.0) / dt; return float(t * t + t + t + 1.0); }
    return 0.f;
}

struct Oscillator {
    double     phase = 0.0;
    Rng        rng;
    PinkFilter pink;

    void reset() { phase = 0.0; }

    // One sample.  `hz` and `dt` are per-sample so pitch may move every sample.
    // `pulse_width` applies to Pulse only; `noise_mix` blends noise over ANY waveform,
    // which is why noise is both a waveform in its own right and a modifier of the rest.
    float render(Wave w, double hz, double dt, float pulse_width, float noise_mix) {
        const double dtp = std::clamp(hz * dt, 0.0, 0.5);
        phase += dtp;
        if (phase >= 1.0) phase -= std::floor(phase);

        float tone = 0.f;
        switch (w) {
            case Wave::Sine:
                tone = float(std::sin(2.0 * M_PI * phase));
                break;
            case Wave::Triangle: {
                // Integrating a square would need DC tracking; the direct form is exact
                // and triangle's own aliasing is ~12 dB/oct steeper, so it needs no BLEP.
                const double p = phase < 0.5 ? phase * 2.0 : (1.0 - phase) * 2.0;
                tone = float(p * 2.0 - 1.0);
                break;
            }
            case Wave::Saw:
                tone = float(2.0 * phase - 1.0) - poly_blep(phase, dtp);
                break;
            case Wave::Square:
            case Wave::Pulse: {
                const double duty =
                    (w == Wave::Pulse) ? std::clamp(double(pulse_width), 0.02, 0.98) : 0.5;
                tone = phase < duty ? 1.f : -1.f;
                tone += poly_blep(phase, dtp);
                double t2 = phase + (1.0 - duty);
                if (t2 >= 1.0) t2 -= 1.0;
                tone -= poly_blep(t2, dtp);
                // Remove the DC term.  A pulse of duty d sits at a mean of 2d-1, so a
                // width sweep would otherwise walk the whole voice off centre — and
                // several voices each carrying DC sum into a thump that the master soft
                // clipper then bakes in.  The AC level does fall as the pulse narrows,
                // which is what a narrow pulse should sound like; only the offset is a bug.
                tone -= float(2.0 * duty - 1.0);
                break;
            }
            case Wave::NoiseWhite: tone = rng.next_bipolar(); break;
            case Wave::NoisePink:  tone = pink.process(rng.next_bipolar()); break;
        }

        const float nm = std::clamp(noise_mix, 0.f, 1.f);
        if (nm <= 0.f) return tone;
        const float n = pink.process(rng.next_bipolar());
        return tone * (1.f - nm) + n * nm;
    }
};

// ---------------------------------------------------------------------------- filter
enum class FilterMode { Bypass, LowPass, HighPass, BandPass, Notch, Vowel };

inline const std::vector<std::string>& filter_mode_names() {
    static const std::vector<std::string> n = {"bypass",   "lowpass", "highpass",
                                               "bandpass", "notch",   "vowel"};
    return n;
}
inline bool filter_mode_from_name(const std::string& s, FilterMode& out) {
    const auto& n = filter_mode_names();
    for (size_t i = 0; i < n.size(); ++i)
        if (n[i] == s) { out = FilterMode(i); return true; }
    return false;
}
inline const char* filter_mode_name(FilterMode m) { return filter_mode_names()[size_t(m)].c_str(); }

// Topology-preserving-transform state variable filter (Zavalishin).  All four responses
// fall out of the same two integrators, so switching mode never clicks, and the
// coefficients may be recomputed every sample without the instability a direct-form
// biquad shows under fast cutoff modulation.
struct SVF {
    double ic1eq = 0.0, ic2eq = 0.0;

    void reset() { ic1eq = ic2eq = 0.0; }

    float process(float x, double cutoff_hz, double q, double sr, FilterMode m) {
        if (m == FilterMode::Bypass || m == FilterMode::Vowel) return x;
        // Below Nyquist with margin: tan() runs away as fc approaches sr/2.
        cutoff_hz = std::clamp(cutoff_hz, 10.0, sr * 0.45);
        q         = std::clamp(q, 0.35, 40.0);
        const double g  = std::tan(M_PI * cutoff_hz / sr);
        const double k  = 1.0 / q;
        const double a1 = 1.0 / (1.0 + g * (g + k));
        const double a2 = g * a1;
        const double a3 = g * a2;

        const double v3 = double(x) - ic2eq;
        const double v1 = a1 * ic1eq + a2 * v3;
        const double v2 = ic2eq + a2 * ic1eq + a3 * v3;
        ic1eq = 2.0 * v1 - ic1eq;
        ic2eq = 2.0 * v2 - ic2eq;

        double out = 0.0;
        switch (m) {
            case FilterMode::LowPass:  out = v2; break;
            case FilterMode::HighPass: out = double(x) - k * v1 - v2; break;
            case FilterMode::BandPass: out = v1; break;
            case FilterMode::Notch:    out = double(x) - k * v1; break;
            default:                   out = double(x); break;
        }
        // A resonant filter fed a step can ring past 1.0; the caller soft-clips, but a
        // non-finite state would persist forever, so trap it here where it starts.
        if (!std::isfinite(out) || !std::isfinite(ic1eq) || !std::isfinite(ic2eq)) {
            reset();
            return 0.f;
        }
        return float(out);
    }
};

// ---------------------------------------------------------------------------- vowels
// Three parallel resonators at the first three formants.  This is what makes the brain
// sound like it is saying something rather than beeping: the same TLE that moved pitch
// now moves the mouth, and vowel identity is a continuous axis a route can drive.
struct Formants {
    std::array<double, 3> f{{730, 1090, 2440}};    // formant centres, Hz
    std::array<double, 3> bw{{60, 90, 120}};       // bandwidths, Hz  (Q = f / bw)
    std::array<double, 3> gain{{1.0, 0.45, 0.35}}; // linear
};

inline const std::vector<std::string>& vowel_names() {
    static const std::vector<std::string> n = {"A", "E", "I", "O", "U"};
    return n;
}

// Classic measured formant centres for a neutral (male) vocal tract.  The exact numbers
// matter less than the ratios — F1 low/F2 high reads as "ee", both low as "oo".
inline Formants vowel_table(const std::string& name) {
    Formants v;
    if (name == "E")      { v.f = {{530, 1840, 2480}}; v.bw = {{60, 100, 120}}; v.gain = {{1.0, 0.25, 0.35}}; }
    else if (name == "I") { v.f = {{270, 2290, 3010}}; v.bw = {{60, 100, 120}}; v.gain = {{1.0, 0.18, 0.25}}; }
    else if (name == "O") { v.f = {{570,  840, 2410}}; v.bw = {{60,  90, 120}}; v.gain = {{1.0, 0.45, 0.25}}; }
    else if (name == "U") { v.f = {{300,  870, 2240}}; v.bw = {{60,  90, 120}}; v.gain = {{1.0, 0.25, 0.12}}; }
    else                  { v.f = {{730, 1090, 2440}}; v.bw = {{60,  90, 120}}; v.gain = {{1.0, 0.45, 0.35}}; }  // A
    return v;
}

// Interpolate in the formant domain, not the audio domain: crossfading two vowel banks
// gives a double-voice smear, while sliding the resonances gives the diphthong the ear
// expects when a modulator sweeps the morph.
inline Formants vowel_lerp(const Formants& a, const Formants& b, double t) {
    t = std::clamp(t, 0.0, 1.0);
    Formants o;
    for (int i = 0; i < 3; ++i) {
        o.f[i]    = a.f[i]    + (b.f[i]    - a.f[i])    * t;
        o.bw[i]   = a.bw[i]   + (b.bw[i]   - a.bw[i])   * t;
        o.gain[i] = a.gain[i] + (b.gain[i] - a.gain[i]) * t;
    }
    return o;
}

struct VowelBank {
    std::array<SVF, 3> bp;

    void reset() { for (auto& f : bp) f.reset(); }

    float process(float x, const Formants& fm, double sr) {
        float out = 0.f;
        for (int i = 0; i < 3; ++i) {
            const double q = std::max(0.5, fm.f[i] / std::max(1.0, fm.bw[i]));
            out += float(fm.gain[i]) * bp[i].process(x, fm.f[i], q, sr, FilterMode::BandPass);
        }
        return out;
    }
};

// One filter slot: an SVF for the four ordinary modes and a formant bank for `vowel`.
// Holding both means switching mode is a branch, not a reallocation, so the control
// socket can flip it live.
struct FilterUnit {
    SVF       svf;
    VowelBank vowel;

    void reset() { svf.reset(); vowel.reset(); }

    float process(float x, FilterMode m, double cutoff_hz, double q, const Formants& fm,
                  double sr, float mix) {
        if (m == FilterMode::Bypass) return x;
        const float wet = (m == FilterMode::Vowel) ? vowel.process(x, fm, sr)
                                                   : svf.process(x, cutoff_hz, q, sr, m);
        const float k = std::clamp(mix, 0.f, 1.f);
        return x * (1.f - k) + wet * k;
    }
};

// ---------------------------------------------------------------------------- notes
inline float note_to_hz(const std::string& t) {
    if (t.empty()) return 0.f;
    if (std::isdigit((unsigned char)t[0])) return std::stof(t);
    static const int idx[7] = {9, 11, 0, 2, 4, 5, 7};      // A B C D E F G
    const char L = char(std::toupper((unsigned char)t[0]));
    if (L < 'A' || L > 'G') return 0.f;
    int    semi = idx[L - 'A'];
    size_t i    = 1;
    if (i < t.size() && (t[i] == '#' || t[i] == 's')) { ++semi; ++i; }
    else if (i < t.size() && t[i] == 'b')            { --semi; ++i; }
    const int oct = std::atoi(t.c_str() + i);
    return 440.f * std::pow(2.f, ((oct + 1) * 12 + semi - 69) / 12.f);
}

inline std::string hz_to_note(float hz) {
    static const char* N[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    if (!(hz > 0.f)) return "-";
    const int midi = int(std::lround(69 + 12 * std::log2(hz / 440.f)));
    char b[16];
    std::snprintf(b, sizeof b, "%s%d", N[((midi % 12) + 12) % 12], midi / 12 - 1);
    return b;
}

// Scales for quantised pitch.  Voices an octave apart on a pentatonic form chords by
// construction, which is why it is the default: several modules sounding at once stays
// listenable for the hours an operator actually runs the sim.
inline const std::vector<std::string>& scale_names() {
    static const std::vector<std::string> n = {"chromatic",  "major_pentatonic",
                                               "minor_pentatonic", "major", "minor",
                                               "whole_tone", "octaves"};
    return n;
}
inline const std::vector<int>& scale_degrees(const std::string& s) {
    static const std::vector<int> chromatic = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    static const std::vector<int> majpent   = {0, 2, 4, 7, 9};
    static const std::vector<int> minpent   = {0, 3, 5, 7, 10};
    static const std::vector<int> major     = {0, 2, 4, 5, 7, 9, 11};
    static const std::vector<int> minor     = {0, 2, 3, 5, 7, 8, 10};
    static const std::vector<int> whole     = {0, 2, 4, 6, 8, 10};
    static const std::vector<int> octaves   = {0};
    if (s == "chromatic")        return chromatic;
    if (s == "minor_pentatonic") return minpent;
    if (s == "major")            return major;
    if (s == "minor")            return minor;
    if (s == "whole_tone")       return whole;
    if (s == "octaves")          return octaves;
    return majpent;
}

inline float quantize_semis(float s, const std::vector<int>& degrees) {
    if (degrees.empty()) return s;
    const int   oct = int(std::floor(s / 12.f));
    const float r   = s - oct * 12.f;
    float best = float(degrees[0]), bd = std::fabs(r - float(degrees[0]));
    for (int p : degrees) {
        const float d = std::fabs(r - float(p));
        if (d < bd) { bd = d; best = float(p); }
    }
    if (std::fabs(r - 12.f) < bd) best = 12.f;      // the octave above closes the scale
    return oct * 12.f + best;
}

}  // namespace xv
