// test_xaq_voice.cpp — the parts that can be wrong silently.
//
// No gtest: this tool is a standalone sidecar with three dependencies and adding a fourth
// to assert arithmetic is a poor trade.  A failing CHECK prints file:line and the
// expression, which is all a test of this size needs.
//
// What is worth pinning, and why:
//   * the patch round-trip, because the studio's Save writes what get_patch returned and a
//     lossy field would silently discard an operator's afternoon of tuning;
//   * the filter under modulation, because that is where a synth goes from "wrong" to
//     "damages a speaker" — and the brain modulates cutoff at every tick;
//   * the normalisers, because every one of them is the difference between a source that
//     is expressive and a source that sits at 0 or 1 forever;
//   * DC offset, because several voices each with a DC term sum into a thump the soft
//     clipper then makes permanent.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "control.hpp"
#include "engine.hpp"
#include "patch.hpp"

using namespace xv;

namespace {

int g_fail = 0;
int g_ran  = 0;

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        ++g_ran;                                                                          \
        if (!(cond)) {                                                                    \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                 \
            ++g_fail;                                                                     \
        }                                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                             \
    do {                                                                                  \
        ++g_ran;                                                                          \
        const double _a = (a), _b = (b);                                                  \
        if (!(std::fabs(_a - _b) <= (tol))) {                                             \
            std::printf("  FAIL %s:%d  %s (%g) !~ %s (%g)\n", __FILE__, __LINE__, #a, _a,  \
                        #b, _b);                                                          \
            ++g_fail;                                                                     \
        }                                                                                 \
    } while (0)

void section(const char* name) { std::printf("%s\n", name); }

// ---------------------------------------------------------------------------- patch
void test_patch_roundtrip() {
    section("patch: round-trip is idempotent");
    Patch p;
    p.master.volume   = 0.37;
    p.master.scale    = "minor_pentatonic";
    p.master.quantize = false;
    p.master.filter.enabled = true;
    p.master.filter.mode    = FilterMode::Vowel;
    p.master.filter.vowel_a = "O";
    p.master.filter.vowel_b = "I";
    p.master.filter.morph   = 0.42;

    VoiceCfg v;
    v.id = v.module = "body_pose";
    v.osc.waveform    = Wave::NoisePink;
    v.osc.base_hz     = 523.25;
    v.osc.pulse_width = 0.31;
    v.filter.enabled  = true;
    v.filter.mode     = FilterMode::BandPass;

    Route r;
    r.source = {"body_pose", "last_tle"};
    r.dest   = Dest::VowelMorph;
    r.norm.mode = NormMode::Delta;
    r.norm.z_hi = 3.5;
    r.depth  = 0.8;
    r.curve  = 0.25;
    r.invert = true;
    v.routes.push_back(r);
    v.events.push_back({{"body_pose", "mitosis_count"}, Trigger::Increase, "two_notes", true});
    p.voices.push_back(v);

    Route mr;
    mr.source = {"voter", "fused_tle"};
    mr.dest   = Dest::Cutoff;
    mr.depth  = -12.0;
    p.master.routes.push_back(mr);

    const json j1 = to_json(p);
    const json j2 = to_json(from_json(j1));
    CHECK(j1 == j2);

    // Every field that carries operator intent must survive, named individually: an
    // `==` on the whole blob passes just as happily when both sides drop the same field.
    const Patch b = from_json(j1);
    CHECK_NEAR(b.master.volume, 0.37, 1e-9);
    CHECK(b.master.scale == "minor_pentatonic");
    CHECK(b.master.quantize == false);
    CHECK(b.master.filter.mode == FilterMode::Vowel);
    CHECK(b.master.filter.vowel_b == "I");
    CHECK_NEAR(b.master.filter.morph, 0.42, 1e-9);
    CHECK(b.voices.size() == 1);
    CHECK(b.voices[0].osc.waveform == Wave::NoisePink);
    CHECK_NEAR(b.voices[0].osc.pulse_width, 0.31, 1e-9);
    CHECK(b.voices[0].filter.mode == FilterMode::BandPass);
    CHECK(b.voices[0].routes.size() == 1);
    CHECK(b.voices[0].routes[0].dest == Dest::VowelMorph);
    CHECK(b.voices[0].routes[0].norm.mode == NormMode::Delta);
    CHECK(b.voices[0].routes[0].invert == true);
    CHECK_NEAR(b.voices[0].routes[0].curve, 0.25, 1e-9);
    CHECK(b.voices[0].events.size() == 1);
    CHECK(b.voices[0].events[0].trigger == Trigger::Increase);
    CHECK(b.master.routes.size() == 1);
    CHECK_NEAR(b.master.routes[0].depth, -12.0, 1e-9);
}

void test_patch_defaults_and_tolerance() {
    section("patch: a partial file keeps defaults rather than zeroing them");
    const json partial = {{"version", 1},
                          {"master", {{"volume", 0.8}}},
                          {"voices", json::array({{{"module", "m"}, {"osc", {{"base_hz", "A3"}}}}})}};
    const Patch p = from_json(partial);
    CHECK_NEAR(p.master.volume, 0.8, 1e-9);
    CHECK(p.master.quantize == true);                 // untouched default
    CHECK(p.master.scale == "major_pentatonic");
    CHECK(p.voices.size() == 1);
    CHECK(p.voices[0].id == "m");                     // id falls back to module
    CHECK_NEAR(p.voices[0].osc.base_hz, 220.0, 0.5);  // a NOTE NAME is accepted for base_hz
    CHECK_NEAR(p.voices[0].osc.release_ms, 150.0, 1e-9);

    // Garbage must not take the tool down mid-session.
    const Patch g = from_json(json{{"voices", "not an array"}, {"master", 7}});
    CHECK(g.voices.empty());
    CHECK_NEAR(g.master.volume, 0.5, 1e-9);
}

void test_apply_ops() {
    section("patch: pointer ops for slider drags");
    Patch p;
    VoiceCfg v;
    v.id = v.module = "a";
    Route r;
    r.source = {"a", "last_tle"};
    r.dest   = Dest::Pitch;
    v.routes.push_back(r);
    p.voices.push_back(v);

    std::string err;
    CHECK(apply_ops(p, json::array({{{"path", "/voices/0/osc/base_hz"}, {"value", 440.0}},
                                    {{"path", "/voices/0/routes/0/depth"}, {"value", 7.5}},
                                    {{"path", "/master/filter/mode"}, {"value", "vowel"}}}),
                    err));
    CHECK_NEAR(p.voices[0].osc.base_hz, 440.0, 1e-9);
    CHECK_NEAR(p.voices[0].routes[0].depth, 7.5, 1e-9);
    CHECK(p.master.filter.mode == FilterMode::Vowel);

    // A bad path must report rather than corrupt: the patch is left as it was.
    const double before = p.voices[0].osc.base_hz;
    CHECK(!apply_ops(p, json::array({{{"path", "/voices/99/osc/base_hz"}, {"value", 1.0}}}), err));
    CHECK(!err.empty());
    CHECK_NEAR(p.voices[0].osc.base_hz, before, 1e-9);
    CHECK(!apply_ops(p, json::array({{{"path", "/voices/0/osc/base_hz"}}}), err));
}

// ---------------------------------------------------------------------------- auto
void test_auto_patch() {
    section("auto_patch: built from what was observed");
    std::vector<ObservedModule> obs = {
        {"motor_epm", "MotorEPMv2", {{"motor_tle", false, 0.04, 0.04, 1e-4, 500},
                                     {"upright", false, 0.99, 0.99, 1e-5, 500}}},
        {"body_pose", "EPM",        {{"last_tle", false, 0.11, 0.10, 1e-3, 500},
                                     {"novelty_threshold_now", false, 0.09, 0.09, 1e-6, 500},
                                     {"nodes", false, 47, 40, 25, 500},
                                     {"mitosis_count", false, 3, 2, 1, 500},
                                     {"baked_now", true, 0, 0, 0, 500}}},
        {"silent", "ActionGate", {}},
    };
    const Patch p = auto_patch(obs, Patch{});

    // A module that publishes nothing gets no voice at all: an oscillator that can never
    // sound is worse than an absence, because nothing reports it.
    CHECK(p.voices.size() == 2);
    CHECK(p.voices[0].module == "motor_epm");
    CHECK(p.voices[1].module == "body_pose");

    // The octave ladder is unchanged, and motor_epm staying LOWEST is what lets the ear
    // separate the mix.
    CHECK_NEAR(p.voices[0].osc.base_hz, 130.81, 0.01);
    CHECK_NEAR(p.voices[1].osc.base_hz, 261.63, 0.01);
    CHECK(p.voices[0].osc.waveform == Wave::Square);      // parity: no timbre variation

    // The named error signal wins over the higher-variance key.
    CHECK(p.voices[0].routes.size() == 2);
    CHECK(p.voices[0].routes[0].source.key == "motor_tle");
    CHECK(p.voices[0].routes[0].dest == Dest::Pitch);
    CHECK(p.voices[0].routes[1].dest == Dest::Amp);

    // MotorEPMv2 publishes no threshold, so the amp route falls back to median + 1 MAD;
    // the EPM publishes one, so it is used.
    CHECK(p.voices[0].routes[1].norm.ref_key.empty());
    CHECK(p.voices[1].routes[1].norm.ref_key == "novelty_threshold_now");
    CHECK_NEAR(p.voices[1].routes[1].norm.gate, 1.4, 1e-9);
    CHECK_NEAR(p.voices[1].routes[1].norm.full, 2.0, 1e-9);
    CHECK_NEAR(p.voices[1].routes[1].curve, 0.5, 1e-9);
    CHECK_NEAR(p.voices[1].routes[0].depth, 24.0, 1e-9);

    // GNG life events are bound only where the module actually publishes them.
    CHECK(p.voices[0].events.empty());
    CHECK(p.voices[1].events.size() == 3);

    const Patch v = auto_patch(obs, Patch{}, true);
    CHECK(v.voices[0].osc.waveform != v.voices[1].osc.waveform);
}

// ---------------------------------------------------------------------------- dsp
void test_waveform_dc() {
    section("dsp: no waveform carries a DC offset");
    const double sr = 48000.0, dt = 1.0 / sr, hz = 100.0;
    const int    n  = 48000;                       // exactly 100 periods
    for (Wave w : {Wave::Sine, Wave::Triangle, Wave::Saw, Wave::Square}) {
        Oscillator o;
        double     sum = 0.0, peak = 0.0;
        for (int i = 0; i < n; ++i) {
            const float s = o.render(w, hz, dt, 0.5f, 0.f);
            sum += s;
            peak = std::max(peak, double(std::fabs(s)));
        }
        CHECK_NEAR(sum / n, 0.0, 0.02);
        CHECK(peak > 0.5);                          // it actually made a sound
        CHECK(peak < 2.0);                          // and did not blow up
    }
    Oscillator noise;
    double sum = 0.0;
    for (int i = 0; i < n; ++i) sum += noise.render(Wave::NoiseWhite, hz, dt, 0.5f, 0.f);
    CHECK_NEAR(sum / n, 0.0, 0.05);
}

void test_pulse_width_is_centred() {
    section("dsp: pulse width sweeps without walking off centre");
    const double sr = 48000.0, dt = 1.0 / sr;
    auto measure = [&](float pw, double& mean, double& rms) {
        Oscillator o;
        double     sum = 0, acc = 0;
        for (int i = 0; i < 48000; ++i) {
            const float s = o.render(Wave::Pulse, 200.0, dt, pw, 0.f);
            sum += s;
            acc += double(s) * s;
        }
        mean = sum / 48000.0;
        rms  = std::sqrt(acc / 48000.0);
    };
    double m, r, m_wide = 0, r_wide = 0;
    // The offset is the bug, at every width: several voices each carrying DC sum into a
    // thump, and the master soft clipper then makes it permanent.
    for (float pw : {0.5f, 0.3f, 0.12f, 0.85f}) {
        measure(pw, m, r);
        CHECK_NEAR(m, 0.0, 0.03);
        if (pw == 0.5f) { m_wide = m; r_wide = r; }
    }
    (void)m_wide;
    // The AC level DOES fall as the pulse narrows, and should: that is what a narrow pulse
    // sounds like.  The bar is that it stays audible rather than collapsing.
    measure(0.12f, m, r);
    CHECK(r > r_wide * 0.4);
    CHECK(r < r_wide * 1.1);
}

void test_filter_stability_under_modulation() {
    section("dsp: the filter survives a full cutoff sweep at max Q");
    const double sr = 48000.0;
    for (FilterMode m : {FilterMode::LowPass, FilterMode::HighPass, FilterMode::BandPass,
                         FilterMode::Notch}) {
        SVF    f;
        double peak = 0.0;
        for (int i = 0; i < 96000; ++i) {
            // Sweep the whole audible range twice a second while feeding full-scale noise
            // — harsher than anything a brain signal will do to it.
            const double u  = double(i % 24000) / 24000.0;
            const double fc = 20.0 * std::pow(1000.0, u);
            const float  x  = (i % 2) ? 1.f : -1.f;
            const float  y  = f.process(x, fc, 40.0, sr, m);
            CHECK_NEAR(std::isfinite(y) ? 0.0 : 1.0, 0.0, 0.5);
            peak = std::max(peak, double(std::fabs(y)));
        }
        // A Q of 40 legitimately rings well above unity; the bar is "bounded", not "quiet".
        CHECK(peak < 200.0);
    }
}

void test_filter_shapes() {
    section("dsp: lowpass and highpass actually pass the right end");
    const double sr = 48000.0, dt = 1.0 / sr;
    auto rms_at = [&](FilterMode m, double cutoff, double tone_hz) {
        SVF        f;
        Oscillator o;
        double     acc = 0;
        for (int i = 0; i < 24000; ++i) {
            const float x = o.render(Wave::Sine, tone_hz, dt, 0.5f, 0.f);
            const float y = f.process(x, cutoff, 0.707, sr, m);
            if (i > 4000) acc += double(y) * y;       // skip the settling transient
        }
        return std::sqrt(acc / 20000.0);
    };
    CHECK(rms_at(FilterMode::LowPass,  1000.0, 100.0)  > 0.5);
    CHECK(rms_at(FilterMode::LowPass,  1000.0, 8000.0) < 0.1);
    CHECK(rms_at(FilterMode::HighPass, 1000.0, 100.0)  < 0.1);
    CHECK(rms_at(FilterMode::HighPass, 1000.0, 8000.0) > 0.5);
    CHECK(rms_at(FilterMode::BandPass, 1000.0, 1000.0) >
          rms_at(FilterMode::BandPass, 1000.0, 8000.0));
}

void test_vowel_bank() {
    section("dsp: vowels differ, and the morph moves between them");
    const Formants a = vowel_table("A"), i = vowel_table("I");
    CHECK(a.f[0] > i.f[0]);          // "ah" has a high F1, "ee" a low one
    CHECK(i.f[1] > a.f[1]);          // and "ee" a much higher F2 — that IS the distinction
    const Formants mid = vowel_lerp(a, i, 0.5);
    CHECK(mid.f[0] < a.f[0] && mid.f[0] > i.f[0]);
    CHECK(mid.f[1] > a.f[1] && mid.f[1] < i.f[1]);
    CHECK(vowel_lerp(a, i, 0.0).f[1] == a.f[1]);
    CHECK(vowel_lerp(a, i, 1.0).f[1] == i.f[1]);

    // An unknown name must fall back rather than produce a silent bank.
    CHECK(vowel_table("zzz").f[0] == a.f[0]);

    VowelBank vb;
    Oscillator o;
    double peak = 0;
    for (int k = 0; k < 24000; ++k) {
        const float x = o.render(Wave::Saw, 120.0, 1.0 / 48000.0, 0.5f, 0.f);
        const float y = vb.process(x, a, 48000.0);
        CHECK_NEAR(std::isfinite(y) ? 0.0 : 1.0, 0.0, 0.5);
        if (k > 2000) peak = std::max(peak, double(std::fabs(y)));
    }
    CHECK(peak > 0.05);
}

void test_quantiser() {
    section("dsp: quantising snaps to the scale");
    const auto& penta = scale_degrees("major_pentatonic");
    CHECK_NEAR(quantize_semis(0.4f, penta), 0.0, 1e-6);
    CHECK_NEAR(quantize_semis(2.2f, penta), 2.0, 1e-6);
    CHECK_NEAR(quantize_semis(3.4f, penta), 4.0, 1e-6);
    CHECK_NEAR(quantize_semis(13.9f, penta), 14.0, 1e-6);     // octave up, second degree
    const auto& chrom = scale_degrees("chromatic");
    CHECK_NEAR(quantize_semis(6.4f, chrom), 6.0, 1e-6);
    CHECK(scale_degrees("nonsense").size() == penta.size());   // falls back, never empty

    CHECK_NEAR(note_to_hz("A4"), 440.0, 0.01);
    CHECK_NEAR(note_to_hz("C3"), 130.81, 0.01);
    CHECK_NEAR(note_to_hz("523.25"), 523.25, 0.01);
    CHECK(hz_to_note(440.0f) == "A4");
}

// ---------------------------------------------------------------------------- engine
// Drive the engine the way the network thread does, so the normalisers are tested through
// the same path the real frames take.
void feed(Engine& e, const std::string& mod, const json& snap, int n, double dt = 0.02) {
    for (int i = 0; i < n; ++i) e.on_frame(mod, "EPM", snap, dt);
}

void test_normalisers() {
    section("engine: each normaliser maps its source onto 0..1");
    // median_mad: a flat signal sits at 0, a spike opens up.
    {
        Engine e(48000);
        Patch  p;
        VoiceCfg v;
        v.id = v.module = "m";
        Route r;
        r.source = {"m", "x"};
        r.dest   = Dest::Pitch;
        r.norm.mode      = NormMode::MedianMad;
        r.norm.smooth_ms = 0.0;
        r.depth  = 12.0;
        v.routes.push_back(r);
        v.osc.base_hz = 100.0;
        p.voices.push_back(v);
        p.master.quantize = false;
        e.set_patch(p);

        feed(e, "m", {{"x", 1.0}}, 60);
        CHECK_NEAR(e.state_json()["voices"][0]["hz"].get<double>(), 100.0, 0.5);
        feed(e, "m", {{"x", 5.0}}, 1);
        CHECK(e.state_json()["voices"][0]["hz"].get<double>() > 150.0);   // 12 semis up
    }
    // threshold_ratio: silent below the gate, full above.
    {
        Engine e(48000);
        Patch  p;
        VoiceCfg v;
        v.id = v.module = "m";
        Route r;
        r.source = {"m", "tle"};
        r.dest   = Dest::Amp;
        r.norm.mode    = NormMode::ThresholdRatio;
        r.norm.ref_key = "thr";
        r.norm.gate    = 1.4;
        r.norm.full    = 2.0;
        r.norm.smooth_ms = 0.0;
        r.depth  = 1.0;
        r.curve  = 1.0;
        v.routes.push_back(r);
        p.voices.push_back(v);
        e.set_patch(p);

        feed(e, "m", {{"tle", 0.10}, {"thr", 0.10}}, 40);       // ratio 1.0 -> silent
        CHECK_NEAR(e.state_json()["voices"][0]["amp"].get<double>(), 0.0, 1e-6);
        feed(e, "m", {{"tle", 0.17}, {"thr", 0.10}}, 1);        // ratio 1.7 -> half
        CHECK_NEAR(e.state_json()["voices"][0]["amp"].get<double>(), 0.5, 0.02);
        feed(e, "m", {{"tle", 0.30}, {"thr", 0.10}}, 1);        // ratio 3.0 -> full
        CHECK_NEAR(e.state_json()["voices"][0]["amp"].get<double>(), 1.0, 1e-6);
        // A threshold of zero must be silence, not a division blow-up.
        feed(e, "m", {{"tle", 0.30}, {"thr", 0.0}}, 1);
        CHECK_NEAR(e.state_json()["voices"][0]["amp"].get<double>(), 0.0, 1e-6);
    }
    // raw and invert.
    {
        Engine e(48000);
        Patch  p;
        VoiceCfg v;
        v.id = v.module = "m";
        Route r;
        r.source = {"m", "u"};
        r.dest   = Dest::Amp;
        r.norm.mode      = NormMode::Raw;
        r.norm.in_lo     = 0.0;
        r.norm.in_hi     = 10.0;
        r.norm.smooth_ms = 0.0;
        r.invert = true;
        v.routes.push_back(r);
        p.voices.push_back(v);
        e.set_patch(p);
        feed(e, "m", {{"u", 2.5}}, 3);
        CHECK_NEAR(e.state_json()["voices"][0]["amp"].get<double>(), 0.75, 1e-6);
    }
    // delta: a monotone counter's VALUE is a ramp; its RATE is where the event is.
    {
        Engine e(48000);
        Patch  p;
        VoiceCfg v;
        v.id = v.module = "m";
        Route r;
        r.source = {"m", "nodes"};
        r.dest   = Dest::Amp;
        r.norm.mode      = NormMode::Delta;
        r.norm.smooth_ms = 0.0;
        r.norm.z_lo = 0.0;
        r.norm.z_hi = 4.0;
        v.routes.push_back(r);
        p.voices.push_back(v);
        e.set_patch(p);
        double x = 0;
        for (int i = 0; i < 60; ++i) { x += 1.0; e.on_frame("m", "EPM", {{"nodes", x}}, 0.02); }
        const double steady = e.state_json()["voices"][0]["amp"].get<double>();
        x += 40.0;                                       // a burst of growth
        e.on_frame("m", "EPM", {{"nodes", x}}, 0.02);
        CHECK(e.state_json()["voices"][0]["amp"].get<double>() > steady + 0.2);
    }
}

void test_nested_sources_and_missing() {
    section("engine: nested maps flatten, missing sources are silent");
    Engine e(48000);
    Patch  p;
    VoiceCfg v;
    v.id = v.module = "voter";
    Route r;
    r.source = {"voter", "trust.video"};              // one level of nesting, dotted
    r.dest   = Dest::Amp;
    r.norm.mode      = NormMode::Raw;
    r.norm.in_hi     = 1.0;
    r.norm.smooth_ms = 0.0;
    v.routes.push_back(r);
    Route missing;
    missing.source = {"nope", "gone"};
    missing.dest   = Dest::Pitch;
    missing.depth  = 99.0;
    v.routes.push_back(missing);
    v.osc.base_hz = 200.0;
    p.voices.push_back(v);
    p.master.quantize = false;
    e.set_patch(p);

    for (int i = 0; i < 5; ++i)
        e.on_frame("voter", "LateralVoter",
                   {{"fused_tle", 0.2}, {"trust", {{"video", 0.6}, {"proprio", 0.4}}}}, 0.02);

    CHECK_NEAR(e.state_json()["voices"][0]["amp"].get<double>(), 0.6, 1e-6);
    // A route pointing at a source that does not exist contributes nothing rather than
    // silencing the voice or throwing.
    CHECK_NEAR(e.state_json()["voices"][0]["hz"].get<double>(), 200.0, 1e-6);

    const json srcs = e.sources_json();
    bool found = false;
    for (const auto& m : srcs)
        if (m["module"] == "voter")
            for (const auto& k : m["keys"])
                if (k["key"] == "trust.proprio") found = true;
    CHECK(found);
}

void test_empty_module_is_reported() {
    section("engine: a module with no diag_lite is visible, not invisible");
    Engine e(48000);
    e.note_module_seen("action_gate", "ActionGate");
    for (int i = 0; i < 5; ++i) e.on_frame("action_gate", "ActionGate", json::object(), 0.02);
    const auto obs = e.observed();
    bool seen = false;
    for (const auto& m : obs)
        if (m.module == "action_gate") { seen = true; CHECK(m.sources.empty()); }
    CHECK(seen);
    CHECK(auto_patch(obs, Patch{}).voices.empty());
}

void test_render_is_finite_and_gated() {
    section("engine: render stays finite, and silence means silence");
    Engine e(48000);
    Patch  p;
    VoiceCfg v;
    v.id = v.module = "m";
    v.osc.waveform   = Wave::Saw;
    v.osc.base_hz    = 220.0;
    v.filter.enabled = true;
    v.filter.mode    = FilterMode::Vowel;
    Route r;
    r.source = {"m", "tle"};
    r.dest   = Dest::Amp;
    r.norm.mode      = NormMode::Raw;
    r.norm.in_hi     = 1.0;
    r.norm.smooth_ms = 0.0;
    v.routes.push_back(r);
    Route cut;
    cut.source = {"m", "tle"};
    cut.dest   = Dest::Cutoff;
    cut.norm.mode  = NormMode::Raw;
    cut.norm.in_hi = 1.0;
    cut.norm.smooth_ms = 0.0;
    cut.depth  = 48.0;                                  // four octaves of cutoff sweep
    v.routes.push_back(cut);
    p.voices.push_back(v);
    p.master.filter.enabled = true;
    p.master.filter.mode    = FilterMode::LowPass;
    e.set_patch(p);

    std::vector<int16_t> buf(512);

    // amp source at 0 -> after the release envelope, true digital silence.
    feed(e, "m", {{"tle", 0.0}}, 5);
    for (int b = 0; b < 60; ++b) e.render(buf.data(), 256);
    long acc = 0;
    for (int b = 0; b < 4; ++b) {
        e.render(buf.data(), 256);
        for (int16_t s : buf) acc += std::abs(int(s));
    }
    CHECK(acc == 0);

    // driven hard, with the cutoff sweeping: bounded and finite.
    for (int b = 0; b < 200; ++b) {
        feed(e, "m", {{"tle", (b % 2) ? 1.0 : 0.05}}, 1);
        e.render(buf.data(), 256);
    }
    long peak = 0;
    for (int16_t s : buf) peak = std::max(peak, long(std::abs(int(s))));
    CHECK(peak > 0);
    CHECK(peak <= 32767);

    // A disabled voice is silent however loud its sources are.
    Patch q = e.patch();
    q.voices[0].enabled = false;
    e.set_patch(q);
    feed(e, "m", {{"tle", 1.0}}, 5);
    for (int b = 0; b < 80; ++b) e.render(buf.data(), 256);
    acc = 0;
    for (int b = 0; b < 4; ++b) {
        e.render(buf.data(), 256);
        for (int16_t s : buf) acc += std::abs(int(s));
    }
    CHECK(acc == 0);
}

void test_runtime_survives_a_patch_edit() {
    section("engine: a slider drag does not restart the instrument");
    Engine e(48000);
    Patch  p;
    VoiceCfg v;
    v.id = v.module = "m";
    Route r;
    r.source = {"m", "x"};
    r.dest   = Dest::Pitch;
    r.norm.mode      = NormMode::MedianMad;
    r.norm.smooth_ms = 0.0;
    r.depth  = 12.0;
    v.routes.push_back(r);
    v.osc.base_hz = 100.0;
    p.voices.push_back(v);
    p.master.quantize = false;
    e.set_patch(p);

    // Fill the running window, then confirm the spike response...
    feed(e, "m", {{"x", 1.0}}, 60);
    feed(e, "m", {{"x", 5.0}}, 1);
    const double hot = e.state_json()["voices"][0]["hz"].get<double>();
    CHECK(hot > 150.0);

    // ...edit an unrelated field, and confirm the window was NOT thrown away.  If the
    // stats reset, the next frame reads as "not ready" and the pitch drops to base — the
    // ten seconds of history a median/MAD route needs is exactly what an operator would
    // destroy by touching a slider.
    std::string err;
    {
        std::lock_guard<std::mutex> lk(e.mtx_);
        CHECK(e.apply_ops_locked(
            json::array({{{"path", "/voices/0/osc/release_ms"}, {"value", 88.0}}}), err));
    }
    feed(e, "m", {{"x", 5.0}}, 1);
    CHECK_NEAR(e.state_json()["voices"][0]["hz"].get<double>(), hot, 1.0);
    CHECK_NEAR(e.patch().voices[0].osc.release_ms, 88.0, 1e-9);
}

void test_events() {
    section("engine: event triggers fire on the right transition");
    Engine e(48000);
    Patch  p;
    VoiceCfg v;
    v.id = v.module = "m";
    v.events.push_back({{"m", "mitosis_count"}, Trigger::Increase, "two_notes", true});
    v.events.push_back({{"m", "nodes"}, Trigger::Decrease, "blip_down", true});
    p.voices.push_back(v);
    e.set_patch(p);

    std::vector<int16_t> buf(512);
    auto energy = [&]() {
        long acc = 0;
        for (int b = 0; b < 12; ++b) {
            e.render(buf.data(), 256);
            for (int16_t s : buf) acc += std::abs(int(s));
        }
        return acc;
    };

    // No event: with no amp route at all the voice is silent.
    e.on_frame("m", "EPM", {{"mitosis_count", 3}, {"nodes", 40}}, 0.02);
    e.on_frame("m", "EPM", {{"mitosis_count", 3}, {"nodes", 40}}, 0.02);
    CHECK(energy() == 0);

    // Mitosis: a chirp is audible even though nothing drives amplitude.
    e.on_frame("m", "EPM", {{"mitosis_count", 4}, {"nodes", 40}}, 0.02);
    CHECK(energy() > 0);

    // A prune (node count falling) fires too.
    for (int b = 0; b < 40; ++b) e.render(buf.data(), 256);
    e.on_frame("m", "EPM", {{"mitosis_count", 4}, {"nodes", 38}}, 0.02);
    CHECK(energy() > 0);
}

void test_hz_report() {
    section("engine: the master mod rack reaches the output filter");
    Engine e(48000);
    Patch  p;
    p.master.filter.enabled   = true;
    p.master.filter.mode      = FilterMode::LowPass;
    p.master.filter.cutoff_hz = 1000.0;
    Route r;
    r.source = {"voter", "fused_tle"};
    r.dest   = Dest::Cutoff;
    r.norm.mode      = NormMode::Raw;
    r.norm.in_hi     = 1.0;
    r.norm.smooth_ms = 0.0;
    r.depth  = 12.0;                          // one octave
    p.master.routes.push_back(r);
    e.set_patch(p);

    e.on_frame("voter", "LateralVoter", {{"fused_tle", 0.0}}, 0.02);
    CHECK_NEAR(e.state_json()["master"]["cutoff"].get<double>(), 1000.0, 1.0);
    e.on_frame("voter", "LateralVoter", {{"fused_tle", 1.0}}, 0.02);
    CHECK_NEAR(e.state_json()["master"]["cutoff"].get<double>(), 2000.0, 2.0);
}

}  // namespace

int main() {
    test_patch_roundtrip();
    test_patch_defaults_and_tolerance();
    test_apply_ops();
    test_auto_patch();
    test_waveform_dc();
    test_pulse_width_is_centred();
    test_filter_stability_under_modulation();
    test_filter_shapes();
    test_vowel_bank();
    test_quantiser();
    test_normalisers();
    test_nested_sources_and_missing();
    test_empty_module_is_reported();
    test_render_is_finite_and_gated();
    test_runtime_survives_a_patch_edit();
    test_events();
    test_hz_report();

    std::printf("\n%d checks, %d failed\n", g_ran, g_fail);
    return g_fail ? 1 : 0;
}
