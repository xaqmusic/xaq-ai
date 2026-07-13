// =============================================================================
// test_motor_fader_streams.cpp
//   Phase 6.6.G — MotorFader pure-blender behavioral / stream tests.
//
// 6.6.G refactor: MotorFader no longer computes α; it reads the latest
// FaderState off motor.fader.alpha (or falls back to alpha_fixed when
// no FaderController is wired into the graph).  Six tests cover the
// new contract:
//
//   1. Fallback at α=0           — no FaderState → output = reflex.
//   2. Fallback at α=1           — no FaderState → output = brain.
//   3. Fallback midpoint         — no FaderState, α_fixed=0.5 → midpoint.
//   4. α from bus overrides      — published FaderState wins over α_fixed.
//   5. Multiple instances share  — two MotorFaders + one FaderState
//                                  → both honor the same α.
//   6. Missing-input safety      — no crash, no NaN; absent side = 0.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/MotorFader.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::ActionOut> make_action(float accel, std::string src = "") {
    auto a = std::make_shared<ogma::ActionOut>();
    a->accel  = accel;
    a->source = std::move(src);
    return a;
}

std::shared_ptr<ogma::FaderState> make_fader_state(float alpha,
                                                    float surprise = 0.0f) {
    auto fs = std::make_shared<ogma::FaderState>();
    fs->alpha           = alpha;
    fs->alpha_target    = alpha;
    fs->surprise_scalar = surprise;
    fs->source          = "fixed";
    return fs;
}

ogma::ParamMap base_params() {
    return {
        {"alpha_fixed", 0.0},
    };
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::MotorFader   fader;
    explicit Fixture(ogma::ParamMap const& p) {
        fader.set_id("fader");
        fader.on_setup(&bus, p);
    }
    void run_tick(uint64_t t,
                  std::shared_ptr<ogma::ActionOut> brain,
                  std::shared_ptr<ogma::ActionOut> reflex,
                  std::shared_ptr<ogma::FaderState> fs = nullptr,
                  std::string brain_topic  = "action.brain",
                  std::string reflex_topic = "action.reflex") {
        bus.begin_tick(t);
        if (fs)     bus.publish(ogma::topics::kMotorFaderAlpha, fs);
        if (brain)  bus.publish(brain_topic,  brain);
        if (reflex) bus.publish(reflex_topic, reflex);
        fader.tick(t);
        bus.end_tick();
    }
    std::shared_ptr<const ogma::ActionOut> last_action(
            std::string topic = "action.out") const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(topic));
    }
};

} // namespace

// =============================================================================
// 1. Fallback at α=0 (no FaderState ever arrives) → output exactly = reflex.
// =============================================================================

TEST(MotorFaderStreams, FallbackAtZeroMatchesReflex) {
    auto p = base_params();
    p["alpha_fixed"] = 0.0;
    Fixture f(p);

    for (uint64_t t = 0; t < 50; ++t) {
        float r = -2.5f + 0.1f * float(t);
        f.run_tick(t, make_action(3.0f, "brain"), make_action(r, "reflex"));
        auto out = f.last_action();
        ASSERT_NE(out, nullptr);
        EXPECT_NEAR(out->accel, r, 1e-5f)
            << "Without FaderController, alpha_fixed=0 → output equals reflex";
    }
    EXPECT_FALSE(f.fader.alpha_from_bus());
}

// =============================================================================
// 2. Fallback at α=1 (no FaderState) → output exactly = brain.
// =============================================================================

TEST(MotorFaderStreams, FallbackAtOneMatchesBrain) {
    auto p = base_params();
    p["alpha_fixed"] = 1.0;
    Fixture f(p);

    for (uint64_t t = 0; t < 50; ++t) {
        float b = 1.5f - 0.05f * float(t);
        f.run_tick(t, make_action(b, "brain"), make_action(99.0f, "reflex"));
        auto out = f.last_action();
        ASSERT_NE(out, nullptr);
        EXPECT_NEAR(out->accel, b, 1e-5f);
    }
}

// =============================================================================
// 3. Fallback midpoint (no FaderState, α_fixed=0.5) → linear blend.
// =============================================================================

TEST(MotorFaderStreams, FallbackMidpointBlends) {
    auto p = base_params();
    p["alpha_fixed"] = 0.5;
    Fixture f(p);

    f.run_tick(0, make_action(4.0f), make_action(-2.0f));
    auto out = f.last_action();
    ASSERT_NE(out, nullptr);
    EXPECT_NEAR(out->accel, 1.0f, 1e-5f);   // 0.5 * 4 + 0.5 * -2
}

// =============================================================================
// 4. α from FaderState overrides alpha_fixed.  Once any FaderState arrives
//    the fallback is permanently bypassed for this fader instance.
// =============================================================================

TEST(MotorFaderStreams, AlphaFromBusOverridesFallback) {
    auto p = base_params();
    p["alpha_fixed"] = 0.0;            // would be reflex-only
    Fixture f(p);

    // Tick 0: no FaderState → fallback says reflex-only.
    f.run_tick(0, make_action(10.0f, "brain"), make_action(-4.0f, "reflex"));
    auto out = f.last_action();
    ASSERT_NE(out, nullptr);
    EXPECT_NEAR(out->accel, -4.0f, 1e-5f);
    EXPECT_FALSE(f.fader.alpha_from_bus());

    // Tick 1: FaderState says α=0.75 → blender uses that, not alpha_fixed.
    f.run_tick(1, make_action(10.0f, "brain"), make_action(-4.0f, "reflex"),
               make_fader_state(0.75f));
    out = f.last_action();
    ASSERT_NE(out, nullptr);
    EXPECT_NEAR(out->accel, 0.75f * 10.0f + 0.25f * -4.0f, 1e-4f);  // = 6.5
    EXPECT_TRUE(f.fader.alpha_from_bus());

    // Tick 2: no fresh FaderState — alpha_ retains the last value (0.75).
    f.run_tick(2, make_action(10.0f, "brain"), make_action(-4.0f, "reflex"));
    out = f.last_action();
    EXPECT_NEAR(out->accel, 0.75f * 10.0f + 0.25f * -4.0f, 1e-4f);

    // Tick 3: FaderState updates to α=0.1 → blender immediately applies it.
    f.run_tick(3, make_action(10.0f, "brain"), make_action(-4.0f, "reflex"),
               make_fader_state(0.1f));
    out = f.last_action();
    EXPECT_NEAR(out->accel, 0.1f * 10.0f + 0.9f * -4.0f, 1e-4f);    // = -2.6

    // Out-of-range FaderState values must clamp to [0, 1].
    f.run_tick(4, make_action(10.0f, "brain"), make_action(-4.0f, "reflex"),
               make_fader_state(2.0f));
    out = f.last_action();
    EXPECT_NEAR(out->accel, 10.0f, 1e-5f);   // α clamped to 1 → all brain

    f.run_tick(5, make_action(10.0f, "brain"), make_action(-4.0f, "reflex"),
               make_fader_state(-1.0f));
    out = f.last_action();
    EXPECT_NEAR(out->accel, -4.0f, 1e-5f);   // α clamped to 0 → all reflex
}

// =============================================================================
// 5. Multiple MotorFader instances share the same FaderState topic and
//    therefore the same α.  This is the contract that lets one
//    FaderController drive a bilateral (left + right) body coherently.
// =============================================================================

TEST(MotorFaderStreams, MultipleInstancesShareAlpha) {
    ogma::InProcessBus bus;
    ogma::MotorFader left, right;

    left.set_id("fader_left");
    right.set_id("fader_right");

    left.on_setup(&bus, {
        {"brain_topic",  std::string("action.brain.left")},
        {"reflex_topic", std::string("action.reflex.left")},
        {"output_topic", std::string("action.left")},
        {"alpha_fixed",  0.0},
    });
    right.on_setup(&bus, {
        {"brain_topic",  std::string("action.brain.right")},
        {"reflex_topic", std::string("action.reflex.right")},
        {"output_topic", std::string("action.right")},
        {"alpha_fixed",  0.0},
    });

    auto fs = make_fader_state(0.4f);
    bus.begin_tick(0);
    bus.publish(ogma::topics::kMotorFaderAlpha, fs);
    bus.publish("action.brain.left",  make_action(5.0f));
    bus.publish("action.reflex.left", make_action(-1.0f));
    bus.publish("action.brain.right", make_action(2.0f));
    bus.publish("action.reflex.right",make_action(-3.0f));
    left.tick(0);
    right.tick(0);
    bus.end_tick();

    auto out_l = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value("action.left"));
    auto out_r = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value("action.right"));
    ASSERT_NE(out_l, nullptr);
    ASSERT_NE(out_r, nullptr);

    EXPECT_NEAR(out_l->accel, 0.4f * 5.0f + 0.6f * -1.0f, 1e-5f);   // = 1.4
    EXPECT_NEAR(out_r->accel, 0.4f * 2.0f + 0.6f * -3.0f, 1e-5f);   // = -1.0
    EXPECT_NEAR(left.alpha(),  0.4f, 1e-5f);
    EXPECT_NEAR(right.alpha(), 0.4f, 1e-5f);
    EXPECT_TRUE(left.alpha_from_bus());
    EXPECT_TRUE(right.alpha_from_bus());
}

// =============================================================================
// 6. Missing-input safety. No crash, no NaN. Absent side counts as 0.
//    Validation contract: ablate brain + α=1 → output = 0 (the "empty
//    graph = frozen agent" promise of 6.6.G).
// =============================================================================

TEST(MotorFaderStreams, MissingInputUsesZero) {
    auto p = base_params();
    p["alpha_fixed"] = 0.5;
    Fixture f(p);

    // Brain only.
    f.run_tick(0, make_action(2.0f), nullptr);
    auto out = f.last_action();
    ASSERT_NE(out, nullptr);
    EXPECT_NEAR(out->accel, 1.0f, 1e-5f);     // 0.5 * 2 + 0.5 * 0
    EXPECT_FALSE(std::isnan(out->accel));
    EXPECT_TRUE(f.fader.brain_seen());
    EXPECT_FALSE(f.fader.reflex_seen());

    // Reflex only.
    f.run_tick(1, nullptr, make_action(-3.0f));
    out = f.last_action();
    ASSERT_NE(out, nullptr);
    EXPECT_NEAR(out->accel, -1.5f, 1e-5f);
    EXPECT_FALSE(std::isnan(out->accel));
    EXPECT_FALSE(f.fader.brain_seen());
    EXPECT_TRUE(f.fader.reflex_seen());

    // Neither.
    f.run_tick(2, nullptr, nullptr);
    out = f.last_action();
    ASSERT_NE(out, nullptr);
    EXPECT_NEAR(out->accel, 0.0f, 1e-5f);
    EXPECT_FALSE(std::isnan(out->accel));

    // The 6.6.G validation contract: brain ablated, α=1, no reflex →
    // output must be 0 (no autonomous motion).
    f.run_tick(3, nullptr, nullptr, make_fader_state(1.0f));
    out = f.last_action();
    ASSERT_NE(out, nullptr);
    EXPECT_NEAR(out->accel, 0.0f, 1e-5f)
        << "Empty-graph validation: α=1 with no brain input must publish 0";
}

// =============================================================================
// 7. Phase 6.6.P inverted-babbler noise — defaults are bit-identical (the
//    falsifiability check the 6.5.37 lesson taught us to demand of any new
//    primitive).  noise_amplitude=0 must produce exactly the deterministic
//    blend, regardless of surprise value on the bus.
// =============================================================================

TEST(MotorFaderStreams, BabblerOffIsBitIdentical) {
    auto p = base_params();
    p["alpha_fixed"]      = 0.5;
    p["noise_amplitude"]  = 0.0;
    Fixture f(p);

    for (uint64_t t = 0; t < 30; ++t) {
        float surprise = (t % 5) * 0.2f;   // varies but should be ignored
        f.run_tick(t, make_action(2.0f), make_action(-2.0f),
                   make_fader_state(0.5f, surprise));
        auto out = f.last_action();
        ASSERT_NE(out, nullptr);
        EXPECT_NEAR(out->accel, 0.0f, 1e-6f) << "tick " << t;
        EXPECT_NEAR(f.fader.last_noise(), 0.0f, 1e-6f);
    }
}

// =============================================================================
// 8. Babbler scales inversely with surprise.  At surprise=1.0 the gain is
//    exactly 0 (decisive); at surprise=0.0 the noise is the full amplitude.
//    Empirical envelope check via the standard deviation of last_noise()
//    across ticks at fixed surprise.
// =============================================================================

TEST(MotorFaderStreams, BabblerGainScalesInverselyWithSurprise) {
    auto run = [](float surprise) {
        auto p = base_params();
        p["alpha_fixed"]      = 0.0;
        p["noise_amplitude"]  = 1.0;
        p["noise_seed"]       = int64_t(7);
        Fixture f(p);
        std::vector<float> samples;
        for (uint64_t t = 0; t < 4000; ++t) {
            f.run_tick(t, nullptr, make_action(0.0f),
                       make_fader_state(0.0f, surprise));
            samples.push_back(f.fader.last_noise());
        }
        double mean = 0.0;
        for (auto v : samples) mean += v;
        mean /= samples.size();
        double var = 0.0;
        for (auto v : samples) var += (v - mean) * (v - mean);
        var /= samples.size();
        return std::sqrt(var);
    };
    float sd_low  = run(0.0f);    // expected gain = 1.0 (full noise)
    float sd_mid  = run(0.5f);    // expected gain = 0.5
    float sd_high = run(1.0f);    // expected gain = 0.0 (zero noise)

    EXPECT_GT(sd_low, 0.85f);            // ~1.0 with finite-sample wiggle
    EXPECT_LT(sd_low, 1.15f);
    EXPECT_NEAR(sd_mid, 0.5f * sd_low, 0.05f);
    EXPECT_NEAR(sd_high, 0.0f, 1e-6f)
        << "surprise=1.0 must yield exactly zero noise";
}

// =============================================================================
// 9. Babbler RNG seeding is deterministic — same seed → same noise sequence,
//    different seed → different noise sequence (paired-seed determinism the
//    A/B harness depends on).
// =============================================================================

TEST(MotorFaderStreams, BabblerSeedDeterminism) {
    auto run_with_seed = [](int64_t seed) {
        auto p = base_params();
        p["alpha_fixed"]      = 0.0;
        p["noise_amplitude"]  = 1.0;
        p["noise_seed"]       = seed;
        Fixture f(p);
        std::vector<float> noise;
        for (uint64_t t = 0; t < 50; ++t) {
            f.run_tick(t, nullptr, make_action(0.0f),
                       make_fader_state(0.0f, 0.0f));
            noise.push_back(f.fader.last_noise());
        }
        return noise;
    };
    auto a = run_with_seed(42);
    auto b = run_with_seed(42);
    auto c = run_with_seed(43);
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i)
        EXPECT_FLOAT_EQ(a[i], b[i]) << "tick " << i;
    bool any_diff = false;
    for (size_t i = 0; i < a.size(); ++i)
        if (a[i] != c[i]) { any_diff = true; break; }
    EXPECT_TRUE(any_diff) << "different seeds must produce different sequences";
}
