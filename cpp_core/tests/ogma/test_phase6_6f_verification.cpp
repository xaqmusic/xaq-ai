// =============================================================================
// test_phase6_6f_verification.cpp
//   Phase 6.6.F end-to-end verification — full chain + behavioral guarantees.
//
// Wires real EPMs + LateralVoter + Premotor + a synthetic reflex emitter +
// MotorFader and asserts the regime works end-to-end:
//
//   1. Cold start: surprise high → α near alpha_min → output ≈ reflex.
//   2. Warmup: stable input → predictions accurate → α rises → output
//      shifts toward brain.
//   3. Shock: inject novel input → surprise spikes → α drops within
//      ~1/alpha_smoothing ticks.
//   4. Bit-identical-numbers safeguard (6.5.37 protective check):
//      same wiring with surprise_gain=0 vs >0 produces measurably
//      different action.out streams.  No silent gating.
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/modules/FaderController.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/modules/MotorFader.hpp"
#include "ogma/modules/Premotor.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap epm_params(std::string const& name, int pathway_steps) {
    return {
        {"modality_group",      std::string("proprio")},
        {"modality_name",       name},
        {"encoder_kind",        std::string("rbf")},
        {"input_topic",         std::string("reality.proprio.") + name + ".raw"},
        {"projection_dim",      int64_t{16}},
        {"proprio_state_dims",  int64_t{4}},
        {"baking_threshold",    int64_t{8}},
        {"min_insertion_error", 0.001},
        {"lambda_new",          int64_t{4}},
        {"history_trace_size",  int64_t{4}},
        {"predicted_pathway_steps", int64_t(pathway_steps)},
        {"subtract_descending_prediction", false},
        {"mitosis_enabled",     false},
        {"stale_prune_enabled", false},
    };
}

ogma::ParamMap voter_params(double surprise_gain) {
    return {
        {"level",                int64_t{0}},
        {"input_pattern",        std::string("reality.")},
        {"trust_epsilon",        0.05},
        {"group_balance",        false},
        {"priority_group",       std::string("proprio")},
        {"surprise_gain",        surprise_gain},
        {"surprise_alpha",       0.2},
        {"surprise_floor",       0.05},
        // Phase 6.6.J: this E2E exercises the 6.6.F surprise→α dynamics in
        // isolation; opt out of the calibration-toward-prior layer that
        // 6.6.J added on top so warmup/shock/bit-identity assertions match
        // the raw EMA dynamics they were written against.
        {"surprise_calibrate",   false},
    };
}

ogma::ParamMap premotor_params() {
    return {
        {"level",                int64_t{0}},
        {"n_intents",            int64_t{5}},
        {"accel_min",           -4.0},
        {"accel_max",            4.0},
        {"learning_rate",        0.05},
        {"sample_action",        false},
        {"use_weighted_accel",   true},
        {"master_seed",          int64_t{7}},
    };
}

// Phase 6.6.G refactor: α-computation lives on FaderController; MotorFader
// is a pure blender that subscribes to motor.fader.alpha.
ogma::ParamMap fader_controller_params(std::string source, double smoothing = 0.2) {
    return {
        {"consensus_topic",      std::string("consensus.0")},
        {"alpha_topic",          std::string("motor.fader.alpha")},
        {"alpha_source",         source},     // "fixed" or "surprise"
        {"alpha_fixed",          0.0},
        {"alpha_smoothing",      smoothing},
        {"alpha_min",            0.05},
        {"alpha_max",            0.95},
        {"surprise_aggregation", std::string("mean")},
        // Phase 6.6.K: explicit opt-out of familiarity coupling so this
        // E2E tests pure surprise→α dynamics. Bilateral configs use 1.0.
        {"pathway_alpha_coupling", 0.0},
    };
}

ogma::ParamMap motor_fader_params() {
    return {
        {"brain_topic",  std::string("action.brain")},
        {"reflex_topic", std::string("action.reflex")},
        {"output_topic", std::string("action.out")},
        {"alpha_topic",  std::string("motor.fader.alpha")},
        {"alpha_fixed",  0.0},     // fallback if no FaderState arrives
    };
}

std::shared_ptr<ogma::ProprioToken>
make_proprio4(float a, float b, float c, float d, std::string sensor) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->sensor = std::move(sensor);
    p->values.resize(4);
    p->values << a, b, c, d;
    return p;
}

// Predictable 4-cycle proprio pattern.
std::shared_ptr<ogma::ProprioToken> stable_input(uint64_t t, std::string const& sensor) {
    static const float patt[4][4] = {
        { 1.0f,  0.0f,  0.5f, -0.5f},
        { 0.0f,  1.0f,  0.5f, -0.5f},
        {-1.0f,  0.0f,  0.5f, -0.5f},
        { 0.0f, -1.0f,  0.5f, -0.5f},
    };
    auto const& v = patt[t % 4];
    return make_proprio4(v[0], v[1], v[2], v[3], sensor);
}

// PRNG-shock proprio (out-of-distribution).
std::shared_ptr<ogma::ProprioToken> shock_input(uint64_t t, std::string const& sensor) {
    std::mt19937 rng(0xDEADBEEFu + uint32_t(t));
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    return make_proprio4(u(rng), u(rng), u(rng), u(rng), sensor);
}

struct E2EFixture {
    ogma::InProcessBus     bus;
    ogma::EPM              epm_a;
    ogma::EPM              epm_b;
    ogma::LateralVoter     voter;
    ogma::Premotor         premotor;
    ogma::FaderController  controller;
    ogma::MotorFader       fader;

    explicit E2EFixture(double surprise_gain, std::string fader_source) {
        epm_a     .set_id("epm_a");
        epm_b     .set_id("epm_b");
        voter     .set_id("voter_0");
        premotor  .set_id("premotor");
        controller.set_id("fader_controller");
        fader     .set_id("fader");

        // Voter first so its prefix subscription catches the very first
        // EPM publish in tick 0.
        voter     .on_setup(&bus, voter_params(surprise_gain));
        premotor  .on_setup(&bus, premotor_params());
        epm_a     .on_setup(&bus, epm_params("a", /*steps=*/3));
        epm_b     .on_setup(&bus, epm_params("b", /*steps=*/3));
        controller.on_setup(&bus, fader_controller_params(fader_source));
        fader     .on_setup(&bus, motor_fader_params());

        // Premotor publishes on action.out; we want the blender to read it
        // as the "brain" side, separate from the synthetic reflex. Bridge
        // via a small adapter: republish action.out → action.brain.
        bus.subscribe("action.out", ogma::SubscriptionKind::Direct,
            [this](std::string_view, ogma::MessagePtr p){
                auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(p);
                if (!a) return;
                if (a->source == "fader") return;   // avoid loop
                auto re = std::make_shared<ogma::ActionOut>(*a);
                bus.publish("action.brain", re);
            });
    }

    void run_tick(uint64_t t,
                  std::shared_ptr<ogma::ProprioToken> a_in,
                  std::shared_ptr<ogma::ProprioToken> b_in,
                  float reflex_accel) {
        bus.begin_tick(t);
        if (a_in) bus.publish("reality.proprio.a.raw", a_in);
        if (b_in) bus.publish("reality.proprio.b.raw", b_in);
        // Synthetic reflex emitter: constant accel.
        auto rx = std::make_shared<ogma::ActionOut>();
        rx->tick_id = t;
        rx->accel   = reflex_accel;
        rx->source  = "synth_reflex";
        bus.publish("action.reflex", rx);
        // Tick order: EPMs publish reality → voter consumes → premotor →
        // controller computes α from latest consensus → fader blends brain
        // (via adapter) with reflex using that α.
        epm_a     .tick(t);
        epm_b     .tick(t);
        voter     .tick(t);
        premotor  .tick(t);
        controller.tick(t);
        fader     .tick(t);
        bus.end_tick();
    }

    std::shared_ptr<const ogma::FaderState> last_fader() const {
        return std::dynamic_pointer_cast<const ogma::FaderState>(
            bus.last_value(ogma::topics::kMotorFaderAlpha));
    }
    std::shared_ptr<const ogma::ActionOut> last_action() const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(ogma::topics::kActionOut));
    }
};

}  // namespace

// =============================================================================
// 1. Cold start: high initial surprise → α stays near alpha_min.
// =============================================================================

TEST(Phase66F_E2E, ColdStartAlphaIsLow) {
    E2EFixture f(/*surprise_gain=*/0.5, "surprise");
    // First 30 ticks of stable input — surprise EMA hasn't dropped yet
    // (predictions still accumulating).  α should be at alpha_min.
    for (uint64_t t = 0; t < 30; ++t)
        f.run_tick(t, stable_input(t, "a"), stable_input(t, "b"), -2.0f);
    auto fs = f.last_fader();
    ASSERT_NE(fs, nullptr);
    EXPECT_LT(fs->alpha, 0.5f)
        << "Cold start α should be below 0.5; got " << fs->alpha;
}

// =============================================================================
// 2. Warmup: predictions stabilise → surprise drops → α rises.
// =============================================================================

TEST(Phase66F_E2E, WarmupRaisesAlphaUnderStableInput) {
    E2EFixture f(/*surprise_gain=*/0.5, "surprise");
    for (uint64_t t = 0; t < 250; ++t)
        f.run_tick(t, stable_input(t, "a"), stable_input(t, "b"), -2.0f);
    auto fs = f.last_fader();
    ASSERT_NE(fs, nullptr);
    EXPECT_GT(fs->alpha, 0.4f)
        << "After 250 ticks of stable input α should rise above 0.4 (got "
        << fs->alpha << ")";
    EXPECT_LT(fs->surprise_scalar, 0.6f)
        << "Aggregated surprise should drop with stable input (got "
        << fs->surprise_scalar << ")";
}

// =============================================================================
// 3. Shock: inject OOD input → surprise spikes → α drops.
// =============================================================================

TEST(Phase66F_E2E, ShockDropsAlpha) {
    E2EFixture f(/*surprise_gain=*/0.5, "surprise");
    // Long warmup.
    for (uint64_t t = 0; t < 250; ++t)
        f.run_tick(t, stable_input(t, "a"), stable_input(t, "b"), -2.0f);
    auto fs_pre = f.last_fader();
    ASSERT_NE(fs_pre, nullptr);
    float alpha_pre = fs_pre->alpha;

    // Now inject 60 ticks of shock.
    for (uint64_t t = 250; t < 310; ++t)
        f.run_tick(t, shock_input(t, "a"), shock_input(t, "b"), -2.0f);
    auto fs_post = f.last_fader();
    ASSERT_NE(fs_post, nullptr);

    EXPECT_LT(fs_post->alpha, alpha_pre)
        << "Shock should reduce α below the pre-shock value (pre="
        << alpha_pre << " post=" << fs_post->alpha << ")";
}

// =============================================================================
// 4. Bit-identical-numbers safeguard (6.5.37 protective check).
//    Same wiring with surprise_gain=0 (no modulation) vs surprise_gain=0.5
//    must produce a measurably different action.out stream.  If identical,
//    the architectural change is not reaching motor output.
// =============================================================================

TEST(Phase66F_E2E, BitIdenticalSafeguard_SurpriseGainChangesAction) {
    E2EFixture f_off(/*surprise_gain=*/0.0, "surprise");
    E2EFixture f_on (/*surprise_gain=*/0.5, "surprise");

    std::vector<float> trace_off, trace_on;
    for (uint64_t t = 0; t < 200; ++t) {
        f_off.run_tick(t, stable_input(t, "a"), stable_input(t, "b"), -2.0f);
        f_on .run_tick(t, stable_input(t, "a"), stable_input(t, "b"), -2.0f);
        if (auto a = f_off.last_action()) trace_off.push_back(a->accel);
        if (auto a = f_on .last_action()) trace_on .push_back(a->accel);
    }
    ASSERT_EQ(trace_off.size(), trace_on.size());
    ASSERT_GT(trace_off.size(), 100u);

    // Compute mean absolute difference over the back half of the run
    // (after both have warmed up).
    int n = int(trace_off.size());
    int start = n / 2;
    float sum_abs_diff = 0.0f;
    for (int i = start; i < n; ++i)
        sum_abs_diff += std::abs(trace_off[i] - trace_on[i]);
    float mean_abs_diff = sum_abs_diff / float(n - start);

    EXPECT_GT(mean_abs_diff, 1e-3f)
        << "BIT-IDENTICAL action streams across surprise_gain=0 vs 0.5 — the "
           "fader's surprise modulation is not reaching motor output. "
           "(mean |Δaccel| over back half = " << mean_abs_diff << ")";
}
