// =============================================================================
// test_phase6_6g_verification.cpp
//   Phase 6.6.G end-to-end verification — bilateral fader chain.
//
// Wires the full 6.6.G crossfade architecture in-process:
//   - 2 EPMs (predicted_pathway_steps > 0)
//   - LateralVoter (surprise_gain configurable, exposes surprise_ema)
//   - Premotor in bilateral mode (output_topic_left / output_topic_right)
//   - synthetic bilateral "reflex" emitter (action.reflex.left/right)
//   - FaderController (consensus.0 → motor.fader.alpha)
//   - 2 MotorFader instances (fader_left + fader_right → action.left/right)
//
// Four tests cover the contract:
//   1. EmptyBrainAtAlphaOneFreezes — α=1 with no brain publish → both
//      action.{left,right} = 0 (the validation contract).
//   2. AlphaZeroPassesReflexBilaterally — α=0 → both channels equal
//      reflex publishes exactly.
//   3. SurpriseDrivenAlphaEvolvesCoherently — both channels honor the
//      same α as it evolves with surprise.
//   4. BitIdenticalSafeguard — surprise_gain=0 vs >0 produces measurably
//      different action.{left,right} streams.
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
    };
}

ogma::ParamMap premotor_bilateral_params() {
    return {
        {"level",                int64_t{0}},
        {"n_intents",            int64_t{5}},
        {"accel_min",           -4.0},
        {"accel_max",            4.0},
        {"learning_rate",        0.05},
        {"sample_action",        false},
        {"use_weighted_accel",   true},
        {"output_topic_left",    std::string("action.brain.left")},
        {"output_topic_right",   std::string("action.brain.right")},
        {"master_seed",          int64_t{7}},
    };
}

ogma::ParamMap fader_controller_params(std::string source, double smoothing = 0.2,
                                        double fixed = 0.0) {
    return {
        {"consensus_topic",      std::string("consensus.0")},
        {"alpha_topic",          std::string("motor.fader.alpha")},
        {"alpha_source",         source},
        {"alpha_fixed",          fixed},
        {"alpha_smoothing",      smoothing},
        {"alpha_min",            0.0},
        {"alpha_max",            1.0},
        {"surprise_aggregation", std::string("mean")},
    };
}

ogma::ParamMap motor_fader_params(std::string brain, std::string reflex,
                                   std::string output) {
    return {
        {"brain_topic",  brain},
        {"reflex_topic", reflex},
        {"output_topic", output},
        {"alpha_topic",  std::string("motor.fader.alpha")},
        {"alpha_fixed",  0.0},
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
    ogma::MotorFader       fader_left;
    ogma::MotorFader       fader_right;
    bool                   premotor_active = true;

    explicit E2EFixture(double surprise_gain,
                        std::string fader_source,
                        double alpha_fixed = 0.0,
                        bool include_premotor = true) {
        epm_a     .set_id("epm_a");
        epm_b     .set_id("epm_b");
        voter     .set_id("voter_0");
        premotor  .set_id("premotor");
        controller.set_id("fader_controller");
        fader_left.set_id("fader_left");
        fader_right.set_id("fader_right");
        premotor_active = include_premotor;

        voter     .on_setup(&bus, voter_params(surprise_gain));
        if (include_premotor) {
            premotor.on_setup(&bus, premotor_bilateral_params());
        }
        epm_a     .on_setup(&bus, epm_params("a", /*steps=*/3));
        epm_b     .on_setup(&bus, epm_params("b", /*steps=*/3));
        controller.on_setup(&bus, fader_controller_params(
            fader_source, /*smoothing=*/0.2, alpha_fixed));
        fader_left .on_setup(&bus, motor_fader_params(
            "action.brain.left",  "action.reflex.left",  "action.left"));
        fader_right.on_setup(&bus, motor_fader_params(
            "action.brain.right", "action.reflex.right", "action.right"));
    }

    void run_tick(uint64_t t,
                  std::shared_ptr<ogma::ProprioToken> a_in,
                  std::shared_ptr<ogma::ProprioToken> b_in,
                  float reflex_left,
                  float reflex_right) {
        bus.begin_tick(t);
        if (a_in) bus.publish("reality.proprio.a.raw", a_in);
        if (b_in) bus.publish("reality.proprio.b.raw", b_in);
        // Bilateral synthetic reflex emitter.
        auto rxl = std::make_shared<ogma::ActionOut>();
        rxl->tick_id = t;
        rxl->accel   = reflex_left;
        rxl->source  = "synth_reflex";
        bus.publish("action.reflex.left", rxl);
        auto rxr = std::make_shared<ogma::ActionOut>();
        rxr->tick_id = t;
        rxr->accel   = reflex_right;
        rxr->source  = "synth_reflex";
        bus.publish("action.reflex.right", rxr);
        // Tick order: EPMs → voter → premotor → controller → faders.
        epm_a     .tick(t);
        epm_b     .tick(t);
        voter     .tick(t);
        if (premotor_active) premotor.tick(t);
        controller.tick(t);
        fader_left.tick(t);
        fader_right.tick(t);
        bus.end_tick();
    }

    std::shared_ptr<const ogma::ActionOut> last(std::string const& topic) const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(bus.last_value(topic));
    }
    std::shared_ptr<const ogma::FaderState> last_fader() const {
        return std::dynamic_pointer_cast<const ogma::FaderState>(
            bus.last_value(ogma::topics::kMotorFaderAlpha));
    }
};

}  // namespace

// =============================================================================
// 1. The validation contract: with α=1 and no brain in the graph, both
//    action channels publish 0 — the differential_paddler body would
//    freeze.  This is the "empty graph = frozen agent" promise.
// =============================================================================

TEST(Phase66G_E2E, EmptyBrainAtAlphaOneFreezes) {
    // surprise_gain doesn't matter when alpha_source="fixed".
    // include_premotor=false simulates "user ablated the brain via patch UI".
    E2EFixture f(/*surprise_gain=*/0.0, "fixed",
                 /*alpha_fixed=*/1.0,
                 /*include_premotor=*/false);

    for (uint64_t t = 0; t < 30; ++t) {
        // Reflex publishes non-zero values, but α=1 must completely
        // suppress them.  brain.* topics never get a publisher.
        f.run_tick(t, stable_input(t, "a"), stable_input(t, "b"),
                   /*reflex_left=*/+3.0f, /*reflex_right=*/-2.0f);
    }
    auto al = f.last("action.left");
    auto ar = f.last("action.right");
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    EXPECT_NEAR(al->accel, 0.0f, 1e-5f)
        << "α=1 with no brain publisher must publish 0 to action.left";
    EXPECT_NEAR(ar->accel, 0.0f, 1e-5f)
        << "α=1 with no brain publisher must publish 0 to action.right";
    auto fs = f.last_fader();
    ASSERT_NE(fs, nullptr);
    EXPECT_NEAR(fs->alpha, 1.0f, 1e-5f);
}

// =============================================================================
// 2. α=0 → both channels equal the reflex publishes exactly.
// =============================================================================

TEST(Phase66G_E2E, AlphaZeroPassesReflexBilaterally) {
    E2EFixture f(/*surprise_gain=*/0.0, "fixed",
                 /*alpha_fixed=*/0.0,
                 /*include_premotor=*/true);

    for (uint64_t t = 0; t < 30; ++t) {
        float lr = -2.5f + 0.1f * float(t);
        float rr =  1.0f - 0.05f * float(t);
        f.run_tick(t, stable_input(t, "a"), stable_input(t, "b"), lr, rr);
        auto al = f.last("action.left");
        auto ar = f.last("action.right");
        ASSERT_NE(al, nullptr);
        ASSERT_NE(ar, nullptr);
        EXPECT_NEAR(al->accel, lr, 1e-4f);
        EXPECT_NEAR(ar->accel, rr, 1e-4f);
    }
}

// =============================================================================
// 3. Surprise-driven α evolves coherently across both channels.  Both
//    faders read the same FaderState so their α must always match.
// =============================================================================

TEST(Phase66G_E2E, SurpriseDrivenAlphaEvolvesCoherently) {
    E2EFixture f_warm(/*surprise_gain=*/0.5, "surprise");
    for (uint64_t t = 0; t < 250; ++t)
        f_warm.run_tick(t, stable_input(t, "a"), stable_input(t, "b"), -2.0f, +2.0f);
    auto fs_warm = f_warm.last_fader();
    ASSERT_NE(fs_warm, nullptr);
    float alpha_warm = fs_warm->alpha;
    EXPECT_GT(alpha_warm, 0.4f)
        << "After 250 stable ticks α should rise above 0.4 (got " << alpha_warm << ")";

    // Both faders saw the same α this tick — verify by checking that the
    // blend math is consistent with that α on both channels.  Brain and
    // reflex outputs are noisy but the EFFECTIVE α used must match.
    // We use a tick where the brain's accel happens to be non-zero on
    // both sides (any bilateral Premotor publish guarantees that).
    auto al = f_warm.last("action.left");
    auto ar = f_warm.last("action.right");
    auto bl = f_warm.last("action.brain.left");
    auto br = f_warm.last("action.brain.right");
    auto rl = f_warm.last("action.reflex.left");
    auto rr = f_warm.last("action.reflex.right");
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    ASSERT_NE(bl, nullptr);
    ASSERT_NE(br, nullptr);
    ASSERT_NE(rl, nullptr);
    ASSERT_NE(rr, nullptr);
    // out = α·brain + (1-α)·reflex
    float a_left  = (rl->accel == bl->accel) ? alpha_warm
                                              : (al->accel - rl->accel) / (bl->accel - rl->accel);
    float a_right = (rr->accel == br->accel) ? alpha_warm
                                              : (ar->accel - rr->accel) / (br->accel - rr->accel);
    EXPECT_NEAR(a_left,  alpha_warm, 1e-3f);
    EXPECT_NEAR(a_right, alpha_warm, 1e-3f);

    // Now shock and verify α drops on both sides simultaneously.
    for (uint64_t t = 250; t < 320; ++t)
        f_warm.run_tick(t, shock_input(t, "a"), shock_input(t, "b"), -2.0f, +2.0f);
    auto fs_shock = f_warm.last_fader();
    ASSERT_NE(fs_shock, nullptr);
    EXPECT_LT(fs_shock->alpha, alpha_warm)
        << "Shock should reduce α below the pre-shock value (pre=" << alpha_warm
        << " post=" << fs_shock->alpha << ")";
}

// =============================================================================
// 4. Bit-identical-numbers safeguard: same wiring, same input, the only
//    knob is surprise_gain (0 vs >0).  The action streams MUST differ
//    measurably.  If identical, surprise modulation is not flowing
//    through to motor output (bug discovered post-Phase 6.5.37).
// =============================================================================

TEST(Phase66G_E2E, BitIdenticalSafeguard_SurpriseGainChangesAction) {
    E2EFixture f_off(/*surprise_gain=*/0.0, "surprise");
    E2EFixture f_on (/*surprise_gain=*/0.5, "surprise");

    std::vector<float> trace_off_left, trace_on_left;
    std::vector<float> trace_off_right, trace_on_right;
    for (uint64_t t = 0; t < 200; ++t) {
        f_off.run_tick(t, stable_input(t, "a"), stable_input(t, "b"), -2.0f, +2.0f);
        f_on .run_tick(t, stable_input(t, "a"), stable_input(t, "b"), -2.0f, +2.0f);
        if (auto a = f_off.last("action.left"))  trace_off_left .push_back(a->accel);
        if (auto a = f_on .last("action.left"))  trace_on_left  .push_back(a->accel);
        if (auto a = f_off.last("action.right")) trace_off_right.push_back(a->accel);
        if (auto a = f_on .last("action.right")) trace_on_right .push_back(a->accel);
    }
    ASSERT_EQ(trace_off_left.size(), trace_on_left.size());
    ASSERT_EQ(trace_off_right.size(), trace_on_right.size());
    ASSERT_GT(trace_off_left.size(), 100u);

    // Mean abs diff over back half (post-warmup) on each channel.
    auto mean_abs_diff = [](std::vector<float> const& a, std::vector<float> const& b){
        int n = int(a.size());
        int start = n / 2;
        float s = 0.0f;
        for (int i = start; i < n; ++i) s += std::abs(a[i] - b[i]);
        return s / float(n - start);
    };
    float mad_left  = mean_abs_diff(trace_off_left,  trace_on_left);
    float mad_right = mean_abs_diff(trace_off_right, trace_on_right);

    EXPECT_GT(mad_left, 1e-3f)
        << "BIT-IDENTICAL action.left streams across surprise_gain=0 vs 0.5; "
           "(mean |Δ| over back half = " << mad_left << ")";
    EXPECT_GT(mad_right, 1e-3f)
        << "BIT-IDENTICAL action.right streams across surprise_gain=0 vs 0.5; "
           "(mean |Δ| over back half = " << mad_right << ")";
}
