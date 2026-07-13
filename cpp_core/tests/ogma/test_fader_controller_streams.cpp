// =============================================================================
// test_fader_controller_streams.cpp
//   Phase 6.6.G — FaderController behavioral / stream tests.
//
// Four tests cover the contract of the α-only module that 6.6.G splits
// out of MotorFader:
//   1. Surprise-driven α      — α tracks 1 − aggregated surprise.
//   2. Fixed mode operator override — alpha_fixed clamped to strict [0,1]
//                              even when alpha_min/alpha_max are tighter.
//   3. Smoothing damps jitter — alternating-surprise input does not
//                              produce alternating-α output.
//   4. Missing-consensus safety — cold start in surprise mode publishes
//                              α=alpha_min every tick without crashing.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/FaderController.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::ConsensusToken>
make_consensus(std::unordered_map<std::string, float> surprise) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding = Eigen::VectorXf::Zero(4);
    c->surprise_ema    = std::move(surprise);
    return c;
}

ogma::ParamMap base_params() {
    return {
        {"alpha_source",     std::string("fixed")},
        {"alpha_fixed",      0.0},
        {"alpha_smoothing",  1.0},      // immediate response by default
        {"alpha_min",        0.0},
        {"alpha_max",        1.0},
    };
}

struct Fixture {
    ogma::InProcessBus     bus;
    ogma::FaderController  ctrl;
    explicit Fixture(ogma::ParamMap const& p) {
        ctrl.set_id("ctrl");
        ctrl.on_setup(&bus, p);
    }
    void run_tick(uint64_t t,
                  std::shared_ptr<ogma::ConsensusToken> cons = nullptr) {
        bus.begin_tick(t);
        if (cons) bus.publish("consensus.0", cons);
        ctrl.tick(t);
        bus.end_tick();
    }
    std::shared_ptr<const ogma::FaderState> last_fader() const {
        return std::dynamic_pointer_cast<const ogma::FaderState>(
            bus.last_value(ogma::topics::kMotorFaderAlpha));
    }
};

} // namespace

// =============================================================================
// 1. Surprise-driven α: with smoothing=1 the smoothed α equals 1 − surprise
//    on every tick, modulo the alpha_min/alpha_max clamp.
// =============================================================================

TEST(FaderControllerStreams, SurpriseDrivesAlphaInverse) {
    auto p = base_params();
    p["alpha_source"]    = std::string("surprise");
    p["alpha_smoothing"] = 1.0;
    Fixture f(p);

    // Cold start: no consensus yet → α stays at alpha_min (0).
    f.run_tick(0);
    auto fs0 = f.last_fader();
    ASSERT_NE(fs0, nullptr);
    EXPECT_NEAR(fs0->alpha, 0.0f, 1e-5f);
    EXPECT_EQ(fs0->source, "surprise");

    // Sustained low surprise → α near 1.
    for (uint64_t t = 1; t <= 5; ++t) {
        f.run_tick(t, make_consensus({{"video.retinal", 0.1f}, {"audio.stft", 0.1f}}));
    }
    auto fs_low = f.last_fader();
    ASSERT_NE(fs_low, nullptr);
    EXPECT_NEAR(fs_low->alpha, 0.9f, 1e-5f)
        << "Mean surprise 0.1 → α target 0.9; smoothing=1 → applied immediately";
    EXPECT_NEAR(fs_low->surprise_scalar, 0.1f, 1e-5f);

    // Surprise spike → α drops near 0.
    for (uint64_t t = 6; t <= 10; ++t) {
        f.run_tick(t, make_consensus({{"video.retinal", 0.95f}, {"audio.stft", 0.95f}}));
    }
    auto fs_high = f.last_fader();
    ASSERT_NE(fs_high, nullptr);
    EXPECT_NEAR(fs_high->alpha, 0.05f, 1e-5f);
}

// =============================================================================
// 2. Fixed-mode operator override: alpha_fixed clamps to strict [0, 1] and
//    bypasses the alpha_min/alpha_max safety floors.  This is the bug fix
//    that landed in 6.6.F.x ("M=full brain" was bleeding 5% reflex).
// =============================================================================

TEST(FaderControllerStreams, FixedModeOverrideClampsStrictUnitInterval) {
    auto p = base_params();
    p["alpha_source"]    = std::string("fixed");
    p["alpha_smoothing"] = 1.0;
    p["alpha_min"]       = 0.05;
    p["alpha_max"]       = 0.95;
    p["alpha_fixed"]     = 1.0;
    Fixture f(p);

    f.run_tick(0);
    auto fs = f.last_fader();
    ASSERT_NE(fs, nullptr);
    EXPECT_NEAR(fs->alpha, 1.0f, 1e-5f)
        << "Fixed-mode operator override must reach strict 1.0, not alpha_max=0.95";

    f.ctrl.on_param_change("alpha_fixed", ogma::ParamValue{0.0});
    f.run_tick(1);
    fs = f.last_fader();
    ASSERT_NE(fs, nullptr);
    EXPECT_NEAR(fs->alpha, 0.0f, 1e-5f)
        << "Fixed-mode operator override must reach strict 0.0, not alpha_min=0.05";

    // Out-of-range values clamp to [0, 1].
    f.ctrl.on_param_change("alpha_fixed", ogma::ParamValue{2.5});
    f.run_tick(2);
    fs = f.last_fader();
    EXPECT_NEAR(fs->alpha, 1.0f, 1e-5f);

    f.ctrl.on_param_change("alpha_fixed", ogma::ParamValue{-1.0});
    f.run_tick(3);
    fs = f.last_fader();
    EXPECT_NEAR(fs->alpha, 0.0f, 1e-5f);
}

// =============================================================================
// 3. Smoothing damps jitter: under alternating-surprise input, α should
//    NOT alternate — it should drift toward the long-run mean and stay
//    within a much tighter band than the raw α_target.
// =============================================================================

TEST(FaderControllerStreams, SmoothingDampensJitter) {
    auto p = base_params();
    p["alpha_source"]    = std::string("surprise");
    p["alpha_smoothing"] = 0.05;       // gentle EMA
    Fixture f(p);

    // Warm up to mid-band by alternating high/low surprise for a long run.
    float min_alpha =  std::numeric_limits<float>::infinity();
    float max_alpha = -std::numeric_limits<float>::infinity();
    for (uint64_t t = 0; t < 200; ++t) {
        bool hi = (t % 2 == 0);
        f.run_tick(t, make_consensus({{"m", hi ? 0.9f : 0.1f}}));
    }
    // Sample α-band over the last 50 ticks (well past warm-up).
    for (uint64_t t = 200; t < 250; ++t) {
        bool hi = (t % 2 == 0);
        f.run_tick(t, make_consensus({{"m", hi ? 0.9f : 0.1f}}));
        auto fs = f.last_fader();
        ASSERT_NE(fs, nullptr);
        if (fs->alpha < min_alpha) min_alpha = fs->alpha;
        if (fs->alpha > max_alpha) max_alpha = fs->alpha;
    }
    // Raw α_target swings from 0.1 to 0.9 (band = 0.8).  With α_smoothing
    // = 0.05, the steady-state band is much tighter — well under 0.2.
    float band = max_alpha - min_alpha;
    EXPECT_LT(band, 0.20f)
        << "Smoothing failed to damp alternating-surprise jitter; band = " << band;
    EXPECT_GT(band, 0.0f) << "Smoothing should not freeze α completely";
}

// =============================================================================
// 4. Missing-consensus safety: in surprise mode with no consensus ever
//    arriving, α must stay at alpha_min and FaderState must publish each
//    tick without throwing or producing NaN.
// =============================================================================

TEST(FaderControllerStreams, MissingConsensusStaysAtAlphaMin) {
    auto p = base_params();
    p["alpha_source"]    = std::string("surprise");
    p["alpha_smoothing"] = 0.5;
    p["alpha_min"]       = 0.10;
    p["alpha_max"]       = 0.90;
    Fixture f(p);

    for (uint64_t t = 0; t < 30; ++t) {
        ASSERT_NO_THROW(f.run_tick(t));
        auto fs = f.last_fader();
        ASSERT_NE(fs, nullptr);
        EXPECT_FALSE(std::isnan(fs->alpha));
        EXPECT_NEAR(fs->alpha, 0.10f, 1e-5f)
            << "Surprise mode without consensus must hold α at alpha_min";
        EXPECT_NEAR(fs->surprise_scalar, 0.0f, 1e-5f);
    }
    EXPECT_EQ(f.ctrl.publish_count(), 30);
}

// =============================================================================
// Phase 6.6.N — learned α setpoint via reward feedback.
// =============================================================================

namespace {

void publish_event(ogma::InProcessBus& bus, uint64_t t,
                    std::string const& name, float intensity = 1.0f) {
    auto e = std::make_shared<ogma::EnvEvent>();
    e->tick_id   = t;
    e->name      = name;
    e->intensity = intensity;
    bus.publish("events." + name, e);
}

}  // namespace

TEST(FaderControllerStreams, LearnedSetpointInitFromParam) {
    auto p = base_params();
    p["alpha_source"]                = std::string("surprise");
    p["alpha_smoothing"]             = 1.0;
    p["alpha_min"]                   = 0.0;
    p["alpha_max"]                   = 1.0;
    p["learned_alpha_lr"]            = 0.01;
    p["learned_alpha_setpoint_init"] = 0.5;
    Fixture f(p);
    EXPECT_NEAR(f.ctrl.learned_setpoint(), 0.5f, 1e-6f)
        << "Init param value should populate learned_setpoint at construction";

    // Also test a non-default init.
    auto p2 = p;
    p2["learned_alpha_setpoint_init"] = 0.8;
    Fixture f2(p2);
    EXPECT_NEAR(f2.ctrl.learned_setpoint(), 0.8f, 1e-6f);
}

TEST(FaderControllerStreams, LearnedSetpointBypassesFamiliarityWhenLrPositive) {
    // Setpoint mode + non-zero surprise → α_target = setpoint * (1 - surprise).
    // Must NOT use familiarity coupling (which would drag α down over time).
    auto p = base_params();
    p["alpha_source"]                = std::string("surprise");
    p["alpha_smoothing"]             = 1.0;       // immediate
    p["alpha_min"]                   = 0.0;
    p["alpha_max"]                   = 1.0;
    p["learned_alpha_lr"]            = 0.01;
    p["learned_alpha_setpoint_init"] = 0.5;
    p["pathway_alpha_coupling"]      = 1.0;       // would normally engage 6.6.K
    Fixture f(p);

    // Drive a stream with low surprise.  Familiarity coupling at 1.0
    // would normally drop α toward 0; learned mode should ignore it
    // and keep α at setpoint*(1-surprise) ≈ setpoint.
    auto cons = make_consensus({{"reality.test", 0.05f}});  // low surprise
    for (uint64_t t = 0; t < 20; ++t) {
        f.run_tick(t, cons);
    }
    auto fs = f.last_fader();
    ASSERT_NE(fs, nullptr);
    EXPECT_NEAR(fs->alpha, 0.5f * (1.0f - 0.05f), 1e-3f)
        << "Learned mode must use setpoint*(1-surprise), not 6.6.K familiarity"
           " (familiarity would have dropped α toward 0).";
}

TEST(FaderControllerStreams, LearnedSetpointDriftsTowardRewardingRegime) {
    // The setpoint update is gradient ascent on Cov(α, reward) using
    // deviations from running means.  To drive the gradient we
    // alternate α between two regimes via on_param_change("alpha_fixed",..)
    // and inject reward only during the HIGH-α regime.  Then α_long_ema
    // settles between the two regimes (≈ 0.5), and during high-α ticks
    // (α - α_long_ema) > 0 with positive reward → setpoint UP.
    auto p = base_params();
    p["alpha_source"]                = std::string("fixed");
    p["alpha_fixed"]                 = 0.5;
    p["alpha_smoothing"]             = 1.0;
    p["alpha_min"]                   = 0.0;
    p["alpha_max"]                   = 1.0;
    p["learned_alpha_lr"]            = 0.5;        // strong learning rate
    p["learned_alpha_setpoint_init"] = 0.5;
    p["reward_alpha"]                = 0.2;        // reward EMA window ~5 ticks
    p["alpha_long_alpha"]            = 0.1;        // α EMA window ~10 ticks
    p["reward_weight_hit"]           = 1.0;
    Fixture f(p);

    auto run_phase = [&](uint64_t start, int len, float alpha_value, bool hit) {
        f.ctrl.on_param_change("alpha_fixed", ogma::ParamValue{double(alpha_value)});
        for (uint64_t t = start; t < start + uint64_t(len); ++t) {
            f.bus.begin_tick(t);
            if (hit) publish_event(f.bus, t, "hit", 1.0f);
            f.ctrl.tick(t);
            f.bus.end_tick();
        }
    };

    float setpoint_before = f.ctrl.learned_setpoint();
    // Alternate high-α + hit / low-α + no-event for many cycles so
    // covariance accumulates: high-α correlated with hit → setpoint UP.
    uint64_t t = 0;
    for (int cycle = 0; cycle < 30; ++cycle) {
        run_phase(t, 5, 0.9f, /*hit=*/true);  t += 5;
        run_phase(t, 5, 0.1f, /*hit=*/false); t += 5;
    }
    EXPECT_GT(f.ctrl.learned_setpoint(), setpoint_before + 1e-3f)
        << "α-high-correlates-with-hit covariance should drift setpoint UP "
           "(before=" << setpoint_before
        << " after=" << f.ctrl.learned_setpoint() << ")";

    // Negative case: hits arrive during LOW-α regime → setpoint drifts DOWN.
    Fixture f2(p);
    float setpoint_before2 = f2.ctrl.learned_setpoint();
    auto run_phase2 = [&](uint64_t start, int len, float alpha_value, bool hit) {
        f2.ctrl.on_param_change("alpha_fixed", ogma::ParamValue{double(alpha_value)});
        for (uint64_t tt = start; tt < start + uint64_t(len); ++tt) {
            f2.bus.begin_tick(tt);
            if (hit) publish_event(f2.bus, tt, "hit", 1.0f);
            f2.ctrl.tick(tt);
            f2.bus.end_tick();
        }
    };
    t = 0;
    for (int cycle = 0; cycle < 30; ++cycle) {
        run_phase2(t, 5, 0.1f, /*hit=*/true);  t += 5;
        run_phase2(t, 5, 0.9f, /*hit=*/false); t += 5;
    }
    EXPECT_LT(f2.ctrl.learned_setpoint(), setpoint_before2 - 1e-3f)
        << "α-low-correlates-with-hit covariance should drift setpoint DOWN "
           "(before=" << setpoint_before2
        << " after=" << f2.ctrl.learned_setpoint() << ")";
}

// =============================================================================
// Phase 6.6.Q boredom-α floor.  Three tests:
//   (a) gain=0 is bit-identical to the 6.6.K target — falsifiability check.
//   (b) Sustained high familiarity (which would normally floor α) plus
//       gain=0.3 pushes equilibrium α to ≈ gain/(1+gain) ≈ 0.23 ± clamp.
//   (c) Boredom decays once α has been high — single-direction stabiliser,
//       not a permanent additive bias.
// =============================================================================

TEST(FaderControllerStreams, BoredomGainZeroIsBitIdentical) {
    auto p = base_params();
    p["alpha_source"]           = std::string("surprise");
    p["alpha_smoothing"]        = 1.0;
    p["pathway_alpha_coupling"] = 1.0;     // 6.6.K active
    p["boredom_gain"]           = 0.0;     // OFF
    Fixture f(p);
    auto cons = make_consensus({{"reality.a", 0.4f}});
    for (uint64_t t = 0; t < 50; ++t) {
        f.run_tick(t, cons);
        auto fs = f.last_fader();
        ASSERT_NE(fs, nullptr);
        EXPECT_NEAR(fs->alpha, 0.6f, 1e-5f) << "tick " << t;
    }
    EXPECT_NEAR(f.ctrl.boredom_term(), 0.0f, 1e-6f);
}

TEST(FaderControllerStreams, BoredomLiftsAlphaFloorWhenChronicallyLow) {
    auto p = base_params();
    p["alpha_source"]           = std::string("surprise");
    p["alpha_smoothing"]        = 1.0;     // no temporal smoothing on alpha_
    p["alpha_long_alpha"]       = 0.05;
    p["pathway_alpha_coupling"] = 0.0;     // keep effective signal = surprise
    p["boredom_gain"]           = 0.3;
    Fixture f(p);
    // Sustained max surprise → effective=1 → 1-effective=0.  Without
    // boredom, alpha_target would clamp at 0.  With gain=0.3, equilibrium
    // alpha* satisfies α = gain*(1 - α) → α = gain/(1+gain) ≈ 0.231.
    auto cons = make_consensus({{"reality.a", 1.0f}});
    for (uint64_t t = 0; t < 2000; ++t) {
        f.run_tick(t, cons);
    }
    float alpha_eq = f.ctrl.alpha();
    float expected = 0.3f / 1.3f;
    EXPECT_NEAR(alpha_eq, expected, 0.02f)
        << "equilibrium α* should approach gain/(1+gain)≈0.231 (got " << alpha_eq << ")";
}

TEST(FaderControllerStreams, BoredomDecaysOnceAlphaIsHealthy) {
    auto p = base_params();
    p["alpha_source"]           = std::string("surprise");
    p["alpha_smoothing"]        = 1.0;
    p["alpha_long_alpha"]       = 0.05;
    p["pathway_alpha_coupling"] = 0.0;
    p["boredom_gain"]           = 0.5;
    Fixture f(p);
    // Drive α high via low surprise — boredom term should be small.
    auto cons_low = make_consensus({{"reality.a", 0.05f}});
    for (uint64_t t = 0; t < 500; ++t) {
        f.run_tick(t, cons_low);
    }
    EXPECT_GT(f.ctrl.alpha(), 0.9f);
    EXPECT_LT(f.ctrl.boredom_term(), 0.1f)
        << "Boredom should decay when α has been chronically high (alpha_long ≈ α high)";
}

TEST(FaderControllerStreams, LearnedAlphaLrZeroPreservesLegacy) {
    // With learned_alpha_lr=0, behaviour is bit-identical to 6.6.K.
    // Surprise + familiarity coupling drives α as before.
    auto p = base_params();
    p["alpha_source"]            = std::string("surprise");
    p["alpha_smoothing"]         = 1.0;
    p["learned_alpha_lr"]        = 0.0;     // OFF
    p["pathway_alpha_coupling"]  = 0.0;     // also legacy
    Fixture f(p);

    auto cons = make_consensus({{"reality.a", 0.3f}, {"reality.b", 0.5f}});
    f.run_tick(0, cons);
    auto fs = f.last_fader();
    ASSERT_NE(fs, nullptr);
    // Expected: α_target = 1 - mean(0.3, 0.5) = 0.6
    EXPECT_NEAR(fs->alpha, 0.6f, 1e-5f);
}
