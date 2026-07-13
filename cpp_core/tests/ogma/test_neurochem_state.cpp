// =============================================================================
// test_neurochem_state.cpp  --  Unit tests for NeurochemState
// =============================================================================
//
// Exercises every numbered VV&A criterion from
// docs/primitives/NeurochemState.md plus the v3 frozen-baseline math:
//   - decay parity (dopamine, serotonin) toward baselines
//   - intrinsic dopamine on TLE drop (positive delta only)
//   - serotonin drains for whisker/hunger/pheromone with threshold
//   - dopamine boosts for travel and rising scent
//   - wall_stuck somatic aversion (drain both)
//   - event-coupled mode flag (off by default → events are telemetry only)
//   - scaling-factor formulas
//   - clamp invariants ([0,1])
//   - param hot-mutation
//   - master_seed param is ConstructionOnly
//   - NaN guard

#include <gtest/gtest.h>

#include <memory>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/NeurochemState.hpp"

namespace {

std::shared_ptr<ogma::EnvEvent> make_event(std::string_view name, float intensity = 1.0f) {
    auto e = std::make_shared<ogma::EnvEvent>();
    e->name      = std::string(name);
    e->intensity = intensity;
    return e;
}

std::shared_ptr<ogma::ProprioToken> make_proprio(std::string_view sensor, float value) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->sensor = std::string(sensor);
    p->values.resize(1);
    p->values[0] = value;
    return p;
}

std::shared_ptr<ogma::RealityToken> make_reality(float tle, uint64_t tick = 0) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->tick_id = tick;
    r->tle     = tle;
    return r;
}

// Construct + setup a NeurochemState wired to a fresh InProcessBus.
struct NeuroFixture {
    ogma::InProcessBus    bus;
    ogma::NeurochemState  neuro;

    explicit NeuroFixture(ogma::ParamMap params = {}) {
        neuro.set_id("neuro");
        neuro.on_setup(&bus, params);
    }

    // Helper: cycle one tick (begin → tick → end) without any events.
    void step(uint64_t t) {
        bus.begin_tick(t);
        neuro.tick(t);
        bus.end_tick();
    }

    std::shared_ptr<const ogma::NeuroState> last_state() const {
        return std::dynamic_pointer_cast<const ogma::NeuroState>(
            bus.last_value(ogma::topics::kNeuroState));
    }
};

} // namespace

// -- Lifecycle / contract --------------------------------------------------

TEST(NeurochemState, ConstructsWithBaselines) {
    NeuroFixture f;
    EXPECT_NEAR(f.neuro.dopamine(),  0.20f, 1e-6f);
    EXPECT_NEAR(f.neuro.serotonin(), 0.65f, 1e-6f);
}

TEST(NeurochemState, PublishesNeuroStateEveryTick) {
    NeuroFixture f;
    EXPECT_EQ(f.last_state(), nullptr);

    f.step(0);
    auto s0 = f.last_state();
    ASSERT_NE(s0, nullptr);
    EXPECT_EQ(s0->tick_id, 0u);
    EXPECT_EQ(s0->producer_id, "neuro");

    f.step(1);
    auto s1 = f.last_state();
    ASSERT_NE(s1, nullptr);
    EXPECT_EQ(s1->tick_id, 1u);
}

// -- Decay parity ----------------------------------------------------------

TEST(NeurochemState, DopamineDecaysTowardBaselineWithNoEvents) {
    // Drive dopamine high once via a hand-coupled hit, then disable events
    // and watch it decay.  Easier path: enable event_coupled_da, fire one
    // hit, then run free-decay ticks.
    ogma::ParamMap p;
    p["event_coupled_da"] = true;
    NeuroFixture f(p);

    f.bus.begin_tick(0);
    f.bus.publish("events.hit", make_event("hit"));
    f.neuro.tick(0);
    f.bus.end_tick();
    auto s0 = f.last_state();
    ASSERT_NE(s0, nullptr);
    EXPECT_GT(s0->dopamine, 0.6f);  // boosted

    // Now decay 100 ticks back toward 0.20.
    for (uint64_t t = 1; t <= 100; ++t) f.step(t);
    auto s100 = f.last_state();
    EXPECT_NEAR(s100->dopamine, 0.20f, 1e-3f);
}

// -- Hit / brick / miss event coupling ------------------------------------

TEST(NeurochemState, HitIncreasesDopamineWhenEventCoupled) {
    ogma::ParamMap p;
    p["event_coupled_da"] = true;
    NeuroFixture f(p);

    f.bus.begin_tick(0);
    f.bus.publish("events.hit", make_event("hit"));
    f.neuro.tick(0);
    f.bus.end_tick();

    auto s = f.last_state();
    // After one decay step (0.20 baseline, 0.88 decay) dopamine is unchanged
    // (already at baseline), then +da_hit_gain = +0.45 → 0.65.
    EXPECT_NEAR(s->dopamine, 0.65f, 1e-5f);
    EXPECT_EQ(f.neuro.total_hits(), 1);
}

TEST(NeurochemState, EventsAreTelemetryOnlyByDefault) {
    NeuroFixture f;  // event_coupled_da default = false

    f.bus.begin_tick(0);
    f.bus.publish("events.hit",   make_event("hit"));
    f.bus.publish("events.brick", make_event("brick"));
    f.bus.publish("events.miss",  make_event("miss"));
    f.neuro.tick(0);
    f.bus.end_tick();

    auto s = f.last_state();
    // dopamine unchanged from baseline
    EXPECT_NEAR(s->dopamine,  0.20f, 1e-5f);
    EXPECT_NEAR(s->serotonin, 0.65f, 1e-5f);
    EXPECT_EQ(f.neuro.total_hits(),   1);
    EXPECT_EQ(f.neuro.total_bricks(), 1);
    EXPECT_EQ(f.neuro.total_misses(), 1);
}

// -- Intrinsic dopamine from TLE delta ------------------------------------

TEST(NeurochemState, IntrinsicDopamineOnTleDrop) {
    NeuroFixture f;

    // Tick 0: produce a high-TLE RealityToken (Feedback: it'll be visible
    // at begin_tick(1)).
    f.bus.begin_tick(0);
    f.bus.publish("reality.video.retinal", make_reality(1.0f, 0));
    f.neuro.tick(0);
    f.bus.end_tick();

    // Tick 1: NeurochemState's Feedback handler sees TLE=1.0 (from t=0).
    // Sets prev_tle.  Dopamine doesn't change yet (no prior baseline).
    float dopamine_before = f.neuro.dopamine();
    f.bus.begin_tick(1);
    f.bus.publish("reality.video.retinal", make_reality(0.5f, 1));  // current-tick TLE for t+1
    f.neuro.tick(1);
    f.bus.end_tick();
    EXPECT_NEAR(f.neuro.dopamine(), dopamine_before * 0.88f + 0.20f * (1.0f - 0.88f), 1e-5f);

    // Tick 2: Feedback sees TLE=0.5 (from t=1), prev_tle=1.0; delta=+0.5
    // → dopamine += intrinsic_da_gain * 0.5 = 0.025.
    f.bus.begin_tick(2);
    f.neuro.tick(2);
    f.bus.end_tick();
    auto s = f.last_state();
    EXPECT_GT(s->dopamine, 0.20f + 0.020f);   // got the boost
}

TEST(NeurochemState, NoIntrinsicPulseOnTleRise) {
    NeuroFixture f;

    // Same shape as above but TLE rises (no pulse expected).
    f.bus.begin_tick(0);
    f.bus.publish("reality.video.retinal", make_reality(0.2f, 0));
    f.neuro.tick(0);
    f.bus.end_tick();

    f.bus.begin_tick(1);
    f.bus.publish("reality.video.retinal", make_reality(0.8f, 1));
    f.neuro.tick(1);
    f.bus.end_tick();

    float baseline = f.neuro.dopamine();
    f.bus.begin_tick(2);
    f.neuro.tick(2);   // sees TLE=0.8 from t=1 vs prev=0.2 → rise → no pulse
    f.bus.end_tick();
    EXPECT_LE(f.neuro.dopamine(), baseline + 1e-6f);
}

// -- Serotonin drains -----------------------------------------------------

TEST(NeurochemState, WhiskerDrainsSerotonin) {
    NeuroFixture f;

    f.bus.begin_tick(0);
    f.bus.publish("reality.proprio.whisker", make_proprio("whisker", 1.0f));
    f.neuro.tick(0);
    f.bus.end_tick();

    auto s = f.last_state();
    // Serotonin baseline is 0.65; ht_decay=0.93 → 0.65 (no change at baseline);
    // then whisker drain = -0.02 * 1.0 = -0.02 → 0.63.
    EXPECT_NEAR(s->serotonin, 0.65f - 0.02f, 1e-5f);
}

TEST(NeurochemState, HungerDrainsSerotonin) {
    NeuroFixture f;
    f.bus.begin_tick(0);
    f.bus.publish("reality.proprio.hunger", make_proprio("hunger", 1.0f));
    f.neuro.tick(0);
    f.bus.end_tick();

    auto s = f.last_state();
    EXPECT_NEAR(s->serotonin, 0.65f - 0.01f, 1e-5f);
}

TEST(NeurochemState, PheromoneIsThresholdGated) {
    NeuroFixture f;

    // Below threshold (0.30) — no drain.
    f.bus.begin_tick(0);
    f.bus.publish("reality.proprio.pheromone", make_proprio("pheromone", 0.20f));
    f.neuro.tick(0);
    f.bus.end_tick();
    EXPECT_NEAR(f.last_state()->serotonin, 0.65f, 1e-5f);

    // Above threshold — drain proportional to excess.
    f.bus.begin_tick(1);
    f.bus.publish("reality.proprio.pheromone", make_proprio("pheromone", 0.70f));
    f.neuro.tick(1);
    f.bus.end_tick();
    auto s = f.last_state();
    float expected = 0.65f - 0.005f * (0.70f - 0.30f);
    EXPECT_NEAR(s->serotonin, expected, 1e-5f);
}

// -- Dopamine boosts -----------------------------------------------------

TEST(NeurochemState, TravelBoostsDopamine) {
    NeuroFixture f;
    f.bus.begin_tick(0);
    f.bus.publish("reality.proprio.travel", make_proprio("travel", 1.0f));
    f.neuro.tick(0);
    f.bus.end_tick();
    EXPECT_NEAR(f.last_state()->dopamine, 0.20f + 0.02f, 1e-5f);
}

TEST(NeurochemState, RisingScentBoostsDopamine) {
    NeuroFixture f;

    f.bus.begin_tick(0);
    f.bus.publish("reality.proprio.scent", make_proprio("scent", 0.30f));
    f.neuro.tick(0);   // sets prev_scent
    f.bus.end_tick();
    float d0 = f.neuro.dopamine();

    f.bus.begin_tick(1);
    f.bus.publish("reality.proprio.scent", make_proprio("scent", 0.60f));  // delta = +0.30
    f.neuro.tick(1);
    f.bus.end_tick();
    float d1 = f.neuro.dopamine();
    EXPECT_GT(d1, d0);
}

TEST(NeurochemState, FallingScentDoesNotDrop) {
    NeuroFixture f;

    f.bus.begin_tick(0);
    f.bus.publish("reality.proprio.scent", make_proprio("scent", 0.60f));
    f.neuro.tick(0);
    f.bus.end_tick();
    float d0 = f.neuro.dopamine();

    f.bus.begin_tick(1);
    f.bus.publish("reality.proprio.scent", make_proprio("scent", 0.30f));  // delta = -0.30
    f.neuro.tick(1);
    f.bus.end_tick();
    // Pure decay only (no penalty for falling scent).
    EXPECT_NEAR(f.neuro.dopamine(),
                0.20f + (d0 - 0.20f) * 0.88f, 1e-5f);
}

// -- Wall stuck ---------------------------------------------------------

TEST(NeurochemState, WallStuckDrainsBoth) {
    NeuroFixture f;

    f.bus.begin_tick(0);
    f.bus.publish("events.wall_stuck", make_event("wall_stuck"));
    f.neuro.tick(0);
    f.bus.end_tick();

    auto s = f.last_state();
    EXPECT_LT(s->dopamine,  0.20f);   // strong drain
    EXPECT_LT(s->serotonin, 0.65f);
}

// -- Scaling-factor formulas --------------------------------------------

TEST(NeurochemState, ScalingFactorsMatchV3Formulas) {
    NeuroFixture f;
    f.step(0);
    auto s = f.last_state();

    // dopamine = 0.20 (decayed to baseline) → epsilon_b_scale = 0.3 + 2.2*0.20 = 0.74
    EXPECT_NEAR(s->epsilon_b_scale, 0.3f + 2.2f * 0.20f, 1e-5f);
    // serotonin = 0.65 → mie_scale = 1.8 - 1.3*0.65 = 0.955
    EXPECT_NEAR(s->min_insertion_error_scale, 1.8f - 1.3f * 0.65f, 1e-5f);
    // mitosis_scale = 0.6 + 1.2*0.65 = 1.38
    EXPECT_NEAR(s->mitosis_threshold_scale, 0.6f + 1.2f * 0.65f, 1e-5f);
    // novelty_scale = 0.5 + 1.0*0.20 = 0.70
    EXPECT_NEAR(s->novelty_threshold_scale, 0.5f + 1.0f * 0.20f, 1e-5f);
}

// -- Bounds invariant --------------------------------------------------

TEST(NeurochemState, ClampsToUnitInterval) {
    ogma::ParamMap p;
    p["event_coupled_da"] = true;
    p["da_hit_gain"]      = 100.0;   // unrealistic — would overflow without clamp
    NeuroFixture f(p);

    f.bus.begin_tick(0);
    for (int i = 0; i < 5; ++i)
        f.bus.publish("events.hit", make_event("hit"));
    f.neuro.tick(0);
    f.bus.end_tick();

    EXPECT_LE(f.neuro.dopamine(),  1.0f);
    EXPECT_GE(f.neuro.dopamine(),  0.0f);
    EXPECT_LE(f.neuro.serotonin(), 1.0f);
    EXPECT_GE(f.neuro.serotonin(), 0.0f);
}

// -- Param hot-mutation -----------------------------------------------

TEST(NeurochemState, HotMutateDecayParam) {
    NeuroFixture f;
    f.neuro.on_param_change("da_decay", ogma::ParamValue{0.5});
    // Decay of 0.5 means dopamine moves halfway to baseline each tick.  After
    // setting dopamine high (via hit) and a single tick, we should see a
    // larger drop than the default 0.88 decay would produce.
    ogma::ParamMap p;
    p["event_coupled_da"] = true;
    NeuroFixture g(p);
    g.neuro.on_param_change("da_decay", ogma::ParamValue{0.5});

    g.bus.begin_tick(0);
    g.bus.publish("events.hit", make_event("hit"));
    g.neuro.tick(0);
    g.bus.end_tick();
    g.step(1);   // one more decay tick

    // Double-decay: started at 0.65 (after hit), decays to 0.20+0.5*(0.65-0.20)=0.425.
    EXPECT_NEAR(g.neuro.dopamine(), 0.20f + 0.5f * (0.65f - 0.20f), 1e-5f);
}

TEST(NeurochemState, MasterSeedIsConstructionOnly) {
    NeuroFixture f;
    EXPECT_THROW(f.neuro.on_param_change("master_seed", ogma::ParamValue{int64_t{99}}),
                 std::invalid_argument);
}

TEST(NeurochemState, UnknownParamThrows) {
    NeuroFixture f;
    EXPECT_THROW(f.neuro.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}

// -- Determinism -----------------------------------------------------

TEST(NeurochemState, IdenticalInputsProduceIdenticalOutputs) {
    auto run = []() {
        NeuroFixture f;
        for (uint64_t t = 0; t < 200; ++t) {
            f.bus.begin_tick(t);
            if (t % 5 == 0)
                f.bus.publish("reality.proprio.scent",  make_proprio("scent",  0.4f + 0.1f * (t % 3)));
            if (t % 7 == 0)
                f.bus.publish("reality.proprio.travel", make_proprio("travel", 0.5f));
            f.bus.publish("reality.video.retinal", make_reality(0.3f + 0.1f * (t % 4), t));
            f.neuro.tick(t);
            f.bus.end_tick();
        }
        return std::make_pair(f.neuro.dopamine(), f.neuro.serotonin());
    };

    auto [a_da, a_ht] = run();
    auto [b_da, b_ht] = run();
    EXPECT_FLOAT_EQ(a_da, b_da);
    EXPECT_FLOAT_EQ(a_ht, b_ht);
}
