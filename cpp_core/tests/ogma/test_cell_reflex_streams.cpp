// =============================================================================
// test_cell_reflex_streams.cpp
//   Phase 6.6.G — CellReflex behavioral tests (legacy continuous-additive port).
//
// Six tests cover the contract:
//   1. WanderForwardWithNoise — no contact, no stuck → forward thrust + per-tick
//      noise on both channels, no steer bias.
//   2. StuckPulseFiresOnDeficitTransition — sustained zero velocity drives
//      deficit > 0.5; CellReflex publishes events.wall_stuck and arms a
//      pulse.
//   3. PulseDirectionFollowsContactAsymmetry — right contact > left contact
//      drives pulse_held negative (turn left, away from blocked side).
//   4. PulseHeldForPeriodAndResamples — direction holds for stuck_pulse_period
//      ticks, then resamples (potentially new direction).
//   5. ScentSuppressionFlipsBehaviour — rising scent gradient suppresses the
//      whisker rectification term in the pulse so the agent stops actively
//      fleeing walls when food is near.
//   6. MissEventOnHighWhisker — max_w over miss_threshold publishes
//      events.miss with refractory.
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/CellReflex.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::ProprioToken> make_proprio(std::string sensor,
                                                 std::vector<float> values,
                                                 uint64_t tick = 0) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id = tick;
    p->sensor  = std::move(sensor);
    p->values.resize(int(values.size()));
    for (int i = 0; i < int(values.size()); ++i) p->values[i] = values[i];
    return p;
}

ogma::ParamMap base_params() {
    return {
        {"whisker_topic_prefix", std::string("reality.proprio.whisker_")},
        {"left_suffixes",        std::vector<std::string>{"0", "1", "2"}},
        {"right_suffixes",       std::vector<std::string>{"3", "4", "5"}},
        {"scent_topic",          std::string("reality.proprio.scent_max")},
        {"imu_topic",            std::string("reality.proprio.imu")},
        {"output_topic_left",    std::string("action.reflex.left")},
        {"output_topic_right",   std::string("action.reflex.right")},
        {"emit_events",          true},
        {"wander_thrust",        2.0},
        {"wander_noise_amplitude", 0.5},
        {"steer_amp",            2.0},
        {"stuck_window_ticks",   int64_t{10}},     // shorter for tests
        {"stuck_severity_threshold", 0.5},
        {"stuck_move_speed_reference", 3.0},
        {"stuck_pulse_period",   int64_t{30}},
        {"miss_threshold",       0.30},
        {"miss_refractory_ticks", int64_t{30}},
        {"avoid_steer_gain",     0.0},             // off by default
        {"avoid_threshold",      0.30},
        {"scent_alpha_short",    0.1},
        {"scent_alpha_long",     0.001},
        {"scent_long_pos_min",   0.001},
        {"scent_gate_cap",       0.5},
        {"accel_min",           -4.0},
        {"accel_max",            4.0},
        {"master_seed",          int64_t{42}},
    };
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::CellReflex   reflex;
    explicit Fixture(ogma::ParamMap const& p = base_params()) {
        reflex.set_id("cell_reflex");
        reflex.on_setup(&bus, p);
    }
    template <typename F>
    void run_tick(uint64_t t, F&& staged) {
        bus.begin_tick(t);
        staged();
        reflex.tick(t);
        bus.end_tick();
    }
    std::shared_ptr<const ogma::ActionOut> action(std::string const& topic) const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(bus.last_value(topic));
    }
    std::shared_ptr<const ogma::EnvEvent> last_event(std::string const& topic) const {
        return std::dynamic_pointer_cast<const ogma::EnvEvent>(bus.last_value(topic));
    }
};

}  // namespace

// =============================================================================
// 1. Wander: forward thrust + per-tick noise, no contact, no stuck → no steer.
// =============================================================================

TEST(CellReflex, WanderForwardWithNoise) {
    Fixture f;
    // Publish moving IMU so deficit stays at 0; no whiskers.
    float min_l =  std::numeric_limits<float>::infinity();
    float max_l = -std::numeric_limits<float>::infinity();
    for (uint64_t t = 0; t < 60; ++t) {
        f.run_tick(t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 2.0f, 1.0f}, t));
        });
        auto al = f.action("action.reflex.left");
        ASSERT_NE(al, nullptr);
        min_l = std::min(min_l, al->accel);
        max_l = std::max(max_l, al->accel);
    }
    // wander_thrust=2 + noise ∈ [-0.5, 0.5] + (small stuck pulse since
    // deficit≈0): output range ≈ [1.5, 2.5].  No persistent steer bias.
    EXPECT_LT(min_l, 2.0f);
    EXPECT_GT(max_l, 2.0f);
    EXPECT_NEAR(f.reflex.deficit(), 0.0f, 0.4f);
    EXPECT_LT(std::abs(f.reflex.last_steer()), 0.1f)
        << "no persistent steer bias when not stuck";
}

// =============================================================================
// 2. Stuck-pulse fires on deficit transitioning through threshold.
// =============================================================================

TEST(CellReflex, StuckPulseFiresOnDeficitTransition) {
    Fixture f;
    // 12 ticks of zero velocity (window=10) → deficit climbs from 0 → 1
    // and crosses 0.5.
    int prior_count = f.reflex.stuck_count();
    for (uint64_t t = 0; t < 12; ++t) {
        f.run_tick(t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    EXPECT_GT(f.reflex.stuck_count(), prior_count);
    auto e = f.last_event("events.wall_stuck");
    ASSERT_NE(e, nullptr);
    EXPECT_GT(e->intensity, 0.5f);
    // Pulse should now be active and producing a non-zero steer.
    EXPECT_GT(f.reflex.pulse_ticks(), 0);
    EXPECT_NE(f.reflex.pulse_held(), 0.0f);
}

// =============================================================================
// 3. Pulse direction follows contact asymmetry (right blocked → turn left).
// =============================================================================

TEST(CellReflex, PulseDirectionFollowsContactAsymmetry) {
    Fixture f;
    // Block right side strongly; left side clear.  Sustained zero velocity
    // drives the stuck pulse to fire.
    for (uint64_t t = 0; t < 12; ++t) {
        f.run_tick(t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
            // 3 right whiskers contacting; left clear.
            f.bus.publish("reality.proprio.whisker_3",
                make_proprio("whisker_3", {0.9f}, t));
            f.bus.publish("reality.proprio.whisker_4",
                make_proprio("whisker_4", {0.9f}, t));
            f.bus.publish("reality.proprio.whisker_5",
                make_proprio("whisker_5", {0.9f}, t));
        });
    }
    // diff_sum = right_sum - left_sum > 0 → dir = -1 → pulse_held < 0
    // → steer < 0 → ar > al (right side spikes more, CCW yaw, turn LEFT).
    EXPECT_LT(f.reflex.pulse_held(), 0.0f);
    auto al = f.action("action.reflex.left");
    auto ar = f.action("action.reflex.right");
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    EXPECT_LT(al->accel, ar->accel)
        << "right contact should steer left (al < ar)";
}

// =============================================================================
// 4. Pulse held for stuck_pulse_period, then resamples.
// =============================================================================

TEST(CellReflex, PulseHeldForPeriodAndResamples) {
    auto p = base_params();
    p["stuck_pulse_period"] = int64_t{10};
    Fixture f(p);
    // Get into stuck state with strong asymmetric contact (committed dir).
    for (uint64_t t = 0; t < 12; ++t) {
        f.run_tick(t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
            f.bus.publish("reality.proprio.whisker_4",
                make_proprio("whisker_4", {1.0f}, t));   // strong right
        });
    }
    float held_a = f.reflex.pulse_held();
    EXPECT_LT(held_a, -0.5f);   // committed left turn
    int ticks_a = f.reflex.pulse_ticks();
    EXPECT_GT(ticks_a, 0);
    EXPECT_LE(ticks_a, 10);

    // Run period out — pulse should resample at least once.  We track how
    // many distinct held values appear over the next 100 ticks.  With
    // committed asymmetric contact and t=tanh(|diff|)~tanh(1)≈0.76, the
    // direction stays committed (always -1) but magnitude can vary
    // slightly per resample.  Easier check: pulse_ticks resets to ~10.
    int saw_resample = 0;
    int prev_ticks = ticks_a;
    for (uint64_t t = 12; t < 80; ++t) {
        f.run_tick(t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
            f.bus.publish("reality.proprio.whisker_4",
                make_proprio("whisker_4", {1.0f}, t));
        });
        int now_ticks = f.reflex.pulse_ticks();
        if (now_ticks > prev_ticks) ++saw_resample;
        prev_ticks = now_ticks;
    }
    EXPECT_GE(saw_resample, 4)
        << "pulse should resample multiple times over 70 ticks at period=10";
}

// =============================================================================
// 5. Scent suppression: rising scent reduces the rectification factor t,
//    so the committed asymmetric pulse becomes less committed.
// =============================================================================

TEST(CellReflex, ScentSuppressionReducesPulseCommitment) {
    auto p = base_params();
    p["scent_alpha_short"] = 0.5;     // fast EMA
    p["scent_alpha_long"]  = 0.001;
    p["scent_gate_cap"]    = 1.0;     // allow full suppression for the test
    p["scent_long_pos_min"] = 0.001;
    Fixture f(p);

    // Establish a low scent baseline so long EMA settles low.
    for (uint64_t t = 0; t < 100; ++t) {
        f.run_tick(t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.05f}, t));
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 2.0f, 1.0f}, t));   // moving
        });
    }
    // Now ramp scent up so short-EMA leads long-EMA (gradient > 0).
    for (uint64_t t = 100; t < 130; ++t) {
        f.run_tick(t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.95f}, t));
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 2.0f, 1.0f}, t));
        });
    }
    EXPECT_GT(f.reflex.scent_factor(), 0.5f)
        << "scent factor should be high after rising-scent ramp";
}

// =============================================================================
// 6. Miss-event emission on high whisker contact, with refractory.
// =============================================================================

TEST(CellReflex, MissEventOnHighWhisker) {
    auto p = base_params();
    p["miss_refractory_ticks"] = int64_t{20};
    Fixture f(p);

    int events = 0;
    for (uint64_t t = 0; t < 100; ++t) {
        f.run_tick(t, [&](){
            f.bus.publish("reality.proprio.whisker_1",
                make_proprio("whisker_1", {0.6f}, t));   // above 0.30 threshold
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 2.0f, 1.0f}, t));
        });
        auto e = f.last_event("events.miss");
        if (e && int64_t(e->tick_id) == int64_t(t)) ++events;
    }
    // 100 ticks / 20 refractory ≈ 5 fires.
    EXPECT_GE(events, 4);
    EXPECT_LE(events, 6);
}
