// =============================================================================
// test_reflex_modules_streams.cpp
//   Behavioral stream tests for the Phase 6.6.D reflex-as-module primitives:
//   WhiskerAversionReflex, DualEMADetector, AdaptiveThresholdTracker, and
//   StuckEscapeReflex.  Each fixture drives synthetic proprio streams and
//   asserts the event/metric emission contracts that body_controller.gd
//   used to provide directly.
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>

#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/AdaptiveThresholdTracker.hpp"
#include "ogma/modules/DualEMADetector.hpp"
#include "ogma/modules/StuckEscapeReflex.hpp"
#include "ogma/modules/ForwardDriveReflex.hpp"
#include "ogma/modules/ScentGateReflex.hpp"
#include "ogma/modules/WhiskerAversionReflex.hpp"
#include "ogma/modules/WhiskerSteerReflex.hpp"

namespace {

// -----------------------------------------------------------------------------
// Helpers shared across fixtures
// -----------------------------------------------------------------------------

std::shared_ptr<ogma::ProprioToken> make_proprio(
        std::string const& sensor, std::vector<float> const& vals,
        uint64_t tick_id) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id = tick_id;
    p->sensor  = sensor;
    p->values.resize(int(vals.size()));
    for (int i = 0; i < int(vals.size()); ++i) p->values[i] = vals[i];
    return p;
}

template <typename Module, typename F>
void run_tick(ogma::InProcessBus& bus, Module& m, uint64_t t, F&& staged) {
    bus.begin_tick(t);
    staged();
    m.tick(t);
    bus.end_tick();
}

int count_events(ogma::InProcessBus& bus, std::string const& topic,
                  std::string const& name_filter = "") {
    auto p = bus.last_value(topic);
    if (!p) return 0;
    auto e = std::dynamic_pointer_cast<const ogma::EnvEvent>(p);
    if (!e) return 0;
    if (!name_filter.empty() && e->name != name_filter) return 0;
    return 1;  // last_value collapses to most recent — fixture uses fire counts.
}

// =============================================================================
// WhiskerAversionReflex
// =============================================================================

struct WhiskerFixture {
    ogma::InProcessBus           bus;
    ogma::WhiskerAversionReflex  reflex;

    WhiskerFixture(ogma::ParamMap p = {}) {
        if (p.empty()) {
            p["whisker_topic_prefix"] = std::string("reality.proprio.whisker_");
            p["threshold"]            = 0.30;
            p["wall_stuck_threshold"] = 0.55;
            p["refractory_ticks"]     = int64_t{30};
        }
        reflex.set_id("whisker_reflex");
        reflex.on_setup(&bus, p);
    }
};

TEST(WhiskerAversionReflex, BootstrapNoEventsBeforeInput) {
    WhiskerFixture f;
    run_tick(f.bus, f.reflex, 0, [](){});
    EXPECT_EQ(f.reflex.miss_count(), 0);
    EXPECT_EQ(f.reflex.wall_stuck_count(), 0);
}

TEST(WhiskerAversionReflex, FiresMissOnceAboveThresholdThenSuppressed) {
    WhiskerFixture f;
    // Single contact at 0.5 — above 0.30 threshold.  First tick should fire.
    for (uint64_t t = 0; t < 5; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.whisker_2",
                make_proprio("whisker_2", {0.5f}, t));
        });
    }
    EXPECT_EQ(f.reflex.miss_count(), 1)
        << "Refractory should suppress further misses for 30 ticks";
}

TEST(WhiskerAversionReflex, FiresAgainAfterRefractoryExpires) {
    ogma::ParamMap p;
    p["refractory_ticks"] = int64_t{3};
    p["threshold"]        = 0.30;
    p["wall_stuck_threshold"] = 0.95;
    p["whisker_topic_prefix"] = std::string("reality.proprio.whisker_");
    WhiskerFixture f(p);
    for (uint64_t t = 0; t < 8; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.whisker_0",
                make_proprio("whisker_0", {0.5f}, t));
        });
    }
    // Expected fires at t=0 and t=4 (refractory=3 means 3 ticks suppression
    // after fire; fire-at-0 → suppress 1,2,3 → fire-eligible at 4).
    EXPECT_EQ(f.reflex.miss_count(), 2);
}

TEST(WhiskerAversionReflex, WallStuckFiresIndependentOfMissRefractory) {
    WhiskerFixture f;
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_3",
            make_proprio("whisker_3", {0.7f}, 0));
    });
    EXPECT_GE(f.reflex.wall_stuck_count(), 1);
    EXPECT_GE(f.reflex.miss_count(), 1);
}

TEST(WhiskerAversionReflex, MaxOverMultipleWhiskersDrivesEvents) {
    WhiskerFixture f;
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_0",
            make_proprio("whisker_0", {0.10f}, 0));
        f.bus.publish("reality.proprio.whisker_5",
            make_proprio("whisker_5", {0.45f}, 0));
    });
    EXPECT_NEAR(f.reflex.last_max_w(), 0.45f, 1e-5);
    EXPECT_EQ(f.reflex.miss_count(), 1);
}

TEST(WhiskerAversionReflex, HotMutateThresholdTakesEffect) {
    WhiskerFixture f;
    f.reflex.on_param_change("threshold", ogma::ParamValue{0.60});
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_0",
            make_proprio("whisker_0", {0.50f}, 0));
    });
    EXPECT_EQ(f.reflex.miss_count(), 0)
        << "0.50 < new threshold 0.60 — must NOT fire after hot-mutate";
}

// =============================================================================
// DualEMADetector
// =============================================================================

struct EMAFixture {
    ogma::InProcessBus      bus;
    ogma::DualEMADetector   det;

    EMAFixture(ogma::ParamMap p = {}) {
        if (p.empty()) {
            p["input_topic"]        = std::string("reality.proprio.scent_max");
            p["input_index"]        = int64_t{0};
            p["output_event_name"]  = std::string("hit");
            p["alpha_short"]        = 0.5;     // fast-converging for tests
            p["alpha_long"]         = 0.05;
            p["ratio_threshold"]    = 1.5;
            p["refractory_ticks"]   = int64_t{0};
            p["require_long_pos"]   = true;
        }
        det.set_id("dual_ema");
        det.on_setup(&bus, p);
    }
};

TEST(DualEMADetector, BootstrapNoFireOnFirstSample) {
    EMAFixture f;
    run_tick(f.bus, f.det, 0, [&](){
        f.bus.publish("reality.proprio.scent_max",
            make_proprio("scent_max", {0.2f}, 0));
    });
    EXPECT_EQ(f.det.fire_count(), 0)
        << "First sample initialises both EMAs; no ratio yet.";
}

TEST(DualEMADetector, FiresWhenShortRisesAboveLongByRatio) {
    EMAFixture f;
    // Settle at low value first, then sharp rise.
    for (uint64_t t = 0; t < 10; ++t) {
        run_tick(f.bus, f.det, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.10f}, t));
        });
    }
    int before = f.det.fire_count();
    for (uint64_t t = 10; t < 20; ++t) {
        run_tick(f.bus, f.det, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {1.0f}, t));
        });
    }
    EXPECT_GT(f.det.fire_count(), before);
}

TEST(DualEMADetector, NoFireOnFlatSignal) {
    EMAFixture f;
    for (uint64_t t = 0; t < 50; ++t) {
        run_tick(f.bus, f.det, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.5f}, t));
        });
    }
    EXPECT_EQ(f.det.fire_count(), 0);
}

TEST(DualEMADetector, MotionFloorGatingSuppressesFire) {
    ogma::ParamMap p;
    p["input_topic"]        = std::string("reality.proprio.scent_max");
    p["output_event_name"]  = std::string("hit");
    p["alpha_short"]        = 0.5;
    p["alpha_long"]         = 0.05;
    p["ratio_threshold"]    = 1.2;
    p["motion_floor_topic"] = std::string("reality.proprio.imu");
    p["motion_floor_index"] = int64_t{2};
    p["motion_floor_min"]   = 0.5;
    EMAFixture f(p);
    for (uint64_t t = 0; t < 10; ++t) {
        run_tick(f.bus, f.det, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.10f}, t));
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    int before = f.det.fire_count();
    for (uint64_t t = 10; t < 20; ++t) {
        run_tick(f.bus, f.det, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {1.0f}, t));
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));  // vx=0
        });
    }
    EXPECT_EQ(f.det.fire_count(), before)
        << "Trend-rise must NOT fire while motion floor (vx=0) blocks";
}

// =============================================================================
// AdaptiveThresholdTracker
// =============================================================================

struct AdaptiveFixture {
    ogma::InProcessBus              bus;
    ogma::AdaptiveThresholdTracker  tr;

    AdaptiveFixture(ogma::ParamMap p = {}) {
        if (p.empty()) {
            p["input_topic"]   = std::string("reality.proprio.scent_max");
            p["input_index"]   = int64_t{0};
            p["output_topic"]  = std::string("metrics.adaptive_threshold.scent_max");
            p["alpha"]         = 0.1;
            p["n_stddev"]      = 2.0;
            p["warmup_ticks"]  = int64_t{20};
            p["min_stddev"]    = 1e-6;
        }
        tr.set_id("adaptive_tracker");
        tr.on_setup(&bus, p);
    }
};

TEST(AdaptiveThresholdTracker, ConvergesToInputMean) {
    AdaptiveFixture f;
    for (uint64_t t = 0; t < 200; ++t) {
        run_tick(f.bus, f.tr, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.5f}, t));
        });
    }
    EXPECT_NEAR(f.tr.mean(), 0.5f, 0.05f);
    EXPECT_LT(f.tr.stddev(), 0.05f);
}

TEST(AdaptiveThresholdTracker, ThresholdAboveMeanByNStddev) {
    AdaptiveFixture f;
    // Alternate between 0.4 and 0.6 — mean ~ 0.5, stddev ~ 0.1
    for (uint64_t t = 0; t < 400; ++t) {
        run_tick(f.bus, f.tr, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {(t % 2 == 0) ? 0.4f : 0.6f}, t));
        });
    }
    EXPECT_NEAR(f.tr.mean(), 0.5f, 0.05f);
    EXPECT_GT(f.tr.threshold(), f.tr.mean());
    EXPECT_GT(f.tr.threshold(), 0.55f);
}

TEST(AdaptiveThresholdTracker, WarmFlagFlipsAfterWarmupTicks) {
    AdaptiveFixture f;
    EXPECT_FALSE(f.tr.warm());
    for (uint64_t t = 0; t < 25; ++t) {
        run_tick(f.bus, f.tr, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.5f}, t));
        });
    }
    EXPECT_TRUE(f.tr.warm());
}

TEST(AdaptiveThresholdTracker, PublishesEachTick) {
    AdaptiveFixture f;
    for (uint64_t t = 0; t < 5; ++t) {
        run_tick(f.bus, f.tr, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.5f}, t));
        });
    }
    auto t = std::dynamic_pointer_cast<const ogma::AdaptiveThreshold>(
        f.bus.last_value("metrics.adaptive_threshold.scent_max"));
    ASSERT_NE(t, nullptr);
    EXPECT_GE(t->threshold, t->mean);
}

// =============================================================================
// StuckEscapeReflex
// =============================================================================

struct StuckFixture {
    ogma::InProcessBus       bus;
    ogma::StuckEscapeReflex  reflex;

    StuckFixture(ogma::ParamMap p = {}) {
        if (p.empty()) {
            p["imu_topic"]            = std::string("reality.proprio.imu");
            p["vx_index"]             = int64_t{2};
            p["vz_index"]             = int64_t{3};
            p["move_speed_reference"] = 3.0;
            p["severity_threshold"]   = 0.5;
            p["window_ticks"]         = int64_t{10};   // shorter for tests
            p["refractory_ticks"]     = int64_t{20};
        }
        reflex.set_id("stuck_reflex");
        reflex.on_setup(&bus, p);
    }
};

TEST(StuckEscapeReflex, WaitsForFullWindowBeforeFiring) {
    StuckFixture f;
    // Pin velocity to zero from tick 0; window=10 so fire shouldn't happen
    // until at least 10 samples are in.
    int prior = f.reflex.stuck_count();
    for (uint64_t t = 0; t < 5; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    EXPECT_EQ(f.reflex.stuck_count(), prior)
        << "Window not yet full — should not fire";
}

TEST(StuckEscapeReflex, FiresWhenSpeedStaysLow) {
    StuckFixture f;
    for (uint64_t t = 0; t < 12; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    EXPECT_GE(f.reflex.stuck_count(), 1);
    EXPECT_GT(f.reflex.severity(), 0.9f);
}

TEST(StuckEscapeReflex, NoFireWhenMovingFast) {
    StuckFixture f;
    for (uint64_t t = 0; t < 30; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            // |v| = sqrt(2.0^2 + 1.0^2) ≈ 2.24 — severity ≈ 1 - 2.24/3 ≈ 0.25
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 2.0f, 1.0f}, t));
        });
    }
    EXPECT_EQ(f.reflex.stuck_count(), 0);
}

TEST(StuckEscapeReflex, RefractorySuppressesRepeatedFires) {
    StuckFixture f;
    for (uint64_t t = 0; t < 60; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    // window=10 ticks, refractory=20 ticks → first fire at t=9, next eligible t=30,
    // next eligible t=51 — expect exactly 3 fires in 60 ticks.
    EXPECT_GE(f.reflex.stuck_count(), 2);
    EXPECT_LE(f.reflex.stuck_count(), 4);
}

// =============================================================================
// Phase 6.6.G — bilateral rotation pulse on stuck.
// =============================================================================

TEST(StuckEscapeReflex, PulseRotatesBilaterallyOnStuck) {
    ogma::ParamMap p;
    p["imu_topic"]            = std::string("reality.proprio.imu");
    p["vx_index"]             = int64_t{2};
    p["vz_index"]             = int64_t{3};
    p["move_speed_reference"] = 3.0;
    p["severity_threshold"]   = 0.5;
    p["window_ticks"]         = int64_t{10};
    p["refractory_ticks"]     = int64_t{200};   // long, prevents re-fire mid-pulse
    p["enable_pulse"]         = true;
    p["output_topic_left"]    = std::string("action.reflex.left");
    p["output_topic_right"]   = std::string("action.reflex.right");
    p["pulse_ticks"]          = int64_t{15};
    p["pulse_rotation"]       = 4.0;
    p["master_seed"]          = int64_t{42};
    StuckFixture f(p);

    // Pre-stuck (window not yet full): no pulse, no publish.
    for (uint64_t t = 0; t < 5; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    EXPECT_EQ(f.reflex.pulse_remaining(), 0);
    EXPECT_EQ(std::dynamic_pointer_cast<const ogma::ActionOut>(
        f.bus.last_value("action.reflex.left")), nullptr);

    // Run until first stuck fire (window=10, fires at t=9).
    for (uint64_t t = 5; t < 10; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    ASSERT_GE(f.reflex.stuck_count(), 1);
    int held_dir = f.reflex.pulse_dir();
    EXPECT_NE(held_dir, 0);
    EXPECT_GT(f.reflex.pulse_remaining(), 0);

    // Bilateral publishes on both sides with correct sign + magnitude.
    auto al = std::dynamic_pointer_cast<const ogma::ActionOut>(
        f.bus.last_value("action.reflex.left"));
    auto ar = std::dynamic_pointer_cast<const ogma::ActionOut>(
        f.bus.last_value("action.reflex.right"));
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    EXPECT_NEAR(al->accel, float(held_dir) *  4.0f, 1e-5f);
    EXPECT_NEAR(ar->accel, float(held_dir) * -4.0f, 1e-5f);
    EXPECT_EQ(al->source, "stuck_escape");

    // Direction must hold across the pulse (no flip mid-rotation).
    for (uint64_t t = 10; t < 20; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
        if (f.reflex.pulse_remaining() > 0) {
            EXPECT_EQ(f.reflex.pulse_dir(), held_dir)
                << "pulse direction must not flip mid-event (tick " << t << ")";
        }
    }
    // After 15 pulse ticks (started at t=9, exhausted by t=24) the
    // pulse should be done.  refractory_ticks=200 prevents re-fire.
    for (uint64_t t = 20; t < 30; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    EXPECT_EQ(f.reflex.pulse_remaining(), 0);
    EXPECT_EQ(f.reflex.pulse_dir(), 0);
}

TEST(StuckEscapeReflex, PulseDisabledByDefault) {
    StuckFixture f;   // default params: enable_pulse not set → false
    for (uint64_t t = 0; t < 30; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.imu",
                make_proprio("imu", {0.0f, 1.0f, 0.0f, 0.0f}, t));
        });
    }
    EXPECT_GT(f.reflex.stuck_count(), 0);   // detection still works
    EXPECT_EQ(f.reflex.pulse_remaining(), 0);
    // Default action.left/right topics should not have been published
    // since enable_pulse is false.
    EXPECT_EQ(std::dynamic_pointer_cast<const ogma::ActionOut>(
        f.bus.last_value(ogma::topics::kActionLeft)), nullptr);
}

// =============================================================================
// ForwardDriveReflex (Phase 6.6.D.8) — constant-thrust pump for honest wiring
// =============================================================================

TEST(ForwardDriveReflex, PublishesEqualThrustOnBothChannelsEachTick) {
    ogma::InProcessBus bus;
    ogma::ForwardDriveReflex drive;
    ogma::ParamMap p;
    p["thrust"] = 2.5;
    drive.set_id("fwd");
    drive.on_setup(&bus, p);
    bus.begin_tick(0);
    drive.tick(0);
    bus.end_tick();

    auto al = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionLeft));
    auto ar = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionRight));
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    EXPECT_FLOAT_EQ(al->accel, 2.5f);
    EXPECT_FLOAT_EQ(ar->accel, 2.5f);
    EXPECT_EQ(al->source, "forward_drive");
}

TEST(ForwardDriveReflex, HotMutateThrustTakesEffect) {
    ogma::InProcessBus bus;
    ogma::ForwardDriveReflex drive;
    ogma::ParamMap p;
    p["thrust"] = 1.0;
    drive.set_id("fwd");
    drive.on_setup(&bus, p);
    drive.on_param_change("thrust", ogma::ParamValue{3.5});
    bus.begin_tick(0);
    drive.tick(0);
    bus.end_tick();
    auto al = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionLeft));
    ASSERT_NE(al, nullptr);
    EXPECT_FLOAT_EQ(al->accel, 3.5f);
}

// =============================================================================
// ScentGateReflex (Phase 6.6.D.8) — scalar suppression gate
// =============================================================================

struct GateFixture {
    ogma::InProcessBus     bus;
    ogma::ScentGateReflex  gate;

    GateFixture(ogma::ParamMap p = {}) {
        if (p.empty()) {
            p["input_topic"]  = std::string("reality.proprio.scent_max");
            p["output_topic"] = std::string("reflex.gate.scent_aversion");
            p["alpha_short"]  = 0.5;     // fast for tests
            p["alpha_long"]   = 0.05;
            p["cap"]          = 0.5;
            p["enabled"]      = true;
            p["long_pos_min"] = 0.001;
        }
        gate.set_id("scent_gate");
        gate.on_setup(&bus, p);
    }

    std::shared_ptr<const ogma::ReflexGate> last() const {
        return std::dynamic_pointer_cast<const ogma::ReflexGate>(
            bus.last_value("reflex.gate.scent_aversion"));
    }
};

TEST(ScentGateReflex, StartsInactiveBeforeWarmup) {
    GateFixture f;
    run_tick(f.bus, f.gate, 0, [](){});
    auto g = f.last();
    ASSERT_NE(g, nullptr);
    EXPECT_FALSE(g->active);
    EXPECT_FLOAT_EQ(g->value, 0.0f);
}

TEST(ScentGateReflex, ActivatesOnRisingScent) {
    GateFixture f;
    // Warm both EMAs to a low baseline.
    for (uint64_t t = 0; t < 10; ++t) {
        run_tick(f.bus, f.gate, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.1f}, t));
        });
    }
    // Sharp rise — short EMA climbs above long.
    for (uint64_t t = 10; t < 20; ++t) {
        run_tick(f.bus, f.gate, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {1.0f}, t));
        });
    }
    auto g = f.last();
    ASSERT_NE(g, nullptr);
    EXPECT_TRUE(g->active);
    EXPECT_GT(g->value, 0.0f);
    EXPECT_LE(g->value, 0.5f);  // honors cap
}

TEST(ScentGateReflex, FlatSignalNoSuppression) {
    GateFixture f;
    for (uint64_t t = 0; t < 50; ++t) {
        run_tick(f.bus, f.gate, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {0.5f}, t));
        });
    }
    auto g = f.last();
    ASSERT_NE(g, nullptr);
    EXPECT_NEAR(g->value, 0.0f, 0.05f)
        << "Flat signal should produce ~zero (short ≈ long)";
}

TEST(ScentGateReflex, EnabledFalseForcesZero) {
    ogma::ParamMap p;
    p["input_topic"]  = std::string("reality.proprio.scent_max");
    p["output_topic"] = std::string("reflex.gate.scent_aversion");
    p["alpha_short"]  = 0.5;
    p["alpha_long"]   = 0.05;
    p["cap"]          = 0.5;
    p["enabled"]      = false;
    p["long_pos_min"] = 0.001;
    GateFixture f(p);
    for (uint64_t t = 0; t < 20; ++t) {
        run_tick(f.bus, f.gate, t, [&](){
            f.bus.publish("reality.proprio.scent_max",
                make_proprio("scent_max", {(t < 10) ? 0.1f : 1.0f}, t));
        });
    }
    auto g = f.last();
    ASSERT_NE(g, nullptr);
    EXPECT_FLOAT_EQ(g->value, 0.0f);
    EXPECT_FALSE(g->active);
}

// Integration: ScentGateReflex producing → WhiskerAversionReflex consuming.
// The reflex's events.miss intensity should be scaled by (1 - gate.value).
TEST(ScentGateReflex, SuppressesDownstreamWhiskerAversion) {
    ogma::InProcessBus bus;
    ogma::ScentGateReflex gate;
    ogma::WhiskerAversionReflex aversion;

    ogma::ParamMap gp;
    gp["input_topic"]  = std::string("reality.proprio.scent_max");
    gp["output_topic"] = std::string("reflex.gate.scent_aversion");
    gp["alpha_short"]  = 0.5;
    gp["alpha_long"]   = 0.05;
    gp["cap"]          = 0.5;
    gp["enabled"]      = true;
    gp["long_pos_min"] = 0.001;

    ogma::ParamMap ap;
    ap["whisker_topic_prefix"] = std::string("reality.proprio.whisker_");
    ap["threshold"]            = 0.30;
    ap["wall_stuck_threshold"] = 0.95;
    ap["refractory_ticks"]     = int64_t{0};
    ap["suppression_topic"]    = std::string("reflex.gate.scent_aversion");

    gate.set_id("gate");
    aversion.set_id("aversion");
    gate.on_setup(&bus, gp);
    aversion.on_setup(&bus, ap);

    auto run_one = [&](uint64_t t, float scent, float whisker) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent_max",
            make_proprio("scent_max", {scent}, t));
        bus.publish("reality.proprio.whisker_2",
            make_proprio("whisker_2", {whisker}, t));
        gate.tick(t);
        aversion.tick(t);
        bus.end_tick();
    };

    // Warm gate at low scent, no contact (no miss yet).
    for (uint64_t t = 0; t < 12; ++t) run_one(t, 0.1f, 0.0f);
    int baseline_misses = aversion.miss_count();

    // Whisker contact at 0.6 with FLAT scent (gate idle ≈ 0) — full strength.
    run_one(12, 0.1f, 0.6f);
    auto e_full = std::dynamic_pointer_cast<const ogma::EnvEvent>(
        bus.last_value("events.miss"));
    ASSERT_NE(e_full, nullptr);
    EXPECT_GT(aversion.miss_count(), baseline_misses);
    float full_intensity = e_full->intensity;

    // Now drive scent up to activate the gate, then re-fire whisker.  Gate
    // value ≈ cap (0.5), so suppression should ~halve the miss intensity.
    for (uint64_t t = 13; t < 25; ++t) run_one(t, 1.0f, 0.0f);
    run_one(25, 1.0f, 0.6f);
    auto e_supp = std::dynamic_pointer_cast<const ogma::EnvEvent>(
        bus.last_value("events.miss"));
    ASSERT_NE(e_supp, nullptr);
    EXPECT_LT(e_supp->intensity, full_intensity * 0.95f)
        << "Active gate should reduce miss intensity";
    EXPECT_GT(e_supp->intensity, full_intensity * 0.45f)
        << "Suppression must not exceed cap=0.5";
}

// =============================================================================
// WhiskerSteerReflex (Phase 6.6.D.7) — bilateral steering output
// =============================================================================

struct SteerFixture {
    ogma::InProcessBus       bus;
    ogma::WhiskerSteerReflex reflex;

    SteerFixture(ogma::ParamMap p = {}) {
        if (p.empty()) {
            p["whisker_topic_prefix"] = std::string("reality.proprio.whisker_");
            p["left_suffixes"]        = std::vector<std::string>{"0", "1", "2"};
            p["right_suffixes"]       = std::vector<std::string>{"3", "4", "5"};
            p["threshold"]            = 0.30;
            p["base_thrust"]          = 0.0;
            p["steer_gain"]           = 8.0;
            p["accel_min"]            = -4.0;
            p["accel_max"]            = 4.0;
            p["refractory_ticks"]     = int64_t{0};
        }
        reflex.set_id("whisker_steer");
        reflex.on_setup(&bus, p);
    }

    std::shared_ptr<const ogma::ActionOut> action(std::string const& topic) const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(bus.last_value(topic));
    }
};

TEST(WhiskerSteerReflex, BootstrapNoOutputBeforeContact) {
    SteerFixture f;
    run_tick(f.bus, f.reflex, 0, [](){});
    EXPECT_EQ(f.reflex.fire_count(), 0);
    EXPECT_EQ(f.action(ogma::topics::kActionLeft),  nullptr);
    EXPECT_EQ(f.action(ogma::topics::kActionRight), nullptr);
}

TEST(WhiskerSteerReflex, LeftContactProducesRightTurn) {
    SteerFixture f;
    // whisker_1 (left side) at 0.6, no right contact.
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_1",
            make_proprio("whisker_1", {0.6f}, 0));
    });
    auto al = f.action(ogma::topics::kActionLeft);
    auto ar = f.action(ogma::topics::kActionRight);
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    // (al - ar) > 0 ⇒ right turn (matches body convention).
    EXPECT_GT(al->accel, ar->accel)
        << "Left contact must drive accel.left > accel.right (right turn)";
    EXPECT_EQ(al->source, "whisker_steer");
}

TEST(WhiskerSteerReflex, RightContactProducesLeftTurn) {
    SteerFixture f;
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_4",
            make_proprio("whisker_4", {0.6f}, 0));
    });
    auto al = f.action(ogma::topics::kActionLeft);
    auto ar = f.action(ogma::topics::kActionRight);
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    EXPECT_LT(al->accel, ar->accel)
        << "Right contact must drive accel.left < accel.right (left turn)";
}

TEST(WhiskerSteerReflex, FrontalContactNoSteerPreference) {
    SteerFixture f;
    // Symmetric contact (whisker_2 left + whisker_3 right at same intensity)
    // is a frontal collision — should produce zero differential.
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_2",
            make_proprio("whisker_2", {0.5f}, 0));
        f.bus.publish("reality.proprio.whisker_3",
            make_proprio("whisker_3", {0.5f}, 0));
    });
    auto al = f.action(ogma::topics::kActionLeft);
    auto ar = f.action(ogma::topics::kActionRight);
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    EXPECT_NEAR(al->accel, ar->accel, 1e-5)
        << "Symmetric contact must produce balanced thrust (no steering bias)";
}

TEST(WhiskerSteerReflex, BelowThresholdNoFire) {
    SteerFixture f;
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_1",
            make_proprio("whisker_1", {0.20f}, 0));   // below 0.30 threshold
    });
    EXPECT_EQ(f.reflex.fire_count(), 0);
    EXPECT_EQ(f.action(ogma::topics::kActionLeft),  nullptr);
    EXPECT_EQ(f.action(ogma::topics::kActionRight), nullptr);
}

TEST(WhiskerSteerReflex, FullAsymmetrySaturatesAccel) {
    SteerFixture f;
    // NEW contract (held-pulse kick reflex): a single-side kick is the
    // IPSILATERAL contact side driven by steer_gain × (weighted contact sum),
    // CLAMPED to [accel_min, accel_max].  The opposite side stays at 0 (no
    // depth/half-magnitude modulation).  whisker_2 is the inner left whisker
    // (position weight 1.0); at full contact left_sum = 1.0 → al = 8×1.0 = 8,
    // which SATURATES to accel_max = 4.  Right side has no contact → ar = 0.
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_2",
            make_proprio("whisker_2", {1.0f}, 0));
    });
    auto al = f.action(ogma::topics::kActionLeft);
    auto ar = f.action(ogma::topics::kActionRight);
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    EXPECT_FALSE(f.reflex.last_head_on())
        << "single-side contact is an ipsilateral turn-away, not a wedge reverse";
    EXPECT_FLOAT_EQ(al->accel, 4.0f)
        << "strong ipsilateral contact saturates to accel_max";
    EXPECT_FLOAT_EQ(ar->accel, 0.0f)
        << "non-contacting side gets no drive (sum × gain, then clamp)";
    EXPECT_GT(al->accel, ar->accel)
        << "left contact still steers right (al > ar)";
}

TEST(WhiskerSteerReflex, RefractoryHonored) {
    ogma::ParamMap p;
    p["whisker_topic_prefix"] = std::string("reality.proprio.whisker_");
    p["left_suffixes"]        = std::vector<std::string>{"0", "1", "2"};
    p["right_suffixes"]       = std::vector<std::string>{"3", "4", "5"};
    p["threshold"]            = 0.30;
    p["steer_gain"]           = 8.0;
    p["accel_min"]            = -4.0;
    p["accel_max"]            = 4.0;
    p["refractory_ticks"]     = int64_t{3};
    SteerFixture f(p);
    for (uint64_t t = 0; t < 8; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.whisker_1",
                make_proprio("whisker_1", {0.6f}, t));
        });
    }
    // Same if/elif refractory shape as the aversion test:
    // fire t=0, skip 1/2/3, fire t=4, skip 5/6/7 → 2 fires in 8 ticks.
    EXPECT_EQ(f.reflex.fire_count(), 2);
}

// =============================================================================
// Phase 6.6.G — symmetric-contact suppression and head-on rotation.
// =============================================================================

TEST(WhiskerSteerReflex, SymmetricContactWedgeReverses) {
    // NEW contract: symmetric (both-side) contact NO LONGER "skips" to let
    // forward drive survive.  Both sides above threshold = boxed in / wedged →
    // the reflex flips sign and BACKS OUT: al = min(-gain·left_sum, -reverse),
    // ar = min(-gain·right_sum, -reverse), both clamped.  This is the emergency
    // reverse, marked head_on.  (The old min_steer "skip symmetric" path is gone.)
    ogma::ParamMap p;
    p["whisker_topic_prefix"] = std::string("reality.proprio.whisker_");
    p["left_suffixes"]        = std::vector<std::string>{"0", "1", "2"};
    p["right_suffixes"]       = std::vector<std::string>{"3", "4", "5"};
    p["threshold"]            = 0.30;
    p["steer_gain"]           = 8.0;
    p["accel_min"]            = -4.0;
    p["accel_max"]            = 4.0;
    p["refractory_ticks"]     = int64_t{0};
    SteerFixture f(p);

    // Both sides equal at 0.5 (both > 0.30) → wedge reverse, not skip.
    run_tick(f.bus, f.reflex, 0, [&](){
        f.bus.publish("reality.proprio.whisker_1",
            make_proprio("whisker_1", {0.5f}, 0));
        f.bus.publish("reality.proprio.whisker_4",
            make_proprio("whisker_4", {0.5f}, 0));
    });
    EXPECT_EQ(f.reflex.fire_count(), 1);
    EXPECT_FALSE(f.reflex.last_skipped());
    EXPECT_TRUE(f.reflex.last_head_on())
        << "both-side contact is a wedge/head-on reverse";
    auto al = f.action(ogma::topics::kActionLeft);
    auto ar = f.action(ogma::topics::kActionRight);
    ASSERT_NE(al, nullptr);
    ASSERT_NE(ar, nullptr);
    // BACK OUT: both sides negative (common-mode reverse), at least reverse_strength.
    EXPECT_LT(al->accel, 0.0f) << "wedge reverse drives left side backward";
    EXPECT_LT(ar->accel, 0.0f) << "wedge reverse drives right side backward";
    EXPECT_LE(al->accel, -4.0f + 1e-5f)
        << "reverse guaranteed at least reverse_strength magnitude (clamped to accel_min)";
    EXPECT_LE(ar->accel, -4.0f + 1e-5f);

    // Single-side contact past threshold (right at 0.3 is NOT > threshold) →
    // ipsilateral turn-away, NOT a reverse.
    SteerFixture g(p);
    run_tick(g.bus, g.reflex, 0, [&](){
        g.bus.publish("reality.proprio.whisker_1",
            make_proprio("whisker_1", {0.7f}, 0));
        g.bus.publish("reality.proprio.whisker_4",
            make_proprio("whisker_4", {0.3f}, 0));   // == threshold, not above
    });
    EXPECT_EQ(g.reflex.fire_count(), 1);
    EXPECT_FALSE(g.reflex.last_head_on())
        << "only left side above threshold → single-side turn-away, not wedge";
    auto al2 = g.action(ogma::topics::kActionLeft);
    auto ar2 = g.action(ogma::topics::kActionRight);
    ASSERT_NE(al2, nullptr);
    ASSERT_NE(ar2, nullptr);
    EXPECT_GT(al2->accel, ar2->accel)
        << "left contact steers right (al > ar)";
    EXPECT_FLOAT_EQ(ar2->accel, 0.0f)
        << "right side below threshold contributes nothing";
}

TEST(WhiskerSteerReflex, WedgeReverseHeldAcrossPulse) {
    // NEW contract: a both-side (head-on / wedge) event is a HELD PULSE.
    // On fresh contact the reflex LATCHES the back-out kick and re-issues the
    // SAME al/ar for pulse_ticks ticks (a 1-tick fire barely imparts impulse),
    // THEN drops into a refractory where it goes SILENT so the cognitive layer
    // resurfaces.  fire_count counts PULSES, not ticks.  The old "asymmetric
    // ±rotation with a randomly-held direction" path no longer exists — both
    // sides back out together (common-mode reverse, last_head_dir stays 0).
    ogma::ParamMap p;
    p["whisker_topic_prefix"] = std::string("reality.proprio.whisker_");
    p["left_suffixes"]        = std::vector<std::string>{"0", "1", "2"};
    p["right_suffixes"]       = std::vector<std::string>{"3", "4", "5"};
    p["threshold"]            = 0.30;
    p["steer_gain"]           = 8.0;
    p["accel_min"]            = -4.0;
    p["accel_max"]            = 4.0;
    p["pulse_ticks"]          = int64_t{3};   // HOLD the kick for 3 ticks
    p["refractory_ticks"]     = int64_t{2};   // then go silent for 2 ticks
    SteerFixture f(p);

    // Sustained both-side contact for the full cycle (pulse + refractory).
    float latched_al = 0.0f, latched_ar = 0.0f;
    for (uint64_t t = 0; t < 5; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.whisker_1",
                make_proprio("whisker_1", {1.0f}, t));
            f.bus.publish("reality.proprio.whisker_4",
                make_proprio("whisker_4", {1.0f}, t));
        });
        if (t < 3) {
            // Pulse phase: fires once on t=0, then HOLDS the latched kick t=1,2.
            EXPECT_EQ(f.reflex.fire_count(), 1)
                << "held pulse counts ONE fire, re-issued each tick (tick " << t << ")";
            EXPECT_FALSE(f.reflex.last_skipped());
            EXPECT_TRUE(f.reflex.last_head_on());
            EXPECT_EQ(f.reflex.last_head_dir(), 0)
                << "wedge reverse is common-mode; no ±direction committed";
            auto al = f.action(ogma::topics::kActionLeft);
            auto ar = f.action(ogma::topics::kActionRight);
            ASSERT_NE(al, nullptr);
            ASSERT_NE(ar, nullptr);
            // Both sides back out (negative, saturated to accel_min).
            EXPECT_FLOAT_EQ(al->accel, -4.0f);
            EXPECT_FLOAT_EQ(ar->accel, -4.0f);
            if (t == 0) { latched_al = al->accel; latched_ar = ar->accel; }
            // The held kick is the SAME commands each tick of the pulse.
            EXPECT_FLOAT_EQ(al->accel, latched_al)
                << "pulse re-issues the latched command (tick " << t << ")";
            EXPECT_FLOAT_EQ(ar->accel, latched_ar);
        } else {
            // Refractory phase: SILENT (publishes nothing) so the cog resurfaces.
            EXPECT_TRUE(f.reflex.last_skipped())
                << "refractory tick must be silent (tick " << t << ")";
            EXPECT_EQ(f.reflex.fire_count(), 1)
                << "refractory does not re-fire (tick " << t << ")";
        }
    }
}

TEST(WhiskerSteerReflex, ReflexReleasesOnContactDrop) {
    // NEW contract (INVERTED from the old "hold direction" expectation): the
    // reflex does NOT latch a held direction across an event.  After the kick
    // is held for pulse_ticks it enters the refractory and goes SILENT; if
    // contact has dropped by then it simply RELEASES — last_head_dir stays 0
    // and it does not re-fire — handing control back to the cognitive layer.
    ogma::ParamMap p;
    p["whisker_topic_prefix"] = std::string("reality.proprio.whisker_");
    p["left_suffixes"]        = std::vector<std::string>{"0", "1", "2"};
    p["right_suffixes"]       = std::vector<std::string>{"3", "4", "5"};
    p["threshold"]            = 0.30;
    p["steer_gain"]           = 8.0;
    p["accel_min"]            = -4.0;
    p["accel_max"]            = 4.0;
    p["pulse_ticks"]          = int64_t{2};
    p["refractory_ticks"]     = int64_t{2};
    SteerFixture f(p);

    // Fresh both-side (wedge) kick at t=0, held through t=1 (pulse_ticks=2).
    for (uint64_t t = 0; t < 2; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.whisker_1",
                make_proprio("whisker_1", {0.9f}, t));
            f.bus.publish("reality.proprio.whisker_4",
                make_proprio("whisker_4", {0.9f}, t));
        });
    }
    EXPECT_EQ(f.reflex.fire_count(), 1);
    EXPECT_TRUE(f.reflex.last_head_on());
    EXPECT_EQ(f.reflex.last_head_dir(), 0)
        << "reflex never commits a ±held direction (common-mode reverse)";

    // Contact drops to zero while the module is in its refractory window.
    // The reflex must RELEASE: stay silent, not re-fire, head_dir cleared.
    for (uint64_t t = 2; t < 4; ++t) {
        run_tick(f.bus, f.reflex, t, [&](){
            f.bus.publish("reality.proprio.whisker_1",
                make_proprio("whisker_1", {0.0f}, t));
            f.bus.publish("reality.proprio.whisker_4",
                make_proprio("whisker_4", {0.0f}, t));
        });
        EXPECT_TRUE(f.reflex.last_skipped())
            << "released reflex is silent during refractory (tick " << t << ")";
    }
    EXPECT_EQ(f.reflex.fire_count(), 1)
        << "no re-fire after contact dropped — the reflex released";
    EXPECT_EQ(f.reflex.last_head_dir(), 0)
        << "head-on direction reset to 0 on contact drop (release, not hold)";

    // With contact gone and the refractory drained, the reflex stays silent.
    run_tick(f.bus, f.reflex, 4, [&](){
        f.bus.publish("reality.proprio.whisker_1",
            make_proprio("whisker_1", {0.0f}, 4));
        f.bus.publish("reality.proprio.whisker_4",
            make_proprio("whisker_4", {0.0f}, 4));
    });
    EXPECT_TRUE(f.reflex.last_skipped());
    EXPECT_EQ(f.reflex.fire_count(), 1);
}

} // namespace
