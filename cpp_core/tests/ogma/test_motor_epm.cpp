// =============================================================================
// test_motor_epm.cpp
//   MotorEPM unit / regression suite (audit F7 + 2026-06-17 hardening plan).
//
//   The MotorEPM is the canonical embodied controller but had no dedicated test
//   coverage.  These tests lock down the claim-critical behavior so future motor
//   work cannot silently regress it:
//
//     1. ConstructionAndValidation — schema + topic-size contracts, null bus.
//     2. FiniteOutputNoNaN         — finite output past the babble warmup.
//     3. OutputBoundsAllReflexes   — every published accel ∈ [-1,1] with all
//                                    scaffolds maxed + extreme sensors.
//     4. AgencyInertWhenDriveZero  — coord_reward_drive=0 / amp_seek_rate=0 leave
//                                    the (1+1) search state untouched.
//     5. SnapshotRestoreDeterminism— two instances produce bit-identical output
//                                    after restore, INCLUDING agency-search,
//                                    height-homeostat, and panic state (proves
//                                    the WS1 snapshot-completeness fix).
//     6. NavBearingSignOdd         — the cell differential nav-steer is odd in the
//                                    target bearing (steers toward the target).
//     7. PanicHysteresis           — distress latch/unlatch with no in-band chatter.
//     8. HotParamApplication       — on_param_change round-trips via current_params.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>   // MotorEPM.hpp only forward-declares json; we read it here

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/MotorEPM.hpp"
#include "ogma/Topics.hpp"

namespace {

using ogma::ParamMap;
using ogma::ParamValue;

// ---- small helpers ----------------------------------------------------------

std::vector<std::string> proprio_topics(int n_legs, std::string const& pre) {
    std::vector<std::string> v;
    for (int i = 0; i < n_legs; ++i) v.push_back(pre + ".p" + std::to_string(i));
    return v;
}
std::vector<std::string> action_topics(int n_legs, int motor_dim, std::string const& pre) {
    std::vector<std::string> v;
    for (int leg = 0; leg < n_legs; ++leg)
        for (int j = 0; j < motor_dim; ++j)
            v.push_back(pre + ".a" + std::to_string(leg) + "." + std::to_string(j));
    return v;
}

// Picrawler-shaped base params (n_legs=4, motor_dim=3); every scaffold + the
// agency search OFF.  Tests opt in to the knobs they exercise.
ParamMap base_params(int n_legs = 4, int motor_dim = 3, std::string pre = "mt") {
    ParamMap p;
    p["n_legs"]         = int64_t{n_legs};
    p["motor_dim"]      = int64_t{motor_dim};
    p["seed"]           = int64_t{1234};
    p["babble_ticks"]   = int64_t{12};
    p["explore_noise"]  = 0.0;                          // deterministic post-warmup
    p["proprio_topics"] = proprio_topics(n_legs, pre);
    p["action_topics"]  = action_topics(n_legs, motor_dim, pre);
    p["imu_topic"]      = std::string(pre + ".imu");
    p["tilt_topic"]     = std::string(pre + ".tilt");
    p["distress_topic"] = std::string(pre + ".distress");
    p["height_topic"]   = std::string(pre + ".height");
    p["nav_topic"]      = std::string("");              // off unless a test sets it
    p["lateral_topic"]  = std::string("");
    p["cog_steer_topic"]= std::string("");
    p["boredom_topic"]  = std::string("");
    p["interest_topic"] = std::string("");
    p["hunger_topic"]   = std::string("");
    p["feet_topic"]     = std::string("");
    p["coupling_gain"]      = 0.0;
    p["coord_reward_drive"] = 0.0;
    p["coord_adapt_rate"]   = 0.0;
    p["amp_seek_rate"]      = 0.0;
    p["amp_homeo_gain"]     = 0.0;
    p["stroke_gain"]        = 0.0;
    p["balance_gain"]       = 0.0;
    p["heading_gain"]       = 0.0;
    p["nav_gain"]           = 0.0;
    p["height_homeo_gain"]  = 0.0;
    p["panic_strength"]     = 0.0;
    return p;
}

struct Sensors {
    float fwd_v = 0, yaw = 0, tc_x = 0, tc_y = 0;
    float distress = -1, height = -1;     // <0 → not published this tick
    float tilt_pitch = 0, tilt_roll = 0;
};

struct Fixture {
    ogma::InProcessBus bus;
    ogma::MotorEPM     m;
    int n_legs, motor_dim;
    std::string pre;

    Fixture(ParamMap const& p, int nl, int md, std::string prefix = "mt")
        : n_legs(nl), motor_dim(md), pre(std::move(prefix)) {
        m.set_id("motor_epm");
        m.on_setup(&bus, p);
    }

    static std::shared_ptr<ogma::ProprioToken> proprio_frame(double t, int leg, int md) {
        auto pt = std::make_shared<ogma::ProprioToken>();
        int n = 3 * md;                                  // [pos,act,delta] per joint
        pt->values = Eigen::VectorXf::Zero(n);
        double ph = 0.15 * t + leg * 1.3;
        for (int j = 0; j < md; ++j) {
            pt->values[3 * j + 0] = float(0.30 * std::sin(ph + j));        // pos
            pt->values[3 * j + 1] = float(0.20 * std::cos(ph + j));        // act
            pt->values[3 * j + 2] = float(0.30 * 0.15 * std::cos(ph + j)); // delta
        }
        pt->sensor = "proprio";
        return pt;
    }
    static std::shared_ptr<ogma::ProprioToken> vec_token(std::vector<float> const& vals) {
        auto pt = std::make_shared<ogma::ProprioToken>();
        pt->values = Eigen::VectorXf((int)vals.size());
        for (int i = 0; i < (int)vals.size(); ++i) pt->values[i] = vals[i];
        return pt;
    }

    void run_tick(uint64_t t, Sensors s = {}) {
        bus.begin_tick(t);
        for (int leg = 0; leg < n_legs; ++leg)
            bus.publish(pre + ".p" + std::to_string(leg), proprio_frame(double(t), leg, motor_dim));
        bus.publish(pre + ".imu",  vec_token({0, 0, s.fwd_v, s.yaw}));
        bus.publish(pre + ".tilt", vec_token({s.tilt_pitch, 1.0f, s.tilt_roll, 1.0f}));
        if (s.distress >= 0.0f) bus.publish(pre + ".distress", vec_token({s.distress}));
        if (s.height   >= 0.0f) bus.publish(pre + ".height",   vec_token({s.height}));
        if (s.tc_x != 0.0f || s.tc_y != 0.0f)
            bus.publish(pre + ".nav", vec_token({s.tc_x, s.tc_y}));
        m.tick(t);
        bus.end_tick();
    }

    float accel(int leg, int j) const {
        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(pre + ".a" + std::to_string(leg) + "." + std::to_string(j)));
        return a ? a->accel : std::numeric_limits<float>::quiet_NaN();
    }
};

} // namespace

// =============================================================================
// 1. Construction + schema + topic-size validation
// =============================================================================
TEST(MotorEPM, ConstructionAndValidation) {
    Fixture f(base_params(), 4, 3);
    EXPECT_EQ(f.m.type_name(), "MotorEPM");
    EXPECT_EQ(f.m.n_legs(), 4);
    EXPECT_EQ(f.m.legs_initialized(), 0);     // lazy: nothing yet

    // null bus
    {
        ogma::MotorEPM bad;
        EXPECT_THROW(bad.on_setup(nullptr, base_params()), std::invalid_argument);
    }
    // proprio_topics length must equal n_legs
    {
        auto p = base_params();
        p["proprio_topics"] = std::vector<std::string>{"only.one"};
        ogma::InProcessBus b; ogma::MotorEPM bad;
        EXPECT_THROW(bad.on_setup(&b, p), std::invalid_argument);
    }
    // action_topics length must equal n_legs*motor_dim
    {
        auto p = base_params();
        p["action_topics"] = std::vector<std::string>{"a", "b", "c"};
        ogma::InProcessBus b; ogma::MotorEPM bad;
        EXPECT_THROW(bad.on_setup(&b, p), std::invalid_argument);
    }
}

// =============================================================================
// 2. Finite, non-NaN output past the babble warmup
// =============================================================================
TEST(MotorEPM, FiniteOutputNoNaN) {
    auto p = base_params();
    p["coupling_gain"] = 0.5;
    Fixture f(p, 4, 3);
    for (uint64_t t = 0; t < 300; ++t) {
        f.run_tick(t, Sensors{/*fwd_v*/ float(0.1 * std::sin(0.13 * t))});
        if (t < 12) continue;                  // skip warmup
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j) {
                float a = f.accel(leg, j);
                ASSERT_TRUE(std::isfinite(a)) << "leg " << leg << " j " << j << " t " << t;
            }
    }
    EXPECT_EQ(f.m.legs_initialized(), 4);
    EXPECT_TRUE(std::isfinite(f.m.motor_tle_mean()));
}

// =============================================================================
// 3. Output is always clamped to [-1,1], even with every scaffold maxed
// =============================================================================
TEST(MotorEPM, OutputBoundsAllReflexesMaxed) {
    auto p = base_params();
    p["coupling_gain"]     = 2.0;
    p["stroke_gain"]       = 3.0;
    p["balance_gain"]      = 3.0;
    p["amp_homeo_gain"]    = 0.5;
    p["heading_gain"]      = 3.0;
    p["height_homeo_gain"] = 0.5;
    p["panic_strength"]    = 1.0;
    p["motor_gain"]        = 4.0;
    Fixture f(p, 4, 3);
    for (uint64_t t = 0; t < 400; ++t) {
        f.run_tick(t, Sensors{/*fwd_v*/ 1.0f, /*yaw*/ 1.0f, /*tc_x*/ 0, /*tc_y*/ 0,
                              /*distress*/ 0.9f, /*height*/ 0.8f,
                              /*tilt_pitch*/ 0.9f, /*tilt_roll*/ -0.9f});
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j) {
                float a = f.accel(leg, j);
                ASSERT_TRUE(std::isfinite(a)) << "t " << t;
                ASSERT_GE(a, -1.0f) << "t " << t << " leg " << leg << " j " << j;
                ASSERT_LE(a,  1.0f) << "t " << t << " leg " << leg << " j " << j;
            }
    }
}

// =============================================================================
// 4. coord_reward_drive=0 / amp_seek_rate=0 leave the (1+1) search inert
// =============================================================================
TEST(MotorEPM, AgencyInertWhenDriveZero) {
    auto p = base_params();
    p["coupling_gain"]      = 0.5;
    p["coord_reward_drive"] = 0.0;
    p["coord_adapt_rate"]   = 0.0;
    p["amp_seek_rate"]      = 0.0;
    Fixture f(p, 4, 3);

    auto gait0 = f.m.snapshot_state()["module"]["gait_phase"].get<std::vector<double>>();
    for (uint64_t t = 0; t < 600; ++t)
        f.run_tick(t, Sensors{/*fwd_v*/ float(0.3 * std::sin(0.1 * t))});

    auto mod = f.m.snapshot_state()["module"];
    EXPECT_EQ(mod["coord_probe_counter"].get<int64_t>(), 0);
    EXPECT_EQ(mod["amp_seek_counter"].get<int64_t>(), 0);
    EXPECT_FALSE(mod["coord_best_init"].get<bool>());
    EXPECT_EQ(mod["gait_phase"].get<std::vector<double>>(), gait0)
        << "agency search must not move the gait offsets when drive=0";
}

// =============================================================================
// 5. Snapshot/restore reproduces a running agent exactly — INCLUDING the
//    agency-search, height-homeostat, and panic state that WS1 added.
// =============================================================================
TEST(MotorEPM, SnapshotRestoreDeterminism) {
    auto p = base_params();
    p["coupling_gain"]      = 0.5;
    p["coord_reward_drive"] = 0.2;   // exercise the (1+1) phase search (uses coord_rng_)
    p["coord_probe_ticks"]  = int64_t{40};
    p["coord_stab_penalty"] = 0.3;
    p["height_homeo_gain"]  = 0.02;  // exercise chassis_h_max_ / height_bias_
    p["panic_strength"]     = 1.0;   // exercise panic_ / panic_phase_ / panic_latched_
    p["panic_noise"]        = 0.0;   // keep panic path RNG-free for determinism

    auto inputs = [](uint64_t t) {
        Sensors s;
        s.fwd_v    = 0.2f * std::sin(0.13 * t);
        s.distress = (t >= 40) ? 0.7f : 0.1f;          // engage panic partway
        s.height   = 0.40f + 0.05f * std::sin(0.07 * t);
        return s;
    };

    // f1 and f2 each own a separate InProcessBus, so they can share topic names.
    const uint64_t K1 = 100, K2 = 100;
    Fixture f1(p, 4, 3);
    for (uint64_t t = 0; t <= K1; ++t) f1.run_tick(t, inputs(t));

    auto snap = f1.m.snapshot_state();
    auto mod  = snap["module"];
    // Prove we actually exercised the previously-dropped state:
    EXPECT_EQ(snap["version"].get<int>(), 2);
    EXPECT_GT(mod["coord_probe_counter"].get<int64_t>(), 0) << "agency search must have run";
    EXPECT_GT(mod["panic"].get<float>(), 0.3f)              << "panic must be engaged";
    EXPECT_GT(mod["chassis_h_max"].get<float>(), 0.0f)      << "height ceiling must be seen";

    Fixture f2(p, 4, 3);
    f2.m.restore_state(snap);

    for (uint64_t t = K1 + 1; t <= K1 + K2; ++t) {
        f1.run_tick(t, inputs(t));
        f2.run_tick(t, inputs(t));
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j)
                ASSERT_FLOAT_EQ(f1.accel(leg, j), f2.accel(leg, j))
                    << "diverged at t " << t << " leg " << leg << " j " << j
                    << " — restored state is incomplete";
    }
}

// Bonus: a v1 snapshot (no "module" block) still loads (back-compat).
TEST(MotorEPM, RestoreLegacyV1SnapshotLoads) {
    Fixture f(base_params(), 4, 3);
    for (uint64_t t = 0; t < 30; ++t) f.run_tick(t, Sensors{0.1f});
    auto snap = f.m.snapshot_state();
    snap.erase("module");
    snap["version"] = 1;                          // emulate an old snapshot
    EXPECT_NO_THROW(f.m.restore_state(snap));
    // unknown version still rejected
    snap["version"] = 99;
    EXPECT_THROW(f.m.restore_state(snap), std::runtime_error);
}

// =============================================================================
// 6. Cell differential nav-steer is ODD in the target bearing (steers toward
//    the target).  Clean 2-output channel (n_legs=1, motor_dim=2), no phase.
// =============================================================================
TEST(MotorEPM, NavBearingSignOdd) {
    auto mk = [](float tc_x) {
        auto p = base_params(1, 2, "cell");
        p["nav_gain"]     = 0.4;
        p["nav_topic"]    = std::string("cell.nav");
        p["babble_ticks"] = int64_t{10};
        Fixture f(p, 1, 2, "cell");
        Sensors s; s.tc_x = tc_x; s.tc_y = 0.0f;      // target straight to the side
        for (uint64_t t = 0; t < 40; ++t) f.run_tick(t, s);
        return f.accel(0, 0) - f.accel(0, 1);          // differential (turn command)
    };
    float diff_plus  = mk(+0.8f);   // target to the right
    float diff_minus = mk(-0.8f);   // target to the left

    // The HK common-mode cancels (identical inputs); only the odd nav term differs.
    EXPECT_GT(std::fabs(diff_plus - diff_minus), 0.1f) << "nav must consume the bearing";
    EXPECT_GT(diff_plus, diff_minus)                   << "differential must reverse with bearing sign";
}

// =============================================================================
// 7. Panic enters above panic_on, holds in-band (no chatter), exits below panic_off
// =============================================================================
TEST(MotorEPM, PanicHysteresis) {
    auto p = base_params();
    p["panic_strength"] = 1.0;
    p["panic_on"]       = 0.5;
    p["panic_off"]      = 0.25;
    Fixture f(p, 4, 3);
    uint64_t t = 0;

    for (int i = 0; i < 30; ++i, ++t) f.run_tick(t, Sensors{0, 0, 0, 0, /*distress*/ 0.1f});
    { auto md = f.m.snapshot_state()["module"];
      EXPECT_FALSE(md["panic_latched"].get<bool>());
      EXPECT_LT(md["panic"].get<float>(), 0.1f); }

    for (int i = 0; i < 90; ++i, ++t) f.run_tick(t, Sensors{0, 0, 0, 0, /*distress*/ 0.7f});
    { auto md = f.m.snapshot_state()["module"];
      EXPECT_TRUE(md["panic_latched"].get<bool>());
      EXPECT_GT(md["panic"].get<float>(), 0.9f); }

    // In the hysteresis band (0.25 < d < 0.5): must STAY latched (no chatter).
    for (int i = 0; i < 40; ++i, ++t) f.run_tick(t, Sensors{0, 0, 0, 0, /*distress*/ 0.35f});
    { auto md = f.m.snapshot_state()["module"];
      EXPECT_TRUE(md["panic_latched"].get<bool>());
      EXPECT_GT(md["panic"].get<float>(), 0.5f); }

    for (int i = 0; i < 120; ++i, ++t) f.run_tick(t, Sensors{0, 0, 0, 0, /*distress*/ 0.1f});
    { auto md = f.m.snapshot_state()["module"];
      EXPECT_FALSE(md["panic_latched"].get<bool>());
      EXPECT_LT(md["panic"].get<float>(), 0.1f); }
}

// =============================================================================
// 8. Hot-mutable params apply via on_param_change and round-trip current_params
// =============================================================================
TEST(MotorEPM, HotParamApplication) {
    Fixture f(base_params(), 4, 3);
    struct KV { const char* k; double v; };
    for (auto const& kv : {KV{"coord_reward_drive", 0.5}, KV{"coord_stab_penalty", 0.15},
                           KV{"nav_gain", -3.0}, KV{"panic_strength", 0.7},
                           KV{"height_k", 0.65}}) {
        f.m.on_param_change(kv.k, ParamValue{kv.v});
        auto cp = f.m.current_params();
        auto it = cp.find(kv.k);
        ASSERT_NE(it, cp.end()) << kv.k << " missing from current_params";
        EXPECT_DOUBLE_EQ(std::get<double>(it->second), kv.v) << kv.k;
    }
}
