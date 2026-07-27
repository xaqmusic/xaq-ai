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
//     9. Gate0ResetMaskingInstruments — gait_coherence + reset counters surfaced in
//                                    diag, driven by events.miss/reset, and
//                                    round-tripped through snapshot (L-1a Gate 0).
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
    p["torque_topic"]   = std::string("");
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


// Same as base_params() but with torque_topic explicitly unwired — the reference arm for
// the gain-0 guard, so "gate off" is compared against "the signal was never subscribed".
ParamMap base_params_no_torque(int n_legs = 4, int motor_dim = 3, std::string pre = "mt") {
    ParamMap p = base_params(n_legs, motor_dim, std::move(pre));
    p["stroke_gain"]      = 1.0;
    p["stroke_phase"]     = -2.85;
    p["torque_topic"]     = std::string("");
    p["stroke_load_gain"] = 0.0;
    return p;
}

struct Sensors {
    float fwd_v = 0, yaw = 0, tc_x = 0, tc_y = 0;
    float distress = -1, height = -1;     // <0 → not published this tick
    float tilt_pitch = 0, tilt_roll = 0;
    // Keep `feet` LAST: existing tests brace-initialize Sensors positionally, so a
    // new leading member would silently land in the wrong field.
    std::vector<float> feet;              // per-leg foot height; empty → not published
    // ... and `torque` after it, for the same reason.  Joint-major, exactly as the body
    // publishes it: [hip1 x n_legs, hip2 x n_legs, knee x n_legs].
    std::vector<float> torque;            // empty → not published
    // ... and `contact` after THAT, same reason again.  Per-leg physics touch flag
    // (1 = foot down), as `reality.proprio.foot_contact` publishes it.
    std::vector<float> contact;           // empty → not published
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

    // `event` (e.g. "reset"/"miss") publishes an EnvEvent on events.<event> this
    // tick — used to exercise the Gate 0 reset-masking hook.  Empty = no event.
    // `obj` (non-null) publishes a per-leg PredictionToken posture objective on
    // <pre>.obj<leg> with confidence `obj_conf` — exercises the L-1b objective socket.
    void run_tick(uint64_t t, Sensors s = {}, std::string const& event = "",
                  Eigen::VectorXf const* obj = nullptr, float obj_conf = 1.0f) {
        bus.begin_tick(t);
        for (int leg = 0; leg < n_legs; ++leg)
            bus.publish(pre + ".p" + std::to_string(leg), proprio_frame(double(t), leg, motor_dim));
        bus.publish(pre + ".imu",  vec_token({0, 0, s.fwd_v, s.yaw}));
        bus.publish(pre + ".tilt", vec_token({s.tilt_pitch, 1.0f, s.tilt_roll, 1.0f}));
        if (!s.feet.empty()) bus.publish(pre + ".feet", vec_token(s.feet));
        if (!s.torque.empty()) bus.publish(pre + ".torque", vec_token(s.torque));
        if (!s.contact.empty()) bus.publish(pre + ".contact", vec_token(s.contact));
        if (s.distress >= 0.0f) bus.publish(pre + ".distress", vec_token({s.distress}));
        if (s.height   >= 0.0f) bus.publish(pre + ".height",   vec_token({s.height}));
        if (s.tc_x != 0.0f || s.tc_y != 0.0f)
            bus.publish(pre + ".nav", vec_token({s.tc_x, s.tc_y}));
        if (obj) {
            for (int leg = 0; leg < n_legs; ++leg) {
                auto pt = std::make_shared<ogma::PredictionToken>();
                pt->predicted_latent = *obj;
                pt->confidence       = obj_conf;
                pt->target_modality  = "posture." + std::to_string(leg);
                bus.publish(pre + ".obj" + std::to_string(leg), pt);
            }
        }
        if (!event.empty()) {
            auto ev = std::make_shared<ogma::EnvEvent>();
            ev->name = event; ev->intensity = 1.0f;
            bus.publish("events." + event, ev);
        }
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

// =============================================================================
// 8b. Per-leg controller symmetry coupling: OFF (gain=0) is byte-identical (the
//     default-off contract), ON (gain>0) actually changes the controllers (not a
//     silent no-op — the hip2 lesson).  Refuted for the picrawler gait but kept as
//     default-off infra; this locks the inert-when-off guarantee.
// =============================================================================
TEST(MotorEPM, ControllerSymmetryCouplingOffInertOnActive) {
    auto base = base_params();
    base["coupling_gain"] = 0.5;                              // a real gait so controllers diverge to couple
    std::vector<double> grp{0.0, 1.0, 0.0, 1.0};             // sign-safe left/right groups
    auto pz  = base; pz["ctrl_symmetry_gain"]  = 0.0;  pz["symmetry_group_of"]  = grp;
    auto pon = base; pon["ctrl_symmetry_gain"] = 0.05; pon["symmetry_group_of"] = grp;
    Fixture f0(base, 4, 3), fz(pz, 4, 3), fon(pon, 4, 3);    // independent buses, identical inputs
    double maxdiff_z = 0.0, maxdiff_on = 0.0;
    for (uint64_t t = 0; t < 250; ++t) {
        f0.run_tick(t); fz.run_tick(t); fon.run_tick(t);
        if (t < 12) continue;
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j) {
                maxdiff_z  = std::max(maxdiff_z,  double(std::abs(f0.accel(leg, j) - fz.accel(leg, j))));
                maxdiff_on = std::max(maxdiff_on, double(std::abs(f0.accel(leg, j) - fon.accel(leg, j))));
            }
    }
    EXPECT_LT(maxdiff_z, 1e-6)  << "ctrl_symmetry_gain=0 must be byte-identical (default-off contract)";
    EXPECT_GT(maxdiff_on, 1e-3) << "ctrl_symmetry_gain>0 must actually change the controllers (no silent no-op)";
}

// =============================================================================
// 8b. Swing-detector deadband (`swing_hyst_frac`, 2026-07-25).
//
// The legacy detector was `foot_y > foot_y_ema` with no deadband.  A signal sits
// above its own moving average about half the time, so that test SPLITS ~50/50 by
// construction — it reports gait phase, not ground contact — and it chatters on any
// small ripple.  Measured on the live robot: 40.3 % of legs called "swinging" while
// the body's own absolute contact test said the feet were down 99.3 %.
//
// WHAT IS AND IS NOT ASSERTED HERE — measured 2026-07-25, and narrower than the first
// draft of this change claimed.  Two unit experiments were run and both are recorded
// because together they bound the mechanism:
//
//  (a) Given a REAL duty cycle (80 ticks planted + a 20-tick lift arc), the legacy
//      no-deadband detector is essentially CORRECT: it reported 0.18 swing against a
//      true duty of 0.20.  The planted phase sits decisively below the EMA (which the
//      lift excursions pull upward), so there is nothing to chatter on.
//  (b) In that same regime a band of 1.0·MAD is too WIDE: the stance deviation
//      (≈0.006) never clears −band (≈0.010), so the detector LATCHES in whatever state
//      it entered — 0.58 swing.  This is the failure mode behind the live `frac=2.0`
//      arm degrading (steps 92, tilt_sd 0.139, 0.25 falls) after `frac=0.5` helped.
//
// So the defect is CONDITIONAL, and the condition is the absence of stepping.  On the
// live robot the feet are down 99.3 % of the time with only micro-excursions — there
// is no duty cycle — and there the legacy detector chattered to a measured 40.3 %
// "swinging", which a 0.5·MAD band cut back substantially.  The detector is therefore
// an AMPLIFIER of the no-stepping problem, not an independent root cause; see the
// ledger's open frontier.
//
// Also note what a MAD-scaled band structurally cannot do.  It is scale-invariant by
// design, so there is no tuned constant (CLAUDE.md §5.5) — but it therefore cannot
// separate jitter from a step by AMPLITUDE: a pure sinusoid has peak/MAD ≈ 1.57 at any
// size, so no frac below that suppresses it and any frac above it suppresses all of
// it.  Genuinely answering "is this foot loaded" needs a contact/load observation, and
// there is none on the bus — a height signal cannot settle it without a reference that
// is either god's-eye or self-referential.
//
// Only the two contract properties are asserted below: the gain-0 guard, and that a
// nonzero band really moves the gate.  The regime-dependent behavioral claim is left
// to the seed-averaged corridor A/Bs, which is where it was actually measured.
// =============================================================================

// The gain-0 guard: swing_hyst_frac=0 must leave the actions byte-identical, and a
// nonzero band must actually change them (no silent no-op in either direction).
TEST(MotorEPM, SwingDeadbandZeroIsByteIdenticalNonzeroActs) {
    auto base = base_params();
    base["feet_topic"]       = std::string("mt.feet");
    base["stance_lift_gain"] = 0.5;          // the consumer whose gate the band moves
    base["coupling_gain"]    = 0.5;
    auto pz  = base; pz["swing_hyst_frac"]  = 0.0;
    auto pon = base; pon["swing_hyst_frac"] = 1.0;
    Fixture f0(base, 4, 3), fz(pz, 4, 3), fon(pon, 4, 3);
    // A real stepping motion, so the detector has genuine transitions to gate.
    auto stepping_feet = [](uint64_t t) {
        std::vector<float> feet(4);
        for (int i = 0; i < 4; ++i)
            feet[i] = -0.01f + 0.05f * float(std::sin(0.09 * double(t) + i * 1.57));
        return feet;
    };
    double maxdiff_z = 0.0, maxdiff_on = 0.0;
    for (uint64_t t = 0; t < 300; ++t) {
        Sensors s; s.feet = stepping_feet(t);
        f0.run_tick(t, s); fz.run_tick(t, s); fon.run_tick(t, s);
        if (t < 12) continue;
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j) {
                maxdiff_z  = std::max(maxdiff_z,  double(std::abs(f0.accel(leg, j) - fz.accel(leg, j))));
                maxdiff_on = std::max(maxdiff_on, double(std::abs(f0.accel(leg, j) - fon.accel(leg, j))));
            }
    }
    EXPECT_LT(maxdiff_z, 1e-6)  << "swing_hyst_frac=0 must be byte-identical to the "
                                   "legacy detector (default-off contract)";
    EXPECT_GT(maxdiff_on, 1e-3) << "swing_hyst_frac>0 must actually move the stance gate";
}

// =============================================================================
// 8c. Reward-free coordination fitness (`coord_fitness_mode=1`, 2026-07-25).
//
// The (1+1) phase search ranks probes and KEEPS the winner.  Mode 0 ranks by forward
// velocity — a task reward (§5.1), and the thing that lets a destructive escape thrash
// become a permanent incumbent.  Mode 1 ranks by coherence·activity/(1+tle).
//
// The property that matters is ANTI-DEGENERACY: a frozen body must not win.
//
// The coherence trap is real but SUBTLER than first written, and the correction is worth
// keeping.  `gait_coherence` is R = |mean_j e^{i(φ_j − P_j)}| — it subtracts the offsets.
// On a frozen body every φ_j = atan2(0,0) = 0, so R = |mean_j e^{−iP_j}|, which depends
// entirely on WHICH OFFSETS ARE BEING PROBED: with the default trot [0,π,π,0] it is 0,
// but with all-equal offsets it is 1.0.  And the (1+1) search MUTATES those offsets — so
// it can walk itself into the self-consistent optimum "all-equal offsets + frozen body",
// scoring a perfect 1.0 on a corpse.  This test therefore pins the trap at the offsets
// where it bites, and shows the activity factor defusing it there.
// =============================================================================
TEST(MotorEPM, RewardFreeCoordFitnessRejectsAFrozenBody) {
    auto p = base_params();
    p["coupling_gain"] = 0.5;
    // All-equal offsets: the corner of the search space where coherence is maximal on a
    // still body.  This is reachable BY THE SEARCH, so it must not be a winning probe.
    p["gait_phase"] = std::vector<double>{0.0, 0.0, 0.0, 0.0};
    Fixture f(p, 4, 3);
    // A frozen body: publish an unchanging proprio frame so every leg's phase vector is
    // (0,0) and the oscillation amplitude decays to nothing.
    for (uint64_t t = 0; t < 400; ++t) {
        f.bus.begin_tick(t);
        for (int leg = 0; leg < f.n_legs; ++leg) {
            auto pt = std::make_shared<ogma::ProprioToken>();
            pt->values = Eigen::VectorXf::Zero(3 * f.motor_dim);   // dead still
            pt->sensor = "proprio";
            f.bus.publish(f.pre + ".p" + std::to_string(leg), pt);
        }
        f.bus.publish(f.pre + ".imu",  Fixture::vec_token({0, 0, 0, 0}));
        f.bus.publish(f.pre + ".tilt", Fixture::vec_token({0, 1.0f, 0, 1.0f}));
        f.m.tick(t);
        f.bus.end_tick();
    }
    auto d = f.m.diag_snapshot();
    const double coherence = d.value("gait_coherence", -1.0);
    const double activity  = d.value("coord_activity", -1.0);
    // The trap, demonstrated rather than asserted away: at these offsets coherence is
    // MAXIMAL on a corpse, so ranking probes on coherence alone would reward freezing.
    EXPECT_GT(coherence, 0.9) << "at all-equal offsets a frozen body should score maximal "
                                 "Kuramoto coherence (the trap); got " << coherence;
    // The guard: the activity factor collapses, so coherence·activity/(1+tle) → ~0 and
    // no frozen probe can out-score a moving one.
    EXPECT_LT(activity, 0.02) << "activity must collapse on a frozen body — it is the ONLY "
                                 "factor preventing a still-body optimum; got " << activity;
}

// Gain-0 guard: coord_fitness_mode=0 must reproduce the legacy fwd_v-ranked search
// exactly, so every existing config is untouched and mode 1 is a clean single lever.
TEST(MotorEPM, CoordFitnessModeZeroIsByteIdentical) {
    auto base = base_params();
    base["coupling_gain"]      = 0.5;
    base["coord_reward_drive"] = 0.3;      // the search must actually be running
    base["coord_probe_ticks"]  = int64_t{60};
    auto pz = base; pz["coord_fitness_mode"] = int64_t{0};
    Fixture f0(base, 4, 3), fz(pz, 4, 3);
    double maxdiff = 0.0;
    for (uint64_t t = 0; t < 400; ++t) {
        Sensors s; s.fwd_v = 0.05f * float(std::sin(0.02 * double(t)));
        f0.run_tick(t, s); fz.run_tick(t, s);
        if (t < 12) continue;
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j)
                maxdiff = std::max(maxdiff, double(std::abs(f0.accel(leg, j) - fz.accel(leg, j))));
    }
    EXPECT_LT(maxdiff, 1e-6) << "coord_fitness_mode=0 must be byte-identical to the legacy "
                                "fwd_v-ranked search (default-off contract)";
}

// =============================================================================
// 9. Gate 0 (L-1a): the reset-masked gait instruments are surfaced in
//    diag_snapshot, driven by events.miss / events.reset, and round-trip through
//    snapshot/restore.  This is the measurement foundation for the L-1 gait work.
// =============================================================================
TEST(MotorEPM, Gate0ResetMaskingInstruments) {
    auto p = base_params();
    p["coupling_gain"] = 0.5;
    Fixture f(p, 4, 3);

    // Warm up with NO disruption.
    for (uint64_t t = 0; t < 60; ++t) f.run_tick(t, Sensors{0.1f});

    auto d = f.m.diag_snapshot();
    ASSERT_TRUE(d.contains("gait_coherence"));
    ASSERT_TRUE(d.contains("reset_count"));
    ASSERT_TRUE(d.contains("ticks_since_reset"));
    ASSERT_TRUE(d.contains("reset_rate"));
    // gait_coherence is a Kuramoto order parameter ∈ [0,1].
    const float coh = d["gait_coherence"].get<float>();
    EXPECT_GE(coh, 0.0f);
    EXPECT_LE(coh, 1.0f);
    // No disruption yet → count 0, ticks-since advanced, rate ~0.
    EXPECT_EQ(d["reset_count"].get<uint64_t>(), 0u);
    EXPECT_GT(d["ticks_since_reset"].get<uint64_t>(), 0u);
    EXPECT_NEAR(d["reset_rate"].get<float>(), 0.0f, 1e-3f);

    // events.reset zeroes the masking counter, bumps the count, moves the rate.
    f.run_tick(60, Sensors{0.1f}, /*event=*/"reset");
    d = f.m.diag_snapshot();
    EXPECT_EQ(d["reset_count"].get<uint64_t>(), 1u);
    EXPECT_EQ(d["ticks_since_reset"].get<uint64_t>(), 0u) << "reset must restart the mask counter";
    EXPECT_GT(d["reset_rate"].get<float>(), 0.0f)          << "the disruption must move the rate EMA";

    // ticks_since_reset counts up again on a quiet tick.
    f.run_tick(61, Sensors{0.1f});
    EXPECT_EQ(f.m.diag_snapshot()["ticks_since_reset"].get<uint64_t>(), 1u);

    // events.miss is treated identically (a fall is also a disruption).
    f.run_tick(62, Sensors{0.1f}, /*event=*/"miss");
    d = f.m.diag_snapshot();
    EXPECT_EQ(d["reset_count"].get<uint64_t>(), 2u);
    EXPECT_EQ(d["ticks_since_reset"].get<uint64_t>(), 0u);

    // The counters + rate round-trip through snapshot/restore.
    auto snap = f.m.snapshot_state();
    EXPECT_EQ(snap["module"]["reset_count"].get<uint64_t>(), 2u);
    Fixture f2(p, 4, 3);
    f2.m.restore_state(snap);
    auto d2 = f2.m.diag_snapshot();
    EXPECT_EQ(d2["reset_count"].get<uint64_t>(), 2u);
    EXPECT_EQ(d2["ticks_since_reset"].get<uint64_t>(), 0u);
    EXPECT_FLOAT_EQ(d2["reset_rate"].get<float>(), d["reset_rate"].get<float>());
}

// =============================================================================
// 10. L-1b objective socket: an active posture objective retargets the controller;
//     a zero-weight objective is a perfect no-op (Gate-0 baseline preserved).
// =============================================================================
TEST(MotorEPM, ObjectiveSocketRetargetsControllerZeroWeightNoOp) {
    auto p = base_params();
    p["coupling_gain"]    = 0.5;
    p["objective_topics"] = std::vector<std::string>{"mt.obj0", "mt.obj1", "mt.obj2", "mt.obj3"};
    auto pn = base_params();               // identical minus the objective socket
    pn["coupling_gain"]   = 0.5;

    Fixture A(p,  4, 3);                    // objective active (w = 0.5)
    Fixture Z(p,  4, 3);                    // objective published at w = 0 (must be a no-op)
    Fixture N(pn, 4, 3);                    // no objective_topics at all

    Eigen::VectorXf tgt(3);                 // motor_dim target joint positions (the socket contract)
    tgt << 0.2f, -0.1f, 0.3f;
    auto inp = [](uint64_t t){ return Sensors{float(0.2 * std::sin(0.11 * t))}; };

    for (uint64_t t = 0; t <= 150; ++t) {
        A.run_tick(t, inp(t), "", &tgt, 0.5f);
        Z.run_tick(t, inp(t), "", &tgt, 0.0f);
        N.run_tick(t, inp(t));
    }

    // (i) a zero-weight objective changes nothing → Z == N byte-for-byte.
    for (int leg = 0; leg < 4; ++leg)
        for (int j = 0; j < 3; ++j)
            EXPECT_FLOAT_EQ(Z.accel(leg, j), N.accel(leg, j))
                << "zero-weight objective must be a perfect no-op (leg " << leg << " j " << j << ")";

    // (ii) an active objective retargets the controller → A diverges from Z.
    bool diverged = false;
    for (int leg = 0; leg < 4 && !diverged; ++leg)
        for (int j = 0; j < 3; ++j)
            if (std::fabs(A.accel(leg, j) - Z.accel(leg, j)) > 1e-4f) { diverged = true; break; }
    EXPECT_TRUE(diverged) << "an active posture objective must change the controller output";

    // (iii) diag reflects the socket state.
    EXPECT_TRUE(A.m.diag_snapshot()["obj_active"].get<bool>());
    EXPECT_FALSE(Z.m.diag_snapshot()["obj_active"].get<bool>());
    EXPECT_NEAR(A.m.diag_snapshot()["obj_weight"].get<float>(), 0.5f, 1e-4f);
}

// =============================================================================
// 10. LOAD-GATED POWER STROKE (`stroke_load_gain`, 2026-07-26).
//
// Phase 0 measured that the stroke is UNGATED with respect to ground contact: the
// fraction of stance spent in the stroke's positive half is 0.512 and over swing 0.513,
// so push direction is statistically independent of purchase and half the power stroke
// is spent in the air.  This lever gates propulsion by each leg's share of hip1 load —
// hip1 because Phase 0 measured it to be the joint that separates stance from swing
// (ratio 1.368, vs 1.124 hip2 and 1.011 knee).
//
// Two properties are locked down here, and they are the two that a future edit could
// silently break:
//   (a) gain 0 is BYTE-IDENTICAL, even with torque_topic wired and load wildly uneven;
//   (b) with the gain on, an UNLOADED leg's propulsion is attenuated relative to a
//       LOADED one — i.e. the gate actually discriminates, in the right direction.
// =============================================================================
TEST(MotorEPM, StrokeLoadGateZeroIsByteIdenticalNonzeroDiscriminates) {
    auto base = base_params();
    base["stroke_gain"]  = 1.0;                 // the term being gated
    base["stroke_phase"] = -2.85;
    base["torque_topic"] = std::string("mt.torque");
    auto p0 = base; p0["stroke_load_gain"] = 0.0;
    auto pg = base; pg["stroke_load_gain"] = 4.0;
    Fixture fref(base_params_no_torque(), 4, 3);   // no torque wired at all
    Fixture f0(p0, 4, 3), fg(pg, 4, 3);

    // Leg 0 carries no hip1 load (foot in the air); legs 1-3 are loaded.  Layout is
    // joint-major, so hip1 occupies indices [0..3].
    std::vector<float> tq(12, 0.10f);
    tq[0] = 0.00f; tq[1] = 0.40f; tq[2] = 0.40f; tq[3] = 0.40f;

    double maxdiff0 = 0.0;
    double gated_leg0 = 0.0, gated_leg1 = 0.0, ref_leg0 = 0.0, ref_leg1 = 0.0;
    for (uint64_t t = 0; t < 200; ++t) {
        Sensors s; s.torque = tq;
        Sensors sref;                              // reference gets no torque published
        fref.run_tick(t, sref); f0.run_tick(t, s); fg.run_tick(t, s);
        if (t < 20) continue;
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j)
                maxdiff0 = std::max(maxdiff0,
                                    double(std::abs(fref.accel(leg, j) - f0.accel(leg, j))));
        // hip1 (joint 0) is where the stroke lands.  Compare magnitudes accumulated over
        // the run so a single zero-crossing tick cannot decide the test.
        ref_leg0   += std::abs(f0.accel(0, 0));
        ref_leg1   += std::abs(f0.accel(1, 0));
        gated_leg0 += std::abs(fg.accel(0, 0));
        gated_leg1 += std::abs(fg.accel(1, 0));
    }
    EXPECT_LT(maxdiff0, 1e-6)
        << "stroke_load_gain=0 must be byte-identical to an unwired torque_topic "
           "(the gain-0 guard), even with a wildly uneven load vector on the bus";
    // The unloaded leg must lose stroke authority RELATIVE to a loaded one.  Stated as a
    // ratio-of-ratios so it cannot be satisfied by simply scaling everything down.
    const double ref_ratio   = ref_leg0   / (ref_leg1   + 1e-9);
    const double gated_ratio = gated_leg0 / (gated_leg1 + 1e-9);
    EXPECT_LT(gated_ratio, ref_ratio * 0.9)
        << "with the gate on, the UNLOADED leg's hip1 stroke must shrink relative to a "
           "LOADED leg's (ungated ratio " << ref_ratio << ", gated " << gated_ratio << ")";
}

// =============================================================================
// 11. STROKE-TO-STEP LOCK (`stroke_phase_src`, 2026-07-27).
//
// Phase 0 measured three per-leg clocks that nothing forced to agree: the stroke rides a
// 22-24 tick knee-derived phase while the leg steps every 26-30, and the fraction of
// STANCE spent in the stroke's positive half is 0.512 against 0.513 over SWING — push
// direction statistically INDEPENDENT of whether the foot is down.  This lever gives the
// stroke a touchdown-referenced step clock instead.
//
// The properties locked down here are the ones a future edit could silently break, and
// each corresponds to a specific failure this project has already paid for:
//   (a) src 0 is BYTE-IDENTICAL even with contact wired and the clock running;
//   (b) phi = 0 AT touchdown and ramps toward 2*pi across the step (the mechanism);
//   (c) an un-debounced contact bounce does NOT reset the phase (the chatter that makes
//       the incumbent foot-height detector unusable);
//   (d) before two touchdowns, and for a leg that stops stepping, the stroke FALLS BACK
//       to L.phase rather than freezing — a frozen phase would silently turn the stroke
//       into a DC bias, which is the refuted "blind knee bias kills the gait" shape;
//   (e) the period rails clamp an anomalous stride out of the EMA.
// =============================================================================
namespace {
// A clean periodic footfall: `duty` of every `period` ticks in contact, one leg only
// (leg 0), the rest permanently planted so nothing else moves the clock.
std::vector<float> contact_train(uint64_t t, int n_legs, int period, int duty) {
    std::vector<float> c(n_legs, 1.0f);
    c[0] = (int(t % uint64_t(period)) < duty) ? 1.0f : 0.0f;
    return c;
}
ParamMap steplock_params() {
    auto p = base_params();
    p["stroke_gain"]    = 1.0;
    p["stroke_phase"]   = 0.0;
    p["contact_topic"]  = std::string("mt.contact");
    p["contact_instrument_only"] = 1.0;   // the stance-gate swap is separately REFUTED
    return p;
}
} // namespace

TEST(MotorEPM, StepPhaseSrcZeroIsByteIdentical) {
    // The gain-0 guard, and deliberately the HARD version of it: contact is on the bus and
    // the legs really are stepping, so the clock would run if anything read the parameter
    // wrong.  Byte-identity must come from the explicit branch at the stroke site, not
    // from the clock happening to be idle.
    auto p0 = steplock_params(); p0["stroke_phase_src"] = 0.0;
    auto pref = steplock_params(); pref["contact_topic"] = std::string("");
    Fixture f0(p0, 4, 3), fref(pref, 4, 3);
    double maxdiff = 0.0;
    for (uint64_t t = 0; t < 400; ++t) {
        Sensors s; s.contact = contact_train(t, 4, 26, 20);
        Sensors sref;                                   // reference gets no contact at all
        f0.run_tick(t, s); fref.run_tick(t, sref);
        if (t < 20) continue;
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j)
                maxdiff = std::max(maxdiff, double(std::abs(f0.accel(leg, j) - fref.accel(leg, j))));
    }
    EXPECT_LT(maxdiff, 1e-9)
        << "stroke_phase_src=0 must be byte-identical to an unwired contact_topic, even "
           "while the feet are visibly stepping on the bus (the gain-0 guard)";
}

TEST(MotorEPM, StepPhaseIsZeroAtTouchdownAndRampsAcrossTheStep) {
    auto p = steplock_params(); p["stroke_phase_src"] = 1.0;
    Fixture f(p, 4, 3);
    const int period = 26, duty = 20;
    // Warm up past babble and past the two touchdowns the clock needs to lock.
    for (uint64_t t = 0; t < 200; ++t) f.run_tick(t, {.contact = contact_train(t, 4, period, duty)});

    auto phase_of_leg0 = [&]() {
        auto d = f.m.diag_snapshot();
        return d["step_phase_legs"][0].get<double>();
    };
    // Only leg 0 steps in this fixture; legs 1-3 are held permanently planted, so they
    // never see a touchdown and must NOT lock.  That is the per-leg fallback doing its
    // job, and asserting it here is stronger than asserting a global lock: a leg with no
    // step data has to keep using L.phase rather than inherit a clock from its neighbours.
    EXPECT_NEAR(f.m.diag_snapshot()["step_lock"].get<double>(), 0.25, 1e-9)
        << "exactly the one stepping leg should lock; the three planted legs must not";
    for (int leg = 1; leg < 4; ++leg)
        EXPECT_LT(f.m.diag_snapshot()["step_phase_legs"][leg].get<double>(), 0.0)
            << "planted leg " << leg << " never touched down, so it must still be on L.phase";
    EXPECT_NEAR(f.m.diag_snapshot()["step_period"].get<double>(), double(period), 2.0)
        << "the measured step period should recover the 26-tick contact train";

    // Step through one full cycle and check phi is monotone between touchdowns and
    // returns to ~0 at the next one.  Touchdown for leg 0 is t % period == 0.
    std::vector<double> phases;
    for (uint64_t t = 200; t < 200 + uint64_t(period) + 1; ++t) {
        f.run_tick(t, {.contact = contact_train(t, 4, period, duty)});
        phases.push_back(phase_of_leg0());
    }
    // t=200 is a touchdown tick (200 % 26 == 18, so find the real one).
    int td = -1;
    for (size_t k = 1; k + 1 < phases.size(); ++k)
        if (phases[k] < phases[k - 1] && phases[k] < 0.5) { td = int(k); break; }
    ASSERT_GE(td, 0) << "no phase reset observed within one step period";
    EXPECT_LT(phases[size_t(td)], 0.5)
        << "phi must be ~0 AT touchdown — that is the entire point of the lock";
    // ...and rising afterwards, i.e. the clock advances rather than sitting at 0.
    ASSERT_LT(size_t(td) + 5, phases.size());
    EXPECT_GT(phases[size_t(td) + 5], phases[size_t(td)])
        << "phi must advance between touchdowns (free-run on the measured period)";
}

TEST(MotorEPM, StepPhaseDebounceRejectsContactBounce) {
    // A single-tick contact flicker mid-swing must NOT be read as a touchdown.  Without
    // this the clock resets twice per step, which is precisely the chatter that makes the
    // incumbent foot-height detector (12-15 ticks against a 26-30 tick step) unusable.
    auto p = steplock_params(); p["stroke_phase_src"] = 1.0; p["step_phase_debounce"] = 3.0;
    Fixture f(p, 4, 3);
    const int period = 26, duty = 20;
    for (uint64_t t = 0; t < 200; ++t) f.run_tick(t, {.contact = contact_train(t, 4, period, duty)});

    // Drive a clean cycle, but inject a 1-tick bounce deep in swing (t%period == 23).
    double min_phase_after_bounce = 1e9;
    for (uint64_t t = 200; t < 400; ++t) {
        auto c = contact_train(t, 4, period, duty);
        const bool bounce = (int(t % uint64_t(period)) == 23);
        if (bounce) c[0] = 1.0f;                       // spurious touch
        f.run_tick(t, {.contact = c});
        if (bounce) {
            // Phase at a bounce is deep in the cycle (23/26 of 2pi ~ 5.6 rad).  A reset
            // would drop it to ~0.
            min_phase_after_bounce = std::min(min_phase_after_bounce,
                                              f.m.diag_snapshot()["step_phase_legs"][0].get<double>());
        }
    }
    EXPECT_GT(min_phase_after_bounce, 3.0)
        << "a 1-tick contact bounce must not reset the step phase when debounce=3 "
           "(observed min phase " << min_phase_after_bounce << " rad)";
}

TEST(MotorEPM, StepPhaseFallsBackToLegPhaseBeforeLock) {
    // Before two touchdowns the clock has no measured period, so the stroke must keep
    // using L.phase.  Verified against an src=0 arm over the pre-lock window: identical
    // output there, and NOT identical later once the clock engages (otherwise this test
    // would also pass if the lever were dead code).
    auto plk = steplock_params(); plk["stroke_phase_src"] = 1.0;
    auto p0  = steplock_params(); p0["stroke_phase_src"]  = 0.0;
    Fixture flk(plk, 4, 3), f0(p0, 4, 3);
    const int period = 60, duty = 45;         // slow steps → a long pre-lock window
    double pre_max = 0.0, post_max = 0.0;
    for (uint64_t t = 0; t < 400; ++t) {
        Sensors s; s.contact = contact_train(t, 4, period, duty);
        flk.run_tick(t, s); f0.run_tick(t, s);
        if (t < 20) continue;
        double d = 0.0;
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j)
                d = std::max(d, double(std::abs(flk.accel(leg, j) - f0.accel(leg, j))));
        // Leg 0's second touchdown lands at t=120; everything before that is pre-lock.
        if (t < 100) pre_max = std::max(pre_max, d);
        else         post_max = std::max(post_max, d);
    }
    EXPECT_LT(pre_max, 1e-9)
        << "before the clock locks, the stroke must fall back to L.phase (identical to src=0)";
    EXPECT_GT(post_max, 1e-4)
        << "after the clock locks the stroke must actually change — otherwise this lever "
           "is dead code and the pre-lock check above proves nothing";
}

TEST(MotorEPM, StepPeriodRailsRejectAnomalousStride) {
    // One absurdly long gap must not drag the period estimate: it is clamped to
    // step_period_max before entering the EMA.
    auto p = steplock_params();
    p["stroke_phase_src"] = 1.0;
    p["step_period_max"]  = 40.0;
    p["step_period_alpha"] = 1.0;              // worst case: the EMA takes the sample whole
    Fixture f(p, 4, 3);
    const int period = 26, duty = 20;
    for (uint64_t t = 0; t < 200; ++t) f.run_tick(t, {.contact = contact_train(t, 4, period, duty)});
    // Now hold leg 0 airborne for 300 ticks, then touch down: a 300-tick "stride".
    for (uint64_t t = 200; t < 500; ++t) {
        std::vector<float> c(4, 1.0f); c[0] = 0.0f;
        f.run_tick(t, {.contact = c});
    }
    for (uint64_t t = 500; t < 520; ++t) {
        std::vector<float> c(4, 1.0f);
        f.run_tick(t, {.contact = c});
    }
    EXPECT_LE(f.m.diag_snapshot()["step_period"].get<double>(), 41.0)
        << "an anomalous 300-tick gap must be clamped by step_period_max before it "
           "enters the period EMA";
}

TEST(MotorEPM, GaitRasterZeroIsInertAndOnRecordsFootfalls) {
    // The raster feeds no command and draws no randomness, so byte-identity is
    // structural — but CLAUDE.md says verify a gain-0 guard by MEASUREMENT, not argument.
    auto poff = steplock_params(); poff["gait_raster_diag"] = 0.0;
    auto pon  = steplock_params(); pon["gait_raster_diag"]  = 1.0;
    Fixture foff(poff, 4, 3), fon(pon, 4, 3);
    double maxdiff = 0.0;
    for (uint64_t t = 0; t < 200; ++t) {
        Sensors s; s.contact = contact_train(t, 4, 26, 20);
        foff.run_tick(t, s); fon.run_tick(t, s);
        if (t < 20) continue;
        for (int leg = 0; leg < 4; ++leg)
            for (int j = 0; j < 3; ++j)
                maxdiff = std::max(maxdiff, double(std::abs(foff.accel(leg, j) - fon.accel(leg, j))));
    }
    EXPECT_LT(maxdiff, 1e-9) << "gait_raster_diag must not perturb the command path";
    EXPECT_FALSE(foff.m.diag_snapshot().contains("gait_raster"));
    auto d = fon.m.diag_snapshot();
    ASSERT_TRUE(d.contains("gait_raster"));
    auto r = d["gait_raster"].get<std::vector<int>>();
    ASSERT_FALSE(r.empty());
    // Leg 0's contact bit must actually toggle across the window (bit 0), and legs 1-3
    // must stay planted (bits 1-3 always set) — i.e. the raster reports the truth it was
    // handed rather than a constant.
    int leg0_on = 0, leg1_off = 0;
    for (int w : r) { if (w & 1) ++leg0_on; if (!(w & 2)) ++leg1_off; }
    EXPECT_GT(leg0_on, 0);
    EXPECT_LT(leg0_on, int(r.size())) << "leg 0's contact bit must toggle, not latch";
    EXPECT_EQ(leg1_off, 0) << "legs 1-3 were held planted and must read as planted";
}

// =============================================================================
// 12. THE PLL IS CONTINUOUS; THE SNAP IS NOT (`step_phase_lock`, 2026-07-27).
//
// This is the regression guard for the failure that killed the first build of the
// stroke-to-step lock.  Snapping `step_phase = 0` at every touchdown collapsed the gait
// (corridor n=4: net_z 4.58 -> -0.16, tilt_sd 0.065 -> 0.34, the body inverting
// repeatedly) because `sin(phi + stroke_phase)` is a CONTINUOUS motor command: stepping
// phi steps the command every time a foot lands off-schedule.
//
// The property that must hold forever after: at the default lock the phase advances by
// approximately omega per tick and NEVER jumps, even across a touchdown that arrives
// early.  Stated as a bound on the per-tick phase increment, which is the quantity the
// motor command actually sees.
// =============================================================================
TEST(MotorEPM, PhaseLockPullIsContinuousWhileSnapJumps) {
    const int period = 26, duty = 20;
    // An EARLY touchdown is the adversarial case: the clock expects the foot at phi=2pi
    // and gets it at phi~pi, so a snap has the largest possible discontinuity to make.
    auto run = [&](double lock) {
        auto p = steplock_params();
        p["stroke_phase_src"] = 1.0;
        p["step_phase_lock"]  = lock;
        Fixture f(p, 4, 3);
        for (uint64_t t = 0; t < 200; ++t)
            f.run_tick(t, {.contact = contact_train(t, 4, period, duty)});
        double prev = f.m.diag_snapshot()["step_phase_legs"][0].get<double>();
        double max_jump = 0.0;
        for (uint64_t t = 200; t < 400; ++t) {
            // Halve the period from here: every touchdown now arrives "early".
            std::vector<float> c(4, 1.0f);
            c[0] = (int(t % 13) < 9) ? 1.0f : 0.0f;
            f.run_tick(t, {.contact = c});
            double ph = f.m.diag_snapshot()["step_phase_legs"][0].get<double>();
            if (ph < 0.0 || prev < 0.0) { prev = ph; continue; }   // unlocked leg
            double d = std::fabs(ph - prev);
            if (d > M_PI) d = 2.0 * M_PI - d;                      // ignore the 2pi wrap
            max_jump = std::max(max_jump, d);
            prev = ph;
        }
        return max_jump;
    };
    const double soft = run(0.10);
    const double snap = run(1.0);
    // omega for a 13-tick period is ~0.48 rad/tick; the soft pull adds at most 10% of the
    // error on a touchdown tick, so ~1.0 rad is a generous ceiling that a snap blows past.
    EXPECT_LT(soft, 1.0)
        << "at step_phase_lock=0.10 the driven phase must advance smoothly (max per-tick "
           "jump " << soft << " rad) — a jump here is a step discontinuity in the motor "
           "command at every off-schedule footfall, which is what collapsed the gait";
    EXPECT_GT(snap, soft * 1.5)
        << "step_phase_lock=1.0 must still reproduce the hard snap (max jump " << snap
        << " rad), so the refuted form stays reproducible rather than becoming folklore";
}

// =============================================================================
// 13. A RESPAWN RE-ANCHORS THE STEP CLOCK (2026-07-27).
//
// A teleport/fall is a discontinuity in the body's contact history.  Without this the leg
// keeps `step_locked` with `last_td_tick` pointing at a touchdown from BEFORE the
// respawn, so the stroke drives off a stale phase through the whole post-reset settle —
// the same shape the ledger already records once ("MotorEPM's leg-phase/EMA survived
// fall+respawn -> any trend across a reset was fake").
// =============================================================================
TEST(MotorEPM, ResetReanchorsTheStepClock) {
    auto p = steplock_params();
    p["stroke_phase_src"] = 1.0;
    Fixture f(p, 4, 3);
    for (uint64_t t = 0; t < 200; ++t)
        f.run_tick(t, {.contact = contact_train(t, 4, 26, 20)});
    ASSERT_GT(f.m.diag_snapshot()["step_lock"].get<double>(), 0.0) << "precondition: locked";

    f.run_tick(200, {.contact = contact_train(200, 4, 26, 20)}, "reset");
    EXPECT_NEAR(f.m.diag_snapshot()["step_lock"].get<double>(), 0.0, 1e-9)
        << "a respawn must drop the step clock to unlocked so the stroke falls back to "
           "L.phase until two REAL touchdowns are seen again";
    // ...and it must re-lock on its own afterwards, rather than being permanently dead.
    for (uint64_t t = 201; t < 400; ++t)
        f.run_tick(t, {.contact = contact_train(t, 4, 26, 20)});
    EXPECT_GT(f.m.diag_snapshot()["step_lock"].get<double>(), 0.0)
        << "the clock must re-lock from post-reset touchdowns";
}
