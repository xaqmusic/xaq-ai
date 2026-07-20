// =============================================================================
// test_body_rhythm_tracker.cpp
//   BodyRhythmTracker (L-1b step 3): the measurement-seeded PLL locks to the
//   body's gait fundamental via a trot collective coordinate.  Verifies (i) the
//   frequency (from the unbiased up-crossing interval) converges to a known input
//   period and phase locks; (ii) the diagonal coordinate rejects common-mode
//   postural motion; (iii) snapshot/restore round-trips the PLL state.
// =============================================================================

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/BodyRhythmTracker.hpp"
#include "ogma/Topics.hpp"

namespace {

using ogma::ParamMap;
constexpr float TWO_PI = 6.28318530717958647692f;
const char* LEGS[4] = {"leg.fl", "leg.fr", "leg.rl", "leg.rr"};

ParamMap tr_params() {
    ParamMap p;
    p["proprio_topics"] = std::vector<std::string>{LEGS[0], LEGS[1], LEGS[2], LEGS[3]};
    p["output_topic"]   = std::string("rhythm.body.gait");
    p["leg_signs"]      = std::vector<double>{1.0, -1.0, -1.0, 1.0};   // trot diagonal
    p["motor_dim"]      = int64_t{3};
    p["swing_joint"]    = int64_t{0};
    p["init_period"]    = 60.0;      // start deliberately OFF the true period
    p["omega_lp"]       = 0.05;
    p["phase_lock"]     = 0.1;
    return p;
}

// Publish one proprio frame: joint-0 position per leg (index 0 of [pos,act,delta]×3), then tick.
void frame(ogma::InProcessBus& bus, ogma::BodyRhythmTracker& tr, uint64_t t,
           std::array<float, 4> const& j0) {
    bus.begin_tick(t);
    for (int l = 0; l < 4; ++l) {
        auto pt = std::make_shared<ogma::ProprioToken>();
        pt->values = Eigen::VectorXf::Zero(9);
        pt->values[0] = j0[l];
        bus.publish(LEGS[l], pt);
    }
    tr.tick(t);
    bus.end_tick();
}

} // namespace

TEST(BodyRhythmTracker, LocksToInputFrequency) {
    ogma::InProcessBus bus; ogma::BodyRhythmTracker tr; tr.set_id("brt");
    tr.on_setup(&bus, tr_params());
    EXPECT_EQ(tr.type_name(), "BodyRhythmTracker");
    EXPECT_EQ(tr.n_legs(), 4);

    const float period_true = 40.0f;                 // ω_true = 2π/40 ≈ 0.157 rad/tick
    const float w_true = TWO_PI / period_true;
    const float A = 0.25f;                           // diagonal sum ⇒ F amplitude ≈ 4A = 1.0

    for (uint64_t t = 0; t < 3000; ++t) {
        float s = A * std::sin(w_true * float(t));
        // Trot: fl,rr in phase (+); fr,rl anti-phase (−) → collective coordinate reinforces.
        frame(bus, tr, t, {s, -s, -s, s});
    }

    EXPECT_TRUE(tr.locked()) << "must have measured ≥2 up-crossings";
    EXPECT_NEAR(tr.period_est(), period_true, 2.0f)
        << "PLL frequency must lock to the input gait fundamental (got " << tr.period_est() << ")";
    // The reference token carries [cos φ, sin φ, ω] on the output topic.
    auto ref = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("rhythm.body.gait"));
    ASSERT_TRUE(ref != nullptr);
    ASSERT_EQ(ref->values.size(), 3);
    EXPECT_NEAR(ref->values[2], w_true, 0.01f) << "published ω_body must equal the locked frequency";
    EXPECT_NEAR(ref->values[0] * ref->values[0] + ref->values[1] * ref->values[1], 1.0f, 1e-3f)
        << "[cos φ, sin φ] must be a unit vector";
}

TEST(BodyRhythmTracker, PhaseIsStablyLocked) {
    // Phase lock = the body-phase↔input-phase relationship is STABLE (doesn't drift), tolerant of
    // a constant offset (hysteresis fires the crossing a few ticks past the true zero). Sampling
    // φ_body at a FIXED input phase each cycle (t ≡ 0 mod 40) should give a nearly constant value.
    ogma::InProcessBus bus; ogma::BodyRhythmTracker tr; tr.set_id("brt");
    auto pp = tr_params(); pp["init_period"] = 40.0;   // near-locked start
    tr.on_setup(&bus, pp);

    const float w = TWO_PI / 40.0f;
    std::vector<float> late;                           // φ sampled at a fixed input phase, settled window
    for (uint64_t t = 0; t < 6000; ++t) {
        float s = 0.25f * std::sin(w * float(t));
        frame(bus, tr, t, {s, -s, -s, s});
        if (t > 4000 && (t % 40 == 0)) late.push_back(tr.phi_body());   // same input phase every cycle
    }
    ASSERT_GT(int(late.size()), 20);
    // Locked ⇒ φ at a fixed input phase barely moves cycle-to-cycle (a constant offset is fine;
    // residual drift → 0). Bound the per-cycle drift, not the absolute value.
    float max_drift = 0.0f;
    for (size_t i = 1; i < late.size(); ++i)
        max_drift = std::max(max_drift, std::fabs(std::remainder(late[i] - late[i - 1], TWO_PI)));
    EXPECT_LT(max_drift, 0.05f) << "φ_body drift per cycle must be small once locked (got " << max_drift << ")";
}

TEST(BodyRhythmTracker, CommonModeRejected) {
    // A whole-body offset that moves every leg IDENTICALLY is not a gait — the diagonal
    // coordinate (signs sum to 0) must cancel it, so the PLL sees ~no drive.
    ogma::InProcessBus bus; ogma::BodyRhythmTracker tr; tr.set_id("brt");
    tr.on_setup(&bus, tr_params());

    const float w = TWO_PI / 40.0f;
    for (uint64_t t = 0; t < 4000; ++t) {
        float s = 0.5f * std::sin(w * float(t));
        frame(bus, tr, t, {s, s, s, s});   // identical on all legs → F ≈ 0
    }
    nlohmann::json d = tr.diag_snapshot();
    float amp0 = d["amp_per_joint"][0].get<float>();
    EXPECT_LT(amp0, 0.05f) << "common-mode motion must be rejected by the diagonal coordinate";
    EXPECT_FALSE(tr.locked()) << "no gait rhythm ⇒ no lock";
}

TEST(BodyRhythmTracker, SnapshotRestoreRoundTrips) {
    ogma::InProcessBus bus; ogma::BodyRhythmTracker tr; tr.set_id("brt");
    tr.on_setup(&bus, tr_params());

    const float w = TWO_PI / 45.0f;
    for (uint64_t t = 0; t < 3000; ++t) {
        float s = 0.25f * std::sin(w * float(t));
        frame(bus, tr, t, {s, -s, -s, s});
    }
    auto snap = tr.snapshot_state();
    EXPECT_EQ(snap["version"].get<int>(), 1);

    ogma::BodyRhythmTracker tr2; tr2.set_id("brt");
    tr2.on_setup(&bus, tr_params());
    tr2.restore_state(snap);
    EXPECT_NEAR(tr2.omega(),    tr.omega(),    1e-6f);
    EXPECT_NEAR(tr2.phi_body(), tr.phi_body(), 1e-4f);
    EXPECT_EQ(tr2.locked(), tr.locked());
}
