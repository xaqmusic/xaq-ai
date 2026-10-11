// Tests for ogma/hw/Actuation.hpp — the brain's action channels to HAT pulses, and the
// per-tick authority decision benchd makes about the brain's command stream.  The mapping
// tests read the real calibration files, so a re-export that breaks parity fails here.
#include "ogma/hw/Actuation.hpp"
#include <gtest/gtest.h>

using namespace ogma::hw::brain;

namespace {
const std::string kSrc = PI_HOST_SOURCE_DIR;
ServoMapping real_map() { return ServoMapping::load(kSrc + "/calib/servo_map.json"); }
ActionMap    real_amap() { return ActionMap::load(kSrc + "/calib/body_measured_fsr.json"); }
}

TEST(ActionMap, TheExportCarriesTheDiscreteMapping) {
    const ActionMap m = real_amap();
    ASSERT_TRUE(m.ok) << m.why;
    EXPECT_NEAR(m.hip1_range, 1.40, 1e-12);
    EXPECT_NEAR(m.hip2_range, 1.40, 1e-12);
    EXPECT_NEAR(m.knee_fold, 3.20, 1e-12);
    EXPECT_NEAR(m.knee_hyperext, 0.85, 1e-12);
    EXPECT_NEAR(m.knee_rest, -1.6, 1e-12);
    EXPECT_TRUE(m.knee_widening);
    EXPECT_GE(m.u_check.size(), 24u);
}

TEST(ActionMap, ReproducesTheSimsOwnTargetsAtEveryExportedSample) {
    // The parity check: the sim computed these with _discrete_joint_targets after the brain
    // path's clamp and splay sign.  Samples include out-of-range u (the clamp) and both knee
    // branches.  Exact to double rounding, because it is the same arithmetic in the same order.
    const ActionMap m = real_amap();
    ASSERT_TRUE(m.ok) << m.why;
    int n = 0;
    for (const auto& s : m.u_check) {
        const int leg = s.at("leg").get<int>();
        const auto& u = s.at("u");
        const auto t = leg_targets_from_u(leg, u[0].get<double>(), u[1].get<double>(), u[2].get<double>(), m);
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(t[size_t(j)], s.at("t")[size_t(j)].get<double>(), 1e-12) << "leg " << leg << " j " << j;
        ++n;
    }
    EXPECT_EQ(n, 24);
}

TEST(ActionMap, AnUnportedBackendIsRefusedNotApproximated) {
    nlohmann::json body = nlohmann::json::parse(std::ifstream(kSrc + "/calib/body_measured_fsr.json"));
    body["action_map"]["backend"] = "bernoulli_impulse";
    const ActionMap m = ActionMap::from_body_json(body);
    EXPECT_FALSE(m.ok);
    body.erase("action_map");
    EXPECT_FALSE(ActionMap::from_body_json(body).ok);
}

TEST(ActuationPulses, InverseOfTheJointsInputRoundTripsWithinHalfAMicrosecond) {
    // The brain reads its own joints from benchd's commanded pulses via hinge_angles_from_us.
    // The command path must be that function's inverse, or the brain's proprioception and its
    // motor output disagree about where a leg is.
    const ServoMapping map = real_map();
    ASSERT_TRUE(map.complete) << map.why;
    const ActionMap m = real_amap();
    ASSERT_TRUE(m.ok) << m.why;
    for (const auto& s : m.u_check) {
        std::array<double, 12> u{};
        const int leg = s.at("leg").get<int>();
        for (int j = 0; j < 3; ++j) u[size_t(leg * 3 + j)] = s.at("u")[size_t(j)].get<double>();
        const auto t = joint_targets_from_u(u, m);
        const auto us = us_from_hinge_angles(t, map, 545.2);
        for (int c = 0; c < 12; ++c) EXPECT_GT(us[size_t(c)], 0) << "every channel is commanded";
        const auto back = hinge_angles_from_us(us, map, 545.2);
        for (int k = 0; k < 12; ++k) EXPECT_NEAR(back[size_t(k)], t[size_t(k)], 0.5 / 545.2 + 1e-12);
    }
}

TEST(ActuationPulses, RestIsTheOriginOnTheHipsAndTheMeasuredScaleOnTheKnee) {
    // u = 0 everywhere: hips at origin_us; the knee at KNEE_REST -1.6 rad, which on a sign -1
    // channel is origin + 1.6 * 545.2 = origin + 872 us.  Unclamped here on purpose — the
    // envelope is ServoDriver's — so a knee rest past max_us is visible in the record.
    const ServoMapping map = real_map();
    const ActionMap m = real_amap();
    ASSERT_TRUE(map.complete && m.ok);
    const auto us = us_from_hinge_angles(joint_targets_from_u({}, m), map, 545.2);
    for (int l = 0; l < 4; ++l) {
        EXPECT_EQ(us[size_t(map.by_lj[size_t(l * 3 + 0)].ch)], int(map.by_lj[size_t(l * 3 + 0)].origin_us));
        EXPECT_EQ(us[size_t(map.by_lj[size_t(l * 3 + 1)].ch)], int(map.by_lj[size_t(l * 3 + 1)].origin_us));
        const ServoChannel& k = map.by_lj[size_t(l * 3 + 2)];
        EXPECT_EQ(us[size_t(k.ch)], int(std::lround(k.origin_us + k.sign * -1.6 * 545.2)));
    }
}

TEST(ActuationTopics, AreTheSimsRegisteredChannelsInLegJointOrder) {
    EXPECT_EQ(action_topics()[0], "action.fl_hip1");
    EXPECT_EQ(action_topics()[5], "action.fr_knee");
    EXPECT_EQ(action_topics()[11], "action.rr_knee");
}

// ---- BrainAuthority -------------------------------------------------------------------

TEST(BrainAuthority, BenchModeNeverActsOnTheStream) {
    BrainAuthority a;
    a.grant(0);
    EXPECT_EQ(a.tick(RunMode::Bench, 20, true, false), BrainAuthority::Event::None);
}

TEST(BrainAuthority, SilenceBeforeTheFirstCommandIsNotAFault) {
    // dev mode entered before ogma_host starts must not latch a stop.
    BrainAuthority a;
    a.grant(0);
    for (int64_t t = 0; t < 10000; t += 20)
        EXPECT_EQ(a.tick(RunMode::Dev, t, false, false), BrainAuthority::Event::None);
    EXPECT_EQ(a.tick(RunMode::Dev, 10000, true, false), BrainAuthority::Event::Apply);
}

TEST(BrainAuthority, DevFaultsOnceWhenTheStreamGoesQuiet) {
    BrainAuthority a;
    a.grant(0);
    EXPECT_EQ(a.tick(RunMode::Dev, 0, true, false), BrainAuthority::Event::Apply);
    int faults = 0;
    for (int64_t t = 20; t <= 6000; t += 20) {
        const auto e = a.tick(RunMode::Dev, t, false, false);
        if (e == BrainAuthority::Event::Fault) { ++faults; EXPECT_GT(t, 200); EXPECT_LE(t, 220); }
        EXPECT_NE(e, BrainAuthority::Event::Rescue) << "dev freezes; it does not move the body";
    }
    EXPECT_EQ(faults, 1);
}

TEST(BrainAuthority, AutonomousHoldsThenRescuesThenRecovers) {
    BrainAuthority a;
    a.grant(0);
    a.tick(RunMode::Autonomous, 0, true, false);
    int holds = 0, rescues = 0;
    for (int64_t t = 20; t <= 8000; t += 20) {
        const auto e = a.tick(RunMode::Autonomous, t, false, false);
        if (e == BrainAuthority::Event::Hold) ++holds;
        if (e == BrainAuthority::Event::Rescue) { ++rescues; EXPECT_GT(t, 5000); }
    }
    EXPECT_EQ(holds, 1);
    EXPECT_EQ(rescues, 1);
    EXPECT_EQ(a.tick(RunMode::Autonomous, 8020, true, false), BrainAuthority::Event::Apply);
    EXPECT_EQ(a.regains(), 1);
    EXPECT_FALSE(a.holding());
}

TEST(BrainAuthority, ABlockedTickNeverAppliesAndResumeIsNotCalledALoss) {
    // A stop of several seconds, then resume: the last command is old, but staleness counts
    // from the end of the block, so the first unblocked ticks are not a "stream lost".
    BrainAuthority a;
    a.grant(0);
    a.tick(RunMode::Dev, 0, true, false);
    for (int64_t t = 20; t <= 4000; t += 20)
        EXPECT_EQ(a.tick(RunMode::Dev, t, true, true), BrainAuthority::Event::None);
    EXPECT_EQ(a.tick(RunMode::Dev, 4020, false, false), BrainAuthority::Event::None);
    EXPECT_EQ(a.tick(RunMode::Dev, 4100, false, false), BrainAuthority::Event::None);
    EXPECT_EQ(a.tick(RunMode::Dev, 4120, true, false), BrainAuthority::Event::Apply);
    EXPECT_EQ(a.losses(), 0);
}

// ---- RateGuard --------------------------------------------------------------------------

TEST(RateGuard, AdmitsTheDashboardsTwentyHertzAndRefusesABrainStream) {
    RateGuard dash(30, 1000);
    for (int64_t t = 0; t < 5000; t += 50) EXPECT_TRUE(dash.admit(t));   // 20 Hz forever
    RateGuard brain(30, 1000);
    int refused = 0;
    for (int64_t t = 0; t < 1000; t += 20)                              // 50 Hz x 12 channels
        for (int c = 0; c < 12; ++c) refused += brain.admit(t) ? 0 : 1;
    EXPECT_GT(refused, 500);
    EXPECT_EQ(brain.refused(), refused);
}
