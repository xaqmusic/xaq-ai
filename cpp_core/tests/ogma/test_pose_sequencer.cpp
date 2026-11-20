// PoseSequencer: the shared FK on a published leg model, the IK, and the crawl's state machine on flat ground.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/PoseSequencer.hpp"
#include "stepper_fixture.hpp"
namespace {
using ogma::PoseSequencer; using ogma::body::Vec3f;
using namespace stepper_test;
struct Fix {
    ogma::InProcessBus bus; PoseSequencer m; std::vector<float> out;
    Fix(ogma::ParamMap p = {}) {
        std::vector<std::string> acts; for (auto l : {"fl", "fr", "rl", "rr"}) for (auto j : {"hip1", "hip2", "knee"}) acts.push_back(std::string("action.") + l + "_" + j);
        p.emplace("gait_action_topics", ogma::ParamValue{acts});
        m.set_id("pose_sequencer"); m.on_setup(&bus, p);
        bus.subscribe("pose.targets", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); out.assign(t->values.data(), t->values.data() + t->values.size()); });
    }
    void run(uint64_t t, std::vector<float> contact, float margin = 0.03f, std::vector<float> up = {0, 1, 0}) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.leg_model", pv(leg_model())); bus.publish("support.state", pv(support(margin))); bus.publish("terrain.patch", pv(flat_patch(-0.09f)));
        bus.publish("reality.proprio.up_body", pv(up)); bus.publish("reality.proprio.foot_contact", pv(contact));
        for (int k = 0; k < 12; ++k) { auto a = std::make_shared<ogma::ActionOut>(); a->accel = (k % 3 == 2) ? 0.5f : 0.0f; static const char* nm[12] = {"action.fl_hip1","action.fl_hip2","action.fl_knee","action.fr_hip1","action.fr_hip2","action.fr_knee","action.rl_hip1","action.rl_hip2","action.rl_knee","action.rr_hip1","action.rr_hip2","action.rr_knee"}; bus.publish(nm[k], a); }
        m.tick(t); bus.end_tick();
    }
};
}

TEST(PoseSequencer, EngageZeroPublishesZeroAndTheModelLoads) {
    Fix f; for (uint64_t t = 0; t < 20; ++t) f.run(t, {1, 1, 1, 1});
    ASSERT_EQ(f.out.size(), 13u); EXPECT_FLOAT_EQ(f.out[0], 0.0f);
    EXPECT_TRUE(f.m.model_ready()); EXPECT_EQ(f.m.phase(), PoseSequencer::Phase::Idle);
}

TEST(PoseSequencer, TheForwardKinematicsStandOnTheFloorAndTheInverseConverges) {
    Fix f; f.run(0, {1, 1, 1, 1});
    // the rest pose (knee u 0.3): the toes sit below the body and splayed to their corners
    for (int i = 0; i < 4; ++i) {
        const auto t = f.m.angles_of_u(i, {0.0, 0.0, 0.3});
        const Vec3f toe = f.m.toe_of(i, t);
        EXPECT_LT(toe.y, -0.05f) << "leg " << i; EXPECT_GT(std::fabs(toe.x), 0.07f); EXPECT_GT(std::fabs(toe.z), 0.07f);
        EXPECT_EQ(toe.x < 0, i == 0 || i == 2) << "legs 0 and 2 at -x"; EXPECT_EQ(toe.z > 0, i == 0 || i == 1) << "legs 0 and 1 forward";
        // a nearby target: the IK reaches it
        const Vec3f target(toe.x + 0.012f, toe.y + 0.02f, toe.z - 0.015f);
        auto a = t; bool done = false; int k = 0;
        for (; k < 40 && !done; ++k) done = f.m.ik_step(i, target, a);
        EXPECT_TRUE(done) << "leg " << i << " did not converge";
        const Vec3f got = f.m.toe_of(i, a);
        EXPECT_NEAR(got.x, target.x, 0.005); EXPECT_NEAR(got.y, target.y, 0.005); EXPECT_NEAR(got.z, target.z, 0.005);
        // and the round trip through action units is the identity within the ranges
        const auto u = f.m.u_of_angles(i, a); const auto back = f.m.angles_of_u(i, u);
        for (int j = 0; j < 3; ++j) EXPECT_NEAR(back[size_t(j)], a[size_t(j)], 1e-9);
    }
}

TEST(PoseSequencer, EngagedOnFlatGroundItCrawls_EveryLegStepsAndTheIkTracks) {
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}});
    std::vector<float> contact = {1, 1, 1, 1};
    std::vector<int> visited;
    for (uint64_t t = 0; t < 2500; ++t) {
        // the ground: a lowering toe touches when its target is at or below the floor (-0.09 in the body frame)
        const int leg = f.m.active_leg();
        for (int i = 0; i < 4; ++i) contact[size_t(i)] = 1.0f;
        if (leg >= 0 && (f.m.phase() == PoseSequencer::Phase::Lift || f.m.phase() == PoseSequencer::Phase::Swing)) contact[size_t(leg)] = 0.0f;
        if (leg >= 0 && f.m.phase() == PoseSequencer::Phase::Lower) {
            auto d = f.m.diag_snapshot(); const double y = d["toe_tgt"][leg][1].get<double>();
            contact[size_t(leg)] = y <= -0.09 ? 1.0f : 0.0f;
        }
        f.run(t, contact);
        if (leg >= 0 && (visited.empty() || visited.back() != leg)) visited.push_back(leg);
    }
    EXPECT_NEAR(f.m.engage_level(), 1.0f, 1e-6);
    int total = 0; for (int i = 0; i < 4; ++i) { EXPECT_GE(f.m.steps(i), 1) << "leg " << i << " never stepped"; total += f.m.steps(i); }
    EXPECT_GE(total, 4);
    auto d = f.m.diag_snapshot();
    EXPECT_LT(d["ik_err"].get<double>(), 0.015) << "the toes track their targets (moving targets lag by the step cap)";
    EXPECT_LE(d["shift_exhausted"].get<int>(), 2) << "with positive margins almost no shift is spent";
    EXPECT_EQ(d["deadlock_ticks"].get<int>(), 0);
    EXPECT_EQ(d["disengaged_tilt"].get<int>(), 0);
    // the commanded pose is published with the blend at 1 and within the action range
    ASSERT_EQ(f.out.size(), 13u); EXPECT_FLOAT_EQ(f.out[0], 1.0f);
    for (int k = 1; k < 13; ++k) { EXPECT_LE(f.out[size_t(k)], 1.0f); EXPECT_GE(f.out[size_t(k)], -1.0f); }
}

TEST(PoseSequencer, AToeThatMeetsSomethingMidSwingIsPutDownWhereItIs_AndAResetDropsToIdle) {
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}});
    std::vector<float> contact = {1, 1, 1, 1};
    // run until the first LIFT has the toe off the ground, then pretend it hits a wall: the flag comes back on in the air
    uint64_t t = 0; bool hit = false; int obstacle_before = 0;
    for (; t < 3000; ++t) {
        const int leg = f.m.active_leg(); const auto ph = f.m.phase();
        for (int i = 0; i < 4; ++i) contact[size_t(i)] = 1.0f;
        if (leg >= 0 && (ph == PoseSequencer::Phase::Lift || ph == PoseSequencer::Phase::Swing)) {
            auto d = f.m.diag_snapshot(); const double y = d["toe_tgt"][leg][1].get<double>();
            contact[size_t(leg)] = (y > -0.06) ? 1.0f : 0.0f;   // once the toe is 2.5 cm up it "touches the wall": the flag returns
            if (ph == PoseSequencer::Phase::Swing) { hit = true; obstacle_before = d["obstacle_hits"].get<int>(); }
        }
        if (leg >= 0 && ph == PoseSequencer::Phase::Lower) { auto d = f.m.diag_snapshot(); contact[size_t(leg)] = d["toe_tgt"][leg][1].get<double>() <= -0.09 ? 1.0f : 0.0f; }
        f.run(t, contact);
        if (f.m.diag_snapshot()["obstacle_hits"].get<int>() > 0) break;
    }
    auto d = f.m.diag_snapshot();
    EXPECT_GE(d["obstacle_hits"].get<int>(), 1) << "the swing must end on a contact in the air";
    EXPECT_EQ(f.m.phase(), PoseSequencer::Phase::Lower) << "and the toe is lowered where it is";
    // a reset event: the sequencer drops to idle with the blend at 0, then re-engages from the standing conditions
    f.bus.begin_tick(t + 1); auto ev = std::make_shared<ogma::EnvEvent>(); ev->name = "reset"; f.bus.publish("events.reset", ev); f.m.tick(t + 1); f.bus.end_tick();
    EXPECT_EQ(f.m.diag_snapshot()["resets_seen"].get<int>(), 1);
    EXPECT_LE(f.m.engage_level(), 0.1f) << "the blend dropped to zero on the reset (one ramp step may follow in the same tick)";
    EXPECT_TRUE(f.m.phase() == PoseSequencer::Phase::Idle || f.m.phase() == PoseSequencer::Phase::Settle) << "back to the start: idle, or already settling again from the standing conditions";
    for (uint64_t k = t + 2; k < t + 40; ++k) f.run(k, {1, 1, 1, 1});
    EXPECT_NE(f.m.phase(), PoseSequencer::Phase::Idle) << "re-engaged";
}

TEST(PoseSequencer, TheHeadingHoldTurnsThePlantedToesAgainstAYawError_AndTheOdometryIntegratesTheShift) {
    // a strong hold so the rotation dominates the shift's own motion within a few ticks
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}},
           {"heading_gain", ogma::ParamValue{5.0}}, {"yaw_rate_max", ogma::ParamValue{1.0}}});
    std::vector<float> contact = {1, 1, 1, 1}; std::vector<float> odom = {0.5f, 0.1f, 0.0f};
    // engage and settle with the body's yaw at 0; the odometry out passes the body's through until engaged
    std::vector<float> od_out;
    f.bus.subscribe("reality.proprio.odom_seq", ogma::SubscriptionKind::Direct, [&](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); od_out.assign(t->values.data(), t->values.data() + 3); });
    uint64_t t = 0;
    for (; t < 400 && f.m.phase() != PoseSequencer::Phase::Shift; ++t) { f.bus.begin_tick(t); f.bus.publish("reality.proprio.odom", pv(odom)); f.bus.end_tick(); f.run(t, contact); }
    ASSERT_EQ(f.m.phase(), PoseSequencer::Phase::Shift);
    EXPECT_NEAR(od_out[0], 0.5f, 1e-5); EXPECT_NEAR(od_out[1], 0.1f, 1e-5);
    // now the body reads a yaw of +0.2 (turned toward +x): during the shift a forward toe must move toward +x in the body frame
    auto d0 = f.m.diag_snapshot(); const double x_fl0 = d0["toe_tgt"][0][0].get<double>(), z_fl0 = d0["toe_tgt"][0][2].get<double>();
    odom[2] = 0.2f;
    for (int k = 0; k < 20 && f.m.phase() == PoseSequencer::Phase::Shift; ++k, ++t) { f.bus.begin_tick(t); f.bus.publish("reality.proprio.odom", pv(odom)); f.bus.end_tick(); f.run(t, contact); }
    auto d1 = f.m.diag_snapshot(); const double x_fl1 = d1["toe_tgt"][0][0].get<double>(), z_fl1 = d1["toe_tgt"][0][2].get<double>();
    EXPECT_NEAR(f.m.yaw_err(), 0.2, 1e-6);
    // the toe's bearing about the body's vertical swings toward +x: the body turns back toward -x
    const double b0 = std::atan2(x_fl0, z_fl0), b1 = std::atan2(x_fl1, z_fl1);
    EXPECT_GT(b1 - b0, 0.05) << "the front-left toe (at -x, +z) rotates toward +x so the body turns back toward -x";
    // the odometry advanced with the shift along the heading, not merely passing the body's through
    EXPECT_GT(f.m.odom_x(), 0.5) << "the body advanced over the toes";
}

TEST(PoseSequencer, ThePoseSearchCrawlsOnFlatGround_EveryLegStepsAndEveryPlanIsFeasible) {
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}, {"plan", ogma::ParamValue{1.0}}});
    std::vector<float> contact = {1, 1, 1, 1};
    int plans_checked = 0;
    for (uint64_t t = 0; t < 2500; ++t) {
        const int leg = f.m.active_leg();
        for (int i = 0; i < 4; ++i) contact[size_t(i)] = 1.0f;
        if (leg >= 0 && (f.m.phase() == PoseSequencer::Phase::Lift || f.m.phase() == PoseSequencer::Phase::Swing)) contact[size_t(leg)] = 0.0f;
        if (leg >= 0 && f.m.phase() == PoseSequencer::Phase::Lower) {
            auto d = f.m.diag_snapshot(); const double y = d["toe_tgt"][leg][1].get<double>();
            contact[size_t(leg)] = y <= -0.09 ? 1.0f : 0.0f;
        }
        f.run(t, contact);
        if (f.m.phase() == PoseSequencer::Phase::Shift && (t % 50) == 0) {   // the planner, asked again from the standing geometry: a feasible step within its grid and the leg's reach
            int l; Vec3f L; double sx, sz, sc;
            if (f.m.plan_step(l, L, sx, sz, sc)) {
                ++plans_checked;
                EXPECT_GE(l, 0); EXPECT_LE(l, 3);
                EXPECT_LE(std::fabs(sx), 0.03 + 1e-9); EXPECT_GE(sz, -0.03 - 1e-9); EXPECT_LE(sz, 0.04 + 1e-9);
                const double hx = (l == 1 || l == 3) ? 0.0383 : -0.0383, hz = (l == 0 || l == 1) ? 0.033 : -0.033;
                const double r = std::hypot(L.x - hx, L.z - hz);
                EXPECT_GE(r, 0.06); EXPECT_LE(r, 0.14) << "the landing stays within the leg's reach";
            }
        }
    }
    EXPECT_GT(plans_checked, 3);
    int total = 0; for (int i = 0; i < 4; ++i) { EXPECT_GE(f.m.steps(i), 1) << "leg " << i << " never stepped"; total += f.m.steps(i); }
    EXPECT_GE(total, 4);
    auto d = f.m.diag_snapshot();
    EXPECT_GT(d["plans"].get<int>(), 0); EXPECT_EQ(d["plan_none"].get<int>(), 0) << "on flat ground the search always has a step";
    EXPECT_EQ(d["deadlock_ticks"].get<int>(), 0); EXPECT_EQ(d["disengaged_tilt"].get<int>(), 0);
    EXPECT_LT(d["ik_err"].get<double>(), 0.015);
}

TEST(PoseSequencer, TheMultiStartSolveLeavesTheWrongBranch_WhereTheSingleSeedSitsAtAStop) {
    Fix f({{"ik_multistart", ogma::ParamValue{1.0}}}); f.run(0, {1, 1, 1, 1});
    for (int leg = 0; leg < 4; ++leg) {
        // a reachable target: the toe of a knee-forward, femur-down pose, low under the hip
        const auto good = f.m.angles_of_u(leg, {0.1, -0.5, 0.1});
        const Vec3f target = f.m.toe_of(leg, good);
        // the single-seed solve from the folded-up branch (femur fully up, knee folded) is left far from it
        auto a_single = f.m.angles_of_u(leg, {0.1, 1.0, -1.0});
        for (int k = 0; k < 40; ++k) f.m.ik_step(leg, target, a_single);
        const double e_single = std::hypot(std::hypot(f.m.toe_of(leg, a_single).x - target.x, f.m.toe_of(leg, a_single).y - target.y), f.m.toe_of(leg, a_single).z - target.z);
        // the multi-start from the same seed reaches it
        auto a_multi = f.m.angles_of_u(leg, {0.1, 1.0, -1.0});
        for (int k = 0; k < 10; ++k) f.m.ik_solve(leg, target, a_multi, true);
        const Vec3f got = f.m.toe_of(leg, a_multi);
        const double e_multi = std::hypot(std::hypot(got.x - target.x, got.y - target.y), got.z - target.z);
        EXPECT_LT(e_multi, 0.004) << "leg " << leg << ": the multi-start did not reach the target (single-seed residual " << e_single << ")";
        EXPECT_LE(e_multi, e_single + 1e-9) << "leg " << leg;
    }
    auto d = f.m.diag_snapshot(); EXPECT_GT(d["branch_switches"].get<int>(), 0);
}

TEST(PoseSequencer, TheFrontDropHoldsTheFrontToesShallowerOnANoseUpBody) {
    // two sequencers on the same nose-up body (the up vector leaning back by 0.2 rad): with the front drop the FRONT toes' targets
    // sit higher by gain x (pitch - deadband), the rear toes' where the levelling alone puts them
    Fix a({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}});
    Fix b({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}, {"front_drop_gain", ogma::ParamValue{0.2}}});
    const float s = std::sin(0.2f), c = std::cos(0.2f);
    for (uint64_t t = 0; t < 40; ++t) { a.run(t, {1, 1, 1, 1}, 0.03f, {0, c, s}); b.run(t, {1, 1, 1, 1}, 0.03f, {0, c, s}); }
    ASSERT_NE(a.m.phase(), PoseSequencer::Phase::Idle); ASSERT_NE(b.m.phase(), PoseSequencer::Phase::Idle);
    auto da = a.m.diag_snapshot(), db = b.m.diag_snapshot();
    const double expect = 0.2 * (0.2 - 0.05);
    EXPECT_NEAR(db["front_drop"].get<double>(), expect, 1e-3);
    for (int i = 0; i < 4; ++i) {
        const double ya = da["toe_tgt"][i][1].get<double>(), yb = db["toe_tgt"][i][1].get<double>(), z = da["toe_tgt"][i][2].get<double>();
        if (z > 0) EXPECT_NEAR(yb - ya, expect, 2e-3) << "front toe " << i; else EXPECT_NEAR(yb - ya, 0.0, 2e-3) << "rear toe " << i;
    }
}


TEST(PoseSequencer, TheExtensionMarginRefusesAShiftThatStraightensAPlantedLeg) {
    Fix f; f.run(0, {1, 1, 1, 1});
    // the slack: a straight knee (u 0) has none, a bent one (u 0.5) has centimetres; the plane estimate matches the true FK
    for (int leg = 0; leg < 4; ++leg) {
        for (double h2u : {-0.6, -0.3, 0.0}) {
            const Vec3f straight = f.m.toe_of(leg, f.m.angles_of_u(leg, {0.2, h2u, 0.0}));
            EXPECT_NEAR(f.m.ext_slack(leg, straight), 0.0, 0.003) << "leg " << leg << " hip2 u " << h2u;
            EXPECT_GT(f.m.ext_slack(leg, f.m.toe_of(leg, f.m.angles_of_u(leg, {0.2, h2u, 0.5}))), 0.02) << "leg " << leg;
        }
    }
    // a stance with the rear-left toe 1.5-2 cm short of full extension: a forward shift of 2 cm stretches it past a 1.5 cm margin
    const double hd[4][2] = {{-0.7071, 0.7071}, {0.7071, 0.7071}, {-0.7071, -0.7071}, {0.7071, -0.7071}};
    auto place = [&](int i, double r) { const double hx = (hd[i][0] > 0 ? 1 : -1) * 0.0383, hz = (hd[i][1] > 0 ? 1 : -1) * 0.033;
                                        return Vec3f(float(hx + hd[i][0] * r), -0.085f, float(hz + hd[i][1] * r)); };
    std::array<Vec3f, 4> toes{place(0, 0.08), place(1, 0.08), place(2, 0.08), place(3, 0.08)};
    double r2 = 0.08; while (r2 < 0.14 && f.m.ext_slack(2, place(2, r2)) > 0.019) r2 += 0.001;
    toes[2] = place(2, r2);
    const double s0 = f.m.ext_slack(2, toes[2]);
    ASSERT_GT(s0, 0.015); ASSERT_LT(s0, 0.02);
    ASSERT_TRUE(f.m.shift_feasible(1, toes, 0.0, 0.02)) << "with the margin off the forward shift is within the horizontal reach";
    f.m.on_param_change("ext_margin_m", ogma::ParamValue{0.015});
    EXPECT_FALSE(f.m.shift_feasible(1, toes, 0.0, 0.02)) << "the forward shift leaves the rear-left toe inside the margin";
    EXPECT_TRUE(f.m.shift_feasible(1, toes, 0.0, -0.01)) << "a backward shift bends it";
    EXPECT_TRUE(f.m.shift_feasible(2, toes, 0.0, 0.02)) << "the stretched leg's own lift is not refused by its own toe";
    toes[2] = place(2, r2 + 0.01);   // already inside the margin: a shift that does not stretch it further stays feasible
    ASSERT_LT(f.m.ext_slack(2, toes[2]), 0.015);
    EXPECT_TRUE(f.m.shift_feasible(1, toes, 0.0, -0.005));
    EXPECT_FALSE(f.m.shift_feasible(1, toes, 0.0, 0.005));
    EXPECT_GT(f.m.diag_snapshot()["ext_rejects"].get<int>(), 0);
}

TEST(PoseSequencer, TheLiftingLegFormHoldsTheLegAboutToLiftToTheMarginToo) {
    Fix f({{"ext_margin_m", ogma::ParamValue{0.015}}}); f.run(0, {1, 1, 1, 1});
    const double hd[4][2] = {{-0.7071, 0.7071}, {0.7071, 0.7071}, {-0.7071, -0.7071}, {0.7071, -0.7071}};
    auto place = [&](int i, double r) { const double hx = (hd[i][0] > 0 ? 1 : -1) * 0.0383, hz = (hd[i][1] > 0 ? 1 : -1) * 0.033;
                                        return Vec3f(float(hx + hd[i][0] * r), -0.085f, float(hz + hd[i][1] * r)); };
    std::array<Vec3f, 4> toes{place(0, 0.08), place(1, 0.08), place(2, 0.08), place(3, 0.08)};
    double r2 = 0.08; while (r2 < 0.14 && f.m.ext_slack(2, place(2, r2)) > 0.019) r2 += 0.001;
    toes[2] = place(2, r2);
    EXPECT_TRUE(f.m.shift_feasible(2, toes, 0.0, 0.02)) << "first form: the lifting toe is exempt";
    f.m.on_param_change("ext_lifting", ogma::ParamValue{1.0});
    EXPECT_FALSE(f.m.shift_feasible(2, toes, 0.0, 0.02)) << "held to the full margin, its own forward shift stretches it past";
    EXPECT_TRUE(f.m.shift_feasible(2, toes, 0.0, -0.01)) << "a backward shift bends it";
    f.m.on_param_change("ext_lifting", ogma::ParamValue{0.3});   // the floor 4.5 mm: a short forward shift is allowed, a long one is not
    EXPECT_TRUE(f.m.shift_feasible(2, toes, 0.0, 0.005));
    EXPECT_FALSE(f.m.shift_feasible(2, toes, 0.0, 0.03));
    EXPECT_GT(f.m.diag_snapshot()["ext_lift_rejects"].get<int>(), 0);
    f.m.on_param_change("ext_margin_m", ogma::ParamValue{0.0});
    EXPECT_TRUE(f.m.shift_feasible(2, toes, 0.0, 0.02)) << "with the margin off the switch does nothing";
}

TEST(PoseSequencer, TheOverdueLegCrawlsOnFlatGround_EveryLegStepsAndNoDecisionIsEmpty) {
    // the pose search with the overdue protection and the urgent jump (at a slack threshold that fires on flat ground): it still
    // crawls with every leg stepping, never runs out of a step, and publishes its counters and the skip reasons
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}},
           {"plan", ogma::ParamValue{1.0}}, {"overdue", ogma::ParamValue{1.0}}, {"ext_urgent_m", ogma::ParamValue{0.035}}});
    std::vector<float> contact = {1, 1, 1, 1};
    for (uint64_t t = 0; t < 2500; ++t) {
        const int leg = f.m.active_leg();
        for (int i = 0; i < 4; ++i) contact[size_t(i)] = 1.0f;
        if (leg >= 0 && (f.m.phase() == PoseSequencer::Phase::Lift || f.m.phase() == PoseSequencer::Phase::Swing)) contact[size_t(leg)] = 0.0f;
        if (leg >= 0 && f.m.phase() == PoseSequencer::Phase::Lower) { auto d = f.m.diag_snapshot(); contact[size_t(leg)] = d["toe_tgt"][leg][1].get<double>() <= -0.09 ? 1.0f : 0.0f; }
        f.run(t, contact);
    }
    int lo = 1 << 30, hi = 0; for (int i = 0; i < 4; ++i) { EXPECT_GE(f.m.steps(i), 1) << "leg " << i << " never stepped"; lo = std::min(lo, f.m.steps(i)); hi = std::max(hi, f.m.steps(i)); }
    EXPECT_LE(hi, 2 * lo + 2) << "a jump does not starve the wave: one per turn";
    auto d = f.m.diag_snapshot();
    EXPECT_EQ(d["plan_none"].get<int>(), 0); EXPECT_EQ(d["deadlock_ticks"].get<int>(), 0);
    ASSERT_TRUE(d.contains("overdue")); ASSERT_EQ(d["overdue"].size(), 4u); ASSERT_EQ(d["skip_nomargin"].size(), 4u);
    EXPECT_GT(d["overdue"][2].get<int>() + d["overdue"][3].get<int>(), 0) << "at 3.5 cm the urgent rule is consulted on flat ground";
}

TEST(PoseSequencer, TheBlockerMirrorsTheShiftTest_ARefusalExactlyWhereTheTestRefuses) {
    Fix f; f.run(0, {1, 1, 1, 1});
    const double hd[4][2] = {{-0.7071, 0.7071}, {0.7071, 0.7071}, {-0.7071, -0.7071}, {0.7071, -0.7071}};
    auto place = [&](int i, double r, double turn) { const double hx = (hd[i][0] > 0 ? 1 : -1) * 0.0383, hz = (hd[i][1] > 0 ? 1 : -1) * 0.033;
                                                     const double c = std::cos(turn), s = std::sin(turn), dx = hd[i][0] * c - hd[i][1] * s, dz = hd[i][0] * s + hd[i][1] * c;
                                                     return Vec3f(float(hx + dx * r), -0.085f, float(hz + dz * r)); };
    int n = 0, refused = 0; std::array<int, 7> by{};
    for (double margin : {0.0, 0.02}) {
        f.m.on_param_change("ext_margin_m", ogma::ParamValue{margin});
        for (int k = 0; k < 6; ++k) {   // stances from compact to stretched, some toes turned toward their sector's edge
            std::array<Vec3f, 4> toes{place(0, 0.065 + 0.012 * k, 0.3), place(1, 0.09, -0.2 * k / 5.0), place(2, 0.07 + 0.012 * k, 0.0), place(3, 0.13 - 0.01 * k, 0.6)};
            for (int lifting = 0; lifting < 4; ++lifting)
                for (int ix = -3; ix <= 3; ++ix) for (int iz = -3; iz <= 4; ++iz) {
                    int toe; const int r = f.m.shift_refusal(lifting, toes, ix * 0.01, iz * 0.01, toe);
                    EXPECT_EQ(r == 0, f.m.shift_feasible(lifting, toes, ix * 0.01, iz * 0.01)) << "stance " << k << " lifting " << lifting << " shift " << ix << "," << iz;
                    if (r != 0) { EXPECT_GE(toe, 0); EXPECT_LE(toe, 3); ++refused; ++by[size_t(r)]; }
                    ++n;
                }
        }
    }
    EXPECT_GT(refused, 0); EXPECT_LT(refused, n);
    EXPECT_GT(by[1] + by[2], 0) << "reach refusals exercised"; EXPECT_GT(by[4], 0) << "sector refusals exercised"; EXPECT_GT(by[5], 0) << "extension refusals exercised";
}

TEST(PoseSequencer, NextLegLiftableFirst_TheSearchStrandsTheNextLegNoMoreOftenAndStillCrawls) {
    auto crawl = [](double on) {
        auto f = std::make_unique<Fix>(ogma::ParamMap{{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}},
                                                      {"belly_topic", ogma::ParamValue{std::string("")}}, {"plan", ogma::ParamValue{1.0}}, {"plan_next_first", ogma::ParamValue{on}}});
        std::vector<float> contact = {1, 1, 1, 1};
        for (uint64_t t = 0; t < 2500; ++t) {
            const int leg = f->m.active_leg();
            for (int i = 0; i < 4; ++i) contact[size_t(i)] = 1.0f;
            if (leg >= 0 && (f->m.phase() == PoseSequencer::Phase::Lift || f->m.phase() == PoseSequencer::Phase::Swing)) contact[size_t(leg)] = 0.0f;
            if (leg >= 0 && f->m.phase() == PoseSequencer::Phase::Lower) { auto d = f->m.diag_snapshot(); contact[size_t(leg)] = d["toe_tgt"][leg][1].get<double>() <= -0.09 ? 1.0f : 0.0f; }
            f->run(t, contact);
        }
        return f;
    };
    auto off = crawl(0.0), on = crawl(1.0);
    for (int i = 0; i < 4; ++i) EXPECT_GE(on->m.steps(i), 1) << "leg " << i << " never stepped";
    auto d0 = off->m.diag_snapshot(), d1 = on->m.diag_snapshot();
    EXPECT_EQ(d1["plan_none"].get<int>(), 0); EXPECT_EQ(d1["deadlock_ticks"].get<int>(), 0);
    const int lift_on = d1["next_lift"][0].get<int>(), strand_on = d1["next_lift"][1].get<int>(), strand_off = d0["next_lift"][1].get<int>();
    EXPECT_GT(lift_on, 0);
    EXPECT_LE(strand_on, strand_off) << "with the lever the search strands the next leg no more often";
}

TEST(PoseSequencer, ThePlanDriftChecksEveryPredictionAndItsCountsAddUp) {
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}, {"plan", ogma::ParamValue{1.0}}});
    std::vector<float> contact = {1, 1, 1, 1};
    for (uint64_t t = 0; t < 2500; ++t) {
        const int leg = f.m.active_leg();
        for (int i = 0; i < 4; ++i) contact[size_t(i)] = 1.0f;
        if (leg >= 0 && (f.m.phase() == PoseSequencer::Phase::Lift || f.m.phase() == PoseSequencer::Phase::Swing)) contact[size_t(leg)] = 0.0f;
        if (leg >= 0 && f.m.phase() == PoseSequencer::Phase::Lower) { auto d = f.m.diag_snapshot(); contact[size_t(leg)] = d["toe_tgt"][leg][1].get<double>() <= -0.09 ? 1.0f : 0.0f; }
        f.run(t, contact);
    }
    auto d = f.m.diag_snapshot(); auto dr = d["drift"]; auto dc = d["drift_cause"];
    EXPECT_GT(dr[0].get<int>(), 3) << "the predictions are checked";
    EXPECT_EQ(dr[1].get<int>() + dr[3].get<int>(), dr[0].get<int>());
    EXPECT_LE(dr[2].get<int>(), dr[1].get<int>());
    EXPECT_LE(dc[3].get<int>(), dr[2].get<int>());
    EXPECT_GE(d["drift_mag_all"][0].get<double>(), 0.0);
}

TEST(PoseSequencer, TheRelocationTagClosesAtEveryTouch_TheRulesSumToTheLandingsOffset) {
    // an obstacle-free flat crawl with the edge move and a reach that clamps: every landing's offset from where its step set out
    // is the sum of the tagged moves, and the counts add up
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}, {"plan", ogma::ParamValue{1.0}}});
    std::vector<float> contact = {1, 1, 1, 1};
    for (uint64_t t = 0; t < 2500; ++t) {
        const int leg = f.m.active_leg();
        for (int i = 0; i < 4; ++i) contact[size_t(i)] = 1.0f;
        if (leg >= 0 && (f.m.phase() == PoseSequencer::Phase::Lift || f.m.phase() == PoseSequencer::Phase::Swing)) contact[size_t(leg)] = 0.0f;
        if (leg >= 0 && f.m.phase() == PoseSequencer::Phase::Lower) { auto d = f.m.diag_snapshot(); contact[size_t(leg)] = d["toe_tgt"][leg][1].get<double>() <= -0.09 ? 1.0f : 0.0f; }
        f.run(t, contact);
    }
    auto d = f.m.diag_snapshot(); auto n = d["reloc_n"];
    EXPECT_GT(n[0].get<int>(), 3) << "landings are tracked";
    EXPECT_EQ(n[1].get<int>() + n[2].get<int>() + n[3].get<int>(), n[0].get<int>());
    EXPECT_GT(n[1].get<int>(), 0) << "the pose search plans steps";
    EXPECT_LT(d["reloc_resid"].get<double>(), 1e-5) << "every move of a landing is tagged";
    int top = 0; for (auto const& v : d["reloc_top"]) top += v.get<int>();
    EXPECT_LE(top, n[4].get<int>());
    double sum = 0.0; for (auto const& v : d["reloc_sum"]) sum += v.get<double>();
    EXPECT_GE(sum + 1e-9, d["reloc_off"].get<double>()) << "the offsets are no larger than the moves that made them";
    // the drift instrument's landed-toe offset is the same quantity, so they agree on planned landings
    EXPECT_GE(d["drift_mag_all"][0].get<double>(), 0.0);
}

TEST(PoseSequencer, ThePrunesOffSwitchesTakeTheirRuleOut_TheObstacleReflexAndTheSettle) {
    // the obstacle reflex off: a toe stopped in the air is not lowered where it is (the same wall as the reflex's own test)
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}, {"obstacle_ticks", ogma::ParamValue{int64_t{0}}}});
    std::vector<float> contact = {1, 1, 1, 1}; int swings = 0;
    for (uint64_t t = 0; t < 3000; ++t) {
        const int leg = f.m.active_leg(); const auto ph = f.m.phase();
        for (int i = 0; i < 4; ++i) contact[size_t(i)] = 1.0f;
        if (leg >= 0 && (ph == PoseSequencer::Phase::Lift || ph == PoseSequencer::Phase::Swing)) {
            auto d = f.m.diag_snapshot(); contact[size_t(leg)] = (d["toe_tgt"][leg][1].get<double>() > -0.06) ? 1.0f : 0.0f; swings += ph == PoseSequencer::Phase::Swing;
        }
        if (leg >= 0 && ph == PoseSequencer::Phase::Lower) { auto d = f.m.diag_snapshot(); contact[size_t(leg)] = d["toe_tgt"][leg][1].get<double>() <= -0.09 ? 1.0f : 0.0f; }
        f.run(t, contact);
    }
    EXPECT_GT(swings, 0);
    EXPECT_EQ(f.m.diag_snapshot()["obstacle_hits"].get<int>(), 0) << "with obstacle_ticks 0 the reflex never fires";
    // the settle off: the crawl starts without the slide (no settle ticks beyond the ten-tick wait)
    Fix g({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}, {"settle", ogma::ParamValue{0.0}}});
    for (uint64_t t = 0; t < 200; ++t) g.run(t, {1, 1, 1, 1});
    EXPECT_LE(g.m.diag_snapshot()["settle_ticks"].get<int>(), 12);
    EXPECT_NE(g.m.phase(), PoseSequencer::Phase::Settle);
}

TEST(PoseSequencer, TheStepOutcomeIsPublishedOncePerLanding_AndTheDiagCarriesTheRoom) {
    Fix f({{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}},
           {"belly_topic", ogma::ParamValue{std::string("")}}, {"outcome_topic", ogma::ParamValue{std::string("step.outcome")}}});
    int published = 0; float last_seq = 0.0f;
    f.bus.subscribe("step.outcome", ogma::SubscriptionKind::Direct, [&](std::string_view, ogma::MessagePtr msg) {
        auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); ASSERT_EQ(int(t->values.size()), ogma::stepper::StepOutcome::kFloats);
        EXPECT_EQ(t->values[0], last_seq + 1.0f) << "one record per landing, in order"; last_seq = t->values[0]; ++published; });
    std::vector<float> contact = {1, 1, 1, 1};
    for (uint64_t t = 0; t < 2500; ++t) {
        const int leg = f.m.active_leg(); contact = {1, 1, 1, 1};
        if (leg >= 0 && (f.m.phase() == PoseSequencer::Phase::Lift || f.m.phase() == PoseSequencer::Phase::Swing)) contact[size_t(leg)] = 0.0f;
        if (leg >= 0 && f.m.phase() == PoseSequencer::Phase::Lower) { auto d = f.m.diag_snapshot(); contact[size_t(leg)] = d["toe_tgt"][leg][1].get<double>() < -0.087 ? 1.0f : 0.0f; }
        f.run(t, contact);
    }
    int total = 0; for (int i = 0; i < 4; ++i) total += f.m.steps(i);
    EXPECT_GT(total, 4); EXPECT_EQ(published, total);
    auto d = f.m.diag_snapshot();
    ASSERT_EQ(d["room"].size(), 4u); ASSERT_EQ(d["room"][0].size(), 5u);
    for (int i = 0; i < 4; ++i) { EXPECT_GT(d["room"][i][3].get<double>(), 0.0) << "hip1 inside its stops"; EXPECT_LE(d["room"][i][1].get<double>(), 1.0); }
    EXPECT_EQ(d["outcome"][0].get<double>(), double(total));
}

// ---- the hand-back from a stand (2026-11-19) ----
namespace {
// the test's ground: a toe in the air has no contact; a toe coming down touches at the floor (-0.09 in the body frame)
std::vector<float> ground_contact(Fix& f) {
    std::vector<float> c = {1, 1, 1, 1}; const int leg = f.m.active_leg(); const auto ph = f.m.phase();
    if (leg >= 0 && (ph == PoseSequencer::Phase::Lift || ph == PoseSequencer::Phase::Swing)) c[size_t(leg)] = 0.0f;
    if (leg >= 0 && (ph == PoseSequencer::Phase::Lower || ph == PoseSequencer::Phase::Return)) c[size_t(leg)] = f.m.diag_snapshot()["toe_tgt"][leg][1].get<double>() <= -0.09 ? 1.0f : 0.0f;
    return c;
}
// crawl until a toe is in the air mid-swing; returns the tick reached
uint64_t crawl_to_a_swing(Fix& f, uint64_t t0 = 0) {
    uint64_t t = t0; int in_swing = 0;
    for (; t < t0 + 3000; ++t) { f.run(t, ground_contact(f)); if (f.m.phase() == PoseSequencer::Phase::Swing && ++in_swing >= 5) break; }
    return t + 1;
}
ogma::ParamMap hb_params(double stand) {
    return {{"engage", ogma::ParamValue{1.0}}, {"engage_ramp_ticks", ogma::ParamValue{int64_t{10}}}, {"engage_min_tick", ogma::ParamValue{int64_t{0}}}, {"belly_topic", ogma::ParamValue{std::string("")}}, {"handback_stand", ogma::ParamValue{stand}}};
}
}

TEST(PoseSequencer, HandbackStandOff_TheLegsGoBackFromWhereverTheCrawlIs) {
    Fix f(hb_params(0.0));
    uint64_t t = crawl_to_a_swing(f); ASSERT_EQ(f.m.phase(), PoseSequencer::Phase::Swing);
    f.m.on_param_change("engage", ogma::ParamValue{0.0});
    f.run(t, ground_contact(f));
    EXPECT_EQ(f.m.phase(), PoseSequencer::Phase::Idle) << "off: idle on the next tick, a toe still in the air";
    EXPECT_LT(f.m.engage_level(), 1.0f);
    EXPECT_FALSE(f.m.diag_snapshot().contains("handback")) << "the default diag is unchanged";
}

TEST(PoseSequencer, HandbackStandOne_TheToeComesDownAndTheToesGoToTheirPlacesBeforeTheBlendRampsOut) {
    Fix f(hb_params(1.0));
    uint64_t t = crawl_to_a_swing(f); ASSERT_EQ(f.m.phase(), PoseSequencer::Phase::Swing);
    const int leg = f.m.active_leg(); ASSERT_GE(leg, 0);
    f.m.on_param_change("engage", ogma::ParamValue{0.0});
    int ret_ticks = 0; bool lowered_seen = false;
    for (int k = 0; k < 600 && (k == 0 || f.m.phase() != PoseSequencer::Phase::Idle); ++k, ++t) {
        f.run(t, ground_contact(f));
        if (f.m.phase() == PoseSequencer::Phase::Return) { ++ret_ticks; EXPECT_FLOAT_EQ(f.m.engage_level(), 1.0f) << "the sequencer holds the legs until it stands"; if (f.m.active_leg() < 0) lowered_seen = true; }
    }
    ASSERT_EQ(f.m.phase(), PoseSequencer::Phase::Idle);
    EXPECT_GT(ret_ticks, 10); EXPECT_TRUE(lowered_seen);
    auto d = f.m.diag_snapshot();
    ASSERT_TRUE(d.contains("handback"));
    EXPECT_EQ(d["handback"][0].get<double>(), 1.0); EXPECT_EQ(d["handback"][2].get<double>(), 0.0) << "it arrived, no timeout"; EXPECT_EQ(d["handback"][3].get<double>(), 1.0) << "one toe was lowered";
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(d["toe_tgt"][i][0].get<double>(), d["neutral"][i][0].get<double>(), 1e-4) << "toe " << i;
        EXPECT_NEAR(d["toe_tgt"][i][2].get<double>(), d["neutral"][i][1].get<double>(), 1e-4) << "toe " << i;
        EXPECT_NEAR(d["toe_tgt"][i][1].get<double>(), d["toe_tgt"][0][1].get<double>(), 0.012) << "the four toes stand at one height on a flat floor (toe " << i << ")";
    }
    // only now the blend ramps out, over engage_ramp_ticks
    for (int k = 0; k < 12; ++k, ++t) f.run(t, {1, 1, 1, 1});
    EXPECT_FLOAT_EQ(f.m.engage_level(), 0.0f);
}

TEST(PoseSequencer, HandbackStandTwo_TheStandIsTheRhythmsOwnPose_LearnedFromItsCommand) {
    auto p = hb_params(2.0); p["engage"] = ogma::ParamValue{0.0};
    Fix f(p);
    uint64_t t = 0; for (; t < 150; ++t) f.run(t, {1, 1, 1, 1});   // the rhythm drives (the fixture's command: knees at 0.5, hips at 0)
    f.m.on_param_change("engage", ogma::ParamValue{1.0});
    t = crawl_to_a_swing(f, t); ASSERT_EQ(f.m.phase(), PoseSequencer::Phase::Swing);
    f.m.on_param_change("engage", ogma::ParamValue{0.0});
    for (int k = 0; k < 600 && (k == 0 || f.m.phase() != PoseSequencer::Phase::Idle); ++k, ++t) f.run(t, ground_contact(f));
    ASSERT_EQ(f.m.phase(), PoseSequencer::Phase::Idle);
    auto d = f.m.diag_snapshot();
    EXPECT_EQ(d["handback"][2].get<double>(), 0.0) << "it arrived";
    EXPECT_GE(d["handback"][5].get<double>(), 100.0) << "the pose was learned while the rhythm drove";
    ASSERT_EQ(f.out.size(), 13u);
    for (int k = 0; k < 12; ++k) EXPECT_NEAR(f.out[size_t(1 + k)], (k % 3 == 2) ? 0.5f : 0.0f, 0.08f) << "joint " << k << ": the pose handed back is the one the rhythm commands";
}

TEST(PoseSequencer, HandbackStand_ABodyGoingOverIsHandedBackAtOnce) {
    Fix f(hb_params(1.0));
    uint64_t t = crawl_to_a_swing(f); ASSERT_EQ(f.m.phase(), PoseSequencer::Phase::Swing);
    f.m.on_param_change("engage", ogma::ParamValue{0.0});
    f.run(t++, ground_contact(f)); ASSERT_EQ(f.m.phase(), PoseSequencer::Phase::Return);
    f.run(t++, ground_contact(f), 0.03f, {0.8f, 0.6f, 0.0f});   // the body's up at 53 deg from vertical: past tilt_disengage
    EXPECT_EQ(f.m.phase(), PoseSequencer::Phase::Idle) << "the return does not hold the legs of a body going over";
}

// THE SELECTION'S AUTHORITY (2026-11-20): engage_topic is followed only with engage_follow 1.
TEST(PoseSequencer, EngageTopicInShadow_TheWantIsHeardAndNotFollowed) {
    ogma::ParamMap p = hb_params(0.0); p["engage"] = ogma::ParamValue{0.0}; p["engage_topic"] = ogma::ParamValue{std::string("select.walker")};
    Fix f(p);
    auto want = std::make_shared<ogma::ProprioToken>(); want->values.resize(1); want->values[0] = 1.0f;
    for (uint64_t t = 0; t < 200; ++t) { f.bus.begin_tick(t); f.bus.publish("select.walker", want); f.bus.end_tick(); f.run(t, ground_contact(f)); }
    EXPECT_EQ(f.m.phase(), PoseSequencer::Phase::Idle); EXPECT_EQ(f.m.engage_level(), 0.0f);
    const auto d = f.m.diag_snapshot(); ASSERT_TRUE(d.contains("follow"));
    EXPECT_EQ(d["follow"][1].get<double>(), 1.0) << "heard"; EXPECT_EQ(d["follow"][2].get<double>(), 0.0) << "not followed";
}
TEST(PoseSequencer, EngageFollow_TheSelectionTakesAndGivesBackTheLegs) {
    ogma::ParamMap p = hb_params(0.0); p["engage"] = ogma::ParamValue{0.0}; p["engage_topic"] = ogma::ParamValue{std::string("select.walker")}; p["engage_follow"] = ogma::ParamValue{1.0};
    Fix f(p);
    auto want = std::make_shared<ogma::ProprioToken>(); want->values.resize(1); want->values[0] = 1.0f;
    uint64_t t = 0;
    for (; t < 200; ++t) { f.bus.begin_tick(t); f.bus.publish("select.walker", want); f.bus.end_tick(); f.run(t, ground_contact(f)); }
    EXPECT_NE(f.m.phase(), PoseSequencer::Phase::Idle) << "the sequencer has the legs"; EXPECT_GT(f.m.engage_level(), 0.9f);
    want = std::make_shared<ogma::ProprioToken>(); want->values.resize(1); want->values[0] = 0.0f;
    for (; t < 400; ++t) { f.bus.begin_tick(t); f.bus.publish("select.walker", want); f.bus.end_tick(); f.run(t, ground_contact(f)); }
    EXPECT_EQ(f.m.phase(), PoseSequencer::Phase::Idle); EXPECT_EQ(f.m.engage_level(), 0.0f);
    EXPECT_EQ(f.m.diag_snapshot()["follow"][2].get<double>(), 2.0) << "two sets: on, off";
}
