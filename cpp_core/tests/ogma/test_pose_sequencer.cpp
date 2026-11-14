// PoseSequencer: the shared FK on a published leg model, the IK, and the crawl's state machine on flat ground.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/PoseSequencer.hpp"
namespace {
using ogma::PoseSequencer; using ogma::body::Vec3f;
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
Vec3f rot(const Vec3f& v, const Vec3f& k, double a) {   // Rodrigues
    const double c = std::cos(a), s = std::sin(a), kd = k.x * v.x + k.y * v.y + k.z * v.z;
    const Vec3f kxv(k.y * v.z - k.z * v.y, k.z * v.x - k.x * v.z, k.x * v.y - k.y * v.x);
    return Vec3f(float(v.x * c + kxv.x * s + k.x * kd * (1 - c)), float(v.y * c + kxv.y * s + k.y * kd * (1 - c)), float(v.z * c + kxv.z * s + k.z * kd * (1 - c)));
}
// the body's _build_leg, in the rest frame (the chassis origin at 0): headings at 45 deg, L1 / L2 / L3, the 7 mm coxa drop,
// the lower leg dropped by 1.396 rad about the lateral, picking the lower of the two
std::vector<float> leg_model() {
    const double L1 = 0.03726, L2 = 0.0536, L3 = 0.0755, drop = 0.007, hx = 0.0766 * 0.5, hz = 0.066 * 0.5;
    const double hd[4][2] = {{-0.7071, 0.7071}, {0.7071, 0.7071}, {-0.7071, -0.7071}, {0.7071, -0.7071}};
    std::vector<float> v;
    for (int i = 0; i < 4; ++i) {
        const Vec3f heading(float(hd[i][0]), 0.0f, float(hd[i][1]));
        const Vec3f lateral(-heading.z, 0.0f, heading.x);   // UP x heading = (uy*hz - 0, 0, 0 - uy*hx) = (hz, 0, -hx)... Godot's cross(UP, h) = (1*h.z - 0*0, 0*h.x - 0*h.z, 0*0 - 1*h.x) = (h.z, 0, -h.x)
        const Vec3f lat(heading.z, 0.0f, -heading.x);
        (void)lateral;
        const Vec3f hip1(float((hd[i][0] > 0 ? 1 : -1) * hx), 0.0f, float((hd[i][1] > 0 ? 1 : -1) * hz));
        const double ch = std::sqrt(L1 * L1 - drop * drop);
        Vec3f cdir(float(heading.x * ch), float(-drop), float(heading.z * ch)); const double cn = std::sqrt(double(cdir.x) * cdir.x + double(cdir.y) * cdir.y + double(cdir.z) * cdir.z);
        cdir = Vec3f(float(cdir.x / cn), float(cdir.y / cn), float(cdir.z / cn));
        const Vec3f hip2(float(hip1.x + cdir.x * L1), float(hip1.y + cdir.y * L1), float(hip1.z + cdir.z * L1));
        const Vec3f knee(float(hip2.x + heading.x * L2), hip2.y, float(hip2.z + heading.z * L2));
        const Vec3f down = rot(heading, lat, -1.396), up = rot(heading, lat, 1.396);
        const Vec3f ldir = down.y < up.y ? down : up;
        const Vec3f coxa_o(0.5f * (hip1.x + hip2.x), 0.5f * (hip1.y + hip2.y), 0.5f * (hip1.z + hip2.z));
        const Vec3f upper_o(float(hip2.x + heading.x * L2 * 0.5), hip2.y, float(hip2.z + heading.z * L2 * 0.5));
        const Vec3f lower_o(float(knee.x + ldir.x * L3 * 0.5), float(knee.y + ldir.y * L3 * 0.5), float(knee.z + ldir.z * L3 * 0.5));
        const Vec3f toe_off(float(ldir.x * L3 * 0.5), float(ldir.y * L3 * 0.5), float(ldir.z * L3 * 0.5));
        for (auto const& p : {hip1, hip2, knee, coxa_o, upper_o, lower_o, lat, lat, toe_off}) { v.push_back(p.x); v.push_back(p.y); v.push_back(p.z); }
    }
    for (float x : {0.0f, 1.4f, 0.0f, 1.4f, -1.6f, 3.2f, 0.85f, 1.0f}) v.push_back(x);
    return v;
}
std::vector<float> flat_patch(float h) {
    const int N = 16; std::vector<float> v(4 + 2 * N * N, h); v[0] = N; v[1] = 0.03f; v[2] = 0; v[3] = 0;
    for (int k = 0; k < N * N; ++k) v[size_t(4 + N * N + k)] = 40000.0f; return v;
}
std::vector<float> support(float m) { std::vector<float> v(26, 0.0f); for (int i = 0; i < 4; ++i) { v[size_t(7 + 5 * i)] = 1; v[size_t(8 + 5 * i)] = 1; v[size_t(9 + 5 * i)] = 0.25f; v[size_t(10 + 5 * i)] = m; } return v; }
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
