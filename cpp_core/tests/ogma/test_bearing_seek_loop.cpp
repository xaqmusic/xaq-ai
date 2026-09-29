// =============================================================================
// test_bearing_seek_loop.cpp
//   ogma::BearingSeekLoop — seeking a thing seen only at stops (the duck's things phase, T2).
//
//   1. SilentWithNothingSeen — no bearing, no target: zero bearing, value 0.
//   2. SeenFixesAPositionAndUnseenHomesToIt — a thing 1 m ahead seen from pose (0,0,0) is remembered at
//      world (1,0); after the body walks 0.4 m and turns left by 90 deg the remembered thing is to its
//      RIGHT at 0.6 m, and the loop says so without seeing it.
//   3. ArrivalDropsTheTarget — within arrive_m the target is dropped and counted; value 0, bearing 0.
//   4. ForgettingDropsTheTarget — unseen for long enough the confidence decays below the floor.
//   6. AWalkingBearingOnlyRefinesAHeldTarget — a bearing flagged as walking never sets a target; with
//      walk_refix_m it moves a held target by up to that much (the approach by sight).
//   5. AnOpenNeedRenewsTheTarget — after an arrival, a need token [need, x, y] re-arms the target there with
//      confidence = need, but not on the arrival's own tick (that tick's zero IS the arrival), not within
//      1.5 x arrive_m, and not when the need is under renew_min.
// =============================================================================
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/BearingSeekLoop.hpp"

namespace {
struct Rig {
    ogma::InProcessBus bus;
    ogma::BearingSeekLoop m;
    uint64_t t = 0;
    explicit Rig(ogma::ParamMap p = {}) { m.set_id("seek"); m.on_setup(&bus, p); }
    void renew(float need, double x, double y) {
        auto n = std::make_shared<ogma::ProprioToken>();
        n->values = Eigen::VectorXf(3); n->values << need, float(x), float(y);
        bus.publish("reality.cognitive.outcome_need", n);
    }
    // THE CHASE: a mover sighting on percept.mover_bearing this tick (the token must carry the tick, as CloudMap's does)
    float mvx = 0.0f, mvy = 0.0f, mprox = 0.0f;
    double mseq = -1.0;   // the token's recompute tick (a sixth value); -1 = the old five-value token
    void mover(float vx, float vy, float prox, double seq = -1.0) { mvx = vx; mvy = vy; mprox = prox; mseq = seq; }
    void step(double x, double y, double yaw, float vx, float vy, float prox, bool walking = false) {
        bus.begin_tick(t);
        if (mprox > 0.0f) {
            auto mv = std::make_shared<ogma::ProprioToken>();
            mv->tick_id = t;
            if (mseq >= 0.0) { mv->values = Eigen::VectorXf(6); mv->values << mvx, mvy, mprox, 10.0f, 300.0f, float(mseq); }
            else { mv->values = Eigen::VectorXf(5); mv->values << mvx, mvy, mprox, 10.0f, 300.0f; }
            bus.publish("percept.mover_bearing", mv);
            mprox = 0.0f;
        }
        auto pose = std::make_shared<ogma::ProprioToken>();
        pose->values = Eigen::VectorXf(3); pose->values << float(x), float(y), float(yaw);
        bus.publish("reality.proprio.odom", pose);
        auto b = std::make_shared<ogma::ProprioToken>();
        b->values = Eigen::VectorXf(4); b->values << vx, vy, prox, (walking ? 1.0f : 0.0f);
        bus.publish("percept.thing_bearing", b);
        m.tick(t);
        bus.end_tick();
        ++t;
    }
};
}  // namespace

TEST(BearingSeekLoop, SilentWithNothingSeen) {
    Rig r;
    r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r.m.have_target());
    EXPECT_EQ(r.m.value(), 0.0f);
    auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("percept.seek_bearing"));
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->values[0], 0.0f); EXPECT_EQ(out->values[1], 0.0f); EXPECT_EQ(out->values[2], 0.0f);
}

TEST(BearingSeekLoop, SeenFixesAPositionAndUnseenHomesToIt) {
    ogma::ParamMap p; p["proximity_range"] = 2.5;
    Rig r(p);
    // dead ahead at 1 m: proximity 1 - 1/2.5 = 0.6
    r.step(0, 0, 0, 0.0f, 1.0f, 0.6f);
    ASSERT_TRUE(r.m.have_target());
    EXPECT_NEAR(r.m.target_x(), 1.0, 1e-6);
    EXPECT_NEAR(r.m.target_y(), 0.0, 1e-6);
    EXPECT_EQ(r.m.value(), 1.0f);
    // the cloud closes, the body walks 0.4 m forward and turns 90 deg left; nothing is seen
    r.step(0.4, 0, M_PI / 2, 0.0f, 0.0f, 0.0f);
    ASSERT_TRUE(r.m.have_target());
    EXPECT_NEAR(r.m.range_left(), 0.6, 1e-6);
    EXPECT_NEAR(r.m.last_cx(), 1.0, 1e-6) << "the thing is now to the body's right";
    EXPECT_NEAR(r.m.last_cy(), 0.0, 1e-6);
    EXPECT_GT(r.m.value(), 0.99f) << "confidence decays slowly (forget_ticks 3000)";
    auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("percept.seek_bearing"));
    EXPECT_NEAR(out->values[0], 1.0, 1e-6);
    auto rg = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("reality.cognitive.seek_range"));
    ASSERT_NE(rg, nullptr);
    EXPECT_NEAR(rg->values[0], 0.6, 1e-5);
}

TEST(BearingSeekLoop, ArrivalDropsTheTarget) {
    Rig r;
    r.step(0, 0, 0, 0.0f, 1.0f, 0.6f);          // 1 m ahead
    r.step(0.8, 0, 0, 0.0f, 0.0f, 0.0f);        // 0.2 m left: under arrive_m 0.25
    EXPECT_FALSE(r.m.have_target());
    EXPECT_EQ(r.m.arrivals(), 1);
    EXPECT_EQ(r.m.value(), 0.0f);
    EXPECT_EQ(r.m.last_cx(), 0.0f); EXPECT_EQ(r.m.last_cy(), 0.0f);
}

TEST(BearingSeekLoop, ForgettingDropsTheTarget) {
    ogma::ParamMap p; p["forget_ticks"] = 10.0; p["floor"] = 0.5;
    Rig r(p);
    r.step(0, 0, 0, 0.0f, 1.0f, 0.6f);
    for (int i = 0; i < 6; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);   // 0.9^6 = 0.53 > 0.5
    EXPECT_TRUE(r.m.have_target());
    r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);                                 // 0.9^7 = 0.48 < 0.5
    EXPECT_FALSE(r.m.have_target());
    EXPECT_EQ(r.m.forgets(), 1);
}

TEST(BearingSeekLoop, AnOpenNeedRenewsTheTarget) {
    ogma::ParamMap p; p["renew_topic"] = std::string("reality.cognitive.outcome_need"); p["arrive_m"] = 0.25;
    Rig r(p);
    auto value = [&]() { return std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("reality.cognitive.seek_value"))->values[0]; };
    for (int i = 0; i < 5; ++i) r.step(0, 0, 0, 0.0f, 1.0f, 0.6f);      // a thing 1 m ahead
    r.step(0.9, 0, 0, 0, 0, 0);                                          // walked to 0.1 m: the arrival
    EXPECT_FLOAT_EQ(value(), 0.0f);
    EXPECT_EQ(r.m.arrivals(), 1);
    // a need at the thing published BEFORE the next tick: the arrival tick already passed, so it renews
    r.renew(1.0f, 1.0, 0.0);
    r.step(0.5, 0, 0, 0, 0, 0);                                          // backed off to 0.5 m
    EXPECT_NEAR(value(), 1.0f, 0.01f) << "renewed with confidence = need (one tick of forgetting already applied)";
    EXPECT_EQ(r.m.renewals(), 1);
    auto b = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("percept.seek_bearing"));
    EXPECT_GT(b->values[1], 0.9f) << "the thing is ahead again";
    r.step(0.9, 0, 0, 0, 0, 0);                                          // walked back: a second arrival
    EXPECT_EQ(r.m.arrivals(), 2);
    EXPECT_FLOAT_EQ(value(), 0.0f) << "the arrival reads 0 for its tick even with the need still open";
    r.step(0.9, 0, 0, 0, 0, 0);
    EXPECT_EQ(r.m.renewals(), 1) << "no renewal within 1.5 x arrive_m of the thing";
    r.renew(0.1f, 1.0, 0.0);
    r.step(0.5, 0, 0, 0, 0, 0);
    EXPECT_EQ(r.m.renewals(), 1) << "a need under renew_min does not renew";
    EXPECT_FLOAT_EQ(value(), 0.0f);
}

TEST(BearingSeekLoop, AWalkingBearingOnlyRefinesAHeldTarget) {
    ogma::ParamMap p; p["walk_refix_m"] = 0.5;
    Rig r(p);
    auto value = [&]() { return std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("reality.cognitive.seek_value"))->values[0]; };
    for (int i = 0; i < 5; ++i) r.step(0, 0, 0, 0.0f, 1.0f, 0.6f, true);   // a thing 1 m ahead, seen while WALKING
    EXPECT_FLOAT_EQ(value(), 0.0f) << "a walking bearing sets no target";
    EXPECT_EQ(r.m.refixes(), 0);
    for (int i = 0; i < 5; ++i) r.step(0, 0, 0, 0.0f, 1.0f, 0.6f, false);  // the same thing from a stop: the target
    EXPECT_FLOAT_EQ(value(), 1.0f);
    // walking toward it, the thing is seen 0.2 m to the right of the remembered spot: the target follows
    r.step(0.5, 0, 0, 0.37f, 1.0f, 0.79f, true);                            // from (0.5,0): the thing at (1.0,-0.2) is ~0.54 m off, 0.37 right / 1 fwd
    EXPECT_EQ(r.m.refixes(), 1);
    auto b = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("percept.seek_bearing"));
    EXPECT_GT(b->values[0], 0.1f) << "the bearing now points a little right, where the thing is";
    // a walking sighting 2 m from the held target is another thing: ignored, the target stays
    r.step(0.5, 0, 0, 0.0f, 1.0f, 0.2f, true);                              // something 2 m ahead
    EXPECT_EQ(r.m.refixes(), 1);
}

// =============================================================================
// THE CHASE (the chase phase, stage 1, 2026-09-27): the moving fix.
// =============================================================================
namespace {
ogma::ParamMap chase_params() {
    ogma::ParamMap p;
    p["mover_topic"] = std::string("percept.mover_bearing");
    p["proximity_range"] = 2.5;
    return p;
}
// a mover straight ahead at range r, seen from the origin: bearing (0, 1), proximity 1 - r / 2.5
float prox_of(double r) { return float(1.0 - r / 2.5); }
}  // namespace

TEST(BearingSeekLoop, AMoverThatFollowsItsOwnPredictionIsChased) {
    Rig r(chase_params());
    // sightings every 4 ticks of a thing walking away along +x at 0.2 m/s from 1.0 m; the body stands at the origin
    for (int k = 0; k < 10; ++k) {
        const double range = 1.0 + 0.2 * (4.0 * k / 50.0);
        r.mover(0.0f, 1.0f, prox_of(range));
        r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        if (k < 6) EXPECT_FALSE(r.m.chasing()) << "not before chase_confirm_ticks (25) have passed since the first sighting, k=" << k;
    }
    EXPECT_TRUE(r.m.chasing()) << "ten sightings over 36 ticks, each where the last predicted";
    EXPECT_EQ(r.m.chases(), 1);
    EXPECT_NEAR(r.m.chase_vx(), 0.2, 0.05) << "the velocity follows the sightings";
    EXPECT_NEAR(r.m.chase_vy(), 0.0, 0.02);
    EXPECT_TRUE(r.m.have_target());
    EXPECT_EQ(r.m.value(), 1.0f) << "a chase is a need of 1";
    // the target is ahead of the last sighting by the lead
    EXPECT_GT(r.m.target_x(), 1.0 + 0.2 * 36.0 / 50.0);
    auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("percept.seek_bearing"));
    ASSERT_NE(out, nullptr);
    EXPECT_NEAR(out->values[1], 1.0, 1e-3) << "straight ahead";
    // no arrival while chasing: the body walks onto the target and the target stays
    r.mover(0.0f, 1.0f, prox_of(1.2));
    r.step(1.15, 0, 0, 0.0f, 0.0f, 0.0f);   // the body is 5 cm from the fix... no: the fix is at 1.15 + 1.2
    EXPECT_TRUE(r.m.chasing());
    EXPECT_EQ(r.m.arrivals(), 0);
}

TEST(BearingSeekLoop, AChaseEndsWhereTheThingWasLastSeenAndIsRemembered) {
    Rig r(chase_params());
    for (int k = 0; k < 10; ++k) {
        r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0)));
        r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    ASSERT_TRUE(r.m.chasing());
    const double last_x = 1.0 + 0.2 * 36.0 / 50.0;
    for (int i = 0; i < 60; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);   // nothing seen for 60 ticks > chase_forget_ticks 50
    EXPECT_FALSE(r.m.chasing());
    EXPECT_TRUE(r.m.have_target()) << "the last predicted position stays as a remembered target";
    EXPECT_NEAR(r.m.target_x(), last_x, 0.05);
    EXPECT_GT(r.m.value(), 0.9f);
    EXPECT_EQ(r.m.chases(), 1);
}

TEST(BearingSeekLoop, AThingNewlyInViewIsNotChasedAndAJumpIsNotAConfirmation) {
    Rig r(chase_params());
    // two sightings 4 ticks apart, then nothing: a static thing that aged out before chase_confirm_ticks
    r.mover(0.0f, 1.0f, prox_of(1.0)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    r.mover(0.0f, 1.0f, prox_of(1.0)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_EQ(r.m.chase_n(), 2);
    for (int i = 0; i < 60; ++i) { r.step(0, 0, 0, 0.0f, 0.0f, 0.0f); EXPECT_FALSE(r.m.chasing()); }
    EXPECT_EQ(r.m.chase_n(), 0) << "the candidate is forgotten";
    EXPECT_FALSE(r.m.have_target()) << "an unconfirmed candidate leaves no target";
    // a jump: 1.0 m ahead, then 2.0 m ahead four ticks later (12.5 m/s) -- another cluster, a fresh candidate
    r.mover(0.0f, 1.0f, prox_of(1.0)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    r.mover(0.0f, 1.0f, prox_of(2.0)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_EQ(r.m.chase_n(), 1) << "the far sighting replaced the candidate instead of confirming it";
    EXPECT_EQ(r.m.chases(), 0);
}

TEST(BearingSeekLoop, WithoutAMoverTopicNothingChanges) {
    Rig r;   // no mover_topic
    r.mover(0.0f, 1.0f, prox_of(1.0)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    r.mover(0.0f, 1.0f, prox_of(1.0)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r.m.chasing()); EXPECT_EQ(r.m.chase_n(), 0); EXPECT_FALSE(r.m.have_target());
}

TEST(BearingSeekLoop, WithChaseMinVAThingThatStaysPutIsNotChased) {
    ogma::ParamMap p = chase_params();
    p["chase_min_v"] = 0.1;
    Rig still(p);
    for (int k = 0; k < 10; ++k) {                       // the same spot, 1.0 m ahead, ten sightings over 36 ticks
        still.mover(0.0f, 1.0f, prox_of(1.0)); still.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) still.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    EXPECT_FALSE(still.m.chasing()) << "young, persistent, but it has not moved";
    EXPECT_EQ(still.m.chases(), 0);
    Rig moving(p);
    for (int k = 0; k < 10; ++k) {                       // the thing walking away at 0.2 m/s
        moving.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); moving.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) moving.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    EXPECT_TRUE(moving.m.chasing()) << "moved 0.14 m over 0.72 s: chased";
}

TEST(BearingSeekLoop, AChaseLostWhileMovingIsDroppedAndOneThatStoppedIsRemembered) {
    ogma::ParamMap p = chase_params();
    p["chase_stop_v"] = 0.05;
    Rig lost(p);
    for (int k = 0; k < 10; ++k) {                       // walking away at 0.2 m/s, then gone from the view
        lost.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); lost.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) lost.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    ASSERT_TRUE(lost.m.chasing());
    for (int i = 0; i < 60; ++i) lost.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(lost.m.chasing());
    EXPECT_FALSE(lost.m.have_target()) << "it left the view still moving: no place to go and look";
    EXPECT_EQ(lost.m.chases_lost(), 1); EXPECT_EQ(lost.m.chases_stopped(), 0);
    Rig stopped(p);
    for (int k = 0; k < 10; ++k) {                       // the same, then six more sightings at the same spot: it stopped
        stopped.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); stopped.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) stopped.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    const double last = 1.0 + 0.2 * 36.0 / 50.0;
    for (int k = 0; k < 14; ++k) {                       // 1.1 s still: the velocity ring (0.8 s) holds only still sightings
        stopped.mover(0.0f, 1.0f, prox_of(last)); stopped.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) stopped.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    ASSERT_TRUE(stopped.m.chasing());
    for (int i = 0; i < 60; ++i) stopped.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(stopped.m.chasing());
    EXPECT_TRUE(stopped.m.have_target()) << "it stopped: remembered where it stands";
    EXPECT_NEAR(stopped.m.target_x(), last, 0.05);
    EXPECT_EQ(stopped.m.chases_stopped(), 1);
}

TEST(BearingSeekLoop, ALostChaseSaysWhereTheThingWentForOneTick) {
    ogma::ParamMap p = chase_params();
    p["chase_stop_v"] = 0.05;
    Rig r(p);
    for (int k = 0; k < 10; ++k) {                       // walking away along +x at 0.2 m/s
        r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    ASSERT_TRUE(r.m.chasing());
    int lost_ticks = 0; double ego = 9.0, range = 0.0;
    for (int i = 0; i < 60; ++i) {
        r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        if (r.m.chase_lost_now()) { ++lost_ticks; ego = r.m.chase_lost_ego(); range = r.m.chase_lost_range(); }
    }
    EXPECT_EQ(lost_ticks, 1) << "the loss is reported on one tick";
    EXPECT_NEAR(ego, 0.0, 0.05) << "it went straight ahead";
    EXPECT_GT(range, 1.0 + 0.2 * 36.0 / 50.0) << "beyond where it was last seen, by its velocity over the forget time";
    EXPECT_LT(range, 2.0);
}

// OBJECT PERMANENCE and the pull's decay (2026-09-29).
TEST(BearingSeekLoop, ALostThingCoastsAtItsVelocityAndIsReacquiredWherePredicted) {
    ogma::ParamMap p = chase_params();
    p["chase_stop_v"] = 0.05; p["chase_permanence_ticks"] = int64_t{150};
    Rig r(p);
    for (int k = 0; k < 10; ++k) {                       // walking away along +x at 0.2 m/s
        r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    ASSERT_TRUE(r.m.chasing());
    for (int i = 0; i < 60; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);   // out of sight past chase_forget_ticks
    EXPECT_FALSE(r.m.chasing());
    EXPECT_TRUE(r.m.coasting()) << "the thing is kept moving in mind";
    EXPECT_TRUE(r.m.have_target());
    EXPECT_GT(r.m.value(), 0.4f); EXPECT_LT(r.m.value(), 1.0f) << "the need falls over the permanence";
    const double tx1 = r.m.target_x();
    for (int i = 0; i < 25; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_GT(r.m.target_x(), tx1 + 0.05) << "the predicted position moves on at 0.2 m/s";
    const double where = r.m.target_x() - 0.2 * 0.3;    // the target is the lead ahead of the prediction
    r.mover(0.0f, 1.0f, prox_of(where)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(r.m.chasing()) << "found near where predicted";
    EXPECT_FALSE(r.m.coasting());
    EXPECT_EQ(r.m.chases_reacquired(), 1);
    EXPECT_EQ(r.m.chases_lost(), 0);
    for (int i = 0; i < 60 + 150 + 5; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r.m.coasting()); EXPECT_FALSE(r.m.have_target());
    EXPECT_EQ(r.m.chases_lost(), 1);
}

TEST(BearingSeekLoop, ThePullDecaysWithEachLossAndRecovers) {
    ogma::ParamMap p = chase_params();
    p["chase_stop_v"] = 0.05; p["chase_pull_decay"] = 0.5; p["chase_pull_recover_ticks"] = 1000.0;
    Rig r(p);
    auto chase_and_lose = [&]() {
        for (int k = 0; k < 10; ++k) {
            r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
            for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        }
        for (int i = 0; i < 60; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    };
    chase_and_lose();
    EXPECT_NEAR(r.m.pull(), 0.5, 0.07) << "halved by the loss (a little recovered since)";
    chase_and_lose();
    EXPECT_LT(r.m.pull(), 0.45) << "halved again (a little recovered since)";
    float v = 0.0f;
    for (int k = 0; k < 8; ++k) {                        // eight sightings over 28 ticks: past chase_confirm_ticks
        r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        v = r.m.value();
    }
    EXPECT_TRUE(r.m.chasing()); EXPECT_LT(v, 0.5f) << "the need while chasing is the pull";
    for (int i = 0; i < 2000; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_GT(r.m.pull(), 0.99) << "recovered";
}

TEST(BearingSeekLoop, ALostMoverSeenWhereItShouldBeIsReacquiredAtOnce) {
    ogma::ParamMap p = chase_params();
    p["chase_stop_v"] = 0.05; p["chase_memory_ticks"] = int64_t{250};
    Rig r(p);
    for (int k = 0; k < 10; ++k) {                       // walking away along +x at 0.2 m/s, then out of sight
        r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    for (int i = 0; i < 60; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    ASSERT_EQ(r.m.chases_lost(), 1);
    EXPECT_FALSE(r.m.have_target()) << "the memory does not drive the walk";
    for (int i = 0; i < 100; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);   // two more seconds pass
    // it reappears where it should be: 1.0 + 0.2 x (36 + 60 + 100 + 1) / 50 s along
    const double where = 1.0 + 0.2 * (36.0 + 61.0 + 100.0) / 50.0;
    r.mover(0.0f, 1.0f, prox_of(where)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(r.m.chasing()) << "one sighting where it should be: the chase resumes at once";
    EXPECT_EQ(r.m.chases_reacquired(), 1);
    EXPECT_NEAR(r.m.chase_vx(), 0.2, 0.05) << "with the remembered velocity";
    // a sighting far from where it should be is a new candidate, not the memory
    Rig q(p);
    for (int k = 0; k < 10; ++k) {
        q.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); q.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) q.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    for (int i = 0; i < 160; ++i) q.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    q.mover(0.5f, 0.866f, prox_of(1.0)); q.step(0, 0, 0, 0.0f, 0.0f, 0.0f);   // 30 deg to the right, 1 m out
    EXPECT_FALSE(q.m.chasing()); EXPECT_EQ(q.m.chases_reacquired(), 0); EXPECT_EQ(q.m.chase_n(), 1);
}

TEST(BearingSeekLoop, TheMemoryHoldsOffAStaticTargetAndTheGazeFollowsTheLostThing) {
    ogma::ParamMap p = chase_params();
    p["chase_stop_v"] = 0.05; p["chase_memory_ticks"] = int64_t{250}; p["chase_memory_holds"] = true;
    Rig r(p);
    for (int k = 0; k < 10; ++k) {                       // the mover walks away along +x, then out of sight
        r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    for (int i = 0; i < 60; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    ASSERT_EQ(r.m.chases_lost(), 1);
    EXPECT_TRUE(r.m.memory_live());
    // a static thing to the left, in view: NOT taken while the memory lives
    r.step(0, 0, 0, -0.7f, 0.7f, prox_of(0.8));
    EXPECT_FALSE(r.m.have_target()) << "the block beside the track does not take the mover's place";
    const double g = r.m.chase_gaze_ego();
    EXPECT_TRUE(std::isfinite(g)); EXPECT_NEAR(g, 0.0, 0.1) << "the gaze goes where the thing went: straight ahead";
    for (int i = 0; i < 260; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);   // the memory expires
    EXPECT_FALSE(r.m.memory_live());
    EXPECT_FALSE(std::isfinite(r.m.chase_gaze_ego()));
    r.step(0, 0, 0, -0.7f, 0.7f, prox_of(0.8));
    EXPECT_TRUE(r.m.have_target()) << "with nothing moving in mind, the static thing is a target again";
}

// 2026-09-29: the cloud recomputes every four ticks but publishes the bearing every tick.  Repeated tokens with the same
// recompute tick are not new sightings (they dragged the velocity to zero); and a crossing whose centroid jitters by 8 cm
// between casts is still chased, because the speed is judged over the watch, not the last step.
TEST(BearingSeekLoop, RepeatedTokensAreOneSightingAndAJitteryCrossingIsChased) {
    ogma::ParamMap p = chase_params();
    p["chase_min_v"] = 0.1;
    Rig r(p);
    // a thing crossing left to right at 0.2 m/s, 0.6 m ahead, seen every 4 ticks with +-4 cm of jitter, the token
    // repeated on the three ticks between (the same recompute tick)
    for (int k = 0; k < 12; ++k) {
        const double x = -0.3 + 0.2 * (4.0 * k / 50.0) + ((k % 2) ? 0.04 : -0.04);   // right of the body = +vx
        const double n = std::hypot(x, 0.6);
        for (int i = 0; i < 4; ++i) {
            r.mover(float(x / n), float(0.6 / n), prox_of(n), double(4 * k));
            r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        }
    }
    EXPECT_TRUE(r.m.chasing()) << "a jittery crossing, chased";
    EXPECT_NEAR(r.m.chase_vy(), -0.2, 0.08) << "moving to the right (body y is left): -0.2 m/s in y";
    EXPECT_LE(r.m.chase_n(), 12) << "at most one sighting per recompute";
}

TEST(BearingSeekLoop, AWalkingSightingCloseByStartsATargetWhenNothingIsHeld) {
    ogma::ParamMap p; p["walk_take_range"] = 1.0;
    Rig r(p);
    r.step(0, 0, 0, 0.0f, 1.0f, 1.0f - 0.8f / 2.5f, true);   // a thing 0.8 m ahead, seen from a WALKING cloud
    EXPECT_TRUE(r.m.have_target()) << "taken from the walk";
    EXPECT_NEAR(r.m.target_x(), 0.8, 0.02);
    EXPECT_EQ(r.m.walk_takes(), 1);
    Rig far(p);
    far.step(0, 0, 0, 0.0f, 1.0f, 1.0f - 1.6f / 2.5f, true);   // 1.6 m: beyond the take range
    EXPECT_FALSE(far.m.have_target());
    Rig off;                                                     // walk_take_range 0: R74's rule
    off.step(0, 0, 0, 0.0f, 1.0f, 1.0f - 0.8f / 2.5f, true);
    EXPECT_FALSE(off.m.have_target());
}

TEST(BearingSeekLoop, AChaseYieldsNearTallStructure) {
    ogma::ParamMap p = chase_params();
    p["chase_stop_v"] = 0.05; p["chase_memory_ticks"] = int64_t{250};
    p["yield_topic"] = std::string("percept.target_tall"); p["chase_yield_tall"] = int64_t{1};
    Rig r(p);
    for (int k = 0; k < 10; ++k) {
        r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    ASSERT_TRUE(r.m.chasing());
    // the cloud reports tall structure around the target
    r.bus.begin_tick(r.t);
    auto y = std::make_shared<ogma::ProprioToken>(); y->values = Eigen::VectorXf(2); y->values << 4.0f, 1.2f;
    r.bus.publish("percept.target_tall", y);
    r.bus.end_tick();
    r.mover(0.0f, 1.0f, prox_of(1.2)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r.m.chasing()) << "yielded";
    EXPECT_EQ(r.m.chases_yielded(), 1);
    EXPECT_EQ(r.m.chases_lost(), 0) << "a yield is not a loss: nothing to look for, no mover in mind";
    EXPECT_FALSE(r.m.chase_lost_now());
    EXPECT_FALSE(r.m.memory_live());
    EXPECT_TRUE(r.m.yield_live()) << "the place is remembered as not-a-mover";
    EXPECT_FALSE(r.m.have_target());
    // the same sighting again, at the yielded place: dropped, no candidate, no chase (sweep 13's churn)
    for (int k = 0; k < 6; ++k) {
        r.bus.begin_tick(r.t); r.bus.publish("percept.target_tall", y); r.bus.end_tick();
        r.mover(0.0f, 1.0f, prox_of(1.2 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    EXPECT_FALSE(r.m.chasing());
    EXPECT_EQ(r.m.chases_yielded(), 1);
    EXPECT_GE(r.m.yield_drops(), 6);
    EXPECT_FALSE(r.m.have_target());
    // a sighting well off the place is a fresh candidate: it confirms and is chased (the cloud reports no tall structure there)
    auto y0 = std::make_shared<ogma::ProprioToken>(); y0->values = Eigen::VectorXf(2); y0->values << 0.0f, 1.0f;
    r.bus.begin_tick(r.t); r.bus.publish("percept.target_tall", y0); r.bus.end_tick();
    for (int k = 0; k < 10; ++k) {
        r.mover(1.0f, 0.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    EXPECT_TRUE(r.m.chasing()) << "a thing elsewhere is chased";
}

TEST(BearingSeekLoop, AYieldMayLookWithoutRemembering) {
    ogma::ParamMap p = chase_params();
    p["chase_stop_v"] = 0.05; p["chase_memory_ticks"] = int64_t{250};
    p["yield_topic"] = std::string("percept.target_tall"); p["chase_yield_tall"] = int64_t{1}; p["chase_yield_look"] = true;
    Rig r(p);
    for (int k = 0; k < 10; ++k) {
        r.mover(0.0f, 1.0f, prox_of(1.0 + 0.2 * (4.0 * k / 50.0))); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    }
    ASSERT_TRUE(r.m.chasing());
    r.bus.begin_tick(r.t);
    auto y = std::make_shared<ogma::ProprioToken>(); y->values = Eigen::VectorXf(2); y->values << 2.0f, 1.2f;
    r.bus.publish("percept.target_tall", y);
    r.bus.end_tick();
    r.mover(0.0f, 1.0f, prox_of(1.2)); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r.m.chasing());
    EXPECT_EQ(r.m.chases_yielded(), 1);
    EXPECT_TRUE(r.m.chase_lost_now()) << "the yield looks";
    EXPECT_NEAR(r.m.chase_lost_ego(), 0.0, 0.05) << "at the target's bearing: straight ahead";
    EXPECT_GT(r.m.chase_lost_range(), 1.0);
    EXPECT_FALSE(r.m.memory_live()) << "but remembers no mover";
    EXPECT_TRUE(r.m.yield_live());
}

TEST(BearingSeekLoop, AStaticTargetYieldsNearTallStructure) {
    ogma::ParamMap p = chase_params();
    p["yield_topic"] = std::string("percept.target_tall"); p["static_yield_tall"] = int64_t{1}; p["forget_ticks"] = 3000.0;
    p["renew_topic"] = std::string("reality.cognitive.outcome_need"); p["renew_min"] = 0.25; p["renew_range"] = 2.0;
    Rig r(p);
    auto tall = [&](float n) { r.bus.begin_tick(r.t); auto y = std::make_shared<ogma::ProprioToken>(); y->values = Eigen::VectorXf(2); y->values << n, 1.0f; r.bus.publish("percept.target_tall", y); r.bus.end_tick(); };
    // a thing seen at a stop, straight ahead at 1 m, nothing tall around it: taken and held
    tall(0.0f);
    for (int i = 0; i < 4; ++i) r.step(0, 0, 0, 0.0f, 1.0f, prox_of(1.0));
    ASSERT_TRUE(r.m.have_target());
    // the cloud reports tall structure around the held target: dropped
    tall(3.0f);
    r.step(0, 0, 0, 0.0f, 0.0f, 0.0f); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r.m.have_target()) << "yielded";
    EXPECT_EQ(r.m.static_yielded(), 1);
    // the same sighting again: not a thing (dropped), no target
    for (int i = 0; i < 4; ++i) r.step(0, 0, 0, 0.0f, 1.0f, prox_of(1.0));
    EXPECT_FALSE(r.m.have_target());
    EXPECT_GE(r.m.static_yield_drops(), 4);
    EXPECT_EQ(r.m.static_yielded(), 1);
    // a thing elsewhere (to the right at 1 m), nothing tall: taken
    tall(0.0f);
    for (int i = 0; i < 4; ++i) r.step(0, 0, 0, 1.0f, 0.0f, prox_of(1.0));
    EXPECT_TRUE(r.m.have_target()) << "a thing elsewhere is taken";
    // a fresh target is not judged by its predecessor's stale count on its first tick
    r.step(0, 0, 0, 0.0f, 0.0f, 0.0f); r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    tall(3.0f);
    r.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r.m.have_target()) << "an older target is judged by the count";
    EXPECT_EQ(r.m.static_yielded(), 2);
    tall(0.0f);
    for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, -1.0f, prox_of(1.0));   // behind, 1 m: a new place
    ASSERT_TRUE(r.m.have_target());
    // the count of the target just taken arrives a tick later: a target under two ticks old is not dropped by a stale count
    Rig r2(p);
    r2.bus.begin_tick(r2.t); { auto y = std::make_shared<ogma::ProprioToken>(); y->values = Eigen::VectorXf(2); y->values << 3.0f, 1.0f; r2.bus.publish("percept.target_tall", y); } r2.bus.end_tick();
    r2.step(0, 0, 0, 0.0f, 1.0f, prox_of(1.0));
    EXPECT_TRUE(r2.m.have_target()) << "first tick: the count is the predecessor's";
    r2.step(0, 0, 0, 0.0f, 1.0f, prox_of(1.0));
    EXPECT_TRUE(r2.m.have_target()) << "second tick: still not judged";
    r2.step(0, 0, 0, 0.0f, 1.0f, prox_of(1.0));
    EXPECT_FALSE(r2.m.have_target()) << "third tick: judged and dropped";
    // the renewal (an outcome need at the yielded place) does not re-arm it
    auto need = std::make_shared<ogma::ProprioToken>(); need->values = Eigen::VectorXf(3); need->values << 0.9f, 1.0f, 0.0f;   // the yielded place: 1 m ahead of the body at the origin, x forward
    r2.bus.begin_tick(r2.t); r2.bus.publish("reality.cognitive.outcome_need", need); r2.bus.end_tick();
    for (int i = 0; i < 3; ++i) r2.step(0, 0, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r2.m.have_target()) << "a renewal at the yielded place is refused";
    EXPECT_EQ(r2.m.static_yielded(), 1) << "no second yield";
}

TEST(BearingSeekLoop, AWalkThatDoesNotCloseForgetsItsTarget) {
    ogma::ParamMap p; p["progress_walk_m"] = 0.5; p["progress_m"] = 0.05; p["forget_ticks"] = 3000.0; p["arrive_m"] = 0.15;
    Rig r(p);
    // a thing 1.5 m ahead, taken at a stop
    for (int i = 0; i < 3; ++i) r.step(0, 0, 0, 0.0f, 1.0f, prox_of(1.5));
    ASSERT_TRUE(r.m.have_target());
    // the body walks SIDEWAYS (y) half a metre: the range barely changes -> forgotten
    for (int i = 1; i <= 30; ++i) r.step(0, 0.02 * i, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(r.m.have_target()) << "half a metre walked, the range not closed";
    EXPECT_EQ(r.m.progress_forgets(), 1);
    // taken again, and the body walks TOWARD it: kept all the way to the arrival
    for (int i = 0; i < 3; ++i) r.step(0, 0.6, 0, 0.0f, 1.0f, prox_of(1.5));
    ASSERT_TRUE(r.m.have_target());
    for (int i = 1; i <= 60; ++i) r.step(0.02 * i, 0.6, 0, 0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(r.m.have_target()) << "1.2 m walked toward it: still held";
    EXPECT_EQ(r.m.progress_forgets(), 1);
}
