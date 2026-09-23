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
    void step(double x, double y, double yaw, float vx, float vy, float prox, bool walking = false) {
        bus.begin_tick(t);
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
