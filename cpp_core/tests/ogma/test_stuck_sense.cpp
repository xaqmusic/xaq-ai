// StuckSense: the range beam as re-afference against the stride's efference.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/StuckSense.hpp"
namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
struct Fix {
    ogma::InProcessBus bus; ogma::StuckSense m;
    Fix(ogma::ParamMap p = {{"window_ticks", int64_t{10}}, {"ema_alpha", 0.5}, {"min_expected_m", 0.02}}) { m.set_id("stuck"); m.on_setup(&bus, p); }
    void run(uint64_t t, float range, bool valid, float vfwd, float speed = 1.0f) {
        bus.begin_tick(t);
        bus.publish("sense.range", pv({range, valid ? 1.0f : 0.0f}));
        bus.publish("reality.proprio.stride_v", pv({0.0f, vfwd}));
        bus.publish("reality.proprio.speed_cmd", pv({speed}));
        m.tick(t); bus.end_tick();
    }
    float score() const { auto s = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("reality.proprio.stuck")); return s ? s->values[0] : -1.0f; }
};
}
TEST(StuckSense, WalkingFreeTowardAWallClosesTheRangeAndScoresLow) {
    Fix f; float r = 1.2f;
    for (uint64_t t = 0; t < 40; ++t) { f.run(t, r, true, 0.2f); r -= 0.2f / 50.0f; }   // the range closes at the stride's rate
    EXPECT_LT(f.score(), 0.15f); EXPECT_TRUE(f.m.measured());
}
TEST(StuckSense, PushingOnAWallKeepsTheRangeAndScoresHigh) {
    Fix f;
    for (uint64_t t = 0; t < 40; ++t) f.run(t, 0.09f, true, 0.2f);   // stepping, range constant
    EXPECT_GT(f.score(), 0.9f);
}
TEST(StuckSense, AbstainsWithoutAnEchoAndDecays) {
    Fix f({{"window_ticks", int64_t{10}}, {"ema_alpha", 0.5}, {"decay", 0.2}, {"min_expected_m", 0.02}});
    for (uint64_t t = 0; t < 40; ++t) f.run(t, 0.09f, true, 0.2f);
    ASSERT_GT(f.score(), 0.9f);
    for (uint64_t t = 40; t < 80; ++t) f.run(t, 1.5f, false, 0.2f);  // no echo: the sense abstains
    EXPECT_FALSE(f.m.measured());
    EXPECT_LT(f.score(), 0.05f) << "the score decays while the sense abstains";
}
TEST(StuckSense, DoesNotJudgeWhileNotCommandingForward) {
    Fix f;
    for (uint64_t t = 0; t < 40; ++t) f.run(t, 0.09f, true, 0.2f, 0.0f);   // speed 0: a stop is not a stall
    EXPECT_FALSE(f.m.measured()); EXPECT_FLOAT_EQ(f.score(), 0.0f);
}
TEST(StuckSense, DoesNotJudgeStandingStill) {
    Fix f;
    for (uint64_t t = 0; t < 40; ++t) f.run(t, 0.5f, true, 0.0f);   // no claimed walking: nothing to compare
    EXPECT_FALSE(f.m.measured()); EXPECT_FLOAT_EQ(f.score(), 0.0f);
}
TEST(StuckSense, TheStallChannelFiresWhenTheStrideOscillatesAroundZeroWhileCommanding) {
    // Walk free for 10 s at 0.03 m/s (v_ref learns the body's speed), then press on a wall with
    // no echo at all (the range channel is blind) while the stride oscillates +/-0.3 around 0.
    Fix f({{"window_ticks", int64_t{10}}, {"v_ema_alpha", 0.05}, {"v_ref_tau_s", 20.0}});
    uint64_t t = 0;
    for (; t < 500; ++t) f.run(t, 1.5f, false, 0.03f);
    EXPECT_LT(f.m.stall(), 0.05f) << "walking at its own speed: no stall";
    for (int i = 0; i < 300; ++i, ++t) f.run(t, 1.5f, false, (i % 2 == 0) ? 0.3f : -0.3f);
    EXPECT_GT(f.m.stall(), 0.9f) << "commanding forward, mean stride ~0: a stall, with no beam needed";
    EXPECT_GT(f.score(), 0.9f);
}
TEST(StuckSense, ASlowerCommandedWalkIsNotAStall) {
    Fix f({{"window_ticks", int64_t{10}}, {"v_ema_alpha", 0.05}});
    uint64_t t = 0;
    for (; t < 500; ++t) f.run(t, 1.5f, false, 0.03f);
    for (int i = 0; i < 300; ++i, ++t) f.run(t, 1.5f, false, 0.015f, 0.5f);   // half speed commanded, half speed walked
    EXPECT_LT(f.m.stall(), 0.05f) << "0.5 of v_ref is above stall_frac 0.25";
}
TEST(StuckSense, AbstainsWhileTheBodyIsActuallyTurningNotWhileATurnIsMerelyCommanded) {
    Fix f({{"window_ticks", int64_t{10}}, {"v_ema_alpha", 0.05}, {"odom_topic", std::string("reality.proprio.odom")}});
    uint64_t t = 0; double yaw = 0.0;
    auto step = [&](float vfwd, double dyaw) {
        f.bus.begin_tick(t); yaw += dyaw;
        f.bus.publish("sense.range", pv({1.5f, 0.0f})); f.bus.publish("reality.proprio.stride_v", pv({0.0f, vfwd}));
        f.bus.publish("reality.proprio.speed_cmd", pv({1.0f})); f.bus.publish("reality.proprio.odom", pv({0.0f, 0.0f, float(yaw)}));
        f.m.tick(t); f.bus.end_tick(); ++t;
    };
    for (int i = 0; i < 500; ++i) step(0.03f, 0.0);                 // walking free, straight
    for (int i = 0; i < 300; ++i) step(0.0f, 0.3 / 50.0);           // TURNING at 0.3 rad/s, stride zero: not a stall
    EXPECT_FLOAT_EQ(f.m.stall(), 0.0f) << "the body is turning: the sense abstains";
    // Pinned on a wall: stride zero, and the gait ROCKS the yaw +/-0.2 rad/s at a 70-tick period
    // with no net turn.  The windowed net yaw sees no turn; a short EMA would have.
    for (int i = 0; i < 400; ++i) step(0.0f, 0.2 / 50.0 * std::sin(2.0 * M_PI * i / 70.0));
    EXPECT_GT(f.m.stall(), 0.9f) << "not moving, rocking but not turning, commanding forward: a stall";
}
