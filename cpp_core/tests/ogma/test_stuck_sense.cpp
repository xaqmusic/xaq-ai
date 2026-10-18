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

TEST(StuckSense, TheRangeChannelExpectedFromTheCommandFiresWhenTheStrideEstimateIsNoise) {
    // The fatal stall of 2026-10-18: a wall ahead at 0.6 m not getting closer, the brain commanding
    // forward, the stride estimate reading noise around zero (carpet).  Legacy: the channel is
    // silent (the stride claims no walking).  From the command: it fires.
    Fix legacy({{"window_ticks", int64_t{10}}, {"ema_alpha", 0.5}, {"min_expected_m", 0.002}});
    Fix cmd({{"window_ticks", int64_t{10}}, {"ema_alpha", 0.5}, {"min_expected_m", 0.002}, {"expect_from_cmd", 0.03}});
    for (uint64_t t = 0; t < 40; ++t) {
        const float noise = (t % 2 == 0) ? 0.02f : -0.02f;
        legacy.run(t, 0.6f, true, noise); cmd.run(t, 0.6f, true, noise);
    }
    EXPECT_FALSE(legacy.m.measured()) << "the stride claims nothing, the legacy channel abstains";
    EXPECT_TRUE(cmd.m.measured());
    EXPECT_GT(cmd.m.last_deficit(), 0.9f) << "commanding forward, the wall does not approach: a full deficit";
    auto s = std::dynamic_pointer_cast<const ogma::ProprioToken>(cmd.bus.last_value("reality.proprio.stuck"));
    ASSERT_TRUE(s); ASSERT_EQ(s->values.size(), 5);
    EXPECT_GT(s->values[4], 0.9f) << "the smoothed range channel is published on its own";
    // Walking free toward it at the commanded rate: no deficit.
    Fix ok({{"window_ticks", int64_t{10}}, {"ema_alpha", 0.5}, {"min_expected_m", 0.002}, {"expect_from_cmd", 0.03}});
    float r = 1.2f;
    for (uint64_t t = 0; t < 40; ++t) { ok.run(t, r, true, 0.0f); r -= 0.03f / 50.0f; }
    EXPECT_LT(ok.m.last_deficit(), 0.15f);
}

TEST(StuckSense, MedianWindowEndsResolveAWalkUnderTheSonarsJitterWhereSingleSamplesCoinFlip) {
    // The robot run of 2026-10-18 (sg2): the sonar's reading jitters 0.04 m (median) to 0.17 m
    // (90th percentile) over a second and a third of its readings are invalid, while the body
    // closes 0.03 m in that second.  Two single end samples over 1 s make the deficit a coin
    // flip (measured 35 % exactly 1 and 38 % exactly 0 while the odometry said progressing);
    // the gate withheld a fifth of the push on a body that was never stuck.
    auto jitter = [](uint64_t t) { const double j[] = {0.0, 0.08, -0.05, 0.12, -0.09, 0.03, -0.11}; return float(j[t % 7]); };
    auto valid  = [](uint64_t t) { return t % 3 != 1; };
    ogma::ParamMap single = {{"window_ticks", int64_t{50}},  {"ema_alpha", 0.05}, {"min_expected_m", 0.02}, {"expect_from_cmd", 0.03}, {"decay", 0.0}};
    ogma::ParamMap median = {{"window_ticks", int64_t{500}}, {"ema_alpha", 0.05}, {"min_expected_m", 0.15}, {"expect_from_cmd", 0.03}, {"decay", 0.0},
                             {"end_median_ticks", int64_t{100}}};
    // Walking toward a wall at the commanded rate under that jitter.
    Fix s_walk(single), m_walk(median);
    float r = 1.2f;
    for (uint64_t t = 0; t < 1500; ++t) {
        s_walk.run(t, r + jitter(t), valid(t), 0.0f); m_walk.run(t, r + jitter(t), valid(t), 0.0f);
        r -= 0.03f / 50.0f;
    }
    EXPECT_GT(s_walk.score(), 0.3f) << "single end samples: the jitter reads as a deficit on a walking body";
    // The jitter pattern's phase at the two ends leaves a residual (~0.05 m on the 0.3 m expected):
    // the walk reads a fifth of a deficit, not a coin flip.
    EXPECT_LT(m_walk.score(), 0.3f) << "median ends over 10 s: the walk is resolved to well under the coin flip";
    EXPECT_LT(m_walk.score(), 0.5f * s_walk.score());
    // Pinned against the wall under the same jitter: the median channel still fires.
    Fix m_stall(median);
    for (uint64_t t = 0; t < 1500; ++t) m_stall.run(t, 0.6f + jitter(t), valid(t), 0.0f);
    EXPECT_GT(m_stall.score(), 0.8f) << "the wall does not approach: a full deficit, jitter or not";
    EXPECT_TRUE(m_stall.m.measured());
}
