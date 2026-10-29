// BearingToIntent: a loop's bearing -> the gait's goal bearing + speed; a held reference when silent.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/BearingToIntent.hpp"
namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
struct Fix {
    ogma::InProcessBus bus; ogma::BearingToIntent m;
    Fix(ogma::ParamMap p = {}) { m.set_id("intent"); m.on_setup(&bus, p); }
    void run(uint64_t t, double yaw, std::vector<float> bearing = {}) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.odom", pv({0.0f, 0.0f, float(yaw)}));
        if (!bearing.empty()) bus.publish("percept.avoid_bearing", pv(bearing));
        m.tick(t); bus.end_tick();
    }
    float speed() const { auto s = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("reality.proprio.speed_cmd")); return s ? s->values[0] : NAN; }
    std::pair<float,float> goal() const { auto g = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("reality.proprio.goal_bearing")); return g ? std::make_pair(g->values[0], g->values[1]) : std::make_pair(NAN, NAN); }
};
}
TEST(BearingToIntent, SilentFromTheStartHoldsTheStartHeadingAtFullSpeed) {
    Fix f; f.run(0, 0.0); f.run(1, 0.0);
    auto g = f.goal(); EXPECT_NEAR(g.first, 0.0f, 1e-6f); EXPECT_NEAR(g.second, 1.0f, 1e-6f);
    EXPECT_FLOAT_EQ(f.speed(), 1.0f);
    EXPECT_FALSE(f.m.active());
}
TEST(BearingToIntent, ALoopBearingIsPassedThroughAndSetsTheReference) {
    Fix f; f.run(0, 0.0, {0.7071f, 0.7071f, 0.8f});      // 45 deg toward +X
    EXPECT_TRUE(f.m.active());
    auto g = f.goal(); EXPECT_NEAR(g.first, 0.7071f, 1e-4f); EXPECT_NEAR(g.second, 0.7071f, 1e-4f);
    EXPECT_NEAR(f.m.ref_yaw(), M_PI / 4.0, 1e-6);
}
TEST(BearingToIntent, WhenTheLoopGoesSilentTheGoalPointsAtTheHeldReference) {
    Fix f; f.run(0, 0.0, {0.7071f, 0.7071f, 0.8f});      // reference = +45 deg
    f.run(1, 0.0, {0.0f, 0.0f, 0.0f});                   // silent; body still at 0
    EXPECT_FALSE(f.m.active());
    auto g = f.goal(); EXPECT_NEAR(std::atan2(g.first, g.second), M_PI / 4.0, 1e-4);
    f.run(2, M_PI / 4.0, {0.0f, 0.0f, 0.0f});            // the body turned to the reference
    auto h = f.goal(); EXPECT_NEAR(std::atan2(h.first, h.second), 0.0, 1e-4) << "on the reference: no error";
    f.run(3, M_PI / 2.0, {0.0f, 0.0f, 0.0f});            // overshot by 45 deg -> goal back toward -X
    auto k = f.goal(); EXPECT_NEAR(std::atan2(k.first, k.second), -M_PI / 4.0, 1e-4);
}
TEST(BearingToIntent, SpeedShedsWithTheTurnAndReverseBehindIsOptIn) {
    Fix f({{"speed_pow", 1.0}, {"speed_min", 0.2}});
    f.run(0, 0.0, {0.0f, 1.0f, 0.5f});                   // dead ahead
    EXPECT_NEAR(f.speed(), 1.0f, 1e-6f);
    f.run(1, 0.0, {0.7071f, 0.7071f, 0.5f});             // 45 deg -> cos = 0.707
    EXPECT_NEAR(f.speed(), 0.7071f, 1e-4f);
    f.run(2, 0.0, {1.0f, 0.0f, 0.5f});                   // 90 deg -> floor
    EXPECT_NEAR(f.speed(), 0.2f, 1e-5f);
    f.run(3, 0.0, {0.0f, -1.0f, 0.5f});                  // behind, reverse off -> floor
    EXPECT_NEAR(f.speed(), 0.2f, 1e-5f);
    Fix r({{"speed_pow", 1.0}, {"reverse_behind", true}});
    r.run(0, 0.0, {0.0f, -1.0f, 0.5f});
    EXPECT_NEAR(r.speed(), -1.0f, 1e-5f) << "behind with reverse on: back up";
}
TEST(BearingToIntent, NoGatingByDefault) {
    Fix f; f.run(0, 0.0, {1.0f, 0.0f, 0.5f});           // 90 deg, speed_pow 0
    EXPECT_FLOAT_EQ(f.speed(), 1.0f);
}
TEST(BearingToIntent, TheReferenceLeaksWhenAsked) {
    Fix f({{"ref_tau_s", 1.0}, {"hz", 50.0}});
    f.run(0, 0.0, {0.7071f, 0.7071f, 0.8f});             // reference +45 deg
    for (uint64_t t = 1; t <= 300; ++t) f.run(t, 0.0, {0.0f, 0.0f, 0.0f});   // 6 s silent, body at 0
    EXPECT_LT(std::fabs(f.m.ref_yaw()), 0.01) << "the reference leaked to the current heading";
}
TEST(BearingToIntent, AnOverrideWinsWhileItsValueIsPositiveAndItsSpeedGoesOutUnchanged) {
    Fix f({{"override_topic", std::string("percept.escape")}, {"speed_pow", 1.0}});
    f.run(0, 0.0, {0.0f, 1.0f, 0.5f});                         // the loop: dead ahead
    f.bus.begin_tick(1); f.bus.publish("reality.proprio.odom", pv({0, 0, 0}));
    f.bus.publish("percept.avoid_bearing", pv({0.0f, 1.0f, 0.5f}));
    f.bus.publish("percept.escape", pv({-0.7071f, 0.7071f, 1.0f, -0.5f}));   // escape: -45 deg, BACK at 0.5
    f.m.tick(1); f.bus.end_tick();
    auto g = f.goal(); EXPECT_NEAR(std::atan2(g.first, g.second), -M_PI / 4.0, 1e-4);
    EXPECT_NEAR(f.speed(), -0.5f, 1e-6f) << "the override's speed is not gated";
    EXPECT_NEAR(f.m.ref_yaw(), -M_PI / 4.0, 1e-6) << "the override set the reference";
    f.bus.begin_tick(2); f.bus.publish("reality.proprio.odom", pv({0, 0, 0}));
    f.bus.publish("percept.avoid_bearing", pv({0.0f, 0.0f, 0.0f}));           // the loop is silent
    f.bus.publish("percept.escape", pv({0.0f, 0.0f, 0.0f, 0.0f}));            // released
    f.m.tick(2); f.bus.end_tick();
    auto h = f.goal(); EXPECT_NEAR(std::atan2(h.first, h.second), -M_PI / 4.0, 1e-4) << "released and the loop silent: the held reference, which the escape set";
    f.bus.begin_tick(3); f.bus.publish("reality.proprio.odom", pv({0, 0, 0}));
    f.bus.publish("percept.avoid_bearing", pv({0.0f, 1.0f, 0.5f}));           // the loop speaks again
    f.m.tick(3); f.bus.end_tick();
    auto k = f.goal(); EXPECT_NEAR(std::atan2(k.first, k.second), 0.0, 1e-4) << "released and the loop active: the loop owns the bearing";
}

TEST(BearingToIntent, LeverA_TheReferenceIsEarnedByPersistenceNotSeizedByABlip) {
    // hz 50, attack 2 s: one active tick moves the reference 1/100 of the way; 400 ticks move it ~98 %.
    Fix f({{"ref_attack_s", 2.0}, {"hz", 50.0}});
    f.run(0, 0.0);                                        // silent: reference = the start heading (0)
    f.run(1, 0.0, {0.7071f, 0.7071f, 0.8f});              // a one-tick blip, 45 deg toward +X
    auto g = f.goal(); EXPECT_NEAR(std::atan2(g.first, g.second), M_PI / 4.0, 1e-4) << "the reflex still steers at full strength";
    EXPECT_NEAR(f.m.ref_yaw(), (M_PI / 4.0) * 0.01, 1e-6) << "but the held heading moved one percent of the way";
    f.run(2, 0.0, {0.0f, 0.0f, 0.0f});                    // silent again: the hold aims at the barely-moved reference
    auto h = f.goal(); EXPECT_LT(std::fabs(std::atan2(h.first, h.second)), 0.01);
    for (uint64_t t = 3; t < 403; ++t) f.run(t, 0.0, {0.7071f, 0.7071f, 0.8f});   // a persistent wall: 8 s
    EXPECT_GT(f.m.ref_yaw(), (M_PI / 4.0) * 0.97) << "a persistent loop earns the reference fully";
}
TEST(BearingToIntent, LeverA_AtZeroTheReferenceJumpsAsBefore) {
    Fix f({{"ref_attack_s", 0.0}});
    f.run(0, 0.0); f.run(1, 0.0, {0.7071f, 0.7071f, 0.8f});
    EXPECT_NEAR(f.m.ref_yaw(), M_PI / 4.0, 1e-6);
}

TEST(BearingToIntent, LeverA2_WeakEvidenceSteersButDoesNotRewriteTheHeldHeading) {
    Fix f({{"ref_value_min", 0.3}});
    f.run(0, 0.0);                                        // reference = 0
    f.run(1, 0.0, {0.7071f, 0.7071f, 0.15f});             // a blip: 45 deg toward +X, value 0.15
    EXPECT_TRUE(f.m.active());
    auto g = f.goal(); EXPECT_NEAR(std::atan2(g.first, g.second), M_PI / 4.0, 1e-4) << "the reflex still steers";
    EXPECT_NEAR(f.m.ref_yaw(), 0.0, 1e-9) << "the held heading is untouched";
    f.run(2, 0.3, {0.0f, 0.0f, 0.0f});                    // silent; the body wiggled 0.3 rad -> the hold aims back
    auto h = f.goal(); EXPECT_NEAR(std::atan2(h.first, h.second), -0.3, 1e-4);
    f.run(3, 0.3, {0.7071f, 0.7071f, 0.7f});              // a wall: value 0.7 -> rewrites
    EXPECT_NEAR(f.m.ref_yaw(), 0.3 + M_PI / 4.0, 1e-6);
}
TEST(BearingToIntent, LeverA2_AtZeroAnyValueRewrites) {
    Fix f({{"ref_value_min", 0.0}});
    f.run(0, 0.0); f.run(1, 0.0, {0.7071f, 0.7071f, 0.06f});
    EXPECT_NEAR(f.m.ref_yaw(), M_PI / 4.0, 1e-6);
}
