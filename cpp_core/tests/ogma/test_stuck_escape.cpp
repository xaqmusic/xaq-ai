// StuckEscape: confirm, back, turn to the freest sector (or the alternating fallback), refract.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/StuckEscape.hpp"
namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
struct Fix {
    ogma::InProcessBus bus; ogma::StuckEscape m;
    Fix(ogma::ParamMap p = {{"confirm_ticks", int64_t{5}}, {"back_ticks", int64_t{4}}, {"turn_ticks", int64_t{6}}, {"refract_ticks", int64_t{3}}}) { m.set_id("escape"); m.on_setup(&bus, p); }
    void run(uint64_t t, float score, std::vector<float> view = {1,1,1,1,1,1,1,1}, float yaw = 0.0f) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.stuck", pv({score, 0, 0, 0}));
        bus.publish("reality.proprio.prox_view", pv(view));
        bus.publish("reality.proprio.odom", pv({0, 0, yaw}));
        m.tick(t); bus.end_tick();
    }
    std::vector<float> out() const { auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("percept.escape")); std::vector<float> v; if (o) for (int i = 0; i < 4; ++i) v.push_back(o->values[i]); return v; }
};
using S = ogma::StuckEscape::State;
}
TEST(StuckEscape, IdleIsSilentAndABriefSpikeDoesNotTrigger) {
    Fix f; for (uint64_t t = 0; t < 3; ++t) f.run(t, 0.9f);
    EXPECT_EQ(f.m.state(), S::Idle); EXPECT_FLOAT_EQ(f.out()[2], 0.0f);
    f.run(3, 0.0f); f.run(4, 0.9f); f.run(5, 0.9f);
    EXPECT_EQ(f.m.state(), S::Idle) << "the confirm counter resets when the score drops";
}
TEST(StuckEscape, ConfirmedStuckBacksThenTurnsToTheFreestSectorThenRefracts) {
    Fix f;
    // Sector 7 (the +X side, +52.5 deg) is free; everything else is blocked at 0.3.
    std::vector<float> view = {0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 1.0f};
    uint64_t t = 0;
    for (; t < 5; ++t) f.run(t, 0.9f, view);
    ASSERT_EQ(f.m.state(), S::Back);
    auto o = f.out(); EXPECT_FLOAT_EQ(o[2], 1.0f); EXPECT_FLOAT_EQ(o[3], -0.5f) << "backing at back_speed"; EXPECT_FLOAT_EQ(o[0], 0.0f);
    EXPECT_NEAR(f.m.chosen_turn_rad(), 52.5 * M_PI / 180.0, 1e-6) << "sector 7 = +52.5 deg";
    for (int i = 0; i < 4; ++i) f.run(t++, 0.9f, view);
    ASSERT_EQ(f.m.state(), S::Turn);
    o = f.out(); EXPECT_GT(o[0], 0.0f) << "turning toward +X"; EXPECT_FLOAT_EQ(o[3], 0.5f);
    // Half way through the turn the body has turned 30 deg: the published bearing is the REMAINING 22.5.
    f.run(t++, 0.9f, view, float(30.0 * M_PI / 180.0));
    o = f.out(); EXPECT_NEAR(std::atan2(o[0], o[1]), 22.5 * M_PI / 180.0, 1e-4);
    for (int i = 0; i < 5; ++i) f.run(t++, 0.9f, view, float(52.5 * M_PI / 180.0));
    ASSERT_EQ(f.m.state(), S::Refract);
    o = f.out(); EXPECT_FLOAT_EQ(o[2], 0.0f) << "released";
    for (int i = 0; i < 3; ++i) f.run(t++, 0.9f, view);
    EXPECT_EQ(f.m.state(), S::Idle);
    EXPECT_EQ(f.m.episodes(), 1);
}
TEST(StuckEscape, AnEmptyViewFallsBackToAnAlternatingTurn) {
    Fix f;
    uint64_t t = 0;
    for (; t < 5; ++t) f.run(t, 0.9f);                   // view all 1.0: nothing remembered
    ASSERT_EQ(f.m.state(), S::Back);
    EXPECT_NEAR(f.m.chosen_turn_rad(), 90.0 * M_PI / 180.0, 1e-6);
    for (int i = 0; i < 4 + 6 + 3; ++i) f.run(t++, 0.0f);   // run the episode out
    ASSERT_EQ(f.m.state(), S::Idle);
    for (int i = 0; i < 5; ++i) f.run(t++, 0.9f);
    ASSERT_EQ(f.m.state(), S::Back);
    EXPECT_NEAR(f.m.chosen_turn_rad(), -90.0 * M_PI / 180.0, 1e-6) << "the other way the second time";
    EXPECT_EQ(f.m.episodes(), 2);
}
TEST(StuckEscape, SectorZeroIsTheMinusXSide) {
    Fix f; std::vector<float> view = {1.0f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f};
    for (uint64_t t = 0; t < 5; ++t) f.run(t, 0.9f, view);
    EXPECT_NEAR(f.m.chosen_turn_rad(), -52.5 * M_PI / 180.0, 1e-6) << "sector 0 = -52.5 deg, toward -X";
}
TEST(StuckEscape, AmongEquallyFreeSectorsTheOneNearestAheadWins) {
    // Sectors 0, 3 and 6 free; the rest blocked.  Sector 3 (-7.5 deg) is nearest ahead, not sector 0.
    Fix f; std::vector<float> view = {1.0f, 0.3f, 0.3f, 1.0f, 0.3f, 0.3f, 1.0f, 0.3f};
    for (uint64_t t = 0; t < 5; ++t) f.run(t, 0.9f, view);
    EXPECT_NEAR(f.m.chosen_turn_rad(), -7.5 * M_PI / 180.0, 1e-6);
}
