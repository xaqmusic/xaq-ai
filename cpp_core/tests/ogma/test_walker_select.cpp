// WalkerCompetence and WalkerSelect (learned stepper plan 9.6 item 2, 2026-11-19): a walker's progress report graded only while it
// holds the legs, and the selection between two walkers from their reports, in shadow.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/WalkerCompetence.hpp"
#include "ogma/modules/WalkerSelect.hpp"
namespace {
using ogma::ParamMap;
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
struct Rep {
    ogma::InProcessBus bus; ogma::WalkerCompetence m; uint64_t t = 0;
    explicit Rep(ParamMap extra = {}) {
        ParamMap p{{"stuck_topic", std::string("stuck")}, {"share_topic", std::string("pose")}, {"speed_topic", std::string("speed")}, {"output_topic", std::string("report.w")}};
        for (auto const& kv : extra) p[kv.first] = kv.second;
        m.set_id("report_w"); m.on_setup(&bus, p);
    }
    void run(int ticks, float share, float vfwd, float range_score = 0.0f, float speed = 1.0f) {
        for (int i = 0; i < ticks; ++i, ++t) {
            bus.begin_tick(t);
            bus.publish("reality.proprio.stride_v", pv({0.0f, vfwd})); bus.publish("stuck", pv({0.0f, 0.0f, 0.0f, 0.0f, range_score}));
            bus.publish("pose", pv({share})); bus.publish("speed", pv({speed}));
            m.tick(t); bus.end_tick();
        }
    }
};
}
TEST(WalkerCompetence, GradesOnlyWhileItHoldsTheLegs) {
    Rep f;
    f.run(1500, 1.0f, 0.10f);
    EXPECT_NEAR(f.m.rate(), 0.10, 0.002); EXPECT_GT(f.m.mean(), 0.08); EXPECT_TRUE(f.m.driving());
    const double rate = f.m.rate(), mean = f.m.mean(), var = f.m.var(); const int64_t n = f.m.graded();
    f.run(1500, 0.0f, 0.005f);   // the other walker crawls the body along: none of it is this walker's
    EXPECT_FALSE(f.m.driving());
    EXPECT_DOUBLE_EQ(f.m.rate(), rate); EXPECT_DOUBLE_EQ(f.m.mean(), mean); EXPECT_DOUBLE_EQ(f.m.var(), var); EXPECT_EQ(f.m.graded(), n);
}
TEST(WalkerCompetence, APartialShareIsNotGraded) {
    Rep f;
    f.run(500, 0.5f, 0.10f);     // mid-crossfade: neither walker's stepping is the body's motion
    EXPECT_EQ(f.m.graded(), 0); EXPECT_FALSE(f.m.driving());
}
TEST(WalkerCompetence, TheInvertedShareIsTheRhythmUnderThePoseSocket) {
    Rep f({{"share_invert", true}});
    f.run(300, 1.0f, 0.10f);     // the socket says the sequencer has the legs
    EXPECT_EQ(f.m.graded(), 0);
    f.run(300, 0.0f, 0.10f);
    EXPECT_EQ(f.m.graded(), 300);
}
TEST(WalkerCompetence, TheRangeAheadNotClosingDiscountsTheStridesClaim) {
    Rep f;
    f.run(1500, 1.0f, 0.02f, 0.9f);   // stepping at a riser: the stride claims 2 cm/s, the range says a tenth of it
    EXPECT_NEAR(f.m.rate(), 0.002, 0.0002);
}
TEST(WalkerCompetence, NoForwardCommandIsNoEvidence) {
    Rep f;
    f.run(1000, 1.0f, 0.10f);
    const double rate = f.m.rate(); const int64_t n = f.m.graded();
    f.run(500, 1.0f, 0.0f, 0.0f, 0.0f);   // commanded to stand: standing is not a stall
    EXPECT_DOUBLE_EQ(f.m.rate(), rate); EXPECT_EQ(f.m.graded(), n);
}
TEST(WalkerCompetence, IdleTheEstimateHoldsUntilTheBodyHasTravelled) {
    Rep f({{"relax_m", 0.25}});
    f.run(3000, 1.0f, 0.10f);         // a minute at pace
    f.run(500, 1.0f, 0.0f);           // then ten seconds stalled
    ASSERT_LT(f.m.rate(), 0.002); const double mean = f.m.mean(); ASSERT_GT(mean, 0.05);
    f.run(1000, 0.0f, 0.0f);          // handed off; the other walker does not move the body either
    EXPECT_LT(f.m.est(), 0.002) << "the same ground: the estimate is what it was at the hand-off";
    EXPECT_NEAR(f.m.p_same(), 1.0, 1e-9); EXPECT_NEAR(f.m.sd(), 0.0, 1e-9);
    f.run(2500, 0.0f, 0.005f);        // 50 s at 5 mm/s: a body length of travel
    EXPECT_NEAR(f.m.p_same(), std::exp(-1.0), 0.01);
    EXPECT_NEAR(f.m.est(), (1.0 - std::exp(-1.0)) * mean, 0.005) << "new ground: the estimate has gone most of the way back to the walker's mean";
    EXPECT_GT(f.m.sd(), 0.0);
    EXPECT_DOUBLE_EQ(f.m.mean(), mean);
}
TEST(WalkerCompetence, RockingInPlaceIsTheSameGround) {
    Rep f({{"relax_m", 0.25}});
    f.run(3000, 1.0f, 0.10f); f.run(500, 1.0f, 0.0f);
    const double held = f.m.rate();
    f.run(1, 0.0f, 0.0f);
    for (int k = 0; k < 60; ++k) { f.run(25, 0.0f, 0.04f); f.run(25, 0.0f, -0.04f); }   // a minute of the other walker's shifts: 2.4 m of path, no travel
    EXPECT_GT(f.m.p_same(), 0.9); EXPECT_NEAR(f.m.est(), held, 0.01);
}
TEST(WalkerCompetence, TheSwitchingCostIsMeasuredAndTheSettleIsNotAStall) {
    Rep f;
    f.run(3000, 1.0f, 0.10f);
    f.run(200, 0.0f, 0.0f);
    const double handed = f.m.est(); const int64_t n = f.m.graded(); ASSERT_GT(handed, 0.08);
    f.run(250, 1.0f, 0.0f);           // the take-over: five seconds of settling, no progress
    EXPECT_TRUE(f.m.settling()); EXPECT_DOUBLE_EQ(f.m.est(), handed) << "the settle is not read as a stall";
    EXPECT_EQ(f.m.graded(), n) << "nor does it enter the walker's statistics";
    f.run(250, 1.0f, 0.10f);          // then it walks
    EXPECT_FALSE(f.m.settling()); EXPECT_EQ(f.m.costs(), 1);
    EXPECT_GT(f.m.switch_cost_s(), 5.5); EXPECT_LT(f.m.switch_cost_s(), 8.0);   // five seconds standing plus the rate's own rise to half pace
}
TEST(WalkerCompetence, ATakeOverThatNeverGetsGoingEndsAsALowReport) {
    Rep f({{"settle_cap_s", 4.0}});
    f.run(3000, 1.0f, 0.10f);
    f.run(200, 0.0f, 0.0f);
    f.run(400, 1.0f, 0.0f);           // eight seconds without progress: past the cap
    EXPECT_FALSE(f.m.settling()); EXPECT_EQ(f.m.costs(), 0);
    EXPECT_LT(f.m.est(), 0.01);
}
TEST(WalkerCompetence, StateRoundTrips) {
    Rep a; a.run(2000, 1.0f, 0.08f); a.run(300, 0.0f, 0.004f);
    Rep b; b.m.restore_state(a.m.snapshot_state());
    a.run(200, 0.0f, 0.004f); b.t = a.t - 200; b.run(200, 0.0f, 0.004f);
    EXPECT_DOUBLE_EQ(a.m.est(), b.m.est()); EXPECT_DOUBLE_EQ(a.m.mean(), b.m.mean()); EXPECT_EQ(a.m.graded(), b.m.graded());
}

namespace {
struct Sel {
    ogma::InProcessBus bus; ogma::WalkerSelect m; uint64_t t = 0;
    explicit Sel(ParamMap extra = {}) {
        ParamMap p{{"rhythm_report_topic", std::string("report.w")}, {"sequencer_report_topic", std::string("report.s")}, {"share_topic", std::string("pose")}};
        for (auto const& kv : extra) p[kv.first] = kv.second;
        m.set_id("select"); m.on_setup(&bus, p);
    }
    // each report: est, sd, n, cost
    void run(int ticks, float share, std::array<float, 4> w, std::array<float, 4> s) {
        for (int i = 0; i < ticks; ++i, ++t) {
            bus.begin_tick(t);
            bus.publish("report.w", pv({w[0], w[1], 0.0f, 0.0f, 0.0f, w[2], w[3]})); bus.publish("report.s", pv({s[0], s[1], 0.0f, 0.0f, 0.0f, s[2], s[3]}));
            bus.publish("pose", pv({share}));
            m.tick(t); bus.end_tick();
        }
    }
};
}
TEST(WalkerSelect, KeepsTheWalkerThatIsAtPace) {
    Sel f;
    f.run(2000, 0.0f, {0.10f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.001f, 1000.0f, 10.0f});
    EXPECT_EQ(f.m.incumbent(), 0); EXPECT_EQ(f.m.want(), 0); EXPECT_EQ(f.m.calls(), 0); EXPECT_EQ(f.m.agree_ticks(), f.m.ticks());
}
TEST(WalkerSelect, CallsForTheSequencerOnceTheRhythmHasStalledForAHandOversLength) {
    Sel f;
    f.run(1000, 0.0f, {0.10f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});
    f.run(499, 0.0f, {0.001f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});   // just under the sequencer's ten seconds
    EXPECT_EQ(f.m.want(), 0) << "the lead has not yet lasted as long as the hand-over takes";
    f.run(2, 0.0f, {0.001f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});
    EXPECT_EQ(f.m.want(), 1); EXPECT_EQ(f.m.calls(), 1);
}
TEST(WalkerSelect, AFlickeringLeadNeverCompletesTheDwell) {
    Sel f;
    f.run(500, 0.0f, {0.10f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});
    for (int k = 0; k < 20; ++k) {   // rough ground: under the sequencer's pace for six seconds, over it for one
        f.run(300, 0.0f, {0.001f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});
        f.run(50, 0.0f, {0.02f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});
    }
    EXPECT_EQ(f.m.calls(), 0);
}
TEST(WalkerSelect, TheHandOversCostIsPricedIntoTheChallenger) {
    Sel f({{"horizon_s", 30.0}});
    // the sequencer's 0.007 is worth 0.007 * (1 - 10 / 30) = 0.0047 after its settle: a rhythm at 0.005 keeps the legs
    f.run(2000, 0.0f, {0.005f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});
    EXPECT_EQ(f.m.want(), 0); EXPECT_LT(f.m.lead(), 0.0);
}
TEST(WalkerSelect, AnUntriedWalkerIsCalledForOnceTheIncumbentHasAGrade) {
    Sel f;
    f.run(50, 0.0f, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f});
    EXPECT_EQ(f.m.want(), 0) << "nobody graded yet: the incumbent stays";
    f.run(200, 0.0f, {0.10f, 0.0f, 500.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f});
    EXPECT_EQ(f.m.want(), 1) << "the sequencer has never driven: it is tried";
}
TEST(WalkerSelect, WhatIsNotKnownCountsForTheIdleWalker) {
    Sel f({{"optimism", 1.0}});
    // the sequencer drives at 0.006; the idle rhythm's estimate is 0.002 with a spread of 0.03 about it after the body has travelled
    f.run(400, 1.0f, {0.002f, 0.03f, 1000.0f, 2.5f}, {0.006f, 0.0f, 1000.0f, 10.0f});
    EXPECT_EQ(f.m.incumbent(), 1); EXPECT_EQ(f.m.want(), 0);
    Sel g({{"optimism", 0.0}});
    g.run(400, 1.0f, {0.002f, 0.03f, 1000.0f, 2.5f}, {0.006f, 0.0f, 1000.0f, 10.0f});
    EXPECT_EQ(g.m.want(), 1) << "without optimism the mean alone keeps the sequencer";
}
TEST(WalkerSelect, AHandOverRestartsTheCount) {
    Sel f;
    f.run(1000, 0.0f, {0.001f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});
    ASSERT_EQ(f.m.want(), 1);
    f.run(1, 1.0f, {0.001f, 0.0f, 1000.0f, 2.5f}, {0.007f, 0.0f, 1000.0f, 10.0f});   // the legs go to the sequencer
    EXPECT_EQ(f.m.incumbent(), 1); EXPECT_EQ(f.m.want(), 1) << "it has what it wanted";
    EXPECT_NEAR(f.m.lead_s(), 0.0, 1e-9);
}
