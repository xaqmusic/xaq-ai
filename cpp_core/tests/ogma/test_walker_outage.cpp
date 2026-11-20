// WalkerOutage and the supply in WalkerSelect (learned stepper plan 9.10, 2026-11-20): the supply's outages attributed to the
// walker that held the legs, the veto, the retry by travel, and the selector's reflex.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/WalkerOutage.hpp"
#include "ogma/modules/WalkerSelect.hpp"
namespace {
using ogma::ParamMap;
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
struct Out {
    ogma::InProcessBus bus; ogma::WalkerOutage m; uint64_t t = 0; float count = 0.0f;
    explicit Out(ParamMap extra = {}) {
        ParamMap p{{"share_topic", std::string("pose")}};
        for (auto const& kv : extra) p[kv.first] = kv.second;
        m.set_id("outage"); m.on_setup(&bus, p);
    }
    // share = the sequencer's; resets = outages in the FIRST tick of the stretch
    void run(int ticks, float share, float vfwd = 0.0f, int resets = 0, float rail = 3.3f) {
        for (int i = 0; i < ticks; ++i, ++t) {
            if (i == 0) count += float(resets);
            bus.begin_tick(t);
            bus.publish("sense.hat_reset", pv({count})); bus.publish("sense.hat_rail", pv({rail}));
            bus.publish("pose", pv({share})); bus.publish("reality.proprio.stride_v", pv({0.0f, vfwd}));
            m.tick(t); bus.end_tick();
        }
    }
};
}
TEST(WalkerOutage, AnOutageBelongsToTheWalkerAtTheLegs) {
    Out f;
    f.run(100, 0.0f);
    f.run(100, 0.0f, 0.1f, 1);
    EXPECT_DOUBLE_EQ(f.m.est(0), 1.0); EXPECT_DOUBLE_EQ(f.m.est(1), 0.0); EXPECT_EQ(f.m.resets_seen(), 1); EXPECT_EQ(f.m.driver(), 0);
    f.run(100, 1.0f);
    f.run(100, 1.0f, 0.0f, 1);
    EXPECT_DOUBLE_EQ(f.m.count(1), 1.0); EXPECT_DOUBLE_EQ(f.m.count(0), 1.0); EXPECT_EQ(f.m.driver(), 1);
}
TEST(WalkerOutage, TheCountAtStartUpIsNotAnOutage) {
    Out f; f.count = 7.0f;       // the host has seen seven before the brain started
    f.run(200, 0.0f);
    EXPECT_EQ(f.m.resets_seen(), 0); EXPECT_DOUBLE_EQ(f.m.est(0), 0.0);
}
TEST(WalkerOutage, AnOutageLeavesTheWindowAfterAMinuteOfDriving) {
    Out f;
    f.run(10, 0.0f); f.run(50 * 59, 0.0f, 0.1f, 1);
    EXPECT_DOUBLE_EQ(f.m.est(0), 1.0);
    f.run(50 * 2, 0.0f, 0.1f);
    EXPECT_DOUBLE_EQ(f.m.est(0), 0.0);
}
TEST(WalkerOutage, TwoInAMinuteVetoTheWalkerAndOneDoesNot) {
    Out f;
    f.run(10, 0.0f); f.run(500, 0.0f, 0.1f, 1);
    EXPECT_FALSE(f.m.veto(0));
    f.run(500, 0.0f, 0.1f, 1);
    EXPECT_TRUE(f.m.veto(0)); EXPECT_FALSE(f.m.veto(1)); EXPECT_EQ(f.m.backoff(0), 1);
}
TEST(WalkerOutage, AnIdleWalkersClockStandsStillAndOnlyTravelReleasesIt) {
    Out f;
    f.run(10, 0.0f); f.run(100, 0.0f, 0.1f, 2);
    ASSERT_TRUE(f.m.veto(0));
    f.run(50 * 300, 1.0f, 0.0f);          // five minutes under the sequencer on the same spot
    EXPECT_TRUE(f.m.veto(0)) << "time alone is not another ground"; EXPECT_DOUBLE_EQ(f.m.count(0), 2.0); EXPECT_DOUBLE_EQ(f.m.est(0), 2.0);
    // backoff 1: the scale is 0.5 m; the veto drops at est <= 1, p_same <= 0.5, 0.347 m of net travel
    f.run(50 * 30, 1.0f, 0.01f);          // 0.30 m
    EXPECT_TRUE(f.m.veto(0));
    f.run(50 * 6, 1.0f, 0.01f);           // 0.36 m
    EXPECT_FALSE(f.m.veto(0)); EXPECT_NEAR(f.m.est(0), 2.0 * std::exp(-0.36 / 0.5), 0.02);
}
TEST(WalkerOutage, RockingInPlaceIsNotTravel) {
    Out f;
    f.run(10, 0.0f); f.run(100, 0.0f, 0.1f, 2);
    for (int k = 0; k < 200; ++k) { f.run(25, 1.0f, 0.05f); f.run(25, 1.0f, -0.05f); }   // 5 m of path, none of it net
    EXPECT_TRUE(f.m.veto(0));
}
TEST(WalkerOutage, ARetryStartsOnWhatWasBelievedAndOneMoreOutageVetoesAgainWithALongerWait) {
    Out f;
    f.run(10, 0.0f); f.run(100, 0.0f, 0.1f, 2);
    f.run(50 * 40, 1.0f, 0.01f);          // 0.40 m under the sequencer: released
    ASSERT_FALSE(f.m.veto(0));
    const double believed = f.m.est(0);
    f.run(100, 0.0f, 0.1f);               // the rhythm takes the legs back
    EXPECT_NEAR(f.m.est(0), believed, 1e-9) << "the weights carry the belief into the retry";
    f.run(100, 0.0f, 0.1f, 1);            // one outage on the retry
    EXPECT_TRUE(f.m.veto(0)); EXPECT_EQ(f.m.backoff(0), 2);
    f.run(50 * 40, 1.0f, 0.01f);          // the same 0.40 m is no longer enough: the scale is 1 m
    EXPECT_TRUE(f.m.veto(0));
}
TEST(WalkerOutage, ACleanMinuteAtTheLegsClearsTheBackoff) {
    Out f;
    f.run(10, 0.0f); f.run(100, 0.0f, 0.1f, 2);
    f.run(50 * 60, 1.0f, 0.01f);
    f.run(50 * 61, 0.0f, 0.1f);           // a clean window on new ground
    EXPECT_EQ(f.m.backoff(0), 0); EXPECT_DOUBLE_EQ(f.m.est(0), 0.0); EXPECT_FALSE(f.m.veto(0));
}
TEST(WalkerOutage, ADipIsOneEntryHoweverLongItLasts) {
    Out f;
    f.run(100, 0.0f);
    f.run(40, 0.0f, 0.1f, 0, 2.8f); f.run(10, 0.0f, 0.1f); f.run(5, 0.0f, 0.1f, 0, 2.9f); f.run(10, 0.0f, 0.1f);
    EXPECT_DOUBLE_EQ(f.m.dips(0), 2.0); EXPECT_DOUBLE_EQ(f.m.dips(1), 0.0); EXPECT_DOUBLE_EQ(f.m.est(0), 0.0) << "a dip is not an outage";
}
TEST(WalkerOutage, MidCrossfadeTheOutageGoesToTheLargerShareAndNobodysClockRuns) {
    Out f;
    f.run(100, 0.0f); f.run(20, 0.7f, 0.0f, 1);
    EXPECT_DOUBLE_EQ(f.m.count(1), 1.0); EXPECT_DOUBLE_EQ(f.m.count(0), 0.0); EXPECT_EQ(f.m.driver(), -1);
}
TEST(WalkerOutage, StateRoundTrips) {
    Out a; a.run(10, 0.0f); a.run(100, 0.0f, 0.1f, 2); a.run(500, 1.0f, 0.01f);
    Out b; b.count = a.count; b.m.restore_state(a.m.snapshot_state());
    a.run(800, 1.0f, 0.01f); b.t = a.t - 800; b.run(800, 1.0f, 0.01f);
    EXPECT_DOUBLE_EQ(a.m.est(0), b.m.est(0)); EXPECT_EQ(a.m.veto(0), b.m.veto(0)); EXPECT_EQ(a.m.backoff(0), b.m.backoff(0));
}

namespace {
struct Sel {
    ogma::InProcessBus bus; ogma::WalkerSelect m; uint64_t t = 0;
    explicit Sel(ParamMap extra = {}) {
        ParamMap p{{"rhythm_report_topic", std::string("report.w")}, {"sequencer_report_topic", std::string("report.s")}, {"share_topic", std::string("pose")}, {"outage_topic", std::string("select.outage")}};
        for (auto const& kv : extra) p[kv.first] = kv.second;
        m.set_id("select"); m.on_setup(&bus, p);
    }
    // reports: est, sd, n, cost;  outage: est_r, est_s, veto_r, veto_s
    void run(int ticks, float share, std::array<float, 4> w, std::array<float, 4> s, std::array<float, 4> o) {
        for (int i = 0; i < ticks; ++i, ++t) {
            bus.begin_tick(t);
            bus.publish("report.w", pv({w[0], w[1], 0.0f, 0.0f, 0.0f, w[2], w[3]})); bus.publish("report.s", pv({s[0], s[1], 0.0f, 0.0f, 0.0f, s[2], s[3]}));
            bus.publish("pose", pv({share})); bus.publish("select.outage", pv({o[0], o[1], o[2], o[3]}));
            m.tick(t); bus.end_tick();
        }
    }
};
const std::array<float, 4> kFast{0.10f, 0.0f, 1000.0f, 2.5f}, kSlow{0.008f, 0.0f, 1000.0f, 6.0f};
}
TEST(WalkerSelectSupply, ACleanSupplyChangesNothing) {
    Sel f;
    f.run(2000, 0.0f, kFast, kSlow, {0.0f, 0.0f, 0.0f, 0.0f});
    EXPECT_EQ(f.m.want(), 0); EXPECT_EQ(f.m.calls(), 0); EXPECT_NEAR(f.m.g(0), 0.10, 1e-6);
}
TEST(WalkerSelectSupply, AnOutageCostsItsShareOfTheMinute) {
    Sel f;
    f.run(100, 0.0f, kFast, kSlow, {1.0f, 0.0f, 0.0f, 0.0f});
    EXPECT_NEAR(f.m.g(0), 0.10 * (1.0 - 2.5 / 60.0), 1e-6); EXPECT_EQ(f.m.want(), 0) << "one outage does not make the fast walker the slower one";
}
TEST(WalkerSelectSupply, AVetoedIncumbentHandsOverAtOnce) {
    Sel f;
    f.run(500, 0.0f, kFast, kSlow, {0.0f, 0.0f, 0.0f, 0.0f});
    f.run(1, 0.0f, kFast, kSlow, {2.0f, 0.0f, 1.0f, 0.0f});
    EXPECT_EQ(f.m.want(), 1) << "the reflex does not wait for a lead"; EXPECT_EQ(f.m.calls(), 1); EXPECT_EQ(f.m.veto_calls(), 1);
}
TEST(WalkerSelectSupply, AVetoedChallengerIsNotWantedWhateverItsLead) {
    Sel f;
    f.run(3000, 1.0f, kFast, kSlow, {2.0f, 0.0f, 1.0f, 0.0f});   // the sequencer has the legs, the rhythm is ten times faster and vetoed
    EXPECT_EQ(f.m.incumbent(), 1); EXPECT_EQ(f.m.want(), 1); EXPECT_EQ(f.m.calls(), 0);
    f.run(400, 1.0f, kFast, kSlow, {0.9f, 0.0f, 0.0f, 0.0f});    // released: the lead completes its dwell and the rhythm is called
    EXPECT_EQ(f.m.want(), 0); EXPECT_EQ(f.m.calls(), 1); EXPECT_EQ(f.m.veto_calls(), 0);
}
TEST(WalkerSelectSupply, BothVetoedKeepsTheOneWithFewerOutages) {
    Sel f;
    f.run(100, 0.0f, kFast, kSlow, {3.0f, 2.0f, 1.0f, 1.0f});
    EXPECT_EQ(f.m.want(), 1);
    Sel g;
    g.run(100, 0.0f, kFast, kSlow, {2.0f, 3.0f, 1.0f, 1.0f});
    EXPECT_EQ(g.m.want(), 0);
}
