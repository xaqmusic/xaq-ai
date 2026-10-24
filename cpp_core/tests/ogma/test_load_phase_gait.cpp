// LoadPhaseGait: a per-leg phase that the leg's own load holds in stance.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/LoadPhaseGait.hpp"
namespace {
constexpr float kPi = 3.14159265358979f;
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
struct Fix {
    ogma::InProcessBus bus; ogma::LoadPhaseGait m;
    Fix(ogma::ParamMap p = {}) {
        if (!p.count("period_ticks")) p["period_ticks"] = 100.0;
        if (!p.count("init_phase")) p["init_phase"] = std::vector<double>{0.0, 0.0, 0.0, 0.0};
        m.set_id("load_phase"); m.on_setup(&bus, p);
    }
    void run(uint64_t t, std::vector<float> contact, std::vector<float> load = {}) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.foot_contact", pv(contact));
        if (!load.empty()) bus.publish("reality.proprio.foot_load", pv(load));
        m.tick(t); bus.end_tick();
    }
    float out(int leg, int k) const {
        static const char* topics[4] = {"rhythm.leg.fl", "rhythm.leg.fr", "rhythm.leg.rl", "rhythm.leg.rr"};
        auto s = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value(topics[leg]));
        return s ? s->values[k] : -99.0f;
    }
};
}
TEST(LoadPhaseGait, UnloadedLegsRunFreeAtThePeriod) {
    Fix f;
    for (uint64_t t = 0; t < 100; ++t) f.run(t, {0, 0, 0, 0});   // nothing touches: no load, no hold
    for (int leg = 0; leg < 4; ++leg) EXPECT_NEAR(f.m.phase(leg), 0.0f, 0.05f) << "one full period returns to the start";
    Fix g;
    for (uint64_t t = 0; t < 50; ++t) g.run(t, {0, 0, 0, 0});
    for (int leg = 0; leg < 4; ++leg) EXPECT_NEAR(g.m.phase(leg), kPi, 0.05f) << "half a period = touchdown";
}
TEST(LoadPhaseGait, SwingIsTheFirstHalfOfTheCycleAndItIsPublished) {
    Fix f;
    f.run(0, {0, 0, 0, 0});
    EXPECT_TRUE(f.m.swing(0)); EXPECT_FLOAT_EQ(f.out(0, 2), 1.0f);
    for (uint64_t t = 1; t < 60; ++t) f.run(t, {0, 0, 0, 0});
    EXPECT_FALSE(f.m.swing(0)); EXPECT_FLOAT_EQ(f.out(0, 2), 0.0f);
    EXPECT_NEAR(f.out(0, 0), std::cos(f.m.phase(0)), 1e-5f); EXPECT_NEAR(f.out(0, 1), std::sin(f.m.phase(0)), 1e-5f);
}
TEST(LoadPhaseGait, ALegCarryingTheWholeLoadIsHeldInStanceAndDoesNotLift) {
    // Only leg 0 touches: its share is 1.  With sigma_ratio 2 the phase has a stable point in
    // stance (cos phi = 1/2, sin < 0): started at mid-stance it must stay in stance for ever.
    Fix f({{"sigma_ratio", 2.0}, {"init_phase", std::vector<double>{1.5 * kPi, 0.0, 0.0, 0.0}}});
    for (uint64_t t = 0; t < 1000; ++t) {
        f.run(t, {1, 0, 0, 0});
        ASSERT_FALSE(f.m.swing(0)) << "tick " << t << ": the only support lifted";
    }
    EXPECT_NEAR(std::cos(f.m.phase(0)), 0.5f, 0.02f) << "held at the rule's fixed point cos phi = omega / (sigma N)";
    EXPECT_FLOAT_EQ(f.out(0, 3), 1.0f) << "the published share";
}
TEST(LoadPhaseGait, ASharedLoadOnlySlowsTheLeg) {
    // Four feet down: each share is 1/4, so the hold factor is at most 1 - 2 * 0.25 = 0.5 and
    // the phase keeps advancing through stance.
    Fix f({{"sigma_ratio", 2.0}, {"init_phase", std::vector<double>{1.5 * kPi, 1.5 * kPi, 1.5 * kPi, 1.5 * kPi}}});
    bool lifted = false;
    for (uint64_t t = 0; t < 200 && !lifted; ++t) { f.run(t, {1, 1, 1, 1}); lifted = f.m.swing(0); }
    EXPECT_TRUE(lifted) << "a quarter share cannot stop the cycle";
}
TEST(LoadPhaseGait, ForceSharesComeFromTheLoadTopicWhenPublished) {
    Fix f({{"load_topic", std::string("reality.proprio.foot_load")}, {"init_phase", std::vector<double>{1.5 * kPi, 1.5 * kPi, 1.5 * kPi, 1.5 * kPi}}});
    f.run(0, {1, 1, 1, 1}, {3.0f, 1.0f, 0.0f, 0.0f});
    EXPECT_NEAR(f.m.share(0), 0.75f, 1e-5f); EXPECT_NEAR(f.m.share(1), 0.25f, 1e-5f); EXPECT_FLOAT_EQ(f.m.share(2), 0.0f);
}
TEST(LoadPhaseGait, SigmaZeroIsTheFreeClockWhateverTheLoad) {
    Fix f({{"sigma_ratio", 0.0}, {"init_phase", std::vector<double>{1.5 * kPi, 0.0, 0.0, 0.0}}});
    for (uint64_t t = 0; t < 100; ++t) f.run(t, {1, 0, 0, 0});
    EXPECT_NEAR(f.m.phase(0), 1.5f * kPi, 0.05f) << "one period, unmoved by the load";
    EXPECT_NEAR(f.m.diag_snapshot()["omega_rel"].get<float>(), 1.0f, 1e-3f);
}
TEST(LoadPhaseGait, TheTouchdownInstrumentReadsOneWhenContactLandsAtOnePhase) {
    // Drive a leg free and switch the contact flag on exactly at phi = pi every cycle.
    Fix f({{"sigma_ratio", 0.0}});
    for (uint64_t t = 0; t < 2000; ++t) {
        const float ph = f.m.phase(0);
        const float c = (ph >= kPi) ? 1.0f : 0.0f;    // touching through the stance half
        f.run(t, {c, c, c, c});
    }
    const auto d = f.m.diag_snapshot();
    EXPECT_GT(d["td_plv"].get<float>(), 0.98f);
    EXPECT_GT(d["agree"].get<float>(), 0.95f) << "the phase's swing/stance matches the flag";
    EXPECT_NEAR(d["retro"].get<float>(), 0.0f, 1e-6f) << "a free clock never runs backwards";
    EXPECT_GT(d["interleg_plv"].get<float>(), 0.98f) << "four identical free clocks hold a constant relative phase";
}

TEST(LoadPhaseGait, PerFootNormalisationMakesAStiffSensorAndAWeakOneCarryEqualShares) {
    // The robot's FSRs under one curve (2026-10-13): one foot reads 3x another at the same
    // true load.  Raw shares hold the stiff foot (0.75) and free the weak one; normalised by
    // each foot's own running reference they read 0.5 each.
    Fix raw({{"load_topic", std::string("reality.proprio.foot_load")}, {"init_phase", std::vector<double>{1.5 * kPi, 1.5 * kPi, 0.0, 0.0}}});
    Fix nrm({{"load_topic", std::string("reality.proprio.foot_load")}, {"load_norm_tau", 20.0}, {"init_phase", std::vector<double>{1.5 * kPi, 1.5 * kPi, 0.0, 0.0}}});
    for (uint64_t t = 0; t < 300; ++t) { raw.run(t, {1, 1, 0, 0}, {3.0f, 1.0f, 0.0f, 0.0f}); nrm.run(t, {1, 1, 0, 0}, {3.0f, 1.0f, 0.0f, 0.0f}); }
    EXPECT_NEAR(raw.m.share(0), 0.75f, 1e-4f); EXPECT_NEAR(raw.m.share(1), 0.25f, 1e-4f);
    EXPECT_NEAR(nrm.m.share(0), 0.5f, 1e-3f);  EXPECT_NEAR(nrm.m.share(1), 0.5f, 1e-3f);
    // A real change of load still reads: foot 0 now carries twice its reference.
    nrm.run(300, {1, 1, 0, 0}, {6.0f, 1.0f, 0.0f, 0.0f});
    EXPECT_GT(nrm.m.share(0), 0.6f);
}
TEST(LoadPhaseGait, AbsoluteModeHoldsAFourFootStanceThatSharesWouldLetThrough) {
    // Four feet each carry a quarter of the body (0.25 body weights).  As shares (mode 0) the
    // hold factor is 1 - 2 * 0.25 * cos phi >= 0.5: every leg flows through stance.  In
    // Owaki's absolute form with his walking ratio (sigma * BW / omega ~ 20) the quarter load
    // gives sigma N = 5 > 1: each leg stops at cos phi = 1 / 5 and none lifts while it bears weight.
    Fix f({{"sigma_ratio", 20.0}, {"load_mode", int64_t{1}}, {"load_topic", std::string("reality.proprio.foot_load")},
           {"init_phase", std::vector<double>{1.5 * kPi, 1.5 * kPi, 1.5 * kPi, 1.5 * kPi}}});
    for (uint64_t t = 0; t < 500; ++t) {
        f.run(t, {1, 1, 1, 1}, {0.25f, 0.25f, 0.25f, 0.25f});
        for (int leg = 0; leg < 4; ++leg) ASSERT_FALSE(f.m.swing(leg)) << "tick " << t << " leg " << leg;
    }
    EXPECT_NEAR(std::cos(f.m.phase(0)), 0.2f, 0.02f) << "the fixed point cos phi = 1 / (sigma_ratio N)";
    EXPECT_FLOAT_EQ(f.out(0, 3), 0.25f) << "mode 1 publishes the load itself, not a share";
}
TEST(LoadPhaseGait, AbsoluteModeSeesTheLoadTheBellyTakesAndSharesDoNot) {
    // The feet carry only half the body (the belly the rest).  Shares renormalise it away: the
    // phases match a full-weight stance.  Absolute mode sees half the load and holds less.
    auto run = [](int64_t mode) {
        Fix f({{"sigma_ratio", 2.0}, {"load_mode", mode}, {"load_topic", std::string("reality.proprio.foot_load")},
               {"init_phase", std::vector<double>{1.5 * kPi, 1.5 * kPi, 1.5 * kPi, 1.5 * kPi}}});
        for (uint64_t t = 0; t < 10; ++t) f.run(t, {1, 1, 1, 1}, {0.125f, 0.125f, 0.125f, 0.125f});
        return f.m.phase(0);
    };
    Fix full({{"sigma_ratio", 2.0}, {"load_topic", std::string("reality.proprio.foot_load")}, {"init_phase", std::vector<double>{1.5 * kPi, 1.5 * kPi, 1.5 * kPi, 1.5 * kPi}}});
    for (uint64_t t = 0; t < 10; ++t) full.run(t, {1, 1, 1, 1}, {0.25f, 0.25f, 0.25f, 0.25f});
    EXPECT_FLOAT_EQ(run(0), full.m.phase(0)) << "shares cannot tell a belly-borne body from a leg-borne one";
    EXPECT_GT(run(1), run(0)) << "absolute: less load on the feet, less hold, the phase runs further";
}
TEST(LoadPhaseGait, AbsoluteModeWithoutALoadTopicIsTheContactFlag) {
    Fix f({{"sigma_ratio", 4.0}, {"load_mode", int64_t{1}}});
    f.run(0, {1, 0, 1, 0});
    EXPECT_FLOAT_EQ(f.out(0, 3), 1.0f); EXPECT_FLOAT_EQ(f.out(1, 3), 0.0f);
}
TEST(LoadPhaseGait, NeighbouringSixtyFourBitSeedsGiveDifferentStartPhases) {
    // OgmaBrain derives per-run seeds as namespace_seed(master, id): 64-bit values a unit apart.
    // Read through a double they collapsed to one value and every run started identically.
    auto start = [](int64_t seed) {
        ogma::InProcessBus bus; ogma::LoadPhaseGait m; m.set_id("load_phase");
        m.on_setup(&bus, {{"seed", ogma::ParamValue{seed}}});
        return m.phase(0);
    };
    const int64_t s = 8409386222832229053LL;
    EXPECT_NE(start(s), start(s + 1));
}
TEST(LoadPhaseGait, TheAdaptivePeriodSlowsWhenTheServosLagAndSpeedsUpWhenTheyKeepUp) {
    // Tegotae plan 7.30: the period moves by rate * (lag ratio - ref) * period per tick, within its bounds.
    auto mk = [](double rate) {
        return ogma::ParamMap{{"track_topic", std::string("motor.track")}, {"period_adapt_rate", rate},
                              {"period_adapt_ref", 0.3}, {"period_adapt_min", 40.0}, {"period_adapt_max", 80.0},
                              {"period_ticks", 50.0}};
    };
    auto drive = [](Fix& f, float ratio, int n) {
        for (int t = 0; t < n; ++t) {
            f.bus.begin_tick(uint64_t(t));
            f.bus.publish("motor.track", pv({ratio}));
            f.bus.publish("reality.proprio.foot_contact", pv({0, 0, 0, 0}));
            f.m.tick(uint64_t(t)); f.bus.end_tick();
        }
    };
    Fix lag(mk(0.01));
    drive(lag, 0.5f, 50);
    const double p1 = lag.m.diag_snapshot()["period_eff"].get<double>();
    EXPECT_GT(p1, 50.0) << "servos trailing: a slower stride";
    drive(lag, 0.5f, 2000);
    EXPECT_NEAR(lag.m.diag_snapshot()["period_eff"].get<double>(), 80.0, 1e-6) << "held at the upper bound";
    Fix fast(mk(0.01));
    drive(fast, 0.1f, 2000);
    EXPECT_NEAR(fast.m.diag_snapshot()["period_eff"].get<double>(), 40.0, 1e-6) << "keeping up: down to the lower bound";
    Fix off(mk(0.0));
    drive(off, 0.5f, 500);
    EXPECT_DOUBLE_EQ(off.m.diag_snapshot()["period_eff"].get<double>(), 50.0) << "rate 0: the configured period";
}
