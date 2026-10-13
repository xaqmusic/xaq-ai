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
