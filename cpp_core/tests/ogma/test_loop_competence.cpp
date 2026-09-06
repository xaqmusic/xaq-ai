// LoopCompetence (Cell round 4, register O21): competence = the fraction of driving windows on
// which the loop's objective moved as the loop predicts; relaxes to the prior when not driving.
#include <gtest/gtest.h>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/LoopCompetence.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> p1(float v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p; }
ogma::ParamMap params() {
    return {{"objective_topic", std::string("reality.proprio.scent_max")}, {"gain_topic", std::string("arbiter.gain.klino")},
            {"modality_name", std::string("klino")}, {"horizon_ticks", int64_t{10}}, {"alpha", 0.2}, {"forget", 0.01}};
}
struct Fix {
    ogma::InProcessBus bus; ogma::LoopCompetence m;
    Fix() { m.set_id("comp_klino"); m.on_setup(&bus, params()); }
    void run(uint64_t t, float obj, float gain) {
        bus.begin_tick(t); bus.publish("reality.proprio.scent_max", p1(obj)); bus.publish("arbiter.gain.klino", p1(gain)); m.tick(t); bus.end_tick();
    }
    std::shared_ptr<const ogma::RealityToken> tok() const {
        return std::dynamic_pointer_cast<const ogma::RealityToken>(bus.last_value("reality.loop.klino"));
    }
};
}

TEST(LoopCompetence, CompetentWhenTheObjectiveRisesWhileDriving) {
    Fix f; uint64_t t = 0; float o = 0.1f;
    for (int i = 0; i < 200; ++i) { o += 0.01f; f.run(t++, o, 1.0f); }
    EXPECT_GT(f.m.checks(), 15u);
    EXPECT_EQ(f.m.improvements(), f.m.checks());
    EXPECT_GT(f.m.competence(), 0.95f);
    auto tk = f.tok(); ASSERT_TRUE(tk);
    EXPECT_NEAR(tk->expected_error, 1.0f - f.m.competence(), 1e-6f);
    EXPECT_EQ(tk->latent.size(), 1); EXPECT_EQ(tk->baked_count, 3); EXPECT_EQ(tk->winner_id, 0);
}

TEST(LoopCompetence, IncompetentWhenTheObjectiveFallsAndUncertainWhenIdle) {
    Fix f; uint64_t t = 0; float o = 2.0f;
    for (int i = 0; i < 200; ++i) { o -= 0.01f; f.run(t++, o, 1.0f); }
    EXPECT_LT(f.m.competence(), 0.05f);
    // not driving: no checks, the competence relaxes toward 0.5
    auto checks = f.m.checks();
    for (int i = 0; i < 300; ++i) f.run(t++, o, 0.0f);
    EXPECT_EQ(f.m.checks(), checks);
    EXPECT_GT(f.m.competence(), 0.4f); EXPECT_LT(f.m.competence(), 0.5f);
}

TEST(LoopCompetence, StagnationCountsAsNotImproved) {
    Fix f; uint64_t t = 0;
    for (int i = 0; i < 200; ++i) f.run(t++, 0.3f, 1.0f);   // drives, nothing changes
    EXPECT_GT(f.m.checks(), 15u); EXPECT_EQ(f.m.improvements(), 0u);
    EXPECT_LT(f.m.competence(), 0.05f) << "a policy whose world does not move is not competent (no low-error-by-constancy trap)";
}

TEST(LoopCompetence, WindowRestartsWhenDrivingResumes) {
    Fix f; uint64_t t = 0;
    for (int i = 0; i < 5; ++i) f.run(t++, 0.1f, 1.0f);      // partial window
    for (int i = 0; i < 5; ++i) f.run(t++, 5.0f, 0.0f);      // idle: a jump while idle must not count
    for (int i = 0; i < 9; ++i) f.run(t++, 5.0f, 1.0f);      // 9 driving ticks: no completed window yet
    EXPECT_EQ(f.m.checks(), 0u);
    f.run(t++, 5.0f, 1.0f);                                  // the 10th completes a window: 5.0 → 5.0, not improved
    EXPECT_EQ(f.m.checks(), 1u); EXPECT_EQ(f.m.improvements(), 0u);
}
