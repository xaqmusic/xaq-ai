// =============================================================================
// test_saccade_reflex.cpp
//   SaccadeReflex — saccadic learning-walk (Pathway C1).
//
//   1. IdleUntilTravelTrigger — silent + accumulates distance until threshold.
//   2. FiresAfterTravel — crosses travel_trigger → enters pivot.
//   3. PivotIsPureRotation — left=+spin, right=−spin (common-mode 0 → spins in place).
//   4. PivotLastsPivotTicksThenRefractory — pivot N ticks, then silent refractory.
//   5. ActiveFlagTracksPivot — saccade.active = 1 only while pivoting.
//   6. DisabledStaysSilent — enable=false → never fires.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/SaccadeReflex.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::ProprioToken> vel(float vr, float vf, uint64_t t = 0) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id = t; p->sensor = "vel_ego";
    p->values.resize(2); p->values[0] = vr; p->values[1] = vf;
    return p;
}
std::shared_ptr<ogma::ProprioToken> scal(float v) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->values.resize(1); p->values[0] = v;
    return p;
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::SaccadeReflex sac;
    explicit Fixture(ogma::ParamMap const& p = {}) {
        sac.set_id("saccade");
        sac.on_setup(&bus, p);
    }
    void run_tick(uint64_t t, float speed) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.vel_ego", vel(speed, 0.0f, t));
        sac.tick(t);
        bus.end_tick();
    }
    float left()  const { return acc("saccade.left"); }
    float right() const { return acc("saccade.right"); }
    float active() const {
        auto p = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("saccade.active"));
        return (p && p->values.size() > 0) ? p->values[0] : -1.0f;
    }
    float acc(std::string const& topic) const {
        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(bus.last_value(topic));
        return a ? a->accel : 0.0f;
    }
};

}  // namespace

TEST(SaccadeReflex, IdleUntilTravelTrigger) {
    ogma::ParamMap p{{"travel_trigger", 10.0}, {"spin_rate", 4.0}};
    Fixture f(p);
    for (uint64_t t = 0; t < 5; ++t) f.run_tick(t, 1.0f);   // accumulated 5 < 10
    EXPECT_EQ(f.sac.state(), 0) << "still idle below the travel trigger";
    EXPECT_FLOAT_EQ(f.left(), 0.0f);
    EXPECT_FLOAT_EQ(f.active(), 0.0f);
}

TEST(SaccadeReflex, FiresAfterTravel) {
    ogma::ParamMap p{{"travel_trigger", 3.0}, {"pivot_ticks", int64_t{10}}, {"spin_rate", 4.0}};
    Fixture f(p);
    for (uint64_t t = 0; t < 3; ++t) f.run_tick(t, 1.0f);   // accumulated 3 ≥ 3 on tick 2
    EXPECT_EQ(f.sac.state(), 1) << "should enter pivot after travelling the trigger distance";
    EXPECT_GE(f.sac.saccade_count(), 1);
}

TEST(SaccadeReflex, PivotIsPureRotation) {
    ogma::ParamMap p{{"travel_trigger", 1.0}, {"pivot_ticks", int64_t{10}}, {"spin_rate", 4.0}};
    Fixture f(p);
    f.run_tick(0, 2.0f);   // trips trigger → pivot
    ASSERT_EQ(f.sac.state(), 1);
    EXPECT_FLOAT_EQ(f.left(),  4.0f);
    EXPECT_FLOAT_EQ(f.right(), -4.0f);
    EXPECT_NEAR(f.left() + f.right(), 0.0f, 1e-6f) << "common-mode 0 → zero forward → spins in place";
}

TEST(SaccadeReflex, PivotLastsPivotTicksThenRefractory) {
    ogma::ParamMap p{{"travel_trigger", 1.0}, {"pivot_ticks", int64_t{5}},
                     {"refractory_ticks", int64_t{4}}, {"spin_rate", 4.0}};
    Fixture f(p);
    f.run_tick(0, 2.0f);                 // tick 0 enters pivot (ticks_left=5)
    for (uint64_t t = 1; t <= 5; ++t) f.run_tick(t, 0.0f);   // pivot ticks
    // after 5 pivot ticks decremented, should be in refractory
    EXPECT_EQ(f.sac.state(), 2) << "pivot exhausted → refractory";
    EXPECT_FLOAT_EQ(f.left(), 0.0f) << "silent during refractory";
}

TEST(SaccadeReflex, ActiveFlagTracksPivot) {
    ogma::ParamMap p{{"travel_trigger", 1.0}, {"pivot_ticks", int64_t{3}}, {"spin_rate", 4.0}};
    Fixture f(p);
    f.run_tick(0, 2.0f);
    EXPECT_FLOAT_EQ(f.active(), 1.0f) << "active=1 while pivoting";
    EXPECT_TRUE(f.sac.is_pivoting());
}

TEST(SaccadeReflex, DisabledStaysSilent) {
    ogma::ParamMap p{{"enable", false}, {"travel_trigger", 1.0}, {"spin_rate", 4.0}};
    Fixture f(p);
    for (uint64_t t = 0; t < 20; ++t) f.run_tick(t, 5.0f);   // lots of travel
    EXPECT_EQ(f.sac.state(), 0);
    EXPECT_FLOAT_EQ(f.left(), 0.0f);
    EXPECT_EQ(f.sac.saccade_count(), 0);
}

// ---- Pathway B: epistemic explore = no-foraging-progress × hunger, in a novel place ----

namespace {
std::shared_ptr<ogma::RealityToken> rtok(float tle) {
    auto r = std::make_shared<ogma::RealityToken>(); r->tle = tle; return r;
}
ogma::ParamMap epi_params() {
    return {{"trigger_mode", std::string("epistemic")},
            {"scent_topic",   std::string("reality.proprio.scent_max")},
            {"hunger_topic",  std::string("reality.proprio.hunger")},
            {"novelty_topic", std::string("reality.video.color")},
            {"short_alpha", 0.3}, {"long_alpha", 0.03},
            {"progress_gate", 0.001}, {"hunger_gate", 0.3}, {"novelty_threshold", 0.3}};
}
struct EpiFixture {
    ogma::InProcessBus bus; ogma::SaccadeReflex sac;
    EpiFixture() { sac.set_id("saccade"); sac.on_setup(&bus, epi_params()); }
    void run(uint64_t t, float scent, float hunger, float novelty) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.vel_ego", vel(0.0f, 0.0f, t));
        bus.publish("reality.proprio.scent_max", scal(scent));
        bus.publish("reality.proprio.hunger", scal(hunger));
        bus.publish("reality.video.color", rtok(novelty));
        sac.tick(t); bus.end_tick();
    }
};
}  // namespace

TEST(SaccadeReflex, EpistemicFiresWhenStalledHungryNovel) {
    EpiFixture f;
    // flat scent (no progress) + hungry + novel → explore
    for (uint64_t t = 0; t < 4; ++t) f.run(t, /*scent=*/0.1f, /*hunger=*/0.9f, /*novelty=*/0.9f);
    EXPECT_EQ(f.sac.state(), 1);
    EXPECT_GE(f.sac.saccade_count(), 1);
}

TEST(SaccadeReflex, EpistemicSuppressedWhenScentRising) {
    // RISING scent (approaching food) → short>long → progress>0 → exploit. After the
    // initial transient (EMA inits flat), rising scent suppresses repeated exploring.
    auto p = epi_params();
    p["pivot_ticks"] = int64_t{2}; p["refractory_ticks"] = int64_t{2};
    ogma::InProcessBus bus; ogma::SaccadeReflex sac; sac.set_id("saccade"); sac.on_setup(&bus, p);
    for (uint64_t t = 0; t < 30; ++t) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.vel_ego", vel(0.0f, 0.0f, t));
        bus.publish("reality.proprio.scent_max", scal(0.05f * float(t)));   // rising
        bus.publish("reality.proprio.hunger", scal(0.9f));
        bus.publish("reality.video.color", rtok(0.9f));
        sac.tick(t); bus.end_tick();
    }
    EXPECT_GT(sac.last_progress(), 0.001f) << "scent should be rising";
    EXPECT_LE(sac.saccade_count(), 1) << "rising scent → no repeated exploring (only the initial transient)";
}

TEST(SaccadeReflex, EpistemicSuppressedWhenSated) {
    EpiFixture f;
    for (uint64_t t = 0; t < 4; ++t) f.run(t, /*scent=*/0.1f, /*hunger=*/0.0f, /*novelty=*/0.9f);
    EXPECT_EQ(f.sac.state(), 0) << "sated → no explore even if stalled + novel";
}

TEST(SaccadeReflex, EpistemicSuppressedWhenFamiliar) {
    EpiFixture f;
    for (uint64_t t = 0; t < 4; ++t) f.run(t, /*scent=*/0.1f, /*hunger=*/0.9f, /*novelty=*/0.1f);
    EXPECT_EQ(f.sac.state(), 0) << "familiar place → no explore";
}
