// =============================================================================
// test_scent_homing_learner.cpp
//   ScentHomingLearner — learned scent-homing (Pathway A, action-consequence).
//
//   1. GrowsPrototypes — distinct rings → VQ grows prototypes.
//   2. NoScentEmitsZero — ring below signal_floor → [0,0].
//   3. ForwardSectorIsForward — first (cold) commit = sector 0 = [0,1].
//   4. UnitHeadingWhenScentPresent — output is a unit bearing.
//   5. ProgressReinforces / FlatDoesNot — rising scent_max → V grows; flat → ~0.
//   6. HitTeleportCreditsPositive — mid-window scent collapse credits +r_hit.
//   7. ShuffleExplores — shuffle ablation always flags an explore pick.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ScentHomingLearner.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::ProprioToken> tok(std::vector<float> v, uint64_t t = 0) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id = t;
    p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[i];
    return p;
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::ScentHomingLearner shl;
    explicit Fixture(ogma::ParamMap const& p = {}) {
        shl.set_id("scent_homing");
        shl.on_setup(&bus, p);
    }
    void run_tick(uint64_t t, std::vector<float> ring, float smax) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent", tok(ring, t));
        bus.publish("reality.proprio.scent_max", tok({smax}, t));
        shl.tick(t);
        bus.end_tick();
    }
    std::shared_ptr<const ogma::ProprioToken> out() const {
        return std::dynamic_pointer_cast<const ogma::ProprioToken>(
            bus.last_value("percept.scent_homing"));
    }
};

const std::vector<float> R1 = {1,0,0,0,0,0,0,0};
const std::vector<float> R2 = {0,0,1,0,0,0,0,0};
const std::vector<float> R3 = {0,0,0,0,1,0,0,0};

}  // namespace

TEST(ScentHomingLearner, GrowsPrototypes) {
    Fixture f;
    f.run_tick(0, R1, 0.5f);
    f.run_tick(1, R2, 0.5f);
    f.run_tick(2, R3, 0.5f);
    EXPECT_GE(f.shl.n_prototypes(), 2) << "well-separated rings should grow prototypes";
}

TEST(ScentHomingLearner, NoScentEmitsZero) {
    Fixture f;
    f.run_tick(0, std::vector<float>(8, 0.0f), 0.0f);
    auto o = f.out();
    ASSERT_NE(o, nullptr);
    EXPECT_FLOAT_EQ(o->values[0], 0.0f);
    EXPECT_FLOAT_EQ(o->values[1], 0.0f);
}

TEST(ScentHomingLearner, ForwardSectorIsForward) {
    // argmax, no epistemic noise → cold start picks sector 0 = straight ahead.
    ogma::ParamMap p{{"temperature", 0.0}, {"epistemic_gain", 0.0}};
    Fixture f(p);
    f.run_tick(0, R1, 0.5f);
    auto o = f.out();
    ASSERT_NE(o, nullptr);
    EXPECT_NEAR(o->values[0], 0.0f, 1e-5f);   // hx = sin(0)
    EXPECT_NEAR(o->values[1], 1.0f, 1e-5f);   // hy = cos(0)
}

TEST(ScentHomingLearner, UnitHeadingWhenScentPresent) {
    Fixture f;
    f.run_tick(0, R1, 0.5f);
    auto o = f.out();
    ASSERT_NE(o, nullptr);
    float mag = std::sqrt(o->values[0]*o->values[0] + o->values[1]*o->values[1]);
    EXPECT_NEAR(mag, 1.0f, 1e-4f);
}

TEST(ScentHomingLearner, ProgressReinforcesAndFlatDoesNot) {
    ogma::ParamMap p{{"temperature", 0.0}, {"epistemic_gain", 0.0}, {"commit_ticks", int64_t{4}}};
    Fixture rising(p), flat(p);
    for (uint64_t t = 0; t < 40; ++t) {
        rising.run_tick(t, R1, 0.02f * float(t));   // scent_max climbs
        flat.run_tick(t, R1, 0.5f);                 // scent_max constant
    }
    EXPECT_GT(rising.shl.last_value_max(), 0.03f) << "rising scent should reinforce the committed heading";
    EXPECT_NEAR(flat.shl.last_value_max(), 0.0f, 1e-3f) << "flat scent → no progress credit";
}

TEST(ScentHomingLearner, HitTeleportCreditsPositive) {
    ogma::ParamMap p{{"temperature", 0.0}, {"epistemic_gain", 0.0},
                     {"commit_ticks", int64_t{4}}, {"hit_drop_thresh", 0.2}, {"r_hit", 1.0}};
    Fixture f(p);
    // window: high scent, then a one-tick collapse (the eat+respawn) mid-window.
    f.run_tick(0, R1, 0.9f);   // start_commit, window_start = 0.9
    f.run_tick(1, R1, 0.9f);
    f.run_tick(2, R1, 0.0f);   // drop 0.9 > 0.2 → hit_in_window
    f.run_tick(3, R1, 0.0f);
    f.run_tick(4, R1, 0.0f);   // window ends → credit
    EXPECT_NEAR(f.shl.last_win_progress(), 1.0f, 1e-4f)
        << "a hit-teleport collapse must credit +r_hit, not the negative raw Δ";
}

TEST(ScentHomingLearner, BootstrapFollowsCompassPreLesionIgnoresPost) {
    const std::vector<float> ring = {0.1f,0.2f,0.3f,0.2f,0.1f,0.05f,0.05f,0.1f};
    // PRE-lesion: compass says RIGHT [1,0]; zero V would give forward — bug must follow compass.
    {
        ogma::InProcessBus bus; ogma::ScentHomingLearner shl; shl.set_id("shl");
        shl.on_setup(&bus, {{"bootstrap_topic", std::string("percept.scent_compass")},
                            {"lesion_after_ticks", int64_t{10000}}, {"commit_ticks", int64_t{2}}});
        for (uint64_t t = 0; t < 4; ++t) {
            bus.begin_tick(t);
            bus.publish("reality.proprio.scent", tok(ring, t));
            bus.publish("reality.proprio.scent_max", tok({0.3f}, t));
            bus.publish("percept.scent_compass", tok({1.0f, 0.0f}, t));   // RIGHT
            shl.tick(t); bus.end_tick();
        }
        EXPECT_FALSE(shl.lesioned());
        auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("percept.scent_homing"));
        ASSERT_NE(o, nullptr);
        EXPECT_GT(o->values[0], 0.7f) << "follows the compass (RIGHT), not zero-V forward";
    }
    // POST-lesion: lesion immediately; zero V + argmax (temp 0) → forward, compass IGNORED.
    {
        ogma::InProcessBus bus; ogma::ScentHomingLearner shl; shl.set_id("shl");
        shl.on_setup(&bus, {{"bootstrap_topic", std::string("percept.scent_compass")},
                            {"lesion_after_ticks", int64_t{0}}, {"commit_ticks", int64_t{2}},
                            {"temperature", 0.0}, {"epistemic_gain", 0.0}});
        for (uint64_t t = 0; t < 4; ++t) {
            bus.begin_tick(t);
            bus.publish("reality.proprio.scent", tok(ring, t));
            bus.publish("reality.proprio.scent_max", tok({0.3f}, t));
            bus.publish("percept.scent_compass", tok({1.0f, 0.0f}, t));   // RIGHT — should be ignored
            shl.tick(t); bus.end_tick();
        }
        EXPECT_TRUE(shl.lesioned());
        auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("percept.scent_homing"));
        ASSERT_NE(o, nullptr);
        EXPECT_LT(std::abs(o->values[0]), 0.3f) << "lesioned → argmax of zero V = forward, compass ignored";
    }
}

TEST(ScentHomingLearner, ShuffleExplores) {
    ogma::ParamMap p{{"shuffle", true}};
    Fixture f(p);
    f.run_tick(0, R1, 0.5f);
    EXPECT_TRUE(f.shl.last_explore_pick()) << "shuffle ablation always flags an explore pick";
}
