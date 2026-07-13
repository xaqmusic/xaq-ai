// GradientEPM — meta-EPM scalar-gradient follower. The key property: from a forward
// model over (consensus ⊕ trend ⊕ heading) → Δscalar, it learns to SELECT the heading
// that makes the scalar rise (directed climb, not random reorient).
#include <gtest/gtest.h>
#include "ogma/modules/GradientEPM.hpp"
#include "ogma/InProcessBus.hpp"

#include <Eigen/Core>
#include <cmath>
#include <memory>

namespace {
std::shared_ptr<ogma::ProprioToken> p1(float v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p;
}
Eigen::VectorXf cons4() { Eigen::VectorXf c(4); c << 1.0f, 0.0f, -0.5f, 0.3f; return c; }

struct Fix {
    ogma::InProcessBus bus; ogma::GradientEPM g;
    Fix(ogma::ParamMap p = {}) { g.set_id("g"); g.on_setup(&bus, p); }
    void step(uint64_t t, float scalar, float heading = 0.0f) {
        bus.begin_tick(t);
        auto ct = std::make_shared<ogma::ConsensusToken>(); ct->fused_embedding = cons4();
        bus.publish("consensus.scent_loop", ct);
        bus.publish("reality.proprio.scent_max", p1(scalar));
        bus.publish("reality.proprio.heading", p1(heading));
        g.tick(t); bus.end_tick();
    }
};
}  // namespace

TEST(GradientEPM, PublishesUnitHeading) {
    Fix f;
    f.step(0, 1.0f);
    auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(f.bus.last_value("percept.gradient_heading"));
    ASSERT_NE(o, nullptr);
    ASSERT_EQ(o->values.size(), 2u);
    float mag = std::hypot(float(o->values[0]), float(o->values[1]));
    EXPECT_NEAR(mag, 1.0f, 1e-3f);
}

TEST(GradientEPM, LearnsToFollowRisingDirection) {
    // The scalar RISES only when the committed heading faces forward (cos h > 0.5).
    // A directed follower must learn to select such a heading.
    Fix f({{"cycle_ticks", int64_t{2}}, {"temperature", 0.05}, {"epistemic_gain", 0.1},
           {"n_headings", int64_t{8}}, {"insertion_dist", 0.3}, {"bake_visits", int64_t{2}},
           {"bake_tle", 2.0}, {"reward_norm", false}});
    float s = 5.0f; uint64_t t = 0;
    f.step(t++, s);                                   // prime the first commit
    for (int c = 0; c < 400; ++c) {
        float h = f.g.chosen_heading();
        float d = (std::cos(h) > 0.5f) ? +0.3f : -0.3f;  // forward-ish climbs
        for (int k = 0; k < 2; ++k) { s += d; f.step(t++, s); }
    }
    EXPECT_GT(f.g.node_count(), 1);
    EXPECT_GT(std::cos(f.g.chosen_heading()), 0.3f)
        << "should have learned to face the rising direction (cos h>0.5 region)";
}

TEST(GradientEPM, FleeModeAvoidsRising) {
    // mode = -1 (flee): with the SAME reward structure it should pick the FALLING side.
    Fix f({{"cycle_ticks", int64_t{2}}, {"temperature", 0.05}, {"epistemic_gain", 0.1},
           {"n_headings", int64_t{8}}, {"insertion_dist", 0.3}, {"bake_visits", int64_t{2}},
           {"bake_tle", 2.0}, {"reward_norm", false}, {"mode", int64_t{-1}}});
    float s = 5.0f; uint64_t t = 0;
    f.step(t++, s);
    for (int c = 0; c < 400; ++c) {
        float h = f.g.chosen_heading();
        float d = (std::cos(h) > 0.5f) ? +0.3f : -0.3f;
        for (int k = 0; k < 2; ++k) { s += d; f.step(t++, s); }
    }
    EXPECT_LT(std::cos(f.g.chosen_heading()), 0.5f) << "flee should avoid the rising direction";
}
