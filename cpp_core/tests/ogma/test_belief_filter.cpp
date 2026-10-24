// BeliefFilter: predict-then-correct over an EPM's nodes (design note generator_and_belief_filter.md §3.2).
#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <nlohmann/json.hpp>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/BeliefFilter.hpp"

namespace {
std::shared_ptr<ogma::RealityToken> rt(int w, std::vector<int> pruned = {}) {
    auto t = std::make_shared<ogma::RealityToken>(); t->winner_id = w; t->pruned_ids = std::move(pruned); return t;
}
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)];
    return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::BeliefFilter f;
    explicit Fix(ogma::ParamMap p = {}) {
        p["reality_topic"] = std::string("reality.t.s"); p["obs_topic"] = std::string("obs.t");
        f.set_id("bf"); f.on_setup(&bus, p);
    }
    void step(uint64_t t, int w, float x, bool with_obs = true) {
        bus.begin_tick(t);
        if (with_obs) { bus.publish("reality.t.s", rt(w)); bus.publish("obs.t", pv({x})); }
        f.tick(t); bus.end_tick();
    }
};
}  // namespace

TEST(BeliefFilter, ItLearnsTheStatesAndTracksThem) {
    Fix fx;
    for (uint64_t t = 0; t < 2000; ++t) { const int k = (t / 50) % 2; fx.step(t, k, k ? 1.0f : -1.0f); }
    EXPECT_GT(fx.f.map_prob(), 0.95);
    EXPECT_NEAR(fx.f.estimate()[0], (1999 / 50) % 2 ? 1.0f : -1.0f, 0.05f);
}

TEST(BeliefFilter, WithNoReadingTheBeliefSpreadsInsteadOfFreezing) {
    Fix fx;
    for (uint64_t t = 0; t < 3000; ++t) { const int k = (t / 25) % 2; fx.step(t, k, k ? 1.0f : -1.0f); }
    const double h0 = fx.f.entropy();
    for (uint64_t t = 3000; t < 3200; ++t) fx.step(t, 0, 0.0f, /*with_obs=*/false);
    EXPECT_GT(fx.f.entropy(), h0 + 0.3) << "200 silent ticks: the belief must spread toward what the transitions allow";
    EXPECT_LT(std::fabs(fx.f.estimate()[0]), 0.5f) << "and the estimate moves toward the mixture, not the stale state";
}

TEST(BeliefFilter, TheTransitionsCarryItThroughAnAmbiguousReading) {
    // A sticky two-state world: one reading half-way between the states must not flip the belief.
    Fix fx;
    std::mt19937 rng(1); std::normal_distribution<float> n(0.0f, 0.2f);
    int k = 0;
    for (uint64_t t = 0; t < 4000; ++t) { if (t % 200 == 0) k = 1 - k; fx.step(t, k, (k ? 1.0f : -1.0f) + n(rng)); }
    const int before = fx.f.map_node();
    fx.step(4000, before, 0.0f);           // the EPM must pick a winner; the reading itself is ambiguous
    EXPECT_EQ(fx.f.map_node(), before) << "one ambiguous reading does not overturn a sticky state";
}

TEST(BeliefFilter, APrunedNodeLeavesAndTheBeliefStaysNormalised) {
    Fix fx;
    for (uint64_t t = 0; t < 500; ++t) fx.step(t, (t / 20) % 3, float((t / 20) % 3));
    ASSERT_EQ(fx.f.nodes(), 3);
    fx.bus.begin_tick(500); fx.bus.publish("reality.t.s", rt(0, {2})); fx.bus.publish("obs.t", pv({0.0f})); fx.f.tick(500); fx.bus.end_tick();
    EXPECT_EQ(fx.f.nodes(), 2);
    EXPECT_NEAR(fx.f.belief_of(0) + fx.f.belief_of(1), 1.0, 1e-6);
}
