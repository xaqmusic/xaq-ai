// BeliefFilter: predict-then-correct over an EPM's nodes (design note generator_and_belief_filter.md §3.2).
#include <gtest/gtest.h>
#include <iostream>
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

TEST(BeliefFilter, ItsInnovationIsConsistent_NisNearOneOnAStreamItsModelDescribes) {
    // two sticky states, Gaussian readings (sigma 0.3), the EPM's label the true state: an honest filter's NIS averages ~1
    Fix fx; std::mt19937 rng(7); std::normal_distribution<float> n(0.0f, 0.3f);
    int k = 0;
    for (uint64_t t = 0; t < 20000; ++t) { if (t % 300 == 0) k = 1 - k; fx.step(t, k, (k ? 1.0f : -1.0f) + n(rng)); }
    EXPECT_NEAR(fx.f.nis_mean(), 1.0, 0.15);
    const auto d = fx.f.diag_snapshot();
    EXPECT_GT(d["nis_n"].get<int64_t>(), 19000); EXPECT_GT(d["winner_switches"].get<int64_t>(), 60);
    EXPECT_LE(d["map_switches"].get<int64_t>(), d["winner_switches"].get<int64_t>() + 5) << "the belief does not switch more than its evidence";
}

TEST(BeliefFilter, ASnapshotRestoresTheLearnedModel_TheNextTickIsTheSame) {
    Fix a; std::mt19937 rng(3); std::normal_distribution<float> n(0.0f, 0.2f);
    int k = 0;
    for (uint64_t t = 0; t < 3000; ++t) { if (t % 150 == 0) k = 1 - k; a.step(t, k, (k ? 1.0f : -1.0f) + n(rng)); }
    const auto snap = a.f.snapshot_state();
    Fix b; b.f.restore_state(snap);
    EXPECT_EQ(b.f.nodes(), a.f.nodes());
    a.step(3000, k, 0.1f); b.step(3000, k, 0.1f);
    EXPECT_EQ(a.f.map_node(), b.f.map_node()); EXPECT_NEAR(a.f.map_prob(), b.f.map_prob(), 1e-6); EXPECT_NEAR(a.f.estimate()[0], b.f.estimate()[0], 1e-5);
    EXPECT_THROW(b.f.restore_state(nlohmann::json{{"version", 9}}), std::invalid_argument);
}

TEST(BeliefFilter, WithManyThinlySampledStatesTheOwnHeadsAreConsistentWhereTheLegacyHeadsAreNot) {
    // 40 states on a line, sticky runs of 10 ticks, readings at the state +- 0.05: each node sees ~50 samples in 2 000 ticks. The decode
    // heads' own NIS (the winner's head against the reading, before it learns it): the legacy heads keep their global-variance prior and
    // move at most gain_cap a sample (NIS << 1); heads built from their own samples state an honest spread (NIS ~ 1).
    auto run = [](const char* mode) {
        // the variance floor (1 % of the global variance: the states span 0..19.5, a floor spread of 0.57 against a true 0.05) is set
        // negligible, so the heads themselves are measured
        Fix fx({{"head_update", ogma::ParamValue{std::string(mode)}}, {"var_rel_floor", ogma::ParamValue{1e-6}}}); std::mt19937 rng(11); std::normal_distribution<float> n(0.0f, 0.05f);
        std::uniform_int_distribution<int> pick(0, 39); int k = 0;
        for (uint64_t t = 0; t < 20000; ++t) { if (t % 10 == 0) k = pick(rng); fx.step(t, k, float(k) * 0.5f + n(rng)); }
        return fx.f.head_nis_mean();
    };
    const double legacy = run("legacy"), own = run("own");
    EXPECT_LT(legacy, 0.5) << "the legacy heads are underconfident on thinly sampled states";
    EXPECT_NEAR(own, 1.0, 0.2) << "heads from their own samples state an honest spread";
    EXPECT_THROW(Fix({{"head_update", ogma::ParamValue{std::string("bogus")}}}), std::invalid_argument);
}

TEST(BeliefFilter, WhenEachVisitDwellsTheVisitHeadsAreConsistentWhereTheOwnHeadsAreOverconfident) {
    // 40 states on a line, each visit a DWELL of 30 ticks (the stance stream at the tick rate): a visit sits at the state plus its
    // own offset (sd 0.05), each tick at that plus a small noise (sd 0.01). The own heads count ticks: the first visit alone passes
    // min_count, and with ~15 visits a state in 18 000 ticks its spread is an estimate from a few independent visits read as known.
    // The visit heads count visits, read the between-visit spread as the Student-t predictive of n estimated visits and add the
    // pooled within-visit spread: a tick in a new visit is predicted honestly.
    auto run = [](const char* mode, uint64_t TICKS) {
        Fix fx({{"head_update", ogma::ParamValue{std::string(mode)}}, {"var_rel_floor", ogma::ParamValue{1e-6}}}); std::mt19937 rng(7);
        std::normal_distribution<float> off(0.0f, 0.05f), tick(0.0f, 0.01f); std::uniform_int_distribution<int> pick(0, 39); int k = 0; float o = 0.0f;
        for (uint64_t t = 0; t < TICKS; ++t) { if (t % 30 == 0) { k = pick(rng); o = off(rng); } fx.step(t, k, float(k) * 0.5f + o + tick(rng)); }
        return fx.f.head_nis_mean();
    };
    const double own = run("own", 18000), visits = run("visits", 18000);
    EXPECT_GT(own, 1.4) << "a head that counts ticks states the spread of an estimate from a few visits as if it were known";
    EXPECT_NEAR(visits, 1.0, 0.2) << "counting visits, and reading the spread as the predictive of n estimated visits";
}

TEST(BeliefFilter, TheVisitHeadsRoundTripThroughASnapshot) {
    Fix a({{"head_update", ogma::ParamValue{std::string("visits")}}}); std::mt19937 rng(3); std::normal_distribution<float> n(0.0f, 0.05f);
    for (uint64_t t = 0; t < 3000; ++t) a.step(t, int(t / 20) % 6, float(int(t / 20) % 6) + n(rng));
    const auto s = a.f.snapshot_state(); ASSERT_TRUE(s.contains("wss"));
    Fix b({{"head_update", ogma::ParamValue{std::string("visits")}}}); b.f.restore_state(s);
    EXPECT_EQ(b.f.snapshot_state(), s);
    Fix l; for (uint64_t t = 0; t < 300; ++t) l.step(t, int(t / 20) % 6, float(int(t / 20) % 6));
    EXPECT_FALSE(l.f.snapshot_state().contains("wss")) << "legacy snapshots unchanged";
}
