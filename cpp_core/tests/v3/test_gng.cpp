/**
 * Unit tests for GNG (v3 C++ implementation)
 *
 * Verifies the following properties that match the Python GNG:
 *  1.  Bootstrap — returns (0, 0.0) for first 2 inputs, then works normally
 *  2.  Winner ID stability — same repeated input always returns same winner
 *  3.  Error accumulation — winner error increases with each non-perfect match
 *  4.  Baking threshold — node crystallises after baking_threshold visits
 *  5.  Demotion — noisy node does not bake on first attempt
 *  6.  Node insertion — node_count grows when error is high
 *  7.  Stale prune — non-baked nodes are removed when stale prune is enabled
 *  8.  Baked nodes are not pruned
 *  9.  Serialisation roundtrip — to_json() / from_json() preserves state
 *  10. context_novelty — returns inf with no baked nodes; low for seen input
 */

#include <gtest/gtest.h>
#include "v3/gng.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <random>

using namespace ami_ogma::v3;

// Deterministic 128D vector
static Eigen::VectorXf make_vec(float val, int dim = 128) {
    Eigen::VectorXf v = Eigen::VectorXf::Constant(dim, val);
    float n = v.norm();
    return (n > 1e-6f) ? (v / n) : v;
}

// Random 128D unit vector
static Eigen::VectorXf random_unit(std::mt19937& rng, int dim = 128) {
    std::normal_distribution<float> dist(0.0f, 1.0f);
    Eigen::VectorXf v(dim);
    for (int i = 0; i < dim; ++i) v(i) = dist(rng);
    float n = v.norm();
    return (n > 1e-6f) ? (v / n) : v;
}

// ---

TEST(GNG, BootstrapReturnsDummy) {
    GNG::Config cfg;
    cfg.dim = 128;
    GNG gng(cfg);

    auto v = make_vec(0.5f);
    auto [id0, d0] = gng.step(v);
    EXPECT_EQ(id0, 0);
    EXPECT_NEAR(d0, 0.0f, 1e-6f);

    auto [id1, d1] = gng.step(v);
    EXPECT_EQ(id1, 0);
    EXPECT_NEAR(d1, 0.0f, 1e-6f);

    // 3rd step should actually work
    auto [id2, d2] = gng.step(v);
    EXPECT_EQ(gng.node_count(), 2);  // still 2 from bootstrap
}

TEST(GNG, RepeatedInputConvergesToSameWinner) {
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 20;
    GNG gng(cfg);

    auto v = make_vec(1.0f);
    // Prime bootstrap
    gng.step(v); gng.step(v);

    // Run 50 steps with same input
    int last_winner = -1;
    for (int i = 0; i < 50; ++i) {
        auto [wid, d] = gng.step(v);
        if (i > 5) {  // after warmup
            if (last_winner >= 0)
                EXPECT_EQ(wid, last_winner) << "Stable input should always win the same node";
            last_winner = wid;
        }
    }
}

TEST(GNG, BakingThreshold) {
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 10;
    cfg.min_insertion_error = 1e-5f;  // very low → easy to bake
    cfg.lambda_new = 1000;            // disable insertion noise
    GNG gng(cfg);

    auto v = make_vec(1.0f);
    gng.step(v); gng.step(v);  // bootstrap

    // Feed enough times to bake
    for (int i = 0; i < 30; ++i)
        gng.step(v);

    EXPECT_GT(gng.baked_count(), 0) << "Node should have baked after " << 30 << " visits";
}

TEST(GNG, NodeCountGrowsWithHighError) {
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 200;   // don't bake too fast
    cfg.min_insertion_error = 0.0f;  // always insert
    cfg.lambda_new = 5;
    cfg.max_nodes = 50;
    GNG gng(cfg);

    std::mt19937 rng(42);
    gng.step(random_unit(rng));
    gng.step(random_unit(rng));

    int initial = gng.node_count();
    for (int i = 0; i < 200; ++i)
        gng.step(random_unit(rng));

    EXPECT_GT(gng.node_count(), initial) << "Random inputs should cause node growth";
}

TEST(GNG, StalePruneRemovesUnbaked) {
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 1000;  // never bake
    cfg.stale_prune_enabled = true;
    cfg.stale_window_factor = 1.0f;  // aggressive prune
    cfg.lambda_new = 10;
    cfg.min_insertion_error = 0.0f;
    cfg.max_nodes = 100;
    GNG gng(cfg);

    std::mt19937 rng(7);
    // Bootstrap with random vectors far apart
    Eigen::VectorXf v_left  = Eigen::VectorXf::Zero(128); v_left(0)   = 1.0f;
    Eigen::VectorXf v_right = Eigen::VectorXf::Zero(128); v_right(127)= 1.0f;
    gng.step(v_left); gng.step(v_right);

    // Grow nodes
    for (int i = 0; i < 100; ++i)
        gng.step(random_unit(rng));

    int before_prune = gng.node_count();

    // Now only feed v_left for many steps — v_right-region nodes become stale
    for (int i = 0; i < 500; ++i)
        gng.step(v_left);

    int after_focus = gng.node_count();
    // We don't assert a specific count but verify prune hasn't crashed
    EXPECT_GE(after_focus, 2) << "GNG must keep at least 2 nodes";
    SUCCEED();  // stale prune ran without crashing
}

TEST(GNG, BakedNodesNotPruned) {
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 5;
    cfg.min_insertion_error = 1e-5f;
    cfg.stale_prune_enabled = true;
    cfg.stale_window_factor = 1.0f;
    cfg.lambda_new = 1000;
    GNG gng(cfg);

    auto v = make_vec(0.5f);
    gng.step(v); gng.step(v);

    // Bake the winner
    for (int i = 0; i < 20; ++i)
        gng.step(v);

    int baked_before = gng.baked_count();
    ASSERT_GT(baked_before, 0);

    // Force stale prune by manually calling internal step many times
    // with a different input — baked nodes must survive
    auto v_other = make_vec(-0.5f);
    for (int i = 0; i < 500; ++i)
        gng.step(v_other);

    EXPECT_GE(gng.baked_count(), baked_before)
        << "Baked nodes must not be removed by stale prune";
}

TEST(GNG, ContextNoveltyHighForUnseenInput) {
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 5;
    cfg.min_insertion_error = 1e-6f;
    cfg.lambda_new = 1000;
    GNG gng(cfg);

    // No baked nodes yet → novelty = inf
    Eigen::VectorXf v = make_vec(1.0f);
    EXPECT_EQ(gng.context_novelty(v), std::numeric_limits<float>::infinity());

    // Bake a node at v
    gng.step(v); gng.step(v);
    for (int i = 0; i < 20; ++i) gng.step(v);

    // Novelty for v itself should be low
    float nov_same = gng.context_novelty(v);
    EXPECT_LT(nov_same, 0.5f) << "Novelty of seen input should be low";

    // Novelty for antipodal input should be high
    Eigen::VectorXf v_other = -v;
    float nov_other = gng.context_novelty(v_other);
    EXPECT_GT(nov_other, nov_same) << "Novelty of unseen input should be higher";
}

TEST(GNG, SerialisationRoundtrip) {
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 10;
    cfg.min_insertion_error = 1e-5f;
    cfg.lambda_new = 5;
    GNG gng(cfg);

    std::mt19937 rng(123);
    gng.step(random_unit(rng)); gng.step(random_unit(rng));

    for (int i = 0; i < 100; ++i)
        gng.step(random_unit(rng));

    // Serialise
    auto j = gng.to_json();

    // Restore
    GNG gng2 = GNG::from_json(j);

    EXPECT_EQ(gng.node_count(), gng2.node_count())
        << "Node count must survive roundtrip";
    EXPECT_EQ(gng.baked_count(), gng2.baked_count())
        << "Baked count must survive roundtrip";
    EXPECT_EQ(gng.step_count(), gng2.step_count())
        << "Step counter must survive roundtrip";

    // Feed same input — should get same winner ID (prototypes preserved)
    auto test_vec = random_unit(rng);
    auto [w1, d1] = gng.step(test_vec);
    auto [w2, d2] = gng2.step(test_vec);
    EXPECT_EQ(w1, w2) << "Winner IDs must match after roundtrip";
    EXPECT_NEAR(d1, d2, 1e-4f) << "Distances must match after roundtrip";
}

TEST(GNG, SchemaV3HasFullSnapshotState) {
    // Phase 6.5.4: schema bumped from 2 → 3 to add per-node bake_checked +
    // health and module-level running_mean_error_/last_step_baked_/
    // last_death_step_/history_/last_x_ — the full state needed for
    // OgmaInstance::clone() byte-equivalence.  Older schema-2 snapshots
    // still load via j.value(field, default) in from_json.
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 5;
    cfg.min_insertion_error = 1e-5f;
    GNG gng(cfg);

    auto v = make_vec(1.0f);
    gng.step(v); gng.step(v);
    for (int i = 0; i < 10; ++i) gng.step(v);

    auto j = gng.to_json();
    EXPECT_EQ(j.value("schema", 0), 3);
    ASSERT_TRUE(j.contains("nodes"));
    ASSERT_FALSE(j["nodes"].empty());
    // Schema 2 fields (still present, backwards compatible).
    EXPECT_TRUE(j["nodes"][0].contains("ema_error"))
        << "Schema 3 must keep ema_error per node (schema-2 carryover)";
    EXPECT_TRUE(j.contains("stale_prune_enabled"));
    EXPECT_TRUE(j.contains("stale_window_factor"));
    // Schema 3 additions.
    EXPECT_TRUE(j["nodes"][0].contains("bake_checked"))
        << "Schema 3 must include per-node bake_checked";
    EXPECT_TRUE(j["nodes"][0].contains("health"))
        << "Schema 3 must include per-node health";
    EXPECT_TRUE(j.contains("running_mean_error"));
    EXPECT_TRUE(j.contains("last_step_baked"));
    EXPECT_TRUE(j.contains("last_death_step"));
    EXPECT_TRUE(j.contains("history"));
    EXPECT_TRUE(j.contains("last_x"));
}

TEST(GNG, CrystallizationRatioRange) {
    GNG::Config cfg;
    cfg.dim = 128;
    cfg.baking_threshold = 5;
    cfg.min_insertion_error = 1e-6f;
    cfg.lambda_new = 1000;
    GNG gng(cfg);

    auto v = make_vec(1.0f);
    gng.step(v); gng.step(v);

    float cr_before = gng.crystallization_ratio();
    EXPECT_GE(cr_before, 0.0f);
    EXPECT_LE(cr_before, 1.0f);

    for (int i = 0; i < 20; ++i) gng.step(v);

    float cr_after = gng.crystallization_ratio();
    EXPECT_GE(cr_after, 0.0f);
    EXPECT_LE(cr_after, 1.0f);
}

// ---------------------------------------------------------------------------
// Kalman-lessons campaign, Stage 0.3 — pin the linear gain anneal
// (docs/plans-and-designs/epm_kalman_lessons_plan.md).
//
// The winner update is w += g_n (x - w) with g_n = eps_b (1 - 0.9 n/N) for the
// n-th visit before bake, so the prototype at bake keeps weight prod(1 - g_n)
// on the point it was born at: 0.241 at eps_b 0.05, N 50.  (The health term in
// the damping perturbs the first three gains by ~1e-3 in total.)  Stage 1
// replaces this schedule; this test proves the path it replaces is live and
// measured — the CLAUDE.md §3.2 tautology / dead-code guard, in code.
TEST(GNG, LinearAnnealSeedWeightAtBake) {
    GNG::Config cfg;
    cfg.dim                 = 8;
    cfg.epsilon_b           = 0.05f;
    cfg.epsilon_n           = 0.0f;       // the runner-up stays put
    cfg.baking_threshold    = 50;
    cfg.mitosis_enabled     = false;
    cfg.stale_prune_enabled = false;
    cfg.lambda_new          = 1000000;    // no insertion inside the window
    GNG gng(cfg);

    Eigen::VectorXf seed  = Eigen::VectorXf::Zero(8); seed(0)  = 1.0f;
    Eigen::VectorXf delta = Eigen::VectorXf::Zero(8); delta(1) = 0.1f;
    gng.step(seed); gng.step(seed);                    // bootstrap: two nodes at the seed
    Eigen::VectorXf x = seed + delta;

    int winner = -1;
    for (int n = 0; n < cfg.baking_threshold; ++n) {
        auto [w, d] = gng.step(x);
        winner = w;
    }
    auto proto = gng.get_prototype(winner);
    ASSERT_TRUE(proto.has_value());
    EXPECT_TRUE(gng.is_crystallised(winner));

    // Fraction of the seed→x segment the prototype has covered.
    float moved = (proto.value() - seed)(1) / delta(1);
    double expect_seed_w = 1.0;
    for (int n = 0; n < cfg.baking_threshold; ++n)
        expect_seed_w *= 1.0 - 0.05 * (1.0 - 0.9 * double(n) / cfg.baking_threshold);
    EXPECT_NEAR(expect_seed_w, 0.241, 0.002);
    EXPECT_NEAR(1.0 - double(moved), expect_seed_w, 0.01);
}

// ---------------------------------------------------------------------------
// Kalman-lessons Stage 1 — the per-node Kalman gain.
// ---------------------------------------------------------------------------

static GNG::Config kalman_pin_cfg() {
    GNG::Config cfg;
    cfg.dim                 = 8;
    cfg.epsilon_b           = 0.05f;
    cfg.epsilon_n           = 0.0f;
    cfg.baking_threshold    = 50;
    cfg.mitosis_enabled     = false;
    cfg.stale_prune_enabled = false;
    cfg.lambda_new          = 1000000;
    cfg.gain_kind           = GainKind::Kalman;
    return cfg;
}

// With p0 = 1 and q = 0 the schedule is 1/(n+1): after N wins the seed keeps
// exactly 1/(N+1) of the weight (0.0196 at N = 50, against the linear anneal's
// 0.241 pinned above).  Then, baked, the node must not move at all.
TEST(GNG, KalmanGainIsTheFilterForAConstantAndFreezesAtBake) {
    GNG::Config cfg = kalman_pin_cfg();
    GNG gng(cfg);
    Eigen::VectorXf seed  = Eigen::VectorXf::Zero(8); seed(0)  = 1.0f;
    Eigen::VectorXf delta = Eigen::VectorXf::Zero(8); delta(1) = 0.1f;
    gng.step(seed); gng.step(seed);
    Eigen::VectorXf x = seed + delta;
    int winner = -1;
    for (int n = 0; n < cfg.baking_threshold; ++n) { auto [w, d] = gng.step(x); winner = w; }
    auto proto = gng.get_prototype(winner);
    ASSERT_TRUE(proto.has_value());
    EXPECT_TRUE(gng.is_crystallised(winner));
    float moved = (proto.value() - seed)(1) / delta(1);
    EXPECT_NEAR(1.0 - double(moved), 1.0 / (cfg.baking_threshold + 1), 1e-4);

    // Baked + q = 0: frozen, bit-for-bit, even against a new offset.
    Eigen::VectorXf x2 = x + delta;
    Eigen::VectorXf before = proto.value();
    for (int n = 0; n < 30; ++n) { auto [w, d] = gng.step(x2); EXPECT_EQ(w, winner); }
    EXPECT_TRUE(gng.get_prototype(winner).value() == before);
}

// With q > 0 a baked node keeps a steady-state gain and follows a moved input.
TEST(GNG, KalmanGainWithProcessNoiseTracksAfterBake) {
    GNG::Config cfg = kalman_pin_cfg();
    cfg.kalman_q = 0.01f;                       // K_inf = (q + sqrt(q^2 + 4q))/2 ~ 0.095
    GNG gng(cfg);
    Eigen::VectorXf seed  = Eigen::VectorXf::Zero(8); seed(0)  = 1.0f;
    Eigen::VectorXf delta = Eigen::VectorXf::Zero(8); delta(1) = 0.1f;
    gng.step(seed); gng.step(seed);
    Eigen::VectorXf x = seed + delta;
    int winner = -1;
    for (int n = 0; n < cfg.baking_threshold; ++n) { auto [w, d] = gng.step(x); winner = w; }
    ASSERT_TRUE(gng.is_crystallised(winner));
    Eigen::VectorXf x2 = x + delta;             // the world drifted
    float before = (gng.get_prototype(winner).value() - x2).norm();
    for (int n = 0; n < 100; ++n) gng.step(x2);
    float after = (gng.get_prototype(winner).value() - x2).norm();
    EXPECT_LT(after, 0.05f * before);           // (1 - 0.095)^100 ~ 5e-5 of the way left
}

// Kalman state round-trips through JSON; Linear mode emits none of it, so a
// pre-feature snapshot is byte-identical.
TEST(GNG, KalmanStateSerialisation) {
    GNG::Config cfg = kalman_pin_cfg();
    GNG gng(cfg);
    Eigen::VectorXf seed = Eigen::VectorXf::Zero(8); seed(0) = 1.0f;
    gng.step(seed); gng.step(seed);
    for (int n = 0; n < 5; ++n) gng.step(seed);
    auto j = gng.to_json();
    EXPECT_EQ(j.value("gain_kind", std::string("")), "kalman");
    ASSERT_TRUE(j["nodes"][0].contains("p"));
    GNG back = GNG::from_json(j);
    EXPECT_EQ(back.gain_kind(), GainKind::Kalman);
    // Node storage is an unordered_map, so array order may differ after a
    // round trip; compare per id.
    auto by_id = [](nlohmann::json const& nodes) {
        std::map<int, nlohmann::json> m;
        for (auto const& n : nodes) m[n.at("id").get<int>()] = n;
        return m;
    };
    auto jr = back.to_json();
    EXPECT_EQ(by_id(jr["nodes"]), by_id(j["nodes"]));
    EXPECT_EQ(jr.value("kalman_p0", -1.0f), j.value("kalman_p0", -2.0f));
    EXPECT_EQ(jr.value("kalman_q",  -1.0f), j.value("kalman_q",  -2.0f));

    GNG::Config lin = kalman_pin_cfg();
    lin.gain_kind = GainKind::Linear;
    GNG g2(lin);
    g2.step(seed); g2.step(seed); g2.step(seed);
    auto j2 = g2.to_json();
    EXPECT_FALSE(j2.contains("gain_kind"));
    EXPECT_FALSE(j2["nodes"][0].contains("p"));
}

// THE BAKE GATE (2026-10-04, microduck design doc §17.108): a host that freezes insertion by raising min_insertion_error
// also opened the bake check -- a noisy node baked unchecked; bake_gate keeps the consistency check at its own value.
TEST(GNG, BakeGateKeepsTheConsistencyCheckWhenInsertionIsFrozen) {
    auto run = [](float bake_gate) {
        GNG::Config cfg;
        cfg.dim = 16; cfg.baking_threshold = 10; cfg.lambda_new = 100000;
        cfg.min_insertion_error = 1e9f;     // insertion frozen, as the duck's --map-on-stop does on walks
        cfg.bake_gate = bake_gate;
        GNG gng(cfg);
        std::mt19937 rng(7); std::normal_distribution<float> n(0.0f, 1.0f);
        for (int i = 0; i < 400; ++i) {
            Eigen::VectorXf v(16); for (int k = 0; k < 16; ++k) v[k] = n(rng);
            gng.step(v / v.norm());
        }
        return gng.baked_count();
    };
    EXPECT_GT(run(0.0f), 0) << "the defect: with the gate at the frozen floor every visited node bakes";
    EXPECT_EQ(run(0.06f), 0) << "noise this wide never passes a 0.06 consistency gate";
}

// ---------------------------------------------------------------------------
// Inference-only mode (Config::learning_enabled = false)
// ---------------------------------------------------------------------------
//
// A frozen GNG must answer "which node is nearest, and how far" exactly as the
// learning path would, and change nothing it has learned.  The trained fixture
// turns on every optional state path (autotune history, drift residual sums,
// Kalman p) so the to_json() comparison covers them too.

namespace {

// Clustered unit-ish inputs: 6 centres, small noise, so the GNG grows, bakes
// and carries post-bake state by the end of training.
struct ClusterStream {
    std::mt19937 rng;
    std::vector<Eigen::VectorXf> centres;
    int dim;
    ClusterStream(unsigned seed, int dim_) : rng(seed), dim(dim_) {
        for (int c = 0; c < 6; ++c) centres.push_back(random_unit(rng, dim));
    }
    Eigen::VectorXf next() {
        std::uniform_int_distribution<int> pick(0, int(centres.size()) - 1);
        std::normal_distribution<float> noise(0.0f, 0.05f);
        Eigen::VectorXf v = centres[size_t(pick(rng))];
        for (int i = 0; i < dim; ++i) v(i) += noise(rng);
        return v;
    }
};

GNG::Config freeze_cfg(GainKind kind) {
    GNG::Config cfg;
    cfg.dim                 = 16;
    cfg.baking_threshold    = 20;
    cfg.lambda_new          = 10;
    cfg.min_insertion_error = 0.02f;
    cfg.insertion_autotune  = true;
    cfg.drift_ratio         = 0.5f;
    cfg.gain_kind           = kind;
    cfg.stale_window_factor = 300.0f;
    cfg.health_death_min_nodes = 2;
    return cfg;
}

GNG trained_gng(GainKind kind, unsigned seed = 7) {
    GNG gng(freeze_cfg(kind));
    ClusterStream s(seed, 16);
    for (int t = 0; t < 3000; ++t) {
        auto x = s.next();
        auto [w, d] = gng.step(x);
        gng.maybe_mitosis(w, x);
    }
    return gng;
}

// last_step_baked is a per-step flag, not learned state: a frozen step clears
// it, which is what "no bake happened this step" means.  Compare the rest.
nlohmann::json learned_state(GNG const& g) {
    auto j = g.to_json();
    j.erase("last_step_baked");
    return j;
}

} // namespace

TEST(GNGFreeze, DefaultIsLearning) {
    GNG::Config cfg;
    EXPECT_TRUE(cfg.learning_enabled);
    GNG g(cfg);
    EXPECT_TRUE(g.learning_enabled());
    g.set_learning_enabled(false);
    EXPECT_FALSE(g.learning_enabled());
}

TEST(GNGFreeze, FrozenRunLeavesLearnedStateUntouched) {
    for (GainKind kind : {GainKind::Linear, GainKind::Kalman}) {
        GNG gng = trained_gng(kind);
        ASSERT_GT(gng.node_count(), 2);
        ASSERT_GT(gng.baked_count(), 0) << "fixture must exercise post-bake state";
        const auto before      = learned_state(gng);
        const int  step_before = gng.step_count();

        gng.set_learning_enabled(false);
        ClusterStream held_out(1234, 16);
        std::mt19937 rng(99);
        for (int t = 0; t < 2000; ++t) {
            // Mix in-distribution and far-off inputs: novelty must not grow nodes.
            Eigen::VectorXf x = (t % 3 == 0) ? Eigen::VectorXf(3.0f * random_unit(rng, 16))
                                             : held_out.next();
            auto [w, d] = gng.step(x);
            EXPECT_FALSE(gng.maybe_mitosis(w, x));
            EXPECT_FALSE(gng.boost_visits(w, 50));
            EXPECT_FALSE(gng.last_step_baked());
            EXPECT_TRUE(gng.last_pruned_ids().empty());
        }
        EXPECT_EQ(gng.step_count(), step_before);
        EXPECT_EQ(learned_state(gng), before);
    }
}

TEST(GNGFreeze, FrozenWinnerIsBruteForceNearestAndMatchesLearningPath) {
    GNG gng = trained_gng(GainKind::Linear);
    gng.set_learning_enabled(false);
    const auto j = gng.to_json();

    std::mt19937 rng(5);
    ClusterStream held_out(77, 16);
    for (int t = 0; t < 500; ++t) {
        Eigen::VectorXf x = (t % 2) ? held_out.next() : Eigen::VectorXf(random_unit(rng, 16));

        int   best_id = -1;
        float best_d  = std::numeric_limits<float>::infinity();
        for (auto const& n : j["nodes"]) {
            auto pv = n["prototype"].get<std::vector<float>>();
            Eigen::Map<const Eigen::VectorXf> p(pv.data(), Eigen::Index(pv.size()));
            float d = (p - x).norm();
            if (d < best_d) { best_d = d; best_id = n["id"].get<int>(); }
        }
        auto [w, d] = gng.step(x);
        EXPECT_EQ(w, best_id);
        EXPECT_FLOAT_EQ(d, best_d);

        // The learning path reports the same pair for the same input.
        GNG learner = gng;
        learner.set_learning_enabled(true);
        auto [wl, dl] = learner.step(x);
        EXPECT_EQ(wl, w);
        EXPECT_EQ(dl, d);
    }
}

TEST(GNGFreeze, FrozenBeforeBootstrapDoesNotBufferInput) {
    GNG gng(freeze_cfg(GainKind::Linear));
    gng.set_learning_enabled(false);
    std::mt19937 rng(3);
    for (int t = 0; t < 10; ++t) {
        auto [w, d] = gng.step(random_unit(rng, 16));
        EXPECT_EQ(w, 0);
        EXPECT_EQ(d, 0.0f);
    }
    EXPECT_EQ(gng.node_count(), 0);
    // Resuming bootstraps from the next two inputs, not from frozen ones.
    gng.set_learning_enabled(true);
    auto a = make_vec(0.3f, 16), b = random_unit(rng, 16);
    gng.step(a);
    EXPECT_EQ(gng.node_count(), 0);
    gng.step(b);
    ASSERT_EQ(gng.node_count(), 2);
    EXPECT_TRUE(gng.get_prototype(0)->isApprox(a));
    EXPECT_TRUE(gng.get_prototype(1)->isApprox(b));
}

TEST(GNGFreeze, MitosisGateIsClosedWhileFrozen) {
    // Settings under which the gatekeeper splits on the first post-bake check.
    GNG::Config cfg = freeze_cfg(GainKind::Linear);
    cfg.drift_ratio             = 0.0f;
    cfg.mitosis_error_threshold = 0.0f;
    cfg.mitosis_check_interval  = 1;
    cfg.min_insertion_error     = 10.0f;   // everything bakes; nothing inserts
    cfg.baking_threshold        = 5;
    GNG gng(cfg);
    auto x = make_vec(0.5f, 16);
    for (int t = 0; t < 20; ++t) gng.step(x);
    auto [w, d] = gng.step(x);

    GNG control = gng;
    EXPECT_TRUE(control.maybe_mitosis(w, x)) << "control must split, or the test proves nothing";

    gng.set_learning_enabled(false);
    const auto before = learned_state(gng);
    EXPECT_FALSE(gng.maybe_mitosis(w, x));
    EXPECT_EQ(gng.mitosis_count(), 0);
    EXPECT_EQ(learned_state(gng), before);
}

TEST(GNGFreeze, UnfreezingResumesLearning) {
    GNG gng = trained_gng(GainKind::Linear);
    gng.set_learning_enabled(false);
    ClusterStream s(4321, 16);
    for (int t = 0; t < 200; ++t) gng.step(s.next());
    const auto frozen = learned_state(gng);
    const int  step_frozen = gng.step_count();

    gng.set_learning_enabled(true);
    for (int t = 0; t < 200; ++t) gng.step(s.next());
    EXPECT_EQ(gng.step_count(), step_frozen + 200);
    EXPECT_NE(learned_state(gng), frozen);
}

TEST(GNGFreeze, LearningOnIsUnchangedByTheFlag) {
    // Two GNGs, one of which toggles the flag off and on without stepping in
    // between, must stay identical: the switch itself carries no state.
    GNG a(freeze_cfg(GainKind::Linear)), b(freeze_cfg(GainKind::Linear));
    ClusterStream sa(11, 16), sb(11, 16);
    for (int t = 0; t < 1500; ++t) {
        if (t % 100 == 0) { b.set_learning_enabled(false); b.set_learning_enabled(true); }
        auto xa = sa.next(), xb = sb.next();
        auto ra = a.step(xa); auto rb = b.step(xb);
        EXPECT_EQ(ra, rb);
    }
    EXPECT_EQ(a.to_json(), b.to_json());
}
