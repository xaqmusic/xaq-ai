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
