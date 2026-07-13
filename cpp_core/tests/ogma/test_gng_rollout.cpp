// =============================================================================
// test_gng_rollout.cpp  --  Unit tests for GNGRollout
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/GNGRollout.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap default_params() {
    return {
        {"K_default",            int64_t{16}},
        {"M_default",            int64_t{4}},
        {"transition_smoothing", 0.01},
        {"max_concurrent_queries", int64_t{4}},
    };
}

std::shared_ptr<ogma::RealityToken> make_winner(int wid) {
    auto t = std::make_shared<ogma::RealityToken>();
    t->winner_id = wid;
    return t;
}

std::shared_ptr<ogma::RolloutQuery> make_query(std::string source, int winner,
                                                int M = 4, int K = 16,
                                                uint64_t request_id = 0) {
    auto q = std::make_shared<ogma::RolloutQuery>();
    q->source_modality = std::move(source);
    q->winner_id       = winner;
    q->M_steps         = M;
    q->K_samples       = K;
    q->request_id      = request_id;
    return q;
}

struct RolloutFixture {
    ogma::InProcessBus  bus;
    ogma::GNGRollout    rollout;
    std::vector<std::shared_ptr<const ogma::RolloutResult>> received;

    explicit RolloutFixture(ogma::ParamMap params = default_params()) {
        rollout.set_id("rollout_0");
        rollout.on_setup(&bus, params);
        bus.subscribe(ogma::topics::kRolloutResult, ogma::SubscriptionKind::Direct,
            [this](std::string_view, ogma::MessagePtr p) {
                received.push_back(std::dynamic_pointer_cast<const ogma::RolloutResult>(p));
            });
    }

    // Drive a deterministic transition graph for "reality.test.x":
    //   1 → 2, 2 → 3, 3 → 1, 1 → 2, ...   (a 3-cycle)
    void prime_three_cycle(int repetitions = 50) {
        int cycle[] = {1, 2, 3};
        for (int t = 0; t < repetitions; ++t) {
            bus.begin_tick(uint64_t(t));
            bus.publish("reality.test.x", make_winner(cycle[t % 3]));
            rollout.tick(uint64_t(t));
            bus.end_tick();
        }
    }
};

} // namespace

// -- Construction / contract ----------------------------------------------

TEST(GNGRollout, ConstructsAndDeclaresContract) {
    RolloutFixture f;
    EXPECT_EQ(f.rollout.type_name(), "GNGRollout");
    EXPECT_EQ(f.rollout.output_topics()[0].name, "rollout.result");
}

// -- Topology caching -----------------------------------------------------

TEST(GNGRollout, RealityStreamPopulatesTransitionCache) {
    RolloutFixture f;
    f.prime_three_cycle(30);
    EXPECT_EQ(f.rollout.known_sources(), 1u);
    EXPECT_EQ(f.rollout.known_nodes("reality.test.x"), 3u);
}

TEST(GNGRollout, BootstrapTokensIgnored) {
    RolloutFixture f;
    f.bus.begin_tick(0);
    f.bus.publish("reality.test.x", make_winner(-1));
    f.rollout.tick(0);
    f.bus.end_tick();
    EXPECT_EQ(f.rollout.known_sources(), 0u);
}

// -- Query handling -------------------------------------------------------

TEST(GNGRollout, QueryReturnsResultWithMatchingId) {
    RolloutFixture f;
    f.prime_three_cycle(60);

    f.bus.begin_tick(60);
    f.bus.publish(ogma::topics::kRolloutQuery,
                  make_query("reality.test.x", /*winner=*/1, /*M=*/4, /*K=*/16, /*req=*/42));
    f.rollout.tick(60);
    f.bus.end_tick();

    ASSERT_FALSE(f.received.empty());
    auto r = f.received.back();
    EXPECT_EQ(r->request_id, 42u);
    EXPECT_FALSE(r->trajectories.empty());
    EXPECT_EQ(r->terminal_values.size(), r->trajectories.size());
    for (auto const& traj : r->trajectories)
        EXPECT_EQ(int(traj.size()), 4);   // M_steps
    EXPECT_GE(r->entropy, 0.0f);
}

TEST(GNGRollout, UnknownSourceReturnsEmptyTrajectories) {
    RolloutFixture f;
    f.bus.begin_tick(0);
    f.bus.publish(ogma::topics::kRolloutQuery,
                  make_query("reality.never.seen", 1, 4, 8, 7));
    f.rollout.tick(0);
    f.bus.end_tick();

    ASSERT_FALSE(f.received.empty());
    auto r = f.received.back();
    EXPECT_EQ(r->request_id, 7u);
    EXPECT_TRUE(r->trajectories.empty());
}

TEST(GNGRollout, UnknownStartWinnerReturnsEmptyTrajectories) {
    RolloutFixture f;
    f.prime_three_cycle(30);

    f.bus.begin_tick(30);
    f.bus.publish(ogma::topics::kRolloutQuery,
                  make_query("reality.test.x", /*winner=*/999, 4, 8, 99));
    f.rollout.tick(30);
    f.bus.end_tick();

    auto r = f.received.back();
    EXPECT_TRUE(r->trajectories.empty());
}

// -- Trajectory shape ----------------------------------------------------

TEST(GNGRollout, ThreeCycleTrajectoriesFollowTopology) {
    // M_default=4 in the fixture caps the requested M; query within cap.
    RolloutFixture f;
    f.prime_three_cycle(60);

    f.bus.begin_tick(60);
    f.bus.publish(ogma::topics::kRolloutQuery,
                  make_query("reality.test.x", /*winner=*/1, /*M=*/3, /*K=*/4, 1));
    f.rollout.tick(60);
    f.bus.end_tick();

    auto r = f.received.back();
    ASSERT_FALSE(r->trajectories.empty());
    for (auto const& traj : r->trajectories) {
        EXPECT_EQ(int(traj.size()), 3);
        // Every transition in the trajectory must respect the cyclic graph
        // we constructed: 1→2, 2→3, 3→1.
        int prev = 1;   // the query's winner_id
        for (int n : traj) {
            int expected = (prev == 1) ? 2 : (prev == 2) ? 3 : 1;
            EXPECT_EQ(n, expected);
            prev = n;
        }
    }
}

// -- Concurrency cap -----------------------------------------------------

TEST(GNGRollout, MaxConcurrentQueriesCapped) {
    auto p = default_params();
    p["max_concurrent_queries"] = int64_t{2};
    RolloutFixture f(p);
    f.prime_three_cycle(30);

    f.bus.begin_tick(30);
    for (uint64_t i = 0; i < 5; ++i)
        f.bus.publish(ogma::topics::kRolloutQuery,
                      make_query("reality.test.x", 1, 3, 4, /*req=*/i));
    f.rollout.tick(30);
    f.bus.end_tick();

    // 5 queries issued; first 2 produce non-empty trajectories, remaining
    // 3 are capped → empty.
    ASSERT_EQ(f.received.size(), 5u);
    int with_traj = 0;
    for (auto const& r : f.received) if (!r->trajectories.empty()) ++with_traj;
    EXPECT_EQ(with_traj, 2);
}

// -- Determinism ---------------------------------------------------------

TEST(GNGRollout, DeterministicAcrossTwoIdenticalRuns) {
    auto run = []() {
        RolloutFixture f;
        f.prime_three_cycle(30);
        f.bus.begin_tick(30);
        f.bus.publish(ogma::topics::kRolloutQuery,
                      make_query("reality.test.x", 1, 4, 8, 1));
        f.rollout.tick(30);
        f.bus.end_tick();
        return f.received.back()->trajectories;
    };
    auto a = run();
    auto b = run();
    EXPECT_EQ(a, b);
}

// -- First-tick safety ---------------------------------------------------

TEST(GNGRollout, FirstTickWithNoInputsProducesNoCrash) {
    RolloutFixture f;
    EXPECT_NO_THROW({
        f.bus.begin_tick(0);
        f.rollout.tick(0);
        f.bus.end_tick();
    });
}

// -- Hot-mutation --------------------------------------------------------

TEST(GNGRollout, HotMutateK_M_AndCap) {
    RolloutFixture f;
    EXPECT_NO_THROW(f.rollout.on_param_change("K_default", ogma::ParamValue{int64_t{8}}));
    EXPECT_NO_THROW(f.rollout.on_param_change("M_default", ogma::ParamValue{int64_t{2}}));
    EXPECT_NO_THROW(f.rollout.on_param_change("max_concurrent_queries",
                                               ogma::ParamValue{int64_t{1}}));
}

TEST(GNGRollout, MasterSeedConstructionOnly) {
    RolloutFixture f;
    EXPECT_THROW(f.rollout.on_param_change("master_seed", ogma::ParamValue{int64_t{99}}),
                 std::invalid_argument);
}

TEST(GNGRollout, UnknownParamThrows) {
    RolloutFixture f;
    EXPECT_THROW(f.rollout.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}
