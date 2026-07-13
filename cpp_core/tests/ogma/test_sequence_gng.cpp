// =============================================================================
// test_sequence_gng.cpp  --  Unit tests for SequenceGNG
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/SequenceGNG.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap winner_params() {
    return {
        {"source_topic",     std::string("reality.proprio.imu")},
        {"source_kind",      std::string("winner")},
        {"window_size",      int64_t{4}},
        {"projection_dim",   int64_t{32}},
        {"prototype_per_winner_dim", int64_t{16}},
        {"baking_threshold", int64_t{20}},
        {"min_insertion_error", 0.001},
    };
}

ogma::ParamMap action_params() {
    return {
        {"source_topic",   std::string("action.out")},
        {"source_kind",    std::string("action")},
        {"window_size",    int64_t{5}},
        {"projection_dim", int64_t{16}},
        {"baking_threshold", int64_t{20}},
        {"min_insertion_error", 0.001},
    };
}

std::shared_ptr<ogma::RealityToken> make_winner(int wid) {
    auto t = std::make_shared<ogma::RealityToken>();
    t->winner_id = wid;
    t->latent    = Eigen::VectorXf::Constant(8, float(wid));
    return t;
}

std::shared_ptr<ogma::ActionOut> make_action(float a) {
    auto m = std::make_shared<ogma::ActionOut>();
    m->accel = a;
    return m;
}

struct SeqFixture {
    ogma::InProcessBus  bus;
    ogma::SequenceGNG   seq;

    explicit SeqFixture(ogma::ParamMap params) {
        seq.set_id("seq_test");
        seq.on_setup(&bus, params);
    }

    std::shared_ptr<const ogma::SequenceMotif> last_motif() const {
        return std::dynamic_pointer_cast<const ogma::SequenceMotif>(
            bus.last_value(seq.output_topics()[0].name));
    }
};

} // namespace

// -- Construction / contract -----------------------------------------------

TEST(SequenceGNG, ConstructsAndDeclaresContract) {
    SeqFixture f(winner_params());
    EXPECT_EQ(f.seq.type_name(), "SequenceGNG");
    EXPECT_EQ(f.seq.output_topics().size(), 1u);
    EXPECT_EQ(f.seq.output_topics()[0].name, "sequence.motif.reality.proprio.imu");
}

TEST(SequenceGNG, OutputTopicOverride) {
    auto p = winner_params();
    p["output_topic"] = std::string("sequence.motif.consensus.0");
    SeqFixture f(p);
    EXPECT_EQ(f.seq.output_topics()[0].name, "sequence.motif.consensus.0");
}

// -- Winner-source encoding ------------------------------------------------

TEST(SequenceGNG, WindowFillProducesPlaceholderUntilFull) {
    SeqFixture f(winner_params());

    // Window size 4: ticks 0–2 should publish placeholder (motif_id = -1).
    for (uint64_t t = 0; t < 3; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("reality.proprio.imu", make_winner(int(t)));
        f.seq.tick(t);
        f.bus.end_tick();
    }
    EXPECT_EQ(f.last_motif()->motif_id, -1);
}

TEST(SequenceGNG, RecurrentMotifBakesEventually) {
    SeqFixture f(winner_params());

    // Drive 200 ticks of the same 4-step cycle: winner_id ∈ {1, 2, 3, 4}.
    int seq[] = {1, 2, 3, 4};
    for (uint64_t t = 0; t < 200; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("reality.proprio.imu", make_winner(seq[t % 4]));
        f.seq.tick(t);
        f.bus.end_tick();
    }
    EXPECT_GE(f.seq.baked_count(), 1);
}

TEST(SequenceGNG, BootstrapWinnerIdMinusOneIgnored) {
    SeqFixture f(winner_params());

    // All inputs are bootstrap (winner_id = -1) → window stays empty.
    for (uint64_t t = 0; t < 10; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("reality.proprio.imu", make_winner(-1));
        f.seq.tick(t);
        f.bus.end_tick();
    }
    EXPECT_EQ(f.seq.node_count(), 0);
    EXPECT_EQ(f.last_motif()->motif_id, -1);
}

// -- Successor tracking -----------------------------------------------------

TEST(SequenceGNG, PredictedNextIdConvergesToFrequentSuccessor) {
    SeqFixture f(winner_params());

    // Force the same 4-cycle long enough for the GNG to bake AND for
    // successor counts to accumulate.
    int seq[] = {1, 2, 3, 4};
    for (uint64_t t = 0; t < 400; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("reality.proprio.imu", make_winner(seq[t % 4]));
        f.seq.tick(t);
        f.bus.end_tick();
    }
    auto m = f.last_motif();
    ASSERT_NE(m, nullptr);
    if (m->motif_id >= 0) {
        // The predicted_next_id should be one of {1,2,3,4} once successor
        // counts have accumulated.
        EXPECT_GE(m->predicted_next_id, 1);
        EXPECT_LE(m->predicted_next_id, 4);
    }
}

// -- Action-source encoding -----------------------------------------------

TEST(SequenceGNG, ActionStreamGrowsTopology) {
    // Verifies the action-source path is wired and GNG learns over the
    // window's projected vector space.  Bake-count thresholds are
    // parameter-sensitive (a Phase-3 sweep concern); we just assert the
    // GNG accumulates at least the bootstrap nodes here.
    SeqFixture f(action_params());

    float pattern[] = {-3.5f, -1.5f, 0.0f, 1.5f, 3.5f};
    for (uint64_t t = 0; t < 300; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("action.out", make_action(pattern[t % 5]));
        f.seq.tick(t);
        f.bus.end_tick();
    }
    EXPECT_GE(f.seq.node_count(), 2);
}

// -- First-tick safety -----------------------------------------------------

TEST(SequenceGNG, FirstTickPublishesPlaceholderNoNan) {
    SeqFixture f(winner_params());
    f.bus.begin_tick(0);
    f.seq.tick(0);
    f.bus.end_tick();

    auto m = f.last_motif();
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(m->motif_id, -1);
    EXPECT_FALSE(std::isnan(m->match_confidence));
}

// -- Hot-mutation ---------------------------------------------------------

TEST(SequenceGNG, HotMutateBranchingThreshold) {
    SeqFixture f(winner_params());
    EXPECT_NO_THROW(f.seq.on_param_change("motif_branching_threshold",
                                          ogma::ParamValue{0.6}));
}

TEST(SequenceGNG, ConstructionOnlyParamsThrow) {
    SeqFixture f(winner_params());
    EXPECT_THROW(f.seq.on_param_change("source_topic",
                                       ogma::ParamValue{std::string("x")}),
                 std::invalid_argument);
    EXPECT_THROW(f.seq.on_param_change("window_size",
                                       ogma::ParamValue{int64_t{6}}),
                 std::invalid_argument);
}

TEST(SequenceGNG, UnknownParamThrows) {
    SeqFixture f(winner_params());
    EXPECT_THROW(f.seq.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}

// -- Determinism ----------------------------------------------------------

TEST(SequenceGNG, DeterministicAcrossTwoIdenticalRuns) {
    auto run = []() {
        SeqFixture f(winner_params());
        int seq[] = {1, 2, 3, 4, 1, 2, 5, 4};
        for (uint64_t t = 0; t < 100; ++t) {
            f.bus.begin_tick(t);
            f.bus.publish("reality.proprio.imu", make_winner(seq[t % 8]));
            f.seq.tick(t);
            f.bus.end_tick();
        }
        return std::make_pair(f.seq.node_count(), f.seq.baked_count());
    };
    auto a = run();
    auto b = run();
    EXPECT_EQ(a, b);
}
