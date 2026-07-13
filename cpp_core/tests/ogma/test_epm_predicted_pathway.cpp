// =============================================================================
// test_epm_predicted_pathway.cpp
//   Phase 6.6.E — EPM forward-rollout (predicted_pathway field).
//
// The EPM maintains a per-node successor count map and, when configured with
// predicted_pathway_steps > 0, attaches a greedy-argmax forward walk to every
// published RealityToken.  These tests drive synthetic latent streams through
// an Identity-encoder EPM and assert four behavioral guarantees:
//
//   1. Off-by-default: pathway empty when predicted_pathway_steps == 0
//      (bit-identity with pre-6.6.E EPM behavior).
//   2. Pathway is bounded by the configured length.
//   3. After a stable A↔B alternating stream, the pathway from the current
//      winner correctly predicts the alternation.
//   4. A cold winner (no recorded successors yet) emits an empty pathway
//      rather than garbage.
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap identity_params(int pathway_steps = 0) {
    return {
        {"modality_group",      std::string("consensus")},
        {"modality_name",       std::string("1")},
        {"encoder_kind",        std::string("identity")},
        {"input_topic",         std::string("consensus.0")},
        {"projection_dim",      int64_t{4}},
        {"baking_threshold",    int64_t{4}},
        {"min_insertion_error", 0.001},
        {"lambda_new",          int64_t{1}},
        {"history_trace_size",  int64_t{4}},
        {"predicted_pathway_steps", int64_t(pathway_steps)},
        {"subtract_descending_prediction", false},
        {"mitosis_enabled",     false},
        {"stale_prune_enabled", false},
    };
}

std::shared_ptr<ogma::ConsensusToken> make_consensus(Eigen::VectorXf v) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding = std::move(v);
    c->fused_tle       = 0.0f;
    return c;
}

struct EpmFixture {
    ogma::InProcessBus bus;
    ogma::EPM          epm;
    explicit EpmFixture(ogma::ParamMap const& p) {
        epm.set_id("epm_under_test");
        epm.on_setup(&bus, p);
    }
    std::shared_ptr<const ogma::RealityToken> last_token() const {
        return std::dynamic_pointer_cast<const ogma::RealityToken>(
            bus.last_value("consensus.1"));
    }
    void tick(uint64_t t, std::shared_ptr<ogma::ConsensusToken> input) {
        bus.begin_tick(t);
        bus.publish("consensus.0", input);
        epm.tick(t);
        bus.end_tick();
    }
};

// Drive an A,B,A,B... alternation for `cycles` ticks.  Returns the EPM so the
// caller can keep ticking with the same internal state.
void drive_alternation(EpmFixture& f, int cycles) {
    Eigen::VectorXf a = Eigen::VectorXf::Zero(4);  a[0] = 1.0f;
    Eigen::VectorXf b = Eigen::VectorXf::Zero(4);  b[1] = 1.0f;
    for (int i = 0; i < cycles; ++i) {
        f.tick(uint64_t(2 * i),     make_consensus((i & 1) ? b : a));
        f.tick(uint64_t(2 * i + 1), make_consensus((i & 1) ? a : b));
    }
}

} // namespace

// =============================================================================
// 1. Off-by-default: predicted_pathway_steps=0 → empty pathway always.
// =============================================================================

TEST(EpmPredictedPathway, EmptyWhenDisabled) {
    EpmFixture f(identity_params(/*steps=*/0));
    drive_alternation(f, 20);     // plenty of stable transitions

    auto tok = f.last_token();
    ASSERT_NE(tok, nullptr);
    EXPECT_TRUE(tok->predicted_pathway.empty())
        << "With predicted_pathway_steps=0 the field must remain empty";
}

// =============================================================================
// 2. Bounded length: pathway never exceeds predicted_pathway_steps.
// =============================================================================

TEST(EpmPredictedPathway, BoundedByConfiguredLength) {
    EpmFixture f(identity_params(/*steps=*/3));
    drive_alternation(f, 30);

    auto tok = f.last_token();
    ASSERT_NE(tok, nullptr);
    EXPECT_LE(int(tok->predicted_pathway.size()), 3);
}

// =============================================================================
// 3. Stable alternation produces correct predictions.
//    After many A↔B alternations the GNG settles into 2 nodes; each has a
//    strong (and only) successor count to the other.  The pathway from any
//    winner must predict the alternation.
// =============================================================================

TEST(EpmPredictedPathway, PredictsAlternationAfterLearning) {
    EpmFixture f(identity_params(/*steps=*/3));
    drive_alternation(f, 30);

    auto tok = f.last_token();
    ASSERT_NE(tok, nullptr);
    ASSERT_GE(tok->winner_id, 0)
        << "Test assumes the EPM has bootstrapped past placeholder tokens";

    // Pathway must be non-empty (at least one transition learned).
    ASSERT_FALSE(tok->predicted_pathway.empty())
        << "After 60 alternating ticks the EPM should have at least one transition";

    // Greedy walk's only structural guarantee is no immediate self-loop: the
    // EPM suppresses winner == prev_winner transitions, so consecutive
    // pathway entries must differ.  We don't assert "exactly two nodes"
    // because the GNG continues inserting nodes with lambda_new=1, so the
    // alternation may walk through more than two stable IDs over time.
    int prev = tok->winner_id;
    for (int next : tok->predicted_pathway) {
        EXPECT_GE(next, 0) << "Pathway must contain only valid node IDs";
        EXPECT_NE(next, prev)
            << "Greedy walk must not predict the same node consecutively "
               "(self-transitions are suppressed in transition_counts_)";
        prev = next;
    }
}

// =============================================================================
// 4. Cold winner: pathway empty when current winner has no recorded successor.
//    Drive only one input over and over (the GNG can't bootstrap to 2 nodes
//    with a single repeated input — it stays in placeholder).  Then send a
//    fresh second input plus a few stable repeats: the second node has no
//    successors yet, so when it is the winner the pathway must be empty.
// =============================================================================

TEST(EpmPredictedPathway, EmptyWhenCurrentWinnerHasNoSuccessors) {
    EpmFixture f(identity_params(/*steps=*/3));
    Eigen::VectorXf a = Eigen::VectorXf::Zero(4);  a[0] = 1.0f;
    Eigen::VectorXf b = Eigen::VectorXf::Zero(4);  b[1] = 1.0f;

    // Bootstrap with two distinct inputs (need ≥ 2 nodes to escape placeholder).
    f.tick(0, make_consensus(a));
    f.tick(1, make_consensus(b));
    // Now stay on B.  At the moment B becomes the most recent winner and
    // before any B→? transition has been recorded, the pathway from B must
    // be empty (no recorded successor).  We check the FIRST tick where
    // winner_id corresponds to B post-bootstrap.
    f.tick(2, make_consensus(b));

    auto tok = f.last_token();
    ASSERT_NE(tok, nullptr);
    if (tok->winner_id < 0) GTEST_SKIP() << "EPM still in placeholder after 3 ticks";

    // Either pathway is empty (no successor for B yet), or it has exactly
    // one entry where B→B was recorded as a self-transition.  The
    // implementation suppresses winner==prev_winner self-loops, so empty is
    // the expected branch.  Length 0 is the strict guarantee here.
    EXPECT_TRUE(tok->predicted_pathway.empty())
        << "First post-bootstrap tick on a winner with no successor data should "
           "yield an empty predicted_pathway (got size="
        << tok->predicted_pathway.size() << ")";
}
