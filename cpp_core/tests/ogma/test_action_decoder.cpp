// =============================================================================
// test_action_decoder.cpp  --  Unit tests for the v4 ActionDecoder module
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap default_params() {
    return {
        {"consensus_level",  int64_t{0}},
        {"proprio_topic",    std::string("reality.proprio.imu")},
        {"action_bins",      int64_t{3}},
        {"accel_min",        -4.0},
        {"accel_max",         4.0},
        {"pragmatic_gain",   10.0},
        {"epistemic_gain",    1.0},
        {"td_lambda",         0.7},
    };
}

std::shared_ptr<ogma::ConsensusToken>
make_consensus(int dim, int active_winner_id, std::string active_modality = "proprio.imu") {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding   = Eigen::VectorXf::Constant(dim, 0.5f);
    c->fused_tle         = 0.1f;
    c->level             = 0;
    c->active_winner_id  = active_winner_id;
    c->active_modality   = std::move(active_modality);
    return c;
}

std::shared_ptr<ogma::DriveErrors>
make_drive(float urgency = 0.0f, float energy_err = 0.0f) {
    auto d = std::make_shared<ogma::DriveErrors>();
    d->urgency           = urgency;
    d->errors["energy"]  = energy_err;
    return d;
}

std::shared_ptr<ogma::NeuroState>
make_neuro(float dopamine = 0.20f, float serotonin = 0.65f, float reward = 0.0f) {
    auto n = std::make_shared<ogma::NeuroState>();
    n->dopamine        = dopamine;
    n->serotonin       = serotonin;
    n->reward_signal   = reward;
    n->epsilon_b_scale = 1.0f;
    return n;
}

std::shared_ptr<ogma::RealityToken>
make_proprio_reality(int winner_id) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->winner_id = winner_id;
    r->latent    = Eigen::VectorXf::Constant(4, 0.0f);
    return r;
}

struct DecoderFixture {
    ogma::InProcessBus    bus;
    ogma::ActionDecoder   dec;

    explicit DecoderFixture(ogma::ParamMap params = default_params()) {
        dec.set_id("action_decoder");
        dec.on_setup(&bus, params);
    }

    std::shared_ptr<const ogma::ActionOut> last_action() const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(ogma::topics::kActionOut));
    }
};

} // namespace

// -- Construction / contract -----------------------------------------------

TEST(ActionDecoder, ConstructsAndDeclaresContract) {
    DecoderFixture f;
    EXPECT_EQ(f.dec.type_name(), "ActionDecoder");
    EXPECT_EQ(f.dec.output_topics()[0].name, "action.out");
    EXPECT_GE(f.dec.input_topics().size(), 4u);
}

// -- First-tick semantics --------------------------------------------------

TEST(ActionDecoder, FirstTickWithNoInputsEmitsZeroAccel) {
    DecoderFixture f;
    f.bus.begin_tick(0);
    f.dec.tick(0);
    f.bus.end_tick();

    auto a = f.last_action();
    ASSERT_NE(a, nullptr);
    EXPECT_FLOAT_EQ(a->accel, 0.0f);
    EXPECT_FALSE(a->probe);
    EXPECT_EQ(a->chunk_id, -1);
}

TEST(ActionDecoder, BootstrapStateEmitsZero) {
    DecoderFixture f;
    f.bus.begin_tick(0);
    // Bootstrap consensus (active_winner = -1) → zero action.
    f.bus.publish("consensus.0",         make_consensus(4, -1));
    f.bus.publish("drive.errors",        make_drive());
    f.bus.publish("neuro.state",         make_neuro());
    f.bus.publish("reality.proprio.imu", make_proprio_reality(-1));
    f.dec.tick(0);
    f.bus.end_tick();
    EXPECT_FLOAT_EQ(f.last_action()->accel, 0.0f);
}

// -- Output bounds ---------------------------------------------------------

TEST(ActionDecoder, AccelStaysWithinClamps) {
    DecoderFixture f;
    for (uint64_t t = 0; t < 50; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",         make_consensus(4, int(t)));
        f.bus.publish("drive.errors",        make_drive(/*urgency=*/0.9f));
        f.bus.publish("neuro.state",         make_neuro(0.5f, 0.5f, 0.5f));
        f.bus.publish("reality.proprio.imu", make_proprio_reality(int(t % 3)));
        f.dec.tick(t);
        f.bus.end_tick();
        auto a = f.last_action();
        EXPECT_GE(a->accel, -4.0f);
        EXPECT_LE(a->accel,  4.0f);
        EXPECT_FALSE(std::isnan(a->accel));
    }
}

// -- TD updates populate the valence map -----------------------------------

TEST(ActionDecoder, RewardSignalPopulatesValenceMap) {
    DecoderFixture f;

    // Tick 0: state (5, 1).  Reward not yet meaningful (no prior action).
    f.bus.begin_tick(0);
    f.bus.publish("consensus.0",         make_consensus(4, 5));
    f.bus.publish("drive.errors",        make_drive());
    f.bus.publish("neuro.state",         make_neuro(0.5f, 0.5f, /*reward=*/0.0f));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.dec.tick(0);
    f.bus.end_tick();

    // Tick 1: same (state, proprio); reward fires.  TD update writes valence.
    f.bus.begin_tick(1);
    f.bus.publish("consensus.0",         make_consensus(4, 5));
    f.bus.publish("drive.errors",        make_drive());
    f.bus.publish("neuro.state",         make_neuro(0.5f, 0.5f, /*reward=*/0.5f));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.dec.tick(1);
    f.bus.end_tick();

    EXPECT_GE(f.dec.valence_size(), 1u);
}

// -- ExplorationDirective override ----------------------------------------
//
// The Probe machine was removed in favour of HomeokineticExploration; its
// ActionDecoder hook is the `exploration.directive` topic.  Behaviour is
// covered end-to-end in pair_homeokinetic_actiondecoder.

TEST(ActionDecoder, ExplorationDirectiveOverridesEFE) {
    DecoderFixture f;

    f.bus.begin_tick(0);
    f.bus.publish("consensus.0",         make_consensus(4, 5));
    f.bus.publish("drive.errors",        make_drive());
    f.bus.publish("neuro.state",         make_neuro(/*da=*/0.5f, /*ht=*/0.5f));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    auto dir = std::make_shared<ogma::ExplorationDirective>();
    dir->active          = true;
    dir->ticks_remaining = 5;
    dir->episode_id      = 1;
    dir->accel           = 2.5f;
    f.bus.publish(ogma::topics::kExplorationDirective, dir);
    f.dec.tick(0);
    f.bus.end_tick();

    auto a = f.last_action();
    ASSERT_NE(a, nullptr);
    EXPECT_FLOAT_EQ(a->accel, 2.5f);
    EXPECT_TRUE(a->probe);
    EXPECT_TRUE(f.dec.exploration_active());
}

// -- Determinism ----------------------------------------------------------

TEST(ActionDecoder, IdenticalSeedsAndStreamsProduceIdenticalActions) {
    auto run = []() {
        DecoderFixture f;
        std::vector<float> actions;
        for (uint64_t t = 0; t < 50; ++t) {
            f.bus.begin_tick(t);
            f.bus.publish("consensus.0",         make_consensus(4, int(t % 7)));
            f.bus.publish("drive.errors",        make_drive(0.4f));
            f.bus.publish("neuro.state",         make_neuro(0.4f, 0.7f, 0.1f));
            f.bus.publish("reality.proprio.imu", make_proprio_reality(int(t % 3)));
            f.dec.tick(t);
            f.bus.end_tick();
            actions.push_back(f.last_action()->accel);
        }
        return actions;
    };
    auto a = run();
    auto b = run();
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i) EXPECT_FLOAT_EQ(a[i], b[i]);
}

// -- Hot-mutation ---------------------------------------------------------

TEST(ActionDecoder, HotMutateGains) {
    DecoderFixture f;
    EXPECT_NO_THROW(f.dec.on_param_change("pragmatic_gain", ogma::ParamValue{20.0}));
    EXPECT_NO_THROW(f.dec.on_param_change("td_lambda",      ogma::ParamValue{0.5}));
}

TEST(ActionDecoder, ConstructionOnlyParamThrows) {
    DecoderFixture f;
    EXPECT_THROW(f.dec.on_param_change("proprio_topic", ogma::ParamValue{std::string("x")}),
                 std::invalid_argument);
    EXPECT_THROW(f.dec.on_param_change("master_seed",   ogma::ParamValue{int64_t{99}}),
                 std::invalid_argument);
}

TEST(ActionDecoder, UnknownParamThrows) {
    DecoderFixture f;
    EXPECT_THROW(f.dec.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}

// -- Hebbian table populated by repeat experience -------------------------

TEST(ActionDecoder, HebbianTableGrowsWithExperience) {
    DecoderFixture f;
    for (uint64_t t = 0; t < 30; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",         make_consensus(4, int(t % 4)));
        f.bus.publish("drive.errors",        make_drive());
        f.bus.publish("neuro.state",         make_neuro(0.4f, 0.5f, 0.2f));
        f.bus.publish("reality.proprio.imu", make_proprio_reality(int(t % 2)));
        f.dec.tick(t);
        f.bus.end_tick();
    }
    EXPECT_GT(f.dec.hebbian_size(), 0u);
}

// -- Phase 6.5.3.1: forward model + action TLE -----------------------------

TEST(ActionDecoder, ForwardModelPopulatesWithExperience) {
    DecoderFixture f;
    EXPECT_EQ(f.dec.forward_model_size(), 0u);
    for (uint64_t t = 0; t < 30; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",         make_consensus(4, int(t % 4)));
        f.bus.publish("drive.errors",        make_drive());
        f.bus.publish("neuro.state",         make_neuro(0.4f, 0.5f, 0.2f));
        f.bus.publish("reality.proprio.imu", make_proprio_reality(int(t % 2)));
        f.dec.tick(t);
        f.bus.end_tick();
    }
    // After 30 ticks of structured (state, action) → s' transitions, the
    // forward-model table should contain entries.  The structure cycles
    // through 4 states × 2 proprio cells, so at least a handful of
    // (prev_state, prev_bin) keys should be populated.
    EXPECT_GT(f.dec.forward_model_size(), 0u);
}

TEST(ActionDecoder, ActionTleInRangeAndDecreasesWithRepetition) {
    DecoderFixture f;
    // First, drive a deterministic transition (state always 5, proprio 1)
    // so the forward model converges to P(5 | prev_state=5, prev_bin) = 1.
    for (uint64_t t = 0; t < 50; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",         make_consensus(4, 5));
        f.bus.publish("drive.errors",        make_drive());
        f.bus.publish("neuro.state",         make_neuro(0.4f, 0.5f, 0.2f));
        f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
        f.dec.tick(t);
        f.bus.end_tick();
    }
    // After convergence the published action_tle should be near 0 (the
    // forward model now predicts state 5 with high probability).  We
    // also verify the value is in [0, 1].
    auto a = f.last_action();
    ASSERT_NE(a, nullptr);
    EXPECT_GE(a->action_tle, 0.0f);
    EXPECT_LE(a->action_tle, 1.0f);
    EXPECT_LT(a->action_tle, 0.1f);  // should be predictable by now
}

TEST(ActionDecoder, ActionTleSpikesOnNovelTransition) {
    DecoderFixture f;
    // Establish a stable transition pattern.
    for (uint64_t t = 0; t < 30; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",         make_consensus(4, 5));
        f.bus.publish("drive.errors",        make_drive());
        f.bus.publish("neuro.state",         make_neuro(0.4f, 0.5f, 0.2f));
        f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
        f.dec.tick(t);
        f.bus.end_tick();
    }
    // Now break the pattern: introduce an unseen next-state.
    f.bus.begin_tick(99);
    f.bus.publish("consensus.0",         make_consensus(4, 17));   // unseen
    f.bus.publish("drive.errors",        make_drive());
    f.bus.publish("neuro.state",         make_neuro(0.4f, 0.5f, 0.2f));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.dec.tick(99);
    f.bus.end_tick();
    // The transition (prev_state=5, prev_bin) → 17 was never observed,
    // so action_tle should be near 1.0.
    auto a = f.last_action();
    ASSERT_NE(a, nullptr);
    EXPECT_GT(a->action_tle, 0.5f);
}

// =============================================================================
// v5.4.J — chunk armed-state Schmitt-trigger gate
// =============================================================================
// Pre-fix: a chunk just created from an eat event has entry_embeddings = the
// current post-eat context.  The next try_dispatch_chunk tick sees match ≈ 1
// (since current entry_history hasn't moved) → chunk fires immediately,
// pulling the agent away from the food spot.
//
// Post-fix: each episodic chunk is armed=false on first observation.  It
// re-arms only after current entry-match drops below rearm_threshold (the
// context has "left").  Then it can fire when match rises above
// entry_match_threshold (the context has "returned").  Edge-triggered.

namespace {

ogma::ParamMap entry_match_params() {
    auto p = default_params();
    p["use_chunks"]                = true;  // enable the kMotorChunks +
                                            //   entry-topic subscriptions
                                            //   (default false).
    p["entry_topic"]               = std::string("reality.slow.consensus");
    p["entry_keyframes"]           = int64_t{2};
    p["entry_match_threshold"]     = 0.70;
    p["chunk_rearm_threshold"]     = 0.40;
    p["min_chunk_score"]           = 0.0;   // disable Beta-gate so we
                                            //   isolate the armed-gate.
    p["chunk_dispatch_min_age_ticks"] = int64_t{0};  // v5.4.L — disable age
                                            //   gate so the Schmitt test
                                            //   below isolates entry-match
                                            //   semantics only.
    return p;
}

std::shared_ptr<ogma::RealityToken>
make_slow_consensus(Eigen::VectorXf const& latent) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->latent       = latent;
    r->winner_id    = 0;
    r->node_count   = 1;
    r->baked_count  = 1;
    r->tle          = 0.1f;
    r->is_novel     = false;
    return r;
}

std::shared_ptr<ogma::MotorChunks>
make_episodic_library(int chunk_id,
                      Eigen::VectorXf const& entry0,
                      Eigen::VectorXf const& entry1) {
    auto lib = std::make_shared<ogma::MotorChunks>();
    ogma::MotorChunk c;
    c.id               = chunk_id;
    c.entry_embeddings = {entry0, entry1};
    c.intent_sequence  = {2, 2, 2};        // any non-empty body
    c.hits_during      = 1;
    c.replay_hits      = 2.0f;
    c.replay_misses    = 0.0f;
    lib->chunks.push_back(std::move(c));
    return lib;
}

}  // namespace

TEST(ActionDecoder, EntryMatchSchmittGate_FreshChunkBlockedUntilArmed) {
    DecoderFixture f(entry_match_params());

    // The entry-history subscription deduplicates exact-repeat embeddings
    // ("slow EPM republish protection" — isApprox 1e-6), so each keyframe
    // we feed has to be byte-distinct from its predecessor.  Pick four
    // near-identical-direction vectors in two clusters: A-cluster (a1,a2)
    // and B-cluster (b1,b2).  Within a cluster vectors are nearly parallel
    // (cosine > rearm threshold); across clusters orthogonal (cosine = 0,
    // well below rearm threshold so the arm-edge triggers).
    Eigen::VectorXf A1(4); A1 << 1.0f, 0.1f, 0.0f, 0.0f;
    Eigen::VectorXf A2(4); A2 << 1.0f, 0.0f, 0.1f, 0.0f;
    Eigen::VectorXf B1(4); B1 << 0.0f, 1.0f, 0.1f, 0.0f;
    Eigen::VectorXf B2(4); B2 << 0.0f, 0.1f, 1.0f, 0.0f;

    // Chunk's entry_embeddings = (A1, A2): the post-eat context the
    // operator wants to replay later.
    auto lib = make_episodic_library(7, A1, A2);

    // 1) Fill entry-history with (A1, A2) — same context as the chunk's
    //    entry.  Library arrives in tick 2.  The chunk is born unarmed →
    //    dispatch must be blocked even though match = 1.0.
    f.bus.begin_tick(1);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(A1));
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(1);
    f.bus.end_tick();
    f.bus.begin_tick(2);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(A2));
    f.bus.publish(ogma::topics::kMotorChunks, lib);
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(2);
    f.bus.end_tick();

    EXPECT_EQ(f.dec.chunks_armed_count(), 0)
        << "fresh chunk must start armed=false to prevent eat→replay loop";
    EXPECT_GE(f.dec.dispatches_blocked_unarmed(), 1)
        << "Schmitt gate must block dispatch when chunk hasn't re-armed";
    EXPECT_EQ(f.dec.entry_match_dispatches(), 0)
        << "no entry-match dispatch should have happened yet";

    // 2) Move context to (B1, B2).  Cosine(B*, A*) = 0.099/1.01 ≈ 0.098 —
    //    well below rearm_threshold=0.40, so the chunk arms.
    f.bus.begin_tick(3);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(B1));
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(3);
    f.bus.end_tick();
    f.bus.begin_tick(4);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(B2));
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(4);
    f.bus.end_tick();
    EXPECT_EQ(f.dec.chunks_armed_count(), 1)
        << "chunk should have re-armed after context dropped below rearm threshold";

    // 3) Return to (A1, A2).  Match climbs back ≥ dispatch threshold AND
    //    armed=true → the chunk dispatches once.
    f.bus.begin_tick(5);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(A1));
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(5);
    f.bus.end_tick();
    f.bus.begin_tick(6);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(A2));
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(6);
    f.bus.end_tick();
    EXPECT_EQ(f.dec.entry_match_dispatches(), 1)
        << "chunk should dispatch once on re-entry to similar context";
    EXPECT_EQ(f.dec.chunks_armed_count(), 0)
        << "chunk should disarm after dispatching";
}

// =============================================================================
// v5.4.L — chunk dispatch age gate
// =============================================================================
// The Schmitt entry-match gate alone can't prevent the eat→replay loop when
// the slow consensus encoding is degenerate (cosine ≈ 1 across all hit
// moments).  A chunk just captured from an eat event re-arms immediately
// and dispatches on the very next tick.  The age gate is an independent
// fallback: any chunk younger than chunk_dispatch_min_age_ticks is
// blocked regardless of cosine match.

TEST(ActionDecoder, AgeGateBlocksFreshChunks) {
    // Default age gate of 60 ticks.  Disable Schmitt by setting
    // rearm threshold above any possible cosine so the chunk is
    // permanently armed — isolates the age gate from the Schmitt path.
    auto p = entry_match_params();
    p["chunk_dispatch_min_age_ticks"] = int64_t{60};   // restore default
    p["chunk_rearm_threshold"]        = 1.5;            // never re-arm via cosine drop
    p["entry_match_threshold"]        = 0.0;            // always pass cosine gate
    DecoderFixture f(p);

    Eigen::VectorXf A1(4); A1 << 1.0f, 0.1f, 0.0f, 0.0f;
    Eigen::VectorXf A2(4); A2 << 1.0f, 0.0f, 0.1f, 0.0f;
    auto lib = make_episodic_library(7, A1, A2);
    // Stamp the chunk's birth tick at tick=10.
    lib->chunks[0].created_tick_id = 10;

    // Fill the entry history with matching context so cosine gate
    // always passes.
    f.bus.begin_tick(20);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(A1));
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(20);
    f.bus.end_tick();

    // Publish library at tick 30.  Chunk age at this point = 30-10 = 20 < 60.
    f.bus.begin_tick(30);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(A2));
    f.bus.publish(ogma::topics::kMotorChunks, lib);
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(30);
    f.bus.end_tick();

    // Age = 20.  Below threshold → should block, NOT dispatch.
    EXPECT_GE(f.dec.dispatches_blocked_too_young(), 1)
        << "age gate must block dispatch for chunk younger than min_age_ticks";
    EXPECT_EQ(f.dec.entry_match_dispatches(), 0)
        << "no entry-match dispatch should happen while chunk is too young";

    // Advance to tick 80.  Chunk age = 80-10 = 70 ≥ 60 → dispatch allowed.
    f.bus.begin_tick(80);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(A1));
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(80);
    f.bus.end_tick();
    f.bus.begin_tick(81);
    f.bus.publish("reality.slow.consensus", make_slow_consensus(A2));
    f.bus.publish("consensus.0", make_consensus(8, 0));
    f.bus.publish("reality.proprio.imu", make_proprio_reality(1));
    f.bus.publish(ogma::topics::kDriveErrors, make_drive());
    f.dec.tick(81);
    f.bus.end_tick();
    EXPECT_GE(f.dec.entry_match_dispatches(), 1)
        << "age gate should NOT block dispatch once chunk crosses min_age_ticks";
}

