// =============================================================================
// pair_gngrollout_actiondecoder.cpp  --  Phase 3 wiring
// =============================================================================
//
// Real GNGRollout ↔ real ActionDecoder.  Verifies that ActionDecoder, with
// `use_rollout = true`, issues rollout.query in its tick() and consumes the
// returned RolloutResult to compute epistemic value.  In Phase 1.5 this
// path was hard-coded to zero; Phase 3 enables the round-trip.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/modules/GNGRollout.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap rollout_params() {
    return {
        {"K_default",            int64_t{16}},
        {"M_default",            int64_t{4}},
        {"transition_smoothing", 0.01},
        {"max_concurrent_queries", int64_t{8}},
    };
}

ogma::ParamMap decoder_params(bool use_rollout) {
    return {
        {"consensus_level",  int64_t{0}},
        {"proprio_topic",    std::string("reality.proprio.imu")},
        {"action_bins",      int64_t{3}},
        {"pragmatic_gain",   10.0},
        {"epistemic_gain",   2.0},
        {"use_rollout",      use_rollout},
        {"rollout_K",        int64_t{8}},
        {"rollout_M",        int64_t{3}},
    };
}

std::shared_ptr<ogma::ConsensusToken> make_consensus(int active_winner_id, std::string mod = "test.x") {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding   = Eigen::VectorXf::Constant(4, 0.5f);
    c->level             = 0;
    c->active_winner_id  = active_winner_id;
    c->active_modality   = std::move(mod);
    return c;
}

std::shared_ptr<ogma::DriveErrors> make_drive(float urgency = 0.0f) {
    auto d = std::make_shared<ogma::DriveErrors>();
    d->urgency = urgency;
    return d;
}

std::shared_ptr<ogma::NeuroState> make_neuro(float reward = 0.0f) {
    auto n = std::make_shared<ogma::NeuroState>();
    n->dopamine        = 0.30f;
    n->serotonin       = 0.50f;
    n->reward_signal   = reward;
    n->epsilon_b_scale = 1.0f;
    return n;
}

std::shared_ptr<ogma::RealityToken> make_proprio_winner(int wid) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->winner_id = wid;
    return r;
}

std::shared_ptr<ogma::RealityToken> make_state_token(int wid) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->winner_id = wid;
    return r;
}

} // namespace

TEST(PairGNGRolloutActionDecoder, DecoderQueriesRolloutWhenWired) {
    ogma::InProcessBus     bus;
    ogma::GNGRollout       rollout;
    ogma::ActionDecoder    decoder;
    rollout.set_id("rollout_0");
    decoder.set_id("action_decoder");

    // Subscribe rollout BEFORE decoder so its query handler is registered
    // before the decoder publishes the query.
    rollout.on_setup(&bus, rollout_params());
    decoder.on_setup(&bus, decoder_params(/*use_rollout=*/true));

    // Prime the rollout's transition cache with a 3-cycle on "reality.test.x".
    int cycle[] = {1, 2, 3};
    for (uint64_t t = 0; t < 60; ++t) {
        bus.begin_tick(t);
        bus.publish("reality.test.x", make_state_token(cycle[t % 3]));
        rollout.tick(t);
        bus.end_tick();
    }

    // Now drive the decoder: it should query the rollout and use the returned
    // entropy as epistemic value.
    for (uint64_t t = 60; t < 80; ++t) {
        bus.begin_tick(t);
        bus.publish("consensus.0",         make_consensus(/*winner=*/1, "test.x"));
        bus.publish("drive.errors",        make_drive());
        bus.publish("neuro.state",         make_neuro(0.2f));
        bus.publish("reality.proprio.imu", make_proprio_winner(0));
        rollout.tick(t);
        decoder.tick(t);
        bus.end_tick();
    }
    EXPECT_GT(decoder.last_rollout_used_request_id(), 0u);
}

TEST(PairGNGRolloutActionDecoder, NoQueryWhenUseRolloutDisabled) {
    ogma::InProcessBus     bus;
    ogma::GNGRollout       rollout;
    ogma::ActionDecoder    decoder;
    rollout.set_id("rollout_0");
    decoder.set_id("action_decoder");
    rollout.on_setup(&bus, rollout_params());
    decoder.on_setup(&bus, decoder_params(/*use_rollout=*/false));

    for (uint64_t t = 0; t < 20; ++t) {
        bus.begin_tick(t);
        bus.publish("consensus.0",         make_consensus(1));
        bus.publish("drive.errors",        make_drive());
        bus.publish("neuro.state",         make_neuro());
        bus.publish("reality.proprio.imu", make_proprio_winner(0));
        rollout.tick(t);
        decoder.tick(t);
        bus.end_tick();
    }
    EXPECT_EQ(decoder.last_rollout_used_request_id(), 0u);
}

TEST(PairGNGRolloutActionDecoder, FirstTickProducesNoNan) {
    ogma::InProcessBus     bus;
    ogma::GNGRollout       rollout;
    ogma::ActionDecoder    decoder;
    rollout.set_id("rollout_0");
    decoder.set_id("action_decoder");
    rollout.on_setup(&bus, rollout_params());
    decoder.on_setup(&bus, decoder_params(true));

    EXPECT_NO_THROW({
        bus.begin_tick(0);
        rollout.tick(0);
        decoder.tick(0);
        bus.end_tick();
    });

    auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionOut));
    ASSERT_NE(a, nullptr);
    EXPECT_FALSE(std::isnan(a->accel));
}
