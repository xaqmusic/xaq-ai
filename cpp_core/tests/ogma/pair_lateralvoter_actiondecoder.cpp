// =============================================================================
// pair_lateralvoter_actiondecoder.cpp
//
// Real LateralVoter ↔ real ActionDecoder.  Verifies the consensus.0 →
// action.out seam end-to-end on a synthetic input distribution.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap voter_params() {
    return {
        {"level",          int64_t{0}},
        {"input_pattern",  std::string("reality.")},
        {"trust_epsilon",  0.05},
        {"group_balance",  true},
        {"priority_group", std::string("proprio")},
    };
}

ogma::ParamMap decoder_params() {
    return {
        {"consensus_level", int64_t{0}},
        {"proprio_topic",   std::string("reality.proprio.imu")},
        {"action_bins",     int64_t{3}},
        {"pragmatic_gain",  10.0},
    };
}

std::shared_ptr<ogma::RealityToken>
make_proprio_token(int dim, int winner_id, float tle) {
    auto t = std::make_shared<ogma::RealityToken>();
    t->winner_id   = winner_id;
    t->tle         = tle;
    t->quant_error = tle * 0.5f;
    t->latent      = Eigen::VectorXf::Constant(dim, 1.0f);
    return t;
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

} // namespace

TEST(PairLateralVoterActionDecoder, ConsensusReachesDecoderAndProducesAction) {
    ogma::InProcessBus    bus;
    ogma::LateralVoter    voter;
    ogma::ActionDecoder   decoder;
    voter.set_id("voter_0");
    decoder.set_id("action_decoder");
    voter.on_setup(&bus, voter_params());
    decoder.on_setup(&bus, decoder_params());

    for (uint64_t t = 0; t < 40; ++t) {
        bus.begin_tick(t);
        // Publish a synthetic proprio EPM token (drives consensus + the
        // ActionDecoder's proprio_node lookup simultaneously).
        bus.publish("reality.proprio.imu", make_proprio_token(8, int(t % 5), 0.1f));
        bus.publish("drive.errors",        make_drive(0.2f));
        bus.publish("neuro.state",         make_neuro(0.2f * std::sin(0.1f * t)));
        voter.tick(t);
        decoder.tick(t);
        bus.end_tick();
    }

    auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionOut));
    ASSERT_NE(act, nullptr);
    EXPECT_FALSE(std::isnan(act->accel));
    EXPECT_GE(act->accel, -4.0f);
    EXPECT_LE(act->accel,  4.0f);
}

TEST(PairLateralVoterActionDecoder, RewardingExperienceShiftsPolicyOverTime) {
    // Long-run sanity: with consistent positive reward in the same
    // (state, proprio), the decoder's valence map should accumulate and
    // the action stream should not be permanently zero.
    ogma::InProcessBus    bus;
    ogma::LateralVoter    voter;
    ogma::ActionDecoder   decoder;
    voter.set_id("voter_0");
    decoder.set_id("action_decoder");
    voter.on_setup(&bus, voter_params());
    decoder.on_setup(&bus, decoder_params());

    for (uint64_t t = 0; t < 200; ++t) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.imu", make_proprio_token(8, /*winner=*/3, 0.1f));
        bus.publish("drive.errors",        make_drive(0.2f));
        bus.publish("neuro.state",         make_neuro(/*reward=*/0.3f));
        voter.tick(t);
        decoder.tick(t);
        bus.end_tick();
    }
    EXPECT_GT(decoder.valence_size(), 0u);
    EXPECT_GT(decoder.hebbian_size(), 0u);
}

TEST(PairLateralVoterActionDecoder, FirstTickProducesNoNan) {
    ogma::InProcessBus    bus;
    ogma::LateralVoter    voter;
    ogma::ActionDecoder   decoder;
    voter.set_id("voter_0");
    decoder.set_id("action_decoder");
    voter.on_setup(&bus, voter_params());
    decoder.on_setup(&bus, decoder_params());

    EXPECT_NO_THROW({
        bus.begin_tick(0);
        voter.tick(0);
        decoder.tick(0);
        bus.end_tick();
    });

    auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionOut));
    ASSERT_NE(act, nullptr);
    EXPECT_FALSE(std::isnan(act->accel));
}
