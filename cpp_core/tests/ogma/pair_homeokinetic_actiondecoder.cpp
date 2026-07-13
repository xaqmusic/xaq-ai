// =============================================================================
// pair_homeokinetic_actiondecoder.cpp
// =============================================================================
//
// HomeokineticExploration ↔ ActionDecoder.  Once the kinesis primitive arms
// an episode, ActionDecoder must override its EFE-selected action with the
// directive's held accel for `episode_ticks` ticks, then resume normal
// behaviour.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/modules/HomeokineticExploration.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap kinesis_params() {
    return {
        {"window_ticks",     int64_t{10}},
        {"urgency_rise_eps", 0.001},
        {"drive_flat_eps",   1e-3},
        {"chunk_match_eps",  0.01},
        {"episode_ticks",    int64_t{5}},
        {"cooldown_ticks",   int64_t{3}},
        {"accel_jitter",     2.0},
        {"master_seed",      int64_t{42}},
    };
}

ogma::ParamMap decoder_params() {
    return {
        {"consensus_level",  int64_t{0}},
        {"proprio_topic",    std::string("reality.proprio.imu")},
        {"action_bins",      int64_t{3}},
        {"pragmatic_gain",   10.0},
    };
}

std::shared_ptr<ogma::ConsensusToken> make_consensus(int wid) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding   = Eigen::VectorXf::Constant(4, 0.5f);
    c->level             = 0;
    c->active_winner_id  = wid;
    c->active_modality   = "proprio.imu";
    return c;
}
std::shared_ptr<ogma::NeuroState> make_neuro() {
    auto n = std::make_shared<ogma::NeuroState>();
    n->dopamine = 0.30f; n->serotonin = 0.50f; n->epsilon_b_scale = 1.0f;
    return n;
}
std::shared_ptr<ogma::DriveErrors> make_drive(float urgency) {
    auto d = std::make_shared<ogma::DriveErrors>();
    d->urgency = urgency;
    return d;
}
std::shared_ptr<ogma::RealityToken> make_proprio(int wid) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->winner_id = wid;
    return r;
}

} // namespace

TEST(PairHomeokineticActionDecoder, DirectiveOverridesEFE) {
    ogma::InProcessBus              bus;
    ogma::HomeokineticExploration   kin;
    ogma::ActionDecoder             dec;
    kin.set_id("kinesis");
    dec.set_id("action_decoder");
    kin.on_setup(&bus, kinesis_params());
    dec.on_setup(&bus, decoder_params());

    // Run 15 ticks of slowly-rising flat urgency with valid consensus +
    // proprio (so the decoder has signal to choose from in the absence of
    // the directive).  After ~10 ticks the kinesis gate should arm.
    int  override_count   = 0;
    int  efe_count        = 0;
    float captured_accel  = 0.0f;

    for (int t = 0; t < 15; ++t) {
        bus.begin_tick(uint64_t(t));
        bus.publish("consensus.0",         make_consensus(5));
        bus.publish("drive.errors",        make_drive(0.5f + 0.002f * float(t)));
        bus.publish("neuro.state",         make_neuro());
        bus.publish("reality.proprio.imu", make_proprio(2));
        kin.tick(uint64_t(t));   // publishes exploration.directive
        dec.tick(uint64_t(t));   // consumes directive + emits action
        bus.end_tick();

        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(ogma::topics::kActionOut));
        ASSERT_NE(a, nullptr);
        if (a->probe) {
            ++override_count;
            captured_accel = a->accel;
        } else {
            ++efe_count;
        }
    }

    // The directive must have armed at least once.
    EXPECT_GT(override_count, 0);
    EXPECT_TRUE(dec.exploration_active() || override_count > 0);

    // While overridden, captured_accel matches the directive's held value.
    auto last_directive = std::dynamic_pointer_cast<const ogma::ExplorationDirective>(
        bus.last_value(ogma::topics::kExplorationDirective));
    ASSERT_NE(last_directive, nullptr);
    if (last_directive->active) {
        EXPECT_FLOAT_EQ(captured_accel, last_directive->accel);
    }
}

TEST(PairHomeokineticActionDecoder, NoDirectiveLeavesEFEUnaffected) {
    ogma::InProcessBus              bus;
    ogma::HomeokineticExploration   kin;
    ogma::ActionDecoder             dec;
    kin.set_id("kinesis");
    dec.set_id("action_decoder");
    kin.on_setup(&bus, kinesis_params());
    dec.on_setup(&bus, decoder_params());

    // Falling urgency → kinesis gate never holds → directive always
    // active=false → ActionDecoder emits EFE-selected actions only.
    for (int t = 0; t < 20; ++t) {
        bus.begin_tick(uint64_t(t));
        bus.publish("consensus.0",         make_consensus(int(t % 5)));
        bus.publish("drive.errors",        make_drive(0.9f - 0.005f * float(t)));
        bus.publish("neuro.state",         make_neuro());
        bus.publish("reality.proprio.imu", make_proprio(int(t % 3)));
        kin.tick(uint64_t(t));
        dec.tick(uint64_t(t));
        bus.end_tick();

        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(ogma::topics::kActionOut));
        ASSERT_NE(a, nullptr);
        EXPECT_FALSE(a->probe) << "tick " << t << " unexpectedly took override path";
    }
    EXPECT_EQ(kin.episodes_armed(), 0u);
}
