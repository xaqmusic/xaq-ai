// =============================================================================
// pair_homeostatic_actiondecoder.cpp
//
// Real HomeostaticDrive ↔ real ActionDecoder.  Phase 1.5 upgrade: the
// MockActionDecoder used in Phase 1.4 is gone per the no-mocks-survive-
// Phase-1 rule.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/modules/HomeostaticDrive.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap drive_params(double drain_rate = 0.0) {
    return {
        {"channels",            std::vector<std::string>{"energy", "integrity", "novelty_satiation"}},
        {"setpoints",           std::vector<double>{0.8, 1.0, 0.5}},
        {"urgency_normalizers", std::vector<double>{1.0, 1.0, 0.5}},
        {"channel_input_topics",
            std::vector<std::string>{
                "reality.proprio.energy",
                "reality.proprio.integrity",
                "consensus.0",
            }},
        {"energy_drain_per_tick",  drain_rate},
        {"integrity_drain_per_miss", 0.10},
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

std::shared_ptr<ogma::ConsensusToken>
make_consensus(int active_winner_id) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding   = Eigen::VectorXf::Constant(4, 0.5f);
    c->fused_tle         = 0.1f;
    c->level             = 0;
    c->active_winner_id  = active_winner_id;
    c->active_modality   = "proprio.imu";
    return c;
}

std::shared_ptr<ogma::NeuroState> make_neuro(float da = 0.20f, float ht = 0.65f, float reward = 0.0f) {
    auto n = std::make_shared<ogma::NeuroState>();
    n->dopamine        = da;
    n->serotonin       = ht;
    n->reward_signal   = reward;
    n->epsilon_b_scale = 1.0f;
    return n;
}

std::shared_ptr<ogma::RealityToken> make_proprio_reality(int winner_id) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->winner_id = winner_id;
    r->latent    = Eigen::VectorXf::Constant(4, 0.0f);
    return r;
}

std::shared_ptr<ogma::EnvEvent> make_event(std::string name) {
    auto e = std::make_shared<ogma::EnvEvent>();
    e->name = std::move(name);
    e->intensity = 1.0f;
    return e;
}

} // namespace

TEST(PairHomeostaticActionDecoder, DriveErrorsReachActionDecoder) {
    ogma::InProcessBus      bus;
    ogma::HomeostaticDrive  drive;
    ogma::ActionDecoder     decoder;
    drive.set_id("drive");
    decoder.set_id("action_decoder");
    drive.on_setup(&bus, drive_params(/*drain=*/0.0));
    decoder.on_setup(&bus, decoder_params());

    for (uint64_t t = 0; t < 30; ++t) {
        bus.begin_tick(t);
        bus.publish("consensus.0",         make_consensus(int(t)));
        bus.publish("neuro.state",         make_neuro());
        bus.publish("reality.proprio.imu", make_proprio_reality(int(t % 3)));
        drive.tick(t);
        decoder.tick(t);
        bus.end_tick();
    }

    auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionOut));
    ASSERT_NE(act, nullptr);
    EXPECT_FALSE(std::isnan(act->accel));
}

TEST(PairHomeostaticActionDecoder, FirstTickProducesNoNan) {
    ogma::InProcessBus      bus;
    ogma::HomeostaticDrive  drive;
    ogma::ActionDecoder     decoder;
    drive.set_id("drive");
    decoder.set_id("action_decoder");
    drive.on_setup(&bus, drive_params(0.0));
    decoder.on_setup(&bus, decoder_params());

    EXPECT_NO_THROW({
        bus.begin_tick(0);
        drive.tick(0);
        decoder.tick(0);
        bus.end_tick();
    });

    auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionOut));
    ASSERT_NE(act, nullptr);
    EXPECT_FALSE(std::isnan(act->accel));
}
