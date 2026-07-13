// =============================================================================
// pair_lateralvoter_homeostatic.cpp
//
// Verifies the consensus.0 → novelty_satiation EMA seam.  Real LateralVoter
// publishes a synthetic ConsensusToken (driven by stub RealityToken inputs);
// real HomeostaticDrive's `novelty_satiation` channel EMA-tracks the
// fused_tle from those consensus tokens.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/HomeostaticDrive.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap drive_params() {
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
        {"novelty_satiation_alpha", 0.20},
        {"energy_drain_per_tick",   0.0},   // disable drain for cleaner test
    };
}

ogma::ParamMap voter_params() {
    return {
        {"level",          int64_t{0}},
        {"input_pattern",  std::string("reality.")},
        {"trust_epsilon",  0.05},
        {"group_balance",  true},
        {"priority_group", std::string("proprio")},
    };
}

std::shared_ptr<ogma::RealityToken>
make_synthetic_token(int dim, int winner_id, float tle) {
    auto t = std::make_shared<ogma::RealityToken>();
    t->winner_id   = winner_id;
    t->tle         = tle;
    t->quant_error = tle * 0.5f;
    t->latent      = Eigen::VectorXf::Constant(dim, 1.0f);
    return t;
}

} // namespace

TEST(PairLateralVoterHomeostatic, NoveltyEmaTracksFusedTle) {
    ogma::InProcessBus       bus;
    ogma::LateralVoter       voter;
    ogma::HomeostaticDrive   drive;
    voter.set_id("voter_0");
    drive.set_id("drive");
    voter.on_setup(&bus, voter_params());
    drive.on_setup(&bus, drive_params());

    // Drive 60 ticks with a high-TLE input → LateralVoter publishes
    // consensus.0 each tick → drive's novelty_satiation EMA climbs.
    for (uint64_t t = 0; t < 60; ++t) {
        bus.begin_tick(t);
        bus.publish("reality.video.retinal", make_synthetic_token(8, int(t), /*tle=*/0.9f));
        voter.tick(t);   // publishes consensus.0 with fused_tle ≈ 0.9
        drive.tick(t);   // EMA updates
        bus.end_tick();
    }

    EXPECT_GT(drive.current_value("novelty_satiation"), 0.6f);
    auto d = std::dynamic_pointer_cast<const ogma::DriveErrors>(
        bus.last_value(ogma::topics::kDriveErrors));
    ASSERT_NE(d, nullptr);
    EXPECT_GT(d->errors.at("novelty_satiation"), 0.1f);    // current > setpoint
}

TEST(PairLateralVoterHomeostatic, UrgencyDrivenByMaxChannelDeviation) {
    ogma::InProcessBus       bus;
    ogma::LateralVoter       voter;
    ogma::HomeostaticDrive   drive;
    voter.set_id("voter_0");
    drive.set_id("drive");
    voter.on_setup(&bus, voter_params());
    drive.on_setup(&bus, drive_params());

    // Drive heavy-novelty signal to push novelty deviation high.
    for (uint64_t t = 0; t < 100; ++t) {
        bus.begin_tick(t);
        bus.publish("reality.video.retinal", make_synthetic_token(8, int(t), /*tle=*/0.95f));
        voter.tick(t);
        drive.tick(t);
        bus.end_tick();
    }
    EXPECT_GT(drive.urgency(), 0.0f);
}

TEST(PairLateralVoterHomeostatic, FirstTickProducesNoNan) {
    ogma::InProcessBus       bus;
    ogma::LateralVoter       voter;
    ogma::HomeostaticDrive   drive;
    voter.set_id("voter_0");
    drive.set_id("drive");
    voter.on_setup(&bus, voter_params());
    drive.on_setup(&bus, drive_params());

    EXPECT_NO_THROW({
        bus.begin_tick(0);
        voter.tick(0);
        drive.tick(0);
        bus.end_tick();
    });

    auto d = std::dynamic_pointer_cast<const ogma::DriveErrors>(
        bus.last_value(ogma::topics::kDriveErrors));
    ASSERT_NE(d, nullptr);
    for (auto const& [k, v] : d->errors) EXPECT_FALSE(std::isnan(v));
    EXPECT_FALSE(std::isnan(d->urgency));
}
