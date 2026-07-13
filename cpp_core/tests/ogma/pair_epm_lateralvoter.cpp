// =============================================================================
// pair_epm_lateralvoter.cpp  --  EPM ↔ LateralVoter modality-group seam
// =============================================================================
//
// Per docs/primitives/_pair_tests.md.  Phase 1.3 upgrade: MockLateralVoter
// is gone.  This is now a real-EPM ↔ real-LateralVoter integration test.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap epm_rbf_params(std::string const& modality_name) {
    return {
        {"modality_group",     std::string("proprio")},
        {"modality_name",      modality_name},
        {"encoder_kind",       std::string("rbf")},
        {"input_topic",        std::string("reality.proprio.") + modality_name},
        {"projection_dim",     int64_t{16}},
        {"proprio_state_dims", int64_t{6}},
        {"baking_threshold",   int64_t{10}},
        {"min_insertion_error", 0.001},
        {"subtract_descending_prediction", false},
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

std::shared_ptr<ogma::ProprioToken> make_proprio6(float seed) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->sensor = "imu";
    p->values.resize(6);
    p->values << std::sin(seed),     std::cos(seed),
                 std::sin(2.f*seed), std::cos(2.f*seed),
                 0.5f * seed,        -0.5f * seed;
    return p;
}

void run_tick(ogma::InProcessBus&    bus,
              ogma::EPM&             epm,
              ogma::LateralVoter&    voter,
              uint64_t               t,
              std::shared_ptr<ogma::ProprioToken> proprio) {
    bus.begin_tick(t);
    if (proprio) bus.publish("reality.proprio.imu", proprio);
    epm.tick(t);     // publishes reality.proprio.imu(t) — voter Direct fires
    voter.tick(t);   // publishes consensus.0(t)
    bus.end_tick();
}

} // namespace

TEST(PairEpmLateralVoter, EpmTokenReachesVoterAndConsensusIsPublished) {
    ogma::InProcessBus  bus;
    ogma::EPM           epm;
    ogma::LateralVoter  voter;
    epm.set_id("epm_imu");
    voter.set_id("voter_0");
    voter.on_setup(&bus, voter_params());
    epm.on_setup(&bus, epm_rbf_params("imu"));

    for (uint64_t t = 0; t < 30; ++t)
        run_tick(bus, epm, voter, t, make_proprio6(0.1f * t));

    auto cons = std::dynamic_pointer_cast<const ogma::ConsensusToken>(
        bus.last_value("consensus.0"));
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->level, 0);
    EXPECT_EQ(cons->fused_embedding.size(), 16);
    EXPECT_GE(cons->active_winner_id, 0);
}

TEST(PairEpmLateralVoter, VoterParsesModalityGroupFromTopic) {
    ogma::InProcessBus  bus;
    ogma::EPM           epm;
    ogma::LateralVoter  voter;
    epm.set_id("epm_imu");
    voter.set_id("voter_0");
    voter.on_setup(&bus, voter_params());
    epm.on_setup(&bus, epm_rbf_params("imu"));

    for (uint64_t t = 0; t < 30; ++t)
        run_tick(bus, epm, voter, t, make_proprio6(0.1f * t));

    auto cons = std::dynamic_pointer_cast<const ogma::ConsensusToken>(
        bus.last_value("consensus.0"));
    ASSERT_NE(cons, nullptr);
    // Topic was "reality.proprio.imu" → group "proprio".
    EXPECT_EQ(cons->active_modality, "proprio.imu");
}

TEST(PairEpmLateralVoter, FirstTickPlaceholderSurvivesVoter) {
    ogma::InProcessBus  bus;
    ogma::EPM           epm;
    ogma::LateralVoter  voter;
    epm.set_id("epm_imu");
    voter.set_id("voter_0");
    voter.on_setup(&bus, voter_params());
    epm.on_setup(&bus, epm_rbf_params("imu"));

    EXPECT_NO_THROW({
        bus.begin_tick(0);
        epm.tick(0);     // bootstrap token — winner_id = -1
        voter.tick(0);
        bus.end_tick();
    });

    auto cons = std::dynamic_pointer_cast<const ogma::ConsensusToken>(
        bus.last_value("consensus.0"));
    ASSERT_NE(cons, nullptr);
    // Voter excludes bootstrap tokens → first-tick placeholder consensus.
    EXPECT_EQ(cons->active_winner_id, -1);
    for (int i = 0; i < cons->fused_embedding.size(); ++i)
        EXPECT_FALSE(std::isnan(cons->fused_embedding[i]));
}
