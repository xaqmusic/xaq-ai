// =============================================================================
// test_lateral_voter.cpp  --  Unit tests for the v4 LateralVoter module
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::RealityToken>
make_token(int dim, int winner_id, float tle, float qe = 0.0f, float fill = 1.0f) {
    auto t = std::make_shared<ogma::RealityToken>();
    t->winner_id   = winner_id;
    t->tle         = tle;
    t->quant_error = qe;
    t->latent      = Eigen::VectorXf::Constant(dim, fill);
    return t;
}

ogma::ParamMap default_params() {
    return {
        {"level",          int64_t{0}},
        {"input_pattern",  std::string("reality.")},
        {"trust_epsilon",  0.05},
        {"group_balance",  true},
        {"priority_group", std::string("proprio")},
    };
}

struct VoterFixture {
    ogma::InProcessBus  bus;
    ogma::LateralVoter  voter;

    explicit VoterFixture(ogma::ParamMap const& params = default_params()) {
        voter.set_id("voter_0");
        voter.on_setup(&bus, params);
    }

    std::shared_ptr<const ogma::ConsensusToken> last_consensus() const {
        return std::dynamic_pointer_cast<const ogma::ConsensusToken>(
            bus.last_value("consensus.0"));
    }
};

} // namespace

// -- Construction / contract -----------------------------------------------

TEST(LateralVoter, ConstructsAndPublishesEveryTick) {
    VoterFixture f;
    EXPECT_EQ(f.voter.type_name(), "LateralVoter");
    EXPECT_EQ(f.voter.output_topics()[0].name, "consensus.0");

    f.bus.begin_tick(0);
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->level, 0);
    EXPECT_EQ(cons->tick_id, 0u);
}

TEST(LateralVoter, PublishesOnConsensusN) {
    auto p = default_params();
    p["level"]         = int64_t{2};
    p["input_pattern"] = std::string("consensus.1.");
    VoterFixture f(p);
    EXPECT_EQ(f.voter.output_topics()[0].name, "consensus.2");
}

// -- Single-modality fusion -----------------------------------------------

TEST(LateralVoter, SingleInputPassesThrough) {
    VoterFixture f;
    f.bus.begin_tick(0);
    f.bus.publish("reality.proprio.imu", make_token(8, 5, 0.1f, 0.05f, 0.7f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->fused_embedding.size(), 8);
    // With one input, fused_embedding equals the input latent (trust = 1.0).
    EXPECT_FLOAT_EQ(cons->fused_embedding[0], 0.7f);
    EXPECT_FLOAT_EQ(cons->fused_tle, 0.1f);
    EXPECT_EQ(cons->active_winner_id, 5);
    EXPECT_EQ(cons->active_modality, "proprio.imu");
}

// -- Modality-group balancing --------------------------------------------

TEST(LateralVoter, GroupBalancingSplitsTrustEqually) {
    VoterFixture f;   // group_balance = true (default)

    // Two video modalities + one proprio.  With balancing, video group's
    // total trust = 0.5 (split between its two members), proprio = 0.5.
    f.bus.begin_tick(0);
    f.bus.publish("reality.video.retinal",  make_token(4, 1, 0.1f, 0.0f, 1.0f));
    f.bus.publish("reality.video.saliency", make_token(4, 2, 0.1f, 0.0f, 1.0f));
    f.bus.publish("reality.proprio.imu",    make_token(4, 3, 0.1f, 0.0f, 1.0f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    float vid_total = cons->trust_weights.at("reality.video.retinal")
                    + cons->trust_weights.at("reality.video.saliency");
    float pro_total = cons->trust_weights.at("reality.proprio.imu");
    EXPECT_NEAR(vid_total, 0.5f, 1e-4f);
    EXPECT_NEAR(pro_total, 0.5f, 1e-4f);
}

TEST(LateralVoter, NoGroupBalanceUsesGlobalNormalization) {
    auto p = default_params();
    p["group_balance"] = false;
    VoterFixture f(p);

    // Three inputs, equal TLE → each gets 1/3 trust.
    f.bus.begin_tick(0);
    f.bus.publish("reality.video.retinal",  make_token(4, 1, 0.1f, 0.0f, 1.0f));
    f.bus.publish("reality.video.saliency", make_token(4, 2, 0.1f, 0.0f, 1.0f));
    f.bus.publish("reality.proprio.imu",    make_token(4, 3, 0.1f, 0.0f, 1.0f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_NEAR(cons->trust_weights.at("reality.video.retinal"),  1.0f/3.0f, 1e-4f);
    EXPECT_NEAR(cons->trust_weights.at("reality.video.saliency"), 1.0f/3.0f, 1e-4f);
    EXPECT_NEAR(cons->trust_weights.at("reality.proprio.imu"),    1.0f/3.0f, 1e-4f);
}

// -- TLE-inverse trust ----------------------------------------------------

TEST(LateralVoter, LowerTleProducesHigherTrust) {
    auto p = default_params();
    p["group_balance"] = false;   // simpler verification with global normalization
    VoterFixture f(p);

    f.bus.begin_tick(0);
    f.bus.publish("reality.video.retinal",  make_token(4, 1, 0.05f, 0.0f, 1.0f));   // low TLE → high trust
    f.bus.publish("reality.video.saliency", make_token(4, 2, 0.50f, 0.0f, 1.0f));   // high TLE → low trust
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_GT(cons->trust_weights.at("reality.video.retinal"),
              cons->trust_weights.at("reality.video.saliency"));
}

// -- Active modality selection -------------------------------------------

TEST(LateralVoter, PriorityGroupWinsTies) {
    VoterFixture f;   // priority_group = "proprio"

    // Two inputs from different groups, equal TLE → equal trust → priority wins.
    f.bus.begin_tick(0);
    f.bus.publish("reality.video.retinal", make_token(4, 1, 0.1f, 0.05f, 1.0f));
    f.bus.publish("reality.proprio.imu",   make_token(4, 2, 0.1f, 0.05f, 1.0f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->active_modality, "proprio.imu");
    EXPECT_EQ(cons->active_winner_id, 2);
}

TEST(LateralVoter, HighestTrustGroupWinsWhenNotTied) {
    // With group_balance=true, every group gets exactly 1/N mass so TLE-based
    // dominance can't shift groups; priority_group wins.  To exercise
    // "highest trust group wins" we have to disable group balance.
    auto p = default_params();
    p["group_balance"] = false;
    VoterFixture f(p);

    f.bus.begin_tick(0);
    f.bus.publish("reality.video.retinal", make_token(4, 1, 0.05f, 0.05f, 1.0f));
    f.bus.publish("reality.proprio.imu",   make_token(4, 2, 0.50f, 0.05f, 1.0f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    // video's lower TLE produces higher trust → wins despite proprio being
    // the priority_group, because priority only breaks TIES.
    EXPECT_EQ(cons->active_modality.substr(0, 5), "video");
}

// -- Active winner is min-QE within active group --------------------------

TEST(LateralVoter, ActiveWinnerIsMinQeInActiveGroup) {
    VoterFixture f;

    f.bus.begin_tick(0);
    // Two video modalities — same group, same TLE, different QE.
    f.bus.publish("reality.video.retinal",  make_token(4, 11, 0.1f, 0.30f, 1.0f));
    f.bus.publish("reality.video.saliency", make_token(4, 22, 0.1f, 0.10f, 1.0f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->active_winner_id, 22);   // the lower-QE member wins
}

// -- First-tick semantics -------------------------------------------------

TEST(LateralVoter, FirstTickWithNoInputsPlaceholder) {
    VoterFixture f;
    f.bus.begin_tick(0);
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->level, 0);
    EXPECT_FLOAT_EQ(cons->fused_tle, 0.0f);
    EXPECT_EQ(cons->active_winner_id, -1);
}

TEST(LateralVoter, BootstrapTokenWithWinnerNegOneIsExcluded) {
    VoterFixture f;
    f.bus.begin_tick(0);
    // EPM bootstrap token: winner_id = -1 → voter must exclude it from fusion.
    f.bus.publish("reality.video.retinal", make_token(4, -1, 0.0f, 0.0f, 1.0f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->active_winner_id, -1);   // no real input → placeholder
}

TEST(LateralVoter, EmptyTickRepublishesPriorWithNewTickId) {
    VoterFixture f;

    // Tick 0: feed one real input.
    f.bus.begin_tick(0);
    f.bus.publish("reality.proprio.imu", make_token(4, 99, 0.1f, 0.05f, 0.5f));
    f.voter.tick(0);
    f.bus.end_tick();
    auto c0 = f.last_consensus();
    ASSERT_NE(c0, nullptr);
    EXPECT_EQ(c0->tick_id, 0u);
    EXPECT_EQ(c0->active_winner_id, 99);

    // Tick 1: no inputs.  Voter should republish prior token with tick_id=1.
    f.bus.begin_tick(1);
    f.voter.tick(1);
    f.bus.end_tick();
    auto c1 = f.last_consensus();
    ASSERT_NE(c1, nullptr);
    EXPECT_EQ(c1->tick_id, 1u);
    EXPECT_EQ(c1->active_winner_id, 99);
    EXPECT_FLOAT_EQ(c1->fused_embedding[0], c0->fused_embedding[0]);
}

// -- Trust-weight invariants ---------------------------------------------

TEST(LateralVoter, TrustWeightsSumToOne) {
    VoterFixture f;
    f.bus.begin_tick(0);
    f.bus.publish("reality.video.retinal",  make_token(4, 1, 0.05f, 0.0f, 1.0f));
    f.bus.publish("reality.video.saliency", make_token(4, 2, 0.20f, 0.0f, 1.0f));
    f.bus.publish("reality.audio.stft", make_token(4, 3, 0.10f, 0.0f, 1.0f));
    f.bus.publish("reality.proprio.imu",    make_token(4, 4, 0.30f, 0.0f, 1.0f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    float total = 0.0f;
    for (auto const& [k, v] : cons->trust_weights) total += v;
    EXPECT_NEAR(total, 1.0f, 1e-4f);
}

// -- Determinism ----------------------------------------------------------

TEST(LateralVoter, DeterministicAcrossTwoIdenticalRuns) {
    auto run = []() {
        VoterFixture f;
        for (uint64_t t = 0; t < 30; ++t) {
            f.bus.begin_tick(t);
            f.bus.publish("reality.video.retinal",  make_token(4, int(t),     0.1f + 0.05f*(t%3), 0.0f, 1.0f));
            f.bus.publish("reality.video.saliency", make_token(4, int(t+100), 0.2f - 0.02f*(t%5), 0.0f, 1.0f));
            f.bus.publish("reality.proprio.imu",    make_token(4, int(t+200), 0.05f,              0.0f, 1.0f));
            f.voter.tick(t);
            f.bus.end_tick();
        }
        return f.last_consensus()->fused_embedding;
    };
    auto a = run();
    auto b = run();
    ASSERT_EQ(a.size(), b.size());
    for (int i = 0; i < a.size(); ++i) EXPECT_FLOAT_EQ(a[i], b[i]);
}

// -- Hot-mutation --------------------------------------------------------

TEST(LateralVoter, HotMutateGroupBalance) {
    VoterFixture f;
    EXPECT_NO_THROW(f.voter.on_param_change("group_balance", ogma::ParamValue{false}));
    EXPECT_NO_THROW(f.voter.on_param_change("priority_group", ogma::ParamValue{std::string("audio")}));
}

TEST(LateralVoter, ConstructionOnlyParamThrowsOnHotMutate) {
    VoterFixture f;
    EXPECT_THROW(f.voter.on_param_change("level", ogma::ParamValue{int64_t{1}}),
                 std::invalid_argument);
    EXPECT_THROW(f.voter.on_param_change("input_pattern", ogma::ParamValue{std::string("consensus.")}),
                 std::invalid_argument);
}

TEST(LateralVoter, UnknownParamThrows) {
    VoterFixture f;
    EXPECT_THROW(f.voter.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}

// =============================================================================
// Phase 6.6.M — input_exclude as list of prefixes (back-compat: still
// accepts a single string).  Bilateral whisker split needs the top voter
// to suppress two raw mid-voter input prefixes simultaneously.
// =============================================================================

TEST(LateralVoter, MultiExcludeFiltersBothPrefixes) {
    auto p = default_params();
    p["input_exclude"] = std::vector<std::string>{
        std::string("reality.whisker_left."),
        std::string("reality.whisker_right.")
    };
    VoterFixture f(p);

    f.bus.begin_tick(0);
    // Should be IGNORED — both excluded prefixes:
    f.bus.publish("reality.whisker_left.whisker_0",  make_token(8, 1, 0.1f));
    f.bus.publish("reality.whisker_right.whisker_3", make_token(8, 2, 0.1f));
    // Should be ACCEPTED — neither prefix matches:
    f.bus.publish("reality.fused.whisker_left",  make_token(8, 10, 0.05f));
    f.bus.publish("reality.fused.whisker_right", make_token(8, 11, 0.05f));
    f.bus.publish("reality.proprio.imu",          make_token(8, 20, 0.05f));
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    auto const& trust = cons->trust_weights;
    EXPECT_EQ(trust.count("reality.whisker_left.whisker_0"),  0u);
    EXPECT_EQ(trust.count("reality.whisker_right.whisker_3"), 0u);
    EXPECT_EQ(trust.count("reality.fused.whisker_left"),  1u);
    EXPECT_EQ(trust.count("reality.fused.whisker_right"), 1u);
    EXPECT_EQ(trust.count("reality.proprio.imu"),          1u);
}

TEST(LateralVoter, SingleStringExcludeStillSupported) {
    // 6.6.M back-compat: legacy configs that pass a single string for
    // input_exclude must still work.
    auto p = default_params();
    p["input_exclude"] = std::string("reality.whisker.");
    VoterFixture f(p);

    f.bus.begin_tick(0);
    f.bus.publish("reality.whisker.whisker_0", make_token(8, 1, 0.1f));   // excluded
    f.bus.publish("reality.proprio.imu",        make_token(8, 2, 0.05f)); // accepted
    f.voter.tick(0);
    f.bus.end_tick();

    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->trust_weights.count("reality.whisker.whisker_0"), 0u);
    EXPECT_EQ(cons->trust_weights.count("reality.proprio.imu"),       1u);
}
