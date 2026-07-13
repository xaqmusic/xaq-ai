// =============================================================================
// test_lateral_voter_streams.cpp
//   Behavioral / stream-level tests for the v4 LateralVoter.
//
// test_lateral_voter.cpp covers per-tick correctness — math contracts, trust
// updates from one publish, group-balance arithmetic.  These tests cover the
// behavioral guarantees the consensus layer is *supposed* to provide over
// many ticks of synthetic traffic, before we trust them in the live brain:
//
//   1. Trust converges to predicted distribution under steady inputs.
//   2. Fused embedding is the trust-weighted mean of input latents
//      (geometric fusion correctness).
//   3. When two modalities point in opposite directions in latent space,
//      consensus follows the higher-trust one.
//   4. Trust shifts when a modality drops out and recovers when it returns.
//
// Pass/fail here is the green light to dig into action-chain / motor issues
// downstream knowing perception fusion is solid.
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

// Build a RealityToken with explicit latent vector (not just constant fill).
std::shared_ptr<ogma::RealityToken>
make_token_vec(int winner_id, float tle, Eigen::VectorXf const& latent,
               float qe = 0.05f) {
    auto t = std::make_shared<ogma::RealityToken>();
    t->winner_id   = winner_id;
    t->tle         = tle;
    t->quant_error = qe;
    t->latent      = latent;
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

    // Run one tick: begin → publish whatever payloads the caller supplied →
    // voter.tick(t) → end.  Caller stages publishes in a lambda before the
    // tick fires so handle_input has already filled pending_.
    template <typename F>
    void run_tick(uint64_t t, F&& publish_fn) {
        bus.begin_tick(t);
        publish_fn();
        voter.tick(t);
        bus.end_tick();
    }
};

} // namespace

// =============================================================================
// 1. Convergence under steady inputs.
//    Two modalities, distinct TLE values, identical inputs across N ticks.
//    The voter is essentially memoryless per-tick (trust is computed fresh
//    each call) so "convergence" here means "stable across consecutive ticks
//    when inputs are stable."  We verify that AND that the stable values
//    match the math: with group_balance=false, trust ∝ 1 / (|tle| + epsilon).
// =============================================================================

TEST(LateralVoterStreams, ConvergesUnderSteadyInputs) {
    auto p = default_params();
    p["group_balance"] = false;       // simpler closed-form prediction
    VoterFixture f(p);

    Eigen::VectorXf latent_a = Eigen::VectorXf::Ones(4);
    Eigen::VectorXf latent_b = Eigen::VectorXf::Ones(4);

    constexpr float kTleA = 0.05f;
    constexpr float kTleB = 0.50f;
    constexpr float kEps  = 0.05f;
    const float raw_a = 1.0f / (kTleA + kEps);
    const float raw_b = 1.0f / (kTleB + kEps);
    const float pred_a = raw_a / (raw_a + raw_b);
    const float pred_b = raw_b / (raw_a + raw_b);

    float last_a = -1.0f, last_b = -1.0f;
    for (uint64_t t = 0; t < 100; ++t) {
        f.run_tick(t, [&]() {
            f.bus.publish("reality.video.retinal",
                          make_token_vec(1, kTleA, latent_a));
            f.bus.publish("reality.video.saliency",
                          make_token_vec(2, kTleB, latent_b));
        });
        auto cons = f.last_consensus();
        ASSERT_NE(cons, nullptr);
        last_a = cons->trust_weights.at("reality.video.retinal");
        last_b = cons->trust_weights.at("reality.video.saliency");
    }

    EXPECT_NEAR(last_a, pred_a, 1e-3f)
        << "Steady-state trust should match TLE-inverse prediction";
    EXPECT_NEAR(last_b, pred_b, 1e-3f);
    EXPECT_GT(last_a, last_b)
        << "Lower-TLE modality should dominate trust";
}

// =============================================================================
// 2. Geometric fusion correctness.
//    Inputs with distinct, orthogonal latent vectors.  Fused embedding must
//    equal the trust-weighted sum of input latents component-by-component.
//    This is the substrate's central perceptual-fusion claim.
// =============================================================================

TEST(LateralVoterStreams, FusedEmbeddingIsTrustWeightedMean) {
    auto p = default_params();
    p["group_balance"] = false;
    VoterFixture f(p);

    Eigen::VectorXf latent_a = Eigen::VectorXf::Zero(4);
    Eigen::VectorXf latent_b = Eigen::VectorXf::Zero(4);
    latent_a[0] = 1.0f;            // A points along axis 0
    latent_b[1] = 1.0f;            // B points along axis 1 (orthogonal)

    f.run_tick(0, [&]() {
        f.bus.publish("reality.video.retinal",  make_token_vec(1, 0.05f, latent_a));
        f.bus.publish("reality.video.saliency", make_token_vec(2, 0.20f, latent_b));
    });
    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    ASSERT_EQ(cons->fused_embedding.size(), 4);

    float ta = cons->trust_weights.at("reality.video.retinal");
    float tb = cons->trust_weights.at("reality.video.saliency");

    // Fused = ta * latent_a + tb * latent_b.  Since A and B are orthogonal
    // unit axes, fused[0] should equal ta and fused[1] should equal tb.
    EXPECT_NEAR(cons->fused_embedding[0], ta, 1e-4f)
        << "fused[0] should equal trust_A (A is the only contributor on axis 0)";
    EXPECT_NEAR(cons->fused_embedding[1], tb, 1e-4f)
        << "fused[1] should equal trust_B (B is the only contributor on axis 1)";
    EXPECT_NEAR(cons->fused_embedding[2], 0.0f, 1e-4f);
    EXPECT_NEAR(cons->fused_embedding[3], 0.0f, 1e-4f);
}

// =============================================================================
// 3. Disagreement resolution.
//    Two modalities point in opposite directions in latent space, with
//    significantly different TLEs.  The fused embedding must lean toward the
//    lower-TLE (higher-trust) modality.  Geometrically: dot(fused, latent_low)
//    should be larger than dot(fused, latent_high).
// =============================================================================

TEST(LateralVoterStreams, DisagreementFavoursLowerTle) {
    auto p = default_params();
    p["group_balance"] = false;
    VoterFixture f(p);

    Eigen::VectorXf latent_low  = Eigen::VectorXf::Zero(4);
    Eigen::VectorXf latent_high = Eigen::VectorXf::Zero(4);
    latent_low [0] =  1.0f;
    latent_high[0] = -1.0f;        // exact opposite direction on axis 0

    f.run_tick(0, [&]() {
        f.bus.publish("reality.video.retinal",
                      make_token_vec(1, 0.02f, latent_low));   // confident
        f.bus.publish("reality.video.saliency",
                      make_token_vec(2, 0.50f, latent_high));  // noisy
    });
    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);

    // Net fused[0] should be positive (the low-TLE direction) and the magnitude
    // should be at least 0.5 of the latent magnitude — i.e., the disagreement
    // didn't merely cancel; the lower-TLE modality clearly won.
    EXPECT_GT(cons->fused_embedding[0], 0.5f)
        << "Fused embedding should follow the lower-TLE modality, not cancel";
}

// =============================================================================
// 4. Modality drop-out and recovery.
//    Start both modalities active.  Modality A goes silent for ticks 50..149
//    (not published).  Trust on B should be the only entry during the gap.
//    When A returns at tick 150, trust on A should re-appear and stabilise
//    back to its TLE-weighted share.
// =============================================================================

TEST(LateralVoterStreams, DropoutAndRecovery) {
    auto p = default_params();
    p["group_balance"] = false;
    VoterFixture f(p);

    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);

    auto trust_of = [](std::shared_ptr<const ogma::ConsensusToken> c,
                       std::string const& topic) -> float {
        auto it = c->trust_weights.find(topic);
        return (it == c->trust_weights.end()) ? -1.0f : it->second;
    };

    // Phase 1: both modalities active.
    for (uint64_t t = 0; t < 50; ++t) {
        f.run_tick(t, [&]() {
            f.bus.publish("reality.video.retinal",  make_token_vec(1, 0.10f, l));
            f.bus.publish("reality.video.saliency", make_token_vec(2, 0.10f, l));
        });
    }
    auto c_both = f.last_consensus();
    ASSERT_NE(c_both, nullptr);
    EXPECT_NEAR(trust_of(c_both, "reality.video.retinal"),  0.5f, 1e-3f);
    EXPECT_NEAR(trust_of(c_both, "reality.video.saliency"), 0.5f, 1e-3f);

    // Phase 2: A drops out.  Only B publishes.
    for (uint64_t t = 50; t < 150; ++t) {
        f.run_tick(t, [&]() {
            f.bus.publish("reality.video.saliency", make_token_vec(2, 0.10f, l));
        });
    }
    auto c_solo = f.last_consensus();
    ASSERT_NE(c_solo, nullptr);
    EXPECT_NEAR(trust_of(c_solo, "reality.video.saliency"), 1.0f, 1e-3f)
        << "Sole active modality should hold all trust";
    // Retinal may or may not appear in the trust map at all; if it does its
    // weight should be 0.  Either way it should NOT carry inherited mass from
    // the previous phase.
    float t_a_solo = trust_of(c_solo, "reality.video.retinal");
    EXPECT_TRUE(t_a_solo < 0.0f || std::abs(t_a_solo) < 1e-3f)
        << "Dropped modality should not carry stale trust";

    // Phase 3: A returns alongside B.  Trust should rebalance to the steady
    // 50/50 within one tick (the voter is per-tick stateless about trust mass).
    f.run_tick(150, [&]() {
        f.bus.publish("reality.video.retinal",  make_token_vec(1, 0.10f, l));
        f.bus.publish("reality.video.saliency", make_token_vec(2, 0.10f, l));
    });
    auto c_recover = f.last_consensus();
    ASSERT_NE(c_recover, nullptr);
    EXPECT_NEAR(trust_of(c_recover, "reality.video.retinal"),  0.5f, 1e-3f)
        << "Recovered modality should immediately get its TLE-weighted share";
    EXPECT_NEAR(trust_of(c_recover, "reality.video.saliency"), 0.5f, 1e-3f);
}

// =============================================================================
// 5. Group_balance behavioural guarantee under streams.
//    With group_balance=true, the per-group mass is fixed at 1/G regardless
//    of how many EPMs live in the group.  Verify that adding a third EPM to
//    the "video" group doesn't change the proprio group's total share, and
//    that the new EPM divides video's mass proportionally to its TLE.
// =============================================================================

TEST(LateralVoterStreams, GroupBalanceIsolatesGroups) {
    VoterFixture f;   // group_balance = true (default)

    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);

    // Phase 1: 1 video + 1 proprio → 50/50 group split.
    f.run_tick(0, [&]() {
        f.bus.publish("reality.video.retinal", make_token_vec(1, 0.10f, l));
        f.bus.publish("reality.proprio.imu",   make_token_vec(2, 0.10f, l));
    });
    auto c1 = f.last_consensus();
    ASSERT_NE(c1, nullptr);
    EXPECT_NEAR(c1->trust_weights.at("reality.proprio.imu"), 0.5f, 1e-3f);

    // Phase 2: add a second video modality.  Video group mass STAYS at 0.5,
    // proprio group's 0.5 is unchanged.
    f.run_tick(1, [&]() {
        f.bus.publish("reality.video.retinal",  make_token_vec(1, 0.10f, l));
        f.bus.publish("reality.video.saliency", make_token_vec(3, 0.10f, l));
        f.bus.publish("reality.proprio.imu",    make_token_vec(2, 0.10f, l));
    });
    auto c2 = f.last_consensus();
    ASSERT_NE(c2, nullptr);
    EXPECT_NEAR(c2->trust_weights.at("reality.proprio.imu"), 0.5f, 1e-3f)
        << "Adding an EPM to a different group should not steal proprio's mass";
    float vid_total = c2->trust_weights.at("reality.video.retinal")
                    + c2->trust_weights.at("reality.video.saliency");
    EXPECT_NEAR(vid_total, 0.5f, 1e-3f);
}
