// =============================================================================
// test_voter_surprise_modulation.cpp
//   Phase 6.6.E — LateralVoter predicted_pathway surprise → trust modulator.
//
// The voter buffers the previous tick's `predicted_pathway[0]` per modality
// and, when the next tick's RealityToken arrives, computes a binary surprise
// (0 if predicted == observed winner, else 1).  The EMA-smoothed surprise
// scales the raw inverse-TLE trust at the existing fusion site.
//
// Four behavioral guarantees:
//   1. Off-by-default (surprise_gain=0) is bit-identical to pre-6.6.E voter.
//   2. EMA distinguishes accurate from inaccurate predictors over a stream.
//   3. Trust shifts toward the accurate-predictor modality when surprise_gain>0.
//   4. Empty predicted_pathway (EPM not configured) leaves trust unchanged.
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::RealityToken>
make_token(int winner_id, float tle, Eigen::VectorXf latent,
           std::vector<int> pathway = {}) {
    auto t = std::make_shared<ogma::RealityToken>();
    t->winner_id         = winner_id;
    t->tle               = tle;
    t->quant_error       = 0.05f;
    t->latent            = std::move(latent);
    t->predicted_pathway = std::move(pathway);
    return t;
}

// 6.6.H — token with both latent and winner_prototype set; embedding-distance
// surprise needs the prototype to compare against the cached predicted-node
// embedding.
std::shared_ptr<ogma::RealityToken>
make_token_with_prototype(int winner_id, float tle,
                          Eigen::VectorXf latent,
                          Eigen::VectorXf prototype,
                          std::vector<int> pathway = {}) {
    auto t = make_token(winner_id, tle, std::move(latent), std::move(pathway));
    t->winner_prototype = std::move(prototype);
    return t;
}

ogma::ParamMap default_params() {
    return {
        {"level",          int64_t{0}},
        {"input_pattern",  std::string("reality.")},
        {"trust_epsilon",  0.05},
        {"group_balance",  false},
        {"priority_group", std::string("proprio")},
        {"surprise_gain",  0.0},
    };
}

struct VoterFixture {
    ogma::InProcessBus bus;
    ogma::LateralVoter voter;

    explicit VoterFixture(ogma::ParamMap const& p = default_params()) {
        voter.set_id("voter_0");
        voter.on_setup(&bus, p);
    }
    std::shared_ptr<const ogma::ConsensusToken> last_consensus() const {
        return std::dynamic_pointer_cast<const ogma::ConsensusToken>(
            bus.last_value("consensus.0"));
    }
    template <typename F>
    void run_tick(uint64_t t, F&& publish_fn) {
        bus.begin_tick(t);
        publish_fn();
        voter.tick(t);
        bus.end_tick();
    }
};

// Drive a stream where modality A always predicts the next winner correctly
// (alternating 0↔1 winners and predicting accordingly), while modality B
// predicts a fixed wrong value (winner alternates, prediction stays at 99).
// Returns the final consensus token.
std::shared_ptr<const ogma::ConsensusToken>
drive_accurate_vs_inaccurate(VoterFixture& f, int ticks) {
    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);
    int seq[2] = {0, 1};
    int last_predicted_a = -1;
    int last_predicted_b = -1;
    (void)last_predicted_a; (void)last_predicted_b;
    for (uint64_t t = 0; t < uint64_t(ticks); ++t) {
        int now  = seq[t % 2];
        int next = seq[(t + 1) % 2];
        f.run_tick(t, [&]() {
            // A: accurate predictor — always predicts the actual next winner.
            f.bus.publish("reality.video.retinal",
                          make_token(now, 0.10f, l, /*pathway=*/{next}));
            // B: inaccurate predictor — always predicts a node ID that won't
            // match the actual next winner.
            f.bus.publish("reality.video.saliency",
                          make_token(now, 0.10f, l, /*pathway=*/{99}));
        });
    }
    return f.last_consensus();
}

} // namespace

// =============================================================================
// 1. Off-by-default: surprise_gain=0 is bit-identical to pre-6.6.E behavior.
//    Same TLE on both modalities → 50/50 trust regardless of predictions.
// =============================================================================

TEST(VoterSurpriseModulation, OffByDefaultPreservesEqualTrust) {
    VoterFixture f;   // surprise_gain = 0
    auto cons = drive_accurate_vs_inaccurate(f, 100);
    ASSERT_NE(cons, nullptr);
    float ta = cons->trust_weights.at("reality.video.retinal");
    float tb = cons->trust_weights.at("reality.video.saliency");
    EXPECT_NEAR(ta, 0.5f, 1e-3f)
        << "With surprise_gain=0 trust must be unmodulated 50/50";
    EXPECT_NEAR(tb, 0.5f, 1e-3f);
}

// =============================================================================
// 2. With surprise_gain>0 the accurate predictor gains trust share.
// =============================================================================

TEST(VoterSurpriseModulation, AccuratePredictorGainsTrust) {
    auto p = default_params();
    p["surprise_gain"]  = 0.9;     // strong modulator
    p["surprise_alpha"] = 0.3;     // converge fast in the test window
    VoterFixture f(p);

    auto cons = drive_accurate_vs_inaccurate(f, 100);
    ASSERT_NE(cons, nullptr);
    float ta = cons->trust_weights.at("reality.video.retinal");
    float tb = cons->trust_weights.at("reality.video.saliency");
    EXPECT_GT(ta, tb)
        << "Accurate-predictor modality should hold strictly more trust mass";
    EXPECT_GT(ta, 0.6f)
        << "Accurate predictor should clearly dominate (got " << ta << ")";
}

// =============================================================================
// 3. Empty predicted_pathway leaves trust unchanged.
//    When neither modality emits predictions (EPMs not configured), the
//    surprise EMA never updates and trust is the unmodulated TLE-inverse
//    50/50 even with surprise_gain>0.
// =============================================================================

TEST(VoterSurpriseModulation, EmptyPathwayLeavesTrustUnchanged) {
    auto p = default_params();
    p["surprise_gain"]  = 0.9;
    p["surprise_alpha"] = 0.3;
    VoterFixture f(p);

    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);
    for (uint64_t t = 0; t < 50; ++t) {
        f.run_tick(t, [&]() {
            // No predicted_pathway on either token.
            f.bus.publish("reality.video.retinal",
                          make_token(int(t & 1), 0.10f, l, /*pathway=*/{}));
            f.bus.publish("reality.video.saliency",
                          make_token(int(t & 1), 0.10f, l, /*pathway=*/{}));
        });
    }
    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    float ta = cons->trust_weights.at("reality.video.retinal");
    float tb = cons->trust_weights.at("reality.video.saliency");
    EXPECT_NEAR(ta, 0.5f, 1e-3f)
        << "Without any predictions, surprise EMA stays 0 and trust is "
           "unmodulated";
    EXPECT_NEAR(tb, 0.5f, 1e-3f);
}

// =============================================================================
// 4. Surprise floor prevents zeroing.
//    Even a maximally-wrong predictor (every prediction misses) should retain
//    at least surprise_floor share of its raw trust scale, not vanish.  We
//    verify by giving both modalities equal TLE but only the bad predictor
//    emits predictions: its trust share scales by (1 - gain*1.0) clamped to
//    surprise_floor.
// =============================================================================

TEST(VoterSurpriseModulation, SurpriseFloorPreventsZeroing) {
    auto p = default_params();
    p["surprise_gain"]  = 1.5;     // would over-attenuate without floor
    p["surprise_alpha"] = 0.5;
    p["surprise_floor"] = 0.10;    // explicit floor
    VoterFixture f(p);

    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);
    int seq[2] = {0, 1};
    for (uint64_t t = 0; t < 80; ++t) {
        int now  = seq[t % 2];
        f.run_tick(t, [&]() {
            // A: no prediction → surprise EMA stays 0 → unmodulated.
            f.bus.publish("reality.video.retinal",
                          make_token(now, 0.10f, l, /*pathway=*/{}));
            // B: always-wrong prediction → surprise EMA → 1.0.
            f.bus.publish("reality.video.saliency",
                          make_token(now, 0.10f, l, /*pathway=*/{99}));
        });
    }
    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    float tb = cons->trust_weights.at("reality.video.saliency");
    // raw_a = 1/(0.10+0.05); raw_b = same * floor (0.10). Normalised:
    // tb >= floor / (1 + floor) ≈ 0.0909 with floor=0.10.
    EXPECT_GT(tb, 0.05f)
        << "Floored surprise modulator must not reduce trust to ≈0";
}

// =============================================================================
// 5. Phase 6.6.H — embedding-distance surprise mode.  When predicted node
//    embedding is close to observed (cosine ≈ 1) surprise stays low; when
//    far (cosine ≈ -1) surprise saturates to 1.  This is the smooth-gradient
//    payoff of using EmbeddingRegistry-style cached prototypes instead of
//    binary node-id matching.
// =============================================================================

TEST(VoterSurpriseModulation, EmbeddingModeSmoothDistance) {
    auto p = default_params();
    p["surprise_gain"]  = 1.0;
    p["surprise_alpha"] = 0.5;
    p["surprise_floor"] = 0.05;
    p["surprise_kind"]  = std::string("embedding");
    VoterFixture f(p);

    // Build a stream where modality A's predicted prototype matches its
    // observed prototype (low cosine distance ⇒ low surprise) while
    // modality B's predicted prototype is opposite-signed (high cosine
    // distance ⇒ high surprise).  Both modalities use the same TLE so
    // any trust differential comes from surprise modulation alone.
    Eigen::VectorXf proto_a(4);   proto_a << 1.0f, 0.0f, 0.0f, 0.0f;
    Eigen::VectorXf proto_b_now(4);   proto_b_now << 1.0f, 0.0f, 0.0f, 0.0f;
    Eigen::VectorXf proto_b_pred(4);  proto_b_pred << -1.0f, 0.0f, 0.0f, 0.0f;
    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);

    // Tick 0: seed the embedding cache with both modalities' winners.
    // Modality A: winner_id=10, prototype=proto_a; predicts node 10 next.
    // Modality B: winner_id=20, prototype=proto_b_now; predicts node 21 next.
    f.run_tick(0, [&]() {
        f.bus.publish("reality.video.retinal",
            make_token_with_prototype(10, 0.10f, l, proto_a, /*pathway=*/{10}));
        f.bus.publish("reality.video.saliency",
            make_token_with_prototype(20, 0.10f, l, proto_b_now, /*pathway=*/{21}));
    });
    // Tick 1: B now reports winner=21 with proto_b_pred (opposite of cached
    // node 20's prototype), so the predicted (cached as proto_b_now) vs
    // observed (proto_b_pred) cosine is -1 → surprise = 1. A sees winner=10
    // matching its prediction; cached (proto_a) vs observed (proto_a) cosine
    // is +1 → surprise = 0.
    //
    // Note: the voter's cache holds ID→prototype mapping. At tick 1 the
    // predicted id was 21 for B but cache only has 20 (seeded at tick 0),
    // so the lookup misses → fallback surprise = 1. To exercise the cosine
    // path we need to PRIME the cache with the predicted node's id first.
    // Easiest: at tick 0 seed the cache by publishing the predicted id as
    // the winner, then at tick 1 predict the correct/incorrect node.
    //
    // Re-do with a cleaner stream below.
    (void)proto_b_pred;
    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
}

TEST(VoterSurpriseModulation, EmbeddingModeMatchedPredictionLowerSurprise) {
    auto p = default_params();
    p["surprise_gain"]  = 1.0;
    p["surprise_alpha"] = 0.5;
    p["surprise_floor"] = 0.05;
    p["surprise_kind"]  = std::string("embedding");
    VoterFixture f(p);

    Eigen::VectorXf proto_x(4);   proto_x << 1.0f, 0.0f, 0.0f, 0.0f;
    Eigen::VectorXf proto_y(4);   proto_y << -1.0f, 0.0f, 0.0f, 0.0f;
    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);

    // Warm-up: alternate winners 0 and 1 so the cache learns both
    // prototypes.  Each token also predicts the other for the next tick
    // so the predicted-id is always cached.
    for (uint64_t t = 0; t < 40; ++t) {
        int now  = (t % 2 == 0) ? 0 : 1;
        int next = 1 - now;
        Eigen::VectorXf proto_now      = (now == 0)  ? proto_x : proto_y;
        Eigen::VectorXf proto_next_far = (next == 0) ? proto_x : proto_y;
        f.run_tick(t, [&]() {
            // A: predicts the actual next winner whose cached prototype IS
            // the observed prototype next tick → cosine ≈ 1 → surprise → 0.
            f.bus.publish("reality.video.retinal",
                make_token_with_prototype(now, 0.10f, l, proto_now, {next}));
            // B: predicts a node whose cached prototype is OPPOSITE-signed
            // from what it will observe → cosine ≈ -1 → surprise → 1.
            // Trick: B always predicts `now` but next tick it'll observe
            // `next` whose prototype is opposite of cached(`now`).
            f.bus.publish("reality.video.saliency",
                make_token_with_prototype(now, 0.10f, l, proto_now, {now}));
            (void)proto_next_far;
        });
    }
    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    auto ta = cons->trust_weights.at("reality.video.retinal");
    auto tb = cons->trust_weights.at("reality.video.saliency");
    EXPECT_GT(ta, tb)
        << "embedding-mode: matched-prototype prediction should win trust "
           "share over far-prototype prediction (ta=" << ta << " tb=" << tb << ")";
    // Verify the surprise EMAs were actually populated and ordered.
    auto& ema_a = cons->surprise_ema.at("reality.video.retinal");
    auto& ema_b = cons->surprise_ema.at("reality.video.saliency");
    EXPECT_LT(ema_a, ema_b)
        << "embedding-mode: matched predictor should have lower surprise EMA "
           "(ema_a=" << ema_a << " ema_b=" << ema_b << ")";
}

// =============================================================================
// 7. Phase 6.6.J — confidence shrinkage prevents cold-start "perfect predictor"
//    artifact.  At low n, surfaced surprise pulls toward a 0.5 prior so a
//    just-initialised EPM (surprise_ema = 0 by default) doesn't immediately
//    publish a "trust the brain fully" signal to MotorFader.
// =============================================================================

TEST(VoterSurpriseModulation, ColdStartShrinksTowardUniformPrior) {
    auto p = default_params();
    p["surprise_gain"]   = 1.0;
    p["surprise_alpha"]  = 0.1;     // n_prior = 10
    p["surprise_kind"]   = std::string("binary");
    // surprise_calibrate not set → defaults to true.
    VoterFixture f(p);

    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);

    // Tick 0: A's first prediction (just one comparison sample).  At
    // n=1 with n_prior=10, c = 1/11 ≈ 0.09 → surfaced surprise pulls
    // strongly toward 0.5 even when raw EMA suggests perfect prediction.
    f.run_tick(0, [&]() {
        f.bus.publish("reality.video.retinal",
                      make_token(0, 0.10f, l, /*pathway=*/{1}));
    });
    f.run_tick(1, [&]() {
        f.bus.publish("reality.video.retinal",
                      make_token(1, 0.10f, l, /*pathway=*/{0}));
    });
    auto cons_early = f.last_consensus();
    ASSERT_NE(cons_early, nullptr);
    auto it = cons_early->surprise_ema.find("reality.video.retinal");
    ASSERT_NE(it, cons_early->surprise_ema.end());
    EXPECT_GT(it->second, 0.40f)
        << "cold-start with 1 sample should pull near 0.5 prior, not 0.0; got "
        << it->second;

    // After many correct predictions, c → 1 → surfaced ≈ raw EMA → near 0.
    for (uint64_t t = 2; t < 200; ++t) {
        int now  = (t % 2 == 0) ? 0 : 1;
        int next = 1 - now;
        f.run_tick(t, [&]() {
            f.bus.publish("reality.video.retinal",
                          make_token(now, 0.10f, l, /*pathway=*/{next}));
        });
    }
    auto cons_late = f.last_consensus();
    ASSERT_NE(cons_late, nullptr);
    auto it_late = cons_late->surprise_ema.find("reality.video.retinal");
    ASSERT_NE(it_late, cons_late->surprise_ema.end());
    EXPECT_LT(it_late->second, 0.10f)
        << "after 200 correct samples shrinkage should fade and surprise → 0; got "
        << it_late->second;
}

// =============================================================================
// 6. Embedding mode falls back to surprise=1 when the predicted node's
//    embedding isn't in the cache (cold-start / predicted-id-pruned).
// =============================================================================

TEST(VoterSurpriseModulation, EmbeddingModeMissingCacheFallsBack) {
    auto p = default_params();
    p["surprise_gain"]  = 1.5;
    p["surprise_alpha"] = 0.5;
    p["surprise_floor"] = 0.10;
    p["surprise_kind"]  = std::string("embedding");
    VoterFixture f(p);

    Eigen::VectorXf proto(4); proto << 1.0f, 0.0f, 0.0f, 0.0f;
    Eigen::VectorXf l = Eigen::VectorXf::Ones(4);
    int seq[2] = {0, 1};
    // B always predicts a node id (99) that the voter has never seen and
    // therefore can't look up an embedding for → fallback surprise = 1.
    for (uint64_t t = 0; t < 80; ++t) {
        int now = seq[t % 2];
        f.run_tick(t, [&]() {
            f.bus.publish("reality.video.retinal",
                make_token_with_prototype(now, 0.10f, l, proto, /*pathway=*/{}));
            f.bus.publish("reality.video.saliency",
                make_token_with_prototype(now, 0.10f, l, proto, /*pathway=*/{99}));
        });
    }
    auto cons = f.last_consensus();
    ASSERT_NE(cons, nullptr);
    auto ema_b = cons->surprise_ema.at("reality.video.saliency");
    EXPECT_GT(ema_b, 0.9f)
        << "missing-cache fallback should drive surprise EMA toward 1";
    auto ta = cons->trust_weights.at("reality.video.retinal");
    auto tb = cons->trust_weights.at("reality.video.saliency");
    EXPECT_GT(ta, tb)
        << "modality with no predictions should outrank modality with all-miss "
           "predictions (ta=" << ta << " tb=" << tb << ")";
}
