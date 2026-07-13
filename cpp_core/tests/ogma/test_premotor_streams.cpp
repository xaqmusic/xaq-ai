// =============================================================================
// test_premotor_streams.cpp
//   Behavioral tests for the Phase 6.5.25 Premotor module.
//
// Same idiom as test_lateral_voter_streams: drive synthetic ConsensusToken /
// EnvEvent streams through the module and verify the policy distribution +
// graded action emission match the design contracts.
//
// Coverage:
//   1. Bootstrap behaviour: zero action before first consensus latent.
//   2. With learned weights, the policy distribution shifts toward the
//      reward-associated intent for the trained latent.
//   3. Graded weighted_accel reflects the distribution (not just argmax).
//   4. DA modulation tightens the distribution (lower entropy under high DA).
//   5. Negative reward suppresses the originally-rewarded intent.
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/Premotor.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap default_params() {
    return {
        {"level",                 int64_t{0}},
        {"n_intents",             int64_t{5}},
        {"accel_min",            -4.0},
        {"accel_max",             4.0},
        {"gain",                  1.0},
        {"learning_rate",         0.05},
        {"temperature_base",      1.0},
        {"temperature_da_gain",   0.5},
        {"sample_action",         false},      // deterministic argmax for testing
        {"use_weighted_accel",    true},
        {"master_seed",           int64_t{42}},
    };
}

struct PremotorFixture {
    ogma::InProcessBus bus;
    ogma::Premotor     pre;

    explicit PremotorFixture(ogma::ParamMap const& p = default_params()) {
        pre.set_id("premotor");
        pre.on_setup(&bus, p);
    }

    std::shared_ptr<const ogma::PolicyToken> last_policy() const {
        return std::dynamic_pointer_cast<const ogma::PolicyToken>(
            bus.last_value(ogma::topics::kPolicyIntent));
    }
    std::shared_ptr<const ogma::ActionOut> last_action() const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(ogma::topics::kActionOut));
    }

    template <typename F>
    void run_tick(uint64_t t, F&& staged_publishes) {
        bus.begin_tick(t);
        staged_publishes();
        pre.tick(t);
        bus.end_tick();
    }

    // Helper: publish a ConsensusToken with given latent.
    void publish_consensus(uint64_t t, Eigen::VectorXf const& latent) {
        auto ct = std::make_shared<ogma::ConsensusToken>();
        ct->tick_id          = t;
        ct->level            = 0;
        ct->fused_embedding  = latent;
        ct->fused_tle        = 0.1f;
        ct->active_modality  = "test";
        ct->active_winner_id = 0;
        bus.publish("consensus.0", ct);
    }

    void publish_event(uint64_t t, std::string const& name, float intensity) {
        auto e = std::make_shared<ogma::EnvEvent>();
        e->tick_id   = t;
        e->name      = name;
        e->intensity = intensity;
        bus.publish(std::string(ogma::topics::kEventsPrefix) + name, e);
    }

    void publish_neuro(uint64_t t, float dopamine) {
        auto n = std::make_shared<ogma::NeuroState>();
        n->tick_id   = t;
        n->dopamine  = dopamine;
        n->serotonin = 0.65f;
        bus.publish(ogma::topics::kNeuroState, n);
    }
};

} // namespace

// =============================================================================
// 1. Bootstrap.  No consensus → zero action, uniform distribution.
// =============================================================================

TEST(PremotorStreams, BootstrapZeroActionUniformDistribution) {
    PremotorFixture f;
    f.run_tick(0, [&](){ /* no publishes */ });
    auto pol = f.last_policy();
    ASSERT_NE(pol, nullptr);
    EXPECT_EQ(pol->intent_distribution.size(), 5);
    // Distribution should be uniform 1/5 = 0.2.
    for (int i = 0; i < 5; ++i)
        EXPECT_NEAR(pol->intent_distribution[i], 0.2f, 1e-4f);
    auto act = f.last_action();
    ASSERT_NE(act, nullptr);
    EXPECT_NEAR(act->accel, 0.0f, 1e-4f);
    EXPECT_EQ(act->source, "premotor");
}

// =============================================================================
// 2. Hebbian learning shifts distribution toward rewarded intent.
//    Drive a fixed latent + after the first action emit a hit event.
//    Run for many ticks alternating "drive latent → tick → check distribution
//    drift" — distribution at the trained intent should grow.
// =============================================================================

TEST(PremotorStreams, RewardLearningShiftsDistribution) {
    auto p = default_params();
    p["learning_rate"] = 0.10;        // larger LR for faster convergence
    p["sample_action"] = false;       // argmax — deterministic
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[0] = 1.0f;                 // distinct latent feature

    // First tick: bootstrap — distribution will be near-uniform.
    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });
    auto p0 = f.last_policy();
    ASSERT_NE(p0, nullptr);

    // Pick whichever intent the argmax landed on initially.  Reinforce it
    // many times and check distribution at that intent grows.
    int chosen = p0->chosen_intent;
    ASSERT_GE(chosen, 0);
    float dist0 = p0->intent_distribution[chosen];

    for (uint64_t t = 1; t < 50; ++t) {
        // Reinforce the chosen intent each tick by emitting a hit event
        // BEFORE the next tick so the credit attaches to last_distribution.
        // (apply_reward is called from handle_event; the next tick uses the
        // updated weights.)
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            f.publish_event(t, "hit", 1.0f);
        });
    }
    auto pol = f.last_policy();
    ASSERT_NE(pol, nullptr);
    float dist_final = pol->intent_distribution[chosen];

    EXPECT_GT(dist_final, dist0)
        << "Repeatedly rewarded intent should gain probability mass";
    // Sanity: distribution still sums to 1.
    float sum = 0.0f;
    for (int i = 0; i < pol->intent_distribution.size(); ++i)
        sum += pol->intent_distribution[i];
    EXPECT_NEAR(sum, 1.0f, 1e-3f);
}

// =============================================================================
// 3. Weighted accel reflects the full distribution, not argmax.
//    Compare use_weighted_accel=true vs false on the same latent.
// =============================================================================

TEST(PremotorStreams, WeightedAccelDiffersFromArgmaxAccel) {
    auto p = default_params();
    p["use_weighted_accel"] = true;
    PremotorFixture f_w(p);

    p["use_weighted_accel"] = false;
    PremotorFixture f_a(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Random(8);

    f_w.run_tick(0, [&](){ f_w.publish_consensus(0, latent); });
    f_a.run_tick(0, [&](){ f_a.publish_consensus(0, latent); });

    auto pol_w = f_w.last_policy();
    auto pol_a = f_a.last_policy();
    ASSERT_NE(pol_w, nullptr);
    ASSERT_NE(pol_a, nullptr);

    // Weighted accel should be the inner product of distribution and accels.
    // With near-uniform initial distribution this is ≈ 0.0 (intent accels
    // are symmetric around zero).  Argmax accel will be one of the discrete
    // values.  They should differ in general.
    float w_accel = pol_w->weighted_accel;
    int argmax_idx = pol_a->chosen_intent;
    float a_accel = pol_a->intent_accels[argmax_idx];

    // Hand-compute weighted accel from f_w's distribution as a sanity check.
    float reconstructed = 0.0f;
    for (int i = 0; i < pol_w->intent_distribution.size(); ++i)
        reconstructed += pol_w->intent_distribution[i] * pol_w->intent_accels[i];
    EXPECT_NEAR(w_accel, reconstructed, 1e-4f);

    // The two flavours don't have to differ NUMERICALLY (rare ties), but
    // the weighted form must not equal argmax across multiple latent
    // realisations.  Run a second random latent — at least one of the
    // pairs should differ measurably.
    Eigen::VectorXf l2 = Eigen::VectorXf::Random(8);
    f_w.run_tick(1, [&](){ f_w.publish_consensus(1, l2); });
    f_a.run_tick(1, [&](){ f_a.publish_consensus(1, l2); });
    auto p2_w = f_w.last_policy();
    auto p2_a = f_a.last_policy();
    int argmax_idx2 = p2_a->chosen_intent;
    float a_accel2 = p2_a->intent_accels[argmax_idx2];

    bool differ_at_least_once =
        std::abs(w_accel  - a_accel ) > 1e-3f ||
        std::abs(p2_w->weighted_accel - a_accel2) > 1e-3f;
    EXPECT_TRUE(differ_at_least_once)
        << "Weighted accel and argmax accel should differ on at least one latent";
}

// =============================================================================
// 4. DA modulation: high DA → low temperature → lower entropy.
// =============================================================================

TEST(PremotorStreams, DopamineLowersEntropy) {
    auto p = default_params();
    p["temperature_da_gain"] = 2.0;   // strong DA effect
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Random(8);

    // Tick at low DA.
    f.run_tick(0, [&](){
        f.publish_neuro(0, /*dopamine=*/0.1f);
        f.publish_consensus(0, latent);
    });
    float ent_low = f.last_policy()->entropy;
    float t_low   = f.last_policy()->temperature;

    // Same latent but at high DA.
    f.run_tick(1, [&](){
        f.publish_neuro(1, /*dopamine=*/2.0f);
        f.publish_consensus(1, latent);
    });
    float ent_hi = f.last_policy()->entropy;
    float t_hi   = f.last_policy()->temperature;

    EXPECT_LT(t_hi,   t_low) << "High DA should lower temperature";
    EXPECT_LT(ent_hi, ent_low) << "Lower temperature should produce lower entropy";
}

// =============================================================================
// 5b. Eligibility traces (Phase 6.5.29) propagate reward backward in time.
//     Train on latent_A for K ticks (no reward), then on latent_B for 1 tick
//     and immediately reward.  With λ=0 (default Premotor) only B's intent
//     should learn.  With λ=0.95, A's intent should ALSO learn (decayed
//     credit from K ticks ago).
// =============================================================================

TEST(PremotorStreams, EligibilityTracesBackpropagateReward) {
    auto p = default_params();
    p["eligibility_lambda"] = 0.95;
    p["learning_rate"]      = 0.10;
    p["sample_action"]      = false;       // deterministic argmax for test
    PremotorFixture f(p);

    Eigen::VectorXf latent_a = Eigen::VectorXf::Zero(8);
    Eigen::VectorXf latent_b = Eigen::VectorXf::Zero(8);
    latent_a[0] = 1.0f;        // A active on axis 0
    latent_b[1] = 1.0f;        // B active on axis 1

    // Phase 1: drive latent_a for 5 ticks with no reward.  Trace builds
    // up p_chosen × latent_a contributions decayed by λ each tick.
    for (uint64_t t = 0; t < 5; ++t) {
        f.run_tick(t, [&](){ f.publish_consensus(t, latent_a); });
    }
    // Phase 2: drive latent_b for 1 tick + reward.  Trace at this point
    // has both A's accumulated history AND B's just-added contribution.
    f.run_tick(5, [&](){
        f.publish_consensus(5, latent_b);
        f.publish_event(5, "hit", 1.0f);
    });

    // Now drive latent_a alone (no reward) and check distribution shifted
    // — if eligibility worked, A's intent that was active during phase 1
    // should have grown.
    f.run_tick(6, [&](){ f.publish_consensus(6, latent_a); });
    auto pol_a_after = f.last_policy();
    ASSERT_NE(pol_a_after, nullptr);

    // Compare to the same protocol with λ=0 (no traces).
    auto p2 = default_params();
    p2["eligibility_lambda"] = 0.0;
    p2["learning_rate"]      = 0.10;
    p2["sample_action"]      = false;
    PremotorFixture f2(p2);
    for (uint64_t t = 0; t < 5; ++t) {
        f2.run_tick(t, [&](){ f2.publish_consensus(t, latent_a); });
    }
    f2.run_tick(5, [&](){
        f2.publish_consensus(5, latent_b);
        f2.publish_event(5, "hit", 1.0f);
    });
    f2.run_tick(6, [&](){ f2.publish_consensus(6, latent_a); });
    auto pol_a_after_no_trace = f2.last_policy();
    ASSERT_NE(pol_a_after_no_trace, nullptr);

    // Measure max divergence from uniform on each.  Higher = more learning
    // attached to latent A's pattern.
    auto max_dev_from_uniform = [](Eigen::VectorXf const& d) -> float {
        float best = 0.0f;
        for (int i = 0; i < d.size(); ++i) {
            float v = std::abs(d[i] - 0.2f);
            if (v > best) best = v;
        }
        return best;
    };
    float dev_traced  = max_dev_from_uniform(pol_a_after->intent_distribution);
    float dev_no_trace = max_dev_from_uniform(pol_a_after_no_trace->intent_distribution);

    EXPECT_GT(dev_traced, dev_no_trace)
        << "λ>0 should produce more learning on latent_a than λ=0 in this protocol "
        << "(traces propagate the reward backward to A's recent history)";
}

// =============================================================================
// 5. Negative reward suppresses the contributing intent.
// =============================================================================

TEST(PremotorStreams, NegativeRewardSuppresses) {
    auto p = default_params();
    p["learning_rate"] = 0.10;
    p["sample_action"] = false;
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[0] = 1.0f;

    // Phase A: reinforce.  Pick the intent that wins after one tick.
    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });
    int chosen = f.last_policy()->chosen_intent;
    ASSERT_GE(chosen, 0);

    // 30 reinforcement ticks.
    for (uint64_t t = 1; t < 30; ++t) {
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            f.publish_event(t, "hit", 1.0f);
        });
    }
    float dist_after_pos = f.last_policy()->intent_distribution[chosen];

    // Phase B: punish the same intent with miss events.
    for (uint64_t t = 30; t < 80; ++t) {
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            f.publish_event(t, "miss", 1.0f);
        });
    }
    float dist_after_neg = f.last_policy()->intent_distribution[chosen];

    EXPECT_LT(dist_after_neg, dist_after_pos)
        << "Negative reward should reduce the previously-reinforced intent's mass";
}

// =============================================================================
// Phase 6.6.F — α-gated reward update tests.
//
// Premotor's Hebbian update is now optionally gated on MotorFader α.  When
// α < update_alpha_threshold the update is skipped (off-policy contamination
// guard for the brain↔reflex crossfade).  Threshold = 0 (default) preserves
// pre-6.6.F behavior bit-for-bit.
// =============================================================================

namespace {

void publish_alpha(ogma::InProcessBus& bus, uint64_t t, float alpha) {
    auto fs = std::make_shared<ogma::FaderState>();
    fs->tick_id      = t;
    fs->alpha        = alpha;
    fs->alpha_target = alpha;
    fs->source       = "fixed";
    bus.publish(ogma::topics::kMotorFaderAlpha, fs);
}

}  // namespace

TEST(PremotorStreams, AlphaGateSuppressesUpdateBelowThreshold) {
    auto p = default_params();
    p["learning_rate"]            = 0.10;
    p["sample_action"]            = false;
    p["update_alpha_threshold"]   = 0.5;     // require α≥0.5 to learn
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[2] = 1.0f;

    // Warm up: bootstrap weights via a single non-rewarded tick.
    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });
    auto pol_before = f.last_policy();
    ASSERT_NE(pol_before, nullptr);
    Eigen::VectorXf dist_before = pol_before->intent_distribution;

    // Many reward ticks WITH α=0.1 (below threshold) → updates skipped.
    for (uint64_t t = 1; t <= 30; ++t) {
        f.run_tick(t, [&](){
            publish_alpha(f.bus, t, 0.1f);
            f.publish_consensus(t, latent);
            f.publish_event(t, "hit", 1.0f);
        });
    }
    auto pol_after = f.last_policy();
    ASSERT_NE(pol_after, nullptr);

    // Without learning, the argmax distribution shape is determined entirely
    // by the random init weights × latent; should not have drifted measurably.
    float total_drift = 0.0f;
    for (int i = 0; i < 5; ++i)
        total_drift += std::abs(pol_after->intent_distribution[i] - dist_before[i]);
    EXPECT_LT(total_drift, 1e-3f)
        << "With α=0.1 < threshold=0.5, Hebbian update should be skipped — "
           "distribution must not drift (got drift=" << total_drift << ")";
}

TEST(PremotorStreams, AlphaGateAllowsUpdateAboveThreshold) {
    auto p = default_params();
    p["learning_rate"]            = 0.10;
    p["sample_action"]            = false;
    p["update_alpha_threshold"]   = 0.3;
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[3] = 1.0f;

    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });
    auto pol_before = f.last_policy();
    ASSERT_NE(pol_before, nullptr);
    Eigen::VectorXf dist_before = pol_before->intent_distribution;

    // α=0.9 > threshold → Hebbian update fires.  We don't need to predict
    // WHICH intent grows (argmax churn over training is allowed); confirm
    // the distribution drifts measurably away from its pre-training shape.
    // Pairs with the suppress test (drift < 1e-3) for the falsifiability
    // contract on the gate.
    for (uint64_t t = 1; t <= 30; ++t) {
        f.run_tick(t, [&](){
            publish_alpha(f.bus, t, 0.9f);
            f.publish_consensus(t, latent);
            f.publish_event(t, "hit", 1.0f);
        });
    }
    auto pol_after = f.last_policy();
    ASSERT_NE(pol_after, nullptr);
    float total_drift = 0.0f;
    for (int i = 0; i < 5; ++i)
        total_drift += std::abs(pol_after->intent_distribution[i] - dist_before[i]);
    EXPECT_GT(total_drift, 1e-2f)
        << "With α=0.9 ≥ threshold=0.3, sustained reward should drift the "
           "distribution measurably (got drift=" << total_drift << ")";
}

// =============================================================================
// Phase 6.6.I — rollout-aware temperature modulation tests.
//
// When pathway_temp_gain > 0, predicted-trajectory familiarity raises the
// softmax temperature so the agent breaks out of repeated-state ruts.
// =============================================================================

namespace {

void publish_consensus_with_pathway(
        ogma::InProcessBus& bus, uint64_t t,
        Eigen::VectorXf const& latent,
        std::unordered_map<std::string, int> winners,
        std::unordered_map<std::string, std::vector<int>> pathways) {
    auto ct = std::make_shared<ogma::ConsensusToken>();
    ct->tick_id                 = t;
    ct->level                   = 0;
    ct->fused_embedding         = latent;
    ct->fused_tle               = 0.1f;
    ct->active_modality         = "test";
    ct->active_winner_id        = winners.empty() ? 0 : winners.begin()->second;
    ct->winner_ids_by_modality  = std::move(winners);
    ct->predicted_pathways      = std::move(pathways);
    bus.publish("consensus.0", ct);
}

}  // namespace

TEST(PremotorStreams, PathwayFamiliarityRaisesTemperature) {
    auto p = default_params();
    p["pathway_temp_gain"] = 4.0;     // strong modulation for the test
    p["state_visit_alpha"] = 0.5;     // EMA climbs fast
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[2] = 1.0f;

    // Warm up the state-visit EMA on a single (modality, node_id) pair so
    // it climbs toward 1.  Each tick reports winner=42 for "reality.test"
    // and predicts node 42 for next tick.
    for (uint64_t t = 0; t < 40; ++t) {
        f.run_tick(t, [&](){
            publish_consensus_with_pathway(
                f.bus, t, latent,
                {{"reality.test", 42}},
                {{"reality.test", {42}}});
        });
    }
    EXPECT_GT(f.pre.last_pathway_familiarity(), 0.9f)
        << "after 40 visits the state-visit EMA should be near saturation";

    // Capture the high-familiarity entropy.
    auto pol_high = f.last_policy();
    ASSERT_NE(pol_high, nullptr);
    float entropy_high = pol_high->entropy;
    float temp_high    = pol_high->temperature;

    // Now drop to a never-seen predicted node to drive familiarity to 0.
    // Use a fresh fixture so the comparison isn't contaminated by prior
    // weight learning on the warm-up stream.
    PremotorFixture f2(p);
    for (uint64_t t = 0; t < 40; ++t) {
        f2.run_tick(t, [&](){
            publish_consensus_with_pathway(
                f2.bus, t, latent,
                {{"reality.test", 42}},
                {{"reality.test", {999}}});   // predicts unseen id
        });
    }
    EXPECT_LT(f2.pre.last_pathway_familiarity(), 0.1f)
        << "predictions of never-seen ids should keep familiarity near 0";
    auto pol_low = f2.last_policy();
    ASSERT_NE(pol_low, nullptr);
    float temp_low = pol_low->temperature;
    float entropy_low = pol_low->entropy;

    // Familiar trajectory → higher T → higher entropy.
    EXPECT_GT(temp_high, temp_low * 1.2f)
        << "familiar pathway should raise T meaningfully (high=" << temp_high
        << " low=" << temp_low << ")";
    EXPECT_GE(entropy_high, entropy_low - 1e-3f)
        << "higher T → ≥ entropy (high=" << entropy_high
        << " low=" << entropy_low << ")";
}

TEST(PremotorStreams, PathwayGainZeroPreservesLegacyTemperature) {
    auto p = default_params();
    // pathway_temp_gain not set → defaults to 0 → bit-identical to pre-6.6.I.
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[2] = 1.0f;

    for (uint64_t t = 0; t < 40; ++t) {
        f.run_tick(t, [&](){
            publish_consensus_with_pathway(
                f.bus, t, latent,
                {{"reality.test", 42}},
                {{"reality.test", {42}}});
        });
    }
    auto pol = f.last_policy();
    ASSERT_NE(pol, nullptr);
    // No DA published → dopamine=default 0.2 in Premotor → T = 1.0 / (1 + 0.2 * 0.5) = 0.909.
    EXPECT_NEAR(pol->temperature, 1.0f / (1.0f + 0.2f * 0.5f), 1e-3f)
        << "with gain=0, temperature must equal the dopamine baseline";
    EXPECT_EQ(f.pre.last_pathway_familiarity(), 0.0f);
}

// =============================================================================
// Phase v5.1 — Monte-Carlo actor-critic.
//
// 1. mc_lr=0 is bit-identical to legacy (no trajectory, no episode counter).
// 2. With mc_lr>0, hit events accumulate into trajectory and weight update
//    is deferred until events.episode_end.
// 3. Episode-end with positive return grows the chosen intent's row;
//    negative return shrinks it (advantage-style credit).
// 4. Advantage normalisation produces near-zero net update on the FIRST
//    episode (no prior stats), then bounded magnitude updates after.
// =============================================================================

TEST(PremotorStreams, McLrZeroBitIdenticalToLegacy) {
    // mc_lr=0 → handle_event still calls apply_reward; no trajectory built.
    auto p = default_params();
    p["learning_rate"] = 0.10;
    p["sample_action"] = false;
    p["mc_lr"]         = 0.0;
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[0] = 1.0f;
    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });
    for (uint64_t t = 1; t < 20; ++t) {
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            f.publish_event(t, "hit", 1.0f);
        });
    }
    EXPECT_EQ(f.pre.mc_episodes_seen(), 0);
    EXPECT_EQ(f.pre.mc_trajectory_size(), 0);
    EXPECT_NEAR(f.pre.mc_last_return(), 0.0f, 1e-6f);
}

TEST(PremotorStreams, McTrajectoryBuffersWhenActive) {
    // mc_lr>0 → reward events accumulate into trajectory; tick appends.
    auto p = default_params();
    p["learning_rate"] = 0.0;          // legacy lr unused in MC mode
    p["mc_lr"]         = 0.05;
    p["mc_gamma"]      = 0.99;
    p["sample_action"] = false;
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[0] = 1.0f;
    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });
    // Bootstrap tick produces no trajectory entry (weights not initialised
    // until consensus arrives, BUT chosen=-1 on the bootstrap path → skip).
    int n0 = f.pre.mc_trajectory_size();

    // 5 active ticks with reward events.
    for (uint64_t t = 1; t <= 5; ++t) {
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            f.publish_event(t, "hit", 1.0f);
        });
    }
    int n5 = f.pre.mc_trajectory_size();
    EXPECT_GT(n5, n0) << "trajectory must grow when mc_lr>0";
    EXPECT_EQ(f.pre.mc_episodes_seen(), 0)
        << "no episode_end yet → no episode counted";
}

TEST(PremotorStreams, McEpisodeEndAppliesPositiveReturnUpdate) {
    auto p = default_params();
    p["learning_rate"] = 0.0;
    p["mc_lr"]         = 0.10;          // strong rate for visible drift
    p["mc_gamma"]      = 0.99;
    p["sample_action"] = false;
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[0] = 1.0f;
    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });
    auto W_before = f.pre.weights();

    // 10 ticks of reward, then episode_end.  G_0 ≈ 1 + γ + γ² + ... + γ⁹.
    for (uint64_t t = 1; t <= 10; ++t) {
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            f.publish_event(t, "hit", 1.0f);
        });
    }
    EXPECT_GT(f.pre.mc_trajectory_size(), 0);
    f.run_tick(11, [&](){
        f.publish_consensus(11, latent);
        f.publish_event(11, "episode_end", 0.0f);
    });

    EXPECT_EQ(f.pre.mc_episodes_seen(), 1);
    // The episode_end tick itself produces a new policy + appends a fresh
    // trajectory entry for episode 2 — so size = 1 (not 0) immediately
    // after the call returns.  finalize_mc_episode drained the original
    // 10-tick buffer; the new entry came from this tick's append step.
    EXPECT_EQ(f.pre.mc_trajectory_size(), 1) << "next episode's first step";
    EXPECT_GT(f.pre.mc_last_return(), 5.0f)
        << "expected G_0 ≈ Σ γ^k ≈ 9.5; got " << f.pre.mc_last_return();
    auto W_after = f.pre.weights();
    EXPECT_GT((W_after - W_before).norm(), 0.05f)
        << "MC update at episode_end must drift weights non-trivially";
}

TEST(PremotorStreams, McAdvantageNormalisationBoundsLongRunGrowth) {
    // After many episodes, advantage normalisation should keep the per-row
    // update magnitudes bounded (advantage_t ≈ (G - μ) / σ with running stats),
    // unlike raw G_t which grows ~linearly with trajectory length.
    auto p = default_params();
    p["learning_rate"]            = 0.0;
    p["mc_lr"]                    = 0.10;
    p["mc_gamma"]                 = 0.99;
    p["advantage_normalization"]  = true;
    p["advantage_window"]         = int64_t(20);
    p["sample_action"]            = false;
    PremotorFixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[0] = 1.0f;
    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });

    auto W0 = f.pre.weights();
    // Run 30 episodes, each 10 ticks with reward, then episode_end.
    uint64_t t = 0;
    for (int ep = 0; ep < 30; ++ep) {
        for (int k = 0; k < 10; ++k) {
            ++t;
            f.run_tick(t, [&](){
                f.publish_consensus(t, latent);
                f.publish_event(t, "hit", 1.0f);
            });
        }
        ++t;
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            f.publish_event(t, "episode_end", 0.0f);
        });
    }
    EXPECT_EQ(f.pre.mc_episodes_seen(), 30);
    auto Wf = f.pre.weights();
    // After ~30 episodes of identical reward, advantage ≈ 0 → bounded growth.
    // (raw return integration would diverge linearly: Δ ~ N_eps * G).
    float drift = (Wf - W0).norm();
    EXPECT_LT(drift, 50.0f)
        << "advantage normalisation should bound long-run weight growth; "
        << "got drift=" << drift;
    // Sanity: stats are populated.
    EXPECT_GT(f.pre.mc_return_std(), 0.0f);
}
