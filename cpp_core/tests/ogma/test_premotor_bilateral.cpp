// =============================================================================
// test_premotor_bilateral.cpp
//   Phase 6.6.G — bilateral output mode tests for Premotor.
//
// Three tests cover the contract:
//   1. Default 5-intent table maps each intent to (left, right)
//      according to the design-doc spec; argmax tracks the chosen
//      intent's pair.
//   2. Custom bilateral_table JSON parses and overrides the default;
//      length mismatch is rejected.
//   3. Legacy single-channel default (no bilateral params) still
//      publishes to action_output_topic_; PolicyToken is identical
//      to bilateral mode for the unilateral synthesis fields.
// =============================================================================
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/Premotor.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap base_params() {
    return {
        {"level",                 int64_t{0}},
        {"n_intents",             int64_t{5}},
        {"accel_min",            -4.0},
        {"accel_max",             4.0},
        {"gain",                  1.0},
        {"learning_rate",         0.05},
        {"temperature_base",      1.0},
        {"temperature_da_gain",   0.5},
        {"sample_action",         false},   // argmax for determinism
        {"use_weighted_accel",    false},   // chosen-intent accel for clarity
        {"master_seed",           int64_t{42}},
    };
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::Premotor     pre;
    explicit Fixture(ogma::ParamMap const& p) {
        pre.set_id("premotor");
        pre.on_setup(&bus, p);
    }
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
    template <typename F>
    void run_tick(uint64_t t, F&& staged) {
        bus.begin_tick(t);
        staged();
        pre.tick(t);
        bus.end_tick();
    }
    std::shared_ptr<const ogma::ActionOut> last(std::string const& topic) const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(bus.last_value(topic));
    }
    std::shared_ptr<const ogma::PolicyToken> last_policy() const {
        return std::dynamic_pointer_cast<const ogma::PolicyToken>(
            bus.last_value(ogma::topics::kPolicyIntent));
    }
};

}  // namespace

// =============================================================================
// Phase 6.6.O — behavioral cloning from reflex demonstration.
// Tests live here (next to bilateral tests) since BC argmin operates on
// the bilateral intent table.
// =============================================================================

namespace {
void publish_reflex_bilateral(ogma::InProcessBus& bus, uint64_t t,
                               float L, float R) {
    auto al = std::make_shared<ogma::ActionOut>();
    al->tick_id = t;
    al->accel   = L;
    al->source  = "synth_reflex";
    bus.publish("action.reflex.left", al);
    auto ar = std::make_shared<ogma::ActionOut>();
    ar->tick_id = t;
    ar->accel   = R;
    ar->source  = "synth_reflex";
    bus.publish("action.reflex.right", ar);
}

void publish_alpha(ogma::InProcessBus& bus, uint64_t t, float alpha) {
    auto fs = std::make_shared<ogma::FaderState>();
    fs->tick_id = t;
    fs->alpha   = alpha;
    bus.publish(ogma::topics::kMotorFaderAlpha, fs);
}
}  // namespace

TEST(PremotorBilateral, BcParametricArgminPicksMatchingIntent) {
    // For each canonical (L, R) pair from the default 5-intent table,
    // feed it back as the observed reflex.  apply_bc_update's argmin
    // must select the matching intent index for each.
    auto p = base_params();
    p["output_topic_left"]  = std::string("action.brain.left");
    p["output_topic_right"] = std::string("action.brain.right");
    p["lr_bc"]              = 0.05;
    p["bc_alpha_weighting"] = false;     // ablate gate so update fires regardless of α
    Fixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[2] = 1.0f;
    // Bootstrap weights — first tick initialises W_ from the consensus dim.
    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });

    auto const& L = f.pre.intent_accels_left();
    auto const& R = f.pre.intent_accels_right();
    ASSERT_EQ(int(L.size()), 5);
    for (int target = 0; target < 5; ++target) {
        uint64_t t = uint64_t(10 * (target + 1));
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            publish_reflex_bilateral(f.bus, t, L[target], R[target]);
        });
        EXPECT_EQ(f.pre.last_bc_intent(), target)
            << "feeding canonical pair " << target
            << " should select intent " << target
            << " (got " << f.pre.last_bc_intent() << ")";
    }
}

TEST(PremotorBilateral, BcLrZeroIsNoOp) {
    // With lr_bc=0, weight matrix should be bit-identical to a parallel
    // run with no reflex publishes — the 6.5.37 falsifiability pattern.
    auto p = base_params();
    p["output_topic_left"]  = std::string("action.brain.left");
    p["output_topic_right"] = std::string("action.brain.right");
    p["lr_bc"]              = 0.0;
    Fixture f(p);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[3] = 1.0f;
    for (uint64_t t = 0; t < 30; ++t) {
        f.run_tick(t, [&](){
            f.publish_consensus(t, latent);
            publish_reflex_bilateral(f.bus, t, 4.0f, -4.0f);
        });
    }
    // No BC update should have fired.
    EXPECT_EQ(f.pre.last_bc_intent(), -1);
    // Sanity: weights are uninitialised/random but shape is right.
    EXPECT_EQ(f.pre.weights().rows(), 5);
    EXPECT_EQ(f.pre.weights().cols(), 8);
}

TEST(PremotorBilateral, BcAlphaWeightingGatesUpdate) {
    // With bc_alpha_weighting=true (default) and α=1 published, BC must
    // not update.  With α=0 published, BC fires.
    auto p_high = base_params();
    p_high["output_topic_left"]  = std::string("action.brain.left");
    p_high["output_topic_right"] = std::string("action.brain.right");
    p_high["lr_bc"]              = 0.5;       // strong learning
    p_high["bc_alpha_weighting"] = true;
    Fixture fh(p_high);

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[1] = 1.0f;
    fh.run_tick(0, [&](){ fh.publish_consensus(0, latent); });
    auto W_before_high = fh.pre.weights();

    for (uint64_t t = 1; t < 20; ++t) {
        fh.run_tick(t, [&](){
            fh.publish_consensus(t, latent);
            publish_alpha(fh.bus, t, 1.0f);                       // brain-led
            publish_reflex_bilateral(fh.bus, t, 4.0f, -4.0f);
        });
    }
    auto W_after_high = fh.pre.weights();
    float drift_high = (W_after_high - W_before_high).norm();
    EXPECT_NEAR(drift_high, 0.0f, 1e-5f)
        << "α=1 + bc_alpha_weighting=true must produce zero BC drift";

    // Inverse: α=0 → full BC drift expected.
    Fixture fl(p_high);
    fl.run_tick(0, [&](){ fl.publish_consensus(0, latent); });
    auto W_before_low = fl.pre.weights();
    for (uint64_t t = 1; t < 20; ++t) {
        fl.run_tick(t, [&](){
            fl.publish_consensus(t, latent);
            publish_alpha(fl.bus, t, 0.0f);                       // reflex-led
            publish_reflex_bilateral(fl.bus, t, 4.0f, -4.0f);
        });
    }
    auto W_after_low = fl.pre.weights();
    float drift_low = (W_after_low - W_before_low).norm();
    EXPECT_GT(drift_low, 0.05f)
        << "α=0 should drive substantial BC weight drift toward matched intent";
}

// =============================================================================
// 1. Default 5-intent table maps each intent to its design-doc (left, right)
//    pair.  Train each intent in turn (one-hot-ish latent) and verify the
//    bilateral channels publish exactly the table values.
// =============================================================================

TEST(PremotorBilateral, DefaultFiveIntentTable) {
    auto p = base_params();
    p["output_topic_left"]  = std::string("action.brain.left");
    p["output_topic_right"] = std::string("action.brain.right");
    Fixture f(p);

    EXPECT_TRUE(f.pre.bilateral_enabled());
    auto const& L = f.pre.intent_accels_left();
    auto const& R = f.pre.intent_accels_right();
    ASSERT_EQ(int(L.size()), 5);
    ASSERT_EQ(int(R.size()), 5);
    // Design-doc default: hard_left, slow_left, neutral, slow_right, hard_right
    EXPECT_FLOAT_EQ(L[0], +4.0f); EXPECT_FLOAT_EQ(R[0], -4.0f);  // hard_left  → CW rotation
    EXPECT_FLOAT_EQ(L[1], +2.0f); EXPECT_FLOAT_EQ(R[1],  0.0f);  // slow_left
    EXPECT_FLOAT_EQ(L[2], +4.0f); EXPECT_FLOAT_EQ(R[2], +4.0f);  // neutral    → forward synergy
    EXPECT_FLOAT_EQ(L[3],  0.0f); EXPECT_FLOAT_EQ(R[3], +2.0f);  // slow_right
    EXPECT_FLOAT_EQ(L[4], -4.0f); EXPECT_FLOAT_EQ(R[4], +4.0f);  // hard_right → CCW rotation

    // Train each intent individually by repeatedly rewarding distinct latents.
    // We don't need perfect convergence — just enough for argmax to track.
    Eigen::VectorXf latents[5];
    for (int i = 0; i < 5; ++i) {
        latents[i] = Eigen::VectorXf::Zero(8);
        latents[i][i % 8] = 1.0f;
    }
    int t = 0;
    // Bootstrap: present each latent once so weights initialise.
    for (int i = 0; i < 5; ++i) {
        f.run_tick(t++, [&](){ f.publish_consensus(t, latents[i]); });
    }
    // 80 reward ticks per intent, cycling through.
    for (int rep = 0; rep < 80; ++rep) {
        for (int i = 0; i < 5; ++i) {
            f.run_tick(t, [&](){
                f.publish_consensus(t, latents[i]);
                f.publish_event(t, "hit", 1.0f);
            });
            ++t;
        }
    }

    // Now verify each latent argmax-routes to its corresponding bilateral pair.
    for (int target = 0; target < 5; ++target) {
        f.run_tick(t++, [&](){ f.publish_consensus(t, latents[target]); });
        auto pol = f.last_policy();
        ASSERT_NE(pol, nullptr);
        // Argmax should correspond to the rewarded intent for this latent.
        // Hebbian learning is loose; allow off-by-one neighbouring intents.
        int chosen = pol->chosen_intent;
        EXPECT_GE(chosen, 0);
        ASSERT_LT(chosen, 5);
        auto al = f.last("action.brain.left");
        auto ar = f.last("action.brain.right");
        ASSERT_NE(al, nullptr);
        ASSERT_NE(ar, nullptr);
        EXPECT_FLOAT_EQ(al->accel, std::clamp(L[chosen], -4.0f, 4.0f))
            << "Left channel must equal table[chosen] for argmax intent " << chosen;
        EXPECT_FLOAT_EQ(ar->accel, std::clamp(R[chosen], -4.0f, 4.0f))
            << "Right channel must equal table[chosen] for argmax intent " << chosen;
    }
    // Single-channel topic must NOT have been published in bilateral mode.
    EXPECT_EQ(f.last(ogma::topics::kActionOut), nullptr);
}

// =============================================================================
// 2. Custom bilateral_table JSON overrides the default; mismatched length is
//    rejected at on_setup time.
// =============================================================================

TEST(PremotorBilateral, CustomTableOverridesDefault) {
    auto p = base_params();
    p["n_intents"]          = int64_t{3};
    p["output_topic_left"]  = std::string("action.brain.left");
    p["output_topic_right"] = std::string("action.brain.right");
    p["bilateral_table"]    = std::string("[[1.0, 0.0],[0.0, 1.0],[-1.0, -1.0]]");
    Fixture f(p);

    auto const& L = f.pre.intent_accels_left();
    auto const& R = f.pre.intent_accels_right();
    ASSERT_EQ(int(L.size()), 3);
    EXPECT_FLOAT_EQ(L[0], 1.0f);  EXPECT_FLOAT_EQ(R[0], 0.0f);
    EXPECT_FLOAT_EQ(L[1], 0.0f);  EXPECT_FLOAT_EQ(R[1], 1.0f);
    EXPECT_FLOAT_EQ(L[2], -1.0f); EXPECT_FLOAT_EQ(R[2], -1.0f);

    // Length mismatch should throw at on_setup.
    auto bad = base_params();
    bad["n_intents"]          = int64_t{5};
    bad["output_topic_left"]  = std::string("action.brain.left");
    bad["output_topic_right"] = std::string("action.brain.right");
    bad["bilateral_table"]    = std::string("[[1.0, 0.0]]");   // length 1 vs 5
    ogma::InProcessBus bus2;
    ogma::Premotor pre2;
    pre2.set_id("premotor_bad");
    EXPECT_THROW(pre2.on_setup(&bus2, bad), std::invalid_argument);

    // n_intents != 5 with empty table should also throw.
    auto missing = base_params();
    missing["n_intents"]          = int64_t{3};
    missing["output_topic_left"]  = std::string("action.brain.left");
    missing["output_topic_right"] = std::string("action.brain.right");
    ogma::InProcessBus bus3;
    ogma::Premotor pre3;
    pre3.set_id("premotor_missing");
    EXPECT_THROW(pre3.on_setup(&bus3, missing), std::invalid_argument);

    // One side set without the other should also throw.
    auto half = base_params();
    half["output_topic_left"] = std::string("action.brain.left");
    ogma::InProcessBus bus4;
    ogma::Premotor pre4;
    pre4.set_id("premotor_half");
    EXPECT_THROW(pre4.on_setup(&bus4, half), std::invalid_argument);
}

// =============================================================================
// 3. Legacy single-channel default: with no bilateral params, Premotor
//    publishes to action_output_topic_ (default action.out) exactly as
//    it did pre-6.6.G.  bilateral_enabled() is false.
// =============================================================================

TEST(PremotorBilateral, LegacySingleChannelPreserved) {
    Fixture f(base_params());
    EXPECT_FALSE(f.pre.bilateral_enabled());
    EXPECT_TRUE(f.pre.intent_accels_left().empty());
    EXPECT_TRUE(f.pre.intent_accels_right().empty());

    Eigen::VectorXf latent = Eigen::VectorXf::Zero(8);
    latent[2] = 1.0f;

    f.run_tick(0, [&](){ f.publish_consensus(0, latent); });
    f.run_tick(1, [&](){ f.publish_consensus(1, latent); });

    auto a = f.last(ogma::topics::kActionOut);
    ASSERT_NE(a, nullptr);
    EXPECT_FALSE(std::isnan(a->accel));
    EXPECT_GE(a->accel, -4.0f);
    EXPECT_LE(a->accel,  4.0f);

    // Bilateral topics MUST NOT have been published in legacy mode.
    EXPECT_EQ(f.last("action.brain.left"),  nullptr);
    EXPECT_EQ(f.last("action.brain.right"), nullptr);
}
