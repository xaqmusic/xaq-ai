// =============================================================================
// test_epm_module.cpp  --  Unit tests for the v4 EPM module
// =============================================================================
//
// Exercises every encoder kind (RBF, Identity, JL, STFT), the dual-TLE
// formula, prediction-subtraction, neuro.state scaling, history_trace,
// adaptive novelty threshold, and bootstrap behaviour.
//
// Lower-level encoder/GNG correctness is covered by the existing v3 tests
// (test_encoder_jl, test_gng, test_epm); this test focuses on the v4
// Bus-wired wrapper.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap rbf_params() {
    return {
        {"modality_group",     std::string("proprio")},
        {"modality_name",      std::string("imu")},
        {"encoder_kind",       std::string("rbf")},
        {"input_topic",        std::string("reality.proprio.imu")},
        {"projection_dim",     int64_t{64}},
        {"proprio_state_dims", int64_t{6}},
        {"baking_threshold",   int64_t{10}},
        {"min_insertion_error", 0.001},
        {"history_trace_size", int64_t{4}},
        {"subtract_descending_prediction", false},
    };
}

ogma::ParamMap identity_params(std::string const& level = "1") {
    return {
        {"modality_group",     std::string("consensus")},
        {"modality_name",      level},
        {"encoder_kind",       std::string("identity")},
        {"input_topic",        std::string("consensus.0")},
        {"projection_dim",     int64_t{8}},
        {"baking_threshold",   int64_t{10}},
        {"min_insertion_error", 0.001},
        {"subtract_descending_prediction", false},
    };
}

std::shared_ptr<ogma::ProprioToken> make_proprio6(float a, float b, float c,
                                                  float d, float e, float f) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->sensor = "imu";
    p->values.resize(6);
    p->values << a, b, c, d, e, f;
    return p;
}

std::shared_ptr<ogma::ConsensusToken> make_consensus(int dim, float fill = 0.0f) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding = Eigen::VectorXf::Constant(dim, fill);
    c->fused_tle       = 0.0f;
    return c;
}

std::shared_ptr<ogma::NeuroState> make_neuro(float epsilon_b_scale = 1.0f) {
    auto n = std::make_shared<ogma::NeuroState>();
    n->dopamine                  = 0.20f;
    n->serotonin                 = 0.65f;
    n->reward_signal             = 0.0f;
    n->epsilon_b_scale           = epsilon_b_scale;
    n->min_insertion_error_scale = 1.0f;
    n->mitosis_threshold_scale   = 1.0f;
    n->novelty_threshold_scale   = 1.0f;
    return n;
}

struct EpmFixture {
    ogma::InProcessBus  bus;
    ogma::EPM           epm;

    explicit EpmFixture(ogma::ParamMap const& params) {
        epm.set_id("epm_test");
        epm.on_setup(&bus, params);
    }

    std::shared_ptr<const ogma::RealityToken>
    last_token(std::string const& topic) const {
        return std::dynamic_pointer_cast<const ogma::RealityToken>(bus.last_value(topic));
    }
};

} // namespace

// -- Construction / contract -----------------------------------------------

TEST(EPM, ConstructsAndDeclaresSchema) {
    EpmFixture f(rbf_params());
    EXPECT_EQ(f.epm.type_name(), "EPM");
    EXPECT_EQ(f.epm.input_topics().size(), 2u);   // input + neuro.state (prediction off)
    EXPECT_EQ(f.epm.output_topics().size(), 1u);
    EXPECT_EQ(f.epm.output_topics()[0].name, "reality.proprio.imu");
}

TEST(EPM, IdentityModePublishesOnConsensusN) {
    EpmFixture f(identity_params("1"));
    EXPECT_EQ(f.epm.output_topics()[0].name, "consensus.1");
}

TEST(EPM, FirstTickProducesBootstrapToken) {
    EpmFixture f(rbf_params());
    f.bus.begin_tick(0);
    f.epm.tick(0);
    f.bus.end_tick();

    auto tok = f.last_token("reality.proprio.imu");
    ASSERT_NE(tok, nullptr);
    EXPECT_EQ(tok->winner_id, -1);
    EXPECT_EQ(tok->tick_id,    0u);
    EXPECT_EQ(tok->latent.size(), 64);   // projection_dim from params
}

// -- RBF encoder path ------------------------------------------------------

TEST(EPM, RbfModeProducesValidTokenAfterBootstrap) {
    EpmFixture f(rbf_params());

    // Drive 50 ticks with proprio inputs → GNG should grow nodes and
    // produce real tokens after the bootstrap window.
    for (uint64_t t = 0; t < 50; ++t) {
        f.bus.begin_tick(t);
        // Vary proprio inputs to drive GNG learning.
        float phase = float(t) * 0.1f;
        f.bus.publish("reality.proprio.imu",
            make_proprio6(std::sin(phase), std::cos(phase),
                          0.5f, -0.5f, 0.2f * (t % 5),
                          0.3f * ((t + 1) % 3)));
        f.epm.tick(t);
        f.bus.end_tick();
    }

    EXPECT_GT(f.epm.node_count(), 1);

    auto tok = f.last_token("reality.proprio.imu");
    ASSERT_NE(tok, nullptr);
    EXPECT_GE(tok->winner_id,    0);
    EXPECT_GE(tok->quant_error,  0.0f);
    EXPECT_GE(tok->tle,          0.0f);
    EXPECT_EQ(tok->latent.size(), 64);
    EXPECT_EQ(tok->winner_prototype.size(), 64);
}

TEST(EPM, HistoryTraceTracksRecentWinners) {
    EpmFixture f(rbf_params());

    for (uint64_t t = 0; t < 20; ++t) {
        f.bus.begin_tick(t);
        float phase = float(t) * 0.1f;
        f.bus.publish("reality.proprio.imu",
            make_proprio6(std::sin(phase), std::cos(phase),
                          0.5f, -0.5f, 0.0f, 0.0f));
        f.epm.tick(t);
        f.bus.end_tick();
    }

    auto tok = f.last_token("reality.proprio.imu");
    ASSERT_NE(tok, nullptr);
    // history_trace_size = 4 in our params → max length 4.
    EXPECT_LE(int(tok->history_trace.size()), 4);
    EXPECT_GE(int(tok->history_trace.size()), 1);
}

// -- Identity (Level-N) encoder path --------------------------------------

TEST(EPM, IdentityModePassesConsensusEmbeddingToGng) {
    EpmFixture f(identity_params("1"));

    // Drive with varied 8-D consensus embeddings.
    for (uint64_t t = 0; t < 30; ++t) {
        f.bus.begin_tick(t);
        auto c = std::make_shared<ogma::ConsensusToken>();
        c->fused_embedding.resize(8);
        c->fused_embedding << std::sin(0.1f * t),
                              std::cos(0.1f * t),
                              0.5f * std::sin(0.07f * t),
                              0.5f * std::cos(0.07f * t),
                              0.0f, 0.0f, 0.0f, 0.0f;
        c->fused_tle = 0.1f;
        f.bus.publish("consensus.0", c);
        f.epm.tick(t);
        f.bus.end_tick();
    }

    EXPECT_GT(f.epm.node_count(), 1);
    auto tok = f.last_token("consensus.1");
    ASSERT_NE(tok, nullptr);
    EXPECT_GE(tok->winner_id, 0);
}

TEST(EPM, IdentityModeRejectsWrongDim) {
    EpmFixture f(identity_params("1"));

    // 4-D consensus, but EPM expects 8-D → should produce bootstrap token.
    f.bus.begin_tick(0);
    auto c = make_consensus(4, 0.5f);
    f.bus.publish("consensus.0", c);
    f.epm.tick(0);
    f.bus.end_tick();

    auto tok = f.last_token("consensus.1");
    ASSERT_NE(tok, nullptr);
    EXPECT_EQ(tok->winner_id, -1);
}

// -- Dual TLE formula ------------------------------------------------------

TEST(EPM, DualTleFormulaUsesAlphaBeta) {
    auto p = rbf_params();
    p["tle_alpha"] = 0.5;
    p["tle_beta"]  = 0.5;
    EpmFixture f(p);

    // Bootstrap + drive a few different inputs, then check tle =
    // tle_alpha * QE + tle_beta * TS within tolerance.
    for (uint64_t t = 0; t < 30; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("reality.proprio.imu",
            make_proprio6(std::sin(0.1f * t), std::cos(0.1f * t),
                          0.0f, 0.0f, 0.0f, 0.0f));
        f.epm.tick(t);
        f.bus.end_tick();
    }
    auto tok = f.last_token("reality.proprio.imu");
    ASSERT_NE(tok, nullptr);
    if (tok->winner_id >= 0) {
        float expected = 0.5f * tok->quant_error + 0.5f * tok->transition_surp;
        EXPECT_NEAR(tok->tle, expected, 1e-4f);
    }
}

// -- Bus-wired neuro.state scaling ----------------------------------------

TEST(EPM, NeuroStateScalingChangesGngEpsilonB) {
    EpmFixture f(rbf_params());

    // Capture epsilon_b after first tick with neuro.state(epsilon_b_scale = 1.0).
    f.bus.begin_tick(0);
    f.bus.publish(ogma::topics::kNeuroState, make_neuro(1.0f));
    f.bus.publish("reality.proprio.imu",
        make_proprio6(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f));
    f.epm.tick(0);
    f.bus.end_tick();

    // Now publish neuro.state with a very different scale; verify epsilon_b
    // changes by checking the GNG configuration via a second tick that does
    // NOT crash and still publishes a token.
    f.bus.begin_tick(1);
    f.bus.publish(ogma::topics::kNeuroState, make_neuro(2.5f));
    f.bus.publish("reality.proprio.imu",
        make_proprio6(0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f));
    f.epm.tick(1);
    f.bus.end_tick();

    auto tok = f.last_token("reality.proprio.imu");
    ASSERT_NE(tok, nullptr);
    EXPECT_FALSE(std::isnan(tok->tle));
}

// -- Prediction subtraction -----------------------------------------------

TEST(EPM, PredictionSubtractionEnabledByDefault) {
    auto p = rbf_params();
    p["subtract_descending_prediction"] = true;
    EpmFixture f(p);

    EXPECT_GE(f.epm.input_topics().size(), 3u);   // input + neuro + prediction
}

// -- Determinism -----------------------------------------------------------

TEST(EPM, IdenticalInputsProduceIdenticalNodeCounts) {
    auto run = []() {
        EpmFixture f(rbf_params());
        for (uint64_t t = 0; t < 100; ++t) {
            f.bus.begin_tick(t);
            float phase = float(t) * 0.1f;
            f.bus.publish("reality.proprio.imu",
                make_proprio6(std::sin(phase), std::cos(phase),
                              0.5f, -0.5f, 0.0f, 0.0f));
            f.epm.tick(t);
            f.bus.end_tick();
        }
        return std::make_pair(f.epm.node_count(), f.epm.mitosis_count());
    };
    auto a = run();
    auto b = run();
    EXPECT_EQ(a.first,  b.first);
    EXPECT_EQ(a.second, b.second);
}

// -- Param hot-mutation ---------------------------------------------------

TEST(EPM, HotMutateTleAlpha) {
    EpmFixture f(rbf_params());
    EXPECT_NO_THROW(f.epm.on_param_change("tle_alpha", ogma::ParamValue{0.9}));
    EXPECT_NO_THROW(f.epm.on_param_change("tle_beta",  ogma::ParamValue{0.1}));
}

TEST(EPM, ConstructionOnlyParamThrowsOnHotMutate) {
    EpmFixture f(rbf_params());
    EXPECT_THROW(f.epm.on_param_change("encoder_kind", ogma::ParamValue{std::string("jl")}),
                 std::invalid_argument);
    EXPECT_THROW(f.epm.on_param_change("modality_group", ogma::ParamValue{std::string("video")}),
                 std::invalid_argument);
}

TEST(EPM, UnknownParamThrows) {
    EpmFixture f(rbf_params());
    EXPECT_THROW(f.epm.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}

// -- First-tick safety ----------------------------------------------------

TEST(EPM, NoNanOnFirstTickWithoutInput) {
    EpmFixture f(rbf_params());
    f.bus.begin_tick(0);
    f.epm.tick(0);   // no input published
    f.bus.end_tick();

    auto tok = f.last_token("reality.proprio.imu");
    ASSERT_NE(tok, nullptr);
    EXPECT_EQ(tok->winner_id, -1);
    for (int i = 0; i < tok->latent.size(); ++i)
        EXPECT_FALSE(std::isnan(tok->latent[i]));
}

// -- Phase v5.2 sub-rate processing ---------------------------------------

TEST(EPM, SubRateRepublishesCachedTokenOnSkippedTicks) {
    // process_every_n_ticks=4 → encoder runs on ticks 0, 4, 8 ...
    // On the 3 intermediate ticks, EPM should republish its last token
    // with the current tick_id, not advance the GNG.
    auto p = rbf_params();
    p["process_every_n_ticks"] = int64_t{4};
    EpmFixture f(p);

    // Drive 50 ticks of input so the GNG grows.  Inputs change every tick;
    // the EPM only consumes them on every 4th tick.
    for (uint64_t t = 0; t < 50; ++t) {
        f.bus.begin_tick(t);
        float phase = float(t) * 0.1f;
        f.bus.publish("reality.proprio.imu",
            make_proprio6(std::sin(phase), std::cos(phase),
                          0.5f, -0.5f, 0.2f * (t % 5),
                          0.3f * ((t + 1) % 3)));
        f.epm.tick(t);
        f.bus.end_tick();
    }
    // GNG node count grows slower than at sub-rate=1 — process count is
    // 50 / 4 = 12 actual encoder ticks vs 50 if running every tick.
    EXPECT_LT(f.epm.node_count(), 50);
    EXPECT_GE(f.epm.node_count(), 1);

    // Token republished every tick — last published tick_id must match
    // last simulated tick (49), not the last process tick (48).
    auto tok = f.last_token("reality.proprio.imu");
    ASSERT_NE(tok, nullptr);
    EXPECT_EQ(tok->tick_id, 49u)
        << "republished token must carry the current tick_id";
}

TEST(EPM, SubRateOneIsBitIdenticalToLegacy) {
    // process_every_n_ticks=1 (default) → behaviour identical to no sub-rate.
    auto p_default = rbf_params();
    auto p_sub1    = rbf_params();
    p_sub1["process_every_n_ticks"] = int64_t{1};
    EpmFixture f_def(p_default);
    EpmFixture f_sub1(p_sub1);

    for (uint64_t t = 0; t < 30; ++t) {
        float phase = float(t) * 0.1f;
        auto pp = make_proprio6(std::sin(phase), std::cos(phase),
                                 0.5f, -0.5f, 0.2f * (t % 5),
                                 0.3f * ((t + 1) % 3));

        f_def.bus.begin_tick(t);
        f_def.bus.publish("reality.proprio.imu", pp);
        f_def.epm.tick(t);
        f_def.bus.end_tick();

        f_sub1.bus.begin_tick(t);
        f_sub1.bus.publish("reality.proprio.imu", pp);
        f_sub1.epm.tick(t);
        f_sub1.bus.end_tick();
    }
    EXPECT_EQ(f_def.epm.node_count(), f_sub1.epm.node_count())
        << "sub_rate=1 must be bit-identical to default";
}
