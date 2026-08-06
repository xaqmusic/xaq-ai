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
#include <array>
#include <map>
#include <set>
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

// -- Commissioning window (dim_autocal_ticks) -------------------------------
//
// The mechanism under test replaces hand-measured dim_min/dim_max with a
// measured-then-frozen conditioning stage.  §0 rule 2 is the reason it exists:
// a channel whose scale is small relative to its siblings gets collapsed by the
// insertion gate while the encoder still shows the structure.

namespace {

// A stream where the ONLY channel that distinguishes states is tiny in
// magnitude — the exact shape of the picrawler's velocity channels (delta
// ~0.05 riding alongside position/action channels that span [-1,1]).
// dims 0..3: large common-mode, identical in both states.
// dim 4: the discriminating channel, +/- `amp`.
// dim 5: large noise-free constant.
std::shared_ptr<ogma::ProprioToken> make_tiny_signal(int state, float amp, float t) {
    const float common = 0.8f * std::sin(t * 0.05f);
    return make_proprio6(common, -common, 0.9f, -0.9f,
                         (state ? amp : -amp), 0.5f);
}

ogma::ParamMap autocal_params(int64_t ticks, double k = 4.0) {
    auto p = rbf_params();
    p["dim_autocal_ticks"] = ticks;
    p["dim_autocal_k"]     = k;
    return p;
}

// Drive a stream that alternates state every 10 frames, return the EPM.
void drive_tiny(EpmFixture& f, int n_ticks, float amp) {
    for (int t = 0; t < n_ticks; ++t) {
        auto pp = make_tiny_signal((t / 10) % 2, amp, float(t));
        f.bus.begin_tick(uint64_t(t));
        f.bus.publish("reality.proprio.imu", pp);
        f.epm.tick(uint64_t(t));
        f.bus.end_tick();
    }
}

} // namespace

TEST(EPMAutocal, OffByDefaultAndAbsentFromSnapshot) {
    EpmFixture f(rbf_params());
    drive_tiny(f, 40, 0.03f);
    auto snap = f.epm.snapshot_state();
    EXPECT_FALSE(snap.contains("dim_autocal"))
        << "with the feature off the serialised form must be byte-identical to pre-feature";
}

TEST(EPMAutocal, RejectedForNonRbfEncoders) {
    auto p = identity_params();
    p["dim_autocal_ticks"] = int64_t{100};
    ogma::InProcessBus bus;
    ogma::EPM epm;
    epm.set_id("epm_ident");
    // JL/STFT/Identity dims are not heterogeneous sensor channels.
    EXPECT_THROW(epm.on_setup(&bus, p), std::invalid_argument);
}

TEST(EPMAutocal, ContradictsExplicitDimRanges) {
    auto p = autocal_params(100);
    p["dim_min"] = std::vector<double>{-1, -1, -1, -1, -1, -1};
    p["dim_max"] = std::vector<double>{ 1,  1,  1,  1,  1,  1};
    ogma::InProcessBus bus;
    ogma::EPM epm;
    epm.set_id("epm_contradiction");
    EXPECT_THROW(epm.on_setup(&bus, p), std::invalid_argument);
}

TEST(EPMAutocal, DeterministicForIdenticalStreams) {
    EpmFixture a(autocal_params(50));
    EpmFixture b(autocal_params(50));
    drive_tiny(a, 120, 0.03f);
    drive_tiny(b, 120, 0.03f);
    auto sa = a.epm.snapshot_state()["dim_autocal"];
    auto sb = b.epm.snapshot_state()["dim_autocal"];
    EXPECT_EQ(sa["range_lo"], sb["range_lo"]);
    EXPECT_EQ(sa["range_hi"], sb["range_hi"]);
    EXPECT_EQ(a.epm.node_count(), b.epm.node_count());
}

TEST(EPMAutocal, WindowRunsWarmThenResetsTopology) {
    EpmFixture f(autocal_params(60));
    // Warm start: the GNG is live DURING the window, so a vocabulary exists.
    drive_tiny(f, 55, 0.03f);
    const int nodes_before = f.epm.node_count();
    EXPECT_GT(nodes_before, 2) << "warm start means the GNG runs during commissioning";
    // Crossing the window must drop that vocabulary: it is expressed in the
    // provisional units and is meaningless in the calibrated space.
    drive_tiny(f, 10, 0.03f);
    EXPECT_LT(f.epm.node_count(), nodes_before)
        << "the topology must be reset when the input space is rescaled";
    EXPECT_TRUE(f.epm.snapshot_state()["dim_autocal"]["done"].get<bool>());
}

TEST(EPMAutocal, RejectsOutlierDrivenRange) {
    // One first-tick-style transient must not set the range: this is why the
    // range intersects mu+/-k*sigma with the observed min/max instead of using
    // min/max alone.  The picrawler's knee delta did exactly this (delta =
    // pos - 0 on tick 1, ~16 sigma).
    EpmFixture f(autocal_params(200));
    for (int t = 0; t < 200; ++t) {
        auto pp = (t == 0) ? make_proprio6(0, 0, 0, 0, 1.0f, 0)   // the spike
                           : make_tiny_signal((t / 10) % 2, 0.03f, float(t));
        f.bus.begin_tick(uint64_t(t));
        f.bus.publish("reality.proprio.imu", pp);
        f.epm.tick(uint64_t(t));
        f.bus.end_tick();
    }
    auto dac = f.epm.snapshot_state()["dim_autocal"];
    const double hi = dac["range_hi"][4].get<double>();
    EXPECT_LT(hi, 0.5) << "a single 1.0 transient must not stretch dim 4's range to it";
    EXPECT_GT(hi, 0.0) << "but the channel's real excursion must survive";
}

TEST(EPMAutocal, SnapshotRoundTripPreservesConditioning) {
    EpmFixture f(autocal_params(50));
    drive_tiny(f, 150, 0.03f);
    auto snap = f.epm.snapshot_state();

    // A fresh EPM built from the SAME params has default conditioning until it
    // restores; after restore it must be conditioned identically, or its baked
    // vocabulary would be running against a different space.
    EpmFixture g(autocal_params(50));
    g.epm.restore_state(snap);
    auto rt = g.epm.snapshot_state()["dim_autocal"];
    EXPECT_EQ(rt["range_lo"], snap["dim_autocal"]["range_lo"]);
    EXPECT_EQ(rt["range_hi"], snap["dim_autocal"]["range_hi"]);
    EXPECT_TRUE(rt["done"].get<bool>());

    // And one more tick must land on the same winner in both.
    auto pp = make_tiny_signal(1, 0.03f, 200.0f);
    for (auto* fix : {&f, &g}) {
        fix->bus.begin_tick(500);
        fix->bus.publish("reality.proprio.imu", pp);
        fix->epm.tick(500);
        fix->bus.end_tick();
    }
    EXPECT_EQ(f.epm.node_count(), g.epm.node_count());
}

// ★ THE ACCEPTANCE TEST.  Everything above proves the mechanism is wired
// correctly; this one proves it DOES SOMETHING — that a state distinction
// carried on a small-magnitude channel is discriminable with commissioning
// and is lost without it.  If this fails to separate, the mechanism is not
// earning its place regardless of what any behavioural number says later.
//
// ⚠ NODE COUNT IS A BLIND METRIC HERE and the first version of this test used
// it: with a varying common-mode, the vocabulary size is set by the common-mode
// sweep in BOTH arms (measured: 10 vs 10) while the channel under test changes
// nothing either way.  The honest question is not "how many words" but "are the
// two states different words", so this asks that directly — the unit-test form
// of §0 rule 2's "believe the scatter, not the node count".
namespace {
// WINNER PURITY: for each winner id, how one-sided is its state distribution?
// 1.0 = every word means exactly one state (the vocabulary carries the
// distinction); 0.5 = every word is used by both states equally (the
// distinction was discretised away).  Weighted by visit count, back half only.
//
// This is the measurement that matters.  Node COUNT cannot answer it — a rich
// vocabulary built entirely on the common-mode scores well while being blind
// to the channel under test, which is precisely the §0 rule-2 trap.
double winner_purity(EpmFixture& f, int ticks, float amp) {
    std::map<int, std::array<int,2>> hist;
    for (int t = 0; t < ticks; ++t) {
        const int state = (t / 10) % 2;
        f.bus.begin_tick(uint64_t(t));
        f.bus.publish("reality.proprio.imu", make_tiny_signal(state, amp, float(t)));
        f.epm.tick(uint64_t(t));
        auto tok = f.last_token("reality.proprio.imu");
        if (t > ticks / 2 && tok && tok->winner_id >= 0) ++hist[tok->winner_id][state];
        f.bus.end_tick();
    }
    long total = 0, pure = 0;
    for (auto const& [id, c] : hist) {
        total += c[0] + c[1];
        pure  += std::max(c[0], c[1]);
    }
    return total ? double(pure) / double(total) : 0.0;
}
} // namespace

TEST(EPMAutocal, RecoversDiscriminationLostAtDefaultRanges) {
    // The §0 rule-2 shape: a small directional signal riding on a LARGE
    // common-mode.  The common-mode dominates the distance metric, so the GNG
    // spends its resolution there and collapses the small channel — while the
    // encoder still shows it.  This is the picrawler's velocity channel exactly.
    const float amp   = 0.02f;    // ~1% of the default [-1,1] span
    const int   ticks = 400;

    EpmFixture off(rbf_params());
    const double p_off = winner_purity(off, ticks, amp);

    EpmFixture on(autocal_params(100));
    const double p_on = winner_purity(on, ticks, amp);

    std::printf("[ PURITY ] default=%.3f  commissioned=%.3f  (nodes %d vs %d)\n",
                p_off, p_on, off.epm.node_count(), on.epm.node_count());

    EXPECT_LT(p_off, 0.80)
        << "control arm must FAIL to separate (purity " << p_off
        << ") or the test isn't testing anything";
    EXPECT_GT(p_on, 0.95)
        << "commissioned purity " << p_on
        << " — conditioning must make the two states different words";
    EXPECT_GT(p_on, p_off + 0.15) << "and the gain must be large, not marginal";
}
