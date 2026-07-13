// =============================================================================
// test_descending_predictor.cpp  --  Unit tests for DescendingPredictor
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/DescendingPredictor.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap default_params() {
    return {
        {"consensus_topic", std::string("consensus.0")},
        {"targets",         std::vector<std::string>{"reality.video.retinal"}},
        {"learning_rate",   0.05},
        {"init_noise_scale", 0.0},      // deterministic init for repeatable tests
    };
}

std::shared_ptr<ogma::ConsensusToken> make_consensus(Eigen::VectorXf v) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding = std::move(v);
    c->fused_tle       = 0.1f;
    c->level           = 0;
    return c;
}

std::shared_ptr<ogma::RealityToken> make_reality(Eigen::VectorXf latent) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->latent    = std::move(latent);
    r->winner_id = 1;
    return r;
}

struct PredFixture {
    ogma::InProcessBus           bus;
    ogma::DescendingPredictor    pred;

    explicit PredFixture(ogma::ParamMap params = default_params()) {
        pred.set_id("predictor_0");
        pred.on_setup(&bus, params);
    }

    std::shared_ptr<const ogma::PredictionToken>
    last_prediction(std::string const& target_label = "video.retinal") const {
        return std::dynamic_pointer_cast<const ogma::PredictionToken>(
            bus.last_value(std::string("prediction.") + target_label));
    }
};

} // namespace

// -- Construction / contract ----------------------------------------------

TEST(DescendingPredictor, ConstructsAndDeclaresContract) {
    PredFixture f;
    EXPECT_EQ(f.pred.type_name(), "DescendingPredictor");
    EXPECT_EQ(f.pred.target_count(), 1);
    EXPECT_EQ(f.pred.output_topics().size(), 1u);
    EXPECT_EQ(f.pred.output_topics()[0].name, "prediction.video.retinal");
}

TEST(DescendingPredictor, RequiresAtLeastOneTarget) {
    auto p = default_params();
    p["targets"] = std::vector<std::string>{};
    EXPECT_THROW({ PredFixture f(p); }, std::invalid_argument);
}

// -- Forward pass --------------------------------------------------------

TEST(DescendingPredictor, FirstTickPublishesZeroBeforeAnyConsensus) {
    PredFixture f;
    f.bus.begin_tick(0);
    f.pred.tick(0);
    f.bus.end_tick();

    auto pt = f.last_prediction();
    ASSERT_NE(pt, nullptr);
    // No consensus seen yet → zero placeholder.
    EXPECT_EQ(pt->predicted_latent.size(), 1);
    EXPECT_FLOAT_EQ(pt->predicted_latent[0], 0.0f);
}

TEST(DescendingPredictor, ForwardPassProducesPredictionAfterConsensus) {
    PredFixture f;

    f.bus.begin_tick(0);
    Eigen::VectorXf c = Eigen::VectorXf::Ones(8);
    f.bus.publish("consensus.0", make_consensus(c));
    // Prime W via a reality(t-1) Feedback path: publish a synthetic reality
    // payload, then begin tick 1 so Feedback delivers it.
    f.bus.publish("reality.video.retinal", make_reality(Eigen::VectorXf::Constant(8, 0.5f)));
    f.pred.tick(0);
    f.bus.end_tick();

    f.bus.begin_tick(1);
    f.bus.publish("consensus.0", make_consensus(c));
    f.pred.tick(1);
    f.bus.end_tick();

    auto pt = f.last_prediction();
    ASSERT_NE(pt, nullptr);
    EXPECT_EQ(pt->predicted_latent.size(), 8);
    EXPECT_FALSE(std::isnan(pt->predicted_latent[0]));
}

// -- Online learning -----------------------------------------------------

TEST(DescendingPredictor, LearnsConstantTarget) {
    PredFixture f;

    Eigen::VectorXf c    = Eigen::VectorXf::Ones(4);
    Eigen::VectorXf tgt  = Eigen::VectorXf::LinSpaced(4, 0.0f, 1.0f);

    // Drive 200 ticks of consistent (consensus, reality) pairs.  Each tick
    // the predictor: forwards using consensus(t), publishes prediction, and
    // updates against reality(t-1) Feedback.
    for (uint64_t t = 0; t < 200; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",          make_consensus(c));
        f.bus.publish("reality.video.retinal", make_reality(tgt));
        f.pred.tick(t);
        f.bus.end_tick();
    }

    // After learning, the prediction should be close to the constant target.
    auto pt = f.last_prediction();
    ASSERT_NE(pt, nullptr);
    Eigen::VectorXf diff = pt->predicted_latent - tgt;
    EXPECT_LT(diff.norm(), 0.5f) << "  predicted=" << pt->predicted_latent.transpose()
                                  << "\n   target=" << tgt.transpose();
}

TEST(DescendingPredictor, ConfidenceImprovesWithCorrectTarget) {
    PredFixture f;

    Eigen::VectorXf c    = Eigen::VectorXf::Ones(4);
    Eigen::VectorXf tgt  = Eigen::VectorXf::LinSpaced(4, 0.0f, 1.0f);

    for (uint64_t t = 0; t < 300; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",          make_consensus(c));
        f.bus.publish("reality.video.retinal", make_reality(tgt));
        f.pred.tick(t);
        f.bus.end_tick();
    }
    EXPECT_GT(f.pred.confidence("reality.video.retinal"), 0.5f);
}

// -- Freeze --------------------------------------------------------------

TEST(DescendingPredictor, FreezeStopsWeightUpdates) {
    auto p = default_params();
    p["freeze_after_ticks"] = int64_t{10};
    PredFixture f(p);

    Eigen::VectorXf c    = Eigen::VectorXf::Ones(4);
    Eigen::VectorXf tgt  = Eigen::VectorXf::LinSpaced(4, 0.0f, 1.0f);

    // Run past the freeze threshold.
    for (uint64_t t = 0; t < 20; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",          make_consensus(c));
        f.bus.publish("reality.video.retinal", make_reality(tgt));
        f.pred.tick(t);
        f.bus.end_tick();
    }
    auto W_before = *f.pred.weights("reality.video.retinal");

    // Continue running with the same inputs — weights must not change after freeze.
    for (uint64_t t = 20; t < 100; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0",          make_consensus(c));
        f.bus.publish("reality.video.retinal", make_reality(tgt));
        f.pred.tick(t);
        f.bus.end_tick();
    }
    auto W_after = *f.pred.weights("reality.video.retinal");
    EXPECT_TRUE(W_before.isApprox(W_after, 1e-6f));
}

// -- Hot-mutation --------------------------------------------------------

TEST(DescendingPredictor, HotMutateLearningRate) {
    PredFixture f;
    EXPECT_NO_THROW(f.pred.on_param_change("learning_rate", ogma::ParamValue{0.005}));
}

TEST(DescendingPredictor, ConstructionOnlyParamThrows) {
    PredFixture f;
    EXPECT_THROW(f.pred.on_param_change("consensus_topic",
                                         ogma::ParamValue{std::string("consensus.1")}),
                 std::invalid_argument);
    EXPECT_THROW(f.pred.on_param_change("targets",
                                         ogma::ParamValue{std::vector<std::string>{"x"}}),
                 std::invalid_argument);
    EXPECT_THROW(f.pred.on_param_change("master_seed",
                                         ogma::ParamValue{int64_t{99}}),
                 std::invalid_argument);
}

TEST(DescendingPredictor, UnknownParamThrows) {
    PredFixture f;
    EXPECT_THROW(f.pred.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}

// -- Determinism ----------------------------------------------------------

TEST(DescendingPredictor, DeterministicAcrossTwoIdenticalRuns) {
    auto run = []() {
        PredFixture f;
        Eigen::VectorXf c   = Eigen::VectorXf::LinSpaced(4, -1.0f,  1.0f);
        Eigen::VectorXf tgt = Eigen::VectorXf::LinSpaced(4,  0.25f, 0.75f);
        for (uint64_t t = 0; t < 50; ++t) {
            f.bus.begin_tick(t);
            f.bus.publish("consensus.0",          make_consensus(c));
            f.bus.publish("reality.video.retinal", make_reality(tgt));
            f.pred.tick(t);
            f.bus.end_tick();
        }
        return f.last_prediction()->predicted_latent;
    };
    auto a = run();
    auto b = run();
    ASSERT_EQ(a.size(), b.size());
    for (int i = 0; i < a.size(); ++i) EXPECT_FLOAT_EQ(a[i], b[i]);
}

// -- Multi-target --------------------------------------------------------

TEST(DescendingPredictor, MultipleTargetsPublishIndependently) {
    auto p = default_params();
    p["targets"] = std::vector<std::string>{
        "reality.video.retinal",
        "reality.proprio.imu",
    };
    PredFixture f(p);

    EXPECT_EQ(f.pred.target_count(), 2);
    EXPECT_EQ(f.pred.output_topics().size(), 2u);

    f.bus.begin_tick(0);
    f.bus.publish("consensus.0",
        make_consensus(Eigen::VectorXf::Ones(4)));
    f.bus.publish("reality.video.retinal", make_reality(Eigen::VectorXf::Constant(4, 0.5f)));
    f.bus.publish("reality.proprio.imu",   make_reality(Eigen::VectorXf::Constant(4, -0.5f)));
    f.pred.tick(0);
    f.bus.end_tick();

    auto a = f.last_prediction("video.retinal");
    auto b = f.last_prediction("proprio.imu");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
}
