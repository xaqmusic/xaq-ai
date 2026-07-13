// =============================================================================
// pair_descendingpred_epm.cpp
//
// Real DescendingPredictor ↔ real EPM.  Verifies the top-down loop seam:
// the predictor publishes prediction.<modality>(t); the EPM consumes
// prediction.<modality>(t-1) via Feedback at tick t+1 and subtracts it
// from its encoder output before stepping the GNG.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/DescendingPredictor.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap epm_params(bool subtract_prediction) {
    return {
        {"modality_group",     std::string("proprio")},
        {"modality_name",      std::string("imu")},
        {"encoder_kind",       std::string("rbf")},
        {"input_topic",        std::string("reality.proprio.imu")},
        {"projection_dim",     int64_t{16}},
        {"proprio_state_dims", int64_t{6}},
        {"baking_threshold",   int64_t{15}},
        {"min_insertion_error", 0.001},
        {"subtract_descending_prediction", subtract_prediction},
    };
}

ogma::ParamMap predictor_params() {
    return {
        {"consensus_topic",   std::string("consensus.0")},
        {"targets",           std::vector<std::string>{"reality.proprio.imu"}},
        {"learning_rate",     0.05},
        {"init_noise_scale",  0.0},
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

std::shared_ptr<ogma::ConsensusToken> make_consensus(int dim, float fill) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding = Eigen::VectorXf::Constant(dim, fill);
    c->fused_tle       = 0.1f;
    return c;
}

// Run one tick with predictor BEFORE EPM (DAG: voter → predictor → EPM,
// even though the host publishes consensus synthetically here).
void run_tick(ogma::InProcessBus& bus,
              ogma::DescendingPredictor& pred,
              ogma::EPM& epm,
              uint64_t t,
              std::shared_ptr<ogma::ProprioToken> proprio,
              std::shared_ptr<ogma::ConsensusToken> consensus) {
    bus.begin_tick(t);
    if (consensus) bus.publish("consensus.0", consensus);
    if (proprio)   bus.publish("reality.proprio.imu", proprio);
    pred.tick(t);
    epm.tick(t);
    bus.end_tick();
}

} // namespace

TEST(PairDescendingPredEpm, EpmReceivesPredictionViaFeedback) {
    ogma::InProcessBus           bus;
    ogma::DescendingPredictor    pred;
    ogma::EPM                    epm;
    pred.set_id("predictor_0");
    epm.set_id("epm_imu");
    pred.on_setup(&bus, predictor_params());
    epm.on_setup(&bus, epm_params(/*subtract_prediction=*/true));

    // 100 ticks with a stable consensus signal — predictor should converge
    // and EPM should keep producing tokens with valid TLE values.
    for (uint64_t t = 0; t < 100; ++t) {
        run_tick(bus, pred, epm, t,
                 make_proprio6(0.1f * float(t)),
                 make_consensus(8, 0.5f));
    }

    auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(
        bus.last_value("reality.proprio.imu"));
    ASSERT_NE(rt, nullptr);
    EXPECT_GE(rt->winner_id, 0);
    EXPECT_FALSE(std::isnan(rt->tle));
}

TEST(PairDescendingPredEpm, FirstTickNoNanWithEmptyPrediction) {
    ogma::InProcessBus           bus;
    ogma::DescendingPredictor    pred;
    ogma::EPM                    epm;
    pred.set_id("predictor_0");
    epm.set_id("epm_imu");
    pred.on_setup(&bus, predictor_params());
    epm.on_setup(&bus, epm_params(true));

    EXPECT_NO_THROW({
        run_tick(bus, pred, epm, 0,
                 make_proprio6(0.0f),
                 make_consensus(8, 0.5f));
    });

    auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(
        bus.last_value("reality.proprio.imu"));
    ASSERT_NE(rt, nullptr);
    for (int i = 0; i < rt->latent.size(); ++i)
        EXPECT_FALSE(std::isnan(rt->latent[i]));
}

TEST(PairDescendingPredEpm, PredictionSubtractionAffectsGngTopology) {
    // Two parallel runs — same input distribution, only difference is
    // whether the EPM subtracts predictions.  After enough learning,
    // node-count or TLE distributions should differ.  The direction of
    // difference depends on the input distribution; we just assert
    // they aren't identical.
    auto run = [](bool subtract) {
        ogma::InProcessBus           bus;
        ogma::DescendingPredictor    pred;
        ogma::EPM                    epm;
        pred.set_id("predictor_0");
        epm.set_id("epm_imu");
        pred.on_setup(&bus, predictor_params());
        epm.on_setup(&bus, epm_params(subtract));

        for (uint64_t t = 0; t < 200; ++t) {
            run_tick(bus, pred, epm, t,
                     make_proprio6(0.05f * float(t)),
                     make_consensus(8, 0.5f + 0.1f * std::sin(0.1f * t)));
        }
        auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(
            bus.last_value("reality.proprio.imu"));
        return std::make_tuple(epm.node_count(), rt ? rt->tle : 0.0f);
    };
    auto with_sub    = run(true);
    auto without_sub = run(false);

    // We don't claim a directional improvement here — just that the
    // prediction path is wired in and produces a different state.
    bool topology_or_tle_differs =
        (std::get<0>(with_sub) != std::get<0>(without_sub)) ||
        (std::get<1>(with_sub) != std::get<1>(without_sub));
    EXPECT_TRUE(topology_or_tle_differs);
}
