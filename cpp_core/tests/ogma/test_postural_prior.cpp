// =============================================================================
// test_postural_prior.cpp
//   PosturalPrior unit test (L-1b step 1): captures the standing rest pose from
//   the first proprio frame and publishes it as a soft PredictionToken objective;
//   rest_pos survives snapshot/restore.
// =============================================================================

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/PosturalPrior.hpp"
#include "ogma/Topics.hpp"

namespace {

using ogma::ParamMap;

std::shared_ptr<ogma::ProprioToken> leg_frame(std::vector<float> const& v) {
    auto pt = std::make_shared<ogma::ProprioToken>();
    pt->values = Eigen::VectorXf((int)v.size());
    for (int i = 0; i < (int)v.size(); ++i) pt->values[i] = v[i];
    return pt;
}

ParamMap pp_params() {
    ParamMap p;
    p["proprio_topics"]          = std::vector<std::string>{"leg.fl", "leg.fr", "leg.rl", "leg.rr"};
    p["objective_output_topics"] = std::vector<std::string>{"obj.fl", "obj.fr", "obj.rl", "obj.rr"};
    p["motor_dim"]        = int64_t{3};
    p["postural_gain"]    = 0.5;
    p["knee_tuck_target"] = 0.7;
    return p;
}

const char* IN[4]  = {"leg.fl", "leg.fr", "leg.rl", "leg.rr"};
const char* OUT[4] = {"obj.fl", "obj.fr", "obj.rl", "obj.rr"};

std::shared_ptr<const ogma::PredictionToken> read_obj(ogma::InProcessBus& bus, const char* topic) {
    return std::dynamic_pointer_cast<const ogma::PredictionToken>(bus.last_value(topic));
}

} // namespace

TEST(PosturalPrior, CapturesRestPoseAndPublishesObjective) {
    ogma::InProcessBus bus;
    ogma::PosturalPrior pp;
    pp.set_id("postural_prior");
    pp.on_setup(&bus, pp_params());
    EXPECT_EQ(pp.type_name(), "PosturalPrior");
    EXPECT_EQ(pp.n_legs(), 4);
    EXPECT_EQ(pp.legs_captured(), 0);

    // First frame: pos at index 3j = {0.1 (hip1), 0.2 (hip2), 0.9 (knee)}.
    bus.begin_tick(0);
    for (int leg = 0; leg < 4; ++leg)
        bus.publish(IN[leg], leg_frame({0.1f, 0, 0, 0.2f, 0, 0, 0.9f, 0, 0}));
    pp.tick(0);
    bus.end_tick();

    EXPECT_EQ(pp.legs_captured(), 4);
    for (int leg = 0; leg < 4; ++leg) {
        auto tok = read_obj(bus, OUT[leg]);
        ASSERT_TRUE(tok) << "leg " << leg << " objective not published";
        ASSERT_EQ(tok->predicted_latent.size(), 3);
        EXPECT_FLOAT_EQ(tok->predicted_latent[0], 0.1f);   // hip1 rest = captured pos
        EXPECT_FLOAT_EQ(tok->predicted_latent[1], 0.2f);   // hip2 rest = captured pos
        EXPECT_FLOAT_EQ(tok->predicted_latent[2], 0.7f);   // knee rest = tuck override, NOT 0.9
        EXPECT_FLOAT_EQ(tok->confidence, 0.5f);            // objective weight w
    }

    // A later, DIFFERENT frame must NOT re-capture (rest pose is a one-shot spawn capture).
    bus.begin_tick(1);
    for (int leg = 0; leg < 4; ++leg)
        bus.publish(IN[leg], leg_frame({0.7f, 0, 0, -0.3f, 0, 0, -1.2f, 0, 0}));
    pp.tick(1);
    bus.end_tick();
    EXPECT_FLOAT_EQ(read_obj(bus, OUT[0])->predicted_latent[0], 0.1f) << "rest pose must not re-capture";
}

TEST(PosturalPrior, SnapshotRestoreRoundTripsRestPose) {
    ogma::InProcessBus bus;
    ogma::PosturalPrior pp;
    pp.set_id("postural_prior");
    pp.on_setup(&bus, pp_params());

    bus.begin_tick(0);
    for (int leg = 0; leg < 4; ++leg)
        bus.publish(IN[leg], leg_frame({0.1f, 0, 0, 0.2f, 0, 0, 0.9f, 0, 0}));
    pp.tick(0);
    bus.end_tick();
    ASSERT_EQ(pp.legs_captured(), 4);

    auto snap = pp.snapshot_state();
    EXPECT_EQ(snap["version"].get<int>(), 1);

    // Fresh instance restored from the snapshot — NO new proprio; it must publish the
    // restored rest pose (never re-capture a possibly-fallen pose after a checkpoint).
    ogma::PosturalPrior pp2;
    pp2.set_id("postural_prior");
    pp2.on_setup(&bus, pp_params());
    EXPECT_EQ(pp2.legs_captured(), 0);
    pp2.restore_state(snap);
    EXPECT_EQ(pp2.legs_captured(), 4);

    bus.begin_tick(1);
    pp2.tick(1);
    bus.end_tick();
    auto tok = read_obj(bus, OUT[0]);
    ASSERT_TRUE(tok);
    EXPECT_FLOAT_EQ(tok->predicted_latent[0], 0.1f);
    EXPECT_FLOAT_EQ(tok->predicted_latent[2], 0.7f);
}
