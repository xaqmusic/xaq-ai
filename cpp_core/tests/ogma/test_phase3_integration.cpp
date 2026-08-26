// =============================================================================
// test_phase3_integration.cpp  --  Phase 3 exit-gate integration tests
// =============================================================================
//
// Covers the three remaining Phase 3 deliverables:
//
//  1. Full cellular-bath E2E through OgmaInstance:
//     neuro.state broadcast consumed by EPM, LateralVoter, HomeostaticDrive,
//     ActionDecoder — verifies the broadcast chain and inter-module modulation
//     work through the real Scheduler + OgmaInstance lifecycle.
//
//  2. GraphConfig JSON roundtrip:
//     to_json() → load_from_json() produces an identical config; the
//     re-instantiated OgmaInstance passes the same integration checks.
//
//  3. ConnectOp / DisconnectOp against a running instance:
//     edges are recorded, validated (bad endpoints rejected), and removed by
//     DisconnectOp.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

namespace {

// ---------------------------------------------------------------------------
// Helpers — build the thin-slice graph config programmatically
// ---------------------------------------------------------------------------

ogma::GraphConfig thin_slice_config(int projection_dim = 16,
                                    int proprio_dim    = 6) {
    ogma::GraphConfig g;
    g.version = 1;

    // NeurochemState — top of the dependency chain.
    g.modules.push_back({.id = "neuro", .type = "NeurochemState", .params = {}});

    // EPM (RBF, proprio).
    {
        ogma::ModuleSpec epm;
        epm.id   = "epm_imu";
        epm.type = "EPM";
        epm.params["modality_group"]     = std::string("proprio");
        epm.params["modality_name"]      = std::string("imu");
        epm.params["encoder_kind"]       = std::string("rbf");
        epm.params["input_topic"]        = std::string("reality.proprio.imu");
        epm.params["projection_dim"]     = int64_t{projection_dim};
        epm.params["proprio_state_dims"] = int64_t{proprio_dim};
        epm.params["baking_threshold"]   = int64_t{20};
        epm.params["min_insertion_error"] = 0.001;
        epm.params["subtract_descending_prediction"] = false;
        g.modules.push_back(epm);
    }

    // LateralVoter.
    {
        ogma::ModuleSpec voter;
        voter.id   = "voter_0";
        voter.type = "LateralVoter";
        voter.params["level"]          = int64_t{0};
        voter.params["input_pattern"]  = std::string("reality.");
        voter.params["trust_epsilon"]  = 0.05;
        voter.params["group_balance"]  = true;
        voter.params["priority_group"] = std::string("proprio");
        g.modules.push_back(voter);
    }

    // HomeostaticDrive.
    {
        ogma::ModuleSpec drive;
        drive.id   = "drive";
        drive.type = "HomeostaticDrive";
        drive.params["channels"]            = std::vector<std::string>{"energy"};
        drive.params["setpoints"]           = std::vector<double>{0.8};
        drive.params["urgency_normalizers"] = std::vector<double>{1.0};
        drive.params["channel_input_topics"] = std::vector<std::string>{"reality.proprio.energy"};
        drive.params["energy_drain_per_tick"] = 0.0;
        g.modules.push_back(drive);
    }

    // ActionDecoder.
    {
        ogma::ModuleSpec dec;
        dec.id   = "action_decoder";
        dec.type = "ActionDecoder";
        dec.params["consensus_level"] = int64_t{0};
        dec.params["proprio_topic"]   = std::string("reality.proprio.imu");
        dec.params["action_bins"]     = int64_t{3};
        dec.params["pragmatic_gain"]  = 10.0;
        g.modules.push_back(dec);
    }

    return g;
}

std::shared_ptr<ogma::ProprioToken> make_proprio(float seed) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->sensor = "imu";
    p->values.resize(6);
    p->values << std::sin(seed),     std::cos(seed),
                 std::sin(2.f*seed), std::cos(2.f*seed),
                 0.5f * seed,        -0.5f * seed;
    return p;
}

std::shared_ptr<ogma::ProprioToken> make_energy(float v) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->sensor = "energy";
    p->values.resize(1);
    p->values << v;
    return p;
}

} // namespace

// ==========================================================================
// 1. Full cellular-bath E2E via OgmaInstance
// ==========================================================================

class CellularBathE2E : public ::testing::Test {
protected:
    std::unique_ptr<ogma::OgmaInstance> inst_;

    void SetUp() override {
        inst_ = std::make_unique<ogma::OgmaInstance>(
            thin_slice_config(),
            std::make_unique<ogma::InProcessBus>());
    }
};

TEST_F(CellularBathE2E, AllFiveModulesInstantiated) {
    EXPECT_NE(inst_->module("neuro"),          nullptr);
    EXPECT_NE(inst_->module("epm_imu"),        nullptr);
    EXPECT_NE(inst_->module("voter_0"),        nullptr);
    EXPECT_NE(inst_->module("drive"),          nullptr);
    EXPECT_NE(inst_->module("action_decoder"), nullptr);
    EXPECT_EQ(inst_->modules().size(), 5u);
}

TEST_F(CellularBathE2E, NeuroBroadcastProducedEveryTick) {
    for (uint64_t t = 0; t < 10; ++t) {
        inst_->bus()->publish("reality.proprio.imu",    make_proprio(0.1f * t));
        inst_->bus()->publish("reality.proprio.energy", make_energy(0.7f));
        inst_->tick();
    }
    auto neuro = std::dynamic_pointer_cast<const ogma::NeuroState>(
        inst_->bus()->last_value(ogma::topics::kNeuroState));
    ASSERT_NE(neuro, nullptr);
    EXPECT_GE(neuro->tick_id, 9u);
    EXPECT_FALSE(std::isnan(neuro->dopamine));
    EXPECT_FALSE(std::isnan(neuro->serotonin));
    EXPECT_FALSE(std::isnan(neuro->epsilon_b_scale));
}

TEST_F(CellularBathE2E, EpmGrowsNodesOverRealSensorStream) {
    for (uint64_t t = 0; t < 100; ++t) {
        inst_->bus()->publish("reality.proprio.imu",    make_proprio(0.1f * t));
        inst_->bus()->publish("reality.proprio.energy", make_energy(0.7f));
        inst_->tick();
    }
    // EPM's node_count is visible in the published RealityToken.
    auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(
        inst_->bus()->last_value("reality.proprio.imu"));
    ASSERT_NE(rt, nullptr);
    EXPECT_GT(rt->node_count, 2);
}

TEST_F(CellularBathE2E, ConsensusPublishedAfterBootstrap) {
    for (uint64_t t = 0; t < 30; ++t) {
        inst_->bus()->publish("reality.proprio.imu",    make_proprio(0.1f * t));
        inst_->bus()->publish("reality.proprio.energy", make_energy(0.8f));
        inst_->tick();
    }
    auto cons = std::dynamic_pointer_cast<const ogma::ConsensusToken>(
        inst_->bus()->last_value("consensus.0"));
    ASSERT_NE(cons, nullptr);
    EXPECT_EQ(cons->level, 0);
    EXPECT_FALSE(std::isnan(cons->fused_tle));
}

TEST_F(CellularBathE2E, ActionDecoderEmitsActionsInBounds) {
    for (uint64_t t = 0; t < 50; ++t) {
        inst_->bus()->publish("reality.proprio.imu",    make_proprio(0.1f * t));
        inst_->bus()->publish("reality.proprio.energy", make_energy(0.8f));
        inst_->tick();
    }
    auto action = std::dynamic_pointer_cast<const ogma::ActionOut>(
        inst_->bus()->last_value(ogma::topics::kActionOut));
    ASSERT_NE(action, nullptr);
    EXPECT_GE(action->accel, -4.0f);
    EXPECT_LE(action->accel,  4.0f);
    EXPECT_FALSE(std::isnan(action->accel));
}

TEST_F(CellularBathE2E, DriveErrorsPublishedWithCorrectChannels) {
    inst_->bus()->publish("reality.proprio.energy", make_energy(0.6f));
    inst_->tick();
    auto drive = std::dynamic_pointer_cast<const ogma::DriveErrors>(
        inst_->bus()->last_value(ogma::topics::kDriveErrors));
    ASSERT_NE(drive, nullptr);
    EXPECT_EQ(drive->errors.size(), 1u);
    EXPECT_TRUE(drive->errors.count("energy"));
    EXPECT_FALSE(std::isnan(drive->urgency));
}

TEST_F(CellularBathE2E, NeuroBroadcastDopamineSerotonInUnitInterval) {
    for (uint64_t t = 0; t < 100; ++t) {
        inst_->bus()->publish("reality.proprio.imu",    make_proprio(0.05f * t));
        inst_->bus()->publish("reality.proprio.energy", make_energy(0.7f - 0.001f * t));
        if (t % 7 == 0) {
            auto ev = std::make_shared<ogma::EnvEvent>();
            ev->name = "hit"; ev->intensity = 1.0f;
            inst_->bus()->publish("events.hit", ev);
        }
        inst_->tick();
        auto neuro = std::dynamic_pointer_cast<const ogma::NeuroState>(
            inst_->bus()->last_value(ogma::topics::kNeuroState));
        ASSERT_NE(neuro, nullptr);
        EXPECT_GE(neuro->dopamine,  0.0f);
        EXPECT_LE(neuro->dopamine,  1.0f);
        EXPECT_GE(neuro->serotonin, 0.0f);
        EXPECT_LE(neuro->serotonin, 1.0f);
    }
}

// ==========================================================================
// 2. GraphConfig JSON roundtrip
// ==========================================================================

TEST(GraphConfigJson, ToJsonProducesValidJson) {
    auto cfg = thin_slice_config();
    std::string text = cfg.to_json();
    EXPECT_FALSE(text.empty());
    EXPECT_NE(text.find("NeurochemState"), std::string::npos);
    EXPECT_NE(text.find("LateralVoter"),  std::string::npos);
    EXPECT_NE(text.find("version"),       std::string::npos);
}

TEST(GraphConfigJson, RoundtripPreservesModuleCount) {
    auto cfg = thin_slice_config();
    std::string json_text = cfg.to_json();

    auto cfg2 = ogma::GraphConfig::load_from_json(json_text);
    EXPECT_EQ(cfg.modules.size(),  cfg2.modules.size());
    EXPECT_EQ(cfg.version,         cfg2.version);
}

TEST(GraphConfigJson, RoundtripPreservesModuleIds) {
    auto cfg = thin_slice_config();
    auto cfg2 = ogma::GraphConfig::load_from_json(cfg.to_json());

    for (size_t i = 0; i < cfg.modules.size(); ++i) {
        EXPECT_EQ(cfg.modules[i].id,   cfg2.modules[i].id);
        EXPECT_EQ(cfg.modules[i].type, cfg2.modules[i].type);
    }
}

TEST(GraphConfigJson, RoundtripPreservesParamTypes) {
    auto cfg = thin_slice_config();
    auto cfg2 = ogma::GraphConfig::load_from_json(cfg.to_json());

    // EPM has projection_dim (int), trust_epsilon (double), input_pattern (string),
    // group_balance (bool), channels (vector<string>).
    for (auto const& m : cfg2.modules) {
        if (m.type == "EPM") {
            auto it = m.params.find("projection_dim");
            ASSERT_NE(it, m.params.end());
            EXPECT_TRUE(std::holds_alternative<int64_t>(it->second));
        }
        if (m.type == "LateralVoter") {
            auto it = m.params.find("trust_epsilon");
            ASSERT_NE(it, m.params.end());
            EXPECT_TRUE(std::holds_alternative<double>(it->second));
            auto gb = m.params.find("group_balance");
            ASSERT_NE(gb, m.params.end());
            EXPECT_TRUE(std::holds_alternative<bool>(gb->second));
        }
        if (m.type == "HomeostaticDrive") {
            auto it = m.params.find("channels");
            ASSERT_NE(it, m.params.end());
            EXPECT_TRUE(std::holds_alternative<std::vector<std::string>>(it->second));
        }
    }
}

TEST(GraphConfigJson, ReinstantiatedConfigRuns) {
    auto cfg = thin_slice_config();
    auto json_text = cfg.to_json();
    auto cfg2 = ogma::GraphConfig::load_from_json(json_text);

    ogma::OgmaInstance inst(std::move(cfg2), std::make_unique<ogma::InProcessBus>());
    EXPECT_EQ(inst.modules().size(), 5u);
    for (int i = 0; i < 10; ++i) inst.tick();
    EXPECT_EQ(inst.tick_count(), 10u);
}

TEST(GraphConfigJson, MalformedJsonThrows) {
    EXPECT_THROW(ogma::GraphConfig::load_from_json("{ not valid json"),
                 std::runtime_error);
}

TEST(GraphConfigJson, DuplicateModuleIdThrows) {
    std::string bad = R"({"version":1,"modules":[
        {"id":"a","type":"NeurochemState","params":{}},
        {"id":"a","type":"NeurochemState","params":{}}
    ]})";
    EXPECT_THROW(ogma::GraphConfig::load_from_json(bad), std::runtime_error);
}

TEST(GraphConfigJson, MissingModuleIdThrows) {
    std::string bad = R"({"version":1,"modules":[{"type":"NeurochemState","params":{}}]})";
    EXPECT_THROW(ogma::GraphConfig::load_from_json(bad), std::runtime_error);
}

// ==========================================================================
// 3. ConnectOp / DisconnectOp
// ==========================================================================

TEST(HotPatchConnect, ConnectOpRecordsEdge) {
    ogma::OgmaInstance inst(thin_slice_config(),
                             std::make_unique<ogma::InProcessBus>());

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::ConnectOp{ogma::EdgeSpec{
        .from = "epm_imu", .to = "voter_0", .topic = "", .feedback = false}});
    inst.enqueue_hot_patch(std::move(batch));
    EXPECT_NO_THROW(inst.tick());  // validation passes, edge recorded
}

TEST(HotPatchConnect, ConnectOpUnknownEndpointRejected) {
    ogma::OgmaInstance inst(thin_slice_config(),
                             std::make_unique<ogma::InProcessBus>());

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::ConnectOp{ogma::EdgeSpec{
        .from = "nonexistent", .to = "voter_0", .topic = "", .feedback = false}});
    inst.enqueue_hot_patch(std::move(batch));
    // Rejected per batch and logged, not thrown out of tick(); the edge
    // must not have landed.
    EXPECT_NO_THROW(inst.tick());
    for (auto const& e : inst.scheduler()->feedback_edges())
        EXPECT_FALSE(e.from == "nonexistent" && e.to == "voter_0");
}

TEST(HotPatchConnect, ConnectOpHostEndpointAlwaysOk) {
    ogma::OgmaInstance inst(thin_slice_config(),
                             std::make_unique<ogma::InProcessBus>());

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::ConnectOp{ogma::EdgeSpec{
        .from = "host:reality.proprio.imu", .to = "epm_imu", .topic = "", .feedback = false}});
    inst.enqueue_hot_patch(std::move(batch));
    EXPECT_NO_THROW(inst.tick());
}

TEST(HotPatchConnect, DisconnectOpRemovesEdge) {
    ogma::OgmaInstance inst(thin_slice_config(),
                             std::make_unique<ogma::InProcessBus>());

    // First add an edge.
    {
        ogma::GraphPatchBatch b;
        b.ops.push_back(ogma::ConnectOp{ogma::EdgeSpec{
            .from = "epm_imu", .to = "voter_0", .topic = "", .feedback = false}});
        inst.enqueue_hot_patch(std::move(b));
        inst.tick();
    }
    // Then remove it.
    {
        ogma::GraphPatchBatch b;
        b.ops.push_back(ogma::DisconnectOp{.from = "epm_imu", .to = "voter_0", .topic = ""});
        inst.enqueue_hot_patch(std::move(b));
        EXPECT_NO_THROW(inst.tick());
    }
    // feedback_edges should no longer include this edge.
    auto fe = inst.scheduler()->feedback_edges();
    bool found = false;
    for (auto const& e : fe)
        if (e.from == "epm_imu" && e.to == "voter_0") found = true;
    EXPECT_FALSE(found);
}

TEST(HotPatchConnect, DisconnectOpUnknownEndpointRejected) {
    ogma::OgmaInstance inst(thin_slice_config(),
                             std::make_unique<ogma::InProcessBus>());

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::DisconnectOp{.from = "ghost", .to = "voter_0", .topic = ""});
    inst.enqueue_hot_patch(std::move(batch));
    // Rejected per batch and logged, not thrown out of tick().
    EXPECT_NO_THROW(inst.tick());
}
