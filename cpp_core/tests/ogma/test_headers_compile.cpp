// =============================================================================
// test_headers_compile.cpp  --  Phase 0 surface compatibility smoke test
// =============================================================================
//
// This test does not exercise any runtime behaviour.  It exists to catch
// compile-time regressions in the frozen Phase 0 header surface:
//
//   - All ogma/* headers include cleanly together.
//   - The TopicSpec / ParamSchema / Bus / Module / Scheduler types compose.
//   - A trivial Module subclass can be declared against the contract.
//
// Phase 1 will replace this file with real per-module unit + pair tests.

#include <gtest/gtest.h>

#include <typeindex>

#include "ogma/Bus.hpp"
#include "ogma/GraphConfig.hpp"
#include "ogma/Module.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Scheduler.hpp"
#include "ogma/Topics.hpp"

namespace {

// -----------------------------------------------------------------------------
// Trivial Module subclass — exercises every pure-virtual override.
// -----------------------------------------------------------------------------

class StubModule : public ogma::Module {
public:
    std::string_view type_name() const override { return "Stub"; }

    std::vector<ogma::TopicSpec> input_topics() const override {
        return { ogma::TopicSpec{ogma::topics::kNeuroState,
                                  std::type_index(typeid(ogma::NeuroState))} };
    }

    std::vector<ogma::TopicSpec> output_topics() const override {
        return { ogma::TopicSpec{"reality.video.stub",
                                  std::type_index(typeid(ogma::RealityToken))} };
    }

    ogma::ParamSchema params_schema() const override { return {}; }

    void on_setup(ogma::Bus* bus, ogma::ParamMap const& /*params*/) override {
        bus_ = bus;
    }

    void tick(uint64_t /*tick_id*/) override {}
};

} // namespace

TEST(Phase0Headers, TopicMessageTypesAreDefined) {
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::RealityToken>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::ConsensusToken>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::NeuroState>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::DriveErrors>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::ActionOut>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::PredictionToken>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::SequenceMotif>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::MotorChunks>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::RolloutQuery>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::RolloutResult>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::ProprioToken>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::EnvEvent>::value));
    EXPECT_TRUE((std::is_base_of<ogma::Message, ogma::FitnessScore>::value));
}

TEST(Phase0Headers, StubModuleCompiles) {
    StubModule m;
    m.set_id("stub_0");
    EXPECT_EQ(m.id(), "stub_0");
    EXPECT_EQ(m.type_name(), "Stub");
    EXPECT_EQ(m.input_topics().size(), 1u);
    EXPECT_EQ(m.output_topics().size(), 1u);
    EXPECT_TRUE(m.input_topics()[0].name == ogma::topics::kNeuroState);
    EXPECT_EQ(m.input_topics()[0].kind, ogma::SubscriptionKind::Direct);
}

TEST(Phase0Headers, ParamValueVariantHoldsExpectedTypes) {
    ogma::ParamValue v_bool   = true;
    ogma::ParamValue v_int    = int64_t{42};
    ogma::ParamValue v_double = 3.14;
    ogma::ParamValue v_string = std::string{"hello"};
    ogma::ParamValue v_doubles = std::vector<double>{1.0, 2.0};
    ogma::ParamValue v_strings = std::vector<std::string>{"a", "b"};

    EXPECT_TRUE(std::holds_alternative<bool>(v_bool));
    EXPECT_TRUE(std::holds_alternative<int64_t>(v_int));
    EXPECT_TRUE(std::holds_alternative<double>(v_double));
    EXPECT_TRUE(std::holds_alternative<std::string>(v_string));
    EXPECT_TRUE(std::holds_alternative<std::vector<double>>(v_doubles));
    EXPECT_TRUE(std::holds_alternative<std::vector<std::string>>(v_strings));
}

TEST(Phase0Headers, GraphPatchOpVariantHoldsExpectedTypes) {
    ogma::GraphPatchOp add_op   = ogma::AddNodeOp{ogma::ModuleSpec{"x", "Stub", {}}};
    ogma::GraphPatchOp rm_op    = ogma::RemoveNodeOp{"x"};
    ogma::GraphPatchOp conn_op  = ogma::ConnectOp{ogma::EdgeSpec{"a", "b", "", false}};
    ogma::GraphPatchOp disc_op  = ogma::DisconnectOp{"a", "b", ""};
    ogma::GraphPatchOp set_op   = ogma::SetParamOp{"a", "k", int64_t{1}};

    EXPECT_TRUE(std::holds_alternative<ogma::AddNodeOp>(add_op));
    EXPECT_TRUE(std::holds_alternative<ogma::RemoveNodeOp>(rm_op));
    EXPECT_TRUE(std::holds_alternative<ogma::ConnectOp>(conn_op));
    EXPECT_TRUE(std::holds_alternative<ogma::DisconnectOp>(disc_op));
    EXPECT_TRUE(std::holds_alternative<ogma::SetParamOp>(set_op));
}

TEST(Phase0Headers, SubscriptionKindEnumExists) {
    auto direct   = ogma::SubscriptionKind::Direct;
    auto feedback = ogma::SubscriptionKind::Feedback;
    EXPECT_NE(static_cast<int>(direct), static_cast<int>(feedback));
}

TEST(Phase0Headers, ThreadPoolPolicyEnumExists) {
    auto per_inst = ogma::ThreadPoolPolicy::PerInstance;
    auto shared   = ogma::ThreadPoolPolicy::Shared;
    EXPECT_NE(static_cast<int>(per_inst), static_cast<int>(shared));
}
