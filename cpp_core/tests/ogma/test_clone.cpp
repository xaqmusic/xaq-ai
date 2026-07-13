// =============================================================================
// test_clone.cpp  --  Phase 6.5.4 OgmaInstance::clone() determinism contract
// =============================================================================
//
// Builds a small graph, ticks it some, clones it, then ticks both source AND
// clone with identical synthetic inputs and verifies they produce IDENTICAL
// outputs over many subsequent ticks.  This is the byte-equivalence contract
// that makes evolutionary forks and frozen-eval benchmarks meaningful: the
// clone IS the source's brain, branched at a moment in time.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>

#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

namespace {

// Build a graph that exercises the modules MountainCar uses: NeurochemState +
// EPM + LateralVoter + ActionDecoder + HomeostaticDrive.  Skip SequenceGNG /
// MotorRepertoire / HomeokineticExploration to keep the harness compact;
// each is unit-tested individually plus by the body-level smoke tests.
ogma::GraphConfig minimal_mc_like_graph() {
    ogma::GraphConfig g;
    g.version = 1;

    {   ogma::ModuleSpec m;
        m.id   = "neuro";
        m.type = "NeurochemState";
        m.params["event_coupled_da"] = ogma::ParamValue{true};
        m.params["da_baseline_ema_alpha"] = ogma::ParamValue{0.001};
        g.modules.push_back(m); }
    {   ogma::ModuleSpec m;
        m.id   = "epm_state";
        m.type = "EPM";
        m.params["modality_group"] = std::string("kinematic");
        m.params["modality_name"]  = std::string("state");
        m.params["encoder_kind"]   = std::string("rbf");
        m.params["input_topic"]    = std::string("reality.proprio.state");
        m.params["projection_dim"] = int64_t{16};
        m.params["proprio_state_dims"] = int64_t{2};
        m.params["subtract_descending_prediction"] = ogma::ParamValue{false};
        g.modules.push_back(m); }
    {   ogma::ModuleSpec m;
        m.id   = "voter_0";
        m.type = "LateralVoter";
        g.modules.push_back(m); }
    {   ogma::ModuleSpec m;
        m.id   = "drive";
        m.type = "HomeostaticDrive";
        m.params["channels"] = std::vector<std::string>{"alive_pulse"};
        m.params["channel_kinds"] = std::vector<std::string>{"alive_pulse"};
        m.params["setpoints"] = std::vector<double>{0.8};
        m.params["urgency_normalizers"] = std::vector<double>{1.0};
        m.params["channel_input_topics"] = std::vector<std::string>{"events.alive"};
        g.modules.push_back(m); }
    {   ogma::ModuleSpec m;
        m.id   = "action_decoder";
        m.type = "ActionDecoder";
        m.params["consensus_level"] = int64_t{0};
        m.params["proprio_topic"]   = std::string("reality.kinematic.state");
        m.params["action_bins"]     = int64_t{3};
        m.params["accel_min"]       = double{-1.0};
        m.params["accel_max"]       = double{ 1.0};
        m.params["td_gamma"]        = double{0.95};
        g.modules.push_back(m); }
    return g;
}

// Publish synthetic body inputs (one proprio frame, one event burst).
// Uses a deterministic RNG seeded from `seed` so source and clone receive
// identical input sequences.
void publish_synthetic_input(ogma::Bus* bus, uint64_t tick) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id = tick;
    p->sensor  = "state";
    p->values.resize(2);
    // Deterministic sinusoidal trajectory.
    p->values(0) = float(std::sin(double(tick) * 0.07));
    p->values(1) = float(std::cos(double(tick) * 0.13));
    bus->publish("reality.proprio.state", p);
    if (tick % 30 == 0) {
        auto ev = std::make_shared<ogma::EnvEvent>();
        ev->tick_id   = tick;
        ev->name      = "hit";
        ev->intensity = 1.0f;
        bus->publish("events.hit", ev);
    }
}

// Returns the recent action.out value as a stable digest.
double action_digest(ogma::Bus const* bus) {
    auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus->last_value(ogma::topics::kActionOut));
    if (!a) return 0.0;
    return a->accel;
}

double dopamine_digest(ogma::Bus const* bus) {
    auto n = std::dynamic_pointer_cast<const ogma::NeuroState>(
        bus->last_value(ogma::topics::kNeuroState));
    if (!n) return 0.0;
    return n->dopamine;
}

}  // namespace

TEST(OgmaInstanceClone, EmptyClonePreservesConfig) {
    auto src = std::make_unique<ogma::OgmaInstance>(
        minimal_mc_like_graph(), std::make_unique<ogma::InProcessBus>());
    auto clone = src->clone();

    // Same module set.
    auto src_mods   = src->modules();
    auto clone_mods = clone->modules();
    ASSERT_EQ(src_mods.size(), clone_mods.size());
    for (size_t i = 0; i < src_mods.size(); ++i) {
        EXPECT_EQ(src_mods[i]->id(),        clone_mods[i]->id());
        EXPECT_EQ(src_mods[i]->type_name(), clone_mods[i]->type_name());
    }
    // Different bus instance.
    EXPECT_NE(src->bus(), clone->bus());
}

TEST(OgmaInstanceClone, ClonedBrainIsByteEquivalentForN_Ticks) {
    auto src = std::make_unique<ogma::OgmaInstance>(
        minimal_mc_like_graph(), std::make_unique<ogma::InProcessBus>());
    // Warm-start the source with 50 ticks of inputs so it builds non-trivial
    // GNG/Hebbian/valence state.
    for (uint64_t t = 0; t < 50; ++t) {
        publish_synthetic_input(src->bus(), t);
        src->tick();
    }

    auto clone = src->clone();

    // Now run BOTH for 100 more ticks with identical inputs.  Every tick's
    // emitted action should match between source and clone.
    for (uint64_t t = 50; t < 150; ++t) {
        publish_synthetic_input(src->bus(),   t);
        publish_synthetic_input(clone->bus(), t);
        src->tick();
        clone->tick();

        double src_a   = action_digest(src->bus());
        double clone_a = action_digest(clone->bus());
        ASSERT_DOUBLE_EQ(src_a, clone_a)
            << "Action divergence at tick " << t
            << " — clone is not byte-equivalent to source.";

        double src_d   = dopamine_digest(src->bus());
        double clone_d = dopamine_digest(clone->bus());
        ASSERT_DOUBLE_EQ(src_d, clone_d)
            << "Dopamine divergence at tick " << t;
    }
}

TEST(OgmaInstanceClone, CloneIsIndependentOfSourceAfterDivergence) {
    auto src = std::make_unique<ogma::OgmaInstance>(
        minimal_mc_like_graph(), std::make_unique<ogma::InProcessBus>());
    for (uint64_t t = 0; t < 30; ++t) {
        publish_synthetic_input(src->bus(), t);
        src->tick();
    }
    auto clone = src->clone();

    // Drive source and clone with DIFFERENT inputs.  They should diverge.
    for (uint64_t t = 30; t < 80; ++t) {
        // Source gets sinusoidal input as before.
        publish_synthetic_input(src->bus(), t);
        // Clone gets a constant input.
        auto p = std::make_shared<ogma::ProprioToken>();
        p->tick_id = t;
        p->sensor  = "state";
        p->values.resize(2);
        p->values(0) = 0.0f;
        p->values(1) = 0.0f;
        clone->bus()->publish("reality.proprio.state", p);
        src->tick();
        clone->tick();
    }
    // After 50 divergent ticks, dopamine state can match-or-differ but the
    // brains are guaranteed independent (no shared mutable state).  Verify
    // by checking that publishing to one bus does not affect the other.
    auto p_src = std::make_shared<ogma::ProprioToken>();
    p_src->sensor = "state";
    p_src->values.resize(2);
    p_src->values(0) = 1.0f; p_src->values(1) = 1.0f; p_src->tick_id = 999;
    src->bus()->publish("reality.proprio.state", p_src);

    auto last_clone = std::dynamic_pointer_cast<const ogma::ProprioToken>(
        clone->bus()->last_value("reality.proprio.state"));
    ASSERT_TRUE(last_clone != nullptr);
    EXPECT_NE(last_clone->values(0), 1.0f)
        << "Publishing on source bus should not affect clone bus.";
}
