// =============================================================================
// test_snapshot_disk_round_trip.cpp  --  W3.6 disk smoke test
// =============================================================================
//
// The clone() determinism contract (test_clone, test_clone_shipping_configs)
// proves snapshot_state / restore_state are byte-equivalent in-memory.  This
// test extends that to the on-disk path the Save/Load UI uses:
//
//   instance.snapshot_state() → json.dump() → write file → read file
//                            → json.parse() → restore_state on fresh instance
//
// If float→string→float round-trip loses bits, divergence shows up as action
// drift over the 100-tick compare phase; we catch it with ASSERT_DOUBLE_EQ
// at every tick.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

namespace {

namespace fs = std::filesystem;

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

void publish_synthetic(ogma::Bus* bus, uint64_t tick) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id = tick;
    p->sensor  = "state";
    p->values.resize(2);
    p->values(0) = float(std::sin(double(tick) * 0.07));
    p->values(1) = float(std::cos(double(tick) * 0.13));
    bus->publish("reality.proprio.state", p);
    if (tick % 30 == 0) {
        auto ev = std::make_shared<ogma::EnvEvent>();
        ev->tick_id = tick;
        ev->name    = "alive";
        ev->intensity = 1.0f;
        bus->publish("events.alive", ev);
    }
}

double action_digest(ogma::Bus const* bus) {
    auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus->last_value(ogma::topics::kActionOut));
    return a ? double(a->accel) : 0.0;
}

double dopamine_digest(ogma::Bus const* bus) {
    auto n = std::dynamic_pointer_cast<const ogma::NeuroState>(
        bus->last_value(ogma::topics::kNeuroState));
    return n ? double(n->dopamine) : 0.0;
}

}  // namespace

TEST(SnapshotDiskRoundTrip, ByteEquivalentAcrossWriteAndRead) {
    auto src = std::make_unique<ogma::OgmaInstance>(
        minimal_mc_like_graph(), std::make_unique<ogma::InProcessBus>());
    for (uint64_t t = 0; t < 50; ++t) {
        publish_synthetic(src->bus(), t);
        src->tick();
    }

    // 1) snapshot → string → temp file
    nlohmann::json snap = src->snapshot_state();
    std::string blob = snap.dump();

    auto tmp = fs::temp_directory_path() /
               ("ogma_disk_smoke_" + std::to_string(::getpid()) + ".json");
    {
        std::ofstream out(tmp, std::ios::binary);
        ASSERT_TRUE(out.good()) << "failed to open " << tmp;
        out << blob;
    }

    // 2) read file → parse → restore into a fresh instance
    std::string read_back;
    {
        std::ifstream in(tmp, std::ios::binary);
        ASSERT_TRUE(in.good()) << "failed to read back " << tmp;
        std::stringstream ss;
        ss << in.rdbuf();
        read_back = ss.str();
    }
    fs::remove(tmp);
    nlohmann::json reparsed = nlohmann::json::parse(read_back);

    auto restored = std::make_unique<ogma::OgmaInstance>(
        minimal_mc_like_graph(), std::make_unique<ogma::InProcessBus>());
    restored->restore_state(reparsed);

    // 3) tick both 100 more times with identical inputs; assert byte-equiv
    for (uint64_t t = 50; t < 150; ++t) {
        publish_synthetic(src->bus(),      t);
        publish_synthetic(restored->bus(), t);
        src->tick();
        restored->tick();
        ASSERT_DOUBLE_EQ(action_digest(src->bus()),
                         action_digest(restored->bus()))
            << "Action divergence at tick " << t
            << " — snapshot did not survive disk round-trip.";
        ASSERT_DOUBLE_EQ(dopamine_digest(src->bus()),
                         dopamine_digest(restored->bus()))
            << "Dopamine divergence at tick " << t;
    }
}
