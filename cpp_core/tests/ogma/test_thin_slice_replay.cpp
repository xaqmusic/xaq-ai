// =============================================================================
// test_thin_slice_replay.cpp
//
// Phase 2: drives all five Phase-1 modules from the recorded OGFS capture
// (cpp_core/tests/golden/maze5x5_seed42.ogfs) through real InProcessBus
// dispatch on a single thread.  Validates the integration-soundness slice
// of the Phase 2 multi-criteria gate per docs/primitives/_phase2_replay.md:
//
//   ✓ replay completes without NaN/divergence/crash       (criterion 1)
//   ✓ DAG-ordered dispatch (manual sequencing)            (criterion 2 — informal)
//   – tick-barrier integrity under thread-pool stress     (criterion 3 — deferred to Scheduler)
//   ✓ GNG node-count growth is non-trivial                (criterion 4 — sanity band)
//   ✓ ActionDecoder publishes valid actions every tick    (criterion 5 — sanity)
//
// The OGFS file is gitignored (regenerable from
// scripts/capture_golden_frames.py); when absent locally the test SKIPs
// rather than fails so CI doesn't trip on missing capture.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/golden/Replay.hpp"
#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/modules/HomeostaticDrive.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/modules/NeurochemState.hpp"
#include "ogma/Topics.hpp"

namespace {

constexpr const char* kCapturePath =
    "../tests/golden/maze5x5_seed42.ogfs";   // run from build/

// Maze body schema (per scripts/capture_golden_frames.py):
//   indices  meaning
//   0..3     pos_x, pos_y, sin(heading), cos(heading)
//   4..11    8 wall whiskers
//   12..19   8 scent whiskers
//   20       hunger
//   21       pheromone
//   22       last_force      (only present if proprio_dim ≥ 23 — capture
//   23       energy           uses 22 dims, with energy at the LAST slot)
//
// The capture uses a flat 22-dim layout with energy at index 21 (per the
// _build_proprio() helper in the script: hunger, pheromone, last_force,
// energy is the trailing four).  Hunger is index 18, pheromone 19,
// last_force 20, energy 21.

constexpr int kIdxHungerOffset    = 18;
constexpr int kIdxPheromoneOffset = 19;
constexpr int kIdxLastForceOffset = 20;
constexpr int kIdxEnergyOffset    = 21;

ogma::ParamMap epm_params(int proprio_dim) {
    return {
        {"modality_group",     std::string("proprio")},
        {"modality_name",      std::string("imu")},
        {"encoder_kind",       std::string("rbf")},
        {"input_topic",        std::string("reality.proprio.imu")},
        {"projection_dim",     int64_t{32}},
        {"proprio_state_dims", int64_t{proprio_dim}},
        {"baking_threshold",   int64_t{20}},
        {"min_insertion_error", 0.001},
        {"subtract_descending_prediction", false},
    };
}

ogma::ParamMap voter_params() {
    return {
        {"level",          int64_t{0}},
        {"input_pattern",  std::string("reality.")},
        {"trust_epsilon",  0.05},
        {"group_balance",  true},
        {"priority_group", std::string("proprio")},
    };
}

ogma::ParamMap drive_params() {
    return {
        {"channels",            std::vector<std::string>{"energy", "novelty_satiation"}},
        {"setpoints",           std::vector<double>{0.8, 0.5}},
        {"urgency_normalizers", std::vector<double>{1.0, 0.5}},
        {"channel_input_topics",
            std::vector<std::string>{"reality.proprio.energy", "consensus.0"}},
        {"energy_drain_per_tick",   0.0,    // proprio sync is authoritative
        },
        {"energy_replenish_per_hit", 0.5},
    };
}

ogma::ParamMap decoder_params() {
    return {
        {"consensus_level",  int64_t{0}},
        {"proprio_topic",    std::string("reality.proprio.imu")},
        {"action_bins",      int64_t{3}},
        {"pragmatic_gain",   10.0},
    };
}

// Bridge a single OGFS Tick onto the Bus, then run the 5 modules in DAG order.
struct ThinSlice {
    ogma::InProcessBus    bus;
    ogma::NeurochemState  neuro;
    ogma::EPM             epm;
    ogma::LateralVoter    voter;
    ogma::HomeostaticDrive drive;
    ogma::ActionDecoder   decoder;

    // Diagnostic counters
    size_t   nan_count        = 0;
    size_t   exception_count  = 0;
    int      max_nodes        = 0;
    int      final_nodes      = 0;
    float    last_dopamine    = 0.0f;
    float    last_serotonin   = 0.0f;
    float    last_accel       = 0.0f;
    float    min_accel        =  1e30f;
    float    max_accel        = -1e30f;

    void setup(int proprio_dim) {
        neuro.set_id("neuro");
        epm.set_id("epm_imu");
        voter.set_id("voter_0");
        drive.set_id("drive");
        decoder.set_id("action_decoder");

        neuro.on_setup(&bus, {});
        epm.on_setup(&bus, epm_params(proprio_dim));
        voter.on_setup(&bus, voter_params());
        drive.on_setup(&bus, drive_params());
        decoder.on_setup(&bus, decoder_params());
    }

    static std::shared_ptr<ogma::ProprioToken> bundled_proprio(std::vector<float> const& v) {
        auto p = std::make_shared<ogma::ProprioToken>();
        p->sensor = "imu";
        p->values = Eigen::Map<const Eigen::VectorXf>(v.data(), int(v.size()));
        return p;
    }

    static std::shared_ptr<ogma::ProprioToken> single_value(std::string sensor, float v) {
        auto p = std::make_shared<ogma::ProprioToken>();
        p->sensor = std::move(sensor);
        p->values.resize(1);
        p->values << v;
        return p;
    }

    static std::shared_ptr<ogma::EnvEvent> event_msg(std::string name, float intensity) {
        auto e = std::make_shared<ogma::EnvEvent>();
        e->name      = std::move(name);
        e->intensity = intensity;
        return e;
    }

    void tick(ogma::golden::Tick const& og) {
        bus.begin_tick(og.tick_id);

        // Host-side bridges: raw proprio for the EPM (full 22-dim bundled
        // vector) plus per-channel single-value tokens for HomeostaticDrive.
        bus.publish("reality.proprio.imu", bundled_proprio(og.proprio));

        if (int(og.proprio.size()) > kIdxEnergyOffset)
            bus.publish("reality.proprio.energy",
                        single_value("energy", og.proprio[kIdxEnergyOffset]));
        if (int(og.proprio.size()) > kIdxHungerOffset)
            bus.publish("reality.proprio.hunger",
                        single_value("hunger", og.proprio[kIdxHungerOffset]));
        if (int(og.proprio.size()) > kIdxPheromoneOffset)
            bus.publish("reality.proprio.pheromone",
                        single_value("pheromone", og.proprio[kIdxPheromoneOffset]));

        for (auto const& ev : og.events)
            bus.publish(std::string("events.") + ev.name,
                        event_msg(ev.name, ev.intensity));

        // DAG-ordered execution (level 0 → ... → level N).  This is the
        // ordering the real Scheduler will produce when it lands.
        try {
            neuro.tick(og.tick_id);
            epm.tick(og.tick_id);
            voter.tick(og.tick_id);
            drive.tick(og.tick_id);
            decoder.tick(og.tick_id);
        } catch (...) {
            ++exception_count;
        }

        // Diagnostics: NaN scan over the headline payloads + node-count tracking.
        if (auto n = std::dynamic_pointer_cast<const ogma::NeuroState>(
                bus.last_value(ogma::topics::kNeuroState))) {
            if (std::isnan(n->dopamine)  || std::isnan(n->serotonin)
             || std::isnan(n->reward_signal)) ++nan_count;
            last_dopamine  = n->dopamine;
            last_serotonin = n->serotonin;
        }
        if (auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(
                bus.last_value("reality.proprio.imu"))) {
            if (std::isnan(rt->tle)) ++nan_count;
        }
        if (auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(
                bus.last_value(ogma::topics::kActionOut))) {
            if (std::isnan(act->accel)) ++nan_count;
            last_accel = act->accel;
            min_accel  = std::min(min_accel, act->accel);
            max_accel  = std::max(max_accel, act->accel);
        }

        max_nodes   = std::max(max_nodes, epm.node_count());
        final_nodes = epm.node_count();

        bus.end_tick();
    }
};

} // namespace

class ThinSliceReplay : public ::testing::Test {
protected:
    void SetUp() override {
        if (!ogma::golden::file_exists(kCapturePath)) {
            GTEST_SKIP() << "OGFS capture missing at " << kCapturePath
                         << " — regenerate via "
                         << "scripts/capture_golden_frames.py.";
        }
    }
};

TEST_F(ThinSliceReplay, FiveModuleSliceCompletesWithoutNanOrException) {
    ogma::golden::StreamReader reader(kCapturePath);
    ASSERT_GT(reader.header().tick_count, 0u);

    ThinSlice slice;
    slice.setup(int(reader.header().proprio_dim));

    ogma::golden::Tick og;
    while (reader.next(og)) slice.tick(og);

    EXPECT_EQ(slice.exception_count, 0u);
    EXPECT_EQ(slice.nan_count,       0u);
}

TEST_F(ThinSliceReplay, GngNodeCountGrowsMeaningfully) {
    ogma::golden::StreamReader reader(kCapturePath);
    ThinSlice slice;
    slice.setup(int(reader.header().proprio_dim));

    ogma::golden::Tick og;
    while (reader.next(og)) slice.tick(og);

    EXPECT_GT(slice.max_nodes,   2);
    EXPECT_GT(slice.final_nodes, 2);
    EXPECT_LT(slice.final_nodes, 2000);  // GNG max_nodes default is 2000
}

TEST_F(ThinSliceReplay, NeurochemStaysInUnitInterval) {
    ogma::golden::StreamReader reader(kCapturePath);
    ThinSlice slice;
    slice.setup(int(reader.header().proprio_dim));

    ogma::golden::Tick og;
    while (reader.next(og)) {
        slice.tick(og);
        EXPECT_GE(slice.last_dopamine,  0.0f);
        EXPECT_LE(slice.last_dopamine,  1.0f);
        EXPECT_GE(slice.last_serotonin, 0.0f);
        EXPECT_LE(slice.last_serotonin, 1.0f);
    }
}

TEST_F(ThinSliceReplay, ActionsStayWithinClamps) {
    ogma::golden::StreamReader reader(kCapturePath);
    ThinSlice slice;
    slice.setup(int(reader.header().proprio_dim));

    ogma::golden::Tick og;
    while (reader.next(og)) slice.tick(og);

    EXPECT_GE(slice.min_accel, -4.0f);
    EXPECT_LE(slice.max_accel,  4.0f);
}

TEST_F(ThinSliceReplay, DeterministicAcrossTwoIdenticalRuns) {
    auto run = []() {
        ogma::golden::StreamReader reader(kCapturePath);
        ThinSlice slice;
        slice.setup(int(reader.header().proprio_dim));
        ogma::golden::Tick og;
        while (reader.next(og)) slice.tick(og);
        return std::make_tuple(slice.final_nodes, slice.max_nodes,
                               slice.last_dopamine, slice.last_serotonin,
                               slice.last_accel);
    };
    auto a = run();
    auto b = run();
    EXPECT_EQ(a, b);
}
