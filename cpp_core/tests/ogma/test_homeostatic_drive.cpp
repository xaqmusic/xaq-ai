// =============================================================================
// test_homeostatic_drive.cpp  --  Unit tests for HomeostaticDrive
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/HomeostaticDrive.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap default_three_channels() {
    return {
        {"channels",            std::vector<std::string>{"energy", "integrity", "novelty_satiation"}},
        {"setpoints",           std::vector<double>{0.8, 1.0, 0.5}},
        {"urgency_normalizers", std::vector<double>{1.0, 1.0, 0.5}},
        {"channel_input_topics",
            std::vector<std::string>{
                "reality.proprio.energy",
                "reality.proprio.integrity",
                "consensus.0",
            }},
        {"energy_drain_per_tick", 0.01},   // bigger than default for fast tests
        {"energy_replenish_per_hit", 0.5},
        {"integrity_drain_per_miss", 0.10},
        {"novelty_satiation_alpha", 0.20},
    };
}

std::shared_ptr<ogma::ProprioToken> make_proprio(std::string sensor, float v) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->sensor = std::move(sensor);
    p->values.resize(1);
    p->values << v;
    return p;
}

std::shared_ptr<ogma::EnvEvent> make_event(std::string name, float intensity = 1.0f) {
    auto e = std::make_shared<ogma::EnvEvent>();
    e->name      = std::move(name);
    e->intensity = intensity;
    return e;
}

std::shared_ptr<ogma::ConsensusToken> make_consensus(float fused_tle) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_tle = fused_tle;
    return c;
}

struct DriveFixture {
    ogma::InProcessBus      bus;
    ogma::HomeostaticDrive  drive;

    explicit DriveFixture(ogma::ParamMap params = default_three_channels()) {
        drive.set_id("drive");
        drive.on_setup(&bus, params);
    }

    std::shared_ptr<const ogma::DriveErrors> last_drive() const {
        return std::dynamic_pointer_cast<const ogma::DriveErrors>(
            bus.last_value(ogma::topics::kDriveErrors));
    }
};

} // namespace

// -- Construction / contract -----------------------------------------------

TEST(HomeostaticDrive, ConstructsWithDefaultThreeChannels) {
    DriveFixture f;
    EXPECT_EQ(f.drive.type_name(), "HomeostaticDrive");
    EXPECT_EQ(f.drive.channel_count(), 3);
    EXPECT_EQ(f.drive.output_topics()[0].name, "drive.errors");
}

TEST(HomeostaticDrive, RejectsMismatchedArrayLengths) {
    auto p = default_three_channels();
    p["setpoints"] = std::vector<double>{0.8, 1.0};   // 2 vs 3 channels
    EXPECT_THROW({
        DriveFixture f(p);
    }, std::invalid_argument);
}

// -- Setpoint / error semantics --------------------------------------------

TEST(HomeostaticDrive, AtSetpointFirstTickHasZeroErrorAndUrgency) {
    DriveFixture f;
    f.bus.begin_tick(0);
    f.drive.tick(0);
    f.bus.end_tick();

    auto d = f.last_drive();
    ASSERT_NE(d, nullptr);
    // Energy: setpoint=0.8, drain=0.01 → after one tick error = -0.01.
    EXPECT_NEAR(d->errors.at("energy"), -0.01f, 1e-5f);
    // Integrity: no events.miss → still at setpoint.
    EXPECT_NEAR(d->errors.at("integrity"), 0.0f, 1e-5f);
    // Novelty: setpoint=0.5, no consensus delivery → current still at setpoint.
    EXPECT_NEAR(d->errors.at("novelty_satiation"), 0.0f, 1e-5f);
}

TEST(HomeostaticDrive, EnergyDecaysEveryTickWithoutHit) {
    DriveFixture f;
    for (uint64_t t = 0; t < 10; ++t) {
        f.bus.begin_tick(t);
        f.drive.tick(t);
        f.bus.end_tick();
    }
    // 10 ticks * 0.01 drain = 0.10 deficit.
    auto d = f.last_drive();
    EXPECT_NEAR(d->errors.at("energy"), -0.10f, 1e-4f);
    EXPECT_NEAR(f.drive.current_value("energy"), 0.70f, 1e-4f);
}

TEST(HomeostaticDrive, EventsHitReducesEnergyDeficit) {
    DriveFixture f;
    // Drain for 50 ticks → deficit = 0.50, current = 0.30.
    for (uint64_t t = 0; t < 50; ++t) {
        f.bus.begin_tick(t);
        f.drive.tick(t);
        f.bus.end_tick();
    }
    EXPECT_NEAR(f.drive.current_value("energy"), 0.30f, 1e-4f);

    // Single hit with replenish=0.5 → reduces deficit by 50% of (0.8 - 0.30)
    //   first one tick of drain: 0.30 - 0.01 = 0.29
    //   then hit: deficit = 0.51, replenish = 0.5*0.51 = 0.255 → current = 0.545
    f.bus.begin_tick(50);
    f.bus.publish("events.hit", make_event("hit"));
    f.drive.tick(50);
    f.bus.end_tick();
    EXPECT_GT(f.drive.current_value("energy"), 0.4f);
    EXPECT_LT(f.drive.current_value("energy"), 0.8f);
}

TEST(HomeostaticDrive, EventsMissDrainsIntegrity) {
    DriveFixture f;
    f.bus.begin_tick(0);
    f.bus.publish("events.miss", make_event("miss"));
    f.drive.tick(0);
    f.bus.end_tick();

    auto d = f.last_drive();
    EXPECT_NEAR(d->errors.at("integrity"), -0.10f, 1e-5f);   // drain=0.10
}

// -- Proprio sync (energy and integrity) ----------------------------------

TEST(HomeostaticDrive, ProprioSnapsCurrentToBodyValue) {
    DriveFixture f;
    f.bus.begin_tick(0);
    // Body reports energy=0.50.  Drive should snap to it (after also draining
    // by 0.01 for the tick).
    f.bus.publish("reality.proprio.energy", make_proprio("energy", 0.50f));
    f.drive.tick(0);
    f.bus.end_tick();
    EXPECT_NEAR(f.drive.current_value("energy"), 0.50f - 0.01f, 1e-5f);
}

// -- Novelty satiation EMA -------------------------------------------------

TEST(HomeostaticDrive, NoveltyEmaTracksConsensusFusedTle) {
    DriveFixture f;
    // 50 ticks of constant fused_tle = 1.0 with alpha = 0.20 → current
    // approaches 1.0.  Setpoint = 0.5, error grows positive.
    for (uint64_t t = 0; t < 50; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("consensus.0", make_consensus(1.0f));
        f.drive.tick(t);
        f.bus.end_tick();
    }
    EXPECT_GT(f.drive.current_value("novelty_satiation"), 0.9f);
    auto d = f.last_drive();
    EXPECT_GT(d->errors.at("novelty_satiation"), 0.4f);
}

// -- Urgency scalar --------------------------------------------------------

TEST(HomeostaticDrive, UrgencyIsZeroAtSetpoint) {
    DriveFixture f;
    // No drain: set energy_drain_per_tick=0 + integrity untouched + no consensus.
    f.drive.on_param_change("energy_drain_per_tick", ogma::ParamValue{0.0});
    f.bus.begin_tick(0);
    f.drive.tick(0);
    f.bus.end_tick();
    EXPECT_FLOAT_EQ(f.drive.urgency(), 0.0f);
}

TEST(HomeostaticDrive, UrgencyClampedToHi) {
    auto p = default_three_channels();
    p["urgency_clamp_hi"] = 0.5;
    DriveFixture f(p);

    // Drain energy heavily.
    for (uint64_t t = 0; t < 100; ++t) {
        f.bus.begin_tick(t);
        f.bus.publish("events.miss", make_event("miss"));
        f.drive.tick(t);
        f.bus.end_tick();
    }
    EXPECT_LE(f.drive.urgency(), 0.5f);
}

// -- First-tick semantics --------------------------------------------------

TEST(HomeostaticDrive, FirstTickWithNoInputsPublishesAllChannels) {
    DriveFixture f;
    f.bus.begin_tick(0);
    f.drive.tick(0);
    f.bus.end_tick();

    auto d = f.last_drive();
    ASSERT_NE(d, nullptr);
    EXPECT_EQ(d->errors.size(), 3u);
    EXPECT_TRUE(d->errors.count("energy"));
    EXPECT_TRUE(d->errors.count("integrity"));
    EXPECT_TRUE(d->errors.count("novelty_satiation"));
}

// -- Hot-mutation ----------------------------------------------------------

TEST(HomeostaticDrive, HotMutateSetpoints) {
    DriveFixture f;
    EXPECT_NO_THROW(f.drive.on_param_change(
        "setpoints", ogma::ParamValue{std::vector<double>{0.5, 0.5, 0.5}}));
}

TEST(HomeostaticDrive, ConstructionOnlyChannelsThrows) {
    DriveFixture f;
    EXPECT_THROW(f.drive.on_param_change(
        "channels", ogma::ParamValue{std::vector<std::string>{"x"}}),
        std::invalid_argument);
}

TEST(HomeostaticDrive, UnknownParamThrows) {
    DriveFixture f;
    EXPECT_THROW(f.drive.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}

// -- Determinism ----------------------------------------------------------

TEST(HomeostaticDrive, DeterministicAcrossTwoIdenticalRuns) {
    auto run = []() {
        DriveFixture f;
        for (uint64_t t = 0; t < 100; ++t) {
            f.bus.begin_tick(t);
            if (t % 7 == 0) f.bus.publish("events.hit", make_event("hit"));
            if (t % 11 == 0) f.bus.publish("events.miss", make_event("miss"));
            f.bus.publish("consensus.0", make_consensus(0.3f + 0.05f * (t % 5)));
            f.drive.tick(t);
            f.bus.end_tick();
        }
        return std::make_tuple(f.drive.current_value("energy"),
                               f.drive.current_value("integrity"),
                               f.drive.current_value("novelty_satiation"),
                               f.drive.urgency());
    };
    auto a = run();
    auto b = run();
    EXPECT_EQ(a, b);
}
