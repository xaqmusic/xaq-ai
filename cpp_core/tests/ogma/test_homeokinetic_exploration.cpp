// =============================================================================
// test_homeokinetic_exploration.cpp  --  Unit tests for HomeokineticExploration
// =============================================================================
//
// The gate is fully self-calibrating: it tracks running mean+std of urgency
// and of the short/long change-ratio, then fires when current values are
// statistically anomalous (≥ mean+σ for urgency, ≤ mean−σ for the ratio).
// Tests therefore exercise the dynamics — sample-count warmup, statistical
// anomaly detection, and the failure modes (chunk gate, in-flight chunk).

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/HomeokineticExploration.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap small_params() {
    // Test-sized params.  Small windows so tests run in O(100) ticks.
    return {
        {"window_ticks",       int64_t{10}},
        {"long_window_ticks",  int64_t{40}},
        {"change_ema_alpha",   0.1},
        {"anomaly_factor",     0.5},
        {"chunk_match_eps",    0.01},
        {"episode_ticks",      int64_t{5}},
        {"cooldown_ticks",     int64_t{3}},
        {"accel_jitter",       2.0},
        {"master_seed",        int64_t{42}},
    };
}

std::shared_ptr<ogma::DriveErrors> make_drive(float urgency) {
    auto d = std::make_shared<ogma::DriveErrors>();
    d->urgency = urgency;
    return d;
}

std::shared_ptr<ogma::MotorChunks> make_chunks_one(float outcome_drive_delta) {
    auto c = std::make_shared<ogma::MotorChunks>();
    ogma::MotorChunk ch;
    ch.id                  = 1;
    ch.outcome_drive_delta = outcome_drive_delta;
    c->chunks.push_back(std::move(ch));
    return c;
}

std::shared_ptr<ogma::ActionOut> make_action(int chunk_remaining_ticks) {
    auto a = std::make_shared<ogma::ActionOut>();
    a->chunk_remaining_ticks = chunk_remaining_ticks;
    return a;
}

struct KinesisFixture {
    ogma::InProcessBus               bus;
    ogma::HomeokineticExploration    kinesis;
    int                              next_tick = 0;

    explicit KinesisFixture(ogma::ParamMap params = small_params()) {
        kinesis.set_id("kinesis");
        kinesis.on_setup(&bus, params);
    }

    std::shared_ptr<const ogma::ExplorationDirective> last_directive() const {
        return std::dynamic_pointer_cast<const ogma::ExplorationDirective>(
            bus.last_value(ogma::topics::kExplorationDirective));
    }

    void feed(float urgency, ogma::MessagePtr extra = nullptr,
              std::string_view extra_topic = "") {
        bus.begin_tick(uint64_t(next_tick));
        bus.publish(ogma::topics::kDriveErrors, make_drive(urgency));
        if (extra) bus.publish(extra_topic, extra);
        kinesis.tick(uint64_t(next_tick));
        bus.end_tick();
        ++next_tick;
    }

    // Build statistics: zigzag urgency with deterministic per-step
    // amplitude variation so the change-ratio buffer ends up with a SPREAD
    // of values (not all identical).  Real cell-env urgency varies smoothly
    // and produces this spread naturally; tests use a simple lcg-derived
    // jitter to mimic it without an RNG dependency.
    void zigzag(int n, float center, float amp) {
        for (int i = 0; i < n; ++i) {
            uint32_t lcg = uint32_t(0x9E3779B1u * uint32_t(next_tick + 1));
            float    j   = (float(lcg & 0xFFFFu) / 65535.0f - 0.5f) * 0.6f;  // [-0.3, +0.3]
            feed(center + amp * ((i % 2) ? 1.0f : -1.0f) * (1.0f + j));
        }
    }

    // Hold urgency flat — produces zero change-ratio.
    void flat_high(int n, float u) {
        for (int i = 0; i < n; ++i) feed(u);
    }
};

} // namespace

// -- Construction ----------------------------------------------------------

TEST(HomeokineticExploration, ConstructsAndDeclaresContract) {
    KinesisFixture f;
    EXPECT_EQ(f.kinesis.type_name(), "HomeokineticExploration");
    ASSERT_EQ(f.kinesis.output_topics().size(), 1u);
    EXPECT_EQ(f.kinesis.output_topics()[0].name, "exploration.directive");
}

// -- Cold-start guards ----------------------------------------------------

TEST(HomeokineticExploration, ColdStartSuppressesArmUntilWindowFilled) {
    KinesisFixture f;   // window_ticks = 10
    for (int t = 0; t < 9; ++t) {
        f.feed(0.6f + 0.05f * float(t % 2));
        ASSERT_NE(f.last_directive(), nullptr);
        EXPECT_FALSE(f.last_directive()->active) << "tick " << t;
    }
}

TEST(HomeokineticExploration, ColdStartSuppressesArmUntilLongBufferFilled) {
    // Long buffer = 40 ticks.  Until it fills, gate must stay cold.
    KinesisFixture f;
    f.zigzag(/*n=*/30, /*center=*/0.5f, /*amp=*/0.05f);
    EXPECT_FALSE(f.last_directive()->active);
    EXPECT_LT(f.kinesis.urgency_buffer_fill(), 40);
}

// -- Statistically-anomalous arming (the wall-stuck case) ------------------

TEST(HomeokineticExploration, ArmsWhenUrgencyAndRatioBothAnomalous) {
    // Phase 1: build statistics with zigzag at moderate urgency (mean 0.6).
    // Phase 2: hold urgency at 0.95 (anomalously high vs mean+σ) AND flat
    // (ratio anomalously low vs ratio mean−σ).  Gate should fire.
    KinesisFixture f;
    f.zigzag(/*n=*/40, /*center=*/0.6f, /*amp=*/0.05f);
    EXPECT_FALSE(f.last_directive()->active) << "phase 1 should not arm";

    f.flat_high(/*n=*/15, /*u=*/0.95f);
    EXPECT_GT(f.kinesis.episodes_armed(), 0u);
}

TEST(HomeokineticExploration, ArmsWhenSaturatedFlatAfterForaging) {
    // Wall-stuck scenario: foraging with hits (urgency oscillates with
    // sharp dips), then body wedges and urgency saturates flat near 1.0.
    // Ceiling is irrelevant — gate looks at min(window) vs urgency_mean+σ
    // and current ratio vs ratio_mean−σ, both of which fire even at the
    // structural ceiling.
    KinesisFixture f;
    f.zigzag(20, 0.7f, 0.10f);   // foraging — mean ~0.7, var ~0.01
    f.feed(0.20f);               // sharp dip — simulated hit
    f.zigzag(15, 0.7f, 0.10f);
    EXPECT_FALSE(f.last_directive()->active);

    // Body wedges — urgency saturates flat at 0.997.
    f.flat_high(/*n=*/15, 0.997f);
    EXPECT_GT(f.kinesis.episodes_armed(), 0u);
}

TEST(HomeokineticExploration, EpisodeRunsForEpisodeTicksThenYields) {
    KinesisFixture f;
    f.zigzag(40, 0.6f, 0.05f);  // build stats
    int  saw_active = 0;
    int  saw_inactive_post = 0;
    bool seen_active = false;
    // Run plenty of flat ticks to allow at least one full episode.  Episode
    // length is adaptive (effective_episode_ticks ≥ episode_ticks); 30
    // ticks is enough headroom for the first full episode + cooldown.
    for (int i = 0; i < 30; ++i) {
        f.feed(0.95f);
        auto d = f.last_directive();
        ASSERT_NE(d, nullptr);
        if (d->active) {
            ++saw_active;
            seen_active = true;
        } else if (seen_active) {
            ++saw_inactive_post;
        }
    }
    EXPECT_GE(saw_active, 5);          // at least the baseline episode_ticks
    EXPECT_GE(saw_inactive_post, 1);
    EXPECT_GE(f.kinesis.episodes_armed(), 1u);
}

// -- Failing gating conditions --------------------------------------------

TEST(HomeokineticExploration, ChunkGateSuppressesArm) {
    KinesisFixture f;
    f.zigzag(40, 0.6f, 0.05f);
    for (int i = 0; i < 15; ++i) {
        f.feed(0.95f, make_chunks_one(0.5f), ogma::topics::kMotorChunks);
    }
    EXPECT_FALSE(f.last_directive()->active);
    EXPECT_TRUE(f.kinesis.gate_chunk_block());
}

TEST(HomeokineticExploration, ActiveChunkPlaybackSuppressesArm) {
    KinesisFixture f;
    f.zigzag(40, 0.6f, 0.05f);
    for (int i = 0; i < 15; ++i) {
        f.feed(0.95f, make_action(/*chunk_remaining_ticks=*/3),
                      ogma::topics::kActionOut);
    }
    EXPECT_FALSE(f.last_directive()->active);
}

TEST(HomeokineticExploration, OngoingHighChangeDoesNotArm) {
    // Sustained zigzag — short_mean_change ≈ long_ema, ratio ≈ 1, well
    // above ratio_mean − σ.  Body is not anomalously still.
    KinesisFixture f;
    f.zigzag(80, 0.6f, 0.05f);
    EXPECT_FALSE(f.last_directive()->active);
}

TEST(HomeokineticExploration, AverageUrgencyDoesNotArm) {
    // Phase 1 builds stats with mean 0.6.  Phase 2 holds urgency *at* the
    // mean (0.6) — flat but not anomalously high.  Gate must not fire.
    KinesisFixture f;
    f.zigzag(40, 0.6f, 0.05f);
    f.flat_high(15, 0.6f);
    EXPECT_FALSE(f.last_directive()->active);
}

// -- Cooldown ---------------------------------------------------------------

TEST(HomeokineticExploration, CooldownEnforcedBetweenEpisodes) {
    KinesisFixture f;
    f.zigzag(40, 0.6f, 0.05f);
    int episodes_seen = 0;
    uint64_t last_id = 0;
    for (int i = 0; i < 50; ++i) {
        f.feed(0.95f);
        auto d = f.last_directive();
        if (d->active && d->episode_id != last_id) {
            ++episodes_seen;
            last_id = d->episode_id;
        }
    }
    EXPECT_GE(episodes_seen, 2);
    EXPECT_GE(f.kinesis.episodes_armed(), 2u);
}

// -- Outcome-feedback adaptation -------------------------------------------

TEST(HomeokineticExploration, EpisodeParamsEscalateWhenEpisodesFail) {
    // Force consecutive failed episodes (urgency stays at the post-arm
    // value).  effective_episode_ticks and effective_accel_jitter should
    // grow above their baseline values as success_rate_ema falls.
    KinesisFixture f;
    f.zigzag(60, 0.6f, 0.05f);   // build baseline + history
    int baseline_ticks = f.kinesis.effective_episode_ticks();
    float baseline_jit = f.kinesis.effective_accel_jitter();

    // Now run many flat-suffering ticks to fire repeated episodes,
    // each of which "fails" (urgency stays high throughout).
    for (int i = 0; i < 200; ++i) f.feed(0.95f);

    EXPECT_GT(f.kinesis.episodes_armed(), 1u);
    // Success rate should fall below 0.5 (initial), causing scale > 1.
    EXPECT_LT(f.kinesis.success_rate(), 0.5f);
    EXPECT_GE(f.kinesis.effective_episode_ticks(), baseline_ticks);
    EXPECT_GE(f.kinesis.effective_accel_jitter(), baseline_jit);
}

// -- Determinism ------------------------------------------------------------

TEST(HomeokineticExploration, IdenticalSeedsProduceIdenticalSamples) {
    auto run = []() {
        KinesisFixture f;
        f.zigzag(40, 0.6f, 0.05f);
        f.flat_high(15, 0.95f);
        return f.last_directive()->accel;
    };
    EXPECT_FLOAT_EQ(run(), run());
}

TEST(HomeokineticExploration, DifferentSeedsProduceDifferentSamples) {
    auto p_a = small_params();
    auto p_b = small_params();
    p_a["master_seed"] = int64_t{1};
    p_b["master_seed"] = int64_t{2};
    KinesisFixture fa(p_a);
    KinesisFixture fb(p_b);
    fa.zigzag(40, 0.6f, 0.05f);  fa.flat_high(15, 0.95f);
    fb.zigzag(40, 0.6f, 0.05f);  fb.flat_high(15, 0.95f);
    EXPECT_NE(fa.last_directive()->accel, fb.last_directive()->accel);
}
