// =============================================================================
// test_motor_repertoire.cpp  --  Unit tests for MotorRepertoire
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/MotorRepertoire.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap default_params() {
    return {
        {"max_chunks",                       int64_t{16}},
        {"chunk_max_ticks",                  int64_t{8}},
        {"crystallization_min_observations", int64_t{3}},
        {"crystallization_min_drive_delta",  0.001},
        {"crystallization_drive_window_ticks", int64_t{5}},
    };
}

std::shared_ptr<ogma::ActionOut> make_action(float a) {
    auto m = std::make_shared<ogma::ActionOut>();
    m->accel = a;
    return m;
}

std::shared_ptr<ogma::SequenceMotif> make_motif(int motif_id, bool just_baked = false) {
    auto m = std::make_shared<ogma::SequenceMotif>();
    m->motif_id    = motif_id;
    m->just_baked  = just_baked;
    // MotorRepertoire requires is_baked=true to accumulate observations.
    // Tests verifying chunk-crystallization behavior need motifs that
    // pass that filter; if a test wants to verify the filter itself it
    // can override after construction.
    m->is_baked    = true;
    m->phase       = 0;
    m->motif_length = 5;
    return m;
}

std::shared_ptr<ogma::DriveErrors> make_drive(float urgency) {
    auto d = std::make_shared<ogma::DriveErrors>();
    d->urgency = urgency;
    return d;
}

std::shared_ptr<ogma::MotorPlayCmd> make_cmd(int chunk_id, uint64_t request_id = 1) {
    auto c = std::make_shared<ogma::MotorPlayCmd>();
    c->chunk_id   = chunk_id;
    c->request_id = request_id;
    return c;
}

struct RepertoireFixture {
    ogma::InProcessBus      bus;
    ogma::MotorRepertoire   repertoire;
    std::vector<std::shared_ptr<const ogma::MotorPlayStream>> streams;

    explicit RepertoireFixture(ogma::ParamMap params = default_params()) {
        repertoire.set_id("motor_repertoire");
        repertoire.on_setup(&bus, params);
        bus.subscribe(ogma::topics::kMotorPlayResp,
                      ogma::SubscriptionKind::Direct,
            [this](std::string_view, ogma::MessagePtr p) {
                streams.push_back(std::dynamic_pointer_cast<const ogma::MotorPlayStream>(p));
            });
    }

    std::shared_ptr<const ogma::MotorChunks> last_library() const {
        return std::dynamic_pointer_cast<const ogma::MotorChunks>(
            bus.last_value(ogma::topics::kMotorChunks));
    }

    // Publish a sequence of (action, motif_id, urgency) for one tick each.
    // Sets latest_drive then publishes action and motif.  Optionally fires
    // an events.hit before the motif is published so the post-Phase-6.1
    // hits_during gate can clear in tests that expect crystallization.
    void run_tick(uint64_t t, float action, int motif_id, float urgency,
                  bool fire_hit = false) {
        bus.begin_tick(t);
        bus.publish(ogma::topics::kDriveErrors, make_drive(urgency));
        bus.publish(ogma::topics::kActionOut,   make_action(action));
        bus.publish("sequence.motif.action.out", make_motif(motif_id));
        if (fire_hit) {
            auto e = std::make_shared<ogma::EnvEvent>();
            e->name      = "hit";
            e->intensity = 1.0f;
            bus.publish("events.hit", e);
        }
        repertoire.tick(t);
        bus.end_tick();
    }
};

} // namespace

// -- Construction / contract -----------------------------------------------

TEST(MotorRepertoire, ConstructsAndDeclaresContract) {
    RepertoireFixture f;
    EXPECT_EQ(f.repertoire.type_name(), "MotorRepertoire");
    EXPECT_EQ(f.repertoire.output_topics().size(), 2u);
}

TEST(MotorRepertoire, FirstTickPublishesEmptyLibrary) {
    RepertoireFixture f;
    f.bus.begin_tick(0);
    f.repertoire.tick(0);
    f.bus.end_tick();

    auto lib = f.last_library();
    ASSERT_NE(lib, nullptr);
    EXPECT_EQ(lib->chunks.size(), 0u);
}

// -- Crystallization ------------------------------------------------------

TEST(MotorRepertoire, CrystallizesMotifAfterRepeatObservations) {
    RepertoireFixture f;

    // Each tick fires a hit event under motif_id=42; after enough baked
    // observations + hits_during ≥ 1, the motif crystallises.
    float u = 1.0f;
    for (uint64_t t = 0; t < 10; ++t) {
        u -= 0.05f;
        f.run_tick(t, /*action=*/0.5f, /*motif_id=*/42, /*urgency=*/u,
                   /*fire_hit=*/true);
    }
    EXPECT_GE(f.repertoire.chunk_count(), 1u);
}

TEST(MotorRepertoire, NoCrystallizationWithoutHits) {
    RepertoireFixture f;
    // Plenty of observations but no events.hit → hits_during stays at 0
    // → no crystallization.  This is the post-Phase-6.1 outcome gate:
    // a stable motif alone isn't enough; it must also have been active
    // during a real reward event.
    float u = 1.0f;
    for (uint64_t t = 0; t < 20; ++t) {
        u -= 0.01f;  // urgency drops, but no hits fire
        f.run_tick(t, 0.5f, /*motif_id=*/77, /*urgency=*/u);
    }
    EXPECT_EQ(f.repertoire.chunk_count(), 0u);
}

TEST(MotorRepertoire, NoCrystallizationBelowMinObservations) {
    auto p = default_params();
    p["crystallization_min_observations"] = int64_t{20};
    RepertoireFixture f(p);

    float u = 1.0f;
    for (uint64_t t = 0; t < 5; ++t) {
        u -= 0.05f;
        f.run_tick(t, 0.5f, 33, u, /*fire_hit=*/true);
    }
    EXPECT_EQ(f.repertoire.chunk_count(), 0u);
}

TEST(MotorRepertoire, BootstrapMotifIdMinusOneIgnored) {
    RepertoireFixture f;
    float u = 1.0f;
    for (uint64_t t = 0; t < 10; ++t) {
        u -= 0.05f;
        f.run_tick(t, 0.5f, /*motif_id=*/-1, u);
    }
    EXPECT_EQ(f.repertoire.chunk_count(), 0u);
}

// -- Library snapshot publishing -----------------------------------------

TEST(MotorRepertoire, LibrarySnapshotPublishedOnCrystallization) {
    RepertoireFixture f;

    float u = 1.0f;
    for (uint64_t t = 0; t < 10; ++t) {
        u -= 0.05f;
        f.run_tick(t, /*action=*/float(t) * 0.1f, 99, u, /*fire_hit=*/true);
    }
    auto lib = f.last_library();
    ASSERT_NE(lib, nullptr);
    EXPECT_GE(lib->chunks.size(), 1u);

    // Check the chunk fields are populated.
    auto& c = lib->chunks.front();
    EXPECT_GT(c.id, 0);
    EXPECT_FALSE(c.action_sequence.empty());
    EXPECT_GT(c.outcome_drive_delta, 0.0f);
}

// -- Playback ------------------------------------------------------------

TEST(MotorRepertoire, PlayCmdReturnsChunkActions) {
    RepertoireFixture f;

    // Crystallize a chunk first.
    float u = 1.0f;
    for (uint64_t t = 0; t < 10; ++t) {
        u -= 0.05f;
        f.run_tick(t, /*action=*/float(t) * 0.1f, /*motif=*/42, u,
                   /*fire_hit=*/true);
    }
    auto lib = f.last_library();
    ASSERT_NE(lib, nullptr);
    ASSERT_FALSE(lib->chunks.empty());
    int chunk_id = lib->chunks.front().id;

    // Issue a play command.
    f.bus.begin_tick(20);
    f.bus.publish(ogma::topics::kMotorPlayCmd, make_cmd(chunk_id, /*req=*/77));
    f.repertoire.tick(20);
    f.bus.end_tick();

    ASSERT_FALSE(f.streams.empty());
    auto stream = f.streams.back();
    EXPECT_EQ(stream->request_id, 77u);
    EXPECT_EQ(stream->chunk_id,   chunk_id);
    EXPECT_FALSE(stream->actions.empty());
}

TEST(MotorRepertoire, UnknownChunkIdReturnsEmptyActions) {
    RepertoireFixture f;
    f.bus.begin_tick(0);
    f.bus.publish(ogma::topics::kMotorPlayCmd, make_cmd(/*chunk_id=*/999, /*req=*/5));
    f.repertoire.tick(0);
    f.bus.end_tick();

    ASSERT_FALSE(f.streams.empty());
    auto stream = f.streams.back();
    EXPECT_EQ(stream->request_id, 5u);
    EXPECT_TRUE(stream->actions.empty());
}

// -- Chunk-ID stability --------------------------------------------------

TEST(MotorRepertoire, ChunkIdsAreMonotonicallyAssigned) {
    auto p = default_params();
    p["crystallization_min_observations"] = int64_t{2};
    RepertoireFixture f(p);

    float u = 1.0f;
    for (uint64_t t = 0; t < 6; ++t) {
        u -= 0.05f;
        // Two distinct motifs over the course of 6 ticks.
        int mid = (t < 3) ? 1 : 2;
        f.run_tick(t, 0.5f, mid, u, /*fire_hit=*/true);
    }

    auto lib = f.last_library();
    ASSERT_GE(lib->chunks.size(), 1u);
    int max_id = 0;
    for (auto const& c : lib->chunks) max_id = std::max(max_id, c.id);
    EXPECT_GE(max_id, 1);
}

// -- Hot-mutation --------------------------------------------------------

TEST(MotorRepertoire, HotMutateThresholds) {
    RepertoireFixture f;
    EXPECT_NO_THROW(f.repertoire.on_param_change("crystallization_min_drive_delta",
                                                  ogma::ParamValue{0.10}));
    EXPECT_NO_THROW(f.repertoire.on_param_change("max_chunks",
                                                  ogma::ParamValue{int64_t{32}}));
}

TEST(MotorRepertoire, MasterSeedConstructionOnly) {
    RepertoireFixture f;
    EXPECT_THROW(f.repertoire.on_param_change("master_seed",
                                               ogma::ParamValue{int64_t{99}}),
                 std::invalid_argument);
}

TEST(MotorRepertoire, UnknownParamThrows) {
    RepertoireFixture f;
    EXPECT_THROW(f.repertoire.on_param_change("not_a_real_key", ogma::ParamValue{1.0}),
                 std::invalid_argument);
}

// -- Phase 6.5.3.3: dispatch outcome tracking + Wilson-CI demotion ---------

TEST(MotorRepertoire, DispatchCountsTickOnPlayCmd) {
    // Phase 6.5.12 — dispatch boundary moved from ActionOut.chunk_id
    // edges to MotorPlayCmd.request_id transitions, so chunks that
    // re-dispatch back-to-back (chunk_id stays the same, no -1 gap)
    // are still counted correctly.
    RepertoireFixture f;
    EXPECT_EQ(f.repertoire.total_dispatch_count(), 0);

    auto fire_play_cmd = [&](uint64_t t, int chunk_id, uint64_t req_id) {
        f.bus.begin_tick(t);
        auto cmd = std::make_shared<ogma::MotorPlayCmd>();
        cmd->tick_id    = t;
        cmd->chunk_id   = chunk_id;
        cmd->request_id = req_id;
        f.bus.publish(ogma::topics::kMotorPlayCmd, cmd);
        f.bus.end_tick();
    };
    fire_play_cmd(0, 7, 1);   // first dispatch → +1
    fire_play_cmd(1, 7, 2);   // re-dispatch same chunk → +1 (key fix)
    fire_play_cmd(2, 9, 3);   // different chunk → +1
    fire_play_cmd(3, 9, 4);   // same chunk again → +1
    EXPECT_EQ(f.repertoire.total_dispatch_count(), 4);
}

TEST(MotorRepertoire, FailedDispatchesDemoteChunkViaWilsonCI) {
    auto p = default_params();
    p["crystallization_min_observations"] = int64_t{2};
    RepertoireFixture f(p);

    // Bring up a real chunk via the standard crystallization path.
    float u = 1.0f;
    for (uint64_t t = 0; t < 6; ++t) {
        u -= 0.05f;
        f.run_tick(t, 0.5f, /*motif_id=*/42, u, /*fire_hit=*/true);
    }
    ASSERT_GE(f.repertoire.chunk_count(), 1u);
    int chunk_id = -1;
    {
        auto lib = f.last_library();
        ASSERT_GE(lib->chunks.size(), 1u);
        chunk_id = lib->chunks.front().id;
    }

    // Now drive failure-heavy replay outcomes: many dispatches that
    // each fire many misses and few hits.  Wilson 95% lower-CI on
    // p_hat should drop below 0.5 with sufficient n; chunk gets
    // marked inactive and dispatches start returning empty streams.
    //
    // Phase 6.5.12 — dispatch boundary is the MotorPlayCmd.request_id
    // transition (handled in handle_play_cmd).  Each iteration sends
    // ONE play_cmd (one new dispatch) and follows with N misses + N
    // hits; on the NEXT play_cmd, the previous dispatch is evaluated
    // for Wilson-CI demotion.
    uint64_t request_id = 0;
    auto run_dispatch = [&](uint64_t t, int hits, int misses) {
        f.bus.begin_tick(t);
        auto cmd = std::make_shared<ogma::MotorPlayCmd>();
        cmd->tick_id    = t;
        cmd->chunk_id   = chunk_id;
        cmd->request_id = ++request_id;
        f.bus.publish(ogma::topics::kMotorPlayCmd, cmd);
        // ActionOut for active_replay_chunk_id_ tracking (used by
        // hit/miss event crediting in handle_event).
        auto a = std::make_shared<ogma::ActionOut>();
        a->accel = 0.0f;
        a->chunk_id = chunk_id;
        f.bus.publish(ogma::topics::kActionOut, a);
        for (int i = 0; i < hits; ++i) {
            auto e = std::make_shared<ogma::EnvEvent>();
            e->name = "hit"; e->intensity = 1.0f;
            f.bus.publish("events.hit", e);
        }
        for (int i = 0; i < misses; ++i) {
            auto e = std::make_shared<ogma::EnvEvent>();
            e->name = "miss"; e->intensity = 1.0f;
            f.bus.publish("events.miss", e);
        }
        f.bus.end_tick();
    };

    int active_before = f.repertoire.active_chunk_count();
    EXPECT_EQ(active_before, 1);

    // Six poor dispatches: 1 hit / 5 misses each → cumulatively
    // overwhelmingly miss-biased.  After ≥5 dispatches with low p_hat,
    // Wilson lower-CI drops below 0.5 → chunk demoted.  Six total so
    // the demotion check (which fires on the FOLLOWING dispatch boundary)
    // gets a chance to evaluate the fifth dispatch.
    for (int d = 0; d < 6; ++d) {
        run_dispatch(uint64_t(100 + d), /*hits=*/1, /*misses=*/5);
    }
    EXPECT_EQ(f.repertoire.total_dispatch_count(), 6);
    EXPECT_EQ(f.repertoire.active_chunk_count(), 0);
    EXPECT_GE(f.repertoire.failed_dispatch_count(), 1);
    EXPECT_LT(f.repertoire.dispatch_success_rate(), 1.0f);
}
