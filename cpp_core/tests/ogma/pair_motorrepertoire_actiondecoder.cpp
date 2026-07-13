// =============================================================================
// pair_motorrepertoire_actiondecoder.cpp  --  Phase 3 wiring
// =============================================================================
//
// Real MotorRepertoire ↔ real ActionDecoder.  When `use_chunks = true`,
// ActionDecoder dispatches motor.play.cmd in its tick() and consumes the
// returned MotorPlayStream to drive subsequent ticks tick-by-tick.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/modules/MotorRepertoire.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap repertoire_params() {
    return {
        {"max_chunks",                       int64_t{16}},
        {"chunk_max_ticks",                  int64_t{6}},
        {"crystallization_min_observations", int64_t{2}},
        {"crystallization_min_drive_delta",  0.001},
    };
}

ogma::ParamMap decoder_params(bool use_chunks) {
    return {
        {"consensus_level",  int64_t{0}},
        {"proprio_topic",    std::string("reality.proprio.imu")},
        {"action_bins",      int64_t{3}},
        {"pragmatic_gain",   10.0},
        {"use_chunks",       use_chunks},
    };
}

std::shared_ptr<ogma::ConsensusToken> make_consensus(int active_winner_id) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->fused_embedding   = Eigen::VectorXf::Constant(4, 0.5f);
    c->level             = 0;
    c->active_winner_id  = active_winner_id;
    c->active_modality   = "proprio.imu";
    return c;
}

std::shared_ptr<ogma::NeuroState> make_neuro() {
    auto n = std::make_shared<ogma::NeuroState>();
    n->dopamine = 0.30f; n->serotonin = 0.50f; n->epsilon_b_scale = 1.0f;
    return n;
}

std::shared_ptr<ogma::DriveErrors> make_drive(float urgency = 0.0f) {
    auto d = std::make_shared<ogma::DriveErrors>();
    d->urgency = urgency;
    return d;
}

std::shared_ptr<ogma::RealityToken> make_proprio(int wid) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->winner_id = wid;
    return r;
}

// Manually crystallize a chunk by feeding actions/motifs/drive into the
// MotorRepertoire ahead of the test.
void prime_repertoire(ogma::InProcessBus& bus, ogma::MotorRepertoire& rep) {
    auto motif = std::make_shared<ogma::SequenceMotif>();
    motif->motif_id    = 7;
    motif->motif_length = 5;
    motif->is_baked    = true;   // MotorRepertoire requires this post-Phase-6.1

    float u = 1.0f;
    for (uint64_t t = 0; t < 6; ++t) {
        u -= 0.05f;
        bus.begin_tick(t);
        bus.publish(ogma::topics::kDriveErrors,    make_drive(u));
        auto a = std::make_shared<ogma::ActionOut>();
        a->accel = 0.5f * float(t);
        bus.publish(ogma::topics::kActionOut,      a);
        bus.publish("sequence.motif.action.out",   motif);
        // Post-Phase-6.1 chunk gate requires hits_during ≥ 1 — fire a
        // hit on each tick so the motif crystallises.
        auto ev = std::make_shared<ogma::EnvEvent>();
        ev->name      = "hit";
        ev->intensity = 1.0f;
        bus.publish("events.hit", ev);
        rep.tick(t);
        bus.end_tick();
    }
}

} // namespace

TEST(PairMotorRepertoireActionDecoder, DecoderDispatchesAndPlaysChunk) {
    ogma::InProcessBus       bus;
    ogma::MotorRepertoire    rep;
    ogma::ActionDecoder      decoder;
    rep.set_id("motor_repertoire");
    decoder.set_id("action_decoder");
    rep.on_setup(&bus, repertoire_params());
    decoder.on_setup(&bus, decoder_params(/*use_chunks=*/true));

    prime_repertoire(bus, rep);

    auto chunks = std::dynamic_pointer_cast<const ogma::MotorChunks>(
        bus.last_value(ogma::topics::kMotorChunks));
    ASSERT_NE(chunks, nullptr);
    ASSERT_FALSE(chunks->chunks.empty());

    // Now run the decoder; it should dispatch the chunk on the first tick
    // it has both consensus + proprio AND a chunk library available.
    bool saw_chunk_emission = false;
    for (uint64_t t = 100; t < 110; ++t) {
        bus.begin_tick(t);
        bus.publish("consensus.0",         make_consensus(5));
        bus.publish("drive.errors",        make_drive());
        bus.publish("neuro.state",         make_neuro());
        bus.publish("reality.proprio.imu", make_proprio(2));
        rep.tick(t);
        decoder.tick(t);
        bus.end_tick();

        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(ogma::topics::kActionOut));
        if (a && a->chunk_id >= 0) saw_chunk_emission = true;
    }
    EXPECT_TRUE(saw_chunk_emission);
}

TEST(PairMotorRepertoireActionDecoder, NoDispatchWhenUseChunksDisabled) {
    ogma::InProcessBus       bus;
    ogma::MotorRepertoire    rep;
    ogma::ActionDecoder      decoder;
    rep.set_id("motor_repertoire");
    decoder.set_id("action_decoder");
    rep.on_setup(&bus, repertoire_params());
    decoder.on_setup(&bus, decoder_params(/*use_chunks=*/false));

    prime_repertoire(bus, rep);

    for (uint64_t t = 100; t < 110; ++t) {
        bus.begin_tick(t);
        bus.publish("consensus.0",         make_consensus(5));
        bus.publish("drive.errors",        make_drive());
        bus.publish("neuro.state",         make_neuro());
        bus.publish("reality.proprio.imu", make_proprio(2));
        rep.tick(t);
        decoder.tick(t);
        bus.end_tick();

        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(ogma::topics::kActionOut));
        ASSERT_NE(a, nullptr);
        EXPECT_EQ(a->chunk_id, -1);   // never dispatched
    }
}

TEST(PairMotorRepertoireActionDecoder, FirstTickProducesNoNan) {
    ogma::InProcessBus       bus;
    ogma::MotorRepertoire    rep;
    ogma::ActionDecoder      decoder;
    rep.set_id("motor_repertoire");
    decoder.set_id("action_decoder");
    rep.on_setup(&bus, repertoire_params());
    decoder.on_setup(&bus, decoder_params(true));

    EXPECT_NO_THROW({
        bus.begin_tick(0);
        rep.tick(0);
        decoder.tick(0);
        bus.end_tick();
    });
    auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value(ogma::topics::kActionOut));
    ASSERT_NE(a, nullptr);
    EXPECT_FALSE(std::isnan(a->accel));
}
