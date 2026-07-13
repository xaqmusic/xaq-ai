// =============================================================================
// pair_seqgng_epm.cpp
//
// Real EPM ↔ real SequenceGNG.  Verifies that SequenceGNG, subscribed to an
// EPM's reality.<modality> stream, accumulates motifs over the EPM's actual
// winner_id sequence.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/modules/SequenceGNG.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap epm_params() {
    return {
        {"modality_group",     std::string("proprio")},
        {"modality_name",      std::string("imu")},
        {"encoder_kind",       std::string("rbf")},
        {"input_topic",        std::string("reality.proprio.imu")},
        {"projection_dim",     int64_t{16}},
        {"proprio_state_dims", int64_t{6}},
        {"baking_threshold",   int64_t{10}},
        {"min_insertion_error", 0.001},
        {"subtract_descending_prediction", false},
    };
}

ogma::ParamMap seq_params() {
    return {
        {"source_topic",     std::string("reality.proprio.imu")},
        {"source_kind",      std::string("winner")},
        {"window_size",      int64_t{4}},
        {"projection_dim",   int64_t{32}},
        {"prototype_per_winner_dim", int64_t{16}},
        {"baking_threshold", int64_t{20}},
        {"min_insertion_error", 0.001},
    };
}

std::shared_ptr<ogma::ProprioToken> make_proprio6(float seed) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->sensor = "imu";
    p->values.resize(6);
    p->values << std::sin(seed),     std::cos(seed),
                 std::sin(2.f*seed), std::cos(2.f*seed),
                 0.5f * seed,        -0.5f * seed;
    return p;
}

void run_tick(ogma::InProcessBus& bus, ogma::EPM& epm, ogma::SequenceGNG& seq,
              uint64_t t, std::shared_ptr<ogma::ProprioToken> proprio) {
    bus.begin_tick(t);
    if (proprio) bus.publish("reality.proprio.imu", proprio);
    epm.tick(t);          // publishes reality.proprio.imu(t)
    seq.tick(t);          // publishes sequence.motif.reality.proprio.imu(t)
    bus.end_tick();
}

} // namespace

TEST(PairSeqGngEpm, EpmWinnersFlowIntoSequenceGNG) {
    ogma::InProcessBus    bus;
    ogma::EPM             epm;
    ogma::SequenceGNG     seq;
    epm.set_id("epm_imu");
    seq.set_id("seq_consensus");
    epm.on_setup(&bus, epm_params());
    seq.on_setup(&bus, seq_params());

    // Drive 400 ticks of a periodic proprio signal — the EPM should produce
    // a recurring winner_id pattern, which the SequenceGNG should bake into
    // motifs.
    for (uint64_t t = 0; t < 400; ++t) {
        float phase = 0.5f * std::sin(0.05f * float(t));
        run_tick(bus, epm, seq, t, make_proprio6(phase));
    }

    EXPECT_GE(epm.node_count(), 1);
    EXPECT_GE(seq.node_count(), 1);
}

TEST(PairSeqGngEpm, FirstTickProducesNoNan) {
    ogma::InProcessBus    bus;
    ogma::EPM             epm;
    ogma::SequenceGNG     seq;
    epm.set_id("epm_imu");
    seq.set_id("seq_consensus");
    epm.on_setup(&bus, epm_params());
    seq.on_setup(&bus, seq_params());

    EXPECT_NO_THROW({
        run_tick(bus, epm, seq, 0, make_proprio6(0.0f));
    });

    auto m = std::dynamic_pointer_cast<const ogma::SequenceMotif>(
        bus.last_value("sequence.motif.reality.proprio.imu"));
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(m->motif_id, -1);
    EXPECT_FALSE(std::isnan(m->match_confidence));
}
