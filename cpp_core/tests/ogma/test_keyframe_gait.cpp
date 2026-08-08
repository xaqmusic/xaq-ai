// =============================================================================
// test_keyframe_gait.cpp
//   KeyframeGait unit test (L-1b): the phase-indexed keyframe map crystallizes a
//   recurring phase→posture pattern, publishes it on the objective socket, and the
//   phase INDEX (not time-averaging) is what does the work (shuffle_phase washes it
//   out).  Map round-trips snapshot/restore.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/KeyframeGait.hpp"
#include "ogma/Topics.hpp"

namespace {

using ogma::ParamMap;
constexpr float TWO_PI = 6.28318530717958647692f;

ParamMap kg_params(bool shuffle = false) {
    ParamMap p;
    p["cpg_topic"]               = std::string("cpg");
    p["proprio_topics"]          = std::vector<std::string>{"leg.fl", "leg.fr", "leg.rl", "leg.rr"};
    p["objective_output_topics"] = std::vector<std::string>{"obj.fl", "obj.fr", "obj.rl", "obj.rr"};
    p["motor_dim"]      = int64_t{3};
    p["n_bins"]         = int64_t{16};
    p["keyframe_alpha"] = 0.1;      // faster convergence for a short test
    p["gain"]           = 0.5;
    p["shuffle_phase"]  = shuffle;
    return p;
}

// A distinct, constant-per-bin posture (so a perfectly phase-locked pattern crystallizes
// to ~0 TLE; bin 0 differs clearly from bin 8).
std::vector<float> bin_posture(float phi) {
    float h = std::fmod(phi, TWO_PI); if (h < 0) h += TWO_PI;
    int b = int(h / TWO_PI * 16.0f); if (b >= 16) b = 15;
    return { 0.5f * float(b) / 16.0f, 0.1f, (b % 2) ? 0.2f : -0.2f };
}

std::shared_ptr<ogma::ProprioToken> vecf(std::vector<float> const& v) {
    auto pt = std::make_shared<ogma::ProprioToken>();
    pt->values = Eigen::VectorXf((int)v.size());
    for (int i = 0; i < (int)v.size(); ++i) pt->values[i] = v[i];
    return pt;
}

// One tick: publish CPG phase [cos,sin] + the same posture on all 4 legs, then tick.
void step(ogma::InProcessBus& bus, ogma::KeyframeGait& kg, uint64_t t,
          float phi, std::vector<float> const& pos3) {
    bus.begin_tick(t);
    bus.publish("cpg", vecf({std::cos(phi), std::sin(phi)}));
    const char* legs[4] = {"leg.fl", "leg.fr", "leg.rl", "leg.rr"};
    for (int l = 0; l < 4; ++l)
        bus.publish(legs[l], vecf({pos3[0], 0, 0, pos3[1], 0, 0, pos3[2], 0, 0}));
    kg.tick(t);
    bus.end_tick();
}

Eigen::VectorXf published(ogma::InProcessBus& bus, const char* topic) {
    auto tok = std::dynamic_pointer_cast<const ogma::PredictionToken>(bus.last_value(topic));
    return tok ? tok->predicted_latent : Eigen::VectorXf();
}

float published_conf(ogma::InProcessBus& bus, const char* topic) {
    auto tok = std::dynamic_pointer_cast<const ogma::PredictionToken>(bus.last_value(topic));
    return tok ? tok->confidence : -1.0f;
}

void drive(ogma::InProcessBus& bus, ogma::KeyframeGait& kg, int cycles, uint64_t& t) {
    for (int c = 0; c < cycles; ++c)
        for (int k = 0; k < 32; ++k, ++t) {
            float phi = std::fmod(float(k) / 32.0f * TWO_PI, TWO_PI);
            step(bus, kg, t, phi, bin_posture(phi));
        }
}

// ---- velocity-map helpers (the propulsive-push lever) ----
// KeyframeGait reads the joint DELTA at proprio index 3j+2 as the velocity; feed a
// phase-varying delta so the velocity map is a real phase-indexed signal, not zeros.
ParamMap kg_params_vel(bool shuffle = false) {
    ParamMap p = kg_params(shuffle);
    p["velocity_output_topics"] = std::vector<std::string>{"vel.fl", "vel.fr", "vel.rl", "vel.rr"};
    return p;
}

// A bin-CONSTANT phase-varying velocity (like bin_posture): constant within each bin so the
// map crystallizes to a fixed point (an extra EMA step is a no-op → exact round-trip), and
// clearly distinct between bin 0 (φ=0 → {0.3,0.1,0}) and bin 8 (φ=π → {−0.3,0.1,0}).
std::vector<float> bin_vel(float phi) {
    float h = std::fmod(phi, TWO_PI); if (h < 0) h += TWO_PI;
    int b = int(h / TWO_PI * 16.0f); if (b >= 16) b = 15;
    float bphi = float(b) / 16.0f * TWO_PI;
    return { 0.3f * std::cos(bphi), 0.1f, 0.2f * std::sin(bphi) };
}

// One tick with BOTH posture (index 3j) and velocity/delta (index 3j+2) populated.
void step_v(ogma::InProcessBus& bus, ogma::KeyframeGait& kg, uint64_t t,
            float phi, std::vector<float> const& pos3, std::vector<float> const& vel3) {
    bus.begin_tick(t);
    bus.publish("cpg", vecf({std::cos(phi), std::sin(phi)}));
    const char* legs[4] = {"leg.fl", "leg.fr", "leg.rl", "leg.rr"};
    for (int l = 0; l < 4; ++l)
        bus.publish(legs[l], vecf({pos3[0], 0, vel3[0], pos3[1], 0, vel3[1], pos3[2], 0, vel3[2]}));
    kg.tick(t);
    bus.end_tick();
}

void drive_v(ogma::InProcessBus& bus, ogma::KeyframeGait& kg, int cycles, uint64_t& t) {
    for (int c = 0; c < cycles; ++c)
        for (int k = 0; k < 32; ++k, ++t) {
            float phi = std::fmod(float(k) / 32.0f * TWO_PI, TWO_PI);
            step_v(bus, kg, t, phi, bin_posture(phi), bin_vel(phi));
        }
}

// One tick with a DISTINCT velocity per leg (for the left-right symmetry prior test).
void step_v4(ogma::InProcessBus& bus, ogma::KeyframeGait& kg, uint64_t t, float phi,
             std::vector<float> const& pos3, std::vector<std::vector<float>> const& vel4) {
    bus.begin_tick(t);
    bus.publish("cpg", vecf({std::cos(phi), std::sin(phi)}));
    const char* legs[4] = {"leg.fl", "leg.fr", "leg.rl", "leg.rr"};
    for (int l = 0; l < 4; ++l) {
        auto const& v = vel4[l];
        bus.publish(legs[l], vecf({pos3[0], 0, v[0], pos3[1], 0, v[1], pos3[2], 0, v[2]}));
    }
    kg.tick(t);
    bus.end_tick();
}

} // namespace

TEST(KeyframeGait, PhaseIndexedNotTimeAveraged) {
    ogma::InProcessBus bus; ogma::KeyframeGait kg; kg.set_id("kg");
    kg.on_setup(&bus, kg_params());
    EXPECT_EQ(kg.type_name(), "KeyframeGait");
    EXPECT_EQ(kg.n_legs(), 4);

    uint64_t t = 0;
    drive(bus, kg, 20, t);

    // Probe bin 0 (φ=0) and bin 8 (φ=π), feeding each bin's own posture so the read
    // doesn't move the map. The published objective IS keyframe[bin] sliced to a leg.
    step(bus, kg, t++, 0.0f,    bin_posture(0.0f));    Eigen::VectorXf k0 = published(bus, "obj.fl");
    step(bus, kg, t++, 3.14159f, bin_posture(3.14159f)); Eigen::VectorXf k8 = published(bus, "obj.fl");
    ASSERT_EQ(k0.size(), 3);
    ASSERT_EQ(k8.size(), 3);
    EXPECT_GT((k0 - k8).norm(), 0.1f) << "phase-indexed: keyframe[0] must differ from keyframe[π]";
    // all four legs must publish a target
    for (const char* o : {"obj.fl", "obj.fr", "obj.rl", "obj.rr"})
        EXPECT_EQ(published(bus, o).size(), 3) << "leg " << o << " objective missing";
}

TEST(KeyframeGait, CrystallizesAndShufflePhaseWashesOut) {
    // Phase-INDEXED keeps keyframe[bin] equal to the recurring posture at that phase →
    // low TLE (crystallized). SHUFFLE routes each phase to a random bin, so every bin
    // becomes the mean of all phases → the posture never matches → TLE stays high. The
    // contrast IS the "the phase index does the work" proof (§2.8). (The "keyframe_tle
    // FALLS over the run" trend is the headless Gate 1; here the pattern is noise-free so
    // it crystallizes near-instantly, hence we assert the crystallized level + the ablation.)
    auto tle_after = [](bool shuffle) {
        ogma::InProcessBus bus; ogma::KeyframeGait kg; kg.set_id("kg");
        kg.on_setup(&bus, kg_params(shuffle));
        uint64_t t = 0;
        drive(bus, kg, 25, t);
        return kg.keyframe_tle();
    };
    float normal   = tle_after(false);
    float shuffled = tle_after(true);
    EXPECT_LT(normal, 0.05f)          << "phase-indexed: a consistent phase→posture crystallizes to low TLE";
    EXPECT_GT(shuffled, 0.1f)         << "shuffle_phase: bins see a random mix → keyframe stays a high-TLE mean";
    EXPECT_GT(shuffled, normal * 3.0f) << "the phase index is what does the work";
}

TEST(KeyframeGait, SelfPrecisionGatesDrive) {
    // The published confidence = gain · self_precision: an unproven bin barely drives (warmup),
    // and a crystallized bin drives near the base gain.  This is the premature-drive fix and the
    // precision the EFE arbiter will later scale.
    ogma::InProcessBus bus; ogma::KeyframeGait kg; kg.set_id("kg");
    auto p = kg_params();
    p["gain"]            = 0.5;
    p["warmup_visits"]   = int64_t{64};
    p["precision_scale"] = 0.6;
    kg.on_setup(&bus, p);

    uint64_t t = 0;
    drive(bus, kg, 2, t);                                   // ~4 visits/bin — well under warmup
    step(bus, kg, t++, 0.0f, bin_posture(0.0f));
    float early = published_conf(bus, "obj.fl");
    EXPECT_GE(early, 0.0f);
    EXPECT_LT(early, 0.1f) << "an unproven bin must barely drive (warmup gate)";

    drive(bus, kg, 40, t);                                  // ~80 visits/bin — crystallized
    step(bus, kg, t++, 0.0f, bin_posture(0.0f));
    float late = published_conf(bus, "obj.fl");
    EXPECT_GT(late, early)  << "a crystallized bin must drive more than an unproven one";
    EXPECT_GT(late, 0.3f)   << "a consistent bin should approach the base gain (0.5)";
    // The target itself is still published throughout (only the confidence is gated).
    EXPECT_EQ(published(bus, "obj.fl").size(), 3);
}

TEST(KeyframeGait, SnapshotRestoreRoundTripsMap) {
    ogma::InProcessBus bus; ogma::KeyframeGait kg; kg.set_id("kg");
    kg.on_setup(&bus, kg_params());
    uint64_t t = 0;
    drive(bus, kg, 20, t);

    step(bus, kg, t++, 0.0f, bin_posture(0.0f));
    Eigen::VectorXf before = published(bus, "obj.fl");
    auto snap = kg.snapshot_state();
    EXPECT_EQ(snap["version"].get<int>(), 2);

    ogma::KeyframeGait kg2; kg2.set_id("kg");
    kg2.on_setup(&bus, kg_params());
    kg2.restore_state(snap);
    EXPECT_EQ(kg2.bins_filled(), kg.bins_filled());
    // restored instance publishes the restored keyframe (feed matching posture → no drift).
    step(bus, kg2, t++, 0.0f, bin_posture(0.0f));
    Eigen::VectorXf after = published(bus, "obj.fl");
    ASSERT_EQ(before.size(), after.size());
    EXPECT_LT((before - after).norm(), 1e-4f) << "snapshot/restore must reproduce the keyframe map";
}

TEST(KeyframeGait, VelocityMapPhaseIndexedCrystallizesAndPublishes) {
    // The propulsive-push lever: alongside the posture map, KeyframeGait learns a phase-indexed
    // VELOCITY map (EMA of the body's own joint delta per CPG-phase bin) and publishes it on the
    // objective.velocity.* socket.  Assert it is (i) phase-indexed (bin0 ≠ binπ), (ii) crystallizes
    // to the recurring per-phase velocity, (iii) published on every leg alongside the posture socket.
    ogma::InProcessBus bus; ogma::KeyframeGait kg; kg.set_id("kg");
    kg.on_setup(&bus, kg_params_vel());
    uint64_t t = 0;
    drive_v(bus, kg, 25, t);

    // Probe bin 0 (φ=0) and bin 8 (φ=π), feeding each bin's own posture+velocity so the read
    // doesn't move the map.
    step_v(bus, kg, t++, 0.0f,     bin_posture(0.0f),     bin_vel(0.0f));
    Eigen::VectorXf v0 = published(bus, "vel.fl");
    step_v(bus, kg, t++, 3.14159f, bin_posture(3.14159f), bin_vel(3.14159f));
    Eigen::VectorXf v8 = published(bus, "vel.fl");
    ASSERT_EQ(v0.size(), 3);
    ASSERT_EQ(v8.size(), 3);
    EXPECT_GT((v0 - v8).norm(), 0.1f) << "phase-indexed: velocity[0] must differ from velocity[π]";

    // Crystallized: the published velocity target matches the recurring velocity fed at that phase.
    auto fed = bin_vel(0.0f);
    Eigen::VectorXf fed0(3); fed0 << fed[0], fed[1], fed[2];
    EXPECT_LT((v0 - fed0).norm(), 0.05f) << "velocity map crystallizes to the per-phase velocity";

    // Every leg publishes a velocity target, and the posture socket still drives alongside it.
    for (const char* o : {"vel.fl", "vel.fr", "vel.rl", "vel.rr"})
        EXPECT_EQ(published(bus, o).size(), 3) << "leg " << o << " velocity objective missing";
    EXPECT_EQ(published(bus, "obj.fl").size(), 3) << "posture socket must still publish";
}

TEST(KeyframeGait, VelocityMapShufflePhaseWashesOut) {
    // The phase INDEX is what makes the velocity map real: shuffling the bin routes every phase
    // to a random bin, so each bin becomes the mean velocity over all phases → bin0 ≈ binπ.
    auto v_gap = [](bool shuffle) {
        ogma::InProcessBus bus; ogma::KeyframeGait kg; kg.set_id("kg");
        kg.on_setup(&bus, kg_params_vel(shuffle));
        uint64_t t = 0;
        drive_v(bus, kg, 30, t);
        step_v(bus, kg, t++, 0.0f,     bin_posture(0.0f),     bin_vel(0.0f));
        Eigen::VectorXf v0 = published(bus, "vel.fl");
        step_v(bus, kg, t++, 3.14159f, bin_posture(3.14159f), bin_vel(3.14159f));
        Eigen::VectorXf v8 = published(bus, "vel.fl");
        return (v0 - v8).norm();
    };
    float indexed  = v_gap(false);
    float shuffled = v_gap(true);
    EXPECT_GT(indexed, 0.1f)          << "phase-indexed: the velocity map separates phases";
    EXPECT_LT(shuffled, indexed * 0.5f) << "shuffle: bins collapse to the mean velocity → the index does the work";
}

TEST(KeyframeGait, VelocitySymmetryEqualizesPairedLegs) {
    // Anti-circling prior: with asymmetric push between paired legs (leg0/leg2 strong,
    // leg1/leg3 weak), vel_symmetry_gain>0 must equalize the paired legs' velocity-target
    // magnitude (so the pump drives straight); gain=0 leaves the asymmetry intact.
    auto run = [](double gain) {
        ogma::InProcessBus bus; ogma::KeyframeGait kg; kg.set_id("kg");
        auto p = kg_params_vel();
        p["symmetry_pairs"]    = std::vector<double>{0, 1, 2, 3};   // FL<->FR, RL<->RR
        p["vel_symmetry_gain"] = gain;
        kg.on_setup(&bus, p);
        uint64_t t = 0;
        std::vector<std::vector<float>> vel4 = {
            {0.40f, 0.10f, 0.20f}, {0.10f, 0.10f, 0.05f},   // leg0 strong, leg1 weak (same pair)
            {0.40f, 0.10f, 0.20f}, {0.10f, 0.10f, 0.05f}};  // leg2 strong, leg3 weak (same pair)
        for (int c = 0; c < 40; ++c)
            for (int k = 0; k < 32; ++k, ++t) {
                float phi = std::fmod(float(k) / 32.0f * TWO_PI, TWO_PI);
                step_v4(bus, kg, t, phi, bin_posture(phi), vel4);
            }
        step_v4(bus, kg, t++, 0.0f, bin_posture(0.0f), vel4);
        float a = std::abs(published(bus, "vel.fl")[0]);   // leg0 joint0 push magnitude
        float b = std::abs(published(bus, "vel.fr")[0]);   // leg1 joint0 push magnitude
        return std::make_pair(a, b);
    };
    auto off = run(0.0);
    auto on  = run(0.5);
    EXPECT_GT(off.first, off.second * 2.0f) << "no symmetry: paired legs stay asymmetric (leg0 >> leg1)";
    float ratio = on.first / (on.second + 1e-6f);
    EXPECT_LT(ratio, 1.5f) << "symmetry prior equalizes paired-leg push magnitude";
    EXPECT_GT(ratio, 0.67f) << "symmetry prior equalizes paired-leg push magnitude";
}

TEST(KeyframeGait, VelocityMapSnapshotRoundTrips) {
    ogma::InProcessBus bus; ogma::KeyframeGait kg; kg.set_id("kg");
    kg.on_setup(&bus, kg_params_vel());
    uint64_t t = 0;
    drive_v(bus, kg, 25, t);
    step_v(bus, kg, t++, 0.0f, bin_posture(0.0f), bin_vel(0.0f));
    Eigen::VectorXf before = published(bus, "vel.fl");
    auto snap = kg.snapshot_state();

    ogma::KeyframeGait kg2; kg2.set_id("kg");
    kg2.on_setup(&bus, kg_params_vel());
    kg2.restore_state(snap);
    step_v(bus, kg2, t++, 0.0f, bin_posture(0.0f), bin_vel(0.0f));
    Eigen::VectorXf after = published(bus, "vel.fl");
    ASSERT_EQ(before.size(), after.size());
    EXPECT_LT((before - after).norm(), 1e-4f) << "snapshot/restore must reproduce the velocity map";
}
