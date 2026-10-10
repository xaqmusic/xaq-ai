// =============================================================================
// test_state_prior.cpp
//   MotorEPMv2 state-space prior (2026-08-31, the microduck lever).
//
//   The objective socket can retarget only joint-position components (idx = 3j);
//   this lever generalises the same ξ̃ blend to arbitrary state indices, so a
//   prior can live on an appended sensor element — the bridge's load slot —
//   e.g. "predicted lean = 0" on a body whose three verified-fired nulls all
//   pointed at the objective (microduck port plan §A1).
//
//   What is locked down, and the failure each guard answers for:
//     1. GainZeroByteIdenticalNonzeroActs — the gain-0 contract, in the HARD form:
//        indices+targets configured and the lean element live on the bus, so
//        byte-identity must come from the explicit gain branch, not from the
//        socket happening to be idle.  Plus the A-arm divergence + diag checks.
//     2. MisSizedTargetsInert — parallel-array mismatch disables the prior AND
//        says so in diag (the postural_gain_joints silent-no-op lesson).
//     3. NegativeIndexResolvesToLast — −1 ≡ explicit last index, exactly.
//     4. OutOfRangeIndexSkipped — no crash, no effect, and the applied-count
//        diag says 0 while active says true (the disambiguation).
//     5. DirectionOfPull — the sign control (v2 plan §7 rule 4): a +target and a
//        −target must pull a neutral integrator plant to OPPOSITE sides.  A
//        lever whose sign does not matter is not a mechanism.
//     6. PriorStabilizesUnstableLean — the capability at unit scale: a scalar
//        plant that diverges under the bare HK rule is held near 0 by the prior,
//        THROUGH the learned model (no hand-wired feedback anywhere in the test).
//     7. HotParamRoundTrip — on_param_change round-trips current_params.
//     8. IsolateHoldsCToThePriorsOwnColumns — state_prior_isolate (2026-09-12, the duck's
//     9. ModelImpliedStepClosesTheErrorItself — state_prior_step_gain (2026-09-12, W5 (b)):
//        the gain-0 guard, that it acts, that it PULLS THE RIGHT WAY on its own (with the
//        descent switched off, so nothing else could be doing it), and the diag read-back.
//        W5 lesion): the gain-0 guard, that it ACTS when on, and that the kept-column
//        count in diag says how many columns survived (the read-back a sweep asserts on,
//        §3.2 rule 5 — the R47 arm was only trustable because spIso said 5).
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/MotorEPMv2.hpp"
#include "ogma/Topics.hpp"

namespace {

using ogma::ParamMap;
using ogma::ParamValue;

// One leg, three motors, state = [pos,act,delta]×3 + 1 appended "lean" slot = 10.
constexpr int kLegs = 1, kMotors = 3, kStateN = 3 * kMotors + 1, kLeanIdx = kStateN - 1;

ParamMap base_params() {
    ParamMap p;
    p["n_legs"]         = int64_t{kLegs};
    p["motor_dim"]      = int64_t{kMotors};
    p["seed"]           = int64_t{1234};
    p["babble_ticks"]   = int64_t{12};
    p["explore_noise"]  = 0.0;                    // deterministic post-warmup
    p["proprio_topics"] = std::vector<std::string>{"sp.p0"};
    p["action_topics"]  = std::vector<std::string>{"sp.a0", "sp.a1", "sp.a2"};
    p["imu_topic"]      = std::string("sp.imu");
    p["coupling_gain"]      = 0.0;
    p["coord_reward_drive"] = 0.0;
    p["amp_seek_rate"]      = 0.0;
    p["stroke_gain"]        = 0.0;
    p["balance_gain"]       = 0.0;
    p["heading_gain"]       = 0.0;
    p["nav_gain"]           = 0.0;
    p["postural_gain"]      = 0.0;
    p["height_homeo_gain"]  = 0.0;
    p["panic_strength"]     = 0.0;
    return p;
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::MotorEPMv2   m;

    explicit Fixture(ParamMap const& p) {
        m.set_id("motor_epm_v2");
        m.on_setup(&bus, p);
    }

    // Scripted joint channels (open loop) + the lean value in the appended slot.
    void run_tick(uint64_t t, float lean) {
        bus.begin_tick(t);
        auto pt = std::make_shared<ogma::ProprioToken>();
        pt->values = Eigen::VectorXf::Zero(kStateN);
        const double ph = 0.15 * double(t);
        for (int j = 0; j < kMotors; ++j) {
            pt->values[3 * j + 0] = float(0.30 * std::sin(ph + j));
            pt->values[3 * j + 1] = float(0.20 * std::cos(ph + j));
            pt->values[3 * j + 2] = float(0.30 * 0.15 * std::cos(ph + j));
        }
        pt->values[kLeanIdx] = lean;
        pt->sensor = "proprio";
        bus.publish("sp.p0", pt);
        m.tick(t);
        bus.end_tick();
    }

    float accel(int j) const {
        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value("sp.a" + std::to_string(j)));
        return a ? a->accel : std::numeric_limits<float>::quiet_NaN();
    }
};

// A scripted lean the open-loop tests share: a slow wobble, clearly non-zero.
float wobble(uint64_t t) { return float(0.5 * std::sin(0.07 * double(t))); }

}  // namespace

// =============================================================================
// 1. The gain-0 contract, hard form — plus the A-arm divergence and its diags.
// =============================================================================
TEST(StatePrior, GainZeroByteIdenticalNonzeroActs) {
    auto pn = base_params();                                // N: no prior params at all
    auto pz = base_params();                                // Z: configured, gain 0
    pz["state_prior_indices"] = std::vector<double>{-1.0};
    pz["state_prior_targets"] = std::vector<double>{0.0};
    pz["state_prior_gain"]    = 0.0;
    auto pa = pz;                                           // A: live
    pa["state_prior_gain"]    = 0.8;

    Fixture N(pn), Z(pz), A(pa);
    double maxdiff_zn = 0.0, maxdiff_az = 0.0;
    for (uint64_t t = 0; t < 300; ++t) {
        const float lean = wobble(t);
        N.run_tick(t, lean); Z.run_tick(t, lean); A.run_tick(t, lean);
        if (t < 12) continue;
        for (int j = 0; j < kMotors; ++j) {
            maxdiff_zn = std::max(maxdiff_zn, double(std::fabs(N.accel(j) - Z.accel(j))));
            maxdiff_az = std::max(maxdiff_az, double(std::fabs(A.accel(j) - Z.accel(j))));
        }
    }
    EXPECT_LT(maxdiff_zn, 1e-6)
        << "state_prior_gain=0 must be byte-identical with indices/targets configured "
           "and the lean element live on the bus (the gain-0 guard)";
    EXPECT_GT(maxdiff_az, 1e-4)
        << "a live state prior changed nothing — the socket is inert";

    auto dA = A.m.diag_snapshot();
    auto dZ = Z.m.diag_snapshot();
    EXPECT_TRUE(dA["state_prior_active"].get<bool>());
    EXPECT_FALSE(dZ["state_prior_active"].get<bool>());
    EXPECT_EQ(dA["state_prior_applied"].get<int>(), 1);
    EXPECT_GT(dA["state_prior_err"].get<float>(), 0.05f)
        << "the wobble never sits at the target, so the err meter must be non-zero";
    EXPECT_NEAR(dZ["state_prior_err"].get<float>(), 0.0f, 1e-6f)
        << "an off prior must DECAY its meter, not latch it";
}

// =============================================================================
// 2. Parallel-array mismatch disables the prior AND says so in diag.
// =============================================================================
TEST(StatePrior, MisSizedTargetsInert) {
    auto pn = base_params();
    auto pm = base_params();
    pm["state_prior_indices"] = std::vector<double>{-1.0};
    pm["state_prior_targets"] = std::vector<double>{0.0, 0.0};   // mismatch
    pm["state_prior_gain"]    = 0.8;

    Fixture N(pn), M(pm);
    double maxdiff = 0.0;
    for (uint64_t t = 0; t < 200; ++t) {
        const float lean = wobble(t);
        N.run_tick(t, lean); M.run_tick(t, lean);
        if (t < 12) continue;
        for (int j = 0; j < kMotors; ++j)
            maxdiff = std::max(maxdiff, double(std::fabs(N.accel(j) - M.accel(j))));
    }
    EXPECT_LT(maxdiff, 1e-6) << "a mis-sized prior must be a perfect no-op";
    EXPECT_FALSE(M.m.diag_snapshot()["state_prior_active"].get<bool>())
        << "and it must SAY it is off (the silent-no-op lesson)";
}

// =============================================================================
// 3. −1 resolves to the last element, exactly.
// =============================================================================
TEST(StatePrior, NegativeIndexResolvesToLast) {
    auto mk = [](double idx) {
        auto p = base_params();
        p["state_prior_indices"] = std::vector<double>{idx};
        p["state_prior_targets"] = std::vector<double>{0.0};
        p["state_prior_gain"]    = 0.8;
        return p;
    };
    Fixture Neg(mk(-1.0)), Pos(mk(double(kLeanIdx)));
    double maxdiff = 0.0;
    for (uint64_t t = 0; t < 200; ++t) {
        const float lean = wobble(t);
        Neg.run_tick(t, lean); Pos.run_tick(t, lean);
        if (t < 12) continue;
        for (int j = 0; j < kMotors; ++j)
            maxdiff = std::max(maxdiff, double(std::fabs(Neg.accel(j) - Pos.accel(j))));
    }
    EXPECT_LT(maxdiff, 1e-9) << "-1 and the explicit last index must be the same prior";
}

// =============================================================================
// 4. Out-of-range index: skipped, no crash, and the diag disambiguates.
// =============================================================================
TEST(StatePrior, OutOfRangeIndexSkipped) {
    auto pn = base_params();
    auto po = base_params();
    po["state_prior_indices"] = std::vector<double>{42.0};
    po["state_prior_targets"] = std::vector<double>{0.0};
    po["state_prior_gain"]    = 0.8;

    Fixture N(pn), O(po);
    double maxdiff = 0.0;
    for (uint64_t t = 0; t < 200; ++t) {
        const float lean = wobble(t);
        N.run_tick(t, lean); O.run_tick(t, lean);
        if (t < 12) continue;
        for (int j = 0; j < kMotors; ++j)
            maxdiff = std::max(maxdiff, double(std::fabs(N.accel(j) - O.accel(j))));
    }
    EXPECT_LT(maxdiff, 1e-6) << "an out-of-range index must change nothing";
    auto d = O.m.diag_snapshot();
    EXPECT_TRUE(d["state_prior_active"].get<bool>())
        << "the config parses, so active reads true...";
    EXPECT_EQ(d["state_prior_applied"].get<int>(), 0)
        << "...and applied==0 is what says the index never resolved";
}

// =============================================================================
// The closed-loop plant the mechanism tests share: a 1-D "lean" the module's own
// commands drive through a fixed (unknown-to-the-controller) authority vector.
// Nothing in the test hands the controller a feedback law — it must LEARN the
// authority through A and be retargeted through ξ̃.
// =============================================================================
namespace {
struct PlantRun {
    float mean_lean = 0.0f;      // late-run mean (signed)
    float mean_abs  = 0.0f;      // late-run mean |lean|
    int   falls     = 0;         // rescue count (the plant's A2 analog)
    int   longest_up = 0;        // longest stretch of ticks without a fall — "it stands"
};

PlantRun run_plant(ParamMap const& p, float alpha, float target_drift, uint64_t ticks,
                   unsigned noise_seed, bool trace = false) {
    Fixture f(p);
    std::mt19937 rng(noise_seed);
    std::uniform_real_distribution<float> nd(-0.02f, 0.02f);
    // Authority: each motor pushes the lean with a different sign/magnitude, so the
    // controller must apportion, not just co-contract.  Sized so the rails are
    // escapable: at the ±1.5 rail with alpha 0.05 escape needs |push| > 0.075, and
    // max |push| here is 0.10 — a plant the controller CAN save is the only kind
    // whose failure means anything.
    const float k[kMotors] = {0.050f, -0.030f, 0.020f};
    float lean = 0.05f;          // a small initial tilt
    int since_fall = 0;
    PlantRun out;
    int late_n = 0;
    for (uint64_t t = 0; t < ticks; ++t) {
        // Joint channels stay at ZERO for the plant runs: the entire sensorimotor loop
        // is the 1-D lean, so attribution is total — nothing else moves.
        f.bus.begin_tick(t);
        auto pt = std::make_shared<ogma::ProprioToken>();
        pt->values = Eigen::VectorXf::Zero(kStateN);
        pt->values[kLeanIdx] = lean;
        pt->sensor = "proprio";
        f.bus.publish("sp.p0", pt);
        f.m.tick(t);
        f.bus.end_tick();

        float push = 0.0f;
        float ys[kMotors] = {0, 0, 0};
        for (int j = 0; j < kMotors; ++j) {
            const float a = f.accel(j);
            ys[j] = a;
            if (std::isfinite(a)) push += k[j] * a;
        }
        lean = (1.0f + alpha) * lean + push + nd(rng) + target_drift;
        // The rescue, mirroring the duck's A2 harness: a railed lean is a FALL — the
        // clamp destroys the action→lean correlation, so the model correctly learns
        // "no authority here" and the learned route dies.  The duck never has to
        // recover from that regime (the scaffold stands it back up); neither does
        // the plant.  A fall is counted and the body is set back near upright.
        if (std::fabs(lean) >= 1.5f) {
            ++out.falls;
            lean = (lean > 0 ? 0.05f : -0.05f);
            since_fall = 0;
        } else {
            out.longest_up = std::max(out.longest_up, ++since_fall);
        }
        if (trace && t % 500 == 0) {
            auto snap = f.m.snapshot_state();
            auto const& lj = snap["legs"][0];
            auto C = lj["C"].get<std::vector<float>>();   // m x n, COLUMN-major (Eigen)
            auto h = lj["h"].get<std::vector<float>>();
            auto A = lj["A"].get<std::vector<float>>();   // n x m, COLUMN-major
            std::fprintf(stderr,
                "    t=%5llu lean=%+.3f push=%+.4f y=[%+.2f %+.2f %+.2f] "
                "C(:,lean)=[%+.3f %+.3f %+.3f] h=[%+.2f %+.2f %+.2f] A(lean,:)=[%+.4f %+.4f %+.4f] falls=%d\n",
                (unsigned long long)t, lean, push, ys[0], ys[1], ys[2],
                C[kLeanIdx * kMotors + 0], C[kLeanIdx * kMotors + 1], C[kLeanIdx * kMotors + 2],
                h[0], h[1], h[2],
                A[0 * kStateN + kLeanIdx], A[1 * kStateN + kLeanIdx], A[2 * kStateN + kLeanIdx],
                out.falls);
        }
        if (t >= ticks * 3 / 4) { out.mean_lean += lean; out.mean_abs += std::fabs(lean); ++late_n; }
    }
    out.mean_lean /= float(late_n);
    out.mean_abs  /= float(late_n);
    return out;
}

ParamMap plant_params(double gain, double target) {
    auto p = base_params();
    p["state_prior_indices"] = std::vector<double>{-1.0};
    p["state_prior_targets"] = std::vector<double>{target};
    p["state_prior_gain"]    = gain;
    // The mechanism arms keep the DEPLOYED configs' excitation: babble long enough to
    // identify the authority channel and persistent explore noise so identification
    // never starves.  (First version ran babble 12 + explore 0 for determinism and the
    // model's A(lean,:) came out sign-INVERTED from n=12 samples, then decayed to 0 —
    // the prior descending through noise.  Determinism never required silence: the
    // noise streams are per-seed deterministic anyway.)
    p["babble_ticks"]  = int64_t{100};
    p["explore_noise"] = 0.05;
    // sat_lr = 0, measured twice on this plant: (a) sat unwinds tonic commands ~5x
    // faster than the h-path builds them; (b) worse, its C-row erosion is proportional
    // to gs·prev_xᵀ — prev_x carries the very element the learned feedback reads — so
    // it erodes the prior's feedback column IN PROPORTION TO ITS USE.  Under sat 0.02
    // the prior arm fell 261 times against 97 for no control at all.
    p["sat_lr"] = 0.0;
    // ctrl_damping stays 0, also measured: L2 cannot tell the feedback column from the
    // windup bias (both O(2) here) and killed balance first (201 falls vs 141 off).
    // The h path's windup is handled by conditional anti-windup at the use site.
    return p;
}
}  // namespace

// =============================================================================
// 5. The sign control: +target and −target must land on OPPOSITE sides.
// =============================================================================
TEST(StatePrior, DirectionOfPull) {
    // A neutral leaky integrator (alpha slightly negative), no drift: where the lean
    // settles is decided by what the controller learned to want.
    const auto plus  = run_plant(plant_params(1.0, +0.4), /*alpha*/ -0.02f, 0.0f, 4000, 7);
    const auto minus = run_plant(plant_params(1.0, -0.4), /*alpha*/ -0.02f, 0.0f, 4000, 7);
    EXPECT_GT(plus.mean_lean, minus.mean_lean + 0.2f)
        << "+target " << plus.mean_lean << " vs -target " << minus.mean_lean
        << " — the pull must follow the target's sign through the learned model";
    EXPECT_GT(plus.mean_lean,  0.05f) << "the +0.4 prior must pull the lean positive";
    EXPECT_LT(minus.mean_lean, -0.05f) << "the -0.4 prior must pull the lean negative";
}

// =============================================================================
// 6. The capability: an unstable lean the bare rule keeps toppling is held up by
//    the prior — through the learned model, with no hand-wired feedback anywhere.
//    Metric shape = the duck harness in miniature: FALLS, plus late-run |lean|.
// =============================================================================
TEST(StatePrior, PriorStabilizesUnstableLean) {
    // alpha > 0: left alone the lean grows ~5%/tick, topples, and is rescued.
    //
    // What is asserted is what the mechanism honestly provides, no more: a large falls
    // reduction AND a long balanced stretch.  NOT a low late-window mean |lean| — the
    // known closed-loop de-identification cycle (see the state_prior_gain docstring)
    // ends long balanced stretches with a collapse-and-re-identify episode, and a
    // window metric straddling one would fail a run whose balance is real.  The
    // stretch metric is the anti-blind complement: free-fall alone cannot produce a
    // 1500-tick stretch when toppling takes ~70 ticks from rest.
    const float kAlpha = 0.05f;
    const auto off = run_plant(plant_params(0.0, 0.0), kAlpha, 0.0f, 6000, 11);
    const auto on  = run_plant(plant_params(1.0, 0.0), kAlpha, 0.0f, 6000, 11);
    EXPECT_GT(off.falls, 20)
        << "the plant must genuinely keep toppling without the prior "
           "(else this test proves nothing); got " << off.falls;
    EXPECT_LT(on.falls, off.falls / 3)
        << "the prior must cut falls at least 3x (measured at lock-in: off 291, on 87 "
           "= 3.3x, the on-arm's residue being identification epochs and one "
           "de-identification collapse; off " << off.falls << ", on " << on.falls << ")";
    EXPECT_LT(off.longest_up, 500)
        << "no-control must never hold a long stretch (got " << off.longest_up << ")";
    EXPECT_GE(on.longest_up, 1500)
        << "the prior must produce a sustained balanced stretch — the capability, "
           "not a different stumble (got " << on.longest_up
        << " vs off " << off.longest_up << "); mean|lean| on " << on.mean_abs
        << " off " << off.mean_abs;
}

// =============================================================================
// 6b. The state-augmented self-model identifies the PLANT, not the feedback.
//     x_hat = A·y + b is structurally unable to represent a state with its own
//     dynamics: under closed-loop feedback A(lean,:) must absorb the pole through
//     the y↔x correlation and de-identifies (measured on the duck: decayed to zero
//     and flipped sign in every long run).  With Bx carrying the pole, A recovers
//     the true authority — signs and rough magnitudes — and keeps it.
// =============================================================================
TEST(StatePrior, StateModelIdentifiesThePole) {
    auto p = plant_params(1.0, 0.0);
    p["state_model_lr"] = 0.02;
    Fixture f(p);
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> nd(-0.02f, 0.02f);
    const float k[kMotors] = {0.050f, -0.030f, 0.020f};
    const float alpha = -0.02f;                      // pole 0.98 — the model must find it
    float lean = 0.05f;
    for (uint64_t t = 0; t < 4000; ++t) {
        f.bus.begin_tick(t);
        auto pt = std::make_shared<ogma::ProprioToken>();
        pt->values = Eigen::VectorXf::Zero(kStateN);
        pt->values[kLeanIdx] = lean;
        pt->sensor = "proprio";
        f.bus.publish("sp.p0", pt);
        f.m.tick(t);
        f.bus.end_tick();
        float push = 0.0f;
        for (int j = 0; j < kMotors; ++j) {
            const float a = f.accel(j);
            if (std::isfinite(a)) push += k[j] * a;
        }
        lean = std::clamp((1.0f + alpha) * lean + push + nd(rng), -1.5f, 1.5f);
    }
    auto snap = f.m.snapshot_state();
    auto const& lj = snap["legs"][0];
    ASSERT_TRUE(lj.contains("Bx")) << "the state model must be in the snapshot when enabled";
    const auto A  = lj["A"].get<std::vector<float>>();    // n x m, column-major
    const auto Bx = lj["Bx"].get<std::vector<float>>();   // n x n, column-major
    const float pole = Bx[size_t(kLeanIdx) * kStateN + kLeanIdx];
    EXPECT_GT(pole, 0.75f) << "Bx(lean,lean) must find the plant's pole (0.98), got " << pole;
    EXPECT_LT(pole, 1.10f) << "…and not overshoot it, got " << pole;
    for (int j = 0; j < kMotors; ++j) {
        const float aj = A[size_t(j) * kStateN + kLeanIdx];
        EXPECT_GT(aj * k[j], 0.0f)
            << "A(lean," << j << ") must carry the TRUE authority sign (k=" << k[j]
            << ", got " << aj << ") — the de-identification this term exists to prevent";
    }
}

// =============================================================================
// 6c. R1 regime banks: per-regime self-models UNMIX a mixture no single model
//     can represent.  The plant's authority FLIPS SIGN by regime (as a fallen
//     body's does vs a standing one); a synthetic RealityToken keys the banks.
//     Asserted: (i) empty regime_topic is byte-identical (the guard, hard form:
//     the token stream is live either way); (ii) with banks, each bank's A
//     learns ITS regime's authority sign — the two banks end OPPOSITE — while
//     the bankless model, fed the same mixture, cannot hold both (its lean
//     column's |value| is smaller than either bank's); (iii) switches counted.
// =============================================================================
TEST(StatePrior, RegimeBanksUnmixOpposedAuthorities) {
    auto mk = [](bool banks) {
        auto p = base_params();
        // Banks engage only after warmup (the R1 delay, measured in), so warmup
        // ends early and persistent explore noise supplies the identification
        // excitation instead (deterministic per seed, identical across arms).
        p["babble_ticks"]   = int64_t{200};
        p["babble_scale"]   = 0.25;
        p["explore_noise"]  = 0.3;
        p["state_model_lr"] = 0.02;
        if (banks) { p["regime_topic"] = std::string("sp.regime"); p["regime_banks"] = int64_t{3}; }
        return p;
    };
    auto run = [](Fixture& f, bool publish_token) {
        const float kA[kMotors] = {0.050f, -0.030f, 0.020f};   // regime 0 authority
        const float kB[kMotors] = {-0.050f, 0.030f, -0.020f};  // regime 7: SIGN-FLIPPED
        std::mt19937 rng(11);
        std::uniform_real_distribution<float> nd(-0.02f, 0.02f);
        float lean = 0.05f;
        for (uint64_t t = 0; t < 6000; ++t) {
            const bool regA = (t / 300) % 2 == 0;              // alternate every 300 ticks
            f.bus.begin_tick(t);
            if (publish_token) {
                auto rt = std::make_shared<ogma::RealityToken>();
                rt->winner_id = regA ? 0 : 7;
                f.bus.publish("sp.regime", rt);
            }
            auto pt = std::make_shared<ogma::ProprioToken>();
            pt->values = Eigen::VectorXf::Zero(kStateN);
            pt->values[kLeanIdx] = lean;
            pt->sensor = "proprio";
            f.bus.publish("sp.p0", pt);
            f.m.tick(t);
            f.bus.end_tick();
            float push = 0.0f;
            const float* k = regA ? kA : kB;
            for (int j = 0; j < kMotors; ++j) {
                const float a = f.accel(j);
                if (std::isfinite(a)) push += k[j] * a;
            }
            lean = std::clamp(0.98f * lean + push + nd(rng), -1.5f, 1.5f);
        }
    };

    // (i) the guard, hard form: token live on the bus, socket not configured.
    Fixture N(mk(false)), Z(mk(false)), B(mk(true));
    {
        std::mt19937 rng(11);
        // N gets no token, Z gets the token with no socket — must match N exactly.
        // (run() publishes per its flag; reuse it.)
    }
    run(N, false); run(Z, true);
    double maxdiff = 0.0;
    for (int j = 0; j < kMotors; ++j)
        maxdiff = std::max(maxdiff, double(std::fabs(N.accel(j) - Z.accel(j))));
    EXPECT_LT(maxdiff, 1e-9)
        << "a live token with no regime_topic configured must change nothing";

    // (ii) banks unmix.
    run(B, true);
    auto snap = B.m.snapshot_state();
    auto const& lj = snap["legs"][0];
    ASSERT_TRUE(lj.contains("banks")) << "banks must be in the snapshot when configured";
    const auto banks = lj["banks"];
    ASSERT_GE(banks.size(), 2u);
    // slots claimed first-seen: regime 0 -> slot 0, regime 7 -> slot 1.
    auto leanrow = [&](nlohmann::json const& bj, int j) {
        const auto A = bj["A"].get<std::vector<float>>();     // n x m col-major
        return A[size_t(j) * kStateN + kLeanIdx];
    };
    ASSERT_TRUE(banks[0].contains("A") && banks[1].contains("A"));
    EXPECT_GT(leanrow(banks[0], 0), 0.0f) << "bank 0 must learn regime A's +authority on motor 0";
    EXPECT_LT(leanrow(banks[1], 0), 0.0f) << "bank 1 must learn regime B's −authority on motor 0";
    EXPECT_GT(leanrow(banks[0], 0) - leanrow(banks[1], 0), 0.02f)
        << "the banks must be separated, not both near zero";
    // The bankless mixture CANNOT hold both signs at once.
    auto snapN = N.m.snapshot_state();
    const auto AN = snapN["legs"][0]["A"].get<std::vector<float>>();
    const float mixed = AN[size_t(0) * kStateN + kLeanIdx];
    EXPECT_LT(std::fabs(mixed),
              std::max(std::fabs(leanrow(banks[0], 0)), std::fabs(leanrow(banks[1], 0))))
        << "the shared model's lean authority must be smaller than the better bank's — "
           "it is fitting a mixture whose true values have opposite signs";

    // (iii) the consumer counter.
    EXPECT_GT(B.m.diag_snapshot()["bank_switches"].get<int64_t>(), 10);
}

// =============================================================================
// 6d. R2 regime-keyed calm: the bank that satisfies the prior anneals toward
//     quiet; the bank that cannot keeps full drive.  Regime A's plant is
//     controllable near the target; regime B is dragged to +0.5 by a drift the
//     motors cannot cancel.  Nothing labels the regimes — the per-bank error
//     statistics decide.  (The five continuous keys all failed storm-coupled;
//     this is the discrete replacement, gated by state_prior_calm_mode.)
// =============================================================================
TEST(StatePrior, RegimeKeyedCalmQuietsTheSatisfiedRegime) {
    auto p = base_params();
    p["babble_ticks"]   = int64_t{200};
    p["babble_scale"]   = 0.25;
    p["explore_noise"]  = 0.10;
    p["state_model_lr"] = 0.02;
    p["regime_topic"]   = std::string("sp.regime");
    p["regime_banks"]   = int64_t{3};
    p["state_prior_indices"] = std::vector<double>{-1.0};
    p["state_prior_targets"] = std::vector<double>{0.0};
    p["state_prior_gain"]    = 1.0;
    p["state_prior_calm"]      = 1.0;
    p["state_prior_calm_mode"] = 1.0;
    Fixture f(p);

    const float k[kMotors] = {0.050f, -0.030f, 0.020f};
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> nd(-0.02f, 0.02f);
    float lean = 0.05f;
    double multA = 0.0, multB = 0.0; int nA = 0, nB = 0;
    for (uint64_t t = 0; t < 12000; ++t) {
        const bool regA = (t / 400) % 2 == 0;
        f.bus.begin_tick(t);
        auto rt = std::make_shared<ogma::RealityToken>();
        rt->winner_id = regA ? 0 : 7;
        f.bus.publish("sp.regime", rt);
        auto pt = std::make_shared<ogma::ProprioToken>();
        pt->values = Eigen::VectorXf::Zero(kStateN);
        pt->values[kLeanIdx] = lean;
        pt->sensor = "proprio";
        f.bus.publish("sp.p0", pt);
        f.m.tick(t);
        f.bus.end_tick();
        float push = 0.0f;
        for (int j = 0; j < kMotors; ++j) {
            const float a = f.accel(j);
            if (std::isfinite(a)) push += k[j] * a;
        }
        // Regime A: STRONGLY self-stable near 0 (leak 0.75 — the prior is
        // satisfiable and stays satisfied).  Regime B: slow plant dragged toward
        // +0.5 by a drift beyond the motors' authority.  (First version gave A
        // the same slow leak and exploration noise kept |lean|_A ≈ |lean|_B —
        // the statistics could not separate what the plant did not separate.)
        const float leak  = regA ? 0.75f : 0.90f;
        const float drift = regA ? 0.0f : 0.05f;
        lean = std::clamp(leak * lean + push + drift + nd(rng), -1.5f, 1.5f);
        // Late run, and only each phase's SETTLED half: the ratchet needs ~100
        // ticks to descend after a bank switch, and averaging the transient in
        // would test the smoothing, not the key.
        if (t > 8000 && (t % 400) >= 200) {
            const double m2 = f.m.diag_snapshot()["state_prior_calm_mult"].get<double>();
            if (regA) { multA += m2; ++nA; } else { multB += m2; ++nB; }
        }
    }
    multA /= nA; multB /= nB;
    EXPECT_LT(multA, 0.45) << "the satisfied regime must anneal toward quiet (got " << multA << ")";
    EXPECT_GT(multB, 0.70) << "the violated regime must keep its drive (got " << multB << ")";
    EXPECT_LT(multA, multB - 0.25)
        << "the two regimes must be clearly separated (A " << multA << " vs B " << multB << ")";
}

// A disabled-by-default probe: watch the on-arm learn (run with
//   --gtest_also_run_disabled_tests --gtest_filter='*TraceProbe*').
TEST(StatePrior, DISABLED_TraceProbe) {
    run_plant(plant_params(1.0, 0.0), 0.05f, 0.0f, 6000, 11, /*trace=*/true);
}
TEST(StatePrior, DISABLED_TraceProbeDirection) {
    run_plant(plant_params(1.0, +0.4), -0.02f, 0.0f, 4000, 7, /*trace=*/true);
}

// =============================================================================
// 7. Hot-param round trip.
// =============================================================================
// =============================================================================
// 8. Gate/objective separation (the R4-neck lesson).  consolidate_n reads only
//    the first N prior indices, so a reach-type target with a large standing
//    error (the head-CoM origin) cannot hold consolidation hostage; and
//    consolidate_spares_prior keeps the objective's pull alive after the HK
//    learning has been annealed away.
// =============================================================================

namespace {
ParamMap consol_params(double consolidate_n) {
    ParamMap p = base_params();
    p["state_prior_gain"]    = 1.0;
    p["state_prior_lr"]      = 0.1;
    // FIRST index = the balance term (lean, satisfied at ~0.02); SECOND = a
    // reach target the scripted state can never satisfy (|e| ~ 1.5).
    p["state_prior_indices"] = std::vector<double>{double(kLeanIdx), 0.0};
    p["state_prior_targets"] = std::vector<double>{0.0, 1.5};
    p["consolidate_gain"]    = 1.0;
    p["consolidate_n"]       = consolidate_n;
    return p;
}
}  // namespace

TEST(StatePrior, GateSubsetArmsDespiteReachError) {
    Fixture subset(consol_params(1.0));   // gate = the balance index only
    Fixture legacy(consol_params(0.0));   // gate = mean over ALL prior indices
    for (uint64_t t = 1; t <= 3500; ++t) {
        subset.run_tick(t, 0.02f);        // balanced: tiny, nonzero lean
        legacy.run_tick(t, 0.02f);
    }
    const float c_subset = subset.m.diag_snapshot()["consolidate_c"].get<float>();
    const float c_legacy = legacy.m.diag_snapshot()["consolidate_c"].get<float>();
    EXPECT_GT(c_subset, 0.5f) << "balance satisfied: the subset gate must arm";
    EXPECT_LT(c_legacy, 0.05f) << "the unreachable reach term must block the legacy gate";
    const float gate = subset.m.diag_snapshot()["consolidate_gate"].get<float>();
    EXPECT_GT(gate, 0.0f);
    EXPECT_LT(gate, 0.15f) << "gate EMA reads the balance subset, not the reach error";
}

TEST(StatePrior, GateWeightScalesOnlyTheGateSubsetsDescent) {
    // 1 = byte-identical to unset (the gain-0 guard), 3 acts, and with no gate
    // subset (consolidate_n 0) the weight has nothing to scale and stays inert.
    ParamMap unit = consol_params(1.0);  unit["state_prior_gate_weight"] = 1.0;
    ParamMap heavy = consol_params(1.0); heavy["state_prior_gate_weight"] = 3.0;
    ParamMap nogate = consol_params(0.0); nogate["state_prior_gate_weight"] = 3.0;
    Fixture a(consol_params(1.0)), b(unit), c(heavy), d(consol_params(0.0)), e(nogate);
    float max_bc = 0.0f;
    for (uint64_t t = 1; t <= 900; ++t) {
        const float lean = wobble(t);
        a.run_tick(t, lean); b.run_tick(t, lean); c.run_tick(t, lean);
        d.run_tick(t, lean); e.run_tick(t, lean);
        for (int j = 0; j < kMotors; ++j) {
            ASSERT_EQ(a.accel(j), b.accel(j)) << "weight 1 must be byte-identical (t=" << t << ")";
            ASSERT_EQ(d.accel(j), e.accel(j)) << "no gate subset: the weight is inert (t=" << t << ")";
            max_bc = std::max(max_bc, std::fabs(b.accel(j) - c.accel(j)));
        }
    }
    EXPECT_GT(max_bc, 1e-4f) << "weight 3 must change the controller the attitude term writes";
}

TEST(StatePrior, SparesPriorKeepsPullingAfterConsolidation) {
    ParamMap keep = consol_params(1.0);
    keep["consolidate_spares_prior"] = 1.0;
    Fixture spared(keep);
    Fixture frozen(consol_params(1.0));
    // Before consolidation can begin (calm needs 1500 ticks) the flag is inert:
    // the two arms must be byte-identical.
    for (uint64_t t = 1; t <= 1000; ++t) {
        spared.run_tick(t, 0.02f);
        frozen.run_tick(t, 0.02f);
        if (t % 250 == 0)
            for (int j = 0; j < kMotors; ++j)
                ASSERT_EQ(spared.accel(j), frozen.accel(j)) << "inert before c > 0";
    }
    // Consolidate both, then the spared arm's prior keeps descending toward the
    // reach target while the frozen arm's pull anneals away with everything else.
    for (uint64_t t = 1001; t <= 4500; ++t) {
        spared.run_tick(t, 0.02f);
        frozen.run_tick(t, 0.02f);
    }
    EXPECT_GT(spared.m.diag_snapshot()["consolidate_c"].get<float>(), 0.5f);
    EXPECT_GT(frozen.m.diag_snapshot()["consolidate_c"].get<float>(), 0.5f);
    float max_diff = 0.0f;
    for (int j = 0; j < kMotors; ++j)
        max_diff = std::max(max_diff, std::fabs(spared.accel(j) - frozen.accel(j)));
    EXPECT_GT(max_diff, 1e-4f) << "the spared prior must still be writing the controller";
}

TEST(StatePrior, ReachDormantUntilConsolidatedThenEngages) {
    ParamMap p = consol_params(1.0);           // gate = the balance index (lean)
    p["consolidate_spares_prior"] = 1.0;
    p["consolidate_reach"]        = 1.0;       // index 1 (the reach target) scales with c
    Fixture f(p);
    for (uint64_t t = 1; t <= 1400; ++t) f.run_tick(t, 0.02f);
    auto d = f.m.diag_snapshot();
    EXPECT_EQ(d["reach_lw"].get<float>(), 0.0f)
        << "the reach term must be dormant before consolidation (c = 0)";
    for (uint64_t t = 1401; t <= 4500; ++t) f.run_tick(t, 0.02f);
    d = f.m.diag_snapshot();
    EXPECT_GT(d["consolidate_c"].get<float>(), 0.5f);
    EXPECT_GT(d["reach_lw"].get<float>(), 0.01f)
        << "consolidated: the reach term must engage at lw*c";
}

TEST(StatePrior, Mode3ReachBiasWaitsForConsolidationNoChaosWindup) {
    ParamMap p = consol_params(1.0);
    p["consolidate_spares_prior"] = 1.0;
    p["consolidate_reach"]        = 3.0;   // C-pull live from tick 0; bias via c-gated hr
    Fixture f(p);
    for (uint64_t t = 1; t <= 1400; ++t) f.run_tick(t, 0.02f);
    auto d = f.m.diag_snapshot();
    EXPECT_EQ(d["hr_max"].get<float>(), 0.0f)
        << "pre-consolidation (c = 0): the reach bias must accumulate NOTHING";
    EXPECT_GT(d["reach_lw"].get<float>(), 0.01f)
        << "mode 3's C-pull is live from tick 0 (the co-adaptation half)";
    for (uint64_t t = 1401; t <= 4500; ++t) f.run_tick(t, 0.02f);
    d = f.m.diag_snapshot();
    EXPECT_GT(d["consolidate_c"].get<float>(), 0.5f);
    EXPECT_GT(d["hr_max"].get<float>(), 1e-4f)
        << "consolidated: the reach bias must be accumulating toward the target";
}

TEST(StatePrior, Mode4ReachScalesWithBalanceSatisfaction) {
    ParamMap p = consol_params(1.0);
    p["consolidate_spares_prior"] = 1.0;
    p["consolidate_reach"]        = 4.0;   // pull ∝ balance-subset satisfaction
    Fixture balanced(p);
    Fixture toppling(p);
    for (uint64_t t = 1; t <= 800; ++t) {
        balanced.run_tick(t, 0.02f);       // balance satisfied → pull engaged
        toppling.run_tick(t, 0.60f);       // balance violated → pull off
    }
    const float lw_bal = balanced.m.diag_snapshot()["reach_lw"].get<float>();
    const float lw_top = toppling.m.diag_snapshot()["reach_lw"].get<float>();
    EXPECT_GT(lw_bal, 0.05f) << "satisfied balance must engage the reach pull";
    EXPECT_EQ(lw_top, 0.0f) << "violated balance (gate ema >= 0.15) must zero the pull";
}

TEST(StatePrior, Mode5BiasOnlyReachIsControlUntilConsolidatedThenWalks) {
    // Twin WITHOUT the reach index at all — the control this mode must match
    // bit-for-bit while unconsolidated.
    ParamMap ctrl = consol_params(1.0);
    ctrl["state_prior_indices"] = std::vector<double>{double(kLeanIdx)};
    ctrl["state_prior_targets"] = std::vector<double>{0.0};
    ParamMap m5 = consol_params(1.0);
    m5["consolidate_reach"]    = 5.0;
    m5["consolidate_reach_lr"] = 0.01;
    Fixture a(ctrl), b(m5);
    // Hold the balance term UNSATISFIED so c stays 0: every path of the listed
    // reach index must be exactly zero.
    for (uint64_t t = 1; t <= 2500; ++t) {
        a.run_tick(t, 0.60f);
        b.run_tick(t, 0.60f);
        if (t % 500 == 0)
            for (int j = 0; j < kMotors; ++j)
                ASSERT_EQ(a.accel(j), b.accel(j))
                    << "unconsolidated mode 5 must be BIT-IDENTICAL to the no-reach twin";
    }
    EXPECT_EQ(b.m.diag_snapshot()["hr_max"].get<float>(), 0.0f);
    // Now satisfy the balance: consolidation arms and the walk begins.
    for (uint64_t t = 2501; t <= 6500; ++t) b.run_tick(t, 0.02f);
    auto d = b.m.diag_snapshot();
    EXPECT_GT(d["consolidate_c"].get<float>(), 0.5f);
    EXPECT_GT(d["hr_max"].get<float>(), 1e-5f)
        << "consolidated: the bias walk must be moving toward the reach target";
}

TEST(StatePrior, ControllerBanksKeepPermanenceRegimeLocal) {
    ParamMap p = consol_params(0.0);
    p["state_prior_indices"] = std::vector<double>{double(kLeanIdx)};
    p["state_prior_targets"] = std::vector<double>{0.0};
    p["babble_ticks"]   = int64_t{200};
    p["regime_topic"]   = std::string("sp.regime");
    p["regime_banks"]   = int64_t{3};
    p["regime_c_banks"] = 1.0;
    Fixture f(p);
    auto tick = [&](uint64_t t, int winner, float lean) {
        f.bus.begin_tick(t);
        auto rt = std::make_shared<ogma::RealityToken>();
        rt->winner_id = winner;
        f.bus.publish("sp.regime", rt);
        auto pt = std::make_shared<ogma::ProprioToken>();
        pt->values = Eigen::VectorXf::Zero(kStateN);
        const double ph = 0.15 * double(t);
        for (int j = 0; j < kMotors; ++j) {
            pt->values[3 * j + 0] = float(0.30 * std::sin(ph + j));
            pt->values[3 * j + 1] = float(0.20 * std::cos(ph + j));
            pt->values[3 * j + 2] = float(0.30 * 0.15 * std::cos(ph + j));
        }
        pt->values[kLeanIdx] = lean;
        pt->sensor = "proprio";
        f.bus.publish("sp.p0", pt);
        f.m.tick(t);
        f.bus.end_tick();
    };
    uint64_t t = 1;
    for (; t <= 4000; ++t) tick(t, 0, 0.02f);        // regime 0: satisfied, consolidates
    const float c_A = f.m.diag_snapshot()["consolidate_c"].get<float>();
    EXPECT_GT(c_A, 0.5f) << "regime 0 must earn consolidation";
    for (uint64_t e = 0; e < 600; ++e, ++t) tick(t, 7, 0.40f);   // regime 7: fresh, unsatisfied
    const float c_B = f.m.diag_snapshot()["consolidate_c"].get<float>();
    EXPECT_LT(c_B, 0.05f) << "a fresh regime's permanence starts at 0 — earned, not inherited";
    for (uint64_t e = 0; e < 50; ++e, ++t) tick(t, 0, 0.02f);    // back to regime 0
    const float c_back = f.m.diag_snapshot()["consolidate_c"].get<float>();
    EXPECT_GT(c_back, 0.5f * c_A)
        << "returning to regime 0 must restore ITS earned consolidation";
}

TEST(StatePrior, RegimeDwellSuppressesFlickerNotTransitions) {
    auto mk = [](double dwell) {
        ParamMap p = base_params();
        p["babble_ticks"]   = int64_t{100};
        p["regime_topic"]   = std::string("sp.regime");
        p["regime_banks"]   = int64_t{3};
        p["regime_dwell"]   = dwell;
        return p;
    };
    auto drive = [](Fixture& f) {
        auto tick = [&](uint64_t t, int winner) {
            f.bus.begin_tick(t);
            auto rt = std::make_shared<ogma::RealityToken>();
            rt->winner_id = winner;
            f.bus.publish("sp.regime", rt);
            auto pt = std::make_shared<ogma::ProprioToken>();
            pt->values = Eigen::VectorXf::Zero(kStateN);
            pt->values[kLeanIdx] = wobble(t);
            pt->sensor = "proprio";
            f.bus.publish("sp.p0", pt);
            f.m.tick(t);
            f.bus.end_tick();
        };
        uint64_t t = 1;
        for (; t <= 400; ++t) tick(t, 0);                       // settle in regime 0
        for (; t <= 900; ++t) tick(t, (t % 10 < 5) ? 0 : 7);    // 5-tick flicker, 10 Hz
        for (; t <= 1100; ++t) tick(t, 7);                      // a REAL transition
        return f.m.diag_snapshot()["bank_switches"].get<int64_t>();
    };
    Fixture raw(mk(0.0)), sticky(mk(25.0));
    const auto sw_raw = drive(raw), sw_sticky = drive(sticky);
    EXPECT_GT(sw_raw, int64_t{50}) << "dwell 0 must chase every flicker (the R6a disease)";
    EXPECT_LE(sw_sticky, int64_t{4}) << "dwell 25 must ignore 5-tick flicker";
    EXPECT_GE(sw_sticky, int64_t{2}) << "…but still take the sustained transition";
}

TEST(StatePrior, ThreeStateRatchetHoldsThroughACaughtStumble) {
    auto mk = [](double hold) {
        ParamMap p = consol_params(1.0);
        p["consolidate_hold"] = hold;
        return p;
    };
    Fixture legacy(mk(0.0)), three(mk(1.0));
    uint64_t t = 1;
    for (; t <= 4000; ++t) { legacy.run_tick(t, 0.02f); three.run_tick(t, 0.02f); }
    const float cL0 = legacy.m.diag_snapshot()["consolidate_c"].get<float>();
    const float cT0 = three.m.diag_snapshot()["consolidate_c"].get<float>();
    ASSERT_GT(cL0, 0.5f); ASSERT_GT(cT0, 0.5f);
    // A caught WOBBLE (no reset event, error below the instant threshold): the
    // three-state must hold what was earned.
    for (uint64_t e = 0; e < 40; ++e, ++t) three.run_tick(t, 0.25f);   // brief: EMA stays under 0.15
    EXPECT_GT(three.m.diag_snapshot()["consolidate_c"].get<float>(), 0.9f * cT0)
        << "a caught wobble (no reset) must not un-earn the stance";
    // A REAL FALL (the harness's reset event — v3: the one signal the frozen
    // meters cannot hide): a bounded slice, never the legacy wipe.
    auto send_reset = [&](Fixture& f, uint64_t tk) {
        f.bus.begin_tick(tk);
        auto ev = std::make_shared<ogma::EnvEvent>();
        f.bus.publish("events.reset", ev);
        f.m.tick(tk);
        f.bus.end_tick();
    };
    send_reset(legacy, t); send_reset(three, t); ++t;
    for (uint64_t e = 0; e < 1000; ++e, ++t) { legacy.run_tick(t, 0.02f); three.run_tick(t, 0.02f); }
    const float cL1 = legacy.m.diag_snapshot()["consolidate_c"].get<float>();
    const float cT1 = three.m.diag_snapshot()["consolidate_c"].get<float>();
    EXPECT_LT(cL1, 0.05f) << "legacy must wipe c through the calm-blocked window";
    EXPECT_GT(cT1, 0.20f * cT0) << "v3: a real fall costs a bounded slice, not the wipe";
    EXPECT_LT(cT1, 0.75f * cT0) << "v3: but a real fall MUST cost something";
    // A genuine regime change (sustained unsatisfied): three-state must still
    // restore plasticity at the legacy rate.
    for (uint64_t e = 0; e < 300; ++e, ++t) three.run_tick(t, 0.60f);
    EXPECT_LT(three.m.diag_snapshot()["consolidate_c"].get<float>(), 0.15f)
        << "sustained real error must decay c fast — the safety re-arm stays";
}

TEST(StatePrior, ConsolidationRestsTheEfferenceFeedback) {
    ParamMap base = consol_params(1.0);
    ParamMap rest = consol_params(1.0);
    rest["consolidate_rests_act"] = 1.0;
    Fixture a(base), b(rest);
    // Identical while unconsolidated (c = 0 → the act view is unscaled).
    for (uint64_t t = 1; t <= 1000; ++t) {
        a.run_tick(t, 0.02f); b.run_tick(t, 0.02f);
        if (t % 250 == 0)
            for (int j = 0; j < kMotors; ++j)
                ASSERT_EQ(a.accel(j), b.accel(j)) << "inert before c > 0";
    }
    // Consolidate both; the rested arm's command must stop hearing the act
    // elements while the plain arm keeps its full efference feedback.
    for (uint64_t t = 1001; t <= 4500; ++t) { a.run_tick(t, 0.02f); b.run_tick(t, 0.02f); }
    EXPECT_GT(a.m.diag_snapshot()["consolidate_c"].get<float>(), 0.5f);
    EXPECT_GT(b.m.diag_snapshot()["consolidate_c"].get<float>(), 0.5f);
    float diff = 0.0f;
    for (int j = 0; j < kMotors; ++j)
        diff = std::max(diff, std::fabs(a.accel(j) - b.accel(j)));
    EXPECT_GT(diff, 1e-5f)
        << "consolidated: the rested command must differ (its act input is scaled away)";
}

TEST(StatePrior, HotParamRoundTrip) {
    Fixture f(base_params());
    f.m.on_param_change("state_prior_gain", ParamValue{0.6});
    f.m.on_param_change("state_prior_indices", ParamValue{std::vector<double>{-1.0, 4.0}});
    f.m.on_param_change("state_prior_targets", ParamValue{std::vector<double>{0.0, 0.2}});
    auto cp = f.m.current_params();
    EXPECT_DOUBLE_EQ(std::get<double>(cp.at("state_prior_gain")), 0.6);
    EXPECT_EQ(std::get<std::vector<double>>(cp.at("state_prior_indices")),
              (std::vector<double>{-1.0, 4.0}));
    EXPECT_EQ(std::get<std::vector<double>>(cp.at("state_prior_targets")),
              (std::vector<double>{0.0, 0.2}));
}

// =============================================================================
// 8. state_prior_isolate — C's columns held to the prior's own indices (W5).
//    Off must be invisible; on must act; and diag must SAY how many columns it
//    kept, because a lesion nobody can read back is a lesion nobody can trust.
// =============================================================================
TEST(StatePrior, IsolateHoldsCToThePriorsOwnColumns) {
    auto pn = base_params();                                // N: prior live, param absent
    pn["state_prior_indices"] = std::vector<double>{-1.0};
    pn["state_prior_targets"] = std::vector<double>{0.0};
    pn["state_prior_gain"]    = 0.8;
    auto pz = pn; pz["state_prior_isolate"] = 0.0;          // Z: configured, off
    auto pi = pn; pi["state_prior_isolate"] = 1.0;          // I: the lesion

    Fixture N(pn), Z(pz), I(pi);
    double maxdiff_zn = 0.0, maxdiff_iz = 0.0;
    for (uint64_t t = 0; t < 300; ++t) {
        const float lean = wobble(t);
        N.run_tick(t, lean); Z.run_tick(t, lean); I.run_tick(t, lean);
        if (t < 12) continue;                               // warmup: the babble owns the command
        for (int j = 0; j < kMotors; ++j) {
            maxdiff_zn = std::max(maxdiff_zn, double(std::fabs(N.accel(j) - Z.accel(j))));
            maxdiff_iz = std::max(maxdiff_iz, double(std::fabs(I.accel(j) - Z.accel(j))));
        }
    }
    EXPECT_LT(maxdiff_zn, 1e-6)
        << "state_prior_isolate=0 must be byte-identical to the param being absent (the gain-0 guard)";
    EXPECT_GT(maxdiff_iz, 1e-4)
        << "the lesion changed nothing — it is not reaching C (the R47 arm would have been a false null)";

    EXPECT_EQ(Z.m.diag_lite()["spIso"].get<int>(), -1) << "off must read as off, not as 0 columns kept";
    EXPECT_EQ(I.m.diag_lite()["spIso"].get<int>(), 1)
        << "one prior index -> exactly one surviving column; the count is the read-back a sweep asserts on";
}

// =============================================================================
// 9. state_prior_step_gain — the command computed from the model, not accumulated
//    into C (W5 fork item (b)).  The sign control matters most here: with the
//    Gauss-Newton descent OFF (state_prior_lr 0) the step is the only thing that
//    can move the plant, so a pull toward the target is the step's own doing.
// =============================================================================
TEST(StatePrior, ModelImpliedStepClosesTheErrorItself) {
    auto pn = base_params();                                // N: prior configured, step absent
    pn["state_prior_indices"] = std::vector<double>{-1.0};
    pn["state_prior_targets"] = std::vector<double>{0.0};
    pn["state_prior_gain"]    = 0.8;
    auto pz = pn; pz["state_prior_step_gain"] = 0.0;        // Z: configured, off
    auto ps = pn; ps["state_prior_step_gain"] = 1.0;        // S: the step

    Fixture N(pn), Z(pz), S(ps);
    double maxdiff_zn = 0.0, maxdiff_sz = 0.0;
    for (uint64_t t = 0; t < 300; ++t) {
        const float lean = wobble(t);
        N.run_tick(t, lean); Z.run_tick(t, lean); S.run_tick(t, lean);
        if (t < 12) continue;
        for (int j = 0; j < kMotors; ++j) {
            maxdiff_zn = std::max(maxdiff_zn, double(std::fabs(N.accel(j) - Z.accel(j))));
            maxdiff_sz = std::max(maxdiff_sz, double(std::fabs(S.accel(j) - Z.accel(j))));
        }
    }
    EXPECT_LT(maxdiff_zn, 1e-6) << "state_prior_step_gain=0 must be byte-identical to the param being absent";
    EXPECT_GT(maxdiff_sz, 1e-4) << "the step changed nothing — it is not reaching the command";
    EXPECT_LT(Z.m.diag_lite()["spStep"].get<float>(), 0.0f) << "off must read as -1, not as a zero step";
    EXPECT_GT(S.m.diag_lite()["spStep"].get<float>(), 0.0f) << "a live step must read back its own size";

    // The sign control, with the descent OFF so only the step can act: a +target and a
    // −target must drive the plant's own lean to opposite sides.  A lever whose sign does
    // not matter is not a mechanism (the v2 plan's rule 4, as test 5 applies it to part 2).
    auto plant = [](double target) {
        auto p = base_params();
        p["state_prior_indices"]  = std::vector<double>{-1.0};
        p["state_prior_targets"]  = std::vector<double>{target};
        p["state_prior_gain"]     = 1.0;
        p["state_prior_lr"]       = 0.0;                    // part 2 silenced: the step acts alone
        p["ctrl_lr"]              = 0.0;                    // and so is HK
        p["state_prior_step_gain"] = 1.0;
        Fixture f(p);
        float lean = 0.0f;
        for (uint64_t t = 0; t < 400; ++t) {
            f.run_tick(t, lean);
            if (t >= 12) lean += 0.02f * f.accel(0);        // a lean the first motor drives
            lean = std::clamp(lean, -2.0f, 2.0f);
        }
        return lean;
    };
    const float up = plant(+0.6), down = plant(-0.6);
    EXPECT_GT(up, down + 0.05f)
        << "the model-implied step must pull toward its target: +0.6 gave " << up
        << " and -0.6 gave " << down << " (sign or solve inverted)";
}

// =============================================================================
// 10. state_prior_weights (2026-10-01, the lean's settle): weight 1 is byte-identical to no weights at all (the
//     guard), and weight 0 on the only index takes the prior's descent and step out (the command differs from w = 1).
// =============================================================================
TEST(StatePrior, WeightsOneIsIdenticalZeroSilences) {
    auto pe = base_params();                                // E: prior, no weights
    pe["state_prior_indices"]   = std::vector<double>{-1.0};
    pe["state_prior_targets"]   = std::vector<double>{0.0};
    pe["state_prior_gain"]      = 0.8;
    pe["state_prior_step_gain"] = 1.0;
    auto p1 = pe; p1["state_prior_weights"] = std::vector<double>{1.0};
    auto p0 = pe; p0["state_prior_weights"] = std::vector<double>{0.0};
    Fixture E(pe), W1(p1), W0(p0);
    double d1 = 0.0, d0 = 0.0;
    for (uint64_t t = 0; t < 300; ++t) {
        const float lean = wobble(t);
        E.run_tick(t, lean); W1.run_tick(t, lean); W0.run_tick(t, lean);
        for (int j = 0; j < kMotors; ++j) {
            d1 = std::max(d1, double(std::fabs(E.accel(j) - W1.accel(j))));
            d0 = std::max(d0, double(std::fabs(E.accel(j) - W0.accel(j))));
        }
    }
    EXPECT_EQ(d1, 0.0) << "weight 1 must be byte-identical to no weights";
    EXPECT_GT(d0, 1e-4) << "weight 0 must take the index's descent and step out";
}

// =============================================================================
// 11. state_grow_at (2026-10-01, grow on restore): a snapshot of a 10-element state restored into a module fed
//     12-element frames grows at the index given -- the old model rows kept at their shifted positions, the new rows
//     zero (unidentified) -- and without the param the wider frames are dropped (the old contract).
// =============================================================================
namespace {
void tick_width(ogma::InProcessBus& bus, ogma::MotorEPMv2& m, uint64_t t, int width, int lean_idx, float lean,
                float extra) {
    bus.begin_tick(t);
    auto pt = std::make_shared<ogma::ProprioToken>();
    pt->values = Eigen::VectorXf::Zero(width);
    const double ph = 0.15 * double(t);
    for (int j = 0; j < kMotors; ++j) {
        pt->values[3 * j + 0] = float(0.30 * std::sin(ph + j));
        pt->values[3 * j + 1] = float(0.20 * std::cos(ph + j));
        pt->values[3 * j + 2] = float(0.30 * 0.15 * std::cos(ph + j));
    }
    for (int k = 3 * kMotors; k < lean_idx; ++k) pt->values[k] = extra;   // the grown elements
    pt->values[lean_idx] = lean;
    pt->sensor = "proprio";
    bus.publish("sp.p0", pt);
    m.tick(t);
    bus.end_tick();
}
}  // namespace

TEST(StatePrior, GrowOnRestoreInsertsUnidentifiedRows) {
    auto p = base_params();
    p["state_model_lr"] = 0.05;                               // Bx present, so its rows and columns grow too
    Fixture F(p);
    for (uint64_t t = 0; t < 200; ++t) F.run_tick(t, wobble(t));
    ASSERT_EQ(F.m.state_dim(), kStateN);
    const nlohmann::json snap = F.m.snapshot_state();
    std::vector<std::vector<double>> A0(kStateN, std::vector<double>(kMotors));
    for (int r = 0; r < kStateN; ++r)
        for (int j = 0; j < kMotors; ++j) A0[size_t(r)][size_t(j)] = F.m.authority_cell(r, j);

    // the restored module learns nothing (model_lr 0, state_model_lr 0) so the grown model can be read exactly
    auto pg = p; pg["model_lr"] = 0.0; pg["state_model_lr"] = 0.0; pg["state_grow_at"] = int64_t{3 * kMotors};
    ogma::InProcessBus bus; ogma::MotorEPMv2 g; g.set_id("grown"); g.on_setup(&bus, pg); g.restore_state(snap);
    const int W = kStateN + 2;
    for (uint64_t t = 200; t < 205; ++t) tick_width(bus, g, t, W, W - 1, wobble(t), 0.3f);
    ASSERT_EQ(g.state_dim(), W) << "the state must have grown";
    for (int j = 0; j < kMotors; ++j) {
        for (int r = 0; r < 3 * kMotors; ++r) EXPECT_EQ(g.authority_cell(r, j), A0[size_t(r)][size_t(j)]) << r;
        EXPECT_EQ(g.authority_cell(3 * kMotors, j), 0.0);        // the new rows: unidentified
        EXPECT_EQ(g.authority_cell(3 * kMotors + 1, j), 0.0);
        EXPECT_EQ(g.authority_cell(W - 1, j), A0[size_t(kLeanIdx)][size_t(j)]);   // the lean row, shifted by two
    }
    EXPECT_TRUE(std::isfinite(double(std::dynamic_pointer_cast<const ogma::ActionOut>(bus.last_value("sp.a0"))->accel)));

    // without state_grow_at the wider frames are dropped: the state keeps its snapshot's width
    auto pd = p; pd["model_lr"] = 0.0;
    ogma::InProcessBus bus2; ogma::MotorEPMv2 d; d.set_id("dropped"); d.on_setup(&bus2, pd); d.restore_state(snap);
    for (uint64_t t = 200; t < 205; ++t) tick_width(bus2, d, t, W, W - 1, wobble(t), 0.3f);
    EXPECT_EQ(d.state_dim(), kStateN);
}

// =============================================================================
// 12. state_prior_gated_by (2026-10-01, the pace gate): the ungated sentinel is byte-identical to no gate; a gate
//     on an element that keeps changing (the scripted joint 0, a sinusoid) lowers the index's precision -- the
//     command differs and the diag's gate reads below 1.
// =============================================================================
TEST(StatePrior, PaceGateUngatedIdenticalChangingElementGates) {
    auto pe = base_params();
    pe["state_prior_indices"]   = std::vector<double>{-1.0};
    pe["state_prior_targets"]   = std::vector<double>{0.0};
    pe["state_prior_gain"]      = 0.8;
    pe["state_prior_step_gain"] = 1.0;
    auto pu = pe; pu["state_prior_gated_by"] = std::vector<double>{9999.0};
    auto pg = pe; pg["state_prior_gated_by"] = std::vector<double>{0.0};
    Fixture E(pe), U(pu), G(pg);
    double du = 0.0, dg = 0.0;
    for (uint64_t t = 0; t < 400; ++t) {
        const float lean = wobble(t);
        E.run_tick(t, lean); U.run_tick(t, lean); G.run_tick(t, lean);
        for (int j = 0; j < kMotors; ++j) {
            du = std::max(du, double(std::fabs(E.accel(j) - U.accel(j))));
            dg = std::max(dg, double(std::fabs(E.accel(j) - G.accel(j))));
        }
    }
    EXPECT_EQ(du, 0.0) << "an ungated index must be byte-identical to no gate";
    EXPECT_GT(dg, 1e-4) << "a gate on a changing element must change the command";
    const auto diag = G.m.diag_snapshot();
    ASSERT_TRUE(diag.contains("state_prior_gate"));
    EXPECT_LT(diag["state_prior_gate"].get<double>(), 1.0);
    EXPECT_GE(diag["state_prior_gate"].get<double>(), 0.0);
}

// =============================================================================
// 13. state_prior_c_weights (2026-10-01, the learned gaze): weight 1 is byte-identical to none; weight 0 leaves the
//     controller's feedback matrix where it started (a pure reach through h) while h still moves.
// =============================================================================
TEST(StatePrior, CWeightZeroMakesAPureReach) {
    auto pe = base_params();
    pe["state_prior_indices"] = std::vector<double>{-1.0};
    pe["state_prior_targets"] = std::vector<double>{0.4};
    pe["state_prior_gain"]    = 1.0;
    pe["ctrl_lr"] = 0.0; pe["sat_lr"] = 0.0; pe["bias_lr"] = 0.0;   // the prior is C's only writer
    auto p1 = pe; p1["state_prior_c_weights"] = std::vector<double>{1.0};
    auto p0 = pe; p0["state_prior_c_weights"] = std::vector<double>{0.0};
    Fixture E(pe), W1(p1), W0(p0);
    double d1 = 0.0;
    nlohmann::json c0_start;
    for (uint64_t t = 0; t < 300; ++t) {
        const float lean = wobble(t);
        E.run_tick(t, lean); W1.run_tick(t, lean); W0.run_tick(t, lean);
        if (t == 12) c0_start = W0.m.snapshot_state()["legs"][0]["C"];
        for (int j = 0; j < kMotors; ++j) d1 = std::max(d1, double(std::fabs(E.accel(j) - W1.accel(j))));
    }
    EXPECT_EQ(d1, 0.0) << "c weight 1 must be byte-identical to none";
    // with the prior C's only writer, weight 0 leaves C exactly where it was after the babble; weight 1 moves it
    const auto cE = E.m.snapshot_state()["legs"][0]["C"].get<std::vector<float>>();
    const auto c0 = W0.m.snapshot_state()["legs"][0]["C"].get<std::vector<float>>();
    const auto cS = c0_start.get<std::vector<float>>();
    double dE = 0.0, d0 = 0.0;
    for (size_t i = 0; i < cS.size(); ++i) { dE += std::fabs(cE[i] - cS[i]); d0 += std::fabs(c0[i] - cS[i]); }
    EXPECT_EQ(d0, 0.0) << "c weight 0: the feedback matrix untouched";
    EXPECT_GT(dE, 1e-4) << "c weight 1: the descent writes C";
    const auto h0 = W0.m.snapshot_state()["legs"][0]["h"].get<std::vector<float>>();
    double hn = 0.0; for (float v : h0) hn += std::fabs(v);
    EXPECT_GT(hn, 1e-4) << "the reach still moves h";
}

// =============================================================================
// 14. state_prior_target_gated_by (2026-10-01): the ungated sentinel is byte-identical to none; a target scaled by the
//     size of an element that varies (the scripted joint 0) changes the command.
// =============================================================================
TEST(StatePrior, TargetGateUngatedIdenticalGatedActs) {
    auto pe = base_params();
    pe["state_prior_indices"]   = std::vector<double>{-1.0};
    pe["state_prior_targets"]   = std::vector<double>{0.4};
    pe["state_prior_gain"]      = 0.8;
    pe["state_prior_step_gain"] = 1.0;
    auto pu = pe; pu["state_prior_target_gated_by"] = std::vector<double>{9999.0};
    auto pg = pe; pg["state_prior_target_gated_by"] = std::vector<double>{0.0};
    Fixture E(pe), U(pu), G(pg);
    double du = 0.0, dg = 0.0;
    for (uint64_t t = 0; t < 400; ++t) {
        const float lean = wobble(t);
        E.run_tick(t, lean); U.run_tick(t, lean); G.run_tick(t, lean);
        for (int j = 0; j < kMotors; ++j) {
            du = std::max(du, double(std::fabs(E.accel(j) - U.accel(j))));
            dg = std::max(dg, double(std::fabs(E.accel(j) - G.accel(j))));
        }
    }
    EXPECT_EQ(du, 0.0) << "an ungated target must be byte-identical to none";
    EXPECT_GT(dg, 1e-4) << "a target gated by a varying element must change the command";
}

// =============================================================================
// 15. state_prior_target_gate_cos (2026-10-01): the geometric gate acts (differs from the ungated run) and differs from
//     the RMS form; absent, the RMS form is unchanged (the facing config reproduces -- checked in the host guard).
// =============================================================================
TEST(StatePrior, TargetGateCosActsAndDiffersFromRms) {
    auto pe = base_params();
    pe["state_prior_indices"]   = std::vector<double>{-1.0};
    pe["state_prior_targets"]   = std::vector<double>{0.4};
    pe["state_prior_gain"]      = 0.8;
    pe["state_prior_step_gain"] = 1.0;
    pe["state_prior_target_gated_by"] = std::vector<double>{0.0};
    auto pc = pe; pc["state_prior_target_gate_cos"] = std::vector<double>{3.14159265};
    auto pn = base_params();
    pn["state_prior_indices"] = std::vector<double>{-1.0}; pn["state_prior_targets"] = std::vector<double>{0.4};
    pn["state_prior_gain"] = 0.8; pn["state_prior_step_gain"] = 1.0;
    Fixture R(pe), C(pc), N(pn);
    double drc = 0.0, dnc = 0.0;
    for (uint64_t t = 0; t < 400; ++t) {
        const float lean = wobble(t);
        R.run_tick(t, lean); C.run_tick(t, lean); N.run_tick(t, lean);
        for (int j = 0; j < kMotors; ++j) {
            drc = std::max(drc, double(std::fabs(R.accel(j) - C.accel(j))));
            dnc = std::max(dnc, double(std::fabs(N.accel(j) - C.accel(j))));
        }
    }
    EXPECT_GT(drc, 1e-4) << "the geometric gate differs from the RMS form";
    EXPECT_GT(dnc, 1e-4) << "the geometric gate acts";
}

// =============================================================================
// 16. state_prior_target_gate_reach (2026-10-01): the turn-before-you-arrive gate acts on top of the target gate (the
//     scripted joint 1 standing in for the range, joint 0 for the heading error); the ungated sentinel is identical.
// =============================================================================
TEST(StatePrior, TargetGateReachActsSentinelIdentical) {
    auto pe = base_params();
    pe["state_prior_indices"]   = std::vector<double>{-1.0};
    pe["state_prior_targets"]   = std::vector<double>{0.4};
    pe["state_prior_gain"]      = 0.8;
    pe["state_prior_step_gain"] = 1.0;
    pe["state_prior_target_gated_by"] = std::vector<double>{0.0};
    pe["state_prior_target_gate_cos"] = std::vector<double>{3.14159265};
    auto pu = pe; pu["state_prior_target_gate_reach"] = std::vector<double>{9999.0};
    auto pr = pe; pr["state_prior_target_gate_reach"] = std::vector<double>{3.0}; pr["state_prior_target_gate_reach_k"] = 0.2;
    Fixture E(pe), U(pu), R(pr);
    double du = 0.0, dr = 0.0;
    for (uint64_t t = 0; t < 400; ++t) {
        const float lean = wobble(t);
        E.run_tick(t, lean); U.run_tick(t, lean); R.run_tick(t, lean);
        for (int j = 0; j < kMotors; ++j) {
            du = std::max(du, double(std::fabs(E.accel(j) - U.accel(j))));
            dr = std::max(dr, double(std::fabs(E.accel(j) - R.accel(j))));
        }
    }
    EXPECT_EQ(du, 0.0) << "no reach element: byte-identical";
    EXPECT_GT(dr, 1e-4) << "the reach gate acts";
}
