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
