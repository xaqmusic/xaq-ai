#pragma once

// =============================================================================
// BodyRhythmTracker.hpp  --  proprioceptive gait-phase estimator (L-1b step 3)
// =============================================================================
//
// Turns per-leg proprioception into a body gait-phase estimate (φ_body, ω_body)
// and publishes it as an AFFERENT REFERENCE on `rhythm.body.gait`, which the
// CPGOscillator entrains to.  This is the morphology-SPECIFIC half of the
// CPG↔body PLL: how you read a *walker's* gait phase (a trot collective
// coordinate over the leg joints) differs from a flyer's wingbeat, so this whole
// module is the swappable piece — same output contract, different sensing.
//
// Mechanism (measurement-seeded phase-locked loop):
//   * Collective coordinate  F(t) = Σ_leg sign[leg] · position[swing_joint]
//     (mean-removed) — a scalar that oscillates at the gait fundamental.  The
//     diagonal trot pattern (+,-,-,+) reinforces the rhythm and cancels the
//     common-mode postural offset.
//   * FREQUENCY comes from the unbiased up-crossing interval of F (hysteresis-
//     gated → noise-robust); ω is low-passed toward 2π/period so it drifts
//     SMOOTHLY (no waveform breaks — the property the downstream CPG needs).
//   * PHASE is a pure integrator φ += ω, softly pulled to the reference at each
//     up-crossing (φ_ref = 0).  Feed-forward frequency + feedback phase lock.
//   (An adaptive Hopf oscillator was tried first; its frequency-learning showed
//    a strong-forcing bias at dt=1 — the crossing measurement is exact instead.)
//
// This module ONLY estimates.  It does not drive the body and it does not own
// the coordinating clock — the CPGOscillator low-passes this (jittery, drops out
// on a fall) reference into the stable phase the rest of the brain keys off.

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class BodyRhythmTracker : public Module {
public:
    BodyRhythmTracker();
    ~BodyRhythmTracker() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot()  const override;
    void           restore_state(nlohmann::json const& s) override;

    // Accessors for tests.
    int   n_legs()       const { return int(proprio_topics_.size()); }
    float period_est()   const { return omega_ > 1e-6f ? kTwoPi_ / omega_ : 0.0f; }
    float omega()        const { return omega_; }
    float phi_body()     const { return phi_body_; }
    bool  locked()       const { return crossings_seen_ >= 2; }

private:
    void handle_proprio(int leg, MessagePtr payload);

    static constexpr float kTwoPi_ = 6.28318530717958647692f;

    // ---- Configuration ----
    std::vector<std::string> proprio_topics_;                 // per-leg [pos,act,delta]×motor_dim
    std::string              output_topic_ = "rhythm.body.gait";
    std::vector<float>       leg_signs_;                       // collective-coordinate signs (trot diagonal)
    int    motor_dim_   = 3;
    int    swing_joint_ = 0;       // which joint index carries the gait rhythm (HotMutable to retune live)
    double mean_alpha_  = 0.005;   // DC-removal EMA rate for the collective coordinate
    double amp_alpha_   = 0.02;    // amplitude EMA rate (diag + zero-cross hysteresis)
    double period_alpha_= 0.10;    // EMA rate for the measured up-crossing period
    double omega_lp_    = 0.05;    // per-tick low-pass of ω toward 2π/period (smooth frequency drift)
    double phase_lock_  = 0.10;    // proportional pull of φ toward φ_ref at each up-crossing
    double period_min_  = 16.0;    // sanity rails on ω (NOT the aliasing clamp — that's the CPG's job)
    double period_max_  = 400.0;

    // ---- Runtime ----
    std::vector<Eigen::VectorXf> pos_;      // latest per-leg positions (motor_dim)
    std::vector<char>            seen_;
    // Per-joint DC-removal + amplitude + up-crossing period (the frequency measurement).
    std::vector<float>   mean_ema_;         // [motor_dim]
    std::vector<float>   amp_ema_;          // [motor_dim]  EMA of |F_j|
    std::vector<float>   f_last_;           // [motor_dim]  last mean-removed coordinate
    std::vector<char>    below_;            // [motor_dim]  hysteresis state for up-crossing
    std::vector<int64_t> ticks_since_up_;   // [motor_dim]
    std::vector<float>   period_zc_ema_;    // [motor_dim]  up-crossing period estimate (unbiased)

    // Per-(leg,joint) RAW oscillation diagnostic (untangles intra- vs inter-leg incoherence;
    // the collective coordinate above confounds per-joint frequency). Flat [leg*motor_dim + joint].
    std::vector<float>   raw_mean_, raw_amp_, raw_period_;
    std::vector<char>    raw_below_;
    std::vector<int64_t> raw_tsu_;

    // Phase-locked loop on the swing-joint coordinate.
    float omega_    = kTwoPi_ / 60.0f;      // smoothed frequency (rad/tick), fed forward from period_zc
    float phi_body_ = 0.0f;                 // integrated gait phase, [0,2π)
    float f_swing_  = 0.0f;                 // last swing-joint coordinate (diag)
    int64_t crossings_seen_ = 0;            // up-crossings on the swing joint (≥2 ⇒ period valid)
    // Lock-quality instrument (2026-08-09, substrate-repair P0).  Sampled at each swing
    // up-crossing BEFORE the phase pull — sampling after measures the corrector, not the
    // lock (the step-clock's documented instrument failure).  lock_plv = |mean e^{iφ}|
    // over crossing phases: 1 + mean angle 0 = locked; near 0 = crossings land anywhere.
    double  lock_cos_sum_ = 0.0;
    double  lock_sin_sum_ = 0.0;
    int64_t lock_n_       = 0;
    float   lock_err_ema_ = -1.0f;          // EMA of |pre-pull phase error|; −1 = no crossing yet
};

} // namespace ogma
