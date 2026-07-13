#pragma once

// =============================================================================
// Klinotaxis.hpp  --  honest scalar-gradient follower by EPISTEMIC FORAGING
// =============================================================================
//
// The SCENT-GRADIENT loop's climber (operator design, 2026-06-26), built to doctrine §2.1
// (Markov blanket / epistemic foraging) + §5 (CPG as demodulation reference). A scalar
// gradient's DIRECTION is a hidden external state — it cannot be read from any instantaneous
// percept (the static GradientEPM-for-scent failure). The agent must ACT to infer it:
//
//   WEAVE (lateral oscillation) → the scalar oscillates at the weave frequency → LOCK-IN
//   detect (correlate the AC scalar with the weave phase, low-passed over a few cycles) →
//   the in-phase component IS the lateral gradient → STEER the weave centre toward it.
//
// Two matched filters (§5 — "the correct temporal averaging"):
//   - TREND (am I climbing): a dual-timescale differentiator  trend = short_EMA − long_EMA
//     (band-pass: averages noise, removes DC + drift).
//   - DIRECTION: synchronous (lock-in) detection against the self-generated weave phase —
//     only the component varying AT the weave frequency survives; off-band noise averages out.
//
// The weave period is SNR-adaptive within the body's physical envelope (can't weave faster
// than ~1 Hz): hill-climb the lock-in magnitude |g| (no static tuning, §6). Generic
// follow/flee via `mode`. Honest: steers on the agent's OWN Δscalar; feeds the FULL advance-
// on HeadingController. Self-contained internal weave (a shared CPG can drive it later).
// Default-off / opt-in.
//
//   reality.proprio.scent_max (SCALAR) + reality.proprio.heading (egomotion)
//        → Klinotaxis → percept.klino_heading  ([vx,vy] → HeadingController)

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class Klinotaxis : public Module {
public:
    Klinotaxis() = default;
    ~Klinotaxis() override = default;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void                   on_param_change(std::string_view key, ParamValue const& value) override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;

    nlohmann::json diag_snapshot() const override;

    // accessors (tests / telemetry)
    float base_heading() const { return base_heading_; }   // the steered climb direction (absolute)
    float lockin_mag()   const { return std::fabs(g_); }    // |correlation| of dscalar with ω
    float weave_period() const { return period_; }
    float trend()        const { return short_ema_ - long_ema_; }
    float cap()          const { return cap_; }                 // self-calibrated proximity ∈[0,1]
    float weave_eff()    const { return weave_adapt_ ? weave_amp_ * (1.0f - cap_) : weave_amp_; }  // applied weave
    float align()        const { return align_; }               // heading-vs-travel alignment weight ∈[0,1]

private:
    void handle_scalar(MessagePtr payload);
    void handle_heading(MessagePtr payload);
    void handle_imu(MessagePtr payload);
    void handle_authority(MessagePtr payload);
    void handle_vel(MessagePtr payload);

    std::string scalar_topic_  = "reality.proprio.scent_max";
    std::string heading_topic_ = "reality.proprio.heading";
    std::string imu_topic_     = "reality.proprio.ang_vel";   // REAFFERENT yaw rate (what the body actually did)
    std::string authority_topic_ = "";      // klino's MotorBus authority; re-anchor base_heading when a reflex has the bus
    float       authority_       = 1.0f;
    bool        have_authority_  = false;
    float       authority_floor_ = 0.5f;
    // heading-vs-travel: the reafference assumes v ‖ heading; wall-slides violate it. vel_ego = the
    // ACTUAL line of travel → gate the reafferent LEARNING by forward-alignment (reject corrupted samples).
    std::string vel_topic_   = "reality.proprio.vel_ego";   // egocentric velocity [v_right, v_forward]
    float       v_forward_   = 0.0f;
    float       v_right_     = 0.0f;
    bool        align_gate_  = false;       // gate the reafferent base_heading update by heading-vs-travel alignment
    float       align_       = 1.0f;        // forward-alignment weight ∈[0,1] (1 = moving along heading, 0 = sliding/wedged)
    std::string output_topic_  = "percept.klino_heading";

    float period_ticks_   = 60.0f;   // weave period (start ~1 Hz at 60 fps)
    float weave_amp_      = 0.5f;     // lateral excursion (rad) — the far-field sensing wiggle
    bool  weave_adapt_    = false;    // shrink the weave with proximity (fine-close commit on the source)
    float peak_decay_     = 0.0005f;  // slow decay of the running scalar peak (the proximity denominator)
    float short_ema_a_    = 0.3f;     // trend fast EMA
    float long_ema_a_     = 0.05f;    // trend slow EMA  (short−long = band-pass derivative)
    float lockin_lr_      = 0.02f;    // lock-in low-pass rate (~ 1 / few periods)
    float steer_gain_     = 0.05f;    // base-heading steer per unit correlation
    float per_tick_gain_  = 0.0f;     // >0 = simple per-tick reafference (raw ddt·ω), bypass the lock-in
    float ddt_scale_      = 0.0f;     // running |ddt| scale for per-tick mode (field-independent gain)
    float turn_commit_    = 1.5708f;  // |climb dir − heading| beyond this (rad) → STOP weaving,
                                      // freeze base, turn in place to face it (the weave makes a
                                      // ~180° target dither; commit to the turn instead).
    int   mode_           = 1;        // +1 follow / −1 flee
    // defensibility controls (doctrine §2 bar c/d) — break or drop the reafference:
    bool  shuffle_omega_  = false;    // (c) replace ω with noise → lock-in correlation → 0
    int   lesion_at_      = -1;       // (d) drop ω (=0) for [lesion_at, lesion_at+lesion_for)
    int   lesion_for_     = 0;        //     → lose the inference; restored after → recover
    bool  adapt_period_   = true;     // SNR-adaptive weave period
    float period_min_     = 50.0f;    // ~1.2 Hz ceiling (body limit) — never faster
    float period_max_     = 240.0f;
    int   adapt_interval_ = 300;      // ticks between period hill-climb steps
    float adapt_step_     = 8.0f;

    // latest inputs
    float scalar_ = 0.0f, heading_ = 0.0f, omega_ = 0.0f;
    bool  have_scalar_ = false, have_heading_ = false;
    bool  inited_ = false, have_prev_ = false;

    // state
    float base_heading_ = 0.0f;
    float phase_        = 0.0f;
    float period_       = 60.0f;
    float short_ema_ = 0.0f, long_ema_ = 0.0f;
    float prev_scalar_ = 0.0f;
    float cov_ = 0.0f, var_ddt_ = 0.0f, var_om_ = 0.0f;   // for the correlation coefficient
    float g_ = 0.0f;                  // corr(dscalar/dt, ω) ∈ [-1,1] ∝ lateral gradient
    float scalar_peak_ = 0.0f;        // slow-decaying running MAX (self-calibrated source scale) — cap's 1-point
    float scalar_min_  = 0.0f;        // slow-tracking running MIN (the searching floor) — cap's 0-point
    bool  have_scalar_peak_ = false;
    float cap_ = 0.0f;                // proximity ∈[0,1]: 0 at the searching floor, 1 on the source (self-calibrated)
    bool  turning_ = false;          // in turn-in-place commit (climb dir far behind)
    float turn_dir_ = 1.0f;          // latched rotation direction during the commit (no ±π dither)
    float out_vx_ = 0.0f, out_vy_ = 1.0f;
    std::mt19937 rng_{99};           // shuffle-ω control only

    // adaptive period (hill-climb on |g|)
    float g_accum_ = 0.0f; int g_count_ = 0;
    float prev_mean_g_ = -1.0f; float period_dir_ = 1.0f;
    int   adapt_timer_ = 0;
};

}  // namespace ogma
