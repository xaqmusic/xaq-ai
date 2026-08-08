#pragma once

// =============================================================================
// KeyframeGait.hpp  --  phase-indexed keyframe accumulator (L-1b centerpiece)
// =============================================================================
//
// Learns the gait as a PHASE-INDEXED KEYFRAME MAP (plan §2.2-§2.6): keyframe[bin]
// is the cross-cycle EMA of the whole-body posture at ticks where the CPG gait
// phase ≈ bin.  It publishes the current bin's posture back through the objective
// socket (per-leg PredictionToken on objective.posture.<leg>, consumed by
// MotorEPM's descent) so the controller descends toward the body's OWN recurring
// motion — the pattern SHARPENS instead of being imposed by coupling+gait_phase.
//
// Composed from three existing patterns:
//   * structure/persistence  <- CylinderBuilder (bin_of, per-bin array, snapshot)
//   * EMA + keyframe-TLE      <- MotorEPM ((1-a)v + a*new)
//   * per-leg objective publish <- PosturalPrior (same PredictionToken socket)
//
// Bakes on cross-cycle CONSISTENCY only (a recurring phase-posture crystallizes;
// a transient doesn't) — never a fitness/reward weight (§2.5).  Escape from the
// standing-DC attractor lives in MotorEPM's anti-freeze, not here.

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class KeyframeGait : public Module {
public:
    KeyframeGait();
    ~KeyframeGait() override;

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

    int   n_legs()       const { return int(proprio_topics_.size()); }
    int   bins_filled()  const;
    float keyframe_tle()  const { return keyframe_tle_ema_; }
    // Per-bin SELF-precision ∈ [0,1] (the objective's own confidence in its prediction at bin b):
    // 0 until the bin is proven (warmup), then scaled by how consistently its posture recurs.
    // This is the bottom-up precision the EFE arbiter will later multiply its allocation onto.
    float self_precision(int b) const;

private:
    void handle_cpg(MessagePtr payload);
    void handle_proprio(int leg, MessagePtr payload);
    void handle_upright(MessagePtr payload);
    void handle_contact(MessagePtr payload);
    void handle_reset(std::string_view topic, MessagePtr payload);
    int  bin_of(float phi) const;

    // ---- Configuration ----
    std::string              cpg_topic_ = "rhythm.cpg.body";   // ProprioToken [cos φ, sin φ]
    std::vector<std::string> proprio_topics_;                  // per-leg [pos,act,delta]×motor_dim
    std::vector<std::string> objective_output_topics_;         // per-leg PredictionToken outputs (posture)
    std::vector<std::string> velocity_output_topics_;          // per-leg PredictionToken outputs (velocity); empty = off
    int    motor_dim_      = 3;
    int    n_bins_         = 16;
    double keyframe_alpha_ = 0.02;   // cross-cycle EMA rate (slow crystallize)
    double gain_           = 0.3;    // base objective weight (policy); w = gain · self_precision
    // Self-precision gate (§2.5 "drive on consistency"): don't drive a smeared/unproven bin.
    int    warmup_visits_  = 24;     // per-bin visits before it may drive (crystallize-then-drive)
    double precision_scale_= 0.6;    // softness: precision = warmup_ramp · exp(−bin_dev/scale)
    // Ablation hooks (§2.8), all HotMutable:
    bool   shuffle_phase_  = false;  // feed a random bin → destroys the phase index (control)
    bool   freeze_map_     = false;  // stop updating the keyframe (proves improvement is learning)
    bool   publish_        = true;   // false = accumulate but don't drive the socket
    // Left-right symmetry prior on the VELOCITY map (anti-circling): equalize the RMS push
    // magnitude of paired legs' joints so the propulsive pump can't develop a yaw bias (which
    // Cvel would otherwise amplify into circling).  Posture map untouched.  gain 0 = off.
    std::vector<int> symmetry_pairs_;      // flat [legA,legB, legC,legD, ...] lr pairs (e.g. [0,1,2,3])
    double vel_symmetry_gain_ = 0.0;       // per-tick pull toward equal paired-leg energy (0..1)

    // ---- Posture-validity BAKE GATE (2026-07-22) ----
    // The stability-plasticity fix: EXPLORE freely (the HK loop is untouched) but
    // LEARN only from states consistent with a well-functioning agent, so an
    // anomalous episode (flipped on a wall, sustained-airborne thrashing) can't
    // bake detrimental keyframes.  "Sustained ALL feet off" (not "some feet off")
    // is the airborne signature — normal swing + brief leaps still bake.  Any gate
    // topic wired turns the gate ON; all empty = OFF (validity=1, byte-identical).
    std::string bake_upright_topic_ = "";   // 1-D upright ∈ [-1,1] (1=upright, -1=inverted)
    std::string bake_contact_topic_ = "";   // n-D per-leg foot contact (sum==0 → all feet off)
    std::string bake_reset_topic_   = "";   // event prefix ("events.") → mask baking after reset/miss
    double  bake_upright_min_        = 0.5;  // below this = flipped/on-its-side → don't bake
    int64_t bake_airborne_max_ticks_ = 60;  // all-feet-off LONGER than this → don't bake (leaps OK)
    int64_t bake_reset_mask_ticks_   = 30;  // suppress baking for this many ticks after a reset

    // ---- Runtime ----
    float  phi_       = 0.0f;                  // latest CPG phase [0,2π)
    bool   phi_seen_  = false;
    std::vector<Eigen::VectorXf> posture_;     // latest per-leg positions (motor_dim)
    std::vector<Eigen::VectorXf> vel_;         // latest per-leg deltas/velocities (motor_dim)
    std::vector<char>            posture_seen_;
    std::mt19937                 rng_;         // for shuffle_phase

    // Phase-indexed keyframe map: keyframe_[bin] is a whole-body posture (n_legs*motor_dim).
    std::vector<Eigen::VectorXf> keyframe_;    // [n_bins]
    // Phase-indexed VELOCITY map: the propulsive trajectory (the "push"), complement of the
    // posture "pose".  vel_keyframe_[bin] = cross-cycle EMA of the whole-body delta at that
    // phase → the objective can carry a phase-indexed velocity feed-forward, not just position.
    std::vector<Eigen::VectorXf> vel_keyframe_;// [n_bins]
    std::vector<float>           bin_dev_ema_; // [n_bins] per-bin ‖posture-keyframe‖ EMA (consistency)
    std::vector<int64_t>         bin_count_;   // [n_bins] ticks accumulated (0 = unseen)
    float                        keyframe_tle_ema_ = 0.0f;     // aggregate posture crystallization signal
    float                        vel_keyframe_tle_ema_ = 0.0f; // aggregate velocity crystallization signal (diag)
    float                        last_lr_imbalance_ = 0.0f;    // mean |eA-eB|/(eA+eB) over pairs BEFORE the last symmetry pull (diag)
    float                        last_drive_w_ = 0.0f;         // last published confidence (diag)
    // Bake-gate runtime.
    bool    bake_gated_          = false;    // any gate topic wired → the gate is active
    float   upright_             = 1.0f;     // latest upright signal
    int     n_contact_           = 0;        // latest # feet in contact
    int64_t all_off_ticks_       = 0;        // consecutive ticks with ALL feet off
    int64_t reset_mask_ticks_    = 0;        // remaining post-reset bake-mask ticks
    bool    bake_valid_diag_     = true;     // last tick's bake validity (diag)
    int64_t bake_suppress_count_ = 0;        // cumulative ticks baking was suppressed (diag)
};

} // namespace ogma
