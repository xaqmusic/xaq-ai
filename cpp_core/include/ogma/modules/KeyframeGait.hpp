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

private:
    void handle_cpg(MessagePtr payload);
    void handle_proprio(int leg, MessagePtr payload);
    int  bin_of(float phi) const;

    // ---- Configuration ----
    std::string              cpg_topic_ = "rhythm.cpg.body";   // ProprioToken [cos φ, sin φ]
    std::vector<std::string> proprio_topics_;                  // per-leg [pos,act,delta]×motor_dim
    std::vector<std::string> objective_output_topics_;         // per-leg PredictionToken outputs
    int    motor_dim_      = 3;
    int    n_bins_         = 16;
    double keyframe_alpha_ = 0.02;   // cross-cycle EMA rate (slow crystallize)
    double gain_           = 0.3;    // published objective weight w (soft — strong over-constrains)
    // Ablation hooks (§2.8), all HotMutable:
    bool   shuffle_phase_  = false;  // feed a random bin → destroys the phase index (control)
    bool   freeze_map_     = false;  // stop updating the keyframe (proves improvement is learning)
    bool   publish_        = true;   // false = accumulate but don't drive the socket

    // ---- Runtime ----
    float  phi_       = 0.0f;                  // latest CPG phase [0,2π)
    bool   phi_seen_  = false;
    std::vector<Eigen::VectorXf> posture_;     // latest per-leg positions (motor_dim)
    std::vector<char>            posture_seen_;
    std::mt19937                 rng_;         // for shuffle_phase

    // Phase-indexed keyframe map: keyframe_[bin] is a whole-body posture (n_legs*motor_dim).
    std::vector<Eigen::VectorXf> keyframe_;    // [n_bins]
    std::vector<float>           bin_dev_ema_; // [n_bins] per-bin ‖posture-keyframe‖ EMA (consistency)
    std::vector<int64_t>         bin_count_;   // [n_bins] ticks accumulated (0 = unseen)
    float                        keyframe_tle_ema_ = 0.0f;  // aggregate crystallization signal
};

} // namespace ogma
