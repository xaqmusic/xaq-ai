#pragma once

// =============================================================================
// CPGOscillator.hpp  --  Biology-inspired spinal Central Pattern Generator
// =============================================================================
//
// Phase 7.x — addresses the front/back leg-role asymmetry found post-bootstrap
// (project_v7_front_back_leg_asymmetry).  Baseline-trained Premotors specialise
// for standing roles (front legs steer, rear legs propel), so when chunks
// dispatch during walking attempts only the rear legs participate — body
// dives nose-down.
//
// Biological analog: vertebrate spinal CPGs produce baseline rhythmic gait
// patterns autonomously, with cortex/cerebellum modulating amplitude and
// phase rather than generating raw motor commands.  Baby quadrupeds walk
// before they learn fine control because the spinal CPG forces all four
// legs into the gait rhythm regardless of supraspinal state.
//
// Architecture (per leg):
//                       action.brain.<leg>_<joint>  (from Premotor)
//                              │
//                              ▼
//      ┌──────────────── CPGOscillator ────────────────┐
//      │   per-joint accel = brain_in + amp * waveform │
//      │                       (phase φ_leg + offset)   │
//      └────────────────────────┬────────────────────────┘
//                               ▼
//                       action.<leg>_<joint>  (body polls)
//
// Phase rotation across the 4 legs implements a lateral-sequence walk
// (fl → rl → fr → rr at quarter-cycle offsets — slowest stable
// quadruped gait, used by elephants/camels/horses at walking pace).
//
// Per-joint waveform offsets within a leg shape lift vs swing vs plant:
//   knee_offset = 0       (knee leads — lifts first)
//   hip2_offset = π/2     (hip2 trails knee by quarter — lifts after)
//   hip1_offset = π       (hip1 swings forward/back opposite to lift)
//
// Amplitude `amp` defaults small (0.2) so Premotor REINFORCE retains policy
// authority — the CPG is a baseline rhythm the brain can learn to modulate
// or override, not a hard-coded gait.
//
// Period is configurable but the default (60 ticks ≈ 1 sim sec on the
// default 60Hz tick) is derived from the body's tick clock — single-second
// stride rate for the picrawler scale.  Future: derive period from chassis
// natural pendulum frequency (sqrt(g/L) ≈ 11.8 rad/s for L=0.07m) ⇒ T≈0.53s.

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class CPGOscillator : public Module {
public:
    CPGOscillator();
    ~CPGOscillator() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json snapshot_state() const override;

    int n_joints()       const { return int(input_topics_.size()); }
    int total_outputs()  const { return total_outputs_; }
    float phase()        const { return phase_; }
    // Inspector / diag accessors — last-tick cached state.
    float last_walking_amp()      const { return last_walking_amp_; }
    float last_standing_factor()  const { return last_standing_factor_; }
    float ema_reward_signal()     const { return ema_reward_signal_; }
    float ema_fused_tle()         const { return ema_fused_tle_; }
    float max_fused_tle_seen()    const { return max_fused_tle_seen_; }
    float latest_fused_tle()      const { return latest_fused_tle_; }
    std::vector<float> const& last_bias_walking()  const { return last_bias_walking_; }
    std::vector<float> const& last_bias_standing() const { return last_bias_standing_; }
    std::vector<float> const& last_blended()       const { return last_blended_; }
    std::vector<float> const& standing_signs()     const { return standing_signs_; }
    std::vector<float> const& leg_phase_offsets()  const { return leg_phase_offsets_; }
    std::vector<float> const& joint_phase_offsets()const { return joint_phase_offsets_; }
    std::vector<std::string> const& output_topics_list() const { return output_topics_; }
    // Phase 7.5 — perceptual-CPG accessors.
    std::vector<std::string> const& perceptual_output_topics_list() const { return perceptual_output_topics_; }
    std::vector<float>       const& perceptual_leg_phase_offsets()  const { return perceptual_leg_phase_offsets_; }
    int n_perceptual_channels() const { return int(perceptual_output_topics_.size()); }
    int period_ticks()            const { return period_ticks_; }
    float base_amplitude_p()      const { return base_amplitude_; }
    float amplitude_floor_p()     const { return amplitude_floor_; }
    float standing_bias_amplitude_p() const { return standing_bias_amplitude_; }
    float gate_ema_alpha_p()      const { return gate_ema_alpha_; }
    float gate_ema_alpha_climb_p()   const { return gate_ema_alpha_climb_; }
    float gate_ema_alpha_decline_p() const { return gate_ema_alpha_decline_; }
    float gate_scale_p()          const { return gate_scale_; }

private:
    void handle_brain_action(int joint_idx, MessagePtr payload);

    // Configuration (lengths must all match n_joints):
    //   input_topics[i]  : ActionOut topic this joint listens on (Premotor side)
    //   output_topics[i] : ActionOut topic this joint publishes to (body side)
    //   leg_phase_offsets[i] : per-joint leg-cycle phase offset (rad)
    //   joint_phase_offsets[i] : per-joint within-cycle waveform offset (rad)
    std::vector<std::string> input_topics_;
    std::vector<std::string> output_topics_;
    std::vector<float>       leg_phase_offsets_;
    std::vector<float>       joint_phase_offsets_;

    // Phase 7.5 — perceptual-CPG channels.  When non-empty, CPG publishes a
    // ProprioToken per channel carrying [cos(phi_c), sin(phi_c)] where
    // phi_c = phase_ + perceptual_leg_phase_offsets_[c].  This is orthogonal
    // to the motor-bias path: when amplitude=0 and standing_bias_amplitude=0
    // the CPG becomes a pure clock publisher.  4-channel default (one per
    // leg) lets per-leg sub-voters / per-Premotor consumers see the gait
    // phase pattern through perception, not through motor injection.
    std::vector<std::string> perceptual_output_topics_;
    std::vector<float>       perceptual_leg_phase_offsets_;
    std::string              perceptual_sensor_label_ = "cpg";

    // Cycle period in ticks (= 2π / Δφ_per_tick).
    int   period_ticks_       = 60;
    // Peak waveform amplitude (the bias added to brain command when the
    // competence-gate is fully open).  Effective per-tick amplitude is
    //   effective = amplitude_floor + (peak - floor) * gate
    // so cold start still emits a baseline CPG impulse (exploration
    // crawling) and the gate AMPLIFIES it as standing competence
    // emerges, rather than gating it off entirely at cold start.
    float base_amplitude_     = 0.3f;
    // Minimum amplitude when competence_gate = 0 (cold start, no
    // sustained dopamine excess).  Provides baseline exploration
    // impulse so a tabula-rasa brain has some leg rhythm to learn
    // from.  Default 0.1 = small but non-zero.
    float amplitude_floor_    = 0.1f;

    // Phase 7.x — standing-bias component for cold-start co-contraction.
    // Walking gait requires phase-rotated alternation (lift-plant-lift);
    // standing requires the opposite — co-contraction (all 4 legs push
    // down together to lift the chassis).  At gate=0 the sinusoidal
    // walking rhythm alone can't bootstrap standing because alternating
    // legs cancel each other's lift force at the chassis.
    //
    // Solution: a DC standing bias term that's strong at gate=0 and
    // fades out as gate opens:
    //   bias_standing = standing_bias_amplitude * (1 - gate) * standing_sign[i]
    // where standing_sign[i] is the per-joint direction pushing toward
    // chassis-lift (typically +1 on knee + hip2, 0 on hip1).
    //
    // Composes additively with the walking rhythm so high-gate phase
    // motion isn't degraded — at gate=1 the standing term is exactly 0.
    float              standing_bias_amplitude_ = 0.3f;
    std::vector<float> standing_signs_;     // per-joint, default 0 if unspecified

    // Phase 7.x — per-joint waveform sign for the sine term.  Picrawler
    // has 4-fold mirror symmetry: rear legs are kinematic mirror images
    // of front legs.  A given +sin(φ) value drives front legs' knees
    // to extend toward +Z (forward in world frame) but rear legs' knees
    // toward -Z (backward).  That's not a CPG bug — it's the body's
    // geometric symmetry surfacing.  Without a sign convention, the
    // configured per-leg phase rotation can't produce coordinated
    // forward locomotion because half the legs paddle the wrong way.
    //
    // joint_waveform_signs[i] multiplies sin(φ_i) for joint i.  Default
    // +1 keeps prior behaviour; for picrawler, set rear-leg joints to
    // ±1 so all four legs' foot velocity projects in the same body-
    // relative direction.  Standing bias is unaffected — it uses
    // standing_signs above for its own per-joint direction control.
    std::vector<float> joint_waveform_signs_;   // per-joint, default +1
    // accel clamp range (matches Premotor's accel_min/accel_max).
    float accel_min_          = -1.0f;
    float accel_max_          =  1.0f;

    // Phase 7.x — competence gate (TLE + reward, geometric mean, ratcheted).
    //
    // The gate combines two independent competence signals:
    //   g_tle    = 1 − ema(fused_tle) / peak(ema_fused_tle)
    //              predictive competence (brain learning to model body)
    //   g_reward = clamp(ema_reward_signal / gate_scale, 0, 1)
    //              behavioural competence (sustained above-baseline reward)
    //   instant  = sqrt(g_tle * g_reward)     // BOTH paths required
    //   gate     = ratchet(instant)           // accumulates, slow decay
    //
    // Why geometric mean: a single-path gate is fragile.  TLE-only
    // stays ~0 because TLE plateaus at the body's noise floor (chaotic
    // standing dynamics that the brain can't perfectly predict).
    // Reward-only opens on prop-induced earnings at cold start (the
    // standing-bias itself lifts the chassis and fires hits before the
    // brain has learned anything).  Demanding BOTH signals nullifies
    // each individual failure mode.
    //
    // Why ratchet: substrate TLE is noisy.  A momentary TLE spike at
    // t=3 min would re-set max-seen and collapse instant gate to 0,
    // even though the substrate had been demonstrating sustained
    // competence for minutes prior.  Ratchet accepts any new high and
    // slowly decays (~17 min half-life) if competence regresses —
    // matching the user expectation that 3 min of standing should
    // produce visible scaffold decay regardless of late noise.
    float gate_ema_alpha_         = 0.001f;   // legacy fallback (used iff climb/decline not specified)
    // Phase 7.x (revised): climb rate matches da_baseline_ema_alpha
    // (0.001 = ~17s window) — the substrate's own self-zeroing
    // dopamine baseline rate.  Earlier draft used 0.005 (~3s) which
    // accumulated brief tabula-rasa standing spikes too aggressively;
    // combined with the asymmetric decline, gate climbed even though
    // the body wasn't actually competent yet.  Matching the
    // substrate's intrinsic timescale is the [[no_tuning]]-principled
    // choice.  Decline stays slower than climb so once competence
    // IS recognised, it's remembered longer than the baseline
    // adapts — the hysteresis goal.
    float gate_ema_alpha_climb_   = 0.001f;   // match substrate baseline-tracking rate
    float gate_ema_alpha_decline_ = 0.0003f;  // slow fall (hysteresis)
    float gate_scale_         = 0.3f;     // dopamine excess of `gate_scale` → gate=1.0

    // Working state
    float phase_              = 0.0f;
    // Slow EMA of NeuroState.reward_signal (= dopamine − adaptive_baseline).
    // PRE-7.x: was the competence-gate signal.  Now KEPT FOR INSPECTOR
    // ONLY — surfaces a useful telemetry view of dopamine excess but no
    // longer drives the gate (dopamine couples to ALL reward sources
    // including prop-induced earnings — see consensus.fused_tle path
    // below for the prop-decoupled replacement).
    float ema_reward_signal_  = 0.0f;
    bool  neuro_seen_         = false;

    // Phase 7.x — TLE-based competence gate.  ConsensusToken.fused_tle
    // is the trust-weighted prediction error across all modalities.
    // As the substrate's predictor learns body dynamics, fused_tle
    // drops — that's the canonical Playful Machine measure of
    // "competence acquired."  Critically, the prop's influence on
    // body motion only contaminates this signal if the brain CAN'T
    // predict prop-induced motion (then TLE rises).  Once the brain
    // models its prop-assisted dynamics, TLE drops — and that IS
    // competence in the predictive-learning sense.
    //
    // Self-adapting reference: max_fused_tle_seen_ tracks the worst-
    // ever TLE so the gate normalises against the substrate's own
    // observed dynamic range (no magic number / no tuned reference).
    float latest_fused_tle_     = 0.0f;
    float ema_fused_tle_        = 0.0f;
    float max_fused_tle_seen_   = 0.0f;
    bool  consensus_seen_       = false;

    // Phase 7.x — competence ratchet.  Raw gate from sqrt(g_tle *
    // g_reward) oscillates with substrate noise (TLE spikes, reward
    // dips), causing standing-bias to snap back to max whenever the
    // momentary signal regresses.  The ratchet climbs fast (tracks any
    // new high) and decays very slowly (~17 min half-life), so once
    // competence has been demonstrated the scaffold STAYS faded —
    // matching the user expectation that "3 min of standing" should
    // produce visible scaffold decay even if subsequent ticks have
    // noisy TLE spikes.  Slow decay still permits genuine regressions
    // to re-engage the scaffold over tens of minutes if the brain
    // catastrophically fails.
    mutable float gate_ratchet_  = 0.0f;
    float gate_decay_per_tick_   = 0.0001f;  // ~17 min half-life

    void handle_consensus(MessagePtr payload);
    std::vector<float> latest_brain_accel_;   // most recent brain command per joint
    std::vector<int64_t> latest_brain_tick_;  // tick_id of latest brain message
    int total_outputs_        = 0;
    // Inspector cache — populated by tick() so the diag stream / inspector
    // can read effective amplitudes + per-joint bias breakdown without
    // having to recompute them.
    float last_walking_amp_     = 0.0f;
    float last_standing_factor_ = 0.0f;
    std::vector<float> last_bias_walking_;
    std::vector<float> last_bias_standing_;
    std::vector<float> last_blended_;

    void handle_neuro(MessagePtr payload);

public:
    // Read-only diag accessor — surfaces the competence-gate value to the
    // JSONL diag stream so the operator can see CPG amplitude scaling live.
    float competence_gate()   const;
};

} // namespace ogma
