#pragma once
// MotorPlanner — PART III's rolling action-prediction buffer, v1: SHADOW OBSERVER.
//
// The operator's architecture (2026-08-10): reflexes write the t0 row (the present);
// the EPM token stream writes the tN rows (the predicted future); the object of study
// is THE PROBABILITY CONE this produces — per-depth distributions over future body
// tokens, phase-conditioned (the M0 diagnosis: on a rhythmic body, transitions are
// phase-dependent and an unconditioned chain degenerates to persistence).
//
// v1 has ZERO AUTHORITY: no outputs on the bus.  It learns a phase-conditioned
// transition model online from the observed token stream, propagates the cone each
// tick, and VERIFIES itself — storing the cone's predictions at probe depths and
// scoring them against reality when the future arrives.  The masking interface
// (mask_mode) sharpens the cone before propagation; masking experiments measure
// cone quality deltas, never behavior (yet).
//
// M1 (2026-08-11, substrate pivot): the CONTINUOUS roll is the substrate — the
// token-argmax chain is refuted at per-tick granularity, but the per-joint marginal
// decode carries verified authority in the k∈[8,34] band.  The MASK AUTHOR
// (author_mode=1) is the first slow loop over that substrate: it PROPOSES region
// masks (inhibition candidates), trials each one, and KEEPS only masks whose
// verified final-vs-raw error ratio shows the inhibition removed predicted mass
// reality never delivered.  Candidates come from the planner's own measured signed
// residuals (where the raw decode systematically hallucinates) plus random
// exploration; keep-rights are EARNED through the meters, never assigned.  While a
// region mask is live the planner propagates a TRUE second unmasked cone, so
// raw-vs-final is a genuine per-tick counterfactual at every depth.
//
// Doctrine anchors: instrument-first (P5/M0 pattern); "feed it phase" (§0 rule 3),
// applied to the VOCABULARY's dynamics for the first time; the plan-as-prediction
// framing (PART III non-negotiables) — this module is the future-buffer half only.

#include "ogma/Module.hpp"
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ogma {

class MotorPlanner : public Module {
public:
    std::string_view type_name() const override { return "MotorPlanner"; }

    std::vector<TopicSpec> input_topics() const override;
    // Shadow BY DEFAULT (empty).  Lever (b): when plan_output_topics is
    // configured, the BASE roll's decode at plan_depth publishes per leg as a
    // band-gated PredictionToken — the first behavioral authority.
    std::vector<TopicSpec> output_topics() const override;
    ParamSchema params_schema() const override;
    ParamMap current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void restore_state(nlohmann::json const& s) override;
    nlohmann::json diag_snapshot() const override;

private:
    static constexpr int   kMaxBins   = 16;
    static constexpr int   kNumProbes = 7;
    static constexpr std::array<int, kNumProbes> kProbes{1, 3, 5, 8, 13, 21, 34};
    static constexpr int   kJoints    = 12;              // 4 hip1 + 4 hip2 + 4 knee
    static constexpr int   kPastRing  = 128;             // shipped whole in diag (raster lesson:
                                                         // client-side accumulation aliases)

    using Dist = std::unordered_map<int, float>;         // token -> probability

    int   phase_bin(float phi) const;
    Dist  propagate(Dist const& d, int bin) const;
    void  apply_mask(Dist& d, int bin) const;
    bool  apply_region_mask(Dist& d, int depth);
    void  decode_row(Dist const& d, float* mean_out, float* sd_out) const;
    // one region spec against one row, with the per-mask no-annihilation revert;
    // the composite paths (kept list, candidate) all funnel through this
    bool  suppress_region(Dist& d, int joint, float lo, float hi, int dlo, int dhi,
                          float strength, int depth, long& hit_counter);
    bool  apply_kept_masks(Dist& d, int depth);   // author_apply: the earned set

    // --- wiring / params
    Bus*        bus_ = nullptr;
    std::string state_topic_  = "reality.bodypose.pose";
    std::string rhythm_topic_ = "rhythm.body.gait";
    std::string pose_topic_   = "reality.proprio.joints";
    double horizon_    = 40.0;    // cone depth in ticks (>= largest probe)
    double beam_k_     = 12.0;    // per-row sparsity cap
    double phase_bins_ = 8.0;
    double mask_mode_  = 0.0;     // 0 none · 1 phase-affinity (refuted) · 2 CONTINUOUS region
    double mask_floor_ = 0.02;    // affinity below this ⇒ masked (mode 1)
    // mode 2 — continuous inhibition on the roll (M1 substrate): suppress token
    // mass whose READOUT pose falls in [val_lo,val_hi] on mask_joint_, at cone
    // depths [depth_lo,depth_hi], by mask_strength_.  All hot-mutable so the
    // inspector widget drives the mask live.  Defaults inert (gain-0).
    double mask_joint_    = -1.0;  // 0..11, or -1 = mask ALL joints' readouts
    double mask_val_lo_   = 0.0;
    double mask_val_hi_   = 0.0;
    double mask_depth_lo_ = 1.0;
    double mask_depth_hi_ = 40.0;
    double mask_strength_ = 1.0;
    // --- the M1 MASK AUTHOR (all inert by default — gain-0 guard).  author_mode=1
    // takes OWNERSHIP of the region-mask params above: it writes a candidate spec
    // at each trial start and zeroes mask_strength_ between trials.
    double author_mode_       = 0.0;    // 0 off · 1 authoring slow loop
    double author_apply_      = 0.0;    // 0 prospector (keeps recorded only) ·
                                        // 1 CLOSE THE LOOP: kept masks apply to the
                                        //   roll continuously, in chronological keep
                                        //   order (each keep was validated MARGINALLY
                                        //   against its predecessors' composite)
    double author_period_     = 800.0;  // ticks per candidate trial
    double author_warmup_     = 3000.0; // ticks before the first trial
    double author_min_n_      = 300.0;  // min per-slot trial verdicts to judge
    double author_keep_ratio_ = 0.95;   // keep iff the SCORING final/raw ratio < this
    double author_score_      = 0.0;    // 0 whole-body ratio · 1 TARGET-JOINT ratio
                                        //   (the ledger's altitude: verified material
                                        //   lives in the per-joint marginals) with the
                                        //   whole-body ratio as a no-damage guard
    double author_guard_      = 1.0;    // score=1: whole-body ratio must stay < this
    double author_depth_min_  = 5.0;    // the ledger's re-use context: k<5 is
    double author_depth_max_  = 34.0;   //   reflex territory on this vocabulary
    double author_max_kept_   = 8.0;
    double seed_              = 1234.0; // reseeded per-run via OGMA_SEED override
    // --- lever (b): the band-gated plan objective (all inert by default) -----
    // The body acts to FULFIL the planner's earned prediction: per leg, a
    // PredictionToken [3 targets | 3 weights] from the BASE (operating) roll at
    // plan_depth.  A joint's weight is 1 only where its verified authority
    // holds at that depth (the earned-bands gate); distress cuts all weights
    // (reflexes own emergencies, t0 and always).
    double plan_publish_      = 0.0;    // 0 = shadow (no publications at all)
    double plan_depth_        = 8.0;    // must be a probe depth {1,3,5,8,13,21,34}
    double plan_distress_cut_ = 0.5;    // distress above this ⇒ all weights 0
    std::vector<std::string> plan_output_topics_;   // 4 per-leg topics; empty = shadow
    std::string distress_topic_ = "reality.proprio.distress";
    float  distress_ = 0.0f;
    std::array<float, kJoints> plan_pose_{};   // BASE decode at plan_depth (this tick)
    std::array<float, kJoints> plan_w_{};      // per-joint gate weights (this tick)
    bool   plan_pose_valid_ = false;
    long   plan_published_ = 0;                // publisher consumer-check counter
    std::vector<uint64_t> subs_;

    // --- live state
    int    cur_tok_  = -1;
    float  phi_      = 0.0f;      // body phase [0,2π)
    float  omega_    = 0.105f;    // rad/tick (≈60-tick stride prior; overwritten by rhythm)
    bool   phi_seen_ = false;
    std::array<float, kJoints> cur_pose_{};
    bool   pose_seen_ = false;
    int    prev_tok_ = -1;        // per-INSTANCE learn context (a static thread_local
    int    prev_bin_ = 0;         // here cross-contaminated two-planner configs)

    // --- learned model: P(next | token, phase_bin), per-tick counts
    // key = token * kMaxBins + bin
    std::unordered_map<int, std::unordered_map<int, float>> trans_;
    std::unordered_map<int, std::array<float, kMaxBins>>    tok_phase_;   // occupancy per bin
    long   n_obs_ = 0;

    // --- per-token pose READOUT (instrument, not percept): Welford mean/var of the
    // 12-D pose observed while each token is winner.  Decodes cone rows to joint space
    // for the piano roll; nothing downstream consumes it.
    struct PoseStat { std::array<float, kJoints> mean{}; std::array<float, kJoints> m2{}; long n = 0; };
    std::unordered_map<int, PoseStat> tok_pose_;

    // --- the cone verification ring: predictions made for tick t, keyed by t.
    // tok0 = the token that was current when the prediction was made — scoring
    // "predict tok0" alongside the cone gives the per-depth PERSISTENCE baseline
    // under the identical pending protocol (the authority line's denominator).
    // pred_pose/pose0 = the DECODED joint prediction and the actual pose at
    // prediction time, frozen when the prediction is made — verified per joint
    // against the arriving pose, giving each track its OWN authority horizon
    // (the global token chain can fail while a joint marginal still predicts).
    struct Pending { uint64_t due; int depth; int tok0; Dist dist;
                     std::array<float, kJoints> pred_pose;      // FINAL decode (kept + candidate/manual mask)
                     std::array<float, kJoints> pred_pose_raw;  // TRUE unmasked-cone decode (== final when unmasked)
                     std::array<float, kJoints> pred_pose_base; // KEPT-ONLY decode (the operating roll under
                                                                // author_apply; == raw when nothing kept/applied)
                     std::array<float, kJoints> pose0;
                     int trial = 0; };                          // author trial serial at prediction time (0 = none)
    std::vector<Pending> pending_;

    // --- per-depth accumulators (index = probe slot)
    std::array<double, kNumProbes> acc_top1_{}, acc_mass_{}, acc_ent_{}, acc_topk_{}, acc_pers_{};
    std::array<long,   kNumProbes> acc_n_{};
    // per-depth × per-joint mean-|error| accumulators: cone decode vs hold-pose
    std::array<std::array<double, kJoints>, kNumProbes> acc_jerr_{}, acc_jpers_{};
    // raw (pre-mask) decode errors — the inhibition damage/benefit meter
    std::array<std::array<double, kJoints>, kNumProbes> acc_jerr_raw_{};
    double marg_top1_ = 0.0; long marg_n_ = 0;    // marginal baseline at depth 1
    std::unordered_map<int, float> marginal_;
    long   masked_out_ = 0;                        // mask activity (consumer check)
    long   mask_saturated_ = 0;                    // rows where total inhibition was refused

    // --- the piano roll: this tick's cone decoded to joint space, all depths.
    // FINAL (post-mask) and RAW (pre-mask, the original excitation) — the widget
    // shows original motion, the mask, and the final motion (operator contract).
    std::vector<float> roll_mean_, roll_sd_;           // roll_len_ × kJoints
    std::vector<float> roll_raw_mean_, roll_raw_sd_;
    bool mask_applied_ = false;                        // any mass suppressed this tick
    int roll_len_ = 0;
    // past ring of actual poses (oldest→newest reconstruction at read time)
    std::array<std::array<float, kJoints>, kPastRing> past_{};
    int past_head_ = 0, past_n_ = 0;

    // --- MASK AUTHOR state (author_mode=1) -----------------------------------
    struct MaskSpec { int joint = -1; float lo = 0, hi = 0; int dlo = 1, dhi = 1; };
    struct Kept { MaskSpec spec; double ratio_all, ratio_tgt; long n; int trial; bool guided; };
    void author_step();                 // the trial state machine (pre-roll)
    void author_start_trial();
    void author_judge_trial();
    uint32_t rng_next();                // xorshift32 off seed_
    uint32_t rng_state_ = 0;
    int      author_phase_  = 0;        // 0 warmup · 1 trial live · 2 drain
    long     author_t_      = 0;        // ticks in current phase
    int      trial_serial_  = 0;        // 0 = no trial yet
    long     trials_done_   = 0;
    MaskSpec cand_{};
    bool     cand_guided_   = false;
    double   last_ratio_all_ = -1.0, last_ratio_tgt_ = -1.0;
    std::vector<Kept> kept_;            // the earned set, CHRONOLOGICAL keep order
                                        // (application order under author_apply;
                                        // cap stops new keeps — eviction would
                                        // invalidate the composite's marginality)
    long kept_masked_out_ = 0;          // rows suppressed by KEPT masks (consumer check)
    long kept_cap_hit_    = 0;          // keeps refused because the cap was reached
    // per-trial MARGINAL verification accumulators: FINAL (kept+cand) vs BASE
    // (kept only), same ticks — a candidate must not inherit the kept set's credit
    std::array<std::array<double, kJoints>, kNumProbes> tacc_jerr_{}, tacc_jerr_base_{};
    std::array<long, kNumProbes> tacc_n_{};
    // signed residual tracker on the RAW decode (Welford) — where the un-inhibited
    // excitation systematically misses reality; the author's proposal gradient
    std::array<std::array<double, kJoints>, kNumProbes> res_mean_{}, res_m2_{};
    std::array<std::array<long,   kJoints>, kNumProbes> res_n_{};
    std::array<int, 24> tried_{};       // (slot,joint,sign) signatures already trialed
    int tried_n_ = 0;
};

} // namespace ogma
