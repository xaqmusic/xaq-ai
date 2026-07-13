#pragma once

// =============================================================================
// PremotorAI.hpp  --  Premotor active-inference upgrade (2026-06-03)
// =============================================================================
//
// STATUS (2026-06-03): renamed clone of Premotor with NO behavioural changes.
// Currently bit-equivalent to Premotor — the EFE upgrade described in
// .claude/plans/yes-read-any-other-humming-charm.md was NOT YET ADDED because
// the diagnostic phase (see
// docs/findings/2026_06_03_premotor_active_inference_diagnostic_findings.md)
// revealed that the originally-planned Mechanism A is insufficient without
// upstream substrate repairs (R1 = symmetry-break per Premotor, R2 = boost
// rhythm trust). The new module exists, compiles, and is registered, but its
// behaviour MUST stay byte-identical to Premotor until the R1/R2 design is
// settled. See also:
//   - project_v6_premotor_bilateral_mirror_collapse.md
//   - project_v6_cpg_period_sweep_invariance.md
//
// Per backward-compat directive (v6-g6dof-migration-landed): the existing
// Premotor module is NOT modified. Any new EFE additions go HERE.
//
// ---------------------------------------------------------------------------
// Original Premotor.hpp comment block follows.
// ---------------------------------------------------------------------------

// Premotor.hpp  --  Phase 6.5.25 graded-policy module
// =============================================================================
//
// Sits between LateralVoter (consensus.<level>) and the body's motor
// translation, providing a graded learned mapping from world-state latent
// to a distribution over named motor intents.  Designed as a drop-in
// replacement for ActionDecoder when the Cell graph wants to test the
// "graded representation beats discrete EFE argmax" hypothesis (see
// project_v4_phase6_5_22_attribution.md for the motivating finding that
// the brain's net contribution at full ActionDecoder weight was ~zero).
//
// Architecture:
//
//   consensus.0 (latent)   ──┐
//   drive.errors (urgency) ──┼──► Premotor ──► policy.intent (PolicyToken)
//   neuro.state  (DA/HT)   ──┘                  │
//                                               └──► action.out  (ActionOut)
//
// Mechanism:
//   * N "intents" with fixed accel values (e.g., 5 = {-4,-2,0,+2,+4}).
//   * Per-intent weight vector w_i in latent space (Hebbian-updated).
//   * Each tick:
//       scores       = (W @ latent + b) * gain
//       T            = T_base / (1 + DA * T_da_gain)   // DA → exploit
//       distribution = softmax(scores / T)
//       weighted_accel = Σ_i distribution[i] * intent_accels[i]
//       (or sample from distribution; either path emits ActionOut)
//
// Learning:
//   * Track the recent latent + recent distribution.
//   * On events.hit (positive valence, intensity r):
//       For each intent i: w_i += lr * r * latent * distribution[i]
//     i.e. credit each intent proportional to its contribution that tick
//     (REINFORCE-style policy gradient with one-step credit).
//   * On events.miss / events.wall_stuck (negative): same update with
//     negated r, weakening the contributing weights.
//
// Why this is the missing layer:
//   * ActionDecoder's 3-bin discrete EFE produces bimodal output (53% zero,
//     44% saturated; Phase 6.5.22 trace).  No graded steering signal.
//   * Premotor's softmax over a graded intent space *can* produce graded
//     accel because weighted_accel is a continuous mixture.
//   * Reward learning is explicit and local (Hebbian, no value-iteration
//     loop), which matches the rest of the substrate's biological idiom.

#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class PremotorAI : public Module {
public:
    PremotorAI();
    ~PremotorAI() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors (used by tests + diag).
    int    n_intents()     const { return int(intent_accels_.size()); }
    float  last_accel()    const { return last_accel_; }
    float  last_entropy()  const { return last_entropy_; }
    int    last_chosen()   const { return last_chosen_intent_; }
    int    raw_chosen()    const { return raw_chosen_intent_; }
    int    held_intent()   const { return held_intent_; }
    int    held_intent_ticks_left() const { return held_intent_ticks_left_; }
    int    dwell_holds()   const { return dwell_holds_; }
    int    dwell_breaks()  const { return dwell_breaks_; }
    int    phase_bin() const { return phase_bin_; }
    int    phase_switch_penalties() const { return phase_switch_penalties_; }
    int    phase_boundary_holds() const { return phase_boundary_holds_; }
    int    phase_bin_changes() const { return phase_bin_changes_; }
    // v5.4.M Diagnostic B — Shannon entropy of the CHOSEN intent
    // histogram over the last chosen_window_size_ ticks (default 60 =
    // 1 sim-sec).  Low entropy = Premotor stuck on one intent (rut
    // signal); high entropy = exploring.  Per the Playful Machine
    // principle, low entropy should trigger exploratory perturbation.
    float  chosen_window_entropy() const;
    int    chosen_window_size()    const { return int(chosen_window_.size()); }
    Eigen::MatrixXf const& weights() const { return W_; }
    Eigen::MatrixXf const& bucket_bias() const { return bucket_bias_; }
    int  n_buckets()    const { return n_buckets_; }
    int  current_bucket() const { return current_bucket_; }
    bool   bilateral_enabled() const { return bilateral_enabled_; }
    std::vector<float> const& intent_accels_left()  const { return intent_accels_left_; }
    std::vector<float> const& intent_accels_right() const { return intent_accels_right_; }
    float  last_pathway_familiarity() const { return last_pathway_familiarity_; }
    int    last_bc_intent() const { return last_bc_intent_; }
    std::vector<int> const& bc_intent_counts() const { return bc_intent_counts_; }
    int    bc_total_updates() const { return bc_total_updates_; }
    std::vector<int> const& chosen_intent_counts() const { return chosen_intent_counts_; }
    int    total_overrides_used() const { return total_overrides_used_; }
    int    total_explore_overrides_used() const { return total_explore_overrides_used_; }
    bool   last_explore_active() const { return last_explore_active_; }
    int    aligned_rewards_seen() const { return aligned_rewards_seen_; }
    // Phase v5.1 MC actor-critic diag accessors.
    int    mc_episodes_seen()    const { return mc_episodes_seen_; }
    int    mc_trajectory_size()  const { return int(mc_trajectory_.size()); }
    float  mc_last_return()      const { return mc_last_return_; }
    float  mc_return_mean()      const { return mc_return_mean_; }
    float  mc_return_std()       const { return mc_return_std_; }

private:
    void handle_consensus(std::string_view topic, MessagePtr payload);
    void handle_drive(std::string_view topic, MessagePtr payload);
    void handle_neuro(std::string_view topic, MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);
    void handle_bc_reflex_left (std::string_view topic, MessagePtr payload);
    void handle_bc_reflex_right(std::string_view topic, MessagePtr payload);
    void handle_bc_reflex_uni  (std::string_view topic, MessagePtr payload);
    void handle_intent_override(std::string_view topic, MessagePtr payload);
    void handle_explore_directive(std::string_view topic, MessagePtr payload);
    void handle_phase_context(std::string_view topic, MessagePtr payload);

    void apply_reward(float intensity);  // Hebbian update from a recent event
    void apply_bc_update(uint64_t tick_id);  // Phase 6.6.O — reflex imitation
    void finalize_mc_episode();          // Phase v5.1 — Monte-Carlo actor-critic
    void handle_leg_phase(std::string_view topic, MessagePtr payload);  // A2: per-leg phase

    // NOTE: Bus* bus_ is inherited from Module base.  Do not redeclare —
    // shadowing it leaves Module::bus_ stuck at nullptr, which makes
    // Module::on_teardown early-return WITHOUT unsubscribing from the
    // bus.  The result is a use-after-free: the bus keeps every lambda
    // captured `this` after the module is destroyed, and the next
    // publish on a matching topic fires a handler whose `this` points
    // at freed memory — segfault inside state_visit_ema_._S_equals.
    int   level_                = 0;       // consensus.<level> we read from
    int   latent_dim_           = 0;       // resolved on first consensus token
    float gain_                 = 1.0f;
    float lr_                   = 0.01f;
    float t_base_               = 1.0f;
    float t_da_gain_            = 0.5f;
    bool  sample_action_        = true;    // sample chosen_intent vs argmax
    bool  use_weighted_accel_   = true;    // emit weighted-mean accel (false = chosen-intent accel)
    float accel_min_            = -4.0f;
    float accel_max_            =  4.0f;
    uint64_t master_seed_       = 0;

    std::vector<float> intent_accels_;     // length = N intents
    std::vector<std::string> intent_names_;

    Eigen::MatrixXf W_;                    // (N intents, latent_dim) — lazy alloc
    Eigen::VectorXf b_;                    // bias (N intents)
    bool weights_initialised_   = false;

    // 2026-06-04 Phase A2 — per-leg phase latent augmentation.
    // When leg_phase_gain_ > 0 AND leg_phase_input_topic_ is non-empty,
    // PremotorAI subscribes to a ProprioToken-publishing topic carrying
    // [cos(φ_leg), sin(φ_leg)] (the per-leg CPG phase from
    // rhythm.cpg.<leg>). A separate learnable W_leg matrix
    // (n_intents × 2) maps this 2D signal to a logit-space contribution
    // that ADDS to scores: scores += leg_phase_gain * W_leg * leg_phase_s.
    //
    // Two W matrices learn jointly under MC REINFORCE: W_ on the
    // consensus latent (shared input across leg-pair Premotors when
    // they read the same consensus.<level>) and W_leg on the per-leg
    // phase signal (genuinely different per Premotor when each is
    // configured with its own leg's rhythm.cpg.<leg>). This is the
    // direct per-leg input-into-W mechanism missing from R1a/A1.
    //
    // Default leg_phase_gain_=0 → W_leg never allocated → bit-identical
    // legacy behavior. Backward-compat preserved.
    std::string      leg_phase_input_topic_;
    float            leg_phase_gain_       = 0.0f;
    float            leg_phase_lr_         = 0.01f;
    Eigen::MatrixXf  W_leg_;                  // (n_intents, 2); zeroed at on_setup
    Eigen::Vector2f  leg_phase_s_          = Eigen::Vector2f::Zero();
    bool             leg_phase_seen_       = false;
    float            last_leg_phase_contribution_ = 0.0f;  // diag

    // Recent state for Hebbian credit assignment.
    Eigen::VectorXf last_latent_;
    Eigen::VectorXf last_distribution_;

    // Phase 6.5.29 — eligibility traces for temporal credit assignment.
    // When eligibility_lambda_ > 0, the credit propagated by reward events
    // is the running sum of past (p_i × latent) contributions decayed by
    // lambda each tick, rather than the immediate one-step product.  At
    // lambda=0 the behaviour is identical to the original Premotor REINFORCE.
    // At lambda close to 1, reward arriving N ticks late still credits the
    // policy/state that was active N ticks ago — necessary for momentum-
    // class problems (MountainCar) where rewards are sparse and temporally
    // distant from the actions that produced them.
    float eligibility_lambda_   = 0.0f;
    Eigen::MatrixXf E_;                    // (N intents, latent_dim) eligibility trace

    // Current modulators (latched from last received messages).
    float dopamine_             = 0.2f;
    float urgency_              = 0.0f;

    // Phase 6.5.30 — drive-coupled reward gradient.  When > 0, drops in
    // DriveErrors.urgency are converted to a synthetic positive-reward
    // event each tick (and rises to negative).  Provides continuous
    // gradient signal for sparse-reward envs (MC) where events.hit fires
    // too rarely to bootstrap learning.  Off by default (chemotaxis envs
    // have plenty of events.hit gradient already).
    float drive_reward_gain_    = 0.0f;
    float prev_urgency_         = 0.0f;
    bool  have_prev_urgency_    = false;

    // Phase 6.5.31 — epistemic novelty bonus.  Score modulation that
    // adds (1 - recent_visit_ema_i) × epistemic_gain to each intent's
    // score before softmax.  Pushes the distribution toward intents
    // that haven't been chosen lately, providing directed exploration
    // analogous to ActionDecoder's epistemic_gain × H(rollout) without
    // requiring a forward rollout.  Off by default (epistemic_gain=0).
    float epistemic_gain_       = 0.0f;
    float epistemic_alpha_      = 0.05f;     // visit_ema decay rate
    Eigen::VectorXf visit_ema_;              // per-intent recent-visit EMA

    // Phase 6.5.36 — value baseline V(s) for advantage-actor-critic
    // Hebbian credit.  When baseline_lr_ > 0 we maintain a linear value
    // head V_w · latent and use the TD-error δ = r + γV(s') − V(s)
    // instead of raw r in apply_reward().  Bounds Hebbian growth
    // (negative δ on bad steps shrinks weights) and lets Premotor
    // converge stably under long dense-reward training (CartPole 300+ eps).
    // Off by default to preserve Phase 6.5.25 Cell behaviour.
    float           baseline_lr_     = 0.0f;
    float           value_gamma_     = 0.95f;
    float           target_tau_      = 0.005f;  // slow EMA toward online V
    Eigen::VectorXf value_w_;                  // (latent_dim,) online linear V head
    Eigen::VectorXf value_w_target_;           // (latent_dim,) target V head (slow EMA)
    float           prev_value_      = 0.0f;
    Eigen::VectorXf prev_value_latent_;
    bool            have_prev_value_ = false;

    // Phase 7.8 — predictive value head (per-intent reward prediction).
    //
    // Diagnosis from Phase 7.5+7.6+7.7 (7 substrate sweeps, all null/regress
    // on max_distance): credit assignment mechanism is correct (score-function
    // REINFORCE + advantage_normalization both enabled since v6); brain learns
    // policy but doesn't link gait → direction.  REINFORCE is BACKWARD-LOOKING
    // (credit-assign actions that already happened) — picrawler needs
    // FORWARD-LOOKING bias (which intent will produce reward NEXT).
    //
    // Per-intent value head: V[i, :] · latent predicts E[reward in next N ticks
    // | intent i chosen, current latent].  Updated via simple TD-error each
    // time the lookahead window closes: δ = accumulated_reward − V[chosen]·latent.
    // At action selection, biases logits by value_head_gain × predicted_values.
    //
    // Aligned with the operator hypothesis: rewards work when pointed at the
    // RIGHT mechanism, especially predictions.  V head is the prediction-of-
    // future-reward; it gets trained against actual reward arrival; Premotor
    // intent selection uses the prediction PROSPECTIVELY rather than
    // REINFORCE's retrospective credit.
    //
    // Composes with existing REINFORCE: W updates (policy gradient) and V
    // updates (value head) train independently from the same trajectory.
    // value_head_gain=0 preserves byte-identical legacy behaviour.
    float            value_head_gain_      = 0.0f;
    float            value_head_lr_        = 0.01f;
    int              value_head_lookahead_ = 30;     // ticks (0.5s at 60Hz)
    Eigen::MatrixXf  V_;                              // (n_intents × latent_dim)
    struct ValueStep {
        int             chosen;
        Eigen::VectorXf latent;
        uint64_t        tick_id;
        float           accumulated_reward;
    };
    std::deque<ValueStep> value_traj_;

    // Phase 7.9 — SynergyTimer rhythm bias.  When rhythm_bias_topic_ is
    // non-empty, Premotor subscribes and adds the received pre-softmax
    // bias vector to its scores each tick.  The bias is already gain-
    // and confidence-scaled by SynergyTimer; Premotor just adds it.
    // Composes additively with W (REINFORCE), epistemic_gain,
    // value_head_gain — all four sources stack in logit space before
    // softmax.
    std::string      rhythm_bias_topic_;
    Eigen::VectorXf  latest_rhythm_bias_;
    bool             rhythm_bias_seen_ = false;

    float last_accel_           = 0.0f;
    float last_entropy_         = 0.0f;
    int   last_chosen_intent_   = -1;

    // Phase 8/P1 — intent dwell / hysteresis.  Default-off.  When enabled,
    // ordinary sampled/argmax choices are held for a short window so tick-wise
    // softmax noise cannot flip a servo Premotor every frame.  Hard overrides
    // (chunk replay / exploration directives) bypass the hold and reset it.
    int   intent_dwell_ticks_       = 0;      // 0 = off
    float intent_dwell_break_bias_  = 0.0f;   // proposed p must exceed held p by this margin
    int   held_intent_              = -1;
    int   held_intent_ticks_left_   = 0;
    int   raw_chosen_intent_        = -1;
    int   dwell_holds_              = 0;
    int   dwell_breaks_             = 0;

    // Phase-viscosity P2 — phase-bin commitment.  Default-off.  When a
    // phase_context_topic carries [cos(phi), sin(phi)], Premotor bins the
    // free-running CPG phase and can discourage or forbid within-bin intent
    // switches.  This gives the helpful clock motor authority without
    // low-pass filtering servo commands.
    std::string phase_context_topic_;
    int         phase_bins_              = 0;
    std::string phase_commit_mode_       = "none";  // none | switch_penalty | bin_boundary
    float       phase_switch_penalty_    = 0.0f;

    // 2026-05-29 — gait-bucket bet.  When n_buckets_>0 and bucket_context_topic_
    // is set, subscribe to a single-float ProprioToken (0..n_buckets-1) carrying
    // the leg's current discrete bucket (e.g. 0=swing, 1=stance), allocate
    // bucket_bias_(n_buckets_, n_intents_), and add bucket_bias_.row(current_bucket_)
    // to logits each tick.  REINFORCE updates bucket_bias_ per-step alongside W_,
    // giving the policy a per-bucket specialization axis (which the per-joint
    // standing-reward landscape currently lacks).
    std::string         bucket_context_topic_;
    int                 n_buckets_       = 0;
    Eigen::MatrixXf     bucket_bias_;            // (n_buckets_, n_intents_); zeroed at on_setup
    int                 current_bucket_  = -1;   // last seen; -1 = no msg yet
    bool                bucket_seen_     = false;
    // Separate lr for bucket_bias (default 0 = use mc_lr).  v1 (mc_lr=0.05)
    // grew bucket_bias norms to 6-9, overpowering W·context contributions
    // (typically O(1)) and killing compass-driven heading regulation.  Setting
    // bucket_bias_lr small (e.g. 0.005) keeps per-bucket bias on the same
    // scale as W contributions so both compose instead of bias dominating.
    float               bucket_bias_lr_  = 0.0f;

    // Anti-symmetric bucket_bias init (Fix 3 / V8): when > 0, initialize
    // bucket_bias_ in on_setup so swing and stance buckets favor OPPOSITE
    // extremes — bias[0, 0] = +init; bias[0, last] = -init; bias[1, 0] = -init;
    // bias[1, last] = +init.  Breaks the symmetry that lets the policy leave
    // hip1 unused (the insect-stride locomotion axis): under this prior every
    // Premotor enters with a bimodal preference between the two extreme intents
    // per bucket; REINFORCE then refines which extreme = which bucket-role
    // per leg.  Generalizes to n_buckets > 2 by alternating sign.
    float               bucket_bias_init_alt_ = 0.0f;
    bool        phase_context_seen_      = false;
    float       phase_value_             = 0.0f;
    int         phase_bin_               = -1;
    int         last_phase_commit_bin_   = -1;
    int         phase_switch_penalties_  = 0;
    int         phase_boundary_holds_    = 0;
    int         phase_bin_changes_       = 0;

    // Phase 7.10 — adaptive entropy anneal (self-correcting symmetry break).
    //
    // Diagnosed failure mode from seed-51 long run + Stage 3 n=10:
    // policy-entropy dropout loop.  Premotors whose actions don't affect
    // chassis dynamics (rear legs lift slightly when front Premotors
    // commit hard) get diffuse credit signals → policies stay near
    // uniform (H ≈ log(N) ≈ 1.6) → weighted_accel ≈ 0 → leg goes
    // visually stiff → the brain has no gradient to discover those legs
    // matter → loop perpetuates.
    //
    // Anneal mechanism: track each Premotor's entropy_ema_.  When it
    // stays above entropy_anneal_threshold_ (Premotor is stuck), multiply
    // softmax temperature DOWN proportionally — sharper distribution
    // → more committed sampling → effective accel becomes non-zero →
    // chassis dynamics depend on this Premotor's actions → REINFORCE
    // gets a discriminative signal → entropy_ema falls → anneal relaxes.
    //
    // Off by default (gain=0).  When on, the mechanism is self-correcting:
    // converged Premotors (low entropy) get unaffected T; stuck ones get
    // T reduction until they commit.  No noise injection — the existing
    // W asymmetries (however tiny) get amplified by softmax sharpening,
    // and the brain learns from the resulting committed behaviour.
    float entropy_anneal_gain_       = 0.0f;     // 0 = off
    float entropy_anneal_threshold_  = 1.2f;     // entropy above this counts as stuck
    float entropy_anneal_alpha_      = 0.005f;   // ~200-tick EMA, ~3s at 60Hz
    float entropy_ema_               = -1.0f;    // -1 = uninitialised → first observation seeds it
    float last_anneal_t_mult_        = 1.0f;     // telemetry: last applied T multiplier

    // v5.4.M Diagnostic B — rolling window of chosen intent indices for
    // computing windowed Shannon entropy (rut signal).  Default 60-tick
    // window = 1 sim-sec at 60Hz.  Capacity bounded — std::deque so
    // pop_front is O(1).
    std::deque<int>  chosen_window_;
    int              chosen_window_size_max_ = 60;

    // Phase 6.6.F — α-gated reward update.  When MotorFader is in the
    // graph and Premotor's intent didn't drive ≥update_alpha_threshold_
    // of the actual motor output, the Hebbian update is skipped to
    // avoid off-policy contamination (crediting Premotor for outcomes
    // caused by reflexes).  Default 0.0 = off → behaviour is identical
    // to pre-6.6.F Premotor.
    std::string alpha_topic_         = topics::kMotorFaderAlpha;
    float       update_alpha_threshold_ = 0.0f;
    float       last_alpha_          = 1.0f;   // assume full authority until told otherwise
    // Phase 6.6.F — configurable action output topic so Premotor can be
    // routed through MotorFader (publish to "action.brain" instead of
    // the default "action.out").  Default-preserves prior behavior.
    std::string action_output_topic_ = std::string(topics::kActionOut);

    // v5.3 Phase B — chunk-replay intent override.  When configured,
    // Premotor subscribes to intent_override_topic_ (default
    // intent.override).  Each tick, BEFORE the softmax sample, we check
    // whether an IntentToken arrived for THIS tick (matched by tick_id);
    // if so we use its index as `chosen` instead of sampling.  Both
    // motor emission AND Hebbian/REINFORCE crediting use the override
    // index, so chunk dispatches naturally feed the policy gradient
    // (the gradient learns "intent X under condition Y produced reward R"
    // regardless of whether X was self-sampled or chunk-supplied).
    //
    // Default empty = no subscription = exact pre-v5.3 behaviour.
    std::string intent_override_topic_ = "";   // empty = override disabled
    int         pending_override_idx_  = -1;
    uint64_t    pending_override_tick_ = uint64_t(-1);
    int         total_overrides_used_  = 0;

    // Phase 7.2-EPM — multi-channel chunk support.
    //   policy_output_topic_  : per-instance PolicyToken publish target.
    //       Default topics::kPolicyIntent (= "policy.intent") preserves
    //       legacy behaviour where all Premotors share one topic (a single
    //       global policy).  For per-leg chunks (4 MotorRepertoires, each
    //       crystallising 3-channel intent_sequences) the 12 Premotors
    //       must publish to distinct topics like "policy.intent.fl_hip1"
    //       so each leg's MotorRepertoire can subscribe to its own three.
    //   intent_channel_       : which slice of IntentToken.indices this
    //       Premotor reads during chunk override.  Default -1 = legacy
    //       (read IntentToken.index instead).  >=0 means "I am channel N
    //       in a multi-channel chunk; use indices[N]".
    std::string policy_output_topic_  = std::string(topics::kPolicyIntent);
    int         intent_channel_       = -1;

    // Phase 6.7 / Stage C.5 — HomeokineticExploration override.  When
    // configured (explore_directive_topic_ non-empty), Premotor subscribes
    // to ExplorationDirective on that topic.  Each tick, AFTER the chunk
    // intent_override check and BEFORE the softmax sample, we check
    // whether the most recent directive is active; if so we override
    // `chosen` with the intent index whose intent_accels_[i] is nearest
    // directive.accel.  Same downstream crediting as a chunk override —
    // the explore action enters REINFORCE just like a self-sampled one.
    //
    // Designed to break policy attractor lock-in: when HomeokineticExploration
    // detects that drive-error variance has collapsed (model too predictable),
    // it fires a held-accel directive for episode_ticks_ ticks, dragging the
    // sampled intent off the entrenched mode.
    //
    // Default empty = no subscription = exact pre-Phase-6.7 behaviour.
    std::string explore_directive_topic_   = "";   // empty = override disabled
    bool        last_explore_active_       = false;
    float       last_explore_accel_        = 0.0f;
    uint64_t    last_explore_tick_         = uint64_t(-1);
    int         total_explore_overrides_used_ = 0;

    // v5.3 Phase C — bonus reward event.  When an event with this name
    // arrives, treat it as +intensity * aligned_reward_gain in the same
    // credit-assignment path as events.hit.  Default empty / 0 = disabled
    // (legacy behaviour).  Used for hand-tuned reward scaffolding so
    // EventConjunction's events.scent_aligned_with_green firings feed
    // Premotor's REINFORCE just like real hits do.
    std::string aligned_event_name_   = "";
    float       aligned_reward_gain_  = 1.0f;
    int         aligned_rewards_seen_ = 0;

    // Phase 6.6.I — rollout-aware exploration.  When pathway_temp_gain_
    // > 0, Premotor reads ConsensusToken.predicted_pathways +
    // winner_ids_by_modality (populated by LateralVoter from the per-
    // modality predicted_pathway fields the EPMs emit) and modulates
    // the softmax temperature by predicted-trajectory familiarity:
    //
    //   For each (modality, observed_winner_id), maintain a per-modality
    //   per-node visit-EMA that decays toward 0 unseen, climbs toward 1
    //   when revisited.
    //
    //   familiarity = mean over all (modality, predicted_id) of
    //                 state_visit_ema[modality][predicted_id]   ∈ [0, 1]
    //
    //   T = T_base * (1 + pathway_temp_gain * familiarity)
    //         / (1 + dopamine * t_da_gain)
    //
    // High familiarity (predicted trajectory has been visited a lot)
    // pushes T up → more entropic distribution → exploration of
    // alternative intents.  Low familiarity (novel trajectory coming)
    // leaves T at the dopamine-modulated baseline → exploit current
    // best intent.  Default 0 = bit-identical to pre-6.6.I behaviour.
    float       pathway_temp_gain_   = 0.0f;
    float       state_visit_alpha_   = 0.05f;
    std::unordered_map<std::string, std::unordered_map<int, float>> state_visit_ema_;
    std::unordered_map<std::string, std::vector<int>> last_predicted_pathways_;
    float       last_pathway_familiarity_ = 0.0f;

    // Phase 6.6.L — motor coordination noise on the published bilateral
    // (or unilateral) action.  The softmax-weighted bilateral mix is
    // deterministic given the consensus latent; repeated state →
    // identical (L, R) pair → identical per-flagellum spike timing
    // → jerky brain-led locomotion when α is high.  Per-tick uniform
    // noise on each published side mirrors the reflex's continuous
    // wander_noise_amplitude and gives the body's spike sampler enough
    // per-tick variability to coordinate flagella smoothly.
    // Biologically: cerebellar/brainstem-level motor noise on top of a
    // higher cortical command.  Default 0 preserves pre-6.6.L behaviour.
    float       output_noise_amplitude_ = 0.0f;

    // Phase 6.6.O — behavioral cloning from reflex.
    //
    // Premotor subscribes to action.reflex.{left,right} (pure reflex,
    // the teacher demonstration — pre-fade, so brain output doesn't
    // contaminate via self-distillation).  At end of each tick, finds
    // the intent index whose (L, R) pair best matches the observed
    // reflex pair (argmin Euclidean distance over the bilateral
    // intent table) and applies a Hebbian-style update:
    //
    //   gate    = bc_alpha_weighting ? (1 − last_alpha) : 1
    //   W_.row(i*).noalias() += lr_bc * gate * last_latent.transpose()
    //
    // The (1 − α) gating means: at α=1 (full brain authority) no BC
    // drift toward reflex; at α=0 (full reflex) full-strength BC.  This
    // dovetails with the existing update_alpha_threshold gate on
    // reward-driven learning — BC fills exactly the regime Hebbian
    // skips.  The brain learns reflex coordination from its own
    // demonstration stream during reflex-led periods.
    //
    // Independent of apply_reward(): both gradients write into W_
    // additively; no eligibility-trace coupling (BC is instantaneous
    // teacher matching, not delayed credit).
    //
    // Default lr_bc = 0 → bit-identical to pre-6.6.O behaviour.
    // bc_weight_decay (default 0) provides optional homeostasis for
    // long-run stability if BC weights would otherwise grow unbounded.
    std::string bc_reflex_topic_       = "action.reflex";
    std::string bc_reflex_left_topic_  = "action.reflex.left";
    std::string bc_reflex_right_topic_ = "action.reflex.right";
    float       lr_bc_                 = 0.0f;
    bool        bc_alpha_weighting_    = true;
    float       bc_weight_decay_       = 0.0f;
    // Cached observed reflex (filled by handlers, consumed by apply_bc_update).
    float       last_reflex_left_      = 0.0f;
    float       last_reflex_right_     = 0.0f;
    float       last_reflex_uni_       = 0.0f;
    int64_t     last_reflex_left_tick_  = -1;
    int64_t     last_reflex_right_tick_ = -1;
    int64_t     last_reflex_uni_tick_   = -1;
    int         last_bc_intent_         = -1;     // diagnostic; -1 = none
    // Diagnostic histograms — counts increment whenever the corresponding
    // tick selects a particular intent index (BC: matched reflex → intent;
    // chosen: brain's own argmax / sampled action).  Used by the Cell
    // diag stream to verify BC isn't collapsing to one row.
    std::vector<int> bc_intent_counts_;
    std::vector<int> chosen_intent_counts_;
    int              bc_total_updates_ = 0;

    // Phase v5.1 — Monte-Carlo actor-critic.  When mc_lr_ > 0, the
    // Hebbian-shaped reward update is deferred from per-event apply_reward
    // to per-episode finalize_mc_episode.  Per tick, append (latent,
    // distribution, chosen_intent, accumulated_reward_this_tick_) to
    // mc_trajectory_.  Reward events are accumulated into the most
    // recent step (instead of triggering apply_reward immediately).
    // On events.episode_end, walk the trajectory backwards computing
    // G_t = r_t + γ G_{t+1}, optionally normalise (G_t - μ) / σ over the
    // last `advantage_window_` episode returns, then apply the Hebbian
    // form: W += mc_lr * advantage_t * latent_t * distribution_t (per
    // step).  Eliminates the bootstrap-correlation collapse identified
    // in Phase 6.5.36/37 by replacing TD with full-trajectory returns.
    //
    // Default mc_lr=0 → MC mode off; behaviour bit-identical to v4
    // Premotor under all v4 configs (events still flow to apply_reward).
    float       mc_lr_                  = 0.0f;
    float       mc_gamma_               = 0.99f;
    bool        advantage_normalization_= false;
    int         advantage_window_       = 100;
    std::string mc_episode_topic_       = "events.episode_end";
    // Phase v5.1 — score-function REINFORCE update (chosen-only credit
    // with negative penalty for non-chosen, ∝ (𝟙 − p_i)).  Default
    // false = legacy Hebbian-distribution-weighted form (structurally
    // symmetric; needs BC-style asymmetric prior to bootstrap).  When
    // true + advantage_normalization, this is policy-gradient with a
    // within-episode baseline — what real actor-critic looks like.
    bool        mc_reinforce_           = false;

    struct MCStep {
        Eigen::VectorXf latent;
        Eigen::VectorXf distribution;
        int             chosen;
        float           reward;
        int             bucket = -1;   // 2026-05-29 per-bucket bias REINFORCE
        Eigen::Vector2f leg_phase_s = Eigen::Vector2f::Zero();  // Phase A2 — per-step phase
    };
    std::vector<MCStep> mc_trajectory_;
    float               accumulated_reward_this_tick_ = 0.0f;
    std::deque<float>   recent_returns_;          // rolling window for normalisation
    int                 mc_episodes_seen_   = 0;
    float               mc_last_return_     = 0.0f;
    float               mc_return_mean_     = 0.0f;
    float               mc_return_std_      = 1.0f;

    // Phase 6.6.G — bilateral output mode.  When both output_topic_left_
    // and output_topic_right_ are non-empty, Premotor publishes a pair
    // of ActionOut messages (one per side) using intent_accels_left_ /
    // intent_accels_right_ instead of the unilateral intent_accels_.
    // The single-channel action_output_topic_ publish is suppressed in
    // bilateral mode.  PolicyToken is unchanged so HUD and other
    // single-channel consumers stay intact.  Hebbian credit assignment
    // operates on the intent index (no change to apply_reward()).
    //
    // bilateral_table_ accepts a JSON string of the form
    //   "[[L0,R0],[L1,R1],...,[LN-1,RN-1]]"
    // aligned with intent_accels_.  Empty + n_intents == 5 falls back to
    // the design-doc default table:
    //   hard_left  → (+4, -4)
    //   slow_left  → (+2,  0)
    //   neutral    → (+4, +4)
    //   slow_right → ( 0, +2)
    //   hard_right → (-4, +4)
    std::string output_topic_left_  = "";
    std::string output_topic_right_ = "";
    std::string bilateral_table_    = "";
    bool        bilateral_enabled_  = false;
    std::vector<float> intent_accels_left_;
    std::vector<float> intent_accels_right_;

    // Phase 6.18 — N-channel multi-output mode.  Orthogonal to
    // bilateral_enabled_ (which is the 2-channel left/right path with
    // bc_reflex_left/right_topic plumbing for behaviour-cloning).  When
    // `output_topics` param is a non-empty JSON array of M topic strings,
    // multi_enabled_ becomes true and Premotor publishes M ActionOut
    // messages per tick — one per output_topics_multi_[c] using the
    // chosen intent's intent_accels_per_channel_[chosen][c].  bilateral_
    // table is parsed as an n_intents × M matrix in this mode.  Designed
    // for the per-leg variant (4 Premotors × 3 channels = same 12 servo
    // channels as per-servo, but each leg's 3 joints are co-learned by
    // one policy).  BC bilateral plumbing stays disabled in this mode
    // (lr_bc=0 on every picrawler config anyway).
    std::string                       output_topics_param_ = "";  // raw JSON-array string at setup
    bool                              multi_enabled_       = false;
    std::vector<std::string>          output_topics_multi_;       // M topic names
    std::vector<std::vector<float>>   intent_accels_per_channel_; // n_intents × M

    std::mt19937 rng_;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).
};

} // namespace ogma
