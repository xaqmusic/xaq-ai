#pragma once

// =============================================================================
// ActionDecoder.hpp  --  Module 5 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/ActionDecoder.md
// v3 reference:    src/ami_ogma_v3/action_decoder.py +
//                  src/ami_ogma_v3/active_inference.py (EFEPolicy +
//                  NodeValenceMap, both reinterpreted here as drive-grounded).
//
// The bath's policy module.  Drive-grounded by design (reads drive.errors
// from HomeostaticDrive); reflex_influence and prediction_confidence gates
// from v3's GlobalWorkspaceHarness are deliberately not reinstated.
//
// Three behavioural layers, in priority order:
//
//   1. Active chunk playback.  When MotorRepertoire has dispatched a chunk
//      (use_chunks=true), play through its action sequence tick-by-tick.
//   2. ExplorationDirective override.  When HomeokineticExploration has armed
//      an episode (`exploration.directive.active=true`), emit the directive's
//      held accel; suppresses chunk dispatch and EFE selection for the
//      episode duration.
//   3. Drive-grounded EFE.  Score each action bin by
//        EFE(a) = pragmatic_gain * V(state, proprio)
//                 + epistemic_gain * H(rollout)
//      where V is the v4 valence-as-drive-reduction map and H is the rollout
//      entropy from GNGRollout.
//
// The v3 Probe machine (gated on serotonin > X AND dopamine < Y) was removed
// after Phase 5 falsification — it gated on a satiety proxy rather than on
// architectural failure of the drive/chunk loop, masked chunk dispatch, and
// hurt the easy config without addressing the dead-zone case.  Its job is now
// owned by HomeokineticExploration.
//
// TD credit assignment runs each tick on the previous (state, proprio, action)
// against the current reward_signal from neuro.state.

#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class ActionDecoder : public Module {
public:
    ActionDecoder();
    ~ActionDecoder() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // White-box accessors
    size_t valence_size() const { return valence_.size(); }
    // Phase 6.9.B — the GREEDY (argmax-Q, pre-ε, pre-epistemic) accel for the
    // most-recent state.  Reading the learned POLICY directly, free of the
    // ε-exploration + epistemic-bonus noise that contaminates the emitted steer.
    // corr(food_bearing, greedy_accel) is the clean homing-learning signal.
    float  greedy_accel_diag() const { return last_greedy_accel_; }
    float  latest_authority_diag() const { return latest_authority_; }
    float  plan_entropy_diag() const { return last_plan_entropy_; }       // ~1 explore, ~0 confident
    float  plan_confidence_diag() const { return last_plan_confidence_; }  // softmax prob of chosen action
    // 2026-06-21 corridor-flat diagnosis — is the PRAGMATIC landscape flat?
    //  pref_obs        : the cy (forward scent component) reaching the planner.
    //  obs_states_known: size of the per-state preferred-obs grounding table.
    //  score_spread    : max-min of the last decision's per-action scores
    //                    (≈0 ⇒ flat landscape ⇒ uniform action choice regardless
    //                     of plan_temperature, the real "is it steering" signal).
    //  nodeval_spread  : range of node_value across known states (does the
    //                    grounded value actually VARY by bearing-state?).
    float  latest_pref_obs_diag() const { return latest_pref_obs_; }
    size_t obs_states_known() const { return obs_value_.size(); }
    float  last_score_spread_diag() const { return last_score_spread_; }
    float  nodeval_spread_diag() const {
        if (obs_value_.empty()) return 0.0f;
        bool first = true; float lo = 0.0f, hi = 0.0f;
        for (auto const& [s, v] : obs_value_) {
            if (first) { lo = hi = v; first = false; }
            else { if (v < lo) lo = v; if (v > hi) hi = v; }
        }
        return hi - lo;
    }
    // 2026-06-19 corridor probe — the committed action this option: index, decoded
    // thrust, and the resolved belief state.  Reveals the actor's actual per-state
    // choices (e.g. a reverse-biased exploration tie-break) in the 1-D rig.
    int    commit_action_idx_diag() const { return commit_action_idx_; }
    float  commit_thrust_diag() const {
        if (joint_action_ && commit_action_idx_ >= 0) { float t, h; decode_joint(commit_action_idx_, t, h); return h; }
        return commit_accel_;
    }
    int    state_node_diag() const { return resolve_state().first; }
    size_t hebbian_size() const { return hebbian_.size(); }
    size_t forward_model_size() const { return forward_model_.size(); }
    float  last_action_tle() const { return last_action_tle_; }
    bool   exploration_active() const { return exploration_active_; }
    bool   chunk_playing() const { return chunk_remaining_ > 0; }
    int    active_chunk_id() const { return active_chunk_id_; }
    int    chunk_remaining() const { return chunk_remaining_; }
    int    chunk_dispatch_count() const { return chunk_dispatch_count_; }
    int    entry_history_seen() const { return entry_history_seen_; }
    int    entry_history_size() const { return int(entry_history_.size()); }
    int    entry_match_dispatches() const { return entry_match_dispatches_; }
    int    manual_dispatches()      const { return manual_dispatches_; }
    int    manual_dispatch_misses() const { return manual_dispatch_misses_; }
    int    dispatches_gated_score() const { return dispatches_gated_score_; }
    int    dispatches_gated_match() const { return dispatches_gated_match_; }
    int    dispatches_blocked_unarmed() const { return dispatches_blocked_unarmed_; }
    int    chunks_armed_count() const {
        int n = 0;
        for (auto const& [_, armed] : chunk_armed_) if (armed) ++n;
        return n;
    }
    float  chunk_rearm_threshold() const { return chunk_rearm_threshold_; }
    // v5.4.L age gate accessors.
    int    chunk_dispatch_min_age_ticks() const { return chunk_dispatch_min_age_ticks_; }
    int    dispatches_blocked_too_young() const { return dispatches_blocked_too_young_; }
    int    last_rollout_used_request_id() const { return last_rollout_request_id_; }

private:
    void handle_consensus(std::string_view topic, MessagePtr payload);
    void handle_drive(std::string_view topic, MessagePtr payload);
    void handle_neuro(std::string_view topic, MessagePtr payload);
    void handle_interest(std::string_view topic, MessagePtr payload);
    void handle_proprio_token(std::string_view topic, MessagePtr payload);
    void handle_rollout_result(std::string_view topic, MessagePtr payload);
    void handle_motor_chunks(std::string_view topic, MessagePtr payload);
    void handle_motor_stream(std::string_view topic, MessagePtr payload);
    void handle_exploration(std::string_view topic, MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);
    void handle_pref_obs(std::string_view topic, MessagePtr payload);
    void handle_authority(std::string_view topic, MessagePtr payload);

    float select_action(int state_node, int proprio_node, std::string const& modality);
    // Phase 6.9 — resolve the actor's (state, proprio) pair from the latest
    // tokens, honouring three wiring modes so the nav graph can drive the actor
    // from a SINGLE coarse belief source (no adapter voter):
    //   - legacy joint   (consensus_level >= 0 AND proprio_topic set): both dims live.
    //   - proprio-only   (consensus_level < 0): the proprio EPM winner IS the state;
    //                     proprio dim collapses to 0 → 1-D Q-table (bundle nav path).
    //   - consensus-only (proprio_topic empty): the consensus winner IS the state;
    //                     proprio dim collapses to 0 (voter nav path).
    // Legacy configs (both set) are byte-identical.
    std::pair<int,int> resolve_state() const;
    void  td_update(float reward_signal);
    // Mode-2 (temporal abstraction): close out the current K-tick option with one
    // SMDP update Q(s0,a0) <- R_integrated + gamma*maxQ(s_end).  The per-tick steer
    // critic can't be credited by a multi-tick closing-speed reward (Q-bins stay
    // tied); committing an action for K ticks gives it an integrated, separable
    // reward consequence.  See docs — cell critic-homing finding (commit 994c403).
    void  td_update_option(float reward_integrated, int s_end, int p_end);
    float request_epistemic_entropy(int state_node, std::string const& modality, uint64_t tick_id);
    bool  try_dispatch_chunk(int state_node, uint64_t tick_id);

    // Configuration
    int          consensus_level_              = 0;
    // Phase 6.6.F — configurable action output topic so the brain side
    // can be routed through a MotorFader (publish to "action.brain"
    // instead of the default "action.out").  Default-preserves
    // pre-6.6.F configs.
    std::string  output_topic_                 = std::string(topics::kActionOut);
    std::string  proprio_topic_                = "reality.proprio.imu";
    int          action_bins_                  = 3;
    float        accel_min_                    = -4.0f;
    float        accel_max_                    =  4.0f;
    float        efe_temperature_              = 1.0f;
    float        pragmatic_gain_               = 10.0f;
    float        epistemic_gain_               = 1.0f;
    // 2026-06-18 — Stage-1 active-inference ignition (opt-in; default off keeps
    // CartPole/MountainCar/picrawler byte-identical).  When efe_select_, choose
    // actions by expected free energy over the learned forward model P(s'|s,b):
    // pragmatic = model 1-step lookahead E[V(s')], epistemic = H(s'|s,b) scaled
    // by the curiosity drive (interest) — the epistemic term replaces ε-greedy.
    bool         efe_select_                   = false;
    std::string  interest_topic_               = "";   // ReflexGate (DistressDrive cognition.interest); empty = off
    // 2026-06-18 — pragmatic GROUNDING over observations (active-inference C prior).
    // When pref_obs_topic_ is set, the EFE pragmatic term scores predicted next
    // states by their expected PREFERRED OBSERVATION (per-state EMA of the obs
    // scalar, e.g. scent proximity) instead of the slow-bootstrapping learned
    // value E[V].  "Act to fulfil the preference over observations" — the spark.
    // Empty = legacy E[V] (CartPole/MountainCar/picrawler byte-identical).
    std::string  pref_obs_topic_               = "";   // ProprioToken (e.g. reality.proprio.scent_max, or percept.scent_compass)
    int          pref_obs_index_               = 0;    // which ProprioToken value to read: 0=scent_max/cx, 1=cy(food-ahead), ...
    float        obs_value_alpha_              = 0.05f; // EMA rate for per-state preferred-obs estimate
    float        latest_pref_obs_              = 0.0f;
    // 2026-06-20 — credit-by-authority: this actor's share of the realized body
    // drive ∈[0,1] (from a MotorBus authority topic).  Gates forward-model
    // learning probabilistically so the actor only credits transitions it drove.
    std::string  authority_topic_              = "";    // empty = full learning (bit-identical)
    float        latest_authority_             = 1.0f;
    std::unordered_map<int,float> obs_value_;          // state_node -> EMA(preferred obs)

    // 2026-06-19 — v1 "coxswain" PLANNING actor (opt-in; defaults keep legacy
    // 1-D/1-step behaviour byte-identical → CartPole/MountainCar/picrawler untouched).
    // (1) JOINT 2-D action (turn × thrust) so the two channels are coordinated,
    //     not redundant copies (fixes the bilateral-mirror collapse).
    // (2) MULTI-STEP receding-horizon plan (depth-H argmax tree over the forward
    //     model) so "aim-then-approach" carries value (fixes the 1-step myopia).
    // (3) GREEN-LOOM as a second long-range target gauge alongside scent.
    bool         joint_action_                 = false; // true → action index = turn_bin*thrust_bins + thrust_bin
    int          thrust_bins_                  = 3;
    float        thrust_accel_min_             = -4.0f;
    float        thrust_accel_max_             =  4.0f;
    std::string  thrust_output_topic_          = "";    // cog.thrust (output_topic_ carries cog.steer)
    int          plan_horizon_                 = 1;     // 1 = legacy 1-step; >1 = receding-horizon plan
    float        plan_gamma_                   = 0.9f;  // discount across the planning horizon
    std::string  green_obs_topic_              = "";    // 2nd target gauge, e.g. reality.proprio.green_fraction
    float        latest_green_obs_             = 0.0f;
    float        w_scent_                      = 1.0f;
    float        w_green_                      = 1.0f;
    // 2026-06-20 — anti-freeze: penalty on a LEARNED staying-put action (a
    // visited (s,a) whose model predicts s'==s) in the planner.  A frozen,
    // perfectly-predictable loop is the dark room — disprefer it so a
    // zero-motion action can't be the argmax when a state-changing one exists.
    // Default 0 = bit-identical.  Does NOT penalize UNSEEN (s,a) (those still
    // get the epistemic explore bonus).
    float        stay_penalty_                 = 0.0f;
    // 2026-06-20 — PERSISTENT EXPLORATION (Playful Machine "keep probing"): when
    // >0, the coxswain SAMPLES the first action from softmax(plan_scores /
    // plan_temperature) instead of pure argmax, so exploration never fully
    // decays → the actor keeps trying thrust/turn → escapes both the bootstrap
    // failure (stumble onto food) AND the facing-freeze degeneration.  Default
    // 0 = argmax (bit-identical).
    float        plan_temperature_             = 0.0f;
    // Observability: explore-vs-exploit readout of the planner's softmax.
    // plan_entropy ∈[0,1] (H/ln N of the softmax dist): ~1 = flat = exploring/
    // uncertain, ~0 = peaked = confidently committing a clear best.
    // plan_confidence = softmax prob of the chosen action.  (Both 0/1 in argmax.)
    mutable float last_plan_entropy_           = 0.0f;
    mutable float last_plan_confidence_        = 1.0f;
    mutable float last_score_spread_           = 0.0f;  // max-min per-action plan scores (last decision)
    std::unordered_map<int,float> obs_value_green_;     // state_node -> EMA(green loom)
    int          commit_action_idx_            = -1;    // joint action index held this commit (for fwd-model learning)
    void  handle_green_obs(std::string_view topic, MessagePtr payload);
    int   n_actions() const { return joint_action_ ? action_bins_ * thrust_bins_ : action_bins_; }
    float node_value(int s) const;                      // w_scent*scent + w_green*green target for a state
    int   predict_next(int s, int a) const;             // argmax forward_model_[(s,a)] (else s)
    float plan_value(int s, int depth) const;           // recursive depth-H best cumulative target
    int   plan_first_action(int s) const;               // best first joint action of the H-step plan
    void  decode_joint(int idx, float& turn_accel, float& thrust_accel) const;
    float        urgency_exploit_threshold_    = 0.6f;
    float        urgency_exploit_bias_         = 1.5f;
    float        td_lambda_                    = 0.7f;
    float        td_gamma_                     = 0.0f;  // 0 = no bootstrap (V-table behaviour); >0 = Q-learning
    float        eligibility_lambda_           = 0.0f;  // Phase 6.5.3.8 — TD(λ) trace decay; 0 = pure TD(0); ~0.7 typical
    int          eligibility_max_len_          = 12;    // trace depth; raise to bridge a long action→reward delay (only used when λ>0)
    // Mode-2 temporal abstraction.  commit_ticks_ > 1 = hold each selected action
    // for K ticks and learn from the reward integrated over the commitment (SMDP).
    // 1 = legacy per-tick TD (bit-identical).
    int          commit_ticks_                 = 1;
    int          commit_remaining_             = 0;     // ticks left in the current option
    float        commit_accel_                 = 0.0f;  // the held action
    int          commit_start_state_           = -1;    // state when the option began
    int          commit_start_proprio_         = -1;
    std::string  commit_start_modality_;
    float        commit_reward_accum_          = 0.0f;  // integrated reward over the option
    bool         commit_active_                = false; // an option is open (has a valid start)
    float        valence_decay_pos_            = 0.99999f;
    float        valence_decay_neg_            = 0.9999f;
    int64_t      valence_max_size_             = 2000;
    int64_t      hebbian_max_size_             = 2000;
    uint64_t     master_seed_                  = 0;

    // Phase 3 integrations (default off — backwards-compatible).
    bool         use_rollout_                  = false;
    int          rollout_K_                    = 16;
    int          rollout_M_                    = 4;
    bool         use_chunks_                   = false;

    // Working state (latest deliveries)
    std::shared_ptr<const ConsensusToken> latest_consensus_;
    std::shared_ptr<const DriveErrors>    latest_drive_;
    std::shared_ptr<const NeuroState>     latest_neuro_;
    float  latest_interest_                  = 0.0f;   // curiosity / epistemic drive (cognition.interest)
    int    latest_proprio_node_              = -1;

    // Hebbian + valence maps.
    using HebbKey = std::string;   // "modality|prev|cur|proprio"
    std::unordered_map<HebbKey, float> hebbian_;       // velocity_bias
    using ValKey  = std::string;   // "state|proprio|bin"
    std::unordered_map<ValKey, float>  valence_;       // expected drive reduction (Q)

    // Phase 6.5.3.1 — empirical forward model.  For each (state, bin)
    // we accumulate counts of observed next-states.  Used for:
    //   (a) action_tle = 1 - P(s' | prev_state, prev_bin) on each
    //       transition — published as ActionOut.action_tle, mirrors
    //       EPM TLE on the perception side.
    //   (b) per-bin epistemic value in select_action: under-visited
    //       (s, b) pairs get an exploration bonus, replacing the
    //       previous bin-independent rollout entropy term.
    using FwdKey = std::string;   // "state|bin"
    std::unordered_map<FwdKey, std::unordered_map<int, int>> forward_model_;
    int64_t      forward_model_max_size_ = 4000;
    float        last_action_tle_        = 0.0f;
    float        last_greedy_accel_      = 0.0f;   // argmax-Q accel for latest state (diag)

    // Eligibility trace: tuples of (state, proprio, action, modality).
    struct Trace {
        int   state;
        int   proprio;
        float action;
        std::string modality;
    };
    std::deque<Trace> eligibility_;
    int   prev_state_   = -1;
    int   prev_proprio_ = -1;
    float prev_action_  = 0.0f;
    std::string prev_modality_;

    // ExplorationDirective override (set by HomeokineticExploration).
    bool   exploration_active_       = false;
    float  exploration_accel_        = 0.0f;
    int    exploration_ticks_remaining_ = 0;

    // Rollout integration
    uint64_t                                next_rollout_request_id_ = 1;
    uint64_t                                pending_rollout_request_ = 0;
    float                                   pending_rollout_entropy_ = 0.0f;
    bool                                    pending_rollout_filled_  = false;
    uint64_t                                last_rollout_request_id_ = 0;

    // Chunk integration
    std::shared_ptr<const MotorChunks>      latest_chunks_;
    std::deque<float>                       chunk_queue_;
    // v5.4 Phase A — entry-matched chunk dispatch.  Subscribes to a slow
    // keyframe topic (RealityToken from slow consensus EPM); maintains
    // rolling buffer of last K entry-context embeddings; scores chunks
    // by cosine similarity of (history) × (chunk.entry_embeddings); both
    // (or all K) cosines must exceed entry_match_threshold for the chunk
    // to be a candidate.  Empty entry_topic = legacy Beta-only dispatch.
    std::string                             entry_topic_                = "";
    int                                     entry_keyframes_            = 2;
    float                                   entry_match_threshold_      = 0.70f;
    // v5.4.L final — when true, populate entry_history_ from the keyframe
    // RealityToken's winner_prototype instead of latent.  MUST be set
    // matching EpisodicCapture.entry_use_winner_prototype on the capture
    // side or the chunks' entry_embeddings won't share a vector space
    // with current entry_history → cosine match dispatches randomly.
    bool                                    entry_use_winner_prototype_ = false;
    // v5.4.M — short+long entry fusion.  When secondary_entry_topic_
    // is non-empty, the runtime entry_history vectors are concat(primary
    // || secondary).  MUST match EpisodicCapture.secondary_keyframe_topic
    // (chunks' stored entry_embeddings dim must equal current
    // entry_history dim).
    std::string                             secondary_entry_topic_      = "";
    Eigen::VectorXf                         last_secondary_src_;
    // v5.4.L final — last raw latent for slow-EPM republish detection,
    // independent of which field (latent vs winner_prototype) we capture.
    Eigen::VectorXf                         last_entry_raw_;
    std::deque<Eigen::VectorXf>             entry_history_;
    int                                     entry_history_seen_         = 0;
    int                                     entry_match_dispatches_     = 0;
    // v5.4 Phase C — manual chunk playback probe.  HotMutable param;
    // when set to a non-(-1) chunk id, every try_dispatch_chunk call
    // force-dispatches THAT chunk (skip scoring + threshold gates) so
    // operator can fire any chunk from the library to inspect playback
    // behaviour from the UI inspector.  Set back to -1 to resume normal
    // scoring dispatch.  When the chunk_id is unknown to MotorRepertoire
    // (no matching chunk in latest_chunks_), dispatch silently no-ops
    // (try_dispatch_chunk returns false) — operator can verify by
    // watching manual_dispatches counter.
    // 0 = sentinel "manual disabled" (no chunk uses id=0; organic starts
    // at 1, seeds use negative ids).  Any other value force-dispatches it.
    int                                     manual_chunk_id_            = 0;
    int                                     manual_dispatches_          = 0;
    int                                     manual_dispatch_misses_     = 0;
    // v5.4 Phase E (Proposal A) — quality-gated dispatch.  Two
    // HotMutable thresholds tighten which chunks are eligible to fire:
    //   min_chunk_score (default 0.5 = current commit threshold)
    //     — Beta(1,1)-prior success rate must clear this.  Raising it
    //       (e.g., 0.75) keeps fresh chunks (boot score ~0.6-0.67) from
    //       dispatching until they accumulate replay history.
    //   min_entry_match_product (default 0.0 = off)
    //     — for episodic chunks, the PRODUCT of cosine similarities at
    //       every entry position must clear this (in addition to the
    //       per-position entry_match_threshold).  Raising it (e.g.,
    //       0.85) demands a tight overall context match.
    float                                   min_chunk_score_            = 0.5f;
    float                                   min_entry_match_product_    = 0.0f;
    int                                     dispatches_gated_score_     = 0;
    int                                     dispatches_gated_match_     = 0;
    // v5.4.J — chunk armed-state gate (Schmitt trigger on entry-match).
    // Without this, a chunk just-captured from an eat event has
    // entry_embeddings == the post-eat context, which then matches the
    // current entry_history with cosine ≈ 1 — the chunk fires immediately
    // and pulls the agent AWAY from the just-eaten food spot.  The fix:
    // chunks must "see" their entry context leave (match < rearm_threshold)
    // before they're armed to fire again on re-entry (match ≥
    // entry_match_threshold).  Edge-triggered dispatch on the rising edge
    // of context similarity, not on continued context match.
    //
    //   chunk_armed_                 — per-chunk armed bool.  Newly-arrived
    //                                  episodic chunks start armed=false.
    //   chunk_rearm_threshold_       — match must drop below this to arm.
    //                                  Lower than entry_match_threshold_ to
    //                                  give hysteresis; default 0.40.
    //   dispatches_blocked_unarmed_  — diag counter for "would have matched
    //                                  but chunk isn't armed yet".
    //   chunks_armed_count_          — current count of armed chunks
    //                                  (diag readback only).
    //
    // Manual probe (manual_chunk_id != 0) bypasses this gate entirely —
    // operator probes are always allowed regardless of armed state.
    // Seeded chunks (no entry_embeddings) bypass the gate naturally
    // because the gate runs only inside the entry-match branch.
    std::unordered_map<int, bool>           chunk_armed_;
    float                                   chunk_rearm_threshold_      = 0.40f;
    int                                     dispatches_blocked_unarmed_ = 0;
    // v5.4.L — chunk dispatch age gate.  A chunk just crystallised from
    // an eat event has its `entry_embeddings` equal to the post-eat
    // context.  Even with the Schmitt rearm gate, in a degenerate
    // encoding the chunk re-arms immediately and fires on the very next
    // tick — the eat→replay symptom.  This independent gate blocks
    // dispatch of any chunk younger than chunk_dispatch_min_age_ticks_
    // regardless of cosine match.  Manual probes bypass.
    int                                     chunk_dispatch_min_age_ticks_ = 60;
    int                                     dispatches_blocked_too_young_ = 0;
    // v5.4 Phase F (Proposal B) — chunk quality of the currently-replaying
    // chunk.  Set when try_dispatch_chunk picks a chunk; carried through
    // ActionOut.chunk_quality so FaderController can modulate α by it.
    float                                   current_chunk_quality_      = 0.0f;
    // v5.4 Phase G — body-position bookkeeping for currently-playing chunk
    int                                     current_chunk_total_intents_  = 0;
    int                                     current_chunk_body_keyframes_ = 0;
    int                                     current_chunk_playback_per_   = 1;
    // v5.3 Phase B — intent-sequence chunks: when a MotorPlayStream arrives
    // with intents non-empty, ActionDecoder queues these instead of action
    // floats and publishes IntentToken on intent_override_topic_ per tick
    // for the chunk's duration.  Premotor (configured with
    // intent_override_topic) substitutes the chunk's intent for its own
    // softmax sample; bilateral motor emission still happens in Premotor.
    // chunk_remaining_ counts whichever queue is active (only one is
    // populated per dispatch).  intent_override_topic_ default empty
    // disables the path = pre-v5.3 behaviour exact.
    std::deque<int>                         intent_chunk_queue_;
    std::string                             intent_override_topic_   = "";
    // Phase 7.2-EPM — when this radix is non-empty, the chunks stored in
    // MotorRepertoire encode multi-channel combined indices (packed by
    // PolicyChannelAggregator at the same radix).  At dispatch time the
    // ActionDecoder unpacks each tick's combined index into
    // IntentToken.indices so downstream Premotors with intent_channel >= 0
    // can pick their slice.  Empty (default) preserves legacy single-
    // channel chunk behaviour bit-identically.
    std::vector<int>                        intent_channel_radix_;
    int                                     chunk_remaining_         = 0;
    int                                     active_chunk_id_         = -1;
    int                                     chunk_dispatch_count_    = 0;     // cumulative # of chunks dispatched (diag)
    // Most recent BAKED consensus motif id — gates chunk dispatch on
    // perceptual context.  -1 if none seen yet (unbaked drift not tracked).
    int                                     current_consensus_motif_id_ = -1;
    // Phase 6.5.12 — drive urgency cached at every tick, used as the
    // second predicate in try_dispatch_chunk's hybrid trigger check.
    // Updated from latest_drive_->urgency on the same tick that the
    // existing drive subscription delivers.
    float                                   current_drive_urgency_      = 0.0f;
    uint64_t                                next_chunk_request_id_   = 1;
    uint64_t                                pending_chunk_request_   = 0;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).

    // Phase 6.5.2.2 — softmax-sampling RNG for select_action.  Seeded from
    // master_seed_ on setup so paired-seed comparisons remain
    // deterministic.  Sampling temperature is derived from neurochem
    // state (1/serotonin) — see select_action.
    mutable std::mt19937_64 rng_;   // mutable: plan_first_action (const) samples when plan_temperature_>0

public:
    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;   // live viz: plan, state, target obs, committed action
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
