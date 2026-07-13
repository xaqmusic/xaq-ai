#pragma once

// =============================================================================
// ScentHomingLearner.hpp  --  LEARNED scent-homing (Pathway A, active-inference)
// =============================================================================
//
// The principled replacement for the analytic ScentCompass→HeadingController
// reflex. The hand-coded vector-sum is an ORACLE (it computes the gradient
// direction). Here the bug learns to home on scent FROM ACTION-CONSEQUENCE,
// using only honest sensors:
//
//     reality.proprio.scent      (raw 8-nostril ring — NOT a computed bearing)
//     reality.proprio.scent_max  (scalar proximity — "how close")
//          → ScentHomingLearner → percept.scent_homing (learned egocentric heading)
//                                → HeadingController (the de-hacked action layer)
//
// MECHANISM (VQ percept + committed-heading bandit, reward-free):
//   STATE  = an online vector-quantizer clusters the raw ring into PROTOTYPES
//            (a learned categorical percept; reconstruction distance = its own
//            inference error). The winning prototype is the state — "what the
//            scent field around me looks like", with NO gradient computed.
//   ACTION = an egocentric heading sector to AIM (n_action_sectors), emitted as a
//            unit direction [sin θ, cos θ] (HeadingController reads bearing =
//            atan2(cx,cy) → sees θ). Sector 0 = straight ahead.
//   VALUE  = V[proto][action] = EMA(Δ scent_max over a commit window). Climbing
//            the gradient raises scent_max → positive Δ reinforces that
//            (proto,heading). Zero-init (the gradient-follow must EMERGE).
//   SELECT = softmax over V + count-based epistemic bonus 1/(1+visits)
//            (persistent exploration; self-anneals). Hold the chosen heading
//            commit_ticks, then credit + reselect.
//   HIT-TELEPORT: a hit respawns food far → scent_max collapses → a big NEGATIVE
//            Δ would punish the winning heading. Detect the one-tick drop and
//            credit a strong POSITIVE reward (a hit = "this heading was right").
//
// This is the action-consequence (teacher-free) percept BearingEstimator's header
// says it could not be in the OPEN arena — the maze supplies the missing signal
// (Δscent_max). No analytic gradient anywhere. shuffle = the learning-control
// ablation. Default-off / opt-in (byte-identical for non-cell envs).

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class ScentHomingLearner : public Module {
public:
    ScentHomingLearner();
    ~ScentHomingLearner() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors (tests + get_module_metrics).
    int   n_prototypes()      const { return int(protos_.size()); }
    int   last_proto()        const { return last_proto_; }
    int   last_action()       const { return last_action_; }
    float last_value_spread() const { return last_vspread_; }   // rising = learning
    float last_value_max()    const { return last_vmax_; }
    float last_win_progress() const { return last_win_dprog_; }  // last credited Δ
    bool  last_explore_pick() const { return last_eps_pick_; }
    float action_bearing()    const;                             // chosen θ/π ∈[-1,1] (corr vs food dir)
    float value_at(int proto, int action) const;                 // test introspection
    bool  lesioned()          const { return lesioned_; }        // bootstrap dropped → learned-alone

private:
    void handle_ring(MessagePtr payload);
    void handle_progress(MessagePtr payload);
    void handle_hit(MessagePtr payload);
    void handle_bootstrap(MessagePtr payload);

    int   vq_winner();                         // VQ the latest ring → prototype id (grows if novel)
    int   select_action(int proto);            // softmax+epistemic over V[proto][*]
    float action_center(int sector) const;     // sector → egocentric angle θ ∈[-π,π]; sector 0 = 0 (fwd)
    int   sector_of(float cx, float cy) const; // egocentric bearing → action sector (inverse of action_center)

    // --- topics ---
    std::string ring_topic_     = "reality.proprio.scent";      // raw 8-nostril ring
    std::string progress_topic_ = "reality.proprio.scent_max";  // proximity truth (Δ = progress)
    std::string hit_topic_      = "events.hit";                 // the agent's OWN reward (ate) — authoritative hit credit
    std::string output_topic_   = "percept.scent_homing";       // [hx,hy] → HeadingController
    // BOOTSTRAP (de-scaffold discipline §4): a teacher bearing (the analytic compass)
    // that GUIDES exploration while V learns the action-consequence. Pre-lesion the
    // committed action = the compass's sector → the bug forages via the compass AND
    // V[state][that-sector] learns the real Δscent. After lesion_after_ticks the compass
    // is dropped: the committed action = argmax V → foraging runs on the LEARNED
    // action-value alone (the compass is gone at runtime). NOT distillation — V holds the
    // Δscent consequence, not the compass's output, so it survives the lesion. Empty
    // bootstrap_topic = self-drive from tick 0 (cold-start; needs content to forage).
    std::string bootstrap_topic_ = "";        // e.g. percept.scent_compass
    int   lesion_after_ticks_    = -1;        // ≥0 → drop the bootstrap, run on learned V

    // --- params ---
    int   ring_dim_         = 8;
    int   max_prototypes_   = 24;
    float novelty_thresh_   = 0.30f;   // L2 distance above which a new prototype grows (unit-pattern scale)
    float proto_lr_         = 0.05f;   // VQ winner→ring update rate
    // Sensor conditioning so the VQ clusters by ANGULAR PATTERN, not concentration:
    // the raw ring is a tiny directional perturbation on a large common-mode → naive
    // VQ is magnitude-/distance-dominated (the direction-blind trap). Centering removes
    // the common-mode (= proximity, already in scent_max); L2-normalizing makes the
    // state scale-independent. This is whitening, NOT computing the bearing — the
    // cluster→heading mapping is still LEARNED from action-consequence.
    bool  center_ring_      = true;    // subtract the per-tick nostril mean before VQ
    bool  normalize_ring_   = true;    // L2-normalize the centered ring before VQ
    int   n_action_sectors_ = 8;
    int   commit_ticks_     = 6;       // hold a chosen heading this long, then credit + reselect
    float value_lr_         = 0.1f;    // EMA rate toward Δprogress
    // Reward whitening (the no-tuning fix for the tabula-rasa value-scale trap):
    // Δscent_max per window is minuscule and gets swamped by the exploration bonus.
    // Normalize it by its own running magnitude so V lands in O(1) std-of-progress
    // units, comparable to epistemic_gain — the policy can then exploit the gradient.
    bool  reward_norm_      = true;    // whiten Δscent by its running scale
    float reward_scale_lr_  = 0.02f;   // EMA rate of the running |Δ| scale
    float epistemic_gain_   = 0.5f;    // count-based exploration bonus weight
    float temperature_      = 1.0f;    // softmax temp (persistent exploration; <=0 = argmax)
    float signal_floor_     = 1e-4f;   // ring-sum below this = no scent → emit [0,0], don't learn
    float hit_drop_thresh_  = 0.2f;    // one-tick scent_max drop > this = hit-teleport in window
    float r_hit_            = 1.0f;    // reward credited on a hit-in-window (strong positive)
    bool  shuffle_          = false;   // ABLATION: ignore V, pick a random action each commit
    uint64_t master_seed_   = 7;

    // --- latest inputs ---
    std::vector<float> ring_;      // raw nostril ring
    std::vector<float> vq_in_;     // conditioned ring (centered + normalized) → VQ
    bool  have_ring_ = false;
    float boot_cx_ = 0.0f, boot_cy_ = 0.0f;   // bootstrap (compass) bearing
    bool  have_boot_ = false;
    uint64_t tick_count_ = 0;
    bool  lesioned_ = false;
    float smax_      = 0.0f;
    float prev_smax_ = 0.0f;
    bool  have_prev_smax_ = false;

    // --- learned state ---
    std::vector<std::vector<float>> protos_;   // [n][ring_dim]
    std::vector<float> V_;                      // [max_prototypes * n_action]
    std::vector<int>   visits_;                 // [max_prototypes * n_action]

    // --- commit/credit state ---
    bool  have_committed_   = false;
    int   committed_proto_  = 0;
    int   committed_action_ = 0;
    int   ticks_left_       = 0;
    float window_start_smax_ = 0.0f;
    bool  hit_in_window_    = false;
    float reward_scale_     = 0.0f;    // running |Δscent| magnitude (reward whitening)

    mutable std::mt19937 rng_;

    // --- telemetry ---
    int   last_proto_     = -1;
    int   last_action_    = -1;
    float last_vspread_   = 0.0f;
    float last_vmax_      = 0.0f;
    float last_win_dprog_ = 0.0f;
    bool  last_eps_pick_  = false;
};

} // namespace ogma
