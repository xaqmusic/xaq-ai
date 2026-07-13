#pragma once

// =============================================================================
// HeadingPlanner.hpp  --  LEARNED heading SELECTION (de-scaffold Stage 1b)
// =============================================================================
//
// Replaces the hardwired heading-SELECTION scaffold: instead of passing the
// scent/belief gradient direction straight through to the HeadingController
// (the chemotaxis reflex), this module LEARNS which heading makes progress and
// SELECTS it.  Sits between GoalBelief and HeadingController:
//     GoalBelief → HeadingPlanner → HeadingController
//
// MECHANISM (tabular, model-free over a committed-heading bandit per state):
//   STATE  = belief-bearing sector: atan2(bx,by) of percept.goal_belief, binned
//            into n_state_sectors.  ("where food is, relative to me", occlusion-
//            bridged by the belief.)
//   ACTION = an egocentric heading sector to AIM (n_action_sectors), emitted as a
//            unit direction [sin θ, cos θ] (so the HeadingController, which reads
//            bearing = atan2(cx,cy), sees exactly θ back — convention LOCKED).
//   VALUE  = V[state][action] = EMA(Δ progress), progress = Δ scent_max over a
//            commit window (reality.proprio.scent_max = body proximity truth).
//            Zero-initialised (honest cold start; the reflex "diagonal" must
//            EMERGE, not be seeded).
//   SELECT = softmax-sample over V + epistemic bonus 1/(1+visits) (persistent
//            exploration; self-anneals as visits accrue).
//
// Load-bearing details (see the plan):
//  - Output magnitude = belief CONFIDENCE (=|[bx,by]|), so the HeadingController's
//    nav-gate still disengages when the belief is lost (Stage-1a behavior kept).
//  - HIT-TELEPORT: food respawns far on a hit → scent_max collapses → a huge
//    NEGATIVE Δ would punish the winning heading.  Detect the one-tick drop and
//    credit a large POSITIVE reward (a hit = strongest "this heading was right").
//  - CREDIT separability: credit the ACHIEVED heading (Δyaw) not the commanded one
//    (credit_mode), so a mis-executing follower can't poison the table.  Default
//    commanded (the open-arena follower is reliable); achieved = a param flip.
//
// GATE: recover ~298 eats/10min in the open corridor (matches the reflex), AND
// prove learning (corr(chosen,fbear) rises early→late + a shuffle ablation
// collapses foraging).  Ablatable: point HeadingController back at
// percept.goal_belief → the reflex.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class HeadingPlanner : public Module {
public:
    HeadingPlanner();
    ~HeadingPlanner() override;

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
    int   last_belief_sector() const { return last_bsec_; }
    int   last_action_sector() const { return last_asec_; }
    float last_value_spread()  const { return last_vspread_; }   // rising = learning
    float last_value_max()     const { return last_vmax_; }
    float last_win_progress()  const { return last_win_dprog_; } // last credited Δ
    bool  last_explore_pick()  const { return last_eps_pick_; }
    float coverage()           const;                            // frac of cells visited
    float action_bearing()     const;                            // chosen heading θ/π ∈[-1,1] (for corr vs fbear)

private:
    void handle_belief(MessagePtr payload);
    void handle_progress(MessagePtr payload);
    void handle_heading(MessagePtr payload);

    int   sector_of(float angle) const;        // angle∈[-π,π] → [0,n_state-1]
    float action_center(int sector) const;     // action sector → center angle ∈[-π,π]
    int   action_sector_of(float angle) const; // angle → [0,n_action-1]

    // --- topics ---
    std::string belief_topic_   = "percept.goal_belief";    // egocentric [bx,by], mag=confidence
    std::string progress_topic_ = "reality.proprio.scent_max"; // proximity truth (Δ = progress)
    std::string heading_topic_  = "reality.proprio.heading"; // abs yaw (achieved-credit)
    std::string output_topic_   = "plan.heading";            // [hx,hy]·conf → HeadingController
    int bx_index_ = 0, by_index_ = 1;

    // --- params ---
    int   n_state_sectors_  = 8;
    int   n_action_sectors_ = 8;
    int   commit_ticks_     = 6;       // K: hold a chosen heading this long (short → less braking-fight)
    float value_lr_         = 0.1f;    // EMA rate toward Δprogress
    float epistemic_gain_   = 0.5f;    // count-based exploration bonus weight
    float temperature_      = 1.0f;    // softmax sampling temp (persistent exploration; <=0 = argmax)
    float min_signal_       = 0.1f;    // belief-confidence floor for a valid state
    bool  credit_achieved_  = false;   // false = credit commanded heading; true = achieved (Δyaw)
    float hit_drop_thresh_  = 0.2f;    // one-tick scent_max drop > this = hit-teleport in window
    float r_hit_            = 1.0f;    // reward credited on a hit-in-window (strong positive)
    bool  allow_resector_abort_ = false; // abort a stale commit if belief sector shifts >1 (default off)
    bool  shuffle_           = false;  // ABLATION: ignore V, pick a random action each commit
    uint64_t master_seed_    = 7;

    // --- latest inputs ---
    float bx_ = 0.0f, by_ = 0.0f, smax_ = 0.0f, yaw_ = 0.0f;
    bool  have_heading_ = false;

    // --- learned tables (row-major [state*n_action + action]) ---
    std::vector<float> V_;
    std::vector<int>   visits_;

    // --- commit/credit state ---
    bool  have_committed_   = false;
    int   committed_state_  = 0;
    int   committed_action_ = 0;
    int   ticks_left_       = 0;
    float window_start_smax_ = 0.0f;
    float window_start_yaw_  = 0.0f;
    bool  hit_in_window_     = false;

    mutable std::mt19937 rng_;

    // --- telemetry ---
    int   last_bsec_      = -1;
    int   last_asec_      = -1;
    float last_vspread_   = 0.0f;
    float last_vmax_      = 0.0f;
    float last_win_dprog_ = 0.0f;
    bool  last_eps_pick_  = false;
    bool  last_committed_ = false;
};

} // namespace ogma
