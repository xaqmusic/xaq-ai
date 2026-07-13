#pragma once

// =============================================================================
// EFEArbiter.hpp  --  Cell L2: policy selection over competent loops (active inference)
// =============================================================================
//
// The Cell has two individually-competent, individually-honest navigation loops:
//   * klino   (RunTumbleNav)      — the NEAR-FOOD CLOSER (methylation chemotaxis).
//   * planner (PlaceGraphPlanner) — the FAR-FIELD SEARCHER/router (map + value iteration).
//
// This L2 arbiter is the active-inference layer that CHOOSES which loop drives the
// body, by Expected Free Energy — pragmatic (reach food) + epistemic (resolve
// uncertainty) — computed from the agent's OWN beliefs (doctrine §2: the arbiter
// chooses a pathway INTO THE FUTURE). It is NOT a reflex threshold: it scores each
// policy by a VALUE (higher value = lower EFE = better policy) and commits to a
// winner, the low-temperature limit of p(policy) ∝ exp(−G/τ).
//
// EFE values (built from inferred/interoceptive quantities, never a hand rule):
//   raw_klino   = hunger × scent_proximity   (pragmatic: closing on a sensed source
//                 reduces hunger; high ⟺ food near AND hungry; epistemic ≈ 0)
//   raw_planner = the planner's FOOD-ROUTE value (value(next_hop) while there IS a
//                 committed route to remembered food, else 0 while merely exploring)
//
// ASYMMETRIC NORMALISATION — klino is a SPIKE, the planner is a LEVEL (the false-interruption fix):
//   The two channels measure DIFFERENT things and must be normalised differently.
//
//   klino  = MAX( z-SCORE excitement , abs-proximity LEVEL , EAT-CALIBRATED proximity LEVEL ):
//     (a) z-SCORE (§6 — scale-free): running baseline (EMA mean) + variance, then
//       v_spike = (raw_k − mean_k) / (√var_k + std_eps)     // SD above klino's own baseline
//     SPIKES when scent RISES (food smelled) → wins the APPROACH. std_eps floors a flat signal.
//     (b) absolute-proximity LEVEL: v_lvl_abs = clamp(raw_klino, 0, 1)  (raw_klino = hunger·scent
//     is already ~[0,1] — HIGH near a real source AND hungry, ~0 far/sated/flat). SUSTAINS the close
//     when the z-score goes QUIET in stable-high scent — but the scent FIELD is source-normalised
//     (source cell 1.0) while the bug samples a sub-1 scent (~0.73) at its CLOSEST approach, so this
//     level is structurally capped BELOW the planner's self-normalised ≈1 → klino ties/loses → oscillate.
//     (c) EAT-CALIBRATED proximity LEVEL: v_lvl_cal = clamp(hunger · cap_klino, 0, 1), where cap_klino
//     is klino's self-reported confidence = current-smell / its own learned EAT-scent (RunTumbleNav
//     folds the scent at each real hit into eat_scent_). cap→1 the instant the bug is in its own eating
//     range → this level reaches ≈hunger AT the close, lifting the ~0.73 cap to the planner's scale so
//     klino OWNS the close on its own merit. Active only once klino reports (have_cap_).
//     v_klino = max(spike, v_lvl_abs, v_lvl_cal): the MAX means the calibration can only BOOST klino
//     near food, never SILENCE it far (a blind klino → cap→0 → no boost, and the z-spike still forages).
//
//   planner = a SUSTAINED LEVEL normalised by its own slow-decaying peak (NOT a z-score):
//       plan_peak_ = max(raw_planner, plan_peak_·(1 − plan_peak_decay));   // remembers a typical good route's value
//       v_planner  = clamp(raw_planner / (plan_peak_ + eps), 0, 1);         // ~1 while routing to good food, 0 exploring, NEVER negative
//     A plan is VALID for as long as it routes to food — a steady-state property, not a change.
//     The OLD z-score on the planner was the bug: a sustained-high raw_planner pulled its running
//     mean up, so v_planner DECAYED toward 0 and dipped NEGATIVE mid-route; a blind klino at z≈0
//     then read "higher" and broke the route. As a level, v_planner stays ~1 the whole route and
//     never goes negative, so a blind klino cannot overtake it — only a real scent z-spike can.
//
//   This yields the priority:  smelling/at-food klino (spike OR high proximity level) >
//     planner-with-food-route (level ~1) > blind-klino (low scent → low spike AND low level) >
//     planner-just-exploring (level 0).
//   The route HOLDS unless klino genuinely smells/reaches food (its spike-or-level exceeds the route
//   level → clean hand-off to close); klino still wanders/forages when there is no route (v_planner=0).
//
// PLANNER CEDES to DIRECT SENSING (precision-weighting, belt-and-suspenders): v_planner ×= (1 − clamp(hunger·scent)).
//   When the goal is DIRECTLY sensed, the belief-based route is less precise than the observation, so
//   the map is down-weighted by proximity. This runs WITH the eat-calibrated klino level (c): the
//   calibration lifts v_klino to the planner's scale (the decisive win), and the cede lowers v_planner
//   near food (a margin against scent jitter at the eat). Both only fire near real food; a blind klino
//   (low scent) leaves v_planner untouched → the route-hold / false-interruption guard is preserved.
//   (The RELATIVE form ÷scent_peak was decisive but ceded valid FAR routes at ambient scent → reverted.)
//
// (i) CAPABILITY as a BOOST, not a silence GATE (§2.1 nested blanket; bar b):
//   klino is a chemotaxer; it SELF-REPORTS a capability ∈ [0,1] = current-smell / its own learned
//   EAT-scent (EMERGENT from its own consummatory events — never a fixed `if scent<eps` gate, §6). The
//   arbiter consumes it as the calibrated proximity level (c above): a BOOST that lifts v_klino toward
//   the planner's scale near food, so klino owns the close. The old multiplicative SILENCE gate
//   (v_klino = cap · v_klino) is permanently retired: it zeroed a BLIND klino, but klino's blind
//   run-and-tumble IS the forager (the planner cannot close), so silencing it collapsed eats (100%:
//   31→5, 50%: 20→0, n=5 seeds). The boost keeps the forager alive (cap→0 far = no boost, z-spike still
//   forages) while fixing the close. The planner's v_p is left as-is (its value field is its own report).
//
// WINNER-TAKE-ALL + ADAPTIVE HYSTERESIS (§2 a trajectory, §6 no hardcoded dwell):
//   keep the incumbent; switch only when v_challenger − v_incumbent > margin, where
//   margin = hysteresis_k · running_std(v_klino − v_planner). The margin adapts to
//   the gap's own scale — commitment is a property of the dynamics, not a magic count.
//   Output a HARD gain 1.0 to the winner's channel, 0.0 to the loser's.
//
// The MotorBus routes each gain into the effective fader (mix AND authority), so a
// muted channel → authority 0 → its HeadingController advance learning pauses, exactly
// as a reflex taking the bus suppresses cognitive learning (§5). Reflexes never get
// an arbiter gain (safety layer is never muted).
//
//   reality.proprio.hunger ─┐
//   reality.proprio.scent_max ─┼─► EFEArbiter ─► arbiter.gain.klino / arbiter.gain.planner
//   reality.cognitive.plan_value ─┘
//
// ABLATIONS (bar c): force_policy = "" (live arbiter) / "klino" / "planner" / "shuffle"
// (random winner each tick, rng seeded by master_seed varied by tick). All ship with
// the module so the live arbiter can be measured against the fixed + random controls.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class EFEArbiter : public Module {
public:
    EFEArbiter() = default;
    ~EFEArbiter() override = default;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void                   on_param_change(std::string_view key, ParamValue const& value) override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;

    nlohmann::json diag_snapshot() const override;

    // ---- accessors (tests / telemetry) ----
    float raw_klino()    const { return raw_klino_; }
    float raw_planner()  const { return raw_planner_; }
    float v_klino()      const { return v_klino_; }      // MAX(z-spike, abs-proximity level, EAT-calibrated proximity level)
    float v_planner()    const { return v_planner_; }    // LEVEL raw_planner/plan_peak ∈[0,1], then CEDED ×(1−klino proximity) near food
    float mean_klino()   const { return mean_klino_; }   // klino's running raw baseline (the value race)
    float plan_peak()    const { return plan_peak_; }    // planner's slow-decaying peak food-route value (the level's denominator)
    int   winner()       const { return winner_; }          // 0 = klino, 1 = planner, 2 = play
    float gain_klino()   const { return gain_klino_; }
    float gain_planner() const { return gain_planner_; }
    float gain_play()    const { return gain_play_; }
    float gain_vision()  const { return gain_vision_; }
    float margin()       const { return margin_; }
    float hunger()       const { return hunger_; }
    float scent()        const { return scent_; }
    float cap_klino()    const { return cap_klino_; }   // klino's self-reported capability ∈[0,1] (BOOSTS v_klino's level near food)
    // ---- explicit-EFE terms (efe scoring_mode; §2.2/§2.3) ----
    std::string const& scoring_mode() const { return scoring_mode_; }
    float g_prag_klino()    const { return g_prag_klino_; }     // hunger · reach-prob(klino)   — pragmatic, sensory precision
    float g_prag_planner()  const { return g_prag_planner_; }   // hunger · reach-prob(planner) — pragmatic, model precision
    float g_epist_klino()   const { return g_epist_klino_; }    // (1−hunger) · normalised z-spike — klino approach/epistemic
    float g_epist_planner() const { return g_epist_planner_; }  // (1−hunger) · planner frontier novelty (Stage 3; 0 until then)
    float G_klino()         const { return G_klino_; }          // g_prag_klino + g_epist_klino
    float G_planner()       const { return G_planner_; }        // g_prag_planner + g_epist_planner
    float plan_novelty()    const { return plan_novelty_; }     // planner frontier novelty ∈[0,1]
    float plan_precision()  const { return plan_precision_; }   // planner model precision ∈[0,1]
    // ---- play policy (task #33; efe mode) — the epistemic GROW loop ----
    float v_play()        const { return v_play_; }             // selected play score (G_play mirrored)
    float G_play()        const { return G_play_; }             // g_epist_play (pragmatic term ≈ 0 — play doesn't reach food)
    float g_epist_play()  const { return g_epist_play_; }       // play_weight · (1−hunger=energy surplus) · play_value
    float play_value()    const { return play_value_; }         // PlayLoop frontier value ∈[0,1] (epistemic, map-growth potential)
    bool  play_active()   const { return play_active_; }        // play participates in the race (play_weight>0 && wired && efe)

    // ---- vision policy (loop #4; efe mode) — CLOSE on a SEEN source (pragmatic, hunger-weighted) ----
    float v_vision()      const { return v_vision_; }           // selected vision score (G_vision mirrored)
    float G_vision()      const { return G_vision_; }           // g_prag_vision (pragmatic close on seen food)
    float g_prag_vision() const { return g_prag_vision_; }      // vision_weight · hunger · vision_value
    float vision_value()  const { return vision_value_; }       // VisualHomingNav sight-confidence ∈[0,1]
    bool  vision_active() const { return vision_active_; }      // vision in the race (vision_weight>0 && wired && efe)

private:
    void handle_hunger(MessagePtr payload);
    void handle_scent(MessagePtr payload);
    void handle_plan_value(MessagePtr payload);
    void handle_klino_confidence(MessagePtr payload);
    void handle_plan_novelty(MessagePtr payload);
    void handle_plan_precision(MessagePtr payload);
    void handle_play_value(MessagePtr payload);
    void handle_vision_value(MessagePtr payload);

    // topics
    std::string hunger_topic_     = "reality.proprio.hunger";       // klino preference weight
    std::string scent_topic_      = "reality.proprio.scent_max";    // klino scent proximity
    std::string plan_value_topic_ = "reality.cognitive.plan_value"; // planner food-route value (0 while exploring)
    std::string klino_confidence_topic_ = "percept.klino_confidence"; // klino's self-reported capability ∈[0,1]
    std::string plan_novelty_topic_   = "reality.cognitive.plan_novelty";   // planner frontier novelty ∈[0,1] → g_epist_planner (efe)
    std::string plan_precision_topic_ = "reality.cognitive.plan_precision"; // planner model precision ∈[0,1] → klino search floor (efe)
    std::string klino_gain_topic_   = "arbiter.gain.klino";         // winner-take-all gain → MotorBus
    std::string planner_gain_topic_ = "arbiter.gain.planner";
    // task #33 — the third policy (play/GROW). Both default EMPTY so existing configs are
    // byte-identical (no subscription, no publish); the play config wires them.
    std::string play_value_topic_   = "";                           // PlayLoop frontier value ∈[0,1] (epistemic)
    std::string play_gain_topic_    = "";                           // winner-take-all gain → MotorBus (play channel)
    // loop #4 — the vision policy (VisualHomingNav/CLOSE on a SEEN source). Both default EMPTY so
    // existing configs are byte-identical (no subscription, no publish); the vision config wires them.
    std::string vision_value_topic_ = "";                           // VisualHomingNav sight-confidence ∈[0,1] (pragmatic)
    std::string vision_gain_topic_  = "";                           // winner-take-all gain → MotorBus (vision channel)

    // params (HotMutable unless noted)
    std::string scoring_mode_ = "value_race"; // "value_race" (legacy, default — byte-identical) | "efe" (explicit-EFE precision scoring)
    float z_peak_decay_  = 0.0005f; // SLOW decay of klino's z-spike running peak (the epistemic normaliser z_ref, §6 — derived, not a constant)
    bool  planner_epistemic_ = true;  // efe: include g_epist_planner=(1−hunger)·plan_novelty (ablation: false → Stage-3 coverage A/B)
    // task #33 — play policy epistemic gain. G_play = play_weight·(1−hunger)·play_value (energy-surplus
    // weighted: curiosity is instrumental → play most when FULL). 0 (default) = play INERT: it never
    // participates in the winner race and the 2-policy klino/planner logic is byte-identical. >0 = play
    // is the third policy (Stage 2+). The (1−hunger) energy weighting is structural (no tuned λ, §2.3).
    float play_weight_   = 0.0f;
    // ABLATION (bar c / downvoting): weight play by HUNGER instead of energy surplus — the WRONG-SIGN
    // control (explore when hungry → should REGRESS, proving play-when-FULL is right). Default false.
    bool  play_hunger_weight_ = false;
    // loop #4 — vision policy pragmatic gain. G_vision = vision_weight·hunger·vision_value (pragmatic
    // close on a SEEN source, hunger-weighted like klino). 0 (default) = vision INERT: it never
    // participates in the winner race and the 2/3-policy logic is byte-identical. >0 = the 4th policy.
    float vision_weight_ = 0.0f;
    bool  epistemic_reach_gated_ = true;   // R1: gate epistemic by (1−max reach), not (1−hunger)
    bool  klino_search_floor_ = false; // efe: add g_epist_klino += (1−hunger)·(1−plan_precision) — undirected klino search when the model is imprecise (§1.4 floor; opt-in)
    float mean_alpha_    = 0.01f;   // EMA rate of klino's running baseline (z-score mean); slow → klino stays excited inside the scent field (~100 ticks)
    float var_alpha_     = 0.01f;   // EMA rate of klino's running variance (z-score scale)
    float std_eps_       = 0.02f;   // floor on the z-score denominator so a FLAT klino signal can't blow up
    float plan_peak_decay_ = 0.0005f; // SLOW decay of the planner's food-route peak (the level's denominator); remembers a typical good-route value (~2000-tick timescale)
    float hysteresis_k_  = 1.0f;    // margin = hysteresis_k · running_std(gap); adaptive dwell
    float gap_std_alpha_ = 0.02f;   // EMA rate of the running mean/var of (v_klino − v_planner)
    std::string force_policy_ = ""; // ablation: "" live / "klino" / "planner" / "shuffle"
    uint64_t master_seed_ = 11;     // RNG seed (shuffle control)

    // latest inputs
    float hunger_ = 0.0f;
    float scent_  = 0.0f;
    float plan_value_ = 0.0f;
    float cap_klino_  = 1.0f;   // klino's self-reported capability; 1.0 if topic absent (behavior unchanged when not wired)
    bool  have_cap_   = false;  // true once klino's confidence has been received (else the calibrated-level boost is inert)
    float plan_novelty_   = 0.0f;  // planner frontier novelty ∈[0,1] (efe g_epist_planner; 0 if unwired)
    float plan_precision_ = 0.0f;  // planner model precision ∈[0,1] (efe klino search floor; 0 if unwired)
    float play_value_     = 0.0f;  // PlayLoop frontier value ∈[0,1] (efe g_epist_play; 0 if unwired)
    bool  play_active_    = false; // play_weight>0 && play wired && efe mode — computed in on_setup
    float vision_value_   = 0.0f;  // VisualHomingNav sight-confidence ∈[0,1] (efe g_prag_vision; 0 if unwired)
    bool  vision_active_  = false; // vision_weight>0 && vision wired && efe mode — computed in on_setup

    // klino z-score state (running baseline mean + variance — for the APPROACH spike; the CLOSE
    // is carried by an absolute-proximity level = clamp(raw_klino,0,1), no extra state needed)
    float mean_klino_   = 0.0f;
    float var_klino_    = 0.0f;
    // efe mode: slow-decaying running peak of the positive z-spike (the epistemic normaliser z_ref)
    float z_peak_       = 0.0f;
    // planner LEVEL state: slow-decaying peak of the food-route value (the level's denominator)
    float plan_peak_    = 0.0f;
    // scent-cede state: slow-decaying peak of scent (the strongest food-scent known → cede denominator)

    // adaptive-hysteresis state (running mean + variance of the value gap)
    float gap_mean_ = 0.0f;
    float gap_var_  = 0.0f;
    bool  have_gap_ema_ = false;

    // working / output
    float raw_klino_   = 0.0f, raw_planner_   = 0.0f;
    float v_klino_     = 0.0f, v_planner_     = 0.0f;   // selected policy scores (value_race values, or the EFE G's mirrored, per scoring_mode)
    // explicit-EFE decomposition (efe mode; 0 in value_race mode)
    float g_prag_klino_ = 0.0f, g_prag_planner_ = 0.0f;
    float g_epist_klino_ = 0.0f, g_epist_planner_ = 0.0f;
    float G_klino_ = 0.0f, G_planner_ = 0.0f;
    float v_play_ = 0.0f, g_epist_play_ = 0.0f, G_play_ = 0.0f;   // play policy (task #33)
    float v_vision_ = 0.0f, g_prag_vision_ = 0.0f, G_vision_ = 0.0f;  // vision policy (loop #4)
    int   winner_      = 0;         // 0 = klino (incumbent at start), 1 = planner, 2 = play, 3 = vision
    float gain_klino_  = 1.0f, gain_planner_ = 0.0f, gain_play_ = 0.0f, gain_vision_ = 0.0f;
    float margin_      = 0.0f;

    std::mt19937 rng_{11};
};

}  // namespace ogma
