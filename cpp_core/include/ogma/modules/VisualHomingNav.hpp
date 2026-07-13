#pragma once

// =============================================================================
// VisualHomingNav.hpp  --  Cell L2 loop #4: CLOSE on a SEEN food source
// =============================================================================
//
// Plan: docs/plans-and-designs/cell_vision_homing_loop_plan.md. Doctrine: §1
// (predict-or-die — the loop's predictive substrate is a vision-food EPM's TLE),
// §2.1 (pragmatic foraging), §2.2 (shared-unit EFE value → arbiter), §2.3
// (precision/gains from own dynamics), §3 (one directive per loop), §8 (default-
// off, verify the consumer, staged gates).
//
// The visual twin of klino: klino CLOSES on a source it can SMELL (scent scalar
// run-and-tumble); VisualHomingNav CLOSES on a source it can SEE. It is the
// scent-INDEPENDENT food channel the scoping diagnostic identified as the eats
// lever (docs/findings/play_frontier_wander_findings_2026-07-10.md: eats are
// SIGNAL-limited — the close loop already homes to contact WITH scent, but there
// is no food signal where walls block scent).
//
// PIPELINE (EPM-native, like the place loops):
//   host.video.color → VisualBearing(learn_appearance) → percept.visual_bearing
//     [vx=+right, vy=+forward, proximity=green_frac]
//   → EPM(rbf, vision-food modality) → reality.<grp>.vision {winner_id, tle, node_count}
//   → VisualHomingNav (this thin policy overlay)
//
// It is a LOOP, not a reactive visual servo (§1): its predictive component is the
// vision-food EPM's TLE (temporal prediction error on the food bearing — telemetry
// + the §1 stake), and its VALUE is gated by the EPM's INFORMATIVENESS (node_count:
// a developed food-bearing structure vs a degenerate occluded-only node — the
// fusion-PoC "trust tracks informativeness" mechanism, here feeding the arbiter).
// The turn gain is HeadingController-learned; the food appearance is learned by
// VisualBearing (taught by eats); the reach confidence is eat-calibrated (below).
// No hand-set "steer at green with gain k" (§2.3 no-tuning).
//
//   VALUE (→ arbiter, pragmatic, ∈[0,1]) — a DETECTION/DIRECTION confidence, DISTANCE-INDEPENDENT:
//     vision_value = green_gate · informativeness
//       green_gate      = proximity > min_conf ? 1 : 0        (occlusion: no food in view → cede)
//       informativeness = clamp(node_count / node_ref, 0, 1)  (EPM developed food structure = trust)
//   NOT scaled by cap_vision (reach). cap_vision = clamp(proximity/eat_green) is a PROXIMITY signal
//   (big blob = close), high only when klino already SMELLS the food — it squeezed vision out of its
//   own regime. klino's scent encodes range; vision's bearing is a DIRECTION, so its value is "I can
//   lead you to food," distance-independent. cap_vision / eat_green kept for telemetry.
//   The arbiter applies hunger·vision_weight (pragmatic close; hungry → home to seen food).
//
// Default-off (§8): empty output/value topics ⇒ no publish; the arbiter's
// vision_weight=0 keeps every existing composition byte-identical.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class VisualHomingNav : public Module {
public:
    VisualHomingNav() = default;
    ~VisualHomingNav() override = default;

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
    bool  have_food()   const { return have_food_; }         // food in view this tick (green_gate)
    bool  have_target() const { return have_target_; }       // a remembered food target is held (persistence)
    bool  persisting()  const { return persisting_; }        // homing to the remembered target THIS tick (food occluded)
    float tgt_conf()    const { return tgt_conf_; }          // decaying confidence in the remembered target
    float tgt_world_bearing() const { return tgt_world_bearing_; }  // allocentric bearing to the remembered food
    float cap_vision()  const { return cap_vision_; }        // eat-calibrated reach confidence ∈[0,1]
    float eat_green()   const { return eat_green_; }         // learned green_frac at the eat (reach scale)
    float informativeness() const { return informativeness_; } // EPM food-structure trust ∈[0,1]
    float epm_tle()     const { return epm_tle_; }           // the vision-food EPM's TLE (§1 predictive error)
    int   node_count()  const { return node_count_; }
    float value()       const { return value_; }             // vision_value ∈[0,1] → arbiter
    float last_vx()     const { return out_vx_; }
    float last_vy()     const { return out_vy_; }

private:
    void handle_eat(MessagePtr payload);

    // topics
    std::string vision_epm_topic_ = "reality.cognitive.vision"; // the food-bearing EPM (winner_id/tle/node_count)
    std::string bearing_topic_    = "percept.visual_bearing";    // VisualBearing [vx, vy, proximity]
    std::string heading_topic_    = "reality.proprio.heading";   // absolute heading → ego↔allocentric target reprojection
    std::string eat_topic_        = "events.eat";                // eat-calibration teacher
    std::string output_topic_     = "percept.vision_bearing";    // → HeadingController (vision channel)
    std::string value_topic_      = "";                          // → EFEArbiter; empty = no publish (default-off)

    // params (all self-scaled / derived; see header)
    float min_conf_      = 0.02f;    // proximity (green_frac) floor below which → no food in view (occluded)
    float node_ref_      = 4.0f;     // node_count at which informativeness saturates (EPM developed food structure)
    float eat_alpha_     = 0.2f;     // EMA of green_frac at the eat → eat_green_ (the reach scale)
    float green_bootstrap_ = 0.15f;  // pre-calibration reach scale (before the first eat teaches eat_green_)
    float centroid_ema_  = 0.0f;     // >0 smooths the output bearing (anti-jitter); 0 = pass-through
    // VISUAL TARGET PERSISTENCE (object permanence): food is FOV-gated (in view only when the bug faces it +
    // line-of-sight), so a per-tick reactive value flickers 0↔1 and vision can't sustain arbiter authority to
    // complete an approach (measured: wins ~2% of ticks → nudges, not homing). Remember the food's ALLOCENTRIC
    // bearing (world_bearing = ego_bearing + heading — invariant under the bug's own rotation), keep HOMING to
    // it while occluded (reproject to egocentric each tick), decaying the confidence. A predictive belief (§1:
    // "I saw food there; act to confirm it's still there"), the visual analog of klino's run-commit. On a real
    // eat the target is fulfilled → dropped (food_alternate moves it). persist_decay = per-tick confidence decay
    // while occluded (the memory timescale — ~1/persist_decay ticks); 0 = OFF (prior per-tick reactive value).
    // DEFAULT OFF: measured NET-NEGATIVE on eats (over-commits to a stale target — rotation-corrected
    // only, not translation — suppressing the exploration that finds food; reactive +170% vs persist −37%).
    float persist_decay_ = 0.0f;     // 0 = reactive (shipped). >0 = opt-in target memory (~1/decay ticks).
    float persist_floor_ = 0.05f;    // drop the target when confidence decays below this

    // state
    float eat_green_ = 0.0f;  bool have_eat_green_ = false;
    bool  eat_pending_ = false;
    float ema_vx_ = 0.0f, ema_vy_ = 0.0f; bool have_ema_ = false;
    float cur_heading_ = 0.0f;
    // persistence (target belief)
    bool  have_target_ = false;
    float tgt_world_bearing_ = 0.0f;   // allocentric bearing to the remembered food
    float tgt_conf_ = 0.0f;            // decaying confidence
    bool  persisting_ = false;         // homing to the remembered target this tick (telemetry)

    // telemetry / outputs
    bool  have_food_        = false;
    float cap_vision_       = 0.0f;
    float informativeness_  = 0.0f;
    float epm_tle_          = 0.0f;
    int   node_count_       = 0;
    float value_            = 0.0f;
    float out_vx_ = 0.0f, out_vy_ = 0.0f;
};

}  // namespace ogma
