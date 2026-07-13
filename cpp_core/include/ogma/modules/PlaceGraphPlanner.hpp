#pragma once

// =============================================================================
// PlaceGraphPlanner.hpp  --  Pathway D: the map drives the heading (reasoning)
// =============================================================================
//
// The place-EPM builds a topological map (cylinder place-codes → GNG nodes), but
// that map is write-only: nothing reads it to navigate. This module is the
// consumer that turns the map into a heading — INSECT-HONEST route-following, no
// metric coordinate frame:
//
//   1. EDGE-HEADINGS (route memory). When the bug moves place A→B it records the
//      ABSOLUTE heading it was travelling (circular mean of unit vectors per
//      directed edge). "To go A→B, head this way." The directions are learned by
//      observation; the EPM only knows which places connect, not which way.
//   2. FOOD-MEMORY. On events.hit the current place is tagged with food value
//      (decays slowly). "There was food here."
//   3. VALUE PROPAGATION. A few value-iteration sweeps over the observed graph:
//      V[n] = food[n] + gamma · max_{m : edge n→m} V[m].  Every place gets a
//      "which way to food" gradient — multi-hop routing, no metric.
//   4. POLICY → EGOCENTRIC HEADING. At the current node the next hop is the
//      highest-V neighbour; the output bearing is [sin Δ, cos Δ] with
//      Δ = edge_heading[cur][next] − cur_heading (same egocentric convention the
//      scent compass uses, so HeadingController is unchanged).
//   5. SCENT-vs-PLAN ARBITER. The module IS the arbiter: it republishes the scent
//      bearing while FORAGING, and the plan bearing only while PLANNING — when the
//      bug is hungry, the scent reflex is STALLED (not getting closer), AND a route
//      to remembered food exists. HeadingController.input_topic = this output.
//
// This is the step where the agent stops being chemotaxis-with-a-map and starts
// REASONING over remembered structure: returning to a known food cache when the
// local gradient can't help.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class PlaceGraphPlanner : public Module {
public:
    PlaceGraphPlanner() = default;
    ~PlaceGraphPlanner() override = default;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void                   on_param_change(std::string_view key, ParamValue const& value) override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;

    nlohmann::json diag_snapshot() const override;

    // ---- accessors (tests/telemetry) ----
    int   cur_node()     const { return cur_node_; }
    int   next_node()    const { return next_node_; }
    bool  planning()     const { return planning_; }
    bool  wandering()    const { return wandering_; }
    bool  homing_vision() const { return homing_vision_; }
    float food_value(int n) const;
    float value(int n)      const;
    float edge_heading(int from, int to) const;   // NaN if no such edge
    int   edge_count(int from, int to)   const;
    int   n_nodes()      const { return int(value_.size()); }
    float last_fx()      const { return out_fx_; }
    float last_fy()      const { return out_fy_; }
    float last_plan_value() const { return last_plan_value_; }   // V[target] published to the L2 arbiter
    float last_plan_precision() const { return last_plan_precision_; } // model precision = sharpness of the food belief ∈[0,1]
    float last_plan_novelty()   const { return last_plan_novelty_; }   // frontier novelty routed toward when NOT on a food route ∈[0,1]
    // ---- habituation telemetry (HK "recent = boring") ----
    float hab_cur()      const;   // habituation at the current node (0 if none)
    int   n_nodes_hab()  const { return int(hab_.size()); }
    float max_hab()      const;   // max habituation over all nodes
    float desperation()  const { return desperation_; }  // = hunger (accelerates cache disconfirmation)
    float steer_bias()   const { return steer_bias_; }   // confinement-steer strength (escape_gain·desperation·stale)
    bool  escaping()     const { return steer_bias_ > 0.01f; }  // the confinement steer is meaningfully pulling toward fresh ground
    bool  patrol_mode()  const { return patrol_mode_; }  // coverage-patroller (no food memory) vs the food-memory planner
    bool  route_ceded()  const { return route_ceded_; }  // ceding a blocked route this tick (never dithers on an unexecutable hop)
    int   route_stall()  const { return route_stall_; }  // ticks on the current committed hop without a transition

private:
    struct Edge { float sum_sin = 0.0f, sum_cos = 0.0f; int count = 0;
                  float heading() const; };
    // Path-integration: a per-place position centroid (insect odometry) so edge
    // directions are GEOMETRIC (place A→B bearing) rather than the conflicting
    // heading-of-travel (a place is entered from many directions → that average is mush).
    struct PlacePos { double sx = 0.0, sy = 0.0; int n = 0; };

    void handle_hit(MessagePtr payload);
    void run_value_iteration();
    float geo_bearing(int from, int to) const;   // bearing from place `from` centroid → `to` centroid; NaN if unknown

    // topics
    std::string place_topic_     = "reality.cognitive.place";   // current node (RealityToken.winner_id)
    std::string heading_topic_   = "reality.proprio.heading";   // absolute heading (ProprioToken[0])
    std::string vision_topic_  = "";    // percept.visual_bearing — fine line-of-sight final approach (highest priority when food in view)
    std::string hunger_topic_    = "reality.proprio.hunger";    // hunger (1−energy) weights the food term in V
    std::string eat_topic_       = "events.eat";                // GROUND-TRUTH eat → food memory (NOT events.hit, which is
                                                                // polluted by the scent-progress inference → phantom caches)
    std::string vel_topic_       = "reality.proprio.vel_ego";   // forward speed (values[0]) → path-integration
    std::string output_topic_    = "percept.nav_bearing";       // → HeadingController.input_topic
    // 2026-06-29 — L2 EFE arbiter input: publish the FOOD-ROUTE value (sustained level) as a
    // ProprioToken scalar so the arbiter can score the planner policy. value(next_node) ONLY
    // when food_known && route_exists (a committed route to remembered food), else 0 (merely
    // exploring). This sustained level — NOT a z-score — is what stops a blind klino from
    // falsely interrupting a steady route. Empty = no publish (default-off compatible).
    std::string plan_value_topic_ = "";                         // → EFEArbiter (reality.cognitive.plan_value)
    // 2026-07-02 — §2.3 model precision + §2.2 epistemic term for the explicit-EFE arbiter:
    //   plan_precision = sharpness of the food belief (1 − H(food_dist)/log N; single cache = 1,
    //     spread/empty → low) → the arbiter's model-precision meter / optional klino search floor.
    //   plan_novelty   = the frontier TLE the planner routes toward when there is NO food route
    //     (else ~0 while exploiting) → the arbiter's g_epist_planner term.
    // Both ProprioToken scalars; empty topic = no publish (default-off compatible).
    std::string plan_precision_topic_ = "";                     // → EFEArbiter (reality.cognitive.plan_precision)
    std::string plan_novelty_topic_   = "";                     // → EFEArbiter (reality.cognitive.plan_novelty)

    // params
    // B.2 wander: when neither foraging (scent bearing present) nor planning (route),
    // drive FORWARD so the bug covers new ground instead of freezing on [0,0]. The
    // saccade tumbles + maps and the whisker avoids walls → run-and-tumble exploration,
    // and a lost bug MOVES until it reaches a place with a route, then plans.
    float wander_thrust_ = 1.0f;    // forward bearing magnitude while exploring
    float vision_floor_  = 0.05f;   // home on vision when |visual bearing| > this (food in view)
    float gamma_         = 0.85f;   // value-iteration discount
    int   vi_sweeps_     = 8;       // sweeps per tick
    float food_reward_   = 1.0f;    // value added to a node on a hit
    float food_decay_    = 0.9995f; // per-tick food-value decay
    // EPISTEMIC exploration (doctrine §1 / Playful Machine "predictive-model degradation"):
    // route toward places the world-model predicts worst. The signal is the place-EPM's TLE
    // (dual = quantisation error + transition surprise) — DYNAMIC: high where unmodelled,
    // decays as the EPM LEARNS the place. Not a visit count. Bootstrapped by klino (its
    // foraging cools crossed places → the frontier stays hot).
    float tle_gain_      = 0.5f;    // β: weight of the per-node TLE in V[n] (0 = pure exploit)
    float tle_ema_alpha_ = 0.1f;    // EMA rate for the per-node TLE
    float tle_peak_decay_ = 0.0005f; // slow decay of the node-TLE running peak (plan_novelty normaliser, §6 — derived not hand-set)
    // HK HABITUATION ("recent regions are boring"). A per-node [0,1] dwell signal that rises
    // fast while the bug sits on a node and recovers slowly when it leaves. Folded into the
    // value field as a SUPPRESSOR of the LOCAL term (food/TLE) — a recently-visited node stops
    // pulling, so value-propagation routes the bug to the least-recent frontier and it SWEEPS
    // the whole map instead of oscillating between two value peaks. The recover timescale
    // (1/hab_decay ≈ 500 ticks) is STRUCTURAL, like the methylation rate in klino: it sets how
    // long the planner covers other ground before a region becomes interesting again.
    float hab_rise_      = 0.1f;    // habituation rise rate while dwelling (fast)
    float hab_decay_     = 0.002f;  // per-tick habituation recovery (slow; ~500-tick timescale)
    // DESPERATION DISCONFIRMATION (the "hungrier → let go of dead caches and search out" drive).
    // A remembered cache the bug DWELLS on but does NOT eat at is disconfirmed — the food is gone
    // (consumed, or moved to the alternate spot). Its food belief decays by disconfirm·hunger·hab:
    // faster the longer it camps (hab) and the hungrier it is (desperation). An empty cache the bug
    // keeps returning to FADES → the value field flattens there → route_exists goes false → the
    // planner drops into its fast run-and-tumble WANDER branch and searches NEW ground, instead of
    // orbiting a dead cache until it starves. A real cache (where it eats) is re-reinforced on the
    // hit, so productive caches persist. This is belief updating from prediction error, not a hunger
    // threshold. Off by default (disconfirm=0 → the exact prior food memory); opt-in per config.
    float disconfirm_  = 0.0f;    // rate of the desperation disconfirmation (0 = off)
    // CONFINEMENT STEER (2026-07-02): late in a run the value field is dense — every node has an uphill
    // neighbour (stale food, TLE, or pure γ-propagation), so route_exists never clears and the planner
    // routes a handful of HABITUATED nodes forever (orbits until it starves) even though it is BORED
    // (habituation saturated) and HUNGRY. escape_gain STEERS the route toward FRESH ground when confined
    // by penalising each neighbour's value by escape_gain·desperation·stale·habituation, so the bug
    // keeps ROUTING (graph-native — scales to complex mazes) but climbs OUT of the exhausted basin
    // instead of orbiting it. 0 = off; ≈0 whenever the map is growing → no foraging regression.
    float escape_gain_  = 0.0f;
    // ==== PATROL MODE (2026-07-07, operator directive): the planner as a COVERAGE PATROLLER ====
    // The planner's job is to BUILD and TRAVERSE its map — visit all known locations, preferring the
    // least-recently-visited — NOT to remember food. Food memory is a liability where the source
    // RELOCATES (food_alternate): a remembered cache goes stale, the planner welds to a phantom route it
    // cannot execute, dithers in place, and starves (observed 2026-07-07). In the 3-loop division play
    // GROWS the map (novelty, high energy) and klino CLOSES on scent; the planner just keeps the bug
    // moving through KNOWN places so klino keeps getting scent chances. patrol_mode drops food+TLE from
    // the value: local becomes a uniform coverage_gain, so V[n] = coverage_gain·(1−hab[n]) + γ·max V
    // routes toward the LEAST-RECENTLY-VISITED known node (the habituation field IS the coverage need).
    // It always flows to a reachable ADJACENT node (edges exist because they were traversed) → never
    // fixates on an unreachable target → never gets stuck. plan_value becomes the frontier coverage need
    // (1−hab[next]) ∈[0,1], hunger-gated in the arbiter (patrol when hungry, yield to klino on scent /
    // to play when full). Default false = the food-memory planner (byte-identical).
    bool  patrol_mode_   = false;
    bool  patrol_fallback_ = false; // HYBRID: keep food memory AND patrol known ground when there is no food route (never silent)
    float coverage_gain_ = 1.0f;   // uniform "want to visit" drive per node in patrol_mode / patrol_fallback (the value scale)
    // ==== ROUTE-EXECUTION ROBUSTNESS (2026-07-07): a capable planner NEVER dithers on a route it cannot
    // execute. In a maze a committed hop can point the bug through a wall (the geometric centroid bearing
    // crosses it) → the bug spins in place trying to face an unreachable target, makes no progress, and
    // starves (observed 2026-07-07). When the current committed hop is taking far longer than the bug's
    // OWN typical hop traversal time (no node transition), the route is BLOCKED → CEDE: publish
    // plan_value=0 so the L2 arbiter hands authority to klino (forage on scent) / play (explore), which
    // MOVE the bug off the stuck spot; a node transition resets the stall and routing resumes. The
    // timeout = stall_factor · (EMA of successful hop durations) — derived from the bug's own dynamics,
    // no fixed tick constant. 0 (default) = off (the exact prior behaviour). ====
    float stall_factor_ = 0.0f;    // cede a route after a hop takes this × the normal traversal time (0 = off)
    int   route_stall_  = 0;       // ticks on the current committed hop with NO node transition
    float hop_ema_      = 0.0f;    // EMA of successful hop durations (the derived timeout scale; 0 = un-bootstrapped)
    bool  route_ceded_  = false;   // telemetry: ceding a blocked route this tick (plan_value forced 0)
    int   stale_ticks_  = 0;      // ticks since the map last GREW (a new node) — the confinement signal:
                                  // ~0 while the bug discovers new ground (foraging/exploring), climbs
                                  // only when it re-treads a known region → gates the escape so it fires
                                  // on SUSTAINED confinement, never on a transient hungry route to real food.
    // EXPLORE locomotion: run-and-tumble climbing the LOCAL place-novelty (TLE). Runs in a
    // committed heading; tumbles to new ground when a run cycle ends without novelty rising
    // (entered already-learned area). Moves (forward run, no crawl) + directed (toward the
    // unmodelled frontier) = homeokinetic exploration, no scent. Same mechanism as RunTumbleNav,
    // climbing the map's own TLE instead of the scent. Gated against the exploit-route by
    // hunger+food-known: curiosity is instrumental — once food is known + hungry, route to it.
    int      explore_cycle_        = 30;       // run length (ticks) before a tumble decision
    float    explore_tumble_range_ = 1.5708f;  // max reorient per tumble (rad); ±π/2 = stay in front
    uint64_t explore_seed_         = 11;
    // PATH-INTEGRATION place-code: when >0, the place node IS the odometry grid cell
    // (fresh + geometric) instead of the lagged panorama winner_id — the panorama lag
    // was tagging food ~5m short of where the bug actually ate. 0 = use place_topic.
    float pi_cell_size_  = 0.0f;

    // state
    std::unordered_map<int, std::unordered_map<int, Edge>> edges_;  // edges_[from][to]
    std::unordered_map<int, float> food_;
    std::unordered_map<int, float> node_tle_;   // per-node EMA of the place-EPM TLE (epistemic novelty)
    std::unordered_map<int, float> hab_;        // per-node HK habituation [0,1] (recent = boring)
    std::unordered_map<int, float> value_;
    // path-integration odometry (drift-prone, self-consistent; scale is arbitrary —
    // bearings are scale-invariant). Per-place centroid → geometric edge directions.
    double odo_x_ = 0.0, odo_y_ = 0.0;
    std::unordered_map<int, PlacePos> place_pos_;
    int   cur_node_  = -1;
    int   prev_node_ = -1;
    int   next_node_ = -1;
    int   committed_next_ = -1;   // sub-goal commitment: held next hop until reached (anti-dither)
    // run-and-tumble explore state
    std::mt19937 explore_rng_{11};
    float explore_dir_       = 0.0f;   // committed absolute run heading (the bug runs toward it)
    int   explore_run_ticks_ = 0;
    float explore_tle_start_ = 0.0f;   // local novelty at the run start (rose? → keep; fell? → tumble)
    bool  explore_active_    = false;
    // turn-commit (mirrors klino): a behind route-target makes the HeadingController's
    // steer dither at ±π (cx≈0,cy<0) → bug stuck oscillating. Latch a turn direction +
    // cap the commanded delta under π so the egocentric x-component stays non-zero.
    float turn_dir_  = 1.0f;
    bool  turning_   = false;
    float cur_heading_ = 0.0f;
    float hunger_ = 0.0f;
    float desperation_  = 0.0f;   // = hunger (telemetry + accelerates disconfirmation of unproductive caches)
    bool  hit_in_window_ = false;
    bool  planning_ = false;
    bool  wandering_ = false;
    bool  homing_vision_ = false;
    float out_fx_ = 0.0f, out_fy_ = 0.0f;
    float last_plan_value_ = 0.0f;   // food-route value last published to the arbiter (0 while exploring)
    // §2.2/§2.3 precision + epistemic (published to the explicit-EFE arbiter)
    float last_plan_precision_ = 0.0f;   // sharpness of the food belief ∈[0,1]
    float last_plan_novelty_   = 0.0f;   // frontier novelty routed toward when NOT on a food route ∈[0,1]
    float tle_peak_ = 0.0f;              // slow-decaying running peak of the node-TLE (the plan_novelty normaliser, §6)
    float steer_bias_ = 0.0f;            // confinement-steer strength this tick (escape_gain·desperation·stale; telemetry)
};

}  // namespace ogma
