#pragma once

// =============================================================================
// PlayLoop.hpp  --  Cell task #33: the third policy — GROW the map (epistemic play)
// =============================================================================
//
// Plan: docs/plans-and-designs/cell_play_loop_plan.md. Doctrine: §1 (predict-or-
// die), §2.1 (epistemic foraging), §2.2 (the epistemic term of expected free
// energy), §3 (one directive per loop), §5 (decompose, don't disable).
//
// PlayLoop is "PlaceGraphPlanner MINUS traverse." Where the planner *descends*
// V→food to route to a remembered cache (pragmatic exploit), PlayLoop *ascends*
// novelty→frontier to GROW the shared place-map (epistemic explore). Both are
// thin overlays on the ONE node-creator (the cylinder place-EPM): PlayLoop reads
// the same discrete winner_id, so the ground it discovers (a new baked node) is
// automatically visible to the planner it later routes on. There is no second map.
//
//   1. NOVELTY VALUE FIELD.  V_play[n] = novelty[n]·(1−hab[n]) + γ·max_m V_play[m],
//      novelty = per-node EMA of the place-EPM TLE (predictive-model degradation,
//      §1 — dynamic, high where the world-model is worst, DECAYS as the EPM learns
//      the place; not a visit count). Value-iterated so the gradient points at the
//      least-modelled frontier. Habituation ("recent = boring") suppresses the
//      LOCAL term so the loop SWEEPS instead of orbiting one novelty peak.
//   2. UPHILL-THEN-WANDER.  Climb V_play to the frontier (highest-novelty known
//      node); at the local novelty max, RUN-AND-TUMBLE beyond the mapped graph
//      into unmapped ground (the one thing the planner structurally cannot do —
//      it can only argmax over OBSERVED neighbours). Same run/tumble mechanism as
//      RunTumbleNav, climbing the map's own TLE instead of scent. No scent, no food.
//   3. EPISTEMIC VALUE → the L2 EFEArbiter.  Publishes play_value = the frontier
//      novelty ∈[0,1] (normalised by its own slow-decaying peak, §6 — derived, not
//      hand-set). The arbiter weights it by ENERGY SURPLUS (curiosity is
//      instrumental — play most when FULL, so the map is built before hunger
//      forces a return). Unlike the planner's plan_value it is NOT zeroed while a
//      route exists (play has no food route to exploit).
//   4. EAT-CREDIT.  EMAs whether its episodes lead to real events.eat (the honest
//      success signal salvaged from the retired HomeokineticExploration) — telemetry
//      now, a self-calibration hook later.
//
// SUBTRACTED from the planner (this loop does NOT route to food): food memory +
// value-iteration on food, the route/argmax-to-food policy, plan_value/plan_precision,
// desperation-disconfirm, and DELETED outright the confinement-steer (escape_gain +
// stale clock) — scar tissue that existed only to force the food-routing planner to
// explore against its own pinned value field. PlayLoop's value IS novelty, which
// clears as the EPM learns, so the orbit pathology can't arise (see plan §3).
//
// Default-off (§8): empty output/play_value topics ⇒ no publish; existing configs
// byte-identical. Lives only in the_cell_arbiter*.json.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class PlayLoop : public Module {
public:
    PlayLoop() = default;
    ~PlayLoop() override = default;

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
    int   cur_node()   const { return cur_node_; }
    int   next_node()  const { return next_node_; }
    bool  climbing()   const { return climbing_; }   // routing UP the novelty gradient toward the frontier
    bool  wandering()  const { return wandering_; }  // run-and-tumble BEYOND the frontier (unmapped ground)
    int   n_nodes()    const { return int(value_.size()); }
    float value(int n)      const;
    float novelty(int n)    const;                   // per-node TLE-EMA
    float last_fx()         const { return out_fx_; }
    float last_fy()         const { return out_fy_; }
    float last_play_value() const { return last_play_value_; }  // frontier VALUE ∈[0,1] → arbiter (propagated novelty)
    float novelty_cur()     const;                   // novelty at the current node
    float value_peak()      const { return val_peak_; } // slow-decaying running peak of V_play (the play_value normaliser)
    float hab_cur()         const;
    float max_hab()         const;
    float frontier_bearing() const { return frontier_bearing_; } // outward heading (away from the visited centroid), if defined
    bool  have_frontier()   const { return have_frontier_; }     // is the frontier bearing defined this tick
    float eat_credit()      const { return eat_credit_; }  // EMA of "episodes led to a real eat"
    int   stale_explore()   const { return stale_explore_; }   // ticks since the map last grew
    bool  forced_wander()   const { return forced_wander_; }   // stall-wander overriding the climb

private:
    struct Edge { float sum_sin = 0.0f, sum_cos = 0.0f; int count = 0;
                  float heading() const; };
    struct PlacePos { double sx = 0.0, sy = 0.0; int n = 0; };

    void handle_eat(MessagePtr payload);
    void run_value_iteration();                       // on NOVELTY, not food
    float geo_bearing(int from, int to) const;

    // topics
    std::string place_topic_   = "reality.cognitive.place";
    std::string heading_topic_ = "reality.proprio.heading";
    std::string vel_topic_     = "reality.proprio.vel_ego";
    std::string eat_topic_     = "events.eat";        // GROUND-TRUTH eat → eat-credit telemetry
    std::string output_topic_  = "percept.play_bearing"; // → HeadingController (play channel)
    std::string play_value_topic_ = "";               // → EFEArbiter (reality.cognitive.play_value); empty = no publish (default-off)

    // params
    float gamma_          = 0.85f;    // value-iteration discount (novelty propagation)
    int   vi_sweeps_      = 8;
    // Kalman-lessons Stage 3: which field of the place token is the novelty.  "tle"
    // (default) = the place EPM's dual TLE, byte-identical; "transition_surp" = the
    // EPM's transition surprise (with transition_surprise_kind=logprob: how unexpected
    // this MOVE was, the epistemic quantity play seeks); "quant_error" = the spatial term.
    std::string novelty_source_ = "tle";
    float tle_ema_alpha_  = 0.1f;     // EMA rate of the per-node TLE novelty
    float tle_peak_decay_ = 0.0005f;  // slow decay of the novelty running peak (play_value normaliser, §6)
    float hab_rise_       = 0.1f;     // habituation rise while dwelling (recent = boring)
    float hab_decay_      = 0.002f;   // habituation recovery (~1/hab_decay ticks)
    int   explore_cycle_        = 30;        // run length (ticks) before a tumble decision
    float explore_tumble_range_ = 1.5708f;   // max reorient per tumble (rad)
    // WANDER-BEYOND-THE-FRONTIER (the fix): climbing the novelty gradient alone follows the map's own
    // growth (the place-EPM bakes fresh nodes ahead → always an uphill neighbour → the bug climbs the
    // MAPPED region forever and never pushes into UNMAPPED ground — the confinement observed in quad).
    // When the map has NOT grown (no new node baked) for wander_stall_ticks, FORCE the run-and-tumble
    // WANDER (override the climb): the bug has mapped this region, so commit a forward run to push past
    // the boundary; a newly-baked node (new ground) resets the counter → back to climb. 0 = off (climb-
    // only, prior behaviour). Timescale is the run length (explore_cycle), not a new tuned scale.
    int   wander_stall_ticks_   = 0;
    // FRONTIER-DIRECTED WANDER (the maze-discovery fix): the run-and-tumble wander is otherwise a
    // MEMORYLESS random walk — diffusive, so in a walled maze it orbits the spawn region and finds the
    // gap-behind-the-wall by luck (measured: 5.3× slower to reach the far region than in an open arena).
    // The bug already accumulates a spatial memory it throws away here: per-node odometry positions
    // (place_pos_) weighted by HK habituation (hab_ = "recently visited"). Steer the wander AWAY from the
    // habituation-weighted centroid of visited ground — toward the frontier (Playful Machine boredom /
    // self-avoidance; the epistemic drive made DIRECTIONAL). Drift-robust: the centroid and the current
    // position share the SAME odometry drift (common-mode), so their difference (the outward bearing) is
    // drift-immune. Magnitude is DERIVED, not hand-set: effective bias = frontier_bias · max_hab (§6/no-
    // tuning) — a confidence ramp: no outward pull on a fresh map, full pull once the explored core is
    // established. frontier_bias∈[0,1] is the enable/ceiling; 1.0 (full outward) beat 0.5 and 0 monotonically
    // on discovery (2.5× faster to the far region, A/B lbend). 0 = OFF (memoryless run-and-tumble, Δ=0).
    float frontier_bias_        = 0.0f;
    uint64_t explore_seed_      = 11;
    float pi_cell_size_   = 0.0f;     // >0 = place node IS the odometry grid cell; 0 = use place_topic
    float eat_credit_alpha_ = 0.01f;  // EMA rate for the eat-credit success signal

    // state — the shared place-graph overlay (novelty + habituation, NO food)
    std::unordered_map<int, std::unordered_map<int, Edge>> edges_;
    std::unordered_map<int, float> node_tle_;   // per-node EMA of the place-EPM TLE (novelty)
    std::unordered_map<int, float> hab_;        // per-node HK habituation [0,1] (recent = boring)
    std::unordered_map<int, float> value_;      // V_play (novelty value-iteration)
    double odo_x_ = 0.0, odo_y_ = 0.0;
    std::unordered_map<int, PlacePos> place_pos_;
    int   cur_node_  = -1;
    int   next_node_ = -1;
    int   committed_next_ = -1;
    int   stale_explore_ = 0;   // ticks since the map last GREW (a new node baked); resets on new ground
    bool  forced_wander_ = false;  // telemetry: the stall-wander is overriding the climb this tick
    // run-and-tumble explore state
    std::mt19937 explore_rng_{11};
    float explore_dir_       = 0.0f;
    int   explore_run_ticks_ = 0;
    float explore_tle_start_ = 0.0f;
    bool  explore_active_    = false;
    // turn-commit (anti-dither; mirrors the planner/klino)
    float turn_dir_  = 1.0f;
    bool  turning_   = false;
    float cur_heading_ = 0.0f;
    // eat-credit
    bool  eat_in_window_ = false;
    float eat_credit_    = 0.0f;
    // outputs / telemetry
    bool  climbing_  = false;
    bool  wandering_ = false;
    float frontier_bearing_ = 0.0f;   // outward heading (from the hab-weighted visited centroid to here)
    bool  have_frontier_    = false;  // telemetry: the frontier bearing was defined + biased the wander this tick
    float out_fx_ = 0.0f, out_fy_ = 0.0f;
    float last_play_value_ = 0.0f;
    float val_peak_ = 0.0f;   // slow-decaying running peak of V_play(frontier) — the play_value normaliser (§6)
    float novel_ref_ = 0.0f;  // ABSOLUTE fresh-discovery novelty scale (EMA of node-TLE at map growth) → play_value denominator
};

}  // namespace ogma
