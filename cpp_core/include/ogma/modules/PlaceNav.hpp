#pragma once

// =============================================================================
// PlaceNav.hpp -- the planner reframed as a PLACE/REGION NAVIGATOR
// =============================================================================
//
// Clean-room successor to PlaceGraphPlanner (kept, untouched, for backward compat).
// Reframe (operator, 2026-07-09): in the 3-loop division play GROWS the map, klino
// CLOSES on sensed food, and the planner's job is to NAVIGATE THE KNOWN MAP to a
// remembered food-REGION so klino can finish the close. It is a place navigator,
// NOT a food-value optimizer. This drops the fragile per-node food-reward field
// (additive, immortal caches, dishonest plan_value) that was the root of every
// "routes to a stale/unreachable cache" failure, and replaces it with:
//
//   * SHARED PLACE MAP (reused): directed edges learn the ABSOLUTE heading actually
//     travelled A->B (circular mean); per-place centroids give a geometric fallback
//     bearing. The same map play builds — PlaceNav TRAVERSES it, no private memory.
//   * LOOSE HONEST FOOD TAG: food_tag[node] is BOUNDED (SET to 1 on a real eat, not
//     accumulated). Honest ON-ARRIVAL FORGET (doctrine §8): arrive at a tagged place
//     and no eat fires within a short window -> COLLAPSE the tag (the food is gone /
//     moved). Plus a slow passive fade. A "loose remembrance", never a super-attractor.
//   * VALUE ITERATION TO A TAGGED PLACE: V[n] = hunger*food_tag[n]*(1-hab[n]) +
//     gamma*max_m (V[m] - block_cost[n][m]). The value field routes toward the nearest
//     REACHABLE fresh tag; klino does the precise close (slight food motion within a
//     region no longer breaks it).
//   * REACHABILITY-AWARE ROUTING: a per-edge block_cost rises when a committed hop
//     stalls (points through a wall) and decays slowly (a dropped wall re-opens); it is
//     subtracted in the VI neighbour max, and the bug steers on the heading that ACTUALLY
//     worked (edge.heading()) rather than the straight centroid line that cuts walls.
//   * HONEST plan_value: reach-to-region probability, high ONLY for a fresh tag + an
//     executable (non-stalled) route -> the L2 arbiter is no longer fed a "food reachable
//     this way" lie. plan_novelty (optional) = coverage need when there is no food route.
//   * EXPLORE/BOOTSTRAP: when no fresh-tag route exists, a run-and-tumble on the local
//     place-TLE (hand-off; the arbiter's play/klino also drive here). No wander crawl.
//
//   reality.cognitive.place (winner_id / odometry cell) + heading + hunger + eat + vel
//        -> PlaceNav -> percept.nav_bearing ([fx,fy] -> HeadingController) + plan_value

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class PlaceNav : public Module {
public:
    PlaceNav() = default;
    ~PlaceNav() override = default;

    // ablation controls (validation; "none" = full navigator)
    enum class Ablation { None, NoForget, NoReachCost, ShuffleEdges };

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void                   on_param_change(std::string_view key, ParamValue const& value) override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;

    nlohmann::json diag_snapshot() const override;

    // accessors (tests / telemetry)
    int   cur_node()     const { return cur_node_; }
    int   next_node()    const { return next_node_; }
    bool  planning()     const { return planning_; }
    bool  wandering()    const { return wandering_; }
    int   n_nodes()      const { return int(value_.size()); }
    float food_tag(int n) const;
    float value(int n)    const;
    float edge_heading(int from, int to) const;   // NaN if no such edge
    float block_cost(int from, int to)   const;   // learned per-edge traversal cost (reachability)
    float hab_cur()      const;
    float last_plan_value()   const { return last_plan_value_; }
    float last_plan_novelty() const { return last_plan_novelty_; }
    int   route_stall()  const { return route_stall_; }
    bool  route_ceded()  const { return route_ceded_; }
    int   n_food_tags()  const { return food_tag_count_; }   // # of nodes holding a live food tag
    int   eats_received() const { return eats_received_; }   // lifetime eat events that reached PlaceNav

private:
    struct Edge { float sum_sin = 0.0f, sum_cos = 0.0f; int count = 0; float block = 0.0f;
                  float heading() const; };
    struct PlacePos { double sx = 0.0, sy = 0.0; int n = 0; };

    void run_value_iteration();
    float geo_bearing(int from, int to) const;

    // topics
    std::string place_topic_   = "reality.cognitive.place";
    std::string heading_topic_ = "reality.proprio.heading";
    std::string hunger_topic_  = "reality.proprio.hunger";
    std::string eat_topic_     = "events.eat";
    std::string vel_topic_     = "reality.proprio.vel_ego";
    std::string output_topic_  = "percept.nav_bearing";
    std::string plan_value_topic_   = "";   // -> EFEArbiter reach_planner (empty = no publish)
    std::string plan_novelty_topic_ = "";   // -> EFEArbiter g_epist_planner (coverage need; empty = no publish)

    // params
    float gamma_        = 0.85f;   // value-iteration discount
    int   vi_sweeps_    = 8;       // sweeps per tick
    // loose honest food tag
    float food_decay_   = 0.999f;  // slow passive fade of a tag (per tick)
    float arrival_forget_ = 0.1f;  // multiply a tag by this on arrival-without-eat (honest disconfirm, R1)
    int   arrival_window_ = 60;    // ticks AWAY from a hungrily-foraged tagged node (committed departure)
                                   // before its tag is disconfirmed; filters a closer's weave in/out
    // reachability edge-cost (R2)
    float block_gain_   = 0.5f;    // block_cost added to a hop that stalls
    float block_decay_  = 0.001f;  // per-tick decay of block_cost (a dropped wall re-opens)
    float stall_factor_ = 4.0f;    // a committed hop is BLOCKED after this * (EMA hop duration) with no transition
    // habituation (coverage / anti-oscillation; recent = boring)
    float hab_rise_     = 0.1f;
    float hab_decay_    = 0.002f;
    float coverage_gain_ = 0.3f;   // "want to visit" drive per node when there is NO food route (-> plan_novelty)
    // odometry place-code
    float pi_cell_size_ = 0.0f;    // >0: node = odometry grid cell (fresh + geometric); 0 = place_topic winner_id
    // explore / bootstrap
    int      explore_cycle_        = 30;
    float    explore_tumble_range_ = 1.5708f;
    // ablation
    Ablation ablation_ = Ablation::None;
    uint64_t master_seed_ = 11;

    // graph state
    std::unordered_map<int, std::unordered_map<int, Edge>> edges_;  // edges_[from][to]
    std::unordered_map<int, float> food_tag_;   // bounded [0,1] loose food remembrance
    std::unordered_map<int, float> node_tle_;   // per-node place-EPM TLE EMA (frontier novelty)
    std::unordered_map<int, float> hab_;        // per-node habituation [0,1]
    std::unordered_map<int, float> value_;
    double odo_x_ = 0.0, odo_y_ = 0.0;
    std::unordered_map<int, PlacePos> place_pos_;
    int   cur_node_  = -1;
    int   prev_node_ = -1;
    int   next_node_ = -1;
    int   committed_next_ = -1;

    // arrival-forget state
    int   arrival_node_  = -1;    // the tagged node we most recently arrived at
    int   arrival_ticks_ = 0;     // ticks since arriving at arrival_node_
    bool  ate_since_arrival_ = false;

    // route stall / reachability
    int   route_stall_ = 0;       // ticks on the current committed hop with NO transition
    float hop_ema_     = 0.0f;     // EMA of successful hop durations (derived cede scale)
    bool  route_ceded_ = false;

    // explore run-and-tumble state
    std::mt19937 explore_rng_{11};
    float explore_dir_       = 0.0f;
    int   explore_run_ticks_ = 0;
    float explore_tle_start_ = 0.0f;

    // turn-commit latch (mirrors klino/planner)
    float turn_dir_ = 1.0f;
    bool  turning_  = false;

    // latest inputs
    float cur_heading_ = 0.0f;
    float hunger_ = 0.0f;
    bool  eat_in_window_ = false;

    // outputs / telemetry
    bool  planning_  = false;
    bool  wandering_ = false;
    float out_fx_ = 0.0f, out_fy_ = 0.0f;
    float last_plan_value_ = 0.0f;
    float last_plan_novelty_ = 0.0f;
    int   food_tag_count_ = 0;   // # of nodes with a live tag (telemetry)
    int   eats_received_ = 0;    // lifetime eat events that reached PlaceNav (delivery diagnostic)
};

}  // namespace ogma
