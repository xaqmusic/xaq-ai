// =============================================================================
// StalkLoop.hpp  --  the hunter's ambush: wait where the mover passes (the duck, design doc §17.113, 2026-10-09)
// =============================================================================
//
// The operator: "the brain needs strong tendencies toward moving objects (like all animal hunters), so the arbiter chooses
// chase/stalk behaviours until truly bored with not finding anything moving."  A chase that loses its mover rarely finds it
// again by following the prediction (9 of 94), and the body is slower than the train; a hunter with a slow body waits where
// the prey passes.
//
//   infers         where movers pass, and whether waiting at such a place will see one
//   senses         the seek loop's confirmed mover sightings (mover_pos_topic) and the motion map's node for each (map_topic,
//                  an EPM over them); the seek loop's target (target_topic) and the mover in mind (mind_topic); the pose
//   predicts       per passing place: its position (the sightings' mean), how often the mover comes back through it and the
//                  interval between its passes (the median), and P(a wait there sees the mover) -- a Beta over its waits
//   honest signal  a wait that sees the mover (a chase starts) or one that outlasts the place's own predicted return
//   need           P of the place chosen (its posterior mean), handed to the seek loop as an ambush target's need: the arbiter
//                  weighs it against play like any target, so failed waits make the hunter bored without a timer
// Publishes ambush_topic [have, x, y (odometry), need, waiting].  The module absent = byte-identical.
#pragma once

#include <deque>
#include <map>
#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class StalkLoop : public Module {
public:
    StalkLoop();
    ~StalkLoop() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    int    state() const { return int(state_); }        // 0 idle, 1 going, 2 waiting
    int    node()  const { return node_; }
    double need()  const { return need_; }
    int    waits_paid()   const { return paid_; }
    int    waits_failed() const { return failed_; }
    int    passes(int w) const { auto it = nodes_.find(w); return it == nodes_.end() ? 0 : it->second.passes; }

private:
    struct Place {
        double sx = 0.0, sy = 0.0; int n = 0;              // the sightings' sum (odometry, metres) and count
        int passes = 0; uint64_t last = 0, pass_start = 0; bool seen = false;
        std::deque<double> intervals;                        // ticks between pass starts, the most recent 16
        double a = 1.0, b = 1.0;                             // P(a wait sees the mover): Beta
        double x() const { return n ? sx / n : 0.0; }
        double y() const { return n ? sy / n : 0.0; }
        double p() const { return a / (a + b); }
        double expected() const;                             // the median interval (ticks); 0 = unknown
    };
    enum class State { Idle, Going, Waiting };

    std::string mover_pos_topic_, map_topic_, target_topic_, mind_topic_, pose_topic_, ambush_topic_;
    int    pass_gap_ticks_ = 250;      // sightings further apart than this start a new pass (the mover-in-mind span)
    int    min_passes_ = 2;            // a place the mover came back through
    double arrive_m_ = 0.25;           // within this of the place the wait begins (the seek loop's renewal reach)

    std::map<int, Place> nodes_;
    uint64_t mp_seen_ = ~uint64_t{0};
    State  state_ = State::Idle;
    int    node_ = -1, last_failed_ = -1;
    uint64_t wait_start_ = 0;
    bool   chased_ = false, chasing_prev_ = false;
    double px_ = 0.0, py_ = 0.0, need_ = 0.0;
    int    paid_ = 0, failed_ = 0, ambushes_ = 0;
};

} // namespace ogma
