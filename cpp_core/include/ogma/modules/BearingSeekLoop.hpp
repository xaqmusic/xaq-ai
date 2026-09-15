// =============================================================================
// BearingSeekLoop.hpp  --  seeking a thing seen only now and then (the duck's things phase, T2)
// =============================================================================
//
// The duck sees a small thing only while it stands: the swept cloud (CloudMap) exists at a stop,
// and its attended thing's bearing reads proximity 0 the moment the body walks.  A loop that
// walked toward things would therefore be silent for the whole walk unless it REMEMBERED where
// the thing was.  This loop does, in the body's own frame of reference: while the bearing is
// live it fixes the thing's position by dead reckoning (its own odometry pose plus the bearing
// and range), and while the bearing is silent it homes to that remembered position, re-aiming
// as the body moves and turns, until it arrives (the remaining range under arrive_m) or forgets
// (its confidence decays to the floor).  The Cell's VisualHomingNav kept an allocentric BEARING
// through an occlusion; the duck has range, so the belief is a POSITION, and arrival is its own.
//
// What it publishes, in the loop unit's five fields (loop_and_arbitration_recipe.md):
//   - the bearing to the thing, [cx = +right, cy = +forward, 0]: the IntentAdapter's heading-
//     reference contract, the same as play's and avoidance's;
//   - its NEED / value in [0,1]: the confidence in the held target (1 while seen, decaying while
//     remembered, 0 when none) -- the arbiter's preference for this loop (hunger_topic);
//   - its HONEST SIGNAL: the remaining range to the target, for LoopCompetence (sign -1: seeking
//     works while the range falls under its own drive).
// Nothing here is a trajectory; the twist prior turns the body toward the reference as it always
// has.  Module absent = byte-identical.
#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class BearingSeekLoop : public Module {
public:
    BearingSeekLoop();
    ~BearingSeekLoop() override;

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
    nlohmann::json diag_lite() const override;
    void           restore_state(nlohmann::json const& s) override;

    bool   have_target() const { return have_target_; }
    bool   seen()        const { return seen_; }
    float  value()       const { return value_; }
    double range_left()  const { return range_left_; }
    int    arrivals()    const { return arrivals_; }
    int    forgets()     const { return forgets_; }
    float  last_cx()     const { return cx_; }
    float  last_cy()     const { return cy_; }
    double target_x()    const { return tx_; }
    double target_y()    const { return ty_; }

private:
    std::string bearing_topic_ = "percept.thing_bearing";      // [vx=+right, vy=+forward, proximity]
    std::string pose_topic_    = "reality.proprio.odom";       // [x, y, yaw] dead-reckoned, the body's own
    std::string output_topic_  = "percept.seek_bearing";
    std::string value_topic_   = "reality.cognitive.seek_value";
    std::string range_topic_   = "reality.cognitive.seek_range";
    double proximity_range_ = 2.5;    // the bearing's proximity is 1 - range / this (CloudMap max_range / things_range)
    float  min_conf_        = 0.02f;  // proximity above this = the thing is in view
    double arrive_m_        = 0.25;   // the remaining range at which the target counts as reached
    double forget_ticks_    = 3000.0; // confidence decays by 1/forget_ticks per tick while the thing is unseen
    float  floor_           = 0.05f;  // ...and the target is dropped below this

    bool   seen_ = false, have_target_ = false;
    double tx_ = 0.0, ty_ = 0.0;          // the remembered position, odometry frame
    double px_ = 0.0, py_ = 0.0, pyaw_ = 0.0;
    bool   have_pose_ = false;
    float  conf_ = 0.0f, value_ = 0.0f, cx_ = 0.0f, cy_ = 0.0f;
    double range_left_ = 0.0;
    int    arrivals_ = 0, forgets_ = 0;
};

} // namespace ogma
