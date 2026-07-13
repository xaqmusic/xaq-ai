#pragma once

// =============================================================================
// GoalBelief.hpp  --  persistent goal-direction belief (path integration)
// =============================================================================
//
// The first cognitive element of the de-scaffolding plan (Stage 1a): a maintained
// BELIEF about where the goal (food) is, that SURVIVES a momentary loss of the
// sensory signal — the thing the belief-less gradient reflex lacks (which makes it
// OSCILLATE when a pillar occludes the food: the scent is line-of-sight occludable,
// so the instantaneous bearing collapses and the reflex flails).
//
// Mechanism = PATH INTEGRATION (biologically grounded): store the goal direction in
// the WORLD frame using the bug's own heading (proprioception, a real self-sense):
//   on confident perception:  world_goal ← rotate(egocentric bearing, +heading)
//   every tick (output):       egocentric_belief ← rotate(world_goal, −heading)
// The +heading/−heading cancel when perception is fresh (the belief is then an
// identity passthrough of the compass), and when the signal is OCCLUDED the held
// world_goal is re-projected through the CURRENT heading → the belief keeps pointing
// at the remembered food location as the bug turns.  Confidence decays without
// perception, so a long loss eventually fades the belief (→ the controller's
// nav-gate sees a weak heading and explores), but a brief occlusion is bridged.
//
// Publishes [bx, by] (egocentric belief direction, magnitude = confidence) on
// output_topic; the HeadingController consumes it in place of the raw compass.
// Ablatable: point the controller back at percept.scent_compass → the reflex.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class GoalBelief : public Module {
public:
    GoalBelief();
    ~GoalBelief() override;

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
    float belief_x()   const { return belief_x_; }     // egocentric belief dir (+right)
    float belief_y()   const { return belief_y_; }     // (+forward)
    float confidence() const { return confidence_; }
    bool  perceiving() const { return last_perceiving_; }  // confident perception this tick

private:
    void handle_scent(MessagePtr payload);
    void handle_heading(MessagePtr payload);

    std::string scent_topic_   = "percept.scent_compass";   // egocentric [cx=+right, cy=+forward]
    std::string heading_topic_ = "reality.proprio.heading"; // absolute yaw (rad)
    std::string output_topic_  = "percept.goal_belief";     // egocentric belief [bx,by]·confidence
    int   cx_index_ = 0, cy_index_ = 1;

    float min_signal_   = 0.1f;    // |compass| above this = confident perception → correct the belief
    float update_rate_  = 0.3f;    // EMA correction toward the observed world direction
    float decay_        = 0.995f;  // per-tick confidence decay when not perceiving (≈140-tick half-life)

    // latest inputs
    float cx_ = 0.0f, cy_ = 0.0f, heading_ = 0.0f;
    bool  have_heading_ = false;

    // world-frame goal direction (unit-ish) + confidence
    float world_gx_ = 0.0f, world_gy_ = 0.0f;
    float confidence_ = 0.0f;

    // telemetry
    float belief_x_ = 0.0f, belief_y_ = 0.0f;
    bool  last_perceiving_ = false;
};

} // namespace ogma
