#pragma once

// =============================================================================
// ChemotaxisAI.hpp  --  reward-free active-inference chemotaxis controller
// =============================================================================
//
// The Cell-environment port of the picrawler Motor-EPM navigation loop, with
// the SAME brain wiring and only the body morphology swapped:
//
//   picrawler:  reality.proprio.target_compass  →  skid-steer leg differential
//   cell:       reality.proprio.scent_compass   →  differential flagellum thrust
//
// Both perceive an egocentric 2-D bearing to the goal (a body-frame unit
// vector [x = +right, y = +forward]) and steer to drive its lateral component
// to zero — Friston's active-inference closure, acting to null the agent's own
// perceived prediction error.  NO reward, NO reinforcement, NO reward shaping.
//
// The morphology accommodation: the cell senses a chemical *gradient* (8-nostril
// ring, body-side `compute_scent_compass`) where the picrawler was handed a
// ground-truth target bearing; the cell actuates two flagella (action.left /
// action.right) where the picrawler drove 12 leg servos.  The steering math
// below is structurally identical to MotorEPM's nav block.
//
// Body convention (body_controller.gd, differential_paddler + reflex_modular):
//   rate_left  = clamp(action.left  / 4, 0, 1)   per-flagellum spike rate
//   rate_right = clamp(action.right / 4, 0, 1)
//   both spike → forward;  left-only spike → turn RIGHT;  right-only → turn LEFT
//   so steering toward a right-side gradient (cx > 0) means al > ar.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class ChemotaxisAI : public Module {
public:
    ChemotaxisAI();
    ~ChemotaxisAI() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors for tests.
    int   tick_count() const { return tick_count_; }
    bool  last_nav_on() const { return last_nav_on_; }
    float last_left()  const { return last_al_; }
    float last_right() const { return last_ar_; }

private:
    void handle_compass(MessagePtr payload);

    // The egocentric goal bearing from the ScentCompass perception module:
    // [x = +right, y = +forward], RAW (magnitude = gradient confidence).
    std::string nav_topic_   = "percept.scent_compass";
    float       compass_x_   = 0.0f;   // lateral (+ = gradient to the right)
    float       compass_y_   = 0.0f;   // forward (+ = gradient ahead)

    // Active-inference steering — same loop as MotorEPM, on the RAW compass
    // (magnitude = gradient confidence):
    //   raw_mag  = |compass|
    //   nav_on   = raw_mag > min_signal     (confident gradient?)
    //   bearing  = atan2(cx, cy) / pi   ∈ [-1, 1]   (scale-invariant direction)
    //   when nav_on:  steer = nav_gain · bearing,  fwd = base · clamp(0.25+0.75·uy)
    //   when nav_off: steer = 0,                   fwd = base   (drive straight)
    //   action.left  = (fwd + steer) · 4 ;  action.right = (fwd - steer) · 4
    // The low min_signal gate is the morphology accommodation (see the .cpp): a
    // weak/flat gradient (near a nutrient's saturated peak, or far away) → drive
    // straight, punching through the food or exploring, instead of pivoting on a
    // noisy near-peak bearing and orbiting the food forever.
    float nav_gain_    = 0.8f;    // steer strength; sign tunable (>0 = toward gradient)
    float base_thrust_ = 1.0f;    // forward thrust fraction (rate units, ×4 → accel)
    float min_signal_  = 0.06f;   // raw |gradient| below this → no confident bearing → drive straight

    std::string output_topic_left_  = std::string(topics::kActionLeft);
    std::string output_topic_right_ = std::string(topics::kActionRight);

    int   tick_count_  = 0;
    bool  last_nav_on_ = false;
    float last_al_     = 0.0f;
    float last_ar_     = 0.0f;
};

} // namespace ogma
