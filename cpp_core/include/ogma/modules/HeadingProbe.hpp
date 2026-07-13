#pragma once

// =============================================================================
// HeadingProbe.hpp  --  isolation harness for the learned action layer (2026-06-21)
// =============================================================================
//
// Feeds the HeadingController a COMMANDED heading that is DECOUPLED FROM FOOD: a
// random WORLD direction, held for hold_ticks, then re-randomized.  Each tick it
// reads the body's own heading (proprioception) and emits the egocentric bearing
// to that world target on output_topic (the same [cx=+right, cy=+forward] token the
// ScentCompass produces), so as the body turns to face the command, the egocentric
// bearing shrinks to ~0 — a proper TRACKING target.
//
// Purpose: separate "action learns to act on ANY heading" (tracking) from "action
// exploits the live-food gradient".  In isolation (no food, no scent) the
// along-heading reward is still well-defined (own velocity · own command), so the
// HeadingController.learn_advance policy can be tested cleanly:
//   - CONVERGES (|bearing| -> ~0 after each switch, time-to-align falling) => the
//     action layer genuinely tracks; an in-arena orbit is a live-food artifact.
//   - ORBITS (|bearing| stuck off-zero) in isolation too => a real tracking limit
//     (then a multi-step value upgrade is warranted).

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class HeadingProbe : public Module {
public:
    HeadingProbe();
    ~HeadingProbe() override;

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

    // White-box accessors (diag).
    float target_world() const { return target_; }       // commanded world heading (rad)
    float ego_bearing()  const { return last_bearing_; } // egocentric bearing to it, [-1,1]
    int   ticks_held()   const { return ticks_held_; }

private:
    void handle_heading(MessagePtr payload);

    std::string output_topic_  = "percept.scent_compass"; // [cx,cy] desired heading (→ HeadingController)
    std::string heading_topic_ = "reality.proprio.heading";
    int         hold_ticks_    = 300;     // ticks a target is held before re-randomizing
    float       sign_          = 1.0f;    // egocentric convention (+1/-1; flip if it spins)
    uint64_t    master_seed_   = 1234;

    std::mt19937 rng_;
    float  heading_      = 0.0f;          // latest body heading (rad)
    float  target_       = 0.0f;          // current commanded world heading (rad)
    int    ticks_held_   = 0;             // ticks remaining on the current target
    bool   started_      = false;
    float  last_bearing_ = 0.0f;          // egocentric bearing emitted this tick, [-1,1]
};

} // namespace ogma
