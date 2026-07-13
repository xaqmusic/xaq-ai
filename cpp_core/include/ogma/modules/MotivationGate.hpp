#pragma once

// =============================================================================
// MotivationGate.hpp  --  homeostatic gate on foraging (2026-06-21, Stage 2)
// =============================================================================
//
// "Forage BECAUSE hungry" (the project's homeokinetic thesis). Sits between
// perception (ScentCompass food-bearing) and the action layer (HeadingController):
// it scales the desired-heading MAGNITUDE by the homeostatic deficit (hunger), so
//   - HUNGRY (energy < setpoint)  → full magnitude → the action layer pursues+charges
//   - SATED  (energy ≥ setpoint)  → magnitude → 0 → HeadingController.nav_on=false →
//                                    the bug idles instead of seeking food.
// The DIRECTION is untouched (atan2 is scale-invariant), so the action layer is
// unchanged (clean separation — option (i)): motivation decides WHETHER to forage,
// the action layer decides HOW to act on a heading. Need-modulation is the gate
// test (forage in hunger-gated bursts; sated → idle; energy sustained, no starve).
//
// Reads reality.proprio.energy directly (self-contained; setpoint-relative deficit),
// NOT the combined HomeostaticDrive urgency (which the scent_proximity channel would
// pollute — that tracks "far from food", not "hungry").
//
// gain g = clamp((sated_energy − energy) / sated_energy, 0, 1)  (0 at/above setpoint,
// 1 at empty). freeze_gain ≥ 0 overrides g with a constant = the ABLATION control
// (freeze_gain=1 → always pursue, no need-modulation).

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class MotivationGate : public Module {
public:
    MotivationGate();
    ~MotivationGate() override;

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
    float last_gain()   const { return last_gain_; }    // pursuit gain ∝ hunger
    float last_energy() const { return latest_energy_; }

private:
    void handle_heading(MessagePtr payload);
    void handle_energy(MessagePtr payload);

    std::string heading_topic_ = "percept.scent_compass";    // [cx,cy,(prox)] food-bearing
    std::string energy_topic_  = "reality.proprio.energy";    // scalar [0,1]
    std::string output_topic_  = "percept.motivated_heading"; // → HeadingController
    int         cx_index_      = 0;
    int         cy_index_      = 1;
    float       sated_energy_  = 0.8f;   // homeostatic setpoint (g=0 at/above it)
    float       freeze_gain_   = -1.0f;  // ≥0 → constant gain (ABLATION: 1=always pursue)

    int         n_in_         = 0;       // dims of the last heading token (passthrough)
    float       cx_           = 0.0f;
    float       cy_           = 0.0f;
    float       prox_         = 0.0f;
    bool        have_prox_    = false;
    float       latest_energy_ = 1.0f;
    float       last_gain_     = 0.0f;
};

} // namespace ogma
