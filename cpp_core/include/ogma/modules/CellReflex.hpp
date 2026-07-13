#pragma once

// =============================================================================
// CellReflex.hpp  --  Phase 6.6.G consolidated Cell-environment reflex
// =============================================================================
//
// Single bilateral-output module that reproduces the body-side autonomous
// reflex behaviour from `body_controller.gd` (the legacy
// `the_cell_premotor.json` configuration that the user calls 'correct') as
// a graph module so it works under modular_passive bilateral configs (where
// body autonomy is zeroed).
//
// The legacy body-side logic is continuous-additive, not a mutex state
// machine.  Each tick the body computes:
//
//   rate_left  = base + bias × steer
//   rate_right = base − bias × steer
//
// where `base = flagellum_base_rate (0.5)` and `steer ∈ [-1, +1]` is the
// sum of brain-driven steering and a deficit-modulated stuck-pulse:
//
//   steer = clamp(steer_brain + deficit × pulse_held, -1, 1)
//
// `pulse_held` is resampled every 30 ticks (or on deficit transitioning
// through 0.5) using contact asymmetry to pick a direction and the lerp
// formula  lerp(|rand|, 1.0, tanh(|diff|))  to interpolate between random
// magnitude (no informative contact) and a committed ±1 (strongly biased
// contact).  Scent gradient suppresses the rectification term so the
// agent stops actively fleeing walls when scent is rising.
//
// CellReflex publishes a bilateral pair every tick:
//
//   al = clamp(wander_thrust + noise + steer_amp × steer, accel_min, accel_max)
//   ar = clamp(wander_thrust + noise − steer_amp × steer, accel_min, accel_max)
//
// Under the modular_passive bilateral body interpretation
//   rate_left  = clamp(al / 4, 0, 1)
//   rate_right = clamp(ar / 4, 0, 1)
// this maps onto the legacy formula directly: wander_thrust=2 ↔ base=0.5,
// steer_amp=2 ↔ flagellum_steer_bias=0.5.  Per-tick noise on each side
// reproduces the per-tick Bernoulli spike noise the body's base_rate
// sampling used to provide.
//
// The module also emits the same events the body used to publish so the
// brain's Hebbian credit channel keeps working:
//   events.miss      — whisker-bump, intensity max_w × (1 − scent_factor)
//   events.wall_stuck — fired on each fresh deficit > stuck_severity entry
//
// Optional whisker-steer-as-reflex: a configurable steer_gain on
// (left_max − right_max) can be added to the steer signal alongside the
// stuck pulse.  Default off (matches premotor config); legacy modular
// preset users can re-enable.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class CellReflex : public Module {
public:
    CellReflex();
    ~CellReflex() override;

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

    // White-box accessors for tests + diag.
    float deficit()         const { return last_deficit_; }
    float pulse_held()      const { return pulse_held_; }
    int   pulse_ticks()     const { return pulse_ticks_remaining_; }
    int   stuck_count()     const { return stuck_count_; }
    int   miss_count()      const { return miss_count_; }
    float last_left()       const { return last_al_; }
    float last_right()      const { return last_ar_; }
    float scent_ema_short() const { return scent_ema_short_; }
    float scent_ema_long()  const { return scent_ema_long_; }
    float scent_factor()    const { return last_scent_factor_; }
    float last_steer()      const { return last_steer_; }

private:
    void handle_whisker(std::string_view topic, MessagePtr payload);
    void handle_scent(MessagePtr payload);
    void handle_imu(MessagePtr payload);

    bool is_left(std::string const& topic) const;
    bool is_right(std::string const& topic) const;

    float sample_noise();
    float sample_uniform_signed();   // uniform [-1, 1]

    // ------------------------------------------------------------------
    // Topology
    // ------------------------------------------------------------------
    std::string whisker_topic_prefix_ = "reality.proprio.whisker_";
    std::vector<std::string> left_suffixes_  = {"0", "1", "2"};
    std::vector<std::string> right_suffixes_ = {"3", "4", "5"};
    std::string scent_topic_ = "reality.proprio.scent_max";
    int         scent_index_ = 0;
    std::string imu_topic_   = "reality.proprio.imu";
    int         vx_index_    = 2;
    int         vz_index_    = 3;
    std::string output_topic_left_  = std::string(topics::kActionLeft);
    std::string output_topic_right_ = std::string(topics::kActionRight);
    bool        emit_events_         = true;

    // ------------------------------------------------------------------
    // Wander layer
    // ------------------------------------------------------------------
    float wander_thrust_           = 2.0f;
    float wander_noise_amplitude_  = 0.5f;
    float steer_amp_               = 2.0f;   // multiplier on steer ∈ [-1,1] before adding to al/-ar

    // ------------------------------------------------------------------
    // Stuck-pulse (legacy body-side semantics)
    // ------------------------------------------------------------------
    int   stuck_window_ticks_         = 60;        // 1 second @ 60 Hz
    float stuck_severity_threshold_   = 0.5f;      // legacy 0.5 (deficit transition trigger)
    float stuck_move_speed_reference_ = 3.0f;
    int   stuck_pulse_period_         = 30;        // resample every 30 ticks (legacy)

    // ------------------------------------------------------------------
    // Whisker miss-event emission (replaces body's _publish_reflex_event)
    // ------------------------------------------------------------------
    float miss_threshold_       = 0.30f;
    int   miss_refractory_ticks_ = 30;

    // ------------------------------------------------------------------
    // Optional whisker-steer-as-reflex (default off; brain handles steering)
    // ------------------------------------------------------------------
    float avoid_steer_gain_     = 0.0f;   // 0 = disabled (legacy premotor config behaviour)
    float avoid_threshold_      = 0.30f;

    // ------------------------------------------------------------------
    // Scent gating
    // ------------------------------------------------------------------
    float scent_alpha_short_       = 0.1f;
    float scent_alpha_long_        = 0.001f;
    float scent_long_pos_min_      = 0.001f;   // legacy denominator floor
    float scent_gate_cap_          = 0.5f;     // legacy default 0.5

    // ------------------------------------------------------------------
    // Output clamps + RNG
    // ------------------------------------------------------------------
    float    accel_min_            = -4.0f;
    float    accel_max_            =  4.0f;
    uint64_t master_seed_          = 0;

    // ------------------------------------------------------------------
    // Working state
    // ------------------------------------------------------------------
    std::mt19937 rng_;

    std::unordered_map<std::string, float> last_whisker_values_;
    std::unordered_set<std::string> left_topics_;
    std::unordered_set<std::string> right_topics_;

    float scent_ema_short_         = 0.0f;
    float scent_ema_long_          = 0.0f;
    bool  scent_initialised_       = false;

    std::deque<float> speed_window_;

    // Stuck-pulse state (legacy semantics).
    float pulse_held_              = 0.0f;
    int   pulse_ticks_remaining_   = 0;
    float prev_deficit_            = 0.0f;

    int   miss_refractory_remaining_ = 0;

    int   stuck_count_             = 0;
    int   miss_count_              = 0;
    float last_al_                 = 0.0f;
    float last_ar_                 = 0.0f;
    float last_deficit_            = 0.0f;
    float last_scent_factor_       = 0.0f;
    float last_steer_              = 0.0f;
};

} // namespace ogma
