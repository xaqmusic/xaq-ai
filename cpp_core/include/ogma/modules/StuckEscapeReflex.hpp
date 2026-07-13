#pragma once

// =============================================================================
// StuckEscapeReflex.hpp  --  Phase 6.6.D.4 stuck-detection reflex
// =============================================================================
//
// Subscribes to the body's IMU proprio topic, tracks |v|/move_speed_reference
// over a sliding window of `window_ticks`, and per-tick publishes
// `events.wall_stuck` whenever average severity exceeds `severity_threshold`,
// with a refractory period afterwards.
//
// Replaces the body-side stuck-baseline detection (the
// `_stuck_total_ticks` / `_stuck_severity` block) in body_controller.gd.
// The directed-escape rectification (60-tick pulses with side-asymmetry
// bias) stays body-side until a steering channel exists in the substrate.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <deque>
#include <random>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class StuckEscapeReflex : public Module {
public:
    StuckEscapeReflex();
    ~StuckEscapeReflex() override;

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

    int   stuck_count()     const { return stuck_count_; }
    float severity()        const { return last_severity_; }
    int   pulse_remaining() const { return pulse_remaining_; }
    int   pulse_dir()       const { return pulse_dir_; }

private:
    void handle_imu(MessagePtr payload);
    void handle_ang_vel(MessagePtr payload);
    void handle_efference(MessagePtr payload);
    void handle_energy(MessagePtr payload);

    std::string imu_topic_           = "reality.proprio.imu";
    int         vx_index_            = 2;
    int         vz_index_            = 3;
    // 2026-06-22 — legacy "fuzzier" velocity mode (count rotation as motion).  Kept for
    // old configs; superseded by the mismatch mode below.  ang_vel_topic/ang_weight = 0
    // → translation-only (byte-identical).
    std::string ang_vel_topic_       = "";
    float       ang_weight_          = 0.0f;
    float       latest_ang_speed_    = 0.0f;
    // EFFERENCE-AFFERENT MISMATCH mode (self-normalizing, no velocity threshold to tune).
    // When efference_topic is set, "stuck" = the bug COMMANDS motion but doesn't ACHIEVE
    // it: stuckness = 1 − |afferent|/|intended| (gated on |intended| > intent_floor).
    // Turning/idle command little forward motion → intended≈0 → not stuck.  Wedged →
    // intended high, afferent≈0 → stuck.  The "threshold" is a dimensionless ratio
    // (achieving < a fraction of command), robust across bodies/speeds — not a tuned knob.
    std::string efference_topic_     = "";       // e.g. reality.proprio.motor_efference
    float       intent_floor_        = 0.1f;     // min |intended| to evaluate (else not "trying")
    float       latest_intended_     = 0.0f;
    float       latest_actual_       = 0.0f;
    float       move_speed_reference_ = 3.0f;
    float       severity_threshold_   = 0.5f;
    int         window_ticks_         = 60;
    int         refractory_ticks_     = 60;
    // 2026-06-22 — HUNGER × INACTION trigger (operator): inaction (the |v|-low severity
    // above) is only "stuck" worth perturbing if the bug is ALSO HUNGRY — idle-when-sated
    // is legitimate rest, not stuck; idle-when-hungry is the "get off your butt and eat"
    // trigger.  hunger = clamp((sated_energy − energy)/sated_energy).  Empty hunger_topic
    // = ungated (always fire on inaction, legacy).  noise_mag adds a random ± perturbation
    // to the escape pulse so it doesn't just rotate but jitters the system out of the trap.
    std::string hunger_topic_   = "";       // e.g. reality.proprio.energy
    float       sated_energy_   = 0.8f;
    float       hunger_floor_   = 0.05f;    // hunger must exceed this to fire (else sated → rest)
    float       latest_energy_  = 1.0f;
    float       noise_mag_      = 0.0f;     // random ± noise added to the escape pulse
    // Phase 6.6.G — optional bilateral rotation pulse output.  When the
    // body is stuck without whisker contact (e.g. wedged against a
    // pillar at a non-whisker angle), the only escape is rotation
    // since the spike model can't reverse.  When `enable_pulse` is
    // true, every events.wall_stuck triggers a held bilateral
    // rotation (al = +mag, ar = -mag, sign chosen per event by a
    // master-seeded PRNG) for `pulse_ticks` ticks.  Published to
    // `output_topic_left`/`output_topic_right` so the Cell's bilateral
    // crossfade chain can pick them up as `action.reflex.{left,right}`.
    // 90 ticks at full magnitude = ~115° of rotation under the body's
    // 0.15 rad/s impulse / 0.10 friction-per-tick configuration —
    // enough to clear the user-reported "≥90°" pillar-stuck case.
    bool        enable_pulse_          = false;
    std::string output_topic_left_     = "";  // default empty = use legacy action.left when enabled
    std::string output_topic_right_    = "";
    int         pulse_ticks_           = 90;
    float       pulse_rotation_        = 4.0f;
    // Phase 6.6.G — per-fire duration jitter.  Each stuck event samples
    // its actual pulse_ticks and refractory_ticks from uniform
    // [base * (1 - jitter), base * (1 + jitter)], so the agent's rotate
    // → forward → rotate cycle has variable arc lengths and forward
    // segment durations instead of a metronomic fixed phase.  Default 0
    // = no jitter (legacy fixed-duration behaviour).
    float       duration_jitter_       = 0.0f;
    uint64_t    master_seed_           = 0;

    std::deque<float> speed_window_;
    int   refractory_remaining_ = 0;
    int   stuck_count_           = 0;
    float last_severity_         = 0.0f;
    // Pulse-actuator state.
    int          pulse_remaining_ = 0;
    int          pulse_dir_       = 0;
    std::mt19937 pulse_rng_;
};

} // namespace ogma
