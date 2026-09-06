#pragma once

// =============================================================================
// ScentCompass.hpp  --  reward-free chemical-gradient perception
// =============================================================================
//
// The Cell's perception stage for active-inference chemotaxis.  Subscribes the
// raw 8-nostril scent ring (reality.proprio.scent — the body's honest sensor,
// scalar concentration per nostril) and reduces it to a 2-D EGOCENTRIC bearing
// toward the up-gradient direction (percept.scent_compass), which the
// ChemotaxisAI control module steers on.
//
// This is a deliberately transparent, single-responsibility module so the whole
// perception→action chain is visible + auditable in the brain graph:
//     reality.proprio.scent (sensor) → ScentCompass → percept.scent_compass
//                                     → ChemotaxisAI → action.left/right
// (Previously the reduction was computed body-side in GDScript — off-graph and
// crossing the boundary undeclared; moving it here keeps the body a raw-sensor
// publisher and the perception in the auditable brain.)
//
// The reduction: the nostrils sit on a body-local ring at angles 2π·i/N; the
// up-gradient direction is the scent-weighted vector sum of those directions
// (nostrils nearer the source read higher).  Output convention matches the
// picrawler's target_compass exactly: [x = +right, y = +forward]; the cell's
// forward is −Z so the forward component negates the ring's Z weight.  The
// vector is RAW (un-normalized): its magnitude is the gradient strength
// (= confidence), which ChemotaxisAI thresholds on.

#include "ogma/Module.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class ScentCompass : public Module {
public:
    ScentCompass();
    ~ScentCompass() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;   // live viz: bearing + nostril ring
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors for tests.
    float last_cx() const { return cx_; }
    float last_cy() const { return cy_; }
    float last_mag() const { return mag_; }   // raw gradient strength (pre-normalize)
    bool  lesioned() const { return lesioned_; }

private:
    void handle_scent(MessagePtr payload);

    std::string input_topic_  = "reality.proprio.scent";
    std::string output_topic_ = "percept.scent_compass";
    int         nostril_count_ = 8;     // ring positions at angle 2π·i/N
    // When true, append a 3rd output value = mean nostril concentration
    // (PROXIMITY / overall scalar).  The ring vector-sum cancels the common-mode
    // by symmetry, so [cx,cy] is pure DIRECTION (gradient strength); proximity is
    // exactly that discarded common-mode.  A state that includes it lets the
    // critic's value function rise with approach (near>far) → a gradient to climb.
    // Default false = legacy 2-D output (bit-identical for existing consumers).
    bool        emit_proximity_ = false;
    float       proximity_gain_ = 1.0f;  // scale on the appended proximity scalar
    // 2026-06-21 — confidence-gated unit-direction output (fixes the direction-blind
    // RBF-EPM state: raw tiny gradient → magnitude-dominated clustering).
    bool        normalize_direction_ = false;
    float       min_signal_ = 0.0f;      // |grad| floor for a confident direction
    // 2026-06-23 — DROPOUT (sensor-fusion perturbation→recovery demo): when lesioned,
    // emit [0,0] (no scent signal) so the fused agent must ride on vision. Mirrors
    // VisualBearing / BearingEstimator. Default off = bit-identical for existing runs.
    int         lesion_after_ticks_ = -1;   // ≥0 → emit [0,0] from this tick on
    bool        force_lesion_       = false;// immediate lesion (UI toggle)
    // Kalman-lessons Stage 2 perturbation: a NOISY sensor.  From noise_after_ticks
    // on, N(0, noise_sd) is added to the published direction (and proximity) —
    // a channel that moves plenty but is unreliable, the case the voter's
    // expected-error / inverse-variance trust targets (a stuck channel is the
    // activity term's).  Deterministic (derive_rng from master_seed).  <0 = off.
    int         noise_after_ticks_ = -1;
    float       noise_sd_          = 0.0f;
    uint64_t    master_seed_       = 0;
    std::mt19937_64 noise_rng_;
    std::normal_distribution<float> noise_dist_{0.0f, 1.0f};
    bool        noisy_ = false;

    std::vector<float> scent_;          // latest per-nostril concentrations
    float cx_ = 0.0f;                   // +right
    float cy_ = 0.0f;                   // +forward
    float mag_ = 0.0f;                  // raw gradient strength (pre-normalize), diag/gate
    float prox_ = 0.0f;                 // mean concentration (proximity), diag
    bool  lesioned_ = false;
    uint64_t tick_count_ = 0;
};

} // namespace ogma
