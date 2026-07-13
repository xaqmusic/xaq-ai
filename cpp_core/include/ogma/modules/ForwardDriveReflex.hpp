#pragma once

// =============================================================================
// ForwardDriveReflex.hpp  --  Phase 6.6.D.8 honest-wiring constant-thrust pump
// =============================================================================
//
// The simplest possible motor publisher.  Each tick emits an ActionOut with
// configurable equal thrust on action.left and action.right (i.e. forward
// motion, no steering bias).  Used by the modular reflex preset to provide a
// baseline forward drive that other reflexes (WhiskerSteerReflex on contact,
// future StuckSteerReflex when pinned) can override via last-publisher-wins
// semantics.
//
// This module is a stand-in for "the agent has metabolic drive to explore" —
// future phases will replace it with an exploration drive coupled to
// HomeostaticDrive urgency or a learned Premotor pushing forward.  Today's
// purpose is honest wiring: when this module is removed from the graph, the
// agent stops moving forward.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class ForwardDriveReflex : public Module {
public:
    ForwardDriveReflex();
    ~ForwardDriveReflex() override;

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

    int   tick_count()       const { return tick_count_; }
    float thrust()           const { return thrust_; }
    float noise_amplitude()  const { return noise_amplitude_; }

private:
    float thrust_     = 2.0f;   // accel ∈ [-4, 4]; default 2.0 ≈ 50% rate
    int   tick_count_ = 0;
    // Phase 6.6.F — when non-empty, publish a single ActionOut to this
    // topic instead of bilateral action.left/right.  The thrust is
    // symmetric so the single-channel value equals `thrust_`.  Empty
    // default preserves the original bilateral pathway.
    std::string output_topic_ = "";
    // Phase 6.6.G — bilateral output topic redirects.  Defaults preserve
    // the existing action.left / action.right pathway.  Setting these to
    // e.g. action.reflex.left / action.reflex.right routes thrust through
    // a bilateral MotorFader pair.
    std::string output_topic_left_  = std::string(topics::kActionLeft);
    std::string output_topic_right_ = std::string(topics::kActionRight);
    // Phase 6.6.G — exploration noise.  Each tick the bilateral publish
    // adds independent uniform(-noise_amplitude, +noise_amplitude) to
    // each side's thrust.  Restores the per-tick spike-rate variance the
    // body's pre-modular flagellum_base_rate provided, so the agent
    // wanders along curved paths instead of locking into linear segments
    // + sharp escape pulses.  Default 0 preserves legacy behaviour.
    float       noise_amplitude_ = 0.0f;
    uint64_t    master_seed_     = 0;
    std::mt19937 rng_;
};

} // namespace ogma
