#pragma once

// =============================================================================
// StaleConfidenceDecay.hpp  --  v6.0.e Playful Machine principle #4
// =============================================================================
//
// "If an agent stays in one part of the topological graph for too long,
//  it should slowly forget or degrade the confidence of that region,
//  artificially inflating the TLE to force a renewed bout of exploration."
//                              — docs/the_playful_machine_principles.md
//
// Implementation: subscribe to a single RealityToken topic (one EPM's
// `reality.<group>.<modality>` output).  Track how many consecutive ticks
// the same winner_id stays the winning GNG node.  When the streak exceeds
// `idle_threshold_ticks`, ramp a `staleness` score from 0 → 1 over
// `decay_window_ticks` ticks.  Reset to 0 the instant the winner changes.
//
// The output is published as a ReflexGate{value=staleness, active=value>0}
// on `cognition.boredom` (or a configurable topic).  Downstream modules
// (FaderController.lyapunov_gain in v6.0.f, NeurochemState's exploration
// channels later) can read this gate to amplify exploration when the
// agent is stuck on a stale prediction.
//
// The module does NOT mutate the EPM's prototype directly.  Decoupling
// the diagnostic from the actuation lets us A/B the signal on its own
// before committing to a particular consumer.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class StaleConfidenceDecay : public Module {
public:
    StaleConfidenceDecay();
    ~StaleConfidenceDecay() override;

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

    // White-box accessors for tests + HUD.
    int   current_winner_id() const { return current_winner_id_; }
    int   streak_ticks()      const { return streak_ticks_; }
    float staleness()         const { return staleness_; }
    int   reset_count()       const { return reset_count_; }
    int   inputs_received()   const { return inputs_received_; }

private:
    void handle_input(MessagePtr payload);

    // Configuration
    std::string input_topic_       = "reality.kinematic.imu";
    std::string output_topic_      = "cognition.boredom";
    int         idle_threshold_ticks_ = 60;     // ramp begins after this many same-winner ticks
    int         decay_window_ticks_   = 240;    // ramp duration to reach staleness=1.0
    bool        publish_when_zero_    = false;  // emit ReflexGate even when staleness==0

    // Working state
    int   current_winner_id_ = -1;
    int   streak_ticks_      = 0;
    float staleness_         = 0.0f;
    int   reset_count_       = 0;     // diagnostic: # times streak reset
    int   inputs_received_   = 0;     // diagnostic: # RealityTokens seen

    // Per-tick input cache.
    int   pending_winner_id_   = -1;
    bool  pending_input_seen_  = false;
};

} // namespace ogma
