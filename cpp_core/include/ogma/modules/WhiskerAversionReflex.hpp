#pragma once

// =============================================================================
// WhiskerAversionReflex.hpp  --  Phase 6.6.D.1 reflex-as-module
// =============================================================================
//
// Subscribes to a prefix-pattern of whisker proprio topics (default
// `reality.proprio.whisker_`), tracks the most-recent contact intensity from
// each, and per-tick publishes:
//
//   * events.miss        when max contact > threshold AND not refractory
//   * events.wall_stuck  when max contact > wall_stuck_threshold (no refractory)
//
// Replaces the body-side whisker-aversion event firing in
// godot_host/.../body_controller.gd.  Steering bias (the side-asymmetric
// directed-escape rectification) is NOT yet ported — the substrate doesn't
// have a steering channel; that's a follow-up.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <string>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class WhiskerAversionReflex : public Module {
public:
    WhiskerAversionReflex();
    ~WhiskerAversionReflex() override;

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
    int   miss_count()       const { return miss_count_; }
    int   wall_stuck_count() const { return wall_stuck_count_; }
    float last_max_w()       const { return last_max_w_; }

private:
    void handle_whisker(std::string_view topic, MessagePtr payload);
    void handle_gate(MessagePtr payload);

    std::string whisker_topic_prefix_   = "reality.proprio.whisker_";
    float       threshold_              = 0.30f;
    float       wall_stuck_threshold_   = 0.55f;
    int         refractory_ticks_       = 30;
    // Phase 6.6.D.8 — optional suppression input.  When non-empty, this
    // module subscribes to a ReflexGate token on the named topic; the
    // miss-event intensity is scaled by (1 - gate.value) so wall-aversion
    // softens while a higher-priority signal (e.g. scent climbing) is
    // active.  Replaces the body-side `_scent_suppress` block.
    std::string suppression_topic_      = "";

    // sensor_name -> latest scalar intensity
    std::unordered_map<std::string, float> last_values_;
    int   refractory_remaining_ = 0;
    int   miss_count_           = 0;
    int   wall_stuck_count_     = 0;
    float last_max_w_           = 0.0f;
    float last_gate_value_      = 0.0f;
    bool  last_gate_active_     = false;
};

} // namespace ogma
