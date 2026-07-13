#pragma once

// =============================================================================
// ScentGateReflex.hpp  --  Phase 6.6.D.8 scalar gate primitive
// =============================================================================
//
// Subscribes to a configurable scalar (default reality.proprio.scent_max),
// tracks short-α and long-α EMAs, and publishes a ReflexGate token each tick
// with value = clamp((short - long) / long, 0, cap).  Downstream reflexes
// (WhiskerAversionReflex with suppression_topic set) consume this and scale
// their event-emission magnitude by (1 - value), so wall-aversion is muted
// when the cart is making positive scent progress.  Replaces the body-side
// `_scent_suppress` block in body_controller.gd.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class ScentGateReflex : public Module {
public:
    ScentGateReflex();
    ~ScentGateReflex() override;

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

    float last_value()  const { return last_value_; }
    float short_ema()   const { return short_ema_; }
    float long_ema()    const { return long_ema_; }

private:
    void handle_input(MessagePtr payload);

    std::string input_topic_   = "reality.proprio.scent_max";
    int         input_index_   = 0;
    std::string output_topic_  = "reflex.gate.scent_aversion";
    float       alpha_short_   = 0.1f;
    float       alpha_long_    = 0.001f;
    float       cap_           = 0.5f;
    bool        enabled_       = true;
    float       long_pos_min_  = 0.001f;  // gate inactive until long_ema > this

    bool   ema_initialized_ = false;
    float  short_ema_       = 0.0f;
    float  long_ema_        = 0.0f;
    float  last_value_      = 0.0f;
};

} // namespace ogma
