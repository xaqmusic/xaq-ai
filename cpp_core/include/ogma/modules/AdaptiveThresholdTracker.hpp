#pragma once

// =============================================================================
// AdaptiveThresholdTracker.hpp  --  Phase 6.6.D.3 generic statistics primitive
// =============================================================================
//
// Subscribes to a configurable scalar input topic, maintains running mean and
// variance EMAs, and publishes a per-tick AdaptiveThreshold token with the
// proposed threshold = mean + N·stddev.  After a configurable warmup-tick
// guard, downstream modules (e.g. DualEMADetector with a wired threshold
// topic) can use this for self-calibrating event detection.
//
// Replaces the body-side `_scent_diff_mean_ema` / `_whisker_mean_ema` blocks
// in body_controller.gd.  Keeping this generic and bus-routed lets one
// instance per signal stream (whisker, scent_diff, urgency) provide adaptive
// cutoffs that other reflex modules can consume.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

// Output payload — broadcast on the configured output_topic each tick.
struct AdaptiveThreshold : Message {
    float mean      = 0.0f;
    float stddev    = 0.0f;
    float threshold = 0.0f;   // mean + n_stddev × stddev
    bool  warm      = false;  // true after warmup_ticks samples received
};

class AdaptiveThresholdTracker : public Module {
public:
    AdaptiveThresholdTracker();
    ~AdaptiveThresholdTracker() override;

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

    float mean()      const { return mean_ema_; }
    float stddev()    const;
    float threshold() const;
    bool  warm()      const { return samples_seen_ >= warmup_ticks_; }

private:
    void handle_input(MessagePtr payload);

    std::string input_topic_  = "reality.proprio.scent_max";
    int         input_index_  = 0;
    std::string output_topic_ = "metrics.adaptive_threshold.scent_max";
    float       alpha_         = 0.001f;
    float       n_stddev_      = 2.0f;
    int         warmup_ticks_  = 1000;
    float       min_stddev_    = 1e-6f;

    float mean_ema_     = 0.0f;
    float var_ema_      = 0.0f;
    int   samples_seen_ = 0;
};

} // namespace ogma
