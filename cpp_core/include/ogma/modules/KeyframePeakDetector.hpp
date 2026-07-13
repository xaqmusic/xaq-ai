#pragma once

// =============================================================================
// KeyframePeakDetector.hpp  --  Phase v5.2 sensor / motor downsampler
// =============================================================================
//
// Subscribes to a single source topic, buffers incoming payload values in a
// rolling window of N ticks, and publishes the per-tick rolling mean as a
// ProprioToken on the configured output topic.
//
// Combined with EPM's `process_every_n_ticks` (Phase 4a), this lets a slow
// EPM consume averaged keyframes of fast sensor / motor streams.  Use case:
//
//   keyframe_motor_left  (window=50, source=action.left)  →  reality.proprio.motor_peak_left
//   keyframe_motor_right (window=50, source=action.right) →  reality.proprio.motor_peak_right
//   epm_motor_slow_left  (process_every_n_ticks=50, input=reality.proprio.motor_peak_left)
//
// Result: epm_motor_slow_left tops a 60Hz motor stream into ~833ms keyframes
// of "average left wheel command over the last second" — a self-model of
// the body's recent motor pattern, available to the consensus latent.
// Closes the predictive-coding loop the user articulated:  "am I on the
// correct long-horizon path at this moment given my short and long sensor
// inputs AND my recent motor pattern?"
//
// Module lifecycle authoring contract: per docs/primitives/_module_lifecycle.md.
// - bus_ / sub_ids_ inherited from Module base (no shadowing).
// - Every input handler starts with input_allowed(producer_id) gate.
// - Every published ProprioToken stamps producer_id = id_.
// - on_setup pushes every bus_->subscribe into sub_ids_ for clean teardown.

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class KeyframePeakDetector : public Module {
public:
    KeyframePeakDetector();
    ~KeyframePeakDetector() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // Read-only accessors (used by tests + diag).
    int   window_fill()         const { return int(buffer_.size()); }
    int   window_size()         const { return window_size_; }
    int   payload_dim()         const { return payload_dim_; }
    std::vector<float> const& last_mean() const { return last_mean_; }
    int   total_inputs_seen()   const { return total_inputs_seen_; }
    int   total_publishes()     const { return total_publishes_; }

private:
    void handle_input(MessagePtr payload);

    // Configuration
    std::string input_topic_     = "action.out";
    std::string output_topic_    = "reality.proprio.motor_peak";
    std::string payload_kind_    = "action_out";  // or "proprio_token"
    int         window_size_     = 50;
    std::string sensor_label_    = "";  // optional ProprioToken.sensor field

    // Working state
    std::deque<std::vector<float>> buffer_;       // each entry = one input frame's values
    int                            payload_dim_  = 0;
    std::vector<float>             last_mean_;
    int                            total_inputs_seen_ = 0;
    int                            total_publishes_   = 0;
};

} // namespace ogma
