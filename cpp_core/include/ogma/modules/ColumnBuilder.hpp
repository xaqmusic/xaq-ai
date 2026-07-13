#pragma once

// =============================================================================
// ColumnBuilder.hpp  --  passive place-recorder (column = view-feature + pose)
// =============================================================================
//
// Replaces the saccade+cylinder mapping scheme (C1+C2) with a PURE PASSIVE
// OBSERVER. No scan, no pivot. Every `record_every` ticks it assembles a
// "column" — a compact appearance signature of the current FPV plus the
// agent's heading and IMU — and publishes it as a ProprioToken. A downstream
// place-EPM clusters these columns into stable place+heading nodes.
//
//   host.video.color + heading_vec + vel_ego + ang_vel
//        → ColumnBuilder → percept.column   (intermittent, every record_every ticks)
//
// Column vector layout (3*n_strips + 4 floats), in this exact order:
//   [0 .. 3*n_strips-1]  n_strips vertical strips × mean RGB, normalized [0,1]
//                        (split FPV WIDTH into n_strips equal columns left→right;
//                         per strip = mean R, mean G, mean B over all its pixels;
//                         ordered strip0_R, strip0_G, strip0_B, strip1_R, ...)
//   [3*n_strips + 0]     sin(heading)  (copied straight from heading_topic[0])
//   [3*n_strips + 1]     cos(heading)  (copied straight from heading_topic[1])
//   [3*n_strips + 2]     forward velocity normalized: clamp(vel_fwd/4.0, -1, 1)
//   [3*n_strips + 3]     yaw rate normalized:         clamp(ang_vel/2.0, -1, 1)
//
// With the default n_strips=6 the output dim is 22.
//
// The output is INTERMITTENT by design: it publishes only on ticks where
// (tick % record_every) == 0, so the place-EPM sees a sparse stream of
// place snapshots rather than a per-tick firehose.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class ColumnBuilder : public Module {
public:
    ColumnBuilder();
    ~ColumnBuilder() override;

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

    // White-box accessors (tests + metrics).
    std::vector<float> const& last_column() const { return column_; }   // [dims()], normalized
    int  dims()               const { return 3 * n_strips_ + 4; }
    int  n_strips()           const { return n_strips_; }
    bool recorded_last_tick() const { return recorded_last_tick_; }

private:
    void handle_frame(MessagePtr payload);
    void handle_heading(MessagePtr payload);
    void handle_vel(MessagePtr payload);
    void handle_ang(MessagePtr payload);

    std::string vision_topic_  = "host.video.color";
    std::string heading_topic_ = "reality.proprio.heading_vec";
    std::string vel_topic_     = "reality.proprio.vel_ego";
    std::string ang_topic_     = "reality.proprio.ang_vel";
    std::string output_topic_  = "percept.column";

    int n_strips_     = 6;     // vertical strips across the FPV width
    int record_every_ = 15;    // emit cadence (ticks)

    // Latest inputs.
    std::vector<uint8_t> pixels_;
    int   width_ = 0, height_ = 0, channels_ = 0;
    float heading_sin_ = 0.0f;
    float heading_cos_ = 1.0f;
    float vel_fwd_     = 0.0f;
    float ang_vel_     = 0.0f;

    // Last assembled + published column (held for HUD/tests). [dims()], 0..1 RGB
    // + bounded pose tail.
    std::vector<float> column_;
    bool recorded_last_tick_ = false;
};

} // namespace ogma
