#pragma once

// =============================================================================
// SensorBundle.hpp  --  concatenate several ProprioToken topics into one vector
// =============================================================================
//
// A small, generic signal-prep module: subscribes an ordered list of
// ProprioToken topics and republishes their concatenated values as a single
// ProprioToken each tick.  Used to assemble a homeokinetic controller's input
// state from separately-published body sensors — e.g. the cell's fast-path
// state for MotorEPM = imu(4) + whisker_0..5(6) → a 10-D vector.
//
// Dimensionality stabilises once every input topic has been seen at least once;
// the bundle is only published from that tick on (so downstream MotorEPM, which
// requires a stable input dim, inits on a complete vector).  A momentarily
// stale input contributes its last value.

#include "ogma/Module.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class SensorBundle : public Module {
public:
    SensorBundle();
    ~SensorBundle() override;

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

    int last_dim() const { return last_dim_; }

private:
    void handle_input(std::size_t idx, MessagePtr payload);

    std::vector<std::string>        input_topics_;
    std::string                     output_topic_ = "fast.proprio.bundle";
    std::string                     sensor_label_ = "bundle";

    std::vector<std::vector<float>> last_values_;   // one buffer per input
    std::vector<bool>               seen_;
    int                             last_dim_ = 0;
};

} // namespace ogma
