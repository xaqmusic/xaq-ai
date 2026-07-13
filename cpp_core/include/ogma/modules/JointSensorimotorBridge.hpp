#pragma once

// =============================================================================
// JointSensorimotorBridge.hpp  --  Phase 7.2-EPM sensorimotor pairing
// =============================================================================
//
// Joins N per-joint ActionOut streams with the matching N indices of a single
// bundled ProprioToken (typically `reality.proprio.joints`) and republishes
// per-joint ProprioTokens on `reality.joint_<leg>.<servo>` (or whatever
// output_topics the config specifies).  Each output value is the 3-D vector
//
//      [ norm_position, last_action_scalar, position_delta_since_prev_tick ]
//
// This is the sensorimotor input shape Per-joint EPMs (Phase 7.2-EPM,
// docs/phase7_chunk_plan.md addendum) consume.  Joining at the substrate
// boundary — not in the env body — keeps env-vs-substrate clean and makes
// the same bridge work on a future PiCrawler HAL.
//
// Module lifecycle authoring contract: per docs/primitives/_module_lifecycle.md.

#include <cstdint>
#include <string>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class JointSensorimotorBridge : public Module {
public:
    JointSensorimotorBridge();
    ~JointSensorimotorBridge() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    int n_joints()         const { return int(action_topics_.size()); }
    int total_publishes()  const { return total_publishes_; }
    int total_proprio_in() const { return total_proprio_in_; }
    int total_action_in()  const { return total_action_in_; }

private:
    void handle_proprio(MessagePtr payload);
    void handle_action(int joint_idx, MessagePtr payload);

    // Configuration
    std::string              proprio_input_topic_ = "reality.proprio.joints";
    std::vector<std::string> action_topics_;
    std::vector<std::string> output_topics_;
    std::vector<int>         proprio_indices_;   // index into ProprioToken.values
    std::string              sensor_label_prefix_ = "joint";  // ProprioToken.sensor
    // Phase 7.2-EPM per-leg mode: when > 1, the bridge groups every
    // `group_size` consecutive action_topics / proprio_indices into ONE
    // output ProprioToken of length 3 * group_size (concatenated
    // [pos, action, delta] triples).  output_topics length must equal
    // action_topics.size() / group_size.  Default 1 preserves per-joint
    // output behaviour bit-identically.
    int                      group_size_         = 1;

    // Working state — sized to n_joints() at setup.
    std::vector<float> last_position_;       // latest per-joint position seen
    std::vector<float> prev_position_;       // last tick's position (for delta)
    std::vector<float> last_action_;         // latest per-joint action seen
    bool               have_proprio_      = false;

    int total_publishes_   = 0;
    int total_proprio_in_  = 0;
    int total_action_in_   = 0;
};

} // namespace ogma
