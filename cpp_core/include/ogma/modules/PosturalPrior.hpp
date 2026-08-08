#pragma once

// =============================================================================
// PosturalPrior.hpp  --  the always-on "brainstem" upright objective (L-1b)
// =============================================================================
//
// Promotes MotorEPM's additive postural reflex out into its own graph module,
// per plan §1.1/§2.10.  It captures the per-leg standing REST pose from the first
// proprio frame and publishes it as a SOFT posture target (PredictionToken) on
// `objective.posture.<leg>`.  MotorEPM's objective socket descends toward that
// target as an OBJECTIVE-CHANGE (not an additive output injection, §2.4).
//
// First-class + arbitrable: later a KeyframeGait / nav / manipulation loop
// publishes onto the SAME socket, and the EFE arbiter gates which one wins —
// this module is just the always-on default (the Gate 0 upright prior).
//
// Module lifecycle authoring contract: per docs/plans-and-designs/primitives/_module_lifecycle.md.

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class PosturalPrior : public Module {
public:
    PosturalPrior();
    ~PosturalPrior() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    int n_legs()        const { return int(proprio_topics_.size()); }
    int legs_captured() const;

private:
    void handle_proprio(int leg, MessagePtr payload);

    // Configuration
    std::vector<std::string> proprio_topics_;          // per-leg [pos,act,delta]×motor_dim ProprioToken topics
    std::vector<std::string> objective_output_topics_; // per-leg PredictionToken output topics
    int    motor_dim_        = 3;
    double postural_gain_    = 1.0;    // published as PredictionToken.confidence = clamp(w,0,1)
    double knee_tuck_target_ = -100.0; // override the last joint's rest with this tuck; < -90 disables

    // Per-leg captured rest pose (target joint positions) — persisted across restore.
    std::vector<Eigen::VectorXf> rest_pos_;   // motor_dim per leg
    std::vector<char>            captured_;    // per-leg: rest pose captured (char, not vector<bool>)
};

} // namespace ogma
