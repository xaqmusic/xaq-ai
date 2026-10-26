// =============================================================================
// GaitErrors.hpp  --  the gait's own prediction errors, per tick, as one vector (G4b's context input)
// =============================================================================
//
// The situations G4 must tell apart are the ones where the gait's errors differ (G3 on the terrain arena, ledger
// 2026-10-25: two-stride summaries of contact, load, belly, tilt and posture did not separate edge terrain from flat).
// This publishes the errors themselves so a FeatureWindow -> EPM can make a vocabulary of them:
//   [sc_fl, sc_fr, sc_rl, sc_rr, stall]
//   sc_i  = 1 when the gait's phase says leg i is in SWING and its foot sensor says it is touching (the phase predicted
//           the foot in the air), else 0 — the same error LiftDescent descends;
//   stall = StuckSense's stall score (commanding forward, stride speed short of the body's own recent best).
// Egocentric only (the gait's phase, the foot sensors, StuckSense).  An instrument: it changes nothing by itself.
// =============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class GaitErrors : public Module {
public:
    std::string_view       type_name()      const override { return "GaitErrors"; }
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

private:
    std::vector<std::string> phase_topics_{"rhythm.leg.fl", "rhythm.leg.fr", "rhythm.leg.rl", "rhythm.leg.rr"};
    std::string contact_topic_ = "reality.proprio.foot_contact";
    std::string stall_topic_   = "reality.proprio.stuck";
    std::string output_topic_  = "reality.proprio.gait_errors";
    std::array<float, 4> swing_{0, 0, 0, 0}, contact_{0, 0, 0, 0};
    float stall_ = 0.0f;
    bool  have_phase_ = false, have_contact_ = false;
    double sc_ema_[4] = {0, 0, 0, 0};
    int64_t ticks_ = 0;
};

} // namespace ogma
