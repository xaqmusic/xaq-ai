// =============================================================================
// LoadPhaseGait.hpp  --  a per-leg phase that the leg's own load slows down
// =============================================================================
//
// The gait mechanism audit (docs/reports/picrawler_gait_mechanism_audit_2026-10-12.md)
// found that every rhythmic term in the deployed motor layer rides a phase read off the
// knee it drives (atan2 of position and one-tick velocity: retrograde two ticks in three,
// a relay loop at the slew cap), that the stance/swing detector reads body attitude rather
// than the feet, and that nothing references ground contact although the robot has foot
// force sensors.  This module is the timing substrate proposed in its place.
//
// THE RULE (Owaki, Kano, Nagasawa, Tero, Ishiguro, J. R. Soc. Interface 2013; Owaki &
// Ishiguro, Sci. Rep. 2017 -- "Tegotae"):
//
//     dphi_i/dt = omega  -  sigma * N_i * cos(phi_i)
//
// phi_i is leg i's phase.  Swing is phi in (0, pi) (sin > 0), stance is (pi, 2pi).  N_i is
// the SHARE of the body's load that foot i carries (0..1, from the foot force sensor, or
// from the contact flags when no force is published).  The load term has a stable point at
// phi = 3pi/2 (mid-stance): a loaded leg is held in its push and does not lift; an unloaded
// leg runs free at omega.  No inter-leg phase is prescribed.  When one leg lifts its share
// moves to the others, which slows THEM -- the legs coordinate through the ground.  On
// Owaki's quadruped this produced walk, trot and gallop as omega rose, with no gait named.
//
// In this project's terms (CLAUDE.md s1, the rewrite rule): the error each leg descends is
// "I intend to be pushing and I feel no ground" (or the reverse); the sensory channel is the
// foot sensor, which exists on the robot; the gate is the load; the owning module is this
// one, which publishes only a phase.  What the joints DO with the phase is MotorEPMv2's
// business (its leg_phase_* sockets), so the learned layer, the posture spring and the
// reflexes stay underneath, untouched.
//
// Published per leg: ProprioToken [cos phi, sin phi, swing (1/0), N_i].
// Module absent = byte-identical.  sigma_ratio = 0 is the free-running clock: the
// open-loop sequencer the record rejected, kept as this lever's structure control.
#pragma once
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class LoadPhaseGait : public Module {
public:
    LoadPhaseGait();
    ~LoadPhaseGait() override;
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

    float phase(int leg) const { return leg >= 0 && leg < int(phase_.size()) ? phase_[size_t(leg)] : 0.0f; }
    float share(int leg) const { return leg >= 0 && leg < int(share_.size()) ? share_[size_t(leg)] : 0.0f; }
    bool  swing(int leg) const { return leg >= 0 && leg < int(phase_.size()) && std::sin(phase_[size_t(leg)]) > 0.0f; }

private:
    void handle_contact(MessagePtr p);
    void handle_load(MessagePtr p);

    std::string contact_topic_ = "reality.proprio.foot_contact";
    std::string load_topic_    = "";          // empty = shares from the contact flags
    std::vector<std::string> output_topics_ = {"rhythm.leg.fl", "rhythm.leg.fr", "rhythm.leg.rl", "rhythm.leg.rr"};
    int    n_legs_       = 4;
    double period_ticks_ = 70.0;   // omega = 2pi / period (the body's measured stride, ledger 2026-07-20)
    double sigma_ratio_  = 2.0;    // sigma / omega.  0 = free clock (the structure control)
    double load_floor_   = 0.02;   // a foot force below this fraction of the body's load is "no load"
    int64_t seed_        = 1234;   // the legs start at random phases, as on Owaki's robot
    std::vector<double> init_phase_;   // optional explicit start phases (radians); empty = seeded random

    std::vector<float> phase_, share_, omega_eff_, contact_, load_;
    std::vector<uint8_t> prev_contact_;
    bool have_contact_ = false, have_load_ = false;
    // Instruments (report-only).
    std::vector<float> td_c_, td_s_;        // EMA of e^{i phi} at touchdown, per leg -> PLV
    std::vector<float> lo_c_, lo_s_;        // the same at liftoff
    std::vector<int32_t> td_n_, lo_n_;      // events seen per leg (the EMAs seed on the first)
    std::vector<float> pair_c_, pair_s_;    // EMA of e^{i(phi_i - phi_j)} per pair -> inter-leg PLV
    float retro_ema_ = 0.0f, agree_ema_ = 0.5f, swing_ema_ = 0.5f;
    int64_t ticks_ = 0, touchdowns_ = 0;
    static constexpr float kTwoPi = 6.28318530717958647692f;
    static constexpr float kPlvAlpha  = 0.05f;   // per event (touchdowns)
    static constexpr float kPairAlpha = 0.01f;   // per tick (~100 ticks, a stride and a half)
    static constexpr float kEmaAlpha  = 0.01f;
};

} // namespace ogma
