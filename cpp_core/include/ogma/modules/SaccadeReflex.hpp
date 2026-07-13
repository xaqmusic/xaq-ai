#pragma once

// =============================================================================
// SaccadeReflex.hpp  --  saccadic learning-walk (Pathway C1, active sensing)
// =============================================================================
//
// The insect "learning walk": on arriving at a new place, pivot in place to scan
// the surround, then resume locomotion. The pivot is an EPISTEMIC action — movement
// in service of perception (reduce uncertainty about "where am I") — and it is what
// lets the cylinder builder (C2) accumulate a heading-indexed panorama = a stable
// place signature, instead of the view-fragmented per-tick frame.
//
//   reality.proprio.vel_ego → SaccadeReflex → saccade.left / saccade.right (MotorBus)
//                                            → saccade.active (1 while pivoting → C2 gate)
//
// TRIGGER (dead-simple, egomotion-honest): accumulate distance travelled (|vel_ego|
// per tick); when it exceeds travel_trigger the bug has reached a "new area" → fire a
// saccade. No engineered schedule, no oracle — just "scan whenever you've moved on."
//
// PIVOT: a pure differential rotation (left=+spin, right=−spin) for pivot_ticks
// (≈ one clock period, so clock phase ≈ sweep heading for C2). The lr differential
// gives common-mode 0 → zero forward → spins in place (same mechanism as
// StuckEscapeReflex's rotation pulse). High MotorBus gain → overrides the forager
// during the sweep. Then a refractory before the next saccade.
//
// Default-off / opt-in (enable). Self-contained motor behaviour; C1 gate = a smooth
// monotonic heading sweep on trigger, then locomotion resumes.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class SaccadeReflex : public Module {
public:
    SaccadeReflex();
    ~SaccadeReflex() override;

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

    // White-box accessors (tests + metrics). state: 0=idle 1=pivot 2=refractory.
    int   state()          const { return state_; }
    bool  is_pivoting()    const { return state_ == 1; }
    float last_spin()      const { return last_spin_; }   // signed left-paddle accel during pivot
    float dist_accum()     const { return dist_accum_; }
    int   saccade_count()  const { return saccade_count_; }
    float last_novelty()   const { return novelty_tle_; }
    float last_scent()     const { return scent_max_; }
    float last_boredom()   const { return boredom_; }
    float last_progress()  const { return scent_short_ - scent_long_; }  // >0 approaching, ≤0 stalled
    float last_hunger()    const { return hunger_; }

private:
    void handle_vel(MessagePtr payload);
    void handle_scent(MessagePtr payload);
    void handle_novelty(MessagePtr payload);

    std::string vel_topic_          = "reality.proprio.vel_ego";
    std::string output_topic_left_  = "saccade.left";
    std::string output_topic_right_ = "saccade.right";
    std::string active_topic_       = "saccade.active";  // scalar 1 while pivoting (→ C2)

    bool   enable_           = true;
    // trigger_mode 0 = "distance" (the BOOTSTRAP: fire after travelling travel_trigger).
    // trigger_mode 1 = "epistemic" (Pathway B de-scaffold): fire only while EXPLORING
    // (scent below the homing gate) AND in a PERCEPTUALLY NOVEL place (vision-EPM TLE
    // above threshold). That makes the saccade a genuine epistemic action — scan
    // because "I'm somewhere unfamiliar and not homing" — instead of a fixed timer, and
    // stops it competing with foraging.
    int    trigger_mode_     = 0;
    // EPISTEMIC explore drive = NO FORAGING PROGRESS × HUNGER (the EFE explore/exploit
    // balance): scan when scent isn't rising (not getting closer to food) AND the bug is
    // hungry AND the place is perceptually novel. Scent-progress = short-EMA − long-EMA of
    // scent_max (>0 = approaching = exploit; ≤0 = stalled = explore). (boredom_topic kept
    // as an OPTIONAL extra gate — DistressDrive measures PHYSICAL stuck, not foraging
    // stall, so it's off by default here.)
    std::string scent_topic_   = "";    // reality.proprio.scent_max → the foraging-progress EMA
    std::string hunger_topic_  = "";    // reality.proprio.hunger (1−energy) → explore-when-hungry
    std::string novelty_topic_ = "";    // perceptual novelty (vision-EPM RealityToken, reads .tle)
    std::string boredom_topic_ = "";    // OPTIONAL physical-stuck gate (DistressDrive ReflexGate)
    float  short_alpha_      = 0.05f;   // fast EMA on scent_max
    float  long_alpha_       = 0.005f;  // slow EMA on scent_max
    float  progress_gate_    = 0.001f;  // explore when (short−long) < this (scent not rising)
    float  hunger_gate_      = 0.3f;    // explore only when hunger > this
    float  novelty_threshold_= 0.3f;    // fire only when vision TLE > this (unfamiliar)
    float  boredom_gate_     = 0.3f;    // (optional) physical-stuck threshold
    float  scent_gate_       = 1e9f;    // (legacy/unused) absolute homing cap
    float  travel_trigger_   = 3.0f;    // accumulated |vel| before a saccade fires (distance mode)
    int    pivot_ticks_      = 120;     // sweep duration (≈ one clock period)
    float  spin_rate_        = 4.0f;    // differential-rotation magnitude during the pivot
    int    refractory_ticks_ = 120;     // min gap after a saccade

    // latest inputs
    float  speed_       = 0.0f;         // |vel_ego| this tick
    float  scent_max_   = 0.0f;         // latest scent proximity
    float  scent_short_ = 0.0f;         // fast EMA of scent_max
    float  scent_long_  = 0.0f;         // slow EMA of scent_max  (short−long = foraging progress)
    bool   have_scent_ema_ = false;
    float  hunger_      = 0.0f;         // 1−energy
    float  boredom_     = 0.0f;         // optional physical-stuck signal
    float  novelty_tle_ = 0.0f;         // perceptual surprise (vision EPM TLE)

    // FSM state
    int    state_      = 0;             // 0=idle 1=pivot 2=refractory
    int    ticks_left_ = 0;             // remaining ticks in pivot/refractory
    float  dist_accum_ = 0.0f;          // distance since last saccade (idle accumulation)
    int    saccade_count_ = 0;

    // telemetry
    float  last_spin_ = 0.0f;
};

} // namespace ogma
