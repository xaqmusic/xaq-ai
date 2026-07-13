#pragma once

// =============================================================================
// HeadingController.hpp  --  LEARNED heading-following controller (2026-06-21)
// =============================================================================
//
// The cognitive layer's locomotor primitive, and the operator's decomposition of
// the monolithic coxswain: instead of learning a tangled (bearing-state) ->
// (discrete turn x thrust that climbs a scent value) map — which is coarse, and
// learned a perverse thrust policy (back away when facing food, because the cy
// value rewards FACING not REACHING) — split into two clean problems:
//
//   1. HEADING SETPOINT (elsewhere): "what heading improves my situation?"  In
//      the open arena this is read straight off the ScentCompass up-gradient
//      bearing (a real sensor, an honest scaffold prior); in the maze it becomes
//      a planning problem (a heading that improves predicted proximity through
//      the topology).
//   2. HEADING-FOLLOWING (this module): given the desired heading as an
//      egocentric bearing [cx=+right, cy=+forward], LEARN the motor commands to
//      turn toward it and ADVANCE along it.
//
// Why this fixes the backing-away: thrust is coupled to ALIGNMENT (advance ~
// cos(bearing)), i.e. to PROGRESS toward where we want to go — not to a value
// that facing alone satisfies.  Turn and thrust get distinct jobs (no mirror
// collapse).
//
// Why it is still AI (bar-b), not a re-hardwired reflex (the ChemotaxisAI
// fixed-gain steer we flagged as cybernetics): the turn GAIN is LEARNED from the
// body's own response — the controller adapts g so that `steer = g * bearing`
// actually NULLS the bearing without oscillating (a 1-parameter forward model of
// the body's turn dynamics).  No hand-set P-gain (the no_tuning principle): g is
// set by the body's dynamics and self-calibrates, so it transfers across bodies/
// friction/fields.
//
// Publishes ActionOut on cog.steer (turn) + cog.thrust (advance) → MotorBus,
// exactly like the ActionDecoder it replaces, so it slots into the same pipeline.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class HeadingController : public Module {
public:
    HeadingController();
    ~HeadingController() override;

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

    // White-box accessors (tests + get_module_metrics).
    float last_bearing() const { return last_bearing_; }   // egocentric, [-1,1] (0=facing)
    float learned_gain() const { return last_gain_; }      // effective turn gain this tick
    float body_turn_gain() const { return k_body_; }       // learned k_body = |ω|/|steer|
    float last_steer()   const { return last_steer_; }
    float last_thrust()  const { return last_thrust_; }
    bool  last_nav_on()  const { return last_nav_on_; }
    // Learned-advance telemetry (only meaningful when learn_advance_).
    bool  learn_advance()    const { return learn_advance_; }
    int   last_err_bin()     const { return last_err_bin_; }
    int   last_thrust_act()  const { return last_thrust_act_; }
    float last_adv_reward()  const { return last_adv_reward_; }
    float adv_value_spread() const { return last_adv_spread_; }  // max-min of current err-bin row
    float adv_coverage()     const { return last_adv_cov_; }     // frac of (bin,act) cells visited

private:
    void handle_compass(MessagePtr payload);
    void handle_ang_vel(MessagePtr payload);
    void handle_vel_ego(MessagePtr payload);
    void handle_authority(MessagePtr payload);
    void handle_speed_gate(MessagePtr payload);

    // --- desired heading source: egocentric up-gradient bearing -------------
    std::string input_topic_ = "percept.scent_compass";   // [cx=+right, cy=+forward]
    int         cx_index_     = 0;
    int         cy_index_     = 1;
    float       compass_x_    = 0.0f;
    float       compass_y_    = 0.0f;

    // --- learned turn gain = FORWARD MODEL of the body's rotation ------------
    // The body publishes its yaw rate ω (ang_vel_topic) — a clean, purely
    // steer-driven signal (translation does NOT move it, unlike the bearing).
    // Learn k_body = EMA(|ω| / |steer|) = "turn rate per unit steer", then command
    // steer = clamp(turn_fraction_ * bearing / k_body) → turn a fixed fraction of
    // the heading error per tick, calibrated to THIS body's response.  k_body is
    // learned from the body's own dynamics (no hand-set P-gain) and CONVERGES to
    // a body-specific value (fast turner → low steer; sluggish → high) instead of
    // railing — and it transfers across bodies/friction.
    std::string ang_vel_topic_ = "reality.proprio.ang_vel";
    float       latest_ang_vel_ = 0.0f;
    float       k_body_         = 0.0f;   // learned EMA(|ω|/|steer|); 0 = not yet learned
    float       gain_lr_        = 0.05f;  // EMA rate for k_body
    float       turn_fraction_  = 0.6f;   // fraction of heading error to null per tick (stability)
    float       gain_init_      = 0.5f;   // fallback gain before k_body is learned
    float       gain_min_       = 0.05f;
    float       gain_max_       = 4.0f;
    float       fixed_gain_     = 0.0f;   // >0 = fixed turn gain, bypass the contaminated online k_body
    float       turn_commit_    = 0.0f;   // >0 = saturating committed turn (max_steer*tanh(commit*bearing))

    // --- output scaling (action range, NOT behavioral tuning) ---------------
    float max_steer_  = 4.0f;    // cog.steer range (matches ActionDecoder accel range)
    float max_thrust_ = 4.0f;    // cog.thrust range
    float min_signal_ = 0.1f;    // |gradient| below = no confident heading → no advance
    // Braking alignment gate: advance at full thrust only when facing within
    // align_angle_deg_; BRAKE (reverse) when more off-axis → the bug kills its
    // coasting forward momentum and turns IN PLACE to face the heading, then
    // charges — instead of arcing/orbiting (forward+turn → wide circle that never
    // closes; operator obs).  align_cos_ = cos(align_angle_deg_).
    float align_angle_deg_ = 30.0f;
    float align_cos_       = 0.8660254f;   // cos(30°)

    // FORWARD-ONLY locomotion (2026-06-22, operator UI obs): a front-mouth forager
    // should never REVERSE to navigate — reversing off-axis is what backs it into walls
    // (the new food respawns behind → off-axis → the cos-gate/learned-policy reverses →
    // wall wedge → degenerate).  When true: off-axis → STOP (the steer rotates in place),
    // facing → forward; reverse is removed from the normal advance entirely (the escape
    // is a forward-probe, so reverse is unused).  Default false = legacy (reverse allowed).
    bool   forward_only_ = false;

    // Reverse stays in the vocabulary, but acts only as a BRAKE: a reverse command applies
    // while the body still has forward momentum (decelerate to a stop), then clamps to 0
    // (turn in place) — so it never drives BACKWARD for several seconds (the long post-eat
    // reverse that backs into walls; operator 2026-06-22).  The cos-gate's reverse was
    // always MEANT to brake; this makes the implementation match the intent.  Needs
    // vel_topic.  Default false.  Preferred over forward_only (keeps reverse in the vocab).
    bool   reverse_brake_only_ = false;

    // --- LEARNED advance policy (opt-in; replaces the hand-designed cos gate) ----
    // "action learns how to act on a heading" (operator's clean decomposition).
    // The turn (k_body) is already learned; this learns the THRUST policy too, so
    // brake-turn-then-charge EMERGES instead of being hardwired.  Default off →
    // the cos gate above runs and every existing config stays byte-identical.
    //
    // STATE  = heading-error bin (|bearing| binned into n_err_bins_).
    // ACTION = a thrust level (n_thrust_acts_ levels spanning [-max_thrust,+max_thrust]).
    // REWARD = max(0, vel_fwd) * cos(bearing*pi)  — FORWARD progress projected onto the
    //   commanded heading.  Food-INDEPENDENT (own velocity + own command only) → learns
    //   from ANY heading, never deadlocks, testable in isolation.  The max(0,·) encodes
    //   the front-mouth morphology (the bug eats facing-forward), so a bidirectional
    //   body can't satisfy it by reversing toward a behind-target.
    // SELECT = UCB over the err-bin row (self-annealing exploration — the adaptive
    //   mechanism, not a tuned epsilon/temperature).
    bool        learn_advance_   = false;
    std::string vel_topic_       = "reality.proprio.vel_ego";  // [v_right, v_forward], move_speed-norm
    int         n_err_bins_      = 4;     // |bearing| bins (ConstructionOnly)
    int         n_thrust_acts_   = 3;     // thrust levels: reverse / stop / forward (ConstructionOnly)
    float       advance_lr_      = 0.1f;  // EMA rate for the advance value table
    float       ucb_c_           = 0.4f;  // UCB exploration weight (~ value-range scale)
    // 2026-06-22 — HOMEOKINETIC advance reward.  Legacy reward = max(0,vel_fwd)·cos(bearing)
    // is FOOD-COUPLED: forward while off-axis (cos≤0) scores NEGATIVE, so a bad heading
    // (nav's domain) decays the advance table to 0 and it gets trapped (can only climb via
    // forward-while-facing-food, unreachable when disoriented).  When true: reward =
    // max(0,vel_fwd) − effort·|vel_fwd| — an INTRINSIC controllable-forward-motion drive,
    // decoupled from food, never punished by the heading → no collapse.  The nav/steer
    // aims; turn-priority mixing handles cornering, so the cos-brake is redundant.
    bool        advance_homeokinetic_ = false;
    // Authority-gated learning (subsumption): when a reflex overrides this controller on
    // the MotorBus, the cog's authority share drops → scale the advance learning rate by
    // it so reflex-driven motion is NOT miscredited to the learned policy.  Empty topic =
    // authority 1 = full learning (byte-identical).  Replaces the in-controller escape's
    // manual learning-suppression with the designed (MotorBus authority) mechanism.
    std::string authority_topic_ = "";
    float       latest_authority_ = 1.0f;
    // ORTHOKINESIS speed gate — scent MAGNITUDE (e.g. klino cap) slows the advance near food.
    std::string speed_gate_topic_ = "";
    float       speed_gate_       = 0.0f;    // latest gate ∈[0,1] (1 = eating range → slow)
    bool        have_speed_gate_  = false;
    float       speed_gate_floor_ = 0.15f;   // min crawl-speed fraction at gate=1 (the single constant)
    float       last_speed_scale_ = 1.0f;    // telemetry: applied thrust scale
    // Energy prior (Playful-Machine "don't waste motor action"): a small cost on
    // ANY motion |vel_fwd|.  Off-axis (cos<0) it breaks the stop≡reverse tie toward
    // STOP (let the steer rotate to face) instead of reversing (backing away); when
    // facing it's dominated by the forward-progress reward.  NOT a behavioral knob —
    // a metabolic prior that makes the brake = stop.
    float       effort_cost_     = 0.15f;
    float       vel_right_       = 0.0f;
    float       vel_fwd_         = 0.0f;
    std::vector<float> adv_value_;        // [n_err_bins_ * n_thrust_acts_] EMA(reward), 0-init
    std::vector<int>   adv_visits_;       // [n_err_bins_ * n_thrust_acts_] visit counts
    std::vector<int>   adv_bin_visits_;   // [n_err_bins_] per-bin total visits (UCB N)
    // credit deferral: thrust chosen at t produces velocity observed at t+1
    bool   have_prev_adv_ = false;
    int    prev_err_bin_  = -1;
    int    prev_thrust_act_ = -1;
    float  prev_cmd_cosb_ = 0.0f;         // cos(bearing*pi) at selection (commanded fwd alignment)
    // telemetry
    int    last_err_bin_    = -1;
    int    last_thrust_act_ = -1;
    float  last_adv_reward_ = 0.0f;
    float  last_adv_spread_ = 0.0f;
    float  last_adv_cov_    = 0.0f;

    std::string steer_topic_  = "cog.steer";
    std::string thrust_topic_ = "cog.thrust";

    // --- learning/telemetry state -------------------------------------------
    float prev_bearing_ = 0.0f;
    float prev_steer_   = 0.0f;
    bool  have_prev_    = false;
    float last_bearing_ = 0.0f;
    float last_steer_   = 0.0f;
    float last_thrust_  = 0.0f;
    float last_gain_    = 0.0f;
    bool  last_nav_on_  = false;
    int   tick_count_   = 0;
};

} // namespace ogma
