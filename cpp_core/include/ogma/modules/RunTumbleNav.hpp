#pragma once

// =============================================================================
// RunTumbleNav.hpp  --  honest temporal chemotaxis (E. coli methylation reflex)
// =============================================================================
//
// The de-scaffolded nav (doctrine §4 morphology honesty): the 8-nostril ring is a
// HOMING sensor — it spatially encodes the gradient direction, which no real
// chemotaxer has. This module navigates from a SCALAR scent concentration only.
// With no instantaneous direction, the ONLY way to climb is TEMPORAL action-
// consequence: run (move in a committed direction), sense whether scent is RISING
// or FALLING, and modulate the chance of a random reorientation (a TUMBLE)
// accordingly. Direction is never sensed — it EMERGES from acting.
//
// THE MECHANISM IS THE REAL BACTERIAL ONE — no clock, no learned policy. E. coli
// adapts its tumble rate via receptor METHYLATION: a leaky chemical memory that
// tracks the recently-experienced ligand concentration. We mirror it directly:
//
//   baseline_  = EMA(scent)                       (the methylation level = a PREDICTION
//                                                   of the scent the bug expects to see)
//   error      = scent − baseline_                (the prediction ERROR; rising > 0)
//   p_tumble   = clamp(base − gain·error_n)       (LOW while rising, HIGH while falling)
//
// Each tick the tumble PROBABILITY is low while scent rises above the methylation
// baseline (positive prediction error → "things are getting better, keep running")
// and rises as scent falls below it (the bug has run past the gradient → tumble).
// Run length is not a parameter — it EMERGES from the gradient: a steady climb gives
// a long string of low-p ticks (a long run); a stall raises p every tick (a tumble
// soon). There is NO cycle clock (the old `cycle_ticks` was a scaffold that injected
// a reaction lag), and NOTHING is learned (the old V[trend][action] table is gone).
//
// Honest by the doctrine:
//   §1 predictive — baseline_ is a forward prediction of scent, (scent−baseline_) is
//      its error, and the tumble rate is the action graded by that error each tick;
//   §4 morphology — this is a structural sensorimotor REFLEX (no reward, no learning),
//      the bacterial run-and-tumble made literal;
//   blanket-clean — SCALAR scent only; direction still emerges from acting, never sensed.
//
//   reality.proprio.scent_max (SCALAR) + reality.proprio.heading (egomotion)
//        → RunTumbleNav → percept.runtumble_heading  ([vx,vy] → HeadingController)
//
// The run direction is committed in the bug's OWN egomotion-heading frame (honest —
// path-integrated, not an external compass); a tumble = a new random direction off the
// CURRENT heading (so it cooperates with whatever the reflexes already turned). The
// methylation baseline is NEVER reset (continuous adaptation = the chemical memory).
// Default-off / opt-in. Gate: beats a gradient-blind random walk (shuffle control).
//
// SELF-CALIBRATED CONFIDENCE from its OWN eats (the nested blanket §2.1). klino reports a
// capability ∈[0,1] the L2 arbiter reads = current-smell / typical-EAT-smell. The denominator
// is EAT-CALIBRATED: klino subscribes to a GROUND-TRUTH consummatory event (`eat_topic`, e.g.
// events.eat — NOT events.hit, which in the Cell is overloaded with a scent-progress reward that
// fires all through the approach and would peg the scale far too low) and learns eat_scent_ =
// EMA(scent when it eats) — the scent scale at which it ACTUALLY scores food. So capability reads
// ~1 the instant the bug is in its own eating range (fixing the arbiter's "klino level structurally
// capped below the planner at the eat"). This is the ONLY use of the eat: the run/tumble POLICY
// stays reward-free — baseline_, p_tumble and the motor output never read the eat or eat_scent_.
// Before the first eat, capability bootstraps off a slow-decaying running scent peak. This is honest
// metacognition (klino models the scale of its own success), not reward shaping.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class RunTumbleNav : public Module {
public:
    RunTumbleNav() = default;
    ~RunTumbleNav() override = default;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void                   on_param_change(std::string_view key, ParamValue const& value) override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;

    nlohmann::json diag_snapshot() const override;

    // accessors (tests / telemetry)
    int   last_action()    const { return committed_action_; }  // 0 run / 1 tumble
    float last_p_tumble()  const { return last_p_tumble_; }      // per-tick tumble probability
    float last_error()     const { return last_error_; }         // normalised prediction error (scent − baseline)
    float baseline()       const { return baseline_; }           // methylation level (EMA of scent)
    int   tumble_count()   const { return tumble_count_; }
    int   run_count()      const { return run_count_; }
    int   forced_tumbles() const { return forced_tumbles_; }
    float capability()     const { return capability_; }         // self-reported pragmatic gain ∈[0,1]: ~0 blind, ~1 in its own eating range
    float scent_peak()     const { return scent_peak_; }         // slow-decaying memory of the typical food-scent magnitude (pre-eat bootstrap denom)
    float eat_scent()      const { return eat_scent_; }          // EMA of the scent at which it actually EATS (the capability denominator once it has eaten)
    bool  have_eat_scent() const { return have_eat_scent_; }
    // KF0 run-length-asymmetry telemetry (a KINESIS is healthy when up-runs ≫ down-runs, even
    // when the instantaneous run direction looks random — the health metric the widget lacked).
    float run_len_up()     const { return run_len_up_; }         // EMA run length (ticks) for runs that RAISED scent
    float run_len_down()   const { return run_len_down_; }       // EMA run length (ticks) for runs that LOWERED scent
    float turn_frac()      const { return turn_frac_; }          // EMA fraction of ticks spent reorienting (not executing a run)
    int   forced_in_turn() const { return forced_in_turn_; }     // forced tumbles that fired mid-turn (the K2 livelock signature)
    // KF1 run integrity + KF6 directional belief (opt-in) telemetry.
    bool  reorienting()    const { return reorienting_; }        // true after a tumble until the body faces run_dir
    float dir_consistency() const { return last_R_; }            // ∈[0,1] mean-resultant-length of run outcomes = belief precision (κ proxy)
    float dir_mu()         const { return last_mu_; }            // believed up-gradient absolute heading (rad)
    bool  muted()          const { return have_authority_ && authority_ < authority_floor_; }  // another loop/reflex has the bus
    float run_dir()        const { return run_dir_abs_; }        // committed absolute run direction (rad)

private:
    void handle_scent(MessagePtr payload);
    void handle_heading(MessagePtr payload);
    void handle_vel(MessagePtr payload);
    void handle_eat(MessagePtr payload);
    void handle_authority(MessagePtr payload);

    std::string scent_topic_   = "reality.proprio.scent_max";  // SCALAR concentration (no ring)
    std::string heading_topic_ = "reality.proprio.heading";    // egomotion heading (run-direction frame)
    std::string vel_topic_     = "reality.proprio.vel_ego";    // egomotion velocity (forward = stuck check)
    std::string eat_topic_     = "events.eat";                 // GROUND-TRUTH eat event → calibrates eat_scent_ (confidence ONLY; policy reward-free)
    std::string output_topic_  = "percept.runtumble_heading";  // [vx,vy] → HeadingController
    std::string confidence_topic_ = "percept.klino_confidence"; // SCALAR capability ∈[0,1] → EFEArbiter (self-reported pragmatic gain)

    // --- methylation mechanism params (the E. coli reflex) ---
    float baseline_alpha_ = 0.05f;   // methylation adaptation rate (EMA of scent = the prediction baseline)
    float scale_alpha_    = 0.02f;   // EMA rate of the running |error| normaliser
    float tumble_base_    = 0.1f;    // per-tick tumble probability at flat gradient (mean run ~1/this)
    float tumble_gain_    = 0.1f;    // how strongly the normalised CHANGE (gradient) modulates tumble probability
    // ORTHOKINESIS ON THE TUMBLE RATE (coarse→fine homing). The change-based tumble_gain gives long
    // runs while scent RISES (the approach) but nothing special AT the source (high, flat scent →
    // p_tumble = tumble_base → 10-tick runs wandering the peak → barrels past / eats-once). tumble_
    // level_gain raises the tumble rate with the scent LEVEL (proximity), so runs SHORTEN near food:
    // p_tumble = clamp(tumble_base·(1 + tumble_level_gain·cap) − tumble_gain·error_n, min, max), where
    // cap = the self-calibrated proximity ∈[0,1] (scent / eat_scent), →1 in the eating range. Far
    // (cap≈0) → unchanged (long approach runs preserved); at the source (cap→1) → short runs = fine
    // sampling to land the close. Keyed on the LEVEL (proximity), NOT the change, so the approach is
    // untouched. Scale is dynamics-derived (cap is learned from the bug's own eats), not a fixed run
    // length. Complements the HeadingController speed_gate (orthokinesis on the ADVANCE). 0 = off.
    float tumble_level_gain_ = 0.0f;
    float tumble_min_     = 0.0f;    // tumble-probability clamp (low end)
    float tumble_max_     = 0.5f;    // tumble-probability clamp (high end)

    float peak_decay_     = 0.0005f; // STRUCTURAL: decay of the slow scent-magnitude memory (halflife
                                     // ~1400 ticks). Long enough that a wilderness stretch doesn't erase
                                     // the remembered food-scent scale → the PRE-EAT bootstrap capability
                                     // stays a HONEST current/typical-smell ratio, not "recent max".
    float eat_scent_alpha_ = 0.2f;   // per-EAT EMA rate of eat_scent_ (the scent at which klino actually
                                     // eats). Fast per-event (hits are rare) → converges over ~5 eats to
                                     // the typical eating-range scent = the capability denominator.
    float tumble_range_    = 1.5708f;// max reorient per tumble (rad). ±π/2 keeps the new
                                     // direction in front → HeadingController turns forward,
                                     // not reverse (a full ±π tumble points behind → erratic).
    float stuck_vel_thresh_= 0.5f;   // forward |vel| below this during a RUN = blocked
    int   stuck_ticks_     = 10;     // sustained-blocked ticks → forced tumble (honest egomotion
                                     // action-consequence: "I commanded a run but didn't move").
    bool  shuffle_         = false;  // ABLATION: gradient-blind ~50% random tumble = TRUE RANDOM WALK

    // --- KF1 RUN INTEGRITY (opt-in; default off = byte-identical) ---
    // The per-tick tumble draw (aa9dc7c) fires a fresh Bernoulli EVERY tick, including the
    // multi-tick turn transient after a tumble while the body is still reorienting toward the
    // new run_dir (thrust ≈ 0 under turn-priority → no travel → error ≈ 0 → p_tumble ≈ base →
    // ~65% of runs re-tumble before the run even starts). The biological drift needs a
    // COMMITTED run. run_commit: a tumble decision belongs to a RUN — while reorienting toward
    // run_dir, suppress the tumble draw AND the stuck counter; resume once the body faces it
    // (|Δheading| < turn-exit). The "clock" is the executed reorientation (the body's own
    // dynamics), not a fixed cycle count — no reintroduced scaffold clock.
    bool  run_commit_      = false;

    // --- KF6 DIRECTIONAL BELIEF (opt-in; default off = byte-identical) ---
    // Morph the tumble from a tabula-rasa UNIFORM draw to a belief-BIASED choice (kinesis →
    // reactive taxis). The gradient DIRECTION is a hidden external state (§2.1) — never READ
    // from a percept (that was the de-scaffolded homing ring), but INFERRED from the history of
    // the agent's OWN run outcomes: after a run in absolute direction θ that RAISED scent, θ is
    // a better bet than 90°/180° away. A circular accumulator g = EMA(outcome·unit(θ)); the
    // mean-resultant-length R = |g|/EMA(|outcome|) ∈[0,1] is the belief precision (directional
    // consistency), SELF-SCALED from the agent's own consequences (no tuned crossover, §2.3/§6);
    // μ = atan2(g) is the believed up-gradient heading. The tumble CENTRE is blended from the
    // current heading toward μ by w = R: R≈0 (tabula rasa / flat / conflicting) → the uniform
    // draw off the current heading (byte-identical floor); R→1 → tumbles centre on μ. Adaptive
    // precision gives free perturbation recovery: bad runs collapse R → the cone re-broadens →
    // re-inference. Reactive (one-run lookahead), not prospective EFE (§2.2).
    bool  dir_belief_      = false;
    float dir_lr_          = 0.1f;   // EMA rate of the directional belief (short timescale ~10-20 runs)
    // The belief BUILDS slowly (per-run, on a good outcome) but must COLLAPSE fast when the
    // committed direction stops paying off — otherwise a relocating source (food_alternate) or a
    // sharp bend leaves it locked on a STALE heading and run_commit holds that bad direction (the
    // L-maze regression). dir_decay_on_loss: each tick the scent is FALLING (the module's own
    // methylation error_n < 0 = "this isn't working"), bleed the directional accumulator toward 0 at
    // a rate ∝ |error_n| — precision R collapses within the run → the cone re-broadens → the next
    // tumble re-infers. Asymmetric (quick to abandon, deliberate to form) = correct for a moving
    // source. Uses the agent's OWN prediction error (§1) + downvoting (§8); no new tuned constant
    // (the rate is dir_lr scaled by the error). Default on (the correct belief).
    bool  dir_decay_on_loss_ = true;

    // --- KF3 AUTHORITY-AWARENESS (opt-in via authority_topic; needed for run_commit in the L2
    // multi-loop arbiter). klino only drives the body when the arbiter grants it authority; while
    // MUTED the heading is driven by other loops, so its committed run_dir goes stale and the
    // run_commit reorientation latch would freeze (in_turn never clears). And a run executed under
    // another loop's control must NOT be credited to klino's directional belief (error attribution,
    // §5). When muted: COAST — track the body's actual heading (run_dir=heading, no stuck reorient),
    // and suppress the tumble decision + belief update (no miscredit). On regaining authority klino
    // resumes a fresh run from wherever the body is. Empty topic = no authority read (byte-identical).
    std::string authority_topic_ = "";
    float authority_floor_ = 0.5f;   // authority below this = muted (another loop / a reflex has the bus)
    float authority_       = 1.0f;   // latest MotorBus authority share for the klino channel ∈[0,1]
    bool  have_authority_  = false;  // an authority reading has arrived (topic wired)

    uint64_t master_seed_  = 11;

    // latest inputs
    float smax_ = 0.0f, heading_ = 0.0f, vel_fwd_ = 0.0f;
    bool  have_scent_ = false;
    int   stuck_counter_ = 0;
    int   forced_tumbles_ = 0;

    // methylation state (the chemical memory + its derived signals)
    float baseline_      = 0.0f;     // methylation EMA of scent = the prediction of expected scent
    float error_scale_   = 0.0f;     // running |error| scale, for normalisation
    float last_p_tumble_ = 0.0f;
    float last_error_    = 0.0f;

    // SELF-REPORTED CAPABILITY (the nested blanket, §2.1): klino is a chemotaxer — with no
    // scent gradient its expected PRAGMATIC gain is ~0, so it tells the arbiter so. This is
    // honest capability, not suppression: capability = (current smell) / (typical EAT smell),
    // EMERGENT from klino's OWN experience — never a fixed `if scent<eps` gate (§6). The
    // denominator is EAT-CALIBRATED (eat_scent_, learned from real hits) once the bug has eaten;
    // a slow-decaying running peak bootstraps it before the first eat.
    float scent_peak_    = 0.0f;     // slow-decaying scent-magnitude memory (telemetry)
    float eat_scent_     = 0.0f;     // EMA of the scent at which it actually EATS (the calibrated denom)
    bool  have_eat_scent_ = false;   // true once the bug has eaten at least once (switch off the bootstrap)
    bool  eat_pending_   = false;    // a real eat fired → fold eat_scent_sample_ into eat_scent_ this tick
    float eat_scent_sample_ = 0.0f;  // scent captured AT the eat moment (food moves right after → sample now, not at tick)
    float capability_    = 0.0f;     // ∈[0,1]: →0 when blind (smax≈0), →~1 in its own eating range

    // run/tumble state
    float run_dir_abs_   = 0.0f;     // committed run direction (egomotion frame)
    bool  have_run_dir_  = false;
    int   committed_action_ = 0;     // run at start
    int   tumble_count_ = 0, run_count_ = 0;
    float out_vx_ = 0.0f, out_vy_ = 1.0f;

    // KF1 run-integrity state
    bool  reorienting_   = false;    // set on a tumble; cleared once the body faces run_dir (executing)
    bool  turning_       = false;    // turn-commit latch (anti-dither at a behind-target); mirrors planner/playloop
    float turn_dir_      = 1.0f;     // committed turn direction while turning_

    // KF0 run-length-asymmetry telemetry (always on; pure observation, no behavior change)
    float run_start_scent_ = 0.0f;   // smax_ at the current run's start (for the run's net-Δscent outcome)
    int   run_ticks_       = 0;      // ticks elapsed in the current run
    float run_len_up_      = 0.0f;   // EMA run length for runs that raised scent
    float run_len_down_    = 0.0f;   // EMA run length for runs that lowered scent
    float turn_frac_       = 0.0f;   // EMA fraction of ticks spent reorienting (|Δheading| > turn-exit)
    int   forced_in_turn_  = 0;      // forced tumbles that fired while still reorienting (K2 signature)

    // KF6 directional-belief state (the good-direction resultant + its self-scale)
    float belief_x_ = 0.0f, belief_y_ = 0.0f;  // EMA(outcome · unit(run_dir_abs)) — the good-direction resultant
    float belief_absw_ = 0.0f;       // EMA(|outcome|) — total outcome weight (R = |belief| / belief_absw)
    float last_R_   = 0.0f;          // directional consistency ∈[0,1] = belief precision (telemetry)
    float last_mu_  = 0.0f;          // believed up-gradient absolute heading (telemetry)

    std::mt19937 rng_{11};
};

}  // namespace ogma
