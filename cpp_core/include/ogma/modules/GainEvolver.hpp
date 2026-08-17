#pragma once

// =============================================================================
// GainEvolver — PART IV: the adaptive gains substrate (v1).
// =============================================================================
//
// A (1+1)-ES over a config-declared gain vector of a consumer module
// (MotorEPMv2's high-value sliders in v1), run over the robot's lifetime from
// within its own Markov blanket.  The charter:
// docs/plans-and-designs/adaptive_gains_substrate_plan.md — the question is
// whether the high-value gains can SETTLE AND STAY ADAPTABLE from egocentric
// sensors alone, rather than being found by hand or frozen by external search.
//
// THE LOOP (interleaved incumbent re-evaluation — an operator fork decision):
//   warmup → [incumbent window → candidate window → compare → accept/revert]*
// Each generation re-measures the incumbent in its own window, so scores are
// always CONTEMPORANEOUS.  There is no stored best-score and no slow-forget:
// "stores a winner" is the residual ratchet shape the ledger flags (a lucky
// escape thrash becomes the incumbent and everything reverts back to it), and
// a stored score also goes stale the moment the world changes — which is
// exactly the mid-run (d)-test this phase must pass.
//
// THE CRITERION (error-form ONLY, §5.1 — never raw speed):
//   J = w_falls·falls + w_tilt_var·var(upright) + w_distress·distress_duty
//     + w_unloaded·unloaded_contact_mean + w_flow·(1 − flow_quality_mean)
// computed EXCLUSIVELY from body-published reality.proprio.* topics with the
// module's own EMAs — never from the consumer's internal state, which its own
// homeostats bend (the stationary-evaluator burns: a self-referential
// threshold is not a sensor; a live homeostat silently restores the measured
// quantity).  flow_quality (magnitude × predictability of fwd-flow) is the one
// sanctioned speed-flavored term, form lifted from MotorEPMv2's fwd-flow
// homeostat.  Post-plant slip is DEFERRED from v1: no egocentric slip signal
// exists in the codebase (lateral_v is a soft oracle).
//
// THE GUARDS (each is a recorded PART III burn, all designed in):
//   G1  falls no-regression        (candidate falls ≤ incumbent falls + tol)
//   G2  PER-LEG loaded-contact minima (min over legs, never a group mean —
//       the stance-capture lesson: the GLOBAL amp homeostat satisfied the
//       group mean by over-driving the living legs while one leg was dead)
//   accept ⇔ G1 ∧ G2 ∧ (J_cand < J_inc)   — the viability guard is SEPARATE
//   from the improvement criterion (the demo-mask lesson: "target wins, body
//   pays" must be rejected, not netted).
//
// GAIN-0 GUARD: mutation_sigma == 0 ⇒ SILENT OBSERVER — the evaluator still
// scores windows (criterion visible on the promoted stack before any search),
// but the module publishes NOTHING, draws NO RNG, and mutates nothing:
// byte-identical behavior.
//
// σ self-anneals (1/5th-success flavor) between sigma_min/sigma_max; an
// operator write to mutation_sigma overrides and clears the anneal history.
//
// Module lifecycle authoring contract: primitives/_module_lifecycle.md.

#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class GainEvolver : public Module {
public:
    GainEvolver();
    ~GainEvolver() override;

    std::string_view type_name() const override;
    std::vector<TopicSpec> input_topics() const override;
    std::vector<TopicSpec> output_topics() const override;
    ParamSchema params_schema() const override;
    ParamMap current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    void tick(uint64_t tick_id) override;

    nlohmann::json snapshot_state() const override;
    void restore_state(nlohmann::json const& s) override;
    nlohmann::json diag_snapshot() const override;

    // Cheap per-tick scalars for OgmaBrain::get_module_metrics — the body-log
    // mirror reads this every diag emit; the full snapshot is NOT the per-tick
    // path (the diag_snapshot ZMQ lesson, OgmaBrain.cpp).
    nlohmann::json metrics() const;

private:
    enum class Phase : int { Warmup = 0, Incumbent = 1, Candidate = 2 };

    // Per-window accumulators.  Continuous terms measure the BACK HALF only
    // (front half = settling, the coord-search precedent); falls count the
    // whole window (rare, discrete, attributable to the vector under test).
    struct WindowStats {
        int      falls = 0;
        double   up_sum = 0.0, up_sq = 0.0;
        int64_t  meas_n = 0, distress_hits = 0;
        double   flow_q_sum = 0.0;
        std::vector<int> td, unloaded;      // per-leg touchdowns / unloaded verdicts
        void reset(int n_legs);
    };
    // A scored window, kept for diag + the guard comparison.
    struct Terms {
        double falls = 0, tilt_var = 0, distress_duty = 0,
               unloaded_mean = 0, flow_term = 0, loaded_min = 0, J = 0;
        bool   valid = false;
    };

    // ---- handlers (cache latest values; edge logic runs in tick()) ----------
    void handle_upright(MessagePtr payload);
    void handle_distress(MessagePtr payload);
    void handle_foot_load(MessagePtr payload);
    void handle_foot_contact(MessagePtr payload);
    void handle_imu(MessagePtr payload);

    // ---- loop internals ------------------------------------------------------
    void  start_window(Phase p);
    void  publish_vector(bool candidate);
    void  mutate_candidate();
    void  anneal(bool accepted);
    Terms score(WindowStats const& w) const;
    double per_leg_loaded_min(WindowStats const& w) const;
    bool  viability_ok(WindowStats const& cand, WindowStats const& inc) const;

    // ---- declared vector (ConstructionOnly; parallel arrays) -----------------
    std::vector<std::string> gain_keys_;
    std::vector<double> gain_seed_, gain_min_, gain_max_, gain_sigma_scale_;
    std::string gain_topic_ = "gain.motor_epm";

    // ---- input topics --------------------------------------------------------
    std::string upright_topic_      = "reality.proprio.upright";
    std::string distress_topic_     = "reality.proprio.distress";
    std::string foot_load_topic_    = "reality.proprio.foot_load";
    std::string foot_contact_topic_ = "reality.proprio.foot_contact";
    std::string imu_topic_          = "reality.proprio.imu";

    // ---- timing / loop params ------------------------------------------------
    int     n_legs_            = 4;
    int64_t warmup_ticks_      = 1500;
    int64_t eval_window_ticks_ = 4000;
    int64_t seed_              = 0;       // OGMA_SEED override rewrites "seed"
    int64_t republish_every_   = 0;       // 0 = publish on window boundaries only

    // ---- criterion weights (HotMutable; FIXED during a run = the stationary
    // evaluator — retune between runs, never let the loop touch them) ---------
    double w_falls_    = 1.0;
    double w_tilt_var_ = 5.0;
    double w_distress_ = 0.5;
    double w_unloaded_ = 0.5;
    double w_flow_     = 0.5;

    // ---- guard + detector params (HotMutable) --------------------------------
    double  viability_load_tol_    = 0.05;
    int64_t viability_falls_tol_   = 0;
    double  upright_fall_thresh_   = 0.0;
    int64_t fall_debounce_ticks_   = 25;   // MUST stay under the body's 30-tick
                                           // inversion dwell or auto-reset snaps
                                           // upright back before the edge fires
    double  distress_thresh_       = 0.05;
    double  load_thresh_           = 0.05;
    int64_t touchdown_horizon_ticks_ = 12;
    int64_t min_touchdowns_        = 3;

    // ---- flow form (copied from MotorEPMv2's fwd-flow homeostat) -------------
    double flow_alpha_    = 0.02;
    double flow_vol_k_    = 4.0;
    double flow_vel_norm_ = 0.05;

    // ---- anneal params -------------------------------------------------------
    int64_t anneal_window_ = 10;
    double  target_accept_ = 0.2;
    double  anneal_up_     = 1.5;
    double  anneal_down_   = 0.85;
    double  sigma_min_     = 0.01;
    double  sigma_max_     = 0.5;

    // ---- live ES state (ALL serialized) --------------------------------------
    std::vector<double> incumbent_, candidate_;
    Phase    phase_      = Phase::Warmup;
    int64_t  win_tick_   = 0;
    uint64_t generation_ = 0;
    int64_t  accepts_ = 0, reverts_ = 0, publishes_ = 0, overrides_ = 0;
    double   sigma_ = 0.0;                 // live annealed σ (0 = silent observer)
    std::deque<uint8_t> accept_hist_;
    std::string accept_log_;               // last ~24 outcomes, "A"/"R", diag only
    std::mt19937_64 rng_;
    WindowStats cur_, inc_stats_;
    Terms inc_terms_, cand_terms_;
    bool  need_publish_ = false;           // set by param transitions, fires next tick

    // ---- sensor caches + edge state (ALL serialized) -------------------------
    float upright_  = 1.0f;
    float distress_ = 0.0f;
    float fwd_v_    = 0.0f;
    std::vector<float> foot_load_, foot_contact_;
    int64_t fall_below_run_ = 0;
    bool    fall_latched_   = false;
    std::vector<float>   contact_prev_;
    std::vector<int64_t> td_horizon_;      // >0 = ticks left in post-touchdown watch
    std::vector<float>   td_maxload_;
    float flow_ema_ = 0.0f, flow_vol_ema_ = 0.0f;
};

} // namespace ogma
