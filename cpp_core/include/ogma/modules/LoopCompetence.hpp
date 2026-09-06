// =============================================================================
// LoopCompetence.hpp  --  a loop graded by whether the world moves as it says
// =============================================================================
//
// Cell round 4 (2026-09-06), register O21.  Precision-weighted arbitration graded on a
// loop's own bearing stream was NULL: a steady route and a smooth wander are both
// "predictable" (Cell ledger, R3).  What a policy's precision should mean is whether its
// prediction about the WORLD holds while it acts: klino says scent rises while it runs,
// the planner says its route value rises as it closes on remembered food, play says novelty
// rises as it climbs, vision says its target confidence rises as it homes.
//
// This module watches one loop's objective stream and the arbiter's gain for that loop.
// Over each window of `horizon_ticks` during which the loop drove continuously, the
// prediction "the objective improves" is checked once: sign·(o_t − o_{t−H}) > 0.  The
// competence c is an EMA of those checks (a fraction in [0,1], scale-free: no constant is
// tuned to the objective's units); while the loop is NOT driving, c relaxes toward the
// uninformed prior 0.5 at `forget` per tick (uncertainty grows without observation, the
// Kalman Q).  It publishes a RealityToken on reality.<group>.<name> with
//     expected_error = tle = quant_error = 1 − c,   latent = [c],   winner_id 0, baked 3
// so a LateralVoter (trust 1/(err+ε) or the inverse-variance `trust_source expected`) grades
// the loops by competence and EFEArbiter's `scoring_mode precision` selects by need × trust.
// The module absent = byte-identical.
#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class LoopCompetence : public Module {
public:
    LoopCompetence();
    ~LoopCompetence() override;

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

    float    competence()    const { return c_; }
    float    published()     const { return pub_; }
    double   beta_a() const { return a_; }
    double   beta_b() const { return b_; }
    uint64_t checks()        const { return checks_; }
    uint64_t improvements()  const { return improvements_; }
    bool     driving()       const { return driving_; }
    std::string const& output_topic() const { return output_topic_; }

private:
    void handle_objective(MessagePtr p);
    void handle_gain(MessagePtr p);

    std::string objective_topic_ = "";                 // ProprioToken scalar, or a RealityToken (field below)
    std::string objective_field_ = "value";            // "value" (ProprioToken values[index]) | "tle" (RealityToken.tle)
    int         objective_index_ = 0;
    std::string gain_topic_      = "";                 // arbiter.gain.<loop>: this loop drives when > 0.5
    std::string modality_group_  = "loop";
    std::string modality_name_   = "";
    std::string output_topic_    = "";                 // reality.<group>.<name>
    float  sign_          = 1.0f;                      // +1: the loop predicts the objective RISES while it drives
    int    horizon_ticks_ = 30;                        // the loop's prediction horizon (checked once per window of continuous driving)
    float  alpha_         = 0.05f;                     // EMA rate of the competence over checks
    float  forget_        = 0.001f;                    // per-tick relaxation toward 0.5 while not driving
    float  prior_         = 0.5f;
    // Second form (2026-09-06): a Beta posterior over the competence with OPTIMISM.  The first
    // form's mean left a never-selected loop at the prior forever (the planner lost every tie and
    // was never observed).  estimator="beta": a = 1 + improvements, b = 1 + failures (counts
    // forgotten at `forget` per tick while not driving, so uncertainty GROWS), and what is
    // published is the upper credible bound mean + optimism·sd -- an untried loop reads high and
    // gets tried; a proven-poor one reads low with certainty.  optimism 0 = the posterior mean.
    std::string estimator_ = "ema";                   // "ema" (default, the first form) | "beta"
    float  optimism_ = 0.0f;                           // κ: published competence = mean + κ·sd of the Beta posterior
    double a_ = 1.0, b_ = 1.0;                         // Beta pseudo-counts

    float    obj_ = 0.0f;  bool have_obj_ = false;
    float    gain_ = 0.0f;
    bool     driving_ = false;
    int      run_ticks_ = 0;                            // consecutive driving ticks
    float    o_start_ = 0.0f;                           // objective at the start of the current window
    float    c_ = 0.5f;
    float    pub_ = 0.5f;                               // the competence actually published (mean, or the optimistic bound)
    uint64_t checks_ = 0, improvements_ = 0;
};

} // namespace ogma
