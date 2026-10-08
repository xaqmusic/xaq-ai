// =============================================================================
// LookLoop.hpp  --  the stop-and-look as a loop (the duck, design doc §17.112, 2026-10-08)
// =============================================================================
//
// The host used to decide when the duck stops to look (a timer, an arrival) and when the look ends (the
// stop's cloud stops growing, a cap, a confirmed chase).  Those are scaffolds.  This module is the loop that
// owns the error they stand in for: the cloud's own surprise.
//
//   infers         whether standing still here would teach the cloud something
//   senses         CloudMap's state_topic [open, walking, voxels, new_fraction] -- new_fraction is the share of the
//                  voxels touched in the last second that were never seen before, the cloud's surprise; and a
//                  context (an EPM's winner over the descriptor this module composes: the place map's TLE over
//                  its own running peak, the seek loop's need, the outcome loop's need)
//   predicts       per context node, the surprise a stop resolves: the integral of new_fraction over a stop
//                  (seconds), a running mean and variance per node
//   honest signal  the realized integral at the stop's end against the prediction made at its start; during a
//                  stop, new_fraction itself, which falls as the cloud resolves
//   need           before a stop: the node's optimistic prediction (mean + optimism x sd / sqrt n; an untried node
//                  reads the running peak) over the running peak of realized integrals.  During one: new_fraction
//                  over this stop's own peak -- the host's cloud-end rule as a continuous need.
//
// A stop (an episode) is a stop's cloud being open: open and not walking -- and, with active_topic set, the
// arbiter's gain for the look above 0.5.  Passive (active_topic empty) the module only watches the host's stops.
// Publishes value_topic [need, prediction / peak, in a stop, the last realized integral, its prediction] and,
// optionally, driving_topic [in a stop] (a LoopCompetence's gain while the loop is passive) and descriptor_topic
// (the context EPM's input).  The module absent = byte-identical.
//
// need_mode "yield" (§17.112 lever 2, the pragmatic look): what a stop delivers is something to go to or watch -- a
// stop PAYS when a seek target appears (the seek need rises from 0) or a skill is requested during it.  Per context
// cell a Beta posterior of the payoff (published need before a stop: its mean + optimism x sd); during a stop, the
// probability a payoff is still to come given none yet, P S(t) / (1 - P + P S(t)), S the learned distribution of the
// time to a payoff (over every stop that paid); 0 once it paid.  trust_topic publishes the look's calibration as a
// RealityToken (expected_error = the mean Brier score of its predictions, 0.25 before any) for the voter.
#pragma once

#include <deque>
#include <map>
#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class LookLoop : public Module {
public:
    LookLoop();
    ~LookLoop() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;
    void           restore_state(nlohmann::json const& s) override;

    double   need()       const { return need_; }
    double   prediction() const { return pred_norm_; }
    bool     in_stop()    const { return in_ep_; }
    uint64_t episodes()   const { return episodes_; }
    double   last_realized()  const { return last_realized_; }
    double   last_predicted() const { return last_predicted_; }
    double   peak()       const { return peak_; }
    int      cell_n(int c) const { auto it = cells_.find(c); return it == cells_.end() ? 0 : it->second.n; }

private:
    struct Cell { int n = 0; double mean = 0.0, m2 = 0.0; double a = 1.0, b = 1.0; };
    double pay_mean(int cell) const;                // the Beta posterior's mean of a payoff here
    double pay_optimistic(int cell) const;          // ... + optimism x sd
    double still_to_come(double p, double tau) const;
    double predict(int cell) const;                 // seconds of surprise a stop here resolves (optimistic)
    int    context_now() const;

    std::string state_topic_, context_topic_, active_topic_;
    std::string place_topic_, seek_value_topic_, outcome_need_topic_;
    std::string value_topic_, driving_topic_, descriptor_topic_;
    std::string need_mode_ = "surprise", skill_topic_, trust_topic_;
    double optimism_ = 1.0;          // the prediction's credit to the unproven: mean + optimism x sd / sqrt(n)
    double peak_decay_ = 0.02;       // per stop: the running peak of realized integrals decays toward the latest
    double tick_hz_ = 50.0;
    int    min_stop_ticks_ = 25;     // a shorter stop's cloud (a twitch) teaches nothing about stops

    std::map<int, Cell> cells_;      // keyed by the context EPM's winner id (stable GNG ids); one cell without a context
    double peak_ = 0.0;              // the running peak of realized integrals (0 = no stop yet)
    double place_peak_ = 0.0;        // the place TLE's running peak (its own scale)
    bool   in_ep_ = false;
    int    ep_ticks_ = 0, ep_cell_ = 0;
    double ep_gain_ = 0.0, ep_nf_peak_ = 0.0, ep_pred_ = 0.0;
    double need_ = 1.0, pred_norm_ = 1.0, nf_ = 0.0;
    double last_realized_ = 0.0, last_predicted_ = 0.0, abs_err_sum_ = 0.0;
    uint64_t episodes_ = 0;
    // yield mode
    std::deque<double> pay_times_;   // seconds from a stop's start to its payoff, the most recent 400
    bool   ep_paid_ = false;
    double ep_p_ = 0.5, seek_prev_ = 0.0, brier_sum_ = 0.0;
    uint64_t paid_ = 0;
};

} // namespace ogma
