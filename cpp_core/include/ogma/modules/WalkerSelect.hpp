// =============================================================================
// WalkerSelect.hpp  --  which walker would carry the body better here?  (in shadow)
// =============================================================================
//
// Learned stepper plan section 9.6 item 2 (2026-11-19).  Two walkers act through one set of legs: the load-phase rhythm and
// the pose sequencer.  They share one goal (progress where the intent points), so this is not the intent arbiter's race of
// policies with different goals: it is a selection at the locomotion level, below it, on one shared quantity -- the progress
// each walker expects on the ground under the body, in m/s of the body's own odometry (WalkerCompetence, one per walker).
//
// THE RULE (the recipe's form, one step, greedy; loop_and_arbitration_recipe.md section 3).  For walker i with report
// [est_i, sd_i, ..., n_i, cost_i]:
//     G_i = est_i + optimism * sd_i                         expected progress plus what is not known about it
//     the incumbent c is the walker that HAS the legs (read from the pose socket), the challenger j the other
//     value_j = G_j * max(0, 1 - cost_j / horizon_s)        the hand-over costs cost_j seconds of the horizon (measured by j)
//     lead    = value_j - G_c
//     want    = j once the lead has been positive for as long as the hand-over to j takes (cost_j seconds, at least
//               dwell_min_s), else c;  a walker never driven (n = 0) leads by definition once the incumbent has a grade
// The hysteresis is the hand-over's own duration, not the recipe's spread of the lead: here the two walkers' rates differ by
// a factor of ten or more, so the step from "walker at pace" to "walker stalled" is itself most of the lead's variance, and
// a threshold in spreads holds the selection for a minute after the very change it should answer (worked through before the
// build, ledger 2026-11-19).  Persistence is scale-free, and a lead that flickers on rough ground never completes it.
//
// IN SHADOW.  Nothing consumes the output: the legs follow whatever sets pose_sequencer.engage (a schedule, the operator).
// The module says, each tick, which walker it would hand the legs to given who has them, and counts how often that agrees.
// Output ProprioToken [want (0 = the rhythm, 1 = the sequencer), lead, lead_s, G_rhythm, G_sequencer, incumbent, agree].
// Module absent = byte-identical.
#pragma once

#include <array>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class WalkerSelect : public Module {
public:
    WalkerSelect();
    ~WalkerSelect() override;

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

    int    want()      const { return want_; }
    int    incumbent() const { return cur_; }
    double lead()      const { return lead_; }
    double lead_s()    const { return double(lead_ticks_) / hz_; }
    double g(int i)    const { return g_[size_t(i)]; }
    int64_t calls()    const { return calls_; }
    int64_t agree_ticks() const { return agree_; }
    int64_t ticks()    const { return ticks_; }

private:
    struct Report { double est = 0.0, sd = 0.0, n = 0.0, cost = 0.0; bool have = false; };
    std::array<std::string, 2> report_topics_{};     // [0] the rhythm's WalkerCompetence output, [1] the sequencer's
    std::string share_topic_  = "pose.targets";      // value share_index = the sequencer's share of the command
    int         share_index_  = 0;
    std::string output_topic_ = "select.walker";
    double optimism_     = 1.0;    // kappa: one sd of what is not known counts as progress
    double horizon_s_    = 30.0;   // SCAFFOLD: the stretch a hand-over is priced against
    double dwell_min_s_  = 2.0;    // the lead must hold at least this long while a walker's switching cost is still unmeasured
    double hz_           = 50.0;

    std::array<Report, 2> r_{};
    double share_ = 0.0; bool have_share_ = false;
    std::array<double, 2> g_{};
    int    cur_ = 0, want_ = 0, prev_cur_ = 0; bool calling_ = false;
    double lead_ = 0.0; int64_t lead_ticks_ = 0;
    int64_t ticks_ = 0, agree_ = 0, calls_ = 0; std::array<int64_t, 2> want_ticks_{};
};

} // namespace ogma
