// =============================================================================
// WalkerCompetence.hpp  --  a walker graded by whether the body moves as it expects while it holds the legs
// =============================================================================
//
// Learned stepper plan section 9.6 item 2 (2026-11-19): the selection between the two walkers (the load-phase rhythm and the
// pose sequencer), in shadow.  The two share one goal -- carry the body where the intent points -- and differ in how well their
// stepping turns into progress on the ground under them.  This module is ONE walker's report of that, in units the two share:
// metres per second of the body's own stride odometry.  One instance per walker; an instrument, nothing reads it yet.
//
// WHAT IT GRADES.  While its walker holds the legs (its share of the command >= drive_min) and the brain commands forward:
//     realised = v_fwd * (1 - range_score)
// v_fwd is the stride odometry's forward speed (reality.proprio.stride_v, egocentric).  range_score is StuckSense's range
// channel (the surface ahead not closing by what the stride claims), because the odometry alone does not see a stall: measured
// on the walker alone (ledger 2026-11-19), pressed on a 64 mm riser the stride reads 1.1 cm/s where the body makes 0.3, and the
// discounted value reads 0.3.  rate = EMA(realised) over tau_s of GRADED ticks; mean and var are the walker's own long-run
// statistics of that rate (count-annealed down to 1 / (mean_tau_s * hz)): what it does on the grounds it has driven.
//
// A WALKER THAT IS NOT IN CONTROL DOES NOT INTEGRATE (ledger 2026-11-18, the amplitude homeostat's lesson).  While the other
// walker has the legs the rate, the mean and the variance are held.  What is published then is the walker's estimate for the
// ground the body is on NOW, and the body has moved since it last drove:
//     p_same = exp(-idle_m / relax_m - idle_s / relax_s)        (a term whose scale is 0 is left out)
//     est    = p_same * rate_at_handoff + (1 - p_same) * mean,   sd = (1 - p_same) * sqrt(var)
// idle_m is the odometry's NET forward travel since the hand-off (not its path length: the first shadow battery counted the
// sequencer's shifts, a metre of rocking a minute on a body that had not left its riser), idle_s the seconds.  The relaxation
// scales are SCAFFOLD constants
// (a body length; a retry time): the learned form is a context -- the terrain ahead as a vocabulary, the walker's rate
// remembered per node -- and is the next lever, not this one.  relax_m = relax_s = 0 holds the estimate where it stood.
//
// THE SWITCHING COST, MEASURED.  When the walker takes the legs it is not at its pace at once (the sequencer settles onto its
// neutral toes for about ten seconds, the rhythm needs two or three).  From the take-over the module counts the seconds until
// the live rate first reaches half the walker's own mean and keeps an average of them (switch_cost_s); during that settle the
// published estimate stays the one the walker was handed the legs on, so a selector does not read the settle as a stall.  A
// settle that outlasts settle_cap_s ends unrecorded: the walker did not get going, and the live rate says so.
//
// OUTPUT.  ProprioToken on output_topic:
//     [0 est (m/s), 1 sd, 2 driving and graded (0/1), 3 rate (held when idle), 4 mean, 5 graded ticks, 6 switch_cost_s,
//      7 p_same, 8 settling (0/1), 9 idle_m]
// Everything is egocentric; no constant names a speed.  Module absent = byte-identical.
#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class WalkerCompetence : public Module {
public:
    WalkerCompetence();
    ~WalkerCompetence() override;

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

    double est()       const { return est_; }
    double sd()        const { return sd_; }
    double rate()      const { return rate_; }
    double mean()      const { return mean_; }
    double var()       const { return var_; }
    double p_same()    const { return p_same_; }
    double switch_cost_s() const { return cost_s_; }
    int64_t graded()   const { return n_; }
    int64_t takeovers() const { return takeovers_; }
    int64_t costs()    const { return cost_n_; }
    bool   driving()   const { return driving_; }
    bool   settling()  const { return settling_; }

private:
    std::string stride_topic_ = "reality.proprio.stride_v";   // ProprioToken [x, y]: the stride odometry's velocity
    int         stride_index_ = 1;                            // y = forward
    std::string stuck_topic_  = "";                           // StuckSense's output; "" = no discount
    int         stuck_index_  = 4;                            // the range channel alone, smoothed
    std::string share_topic_  = "";                           // ProprioToken: a walker's share of the command at share_index
    int         share_index_  = 0;
    bool        share_invert_ = false;                        // true: this walker's share is 1 - the value (the rhythm under the pose socket)
    std::string speed_topic_  = "";                           // the forward command; "" = always commanding
    std::string output_topic_ = "";
    double drive_min_    = 0.99;   // the walker holds the legs when its share is at least this
    double cmd_min_      = 0.05;   // no forward command, no evidence
    double tau_s_        = 2.0;    // the rate's EMA, in graded seconds (the walker's own cadence: a stride, a few steps)
    double mean_tau_s_   = 600.0;  // the long-run statistics' memory, in graded seconds
    double relax_m_      = 0.25;   // SCAFFOLD: the ground is another ground after about a body length
    double relax_s_      = 0.0;    // SCAFFOLD: a retry time; 0 = time alone changes nothing
    double settle_cap_s_ = 30.0;   // a settle longer than this is not a settle
    double hz_           = 50.0;

    // inputs as last heard
    double vfwd_ = 0.0, stuck_ = 0.0, share_ = 0.0, speed_ = 1.0;
    bool   have_share_ = false, have_speed_ = false;

    // the walker's own statistics (held while it does not drive)
    double  rate_ = 0.0, mean_ = 0.0, var_ = 0.0;
    int64_t n_ = 0;
    // the take-over and its settle
    bool    driving_ = false, settling_ = false;
    double  live_ = 0.0;              // the rate since the take-over, from zero
    int64_t since_ = 0;               // ticks since the take-over
    double  handed_est_ = 0.0, handed_sd_ = 0.0;   // what the walker was handed the legs on
    double  cost_s_ = 0.0; int64_t cost_n_ = 0, takeovers_ = 0;
    // idle
    double  hold_ = 0.0;              // the rate at the hand-off
    double  idle_net_ = 0.0, idle_m_ = 0.0; int64_t idle_ticks_ = 0;   // the signed sum of the stride's forward speed, and its magnitude
    double  p_same_ = 1.0, est_ = 0.0, sd_ = 0.0;
};

} // namespace ogma
