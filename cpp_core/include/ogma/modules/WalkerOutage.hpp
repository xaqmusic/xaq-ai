// =============================================================================
// WalkerOutage.hpp  --  the supply's outages, attributed to the walker that held the legs
// =============================================================================
//
// Learned stepper plan section 9.10 (2026-11-20, operator): "when I place the robot on mulch, soft carpet, or other uneven
// surfaces, the stepper will automatically become engaged because the brownouts reach a certain threshold per minute ... on
// flat surfaces the power consumption is reasonable and we want the robot to move quickly."
//
// THE OBSERVATION.  On the robot a brownout is the HAT's 3.3 V rail folding under the servos' demand until its MCU resets
// (ledger 2026-10-05: every deep dip comes with the battery current at its ceiling; the MCU rides out 2.65 V and resets by
// 2.5 V).  The daemon counts those resets; the simulator's power model produces the same event from its rail.  Both sides put
// the count on one topic (sense.hat_reset [count, ...]) and the rail on another (sense.hat_rail [V]).  This module is the
// reduction of those two channels into what a selection between the walkers needs: how many outages each walker has cost
// in its last minute at the legs.
//
// PER WALKER w (0 = the rhythm, 1 = the pose sequencer):
//     n_w    = the outages attributed to w inside its last window_s seconds OF DRIVING (a weight each; see the take-over)
//     dips_w = the same for the rail's dips (ENTRIES below dip_v, one per excursion: a dwell is one observation)
// An outage is attributed to the walker with the larger share of the command at that tick.  A WALKER THAT IS NOT IN CONTROL
// DOES NOT INTEGRATE (ledger 2026-11-18): while the other walker has the legs w's clock stands still and its outages do not
// age.  What is published for it then is its estimate for the ground under the body NOW:
//     p_same = exp(-idle_m / (relax_m * 2^backoff_w)),   est_w = p_same * n_w
// idle_m is the odometry's net forward travel since the hand-off.  At the take-over the weights are multiplied by p_same, so
// the walker starts on what it was believed to cost here and one more outage counts on top of it.
//
// THE VETO (a reflex, and a SCAFFOLD).  A reset is the fatal event of this loop and has no gradient: veto_w is raised when
// est_w >= veto_n and dropped when est_w <= release * veto_n.  veto_n is not free: the robot's daemon stops recovering on its
// own past three outages in sixty seconds, so the brain has to act below that.  Each veto doubles the travel the next retry
// waits for (backoff_w, capped), and a full clean window at the legs clears it: the retry is count-annealed, not a timer.
// Until that clean window the walker is on probation: one more outage vetoes it again, so a retry costs one brownout.
// The learned form this stands in for is the rail's dip rate per ground (dips_w is published for it and nothing reads it yet).
//
// OUTPUT.  ProprioToken on output_topic:
//     [0 est_rhythm, 1 est_sequencer, 2 veto_rhythm, 3 veto_sequencer, 4 dips_rhythm, 5 dips_sequencer, 6 driver (-1 none),
//      7 resets seen, 8 p_same_rhythm, 9 p_same_sequencer, 10 backoff_rhythm, 11 backoff_sequencer]
// Module absent = byte-identical.  An instrument: WalkerSelect reads it only when told to (outage_topic).
#pragma once

#include <array>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class WalkerOutage : public Module {
public:
    WalkerOutage();
    ~WalkerOutage() override;

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

    double est(int w)     const { return est_[size_t(w)]; }
    double count(int w)   const { return sum(ev_[size_t(w)]); }
    double dips(int w)    const { return sum(dip_[size_t(w)]); }
    bool   veto(int w)    const { return veto_[size_t(w)]; }
    int    backoff(int w) const { return backoff_[size_t(w)]; }
    double p_same(int w)  const { return p_same_[size_t(w)]; }
    int    driver()       const { return driver_; }
    int64_t resets_seen() const { return resets_seen_; }

private:
    struct Ev { double age_s = 0.0, w = 1.0; };
    static double sum(std::vector<Ev> const& v) { double s = 0.0; for (auto const& e : v) s += e.w; return s; }

    std::string reset_topic_  = "sense.hat_reset";            // ProprioToken [count, ...]: outages since the host started
    std::string rail_topic_   = "sense.hat_rail";             // ProprioToken [V]; "" = no dips
    std::string share_topic_  = "pose.targets";               // value share_index = the sequencer's share of the command
    int         share_index_  = 0;
    std::string stride_topic_ = "reality.proprio.stride_v";   // the stride odometry's velocity; stride_index = forward
    int         stride_index_ = 1;
    std::string output_topic_ = "select.outage";
    double window_s_   = 60.0;   // the daemon's own window
    double veto_n_     = 2.0;    // SCAFFOLD: outages in a window at which a walker is vetoed (the daemon gives up past 3)
    double release_    = 0.5;    // the veto drops at release * veto_n
    double relax_m_    = 0.25;   // SCAFFOLD: a body length of net travel (WalkerCompetence's)
    int    backoff_max_ = 4;
    double dip_v_      = 3.0;    // a rail dip is an entry below this (the robot: 15 such per fatal one)
    double drive_min_  = 0.99;
    double hz_         = 50.0;

    double reset_count_ = 0.0; bool have_reset_ = false; double last_count_ = 0.0; bool primed_ = false;
    double rail_ = 3.3; bool have_rail_ = false; bool in_dip_ = false;
    double share_ = 0.0; bool have_share_ = false;
    double vfwd_ = 0.0;

    std::array<std::vector<Ev>, 2> ev_{}, dip_{};
    std::array<double, 2> est_{}, p_same_{{1.0, 1.0}}, idle_net_{}, clean_s_{};
    std::array<bool, 2>   veto_{}, driving_{};
    std::array<int, 2>    backoff_{};
    int     driver_ = -1;
    int64_t resets_seen_ = 0, dips_seen_ = 0, ticks_ = 0;
    std::array<int64_t, 2> vetoes_{};
};

} // namespace ogma
