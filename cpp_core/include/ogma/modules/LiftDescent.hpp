// =============================================================================
// LiftDescent.hpp  --  G4a: the swing lift chosen by local descent on the gait's own errors
//                      (design note generator_and_belief_filter.md §5.1, operator 2026-10-25)
// =============================================================================
//
// One GLOBAL operating point u = (front, rear) — multipliers on MotorEPMv2's phase-shaped swing lift, read through its
// lift_handle_topic.  No situations yet (G4a); per-situation operating points are G4b, once a context vocabulary
// separates the situations that need different settings.
//
// The outcome is a set of PREDICTION ERRORS, never a reward (CLAUDE.md §5.1):
//   e_front, e_rear  the fraction of swing ticks in which a foot of that pair touched (the gait's phase said the foot
//                    is in the air; the contact sensor says it is not) — the error a swing lift exists to remove;
//   e_stall          StuckSense's stall score (commanding forward, stride speed short of the body's own recent best);
//   e_tilt           the spread of pitch and roll over the trial — the COMPLEMENT: "always lift high" would also lower
//                    swing contact, and is what this term catches (the blind-metric check).
// Each is normalised by its own running spread over trials (adapted, not tuned: §5.5) and summed.
//
// A linear outcome model always predicts a bound as optimal, so the selection is a DESCENT, not an argmin:
//   trials dither u ± δ along one axis at a time, held `hold_ticks` (the first `settle_ticks` not measured);
//   RLS (with forgetting) fits objective ≈ a + g · (u_trial − u) over valid trials;
//   when a slope component is significant (|g| > 2 standard errors) u steps a bounded amount downhill;
//   δ anneals with the trial count.  A trial is discarded if a reset event, distress, or too few swing ticks
//   occurred in it (conservative exploration, operator 2026-10-25).
//
// enabled = false (default): nothing is published, so MotorEPMv2 — which only scales its lift when a handle token
// has arrived — runs byte-identically.
// =============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class LiftDescent : public Module {
public:
    LiftDescent();
    ~LiftDescent() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    // ---- accessors for tests ----
    double u(int axis) const { return sit_const(sit_now_).u[size_t(axis)]; }
    double u_of(int situation, int axis) const { return sit_const(situation).u[size_t(axis)]; }
    int    situation() const { return sit_now_; }
    int    n_situations() const { return int(sits_.size()) - 1; }   // without the global point
    int    trials_valid() const { return trials_valid_; }
    int    trials_discarded() const { return trials_discarded_; }
    int    steps() const { return steps_; }

private:
    void handle_phase(size_t leg, MessagePtr p);
    void handle_contact(MessagePtr p);
    void handle_stall(MessagePtr p);
    void handle_tilt(MessagePtr p);
    void handle_distress(MessagePtr p);
    void handle_situation(MessagePtr p);
    void start_trial();
    void end_trial();
    // G4b (2026-11-02): per-SITUATION operating points.  A situation is the winner of a RealityToken stream
    // (situation_topic; the encounter EPM's node), held by an occupancy EMA so a per-footfall vocabulary becomes a
    // context that lasts a stride or more.  Each situation keeps its own u, slope estimate and trial count; the
    // outcome normalisation stays global (the spread of an error does not depend on where it was measured).  A trial
    // is attributed to the situation that held most of its measured ticks and discarded when none held
    // situation_min_frac of them.  Key -1 is the global point, the only one when situation_topic is empty, so G4a's
    // behaviour is byte-identical.
    struct Situation {
        std::array<double, 2> u{1.0, 1.0};
        Eigen::Vector3d theta = Eigen::Vector3d::Zero();
        Eigen::Matrix3d P     = Eigen::Matrix3d::Identity() * 100.0;
        double res_var = 1.0, res_n = 0.0;
        int trials_valid = 0, steps = 0;
    };
    Situation&       sit(int key);
    Situation const& sit_const(int key) const;
    double delta(Situation const& s) const;

    // config
    std::vector<std::string> phase_topics_;
    std::string contact_topic_  = "reality.proprio.foot_contact";
    std::string stall_topic_    = "reality.proprio.stuck";
    std::string tilt_topic_     = "reality.proprio.tilt";
    std::string distress_topic_;
    std::string output_topic_   = "handle.lift";
    std::string situation_topic_;            // "" = one global operating point (G4a)
    double situation_tau_ticks_ = 60.0;      // occupancy EMA of the winners (~2 strides)
    double situation_min_occ_   = 0.5;       // the leading occupancy a situation needs to take over
    double situation_min_frac_  = 0.6;       // the fraction of a trial's measured ticks the dominant situation needs
    int    situation_max_       = 16;        // more distinct winners than this fall back to the global point
    bool   enabled_       = false;
    double u_init_front_  = 1.0, u_init_rear_ = 1.0;
    double u_lo_          = 0.5, u_hi_ = 3.0;
    int    hold_ticks_    = 400;
    int    settle_ticks_  = 100;
    double delta0_        = 0.25;
    double delta_min_     = 0.05;
    double delta_anneal_  = 20.0;   // trials over which δ halves (δ = max(min, δ0 / sqrt(1 + n / this)))
    double step_max_      = 0.05;
    double forget_        = 0.97;   // RLS forgetting per trial
    double distress_max_  = 0.5;
    int    min_swing_     = 40;     // swing ticks a trial needs (per pair) to count
    int    report_every_  = 0;

    // state
    std::map<int, Situation> sits_;          // -1 = global
    int    sit_now_ = -1;
    int    winner_  = -1;
    std::map<int, double> occ_;              // occupancy EMA per winner
    std::map<int, int> trial_sit_ticks_;     // measured ticks per situation in the current trial
    double trial_delta_ = 0.0;               // the dither half-width fixed at the trial's start
    std::array<double, 2> u_trial_{1.0, 1.0};   // the handle as last published
    int    axis_ = 0, sign_ = 1;    // the current trial's dither
    int    trial_tick_ = 0;
    bool   trial_bad_ = false;
    std::array<float, 4> swing_{0, 0, 0, 0};
    std::array<float, 4> contact_{0, 0, 0, 0};
    float  stall_ = 0.0f, distress_ = 0.0f;
    float  sin_pitch_ = 0.0f, sin_roll_ = 0.0f;
    bool   have_tilt_ = false;
    // per-trial accumulators
    double sw_n_[2] = {0, 0}, sw_touch_[2] = {0, 0};
    double stall_sum_ = 0, tilt_n_ = 0, p_sum_ = 0, p_sq_ = 0, r_sum_ = 0, r_sq_ = 0;
    // running spread of each outcome over trials (Welford), for the normalisation
    std::array<double, 4> o_mean_{0, 0, 0, 0}, o_m2_{0, 0, 0, 0};
    double o_n_ = 0;
    std::array<double, 4> last_o_{0, 0, 0, 0};
    double last_obj_ = 0;
    int    last_trial_sit_ = -1;
    int    trials_valid_ = 0, trials_discarded_ = 0, steps_ = 0;   // totals over situations
    int64_t ticks_ = 0;
};

} // namespace ogma
