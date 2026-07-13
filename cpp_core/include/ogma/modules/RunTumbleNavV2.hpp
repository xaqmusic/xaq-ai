#pragma once

// =============================================================================
// RunTumbleNavV2.hpp -- honest scalar chemotaxis, promoted kinesis -> reactive taxis
// =============================================================================
//
// Clean-room successor to RunTumbleNav (which is kept, untouched, for backward
// compatibility). Same contract: navigate from a SCALAR scent concentration only
// (no direction sensor), climbing by temporal action-consequence. The 2026-07-08
// isolation audit showed the V1 SHIPPED config (orthokinesis on, run-commit off)
// LOSES to a gradient-blind random walk (6 vs 24 eats/120s @24m) because the tumble
// machinery strangles the runs. V2 removes the reactive crank and assembles the
// doctrine-mandated ladder as the DEFAULT, held to one bar: it must BEAT a random
// walk across the env-battery.
//
// The four mechanisms, all always-on (each doctrine-anchored, no static tuning):
//   * methylation prediction/error + KF4 noise floor  -- baseline_=EMA(scent) is a
//       PREDICTION, error=scent-baseline its error (§1); error_n normalised by a
//       running |error| scale FLOORED by the sensor noise measured while stationary
//       (true gradient contribution=0 there) so flat/saddle fields don't amplify
//       noise into phantom gradient (§6, dynamics-derived floor).
//   * KF1 run integrity -- a tumble decision belongs to a RUN; while reorienting to
//       face run_dir, suppress the draw + the stuck counter. The run "clock" is the
//       executed reorientation (own dynamics), not a fixed count. Fixes runs dying
//       in their own turn transient.
//   * KF2 efference-matched stuck -- blocked = INTENDING forward (executing a run)
//       but achieving << the body's own capable speed (vel vs a learned vel_scale),
//       not a static velocity threshold. No mid-turn false tumbles; no magic m/s.
//   * KF6 directional belief (the taxis) -- infer the hidden up-gradient DIRECTION
//       from the agent's OWN run outcomes (never read from a percept, §2.1): circular
//       accumulator g=EMA(outcome*unit(run_dir)); R=|g|/EMA(|outcome|) is the belief
//       precision, SELF-SCALED (§2.3/§6); mu=atan2(g). Tumble centre blends from the
//       current heading toward mu by R (R->0 = uniform floor byte-identical to a
//       kinesis); decay-on-loss collapses R when the direction stops paying off ->
//       free perturbation recovery. Reactive one-run lookahead, not prospective EFE.
//
// Ablations for validation (single `ablation` enum, default "none" = full taxis):
//   "shuffle"     -> gradient-blind tumble at the flat base rate = the RANDOM-WALK floor
//   "kinesis"     -> directional belief off (R clamped 0) = pure run-and-tumble kinesis
//   "wrong_sign"  -> belief update sign flipped (bias toward BAD directions; must regress)
//   "shuffle_dir" -> randomise run_dir in the belief update (breaks direction<->outcome; -> kinesis)
//
//   reality.proprio.scent_max (SCALAR) + heading + vel_ego  -> RunTumbleNavV2
//        -> percept.klino_heading ([vx,vy] -> HeadingController) + percept.klino_confidence

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class RunTumbleNavV2 : public Module {
public:
    RunTumbleNavV2() = default;
    ~RunTumbleNavV2() override = default;

    // ablation mode (validation controls; "none" = full taxis)
    enum class Ablation { None, Shuffle, Kinesis, WrongSign, ShuffleDir };

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
    int   last_action()    const { return committed_action_; }   // 0 run / 1 tumble
    float last_p_tumble()  const { return last_p_tumble_; }
    float last_error()     const { return last_error_; }         // normalised prediction error
    float baseline()       const { return baseline_; }           // methylation level
    float error_scale()    const { return error_scale_; }
    float noise_floor()    const { return noise_floor_; }        // KF4 stationary-noise floor
    float vel_scale()      const { return vel_scale_; }          // KF2/KF4 learned forward-speed scale
    int   tumble_count()   const { return tumble_count_; }
    int   run_count()      const { return run_count_; }
    int   forced_tumbles() const { return forced_tumbles_; }
    int   forced_in_turn() const { return forced_in_turn_; }     // K2 livelock signature (should stay ~0)
    float capability()     const { return capability_; }
    float eat_scent()      const { return eat_scent_; }
    bool  have_eat_scent() const { return have_eat_scent_; }
    float run_len_up()     const { return run_len_up_; }         // KF0: EMA run length, scent-rising runs
    float run_len_down()   const { return run_len_down_; }       // KF0: EMA run length, scent-falling runs
    float turn_frac()      const { return turn_frac_; }
    bool  reorienting()    const { return reorienting_; }
    float dir_consistency() const { return last_R_; }            // KF6 belief precision (kappa proxy) in [0,1]
    float dir_mu()         const { return last_mu_; }
    float run_dir()        const { return run_dir_abs_; }
    bool  muted()          const { return have_authority_ && authority_ < authority_floor_; }

private:
    void handle_scent(MessagePtr payload);
    void handle_heading(MessagePtr payload);
    void handle_vel(MessagePtr payload);
    void handle_eat(MessagePtr payload);
    void handle_authority(MessagePtr payload);

    Ablation ablation_ = Ablation::None;

    std::string scent_topic_      = "reality.proprio.scent_max";
    std::string heading_topic_    = "reality.proprio.heading";
    std::string vel_topic_        = "reality.proprio.vel_ego";
    std::string eat_topic_        = "events.eat";
    std::string output_topic_     = "percept.klino_heading";
    std::string confidence_topic_ = "percept.klino_confidence";
    std::string authority_topic_  = "";   // KF3 (composition); empty = no read

    // methylation / tumble params
    float baseline_alpha_   = 0.05f;
    float scale_alpha_      = 0.02f;
    float noise_floor_alpha_= 0.01f;   // KF4: EMA rate of the stationary-noise floor
    float tumble_base_      = 0.1f;
    float tumble_gain_      = 0.1f;
    float tumble_min_       = 0.0f;
    float tumble_max_       = 0.5f;
    float tumble_range_     = 1.5708f; // +/- pi/2 keeps the reorient forward
    // KF2 efference-matched stuck
    float stuck_frac_       = 0.2f;    // blocked while executing = |vel| < stuck_frac * vel_scale
    int   stuck_ticks_      = 10;
    // capability (arbiter-facing; policy-free)
    float peak_decay_       = 0.0005f;
    float eat_scent_alpha_  = 0.2f;
    // KF6 directional belief
    float dir_lr_           = 0.1f;
    // KF3
    float authority_floor_  = 0.5f;
    uint64_t master_seed_   = 11;

    // latest inputs
    float smax_ = 0.0f, heading_ = 0.0f, vel_fwd_ = 0.0f;
    bool  have_scent_ = false;
    float authority_ = 1.0f;
    bool  have_authority_ = false;

    // methylation state
    float baseline_    = 0.0f;
    float error_scale_ = 0.0f;
    float noise_floor_ = 0.0f;   // KF4
    float last_p_tumble_ = 0.0f;
    float last_error_    = 0.0f;

    // capability state
    float scent_peak_ = 0.0f;
    float eat_scent_  = 0.0f;
    bool  have_eat_scent_ = false;
    bool  eat_pending_ = false;
    float eat_scent_sample_ = 0.0f;
    float capability_ = 0.0f;

    // run/tumble state
    float run_dir_abs_ = 0.0f;
    bool  have_run_dir_ = false;
    int   committed_action_ = 0;
    int   tumble_count_ = 0, run_count_ = 0, forced_tumbles_ = 0;
    int   stuck_counter_ = 0;
    float vel_scale_ = 0.0f;     // KF2/KF4: learned scale of the body's own forward speed
    float out_vx_ = 0.0f, out_vy_ = 1.0f;

    // KF1 run-integrity state
    bool  reorienting_ = false;
    bool  turning_     = false;
    float turn_dir_    = 1.0f;

    // KF0 telemetry
    float run_start_scent_ = 0.0f;
    int   run_ticks_       = 0;
    float run_len_up_      = 0.0f;
    float run_len_down_    = 0.0f;
    float turn_frac_       = 0.0f;
    int   forced_in_turn_  = 0;

    // KF6 belief state
    float belief_x_ = 0.0f, belief_y_ = 0.0f, belief_absw_ = 0.0f;
    float last_R_   = 0.0f;
    float last_mu_  = 0.0f;

    std::mt19937 rng_{11};
};

}  // namespace ogma
