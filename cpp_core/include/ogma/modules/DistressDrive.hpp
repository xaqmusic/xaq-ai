#pragma once

// =============================================================================
// DistressDrive.hpp  --  Phase 6.9 Stage A: "boredom of being stuck" combiner
// =============================================================================
//
// Playful Machine principles #1 (Boredom-Driven Exploration) + #2 (Edge-of-
// Chaos): a frozen sensorimotor loop (the cell pinned against a wall) is
// perfectly predictable -> BORING -> the agent should explore harder until
// something changes.  This module produces the scalar boredom signal that
// MotorEPM consumes to escalate rotational exploration noise and fade the
// cognitive steer when the world has stopped changing.
//
// The signal is read off the SLOW META-EPM stack (the H-JEPA hierarchical
// world model), NOT the phantom motor-TLE (a linear forward model can't
// represent the rectified flagella, so motor-TLE reads non-zero even when
// frozen).  Three readouts, three timescales:
//   * meta_epm.tle above its slow baseline  -> ONSET alarm (high TLE = focus)
//   * StaleConfidenceDecay(meta_epm)         -> SUSTAINED (low TLE = boredom)
//   * inter-tick motion of the pooled state  -> MODEL-FREE freeze (works cold,
//                                               before the meta-EPM has learnt
//                                               the normal motion manifold)
// Combined, suppressed when dopamine is RISING (the agent is approaching food
// -- don't thrash, let the brain home).  Published as a ReflexGate on
// `cognition.boredom`.
//
// NOT a turn-away reflex: this module knows nothing about walls or which way
// to turn.  It only reports "the loop has frozen"; MotorEPM responds with
// UNDIRECTED rotational noise.  Direction is random -- it cannot find food
// or solve a maze, it only keeps the bug alive so the cognitive critic can
// learn.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class DistressDrive : public Module {
public:
    DistressDrive();
    ~DistressDrive() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors for tests + HUD/diag (the three components are
    // surfaced so the signal is A/B-able on its own).
    float boredom()    const { return boredom_; }
    float tle_spike()  const { return tle_spike_; }
    float staleness()  const { return staleness_; }
    float motion_inv() const { return motion_inv_; }
    float motion_raw() const { return motion_raw_; }   // pre-normalisation pooled-state motion
    float motion_ema() const { return motion_ema_; }
    float suppress()   const { return suppress_; }

    float mismatch()   const { return mismatch_; }   // reafference (efferent vs afferent)
    float no_progress() const { return no_progress_; } // sustained lack of net displacement
    float interest()    const { return interest_; }    // curiosity: scent-novelty + clearance + green
    float scent_novelty() const { return scent_novelty_; }
    float clearance()   const { return clearance_; }
    float green()       const { return green_; }        // green-food saliency in view [0,1]

private:
    void handle_meta(MessagePtr p);
    void handle_staleness(MessagePtr p);
    void handle_pool(MessagePtr p);
    void handle_neuro(MessagePtr p);
    void handle_imu(MessagePtr p);
    void handle_efference(MessagePtr p);
    void handle_scent(MessagePtr p);
    void handle_clearance(MessagePtr p);
    void handle_green(MessagePtr p);

    // Configuration ----------------------------------------------------------
    std::string meta_topic_      = "meta.distress";            // RealityToken (.tle)
    std::string staleness_topic_ = "cognition.meta_staleness"; // ReflexGate (.value)
    std::string pool_topic_      = "distress.pool";            // ProprioToken (pooled aggregate)
    std::string neuro_topic_     = "neuro.state";              // NeuroState (.dopamine)
    std::string imu_topic_       = "reality.proprio.imu";      // afferent (dims 2,3 = actual vel)
    std::string efference_topic_ = "reality.proprio.motor_efference"; // commanded vel
    std::string scent_topic_     = "reality.olfactory.scent";  // scent EPM RealityToken (.tle = novelty)
    std::string clearance_topic_ = "reality.proprio.clearance";// forward openness (ProprioToken)
    std::string green_topic_     = "reality.proprio.green_fraction"; // green-food saliency (ProprioToken)
    std::string output_topic_    = "cognition.boredom";        // ReflexGate (.value=boredom)
    std::string interest_topic_  = "cognition.interest";       // ReflexGate (.value=interest) — ③ escape direction

    double w_mismatch_      = 1.0;    // weight: reafference mismatch (catches RAM — trying but not moving)
    double min_effort_      = 0.05;   // below this commanded speed, "not trying" → no mismatch (gates rest)
    double w_progress_      = 1.0;    // weight: sustained lack of net progress (catches PARK — idling stuck)
    double progress_alpha_  = 0.01;   // EMA rate on the afferent velocity VECTOR (~window; slow = pauses OK)
    double progress_scale_  = 0.2;    // net-progress speed counted as "getting somewhere"; below → no_progress→1
    double w_tle_           = 0.3;    // weight: meta-EPM TLE onset spike (corroborating)
    double w_staleness_     = 0.3;    // weight: sustained winner-staleness (corroborating)
    double w_motion_        = 0.0;    // weight: pooled-state freeze (noisy on this body — telemetry only by default)
    double tle_ema_alpha_   = 0.02;   // slow baseline so a freeze reads as a spike above it
    double tle_spike_scale_ = 0.5;    // normalises (tle - baseline) into [0,1]
    double motion_alpha_    = 0.1;    // low-pass smoother on raw pooled-state motion
    double motion_scale_    = 0.001;  // FIXED free-swim motion reference; minv = 1 - smoothed/scale
    double suppress_gain_   = 5.0;    // rising-dopamine suppression strength
    // ③ curiosity interest = w_scent·scent_novelty + w_clear·clearance.  Runs the
    // escape toward scent-rich OR spatially-open headings; tumbles when boring.
    double w_scent_int_     = 0.5;    // weight of (short-term) scent novelty
    double w_clear_int_     = 0.5;    // weight of forward clearance
    double w_green_int_     = 0.5;    // weight of green-food saliency (bee→flower sensory prior; drives curiosity toward food)
    double scent_nov_scale_ = 0.03;   // scent-TLE-above-baseline that counts as "novel"
    double scent_nov_alpha_ = 0.01;   // slow baseline EMA → short-term novelty (vs the stuck state)

    // Working state ----------------------------------------------------------
    float  meta_tle_      = 0.0f;
    float  tle_ema_       = 0.0f;
    bool   tle_ema_init_  = false;
    float  staleness_     = 0.0f;

    std::vector<float> pool_cur_;
    std::vector<float> pool_prev_;
    bool   pool_seen_     = false;   // a fresh pool frame arrived this tick
    bool   pool_prev_init_ = false;
    float  motion_ema_    = 0.0f;
    bool   motion_ema_init_ = false;
    float  motion_inv_    = 0.0f;
    float  motion_raw_    = 0.0f;

    float  da_prev_       = 0.0f;
    bool   da_init_       = false;
    float  suppress_      = 0.0f;

    float  tle_spike_     = 0.0f;
    float  boredom_       = 0.0f;

    float  afferent_speed_ = 0.0f;   // |actual world velocity| (normalised)
    float  afferent_vx_    = 0.0f;   // actual velocity vector components (normalised)
    float  afferent_vz_    = 0.0f;
    float  efferent_speed_ = 0.0f;   // |commanded velocity| (normalised)
    float  mismatch_       = 0.0f;   // fraction of intended motion not realised
    float  vel_ema_x_      = 0.0f;   // slow EMA of the afferent velocity VECTOR
    float  vel_ema_z_      = 0.0f;
    bool   vel_ema_init_   = false;
    float  no_progress_    = 0.0f;   // 1 - |EMA(vel)|/scale : sustained no net displacement

    float  scent_tle_      = 0.0f;   // scent-EPM surprise (novelty)
    float  scent_tle_ema_  = 0.0f;   // slow baseline → short-term novelty
    bool   scent_ema_init_ = false;
    float  scent_novelty_  = 0.0f;   // clamp01((scent_tle - baseline)/scale)
    float  clearance_      = 0.0f;   // forward openness [0,1]
    float  green_          = 0.0f;   // green-food saliency in view [0,1]
    float  interest_       = 0.0f;   // curiosity signal driving the escape direction
};

} // namespace ogma
