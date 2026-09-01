#pragma once

// =============================================================================
// JointSensorimotorBridge.hpp  --  Phase 7.2-EPM sensorimotor pairing
// =============================================================================
//
// Joins N per-joint ActionOut streams with the matching N indices of a single
// bundled ProprioToken (typically `reality.proprio.joints`) and republishes
// per-joint ProprioTokens on `reality.joint_<leg>.<servo>` (or whatever
// output_topics the config specifies).  Each output value is the 3-D vector
//
//      [ norm_position, last_action_scalar, position_delta_since_prev_tick ]
//
// This is the sensorimotor input shape Per-joint EPMs (Phase 7.2-EPM,
// docs/phase7_chunk_plan.md addendum) consume.  Joining at the substrate
// boundary — not in the env body — keeps env-vs-substrate clean and makes
// the same bridge work on a future PiCrawler HAL.
//
// Module lifecycle authoring contract: per docs/primitives/_module_lifecycle.md.

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class JointSensorimotorBridge : public Module {
public:
    JointSensorimotorBridge();
    ~JointSensorimotorBridge() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    int n_joints()         const { return int(action_topics_.size()); }
    int total_publishes()  const { return total_publishes_; }
    int total_proprio_in() const { return total_proprio_in_; }
    int total_action_in()  const { return total_action_in_; }

private:
    void handle_proprio(MessagePtr payload);
    void handle_action(int joint_idx, MessagePtr payload);

    // Configuration
    std::string              proprio_input_topic_ = "reality.proprio.joints";
    std::vector<std::string> action_topics_;
    std::vector<std::string> output_topics_;
    std::vector<int>         proprio_indices_;   // index into ProprioToken.values
    std::string              sensor_label_prefix_ = "joint";  // ProprioToken.sensor
    // Phase 7.2-EPM per-leg mode: when > 1, the bridge groups every
    // `group_size` consecutive action_topics / proprio_indices into ONE
    // output ProprioToken of length 3 * group_size (concatenated
    // [pos, action, delta] triples).  output_topics length must equal
    // action_topics.size() / group_size.  Default 1 preserves per-joint
    // output behaviour bit-identically.
    int                      group_size_         = 1;
    // 2026-08-07 — OPTIONAL PER-LEG LOAD CHANNEL appended to each output vector.
    //
    // MotorEPMv2 sizes its forward model from the arriving vector (L.n = values.size();
    // A is n x m, C is m x n), and its guards are `>= 3*m` rather than `==`, so an extra
    // trailing element is LEARNED AUTOMATICALLY while every existing index is unchanged
    // ([pos,act,delta] per joint, joint j position at 3j).  That is what makes this a
    // one-element change rather than a refactor.
    //
    // ⚠ THE WIDTH MUST BE CONSTANT FROM THE FIRST FRAME.  MotorEPMv2 rejects any frame
    // whose dimensionality differs from the one it initialised on ("dimensionality must
    // be stable"), so if the bridge emitted 9 dims before the load topic arrived and 10
    // after, every leg would latch at 9 and silently drop every later frame.  Therefore
    // the width is decided by whether load_topic is CONFIGURED, not by whether a load
    // value has been received yet; the slot carries 0.0 until the first one lands.
    std::string              load_topic_;                 // empty = off, byte-identical
    int                      load_slots_         = 1;     // trailing elements per group (1 = historical)
    std::vector<float>       last_load_;                  // group-major: [o*load_slots + s]
    bool                     have_load_          = false;
    // 2026-08-02 · IMPORT I4b — POSITION-CHANNEL-ONLY colored sensor noise.
    // PM wires every legged controller through ColorUniformNoise(0.1) on every sensor.
    // Injecting that at the BODY (picrawler_body.gd sensor_noise_sigma) lands on the raw
    // joint angles, and because `delta` here is a DIFFERENCE of successive positions the
    // same sigma then re-enters the velocity channel — the channel the HK gradient
    // weights most heavily (44% of |C| mass).  Measured effect: transport falls
    // monotonically with sigma while posture peaks at sigma=0.03, i.e. a
    // stochastic-resonance curve whose optimum sits below PM's nominal value.
    // Injecting HERE instead perturbs `pos` only and leaves `delta` computed from the
    // clean positions, so the dose reaches one channel rather than two.
    // sigma = 0 (default) is byte-identical.
    double                   pos_noise_sigma_    = 0.0;
    // I4c — the COMPLEMENT: noise on the VELOCITY (delta) channel only, position clean.
    // Measured 2026-08-02: both-channel body-side noise at sigma 0.03 makes the body
    // progressively STAND UP (chassis 0.045 -> 0.071 over 40k) while position-only
    // reproduces it at no sigma.  So the active ingredient is the velocity component --
    // which is also the channel the HK gradient weights most (44% of |C| mass).  This
    // isolates it, to CONFIRM the mechanism rather than infer it from the complement.
    double                   vel_noise_sigma_    = 0.0;
    double                   pos_noise_tau_      = 8.0;   // correlation length in ticks; 1 = white
    uint64_t                 pos_noise_seed_     = 0;
    std::vector<float>       pos_noise_;                  // per-joint colored state
    std::vector<float>       vel_noise_;                  // per-joint colored state (velocity channel)
    std::mt19937             pos_noise_rng_;

    // 2026-08-06 — PER-CHANNEL RANGE PROBE (instrument, not a lever).
    //
    // An RBF EPM downstream normalises each input dim over a CONSTANT range
    // (EPM `dim_min`/`dim_max`, default [-1,1]).  The three channels of this
    // bridge's output do NOT share a scale: `pos` and `action` are ~[-1,1],
    // but `delta` is a per-tick difference an order of magnitude smaller, so
    // the default range crushes the velocity channels into a few percent of
    // [0,1] and the GNG's insertion gate never sees them.  That is CLAUDE.md
    // §0 rule 2 — the documented way EPM use goes wrong — and the only honest
    // way to set those constants is to MEASURE the channels first.
    //
    // range_probe_ticks = 0 (default) accumulates nothing and prints nothing,
    // so the off-path is byte-identical.
    int                      range_probe_ticks_  = 0;
    std::vector<float>       rp_min_, rp_max_;   // per output, per dim
    std::vector<double>      rp_sum_, rp_sumsq_;
    uint64_t                 rp_count_           = 0;
    void                     range_probe_accum(int out_idx, int dim, float v);
    void                     range_probe_report(uint64_t tick_id) const;

    // Working state — sized to n_joints() at setup.
    std::vector<float> last_position_;       // latest per-joint position seen
    std::vector<float> prev_position_;       // last tick's position (for delta)
    std::vector<float> last_action_;         // latest per-joint action seen
    bool               have_proprio_      = false;

    int total_publishes_   = 0;
    int total_proprio_in_  = 0;
    int total_action_in_   = 0;
};

} // namespace ogma
