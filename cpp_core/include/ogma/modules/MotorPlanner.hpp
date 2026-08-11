#pragma once
// MotorPlanner — PART III's rolling action-prediction buffer, v1: SHADOW OBSERVER.
//
// The operator's architecture (2026-08-10): reflexes write the t0 row (the present);
// the EPM token stream writes the tN rows (the predicted future); the object of study
// is THE PROBABILITY CONE this produces — per-depth distributions over future body
// tokens, phase-conditioned (the M0 diagnosis: on a rhythmic body, transitions are
// phase-dependent and an unconditioned chain degenerates to persistence).
//
// v1 has ZERO AUTHORITY: no outputs on the bus.  It learns a phase-conditioned
// transition model online from the observed token stream, propagates the cone each
// tick, and VERIFIES itself — storing the cone's predictions at probe depths and
// scoring them against reality when the future arrives.  The masking interface
// (mask_mode) sharpens the cone before propagation; masking experiments measure
// cone quality deltas, never behavior (yet).
//
// Doctrine anchors: instrument-first (P5/M0 pattern); "feed it phase" (§0 rule 3),
// applied to the VOCABULARY's dynamics for the first time; the plan-as-prediction
// framing (PART III non-negotiables) — this module is the future-buffer half only.

#include "ogma/Module.hpp"
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ogma {

class MotorPlanner : public Module {
public:
    std::string_view type_name() const override { return "MotorPlanner"; }

    std::vector<TopicSpec> input_topics() const override;
    std::vector<TopicSpec> output_topics() const override { return {}; }   // shadow: none
    ParamSchema params_schema() const override;
    ParamMap current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void restore_state(nlohmann::json const& s) override;
    nlohmann::json diag_snapshot() const override;

private:
    static constexpr int   kMaxBins   = 16;
    static constexpr int   kNumProbes = 7;
    static constexpr std::array<int, kNumProbes> kProbes{1, 3, 5, 8, 13, 21, 34};

    using Dist = std::unordered_map<int, float>;         // token -> probability

    int   phase_bin(float phi) const;
    Dist  propagate(Dist const& d, int bin) const;
    void  apply_mask(Dist& d, int bin) const;

    // --- wiring / params
    Bus*        bus_ = nullptr;
    std::string state_topic_  = "reality.bodypose.pose";
    std::string rhythm_topic_ = "rhythm.body.gait";
    double horizon_    = 40.0;    // cone depth in ticks (>= largest probe)
    double beam_k_     = 12.0;    // per-row sparsity cap
    double phase_bins_ = 8.0;
    double mask_mode_  = 0.0;     // 0 none · 1 phase-affinity mask
    double mask_floor_ = 0.02;    // affinity below this ⇒ masked (mode 1)
    std::vector<uint64_t> subs_;

    // --- live state
    int    cur_tok_  = -1;
    float  phi_      = 0.0f;      // body phase [0,2π)
    float  omega_    = 0.105f;    // rad/tick (≈60-tick stride prior; overwritten by rhythm)
    bool   phi_seen_ = false;

    // --- learned model: P(next | token, phase_bin), per-tick counts
    // key = token * kMaxBins + bin
    std::unordered_map<int, std::unordered_map<int, float>> trans_;
    std::unordered_map<int, std::array<float, kMaxBins>>    tok_phase_;   // occupancy per bin
    long   n_obs_ = 0;

    // --- the cone verification ring: predictions made for tick t, keyed by t
    struct Pending { uint64_t due; int depth; Dist dist; };
    std::vector<Pending> pending_;

    // --- per-depth accumulators (index = probe slot)
    std::array<double, kNumProbes> acc_top1_{}, acc_mass_{}, acc_ent_{}, acc_topk_{};
    std::array<long,   kNumProbes> acc_n_{};
    double marg_top1_ = 0.0; long marg_n_ = 0;    // marginal baseline at depth 1
    std::unordered_map<int, float> marginal_;
    long   masked_out_ = 0;                        // mask activity (consumer check)
};

} // namespace ogma
