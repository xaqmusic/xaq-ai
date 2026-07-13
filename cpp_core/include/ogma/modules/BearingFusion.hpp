#pragma once

// =============================================================================
// BearingFusion.hpp  --  trust-weighted vision+scent bearing fusion (2026-06-23)
// =============================================================================
//
// The decode adapter for the sensor-fusion proof-of-concept. Sits between the
// LateralVoter (which fuses the per-modality EPMs into a consensus and exposes
// per-modality TRUST = 1/(tle+ε), informativeness-gated) and the action layer
// (HeadingController follows a bearing).  It reads the voter's trust weights and
// blends the two ANALYTIC bearings by trust:
//
//     fused = (w_scent·b_scent + w_vision·b_vision) / (w_scent + w_vision)
//
// so whichever modality the voter currently finds informative dominates the
// heading.  In the L-bend maze: vision is occluded behind the wall (its EPM bakes
// a degenerate node → trust→0) so scent's around-corner diffusion bearing carries;
// once food enters line-of-sight, vision's sharp bearing earns trust and locks the
// final approach.  This is precision-weighting (active inference) done by the
// architecture's own consensus layer — the "vision and scent work together" claim.
//
//     consensus.0 (trust_weights) ┐
//     percept.scent_compass       ┼→ BearingFusion → percept.fused_bearing → HeadingController
//     percept.visual_bearing      ┘
//
// CONFIDENCE FLOOR: a bearing whose OWN magnitude is below confidence_floor (e.g.
// an occluded/lesioned modality emitting [0,0]) is down-weighted to 0 regardless
// of its trust — instant fallback for the dropout (perturbation→recovery) demo,
// before the voter's baked-node informativeness gate has caught up.
//
// The fused [fx,fy] is renormalized to unit length (default) so disagreement
// between the two bearings (which shortens their weighted sum) is NOT read by the
// HeadingController as low confidence / nav-off.
//
// trust_weights are keyed by the EPM's FULL output topic (reality.<group>.<name>);
// scent_trust_key / vision_trust_key MUST match the epm_scent / epm_vision
// modality_group.modality_name or the channel silently gets weight 0.

#include "ogma/Module.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class BearingFusion : public Module {
public:
    BearingFusion();
    ~BearingFusion() override;

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

    // White-box accessors for tests / metrics.
    float last_fx()       const { return fx_; }
    float last_fy()       const { return fy_; }
    float last_w_scent()  const { return w_scent_; }   // applied weight (post confidence-gate)
    float last_w_vision() const { return w_vision_; }

private:
    void handle_consensus(MessagePtr payload);
    void handle_scent(MessagePtr payload);
    void handle_vision(MessagePtr payload);
    float lookup_trust(std::string const& key) const;

    std::string consensus_topic_  = "consensus.0";
    std::string scent_topic_      = "percept.scent_compass";
    std::string vision_topic_     = "percept.visual_bearing";
    std::string output_topic_     = "percept.fused_bearing";
    std::string scent_trust_key_  = "reality.nav.scent";   // = epm_scent  group.name
    std::string vision_trust_key_ = "reality.nav.vision";  // = epm_vision group.name
    int   cx_index_ = 0;
    int   cy_index_ = 1;
    float confidence_floor_ = 1e-4f;   // own-|bearing| below this → weight 0 (dropout fallback)
    bool  passthrough_prox_ = true;    // carry a 3rd (proximity) value through when present
    bool  renormalize_      = true;    // unit-normalize the fused bearing

    // Cached scent bearing.
    float s_cx_ = 0.0f, s_cy_ = 0.0f, s_prox_ = 0.0f;
    bool  s_has_prox_ = false;
    // Cached vision bearing.
    float v_cx_ = 0.0f, v_cy_ = 0.0f, v_prox_ = 0.0f;
    bool  v_has_prox_ = false;
    // Cached trust weights from the latest consensus token.
    std::unordered_map<std::string, float> trust_;

    // Outputs.
    float fx_ = 0.0f, fy_ = 0.0f, w_scent_ = 0.0f, w_vision_ = 0.0f;
};

} // namespace ogma
