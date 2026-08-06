#pragma once

// =============================================================================
// EPM.hpp  --  Module 2 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/EPM.md
// v3 references:   cpp_core/include/v3/{encoder_jl,encoder_stft,encoder_rbf,gng}.hpp
//
// One EPM = one frozen encoder + one GNG + dual-TLE bookkeeping.  Wraps the
// v3 encoders and GNG (which are drop-in for v4) under the v4 Bus/Module
// contract.  Adds:
//   - Bus subscription for the input topic (encoder-kind-specific payload)
//   - Direct subscription to neuro.state for current-tick scaling factors
//   - Feedback subscription to prediction.<modality> (off when no
//     DescendingPredictor is wired in)
//   - history_trace bookkeeping (rolling N most-recent winners)
//   - RealityToken publishing on `<output_topic>`
//
// One EPM instance per modality declared in the graph config.  Level-N
// EPMs use `encoder_kind = identity` and read consensus.<n-1> directly.

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include "v3/encoder_jl.hpp"
#include "v3/encoder_stft.hpp"
#include "v3/encoder_rbf.hpp"
#include "v3/gng.hpp"

namespace ogma {

class EPM : public Module {
public:
    EPM();
    ~EPM() override;

    // Module interface
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // Read-only accessors for white-box tests.
    int   node_count()      const { return gng_ ? gng_->node_count() : 0; }
    int   baked_count()     const { return gng_ ? gng_->baked_count() : 0; }
    int   mitosis_count()   const { return gng_ ? gng_->mitosis_count() : 0; }
    float ema_tle()         const { return ema_tle_; }
    float novelty_thresh()  const { return novelty_threshold_now_; }
    // v5.4.L Diagnostic B — winner-id histogram for GNG saturation check.
    std::unordered_map<int, int> const& winner_counts() const { return winner_counts_; }

    enum class EncoderKind { JL, STFT, RBF, Identity };

private:

    // Subscription handlers
    void handle_input(std::string_view topic, MessagePtr payload);
    void handle_neuro(std::string_view topic, MessagePtr payload);
    void handle_prediction(std::string_view topic, MessagePtr payload);

    // Internal helpers
    bool encode_pending_input(Eigen::VectorXf& out);
    void apply_neuro_scaling();
    void compute_dual_tle(float quant_error,
                          int   winner_id,
                          float& transition_surp_out,
                          float& tle_out);
    void publish_token(uint64_t tick_id,
                       int      winner_id,
                       float    quant_error,
                       float    transition_surp,
                       float    tle,
                       Eigen::VectorXf const& latent);
    void publish_bootstrap_token(uint64_t tick_id);

    // Configuration
    EncoderKind encoder_kind_           = EncoderKind::Identity;
    std::string modality_group_;        // "video" | "audio" | "proprio" | "consensus"
    std::string modality_name_;         // "retinal" | "audio" | "imu" | "0"
    std::string input_topic_;
    std::string output_topic_;
    int         projection_dim_         = 128;

    // TLE blend parameters (v4 contract): tle = α·QE + β·TS
    float tle_alpha_                    = 0.7f;
    float tle_beta_                     = 0.3f;
    float tle_ema_alpha_                = 0.05f;
    float novelty_threshold_multiplier_ = 1.5f;
    float novelty_floor_                = 0.01f;
    int   history_trace_size_           = 5;

    // Phase 6.6.E: forward-rollout configuration.  When > 0, the EPM
    // attaches a predicted_pathway of this length to every published token
    // by greedy-argmax-walking transition_counts_ from the current winner.
    // Default 0 = off; existing behavior unchanged.
    int   predicted_pathway_steps_      = 0;

    // Phase v5.2 — sub-rate processing.  When > 1, the EPM skips
    // encoder + GNG step on (process_every_n_ticks_ - 1) of every
    // process_every_n_ticks_ ticks, republishing its last token with
    // the current tick_id on skipped ticks so downstream voters see a
    // continuous contribution.  Default 1 = process every tick
    // (legacy behaviour bit-identical).  Combined with a
    // KeyframeAverager upstream, lets a slow-EPM see averaged sensor
    // windows at sub-Hz rates without losing consensus continuity.
    int   process_every_n_ticks_        = 1;
    int   sub_rate_tick_count_          = 0;
    std::shared_ptr<RealityToken> last_published_token_;

    bool subtract_descending_prediction_ = true;
    uint64_t master_seed_               = 0;

    // Encoders (exactly one populated based on encoder_kind_).
    std::unique_ptr<ami_ogma::v3::FrozenJLEncoder>   enc_jl_;
    std::unique_ptr<ami_ogma::v3::FrozenSTFTEncoder> enc_audio_;
    std::unique_ptr<ami_ogma::v3::FrozenRBFEncoder>  enc_rbf_;

    // The GNG.  Always present.
    std::unique_ptr<ami_ogma::v3::GNG> gng_;

    // Mirrors GNG::Config::insertion_autotune so apply_neuro_scaling() knows
    // which of its two scale-application paths is live (see its comment).
    bool            insertion_autotune_  = false;

    // -------------------------------------------------------------------------
    // Commissioning window — per-dim input autocalibration (dim_autocal_ticks)
    // -------------------------------------------------------------------------
    //
    // §0 rule 2 requires every EPM's input to be conditioned before the GNG
    // discretises it: a channel whose scale is small relative to its siblings
    // is collapsed by the insertion gate WHILE the encoder still shows the
    // structure.  Doing that by hand means measuring each sensor and writing
    // dim_min/dim_max into config — which is a constant tuned to a signal's
    // scale, i.e. exactly the smell that names a missing adaptive mechanism.
    //
    // This is that mechanism.  For the first `dim_autocal_ticks` input frames
    // the EPM runs NORMALLY (warm start) while accumulating per-dim statistics;
    // it then derives ranges, installs them, and RESETS the GNG topology so the
    // vocabulary is re-earned in the calibrated space.  The reset is mandatory:
    // every prototype learned during commissioning is expressed in provisional
    // units and is meaningless afterwards.
    //
    // Range per dim = intersect(mu +/- k*sigma, [min_obs, max_obs]).  Pure
    // min/max is outlier-driven (a single first-tick transient can set a range
    // an order of magnitude too wide); pure mu +/- k*sigma can invent range a
    // bounded channel never occupies.  The intersection takes the tighter and
    // can never exceed what was actually observed.
    //
    // 0 (default) = off: nothing accumulated, no reset, byte-identical.
    int             dim_autocal_ticks_   = 0;
    double          dim_autocal_k_       = 4.0;
    bool            dim_autocal_done_    = false;
    uint64_t        dim_autocal_seen_    = 0;
    std::vector<double> dac_min_, dac_max_, dac_sum_, dac_sumsq_;
    void            dim_autocal_observe(const float* v, int n);
    void            dim_autocal_finalise();

    // Working state
    Eigen::VectorXf prev_winner_prototype_;
    bool            has_prev_prototype_ = false;
    float           ema_tle_            = 0.1f;
    float           novelty_threshold_now_ = 0.0f;
    // Per-tick raw TLE + quant_error from the most recent step.  Distinct
    // from ema_tle_ which smooths over many ticks.  Surfaced so inspector
    // widgets can plot the responsive raw curve next to the EMA trend.
    // (W2 EPM dashboard / v3 visualizer.py parity.)
    float           last_tle_           = 0.0f;
    float           last_quant_error_   = 0.0f;
    std::deque<int> history_trace_;

    // Phase 6.6.E: per-node successor counts.  Updated each tick from the
    // (prev_winner, curr_winner) pair; used to roll forward greedy-argmax
    // for predicted_pathway emission.  Cleaned up when GNG prunes a node.
    int prev_winner_id_for_transitions_ = -1;
    std::unordered_map<int, std::unordered_map<int, int>> transition_counts_;

    // v5.4.L Diagnostic B — per-winner-id histogram across all ticks.
    // Identifies premature GNG saturation: if 1-2 winner_ids account
    // for >90% of ticks, the encoder is severely under-discriminating
    // and the published latent is wildcard-class regardless of input
    // signal.  Pruned when GNG erases a node.
    std::unordered_map<int, int> winner_counts_;

    // Pending input (latest payload received this tick).
    std::shared_ptr<const RawImageFrame> pending_image_;
    std::shared_ptr<const RawAudioFrame> pending_audio_;
    std::shared_ptr<const ProprioToken>  pending_proprio_;
    std::shared_ptr<const RealityToken>  pending_reality_;       // for identity from EPM
    std::shared_ptr<const ConsensusToken> pending_consensus_;    // for identity from voter

    // Latest neuro.state scaling factors (defaults until first publish).
    float epsilon_b_scale_           = 1.0f;
    float min_insertion_error_scale_ = 1.0f;
    float mitosis_threshold_scale_   = 1.0f;
    float novelty_threshold_scale_   = 1.0f;

    // Latest descending prediction (for subtraction).  Held until consumed.
    std::shared_ptr<const PredictionToken> pending_prediction_;

    // Frozen baseline values from params; neuro.state scales modulate them.
    float base_epsilon_b_              = 0.05f;
    float base_min_insertion_error_    = 0.02f;
    float base_mitosis_error_threshold_ = 0.30f;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).

public:
    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
