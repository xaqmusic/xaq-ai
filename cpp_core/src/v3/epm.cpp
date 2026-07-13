/**
 * EPM implementation — Episodic Predictive Module (v3 C++)
 */

#include "v3/epm.hpp"
#include <chrono>
#include <cmath>

namespace ami_ogma {
namespace v3 {

// ---------------------------------------------------------------------------
// Internal helper: translate EPMConfig → GNG::Config
// ---------------------------------------------------------------------------

static GNG::Config make_gng_config(const EPMConfig& epm) {
    GNG::Config gc;
    gc.dim                    = epm.projection_dim;
    gc.baking_threshold       = epm.baking_threshold;
    gc.min_insertion_error    = epm.min_insertion_error;
    gc.lambda_new             = epm.lambda_new;
    gc.max_age                = epm.max_age;
    gc.epsilon_b              = epm.epsilon_b;
    gc.epsilon_n              = epm.epsilon_n;
    gc.stale_prune_enabled    = epm.stale_prune_enabled;
    gc.stale_window_factor    = epm.stale_window_factor;
    gc.mitosis_enabled        = epm.mitosis_enabled;
    gc.mitosis_error_threshold= epm.mitosis_error_threshold;
    gc.mitosis_check_interval = epm.mitosis_check_interval;
    gc.mitosis_split_distance = epm.mitosis_split_distance;
    // Biological health model
    gc.health_boost            = epm.health_boost;
    gc.health_base_decay       = epm.health_base_decay;
    gc.health_resilience_k     = epm.health_resilience_k;
    gc.health_death_threshold  = epm.health_death_threshold;
    gc.max_deaths_per_tick     = epm.max_deaths_per_tick;
    gc.health_death_min_nodes  = epm.health_death_min_nodes;
    gc.death_cooldown_steps    = epm.death_cooldown_steps;
    gc.near_baked_fraction     = epm.near_baked_fraction;
    return gc;
}

// ---------------------------------------------------------------------------
// Constructor — build encoder and GNG from config
// ---------------------------------------------------------------------------

EPM::EPM(const Config& cfg)
    : cfg_(cfg)
    , gng_(make_gng_config(cfg_.epm))
{
    const std::string& mod = cfg_.epm.modality;

    if (mod == "audio") {
        // Generic STFT audio slot (IP-free); the bio-mimetic encoder is AMI-Awen-only.
        FrozenSTFTEncoder::Config scfg;
        scfg.n_filters   = cfg_.epm.projection_dim;
        scfg.sample_rate = cfg_.epm.sample_rate;
        scfg.f_min       = cfg_.epm.f_min;
        scfg.f_max       = cfg_.epm.f_max;
        enc_audio_ = std::make_unique<FrozenSTFTEncoder>(scfg);
    } else if (mod == "proprioceptive") {
        // JL projection — dim-agnostic; RBF's center grid collapsed in 21D.
        enc_jl_ = FrozenJLEncoder::make_state_encoder(
            mod, cfg_.epm.projection_dim, cfg_.epm.proprio_state_dims);
    } else {
        enc_jl_ = std::make_unique<FrozenJLEncoder>(mod, cfg_.epm.projection_dim,
                                                     cfg_.epm.encoder_res,
                                                     cfg_.epm.inject_centroid,
                                                     cfg_.epm.centroid_gain);
    }
}

// ---------------------------------------------------------------------------
// Reset — rebuild GNG, keep encoder frozen matrix
// ---------------------------------------------------------------------------

void EPM::reset() {
    gng_ = GNG(make_gng_config(cfg_.epm));
    prev_prototype_.reset();
    ema_tle_ = 0.1f;
}

// ---------------------------------------------------------------------------
// Core token builder — shared by audio and video paths
// ---------------------------------------------------------------------------

RealityToken EPM::make_token(const Eigen::VectorXf& latent) {
    // GNG step → winner ID and quantization error
    auto [winner_id, qe] = gng_.step(latent);

    // Transition surprise: distance between current and previous winner prototypes
    float ts = 0.0f;
    auto proto_opt = gng_.get_prototype(winner_id);
    if (proto_opt.has_value() && prev_prototype_.has_value()) {
        ts = (proto_opt.value() - prev_prototype_.value()).norm();
    }
    if (proto_opt.has_value()) {
        prev_prototype_ = proto_opt.value();
    }

    // Combined TLE
    float tle = qe + cfg_.transition_weight * ts;

    // Adaptive novelty threshold (EMA of TLE, scaled)
    ema_tle_ = cfg_.alpha_tle * tle + (1.0f - cfg_.alpha_tle) * ema_tle_;
    float threshold = std::max(cfg_.novelty_floor,
                               ema_tle_ * cfg_.threshold_multiplier);
    bool is_novel = (tle > threshold);

    // Baking event: winner just crossed the baking gate this tick
    bool just_baked = gng_.last_step_baked();

    // Mitosis Gatekeeper: attempt split on the winner if it is saturated
    bool just_mitosis = gng_.maybe_mitosis(winner_id, latent);

    // Timestamp
    auto now = std::chrono::system_clock::now();
    int64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        now.time_since_epoch()).count();

    RealityToken tok;
    tok.timestamp_us    = ts_us;
    tok.winner_id       = winner_id;
    tok.quant_error     = qe;
    tok.transition_surp = ts;
    tok.tle             = tle;
    tok.threshold       = threshold;
    tok.is_novel        = is_novel;
    tok.just_baked      = just_baked;
    tok.just_mitosis    = just_mitosis;
    tok.pruned_ids      = gng_.last_pruned_ids();
    tok.just_pruned     = !tok.pruned_ids.empty();
    tok.node_count      = gng_.node_count();
    tok.baked_count     = gng_.baked_count();
    tok.mitosis_count   = gng_.mitosis_count();
    tok.latent          = latent;
    return tok;
}

// ---------------------------------------------------------------------------
// Audio processing
// ---------------------------------------------------------------------------

RealityToken EPM::process_audio(const float* samples, int n, int channels) {
    if (!enc_audio_) return RealityToken{};

    std::vector<float> buf(samples, samples + n * channels);
    Eigen::VectorXf latent = enc_audio_->encode(buf, channels);
    return make_token(latent);
}

// ---------------------------------------------------------------------------
// Video processing
// ---------------------------------------------------------------------------

RealityToken EPM::process_video(const uint8_t* pixels, int h, int w, int c) {
    if (!enc_jl_) return RealityToken{};

    Eigen::VectorXf latent = enc_jl_->encode(pixels, h, w, c);
    return make_token(latent);
}

// ---------------------------------------------------------------------------
// Proprioceptive state processing
// ---------------------------------------------------------------------------

RealityToken EPM::process_state(const float* state, int n_dims) {
    if (!enc_jl_) return RealityToken{};

    Eigen::VectorXf latent = enc_jl_->encode_state(state, n_dims);
    return make_token(latent);
}

} // namespace v3
} // namespace ami_ogma
