#pragma once

/**
 * EPM — Episodic Predictive Module (v3 C++)
 *
 * Wraps FrozenJLEncoder (or FrozenSTFTEncoder) + GNG into a single
 * processing unit that emits a RealityToken on every tick.
 *
 * Dual TLE (Time-Loop Error) — matching src/ami_ogma_v3/epm.py:
 *
 *   Component 1 — Quantization Error (QE):
 *     The Euclidean distance from the encoder output to the winning GNG
 *     prototype.  High QE means the current input doesn't fit existing
 *     concepts well → novelty / surprise.
 *
 *   Component 2 — Transition Surprise (TS):
 *     The Euclidean distance between the CURRENT and PREVIOUS winner
 *     prototypes.  High TS means the system "jumped" to a very different
 *     concept → boundary / discontinuity signal.
 *
 *   Combined TLE:
 *     tle = qe + transition_weight * ts
 *
 * The is_novel flag is set when tle exceeds a running adaptive threshold
 * (EMA of tle, scaled by threshold_multiplier).
 *
 * Usage:
 *   EPM epm(config);
 *   // Audio:
 *   RealityToken tok = epm.process_audio(samples.data(), n, channels);
 *   // Video:
 *   RealityToken tok = epm.process_video(pixels.data(), h, w, c);
 */

#include "v3/types.hpp"
#include "v3/encoder_jl.hpp"
#include "v3/encoder_stft.hpp"
#include "v3/encoder_rbf.hpp"
#include "v3/gng.hpp"
#include <memory>
#include <optional>
#include <chrono>

namespace ami_ogma {
namespace v3 {

class EPM {
public:
    struct Config {
        EPMConfig epm;          // modality, projection_dim, GNG hyperparams, etc.

        // TLE parameters
        float transition_weight    = 1.0f;  // TS weight in combined TLE
        float threshold_multiplier = 1.5f;  // novelty threshold = ema_tle * multiplier
        float alpha_tle            = 0.05f; // EMA decay for running TLE estimate

        // Novelty floor (prevents hair-trigger on a flat baseline)
        float novelty_floor = 0.01f;
    };

    explicit EPM(const Config& cfg);

    // Process one audio chunk.
    // samples: float PCM, range [-1, 1]
    // n:       number of samples (per channel)
    // channels: 1 (mono) or 2 (stereo)
    RealityToken process_audio(const float* samples, int n, int channels = 1);

    // Process one video frame.
    // pixels: uint8 RGB (or grayscale), row-major
    RealityToken process_video(const uint8_t* pixels, int h, int w, int c);

    // Process one proprioceptive body state vector.
    // state: float array [position, velocity, acceleration, efference_error, ...]
    // n_dims: number of dimensions (must match body schema config)
    RealityToken process_state(const float* state, int n_dims);

    // Access underlying components
    GNG& gng() { return gng_; }
    const GNG& gng() const { return gng_; }

    // Runtime parameter updates (wired to UI sliders via control server)
    void set_threshold_multiplier(float m) { cfg_.threshold_multiplier = m; }
    void set_transition_weight(float w)    { cfg_.transition_weight    = w; }
    void set_stale_prune_enabled(bool e)       { gng_.set_stale_prune_enabled(e); }
    void set_stale_window_factor(float f)      { gng_.set_stale_window_factor(f); }
    void set_min_insertion_error(float e)      { gng_.set_min_insertion_error(e); }
    void set_mitosis_enabled(bool e)           { gng_.set_mitosis_enabled(e); }
    void set_mitosis_error_threshold(float t)  { gng_.set_mitosis_error_threshold(t); }
    void set_mitosis_check_interval(int n)     { gng_.set_mitosis_check_interval(n); }
    void set_mitosis_split_distance(float d)   { gng_.set_mitosis_split_distance(d); }

    // Reset GNG (keeps encoder frozen matrix)
    void reset();

private:
    Config cfg_;

    // Encoders — exactly one is active depending on modality
    std::unique_ptr<FrozenJLEncoder>   enc_jl_;    // visual modalities
    std::unique_ptr<FrozenSTFTEncoder> enc_audio_; // generic audio (STFT)
    std::unique_ptr<FrozenRBFEncoder>  enc_rbf_;   // proprioceptive

    GNG gng_;

    // Dual TLE state
    std::optional<Eigen::VectorXf> prev_prototype_; // last winner's prototype
    float ema_tle_ = 0.1f;   // running EMA of TLE for adaptive threshold

    // Build a RealityToken from a latent vector
    RealityToken make_token(const Eigen::VectorXf& latent);
};

} // namespace v3
} // namespace ami_ogma
