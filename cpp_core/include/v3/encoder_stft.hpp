#pragma once

/**
 * FrozenSTFTEncoder — generic, IP-free audio front-end for the EPM audio slot.
 *
 * This is the C++ analogue of zanshin/encoders/audio.py :: FrozenSTFTEncoder.
 * It occupies the EPM's "audio" encoder kind so the multi-modal fusion path
 * keeps an audio modality WITHOUT shipping any bio-mimetic cochlear IP (the
 * Hopf/ERB/MOC/binaural pipeline lives in the private AMI-Awen tree).
 *
 * Algorithm (fully determined at construction — no learned parameters):
 *   1. Log-spaced band centre frequencies between f_min and f_max.
 *   2. Per-band magnitude via a direct DFT evaluation over the chunk
 *      (Goertzel-style: |Σ x[n]·exp(-j2π f n / fs)|). O(F·N) per tick.
 *   3. log1p compression, mean over the chunk, soft L2-normalisation.
 *   4. Stereo → mono mean (no ITD/binaural — that is AMI-Awen only).
 *
 * Output: n_filters-D float vector, soft-normalised — same shape contract as
 * the other v3 encoders so the GNG needs no reconfiguration.
 *
 * TODO(zanshin-audio): replace the single-frame direct-DFT with a proper
 * windowed STFT (framing + hop + Hann window), a fixed mel filterbank, and a
 * seeded Johnson–Lindenstrauss projection into encoder_jl — the efficient,
 * higher-fidelity generic pipeline. This stub is deliberately minimal but
 * correct and deterministic.
 */

#include <vector>
#include <cstdint>
#include <Eigen/Dense>

namespace ami_ogma {
namespace v3 {

class FrozenSTFTEncoder {
public:
    struct Config {
        int   n_filters   = 128;
        int   sample_rate = 48000;
        float f_min       = 80.0f;
        float f_max       = 8000.0f;
    };

    explicit FrozenSTFTEncoder(const Config& cfg);

    // Encode a mono audio chunk (samples in [-1, 1]).
    Eigen::VectorXf encode_mono(const float* samples, int n);

    // Encode a stereo chunk — generic path folds L/R to mono (no ITD).
    Eigen::VectorXf encode_stereo(const float* left, const float* right, int n);

    // Dispatch from a flat buffer: (n,) mono, (n,2) interleaved, (2,n) planar.
    Eigen::VectorXf encode(const std::vector<float>& buf, int channels = 1);

    int   output_dim() const { return cfg_.n_filters; }
    float last_mu()    const { return 0.0f; }  // inert — no MOC in the generic path

private:
    Config             cfg_;
    std::vector<float> center_freqs_;  // log-spaced band centres [Hz]

    static std::vector<float> log_center_freqs(int n, float f_min, float f_max);
};

} // namespace v3
} // namespace ami_ogma
