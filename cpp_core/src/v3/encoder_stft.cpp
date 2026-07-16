#include "v3/encoder_stft.hpp"

#include <cmath>
#include <algorithm>

namespace ami_ogma {
namespace v3 {

namespace {
constexpr float kTwoPi = 6.28318530717958647692f;
}

FrozenSTFTEncoder::FrozenSTFTEncoder(const Config& cfg)
    : cfg_(cfg),
      center_freqs_(log_center_freqs(cfg.n_filters, cfg.f_min, cfg.f_max)) {}

std::vector<float>
FrozenSTFTEncoder::log_center_freqs(int n, float f_min, float f_max) {
    std::vector<float> out(std::max(n, 1));
    const float lo = std::log(std::max(f_min, 1.0f));
    const float hi = std::log(std::max(f_max, f_min + 1.0f));
    const int   denom = std::max(n - 1, 1);
    for (int i = 0; i < n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(denom);
        out[i] = std::exp(lo + t * (hi - lo));
    }
    return out;
}

Eigen::VectorXf FrozenSTFTEncoder::encode_mono(const float* samples, int n) {
    const int F = cfg_.n_filters;
    Eigen::VectorXf feat(F);
    if (n <= 0 || samples == nullptr) {
        feat.setZero();
        return feat;
    }
    const float fs = static_cast<float>(cfg_.sample_rate);
    // Per-band magnitude via a direct DFT evaluation at the band centre.
    // TODO(xaq-audio): windowed STFT frames + mel filterbank + JL projection.
    for (int b = 0; b < F; ++b) {
        const float w = kTwoPi * center_freqs_[b] / fs;
        float re = 0.0f, im = 0.0f;
        for (int k = 0; k < n; ++k) {
            const float phase = w * static_cast<float>(k);
            re += samples[k] * std::cos(phase);
            im -= samples[k] * std::sin(phase);
        }
        const float mag = std::sqrt(re * re + im * im) / static_cast<float>(n);
        feat[b] = std::log1p(mag);
    }
    const float norm = feat.norm() + 0.01f;  // soft-normalise, matches v3 encoders
    feat /= norm;
    return feat;
}

Eigen::VectorXf
FrozenSTFTEncoder::encode_stereo(const float* left, const float* right, int n) {
    if (n <= 0 || left == nullptr || right == nullptr) {
        return encode_mono(nullptr, 0);
    }
    std::vector<float> mono(static_cast<size_t>(n));
    for (int k = 0; k < n; ++k) mono[k] = 0.5f * (left[k] + right[k]);
    return encode_mono(mono.data(), n);
}

Eigen::VectorXf FrozenSTFTEncoder::encode(const std::vector<float>& buf, int channels) {
    if (channels <= 1) {
        return encode_mono(buf.data(), static_cast<int>(buf.size()));
    }
    // Interleaved stereo (n, 2): de-interleave then fold to mono.
    const int n = static_cast<int>(buf.size()) / 2;
    std::vector<float> left(static_cast<size_t>(n)), right(static_cast<size_t>(n));
    for (int k = 0; k < n; ++k) {
        left[k]  = buf[static_cast<size_t>(2 * k)];
        right[k] = buf[static_cast<size_t>(2 * k + 1)];
    }
    return encode_stereo(left.data(), right.data(), n);
}

} // namespace v3
} // namespace ami_ogma
