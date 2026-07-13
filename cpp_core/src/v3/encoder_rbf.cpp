/**
 * FrozenRBFEncoder implementation — Radial Basis Function encoder for
 * proprioceptive body state.
 */

#include "v3/encoder_rbf.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace ami_ogma {
namespace v3 {

// ---------------------------------------------------------------------------
// Halton sequence for quasi-random center placement in high dimensions
// ---------------------------------------------------------------------------

static float halton(int index, int base) {
    float result = 0.0f;
    float f = 1.0f / static_cast<float>(base);
    int i = index;
    while (i > 0) {
        result += f * (i % base);
        i /= base;
        f /= static_cast<float>(base);
    }
    return result;
}

// First 20 primes for Halton sequence bases
static const int PRIMES[] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29,
                              31, 37, 41, 43, 47, 53, 59, 61, 67, 71};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

FrozenRBFEncoder::FrozenRBFEncoder(const Config& cfg)
    : cfg_(cfg)
{
    // Default dim_ranges to [-1, 1] if not specified
    if (cfg_.dim_ranges.empty()) {
        cfg_.dim_ranges.resize(cfg_.state_dims, {-1.0f, 1.0f});
    }

    build_centers();

    // Auto-compute sigma from mean nearest-neighbor distance between centers
    if (cfg_.sigma <= 0.0f) {
        float total_nn_dist = 0.0f;
        int n = cfg_.projection_dim;
        for (int i = 0; i < n; ++i) {
            float min_d = std::numeric_limits<float>::infinity();
            for (int j = 0; j < n; ++j) {
                if (i == j) continue;
                float d = (centers_.row(i) - centers_.row(j)).norm();
                if (d < min_d) min_d = d;
            }
            total_nn_dist += min_d;
        }
        cfg_.sigma = (total_nn_dist / n) * 0.8f;  // slightly tighter than spacing
    }

    neg_inv_2sig2_ = -1.0f / (2.0f * cfg_.sigma * cfg_.sigma);
}

// ---------------------------------------------------------------------------
// Build RBF centers
// ---------------------------------------------------------------------------

void FrozenRBFEncoder::build_centers() {
    int n_centers = cfg_.projection_dim;
    int dims = cfg_.state_dims;

    centers_.resize(n_centers, dims);

    if (dims <= 6) {
        // Regular grid: compute points per dimension
        // n_centers ≈ pts_per_dim^dims → pts_per_dim = ceil(n_centers^(1/dims))
        int pts = static_cast<int>(std::ceil(std::pow(
            static_cast<double>(n_centers), 1.0 / dims)));

        // Generate grid points in [0, 1]^dims, take first n_centers
        int idx = 0;
        std::vector<int> coord(dims, 0);

        while (idx < n_centers) {
            for (int d = 0; d < dims; ++d) {
                centers_(idx, d) = static_cast<float>(coord[d]) /
                                   static_cast<float>(std::max(1, pts - 1));
            }
            ++idx;

            // Increment coordinate (mixed-radix counter)
            int carry = 1;
            for (int d = dims - 1; d >= 0 && carry; --d) {
                coord[d] += carry;
                if (coord[d] >= pts) {
                    coord[d] = 0;
                } else {
                    carry = 0;
                }
            }
            if (carry) break;  // overflow — shouldn't happen if pts^dims >= n_centers
        }

        // If grid produced fewer points than needed, fill remainder with Halton
        for (; idx < n_centers; ++idx) {
            for (int d = 0; d < dims; ++d) {
                centers_(idx, d) = halton(idx + 1, PRIMES[d % 20]);
            }
        }
    } else {
        // High-dim: Halton quasi-random sequence
        for (int i = 0; i < n_centers; ++i) {
            for (int d = 0; d < dims; ++d) {
                centers_(i, d) = halton(i + 1, PRIMES[d % 20]);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Normalise state to [0, 1] per dimension
// ---------------------------------------------------------------------------

Eigen::VectorXf FrozenRBFEncoder::normalise_state(const float* state, int n_dims) const {
    Eigen::VectorXf norm(n_dims);
    for (int d = 0; d < n_dims; ++d) {
        float lo = cfg_.dim_ranges[d].first;
        float hi = cfg_.dim_ranges[d].second;
        float range = hi - lo;
        if (range < 1e-8f) {
            norm(d) = 0.5f;
        } else {
            norm(d) = std::clamp((state[d] - lo) / range, 0.0f, 1.0f);
        }
    }
    return norm;
}

// ---------------------------------------------------------------------------
// Encode
// ---------------------------------------------------------------------------

Eigen::VectorXf FrozenRBFEncoder::encode(const float* state, int n_dims) {
    if (n_dims != cfg_.state_dims) {
        return Eigen::VectorXf::Zero(cfg_.projection_dim);
    }

    Eigen::VectorXf s = normalise_state(state, n_dims);
    Eigen::VectorXf activations(cfg_.projection_dim);

    for (int i = 0; i < cfg_.projection_dim; ++i) {
        float dist_sq = (s.transpose() - centers_.row(i)).squaredNorm();
        activations(i) = std::exp(dist_sq * neg_inv_2sig2_);
    }

    // L2 normalise
    float norm = activations.norm();
    if (norm > 1e-8f) {
        activations /= norm;
    }

    return activations;
}

Eigen::VectorXf FrozenRBFEncoder::encode(const std::vector<float>& state) {
    return encode(state.data(), static_cast<int>(state.size()));
}

} // namespace v3
} // namespace ami_ogma
