#pragma once

/**
 * FrozenRBFEncoder — Radial Basis Function encoder for proprioceptive state (v3)
 *
 * Encodes a low-dimensional body state vector (position, velocity, acceleration,
 * efference copy error, etc.) into a projection_dim-D latent vector suitable for
 * the GNG.
 *
 * The encoder places a grid of Gaussian RBF centers across the state space.
 * Each center contributes exp(-||state - center||² / 2σ²) to the output.
 * The result is L2-normalised.
 *
 * For low-dim state (≤6D): regular grid along each dimension.
 * For high-dim state (>6D): quasi-random Halton sequence placement of centers.
 *
 * All parameters are frozen at construction time — no learning, matching the
 * FrozenJL and FrozenSTFT design philosophy.
 *
 * Body Schema Config:
 *   state_dims   — number of body state dimensions
 *   dim_ranges   — per-dimension [min, max] for normalisation
 *   sigma        — RBF bandwidth (0 = auto-compute from center spacing)
 *
 * Usage:
 *   FrozenRBFEncoder enc(config);
 *   Eigen::VectorXf latent = enc.encode(state_vector.data(), state_dims);
 */

#include <vector>
#include <cstdint>
#include <Eigen/Dense>

namespace ami_ogma {
namespace v3 {

class FrozenRBFEncoder {
public:
    struct Config {
        int   state_dims     = 4;       // input dimensionality
        int   projection_dim = 128;     // output dimensionality (= number of RBF centers)

        // Per-dimension normalisation ranges: [(min0,max0), (min1,max1), ...]
        // If empty, defaults to [-1, 1] for all dimensions.
        std::vector<std::pair<float,float>> dim_ranges;

        // RBF bandwidth. 0 = auto-compute from mean inter-center distance.
        float sigma = 0.0f;
    };

    explicit FrozenRBFEncoder(const Config& cfg);

    /**
     * Encode a body state vector.
     *
     * @param state  Pointer to float array of size state_dims
     * @param n_dims Number of dimensions (must match config.state_dims)
     * @return Eigen::VectorXf of size projection_dim, L2-normalised.
     */
    Eigen::VectorXf encode(const float* state, int n_dims);

    // Convenience overload
    Eigen::VectorXf encode(const std::vector<float>& state);

    int output_dim()  const { return cfg_.projection_dim; }
    int state_dims()  const { return cfg_.state_dims; }

    /**
     * Replace the per-dimension input ranges after construction.
     *
     * This does NOT touch the frozen part of the encoder: centers are laid out
     * in normalised [0,1]^d and sigma is derived from inter-center distances in
     * that same space, so neither depends on dim_ranges.  The ranges are read
     * in exactly one place — normalise_state() — where they act as a per-dim
     * affine map into [0,1] (with a clamp).  Swapping them therefore recalibrates
     * the SENSOR CONDITIONING upstream of the encoder while leaving the encoder
     * itself frozen, which is what lets EPM's commissioning window exist without
     * violating the "frozen encoder" contract.
     *
     * Caller is responsible for whatever downstream state was expressed in the
     * old units (EPM resets its GNG topology).
     *
     * @param ranges  Exactly state_dims entries; each must have hi > lo.
     */
    void set_dim_ranges(const std::vector<std::pair<float,float>>& ranges);

    /// The installed per-dim ranges (post-default-fill, post-set_dim_ranges).
    const std::vector<std::pair<float,float>>& dim_ranges() const { return cfg_.dim_ranges; }

private:
    Config cfg_;

    // RBF centers: shape (projection_dim, state_dims)
    Eigen::MatrixXf centers_;

    // Precomputed: -1 / (2 * sigma²)
    float neg_inv_2sig2_;

    // Build RBF centers — grid for low-dim, Halton for high-dim
    void build_centers();

    // Normalise raw state to [0, 1] per dimension
    Eigen::VectorXf normalise_state(const float* state, int n_dims) const;
};

} // namespace v3
} // namespace ami_ogma
