#pragma once

/**
 * FrozenJLEncoder — Johnson-Lindenstrauss random projection encoder (v3)
 *
 * Ports src/ami_ogma_v3/encoders.py :: FrozenProjectionEncoder to C++.
 *
 * Each visual modality gets its own frozen Gaussian projection matrix R drawn
 * once from a seeded RNG.  The seed is derived from the modality name via the
 * same polynomial hash used in Python so the C++ encoder is independently
 * reproducible (same C++ run → same matrix; different from Python's PCG64 but
 * same statistical guarantees).
 *
 * Algorithm (per tick):
 *   1. Resize raw frame to canonical (H, W, C) resolution.
 *   2. Flatten to 1-D and L2-normalise.
 *   3. Multiply: flat @ R   → shape (projection_dim,)
 *   4. L2-normalise result.
 *
 * Stateful modalities:
 *   optical_flow / dorsal : Farneback flow at full input resolution (requires OpenCV)
 *   saliency              : motion-weighted temporal difference
 *
 * Without OpenCV, flow and saliency modalities fall back to retinal-style
 * still-image projection (still useful, just no motion information).
 */

#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <Eigen/Dense>

namespace ami_ogma {
namespace v3 {

// Per-modality canonical spatial resolution and channel count.
// These must match encoders.py :: _SPATIAL_RES and _SPATIAL_CHANNELS exactly.
struct ModalitySpec {
    int h, w, c;
};

ModalitySpec get_modality_spec(const std::string& modality);

// ---------------------------------------------------------------------------
// FrozenJLEncoder
// ---------------------------------------------------------------------------

class FrozenJLEncoder {
public:
    /**
     * Construct encoder for the given modality.
     *
     * @param modality      One of: retinal, color, optical_flow, saliency,
     *                      dorsal, ventral
     * @param projection_dim Output dimensionality (must match GNG dim)
     */
    /**
     * @param encoder_res      Override spatial resolution (0 = use default from modality spec).
     *                         Sets both H and W to this value; channels stay per-modality.
     * @param inject_centroid  Append the 2D center-of-mass of the preprocessed
     *                         map (currently saliency only) to the pre-JL input.
     *                         Adds 2 rows to the projection matrix R.  Output dim
     *                         is unchanged (still projection_dim).
     * @param centroid_gain    Energy scaling for the centroid channels (default
     *                         sqrt(input_dim/2) ≈ 22.6).  Only used when
     *                         inject_centroid=true.
     */
    explicit FrozenJLEncoder(const std::string& modality, int projection_dim = 128,
                             int encoder_res = 0,
                             bool inject_centroid = false,
                             float centroid_gain = 22.6f);

    /**
     * State-input constructor (proprioceptive / body-state modalities).
     *
     * Builds R with shape (input_dim, projection_dim) instead of deriving the
     * input size from a pixel spec. The projection is dimension-agnostic —
     * JL preserves pairwise distances for any input dim, which is why we use
     * it for proprio instead of RBF (RBF needs a center grid that grows
     * exponentially with input_dim).
     *
     * Call encode_state() for this path; encode(pixels,...) will return zero
     * because spec_ is unset here.
     */
    static std::unique_ptr<FrozenJLEncoder> make_state_encoder(
        const std::string& modality, int projection_dim, int input_dim);

    /**
     * Encode a body-state vector (proprioceptive path).
     *
     * @param state  Pointer to float array of length n_dims.
     * @param n_dims Must equal the input_dim passed to the state ctor.
     * @return projection_dim-vector, L2-normalised. Zero vector on mismatch.
     */
    Eigen::VectorXf encode_state(const float* state, int n_dims);

    /**
     * Encode a raw video frame.
     *
     * @param pixels  Flat pixel data, row-major RGB uint8 (H × W × 3)
     *                or grayscale uint8 (H × W).
     * @param in_h    Input frame height in pixels
     * @param in_w    Input frame width in pixels
     * @param in_c    Input channel count (1 = grayscale, 3 = RGB)
     *
     * @return Eigen::VectorXf of size projection_dim, L2-normalised.
     *         Returns zero vector if input is empty.
     */
    Eigen::VectorXf encode(const uint8_t* pixels,
                           int in_h, int in_w, int in_c);

    // Convenience overload accepting a flat std::vector<uint8_t>
    Eigen::VectorXf encode(const std::vector<uint8_t>& pixels,
                           int in_h, int in_w, int in_c);

    // Output dimension
    int output_dim() const { return projection_dim_; }

    // Canonical input dimensions after resize
    int target_h() const { return spec_.h; }
    int target_w() const { return spec_.w; }
    int channels() const { return spec_.c; }

    const std::string& modality() const { return modality_; }

private:
    std::string  modality_;
    int          projection_dim_;
    ModalitySpec spec_;

    // Frozen projection matrix R: shape (input_dim [+2 if centroid], projection_dim)
    Eigen::MatrixXf R_;

    // Stateful previous grayscale frame for flow/saliency modalities
    // Shape: (spec_.h * spec_.w) uint8 at target resolution
    std::vector<uint8_t> prev_gray_;
    bool has_prev_ = false;

    // Centroid injection configuration (see ctor doc)
    bool  inject_centroid_ = false;
    float centroid_gain_   = 22.6f;

    // Internal helpers
    static uint32_t modality_seed(const std::string& modality);
    void build_projection_matrix(uint32_t seed);

    // Resize float frame [0,1] from (in_h,in_w,in_c) to (th,tw,tc)
    std::vector<float> resize_frame(const float* src,
                                    int in_h, int in_w, int in_c,
                                    int th, int tw, int tc) const;

    // Still-image projection path (retinal, color, ventral, dorsal)
    Eigen::VectorXf encode_still(const float* frame_01,
                                 int in_h, int in_w, int in_c);

    // Apply JL projection to a flat, L2-normalised input vector
    Eigen::VectorXf project(const std::vector<float>& flat) const;

#ifdef USE_OPENCV
    // Optical flow projection path (optical_flow, dorsal)
    Eigen::VectorXf encode_flow(const float* frame_01,
                                int in_h, int in_w, int in_c);

    // Motion-weighted saliency path
    Eigen::VectorXf encode_saliency(const float* frame_01,
                                    int in_h, int in_w, int in_c);
#endif
};

} // namespace v3
} // namespace ami_ogma
