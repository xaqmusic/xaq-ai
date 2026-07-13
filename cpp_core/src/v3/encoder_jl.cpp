/**
 * FrozenJLEncoder implementation — C++ port of encoders.py :: FrozenProjectionEncoder
 */

#include "v3/encoder_jl.hpp"
#include <cmath>
#include <random>
#include <stdexcept>
#include <cstring>
#include <algorithm>

#ifdef USE_OPENCV
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#endif

namespace ami_ogma {
namespace v3 {

// ---------------------------------------------------------------------------
// Modality spec table — must match encoders.py :: _SPATIAL_RES / _SPATIAL_CHANNELS
// ---------------------------------------------------------------------------

ModalitySpec get_modality_spec(const std::string& modality) {
    if (modality == "retinal")      return {32, 32, 1};
    if (modality == "color")        return {24, 24, 3};
    if (modality == "optical_flow") return {20, 20, 2};
    if (modality == "saliency")     return {32, 32, 1};
    if (modality == "dorsal")       return {32, 32, 1};
    if (modality == "ventral")      return {32, 32, 1};
    // Unknown modality: generic 32×32 grayscale
    return {32, 32, 1};
}

// ---------------------------------------------------------------------------
// Modality seed — polynomial hash matching encoders.py :: _modality_seed()
// h = 0; for ch in modality.encode(): h = (h*31 + ch) & 0xFFFFFFFF
// ---------------------------------------------------------------------------

uint32_t FrozenJLEncoder::modality_seed(const std::string& modality) {
    uint32_t h = 0;
    for (unsigned char ch : modality) {
        h = (h * 31u + ch) & 0xFFFFFFFFu;
    }
    return h;
}

// ---------------------------------------------------------------------------
// Build frozen projection matrix R: (input_dim, projection_dim)
// Drawn from N(0, 1/sqrt(projection_dim)) with seeded mt19937.
//
// Note: C++ mt19937 + std::normal_distribution generates different numbers
// than Python's PCG64, but the JL guarantee holds for any zero-mean Gaussian
// with variance 1/projection_dim.  The matrix is reproducible across C++ runs.
// ---------------------------------------------------------------------------

void FrozenJLEncoder::build_projection_matrix(uint32_t seed) {
    // With centroid injection the flat input vector grows by 2 (x_norm, y_norm).
    // The extra rows of R map those centroid channels into the 128D latent.
    const int base_dim  = spec_.h * spec_.w * spec_.c;
    const int input_dim = base_dim + (inject_centroid_ ? 2 : 0);
    std::mt19937 rng(seed);
    const float scale = 1.0f / std::sqrt(static_cast<float>(projection_dim_));
    std::normal_distribution<float> dist(0.0f, scale);

    R_.resize(input_dim, projection_dim_);
    for (int i = 0; i < input_dim; ++i)
        for (int j = 0; j < projection_dim_; ++j)
            R_(i, j) = dist(rng);
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

FrozenJLEncoder::FrozenJLEncoder(const std::string& modality, int projection_dim,
                                 int encoder_res,
                                 bool inject_centroid, float centroid_gain)
    : modality_(modality)
    , projection_dim_(projection_dim)
    , spec_(get_modality_spec(modality))
    , inject_centroid_(inject_centroid && modality == "saliency")
    , centroid_gain_(centroid_gain)
{
    if (encoder_res > 0) {
        spec_.h = encoder_res;
        spec_.w = encoder_res;
        // channels stay per-modality default
    }
    build_projection_matrix(modality_seed(modality));
}

// ---------------------------------------------------------------------------
// State-input constructor (proprioceptive / body-state modalities).
// Builds R directly of shape (input_dim, projection_dim) — no pixel spec.
// ---------------------------------------------------------------------------

std::unique_ptr<FrozenJLEncoder> FrozenJLEncoder::make_state_encoder(
    const std::string& modality, int projection_dim, int input_dim) {
    // Construct with encoder_res=1, no centroid → builds a dummy (1*1*c) R_,
    // then we overwrite R_ with the correct (input_dim, projection_dim) matrix.
    auto enc = std::unique_ptr<FrozenJLEncoder>(
        new FrozenJLEncoder(modality, projection_dim, 1, false, 1.0f));
    enc->spec_ = {0, 0, 0};
    std::mt19937 rng(modality_seed(modality));
    const float scale = 1.0f / std::sqrt(static_cast<float>(projection_dim));
    std::normal_distribution<float> dist(0.0f, scale);
    enc->R_.resize(input_dim, projection_dim);
    for (int i = 0; i < input_dim; ++i)
        for (int j = 0; j < projection_dim; ++j)
            enc->R_(i, j) = dist(rng);
    return enc;
}

Eigen::VectorXf FrozenJLEncoder::encode_state(const float* state, int n_dims) {
    if (!state || n_dims <= 0 || n_dims != R_.rows())
        return Eigen::VectorXf::Zero(projection_dim_);
    std::vector<float> flat(state, state + n_dims);
    return project(flat);
}

// ---------------------------------------------------------------------------
// Bilinear resize: (in_h, in_w, in_c) float → (th, tw, tc) float
// Handles channel mismatch: collapse to 1 ch or replicate to fill.
// ---------------------------------------------------------------------------

std::vector<float> FrozenJLEncoder::resize_frame(const float* src,
                                                   int in_h, int in_w, int in_c,
                                                   int th, int tw, int tc) const {
    // Step 1: channel adjustment on the fly during resize
    // We produce target_c channels from in_c input channels:
    //   tc==1 && in_c>1  → mean across channels
    //   tc>1  && in_c==1 → replicate
    //   tc==in_c         → copy
    //   tc < in_c        → take first tc channels

    std::vector<float> out(th * tw * tc, 0.0f);

    const float scale_y = static_cast<float>(in_h) / th;
    const float scale_x = static_cast<float>(in_w) / tw;

    for (int oy = 0; oy < th; ++oy) {
        float fy = (oy + 0.5f) * scale_y - 0.5f;
        int y0 = static_cast<int>(std::floor(fy));
        int y1 = y0 + 1;
        float wy1 = fy - y0;
        float wy0 = 1.0f - wy1;
        y0 = std::max(0, std::min(y0, in_h - 1));
        y1 = std::max(0, std::min(y1, in_h - 1));

        for (int ox = 0; ox < tw; ++ox) {
            float fx = (ox + 0.5f) * scale_x - 0.5f;
            int x0 = static_cast<int>(std::floor(fx));
            int x1 = x0 + 1;
            float wx1 = fx - x0;
            float wx0 = 1.0f - wx1;
            x0 = std::max(0, std::min(x0, in_w - 1));
            x1 = std::max(0, std::min(x1, in_w - 1));

            float* dst_pix = &out[(oy * tw + ox) * tc];

            // For each output channel, bilinearly interpolate from the input
            for (int oc = 0; oc < tc; ++oc) {
                // Determine which input channel(s) to read
                if (in_c == tc) {
                    int ic = oc;
                    float v00 = src[(y0*in_w + x0)*in_c + ic];
                    float v01 = src[(y0*in_w + x1)*in_c + ic];
                    float v10 = src[(y1*in_w + x0)*in_c + ic];
                    float v11 = src[(y1*in_w + x1)*in_c + ic];
                    dst_pix[oc] = wy0*(wx0*v00 + wx1*v01) + wy1*(wx0*v10 + wx1*v11);
                } else if (tc == 1) {
                    // Average all input channels
                    float sum = 0.0f;
                    for (int ic = 0; ic < in_c; ++ic) {
                        float v00 = src[(y0*in_w + x0)*in_c + ic];
                        float v01 = src[(y0*in_w + x1)*in_c + ic];
                        float v10 = src[(y1*in_w + x0)*in_c + ic];
                        float v11 = src[(y1*in_w + x1)*in_c + ic];
                        sum += wy0*(wx0*v00 + wx1*v01) + wy1*(wx0*v10 + wx1*v11);
                    }
                    dst_pix[oc] = sum / in_c;
                } else if (in_c == 1) {
                    // Replicate single channel
                    float v00 = src[(y0*in_w + x0)];
                    float v01 = src[(y0*in_w + x1)];
                    float v10 = src[(y1*in_w + x0)];
                    float v11 = src[(y1*in_w + x1)];
                    dst_pix[oc] = wy0*(wx0*v00 + wx1*v01) + wy1*(wx0*v10 + wx1*v11);
                } else {
                    // Take first min(in_c, tc) channels; zero-pad
                    int ic = (oc < in_c) ? oc : in_c - 1;
                    float v00 = src[(y0*in_w + x0)*in_c + ic];
                    float v01 = src[(y0*in_w + x1)*in_c + ic];
                    float v10 = src[(y1*in_w + x0)*in_c + ic];
                    float v11 = src[(y1*in_w + x1)*in_c + ic];
                    dst_pix[oc] = wy0*(wx0*v00 + wx1*v01) + wy1*(wx0*v10 + wx1*v11);
                }
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// JL projection: flatten → L2 normalize → R @ flat → L2 normalize
// ---------------------------------------------------------------------------

Eigen::VectorXf FrozenJLEncoder::project(const std::vector<float>& flat) const {
    const int n = static_cast<int>(flat.size());
    Eigen::Map<const Eigen::VectorXf> v(flat.data(), n);

    float norm = v.norm();
    Eigen::VectorXf v_norm = v;
    if (norm > 1e-6f) v_norm /= norm;

    Eigen::VectorXf projected = R_.transpose() * v_norm;  // (projection_dim,)

    float p_norm = projected.norm();
    if (p_norm > 1e-6f)
        projected /= p_norm;

    return projected;
}

// ---------------------------------------------------------------------------
// Still-image encode (retinal, color, ventral, dorsal)
// ---------------------------------------------------------------------------

Eigen::VectorXf FrozenJLEncoder::encode_still(const float* frame_01,
                                               int in_h, int in_w, int in_c) {
    auto resized = resize_frame(frame_01, in_h, in_w, in_c,
                                spec_.h, spec_.w, spec_.c);
    return project(resized);
}

// ---------------------------------------------------------------------------
// Optical flow encode (requires OpenCV) — matching Python flow branch
// ---------------------------------------------------------------------------

#ifdef USE_OPENCV

Eigen::VectorXf FrozenJLEncoder::encode_flow(const float* frame_01,
                                              int in_h, int in_w, int in_c) {
    // Convert float RGB to uint8 grayscale at FULL input resolution
    cv::Mat frame_u8(in_h, in_w, (in_c >= 3) ? CV_8UC3 : CV_8UC1);
    for (int i = 0; i < in_h * in_w * in_c; ++i)
        frame_u8.data[i] = static_cast<uint8_t>(std::min(frame_01[i] * 255.0f, 255.0f));

    cv::Mat gray_full;
    if (in_c >= 3)
        cv::cvtColor(frame_u8, gray_full, cv::COLOR_RGB2GRAY);
    else
        gray_full = frame_u8.clone();

    Eigen::VectorXf result = Eigen::VectorXf::Zero(projection_dim_);

    if (has_prev_) {
        cv::Mat prev_gray(in_h, in_w, CV_8UC1,
                          const_cast<uint8_t*>(prev_gray_.data()));

        cv::Mat flow_full;
        cv::calcOpticalFlowFarneback(
            prev_gray, gray_full, flow_full,
            0.5,   // pyr_scale
            2,     // levels    (was 3 — saves ~33%)
            7,     // winsize   (was 15 — saves ~75% per level)
            2,     // iterations (was 3 — saves ~33%)
            5,     // poly_n
            1.2,   // poly_sigma
            0      // flags
        );  // (in_h, in_w, CV_32FC2)

        // Downsample flow to target (spec_.h, spec_.w)
        cv::Mat flow_small;
        cv::resize(flow_full, flow_small,
                   cv::Size(spec_.w, spec_.h),
                   0, 0, cv::INTER_AREA);

        // Normalize ±16px → [-1, 1]
        const float max_disp = 16.0f;
        std::vector<float> flat(spec_.h * spec_.w * 2);
        for (int i = 0; i < spec_.h * spec_.w; ++i) {
            cv::Vec2f pv = flow_small.at<cv::Vec2f>(i / spec_.w, i % spec_.w);
            flat[i*2 + 0] = std::max(-1.0f, std::min(1.0f, pv[0] / max_disp));
            flat[i*2 + 1] = std::max(-1.0f, std::min(1.0f, pv[1] / max_disp));
        }

        result = project(flat);
    }

    // Update previous frame
    prev_gray_.resize(in_h * in_w);
    std::memcpy(prev_gray_.data(), gray_full.data, in_h * in_w);
    has_prev_ = true;

    return result;
}

Eigen::VectorXf FrozenJLEncoder::encode_saliency(const float* frame_01,
                                                   int in_h, int in_w, int in_c) {
    // Convert to uint8 grayscale at full resolution
    cv::Mat frame_u8(in_h, in_w, (in_c >= 3) ? CV_8UC3 : CV_8UC1);
    for (int i = 0; i < in_h * in_w * in_c; ++i)
        frame_u8.data[i] = static_cast<uint8_t>(std::min(frame_01[i] * 255.0f, 255.0f));

    cv::Mat gray_full;
    if (in_c >= 3)
        cv::cvtColor(frame_u8, gray_full, cv::COLOR_RGB2GRAY);
    else
        gray_full = frame_u8.clone();

    // Laplacian edge magnitude
    cv::Mat lap;
    cv::Laplacian(gray_full, lap, CV_32F);
    cv::Mat edges_abs;
    cv::convertScaleAbs(lap, edges_abs);
    cv::Mat edges_f;
    edges_abs.convertTo(edges_f, CV_32F, 1.0 / 255.0);
    double e_max;
    cv::minMaxLoc(edges_f, nullptr, &e_max);
    if (e_max > 1e-6)
        edges_f /= static_cast<float>(e_max);

    cv::Mat gray_f;
    gray_full.convertTo(gray_f, CV_32F, 1.0 / 255.0);

    cv::Mat salient;
    if (has_prev_) {
        cv::Mat prev_gray(in_h, in_w, CV_8UC1,
                          const_cast<uint8_t*>(prev_gray_.data()));
        cv::Mat prev_f;
        prev_gray.convertTo(prev_f, CV_32F, 1.0 / 255.0);

        cv::Mat motion;
        cv::absdiff(gray_f, prev_f, motion);

        // salient = motion * (1 + 0.5 * edges_norm)
        cv::Mat weight = 1.0f + 0.5f * edges_f;
        cv::multiply(motion, weight, salient);

        // Log-compress: log1p(salient * 9) / log1p(9)
        for (int i = 0; i < salient.rows * salient.cols; ++i) {
            float& v = salient.at<float>(i / salient.cols, i % salient.cols);
            v = std::log1p(v * 9.0f) / std::log1p(9.0f);
        }
    } else {
        // First frame: spatial edges only
        salient = edges_f * 0.3f;
    }

    // Downsample to target resolution
    cv::Mat sal_small;
    cv::resize(salient, sal_small, cv::Size(spec_.w, spec_.h), 0, 0, cv::INTER_AREA);

    const int n_px = spec_.h * spec_.w;
    std::vector<float> flat(n_px + (inject_centroid_ ? 2 : 0));
    for (int i = 0; i < n_px; ++i)
        flat[i] = sal_small.at<float>(i / spec_.w, i % spec_.w);

    // 2D center-of-mass of the saliency map.  Encodes "where is the moving
    // high-contrast region" as a 2-channel spatial anchor — the axis the raw
    // JL projection destroys.  Gain matches per-pixel energy so centroid
    // influence survives the L2 normalization inside project().
    if (inject_centroid_) {
        double mass  = 0.0;
        double mom_x = 0.0;
        double mom_y = 0.0;
        for (int y = 0; y < spec_.h; ++y) {
            for (int x = 0; x < spec_.w; ++x) {
                const float v = sal_small.at<float>(y, x);
                mass  += v;
                mom_x += v * x;
                mom_y += v * y;
            }
        }
        float x_norm = 0.0f;
        float y_norm = 0.0f;
        if (mass > 1e-9 && spec_.w > 1 && spec_.h > 1) {
            const float cx = static_cast<float>(mom_x / mass);
            const float cy = static_cast<float>(mom_y / mass);
            x_norm = 2.0f * cx / (spec_.w - 1) - 1.0f;  // [-1, 1]
            y_norm = 2.0f * cy / (spec_.h - 1) - 1.0f;  // [-1, 1]
        }
        flat[n_px + 0] = centroid_gain_ * x_norm;
        flat[n_px + 1] = centroid_gain_ * y_norm;
    }

    // Update previous frame
    prev_gray_.resize(in_h * in_w);
    std::memcpy(prev_gray_.data(), gray_full.data, in_h * in_w);
    has_prev_ = true;

    return project(flat);
}

#endif  // USE_OPENCV

// ---------------------------------------------------------------------------
// Public encode entry point
// ---------------------------------------------------------------------------

Eigen::VectorXf FrozenJLEncoder::encode(const uint8_t* pixels,
                                         int in_h, int in_w, int in_c) {
    if (!pixels || in_h <= 0 || in_w <= 0 || in_c <= 0)
        return Eigen::VectorXf::Zero(projection_dim_);

    // Convert uint8 → float [0, 1]
    const int n_px = in_h * in_w * in_c;
    std::vector<float> frame_f(n_px);
    for (int i = 0; i < n_px; ++i)
        frame_f[i] = pixels[i] / 255.0f;

    const float* f = frame_f.data();

#ifdef USE_OPENCV
    if (modality_ == "optical_flow" || modality_ == "dorsal")
        return encode_flow(f, in_h, in_w, in_c);
    if (modality_ == "saliency")
        return encode_saliency(f, in_h, in_w, in_c);
#endif

    return encode_still(f, in_h, in_w, in_c);
}

Eigen::VectorXf FrozenJLEncoder::encode(const std::vector<uint8_t>& pixels,
                                         int in_h, int in_w, int in_c) {
    return encode(pixels.data(), in_h, in_w, in_c);
}

} // namespace v3
} // namespace ami_ogma
