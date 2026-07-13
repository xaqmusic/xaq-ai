#pragma once

/**
 * AMI-Ogma v3 — Core Types
 *
 * Clean tabula rasa architecture: fully geometric, no ONNX, no pretrained weights.
 * All encoding is via frozen random projections (JL) or frozen spectral filterbanks (STFT).
 */

#include <cstdint>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>

namespace ami_ogma {
namespace v3 {

// ---------------------------------------------------------------------------
// EPM configuration
// ---------------------------------------------------------------------------

struct EPMConfig {
    std::string modality = "retinal";   // encoder type
    int projection_dim   = 128;         // GNG input dimension (encoder output dim)

    // GNG hyperparameters
    int   baking_threshold    = 50;
    float min_insertion_error = 0.02f;
    int   lambda_new          = 25;
    int   max_age             = 88;
    float epsilon_b           = 0.05f;  // winner learning rate
    float epsilon_n           = 0.003f; // neighbour learning rate
    float alpha               = 0.5f;   // error halving on insertion
    float beta                = 0.0005f;// global error decay per step
    int   max_nodes           = 2000;

    bool  stale_prune_enabled = true;
    float stale_window_factor = 12000.0f; // absolute steps (~400s at 30fps)

    // Biological health model
    float health_boost             = 0.5f;
    float health_base_decay        = 0.995f;
    float health_resilience_k      = 0.08f;
    float health_death_threshold   = 0.01f;
    int   max_deaths_per_tick      = 1;
    // Anti-collapse: population floor, death cooldown, near-baked protection.
    int   health_death_min_nodes   = 16;
    int   death_cooldown_steps     = 25;
    float near_baked_fraction      = 0.6f;

    // Mitosis Gatekeeper
    bool  mitosis_enabled         = true;
    float mitosis_error_threshold = 0.30f; // post-bake mean error to trigger split
    int   mitosis_check_interval  = 50;    // post-bake visits between checks
    float mitosis_split_distance  = 0.10f; // daughter offset (fraction of prototype norm)

    // Audio (generic STFT)
    int   sample_rate    = 48000;
    float f_min          = 80.0f;
    float f_max          = 8000.0f;

    // Encoder resolution override (0 = use modality default)
    int   encoder_res    = 0;      // sets both H and W; channels stay per-modality

    // Saliency-map centroid injection (visual modalities only).
    // When enabled, the 2D center-of-mass of the preprocessed map is appended
    // to the pre-JL input vector (input_dim grows by 2).  Output dim unchanged.
    // Self-derived, preserves tabula rasa.  See docs/v3_architecture_status.md §7.
    bool  inject_centroid = false;
    float centroid_gain   = 22.6f; // ≈ sqrt(input_dim/2) — matches per-pixel energy

    // Proprioceptive body schema (proprioceptive modality only)
    int   proprio_state_dims = 4;   // [position, velocity, acceleration, efference_error]
    std::vector<std::pair<float,float>> proprio_dim_ranges;  // per-dim [min,max]; empty = [-1,1]

    // Control server
    uint16_t control_port = 7200;
};

// ---------------------------------------------------------------------------
// Reality Token — emitted by the EPM on every tick
// ---------------------------------------------------------------------------

struct RealityToken {
    int64_t  timestamp_us    = 0;       // microseconds since epoch
    int      winner_id       = -1;      // stable GNG node ID
    float    quant_error     = 0.0f;    // distance from input to winner prototype
    float    transition_surp = 0.0f;    // distance between consecutive winners (TLE component 2)
    float    tle             = 0.0f;    // combined time-loop error
    float    threshold       = 0.0f;   // adaptive novelty threshold this tick
    bool     is_novel        = false;   // quant_error above novelty threshold
    bool     just_baked      = false;   // winner crossed baking gate this tick
    bool     just_pruned     = false;   // a node was pruned this tick
    bool     just_mitosis    = false;   // a baked node was split this tick
    std::vector<int> pruned_ids;        // IDs of nodes killed this tick
    int      node_count      = 0;       // current GNG size
    int      baked_count     = 0;       // baked nodes
    int      mitosis_count   = 0;       // cumulative splits performed
    Eigen::VectorXf latent;             // encoder output (projection_dim D)
};

} // namespace v3
} // namespace ami_ogma
