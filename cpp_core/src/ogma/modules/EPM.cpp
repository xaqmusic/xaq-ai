#include "ogma/modules/EPM.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename T>
T clamp01(T v) { return std::clamp(v, T{0}, T{1}); }

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("EPM param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("EPM param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("EPM param '" + key + "' must be bool");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("EPM param '" + key + "' must be string");
}
std::vector<double> get_doubles_or_empty(ParamValue const& v) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    return {};
}

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

EPM::EncoderKind parse_encoder_kind(std::string const& s) {
    if (s == "jl"      ) return EPM::EncoderKind::JL;
    if (s == "stft" || s == "audio") return EPM::EncoderKind::STFT;
    if (s == "rbf"     ) return EPM::EncoderKind::RBF;
    if (s == "identity") return EPM::EncoderKind::Identity;
    throw std::invalid_argument("EPM: unknown encoder_kind '" + s + "' (expected jl/stft/rbf/identity)");
}

} // namespace

EPM::EPM()  = default;
EPM::~EPM() = default;

std::string_view EPM::type_name() const { return "EPM"; }

std::vector<TopicSpec> EPM::input_topics() const {
    std::vector<TopicSpec> specs;
    // The single input topic varies by encoder kind.
    switch (encoder_kind_) {
        case EncoderKind::JL:
            specs.push_back(TopicSpec{input_topic_, std::type_index(typeid(RawImageFrame))});
            break;
        case EncoderKind::STFT:
            specs.push_back(TopicSpec{input_topic_, std::type_index(typeid(RawAudioFrame))});
            break;
        case EncoderKind::RBF:
            specs.push_back(TopicSpec{input_topic_, std::type_index(typeid(ProprioToken))});
            break;
        case EncoderKind::Identity:
            // Either a RealityToken (Level-N stacking) or ConsensusToken.
            // We declare the union as RealityToken; the handler dynamic_casts to the
            // type that arrives.
            specs.push_back(TopicSpec{input_topic_, std::type_index(typeid(RealityToken))});
            break;
    }
    specs.push_back(TopicSpec{topics::kNeuroState,
                              std::type_index(typeid(NeuroState)),
                              SubscriptionKind::Direct, /*required=*/false});
    if (subtract_descending_prediction_) {
        specs.push_back(TopicSpec{std::string("prediction.") + modality_group_ + "." + modality_name_,
                                  std::type_index(typeid(PredictionToken)),
                                  SubscriptionKind::Feedback, /*required=*/false});
    }
    return specs;
}

std::vector<TopicSpec> EPM::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(RealityToken))} };
}

ParamSchema EPM::params_schema() const {
    return {
        {"modality_group", ParamMutability::ConstructionOnly, "video|audio|proprio|consensus", std::nullopt},
        {"modality_name",  ParamMutability::ConstructionOnly, "Trailing component of output topic", std::nullopt},
        {"encoder_kind",   ParamMutability::ConstructionOnly, "jl|stft|rbf|identity", std::nullopt},
        {"input_topic",    ParamMutability::ConstructionOnly, "Bus topic to subscribe", std::nullopt},
        {"projection_dim", ParamMutability::ConstructionOnly, "GNG input dim.  When OMITTED and `proprio_state_dims` is provided (RBF encoder), derived as max(48, 8 * proprio_state_dims) so the GNG always has enough random-projection capacity for its input to spread into distinguishable clusters.  Empirical floor at pd=48 + per-dim allowance of 8x — phase 7.2-EPM Stage 2 showed pd=24 with 3-D input collapses cluster discrimination (chassis_y -50%, falls 10x), while pd=48 recovers.  Explicit values in config still honoured.", ParamValue{int64_t{128}}},
        {"baking_threshold",        ParamMutability::HotMutable, "GNG baking visit count",      ParamValue{int64_t{50}}},
        {"min_insertion_error",     ParamMutability::HotMutable, "GNG min_insertion_error",     ParamValue{0.02}},
        {"lambda_new",              ParamMutability::HotMutable, "GNG lambda_new",              ParamValue{int64_t{25}}},
        {"max_age",                 ParamMutability::HotMutable, "GNG edge max_age",            ParamValue{int64_t{88}}},
        {"epsilon_b",               ParamMutability::HotMutable, "GNG winner LR",               ParamValue{0.05}},
        {"epsilon_n",               ParamMutability::HotMutable, "GNG neighbour LR",            ParamValue{0.003}},
        {"alpha",                   ParamMutability::HotMutable, "Error halving on insert",     ParamValue{0.5}},
        {"beta",                    ParamMutability::HotMutable, "Global error decay",          ParamValue{0.0005}},
        {"max_nodes",               ParamMutability::HotMutable, "GNG max_nodes",               ParamValue{int64_t{2000}}},
        {"tle_alpha",               ParamMutability::HotMutable, "Weight of QE in dual TLE",    ParamValue{0.7}},
        {"tle_beta",                ParamMutability::HotMutable, "Weight of TS in dual TLE",    ParamValue{0.3}},
        {"tle_ema_alpha",           ParamMutability::HotMutable, "TLE EMA decay",               ParamValue{0.05}},
        {"novelty_threshold_multiplier", ParamMutability::HotMutable, "EMA scale for novelty",  ParamValue{1.5}},
        {"novelty_floor",           ParamMutability::HotMutable, "Min novelty threshold",       ParamValue{0.01}},
        {"history_trace_size",      ParamMutability::HotMutable, "Rolling winner trace length", ParamValue{int64_t{5}}},
        {"predicted_pathway_steps", ParamMutability::HotMutable, "Forward-rollout length emitted in token (0=off)", ParamValue{int64_t{0}}},
        {"process_every_n_ticks",   ParamMutability::HotMutable, "Phase v5.2 sub-rate processing.  When > 1, EPM skips encoder + GNG step on (N-1) of every N ticks; on skipped ticks it republishes its last RealityToken with the current tick_id so downstream voters see a continuous contribution.  Default 1 = process every tick (bit-identical to legacy).", ParamValue{int64_t{1}}},
        {"mitosis_enabled",         ParamMutability::HotMutable, "GNG mitosis on/off",          ParamValue{true}},
        {"mitosis_error_threshold", ParamMutability::HotMutable, "Post-bake mean error trigger", ParamValue{0.30}},
        {"mitosis_check_interval",  ParamMutability::HotMutable, "Visits between mitosis checks", ParamValue{int64_t{50}}},
        {"stale_prune_enabled",     ParamMutability::HotMutable, "GNG stale-prune",             ParamValue{true}},
        {"health_death_spares_baked", ParamMutability::HotMutable,
         "Exempt BAKED nodes from the GNG health-death sweep (2026-09-01).  The health system "
         "silently removed baked-immunity: a long perturbation starves an earned node of "
         "visits and its health decays to death in minutes — the operator's observed "
         "prune-then-relearn cascade on the picrawler, and measured on the microduck regime "
         "EPM as 25/41 node ids dead in 50 min with the standing regime's identity churning "
         "1→16→29 (orphaning every consumer keyed on winner_id).  For a REGIME vocabulary — "
         "permanent facts about the body — earned nodes should not be forgotten for a long "
         "absence.  false = legacy, byte-identical.", ParamValue{false}},
        {"stale_window_factor",     ParamMutability::HotMutable, "Stale prune window",          ParamValue{12000.0}},
        {"subtract_descending_prediction", ParamMutability::HotMutable, "Subtract prediction.<m>", ParamValue{true}},
        {"normalize_residual", ParamMutability::ConstructionOnly,
         "B v2 (2026-08-14): running-RMS normalize the post-subtraction residual before the GNG, "
         "so the vocabulary tiles the residual's DIRECTION at unit scale instead of collapsing on "
         "its (small) absolute size — §6's insertion-gate collapse, measured on the motor path "
         "(stronger predictor → vocab 41→25).  Only meaningful with subtract_descending_prediction. "
         "false = byte-identical (B v1).", ParamValue{false}},
        {"sample_rate",             ParamMutability::ConstructionOnly, "STFT sample rate",      ParamValue{int64_t{48000}}},
        {"f_min",                   ParamMutability::ConstructionOnly, "STFT f_min",            ParamValue{80.0}},
        {"f_max",                   ParamMutability::ConstructionOnly, "STFT f_max",            ParamValue{8000.0}},
        {"proprio_state_dims",      ParamMutability::ConstructionOnly, "RBF state_dims",        ParamValue{int64_t{22}}},
        {"dim_min",                 ParamMutability::ConstructionOnly, "RBF per-dim normalisation MIN (vector, len=proprio_state_dims). With dim_max, maps each input dim to [0,1] over its ACTUAL range so small-magnitude sensors (e.g. vision loom ~0.09) aren't washed out by the default [-1,1].  Omitted → [-1,1].", std::nullopt},
        {"dim_max",                 ParamMutability::ConstructionOnly, "RBF per-dim normalisation MAX (vector, len=proprio_state_dims).", std::nullopt},
        {"master_seed",             ParamMutability::ConstructionOnly, "RNG namespace seed",    ParamValue{int64_t{0}}},
        {"insertion_autotune",      ParamMutability::ConstructionOnly,
            "ECOLOGICAL SELF-TUNING OF THE INSERTION GATE.  `min_insertion_error` is an ABSOLUTE threshold and only means something relative to the typical quantisation error of the signal in front of it; held fixed on a stream whose typical error exceeds it, insertion never stops being justified and the GNG grows until it hits max_nodes.  When on, the GNG sets its own gate from the `insertion_autotune_quantile`-th percentile of its OWN recent squared-TLE distribution, and the configured `min_insertion_error` becomes the FLOOR it always claimed to be: effective = max(configured, quantile) * neuro_scale.  Restores behaviour that existed in the v3 Python reference and was lost in the C++ port.  false (default) = the fixed-threshold path, byte-identical.",
            ParamValue{false}},
        {"insertion_autotune_quantile", ParamMutability::ConstructionOnly,
            "Percentile of the GNG's own recent squared-TLE distribution used as the insertion floor.  A RANK, not a scale: dimensionless and invariant to the signal's units, which is what makes this adaptive rather than another constant tuned to a signal's magnitude.  0.30 matches the v3 reference.",
            ParamValue{0.30}},
        {"dim_autocal_ticks",       ParamMutability::ConstructionOnly,
            "COMMISSIONING WINDOW, in input frames.  When > 0, the EPM measures its own per-dim input ranges over the first N frames instead of being told them, then installs them and RESETS the GNG topology so the vocabulary is re-earned in the calibrated space.  This is the adaptive form of `dim_min`/`dim_max`: §0 rule 2 requires the input be conditioned before discretisation, and a hand-measured constant per sensor is the smell that names a missing mechanism.  Window length is legitimately application-set — it must cover the body's characteristic motion (several stride cycles for a gait, a full sweep for a sensor), because a range set by a startup transient is worse than the default.  Mutually exclusive with explicit dim_min/dim_max (throws).  RBF encoder only (throws for jl/stft, whose dims are homogeneous pixels/samples and must not be rescaled per-dim).  0 = off, byte-identical.",
            ParamValue{int64_t{0}}},
        {"dim_autocal_k",           ParamMutability::ConstructionOnly,
            "Sigma multiplier for the commissioning range: per dim, range = intersect(mu +/- k*sigma, [min_obs, max_obs]).  Larger k = wider range, less clamping, coarser resolution.  Values outside the installed range saturate (normalise_state clamps), so under-covering the tails is lossy but never unstable.",
            ParamValue{4.0}},
    };
}

void EPM::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("EPM requires a non-null Bus");

    // Required params.
    auto find_required = [&](std::string const& key) -> ParamValue const& {
        auto it = params.find(key);
        if (it == params.end())
            throw std::invalid_argument("EPM: required param '" + key + "' missing");
        return it->second;
    };
    modality_group_      = get_string(find_required("modality_group"), "modality_group");
    modality_name_       = get_string(find_required("modality_name"),  "modality_name");
    std::string ek_str   = get_string(find_required("encoder_kind"),   "encoder_kind");
    encoder_kind_        = parse_encoder_kind(ek_str);
    input_topic_         = get_string(find_required("input_topic"),    "input_topic");

    // Optional params with defaults.
    //
    // projection_dim auto-derivation: when the config omits projection_dim
    // but provides proprio_state_dims, derive pd = max(48, 8 * input_dim).
    // Rationale: in Phase 7.2-EPM Stage 2, pd=24 with 3-D input failed
    // hard (chassis_y -50% vs baseline, falls 10x, brain learning halted)
    // because the GNG had insufficient random-projection capacity to
    // cluster the 3-D manifold into distinguishable states.  pd=48
    // recovered.  The "8x per-input-dim + absolute floor 48" rule fits
    // every empirically-working config without hand-tuning.
    bool projection_dim_explicit = false;
    int  proprio_state_dims_seen = 22;   // matches schema default for RBF
    bool proprio_state_dims_seen_flag = false;
    apply_param(params, "projection_dim", [&](auto const& v){
        projection_dim_ = int(get_int(v, "projection_dim"));
        projection_dim_explicit = true;
    });
    apply_param(params, "proprio_state_dims", [&](auto const& v){
        proprio_state_dims_seen = int(get_int(v, "proprio_state_dims"));
        proprio_state_dims_seen_flag = true;
    });
    apply_param(params, "tle_alpha",      [&](auto const& v){ tle_alpha_      = float(get_double(v, "tle_alpha")); });
    apply_param(params, "tle_beta",       [&](auto const& v){ tle_beta_       = float(get_double(v, "tle_beta")); });
    apply_param(params, "tle_ema_alpha",  [&](auto const& v){ tle_ema_alpha_  = float(get_double(v, "tle_ema_alpha")); });
    apply_param(params, "novelty_threshold_multiplier", [&](auto const& v){ novelty_threshold_multiplier_ = float(get_double(v, "novelty_threshold_multiplier")); });
    apply_param(params, "novelty_floor",  [&](auto const& v){ novelty_floor_  = float(get_double(v, "novelty_floor")); });
    apply_param(params, "history_trace_size", [&](auto const& v){ history_trace_size_ = int(get_int(v, "history_trace_size")); });
    apply_param(params, "predicted_pathway_steps", [&](auto const& v){ predicted_pathway_steps_ = int(get_int(v, "predicted_pathway_steps")); });
    apply_param(params, "process_every_n_ticks",   [&](auto const& v){ process_every_n_ticks_   = std::max(1, int(get_int(v, "process_every_n_ticks"))); });
    apply_param(params, "subtract_descending_prediction", [&](auto const& v){ subtract_descending_prediction_ = get_bool(v, "subtract_descending_prediction"); });
    apply_param(params, "normalize_residual", [&](auto const& v){ normalize_residual_ = get_bool(v, "normalize_residual"); });
    apply_param(params, "master_seed",    [&](auto const& v){ master_seed_    = uint64_t(get_int(v, "master_seed")); });
    apply_param(params, "dim_autocal_ticks", [&](auto const& v){ dim_autocal_ticks_ = std::max(0, int(get_int(v, "dim_autocal_ticks"))); });
    apply_param(params, "dim_autocal_k",     [&](auto const& v){ dim_autocal_k_     = get_double(v, "dim_autocal_k"); });

    // Output topic name.
    if (modality_group_ == "consensus") {
        output_topic_ = std::string("consensus.") + modality_name_;
    } else {
        output_topic_ = std::string("reality.") + modality_group_ + "." + modality_name_;
    }

    // projection_dim auto-derivation (see comment block above).  Only
    // triggers for RBF encoder + omitted projection_dim + explicit
    // proprio_state_dims.  Other encoder kinds (jl, stft, identity) keep
    // the schema default since their input-dim semantics differ.
    if (!projection_dim_explicit
        && proprio_state_dims_seen_flag
        && encoder_kind_ == EncoderKind::RBF) {
        int derived = std::max(48, 8 * proprio_state_dims_seen);
        if (derived != projection_dim_) {
            std::cerr << "EPM[" << output_topic_
                      << "]: projection_dim auto-derived = " << derived
                      << " (proprio_state_dims=" << proprio_state_dims_seen
                      << ", rule = max(48, 8 × input_dim))" << std::endl;
            projection_dim_ = derived;
        }
    }

    // GNG configuration.
    ami_ogma::v3::GNG::Config gng_cfg;
    gng_cfg.dim                 = projection_dim_;
    apply_param(params, "baking_threshold",        [&](auto const& v){ gng_cfg.baking_threshold        = int(get_int(v, "baking_threshold")); });
    apply_param(params, "min_insertion_error",     [&](auto const& v){ gng_cfg.min_insertion_error     = float(get_double(v, "min_insertion_error")); });
    apply_param(params, "lambda_new",              [&](auto const& v){ gng_cfg.lambda_new              = int(get_int(v, "lambda_new")); });
    apply_param(params, "max_age",                 [&](auto const& v){ gng_cfg.max_age                 = int(get_int(v, "max_age")); });
    apply_param(params, "epsilon_b",               [&](auto const& v){ gng_cfg.epsilon_b               = float(get_double(v, "epsilon_b")); });
    apply_param(params, "epsilon_n",               [&](auto const& v){ gng_cfg.epsilon_n               = float(get_double(v, "epsilon_n")); });
    apply_param(params, "alpha",                   [&](auto const& v){ gng_cfg.alpha                   = float(get_double(v, "alpha")); });
    apply_param(params, "beta",                    [&](auto const& v){ gng_cfg.beta                    = float(get_double(v, "beta")); });
    apply_param(params, "max_nodes",               [&](auto const& v){ gng_cfg.max_nodes               = int(get_int(v, "max_nodes")); });
    apply_param(params, "mitosis_enabled",         [&](auto const& v){ gng_cfg.mitosis_enabled         = get_bool(v, "mitosis_enabled"); });
    apply_param(params, "mitosis_error_threshold", [&](auto const& v){ gng_cfg.mitosis_error_threshold = float(get_double(v, "mitosis_error_threshold")); });
    apply_param(params, "mitosis_check_interval",  [&](auto const& v){ gng_cfg.mitosis_check_interval  = int(get_int(v, "mitosis_check_interval")); });
    apply_param(params, "stale_prune_enabled",     [&](auto const& v){ gng_cfg.stale_prune_enabled     = get_bool(v, "stale_prune_enabled"); });
    apply_param(params, "health_death_spares_baked", [&](auto const& v){ gng_cfg.health_death_spares_baked = get_bool(v, "health_death_spares_baked"); });
    apply_param(params, "stale_window_factor",     [&](auto const& v){ gng_cfg.stale_window_factor     = float(get_double(v, "stale_window_factor")); });
    apply_param(params, "insertion_autotune",          [&](auto const& v){ gng_cfg.insertion_autotune          = get_bool(v, "insertion_autotune"); });
    apply_param(params, "insertion_autotune_quantile", [&](auto const& v){ gng_cfg.insertion_autotune_quantile = float(get_double(v, "insertion_autotune_quantile")); });
    insertion_autotune_ = gng_cfg.insertion_autotune;

    base_epsilon_b_               = gng_cfg.epsilon_b;
    base_min_insertion_error_     = gng_cfg.min_insertion_error;
    base_mitosis_error_threshold_ = gng_cfg.mitosis_error_threshold;

    gng_ = std::make_unique<ami_ogma::v3::GNG>(gng_cfg);

    // Encoder construction.
    switch (encoder_kind_) {
        case EncoderKind::JL: {
            int encoder_res = 0;
            apply_param(params, "encoder_res", [&](auto const& v){ encoder_res = int(get_int(v, "encoder_res")); });
            enc_jl_ = std::make_unique<ami_ogma::v3::FrozenJLEncoder>(
                modality_name_, projection_dim_, encoder_res, /*inject_centroid=*/false, 22.6f);
            break;
        }
        case EncoderKind::STFT: {
            ami_ogma::v3::FrozenSTFTEncoder::Config cfg;
            cfg.n_filters = projection_dim_;
            apply_param(params, "sample_rate",  [&](auto const& v){ cfg.sample_rate = int(get_int(v, "sample_rate")); });
            apply_param(params, "f_min",        [&](auto const& v){ cfg.f_min       = float(get_double(v, "f_min")); });
            apply_param(params, "f_max",        [&](auto const& v){ cfg.f_max       = float(get_double(v, "f_max")); });
            enc_audio_ = std::make_unique<ami_ogma::v3::FrozenSTFTEncoder>(cfg);
            break;
        }
        case EncoderKind::RBF: {
            ami_ogma::v3::FrozenRBFEncoder::Config cfg;
            cfg.projection_dim = projection_dim_;
            apply_param(params, "proprio_state_dims", [&](auto const& v){ cfg.state_dims = int(get_int(v, "proprio_state_dims")); });
            // Per-dim normalisation ranges so any-scale sensors map cleanly to
            // [0,1] (default [-1,1] washes out small signals like vision loom).
            {
                std::vector<double> dmin, dmax;
                apply_param(params, "dim_min", [&](auto const& v){ if (auto p = std::get_if<std::vector<double>>(&v)) dmin = *p; });
                apply_param(params, "dim_max", [&](auto const& v){ if (auto p = std::get_if<std::vector<double>>(&v)) dmax = *p; });
                if (!dmin.empty() || !dmax.empty()) {
                    // Contradiction, not a precedence question: one says "I know
                    // the ranges", the other says "measure them".  Silently
                    // picking a winner is how a config means something other
                    // than what it reads like.
                    if (dim_autocal_ticks_ > 0)
                        throw std::invalid_argument(
                            "EPM: dim_autocal_ticks and explicit dim_min/dim_max are mutually exclusive — "
                            "either state the ranges or commission them, not both");
                    if (dmin.size() != size_t(cfg.state_dims) || dmax.size() != size_t(cfg.state_dims))
                        throw std::invalid_argument("EPM: dim_min/dim_max length must equal proprio_state_dims");
                    cfg.dim_ranges.clear();
                    for (int i = 0; i < cfg.state_dims; ++i)
                        cfg.dim_ranges.emplace_back(float(dmin[i]), float(dmax[i]));
                }
            }
            enc_rbf_ = std::make_unique<ami_ogma::v3::FrozenRBFEncoder>(cfg);
            if (dim_autocal_ticks_ > 0) {
                const size_t n = size_t(cfg.state_dims);
                dac_min_.assign(n,  std::numeric_limits<double>::max());
                dac_max_.assign(n, -std::numeric_limits<double>::max());
                dac_sum_.assign(n, 0.0);
                dac_sumsq_.assign(n, 0.0);
            }
            break;
        }
        case EncoderKind::Identity:
            // No encoder.  GNG receives input vectors directly.
            break;
    }

    // Commissioning is defined for encoders whose input dims are HETEROGENEOUS
    // sensor channels.  JL and STFT consume homogeneous pixels/samples, where a
    // per-dim rescale would destroy the very structure the frozen encoder exists
    // to preserve.  Identity is deferred, not refused: stacked latents plausibly
    // want it, but nothing has measured a need, and a mechanism enabled on a
    // guess is one nobody can later attribute a result to.
    if (dim_autocal_ticks_ > 0 && encoder_kind_ != EncoderKind::RBF) {
        throw std::invalid_argument(
            "EPM: dim_autocal_ticks is supported for encoder_kind='rbf' only (got '" + ek_str + "')");
    }

    // Subscribe to the input topic.
    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_input(t, p); }));

    // Subscribe to neuro.state for current-tick scaling factors.
    sub_ids_.push_back(bus_->subscribe(topics::kNeuroState, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_neuro(t, p); }));

    // Subscribe to descending prediction (Feedback — prior-tick read).
    if (subtract_descending_prediction_) {
        std::string pred_topic = std::string("prediction.") + modality_group_ + "." + modality_name_;
        sub_ids_.push_back(bus_->subscribe(pred_topic, SubscriptionKind::Feedback,
            [this](auto t, auto p){ this->handle_prediction(t, p); }));
    }
}

void EPM::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if (k == "tle_alpha")             tle_alpha_          = float(get_double(value, k));
    else if (k == "tle_beta")         tle_beta_           = float(get_double(value, k));
    else if (k == "tle_ema_alpha")    tle_ema_alpha_      = float(get_double(value, k));
    else if (k == "novelty_threshold_multiplier") novelty_threshold_multiplier_ = float(get_double(value, k));
    else if (k == "novelty_floor")    novelty_floor_      = float(get_double(value, k));
    else if (k == "history_trace_size") history_trace_size_ = int(get_int(value, k));
    else if (k == "predicted_pathway_steps") predicted_pathway_steps_ = int(get_int(value, k));
    else if (k == "process_every_n_ticks")   process_every_n_ticks_   = std::max(1, int(get_int(value, k)));
    else if (k == "subtract_descending_prediction") subtract_descending_prediction_ = get_bool(value, k);
    // GNG hot-mutable params route into the underlying GNG.
    else if (k == "epsilon_b")        { base_epsilon_b_ = float(get_double(value, k)); gng_->set_epsilon_b(base_epsilon_b_ * epsilon_b_scale_); }
    else if (k == "epsilon_n")        gng_->set_epsilon_n(float(get_double(value, k)));
    else if (k == "min_insertion_error") { base_min_insertion_error_ = float(get_double(value, k)); gng_->set_min_insertion_error(base_min_insertion_error_ * min_insertion_error_scale_); }
    else if (k == "baking_threshold") gng_->set_baking_threshold(int(get_int(value, k)));
    else if (k == "max_age")          gng_->set_max_age(int(get_int(value, k)));
    else if (k == "lambda_new")       gng_->set_lambda_new(int(get_int(value, k)));
    else if (k == "mitosis_enabled")  gng_->set_mitosis_enabled(get_bool(value, k));
    else if (k == "mitosis_error_threshold") { base_mitosis_error_threshold_ = float(get_double(value, k)); gng_->set_mitosis_error_threshold(base_mitosis_error_threshold_ * mitosis_threshold_scale_); }
    else if (k == "mitosis_check_interval")  gng_->set_mitosis_check_interval(int(get_int(value, k)));
    else if (k == "stale_prune_enabled")     gng_->set_stale_prune_enabled(get_bool(value, k));
    else if (k == "health_death_spares_baked") gng_->set_health_death_spares_baked(get_bool(value, k));
    else if (k == "stale_window_factor")     gng_->set_stale_window_factor(float(get_double(value, k)));
    else if (k == "modality_group" || k == "modality_name" || k == "encoder_kind"
          || k == "input_topic"    || k == "projection_dim" || k == "master_seed"
          || k == "sample_rate"    || k == "f_min" || k == "f_max"
          || k == "proprio_state_dims" || k == "dim_min" || k == "dim_max")
        throw std::invalid_argument("EPM param '" + k + "' is ConstructionOnly");
    else
        throw std::invalid_argument("EPM: unknown param '" + k + "'");
}

void EPM::handle_input(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    switch (encoder_kind_) {
        case EncoderKind::JL:
            pending_image_   = std::dynamic_pointer_cast<const RawImageFrame>(payload);
            break;
        case EncoderKind::STFT:
            pending_audio_   = std::dynamic_pointer_cast<const RawAudioFrame>(payload);
            break;
        case EncoderKind::RBF:
            pending_proprio_ = std::dynamic_pointer_cast<const ProprioToken>(payload);
            break;
        case EncoderKind::Identity:
            // Could be a RealityToken (Level-N) or a ConsensusToken.
            if (auto rt = std::dynamic_pointer_cast<const RealityToken>(payload)) {
                pending_reality_ = rt;
            } else if (auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload)) {
                pending_consensus_ = ct;
            }
            break;
    }
}

void EPM::handle_neuro(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto n = std::dynamic_pointer_cast<const NeuroState>(payload);
    if (!n) return;
    epsilon_b_scale_           = n->epsilon_b_scale;
    min_insertion_error_scale_ = n->min_insertion_error_scale;
    mitosis_threshold_scale_   = n->mitosis_threshold_scale;
    novelty_threshold_scale_   = n->novelty_threshold_scale;
}

void EPM::handle_prediction(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    pending_prediction_ = std::dynamic_pointer_cast<const PredictionToken>(payload);
}

// ---------------------------------------------------------------------------
// Commissioning window (dim_autocal_ticks) — see EPM.hpp for the rationale
// ---------------------------------------------------------------------------

void EPM::dim_autocal_observe(const float* v, int n) {
    if (int(dac_min_.size()) != n) return;   // dim mismatch: nothing to learn from
    for (int d = 0; d < n; ++d) {
        const double x = double(v[d]);
        dac_min_[d]    = std::min(dac_min_[d], x);
        dac_max_[d]    = std::max(dac_max_[d], x);
        dac_sum_[d]   += x;
        dac_sumsq_[d] += x * x;
    }
    ++dim_autocal_seen_;
}

void EPM::dim_autocal_finalise() {
    const int n      = int(dac_min_.size());
    const double cnt = double(dim_autocal_seen_);
    if (n == 0 || cnt <= 0.0) { dim_autocal_done_ = true; return; }

    std::vector<std::pair<float,float>> ranges;
    ranges.reserve(size_t(n));
    int degenerate = 0;
    for (int d = 0; d < n; ++d) {
        const double mu  = dac_sum_[d] / cnt;
        const double var = std::max(0.0, dac_sumsq_[d] / cnt - mu * mu);
        const double sd  = std::sqrt(var);
        // Intersect the statistical range with what was actually observed:
        // mu +/- k*sd rejects transients, min/max refuses to invent range.
        double lo = std::max(dac_min_[d], mu - dim_autocal_k_ * sd);
        double hi = std::min(dac_max_[d], mu + dim_autocal_k_ * sd);
        // A channel that never moved carries no information at any scale.
        // Fall back to the encoder default rather than dividing by ~0.
        if (!(hi - lo > 1e-6)) { lo = -1.0; hi = 1.0; ++degenerate; }
        ranges.emplace_back(float(lo), float(hi));
    }

    enc_rbf_->set_dim_ranges(ranges);

    // MANDATORY: every prototype learned during the window is expressed in the
    // provisional units and is garbage in the new space.  Node IDs deliberately
    // continue from where they were (Invariant 4).
    gng_->reset_topology();
    prev_winner_prototype_ = Eigen::VectorXf();
    has_prev_prototype_    = false;
    prev_winner_id_for_transitions_ = -1;
    transition_counts_.clear();
    winner_counts_.clear();
    history_trace_.clear();
    ema_tle_               = 0.1f;      // the constructed default, not a learned value
    last_tle_              = 0.0f;
    last_quant_error_      = 0.0f;
    last_published_token_.reset();

    dim_autocal_done_ = true;

    std::printf("{\"EPM_AUTOCAL\":{\"id\":\"%s\",\"topic\":\"%s\",\"frames\":%llu,\"k\":%.2f,"
                "\"degenerate_dims\":%d,\"ranges\":[",
                id_.c_str(), output_topic_.c_str(),
                (unsigned long long)dim_autocal_seen_, dim_autocal_k_, degenerate);
    for (int d = 0; d < n; ++d)
        std::printf("%s[%.5f,%.5f]", d ? "," : "", double(ranges[size_t(d)].first),
                                                   double(ranges[size_t(d)].second));
    std::printf("]}}\n");
    std::fflush(stdout);
}

bool EPM::encode_pending_input(Eigen::VectorXf& out) {
    switch (encoder_kind_) {
        case EncoderKind::JL: {
            if (!pending_image_ || pending_image_->pixels.empty()) return false;
            out = enc_jl_->encode(pending_image_->pixels.data(),
                                  pending_image_->height,
                                  pending_image_->width,
                                  pending_image_->channels);
            return out.size() == projection_dim_;
        }
        case EncoderKind::STFT: {
            if (!pending_audio_ || pending_audio_->samples.empty()) return false;
            if (pending_audio_->channels == 2 && int(pending_audio_->samples.size()) >= 2 * pending_audio_->n_samples) {
                std::vector<float> left(pending_audio_->n_samples);
                std::vector<float> right(pending_audio_->n_samples);
                for (int i = 0; i < pending_audio_->n_samples; ++i) {
                    left[i]  = pending_audio_->samples[2*i + 0];
                    right[i] = pending_audio_->samples[2*i + 1];
                }
                out = enc_audio_->encode_stereo(left.data(), right.data(), pending_audio_->n_samples);
            } else {
                out = enc_audio_->encode_mono(pending_audio_->samples.data(), pending_audio_->n_samples);
            }
            return out.size() == projection_dim_;
        }
        case EncoderKind::RBF: {
            if (!pending_proprio_ || pending_proprio_->values.size() == 0) return false;
            out = enc_rbf_->encode(pending_proprio_->values.data(), int(pending_proprio_->values.size()));
            return out.size() == projection_dim_;
        }
        case EncoderKind::Identity: {
            if (pending_reality_ && pending_reality_->latent.size() == projection_dim_) {
                out = pending_reality_->latent;
                return true;
            }
            if (pending_consensus_ && pending_consensus_->fused_embedding.size() == projection_dim_) {
                out = pending_consensus_->fused_embedding;
                return true;
            }
            return false;
        }
    }
    return false;
}

void EPM::apply_neuro_scaling() {
    if (!gng_) return;
    gng_->set_epsilon_b(              base_epsilon_b_           * epsilon_b_scale_);
    // ⚠ THE SCALE IS APPLIED IN TWO DIFFERENT PLACES ON PURPOSE.
    //
    // Legacy path (autotune off): fold the neurochemical scale into the value
    // handed to the GNG, exactly as before — that arithmetic must stay
    // bit-identical or every existing config moves.
    //
    // Autotune path: hand the GNG the UNSCALED configured floor and pass the
    // scale separately, so the GNG applies it to max(floor, quantile).  The
    // scale then modulates growth relative to the body's CURRENT typical
    // surprise rather than relative to a fixed constant, which is the only
    // reading of "min_insertion_error_scale" that still means something once
    // the base is no longer fixed.
    if (insertion_autotune_) {
        gng_->set_min_insertion_error(base_min_insertion_error_);
        gng_->set_neuro_min_insertion_scale(min_insertion_error_scale_);
    } else {
        gng_->set_min_insertion_error(base_min_insertion_error_ * min_insertion_error_scale_);
    }
    gng_->set_mitosis_error_threshold(base_mitosis_error_threshold_ * mitosis_threshold_scale_);
}

void EPM::compute_dual_tle(float quant_error, int winner_id,
                           float& transition_surp_out, float& tle_out) {
    transition_surp_out = 0.0f;

    auto winner_proto = gng_->get_prototype(winner_id);
    if (winner_proto.has_value() && has_prev_prototype_ &&
        winner_proto->size() == prev_winner_prototype_.size()) {
        transition_surp_out = (winner_proto.value() - prev_winner_prototype_).norm();
    }
    if (winner_proto.has_value()) {
        prev_winner_prototype_ = winner_proto.value();
        has_prev_prototype_    = true;
    }

    tle_out = tle_alpha_ * quant_error + tle_beta_ * transition_surp_out;
    if (std::isnan(tle_out)) tle_out = 0.0f;

    // Update EMA & adaptive novelty threshold.
    ema_tle_ = (1.0f - tle_ema_alpha_) * ema_tle_ + tle_ema_alpha_ * tle_out;
    novelty_threshold_now_ = std::max(novelty_floor_,
                                      ema_tle_ * novelty_threshold_multiplier_ * novelty_threshold_scale_);
}

void EPM::publish_token(uint64_t tick_id,
                        int      winner_id,
                        float    quant_error,
                        float    transition_surp,
                        float    tle,
                        Eigen::VectorXf const& latent) {
    auto tok = std::make_shared<RealityToken>();
    tok->tick_id           = tick_id;
    tok->producer_id       = id_.empty() ? std::string("epm") : id_;
    tok->winner_id         = winner_id;
    tok->quant_error       = quant_error;
    tok->transition_surp   = transition_surp;
    tok->tle               = tle;
    tok->novelty_threshold = novelty_threshold_now_;
    tok->is_novel          = quant_error > novelty_threshold_now_;
    tok->just_baked        = gng_->last_step_baked();
    tok->just_pruned       = !gng_->last_pruned_ids().empty();
    tok->just_mitosis      = false; // TODO: gng_ doesn't expose a per-step flag yet
    tok->pruned_ids        = gng_->last_pruned_ids();
    tok->node_count        = gng_->node_count();
    tok->baked_count       = gng_->baked_count();
    tok->mitosis_count     = gng_->mitosis_count();
    tok->history_trace.assign(history_trace_.begin(), history_trace_.end());
    tok->latent            = latent;

    if (winner_id >= 0) {
        if (auto wp = gng_->get_prototype(winner_id)) tok->winner_prototype = *wp;
    }

    // Phase 6.6.E: optional forward rollout via greedy argmax over
    // transition_counts_.  We walk at most predicted_pathway_steps_ steps
    // and stop early at any node with no recorded successors (cold spot).
    if (predicted_pathway_steps_ > 0 && winner_id >= 0) {
        tok->predicted_pathway.reserve(predicted_pathway_steps_);
        int cursor = winner_id;
        for (int step = 0; step < predicted_pathway_steps_; ++step) {
            auto it = transition_counts_.find(cursor);
            if (it == transition_counts_.end() || it->second.empty()) break;
            int best_next = -1;
            int best_cnt  = 0;
            for (auto const& [next_id, cnt] : it->second) {
                if (cnt > best_cnt) { best_cnt = cnt; best_next = next_id; }
            }
            if (best_next < 0) break;
            tok->predicted_pathway.push_back(best_next);
            cursor = best_next;
        }
    }

    last_published_token_ = tok;       // Phase v5.2 sub-rate cache
    bus_->publish(output_topic_, tok);
}

void EPM::publish_bootstrap_token(uint64_t tick_id) {
    Eigen::VectorXf zero = Eigen::VectorXf::Zero(projection_dim_);
    auto tok = std::make_shared<RealityToken>();
    tok->tick_id           = tick_id;
    tok->producer_id       = id_.empty() ? std::string("epm") : id_;
    tok->winner_id         = -1;
    tok->latent            = zero;
    tok->history_trace.assign(history_trace_.begin(), history_trace_.end());
    tok->novelty_threshold = novelty_floor_;
    // Preserve GNG state in the no-input token so downstream metrics
    // (and replay-style consumers) don't see node_count flicker to 0
    // every tick the host hasn't published a fresh frame.  Sub-rate
    // sensors (e.g. vision sampled every Nth tick) need this to keep
    // the live picture coherent.
    tok->node_count        = gng_ ? gng_->node_count() : 0;
    tok->baked_count       = gng_ ? gng_->baked_count() : 0;
    bus_->publish(output_topic_, tok);
}

void EPM::tick(uint64_t tick_id) {
    // Phase v5.2 — sub-rate gate.  When process_every_n_ticks_ > 1, only
    // run the encoder + GNG step on every Nth tick.  On skipped ticks,
    // republish the cached last token with the new tick_id so the voter
    // sees a continuous contribution (same pattern as LateralVoter
    // Invariant 7).  Pending inputs are still drained on every tick so
    // a stale payload doesn't leak into the next process tick.
    if (process_every_n_ticks_ > 1
        && (sub_rate_tick_count_ % process_every_n_ticks_) != 0) {
        ++sub_rate_tick_count_;
        pending_image_.reset();
        pending_audio_.reset();
        pending_proprio_.reset();
        pending_reality_.reset();
        pending_consensus_.reset();
        if (last_published_token_) {
            auto rep = std::make_shared<RealityToken>(*last_published_token_);
            rep->tick_id = tick_id;
            bus_->publish(output_topic_, rep);
        }
        return;
    }
    ++sub_rate_tick_count_;

    // Commissioning window: observe the RAW input, before conditioning, so the
    // measured range describes the sensor rather than the current mapping of it.
    if (dim_autocal_ticks_ > 0 && !dim_autocal_done_ &&
        pending_proprio_ && pending_proprio_->values.size() > 0) {
        dim_autocal_observe(pending_proprio_->values.data(),
                            int(pending_proprio_->values.size()));
        // Finalise BEFORE encoding this frame, so the window is exactly N frames
        // and frame N+1 is the first one encoded in the calibrated space.  The
        // reset leaves the GNG pre-bootstrap, so this tick publishes the
        // placeholder token — the same shape as any other bootstrap tick.
        if (dim_autocal_seen_ >= uint64_t(dim_autocal_ticks_)) dim_autocal_finalise();
    }

    Eigen::VectorXf latent;
    bool have_input = encode_pending_input(latent);

    // Clear pending inputs so a stale payload doesn't leak into the next tick.
    pending_image_.reset();
    pending_audio_.reset();
    pending_proprio_.reset();
    pending_reality_.reset();
    pending_consensus_.reset();

    if (!have_input) {
        publish_bootstrap_token(tick_id);
        return;
    }

    // Optional descending-prediction subtraction.  Prediction is the *expected*
    // next encoder output for THIS tick; subtracting yields the surprise residual.
    if (subtract_descending_prediction_ &&
        pending_prediction_ &&
        pending_prediction_->predicted_latent.size() == latent.size()) {
        latent.noalias() -= pending_prediction_->predicted_latent;
        // RESIDUAL NORMALIZATION (B v2, 2026-08-14).  A working predictor leaves
        // a residual of norm ~0.1–0.2 against GNG insertion/error scales sized
        // for encoder outputs of norm ~1 — the vocabulary then COLLAPSES instead
        // of tiling the residual's shape (§6's insertion-gate collapse, measured:
        // the STRONGER context arm shrank the vocabulary 41→25).  Scale by the
        // residual's own running RMS (§5: adapt, don't tune) so the GNG tiles
        // DIRECTION at unit scale.  Off = byte-identical (B v1 behavior).
        if (normalize_residual_) {
            const float nrm = latent.norm();
            residual_rms_ = residual_rms_ <= 0.0f
                                ? std::max(nrm, 1e-6f)
                                : 0.99f * residual_rms_ + 0.01f * nrm;
            if (residual_rms_ > 1e-6f) latent *= (1.0f / residual_rms_);
        }
    }

    apply_neuro_scaling();

    auto [winner_id, quant_error] = gng_->step(latent);

    // GNG bootstrap: first ~2 ticks return placeholder (winner_id=0, qe=0).
    // Distinguish bootstrap from real winner by inspecting the GNG step count.
    if (winner_id < 0 || gng_->node_count() < 2) {
        // Not enough nodes yet — emit a placeholder that downstream knows
        // to ignore (winner_id = -1 per the contract).
        publish_bootstrap_token(tick_id);
        return;
    }

    // Update history trace.
    history_trace_.push_back(winner_id);
    while (int(history_trace_.size()) > history_trace_size_) history_trace_.pop_front();

    // Phase 6.6.E: maintain per-node successor counts for forward rollout.
    // Drop entries for any node the GNG just pruned so a stale ID can't
    // surface in a future predicted_pathway.
    if (!gng_->last_pruned_ids().empty()) {
        for (int pid : gng_->last_pruned_ids()) {
            transition_counts_.erase(pid);
            for (auto& [src, succ] : transition_counts_) succ.erase(pid);
        }
        if (prev_winner_id_for_transitions_ >= 0) {
            for (int pid : gng_->last_pruned_ids()) {
                if (prev_winner_id_for_transitions_ == pid) {
                    prev_winner_id_for_transitions_ = -1;
                    break;
                }
            }
        }
    }
    if (prev_winner_id_for_transitions_ >= 0 &&
        prev_winner_id_for_transitions_ != winner_id) {
        ++transition_counts_[prev_winner_id_for_transitions_][winner_id];
    }
    prev_winner_id_for_transitions_ = winner_id;

    float transition_surp = 0.0f, tle = 0.0f;
    compute_dual_tle(quant_error, winner_id, transition_surp, tle);
    last_tle_         = tle;
    last_quant_error_ = quant_error;

    // v5.4.L Diagnostic B — increment winner histogram.  Skip winner_id<0
    // (bootstrap placeholder).  Diagnostic-only: a 1-2 node domination
    // pattern means the GNG saturated and the published latent is
    // wildcard-class regardless of upstream signal variance.
    if (winner_id >= 0) ++winner_counts_[winner_id];

    publish_token(tick_id, winner_id, quant_error, transition_surp, tle, latent);
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------
//
// Pending input messages (pending_image_, pending_audio_, etc.) are NOT
// snapshotted — they're shared_ptrs to inbound bus messages that get
// refreshed on every tick by the body's publishes.  After restore, the
// first body tick repopulates them.
//
// Encoders are reconstructed from the same seed via on_setup → no state
// to snapshot for the JL/STFT/RBF projection matrices themselves.  The
// FrozenJLEncoder's prev_gray_ buffer (used for optical-flow modalities)
// is a known omission — would need migration if EPM is used for optical
// flow with cloning.  For MC/CartPole (RBF encoder, stateless), this is
// not a concern.

nlohmann::json EPM::diag_lite() const {
    nlohmann::json j = {
        {"last_tle",              last_tle_},
        {"ema_tle",               ema_tle_},
        {"last_quant_error",      last_quant_error_},
        {"novelty_threshold_now", novelty_threshold_now_},
    };
    if (gng_) {
        j["nodes"]         = gng_->node_count();
        j["baked"]         = gng_->baked_count();
        j["mitosis_count"] = gng_->mitosis_count();
        j["baked_now"]     = gng_->last_step_baked();   // a node earned its place THIS step
    }
    return j;
}

nlohmann::json EPM::snapshot_state() const {
    nlohmann::json prev_proto = nlohmann::json::array();
    if (has_prev_prototype_)
        for (int i = 0; i < prev_winner_prototype_.size(); ++i)
            prev_proto.push_back(prev_winner_prototype_(i));
    nlohmann::json hist = nlohmann::json::array();
    for (auto v : history_trace_) hist.push_back(v);
    nlohmann::json gng_json = nullptr;
    if (gng_) gng_json = gng_->to_json();
    nlohmann::json trans = nlohmann::json::object();
    for (auto const& [src, succ] : transition_counts_) {
        nlohmann::json m = nlohmann::json::object();
        for (auto const& [dst, cnt] : succ) m[std::to_string(dst)] = cnt;
        trans[std::to_string(src)] = std::move(m);
    }
    nlohmann::json out = nlohmann::json{
        {"version",                       1},
        {"gng",                           gng_json},
        {"prev_winner_prototype",         prev_proto},
        {"has_prev_prototype",            has_prev_prototype_},
        {"ema_tle",                       ema_tle_},
        {"last_tle",                      last_tle_},
        {"last_quant_error",              last_quant_error_},
        {"novelty_threshold_now",         novelty_threshold_now_},
        {"history_trace",                 hist},
        {"transition_counts",             trans},
        {"prev_winner_id_for_transitions", prev_winner_id_for_transitions_},
        {"epsilon_b_scale",               epsilon_b_scale_},
        {"min_insertion_error_scale",     min_insertion_error_scale_},
        {"mitosis_threshold_scale",       mitosis_threshold_scale_},
        {"novelty_threshold_scale",       novelty_threshold_scale_},
    };
    // Commissioning state.  Emitted ONLY when the feature is enabled, so every
    // config that leaves dim_autocal_ticks at 0 produces a byte-identical
    // snapshot to before this feature existed (the gain-0 guard extends to the
    // serialised form, which is what the pristine-snapshot differ compares).
    //
    // Both the accumulator AND the installed ranges must round-trip: an EPM
    // restored mid-window has to finish the window with the frames it already
    // saw, and one restored after the window must come back conditioned the
    // same way — the encoder is rebuilt from params by on_setup and would
    // otherwise silently revert to the defaults.  Same requirement, and the
    // same reason, as Invariant 10's Hopf MOC EMA.
    if (dim_autocal_ticks_ > 0) {
        nlohmann::json dac = nlohmann::json::object();
        dac["done"] = dim_autocal_done_;
        dac["seen"] = dim_autocal_seen_;
        dac["min"]  = dac_min_;
        dac["max"]  = dac_max_;
        dac["sum"]  = dac_sum_;
        dac["sumsq"] = dac_sumsq_;
        if (dim_autocal_done_ && enc_rbf_) {
            nlohmann::json lo = nlohmann::json::array(), hi = nlohmann::json::array();
            for (auto const& r : enc_rbf_->dim_ranges()) { lo.push_back(r.first); hi.push_back(r.second); }
            dac["range_lo"] = std::move(lo);
            dac["range_hi"] = std::move(hi);
        }
        out["dim_autocal"] = std::move(dac);
    }
    if (normalize_residual_) {
        out["residual_rms"] = residual_rms_;   // B v2 adaptive scale (replay fidelity)
    }
    return out;
}

void EPM::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("EPM::restore_state: unknown version " +
                                 std::to_string(version));
    }
    if (gng_ && s.contains("gng") && !s["gng"].is_null()) {
        *gng_ = ami_ogma::v3::GNG::from_json(s["gng"]);
    }
    has_prev_prototype_ = s.value("has_prev_prototype", false);
    if (has_prev_prototype_ && s.contains("prev_winner_prototype") &&
        s["prev_winner_prototype"].is_array()) {
        auto const& a = s["prev_winner_prototype"];
        prev_winner_prototype_.resize(int(a.size()));
        for (size_t i = 0; i < a.size(); ++i)
            prev_winner_prototype_(int(i)) = a[i].get<float>();
    }
    ema_tle_                   = s.value("ema_tle",                   ema_tle_);
    last_tle_                  = s.value("last_tle",                  last_tle_);
    residual_rms_              = s.value("residual_rms",              residual_rms_);
    last_quant_error_          = s.value("last_quant_error",          last_quant_error_);
    novelty_threshold_now_     = s.value("novelty_threshold_now",     novelty_threshold_now_);
    history_trace_.clear();
    if (s.contains("history_trace") && s["history_trace"].is_array())
        for (auto const& v : s["history_trace"]) history_trace_.push_back(v.get<int>());
    transition_counts_.clear();
    if (s.contains("transition_counts") && s["transition_counts"].is_object()) {
        for (auto const& [src_str, succ] : s["transition_counts"].items()) {
            int src = std::stoi(src_str);
            for (auto const& [dst_str, cnt] : succ.items()) {
                transition_counts_[src][std::stoi(dst_str)] = cnt.get<int>();
            }
        }
    }
    prev_winner_id_for_transitions_ =
        s.value("prev_winner_id_for_transitions", prev_winner_id_for_transitions_);
    epsilon_b_scale_           = s.value("epsilon_b_scale",           epsilon_b_scale_);
    min_insertion_error_scale_ = s.value("min_insertion_error_scale", min_insertion_error_scale_);
    mitosis_threshold_scale_   = s.value("mitosis_threshold_scale",   mitosis_threshold_scale_);
    novelty_threshold_scale_   = s.value("novelty_threshold_scale",   novelty_threshold_scale_);

    // Commissioning state.  Absent from snapshots taken with the feature off,
    // and from every pre-feature snapshot — in both cases the defaults set by
    // on_setup are already correct, so absence is not an error.
    if (s.contains("dim_autocal") && s["dim_autocal"].is_object()) {
        auto const& dac = s["dim_autocal"];
        dim_autocal_done_ = dac.value("done", false);
        dim_autocal_seen_ = dac.value("seen", uint64_t{0});
        if (dac.contains("min"))   dac_min_   = dac["min"].get<std::vector<double>>();
        if (dac.contains("max"))   dac_max_   = dac["max"].get<std::vector<double>>();
        if (dac.contains("sum"))   dac_sum_   = dac["sum"].get<std::vector<double>>();
        if (dac.contains("sumsq")) dac_sumsq_ = dac["sumsq"].get<std::vector<double>>();
        // Re-install the calibrated ranges: on_setup rebuilt the encoder from
        // params, so without this the restored EPM would run its baked GNG
        // against DEFAULT conditioning — a silent space swap, the exact class
        // of bug this whole mechanism exists to make impossible.
        if (dim_autocal_done_ && enc_rbf_ &&
            dac.contains("range_lo") && dac.contains("range_hi")) {
            auto lo = dac["range_lo"].get<std::vector<double>>();
            auto hi = dac["range_hi"].get<std::vector<double>>();
            if (lo.size() == hi.size() && !lo.empty()) {
                std::vector<std::pair<float,float>> ranges;
                ranges.reserve(lo.size());
                for (size_t i = 0; i < lo.size(); ++i)
                    ranges.emplace_back(float(lo[i]), float(hi[i]));
                enc_rbf_->set_dim_ranges(ranges);
            }
        }
    }
}

} // namespace ogma
