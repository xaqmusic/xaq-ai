#include "ogma/modules/DescendingPredictor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

#include "ogma/Rng.hpp"

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("DescendingPredictor param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("DescendingPredictor param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("DescendingPredictor param '" + key + "' must be string");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    if (auto p = std::get_if<double>(&v))  return *p != 0.0;
    throw std::invalid_argument("DescendingPredictor: param '" + key + "' must be a bool");
}
std::vector<std::string> get_strings(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("DescendingPredictor param '" + key + "' must be vector<string>");
}

// Topic-to-label conversion: "reality.video.retinal" → "video.retinal".
std::string label_from_topic(std::string const& topic) {
    constexpr std::string_view kPrefix = "reality.";
    if (topic.size() > kPrefix.size() &&
        topic.compare(0, kPrefix.size(), kPrefix) == 0)
        return topic.substr(kPrefix.size());
    return topic;
}

} // namespace

DescendingPredictor::DescendingPredictor()  = default;
DescendingPredictor::~DescendingPredictor() = default;

std::string_view DescendingPredictor::type_name() const { return "DescendingPredictor"; }

std::vector<TopicSpec> DescendingPredictor::input_topics() const {
    std::vector<TopicSpec> out;
    out.push_back(TopicSpec{consensus_topic_,
                             std::type_index(typeid(ConsensusToken)),
                             SubscriptionKind::Direct, /*required=*/true});
    for (auto const& t : targets_)
        out.push_back(TopicSpec{t.topic,
                                 std::type_index(typeid(RealityToken)),
                                 SubscriptionKind::Feedback, /*required=*/true});
    return out;
}

std::vector<TopicSpec> DescendingPredictor::output_topics() const {
    std::vector<TopicSpec> out;
    for (auto const& t : targets_)
        out.push_back(TopicSpec{t.out_topic,
                                 std::type_index(typeid(PredictionToken))});
    return out;
}

ParamSchema DescendingPredictor::params_schema() const {
    return {
        {"consensus_topic",     ParamMutability::ConstructionOnly, "Source topic", ParamValue{std::string("consensus.0")}},
        {"targets",             ParamMutability::ConstructionOnly, "List of reality.<group>.<modality> topics to predict", std::nullopt},
        {"update_method",       ParamMutability::ConstructionOnly, "sgd | rls",   ParamValue{std::string("sgd")}},
        {"learning_rate",       ParamMutability::HotMutable,       "SGD step size", ParamValue{0.01}},
        {"rls_forget",          ParamMutability::HotMutable,       "RLS forgetting factor", ParamValue{0.99}},
        {"init_noise_scale",    ParamMutability::ConstructionOnly, "Std-dev for W initialisation", ParamValue{0.01}},
        {"freeze_after_ticks",  ParamMutability::HotMutable,       "0 = never freeze; >0 = freeze after this many ticks", ParamValue{int64_t{0}}},
        {"confidence_window",   ParamMutability::HotMutable,       "Rolling window for confidence",  ParamValue{int64_t{100}}},
        {"master_seed",         ParamMutability::ConstructionOnly, "RNG namespace seed",  ParamValue{int64_t{0}}},
        {"target_is_residual",  ParamMutability::ConstructionOnly,
         "Set true when the target EPM runs subtract_descending_prediction: its published "
         "latent is then the RESIDUAL (encode − prediction), which IS the prediction error — "
         "the update integrates it directly (err = latent), driving the residual to zero. "
         "The legacy rule (err = latent − cached_prediction) against a residual target "
         "converges to HALF-subtraction (P→E/2), found 2026-08-14 when the pair was closed "
         "over the motor path for the first time.  false = legacy (raw-latent targets).",
         ParamValue{false}},
    };
}

void DescendingPredictor::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("DescendingPredictor requires a non-null Bus");

    auto find_required = [&](std::string const& key) -> ParamValue const& {
        auto it = params.find(key);
        if (it == params.end())
            throw std::invalid_argument("DescendingPredictor: required param '" + key + "' missing");
        return it->second;
    };

    apply_param(params, "consensus_topic",   [&](auto const& v){ consensus_topic_   = get_string(v, "consensus_topic"); });
    apply_param(params, "update_method",     [&](auto const& v){ update_method_     = get_string(v, "update_method"); });
    apply_param(params, "learning_rate",     [&](auto const& v){ learning_rate_     = float(get_double(v, "learning_rate")); });
    apply_param(params, "rls_forget",        [&](auto const& v){ rls_forget_        = float(get_double(v, "rls_forget")); });
    apply_param(params, "init_noise_scale",  [&](auto const& v){ init_noise_scale_  = float(get_double(v, "init_noise_scale")); });
    apply_param(params, "freeze_after_ticks",[&](auto const& v){ freeze_after_ticks_= get_int(v, "freeze_after_ticks"); });
    apply_param(params, "confidence_window", [&](auto const& v){ confidence_window_ = get_int(v, "confidence_window"); });
    apply_param(params, "master_seed",       [&](auto const& v){ master_seed_       = uint64_t(get_int(v, "master_seed")); });
    apply_param(params, "target_is_residual",[&](auto const& v){ target_is_residual_= get_bool(v, "target_is_residual"); });

    auto target_topics = get_strings(find_required("targets"), "targets");
    if (target_topics.empty())
        throw std::invalid_argument("DescendingPredictor: 'targets' must declare ≥ 1 topic");

    targets_.clear();
    target_idx_by_topic_.clear();
    for (auto const& topic : target_topics) {
        Target t;
        t.topic     = topic;
        t.label     = label_from_topic(topic);
        t.out_topic = std::string("prediction.") + t.label;
        target_idx_by_topic_[topic] = targets_.size();
        targets_.push_back(std::move(t));
    }

    // Subscriptions.  Source consensus is Direct; per-target reality is
    // Feedback (the supervisory signal — last tick's actual latent).
    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(consensus_topic_, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_consensus(t, p); }));
    for (size_t i = 0; i < targets_.size(); ++i) {
        size_t idx = i;
        sub_ids_.push_back(bus_->subscribe(targets_[i].topic, SubscriptionKind::Feedback,
            [this, idx](std::string_view t, MessagePtr p) {
                this->handle_reality(t, p, idx);
            }));
    }
}

void DescendingPredictor::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if (k == "learning_rate")           learning_rate_      = float(get_double(value, k));
    else if (k == "rls_forget")         rls_forget_         = float(get_double(value, k));
    else if (k == "freeze_after_ticks") freeze_after_ticks_ = get_int(value, k);
    else if (k == "confidence_window")  confidence_window_  = get_int(value, k);
    else if (k == "consensus_topic" || k == "targets" || k == "update_method"
          || k == "init_noise_scale" || k == "master_seed")
        throw std::invalid_argument("DescendingPredictor param '" + k + "' is ConstructionOnly");
    else
        throw std::invalid_argument("DescendingPredictor: unknown param '" + k + "'");
}

void DescendingPredictor::handle_consensus(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    // Context source: ConsensusToken (the original design) OR any ProprioToken
    // vector stream — the motor path's surprise vocabulary (PART III B) drives
    // the predictor from the CPG clock [cosφ, sinφ], which is a ProprioToken.
    // Without this fallback a ProprioToken source was SILENTLY dropped: no
    // context → no prediction → no subtraction → the pc-EPM quietly degrades
    // to a plain EPM (the §3.2 dead-code trap, caught at design time).
    Eigen::VectorXf emb;
    if (auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload)) {
        emb = ct->fused_embedding;
    } else if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload)) {
        emb = pt->values;
    } else if (auto rt = std::dynamic_pointer_cast<const RealityToken>(payload)) {
        // Latent-autoregression context (B gate, arm 2): the raw encoding
        // itself — a full linear map in latent space, against which the 2-D
        // clock context (first stride harmonic only) is the weak baseline.
        emb = rt->latent;
    }
    if (emb.size() == 0) return;
    latest_consensus_ = std::move(emb);
    consensus_seen_   = true;

    // Lazy initialise W/b once we know the source dim.
    if (!dim_known_) {
        source_dim_ = int(latest_consensus_.size());
        std::mt19937_64 rng = derive_rng(master_seed_,
            std::string("predictor.") + (id_.empty() ? std::string("predictor") : id_) + ".weights_init");
        std::normal_distribution<float> dist(0.0f, init_noise_scale_);
        for (auto& t : targets_) {
            // Target dim is unknown until first reality.* delivery; we lazily
            // size W when that happens.  For now placeholder: source dim ×
            // source dim, will be resized once we see reality.
            t.W = Eigen::MatrixXf::Zero(source_dim_, source_dim_);
            t.b = Eigen::VectorXf::Zero(source_dim_);
            for (int i = 0; i < t.W.rows(); ++i)
                for (int j = 0; j < t.W.cols(); ++j)
                    t.W(i, j) = dist(rng);
        }
        dim_known_ = true;
    }
}

void DescendingPredictor::handle_reality(std::string_view /*topic*/, MessagePtr payload, size_t target_idx) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt || target_idx >= targets_.size()) return;
    auto& t = targets_[target_idx];

    // Ensure W/b have correct target dim — resize lazily if needed.
    int tgt_dim = int(rt->latent.size());
    if (tgt_dim <= 0) return;
    if (t.W.rows() != tgt_dim || t.W.cols() != source_dim_) {
        std::mt19937_64 rng = derive_rng(master_seed_,
            std::string("predictor.") + (id_.empty() ? std::string("predictor") : id_) + ".weights_resize");
        std::normal_distribution<float> dist(0.0f, init_noise_scale_);
        Eigen::MatrixXf W2 = Eigen::MatrixXf::Zero(tgt_dim, source_dim_);
        for (int i = 0; i < W2.rows(); ++i)
            for (int j = 0; j < W2.cols(); ++j)
                W2(i, j) = dist(rng);
        t.W = std::move(W2);
        t.b = Eigen::VectorXf::Zero(tgt_dim);
    }

    // Supervisory update against the cached forward pair.
    bool can_update = t.cached_valid &&
                      cached_consensus_valid_ &&
                      t.cached_prediction.size() == rt->latent.size() &&
                      cached_consensus_.size()   == source_dim_;

    bool frozen = (freeze_after_ticks_ > 0) &&
                  (int64_t(ticks_run_) >= freeze_after_ticks_);

    if (can_update && !frozen) {
        Eigen::VectorXf err = target_is_residual_
                                  ? rt->latent
                                  : (rt->latent - t.cached_prediction);

        if (update_method_ == "rls") {
            // RLS approximation via diagonal-only inverse-covariance for
            // simplicity in Phase 1.6; full block-RLS is a Phase 3 stretch.
            float gain = learning_rate_ / std::max(1e-6f, rls_forget_);
            t.W.noalias() += gain * err * cached_consensus_.transpose();
            t.b.noalias() += gain * err;
        } else {
            // Plain SGD step.
            t.W.noalias() += learning_rate_ * err * cached_consensus_.transpose();
            t.b.noalias() += learning_rate_ * err;
        }

        // Confidence EMA over rolling error magnitudes.
        float a = 1.0f / std::max(1.0f, float(confidence_window_));
        t.err_ema  = (1.0f - a) * t.err_ema  + a * err.norm();
        t.norm_ema = (1.0f - a) * t.norm_ema + a * std::max(1e-6f, rt->latent.norm());
    }
}

void DescendingPredictor::tick(uint64_t tick_id) {
    ++ticks_run_;

    // Forward pass + publish for each target.
    for (auto& t : targets_) {
        auto out = std::make_shared<PredictionToken>();
        out->tick_id          = tick_id;
        out->producer_id      = id_.empty() ? std::string("predictor") : id_;
        out->target_modality  = t.label;

        if (consensus_seen_ && t.W.cols() == latest_consensus_.size() && t.W.rows() > 0) {
            out->predicted_latent = t.W * latest_consensus_ + t.b;
            t.cached_prediction = out->predicted_latent;
            t.cached_valid      = true;
        } else {
            // No consensus yet → publish zero prediction (no-op subtraction
            // downstream).  Don't update cache so the next supervisory step
            // is skipped rather than fed a bogus pair.
            out->predicted_latent = Eigen::VectorXf::Zero(t.W.rows() > 0 ? t.W.rows() : 1);
            t.cached_valid = false;
        }
        out->confidence = std::max(0.0f, std::min(1.0f,
            1.0f - t.err_ema / std::max(1e-6f, t.norm_ema)));

        bus_->publish(t.out_topic, out);
    }

    // Cache the consensus we just used so the next supervisory update can
    // pair it with the reality(t-1) feedback that arrives at tick t+1.
    if (consensus_seen_) {
        cached_consensus_       = latest_consensus_;
        cached_consensus_valid_ = true;
    }
    consensus_seen_ = false;
}

float DescendingPredictor::confidence(std::string const& target) const {
    auto it = target_idx_by_topic_.find(target);
    if (it == target_idx_by_topic_.end()) return 0.0f;
    auto const& t = targets_[it->second];
    return std::max(0.0f, std::min(1.0f, 1.0f - t.err_ema / std::max(1e-6f, t.norm_ema)));
}

Eigen::MatrixXf const* DescendingPredictor::weights(std::string const& target) const {
    auto it = target_idx_by_topic_.find(target);
    if (it == target_idx_by_topic_.end()) return nullptr;
    return &targets_[it->second].W;
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4 — UI-dev W3.2)
// ---------------------------------------------------------------------------
//
// Captures: source_dim_ + dim_known_, latest/cached consensus + flags,
// ticks_run_, and per-target {W, b, cached_prediction, cached_valid,
// err_ema, norm_ema} keyed by topic so restore is robust to target order.
// Static config (consensus_topic_, learning_rate_, etc.) lives in
// GraphConfig and is restored at construction.

namespace {

nlohmann::json eigen_vec_to_json(Eigen::VectorXf const& v) {
    nlohmann::json a = nlohmann::json::array();
    for (int i = 0; i < v.size(); ++i) a.push_back(v(i));
    return a;
}

void json_to_eigen_vec(nlohmann::json const& j, Eigen::VectorXf& out) {
    if (!j.is_array()) { out = Eigen::VectorXf(); return; }
    out.resize(int(j.size()));
    for (size_t i = 0; i < j.size(); ++i) out(int(i)) = j[i].get<float>();
}

nlohmann::json eigen_mat_to_json(Eigen::MatrixXf const& m) {
    nlohmann::json data = nlohmann::json::array();
    for (int r = 0; r < m.rows(); ++r)
        for (int c = 0; c < m.cols(); ++c)
            data.push_back(m(r, c));
    return nlohmann::json{
        {"rows", int(m.rows())},
        {"cols", int(m.cols())},
        {"data", data},
    };
}

void json_to_eigen_mat(nlohmann::json const& j, Eigen::MatrixXf& out) {
    if (!j.is_object() || !j.contains("rows") || !j.contains("cols")) {
        out = Eigen::MatrixXf(); return;
    }
    int r = j.value("rows", 0);
    int c = j.value("cols", 0);
    out.resize(r, c);
    auto const& data = j["data"];
    for (int i = 0; i < r * c; ++i) out(i / c, i % c) = data[i].get<float>();
}

} // namespace

nlohmann::json DescendingPredictor::snapshot_state() const {
    nlohmann::json targets = nlohmann::json::object();
    for (auto const& t : targets_) {
        targets[t.topic] = nlohmann::json{
            {"W",                 eigen_mat_to_json(t.W)},
            {"b",                 eigen_vec_to_json(t.b)},
            {"cached_prediction", eigen_vec_to_json(t.cached_prediction)},
            {"cached_valid",      t.cached_valid},
            {"err_ema",           t.err_ema},
            {"norm_ema",          t.norm_ema},
        };
    }
    return nlohmann::json{
        {"version",                1},
        {"source_dim",             source_dim_},
        {"dim_known",              dim_known_},
        {"latest_consensus",       eigen_vec_to_json(latest_consensus_)},
        {"consensus_seen",         consensus_seen_},
        {"cached_consensus",       eigen_vec_to_json(cached_consensus_)},
        {"cached_consensus_valid", cached_consensus_valid_},
        {"ticks_run",              ticks_run_},
        {"targets",                targets},
    };
}

void DescendingPredictor::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error(
            "DescendingPredictor::restore_state: unknown version " +
            std::to_string(version));
    }
    source_dim_              = s.value("source_dim", source_dim_);
    dim_known_               = s.value("dim_known",  dim_known_);
    consensus_seen_          = s.value("consensus_seen",         consensus_seen_);
    cached_consensus_valid_  = s.value("cached_consensus_valid", cached_consensus_valid_);
    ticks_run_               = s.value("ticks_run",              ticks_run_);
    if (s.contains("latest_consensus"))
        json_to_eigen_vec(s["latest_consensus"], latest_consensus_);
    if (s.contains("cached_consensus"))
        json_to_eigen_vec(s["cached_consensus"], cached_consensus_);
    if (s.contains("targets") && s["targets"].is_object()) {
        for (auto it = s["targets"].begin(); it != s["targets"].end(); ++it) {
            auto idx_it = target_idx_by_topic_.find(it.key());
            if (idx_it == target_idx_by_topic_.end()) continue;
            auto& t = targets_[idx_it->second];
            auto const& tj = it.value();
            if (tj.contains("W")) json_to_eigen_mat(tj["W"], t.W);
            if (tj.contains("b")) json_to_eigen_vec(tj["b"], t.b);
            if (tj.contains("cached_prediction"))
                json_to_eigen_vec(tj["cached_prediction"], t.cached_prediction);
            t.cached_valid = tj.value("cached_valid", t.cached_valid);
            t.err_ema      = tj.value("err_ema",      t.err_ema);
            t.norm_ema     = tj.value("norm_ema",     t.norm_ema);
        }
    }
}

} // namespace ogma
