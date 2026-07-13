#include "ogma/modules/GaitSelector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("GaitSelector param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("GaitSelector param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("GaitSelector param '" + key + "' must be bool");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("GaitSelector param '" + key + "' must be string");
}

} // namespace

GaitSelector::GaitSelector()  = default;
GaitSelector::~GaitSelector() = default;

std::string_view GaitSelector::type_name() const { return "GaitSelector"; }

std::vector<TopicSpec> GaitSelector::input_topics() const {
    std::string consensus_topic = std::string(topics::kConsensusPrefix) + std::to_string(level_);
    return {
        TopicSpec{consensus_topic,       std::type_index(typeid(ConsensusToken)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kNeuroState,   std::type_index(typeid(NeuroState)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kEventsPrefix, std::type_index(typeid(EnvEvent)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> GaitSelector::output_topics() const {
    return {
        TopicSpec{intent_override_topic_, std::type_index(typeid(IntentToken)),
                  SubscriptionKind::Direct, /*required=*/true},
    };
}

ParamSchema GaitSelector::params_schema() const {
    return {
        {"level",               ParamMutability::ConstructionOnly, "consensus.<level> to read latent from", ParamValue{int64_t{0}}},
        {"intent_override_topic", ParamMutability::ConstructionOnly, "topic to publish IntentToken (posture index) on; whole-body Premotor consumes it", ParamValue{std::string(topics::kIntentOverride)}},
        {"gaits",               ParamMutability::ConstructionOnly, "REQUIRED: array of JSON-string posture-index sequences, e.g. [\"[0,0,0]\", \"[1,1,2,2]\"].  Each is one gait primitive (option).", std::nullopt},
        {"learning_rate",       ParamMutability::HotMutable,       "REINFORCE step size on the gait-selection policy", ParamValue{0.05}},
        {"temperature_base",    ParamMutability::HotMutable,       "softmax temperature baseline", ParamValue{1.0}},
        {"temperature_da_gain", ParamMutability::HotMutable,       "T = temperature_base / (1 + dopamine*this); DA hardens the policy", ParamValue{0.5}},
        {"sample_action",       ParamMutability::HotMutable,       "true = sample gait from softmax; false = argmax", ParamValue{true}},
        {"force_gait_id",       ParamMutability::HotMutable,       ">=0 = open-loop: always dispatch this gait, no learning (A2.0 feasibility / UI probe).  -1 = learned selection.", ParamValue{int64_t{-1}}},
        {"advantage_normalization", ParamMutability::HotMutable,   "normalize advantage by running std over advantage_window", ParamValue{true}},
        {"advantage_window",    ParamMutability::HotMutable,       "window of recent gait returns for the baseline/std", ParamValue{int64_t{100}}},
        {"master_seed",         ParamMutability::ConstructionOnly, "RNG seed for gait sampling (paired-seed determinism)", ParamValue{int64_t{0}}},
    };
}

void GaitSelector::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;

    apply_param(params, "level",                 [&](auto const& v){ level_                   = int(get_int(v, "level")); });
    apply_param(params, "intent_override_topic", [&](auto const& v){ intent_override_topic_   = get_string(v, "intent_override_topic"); });
    apply_param(params, "learning_rate",         [&](auto const& v){ lr_                      = float(get_double(v, "learning_rate")); });
    apply_param(params, "temperature_base",      [&](auto const& v){ temperature_base_        = float(get_double(v, "temperature_base")); });
    apply_param(params, "temperature_da_gain",   [&](auto const& v){ temperature_da_gain_     = float(get_double(v, "temperature_da_gain")); });
    apply_param(params, "sample_action",         [&](auto const& v){ sample_action_           = get_bool(v, "sample_action"); });
    apply_param(params, "force_gait_id",         [&](auto const& v){ force_gait_id_           = int(get_int(v, "force_gait_id")); });
    apply_param(params, "advantage_normalization",[&](auto const& v){ advantage_normalization_ = get_bool(v, "advantage_normalization"); });
    apply_param(params, "advantage_window",      [&](auto const& v){ advantage_window_        = std::max(2, int(get_int(v, "advantage_window"))); });
    apply_param(params, "master_seed",           [&](auto const& v){ master_seed_             = uint64_t(get_int(v, "master_seed")); });

    // Parse the gait library: vector<string>, each a JSON int array.
    apply_param(params, "gaits", [&](ParamValue const& v){
        auto const* arr = std::get_if<std::vector<std::string>>(&v);
        if (!arr) throw std::invalid_argument("GaitSelector 'gaits' must be an array of JSON-string posture-index sequences");
        for (auto const& s : *arr) {
            auto j = nlohmann::json::parse(s);
            if (!j.is_array() || j.empty())
                throw std::invalid_argument("GaitSelector gait must be a non-empty JSON int array: " + s);
            std::vector<int> seq;
            seq.reserve(j.size());
            for (auto const& e : j) seq.push_back(e.get<int>());
            gaits_.push_back(std::move(seq));
        }
    });
    if (gaits_.empty())
        throw std::invalid_argument("GaitSelector requires a non-empty 'gaits' library");

    gait_select_counts_.assign(gaits_.size(), 0);
    rng_.seed(master_seed_ ? master_seed_ : 0xC0FFEEu);

    sub_ids_.clear();
    std::string consensus_topic = std::string(topics::kConsensusPrefix) + std::to_string(level_);
    sub_ids_.push_back(bus_->subscribe(consensus_topic, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_consensus(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kNeuroState, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_neuro(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_event(t, p); }));
}

void GaitSelector::handle_consensus(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (!ct) return;
    if (!weights_initialised_ && ct->fused_embedding.size() > 0) {
        latent_dim_ = int(ct->fused_embedding.size());
        int K = int(gaits_.size());
        std::normal_distribution<float> nd(0.0f, 0.01f);
        W_.resize(K, latent_dim_);
        b_.resize(K);
        for (int i = 0; i < K; ++i) {
            for (int j = 0; j < latent_dim_; ++j) W_(i, j) = nd(rng_);
            b_[i] = 0.0f;
        }
        weights_initialised_ = true;
    }
    last_latent_ = ct->fused_embedding;
}

void GaitSelector::handle_neuro(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto n = std::dynamic_pointer_cast<const NeuroState>(payload);
    if (!n) return;
    dopamine_ = float(n->dopamine);
}

void GaitSelector::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!e) return;
    std::string name(topic.substr(std::string(topics::kEventsPrefix).size()));

    if (name == "episode_end") {
        // Credit the partial gait for the reward it accrued, then drop it so
        // the next tick reselects fresh (don't carry a gait across a teleport).
        credit_completed_gait();
        active_gait_id_ = -1;
        accrued_reward_ = 0.0f;
        return;
    }

    if (name == "hit")                                              accrued_reward_ += e->intensity;
    else if (name == "miss" || name == "wall_stuck" || name == "failed") accrued_reward_ -= e->intensity;
}

int GaitSelector::select_gait() {
    int K = int(gaits_.size());
    if (K == 0) return -1;

    // Open-loop / probe mode: always dispatch the forced gait (no learning).
    if (force_gait_id_ >= 0) {
        int g = std::min(force_gait_id_, K - 1);
        sel_gait_ = g;
        ++total_selections_;
        if (g >= 0 && g < int(gait_select_counts_.size())) ++gait_select_counts_[g];
        return g;
    }

    // Learned selection needs an initialised policy + a latent this tick.
    if (!weights_initialised_ || last_latent_.size() != latent_dim_) return -1;

    Eigen::VectorXf scores = W_ * last_latent_ + b_;
    float T = temperature_base_ / (1.0f + dopamine_ * temperature_da_gain_);
    T = std::max(0.05f, T);
    scores /= T;
    float smax = scores.maxCoeff();
    Eigen::VectorXf dist(K);
    float sum = 0.0f;
    for (int i = 0; i < K; ++i) { dist[i] = std::exp(scores[i] - smax); sum += dist[i]; }
    if (sum <= 0.0f) sum = 1.0f;
    dist /= sum;

    float ent = 0.0f;
    for (int i = 0; i < K; ++i) if (dist[i] > 1e-12f) ent -= dist[i] * std::log(dist[i]);
    last_entropy_ = ent;

    int chosen = 0;
    if (sample_action_) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        float r = u(rng_), c = 0.0f;
        chosen = K - 1;
        for (int i = 0; i < K; ++i) { c += dist[i]; if (r <= c) { chosen = i; break; } }
    } else {
        float best = dist[0]; chosen = 0;
        for (int i = 1; i < K; ++i) if (dist[i] > best) { best = dist[i]; chosen = i; }
    }

    sel_latent_ = last_latent_;
    sel_dist_   = dist;
    sel_gait_   = chosen;
    ++total_selections_;
    if (chosen >= 0 && chosen < int(gait_select_counts_.size())) ++gait_select_counts_[chosen];
    return chosen;
}

void GaitSelector::credit_completed_gait() {
    if (force_gait_id_ >= 0) return;                       // no learning in probe mode
    if (!weights_initialised_) return;
    if (sel_gait_ < 0 || sel_latent_.size() != latent_dim_ || sel_dist_.size() != int(gaits_.size())) return;

    float G = accrued_reward_;
    // Baseline + std over PRIOR returns (exclude this one), then record it.
    float mean = 0.0f, var = 0.0f;
    int n = int(recent_returns_.size());
    if (n >= 1) {
        for (float v : recent_returns_) mean += v;
        mean /= float(n);
        for (float v : recent_returns_) { float d = v - mean; var += d * d; }
        var /= float(n);
    }
    float sd = std::sqrt(std::max(var, 1e-6f));
    float advantage = G - mean;
    if (advantage_normalization_ && n >= 2) advantage /= std::max(sd, 1e-3f);
    last_advantage_ = advantage;

    int K = int(gaits_.size());
    for (int i = 0; i < K; ++i) {
        float indicator = (i == sel_gait_) ? 1.0f : 0.0f;
        float grad = indicator - sel_dist_[i];
        W_.row(i).noalias() += (lr_ * advantage * grad) * sel_latent_.transpose();
    }

    recent_returns_.push_back(G);
    while (int(recent_returns_.size()) > advantage_window_) recent_returns_.pop_front();
    ++gaits_completed_;
}

void GaitSelector::tick(uint64_t tick_id) {
    if (gaits_.empty()) return;

    bool need_select = (active_gait_id_ < 0)
                    || (gait_pos_ >= int(gaits_[active_gait_id_].size()));
    if (need_select) {
        if (active_gait_id_ >= 0) credit_completed_gait();   // a gait just finished
        int g = select_gait();
        if (g < 0) return;                                   // can't select yet (no latent) — emit nothing
        active_gait_id_ = g;
        gait_pos_       = 0;
        accrued_reward_ = 0.0f;
    }

    int posture_idx = gaits_[active_gait_id_][gait_pos_];
    auto tok = std::make_shared<IntentToken>();
    tok->tick_id     = tick_id;
    tok->producer_id = id_.empty() ? std::string("gait_selector") : id_;
    tok->index       = posture_idx;
    bus_->publish(intent_override_topic_, tok);

    ++gait_pos_;
}

void GaitSelector::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "learning_rate")           lr_                      = float(get_double(value, k));
    else if (k == "temperature_base")        temperature_base_        = float(get_double(value, k));
    else if (k == "temperature_da_gain")     temperature_da_gain_     = float(get_double(value, k));
    else if (k == "sample_action")           sample_action_           = get_bool(value, k);
    else if (k == "force_gait_id")           force_gait_id_           = int(get_int(value, k));
    else if (k == "advantage_normalization") advantage_normalization_ = get_bool(value, k);
    else if (k == "advantage_window")        advantage_window_        = std::max(2, int(get_int(value, k)));
    else throw std::invalid_argument("GaitSelector: unknown or construction-only param '" + k + "'");
}

nlohmann::json GaitSelector::snapshot_state() const {
    nlohmann::json j;
    j["version"]              = 1;
    j["weights_initialised"]  = weights_initialised_;
    j["latent_dim"]           = latent_dim_;
    j["active_gait_id"]       = active_gait_id_;
    j["gait_pos"]             = gait_pos_;
    j["accrued_reward"]       = accrued_reward_;
    j["total_selections"]     = total_selections_;
    j["gaits_completed"]      = gaits_completed_;
    j["gait_select_counts"]   = gait_select_counts_;
    j["recent_returns"]       = std::vector<float>(recent_returns_.begin(), recent_returns_.end());
    if (weights_initialised_) {
        nlohmann::json wj = nlohmann::json::array();
        for (int i = 0; i < W_.rows(); ++i) {
            nlohmann::json row = nlohmann::json::array();
            for (int jj = 0; jj < W_.cols(); ++jj) row.push_back(W_(i, jj));
            wj.push_back(row);
        }
        j["W"] = wj;
        j["b"] = std::vector<float>(b_.data(), b_.data() + b_.size());
    }
    return j;
}

void GaitSelector::restore_state(nlohmann::json const& j) {
    weights_initialised_ = j.value("weights_initialised", false);
    latent_dim_          = j.value("latent_dim", 0);
    active_gait_id_      = j.value("active_gait_id", -1);
    gait_pos_            = j.value("gait_pos", 0);
    accrued_reward_      = j.value("accrued_reward", 0.0f);
    total_selections_    = j.value("total_selections", 0);
    gaits_completed_     = j.value("gaits_completed", 0);
    if (j.contains("gait_select_counts"))
        gait_select_counts_ = j["gait_select_counts"].get<std::vector<int>>();
    recent_returns_.clear();
    if (j.contains("recent_returns"))
        for (float v : j["recent_returns"].get<std::vector<float>>()) recent_returns_.push_back(v);
    if (weights_initialised_ && j.contains("W")) {
        auto const& wj = j["W"];
        int K = int(wj.size());
        W_.resize(K, latent_dim_);
        for (int i = 0; i < K; ++i)
            for (int c = 0; c < latent_dim_; ++c) W_(i, c) = wj[i][c].get<float>();
        auto bvec = j["b"].get<std::vector<float>>();
        b_.resize(int(bvec.size()));
        for (int i = 0; i < int(bvec.size()); ++i) b_[i] = bvec[i];
    }
}

nlohmann::json GaitSelector::diag_snapshot() const {
    nlohmann::json j;
    j["active_gait"]      = active_gait_id_;
    j["num_gaits"]        = int(gaits_.size());
    j["total_selections"] = total_selections_;
    j["gaits_completed"]  = gaits_completed_;
    j["entropy"]          = last_entropy_;
    j["advantage"]        = last_advantage_;
    j["select_counts"]    = gait_select_counts_;
    return j;
}

} // namespace ogma
