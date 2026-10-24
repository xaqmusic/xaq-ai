#include "ogma/modules/BeliefFilter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

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
    throw std::invalid_argument("BeliefFilter param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("BeliefFilter param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("BeliefFilter param '" + key + "' must be a string");
}
constexpr double kLog2Pi = 1.8378770664093453;
} // namespace

BeliefFilter::BeliefFilter()  = default;
BeliefFilter::~BeliefFilter() = default;

std::string_view BeliefFilter::type_name() const { return "BeliefFilter"; }

std::vector<TopicSpec> BeliefFilter::input_topics() const {
    return {TopicSpec{reality_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false},
            TopicSpec{obs_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false}};
}
std::vector<TopicSpec> BeliefFilter::output_topics() const {
    return {TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))}};
}

ParamSchema BeliefFilter::params_schema() const {
    return {
        {"reality_topic", ParamMutability::ConstructionOnly, "The EPM whose nodes are the filter's states.", ParamValue{std::string("reality.bodypose.pose")}},
        {"obs_topic", ParamMutability::ConstructionOnly, "The observation (ProprioToken, own units) the states' likelihoods are computed on.", ParamValue{std::string("reality.proprio.joints")}},
        {"output_topic", ParamMutability::ConstructionOnly, "Published [estimate..., MAP prob, entropy, surprise]. Empty = belief.<id>.", ParamValue{std::string("")}},
        {"alpha", ParamMutability::HotMutable, "Dirichlet pseudo-count per transition: smoothing, and the diffusion that spreads the belief when nothing is observed.", ParamValue{0.1}},
        {"gain_cap", ParamMutability::HotMutable, "Upper clamp on the decode heads' Kalman-schedule gain.", ParamValue{0.2}},
        {"gain_floor", ParamMutability::HotMutable, "Lower clamp on the decode heads' gain (drift).", ParamValue{0.002}},
        {"var_rel_floor", ParamMutability::HotMutable, "A node's variance never below this fraction of the observation's global variance.", ParamValue{0.01}},
        {"min_count", ParamMutability::HotMutable, "A node's own likelihood is used once its decode head has this many samples (before: the global one).", ParamValue{int64_t{5}}},
    };
}

ParamMap BeliefFilter::current_params() const {
    return {{"reality_topic", ParamValue{reality_topic_}}, {"obs_topic", ParamValue{obs_topic_}},
            {"output_topic", ParamValue{output_topic_}}, {"alpha", ParamValue{alpha_}}, {"gain_cap", ParamValue{gain_cap_}},
            {"gain_floor", ParamValue{gain_floor_}}, {"var_rel_floor", ParamValue{var_rel_floor_}},
            {"min_count", ParamValue{int64_t(min_count_)}}};
}

void BeliefFilter::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("BeliefFilter requires a non-null Bus");
    apply_param(params, "reality_topic", [&](auto const& v){ reality_topic_ = get_string(v, "reality_topic"); });
    apply_param(params, "obs_topic",     [&](auto const& v){ obs_topic_     = get_string(v, "obs_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v, "output_topic"); });
    apply_param(params, "alpha",         [&](auto const& v){ alpha_         = std::max(1e-6, get_double(v, "alpha")); });
    apply_param(params, "gain_cap",      [&](auto const& v){ gain_cap_      = get_double(v, "gain_cap"); });
    apply_param(params, "gain_floor",    [&](auto const& v){ gain_floor_    = get_double(v, "gain_floor"); });
    apply_param(params, "var_rel_floor", [&](auto const& v){ var_rel_floor_ = std::max(0.0, get_double(v, "var_rel_floor")); });
    apply_param(params, "min_count",     [&](auto const& v){ min_count_     = std::max(0, int(get_int(v, "min_count"))); });
    if (output_topic_.empty()) output_topic_ = "belief." + (id().empty() ? std::string("filter") : std::string(id()));
    sub_ids_.push_back(bus_->subscribe(reality_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_reality(p); }));
    sub_ids_.push_back(bus_->subscribe(obs_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_obs(p); }));
}

void BeliefFilter::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "alpha")         alpha_ = std::max(1e-6, get_double(value, k));
    else if (k == "gain_cap")      gain_cap_ = get_double(value, k);
    else if (k == "gain_floor")    gain_floor_ = get_double(value, k);
    else if (k == "var_rel_floor") var_rel_floor_ = std::max(0.0, get_double(value, k));
    else if (k == "min_count")     min_count_ = std::max(0, int(get_int(value, k)));
    else throw std::invalid_argument("BeliefFilter param '" + k + "' is ConstructionOnly or unknown");
}

void BeliefFilter::handle_reality(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(p);
    if (!rt) return;
    for (int k : rt->pruned_ids) drop_node(k);
    winner_ = rt->winner_id;
    winner_fresh_ = winner_ >= 0;
}

void BeliefFilter::handle_obs(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() == 0) return;
    obs_ = pt->values;
    obs_fresh_ = true;
}

int BeliefFilter::index_of(int node) {
    auto it = idx_.find(node);
    if (it != idx_.end()) return it->second;
    const int i = int(ids_.size());
    ids_.push_back(node); idx_[node] = i;
    mu_.push_back(dim_ > 0 ? g_mu_ : Eigen::VectorXf());
    var_.push_back(dim_ > 0 ? g_var_ : Eigen::VectorXf());
    n_.push_back(0.0);
    for (auto& row : C_) row.push_back(0.0);
    C_.push_back(std::vector<double>(ids_.size(), 0.0));
    // a new state enters with the belief the smoothing would give it
    b_.push_back(b_.empty() ? 1.0 : 1.0 / double(ids_.size()));
    double s = 0.0; for (double v : b_) s += v; for (double& v : b_) v /= s;
    return i;
}

void BeliefFilter::drop_node(int node) {
    auto it = idx_.find(node);
    if (it == idx_.end()) return;
    const int i = it->second;
    ids_.erase(ids_.begin() + i); mu_.erase(mu_.begin() + i); var_.erase(var_.begin() + i); n_.erase(n_.begin() + i);
    b_.erase(b_.begin() + i);
    C_.erase(C_.begin() + i);
    for (auto& row : C_) row.erase(row.begin() + i);
    idx_.clear();
    for (int k = 0; k < int(ids_.size()); ++k) idx_[ids_[size_t(k)]] = k;
    if (last_winner_ == node) last_winner_ = -1;
    double s = 0.0; for (double v : b_) s += v;
    if (s > 0) for (double& v : b_) v /= s;
}

double BeliefFilter::belief_of(int node) const {
    auto it = idx_.find(node);
    return it == idx_.end() ? 0.0 : b_[size_t(it->second)];
}

void BeliefFilter::tick(uint64_t tick_id) {
    ++ticks_;
    if (obs_fresh_ && dim_ == 0 && obs_.size() > 0) {
        dim_ = int(obs_.size());
        g_mu_ = obs_; g_var_ = Eigen::VectorXf::Constant(dim_, 1e-6f);
        for (size_t i = 0; i < mu_.size(); ++i) { mu_[i] = g_mu_; var_[i] = g_var_; }
    }
    if (winner_fresh_) index_of(winner_);
    const int N = int(ids_.size());
    if (N == 0 || dim_ == 0) { obs_fresh_ = winner_fresh_ = false; return; }

    // ---- predict: b- = b T, T row-normalised counts + alpha ----
    std::vector<double> bp(size_t(N), 0.0);
    for (int i = 0; i < N; ++i) {
        if (b_[size_t(i)] <= 0.0) continue;
        double row = 0.0; for (int j = 0; j < N; ++j) row += C_[size_t(i)][size_t(j)];
        const double den = row + alpha_ * N;
        for (int j = 0; j < N; ++j) bp[size_t(j)] += b_[size_t(i)] * (C_[size_t(i)][size_t(j)] + alpha_) / den;
    }

    // ---- correct on a fresh observation ----
    const bool have_obs = obs_fresh_ && obs_.size() == dim_;
    if (have_obs) {
        const Eigen::VectorXf vfl = (g_var_ * float(var_rel_floor_)).cwiseMax(1e-6f);
        std::vector<double> ll(static_cast<size_t>(N), 0.0);
        double mx = -1e300;
        for (int j = 0; j < N; ++j) {
            const bool own = n_[size_t(j)] >= double(min_count_);
            const Eigen::VectorXf& m = own ? mu_[size_t(j)] : g_mu_;
            const Eigen::VectorXf v = (own ? var_[size_t(j)] : g_var_).cwiseMax(vfl);
            double l = 0.0;
            for (int d = 0; d < dim_; ++d) {
                const double e = double(obs_[d] - m[d]);
                l += -0.5 * (e * e / double(v[d]) + std::log(double(v[d])) + kLog2Pi);
            }
            ll[size_t(j)] = l; mx = std::max(mx, l);
        }
        double z = 0.0;
        for (int j = 0; j < N; ++j) { bp[size_t(j)] *= std::exp(ll[size_t(j)] - mx); z += bp[size_t(j)]; }
        surprise_ = (z > 0.0) ? -(std::log(z) + mx) / dim_ : 1e9;
        if (z > 0.0) for (double& v : bp) v /= z;
        else std::fill(bp.begin(), bp.end(), 1.0 / N);
        ++corrected_;
    } else {
        double z = 0.0; for (double v : bp) z += v;
        if (z > 0.0) for (double& v : bp) v /= z;
    }
    b_ = std::move(bp);

    // ---- outputs ----
    est_ = Eigen::VectorXf::Zero(dim_);
    map_ = -1; map_p_ = 0.0; entropy_ = 0.0;
    for (int j = 0; j < N; ++j) {
        const double w = b_[size_t(j)];
        est_ += float(w) * (n_[size_t(j)] >= double(min_count_) ? mu_[size_t(j)] : g_mu_);
        if (w > map_p_) { map_p_ = w; map_ = ids_[size_t(j)]; }
        if (w > 0.0) entropy_ -= w * std::log(w);
    }
    if (winner_fresh_) { ++compared_; if (map_ == winner_) ++agree_; }

    // ---- learn from the EPM's label: transitions, and the winner's decode head ----
    if (winner_fresh_) {
        const int wi = idx_[winner_];
        if (last_winner_ >= 0 && idx_.count(last_winner_)) C_[size_t(idx_[last_winner_])][size_t(wi)] += 1.0;
        last_winner_ = winner_;
        if (have_obs) {
            n_[size_t(wi)] += 1.0;
            const float g = float(std::clamp(1.0 / (n_[size_t(wi)] + 1.0), gain_floor_, gain_cap_));
            const Eigen::VectorXf dl = obs_ - mu_[size_t(wi)];
            mu_[size_t(wi)]  += g * dl;
            var_[size_t(wi)] += g * (dl.array().square().matrix() - var_[size_t(wi)]);
            var_[size_t(wi)]  = var_[size_t(wi)].cwiseMax(1e-6f);
        }
    }
    if (have_obs) {
        g_n_ += 1.0;
        const float gg = float(std::max(1.0 / (g_n_ + 1.0), gain_floor_));
        const Eigen::VectorXf dg = obs_ - g_mu_;
        g_mu_  += gg * dg;
        g_var_ += gg * (dg.array().square().matrix() - g_var_);
        g_var_  = g_var_.cwiseMax(1e-6f);
    }

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("belief") : std::string(id());
    out->sensor = "belief"; out->values.resize(dim_ + 3);
    out->values.head(dim_) = est_;
    out->values[dim_] = float(map_p_); out->values[dim_ + 1] = float(entropy_); out->values[dim_ + 2] = float(surprise_);
    bus_->publish(output_topic_, out);
    obs_fresh_ = winner_fresh_ = false;
}

nlohmann::json BeliefFilter::diag_snapshot() const {
    return nlohmann::json{{"nodes", nodes()}, {"map_node", map_}, {"map_prob", map_p_}, {"entropy", entropy_},
                          {"surprise", surprise_}, {"ticks", ticks_}, {"corrected", corrected_},
                          {"agree_with_epm", compared_ > 0 ? double(agree_) / compared_ : 0.0}};
}

} // namespace ogma
