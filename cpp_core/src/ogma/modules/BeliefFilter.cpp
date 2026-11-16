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
        {"head_update", ParamMutability::ConstructionOnly, "\"legacy\" = a head starts at the global mean and variance and moves at most gain_cap per sample (byte-identical); \"own\" = it starts from its first sample and keeps its own samples' running mean and variance (gain 1/n down to gain_floor), read with the n/(n-1) correction; \"visits\" = as own, but n counts visits: each visit folds in once, as its mean, and its within-visit spread is pooled and added (the header).", ParamValue{std::string("legacy")}},
    };
}

ParamMap BeliefFilter::current_params() const {
    return {{"reality_topic", ParamValue{reality_topic_}}, {"obs_topic", ParamValue{obs_topic_}},
            {"output_topic", ParamValue{output_topic_}}, {"alpha", ParamValue{alpha_}}, {"gain_cap", ParamValue{gain_cap_}},
            {"gain_floor", ParamValue{gain_floor_}}, {"var_rel_floor", ParamValue{var_rel_floor_}},
            {"min_count", ParamValue{int64_t(min_count_)}}, {"head_update", ParamValue{std::string(head_visits_ ? "visits" : head_own_ ? "own" : "legacy")}}};
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
    apply_param(params, "head_update",   [&](auto const& v){ const std::string h = get_string(v, "head_update");
        if (h != "legacy" && h != "own" && h != "visits") throw std::invalid_argument("BeliefFilter: head_update must be \"legacy\", \"own\" or \"visits\"");
        head_own_ = h == "own" || h == "visits"; head_visits_ = h == "visits"; });
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
    wss_.push_back(dim_ > 0 ? Eigen::VectorXf::Zero(dim_) : Eigen::VectorXf()); wT_.push_back(0.0);
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
    wss_.erase(wss_.begin() + i); wT_.erase(wT_.begin() + i);
    if (vis_j_ == node) { vis_j_ = -1; vis_len_ = 0.0; }
    b_.erase(b_.begin() + i);
    C_.erase(C_.begin() + i);
    for (auto& row : C_) row.erase(row.begin() + i);
    idx_.clear();
    for (int k = 0; k < int(ids_.size()); ++k) idx_[ids_[size_t(k)]] = k;
    if (last_winner_ == node) last_winner_ = -1;
    double s = 0.0; for (double v : b_) s += v;
    if (s > 0) for (double& v : b_) v /= s;
}

Eigen::VectorXf BeliefFilter::head_var(int j) const {
    if (!head_own_) return var_[size_t(j)];
    const double n = n_[size_t(j)];
    Eigen::VectorXf v = n >= 2.0 ? Eigen::VectorXf(var_[size_t(j)] * float(n / (n - 1.0))) : var_[size_t(j)];
    // "visits": the between-visit part as the PREDICTIVE spread of a new visit when the mean and the variance are both estimated
    // from n visits (Student-t, n - 1 degrees of freedom): s^2 (1 + 1/n) (n - 1)/(n - 3) = var_ (n + 1)/(n - 3) -- derived, not tuned;
    // a few independent visits is exactly the regime a dwelling stream leaves a head in.
    if (head_visits_ && n > 3.0) v = var_[size_t(j)] * float((n + 1.0) / (n - 3.0));
    if (head_visits_ && wT_[size_t(j)] - n > 0.0) v += wss_[size_t(j)] / float(wT_[size_t(j)] - n);   // + the pooled within-visit variance
    return v;
}

// head_update "visits": fold the visit that just ended into its node's head -- its mean once (the "own" recursion over visit
// means), its spread about that mean into the pooled within-visit sum.
void BeliefFilter::close_visit() {
    if (vis_j_ < 0 || vis_len_ <= 0.0 || !idx_.count(vis_j_)) { vis_j_ = -1; vis_len_ = 0.0; return; }
    const size_t j = size_t(idx_[vis_j_]);
    const Eigen::VectorXd vm = vis_sum_ / vis_len_;
    wss_[j] += (vis_ss_ - vis_len_ * vm.cwiseProduct(vm)).cwiseMax(0.0).cast<float>(); wT_[j] += vis_len_;
    const double nn = (n_[j] += 1.0); const Eigen::VectorXf vmf = vm.cast<float>();
    if (nn <= 1.0) { mu_[j] = vmf; var_[j] = Eigen::VectorXf::Zero(dim_); }
    else {
        const float g = float(std::max(1.0 / nn, gain_floor_));
        const Eigen::VectorXf dl = vmf - mu_[j];
        mu_[j] += g * dl;
        var_[j] = (1.0f - g) * (var_[j] + g * dl.array().square().matrix());
    }
    vis_j_ = -1; vis_len_ = 0.0;
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
        for (size_t i = 0; i < mu_.size(); ++i) { mu_[i] = g_mu_; var_[i] = g_var_; wss_[i] = Eigen::VectorXf::Zero(dim_); }
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
        {   // the consistency check (diag only): the observation against the predicted belief's moment-matched mixture
            Eigen::VectorXd m = Eigen::VectorXd::Zero(dim_), s2 = Eigen::VectorXd::Zero(dim_); double bs = 0.0;
            for (int j = 0; j < N; ++j) {
                const double w = bp[size_t(j)]; if (w <= 0.0) continue;
                const bool own = n_[size_t(j)] >= double(min_count_);
                const Eigen::VectorXd mj = (own ? mu_[size_t(j)] : g_mu_).cast<double>(), vj = (own ? head_var(j) : g_var_).cwiseMax(vfl).cast<double>();
                m += w * mj; s2 += w * (vj + mj.cwiseProduct(mj)); bs += w;
            }
            if (bs > 0.0) {
                m /= bs; s2 = s2 / bs - m.cwiseProduct(m);
                double acc = 0.0; int nd = 0;
                for (int d = 0; d < dim_; ++d) { if (double(g_var_[d]) <= 1e-5 || s2[d] <= 1e-12) continue; const double e = double(obs_[d]) - m[d]; acc += e * e / s2[d]; ++nd; }
                if (nd > 0) { nis_ = acc / nd; nis_ema_ += 0.01 * (nis_ - nis_ema_); nis_sum_ += nis_; ++nis_n_; }
            }
        }
        std::vector<double> ll(static_cast<size_t>(N), 0.0);
        double mx = -1e300;
        for (int j = 0; j < N; ++j) {
            const bool own = n_[size_t(j)] >= double(min_count_);
            const Eigen::VectorXf& m = own ? mu_[size_t(j)] : g_mu_;
            const Eigen::VectorXf v = (own ? head_var(j) : g_var_).cwiseMax(vfl);
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
    if (map_ != prev_map_) { if (prev_map_ >= 0) ++map_switches_; prev_map_ = map_; }
    if (winner_fresh_ && winner_ != prev_winner_) { if (prev_winner_ >= 0) ++winner_switches_; prev_winner_ = winner_; }

    // ---- learn from the EPM's label: transitions, and the winner's decode head ----
    if (winner_fresh_ && have_obs && n_[size_t(idx_[winner_])] >= double(min_count_)) {   // the winner's head, before it learns this reading (diag only)
        const int wi = idx_[winner_]; const Eigen::VectorXf vfl = (g_var_ * float(var_rel_floor_)).cwiseMax(1e-6f), v = head_var(wi).cwiseMax(vfl);
        double acc = 0.0; int nd = 0;
        for (int d = 0; d < dim_; ++d) { if (double(g_var_[d]) <= 1e-5) continue; const double e = double(obs_[d] - mu_[size_t(wi)][d]); acc += e * e / double(v[d]); ++nd; }
        if (nd > 0) { head_nis_ = acc / nd; head_nis_sum_ += head_nis_; ++head_nis_n_; }
    }
    if (winner_fresh_) {
        const int wi = idx_[winner_];
        if (last_winner_ >= 0 && idx_.count(last_winner_)) C_[size_t(idx_[last_winner_])][size_t(wi)] += 1.0;
        last_winner_ = winner_;
        if (have_obs && head_visits_) {   // a visit's ticks accumulate; it folds into the head when the winner moves on
            if (winner_ != vis_j_) { close_visit(); vis_j_ = winner_; vis_len_ = 0.0; vis_sum_ = Eigen::VectorXd::Zero(dim_); vis_ss_ = Eigen::VectorXd::Zero(dim_); }
            const Eigen::VectorXd o = obs_.cast<double>(); vis_sum_ += o; vis_ss_ += o.cwiseProduct(o); vis_len_ += 1.0;
        }
        else if (have_obs && head_own_) {   // the head's own samples: exact running mean and variance, then the drift floor
            const double nn = (n_[size_t(wi)] += 1.0);
            if (nn <= 1.0) { mu_[size_t(wi)] = obs_; var_[size_t(wi)] = Eigen::VectorXf::Zero(dim_); }
            else {
                const float g = float(std::max(1.0 / nn, gain_floor_));
                const Eigen::VectorXf dl = obs_ - mu_[size_t(wi)];
                mu_[size_t(wi)] += g * dl;
                var_[size_t(wi)] = (1.0f - g) * (var_[size_t(wi)] + g * dl.array().square().matrix());
            }
        }
        else if (have_obs) {
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
                          {"agree_with_epm", compared_ > 0 ? double(agree_) / compared_ : 0.0},
                          {"nis", nis_}, {"nis_ema", nis_ema_}, {"nis_mean", nis_mean()}, {"nis_n", nis_n_},
                          {"head_nis", head_nis_}, {"head_nis_mean", head_nis_mean()},
                          {"map_switches", map_switches_}, {"winner_switches", winner_switches_}};
}

namespace {
std::vector<double> vecd(const Eigen::VectorXf& v) { return std::vector<double>(v.data(), v.data() + v.size()); }
Eigen::VectorXf vecf(const nlohmann::json& j) { const auto d = j.get<std::vector<double>>(); Eigen::VectorXf v(int(d.size())); for (int i = 0; i < int(d.size()); ++i) v[i] = float(d[size_t(i)]); return v; }
}

// the learned model across runs (learned stepper Phase 3): nodes, decode heads, transition counts, the belief
nlohmann::json BeliefFilter::snapshot_state() const {
    nlohmann::json mu = nlohmann::json::array(), var = nlohmann::json::array();
    for (size_t i = 0; i < ids_.size(); ++i) { mu.push_back(vecd(mu_[i])); var.push_back(vecd(var_[i])); }
    nlohmann::json s{{"version", 1}, {"dim", dim_}, {"ids", ids_}, {"mu", mu}, {"var", var}, {"n", n_}, {"C", C_}, {"b", b_},
                     {"last_winner", last_winner_}, {"g_mu", vecd(g_mu_)}, {"g_var", vecd(g_var_)}, {"g_n", g_n_}};
    if (head_visits_) {   // emitted only in "visits" mode (byte-identical otherwise); the open visit is closed at the run's end, not carried
        nlohmann::json w = nlohmann::json::array(); for (auto const& v : wss_) w.push_back(vecd(v));
        s["wss"] = w; s["wT"] = wT_;
    }
    return s;
}

void BeliefFilter::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    if (s.value("version", 0) != 1) throw std::invalid_argument("BeliefFilter::restore_state: unknown version");
    dim_ = s.at("dim").get<int>(); ids_ = s.at("ids").get<std::vector<int>>();
    mu_.clear(); var_.clear();
    for (auto const& m : s.at("mu")) mu_.push_back(vecf(m));
    for (auto const& v : s.at("var")) var_.push_back(vecf(v));
    n_ = s.at("n").get<std::vector<double>>(); C_ = s.at("C").get<std::vector<std::vector<double>>>(); b_ = s.at("b").get<std::vector<double>>();
    last_winner_ = s.at("last_winner").get<int>(); g_mu_ = vecf(s.at("g_mu")); g_var_ = vecf(s.at("g_var")); g_n_ = s.at("g_n").get<double>();
    idx_.clear(); for (int k = 0; k < int(ids_.size()); ++k) idx_[ids_[size_t(k)]] = k;
    wss_.clear(); wT_.assign(ids_.size(), 0.0);
    if (s.contains("wss")) { for (auto const& v : s.at("wss")) wss_.push_back(vecf(v)); wT_ = s.at("wT").get<std::vector<double>>(); }
    else wss_.assign(ids_.size(), Eigen::VectorXf::Zero(dim_));
    vis_j_ = -1; vis_len_ = 0.0;
    if (wss_.size() != ids_.size() || wT_.size() != ids_.size()) throw std::invalid_argument("BeliefFilter::restore_state: inconsistent visit sizes");
    if (mu_.size() != ids_.size() || var_.size() != ids_.size() || n_.size() != ids_.size() || C_.size() != ids_.size() || b_.size() != ids_.size())
        throw std::invalid_argument("BeliefFilter::restore_state: inconsistent sizes");
}

} // namespace ogma
