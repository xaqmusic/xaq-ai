#include "ogma/modules/Generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <iostream>

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
    throw std::invalid_argument("Generator param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("Generator param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("Generator param '" + key + "' must be a string");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    if (auto p = std::get_if<std::string>(&v)) return {*p};
    throw std::invalid_argument("Generator param '" + key + "' must be a list of strings");
}
std::vector<double> to_vec(Eigen::VectorXf const& v) { return std::vector<double>(v.data(), v.data() + v.size()); }
std::vector<double> per_dim(Eigen::VectorXd const& sum, double n) {
    std::vector<double> out;
    if (n <= 0.0) return out;
    for (int i = 0; i < sum.size(); ++i) out.push_back(sum[i] / n);
    return out;
}

} // namespace

Generator::Generator()  = default;
Generator::~Generator() = default;

std::string_view Generator::type_name() const { return "Generator"; }

std::vector<TopicSpec> Generator::input_topics() const {
    std::vector<TopicSpec> v{TopicSpec{reality_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false}};
    for (auto const& t : target_topics_)
        v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> Generator::output_topics() const {
    return {TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))}};
}

ParamSchema Generator::params_schema() const {
    return {
        {"reality_topic", ParamMutability::ConstructionOnly,
         "The EPM whose nodes this generator decodes (its RealityToken topic).", ParamValue{std::string("reality.bodypose.pose")}},
        {"target_topics", ParamMutability::ConstructionOnly,
         "ProprioToken topics, concatenated in order, forming the target each node predicts (in their own units).",
         ParamValue{std::vector<std::string>{}}},
        {"output_topic", ParamMutability::ConstructionOnly,
         "Published ProprioToken: [predicted target..., predicted standard deviation...] for the current winner. Empty = gen.<id>.",
         ParamValue{std::string("")}},
        {"horizon", ParamMutability::ConstructionOnly,
         "Ticks ahead: a node predicts the target this many ticks after it wins (0 = decode the present).", ParamValue{int64_t{0}}},
        {"gain_cap", ParamMutability::HotMutable,
         "Upper clamp on the per-node Kalman-schedule gain 1/(n+1) (the uncapped head chases a moving input).", ParamValue{0.2}},
        {"gain_floor", ParamMutability::HotMutable,
         "Lower clamp on the gain: a small process noise so a node's estimate can follow drift.", ParamValue{0.002}},
        {"var_rel_floor", ParamMutability::HotMutable,
         "A node's variance is never below this fraction of the target's global variance: a node that saw a binary target "
         "constant for a long time must not claim certainty that one exception turns into a huge normalised error "
         "(G1, 2026-10-24: one foot-contact channel read NIS 45-90 without it).", ParamValue{0.01}},
        {"report_every", ParamMutability::HotMutable,
         "> 0: print {\"event\":\"GEN_DIAG\", \"id\": ..., diag...} on stdout every N ticks, for headless runs. 0 = off.",
         ParamValue{int64_t{0}}},
        {"warmup", ParamMutability::HotMutable,
         "A node's predictions are scored only after it has this many updates (its first predictions are the global prior).",
         ParamValue{int64_t{0}}},
    };
}

ParamMap Generator::current_params() const {
    return {
        {"reality_topic", ParamValue{reality_topic_}}, {"target_topics", ParamValue{target_topics_}},
        {"output_topic", ParamValue{output_topic_}},   {"horizon", ParamValue{int64_t(horizon_)}},
        {"gain_cap", ParamValue{gain_cap_}},           {"gain_floor", ParamValue{gain_floor_}},
        {"warmup", ParamValue{int64_t(warmup_)}}, {"report_every", ParamValue{int64_t(report_every_)}},
        {"var_rel_floor", ParamValue{var_rel_floor_}},
    };
}

void Generator::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("Generator requires a non-null Bus");
    apply_param(params, "reality_topic", [&](auto const& v){ reality_topic_ = get_string(v, "reality_topic"); });
    apply_param(params, "target_topics", [&](auto const& v){ target_topics_ = get_string_vec(v, "target_topics"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v, "output_topic"); });
    apply_param(params, "horizon",       [&](auto const& v){ horizon_       = std::max(0, int(get_int(v, "horizon"))); });
    apply_param(params, "gain_cap",      [&](auto const& v){ gain_cap_      = get_double(v, "gain_cap"); });
    apply_param(params, "gain_floor",    [&](auto const& v){ gain_floor_    = get_double(v, "gain_floor"); });
    apply_param(params, "warmup",        [&](auto const& v){ warmup_        = std::max(0, int(get_int(v, "warmup"))); });
    apply_param(params, "report_every",  [&](auto const& v){ report_every_  = std::max(0, int(get_int(v, "report_every"))); });
    apply_param(params, "var_rel_floor", [&](auto const& v){ var_rel_floor_ = std::max(0.0, get_double(v, "var_rel_floor")); });
    if (target_topics_.empty()) throw std::invalid_argument("Generator: target_topics must name at least one topic");
    if (output_topic_.empty()) output_topic_ = "gen." + (id().empty() ? std::string("generator") : std::string(id()));
    target_parts_.assign(target_topics_.size(), Eigen::VectorXf());
    part_fresh_.assign(target_topics_.size(), false);

    sub_ids_.push_back(bus_->subscribe(reality_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_reality(p); }));
    for (size_t i = 0; i < target_topics_.size(); ++i)
        sub_ids_.push_back(bus_->subscribe(target_topics_[i], SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){ handle_target(i, p); }));
}

void Generator::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "gain_cap")   gain_cap_   = get_double(value, k);
    else if (k == "gain_floor") gain_floor_ = get_double(value, k);
    else if (k == "warmup")     warmup_     = std::max(0, int(get_int(value, k)));
    else if (k == "report_every") report_every_ = std::max(0, int(get_int(value, k)));
    else if (k == "var_rel_floor") var_rel_floor_ = std::max(0.0, get_double(value, k));
    else throw std::invalid_argument("Generator param '" + k + "' is ConstructionOnly or unknown");
}

void Generator::handle_reality(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(p);
    if (!rt) return;
    winner_ = rt->winner_id;
    for (int k : rt->pruned_ids) nodes_.erase(k);
}

void Generator::handle_target(size_t idx, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() == 0) return;
    target_parts_[idx] = pt->values;
    part_fresh_[idx] = true;
}

Generator::NodeStat& Generator::node(int k) {
    auto it = nodes_.find(k);
    if (it != nodes_.end()) return it->second;
    NodeStat s;                       // a node not yet seen starts from the target's global statistics
    s.mu  = g_mu_;
    s.var = g_var_.cwiseMax(float(var_floor_));
    return nodes_.emplace(k, std::move(s)).first->second;
}

Eigen::VectorXf Generator::node_mean(int k) const { auto it = nodes_.find(k); return it == nodes_.end() ? Eigen::VectorXf() : it->second.mu; }
Eigen::VectorXf Generator::node_var(int k)  const { auto it = nodes_.find(k); return it == nodes_.end() ? Eigen::VectorXf() : it->second.var; }

void Generator::tick(uint64_t tick_id) {
    ++ticks_;
    if (report_every_ > 0 && ticks_ % report_every_ == 0) {
        auto d = diag_snapshot();
        d["event"] = "GEN_DIAG"; d["id"] = std::string(id()); d["tick"] = ticks_;
        std::cout << d.dump() << std::endl;
    }
    bool any_fresh = false, all_set = true;
    for (size_t i = 0; i < target_parts_.size(); ++i) {
        any_fresh = any_fresh || part_fresh_[i];
        all_set = all_set && target_parts_[i].size() > 0;
    }
    if (!all_set || !any_fresh) return;   // nothing new to learn from or score against this tick
    std::fill(part_fresh_.begin(), part_fresh_.end(), false);

    int d = 0;
    for (auto const& v : target_parts_) d += int(v.size());
    Eigen::VectorXf x(d);
    for (int i = 0, o = 0; i < int(target_parts_.size()); ++i) { x.segment(o, target_parts_[i].size()) = target_parts_[i]; o += int(target_parts_[i].size()); }
    if (dim_ == 0) {
        dim_ = d;
        g_mu_ = x; g_var_ = Eigen::VectorXf::Constant(d, float(var_floor_));
    }
    if (d != dim_) return;   // a target changed width: ignore rather than mix

    const Eigen::VectorXf prev_target = past_targets_.empty() ? x : past_targets_.back();

    // 1. The new prediction, for the node winning now.
    if (winner_ >= 0) {
        NodeStat& s = node(winner_);
        // The PREDICTIVE variance: the node's spread plus the uncertainty of its own mean (var / n).
        const Eigen::VectorXf vfl = (g_var_ * float(var_rel_floor_)).cwiseMax(float(var_floor_));
        const Eigen::VectorXf pvar = s.var.cwiseMax(vfl) * float(1.0 + 1.0 / std::max(1.0, s.n));
        Pending pnd{winner_, s.mu, pvar, horizon_ == 0 ? prev_target : x};
        if (s.n < double(warmup_)) pnd.winner = -1 - winner_;   // marks "not scored" while keeping the id
        pending_.push_back(std::move(pnd));
        auto out = std::make_shared<ProprioToken>();
        out->tick_id = tick_id;
        out->producer_id = id().empty() ? std::string("generator") : std::string(id());
        out->sensor = "generator";
        out->values.resize(2 * dim_);
        out->values.head(dim_) = s.mu;
        out->values.tail(dim_) = pvar.cwiseSqrt();
        bus_->publish(output_topic_, out);
        ++predicted_;
    }

    // 2. Score and learn from the prediction made `horizon` ticks ago, now that its target is here.
    while (int(pending_.size()) > horizon_) {
        Pending p = std::move(pending_.front());
        pending_.pop_front();
        const bool scored = p.winner >= 0;
        const int  k = scored ? p.winner : -1 - p.winner;
        if (scored) {
            const Eigen::VectorXf e = x - p.mu;
            const Eigen::VectorXf em = x - g_mu_;
            const Eigen::VectorXf ep = x - p.pers;
            mse_sum_       += double(e.squaredNorm())  / dim_;
            base_mean_sum_ += double(em.squaredNorm()) / dim_;
            base_pers_sum_ += double(ep.squaredNorm()) / dim_;
            nis_sum_       += double((e.array().square() / p.var.array().max(float(var_floor_))).sum()) / dim_;
            mse_n_ += 1.0;
            if (mse_dim_.size() != dim_) {
                mse_dim_ = Eigen::VectorXd::Zero(dim_); nis_dim_ = Eigen::VectorXd::Zero(dim_);
                base_mean_dim_ = Eigen::VectorXd::Zero(dim_); base_pers_dim_ = Eigen::VectorXd::Zero(dim_);
            }
            mse_dim_       += e.array().square().cast<double>().matrix();
            nis_dim_       += (e.array().square() / p.var.array().max(float(var_floor_))).cast<double>().matrix();
            base_mean_dim_ += em.array().square().cast<double>().matrix();
            base_pers_dim_ += ep.array().square().cast<double>().matrix();
        }
        auto it = nodes_.find(k);
        if (it != nodes_.end()) {                         // pruned in the meantime: nothing to learn into
            NodeStat& s = it->second;
            s.n += 1.0;
            const float g = float(std::clamp(1.0 / (s.n + 1.0), gain_floor_, gain_cap_));
            const Eigen::VectorXf dlt = x - s.mu;
            s.mu  += g * dlt;
            s.var += g * (dlt.array().square().matrix() - s.var);
            s.var  = s.var.cwiseMax(float(var_floor_));
            s.err_ema += 0.05 * (double(dlt.squaredNorm()) / dim_ - s.err_ema);
        }
    }

    // 3. The global statistics (the prior for new nodes, and the mean baseline), after scoring.
    g_n_ += 1.0;
    const float gg = float(std::max(1.0 / (g_n_ + 1.0), gain_floor_));
    const Eigen::VectorXf dg = x - g_mu_;
    g_mu_  += gg * dg;
    g_var_ += gg * (dg.array().square().matrix() - g_var_);
    g_var_  = g_var_.cwiseMax(float(var_floor_));

    past_targets_.push_back(x);
    while (int(past_targets_.size()) > horizon_ + 2) past_targets_.pop_front();
}

nlohmann::json Generator::diag_snapshot() const {
    const double m = mse(), bm = mse_mean_baseline(), bp = mse_persistence();
    double worst = 0.0; int worst_k = -1;
    for (auto const& [k, s] : nodes_) if (s.n >= 20 && s.err_ema > worst) { worst = s.err_ema; worst_k = k; }
    return nlohmann::json{
        {"nodes_tracked", nodes_tracked()}, {"scored", scored()}, {"predicted", predicted_}, {"ticks", ticks_},
        {"horizon", horizon_}, {"dim", dim_},
        {"mse", m}, {"mse_mean_baseline", bm}, {"mse_persistence", bp},
        {"skill_vs_mean", bm > 0 ? 1.0 - m / bm : 0.0},          // > 0: the nodes know more than the average
        {"skill_vs_persistence", bp > 0 ? 1.0 - m / bp : 0.0},   // > 0: better than "the same as before"
        {"nis", nis()},                                           // ~1 when the stated variance is honest
        {"global_mean", to_vec(g_mu_)}, {"worst_node", worst_k}, {"worst_node_err", worst},
        {"mse_dim", per_dim(mse_dim_, mse_n_)}, {"nis_dim", per_dim(nis_dim_, mse_n_)},
        {"mean_base_dim", per_dim(base_mean_dim_, mse_n_)}, {"pers_base_dim", per_dim(base_pers_dim_, mse_n_)}};
}

} // namespace ogma
