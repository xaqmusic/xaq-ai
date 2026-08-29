#include "ogma/modules/GradientEPM.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ogma {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float wrap_pi(float a) { return std::atan2(std::sin(a), std::cos(a)); }

template <class Fn>
void apply_param(ParamMap const& params, char const* key, Fn fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("GradientEPM: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("GradientEPM: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("GradientEPM: param '" + k + "' must be a string");
}
bool get_bool(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("GradientEPM: param '" + k + "' must be bool");
}
}  // namespace

std::string_view GradientEPM::type_name() const { return "GradientEPM"; }

std::vector<TopicSpec> GradientEPM::input_topics() const {
    return {
        TopicSpec{consensus_topic_, std::type_index(typeid(ConsensusToken)), SubscriptionKind::Direct, false},
        TopicSpec{scalar_topic_,    std::type_index(typeid(ProprioToken)),   SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_,   std::type_index(typeid(ProprioToken)),   SubscriptionKind::Direct, false},
        TopicSpec{hit_topic_,       std::type_index(typeid(EnvEvent)),       SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> GradientEPM::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema GradientEPM::params_schema() const {
    return {
        {"consensus_topic", ParamMutability::ConstructionOnly, "Loop voter consensus (ConsensusToken).", ParamValue{std::string("consensus.scent_loop")}},
        {"scalar_topic",    ParamMutability::ConstructionOnly, "Direct scalar (the gradient to follow).", ParamValue{std::string("reality.proprio.scent_max")}},
        {"heading_topic",   ParamMutability::ConstructionOnly, "Egomotion heading (orientation frame).", ParamValue{std::string("reality.proprio.heading")}},
        {"hit_topic",       ParamMutability::ConstructionOnly, "Eat event — own reward.", ParamValue{std::string("events.hit")}},
        {"output_topic",    ParamMutability::ConstructionOnly, "Chosen heading [vx,vy] → HeadingController.", ParamValue{std::string("percept.gradient_heading")}},
        {"proj_dim",        ParamMutability::ConstructionOnly, "Consensus random-projection dim.", ParamValue{int64_t{16}}},
        {"n_headings",      ParamMutability::HotMutable, "Candidate headings swept at select.", ParamValue{int64_t{12}}},
        {"cycle_ticks",     ParamMutability::HotMutable, "Commit window (Δscalar measured over this).", ParamValue{int64_t{30}}},
        {"value_lr",        ParamMutability::HotMutable, "pred-Δscalar EMA rate.", ParamValue{0.15}},
        {"tle_lr",          ParamMutability::HotMutable, "consistency (TLE) EMA rate.", ParamValue{0.1}},
        {"reward_norm",     ParamMutability::HotMutable, "Whiten Δscalar by running scale.", ParamValue{true}},
        {"reward_scale_lr", ParamMutability::HotMutable, "Running |Δ| scale EMA rate.", ParamValue{0.02}},
        {"r_hit",           ParamMutability::HotMutable, "Reward on an eat-in-cycle.", ParamValue{1.0}},
        {"insertion_dist",  ParamMutability::HotMutable, "GNG: insert if nearest farther than this.", ParamValue{0.6}},
        {"max_nodes",       ParamMutability::HotMutable, "Node cap.", ParamValue{int64_t{64}}},
        {"adapt_lr",        ParamMutability::HotMutable, "GNG winner move rate.", ParamValue{0.05}},
        {"bake_visits",     ParamMutability::HotMutable, "Min visits to bake a node.", ParamValue{int64_t{8}}},
        {"bake_tle",        ParamMutability::HotMutable, "Max TLE to bake.", ParamValue{0.5}},
        {"temperature",     ParamMutability::HotMutable, "Softmax over candidate Δscalar; <=0 = argmax.", ParamValue{0.3}},
        {"epistemic_gain",  ParamMutability::HotMutable, "Novelty bonus for under-modeled headings.", ParamValue{0.4}},
        {"trend_ema",       ParamMutability::HotMutable, "Scalar-trend EMA rate.", ParamValue{0.2}},
        {"mode",            ParamMutability::HotMutable, "+1 follow (climb) / -1 flee.", ParamValue{int64_t{1}}},
        {"w_head",          ParamMutability::HotMutable, "Heading weight in feature distance.", ParamValue{1.0}},
        {"master_seed",     ParamMutability::ConstructionOnly, "RNG seed.", ParamValue{int64_t{7}}},
    };
}

ParamMap GradientEPM::current_params() const {
    ParamMap m;
    m["consensus_topic"] = ParamValue{consensus_topic_};
    m["scalar_topic"] = ParamValue{scalar_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["hit_topic"] = ParamValue{hit_topic_};
    m["output_topic"] = ParamValue{output_topic_};
    m["proj_dim"] = ParamValue{int64_t(proj_dim_)};
    m["n_headings"] = ParamValue{int64_t(n_headings_)};
    m["cycle_ticks"] = ParamValue{int64_t(cycle_ticks_)};
    m["value_lr"] = ParamValue{double(value_lr_)};
    m["tle_lr"] = ParamValue{double(tle_lr_)};
    m["reward_norm"] = ParamValue{reward_norm_};
    m["reward_scale_lr"] = ParamValue{double(reward_scale_lr_)};
    m["r_hit"] = ParamValue{double(r_hit_)};
    m["insertion_dist"] = ParamValue{double(insertion_dist_)};
    m["max_nodes"] = ParamValue{int64_t(max_nodes_)};
    m["adapt_lr"] = ParamValue{double(adapt_lr_)};
    m["bake_visits"] = ParamValue{int64_t(bake_visits_)};
    m["bake_tle"] = ParamValue{double(bake_tle_)};
    m["temperature"] = ParamValue{double(temperature_)};
    m["epistemic_gain"] = ParamValue{double(epistemic_gain_)};
    m["trend_ema"] = ParamValue{double(trend_ema_)};
    m["mode"] = ParamValue{int64_t(mode_)};
    m["w_head"] = ParamValue{double(w_head_)};
    m["master_seed"] = ParamValue{int64_t(master_seed_)};
    return m;
}

void GradientEPM::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "n_headings")     n_headings_     = std::max(2, int(get_int(value, k)));
    else if (k == "cycle_ticks")    cycle_ticks_    = std::max(1, int(get_int(value, k)));
    else if (k == "value_lr")       value_lr_       = float(get_double(value, k));
    else if (k == "tle_lr")         tle_lr_         = float(get_double(value, k));
    else if (k == "reward_norm")    reward_norm_    = get_bool(value, k);
    else if (k == "reward_scale_lr")reward_scale_lr_= float(get_double(value, k));
    else if (k == "r_hit")          r_hit_          = float(get_double(value, k));
    else if (k == "insertion_dist") insertion_dist_ = float(get_double(value, k));
    else if (k == "max_nodes")      max_nodes_      = std::max(1, int(get_int(value, k)));
    else if (k == "adapt_lr")       adapt_lr_       = float(get_double(value, k));
    else if (k == "bake_visits")    bake_visits_    = std::max(1, int(get_int(value, k)));
    else if (k == "bake_tle")       bake_tle_       = float(get_double(value, k));
    else if (k == "temperature")    temperature_    = float(get_double(value, k));
    else if (k == "epistemic_gain") epistemic_gain_ = float(get_double(value, k));
    else if (k == "trend_ema")      trend_ema_      = float(get_double(value, k));
    else if (k == "mode")           mode_           = int(get_int(value, k)) >= 0 ? 1 : -1;
    else if (k == "w_head")         w_head_         = float(get_double(value, k));
    else throw std::invalid_argument("GradientEPM: param '" + k + "' is construction-only / unknown");
}

void GradientEPM::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "consensus_topic", [&](auto const& v){ consensus_topic_ = get_string(v,"consensus_topic"); });
    apply_param(params, "scalar_topic",    [&](auto const& v){ scalar_topic_    = get_string(v,"scalar_topic"); });
    apply_param(params, "heading_topic",   [&](auto const& v){ heading_topic_   = get_string(v,"heading_topic"); });
    apply_param(params, "hit_topic",       [&](auto const& v){ hit_topic_       = get_string(v,"hit_topic"); });
    apply_param(params, "output_topic",    [&](auto const& v){ output_topic_    = get_string(v,"output_topic"); });
    apply_param(params, "proj_dim",        [&](auto const& v){ proj_dim_        = std::max(1, int(get_int(v,"proj_dim"))); });
    apply_param(params, "n_headings",      [&](auto const& v){ n_headings_      = std::max(2, int(get_int(v,"n_headings"))); });
    apply_param(params, "cycle_ticks",     [&](auto const& v){ cycle_ticks_     = std::max(1, int(get_int(v,"cycle_ticks"))); });
    apply_param(params, "value_lr",        [&](auto const& v){ value_lr_        = float(get_double(v,"value_lr")); });
    apply_param(params, "tle_lr",          [&](auto const& v){ tle_lr_          = float(get_double(v,"tle_lr")); });
    apply_param(params, "reward_norm",     [&](auto const& v){ reward_norm_     = get_bool(v,"reward_norm"); });
    apply_param(params, "reward_scale_lr", [&](auto const& v){ reward_scale_lr_ = float(get_double(v,"reward_scale_lr")); });
    apply_param(params, "r_hit",           [&](auto const& v){ r_hit_           = float(get_double(v,"r_hit")); });
    apply_param(params, "insertion_dist",  [&](auto const& v){ insertion_dist_  = float(get_double(v,"insertion_dist")); });
    apply_param(params, "max_nodes",       [&](auto const& v){ max_nodes_       = std::max(1, int(get_int(v,"max_nodes"))); });
    apply_param(params, "adapt_lr",        [&](auto const& v){ adapt_lr_        = float(get_double(v,"adapt_lr")); });
    apply_param(params, "bake_visits",     [&](auto const& v){ bake_visits_     = std::max(1, int(get_int(v,"bake_visits"))); });
    apply_param(params, "bake_tle",        [&](auto const& v){ bake_tle_        = float(get_double(v,"bake_tle")); });
    apply_param(params, "temperature",     [&](auto const& v){ temperature_     = float(get_double(v,"temperature")); });
    apply_param(params, "epistemic_gain",  [&](auto const& v){ epistemic_gain_  = float(get_double(v,"epistemic_gain")); });
    apply_param(params, "trend_ema",       [&](auto const& v){ trend_ema_       = float(get_double(v,"trend_ema")); });
    apply_param(params, "mode",            [&](auto const& v){ mode_            = int(get_int(v,"mode")) >= 0 ? 1 : -1; });
    apply_param(params, "w_head",          [&](auto const& v){ w_head_          = float(get_double(v,"w_head")); });
    apply_param(params, "master_seed",     [&](auto const& v){ master_seed_     = uint64_t(get_int(v,"master_seed")); });

    rng_.seed(master_seed_);
    proj_c_ = Eigen::VectorXf::Zero(proj_dim_);   // valid feature before the first consensus
    ticks_left_ = 0;                              // first tick selects

    if (!consensus_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(consensus_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_consensus(p); }));
    if (!scalar_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(scalar_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_scalar(p); }));
    if (!heading_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_heading(p); }));
    if (!hit_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(hit_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_hit(p); }));
}

Eigen::VectorXf GradientEPM::project(Eigen::VectorXf const& c) {
    if (proj_R_.rows() == 0) {                      // lazy init once consensus dim is known
        consensus_dim_ = int(c.size());
        proj_R_.resize(proj_dim_, consensus_dim_);
        std::normal_distribution<float> g(0.0f, 1.0f);
        for (int i = 0; i < proj_dim_; ++i)
            for (int j = 0; j < consensus_dim_; ++j) proj_R_(i, j) = g(rng_);
    }
    if (int(c.size()) != consensus_dim_) return Eigen::VectorXf::Zero(proj_dim_);
    Eigen::VectorXf p = proj_R_ * c;
    float n = p.norm();
    if (n > 1e-6f) p /= n;                           // unit → fixed consensus magnitude (§6)
    return p;
}

Eigen::VectorXf GradientEPM::feature(Eigen::VectorXf const& proj_c, float trend, float a) const {
    Eigen::VectorXf x(proj_dim_ + 3);
    x.head(proj_dim_) = proj_c;
    x[proj_dim_]     = trend;
    x[proj_dim_ + 1] = w_head_ * std::sin(a);
    x[proj_dim_ + 2] = w_head_ * std::cos(a);
    return x;
}

int GradientEPM::nearest(Eigen::VectorXf const& x, bool baked_only) const {
    int best = -1; float bd = 1e30f;
    for (int i = 0; i < int(nodes_.size()); ++i) {
        if (baked_only && !nodes_[i].baked) continue;
        float d = (nodes_[i].proto - x).squaredNorm();
        if (d < bd) { bd = d; best = i; }
    }
    return best;
}

void GradientEPM::handle_consensus(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (ct && ct->fused_embedding.size() > 0) { proj_c_ = project(ct->fused_embedding); have_consensus_ = true; }
}
void GradientEPM::handle_scalar(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { scalar_ = float(pt->values[0]); have_scalar_ = true; }
}
void GradientEPM::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) heading_ = float(pt->values[0]);
}
void GradientEPM::handle_hit(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    if (auto e = std::dynamic_pointer_cast<const EnvEvent>(payload))
        if (e->name == "hit" || e->name.empty()) hit_in_cycle_ = true;
}

float GradientEPM::select_heading() {
    if (nodes_.empty()) { std::uniform_real_distribution<float> u(-kPi, kPi); last_pred_ = 0.0f; return u(rng_); }
    std::vector<float> head(n_headings_), pred(n_headings_), score(n_headings_);
    for (int i = 0; i < n_headings_; ++i) {
        float a = -kPi + 2.0f * kPi * float(i) / float(n_headings_);
        head[i] = a;
        Eigen::VectorXf x = feature(proj_c_, trend_, a);
        int nb = nearest(x, true);
        int na = (nb >= 0) ? nb : nearest(x, false);
        float p   = (na >= 0) ? nodes_[na].pred : 0.0f;
        float vis = (na >= 0) ? float(nodes_[na].visits) : 0.0f;
        pred[i]  = p;
        score[i] = float(mode_) * p + epistemic_gain_ / (1.0f + vis);   // value + novelty
    }
    int ci = 0;
    if (temperature_ > 0.0f) {
        float mx = *std::max_element(score.begin(), score.end());
        std::vector<float> w(n_headings_); float Z = 0.0f;
        for (int i = 0; i < n_headings_; ++i) { w[i] = std::exp((score[i] - mx) / temperature_); Z += w[i]; }
        std::uniform_real_distribution<float> u(0.0f, Z); float r = u(rng_), acc = 0.0f;
        ci = n_headings_ - 1;
        for (int i = 0; i < n_headings_; ++i) { acc += w[i]; if (r <= acc) { ci = i; break; } }
    } else {
        for (int i = 1; i < n_headings_; ++i) if (score[i] > score[ci]) ci = i;
    }
    last_pred_ = pred[ci];
    return head[ci];
}

void GradientEPM::credit_and_adapt() {
    if (!have_commit_) return;
    float raw = scalar_ - commit_scalar_;
    float ds;
    if (hit_in_cycle_) {
        ds = r_hit_;
    } else if (reward_norm_) {
        if (reward_scale_ <= 0.0f) reward_scale_ = std::max(std::fabs(raw), 1e-6f);
        else reward_scale_ += reward_scale_lr_ * (std::fabs(raw) - reward_scale_);
        ds = raw / (reward_scale_ + 1e-6f);
    } else {
        ds = raw;
    }
    last_dscalar_ = ds;
    int nb = nearest(commit_feat_, false);
    float d = (nb >= 0) ? (nodes_[nb].proto - commit_feat_).norm() : 1e30f;
    if (nb < 0 || (d > insertion_dist_ && int(nodes_.size()) < max_nodes_)) {
        Node n; n.proto = commit_feat_; n.pred = ds; n.tle = 0.0f; n.visits = 1; n.baked = false;
        nodes_.push_back(std::move(n));
    } else {
        Node& n = nodes_[nb];
        n.proto += adapt_lr_ * (commit_feat_ - n.proto);
        float err = std::fabs(ds - n.pred);
        n.pred += value_lr_ * (ds - n.pred);
        n.tle  += tle_lr_   * (err - n.tle);
        n.visits += 1;
        if (n.visits >= bake_visits_ && n.tle <= bake_tle_) n.baked = true;
    }
}

void GradientEPM::tick(uint64_t tick_id) {
    if (have_scalar_) {
        if (have_prev_) { float inst = scalar_ - prev_scalar_; trend_ += trend_ema_ * (inst - trend_); }
        prev_scalar_ = scalar_; have_prev_ = true;
    }

    if (--ticks_left_ <= 0) {
        credit_and_adapt();
        committed_heading_ = select_heading();
        commit_feat_   = feature(proj_c_, trend_, committed_heading_);
        commit_scalar_ = scalar_;
        have_commit_   = true;
        ticks_left_    = cycle_ticks_;
        hit_in_cycle_  = false;
    }

    float delta = wrap_pi(committed_heading_ - heading_);
    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("gradient") : id_;
    out->sensor      = "gradient_heading";
    out->values.resize(2);
    out->values[0] = std::sin(delta);
    out->values[1] = std::cos(delta);
    bus_->publish(output_topic_, out);
}

int GradientEPM::baked_count() const {
    int n = 0; for (auto const& nd : nodes_) if (nd.baked) ++n; return n;
}

// This module's diag_snapshot() is already seven flat scalars — nothing in it grows with
// run time — so the high-rate payload is the same payload.  Without this override the
// module would publish `{}` to a "lite" subscriber while still matching xaq_voice's
// EPM-type filter, which is a silent mute rather than an error.
nlohmann::json GradientEPM::diag_lite() const { return diag_snapshot(); }

nlohmann::json GradientEPM::diag_snapshot() const {
    return nlohmann::json{
        {"nodes", int(nodes_.size())},
        {"baked", baked_count()},
        {"pred", last_pred_},
        {"dscalar", last_dscalar_},
        {"trend", trend_},
        {"heading", committed_heading_},
        {"have_consensus", have_consensus_},
    };
}

}  // namespace ogma
