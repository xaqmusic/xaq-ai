// =============================================================================
// LookLoop.cpp  --  see the header (the duck, design doc §17.112)
// =============================================================================
#include "ogma/modules/LookLoop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("LookLoop: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("LookLoop: param '" + k + "' must be a string");
}
float scalar(Bus* bus, std::string const& topic, int index, float fallback) {
    if (topic.empty()) return fallback;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus->last_value(topic));
    return (pt && pt->values.size() > index) ? pt->values[index] : fallback;
}
} // namespace

LookLoop::LookLoop()  = default;
LookLoop::~LookLoop() = default;

std::string_view LookLoop::type_name() const { return "LookLoop"; }

std::vector<TopicSpec> LookLoop::input_topics() const {
    std::vector<TopicSpec> v;
    v.push_back(TopicSpec{state_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!context_topic_.empty())      v.push_back(TopicSpec{context_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false});
    if (!active_topic_.empty())       v.push_back(TopicSpec{active_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!place_topic_.empty())        v.push_back(TopicSpec{place_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false});
    if (!seek_value_topic_.empty())   v.push_back(TopicSpec{seek_value_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!outcome_need_topic_.empty()) v.push_back(TopicSpec{outcome_need_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> LookLoop::output_topics() const {
    std::vector<TopicSpec> v;
    v.push_back(TopicSpec{value_topic_, std::type_index(typeid(ProprioToken))});
    if (!driving_topic_.empty())    v.push_back(TopicSpec{driving_topic_, std::type_index(typeid(ProprioToken))});
    if (!descriptor_topic_.empty()) v.push_back(TopicSpec{descriptor_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema LookLoop::params_schema() const {
    return {
        {"state_topic", ParamMutability::ConstructionOnly,
            "CloudMap's state_topic [open, walking, voxels, new_fraction]: a stop is a stop's cloud open (open, not walking); "
            "new_fraction is the cloud's surprise.", std::nullopt},
        {"value_topic", ParamMutability::ConstructionOnly,
            "Output [need, prediction / peak, in a stop, the last stop's realized surprise (s), its prediction (s)]; values[0] is "
            "the need the arbiter reads.", std::nullopt},
        {"context_topic", ParamMutability::ConstructionOnly,
            "A context EPM's RealityToken: its winner keys the prediction table.  Empty = one cell.", ParamValue{std::string("")}},
        {"descriptor_topic", ParamMutability::ConstructionOnly,
            "Output [place TLE / its running peak, seek need, outcome need] for the context EPM.  Empty = not published.",
            ParamValue{std::string("")}},
        {"place_topic", ParamMutability::ConstructionOnly, "The place EPM's RealityToken (its TLE: have I stood here?).", ParamValue{std::string("")}},
        {"seek_value_topic", ParamMutability::ConstructionOnly, "The seek loop's need [v].", ParamValue{std::string("")}},
        {"outcome_need_topic", ParamMutability::ConstructionOnly, "The outcome loop's need [v].", ParamValue{std::string("")}},
        {"active_topic", ParamMutability::ConstructionOnly,
            "The arbiter's gain for the look (arbiter.gain.look): a stop counts as the look's only while it is above 0.5.  "
            "Empty = passive: every stop's cloud is watched.", ParamValue{std::string("")}},
        {"driving_topic", ParamMutability::ConstructionOnly,
            "Output [1 in a stop, else 0]: a LoopCompetence's gain while the loop is passive.  Empty = not published.",
            ParamValue{std::string("")}},
        {"optimism", ParamMutability::HotMutable,
            "The prediction's credit to the unproven: mean + optimism x sd / sqrt(n) (an untried cell reads the peak).", ParamValue{1.0}},
        {"peak_decay", ParamMutability::HotMutable,
            "Per stop, the running peak of realized surprise decays this fraction toward the latest (its own scale, adapted).",
            ParamValue{0.02}},
        {"min_stop_ticks", ParamMutability::HotMutable, "A stop's cloud open fewer ticks than this is not a stop (a twitch).", ParamValue{int64_t{25}}},
        {"tick_hz", ParamMutability::ConstructionOnly, "The brain's tick rate (the integral is in seconds).", ParamValue{50.0}},
    };
}

ParamMap LookLoop::current_params() const {
    ParamMap m;
    m["state_topic"] = ParamValue{state_topic_}; m["value_topic"] = ParamValue{value_topic_};
    m["context_topic"] = ParamValue{context_topic_}; m["descriptor_topic"] = ParamValue{descriptor_topic_};
    m["place_topic"] = ParamValue{place_topic_}; m["seek_value_topic"] = ParamValue{seek_value_topic_};
    m["outcome_need_topic"] = ParamValue{outcome_need_topic_}; m["active_topic"] = ParamValue{active_topic_};
    m["driving_topic"] = ParamValue{driving_topic_};
    m["optimism"] = ParamValue{optimism_}; m["peak_decay"] = ParamValue{peak_decay_};
    m["min_stop_ticks"] = ParamValue{int64_t(min_stop_ticks_)}; m["tick_hz"] = ParamValue{tick_hz_};
    return m;
}

void LookLoop::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("LookLoop requires a non-null Bus");
    apply_param(params, "state_topic",        [&](auto const& v){ state_topic_ = get_string(v, "state_topic"); });
    apply_param(params, "value_topic",        [&](auto const& v){ value_topic_ = get_string(v, "value_topic"); });
    apply_param(params, "context_topic",      [&](auto const& v){ context_topic_ = get_string(v, "context_topic"); });
    apply_param(params, "descriptor_topic",   [&](auto const& v){ descriptor_topic_ = get_string(v, "descriptor_topic"); });
    apply_param(params, "place_topic",        [&](auto const& v){ place_topic_ = get_string(v, "place_topic"); });
    apply_param(params, "seek_value_topic",   [&](auto const& v){ seek_value_topic_ = get_string(v, "seek_value_topic"); });
    apply_param(params, "outcome_need_topic", [&](auto const& v){ outcome_need_topic_ = get_string(v, "outcome_need_topic"); });
    apply_param(params, "active_topic",       [&](auto const& v){ active_topic_ = get_string(v, "active_topic"); });
    apply_param(params, "driving_topic",      [&](auto const& v){ driving_topic_ = get_string(v, "driving_topic"); });
    apply_param(params, "optimism",           [&](auto const& v){ optimism_ = get_double(v, "optimism"); });
    apply_param(params, "peak_decay",         [&](auto const& v){ peak_decay_ = get_double(v, "peak_decay"); });
    apply_param(params, "min_stop_ticks",     [&](auto const& v){ min_stop_ticks_ = std::max(1, int(get_double(v, "min_stop_ticks"))); });
    apply_param(params, "tick_hz",            [&](auto const& v){ tick_hz_ = std::max(1.0, get_double(v, "tick_hz")); });
    if (state_topic_.empty()) throw std::invalid_argument("LookLoop: state_topic is required");
    if (value_topic_.empty()) throw std::invalid_argument("LookLoop: value_topic is required");
}

void LookLoop::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "optimism")       optimism_ = get_double(value, k);
    else if (k == "peak_decay")     peak_decay_ = get_double(value, k);
    else if (k == "min_stop_ticks") min_stop_ticks_ = std::max(1, int(get_double(value, k)));
    else throw std::invalid_argument("LookLoop: param '" + k + "' is construction-only / unknown");
}

int LookLoop::context_now() const {
    if (context_topic_.empty()) return 0;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(context_topic_));
    return rt ? std::max(0, rt->winner_id) : 0;
}

double LookLoop::predict(int cell) const {
    auto it = cells_.find(cell);
    if (it == cells_.end() || it->second.n == 0) return peak_;                       // untried: the best seen
    const Cell& c = it->second;
    const double sd = c.n >= 2 ? std::sqrt(c.m2 / double(c.n - 1)) : peak_;
    return c.mean + optimism_ * sd / std::sqrt(double(c.n));
}

void LookLoop::tick(uint64_t tick_id) {
    const bool open    = scalar(bus_, state_topic_, 0, 0.0f) > 0.5f;
    const bool walking = scalar(bus_, state_topic_, 1, 0.0f) > 0.5f;
    nf_ = std::clamp(double(scalar(bus_, state_topic_, 3, 0.0f)), 0.0, 1.0);
    const bool active = active_topic_.empty() || scalar(bus_, active_topic_, 0, 0.0f) > 0.5f;
    const bool in_now = open && !walking && active;

    // the context: the descriptor for the context EPM (published every tick), and the winner it last returned
    if (!descriptor_topic_.empty()) {
        float place = 0.0f;
        if (!place_topic_.empty())
            if (auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(place_topic_))) {
                place_peak_ = std::max(double(rt->tle), place_peak_ * (1.0 - 1e-4));
                place = place_peak_ > 0.0 ? float(std::clamp(double(rt->tle) / place_peak_, 0.0, 1.0)) : 0.0f;
            }
        auto d = std::make_shared<ProprioToken>();
        d->tick_id = tick_id; d->producer_id = id_.empty() ? std::string("look") : id_; d->sensor = "look_context";
        d->values = Eigen::VectorXf(3);
        d->values[0] = place;
        d->values[1] = std::clamp(scalar(bus_, seek_value_topic_, 0, 0.0f), 0.0f, 1.0f);
        d->values[2] = std::clamp(scalar(bus_, outcome_need_topic_, 0, 0.0f), 0.0f, 1.0f);
        bus_->publish(descriptor_topic_, d);
    }
    const int cell = context_now();

    if (in_now && !in_ep_) {                                   // a stop's cloud opens: the prediction is made now
        in_ep_ = true; ep_ticks_ = 0; ep_gain_ = 0.0; ep_nf_peak_ = 0.0; ep_cell_ = cell; ep_pred_ = predict(cell);
    }
    if (in_ep_ && in_now) {
        ++ep_ticks_; ep_gain_ += nf_ / tick_hz_; ep_nf_peak_ = std::max(ep_nf_peak_, nf_);
    }
    if (in_ep_ && !in_now) {                                   // the stop ended: the honest signal
        in_ep_ = false;
        if (ep_ticks_ >= min_stop_ticks_) {
            Cell& c = cells_[ep_cell_];
            ++c.n; const double d = ep_gain_ - c.mean; c.mean += d / double(c.n); c.m2 += d * (ep_gain_ - c.mean);
            peak_ = peak_ <= 0.0 ? ep_gain_ : std::max(ep_gain_, peak_ + peak_decay_ * (ep_gain_ - peak_));
            last_realized_ = ep_gain_; last_predicted_ = ep_pred_;
            abs_err_sum_ += std::fabs(ep_gain_ - ep_pred_);
            ++episodes_;
        }
    }

    pred_norm_ = peak_ > 0.0 ? std::clamp(predict(cell) / peak_, 0.0, 1.0) : 1.0;   // nothing seen yet: untried
    need_ = (in_ep_ && ep_nf_peak_ > 0.0) ? std::clamp(nf_ / ep_nf_peak_, 0.0, 1.0) : pred_norm_;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("look") : id_; out->sensor = "look";
    out->values = Eigen::VectorXf(5);
    out->values[0] = float(need_); out->values[1] = float(pred_norm_); out->values[2] = in_ep_ ? 1.0f : 0.0f;
    out->values[3] = float(last_realized_); out->values[4] = float(last_predicted_);
    bus_->publish(value_topic_, out);
    if (!driving_topic_.empty()) {
        auto g = std::make_shared<ProprioToken>();
        g->tick_id = tick_id; g->producer_id = out->producer_id; g->sensor = "look_driving";
        g->values = Eigen::VectorXf::Constant(1, in_ep_ ? 1.0f : 0.0f);
        bus_->publish(driving_topic_, g);
    }
}

nlohmann::json LookLoop::snapshot_state() const {
    nlohmann::json cells = nlohmann::json::array();
    for (auto const& [k, c] : cells_) cells.push_back({k, c.n, c.mean, c.m2});
    return nlohmann::json{{"version", 1}, {"cells", cells}, {"peak", peak_}, {"place_peak", place_peak_},
                          {"episodes", episodes_}, {"abs_err_sum", abs_err_sum_}};
}

void LookLoop::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    cells_.clear();
    for (auto const& c : s.value("cells", nlohmann::json::array()))
        if (c.is_array() && c.size() == 4) cells_[c[0].get<int>()] = Cell{c[1].get<int>(), c[2].get<double>(), c[3].get<double>()};
    peak_ = s.value("peak", 0.0); place_peak_ = s.value("place_peak", 0.0);
    episodes_ = s.value("episodes", uint64_t{0}); abs_err_sum_ = s.value("abs_err_sum", 0.0);
    in_ep_ = false;
}

nlohmann::json LookLoop::diag_snapshot() const {
    nlohmann::json cells = nlohmann::json::object();
    for (auto const& [k, c] : cells_) cells[std::to_string(k)] = {{"n", c.n}, {"mean", c.mean}};
    return nlohmann::json{{"need", need_}, {"prediction", pred_norm_}, {"in_stop", in_ep_}, {"new_fraction", nf_},
                          {"peak", peak_}, {"episodes", episodes_}, {"last_realized", last_realized_},
                          {"last_predicted", last_predicted_},
                          {"mean_abs_error", episodes_ ? abs_err_sum_ / double(episodes_) : 0.0}, {"cells", cells}};
}

} // namespace ogma
