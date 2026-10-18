// =============================================================================
// StuckSense.cpp  --  see the header
// =============================================================================
#include "ogma/modules/StuckSense.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <vector>

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
    throw std::invalid_argument("StuckSense: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("StuckSense: param '" + k + "' must be a string");
}
} // namespace

StuckSense::StuckSense()  = default;
StuckSense::~StuckSense() = default;

std::string_view StuckSense::type_name() const { return "StuckSense"; }

std::vector<TopicSpec> StuckSense::input_topics() const {
    std::vector<TopicSpec> v = {
        TopicSpec{range_topic_,  std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{stride_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
    if (!speed_topic_.empty()) v.push_back(TopicSpec{speed_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> StuckSense::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema StuckSense::params_schema() const {
    return {
        {"range_topic",  ParamMutability::ConstructionOnly, "ProprioToken [distance_m, valid]: the forward range beam.", ParamValue{std::string("sense.range")}},
        {"stride_topic", ParamMutability::ConstructionOnly, "ProprioToken [v_x, v_fwd]: the stride estimate (the efference's own report of walking).", ParamValue{std::string("reality.proprio.stride_v")}},
        {"speed_topic",  ParamMutability::ConstructionOnly, "ProprioToken [s]: the commanded speed; the sense judges only while s >= cmd_min. Empty = assume forward.", ParamValue{std::string("reality.proprio.speed_cmd")}},
        {"output_topic", ParamMutability::ConstructionOnly, "Output: [score, deficit, expected_m, observed_m].", ParamValue{std::string("reality.proprio.stuck")}},
        {"odom_topic",   ParamMutability::ConstructionOnly, "Optional ProprioToken [x, y, yaw]: the body's heading. The sense abstains while the body IS turning (|yaw rate| > yaw_rate_max): a turn zeroes forward stride legitimately. Empty = no gate.", ParamValue{std::string("")}},
        {"yaw_rate_max", ParamMutability::HotMutable, "The NET yaw rate over yaw_window_ticks above which the sense abstains (rad/s). The gait's own per-stride rocking averages out over the window; a turn does not.", ParamValue{0.15}},
        {"yaw_window_ticks", ParamMutability::HotMutable, "The window the net yaw is measured over (ticks); whole strides (the gait period is ~70).", ParamValue{int64_t{150}}},
        {"window_ticks",   ParamMutability::HotMutable, "The window the range must have closed over (ticks).", ParamValue{int64_t{50}}},
        {"min_expected_m", ParamMutability::HotMutable, "Below this much claimed walking the window says too little to judge.", ParamValue{0.05}},
        {"cmd_min",        ParamMutability::HotMutable, "The body must command at least this forward speed for the sense to judge.", ParamValue{0.3}},
        {"range_max_m",    ParamMutability::HotMutable, "A range beyond this is the beam's far end, not a thing: not judged.", ParamValue{1.4}},
        {"ema_alpha",      ParamMutability::HotMutable, "The score's EMA on measured ticks.", ParamValue{0.05}},
        {"decay",          ParamMutability::HotMutable, "Per-tick decay toward 0 while the sense abstains (no echo / not commanding).", ParamValue{0.01}},
        {"hz",             ParamMutability::ConstructionOnly, "Tick rate.", ParamValue{50.0}},
        {"v_ema_alpha",    ParamMutability::HotMutable, "The stride forward's EMA (the stall channel's sensed speed). 0.01 = ~2 s: long enough to average the gait's own +/-0.3 m/s stride oscillation.", ParamValue{0.01}},
        {"v_ref_tau_s",    ParamMutability::HotMutable, "The decay of the peak-hold of v_ema: the body's own recent best forward speed (s).", ParamValue{20.0}},
        {"v_floor",        ParamMutability::HotMutable, "v_ref never falls below this (m/s): a body that never walked has no stall.", ParamValue{0.01}},
        {"stall_frac",     ParamMutability::HotMutable, "v_ema below this fraction of v_ref, while commanding, is a full stall.", ParamValue{0.25}},
        {"expect_from_cmd", ParamMutability::HotMutable, "The range channel's expectation from the COMMANDED speed: expected = sum(speed_cmd * this / hz) over the window (m/s at full command; the body walks ~0.03). The channel then speaks whenever the brain commands forward with a surface ahead, not only when the stride estimate (noise on carpet) claims walking. 0 = legacy.", ParamValue{0.0}},
        {"end_median_ticks", ParamMutability::HotMutable, "The range at each end of the window = the MEDIAN of the valid samples in the window's first / last this many ticks (an end needs a quarter of them valid). The sonar jitters 0.04-0.17 m over a second against the 0.03 m a walking body closes, so single end samples make the deficit a coin flip; a 10 s window with 2 s median ends resolves it. 0 = legacy single samples.", ParamValue{int64_t{0}}},
    };
}

ParamMap StuckSense::current_params() const {
    ParamMap m;
    m["range_topic"] = ParamValue{range_topic_}; m["stride_topic"] = ParamValue{stride_topic_};
    m["speed_topic"] = ParamValue{speed_topic_}; m["output_topic"] = ParamValue{output_topic_};
    m["odom_topic"] = ParamValue{odom_topic_}; m["yaw_rate_max"] = ParamValue{yaw_rate_max_}; m["yaw_window_ticks"] = ParamValue{int64_t(yaw_window_ticks_)};
    m["window_ticks"] = ParamValue{int64_t(window_ticks_)}; m["min_expected_m"] = ParamValue{min_expected_m_};
    m["cmd_min"] = ParamValue{cmd_min_}; m["range_max_m"] = ParamValue{range_max_m_};
    m["ema_alpha"] = ParamValue{ema_alpha_}; m["decay"] = ParamValue{decay_}; m["hz"] = ParamValue{hz_};
    m["v_ema_alpha"] = ParamValue{v_ema_alpha_}; m["v_ref_tau_s"] = ParamValue{v_ref_tau_s_};
    m["v_floor"] = ParamValue{v_floor_}; m["stall_frac"] = ParamValue{stall_frac_}; m["expect_from_cmd"] = ParamValue{expect_from_cmd_};
    m["end_median_ticks"] = ParamValue{int64_t(end_median_ticks_)};
    return m;
}

void StuckSense::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("StuckSense requires a non-null Bus");
    apply_param(params, "range_topic",  [&](auto const& v){ range_topic_  = get_string(v, "range_topic"); });
    apply_param(params, "stride_topic", [&](auto const& v){ stride_topic_ = get_string(v, "stride_topic"); });
    apply_param(params, "speed_topic",  [&](auto const& v){ speed_topic_  = get_string(v, "speed_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "odom_topic",   [&](auto const& v){ odom_topic_   = get_string(v, "odom_topic"); });
    apply_param(params, "yaw_rate_max", [&](auto const& v){ yaw_rate_max_ = get_double(v, "yaw_rate_max"); });
    apply_param(params, "yaw_window_ticks", [&](auto const& v){ yaw_window_ticks_ = std::max(2, int(get_double(v, "yaw_window_ticks"))); });
    apply_param(params, "window_ticks",   [&](auto const& v){ window_ticks_   = std::max(2, int(get_double(v, "window_ticks"))); });
    apply_param(params, "min_expected_m", [&](auto const& v){ min_expected_m_ = get_double(v, "min_expected_m"); });
    apply_param(params, "cmd_min",        [&](auto const& v){ cmd_min_        = get_double(v, "cmd_min"); });
    apply_param(params, "range_max_m",    [&](auto const& v){ range_max_m_    = get_double(v, "range_max_m"); });
    apply_param(params, "ema_alpha",      [&](auto const& v){ ema_alpha_      = get_double(v, "ema_alpha"); });
    apply_param(params, "decay",          [&](auto const& v){ decay_          = get_double(v, "decay"); });
    apply_param(params, "hz",             [&](auto const& v){ hz_             = get_double(v, "hz"); });
    apply_param(params, "v_ema_alpha",    [&](auto const& v){ v_ema_alpha_    = get_double(v, "v_ema_alpha"); });
    apply_param(params, "v_ref_tau_s",    [&](auto const& v){ v_ref_tau_s_    = get_double(v, "v_ref_tau_s"); });
    apply_param(params, "v_floor",        [&](auto const& v){ v_floor_        = get_double(v, "v_floor"); });
    apply_param(params, "stall_frac",     [&](auto const& v){ stall_frac_     = get_double(v, "stall_frac"); });
    apply_param(params, "expect_from_cmd", [&](auto const& v){ expect_from_cmd_ = get_double(v, "expect_from_cmd"); });
    apply_param(params, "end_median_ticks", [&](auto const& v){ end_median_ticks_ = std::max(0, int(get_double(v, "end_median_ticks"))); });
    sub_ids_.push_back(bus_->subscribe(range_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_range(p); }));
    sub_ids_.push_back(bus_->subscribe(stride_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_stride(p); }));
    if (!speed_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(speed_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_speed(p); }));
    if (!odom_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(odom_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_odom(p); }));
}

bool StuckSense::end_range(size_t begin, size_t end, double& r_out) const {
    std::vector<double> v;
    for (size_t i = begin; i < end && i < ring_.size(); ++i)
        if (ring_[i].valid && ring_[i].r <= range_max_m_) v.push_back(ring_[i].r);
    const size_t need = std::max<size_t>(1, (end - begin) / 4);
    if (v.size() < need) return false;
    std::nth_element(v.begin(), v.begin() + long(v.size() / 2), v.end());
    r_out = v[v.size() / 2];
    return true;
}

void StuckSense::handle_odom(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 3) return;
    yaw_ = double(pt->values[2]); have_odom_ = true;
}

void StuckSense::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "window_ticks")   window_ticks_   = std::max(2, int(get_double(value, k)));
    else if (k == "min_expected_m") min_expected_m_ = get_double(value, k);
    else if (k == "cmd_min")        cmd_min_        = get_double(value, k);
    else if (k == "range_max_m")    range_max_m_    = get_double(value, k);
    else if (k == "ema_alpha")      ema_alpha_      = get_double(value, k);
    else if (k == "decay")          decay_          = get_double(value, k);
    else if (k == "v_ema_alpha")    v_ema_alpha_    = get_double(value, k);
    else if (k == "v_ref_tau_s")    v_ref_tau_s_    = get_double(value, k);
    else if (k == "v_floor")        v_floor_        = get_double(value, k);
    else if (k == "stall_frac")     stall_frac_     = get_double(value, k);
    else if (k == "expect_from_cmd") expect_from_cmd_ = get_double(value, k);
    else if (k == "end_median_ticks") end_median_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "yaw_rate_max")   yaw_rate_max_   = get_double(value, k);
    else if (k == "yaw_window_ticks") yaw_window_ticks_ = std::max(2, int(get_double(value, k)));
    else throw std::invalid_argument("StuckSense: param '" + k + "' is construction-only / unknown");
}

void StuckSense::handle_range(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 2) return;
    d_ = double(pt->values[0]); valid_ = pt->values[1] > 0.5f;
}
void StuckSense::handle_stride(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 2) return;
    vfwd_ = double(pt->values[1]);
}
void StuckSense::handle_speed(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 1) return;
    speed_cmd_ = double(pt->values[0]); have_speed_ = true;
}

void StuckSense::tick(uint64_t tick_id) {
    ring_.push_back(Sample{d_, valid_, vfwd_, (have_speed_ || speed_topic_.empty()) ? std::max(0.0, speed_cmd_) : 1.0});
    while (int(ring_.size()) > window_ticks_ + 1) ring_.pop_front();
    measured_ = false;
    if (have_odom_) {
        yaw_ring_.push_back(yaw_);
        while (int(yaw_ring_.size()) > yaw_window_ticks_ + 1) yaw_ring_.pop_front();
        yaw_rate_ = (int(yaw_ring_.size()) == yaw_window_ticks_ + 1)
                  ? (yaw_ring_.back() - yaw_ring_.front()) * hz_ / double(yaw_window_ticks_) : 0.0;
    }
    const bool straight   = odom_topic_.empty() || !have_odom_ || std::fabs(yaw_rate_) <= yaw_rate_max_;
    const bool commanding = (speed_topic_.empty() || !have_speed_ || speed_cmd_ >= cmd_min_) && straight;
    // Channel 1: the stall.  The stride forward's EMA against the body's own recent best.
    v_ema_ += v_ema_alpha_ * (vfwd_ - v_ema_);
    v_ref_  = std::max(v_floor_, std::max(v_ema_, v_ref_ * (1.0 - 1.0 / std::max(1.0, v_ref_tau_s_ * hz_))));
    // A body that has never walked has no stall: the channel speaks only once v_ref has risen
    // above its floor (so a standing start, servos ramping, is not an escape).
    stall_  = (commanding && v_ref_ > v_floor_)
            ? float(std::clamp(1.0 - v_ema_ / (stall_frac_ * v_ref_), 0.0, 1.0)) : 0.0f;
    // Channel 2: the range deficit (the slip case).
    if (int(ring_.size()) == window_ticks_ + 1 && commanding) {
        const Sample& a = ring_.front(); const Sample& b = ring_.back();
        double r_a = a.r, r_b = b.r;
        const bool ends_ok = end_median_ticks_ > 0
            ? (end_range(0, size_t(std::min(end_median_ticks_, int(ring_.size()))), r_a)
               && end_range(ring_.size() - size_t(std::min(end_median_ticks_, int(ring_.size()))), ring_.size(), r_b))
            : (a.valid && b.valid && a.r <= range_max_m_ && b.r <= range_max_m_);
        if (ends_ok) {
            double exp = 0.0;
            if (expect_from_cmd_ > 0.0) for (size_t i = 1; i < ring_.size(); ++i) exp += ring_[i].cmd * expect_from_cmd_ / hz_;
            else                        for (size_t i = 1; i < ring_.size(); ++i) exp += ring_[i].vfwd / hz_;
            if (exp >= min_expected_m_) {
                expected_ = exp; observed_ = r_a - r_b;
                deficit_ = float(std::clamp(1.0 - observed_ / expected_, 0.0, 1.0));
                range_score_ += float(ema_alpha_) * (deficit_ - range_score_);
                measured_ = true; ++measurements_;
            }
        }
    }
    if (!measured_) range_score_ = std::max(0.0f, range_score_ - float(decay_) * range_score_);
    score_ = std::max(stall_, range_score_);
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("stuck") : id_; out->sensor = "stuck";
    out->values = Eigen::VectorXf(5);
    out->values[0] = score_; out->values[1] = measured_ ? deficit_ : 0.0f;
    out->values[2] = float(expected_); out->values[3] = float(observed_);
    out->values[4] = range_score_;    // the range channel alone, smoothed: what a push gate should read
    bus_->publish(output_topic_, out);
}

nlohmann::json StuckSense::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"score", score_}, {"measurements", measurements_}};
}
void StuckSense::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    score_ = s.value("score", 0.0f); measurements_ = s.value("measurements", int64_t{0});
}
nlohmann::json StuckSense::diag_snapshot() const {
    return nlohmann::json{{"score", score_}, {"stall", stall_}, {"range_score", range_score_},
                          {"v_ema", v_ema_}, {"v_ref", v_ref_}, {"deficit", deficit_}, {"measured", measured_},
                          {"expected", expected_}, {"observed", observed_}, {"measurements", measurements_},
                          {"range", d_}, {"valid", valid_}, {"vfwd", vfwd_}, {"speed_cmd", speed_cmd_},
                          {"yaw_rate", yaw_rate_}};
}

} // namespace ogma
