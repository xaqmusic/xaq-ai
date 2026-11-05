#include "ogma/modules/Tof8Range.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

namespace ogma {
namespace {
template <typename Fn> void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key); if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("Tof8Range: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("Tof8Range: param '" + k + "' must be a string");
}
constexpr double kPi = 3.14159265358979323846;
} // namespace

Tof8Range::Tof8Range()  = default;
Tof8Range::~Tof8Range() = default;
std::string_view Tof8Range::type_name() const { return "Tof8Range"; }

std::vector<TopicSpec> Tof8Range::input_topics() const {
    return { TopicSpec{input_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}
std::vector<TopicSpec> Tof8Range::output_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
    if (!cols_topic_.empty()) v.push_back(TopicSpec{cols_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema Tof8Range::params_schema() const {
    return {
        {"input_topic",  ParamMutability::ConstructionOnly, "ProprioToken [64]: the forward 8x8 field, slant range (m), row-major, row 0 top, column 0 left, -1 none.", ParamValue{std::string("sense.tof8")}},
        {"output_topic", ParamMutability::ConstructionOnly, "ProprioToken [distance_m, valid]: the nearest feature ahead, in the sonar's contract (StuckSense / ProxMemory read it unchanged). No feature = [max_range_m, 0].", ParamValue{std::string("sense.range_tof")}},
        {"cols_topic",   ParamMutability::ConstructionOnly, "ProprioToken [8]: the nearest feature per column, left to right (max_range_m = none). \"\" = not published.", ParamValue{std::string("reality.proprio.tof8_cols")}},
        {"pitch_deg",    ParamMutability::HotMutable, "The mount's downward pitch (deg); the beams are turned by it before the floor is fitted.", ParamValue{30.0}},
        {"max_range_m",  ParamMutability::HotMutable, "Returns beyond this are ignored; also the no-feature encoding.", ParamValue{4.0}},
        {"thing_min_m",  ParamMutability::HotMutable, "A return this far above the fitted floor is a thing; this far below it, a drop. The part's own accuracy is ~15 mm near.", ParamValue{0.02}},
        {"fit_rows",     ParamMutability::HotMutable, "The bottom rows the floor plane is fitted from (then re-fitted without outliers).", ParamValue{int64_t{3}}},
        {"fit_slope_max", ParamMutability::HotMutable, "A fitted floor steeper than this (|dy/dz| or |dy/dx|) is not a floor (the rows hit a wall): then every valid return is a feature.", ParamValue{0.55}},
        {"col_lo",       ParamMutability::HotMutable, "First column the nearest feature is taken over (0 = left).", ParamValue{int64_t{0}}},
        {"col_hi",       ParamMutability::HotMutable, "Last column (7 = right).", ParamValue{int64_t{7}}},
    };
}

ParamMap Tof8Range::current_params() const {
    return {{"input_topic", ParamValue{input_topic_}}, {"output_topic", ParamValue{output_topic_}}, {"cols_topic", ParamValue{cols_topic_}},
            {"pitch_deg", ParamValue{pitch_deg_}}, {"max_range_m", ParamValue{max_range_m_}}, {"thing_min_m", ParamValue{thing_min_m_}},
            {"fit_rows", ParamValue{int64_t(fit_rows_)}}, {"fit_slope_max", ParamValue{fit_slope_max_}},
            {"col_lo", ParamValue{int64_t(col_lo_)}}, {"col_hi", ParamValue{int64_t(col_hi_)}}};
}

void Tof8Range::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("Tof8Range requires a non-null Bus");
    apply_param(params, "input_topic",  [&](auto const& v){ input_topic_  = get_string(v, "input_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "cols_topic",   [&](auto const& v){ cols_topic_   = get_string(v, "cols_topic"); });
    for (auto const& k : {"pitch_deg", "max_range_m", "thing_min_m", "fit_rows", "fit_slope_max", "col_lo", "col_hi"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    for (auto& c : cols_) c = float(max_range_m_);
    sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle(p); }));
}

void Tof8Range::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "pitch_deg")     pitch_deg_ = get_double(value, k);
    else if (k == "max_range_m")   max_range_m_ = std::max(0.1, get_double(value, k));
    else if (k == "thing_min_m")   thing_min_m_ = std::max(0.001, get_double(value, k));
    else if (k == "fit_rows")      fit_rows_ = std::clamp(int(get_double(value, k)), 1, kRows);
    else if (k == "fit_slope_max") fit_slope_max_ = std::max(0.05, get_double(value, k));
    else if (k == "col_lo")        col_lo_ = std::clamp(int(get_double(value, k)), 0, kCols - 1);
    else if (k == "col_hi")        col_hi_ = std::clamp(int(get_double(value, k)), 0, kCols - 1);
    else throw std::invalid_argument("Tof8Range: param '" + k + "' is construction-only / unknown");
}

std::array<double, 3> Tof8Range::beam(int zone) {
    const int r = zone / kCols, c = zone % kCols;
    const double pitch = kFovDeg / kCols;
    const double az = (double(c) - 3.5) * pitch * kPi / 180.0;   // left negative (x = right)
    const double el = (3.5 - double(r)) * pitch * kPi / 180.0;   // row 0 = up
    // x = right, y = up, z = forward -- the sim's sensor frame (tof8.gd) with its x sign: the sim's +X is the body's
    // LEFT, and column 0 is the left as the robot sees it, so x = -sin(az) there; here x is "column 0 negative".
    return {std::cos(el) * std::sin(az), std::sin(el), std::cos(el) * std::cos(az)};
}
std::array<double, 3> Tof8Range::beam_pitched(int zone) const {
    const auto b = beam(zone);
    const double p = pitch_deg_ * kPi / 180.0;
    return {b[0], b[1] * std::cos(p) - b[2] * std::sin(p), b[1] * std::sin(p) + b[2] * std::cos(p)};
}

void Tof8Range::handle(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < kZones) return;
    for (int z = 0; z < kZones; ++z) field_[size_t(z)] = pt->values[z];
    have_field_ = true; field_tick_ = pt->tick_id;
}

void Tof8Range::reduce() {
    ++frames_;
    // the returns as points in the pitched sensor frame
    std::array<std::array<double, 3>, kZones> P{}; std::array<bool, kZones> ok{};
    n_valid_ = 0;
    for (int z = 0; z < kZones; ++z) {
        const double r = field_[size_t(z)];
        ok[size_t(z)] = r > 0.0 && r <= max_range_m_;
        if (!ok[size_t(z)]) continue;
        const auto d = beam_pitched(z);
        P[size_t(z)] = {r * d[0], r * d[1], r * d[2]}; ++n_valid_;
    }
    // the floor: y = a + b z + c x over the bottom fit_rows_ rows, least squares, then once more without the outliers
    auto fit = [&](std::array<bool, kZones> const& use, double& a, double& b, double& c) -> int {
        double S = 0, Sx = 0, Sz = 0, Sxx = 0, Szz = 0, Sxz = 0, Sy = 0, Sxy = 0, Szy = 0; int n = 0;
        for (int z = 0; z < kZones; ++z) {
            if (!use[size_t(z)]) continue;
            const double x = P[size_t(z)][0], y = P[size_t(z)][1], w = P[size_t(z)][2];
            S += 1; Sx += x; Sz += w; Sxx += x * x; Szz += w * w; Sxz += x * w; Sy += y; Sxy += x * y; Szy += w * y; ++n;
        }
        if (n < 4) return n;
        // normal equations for [a, b (z), c (x)]
        const double M[3][3] = {{S, Sz, Sx}, {Sz, Szz, Sxz}, {Sx, Sxz, Sxx}};
        const double R[3] = {Sy, Szy, Sxy};
        const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
        if (std::fabs(det) < 1e-12) return 0;
        auto solve = [&](int col) {
            double N[3][3]; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) N[i][j] = (j == col) ? R[i] : M[i][j];
            return (N[0][0] * (N[1][1] * N[2][2] - N[1][2] * N[2][1]) - N[0][1] * (N[1][0] * N[2][2] - N[1][2] * N[2][0]) + N[0][2] * (N[1][0] * N[2][1] - N[1][1] * N[2][0])) / det;
        };
        a = solve(0); b = solve(1); c = solve(2);
        return n;
    };
    std::array<bool, kZones> use{};
    for (int z = 0; z < kZones; ++z) use[size_t(z)] = ok[size_t(z)] && (z / kCols) >= kRows - fit_rows_;
    double a = 0, b = 0, c = 0; int n = fit(use, a, b, c);
    if (n >= 4) {   // drop the outliers (a thing standing in the bottom rows) and fit again
        for (int z = 0; z < kZones; ++z)
            if (use[size_t(z)] && std::fabs(P[size_t(z)][1] - (a + b * P[size_t(z)][2] + c * P[size_t(z)][0])) > thing_min_m_) use[size_t(z)] = false;
        double a2 = a, b2 = b, c2 = c;
        if (fit(use, a2, b2, c2) >= 4) { a = a2; b = b2; c = c2; }
    }
    floor_ok_ = n >= 4 && std::fabs(b) <= fit_slope_max_ && std::fabs(c) <= fit_slope_max_ && a < 0.0;
    plane_a_ = a; plane_b_ = b; plane_c_ = c;
    // the features
    n_floor_ = n_up_ = n_down_ = 0; nearest_ = max_range_m_; nearest_valid_ = false; nearest_zone_ = -1;
    for (auto& cc : cols_) cc = float(max_range_m_);
    for (int z = 0; z < kZones; ++z) {
        if (!ok[size_t(z)]) continue;
        bool feature; 
        if (floor_ok_) {
            const double h = P[size_t(z)][1] - (a + b * P[size_t(z)][2] + c * P[size_t(z)][0]);
            if (h > thing_min_m_) { ++n_up_; feature = true; }
            else if (h < -thing_min_m_) { ++n_down_; feature = true; }
            else { ++n_floor_; feature = false; }
        } else feature = true;   // no floor in the bottom rows: whatever returns is a thing
        if (!feature) continue;
        const int col = z % kCols; const double r = field_[size_t(z)];
        cols_[size_t(col)] = std::min(cols_[size_t(col)], float(r));
        if (col >= col_lo_ && col <= col_hi_ && r < nearest_) { nearest_ = r; nearest_valid_ = true; nearest_zone_ = z; }
    }
}

void Tof8Range::tick(uint64_t tick_id) {
    if (!have_field_) return;
    if (field_tick_ != last_reduced_tick_) { reduce(); last_reduced_tick_ = field_tick_; }
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("tof8_range") : std::string(id()); out->sensor = "range_tof";
    out->values = Eigen::VectorXf(2); out->values[0] = float(nearest_); out->values[1] = nearest_valid_ ? 1.0f : 0.0f;
    bus_->publish(output_topic_, out);
    if (!cols_topic_.empty()) {
        auto cv = std::make_shared<ProprioToken>();
        cv->tick_id = tick_id; cv->producer_id = out->producer_id; cv->sensor = "tof8_cols";
        cv->values = Eigen::VectorXf(kCols); for (int c = 0; c < kCols; ++c) cv->values[c] = cols_[size_t(c)];
        bus_->publish(cols_topic_, cv);
    }
}

nlohmann::json Tof8Range::diag_snapshot() const {
    return nlohmann::json{{"frames", frames_}, {"nearest_m", nearest_}, {"valid", nearest_valid_}, {"nearest_zone", nearest_zone_},
                          {"n_valid", n_valid_}, {"n_floor", n_floor_}, {"n_thing_up", n_up_}, {"n_thing_down", n_down_}, {"floor_ok", floor_ok_},
                          {"plane", {plane_a_, plane_b_, plane_c_}}, {"cols", std::vector<float>(cols_.begin(), cols_.end())}};
}

} // namespace ogma
