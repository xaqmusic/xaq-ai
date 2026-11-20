// =============================================================================
// AheadField.cpp  --  see the header (learned stepper plan 9.7 item 1, 2026-11-20)
// =============================================================================
#include "ogma/modules/AheadField.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <Eigen/Dense>
#include "ogma/modules/TerrainMemory.hpp"

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
    if (auto p = std::get_if<bool>(&v))    return *p ? 1.0 : 0.0;
    throw std::invalid_argument("AheadField: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("AheadField: param '" + k + "' must be a string");
}
} // namespace

AheadField::AheadField()  = default;
AheadField::~AheadField() = default;

std::string_view AheadField::type_name() const { return "AheadField"; }

std::vector<TopicSpec> AheadField::input_topics() const {
    const auto pt = std::type_index(typeid(ProprioToken));
    return { TopicSpec{field_topic_, pt, SubscriptionKind::Direct, false}, TopicSpec{up_topic_, pt, SubscriptionKind::Direct, false},
             TopicSpec{toe_y_topic_, pt, SubscriptionKind::Direct, false}, TopicSpec{contact_topic_, pt, SubscriptionKind::Direct, false} };
}

std::vector<TopicSpec> AheadField::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema AheadField::params_schema() const {
    return {
        {"field_topic",   ParamMutability::ConstructionOnly, "ProprioToken [64]: the forward 8x8 field (m; <= 0 = no return).", ParamValue{std::string("sense.tof8")}},
        {"up_topic",      ParamMutability::ConstructionOnly, "ProprioToken [x, y, z]: the body's fused up vector, for levelling.", ParamValue{std::string("reality.proprio.up_body")}},
        {"toe_y_topic",   ParamMutability::ConstructionOnly, "ProprioToken [4]: each toe tip's height along gravity, body origin 0.", ParamValue{std::string("reality.proprio.toe_y")}},
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4]: per-leg contact.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "The ground-ahead grid, TerrainMemory's ahead format, heights against the front toes' ground.", ParamValue{std::string("terrain.ahead")}},
        {"pitch_deg",     ParamMutability::ConstructionOnly, "The field's pitch below the body's forward axis (deg).", ParamValue{30.0}},
        {"sensor_x_m",    ParamMutability::ConstructionOnly, "The sensor's place in the body frame, x (m).", ParamValue{0.0}},
        {"sensor_y_m",    ParamMutability::ConstructionOnly, "... y, up (m).", ParamValue{0.049}},
        {"sensor_z_m",    ParamMutability::ConstructionOnly, "... z, forward (m).", ParamValue{0.049}},
        {"max_range_m",   ParamMutability::HotMutable, "Returns beyond this are not used (m).", ParamValue{2.0}},
        {"rows",          ParamMutability::ConstructionOnly, "The grid's rows (forward).", ParamValue{int64_t{9}}},
        {"cols",          ParamMutability::ConstructionOnly, "The grid's columns (lateral, centred on the body's axis).", ParamValue{int64_t{3}}},
        {"row_m",         ParamMutability::ConstructionOnly, "A row's depth (m).", ParamValue{0.04}},
        {"col_m",         ParamMutability::ConstructionOnly, "A column's width (m).", ParamValue{0.08}},
        {"start_m",       ParamMutability::ConstructionOnly, "Row 0's centre, forward of the body's origin (m): the front toes' line.", ParamValue{0.10}},
        {"front_legs",    ParamMutability::ConstructionOnly, "The front legs' indices in toe_y and foot_contact, as 10 a + b (1 = legs 0 and 1).", ParamValue{int64_t{1}}},
        {"tau_s",         ParamMutability::HotMutable, "A cell's averaging time (s): a stride of the rhythm, so the rock is out.", ParamValue{0.6}},
        {"astride_tau_s", ParamMutability::HotMutable, "The averaging time of the step the body stands across (the front toes over the rear toes) (s).", ParamValue{1.0}},
        {"hold_s",        ParamMutability::HotMutable, "A cell no return has reached for this long is unknown again (s).", ParamValue{1.5}},
        {"hz",            ParamMutability::ConstructionOnly, "The tick rate (Hz).", ParamValue{50.0}},
        {"up_min",        ParamMutability::HotMutable, "The up vector's y below which the body is not upright and frames are dropped.", ParamValue{0.8}},
    };
}

ParamMap AheadField::current_params() const {
    return { {"field_topic", ParamValue{field_topic_}}, {"up_topic", ParamValue{up_topic_}}, {"toe_y_topic", ParamValue{toe_y_topic_}}, {"contact_topic", ParamValue{contact_topic_}},
             {"output_topic", ParamValue{output_topic_}}, {"pitch_deg", ParamValue{pitch_deg_}}, {"sensor_x_m", ParamValue{sensor_x_m_}}, {"sensor_y_m", ParamValue{sensor_y_m_}},
             {"sensor_z_m", ParamValue{sensor_z_m_}}, {"max_range_m", ParamValue{max_range_m_}}, {"rows", ParamValue{int64_t(rows_)}}, {"cols", ParamValue{int64_t(cols_)}},
             {"row_m", ParamValue{row_m_}}, {"col_m", ParamValue{col_m_}}, {"start_m", ParamValue{start_m_}}, {"front_legs", ParamValue{int64_t(10 * front_a_ + front_b_)}},
             {"tau_s", ParamValue{tau_s_}}, {"astride_tau_s", ParamValue{astride_tau_s_}}, {"hold_s", ParamValue{hold_s_}}, {"hz", ParamValue{hz_}}, {"up_min", ParamValue{up_min_}} };
}

void AheadField::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "max_range_m") max_range_m_ = std::max(0.05, get_double(value, k));
    else if (k == "tau_s")       tau_s_       = std::max(0.0, get_double(value, k));
    else if (k == "hold_s")      hold_s_      = std::max(0.0, get_double(value, k));
    else if (k == "astride_tau_s") astride_tau_s_ = std::max(0.0, get_double(value, k));
    else if (k == "up_min")      up_min_      = std::clamp(get_double(value, k), -1.0, 1.0);
}

void AheadField::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("AheadField requires a non-null Bus");
    for (auto pr : std::initializer_list<std::pair<const char*, std::string*>>{
            {"field_topic", &field_topic_}, {"up_topic", &up_topic_}, {"toe_y_topic", &toe_y_topic_}, {"contact_topic", &contact_topic_}, {"output_topic", &output_topic_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    for (auto pr : std::initializer_list<std::pair<const char*, double*>>{
            {"pitch_deg", &pitch_deg_}, {"sensor_x_m", &sensor_x_m_}, {"sensor_y_m", &sensor_y_m_}, {"sensor_z_m", &sensor_z_m_}, {"row_m", &row_m_}, {"col_m", &col_m_}, {"start_m", &start_m_}, {"hz", &hz_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_double(v, pr.first); });
    apply_param(params, "rows", [&](auto const& v){ rows_ = std::clamp(int(get_double(v, "rows")), 2, 32); });
    apply_param(params, "cols", [&](auto const& v){ cols_ = std::clamp(int(get_double(v, "cols")), 1, 16); });
    apply_param(params, "front_legs", [&](auto const& v){ const int x = int(get_double(v, "front_legs")); front_a_ = std::clamp(x / 10, 0, 3); front_b_ = std::clamp(x % 10, 0, 3); });
    for (auto const& k : {"max_range_m", "tau_s", "hold_s", "up_min", "astride_tau_s"}) apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    if (field_topic_.empty() || output_topic_.empty() || toe_y_topic_.empty() || contact_topic_.empty()) throw std::invalid_argument("AheadField: field, toe_y, contact and output topics are required");
    row_m_ = std::clamp(row_m_, 0.005, 0.5); col_m_ = std::clamp(col_m_, 0.005, 0.5); hz_ = std::max(1.0, hz_);
    est_.assign(size_t(rows_ * cols_), 0.0); cnt_.assign(size_t(rows_ * cols_), 0.0); seen_tick_.assign(size_t(rows_ * cols_), -1);
    auto sub = [this](std::string const& topic, int n, float* dst, bool* have) {
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct, [this, n, dst, have](std::string_view, MessagePtr p){
            if (!input_allowed(p->producer_id)) return;
            auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
            if (!pt || pt->values.size() < n) return;
            for (int i = 0; i < n; ++i) dst[i] = pt->values[i];
            if (have) *have = true;
        }));
    };
    sub(field_topic_, body::tof8::kZones, field_.data(), &fresh_);
    sub(toe_y_topic_, 4, toe_y_.data(), &have_toe_);
    sub(contact_topic_, 4, contact_.data(), &have_contact_);
    if (!up_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(up_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){
            if (!input_allowed(p->producer_id)) return;
            auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
            if (!pt || pt->values.size() < 3) return;
            for (int i = 0; i < 3; ++i) up_[size_t(i)] = double(pt->values[i]);
            have_up_ = true;
        }));
}

double AheadField::cell(int row, int col, bool& known) const {
    const size_t k = size_t(row * cols_ + col);
    known = cnt_[k] > 0.0;
    return known ? est_[k] : 0.0;
}

void AheadField::tick(uint64_t tick_id) {
    using namespace body::tof8;
    const int N = rows_ * cols_;
    const int64_t hold_ticks = int64_t(std::lround(hold_s_ * hz_));
    for (int k = 0; k < N; ++k)   // a cell no return has reached for hold_s is unknown again
        if (cnt_[size_t(k)] > 0.0 && int64_t(tick_id) - seen_tick_[size_t(k)] > hold_ticks) cnt_[size_t(k)] = 0.0;
    // the step the body stands across: the planted front toes over the planted rear toes, every tick both ends are on the ground
    if (have_toe_ && have_contact_ && (!have_up_ || up_[1] >= up_min_)) {
        double fs = 0.0, bs = 0.0; int fnn = 0, bnn = 0;
        for (int l = 0; l < 4; ++l) {
            if (contact_[size_t(l)] <= 0.5f) continue;
            if (l == front_a_ || l == front_b_) { fs += double(toe_y_[size_t(l)]); ++fnn; } else { bs += double(toe_y_[size_t(l)]); ++bnn; }
        }
        if (fnn > 0 && bnn > 0) {
            ++astride_n_;
            const double a = std::max(1.0 / double(astride_n_), astride_tau_s_ > 0.0 ? std::min(1.0, 1.0 / (astride_tau_s_ * hz_)) : 1.0);
            astride_ += a * (fs / fnn - bs / bnn - astride_);
        }
    }
    if (fresh_) {
        fresh_ = false;
        const bool upright = !have_up_ || up_[1] >= up_min_;
        const bool pa = have_contact_ && contact_[size_t(front_a_)] > 0.5f, pb = have_contact_ && contact_[size_t(front_b_)] > 0.5f;
        if (!upright) ++tilted_;
        else if (!have_toe_ || !(pa || pb)) ++no_ref_;   // no front toe on the ground this tick: nothing to measure against
        else {
            ++frames_;
            ref_ = pa && pb ? 0.5 * (double(toe_y_[size_t(front_a_)]) + double(toe_y_[size_t(front_b_)])) : double(toe_y_[size_t(pa ? front_a_ : front_b_)]);
            // the horizontal axes of the levelled frame: forward is the body's z less its part along gravity
            const double ux = up_[0], uy = up_[1], uz = up_[2];
            double fx = -uz * ux, fy = -uz * uy, fz = 1.0 - uz * uz; const double fn = std::sqrt(fx * fx + fy * fy + fz * fz);
            if (fn > 1e-6) {
                fx /= fn; fy /= fn; fz /= fn;
                const double lx = uy * fz - uz * fy, ly = uz * fx - ux * fz, lz = ux * fy - uy * fx;   // up x forward
                std::array<Vec3, kZones> P{}; std::array<bool, kZones> ok{};
                points(field_.data(), pitch_deg_, max_range_m_, P, ok);
                std::vector<double> sum(size_t(N), 0.0); std::vector<int> n(size_t(N), 0);
                for (int z = 0; z < kZones; ++z) {
                    if (!ok[size_t(z)]) continue;
                    const double bx = P[size_t(z)].x + sensor_x_m_, by = P[size_t(z)].y + sensor_y_m_, bz = P[size_t(z)].z + sensor_z_m_;
                    const double fwd = bx * fx + by * fy + bz * fz, lat = bx * lx + by * ly + bz * lz, h = bx * ux + by * uy + bz * uz;
                    const int i = int(std::floor((fwd - (start_m_ - 0.5 * row_m_)) / row_m_)), j = int(std::floor((lat + 0.5 * cols_ * col_m_) / col_m_));
                    if (i < 0 || i >= rows_ || j < 0 || j >= cols_) continue;
                    sum[size_t(i * cols_ + j)] += h - ref_; ++n[size_t(i * cols_ + j)];
                }
                // the averaging: a frame's weight is the time since the last frame over tau, never less than 1 / (the cell's frames)
                const double dt = have_frame_ ? double(tick_id - last_frame_tick_) / hz_ : 1.0 / hz_;
                const double a_min = tau_s_ > 0.0 ? std::min(1.0, dt / tau_s_) : 1.0;
                for (int k = 0; k < N; ++k) {
                    if (n[size_t(k)] == 0) continue;
                    cnt_[size_t(k)] += 1.0;
                    const double a = std::max(1.0 / cnt_[size_t(k)], a_min);
                    est_[size_t(k)] += a * (sum[size_t(k)] / double(n[size_t(k)]) - est_[size_t(k)]);
                    seen_tick_[size_t(k)] = int64_t(tick_id);
                }
                last_frame_tick_ = tick_id; have_frame_ = true;
            }
        }
    }
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("ahead_field") : std::string(id()); out->sensor = "terrain_ahead";
    const int H = TerrainMemory::kAheadHeader;
    out->values = Eigen::VectorXf::Constant(H + 2 * N, TerrainMemory::kNoHeight);
    out->values[0] = float(rows_); out->values[1] = float(cols_); out->values[2] = float(row_m_); out->values[3] = float(col_m_); out->values[4] = float(start_m_); out->values[5] = float(astride_);
    const double cap = tau_s_ > 0.0 ? std::max(1.0, tau_s_ * hz_) : 1.0;
    for (int k = 0; k < N; ++k) {
        if (cnt_[size_t(k)] > 0.0) { out->values[H + k] = float(est_[size_t(k)]); out->values[H + N + k] = float(std::min(cnt_[size_t(k)], cap)); }
        else out->values[H + N + k] = 0.0f;
    }
    bus_->publish(output_topic_, out);
}

nlohmann::json AheadField::diag_snapshot() const {
    int known = 0; for (double c : cnt_) known += c > 0.0 ? 1 : 0;
    return nlohmann::json{{"frames", frames_}, {"no_ref", no_ref_}, {"tilted", tilted_}, {"ref", ref_}, {"known", known}, {"astride", astride_}};
}

} // namespace ogma
