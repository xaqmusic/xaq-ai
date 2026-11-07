#include "ogma/modules/TerrainMemory.hpp"
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
    throw std::invalid_argument("TerrainMemory: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("TerrainMemory: param '" + k + "' must be a string");
}
constexpr int kFloor = 1, kThing = 2, kDrop = 3;
} // namespace

TerrainMemory::TerrainMemory()  = default;
TerrainMemory::~TerrainMemory() = default;
std::string_view TerrainMemory::type_name() const { return "TerrainMemory"; }

std::vector<TopicSpec> TerrainMemory::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : {field_topic_, contact_topic_, feet_xz_topic_, feet_y_topic_, odom_topic_})
        if (!t.empty()) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> TerrainMemory::output_topics() const {
    return { TopicSpec{predict_topic_, std::type_index(typeid(ProprioToken))}, TopicSpec{error_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema TerrainMemory::params_schema() const {
    return {
        {"field_topic",   ParamMutability::ConstructionOnly, "ProprioToken [64]: the forward 8x8 field (sense.tof8). \"\" = touches only.", ParamValue{std::string("sense.tof8")}},
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4] per-leg contact; a touch is the rising edge.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"feet_xz_topic", ParamMutability::ConstructionOnly, "ProprioToken [8] per leg [x = body +X, z = forward] of the toe.", ParamValue{std::string("reality.proprio.feet_xz")}},
        {"feet_y_topic",  ParamMutability::ConstructionOnly, "ProprioToken [4] the toe TIP's height along gravity, body origin 0 (reality.proprio.toe_y).", ParamValue{std::string("reality.proprio.toe_y")}},
        {"odom_topic",    ParamMutability::ConstructionOnly, "ProprioToken [x, y, yaw]: the body's dead-reckoned pose; the voxels live in its frame.", ParamValue{std::string("reality.proprio.odom")}},
        {"up_topic",      ParamMutability::ConstructionOnly, "ProprioToken [3]: the body's gravity-up in the body frame (reality.proprio.up_body); levels the field's points. \"\" or absent = the body frame as is.", ParamValue{std::string("reality.proprio.up_body")}},
        {"stall_topic",   ParamMutability::ConstructionOnly, "Optional ProprioToken whose value[0] is the stall score (reality.proprio.stuck): the memory's frame advances by the odometry's translation x (1 - stall_gate x stall), so a held body's memory does not slide under it while its legs claim stride. \"\" = the odometry as is.", ParamValue{std::string("")}},
        {"stall_gate",    ParamMutability::HotMutable, "How fully the stall freezes the frame (1 = a full stall stops it).", ParamValue{1.0}},
        {"register_gain", ParamMutability::HotMutable, "Registration by re-observation: the fraction of the forward shift that best matches a field frame to the remembered heights that moves the memory's frame, each frame (0 = off; 0.3-0.5 converges in a few frames without chasing noise).", ParamValue{0.0}},
        {"register_range_m", ParamMutability::HotMutable, "The forward shifts searched, +/- (m), in half-voxel steps.", ParamValue{0.15}},
        {"register_min_points", ParamMutability::HotMutable, "Returns of the frame that must land on remembered ground for a match to count.", ParamValue{int64_t{16}}},
        {"register_min_spread_m", ParamMutability::HotMutable, "The spread (sd) of the frame's heights needed: a flat floor registers nothing (and drifts harmlessly).", ParamValue{0.015}},
        {"datum_edge_max_m", ParamMutability::HotMutable, "A touch on known ground moves the datum only where the remembered heights around it (the 3x3 cells) spread less than this: a cell on a riser's edge is touched at two levels and must not move the body. 0 = no gate.", ParamValue{0.02}},
        {"load_topic",    ParamMutability::ConstructionOnly, "ProprioToken [4] foot load as a fraction of body weight: the datum moves only for a toe that comes to BEAR (a scrape on a riser face is not the body climbing). \"\" = every touch counts.", ParamValue{std::string("reality.proprio.foot_load")}},
        {"predict_topic", ParamMutability::ConstructionOnly, "ProprioToken [h_pred, precision, has, fwd, lat] x 4: what each leg's next landing will meet, every tick.", ParamValue{std::string("predict.foothold")}},
        {"error_topic",   ParamMutability::ConstructionOnly, "ProprioToken [e, valid] x 4: at a touch, h_touch - h_pred for that leg (0 otherwise).", ParamValue{std::string("predict.foothold_error")}},
        {"pitch_deg",   ParamMutability::HotMutable, "The field's mount: downward pitch (deg).", ParamValue{30.0}},
        {"sensor_x_m",  ParamMutability::HotMutable, "The field's mount in the body frame: x (+ = the body's left).", ParamValue{0.0}},
        {"sensor_y_m",  ParamMutability::HotMutable, "The field's mount in the body frame: y (the sim's 70 mm above a chassis bottom at -0.021 = 0.049).", ParamValue{0.049}},
        {"sensor_z_m",  ParamMutability::HotMutable, "The field's mount in the body frame: z forward (the front face, 0.049).", ParamValue{0.049}},
        {"max_range_m", ParamMutability::HotMutable, "Field returns beyond this are not remembered.", ParamValue{2.0}},
        {"thing_min_m", ParamMutability::HotMutable, "A return this far above the fitted floor is a thing, this far below a drop (the class only; the height is remembered either way).", ParamValue{0.02}},
        {"fit_rows",    ParamMutability::HotMutable, "The field's bottom rows the floor is fitted from.", ParamValue{int64_t{3}}},
        {"fit_slope_max", ParamMutability::HotMutable, "A fitted floor steeper than this is not a floor.", ParamValue{0.55}},
        {"voxel_m",     ParamMutability::HotMutable, "Voxel edge (m) on the ground.", ParamValue{0.03}},
        {"max_voxels",  ParamMutability::HotMutable, "The memory's size; the least recently seen voxel goes first.", ParamValue{int64_t{12000}}},
        {"max_age_ticks", ParamMutability::HotMutable, "A voxel unseen for this many ticks leaves (1500 = 30 s at 50 Hz; 0 = forever).", ParamValue{int64_t{1500}}},
        {"sigma_field_m", ParamMutability::HotMutable, "A field return's height uncertainty (m).", ParamValue{0.02}},
        {"sigma_touch_m", ParamMutability::HotMutable, "A touch's height uncertainty (m); also the floor of any voxel's uncertainty.", ParamValue{0.005}},
        {"predict_tau_touches", ParamMutability::HotMutable, "Touches per leg over which its landing position's mean adapts.", ParamValue{20.0}},
        {"predict_radius_m", ParamMutability::HotMutable, "Voxels within this of the predicted landing point are pooled, precision-weighted.", ParamValue{0.045}},
        {"plane_tau_ticks", ParamMutability::HotMutable, "EMA of the planted feet's plane (the touch height's fallback floor).", ParamValue{50.0}},
        {"err_alpha",   ParamMutability::HotMutable, "EMA rate of the squared prediction error per leg (the running RMS).", ParamValue{0.05}},
        {"datum_gain",  ParamMutability::HotMutable, "The fraction of a touch's disagreement with ALREADY-TOUCHED ground that moves the body's height above the datum (1 = the ground did not move, the body did).", ParamValue{1.0}},
        {"datum_min_load", ParamMutability::HotMutable, "The load (fraction of body weight) a touching toe must reach within 25 ticks for its disagreement to move the datum.", ParamValue{0.15}},
        {"datum_step_max_m", ParamMutability::HotMutable, "The most one touch moves the datum (m): a riser takes a few bearing touches, a single bad one cannot run away with it.", ParamValue{0.03}},
        {"datum_window", ParamMutability::HotMutable, "The datum moves toward the MEDIAN of this many recent bearing innovations (1 = each touch alone, the 2026-11-07 morning form): a toe on a voxel straddling a riser's edge disagrees by the riser, and one such touch must not move the body.", ParamValue{int64_t{5}}},
        {"diag_max",    ParamMutability::HotMutable, "Voxels listed in the diag, nearest first (0 = all).", ParamValue{int64_t{300}}},
        {"recent_max",  ParamMutability::HotMutable, "Recent prediction errors kept in the diag as rows [tick, leg, h_pred, prec, h_touch, e].", ParamValue{int64_t{32}}},
    };
}

ParamMap TerrainMemory::current_params() const {
    return {{"field_topic", ParamValue{field_topic_}}, {"contact_topic", ParamValue{contact_topic_}}, {"feet_xz_topic", ParamValue{feet_xz_topic_}},
            {"feet_y_topic", ParamValue{feet_y_topic_}}, {"odom_topic", ParamValue{odom_topic_}}, {"predict_topic", ParamValue{predict_topic_}}, {"error_topic", ParamValue{error_topic_}},
            {"pitch_deg", ParamValue{pitch_deg_}}, {"sensor_x_m", ParamValue{sensor_x_m_}}, {"sensor_y_m", ParamValue{sensor_y_m_}}, {"sensor_z_m", ParamValue{sensor_z_m_}},
            {"max_range_m", ParamValue{max_range_m_}}, {"thing_min_m", ParamValue{thing_min_m_}}, {"fit_rows", ParamValue{int64_t(fit_rows_)}}, {"fit_slope_max", ParamValue{fit_slope_max_}},
            {"voxel_m", ParamValue{voxel_m_}}, {"max_voxels", ParamValue{int64_t(max_voxels_)}}, {"max_age_ticks", ParamValue{int64_t(max_age_ticks_)}},
            {"sigma_field_m", ParamValue{sigma_field_m_}}, {"sigma_touch_m", ParamValue{sigma_touch_m_}}, {"predict_tau_touches", ParamValue{predict_tau_touches_}},
            {"predict_radius_m", ParamValue{predict_radius_m_}}, {"plane_tau_ticks", ParamValue{plane_tau_ticks_}}, {"err_alpha", ParamValue{err_alpha_}},
            {"diag_max", ParamValue{int64_t(diag_max_)}}, {"recent_max", ParamValue{int64_t(recent_max_)}}, {"up_topic", ParamValue{up_topic_}}, {"datum_gain", ParamValue{datum_gain_}}, {"load_topic", ParamValue{load_topic_}},
            {"datum_min_load", ParamValue{datum_min_load_}}, {"datum_step_max_m", ParamValue{datum_step_max_m_}},
            {"stall_topic", ParamValue{stall_topic_}}, {"stall_gate", ParamValue{stall_gate_}}, {"register_gain", ParamValue{register_gain_}}, {"datum_window", ParamValue{int64_t(datum_window_)}},
            {"register_range_m", ParamValue{register_range_m_}}, {"register_min_points", ParamValue{int64_t(register_min_points_)}}, {"register_min_spread_m", ParamValue{register_min_spread_m_}},
            {"datum_edge_max_m", ParamValue{datum_edge_max_m_}}};
}

void TerrainMemory::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("TerrainMemory requires a non-null Bus");
    for (auto pr : std::initializer_list<std::pair<const char*, std::string*>>{
            {"field_topic", &field_topic_}, {"contact_topic", &contact_topic_}, {"feet_xz_topic", &feet_xz_topic_}, {"feet_y_topic", &feet_y_topic_},
            {"odom_topic", &odom_topic_}, {"predict_topic", &predict_topic_}, {"error_topic", &error_topic_}, {"up_topic", &up_topic_}, {"load_topic", &load_topic_}, {"stall_topic", &stall_topic_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    for (auto const& k : {"pitch_deg", "sensor_x_m", "sensor_y_m", "sensor_z_m", "max_range_m", "thing_min_m", "fit_rows", "fit_slope_max", "voxel_m",
                          "max_voxels", "max_age_ticks", "sigma_field_m", "sigma_touch_m", "predict_tau_touches", "predict_radius_m", "plane_tau_ticks",
                          "err_alpha", "diag_max", "recent_max", "datum_gain", "datum_min_load", "datum_step_max_m", "stall_gate", "register_gain", "register_range_m", "register_min_points", "register_min_spread_m", "datum_window", "datum_edge_max_m"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    auto sub = [&](std::string const& topic, std::string which) {
        if (topic.empty()) return;
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct, [this, which](std::string_view, MessagePtr p){ handle(which, p); }));
    };
    sub(field_topic_, "field"); sub(contact_topic_, "contact"); sub(feet_xz_topic_, "xz"); sub(feet_y_topic_, "y"); sub(odom_topic_, "odom"); sub(up_topic_, "up"); sub(load_topic_, "load"); sub(stall_topic_, "stall");
}

void TerrainMemory::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "pitch_deg")     pitch_deg_ = get_double(value, k);
    else if (k == "sensor_x_m")    sensor_x_m_ = get_double(value, k);
    else if (k == "sensor_y_m")    sensor_y_m_ = get_double(value, k);
    else if (k == "sensor_z_m")    sensor_z_m_ = get_double(value, k);
    else if (k == "max_range_m")   max_range_m_ = std::max(0.1, get_double(value, k));
    else if (k == "thing_min_m")   thing_min_m_ = std::max(0.001, get_double(value, k));
    else if (k == "fit_rows")      fit_rows_ = std::clamp(int(get_double(value, k)), 1, body::tof8::kRows);
    else if (k == "fit_slope_max") fit_slope_max_ = std::max(0.05, get_double(value, k));
    else if (k == "voxel_m")       voxel_m_ = std::max(0.005, get_double(value, k));
    else if (k == "max_voxels")    max_voxels_ = std::max(1, int(get_double(value, k)));
    else if (k == "max_age_ticks") max_age_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "sigma_field_m") sigma_field_m_ = std::max(1e-4, get_double(value, k));
    else if (k == "sigma_touch_m") sigma_touch_m_ = std::max(1e-4, get_double(value, k));
    else if (k == "predict_tau_touches") predict_tau_touches_ = std::max(1.0, get_double(value, k));
    else if (k == "predict_radius_m") predict_radius_m_ = std::max(0.0, get_double(value, k));
    else if (k == "plane_tau_ticks") plane_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "err_alpha")     err_alpha_ = std::clamp(get_double(value, k), 1e-4, 1.0);
    else if (k == "diag_max")      diag_max_ = std::max(0, int(get_double(value, k)));
    else if (k == "recent_max")    recent_max_ = std::max(0, int(get_double(value, k)));
    else if (k == "datum_gain")    datum_gain_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "datum_min_load") datum_min_load_ = std::max(0.0, get_double(value, k));
    else if (k == "datum_step_max_m") datum_step_max_m_ = std::max(0.001, get_double(value, k));
    else if (k == "stall_gate")    stall_gate_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "register_gain") register_gain_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "datum_window")  datum_window_ = std::max(1, int(get_double(value, k)));
    else if (k == "datum_edge_max_m") datum_edge_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "register_range_m") register_range_m_ = std::max(0.0, get_double(value, k));
    else if (k == "register_min_points") register_min_points_ = std::max(1, int(get_double(value, k)));
    else if (k == "register_min_spread_m") register_min_spread_m_ = std::max(0.0, get_double(value, k));
    else throw std::invalid_argument("TerrainMemory: param '" + k + "' is construction-only / unknown");
}

void TerrainMemory::handle(std::string const& which, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    auto const& v = pt->values;
    if      (which == "field"   && v.size() >= body::tof8::kZones) { for (int z = 0; z < body::tof8::kZones; ++z) field_[size_t(z)] = v[z]; have_field_ = true; field_tick_ = pt->tick_id; }
    else if (which == "contact" && v.size() >= 4) { for (int i = 0; i < 4; ++i) contact_[size_t(i)] = v[i]; have_contact_ = true; }
    else if (which == "xz"      && v.size() >= 8) { for (int i = 0; i < 8; ++i) feet_xz_[size_t(i)] = v[i]; have_xz_ = true; }
    else if (which == "y"       && v.size() >= 4) { for (int i = 0; i < 4; ++i) feet_y_[size_t(i)] = v[i]; have_y_ = true; }
    else if (which == "odom"    && v.size() >= 3) {
        // the memory's frame: the odometry's yaw as published; its translation integrated from the increments, each scaled by
        // the body's trust in its own motion (1 - stall_gate x stall) when a stall topic is read
        if (have_raw_odom_) {
            const double trust = stall_topic_.empty() ? 1.0 : std::clamp(1.0 - stall_gate_ * stall_, 0.0, 1.0);
            ox_ += (double(v[0]) - raw_ox_) * trust; oy_ += (double(v[1]) - raw_oy_) * trust;
        } else { ox_ = v[0]; oy_ = v[1]; }
        raw_ox_ = v[0]; raw_oy_ = v[1]; have_raw_odom_ = true; oyaw_ = v[2]; have_odom_ = true;
    }
    else if (which == "stall"   && v.size() >= 1) { stall_ = std::clamp(double(v[0]), 0.0, 1.0); }
    else if (which == "load"    && v.size() >= 4) { for (int i = 0; i < 4; ++i) load_[size_t(i)] = v[i]; have_load_ = true; }
    else if (which == "up"      && v.size() >= 3) { const double n = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]); if (n > 0.5) { up_ = {v[0] / n, v[1] / n, v[2] / n}; have_up_ = true; } }
}

void TerrainMemory::odom_pose(double& px, double& py, double& c, double& s) const {
    if (!have_odom_) { px = 0; py = 0; c = 1; s = 0; return; }
    px = ox_; py = oy_; c = std::cos(oyaw_); s = std::sin(oyaw_);
}

void TerrainMemory::evict_if_full() {
    while (int(vox_.size()) >= max_voxels_) {
        auto oldest = vox_.begin();
        for (auto it = vox_.begin(); it != vox_.end(); ++it) if (it->second.last_t < oldest->second.last_t) oldest = it;
        vox_.erase(oldest);
    }
}

// One claim about the ground's height at the odom point (X, Y): a Kalman update of the voxel's estimate, its variance
// floored at a touch's.
void TerrainMemory::observe(double X, double Y, double h, double sigma, int cls, bool touch, uint64_t tick_id) {
    const Key k{int64_t(std::floor(X / voxel_m_)), int64_t(std::floor(Y / voxel_m_))};
    auto it = vox_.find(k);
    if (it == vox_.end()) {
        evict_if_full();
        Voxel v; v.h = h; v.var = sigma * sigma; v.cls = cls; v.first_t = tick_id; v.last_t = tick_id; (touch ? v.n_touch : v.n_field) = 1;
        vox_.emplace(k, v); return;
    }
    Voxel& v = it->second;
    if (!touch && v.n_touch > 0) {
        // touched ground belongs to the toes: the field refreshes its age and class but no longer moves its height
        // (at 15 Hz the field's 2 cm claims would erase a toe's 0.5 cm one within a second -- 2026-11-07)
        if (cls != 0) v.cls = cls;
        v.last_t = tick_id; ++v.n_field; return;
    }
    const double r = sigma * sigma;
    const double K = v.var / (v.var + r);
    v.h += K * (h - v.h);
    v.var = std::max(sigma_touch_m_ * sigma_touch_m_, (1.0 - K) * v.var);
    if (cls != 0) v.cls = cls;   // the latest reading's class (a touch leaves the field's class alone)
    v.last_t = tick_id; (touch ? v.n_touch : v.n_field) += 1;
}

void TerrainMemory::ingest_field(uint64_t tick_id) {
    using namespace body::tof8;
    ++frames_;
    std::array<Vec3, kZones> P{}; std::array<bool, kZones> ok{};
    points(field_.data(), pitch_deg_, max_range_m_, P, ok);
    plane_ = fit_floor(P, ok, fit_rows_, thing_min_m_, fit_slope_max_);
    if (!plane_.ok) return;   // nose at a wall or no floor: nothing is placed this frame
    // the frame's points: body-frame place (bx, bz), class from the field's own floor, height in the body's
    // gravity-levelled frame plus the datum
    std::array<double, kZones> BX{}, BZ{}, HM{}; std::array<int, kZones> CLS{}; int n_pts = 0; double hsum = 0, hsq = 0;
    for (int z = 0; z < kZones; ++z) {
        if (!ok[size_t(z)]) continue;
        const double bx = P[size_t(z)].x + sensor_x_m_, by = P[size_t(z)].y + sensor_y_m_, bz = P[size_t(z)].z + sensor_z_m_;
        const double over_floor = by - (plane_.y_at(P[size_t(z)].x, P[size_t(z)].z) + sensor_y_m_);
        CLS[size_t(z)] = over_floor > thing_min_m_ ? kThing : (over_floor < -thing_min_m_ ? kDrop : kFloor);
        const double h_level = have_up_ ? bx * up_[0] + by * up_[1] + bz * up_[2] : by;
        BX[size_t(z)] = bx; BZ[size_t(z)] = bz; HM[size_t(z)] = h_level + datum_; ++n_pts; hsum += HM[size_t(z)]; hsq += HM[size_t(z)] * HM[size_t(z)];
    }
    double px, py, c, s; odom_pose(px, py, c, s);
    // registration: the forward shift of the frame that best explains the heights already remembered
    reg_last_shift_ = 0.0; reg_last_matched_ = 0; reg_last_gain_ratio_ = 1.0;
    const double spread = n_pts > 1 ? std::sqrt(std::max(0.0, hsq / n_pts - (hsum / n_pts) * (hsum / n_pts))) : 0.0;
    if (register_gain_ > 0.0 && register_range_m_ > 0.0 && spread >= register_min_spread_m_ && !vox_.empty()) {
        const double step = 0.5 * voxel_m_; const int K = int(std::ceil(register_range_m_ / step));
        double best_cost = 1e18, cost0 = 1e18; double best_shift = 0.0; int best_n = 0;
        for (int k = -K; k <= K; ++k) {
            const double sh = k * step; double cost = 0; int n = 0;
            for (int z = 0; z < kZones; ++z) {
                if (!ok[size_t(z)]) continue;
                const double bz = BZ[size_t(z)] + sh, bx = BX[size_t(z)];
                const double X = px + bz * c - bx * s, Y = py + bz * s + bx * c;
                auto it = vox_.find(Key{int64_t(std::floor(X / voxel_m_)), int64_t(std::floor(Y / voxel_m_))});
                if (it == vox_.end()) continue;
                const double d = HM[size_t(z)] - it->second.h; cost += d * d; ++n;
            }
            if (n < register_min_points_) continue;
            cost /= n;
            if (k == 0) cost0 = cost;
            if (cost < best_cost || (cost == best_cost && std::fabs(sh) < std::fabs(best_shift))) { best_cost = cost; best_shift = sh; best_n = n; }
        }
        if (best_n >= register_min_points_ && cost0 < 1e17 && best_shift != 0.0 && best_cost < 0.8 * cost0) {
            // the frame moved too far forward by best_shift (the remembered ground sits best_shift further along the body's
            // forward than the frame says): pull the frame back by a fraction of it
            const double move = register_gain_ * best_shift;
            ox_ += move * c; oy_ += move * s; reg_applied_ += move; ++reg_count_;
            reg_last_shift_ = best_shift; reg_last_matched_ = best_n; reg_last_gain_ratio_ = best_cost / cost0;
            odom_pose(px, py, c, s);
        }
    }
    for (int z = 0; z < kZones; ++z) {
        if (!ok[size_t(z)]) continue;
        const double X = px + BZ[size_t(z)] * c - BX[size_t(z)] * s, Y = py + BZ[size_t(z)] * s + BX[size_t(z)] * c;
        observe(X, Y, HM[size_t(z)], sigma_field_m_, CLS[size_t(z)], false, tick_id);
    }
}

void TerrainMemory::ingest_touch(int leg, double fwd, double lat, double h, uint64_t tick_id) {
    ++touches_;
    // the prediction error, before the touch corrects anything (both in the body's gravity frame)
    auto const& pr = pred_[size_t(leg)];
    if (pr.has) {
        const double e = h - pr.h;
        err_last_[size_t(leg)] = e; err_valid_[size_t(leg)] = true; ++n_err_;
        err_ms_[size_t(leg)] += err_alpha_ * (e * e - err_ms_[size_t(leg)]);
        double px0, py0, c0, s0; odom_pose(px0, py0, c0, s0);
        auto it0 = vox_.find(Key{int64_t(std::floor((px0 + fwd * c0 - lat * s0) / voxel_m_)), int64_t(std::floor((py0 + fwd * s0 + lat * c0) / voxel_m_))});
        const int nt = it0 == vox_.end() ? 0 : it0->second.n_touch, nf = it0 == vox_.end() ? 0 : it0->second.n_field;
        if (recent_max_ > 0) {
            recent_.push_back({float(tick_id), float(leg), float(pr.h), float(pr.prec), float(h), float(e), float(nt), float(nf),
                               float(std::hypot(fwd - pr.fwd, lat - pr.lat))});   // how far the toe landed from the predicted spot
            while (int(recent_.size()) > recent_max_) recent_.pop_front();
        }
    } else err_valid_[size_t(leg)] = false;
    // the landing position's running mean (the next prediction's place)
    auto& m = land_mean_[size_t(leg)]; auto& n = land_n_[size_t(leg)];
    if (n == 0) { m = {fwd, lat}; } else { const double a = 1.0 / std::min(double(n + 1), predict_tau_touches_); m[0] += a * (fwd - m[0]); m[1] += a * (lat - m[1]); }
    ++n;
    double px, py, c, s; odom_pose(px, py, c, s);
    const double X = px + fwd * c - lat * s, Y = py + fwd * s + lat * c;
    // touched ground calibrates the body's height above the datum; ground only the field saw is corrected by the toe
    auto it = vox_.find(Key{int64_t(std::floor(X / voxel_m_)), int64_t(std::floor(Y / voxel_m_))});
    if (it != vox_.end() && it->second.n_touch > 0) {
        const double innov = (h + datum_) - it->second.h;     // > 0: the body reads higher than the ground it knew -> it is lower
        it->second.last_t = tick_id; ++it->second.n_touch;
        it->second.var = std::max(sigma_touch_m_ * sigma_touch_m_, it->second.var * 0.5);
        if (datum_edge_max_m_ > 0.0) {   // only flat known ground may move the body: an edge cell is ambiguous by the riser
            const Key k0 = it->first; double lo = 1e9, hi = -1e9;
            for (int64_t dx = -1; dx <= 1; ++dx) for (int64_t dy = -1; dy <= 1; ++dy) {
                auto n = vox_.find(Key{k0.ix + dx, k0.iy + dy}); if (n == vox_.end()) continue;
                lo = std::min(lo, n->second.h); hi = std::max(hi, n->second.h);
            }
            if (hi - lo > datum_edge_max_m_) { ++datum_edge_skips_; return; }
        }
        if (load_topic_.empty() || !have_load_) datum_move(innov);
        else { pending_datum_[size_t(leg)] = 25; pending_innov_[size_t(leg)] = innov; }   // the toe must come to bear first
        return;
    }
    observe(X, Y, h + datum_, sigma_touch_m_, 0, true, tick_id);
}

// One bearing innovation (the body reads `innov` higher than the ground it knew): the datum moves toward the median of the
// recent ones, by datum_gain, bounded; the innovations already explained by earlier moves are discounted as the datum moves.
void TerrainMemory::datum_move(double innov) {
    datum_innov_.push_back(innov);
    while (int(datum_innov_.size()) > datum_window_) datum_innov_.pop_front();
    std::vector<double> v(datum_innov_.begin(), datum_innov_.end()); std::sort(v.begin(), v.end());
    const double med = (v.size() % 2) ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]);
    const double step = datum_gain_ * std::clamp(med, -datum_step_max_m_, datum_step_max_m_);
    if (step == 0.0) return;
    datum_ -= step; ++datum_updates_;
    for (auto& x : datum_innov_) x -= step;   // what the move explained is no longer owed
}

void TerrainMemory::note_load(int leg, uint64_t) {
    if (pending_datum_[size_t(leg)] <= 0) return;
    if (contact_[size_t(leg)] <= 0.5f) { pending_datum_[size_t(leg)] = 0; return; }   // it left: a scrape, not a step
    if (load_[size_t(leg)] >= datum_min_load_) { datum_move(pending_innov_[size_t(leg)]); pending_datum_[size_t(leg)] = 0; return; }
    --pending_datum_[size_t(leg)];
}

TerrainMemory::Prediction TerrainMemory::predict_at(double fwd, double lat) const {
    Prediction p; p.fwd = fwd; p.lat = lat;
    double px, py, c, s; odom_pose(px, py, c, s);
    const double X = px + fwd * c - lat * s, Y = py + fwd * s + lat * c;
    const int64_t ix = int64_t(std::floor(X / voxel_m_)), iy = int64_t(std::floor(Y / voxel_m_));
    const int reach = int(std::ceil(predict_radius_m_ / voxel_m_));
    double sw = 0, swh = 0;
    for (int64_t dx = -reach; dx <= reach; ++dx) for (int64_t dy = -reach; dy <= reach; ++dy) {
        auto it = vox_.find(Key{ix + dx, iy + dy});
        if (it == vox_.end()) continue;
        const double cx = (double(ix + dx) + 0.5) * voxel_m_, cy = (double(iy + dy) + 0.5) * voxel_m_;
        if (std::hypot(cx - X, cy - Y) > predict_radius_m_ + 0.5 * voxel_m_) continue;
        const double w = 1.0 / std::max(1e-9, it->second.var);
        sw += w; swh += w * it->second.h;
    }
    if (sw > 0) { p.h = swh / sw - datum_; p.prec = sw; p.has = true; }
    return p;
}

void TerrainMemory::tick(uint64_t tick_id) {
    last_tick_ = tick_id;
    if (max_age_ticks_ > 0)
        for (auto it = vox_.begin(); it != vox_.end();)
            it = (tick_id > it->second.last_t && tick_id - it->second.last_t > uint64_t(max_age_ticks_)) ? vox_.erase(it) : std::next(it);
    if (have_field_ && field_tick_ != field_done_) { ingest_field(tick_id); field_done_ = field_tick_; }
    // the touches
    std::array<float, 8> err_out{};
    if (have_contact_) {
        bool rising[4];
        for (int i = 0; i < 4; ++i) rising[i] = contact_[size_t(i)] > 0.5f && prev_contact_[size_t(i)] <= 0.5f;
        if (have_y_) {   // the planted feet's plane: the fallback floor
            float planted[4]; int n = 0;
            for (int i = 0; i < 4; ++i) if (contact_[size_t(i)] > 0.5f && !rising[i]) planted[n++] = feet_y_[size_t(i)];
            if (n > 0) { std::sort(planted, planted + n); const double med = (n % 2) ? planted[n / 2] : 0.5 * (planted[n / 2 - 1] + planted[n / 2]);
                if (!have_planted_plane_) { planted_plane_y_ = med; have_planted_plane_ = true; } else planted_plane_y_ += (med - planted_plane_y_) / plane_tau_ticks_; }
        }
        if (have_xz_ && have_y_) {
            for (int i = 0; i < 4; ++i) {
                if (!rising[i]) continue;
                const double lat = feet_xz_[size_t(2 * i)], fwd = feet_xz_[size_t(2 * i + 1)];
                ingest_touch(i, fwd, lat, double(feet_y_[size_t(i)]), tick_id);   // the toe's gravity height, body origin 0
                err_out[size_t(2 * i)] = float(err_last_[size_t(i)]); err_out[size_t(2 * i + 1)] = err_valid_[size_t(i)] ? 1.0f : 0.0f;
            }
        }
        for (int i = 0; i < 4; ++i) { note_load(i, tick_id); prev_contact_[size_t(i)] = contact_[size_t(i)]; }
    }
    // the predictions, every tick
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("terrain_memory") : std::string(id()); out->sensor = "foothold";
    out->values = Eigen::VectorXf(20);
    for (int i = 0; i < 4; ++i) {
        pred_[size_t(i)] = land_n_[size_t(i)] > 0 ? predict_at(land_mean_[size_t(i)][0], land_mean_[size_t(i)][1]) : Prediction{};
        auto const& p = pred_[size_t(i)];
        out->values[5 * i] = float(p.h); out->values[5 * i + 1] = float(p.prec); out->values[5 * i + 2] = p.has ? 1.0f : 0.0f;
        out->values[5 * i + 3] = float(p.fwd); out->values[5 * i + 4] = float(p.lat);
    }
    bus_->publish(predict_topic_, out);
    auto eo = std::make_shared<ProprioToken>();
    eo->tick_id = tick_id; eo->producer_id = out->producer_id; eo->sensor = "foothold_error";
    eo->values = Eigen::VectorXf(8); for (int k = 0; k < 8; ++k) eo->values[k] = err_out[size_t(k)];
    bus_->publish(error_topic_, eo);
}

std::vector<TerrainMemory::BodyVoxel> TerrainMemory::body_voxels(int n) const {
    double px, py, c, s; odom_pose(px, py, c, s);
    std::vector<BodyVoxel> out; out.reserve(vox_.size());
    for (auto const& [k, v] : vox_) {
        const double x = (double(k.ix) + 0.5) * voxel_m_, y = (double(k.iy) + 0.5) * voxel_m_;
        const double dx = x - px, dy = y - py;
        out.push_back(BodyVoxel{dx * c + dy * s, -dx * s + dy * c, v.h - datum_, v.var, v.n_field, v.n_touch, v.cls, last_tick_ >= v.last_t ? last_tick_ - v.last_t : 0});
    }
    std::sort(out.begin(), out.end(), [](BodyVoxel const& a, BodyVoxel const& b) { return a.fwd * a.fwd + a.lat * a.lat < b.fwd * b.fwd + b.lat * b.lat; });
    if (n > 0 && int(out.size()) > n) out.resize(size_t(n));
    return out;
}

nlohmann::json TerrainMemory::diag_snapshot() const {
    nlohmann::json vs = nlohmann::json::array();
    for (auto const& b : body_voxels(diag_max_))
        vs.push_back({std::round(b.fwd * 1e3) / 1e3, std::round(b.lat * 1e3) / 1e3, std::round(b.h * 1e4) / 1e4, std::round(b.var * 1e6) / 1e6, b.n_field, b.n_touch, b.cls, b.age});
    nlohmann::json pr = nlohmann::json::array(), er = nlohmann::json::array(), rms = nlohmann::json::array(), rec = nlohmann::json::array();
    for (int i = 0; i < 4; ++i) { pr.push_back({pred_[size_t(i)].h, pred_[size_t(i)].prec, pred_[size_t(i)].has ? 1 : 0, pred_[size_t(i)].fwd, pred_[size_t(i)].lat});
                                  er.push_back(err_valid_[size_t(i)] ? err_last_[size_t(i)] : 0.0); rms.push_back(std::sqrt(err_ms_[size_t(i)])); }
    for (auto const& r : recent_) { nlohmann::json row = nlohmann::json::array(); for (float x : r) row.push_back(x); rec.push_back(row); }
    return nlohmann::json{{"voxels", vs}, {"count", int(vox_.size())}, {"frames", frames_}, {"touches", touches_}, {"floor_ok", plane_.ok},
                          {"datum", datum_}, {"datum_updates", datum_updates_}, {"have_up", have_up_}, {"frame", {ox_, oy_, oyaw_}}, {"stall", stall_},
                          {"datum_edge_skips", datum_edge_skips_},
                          {"reg_applied", reg_applied_}, {"reg_count", reg_count_}, {"reg_last_shift", reg_last_shift_}, {"reg_last_matched", reg_last_matched_}, {"reg_last_ratio", reg_last_gain_ratio_},
                          {"plane", {plane_.a, plane_.b, plane_.c}}, {"prediction", pr}, {"err_last", er}, {"err_rms", rms}, {"n_err", n_err_}, {"recent", rec}};
}

} // namespace ogma
