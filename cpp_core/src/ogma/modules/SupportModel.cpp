#include "ogma/modules/SupportModel.hpp"
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
    throw std::invalid_argument("SupportModel: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("SupportModel: param '" + k + "' must be a string");
}
using P = SupportModel::Point;
double dist(P a, P b) { return std::hypot(a.fwd - b.fwd, a.lat - b.lat); }
// the distance from c to the segment a-b
double seg_dist(P a, P b, P c) {
    const double dx = b.fwd - a.fwd, dy = b.lat - a.lat, L2 = dx * dx + dy * dy;
    if (L2 < 1e-12) return dist(a, c);
    const double t = std::clamp(((c.fwd - a.fwd) * dx + (c.lat - a.lat) * dy) / L2, 0.0, 1.0);
    return dist(P{a.fwd + t * dx, a.lat + t * dy}, c);
}
double cross(P o, P a, P b) { return (a.fwd - o.fwd) * (b.lat - o.lat) - (a.lat - o.lat) * (b.fwd - o.fwd); }
// 3x3 inverse by cofactors; false when singular
bool inv3(const double m[9], double out[9]) {
    const double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (std::fabs(det) < 1e-12) return false;
    const double d = 1.0 / det;
    out[0] =  (m[4] * m[8] - m[5] * m[7]) * d; out[1] = -(m[1] * m[8] - m[2] * m[7]) * d; out[2] =  (m[1] * m[5] - m[2] * m[4]) * d;
    out[3] = -(m[3] * m[8] - m[5] * m[6]) * d; out[4] =  (m[0] * m[8] - m[2] * m[6]) * d; out[5] = -(m[0] * m[5] - m[2] * m[3]) * d;
    out[6] =  (m[3] * m[7] - m[4] * m[6]) * d; out[7] = -(m[0] * m[7] - m[1] * m[6]) * d; out[8] =  (m[0] * m[4] - m[1] * m[3]) * d;
    return true;
}
} // namespace

SupportModel::SupportModel()  = default;
SupportModel::~SupportModel() = default;
std::string_view SupportModel::type_name() const { return "SupportModel"; }

std::vector<TopicSpec> SupportModel::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : {feet_xz_topic_, feet_y_topic_, up_topic_, contact_topic_, load_topic_, patch_topic_})
        if (!t.empty()) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> SupportModel::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema SupportModel::params_schema() const {
    return {
        {"feet_xz_topic", ParamMutability::ConstructionOnly, "ProprioToken [x, z] x 4: the toes in the body frame (x = +left, z = forward), the commanded FK.", ParamValue{std::string("reality.proprio.feet_xz")}},
        {"feet_y_topic",  ParamMutability::ConstructionOnly, "ProprioToken [4]: the toe TIP's height along gravity, body origin 0.", ParamValue{std::string("reality.proprio.toe_y")}},
        {"up_topic",      ParamMutability::ConstructionOnly, "ProprioToken [3]: the body's gravity-up in the body frame; levels the toes and the masses.", ParamValue{std::string("reality.proprio.up_body")}},
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4] per-leg contact: grades the contact prediction, and stands in for the ground where the memory has none.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"load_topic",    ParamMutability::ConstructionOnly, "ProprioToken [4] foot load as a fraction of body weight: grades the load prediction. \"\" = no grading.", ParamValue{std::string("reality.proprio.foot_load")}},
        {"patch_topic",   ParamMutability::ConstructionOnly, "ProprioToken: TerrainMemory's patch (its patch_topic): the remembered heights on a body-centred grid.", ParamValue{std::string("terrain.patch")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "ProprioToken [margin, n_support, com_fwd, com_lat, gap_slope_fwd, gap_slope_lat] + [gap, has, contact, load, lift_margin] x 4.", ParamValue{std::string("support.state")}},
        {"mass_chassis_kg", ParamMutability::HotMutable, "The chassis's mass (the sim's cad.json: 0.3).", ParamValue{0.3}},
        {"mass_leg_kg",   ParamMutability::HotMutable, "One leg's mass, all three segments (0.075).", ParamValue{0.075}},
        {"leg_com_frac",  ParamMutability::HotMutable, "Where a leg's mass sits, as a fraction of the way from its hip to its toe (coxa, upper and lower at ~0.2, 0.5, 0.75 -> 0.48).", ParamValue{0.48}},
        {"com_x_m",       ParamMutability::HotMutable, "The chassis's centre of mass in the body frame, x (+left).", ParamValue{0.0}},
        {"com_y_m",       ParamMutability::HotMutable, "... y (up).", ParamValue{0.0}},
        {"com_z_m",       ParamMutability::HotMutable, "... z (forward).", ParamValue{0.0}},
        {"hip_lat_m",     ParamMutability::HotMutable, "The hip rectangle's half width (m): legs 0 and 2 at -x, 1 and 3 at +x (the side the toes' frame puts them on).", ParamValue{0.0383}},
        {"share_min",     ParamMutability::HotMutable, "The load prediction is also graded against each toe's SHARE of the FSR sum, on ticks when the four carry at least this fraction of the weight.", ParamValue{0.3}},
        {"hip_fwd_m",     ParamMutability::HotMutable, "The hip rectangle's half length (m): the front hips at +.", ParamValue{0.033}},
        {"gap_tol_m",     ParamMutability::HotMutable, "A toe within this of the remembered ground is predicted to bear (the gap prediction; graded against the flag).", ParamValue{0.015}},
        {"support_source", ParamMutability::HotMutable, "Which toes bear in the evaluated (current) pose: \"flag\" = the contact flag (default: the FSR is the sensor of bearing); \"gap\" = the memory's prediction, as a candidate pose would be judged.", ParamValue{std::string("flag")}},
        {"unsafe_margin_m", ParamMutability::HotMutable, "A toe that leaves (its flag falls) while its lift margin is below this counts as an UNSAFE LIFT (m).", ParamValue{-0.02}},
        {"margin_floor",  ParamMutability::HotMutable, "The margin reported with no support at all (m).", ParamValue{-0.10}},
        {"err_alpha",     ParamMutability::HotMutable, "EMA rate of the squared load error per leg.", ParamValue{0.02}},
        {"up_min",        ParamMutability::HotMutable, "Below this body-up component the body is on its side: no support is evaluated.", ParamValue{0.5}},
    };
}

ParamMap SupportModel::current_params() const {
    return {{"feet_xz_topic", ParamValue{feet_xz_topic_}}, {"feet_y_topic", ParamValue{feet_y_topic_}}, {"up_topic", ParamValue{up_topic_}},
            {"contact_topic", ParamValue{contact_topic_}}, {"load_topic", ParamValue{load_topic_}}, {"patch_topic", ParamValue{patch_topic_}}, {"output_topic", ParamValue{output_topic_}},
            {"mass_chassis_kg", ParamValue{mass_chassis_}}, {"mass_leg_kg", ParamValue{mass_leg_}}, {"leg_com_frac", ParamValue{leg_com_frac_}},
            {"com_x_m", ParamValue{com_offset_[0]}}, {"com_y_m", ParamValue{com_offset_[1]}}, {"com_z_m", ParamValue{com_offset_[2]}},
            {"hip_lat_m", ParamValue{hip_lat_m_}}, {"hip_fwd_m", ParamValue{hip_fwd_m_}}, {"gap_tol_m", ParamValue{gap_tol_m_}},
            {"margin_floor", ParamValue{margin_floor_}}, {"err_alpha", ParamValue{err_alpha_}}, {"up_min", ParamValue{up_min_}}, {"share_min", ParamValue{share_min_}},
            {"support_source", ParamValue{support_source_}}, {"unsafe_margin_m", ParamValue{unsafe_margin_m_}}};
}

void SupportModel::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("SupportModel requires a non-null Bus");
    for (auto pr : std::initializer_list<std::pair<const char*, std::string*>>{
            {"feet_xz_topic", &feet_xz_topic_}, {"feet_y_topic", &feet_y_topic_}, {"up_topic", &up_topic_}, {"contact_topic", &contact_topic_},
            {"load_topic", &load_topic_}, {"patch_topic", &patch_topic_}, {"output_topic", &output_topic_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    for (auto const& k : {"mass_chassis_kg", "mass_leg_kg", "leg_com_frac", "com_x_m", "com_y_m", "com_z_m", "hip_lat_m", "hip_fwd_m", "gap_tol_m", "margin_floor", "err_alpha", "up_min", "share_min", "unsafe_margin_m", "support_source"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    auto sub = [&](std::string const& topic, std::string which) {
        if (topic.empty()) return;
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct, [this, which](std::string_view, MessagePtr p){ handle(which, p); }));
    };
    sub(feet_xz_topic_, "xz"); sub(feet_y_topic_, "y"); sub(up_topic_, "up"); sub(contact_topic_, "contact"); sub(load_topic_, "load"); sub(patch_topic_, "patch");
}

void SupportModel::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "mass_chassis_kg") mass_chassis_ = std::max(0.0, get_double(value, k));
    else if (k == "mass_leg_kg")     mass_leg_ = std::max(0.0, get_double(value, k));
    else if (k == "leg_com_frac")    leg_com_frac_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "com_x_m")         com_offset_[0] = get_double(value, k);
    else if (k == "com_y_m")         com_offset_[1] = get_double(value, k);
    else if (k == "com_z_m")         com_offset_[2] = get_double(value, k);
    else if (k == "hip_lat_m")       hip_lat_m_ = get_double(value, k);
    else if (k == "hip_fwd_m")       hip_fwd_m_ = get_double(value, k);
    else if (k == "gap_tol_m")       gap_tol_m_ = std::max(0.0, get_double(value, k));
    else if (k == "margin_floor")    margin_floor_ = get_double(value, k);
    else if (k == "err_alpha")       err_alpha_ = std::clamp(get_double(value, k), 1e-4, 1.0);
    else if (k == "up_min")          up_min_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "share_min")       share_min_ = std::max(0.0, get_double(value, k));
    else if (k == "unsafe_margin_m") unsafe_margin_m_ = get_double(value, k);
    else if (k == "support_source")  { const std::string v = get_string(value, k); if (v != "flag" && v != "gap") throw std::invalid_argument("SupportModel: support_source must be \"flag\" or \"gap\""); support_source_ = v; }
}

void SupportModel::handle(std::string const& which, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    auto const& v = pt->values;
    if      (which == "xz"      && v.size() >= 8) { for (int i = 0; i < 8; ++i) xz_[size_t(i)] = v[i]; have_xz_ = true; }
    else if (which == "y"       && v.size() >= 4) { for (int i = 0; i < 4; ++i) y_[size_t(i)] = v[i]; have_y_ = true; }
    else if (which == "contact" && v.size() >= 4) { for (int i = 0; i < 4; ++i) contact_[size_t(i)] = v[i]; have_contact_ = true; }
    else if (which == "load"    && v.size() >= 4) { for (int i = 0; i < 4; ++i) load_[size_t(i)] = v[i]; have_load_ = true; }
    else if (which == "up"      && v.size() >= 3) { const double n = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]); if (n > 0.5) { up_ = {v[0] / n, v[1] / n, v[2] / n}; have_up_ = true; } }
    else if (which == "patch"   && v.size() >= 4) {
        const int N = int(v[0]); if (N < 2 || v.size() < 4 + 2 * N * N) return;
        N_ = N; cell_ = std::max(1e-3, double(v[1]));
        patch_.assign(v.data(), v.data() + 4 + 2 * N * N); have_patch_ = true;
    }
}

bool SupportModel::patch_at(double fwd, double lat, double& h, double& prec, double& spread) const {
    h = 0.0; prec = 0.0; spread = 0.0;
    if (!have_patch_) return false;
    const int ci = int(std::floor(fwd / cell_)) + N_ / 2, cj = int(std::floor(lat / cell_)) + N_ / 2;
    double sw = 0.0, swh = 0.0, lo = 1e9, hi = -1e9;
    for (int di = -1; di <= 1; ++di) for (int dj = -1; dj <= 1; ++dj) {
        const int i = ci + di, j = cj + dj;
        if (i < 0 || j < 0 || i >= N_ || j >= N_) continue;
        const float hv = patch_[size_t(4 + i * N_ + j)]; const double w = patch_[size_t(4 + N_ * N_ + i * N_ + j)];
        if (hv <= -98.0f || w <= 0.0) continue;
        sw += w; swh += w * hv; lo = std::min(lo, double(hv)); hi = std::max(hi, double(hv));
    }
    if (sw <= 0.0) return false;
    h = swh / sw; prec = sw; spread = hi - lo; return true;
}

std::vector<double> SupportModel::loads_of(std::vector<Point> const& pts, Point c) {
    const int n = int(pts.size());
    std::vector<double> w(size_t(n), 0.0);
    if (n == 0) return w;
    if (n == 1) { w[0] = 1.0; return w; }
    auto two = [&](int a, int b) {   // the projection of c onto the segment a-b: the shares two points can carry
        const double dx = pts[size_t(b)].fwd - pts[size_t(a)].fwd, dy = pts[size_t(b)].lat - pts[size_t(a)].lat, L2 = dx * dx + dy * dy;
        const double t = L2 < 1e-12 ? 0.5 : std::clamp(((c.fwd - pts[size_t(a)].fwd) * dx + (c.lat - pts[size_t(a)].lat) * dy) / L2, 0.0, 1.0);
        w[size_t(a)] = 1.0 - t; w[size_t(b)] = t;
    };
    if (n == 2) { two(0, 1); return w; }
    // the minimum-norm solution of [1; fwd; lat] w = [1; c]: w = A^T (A A^T)^-1 b
    double M[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (auto const& p : pts) {
        const double r[3] = {1.0, p.fwd, p.lat};
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) M[i * 3 + j] += r[i] * r[j];
    }
    double Mi[9];
    if (!inv3(M, Mi)) {   // collinear: the two farthest-apart points carry it
        int a = 0, b = 1; double best = -1.0;
        for (int i = 0; i < n; ++i) for (int j = i + 1; j < n; ++j) { const double d = dist(pts[size_t(i)], pts[size_t(j)]); if (d > best) { best = d; a = i; b = j; } }
        two(a, b); return w;
    }
    const double b[3] = {1.0, c.fwd, c.lat};
    double y[3]; for (int i = 0; i < 3; ++i) y[i] = Mi[i * 3] * b[0] + Mi[i * 3 + 1] * b[1] + Mi[i * 3 + 2] * b[2];
    int worst = -1; double wmin = -1e-9;
    for (int k = 0; k < n; ++k) { w[size_t(k)] = y[0] + y[1] * pts[size_t(k)].fwd + y[2] * pts[size_t(k)].lat; if (w[size_t(k)] < wmin) { wmin = w[size_t(k)]; worst = k; } }
    if (worst >= 0) {   // a toe that would have to PULL is unloaded, and the rest re-solved
        std::vector<Point> rest; std::vector<int> idx;
        for (int k = 0; k < n; ++k) if (k != worst) { rest.push_back(pts[size_t(k)]); idx.push_back(k); }
        auto wr = loads_of(rest, c);
        std::fill(w.begin(), w.end(), 0.0);
        for (size_t k = 0; k < idx.size(); ++k) w[size_t(idx[k])] = wr[k];
    }
    return w;
}

double SupportModel::margin_of(std::vector<Point> const& pts_in, Point c) {
    std::vector<Point> pts = pts_in;
    if (pts.empty()) return -1.0;
    if (pts.size() == 1) return -dist(pts[0], c);
    if (pts.size() == 2) return -seg_dist(pts[0], pts[1], c);
    // drop a point inside the triangle of the others (four toes, one under the body)
    if (pts.size() == 4) {
        for (int k = 0; k < 4; ++k) {
            std::vector<Point> tri; for (int j = 0; j < 4; ++j) if (j != k) tri.push_back(pts[size_t(j)]);
            const double s1 = cross(tri[0], tri[1], pts[size_t(k)]), s2 = cross(tri[1], tri[2], pts[size_t(k)]), s3 = cross(tri[2], tri[0], pts[size_t(k)]);
            const bool inside = (s1 >= 0 && s2 >= 0 && s3 >= 0) || (s1 <= 0 && s2 <= 0 && s3 <= 0);
            if (inside && std::fabs(cross(tri[0], tri[1], tri[2])) > 1e-9) { pts = tri; break; }
        }
    }
    // collinear: a segment between the extremes
    bool collinear = true;
    for (size_t k = 2; k < pts.size() && collinear; ++k) if (std::fabs(cross(pts[0], pts[1], pts[k])) > 1e-9) collinear = false;
    if (collinear) {
        int a = 0, b = 1; double best = -1.0;
        for (size_t i = 0; i < pts.size(); ++i) for (size_t j = i + 1; j < pts.size(); ++j) { const double d = dist(pts[i], pts[j]); if (d > best) { best = d; a = int(i); b = int(j); } }
        return -seg_dist(pts[size_t(a)], pts[size_t(b)], c);
    }
    // order counter-clockwise about the centroid; the margin is the least signed distance to an edge (+ inside)
    Point cen{0, 0}; for (auto const& p : pts) { cen.fwd += p.fwd / double(pts.size()); cen.lat += p.lat / double(pts.size()); }
    std::sort(pts.begin(), pts.end(), [&](Point const& a, Point const& b){ return std::atan2(a.lat - cen.lat, a.fwd - cen.fwd) < std::atan2(b.lat - cen.lat, b.fwd - cen.fwd); });
    double m = 1e9;
    for (size_t i = 0; i < pts.size(); ++i) {
        const Point a = pts[i], b = pts[(i + 1) % pts.size()];
        const double L = dist(a, b); if (L < 1e-9) continue;
        m = std::min(m, cross(a, b, c) / L);
    }
    return m;
}

void SupportModel::evaluate() {
    Eval e;
    const double ux = up_[0], uy = up_[1], uz = up_[2];
    if (uy < up_min_) { e.margin = margin_floor_; eval_ = e; have_eval_ = true; return; }   // on its side: nothing bears
    // the levelled frame: f forward-horizontal, l left-horizontal
    double fx = -uz * ux, fy = -uz * uy, fz = 1.0 - uz * uz; { const double n = std::sqrt(fx * fx + fy * fy + fz * fz); fx /= n; fy /= n; fz /= n; }
    const double lx = uy * fz - uz * fy, ly = uz * fx - ux * fz, lz = ux * fy - uy * fx;
    auto level = [&](double x, double y, double z) { return Point{x * fx + y * fy + z * fz, x * lx + y * ly + z * lz}; };
    // the toes, completed to body points and levelled; the ground under each
    std::array<Point, 4> toe{}; std::array<std::array<double, 3>, 4> toe_b{};
    for (int i = 0; i < 4; ++i) {
        const double x = xz_[size_t(2 * i)], z = xz_[size_t(2 * i + 1)], ty = y_[size_t(i)];
        const double yb = (ty - x * ux - z * uz) / uy;
        toe_b[size_t(i)] = {x, yb, z}; toe[size_t(i)] = level(x, yb, z);
        double h, prec, spread;
        e.has[size_t(i)] = patch_at(toe[size_t(i)].fwd, toe[size_t(i)].lat, h, prec, spread);
        e.gap[size_t(i)] = e.has[size_t(i)] ? ty - h : 0.0;
        e.gap_contact[size_t(i)] = e.has[size_t(i)] ? e.gap[size_t(i)] <= gap_tol_m_ : (have_contact_ && contact_[size_t(i)] > 0.5f);
        e.contact[size_t(i)] = (support_source_ == "flag" && have_contact_) ? contact_[size_t(i)] > 0.5f : e.gap_contact[size_t(i)];
        if (!e.has[size_t(i)]) ++no_ground_;
    }
    // the centre of mass: the chassis plus each leg at leg_com_frac of hip -> toe
    const double mt = mass_chassis_ + 4.0 * mass_leg_;
    Point cm = level(com_offset_[0], com_offset_[1], com_offset_[2]); cm.fwd *= mass_chassis_ / mt; cm.lat *= mass_chassis_ / mt;
    for (int i = 0; i < 4; ++i) {
        const double sx = (i == 0 || i == 2) ? -1.0 : 1.0, sz = (i == 0 || i == 1) ? 1.0 : -1.0;   // legs 0 and 2 at -x, as the toes' frame has them
        const double hx = sx * hip_lat_m_, hy = 0.0, hz = sz * hip_fwd_m_;
        const Point lc = level(hx + leg_com_frac_ * (toe_b[size_t(i)][0] - hx), hy + leg_com_frac_ * (toe_b[size_t(i)][1] - hy), hz + leg_com_frac_ * (toe_b[size_t(i)][2] - hz));
        cm.fwd += lc.fwd * mass_leg_ / mt; cm.lat += lc.lat * mass_leg_ / mt;
    }
    e.com_fwd = cm.fwd; e.com_lat = cm.lat;
    // the support: the bearing toes
    std::vector<Point> sup; std::vector<int> idx;
    for (int i = 0; i < 4; ++i) if (e.contact[size_t(i)]) { sup.push_back(toe[size_t(i)]); idx.push_back(i); }
    e.n_support = int(sup.size());
    e.margin = sup.empty() ? margin_floor_ : std::max(margin_floor_, margin_of(sup, cm));
    const auto w = loads_of(sup, cm);
    for (size_t k = 0; k < idx.size(); ++k) e.load[size_t(idx[k])] = w[k];
    for (int i = 0; i < 4; ++i) {
        if (!e.contact[size_t(i)]) { e.lift_margin[size_t(i)] = e.margin; continue; }
        std::vector<Point> rest; for (size_t k = 0; k < idx.size(); ++k) if (idx[k] != i) rest.push_back(sup[k]);
        e.lift_margin[size_t(i)] = rest.empty() ? margin_floor_ : std::max(margin_floor_, margin_of(rest, cm));
    }
    // the gap plane over the bearing toes with remembered ground: how the body will rotate as the gaps close
    {
        double M[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}, b[3] = {0, 0, 0}; int n = 0;
        for (int i = 0; i < 4; ++i) {
            if (!e.contact[size_t(i)] || !e.has[size_t(i)]) continue;
            const double r[3] = {1.0, toe[size_t(i)].fwd, toe[size_t(i)].lat};
            for (int a = 0; a < 3; ++a) { b[a] += r[a] * e.gap[size_t(i)]; for (int c2 = 0; c2 < 3; ++c2) M[a * 3 + c2] += r[a] * r[c2]; }
            ++n;
        }
        double Mi[9];
        if (n >= 3 && inv3(M, Mi)) { e.slope_fwd = Mi[3] * b[0] + Mi[4] * b[1] + Mi[5] * b[2]; e.slope_lat = Mi[6] * b[0] + Mi[7] * b[1] + Mi[8] * b[2]; }
    }
    // the errors: the load prediction against the FSRs on toes in contact, the contact prediction against the flag
    for (int i = 0; i < 4; ++i) {
        load_err_valid_[size_t(i)] = false;
        if (have_contact_ && have_load_ && contact_[size_t(i)] > 0.5f) {
            const double err = e.load[size_t(i)] - double(load_[size_t(i)]);
            load_err_[size_t(i)] = err; load_err_valid_[size_t(i)] = true;
            load_ms_[size_t(i)] += err_alpha_ * (err * err - load_ms_[size_t(i)]);
            const double x = e.load[size_t(i)], y = double(load_[size_t(i)]);
            load_corr_sx_ += x; load_corr_sy_ += y; load_corr_sxx_ += x * x; load_corr_syy_ += y * y; load_corr_sxy_ += x * y; load_corr_n_ += 1.0;
        }
        if (have_contact_ && e.has[size_t(i)]) {
            ++agree_n_; if (e.gap_contact[size_t(i)] == (contact_[size_t(i)] > 0.5f)) ++agree_hits_;
            const double g = e.gap[size_t(i)];
            if (contact_[size_t(i)] > 0.5f) { ++gap_on_n_;  gap_on_mean_  += err_alpha_ * (g - gap_on_mean_);  gap_on_ms_  += err_alpha_ * (g * g - gap_on_ms_); }
            else                            { ++gap_off_n_; gap_off_mean_ += err_alpha_ * (g - gap_off_mean_); gap_off_ms_ += err_alpha_ * (g * g - gap_off_ms_); }
        }
    }
    if (have_contact_ && have_load_) {   // the shares: the FSR sum normalised away, on ticks when the toes carry the body
        double sum = 0.0; for (int i = 0; i < 4; ++i) sum += std::max(0.0, double(load_[size_t(i)]));
        if (sum >= share_min_) for (int i = 0; i < 4; ++i) {
            const double x = e.load[size_t(i)], y = std::max(0.0, double(load_[size_t(i)])) / sum;
            share_sx_ += x; share_sy_ += y; share_sxx_ += x * x; share_syy_ += y * y; share_sxy_ += x * y; share_n_ += 1.0;
        }
    }
    // the support over time, and the unsafe lifts: a toe whose flag falls this tick while, last tick, its leaving left too little
    unsup_frac_ += err_alpha_ * ((e.margin < 0.0 ? 1.0 : 0.0) - unsup_frac_);
    if (e.margin < 0.0) { ++unsup_streak_; unsup_streak_max_ = std::max(unsup_streak_max_, unsup_streak_); } else unsup_streak_ = 0;
    if (have_contact_) {
        if (have_prev_) for (int i = 0; i < 4; ++i)
            if (prev_contact_[size_t(i)] > 0.5f && contact_[size_t(i)] <= 0.5f) { ++lifts_[size_t(i)]; if (prev_lift_margin_[size_t(i)] < unsafe_margin_m_) ++unsafe_lifts_[size_t(i)]; }
        prev_contact_ = contact_; prev_lift_margin_ = e.lift_margin; have_prev_ = true;
    }
    eval_ = e; have_eval_ = true; ++evals_;
}

void SupportModel::tick(uint64_t tick_id) {
    if (!(have_xz_ && have_y_)) return;
    evaluate();
    auto const& e = eval_;
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("support_model") : std::string(id()); out->sensor = "support";
    out->values = Eigen::VectorXf(26);
    out->values[0] = float(e.margin); out->values[1] = float(e.n_support); out->values[2] = float(e.com_fwd); out->values[3] = float(e.com_lat);
    out->values[4] = float(e.slope_fwd); out->values[5] = float(e.slope_lat);
    for (int i = 0; i < 4; ++i) {
        out->values[6 + 5 * i] = float(e.gap[size_t(i)]); out->values[7 + 5 * i] = e.has[size_t(i)] ? 1.0f : 0.0f;
        out->values[8 + 5 * i] = e.contact[size_t(i)] ? 1.0f : 0.0f; out->values[9 + 5 * i] = float(e.load[size_t(i)]); out->values[10 + 5 * i] = float(e.lift_margin[size_t(i)]);
    }
    bus_->publish(output_topic_, out);
}

nlohmann::json SupportModel::diag_snapshot() const {
    auto const& e = eval_;
    nlohmann::json gap = nlohmann::json::array(), has = nlohmann::json::array(), con = nlohmann::json::array(), gcon = nlohmann::json::array(), load = nlohmann::json::array(),
                   lm = nlohmann::json::array(), le = nlohmann::json::array(), lr = nlohmann::json::array(), ul = nlohmann::json::array(), lf = nlohmann::json::array();
    for (int i = 0; i < 4; ++i) {
        gap.push_back(e.gap[size_t(i)]); has.push_back(e.has[size_t(i)] ? 1 : 0); con.push_back(e.contact[size_t(i)] ? 1 : 0); gcon.push_back(e.gap_contact[size_t(i)] ? 1 : 0); load.push_back(e.load[size_t(i)]);
        lm.push_back(e.lift_margin[size_t(i)]); le.push_back(load_err_valid_[size_t(i)] ? load_err_[size_t(i)] : 0.0); lr.push_back(std::sqrt(load_ms_[size_t(i)]));
        ul.push_back(unsafe_lifts_[size_t(i)]); lf.push_back(lifts_[size_t(i)]);
    }
    auto corr_of = [](double n, double sx, double sy, double sxx, double syy, double sxy) {
        if (n <= 2) return 0.0;
        const double cov = sxy / n - (sx / n) * (sy / n), vx = sxx / n - (sx / n) * (sx / n), vy = syy / n - (sy / n) * (sy / n);
        return (vx > 1e-12 && vy > 1e-12) ? cov / std::sqrt(vx * vy) : 0.0;
    };
    const double corr = corr_of(load_corr_n_, load_corr_sx_, load_corr_sy_, load_corr_sxx_, load_corr_syy_, load_corr_sxy_);
    const double share_corr = corr_of(share_n_, share_sx_, share_sy_, share_sxx_, share_syy_, share_sxy_);
    return nlohmann::json{{"margin", e.margin}, {"n_support", e.n_support}, {"com", {e.com_fwd, e.com_lat}}, {"slope", {e.slope_fwd, e.slope_lat}},
                          {"gap", gap}, {"has", has}, {"contact", con}, {"gap_contact", gcon}, {"load", load}, {"lift_margin", lm}, {"load_err", le}, {"load_rms", lr},
                          {"unsup_frac", unsup_frac_}, {"unsup_streak", unsup_streak_}, {"unsup_streak_max", unsup_streak_max_}, {"unsafe_lifts", ul}, {"lifts", lf},
                          {"load_corr", corr}, {"load_n", load_corr_n_}, {"share_corr", share_corr}, {"share_n", share_n_}, {"agree", contact_agreement()}, {"agree_n", agree_n_}, {"evals", evals_}, {"no_ground", no_ground_},
                          {"gap_on", {gap_on_mean_, std::sqrt(std::max(0.0, gap_on_ms_ - gap_on_mean_ * gap_on_mean_)), double(gap_on_n_)}},
                          {"gap_off", {gap_off_mean_, std::sqrt(std::max(0.0, gap_off_ms_ - gap_off_mean_ * gap_off_mean_)), double(gap_off_n_)}},
                          {"have_patch", have_patch_}, {"have_up", have_up_}};
}

} // namespace ogma
