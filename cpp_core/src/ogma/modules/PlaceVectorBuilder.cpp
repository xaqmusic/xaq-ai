// =============================================================================
// PlaceVectorBuilder.cpp  --  see the header (Cell round 2, lever A5)
// =============================================================================
#include "ogma/modules/PlaceVectorBuilder.hpp"

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
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("PlaceVectorBuilder: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("PlaceVectorBuilder: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PlaceVectorBuilder: param '" + k + "' must be a string");
}
} // namespace

PlaceVectorBuilder::PlaceVectorBuilder()  = default;
PlaceVectorBuilder::~PlaceVectorBuilder() = default;

std::string_view PlaceVectorBuilder::type_name() const { return "PlaceVectorBuilder"; }

std::vector<TopicSpec> PlaceVectorBuilder::input_topics() const {
    return {
        TopicSpec{pano_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{vel_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> PlaceVectorBuilder::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema PlaceVectorBuilder::params_schema() const {
    return {
        {"pano_topic", ParamMutability::ConstructionOnly,
            "CylinderBuilder's held panorama ProprioToken [n_pano], 0..1 (the appearance half).",
            ParamValue{std::string("percept.cylinder")}},
        {"heading_topic", ParamMutability::ConstructionOnly,
            "Absolute heading ProprioToken (rad). Integrated own-yaw; its drift-free perfection is the named scaffold (audit S1).",
            ParamValue{std::string("reality.proprio.heading")}},
        {"vel_topic", ParamMutability::ConstructionOnly,
            "Egocentric velocity ProprioToken [lateral, forward] (÷move_speed) → the path integral, exactly as PlayLoop/PlaceGraphPlanner keep it.",
            ParamValue{std::string("reality.proprio.vel_ego")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Place vector ProprioToken [n_pano + 4·pose_repeat]: panorama, then (x/odo_scale, y/odo_scale, cos h, sin h) repeated → the place-EPM (proprio_state_dims = out_dims).",
            ParamValue{std::string("percept.place_vec")}},
        {"n_pano", ParamMutability::ConstructionOnly,
            "Panorama length (CylinderBuilder n_bins × 3). pose_repeat = max(1, n_pano/4) so the two groups weigh equally in the encoder's L2 (derived, not tuned).",
            ParamValue{int64_t{24}}},
        {"odo_scale", ParamMutability::ConstructionOnly,
            "Odometry divisor. The integral's unit is tick·speed⁻¹ (≈ 20 per metre at move_speed 3); 240 ≈ 12 m maps a 24 m room onto [-1, 1] (clamped).",
            ParamValue{240.0}},
    };
}

ParamMap PlaceVectorBuilder::current_params() const {
    ParamMap m;
    m["pano_topic"]    = ParamValue{pano_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["vel_topic"]     = ParamValue{vel_topic_};
    m["output_topic"]  = ParamValue{output_topic_};
    m["n_pano"]        = ParamValue{int64_t(n_pano_)};
    m["odo_scale"]     = ParamValue{odo_scale_};
    return m;
}

void PlaceVectorBuilder::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("PlaceVectorBuilder requires a non-null Bus");
    apply_param(params, "pano_topic",    [&](auto const& v){ pano_topic_    = get_string(v,"pano_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v,"heading_topic"); });
    apply_param(params, "vel_topic",     [&](auto const& v){ vel_topic_     = get_string(v,"vel_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "n_pano",        [&](auto const& v){ n_pano_        = int(get_int(v,"n_pano")); });
    apply_param(params, "odo_scale",     [&](auto const& v){ odo_scale_     = get_double(v,"odo_scale"); });
    if (n_pano_ < 0) n_pano_ = 0;
    if (odo_scale_ <= 0.0) throw std::invalid_argument("PlaceVectorBuilder: odo_scale must be > 0");
    pose_repeat_ = std::max(1, n_pano_ / 4);
    pano_.assign(size_t(n_pano_), 0.0f);
    last_.assign(size_t(out_dims()), 0.0f);

    if (!pano_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(pano_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_pano(p); }));
    if (!heading_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_heading(p); }));
    if (!vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_vel(p); }));
}

void PlaceVectorBuilder::on_param_change(std::string_view key, ParamValue const&) {
    throw std::invalid_argument("PlaceVectorBuilder: param '" + std::string(key) + "' is construction-only / unknown");
}

void PlaceVectorBuilder::handle_pano(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    const int n = std::min(n_pano_, int(pt->values.size()));
    for (int i = 0; i < n; ++i) pano_[size_t(i)] = float(pt->values[i]);
    have_pano_ = n > 0;
}
void PlaceVectorBuilder::handle_heading(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (pt && pt->values.size() > 0) heading_ = float(pt->values[0]);
}
void PlaceVectorBuilder::handle_vel(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    if (pt->values.size() > 0) vlat_ = float(pt->values[0]);
    if (pt->values.size() > 1) vfwd_ = float(pt->values[1]);
}

void PlaceVectorBuilder::tick(uint64_t tick_id) {
    // The same path integral PlayLoop / PlaceGraphPlanner keep (body convention:
    // rotation.y = heading, forward = (-sin h, -cos h), right = (cos h, -sin h)).
    odo_x_ += -double(vfwd_) * std::sin(heading_) + double(vlat_) * std::cos(heading_);
    odo_y_ += -double(vfwd_) * std::cos(heading_) - double(vlat_) * std::sin(heading_);
    vlat_ = vfwd_ = 0.0f;   // a velocity sample integrates once

    const float px = float(std::clamp(odo_x_ / odo_scale_, -1.0, 1.0));
    const float py = float(std::clamp(odo_y_ / odo_scale_, -1.0, 1.0));
    const float ch = std::cos(heading_), sh = std::sin(heading_);

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("place_vec") : id_;
    out->sensor      = "place_vec";
    out->values = Eigen::VectorXf::Zero(out_dims());
    int k = 0;
    for (int i = 0; i < n_pano_; ++i) out->values[k++] = pano_[size_t(i)];
    for (int r = 0; r < pose_repeat_; ++r) {
        out->values[k++] = px; out->values[k++] = py; out->values[k++] = ch; out->values[k++] = sh;
    }
    for (int i = 0; i < out_dims(); ++i) last_[size_t(i)] = out->values[i];
    bus_->publish(output_topic_, out);
}

nlohmann::json PlaceVectorBuilder::snapshot_state() const {
    return nlohmann::json{
        {"odo_x", odo_x_}, {"odo_y", odo_y_}, {"heading", heading_},
        {"vlat", vlat_}, {"vfwd", vfwd_}, {"pano", pano_}, {"have_pano", have_pano_},
    };
}
void PlaceVectorBuilder::restore_state(nlohmann::json const& s) {
    odo_x_ = s.value("odo_x", 0.0); odo_y_ = s.value("odo_y", 0.0);
    heading_ = s.value("heading", 0.0f);
    vlat_ = s.value("vlat", 0.0f); vfwd_ = s.value("vfwd", 0.0f);
    if (s.contains("pano")) { pano_ = s["pano"].get<std::vector<float>>(); pano_.resize(size_t(n_pano_), 0.0f); }
    have_pano_ = s.value("have_pano", false);
}
nlohmann::json PlaceVectorBuilder::diag_snapshot() const {
    return nlohmann::json{
        {"odo_x", odo_x_}, {"odo_y", odo_y_}, {"heading", heading_}, {"have_pano", have_pano_},
        {"out_dims", out_dims()}, {"pose_repeat", pose_repeat_},
    };
}

} // namespace ogma
