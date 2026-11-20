// =============================================================================
// AheadFeatures.cpp  --  see the header (learned stepper plan 9.7 item 1, 2026-11-20)
// =============================================================================
#include "ogma/modules/AheadFeatures.hpp"

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
    throw std::invalid_argument("AheadFeatures: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("AheadFeatures: param '" + k + "' must be a string");
}
} // namespace

AheadFeatures::AheadFeatures()  = default;
AheadFeatures::~AheadFeatures() = default;

std::string_view AheadFeatures::type_name() const { return "AheadFeatures"; }

std::vector<TopicSpec> AheadFeatures::input_topics() const {
    const auto pt = std::type_index(typeid(ProprioToken));
    std::vector<TopicSpec> v{ TopicSpec{ahead_topic_, pt, SubscriptionKind::Direct, false} };
    if (!up_topic_.empty()) v.push_back(TopicSpec{up_topic_, pt, SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> AheadFeatures::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema AheadFeatures::params_schema() const {
    return {
        {"ahead_topic",  ParamMutability::ConstructionOnly, "TerrainMemory's ground-ahead grid (its ahead_topic).", ParamValue{std::string("terrain.ahead")}},
        {"up_topic",     ParamMutability::ConstructionOnly, "ProprioToken [x, y, z]: the body's up vector; nothing is published while y < up_min. \"\" = always publish.", ParamValue{std::string("reality.proprio.up_body")}},
        {"output_topic", ParamMutability::ConstructionOnly, "ProprioToken [(rows-1) x cols reliefs, cols unknown shares, the constant]: the ground-ahead EPM's input.", ParamValue{std::string("reality.proprio.ahead_features")}},
        {"ref_row0",     ParamMutability::ConstructionOnly, "true: the reliefs are against row 0 of the grid (TerrainMemory's grid). false: the grid is already against the front toes' ground (AheadField).", ParamValue{true}},
        {"under_rows",   ParamMutability::ConstructionOnly, "> 0: the ground under the body (the grid's astride value, AheadField only) is one more feature with the weight of this many rows; 0 = off.", ParamValue{int64_t{0}}},
        {"scale_m",      ParamMutability::HotMutable, "The length one unit of relief stands for (m): the body's standing height.", ParamValue{0.085}},
        {"clip",         ParamMutability::HotMutable, "Reliefs are clipped at +- this many units.", ParamValue{2.0}},
        {"bias_ratio",   ParamMutability::HotMutable, "The constant component is this x sqrt(cells): a relief of bias_ratio on every cell sits at 45 degrees to flat ground.", ParamValue{0.5}},
        {"up_min",       ParamMutability::HotMutable, "The up vector's y below which the body is not upright and the features are held (0.8 = 37 degrees).", ParamValue{0.8}},
    };
}

ParamMap AheadFeatures::current_params() const {
    return { {"ahead_topic", ParamValue{ahead_topic_}}, {"up_topic", ParamValue{up_topic_}}, {"output_topic", ParamValue{output_topic_}},
             {"ref_row0", ParamValue{ref_row0_}}, {"under_rows", ParamValue{int64_t(under_rows_)}}, {"scale_m", ParamValue{scale_m_}}, {"clip", ParamValue{clip_}}, {"bias_ratio", ParamValue{bias_ratio_}}, {"up_min", ParamValue{up_min_}} };
}

void AheadFeatures::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "scale_m")    scale_m_    = std::max(1e-4, get_double(value, k));
    else if (k == "clip")       clip_       = std::max(0.1, get_double(value, k));
    else if (k == "bias_ratio") bias_ratio_ = std::max(0.0, get_double(value, k));
    else if (k == "up_min")     up_min_     = std::clamp(get_double(value, k), -1.0, 1.0);
}

void AheadFeatures::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("AheadFeatures requires a non-null Bus");
    apply_param(params, "ahead_topic",  [&](auto const& v){ ahead_topic_  = get_string(v, "ahead_topic"); });
    apply_param(params, "up_topic",     [&](auto const& v){ up_topic_     = get_string(v, "up_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "ref_row0",     [&](auto const& v){ ref_row0_ = get_double(v, "ref_row0") != 0.0; });
    apply_param(params, "under_rows",   [&](auto const& v){ under_rows_ = std::clamp(int(get_double(v, "under_rows")), 0, 32); });
    if (under_rows_ > 0 && ref_row0_) throw std::invalid_argument("AheadFeatures: under_rows needs a grid that carries the astride value (AheadField, ref_row0 false)");
    for (auto const& k : {"scale_m", "clip", "bias_ratio", "up_min"}) apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    if (ahead_topic_.empty() || output_topic_.empty()) throw std::invalid_argument("AheadFeatures: ahead_topic and output_topic are required");
    sub_ids_.push_back(bus_->subscribe(ahead_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){
        if (!input_allowed(p->producer_id)) return;
        auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
        if (!pt || pt->values.size() < TerrainMemory::kAheadHeader) return;
        grid_.assign(pt->values.data(), pt->values.data() + pt->values.size()); fresh_ = true;
    }));
    if (!up_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(up_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){
            if (!input_allowed(p->producer_id)) return;
            auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
            if (!pt || pt->values.size() < 3) return;
            up_y_ = double(pt->values[1]); have_up_ = true;
        }));
}

void AheadFeatures::tick(uint64_t tick_id) {
    if (!fresh_) return;
    fresh_ = false;
    const int H = TerrainMemory::kAheadHeader;
    const int R = int(std::lround(grid_[0])), C = int(std::lround(grid_[1]));
    if (R < 2 || C < 1 || int(grid_.size()) < H + 2 * R * C) return;
    if (have_up_ && up_y_ < up_min_) { ++held_; return; }
    rows_ = R; cols_ = C;
    auto h_at = [&](int i, int j) { return double(grid_[size_t(H + i * C + j)]); };
    auto w_at = [&](int i, int j) { return double(grid_[size_t(H + R * C + i * C + j)]); };
    // the ground the front toes stand on: row 0, or the nearest row that remembers anything
    ref_row_ = -1; ref_ = 0.0;
    for (int i = 0; i < R && ref_row_ < 0; ++i) {
        double sw = 0.0, sh = 0.0;
        for (int j = 0; j < C; ++j) if (w_at(i, j) > 0.0) { sw += w_at(i, j); sh += w_at(i, j) * h_at(i, j); }
        if (sw > 0.0) { ref_ = sh / sw; ref_row_ = i; }
    }
    if (ref_row_ < 0) { ++no_ref_; return; }   // nothing ahead is remembered at all: no claim
    if (!ref_row0_) ref_ = 0.0;                // the grid is already against the front toes' ground
    const int n = (R - 1) * C;
    const int U = under_rows_ > 0 ? 1 : 0;
    feat_.assign(size_t(n + C + U + 1), 0.0f); relief_m_.assign(size_t(n), 0.0f); unknown_.assign(size_t(C), 0.0f);
    double ss = 0.0;
    for (int j = 0; j < C; ++j) {
        int unk = 0;
        for (int i = 1; i < R; ++i) {
            const int k = (i - 1) * C + j;
            if (w_at(i, j) <= 0.0) { ++unk; continue; }
            const double d = h_at(i, j) - ref_;
            relief_m_[size_t(k)] = float(d);
            feat_[size_t(k)] = float(std::clamp(d / scale_m_, -clip_, clip_));
            ss += double(feat_[size_t(k)]) * double(feat_[size_t(k)]);
        }
        unknown_[size_t(j)] = float(double(unk) / double(R - 1));
        feat_[size_t(n + j)] = unknown_[size_t(j)];
    }
    if (U) {   // the rear toes' ground against the front toes': the rows the body covers
        astride_m_ = double(grid_[5]);
        feat_[size_t(n + C)] = float(std::clamp(-astride_m_ / scale_m_, -clip_, clip_) * std::sqrt(double(under_rows_ * C)));
    }
    feat_[size_t(n + C + U)] = float(bias_ratio_ * std::sqrt(double(n)));
    norm_ = std::sqrt(ss);
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("ahead_features") : std::string(id()); out->sensor = "ahead_features";
    out->values = Eigen::Map<const Eigen::VectorXf>(feat_.data(), Eigen::Index(feat_.size()));
    bus_->publish(output_topic_, out);
    ++published_;
}

nlohmann::json AheadFeatures::diag_snapshot() const {
    nlohmann::json mm = nlohmann::json::array(), un = nlohmann::json::array();
    for (float v : relief_m_) mm.push_back(int(std::lround(double(v) * 1000.0)));
    for (float v : unknown_)  un.push_back(std::round(double(v) * 100.0) / 100.0);
    return nlohmann::json{{"published", published_}, {"held", held_}, {"no_ref", no_ref_}, {"ref", ref_}, {"ref_row", ref_row_},
                          {"norm", norm_}, {"rows", rows_}, {"cols", cols_}, {"astride", astride_m_}, {"relief_mm", mm}, {"unknown", un}};
}

} // namespace ogma
