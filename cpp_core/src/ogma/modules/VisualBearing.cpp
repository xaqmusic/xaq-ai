// =============================================================================
// VisualBearing.cpp  --  reward-free VISION food-direction perception
// =============================================================================
#include "ogma/modules/VisualBearing.hpp"

#include "ogma/Topics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("VisualBearing: param '" + key + "' must be integer");
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("VisualBearing: param '" + key + "' must be numeric");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("VisualBearing: param '" + key + "' must be a string");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("VisualBearing: param '" + key + "' must be bool");
}

} // namespace

VisualBearing::VisualBearing()  = default;
VisualBearing::~VisualBearing() = default;

std::string_view VisualBearing::type_name() const { return "VisualBearing"; }

std::vector<TopicSpec> VisualBearing::input_topics() const {
    return {
        TopicSpec{input_topic_, std::type_index(typeid(RawImageFrame)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> VisualBearing::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))},
    };
}

ParamSchema VisualBearing::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly,
            "Raw first-person colour frame (RawImageFrame, HxWx3 uint8). The body's "
            "FPV raycast output.", ParamValue{std::string("host.video.color")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "2-D egocentric food bearing ProprioToken [x=+right, y=+forward]; magnitude "
            "0 = occluded/no food in view, ~1 = food visible (same convention as "
            "percept.scent_compass).", ParamValue{std::string("percept.visual_bearing")}},
        {"green_r_max", ParamMutability::ConstructionOnly,
            "Food-pixel test: R strictly below this (matches body green_fraction).",
            ParamValue{int64_t{128}}},
        {"green_b_max", ParamMutability::ConstructionOnly,
            "Food-pixel test: B strictly below this.", ParamValue{int64_t{128}}},
        {"green_g_min", ParamMutability::ConstructionOnly,
            "Food-pixel test: G strictly above this.", ParamValue{int64_t{128}}},
        {"fov_deg", ParamMutability::ConstructionOnly,
            "FPV camera field of view (degrees). MUST match the scene Camera3D.fov "
            "(the_cell.tscn = 80) so the column centroid maps to the body's ray "
            "geometry. Sign is fov-independent; only the lateral angle scaling depends "
            "on it (and the output is unit-normalized).", ParamValue{80.0}},
        {"normalize_direction", ParamMutability::ConstructionOnly,
            "Emit a UNIT bearing when food is visible (so a downstream RBF-EPM clusters "
            "by ANGLE, not magnitude — the ScentCompass direction-blind lesson). "
            "Occluded still emits [0,0]. Default true.", ParamValue{true}},
        {"min_confidence", ParamMutability::ConstructionOnly,
            "Green-fraction floor: below this → [0,0] (occluded / too little food in "
            "view). The clean no-signal state.", ParamValue{0.0008}},
        {"emit_proximity", ParamMutability::ConstructionOnly,
            "Append values[2] = proximity_gain·green_fraction (graded 'how much food in "
            "view'). Default false = 2-D output.", ParamValue{false}},
        {"proximity_gain", ParamMutability::ConstructionOnly,
            "Scale on the appended proximity scalar. Only used when emit_proximity.",
            ParamValue{1.0}},
        {"centroid_ema_alpha", ParamMutability::ConstructionOnly,
            "EMA smoothing on the column centroid (anti-jitter for small/distant food "
            "blobs). 0 = off (raw centroid each tick).", ParamValue{0.0}},
        {"lesion_until_ticks", ParamMutability::HotMutable,
            "≥0 → the lesion ENDS at this tick (vision restored): a dropout WINDOW [after,until) for the (d) "
            "perturbation→degradation→RECOVERY test. <0 = permanent onset (prior behaviour).", ParamValue{int64_t{-1}}},
        {"lesion_after_ticks", ParamMutability::HotMutable,
            "≥0 → from this many ticks on, emit [0,0] (DROPOUT: knock vision out mid-run "
            "for the perturbation→recovery demo). <0 = never.", ParamValue{int64_t{-1}}},
        {"force_lesion", ParamMutability::HotMutable,
            "Immediate lesion — emit [0,0] every tick (the UI 'knock out vision' toggle).",
            ParamValue{false}},
        {"learn_appearance", ParamMutability::ConstructionOnly,
            "NON-SCAFFOLDED: learn food's colour from eat events instead of the hard-coded "
            "green test. false = green scaffold.", ParamValue{false}},
        {"hit_topic", ParamMutability::ConstructionOnly,
            "Teacher signal (EnvEvent) — on a hit, the central FPV is learned as food.",
            ParamValue{std::string("events.hit")}},
        {"appearance_alpha", ParamMutability::HotMutable,
            "EMA rate blending the central colour at eat into the food prototype.", ParamValue{0.3}},
        {"color_match_dist", ParamMutability::HotMutable,
            "RGB distance (0..255) within which a pixel matches the learned food prototype.", ParamValue{60.0}},
        {"central_ema_rate", ParamMutability::HotMutable,
            "EMA rate on the central-region mean colour (the approach view).", ParamValue{0.2}},
        {"central_frac", ParamMutability::ConstructionOnly,
            "Central fraction of the frame sampled as the approach view (food colour source).", ParamValue{0.3}},
    };
}

ParamMap VisualBearing::current_params() const {
    ParamMap m;
    m["input_topic"]         = ParamValue{input_topic_};
    m["output_topic"]        = ParamValue{output_topic_};
    m["green_r_max"]         = ParamValue{int64_t(green_r_max_)};
    m["green_b_max"]         = ParamValue{int64_t(green_b_max_)};
    m["green_g_min"]         = ParamValue{int64_t(green_g_min_)};
    m["fov_deg"]             = ParamValue{double(fov_deg_)};
    m["normalize_direction"] = ParamValue{normalize_direction_};
    m["min_confidence"]      = ParamValue{double(min_confidence_)};
    m["emit_proximity"]      = ParamValue{emit_proximity_};
    m["proximity_gain"]      = ParamValue{double(proximity_gain_)};
    m["centroid_ema_alpha"]  = ParamValue{double(centroid_ema_alpha_)};
    m["lesion_after_ticks"]  = ParamValue{int64_t(lesion_after_ticks_)};
    m["lesion_until_ticks"]  = ParamValue{int64_t(lesion_until_ticks_)};
    m["force_lesion"]        = ParamValue{force_lesion_};
    m["learn_appearance"]    = ParamValue{learn_appearance_};
    m["hit_topic"]           = ParamValue{hit_topic_};
    m["appearance_alpha"]    = ParamValue{double(appearance_alpha_)};
    m["color_match_dist"]    = ParamValue{double(color_match_dist_)};
    m["central_ema_rate"]    = ParamValue{double(central_ema_rate_)};
    m["central_frac"]        = ParamValue{double(central_frac_)};
    return m;
}

void VisualBearing::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("VisualBearing requires a non-null Bus");

    apply_param(params, "input_topic",  [&](auto const& v){ input_topic_  = get_string(v, "input_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "green_r_max",  [&](auto const& v){ green_r_max_  = int(get_int(v, "green_r_max")); });
    apply_param(params, "green_b_max",  [&](auto const& v){ green_b_max_  = int(get_int(v, "green_b_max")); });
    apply_param(params, "green_g_min",  [&](auto const& v){ green_g_min_  = int(get_int(v, "green_g_min")); });
    apply_param(params, "fov_deg",      [&](auto const& v){ fov_deg_      = float(get_double(v, "fov_deg")); });
    apply_param(params, "normalize_direction", [&](auto const& v){ normalize_direction_ = get_bool(v, "normalize_direction"); });
    apply_param(params, "min_confidence",      [&](auto const& v){ min_confidence_      = float(get_double(v, "min_confidence")); });
    apply_param(params, "emit_proximity",      [&](auto const& v){ emit_proximity_      = get_bool(v, "emit_proximity"); });
    apply_param(params, "proximity_gain",      [&](auto const& v){ proximity_gain_      = float(get_double(v, "proximity_gain")); });
    apply_param(params, "centroid_ema_alpha",  [&](auto const& v){ centroid_ema_alpha_  = float(get_double(v, "centroid_ema_alpha")); });
    apply_param(params, "lesion_after_ticks",  [&](auto const& v){ lesion_after_ticks_  = int(get_int(v, "lesion_after_ticks")); });
    apply_param(params, "lesion_until_ticks",  [&](auto const& v){ lesion_until_ticks_  = int(get_int(v, "lesion_until_ticks")); });
    apply_param(params, "force_lesion",        [&](auto const& v){ force_lesion_        = get_bool(v, "force_lesion"); });
    apply_param(params, "learn_appearance",    [&](auto const& v){ learn_appearance_    = get_bool(v, "learn_appearance"); });
    apply_param(params, "hit_topic",           [&](auto const& v){ hit_topic_           = get_string(v, "hit_topic"); });
    apply_param(params, "appearance_alpha",    [&](auto const& v){ appearance_alpha_    = float(get_double(v, "appearance_alpha")); });
    apply_param(params, "color_match_dist",    [&](auto const& v){ color_match_dist_    = float(get_double(v, "color_match_dist")); });
    apply_param(params, "central_ema_rate",    [&](auto const& v){ central_ema_rate_    = float(get_double(v, "central_ema_rate")); });
    apply_param(params, "central_frac",        [&](auto const& v){ central_frac_        = float(get_double(v, "central_frac")); });

    if (!input_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_video(p); }));
    }
    if (learn_appearance_ && !hit_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(hit_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_hit(p); }));
    }
}

void VisualBearing::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "lesion_after_ticks") lesion_after_ticks_ = int(get_int(value, k));
    else if (k == "lesion_until_ticks") lesion_until_ticks_ = int(get_int(value, k));
    else if (k == "force_lesion")       force_lesion_       = get_bool(value, k);
    else if (k == "appearance_alpha")   appearance_alpha_   = float(get_double(value, k));
    else if (k == "color_match_dist")   color_match_dist_   = float(get_double(value, k));
    else if (k == "central_ema_rate")   central_ema_rate_   = float(get_double(value, k));
    else throw std::invalid_argument("VisualBearing: param '" + k + "' is construction-only / unknown");
}

void VisualBearing::handle_hit(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (e && (e->name == "hit" || e->name.empty())) hit_pending_ = true;
}

void VisualBearing::handle_video(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto img = std::dynamic_pointer_cast<const RawImageFrame>(payload);
    if (!img) return;
    pixels_   = img->pixels;
    height_   = img->height;
    width_    = img->width;
    channels_ = img->channels;
}

void VisualBearing::tick(uint64_t tick_id) {
    // Dropout: force_lesion (live UI toggle) or lesion_after_ticks (headless,
    // reproducible onset) → emit the occluded/no-signal bearing [0,0].
    bool timed = lesion_after_ticks_ >= 0 && int64_t(tick_count_) >= int64_t(lesion_after_ticks_)
                 && (lesion_until_ticks_ < 0 || int64_t(tick_count_) < int64_t(lesion_until_ticks_));  // window [after,until)
    lesioned_ = force_lesion_ || timed;
    ++tick_count_;

    vx_ = 0.0f; vy_ = 0.0f; mag_ = 0.0f; green_frac_ = 0.0f;

    bool frame_ok = channels_ >= 3 && width_ > 0 && height_ > 0 &&
                    int(pixels_.size()) >= width_ * height_ * channels_;

    // LEARNED-APPEARANCE: track the central (approach) colour each tick; on a hit,
    // blend it into the food prototype. The bug learns what food looks like by eating.
    if (learn_appearance_ && frame_ok) {
        int c0 = int(float(width_)  * (1.0f - central_frac_) * 0.5f), c1 = width_  - c0;
        int r0 = int(float(height_) * (1.0f - central_frac_) * 0.5f), r1 = height_ - r0;
        double cr = 0, cg = 0, cb = 0; long cn = 0;
        for (int j = r0; j < r1; ++j)
            for (int i = c0; i < c1; ++i) {
                int idx = (j * width_ + i) * channels_;
                cr += pixels_[idx]; cg += pixels_[idx + 1]; cb += pixels_[idx + 2]; ++cn;
            }
        if (cn > 0) {
            float mr = float(cr / cn), mg = float(cg / cn), mb = float(cb / cn);
            if (!have_central_) { central_ema_[0] = mr; central_ema_[1] = mg; central_ema_[2] = mb; have_central_ = true; }
            else { central_ema_[0] += central_ema_rate_ * (mr - central_ema_[0]);
                   central_ema_[1] += central_ema_rate_ * (mg - central_ema_[1]);
                   central_ema_[2] += central_ema_rate_ * (mb - central_ema_[2]); }
        }
    }
    if (learn_appearance_ && hit_pending_ && have_central_) {
        if (!have_proto_) { for (int c = 0; c < 3; ++c) food_proto_[c] = central_ema_[c]; have_proto_ = true; }
        else { for (int c = 0; c < 3; ++c) food_proto_[c] += appearance_alpha_ * (central_ema_[c] - food_proto_[c]); }
    }
    hit_pending_ = false;

    if (!lesioned_ && frame_ok) {
        int   n_pix = width_ * height_;
        int   green_count = 0;
        float sum_u = 0.0f;
        const float inv_w = 1.0f / float(width_);
        for (int j = 0; j < height_; ++j) {
            for (int i = 0; i < width_; ++i) {
                int idx = (j * width_ + i) * channels_;
                int r = pixels_[idx];
                int g = pixels_[idx + 1];
                int b = pixels_[idx + 2];
                bool is_food;
                if (learn_appearance_) {
                    if (!have_proto_) { is_food = false; }
                    else {
                        float dr = r - food_proto_[0], dg = g - food_proto_[1], db = b - food_proto_[2];
                        is_food = (dr * dr + dg * dg + db * db) < color_match_dist_ * color_match_dist_;
                    }
                } else {
                    is_food = (r < green_r_max_ && b < green_b_max_ && g > green_g_min_);
                }
                if (is_food) {
                    // Column centroid in [-1,+1], +u = right (matches the raycast).
                    float u = -1.0f + 2.0f * (float(i) + 0.5f) * inv_w;
                    sum_u += u;
                    ++green_count;
                }
            }
        }
        green_frac_ = float(green_count) / float(n_pix);

        if (green_count > 0 && green_frac_ > min_confidence_) {
            float u_centroid = sum_u / float(green_count);
            if (centroid_ema_alpha_ > 0.0f) {
                if (!have_ema_) { ema_u_ = u_centroid; have_ema_ = true; }
                else            { ema_u_ += centroid_ema_alpha_ * (u_centroid - ema_u_); }
                u_centroid = ema_u_;
            }
            // Invert the raycast geometry: a ray at column u points along
            // forward + right·(u·tanH); so the egocentric bearing to the food
            // blob is [u·tanH, 1] (right, forward).
            const float kPi = 3.14159265358979323846f;
            float tan_half = std::tan(fov_deg_ * (kPi / 180.0f) * 0.5f);
            float offset_x = u_centroid * tan_half;
            if (normalize_direction_) {
                float inv = 1.0f / std::sqrt(offset_x * offset_x + 1.0f);
                vx_ = offset_x * inv;   // +right (= sin of the horizontal angle)
                vy_ = inv;              // +forward (= cos), always > 0 (food is ahead)
            } else {
                vx_ = offset_x;
                vy_ = 1.0f;
            }
            mag_ = std::sqrt(vx_ * vx_ + vy_ * vy_);   // ~1 when normalized + visible
        }
    }

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("visual_bearing") : id_;
    out->sensor      = "visual_bearing";
    out->values.resize(emit_proximity_ ? 3 : 2);
    out->values[0] = vx_;
    out->values[1] = vy_;
    if (emit_proximity_) out->values[2] = proximity_gain_ * green_frac_;
    bus_->publish(output_topic_, out);
}

nlohmann::json VisualBearing::snapshot_state() const {
    return nlohmann::json{{"version", 1}};   // stateless beyond the centroid EMA
}

nlohmann::json VisualBearing::diag_snapshot() const {
    return nlohmann::json{
        {"vx", vx_},
        {"vy", vy_},
        {"mag", mag_},
        {"green_frac", green_frac_},
        {"lesioned", lesioned_},
    };
}

void VisualBearing::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("VisualBearing::restore_state: unknown version " +
                                 std::to_string(version));
    }
}

} // namespace ogma
