// MotionField -- see the header.  The always-on motion loop's sensor (the ten-minutes phase, the chase push, 2026-10-02).
#include "ogma/modules/MotionField.hpp"

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
    throw std::invalid_argument("MotionField: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("MotionField: param '" + k + "' must be a string");
}
constexpr double kHz = 50.0;
}  // namespace

MotionField::MotionField()  = default;
MotionField::~MotionField() = default;

std::string_view MotionField::type_name() const { return "MotionField"; }

std::vector<TopicSpec> MotionField::input_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{input_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
    if (!cloud_mover_topic_.empty()) v.push_back(TopicSpec{cloud_mover_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> MotionField::output_topics() const {
    std::vector<TopicSpec> v;
    if (!output_topic_.empty()) v.push_back(TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema MotionField::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly, "The ToF cast token (the one CloudMap reads).", ParamValue{std::string("reality.proprio.tof_points")}},
        {"output_topic", ParamMutability::ConstructionOnly,
         "[vx +right, vy +forward, proximity, salience, track casts, the cast's tick (a sighting's stamp, as the cloud's mover token), world vx, world vy] for the most salient published "
         "motion track; proximity 0 = none.  Empty = computed but not published (passive).", ParamValue{std::string("")}},
        {"cloud_mover_topic", ParamMutability::ConstructionOnly,
         "The cloud's mover token (CloudMap mover_topic): with no published track of its own, its sighting is passed through on "
         "output_topic -- the union of the two detectors for the chase.  Empty = own tracks only.", ParamValue{std::string("")}},
        {"vouch_m", ParamMutability::HotMutable, "M7: the cloud's sighting passes only with motion evidence within this (m) in the last vouch_s, or continuing a vouched mover.  0 = pass every sighting.", ParamValue{0.0}},
        {"vouch_s", ParamMutability::HotMutable, "M7: ...the evidence's age limit (s).", ParamValue{0.6}},
        {"own_tracks", ParamMutability::HotMutable, "Publish the module's own tracks (true) or only the (vouched) cloud sightings (false).", ParamValue{true}},
        {"memory_s", ParamMutability::HotMutable, "How long the rays are remembered (s): the 'a moment ago' of the free-space test.", ParamValue{1.5}},
        {"near_m", ParamMutability::HotMutable, "A past ray within this of a new return (m) passed through its place.", ParamValue{0.025}},
        {"beyond_m", ParamMutability::HotMutable, "...and went on at least this far past it (m).", ParamValue{0.08}},
        {"min_height", ParamMutability::HotMutable, "Returns below this height (m) are the floor and judged not.", ParamValue{0.03}},
        {"max_range", ParamMutability::HotMutable, "Returns beyond this horizontal range (m) are judged not.", ParamValue{2.5}},
        {"cluster_m", ParamMutability::HotMutable, "Evidence points within this (m) of each other in one cast form a blob.", ParamValue{0.12}},
        {"min_points", ParamMutability::HotMutable, "A blob needs at least this many evidence points.", ParamValue{int64_t{2}}},
        {"gate_m", ParamMutability::HotMutable, "A blob within this (m) of a track's predicted position continues it.", ParamValue{0.25}},
        {"persist_casts", ParamMutability::HotMutable, "A track is published after this many consecutive casts.", ParamValue{int64_t{2}}},
        {"drop_s", ParamMutability::HotMutable, "A track unseen for this long (s) is dropped.", ParamValue{0.5}},
        {"proximity_range", ParamMutability::HotMutable, "proximity = 1 - range / this.", ParamValue{2.5}},
        {"bg_m", ParamMutability::HotMutable, "THE BACKGROUND: a return with a remembered return within this (m), bg_min_s to bg_s old, is the static world (an edge already hit), not motion.  0 = off.", ParamValue{0.0}},
        {"bg_min_s", ParamMutability::HotMutable, "...returns younger than this (s) are not background (a mover's own trail).", ParamValue{1.0}},
        {"bg_s", ParamMutability::HotMutable, "...returns older than this (s) are forgotten.", ParamValue{10.0}},
        {"min_travel_m", ParamMutability::HotMutable, "A track is published only once it has travelled this far (m) in the world since it began.  0 = off.", ParamValue{0.0}},
    };
}

ParamMap MotionField::current_params() const {
    ParamMap m;
    m["input_topic"] = ParamValue{input_topic_}; m["output_topic"] = ParamValue{output_topic_};
    m["cloud_mover_topic"] = ParamValue{cloud_mover_topic_};
    m["vouch_m"] = vouch_m_; m["vouch_s"] = vouch_s_; m["own_tracks"] = own_tracks_;
    m["memory_s"] = memory_s_; m["near_m"] = near_m_; m["beyond_m"] = beyond_m_; m["min_height"] = min_height_;
    m["max_range"] = max_range_; m["cluster_m"] = cluster_m_; m["min_points"] = int64_t(min_points_); m["gate_m"] = gate_m_;
    m["persist_casts"] = int64_t(persist_casts_); m["drop_s"] = drop_s_; m["proximity_range"] = proximity_range_;
    m["bg_m"] = bg_m_; m["bg_min_s"] = bg_min_s_; m["bg_s"] = bg_s_; m["min_travel_m"] = min_travel_m_;
    return m;
}

void MotionField::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("MotionField requires a non-null Bus");
    apply_param(params, "input_topic",  [&](auto const& v){ input_topic_ = get_string(v, "input_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "cloud_mover_topic", [&](auto const& v){ cloud_mover_topic_ = get_string(v, "cloud_mover_topic"); });
    for (const char* k : {"memory_s", "near_m", "beyond_m", "min_height", "max_range", "cluster_m", "min_points", "gate_m",
                          "persist_casts", "drop_s", "proximity_range", "bg_m", "bg_min_s", "bg_s", "min_travel_m",
                          "vouch_m", "vouch_s", "own_tracks"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
}

void MotionField::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "memory_s")        memory_s_ = get_double(value, k);
    else if (k == "near_m")          near_m_ = get_double(value, k);
    else if (k == "beyond_m")        beyond_m_ = get_double(value, k);
    else if (k == "min_height")      min_height_ = get_double(value, k);
    else if (k == "max_range")       max_range_ = get_double(value, k);
    else if (k == "cluster_m")       cluster_m_ = get_double(value, k);
    else if (k == "min_points")      min_points_ = std::max(1, int(get_double(value, k)));
    else if (k == "gate_m")          gate_m_ = get_double(value, k);
    else if (k == "persist_casts")   persist_casts_ = std::max(1, int(get_double(value, k)));
    else if (k == "drop_s")          drop_s_ = get_double(value, k);
    else if (k == "proximity_range") proximity_range_ = get_double(value, k);
    else if (k == "bg_m")            bg_m_ = get_double(value, k);
    else if (k == "bg_min_s")        bg_min_s_ = get_double(value, k);
    else if (k == "bg_s")            bg_s_ = get_double(value, k);
    else if (k == "min_travel_m")    min_travel_m_ = get_double(value, k);
    else if (k == "vouch_m")         vouch_m_ = get_double(value, k);
    else if (k == "vouch_s")         vouch_s_ = get_double(value, k);
    else if (k == "own_tracks")      { if (auto b = std::get_if<bool>(&value)) own_tracks_ = *b; else own_tracks_ = get_double(value, k) > 0.5; }
    else throw std::invalid_argument("MotionField: unknown param '" + k + "'");
}

void MotionField::tick(uint64_t tick_id) {
    cast_now_ = false;
    if (!bus_) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(input_topic_));
    if (pt && pt->values.size() >= 5 + 3 * kZones) {
        pyaw_ = double(pt->values[1]); px_ = double(pt->values[3]); py_ = double(pt->values[4]);
        // a new cast: the 64 points' x and y changed (the sensor updates every 4 ticks; the token repeats in between --
        // and its z carries the trunk's height of THIS tick, so z alone changes every tick: compare x and y only)
        std::vector<float> pts;
        pts.reserve(2 * kZones);
        for (int i = 0; i < kZones; ++i) { pts.push_back(pt->values[5 + 3 * i]); pts.push_back(pt->values[5 + 3 * i + 1]); }
        bool changed = pts.size() != prev_pts_.size();
        for (size_t i = 0; !changed && i < pts.size(); ++i) {
            const float a = pts[i], b = prev_pts_[i];
            if (!(a == b || (std::isnan(a) && std::isnan(b)))) changed = true;
        }
        if (changed) { prev_pts_ = std::move(pts); process_cast(pt->values, tick_id); cast_now_ = true; }
    }
    publish(tick_id);
}

void MotionField::process_cast(const Eigen::VectorXf& v, uint64_t tick_id) {
    ++casts_seen_;
    const double c = std::cos(pyaw_), s = std::sin(pyaw_);
    const auto world = [&](double bx, double by) { return std::array<double, 2>{px_ + c * bx - s * by, py_ + s * bx + c * by}; };
    const int nb = 5 + 3 * kZones, nf = nb + 3;
    const bool have_origin = v.size() >= nb + 3 && std::isfinite(v[nb]) && std::isfinite(v[nb + 1]) && std::isfinite(v[nb + 2]);
    if (!have_origin) return;
    const auto o = world(double(v[nb]), double(v[nb + 1]));
    const double oz = double(v[nb + 2]);

    // this cast's rays and its off-floor returns
    Cast cur; cur.tick = tick_id;
    std::vector<std::array<double, 3>> returns;
    for (int i = 0; i < kZones; ++i) {
        const int b = 5 + 3 * i, bf = nf + 3 * i;
        if (std::isfinite(v[b]) && std::isfinite(v[b + 1]) && std::isfinite(v[b + 2])) {
            const auto e = world(double(v[b]), double(v[b + 1]));
            const double ez = double(v[b + 2]);
            cur.rays.push_back(Ray{o[0], o[1], oz, e[0], e[1], ez});
            if (ez >= min_height_ && std::hypot(e[0] - o[0], e[1] - o[1]) <= max_range_) returns.push_back({e[0], e[1], ez});
        } else if (int(v.size()) >= bf + 3 && std::isfinite(v[bf]) && std::isfinite(v[bf + 1]) && std::isfinite(v[bf + 2])) {
            const auto e = world(double(v[bf]), double(v[bf + 1]));
            cur.rays.push_back(Ray{o[0], o[1], oz, e[0], e[1], double(v[bf + 2])});
        }
    }

    // the evidence: a return where a remembered ray passed and went on
    evidence_xyz_.clear(); why_.clear();
    const double near2 = near_m_ * near_m_;
    const double bg2 = bg_m_ * bg_m_;
    const uint64_t bg_young = uint64_t(bg_min_s_ * kHz);
    for (auto const& p : returns) {
        if (bg_m_ > 0.0) {                                                // an edge already hit: the static world
            bool known = false;
            for (auto const& q : bg_) {
                if (tick_id - q.tick < bg_young) break;                   // oldest first: the rest are younger
                const double dx = q.x - p[0], dy = q.y - p[1], dz = q.z - p[2];
                if (dx * dx + dy * dy + dz * dz < bg2) { known = true; break; }
            }
            if (known) continue;
        }
        bool ev = false;
        for (auto const& past : casts_) {
            for (auto const& r : past.rays) {
                const double ux = r.ex - r.ox, uy = r.ey - r.oy, uz = r.ez - r.oz;
                const double len = std::sqrt(ux * ux + uy * uy + uz * uz);
                if (len < 1e-6) continue;
                const double t = ((p[0] - r.ox) * ux + (p[1] - r.oy) * uy + (p[2] - r.oz) * uz) / len;
                if (t < 0.0 || t > len - beyond_m_) continue;
                const double qx = r.ox + ux * t / len - p[0], qy = r.oy + uy * t / len - p[1], qz = r.oz + uz * t / len - p[2];
                if (qx * qx + qy * qy + qz * qz < near2) {
                    ev = true; why_.push_back({r.ox, r.oy, r.oz, r.ex, r.ey, r.ez, double(tick_id - past.tick)}); break;
                }
            }
            if (ev) break;
        }
        if (ev) evidence_xyz_.push_back(p);
    }
    last_evidence_ = int(evidence_xyz_.size());
    if (vouch_m_ > 0.0) {
        for (auto const& e : evidence_xyz_) recent_ev_.push_back({e[0], e[1], e[2], double(tick_id)});
        while (!recent_ev_.empty() && double(tick_id) - recent_ev_.front()[3] > vouch_s_ * kHz) recent_ev_.pop_front();
    }

    // blobs: single linkage within cluster_m (horizontal)
    std::vector<int> lab(evidence_xyz_.size(), -1);
    int nlab = 0;
    for (size_t i = 0; i < evidence_xyz_.size(); ++i) {
        if (lab[i] >= 0) continue;
        std::vector<size_t> stack{i}; lab[i] = nlab;
        while (!stack.empty()) {
            const size_t a = stack.back(); stack.pop_back();
            for (size_t j = 0; j < evidence_xyz_.size(); ++j)
                if (lab[j] < 0 && std::hypot(evidence_xyz_[a][0] - evidence_xyz_[j][0], evidence_xyz_[a][1] - evidence_xyz_[j][1]) <= cluster_m_) {
                    lab[j] = nlab; stack.push_back(j);
                }
        }
        ++nlab;
    }
    struct Blob { double x = 0.0, y = 0.0; int n = 0; };
    std::vector<Blob> blobs(static_cast<size_t>(nlab));
    for (size_t i = 0; i < evidence_xyz_.size(); ++i) { auto& bl = blobs[size_t(lab[i])]; bl.x += evidence_xyz_[i][0]; bl.y += evidence_xyz_[i][1]; ++bl.n; }
    std::vector<Blob> kept;
    for (auto& bl : blobs) if (bl.n >= min_points_) { bl.x /= bl.n; bl.y /= bl.n; kept.push_back(bl); }
    last_blobs_ = int(kept.size());

    // tracks: each blob continues the nearest track whose prediction is within gate_m, else starts one
    std::vector<bool> matched(tracks_.size(), false);
    for (auto const& bl : kept) {
        int best = -1; double bd = gate_m_;
        for (size_t k = 0; k < tracks_.size(); ++k) {
            if (matched[k]) continue;
            const double dt = double(tick_id - tracks_[k].last) / kHz;
            const double d = std::hypot(bl.x - (tracks_[k].x + tracks_[k].vx * dt), bl.y - (tracks_[k].y + tracks_[k].vy * dt));
            if (d <= bd) { bd = d; best = int(k); }
        }
        if (best >= 0) {
            Track& tr = tracks_[size_t(best)];
            const double dt = std::max(1e-3, double(tick_id - tr.last) / kHz);
            const double ivx = (bl.x - tr.x) / dt, ivy = (bl.y - tr.y) / dt;
            const double a = tr.casts >= 2 ? 0.5 : 1.0;
            tr.vx = (1.0 - a) * tr.vx + a * ivx; tr.vy = (1.0 - a) * tr.vy + a * ivy;
            tr.x = bl.x; tr.y = bl.y; ++tr.casts; tr.points += bl.n; tr.misses = 0; tr.last = tick_id;
            tr.salience = 0.5 * tr.salience + double(bl.n);
            matched[size_t(best)] = true;
        } else {
            Track tr; tr.id = next_id_++; tr.x = tr.x0 = bl.x; tr.y = tr.y0 = bl.y; tr.casts = 1; tr.points = bl.n; tr.first = tr.last = tick_id;
            tr.salience = double(bl.n);
            tracks_.push_back(tr); matched.push_back(true);
        }
    }
    for (size_t k = 0; k < tracks_.size(); ++k) if (!matched[k]) { ++tracks_[k].misses; tracks_[k].salience *= 0.5; }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& t) {
        return double(tick_id - t.last) / kHz > drop_s_; }), tracks_.end());

    // remember this cast's returns as the background-to-be; forget the old
    if (bg_m_ > 0.0) {
        for (auto const& p : returns) bg_.push_back(BgPt{p[0], p[1], p[2], tick_id});
        while (!bg_.empty() && double(tick_id - bg_.front().tick) / kHz > bg_s_) bg_.pop_front();
    }
    // remember this cast; forget the old
    casts_.push_back(std::move(cur));
    while (!casts_.empty() && double(tick_id - casts_.front().tick) / kHz > memory_s_) casts_.pop_front();
}

void MotionField::publish(uint64_t tick_id) {
    published_ = -1;
    double best = 0.0;
    for (size_t k = 0; k < tracks_.size(); ++k) {
        const Track& t = tracks_[k];
        if (t.casts < persist_casts_) continue;
        if (min_travel_m_ > 0.0 && std::hypot(t.x - t.x0, t.y - t.y0) < min_travel_m_) continue;   // edges do not travel
        if (t.salience > best) { best = t.salience; published_ = int(k); }
    }
    if (published_ >= 0) ++published_ticks_;
    if (output_topic_.empty()) return;
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("motion") : id_; out->sensor = "motion";
    out->values = Eigen::VectorXf::Zero(8);
    if (published_ >= 0 && own_tracks_) {
        const Track& t = tracks_[size_t(published_)];
        const double dt = double(tick_id - t.last) / kHz;
        const double dx = t.x + t.vx * dt - px_, dy = t.y + t.vy * dt - py_;
        const double c = std::cos(pyaw_), s = std::sin(pyaw_);
        const double fwd = c * dx + s * dy, left = -s * dx + c * dy, rng = std::hypot(fwd, left);
        if (rng > 1e-6) {
            out->values[0] = float(-left / rng); out->values[1] = float(fwd / rng);
            out->values[2] = float(std::clamp(1.0 - rng / proximity_range_, 0.0, 1.0));
            // the cloud's mover token's layout where the seek loop reads it: [vx, vy, prox, ., ., the sighting's stamp]
            out->values[3] = float(t.salience); out->values[4] = float(t.casts); out->values[5] = float(t.last);
            out->values[6] = float(t.vx); out->values[7] = float(t.vy);
        }
    } else if (!cloud_mover_topic_.empty()) {
        // no track of its own: the cloud's sighting this tick, passed through (the union)
        if (auto cm = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(cloud_mover_topic_)))
            if (cm->tick_id == tick_id && cm->values.size() >= 3 && cm->values[2] > 0.0f) {
                bool pass = true;
                if (vouch_m_ > 0.0) {
                    // the sighting in the odometry frame: the body's pose plus the bearing at the range the proximity encodes
                    const double n = std::hypot(double(cm->values[0]), double(cm->values[1]));
                    const double rng = std::max(0.0, 1.0 - double(cm->values[2])) * proximity_range_;
                    const double fwd = n > 1e-6 ? double(cm->values[1]) / n : 1.0, left = n > 1e-6 ? -double(cm->values[0]) / n : 0.0;
                    const double c = std::cos(pyaw_), s = std::sin(pyaw_);
                    const double sx = px_ + c * fwd * rng - s * left * rng, sy = py_ + s * fwd * rng + c * left * rng;
                    bool ev = false;
                    for (auto const& e : recent_ev_) if (std::hypot(e[0] - sx, e[1] - sy) <= vouch_m_) { ev = true; break; }
                    const bool cont = vouched_ && double(tick_id - v_tick_) <= drop_s_ * kHz && std::hypot(sx - vx_last_, sy - vy_last_) <= gate_m_;
                    pass = ev || cont;
                    if (pass) { vouched_ = true; vx_last_ = sx; vy_last_ = sy; v_tick_ = tick_id; ++vouched_n_; } else ++refused_n_;
                }
                if (pass) {
                    for (int i = 0; i < std::min<int>(6, int(cm->values.size())); ++i) out->values[i] = cm->values[i];
                    ++passed_through_;
                }
            }
    }
    bus_->publish(output_topic_, out);
}

nlohmann::json MotionField::diag_lite() const {
    return {{"evidence", last_evidence_}, {"blobs", last_blobs_}, {"tracks", int(tracks_.size())}, {"published", published_ >= 0},
            {"casts", casts_seen_}, {"published_ticks", published_ticks_}, {"passed_through", passed_through_}, {"vouched", vouched_n_}, {"refused", refused_n_}};
}

nlohmann::json MotionField::diag_snapshot() const {
    nlohmann::json j = diag_lite();
    nlohmann::json tr = nlohmann::json::array();
    for (auto const& t : tracks_)
        tr.push_back({{"id", t.id}, {"x", t.x}, {"y", t.y}, {"vx", t.vx}, {"vy", t.vy}, {"casts", t.casts}, {"points", t.points},
                      {"salience", t.salience}});
    j["track_list"] = tr;
    nlohmann::json ev = nlohmann::json::array();
    for (auto const& p : evidence_xyz_) ev.push_back({p[0], p[1], p[2]});
    j["evidence_xyz"] = ev;
    j["pose"] = {px_, py_, pyaw_};
    j["published_index"] = published_;
    return j;
}

}  // namespace ogma
