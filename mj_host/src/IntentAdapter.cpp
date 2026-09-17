#include "IntentAdapter.hpp"

#include <algorithm>
#include <cmath>

#include <nlohmann/json.hpp>
#include <variant>

#include "ogma/GraphConfig.hpp"
#include "ogma/modules/CloudMap.hpp"
#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Rng.hpp"
#include "ogma/Topics.hpp"

namespace mjhost {

IntentAdapter::IntentAdapter(const std::string& graph_path, uint64_t seed) {
    auto cfg = ogma::GraphConfig::load_from_file(graph_path);
    if (seed != 0) {
        for (auto& m : cfg.modules)
            for (const char* pname : {"master_seed", "seed"}) {
                auto it = m.params.find(pname);
                if (it != m.params.end())
                    it->second = ogma::ParamValue{int64_t(ogma::namespace_seed(seed, m.id))};
            }
    }
    // The place vector's form follows the graph (see the header): the map EPM's declared width
    // on reality.proprio.place_in, and whether an EPM consumes reality.proprio.depth_in.
    bool have_map = false;
    for (const auto& m : cfg.modules) {
        auto it = m.params.find("input_topic");
        if (it == m.params.end()) continue;
        const auto* topic = std::get_if<std::string>(&it->second);
        if (!topic) continue;
        const auto get_int = [&](const char* k, int64_t dflt) {
            auto jt = m.params.find(k);
            if (jt == m.params.end()) return dflt;
            const auto* d = std::get_if<int64_t>(&jt->second);
            return d ? *d : dflt;
        };
        const auto get_str = [&](const char* k) {
            auto jt = m.params.find(k);
            if (jt == m.params.end()) return std::string();
            const auto* d = std::get_if<std::string>(&jt->second);
            return d ? *d : std::string();
        };
        if (*topic == "reality.proprio.place_in") {
            have_map = true;
            map_module_id_ = m.id;
            // The EPM does not report its live params (Module::current_params is empty for it), so the
            // map gate (set_map_learning) restores what the graph configured, else the EPM's own defaults.
            const auto get_dbl = [&](const char* k, double dflt) {
                auto jt = m.params.find(k);
                if (jt == m.params.end()) return dflt;
                if (const auto* d = std::get_if<double>(&jt->second)) return *d;
                if (const auto* i = std::get_if<int64_t>(&jt->second)) return double(*i);
                if (const auto* b = std::get_if<bool>(&jt->second)) return *b ? 1.0 : 0.0;
                return dflt;
            };
            map_saved_["min_insertion_error"] = get_dbl("min_insertion_error", 0.02);
            map_saved_["epsilon_b"]           = get_dbl("epsilon_b", 0.05);
            map_saved_["epsilon_n"]           = get_dbl("epsilon_n", 0.003);
            map_saved_["stale_prune_enabled"] = get_dbl("stale_prune_enabled", 1.0);
            place_dims_ = int(get_int("proprio_state_dims", 12));
        } else if (*topic == "reality.proprio.depth_in") {
            if (get_int("proprio_state_dims", 0) != 64)
                throw std::invalid_argument("IntentAdapter: the EPM on reality.proprio.depth_in must declare proprio_state_dims 64 (the ToF's 8x8)");
            depth_dims_  = int(get_int("projection_dim", 128));
            depth_topic_ = "reality." + get_str("modality_group") + "." + get_str("modality_name");
        }
    }
    if (depth_dims_ > 0) {
        place_form_ = PlaceForm::Stacked;
        if (!have_map || place_dims_ != 4 + depth_dims_)
            throw std::invalid_argument("IntentAdapter: with a depth EPM (projection_dim " + std::to_string(depth_dims_)
                                        + ") the map EPM on reality.proprio.place_in must declare proprio_state_dims "
                                        + std::to_string(4 + depth_dims_) + " (pose + depth latent); it declares " + std::to_string(place_dims_));
    } else if (place_dims_ == 12) {
        place_form_ = PlaceForm::Columns;
    } else if (place_dims_ == 13) {
        place_form_ = PlaceForm::ColumnsGaze;   // W3: pose, the head yaw, the 8 column ranges — a VIEW
    } else if (place_dims_ == 68) {
        place_form_ = PlaceForm::Zones;
    } else {
        throw std::invalid_argument("IntentAdapter: the map EPM on reality.proprio.place_in declares proprio_state_dims "
                                    + std::to_string(place_dims_) + "; the host builds 12 (pose + 8 column ranges), 13 (pose + head yaw + 8 column ranges), 68 (pose + 64 zone ranges) or 4 + a depth EPM's projection_dim");
    }
    instance_ = std::make_unique<ogma::OgmaInstance>(std::move(cfg), std::make_unique<ogma::InProcessBus>());
    inspector_ = std::make_unique<InspectorSurface>(*instance_, instance_mtx_, graph_path);
}

IntentAdapter::~IntentAdapter() = default;

std::array<double, 3> IntentAdapter::tick(const std::array<double, 3>& vel_body,
                                          const std::array<double, 3>& g,
                                          const std::array<double, 3>& w,
                                          const std::array<double, 3>& a,
                                          double odom_yaw, const std::array<float, 4>& tof,
                                          const PlaceInputs* place) {
    std::lock_guard<std::recursive_mutex> lk(instance_mtx_);
    auto* bus = instance_->bus();
    const auto publish = [&](const char* sensor, const std::vector<float>& values) {
        auto p = std::make_shared<ogma::ProprioToken>();
        p->tick_id = tick_id_;
        p->producer_id = "host";
        p->sensor = sensor;
        p->values.resize(int(values.size()));
        for (size_t i = 0; i < values.size(); ++i) p->values[int(i)] = values[i];
        bus->publish(std::string("reality.proprio.") + sensor, p);
    };
    const auto unit = [](double v) { return float(std::clamp(v, -1.0, 1.0)); };
    // The heading, unwrapped.  A wrapped angle is not one linear row — its slope flips
    // sign with where the body faces (R22 identified the sine row at 180° and held 180°,
    // tightly).  Accumulating the wrapped increments gives a continuous heading whose
    // row has one sign everywhere; beyond ±180° it saturates in the unit clamp.
    if (have_yaw_) {
        double d = odom_yaw - prev_yaw_;
        while (d > 3.14159265358979323846) d -= 2.0 * 3.14159265358979323846;
        while (d < -3.14159265358979323846) d += 2.0 * 3.14159265358979323846;
        heading_ += d;
    } else {
        heading_ref_ = 0.0;
    }
    prev_yaw_ = odom_yaw; have_yaw_ = true;
    // An unwrapped heading alone saturates: the body winds past a half turn during the
    // babble and the row identifies as zero.  So the sense is the deviation from a slow
    // running average of the heading (τ 3000 ticks = 60 s): bounded, linear, and a
    // memory that forgets over a minute — long enough to answer a shove, short enough
    // never to saturate.
    heading_ref_ += (1.0 / 3000.0) * (heading_ - heading_ref_);

    // The level-2 "joints": the body's velocity in the walker's own command units.
    last_sensed_ = {unit(vel_body[0] / kTwistRangeVx), unit(vel_body[1] / kTwistRangeVy),
                    unit(vel_body[2] / kTwistRangeVyaw)};
    publish("intent", {last_sensed_[0], last_sensed_[1], last_sensed_[2]});
    publish("imu", {float(g[0]), float(g[1]), float(g[2]), float(w[0]), float(w[1]), float(w[2])});
    // The 12-slot sense the bridge appends as load slots: attitude, rates, accel, the
    // sensed velocity again, two spare.
    std::array<float, 4> tof_s = tof;
    if (seek_gate_ && last_steer_ == 3 && seek_present_) {
        // the seek loop won the reference last tick: the slot of its target's sector reads free
        // (slots 12/13/14 = columns 0-2 / 3-4 / 5-7 = left / ahead / right; the sector by the bearing)
        const int slot = seek_ego_ < -0.3 ? 0 : (seek_ego_ > 0.3 ? 2 : 1);
        if (tof_s[size_t(slot)] > 0.0f) { tof_s[size_t(slot)] = 0.0f; ++seek_gated_; }
    }
    publish("sense", {float(g[0]), float(g[1]), unit(0.3 * w[1]), unit(0.3 * w[0]), unit(0.3 * w[2]),
                      unit(a[0] / 20.0), unit(a[1] / 20.0), unit(a[2] / 20.0),
                      last_sensed_[0], last_sensed_[1], unit((heading_ - heading_ref_) / 3.14159265358979323846),
                      unit(map_tle_),                    // slot 11: the map's surprise — novelty
                      tof_s[0], tof_s[1], tof_s[2], tof_s[3]});
    if (place) {
        std::vector<float> v(place->pose.begin(), place->pose.end());
        switch (place_form_) {
            case PlaceForm::Columns: v.insert(v.end(), place->cols.begin(), place->cols.end()); break;
            case PlaceForm::ColumnsGaze: v.push_back(place->head_yaw); v.insert(v.end(), place->cols.begin(), place->cols.end()); break;
            case PlaceForm::Zones:   v.insert(v.end(), place->zones.begin(), place->zones.end()); break;
            case PlaceForm::Stacked: {
                // The depth frame with its own mean taken out (CLAUDE.md §0 rule 2: the common mode --
                // here mostly the floor in the lower rows -- would otherwise dominate the projection);
                // the JL encoder normalises the scale.  Then the depth EPM's latent from its last tick.
                float mean = 0.0f;
                for (float z : place->zones) mean += z;
                mean /= float(place->zones.size());
                std::vector<float> d(place->zones.size());
                for (size_t i = 0; i < d.size(); ++i) d[i] = place->zones[i] - mean;
                publish("depth_in", d);
                v.resize(size_t(place_dims_), 0.0f);
                if (auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(bus->last_value(depth_topic_)))
                    if (rt->latent.size() == depth_dims_)
                        for (int i = 0; i < depth_dims_; ++i) v[size_t(4 + i)] = rt->latent[i];
                break;
            }
        }
        publish("place_in", v);
    }
    if (place && place->tof_points_valid)
        publish("tof_points", std::vector<float>(place->tof_points.begin(), place->tof_points.end()));
    // The Cell recipe's two egocentric inputs, for a loop that plans over the map: the unwrapped
    // heading and the body velocity as [lateral, forward] in command units.  Nothing in the
    // level-0..2 graphs reads them; a graph that does (R27's PlayLoop) is a new arm.
    publish("heading", {float(heading_)});
    // The dead-reckoned pose as [x, y, yaw] for a loop that remembers a POSITION (BearingSeekLoop, things
    // phase T2): the place pose carries x/2 and y/2; the yaw is the unwrapped heading (same frame).
    if (place) publish("odom", {float(2.0 * place->pose[0]), float(2.0 * place->pose[1]), float(heading_)});
    publish("vel_ego", {unit(vel_body[1] / kTwistRangeVy), unit(vel_body[0] / kTwistRangeVx)});
    publish("tof", {tof[0], tof[1], tof[2], tof[3]});   // the ToF summary on its own topic (an avoidance LOOP reads it)

    instance_->tick();
    inspector_->publish_tick(tick_id_);
    if (auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(bus->last_value("reality.proprio.place"))) {
        map_tle_ = rt->tle; map_novel_ = rt->is_novel; map_winner_ = rt->winner_id; map_baked_now_ = rt->just_baked;
        map_node_count_ = rt->node_count; map_baked_count_ = rt->baked_count;
        map_qe_ = rt->quant_error; map_expected_ = rt->expected_error; map_trans_ = rt->transition_surp;
        map_pruned_ids_ = rt->just_pruned ? rt->pruned_ids : std::vector<int>{};
    }
    thing_seen_ = false;
    if (auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(bus->last_value("reality.cognitive.thing"))) {
        if (rt->tick_id == tick_id_) { thing_seen_ = true; thing_winner_ = rt->winner_id; thing_tle_ = rt->tle; thing_nodes_ = rt->node_count; }
    }
    // R27: a loop's bearing becomes the heading reference (cx = +right is a clockwise turn, i.e.
    // a negative yaw in the odometry's right-handed frame).  Absent loop -> nothing happens.
    // R28: with an arbiter in the graph, the WINNING loop's bearing sets the reference (gains are
    // 1/0 on arbiter.gain.<loop>: klino = the avoidance loop, play = the play loop); without one,
    // the play bearing alone (R27).  Either way, by presence.
    const auto gain_of = [&](const char* topic) {
        auto g = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus->last_value(topic));
        return (g && g->values.size() > 0) ? double(g->values[0]) : -1.0;
    };
    // T2 (things phase): the arbiter's vision channel is the SEEK loop on the duck (BearingSeekLoop on
    // percept.seek_bearing); its gain is arbiter.gain.vision.  Steer code 3.
    const double g_avoid = gain_of("arbiter.gain.klino"), g_play = gain_of("arbiter.gain.play"), g_seek = gain_of("arbiter.gain.vision");
    last_steer_ = 0;
    const char* bearing_topic = "percept.play_bearing";
    int steer_code = 1;
    if (g_avoid >= 0.0 || g_play >= 0.0 || g_seek >= 0.0) {
        if (g_avoid > 0.5)      { bearing_topic = "percept.avoid_bearing"; steer_code = 2; }
        else if (g_seek > 0.5)  { bearing_topic = "percept.seek_bearing";  steer_code = 3; }
        else if (g_play > 0.5)  { bearing_topic = "percept.play_bearing";  steer_code = 1; }
        else bearing_topic = nullptr;
    }
    seek_present_ = false; seek_arrived_ = false;
    if (auto sv = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus->last_value("reality.cognitive.seek_value")))
        if (sv->values.size() > 0) { seek_present_ = true; seek_value_ = sv->values[0]; }
    if (auto sr = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus->last_value("reality.cognitive.seek_range")))
        if (sr->values.size() > 0) seek_range_ = sr->values[0];
    // a skill request from the graph (O54): a fresh token with request > 0.5 on this tick
    skill_request_ = -1;
    if (auto sk = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus->last_value("intent.skill")))
        if (sk->values.size() >= 2 && sk->values[1] > 0.5f && sk->tick_id == tick_id_) skill_request_ = int(std::lround(sk->values[0]));
    // ARRIVAL (things phase T4): the loop drops a target it has reached -- its need goes to 0 with the range
    // under its arrive threshold -- and the host may start a stop on it (--stop-on-arrive).
    if (seek_present_ && seek_value_prev_ > 0.0 && seek_value_ == 0.0 && seek_range_ < 0.3) seek_arrived_ = true;
    seek_value_prev_ = seek_present_ ? seek_value_ : 0.0;
    if (bearing_topic)
    if (auto pb = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus->last_value(bearing_topic))) {
        if (pb->values.size() >= 2) {
            const double cx = pb->values[0], cy = pb->values[1];
            const auto won = [&]() { ++play_steers_; last_steer_ = steer_code; if (steer_code == 2) ++avoid_steers_; if (steer_code == 3) ++seek_steers_; };
            if (cx * cx + cy * cy > 1e-6) { heading_ref_ = heading_ - std::atan2(cx, cy); won(); if (steer_code == 3) seek_ego_ = std::atan2(cx, cy); }
            else if (g_avoid > 0.5 || g_play > 0.5 || g_seek > 0.5) { heading_ref_ = heading_; won(); }   // a winner with NO bearing releases the reference: no direction held, the reflex acts
        }
    }
    // Wander: boredom is the map's surprise sitting below 0.8 of its own long average
    // (τ 3000 ticks) — self-scaled, no constant tuned to the signal — for bored_s.
    if (wander_bored_s_ > 0.0) {
        map_tle_long_ += (1.0 / 3000.0) * (map_tle_ - map_tle_long_);
        const bool bored_now = map_tle_long_ > 0.0 && map_tle_ < 0.8 * map_tle_long_;
        bored_ticks_ = bored_now ? bored_ticks_ + 1 : 0;
        if (bored_ticks_ >= int(wander_bored_s_ * 50.0)) {
            bored_ticks_ = 0;
            wander_rng_ ^= wander_rng_ << 13; wander_rng_ ^= wander_rng_ >> 7; wander_rng_ ^= wander_rng_ << 17;
            const double sign = (wander_rng_ & 1) ? 1.0 : -1.0;
            heading_ref_ += sign * wander_turn_deg_ * 3.14159265358979323846 / 180.0;
            ++wander_turns_;
        }
    }

    static const char* const kActions[3] = {"action.vx", "action.vy", "action.vyaw"};
    static const double kRanges[3] = {kTwistRangeVx, kTwistRangeVy, kTwistRangeVyaw};
    for (int i = 0; i < 3; ++i) {
        if (auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(bus->last_value(kActions[i])))
            last_twist_[i] = kRanges[i] * std::clamp(double(act->accel), -1.0, 1.0);
    }
    // STUCK (see the header): the stall run and its running median.  Read before the reflex so the
    // command it judges is the brain's own; the sensed velocity is the body's answer to last tick's.
    stuck_now_ = false;
    if (stuck_k_ > 0.0) {
        const bool stalled = last_twist_[0] / kTwistRangeVx > 0.75 && last_sensed_[0] < 0.25f;
        if (stalled) {
            ++stall_run_;
            if (!stuck_fired_ && double(stall_run_) > stuck_k_ * stall_med_ && stall_run_ >= 50) { stuck_now_ = true; stuck_fired_ = true; }
        } else {
            if (stall_run_ > 0) stall_med_ += 0.05 * (double(stall_run_) - stall_med_);   // a slow median-like tracker of stall lengths
            stall_run_ = 0; stuck_fired_ = false;
        }
    }
    // The heading reflex (see the header).  Only while a loop's bearing set the reference this tick.
    hr_share_ = 0.0;
    if (hr_tau_ > 0.0 && last_steer_ != 0) {
        double err = heading_ - heading_ref_;                       // + = the body points left of the reference
        while (err > 3.14159265358979323846) err -= 2.0 * 3.14159265358979323846;
        while (err < -3.14159265358979323846) err += 2.0 * 3.14159265358979323846;
        // the yaw rate that closes the error in tau seconds, minus damping on the sensed rate, in rad/s
        const double want = -err / hr_tau_ - hr_damp_ * vel_body[2];
        const double reflex = std::clamp(want, -kTwistRangeVyaw, kTwistRangeVyaw);
        // the share: 1 with nothing within the gate's reach, 0 at a wall (the brain's avoidance keeps the yaw)
        const double near = std::max({double(tof[0]), double(tof[1]), double(tof[2])});
        hr_share_ = std::clamp(1.0 - near / std::max(1e-6, hr_gate_), 0.0, 1.0);
        last_twist_[2] = hr_share_ * reflex + (1.0 - hr_share_) * last_twist_[2];
    }
    ++tick_id_;
    if (has_override_) return override_;
    if (no_backing_ && last_twist_[0] < 0.0) { last_twist_[0] = 0.0; ++backing_clamped_; }
    return last_twist_;
}

void IntentAdapter::on_reset() {
    auto ev = std::make_shared<ogma::EnvEvent>();
    ev->tick_id = tick_id_;
    ev->producer_id = "host";
    ev->name = "reset";
    ev->intensity = 1.0f;
    instance_->bus()->publish("events.reset", ev);
    last_twist_ = {0.0, 0.0, 0.0};
}

void IntentAdapter::set_learning(bool on) {
    // Freeze through parameters, as the joint-level adapter does: the brain keeps
    // observing while the rescue drives, but must not fit it.
    if (on == !frozen_) return;
    frozen_ = !on;
    static const char* const kRates[] = {"model_lr", "ctrl_lr", "bias_lr", "sat_lr",
                                         "state_prior_lr", "state_prior_h_lr", "state_model_lr"};
    for (auto* module : instance_->modules()) {
        const std::string type(module->type_name());
        if (type != "MotorEPM" && type != "MotorEPMv2") continue;
        const auto params = module->current_params();
        const std::string id(module->id());
        for (const char* rate : kRates) {
            auto it = params.find(rate);
            if (it == params.end()) continue;
            const std::string key = id + ":" + rate;
            if (frozen_) {
                double v = 0.0;
                if (auto d = std::get_if<double>(&it->second)) v = *d;
                else if (auto i = std::get_if<int64_t>(&it->second)) v = double(*i);
                frozen_rates_[key] = v;
                module->on_param_change(rate, ogma::ParamValue{0.0});
            } else if (frozen_rates_.count(key)) {
                module->on_param_change(rate, ogma::ParamValue{frozen_rates_[key]});
            }
        }
    }
}

void IntentAdapter::set_map_learning(bool on) {
    if (map_module_id_.empty() || on == !map_frozen_) return;
    map_frozen_ = !on;
    for (auto* module : instance_->modules()) {
        if (std::string(module->id()) != map_module_id_) continue;
        if (map_frozen_) {
            module->on_param_change("min_insertion_error", ogma::ParamValue{1e9});
            module->on_param_change("epsilon_b",           ogma::ParamValue{0.0});
            module->on_param_change("epsilon_n",           ogma::ParamValue{0.0});
            module->on_param_change("stale_prune_enabled", ogma::ParamValue{false});
        } else {
            module->on_param_change("min_insertion_error", ogma::ParamValue{map_saved_["min_insertion_error"]});
            module->on_param_change("epsilon_b",           ogma::ParamValue{map_saved_["epsilon_b"]});
            module->on_param_change("epsilon_n",           ogma::ParamValue{map_saved_["epsilon_n"]});
            module->on_param_change("stale_prune_enabled", ogma::ParamValue{map_saved_["stale_prune_enabled"] != 0.0});
        }
    }
}

std::string IntentAdapter::place_form_desc() const {
    switch (place_form_) {
        case PlaceForm::Columns: return "12 dims: x, y, cos, sin, the 8 ToF column ranges / 4 m";
        case PlaceForm::ColumnsGaze: return "13 dims: x, y, cos, sin, head yaw / 1.4, the 8 ToF column ranges / 4 m (a view)";
        case PlaceForm::Zones:   return "68 dims: x, y, cos, sin, the 64 ToF zone ranges / 4 m";
        case PlaceForm::Stacked: return std::to_string(place_dims_) + " dims: x, y, cos, sin, the " + std::to_string(depth_dims_)
                                        + "-dim latent of the depth EPM on " + depth_topic_ + " (the 64 zone ranges, frame mean out, on reality.proprio.depth_in)";
    }
    return "?";
}

nlohmann::json IntentAdapter::brain_state() const { return instance_->snapshot_state(); }
std::vector<std::string> IntentAdapter::take_inspector_events() { return inspector_ ? inspector_->take_events() : std::vector<std::string>{}; }

namespace {
// The one CloudMap in the graph, or nullptr.  Looked up each call: there is at most one, the
// module list is short, and this runs only where the host logs.
const ogma::CloudMap* find_cloud(ogma::OgmaInstance& inst) {
    for (auto* m : inst.modules())
        if (auto* c = dynamic_cast<const ogma::CloudMap*>(m)) return c;
    return nullptr;
}
}  // namespace

bool IntentAdapter::cloud_present() const { return find_cloud(*instance_) != nullptr; }
bool IntentAdapter::cloud_open() const { auto* c = find_cloud(*instance_); return c && c->is_open(); }
bool IntentAdapter::cloud_just_closed() const { auto* c = find_cloud(*instance_); return c && c->just_closed(); }
int  IntentAdapter::cloud_voxels() const { auto* c = find_cloud(*instance_); return c ? c->voxels() : 0; }
int  IntentAdapter::cloud_break() const { auto* c = find_cloud(*instance_); return c ? c->break_voxels() : 0; }
int  IntentAdapter::cloud_place() const { auto* c = find_cloud(*instance_); return c ? c->last_key() : -1; }
double IntentAdapter::cloud_newfrac() const { auto* c = find_cloud(*instance_); return c ? c->new_fraction() : 0.0; }
double IntentAdapter::cloud_revisit() const { auto* c = find_cloud(*instance_); return c ? c->revisit_change() : -1.0; }
double IntentAdapter::cloud_revisit_dist() const { auto* c = find_cloud(*instance_); return c ? c->revisit_anchor_dist() : -1.0; }
int  IntentAdapter::cloud_cached() const { auto* c = find_cloud(*instance_); return c ? c->cached() : 0; }
std::vector<int32_t> IntentAdapter::cloud_filed_voxels() const {
    auto* c = find_cloud(*instance_);
    return c ? c->last_filed_voxels() : std::vector<int32_t>{};
}
std::vector<float> IntentAdapter::cloud_profile() const {
    auto* c = find_cloud(*instance_);
    return c ? c->profile() : std::vector<float>{};
}
std::vector<float> IntentAdapter::cloud_view() const {
    auto* c = find_cloud(*instance_);
    return c ? c->view() : std::vector<float>{};
}
bool IntentAdapter::cloud_things_on() const {
    auto* c = find_cloud(*instance_);
    if (!c) return false;
    const auto p = c->current_params();
    for (const char* k : {"things_topic", "thing_bearing_topic"}) {
        auto it = p.find(k);
        if (it == p.end()) continue;
        if (auto sv = std::get_if<std::string>(&it->second)) if (!sv->empty()) return true;
    }
    return false;
}
std::vector<ogma::CloudMap::Thing> IntentAdapter::cloud_things() const {
    auto* c = find_cloud(*instance_);
    return c ? c->things() : std::vector<ogma::CloudMap::Thing>{};
}
int IntentAdapter::cloud_attended() const { auto* c = find_cloud(*instance_); return c ? c->attended() : -1; }
std::array<float, 3> IntentAdapter::cloud_thing_bearing() const {
    auto* c = find_cloud(*instance_);
    return c ? c->thing_bearing() : std::array<float, 3>{0.0f, 0.0f, 0.0f};
}
std::vector<ogma::CloudMap::Thing> IntentAdapter::cloud_filed_things() const {
    auto* c = find_cloud(*instance_);
    return c ? c->last_filed_things() : std::vector<ogma::CloudMap::Thing>{};
}
double IntentAdapter::cloud_voxel_m() const {
    auto* c = find_cloud(*instance_);
    if (!c) return 0.0;
    const auto p = c->current_params();
    auto it = p.find("voxel_m");
    if (it == p.end()) return 0.0;
    if (auto d = std::get_if<double>(&it->second)) return *d;
    return 0.0;
}

double IntentAdapter::motor_tle() const {
    for (auto* m : instance_->modules()) {
        const std::string type(m->type_name());
        if (type != "MotorEPM" && type != "MotorEPMv2") continue;
        const auto d = m->diag_lite();
        if (d.contains("motor_tle")) return d["motor_tle"].get<double>();
    }
    return -1.0;
}

void IntentAdapter::set_wander(double bored_s, double turn_deg, uint64_t seed) {
    wander_bored_s_ = bored_s; wander_turn_deg_ = turn_deg;
    wander_rng_ ^= (seed + 1) * 0x9E3779B97F4A7C15ull;
}

int IntentAdapter::map_nodes() const {
    for (auto* m : instance_->modules()) {
        if (std::string(m->id()) != "map_epm") continue;
        const auto d = m->diag_lite();
        if (d.contains("nodes")) return d["nodes"].get<int>();
    }
    return -1;
}

std::vector<std::string> IntentAdapter::diagnostics() const {
    std::vector<std::string> out;
    if (play_steers_ > 0) out.push_back("loop-heading: " + std::to_string(play_steers_) + " ticks steered by a loop's bearing (" + std::to_string(avoid_steers_) + " by avoidance)");
    for (auto* m : instance_->modules()) {
        const auto d = m->diag_lite();
        if (!d.is_null() && !d.empty()) out.emplace_back(std::string(m->id()) + " " + d.dump());
    }
    return out;
}

}  // namespace mjhost
