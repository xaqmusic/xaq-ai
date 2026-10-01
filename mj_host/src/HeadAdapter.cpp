#include "HeadAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <nlohmann/json.hpp>

#include "DuckBody.hpp"          // kBrainHz
#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Rng.hpp"
#include "ogma/Topics.hpp"

namespace mjhost {

HeadAdapter::HeadAdapter(const std::string& graph_path, uint64_t seed) {
    auto cfg = ogma::GraphConfig::load_from_file(graph_path);
    if (seed != 0) {
        for (auto& m : cfg.modules)
            for (const char* pname : {"master_seed", "seed"}) {
                auto it = m.params.find(pname);
                if (it != m.params.end())
                    it->second = ogma::ParamValue{int64_t(ogma::namespace_seed(seed, m.id))};
            }
    }
    for (const auto& m : cfg.modules) {                // the babble's length, so the yaw mask waits for it
        // a graph that owns the yaw axis (an action.head_yaw topic) is not masked: yaw is then the
        // loop's to hold — the gaze axis (2026-09-10: yaw carries the largest share of the picture's motion)
        for (const char* key : {"action_topics"}) {
            auto at = m.params.find(key);
            if (at == m.params.end()) continue;
            if (auto lst = std::get_if<std::vector<std::string>>(&at->second))
                for (const auto& t : *lst) if (t == "action.head_yaw") mask_yaw_ = false;
        }
        auto it = m.params.find("babble_ticks");
        if (it == m.params.end()) continue;
        if (auto i = std::get_if<int64_t>(&it->second)) babble_ticks_ = uint64_t(std::max<int64_t>(0, *i));
        else if (auto d = std::get_if<double>(&it->second)) babble_ticks_ = uint64_t(std::max(0.0, *d));
    }
    instance_ = std::make_unique<ogma::OgmaInstance>(std::move(cfg), std::make_unique<ogma::InProcessBus>());
    // No inspector surface: the twist brain owns the host's inspector port. The head brain's
    // diagnostics reach the log through diagnostics() and readback().
}

HeadAdapter::~HeadAdapter() = default;

std::array<double, 4> HeadAdapter::tick(const std::array<double, 4>& head_q,
                                        const std::array<double, 3>& hg,
                                        const std::array<double, 3>& hw,
                                        const std::array<double, 3>& g,
                                        const std::array<double, 3>& w,
                                        double gait) {
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

    // The head "joints": the four head positions in the walker's command units — where the
    // command was answered.
    std::vector<float> q(4);
    for (int i = 0; i < 4; ++i) q[size_t(i)] = unit(head_q[size_t(i)] / kHeadRange[size_t(i)]);
    publish("head", q);
    publish("imu", {float(g[0]), float(g[1]), float(g[2]), float(w[0]), float(w[1]), float(w[2])});
    // The 12-slot sense the bridge appends as load slots.  The head IMU's frame has its x
    // axis DOWN when the camera is level (measured 2026-09-10: head joints at zero → camera
    // forward = world +x, head gravity = (−1, 0, 0)), so a level head reads gravity (−1, 0, 0)
    // and the errors are the y (roll) and z (pitch) components.  Slots 0-2 are those
    // deviations — roll, pitch, and the down component's shortfall (hg_x + 1) — all ≈ 0 when
    // level, so no large common-mode rides into the model (CLAUDE.md §0 rule 2; the first
    // layout carried hg_x ≈ −1 on slot 0 and the idle controller turned it into a constant
    // 0.3 rad command).  3-5: head gyro, 0.3 rad/s to unit; 6-7: trunk gravity x, y; 8-10:
    // trunk gyro; 11: spare.  The H2 prior is on slots 0, 1 (level) and 3, 4 (still).
    publish("head_sense", {float(hg[1]), float(hg[2]), float(hg[0] + 1.0), unit(0.3 * hw[0]), unit(0.3 * hw[1]), unit(0.3 * hw[2]),
                           float(g[0]), float(g[1]), unit(0.3 * w[0]), unit(0.3 * w[1]), unit(0.3 * w[2]),
                           gaze_sense_ ? unit(gaze_err_ / kHeadRange[2]) : 0.0f});   // 11: the gaze error (--head-gaze-sense), else spare

    instance_->tick();

    static const char* const kActions[4] = {"action.neck_pitch", "action.head_pitch", "action.head_yaw", "action.head_roll"};
    for (int i = 0; i < 4; ++i) {
        if (auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(bus->last_value(kActions[i])))
            last_cmd_[size_t(i)] = kHeadRange[size_t(i)] * std::clamp(double(act->accel), -1.0, 1.0);
    }
    // Yaw follows the trunk (the plan's design choice, §4b): the head loop levels and steadies
    // pitch and roll and never turns the head. Measured reason (H2, 2026-09-10): the
    // controller's identity start makes an unpriored axis hold its position, and a head that
    // holds its yaw while the body turns under it winds to the rail. The babble still moves
    // yaw (the action is read above for the model's sake); only the command is masked.
    if (mask_yaw_ && tick_id_ >= babble_ticks_) last_cmd_[2] = 0.0;
    const double yt = std::clamp(yaw_target_, -kHeadRange[2], kHeadRange[2]);
    const double pt = std::clamp(pitch_target_, -kHeadRange[1], kHeadRange[1]);
    if (slew_ > 0.0 && (yaw_override_ || pitch_override_)) {
        // Prime on the first overridden tick so the slew starts from where the head IS, not from
        // zero -- otherwise the limiter itself commands a sweep the moment the override turns on.
        if (!slew_primed_) { yaw_held_ = last_cmd_[2]; pitch_held_ = last_cmd_[1]; slew_primed_ = true; }
        const double step = slew_ / kBrainHz;
        yaw_held_   += std::clamp(yt - yaw_held_,   -step, step);
        pitch_held_ += std::clamp(pt - pitch_held_, -step, step);
        if (yaw_override_)   last_cmd_[2] = yaw_held_;
        if (pitch_override_) last_cmd_[1] = pitch_held_;
    } else {
        slew_primed_ = false;
        if (yaw_override_)   last_cmd_[2] = yt;
        if (pitch_override_) last_cmd_[1] = pt;
    }
    if (vor_tau_ > 0.0 && tick_id_ >= babble_ticks_) {
        constexpr double dt = 1.0 / 50.0;
        vor_state_ += w[2] * dt;                        // the trunk's yaw increment this tick
        vor_state_ -= vor_state_ * (dt / vor_tau_);     // the leak: a slow turn passes, a wobble is held
        last_cmd_[2] = std::clamp(-(vor_state_ + vor_lead_ * w[2]), -kHeadRange[2], kHeadRange[2]);
    }
    if (phase_lead_ > 0.0 && tick_id_ >= babble_ticks_) {
        // 1. the stride clock: upward crossings of the hip pitch through its own running mean
        if (!gait_mean_init_) { gait_mean_ = gait; gait_prev_ = gait; gait_mean_init_ = true; }
        gait_mean_ += (1.0 / 100.0) * (gait - gait_mean_);            // τ 2 s: the mean, not the stride
        const bool up = (gait_prev_ - gait_mean_) <= 0.0 && (gait - gait_mean_) > 0.0;
        gait_prev_ = gait;
        if (up && tick_id_ - last_cross_ >= 8) {                       // ≥ 160 ms: not a wobble
            const double p = double(tick_id_ - last_cross_);
            if (last_cross_ > 0 && p >= 10.0 && p <= 60.0) {           // a gait: 0.2–1.2 s per stride
                period_ticks_ = (period_ticks_ > 0.0) ? 0.8 * period_ticks_ + 0.2 * p : p;
                ++crossings_;
            }
            last_cross_ = tick_id_;
        }
        const bool have_clock = period_ticks_ > 0.0 && crossings_ >= 3 && double(tick_id_ - last_cross_) < 2.0 * period_ticks_;
        const double phase = have_clock ? std::fmod(double(tick_id_ - last_cross_) / period_ticks_, 1.0) : -1.0;
        const uint64_t learn_until = babble_ticks_ + uint64_t(phase_learn_s_ * 50.0);
        if (have_clock && tick_id_ < learn_until) {
            // 2. learn: the head's yaw rate per phase bin, with the command at zero
            const int b = int(phase * kPhaseBins) % kPhaseBins;
            rate_bin_[size_t(b)] += 0.05 * (hw[0] - rate_bin_[size_t(b)]); ++rate_n_[size_t(b)];
            learn_ms_ += hw[0] * hw[0]; ++learn_n_;
            last_cmd_[2] = 0.0;
        } else if (tick_id_ >= learn_until) {
            if (!phase_frozen_) {
                // 3. freeze: integrate the periodic rate into the periodic yaw angle, zero-mean
                const double dt_bin = (period_ticks_ / kPhaseBins) / 50.0;
                double acc = 0.0, mean = 0.0;
                for (int i = 0; i < kPhaseBins; ++i) { acc += rate_bin_[size_t(i)] * dt_bin; angle_bin_[size_t(i)] = acc; mean += acc; }
                mean /= kPhaseBins;
                for (auto& a : angle_bin_) a -= mean;
                phase_frozen_ = true;
            }
            // 4. act: the opposite of the angle the head is about to make, LEAD ticks early
            if (have_clock) {
                const double ph = std::fmod(phase + phase_lead_ / period_ticks_, 1.0);
                const double x = ph * kPhaseBins; const int i0 = int(x) % kPhaseBins, i1 = (i0 + 1) % kPhaseBins;
                const double f = x - std::floor(x);
                const double a = (1.0 - f) * angle_bin_[size_t(i0)] + f * angle_bin_[size_t(i1)];
                last_cmd_[2] = std::clamp(-a, -kHeadRange[2], kHeadRange[2]);
                const int b = int(phase * kPhaseBins) % kPhaseBins;
                act_ms_ += hw[0] * hw[0]; act_res_ms_ += (hw[0] - rate_bin_[size_t(b)]) * (hw[0] - rate_bin_[size_t(b)]); ++act_n_;
            } else {
                last_cmd_[2] = 0.0;                                     // no stride clock (standing): nothing to cancel
            }
        }
    }
    if (rate_k_ > 0.0 && rate_tau_ > 0.0 && tick_id_ >= babble_ticks_) {
        constexpr double dt = 1.0 / 50.0;
        rate_state_ += hw[0] * dt;                       // the head's own yaw increment (gyro x: the down axis)
        rate_state_ -= rate_state_ * (dt / rate_tau_);   // the anchor: leak to centre
        last_cmd_[2] = std::clamp(-rate_k_ * rate_state_, -kHeadRange[2], kHeadRange[2]);
    }
    ++tick_id_;
    return last_cmd_;
}

void HeadAdapter::on_reset() {
    auto ev = std::make_shared<ogma::EnvEvent>();
    ev->tick_id = tick_id_;
    ev->producer_id = "host";
    ev->name = "reset";
    ev->intensity = 1.0f;
    instance_->bus()->publish("events.reset", ev);
    last_cmd_ = {0.0, 0.0, 0.0, 0.0};
    vor_state_ = 0.0;
    rate_state_ = 0.0;
}

void HeadAdapter::set_learning(bool on) {
    // As the twist brain: freeze through the learning rates while the rescue drives.
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

std::vector<std::string> HeadAdapter::phase_report() const {
    std::vector<std::string> out;
    if (phase_lead_ <= 0.0) return out;
    char buf[256];
    double pk = 0.0; for (auto a : angle_bin_) pk = std::max(pk, std::fabs(a));
    std::snprintf(buf, sizeof buf, "phase feed-forward: stride %.0f ticks (%.2f Hz) from %d crossings; learned yaw-rate RMS %.2f rad/s over %llu ticks; "
                  "angle waveform ±%.3f rad; while acting: yaw-rate RMS %.2f, residual vs the table %.2f (%llu ticks)",
                  period_ticks_, period_ticks_ > 0 ? 50.0 / period_ticks_ : 0.0, crossings_,
                  learn_n_ ? std::sqrt(learn_ms_ / double(learn_n_)) : 0.0, (unsigned long long)learn_n_, pk,
                  act_n_ ? std::sqrt(act_ms_ / double(act_n_)) : 0.0, act_n_ ? std::sqrt(act_res_ms_ / double(act_n_)) : 0.0, (unsigned long long)act_n_);
    out.emplace_back(buf);
    std::string tbl = "  yaw-rate table by phase bin:";
    for (auto v : rate_bin_) { std::snprintf(buf, sizeof buf, " %+.2f", v); tbl += buf; }
    out.emplace_back(tbl);
    return out;
}

nlohmann::json HeadAdapter::brain_state() const { return instance_->snapshot_state(); }
void HeadAdapter::restore_brain_state(const nlohmann::json& s) {
    std::lock_guard<std::recursive_mutex> lk(instance_mtx_);
    instance_->restore_state(s);
}

std::vector<std::string> HeadAdapter::diagnostics() const {
    std::vector<std::string> out;
    for (auto* m : instance_->modules()) {
        const auto d = m->diag_lite();
        if (!d.is_null() && !d.empty()) out.emplace_back(std::string("head/") + std::string(m->id()) + " " + d.dump());
    }
    return out;
}

std::vector<std::string> HeadAdapter::readback() const {
    // The bridge lays the module's state out as [pos, act, delta] per motor (12 rows for the
    // four head axes), then the 12 load slots.  Row 3i = sensed position of axis i; the load
    // rows follow at 12 + k.  Column j = command j.  Column-major, as the snapshot stores it.
    std::vector<std::string> out;
    const auto snap = instance_->snapshot_state();
    if (!snap.contains("modules")) return out;
    const auto& mods = snap.at("modules");
    for (auto it = mods.begin(); it != mods.end(); ++it) {
        if (!it.value().contains("legs")) continue;
        const auto& leg = it.value().at("legs").at(0);
        const int rows = leg.at("rows_A").get<int>(), cols = leg.at("cols_A").get<int>();
        const auto A = leg.at("A").get<std::vector<double>>();
        if ((cols != 4 && cols != 3 && cols != 2) || rows < 3 * cols + 6) continue;
        const auto at = [&](int i, int j) { return A[size_t(i) + size_t(rows) * size_t(j)]; };
        // the four-motor graph commands all head joints; the two-motor graph head_pitch and head_roll
        static const char* const kCmd4[4] = {"neck_pitch", "head_pitch", "head_yaw", "head_roll"};
        static const char* const kCmd3[3] = {"head_pitch", "head_yaw", "head_roll"};
        static const char* const kCmd2[2] = {"head_pitch", "head_roll"};
        const char* const* kCmd = (cols == 4) ? kCmd4 : (cols == 3) ? kCmd3 : kCmd2;
        char buf[256];
        std::string head = "identified A of " + it.key() + " (row vs command";
        for (int j = 0; j < cols; ++j) head += std::string(j ? "/" : " ") + kCmd[j];
        head += "; want the position diagonal positive and dominant, the pitches on head pitch (g_z), roll on head roll (g_y)):";
        out.emplace_back(head);
        const auto row = [&](const std::string& label, int i) {
            std::string line = "  " + label + ":";
            for (int j = 0; j < cols; ++j) { std::snprintf(buf, sizeof buf, " %+.4f", at(i, j)); line += buf; }
            out.emplace_back(line);
        };
        for (int i = 0; i < cols; ++i) row(std::string("pos ") + kCmd[i] + std::string(10 - std::min<size_t>(10, std::strlen(kCmd[i])), ' '), 3 * i);
        static const char* const kLoad[6] = {"head roll     ", "head pitch    ", "head down     ", "head w_x      ", "head w_y      ", "head w_z      "};
        for (int k = 0; k < 6 && 3 * cols + k < rows; ++k) row(kLoad[k], 3 * cols + k);
    }
    return out;
}

}  // namespace mjhost
