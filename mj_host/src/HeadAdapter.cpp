#include "HeadAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <nlohmann/json.hpp>

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
    instance_ = std::make_unique<ogma::OgmaInstance>(std::move(cfg), std::make_unique<ogma::InProcessBus>());
    // No inspector surface: the twist brain owns the host's inspector port. The head brain's
    // diagnostics reach the log through diagnostics() and readback().
}

HeadAdapter::~HeadAdapter() = default;

std::array<double, 4> HeadAdapter::tick(const std::array<double, 4>& head_q,
                                        const std::array<double, 3>& hg,
                                        const std::array<double, 3>& hw,
                                        const std::array<double, 3>& g,
                                        const std::array<double, 3>& w) {
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
    // The 12-slot sense the bridge appends as load slots.  Slots 0-1: head gravity x, y (level
    // is 0, 0); 2-4: head gyro, 0.3 rad/s to unit; 5-6: trunk gravity x, y; 7-9: trunk gyro;
    // 10-11: spare.  A prior on slots 0, 1 (level) and 2, 3 (still) is H2.
    publish("head_sense", {float(hg[0]), float(hg[1]), unit(0.3 * hw[0]), unit(0.3 * hw[1]), unit(0.3 * hw[2]),
                           float(g[0]), float(g[1]), unit(0.3 * w[0]), unit(0.3 * w[1]), unit(0.3 * w[2]),
                           0.0f, 0.0f});

    instance_->tick();

    static const char* const kActions[4] = {"action.neck_pitch", "action.head_pitch", "action.head_yaw", "action.head_roll"};
    for (int i = 0; i < 4; ++i) {
        if (auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(bus->last_value(kActions[i])))
            last_cmd_[size_t(i)] = kHeadRange[size_t(i)] * std::clamp(double(act->accel), -1.0, 1.0);
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

nlohmann::json HeadAdapter::brain_state() const { return instance_->snapshot_state(); }

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
        if (cols != 4 || rows < 12) continue;
        const auto at = [&](int i, int j) { return A[size_t(i) + size_t(rows) * size_t(j)]; };
        static const char* const kCmd[4] = {"neck_pitch", "head_pitch", "head_yaw", "head_roll"};
        char buf[256];
        std::snprintf(buf, sizeof buf, "identified A of %s (row vs command %s/%s/%s/%s; want the position diagonal "
                      "positive and dominant, the pitches on head-gravity x, roll on y):", it.key().c_str(),
                      kCmd[0], kCmd[1], kCmd[2], kCmd[3]);
        out.emplace_back(buf);
        for (int i = 0; i < 4; ++i) {
            std::snprintf(buf, sizeof buf, "  pos %-10s: %+.4f %+.4f %+.4f %+.4f", kCmd[i], at(3 * i, 0), at(3 * i, 1), at(3 * i, 2), at(3 * i, 3));
            out.emplace_back(buf);
        }
        static const char* const kLoad[5] = {"head g_x  ", "head g_y  ", "head w_x  ", "head w_y  ", "head w_z  "};
        for (int k = 0; k < 5 && 12 + k < rows; ++k) {
            std::snprintf(buf, sizeof buf, "  %s    : %+.4f %+.4f %+.4f %+.4f", kLoad[k], at(12 + k, 0), at(12 + k, 1), at(12 + k, 2), at(12 + k, 3));
            out.emplace_back(buf);
        }
    }
    return out;
}

}  // namespace mjhost
