#include "IntentAdapter.hpp"

#include <algorithm>
#include <cmath>

#include <nlohmann/json.hpp>

#include "ogma/GraphConfig.hpp"
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
    instance_ = std::make_unique<ogma::OgmaInstance>(std::move(cfg), std::make_unique<ogma::InProcessBus>());
}

IntentAdapter::~IntentAdapter() = default;

std::array<double, 3> IntentAdapter::tick(const std::array<double, 3>& vel_body,
                                          const std::array<double, 3>& g,
                                          const std::array<double, 3>& w,
                                          const std::array<double, 3>& a,
                                          double odom_yaw) {
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
    publish("sense", {float(g[0]), float(g[1]), unit(0.3 * w[1]), unit(0.3 * w[0]), unit(0.3 * w[2]),
                      unit(a[0] / 20.0), unit(a[1] / 20.0), unit(a[2] / 20.0),
                      last_sensed_[0], last_sensed_[1], unit((heading_ - heading_ref_) / 3.14159265358979323846), float(std::cos(odom_yaw))});

    instance_->tick();

    static const char* const kActions[3] = {"action.vx", "action.vy", "action.vyaw"};
    static const double kRanges[3] = {kTwistRangeVx, kTwistRangeVy, kTwistRangeVyaw};
    for (int i = 0; i < 3; ++i) {
        if (auto act = std::dynamic_pointer_cast<const ogma::ActionOut>(bus->last_value(kActions[i])))
            last_twist_[i] = kRanges[i] * std::clamp(double(act->accel), -1.0, 1.0);
    }
    ++tick_id_;
    if (has_override_) return override_;
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

nlohmann::json IntentAdapter::brain_state() const { return instance_->snapshot_state(); }

std::vector<std::string> IntentAdapter::diagnostics() const {
    std::vector<std::string> out;
    for (auto* m : instance_->modules()) {
        const auto d = m->diag_lite();
        if (!d.is_null() && !d.empty()) out.emplace_back(std::string(m->id()) + " " + d.dump());
    }
    return out;
}

}  // namespace mjhost
