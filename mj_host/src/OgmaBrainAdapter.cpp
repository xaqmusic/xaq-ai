#include "OgmaBrainAdapter.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <stdexcept>
#include <functional>
#include <variant>

#include <ogma/GraphConfig.hpp>
#include <ogma/InProcessBus.hpp>
#include <ogma/Module.hpp>
#include <ogma/OgmaInstance.hpp>
#include <ogma/Rng.hpp>
#include <ogma/Topics.hpp>

#include <nlohmann/json.hpp>

namespace mjhost {

namespace {

// Every module that owns an RNG names the param one of these two. Rewriting only
// the first is how the picrawler spent a campaign with a seed flag that changed
// nothing: MotorEPM's stream is called "seed", so OGMA_SEED moved no numbers.
const char* const kSeedParams[] = {"master_seed", "seed"};

}  // namespace

OgmaBrainAdapter::OgmaBrainAdapter(const DuckBody& body, Config config) : c_(std::move(config)) {
    auto cfg = ogma::GraphConfig::load_from_file(c_.graph_path);

    if (c_.seed != 0) {
        for (auto& m : cfg.modules) {
            for (const char* pname : kSeedParams) {
                auto it = m.params.find(pname);
                if (it == m.params.end()) continue;
                it->second = ogma::ParamValue{int64_t(ogma::namespace_seed(c_.seed, m.id))};
            }
        }
    }

    instance_ = std::make_unique<ogma::OgmaInstance>(std::move(cfg),
                                                     std::make_unique<ogma::InProcessBus>());

    // The action topics this host polls, one per policy joint, named after the
    // joint itself. The graph config must publish exactly these.
    for (int i = 0; i < kNumPolicyJoints; ++i) action_topics_[i] = std::string("action.") + kPolicyJoints[i];

    // Joint travel, read from the model rather than transcribed, so a command can
    // never be sent outside the range the body actually has.
    const mjModel* m = body.model();
    range_.resize(kNumPolicyJoints);
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const int jid = mj_name2id(m, mjOBJ_JOINT, kPolicyJoints[i]);
        range_[i] = {m->jnt_range[2 * jid], m->jnt_range[2 * jid + 1]};
    }
}

OgmaBrainAdapter::~OgmaBrainAdapter() = default;

void OgmaBrainAdapter::publish_sensors(const DuckBody& body) {
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

    // joints — centred on the home pose and scaled by the command amplitude, so
    // the resting pose is the origin and sensor and action share a unit. Clamped
    // because the bridge's own channel is [-1, 1] and a value outside it would be
    // silently squashed there instead of visibly here.
    const auto q = body.joint_positions();
    std::vector<float> joints(kNumPolicyJoints);
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const double norm = (q[i] - kHomePose[i]) / c_.amplitude;
        joints[size_t(i)] = float(std::clamp(norm, -1.0, 1.0));
    }
    publish("joints", joints);

    // imu — projected gravity and the gyro, the two channels the real robot
    // publishes. Egocentric, and the same six numbers a hardware brain would see.
    const auto g = body.gravity();
    const auto w = body.gyro();
    publish("imu", {float(g[0]), float(g[1]), float(g[2]),
                    float(w[0]), float(w[1]), float(w[2])});

    // lean — SIGNED FORE/AFT LEAN, the channel A1 was missing.
    //
    // Each motor group's state was [pos, action, delta] per joint: angles, actions,
    // joint velocities, and nothing else. A duck at its exact rest pose has the same
    // proprioceptive state standing up as lying on its side, so an inverted pendulum
    // cannot be balanced from it — which is also why the postural reflex changed
    // nothing, since it pulls toward a joint pose the body was already in.
    //
    // Doctrine §1 step 2: the fix for that is a SENSOR, not a smarter policy.
    // The bridge's load socket appends this as the trailing element of every group's
    // vector, and MotorEPM sizes its state from the arriving vector (`L.x =
    // pt->values`), so the forward model learns how its own actions move the lean
    // without any module being edited.
    //
    // x is fore/aft in the trunk frame (the hip bodies are separated on y), and the
    // same value goes to every group on purpose: each leg owns its own C, so the two
    // legs LEARN opposite responses rather than being told them. LEARNED cooperates,
    // IMPOSED fights.
    //
    // Roll is deliberately absent — the socket is one element wide, and widening it
    // is a module change and therefore a separate conversation.
    publish("lean", {float(g[0]), float(g[0])});
}

std::array<double, kNumPolicyJoints> OgmaBrainAdapter::act(const DuckBody& body) {
    publish_sensors(body);

    instance_->tick();

    // Poll AFTER the tick and BEFORE bumping the tick id: modules stamped their
    // tokens with the Scheduler's current tick, and bumping first shifts the
    // comparison frame so every fresh token reads as stale by one.
    auto* bus = instance_->bus();
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        if (auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
                bus->last_value(action_topics_[size_t(i)]))) {
            last_u_[size_t(i)] = double(a->accel);
        }
    }
    ++tick_id_;

    std::array<double, kNumPolicyJoints> target{};
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const double u = std::clamp(last_u_[size_t(i)], -1.0, 1.0);
        action_abs_sum_ += std::fabs(u);
        ++action_samples_;
        const double want = kHomePose[i] + c_.amplitude * u;
        target[size_t(i)] = std::clamp(want, range_[size_t(i)].first, range_[size_t(i)].second);
    }
    return target;
}

void OgmaBrainAdapter::on_reset() {
    // The picrawler's auto-reset fired no event and MotorEPM's phase and EMAs
    // survived fall-plus-respawn, so every trend spanning a reset was fake. This
    // is that bug's fix, on the bus where a consumer can actually mask on it.
    auto ev = std::make_shared<ogma::EnvEvent>();
    ev->tick_id = tick_id_;
    ev->producer_id = "host";
    ev->name = "reset";
    ev->intensity = 1.0f;
    instance_->bus()->publish("events.reset", ev);
    last_u_.fill(0.0);
}

void OgmaBrainAdapter::set_learning(bool on) {
    if (on == learning_) return;
    learning_ = on;

    // Freeze through PARAMETERS rather than by skipping ticks. The brain must keep
    // observing while the scaffold drives — the fall is the most informative thing
    // that happens — but it must not fit the scaffold's policy (§5.6). All four
    // rates are HotMutable, so no module is edited to do this.
    static const char* const kRates[] = {"model_lr", "ctrl_lr", "bias_lr", "sat_lr"};
    bool reported = false;
    for (auto* module : instance_->modules()) {
        const std::string type(module->type_name());
        if (type != "MotorEPM" && type != "MotorEPMv2") continue;
        const auto params = module->current_params();
        const std::string id(module->id());
        for (const char* rate : kRates) {
            auto it = params.find(rate);
            if (it == params.end()) continue;
            const std::string key = id + ":" + rate;
            // §3.2 rule 5: verify the consumer fired. Reported once, the first time
            // a freeze happens, because "I called set_param" and "the value changed"
            // are different claims and only the second one matters.
            if (!announced_freeze_ && !on) {
                if (const double* v = std::get_if<double>(&it->second))
                    std::fprintf(stderr, "  freeze: %s %s %.4f -> 0\n", id.c_str(), rate, *v);
            }
            if (on) {
                auto saved = frozen_rates_.find(key);
                if (saved != frozen_rates_.end())
                    module->on_param_change(rate, ogma::ParamValue{saved->second});
            } else {
                if (const double* v = std::get_if<double>(&it->second)) frozen_rates_[key] = *v;
                module->on_param_change(rate, ogma::ParamValue{0.0});
                reported = true;
            }
        }
    }
    if (reported) announced_freeze_ = true;

    // And confirm it landed, by reading the params back rather than assuming.
    if (!on) {
        for (auto* module : instance_->modules()) {
            const std::string type(module->type_name());
            if (type != "MotorEPM" && type != "MotorEPMv2") continue;
            const auto after = module->current_params();
            for (const char* rate : kRates) {
                auto it = after.find(rate);
                if (it == after.end()) continue;
                const double* v = std::get_if<double>(&it->second);
                if (v && *v != 0.0) {
                    std::fprintf(stderr, "  !! freeze did NOT take: %s %s is still %.4f\n",
                                 std::string(module->id()).c_str(), rate, *v);
                }
            }
        }
    }
}

void OgmaBrainAdapter::sample_tle(bool upright) {
    double tle = 0.0;
    int n = 0;
    for (auto* m : instance_->modules()) {
        const std::string type(m->type_name());
        if (type != "MotorEPM" && type != "MotorEPMv2") continue;
        const auto d = m->diag_lite();
        if (!d.contains("motor_tle")) continue;
        tle += d["motor_tle"].get<double>();
        ++n;
    }
    if (!n) return;
    tle /= n;
    if (upright) { tle_up_sum_ += tle; ++tle_up_n_; }
    else         { tle_down_sum_ += tle; ++tle_down_n_; }
}

double OgmaBrainAdapter::tle_upright() const {
    return tle_up_n_ ? tle_up_sum_ / double(tle_up_n_) : 0.0;
}
double OgmaBrainAdapter::tle_down() const {
    return tle_down_n_ ? tle_down_sum_ / double(tle_down_n_) : 0.0;
}

std::vector<std::string> OgmaBrainAdapter::diagnostics() const {
    std::vector<std::string> out;
    for (auto* m : instance_->modules()) {
        const auto d = m->diag_lite();
        if (d.is_null() || d.empty()) continue;
        std::string line = std::string(m->id()) + " " + d.dump();
        // §3.2 rule 5 again: the state width, read from the module rather than
        // inferred from the config. An appended sensor channel that never widened
        // the model is a knob that cannot act, and it would look causal anyway.
        // Search the snapshot for the forward model's own row count rather than
        // guessing at its layout: A maps action -> state, so rows_A IS the state
        // width the model was built to.
        std::function<int(const nlohmann::json&)> find_rows = [&](const nlohmann::json& j) -> int {
            if (j.is_object()) {
                auto it = j.find("rows_A");
                if (it != j.end() && it->is_number()) return it->get<int>();
                for (auto& kv : j.items()) { int r = find_rows(kv.value()); if (r) return r; }
            } else if (j.is_array()) {
                for (auto& e : j) { int r = find_rows(e); if (r) return r; }
            }
            return 0;
        };
        if (const int rows = find_rows(m->snapshot_state())) line += "  state_dim=" + std::to_string(rows);
        out.push_back(line);
    }
    return out;
}

double OgmaBrainAdapter::mean_abs_action() const {
    return action_samples_ ? action_abs_sum_ / double(action_samples_) : 0.0;
}

std::vector<std::string> OgmaBrainAdapter::module_ids() const {
    std::vector<std::string> out;
    for (auto* m : instance_->modules()) out.emplace_back(m->id());
    return out;
}

}  // namespace mjhost
