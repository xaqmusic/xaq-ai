#include "InspectorSurface.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

#include <nlohmann/json.hpp>

#include "control_server.hpp"
#include "ogma/DiagPublisher.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Module.hpp"

namespace mjhost {

InspectorSurface::InspectorSurface(ogma::OgmaInstance& instance, std::recursive_mutex& mtx)
    : instance_(instance), mtx_(mtx) {
    uint16_t control_port = 7400;
    if (const char* env = std::getenv("OGMA_INSPECTOR_PORT")) {
        const int p = std::atoi(env);
        if (p == 0) return;                       // disabled on purpose
        if (p > 1024 && p < 65534) control_port = uint16_t(p);
    }
    const uint16_t diag_port = uint16_t(control_port + 1);
    try {
        diag_ = std::make_unique<ogma::DiagPublisher>(diag_port);
        if (!diag_->start()) {
            std::fprintf(stderr, "inspector: diag port %u busy — running without an inspector\n", diag_port);
            diag_.reset();
            return;
        }
        control_ = std::make_unique<ami_ogma::control::ControlServer>(control_port);
        control_->set_command_handler([this](nlohmann::json const& req) -> nlohmann::json {
            std::lock_guard<std::recursive_mutex> lk(mtx_);
            const std::string verb = req.value("verb", std::string());
            try {
                if (verb == "list_modules") {
                    nlohmann::json mods = nlohmann::json::array();
                    for (auto* m : instance_.modules())
                        mods.push_back({{"id", std::string(m->id())}, {"type", std::string(m->type_name())}});
                    return {{"status", "ok"}, {"modules", mods}};
                }
                if (verb == "module_snapshot") {
                    const std::string id = req.value("id", std::string());
                    auto* m = instance_.module(id);
                    if (!m) return {{"status", "error"}, {"message", "unknown module: " + id}};
                    return {{"status", "ok"}, {"module_id", id}, {"snapshot", m->snapshot_state()}};
                }
                if (verb == "module_subscribe_diag") {
                    const std::string id = req.value("id", std::string());
                    const std::string topic = req.value("topic", std::string());
                    const double hz = req.value("hz", 30.0);
                    if (!instance_.module(id)) return {{"status", "error"}, {"message", "unknown module: " + id}};
                    const int sub_id = diag_->subscribe(id, topic, hz);
                    return {{"status", "ok"}, {"sub_id", sub_id}, {"diag_port", diag_->port()},
                            {"topic_prefix", "diag." + std::to_string(sub_id) + "."}};
                }
                if (verb == "unsubscribe") {
                    diag_->unsubscribe(req.value("sub_id", 0));
                    return {{"status", "ok"}};
                }
                if (verb == "set_param") {
                    // Live hot-mutation of a scalar param, through the same path the
                    // host's learning freeze uses.
                    const std::string id = req.value("id", std::string());
                    const std::string key = req.value("key", std::string());
                    auto* m = instance_.module(id);
                    if (!m) return {{"status", "error"}, {"message", "unknown module: " + id}};
                    if (!req.contains("value")) return {{"status", "error"}, {"message", "set_param requires 'value'"}};
                    const auto& jv = req["value"];
                    ogma::ParamValue v;
                    if (jv.is_boolean()) v = jv.get<bool>();
                    else if (jv.is_number_integer()) v = int64_t(jv.get<int64_t>());
                    else if (jv.is_number()) v = jv.get<double>();
                    else if (jv.is_string()) v = jv.get<std::string>();
                    else return {{"status", "error"}, {"message", "set_param: unsupported value type"}};
                    m->on_param_change(key, v);
                    return {{"status", "ok"}};
                }
                return {{"status", "error"}, {"message", "unknown verb: " + verb}};
            } catch (const std::exception& e) {
                return {{"status", "error"}, {"message", e.what()}};
            }
        });
        control_->start();
        active_ = true;
        std::fprintf(stderr, "inspector: control tcp://127.0.0.1:%u  diag tcp://127.0.0.1:%u\n", control_port, diag_port);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "inspector: %s — running without an inspector\n", e.what());
        control_.reset();
        diag_.reset();
        active_ = false;
    }
}

InspectorSurface::~InspectorSurface() {
    if (control_) control_->stop();
    if (diag_) diag_->stop();
}

void InspectorSurface::publish_tick(uint64_t tick_id) {
    if (diag_) diag_->publish_tick(tick_id, instance_);
}

}  // namespace mjhost
