#include "TrialSetup.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>

#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"

namespace bb {

void warm_registry() {
    (void)ogma::ModuleRegistry::instance().registered_types();
}

ogma::ParamMap param_map_from_json(nlohmann::json const& params) {
    nlohmann::json module = {{"id", "_"}, {"type", "_"}};
    module["params"] = params.is_object() ? params : nlohmann::json::object();
    nlohmann::json root = {{"version", 1}, {"modules", nlohmann::json::array({module})}};
    auto g = ogma::GraphConfig::load_from_json(root.dump());
    return g.modules.empty() ? ogma::ParamMap{} : g.modules.front().params;
}

nlohmann::json param_to_json(ogma::ParamValue const& v) {
    return std::visit([](auto const& x) -> nlohmann::json { return x; }, v);
}

TrialResult trial_setup(std::string const& type, std::string const& id,
                        ogma::ParamMap const& params) {
    TrialResult r;
    try {
        // The bus outlives the module: the module's teardown unsubscribes.
        ogma::InProcessBus bus;
        ogma::ModulePtr m = ogma::ModuleRegistry::instance().create(type);
        if (!m) { r.error = "unknown module type '" + type + "'"; return r; }
        m->set_id(id);
        m->on_setup(&bus, params);
        r.ports.inputs  = m->input_topics();
        r.ports.outputs = m->output_topics();
        m->on_teardown();
        r.ok = true;
    } catch (std::exception const& e) {
        r.error = e.what();
    } catch (...) {
        r.error = "non-standard exception in on_setup";
    }
    return r;
}

StdoutSilencer::StdoutSilencer() {
    std::fflush(stdout);
    std::cout.flush();
    saved_ = ::dup(1);
    int devnull = ::open("/dev/null", O_WRONLY);
    if (devnull >= 0) { ::dup2(devnull, 1); ::close(devnull); }
}

StdoutSilencer::~StdoutSilencer() {
    std::fflush(stdout);
    std::cout.flush();
    if (saved_ >= 0) { ::dup2(saved_, 1); ::close(saved_); }
}

} // namespace bb
