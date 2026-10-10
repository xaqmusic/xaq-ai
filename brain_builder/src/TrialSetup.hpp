#pragma once
// Trial setup: construct a module against a scratch bus so its ports and its
// setup errors surface without a running brain.  This is the same idiom the
// scheduler uses to validate a hot patch (cpp_core/src/ogma/Scheduler.cpp,
// validate_add_against) and the only way to learn a module's input/output
// topics, which are set inside on_setup.
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"

namespace bb {

struct Ports {
    std::vector<ogma::TopicSpec> inputs;
    std::vector<ogma::TopicSpec> outputs;
};

struct TrialResult {
    bool        ok = false;
    std::string error;
    Ports       ports;
};

// Touch the registry once on the main thread: its first-call initialisation
// is not thread-safe, and the wiring model runs trial setups on a worker.
void warm_registry();

// JSON params → ParamMap using the core's own conversion (GraphConfig), so
// the builder never re-implements the numeric rules.
ogma::ParamMap  param_map_from_json(nlohmann::json const& params);
nlohmann::json  param_to_json(ogma::ParamValue const& v);

// Never throws; a failure is reported in `error`.
TrialResult trial_setup(std::string const& type, std::string const& id,
                        ogma::ParamMap const& params);

// Modules print diagnostics to stdout from on_setup.  CLI modes that print
// JSON to stdout wrap their trial setups in one of these.
class StdoutSilencer {
public:
    StdoutSilencer();
    ~StdoutSilencer();
    StdoutSilencer(StdoutSilencer const&) = delete;
    StdoutSilencer& operator=(StdoutSilencer const&) = delete;
private:
    int saved_ = -1;
};

} // namespace bb
