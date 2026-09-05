#pragma once
// Dry run: construct the brain for real (OgmaInstance on an in-process bus),
// feed it synthetic body input shaped by the manifest, tick N times, and
// report what came out.  This is the same exercise cpp_core's
// test_clone_shipping_configs performs on every shipped config.
#include <string>
#include <vector>

#include "Body.hpp"
#include "Graph.hpp"
#include "Wiring.hpp"

namespace bb {

struct DryRunReport {
    bool        constructed = false;
    std::string error;                 // construction or tick failure
    int         ticks_done  = 0;
    double      construct_ms = 0, tick_ms = 0;
    std::vector<std::string> fed;              // body topics synthesised each tick
    std::vector<std::string> published;        // module outputs holding a value at the end
    std::vector<std::string> silent;           // module outputs never published
    std::vector<std::string> actions_seen;     // body sinks that received a value
    std::vector<std::string> actions_missing;  // body sinks that did not
};

DryRunReport dry_run(Graph const& g, Wiring const& w, Body const* body, int ticks);

} // namespace bb
