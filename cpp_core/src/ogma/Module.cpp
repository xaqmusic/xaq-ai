// Module.cpp — base-class default implementations.
//
// Most defaults live in the header; this file exists so members that need
// the full nlohmann::json type (not just the forward-decl) have somewhere
// to be defined.  Modules with persistent state override these.

#include <nlohmann/json.hpp>

#include "ogma/Module.hpp"

namespace ogma {

nlohmann::json Module::snapshot_state() const {
    return nlohmann::json::object();
}

void Module::restore_state(nlohmann::json const&) {
    // Default: no-op.  Modules with state override.
}

nlohmann::json Module::diag_snapshot() const {
    // Default: same payload as the clone-ready snapshot.  Modules whose
    // snapshot grows unboundedly with run-time should override.
    return snapshot_state();
}

} // namespace ogma
