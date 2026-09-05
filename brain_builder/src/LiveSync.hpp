#pragma once
// The live link's edit model: the document is edited as usual; the diff
// between it and the modules the host holds becomes patch ops.  A change to
// a construction-only param cannot be patched — the module must be recreated
// (losing its learned state), which is the operator's call.
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "Catalogue.hpp"
#include "Graph.hpp"

namespace bb {

struct LiveOps {
    nlohmann::json           ops = nlohmann::json::array();   // sent as-is
    std::vector<std::string> recreate;                        // ids needing remove + add
    std::vector<std::string> notes;
};

// synced: the host's modules as [{id,type,params}] (JSON, not ordered).
LiveOps diff_for_live(Graph const& g, Catalogue const& cat, nlohmann::json const& synced);

// The remove + add pair for one module, from the document's current spec.
nlohmann::json recreate_ops(Graph const& g, std::string const& id);

// Merge the host's modules into the document (layout and metadata kept).
void adopt_live_modules(Graph& g, nlohmann::json const& config);

} // namespace bb
