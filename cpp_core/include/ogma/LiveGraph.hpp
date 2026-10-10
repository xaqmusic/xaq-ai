#pragma once
// LiveGraph: the running brain's graph as a document, kept current across
// hot patches, plus the logic behind the control-socket verbs that let a
// client (the brain builder) pull the graph and patch it while the brain
// runs.  Host-agnostic: the host owns the instance mutex and decides what
// runs under it — validate_offline() needs no lock, apply() and get_graph()
// do.
#include <cstdint>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "ogma/GraphConfig.hpp"

namespace ogma {

class OgmaInstance;

class LiveGraph {
public:
    // Seeds the ledger from the instance's boot config.  source_path is the
    // file the host loaded, reported to clients so they can open it for its
    // metadata and layout.
    LiveGraph(OgmaInstance& inst, std::string source_path);

    // Wire shape (the Godot panel's dictionary shape, as JSON):
    //   {"op":"add_node","id":..,"type":..,"params":{..}}
    //   {"op":"remove_node","id":..}
    //   {"op":"connect","from":..,"to":..,"topic":..,"feedback":bool}
    //   {"op":"disconnect","from":..,"to":..,"topic":..}
    //   {"op":"set_param","id":..,"key":..,"value":..}
    // Throws std::invalid_argument on a malformed op.
    static GraphPatchBatch batch_from_json(nlohmann::json const& ops, std::string const& source);

    // Trial-construct every AddNodeOp on a scratch bus.  No instance access,
    // so a host runs it without holding the tick mutex.  Returns the errors.
    static std::vector<std::string> validate_offline(GraphPatchBatch const& batch);

    // Under the instance lock: check ids and mutability against the live
    // graph, enqueue the batch (applied by the scheduler between ticks),
    // update the ledger, bump the version.  Throws std::invalid_argument.
    nlohmann::json apply(GraphPatchBatch batch);

    // Under the instance lock: a host that hot-mutates a param directly
    // (mj_host's set_param verb) records it so get_graph() stays true.
    void record_set_param(std::string const& id, std::string const& key, ParamValue const& value);

    // Under the instance lock: {"graph_version", "source_path", "config":
    // {version, runtime, modules[{id,type,params}], edges}, "edges": live}.
    nlohmann::json get_graph();

    uint64_t           version() const { return version_; }
    std::string const& source_path() const { return source_path_; }

private:
    OgmaInstance&           inst_;
    std::string             source_path_;
    std::vector<ModuleSpec> specs_;
    uint64_t                version_ = 0;

    void reconcile();   // drop ledger entries no longer live; adopt live modules the ledger lacks
};

} // namespace ogma
