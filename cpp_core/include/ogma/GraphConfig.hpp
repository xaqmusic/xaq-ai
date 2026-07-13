#pragma once

// =============================================================================
// GraphConfig.hpp  --  Parsed module graph + diff-patch API
// =============================================================================
//
// The JSON graph is the authoring surface for v4.  A human edits it via the
// Phase-5 Godot GraphEdit dock; the mitosis gatekeeper and external Claude
// agents mutate it via the diff-patch API; the Runtime constructs an
// OgmaInstance from it.
//
// JSON schema (v1)
// ----------------
//
//   {
//     "version": 1,
//     "runtime": {
//       "thread_pool":      "per_instance" | "shared",   // default per_instance
//       "num_threads":      <int>                         // 0 = auto
//     },
//     "modules": [
//       {
//         "id":     "<unique within graph>",
//         "type":   "<ModuleRegistry-known type name>",
//         "params": { "<key>": <value>, ... }
//       },
//       ...
//     ],
//     "edges": [
//       {
//         "from":     "<source module id> | host:<topic>",
//         "to":       "<sink module id>   | host:<topic>",
//         "topic":    "<optional explicit topic name>",
//         "feedback": false                              // default false
//       },
//       ...
//     ]
//   }
//
// Edge semantics
// --------------
//
// Edges are derived from each module's input_topics() / output_topics() lists,
// but the explicit `edges` array lets the graph author override or add wiring.
// A canonical edge connects one module's output topic to another module's
// input subscription — the Runtime translates `from`/`to` IDs into Bus
// subscriptions at construction time.
//
// Boundary edges (host I/O):
//   { "from": "host:reality.video.retinal",  "to": "epm_retinal" }
//   { "from": "action_decoder",              "to": "host:action.out" }
//
// `host:<topic>` indicates the topic is bridged to physical I/O by the host
// (Godot bridges sensors and actuators; HAL Host bridges I2C/GPIO; Debug Host
// bridges recorded streams).  The host wires those topics to its own publish/
// subscribe code; the module graph itself is unchanged across hosts.
//
// Feedback edges:
//   { "from": "neuro", "to": "epm_retinal", "feedback": true }
//
// The Scheduler excludes feedback edges from levelization (preventing cycles
// from breaking the topological sort) and the consumer's subscription on this
// edge is registered as SubscriptionKind::Feedback so it sees prior-tick values.
//
// Diff-patch API
// --------------
//
// Operations are applied transactionally between Scheduler ticks.  Rule
// (Pillar 2): the API is the only mutation surface; the mitosis gatekeeper,
// the v3-style launcher UI, and external Claude agents all funnel through it.

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "ogma/Module.hpp"

namespace ogma {

// -----------------------------------------------------------------------------
// Parsed graph representation
// -----------------------------------------------------------------------------

struct ModuleSpec {
    std::string id;
    std::string type;     // matches Module::type_name() of registered factory
    ParamMap    params;
};

struct EdgeSpec {
    std::string from;     // "<module_id>" | "host:<topic>"
    std::string to;       // "<module_id>" | "host:<topic>"
    std::string topic;    // optional override; empty = derive from declared topics
    bool        feedback = false;
};

enum class ThreadPoolPolicy {
    PerInstance,          // each OgmaInstance owns its own pool
    Shared,               // pool shared across all instances in the host process
};

struct RuntimeSpec {
    ThreadPoolPolicy thread_pool = ThreadPoolPolicy::PerInstance;
    int              num_threads = 0;     // 0 = auto (min(num_modules, num_cores))
    // Per-graph routing mode (UI-dev manual-routing feature).
    // true  = each module's input handlers admit any matching topic
    //         delivery (current bus-driven behaviour, back-compat default).
    // false = default-deny per-primitive gate; modules only ingest from
    //         producer ids that appear as `from` of an edge to them in
    //         the scheduler's edge list.  Drag-connecting / removing
    //         edges in patch mode mutates each receiver's allowlist and
    //         starts / stops actual data flow.
    bool             auto_subscribe = true;
};

struct GraphConfig {
    int                     version = 1;
    RuntimeSpec             runtime;
    std::vector<ModuleSpec> modules;
    std::vector<EdgeSpec>   edges;

    // Throws on parse failure with a human-readable message.  Validates:
    //   - schema version
    //   - module IDs unique
    //   - module types known to ModuleRegistry
    //   - edge endpoints reference declared modules or `host:<topic>`
    //   - param values match each module's ParamSchema
    static GraphConfig load_from_json(std::string_view json_text);
    static GraphConfig load_from_file(std::string_view path);

    std::string to_json() const;
};

// -----------------------------------------------------------------------------
// Diff-patch operations  (Pillar 2 mitosis API)
// -----------------------------------------------------------------------------
//
// Applied between ticks by Scheduler::process_hot_patch_queue().  Operations
// can be batched into a single transaction; on validation failure the entire
// batch is rejected (no partial application).

struct AddNodeOp {
    ModuleSpec spec;
};

struct RemoveNodeOp {
    std::string id;
};

struct ConnectOp {
    EdgeSpec edge;
};

struct DisconnectOp {
    std::string from;
    std::string to;
    std::string topic;   // optional, distinguishes parallel edges
};

struct SetParamOp {
    std::string target_id;
    std::string key;
    ParamValue  value;
};

using GraphPatchOp = std::variant<AddNodeOp,
                                  RemoveNodeOp,
                                  ConnectOp,
                                  DisconnectOp,
                                  SetParamOp>;

struct GraphPatchBatch {
    std::vector<GraphPatchOp> ops;
    std::string source;     // "mitosis" | "agent:<id>" | "ui" — for telemetry
};

} // namespace ogma
