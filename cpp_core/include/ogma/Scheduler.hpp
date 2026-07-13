#pragma once

// =============================================================================
// Scheduler.hpp  --  Tick-driven DAG executor for an OgmaInstance
// =============================================================================
//
// The Scheduler owns the module DAG built from a GraphConfig.  Every tick it
// runs every module exactly once, with modules in the same DAG level executing
// in parallel across the OgmaInstance's thread pool.
//
// DAG construction
// ----------------
//
//   1. Build a directed graph with one node per Module and one edge per
//      EdgeSpec where `feedback == false`.  Feedback edges are recorded
//      separately and used only to register Bus::SubscriptionKind::Feedback
//      subscriptions; they do NOT participate in topological ordering.
//
//   2. Topologically sort the DAG.  If the non-feedback subgraph contains a
//      cycle, construction fails with a message naming the cycle — the graph
//      author is required to mark a back-edge as `feedback: true` to break it.
//
//   3. Group modules into LEVELS: level 0 contains all modules with no
//      non-feedback predecessors; level k contains modules whose predecessors
//      all sit at levels < k.  Modules in the same level run concurrently.
//
// Tick lifecycle
// --------------
//
//   tick(t):
//     bus_->begin_tick(t)             // rotates last-value cache; Feedback
//                                     // subscribers will be invoked with t-1.
//     for level in 0..N:
//         parallel_for module in level:
//             module->tick(t)         // module reads Bus, writes Bus
//         barrier                     // wait for all parallel ticks to finish
//         bus_->end_level()           // any per-level Bus housekeeping
//     bus_->end_tick()                // flush telemetry, run hot-patch queue
//
// Hot-patch ops are applied AFTER end_tick() and BEFORE the next begin_tick().
// This guarantees graph topology is stable for the entire duration of a tick.
//
// Restartability scope (Pillar 1)
// -------------------------------
//
// Under InProcessBus, a fatal exception escaping a Module::tick() is fatal to
// the OgmaInstance.  Other instances in the same host process are unaffected
// (instance-level fault isolation).  The Scheduler does NOT install per-module
// exception fences on the hot path — the cost outweighs the benefit on a Pi5
// target.  ZmqBus deployments have a separate restartability story (see
// docs/v4_refactor.md Pillar 1 Design Rules).

#include <cstdint>
#include <memory>
#include <vector>

#include "ogma/Bus.hpp"
#include "ogma/GraphConfig.hpp"
#include "ogma/Module.hpp"

namespace ogma {

class ThreadPool;   // implementation detail, defined in src/ogma/

class Scheduler {
public:
    virtual ~Scheduler() = default;

    // Run exactly one global tick (begin_tick → all levels → end_tick →
    // hot-patch queue).  Blocks until every module has finished.
    virtual void tick() = 0;

    // Current global tick counter (incremented after each tick() call).
    virtual uint64_t current_tick() const = 0;

    // -------------------------------------------------------------------------
    // Hot-patch API
    // -------------------------------------------------------------------------
    //
    // Operations enqueued during a tick are applied between ticks.  Returns a
    // batch ID so callers can correlate a patch submission with a later
    // success/failure callback.  Validation runs at apply time, not enqueue
    // time, so a stale add_node referring to a module type unregistered after
    // submission will fail.

    using BatchId = uint64_t;
    virtual BatchId enqueue_hot_patch(GraphPatchBatch batch) = 0;

    // -------------------------------------------------------------------------
    // Introspection
    // -------------------------------------------------------------------------

    virtual std::vector<std::vector<std::string>> levels() const = 0;
    virtual std::vector<EdgeSpec>                  feedback_edges() const = 0;

    // All edges currently tracked by the scheduler — boot-config edges plus
    // the cumulative effect of hot-patch ConnectOp/DisconnectOp.  Used by
    // UI/inspection paths so manual connections drawn in the live graph
    // panel persist across the periodic repopulate.
    //
    // Note: ConnectOp/DisconnectOp now drive each receiver module's
    // input gate (per-primitive aux-send filter) when auto_subscribe
    // is false.  See Module::input_allowed for the routing semantics.
    virtual std::vector<EdgeSpec>                  edges() const = 0;

    // -------------------------------------------------------------------------
    // Per-graph routing mode (UI-dev manual-routing feature)
    // -------------------------------------------------------------------------
    //
    // Toggle the input gate on every live module.  When auto_subscribe is
    // true (default), modules' input_default_deny_ flag is cleared so any
    // matched delivery passes — preserves back-compat behaviour for
    // every shipping config.  When false, modules' default-deny is set
    // and their allowlists are rebuilt from the current edge list.
    //
    // Round-trips through GraphConfig.runtime.auto_subscribe so saved
    // topologies preserve their wiring intent.
    virtual void  set_auto_subscribe(bool enabled) = 0;
    virtual bool  is_auto_subscribe() const        = 0;
};

using SchedulerPtr = std::unique_ptr<Scheduler>;

} // namespace ogma
