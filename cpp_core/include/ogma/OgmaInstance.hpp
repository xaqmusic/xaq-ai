#pragma once

// =============================================================================
// OgmaInstance.hpp  --  The unit of instanceability
// =============================================================================
//
// One OgmaInstance owns:
//   - exactly one Bus
//   - exactly one Scheduler
//   - one set of Modules built from a GraphConfig
//   - per-instance state for every Module (GNG topologies, neurochem levels,
//     Hebbian tables, valence maps)
//   - optionally its own thread pool, or a handle to a host-shared pool
//
// Multiple OgmaInstances coexist in one host process with NO shared mutable
// state.  This is what enables:
//   - N-brain evolutionary experiments in a single Godot scene
//   - Octopus-like morphologies where each limb runs its own brain
//   - Multi-agent simulations
//   - Per-body mutated graph configs without global config races
//
// Construction
// ------------
//
// The constructor takes a GraphConfig and a Bus implementation.  It:
//   1. Instantiates every Module via ModuleRegistry.
//   2. Calls each Module::set_id() and Module::on_setup(bus, params).
//   3. Constructs the Scheduler from the config + module list + bus.
//
// Failure modes are construction-time exceptions: unknown module type,
// invalid params, cycle without feedback annotation, edge endpoint refers to
// non-existent module, etc.  A constructed instance is guaranteed runnable.
//
// Thread-pool policy
// ------------------
//
// Default: each instance owns its own pool sized to
// `min(num_modules, num_cores)`.  Hosts running >4 instances (Phase 6
// evolutionary scenarios with 64+ brains) should pass a shared pool to avoid
// hundreds of threads — see RuntimeSpec::thread_pool == Shared.

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "ogma/Bus.hpp"
#include "ogma/GraphConfig.hpp"
#include "ogma/Module.hpp"
#include "ogma/Scheduler.hpp"

namespace ogma {

class OgmaInstance {
public:
    // Builds the instance from `config` using `bus` as the message transport.
    // The instance does NOT take ownership of `bus`; the host (Godot Host,
    // HAL Host, Debug Host) owns the Bus and ensures it outlives the instance.
    OgmaInstance(GraphConfig config, std::unique_ptr<Bus> bus);
    ~OgmaInstance();

    OgmaInstance(OgmaInstance const&) = delete;
    OgmaInstance& operator=(OgmaInstance const&) = delete;

    // -------------------------------------------------------------------------
    // Tick advancement (delegate to the Scheduler)
    // -------------------------------------------------------------------------

    // Advance one global tick.  Blocking.
    void tick();

    // Number of completed ticks since construction.
    uint64_t tick_count() const;

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    Bus*       bus();
    Bus const* bus() const;
    Scheduler* scheduler();

    Module*       module(std::string_view id);
    Module const* module(std::string_view id) const;

    std::vector<Module*> modules();

    GraphConfig const& config() const;

    // -------------------------------------------------------------------------
    // Hot-patch convenience (forwards to Scheduler::enqueue_hot_patch)
    // -------------------------------------------------------------------------

    Scheduler::BatchId enqueue_hot_patch(GraphPatchBatch batch);

    // Live edge list as the Scheduler currently sees it: cumulative effect
    // of every applied ConnectOp/DisconnectOp.  Distinct from `config()
    // .edges` which is the immutable boot-time GraphConfig.  UI/inspection
    // paths typically want to merge both: boot edges describe the static
    // wiring intent, scheduler edges describe runtime augmentations.
    std::vector<EdgeSpec> current_edges() const;

    // -------------------------------------------------------------------------
    // Identity (Phase 6 — for cross-instance fitness reporting)
    // -------------------------------------------------------------------------

    void               set_label(std::string label);
    std::string const& label() const;

    // -------------------------------------------------------------------------
    // Snapshot / clone  (Phase 6.5.4 — instance branching)
    // -------------------------------------------------------------------------
    //
    // `snapshot_state()` serialises every module's working state plus the
    // bus's last-value cache into a JSON blob.  `restore_state(json)` applies
    // a previously-taken snapshot in-place — both instances must have been
    // constructed from byte-identical GraphConfigs (the snapshot does not
    // include the static config; that is GraphConfig's responsibility).
    //
    // `clone()` is the convenience method: it constructs a fresh InProcessBus,
    // a fresh OgmaInstance with the same config, copies the bus cache by
    // shared_ptr (cheap — Messages are immutable), then restores every
    // module's state.  The cloned instance is independent of the source: it
    // has its own bus subscriptions, its own RNG, its own future state.
    // Plasticity is unaffected — the clone learns normally as it ticks.
    //
    // Use cases:
    //   - "Train continuously, snapshot, run a benchmark on the clone, throw
    //     it away" — the live brain never pauses (Phase 6.5.4 frozen-eval
    //     methodology).
    //   - "Fork into N variants for evolutionary search" (Phase 6.3+).
    //   - "Snapshot before risky decision, replay if it goes badly"
    //     (debugging).
    //
    // Determinism contract: if both source and clone are ticked from the
    // same point with identical inputs, they produce IDENTICAL outputs for
    // every tick.  Caveats:
    //   - Scheduler tick counter starts at 0 in clone (source's tick_id is
    //     not propagated; see Scheduler::current_tick).
    //   - FrozenJLEncoder optical-flow state (prev_gray_) is not snapshotted
    //     — affects only EPMs configured for optical flow.
    nlohmann::json snapshot_state() const;
    void           restore_state(nlohmann::json const&);
    std::unique_ptr<OgmaInstance> clone() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

using OgmaInstancePtr = std::unique_ptr<OgmaInstance>;

} // namespace ogma
