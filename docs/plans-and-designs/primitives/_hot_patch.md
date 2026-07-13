# Cross-Cutting: Hot-Patch Transactional Semantics

> **⚠️ PARTIALLY SUPERSEDED (2026-06-17).** The ConnectOp/DisconnectOp description below treats
> them as mutating bus *subscription records*. The effective implementation is module-side
> **aux-send allowlist gating** — see **`_aux_send_routing.md`**. Consequence: a declared
> ConnectOp edge can look connected in the graph while the receiver never subscribes, so the
> edge is behaviorally dead. Read `_aux_send_routing.md` for the authoritative routing model;
> the transactional/rollback semantics below remain valid.

**Applies to:** the Scheduler implementation and every module's `on_param_change()`.

---

## The Three Op Classes

`GraphPatchOp` is a `std::variant` covering five operations grouped into three classes by when they take effect:

| Class | Ops | Apply timing | Cost |
|---|---|---|---|
| **Topology mutation** | `AddNodeOp`, `RemoveNodeOp`, `ConnectOp`, `DisconnectOp` | Between-tick boundary (after `Bus::end_tick()`, before next `Bus::begin_tick()`). | Rebuilds the dispatch slot table for affected topics; rebuilds the DAG levelization. |
| **Parameter mutation (hot-mutable)** | `SetParamOp` where `target.params_schema()[key].mutability == HotMutable` | Between-tick boundary, but does not require DAG rebuild — module's own `on_param_change()` runs in-place. | One module's internal state update. |
| **Parameter mutation (construction-only)** | `SetParamOp` where mutability is `ConstructionOnly` | **Rejected at validation time** — must be a `RemoveNodeOp` + `AddNodeOp` pair instead. | n/a |

The "between-tick boundary" rule is non-negotiable: mid-tick application would race with the parallel-level execution model. The Scheduler enqueues ops as they arrive; `Scheduler::tick()` applies them at the start of the next tick before `Bus::begin_tick()` runs.

---

## Transactional Semantics

A `GraphPatchBatch` is the unit of atomicity. The Scheduler:

1. Validates every op in the batch against the current graph state and module schemas.
2. If any op fails validation, the entire batch is rejected; no state changes; the caller's `BatchId` is reported as failed.
3. If all ops validate, they apply in order. Mid-batch failures (which should not happen if validation was thorough) result in the partial state being rolled back to the pre-batch snapshot.

A batch is the right granule for compound operations like:

```
batch = [
    AddNodeOp{ id="epm_l1", type="EPM", params={ input_topic: "consensus.0", ... } },
    ConnectOp{ from="voter_0", to="epm_l1" },
    ConnectOp{ from="epm_l1", to="voter_1" },
    SetParamOp{ target_id="voter_1", key="input_pattern", value="consensus.1." }
]
```

Either all four apply, producing a coherent Level-1 EPM wired correctly, or none apply, leaving the OgmaInstance in its prior state.

---

## Validation Rules

For every op, the Scheduler checks:

### `AddNodeOp`

- `spec.id` is unique within the OgmaInstance.
- `spec.type` is registered in `ModuleRegistry`.
- `spec.params` validates against the registered factory's `ParamSchema`:
  - Every required key (no default value) is present.
  - Every present key has the correct variant type.
  - Numeric values within `[min, max]` if specified.
- After insertion, the resulting graph remains DAG-acyclic (excluding feedback edges) and every output topic of the new module declared in `output_topics()` doesn't conflict with another module's output topic (single-producer rule).

### `RemoveNodeOp`

- `id` exists.
- No remaining module has a `required = true` subscription whose only producer was the removed module — the removal cannot starve a downstream module of a required input. (If you want to remove the producer first then the consumer, send both ops in one batch.)

### `ConnectOp`

- `from` and `to` resolve to known modules or `host:<topic>`.
- The source module's `output_topics()` includes `topic` (or it's derived from the source's declared output if `topic` is empty).
- The destination module's `input_topics()` includes a pattern matching `topic`.
- After insertion, the graph remains DAG-acyclic excluding feedback edges, OR `feedback = true` is set.

### `DisconnectOp`

- The named edge exists.
- After removal, the destination module is not left without any producer for a `required = true` subscription.

### `SetParamOp`

- `target_id` exists.
- `key` is in the module's `ParamSchema`.
- `mutability == HotMutable` for runtime patches. (Note: `ConstructionOnly` params are also settable when the op is part of an `AddNodeOp` payload — that's the construction path, not the patch path.)
- `value` matches the variant type and range from the schema.

---

## Apply Phase

For each op, in order:

### `AddNodeOp`

1. `ModuleRegistry::create(spec.type)` produces a `ModulePtr`.
2. `module->set_id(spec.id)`.
3. `module->on_setup(bus_, spec.params)` — module subscribes its topics here. Bus subscriptions register synchronously.
4. Module is added to the registry. Scheduler rebuilds the DAG and re-levels.

### `RemoveNodeOp`

1. `module->on_teardown()`.
2. All Bus subscriptions registered by this module are unregistered.
3. All edges incident to the module are removed.
4. Module is dropped (its destructor runs).
5. Scheduler rebuilds the DAG and re-levels.

### `ConnectOp` / `DisconnectOp`

1. The Bus's subscription record for the affected topic is mutated.
2. The DAG is updated; levelization changes if topology shape changed.

### `SetParamOp`

1. The module's `on_param_change(key, value)` is called.
2. The module updates its internal state. No Bus or DAG changes.

---

## Cost of DAG Rebuild

Naively, every topology mutation rebuilds the DAG from scratch (O(modules + edges)). For a typical OgmaInstance with ~20 modules and ~40 edges, this is sub-millisecond — acceptable as an inter-tick operation. Phase 3+ MAY introduce incremental DAG mutation if profiling shows it matters; not a Phase 0 concern.

The Bus's dispatch-slot table is rebuilt for affected topics only, not globally. A `ConnectOp` adds one entry to one topic's subscriber list; `RemoveNodeOp` walks the module's declared topics and removes the corresponding entries.

---

## Logging and Telemetry

Every applied batch produces a log entry:

```
[hot_patch] batch_id=42 source="mitosis" ops=4 status=applied levels_before=5 levels_after=6
```

Failed batches log validation errors verbosely for debugging. The host (Debug Host, Godot Host, HAL Host) can surface these to a UI or telemetry sink.

---

## What is NOT a Hot Patch

- **GNG node birth/death.** When a GNG bakes, prunes, or mitosis-splits a node, that's internal module state, not a graph patch. The `RealityToken.pruned_ids` field broadcasts the eviction; consumers (ActionDecoder valence map, GNGRollout cache, etc.) handle it on their own.
- **Hebbian table updates.** Pure module-internal state.
- **Module-internal config that isn't in `ParamSchema`.** If it's not in the schema, it's not patchable. (If it should be, amend the contract.)

The hot-patch API is the topology + declared-parameter mutation surface. Everything else is internal module evolution and stays internal.

---

## Phase 0 Deliverable

This doc + the `GraphPatchOp` variant in `cpp_core/include/ogma/GraphConfig.hpp` (already shipped in Section B). Phase 1 implementation lives in the Scheduler.

Phase 3's exit gate (per `v4_refactor.md`) requires: "Module graph hot-patch API ... validated against a running OgmaInstance." The validation suite tests:

- Every `GraphPatchOp` variant against a populated OgmaInstance.
- Batch atomicity: a batch with one bad op leaves state untouched.
- Concurrent submission: two patches enqueued in the same tick apply in submission order at the next inter-tick boundary.
- The `mitosis_gatekeeper` and a stub `claude_agent` both successfully mutate the graph through the same API surface (Pillar 2 critical rule).
