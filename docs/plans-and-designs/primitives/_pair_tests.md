# Cross-Cutting: Pair Test Matrix

**Applies to:** every Phase 1 primitive.

---

## Why a Matrix Doc

Each primitive contract names its own pair tests in its VV&A section. This index doc consolidates them so the Phase 1 acceptance gate has a single checklist, and so we can audit the seam coverage across the bath.

A **pair test** wires exactly two real Phase-1-implemented modules through a real `InProcessBus` (no mocks beyond the host-side input bridge) and validates the contract surface where they meet — message types, units, publish order, first-tick semantics. v4_refactor.md frames these as the catch-net for "contract drift that unit tests systematically miss."

Pair tests live under `cpp_core/tests/ogma/pair/`. Each test compiles to a separate GoogleTest binary so a failure points at one seam.

---

## The Matrix

|   | NeurochemState | EPM | LateralVoter | HomeostaticDrive | ActionDecoder | DescendingPredictor | SequenceGNG | GNGRollout | MotorRepertoire |
|---|---|---|---|---|---|---|---|---|---|
| **NeurochemState** | — | ✓ | — | — | — | — | — | — | — |
| **EPM** | ✓ | — | ✓ | — | — | ✓ | ✓ | — | — |
| **LateralVoter** | — | ✓ | — | ✓ | ✓ | — | — | — | — |
| **HomeostaticDrive** | — | — | ✓ | — | ✓ | — | — | — | — |
| **ActionDecoder** | — | — | ✓ | ✓ | — | — | — | ✓ | ✓ |
| **DescendingPredictor** | — | ✓ | — | — | — | — | — | — | — |
| **SequenceGNG** | — | ✓ | — | — | — | — | — | — | ✓ |
| **GNGRollout** | — | — | — | — | ✓ | — | — | — | — |
| **MotorRepertoire** | — | — | — | — | ✓ | — | ✓ | — | — |

Eight pair tests cover the complete seam set required by the Phase 1 dependency chain plus the back-half new primitives:

1. **`pair_neurochem_epm`** — the cycle. Verifies NeurochemState reads EPM(t-1) via Feedback while EPM reads NeurochemState(t) via Direct. (NeurochemState.md, EPM.md)
2. **`pair_epm_lateralvoter`** — modality-group parsing. Verifies LateralVoter extracts `<group>` from `reality.<group>.<modality>`. (EPM.md, LateralVoter.md)
3. **`pair_lateralvoter_homeostatic`** — when `novelty_satiation` channel is declared, verifies the EMA on `consensus.0.fused_tle`. (HomeostaticDrive.md)
4. **`pair_homeostatic_actiondecoder`** — drive-grounded EFE seam. (HomeostaticDrive.md, ActionDecoder.md)
5. **`pair_lateralvoter_actiondecoder`** — consensus → action. (LateralVoter.md, ActionDecoder.md)
6. **`pair_descendingpred_epm`** — top-down prediction subtraction. (DescendingPredictor.md, EPM.md)
7. **`pair_seqgng_epm`** — windowed motif clustering over a real EPM's winner stream. (SequenceGNG.md)
8. **`pair_gngrollout_actiondecoder`** — REQ/REP rollout for EFE epistemic value. (GNGRollout.md, ActionDecoder.md)
9. **`pair_motorrepertoire_actiondecoder`** — chunk dispatch round-trip. (MotorRepertoire.md, ActionDecoder.md)
10. **`pair_motorrepertoire_seqgng`** — chunk crystallization from action-stream motifs. (MotorRepertoire.md, SequenceGNG.md)

(The matrix shows checks both above and below the diagonal for symmetry; the pair test list is a deduplicated set.)

---

## What Each Pair Test Asserts

A pair test is **not** a behavioural test — that's what unit tests and the Phase 2 thin-slice replay do. A pair test asserts:

1. **Type compatibility.** Subscribing module receives exactly the payload type the publishing module promised, with all required fields populated.
2. **Topic name compatibility.** The publisher's actual topic name matches the subscriber's pattern (exact or trailing-dot prefix).
3. **Tick alignment.** Direct subscriptions deliver same-tick values; Feedback subscriptions deliver previous-tick values. Verified via `tick_id` field on the payload.
4. **First-tick semantics.** The subscriber handles a `last_value()` of nullptr correctly (uses defaults from `_first_tick.md`).
5. **Failure-mode survival.** When the publisher fails to produce (e.g. EPM bootstrap returns `winner_id = -1`), the subscriber does not throw, does not produce NaN.
6. **Determinism.** Two runs with the same `master_seed` and identical synthetic inputs produce bit-identical outputs from the pair.

---

## Pair Test Skeleton

A pair test is structured as:

```cpp
TEST(PairNeurochemEpm, CycleSemantics) {
    auto bus = std::make_unique<ogma::InProcessBus>();

    // Construct both modules with controlled params.
    auto neuro = std::make_unique<NeurochemState>();
    auto epm   = std::make_unique<EPM>();
    neuro->set_id("neuro");
    epm  ->set_id("epm_retinal");
    neuro->on_setup(bus.get(), neuro_params);
    epm  ->on_setup(bus.get(), epm_params);

    // Build a synthetic input stream.
    auto stream = make_synthetic_proprio_stream(/*seed=*/42, /*ticks=*/500);

    // Run the pair through a fixed input schedule.
    for (uint64_t t = 0; t < 500; ++t) {
        bus->begin_tick(t);
        bridge_host_inputs(bus.get(), stream[t]);
        // The Scheduler ordering for this pair: EPM runs first, neuro second.
        epm  ->tick(t);
        bus->end_level();
        neuro->tick(t);
        bus->end_level();
        bus->end_tick();

        assert_invariants(bus.get(), t);
    }
}
```

This skeleton + per-pair invariants + the deterministic input stream are what each pair test ships.

---

## Mocks Are Forbidden Beyond the Host Bridge

Per `v4_refactor.md` Phase 1: "Mocks are permitted only where the real neighbour does not yet exist (inevitable for the first primitive in the chain, NeurochemState). Mocks are replaced by the real neighbour the moment it is merged; mocks do not survive Phase 1."

The host bridge (the synthetic proprio / event stream) is not a mock — it's the same role the Godot Host or HAL Host plays in production. Anything inside the cellular bath, including the Bus, must be the real implementation.

The single exception during Phase 1 implementation is when a *brand-new* primitive's pair test would require all preceding primitives. The primitive's unit tests use mocks; its pair test waits until both real neighbours are merged. The Phase 1 dependency chain (`NeurochemState → EPM → LateralVoter → HomeostaticDrive → ActionDecoder → DescendingPredictor → SequenceGNG → GNGRollout → MotorRepertoire`) is built precisely so each pair test has both its neighbours available.

---

## Phase 1 Acceptance Gate

A primitive is admitted to Phase 2 thin-slice integration only after:

1. All its declared unit tests pass on three consecutive seeds.
2. Every pair test in this matrix that involves the primitive AS PUBLISHER OR SUBSCRIBER is green on CI.
3. Latency p99 budgets from the contract are met on Pi5.

The Phase 2 thin-slice replay harness (Section E) requires pair tests 1, 2, 3, 4, 5 (the Phase 2 thin-slice modules: NeurochemState + EPM + LateralVoter + HomeostaticDrive + ActionDecoder) to be green before the harness runs.
