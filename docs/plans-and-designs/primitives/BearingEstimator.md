# BearingEstimator — Primitive Contract

**Lineage:** added 2026-06-21 as Stage 3 of the Cell de-scaffold — perception-as-inference
(replace the hand-coded `ScentCompass` vector-sum with a learned percept). EPM-lineage
(GNG/RBF-style vector quantization + dual-TLE), in-runtime (non-backprop).
**Header:** `cpp_core/include/ogma/modules/BearingEstimator.hpp`
**Impl:** `cpp_core/src/ogma/modules/BearingEstimator.cpp`

---

## Purpose

`ScentCompass` reduces the raw 8-nostril ring to a bearing `[cx, cy]` via a **hand-coded**
weighted vector-sum (the nostril geometry is hard-set). BearingEstimator **de-scaffolds** that:
it learns the percept as a model with its own error, and the `HeadingController` follows the
**inferred** bearing — advancing **bar-(a)** ("perception is an inferred latent, not an analytic
given").

The de-scaffold is verified by the **lesion**: BearingEstimator learns while the analytic compass
**teaches** it, then the teacher is dropped. If foraging continues on the learned percept alone,
the hand-coded vector-sum is gone from the runtime.

> This is the **in-runtime (GNG-lineage)** version. It is *not* a learned ring→bearing-from-
> egomotion model (which would need backprop / the PyTorch→ONNX path *and* the action-consequence
> loop). See "Findings" for why that distinction is load-bearing.

---

## The mechanism

### 1. Online vector quantization of the raw ring (self-supervised)

Cluster the raw nostril ring (`reality.proprio.scent`, ~8 dims) into a growing set of **prototypes**
(centroids in ring-space):

- Each tick, find the nearest prototype (L2). If the nearest is farther than `novelty_thresh`
  (and below `max_prototypes`), **grow** a new prototype = the current ring.
- Move the winner toward the ring: `proto += proto_lr · (ring − proto)`.
- This is **purely self-supervised** (no teacher) — it tracks the distribution of ring patterns
  the bug experiences, and keeps adapting/growing **even when lesioned**.
- **TLE** = the winner's reconstruction distance = the inference error.

### 2. Per-prototype bearing readout, distilled from the teacher

Each prototype carries a bearing `[cx, cy]`, learned by EMA toward the analytic compass while the
teacher is present:

```
proto_cx += bearing_lr · (teacher_cx − proto_cx)   (likewise cy)
```

The **inferred bearing** = the winning prototype's readout (+ a proximity proxy = scaled ring sum,
teacher-independent). So the bug acts on its **learned categorical belief** "this ring-pattern
means food in direction d" — denoised over experience — rather than the raw analytic formula.

### 3. The lesion (the de-scaffold test)

After `lesion_after_ticks` (or immediately if `force_lesion`), the teacher is dropped and the
bearing readouts **freeze**, but the **VQ keeps adapting** (self-supervised → stays covered as the
bug moves). A prototype grown post-lesion **inherits** the nearest existing prototype's bearing
(so it is approximately right, not blank). The percept is then **teacher-free**: frozen bearings +
self-supervised clustering.

---

## Input Topics

| Topic (default) | Kind | Payload | Required | Notes |
|---|---|---|---|---|
| `ring_topic` = `reality.proprio.scent` | Direct | `ProprioToken` (8-nostril ring) | yes | The raw percept to quantize. |
| `teacher_topic` = `percept.scent_compass` | Direct | `ProprioToken` `[cx, cy, (prox)]` | distillation only | Analytic bearing used as the readout target (pre-lesion). Empty / `force_lesion` = no teacher ever (hard ablation). |

## Output Topics

| Topic (default) | Payload | Cadence |
|---|---|---|
| `output_topic` = `percept.bearing_inferred` | `ProprioToken` `[cx, cy, prox]` | every tick |

→ `MotivationGate` / `HeadingController` (replaces `percept.scent_compass` in the control path).

---

## Parameter Schema

| Key | Mutability | Default | Description |
|---|---|---|---|
| `ring_topic` | ConstructionOnly | `reality.proprio.scent` | Raw nostril ring (quantized). |
| `teacher_topic` | ConstructionOnly | `percept.scent_compass` | Distillation target; empty = no teacher. |
| `output_topic` | ConstructionOnly | `percept.bearing_inferred` | Inferred bearing out. |
| `max_prototypes` | ConstructionOnly | 24 (configs use 40) | Cap on learned ring prototypes. |
| `novelty_thresh` | HotMutable | 0.05 | L2 distance above which a novel ring grows a prototype. |
| `proto_lr` | HotMutable | 0.05 | VQ update rate (winner → ring). |
| `bearing_lr` | HotMutable | 0.05 | EMA rate for the readout toward the teacher. |
| `lesion_after_ticks` | HotMutable | −1 | ≥0 → freeze readouts + drop the teacher after N ticks (keep VQ adapting). <0 = keep learning. |
| `force_lesion` | ConstructionOnly | false | true → no teacher from the start (hard ablation). |

---

## Diagnostics

`n_proto` (learned prototypes), `winner`, `tle` (reconstruction error), `cx` / `cy` (inferred
bearing), `lesioned`.

---

## Invariants

1. Publishes exactly one inferred bearing per tick.
2. The VQ clustering is self-supervised — it adapts/grows regardless of `lesioned`.
3. The bearing readout is distilled **only** while not lesioned **and** a teacher is present;
   post-lesion the readouts are frozen (teacher-free) and new prototypes inherit the nearest
   bearing.
4. Lesioning the estimator (or `force_lesion`) lets the caller fall back to the analytic compass
   gracefully (it is still a separate module in the graph).

---

## Status / findings (turn rig, n=3) — **partial de-scaffold; teacher is irreducible here**

- **With the teacher present**, the distillation tracks the analytic excellently
  (`corr(inferred, true-food) = 0.87–0.99`) and the inferred bearing **drives control** as well as
  the analytic (foraging ≈ the analytic baseline). So the control path genuinely runs on a learned
  percept — bar-(a) advanced.
- **At the lesion, foraging collapses** (even on the stable hand-gate: ~130 eats pre-lesion →
  ~5 post). Root cause: the bearing readout has **no self-supervised learning signal** in the open
  arena — a quantized percept is lossy vs the continuous analytic → it drifts → the bug strays into
  ring-patterns whose frozen/inherited bearings are stale → drift compounds → collapse. The teacher
  continuously corrects this; remove it and the percept can't self-correct.
- **Conclusion:** genuine *teacher-free* perception-as-inference, like nav heading-selection,
  **cannot be de-scaffolded in the open arena** — both need a non-teacher learning signal
  (egomotion-prediction or action-consequence), which is the **maze's** richness. The achievable
  open-arena result is the partial de-scaffold (control on a learned percept, analytic demoted to
  an irreducible teacher).

Configs: `the_cell_corridor_turn_heading_infer.json` (on learned advance),
`…_infer_hg.json` (on the stable hand-gate — the clean isolation). Analyzer:
`scripts/head_infer_analyze.py`. See `docs/plans-and-designs/cell_descaffolding_plan.md` (Stage 3).
