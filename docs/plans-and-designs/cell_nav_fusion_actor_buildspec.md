# Build Spec — Cell Nav: Fusion + Actor Rebuild (toward ignition)

**Status:** APPROVED design (2026-06-18/19 discussion). Not yet built.
**Branch:** `gait-ignition-homeokinetic`. **Gate metric:** board-clearing (no respawn).
**Discipline:** cheapest-discriminating-experiment first; gate each step on board-clearing
(brain-on vs ablated floor, n≥10 paired seeds, `scripts/nav_signal_agg.py`) BEFORE stacking
the next. Verify the *consumer* changes (consensus modality flips, body maneuvers differently),
not just internal scores. Do not bundle changes — attribution dies if we do.

---

## Why (the problem, in one line)
On board-clearing, the brain shows only a **weak, borderline directed-foraging edge**
(n=20: clears ~4.0/12 vs ablated floor ~2.75, Welch t=1.53, **NS**). The loop clears the
local cluster then can't reliably reach the rest. Root causes are now **verified**, not assumed.

## Verified evidence (the ground we build on)
1. **`epm_bearing` already resolves near/far × left/right.** Per-sample winner reconstruction:
   its nodes span `scent.max` 0.78 (far) → 4.54 (near) and bearing centered → hard-left. The
   approach coordinate (heading + magnitude) is *already computed* — `ScentCompass(emit_proximity)`
   emits `[cx, cy, prox]` and `epm_bearing` (proprio_state_dims=3) encodes all three.
2. **The voter discards it.** Consensus is winner-take-all by trust; `voter_nav.mod` = `color`
   (green) **98–100%** of ticks (seed5 118/121, seed6 121/121). The actor's state is the green
   channel; `epm_bearing` almost never wins. Trust: bearing 0.18–0.40 vs green 0.50–0.82.
3. **Green is NOT junk — it's the long-range forward signal, but it's being flattened.** `green_frac`
   is the FPV camera's "food in the cone ahead," graded 0 → 0.29 (mean 0.03). `epm_green` crushes
   it into **2 overlapping nodes** (node0 [0,0.135], node1 [0,0.287] — they don't even cleanly
   threshold presence). Cause: `green_frac` magnitude is tiny vs the EPM's RBF vigilance.
4. **The flattening is *why* green wins trust (precision-trap).** Flattened green is near-constant →
   TLE 0.013 → `1/(TLE+ε)` hands it ~0.72 trust. A channel flattened into predictable near-noise
   dominates the consensus *because* it's been flattened.
5. **Falsified levers (don't repeat):** turn-actor grounded on `cy` (short-range scent forward
   component) — hurt; epistemic-gain boost — no help. (`cy` failed because it grounds the turn on
   the channel that *can't* see far; the long-range channel is *vision*.)

## North-star framing
- Trust = `1/(TLE+ε)` is precision-weighting; it rewards **predictability**, which is orthogonal
  to **informativeness**. Fast/reactive path = precision (correct). Slow/cognitive path = should
  weight by **information** (discrimination). Winner-take-all is the degenerate `argmax` of
  precision-weighted fusion; trust-weighted summing is what predictive coding / HDC bundling do.
- Scent (short-range, lateral) and resolved green (long-range, food-ahead) are **complementary** →
  combine, don't discard.

---

## The build (sequenced; gate each)

### Step 0 — Pluggable voter (enabling change; default = no behavior change)
**File:** `cpp_core/{include,src}/ogma/modules/LateralVoter.{hpp,cpp}`
- Add `trust_mode` ∈ {`precision`, `info_precision`} — default `precision`.
- Add `fusion_mode` ∈ {`winner_take_all`, `trust_weighted_sum`} — default `winner_take_all`.
- Defaults reproduce current behavior **byte-identical** → picrawler & all fast-path voters untouched.
- Rebuild GDExtension (`cmake --build godot_host/build`).

### Step 1 — Information-weighted trust + resolve green (the verified unlock)
**1a. `info_precision` trust:** `trust_i ∝ H(winner_dist_i) / (TLE_i + ε)`, where `H` is the
Shannon entropy of modality *i*'s winner distribution over a sliding window (~100 samples). A
channel must be **both discriminative (high H) and reliable (low TLE)** to win. Demotes flattened
green (low H) and noise (high TLE); promotes the varied-but-reliable bearing.

**1b. Resolve green:** make `epm_green` actually grade the signal instead of 2 overlapping nodes.
Cheapest: scale/transform the published `green_fraction` (e.g. `sqrt` or ×scale so its 0–0.29 range
matches the EPM's vigilance) — OR lower `epm_green` `min_insertion_error` + raise `max_nodes`.
Target: ≥3–4 nodes that separate by `green_frac` level (none/faint/strong-ahead). Then when green
earns trust it carries signal, not flattened noise.

**Config:** `voter_nav.trust_mode = info_precision`; green-resolution fix on `epm_green` /
the body's `green_fraction` publish. Keep `fusion_mode = winner_take_all` for this step (with
`info_precision`, bearing should win the consensus).

**ACCEPTANCE (consumer check):** re-run the per-sample winner audit —
`voter_nav.mod` should stop being pinned to flattened green; the actor's consensus should reflect
the bearing's near/far × left/right (and a *graded* green once resolved).
**GATE:** board-clearing n≥10 vs floor — beats the n=20 baseline edge (clears >> ~4/12)?

### Step 2 — Combine via trust-weighted sum + meta-EPM (once both channels are good)
**Rationale:** with green resolved, scent-bearing and green-ahead are genuinely complementary →
winner-take-all forces a bad either/or. Switch to summing.
- `fusion_mode = trust_weighted_sum` → fused latent = `Σ trust_i · latent_i` in the common
  RP/HDC space (the voter already emits a `dim:16` fused latent — verify it's the trust-weighted sum).
- Add a **meta-EPM** on the fused latent ("meta-EPM after voting") to re-quantize the continuous
  blend → discrete state → actor reads via `proprio_topic` (consensus-optional already supports this).
- Now both bearing (left/right, near/far) AND graded green-ahead reach the actor.
**Caveat:** blending heterogeneous channels can blur; the meta-EPM must produce a *discriminative*
state, not mush — check its node spread. **GATE:** board-clearing vs Step 1.

### Step 3 — Motor: shallow multi-step planning (equal attention to the actor)
**File:** `cpp_core/.../ActionDecoder.{hpp,cpp}`
- Extend the EFE pragmatic from **1-step → 2–3 step rollout** over the forward model, scoring
  *cumulative* predicted homeostatic improvement: roll `P(s'|s,a)` forward H steps, accumulate
  `obs_value[s']` (scent magnitude + graded green-ahead). New `plan_horizon` param (default 1 =
  legacy). The state now carries an approach coordinate (Step 1/2), so planning is meaningful —
  "turn → approach → eat" finally has visible value (fixes the 1-step myopia).
- (Optional, same pass or next) **joint turn+thrust action** (2-D action / 9 bins, joint forward
  model) so coordinated curves emerge instead of two independent channels.
- Model-based only (no REINFORCE — it collapses 4/5). **GATE:** beats the 1-step actor on board-clearing?

### Step 4 — VisionCompass (long-range *directional* channel)
- Green **centroid** (horizontal centroid of green pixels in the FPV frame) → a green *bearing*
  (ahead-left vs ahead-right), the long-range analog of `ScentCompass`. New module
  `VisionCompass` (or extend the green pathway in `_publish_video_frame`). Feed as a directional
  channel into the (info-weighted, weighted-sum) fusion.
- **Principles-pure version (later):** a vision EPM that *infers* the directional latent from the
  raw frame (dual-TLE = inference + its own error → bar-a). The hand-coded centroid is the scaffold.
- **GATE:** board-clearing, especially the spread/distant-food arena (where long-range matters most).

### Step 5 — Deferred (only if needed)
Descending predictor (residual encoding → sharper, more informative embeddings; needs the top-down
pathway re-wired); CPG clock (gait-specific — only if we add temporal abstraction / clocked
exploration); latent/continuous actor planning in the EPM's own predictor (the JEPA endpoint that
unifies perception ↔ motor).

---

## Principles posture
- Hand-coded compasses (scent, vision) are perception scaffolds carrying the **same bar-(a) caveat**
  (heading is computed, not inferred) — no *new* violation; the EPM-inferred version is the pure path.
  The defensible-AI content is the **actor learning to act (bar-b)**, not the heading computation.
- `info_precision` + `trust_weighted_sum` are *more* aligned with active-inference precision-weighting
  than winner-take-all, not less.
- No reward shaping (intrinsic cost only); model-based planning (collapse-resistant); opt-in params
  keep picrawler / fast paths byte-identical.

## Sequencing rationale
Step 1 is the cheapest, highest-information move — it's the **verified** unlock (the data already
shows the state is computed and discarded). It needs no new sensor and no actor change, so if
board-clearing improves we've *proven* fusion was the bottleneck. Steps 2–4 then layer combination,
planning, and long-range vision, each gated so we keep attribution.
