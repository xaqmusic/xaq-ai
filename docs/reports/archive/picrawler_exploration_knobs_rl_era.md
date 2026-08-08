> **ARCHIVE — ported from the pre-split `ami-ogma` repo, 2026-07-25.** ⚠️ **This documents the
> reward-shaped RL era of the picrawler**, which [`../../the-picrawler-detour.md`](../../the-picrawler-detour.md)
> disowns as the cautionary origin story. **Its individual mechanism verdicts do NOT transfer** to
> the current reward-free active-inference stack — different substrate, different objective,
> different baseline. It is kept as an honest record and because its **failure shapes and
> measurement lessons are permanently valuable** (those are distilled into
> [`../picrawler_lever_ledger.md`](../picrawler_lever_ledger.md) §7 and
> [`../../../CLAUDE.md`](../../../CLAUDE.md) §3.2). For the **current** verdicts, read the ledger.
> `ami_ogma`/`ogma`/`AMI-Ogma` == xaq.

# PiCrawler Stage C — Exploration-Knob A/B Report

**Date:** 2026-05-19
**Plan:** `~/.claude/plans/i-ve-observed-the-system-proud-lynx.md`
**Configs:**
- Baseline: `the_picrawler_stand_target_lr_symmetric.json` (Stage B default)
- C1 variant: `the_picrawler_stand_target_lr_sym_c1_epistemic.json` — `epistemic_gain=0.1` on every Premotor
- C2 variant: `the_picrawler_stand_target_lr_sym_c2_lambda.json` — `eligibility_lambda=0.5` on every Premotor
**Per-seed JSON:** `results/picrawler_C1_epistemic.json`, `results/picrawler_C2_lambda.json`

## Setup

All A/Bs at n=20 paired seeds (42–61), duration 1800 s sim, `OGMA_PICRAWLER_MC_PERIOD=1500`, `OGMA_PICRAWLER_LEG_SYMMETRY=lr_pairs`, `--turbo --parallel 4`. Curriculum disabled (Stage C anti-goal — teleport pollutes paired-seed comparison). Both arms identical `auto_reset_on_inversion=false`. C1 self-check Δ=0 verified before the real run (config loads, harness deterministic).

Stage C added two metrics to `picrawler_ab.py::PAIRED_METRICS`: `max_distance_from_origin` and `total_path_length` — Stage C's primary metrics since the variants target exploration coverage, not stability.

C3 (HomeokineticExploration instantiation) was deferred from this stage at planning time after the explore agent confirmed `kExplorationDirective` is consumed by `ActionGate`/`ActionDecoder` — neither of which sits in PiCrawler's per-DOF Premotor action chain. Wiring HomeokineticExploration into Premotor requires a ~30–50 line C++ patch (subscribe to the directive, override softmax-sampled intent when active). Stage C's "no C++ build" anti-goal pushes that work to a follow-on stage.

## Results

### C1 — Epistemic novelty bonus (`epistemic_gain=0.1`)

| Metric | Δμ | σ | p | 95% CI |
|---|---|---|---|---|
| Δy_max | +0.002 m | 0.021 | 0.63 | [−0.007, +0.011] |
| Δy_mean_late | −0.003 m | 0.043 | 0.74 | [−0.022, +0.016] |
| Δpct_below | +0.023 | 0.332 | 0.76 | [−0.122, +0.162] |
| Δpct_tilt | +0.053 | 0.273 | 0.39 | [−0.059, +0.175] |
| Δtilt_mean | +0.083 rad | 1.009 | 0.71 | [−0.348, +0.513] |
| Δn_falls | −9 | 41 | 0.36 | [−28, +2] |
| Δfirst_fall | −3,090 | 12,798 | 0.28 | [−8,598, +2,247] |
| **Δda_mean** | **+0.024** | 0.056 | **0.055** | [+0.002, +0.050] |
| Δpre_w_growth | −0.99 | 12.63 | 0.72 | [−6.58, +4.24] |
| Δmax_distance_from_origin | +0.35 m | 3.08 | 0.61 | [−0.86, +1.75] |
| Δtotal_path_length | −2.97 m | 13.80 | 0.34 | [−9.30, +2.61] |

**Mechanism check (informal):** per-seed deltas are non-trivial (e.g. seed 47: Δn_falls=−184, Δfirst_fall=−17,101, Δpath_len=+17.77; seed 55: Δn_falls=+15, Δtilt_mean=+2.23). The variant is *not* bit-identical to baseline — the knob is firing. But the per-seed variance is large and cancels at n=20.

**Read:** behaviorally null. The borderline Δda_mean trend (p=0.055) suggests the variant finds marginally more reward pulses on average, but it does not translate to standing stability, fall reduction, or locomotion coverage. With `temperature_base=1.0` the epistemic bonus `(1 − visit_ema_i) · 0.1 ≤ 0.1` in pre-softmax score units is well under softmax noise; per `feedback-no-tuning`, the right move is an adaptive scheduler (gain tied to DA suppression or drive urgency), not bumping the static value to 1.0.

### C2 — Eligibility traces (`eligibility_lambda=0.5`)

| Metric | Δμ | σ | p | 95% CI |
|---|---|---|---|---|
| All 11 paired metrics | **+0** | **0** | **1.00** | **[0, 0]** |

Bit-identical Δ=0 across all 20 paired seeds. Same signature as the Stage B Hebbian null: when a knob is architecturally disconnected in the active code path, the variant produces a trajectory bit-identical to baseline.

**Root cause (Premotor.cpp:573–589):** the eligibility-trace block lives in `apply_reward()`, which is the *legacy per-event Hebbian update path*. The mc_reinforce/MC-mode path (used by PiCrawler with `mc_lr=0.05`) defers all updates to `finalize_mc_episode()` on `events.episode_end`. Per the schema comment at Premotor.cpp:120: "When mc_lr > 0, defers per-event Hebbian updates from apply_reward to per-episode finalize_mc_episode." MC mode never enters the eligibility-trace branch; the trace matrix E_ exists but is never read or written.

So `eligibility_lambda` is **mechanism-inert in PiCrawler's active code path.** This is an architectural gap, not a tuning issue. Eligibility traces with MC REINFORCE would require folding the trace into the per-episode G_t computation (e.g. λ-return) — that's a Premotor design exercise, not a config knob.

### C3 — HomeokineticExploration (deferred)

Not run in Stage C. Reasoning: requires C++ wiring (Premotor must subscribe to `kExplorationDirective` and override softmax-sampled intent when active) because PiCrawler's 12-channel action chain does not route through `ActionGate`. Pursued in Stage C.5 / Phase 6.7 once one of the two simpler paths below has demonstrated lift.

## What the snapshot from the user's long live run added

The user observed a 1 h 47 min UI run where the front two legs crystallized into a bent-up pose and the body stayed locked in that pose for 24 m 38 s of contiguous "standing". End-of-run module metrics from that snapshot, surfaced separately from the Stage C A/Bs:

- **Front pitch Premotor pair: W_norm=13.26, H=0.62** — entropy of 0.62 nats over n_intents=5 implies one intent fires ~85% of the time. Fully collapsed distribution.
- **Rear pitch Premotor pair: W_norm=9.03, H=1.32** — still exploratory.
- **47% W-norm asymmetry front-vs-rear.** LR-symmetry averages left↔right, not front↔back, so front and rear pairs can and do diverge. Here that asymmetry crystallized into a pathology.
- **DA = 0.000, HT = 0.000, drive urgency = 1.000, alive = −1.000.** All chemicals decayed; HomeostaticDrive at full alarm. r_sig = −0.100 (tonic over-height penalty, no per-tick variance for REINFORCE advantage to credit).
- **Final position 0.71 m from origin** — not against any pyramid (r ∈ [3.5, 9]) or wedge ring (r=10). The stuck pose is a self-stabilizing tripod against the body's own bent-leg geometry. *Not* a terrain-climbing artifact, as initially hypothesized.

This is exactly the failure mode HomeokineticExploration was designed to address: drive errors saturated + state is too predictable (low TLE variance) → fire babble directive. The C++ wiring required to consume the directive in Premotor is the deferred C3 work.

## Recommendation — Stage C.5 path

Stage C closes outcome-null on both variants. Promotion of neither knob to default is warranted. But the snapshot diagnostic plus the C1/C2 null pattern points to a clear next mechanism: **break policy attractor lock-in when entropy collapses and reward signal goes flat**. Three ways to ship this, in increasing cost:

1. **Per-Premotor entropy in JSONL (4 lines):** observability prerequisite. Surface each Premotor's `intent_entropy` alongside `W_total_norm` in the per-tick JSONL. Without this, attractor lock-in is invisible during a run — only catchable post-hoc from end-of-run metrics. Costs nothing, pays for all subsequent work.

2. **Body-side adaptive escape (no C++):** in `picrawler_body.gd`, monitor per-Premotor entropy + chassis_xz motion + DA floor. When `entropy < entropy_ema − k·σ` AND chassis_xz hasn't moved > ε for > T ticks AND DA at floor → inject Gaussian perturbation into that Premotor's W via `brain.get_module_snapshot` / `set_module_snapshot` (same path Stage B's leg_symmetry sync uses). Mechanism-shaped, adaptive, body-side — no C++ build. Ship this *before* the proper version to falsify whether breaking the attractor moves the metric.

3. **HomeokineticExploration → Premotor wiring (~30–50 lines C++):** the architecturally clean answer. Add a `kExplorationDirective` subscription to Premotor; when `directive.active`, override the softmax-sampled intent. Instantiate HomeokineticExploration in the picrawler topology. Brain-internal — no boundary violation (the user asked, and the directive lives entirely inside the brain; the body never sees it).

Mitosis Gatekeeper does NOT fit this failure mode: it spawns nodes when prediction error is high in regions existing nodes don't cover. The stuck pose is *repetitive*, so prediction error is low — Mitosis would not fire. Mitosis still belongs in the long-term roadmap for context-conditional policies (climbing as a distinct skill once we add foraging-induced terrain interaction in Stage D), but it's not the fix for this failure.

## Anti-goals satisfied

- Curriculum disabled during both A/Bs (no teleport contamination)
- n=20 paired seeds (above the v9 retraction threshold)
- Both arms shared identical `auto_reset_on_inversion=false`
- C1 self-check Δ=0 verified before the variant run
- No C++ build performed in Stage C (C3 deferred for this exact reason)
- No static-parameter promotion attempted off Stage C results — both variants are outcome-null, so no default change

## Files

- C1 config: `godot_host/project/addons/ami_ogma/configs/the_picrawler_stand_target_lr_sym_c1_epistemic.json`
- C2 config: `godot_host/project/addons/ami_ogma/configs/the_picrawler_stand_target_lr_sym_c2_lambda.json`
- Aggregator extension: `scripts/picrawler_ab.py` (added `max_distance_from_origin`, `total_path_length` to `PAIRED_METRICS`)
- Per-seed data: `results/picrawler_C1_epistemic.json`, `results/picrawler_C2_lambda.json`
