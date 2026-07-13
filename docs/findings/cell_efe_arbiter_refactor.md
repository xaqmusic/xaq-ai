# EFE arbiter precision refactor — Stages 0–2 result

*Session 2026-07-02, branch `cell-maze`. Executes
[`../plans-and-designs/efe_arbiter_precision_refactor_plan.md`](../plans-and-designs/efe_arbiter_precision_refactor_plan.md)
(doctrine §2.2 explicit-EFE scoring / §2.3 precision as a controlled variable). Supersedes the
value-race arbitration documented in [`cell_realeat_foraging_baseline.md`](cell_realeat_foraging_baseline.md).
Stages 0–2 done; Stages 3–4 (planner epistemic + model precision, food-relocation re-inference)
are the next leads.*

## What changed

`EFEArbiter` gains a `scoring_mode ∈ {value_race, efe}` param (**default `value_race` —
byte-identical to prior builds**; every other env stays reproducible). `the_cell_arbiter.json`
opts into `efe`. In `efe` mode each policy is scored as an **expected free energy**
`G = pragmatic + epistemic`, with the two pragmatic terms in **shared units**:

```
g_prag_klino   = hunger · reach_klino     reach_klino   = eat-calibrated capability cap∈[0,1]  (→1 in klino's own eating range)
g_prag_planner = hunger · reach_planner   reach_planner = clamp(plan_value)  (discounted route value ≈ γ^hops < 1 to a remembered cache)
g_epist_klino   = (1−hunger) · z_spike_norm    (scent rising above klino's methylation baseline = "food this way")
g_epist_planner = 0                              (planner frontier-novelty — Stage 3)
G_klino = g_prag_klino + g_epist_klino ;  G_planner = g_prag_planner + g_epist_planner
```

Fed into the **existing** adaptive-hysteresis winner-take-all (unchanged). `hunger`
(interoceptive `1−energy`) is the **preference precision** — it sets the exploit↔explore
balance, so there is no tuned λ.

**The scale mismatch is gone by construction.** In value_race, klino's proximity level was
structurally capped ~0.73 (source-normalised field, sub-1 sample at closest approach) while the
planner self-normalised to ≈1 — patched by three accreted hacks (the eat-cal boost, the
`max`-of-three, the `×(1−hunger·scent)` cede). In efe **none of those exist**: near food
`cap→1 ⇒ g_prag_klino→hunger`, which necessarily exceeds the planner's discounted
`hunger·γ^hops` to any *remembered* (non-adjacent) cache → klino owns the close on pragmatic
value alone. `G_planner = hunger·clamp(plan_value)` is a direct product ≥0, so the negative-dip
"false interruption" the plan_peak/cede were built to prevent is **structurally impossible**.

## Evidence

**Unit (18/18 `test_efe_arbiter`):** three efe tests — blind-forager preserved (the historical
trap: silencing it collapsed eats 31→5); klino owns the close by shared units with **no cede**;
route-hold margin `G_planner−G_klino` stays strictly positive across a 3000-tick sustained route
(no false interruption), then hands off to a hungry approach.

**Headless (`the_cell_arbiter`, seed 42, 18 000 ticks, per-tick diag):** the close is clean in
**both** modes — klino owns **100 %** of near-food ticks (scent > 0.6), **0** winner flips,
100 % of the 30 ticks before every `events.eat` — but efe does it **without the cede or
plan_peak**. The operator-observed oscillation was already suppressed in value_race by the prior
session's interim hacks; efe reproduces the clean close from correct semantics instead.

**The foraging win (robust across the tumble/explore RNG, 3 seeds each):**

| | value_race | **efe** |
|---|---|---|
| eats / 300 s | 2 (all seeds) | **4 (all seeds)** |
| mean hunger | 0.263 | **0.189** (better fed) |
| klino drives | 20 % of ticks | **73 %** |
| planner drives | 80 % | 27 % |
| eat ticks (seed 42) | 270, 8204 | 270, 8226, 10863, 16297 |

The two are **identical until the 2nd eat (~tick 8200)**, then diverge: value_race stalls into a
~7900-tick starvation gap (the planner routing the bug on the slow **graph-crawl** — the
"exploit-route crawl / circle-until-starve" blocker named in the baseline doc), while efe keeps
eating. Mechanism: once sated, exploiting a *remembered* route has low pragmatic value
(`hunger·γ^hops` with small hunger), so the curious direct forager (klino, fast run-and-tumble)
wins — homeostatic explore/exploit falling straight out of the AIF math. efe hands the competent
forager more authority exactly when routing is not worth it, which **doubles the eats** and
mitigates the crawl blocker.

## Telemetry

The four EFE terms (`g_prag_klino/planner`, `g_epist_klino/planner`, `G_klino/planner`) and
`scoring_mode` are exposed through `diag_snapshot` → `OgmaBrain::get_module_metrics` (via
**accessors** — the documented gotcha) → the GDScript flattener → the JSONL, ready for the
Stage-4 four-term inspector.

## Stage 3 — planner epistemic term + model precision (built; term is a FAST-FAIL here)

`PlaceGraphPlanner` now publishes two §2.2/§2.3 scalars (default-off unless the topic is set):

- **`plan_precision ∈ [0,1]`** = sharpness of the food belief = `1 − H(food_dist)/log N` (single
  known cache → 1; food spread over many caches → lower; empty map → next-hop value margin). This
  is §2.3's *controlled* precision — near a known source the belief is sharp, and the arbiter can
  compare it against klino's *sensory* precision (`cap`) rather than a static `1/(tle+ε)`.
- **`plan_novelty ∈ [0,1]`** = the frontier TLE the planner routes toward when there is **no**
  committed food route (normalised by its own running peak, §6), ~0 while exploiting a route.

The arbiter subscribes both and adds `g_epist_planner = (1−hunger)·plan_novelty` (ablation param
`planner_epistemic`), plus an opt-in klino undirected-search floor
`g_epist_klino += (1−hunger)·(1−plan_precision)` (param `klino_search_floor`, default off).

**The mechanism works and is verified** (unit `EfePlannerEpistemicDrivesSatedExploration`; headless
`gep` fires 4400+ ticks, `plan_precision` populates ~0.99). **But turning it on REGRESSES foraging
in this env** — the opposite of the plan's prediction. A/B (`planner_epistemic` on vs off, 18 000
ticks; on-arm confirmed at 1 eat across seeds 42/43/44):

| | epistemic ON | epistemic OFF (shipped) |
|---|---|---|
| eats / 300 s | **1** (all seeds) | **4** (all seeds) |
| coverage bbox | 339 | **538** |
| near-food ticks (scent > 0.6) | **0** | 2289 |

Mechanism: mid-field a *weak* scent gradient makes klino's epistemic z-spike small, so the planner's
frontier novelty (`gep` high) **wins and pulls the bug off to map new ground instead of climbing the
weak gradient to food** — near-food ticks collapse to 0. In a **scent-solvable, static-food** maze,
frontier curiosity is all cost and no payoff (the food never moves, so mapping finds nothing). The
homeostatic "sated → explore" is correct AIF, but its value is conditional on there being *something
new to find* — which this env does not provide.

**Decision (fast-fail, promote-or-kill §8):** the term is **built, ablatable, telemetered, and
OFF by default** (`planner_epistemic=false` ships — the 4-eat setting). `plan_precision` **does** ship
on, because it is the signal the Stage-4 food-relocation test needs (belief-entropy rises as the old
cache disconfirms). **Doctrine implication (§2.2):** the epistemic term raising coverage is *not*
unconditional — it requires an env where exploration has instrumental food value. Fold this caveat
into §2.2 when the doctrine is next revised.

## Honest limitations / next

- **n = 1 deterministic env** (seeds 42/43/44 give identical macro; sweeps vary only the tumble RNG
  and reproduce every headline exactly). Rank by structural signals, not a hit-count distribution.
- **The close was already clean in value_race** (interim hacks). efe's *close* headline is
  therefore "same clean close, from correct semantics, no hacks", not a fresh oscillation fix.
  The *fresh* result is the foraging rebalance (2× eats).
- **efe reduces planner authority** (80 %→27 %). In *this* L-bend that helped; a regime where
  routing to a far/occluded cache is essential could regress — untested.
## Stage 4 — re-inference under food relocation (the (d)-bar — DEMONSTRATED)

Relocation is **intrinsic** to this env, no scripting needed: `nutrients=1`, `food_alternate=true`,
two `food_positions` — the single nutrient hops to the *other* spot on every eat, so the bug must
re-infer where food is after each meal. Over an 18 000-tick run (shipped config) the four eats land
at **alternating positions** — `(4.5,−4.8) → (−8.2,8.1) → (6.3,−6.2) → (−5.4,8.0)` — i.e. the bug
**re-finds the relocated source every time** (A→B→A→B). That is the re-inference the (d)-bar asks for.

`plan_precision` is the visible §2.3 signal and shows the **dip-then-recover** signature. Cleanest
interval (eat@10863 → eat@16297): precision **drops to 0.07** the instant the food hops away (the
old cache is now empty → belief uncertain), then **recovers to 0.98** as the bug re-acquires the new
location — then it eats again. The arbiter stays stable across the relocation (10 winner flips over
5434 ticks = clean klino↔planner handoffs as it re-searches then closes, not chatter). Unit-locked by
`PlanPrecisionSharpForOneCacheLowForTwo` (one cache → 1, two ~equal → <0.5).

**Gate: met at the mechanism level** — the bug resumes eating at the relocated spot, `plan_precision`
visibly dips and recovers, the winner stays stable. **Honest caveat:** the timescale is SLOW
(1700–5400 ticks between eats) because foraging is the graph-crawl — the re-inference is *correct* but
not *crisp*. A snappy headline demo is gated on the exploit-route **crawl locomotion** blocker (named
in the baseline doc), which is outside the arbiter's scope. And the planner epistemic term did **not**
pay off even here (relocation is to a *known* alternate spot, not a novel frontier), consistent with
the Stage-3 fast-fail.

## Honest limitations / summary

- **n = 1 deterministic env** (seeds 42/43/44 give identical macro; sweeps vary only the tumble RNG
  and reproduce every headline exactly). Rank by structural signals, not a hit-count distribution.
- **The close was already clean in value_race** (interim hacks). efe's *close* headline is
  "same clean close, from correct semantics, no hacks", not a fresh oscillation fix. The *fresh*
  result is the foraging rebalance (2× eats).
- **efe reduces planner authority** (80 %→27 %). In *this* L-bend that helped; a routing-critical
  regime could regress — untested.
- **The crawl remains the next real blocker.** efe converts more of the bug's time into direct
  foraging and re-inference works, but the graph-route locomotion is slow. Making routing move at
  run-speed is the lead that would turn all of this into markedly more eats.
