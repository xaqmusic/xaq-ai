# Cell PlayLoop — the third policy: grow the map (epistemic explore)

*Design + build spec. Branch `cell-maze`. Task #33 (the PLAY loop postulated in
`cell_efe_arbiter_plan.md` §Open/N-loop, never built). Doctrine refs: §1 (predict-
or-die), §2.1 (epistemic foraging), §2.2 (explicit EFE ledger — pragmatic vs
epistemic), §2.3 (precision from own dynamics), §3 (decompose one goal per loop),
§5 (never disable a working loop — decompose it), §8 (default-off, verify the
consumer, staged gates).*

---

## 0. The problem this fixes

`PlaceGraphPlanner` is **overloaded**: it is a navigator (route to remembered
food) *and* an explorer (TLE-novelty term, habituation, desperation-disconfirm,
confinement-steer, run-tumble explore locomotion — five of its six mechanisms).
Two goals folded into one value field `V[n]` → neither is clean (§3), and the
overloading is *structural*, not tunable:

- **The value field can't do both.** Every explore knob patched onto `V` either
  regresses foraging (`planner_epistemic` 4→1) or doesn't fire (`escape_gain`
  needs 1.0, which drops single-zone 4→2). A uniform explore-drive added to `V`
  *backfired* — it kept `route_exists` true so the bug **routed** on the staleness
  gradient instead of **wandering** (memory: FALSIFIED). Both results say the same
  thing: **exploration wants its own locomotion and its own arbiter slot**, not a
  term in the navigator's field.
- **The planner routes only over MAPPED nodes** (`argmax` over observed
  neighbours). It structurally **cannot reach the unmapped frontier** — the quad
  failure: post-eat the bug routes 99% / wanders 0%, confined to mapped zone SW,
  never reaches the 9.4 m connector.

Fix = **decompose the overloaded planner into two single-directive loops** (§3),
arbitrated as a three-way EFE race (§2.2):

| loop | directive | value | doctrine term |
|---|---|---|---|
| **klino** (RunTumbleNav) | *close* on a sensed source | `hunger · scent_proximity` | pragmatic |
| **planner** (PlaceGraphPlanner) | *traverse* the graph to remembered food | `hunger · food_route_value` | pragmatic |
| **play** (PlayLoop, NEW) | *grow* the graph — wander to the frontier | `energy_surplus · novelty` | **epistemic** |

---

## 1. Curiosity is instrumental — play most when FULL (the corrected posture)

Rejected: my first cut weighted play by `confinement · hunger` (explore when
trapped-and-hungry). **Operator correction, adopted:** *a curious bug is one that
survives* — if you save exploration for when you're hungry you starve. Play should
run **most when full, with energy budget to spare**, so returning to food is cheap
*when it later becomes necessary*. This is epistemic foraging done right (§2.1):
invest in uncertainty-reduction (map growth) while you can afford it; spend the map
(routing) when you must.

- **pragmatic weight** (klino, planner) ∝ **hunger**
- **epistemic weight** (play) ∝ **energy surplus** (≈ 1 − hunger)

Two consequences fall out for free:

1. **Quad is unblocked by construction.** The bug spawns full (E≈0.99) → play wins
   → it **wanders the frontier and grows the map across zones** (discovers the
   connector) *before* hunger — exactly when cross-zone discovery should happen.
   The steer we couldn't make work while hungry is the wrong tool; the full bug
   just explores.
2. **The single-zone 4-eat baseline is protected.** When the bug is hungry-and-
   smelling, play is down-weighted by low energy → klino's scent-follow is no
   longer out-competed by curiosity (the thing that broke `planner_epistemic`).

**No hand-set energy threshold (§2.3, no-tuning).** `G_play = energy_surplus ·
novelty`, both self-scaled to [0,1]; the arbiter's own race decides the crossover
from context. Never a "if E > 0.6 then explore" constant.

---

## 2. Shared map (Option A) — one node-creator, two overlays

Verified structure: `CylinderBuilder → EPM(epm_place) → RealityToken{winner_id,
tle}`; **the place-EPM is the sole node-creator.** `PlaceGraphPlanner` is a thin
*overlay* — it reads the discrete `winner_id` + scalar `tle` and builds its own
travel-edges + food/value overlays keyed by that id. It never re-embeds.

`PlayLoop` is a **second overlay on the same node graph.** "Grow the graph"
physically means: **move the body to novel ground → the place-EPM bakes a new
`winner_id` → both planner and play see the new node.** Growth is shared for free;
that is what makes play *instrumental to the planner* (it builds the nodes the
planner later routes on). There is **no second map.**

**Vision-EPM de-aliasing: TABLED** (operator: table until a signal warrants it).
Rationale on record (§5 below / findings): building play's map on the vision-EPM
is the *wrong granularity* (per-view, saccade-harvestable novelty, aliases more
than the cylinder); the useful use is a place-scale de-aliasing *sensor* via a
**conditioned concat** at the place-EPM input (split, don't blend), gated on
**measured** aliasing. Not v1. The vision-EPM's "give it an honest job or trim it"
question (today it feeds only averaged arousal) rides on that decision.

---

## 3. PlayLoop module — "PlaceGraphPlanner minus traverse"

Build path (operator): **duplicate `PlaceGraphPlanner` → `PlayLoop`, then
subtract.** The explore machinery already lives in the planner; extraction is a
refactor, not an invention.

**KEEP (the grow half):**
- **Novelty value field.** `V_play[n] = node_tle[n] + γ · max_{m} V_play[m]` — the
  per-node EMA of the place-EPM TLE (predictive-model degradation — §1/§2.1, the
  honest epistemic signal), value-iterated so it climbs toward the frontier.
- **Uphill-then-wander gate.** Climb `V_play` to the frontier (highest-novelty
  known node), then at the local novelty max **run-and-tumble WANDER beyond the
  mapped graph** into unmapped ground. This is the one behaviour the planner
  structurally lacks. Same run-tumble mechanism as RunTumbleNav, climbing the map's
  own TLE instead of scent.
- **Habituation** ("recent = boring") — suppresses recently-dwelt nodes so play
  *sweeps* the map instead of orbiting one novelty peak.
- **Success credit off `events.eat`** (the salvaged-from-HK idea, actually used
  this time): play EMAs whether its episodes lead to eats — telemetry that the
  wander is productive, and a hook for later self-calibration.

**SUBTRACT (the traverse half):**
- food-memory value-iteration `V = food + γ·maxV`, the route/argmax-to-food policy,
  and the `plan_value` publish. Play does not route to food.

**DELETE outright — scar tissue of the overloading:**
- `escape_gain` (confinement-steer) + its `stale_ticks` clock, and `disconfirm`
  as an *explore* driver. These existed only to force the *food-routing* planner to
  explore despite its own food field pinning `route_exists` true. PlayLoop has no
  food field to fight — its value **is** novelty, which *clears as the EPM learns*
  (node_tle decays), so the orbit pathology can't arise. Separating the loops is
  what lets us drop the hacks (a real simplification win, not a loss).

**Outputs:** `percept.play_bearing` (egocentric heading, same convention as klino/
planner) + `reality.cognitive.play_value` (normalised frontier novelty ∈[0,1], the
epistemic value the arbiter scores; normaliser = slow-decaying TLE peak, §6-derived
not hand-set).

**Inputs:** `place_topic` (shared map), `heading_topic`, `vel_topic` (path-
integration), `hunger_topic`/energy, `eat_topic = events.eat`.

**Default-off (§8):** empty output/value topics ⇒ no publish; existing configs
byte-identical. Lives only in `the_cell_arbiter*.json`.

---

## 4. Planner cleanup — pure traverse (config, not deletion)

§5 forbids disabling a working loop. We do **not** delete the planner's explore
code — we **configure** the arbiter-config planner to pure-traverse and let
`PlayLoop` own exploration, keeping the current planner-with-explore config as the
fallback **baseline** for the A/B.

Pure-traverse planner (in `the_cell_arbiter*.json`): `tle_gain=0`, habituation off,
`escape_gain=0`, explore-locomotion off. **Keep `disconfirm`** — fading a camped-
but-uneaten cache is honest belief-updating for a *router* (§2.1 re-inference), not
an explore behaviour. Result: planner routes to food when a route exists, else
publishes `plan_value=0` and **yields to play** (its old wander branch becomes
vestigial — the arbiter hands authority to play instead).

---

## 5. Arbiter — the three-way EFE ledger (§2.2)

The efe-mode arbiter already scores `G = pragmatic + epistemic` per policy in
SHARED UNITS. Add play as the third policy and **move the epistemic term out of the
planner into play** (the planner's `g_epist_planner` ships OFF — superseded):

```
G_klino   = hunger        · scent_proximity      (pragmatic close)
G_planner = hunger        · food_route_value      (pragmatic route)
G_play    = energy_surplus · novelty              (epistemic grow)      ← NEW
winner    = argmax G   (winner-take-all gain vector, adaptive hysteresis)
```

All three terms in [0,1] shared units (novelty via `plan/play_value`'s peak-
normaliser, §6) → no cede constant, no scale mismatch (the whole point of the efe
refactor). The gain vector already generalises to N channels ("N-loop" note);
this instantiates the third.

**Motor wiring:** a third `HeadingController` (`heading_controller_play` →
`cogp.steer/thrust`) driven by `percept.play_bearing`, gated by
`arbiter.gain.play`; MotorBus mixes it as a fourth cognitive channel (klino/
planner/play + reflexes). Feed the **full** lower loop (§5 — its learned advance
ON), not a stripped one.

---

## 6. Telemetry / inspector (the recurring gotcha)

`get_module_metrics` uses **accessors**, not `diag_snapshot` — new PlayLoop fields
(novelty, play_value, wander/climb state, eat-credit EMA) and the arbiter's third
`G_play`/`gain.play` need explicit accessor + GDScript-flattener lines or they read
0 in the JSONL. v4_inspector: extend the arbiter widget's race to **three** stacked
bars; add a `play` widget (frontier novelty meter + climb↔wander gauge + coverage).
HUD tug-of-war → three-way.

---

## 7. Staged plan (fast-fail, promote-or-kill gates)

**Stage 1 — Wiring (pilot).** PlayLoop compiles (duplicate→subtract); wired end to
end; arbiter adds `gain.play` but with the **epistemic weight forced 0 (inert)** so
the winner is still klino/planner. **GATE:** PlayLoop publishes sane novelty ∈[0,1];
its HeadingController produces motion when force-authoritied; the third channel
shows at 0 authority; single-zone run byte-identical (default-off proven); consumer
fires (§8). Unit tests for PlayLoop (novelty field, uphill-then-wander, eat-credit)
+ arbiter 3-policy race.

**Stage 2 — Signal.** Turn play on (energy-gated). Headless quad + single-zone,
seed 42, structural read (det env → rank by structural signal, not brittle hits).
**GATE:** (a) play WINS when full, LOSES when hungry (energy→authority visible in
the race); (b) play's run-tumble **leaves the mapped graph** (reaches unmapped
frontier); (c) **quad: play grows into zone 2** (discovers the connector — the steer
couldn't); (d) single-zone 4-eat baseline preserved (play doesn't out-compete klino
near food).

**Stage 3 — Direction.** Switch the arbiter-config planner to pure-traverse; confirm
the clean split (klino closes / planner routes / play grows) and the homeostatic
cycle (explore-full → route-hungry → eat → refuel → explore). **GATE:** quad cross-
zone foraging works; no regression vs Stage 2; the crossover emerges from the race,
not a threshold.

**Stage 4 — Powered.** n ≥ 10 by **varying the ENV** (not the seed — det-env caveat).
Metrics: quad eats/coverage/zones-reached vs the single-loop baseline. **Ablations
ship with the loop (§2c):** play-off (`play_value`→dead topic ⇒ 2-loop baseline);
play-forced-on (pure explore — should cover but starve = instrumental-only control);
**wrong-sign** (weight play by hunger not energy_surplus — should REGRESS, proving
play-when-full is the right sign, §8 downvoting).

---

## 8. Open items / future

- **Vision-EPM de-aliasing (v2, TABLED).** Conditioned-concat split at the place-EPM
  input, gated on *measured* aliasing (does the cylinder collapse two distinct areas
  into one node and suppress play's novelty there?). LateralVoter/mitosis as
  escalation. See `docs/findings/` (to write if warranted).
- **Vision-EPM job-or-trim.** Today it feeds only averaged arousal (rich embedding
  → one TLE scalar). Either the de-aliasing sensor above, or the occlusion-break
  vision-homing (`planner.vision_topic` hook), or trim it (§8 dead-channel).
- **Arousal-as-global-precision (§2.3, speculative Stage 5).** If wired, the vision-
  EPM's arousal contribution becomes load-bearing precision control.
- **Exploit-route CRAWL locomotion** — still the named survival blocker (the graph-
  route crawl is slow); orthogonal to PlayLoop but gates the homeostatic *cycle*.

---

*Living doc. Fold results back into `docs/findings/` and the memory topic file as
each stage promotes or kills.*
