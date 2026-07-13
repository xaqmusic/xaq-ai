# PlayLoop (task #33) — Stage 2 findings: turning the third policy ON

*Branch `cell-maze`. Autonomous iteration session 2026-07-03. Plan:
`docs/plans-and-designs/cell_play_loop_plan.md`. Det env → rank by STRUCTURAL
signals (bbox/coverage/zones), not brittle eat-counts (§process-discipline).*

## TL;DR

- **Single-zone: play-on does NOT regress foraging.** In the food-dense L-bend the
  bug eats ~1 food / 2.7 ticks, so it is never hungry and always smelling food →
  klino's z-spike wins 100%, play 0%. Play is correctly **dormant** where there is
  nothing to explore. Gate (d) PASS.
- **Quad, climb-only play: play is a live loop but adds NO coverage.** It wins 65%
  of the arbiter, but produces **spatial coverage identical to the no-play control**
  (bbox 529, max 24.5 m, zone 1 — to the metre). Root cause: play explores by
  **climbing freshly-baked mapped nodes** (climb 385 / wander 3) — the place-EPM
  bakes a fresh high-novelty node ahead as the bug moves, so there is always an
  uphill neighbour and the `wander→beyond` branch never fires. Same confinement the
  planner had, now in play.
- **The wrong-sign ablation explored 2× (reached zone 2)** — but it *also* never
  wanders (wander 0), so that is a deterministic-trajectory artifact of a different
  winner timing, NOT evidence the energy-gate sign is wrong.
- **Fix built: map-growth-stall → wander** (`wander_stall_ticks`). When the map has
  not grown for N ticks the bug has mapped this region → FORCE the run-tumble to
  push PAST the frontier; a newly-baked node resets it → back to climb. This is the
  `wander→beyond` half of the memory's steer→frontier + wander→beyond hybrid.

## The energy-gate, measured (quad, climb-only)

`G_play = play_weight·(1−hunger)·play_value`. Play wins 65% overall, but the hunger
breakdown is **inverted** from the design intent (play wins *more* when hungry:
full 53% / mid 76% / hungry 96%). Reason: in the quad far-field there is no scent to
exploit when hungry, so klino/planner go to ~0 and play wins **by default**, not by
strength. Important nuance about this env:

> **The quad's one-shot-far-food design decouples "full" from "near food."** In a
> normal foraging env (food available where you are) play-when-full is right. In the
> quad, "full" means "just ate the only local food, which is now gone" — so being
> full does NOT mean food is near. The energy-gate's assumption is weakened by the
> env, not necessarily wrong as a principle. Keep the operator's play-when-full
> design; note the quad is a poor test of the *sign* (a food-respawn or multi-food
> env would test it cleanly).

## A/B table (quad, 18000t, seed 42, eat-gated connectors)

| arm | bbox | max dist | zones | play win% | climb/wander | map nodes |
|---|---|---|---|---|---|---|
| no-play control | 529 | 24.5 m | 1 | — (klino52/plan48) | — | — |
| play energy-gate | 529 | 24.5 m | 1 | 65% | 385 / 3 | 2→10 |
| play wrong-sign | 1086 | 31.5 m | 2 | 22% | 129 / 0 | 2→11 |

Structural reading: energy-gate play = control coverage (the wander never fires);
the wrong-sign's wider spread is a trajectory artifact (it also never wanders).

## Cross-zone is double-gated

Two independent blockers, only one of which is play's job:
1. **Connectors open on EATS** (`wall_drop_on_eats=1`), and far-field foraging is
   slow (**1 real eat in 18000t** across all arms) → only 1 connector ever dropped.
   Cross-zone is gated on foraging speed, which play does not accelerate. (Use
   `OGMA_WALL_DROP_PERIOD` / `wall_drop_period` to open connectors on a clock and
   isolate play's discovery from foraging — the timed test below.)
2. **Play climbs mapped ground, doesn't wander into unmapped ground** — the deficiency
   the stall-wander fix addresses.

## Timed-connector A/B — the stall-wander fix (decisive test)

*Connectors dropped every 3000t (`OGMA_WALL_DROP_PERIOD=3000`) so cross-zone is not
gated on the slow eat; stall-wander ON (`wander_stall_ticks=30`) vs OFF. 18000t, seed 42.*

| arm | bbox | zones | first cross-zone | eats | play climb/wander | forced_wander |
|---|---|---|---|---|---|---|
| B: timed + climb-only | 2227.8 | **4** (all) | t=9420 | 2 | 212 / 34 | 0 |
| A: timed + stall-wander | 2227.8 | **4** (all) | **t=5160** | **3** | **0 / 308** | 589 |

**The correction to my initial read:** with the connectors OPEN, *both* arms reach all
4 zones — **bbox is identical (2227.8 = the full-maze extent).** So the quad confinement
was primarily the **connector-open mechanic** (eat-gated + slow far-field foraging → only
1 of 3 connectors ever dropped in the eat-gated runs → the bug is walled into zone 1),
**NOT** play's climb-vs-wander. That is an env mechanic, not a play deficiency.

**What the stall-wander DOES do (real, but modest):** it makes play genuinely
`wander→beyond` (308/0 vs 212/34 — pure wander vs mostly-climb), and in this run it found
the first opening **faster (t=5160 vs 9420)** and got **3 eats vs 2**. Total coverage is
geometry-bounded once the zones open, so the effect is on **exploration style + discovery
speed + foraging**, not final coverage. n=1 deterministic — do not over-read the eat delta.

**Net:** the dramatic "all 4 zones" is the timed connectors doing the heavy lifting; the
stall-wander is a legitimate, on-design improvement (play stops treadmilling the mapped
region and pushes past the frontier) with a modest measured benefit here. It is **off by
default** (`wander_stall_ticks=0`); the play configs ship it at 30 (≈ explore_cycle).

## Eat-gated (shipped env) — the connector wall is the hard blocker

`the_cell_arbiter_quad_play` as designed (connectors open on eats, no timer), 18000t:

| arm | bbox | zones | eats | play win% | play climb/wander |
|---|---|---|---|---|---|
| eat-gated + stall-wander | 537 | 1 | 1 | 23% | 0 / 137 (fwand 591) |
| eat-gated + climb-only | 529 | 1 | 1 | 65% | 385 / 3 |

**Even with the stall-wander pushing play to wander widely (fwand 591), the bug stays in
zone 1** — it hits the CLOSED connector walls and physically cannot cross. Only 1 of 3
connectors ever drops (1 eat in 18000t). So: **cross-zone in the shipped eat-gated quad is
blocked by the connector-open mechanic + slow far-field foraging, which play does not
address.** (Side effect worth noting: the stall-wander *lowers* play's arbiter win% 65%→23%
— wandering into already-explored ground drops `play_value`, so klino wins more.)

**To SEE play cross zones, the connectors must open** — either press **KEY_D** in the UI, or
set `wall_drop_period` / `OGMA_WALL_DROP_PERIOD` (the timed A/B above). The play configs now
ship `wall_drop_period=3000` so the UI demo reveals the spiral hands-off (KEY_D still works).

## The real remaining quad blocker (for the operator's UI session)

**Far-field foraging is slow (~1 eat / 18000t).** The one-shot food sits at each zone corner
and the scent is too weak/short-range for the bug to home reliably from across the zone. This
is the memory's noted "quad foraging SLOW" — an ENV / scent-FEEL tuning task (falloff,
food placement, sigma), best done live in the UI (operator's stated preference), not a play
change. Fix that and the eat-gated adaptive reveal (eat → connector opens → play explores the
new zone) closes the loop on its own.

## Configs for UI review (all in the launcher dropdown)

- `the_cell_arbiter.json` — single-zone 2-loop reference.
- `the_cell_arbiter_play.json` — single-zone + play (dormant here; `arbiter.play_weight`
  is a live 0↔1 slider).
- `the_cell_arbiter_quad.json` — **quad no-play control**.
- `the_cell_arbiter_quad_play.json` — **quad + play** (the A/B partner). Live sliders:
  `arbiter.play_weight` (0=2-loop), `play.wander_stall_ticks` (0=climb-only),
  `arbiter.play_hunger_weight` (wrong-sign). Press **KEY_D** to drop connectors.
- `the_cell_arbiter_quad_wrongsign.json` — **wrong-sign ablation** (explore-when-hungry).

## Honest status

Play is a real, competent, arbitrated third loop (Stage 1 byte-identical + Stage 2
energy-gated, both verified). What it does NOT yet do at n=1 in the quad is produce a
clean cross-zone *coverage win* over the no-play control — that needed the wander-beyond
fix + de-confounding the connector-on-eat gate. n=1 deterministic throughout; the
sign question wants a food-respawn env, and any powered claim wants ENV variation, not
seed variation.
