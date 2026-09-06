# Cell round 2 — the charter, with results inline

*Started 2026-09-06 on branch `cell-audit`, after the [system audit](../reports/cell_system_audit_2026-09.md).
The verdict record is the [Cell lever ledger](../reports/cell_lever_ledger.md); the claims the
round tests are rows in the [audit register](../reports/cell_system_audit_2026-09_appendix.md);
the recipe the round refines is [`loop_and_arbitration_recipe.md`](loop_and_arbitration_recipe.md);
what stays undecided is in the [open-items register](open_items_register.md). Convention as in
the Kalman charter: every measurement is recorded here, in the section of the stage that
produced it.*

## Purpose

The Cell study's composition tied a hand-written reactive forager at best and lost to it as
shipped, in a room where a map is overhead. Round 2 puts the same composition in a regime
that *demands* memory, repairs what the audit found broken, and measures one lever at a
time against the reactive specialist and a gradient-blind floor, so that what the microduck
inherits is a loop-and-arbitration recipe with a verdict, not a design.

Operator decisions (2026-09-06): the far-food relocation arena leads, the L-bend is the
second room for the perturbation battery; repairs plus one build lever (the vocabulary-native
place map), then, after the audit located the crowding in the arbiter's units, the
normalisation lever ahead of it; the PR waits; the food-memory verdict (register O9) stays
open.

## Regime

**R-A, `the_cell_route_far_pillars.json`.** A 24 m open room with pillars (`obstacle_density`
0.06, scent attenuation 0.3), two food sites 16 m apart, the food alternating between them on
every eat, scent falloff **2.0**. The base is the study's four-loop brain with the arbiter in
`efe` mode. The reactive specialist in the same world is
`the_cell_chemotaxis_baseline__farfood.json`; the floor is that specialist gradient-blind
(`RunTumbleNavV2.ablation=shuffle`).

Why pillars: without them the absolute-heading panorama is a constant vector and the place
vocabulary holds two nodes for the whole run (audit L9); with them it holds ten in 120 s.

Why falloff 2.0 (calibration, n = 6 paired worlds, 240 s): the screened-Poisson field never
reaches zero (0.44 at the food, 0.01 at the far wall), so at the shipped falloff 6.0 the
reflex returns to the relocated food in ~3000 ticks (3.7 eats). The floor eats 1.8 at every
falloff. At 2.0 the specialist eats 3.0, returns in 4966 ticks against the floor's 5927, and
still earns the first eat locally: the return leg is nearly chance for the reflex, so a
map-user's return would be loud.

**R-B, `the_cell_arbiter_fused_lbend.json`** with `the_cell_chemotaxis_baseline__lbend.json`:
the transfer room for the promoted stack and the (d) battery. Not yet run.

## Harness (P0, done)

`cell_coverage.py` with distinct world and play seeds per job, `OGMA_OBSTACLE_SEED` per job,
the planner's own RNG varied, a seed manifest, `--crossing-mode auto` (the sites' bisector in
a two-site room), relocation metrics (eats after the first relocation, time-to-return,
time-to-near-the-other-site), `--legacy-seeding` for the report's fixed world, `--logdir`.
`cell_liveness.py` before any battery. Body-side scent-noise and heading-drift instruments,
gain-0. `the_cell_world.gd` reads `metadata.obstacle_seed`.

## Arms and verdicts

All n = 6 paired worlds at falloff 2.0 unless stated; verdict vocabulary as in the ledger.

| arm | lever | eats (base 1.0) | what moved | verdict |
|---|---|---|---|---|
| A0 | seeding re-baseline (study room) | legacy reproduces the earlier numbers exactly; repaired: specialist 2.30 / full 0.75 / no-play 1.65 | every direction stands at ~⅔ the effect and 2× the spread; the no-play tie becomes a trend below the specialist | the report's population was one world |
| A1 | `epistemic_reach_gated=false` (study room, n = 20, both seedings) | 0.6 → 0.5; 0.75 → 0.60 | play's share 0.91 → 0.82 | `NULL` — the crowding is a units defect (audit V6), not a gating one |
| A2 | `PlayLoop.wander_stall_ticks=0` | n = 6: 1.5 (Δ +0.5, sd 1.05); **n = 20: 1.60 vs 1.30 (Δ +0.35, sd 1.18, t 1.3); eats after relocation +0.25, sd 0.91** | climb 0.0 → 0.2, forced wander → 0 | `PARTIAL` — the mechanism is restored; the foraging effect is a trend inside its interval |
| A3 | `planner_epistemic=true` | 0.8 (Δ −0.17); at falloff 6.0: 1.7 → 2.7 | the planner's share 0.03 → 0.44 both times | `NULL` — the term buys the planner the motor, not eats |
| A4 | `pragmatic_norm=planner_peak` | 1.0 (identical per-run eats) | the planner's share 0.03 → 0.075, after the eats that occurred | `NULL` alone |
| A5 | `PlaceVectorBuilder` → `epm_place` → `pi_cell_size 0` | 1.33 (Δ +0.33, sd 1.37); food distance worse on 6/6 (+1.17 m, t 3.4) | vocabulary ~19 nodes, rarely grows; play has a route on 0.4 of samples and climbs on none under the latch | mixed alone |
| A5+A4 | | 1.67 (Δ +0.67, sd 1.86, 4+/2−) | the planner's share 0.18 | trend |
| A5+A4+A2 | the stack | n = 6: 1.5 (Δ +0.5, sd 0.84, 2+/0−); **n = 20: 1.35 vs 1.30 (Δ +0.05, sd 1.05, t 0.2, 7+/5−); food distance −0.23 m, t −0.6** | the planner's share 0.10, climb 0.4, the vocabulary 17.6 nodes and rarely growing (stale 832 samples), return legs 8/20 | **`NULL`** — the n = 6 trend was noise |
| floors | specialist / gradient-blind | n = 20: **2.80 / 1.60** (the specialist beats the floor by +1.20, sd 2.14, t 2.5; returns after relocation on 16 of 20 runs vs 8) | | the base composition **ties the random walk** (1.30 vs 1.60, Δ −0.30, sd 1.38, t −1.0) and sits **1.50 eats below the specialist (sd 1.67, t −4.0, worse on 16 of 20 worlds)**; the stack likewise (−1.45, t −4.5) |

**The finding at n = 20, varied worlds (2026-09-06):** in a regime calibrated to demand
memory, the four-loop composition forages at the gradient-blind floor and a full 1.5 eats
below the reactive specialist, and none of the audit's repairs moves it: the play-climb
repair is a trend (+0.35 ± 1.18), the units repair alone changes no eat, and the
vocabulary-native map, stacked with both, ties the base (+0.05 ± 1.05). None of the levers
is loud (CLAUDE.md §3.3), and by that section's own rule the next move is not a further
lever on this arbiter but the bigger idea the mechanism points at. The mechanism is
register R6: **the composition starves while playing.** The body is
hungry on 44 % of samples and play holds 82 % of decisions while it is hungry, because once
the food has moved no pragmatic loop has reach (klino's eat-calibrated capability and the
planner's route value are both near zero), the need gate opens fully, and novelty-seeking
steers away from the visited places the alternating food returns to. The units lever gives
the planner the motor when it has a route; the vocabulary lever gives it more to route over;
neither gives a hungry body with a memory a policy whose reach survives the site's temporary
emptiness (the planner's `disconfirm` erodes the memory on arrival at an empty site).

The panorama check (register V4) stands beside this: the panorama carries 8–10 principal
dimensions to 90 % of its variance and 150–158 distinct values per run, while the place
vocabulary reading it ends at 10–19 nodes of an allowed 40. The insertion gate, not the
input, sets the vocabulary's size; if the stack under-tiles at n = 20, the conditioning pass
(`min_insertion_error`, `dim_autocal_ticks`) precedes any verdict on the map.

## What round 2 hands the duck

The recipe's currency (a heading and a confidence per loop; a heading reference into the
duck's slot 10) is unchanged by these results. What changed is the arbitration question the
duck's rung-2 fork asks: the `efe` race can be made unit-consistent (A4) and still leaves a
hungry body playing; the round-3 candidates are precision-weighted arbitration (loops under
`LateralVoter` trust, register O3) and a pragmatic prior with reach — return to the remembered
site when hungry, its value not disconfirmed by one empty visit. The level-2 seed-averaged
harness (`mj_host/tools/l2_sweep.py`, register O12) is still the prerequisite for the port's
first A/B and is the round's last stage.

## Next

1. ~~n = 20~~ done: the stack is `NULL`; the (d) battery and R-B wait for a stack with a
   verdict worth perturbing.
2. **Round 3's lever is the arbitration for a hungry body with a memory**, the operator's
   call between (a) precision-weighted arbitration — the loops as channels under
   `LateralVoter` trust with the activity term (register O3, the duck's §17.4 fork) — and
   (b) a pragmatic prior with reach: return to the remembered site when hungry, its value
   surviving one empty visit (which needs the food-memory verdict, O9). Either is measured
   here, against the same specialist and floor, before anything ports.
3. `l2_sweep.py` (register O12) is independent of that choice and can go first.
