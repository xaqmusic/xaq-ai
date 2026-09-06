# Cell lever ledger — what is promoted, what is refuted, and what the harness did to us

*The per-lever verdict record and process log for the Cell navigator. **Read this before
proposing a Cell lever.** Companions: the study
[`cell_markov_blanket_loops_report.md`](cell_markov_blanket_loops_report.md) (with its errata
appendix), the system audit [`cell_system_audit_2026-09.md`](cell_system_audit_2026-09.md)
and its [claim register](cell_system_audit_2026-09_appendix.md), the canonical
[loop and arbitration recipe](../plans-and-designs/loop_and_arbitration_recipe.md), the
[open-items register](../plans-and-designs/open_items_register.md), the picrawler's
[ledger](picrawler_lever_ledger.md) (whose header this one borrows), and
[`CLAUDE.md`](../../CLAUDE.md) §3 (the A/B protocol these verdicts were produced under).*

**Started 2026-09-06** (the cell system audit). Verdicts from the 2026-09-05 Kalman-lessons
campaign are mirrored in §1 from its charter; verdicts from the 2026-07 study are in the
report and are **not** repeated here except where the audit changed their reading.

---

## How to read this ledger

> **Nothing in this ledger is dead.** Every negative entry means *refuted in the context, at
> the power, and against the baseline stated* — never "this idea does not work." A refuted
> lever whose re-use context arrives is a lever to try again.

- **A verdict is scoped to its SCENARIO and its BASELINE.** The study room (40 m, pillars,
  scent-poor), the far-food arena (24 m, two sites beyond scent reach of each other) and the
  L-bend are three regimes; a verdict in one says nothing about the others.
- **Every Cell battery before 2026-09-06 ran in ONE pillar layout with ONE body / arbiter /
  respawn RNG** (`OGMA_SEED` was fixed at 42 for every job; only play's exploration seed and,
  under `--vary-world`, the two food sites and the spawn varied). Those numbers are paired
  food-layout draws in a fixed world, not draws from a world population. Re-measure with the
  repaired harness before building on them (`cell_coverage.py` without `--legacy-seeding`).
- **Nothing counts until the config passes `cell_liveness.py`.** Twelve EPMs across six
  configs were fed zeros for months; a study config shipped with its vision loop weighted to
  zero. Eats cannot see either.
- **Every lever ships gain-0-guarded** and refuted levers are kept default-off, not removed.

### Verdict vocabulary

| Verdict | Meaning |
|---|---|
| `BASELINE` | in the deployed stack |
| `WORKING` | real positive effect, kept but not in the default stack |
| `PARTIAL` | effect on secondary metrics, null/regression on the primary |
| `NULL` | no measurable effect **at the power and against the baseline stated** |
| `REGRESSION` | measurable negative effect |
| `TAUTOLOGY` | the variant was byte-identical — the mechanism was already on |
| `DEAD_CODE` | no effect because the code path wasn't live in that config |
| `ABLATED` | actively removed for negative consequences |
| `DEFERRED` | built but never tested at adequate power |
| `IN_FLIGHT` | under test now |

The audit's claim statuses (`UNSUPPORTED`, `DEAD_PARAM`, `DROPPED_PARAM`, `DEFAULT_TRAP`,
`SCAFFOLD_UNNAMED`, `HARNESS_INVALID`, `CONFIG_BROKEN`, `MISATTRIBUTED`, `RULE_VIOLATION`,
`OPEN`) live in the [claim register](cell_system_audit_2026-09_appendix.md); they describe
claims and harnesses, not levers.

---

## 1. Verdicts mirrored from the Kalman-lessons campaign (2026-09-05)

All in the study room unless stated; n = 20 paired worlds under the **legacy seeding** (one
pillar layout), 240 s; full detail in
[`epm_kalman_lessons_plan.md`](../plans-and-designs/epm_kalman_lessons_plan.md).

| lever | config / arm | verdict | re-use context |
|---|---|---|---|
| per-node Kalman gain, cap 0.05, on the two maze-fusion EPMs | `the_cell_maze_fusion.json` as shipped | `DEAD_CODE` — the EPMs were zero-encoded (`proprio_state_dims 2` vs 3 emitted values) | any EPM lever on the maze configs needs `__dims3` |
| the dims repair itself | `the_cell_maze_fusion__dims3.json` vs the broken config | `NULL` (eats −0.30 ± 0.96) — a live 0.2/0.8 trust split changes nothing on an intact forager | measure fusion levers under damage, not on the intact forage |
| Kalman gain cap 0.05 on the repaired fusion EPMs | `__dims3` | `NULL` (eats +0.95 ± 1.53, t 1.3) | under the vision dropout, where trust decides who steers |
| Kalman gain cap 0.05 on the study room's place + vision EPMs | `the_cell_arbiter_room_pillars_vision.json`, full and minus-play | `NULL` both (+0.05 ± 0.42; −0.15 ± 0.55, food distance −1.27 ± 1.78 trending closer) | a regime that demands a map (round 2), or the dropout |
| voter activity term (`activity_gain 1`) | `__dims3` + stuck camera at tick 4800 | **`WORKING`** — recovered 78 % of the eats the stuck camera cost (+4.6 ± 2.1); the sign control (`activity_gain −1`) inverted it | any fusion with a channel that can freeze |
| inverse-variance expected-error trust (`trust_source expected`, `trust_power 2`) | `__dims3` + noisy scent (σ 0.1 at 4800) | `NULL` — no second live channel (vision blind most of the run) | a fusion with two live channels |
| log-probability transition surprise fed to play (`novelty_source transition_surp`) | study room, full composition | `NULL` — play's climb fraction 0.0 in every arm, so what play is handed as novelty cannot matter | a play loop whose climb fires (round 2 arm A2) |

## 2. Audit-era entries (2026-09-06 →)

Verdicts on levers go here as round 2 runs them; this section also carries the **process
mistakes** REPORTS.md §5 keeps out of reports.

### Levers

| lever | config / arm | verdict | re-use context |
|---|---|---|---|
| **A1 — the reach gate off** (`EFEArbiter.epistemic_reach_gated=false`, the legacy `1 − hunger` gate the report §4 asks for) | study room, n = 20 paired, 240 s, both seedings: legacy full 0.6 vs gate-off 0.5; repaired 0.75 vs 0.60 | **`NULL`** twice — legacy Δ +0.10, sd 1.07, t 0.42; repaired Δ +0.15, sd 1.14, t 0.59; play's share 0.91 → 0.82 in both; food distance tie | The crowding is a units problem (audit V6): pragmatic terms peak near 0.04, play's at 1. Re-use: none for the gate; the lever is the arbiter's normalisation (register O3). |
| **A2 — the stall latch off** (`PlayLoop.wander_stall_ticks=0`); A2b + `frontier_bias=1.0` | far-food arena + pillars, n = 6 paired worlds, 240 s, at falloff 6.0 and again at 2.0 | **`PARTIAL`** — the mechanism is restored (climb 0.0 → 0.2–0.3, forced wander → 0.0; A2b's frontier bearing engaged on 0.5 of samples) with at most a weak foraging signal: falloff 6.0 eats 1.7 → 1.8 (Δ −0.17 ± 2.0); falloff 2.0 eats 1.0 → 1.5 (Δ +0.5, sd 1.05, 3+/1−). **n = 20 (falloff 2.0): 1.60 vs 1.30, Δ +0.35, sd 1.18, t 1.3; eats after relocation +0.25 ± 0.91 — a trend inside its interval.** | A climbing play with the grid map treadmills freshly baked cells (the wander-beyond fix's own premise); re-use with the EPM-native map (A5), where "novel" means a place the vocabulary has not seen. |
| **A3 — the planner's epistemic term on** (`EFEArbiter.planner_epistemic=true`) | same, n = 6 at falloff 6.0 and 2.0 | **`NULL`** — the falloff-6.0 signal (eats 1.7 → 2.7, Δ +1.0, sd 1.9) did not replicate at 2.0 (1.0 → 0.8, Δ −0.17, sd 1.33) although the planner's share of decisions rose both times (0.056 → 0.288; 0.029 → 0.438). A normalised epistemic term buys the planner the motor, not eats: what it routes over (the grid, the food memory) is the limit. | With the vocabulary map; and as the term the play plan moved out of the planner (register O7). |
| **A4 — the planner's reach peak-normalised** (`EFEArbiter.pragmatic_norm=planner_peak`; default `none`, byte-identical; 31/31 arbiter tests) | far-food arena + pillars, falloff 2.0, n = 6 paired | **`NULL` alone** — the planner's share of decisions 0.029 → 0.075, and the eats are identical on all six worlds ([2,0,1,1,0,2]): the decisions it added came after the eats that occurred. Built after A1's null located the crowding in the arbiter's units (audit V6); the first form (every reach by its own peak) failed its own test (a constant weak scent read as full reach for klino). | With the vocabulary map (the A5 stack, below), where the planner has more to route over. |
| **A5 — the EPM-native place map** (`PlaceVectorBuilder` → `epm_place` 48-D → play/planner at `pi_cell_size 0`; `the_cell_route_far_pillars__a5_epm_map.json`) | far-food arena + pillars, falloff 2.0, n = 6 paired (n = 20 on the stack in flight) | **alone: mixed** — eats 1.0 → 1.33 (+0.33, sd 1.37) with food distance *worse* on 6 of 6 worlds (+1.17 m, t 3.4); the vocabulary holds ~19 nodes and rarely grows (stale ~700 samples). **Stacked A5+A4: +0.67 (sd 1.86, 4+/2−); A5+A4+A2: +0.50 (sd 0.84, 2+/0−, four ties), the planner's share 0.029 → 0.123, return legs 3 of 6.** **n = 20 on the stack A5+A4+A2: 1.35 vs 1.30, Δ +0.05, sd 1.05, t 0.2, 7+/5− — `NULL`; the n = 6 trend was noise.** The vocabulary held 17.6 nodes and rarely grew (the insertion-gate collapse, audit V4); the conditioning pass precedes any second attempt. Liveness 2026-09-06 (120 s): the vocabulary holds 16 baked nodes, top winner 0.22 (against the base's 8 nodes / 0.55), play has a novelty route on 62 of 121 samples (base 30) — and climbs on none, because the stall latch counts *vocabulary* growth, rarer than grid growth; the planner scores routes on 51 samples and wins none without A4. So A5 is measured alone and stacked (A5+A4, A5+A4+A2). | — |
| **no play** (`EFEArbiter.play_weight=0`, the operator's fallback) | far-food arena + pillars, falloff 2.0, n = 20 | **v1-klino composition:** 1.05 vs base 1.30, below the floor (a weaker chemotaxis module cannot earn the first eats the memory is built from). **Study-brain composition: `WORKING` — 2.45 vs base 1.50 (+0.95); beats the gradient-blind floor (+0.85, sd 1.95, t 2.0, 12+/5−; food distance −1.70 m, t −3.1) and ties the specialist (−0.35, sd 1.93, t −0.8); the planner takes 0.61 of decisions and the body returns to the relocated food on 14 of 20 runs.** The memory works; play is the cost. | The bar the precision lever must reach with play present. |
| **R3 — precision-weighted arbitration, corrected** (`group_balance false`; gate on) | study-brain base, far-food + pillars, falloff 2.0, n = 20 | **`NULL`** — 1.30 eats vs base 1.50 (Δ −0.20, sd 1.44, t −0.6); 1.15 below the two-loop brain (t −3.2); 1.50 below the specialist (t −4.4). The mechanism now operates: the planner's trust swings 0.27–0.66, klino's flat channel reads 0, the planner takes **0.31** of decisions (the race gave it 0.06) — and the body still plays 0.69 while hungry and does not eat more. **Trust in a loop's own steadiness is not competence:** the planner is trusted whether or not its route leads to food; play's smooth wander earns trust for being smooth. | The competence form: each loop publishes its own prediction error about the world (klino: scent rising while running; planner: distance-to-goal falling; play: novelty gained), and the voter grades *that* (register O21). |
| R3 — pure form (`informativeness_gain 0`) | same | **`REGRESSION` trend** — 0.95 eats (Δ −0.55, sd 1.61, t −1.5; food distance 1.22 m worse than the two-loop brain, t 2.7): without the gate the flat klino channel (tle 0) is the most trusted — the low-TLE trap, by design. | — |
| R3 — wrong-sign control (`precision_sign −1`) | same | 1.30, identical share structure (planner 0.31, play 0.67): **degenerate as a control here** — with two live channels whose trusts sum to one, 1 − t just swaps which trust multiplies which preference. A valid control needs three live channels or a fixed-trust arm. | — |
| **R4 — competence-graded arbitration** (`LoopCompetence` per loop → `LateralVoter` level 1, `trust_source expected`, `group_balance false` → `scoring_mode precision`; `the_cell_route_far_pillars__r4_competence.json`) | study-brain base, far-food + pillars, falloff 2.0, n = 20 | `IN_FLIGHT` — built 2026-09-06 (4/4 unit tests; module absent = byte-identical). Competence = the fraction of 30-tick driving windows on which the loop's own objective moved as it predicts; stagnation counts as failure; non-drivers relax to 0.5. | — |
| **the study brain in the far-food arena** (the corrected base, n = 20) | falloff 2.0, pillars | base 1.50 (ties the floor 1.60; 1.30 below the specialist 2.80, t −2.9); A2 1.55 (`NULL`); the stack A5+A4+A2 1.30 (`NULL`, Δ −0.20 ± 1.64). The v1-klino numbers above are superseded for any composition-vs-specialist claim. | — |
| **R3 — precision-weighted arbitration, first form** (`EFEArbiter.scoring_mode=precision`; per-loop EPMs → `LateralVoter` level 1 with the fusion testbed's `informativeness_gain 1.0` → preference × trust selection) | study-brain base, far-food + pillars, falloff 2.0, n = 20 | **`NULL` — uninterpretable as built:** 1.95 eats vs base 1.50 (+0.45, sd 1.23, t 1.6; food distance better on 16 of 20) but **indistinguishable from its wrong-sign control** (1.90; planner 0.0 of decisions in both, klino 0.19 / play 0.81 in both) and below the no-play bar (2.45, Δ −0.50, t −1.4). The planner never wins because the voter's informativeness gate, built to strip a dead sensor whose vocabulary never grows, strips a policy whose bearing is steady or zero most of the time — steadiness is a virtue in a policy, a defect in a sensor. `activity_gain 1.0` produced identical runs (the planner already at zero trust). The +0.45 over the base is klino winning more (0.19 vs 0.05), not the mechanism. | Superseded: the trust was uniform by construction (the group-balance trap, process note below); see the corrected rows. |
| **A0 — the harness itself** (legacy vs repaired seeding) | study room, full / no-play / gate + the specialist, n = 20 each | legacy arm reproduces the Kalman-campaign numbers exactly (full 0.6 / no-play 2.0 / specialist 1.9). **Repaired seeding** (20 distinct worlds, every RNG varied): specialist 2.30, full 0.75, no-play 1.65, gate-off 0.60; full − no-play −0.90 ± 1.48 (t −2.7) vs −1.40 ± 0.82 (t −7.6) legacy; no-play − specialist −0.65 ± 1.84 (t −1.6). **The report's directions stand at about two-thirds the effect and twice the spread; the no-play tie becomes a non-significant trend below the specialist.** | Every prior Cell number is a one-world number; cite the repaired ones from here on. |

### Regime notes

- **2026-09-06, the (d) battery on the two-loop brain (far-food, n = 12 varied worlds, the
  lesion over the middle third, paired by world):** scent noise (σ 0.05) — approach +1.47 m
  worse than the control at the lesion phase (sd 2.32, t 2.2, 8+/4−), level afterwards: the
  scent sense is load-bearing and the loop recovers (`WORKING` as a (d) signature, signal
  power). Heading drift (random walk, ≈ 40° by the window's end) — in-window approach +0.32 m
  (t 0.4), post-window eats −0.25 (sd 1.36, t −0.6): **`NULL`**; a slow compass drift that
  snaps back does not touch this brain at this power. Re-use: a persistent bias, or the
  grid-vs-vocabulary maps compared under drift.

- **2026-09-06, the transfer room (R-B, the L-bend, n = 20 varied worlds):** two-loop brain
  2.10 eats vs the four-loop brain 0.95 (paired +1.15, sd 1.66, t 3.1, 13+/3−; food distance
  −2.73 m, t −3.0; eats after the relocation +0.80, t 2.7). Specialist 2.10 (= the two-loop
  brain, Δ 0.00 ± 1.12), floor 0.95 (= the four-loop brain). **The far-food finding transfers:
  dropping play is `WORKING` in both regimes at n = 20; the four-loop brain is at the
  random-walk floor in both.**

- **2026-09-06, the finding at n = 20 (varied worlds, falloff 2.0):** specialist 2.80 eats,
  gradient-blind floor 1.60 (the specialist beats it by 1.20, t 2.5, returning after the
  relocation on 16 of 20 runs against 8), the composition 1.30 (ties the floor; 1.50 below the
  specialist, t −4.0, worse on 16 of 20 worlds), the stack 1.35 — **on the v1-klino base;
  superseded** by the study-brain base (below): base 1.50, no-play 2.45. **In a regime that
  demands memory the composition's memory works once play is out of the race**: with play,
  play holds 82 % of decisions while the body is hungry (register R6) and the composition
  forages at chance; without it, the planner takes 0.61 of decisions and the composition ties
  the reflex. Re-use context for every lever above: an arbitration that silences play when a
  hungry body has a remembered site (round 3), or the two-loop brain.

- **2026-09-06, the far-food arena's premise.** The reactive specialist eats 3.7 in 240 s
  there (n = 6; 2.7 after the first relocation, returning in ~3000 ticks on 5 of 6 runs), so
  at scent falloff 6 the field still reaches across the gap and "blind after eating" is not
  yet true; the regime does not yet *demand* a map. Measured from the base arm's logs, the
  screened-Poisson field falls from 0.44 at the food to 0.01 at the far wall and never
  reaches zero, so a run-and-tumble climber always has a gradient.
- **Calibration (n = 6 paired worlds, specialist vs the gradient-blind floor
  `RunTumbleNavV2.ablation=shuffle`):** the floor eats 1.8 at every falloff (it ignores
  scent); the specialist eats 3.7 at falloffs 6.0 and 3.5 (return ~3000 / 3900 ticks) and
  3.0 at 2.0 (return 4966 ticks vs the floor's 5927; 2.0 eats after the first relocation).
  **Falloff 2.0 is the round-2 base**: the first eat is still earned locally, the return
  leg is nearly chance for the reflex, so a map-user's return would be loud. The A2/A3
  n = 6 signals above were taken at falloff 6.0 and are re-run at 2.0 in the lever battery.

### Process

- **2026-09-06, the round-2 base's klino was not the study brain's (CLAUDE.md §3.2 check 6,
  faithfulness).** `the_cell_route_far_pillars.json` was derived from the old far-food config,
  whose klino is `RunTumbleNav` (v1); the study brain and the reactive specialist run
  `RunTumbleNavV2`. Every far-food composition-vs-specialist number above this note
  (base 1.30, no-play 1.05, the stack 1.35 against the specialist's 2.80) conflates the
  arbitration with a weaker chemotaxis module. Caught by reading the module graphs side by
  side after the no-play arm failed to help. The base is rebuilt from the study brain's
  sixteen modules (`the_cell_route_far_pillars.json`; the old one kept as
  `_v1klino.json`), the A5 arm regenerated from it, and the n = 20 battery re-run; the
  v1-klino numbers stay recorded as what they are. Instrument gap: `cell_liveness.py`
  should print the module graph (id:type) so a base's composition is read, not assumed.


- **2026-09-05, Kalman charter, misattribution (corrected in the audit).** The charter wrote
  that "the vision-dropout recovery in the Cell report ran through `BearingFusion`'s own
  `confidence_floor`, not the EPM → voter path". The report's (d) test did not use the
  maze-fusion configs at all: its figure is hardcoded from a run of
  `the_cell_vision_dtest.json` (pillar-free, 40 m, no voter, no `BearingFusion`, `epm_vision`
  alive at dims 3). The zero-encode finding stands for the six maze-fusion configs; the
  sentence about the report's (d) test does not. Lesson: **name the config behind a figure
  before attributing its mechanism** — the figure script hardcodes numbers and the raw file
  names arms, not configs.
- **2026-07 → 2026-09-06, harness seeding.** `cell_coverage.py` passed a literal `42` as the
  world seed for every job and the world script never read the `obstacle_seed` the harness
  wrote into metadata, so every `--vary-world` battery (the report's n = 20 and the Kalman
  campaign's) ran in one pillar layout. Found by tracing the seed, not by any metric.
  Instrument: the seed manifest every job now prints, and `cell_liveness.py`'s seed line.
- **2026-09-06, the round-3 voter's trust was uniform by construction (CLAUDE.md §3.2
  check 5, consumer fired?).** `LateralVoter` derives a channel's group from the first path
  segment after its `input_pattern`; with a one-level pattern (`reality.loop.`) every channel
  is its own group, and `group_balance` (default true) gives each group an equal 1/N — an
  exact 0.25 to all four loops every tick, whatever their errors. The first two round-3
  batteries measured preference alone. The fusion testbed's config had set `group_balance:
  false` for exactly this reason, in its own `_comment`; the round-3 config copied its pattern
  depth and not its flag. Found by putting the arbiter's trust inputs into the diagnostic
  stream (now permanent: `tk`/`tp`/`tpl`/`tkeys`/`tupd`). Lesson: **copy a working testbed's
  module params whole, then diff, rather than re-deriving them**; and a lever whose control arm
  cannot be told from the lever has not operated (§3.2), whatever its eats say.
- **2026-09-06, three liveness runs raced on one temp config name** (the first version of
  `cell_liveness.py` used a fixed tag); fixed to a per-process tag before any result was read.
