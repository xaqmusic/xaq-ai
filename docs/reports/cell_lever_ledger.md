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
| **A2 — the stall latch off** (`PlayLoop.wander_stall_ticks=0`); A2b + `frontier_bias=1.0` | far-food arena + pillars (`the_cell_route_far_pillars.json`), n = 6 paired worlds, 240 s, promote-or-kill | **`PARTIAL`** — the mechanism is restored (climb fraction 0.0 → 0.2, forced wander 0.3 → 0.0; A2b frontier bearing engaged on 0.5 of samples) with no foraging effect: eats 1.7 → 1.8 / 1.8, Δ −0.17 ± 2.0; time-to-return worse on the 2 paired returns. A signal, not a finding. | A climbing play with the grid map treadmills freshly baked cells (the wander-beyond fix's own premise); re-use with the EPM-native map (A5), where "novel" means a place the vocabulary has not seen. |
| **A3 — the planner's epistemic term on** (`EFEArbiter.planner_epistemic=true`) | same, n = 6 | **signal, promote to n = 20** — eats 1.7 → 2.7 (Δ +1.0, sd 1.9, 4+/2−), the planner's share of decisions 0.056 → 0.288, eats after the first relocation 1.0 → 1.7 (5 of 6 runs returned). Consistent with audit V6: the one normalised term the planner can carry lets it compete. | — |
| **A4 — the planner's reach peak-normalised** (`EFEArbiter.pragmatic_norm=planner_peak`; default `none`, byte-identical; 31/31 arbiter tests) | far-food arena + pillars, n = 6 then 20 | `IN_FLIGHT` — built 2026-09-06 after A1's null located the crowding in the arbiter's units (audit V6). The first form (every reach by its own peak) failed its own test: a constant weak scent read as full reach for klino; klino keeps its eat-calibrated reach. | — |
| **A0 — the harness itself** (legacy vs repaired seeding) | study room, full / no-play / gate + the specialist, n = 20 each | legacy arm reproduces the Kalman-campaign numbers exactly (full 0.6 / no-play 2.0 / specialist 1.9). **Repaired seeding** (20 distinct worlds, every RNG varied): specialist 2.30, full 0.75, no-play 1.65, gate-off 0.60; full − no-play −0.90 ± 1.48 (t −2.7) vs −1.40 ± 0.82 (t −7.6) legacy; no-play − specialist −0.65 ± 1.84 (t −1.6). **The report's directions stand at about two-thirds the effect and twice the spread; the no-play tie becomes a non-significant trend below the specialist.** | Every prior Cell number is a one-world number; cite the repaired ones from here on. |

### Regime notes

- **2026-09-06, the far-food arena's premise.** The reactive specialist eats 3.7 in 240 s
  there (n = 6; 2.7 after the first relocation, returning in ~3000 ticks on 5 of 6 runs), so
  at scent falloff 6 the field still reaches across the gap and "blind after eating" is not
  yet true; the regime does not yet *demand* a map. Calibrate the falloff against the scent
  reach measured from the logs before any n = 20 battery counts (P0, in flight).

### Process

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
- **2026-09-06, three liveness runs raced on one temp config name** (the first version of
  `cell_liveness.py` used a fixed tag); fixed to a per-process tag before any result was read.
