# Open-items register — the one list of what is undecided, unbuilt, or unmeasured

*Started 2026-09-06 by the Cell system audit. Before this file, open items lived in fifteen
documents, several closed by later documents that never back-annotated the earlier one. This
is the single list. An item is `OPEN` (undecided), `DEFERRED` (decided to wait, with the
condition), `IN_FLIGHT` (being measured now), or `RESOLVED by <doc or commit>`. Each item has
one owning document, which is where its detail lives; this file carries only the state.*

*Companions: [`cell_system_audit_2026-09.md`](../reports/cell_system_audit_2026-09.md)
(what the audit found), [`loop_and_arbitration_recipe.md`](loop_and_arbitration_recipe.md)
(the recipe as built), [`cell_lever_ledger.md`](../reports/cell_lever_ledger.md) and
[`picrawler_lever_ledger.md`](../reports/picrawler_lever_ledger.md) (verdicts).*

| id | item | state | owning doc | discussed in |
|---|---|---|---|---|
| O1 | **Race vs fuse.** Fuse redundant estimates of one hidden state by precision; race policies with different goals. The rule is stated; whether fusing the redundant pragmatic loops (scent + sight) beats racing them is unmeasured. | `OPEN` — measure under the stuck camera, after Cell round 2 | recipe §4 | doctrine §3 (open); `fusion_notes.md` (resolved); `slow_loop_design_notes.md` (re-opened) |
| O2 | **Horizon.** The arbiter is one-step and greedy. A rollout horizon > 1 measured worse in v3; the slow-loop rollout is the proposed form. | `DEFERRED` until a slow EPM exists over a live consensus | recipe §3 | doctrine §2.2; `slow_loop_design_notes.md` |
| O3 | **Value race vs precision-weighted arbitration.** The shipped `efe` scoring is hunger × reach in shared units plus a need-gated epistemic term; whether that counts as the precision weighting doctrine §2.3 prescribes, or is the mis-scaled proxy §2.2 forbids, — A1 (the reach gate) is NULL under both seedings; the cause is the arbiter's units (audit V6), and the lever is `EFEArbiter.pragmatic_norm: planner_peak` (round 2 A4, built 2026-09-06, default off), then the loops under `LateralVoter` trust as the follow-on. The duck's rung-2 fork (§17.4) is the same question. | `IN_FLIGHT` (A4) | recipe §3 | doctrine §2.2–2.3; `cell_efe_arbiter_plan.md`; `microduck_rung2_regime_design.md` §17.4 |
| O4 | **Mitosis liveness.** The v4 EPM never called the gatekeeper; restored as `mitosis_gatekeeper` (default off), the absolute threshold 0.30 never splits on the bench. A rank- or spread-normalised threshold is the follow-up. | `OPEN` | `primitives/EPM.md` | Kalman charter Stage 4; audit L8 |
| O5 | **Level-N instantiation.** "Hierarchy is configuration" has no live config. One instance over a live consensus, measured, resolves it. | `OPEN` | `slow_loop_design_notes.md` | `CLAUDE.md` §0; `fusion_notes.md`; audit L7 |
| O6 | **The voter's novelty flag.** `LateralVoter.novelty_threshold` is parsed and never read; `ConsensusToken` carries no novelty field. Spawn gate 1 has no localizer until it is wired. | `OPEN` | `primitives/LateralVoter.md` | `slow_loop_design_notes.md`; audit L4 |
| O7 | **Where the epistemic term lives.** As built: in `PlayLoop`; the planner's own frontier-novelty term exists and is off (`planner_epistemic false`). Round 2 A3 turns it on. | `IN_FLIGHT` (A3, queued) | recipe §3 | `cell_efe_arbiter_plan.md` vs `cell_play_loop_plan.md`; audit L1 |
| O8 | **The compass scaffold.** The published heading is the world yaw by construction (drift-free); the grid maps depend on it. Named in the recipe; the heading-drift perturbation measures the dependence. | `IN_FLIGHT` (round 2 (d) battery) | recipe §2 | audit S1, S7 |
| O9 | **Food memory: reward or interoception?** `PlaceGraphPlanner` value-iterates a fixed increment on the ground-truth eat event, never decays it. | `OPEN` — operator verdict | recipe §2 | audit V3 |
| O10 | **EPM-native place map.** Replace the grid over the path integral with the place EPM's winner as the node id (`PlaceVectorBuilder` → `epm_place` → `pi_cell_size 0`). | `IN_FLIGHT` (round 2 A5) | Cell round 2 charter | audit V1 |
| O11 | **Seek-and-fetch on the duck.** No design and no sensor. The recipe carries the design note: a ToF-derived object bearing as the smallest legal sensor; touch as a named scaffold; fetch as the planner's return leg under the duck's interoceptive prior. | `DEFERRED` until the sensor exists | recipe §6 | `microduck_port_plan.md` |
| O12 | **Duck level-2 seed-averaged harness.** Every rung-2 verdict was read at seed 2. `mj_host/tools/l2_sweep.py` is round 2's last stage. | `IN_FLIGHT` (queued) | Cell round 2 charter | audit K1 |
| O13 | **Bake-threshold default merge.** Schema 50, effective 100 when omitted (EPM, SequenceGNG); merging moves 105 instances, so it is a lever, not hygiene. | `DEFERRED` — a lever with a byte-identity cost | `primitives/EPM.md` | Kalman charter Stage 0; audit L5 |
| O14 | **Three more default traps.** `WhiskerSteerReflex.steer_gain` 8→10, `PlaceNav.arrival_window` 30→60, `SaccadeReflex.scent_gate` 0.05→1e9; pinned in `test_schema_defaults_effective`. | `DEFERRED` — each a one-line lever | audit L6 | — |
| O15 | **Dropped config keys.** 681 across the shipping configs; warned at load and in the builder; the census test asserts clean on a growing set. The config pass removes them from the Cell study and round-2 configs first. | `IN_FLIGHT` (config pass) | audit L3 | — |
| O16 | **`primitives/_overview.md` refresh.** Self-declared stale since 2026-05-23; still the only module taxonomy. | `OPEN` | `primitives/_overview.md` | — |
| O17 | **Scale-tuned constants.** `min_insertion_error 0.1`, `node_ref 4.0`, `pi_cell_size` (units of speed⁻¹·tick), mitosis 0.30. | `OPEN` | audit V5 | doctrine §5 rule 5 |
| O18 | **The klino leave-one-out lever.** Clearing `scent_topic` leaves `g_prag_klino = hunger·cap`; a clean lever (a `klino_weight`, or `force_policy`) is needed before the next leave-one-out. | `OPEN` | audit H5 | Cell report §2 |
| O19 | **The shuffle null.** `force_policy shuffle` re-draws every tick; a fair null holds a random policy for the arbiter's own dwell. | `OPEN` | audit H4 | — |
| O20 | **Untested modules.** `HeadingController` (every loop's action layer), `ScentCompass`, `MotivationGate`, `GoalBelief`; on the planner `pi_cell_size`, `escape_gain`, `disconfirm`. | `OPEN` | audit T | — |
