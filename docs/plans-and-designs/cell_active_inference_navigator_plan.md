# The Active-Inference Navigator — Staged Plan

## Context

The Cell must learn, tabula rasa, to survive in a maze: explore (or starve), build a map, and reach food. The driving force is **hunger**. The architecture is active inference made literal — expected free energy = **epistemic** (explore to reduce map uncertainty) + **pragmatic** (exploit scent to reach food), with hunger setting the gain on both.

This plan retires the oracle-perception violation (analytic `ScentCompass`/`VisualBearing` wired straight to action) by making each perceptual contribution **learned / inferred**, decomposed into **four independently-learnable pathways** that compose only after each works alone (the nav/action methodology).

Roles: **scent = local homing** (learned gradient-following), **vision = global mapping** (place-recognition), **IMU + compass = the metric frame**, **sliding-window = long-horizon planning** over the map.

Doctrine reference: `docs/brain_building_doctrine.md` — the reusable principles this build exercises. Read it first.

## Current state (2026-06-23)

- **Sensor-fusion PoC (substrate, branch cell-maze, not committed):** `VisualBearing` + `BearingFusion` via `LateralVoter`. Honest result — graceful fusion + dropout robustness; vision auxiliary in the scent-solvable maze; voter trust flat (degenerate baking). See [[v6-vision-scent-fusion-poc]].
- **Pathway A first attempt — `ScentHomingLearner`:** built + unit-tested; three real scale-traps (doctrine §6) found & fixed: (1) node-creation sensitivity (raw ring common-mode hid angular detail → 1 node; fixed by center+normalize → 24); (2) value-scale trap (Δscent swamped by exploration; fixed by reward whitening → spread 2–4); (3) hit-credit bug (fixed scent-drop threshold mis-fired → *reaching food punished*; fixed by crediting real `events.hit`). **Remaining wall:** still learns weakly (corridor ~2 hits/5min, corr≈0.14 vs HeadingPlanner ~150) — the **VQ of the raw ring destroys the angular structure** a clean directional state needs. Root cause = **perception**, the open-arena teacher-free limit. **Decision (operator): defer to the proven EPM for the percept** (not a hand-rolled VQ), and de-scaffold perception in the MAP where action-consequence exists.

## The four pathways

Each: honest learning signal · independent gate · existing module reuse.

### A — Learned scent-homing  ⟵ BUILD FIRST (retires the analytic-compass oracle)
- **Honest input:** the raw 8-nostril ring + scalar `reality.proprio.scent_max` (proximity). **No hand-computed gradient direction.**
- **Structure:** VQ the raw ring → categorical prototypes (inferred percept, reconstruction error = its own uncertainty). Per prototype, learn `V[proto][egocentric-heading] = EMA(Δscent_max over a commit window)` — i.e. discover *by acting* which heading climbs the gradient. Select via softmax + count-based epistemic bonus (persistent exploration). Hit-teleport credit (food respawns far on eat → big negative Δ → detect + credit positive).
- **Output:** learned egocentric heading `percept.scent_homing` → the de-hacked `HeadingController` (action layer unchanged).
- **Reuse (pattern):** VQ from `BearingEstimator` (but learned from **action-consequence**, not the analytic teacher — fixes its open-arena lesion-collapse, since the maze supplies the teacher-free signal); tabular V + commit + hit-teleport + softmax from `HeadingPlanner`. New self-contained module `ScentHomingLearner`.
- **Principle:** perception-as-inference (VQ'd ring) + action reduces own inferred error (climb Δscent). No oracle.
- **GATE:** recovers maze foraging with `ScentCompass` **deleted from the live loop**; learning is real (corr(chosen-heading, food-direction) rises early→late; shuffle ablation collapses it).

### B — Hunger-gated exploration drive (epistemic)
- When scent is flat/absent, go to the **frontier** (unvisited / high-prediction-error places). Curiosity = epistemic free-energy reduction; hunger sets urgency. Arbitration: scent present → A (home); scent absent → explore.
- **Reuse:** `DistressDrive` (boredom from no-progress), `MotivationGate` (hunger gain), homeokinetic exploration.
- **GATE:** covers the maze (doesn't orbit one region) when no scent; flips to homing the instant scent appears.

### C — Vision-led mapping via SACCADIC PLACE-CODES (where vision earns its keep)
Refined into a saccade→cylinder→map decomposition (insect snapshot / learning-walk model),
riding a shared temporal clock. Each sub-part has its own gate.

**C0 — Temporal-context clock (cross-cutting substrate, build with A's successor).**
`CPGOscillator → EPM → LateralVoter` — an intrinsic oscillator's reality token, voted
alongside everything, stamps every contribution with phase/when. **Proven load-bearing on
picrawler (RL standing only worked with it; ablation broke it).** Phase-lock the saccade to
it → clock phase ≈ sweep heading → the temporal coordinate becomes the cylinder index.
*Gate:* phase appears in consensus; ablation degrades a temporal-dependent downstream.

**C1 — Saccade / learning-walk behavior.** On arrival at novelty, pivot in place (~360°,
phase-locked to the clock), then resume. An **epistemic action** (move to perceive — the (b)
bar). *Gate (pure motor):* pivots smoothly, heading sweeps monotonically, resumes locomotion.
Keep the trigger dead-simple (novelty/arrival) — don't engineer the schedule (de-scaffold corner).

> **SHIPPED as a BOOTSTRAP REFLEX (acknowledged scaffold, committed 42812c7).** The pivot
> motor pattern is an acceptable innate primitive (like the clock/CPG). But the **trigger**
> (fixed distance-travelled) and the **arbitration** (gain-based MotorBus priority — why it
> yields to stuck-escape) are hand-designed scaffolds: the saccade fires on a timer, not on
> the agent's own state, so it does NOT yet satisfy bar (b). This is intentional — it's the
> chicken-and-egg bootstrap that GENERATES the panoramas to build the map; there's no
> uncertainty signal to drive an epistemic trigger until the map exists.
> **DE-SCAFFOLD TARGET (after C3/B):** replace the distance timer with the **map's own
> place-uncertainty / novelty (C3)** — scan *because* "I don't recognise this place" — and
> the gain-based arbitration with the **explore/exploit drive (Pathway B, EFE balance)**.
> Then the saccade becomes a genuine epistemic action (satisfies (b)).

**C2 — Cylinder builder (heading-indexed panoramic aggregator).** During a saccade, accumulate
per-tick `(heading, view)` into a heading-binned, **time-smoothed panorama** = the place
signature. Reuse a `KeyframeAverager` keyed by heading. *Gate (THE anti-aliasing test):*
pivoting at the same spot twice → the **same** embedding; two different spots →
**distinguishable** embeddings. This is the real test of the whole map idea.

**C3 — Place EPM (the map).** A slow EPM clusters cylinders → **zone-nodes**; bug-moves-
between-cylinders → **transition edges**; TLE = predict-next-place. The map is itself a
predictive model (the substrate D plans over). *Gate:* revisiting a zone re-fires its node
(loop closure); transition graph matches the maze topology.

**A place needs four axes** (doctrine §7): appearance (vision colour) + orientation
(heading, sin/cos) + metric/translation (**path-integration** of self-motion, NOT a depth
sensor — drift corrected by visual loop-closure) + temporal/transition context (the EPM's
"where I came from"). **Depth is motion-derived only** (optic flow / looming), never an
instantaneous range channel (that's lidar, a different creature).

**PREREQUISITE — perceptual distinctiveness.** "Twisty passages all alike" → every place
aliases to one node. The world must look different per zone: **zone-coloured walls** in the
raycast (cheap, deterministic, dense) as the primary place signal; **pillars** for maze
topology. *Nothing in C can pass its gate until this exists.*

### D — Sliding-window planning over the map (long-horizon)
- Pick a target node — a frontier to explore, or the **remembered** food location — and roll out a waypoint path over the place-graph.
- **Reuse:** `GNGRollout` over the graph + the slow `KeyframeAverager` sliding-window stream for temporally-extended objectives.
- **GATE:** given a learned map + target, produces and follows a valid waypoint path.

### Composition (the reasoning result)
Hungry → explore (B) → map forms (C) → scent picked up → home (A) → eat. Next time, map + planner (D) **route back toward remembered food / reason about which region to try** — solving a maze where reactive scent alone cannot, because food is far and you must navigate *to* the gradient. That is the path-reasoning result.

## Build order & gates (revised 2026-06-23)

The map (C) is now the spine, because perception de-scaffolds there. Order:

1. **World enrichment** — zone-coloured walls (raycast) + pillars. Gate: distinct visual signature per zone. *Prerequisite for everything in C.* ← NEXT
2. **C0 clock** — `CPGOscillator → EPM → voter`. Gate: phase in consensus + ablation degrades a temporal downstream. (Reuse, proven.)
3. **C1 saccade** — learning-walk pivot, phase-locked. Gate: smooth ~360° scan on arrival, resumes.
4. **C2 cylinder** — heading-binned time-smoothed panorama. Gate: same-spot→same-embedding, different-spot→distinguishable (anti-aliasing).
5. **C3 place-EPM** — slow EPM on consensus → zone-nodes + transitions. Gate: revisit re-fires node (loop closure); graph matches topology.
6. **B** — hunger-gated exploration drive (drives C1 to new zones). Gate: coverage + scent-switch.
7. **A′** — re-do scent-homing with an **EPM** percept (not the VQ), de-scaffolded against the now-richer signal / map. Gate: recover foraging without the analytic compass + prove learning.
8. **D** — planning over the map. Gate: waypoint route execution.
9. **Compose** — explore→map→remember→plan→home in a complex maze (the reasoning result).

## Cross-cutting risks (decide as we go)
- **Dead-reckoning drift** — the metric frame integrates IMU and drifts; vision loop-closure (C) corrects it. Prefer a **topological** place-graph over a metric grid (drift-tolerant).
- **Bootstrap chicken-and-egg** — A learns from scent samples but needs B to find scent. Resolve by proving A first in a **scent-reachable** maze (scent fills the space), then widen.
- **Opt-in / default-off** — every new module default-off so CartPole/MountainCar/picrawler stay byte-identical.

## Principle adherence (the a–d bar)
- (a) inferred-not-oracle: scent VQ'd + heading learned (A); vision = inferred place-map (C).
- (b) action reduces own inferred error: climb Δscent (A); reduce map uncertainty / reach frontiers (B); EFE-balanced explore/exploit.
- (c) loop-isolation: each pathway has a shuffle/ablation control + an independent gate.
- (d) perturbation→recovery: relocate food / drop a sensor → re-infer + re-act.

See [[v6-vision-scent-fusion-poc]] (the substrate this builds on) and [[v6-reasoning-breakthrough-resume]].
