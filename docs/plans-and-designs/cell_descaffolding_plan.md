# Cell — De-scaffolding plan: turn the heading-follower into genuine cognition

**Premise (from `docs/findings/cell_heading_follower_milestone.md`):** the current forager is a
*learned heading-following locomotor primitive* (bar-(b) at the motor level) wrapped in hardwired
scaffolds. This plan removes each scaffold and replaces it with a learned/cognitive element, in
priority order — attack the cognitive core first; defer the defensible locomotor/perception primitives.

**Discipline (per `feedback-active-inference-discipline`, `feedback-no-tuning`):** every stage (a)
keeps the change **opt-in** so the prior rung stays reproducible/byte-identical, (b) ships with an
**ablation control** (the confound-killer), (c) names the **bar-(a–d) rung** it advances, and (d)
prefers an **adaptive mechanism over a tuned constant**. Each stage is gated *before* the next stacks.

**METHODOLOGY (operator, 2026-06-21) — de-scaffold piece-by-piece, IN PLACE, against a KNOWN target.**
The current heading-follower is behaviorally the chemotaxis reflex (~298 eats/10min, genuine approach) —
a clean, fully-understood baseline. The de-scaffold proceeds by replacing ONE scaffold at a time with a
LEARNED element, **in the same tractable open/corridor arena**, and the success bar for each piece is:
*after the learned element converges, the bug RECOVERS the same foraging behavior the scaffold gave us
(~298 eats, genuine turn-and-approach).* Recovery ⇒ the learned element genuinely learned what the
scaffold hardwired (replacing scaffold with cognition, behavior preserved). Failure-to-recover ⇒ that
learning problem is the blocker, cleanly isolated against a known target. **Do NOT introduce the maze
(or obstacles) during per-piece de-scaffolding** — it is an order of magnitude harder and would confound
every measurement. **The maze is the FINAL step**, run only after the system is fully de-scaffolded, as
the *transfer falsifier*: a genuinely model-based/belief-based cognition transfers; a re-derived reflex
breaks on the topology (and the gradient-reflex control arm provably can't route). Obstacle perception +
routing (Stage, below) is introduced just before the maze, also in a controlled (sparse-pillar) arena
with the same recover-the-behavior bar, not the full maze.

The two systems share only the **heading interface** (setpoint down, *achieved* heading up), so they
learn **separably**: the follower learns from its own setpoint-tracking error; a learning planner must
credit the **achieved** heading (efference copy), never the commanded one — so the follower's
bootstrap mis-execution can't poison the planner.

---

## Scaffold inventory → replacement (priority order)

### Stage 1 — Heading **SELECTION** + **BELIEF**: gradient reflex → learned model-based planner  ⭐ the cognitive core
**Scaffold:** desired heading = the *instantaneous* scent up-gradient bearing (a hardwired chemotaxis
reflex; "where to go" is given, not reasoned, and **has no memory**).
**Belief (Stage 1a) — BUILT (`GoalBelief`, commit 6976b93).** Path-integration goal memory (world-frame
goal via heading). Status: an **identity passthrough in the open corridor** (the scent is never lost
here, so there is nothing to bridge) — correctly built but ahead of its gating env (the maze's
short-range diffusion is where it earns its keep). NB the earlier "belief fixes the pillar oscillation"
gate was RETRACTED: with normalized/transparent scent there is no signal LOSS at a pillar (the pillar is
a physical-obstruction + a per-nostril bearing-corruption problem, not a memory problem — see env-fidelity
+ obstacle stages). So the belief's real gate waits for the maze.
**Replacement (Stage 1b — the learned selection, NEXT):** a **coxswain** that *learns* the consequence
of choosing a heading — a forward model `P(Δpreferred-obs | heading)` over candidate headings — then
**selects** the heading maximizing predicted progress (EFE), on top of the belief. In the open arena
this *recovers* gradient-following but as a **learned, model-based** decision. Credits the **achieved**
heading (efference copy from the follower) → separable from the controller during bootstrap.
**Gate (recover-the-behavior, in place):** after the heading-value model converges, the learned planner
**recovers ~298 eats / genuine approach in the open corridor** (matches the gradient reflex) — proving
the learned selection genuinely learned "where to go." **Ablation:** shuffled/wrong-sign/lagged heading →
foraging collapses (bar-(c)); freeze the learned model → reverts to chance headings.
**Advances:** bar-(b) cognitive (the decision is learned + model-based, not a hardwired gradient).

### Stage 2 — **MOTIVATION**: decorative drive → homeostatic active inference
**Scaffold/gap:** `HomeostaticDrive`/`NeurochemState` are present but unused — the bug forages
reward-free, regardless of hunger.
**Replacement:** ground the coxswain's preference in the **homeostatic deficit** (energy/hunger): the
preferred observation is the *fed/low-deficit* state; foraging is driven *because* hungry, and the
agent rests/explores when sated. No external reward shaping — intrinsic homeostatic value (the H-JEPA
loop). The planner's value = predicted homeostatic-error reduction, not a sparse eat-counter.
**Gate:** behavior is modulated by need (forage-rate rises with hunger; sated → explore/idle), and
energy is sustained without starvation. **Ablation:** freeze the drive (constant urgency) → loses the
need-modulation.
**Advances:** bar-(b) grounded in intrinsic need (the project thesis: homeokinetic, not reward-driven).

### Stage 3 — **PERCEPTION**: analytic gradient → inferred latent
**Scaffold:** the bearing is a hand-coded vector-sum reduction (ScentCompass), not a learned generative
model.
**Replacement:** let the (now direction-selective) **bearing EPM** infer the bearing latent with its
own prediction error (dual-TLE), and have the coxswain plan over the *latent* rather than the analytic
gradient. The hand-coded compass remains as an honest scaffold/​bootstrap (same status as trusting the
raw sensor) until the EPM latent matches or beats it.
**Gate:** coxswain on the EPM latent ≥ coxswain on the analytic gradient. **Ablation:** lesion the EPM →
fall back to the gradient (graceful).
**Advances:** bar-(a) (inferred, not analytically-given).

### Stage 4 — **LOCOMOTOR POLICY**: hand-designed braking gate → learned control
**Scaffold:** the turn-in-place ↔ charge braking gate + `turn_fraction` are a hand-designed control
law (on top of the genuinely-learned `k_body`).
**Replacement:** learn the full alignment/advance policy from the body's dynamics (extend the
homeokinetic motor model, or a learned gate), so "brake-turn-then-charge" *emerges* rather than being
prescribed. **Lower priority** — the braking gate is a defensible locomotor primitive; this is
rigor/polish, not a blocker.
**Gate:** learned policy ≥ the hand-designed gate's foraging, without the hand constants.
**Advances:** bar-(b) deepened (more of the motor stack is learned).

### Env fidelity (prereq for Stage 5) — scent must DIFFUSE, not occlude like light
**Verified 2026-06-21 (pillar diagnostics):** the pillar lock-up is TWO independent problems, both
real:
1. **Sensory — wrong scent model.** `_scent_at` raycasts food→sample PER NOSTRIL and attenuates ×0.30
   (`obstacle_scent_attenuation`) on a pillar hit — treating a *chemical* like *light* (line-of-sight
   shadow). A pillar then occludes some nostrils' rays but not others → the gradient swings toward the
   un-occluded side → the BEARING points wrong (corr to true food 1.0 → 0.55), with magnitude staying
   confident (so a magnitude gate misses it). **Fix:** model scent as DIFFUSION. Near-term (sparse
   pillars): transparent (`obstacle_scent_attenuation=1.0`) restores corr 1.0. **Maze: a 2D-GRID
   diffusion map** — discretize the floor, walls=barriers, food=sources, relax the heat/Laplace eqn over
   FREE cells only (few Jacobi sweeps), sample (bilinear) in `_scent_at`. The gradient then flows along
   corridors / around corners, never through walls — the geodesic field, of which the current
   straight-line + ray-occlusion is the broken approximation. **Cognitive caveat:** give the diffusion a
   SHORT range (strong absorption) so the bug can't smell food across the maze — it must EXPLORE to pick
   up the trail and REMEMBER (where the belief/planner earns its keep); a globally-diffused field hands
   over the solution (env does the reasoning).
2. **Physical — wedging (separate).** Even with a perfect bearing (transparent scent), the bug PINS
   dead-on against the pillar (97% close-but-can't-eat, commands full forward, ~0 translation) — it does
   NOT slide past, because the controller corrects off-axis to face the food, so velocity is
   perpendicular into the pillar (no tangential slide). This needs **obstacle perception + routing**
   (sense the pillar, choose a heading AROUND it) — scent can't provide it. Belongs to Stage 1b
   (planner takes obstacle perception as an input) and Stage 5 (the maze). A reactive blocked-escape
   (wedged ⇒ commanded-fwd but no afferent progress ⇒ turn/explore) is the cheap band-aid.
NB the occlusion is a PHYSICS raycast (vision capture too — the SubViewport render path was abandoned),
so it is identical headless vs UI — no render-only divergence.

### Stage 5 — **THE FALSIFIER**: corridor → maze + perturbation
**Scaffold:** the corridor rig makes "gradient = correct heading" *trivially true*, so it cannot
distinguish a planner from the gradient reflex.
**Replacement:** graduate to the **maze**, where the gradient points *through walls* and the coxswain
**must plan a heading through topology** (the genuine model-based reasoning the project is chasing) —
and run the **perturbation assay** (relocate food / drop scent mid-episode → re-inference → recovery)
for bar-(d). Transfer gate: a model-based coxswain transfers; a gradient-reflex-in-disguise breaks on
the topology (the falsifier). The wide cell (multi-food, no reset, mixed bus) is the intermediate step.
**Advances:** bar-(d) + the headline cognitive claim.

---

## Sequencing & rationale
- **Stage 1 then 2 are the cognitive core** and should come first — they convert "follows the gradient,
  unmotivated" into "decides where to go, because it needs to." Do 1 (planner) before 2 (motivation) so
  the value signal has a decision to attach to; or interleave (the planner's value *is* the homeostatic
  prediction).
- **Stage 3 (perception-as-inference)** and **Stage 4 (learned locomotion)** are rigor upgrades that
  strengthen bars (a)/(b) but aren't blockers — defer behind the cognitive core.
- **Stage 5 is the proof** — the corridor validated the *mechanism*; only the maze + perturbation can
  show it's *reasoning* and not a dressed-up reflex. Keep the gradient-reflex follower as the **control
  arm** throughout (it should win in open space and lose in the maze — that contrast is the evidence).

## What stays (legitimately) scaffold
The body/physics, the MotorBus routing, the raw scent sensor, and the test rig are **substrate/harness**,
not scaffolds to remove. The analytic compass and braking gate are **bootstrap scaffolds** kept as
control arms until their learned replacements match them.
