# Cell — Maze track plan (the genuine-reasoning falsifier)

**Branch:** `cell-maze` (off `main` @ `544cc33`, the open-arena de-scaffold milestone).
**Premise:** the open-arena de-scaffold is complete — **action** (learned heading-following) and
**motivation** (homeostatic regulation) are genuine cognitive wins; **perception-analytic** and
**nav-selection** are honest bootstraps that, as Stage 3 empirically showed, *cannot* be
de-scaffolded in the open arena (no non-teacher learning signal). The **maze** is where both get
**content** via the action-consequence loop — and where the project's actual thesis (genuine
problem-solving, not reflexes) is tested.

---

## The reframe: the gate is the FALSIFIER, not "recover N eats"

The open-arena gate ("after learning, recover ~298 eats") does not apply here. The maze gate is a
**contrast**:

> The gradient-reflex control arm (analytic/learned bearing → follower) **fails** to route to food
> it cannot smell across topology; the belief-carrying **planner solves** it. That contrast — same
> body, same env, reflex breaks / planner succeeds — is the evidence of reasoning. A reflex
> dressed as a planner would break on the topology too.

---

## The two composing pathways (the architecture)

The Cell carries **two complementary navigation systems** that hierarchically compose (the same
"two clean systems, one interface" pattern as nav/action):

- **Reactive gradient-follower (chemotaxis)** — `ScentCompass`/learned-bearing → `HeadingController`.
  Descends the *local* scent gradient. Fast, reflexive, already mostly built.
- **Deliberative pathway-planner (cognitive map)** — `HeadingPlanner` + `GoalBelief`. When the
  gradient is uninformative (gap / junction / dead-end), plans a heading through topology from
  **belief + memory**.
- **Composition:** the planner sets a sub-goal heading; the follower chemotaxes toward it. How much
  each pathway carries is set by the **scent falloff radius**.

---

## The falloff-radius curriculum (the key dial)

`scent_falloff_radius` = the decay length of the diffusion field. It is the maze's **single
difficulty dial** *and* the cleanest falsifier control:

- **LONG radius** → the gradient reaches across corridors/corners → a *followable* chemotactic
  pathway → the reactive follower works. **The foundation.** Critically, this is the
  action-consequence richness the open arena lacked: following a gradient and *reaching food*
  generates the egomotion + scent-change experience to learn the ring→bearing map **from
  consequence** (non-circularly) — so the **fully-learned scent follower (Stage-3 perception)
  finally becomes trainable here.**
- **SHORT radius** → the gradient dies out locally → following alone *provably fails* to reach food
  it can't smell → the **planner must route**. This is the falsifier rung — produced by *turning
  the dial*, not by hand-building a "hard" maze.

**Curriculum:** start **long** (build + validate the learned follower foundation), **progressively
shorten** (gradient-gaps appear → the planner is recruited for routing), **short** = the falsifier.
Build the foundation first, ramp difficulty — same methodology as the de-scaffold.

**No-tuning end-state:** the radius should not stay a hand-set sweep. The principled form is to
**anneal it by the follower's own competence** — shrink when the follower is mastering the current
radius, so the planner is recruited *exactly* when the gradient stops sufficing. Swept arms
(long / med / short) first to prove the mechanism; the competence-driven auto-shrink is the
no-tuning follow-up.

---

## Components & sequence (each gated before the next)

### 1. Env + 2D-diffusion scent (foundation) — **START HERE**

- **Maze layout:** start **minimal** — the smallest layout where the straight-line gradient points
  *through a wall* and the bug must route around it (a single **L-bend** or **T-junction**, food out
  of line-of-sight). Config-driven (a grid mask / wall-segment list). Scale up only after the minimal
  falsifier works.
- **Diffusion model (your idea):** discretize the floor into a grid; mark cells FREE / WALL; food
  cells are **sources**, walls are **barriers**. Relax a **screened-Poisson** field over free cells
  only (`∇²S − κ²S + source = 0`) with a few Jacobi sweeps — steady-state decays exponentially with
  length `1/κ`, so **`scent_falloff_radius = 1/κ`** is the curriculum dial. Recompute when the food
  set changes (mostly static between eats), cache, **sample bilinearly** in `_scent_at` at each
  nostril. The gradient then flows **along corridors / around corners, never through walls** — the
  geodesic field of which the current straight-line + ray-occlusion is the broken approximation.
- **`scent_falloff_radius`** param (metadata + `OGMA_SCENT_FALLOFF`). Long default to start.
- **Sanity-viz (the gate):** confirm the field flows along corridors and is zero across walls —
  dump the grid to a heatmap (PNG / array) or a debug overlay. **Gate:** at any free cell the
  gradient points *along the corridor toward food*, and is ~0 across a wall.

### 2. Wall-collision handling in the action layer (prerequisite — see dedicated section below)

The maze is *made of walls*; the follower will contact them constantly. Must be solid before any
maze foraging run is meaningful.

### 3. Learned scent follower @ long radius (foundation + Stage-3 perception)

- At long radius the gradient is followable everywhere → the follower (`ScentCompass`/learned-bearing
  → `HeadingController`, **stable hand-gate** advance) forages the maze by chemotaxis.
- **Revisit the fully-learned scent follower here** (the deferred Stage-3 perception): the
  gradient-following + reach-food consequence + egomotion now provide the non-circular signal the
  open arena lacked — learn the ring→bearing map from consequence/egomotion (BearingEstimator with a
  real signal, or a learned encoder).
- **Gate:** the follower solves the *followable* maze via the gradient; the learned bearing
  matches/beats the analytic. Foundation validated.

### 4. Shorten radius → planner routes

- As the radius shrinks, the gradient develops **gaps** (junctions, far corridors); the follower
  stalls there. The **planner** (`HeadingPlanner` + `GoalBelief`) learns to **create a heading that
  routes through topology** from belief + memory + obstacle perception. Credit design (locked,
  2026-06-21): **per-tick EMA on the IMU achieved-COURSE** (dir of velocity, not facing),
  `V[course_sector] += lr·(v·b̂_food − V)` — reads the achieved course, never the command, so nav
  and the follower co-bootstrap without poisoning each other.
- **Composition:** planner sets the sub-goal heading; follower chemotaxes locally.
- **Gate:** at a radius where the follower *alone* stalls, follower + planner reaches food.

### 5. Short radius = falsifier + perturbation (bar-d)

- Short radius: gradient local-only. The gradient-reflex control arm **provably cannot** reach a food
  it can't smell across the maze → **fails**. The belief-planner **solves** it (explore → pick up the
  trail → remember → route). The contrast = the headline reasoning evidence.
- **Perturbation assay (bar-(d)):** relocate food / drop scent mid-episode → re-inference → recovery.
  A genuine planner re-routes; a reflex-in-disguise breaks.

---

## Wall-collision handling in the action layer (the operator's section)

**Problem (operator observation):** the bug **freezes when it backs into a wall**. The forward-fan
whiskers don't sense rear/side contact, the `HeadingController` has no escape, and once wedged the
afferent velocity → 0 with no recovery → it sits. In a maze this is fatal (constant wall contact).

Two independent fixes, both wired into the **new** `HeadingController → MotorBus` pipeline (the old
whisker reflex wiring was for the `ActionDecoder`/`MotorEPM` bus and is **untested** here):

**(a) Whisker reflex in the new pipeline (forward/side contact).**
`WhiskerSteerReflex` was rewritten to the weighted-sum + reverse-out-of-wedge spec but never tested
against `HeadingController` + the current `MotorBus`. Wire it as a **second MotorBus influencer**
(`reflex.steer` / `reflex.thrust`) with **subsumption**: it dominates the mix on contact and is
silent (0) otherwise, so the cog (`HeadingController`) passes through untouched off-contact and is
overridden only when a wall is felt. Verify the cog is not *permanently* ducked.

**(b) Wedge / blocked-escape (omnidirectional — the backward freeze the whiskers miss).**
The forward whiskers cannot sense a wall *behind* the bug. Use the **homeokinetic broken-contingency
signal** we already publish: if the commanded motion is significant (`|cog.thrust|` or `|steer|`)
but the **afferent velocity** `|reality.proprio.vel_ego| ≈ 0` for `K` ticks, the bug is **wedged**
(any direction) → trigger an **escape** (reverse the command / turn / random kick) until it
displaces again. Direction-agnostic → catches backing into a wall. Wire as a reflex influencer with
subsumption. (NB `StuckEscapeReflex` exists from the old pipeline — reuse/adapt or build a lean new
one against `vel_ego`.)

**Gate:** in a walled arena, the bug **never freezes** against a wall — forward, side, *or
backward* contact is detected and escaped within `K` ticks, and foraging resumes. **Test
specifically with the new `HeadingController` + `MotorBus`** (the untested seam).

**Arbitration note:** reflexes are MotorBus influencers that dominate when active and are silent
otherwise (subsumption); confirm the mix doesn't leave the cog attenuated off-contact (the
`idle_reflex_passthrough` lesson from the MotorFader work).

---

## Design decisions (defaults)

| Decision | Default | Rationale |
|---|---|---|
| Maze complexity | **minimal** (L-bend / T-junction) | the smallest falsifier; scale up after it works (avoid all-at-once confounds). |
| Scent model | screened-Poisson grid diffusion | flows along corridors, never through walls; `falloff_radius` = curriculum dial. |
| Obstacle sense | whiskers (forward) **+** wedge-escape (omnidirectional, `vel_ego`) | whiskers exist/cheap; wedge-escape catches the backward freeze. |
| nav-selection | `HeadingPlanner` + `GoalBelief`, IMU-course credit | built; achieved-course credit decouples bootstrap. |
| Advance | **stable hand-gate** (`learn_advance=false`) | keep the learned-advance orbit out of the maze confound; revisit later. |
| Curriculum | radius long → short, swept then competence-annealed | foundation first, ramp difficulty; no-tuning end-state. |

---

## Gates summary

1. **Env + scent:** field flows along corridors, zero through walls (sanity-viz).
2. **Wall-collision:** bug never freezes on wall contact (any direction), new pipeline.
3. **Follower @ long radius:** chemotaxis solves the followable maze; learned bearing ≥ analytic.
4. **Planner @ shortening radius:** follower+planner reaches food where follower-alone stalls.
5. **Falsifier @ short radius:** reflex control arm FAILS, belief-planner SOLVES + recovers under
   perturbation → bar-(d) + the genuine-reasoning claim.
