# Cell — Decomposed Coxswain: a learned heading-following forager (MILESTONE)

**Date:** 2026-06-21 · **Branch:** `cell-forager-hardening` · **Config:** `the_cell_corridor_turn_heading.json`
**Repro (headless):**
```
OGMA_REFLEX_MODULAR=1 OGMA_EAT_FORWARD_MIN=0.5 \
OGMA_CELL_CONFIG=res://addons/ami_ogma/configs/the_cell_corridor_turn_heading.json \
godot4 --path godot_host/project --headless --fixed-fps 120 --disable-render-loop res://scenes/the_cell.tscn
```
Analyzers: `scripts/head_analyze.py`, `scripts/turn_cog_analyze.py`.

---

## The result

The cognitive bug forages by **deliberating**, not reacting: when food appears off-axis it **brakes,
turns in place to acquire the heading, then drives forward and eats** — visibly different from a
"homing-missile" reflex that just curves toward the target. Across **5 seeds × 10 min** (turn rig,
one food at a random ±45° bearing, respawned a genuine 4 m away on each eat, no reset):

| metric | value | meaning |
|---|---|---|
| eats / 10 min | **298.6** (293–303, cv ≈ 2%) | rock-solid across seeds |
| backs-away baseline (cy + respawn-far) | **2.7** | the monolithic actor it replaces |
| inter-hit displacement | 2.5–2.7 m; **path/hit ≈ 4.1 m ≈ respawn distance** | genuine turn-and-approach (not spurious close-eats) |
| facing-forward velocity | **+3.3** (all seeds) | approaches food (no backing-away) |
| learned turn gain `k_body` | converges (interior 3.7 at 5 min, ~max at 10 min) | a real, body-specific sensorimotor model |

This is a **locomotor** milestone: a clean, learned heading-following primitive. It is **not yet a
navigation/cognition** milestone — see the scaffold audit below.

---

## How we got here (the arc)

1. **Nav blocker root-caused (commit a7d6978):** the bearing EPM was **direction-blind** — it clustered
   the *raw, tiny* scent gradient by distance, not angle (≈48% direction-blind states; GNG growth
   stalled). Confidence-gated **unit-direction normalization** in ScentCompass fixed it (dir-blind
   56%→2%, baking reignited, steering emerged).
2. **But the monolithic planner backed away when facing food** — its value rewarded *facing* (`cy`),
   not *reaching*; the thrust channel learned to reverse/hold to keep food framed. Confirmed across
   seeds (`fwd_v ≈ −0.5` when facing). Removing the episodic reset did **not** fix it → a true value
   Goodhart, not reset-aversion.
3. **Decomposition (operator's design):** split the monolithic coxswain into (a) a **heading setpoint**
   and (b) a **learned heading-following controller** (`HeadingController`). The controller couples
   thrust to *progress* (advance when aligned), which structurally kills the backing-away.
4. **Three UI-driven fixes (operator observations):**
   - **Overlap-eat** — eat on any front-mouth *overlap* each tick, not just the `body_entered` edge
     (food that *spawns inside* the mouth froze the bug).
   - **Turn-in-place braking gate** — full forward only within ~30° of facing; **brake/reverse** when
     off-axis → the bug kills its coasting momentum and rotates in place to face, then charges. This is
     the "back up, acquire heading, drive in" behavior (the body's `forward=thrust`/`turn=steer` are
     decoupled, so in-place rotation is physically available).
   - **Reflection respawn + eat-debounce** — food is placed a *genuine* 4 m away (reflect the bearing
     off the interior bounds instead of clamping the distance), and a 6-tick debounce kills a
     stale-overlap rapid-fire (which had inflated the count ~5×, eats at 0 m displacement).

---

## ⚖️ HONEST SCAFFOLD AUDIT — what is cognition vs. what is given

> The bug looks intelligent. Most of that intelligence is **scaffold**. One element is genuinely learned.

| element | classification | notes |
|---|---|---|
| 8-nostril scent ring | **sensor / substrate** | a real *diffusing* field (not ground-truth coords) — the honest foundation for bar-(a). |
| ScentCompass: ring → up-gradient bearing `[cx,cy]` (analytic vector sum + normalize/gate) | **SCAFFOLD** (perception) | the "which way is food" reduction is *hand-coded*, not a learned/inferred generative model. |
| **Heading SELECTION = the gradient direction itself** | **SCAFFOLD — the big one** | "where to go" = "up the gradient" = a hardwired *chemotaxis reflex*. No planning, no learning of *where*. Trivial in open space; the cognitive version (plan a heading through topology) is what the **maze** demands. |
| `HeadingController` turn-gain `k_body = EMA(|ω|/|steer|)` + inverse control | **LEARNED — the one genuine cognitive element** | a forward model of the body's *own* turn response; the bug acts to null its heading-prediction error via that model. **bar-(b) at the motor/locomotor level.** Converges to a body-specific value (transfers across bodies/friction). |
| braking alignment gate (turn-in-place ↔ charge), `turn_fraction`, thrust-when-aligned | **SCAFFOLD** (control policy) | the admired "back up → acquire → charge" *structure* is a hand-designed control law, not learned. |
| MotorBus (cog → flagella), `bidirectional_paddler` body | **substrate** | embodiment + fixed signal routing; the body *can* turn in place (decoupled forward/turn). |
| HomeostaticDrive + NeurochemState (energy/hunger/dopamine) | **PRESENT but DECORATIVE** | the controller forages **regardless of hunger** — there is currently **no value learning and no motivation**. Foraging is reward-free gradient-following. |
| corridor turn rig, respawn-far, mouth eat | **test scaffold** | single food, bounded interior, episodic-ish; a measurement harness, not the agent. |

### One-paragraph honest summary
What we have is a **beautiful locomotor primitive with one real piece of sensorimotor learning** (the
body's turn gain), wrapped in hardwired scaffolds: the **where-to-go is a gradient reflex**, the
**back-up-and-charge is a hand-designed control policy**, and there is **no motivation/value learning**
(the homeostatic drive is decorative). The deliberate, un-reflexive *look* comes from the braking
control policy, not from cognition. This is genuine progress (a transferable, model-based heading
follower — bar-(b) at the motor level), and it is the right substrate to build cognition *on top of* —
but the cognitive layer (decide where to go, be driven by need, re-infer under change) is scaffold or
absent.

### Observed failure mode (operator, pillars on) — the missing-belief tell
With pillars enabled, when one **directly occludes the food the bug oscillates** between
occluded/not-occluded and stops homing; remove the pillars and it homes immediately. Root cause: the
scent sensor is **line-of-sight occludable** — `_scent_at` (the_cell_world.gd:923) raycasts food→sample
and attenuates the contribution to `obstacle_scent_attenuation=0.30` when a pillar blocks it, so the
scent behaves *like vision*. The hardwired gradient-follower has **no belief/memory** to bridge the
momentary signal loss → it flails. **Key implication:** vision would NOT fix this (it is line-of-sight
too — occluded by the same pillar); the bridge is a **maintained belief about where the food is**, i.e.
cognition. This is the cleanest demonstration that the missing piece is a *belief-carrying planner*, not
another sensor — and it argues for **de-scaffolding before adding modalities**. The occlusion-pillar
arena is the natural gate for Stage 1.

### bar-(a–d) status (the defensible-AI checklist)
- **(a) inferred, not oracle** — **PARTIAL+.** Scent is a real diffusing sensor (not coords); the bug
  acts on a bearing *derived* from it. But the derivation is analytic (hand-coded), not a learned
  generative model with its own error.
- **(b) action reduces the agent's own inferred error** — **YES at the locomotor level** (learns a turn
  forward model, acts to null heading error), **NOT at the cognitive level** (the heading is *handed*
  by the gradient; homeostatic error isn't what's being minimized).
- **(c) loop isolation** — **ready, not yet run as a formal control** (ablate/​shuffle/​wrong-sign the
  heading → foraging should collapse).
- **(d) perturbation → re-inference** — **not demonstrated.** A gradient follower would *reactively*
  re-track moved food; that is reflex re-tracking, not cognitive re-inference.

---

## Current scaffold state (2026-06-21, post-belief + transparent-scent + pillar diagnosis)

Pipeline = perception → belief → heading-selection → heading-following → body. Honest classification:

| component | class | note |
|---|---|---|
| 8-nostril scent ring | substrate (sensor) | real diffusing field; transparent (atten 1.0) ≈ correct for sparse pillars; path-grid for maze. |
| ScentCompass: ring→bearing (analytic sum+normalize) | SCAFFOLD (perception) | hand-computed, not a learned/inferred model. |
| GoalBelief (path-integration goal memory) | hand-designed cognitive FACULTY | real working-memory-of-goal; KEEP (planner builds on it); idle here (no signal loss to bridge). |
| heading SELECTION = follow gradient/belief | **SCAFFOLD — the big one** | "where to go" = up-gradient reflex; no planning/routing. The maze demands this be replaced. |
| HeadingController turn-gain k_body + inverse | **LEARNED — the only one** | body turn-response forward model; bar-(b) at the MOTOR level. |
| braking alignment gate + turn_fraction | SCAFFOLD (control policy) | hand-designed; also CAUSES the dead-ahead wedge (corrects off-axis → no slide). |
| obstacle perception + routing | **ABSENT (gap)** | nothing senses/routes around a pillar → dead-ahead wedge. The maze is made of this. |
| HomeostaticDrive + NeurochemState | **DECORATIVE** | present, unused; no value learning, no motivation (forages reward-free). |
| MotorBus + bidirectional_paddler | substrate | embodiment + routing; can turn-in-place + slide. |
| corridor rig / respawn-far / overlap-eat / transparent scent | test harness | not the agent. |

**One-line:** exactly ONE element is learned cognition (the turn gain); the belief is a kept hand-designed
faculty; *where* it goes (gradient reflex) and *how* it commits (braking policy) are scaffold; there is NO
motivation (reward decorative) and NO obstacle cognition (absent). A superb locomotor primitive; the
navigational/decisional intelligence is scaffold or missing.

**Pillar diagnosis (verified):** two independent failures — (1) SENSORY: occludable scent (line-of-sight
×0.30) corrupts the bearing (corr 1.0→0.55); fixed by the diffusion model (transparent now / grid maze).
(2) PHYSICAL: even with a perfect bearing the bug WEDGES dead-on against a pillar (the braking policy
corrects off-axis → no tangential slide); needs obstacle perception + routing. Both are PHYSICS-raycast
(vision capture too — SubViewport render path abandoned) → identical headless vs UI, no render divergence.

**De-scaffold-before-maze priority:** (1) heading SELECTION → learned planner; (2) obstacle perception +
routing → planner input; (3) motivation → homeostatic value. Keep the belief; defer analytic-perception
and braking-policy learning.

## Next: de-scaffolding plan

See `docs/plans-and-designs/cell_descaffolding_plan.md` — staged replacement of each scaffold with a
cognitive element (heading **selection** → homeostatic **motivation** → perception as **inference** →
learned **locomotor policy** → the **maze + perturbation** falsifier), each gated on an ablation
control and the bar-(a–d) rung it advances.
