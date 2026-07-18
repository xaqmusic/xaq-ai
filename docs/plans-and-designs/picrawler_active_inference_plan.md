# Picrawler — Active-Inference Port Plan

*Bringing the Cell navigator's defensible active-inference discipline back to the
picrawler quadruped. Built to `docs/brain_building_doctrine.md` — especially §1
(always predicting), §2 (the a–d bar + the EFE arbiter chooses a pathway into the
future), §3 (decomposition), §5 (layered strange loops, never disable a working
loop), §6 (no tuning), §7 (place/phase-indexed coding), §8 (predictable env,
rising-competence signature, reset-masked reporting).*

*Provenance: the 2026-07-04 audit of the Motor-EPM line (branch `dev`, tip
`4b2160d`) against the doctrine. Companion to `motor_epm_vision_and_roadmap.md`
(which this supersedes on the nav layer) and the Cell plans
`cell_nav_strange_loops_plan.md` / `cell_efe_arbiter_plan.md` (the machinery this
ports). Living/agile — re-wire as the loops teach us.*

---

## 0. Where we stand (the audit verdict)

- **The motor loop is already defensible active inference — at the motor level.**
  The Motor-EPM HK-TLE core (`MotorEPM.cpp:941-990`) is a genuine generative model:
  a forward self-model `x̂(t+1)=A·y+b` whose controller descends its OWN motor TLE
  `ξ = x − x̂` (Der–Martius). It clears **(a)** inferred-not-oracle (proprioception)
  and **(b)** action-reduces-own-error. `motor_tle` + `loop_gain` are surfaced. Keep
  this; per §5 it is now **foundational substrate**, never disabled.
- **The nav loop is the disqualifier.** `target_compass` is computed from the
  ground-truth target position (`picrawler_body.gd:3823-3859`); steering is pure
  P-control on that oracle bearing (`MotorEPM.cpp:1101-1129`). This is verbatim the
  §2 anti-pattern: *"cybernetic feedback (oracle-bearing → proportional steer) fails
  (a) and (b)."* `vision_steer` does not fix it — `vision_compass` is a **frozen
  offline linear readout distilled from the oracle** (the §4 "a copy dies with its
  source" anti-pattern), not a percept with its own error.
- **The oldest, deepest problem is gait.** Real inter-leg rhythm never emerged.
  Every attempt collapsed into per-tick noise or was averaged out (see §2 below).
  Coordination was only ever *imposed* (Kuramoto `gait_phase=[0,π,π,0]`), so nothing
  there could learn or improve.

**Goal of this plan:** a compelling, undeniable active-inference picrawler — a
quadruped whose **gait improves over time from an intrinsic prediction signal** and
whose **navigation is an inferred belief the body acts to resolve**, with the full
a–d control suite, proven by a mid-episode **perturbation** it re-infers through.

---

## 1. The layered architecture (every lower layer stays ON — §5)

```
                                                          ARBITER            ACTION (foundational)      GAIT (foundational)
 INPUTS        per-input EPM     per-LOOP voter → heading generator

 SCALAR ─►EPM─┐   ┌ PLAY       : sensorimotor ─►voter─► homeokinesis ──┐
 VISION ─►EPM─┼─► ┤ SCENT/BEACON: scalar ──────►voter─► klino heading  ┤─►EFE ARBITER─► HEADING+SPEED ─► KEYFRAME GAIT ─► HK ─► L
 IMU    ─►EPM─┤   └ PLACE-PLAN : window-avg ───►voter─► place route     ┘   IMPORTANT?     command          (phase→posture)   CTRL   R
 CPG    ─►EPM─┘                                                            HEADING? GO!       │                  ▲                ▲
   ▲                                                                                          │            CPG phase clock  balance/panic
   └──────────── DESCENDING PREDICTOR : consensus → predict each EPM's next latent → EPM subtracts (pred-error) ─┘  reflexes subsume
```

- **L-1  GAIT (this plan's centerpiece):** a self-improving, coordinated limit cycle
  driven by an intrinsic prediction signal, feeding the HK controller.  **NEW wiring.**
- **L0  ACTION:** the HK controller + its `steer`/`speed` sockets + MotorBus + reflexes.
  **EXISTS.** The nav layer plugs into `steer`/`speed`; it never bypasses the gait.
- **L1  NAV LOOPS:** decomposed heading-generators, each its own honest signal + gate.
- **L2  EFE ARBITER:** selects a loop by expected free energy (port `EFEArbiter`).
- **L3  DESCENDING PREDICTOR:** closes the top-down loop (port `DescendingPredictor`).

The decomposition is the point (§3): **gait = "predict/stabilize my own body,"**
**nav = "where to go."** They share no signal, so nav can never overload gait — the
mistake that standing-reward + navigation always was.

---

## 2. L-1 — The self-improving gait (the centerpiece)

### 2.1 The two failure modes, precisely

- **"Collapsed into per-tick noise."** A tick-horizon predictor `x(t)→x(t+1)` has its
  least-surprising prediction at "next ≈ current" — **standing is the free-energy
  minimum of any per-tick objective.** Bare HK lived here; `GaitSelector`'s own header
  records the same: *"static postures selected per tick could never converge to a
  coherent gait."*
- **"Averaged out."** Time-averaging an oscillation returns its mean — a single
  mid-stride posture = DC = standing. `KeyframeAverager`/`KeyframePeakDetector` are
  exactly this rolling-window mean, so any naïve slow layer washes the rhythm out.

Both failure modes share the **standing-DC attractor**: speeding up lands there,
slowing down lands there. This is why one-mechanism fixes kept failing.

### 2.2 What handles the coordination

**The coordination IS a whole-body phase-indexed keyframe, not a coupling law.** Each
keyframe is a full 12-joint posture, so the inter-leg geometry ("FL+RR forward while
FR+RL are back") is simply *what keyframe K is*, with the diagonal swapped in K+1.
There are no separate per-leg oscillators to relate — one body-state trajectory whose
keyframes already contain the relationships. This is why per-leg HK never coordinated
(four independent oscillators needing a coupling law) and why Kuramoto had to *impose*
one.

### 2.3 The division of labor across timescales (the resolution)

No single mechanism carries a gait. Three do, at three timescales, and every prior
attempt failed by asking one to do all three:

| Role | Mechanism | What it guarantees |
|---|---|---|
| **Timing** | `CPGOscillator` (the clock) | temporal stability *by construction* — an autonomous oscillator; the legs don't invent the rhythm, they fill in phase→posture |
| **Coordination / content** | phase-indexed keyframe map (phase → 12-joint posture) | spatial stability; sharpens over cycles |
| **Closure** | `DescendingPredictor` (keyframe → top-down target) | turns the map into an *objective the controller descends toward* |
| **Anti-freeze** | HK loop-gain + boredom neuromod | the *only* thing that excludes the standing DC |

**The key correction to the keyframe intuition: index by CPG phase, do not average
over time.** Keep N phase-bins; `keyframe[φ]` is an EMA over ticks where CPG phase ≈ φ,
accumulated **across cycles at matching phase.** Then `keyframe[0] ≠ keyframe[π]` — the
rhythm is carried by the *index*, and cross-cycle averaging at fixed phase **sharpens**
each keyframe instead of washing it out. This is the Cell's `CylinderBuilder` trick
(heading-indexed panorama) ported to the motor stream (phase-indexed posture), and the
`chunk temporal crystallization` lesson (integrate over the chunk's extent, not a
single tick). **You never store the oscillation in the EPM's values — you store the
phase→posture map and let the clock carry the time.**

This is also the missing piece behind the CPG-period-sweep invariance (the legs ignore
the CPG): there was a metronome and muscles but **no sheet music** mapping phase→posture
and driving the body there. The keyframe map is that sheet music; the descending
predictor is the musician; the HK loop is the muscle.

### 2.4 The load-bearing distinction — objective, not injection

The descending prediction is an **objective the loop minimizes surprise against**, not
a signal injected onto its output. That is the exact line between what worked and what
failed in the gait history: the homeokinetic objective *changed what the brain wants*
(worked); E0/E2 rhythm **injection** was *resisted — the brain defends standing*
(failed). It is also on the safe side of the durable lesson **"additive joint-bias
disrupts the HK limit cycle; only objective-change and constraint-removal help"** — an
error-to-descend is an objective change, not an additive bias. And it is **not
open-loop E0**: the map is learned from the body's own recurring motion, offered as a
*soft* target the HK loop overrides for balance, and it improves and re-forms — the
opposite of a hand-authored script in every load-bearing way.

### 2.5 The central risk — the standing-DC bootstrap

**No representation excludes standing** — it is the most consistent, most predictable,
most bake-able state, so a keyframe map left alone will bake "every phase = standing"
(a constant map, zero TLE). Two consequences, stated plainly:

- **Bake on cross-cycle *consistency only* — never a fitness/reward weight** (no
  Goodhart). A phase-posture that recurs across cycles crystallizes; a transient
  doesn't. Standing is consistent too, so this gate alone does not exclude it.
- **The escape from standing lives entirely in the anti-freeze**, not the EPM: the HK
  loop (a frozen body has degenerate loop gain — the sensitivity metric blows up) plus
  **boredom** neuromodulation (a fully-predicted posture has no reducible surprise →
  raise explore/destabilize gain → leave the fixed point). The EPM gives the rhythm a
  *stable shape*; the HK/boredom drive gives it a *reason to exist*.
- **Bootstrap chicken-and-egg + mitigation.** Phase-indexed averaging only sharpens
  once the body is phase-locked to the CPG, but the body only phase-locks once the map
  drives it there. Break it with **mutual entrainment**: let the CPG frequency adapt to
  the body's dominant emergent rhythm (a phase-locked loop; the `RunTumbleNav`
  SNR-adaptive-period trick), so the clock tracks the body while the keyframe map +
  descending predictor tighten the body onto the clock. If the map still collapses to
  standing, that is a **clean, falsifiable diagnostic that the anti-freeze is too
  weak** — not a mystery.

### 2.6 What "improves over time" IS, measured (§8)

The improving quantity is honest and intrinsic: the **keyframe/sequence model's TLE
falling** and **`gait_coherence()` rising** (currently a dead accessor — surface it),
both **reset-masked** (the reset cycle faked every prior rhythm claim and prevents a
real one from accumulating — see Gate 0). That is the locomotor analog of the §8
eat-rate-rising signature. The *ultimate* improvement proof is generalization under
perturbation (§2.8).

### 2.7 Reuse vs. new (the novelty is narrow)

Most of the machinery exists — which de-risks this:

- **EXISTS / reuse:** `CPGOscillator` (clock), `MotorEPM` (HK loop + per-leg phase est.
  + boredom/interest states already present), `SequenceGNG`→`GNGRollout`→
  `DescendingPredictor` (a temporal-sequence-prediction chain with forward rollout +
  top-down closure — check the `docs/primitives/*.md` contracts before committing),
  `CylinderBuilder` (the phase/heading-indexed signature pattern to mirror),
  `GaitSelector` (the falsified Phase-8 ancestor — reuse its option-playback plumbing,
  drop its per-tick reward selection).
- **NEW:** the **phase-indexed keyframe accumulator** (vs. the time-mean
  `KeyframeAverager`), its wiring onto the MotorEPM proprioceptive stream indexed by
  CPG phase, the CPG↔body **PLL** (adaptive frequency), and the descending-predictor
  closure into the HK controller's objective.

### 2.8 Ablations — prove coordination is *baked*, not *imposed* (§2c)

Each ships with the module:
- **shuffle the phase index** (feed a random φ) → keyframes decohere → gait fails.
  Proves the *phase-indexing* does the work.
- **freeze the map** (no cross-cycle update) → no improvement over the run. Proves
  improvement is *learning*, not mechanics.
- **ablate the descending predictor** → per-leg rhythm returns but no whole-body
  coordination. Proves the closure carries coordination.
- **Kuramoto-contrast arm** (imposed `gait_phase`) → coordinates but **cannot improve
  or re-coordinate** to a novel perturbation. The contrast that makes "learned" visible.
- **wrong-sign / lagged descending target** → coordination fails → the prediction (not
  a coincidence) is causal.

### 2.9 Gates (fast-fail; promote-or-kill)

- **Gate 0 (prerequisite):** the upright homeostatic prior holds; reset/fall rate falls
  and stays low, reset-masked. Without this no limit cycle can accumulate. This is
  "standing aliveness" redeemed as a **prior, not an objective.**
- **Gate 1:** keyframe/sequence TLE falls over the run (reset-masked).
- **Gate 2 (real-gait-formation):** `gait_coherence` rises **with Kuramoto ablated** →
  coordination is *learned, not imposed.*
- **Gate 3 (aliveness / locomotor-(d)):** shove the body or drop one leg's authority →
  the gait destabilizes then **re-forms** (and, with a leg degraded, re-coordinates).

---

## 3. L0 — Action loop (EXISTS — the socket)

The HK controller + `steer` (skid-steer differential) + a `speed` command + MotorBus
subsumption + whisker/stuck/balance/panic reflexes. The bilateral-mirror steering
blocker is already solved by the per-leg symmetry-broken Motor-EPM. **The nav layer
feeds the FULL controller through `steer`/`speed`** — never a stripped one (§5), and
as a *command/objective*, never an injected joint bias (§2.4).

---

## 4. L1 — Nav loops (decomposed, each its own honest signal — §3)

Replaces the oracle `target_compass` steering. Each loop stands alone with its own
gate before the arbiter is built.

- **PLAY — pure homeokinesis (epistemic floor).** `PlayLoop` (+ `MotorEPM`; note
  `HomeokineticExploration` is deprecated 2026-07-03). Always alive; the arbiter exposes
  it when the pragmatic loops lack confidence. Reward-free. Gate: env coverage.
- **BEACON-GRADIENT — honest scalar klino (pragmatic, local).** A **scalar** beacon
  signal on the body (one number, or a bilateral pair — *not* a ring/compass; morphology
  honesty §4). Direction is hidden → it must be **acted out**: the body's own gait weave
  is the demodulation reference (klinotaxis §5), `Klinotaxis`/`RunTumbleNav` port —
  finish the open fix (demodulate against *actual* body deviation, not the commanded
  weave). Predicts Δbeacon of the chosen heading. Gate: **beats a true random walk** in
  the large arena (the discriminating regime the Cell's cramped rig never gave klino).
- **PLACE-PLANNER — map route (pragmatic, global).** `PlaceGraphPlanner` on a
  window-averaged consensus map. Optional/later. Gate: routes to remembered target.

**Sensor honesty:** scalar-beacon first — it is the clean morphology-honest route to
the (b)/(d) contrast. Vision is a *later parallel loop* that predicts its own
consequence (never a regression onto the oracle — that is what `vision_steer` wrongly
is). Depth-blob bearing is nearly instantaneous (≈ an oracle) → wrong first sensor.

---

## 5. L2 — EFE arbiter (chooses a pathway into the future — §2)

Port `EFEArbiter` + `cell_efe_arbiter_plan.md` wholesale. It selects a loop by expected
free energy from the agent's OWN quantities — pragmatic (divergence from the
homeostatic prior) + epistemic (belief-entropy reduction) — with **asymmetric,
scale-free normalisation** (excitement *spike* vs sustained *level*) and **adaptive
hysteresis** so the crossover is a property of the dynamics, not a tuned constant
(§2.2/§2.3/§6). It gates MotorBus channel gains (winner ≈1 / loser ≈0) and **pauses the
loser's learning via the authority mechanism** (verify the consumer fires — §8). PLAY is
the floor exposed under low confidence + rising hunger. This is the "chooses a future"
layer the picrawler never had.

---

## 6. L3 — Descending predictor (closes the strange loop — §5)

`DescendingPredictor`: `predicted_latent = W·consensus + b` per target; each EPM
subtracts the prediction before its GNG step → its residual IS the prediction error;
SGD on the error. Top-down prediction meets bottom-up sensation; **action makes the
prediction come true.** Note this is the *same primitive* that closes the gait keyframe
loop (§2.3) — one mechanism, two placements (nested blankets, §2.1).

---

## 7. Measurement env + the keystone demonstration (§8)

- **Predictable env:** a small set of **FIXED** target/beacon locations (never random
  respawn) so the quadruped can come to KNOW its world. Success signature = the
  behavioral rate **rising over time**, reset-masked — not a high instantaneous rate,
  and explicitly not "beats the reflex."
- **The keystone (undeniable) demonstration = the (d) perturbation.** Mid-episode,
  **relocate the beacon.** With an inferred (non-oracle) bearing the belief breaks →
  the body **acts (weaves/scans) to re-infer** → re-homes. Shipped alongside:
  - **(c) control arms:** shuffle the scalar (→ random walk), wrong-sign (→ flees),
    sever the loop (→ no homing), and the **oracle arm as the upper baseline** — it homes
    trivially but *cannot* demonstrate re-inference, which is exactly the contrast that
    makes the point.
  - The gait's own **locomotor-(d)** (§2.8): shove → the gait re-forms.
  This is the picrawler analog of the Cell's D-value maze-rescue: both arms look alike
  until you perturb; only the inferred loop survives.

---

## 8. Build order (staged — every lower layer stays ON)

- **L-1a  Gait prerequisite (Gate 0).** Upright homeostatic prior tames the reset/fall
  cycle so a limit cycle can accumulate. Surface `gait_coherence` + keyframe TLE.
- **L-1b  Keyframe gait (Gates 1–3).** Phase-indexed keyframe accumulator + CPG↔body
  PLL + descending-predictor closure. Prove coordination is baked (ablation suite §2.8),
  improving (TLE↓/coherence↑), and re-forms (locomotor-(d)).
- **L0  Action loop.** Already ON; confirm `steer`/`speed` sockets drive the keyframe
  gait (command, not bias).
- **L1  Nav loops ALONE, gated.** Add the scalar beacon; port klino → beats random.
  Place-planner optional.
- **L2  EFE arbiter.** Port; verify the consumer fires (loser muted + learning paused).
- **L3  Keystone.** Beacon relocation + (c) arms + oracle upper-baseline, powered n≥10.

Never skip Gate 0 or the powered stage; lesions are *tests*, never the operating mode.

---

## 9. Per-file changes (indicative — confirm primitive contracts first)

- **NEW** `cpp_core/{include,src}/ogma/modules/KeyframeGait.{hpp,cpp}` (or extend
  `SequenceGNG`+`GNGRollout`): phase-indexed keyframe accumulator (index = CPG phase),
  cross-cycle-consistency bake, publishes the phase→posture target for the descending
  predictor. Params HotMutable; ablation hooks (`shuffle_phase`, `freeze_map`,
  `no_descend`, `kuramoto_contrast`, `wrong_sign`). No tuned thresholds (§6).
- **EDIT** `CPGOscillator`: optional adaptive frequency (PLL) tracking the body's
  dominant rhythm; default-off.
- **EDIT** `MotorEPM`: accept the descending keyframe target as a controller objective
  (surprise-to-descend, not additive bias); expose boredom→explore-gain neuromod path;
  surface `gait_coherence`/keyframe-TLE in `diag_snapshot`.
- **EDIT** `MotorBus`: subscribe `arbiter.gain.<loop>` (effective gain = base × arbiter;
  mutes + pauses learning) — same as `cell_efe_arbiter_plan.md`.
- **EDIT** `picrawler_body.gd`: publish a **scalar** beacon signal (`reality.proprio.
  beacon`) from fixed sources; keep `target_compass` only as an oracle **measurement
  baseline**. Beacon-relocation hook for the (d) test.
- **PORT** `EFEArbiter`, `Klinotaxis`/`RunTumbleNav`, `DescendingPredictor`, `PlayLoop`
  into a picrawler config graph feeding `steer`/`speed`.
- **NEW configs:** `the_picrawler_keyframe_gait.json` (+ `_abl_*` per §2.8),
  `the_picrawler_ai_nav.json` (+ oracle-baseline, shuffle, wrong-sign, severed arms).
  Default-off so existing envs stay byte-identical (§8). Add to launcher allowlist.
- **NEW tests:** keyframe bake/shuffle/freeze determinism; descending-target-is-objective
  (not additive); arbiter 0/1 partition + hysteresis + force-policy ablations;
  MotorBus gain-mute pauses learning.
- **NEW inspector widgets:** the keyframe **phase wheel** (posture per phase-bin,
  coherence, TLE-over-time) and the arbiter **value race** — the demo artifacts.

---

## 10. Defensibility summary (a–d)

| Bar | Gait (L-1) | Nav (L1–L3) |
|---|---|---|
| **(a) inferred not oracle** | proprioception (already) | scalar beacon; direction inferred through action (klino) |
| **(b) action reduces own error** | controller descends keyframe/sequence TLE | steer to minimize inferred-bearing error (EFE), not oracle range |
| **(c) loop-isolation controls** | shuffle-phase / freeze / no-descend / Kuramoto-contrast / wrong-sign | shuffle / wrong-sign / severed / oracle-upper-baseline / force-policy |
| **(d) perturbation → re-infer** | shove → gait re-forms; degrade a leg → re-coordinates | relocate beacon → re-infer through action → re-home |

---

## 11. Traps to avoid

1. **Don't inject a rhythm** — the brain resists it (E0/E2). Give it a *prediction* to
   fulfill (§2.4).
2. **Don't average over time — index by phase.** The `KeyframeAverager` time-mean is the
   "averaged out" failure (§2.3).
3. **Don't expect any representation to exclude standing.** Only the HK/boredom
   anti-freeze does; if the map bakes toward standing, the anti-freeze is too weak (§2.5).
4. **Don't impose the coordination topology** if you want it to improve — impose a
   predictive objective and let the antiphase diagonal emerge (§2.2, Gate 2).
5. **Don't bake keyframes on a fitness weight** — cross-cycle consistency only, or you
   re-import Goodhart (§2.5).
6. **Don't distill a percept from the oracle** (`vision_steer`'s current form) — a copy
   dies with its source (§4). A learned percept must have its own prediction error.
7. **Don't add a sensor that redefines the creature** — scalar/bilateral, direction from
   motion; no ground-truth compass, no instantaneous depth (§4 morphology honesty).
8. **Don't feed a stripped lower loop, and don't inject joint bias** — nav drives the
   FULL controller through `steer`/`speed` as a command (§2.4, §5).
9. **Don't tune the arbiter crossover** — z-score-spike-vs-level + adaptive hysteresis
   from the system's own dynamics (§6).
10. **Verify the consumer fires** — the arbiter gain must mute the loser AND pause its
    learning; a published gain nobody acts on is a silent dead channel (§8).
11. **Reset-mask everything; gate loud; verify sign against behavior, not geometry** —
    the reset artifact burned the μ=0.55 ignition claim, and a geometric sign check
    nearly reversed a working gait (§8).

---

*Living document. Prove each part ALONE in the simplest env that gives it content; let
them become emergently complementary. Fold proven/falsified findings back into
`docs/brain_building_doctrine.md`.*
