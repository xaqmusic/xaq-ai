# The Playful Machine source — deep dive, and what transfers to the picrawler

**Date:** 2026-08-02 · **Source:** `~/Documents/PlayfulMachine/` (two distributions:
`playfulmachines-1.1` = simulation sources; `ThePlayfulMachine-Experiments-1.0-x86_64-kernel-3`
= the same sims packaged with a CDE-bundled binary tree).
**Occasion:** the deployed gait is a tripod shuffle that does not progress; PM's hexapod and
humanoid show louder emergence. What is actually different?

> **Evidence discipline.** Everything below marked ✅ is read directly from files in the
> download. Marked ⚠️ is recovered from the compiled `libselforg.so` (parameter names and
> their description strings) — real, but structure-only. Marked 📖 is from the book and is
> **not verifiable in this download**; treat as hypothesis, not fact.

---

## 0. Executive summary — in plain language

*No equations. Read this section alone if you want the argument without the machinery.*

### What both projects are trying to do

Neither our robot nor the Playful Machine's robots are ever *told* to walk, and neither is
rewarded for walking. Both run on the same idea: **the robot tries to keep its own body as
lively as possible while still being able to predict what that body will do next.** A robot
standing perfectly still is very predictable but not lively at all. A robot thrashing at random
is lively but completely unpredictable. Walking sits in between — and that is why walking tends
to fall out of the arrangement without anyone asking for it. This is *homeokinesis*, and it is
the same family our Motor-EPM belongs to.

The two projects therefore ought to behave alike. They don't: the Playful Machine's six-legged
and human-shaped robots find rich, repetitive movement; ours settles into a shuffle. This
report asks why, by reading their source.

### What we found on their side

**Their whole six-legged brain is five settings.** No step timer, no prescribed leg order, no
posture controller, no balance controller, no steering. Just the learning rule and a wire from
each joint's sensor to that joint's motor.

**Their help comes from the body, not the brain.** Every one of their walking experiments runs
at about 60 % of Earth gravity, on rubber-grippy ground, with springy passive foot joints, and
with an invisible hand that stops the robot tipping past a certain angle. They made the *body*
easy to move well, then left the *brain* alone.

**We did the opposite.** Our brain block carries fifty-five settings — including a
hand-written trot pattern ("front-left and back-right together, then the other two") that the
legs are actively pulled toward. Our body meanwhile runs at full gravity with rigid legs. We
made the body hard and then added brain machinery until it walked.

**Their controller is switched on already trembling.** This is the single biggest difference in
the learning rule. Their robot's sensor-to-motor connection starts strong enough that the body
is right at the edge of instability from the first instant — it twitches on its own, and the
learning rule's job is to *shape* that twitching into a gait. Ours starts limp and silent, and
we had to bolt on three separate mechanisms (a random-flailing warm-up period, an
anti-freezing term, and a permanent noise injection) to shake it awake. We were manufacturing,
badly, the liveliness they get for free.

Two smaller ones: they have a principled way of stopping the controller from pushing itself
into a corner, whereas we replaced that piece with a hand-picked number; and their error
measure is scale-free, so it behaves the same whether the robot is moving a little or a lot.

### What we found on *our* side — and this is the important part

Reading their code sent us back to read ours properly, and turned up four things that are
problems regardless of anything the Playful Machine does.

**1. The robot is shouting through a megaphone that's stuck at maximum.** Our brain produces
commands on a scale of roughly ±3. The body only accepts −1 to +1; anything beyond is flattened
to the limit. On top of that, six other mechanisms (the power stroke, posture, height, leg
coupling, and so on) each add their own contribution *before* that flattening happens — and the
power stroke alone is already over the limit for most of its cycle.

Two consequences. First, the learning rule believes it still has most of its influence
available at the exact point where it in fact has none, so it is optimising against a picture
of the body that isn't true. Second — and this is the practical one — **when everything is
already pinned at the limit, adjusting the timing of one contribution barely changes what
comes out.** That is what a mixing desk sounds like with every channel driven into distortion:
turning any single knob does almost nothing.

This matters enormously for our record. We have nine separate timing experiments in the ledger
that all produced the same non-result, and a standing finding that the robot's footsteps have
no rhythm at all — they arrive at essentially random intervals. A permanently-saturated
actuator being driven by several out-of-step rhythms would produce *exactly* that: irregular,
on/off, rhythm-free motion, and timing changes that don't register. This is the first single
explanation that covers both facts.

**2. The robot partly listens to its own voice.** What we hand the brain as "what my leg is
sensing" includes, mixed in with the real joint readings, **a copy of the command it just
sent.** That channel is perfectly predictable and perfectly controllable — which is precisely
what the learning rule is trying to achieve. So there is a way for the robot to score well at
"lively but predictable" *without moving its body at all*. A hall of mirrors inside the
objective.

**3. We measure the body's responsiveness in nine directions when the leg can only push in
three.** The remaining six get filled with a placeholder value that then dominates the answer.
Their setup avoids this entirely by keeping one sensor per motor, so the sum always works out.

**4. We have never checked whether the learning part is doing anything.** A configuration that
switches the learning off — changing exactly one number — has been sitting in the repository
unused. Nobody has run it. So we cannot presently say whether the gait we have is a *learned*
gait at all, or whether it is the hand-written stroke-and-trot machinery with a decorative
learning term riding along.

### The uncomfortable observation

Our own project rules explicitly forbid prescribing a rhythm or a leg-coordination pattern —
the whole premise is that coordination should emerge. Our deployed configuration prescribes a
trot and actively pulls the legs toward it. The Playful Machine's six-legged robot, the one
whose emergence we're trying to match, prescribes nothing.

It is at least plausible that **the shuffle we can't get past is the prescribed pattern showing
through** — and that the nine timing experiments found nothing because they were all adjusting
the phase of contributions inside an actuator that is flattened most of the time.

That is a hypothesis, not a conclusion. The next step is designed to test it.

### What happens next

**First, three measurements that change no behaviour at all** — they only add instruments and
run something that already exists:

1. How often is the command actually hitting the limit? (If the answer is "most of the time",
   a large part of our experimental record was measured through a broken instrument.)
2. Has the robot's internal self-model latched onto the echo channel?
3. Run the never-run configuration that switches the learning off, and see whether the gait
   changes.

**Then, one change at a time**, in this order: start the controller trembling; give the sensors
the kind of gentle, drifting noise their robots have and ours lack; restore the principled
version of the piece we replaced with a hand-picked number; fix the megaphone; and switch on
the servo's "push harder when the ground resists" term, which currently sits at zero and is
present in every one of their walking experiments.

Everything is judged on whether the footsteps become *regular* — which is the operator's actual
goal ("a good, repetitive, efficient stepping gait") expressed as a number we can now measure.

---

## 1. What is in the download, and what is not

✅ **Present as source:** every experiment's `main.cpp` (the full wiring: robot config,
controller choice, every `setParam`, every physics setting, every scaffold operator), the dog
body (`vierbeiner.cpp/h`), and — the one piece of real controller source — **`shortcircuit_avggrad/sos_avggrad.{h,cpp}`**, described in its own header as *"the standard
algorithm described in Chapter 3 (Homeokinesis)"*.

❌ **Absent as source:** `Sox`, `SeMoX`, `SoML`, `Sos`, `CrossMotorCoupling`,
`ForceBoostWiring`, `DerivativeWiring`, `OneAxisServoVel`, `Hexapod`, `Skeleton`. These live in
`libselforg.so` / `libode_robots.so` (binary only). Their **parameter contracts** are
recoverable from the binaries' description strings ⚠️, and their **usage** is fully visible in
the sim sources ✅.

**Consequence:** we can read the recipe exactly, and one canonical implementation of the
learning rule exactly. We cannot diff Sox's source against ours.

---

## 2. The PM legged recipe, verbatim

Every legged/limbed experiment, transcribed from source. Note how short each column is.

| | **Hexapod** (`zoo/main.cpp:289`) | **Dog** (`dog/main.cpp:126`) | **Humanoid** (`humanoid/main.cpp:266`) |
|---|---|---|---|
| controller | `Sox(1.2, false)` | `Sox(0.7, false)` (zoo variant 1.1) | `Sox(cInit, ext)` or `SoML` |
| `epsC` (controller rate) | 0.1 | 0.05 | 0.05 → 0.1 |
| `epsA` (model rate) | 0.05 | 0.01 | 0.01 |
| `Logarithmic` | 1 | 1 | 1 |
| `sense` | 1.5 | — | — |
| `damping` | — | 0.0001 | 0.0001–0.0003 |
| `s4avg` / `s4delay` | — | — | 1 / 1 |
| wiring | `ForceBoostWiring(ColorUniformNoise(0.1), 0.05)` | same, booster 0.05 (zoo 0.1) | same, booster 0–0.075 |
| sensors | joint angle, **one per motor** | joint angle, **one per motor** ✅ | joint angle, one per motor |
| ODE sensor noise | 0.01 | 0.01 | 0.01 (0.0 for one variant) |
| gravity | **−6** | **−6** | **−6** (−4 in one setup) |
| control interval | **4** (≈25 Hz) | 1 (≈100 Hz) | 2–3 (≈33–50 Hz) |
| ground | rubber, µ-high (`toRubber(20)`) | `toRubber(10)` | — |
| passive compliance | **tarsus + tarsus joints on** | ankle servos used as springs | — |
| actuation | velocity servos | `getDefaultConfVelServos()` ✅ | `getDefaultConfVelServos()` ✅ |
| anti-fall scaffold | `LimitOrientationOperator(0.3π, 30)` ✅ | `LimitOrientationOperator(0.35π, 10)` ✅ | `PullToPointOperator` / `BoxRingOperator` |
| gait phase / CPG | **none** | **none** | **none** |
| inter-leg coupling | **none** | **none** | **none** |
| postural / height / heading term | **none** | **none** | **none** |

**That is the whole hexapod brain: five controller parameters, one wiring, no gait layer.**

Our deployed `..._imufused__steplock_off.json` MotorEPM block carries **55 parameters**,
including `gait_phase=[0, π, π, 0]` (a hand-specified trot), `coupling_gain=1.55` (Kuramoto
toward those offsets), `stroke_gain=1.65`, `stroke_phase=−2.85`, `postural_gain=0.7`,
`height_homeo_gain`, `heading_bearing_hold_gain=7.0`, `nav_gain=5.0`, `stance_lift_gain=0.5`,
`motor_gain=3.0`.

**PM's scaffolds are physical (gravity, friction, passive joints, an external upright
constraint). Ours are all in the control layer.** That is the strategic difference, and it is
worth stating plainly: PM makes the *body* easy to self-organize and then leaves the
controller alone; we left the body hard and added controller terms until it walked.

---

## 3. The learning rule: theirs vs ours, side by side

### 3.1 Theirs (✅ `sos_avggrad.cpp:167`, verbatim structure)

```
xsi = x_fut − (A·y + b)                      // model error
A  += ε_A · xsi·yᵀ  (− 0.0001·A decay), clipped
b  += ε_A · xsi, clipped

z       = C·x ;  g' = g_s(z)
L       = A·(C ⊙ g') + S                     // loop Jacobian
Q       = pinv(L·Lᵀ·L)
epsrel  = diag(C·Q·A) ⊙ g' · 2 · sense       // ← the CONFINEMENT term
C_update = ( (Aᵀ ⊙ g')·Qᵀ  −  (epsrel ⊙ y)·xᵀ ) · ε_C
C += clip(C_update, ±1)

E = tr((L·Lᵀ)⁻¹)                             // reported error
```

Init: `A.toId()` (identity), `C = 0.5·I` ✅ — and in the robot sims `Sox(cInit)` with
cInit ∈ {0.7, 1.1, 1.2}.

### 3.2 Ours (✅ `cpp_core/src/ogma/modules/MotorEPM.cpp:2356`)

```
ξ    = x − (A·prev_y + b)
A   += η_M·ξ·prev_yᵀ ;  b += η_M·ξ
G    = diag(g'(z)) ,  z = C·prev_x + h (+ Cφ·ctx + Cv·ctx)
Lp   = A·G·C
P    = (Lp·Lpᵀ + ε·I)⁻¹
q    = P·ξ̃
ΔC   = 2·η_K·(A·G)ᵀ·q·(qᵀ·Lp)                 // clamped to ‖ΔC‖_F ≤ 0.05
h   += η_h·G·Aᵀ·q
// (3) anti-saturation, "surrogate for the dropped ∂G term":
C.row(i) −= sat_lr · z_i·tanh²(z_i) · prev_xᵀ
```

Init: `A = 0.01·N(0,1)`, `C = 0.01·N(0,1)` ✅ (`ensure_leg_init`).

### 3.3 The four differences that matter

| # | Theirs | Ours | Why it matters |
|---|---|---|---|
| **D1** | `C₀ = cInit·I`, **0.7–1.2** | `C₀ = 0.01·N(0,1)` | PM **starts the loop in the self-exciting regime** — spontaneous oscillation exists from tick 1 and the rule *shapes* it. We start at a dead, unresponsive fixed point where the metric gradient is nearly flat, and then need `babble_ticks`, `sat_lr`, and `explore_noise` to manufacture the motion PM gets for free. Our own header records the symptom: *"the bare metric-gradient update saturated tanh in ~2 s then froze"*. |
| **D2** | `A₀ = I` | `A₀ = 0.01·N(0,1)` | With A≈0, `L = A·G·C ≈ 0`, so `(LLᵀ+εI)⁻¹ ≈ I/ε = 100·I` for the first hundreds of ticks — the metric is pure regularizer, i.e. the update is *not* homeokinetic during the window when C is being shaped. |
| **D3** | The `epsrel` term **is** ∂/∂C through g′, weighted by `sense` | `∂G` term **dropped**, replaced by an ad-hoc `sat_lr·z·tanh²(z)` penalty | The dropped term has a **closed form in the source we now hold**. Ours is a hand-set constant standing in for a derivative — precisely the `feedback_no_tuning` anti-pattern ("a parameter that feels tunable is the signal that the adaptive mechanism which would set it is missing"). |
| **D4** | `Logarithmic=1` on **every** legged experiment ⚠️ (param exists in `libselforg.so`; semantics 📖: the error enters logarithmically, making the update scale-free) | no analogue | A log objective normalizes gradient magnitude across operating scales — doctrine §5.5 ("don't tune a constant to a signal's scale — adapt it") expressed *inside the objective*. |

---

## 4. Four structural findings on our side (measurement gaps, not levers)

These are the "you measured your harness, not your idea" class (§3.2), like the three gym
defects. Each is cheap and each, if confirmed, changes how every past timing verdict reads.

### F1 — ★★★ The motor command is hard-clipped, and the controller does not know it

✅ `MotorEPM.cpp:2472` computes `y = motor_gain · tanh(C·x + h)` with **`motor_gain = 3.0`**.
✅ `MotorEPM.cpp:2865` — 400 lines and ~7 additive terms later — applies the **only** output
clamp: `y = clamp(y, −1, +1)`.

So:

- The HK branch alone spans **±3** before the clamp. The command saturates whenever
  `|tanh(z)| > 1/3`, i.e. **|z| > 0.35** — deep in tanh's *linear* band.
- The homeokinetic gradient uses `G = diag(1 − tanh²(z))`. At the clip point, `G ≈ 0.89`: the
  controller believes it has ~89 % loop gain where the true incremental gain is **zero**.
  `L = A·G·C` therefore **systematically overstates the loop gain**, and the sensitivity
  objective is descending a fiction.
- `sat_lr` (the anti-saturation surrogate) is inert here: at |z| = 0.35, `z·tanh²(z) ≈ 0.04`.
  **The clip bites long before the tanh does**, so the mechanism built to prevent saturation
  does not engage against the saturation that is actually happening.
- `stroke_gain=1.65` alone exceeds ±1 over most of its cycle. The clamp is applied to the
  *sum* of HK + postural + height + cruse + coupling + stroke + noise.

**Why this is the leading unified explanation for the record.** A rail-to-rail-clipped sum of
several incommensurate oscillators is a bang-bang switching process with no period — which is
exactly the two standing findings: `step_cv = 0.98` (memoryless footfall) and `flat_v` pinned
at 0.04–0.05 across **nine** isolated timing levers. If the actuator is saturated a large
fraction of the time, re-phasing one additive term among six barely moves the clipped output.
It would also explain why the load-gated stroke's gate "fired monotonically and the gait did
not care".

**The measurement:** publish per-joint **clip duty** = fraction of ticks where the pre-clamp
`|y| > 1`, plus the pre-clamp magnitude histogram. Pure instrument, no behavior change. If clip
duty is high (say >40 %), a large part of the ledger was measured through a saturating
actuator.

### F2 — The sensor vector contains the previous action (an echo channel inside the metric)

✅ `JointSensorimotorBridge.hpp:11` — each joint contributes
`[ norm_position, last_action_scalar, position_delta ]`, so per leg **n = 9, m = 3**.

Component `3j+1` of the "sensor" **is the command we just issued**. Therefore:

- The forward model can predict it *exactly* (`A(3j+1, j) → 1`) — ξ → 0 on that component.
- The controller can drive it *perfectly* — maximal apparent sensitivity.
- Homeokinesis rewards exactly this combination: high loop gain, low prediction error.

**So there is a subspace in which the HK objective can be satisfied without moving the body at
all.** This is a candidate `TAUTOLOGY`-shaped defect in the learning signal itself.

**The measurement:** inspect `A` after warmup — does the action-row converge to a one-hot? Does
`C`'s weight concentrate on columns `3j+1`? Both are already reachable via `diag_snapshot`.

### F3 — The homeokinetic metric is inverted in a space the motors cannot span

With n=9, m=3: `Lp = A·G·C` is **9×9 of rank ≤ 3**. `(Lp·Lpᵀ + 0.01·I)⁻¹` therefore has
**six directions with eigenvalue 1/ε = 100**, and `q = P·ξ` is dominated by the components of ξ
that lie in the *uncontrollable* subspace. PM keeps the loop **square** — one sensor per motor,
`One2OneWiring` — so `L·Lᵀ` is genuinely invertible and the sensitivity metric means what the
theory says it means. Our `reg_eps` is not a small regularizer; it is the dominant term in
two-thirds of the metric.

**Cheapest honest test:** run the HK core on the position components only (n = m = 3, square),
as an arm.

### F4 — The HK core's contribution has never been ablated

`the_picrawler_motor_epm_abl_no_hk.json` exists and differs from `abl_full` in **exactly one
parameter**: `ctrl_lr 0.01 → 0` ✅. That freezes C at its 0.01-scale random init, so the HK
branch contributes ≈ `3·tanh(0.01·x) ≈ 0.03` against a stroke of 1.65. **No result for this
ablation appears anywhere in the ledger.** We do not currently know whether the deployed gait
is homeokinetic at all, or whether it is the stroke/CPG/postural stack with a decorative HK
term. One n=4 run answers it.

---

## 5. What to import, ranked

Each framed per §1 as *the error the behavior minimizes*, and checked against the ledger.

### I1 — ★★★ Start the loop self-excited (`init_scale` → `cInit·I`)
**Rewrite-rule framing:** not a behavior at all — it is the *initial condition of the error
landscape*. HK's premise is that the loop is already at the edge of instability and the rule
steers the instability; starting at a dead fixed point makes the first phase of learning a
different (and nearly flat) problem.
**Change:** `C₀ = cInit·I` (sweep 0.5 / 0.7 / 1.0 / 1.2 — PM's whole observed range),
`A₀ = I`. Gain-0 guard: `cInit = 0` + random reproduces today byte-identically.
**Ledger check:** not present anywhere. Not a timing lever, so the `step_cv` blocker does not
apply. **This is the cheapest structural import and the one PM most obviously relies on.**

### I2 — ★★ Restore the real ∂G (confinement) term, retire `sat_lr`
**Framing:** replace a hand-set constant with the derivative it stands in for.
**Change:** implement `epsrel = diag(C·Q·A) ⊙ g' · 2 · sense` and the
`−(epsrel ⊙ y)·xᵀ` update term (✅ we have this source), with `sense` as the one knob (PM
hexapod: 1.5; zoo Sox generator: 4).
**Ledger check:** untried. Directly serves `feedback_no_tuning`.

### I3 — ★★ Fix the actuator honesty (follows F1, and is a prerequisite for I1/I2 to be readable)
**Framing:** the controller's model of its own output nonlinearity must match the one the body
applies, or the loop Jacobian is wrong.
**Options, in preference order:** (a) make the clamp the *only* nonlinearity and give the HK
branch unity gain into it; (b) keep `motor_gain` but apply `tanh` to the **final sum** so the
saturation the gradient assumes is the saturation the body sees; (c) leave the law alone and
lower `motor_gain` until clip duty is small. **Measure clip duty first (F1) — the number
decides which of these is even needed.**

### I4 — ★★ Colored sensor noise
✅ PM adds `ColorUniformNoise(0.1)` — ~10 % of range, **temporally correlated** — to *every*
sensor, on every legged experiment, plus ODE-level noise 0.01. Ours: `reality.proprio.joints`
is built from clamped angles with **no noise** ✅ (`picrawler_body.gd:4816`), and our only noise
is **white, motor-side, post-controller** (`explore_noise = 0.05`).
**Framing:** in homeokinesis the loop *amplifies its own sensory noise into behavior* — the
noise is the seed of the motion, not a robustness test. White noise at 52 Hz is filtered out by
servo dynamics; colored noise excites the low-frequency modes a leg can actually follow.
**Change:** colored (OU / first-order-filtered) noise on the proprio channel, σ sweep around
0.05–0.1. Gain-0 guard trivial.

### I5 — ★ Error-proportional force boost (`ForceBoostWiring`, booster 0.05)
⚠️ Recovered semantics: `wirings/forceboostwiring.cpp`, param `booster` = *"force boosting
rate"*, internal `errorForce`, asserts `cmotornumber <= csensornumber` — an error-proportional
boost of motor authority, present at 0.05 on **both** the dog and the hexapod, and swept
0–0.075 on the humanoid. 📖 the exact expression is not verifiable here.
**Our equivalent already exists and is switched off:** `SERVO_KI = 0.0`
(`picrawler_body.gd:138`). An integral term on servo tracking error *is* an error-proportional
force boost — the leg pushes harder precisely when the ground resists it, which is what makes a
stance leg propulsive rather than compliant.
**Ledger check:** untried; it is a *body* lever, and the ledger's body-side family (limb
geometry) is closed on geometry, not on actuation.

### I6 — ★ Physical scaffolds instead of control-layer scaffolds
PM runs **every** legged experiment at gravity −6 (−4 for the snake) ✅, on rubber ✅, with
passive compliant distal joints ✅, under an external upright constraint ✅. We run at full
gravity with rigid legs and buy uprightness with `postural_gain`, `height_homeo_gain`,
`stance_lift_gain` and `balance_gain`.
**Framing:** a scaffold that makes the *body* easier to self-organize can be removed later and
tested (§de-scaffold); a control term that does the same job becomes load-bearing and cannot.
**Proposal:** a reduced-gravity / raised-friction *diagnostic arm* — explicitly named a
scaffold — to answer "does our HK core produce a gait at all when the body is as forgiving as
PM's?" If it does, the target becomes de-scaffolding. If it does not, the defect is in the
controller and no amount of gait layer will fix it.

### I7 — ⚠ Whole-body C (one controller across all 12 joints) — genuinely two-sided
PM's dog, hexapod and humanoid all use **one Sox over every joint** ✅, so inter-leg
coordination lives in C's off-diagonal blocks and is *learned*. We use **four independent 3×3
per-leg controllers** and then add coordination by hand (Kuramoto + `gait_phase`).
**But the counter-evidence is also in this download:** `armband_split` ✅ runs **one independent
`Sos` per joint** (`OneControllerPerChannel`) with no shared state at all — and the book's
"spontaneous cooperation in high-dimensional systems" result is exactly that cooperation
emerging **through the body**. So split control is not PM's mistake; it is one of their
results.
**The honest reading:** split control cooperates *when nothing else is coordinating the
joints*. We have split control **and** a hand-specified trot on top. That combination is in
neither PM configuration.
**Ledger check:** `ctrl_symmetry_gain` (per-leg controller coupling) is in the refuted
symmetry family — but that is a *symmetry-forcing* lever, not "let C learn cross-leg terms".
Different mechanism; re-usable in a new form. Expensive (C becomes 12×36 or 12×12 if squared
per I3), so **sequence it after I1–I4.**

---

## 6. What NOT to import

- **`CrossMotorCoupling` / motor teaching (`gamma_teach`)** ✅ `guided_armband_cmt/main.cpp:104`
  builds a *permutation* teaching signal — motor *i* is taught toward motor
  `(i + k + len/2) mod len`'s own output, at strength γ ≤ 0.5. It is elegant (the *content* is
  the agent's own emergent motion; only the *relation* is imposed) but it is still an imposed
  coordination topology delivered as a teacher signal — §5.6 and §5.7. **And note we already
  have its cruder cousin deployed:** `gait_phase=[0, π, π, 0]` + `coupling_gain=1.55` is a
  hand-specified trot enforced by Kuramoto. Importing CMC would double down on the thing our
  own doctrine prohibits.
- **`LimitOrientationOperator` as an operating mode** — an external torque that prevents
  falling is fine as a *named scaffold* for a diagnostic arm, never as the deployed body
  (§"never disable a working loop", and a falls-count metric becomes meaningless under it).

---

## 7. The uncomfortable observation, stated plainly

The user's complaint is "stuck at a tripod shuffle". Our deployed config **specifies a trot**
(`gait_phase=[0, π, π, 0]`) and drives the legs toward it with a Kuramoto term at gain 1.55.
PM's hexapod — the body whose emergence we are trying to match — specifies **no phase
relationship at all** and has no coupling term.

Doctrine §5.7 prohibits exactly this ("don't inject a rhythm or impose a coordination
topology"). The ledger's own §1 entry for adaptive coordination already records that its
original rationale was wrong and that `coord_reward_drive` "overwrites `gait_phase` wholesale
every 240 ticks". **It is at least plausible that the shuffle we cannot get past is the
imposed topology showing through** — and that the reason nine timing levers moved nothing is
that they were all adjusting the phase of terms inside an actuator that is clipped most of the
time (F1).

That is a hypothesis, not a finding. F1–F4 are how it gets tested, and none of them requires a
new lever.

---

## 8. Recommended order

**Phase 0 — instruments only (no behavior change, no A/B needed).**
F1 clip duty + pre-clamp magnitude histogram · F2 inspect A's action-rows and C's column mass ·
F4 run the `abl_no_hk` ablation that already exists.

**Phase 1 — one lever at a time, gain-0 guarded, n≥4 seed-avg (§3).**
I1 (`cInit`) → I4 (colored sensor noise) → I2 (`sense` / real ∂G) → I3 (actuator honesty, if F1
says so) → I5 (`SERVO_KI`).

**Phase 2 — only if Phase 1 moves `step_cv`.**
I6 (scaffolded diagnostic arm) and I7 (square the loop / cross-leg C).

**Judge on the full metric set** (§3 rule 4) with `step_cv` promoted to a first-class metric —
it is the operator's goal as a number — and **observe in the UI before promoting** (§3 rule 5).

---

## 8b. PHASE 0 RESULTS — measured 2026-08-02

**Protocol:** corridor, n=4 fixed seeds, 6 000 ticks, difficulty 0.3, `seedavg.py`. Arms differ
by exactly one parameter (`mkarm.py`, diff printed). Instruments added to **both**
`snapshot_state()`'s `mod` dict and `diag_snapshot()` per the standing lesson.

**Gain-0 guard verified by measurement:** the instrumented build reproduces the recorded
baseline exactly — net_z **4.58 ± 0.27**, straight 0.73, planted 3.69, tilt_sd 0.065, 0 falls.
The instruments perturb nothing.

### F1 — Saturation: CONFIRMED, and localized to the one joint that produces locomotion

| clip duty (frac of post-warmup leg-ticks the command was flattened) | mean pre-clamp \|cmd\| |
|---|---|
| **hip1 (stride): 0.559 ± 0.044** | **1.404 ± 0.024** |
| knee: 0.179 ± 0.004 | 0.592 ± 0.009 |
| hip2: 0.010 ± 0.004 | 0.091 ± 0.012 |

**The stride joint is clipped 56 % of the time and its average requested command is 40 % past
the rail.** hip2 is never clipped; the knee occasionally. So the effect is not diffuse — it is
concentrated precisely on hip1, which is where `stroke_gain=1.65`, `steer`,
`heading_bearing_hold_gain=7.0` and the HK stride term all inject.

**What this means mechanically.** `mean|1.65·sin| = 1.05` on its own, so the power stroke alone
spends ~42 % of every cycle past the rail. The hip1 command is therefore **a clipped sinusoid —
closer to a square wave than to the smooth stroke the design assumes.** Two consequences:

1. `stroke_gain` above ~1.0 is not an amplitude control, it is a **duty-cycle** control.
2. The heading controller (gain 7.0) and the stroke compete for a channel that is already
   railed half the time, so a heading correction has full authority on one side of the stroke
   and *none* on the other — an asymmetric, direction-dependent nonlinearity nobody designed.

**A retrodiction this explains, and a test for it.** The ledger's one lever that moved distance
without cost — **shorter stride, `stroke_gain` 1.65→1.2, +12 % arena** (§6, `PARTIAL` awaiting
UI) — reduces `mean|stroke|` from 1.05 to 0.76, i.e. it is the first lever in the campaign that
*de-saturated hip1*. **Prediction: re-running `stroke_gain=1.2` with these instruments should
show hip1 clip duty far below 0.56.** If it does, "shorter stride" is not a stride result at
all — it is a saturation result, and the real lever is the operating point of the whole hip1
channel.

**Scope honestly.** My §4 framing said "most of the time"; pooled across joints it is 25 %, and
on hip2 it is nil. The claim that survives is narrower and sharper: *the stride channel is
saturated more than half the time.* Whether that explains the nine flat timing levers is now a
**testable** question rather than a rhetorical one — the levers that acted on hip1 (stroke
phase, stroke gain, the load gate, the step-lock) all acted on a saturated channel; the ones
that acted on the knee did not.

### F2 — Echo channel: the self-model HAS latched it; the controller is NOT hiding in it

| | baseline | ctrl_lr=0 |
|---|---|---|
| `echo_a` — self-model gain on the action channel | **0.945 ± 0.018** | 0.936 ± 0.021 |
| `\|C\|` mass on position columns | 0.397 | 0.709 |
| `\|C\|` mass on **action** columns | **0.159** | 0.249 |
| `\|C\|` mass on **delta** (velocity) columns | **0.445** | 0.042 |

**Half confirmed, half refuted — and the refuted half is the good news.** The forward model has
indeed latched the echo (0.945, against a theoretical ceiling of 1.0): one of the model's three
output directions is spent predicting a copy of its own command. But the controller puts
**less** weight on those columns than a uniform split would (0.159 vs 0.333), so it is *not*
preferentially exploiting the free-win subspace. The defect is real and structural — it wastes
model rank and contributes a zero-error, high-gain direction to the loop Jacobian — but on this
evidence **it is not the dominant failure**, and I am downgrading it accordingly.

**The unexpected result is the last row.** With HK learning on, the controller places **44 % of
its weight on the velocity channel**; with HK off, that collapses to **4 %**. So what the
homeokinetic gradient actually learns on this body is a **velocity-feedback law** — which is
exactly what homeokinesis is supposed to do (make the sensorimotor loop resonant). That is the
clearest evidence in this report that our HK core is doing something real and correct.

### F4 — The never-run ablation: HK is load-bearing, but the gait is mostly scaffold

`ctrl_lr` 0.01 → 0, single-parameter arm off the deployed base:

| | baseline | **ctrl_lr=0** | Δ |
|---|---|---|---|
| net_z | 4.58 ± 0.27 | **3.78 ± 0.92** | −17 %, **variance ×3.4** |
| straight | 0.73 | 0.68 | − |
| step_bal | 0.48 ± 0.11 | **0.26 ± 0.17** | **−46 %** |
| tilt_sd | 0.065 | 0.073 | worse |
| planted | 3.69 | 3.63 | ≈ |
| falls | 0 | 0 | = |
| `hk_share` of pre-clamp magnitude | **0.111 ± 0.011** | — | — |

**Read it both ways, because both readings are true.**

- **HK is load-bearing.** Removing it costs 17 % of distance, nearly halves leg participation,
  and triples seed variance. It is not decorative.
- **The gait is mostly not HK.** With the homeokinetic gradient switched off entirely, the robot
  still walks **3.78 m** — 82 % of the deployed distance — and `hk_share` says the HK branch
  contributes only **11 %** of the command magnitude in the first place. The remaining ~89 % is
  the additive scaffold stack (stroke, postural, height, coupling, cruse, noise).

**⚠️ Faithfulness caveat — this is a weakened slice, and it must be recorded as one (§3.2 rule
6).** `ctrl_lr=0` removes the *homeokinetic TLE descent* but **not** all controller learning:
`Cphi`/`Cvel` (the CPG-embedding feed-forward — the ledger's "milestone" lever) still train on
`embed_lr`, and `sat_lr` still modifies `C`. So the honest statement is **"removing the
homeokinetic gradient specifically, with the phase-conditioned feed-forward left intact, costs
17 % distance and half the step balance."** A full learning ablation (`ctrl_lr=0` +
`embed_lr=0` + `sat_lr=0`) is the follow-up and is one more `mkarm` call.

### What Phase 0 changes about the plan

- **`step_cv` did not move** (1.029 baseline vs 1.058 no-HK). The footfall irregularity is not
  produced by, nor fixed by, the homeokinetic core. It is a property of the scaffold stack.
- **Promote a new candidate to the front of Phase 1: the hip1 operating point.** F1 says the
  channel that produces locomotion is bang-bang half the time. Framed as an error rather than a
  gain: *the controller should not be requesting authority the actuator cannot deliver* — the
  clean form is to make the assembled command pass through the same bounded nonlinearity the HK
  Jacobian assumes, so `G` tells the truth. This is import **I3**, and F1 has now earned it a
  place ahead of I2 and I4.
- **I1 (`cInit`) is unaffected and stays first** — it is about where learning starts, and
  nothing in Phase 0 speaks to it.
- **Downgrade I-priority of the echo fix (F2)** from "structural defect to repair" to "wasteful
  and worth removing when the state vector is next touched" — the controller is not exploiting
  it.

---

## 9. Source index (for the next session)

| What | Where |
|---|---|
| The homeokinetic update, in full | `playfulmachines-1.1/Simulations/src/shortcircuit_avggrad/sos_avggrad.cpp:167` |
| Hexapod recipe | `.../zoo/main.cpp:289` (`createHexapod`) |
| Dog recipe + velocity servos | `.../dog/main.cpp:126`, `.../dog/vierbeiner.h:110` |
| Humanoid recipe + scaffold operators | `.../humanoid/main.cpp:266` |
| Split control (cooperation through the body) | `.../armband_split/main.cpp:62` |
| Cross-motor teaching (permutation CMC) | `.../guided_armband_cmt/main.cpp:98` |
| Controller parameter contracts | `strings ThePlayfulMachine-.../cde-root/usr/lib/libselforg.so` |
