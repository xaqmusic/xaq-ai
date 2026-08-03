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

### The three measurements came back — in plain language

**The megaphone problem is real, and it is all in one joint.** Pooled across the whole leg the
command is over the limit about a quarter of the time — but that average hides everything. The
two joints that lift and fold the leg are essentially never over the limit. **The joint that
swings the leg forwards and backwards — the one that actually produces walking — is over the
limit 56 % of the time, and on average asks for 40 % more than the body can give.** Its motion
command is closer to an on/off square wave than to the smooth push the design intends. Three
different mechanisms (the power stroke, the steering, and the learned controller) are all
shouting down that same overloaded channel.

This also retroactively explains our one recent success. The single change that improved
distance without costing anything else — shortening the stride — happens to be exactly the
change that brings that joint back under the limit. If that holds up when we re-measure it,
then "shorten the stride" was never really about stride length; it was about un-jamming the
channel, and there is more to gain there.

**The self-echo is real but harmless — and it turned up something better.** The robot's
internal self-model has indeed latched onto the copy of its own command, almost perfectly
(0.95 out of a possible 1.0). But the *controller* is not leaning on that shortcut — it puts
less weight there than an even split would. So it's wasteful rather than damaging, and I've
downgraded it. What we found instead is genuinely encouraging: with learning switched on, the
controller puts **44 % of its attention on how fast its joints are moving**; with learning
switched off, that drops to **4 %**. Learning to respond to its own motion — rather than just
its own position — is precisely what this kind of learning is supposed to produce. **Our
learning core is doing something real and correct.**

**Switching the main learning rule off costs real ground — but not everything.** The robot still
walks 3.78 m against the normal 4.58 m. It walks less far, far less consistently (three times
the run-to-run variation), and the legs share the work about half as evenly. So the learning is
not decorative; it is also not the whole gait.

**And then the follow-up run corrected me.** I ran a fuller version — switching off *all three*
learning components at once — expecting a cleaner version of the same answer. Instead the robot
**did not move at all**: zero steps, zero body sway, belly sagging to the floor. That looks like
a dramatic result, and it is a false one. The instruments we had just added showed why within
seconds: every joint was jammed against its limit **99 % of the time**, asking for **nine times**
what the body can deliver.

The cause is a plumbing fault, not a fact about learning. One of the three things I switched off
turns out to be the only brake on a slowly-accumulating internal offset. Remove the brake and
that offset grows without limit until every joint is pinned at its stop and the robot is a
statue. This is the second time in this project that an unbraked accumulator has manufactured a
fake result — and this time we caught it in minutes because the instruments were already there.

**One thing that follows is more valuable than the run itself:** a component we documented as a
minor mathematical patch is in fact the only brake on that accumulator, and is holding the whole
controller together. Switching off *just* that one component — leaving everything else running —
stops the robot walking almost entirely (it travels 12 cm instead of 4.6 m, and three of four
runs take **zero** steps). The improvement I proposed for it earlier in this report — replacing
it with the Playful Machine's principled version — would have walked straight into this failure.
That trap is now marked, and the fix is known: the Playful Machine's controller carries a
separate damping setting that we have no equivalent of, and this is almost certainly what it is
for.

### ★★★ And then the corrected experiment landed, and it is the most important result here

With **every** learned component switched off properly — the learning rule, the second learned
component, and the accumulator all cleanly disabled — the robot walks **4.75 m**.

The normal robot, with all its learning running, walks **4.58 m**.

**Turning off the entire learned controller does not make our robot walk worse. If anything it
walks slightly further, and straighter-legged runs are just as stable, just as upright, with the
same zero falls.** The only things the learning buys are ~14 % more footsteps and somewhat
better sharing of work between the four legs.

So the honest description of what we have built is: **a hand-written walking script, with a
learning system attached that the script has been tuned around.** The distance, the straightness,
the stability, the obstacle clearance — that is the stroke pattern, the prescribed trot, the
posture reflex, the height reflex and the heading controller. Not the brain.

That reframes the original question. We asked "why is our emergence weaker than the Playful
Machine's?" The measurement says: **on distance, we do not currently have emergence to compare.**
Their six-legged robot's walking is ~100 % the learned part. Ours is ~0 %.

This is *not* a verdict on our learning core — the earlier result showed it learns something real
and correct (responding to joint velocity, exactly as this class of algorithm should). It is a
verdict on **where we have been measuring it**: inside a configuration whose hand-written parts
are loud enough to drown it out. Every experiment in our ledger was run in that configuration.

**A caution on how strongly to read this.** Four runs is a signal, not proof — and "no difference"
claims need more runs than "big difference" claims. This needs repeating at scale before it goes
in the record as settled. But it points the same way as three other things we already knew, so I
would not bet against it.

**What we do about it:** stop testing improvements against the scripted configuration, and start
testing them against a stripped-down one where the learning is the only thing driving the legs.
That configuration already exists in the repository. Everything from here is measured there
first.

**A smaller prediction, half right.** I predicted that our one recent success — shortening the
stride — worked by un-jamming the overloaded joint. The measurement says: partly. It does reduce
the overload (the joint's average request drops 26 %), and it produces **27 % more footsteps**
with better-balanced legs. But the joint is *still* over the limit half the time, so shortening
the stride is a less-jammed setting rather than an un-jammed one. The real fix is the plumbing
change, not a different stride number.

**And the footstep irregularity is untouched by any of this** — it measured the same with
learning on and off. Whatever makes our footfalls arrhythmic lives in the hand-written
scaffolding, not in the learning core.

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

> **⚠️ Written BEFORE measurement. All four were then measured — see §8b for what survived.**
> F1 confirmed but localized to hip1 (and it is a *deployed-config* problem: on `pure_hk` the
> actuator is honest). F2 half-confirmed — the model latches the echo, the controller does not
> exploit it. F4 has now been run, and its answer (§8b F5) is stronger than what is predicted
> here. This section is kept as the pre-registration of the predictions.

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

> **⚠️ SUPERSEDED by "What Phase 0 changes about the plan" at the end of §8b.** The measured
> result changed the ordering *and* the base every lever is read on. Kept as written.

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

> **⚠️ SUPERSEDED by F5.** This arm zeroed only `ctrl_lr`, leaving `Cphi`/`Cvel` and `sat_lr`
> learning. Read F4b (why the first correction attempt was invalid) and then **F5**, which is
> the faithful ablation and reverses this section's headline: with ALL controller learning off
> the gait is *unchanged*, not degraded. Kept for the audit trail.

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

### F4b — ⚠️ THE FULL ABLATION WAS INVALID, AND THE INSTRUMENTS CAUGHT IT

The follow-up arm meant to close F4's caveat — `ctrl_lr=0` **+** `embed_lr=0` **+** `sat_lr=0` —
produced a robot that does not move at all: net_z **0.01 ± 0.02**, `steps` **0**, `tilt_sd`
**0.000**, chassis_y 0.058 → **0.028**, belly clearance 0.023 → 0.006, planted exactly 3.00.

**It is not an ablation result. It is an integrator windup**, and the Phase-0 instruments named
it on sight:

| | baseline | `nolearn` (3-param) |
|---|---|---|
| clip duty, hip1 / hip2 / knee | 0.56 / 0.01 / 0.18 | **1.00 / 0.98 / 0.99** |
| mean pre-clamp \|cmd\|, hip1 | 1.40 | **12.68** |
| `motor_tle` | 0.250 | **0.0000** |

**Cause, confirmed in source:** the controller-bias update `h += bias_lr · μ`
(`MotorEPM.cpp:2432`) is **not gated by `ctrl_lr`**, and the only term that pushes `h` back is
the anti-saturation term `h -= sat_lr · z·tanh²(z)`. Setting `sat_lr = 0` therefore removes the
sole bound on an integrator that keeps running — `h` winds up, `tanh` pins at ±1, the command
goes to `motor_gain·1 = 3` plus the scaffold stack, and every joint sits on its rail forever.
`motor_tle = 0.0000` is the tell: a body frozen against its stops is trivially predictable.

**This is a §3.2 rule-7 catch (the arm you think you ran is not the arm that ran)** and it is
the second time in this campaign an unbounded integrator has produced a spurious verdict (cf.
the 2026-07-26 windup entry in the ledger). The corrected arm adds `bias_lr=0`.

**It also documents an undocumented, load-bearing role for `sat_lr`.** The code comments call it
"anti-saturation — surrogate for the dropped ∂G term". It is *also* the only thing bounding the
bias integrator. **Import I2 (restore the real ∂G term and retire `sat_lr`) must therefore
supply a bound for `h`, or it will reproduce exactly this failure.** That is a live trap and it
was invisible before this run.

### F4c — the earlier `nohk` arm survives, but `hk_share` does not mean what it looks like

The `ctrl_lr=0`-only arm (F4) kept `sat_lr = 0.02`, so `h` stayed bounded and that arm is valid.
Its result stands: removing the homeokinetic gradient alone costs net_z 4.58 → 3.78 and halves
step balance.

But pairing it with the invalid arm exposes a measurement error in my own reading of `hk_share`.
I wrote that HK "supplies only 11 % of the command magnitude" and glossed that as *the gait is
mostly scaffold*. **Magnitude share is a blind metric for causal importance** — the scaffolds
carry ~89 % of the magnitude and, as the windup arm shows in the extreme, magnitude without the
right learned modulation produces *zero* locomotion. `hk_share` measures how loud a term is, not
how much it matters. Recorded here as an instrument caveat: **read `hk_share` only alongside an
ablation, never as a standalone claim about contribution.**

### F1b — the de-saturation retrodiction: directionally right, magnitude insufficient

`stroke_gain` 1.65 → 1.2, single parameter, n=4 corridor:

| | baseline (1.65) | **1.2** |
|---|---|---|
| hip1 mean pre-clamp \|cmd\| | 1.404 | **1.038** (−26 %) |
| hip1 clip duty | 0.559 | **0.476** |
| **steps** | 50.0 ± 5.6 | **63.8 ± 7.5 (+27 %)** |
| step_bal | 0.48 | 0.52 |
| net_z | 4.58 ± 0.27 | 4.70 ± 0.58 (tie; variance ×2) |
| straight | 0.73 | 0.75 |
| tilt_sd | 0.065 | 0.074 |

**The predicted arithmetic held exactly** (mean\|1.65·sin\| = 1.05 → mean\|1.2·sin\| = 0.76,
and the measured channel mean fell 1.40 → 1.04). **The behavioural consequence is `steps`, not
distance** — 27 % more footfalls, better leg balance, distance a tie in the corridor exactly as
the ledger recorded.

**But the retrodiction does not carry the weight I put on it.** At `stroke_gain=1.2` hip1 is
*still* clipped 48 % of the time and its mean request is *still* 1.04 — above the rail. 1.2 is a
**less-saturated** operating point, not a de-saturated one. So "shorter stride" is not simply "a
saturation result in disguise"; saturation is part of what it moved, and the part it moved shows
up in step count. A genuine de-saturation test needs the stride channel brought under the rail —
which is import **I3**, not a `stroke_gain` value.

### ★★★ F5 — THE HEADLINE: with ALL controller learning off, the gait is unchanged

The corrected full ablation (`ctrl_lr=0` + `embed_lr=0` + `sat_lr=0` + `bias_lr=0`, single arm
off the deployed base, n=4 corridor 6 000):

| arm | net_z | straight | flat_v | steps | step_bal | tilt_sd | planted | falls |
|---|---|---|---|---|---|---|---|---|
| **baseline** (all learning on) | 4.58 ± 0.27 | 0.73 | 0.05 | 50.0 | **0.48** | 0.065 | 3.69 | 0 |
| `ctrl_lr=0` only | 3.78 ± 0.92 | 0.68 | 0.04 | 57.3 | 0.26 | 0.073 | 3.63 | 0 |
| `embed_lr=0` only | 4.23 ± 0.50 | 0.71 | 0.04 | 41.8 | 0.29 | 0.062 | 3.60 | 0 |
| **ALL learning off** | **4.75 ± 0.48** | 0.72 | **0.06** | 44.0 | 0.40 | 0.067 | 3.66 | 0 |

**Switching off every learned component in the controller does not cost any distance. It gains
a little.** `straight`, `tilt_sd`, `planted`, `falls` and belly clearance are all ties. The only
things the learned layer buys are **step count (50 vs 44)** and **step balance (0.48 vs 0.40)**.

**The ablation is verified, not assumed** (§3.2, all seven checks):

| check | evidence |
|---|---|
| tautology? | `mkarm` printed 4 real diffs; `ctrl_lr` 0.01→0 etc. |
| consumer fired? | `hk_share` **0.111 → 0.006**; `\|C\|` mass reverts to the flat random-init profile (0.32/0.38/0.30) |
| dead code? | no — the module still runs; `motor_tle` is **0.2495 in both arms** (the *model* still learns, as intended — only the controller was ablated) |
| baseline valid? | reproduces the ledger exactly, 4.58 ± 0.27 |
| guards? | `bias_lr=0` added precisely because F4b showed the integrator was ungated |
| faithfulness? | this IS the faithful version; F4b was the weakened slice |
| silent confound? | one config, one printed diff, same gym/protocol as every other arm |

**And the shape of the partial ablations is itself informative.** Both single-component arms
(3.78, 4.23) are *worse than either extreme*. The learned components are mutually compensating,
and the whole scaffold stack was hand-tuned with them running — so removing one leaves the rest
mis-tuned, while removing all of them lets the scaffolds run clean. That is the signature of a
learned layer that the surrounding hand-built controller has been fitted **around**, rather than
one the gait is built **on**.

**Scope this correctly (§3.3).** n=4 fixed-seed is a *signal*, not a defensible null — and a
null is exactly what this is, so it needs powering (n≥20, varied seeds) before it becomes a
finding. But it is a loud signal in the one direction that matters, and it is congruent with
`hk_share=0.111`, with `step_cv` being identical across every arm, and with the entire nine-lever
flat-`flat_v` record.

**What it means, stated plainly.** In the deployed configuration, **locomotion is produced by
the hand-written scaffold stack — stroke, Kuramoto coupling toward a specified trot, postural
reflex, height homeostat, heading PD, stance-lift — and the homeokinetic controller plus the
CPG-embedding contribute nothing to distance.** The Playful Machine's hexapod is ~100 % the
learned part with no scaffold layer at all. We are at approximately the opposite end.

This is not a verdict on the Motor-EPM — F2's velocity-column result shows the HK gradient
learns something real and correct. It is a verdict on **the context we have been measuring it
in**: a config where the scaffolds are loud enough that the learned signal cannot be read.

### ★★ F6 — `sat_lr` is the sole brake on the bias integrator, and removing it destroys the gait

Single parameter, `sat_lr` 0.02 → 0, everything else at deployed values, n=4:

| | baseline | **`sat_lr=0`** |
|---|---|---|
| net_z | 4.58 ± 0.27 | **0.12 ± 0.41** |
| steps | 50.0 | **20.5 ± 35.5** — `[82, 0, 0, 0]`, three seeds took **zero** steps |
| tilt_sd | 0.065 | **0.380 ± 0.426** (one seed 1.117 — the body convulsed) |
| clip duty (pooled) | 0.25 | **0.75 ± 0.23** |
| mean pre-clamp \|cmd\| hip1 | 1.40 | **14.27 ± 11.16** — seeds at 4.2, 9.3, 13.6, **30.0** |

**`REGRESSION`, with the mechanism proven by the pairing.** The unbounded, still-growing,
seed-divergent pre-clamp magnitude is the signature of an integrator with no brake — and the
integrator is `h`, updated by `h += bias_lr·μ` with `h -= sat_lr·z·tanh²(z)` as its only
restoring term. The proof is F5: **with `bias_lr` ALSO zeroed, `sat_lr=0` is completely
harmless** (net_z 4.75, the best arm measured). So the failure is not "saturation" — it is
*bias windup*, and `sat_lr` has been silently holding the controller together.

**Consequences.**

1. **A documentation defect with teeth.** `sat_lr` is described in-source as "anti-saturation —
   surrogate for the dropped ∂G term". Nothing records that it is load-bearing for stability.
2. **Import I2 has a trap in it.** Replacing `sat_lr` with the principled `sense`/`epsrel` term
   (§5, I2) removes this brake. **Any I2 implementation must supply an explicit bound on `h`**
   — PM's Sox has a separate `damping` parameter (dog 0.0001, humanoid 0.0001–0.0003) which our
   controller has no analogue of, and that is very likely what it is for.
3. **Third windup in this campaign** (cf. the 2026-07-26 ledger entry, and F4b above). Unbounded
   integrators are this codebase's characteristic failure shape.

### What Phase 0 changes about the plan

**The plan needs a change of CONTEXT before it needs a change of levers.** F5 says the deployed
config's locomotion is produced by the scaffolds; `step_cv` is identical (1.03–1.06) in every arm
including the fully-ablated one. Running import levers against that base measures **how a lever
perturbs a script**, which is not the question. This is the ledger's own §3.1 lesson — *a
refutation is a statement about a mechanism IN A CONTEXT* — arriving from the other direction:
we have been generating verdicts in a context that cannot express the mechanism.

| | revised |
|---|---|
| **Read levers on a PM-faithful base** | The `pure_hk` tier already exists (`motor_epm_pure_hk.json`: stroke, coupling, postural, height, nav all at 0). Instrumented, it is the only context where a homeokinetic lever's effect is legible. Every import from §5 gets tested there **first**. |
| **I1 (`c_init`) still first — but on the pure_hk base** | Its whole claim is about where the loop starts. On the deployed base the answer is pre-empted by scaffolds; on pure_hk it is the question. Note `motor_gain=3.0` means the PM-equivalent of Sox's cInit 0.7–1.2 is `c_init` ≈ 0.23–0.4. |
| **I3 (`cmd_squash`) tested on BOTH bases** | On the deployed base it asks "does the hard discontinuity cost the shipped robot anything?"; on pure_hk it is a prerequisite for the HK Jacobian to be honest. |
| **I2 (real ∂G term) carries a mandatory bound on `h`** | F6. Do not ship I2 without it. Sox's `damping` parameter is the likely analogue. |
| **F2 (echo channel) downgraded** | Real but not exploited — fix when the state vector is next touched, not as a lever. |
| **The deployed base keeps one job** | It remains the reference for "does this help the robot we have". It is no longer the place where mechanisms are *judged*. |

**And one result should be carried back to the ledger regardless of what follows:** every arm in
this campaign — baseline, no-HK, no-embed, full-ablation, shorter-stride — has `step_cv` between
0.94 and 1.06. **Nothing in the controller layer, learned or hand-built, has ever moved footfall
regularity.** That is consistent with the operator's standing goal being blocked by something
outside the levers tried so far.

---

## 8c. PHASE 1 — imports built and measured

### Levers added (both default-off, gain-0 guard verified BY MEASUREMENT)

| param | what it does | default |
|---|---|---|
| **`c_init`** (import I1) | Adds `c_init` to `C(j, 3j)` — each motor's positive feedback from its OWN joint position — so the loop starts self-exciting instead of at a dead fixed point. **Added to**, not replacing, the per-leg random init, because that randomness is this module's inter-leg symmetry breaker. Effective loop gain is `motor_gain · c_init`, so the PM equivalent of Sox's cInit 0.7–1.2 is `c_init` ≈ **0.23–0.40**. | 0 = off |
| **`cmd_squash`** (import I3) | 0 = today's hard clamp of the assembled command to [−1,1]; 1 = a smooth `tanh` squash. Compresses instead of truncating, so the stroke's *shape* — and therefore its phase information — survives the bound, and the HK Jacobian's `G = diag(1−tanh²)` stops overstating loop gain at the rail. | 0 = off |

**Gain-0 guard, verified by measurement rather than asserted:** the deployed baseline re-run on
the build carrying both levers returns net_z **4.58 ± 0.27**, seeds `[4.37, 4.91, 4.27, 4.79]` —
identical seed-for-seed to the pre-lever build, and to the ledger's recorded value.

### I3 (`cmd_squash=1`) on the DEPLOYED base — `NULL` / slight `REGRESSION`, as predicted

| | baseline | `cmd_squash=1` |
|---|---|---|
| net_z | 4.58 ± 0.27 | 4.17 ± 0.50 |
| straight | 0.73 | 0.70 |
| planted | 3.69 | 3.54 |
| steps | 50.0 ± 5.6 | 58.5 ± 16.0 |
| step_bal | 0.48 | 0.45 |
| tilt_sd | 0.065 | 0.063 (tie) |
| falls | 0 | 0 |

**This is what F5 predicts, and it is worth stating as a passed prediction rather than a failed
lever.** On a base where the learned controller contributes nothing to distance, making the
actuator honest *for the learning rule* cannot buy anything — all it does is compress the
scripted stroke's peak by ~11 % (`tanh(1.4) = 0.885`). **Re-use context: `cmd_squash` is a
prerequisite for the HK Jacobian to be honest, so it must be re-read on the `pure_hk` base,
where it is a mechanism question rather than a stroke-amplitude question.**

Note the instrument behaves correctly here: `clip_duty` still reads 0.55 on hip1 because it
measures the *request*, which the squash does not change — only what the body receives changes.

### I1 (`c_init`) on the PM-faithful base — the control arm first

`motor_epm_pure_hk__inst.json` (stroke, coupling, postural, height, nav all 0; HK + explore
noise only), `c_init = 0`, n=4:

| net_z | chassis_y | bellyc | steps | step_bal | turns | falls |
|---|---|---|---|---|---|---|
| **−0.03 ± 0.02** | **0.026** (standing ≈ 0.058) | 0.005 | 9.75 | 0.00 | ±2–3 | 0.25 |

**The pure homeokinetic core does not locomote, and — importantly — it does not stay up.**
chassis_y 0.026 with belly clearance 0.005 means the body has folded onto the floor; `turns`
of ±0.7 to ±3.0 across seeds means what motion exists is spin. This matches the tier's own
description ("expected to spin/fall") and is the honest starting point.

**The instruments confirm the context is finally the right one, and re-scope two earlier
findings:**

| instrument | deployed base | **pure_hk base** | reading |
|---|---|---|---|
| `hk_share` | 0.111 | **0.991** | HK is now ~100 % of the command — this is the context where an HK lever can be read |
| clip duty (hip1) | 0.559 | **0.044** | **F1's saturation is a property of the SCAFFOLD STACK, not of the homeokinetic controller.** On the PM-faithful base the actuator is already honest |
| `echo_a_gain` | 0.945 | **0.991** (and **1.0007** at `c_init=0.25`) | **F2's echo latch is TOTAL here.** With no scaffolds, the self-model spends a full one of its three output directions predicting a copy of its own command |
| `contact_duty` | 0.797 | **0.232** | the pure-HK body has its feet on the ground less than a quarter of the time |

**So F1 and F2 swap places on this base.** Saturation is a deployed-config problem and simply
does not arise here; the echo channel, which was mild in the deployed config, is **complete** in
the context where HK is the only driver — which is exactly where it does the most damage,
because one third of the model's output rank is spent on a channel that cannot move the body.

### I1 (`c_init`) full sweep on the PM-faithful base — `WORKING` on activity, `NULL` on locomotion

`c_init = 0.25` ≈ PM's Sox `cInit = 0.75` at our `motor_gain = 3.0`; 0.5 and 1.0 go beyond PM's
observed range (≈ cInit 1.5 and 3.0 equivalent).

| `c_init` | net_z | chassis_y | belly | **steps** | **step_bal** | tilt_sd | falls |
|---|---|---|---|---|---|---|---|
| **0** (control) | −0.03 | 0.026 | 0.005 | 9.75 | **0.00** | 0.064 | 0.25 |
| **0.25** | −0.07 | **0.038** | **0.036** | 20.0 | **0.32** | 0.567 | 2.00 |
| **0.5** | +0.10 | 0.033 | 0.017 | 18.0 | 0.19 | 0.270 | 0.25 |
| **1.0** | +0.11 | **0.038** | 0.030 | **32.8** | 0.22 | 0.395 | 1.00 |

**Every non-zero `c_init` converts a folded, inert body into an active one.** The chassis rises
(0.026 → 0.033–0.038), belly clearance improves 3–7×, step count multiplies 2–3×, and leg
participation goes from **literally zero** to 0.19–0.32. Activity rises broadly monotonically
with `c_init`; the *stability* cost is non-monotonic (0.25 is the wobbliest, 0.5 the calmest —
consistent with higher `C` pushing `z` further into `tanh`, where `g'` shrinks and the effective
gain self-limits).

**Verdict: the mechanism is confirmed and the lever does what PM's source says it should — but
activity is not locomotion.** No setting produces meaningful forward progress (net_z ≤ 0.11
against a deployed 4.58), and the body cannot hold itself up at any of them.

**This isolates precisely what is missing.** We have now implemented PM's excitation and
verified it works. What we have *not* implemented is the half PM pairs it with: **physical
uprightness.** Their hexapod runs this same self-exciting controller behind a
`LimitOrientationOperator` capping body tilt at 0.3π with strength 30, at **gravity −6**, on
rubber (`toRubber(20)`), with compliant passive tarsus joints. **We have built one half of a
two-part recipe and measured exactly the failure mode the other half exists to prevent.**

**It also isolates what is actually missing.** We have now added PM's excitation and confirmed
it works. What we have not added is the thing PM pairs it with: **physical uprightness.** Their
hexapod runs the same self-exciting controller behind a `LimitOrientationOperator` capping body
tilt at 0.3π, at gravity −6, on rubber, with compliant passive foot joints. **Excitation plus a
physical anti-fall scaffold is their recipe; we have implemented one half of it and measured
exactly the failure mode the missing half prevents.**

**Read this against §2 before interpreting it as a verdict on HK.** PM's robots have no postural
control term either — they are kept upright by **physical** scaffolds: `LimitOrientationOperator`
(an external torque capping body tilt), gravity −6 instead of −9.81, rubber ground, and passive
compliant distal joints. Our `pure_hk` tier removes the *control-layer* uprightness (`postural_gain
= 0`) without substituting the *physical* uprightness PM relies on. **So the pure-HK arm as
configured is not the PM-faithful comparison — it is PM's controller on a body that PM's
experiments never used.** That makes import **I6 (physical scaffolds, named as scaffolds)** a
prerequisite for the comparison rather than a Phase-2 nicety.

---

## 8d. PHASE 2 — the physical scaffold (import I6)

**Why this became the next step rather than a Phase-2 nicety.** Phase 1 confirmed PM's
excitation works (I1) and left the body unable to stay upright. §2 records that *all* of PM's
legged emergence is measured under physical scaffolds we do not have. So the open question is
no longer "which controller lever next" but **"does our homeokinetic core produce a gait when
the body is as forgiving as the one PM's results come from?"**

**Built:** `scaffold_gravity_scale` on `picrawler_body.gd` (export + `OGMA_PICRAWLER_GRAVITY_SCALE`
env override, read before `_build_body`), applied as `gravity_scale` to the chassis and all
twelve leg segments. **1.0 = off = byte-identical.** PM equivalent = 6/9.81 = **0.61**.

**It is named as a scaffold in the code and it must be named as one in every claim.** Its
purpose is diagnostic: it tells us whether the substrate can express a gait at all. A result
that only exists at 0.61 g is a result about a scaffolded body, and de-scaffolding is then the
work — exactly the pattern the doctrine's scaffold / de-scaffold vocabulary exists for.

**Arms run:** deployed baseline with the knob unset (the byte-identical guard), then
`pure_hk` + `c_init=0.25` at 0.61 g, and `pure_hk` + `c_init=0` at 0.61 g as the scaffold-only
control — so gravity and excitation are separable rather than confounded.

**Guard:** deployed baseline with the knob unset returns net_z **4.58 ± 0.27**, seeds
`[4.37, 4.91, 4.27, 4.79]` — byte-identical.

### Result: `NULL`. Reduced gravity is NOT the missing half.

| | 1.0 g | **0.61 g** |
|---|---|---|
| `c_init=0` — chassis_y | 0.026 | **0.026** (identical) |
| `c_init=0` — belly clearance | 0.005 | **0.005** (identical) |
| `c_init=0` — net_z | −0.03 | +0.09 ± 0.19 |
| `c_init=0` — turns | ±0.7–3.0 | **±1.1–5.6** (spinning harder) |
| `c_init=0.25` — chassis_y | 0.038 | 0.035 |
| `c_init=0.25` — net_z | −0.07 | −0.02 |
| `c_init=0.25` — steps | 20.0 | 24.0 |
| `c_init=0.25` — tilt_sd / falls | 0.567 / 2.00 | 0.387 / 1.50 (mildly calmer) |

**At 0.61 g the un-excited body collapses to exactly the same chassis height and exactly the same
belly clearance as at 1.0 g.** Reducing gravity buys a little less wobble and nothing else. No arm
locomotes.

**Diagnosis — why the scaffold missed.** Gravity was never the binding constraint on *standing*:
the servos are torque-limited at 0.15 Nm against a static hip2 load of ~0.037 Nm, i.e. ~4×
headroom already. PM's gravity reduction buys them **dynamic** margin (slower falls, more time
for the loop to react), not the ability to hold a pose. Our body's inability to stand under
`pure_hk` is therefore not a load problem, and lightening it does not touch the cause.

**What the identical 0.026 / 0.005 numbers actually point at.** With all learning ablated on the
*deployed* config the body stands perfectly well (chassis 0.058). The difference between that
config and `pure_hk` that governs standing is **`postural_gain` (0.7 → 0)** — the spinal-tone
reflex, which the deployed stack already carries as a promoted lever and which the doctrine
treats as a reflex rather than a gait scaffold. **So the honest next test is not more physical
scaffolding: it is `pure_hk` + the stance reflex, which asks the actual question —**

> *Given a body that can hold itself up, does the homeokinetic core generate locomotion?*

That is a single parameter on the `pure_hk` base, it isolates standing from propulsion (no
stroke, no coupling, no gait phase, no heading, no height homeostat), and it is the closest
configuration we can build to what PM's hexapod actually is: a body that stands, with nothing
but the homeokinetic loop driving the legs.

---

## 8e. PHASE 2b — "given a body that can stand, does HK walk?"

The gravity null pointed here: `pure_hk` + the **stance reflex** (`postural_gain = 0.7`, the
promoted spinal-tone lever), with no propulsion scaffold of any kind — no stroke, no coupling,
no gait phase, no heading, no nav.

| pure_hk + stance | net_z | chassis_y | belly | steps | step_bal | tilt_sd | falls |
|---|---|---|---|---|---|---|---|
| `c_init=0` | +0.13 | 0.029 | 0.008 | **0.5** | 0.00 | 0.044 | 0.50 |
| **`c_init=0.25`** | **+0.23 ± 0.38** | **0.040** | 0.021 | **21.0** | 0.18 | **0.119** | 1.00 |
| `c_init=0.5` | +0.03 ± 0.25 | 0.038 | 0.019 | 10.0 | 0.32 | 0.147 | 0.50 |

`c_init=0.25` is confirmed as the optimum on this base too — the same non-monotonicity the
unscaffolded sweep showed, for the same reason (higher `C` drives `z` deeper into `tanh`, `g'`
shrinks, effective gain self-limits).

**Two results, and the second is the important one.**

**(a) The stance reflex alone freezes the body.** `postural_gain=0.7` with `knee_tuck_target=0.7`
— *identical* to the deployed values — holds the robot at chassis **0.029**, taking **0.5 steps
in 6 000 ticks**. The deployed config stands at **0.058**. The difference is entirely
`height_homeo_gain` (0.04) and `stance_lift_gain` (0.5), both off here.

⇒ **The picrawler's "standing" posture is actively constructed by the height homeostat, not held
by postural tone.** This is a *body* fact, and it connects directly to the ledger's kinematic
dead-end entry (feet plant at 170 mm against 166 mm total leg reach). **PM's robots have no
analogue of this problem: their servos' neutral pose *is* a standing pose.** It is a large part of
why our project needed a scaffold stack at all and theirs did not.

**(b) Excitation + stance is the best-behaved pure-HK configuration measured.** Adding
`c_init=0.25` to the frozen stance body restores activity (0.5 → **21 steps**), raises the chassis
(0.029 → 0.040), and — crucially — is **far calmer than excitation without stance**
(tilt_sd 0.567 → **0.119**, and `planted` 3.89 with `unstable` 0.00). net_z rises to +0.23, the
highest pure-HK value measured, and still ~2 % of the deployed 4.58.

### The diagnosis this converges on: HK produces motion, but not DIRECTED motion

Across every pure-HK arm — 1.0 g, 0.61 g, with and without stance, `c_init` 0 → 1.0 — the pattern
is identical: **legs move, the body does not travel.** In the deployed config the thing that
converts leg motion into translation is the hip1 power stroke with its per-leg `stroke_signs`
(`{+1, −1, +1, −1}`) — a hand-specified directional bias.

**Why this is structural rather than a tuning gap.** Our controller is **four independent 3×3
per-leg blocks**. `C` has no cross-leg terms at all, so it *cannot represent* an inter-leg phase
relationship — four uncoupled oscillators at arbitrary relative phase sum to no net thrust. PM's
dog, hexapod and humanoid all use **one Sox across every joint**, where inter-leg coordination
lives in `C`'s off-diagonal blocks and is *learned*. Their split-control result (`armband_split`)
shows cooperation emerging through the body — but with **1×1** controllers and no competing
coordinator; we have split control **and** a hand-specified trot, a combination that appears in
neither PM configuration.

⇒ **Import I7 (whole-body `C`) is promoted from "expensive, sequence it later" to the leading
candidate**, because it is the only untested import that could supply directed motion *as a
learned quantity* rather than as `stroke_signs`. Second candidate: **I4 (colored sensor noise)**,
still unbuilt and cheap — PM's loop amplifies correlated sensory noise into motion, and ours has
none.

---

## 8f. ★★★ OPERATOR OBSERVATION (2026-08-02) — corrects the analysis on three points

Both arms watched in the UI at seed 6000. Verbatim substance:

> **PURE-HK CONTROL** — "each leg settles into a separate basic behavior. The most interesting
> being the front right leg forming an **inchworm motion that drags the body forward** while the
> other legs look mostly stuck."
>
> **SELF-EXCITE (c_init=0.25)** — "a period of slow convulsions. The convulsions do morph over
> time into sort of a **crab walk with the chassis off the floor occasionally by 10k ticks**.
> There is a **hint of diagonal symmetry with front-left and rear-right working together** while
> the others drag or balance." … "the robot seems a bit **lobotomized** compared to our earlier
> work that had good emergent climbing skills. Placing this config on obstacles after ~20k ticks
> **does not exhibit the escape behaviors** that marked progress during our recent experiments."

### Correction 1 — "the controller structurally cannot coordinate the legs" was TOO STRONG

§8e argued that four independent 3×3 per-leg blocks cannot represent an inter-leg phase
relationship, and concluded that directed motion therefore requires whole-body `C`. **The first
half is still true; the conclusion does not follow.** The operator observed **diagonal
coordination — front-left with rear-right — emerging with `coupling_gain = 0` and no gait phase
in play at all.** `C` cannot *hold* a cross-leg term, but the four controllers are coupled
**through the body**, and that coupling is evidently enough to produce a trot diagonal
unaided.

**This is precisely PM's own `armband_split` result** (§5, I7): independent per-channel
controllers cooperating through the physical medium, with no shared state. We reproduced it and
the metrics did not show it.

⇒ **I7 (whole-body `C`) drops from "the leading candidate" to "one candidate."** The honest
framing is now a question rather than a fix: *coordination does emerge through the body — is it
faster/more stable if `C` can also represent it directly?* That is worth testing, but it is no
longer the diagnosis.

### Correction 2 — ★ 6 000 TICKS IS TOO SHORT FOR THIS ARM. Every pure-HK number above is pre-convergence.

The operator reports convulsions **morphing into a crab walk by ~10 k ticks**. Our entire
pure-HK campaign — the `c_init` sweep, the gravity arms, the stance arms, 16 seed-runs — ran at
**6 000 ticks.** We were measuring the transient and reporting it as the behaviour.

This is a protocol error, and it is mine: 6 000 is the *deployed* config's protocol, chosen when
the deployed config walks from ~2 400 ticks. A stripped controller that has to *find* its
behaviour needs a longer horizon, and nothing checked that assumption before 16 runs were spent
on it. **Every `NULL` on locomotion in §8c–8e is provisional and must be re-measured at ≥20 k
ticks before it means anything.**

### Correction 3 — ★★ `steps` DOES NOT SEE AN INCHWORM

The control arm reports **`steps` 0.5 per 6 000 ticks** and `net_z` +0.13, and the operator
watched a front-right leg inchworm the body forward. `steps` counts foot-lift events above a
height threshold; a leg that flexes and drags without clearing that threshold registers nothing.
**So the arm the metrics called "essentially frozen" was in fact locomoting by a mechanism the
instrument cannot see.** Add to the standing blind-metric list alongside `turns`, chassis height
and `fwd_v`.

### Correction 4 — ★★★ "COSTS NOTHING" WAS MEASURED ON A BLIND METRIC

The headline of §8b F5 — ablating all learning costs the deployed gait nothing — rests on flat
corridor metrics: net_z, straight, tilt_sd, planted, falls. **The operator's three aliveness
signals are heading regulation, proto-gait steps, and obstacle-triggered adaptation, and the
ablation was never tested against the third.** The observation that pure-HK shows *no escape
behaviour on obstacles after 20 k ticks* is the direct warning: **flat distance can be preserved
while the adaptive capability is gone.**

That is the same blind-metric failure this ledger has already recorded twice (distance certifying
a paddling sequencer; `turns` blind to a body that swings and nets zero). **The obstacle gate on
the full-ablation arm is therefore a required test, not an optional one.**

#### The gate ran, and F5 SURVIVES it — with a caveat that is now the sharper question

`humpavg.py`, teleport onto the hump crest at tick 3 000, n=4:

| | baseline (all learning on) | **ALL learning off** |
|---|---|---|
| final_z | 4.52 ± 0.45 | **4.50 ± 0.19** |
| gain_z (post-teleport) | 1.91 ± 0.45 | **1.88 ± 0.20** |
| belly clearance | 0.021 | 0.020 |
| belly min | 0.002 | 0.004 |
| falls | 0 | 0 |

**Identical, at lower variance.** So the learned layer is not carrying hump traversal either, and
the F5 null now stands against two of the operator's three aliveness signals rather than one.
I expected this test to overturn the headline; it did not.

**But the caveat is the more useful output.** The ledger already records that hump clearance
"works by letting the belly ride low, which **may be a sim exploit**" — i.e. the robot may be
*bulldozing* the hump rather than negotiating it. **A gate that a fully-scripted walker passes
identically is not a test of adaptation.** So the correct conclusion is not "adaptation is
intact" but:

> **Our obstacle gates cannot distinguish a learned adaptive system from a scripted one.** The
> ablation passes both, which is evidence about the *gates* as much as about the ablation.

The operator's criterion is sharper than either gate: *error spikes on contact, the body feels
around and sometimes traverses — proving plasticity is still live late in a run.* A scripted
walker shows no exploratory variation on contact and no early-vs-late difference. **Building that
comparison is now the open instrument job**, and `recoveravg.py` (4 perturbations per run,
measures whether progress resumes and whether coordination returns) is the closest existing tool.

**One scoping note on the operator's "lobotomized" observation:** it was made on the **pure-HK**
arms, which have no panic reflex, no stuck→explore, no height homeostat and no amplitude
homeostat. It does not bear directly on the ablation arm, which retains all of those. The escape
behaviour being missed may live in those reflexes rather than in the learned controller — and
this gate result is consistent with exactly that.

---

## 8g. The 20 k re-measurement, and the operator's correction to their own report

### The window analysis — the control DIES, the self-excited arm DOES NOT

Whole-run aggregates average the transition away, so each 20 k run was split into windows and
read as per-tick **rates** (`windowavg.py`, new tool, per 1000 ticks, n=4):

| | **PURE-HK CONTROL** (`c_init=0`) | | | **SELF-EXCITE** (`c_init=0.25`) | | |
|---|---|---|---|---|---|---|
| window | early 0–6k | mid 7–13k | late 14–20k | early 0–6k | mid 7–13k | late 14–20k |
| **step rate** | 0.89 | **0.00** | **0.00** | 4.80 | 1.35 | **1.43** |
| tilt_sd | 0.208 | 0.031 | 0.027 | 0.320 | 0.084 | 0.076 |
| path rate | 0.267 | 0.185 | 0.175 | 0.549 | 0.484 | 0.459 |
| disp rate | 0.039 | 0.035 | 0.065 | 0.084 | 0.101 | 0.065 |
| chassis_y | 0.031 | 0.027 | 0.027 | 0.041 | 0.033 | 0.033 |

**The control converges to a frozen state.** Its step rate reaches **exactly zero by 7 k ticks and
stays there** for the remaining 13 000. That is the dead-fixed-point failure our own source
comments describe ("the bare metric-gradient update saturated tanh in ~2 s then froze").

**The self-excited arm does not die.** Its activity falls from the violent opening (4.80 → 1.35)
and then **holds — 1.35 → 1.43, slightly rising — through 20 k**, while tilt_sd falls
monotonically 0.320 → 0.076. **That is the operator's "convulsions morph into a crab walk",
quantified**, and it is precisely the failure mode PM's `cInit` exists to prevent.

**Neither transports.** Late disp rate is **0.065 for both**, while the self-excited arm's path
rate is 2.6× higher — it is churning in place. **`c_init` buys sustained activity, not transport.**

**This vindicates Correction 2 concretely.** At 6 000 ticks both arms were still in transient and
the difference was invisible; the qualitative distinction — one converges to death, the other to
sustained activity — only appears with the long horizon *and* the window split. The 6 k protocol
was measuring the wrong thing.

### ★ Operator correction to their own report: THE ESCAPE BEHAVIOUR IS THERE, IT IS JUST SLOW

> *"I should correct the 'no escape behaviors' — after writing that I placed the robot on more
> pyramids and did observe the escape. It's just very slow. The legs take a while to find the
> boredom point but will eventually increase their amplitude so movement can occur. We may see
> more interesting results with a steeper learning rate (if that applies here)."*

**This retracts Correction 4's premise and is consistent with the window data.** The arm that
"increases amplitude until movement occurs" is the arm whose activity *sustains* rather than
decays — and note **`amp_homeo_gain = 0` in `pure_hk`**, so the amplitude growth the operator
watched is not the amplitude homeostat. It is **the homeokinetic gradient itself**, destabilizing
the loop until the body becomes responsive again. That is homeokinesis doing exactly the job it
is for, on the picrawler, observed directly. It is the single most positive result in this report.

**"Steeper learning rate" applies directly, and the source comparison already flagged the gap:**

| | PM hexapod | PM dog | **ours** |
|---|---|---|---|
| controller rate | `epsC` **0.1** | `epsC` **0.05** | `ctrl_lr` **0.01** |
| model rate | `epsA` 0.05 | `epsA` 0.01 | `model_lr` 0.02 |

**We are running the controller learning rate 5–10× below every Playful Machine legged
experiment.** §2 recorded that number on day one and nothing tested it. Sweep in flight:
`ctrl_lr` ∈ {0.05, 0.1} on the self-excite base at 20 k, read with `windowavg.py`.

⚠️ **Caveat to check in the result:** `max_dctrl = 0.05` clamps ‖ΔC‖_F per tick. If that clamp
binds, raising `ctrl_lr` changes nothing and the two arms will read *identically* — which is the
free tell this ledger already learned to read (a flat sweep where the control's is structured
means the mechanism never engaged). If they are flat, `max_dctrl` is the real lever.

---

## 8h. ★★★ THE LEARNING RATE WAS THE GAP — and the operator called it

`ctrl_lr` sweep on the self-excite pure-HK base, 20 k ticks, n=4. **The sweep is not flat, so
`max_dctrl` is not binding and the mechanism engaged.**

| `ctrl_lr` | net_disp | net_z | **steps** | step_bal | tilt_sd | falls |
|---|---|---|---|---|---|---|
| **0.01** (ours) | 0.56 | +0.56 ± 0.32 | 41.0 | 0.16 | 0.099 | 1.00 |
| **0.05** (PM dog) | 0.68 | +0.36 ± 0.53 | **141.5** | **0.42** | 0.225 | 0.75 |
| **0.10** (PM hexapod) | **1.79 ± 1.53** | **+1.77 ± 1.54** | **163.3** | 0.29 | 0.279 | 1.75 |

**Steps ×4. Displacement ×3.2. One seed at `net_z` = 4.36 — inside the deployed config's range
(4.58), from a controller with no stroke, no coupling, no gait phase, no heading and no nav.**

### The window split is the real result: at PM's rate, the arm is STILL IMPROVING at 20 k

| late window (14–20k), per 1000 ticks | `ctrl_lr` 0.01 | 0.05 | **0.10** |
|---|---|---|---|
| **displacement rate** | 0.065 | 0.051 | **0.152** |
| straightness | 0.139 | 0.083 | **0.214** |
| **tilt_sd** | 0.076 | 0.103 | **0.069** |
| step rate | 1.43 | 5.35 | 5.22 |

| `ctrl_lr = 0.10` trajectory | early 0–6k | mid 7–13k | late 14–20k |
|---|---|---|---|
| displacement rate | 0.1375 | 0.1189 | **0.1524** |
| straightness | 0.1914 | 0.1648 | **0.2142** |
| tilt_sd | 0.5136 | 0.0800 | **0.0689** |

**`ctrl_lr = 0.10` is the only arm that gets BETTER late.** Displacement rate and straightness are
both *higher* in the final window than the first, while wobble falls monotonically to **0.069 —
better than the deployed baseline's 0.065 is not, but comparable to it**, and calmer than the
slower-learning arms. The other two arms decline over the run (0.084 → 0.065 and 0.070 → 0.051).

**This is the closest thing to a loud result in the whole campaign**, and it belongs to the
operator: *"we may see more interesting results with a steeper learning rate (if that applies
here)."* It did apply. §2 recorded the 5–10× gap against PM on day one and nothing tested it
until they asked.

**Scope it honestly.** n=4 with `net_z` std 1.54 (seeds 0.40 / 4.36 / 1.32 / 0.99) is a *signal*,
not a finding — the spread is larger than the mean. Falls rise 1.00 → 1.75. And "still improving
at 20 k" is a three-window trend on four seeds, which is exactly the kind of shape that needs a
longer run to confirm rather than a bigger claim. **In flight: `ctrl_lr = 0.20` at 20 k (is 0.10
the peak, or does it keep climbing past PM's range?) and `ctrl_lr = 0.10` at 40 k (does the
improvement continue, or plateau?).**

**What it does NOT yet show:** transport is still ~1.8 m against the deployed 4.58, and
`straight` is 0.13 against 0.73 — this is a body that moves, not a body that walks somewhere.

### ⚠️ Retraction: "still improving at 20 k" does NOT survive to 40 k

Both follow-ups landed and the second one corrects me.

**`ctrl_lr = 0.20` at 20 k** — past PM's range: net_disp **2.17 ± 1.65** (up from 1.79),
`steps` **226** (up from 163), but `turns` **+3.58 ± 2.30**, tilt_sd 0.335, falls 1.75. More
distance and far more churn; the useful band is 0.10–0.20 and the cost is stability.

**`ctrl_lr = 0.10` at 40 k** — net_disp **1.46 ± 1.95**, *lower* than the same arm at 20 k
(1.79), with `turns` **± 12**. The window split shows why:

| `ctrl_lr = 0.10`, 40 k | early 0–10k | mid 13–26k | late 27–40k |
|---|---|---|---|
| **step rate** | 12.02 | 5.21 | **2.95** |
| path rate | 0.742 | 0.682 | **0.577** |
| displacement rate | 0.107 | 0.080 | **0.042** |
| straightness | 0.150 | 0.118 | **0.071** |
| tilt_sd | 0.436 | 0.079 | 0.071 |

**The arm peaks around 14–20 k and then decays monotonically.** Activity falls 12.0 → 5.2 → 2.95
steps per 1000 ticks. **I over-read a three-window trend as a continuing one; it is a peak.**
The claim that survives is narrower: *raising `ctrl_lr` to PM's rate raises and delays the
activity peak by a large factor — it does not prevent the eventual decay.*

### ⇒ This indicates the LAST unbuilt import, and indicates it sharply

The homeokinetic loop still settles toward a fixed point; `c_init` sets where it *starts* and
`ctrl_lr` sets how fast it explores, but neither keeps it excited indefinitely. **In PM the thing
that does that is continuous sensory perturbation: `ColorUniformNoise(0.1)` on every sensor, in
every legged experiment** (§2), plus ODE-level noise 0.01. Our proprio channel is **noiseless**,
and our only noise is white, motor-side and post-controller (`explore_noise = 0.05`).

That is import **I4**, it is the only §5 import still unbuilt, and the decay curve above is
exactly the symptom it addresses: a loop with nothing to amplify runs down. It also matches the
operator's description of the mechanism they watched — *"the legs take a while to find the
boredom point but will eventually increase their amplitude"* — which is a loop **oscillating**
between decay and re-excitation, i.e. what homeokinesis does when a persistent perturbation keeps
feeding it.

---

## 8i. I4 — colored proprioceptive noise: `REGRESSION` at PM's dose, with the predicted mechanism faintly visible

**Built:** first-order (OU-like) colored noise on `reality.proprio.joints` in
`picrawler_body.gd` — `n ← (1−a)·n + a·U(−σ, σ)`, `a = 1/τ`, so `τ = 1` degenerates to white and
the knob spans both regimes. Seeded off `OGMA_SEED` so the trace varies per seed rather than every
seed sharing one. `sensor_noise_sigma = 0` (default) is byte-identical; guard verified — the
deployed baseline reproduces net_z 4.58 ± 0.27, tilt_sd 0.065, steps 50.0, 0 falls.

**σ = 0.1 (PM's value), on the best pure-HK arm (`c_init=0.25`, `ctrl_lr=0.10`), 40 k, n=4:**

| | no noise | **σ = 0.1** |
|---|---|---|
| net_disp | 1.46 ± 1.95 | **0.45 ± 0.32** |
| steps | 240 | 185 |
| tilt_sd | 0.208 | 0.165 |
| falls | 1.75 | 1.25 |
| turns | ±12 | ±10 |

**It costs about two-thirds of the transport.** Calmer and fewer falls, but that is the trade a
weaker gait always shows. **Verdict `REGRESSION` at σ = 0.1 on this base.**

**But the window split shows the predicted mechanism, faintly:**

| per 1000 ticks | no noise: early → mid → late | **σ = 0.1: early → mid → late** |
|---|---|---|
| displacement rate | 0.107 → 0.080 → **0.042** | 0.074 → 0.044 → **0.045** |
| path rate | 0.742 → 0.682 → **0.577** | 0.696 → 0.631 → **0.639** |
| step rate | 12.02 → 5.21 → 2.95 | 11.79 → 3.24 → **1.60** |

**Without noise, locomotor output declines all the way to 40 k. With noise it FLATTENS in the
late window** (0.044 → 0.045 displacement, 0.631 → 0.639 path) instead of continuing to fall.
That is the arrested-decay signature the import predicts — **at a level too low to be worth
having, and the step rate decays faster regardless.**

**Do not record this as "colored noise is refuted" (§3.1).** It is one dose on one base. The
honest verdict is *`REGRESSION` at σ = 0.1*, with two specific reasons to think the dose rather
than the idea is wrong:

1. **Our state vector amplifies it.** PM's controller sees one sensor per motor. Ours sees
   `[pos, action, delta]` per joint, and `delta` is a *difference of successive positions* — so
   injecting `n_t` into position also injects `n_t − n_{t−1}` into the velocity channel. The same
   σ therefore perturbs two of three channels per joint, and the velocity channel is the one the
   HK gradient weights most heavily (§8b F2: 44 % of `|C|` mass).
2. **The decay-arrest is real but under-powered**, which is what a too-large dose looks like when
   it is simultaneously helping (persistent excitation) and hurting (drowning the signal the
   controller is trying to model).

### ★★ σ = 0.03 — the dose response is NON-MONOTONIC, and the body STANDS UP

| 40 k, n=4 | net_disp | steps | falls | tilt_sd |
|---|---|---|---|---|
| σ = 0 | **1.46 ± 1.95** | 240 | 1.75 | 0.208 |
| **σ = 0.03** | 0.88 ± 0.23 | 151 | **0.75** | 0.179 |
| σ = 0.10 | 0.45 ± 0.32 | 185 | 1.25 | 0.165 |

Transport still falls monotonically with σ, so **`REGRESSION` on transport stands.** But the
window split shows the aggregate was hiding the actual effect:

| chassis_y, per window | early 0–10k | mid 13–26k | late 27–40k |
|---|---|---|---|
| σ = 0 | 0.0424 | 0.0374 | **0.0341** ↓ |
| **σ = 0.03** | 0.0453 | 0.0550 | **0.0706** ↑ |
| σ = 0.10 | 0.0443 | 0.0346 | **0.0327** ↓ |

| step rate, late window | σ=0: **2.95** | σ=0.03: **2.99** | σ=0.10: **1.60** |
|---|---|---|---|
| path rate, late | 0.577 | **0.400** | 0.639 |
| displacement rate, late | 0.042 | **0.049** | 0.045 |

**At σ = 0.03 the body progressively STANDS UP over the run** — chassis 0.045 → 0.055 → **0.071**,
monotone, ending **above the deployed baseline's 0.058** — while every other arm sinks. And it
does so with **`height_homeo_gain = 0` and no stance-lift**: the pure-HK tier has no height
mechanism at all. It also holds the step rate that σ = 0.10 destroys (2.99 vs 1.60) while
spending the *least* path per unit displacement of the three.

**The dose response is non-monotonic — σ = 0.03 helps where σ = 0.10 hurts — which is the
stochastic-resonance shape**: an optimum exists and PM's value is past it for our body. That is
consistent with the state-vector argument above (our `[pos, action, delta]` layout doubles the
same σ into the velocity channel, so our effective dose is roughly twice PM's nominal one).

**Verdict: `REGRESSION` on transport at every σ tested; `WORKING` on posture at σ = 0.03**, where
it produces the one thing the whole pure-HK campaign could not — a body that gets *taller* as it
learns, unaided. Aggregate metrics called this `chassis_y 0.059 ± 0.026` and it read as noise;
only the window split showed it was a monotone rise.

**Re-use context: sweep σ ∈ [0.01, 0.05] where the optimum evidently lives, and inject on the
POSITION channel only so the same σ does not double into the velocity channel.**

### ⚠️ The position-only test ran, and it REFUTES the channel hypothesis — my reasoning was backwards

Built as `pos_noise_sigma`/`_tau` in `JointSensorimotorBridge`, injected **after** `delta` is
computed from clean positions, so the dose lands on `pos` alone. σ ∈ {0.01, 0.03, 0.05}, 40 k, n=4.

| arm | chassis_y early → mid → late | | steps/1k early → late |
|---|---|---|---|
| no noise | 0.0424 → 0.0374 → 0.0341 | sinks | 12.0 → 3.0 |
| **BODY-side σ = 0.03 (both channels)** | 0.0453 → 0.0550 → **0.0706** | **RISES** | 7.6 → 3.0 |
| POS-only σ = 0.01 | 0.0437 → 0.0366 → 0.0391 | sinks | 11.9 → 3.9 |
| POS-only σ = 0.03 | 0.0420 → 0.0439 → 0.0331 | sinks | 11.5 → 1.8 |
| POS-only σ = 0.05 | 0.0434 → 0.0401 → 0.0438 | flat | 12.2 → 1.8 |

**Position-only noise does not reproduce the posture rise at any σ.** Transport also still
declines monotonically (net_disp 1.46 → 1.13 → 0.87 → 0.71), and spin gets much worse
(`turns` ±15–20 vs ±12 unnoised).

**I predicted the opposite.** The re-use context above argued the velocity channel was where the
dose was doing damage, and that isolating position would recover the benefit. The measurement says
the reverse: **the posture rise REQUIRES the noise to reach the velocity channel.** Removing it
removes the effect.

**And that is the more sensible mechanism in hindsight** — the velocity/delta channel is the one
the HK gradient weights most heavily (44 % of `|C|` mass, §8b F2). Homeokinesis maximizes loop
sensitivity; a persistent perturbation on the channel it cares most about is exactly what it will
work hardest to become responsive to, and stiffening that loop is what lifts the body. Noise on
position alone perturbs a channel the controller has largely ignored.

**Revised verdict on I4:** `REGRESSION` on transport at every σ and both injection sites;
**`WORKING` on posture only for BOTH-channel noise at σ ≈ 0.03**, and the active ingredient is
the **velocity** component, not the position one. **Next test to close this cleanly: velocity-
channel-ONLY noise** — the exact complement, which the same bridge hook can deliver, and which
would confirm rather than infer the mechanism.

### ⚠️⚠️ The velocity-only test ALSO fails — so BOTH single-channel hypotheses were wrong

Built as `vel_noise_sigma` (noise added to `delta` after it is computed, position left clean).
**Dose scaling was computed, not guessed:** `delta` is a per-tick difference (~0.01–0.05), so the
body-side σ = 0.03 arm delivers an effective delta-noise std of only ≈ 0.0022 (an OU increment
with a = 1/8) — the equivalent velocity-channel dose is σ ≈ 0.015, not 0.03. Swept
{0.005, 0.015, 0.04} to bracket it. 40 k, n=4.

| arm | chassis_y early → mid → late | verdict |
|---|---|---|
| no noise | 0.0424 → 0.0374 → 0.0341 | sinks |
| **BODY-side, BOTH channels, σ = 0.03** | 0.0453 → 0.0550 → **0.0706** | **RISES** |
| POS-only σ = 0.03 | 0.0420 → 0.0439 → 0.0331 | sinks |
| VEL-only σ = 0.005 | 0.0406 → 0.0337 → 0.0432 | flat |
| VEL-only σ = 0.015 | 0.0440 → 0.0377 → 0.0418 | flat |
| VEL-only σ = 0.04 | 0.0422 → 0.0395 → 0.0324 | sinks |

**Neither channel alone reproduces the posture rise.** Velocity-only *arrests* the sink at low σ
(flat rather than falling — half the effect) but never produces the rise. The correct statement
is therefore neither of my two hypotheses:

> **The posture rise requires noise on BOTH channels SIMULTANEOUSLY, and is not attributable to
> either alone.**

### The diagnosis — and it is a real methodological lesson

Body-side noise is **physically coherent**: it perturbs the joint angle, and `delta` is then the
*true derivative of the perturbed angle*. Position error and velocity error are the same physical
event, correctly related. Channel-isolated noise is **physically incoherent** — a position wobble
with no matching velocity, or a velocity with no matching position. That is a perturbation no real
body can produce.

The HK loop learns a **forward model**. A coherent perturbation is *learnable*, and the controller
can stiffen the loop against it — which is what lifts the body. An incoherent one is unmodellable
corruption, so the model error never resolves and there is nothing to stiffen against.

⇒ **Sensor noise must be injected at the SENSOR, not at the derived feature vector, because the
feature vector's components are not independent.** `pos_noise_sigma` / `vel_noise_sigma` were a
mis-framing on my part; they earned their keep as the diagnostics that established this, and they
should stay default-off as diagnostics, but **the deployable form is the body-side one**
(`sensor_noise_sigma` in `picrawler_body.gd`).

**Secondary result worth keeping:** velocity-channel noise does something the other modes do not —
`step_bal` improves monotonically with σ (0.19 → 0.32 → 0.38) while **`turns` falls** (9.0 → 4.7 →
3.0). It reduces spin and evens out leg participation. And σ = 0.005 gives net_disp **1.87 ± 1.15**,
the best transport of any noise arm and nominally above the un-noised 1.46 (within variance).
That is a *different* benefit from the posture one and worth a proper look on its own.

---

## 8j. ★★★ THE ABLATION, POWERED TO n=20 — it is not a tie, it is a win for the ablation

| metric | baseline n=20 | **ALL learning off n=20** | Δ | t |
|---|---|---|---|---|
| **net_z** | 4.322 ± 0.672 | **4.915 ± 0.459** | **+0.593** | **+3.26** |
| **flat_v** | 0.044 ± 0.012 | **0.059 ± 0.011** | **+0.014** | **+3.80** |
| **straight** | 0.698 ± 0.057 | **0.737 ± 0.031** | +0.039 | +2.68 |
| steps | 55.45 ± 11.83 | 46.30 ± 7.55 | −9.15 | −2.92 |
| tilt_sd · planted · belly · falls | 0.072 · 3.60 · 0.023 · 0 | 0.066 · 3.69 · 0.022 · 0 | ties | — |

**At n=4 this read as "costs nothing". At n=20 it is an improvement.** Removing every learned
component makes the deployed gait travel further, straighter and faster — **with lower variance
on all three**. The learned layer buys `steps` (+20 %) and a little `step_bal`, and pays in
distance, straightness and consistency.

**★ Note what finally moved `flat_v`.** It sat at 0.03–0.05 across *nine* isolated timing levers
and a tenth load-gated stroke — the longest-standing open question in the ledger. The first
change to move it (0.044 → 0.059, t = 3.8) is **deleting the learned controller.** The eleventh
lever is a subtraction.

⇒ The verdict upgrades from `NULL` to a mild **`REGRESSION` for the learned layer on this base.**
On the deployed config the homeokinetic controller and the CPG-embedding are not inert — they are
slightly harmful to locomotion.

**Corroborated independently.** Raising `ctrl_lr` *on the deployed base* (n=6) does nothing good
and destabilises: 0.05 → net_z 4.20 ± 0.95; 0.10 → net_z 4.39 ± 0.77 but **tilt_sd 0.070 →
0.312 ± 0.504** (one seed diverged) and belly 0.023 → 0.045 ± 0.046. **More learning on the
deployed base is worse, in the same direction the ablation points** — the scaffold stack was
hand-tuned around a *weak* learned signal, so both strengthening and removing it move away from
that tuning; removal favourably, amplification not.

**⚠️ Do not over-generalise this.** It is a statement about the deployed base only. On the
`pure_hk` tier the same controller is the *only* thing producing motion at all (§8g–8h), and at
PM's learning rate it produces seeds inside the deployed config's distance range. The two results
together say something specific and useful:

> **The learned controller and the hand-built scaffold stack are two competing solutions to the
> same problem, and running them together is worse than running either alone.**

That is the sharpest framing this campaign has produced, and it is what the whole Playful Machine
comparison was pointing at from §2 onward: PM runs the learned solution with a *body* scaffold and
no control-layer scaffold; we run a control-layer scaffold with a learned solution fighting it.

---

## 8k. ★★ "MOVE THE SCAFFOLDS ONTO THE OBJECTIVE SOCKET" — REFUTED, and the reason is the useful part

**The proposal.** §8j concluded that the learned controller and the scaffold stack are competing
solutions. The suggested resolution: the competition exists because scaffolds are *additive terms
on the same output*, which is a rewrite-rule violation (`postural_gain·(x − x*)` added to the
command **is a behavior**). MotorEPM already has the correct path — the L-1b objective socket,
where a per-leg `PredictionToken` blends the target error **into the HK descent**
(`ξ̃ = (1−w)·ξ + w·(x − x*)`). Posture is **double-implemented**: it goes through the socket
*and* carries `postural_gain = 0.7` additively. So: delete the additive copy, keep the objective.

**Instrument fix required first (§3.2 rule 5).** `obj_active` / `obj_weight` existed **only in
`diag_snapshot()`** — the documented trap — so they read 0.0 in every headless run and "the
objective is driving" had never been verifiable from a seedavg arm. Mirrored into
`snapshot_state()`'s `mod` dict and the JSONL.

**Result (corridor, n=6, 6 000 ticks):**

| arm | net_z | straight | flat_v | chassis_y | tilt_sd | falls | steps | step_bal | **obj_w** |
|---|---|---|---|---|---|---|---|---|---|
| **CONTROL** — postural 0.7 additive | **4.49 ± 0.53** | **0.70** | **0.044** | **0.058** | **0.065** | **0** | 54.8 | **0.52** | 0.10 |
| **A** — postural 0, obj gain 0.3 | 1.51 ± 1.17 | 0.34 | 0.016 | 0.049 | 0.124 | 0.50 | **99.2** | 0.13 | **0.06** |
| **B** — postural 0, obj gain 0.7 | 1.93 ± 1.14 | 0.41 | 0.019 | 0.047 | 0.088 | 0.17 | 97.0 | 0.11 | 0.13 |

**Moving posture from additive to objective costs two-thirds of the distance**, halves
straightness, cuts flat speed to a third, and introduces falls. `REGRESSION`. The proposal is
refuted in this form.

### Why — and this is a real finding about the socket, not just a failed lever

The socket **fired correctly in every arm** (`obj_active = 1`, all 4 legs, every seed), so this is
not `DEAD_CODE`. The problem is the *weight*:

- Delivered weight is `w = gain · self_precision(bin)`. **At gain 0.3 the delivered `w` is 0.10;
  at gain 0.7 it is only 0.13.** The objective path is **~5× weaker than the additive term it
  replaces** even when its policy gain is more than doubled.
- **And in arm A the weight FELL when the additive term was removed — 0.10 → 0.06**, i.e.
  measured self-precision dropped from ≈0.33 to ≈0.20.

That second number is the mechanism:

> **The objective's authority is precision-gated, and its precision is downstream of the very
> postural stability the objective is supposed to produce.** Remove the additive term that was
> holding the body steady, the posture becomes inconsistent, self-precision falls, and the
> objective weakens *exactly when it is needed most.* A self-defeating loop.

**A precision-weighted objective can REFINE a posture something else is holding. It cannot
BOOTSTRAP one.** That is a design fact about this socket and it was invisible until the
instrument was mirrored.

### The secondary result, which points somewhere

**Removing the additive postural term nearly DOUBLED the step count** — 54.8 → 99.2 — while
`step_bal` collapsed 0.52 → 0.13. So `postural_gain` is doing **two** jobs: holding the body up
*and heavily damping the gait*. Releasing it releases a great deal of leg motion, but the motion
is concentrated in one or two legs and buys nothing. That is congruent with §8j (the scaffold
stack and the learned layer competing) and it says the postural term is a particularly costly
scaffold — it is *both* the thing keeping the robot upright and the thing suppressing its gait.

### What this means for the architecture question

**The uncomfortable conclusion: the scaffolds that are load-bearing for STABILITY cannot be
learned in from a body that is not yet stable.** That is a bootstrapping problem, and it is
exactly the problem PM solves with an external anti-fall operator — a temporary prop, removed
later. We rejected the prop (correctly, §6) and solved the same bootstrap with a *permanent
internal control term* instead. **Both are scaffolds. Ours is simply permanent and internal
rather than temporary and external**, which is arguably the worse of the two because it can never
be de-scaffolded.

⇒ **The genuinely doctrine-clean route is the one PM-legitimate scaffold class we have not
tried: MORPHOLOGY.** A body whose *passive* equilibrium is standing needs no postural term to
bootstrap from. The ledger's kinematic dead-end entry already points there — feet plant at 170 mm
against 166 mm total leg reach, so the limb is at its geometric limit and the shank must splay
just to reach the ground. **Fixing that is a body change, it is permanent rather than propped,
and it is the only path measured so far that removes the bootstrap instead of hiding it.**

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
