# The `feet_y` oracle, the sensor-legitimacy audit, and the IMU/IK refactor

**Status:** design note, 2026-07-25. Documents a live Markov-blanket violation in the
deployed picrawler stack, audits every sensor the body publishes, and lays out the options
for replacing the oracle with something a physical robot could actually have.

> **One-line summary — RESOLVED 2026-07-25.** The swing detector that gates `stance_lift`
> and every Cruse rule read **absolute world-Y of the foot**. Three Markov-compliant
> replacements lost badly and two phase hypotheses died at chance agreement; the property
> they all lacked was a **gravity reference**, which is exactly what an accelerometer
> provides. **`feet_y_gravity` (encoder-FK foot position · accelerometer gravity-up = IK ⊕
> IMU) replaces the oracle and BEATS it**: net_z 3.76→3.95, flat_v 0.04→**0.05** (the first
> flat-speed movement across 10+ levers), step_bal 0.25→0.42, hump gate 5.21→5.35, 0 falls.
> Costs straightness (0.70→0.62) and wobble (0.067→0.095). See §5 for the remaining
> **sim-to-real gaps**, which are not yet tested.

---

## 1. The oracle

`reality.proprio.feet_y` is published as:

```gdscript
feet_y_arr.append(_lowers[i].global_transform.origin.y - L3 * 0.5)
```

`_lowers[i].global_transform.origin.y` is the **world-space Y coordinate** of the lower-leg
rigid body. No physical picrawler can sense this. It is the same class of violation that
retired `chassis_y_norm` (CLAUDE.md §5.3: *"a god's-eye quantity is not an observation"*).

**What consumes it.** `MotorEPM::feet_topic` → `update_cruse_state()` → `in_swing_[]`, which
gates:
- `stance_lift_gain` (the belly-up knee tuck — a promoted lever)
- `cruse_gain` (Walknet rules 1/2/3)
- `cruse_rule5_gain` (load distribution)

**How much the detector launders it.** The test is `foot_y > foot_y_ema` — a *difference* of
world-Y values ~50 ticks apart. On flat ground with a steady chassis that difference ≈ the
foot's motion relative to the body, which is legitimate proprioception. But the body's own
vertical drift stays in, so it is contaminated exactly where terrain matters. Partial
laundering, failing in the worst place.

**A second, worse use.** The body's own "planted" test, `feet_y < stance_y_threshold`, is
**doubly** god's-eye: absolute world height against a fixed constant, with no terrain or
chassis reference. It is not used by the control loop, but it *was* used as ground truth in
analysis — and it under-reports swing by ~16× versus the true contact sensor. **A proxy
cannot be the control for the proxy it replaces.**

### 1.1 The replacements, and why this is still open

Seed-averaged n=4, corridor @ difficulty 0.3, identical in every respect but the swing input:

| swing-detector input | legal? | net_z | straight | tilt_sd |
|---|---|---|---|---|
| `feet_y` — absolute world-Y | **✗** | **3.76 ± 0.40** | **0.70** | **0.067** |
| `feet_y_body` — foot pose vs chassis (encoder FK) | ✓ | 2.52 ± 1.34 | 0.50 | 0.095 |
| `feet_y_ground` — FK + belly ToF (terrain-relative) | ✓ | 2.26 ± 1.65 | 0.46 | 0.101 |
| `foot_contact` — true physics touch flag | ✓ | 2.37 ± 0.86 | 0.46 | 0.088 |

`feet_y_body` is the clean experiment: **identical formula, identical toe approximation, only
the reference frame changed.** The delta isolates the god's-eye component and nothing else.

**Best current reading — NOT verified.** World-Y minus body-relative equals exactly the
chassis's own vertical motion. So what the illegal signal carries, and the legal twin loses,
is the **body's bounce**: a whole-body vertical phase reference. Gating a knee push on that
synchronises the push with when the legs are actually being loaded. Supporting evidence: the
detector's duty is 0.408, close to the ~0.5 a mean-crossing test structurally produces, while
true contact is 0.229 — i.e. the "detector" behaves like a *phase* indicator, not a contact
indicator. `feet_y_ground` was built to reconstruct the bounce legally via the belly ToF and
**did not recover it**, which either falsifies the reading or reflects the ToF being a noisy,
single-point, short-range stand-in for true chassis height.

**Also measured:** wiring the *true* contact sensor makes the gait worse, and a compensating
`stance_lift` sweep {0.3, 0.35, 0.4} does not rescue it (2.53 / 2.10 / 1.95 vs 3.76). **The
more accurate sensor is the worse input** — because the consumer wants gait phase, not
ground truth.

---

## 2. Sensor-legitimacy audit — everything the body publishes

Classified by what a physical picrawler could actually measure.

### Legal — a real robot has these

| Topic | Physical sensor | Notes |
|---|---|---|
| `joints` | joint encoders | 12 floats, normalized hip1/hip2/knee × 4 |
| **`joint_torque`** | **servo current sensing** | **12 floats. A REAL LOAD SIGNAL — and MotorEPM has never consumed it** (see §3) |
| `foot_contact` | foot switches | binary per leg; ground truth for contact |
| `ground_clearance` | belly ToF / ultrasonic | already replaced god's-eye height; solved the hump |
| `tilt` | accelerometer gravity vector | pitch/roll as sin/cos |
| `upright` | accelerometer | chassis up-vector vs gravity |
| `compass` | integrated gyro (dead-reckoned yaw) | drifts, but own-yaw integration is accepted as Markov-compliant — it is what the heading-hold PD rides on |
| `feet_y_body` | encoder FK + link lengths | added 2026-07-25 |
| `feet_y_ground` | encoder FK + belly ToF | added 2026-07-25; terrain-relative |

### Oracle — god's-eye, not physically sensible

| Topic | Why illegal | Status |
|---|---|---|
| **`feet_y`** | absolute world-Y of the foot | **LIVE in the deployed stack** (§1) |
| `chassis_y_norm` | absolute world-Y of the chassis | retired from use; still published |
| `target_compass`, `radial_compass` | ground-truth bearing/vector to a target | already the plan's declared disqualifier (CLAUDE.md §5.2); nav layer not yet built |
| `vision_compass` | depends on the vision path | separate audit needed |

### Soft oracle — sensible only with machinery we have not built

| Topic | Why | Notes |
|---|---|---|
| `fwd_v`, `lateral_v` | `_chassis.linear_velocity` projected on heading — **world-frame velocity** | A real robot needs odometry or optical flow to know its ground speed. Weaker than a position oracle, but not free. **Consumed by `progress_commit`, `stuck_explore`, `forward_flow`, and (until 2026-07-25) the coordination search's reward.** Worth its own pass |

### What a real IMU gives that we do NOT publish

`reality.proprio.imu` = `[sin(yaw), cos(yaw), fwd_v, ang_v]` — heading, horizontal speed,
**yaw rate only**. A 6-axis IMU also provides:

- **vertical linear acceleration** ← *exactly the body-bounce component §1.1 identifies as load-bearing*
- lateral / fore-aft linear acceleration
- **pitch and roll angular rates** (we publish pitch/roll *angles* via `tilt`, never their rates)

**This is the gap.** The richest, cheapest, most physically honest channel on the robot is
almost entirely unpublished.

---

## 3. Design options for the refactor

### The key insight about accelerometers

For a **phase** reference you never integrate. A band-passed vertical accelerometer gives the
bounce oscillation with correct phase and **no drift**. Accelerometers are poor at position
and excellent at oscillation phase — which is precisely what §1.1 says the gate needs. The
usual objection to accelerometers (drift) does not apply to the thing we actually want.

### ✗ RESULT — the "it's really a phase gate" hypothesis is FALSIFIED

Measured before building anything (`phase_agree` / `legphase_agree` diagnostics in MotorEPM,
n=3 seeds, 6000 ticks): agreement between a legal phase gate and the oracle detector's actual
per-tick output.

| candidate phase reference | legal? | agreement with the oracle detector |
|---|---|---|
| **global** body/CPG phase, `sin(φ_body + gait_phase[leg])` | ✓ | **0.510 / 0.497 / 0.491** |
| **per-leg** joint phase, `cos(L.phase)` | ✓ | **0.525 / 0.499 / 0.559** |

**Both are chance.** Neither the whole-body rhythm nor each leg's own joint-derived oscillation
carries information about what the world-Y detector outputs. Option A is dead as specified, and
the interpretation in §1.1 — that the detector "is really a phase gate" — **does not survive its
own test.** Recorded rather than quietly dropped: it was a plausible reading of the duty-cycle
numbers, and it was wrong.

### → What the negative result localizes

Everything tried so far and failed shares one deficiency: **none of them is gravity-referenced.**

- `feet_y_body` — foot height in the **chassis** frame. Rotation-invariant in the body, but a
  foot at a fixed body-frame position sits at different *gravitational* heights as the chassis
  pitches and rolls.
- `feet_y_ground` — adds body height via the belly ToF, but the ToF points along the **chassis**
  down-axis, so it inherits the same attitude error.
- `foot_contact`, phase gates — no height reference at all.

`feet_y` (world-Y) is **gravity-aligned by construction.** That is the one property all four
failed candidates lack, and it is the remaining explanation for why it is load-bearing.

**And gravity alignment is exactly what an accelerometer measures.** The legal reconstruction is
therefore *IMU attitude ⊕ IK*:

> **`feet_y_gravity` = FK foot position from joint encoders, rotated into a
> gravity-aligned frame using the chassis attitude from the accelerometer.**

Both halves are already available — `tilt` publishes pitch/roll from the gravity vector, and
`feet_y_body` is the FK. This is the first candidate that shares world-Y's defining property
while remaining physically realizable, and it is the next thing to build.

### Option A — gate on the body's own gait phase (cheapest, no new sensor) — ✗ REFUTED ABOVE

`BodyRhythmTracker` **already publishes** `rhythm.body.gait` = (cos φ, sin φ, ω), a
proprioceptive collective coordinate, and it is already live in the deployed config. Gate the
knee on `sin(φ_body + gait_phase[leg]) > 0` instead of on a height threshold.

- Fully legal, zero new plumbing, directly tests the phase hypothesis.
- Predicted by the standing Cruse re-use context, which says its foot-height detector should
  be *replaced by the emergent gait's own phase*.
- **If the phase reading is right, this should match or beat the oracle.** If it does not,
  the reading is wrong and the bounce is doing something else.

### Option B — publish the vertical IMU channel

Add vertical (and lateral) linear acceleration plus pitch/roll rates to the IMU token. Derive
a band-passed bounce phase. Targets the identified missing component directly, and is a
strictly-better sensor surface regardless of what wins here.

### Option C — consume `joint_torque` (the load signal we already have)

Servo torque is a genuine per-joint load measurement and **nothing in MotorEPM reads it.**
This is the signal Walknet's rules were always about; every Cruse refutation to date gated on
a foot-height proxy instead. It also enables the doctrinally-preferred framing below.

### Option D — FK foot height + IMU bounce phase

Structurally reconstruct what world-Y is (leg motion + body motion) from two legal sources.
Only worth building after A/B/C establish which component actually matters.

### The reframe worth taking seriously

**A gate is a schedule, and §1 says implement the error, not the behavior.** Every option
above still asks "when may this leg lift?" The doctrinally-preferred alternative is to give
the knee an *objective* — for example, regulate per-leg **load** toward even distribution —
and let stance and swing emerge from descending that error. `joint_torque` is what makes this
possible for the first time. It is also the honest reading of Cruse: its rules are load
rules, and they have never once had a load signal to gate on.

---

## 4. Proposed order (revised after the phase result)

1. **`feet_y_gravity` — IMU attitude ⊕ IK.** FK foot position rotated into a gravity-aligned
   frame using accelerometer-derived chassis attitude. The only untried candidate that shares
   world-Y's defining property. Cheap: both inputs already exist.
   *Falsifiable prediction:* if gravity reference is what matters, this recovers ~net_z 3.7.
   If it lands at ~2.4 like the others, the oracle is carrying something else again and the
   next suspect is absolute-vs-relative *scale* rather than orientation.
2. **C — `joint_torque` as a load observation.** First as an instrument: does servo torque
   separate stance from swing cleanly (measure agreement against `foot_contact` the same way
   the phase hypothesis was tested — cheap, and it settles the question before any build)?
   Then as an *objective* per the reframe below.
3. **B — publish the vertical IMU channel** (vertical/lateral acceleration, pitch/roll rates).
   Worth doing regardless of this refactor: a strictly better sensor surface, and the honest
   basis for any body-bounce signal.
4. **D — fusion**, only once 1–3 say which component actually matters.

**Method note earned the hard way:** every hypothesis here should be tested by a *cheap
agreement diagnostic against the incumbent signal* before a replacement is built and
seed-averaged. Two hypotheses died at ~0.5 agreement for the cost of one build each. Do that
first, every time.

Throughout: gain-0-guarded, one lever at a time, n≥4 seed-avg, judged on the full metric set
plus the hump gate (obstacle performance must not regress — it is currently good) and the
recovery gate for anything always-on.

**Until this is resolved, any "the gait works" claim must carry the caveat that its swing
gate is not physically realizable.**

---

## 5. Sim-to-real: what the physical PiCrawler actually has

Operator constraints (2026-07-25): the port has **an IMU** (a second could be added) and
**hobby servos** — angle-commanded, with **current draw as the only torque surrogate**.

That is a tighter constraint than "Markov-compliant," and it is the better bar. A signal is
only real if it is expressible in:

> **{ commanded joint angles, IMU 6-axis (accel + gyro), servo current, known link geometry }**

### 5.1 What this immediately rules in and out

| Signal | On hardware? |
|---|---|
| `feet_y_gravity` (the new promoted signal) | **Yes** — with two caveats below |
| `joint_torque` | **As current draw**, a noisy monotone surrogate (§5.3) |
| `foot_contact` | **NO — the robot has no foot switches.** Sim gives us more than hardware does. Fortunate that it lost anyway; adding FSRs or microswitches per foot is cheap if we ever want it |
| `joints` as *measured* encoder angles | **NO — hobby servos do not report position** (§5.2) |
| `feet_y` (the oracle) | Never |

### 5.2 The commanded-vs-achieved gap — the important one

Hobby servos accept an angle and report nothing. So on hardware, FK is computed from
**commanded** angles, not achieved ones. Our sim `feet_y_gravity` reads the *achieved*
rigid-body pose, so **the sim signal is strictly better-informed than the hardware signal.**
This is a sim-to-real gap in the dangerous direction.

It matters most exactly where the gate matters: a **planted** leg deflects under load, so its
commanded angle says "I am here" while the foot is actually higher. Commanded-angle FK reports
the *intended* foot trajectory, not the real one — cleaner (no noise) but blind to load.

**This is directly testable in sim, and should be the next experiment:** publish
`feet_y_gravity_cmd`, computed identically but from the commanded angles the brain issued, and
A/B it. If it holds up, the promoted result transfers. If it collapses, the win depends on
information the hardware does not have — which we would want to know now, not after a port.

### 5.3 Servo current as the torque surrogate

Genuinely the right instinct, with practical caveats worth designing around:

- **Per-channel vs bus.** A Robot HAT typically exposes *bus* current, not per-servo.
  Per-leg sensing needs shunts (e.g. INA3221 = 3 channels, so four of them for 12 servos, or
  a multiplexed shunt). **Bus current alone is still valuable**: total current ≈ total support
  effort, and its *oscillation* is a legitimate whole-body load rhythm — arguably another route
  to the body-bounce signal, from a completely independent channel.
- **Current conflates load with tracking error.** A hobby servo draws current both when
  bearing weight and when fighting a position error it cannot reach. For stance detection that
  conflation is mostly benign (both mean "this leg is working"); for a clean load reading it is
  not.
- **It is a noisy monotone function of torque**, not torque: stall behaviour, back-EMF with
  velocity, temperature drift, PWM chop. Needs low-pass filtering; ~50 Hz brain tick is easy
  for I²C sensing.
- **Useful asymmetry:** an unloaded servo at its target draws very little. So current has good
  contrast between free-swinging and weight-bearing — which is exactly the distinction Walknet's
  load rules want.
- **Honest sim modelling:** our `joint_torque` is *actual applied torque*. To be predictive of
  hardware it should be modelled as `current ≈ k·|torque| + noise + deadband`, and if only bus
  current is available, as the **sum across joints**. Testing the load rules on an idealized
  torque signal would be a weakened-slice result in the opposite direction from usual — too
  good, not too weak.

### 5.4 The attitude gap

`feet_y_gravity` needs gravity-up in the body frame. Our sim uses the *exact* attitude. A real
accelerometer measures **gravity + body linear acceleration**, so during a bouncy gait "down"
wobbles in step with the bounce. Standard fix is gyro/accel fusion (complementary or Madgwick);
the IMU has the gyro, so this is routine — but it is another place the sim is easier than
reality, and it should be modelled before claiming transfer.

### 5.5 Where a second IMU would actually pay

Not a second body IMU. The hobby-servo constraint denies us *per-leg* position feedback, so
the high-value placement is **on a leg segment**: a leg-mounted IMU gives that leg's own
acceleration and attitude directly, recovering per-leg motion without encoders. That is a
targeted fix for the exact gap in §5.2. A belly-mounted second IMU (differential body motion)
is much less useful by comparison.

### 5.6 ✓ RESULT — the hardware's POORER information is the BETTER signal

`feet_y_gravity_cmd` (FK from **commanded** servo angles) was built and measured. It does not
merely survive — **it is the best arm of the campaign.**

| swing input | legal? | hardware? | net_z | straight | tilt_sd | hump final_z |
|---|---|---|---|---|---|---|
| `feet_y` — world-Y oracle | ✗ | ✗ | 3.76 ± 0.40 | 0.70 | **0.067** | 5.21 ± 0.67 |
| `feet_y_gravity` — FK from **achieved** pose | ✓ | ✗ (needs encoders) | 3.95 ± 0.56 | 0.62 | 0.095 | 5.35 ± 1.15 |
| **`feet_y_gravity_cmd` — FK from commanded angles** | ✓ | **✓** | **4.36 ± 0.28** | **0.74 ± 0.01** | 0.084 | **6.10 ± 0.46** |

**Validation first, before trusting any of it:** the measured-angle FK reproduces the
achieved-pose signal to **1.1 mm**, so the FK chain is correctly wired; the
**commanded-vs-achieved gap is 22 mm mean / 38 mm max** at the foot — genuine servo deflection
under load, 20× the FK error, and comparable to the robot's entire belly clearance.

**Why the worse sensor wins:** load deflection is *noise* from the gate's point of view. The
commanded angles are the clean **intended** foot trajectory. Removing the deflection makes the
gate more consistent, which shows up as extraordinarily tight straightness (± 0.01 across
seeds, every seed 0.72–0.75) and halved distance variance.

**Costs vs the achieved twin:** `flat_v` 0.05 → 0.04 and steps 113 → 50. The achieved signal's
extra liveliness came from deflection noise — more steps, less productive ones. Commanded
covers more ground with a third fewer steps.

**Read this as a sim-to-real result with an unusual sign:** the usual worry is that sim
privilege inflates results. Here the sim-privileged signal was *worse*, and the constraint the
hardware imposes turned out to be a filter we wanted. Do not generalize it — but do check the
direction rather than assuming it.

### 5.7 Remaining next steps

1. ~~`feet_y_gravity_cmd`~~ — **DONE, promoted (§5.6).**
2. **Model the accelerometer honestly** — gravity + linear-acceleration contamination, then
   gyro fusion. Second sim-to-real gap on the same signal.
3. **Publish the vertical IMU channel** (vertical/lateral acceleration, pitch/roll rates) —
   absent today, physically free, and the honest basis for any bounce signal.
4. **`joint_torque` as a current surrogate** (noise + deadband; bus-sum variant too), then as
   an *objective* rather than a gate — the load rules' first real load signal.

---

## 6. Robustness matrix — the substrate is NOT sensor-agnostic, but the GRAVITY FAMILY is

`scripts_tools/robustavg.py`, 8 sensor references × 2 actuation backends
(`OGMA_PICRAWLER_JOINT_BACKEND` ∈ {hinge, g6dof}), n=3 seeds/cell, 6000 ticks, diff 0.3.
Emergence criterion fixed **before** the runs and deliberately low — the question is
whether a walk *exists*, not whether it is fast: `net_z > 1.0 AND straight > 0.4 AND
falls == 0`, required on **every** seed.

| sensor reference | legal | hinge | g6dof |
|---|---|---|---|
| world-Y (god's-eye) | ✗ | **YES** 3.70 | no — 1 fall (3.59) |
| body-frame FK | ✓ | no (2.33) | no — 3 falls |
| FK + belly ToF | ✓ | no (2.22) | no (1.98) |
| true foot contact | ✓ | **YES** 3.90 | no — 1 fall |
| gravity, achieved FK | ✓ | **YES** 3.77 | **YES** 4.40 |
| gravity, commanded FK | ✓ | **YES** 4.40 | **YES** 3.70 |
| gravity, accel-only IMU | ✓ | **YES** 4.04 | **YES** 4.07 |
| gravity, fused IMU | ✓ | **YES** 4.74 | **YES** 3.07 |

**Emergence rate 10/16 (62 %). net_z spread 1.98 … 4.74 (2.4×).**

Three findings, in order of importance:

1. **Every gravity-referenced signal emerges in BOTH backends — 8/8 cells.** No other
   family does. Robustness here is a property of *the signal family*, not of the substrate
   alone: a gravity reference is the invariant, and it holds across an actuation backend the
   gait was never tuned for. **That is the thing to preserve when porting.**
2. **The god's-eye oracle is LESS robust than its hardware-honest replacement** — it fails
   on g6dof (a fall), while all four gravity variants pass both. So the illegal signal was
   not merely unnecessary, it was *more brittle*. Same for the true-contact sensor.
3. **Robust in kind, sensitive in degree.** 62 % emergence at a low bar means the substrate
   is *not* sensor-agnostic — six cells fail. But within the gravity family the failure mode
   is never "falls over", only "walks less far" (3.07 … 4.74, still a 1.5× spread).

**Power caveat:** n=3 with an all-seeds criterion is strict — a single unlucky seed flips a
cell, and some "no" cells have healthy means (body-frame FK at 2.33 mean failed on one seed).
The defensible claim is the **8/8 gravity family**, not the exact 62 %. One gym, one
difficulty. A finding-grade version wants n≥20 and varied terrain.

**Honest reading of the operator's thesis:** the architecture is not indifferent to what it
senses through — it needs a gravity reference. Given one, it is robust to the reference
frame, to servo deflection, to 21° of attitude error, and to the actuation backend. That is
a narrower but much more useful claim than "sensor agnostic", because it names the invariant.
