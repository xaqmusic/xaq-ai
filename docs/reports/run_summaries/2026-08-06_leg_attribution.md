# Which leg is responsible for a forward-velocity pulse?

**2026-08-06 · arena · seed 1 · n=1 · warmup 1000 ticks · `leg_attribution.py`**

Status: **n=6 seed-averaged, arena, 10k ticks, warmup 1000.** The engine identity
is unanimous (6/6), so it is a loud structural result rather than a marginal one —
but it is still a *signal* under §3 until it survives varied world seeds and a (d)
perturbation. ⚠ The kinematic metric in the first version of this report FAILED
validation and its ranking is retracted below; the force-based metric replaces it. Enough to license the UI work and
to direct the next lever; not enough to state as fact.

Instrument: per-tick trace from `picrawler_body.gd` (`OGMA_PICRAWLER_TRACE`,
off by default). Analysis: `godot_host/project/scripts_tools/leg_attribution.py`.

⚠ **God's-eye instrument.** `fwd_v` and foot contact are not egocentric — legal
for a diagnostic, illegal for control (§5.3). Nothing here may be fed to the
brain. Its egocentric counterpart is the new `reality.proprio.joint_load`.

---

## Why this is kinematic and not "motor current"

The intuitive metric is mechanical power `τ·ω` per joint. It does not work here,
for two independent reasons found in this order:

1. **`reality.proprio.joint_torque` is not applied torque.** It is the PD value
   used to set the motor's impulse *cap* (the code calls `_powered_torque` "the
   telemetry path"), and with `Kp=20 / Kd=8` against ω up to 6 rad/s it is
   dominated by its `−Kd·ω` damping term. Measured `corr(τ, Δθ) = −0.46…−0.56`
   on all three joints: it is essentially a negated velocity copy. Anything
   load-gating on it (Cruse Rule 5, a future `epm_joint_torque`) would have been
   gating on −ω. Replaced by `joint_load`, whose `corr(load, Δθ)` is −0.066 /
   −0.059 on hip1 / knee.
2. **`load × ω` is not power either.** A velocity-tracking deficit is
   anti-correlated with ω by construction, so the product is negative almost by
   definition. The sign control caught both errors: swing power came out
   negative, when a motor-driven swinging leg must receive positive work.

**Propulsion is kinematic** — a *stance* leg sweeping its foot backward drives
the body forward — and needs neither torque nor any assumption about delivered
force. Per-leg hip1 signs are *derived from the data*, and return
`{fl:−1, fr:+1, rl:−1, rr:+1}` — a left/right mirror, which the geometry
requires. That is a validity check the torque approach never passed, and the
script now fails loudly if the derivation is not a mirror.

## Result — from the FORCE metric (the kinematic one failed validation)

**Ground reaction impulse per foot, projected on the body-forward axis.** This is
literally the propulsion each leg delivers, and it is the only metric here that
survived the sufficient test.

| leg | mean forward GRF | share of \|forward GRF\| | sign |
|---|---|---|---|
| fl | −0.00176 | 25 % | braking |
| fr | −0.00034 | 5 % | braking |
| rl | −0.00050 | 7 % | braking |
| **rr** | **+0.00446** | **63 %** | **propelling** |

**`rr` delivers ~63 % of the forward ground force and is the only leg with a net
propulsive sign; the other three are net braking.** Summed, the body is net
forward (+0.00186), consistent with it making progress.

This is consistent with the ledger's standing description of `rr` as the
under-plant leg of a tripod-skid that is load-bearing — but note it makes `rr`
the *propulsor*, not merely a stabiliser.

### Seed-averaged — n=6, and the engine never changes

| leg | mean forward GRF (n=6) | propelling on |
|---|---|---|
| fl | **−0.00238 ± 0.00113** | **0 / 6 seeds** |
| fr | +0.00016 ± 0.00078 | 3 / 6 |
| rl | −0.00028 ± 0.00061 | 1 / 6 |
| **rr** | **+0.00410 ± 0.00113** | **6 / 6 seeds** |

Per-seed engine (most positive forward GRF): **rr, rr, rr, rr, rr, rr**. Its share
of |forward GRF| ranges 42–61 %. Mean `fwd_v` is +0.042…+0.053 on every seed, so
the body genuinely makes progress throughout.

**The power leg does NOT change per seed — it is `rr` every time.** `rr`'s mean is
3.6σ from zero and `fl`'s is −2.1σ; the two middle legs straddle zero. So the
picture is not "an asymmetric gait" in the vague sense: it is **one engine (`rr`),
one consistent brake (`fl`), and two roughly neutral legs**. `rr` and `fl` are a
DIAGONAL pair, and the config drives them at the SAME gait phase (`gait_phase =
[0, π, π, 0]`) — the same command producing opposite force contributions.

Number of net-propelling legs per seed: **1, 2, 3, 1, 1, 2**. The body is
locomoting on roughly one leg.

This is the physical content behind the ledger's long-standing "RR-under-plant
tripod-skid", and behind "every symmetry-forcing lever → circling": the asymmetry
is not incidental, it is *how this gait produces thrust at all*.

### ⚠ The kinematic metric said the OPPOSITE, and was wrong

An earlier version of this report (commit `d087df2`) ranked legs by stance-gated
hip1 sweep and reported `fl` best (+0.424) and `rr` worst (+0.060). **That
ranking is retracted.** The force metric inverts it, and the reason is now clear:
when the body moves forward, a planted foot's leg is necessarily swept backward
relative to the chassis, so stance sweep correlates with `fwd_v` whether the leg
is *driving* the body or being *dragged* by it. Stance kinematics cannot see the
direction of causation.

## Validation

**The decisive test is a per-foot friction ablation** (`OGMA_PICRAWLER_SLICK_LEG`
/ `_AT`, added for this): drop one foot to μ=0.05 so the leg still sweeps
normally but cannot transmit thrust. A propulsion metric must lose that leg's
share; a movement metric will not notice.

| metric | FL share before → after slick | FL sweep | verdict |
|---|---|---|---|
| stance-gated hip1 sweep | 0.230 → 0.230 (**−0 %**) | +2 % | **FAILED** |
| **forward ground reaction impulse** | **25 % → 4 %** | +2 % | **PASSED** |

Why the earlier controls missed it — both were degenerate against this failure
mode. A swing leg does not sweep *in stance*, so the swing control is tautological
under a stance gate; a lesioned leg does not move at all, so its raw signal
collapses alongside its share. **Only a perturbation that preserves the motion and
removes the force can separate propulsion from kinematics.**

Prior (weaker) evidence, retained for the record: stance-gated mean |r| 0.208 vs
swing-gated 0.093 vs shuffled-pulse null 0.067; lesion of FL (commands ×0) drove
its stance-sweep share −91.1 %. Both are consistent with the kinematic metric
tracking *motion*, which is what it turned out to measure.

## Side findings

- **The velocity channel carries no stride phase**, in *both* gyms. `delta`
  autocorrelation 0.79 → ~0.05 by lag 5, flat at lags 20/50/70; peak |AC| over
  lags 10–200 is 0.18–0.25 and sits at **lag 10–13, not a stride lag of 50–70**.
  The lag-1 value is the servo's 30 ms torque-rise. This is the same fact as
  `step_cv` = 0.98, from an independent angle, and it explains the commissioning
  regression mechanically.
- **`hip2` is close to a passive element** in the deployed gait: commanded range
  ±0.5 vs hip1's ±1.4, `corr(target−angle, Δθ) ≈ −0.08` against +0.63 / +0.70 for
  hip1 / knee. Its motion is externally driven, so it has no usable load signal
  and its velocity channel is noise at every lag. This does **not** retroactively
  invalidate the refuted hip2 levers — those commanded it harder than the
  deployed gait does.

## Next

1. Per-foot friction ablation, for the sufficient version of the validation.
2. Seed-average before any claim is promoted from signal to finding.
3. The UI colouring is licensed by these controls — colour by dominant leg with
   **saturation ∝ the margin**, so a near-tie reads as a near-tie.

---

# Addendum — the diagonal rhythm, and why `gait_phase` cannot fix it

**Operator, from crawling on the floor:** *"two diagonal legs with the exact same
phase does not work for a quadruped. The diagonal pair cannot move together, the
front legs must lead the back leg slightly or the stance becomes unstable. That's
why horses make the clip-clop clip-clop sound, not clop clop clop."*

Correct, and the rhythm is measurably absent. But the knob that looks like the fix
turns out to have no authority.

## 1. The rhythm is absent (n=6, arena, stride ≈ 37 ticks)

| pair | median offset | IQR | front leads | IQR if legs were INDEPENDENT |
|---|---|---|---|---|
| fl/rr | +2 ticks | −3..+6 | 64 % | 18 |
| fr/rl | +1 tick | −4..+6 | 53 % | 20 |

The pair *is* coupled — IQR 9–10 against 18–20 for an independent null — but
coupled at **near-zero lag**. A 1–2 tick offset is 3–5 % of stride, the sign flips
about as often as not (`fr/rl` front-leads 53 %, a coin flip), and ~30 % of
landings fall within ±2 ticks. That is the distribution of **"clop"**. A correct
trot would be a *tight* distribution centred on a consistent lead.

## 2. ★ But `gait_phase` does not control footfall timing

The config seeds `gait_phase = [0, π, π, 0]` — the diagonal pair commanded
simultaneously, exactly the unstable pattern. **The runtime phase search has
already moved it, far, and differently on every seed:**

```
start   [0.0,  3.113,  3.108, -0.006]     (the config seed)
seed1   [0.0, -2.626,  1.939, -0.912]     rr-fl =  -5.4 ticks
seed3   [0.0,  1.903,  1.640, -2.577]     rr-fl = -15.2 ticks
```

Commanded diagonal offset across the 12 pair-seeds spans **−15.2 … +8.6 ticks**
(~65 % of a stride). The measured footfall offset spans **−1.0 … +3.0 ticks**
(~11 %).

**The command varies 6× more than the timing it supposedly sets.** Whatever
determines when a foot lands, it is not `gait_phase`.

### Why — and it is mechanical, not mysterious

`gait_phase` modulates **hip1**, the fore/aft swing joint. Touchdown is a
**vertical** event, governed by knee extension, femur lift, and body height. And
this body has almost no authority over the vertical: **`hip2` (the lift joint) is
commanded over ±0.5 against hip1's ±1.4, and does not track its command at all**
(`corr(target−angle, Δθ) ≈ −0.08`, against +0.63 / +0.70 for hip1 / knee — see
Side findings). The legs are swung fore/aft under command and dropped
uncontrolled.

## 3. What this invalidates

This is the **premise of the whole timing-lever family**. The stroke-to-step lock
that "never entrained" (`td_plv` 0.04–0.10 at every loop gain), `step_cv` ≈ 0.98,
the nine timing levers whose premise was already known to be broken, and the
absent stride autocorrelation measured earlier today — all of them act through, or
are read from, a channel with no authority over footfall timing.

⚠ **So the fix is NOT to inject a diagonal offset into `gait_phase`.** That is
prohibition 7 (don't inject a rhythm) *and* it would be inert: the command already
takes wildly different values per seed and produces the same near-zero-lag
landings. **The lever, if there is one, is on the VERTICAL channel** — the body
cannot presently choose when to put a foot down.

---

# Addendum 2 — joint roles: the knee is doing hip2's job, from a bad angle

**Operator's model:** *"hip1 acting to swing the leg, hip2 elevates the chassis,
and the knee (when it works) plants ... the servos are plenty powerful and have
good individual authority but coordination is required."*

Checked against 6 seeds / 6831 touchdowns. Two thirds of the model confirmed, one
third is not happening, and the authority read is confirmed by measurement.

## 1. The knee plants — clearly

Touchdown-triggered average velocity (×10⁻³ rad/tick), pooled per leg. The knee
extends toward the ground, then **reverses hard at contact**:

| leg | joint | w=−10 | w=−4 | **w=0** | w=+4 |
|---|---|---|---|---|---|
| fl | knee | −9.9 | −8.7 | **+18.4** | +22.2 |
| fr | knee | −7.4 | −23.4 | **+2.0** | +19.6 |
| rl | knee | −14.2 | +4.2 | **+24.2** | +16.0 |
| rr | knee | −5.8 | +12.9 | **+20.7** | +10.9 |
| fl | **hip2** | +2.1 | +2.1 | **−4.0** | −1.9 |
| rl | **hip2** | +0.1 | +0.0 | **−2.4** | −0.3 |

The knee's signature is **5–10× larger than hip2's** at every leg. hip1 is
comparable in magnitude but its reversal is not locked to contact the way the
knee's is. **Knee = plant is confirmed.**

## 2. hip2 is NOT elevating

| | hip1 | hip2 | knee |
|---|---|---|---|
| rms velocity | 0.055–0.072 | **0.028** | 0.062–0.066 |
| range occupied (p5–p95) | 1.00–1.28 rad | **0.23–0.27 rad** | 0.93–1.07 rad |
| where it sits | ±0.6 | **−0.20 … +0.05** | −1.36 … −0.44 |

hip2 traverses **one quarter** the angular range of the other two joints and
lives in a narrow band biased negative. It is not idle — 43–45 % of the knee's
rms velocity — but it is not doing a chassis-elevation stroke.

## 3. ★ It is a COMMAND problem, not an authority problem

The operator's read is confirmed by measurement:

| leg | hip1 achieved/commanded | hip2 | knee |
|---|---|---|---|
| fl | 86 % | **79 %** | 91 % |
| fr | 88 % | **83 %** | 91 % |
| rl | 87 % | **82 %** | 92 % |
| rr | 94 % | **88 %** | 86 % |

**Every joint achieves 79–94 % of what it is told to do.** hip2 obeys; it is
simply asked for very little. And the mapping is not the constraint —
`t_hip2_cmd = u_hip2 * HIP_TARGET_RANGE + HIP2_REST` with **`HIP_TARGET_RANGE =
1.40`, the same authority hip1 gets**. The controller's own output is the limit:
`u_hip1` swings ~±0.9 of full scale, **`u_hip2` only ~±0.20 — 4.5× under-driven.**

## 4. hip2 and knee do not coordinate

corr(Δhip2, Δknee) across lags −8…+8: **peak |r| = 0.17**, and mostly ~0.05–0.13.
There is a faint antisymmetric hint (positive at lag −4, negative at +2…+4) on
fl/fr/rl but not rr — a trace, not a pattern. The two joints that should lift and
plant together are effectively independent.

## 5. The mechanical-advantage problem, quantified

`KNEE_REST = −1.6 rad` is a straight leg. The knee occupies **−1.36 … −0.44**, so
it works **0.24–1.16 rad bent, and never approaches extension**. Meanwhile hip2 —
the joint with the leverage to raise the chassis — sits within ±0.2 of `HIP2_REST
= 0.0`.

**The knee is doing both jobs, planting and supporting, from a permanently flexed
low-leverage posture, while hip2 idles near neutral.** That is the coordination
failure in one sentence, and it is upstream in the controller, not in the body.

⚠ **Constraint on any fix.** The ledger already refutes *hip2 stroke*, *hip2
tuck*, and *learned hip2* — all as imposed or DC biases, all of which destabilised
the gait. The proven third option is a **state-gated** bias (stance-gated knee
tuck was promoted on exactly this reasoning), and the rewrite rule prefers an
*objective whose error hip2 motion reduces* over any commanded offset. A DC hip2
push is the one thing already known not to work.

---

# Addendum 3 — the belly IS on the ground, in the window every analysis excluded

**Operator:** *"the chassis is constantly colliding with the ground."* Correct,
and I could not see it because every analysis in this document discards the first
1000 ticks (at the operator's own instruction, so the pause/step cycle can emerge).
That window is exactly where the belly is down.

## Belly clearance by phase (arena, n=3, `chassis_collides=1`)

| window | mean gc | min | **% ticks < 5 mm** | **% < 10 mm** |
|---|---|---|---|---|
| t = 1–200 | 0.0106 | 0.0018 | **19–32 %** | **58–64 %** |
| t = 200–500 | 0.0104–0.0206 | 0.0018 | 7–32 % | 26–59 % |
| t = 500–1000 | 0.0208–0.0303 | 0.0020 | 0–26 % | 2–37 % |
| t > 1000 | 0.0188–0.0240 | 0.0024 | 0–5 % | 2–19 % |

**For the first ~200–500 ticks the robot is effectively belly-down** — 60 % of
ticks under 10 mm — and it slowly works its way up. One seed (2) never fully
clears: still 26 % under 5 mm at t=500–1000 and 5 % after. Steady-state p1 is
4 mm, so the body stays marginal even once "up".

## Two things this settles

**The ghost chassis is not the cause.** Ghost vs colliding, n=3: `fwd_v` +0.0486
vs +0.0491 (**+1.1 %, inside the ±13 % seed noise**), chassis_y 0.0461 vs 0.0478,
and **0.0 % of ticks below y=0 in either arm**. If the belly were dragging
continuously, making it solid would cost real speed; it costs nothing. So the
chassis is *near* the ground, not *through* it, and `chassis_collides` is not the
lever.

**Mean clearance is the blind metric.** Mean gc ≈ 22 mm reads "fine" while 60 % of
early ticks are under 10 mm. **Judge belly work on percentiles (p1, % under 5/10
mm) and never on the mean** — this is the "chassis height is blind to belly-drag"
trap in its own right.

## Why the body cannot lift itself, connecting Addendum 2

The joint analysis already named the mechanism: **hip2 — the chassis elevator — is
4.5× under-driven** (`u_hip2` reaches ~±0.20 of full scale against `u_hip1`'s
~±0.9), while the **knee does the supporting from a permanently flexed,
low-leverage posture** (occupies −1.36…−0.44 against `KNEE_REST = −1.6` straight).
The body is lifting itself with the wrong joint, from the wrong angle. That it
takes ~500 ticks to stand, and stays at a 4 mm p1 afterwards, is what that
predicts.

## The lever this points at — and the one already refuted

`stance_lift_gain = 0.5` is live: the **stance-gated knee tuck**, promoted
precisely because a blind DC knee bias kills the gait while a *gated* one works
(CLAUDE.md §1). The ledger separately refutes **hip2 stroke**, **hip2 tuck**, and
**learned hip2** — all as imposed or DC biases.

So the untried combination is the promoted *mechanism* applied to the joint the
traces say should be doing the job: **stance-gated hip2 elevation**, gated by the
same stance signal, on the joint with both the leverage and 4.5× of unused
command range. Gain-0 guarded, and judged on the pair the operator named:
**belly-clearance percentiles must improve AND `fwd_v` must not fall** — with
`straight` and `falls` alongside, since a stiffer stance is the obvious way to buy
clearance by destroying the gait.

---

# Addendum 4 — the controller cannot see which legs are planted (2026-08-07)

**Operator, from the UI + slow motion:** *"when the diagonal legs are thrusting,
they are thrusting at the same time, and there's always a very high likelihood that
the other legs are planted, and that could result in backwards movement … during a
pause you see a sinusoidal velocity across zero … when the synchronization works,
the front plants first and then the diagonal back second, and that is what works for
a quadruped."*

Tested both halves against the traces (arm: `stancehip2`, n=6, arena+corridor).

## 1. Over-planting kills progress — CONFIRMED, and it is the dominant effect

| legs planted | ticks | mean fwd_v |
|---|---|---|
| 2 | 4 167 | **+0.0996** |
| 3 | 14 295 | +0.0489 |
| 4 | 11 181 | **+0.0128** |

Monotonic. **With all four down, forward velocity is essentially zero; with two down
it is 8× higher.** And the body sits at 3–4 planted **85 % of the time**.

## 2. Front-leads-then-hind does NOT predict progress — here

| diagonal offset | mean fwd_v after | n |
|---|---|---|
| front leads (≥ +2) | +0.0462 | 836 |
| simultaneous (−1…+1) | +0.0434 | 390 |
| hind leads (≤ −2) | +0.0484 | 603 |

t(front-leads vs simultaneous) = **+0.77** — nothing. **Reconciliation: diagonal
advanced placement is a property of a TROT, and this body is not trotting.** A trot
has two feet down; this one has 3–4 down 85 % of the time. It is crawling and
dragging, so diagonal phase ordering has nothing to bite on yet. The operator's
biomechanics is right; the precondition is absent.

## 3. ★★★ WHY — the stance gate is not a contact signal

```cpp
sw = foot_y_[i] > foot_y_ema_[i];   // "swinging" = foot above its OWN running mean
```

A signal sits above its own moving average ~50 % of the time **by construction**,
which is exactly why `swing_frac` reads 0.49 on every arm and every seed.

| | value |
|---|---|
| detector (`foot_y` > own EMA) | **0.492 ± 0.005** |
| TRUE (physics contact flag) | **0.198 ± 0.010** |
| | **2.49× over-report** |

**Of the leg-ticks the controller calls "swing", ~60 % are legs that are actually
planted and bearing load.** So every stance-gated mechanism — `stance_lift`,
`stance_lift_hip2`, the Cruse rules, `swing_tuck` — is gated on a signal that is
wrong most of the time it fires. **The controller cannot see stance overlap, which
is why nothing manages it, which is why the body sits at 3–4 planted.**

**And the true signal exists and is not wired.** The body publishes
`reality.proprio.foot_contact` every tick; `MotorEPMv2`'s `contact_topic` is
**unset** in every current config, so `have_contact_` is false and the true-contact
branch never executes. The sensor is on the bus, unconsumed.

⚠ The in-code note says *"wiring contact as the gate is separately refuted — that
consumer wanted phase"*. That refutation is about a consumer that needed a PHASE
reference, which contact cannot supply. It is not a verdict on using contact as a
STANCE gate, which is what `stance_lift` and friends actually ask for. Check the
ledger before re-proposing, but the two are different asks.

## Where this points

Not at timing. `gait_phase` already has no authority over footfalls (Addendum 1),
and DAP has no precondition. The target is **stance overlap** — get from 3–4 feet
down to 2 — and the first prerequisite is a gate that knows which feet are down.
