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
