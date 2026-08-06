# Which leg is responsible for a forward-velocity pulse?

**2026-08-06 · arena · seed 1 · n=1 · warmup 1000 ticks · `leg_attribution.py`**

Status: **SIGNAL, not a finding** (§3 — n=1). Enough to license the UI work and
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

## Result

| leg | mean \|share\| | corr(share, pulse impulse) | stance duty |
|---|---|---|---|
| **fl** | 0.223 | **+0.424** | 0.73 |
| fr | 0.267 | +0.195 | 0.89 |
| rl | 0.188 | +0.153 | 0.85 |
| **rr** | 0.323 | **+0.060** | 0.81 |

**`rr` holds the largest share of stance sweep and earns the least forward
credit; `fl` is the inverse.** That is the effort-vs-credit divergence the tool
was built to expose. It is consistent with the ledger's standing description of
`rr` as the under-plant leg of a tripod-skid that is load-bearing for
*straightness* — a leg that stabilises rather than propels — but this run does
not establish that.

## Validation

| control | mean \|r\| | reading |
|---|---|---|
| **stance-gated (the metric)** | **0.208** | — |
| swing-gated (moves, cannot propel) | 0.093 | at the noise floor ✓ |
| ungated (pure movement) | 0.104 | halfway — stance gating carries the signal ✓ |
| shuffled pulses (null) | 0.067 | noise floor |

Plus a **lesion test** (FL commands ×0 at t=6000): FL's attributed share
**−91.1 %**, `corr` **+0.339 → −0.038**, and the load redistributes to `fr`
(+0.229→+0.445) and `rl` (+0.186→+0.344) while `rr` goes negative (−0.065).

⚠ **The lesion test is necessary but NOT sufficient**, and this is the honest
limit: the lesion zeroes the leg's commands, so its *raw sweep* also collapsed
(−94.6 %). A metric that merely counted movement would score identically. The
swing-gated control is what actually discriminates propulsion from motion, and a
per-foot friction ablation (a leg that still sweeps but slips) would be the
cleaner physical test — **it does not currently exist**; only chassis and limb
friction are ablatable.

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
