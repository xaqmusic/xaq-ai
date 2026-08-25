# PART IV — GainEvolver: where it stands, and what to audit

**Working document, not a report.** It exists so a fresh reader can audit PART IV's claims
before the formal write-up is attempted. `REPORTS.md` governs that write-up; this file
deliberately does not follow it. The tone here is a ledger's, not a report's.

**Status as of 2026-08-24, post-audit and post-decision-experiment: the criterion carries
real signal, the search descends it, and what fails is the criterion's own shape — its two
motion-magnitude proxies dig a nearer basin (coupling→0, energy paying flow) than the band
the displacement test demanded.** The audit is `picrawler_part4_audit.md`; the
pre-registered experiment that settled the search-vs-criterion question is its §6 and the
ledger's closing 2026-08-24 entry. The sentence the formal report has to be built around:
**the search follows its criterion; the criterion does not yet point at locomotion.**

---

## 1. What is built

A `GainEvolver` module runs a lifetime (1+1)-ES over a config-declared vector of another
module's gains, from inside the robot's own Markov blanket. It publishes a `GainVector` on
`gain.motor_epm`; `MotorEPMv2` applies each pair through its existing `on_param_change`
and **read-back verifies** it (`gains_applied` / `gains_rejected`), so a typo'd key is
visible as a counter rather than a silence.

- **Loop:** interleaved incumbent re-evaluation — incumbent window, candidate window,
  contemporaneous compare. No stored winner (the ratchet shape), no slow-forget.
- **Criterion (error form, lower better):**
  `J = w_tilt_sd·sd(upright) + w_unloaded·unloaded_mean + w_flow·flow_term + w_energy·energy`
  at weights 1.0 / 1.0 / 1.0 / 4.0. `w_falls` and `w_distress` ship at 0.
- **Guards, separate from J:** G1 falls no-regression, G2 per-leg loaded minima.
- **Gain-0 guard:** `mutation_sigma = 0` is a silent observer — scores windows, publishes
  nothing, draws no RNG. Byte-identical.
- **Tests:** 33 in `test_gain_evolver`, plus `test_clone_shipping_configs` for snapshot
  round-trip.

---

## 2. Claims, with evidence level

Read the middle column first. **`signal` means promote-or-kill only** — n=4–6 fixed-seed,
never a finding, and explicitly never a defensible null.

| # | Claim | Evidence | Where |
|---|---|---|---|
| C1 | `j1s4` is a better operating point than the hand point | **finding** — n=20, falls 4/20 → 0/20, `amp_min` sd 0.111 → 0.018, F=37.8 | job #1 |
| C2 | The criterion can see 3 of 8 gains; 2 are flat | **signal** — 6 levels × 3 windows × 3 seeds, one operating point | landscape sweeps |
| C3 | `coupling_gain`'s optimum ≈ 1.2–2.0 | **signal, replicated** — two independent operating points agree, and land on the hand-found 1.55 | step 0 + landscape |
| C4 | Measured authority predicts which gains resist scattering | **signal, replicated on 2 bodies** — STRONG 0.95/0.96, weak 1.11/1.11, FLAT 1.70/1.41 | basin study |
| C5 | The 8-D search diffuses from random starts | **signal** — spread 1.070 → 1.206 / 1.159 | basin study |
| C6 | 3-D does not fix it | **signal** — spread 0.747 → 0.777; mean moves are coin flips | 3-D basin |
| C7 | Longer windows do not change acceptance | **signal** — 0.396 vs 0.373, z = −0.42 | winverify |
| C8 | Acceptance is servoed to chance by construction | **structural** — `target_accept` AUTO resolves to the noise floor; read the code, not a run | `GainEvolver.cpp` anneal |
| C9 | `target_accept` is only a step-size selector | **signal** — `tgt070` byte-identical to `fix008` | step-size sweep |
| C10 | ~~Searching is indistinguishable from not searching~~ **SUPERSEDED by C15/C16** — the σ=0 control in this sweep never published, so its body ran the j1s4 point, not the displaced start | (retracted control; searching-arm null itself stands) | step-size sweep |
| C15 | **Displacement recovery fails at both step sizes against a TRUE displaced control** — the pre-registered null | **signal, controlled, pre-registered** — n=6/arm, window 12000, 24 gens; fix020 ΔJ −0.073 (t −1.41 ns), re-entry 1/6; fix008 0/6; control +0.029 | decision experiment |
| C16 | The search DESCENDS its criterion — toward coupling≈0, with energy (w=4) improving and flow (w=1) paying, in 7/12 runs | **signal** — per-term decomposition; pooled searching-vs-control t ≈ −1.75 (exploratory) | decision experiment |
| C17 | The single-gain landscape does not compose: from coupling 0.30 the full-context criterion has a NEARER descent (0→, 0.3 units) than the band (0.9 units) | **signal, replicates the n=1 observation** — the displacement premise, not the search, is what failed | decision experiment |
| C11 | The flow term is an amplitude proxy, not travel | **measured** — r = +0.97 with amplitude, +0.26 with `\|fwd_v\|` | flow A/B |
| C12 | The energy term reads a negated velocity copy | **documentary** — the body's own registration string says so; **not yet measured here** | `picrawler_body.gd` |
| C13 | `fwd_v` is an oracle | **structural** — `_chassis.linear_velocity` on world yaw | `picrawler_body.gd:5916` |
| C14 | 1 of 7 criterion weight units survives the port | **structural** — from the hardware inventory | sim2real port doc |

---

## 3. Retracted today — read these before trusting anything built on them

Four diagnoses died on contact with data in one session. Each was written up before it was
tested, which is now doctrine §8.

1. **"Flow prefers a slower body."** False. Flow's optimum was at the level the robot
   travelled *fastest*. `|fwd_v|` is not monotone in amplitude and the disproof was in the
   next column of my own table. The `flow_min_form` lever built on it is `NULL`, ships OFF.
2. **"Two optima sit on their range boundaries, so the search is truncated."** False — both
   argmins sit inside flat bands. The criterion resolves **bands, not points**.
3. **"Five of eight dimensions are noise and tax the three that work."** Falsified by the
   3-D test, which changed nothing.
4. **"The window is too short."** Falsified by `winverify`. The margin is `accept_k·σ̂`, so a
   longer window shrinks both together and the noise floor depends on `accept_k` alone.

Also retracted: **"planted-foot odometry needs only encoders and contact, both of which this
body already publishes"** — true in sim only; the physical robot has neither.

---

## 4. Harness traps hit in this session

Each cost real time, and each is invisible when it fires.

- **`OGMA_SEED` overrides the module's `seed` param.** Pinning it while varying the config
  seed produced three byte-identical "replicates". n=3 was n=1. An analyzer divide-by-zero
  was the only thing that caught it.
- **Configs drifted from a module default.** Every config ran `eval_window_ticks: 4000`
  while the module default was 12000 — raised in code after gate 2 measured 4000 inadequate,
  never picked up by the configs. Legal value, in range, nothing failed.
- **40,000 spurious `ERROR` lines per run** from `_emit_jsonl` probing eight module ids from
  an older stack generation. Fixed. It nearly caused a false crash alarm.
- **Acceptance rate is a controlled variable.** Three diagnoses were built on it before
  anyone read the anneal.

---

## 5. Audit brief — what a fresh reader should check first

Ordered by how much rests on it.

1. **C4, the authority cross-check.** This is the campaign's best positive and it is three
   gains per group with n=12 runs. Is the STRONG/weak/FLAT grouping robust to regrouping, or
   is it carried by `rear_land_gain`'s single large ratio? `gainevo_basin.py`.
2. **C3, coupling's "replication".** Two measurements from different operating points agree.
   Are they actually independent, or did both inherit the same fixed context for the other
   seven gains? If the latter, "replicated" is too strong a word.
3. **C2's band logic.** Bands come from one within-level noise width. Check the ascending /
   descending attribution — windows map to levels **by order**, which is exact only if the
   schedule is. Two gains were flagged for hysteresis exceeding their span.
4. **C1's attribution.** `j1s4` is genuinely better at n=20. But the protocol had the
   operator select a settled vector from the panel, so if the search diffuses this is random
   search plus human selection. The A/B stands; the mechanism credited may not.
5. **C10's ΔJ statistic.** Last third minus first third of scored windows. Is that the right
   estimator, and does the conclusion survive a slope-based one?
6. **C12 is documentary, not measured.** The energy term carries weight 4.0 — more than
   everything else combined — on a signal the body script says is "mostly a negated velocity
   copy". Measure `corr(joint_torque, joint velocity)` directly before the report repeats it.
7. **Is `flow_turn` (anti-circling) dead code?** Built, unit-pinned, never A/B'd, ships at
   `flow_turn_k = 0`. Confirm the consumer fires on `j1s4_flowturn` before it is mentioned
   as anything but unfinished.

---

## 6. Open, in rough priority (reordered by the decision experiment)

- **The CRITERION is where the failure lives — the loop was exonerated.** C15–C17: the
  search descends J; the two motion-magnitude proxies (C11 measured; C12 now also
  measured — 2026-08-06 corr(τ,dθ) −0.46..−0.56, publish site confirmed) dig a coupling→0
  basin nearer than the band. Fix the terms first: a legal travel magnitude (leg-FK+IMU),
  and energy re-pointed at `joint_load` or de-weighted from 4.0. Only then re-arm the
  displacement test — same protocol, control included. **This is now PART V's charter:
  `stride_odometry_and_criterion_repair_plan.md` (drafted 2026-08-25, operator-agreed
  direction).**
- **The accept/anneal loop, after the criterion.** Acceptance cannot be its own feedback
  signal when servoed to chance; and per the registered rule, `sigma_min` stays 0.08 for
  now — σ=0.20's faster descent and sole band re-entry is its recorded re-use context.
- **A legal travel signal.** Leg FK + IMU fusion — their disagreement is also the deferred
  post-plant-slip term. First step is a measurement, not an estimator: the commanded-vs-
  achieved FK error is published for foot *height* (22 mm mean), but odometry needs the
  **horizontal** component, never measured.
- **Gate 3, the (d)-test**, remains unrun — the charter's designated headline evidence.
- **Hardware:** an INA219 (energy term), 4 foot FSRs (`foot_load` + G2 guard + stance truth)
  and a VL53L0X belly ToF take the port from 1 of 7 weight units to 6 of 7.

---

## 7. Reproducing any of it

```sh
python3 godot_host/project/scripts_tools/gainevo_make_arms.py <basin|basin3d|winverify|tsweep> <dir>
```
emits the configs plus a `spec.json` the analyzers read. **Vary `OGMA_SEED` between
replicates** — the module's `seed` param is not an independent knob.

Analyzers: `gainevo_landscape.py` (bands + authority), `gainevo_basin.py` (convergence +
the authority cross-check), `gainevo_tsweep.py` (ΔJ vs the no-search control),
`coupling_authority.py`, `gainevo_robustness.py`.
