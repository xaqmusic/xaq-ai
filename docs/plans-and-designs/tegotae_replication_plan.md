# Tegotae on the picrawler: what it takes to get Owaki and Ishiguro's results

*Plan and running record, opened 2026-10-18. Branch `picrawler-dev`, simulation first.
The module under test is `LoadPhaseGait` (ledger 2026-10-12 onward); the harness is
`godot_host/project/scripts_tools/tegotae_sweep.py`.*

## 1. What they showed, as claims we can test

The rule is one line per leg, with no connection between legs:

    dφ_i/dt = ω − σ · N_i · cos φ_i

φ_i is the leg's phase (swing in (0, π), stance in (π, 2π)), ω the free rate, N_i the leg's
ground reaction force and σ the feedback weight. A loaded leg is pulled toward mid-stance and
held there, and an unloaded one runs free. The legs coordinate through the body: when one
foot lifts, its weight moves to the others and holds them down.

From Owaki, Kano, Nagasawa, Tero & Ishiguro (J. R. Soc. Interface 10:20120669, 2013) and
Owaki & Ishiguro (Sci. Rep. 7:277, 2017):

| # | Claim | Their evidence |
|---|---|---|
| R1 | Four uncoupled legs started at random phases settle into a stable gait | robot, phase convergence from varied initial conditions |
| R2 | Without the load term there is no coordination | σ = 0 control |
| R3 | ω alone moves the gait: lateral-sequence walk (ω = 6 rad/s), trot (9), a brief canter, gallop (26) | treadmill, ω ramp, σ = 6 rad/(N·s) fixed |
| R4 | Near a transition two gaits are both stable (bistability, hysteresis) | return maps at ω = 12, 14, 17 |
| R5 | Each gait is the low-cost one for its speed | cost of transport against Froude number |
| R6 | The gait re-forms when the body's weight distribution changes, with no retuning | 2013 abstract: "good adaptability to changes in weight distribution" |

## 2. Their robot and ours

| | Oscillex 3 (2017) | picrawler (sim body `measured_fsr`) |
|---|---|---|
| Mass | 2.0 kg | 0.60 kg |
| Leg | 0.20 m, **one DoF**: a DC motor turns a crank, so the foot traces one fixed loop per phase cycle | three hobby servos per leg, sprawled; the foot path is whatever the motor layer commands |
| Actuator speed | DC motor + gearbox, a full crank turn per stride up to ω = 26 rad/s | servo slew **3.67 rad/s** (measured on the robot) |
| Sensing the rule uses | FSR400 pressure sensor in each foot | FSR in each foot (robot: 3× sensitivity spread between feet; sim: solver normal force) |
| Joint sensing | none in the rule. The motor's own PID holds the crank at the phase | none. The servo's own loop holds the joint at the command |
| N_i | absolute force (newtons) | the deployed module uses each foot's **share** of the total (0..1) |

Two points from this table set the plan.

**Joint sensing is not what the rule needs.** Oscillex 3 closes its joint loop inside the motor
controller, exactly as a hobby servo does. The rule's only sensor is foot load, and the
picrawler has it. The missing joint sensing limits other things here (sag is invisible, the
ledger's 2026-10-13 entries), not this rule.

**The leg is the big difference.** Owaki's phase *is* a crank angle, so a phase maps to one
foot position. Ours maps phase to a hip stroke and a swing lift through MotorEPMv2's
`leg_phase_*` sockets, with the posture spring, the stance press, the learned controller and
the heading hold all writing to the same joints.

## 3. The regime we are in: σN/ω

The rule has one dimensionless number per leg, σN_i/ω. Above 1 the loaded leg stops in late
stance at cos φ = ω/(σN) and waits for the load to leave (Owaki calls this the excitatory
regime). Below 1 it only slows (oscillatory). With σ = 6 rad/(N·s), a 19.6 N body and a quarter
of the weight per foot:

| Owaki gait | ω (rad/s) | σ·(BW/4)/ω |
|---|---|---|
| walk | 6 | **4.9** |
| trot | 9 | 3.3 |
| gallop | 26 | 1.1 |

The deployed module computes dφ = ω(1 − σ_r · share · cos φ) with σ_r = 2. With four feet down
the share is about 0.25, so σN/ω ≈ **0.5**: below Owaki's gallop, and a factor of ten below his
walk. On this body the legs never stop while loaded. They flow through stance more slowly.
Owaki's account says that regime favours trot-like phase locking. The first smoke run
(n = 1, 2000 ticks) agrees: the four oscillators lock diagonally (FL–RR relative phase 0.94,
R 0.98), though the touchdowns do not yet follow cleanly.

Shares also hide two things the absolute force carries: the body's dynamic load (a trotting
foot carries well over a quarter of the weight at mid-stance) and any weight the belly takes.
`load_mode` 1 (built 2026-10-18, default 0 = byte-identical) gives the absolute form. In the sim,
N_i is in body weights, so σ_r = σ·BW/ω and Owaki's numbers translate directly: σ_r ≈ 20 at his
walk, 13 at trot, 4.5 at gallop. On the robot, absolute mode needs `load_norm_tau`, and N_i
becomes the foot's load over its own stance reference.

## 4. The limits, stated before the runs

- **L1, slew.** The hip target moves at most 3.67 rad/s. A stroke of amplitude A (command
  units, 1.4 rad each) at rate ω needs 1.4·A·ω ≤ 3.67. At the deployed period (70 ticks,
  ω = 4.49 rad/s) and stroke 0.7 the demand is 4.4 rad/s, so **the deployed stroke is already
  slew-clipped.** Speed scales with A·ω, which the servo caps, so Owaki's speed axis does not
  exist on this body. His coordination axis still can: raise ω and shrink A to match the slew.
  Slew-matched strokes (90 % of the cap): period 100 → 0.75, 70 → 0.53, 50 → 0.38, 35 → 0.26.
- **L2, gallop.** A gallop at Owaki's ω ratio (4.3× walk) would need A ≈ 0.1. It is not
  attempted. Walk and trot are the targets.
- **L3, belly.** A sprawled body whose belly carries weight would starve the rule of its
  signal: a lifted foot's weight goes to the belly instead of the other feet. The instrument
  is `bload`. The smoke run, hip2 +0.3 posture: belly 0 % of the weight, 91 % of a lifted foot's
  load reappears on the other feet. The sim body passes. The robot is untested.
- **L4, integration step.** The phase is integrated once per 20 ms tick. The fixed point is
  stable only while ω·σ_r·N < 2 per tick. At σ_r = 20 and period 70 that allows N up to about
  1.1 body weights. Watch `retro` and period-2 flicker at high σ_r. Substepping is the fix if
  it bites.
- **L5, the robot's FSRs.** Their sensitivities differ 3× and they flicker at threshold
  (70–216 transitions per minute per foot, 2026-10-13). Absolute mode on the robot rides on
  per-foot normalisation. Contact-only mode (no force magnitude) is tested in sim as the
  fallback, and as Owaki's "phasic receptor" control.

## 5. The experiments

Each arm is one lever on its base, n = 6, seeds 1–6, arena, difficulty 0.3, 4500 ticks,
per-tick diag, `OGMA_DUMP_PARAMS` checked by the harness (an arm whose lever did not land is
dropped and named). That is a signal, not a finding (CLAUDE.md §3 rule 7).

**What the harness reads.** Gait first: relative touchdown phases of FR, RL and RR against FL
per stride, each stride matched to a template (LS-walk, DS-walk, trot, pace, bound, pronk,
irregular). The same is read off the oscillators, so a pattern the phases hold but the feet do
not follow shows as a disagreement. Then the hold (fraction of stance ticks the phase is
nearly stopped), retrograde fraction, load transfer at liftoff (to feet, to belly), belly
share, duty factor, stride period. Transport and safety last: net displacement, straightness,
speed, Froude number on hip height, falls, tilt.

**The stripped rule ("Oscillex mode").** To ask what the rule does on this body, every other
writer to the joints is zeroed: learned controller (`motor_gain` 0), babble, explore noise,
panic, stance press, swing tuck, rear landing, heading holds, progress commit, level gate,
amplitude homeostat. **The GainEvolver is removed from every config in this work** (operator,
2026-10-18: the feature needs more work and would confound these results). Left in, it sets
`coupling_gain` back to about 1.5 after its 10 000-tick warmup. Batches 1b–3 ran it at
`mutation_sigma` 0, which never publishes; removing the module reproduces those runs exactly
(checked on six seeds). The harness takes `--drop-module gain_evolver`. Kept: the posture spring (it
holds the stance) and the phase's stroke and lift. This mode is a **measurement scaffold**.
Its phase-to-joint map is an imposed foot path, which stands in for Owaki's crank. It is not
a brain to deploy. What it finds goes back into the stack as single levers (E6).

| Exp | Question | Arms | Prediction (written before the run) |
|---|---|---|---|
| E1 | Does a lifted foot's weight reach the other feet? | read from every run; then hip2 0 vs +0.3 posture | sim: yes at +0.3 (smoke 0.91). Falls toward the belly as the posture sprawls |
| E2 | Does the stripped rule stand and walk, and which regime is it in? | stack · stripped shares σ_r 2 · free clock σ_r 0 · absolute σ_r 2, 5, 10, 20 · contact-only absolute σ_r 5 | free clock: no coordination (R2). Shares σ_r 2: trot-like oscillators. Absolute σ_r 10–20: legs held in stance, sequence moves toward a lateral-sequence walk with duty > 0.6. Contact-only: deadlocks at high σ_r, since a touching leg is held whether or not it carries weight |
| E3 | Does ω alone move the gait (R3)? | best σ from E2 held fixed in rad/tick (σ_r = σ/ω), period 100/70/50/35 with slew-matched stroke | walk at long periods, trot at short, if the excitatory regime is reachable |
| E4 | Is there hysteresis (R4)? | period ramped up then down in one run (`OGMA_PICRAWLER_SETPARAM_AT`) | gait at a given period depends on the ramp's direction near the transition |
| E5 | Does the gait re-form after a weight shift (R6)? | `OGMA_PICRAWLER_COM_SHIFT` ±15 mm fore/aft, ±10 mm lateral, against the same shift on a fixed-phase clock | relative phases move with the centre of mass. The clock's cannot |
| E6 | The (d) test | mid-run leg lesion (`OGMA_PICRAWLER_LESION_*`) and slick foot (`SLICK_*`) | phases re-converge within a few strides; time to re-lock is the number |
| E7 | Back into the stack | the winning regime as one lever on `…__lpg_sl4s_st07_int` (sim), then room | one lever at a time, full metric set |
| E8 | Robot | absolute mode with `load_norm_tau` on the robot | the operator's call, after E7. Push gates are suspended until the servo rail's hardware fix |

Cost of transport (R5) needs the servo power model in the log (`OGMA_PICRAWLER_POWER_MODEL`).
It waits until a second gait exists to compare.

## 6. Built for this (2026-10-18)

- `LoadPhaseGait.load_mode` (0 shares, default and byte-identical; 1 absolute) and three unit
  tests. The duplicated statements an earlier patch left in its param handling are removed
  (they were idempotent).
- Body instruments, off by default: `OGMA_PICRAWLER_LOAD_LOG=1` adds `fload` (per-foot load,
  body weights) and `bload` (the chassis's own ground load, god's-eye, never published) to every
  diag line. Smoke run: trajectories identical with the instrument on and off.
  `OGMA_PICRAWLER_COM_SHIFT="dx,dy,dz"` moves the chassis centre of mass (a body fixture for E5).
- `LoadPhaseGait` seed read as an integer (7.1.2), with a regression test.
- `MotorEPMv2.leg_phase_lift_lead` (rad, default 0 and byte-identical, verified on six seeds):
  the swing lift curve becomes max(0, sin(φ + lead)), so lift starts in late stance.
- `tegotae_sweep.py`: writes arm configs into the run directory (never the project's config
  dir), prints each arm's diff, runs seeds in parallel, checks the lever landed, reports the
  gait metrics above.

## 7. Results

Run directories: `~/xaq_runs/tegotae_20261018/`.

### 7.1 Harness defects found on the way (each would have produced a false verdict)

1. **`babble_scale` 0 freezes MotorEPMv2.** The pulse identification divides by
   `2 · babble_scale` (`MotorEPMv2.cpp`, the babble block), so the model fills with NaN and
   every command after warmup is NaN. The joints sit at zero for the whole run, even with
   `motor_gain` 0, because 0 · NaN is NaN. The first batch's stripped arms were all this and
   were discarded. The stripped mode keeps babble (200 ticks, before the stroke engages).
2. **`LoadPhaseGait` gave every seed the same start phases.** It read its `seed` through a
   double, and the 64-bit per-run seeds OgmaBrain derives (`namespace_seed(master, id)`, one
   apart) all rounded to one value. Every LPG seed-average in the record (ledger 2026-10-12
   onward) therefore shares one set of start phases. Its spread came from the motor's RNG.
   Fixed 2026-10-18 (read as an integer), with a regression test. Seeded runs of LPG configs
   are not byte-identical to before the fix. That change is the fix.
3. **The GainEvolver turns coupling back on at tick 10 000** in every LPG config. Not hit at
   4500 ticks. Raised as a separate task, because it touches the promoted configs.
4. **Every batch here ran on the `cad` body.** Headless harness runs (`arenaavg`, `seedavg`,
   this one) ignore the config's `metadata.body` (`measured_fsr`), which only the launcher
   applies. The record's sim results share this, so the comparisons here are internally
   consistent. **But the result does not carry over to the measured body** (7.5).

### 7.2 Batch 1b — the regime map (arena, n = 6 × 4500, **one shared set of start phases**, see 7.1.2)

| arm | net (m) | straight | touchdown rel. phase FR / RL / RR (R) | strides matching a gait | hold | falls |
|---|---|---|---|---|---|---|
| stack (`…lpg_cand`) | 5.37 ± 0.40 | 0.54 | 0.66 / 0.76 / 0.24 (0.7) | irregular 99 % | 0.20 | 0 |
| stripped, shares σ_r 2 | 3.92 ± 0.37 | 0.47 | **0.44 / 0.47 / 0.06 (0.98–1.00)** | **trot 42 %** | 0.01 | 0 |
| stripped, free clock (σ_r 0) | 0.80 | 0.25 | no touchdowns (no foot ever lifts) | — | 0 | 0 |
| stripped, contact-count shares σ_r 2 | 0.67 ± 0.42 | 0.16 | (R 0.17) | trot 1 % | 0.00 | 0 |
| stripped, absolute σ_r 2 | 4.04 ± 0.53 | 0.48 | 0.45 / 0.47 / 0.06 (0.96–1.00) | trot 38 % | 0.02 | 0 |
| stripped, absolute σ_r 3 | 0.29 | 0.07 | irregular | 0 % | 0.45 | 0 |
| stripped, absolute σ_r 4 | 0.30 | 0.27 | none | — | 0.75 | 0 |
| stripped, absolute σ_r 5 | 0.15 | 0.18 | none | — | 0.84 | 0 |
| stripped, absolute σ_r 7 | 0.01 | — | none | — | **0.99** | 0 |

Readings (provisional until the seed-fixed rerun, batch 2):

- **The stripped rule trots.** With nothing but four load-held phases, a posture spring, a
  stroke and a lift, the diagonal pairs land together on every seed, at a relative phase
  locked to R ≈ 1, and 40 % of strides match the trot template within an eighth of a cycle.
  The oscillators themselves lock the same way. This is the oscillatory regime (σN/ω ≈ 0.5),
  and trot is the gait Owaki's account assigns it. The deployed stack's extra terms (learned
  layer, stance press, tuck, holds) carry it 37 % further but break the touchdown pattern
  (irregular 99 %): the stack's oscillators trot, its feet do not.
- **R2 reproduces.** The free clock goes nowhere, and here no foot even leaves the ground.
- **Force matters, not just contact.** Contact-count shares (Owaki's "phasic receptor")
  lose the pattern and the transport.
- **The held regime deadlocks.** From σ_r 3 the legs park at the stance fixed point. By σ_r 7
  all four hold for the whole run (hold 0.99) and the body stops. A held leg is released only
  when its load falls below ω/σ, and on this body nothing unloads a leg that has stopped:
  every other leg has stopped too, so nothing moves the centre of mass off it. Owaki's walk
  sits at σN/ω ≈ 5, deep inside this regime, yet it walks. The difference to test (batch 2) is
  the foot path. A crank's foot rises before the phase reaches liftoff, so a held leg sheds
  its own load and lets go. Our lift, max(0, sin φ), starts exactly at liftoff.
  `leg_phase_lift_lead` starts it earlier.

### 7.3 Batch 2 — seed-fixed rerun and the late-stance lift (arena, n = 6 × 4500, start phases now differ per seed)

Gain-0 guard: `stack` with `leg_phase_lift_lead` 0 explicit equals `stack` on every metric, on
all six seeds.

| arm | net (m) | straight | touchdown rel. phase FR / RL / RR (R) | strides matching | hold | falls |
|---|---|---|---|---|---|---|
| stack | 5.16 ± 0.62 | 0.54 | 0.46 / 0.62 / 0.09 (0.8–0.9) | irregular 97 %, trot 2 % | 0.21 | 0 |
| stripped, shares σ_r 2 | 4.09 ± 0.40 | 0.49 | **0.45 / 0.48 / 0.06 (0.95–1.00), trot on 6/6 seeds** | **trot 38 %** | 0.01 | 0 |
| stripped, free clock | 0.33 ± 0.34 | 0.05 | none | — | 0 | 0 |
| stripped, contact-count shares | 0.31 ± 0.26 | 0.08 | none | — | 0 | 0 |
| stripped, absolute σ_r 3 | 0.28 | 0.06 | irregular | 0 % | 0.45 | 0 |
| stripped, absolute σ_r 5 | 0.15 | 0.18 | none | — | 0.84 | 0 |
| shares σ_r 2 + lift lead 0.6 | 0.22 | 0.02 | none (RR never lifts) | — | 0.01 | 0 |
| absolute σ_r 3 + lead 0.6 | **2.90 ± 0.06** | 0.27 | 0.38 / 0.42 / 0.02 (0.83), trot 4/6 seeds | trot 2 % | 0.18 | 0 |
| absolute σ_r 5 + lead 0.6 | 0.15 | 0.15 | none | — | 0.77 | 0 |
| absolute σ_r 5 + lead 1.2 | 0.28 | 0.10 | irregular | 0 % | 0.64 | 0 |
| absolute σ_r 10 + lead 1.2 | 0.23 | 0.07 | none | — | 0.73 | 0 |

**R1 reproduces (signal).** From six different random start phases the stripped rule settles on
the same trot every time, with the same diagonal pairing (FL with RR) and the same stride
(81 ticks). The free clock and the contact-count control stay uncoordinated.

**The lift lead is not the release.** It frees σ_r 3 (0.28 → 2.90 m, trot-like phases) but not
σ_r 5 or 10. In the oscillatory regime it breaks the trot (one leg stops lifting): a
`REGRESSION` there. Verdict: `PARTIAL`, in the stripped mode, arena, n = 6. Re-use if the
posture changes the load profile (below).

**Why the ceiling sits at σ_r ≈ 3: the late-stance load.** Mean foot load by stance phase
(eighths of stance, body weights, stripped σ_r 2):

| leg | early → late stance |
|---|---|
| FL | 0.23 0.25 0.28 0.30 0.34 **0.41 0.37** 0.30 |
| FR | 0.27 0.29 0.33 0.39 0.45 **0.43 0.40** 0.26 |
| RL | 0.31 0.40 0.48 0.46 0.44 **0.43 0.37** 0.30 |
| RR | 0.18 0.22 0.24 0.29 0.42 **0.39 0.34** 0.28 |

The load *peaks* in late stance, where the rule's hold point sits (cos φ > 0). A held leg leaves
only when σ_r · N · cos φ < 1 for the rest of its stance. With this profile the largest
N · cos φ is about 0.33, so the strongest hold the body can release is σ_r ≈ 1/0.33 = **3.0**
(the harness's `sigma_crit`, 3.02 ± 0.03). The observed deadlock starts between 2 and 3.
Owaki's mechanism (his Fig. 4D) is the opposite profile: as a leg sweeps back, the body's
weight moves forward onto the other legs and the held leg's load falls toward zero. On this
sprawled body the hip-1 stroke swings a foot around a vertical axis far out to the side, so a
foot's fore-aft position barely changes how much weight it carries.

**What this says is necessary for Owaki's walk:** a body whose late-stance foot load falls as
the leg sweeps back. That means feet that move fore-aft under the body, not around it. The
sprawl, not the sensing, is the limit. Batch 3 tests whether posture (`hip2_tuck_target`) moves
`sigma_crit`.

### 7.4 Batch 3 — Owaki's ω sweep, the weight shift, posture (stripped rule, absolute load, n = 6 × 4500)

**E3, ω at fixed σ** (0.157 rad/tick per body weight, so σ_r = 0.025 · period; stroke at 90 % of
the slew cap, A = 0.0075 · period):

| period (ticks) | σ_r | stroke | net (m) | straight | touchdown pattern (R) | trot strides | oscillators | what the feet do |
|---|---|---|---|---|---|---|---|---|
| 100 | 2.5 | 0.75 | 0.35 | 0.08 | irregular | 0 % | irregular | FR almost never lifts (near the hold ceiling, 3.24) |
| 85 | 2.1 | 0.64 | 1.36 | 0.23 | irregular (0.35–1.0) | 1 % | trot 6/6 | rear pair lifts half as often |
| **70** | 1.75 | 0.53 | **4.35 ± 0.67** | **0.55** | **trot 6/6 (R 1.00)** | **59 %** | trot 6/6 | all four step evenly (46–51 touchdowns) |
| 55 | 1.4 | 0.41 | 4.84 ± 0.95 | 0.51 | lock lost (R 0.2) | 2 % | trot 5/6 | RR lifts a third as often |
| 40 | 1.0 | 0.30 | 5.24 ± 0.48 | 0.54 | none | 0 % | trot 6/6 | **rear feet never lift** (4 touchdowns); front pair paddles |
| 30 | 0.75 | 0.23 | 4.88 ± 1.60 | 0.46 | none | 0 % | trot 6/6 | rears lift rarely |

**R3 does not reproduce on this body, and the record says why.** The oscillators lock into trot
across a 3.3× range of ω, and no walk or gallop phase pattern appears anywhere. At long periods
the hold strength nears the body's ceiling and legs start to park. At short periods the
oscillators still trot, but the rear feet stop following: the swing lift is slew-clipped
(a 0.84 rad hip-2 lift in a quarter of a 30–40 tick period asks for 4–6 rad/s against 3.67),
and the rears carry more load. Transport rises only 20 % from period 70 to 40 while ω rises
75 %, which is the slew cap on A·ω (L1) showing directly. The clean trot sits at the deployed
period with a stroke inside the slew cap (0.53): 59 % of strides match the template, against
42 % at the slew-clipped 0.7 stroke (control below).

**E5, weight shift** (against the matched control, stripped absolute σ_r 2: 4.26 ± 0.36 m,
straight 0.51, touchdown trot 6/6, trot strides 42 %):

| COM shift | net (m) | touchdown pattern | oscillators | reading |
|---|---|---|---|---|
| none (control) | 4.26 | trot 6/6 (R 0.96–1.00) | trot 6/6 | — |
| 30 mm forward | 0.22 | irregular | trot | FR stops lifting. The gait does not re-form: `REGRESSION` |
| 30 mm back | 1.25 | trot 6/6 (R 1.00) | **DS-walk pattern 6/6** | the oscillators re-organise, the feet keep a trot, transport −70 % |
| 15 mm left | 3.60 | lock lost | DS-walk 4/6 | partial |

The oscillators' phase relationships do move with the weight distribution, which a fixed-phase
clock cannot do. That is the mechanism behind R6. But the body does not keep walking well
through it. `PARTIAL` (signal). The fixed-phase clock comparison arm has not been run.

**E1′, posture** (`hip2_tuck_target`, base 0.3):

| tuck | σ_r 2 | σ_r 3 | `sigma_crit` |
|---|---|---|---|
| 0 | 0.51 m, irregular | 1.06 m, hold 0.46 | 3.06 / 3.22 |
| 0.3 (base) | 4.26 m, trot | 0.28 m, hold 0.45 | 3.05 / 3.27 |
| 0.5 | 4.80 m, trot 6/6, 39 % | 0.27 m, hold 0.44 | 2.99 / 3.37 |

The posture range this knob reaches leaves the hold ceiling at 3.0–3.4, so the held regime stays
locked at σ_r 3 in every posture. Tuck 0 also breaks the trot at σ_r 2. The ceiling is a
property of the sprawl, not of this knob's setting.

### 7.5 The measured body (2026-10-18)

The trot arm (σ_r 1.75, period 70, stroke 0.525, no GainEvolver), n = 6 × 4500, arena, the body
selected with `OGMA_PICRAWLER_BODY`:

| body | net (m) | straight | touchdowns FL / FR / RL / RR | clean trot strides | oscillators |
|---|---|---|---|---|---|
| `cad` | 4.35 ± 0.67 | 0.55 | 46 / 49 / 51 / 48 | 59 % (6/6 seeds trot) | trot 6/6 |
| `measured_fsr` | 2.98 ± 0.43 | 0.40 | 47 / 51 / **12 / 0** | **0 %** | trot 6/6 |

On the measured body (the robot's measured masses and centre of mass) the oscillators still lock
into trot, but the rear feet barely leave the ground, so the pattern never reaches them. This
matches the robot-side record that the rear feet scrub rather than rise (2026-10-13). **Every
finding in 7.2–7.4 is scoped to the CAD body until rerun on `measured_fsr`.** The first rerun is
the lift dose on the measured body, because the rears are the ones not clearing.

Both bodies are in the launcher: "EXP · TEGOTAE · bare load rule trots (sim, CAD body)" and
"EXP · TEGOTAE · same rule, measured body: rear feet drag".

### 7.6 Why the yellow leg drags (operator question, 2026-10-18)

Yellow is code leg `rr` (anatomical rear-left, the naming mirror at `picrawler_body.gd:343`).
On the measured body it never leaves the ground once stepping starts, and the other rear (blue,
`rl`) lifts only rarely. On the CAD body it lifts, but it is the leg in contact longest in
mid-swing (61 % against 14 % for red). Trot arm, measured body, n = 6 × 4500 unless noted.

**Ruled out:**

| suspect | test | result |
|---|---|---|
| the servo cannot lift it | trace, one seed: target vs achieved joint angle in mid-swing | the joints go *past* the command (hip 2 −0.23 vs −0.16 rad, knee −0.73 vs −0.68): the leg is folded by the body sinking onto the foot, which stays in contact 100 % of mid-swing at ~0.09 BW |
| the chassis attitude (yellow is the lowest corner) | the planted-leg corner integrator (`level_int_gain` 0.005) | pitch and roll to 0.000, and yellow still airborne 0.00 — `NULL` |
| static geometry | stance foot positions vs the centre of mass | feet symmetric (centroid within 1 mm of the chassis centre); the measured body's COM is centred fore-aft, only 8 mm higher |
| the COM height | COM 8 mm lower | unchanged (yellow 1 touchdown) — `NULL` |
| too little lift | lift × 1.5 more (hip 2 −0.9 / knee +0.6) | worse: both rears stop, pitch −0.085, net 0.10 m — `REGRESSION` |

**What frees it:**

| arm | touchdowns FL / FR / RL / RR | clean trot strides | net (m) | yellow's load in its own mid-swing (BW) |
|---|---|---|---|---|
| measured, base | 47 / 51 / 12 / **0** | 0 % | 2.98 | 0.070 |
| COM 15 mm **forward** | 48 / 46 / 40 / **46** | **77 %** | 3.33 | 0.041 |
| stroke 0.35 (from 0.525) | 48 / 48 / 50 / **38** | **77 %** | 2.82 | 0.046 |
| CAD body, base | 46 / 49 / 51 / 48 | 59 % | 4.35 | 0.037 |

**Reading.** The rear swing foot is pinned by weight that arrives *while the gait runs*, not
by where the body sits at rest. Each push of the stance legs is a forward force at ground level,
below the centre of mass, and it pitches the body nose-up and loads the rear. Taking load off
the rear in either of two independent ways — a weaker push, or the COM moved forward — frees
both rear legs, and the trot reaches all four feet (77 % clean strides, more than on the CAD
body). Raising the front feet higher pitches the body further back and makes it worse. That is
also why it begins "after a few steps": the stepping and the push build up first. Yellow is
the worst of the two rears because the body also rolls toward it (roll −0.03 in every arm,
the record's long-standing "right rear lowest" corner in code names). The roll alone is not the
cause, because levelling it does not help. The measured body sits closer to this edge than the
CAD body (yellow's mid-swing load 0.070 vs 0.037). *Superseded by 7.7: the swing-load gate drove that load to 0.032 and the foot still did not lift, so load is not what holds it down.*

Verdict: diagnosis, n = 6, one gym, measured body; the COM shift is a body fixture, not a
fix. The fix in the rewrite rule's terms: the error is "this foot should be swinging and still
carries load", and the foot sensor already reports it. A brain-side lever would gate the stance
legs' push by the swinging feet's load — the same shape as the promoted level gate, on a
different error. Not built yet.

### 7.7 The swing-load push gate (A/B, 2026-10-18): `REGRESSION`, and the 7.6 reading is refuted in part

Built as `MotorEPMv2.stroke_swingload_gain` (+ `_floor`). Measured body, n = 6 × 4500. Gain-0 arm
byte-identical per tick on all six seeds. The gate fired (mean 0.70 / 0.47 / 0.35 at gains
4 / 8 / 15) and cut yellow's mid-swing load from 0.070 to 0.032 body weights, below the CAD body's
0.037. Yellow still never lifted (0 touchdowns in every arm), and transport fell (3.11 / 0.77 /
1.18 m against 2.98). A constant stroke of 0.35, the same mean push as gain 4, frees yellow
(38 touchdowns, 77 % trot) where the gate does not. On the CAD body, gain 8 cuts the clean trot
from 59 % to 13 %. Full table in the ledger (2026-10-18 evening).

So the foot is not held down by load. 7.6's "the push pitches the body onto the swinging rear
foot" is refuted in its within-stride form. What freed it in 7.6 (a smaller stroke, the COM
forward) changes the foot's path or the standing load, and the record cannot yet tell those
apart: `feet_y` is a shank-angle proxy, and the contact flag says only *that* the foot touches.
Next: a true foot-clearance instrument (the lower segment's tip height, god's-eye, instrument
only), then rerun base / stroke 0.35 / COM forward with it.

### 7.8 Can a leg's press control its corner? The late-stance press probe (2026-10-18)

The operator's proposal: each leg learns how hard to push, to hold up the chassis corner it is
responsible for. The physics constraint first: in a trot's diagonal support the two stance feet
lie on the axis the body tips about, so their forces have no moment about it. A corner's dip
during its own leg's swing is therefore set *before* liftoff. Probe: `leg_phase_late_press` (a
fixed press over the second half of stance, w = −sin 2φ, + raises the corner) and
`leg_phase_late_press_legs` (per-leg multipliers). Measured body, n = 6 × 4500. Gain 0
byte-identical per tick on six seeds. Corner error = MotorEPMv2's level convention, ×1000,
+ = the corner is low.

| arm | yellow's corner, own swing | yellow, own stance | touchdowns FL/FR/RL/RR | net (m) |
|---|---|---|---|---|
| base | +177 | +56 | 47 / 51 / 12 / 0 | 2.98 |
| uniform +0.1 | +131 | +100 | 47 / 56 / 40 / 0 | 2.89 |
| uniform +0.2 / +0.4 | +136 / +133 | +153 / +214 | 49/9/6/0, 52/12/6/0 | 2.75 / 2.92 |
| uniform −0.2 | +65 | +46 | 13 / 4 / 3 / 9 (gait collapses) | 0.25 |
| **signed per corner** ×0.1 (multipliers −1, −0.38, +0.12, +1 from each leg's measured swing dip) | **+101** | **+23** | 47 / 44 / 0 / 0 | 1.80 |
| signed ×0.2 | **+96** | **−10** | 48 / 37 / 0 / 0 | 1.22 |
| signed ×0.4 | +88 | −58 | 49 / 0 / 0 / 0 | 0.25 |
| yellow only +0.2 | +152 | +38 | 47 / 50 / 34 / 0 | 2.93 |
| CAD body, uniform +0.2 | +132 (base +129) | +105 (base +41) | 46/62/61/48 | 4.89 (base 4.35) |

**Readings.** (1) The press *has* authority over the corners. A uniform press only trades the
stance dip for the swing dip. A press signed per corner levels both: stance near 0, yellow's
swing dip roughly halved, to below the CAD body's +129 where yellow lifts. The error the
operator proposes is controllable by the actuator proposed. (2) **A level corner does not free
the yellow foot.** At signed ×0.2, yellow's corner is lower in swing than on the CAD body and the
foot still never lifts, while blue stops too and transport falls. Like load (7.7), corner
attitude is not what holds the foot down.

**What the true clearance instrument found** (`ftip`/`fknee` under `OGMA_PICRAWLER_LOAD_LOG`, the
lower capsule's toe and knee-end height): yellow's toe sits ~5 mm deeper than the other three
toes **from spawn, before any gait runs** (first 150 ticks: −10 to −11 mm against −6), in stance
(−12 against −6 to −8 on the measured body), and ~15 mm through the first half of its swing in
*every* arm, freed ones included. With the brain off (all commands 0, legs hanging), leg 3's
joints settle at different angles from the other three (hip 2 +0.017/+0.025 against
+0.009–0.013, knee −1.590 against −1.600). Leg 3 is built as an exact mirror (its `lower_dir`
is the others' reflected), and the body files carry no per-leg offsets. **There is a body-side
asymmetry at leg 3**, present on both bodies and with no brain. It is the likeliest source of the
record's long-standing "right rear lowest" corner (code names) and of yellow dragging. Not yet
located: joint construction for leg 3 (anchor, axis, limits), solver ordering, the spawn pose.

**For the learned per-leg corner press:** the probe supports it as a terrain and levelling
mechanism (the corner error is observable and the late-stance press moves it, per corner and
signed). It is not the stuck-leg fix. Judge it on corner level and terrain, after the leg-3
asymmetry is found, because a body defect at one leg would confound every per-leg result.

### 7.9 The leg-3 asymmetry is the physics engine (2026-10-18)

**Cause.** Godot Physics 3D (the project's engine) lets a contact sink up to
`SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION` = **10 mm** into the floor, and it solves contacts and
joints sequentially, in creation order. The leg built last settles deepest. Legs are built 0, 1, 2,
3, so leg 3 (`rr`, yellow) is the deep one.

**Evidence.**
1. *Build order moves the odd leg* (diagnostic `OGMA_PICRAWLER_LEG_BUILD_ORDER`, every per-leg array
   restored to index order; measured body, n = 3 × 2500). Default order: yellow's stance toe −13.4 mm
   (others −6.4 to −7.8), 0 touchdowns. Built 3, 2, 1, 0: yellow −6.9 mm and 20 touchdowns, and leg 0
   (now built last) goes deep (−10.8). Brain off (all commands 0, legs hanging): the last-built leg's
   hip 2 settles off the others (default leg 3 +0.016 vs ~+0.012; reversed leg 0 +0.023, leg 3
   back to +0.012).
2. *A tighter penetration allowance removes it* (`OGMA_PICRAWLER_CONTACT_PENETRATION`, new, unset =
   engine default). Measured body, n = 6 × 4500:

| arm | stance toe FL / FR / RL / RR (mm) | touchdowns | trot strides | corner dip in own swing FL / RR | net (m) | straight | tilt_sd |
|---|---|---|---|---|---|---|---|
| measured, 10 mm (default) | −7.7 / −6.1 / −6.3 / **−13.0** | 47 / 51 / 12 / **0** | 0 % | −180 / +177 | 2.98 | 0.40 | 0.031 |
| measured, **2 mm** | −6.2 / −6.0 / −6.0 / −6.4 | 47 / 56 / 28 / **39** | **31 %** (trot 5/6 seeds) | −105 / +99 | 1.93 | 0.28 | 0.022 |
| CAD, 10 mm (default) | −5.6 / **+0.7** / −5.8 / **−10.1** | 46 / 49 / 51 / 48 | 59 % | −180 / +129 | 4.35 | 0.55 | 0.047 |
| CAD, **2 mm** | −6.3 / −5.2 / −5.6 / −6.5 | 46 / 58 / 54 / 46 | 55 % | −163 / +115 | 4.45 | 0.56 | 0.028 |

At 2 mm the four feet sit within 0.4 mm of each other. On the measured body yellow steps and the trot
reaches the feet. The base's extra distance came from the front pair paddling while the rears
slid. On CAD distance and straightness tie and tilt falls 40 %. More solver iterations could not be
tested: `OGMA_SOLVER_ITERATIONS=256` left the space at 64 iterations (the setting goes to
ProjectSettings after the space exists), so that knob looks dead at runtime. Recorded, not chased.
A physics rate of 480 Hz breaks the gait (not informative here).

**What it explains.** The record's "right rear corner lowest" (code names, ledger 2026-10-13: "every
leg is equal so asymmetry should not be a normal outcome"), the yellow drag, and why every
load-, attitude- and press-based fix (7.6–7.8) failed. The foot was ~6 mm deeper *in the floor*.
**Every sim result in the record carries this bias**, at leg 3 and to a lesser degree elsewhere (FR at
+0.7 mm on CAD).

**Status: 2 mm is the sim default from 2026-10-18** (operator: "not unlike the travel on the real FSR
toes"). `picrawler_body.gd` `contact_max_penetration` = 0.002; `OGMA_PICRAWLER_CONTACT_PENETRATION=0.01`
reproduces earlier runs. Check on the deployed stacks (arena, n = 6 × 4500, 2 mm vs 10 mm): the
promoted LPG stack (sim) 5.73 ± 0.41 vs 5.80 ± 0.52 m, straight 0.54 vs 0.56, 0 falls in both;
the hip 2 +0.3 benchmark 2.80 ± 0.30 vs 2.90 ± 0.36 m, 0 falls in both. Both tie. **Every sim
number in this plan before 7.9 ran at 10 mm.** The Tegotae launcher entries now describe the 2 mm
runs.

### 7.10 The corridor stall is the walls, not leg 3 (operator observation, 2026-10-18)

Operator: in the corridor the left rear (yellow) leg gets stuck again; in the arena it works. The
configs are current, and the 2 mm contact default is live in the corridor (the run log prints it).
Corridor, difficulty 0.3, n = 6 × 4500, 2 mm:

| arm | net (m) | straight | touchdowns FL/FR/RL/RR | trot strides | where yellow stops |
|---|---|---|---|---|---|
| measured, stripped (no heading hold) | 0.99 ± 0.45 | 0.23 | 38 / 30 / 20 / 32 | 7 % | against the LEFT wall: x −0.65 to −0.87 (the flat floor ends at ±0.75, then the 30° μ = 3 wall), yaw −62 to −84°, on 3/6 seeds; the others keep stepping |
| CAD, stripped | 2.27 ± 1.29 | 0.33 | 41 / 41 / 44 / 40 | 36 % | left wall (2 seeds), the first rumble strip at z 4.25 (1), turned round (2) |
| measured + heading hold (the stack's 0.3 / 7.0) | 1.96 ± 0.15 | 0.34 | 39 / 32 / 25 / 25 | 6 % | yaw held within ±15°; yellow barely steps for the first 2000 ticks (0–4 touchdowns per 1000) while the body crabs right, then steps at the right wall |
| CAD + heading hold | 3.41 ± 0.15 | 0.48 | 44 / 45 / 40 / 45 | 37 % | no stall |

The stripped mode has no steering, so in the corridor the body curves into a side wall (on the
measured body, the left wall on 5/6 seeds) and the leg pressed against it stops. That is a
different mechanism from the leg-3 contact bias, and it does not appear in the open arena. The
heading hold keeps the body pointed down the corridor and doubles transport on both bodies. On the
measured body, though, the hold leaves yellow nearly still for the first half of the run on flat
floor. The heading hold rides the stroke as a left/right differential, and on this body the rear
pair is marginal (7.6), so the steer differential may be what tips yellow. Not yet tested:
yellow's stroke amplitude under the hold, and the hold with the stroke at 0.35.

### 7.11 Why both rear legs drag (operator: "both back legs drag in the corridor, the fronts step") — 2026-10-18

All at 2 mm contact, measured body, Tegotae trot config, n = 6 × 3000, true toe clearance
(`ftip`), contact class (`fcol`, new: floor / wall / obstacle).

**It is not the corridor.** In the open arena the rear feet are also in contact 89–94 % of their
own mid-swing, and the rear toes never rise above −4 mm (fronts +5 to +15 mm). The arena's
touchdown counts were flickers. The feet touch the floor, not a wall.

**Ruled out at 2 mm:** chassis pitch (levelled to 0.000: unchanged); swing lift (× 1.5: fronts
rise to 34–52 mm, rears worse, in contact 100 %); posture tuck 0.5 (partial: rear contact
0.53/0.77, net 1.95 → 3.0 m); foot–floor friction (μ 0.8/0.5/0.3: worse — at 0.3 no foot lifts
and the feet slide; a per-foot material had **no effect at all** first, because Godot combines
two `rough` materials by keeping the higher friction, so the diagnostic now sets the shared
ground material, `OGMA_PICRAWLER_GROUND_FRICTION`); gait pattern (an imposed trot clock: fronts
+28–32 mm, rears in contact 100 %; an imposed walk clock: three legs pinned); build order (moves
the pinned leg when marching in place, not when walking forward).

**The mechanism:**

| arm | load at the phase's liftoff FL / FR / RL / RR (BW) | 10 ticks into swing | mid-swing airborne FL / FR / RL / RR |
|---|---|---|---|
| walking forward | 0.20 / 0.22 / 0.23 / 0.22 | 0.03 / 0.02 / **0.08 / 0.08** | 0.70 / 0.47 / **0.07 / 0.11** |
| COM 30 mm forward | 0.19 / 0.27 / 0.24 / 0.15 | 0.02 / 0.02 / 0.10 / 0.04 | 0.48 / 0.10 / 0.39 / 0.33 |
| COM 15 mm back | 0.21 / 0.22 / 0.24 / 0.21 | 0.03 / 0.04 / 0.09 / 0.11 | 0.81 / 0.62 / 0.03 / 0.00 |

1. **The rule lifts every leg while it still carries a quarter of the body.** At σ_r 1.75 the hold
   is σ_r·N ≈ 0.4 < 1: the load slows the phase but never stops it, so liftoff comes on the clock,
   loaded. Owaki's walk (σN/ω ≈ 5) waits until the load has gone.
2. **The front legs shed that load within a few ticks; the rear legs do not.** The forward push
   keeps the body's weight on the rear pair. Sweeping the COM forward moves the drag continuously
   from the rear pair to the front pair (rear mid-swing contact 0.94/0.89 → 0.61/0.67 at
   +30 mm; trot strides 33 → 90 %), and back 15 mm makes the rears drag completely.
3. **Waiting for the unload deadlocks** (7.2: from σ_r ≈ 3 every leg parks), because nothing on
   this body moves the weight off a leg that is waiting.

**So what is missing is a weight shift**: something that takes the load off a leg *before* its phase
lifts it. Owaki's narrow, legs-under-body robot does it passively. The sprawled picrawler does not.
In this project's terms the error is "my phase says swing and my foot still carries weight". The
foot sensor already measures it, and the legs that can descend it are the *other* legs, by moving
the body off this one. That is the operator's per-leg learned press (7.8), pointed at the neighbour's
unload as well as its own corner.

**Correction to the record:** the "trot" on the measured body (7.5, 7.9) is a touchdown-timing
pattern whose rear feet never clear. The CAD body's rears clear by a few millimetres (7.3).

### 7.12 The force sensors, the robot's own stance press, and the learned weight-shift press (2026-10-18)

Operator: the diagonal trot works on the real robot, its diagonal legs push together, and its rear legs
do not drag; check the sim's force sensors so the back legs know when to release.

**The sim force channel is correct** (measured body, 2 mm, a walking run): feet + belly = 0.999 BW on
average (5–95 %: 0.92–1.10); LoadPhaseGait's N equals the published `foot_load` on the same tick (to its
0.005 rounding); a foot the physics flag calls airborne reads 0.0007 BW on average; the load falls below
0.02 BW on the same tick as liftoff. The rear legs do sense their load. The stripped rule (σ_r 1.75)
lifts them on the clock anyway.

**The robot's config, run in sim, clears its feet, and its stance press is the weight shift** (the real
robot's `…__lpgn_r05_e0d0_ref300`, measured body, corridor, n = 6 × 3000, GainEvolver removed):

| arm | mid-swing contact FL / FR / RL / RR | toe peak in swing (mm) | load 10 ticks into swing | net (m) |
|---|---|---|---|---|
| robot config (stance press 0.5) | 0.14 / 0.27 / 0.61 / 0.84 | 62 / 53 / 19 / 6 | 0.03 / 0.06 / 0.12 / 0.10 | 2.47 |
| robot config, stance press 0 | 0.94 / 0.91 / 0.94 / 0.89 | −2 / 2 / −2 / 1 | 0.19 / 0.22 / 0.21 / 0.15 | 0.65 |

The stripped Tegotae entries removed the stance press, which is why their rear feet drag and the
robot's do not. The stance press is "the diagonal legs push together".

**Lever: the learned weight-shift press** (`MotorEPMv2.wshift_rate`, `_leak`, `_max`, `_hip2`). Each
standing leg's press amplitude rises with the swing-weighted load still on the other feet and leaks
back. Gain 0 byte-identical on six seeds. Robot config, corridor, measured body, n = 6 × 4500:

| arm | mid-swing contact RL / RR | rear toe peak (mm) | learned press (end) | net (m) | straight | falls | tilt_sd |
|---|---|---|---|---|---|---|---|
| base (fixed press 0.5) | 0.74 / 0.87 | 21 / 20 | — | 3.23 ± 0.41 | 0.42 | 0.5 | 0.075 |
| + learned, rate 0.02, max 1 | 0.33 / 0.51 | 75 / 45 | 0.74–0.89 | 0.34 | 0.03 | 1.2 | 0.543 |
| + learned, rate 0.05, max 1 | 0.28 / 0.40 | 103 / 55 | 0.78–0.98 | 0.35 | 0.04 | 2.7 | 0.743 |
| learned replaces fixed, 0.05 | 0.31 / 0.47 | 56 / 31 | 0.98–0.99 | 0.39 | 0.03 | 2.2 | 0.547 |
| + learned, max 0.1 | 0.61 / 0.83 | 25 / 13 | 0.10 (all) | 2.31 | 0.34 | 0.3 | 0.115 |
| + learned, max 0.2 | 0.26 / 0.58 | 41 / 19 | 0.20 (all) | 2.12 | 0.30 | 0.2 | 0.078 |
| + learned, max 0.4 | 0.53 / 0.51 | 21 / 24 | 0.40 (all) | 0.47 | 0.04 | 0.8 | 0.227 |
| fixed press 0.7 (control) | 0.32 / 0.62 | 37 / 15 | — | 1.77 | 0.25 | 0.2 | 0.090 |
| stripped Tegotae + learned 0.05 | 0.44 / 0.58 (base 0.94 / 0.66) | 21 / 23 | 0.99 (all) | 0.53 (base 0.99) | 0.04 | 0.3 | 0.157 |

**Verdict: `REGRESSION`.** Every leg's amplitude winds up to its bound, so the "learned" press is
a constant extra press. It never converges, because some residual swing load always remains. A
larger press frees the rear feet and costs transport, and the fixed 0.7 control shows the same trade.
Above about 0.3 extra the body rocks (tilt_sd ×3–10) and the residual swing load *rises*
(0.27–0.44 BW), so the error's sensitivity to the press changes sign. An integrator that assumes the
sign cannot find the optimum. Two physical notes for the next form: (1) pressing every standing leg
equally raises the body but does not move weight between legs, so it cannot unload one particular
swinging foot; the shift has to be *differential* (the diagonal pair, the sinking corner); (2) the
learning has to estimate whether more press helps (a gradient, or a target with a deadband), rather
than assume it. **Re-use context:** a press learned per leg *and per neighbour* with an estimated
sensitivity, judged against the fixed 0.5 press.

### 7.13 The dragging rear feet are an unconverged physics solver (2026-10-18)

After the stance press, one rear foot still dragged, and which one depended on the legs' build
order (stripped rule + press, corridor: normal order yellow drags, 3.57 m; reversed order blue
drags, 1.14 m). The gait sat on a knife edge and the solver's order picked the side.

**`OGMA_SOLVER_ITERATIONS` never reached the engine.** `_ready()` configures physics before
`_resolve_env()` runs, so the space stayed at 64 iterations whatever the env said. `OGMA_PHYSICS_HZ`
has the same order bug: it changes this script's normalisations but not the engine rate, which is
why "480 Hz broke the gait" in 7.9. Fixed for iterations (applied to the space after the env is read).
The physics rate is recorded, not fixed.

**At 256 iterations the rear feet lift** (corridor, measured body, 2 mm, n = 6 × 4500):

| arm | mid-swing contact FL / FR / RL / RR | toe peak (mm) | load 10 ticks into swing | net (m) | straight | falls |
|---|---|---|---|---|---|---|
| robot config, 64 | 0.18 / 0.36 / **0.74 / 0.87** | 60 / 64 / 21 / 20 | 0.02 / 0.07 / 0.12 / 0.10 | 3.23 | 0.42 | 0.5 |
| robot config, 256 | 0.24 / 0.32 / **0.13 / 0.17** | 82 / 79 / 55 / 47 | 0.05 / 0.03 / 0.04 / 0.03 | 3.70 | 0.48 | 0.2 |
| robot config, 256, reversed build order | 0.16 / 0.42 / 0.11 / 0.22 | 79 / 61 / 48 / 44 | | 3.05 | 0.40 | 0.7 |
| stripped rule (no press), 64 | 0.58 / 0.72 / 0.94 / 0.66 | 60 / 52 / 6 / 12 | | 0.99 | 0.23 | 0 |
| stripped rule (no press), 256 | 0.50 / 0.47 / 0.15 / 0.20 | 79 / 73 / 76 / 68 | | 3.18 | 0.42 | 0.2 |
| stripped rule, 128 | 0.47 / 0.53 / 0.24 / 0.49 | 53 / 38 / 40 / 26 | | 3.16 | 0.44 | 0 |

**Convergence, not servo strength** (the hinge motor's impulse cap applies per iteration, so 256
iterations also give 4× the authority; `HONEST_TORQUE_CAP` holds authority fixed in N·m): 256
iterations at the 64-iteration authority (9.6 N·m) still lift the rears (0.10 / 0.22); 64 iterations
at the 256-iteration authority (38.4 N·m) are identical to the base on every metric. At 64 the
loaded leg chain does not converge, so the joints of the most loaded legs give way instead of
tracking their lift. This is probably also the "rear femurs sag ~0.04 rad more under load" of the
2026-10-13 ledger. Cost: 143 s per 4500-tick run against 129 s (+11 %).

**But it moves the promoted stacks** (arena, n = 6 × 4500, 256 vs 64): promoted LPG stack (sim)
4.52 ± 0.78 vs 5.73 ± 0.41 m, straight 0.38 vs 0.54, 0 falls in both, rear mid-swing contact
0.04 / 0.05 vs 0.21 / 0.28, touchdowns about twice as many; benchmark 2.31 vs 2.80 m, straight 0.20
vs 0.27. Those stacks were tuned on the sagging body: they step more and go less far and less
straight. **The default stays 64 (verified byte-identical per tick) until the operator decides.**
256 is the more faithful setting: the real robot's servos hold, and its rear feet do not drag.

**Re-reading this session's yellow-leg and weight-shift results:** all ran at 64. The swing-load
gate (7.7), the press probe (7.8) and the learned weight-shift press (7.12) were judged against a
body whose loaded joints gave way. Their re-use context is "rerun at 256".

### 7.14 Re-baseline at the new sim defaults (2026-10-19)

256 solver iterations became the default on 2026-10-19 (operator: "that slower speed sounds like
what I see on the real robot"). Every launcher config was re-measured, each paired with
`OGMA_SOLVER_ITERATIONS=64` and both at 2 mm contact, with the standard harnesses (`arenaavg`,
`seedavg`, `roomavg`), n = 6, difficulty 0.3. The benchmarks and promoted configs ran headless on the
CAD body (as the record always has); the two measured-body Tegotae entries ran on `measured_fsr`.
Runs: `~/xaq_runs/rebase_20261019/`. Each cell is **256 / 64**.

**Arena, 4500 ticks**

| config | net_disp (m) | straight | falls | steps (proxy) | belly clearance (mm) | tilt_sd |
|---|---|---|---|---|---|---|
| P-e·h0 benchmark | 2.34 / 4.05 | 0.54 / 0.78 | 0 / 0 | 39 / 5 | 47 / 28 | 0.073 / 0.066 |
| + hip2 +0.3 benchmark | 2.82 / 3.33 | 0.63 / 0.71 | 0.17 / 0 | 21 / 2.5 | 65 / 52 | 0.059 / 0.065 |
| robot-stack benchmark | 2.70 / 3.31 | 0.61 / 0.69 | 0.33 / 0.17 | 21 / 5 | 65 / 51 | 0.058 / 0.080 |
| PROMOTED · LPG stack (sim) | 4.99 / 6.47 | 0.78 / 0.84 | 0.17 / 0 | 29 / 13 | 68 / 53 | 0.055 / 0.045 |
| PROMOTED · ROBOT · LPG + running-attitude gate | 5.14 / 5.63 | 0.83 / 0.86 | 0 / 0 | 69 / 56 | 65 / 49 | 0.064 / 0.089 |
| Tegotae bare rule, CAD | 5.32 / 5.20 | 0.82 / 0.84 | 0 / 0 | 13 / 0 | 43 / 31 | 0.087 / 0.031 |
| Tegotae bare rule, measured | **6.24 / 1.73** | **0.88 / 0.37** | 0 / 0 | 2 / 0 | 42 / 27 | 0.046 / 0.025 |
| Tegotae + stance press, measured | 3.78 / 4.49 | 0.72 / 0.75 | 0 / 0 | 84 / 0 | 68 / 23 | 0.091 / 0.068 |

**Corridor, 6000 ticks**

| config | net_z (m) | straight | falls | steps | tilt_sd |
|---|---|---|---|---|---|
| P-e·h0 benchmark | 3.13 / 4.64 | 0.54 / 0.74 | 0 / 0 | 91 / 35 | 0.073 / 0.069 |
| + hip2 +0.3 benchmark | 3.42 / 3.97 | 0.60 / 0.67 | 0.17 / 0 | 71 / 36 | 0.059 / 0.066 |
| robot-stack benchmark | 3.04 / 3.75 | 0.54 / 0.63 | 0.33 / 0.17 | 65 / 40 | 0.059 / 0.078 |
| PROMOTED · LPG stack (sim) | 4.08 / 4.03 | 0.60 / 0.63 | 0.17 / 0 | 96 / 47 | 0.061 / 0.062 |
| PROMOTED · ROBOT | 4.08 / 4.02 (std 0.11 / 0.56) | 0.66 / 0.66 | 0 / 0 | 82 / 66 | 0.164 / 0.149 |
| Tegotae bare rule, CAD | 4.45 / 2.83 | 0.64 / 0.54 | **0.67** / 0 | 29 / 26 | **0.325** / 0.117 |
| Tegotae bare rule, measured | 4.69 / 1.63 | 0.68 / 0.58 | 0.17 / 0 | 36 / 22 | 0.122 / 0.139 |
| Tegotae + stance press, measured | 3.79 / 4.17 | 0.66 / 0.62 | 0 / 0 | 85 / 47 | 0.189 / 0.151 |

**Room (3 m), 12 000 ticks** (the promoted configs still carry the GainEvolver, which switches
coupling on at tick 10 000 in both arms; most falls at 256 come *before* it: promoted robot 11 of
14, promoted sim 6 of 6, robot-stack benchmark 4 of 5)

| config | coverage (cells) | wall_frac | falls | net_disp |
|---|---|---|---|---|
| robot-stack benchmark | 33.0 / 39.0 | 0.020 / 0.055 | 0.83 / 0.17 | 0.90 / 0.95 |
| PROMOTED · LPG stack (sim) | 39.2 / 41.5 | 0.032 / 0.024 | 1.00 / 0.33 | 0.89 / 0.62 |
| PROMOTED · ROBOT | 44.2 / 50.3 | 0.123 / 0.142 | **2.33** / 0.33 | 0.71 / 1.16 |

**Readings.** On the stiffer body every config steps more (2–8× by the stride proxy) and carries
its belly 12–20 mm higher. In the open arena the benchmarks and promoted stacks travel 10–40 %
less and less straight. In the corridor the two promoted configs tie (the robot config's spread
falls fivefold, std 0.56 → 0.11) while the benchmarks lose about 0.6 m. In the room they **fall
more**: the promoted robot config 0.33 → 2.33 per 12 000 ticks. That is the new body's open
problem, and these stacks were tuned on the sagging one. The bare Tegotae rule on the measured body
goes from the worst entry to the best in the arena (1.73 → 6.24 m, straight 0.37 → 0.88), because
its rear feet now lift. The CAD bare rule now falls in the corridor (0.67, tilt_sd 0.33): with no
heading hold it climbs the side walls. **These are the baselines from 2026-10-19.** Numbers before
that date ran at 10 mm / 64 iterations.

### 7.15 Adaptation on terrain: the escape test, stance reach, the wall transition (2026-10-19/20)

Operator: the original stack, noisy and slow, adapts to terrain and gets itself unstuck from almost
anywhere in the corridor; the LPG stack "looks like a wind-up toy". Measured body, 2 mm / 256, the
GainEvolver removed, n = 6. New instruments: true chassis attitude (`cpitch`/`croll`, load log),
`terrain_scorecard.py` (per corridor zone), `escape_score.py` (dropped onto a feature at tick 1200,
3000 ticks to get off), `wall_score.py` (dropped on the floor facing a wall), and
`OGMA_PICRAWLER_TELEPORT_YAW_DEG` (drop heading). The chassis is solid in every run.

**Corridor scorecard** (9000 ticks). On the hump's climb (≈ 3.7°) the original pitches −0.053 (it
levels against part of the slope), the LPG robot config −0.074 (more than the slope, and it stays
nose-up on the descent and at 8° on the rumble strips), the LPG sim stack −0.005 (its planted-leg
corner integrator holds it level). Stance-posture change from the flat: original 0.038–0.040 rad,
LPG robot 0.017–0.028, LPG sim 0.059–0.073. The robot config lacks the corner integrator.

**Escape** (escaped / 6, median ticks): pyramid i0, i5 and the hump: every stack 6/6. **Pyramid i2**
(centre, ~20 cm): original 5/6 (2071, 3 tip-overs), **both LPG stacks 0/6**, high-centred (belly 0.66–0.77
BW, feet 0.03–0.14), the legs cycling a fixed shape in the air (knee sd 0.12–0.16 vs the original's
0.40–0.51). Side walls: LPG robot 4/6 per side with 0 tip-overs; original 1/6 and 3/6 with 10 tip-overs
in 12 drops; LPG sim 0/6 with 13.

**Lever: stance reach** (`MotorEPMv2.reach_load_topic`, `reach_rate`, `_target`, `_leak`, `_max`,
`_hip2`, `_ref_tau`). The Tegotae error applied to leg length: while a leg's phase says stance and
its foot's load, normalised by its own stance reference, is below half, its extension grows; it leaks
back when the foot bears weight. It reads `foot_load` directly, because shares sum to 1 even when
the belly carries the body. On the LPG robot config:

| arm | pyramid i2 | pyramid i0 | left wall | hump | flat arena: net / straight / falls / tilt_sd |
|---|---|---|---|---|---|
| original | 5/6 (2071, 3 falls) | 6/6 | 1/6 (6 falls) | 6/6 | 2.82 / 0.23 / 0 / 0.066 |
| LPG robot | **0/6** | 6/6 | 4/6 (0) | 6/6 | 4.46 / 0.56 / 0 / 0.061 |
| + reach 0.01 | **5/6 (1232, 1 fall)** | 5/6 (2) | 3/6 (4) | 6/6 | 4.05 / 0.49 / **0.67** / 0.193 |
| + reach 0.03 | 5/6 (14 falls) | 3/6 (9) | 3/6 (9) | 6/6 | 2.04 / 0.23 / 2.0 / 0.390 |

**Verdict: `PARTIAL` (signal) at 0.01, `REGRESSION` at 0.03.** It frees the high-centred robot
(0/6 → 5/6, faster than the original, fewer tip-overs) and stays near zero on flat ground on five
seeds (extension 0.00–0.04). On the sixth seed it **winds up** after a fall (extension 0.24–0.36 on all
legs, normalised load 0.25–0.51) and falls 4 times. The leak scales with the load felt, so a body
whose loads stay low never unwinds. To fix before any promotion: reset on the reset event, and a leak
floor independent of the load.

**Wall transition** (dropped on the floor 0.2–0.3 m from the wall's foot, facing it or at 45°, 3000
ticks): **no stack gets onto the wall** (|x| > 0.95). The LPG robot walks straight to the foot and
stays pressed against it (2400–2600 ticks at the foot, heading held, max |x| 0.82, rise 60 mm). The
original never reaches the foot in this window (heading wanders, max |x| 0.68). Stance reach does not
change it. The operator's "the original eventually makes it" needs a longer window to test, and the
LPG failure is at the foot: the front feet do not step up onto the slope.

### 7.16 Round 2: the stance-reach fix, the corner integrator on the robot config, the 5-minute wall test (2026-10-20)

Same protocol as 7.15 (measured body, 2 mm / 256, no GainEvolver, n = 6).

| arm (LPG robot config) | pyramid i2 | pyramid i0 | left wall | hump | flat: net / falls / tilt_sd |
|---|---|---|---|---|---|
| base | 0/6 | 6/6 | 4/6 (0 falls) | 6/6 | 4.46 / 0 / 0.061 |
| stance reach 0.01 (7.15) | 5/6 | 5/6 | 3/6 (4) | 6/6 | 4.05 / 0.67 / 0.193 |
| stance reach 0.01, reset + leak floor | 4/6 | 5/6 | 1/6 (7) | 6/6 | 3.82 / 0.83 / 0.190 |
| corner integrator (`level_int_gain` 0.005, planted only) | 3/6 | 4/6 | **0/6 (6)** | 6/6 | 3.33 / 0.33 / 0.167 |

**The wind-up fix did not remove the flat falls**, so wind-up was not their cause. On the falling
seeds the reach extension before the fall is small (0.01–0.13), and seed 1 falls at t ≈ 340 under
both levers (the base does not): each lever's small stance extension tips a marginally stable start.
Both levers: `PARTIAL` (they free the high-centred robot, 3–5/6 vs 0/6) at a cost in flat-ground
robustness and wall escapes. Neither is promotable as is.

**The 5-minute wall test** (15 000 ticks after the drop, facing the wall and at 45°): **no stack gets
onto the wall, at 256 or at 64 iterations.** Original: max |x| 0.74 / 0.74 (it never gets past the
wall's foot), 9 tip-overs head-on; at 64: 0.73 / 0.67. LPG base: 0.86 / 0.78, rise 77–88 mm, 1 tip-over;
at 64: 0.83 / 0.78. Pyramid i2 at 64 over 15 000 ticks: original 6/6 (median 1426, 14 tip-overs),
LPG 3/6 (median 10 465). The operator's "the original eventually makes the transition" is not
reproduced from these placements. The approach or placement where it was seen is needed.

**At the wall's foot the LPG robot** is pitched about 31° nose-up with its front feet on the slope
(touching the face 44–53 % of the time, toes 114–161 mm up). The rear pair carries the body (0.28–0.33
BW) and cannot lift: rear toes ~0 mm through swing, jammed in the floor–wall corner. This is the
weight-shift problem of 7.11–7.12 in its extreme form.

### 7.17 The pre-limits reference restored: P-e at 6.0 rad/s (2026-10-20)

Operator: every launcher config runs the robot's slow servo dynamics; in sim, performance for future or
modified robots matters more than the picrawler's exact sim2real; restore the best config from before
the limits. That config is **P-e at the sim's 6.0 rad/s**: `…native_measured__tofboom__fsrleg` (the
original gait mechanism: learned layer, knee-derived phase, Kuramoto coupling 1.509, stroke 1.2; FSR-leg
measured body; the sim's own inputs) with `MAX_SERVO_SPEED` 6.0 instead of the robot's 3.668. Restored as
`…__ref_servo6.json` (GainEvolver removed; it never acts inside 6000 ticks) and put first in the launcher.

**Reproduction** (`arenaavg`, n = 6 × 6000, arena 0.3): at the old 10 mm / 64 physics, **8.61 ± 0.57 m,
straight 0.87, 28.17 steps, tilt_sd 0.066, 0 falls — the 2026-10-01/02 record to the digit**. At today's 2 mm /
256: 7.06 ± 0.94 m, straight 0.75, 81 steps, tilt_sd 0.115, 0.17 falls.

**Against the stacks of 7.15–7.16** (measured body, 2 mm / 256, n = 6):

| test | reference (6.0 rad/s) | original (3.668) | LPG robot (3.668) |
|---|---|---|---|
| pyramid i2 escape | 5/6 (median 918, 4 falls) | 5/6 (2071, 3) | 0/6 |
| pyramid i0 | 5/6 (603) | 6/6 (624) | 6/6 (782) |
| left-wall escape | **6/6** (880, 2) | 1/6 (6) | 4/6 (0) |
| hump | 6/6 (**246**) | 6/6 (495) | 6/6 (462) |
| **wall transition, head-on, 15 000 ticks** | **3/6 onto the wall** (median 13 024 ticks, up to 188 mm) | 0/6 (59 mm) | 0/6 (88 mm) |
| wall transition, 45° | 1/6 (144 mm) | 0/6 | 0/6 |
| corridor: hump climb + descent (ticks) | 885 + 638 | 1560 + 3005 | 811 + 1520 |
| corridor: reached the rumble strips | **6/6** | 2/6 | 5/6 |
| corridor: posture change on the hump | 0.037–0.047 | 0.038–0.039 | 0.017–0.028 |
| corridor: belly clearance, mean / 5th pct (mm) | 51 / 35 (rumble 40 / **9**) | 67 / 54 | 68 / 61 |

**Reading.** The reference is the only stack that makes the floor-to-wall transition: 3 of 6 head-on,
"eventually" (median 4.3 minutes), as the operator remembered. It also escapes the walls best and crosses
the corridor fastest. Its posture adaptation is the original's, and its rhythm is the same mechanism. The
servo speed is the difference from the "original" of 7.15–7.16, which is the same brain at 3.668 rad/s.
It carries its belly lower (it scrapes on the rumble strips, 9 mm 5th percentile) and tips over more. **For
future or modified robots the question is now the mechanism at 6.0 rad/s**: the LPG stacks have never been
measured at full servo speed.

### 7.18 The LPG stacks at 6.0 rad/s (2026-10-20)

The same battery as the reference (7.17), every stack at `MAX_SERVO_SPEED` 6.0 (override checked in the
run logs), measured body, 2 mm / 256, n = 6. Flat arena with `arenaavg` (6000 ticks), the rest with the
Tegotae harness.

| test | reference (original gait) | LPG robot | LPG robot + stance reach | LPG sim stack |
|---|---|---|---|---|
| flat arena: net / straight / falls / steps | **7.06** / 0.75 / 0.17 / 81 | 6.44 / 0.83 / 0.33 / 94 | **6.63 / 0.84 / 0.17** / 102 | 5.96 / 0.74 / 0.33 / 38 |
| belly clearance, arena (mm) | — | 67 | 70 | 68 |
| pyramid i2 escape | 5/6 (918, 4 tip-overs) | 0/6 | **5/6 (1217, 0 tip-overs)** | 3/6 (2341, 5) |
| pyramid i0 | 5/6 | 6/6 | 6/6 (0) | 6/6 |
| left-wall escape | **6/6** (2) | 5/6 (0) | 4/6 (1) | 0/6 (11) |
| hump escape (median ticks) | 6/6 (**246**) | 6/6 (457) | 6/6 (412) | 6/6 (357) |
| **floor → wall, head-on, 15 000 ticks** | **3/6** (188 mm) | 1/6 (88 mm) | 0/6 (72 mm) | 0/6 (41 mm) |
| floor → wall, 45° | 1/6 | 0/6 | 0/6 | 0/6 |
| corridor: reached the rumble strips | 6/6 | 5/6 | 5/6 | 4/6 |
| corridor: posture change on the hump | 0.037–0.047 | 0.016–0.018 | 0.017–0.035 | 0.050–0.071 |
| corridor: belly 5th percentile, rumble (mm) | **9** | 43 | 39 | 38 |

**Readings.**
1. **On flat ground the servo speed was most of the gap.** At 6.0 rad/s the LPG robot with stance reach comes
   within 6 % of the reference's distance, goes straighter (0.84 vs 0.75), falls as rarely (0.17), and
   carries its belly higher. Stance reach's flat-ground cost at 3.668 (7.16) is gone here: 0.17 falls
   against the base's 0.33.
2. **On the tall pyramid stance reach matches the reference** (5/6) with no tip-overs (the reference has 4).
3. **The floor-to-wall transition is mechanism, not servo speed.** The reference climbs onto the wall 3/6
   head-on. The LPG stacks manage 0–1/6 even at 6.0 rad/s. The reference also escapes walls best and
   changes its posture on the hump 2–3× as much as the LPG robot configs. The LPG sim stack (with the
   corner integrator) adapts its posture as much but tips over on walls.
4. **Where the reference pays**: it scrapes its belly on the rumble strips (5th percentile 9 mm) and tips
   over more on the pyramid.

So the "wind-up toy" was half the slow servos (flat-ground performance) and half the mechanism (posture
adaptation and the wall transition). The next lever targets the second half: the rear legs jammed at the
wall's foot (7.16) with the weight on them.

### 7.19 The cadence sweep at 6.0 rad/s (2026-10-20)

Operator: was the step-speed parameter changed with the slow servos, and is it back? LoadPhaseGait's
`period_ticks` (70, from a July stride measurement) is a clock: the LPG stride stayed at 92 ticks (1.84 s)
at both 3.668 and 6.0 rad/s, while the reference's body-paced rhythm went from 45 ticks at 3.668 to 29 at
6.0. The 6.0 comparison of 7.18 held LPG at the slow cadence. Sweep: the LPG robot config + stance reach at
6.0 rad/s, `period_ticks` 70 / 50 / 40 / 30, measured body, 2 mm / 256, n = 6.

**Executed** (hip-1 oscillation, flat arena): stride 92 / 66 / 53 / 41 ticks (0.82 s at 30); stroke range
0.86 / 0.88 / 0.88 / 0.90 rad, so no slew clipping at any period.

| test | reference | p70 | p50 | p40 | **p30** |
|---|---|---|---|---|---|
| flat arena 6000 (Tegotae harness): net / straight / falls / tilt_sd | 6.14 / 0.31 / 0.17 / 0.107 | 6.19 / 0.54 / 0 / 0.063 | 6.88 / 0.50 / 0 / 0.089 | 7.47 / 0.45 / 0 / 0.104 | **8.95 / 0.46 / 0 / 0.095** |
| pyramid i2 escape (tip-overs) | 5/6 (4) | 5/6 (0) | 5/6 (2) | **6/6 (0)** | 5/6 (0) |
| pyramid i0 (median ticks) | 5/6 (603) | 6/6 (689) | 6/6 (782) | 6/6 (444) | 6/6 (478) |
| left-wall escape (tip-overs) | 6/6 (2) | 4/6 (1) | 5/6 (0) | 5/6 (1) | **6/6 (0)** |
| hump escape (median ticks) | 246 | 412 | 359 | 298 | **254** |
| floor → wall head-on, 15 000 ticks | **3/6** (188 mm) | 0/6 (72) | 0/6 (68) | 0/6 (84) | 0/6 (90) |
| floor → wall 45° | 1/6 | 0/6 | 0/6 | 0/6 | 0/6 |
| corridor: hump climb + descent (ticks) | 885 + 638 | 858 + 2504 | 897 + 1718 | 1218 + 2267 | **419 + 650** |
| corridor: reached the rumble strips | 6/6 | 5/6 | 5/6 | 4/6 | 6/6 |
| corridor: ticks on the rumble strips | **3534** | 3274 | 3855 | 5288 | 5066 |
| corridor: posture change on the hump | **0.037–0.047** | 0.017–0.035 | 0.022–0.032 | 0.016–0.031 | 0.021–0.025 |
| corridor: belly 5th pct on the rumble (mm) | 9 | 39 | 49 | 47 | 44 |

**Readings.** The cadence was the rest of the flat-ground gap and more. At period 30, LPG + stance reach covers
46 % more ground than the reference with no falls, escapes the pyramid and the wall with no tip-overs, and
crosses the hump faster, with the belly 35 mm higher over the rumble strips. Transport rises monotonically with
cadence. **What it still does not do is the reference's terrain adaptation**: it never climbs from the floor
onto the wall at any period (rise ≤ 90 mm vs 188), its posture changes half as much on the hump, and it is
slower over the rumble strips. Those belong to the mechanism, not to speed or cadence. (Scorer fix: both
escape and wall scorers now read the drop tick from the body's teleport line; two seeds that tipped before the
drop had been misread. Rescored: only the stance-reach 0.03 arm of 7.15 changed, and its `REGRESSION` stands.)

## 8. What it takes, as of 2026-10-18

| Owaki & Ishiguro result | On this body, sim | What it needs |
|---|---|---|
| R1 gait from random starts | **reproduced** (signal): trot from 6/6 random starts, R ≈ 1 | — |
| R2 no load term, no gait | **reproduced** | — |
| tonic beats phasic sensing (their Discussion) | **reproduced**: contact-count shares lose the gait | force magnitude from the FSRs, not just the contact flag |
| R3 ω moves the gait | **not reproduced**: trot at every ω | (a) a foot whose load falls in late stance, to open the held regime where their walk lives; (b) actuator speed for the short periods. Neither is a control-law change |
| R4 hysteresis | not testable until there are two gaits | — |
| R5 efficiency | not run (one gait) | the power model in the log |
| R6 weight distribution | **partial**: the oscillators re-organise, transport does not survive | — |

The rule needs no joint sensing, and the robot has the sensor it does need. Two physical
properties stand between this body and their walk:

1. **The late-stance load.** On the sprawled picrawler a stance foot carries most weight just
   before liftoff, which caps the usable hold at σ_r ≈ 3. Owaki's walk runs at about 5. Cheap
   tests, in order: a narrower stance (feet nearer the body line, if the hip-2/knee geometry
   reaches it); a hip-1 stroke that moves the foot fore-aft along the body rather than around
   a vertical axis; a larger late-stance self-unload than `leg_phase_lift_lead` gives.
   `sigma_crit` from any run says straight away whether a change moved the ceiling.
2. **Servo slew.** 3.67 rad/s caps A·ω, so speed cannot rise with ω, and short periods clip the
   swing lift until the rear feet stop lifting.

**For the deployed stack (E7), the cheap lever this suggests:** a stroke inside the slew cap.
At the deployed period, 0.53 instead of 0.7 raised the stripped rule's clean trot strides from
42 % to 59 % at equal transport. On the stack this is one gain-0-guarded arm, to run next. The
stack's own oscillators already trot. Its feet do not (97 % irregular), and the terms the
stripped mode removes are where to look.

## 9. Next

0. Rerun the core arms (stripped σ_r 2 and 3, free clock, lift dose) on `measured_fsr` and
   default the harness to the launcher's body, so the UI and the batches agree.
1. E7: the slew-matched stroke (0.53) as one lever on `…__lpg_cand` and on the promoted
   `…__lpg_sl4s_st07_int`, arena + corridor + room, n = 6.
2. Attribution on the stack: which zeroed term breaks the touchdown trot (one term back at a
   time onto the stripped mode). Same lesion method as the ledger's 2026-10 knee study.
3. The late-stance load: a stance-geometry probe (narrower stance, fore-aft stroke). Judge
   each on `sigma_crit` first, and look for a walk pattern only once the ceiling moves past 5.
4. E6, the (d) test on the trot: a leg lesion and a slick foot mid-run, time to re-lock.
5. Long runs (≥ 20 000 ticks) once the GainEvolver task is settled.
