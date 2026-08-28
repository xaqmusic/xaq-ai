> **LIVING DOC — the single reference for the PiCrawler hardware port.** Started 2026-08-10, the
> day the physical robot reached the bench. Measurements live in
> [`../operational/picrawler_geometry.md`](../operational/picrawler_geometry.md)
> §"Measured from the built robot"; this doc is the *plan* that consumes them. Update it in
> place — do not fork a second port doc.

# PiCrawler sim2real port

## Context

The physical PiCrawler is **built**. Tape-measure readings differ materially from the STEP/CAD
numbers the sim has used since inception, and the deltas are not cosmetic: they **reopen a
question the ledger had closed**, and they make an **already-diagnosed gait pathology worse**.

Current scope is **geometry additions and the servo safety envelope only.** Later phases are
recorded so the path stays visible, but are expected to be refined as the early ones land.

**⚠ Sprawl work is OUT OF SCOPE here** — a sprawl fix is in flight in a separate session. This
plan holds at that boundary and reassesses when the PR lands (§Coordination).

---

## Coordination with the in-flight sprawl PR

Three couplings. All manageable; none should be discovered at merge time.

**1. Same file, large surface.** Phase 0 touches geometry constants throughout
`picrawler_body.gd` (10 741 lines); the sprawl PR touches gait paths in the same file. Phase 0
stays **mechanical and confined** to the constants block plus their use sites — no opportunistic
cleanup — so the diff stays reviewable against a moving base.

**2. `θ_tibia` is a shared dependency.** The plumb reflex uses
`θ_tibia = 1.40·x[hip2] + x[knee] − 0.0292`, derived from `HIP2_LIMIT`/`KNEE_REST` and validated
against the **CAD rest pose**. If the sprawl PR touches `tibia_plumb_gain` it inherits a formula
the geometry correction invalidates. Flag it to that session now.

**3. ⚠ The default-body flip would re-baseline their work underneath them.** Measured geometry
changes `L1`–`L3`, which feed the FK chain behind `feet_y_gravity_cmd` — the promoted swing-gate
input. Every metric moves. **The flip is therefore split out and held** (Phase 1b): build the
capability, validate it, leave `cad` default until the sprawl PR lands.

Corollary, stated plainly: **a sprawl result validated on CAD geometry does not automatically
transfer to the measured body.** Per ledger §3.1 that verdict is scoped to the geometry it was
measured on and needs re-checking after the flip. Not a criticism of that work — its re-use
context.

---

## The body of record

Reference plane: **hip2 axis, y = 0.** Operator-measured 2026-08-10 unless noted.

| Quantity | Value |
|---|---|
| `L1` hip2→hip1 | 32.0 mm |
| `L2` hip2→knee (femur) | 48.0 mm |
| `L3` knee→toe (tibia) | 76.5 mm |
| hip1 pattern | **square, 76 mm side** → anchors (±38, ±38) mm |
| base box | 98 × 98 × 41 mm, spans **−19 → +22 mm** (underside = belly) |
| electronics box | 90 × 56 × 62 mm, spans +22 → +84 mm |
| total chassis height | 103 mm (wires in, no shell) |
| total mass | 590 g |
| per-leg mass | 62 g (× 4 = 248 g) |
| body mass | 342 g → **base 252 g / electronics 90 g** (solved from CoG) |
| CoG | +15 mm above hip2, chassis centre |
| knee mobility | **full 180°** — folds into the leg structure |
| min chassis clearance | **9 mm** — knee folded tight, hip2 vertical ⇒ toe at **−28 mm** |

### ⚠ The leg-naming mirror — MUST be resolved at the servo map

**Operator-diagnosed 2026-08-11 on the piano roll** (the first instrument to
put per-leg motor traces beside the 3-D view): the sim body's internal leg
names are anatomically **swapped left↔right**. True forward is +Z (eyes /
corridor / fwd_v), so body-left = +X — yet leg 0 "fl" is built at x<0, the
anatomical FRONT-RIGHT (names were assigned in the default-camera screen
frame: the mirror illusion of labelling a body that faces you). The frame is
used *consistently* by every action topic, config, event, instrument, and
per-leg finding — "fl" = red = anatomical FR throughout history — so the sim
record is coherent and the mirror is behaviorally null **in sim**.

**At the port it is not null.** The servo map (Phase 4) must pin each
`action.<leg>_<joint>` topic to a PHYSICAL servo channel; mapping "fl" to the
real robot's front-left servo would mirror the learned gait across the body.
The map must be written **by anatomy, not by name**: `action.fl_*` → the
real FRONT-RIGHT leg's servos (and likewise fr→FL, rl→RR, rr→RL), with the
calibration colors as the cross-check (red leg = the one "fl" drives).
Decide at that point whether to do the full rename (topics + configs +
ledger annotation) or keep the body frame and carry this mapping note — but
never both halves of each.

### Validation gates — checks, not A/Bs

Constraints the measurements *impose*, independently checkable in sim. **If one fails, the model
is wrong, not the tape measure.**

| Gate | Constraint |
|---|---|
| **G1 — CoG** | `_body_cog()` (`picrawler_body.gd:8713`) reports **+15 mm above hip2** at standing pose. Currently assumes a single 300 g chassis body; the 252/90 split must reproduce the measurement *by construction* |
| **G2 — crouch reach** | FK reaches a pose with the **toe 28 mm below hip2** (the 9 mm clearance posture). If joint limits forbid it, the limits are wrong |
| **G3 — knee travel** | Full **180°** fold, no linkage interpenetration in the collision model |
| **G4 — gain-0** | With `cad.json`, `seedavg.py`/`arenaavg.py` output **byte-identical** to pre-refactor |

---

## Phase 0 — Geometry as swappable data · ✅ **DONE 2026-08-10 (G4 PASS)**

**Landed:** `addons/ami_ogma/body/cad.json` + `_load_geometry()` / `_apply_geometry()` /
`_recompute_derived_geometry()` / `_rebuild_body()` in `picrawler_body.gd`, and a `[B]` hotkey
that toggles cad ↔ measured mid-run.

**G4 verified twice** — once after the extraction, once after `_rebuild_body()` landed — via
`seedavg.py` n=3 × 3000 ticks against the stashed pre-refactor build: **all 40 metrics
byte-identical**, including the per-leg TLE/amp tables.

Two things worth carrying forward:

- `const`→`var` needed **no use-site edits at all** — the identifiers are unchanged, and the
  audit confirmed no geometry constant appears in a function default parameter, a `const`
  expression (except `_TOTAL_MASS`), or another file. ~130 use sites, zero touched.
- `OGMA_PICRAWLER_BODY=<name|res://path>` selects the body without editing the scene, so the
  harnesses can A/B bodies per-arm.

<details><summary>Original Phase 0 spec</summary>

Make the body a data file with no behavioral change. Gain-0-guarded and provable.

- Extract to `addons/ami_ogma/body/cad.json`: link lengths (`L1`–`L3`, `COXA_Z_DROP`), hip anchors
  (`HIP_X_SPAN`/`HIP_Z_SPAN`), chassis dims, masses, `STANDING_CHASSIS_Y`, rest poses, joint limits.
- These are `const` today → become `var`s populated before `_build_body()` (`:4029`) /
  `_build_leg()` (`:4115`).
- **Audit findings (complete).** Scope is fully contained in `picrawler_body.gd` — no cross-file
  references, no scene overrides. ~130 use sites across 27 constants. **Three init-time hazards,
  and only three:**
  - `:1330` `const _TOTAL_MASS = CHASSIS_MASS + 4.0*(...)` — **const folded from consts**, the
    exact silent-stale-value trap. Must become a computed var.
  - `:1555` `@export var target_height = STANDING_CHASSIS_Y`
  - `:1583` `@export var peak_height = STANDING_CHASSIS_Y`
  - (`KNEE_RANGE_FOLD`/`HYPEREXT` at `:180-181` reference `KNEE_REST` only in *comments* — the
    values are literal. No hazard, but the comment arithmetic goes stale.)
- `@export` defaults stay literal `0.082` (byte-identical), with geometry-follow applied in
  `_load_geometry()` **only when still at that default** — so env
  (`OGMA_PICRAWLER_TARGET_HEIGHT`, `:3039`) and curriculum
  (`ExperimentConfig.resolve_picrawler_target_height`, `:3196`) overrides still win.
- Add `_rebuild_body(geometry)`: reconstruct segments + joints preserving chassis pose/velocity
  **without resetting the brain**. Follow the existing live-teardown precedent at `:3823–3839`
  (`_world_root.queue_free()` → `_rebuild_world_contents()`), already used for the `[1]`/`[2]`
  gym swap.
- Reuse the existing calibration schema — `picrawler_servo_panel.gd` already exports
  sign/origin/limits as JSON. **Extend it, don't invent one**, so calibration exported from the
  real robot drops straight into sim.

**Gate: G4.** Byte-identical with `cad.json`, or the refactor is not done.
</details>

## Phase 1a — Add the measured body, opt-in · **IN PROGRESS**

**Landed:** `body/measured.json`, optional multi-box chassis + explicit `center_of_mass`
(`CENTER_OF_MASS_MODE_CUSTOM`), `_compute_body_cog_y()` reading the real CoM, and
`_report_geometry()` — a per-build receipt printing the G1–G3 evidence into every run's log.

| Gate | Status |
|---|---|
| **G1 CoG** | ✅ **PASS — +15.0 mm rel hip2 exactly** (world y 0.0903) |
| **G2 belly offset** | ✅ −19.0 mm rel hip2 exactly; 56.3 mm above floor at spawn |
| **G2 crouch reach** | ⬜ **not yet checked** — needs FK to reach toe −28 mm (the 9 mm posture) |
| **G3 knee 180° fold** | ⬜ **not yet checked** — receipt confirms the *commanded* span is still 241°, the known fiction (limits deliberately unchanged; Phase 2) |
| **G4 gain-0** | ✅ PASS, re-verified 4× as the code changed |

⚠ **The +15 mm turned out to be the WHOLE-ROBOT CoG, not the chassis's** — the operator measured
it by balancing the assembled robot. The chassis CoM is therefore back-solved to **+34.1 mm rel
hip2**, giving **base ≈ 126 g / electronics ≈ 217 g** — the battery is in the **top stack**,
matching CAD's "Top plate / battery" label. The earlier 252/90 split in this doc was wrong and
has been corrected. Full derivation + sensitivity check in `picrawler_geometry.md`.

<details><summary>Original Phase 1a spec</summary>

**One design decision to settle first: how the two-box chassis is represented.**
A `RigidBody3D` has a single mass, and `_compute_body_cog_y()` currently takes
`CHASSIS_MASS × _chassis.global_transform.origin.y` — i.e. it assumes the chassis CoG sits at
its node origin. That assumption breaks the moment the chassis has an internal mass
distribution. Recommended: **one rigid body, two `BoxShape3D` children at their offsets, total
mass 0.342 kg, and an explicit `center_of_mass` (`CENTER_OF_MASS_MODE_CUSTOM`)** placed to
satisfy the measured +15 mm — with `_compute_body_cog_y()` updated to read the real CoM rather
than the origin. Two separate rigid bodies would need a joint and would add solver cost for no
physical gain on a rigid chassis.

Schema plan: `chassis.boxes` is **optional**; absent ⇒ the existing single-box path runs
untouched, so `cad.json` stays byte-identical without a code-path change.
</details>

- `body/measured.json`; two-box chassis collision replacing the single chassis body; mass split
  252/90.
- Run **G1–G3**.
- Ships **opt-in**; `cad` remains default, so no existing config moves and the concurrent sprawl
  session is undisturbed.

## Phase 1b — Flip the default · **HELD** (blocked on the sprawl PR)

- A/B `cad` vs `measured` as its own lever, **alone**, in the **arena** (`arenaavg.py` — the
  corridor's self-centering 30° walls mask this family), `OGMA_PICRAWLER_CHASSIS_COLLIDE=1`, n ≥ 6.
- Record the new baseline in the ledger. **Every ledger number re-baselines here.** A
  post-correction regression is **not** a verdict on any earlier lever.
- Re-derive `θ_tibia` against the new rest pose before anything consumes it.
- `measured` becomes default; `cad` stays selectable for reproducing historical results.
- Sim-side servo-envelope mirror (Phase 2) lands here, so gait behavior isn't touched while the
  sprawl PR is open.

## Phase 2 — Servo safety envelope · **ACTIVE** (independent of the sprawl PR)

Two nested envelopes; the **inner** one strips gears. Servo travel (~180°) merely clamps at the
controller. **Linkage hard stops** make the servo drive in at full torque and stall.

- **Derive the granted range from measured occupancy, not from the limits.** The deployed gait
  occupies `hip2` **29°** (−23…+6) and `knee` **113°** (−98…+15) — both far inside 180°. Grant
  occupancy plus margin, not the **232°** the brain is currently allowed to command
  (`KNEE_RANGE_FOLD 3.20` / `KNEE_RANGE_HYPEREXT 0.85` about `KNEE_REST −1.6`).
- **Per-servo protocol** — robot on a stand, legs unloaded, **one servo at a time**: horn off →
  1500 µs → mount horn at reference pose (spline granularity 14.4–18°/tooth leaves ±7–9° residual
  → software `origin`) → sweep outward ~2°/step with dwell to the linkage stop → back off ~5° →
  record.
- **Stall detection works during calibration specifically.** A stalled MG90S pulls ~1 A; on a 2S
  pack (~100 mΩ ESR) that is ~100 mV sag against A4's 2.4 mV LSB ≈ **40 counts**. Useless during
  gait (everything moves at once), sufficient one-servo-at-a-time.
- **Clamp in the driver, below the brain** — a limit the brain cannot route around.
- Runtime protections that matter **more** than the static limits:
  - **slew limiting** — legs are 42 % of body mass; a 50 Hz step command slams the gear train
  - **watchdog → safe pose.** A servo holds its last pulse indefinitely; going limp requires
    setting the channel pulse to **0** so pulses stop entirely
  - **per-servo time-at-limit accumulator** — a sustained stall against carpet is invisible
    without current sensing and will cook a servo quietly

## SPEC — belly clearance without a rangefinder: IK estimator + one touch switch

> **STATUS: SPEC ONLY. NOTHING BUILT. Blocked on the sprawl PR** (same file, and the FK chain it
> shares). Written 2026-08-10 so the design is settled before the window opens.

### Why

The belly ToF rangefinder is the sensor CLAUDE.md names as the hump breakthrough — *"a new
sensory channel"* that replaced the god's-eye `chassis_y_norm`. **The operator does not have one
yet.** A VL53L0X/VL53L1X is a few dollars and speaks I²C (`0x29` is free alongside the MCU at
`0x14` and the IMU at `0x68`/`0x69`), and **is expected to be acquired** — but it is worth knowing
how much of the result survives on parts already in hand, and the simplest thing that could work
deserves a real test before more hardware is added.

### The two halves, and why they are not a rangefinder

**Both are §5.1-legal.** FK from *commanded* angles is the already-promoted path
(`feet_y_gravity_cmd`); the sensor-legitimacy doc explicitly contemplates contact switches
(*"adding FSRs or microswitches per foot is cheap if we ever want it"*). A belly switch is the
same class — a real egocentric contact sensor.

**A. `belly_clearance_ik` — the estimator.**
1. FK from the 12 commanded angles → foot positions in the chassis frame (existing chain).
2. Rotate into a gravity-aligned frame using IMU attitude (accel + gyro fusion).
3. Ground plane ≈ the lowest **planted** foot. ⚠ Hardware has **no foot contact sensors**, so
   "planted" must be inferred — reuse the existing stance test (`stance_y_threshold`, 0.04), do
   not invent a second one.
4. Belly height = **minimum over the four bottom-plate corners**, not the plate centre. Under
   tilt the robot grounds on a corner, and a centre-only estimate is optimistic exactly when it
   matters.
5. Publish normalised on the existing scale (`GROUND_CLEARANCE_RANGE` 0.3, `GROUND_CLEARANCE_STAND`
   0.06) so it is drop-in comparable with `gc_norm`.

**Blind spot, and it is the whole point:** IK gives height above the **feet**, not above whatever
is under the **belly**. Identical on flat ground; divergent on a rise — which is precisely the
hump case the rangefinder was introduced to solve.

**B. One belly touch switch.**
- **Placement is a measurement, not a guess.** Before committing, measure the *distribution of
  bottom-plate contact points* in sim and put the switch where grounding actually happens.
  Priors: plate centre is the unbiased choice but is the LAST point to touch under tilt;
  front-centre is the informed bet, since the dominant grounding mode is nose-down during
  forward locomotion and hump approach. Let the measurement decide.
- Microswitch to a Robot HAT digital pin (D0–D3 = GPIO17/4/27/22), internal pull-up, switch to
  ground. Bounce is ~1–5 ms, sub-tick at 50 Hz; a 2-tick software debounce suffices.
- Model it honestly in sim: contact on the base box + debounce + latency, per the §5.4 pattern.

### The fusion — the switch is a PREDICTION ERROR, not a sensor reading

Do not consume the switch as another input channel. The estimator *predicts*; the switch
*observes*; the residual is the signal:

```
predicted_contact = (belly_clearance_ik < ε)
belly_error       = switch_state − predicted_contact
```

- **switch = 1, IK says clear ⇒ the ground under the belly is HIGHER than the plane under the
  feet.** That is the terrain signal, recovered as a residual instead of measured directly.
- switch = 0, IK says grounded ⇒ feet on a rise / belly over a gap, or FK error.

Magnitude is lost (binary); occurrence is kept. **Occurrence may be enough:** the promoted
`height_ground_gain` lever already consumes a near-binary grounding signal — the ledger reports
it as `%<10 mm`, a threshold on `gc_raw`. A switch is that same threshold at 0 mm, so that lever
is the one most likely to port unchanged.

### ⚠ Do the authority check FIRST — ten minutes, and it gates the build

Per the ledger's own rule (*"before building a lever, measure whether the actuator has AUTHORITY
over the target"*): compute **`corr(belly_clearance_ik, gc_raw)` on existing traces.**

- Correlates well on flat ⇒ the FK + attitude chain is sound, substitution is worth building.
- **Does not correlate even on flat ⇒ stop.** That is a broken FK/attitude chain, and no amount
  of switch fusion repairs it. Three levers were burned in one session for want of this check.

### A/B plan

Three arms, one base config, **arena** (`arenaavg.py`) + the **corridor hump gate**,
`CHASSIS_COLLIDE=1`, n ≥ 6:

| arm | channel |
|---|---|
| incumbent | `gc_raw` (simulated ToF) |
| candidate | `belly_clearance_ik` + switch residual |
| ablation | neither — confirms the channel matters at all |

Ships gain-0: new channels default OFF, switch published instrument-only first.

**Blind-metric guard (§3 rule 4):** a robot that simply stands very tall never grounds, so
`bellyc` looks excellent while transport dies. Always read it with `net_disp` / `straight`, and
watch `tilt_sd` — the ledger records that clearance bought by wobbling is the degenerate win here.

**Prediction, recorded before running:** IK + switch ≈ ToF on flat ground, and **loses on the
hump gate**, because anticipation is the difference — the rangefinder senses a rise *before*
contact, touch + IK only reacts *at* contact. Recording this now makes either outcome diagnostic
rather than a surprise.

### Upgrade paths, in order

1. **Four switches** (one per bottom-plate quadrant) — upgrades "am I grounded" to "which corner",
   which is directionally actionable. Almost free. Deliberately NOT in this spec: start simplest.
2. **FSR pads** instead of microswitches — graded contact force rather than a bit. ⚠ **Overtaken
   for the FEET by the foot-FSR spec below** (ordered 2026-08-27); this path now refers only to a
   belly pad.
3. **VL53L0X/VL53L1X ToF** — the actual channel, on the I²C header already established. The
   direct fix if the hump gate regresses.

## Phase 3 — Sprawl · **DEFERRED** — reassess when the PR lands

Not scoped here. What to reassess *with*, recorded so it isn't lost:

**The geometry correction reopens a family the ledger closed.** "Get the tibia vertical" was shut
down as a kinematic dead end — *"the leg is not long enough for the foot placement the gait
uses"* — with an explicit re-use context: **"a body with a longer tibia or a shorter femur."**
The measured robot is exactly that:

| | CAD | Measured | Direction |
|---|---|---|---|
| femur `L2` | 53.6 mm | **48.0 mm** | shorter ✓ |
| tibia `L3` | 75.5 mm | **76.5 mm** | longer ✓ |
| femur/tibia ratio | 0.710 | **0.627** | −12 % ✓ |

`tibia_plumb_gain +0.15` sits at `IN_FLIGHT` with the largest single effect measured in its
session (+32 % distance, `straight` 0.82 ± 0.00), blocked only by a −29 % belly-clearance
regression — belly-up being a promoted invariant.

**Counter-pressure — do not assume the favorable direction.** Total reach drops
166.4 → **156.5 mm** while the gait plants at a measured **170 mm** radius. Overshoot worsens
+3.6 → **+13.5 mm**, and some current foot placements become geometrically unreachable. The
improved ratio and the shortened reach pull opposite ways; only the A/B decides.

**Hardware gate when this resumes:** `foot_r` must come inside **156.5 mm** with margin. That is
the concrete pass/fail for "safe to put on hardware."

## Phase 4 — Hardware host · recorded, not scheduled

### The bus map, as of 2026-08-27 (parts ordered)

> **Parts, connectors, power path and bring-up order are in
> [`../operational/picrawler_sensor_wiring_and_bom.md`](../operational/picrawler_sensor_wiring_and_bom.md).**
> This section stays the *why*; that file is the bench reference.

| device | bus | address / pins | carries |
|---|---|---|---|
| Robot HAT MCU | I²C | `0x14` | 12 servo channels + ADC A0–A4 |
| **ICM-20948** IMU | I²C **or SPI** | `0x69` (AD0 high) / `0x68` | attitude, yaw rate — SPI preferred, see below |
| **INA219** | I²C | `0x40` default | servo-rail bus current → the energy term |
| **VL53L0X** belly ToF | I²C | `0x29` | belly clearance |
| **4 × FSR** | HAT ADC | A0–A3 | `foot_load` + the G2 guard |
| battery voltage | HAT ADC | **A4 — keep it** | brownout + calibration stall detection |

No address conflicts. If the IMU moves to SPI the I²C bus carries only the MCU, INA219 and ToF,
which is the reason to prefer it.

- **C++ host linking `ogma_core` directly** (already Godot-free — the gtest suite proves it).
  Rationale is **sim/real parity**: sensor derivation must be the *same code*, or every
  discrepancy is unattributable between body and reimplementation.
- Take SunFounder's **wire protocol, not their library**: MCU `0x14`, one 3-byte txn
  `[reg, hi, lo]`, `REG_CHN = 0x20 + channel`, `REG_PSC/ARR = 0x40/0x44 + timer`,
  `timer = channel // 4`, 72 MHz clock, 50 Hz, period 4095, 500–2500 µs.
  **Do not depend on the `picrawler` library** — it is scripted gaits (`do_action('forward')`),
  an imposed coordination topology, prohibition §7.
- **ICM-20948, 6-axis in use** (ordered 2026-08-27, superseding the MPU-9150 this plan named
  until then — that part is long EOL and what ships is old stock or counterfeit). Same price
  class, better bias behaviour, and **it can leave the contended bus entirely** (see SPI below).
  1.71–3.6 V, so the HAT's 3.3 V rail is fine.
  - **Address:** `0x69` on most breakouts (AD0 high), `0x68` with AD0 low. **Both are free** —
    MCU `0x14`, ToF `0x29`, INA219 `0x40`, no RTC, no onboard IMU on HAT V4. Read AD0 off the
    board rather than assuming; `WHO_AM_I` (`0x00`) returns **`0xEA`**, which is the bring-up
    check that the right part answered.
  - ⚠ **The register map is BANKED — this is the real porting difference from the MPU-6050/9150
    family, whose map is flat.** Four banks selected through `REG_BANK_SEL` (`0x7F`); a driver
    written from MPU-family habit reads plausible garbage from the wrong bank. Since Phase 4
    takes the wire protocol rather than a vendor library, budget for this explicitly.
  - **If on I²C:** set `i2c_arm_baudrate=400000` (the part does fast-mode); at 100 kHz the
    servo + IMU traffic exceeds the tick budget. **FIFO reads are mandatory** — the bus is shared
    with all 12 servo writes, so host-side jitter integrates directly into dead-reckoned yaw.
  - **★ SPI is the better option and the MPU-9150 could not offer it** (that part is I²C-only).
    The ICM-20948 speaks SPI up to ~7 MHz, which takes the IMU **off the servo-contended I²C bus
    altogether** and removes the jitter concern at its source rather than filtering it after the
    fact. Confirm the breakout exposes CS/SCK/SDI/SDO before committing. Keep the FIFO regardless
    — it is what makes the 50 Hz decimation honest.
  - **Magnetometer (AK09916) stays disabled.** 12 servos ≈ 7 µT each at 3 cm against a 50 µT
    earth field, varying with gait. On this part it sits behind the internal I²C-master aux bus,
    so the cheapest correct action is simply **never to enable that master** — the mag costs
    nothing as long as it is not turned on.
  - ⚠ **Do not use the DMP.** The on-chip DMP3 needs an undocumented firmware blob and returns a
    fused attitude from a filter that cannot be inspected, tuned for non-legged motion. That is a
    teacher whose internals we cannot audit (prohibition §6), and it would destroy the design's
    load-bearing quantity: **the FK/IMU disagreement IS the slip signal**, which requires the raw
    channels and our own fusion. Take raw accel + gyro from the FIFO and nothing else.
- Gyro **bias estimator gated on quasi-static windows**, not a hardcoded constant (prohibition
  §5). The ICM-20948's untrimmed ZRO is roughly **±5 °/s** against the MPU-9150's ±20 °/s
  (confirm against the datasheet of the board that arrives) — but **initial offset is close to
  irrelevant, because the estimator subtracts it either way.** What the newer part actually buys
  is **in-run bias stability and a smaller temperature coefficient**, which matters because the
  servos heat the board: it lengthens the interval the estimate stays good between quasi-static
  windows. Real, and second-order.
- ⚠ **The IMU is not the limit, and buying a better one does not move it.** The binding
  constraint is **observability**: on a walking robot the accelerometer cannot separate tilt from
  body acceleration, so its attitude correction is corrupted exactly during stance transitions
  (ledger 2026-08-24 ★4). A 1° attitude error leaks false velocity exceeding the true 0.05–0.15
  m/s signal within one second, as **bias** and **gait-synchronously**, so it does not average
  down. No gyro grade fixes that. The fix is the one the ledger already names (★5): bound the
  drift with the **leg-FK velocity observation**, whose error profile is exactly complementary.
- **Placement beats sophistication for any SECOND unit.** Per legitimacy §5.5, the high-value
  second IMU is **on a leg segment**, not a second body unit — hobby servos deny per-leg position
  feedback, and a leg-mounted IMU recovers that limb's own motion directly. That targets the
  commanded-vs-achieved gap the ledger calls the *dominant* odometry error (22 mm mean foot
  deflection, 20× the FK error). ⚠ **Gated on a free measurement that has never been run:** the
  **horizontal** within-stance FK error (ledger 2026-08-24 ★4). Both FK variants already run every
  tick through the same chain with the miswiring control in place. Run that before buying against
  the requirement.
- Loop locked to **exactly 50 Hz** (`clock_nanosleep(TIMER_ABSTIME)`, `SCHED_FIFO`), overruns
  counted as a logged metric.
- **Belly ToF rangefinder — to be acquired.** VL53L0X / VL53L1X, a few dollars, I²C at `0x29`
  (free alongside MCU `0x14`, the IMU `0x68`/`0x69` and INA219 `0x40`) on the header already
  established. This is the
  sensor CLAUDE.md names as the hump breakthrough; the sim channel (`gc_raw`/`gc_norm`,
  `GROUND_CLEARANCE_RANGE` 0.3 m) already exists and assumes it. Until it arrives, see the
  IK-estimator + touch-switch spec above — and note that spec's recorded prediction is that the
  substitute **loses on the hump gate**, since it cannot anticipate a rise before contact.
- ⚠ **No current sensing exists.** Robot HAT V4 ADC is A0–A3 user + **A4 = battery *voltage***
  (20K/10K divider, `Vbat = A4/4095 × 3.3 × 3`). The sensor-legitimacy doc's §5.3 claim that a
  Robot HAT "typically exposes bus current" is **wrong for V4**. Real servo-current work needs an
  **INA219** (bus) or **INA3221** (3ch) on the same I²C header.
- `robot_hat` in Python for one "does servo 3 move" smoke test, then discarded.

## SPEC — foot FSRs: conditioning, mounting, calibration

> **STATUS: SPEC ONLY. NOTHING BUILT.** Written 2026-08-27 as the parts were ordered, so the
> conditioning decisions are settled before assembly. Supersedes the four-microswitch upgrade
> path in the belly-clearance spec above: **an FSR above threshold IS a contact switch and also
> carries load magnitude, so do not spend GPIO on both** (ledger, 2026-08-24).

Four circular FSRs, **20 g – 2 kg**, one per foot, on the Robot HAT's existing ADC **A0–A3**.
No new bus. **A4 stays on battery voltage** — it is the brownout and calibration-stall detector.

### The operating point — and why 20 g – 2 kg is the right part

`foot_load` is published as a **fraction of total body weight**, not in newtons:
`picrawler_body.gd:6768` divides the per-foot GRF EMA by `_fl_norm()` (`:10006`,
`_TOTAL_MASS × 9.81 / physics_hz`) and clamps to ±2.0. Measured total mass is **590 g**, so the
channel the consumer already expects maps onto force like this:

| condition | `foot_load` | force on one foot |
|---|---|---|
| swing leg | 0.0 | 0 g |
| static, 4 feet down | 0.25 | **148 g** |
| static, 3 feet down (the crawl) | 0.33 | **197 g** |
| single-leg support (the `rr` finding) | 1.0 | **590 g** |
| **software clamp** | 2.0 | **1180 g** |

- **The upper end is not a constraint.** The channel saturates at 1.18 kg; the sensor at 2 kg.
  **Software discards the range before the sensor does**, so full-scale is untouchable in normal
  operation.
- **The lower end is not a constraint either.** 20 g is 0.034 body weight — about **10× below**
  the static planted load that must be told apart from zero.
- **The band that matters (150–200 g) lands in the lower third of the range**, which is where an
  FSR's power-law response is steepest and best resolved. A 100 g – 10 kg part (the common
  FSR 402 spec) would put the whole gait down in its compressed, noisy region. **The smaller
  range is the better part here, not a compromise.**

### ⚠ Sim-to-real gap: the channel's negative half is unrepresentable

`foot_load` clamps to **−2.0**, but an FSR is **compression-only**. Nothing on hardware can
report an upward pull on a foot. Clamp the hardware publisher at `[0, 2]` and record any sim
excursion below zero as a sim-only artifact — do not let a consumer come to depend on it.

### Divider sizing — measure, do not pick a number

Standard series divider, sensor on the high side so the reading rises with force:

```
3.3 V ──[ FSR ]──┬── A_n (ADC, 12-bit, 0–3.3 V, 4095 counts)
                 │
                [ R_g ]
                 │
                GND
```

`V_out = 3.3 × R_g / (R_g + R_fsr)` — monotone increasing in force, since `R_fsr` falls as load
rises.

**`R_g` is the one decision that sets whether the range buys anything, and it cannot be chosen
from the datasheet.** FSR resistance at a given force varies with the puck, the backing
stiffness, and the part. **Procedure:** assemble one foot completely, rest a **175 g** weight on
it (the mid-stance operating point), measure `R_fsr` in place, and **set `R_g` to that measured
value**. That places the divider's steepest region on the load the gait actually spends its time
at. Use the same `R_g` on all four channels so per-foot differences show up in calibration rather
than in hardware.

### Mounting — the puck is more of the design than the sensor is

1. **A rigid disc, slightly SMALLER than the active area.** Force must arrive through a puck that
   stays inside the active circle. A puck that overlaps the inactive border ring loads the
   substrate instead of the sensing layer and produces nonlinearity and hysteresis that no
   calibration removes. **Design the toe cap AS the puck** — do not mount a sensor under a toe
   that happens to touch it.
2. **⚠ Shear is the hazard, not force.** This robot *propels*: the toe applies tangential force
   through stance, and the ledger's own attribution finding has `rr` doing the net propelling on
   6/6 seeds. FSRs sense normal force only, and sustained shear delaminates the layers. Put a
   **low-friction slip layer** (PTFE shim or a smooth-faced compliant pad) between the ground
   contact and the sensor face so the tangential load goes to the structure, not across the film.
   A foot is a far harsher application than the button these parts are designed for.
3. **Compliant backing behind the sensor**, so a point contact on a rounded toe is spread across
   the puck rather than concentrated at one spot.
4. Route the tails so a leg's full swing does not flex them at the sensor — flex at the tail root
   is a common failure.

### Calibration — a physical measurement, not a tuned constant

Part-to-part variation runs ±15–25 %, and the channel is normalized by body weight, so **each
foot needs its own curve**. This is a measurement of a physical device, so it does not collide
with prohibition §5 (*don't tune a constant to a signal's scale*) — but the **fit must be stored
per foot in the shared calibration JSON**, alongside the servo sign/origin/limits, so it travels
with the robot the way the servo map does.

- Robot on a stand, one foot at a time, load a known series (**0 / 50 / 100 / 175 / 300 / 590 g**)
  onto the assembled foot through its own puck.
- Record ADC counts, fit `counts → grams`, store per foot.
- Publish `foot_load = grams / 590` so the hardware channel is **numerically identical** to the
  sim channel and every existing consumer reads it unchanged.
- **Re-check after any foot re-assembly.** The puck geometry is part of the calibration.

### ⚠ What FSRs will NOT deliver, and what depends on it

- **Hysteresis and creep run ~10 %,** and creep grows under sustained load — a standing robot's
  reading drifts. **Benign for the stance threshold** (planted vs swing is a 10× separation);
  **a real caveat for the graded `unloaded` criterion term (weight 1.0)**, which is one of the
  two things these parts are being bought for. **Measure the creep before that term is trusted:**
  hold a static 175 g for 60 s and record the drift. If it is large, the term wants the
  *threshold*, not the magnitude.
- **The 20 g floor means light initial contact reads zero.** For stance detection that is a free
  noise floor. Touchdown *timing* was already assigned to the accelerometer, not the FSR
  (ledger 2026-08-24 ★3), so this costs nothing.
- **Contact sensing is not a prerequisite for stride odometry.** The median across four legs lets
  swing legs fall out as outliers with no contact input at all (ledger 2026-08-24 ★2). FSRs
  sharpen that; there they are an optimization. They are **required** for the `foot_load` weight
  unit and the G2 per-leg minima guard.

### Consuming it — one recorded negative

**Do not append per-leg `foot_load` to the motor EPM input.** Measured `NULL` on behaviour and a
`REGRESSION` on the self-model (`motor_tle` 0.263 → 0.320, t = +2.75) — load is a **body-level**
quantity and `MotorEPMv2`'s model is per-leg, so a per-leg model asked to predict it can only
accumulate irreducible error (ledger `:133`). **Re-use context: a body-level consumer** — the
support EPM sees all four at once and did find structure.

### Sim-side honesty model, and the gain-0 guard

Per the §5.4 pattern, the sim channel should be degraded to what hardware can actually deliver
before any result is claimed to transfer: **quantize to the 12-bit ADC through the fitted curve,
add the measured creep and hysteresis, clamp at `[0, 2]`, and apply the 20 g floor.** Testing the
load rules on the current idealized GRF is a weakened-slice result in the *too-good* direction.

Ships gain-0: the degradation model defaults OFF, and the hardware publisher is instrument-only
until its A/B runs.

## Phase 5 — Bring-up and the (d) test · recorded, not scheduled

- Hand-calibrate per Phase 2, export to the shared JSON, verify in sim.
- **Self-calibration check:** command the standing pose on a flat floor — the IMU should read
  pitch/roll ≈ 0; miscalibrated legs tilt the body. One pose gives 2 constraints against 12
  unknowns; a pose sweep (tripod stances, single-leg lifts) makes it observable. Hold until a
  hand-calibrated baseline works — don't debug calibration and its estimator together.
- **Hardware A/B has no seed.** Battery voltage decays monotonically through a session and
  confounds arm-order exactly like a lever effect. **Interleave ABBA, log A4 per trial, report
  voltage beside every metric.**
- **Mid-run body swap is the (d) test, not a demo.** Morphological perturbation is a stronger
  axis than goal relocation. Two requirements to keep it evidence: the **brain must not reset**
  across the swap (GNG, EPMs, learned weights persist — only the body rebuilds), and **TLE must
  be logged through the transition** — the spike-then-decay *is* the re-inference. Without it,
  it's a video.

---

## Belly-up auto-reset — made shape-aware 2026-08-10

`auto_reset_max_height` (default 0.030 m) is compared against **`chassis_y`, the node ORIGIN
height** — but where the origin sits when the body is on its back is set by the chassis *shape*.

| body | topmost surface (rel origin) | origin height when inverted | old fixed threshold |
|---|---|---|---|
| cad (single box) | +21 mm | 0.021 | 0.030 ✓ fires |
| **measured** | **+77 mm** (RPi/HAT stack) | **0.077** | 0.030 ✗ **can never fire** |

The 0.030 default really encoded *"9 mm of slack above the height this body rests at when
inverted"*. Fix preserves the **slack**, not the absolute number, keying off `_chassis_top_local`
(the highest chassis surface). Thresholds are now printed in the geometry receipt:
**cad 0.030** (bit-identical — the correction term is exactly 0.0), **measured 0.086**.

✓ **The flat-floor-only behaviour is CORRECT, not a gap.** The grounding test compares an
absolute world Y against a constant, so an inverted robot on the corridor hump (y ≈ 0.139) never
triggers. I initially filed that as a limitation; **the operator's call is that it is the desired
behaviour** — *"we don't want to reset on obstacles since the robot can usually get unstuck from
them."* Auto-reset is a safety net for the genuinely unrecoverable belly-up-on-flat-ground case,
and resetting on obstacles would delete exactly the recoveries the substrate is supposed to
learn. **Do not "fix" this by making it terrain-relative.**

**Verification status:** the threshold derivation is receipt-verified for both bodies. **End-to-end
firing is NOT yet verified** — headless attempts failed to produce a sustained inverted-on-flat-
floor state (`TELEPORT_RAMP_AT` drops onto the hump, which is elevated; in the arena the
hump-gated teleport never fires). To verify in the UI: **`[4]` → right-click drops the robot
inverted** (left-click = upright).

## Follow-up: make the live swap orientation-preserving

`_rebuild_body()` currently rebuilds the body **axis-aligned**, discarding orientation and
keeping position + velocities only. Reason: `_build_leg()` constructs the whole limb in **world
axes** — `heading`/`lateral` from world-frame `NEUTRAL_HEADINGS`, `hip1_local` added to the
chassis origin *without* the basis, `coxa_dir` from world `Vector3.DOWN`, and hinge frames from
world RIGHT/UP with hand-verified handedness. Those hold only while the chassis basis is
identity, which it always is at spawn.

A first cut preserved the full transform and **visibly deformed the robot** — front hip1 anchors
riding higher than rear, left-side anchors forward of right — because the legs were laid out on
world axes around a rotated chassis. Operator-caught in the UI, 2026-08-10.

**Cost of the current workaround:** a `[B]` swap resets body orientation, so it is a morphology
change *plus* an attitude reset. Acceptable for eyeballing, **not** clean enough for the (d) test
as evidence — the perturbation is no longer isolated to morphology.

**To fix properly:** rewrite `_build_leg()` in chassis-local space and let the basis carry the
rotation. The delicate part is the joint frames, whose handedness is hand-verified and commented
as such — get that wrong and the constraint solver silently inverts a sign.

**Operator note worth keeping:** with the deformed body, *"the robot is moving well and adapting
to its body."* Not a controlled result, but it is the same signature the substrate is supposed to
have — and it is exactly what the (d) test is meant to measure properly once the swap is clean.

## Open items

- Base box 98 × 98 mm **operator-confirmed adequate for collision dynamics**; carried from CAD
  rather than directly measured.
- The 9 mm crouch is recorded as an FK **reach constraint** (toe −28 mm rel. hip2). The exact
  hip2/knee angle pair achieving it was not reconcilable from the description — confirm against
  the model when G2 runs.
- Battery location: the 252 g base mass implies the pack sits in the **lower** structure, not the
  top plate as CAD's "Top plate / battery" label suggests. Visual confirmation would firm up G1.
- `KNEE_DROP_SIGN` (`:289`) has **zero use sites** — dead code. Out of Phase 0 scope
  deliberately (no opportunistic cleanup while the sprawl PR is open); revisit after the merge.
