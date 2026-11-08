# PiCrawler on the robot — ▶ Resume here (2026-10-27)

The working state of the real robot's brain, for a session starting fresh. The evidence for every claim
is in [`picrawler_lever_ledger.md`](../reports/picrawler_lever_ledger.md), entries dated 2026-10-26 and
2026-10-27. This page says where things stand and what to do next.

## The goal now

**Converge on a slow, steady pace** (operator, 2026-10-27). The robot walks, every leg pushes, and nothing
saturates. But the pace is not steady: the body's path speed is roughly constant, while its net progress
swings with heading wander and loose leg coordination. Per CLAUDE.md §1, "steady pace" has to become an error
the brain minimizes, not a schedule it follows.

## The deployed stack

Config: `…__lpgn_r05_e0d0_ref300__robot_p1_presskin_lz_sgk_hk0.json` (launcher and picrawler-dash). Each
suffix is one lever on the one before it:

| suffix | lever | what it fixed | verdict |
|---|---|---|---|
| `robot_p1` | the promoted robot stack (LoadPhaseGait, running-attitude gate, corner integrator) | — | `BASELINE` |
| `presskin` | MotorEPMv2 `press_kin`: the learned stance press pushes along each leg's lift direction (`feet_jac`) | a fixed hip2+knee press folded a leg under | `WORKING` |
| `_lz` | `level_int_zero_mean`: the corner integrator's offsets held zero-mean | it ran all legs to −0.30 against the press at +0.30; belly 48 → 62 mm | `WORKING` (2 runs) |
| `_sg` | LoadPhaseGait `support_gate`: a leg waits at the end of stance until the CoG is within a learned allowance of the feet that stay down (toe positions `feet_xz` from the commanded FK) | sim dips 29 → 18 % of lift-offs | `WORKING` sim / robot unclear (holds ~65 % of lift-offs) |
| `k` | MotorEPMv2 `kin_route`: the stance reach and the integrator also push along the lift direction | a rear leg folded under, knee pinned at the keep-out cap | `WORKING` (no fold since) |
| `_hk0` | `ctrl_lr` 0, `bias_lr` 0: the HK controller frozen at its start (its output remains, ~14 % of the command) | one leg at a time slid into saturated chatter after minutes (FR 82 %, stroke reversed, body moved backward) | `WORKING`; operator: HK off is right for the robot now |
| `_cs` | LoadPhaseGait `load_topic` empty: the Tegotae rule's load share from the contact flags (1/n touching), not the FSR force through the shared curve | the legs never locked (diagonal phase lock ~0.3 vs the sim's 0.88; per-leg rates 30–45 % apart); with contact shares 0.51 and 0.89, touchdown CV 0.38, the support gate's dips at target, no "swim" marks | `WORKING` (2 runs, 2026-10-28) — **the operator's best stepping so far**; circles right (below) |
| `_cs_pf` | **lever A, fourth form**: ProxMemory `pinged_range_floor` 1 — a remembered sonar point comes nearer only by a new ping, never by dead reckoning | `_cs` circled right at 4–6°/s: the stride odometry (2–3× over-read on slip) dragged far pings into the body through the proximity memory | `WORKING` (signal, 1 flat run, 2026-10-29: −0.8°/s, the hold alone −0.2°/s, avoid active 11 %, stepping held). `_cs_rv` (BearingToIntent `ref_value_min` 0.3) also removed it (+0.1°/s, 1 run). Refuted in the sim room: `ref_attack_s` |
| `_cs_pf_sl0` | LoadPhaseGait `support_learn_rate` 0: the support gate's allowance fixed at its 30 mm start | the learned allowance collapsed to its 5 mm floor at every obstacle contact and never recovered (the late-run swimming); with it fixed, 376 s through ten obstacle turns at a diagonal lock of 0.86 (0.94 in the last minute) | `WORKING` (robot n = 1 long run, 2026-10-31; sim corridor n = 6 ties or improves) — **the base now** |

On the shelf, not deployed: `_sgkc` (ProxMemory corroboration, below), `_sg0` (the gate watching only).

## Read before the next session (2026-10-28)

- **Leg names in brain_diag are the sim's order and the sim is mirrored:** sim `fl` = physical FR (ADC A1), `fr` = FL (A0), `rl` = RR (A3), `rr` = RL (A2).
- **The stride odometry over-reads 2–3× on this floor** (sonar closing rate vs the stride estimate); the brain's progress sense cannot see slip. `progress_commit_gain` is fed by it (`_pc0` = off, `NULL` on coordination, n = 1).
- **A severed FSR wire (physical FR) broke the gait** on two runs of the 28th morning; check the four channels' duty in the feed before trusting a run.
- **The operator's mark key** (`g` / `w` on the dash) and the per-tick IMU rows in the host log exist since 2026-10-27.
- **The late-run swimming is solved for now (2026-10-31):** the support gate's learner collapsed its allowance at obstacle contact and could not recover; with the allowance fixed at 30 mm (`_cs_pf_sl0`) a 376 s run with placed obstacles held a diagonal lock of 0.86 and finished at 0.94. The learner is `REGRESSION` with contact shares (its 10 % dip target is below the clean rate); re-use context in the ledger.
- **Next:** the carpet → vinyl crossing on `_cs_pf_sl0` (one video, obstacles optional); `_pc0` on top of `_cs_pf_sl0`; a second long `_cs_pf_sl0` run to make the n = 2; in the sim, the slip-sheet gate for lever A, the robot-measured disturbance layer and the contact belief for the gait's shares (`generator_and_belief_filter.md` G2). Readers: `prox_replay`, `sag_from_log`, the 20 s / per-minute scratch readers; the videos align at the dash's "STARTING IN" frame (~27 s). **Handles:** the dash's `o`/`p` keys and the sim panel's first slider are the PACE meta-control (LPG period 90 → 30 ticks with the stroke scaled from the config's point; `dash_run.pace_targets`); `_cs_pf_sl0_sim60` is the robot base with ideal servos for sim gating; the launcher allowlist holds only promoted configs and baselines (add a config there only while it is under test on the robot).

## What the last run says about pace (run 185039, 336 s)

- **Path speed is steady** (144–227 cm per 30 s, dead-reckoned); **net progress is not** (−25 to +182 cm per 30 s).
  The difference is heading wander: summed |Δyaw| 60–306° per 30 s.
- **The per-sample stride velocity is a blind metric.** Its sd is ~11 cm/s around a mean of −2 to +3 cm/s, so pace
  must be read per stride or per window, not per sample.
- **Coordination is loose**: touchdown-phase PLV 0.15–0.35, inter-leg PLV 0.25–0.45.
- **The support gate holds ~65 % of lift-offs**, and 26–51 % of lift-offs are still followed by a belly dip.
  The gate may be fighting the rhythm on the robot; it has never had a robot A/B.
- The LPG period is fixed at 50 ticks, and the level gate shortens the stroke to 0.4–0.65 of its nominal value.
- **Speed is set by period and stroke together** (sim speed map, 2026-10-26): with the stroke at 1.0, the period alone
  sweeps 4.9 → 7.0 cm/s without a fall. Short strides slip.

**Directions to consider** (none built). Check the ledger before building any:
1. **A pace error.** A per-stride speed estimate (odometry over each stride, not per sample) against an intended
   slow pace. The descent direction should go to ONE handle (the LPG period, with the stroke at 1.0) as a meta-control,
   rather than to several knobs.
2. **Heading.** Net progress is lost to yaw wander. Check what the heading hold sees on the robot (intent `err`
   runs ±0.5 rad) and whether its correction rides the stroke differential.
3. **The support gate on the robot.** One A/B run, `_sgk_hk0` with `support_gate` 0, to see whether the 65 % hold rate
   costs regularity.
4. **Instruments first.** Stride-period CV per leg, per-stride displacement, and per-stride heading change, so a pace
   lever has a full metric set (CLAUDE.md §3 rule 4).

## Open items

- **Range false readings.** About half are a front toe in the beam (44 % vs 10.9 % base rate). ProxMemory
  `support_n` / `too_close_pings` are built and unit-tested. Replay picked 6 / 6 in-sample (0 false). Out of sample
  it left 56 false ticks and may have delayed a real approach by 13 s. Needs a better truth criterion (the camera)
  or the toe-in-beam gate (the toes' FK plus the sensor's mount and beam width) before it drives the robot. The
  234 s cluster in run 164309 was a real reflex turn.
- **FSR scale.** Rear-left (sim RL) reads ~680 g while planted, others 30–56 g. LoadPhaseGait self-normalises, so the
  gait is fine, but any load-magnitude use (the CoG at stand, `support_cog`) needs a per-foot calibration first.
- **Not deployed yet:** the dash's second sync after "SAFE TO POWER OFF" (that line was lost twice), and
  `hk_cnorm` / `hk_hnorm` in the brain_diag snapshot.
- **Sim fidelity:** the sim does not sink on low friction (the slip sheet `OGMA_PICRAWLER_TERRAIN_SLIP` loses traction
  only), never tucks a leg, and its ultrasonic does not see the legs. The chassis friction and the per-foot slick
  ablation were inert (Godot's rough-material rule; a separate task is open for the chassis).
- **Sim start-up flip** with the support gate (3 flips at tick ~300 across 12 gate runs). Operator: not an issue on
  the robot.

## Tools

- `pi_host/tools/leg_jac_from_log.cpp` — per frame, per leg: depth, lift gains (hip2, knee), reach, hip2, knee,
  femur and shin tilt, toe x/z (chassis frame), plus grams per foot. Build: `g++ -std=c++20 -O2 -I cpp_core/include
  -I pi_host/include -I pi_host/build_local/_deps/json-src/single_include pi_host/tools/leg_jac_from_log.cpp`.
  Run from the repo root.
- `cpp_core/build/prox_replay <run.log> [k tc]...` — a run's pings and odometry through the real ProxMemory.
- **Propulsion:** the commanded toe's fore-aft speed while planted (negative = pushing forward), from the leg tool.
  It found the reversed FR stroke.
- **Saturation:** the run log's `{"kind":"actions"}` rows (u leg-major: leg × 3 + [hip1, hip2, knee]); % of ticks
  at |u| ≥ 0.99 per leg per window.
- **Pace:** `brain_inputs` rows every 50 ticks carry `odom` [x, y, yaw] and `stride_v`; brain_diag carries
  `load_phase`, `motor_epm`, `intent`, `avoid` at ~5 Hz. The run log carries every range ping
  (`{"kind":"range"}`).
- **Long sims:** run past the slowest integrator. The one-leg saturation only appears after ~9000 ticks; use 24 000.

- **The cloud of touches and the MIDI controller (2026-11-01, palpation stage 0).** `TouchMemory` (in `_cs_pf_sl0` and
  its sim twin; an instrument, gait byte-identical) stores every contact onset's commanded-FK toe position in the odom
  frame and lists them in its diag; the sim draws them body-locked (purple = the body's own, lavender = the brain's,
  `[X]` hides). The Godot bench dashboard draws the TWIN's FK toe at each FSR onset (purple) beside the brain's cloud
  from ogma_host (lavender), so the two FKs can disagree visibly, and takes the AKAI MIDI Mix on the twelve servos:
  LEARN per row, no ARM step (a slider or a fader is the servo's command, after pickup), the map in
  `godot_host/project/local/midi_map.json` (`midi_map.example.json` committed). `pi_host/PROTOCOL.md` §"MIDI".

- **Palpation session 1 (2026-11-01, ledger):** the toe's FSR registers every floor tap; the FK floor height repeats to
  2.5 mm when the tap is slow; the pose record was the limit (10 Hz telemetry) — fixed: `adc_fast` now carries the twelve
  pulses (rebuild benchd on the Pi before session 2). Reader: `python3 pi_host/tools/touch_session.py <benchd_*.jsonl>`.
  **The scripted probe** (`pi_host/tools/palpate.py --cond <label>`, run with the venv) replaces hand taps: 20/20 floor
  contacts, 0.3–1.6 mm repeatability; the biases are measured (FL 14–17 mm lower than FR; knee 3–4 mm; hip1 3 mm/120°).
  Box pass (2026-11-02): the 60 mm box reads 57.5 ± 4.6 mm above each leg's own floor; a face push shows as current
  (0.6–0.8 A) before the sole; knee B needs `--lift`. **The per-leg floor calibration is done** (`toe_floor_calib.py`
  → `pi_host/calib/toe_height_calib.json`, from `toes_up` with all four legs): one floor at −29 ± 1 mm on every leg,
  the box 61.7 ± 2.6 mm; applied in the reader, the bench twin and, since the encounter EPM, in ogma_host's `toe_y`
  channel (the TouchMemory offsets it was first written into were in the wrong frame — below). Pillow pass: 10/10 contacts at 62 ± 7 mm (box 62 ± 3), no current step; compliance over 120 µs separates on FL only —
  `PARTIAL` as a sole measurement; the deep press (400 µs ≈ 70 mm) settled it: compliance is in the BODY's response —
  current per commanded mm 0.045 A (box) vs 0.004 (pillow), lift 0.65 vs 0.15 mm/mm — `WORKING`; the sole is blind to it;
  `palpate.py` now has a lean guard (12°). **The horizontal probe** (`palpate_face.py --cond wall`): the sole FINDS a
  wall face on a forward advance (4/4 probes at 150–154 mm reach, 4–15 mm before the sonar's wall); it floor-touches
  at the start for its own height and reads the sonar for its travel. **The encounter EPM** (ledger 2026-11-02, last
  entry): `EncounterFeatures` (ten features per contact: height over the floor, position, sole load, the current's
  rise before and after, belly and lean rise, sonar, vertical fraction) → `EPM` (touch/encounter) → `BeliefFilter`,
  in `_cs_pf_sl0` and `_sim60`, consumed by nothing yet; replayed offline over the week's records
  (`cpp_core/bench/encounter_replay servo_map body_calib toe_height_calib benchd.jsonl...`): six nodes, condition
  separation NMI 0.35 vs 0.05 chance; the pillows on one node, the wall on one, the box tops NOT split from the floor
  (the conditioning lever next). **A frame defect fixed with it:** the brain's touch height had been the gait's parity
  foot (up to 43 mm below the toe tip); the new channel `reality.proprio.toe_y` (ogma_host `--toe-y`, the floor
  calibration applied in the host; the sim under `OGMA_PICRAWLER_TOE_Y`) is what both touch modules read now, and the
  TouchMemory offsets are gone. **The Pi's ogma_host must be rebuilt** before the next robot run of `_cs_pf_sl0`
  (the config now asks for `--toe-y` and the three new modules). **The terrain plan in the sim (2026-11-02 pm, ledger's last three entries):** step 1 done (position centring
  promoted, NMI 0.35 → 0.44); step 2 running (`_sim60_terrain`, `enc_events` in the body log, `stall_events.py` in the
  session scratch reads stall-vs-free; the 9000-tick battery met no stall below 38 mm, the 24 000-tick varied-world
  battery is the one that counts); step 3 built and first-measured (LiftDescent per encounter situation, G4b; configs `_sim60_terrain_g4a`
  control and `_sim60_terrain_g4b` lever, all three in the launcher): the three arms TIE at n = 4 and the descent
  never stepped in either form — `NULL` in context (ledger 2026-11-04). **The forward 8x8 field** (ledger 2026-11-04/05/06): `scripts/tof8.gd` in the sim (rays `[Y]`, mount `[7]`–`[0]`,
  30° / 70 mm chosen for now), its own cloud (green floor / yellow thing), every cloud's voxel living 30 s;
  `Tof8Range` reduces it to the nearest feature in the sonar's contract and **StuckSense's stall sense is ADMITTED
  through it** (battery 3: a mean score > 0.5 flags 92 % of stalled windows and 7 % of walking ones; the stride
  channel alone, with its commanded-turn gate and command scaling, 92 % / 0 %). The swing-lift dose response at 6 rad/s (ledger 2026-11-06): ×0.75 to ×2.0 all tie, the lift is NOT a handle on
  this course at either servo speed. The stride period (ledger 2026-11-07): short periods halve the stalled windows, a weak signal, not a handle. **The
  careful stepper** (operator, 2026-11-06: predictive, not stall-driven — "if I place my foot on these voxels my FSRs
  will report x and my IMU y"): the STAIRCASE gym and `TerrainMemory` with the per-leg foothold prediction are built
  (ledger 2026-11-07; `_sim60_stairs` in the launcher). The datum tracks the climb to ±2.4 cm; the prediction's error
  is 2.5 cm RMS at first contact and the same at re-contact because the frame slides under a hung body. Registration by
  re-observation is built and measured (ledger 2026-11-07, PARTIAL): first contact 2.6 → 2.3 cm at 50 mm, re-contact
  2.5 → 2.1, the 1 cm bar missed by two; the residual is the 3 cm voxel on a riser's edge: 1.5 cm voxels and a flat-ground gate on the datum take the
  error beyond 3 cm from 22 % to 13–16 % at 50 mm but the RMS floors near 2 cm, and the field cannot be the datum's
  second source (ledger, same day). Step 2 built and measured (ledger 2026-11-07, two forms): the prediction carries its spread; MotorEPMv2 has a
  per-leg `swing_target_topic` ([lift_mult, reach_m] × 4); `FootholdShaper` lifts the leg whose toe must rise and reaches
  past an up-edge — PARTIAL: a weak gain at 38 mm, a loss at 50, a scaled swing lift is not a step up; on the graded
  flight it FLIPS the body backward at the 64 mm riser (ledger 2026-11-07, the mechanism reproduced). **The pose
  evaluator** `SupportModel` (ledger 2026-11-08; `TerrainMemory patch_topic` feeds it; `sm_*` in the log of both
  stairs configs): bearing toes, static load shares, the support margin and each toe's LIFT margin from the commanded
  FK, the up vector and the terrain memory — the shares track the FSR shares at r ≈ 0.7, and the model shows the
  gait is a TROT with no static margin anywhere (two or three toes bearing in 87 % of rows, 55–75 % of lifts unsafe
  in both configs), so the flip is the trot's unsupported phase stretched by the policy's lift. **The stance depth
  handle** (`MotorEPMv2 stance_depth_topic`, ledger 2026-11-08; arms `sim60_stairs_shift*` with the `ConstantSource`
  scaffold) pitches the body as asked, byte-identical when off, and does NOT move the FSR share — a pitch is not a
  weight shift, as the model predicted; the shift a crawl needs is the body's translation over the planted toes, which
  the stance stroke already makes. **The crawl as a GATE on the lift** (`LoadPhaseGait margin_topic`, ledger 2026-11-08; arms
  `sim60_stairs_crawl` / `_cap`) was measured: the trot relaxes HALFWAY — static support doubles on the flat at a 10–20 %
  speed cost, no deadlock there; on the stairs support rises a third, tilt falls, the climb is unchanged (every arm stops
  at the 64 mm riser) and 20–29 % of the run is spent with every bearing leg held, because the held legs sit at the end
  of their stroke and nothing translates the body — a gate cannot CREATE support. **The sequencer** (`PoseSequencer` → `MotorEPMv2 pose_topic`, ledger 2026-11-08; `sim60_stairs_seq` in the
  launcher; the body publishes its leg model under `OGMA_PICRAWLER_LEG_MODEL`) is built: a pose-level controller
  with IK on the body's own FK, a load-seeking stance, and a crawl of SHIFT / LIFT / SWING / LOWER gated by the support
  model. The controller works (static support 89–98 % of rows, never flipped in eight runs, landings 2–6 mm from the
  predicted ground on the flat) and the planner does not yet walk (0.1 m per minute; the rear legs starved by the
  greedy leg choice; the stairs reached, one 12 mm riser climbed once). Second form (same day): the FSR filtered
  (operator), the fixed wave order, the standing sector ±0.9 rad — a balanced four-legged crawl on the corridor at
  0.2 m/min, statically supported 96–98 % of rows; on the staircase's approach it misses landings and times out swings
  because the terrain memory's frame rides the STRIDE odometry, which the crawl does not drive. Third form (same day, the operator's
  observation that the rear legs never lifted): a SETTLE phase on engage, body-frame lift heights, the blocked-toe rule
  tightened, the levelling extend-only, the patch's elevation off — on 3 of 4 traces every leg lifts 2.5–4 cm in turn,
  no missed landing, tilt 0.03–0.05, 0.36–0.46 m per run; the first riser reached. **Next: the sequencer publishes
  its own odometry for the memory** (it commands the body's motion over the planted toes), checked by the landing
  error; then the riser (the lift over the predicted tread, the depth handle as the crouch). IN_FLIGHT. Then the robot: the
  VL53L5CX driver and the staircase from lumber. Configs in the launcher: `_sim60_stairs` (instrument) and `_sim60_stairs_step2` (the policy). On the robot: step 2, the VL53L5CX
  driver and mount (it shares the belly ToF's default I²C address), then the admission run at the box, wall, pillow
  and the 38 mm step; the stride channel's two gates are a bench lever for `_cs_pf_sl0` as they are. The robot's next session: the sonar through
  ogma_host, the brain's cloud against the twin's at the toe height, whether the six nodes reappear. The
  front hip2 origin shifts (+49 µs P4, −62 µs P10) are a servo-map lever for a gait A/B, not applied. `touch_ready` leaves the FR toe on the floor (~300 g).
- **Before climbing: the belly ToF's pitch compensation** ([register O73](open_items_register.md)). The brain reads the
  uncompensated clearance; a pitched chassis reads as a changed height to the stance press and the learned stance
  height. The compensated channel exists in benchd at gain 0; switch and A/B it on slopes before any climb work.

## Procedures

- **Deploy:** `ssh picrawler 'cd ~/xaq-ai && git fetch -q origin && git merge --ff-only -q origin/picrawler-dev'`,
  then `cmake --build pi_host/build --target ogma_host -j3` on the Pi (~1 min). No service restart: the dash
  starts the new binary on the next run. Deploys need the operator's go-ahead.
- **After a boot** the Pi answers on `10.0.0.114` before `picrawler.local` resolves.
- **Runs land in** `~/xaq-ai/pi_host/log/dashrun_<stamp>/` (feed, brain_diag, events, meta) plus the host log
  `dashrun_<stamp>_…__<config>.log`. Copy them to `~/xaq_runs/robot_<date>/`.
- **The operator powers off after every run** (bench supply). The dash flushes to the SD card and says SAFE TO
  POWER OFF. A red light on power-up was cleared by reseating the power (2026-10-27).
