# ★ LA1 — verified technical fact sheet: the MicroDuck's brain

Prepared 2026-10-04 from the code at branch `duck-l2` (HEAD `1534859`, plus uncommitted edits listed in git status; none of
the uncommitted edits touch the three LA1 configs except a `body_manifest` metadata key added to
`head3o_h2_gaze_w10.json`, which the host does not read for behaviour). Read-only: no repo file was changed.

**How to read this.** Every claim carries a `file:line` anchor relative to `/home/xaqmusic/xaq-ai`. Abbreviations used in
anchors: `main.cpp` = `mj_host/src/main.cpp`; `IA` = `mj_host/src/IntentAdapter.cpp`; `HA` = `mj_host/src/HeadAdapter.cpp`;
`OBA` = `mj_host/src/OgmaBrainAdapter.cpp`; `MEPM`/`MEPMh` = `cpp_core/src/ogma/modules/MotorEPMv2.cpp`/`.hpp`;
`JSB` = `cpp_core/src/ogma/modules/JointSensorimotorBridge.cpp`; "config :N" = a line of the brain's own JSON config.
Values marked **observed / probe log** come from a 120 s LA1 run's stderr summary and JSONL
(`/tmp/claude-1000/-home-xaqmusic-xaq-ai/436c0496-3ca4-4372-9504-362323c2322a/scratchpad/la_probe.err` and `la_probe.jsonl` beside it; section drafts in `scratchpad/la1/`), not from code. **Caveat: that probe ran at `--seed 5`** (its train line
prints `(seed 5)`, `main.cpp:1764-1767`), not the preset's `--seed 1`; runtime numbers are illustrative of one 120 s run,
not statistics. "UNCLEAR" marks things not settled from the code.

Structure: §1 overview, rates, restore and learning matrix · §2 intent brain, module by module in tick order ·
§3 head brain · §4 stop brain · §5 host mechanisms (scaffolds, reflexes, schedules, sensors, the twist path) ·
§6 data-flow edge list · §7 open questions and inconsistencies (consolidated).

---

# §1 Overview and rates

## 1.1 What ★ LA1 is

- ★ LA1 is a launcher preset (`tools/duck_launcher/presets.json:27-48`), promoted in commit `1534859` ("the head looks
  where it has not seen"): it is the T11 configuration plus the host flag `--look-around 0.9 2.0`. The preset's hint:
  n = 18 against T11 "ties every number" with a livelier head (`presets.json:28`).
- Full argv as built by the launcher (`tools/duck_launcher/launcher.py:201-265`):
  `--level2 --graph mj_host/configs/a1v2_t11_not_target.json --secs 600 --seed 1 --noise 0.05
  --load-brain mj_host/checkpoints/duck_forebody_s1.brain.json <host_args> mj_host/models/microduck/scene_playroom_train.xml`
  where `<host_args>` is the verbatim string at `presets.json:45`. `--head-graph` and `--load-head` appear twice; both are
  plain assignments (`main.cpp:3368-3369`, `3561-3562`), so the **second** wins: head graph
  `head3o_h2_gaze_w10.json`, head checkpoint `head3o_gaze_h1_s2_nohold.json`. The first pair (`head2_h2_level_slow.json`,
  `head2j_h1_s2.json`) is never loaded.
- `--level2` runs `cmd_level2` (`main.cpp:3640-3641`, body `main.cpp:1610-3190`, tick loop `main.cpp:2013-3083`).

## 1.2 The layers, bottom to top

| Layer | What it is | Rate | Anchor |
|---|---|---|---|
| Physics | MuJoCo 3.12.0, timestep 0.002 s (default; no `<option>` in the scene), 10 substeps per brain tick | 500 Hz | `mj_host/CMakeLists.txt:41`; `mj_host/models/microduck/README.md:36`; `mj_host/src/DuckBody.cpp:57-66` |
| Walking policy (scaffold, Pollen's) | `alpha_walking.onnx`, 61-D observation, joint targets `HOME + 0.9·action`, low-passed 0.7 (legs) / 0.5 (head) | 50 Hz | `main.cpp:96-98`, `2765-2774`; `mj_host/src/Observation.hpp:11-27` |
| INTENT brain (level 2) | ogma graph `a1v2_t11_not_target.json`, 15 modules; commands the walker's twist `[vx, vy, vyaw]` and a head fore-aft slide | 50 Hz, every host tick (also during stops and rescues) | `main.cpp:2080`; `IA:223` |
| HEAD brain | ogma graph `head3o_h2_gaze_w10.json`, 2 modules; its commands are written directly as the four head joint targets (`--head-joints`) | 50 Hz, every host tick | `main.cpp:2626`, `2632-2634`, `2773-2774` |
| STOP brain ("stander") | ogma graph `a1v2_r19_settle_each.json`, 5 modules; stands the 10 leg joints during stops | 50 Hz, only inside stops | `main.cpp:2217`, `2760` |
| Fall rescue (scaffold) | `alpha_stand.onnx` drives all joints while fallen | 50 Hz while active | `main.cpp:91`, `2713-2717`; `mj_host/src/Recovery.hpp:95-180` |
| Skills (scaffolds) | Pollen's `ball_kick_left/right.onnx`, `alpha_ground_pick.onnx` (peck); "push" = the walker at 0.25 m/s | 50 Hz during a skill window | `main.cpp:1449-1456`, `2742-2754` |

Brain tick constant `kBrainHz = 50.0` (`mj_host/src/DuckBody.hpp:45`). The ToF sensor casts every 4th tick = 12.5 Hz
(`main.cpp:2786-2788`). `map_epm` processes every 5th tick = 10 Hz (config :201). CloudMap recomputes clusters every 4th
tick (config :250). LA1 runs 600 s = 30 000 ticks (`main.cpp:1844`).

## 1.3 Order of one host tick (`main.cpp:2013-2881`)

1. `recovery.update(gravity, gyro)` decides the driver (walker/brain vs the `alpha_stand` rescue) — `main.cpp:2015`.
2. The head's measured fore-aft position is handed to the intent adapter, then the **intent brain ticks**
   (`brain.tick(vel_body, g, w, a, odom.yaw(), tof_summary, &place)`, `main.cpp:2075-2080`). Its inputs were computed after
   the previous tick's physics step, so they describe the current body state.
3. Twist post-processing (chase speed floor, yaw linearisation) `main.cpp:2095-2101`; the stop / skill / unwind state
   machine, which may overwrite the twist, `main.cpp:2120-2560`.
4. Gaze error computed, then the **head brain ticks**; head joint targets composed — `main.cpp:2595-2693`.
5. Joint targets chosen: rescue scaffold / skill network / **stop brain** / walking policy — `main.cpp:2703-2775`.
6. `body.step(ctrl)` (10 × 2 ms) — `main.cpp:2777`.
7. Odometry update; ToF cast on `t % 4 == 0`; place vector, `tof_points`, body-velocity estimate — `main.cpp:2779-2881`.

## 1.4 Bus and scheduling semantics (all three brains)

- Each brain is an `OgmaInstance` whose scheduler ticks modules **in config (registration) order**
  (`cpp_core/src/ogma/Scheduler.cpp:5-13`, `58-65`).
- Publishing stores the topic's last value and synchronously dispatches to Direct subscribers
  (`cpp_core/src/ogma/InProcessBus.cpp:21-34`); a pattern ending in `.` is a prefix match (`InProcessBus.cpp:11-18`).
  Modules that read with `last_value()` therefore see a later-ticking producer's value from the **previous** tick.
  Feedback subscriptions get t−1 values at `begin_tick` (`InProcessBus.cpp:59-80`).
- The host adapter publishes its sensors as `ProprioToken`s on `reality.proprio.<sensor>` (producer `"host"`) **before**
  `instance->tick()` and reads `action.*` and other outputs **after** it (`IA:105-223`, `HA:198-237`, `OBA:189-205`).
- Params are passed to modules verbatim; schema defaults are NOT merged in, so an omitted key takes the C++ member
  default (`cpp_core/src/ogma/OgmaInstance.cpp:60-75`). Keys beginning with `_` are skipped (`OgmaInstance.cpp:66`).
- Every adapter rewrites every module's `master_seed`/`seed` param from the run seed (`IA:24-31`, `HA:22-29`, `OBA:37-42`).

## 1.5 What is restored from checkpoints (everything else starts empty each run)

| Brain | Checkpoint | Restored modules | Notes |
|---|---|---|---|
| intent | `mj_host/checkpoints/duck_forebody_s1.brain.json` | **only `motor_epm_intent`** (`--load-brain-modules` default, `main.cpp:1500`, filter `main.cpp:1713-1722`) | The file holds 14 modules, but map, cloud, things, play, seek, competence, voter, arbiter and outcome all start fresh. Body state in the file (`qpos`, `qvel`) is not restored ("the body at its reset", probe log). Saved at state width 26; LA1's bridge emits 27, so one zero, unidentified element is inserted at index 10 (`state_grow_at` 10, `MEPM:3184-3218`; probe: "the state grew 26 -> 27") |
| head | `mj_host/checkpoints/head3o_gaze_h1_s2_nohold.json` | whole graph (`head_bridge`, `motor_epm_head`) (`main.cpp:1788-1794`) | Identified self-model A (21×3); controller C near-identity on head pitch and roll, ≈0 on yaw; tonic h = 0 |
| stop | `mj_host/checkpoints/duck_r19_s2.json` | whole graph incl. `regime_epm` (`main.cpp:1863-1868`) | 6 regime banks; consolidation c = 0.99999; restoring the GNG drops some config settings (§4) |

No instance babbles in LA1: every restored step counter exceeds its `babble_ticks` (intent 31 000 > 30 000; head
35 000 > 0; stop 353 941 > 3 000) (`MEPM:4172-4175`).

## 1.6 Learning on/off by phase (code facts; details in §2–§5)

| Module | Walk | Stop | Rescue | Anchor |
|---|---|---|---|---|
| `motor_epm_intent` (A, b, Bx, C, h) | **on** | off | off | off/on `main.cpp:2207`/`2496`, `2018`/`2033`; also off in unwind and impeded back-offs `main.cpp:2508`, `2520`; mechanism `IA:408-436` |
| `map_epm` insertion / adaptation / stale-prune | off | **on** after the stop's settle | off | `--map-on-stop`: `main.cpp:1903`, `2241`, `2461`, `2026`; `IA:438-455` (baking and some GNG housekeeping still run on walks, §2.3) |
| `object_epm`, `thing_epm`, `thing_kind_epm`, `thing_context_epm` | on whenever input arrives | on | on | never frozen (`IA:418` touches only MotorEPM types) |
| PlayLoop graph, SkillOutcomeLoop table, LoopCompetence Beta counts, arbiter margin | on | on | on | never frozen |
| head `motor_epm_head` (C, h only) | **on** | off (`--stop-freeze-head`) | off | `main.cpp:2209`, `2497`, `2022`, `2036` |
| stop `motor_epm_legs` | (not ticked) | **on during the stand phase**, gated to near-upright (`g_z < −0.90`) | off | `main.cpp:1869`, `2247`, `2758`, `1996-2001` |
| stop `motor_epm_head` | — | frozen all run; commands discarded | — | `main.cpp:1870`; `OBA:270-288` |
| stop `regime_epm` | (not ticked) | on whenever the stander ticks | — | never frozen (`OBA:308`) |


---

# §2 The INTENT brain (`mj_host/configs/a1v2_t11_not_target.json`), modules in tick order

## 2.0 Summary table (tick order = config order)

| # | id | type | role in plain words | reads | publishes | learns (during LA1) |
|---|---|---|---|---|---|---|
| 1 | `twist_bridge` | JointSensorimotorBridge | packs the sensed twist, the brain's own last command and 18 sense slots into one 27-element state | `reality.proprio.intent` (host), `reality.proprio.sense` (host), `action.vx/vy/vyaw` (own echo) | `reality.motor_limb.twist` (27) | nothing |
| 2 | `motor_epm_intent` | MotorEPMv2 | the walker brain: a linear self-model of how its 4 commands move its senses, and a controller that descends 7 state priors (walk at 0.30 m/s gated by heading error and range; heading error → 0; ToF proximities → 0; head centred) | `reality.motor_limb.twist`, `reality.proprio.imu` (diagnostic only) | `action.vx`, `action.vy`, `action.vyaw`, `action.head_fore` | A, b (0.02), Bx (0.05), C, h via priors (0.1) — walks only |
| 3 | `map_epm` | EPM (rbf, 13→104) | the place vocabulary: pose + the last stop's cloud view | `reality.proprio.place_in` (host) | `reality.proprio.place` | GNG at stops only (`--map-on-stop`) |
| 4 | `play` | PlayLoop | exploration: its own transition graph over map nodes, value = place TLE novelty, climbs one hop toward higher value, else run-and-tumble | `reality.proprio.place`, `.heading`, `.vel_ego` | `percept.play_bearing`, `reality.cognitive.play_value` | graph, node positions, novelty EMAs, habituation |
| 5 | `cloud` | CloudMap | voxel cloud from ToF casts; the stack rule finds small things; detects movers; context of a thing; floor-break profile | `reality.proprio.tof_points`, `.place`, `percept.seek_bearing` (t−1), `reality.cognitive.seek_range` (t−1) | `reality.proprio.cloud` (36), `percept.cloud_change`, `reality.proprio.thing` (8), `reality.proprio.thing_context` (2), `percept.thing_bearing` (4), `percept.mover_bearing` (6), `percept.target_tall` (2) | nothing (fixed geometry) |
| 6 | `object_epm` | EPM (jl_state, 36→104) | vocabulary over the cloud's floor-break profile | `reality.proprio.cloud` | `reality.cognitive.object` | GNG, always — **output read by nothing** |
| 7 | `thing_epm` | EPM (rbf, 8→64, ≤64 nodes) | fine vocabulary of attended small things | `reality.proprio.thing` | `reality.cognitive.thing` | GNG, always — **read only by the host's log** |
| 8 | `thing_kind_epm` | EPM (rbf, 8→64, ≤4 nodes) | the KIND of thing (coarse) | `reality.proprio.thing` | `reality.cognitive.thing_kind` | GNG until 4 nodes baked |
| 9 | `thing_context_epm` | EPM (rbf, 2→48, ≤3 nodes) | where the thing stands: in the open / near tall / on a wall's line | `reality.proprio.thing_context` | `reality.cognitive.thing_context` | GNG until 3 nodes baked |
| 10 | `seek` | BearingSeekLoop | fixes a small thing seen at a stop in odometry coordinates, homes to it, arrives / forgets; chases a mover by constant-velocity prediction | `percept.thing_bearing`, `reality.proprio.odom`, `percept.mover_bearing`, `percept.target_tall`, `reality.cognitive.outcome_need` (t−1), `.outcome_pull` (t−1) | `percept.seek_bearing` (4), `reality.cognitive.seek_value`, `reality.cognitive.seek_range` | state only |
| 11 | `comp_seek` | LoopCompetence | grades seek: does the range fall while seek drives? | `reality.cognitive.seek_range`, `arbiter.gain.vision` (t−1) | `reality.loop.seek` | Beta counts |
| 12 | `comp_play` | LoopCompetence | grades play: does place TLE rise while play drives? | `reality.proprio.place` (tle), `arbiter.gain.play` (t−1) | `reality.loop.play` | Beta counts |
| 13 | `voter_loops` | LateralVoter (level 1) | turns the two competences into trust weights ∝ 1/(1 − competence + 0.05) | prefix `reality.loop.` | `consensus.1` | nothing |
| 14 | `arbiter` | EFEArbiter (precision mode) | picks the heading owner: G_seek = seek_need·trust_seek vs G_play = (1 − seek_need)·trust_play, with hysteresis | `reality.cognitive.seek_value`, `consensus.1`, `reality.cognitive.play_value` (channel switch only) | `arbiter.gain.vision/play/klino/planner` (one-hot) | running mean/var of the score gap |
| 15 | `outcome` | SkillOutcomeLoop | at an arrival asks for the least-known skill (kick / peck / push) and learns each skill's effect per kind × context | `percept.thing_bearing`, `reality.cognitive.thing_kind`, `.thing_context`, `.seek_value`, `.seek_range`, `reality.proprio.odom` | `intent.skill`, `reality.cognitive.outcome`, `.outcome_need`, `.outcome_pull` | per-cell displacement mean/variance |

The rows are checked against the module subsections below. "(t−1)" = the producer ticks later in the same brain, so the
reader sees the previous tick's value (§1.4).

**Inputs the host computes for this brain** are specified slot by slot in §5.3 (sense, place_in, tof_points, odom, etc.; sensors in §5.2).
**The heading reference and twist path** that turn the loops' bearings and the brain's actions into the walker's command
are in §5.4.

**Reading note for §2–§5.** Each module subsection was drafted from that module's own source files; its internal sub-headings carry a bracketed tag (e.g. `[CloudMap 3.6]`, `[EPM A.8]`, `[MotorEPM 2.4]`), and a bare cross-reference such as "see 3.6" or "(A.8)" inside a subsection points to the tag with the same number from the same source. A bare line anchor such as `:412` refers to the source file that subsection is about (its module `.cpp`, or the adapter file named in its heading); "config :N" is a line of that brain's JSON config.

## 2.1 `twist_bridge` — JointSensorimotorBridge (the bridge algorithm, shared by all four bridge instances, then this instance)


**Role:** pairs each motor's own last command with the sensed position of what it moves, and appends a block of "sense"
slots, so the downstream MotorEPMv2 sees one flat state vector per motor group.

**Algorithm (`JSB:313-412`).**
- On each message of `proprio_input_topic` it stores `last_position_[i] = values[proprio_indices[i]]` (`JSB:313-326`).
- On each `ActionOut` on `action_topics[i]` it stores `last_action_[i] = accel` (`JSB:328-336`). These are the
  MotorEPMv2's own published commands (MEPM publishes `accel = y[j]`, `MEPM:5889`), i.e. the brain's command **before**
  any host post-processing.
- On `load_topic` it copies `values[0 .. n_outputs·load_slots − 1]` into `last_load_` (`JSB:275-285`); output group `o`
  later reads `last_load_[o·load_slots + s]` (`JSB:397-401`).
- `tick()`: nothing until the first proprio frame (`JSB:339`). Then per output group `o` it emits a `ProprioToken` of
  width `3·group_size + load_slots` (`JSB:348`): for each joint `g`, `[pos, act, delta]` at `3g, 3g+1, 3g+2` with
  `delta = pos − prev_pos` (`JSB:360-390`), then the `load_slots` sense values (`JSB:397-401`). `prev_position_` is
  updated after publishing (`JSB:411`). The width is fixed by whether `load_topic` is configured, not by whether a load
  value arrived (`JointSensorimotorBridge.hpp:79-84`); slots carry 0.0 until the first value lands.
- Noise channels (`pos_noise_sigma`, `vel_noise_sigma`) default 0 and are not set: off (`JSB:366-387`).
- Module tick order: the bridge ticks before its MotorEPMv2 (config order), so the `act` slot holds the command issued on
  the previous tick, i.e. the command whose outcome the current `pos` is.

### [Bridge 1a] `twist_bridge` (intent brain)

Params: `proprio_input_topic reality.proprio.intent`, `proprio_indices [0,1,2]`, `group_size 3`,
`action_topics [action.vx, action.vy, action.vyaw]`, `output_topics [reality.motor_limb.twist]`,
`load_topic reality.proprio.sense`, `load_slots 18`. Output width 3·3 + 18 = **27** (probe log: "the state grew 26 -> 27",
`la_probe.err:71`).

Inputs (both published by `IntentAdapter::tick` before the graph ticks, `IA:99-221`):
- `reality.proprio.intent` = `[vx/0.4, vy/0.3, wz/yaw_range]`, each clamped to [−1, 1] (`IA:138-140`;
  ranges `IntentAdapter.hpp:29`; `yaw_range_` = 1.0 because `--twist-yaw-range` is not given, `IntentAdapter.hpp:390`,
  `MAIN:1645`). `vel_body` is the contact-odometry velocity in the body frame and the gyro yaw rate, each EMA-smoothed at
  0.1 per tick (`MAIN:2009`, `MAIN:2875-2878`).
- `reality.proprio.sense`: a base of 16 slots (`IA:153-156`), then with `--intent-fore-sense` one slot inserted at the
  front (`IA:161`) and with `--intent-range-sense` one slot inserted after it (`IA:162-165`). `--intent-head-sense` and
  `--intent-mover-sense` are not in the LA1 args, so the sense is exactly 18 values.

**Resolved state vector of `motor_epm_intent` (27 elements):**

| idx | from end | content | source |
|---|---|---|---|
| 0 | −27 | sensed forward velocity vx / 0.4 m/s ("pos" of motor vx) | `IA:138-140` |
| 1 | | own last vx command (act echo) | `JSB:333` |
| 2 | | Δ sensed vx per tick | `JSB:362` |
| 3, 4, 5 | | sensed vy / 0.3, act vy, Δ | as above |
| 6, 7, 8 | | sensed yaw rate wz / 1.0 rad/s, act vyaw, Δ | as above |
| 9 | −18 | **head fore-aft position** (`--intent-fore-sense`): `−0.5·((q5−home5)+(q6−home6)) / (0.6·1.10)`, clamped | `MAIN:2074-2079`, `IA:161` |
| 10 | −17 | **range to the seek target / 2 m**, clamped to [0, 1]; 1 when seek holds no target (`--intent-range-sense`). This is the "1 unidentified element inserted at 10" | `IA:162-165`, `la_probe.err:71` |
| 11, 12 | −16, −15 | trunk projected gravity g_x (fore-aft tilt), g_y (side tilt) | `IA:153` |
| 13, 14, 15 | | gyro ·0.3: w_y, w_x, w_z (clamped) | `IA:153` |
| 16, 17, 18 | | accelerometer x, y, z / 20 (clamped) | `IA:154` |
| 19, 20 | −8, −7 | sensed vx and vy again | `IA:155` |
| 21 | **−6** | **heading error** = (unwrapped heading − heading reference) / π, clamped to ±1 | `IA:155`; reference `IA:135`, set from the winning loop's bearing `IA:286-299` |
| 22 | −5 | map EPM surprise (TLE of `reality.proprio.place`), clamped | `IA:156`, `IA:225-226` |
| 23 | **−4** | ToF proximity left = 1 − (nearest return in sector)/1 m | `IA:156`, `MAIN:2789-2816` |
| 24 | **−3** | ToF proximity ahead | same |
| 25 | **−2** | ToF proximity right | same |
| 26 | **−1** | ToF "too close" fraction (contact) | `mj_host/src/Tof.hpp:51-54` |

Notes on the sense slots:
- With `--tof-body 0.5` the left/ahead/right slots are rebuilt from every Hit return of the last 0.5 s, carried by
  odometry into the current body frame and binned by body azimuth (left 5.6–22.5°, ahead ±5.6°, right); the contact slot
  stays the sensor's own (`MAIN:2790-2817`). The ToF is cast every 4th tick (`MAIN:2787`).
- `--seek-gate`: while the seek loop won the reference last tick, the ToF slot of the sector the seek target lies in is
  forced to 0 (`IA:144-151`).
- The heading reference, the range and the seek state are read from the graph's outputs after the previous tick
  (`IA:259-299`), so slots 10 and 21 lag the graph by one tick.
- The base sense comment in the code still calls it "the 12-slot sense ... two spare" (`IA:142-143`); it now has 16 base
  slots.

## 2.2 `motor_epm_intent` — MotorEPMv2

First an overview of all four MotorEPMv2 instances, then the mechanism as live in LA1 (shared by the head and stop instances in §3–§4), then this instance.

### Overview of the four MotorEPMv2 instances


| Brain | Bridge → state | MotorEPMv2 | Motors | What drives the action in LA1 | Learns during the run? |
|---|---|---|---|---|---|
| intent (level 2) | `twist_bridge`: 3 twist "joints" × [pos, act, delta] + 18 sense slots = **27** | `motor_epm_intent`, 1 "leg", motor_dim 4 | `action.vx`, `action.vy`, `action.vyaw`, `action.head_fore` | `tanh(C·x + h + model-implied step)` + noise σ 0.02 | Yes on walks: A, b (LMS 0.02), Bx (NLMS 0.05), C and h via the state-prior descent (0.1). Frozen during stops, unwinds, back-offs and rescues |
| head | `head_bridge`: 3 head joints × 3 + 12 sense slots = **21** | `motor_epm_head`, 1 leg, motor_dim 3 | `action.head_pitch`, `action.head_roll`, `action.head_yaw` | `tanh(C·x + h)`, no noise, no step | Only C and h via the state-prior descent (0.02), on walks. Model frozen (model_lr 0, state_model_lr 0). Frozen at stops (`--stop-freeze-head`) |
| stop (legs) | `legs_bridge`: 2 legs × (5 joints × 3 + 12 sense) = **27** each | `motor_epm_legs`, 2 legs, motor_dim 5 | 10 leg joints | `tanh(z_att + 0.3·(C·x + h − z_att))` + noise σ 0.015 | Frozen at setup and between stands; **turned ON during each stand** (`StopPhase::Brain`), gated to near-upright; HK annealed by consolidation |
| stop (head) | `head_bridge`: 4 head joints × 3 + 12 = **24** | `motor_epm_head` (stop), 1 leg, motor_dim 4 | neck/head pitch, head yaw, head roll | computed but **not applied** | Frozen for the whole run (`--stop-keep-head`) |

Major MotorEPMv2 features that are OFF in all three configs (gain 0 or socket empty): the homeokinetic (HK) controller update
(ctrl_lr 0 in intent and head; in the stop brain it is live only during stands and annealed),
bias_lr, sat_lr, postural reflex (`postural_gain` 0, default 0.3 overridden), panic (`panic_strength` 0, default 1.0
overridden), coupling/stroke/balance/heading/nav/amp/height homeostats, coordination search, DEP, `sense`,
`ctrl_damping`, whole-body C, lookahead, state_prior_split, state_prior_isolate, objective/plan/velocity sockets, CPG
embedding, intent_topic (empty). Config values: `mj_host/configs/a1v2_t11_not_target.json`,
`mj_host/configs/head3o_h2_gaze_w10.json`, `mj_host/configs/a1v2_r19_settle_each.json` (module params). Schema defaults
with a nonzero value that the configs override: postural_gain 0.3 (`MEPM:735`), panic_strength 1.0 (`MEPM:1106`),
ctrl_lr 0.01 (`MEPM:647`), bias_lr 0.005 (`MEPM:649`), sat_lr 0.02 (`MEPM:732`).

### The MotorEPMv2 mechanism as live in LA1 (all instances)


#### [MotorEPM 2.1] What it is

Per "leg" (motor group) it holds a forward self-model and a controller (`MEPMh:31-60`, struct `Leg` `MEPMh:1277-1355`):
- **Self-model:** `x̂(t+1) = A·ỹ + Bx·x(t) + b`, with A (n×m, motor → state), Bx (n×n, the state's own transition,
  allocated only when `state_model_lr > 0`, `MEPM:4331`), b (n). ỹ is an eligibility trace of the command,
  `ỹ += model_trace·(y − ỹ)` (`MEPM:4336-4341`); all three configs set `model_trace` (0.05 or 0.15). So the model is a
  linear state-space model x(t+1) ≈ A·ỹ(t) + Bx·x(t) + b, regressed on the trace of the command and the previous state.
- **Controller:** `y = tanh(C·x + h + c·hr [+ step])` (`MEPM:4985`, `MEPM:5197`). hr is zero in all three checkpoints
  and is only written in consolidate_reach modes 3–5 (off), so the `c·hr` term is 0.
- The motor time-loop error is `ξ = x − x̂` (`MEPM:4345`); `tle_ema` is an EMA (α 0.02) of ‖ξ‖ (`MEPM:4363`), reported as
  `motor_tle`.

#### [MotorEPM 2.2] Per-tick order (per leg), `MEPM:4168-5901`

1. `steps_seen += 1`; `warmup = steps_seen ≤ babble_ticks` (`MEPM:4172-4175`).
2. Regime-bank swap if `regime_topic` set (stop brain only), `MEPM:4187-4256` (2.6).
3. Consolidation ratchet if `consolidate_gain > 0` (stop brain only), `MEPM:4269-4319`.
4. **Learning**, only if `have_prev` (`MEPM:4326`): model update (A, b, Bx); then, if not warmup (`MEPM:4365`), the HK
   controller update, then the **state-prior descent** on C and h (`MEPM:4545-4752`).
5. **Emit** (`MEPM:4864`): babble during warmup, else `C·x + h`, the calm squelch (stop brain), the model-implied step
   (intent), `tanh`, motor_gain, exploration noise, hard clamp to [−1, 1] (`MEPM:5882`), publish each motor as
   `ActionOut.accel` on its action topic (`MEPM:5884-5891`), store `prev_x`, `prev_y` (`MEPM:5895-5901`).

#### [MotorEPM 2.3] Model learning (when the rates are nonzero)

- `A += lr_scale·model_lr·ξ·ỹᵀ`, unless the isolate-babble owns A (`a_lms`, `MEPM:4350-4352`):
  `a_lms = !(babble_isolate && warmup) && !(babble_owns_a && babble_isolate)`.
- `b += lr_scale·model_lr·ξ` (`MEPM:4353`).
- `Bx += lr_scale·(state_model_lr / (‖prev_x‖² + reg_eps))·ξ·prev_xᵀ` (normalised LMS, `MEPM:4354-4362`).
- `lr_scale = 1 − consolidate_gain·c` (`MEPM:4319`); 1 for intent and head.
- When `state_model_lr` is 0 (the head brain always; the intent while frozen) the prediction drops the Bx term
  (`smodel`, `MEPM:4330`, `MEPM:4343`), even though the checkpoint holds a Bx.

#### [MotorEPM 2.4] Action selection: the state prior, two halves plus a computed step

`state_prior_indices` lists state elements (negative = counted from the end, `idx += n`, `MEPM:4562`) with targets
`state_prior_targets`. Effective per-index target: `x*_k = target_k · tgate_k` (`MEPM:4564-4565`).

**(a) Gauss–Newton descent on C and h** (`MEPM:4545-4752`; runs only when `state_prior_gain > 0 && state_prior_lr > 0`,
`MEPM:4545-4547`). For each prior index k with error `e = x*_k − x[idx]`:
- `anorm = ‖A(idx,:)‖² + reg_eps` (`MEPM:4575`).
- precision `pw = state_prior_weights[k] · gate_g[k]` (pace gate, `MEPM:4642-4643`); `lw_k = state_prior_lr·gain·pw`
  (× `consolidate`/gate factors that are 1 here); `hlw_k` the same with `state_prior_h_lr` (−1 = follow
  state_prior_lr, `MEPM:4557-4559`).
- For each motor j below `state_prior_motors[k]` (0 = all, `MEPM:4701-4703`):
  `g = lw_k·e·A(idx,j)·G_j / anorm` (`MEPM:4704`), with `G_j = 1 − tanh²(z_j)` at the operating point the body actually
  ran (`z = C·prev_x + h + last step`, `MEPM:4367-4384`; recomputed under the calm squelch, `MEPM:4675-4697`).
  - feedback half: `C(j,:) += c_weight_k·g·prev_xᵀ` (`MEPM:4705-4710`; absent c_weights = full update). This writes the full
    outer product, i.e. every column of motor j's row, not only column idx.
  - tonic half: `h(j) += g·hlw_k/lw_k`, skipped if it would deepen a saturated command (|tanh z_j| > 0.95 in the same
    direction; `MEPM:4725-4748`).
- So "how to satisfy the prior" comes from the model's own authority A(idx, j); with a zero target the C column for idx
  acquires negative feedback by itself (`MEPM:4533-4544`).

**(b) Error attenuation for HK:** `ξ̃[idx] = (1 − w)·ξ[idx]` (`MEPM:4442-4490`, assignment `MEPM:4488`). It only feeds the HK update; HK is off
(ctrl_lr 0) in intent and head, so this half has no effect there.

**(c) The model-implied step** (`state_prior_step_gain`, `MEPM:5159-5196`; intent only). Stack the A rows of the prior
indices `A_p` (K×m) and their errors `e_p`, each row and error scaled by `sqrt(weight_k·gate_g[k])`; solve the ridge
least-squares `step = (A_pᵀA_p + reg_eps·I)⁻¹ A_pᵀ e_p` (`MEPM:5172-5189`), clamp each motor to ±1, multiply by the
gain (`MEPM:5190-5191`), and add it to the pre-tanh command (`MEPM:5192-5193`). It is stored as `prior_step` so the next learning
step's G is honest (`MEPM:4377`). The step does **not** apply `state_prior_motors` (no mask in `MEPM:5161-5194`).

**Pace gate** (`state_prior_gated_by`, `update_prior_gates`, `MEPM:3121-3149`): for gated index k with gating element j,
`d = EMA_25(x_j) − EMA_100(x_j)` (0.5 s minus 2 s), `rms = sqrt(EMA_1500(d²))` (30 s), `gate_g = clamp(1 − |d|/rms, 0, 1)`.

**Target gate** (`state_prior_target_gated_by`, `MEPM:3150-3178`): with `state_prior_target_gate_cos[k] > 0`,
`tgate = max(0, cos(x_j · gate_cos))^pow`; with `state_prior_target_gate_reach[k]` set to element r,
`tgate = min(tgate, clamp(reach_k · max(0, x_r) / |x_j|, 0, 1))` (`MEPM:3167-3175`).

**Important code fact:** `update_prior_gates` is called only inside the learning branch, when the descent is active
(`MEPM:4550`, guarded by `state_prior_lr > 0`, after `have_prev` and `!warmup`). While the host has frozen the rates,
`gate_g` and `tgate` keep their last values, and the step (which still runs) uses stale gates. The gates are transient:
not saved in the snapshot (`MEPMh:1322-1325`), so they restart each run (the 30 s variance EMA starts from 0).

#### [MotorEPM 2.5] Babble, exploration noise, output scaling

- **Babble** (warmup only): with `babble_isolate 1`, one motor at a time, held for `babble_hold` ticks at
  ±`babble_scale`, and A's column written from the paired ± difference (`MEPM:4866-4932`); otherwise uniform random
  commands (`MEPM:4933-4937`). In LA1 no instance babbles: every restored `steps_seen` exceeds its `babble_ticks`
  (intent 31000 > 30000; head 35000 > 0; stop 353941 > 3000; checkpoint values below), so `warmup` is false from the first
  tick. Probe log: "no babble" (`la_probe.err:38`; the "no babble" text is printed unconditionally by the host,
  `MAIN:1728`, the code fact is the `steps_seen` comparison at `MEPM:4174-4175`).
- **Exploration noise:** `σ = explore_noise·explore_mult` (× `calm_mult` when `state_prior_calm > 0`), Gaussian, added
  after the tanh, before the clamp (`MEPM:5434-5440`). `explore_mult` stays 1 (commit off; checkpoint value 1.0). The
  noise draws come from the per-leg `babble_rng` (`MEPM:5439`), which is **restored from the checkpoint**
  (`MEPM:7051-7052`), overriding the per-seed `seed` param that the adapters rewrite (`IA:22-30`, `OBA:37-43`).
- **Output:** `y_j = motor_gain·tanh(·)` (`MEPM:5197`, motor_gain 1 everywhere), hard clamp to [−1, 1] (`MEPM:5882`;
  `cmd_squash` 0). `max_dctrl` (0.05) clamps only the HK ΔC norm (`MEPM:4525-4527`), so it matters only in the stop brain.

#### [MotorEPM 2.6] Regime banks and consolidation (stop brain only)

- **Regime banks** (`regime_topic reality.proprio.regime`, `regime_banks 6`): the regime EPM's `winner_id` maps to a slot
  (first-seen order); on a winner change the active A, Bx, b (and tle_ema) are stored and the new slot's copy loaded
  (warm-started from the incumbent on first sight), and the boundary pairing sample is dropped (`MEPM:4187-4256`).
  C and h are not banked (`regime_c_banks` not set). Probe log end: `rbsw=28` swaps for legs, bank sample counts
  `n=[320544, 1965, 28920, 0, 0, 0]` (`la_probe.err:109`).
- **Consolidation** (`consolidate_gain 1`, `consolidate_n 4`, `consolidate_hold 1`, `consolidate_down_rate 0.001`,
  `consolidate_calm_ticks 500`, `consolidate_spares_prior 1`, `consolidate_reach 0`): c ramps up (+0.002·(1−c) per tick)
  while the first-4-index prior error EMA is < 0.15 and > 500 ticks since the last reset; decays at 0.001·c per tick in
  "chaos" (instant gate error > 0.30, gate EMA ≥ 0.15, or < 100 ticks since reset); holds otherwise (`MEPM:4269-4317`).
  All learning rates scale by `1 − c` (`MEPM:4319`) except the prior's own descent (`spares_prior`, `MEPM:4553-4556`).
  The checkpoint carries c = 0.99999; probe log end `cons=0.80`. Each `stander->on_reset()` (every stop and handback)
  publishes `events.reset`, which zeroes `ticks_since_reset` (`MEPM:1809-1811`) and therefore forces ≥ 100 decay ticks.
- **Calm squelch** (`state_prior_calm 1`, `state_prior_calm_fixed 0.3`, shared-C branch `MEPM:5060-5150`):
  `z_att = Σ_k C(:,idx_k)·x[idx_k]` over the 9 prior indices; command pre-tanh = `z_att + 0.3·(C·x + h − z_att)`
  (`MEPM:5147-5148`); the noise σ is also × 0.3 (`MEPM:5436`). So the prior's own feedback columns act at full slope and
  everything else at 0.3.

---

### This instance: `motor_epm_intent`


**Role:** the walker's level-2 brain: it commands the twist (vx, vy, vyaw) and the head's fore-aft translation, given to
Pollen's walking policy, by descending priors on speed, heading, ToF proximity and head position through a self-model of
how its commands move its own senses.

**Inputs:** `reality.motor_limb.twist` (27-element state, §1a). `imu_topic reality.proprio.imu` is subscribed but read with
a picrawler layout (`values[2]` as fwd_v, `values[3]` as yaw rate, `MEPM:1919-1931`); on the duck `values[2]` is gravity z,
so the diag `fwd_v` reads ≈ −1.0 (probe log line 114). Its consumers (heading gains) are all 0: diagnostic only.

**Key params (config):** n_legs 1, motor_dim 4, model_lr 0.02, state_model_lr 0.05, model_trace 0.05, ctrl_lr 0,
bias_lr 0, sat_lr 0, reg_eps 0.01, max_dctrl 0.05, c_init 1.0 / init_scale 0.01 (init only; overwritten by the restore),
babble_ticks 30000 / scale 0.8 / isolate 1 / hold 75 / owns_a 0, explore_noise 0.02, motor_gain 1,
reset_breaks_model_pairing 1, state_prior_gain 1, state_prior_lr 0.1 (h_lr default −1 → 0.1), state_prior_step_gain 1,
state_grow_at 10, consolidate_gain 0.

**The seven priors resolved on the 27-element state:**

| k | index | element | target | weight | pace gate (`gated_by`) | target gate |
|---|---|---|---|---|---|---|
| 0 | 0 | sensed vx (unit 0.4 m/s) | **0.75** (= 0.30 m/s) × tgate | 1 | none | cos and reach, below |
| 1 | −6 → 21 | heading error (unit π rad) | 0 | 1 | none | none |
| 2 | −4 → 23 | ToF proximity left | 0 | 1 | none | none |
| 3 | −3 → 24 | ToF proximity ahead | 0 | 1 | none | none |
| 4 | −2 → 25 | ToF proximity right | 0 | 1 | none | none |
| 5 | −1 → 26 | ToF too-close share | 0 | 1 | none | none |
| 6 | 9 | head fore-aft position | 0 | 1 | **element 0 (sensed vx)** | none |

No `state_prior_motors`, no `state_prior_c_weights`: every prior descends through all four motors, C and h alike.

**The speed target.** For k = 0: `target_gated_by = −6` (the heading error, element 21), `gate_cos = π`, `pow 1`,
`gate_reach = 10` (the range element), `reach_k = 0.64`. With `x21 = e/π` (e = heading error in rad) and `x10 = min(r/2, 1)`
(r = range to the seek target in m), the code gives (`MEPM:3160-3175`):

`tgate = min( max(0, cos(π·x21)), clamp(0.64 · x10 / |x21|, 0, 1) ) = min( max(0, cos e), clamp(0.64·π/2 · r/|e|, 0, 1) )`

so the vx target is `0.75 · min(cos e, ≈1.005·r/|e|)` (r in m, e in rad). The config description's "min(cos e, 0.64
range/|e|)" is the same expression in the state elements' own units (range in units of 2 m, e in units of π). Far away or
without a seek target (x10 = 1), the reach term only binds when |e| > 0.64π ≈ 115°, where cos e is already 0. The gated
target enters both the descent's error (`MEPM:4564-4565`) and the step's error (`MEPM:5173-5174`). The schema's
worked example derives reach_k = 2.1 for this walker (`MEPM:213-216`); the config uses 0.64.

**The head-centring prior (k = 6).** Element 9 (head fore-aft) → 0 with its precision multiplied by the pace gate on
element 0: full while sensed forward speed is steady at the stride timescale, falling to 0 while it changes by its own
typical amount (`MEPM:3131-3146`, applied at `MEPM:4643` and in the step's row weights `MEPM:5177-5182`).

**What it computes each tick (LA1, after the restore):** the model update (A by LMS at 0.02 with ỹ, b, Bx by NLMS at 0.05);
HK update computed but zero (ctrl_lr 0, bias_lr 0); the state-prior descent on C and h through A at 0.1 (with updated
pace and target gates); then the command `y = clamp(tanh(C·x + h + step) + N(0, 0.02²), ±1)` where `step` is the
ridge-LS solution over the 7 prior rows of A (§2.4c). Telemetry at the end of the probe: `motor_tle 0.218`, `spStep 1.03`
(|step|) (probe log line 114).

**Outputs and consumers:** `action.vx`, `action.vy`, `action.vyaw`, `action.head_fore` (`ActionOut.accel` ∈ [−1, 1]).
- `IntentAdapter::tick` reads the three twist actions after the graph tick and scales them by 0.4 m/s, 0.3 m/s,
  1.0 rad/s (`IA:325-331`); `action.head_fore` becomes `fore_target = 0.6·1.10·a` rad (`IA:332-334`).
- `twist_bridge` reads the same three twist topics back as the `act` slots (`action.head_fore` has no bridge pairing; its
  sensed counterpart is the fore-aft sense at index 9 = 3·3, the slot MEPM's conventions assign to motor 3's position,
  e.g. `c_init` at `C(j, 3j)`, `MEPM:3108-3110`).
- **Host post-processing that the model never sees** (the act echo is the brain's own command, `JSB:333`):
  the heading reflex mixes the applied yaw, `vyaw = share·reflex + (1−share)·vyaw_brain` while a loop holds the reference
  (`IA:373-390`; `--heading-reflex 1.0 0.3 1.0`; probe log: 6000 ticks steered by a loop's bearing, line 113);
  `--chase-vx 0.35` raises vx to ≥ 0.35 m/s while a mover is chased (`MAIN:2097-2098`); `--yaw-linearize-below 0.15`
  remaps vyaw while |vx| < 0.15 m/s (`MAIN:2100-2101`); at stops the twist is zeroed (`MAIN:2500`); the head translation
  is rate-limited at 1.0 rad/s, centred at stops, and applied on top of the head brain's joints (`MAIN:2655-2662`).

**Learning during the run.** Live rates while unfrozen: model_lr 0.02 (A and b), state_model_lr 0.05 (Bx),
state_prior_lr 0.1 (C and h through the prior; h_lr follows). ctrl_lr, bias_lr, sat_lr = 0.
The host freezes through parameters (`IntentAdapter::set_learning`, `IA:408-431`: sets model_lr, ctrl_lr, bias_lr, sat_lr,
state_prior_lr, state_prior_h_lr, state_model_lr to 0 and restores them later). Freeze points:
- every stop start (`MAIN:2207`) until the stop ends (`MAIN:2496`);
- rescue hand-off (`MAIN:2018`) until hand-back (`MAIN:2033`);
- skill unwind back-off (`MAIN:2508`) and the impeded back-off (`MAIN:2520`).
So `motor_epm_intent` **does learn during walks** and **does not learn during stops**. The brain still ticks and computes
commands during stops (`MAIN:2080`); they are replaced by a zero twist. Probe JSONL (6000 ticks): `stop == 0 && drive ==
walk` on 4316 ticks (71.9 %); stop phases on 1462 ticks (24.4 %); scaffold 222 (3.7 %). The probe summary's "learning
frozen 4%" counts only rescue ticks (`MAIN:2038-2039`, printed at `MAIN:3182`), not the stops.

**Grow on restore.** The checkpoint was saved at n = 26 (fore-sense only); LA1's bridge emits 27. With
`state_grow_at 10`, the first 27-wide frame inserts one zero element at index 10 in A's rows, C's columns, Bx's rows and
columns, b, x, prev_x (`MEPM:3184-3208`, call `MEPM:3217`): the range slot starts with no identified authority (A row 0) and
no feedback. Without a re-babble (`--rebabble` not given), A's row 10 is learned only by closed-loop LMS on the walk.
Without `state_grow_at`, every frame would be dropped (`MEPM:3218`).

---

## 2.3 `map_epm` — EPM (with the EPM algorithm common to all six EPM instances)

Six EPM (Episodic Predictive Module) instances run in LA1. There are five in the INTENT brain
(`map_epm`, `object_epm`, `thing_epm`, `thing_kind_epm`, `thing_context_epm`) and one in the STOP brain
(`regime_epm`). The HEAD brain has no EPM (its modules are `head_bridge` and `motor_epm_head`, per the preamble).

Anchors are relative to `/home/xaqmusic/xaq-ai`. "Probe log" means the 120 s LA1 probe run
(`scratchpad/la_probe.err` / `la_probe.jsonl`). Those values were observed at runtime and are not taken from code.

---

### EPM common: the algorithm as implemented


#### [EPM A.1] Structure and wiring

- An EPM is one frozen encoder, one GNG (Growing Neural Gas) and a dual-TLE bookkeeper. All of them are
  wrapped as an `ogma::Module` (`cpp_core/include/ogma/modules/EPM.hpp:10-21`). The encoder and GNG are the
  v3 classes (`EPM.hpp:33-36`; `cpp_core/include/v3/gng.hpp`, `cpp_core/src/v3/gng.cpp`,
  `cpp_core/src/v3/encoder_rbf.cpp`, `cpp_core/src/v3/encoder_jl.cpp`).
- **Output topic naming.** The topic is `reality.<modality_group>.<modality_name>`, or `consensus.<name>` when the group is
  `consensus` (`cpp_core/src/ogma/modules/EPM.cpp:264-269`).
- **Inputs subscribed** (`EPM.cpp:408-422`). There are three:
  - the `input_topic`, which must carry a `ProprioToken` for the `rbf` and `jl_state` encoders (`EPM.cpp:75-78`);
  - `neuro.state`, which is optional;
  - `prediction.<group>.<name>`, a Feedback subscription that is made only when `subtract_descending_prediction`
    is set. That flag defaults to true (`EPM.hpp:131`).
- **Neuro scaling is inert in LA1.** `neuro.state` is published only by `NeurochemState`
  (`cpp_core/src/ogma/modules/NeurochemState.cpp:383`), and none of the three graphs contains one. All four
  scales therefore stay at their defaults of 1.0 (`EPM.hpp:240-243`), so `apply_neuro_scaling()`
  (`EPM.cpp:629-651`) just re-installs the base `epsilon_b`, `min_insertion_error` and mitosis threshold on every tick.
- **The descending predictor is inert in LA1.** No module in the three graphs publishes `prediction.*`. The
  module types are JointSensorimotorBridge, MotorEPMv2, EPM, PlayLoop, CloudMap, BearingSeekLoop, LoopCompetence,
  LateralVoter, EFEArbiter and SkillOutcomeLoop; a grep for `"prediction.` in their sources finds nothing.
  `pending_prediction_` is therefore always null, and the subtraction branch never fires (`EPM.cpp:834-852`).
  Every EPM here topologises the **raw encoded observation**, not a residual.
- **Params are passed verbatim.** Schema defaults are NOT merged. `OgmaInstance` calls
  `m->on_setup(bus, spec.params)` with exactly the config's params (`cpp_core/src/ogma/OgmaInstance.cpp:60-75`), and the EPM
  applies only the keys that are present (`EPM.cpp:43-47`, `291-320`). A key omitted from the config therefore takes the
  **`GNG::Config` / `EPM.hpp` member default**, not the `params_schema()` default. This matters for
  `baking_threshold`: the schema says 50 (`EPM.cpp:108`), but the GNG default is **100** (`gng.hpp:89`). The
  contract notes the same trap at `docs/plans-and-designs/primitives/EPM.md:58`.

#### [EPM A.2] Encoders used here

**`rbf` (FrozenRBFEncoder)**, used by `map_epm`, `thing_epm`, `thing_kind_epm`, `thing_context_epm` and `regime_epm`.
- **Normalisation.** Each input dim is mapped affinely by `(x − dim_min)/(dim_max − dim_min)` into [0,1] and **clamped**
  (`encoder_rbf.cpp:129-142`). If `dim_min`/`dim_max` are omitted, every dim defaults to [-1, 1]
  (`encoder_rbf.cpp:43-46`). They are set via `EPM.cpp:352-371`, and their lengths must equal `proprio_state_dims`.
- **Centres.** There are `projection_dim` centres in [0,1]^d. For d ≤ 6 they form a regular grid with
  `ceil(projection_dim^(1/d))` points per dim, taking the first `projection_dim` points
  (`encoder_rbf.cpp:79-114`). For d > 6 they are a Halton quasi-random sequence over prime bases (`encoder_rbf.cpp:115-122`).
- **Bandwidth.** σ = 0.8 × the mean nearest-neighbour distance between centres (`encoder_rbf.cpp:50-64`).
- **Output.** Each output is `exp(−‖s − c_i‖² / 2σ²)`, and the output vector is L2-normalised to unit length
  (`encoder_rbf.cpp:148-168`).
- **Dim mismatch.** On a mismatch the encoder returns a zero vector (`encoder_rbf.cpp:149-151`), which is the
  "zero-encode trap" in `EPM.md:236`. All five RBF EPMs here receive exactly their declared width (see C).
- **`projection_dim` auto-derivation.** This applies only to `rbf` with `projection_dim` omitted and `proprio_state_dims`
  given. The rule is `pd = max(48, 8 × proprio_state_dims)`, and it is printed to stderr (`EPM.cpp:271-286`).
- `master_seed` is parsed (`EPM.cpp:260`) but is never read anywhere else in `EPM.cpp`. The RBF centres are
  deterministic (Halton or grid), so `master_seed` has **no effect** on these EPMs.

**`jl_state` (FrozenJLEncoder::make_state_encoder)**, used by `object_epm`.
- **Projection matrix.** R has shape (input_dim × projection_dim), drawn N(0, 1/√pd) from `mt19937` seeded by a polynomial
  hash of `modality_name` (`encoder_jl.cpp:36-46`, `98-113`). The seed is not `master_seed`.
- **Encoding.** The input vector is **L2-normalised first**, then projected, then L2-normalised again
  (`encoder_jl.cpp:209-224`). Only the input's **direction** survives, not its magnitude. An all-zero input stays
  zero, because the normalisation is skipped below a norm of 1e-6.
- **Rules.** `projection_dim` must be explicit: there is no auto-derivation, and an omitted value falls back to the member default of 128
  (`EPM.hpp:103`). `dim_min`/`dim_max` are refused (`EPM.cpp:382-390`; `EPM.md:55`).

**Not used.** `dim_autocal_ticks` is 0 everywhere (no commissioning, `EPM.hpp:175`), and `insertion_autotune` is
false (fixed threshold, `gng.hpp:126`).

#### [EPM A.3] The GNG, step by step (`gng.cpp:94-314`, Linear gain mode, the default `gng.hpp:153`)

| Stage | What happens | Anchor |
|---|---|---|
| Bootstrap | The first 2 inputs become nodes 0 and 1. Until then `step` returns `{0, 0}`, and the EPM publishes a placeholder token with `winner_id = −1` and a zero latent (`EPM.cpp:856-865`, `761-778`). | `gng.cpp:96-105` |
| Winner search | Brute-force O(N) Euclidean scan for the nearest node (s1) and the second nearest (s2). | `gng.cpp:73-88`, `112` |
| Error bookkeeping | `s1.error += d1²` and `s1.ema_error = 0.9·ema + 0.1·d1²`. These are SQUARED distances. | `gng.cpp:118-120` |
| Winner move | Only if `visits < baking_threshold`: `w += ε_b·(1 − 0.9·stab)·(x − w)`, with `stab = max(visits/bt, min(1, 0.02·health))`. **Baked nodes never move.** | `gng.cpp:131-142` |
| Neighbour move | Each edge-neighbour that is not baked moves by `ε_n·(1 − 0.9·stab)·(x − w)`. | `gng.cpp:157-169` |
| Edge ageing | s1's edges age by 1 and are removed above `max_age`. An edge is **exempt** if either endpoint is baked, near-baked (visits ≥ 0.6·bt) or has health > 20. The s1–s2 edge is then created or reset to age 0. | `gng.cpp:171-205` |
| Isolated removal | Every step, non-baked, non-near-baked nodes with health ≤ 5 and **no edges** are killed, keeping at least 2 nodes. These kills are **not** recorded in `last_pruned_ids_`. | `gng.cpp:208`, `320-339` |
| Insertion | Every `lambda_new` GNG steps, and only if `node_count < max_nodes`. Take q = the node with the largest accumulated error. **Skip if `q.ema_error < min_insertion_error`.** Take f = q's neighbour with the largest error. If q is baked, insert at the **current input** and link to q and f; otherwise insert at the midpoint of q and f (Fritzke). Then q.err and f.err are multiplied by α, and the new node gets q's error. | `gng.cpp:210-215`, `390-456` |
| Full cap | When `node_count ≥ max_nodes`, insertion is simply skipped. Nothing else changes (no eviction). | `gng.cpp:212` |
| Stale prune | Runs at the same cadence. If enabled, it kills non-baked, non-near-baked nodes with health ≤ 5 whose last visit is more than `stale_window_factor` GNG steps ago (default 12000). | `gng.cpp:214`, `345-368`, `gng.hpp:159` |
| Global error decay | `error *= (1 − β)` for every node, every step (β = 0.0005). | `gng.cpp:233` |
| Health | Every node, every step: `health *= 0.995^(1/(1 + 0.08·health))`, at half speed for near-baked nodes. The winner gains +0.5, capped at 100. | `gng.cpp:225-245`, `281`; constants `gng.hpp:191-207` |
| Health death | Only the weakest node can die, and only if its health < 0.01, `node_count > 16` (`health_death_min_nodes`), and at least 25 steps have passed since the last death (at most 1 per step). With `health_death_spares_baked`, baked nodes are excluded. | `gng.cpp:251-272`, `gng.hpp:172`, `194-202` |
| Baking (consistency gate) | Fires **once**, when the winner's `visits` first reaches `baking_threshold`. If `s1.ema_error ≥ effective_min_insertion_error` the node is **demoted**: visits are reset to bt−3 and its errors halved. Otherwise it is baked and `last_step_baked_` is set. "Baked" means `visits ≥ baking_threshold` (`gng.cpp:466-471`). | `gng.cpp:287-300` |
| Return value | `{s1_id, d1}`. The published `quant_error` is the **Euclidean distance** d1, not squared. | `gng.cpp:313` |

Two consequences of this table matter later:
- **The insertion floor and the bake gate share a scale.** Both compare a node's EMA of **squared** distance against
  `min_insertion_error`. With unit-norm latents, 0.06 corresponds to a distance of about 0.245, and 0.25 to a distance of 0.5.
- **Node IDs are never reused** (`gng.cpp:25-37`, `next_id_`).

#### [EPM A.4] Mitosis: inert in all six EPMs

- `GNG::maybe_mitosis` is called only when `mitosis_gatekeeper` is true (`EPM.cpp:900-902`). That flag defaults to false
  (`EPM.hpp:222`; schema `EPM.cpp:135-140`), and none of the six configs sets it.
- `mitosis_error_threshold: 0.1` on `map_epm` (`mj_host/configs/a1v2_t11_not_target.json:202`) and on `regime_epm`
  (`mj_host/configs/a1v2_r19_settle_each.json:274`) is therefore **inert**. It is installed into the GNG config
  (`EPM.cpp:301`, `650`), but the split path is never reached.
- Probe log: `mitosis_count 0` for every EPM (`la_probe.err:111`, `115`, `118-121`).

#### [EPM A.5] TLE, novelty and EMA (`EPM.cpp:669-704`, `706-759`)

- **Transition surprise** (`transition_surprise_kind` = `displacement`, the default, `EPM.cpp:120-126`) is
  `‖prototype(winner_t) − prototype(winner_{t−1})‖`. It is 0 when the winner repeats (`EPM.cpp:674-683`). The
  `logprob` variant is not used by any of these configs.
- **TLE** is `tle = tle_alpha·quant_error + tle_beta·transition_surp`, with defaults 0.7 and 0.3 (`EPM.hpp:106-107`,
  `EPM.cpp:685`). No LA1 EPM overrides the weights.
- **EMA.** `ema_tle = 0.95·ema_tle + 0.05·tle` (`tle_ema_alpha` 0.05, `EPM.hpp:108`, `EPM.cpp:689`), updated once per
  *processed* tick. It starts at 0.1 (`EPM.hpp:186`).
- **Novelty threshold.** `novelty_threshold = max(novelty_floor = 0.01, 1.5 · ema_tle · neuro_scale)` (`EPM.cpp:690-691`).
- **`is_novel`** is `quant_error > novelty_threshold`, using the threshold just updated this tick (`EPM.cpp:721`). Note
  that the QE is compared against a threshold built from the TLE EMA.
- **`tle_norm`** is a diagnostic only (`diag_lite`): `last_tle / ema_tle` (`EPM.cpp:946`). `qe_lag1` is the lag-1
  autocorrelation of QE (`EPM.cpp:947-951`). Neither is on the token.

#### [EPM A.6] The RealityToken (`EPM.cpp:706-759`; struct `cpp_core/include/ogma/Topics.hpp:97-129`)

| Field | Content |
|---|---|
| `winner_id`, `quant_error`, `tle`, `is_novel` | As described above |
| `expected_error` | `ema_tle` |
| `transition_surp`, `novelty_threshold` | As described above |
| `just_baked`, `just_pruned`, `pruned_ids` | Bake and prune events from this step |
| `node_count`, `baked_count`, `mitosis_count`, `drift_count` | GNG counters |
| `history_trace` | The last 5 winners (`history_trace_size` 5, `EPM.hpp:111`) |
| `latent` | The **encoder output** (after any subtraction, none here), not the prototype |
| `winner_prototype` | The winner's prototype vector |
| `predicted_pathway` | Empty (`predicted_pathway_steps` 0) |

- On a tick with no input, or before bootstrap, the EPM publishes a placeholder: `winner_id −1`, a zero latent, and the node
  and baked counts kept (`EPM.cpp:761-778`, `827-830`). Consumers that key on `winner_id` therefore see −1 whenever the input
  topic was silent this tick. This matters for the thing EPMs.

#### [EPM A.7] `process_every_n_ticks` (`EPM.cpp:780-802`)

- When N > 1, the EPM runs encoder and GNG only on every Nth tick.
- On the other ticks it drops pending input and **republishes its last token with the current `tick_id`**, so consumers see a
  constant token for N ticks.
- The GNG's own step counter (and therefore `lambda_new`, `stale_window_factor` and the health decay) advances only on
  processed ticks. N = 5 at the 50 Hz brain rate (`mj_host/src/DuckBody.hpp:45`) means:
  - 10 GNG steps per second;
  - an insertion check every 125 ticks (2.5 s);
  - a 12000-step stale window of about 20 min.

#### [EPM A.8] Host-side control of EPM learning

**`map_epm`: "map learns only at stops" (`--map-on-stop`).**
- Setup: `mj_host/src/main.cpp:1537` declares the flag and `3421-3422` parses it.
- At start-up the map is frozen (`main.cpp:1903`). It is unfrozen when a stop's Settle phase ends (`main.cpp:2241`). It is frozen
  again at the stop's end (`main.cpp:2461`) or when a stop ends in a fall (`main.cpp:2026`).
- Freezing is done by `IntentAdapter::set_map_learning(false)` (`mj_host/src/IntentAdapter.cpp:438-455`). It sets `min_insertion_error = 1e9`,
  `epsilon_b = 0`, `epsilon_n = 0` and `stale_prune_enabled = false` through `on_param_change`.
- Unfreezing restores the configured values (min_insertion_error 0.06) or the EPM defaults (ε_b 0.05, ε_n 0.003, stale prune
  on), which are captured at construction (`IntentAdapter.cpp:52-68`).

What the freeze does and does not stop, from the GNG code:
- **It stops** insertion (`gng.cpp:409`), the winner and neighbour moves (`gng.cpp:140`, `166`; `apply_neuro_scaling` keeps the zero
  because it multiplies the new base, `EPM.cpp:443`, `631`), and stale pruning.
- **It does not stop** the winner search, the visit counting, edge creation and ageing, the health decay, isolated-node
  removal (`gng.cpp:208`) or health death (`gng.cpp:259-272`). Health death is moot here: with the
  `node_count > 16` floor and `health_death_spares_baked`, it cannot fire on a map of 10 nodes.
- **Side effect: on the walk, baking is unconditional.** The bake gate compares `ema_error` against the *effective*
  `min_insertion_error`, which is now 1e9 (`gng.cpp:291`). Any node that reaches 20 visits during a walk therefore bakes **without the
  consistency check**. Probe log: "baked 3 at stops / 4 on walks", "inserted 10 at stops / 0 on walks"
  (`la_probe.err:103-104`).
- The stderr message "insertion, adaptation and pruning off on the walk" (`main.cpp:1903`, `la_probe.err:65`) is
  therefore partly inaccurate: isolated removal and baking continue.

**Intent brain `set_learning`** (`IntentAdapter.cpp:408-436`) touches only `MotorEPM`/`MotorEPMv2` rate params
(`IntentAdapter.cpp:418`). The other four intent EPMs are **never frozen**: they learn whenever their input topic has data.

**Stop brain `set_learning` / `set_regime_learning` / `freeze_module`** (`mj_host/src/OgmaBrainAdapter.cpp:260-287`, `290-330`)
also touch only `MotorEPM`/`MotorEPMv2` rates (`OgmaBrainAdapter.cpp:308`). **`regime_epm` is never frozen.** It learns on
every tick the stop brain is ticked. Without `--body-predicts` (`main.cpp:1480`, `2073`), that happens only during stops: the Settle
phase (`main.cpp:2217`) and the Brain phase (`main.cpp:2760`).

**Checkpoint restore.**
- **Intent brain.** `--load-brain` restores only `motor_epm_intent` by default (`main.cpp:1500`, `1713-1720`). All five intent
  EPMs therefore **start empty every run**. Probe: the map has 0 nodes at t = 0 and 2 nodes at 0.52 s (`la_probe.jsonl`).
- **Stop brain.** `--stop-load` restores the **whole graph**, `regime_epm` included (`main.cpp:1863-1868`).
- **⚠ The GNG restore does not carry every setting.** `GNG::from_json` rebuilds a fresh `GNG::Config` and reads only `dim`,
  `baking_threshold`, `min_insertion_error`, `lambda_new`, `max_age`, the stale settings, autotune and Kalman fields
  (`gng.cpp:734-750`). The EPM then replaces its whole GNG with it (`EPM.cpp:1027-1029`). Nothing re-applies the config afterwards
  (`OgmaInstance.cpp:174-195`). So after the restore, `regime_epm` runs with these defaults:
  - **`max_nodes = 2000`** (`gng.hpp:82`), not 64;
  - **`health_death_spares_baked = false`** (`gng.hpp:172`), not true;
  - `epsilon_n` 0.003, `alpha` 0.5, `beta` 0.0005, and `mitosis_enabled` true.
  `epsilon_b`, `min_insertion_error` and the mitosis threshold are re-installed every tick from the EPM's base values
  (`EPM.cpp:629-651`), so those three are correct. In this run the dropped settings change nothing observable: 3 nodes is far from either cap,
  and health death is gated by `node_count > 16` (`gng.cpp:263`). But the config values on `a1v2_r19_settle_each.json:271`
  and `:275` are not what runs.

---

### EPM summary table (all six instances, incl. the stop brain's `regime_epm`)


| id | brain | encoder | in dims → pd | max_nodes | bake thr. | min_ins_err | every N | spares baked | output topic | nodes / baked (probe) |
|---|---|---|---|---|---|---|---|---|---|---|
| map_epm | intent | rbf, Halton | 13 → 104 (auto) | 128 | 20 | 0.06 | 5 | true | reality.proprio.place | 10 / 7 |
| object_epm | intent | jl_state | 36 → 104 (explicit) | 128 | 20 | 0.06 | 1 | false (default) | reality.cognitive.object | 18 / 18 |
| thing_epm | intent | rbf, Halton | 8 → 64 (auto) | 64 | 20 | 0.06 | 1 | true | reality.cognitive.thing | 15 / 14 |
| thing_kind_epm | intent | rbf, Halton | 8 → 64 (auto) | 4 | 20 | 0.25 | 1 | true | reality.cognitive.thing_kind | 4 / 4 (full) |
| thing_context_epm | intent | rbf, 7×7 grid | 2 → 48 (auto) | 3 | 20 | 0.25 | 1 | true | reality.cognitive.thing_context | 3 / 3 (full) |
| regime_epm | stop | rbf, Halton | 12 → 96 (auto) | 64 in config, **2000 after restore** | **100** (default) | 0.06 | 5 | true in config, **false after restore** | reality.proprio.regime | 3 / 3 |

**Common to all six:** TLE 0.7/0.3; lambda_new 25; max_age 88; ε_b 0.05; ε_n 0.003; novelty ×1.5, floor 0.01; no
mitosis; no descending prediction; no neuro scaling; Linear gain.

**Sources for the table:**
- Projection dims printed by the binary: `la_probe.err:1-4`, `48`. `object_epm`'s 104 is explicit
  (`a1v2_t11_not_target.json:282`), so it prints no line.
- Node and baked counts: `la_probe.err:111`, `115`, `118-121`.

**RBF σ** was computed by replicating `encoder_rbf.cpp` in Python (`scratchpad/rbf_sigma.py`); the binary does not print it:

| EPM | σ, in normalised [0,1]^d units |
|---|---|
| map | ≈ 0.630 |
| regime | ≈ 0.612 |
| thing / thing_kind | ≈ 0.460 |
| thing_context | ≈ 0.133 (grid spacing 1/6) |

For thing_context, the 7×7 = 49-point grid is truncated to 48 centres, so the (1,1) corner centre is the one dropped
(`encoder_rbf.cpp:89-107`).

---

### This instance: `map_epm` — the place/view vocabulary


**Config:** `mj_host/configs/a1v2_t11_not_target.json:161-207`
- `modality_group` proprio, `modality_name` place, encoder rbf;
- input `reality.proprio.place_in`, `proprio_state_dims` 13, with `dim_min`/`dim_max` per the table below;
- `max_nodes` 128, `master_seed` 4343 (no effect, A.2);
- `process_every_n_ticks` 5, `mitosis_error_threshold` 0.1 (inert, A.4), `health_death_spares_baked` true;
- `min_insertion_error` 0.06, `baking_threshold` 20.

**Input.** The producer is the host, through the IntentAdapter.
- **Building the vector.** `main.cpp:2821-2864` fills a `PlaceInputs` struct (`mj_host/src/IntentAdapter.hpp:33-58`).
  `IntentAdapter::step` then builds the vector as `pose` + `head_yaw` + `cols` (`IntentAdapter.cpp:177-181`, form
  `PlaceForm::ColumnsGaze`, selected because `proprio_state_dims == 13`: `IntentAdapter.cpp:85-86`). It publishes the vector as
  `reality.proprio.place_in` (`IntentAdapter.cpp:200`, topic prefix `IntentAdapter.cpp:107-115`) **before** the intent
  graph ticks (`IntentAdapter.cpp:223`).
- **Pose source.** p and yaw come from `odom.position()` / `odom.yaw()`, the leg odometry updated from feet plus the IMU
  quaternion (`main.cpp:2783-2785`).
- Probe log: "place vector: 13 dims: x, y, cos, sin, head yaw / 1.4, the 8 ToF column ranges / 4 m" (`la_probe.err:24`).

| slots | content | source | dim range → [0,1] |
|---|---|---|---|
| 0-1 | odometry x/2, y/2 | `main.cpp:2821` | [-0.6, 0.6], so it **clamps beyond ±1.2 m** from the odometry origin |
| 2-3 | cos yaw, sin yaw | `main.cpp:2821` | [-1, 1] |
| 4 | head-yaw joint offset / 1.4. **Forced to 0** under `--map-view cloud` | `main.cpp:2822`, `2862` | [-1, 1] |
| 5-12 | without `--map-view cloud`: the nearest Hit per ToF column / 4 m. **In LA1 (`--map-view cloud`): `CloudMap::view()`** | `main.cpp:2856`, `2857-2864` | [0, 1] |

How `CloudMap::view()` fills slots 5-12 in LA1:
- It has 8 azimuth sectors across ±64° (`view_half_fov_` 64; `view_range_` 4.0, `cpp_core/include/ogma/modules/CloudMap.hpp:340`).
  Each sector holds the nearest off-floor voxel (mean height ≥ `break_lo` 0.02 m) over 4 m, and 1.0 means nothing
  (`cpp_core/src/ogma/modules/CloudMap.cpp:1188-1205`). Sector 0 is on the right (`main.cpp:2466-2471` comment).
- `main.cpp` copies the view into `map_view_held` **only while the cloud is open and not a walking cloud**, that is,
  at a stop (`main.cpp:2858-2861`). The array starts at all 1.0 (`main.cpp:1970`) and is **held unchanged through the walk**
  (`la_probe.err:70`).
- **Consequence:** on the walk, only the pose slots 0-3 vary.

- **Conditioning.** There is no mean removal. Every slot is scaled into a fixed range at the source and then affinely mapped
  by `dim_min`/`dim_max` (`a1v2_t11_not_target.json:169-198`).
- **⚠ Pose clamping.** In the 120 s probe, |odom x| or |odom y| exceeded 1.2 m on 3663 of 6000 ticks (`la_probe.jsonl`,
  `odom` field = `odom.position()` x, y, yaw, `main.cpp:2885`, `2894`). On those ticks the pose dims sit at their clamp.

**Computes.**
- Every 5th tick: 13-d → RBF 104-d unit latent → GNG step → dual TLE → token. On the other 4 ticks it republishes the cached token.
- Learning is gated by the host (A.8): it is on only from the end of a stop's Settle phase to the stop's end.
- **Probe:** grew 0 → 10 nodes, all insertions at stops. 7 baked at the end, 4 of them baked on walks under the freeze side effect
  (`la_probe.err:103-104`). Node count steps at 0.5, 3, 5.5, 8, 25.5, 30.5, 38, 43 and 45.5 s, then stays at 10 to 120 s
  (`la_probe.jsonl` `map[3]`). Final ema_tle 0.499; `is_novel` was true on 2065 of 6000 ticks.

**Output.** `reality.proprio.place`. Consumers:

| consumer | reads | anchor |
|---|---|---|
| PlayLoop `play` | `winner_id` as the current node; `tle` as its novelty (`novelty_source` "tle") | config `a1v2_t11_not_target.json:212`, `220`; code `cpp_core/src/ogma/modules/PlayLoop.cpp:244-250` |
| CloudMap `cloud` | `winner_id`, histogrammed while a cloud is open, as the **cache key** for the cloud by place | config `:231`; code `CloudMap.cpp:697-700` |
| LoopCompetence `comp_play` | `tle` as the play loop's objective (sign +1) | config `:445-447`; code `cpp_core/src/ogma/modules/LoopCompetence.cpp:131-132` |
| IntentAdapter (host) | `tle`, `is_novel`, `winner_id`, `just_baked`, `node_count`, `baked_count`, `quant_error`, `expected_error`, `transition_surp`, `pruned_ids` | `IntentAdapter.cpp:225-230`, read **after** the tick |

The host uses those values in three ways:
- **(a) Walker sense slot.** `tle` is clamped to [-1,1] and fed as a slot of `reality.proprio.sense`, the
  walker's (`motor_epm_intent`) load vector via `twist_bridge` with `load_slots` 18 (`IntentAdapter.cpp:153-158`;
  `a1v2_t11_not_target.json:38-39`). It is base slot 11, which becomes index 13 once the `--intent-fore-sense` and
  `--intent-range-sense` slots are inserted at the front (`IntentAdapter.cpp:161-165`). It is the previous tick's value,
  further held 5 ticks by the sub-rate.
- **(b) Stop-gaze novelty rule.** Under `--stop-gaze-sweep` with `--stop-gaze-learn 0.5` and `--stop-gaze-residual 1.0`:
  - At hold tick 11, the view is "arrival-novel" if `quant_error > 1.0 × expected_error`.
  - After that, `view_novel` holds while `quant_error > 0.5 ×` the arrival QE.
  - `view_novel` extends a sweep hold (`main.cpp:2305-2318`; flags `main.cpp:3401-3410`; `sweep_on` `main.cpp:1931`).
- **(c) Logs and counters.** The JSON `map` field, the baked-id set and the prune counters (`main.cpp:2108-2115`, `2914-2917`).

`map_tle_long_` / wander boredom (`IntentAdapter.cpp:311-315`) needs `--wander-bored`, which is not in the LA1 args, so it is inactive.

**Learning summary.** GNG ε_b 0.05 (damped), ε_n 0.003, insertion every 25 GNG steps (2.5 s) when `q.ema_error ≥ 0.06`.
It is active **only during stops** after the Settle phase. On walks: no insertion and no adaptation, but visits accumulate and
baking skips the error check.

## 2.4 `play` — PlayLoop (preliminaries shared with `seek` and `outcome`)

Config: `mj_host/configs/a1v2_t11_not_target.json` (INTENT brain). Tick order (preamble): … map_epm, **play**, cloud, object_epm, thing_epm, thing_kind_epm, thing_context_epm, **seek**, comp_seek, comp_play, voter_loops, arbiter, **outcome**. All three read their inputs with `bus_->last_value(...)` (pull-by-value), so a producer later in the order is seen one tick late (e.g. `seek` reads `outcome`'s previous-tick `outcome_need` / `outcome_pull`).

Tick rate: the host brain runs at `kBrainHz = 50.0` (`mj_host/src/DuckBody.hpp:45`). `BearingSeekLoop` hard-codes 50 Hz in its tick→seconds conversions (`cpp_core/src/ogma/modules/BearingSeekLoop.cpp:342, 348, 482, 488, 490, 493, 520, 578, 628, 640, 641, 660`). `SkillOutcomeLoop` works in ticks (config comment: 1500 ticks = "30 s"). `PlayLoop` has no seconds conversion; its odometry is in command-units-per-tick (below).

Restore: only `motor_epm_intent` is restored from the checkpoint by default (`mj_host/src/main.cpp:1500`, rationale at `main.cpp:1713-1716`), so **all three loops start empty every run** (no map graph, no held target, no outcome table).

Config params vs code: every non-`_comment` param set in the config for these three modules is read in the module's `on_setup` (PlayLoop.cpp:157-182; BearingSeekLoop.cpp:201-245; SkillOutcomeLoop.cpp:106-133). No unread params found.

---

### `play` in detail


**Role:** epistemic exploration. Keeps its own graph over the map EPM's place vocabulary, scores each place by how badly the map EPM predicts it (TLE), and points the body toward the most novel reachable place (or wanders by run-and-tumble when no neighbour is more novel).

#### Inputs
| Topic | Producer | Layout / use | Anchor |
|---|---|---|---|
| `reality.proprio.place` (RealityToken) | `map_epm` (EPM, modality proprio/place, `process_every_n_ticks 5` → republishes its last token on 4 of 5 ticks, `cpp_core/src/ogma/modules/EPM.cpp:131, 787-788`) | `winner_id` = current node; `tle` = novelty (novelty_source `tle`) | PlayLoop.cpp:244-250 |
| `reality.proprio.heading` (ProprioToken) | host IntentAdapter: `publish("heading", {heading_})` = unwrapped odometry yaw, rad | `values[0]`, multiplied by `heading_sign` | IntentAdapter.cpp:207, 121-126; PlayLoop.cpp:251-252 |
| `reality.proprio.vel_ego` (ProprioToken) | host IntentAdapter: `{unit(vel_body[1]/0.3), unit(vel_body[0]/0.4)}` = [lateral, forward] in command units clamped to ±1 (kTwistRangeVy 0.3, Vx 0.4) | `values[0]` = vlat, `values[1]` = vfwd | IntentAdapter.cpp:220; IntentAdapter.hpp:29; PlayLoop.cpp:253-257 |
| `events.eat` (EnvEvent, default) | nothing on the duck | eat-credit telemetry only; stays 0 | PlayLoop.cpp:114, 184-191, 308-310 |

Host prefix: the adapter's `publish(sensor, …)` writes `reality.proprio.<sensor>` with producer "host" (IntentAdapter.cpp:107-115).

#### Key parameters
Config: `pi_cell_size 0` (node = map EPM winner, not an odometry grid cell; PlayLoop.cpp:273-277), `wander_stall_ticks 0` (forced wander off; :367), `frontier_bias 0` (wander not biased outward; :427), `novelty_source "tle"` (:247-249), `explore_seed 11` (RNG seeded at :179-180), `heading_sign -1` (incoming heading negated so the loop's clockwise-positive frame becomes a rotation of the duck's CCW yaw rather than a reflection; rationale PlayLoop.hpp:192-200), `play_value_topic reality.cognitive.play_value`.
Defaults relied on (PlayLoop.hpp:119-160; schema PlayLoop.cpp:81-99): `gamma 0.85`, `vi_sweeps 8`, `tle_ema_alpha 0.1`, `tle_peak_decay 0.0005`, `hab_rise 0.1`, `hab_decay 0.002`, `explore_cycle 30`, `explore_tumble_range 1.5708`, `commit_hold false`, `lookahead false`.

#### What it computes each tick (PlayLoop::tick, PlayLoop.cpp:240-486)
1. **Path integration** (its own, separate from the host odometry): `odo += (−vfwd·sin h + vlat·cos h, −vfwd·cos h − vlat·sin h)` with h = −yaw; units are command-units per tick, not metres (:258-259).
2. **Graph building:** a winner id never seen before is added to `value_` (`grew = true`) — note: first *appearance* of an id, not baking (:281-283). On a change of winner, a **directed edge** cur→new is created/updated, storing the circular mean of the heading at transition (:284-289; `Edge::heading` :50-53). Each node's **position** is the running mean of the loop's odometry over all ticks it was the winner (:293-296).
3. **Novelty:** per-node EMA (α 0.1) of the place token's `tle`, first value initialising (:298-301).
4. **Habituation:** all nodes decay ×(1−0.002) per tick, the current node rises `h += 0.1(1−h)` (:305-306).
5. **Value iteration** (8 Gauss–Seidel sweeps): `V[n] = novelty[n]·(1−hab[n]) + 0.85·max_{m ∈ out-edges(n)} V[m]` (:194-211, called :312).
6. **Choosing the target ("the most novel reachable node"):** with `commit_hold`/`lookahead` off, the committed next node is kept only if it is still an out-neighbour of the current node and still strictly uphill (:340-344); otherwise `next_node` = the out-neighbour with highest V (:347-357). It is a **one-hop climb on the value field along observed transition edges**, not a global argmax. `climbing = (V[next] > V[cur] + 1e-4)` (no forced wander since `wander_stall_ticks 0`) (:361-368).
7. **Bearing:** if climbing, target heading = bearing from the current node's mean position to the next node's mean position (`geo_bearing`, :213-222, fallback to edge heading :405-407); delta = target − heading, with a turn-commit hysteresis (enter turning when |δ|>90°, leave when <36°, cap 0.92π) (:408-413). Otherwise **wander**: run-and-tumble — hold `explore_dir` for 30 ticks; if the local novelty did not rise over the run, re-draw direction uniformly within ±90° of the current heading (seeded RNG) (:414-466).
8. **play_value:** `novel_ref` rises toward the node TLE on a `grew` tick (rate 0.1) and fades ×(1−0.0005)/tick; `climb_v = clamp(V[next]/max(novel_ref,1e-3))`, `wander_v = 1 − hab[cur]`; `play_value = max(climb_v, wander_v)` ∈ [0,1] (:391-397).

#### Outputs
| Topic | Layout | Consumers |
|---|---|---|
| `percept.play_bearing` | `[fx = sin δ (+right), fy = cos δ (+forward), 0]` (:469-475) | IntentAdapter: when `arbiter.gain.play > 0.5` (and klino/vision not winning) the bearing sets the heading reference `heading_ref = heading − atan2(cx, cy)` (steer code 1) (IntentAdapter.cpp:250-257, 284-307); also logged as `pb` (IntentAdapter.cpp:270-271, main.cpp:2976) |
| `reality.cognitive.play_value` | `[play_value]` (:477-485) | EFEArbiter `play_value_topic`. **In `scoring_mode "precision"` (this config) the value is not used in the score**: G_play = play_weight·(1−hunger)·trust_play, where hunger = seek_value (EFEArbiter.cpp:312-313); `play_value_` enters scoring only in the `efe` branch (EFEArbiter.cpp:396, branch 318-411). The non-empty topic only switches `play_active_` on (EFEArbiter.cpp:236). |

**`reality.loop.play` is NOT published by PlayLoop.** It is published by `comp_play` (LoopCompetence, `modality_group loop`, `modality_name play` → topic `reality.loop.play`, LoopCompetence.cpp:110, 181), whose objective is the **place token's `tle`** with sign +1 (play "works" while place TLE rises while it drives; LoopCompetence.cpp:131-133; config `comp_play`). `voter_loops` fuses `reality.loop.*` into trust weights for the arbiter.

#### Learning
In-run only, no error-driven weights: graph nodes + directed edges with circular-mean headings, node mean positions, per-node TLE-EMA novelty, habituation, `novel_ref`. Nothing is ever deleted (no pruning of graph nodes). Not restored from the checkpoint. Observed (probe log `la_probe.err:116`): at 120 s `{"climbing":true,"n_nodes":10,"next_node":1}`.

---

## 2.5 `cloud` — CloudMap


**Type:** `CloudMap` (`cpp_core/include/ogma/modules/CloudMap.hpp`, `cpp_core/src/ogma/modules/CloudMap.cpp`).
**Tick position:** 5th module of the intent brain. It runs after `map_epm` (2nd), so it sees this tick's place winner, and before `seek` (10th), so anything it reads from `seek` comes from the previous tick (config order, `a1v2_t11_not_target.json:161, 227, 390`).

**Role in plain words.** CloudMap builds a 3-D voxel point cloud from the head's 8×8 time-of-flight (ToF) depth sensor. It accumulates the cloud while the robot stands, and also while it walks. It reduces the cloud to:
- a 36-number "floor-break profile";
- the nearest **small thing**, found by a geometric "stack rule", with its descriptor, its body-frame bearing and its context;
- the nearest **mover**, a cluster whose voxels are young;
- a count of tall structure around the seek loop's current target.

Stop clouds are cached and keyed by the place map's winner node. CloudMap is a frozen geometric reduction. It learns nothing and is not a clusterer in the EPM sense (header `CloudMap.hpp:44-47`).

---

### [CloudMap 1] INPUTS

| Topic | Producer | Kind | Layout | Read at |
|---|---|---|---|---|
| `reality.proprio.tof_points` | host (`IntentAdapter.cpp:202-203`, filled in `mj_host/src/main.cpp:2822-2854`) | ProprioToken | 392 floats, see 1.1 | `CloudMap.cpp:670-677` |
| `reality.proprio.place` | `map_epm` (EPM, modality `proprio.place`, `a1v2_t11_not_target.json:164-165`) | RealityToken | only `winner_id` is used | `CloudMap.cpp:700-702` |
| `percept.seek_bearing` | `seek` (BearingSeekLoop `output_topic`, `a1v2_t11_not_target.json:395`) | ProprioToken | `[cx=+right, cy=+forward, value, chasing]`; the 4th value exists because `publish_chase_flag: true` (`BearingSeekLoop.cpp:436-438`) | `CloudMap.cpp:1024-1030, 1110-1111` |
| `reality.cognitive.seek_range` | `seek` (`range_topic`, `a1v2_t11_not_target.json:397`) | ProprioToken | `[range_left]` in metres (`BearingSeekLoop.cpp:444-448`) | `CloudMap.cpp:1026-1027, 1112-1113` |

**Subscribe or publish?** CloudMap **subscribes** to `percept.seek_bearing` and `reality.cognitive.seek_range`. Both appear in `input_topics()` (`CloudMap.cpp:37-38`) and are read with `bus_->last_value`. `seek` is their only publisher. `percept.target_tall` is a CloudMap **output** (`CloudMap.cpp:49`), consumed by `seek` as its `yield_topic` (`a1v2_t11_not_target.json:413`).

#### [CloudMap 1.1] `reality.proprio.tof_points` (host side)

The layout is `[still, yaw, trunk_z, odom_x, odom_y, 64×(x,y,z), origin(x,y,z), 64×(x,y,z) free-ray ends]`. That is 5 + 192 + 3 + 192 = 392 floats (`IntentAdapter.hpp:36-51`; array size at `:51`).

- **Frame.** The points are in the **gravity-levelled trunk (body) frame**: origin at the trunk, +z up by the measured gravity, +x/+y turning with the body (`Tof.hpp:22-26`). They are **not** in the world frame. The point is `point_level` rotated by `level_from_gravity(body.gravity())` (`Tof.cpp:78-81, 141`).
- **z values.** Each z is height above the floor: `point_level[2] + p[2]`, where `p` is the odometry position (`main.cpp:2841`).
- **Header values.**
  - `yaw` is `odom.yaw()`, the IMU yaw (`main.cpp:2831`; `Odometry.hpp:10-12`).
  - `trunk_z` is `p[2]` (`main.cpp:2832`).
  - `odom_x` and `odom_y` are the leg-and-IMU dead-reckoned position (`main.cpp:2833-2834`; `Odometry.hpp:1-17`).
- **Which zones become points.** A zone carries a point if its class is Hit or Floor. TooClose zones count only with `--contact-cloud`, which LA1 does not use (`main.cpp:2836-2838`). Other zones are NaN (`main.cpp:2839-2841`).
- **Origin.** The ray origin is the sensor position in the same levelled frame, with z above the floor (`main.cpp:2843-2846`; `Tof.cpp:80-81`).
- **Free rays (`--tof-free-rays`, set in LA1, `main.cpp:3574-3575`).** Each Empty zone contributes its beam end at `kMaxRangeM` = 4.0 m in the same frame. All other zones are NaN (`main.cpp:2847-2853`; `Tof.cpp:115-122`; `Tof.hpp:36`).
- **Stillness flag.** `still` = `(g_stop_is_still && stop_phase ∈ {Brain, Walker}) || (gravity_z < -0.999 && max|gyro| < 0.15)` (`main.cpp:2827-2828`).
  - LA1 sets `--stop-is-still` (`main.cpp:3588-3589`), so a stop's standing phases always count as still. This is efference rather than an oracle (rationale at `main.cpp:1302-1307`).
- **Rate.**
  - The ToF is cast only every 4th host tick: `if (t % 4 == 0) tof.sense(...)` (`main.cpp:2787-2788`). That is 12.5 Hz at `kBrainHz = 50` (`DuckBody.hpp:45`).
  - `tof_points` is rebuilt from `tof.zones()` and published on **every** brain tick (`main.cpp:2822-2854`; `IntentAdapter.cpp:202-203`).
  - The intent brain also ticks every host tick (`main.cpp:2041, 2080`).
  - **Consequence:** CloudMap adds each physical cast on 4 consecutive ticks. See §7, item 1.
- **Conditions.** Publication is on only with `--cloud` (`g_cloud_voxel = 0.04`, `main.cpp:3443-3445, 1686`). Simulated sensor: an 8×8 grid of rays, 45° field of view, 4 m range, cast against world geometry only (`Tof.hpp:1-9, 35-36`).

---

### [CloudMap 2] KEY PARAMETERS

Columns: **Set** is the value in the config (`a1v2_t11_not_target.json:229-269`). **Default** is the module default (schema `CloudMap.cpp:54-289`, member initialisers `CloudMap.hpp:267-360`, read in `on_setup` `CloudMap.cpp:392-453`).

| Param | Set | Default | Meaning |
|---|---|---|---|
| input_topic | reality.proprio.tof_points | "" (inert) | the cast (`:56-63`) |
| place_topic | reality.proprio.place | "" | the cache key source (`:64-69`) |
| output_topic | reality.proprio.cloud | "" | the 36-dim profile (`:70-75`) |
| change_topic | percept.cloud_change | "" | `[new_fraction, revisit_change]` (`:76-80`) |
| voxel_m | 0.04 | 0.04 | voxel edge in metres (`:81-83`) |
| break_lo | 0.02 | 0.02 | above this height a return is "not floor" (`:84-85`) |
| break_hi | 0.20 | 0.20 | below this height a return is "standing on the floor" (`:86-88`) |
| half_fov | 40.0 | 40.0 | ± degrees covered by the profile's 8 sectors (`:89-91`) |
| max_range | 2.5 | 2.5 | horizontal cut in metres; also the proximity scale (`:92-94`) |
| still_ticks | 25 | 25 | consecutive still ticks before a stop cloud opens (`:110-113`) |
| cache_size | 8 | 8 | LRU cache of stop clouds (`:114-116`) |
| new_window_ticks | 50 | 50 | window for `new_fraction` (`:168-173`) |
| things_topic | reality.proprio.thing | "" | 8-dim descriptor (`:117-124`) |
| thing_bearing_topic | percept.thing_bearing | "" | bearing to the attended thing (`:125-130`) |
| small_top | 0.16 | 0.16 | top of a small thing's stack, metres (`:131-134`) |
| small_ext | 0.2 | 0.20 | largest footprint span of a small thing (`:135-136`) |
| small_ext_min | 0.08 | 0.0 | smallest footprint span; 0.08 m = 2 voxels (`:137-142`) |
| gap_min | 0.1 | 0.10 | floor on the stack-chain gap (`:148-150`) |
| gap_k | 0.12 | 0.12 | gap growth per metre of range (`:151-153`) |
| things_range | 0.0 | 0.0 | 0 means use max_range (2.5 m) (`:154-156`) |
| things_every | 4 | 4 | ticks between cluster recomputes (`:165-167`) |
| walk_cloud | true | false | also accumulate while walking (`:157-161`) |
| mover_topic | percept.mover_bearing | "" | the mover token (`:205-209`) |
| mover_ext_max | 0.35 | 0.0 (any) | widest footprint a mover candidate may have (`:282-285`) |
| mover_range | 1.2 | 1.5 | range limit for starting a new mover (`:218-221`) |
| things_skip_movers | true | false | never attend a young cluster (`:222-225`) |
| mover_isolated | true | false | a mover candidate needs `tall_near == 0` (`:231-233`) |
| target_topic | percept.seek_bearing | "" | input (`:196-199`) |
| target_range_topic | reality.cognitive.seek_range | "" | input (`:200`) |
| target_tall_topic | percept.target_tall | "" | output (`:201-203`) |
| free_rays | true | false | trace rays to record free space (`:234-238`) |
| small_needs_top | true | false | "small" requires a seen top (`:239-242`) |
| context_topic | reality.proprio.thing_context | "" | `[on_line, near_tall]` (`:248-252`) |
| line_tol_k | 0.049 | 0.0 | the "line" refusal (`:243-247`) |
| mover_range_hold | 2.5 | 0.0 | range limit for a mover already being followed (`:264-266`) |
| mover_hold_any_age | true | false | a held mover is exempt from the youth gate (`:268-271`) |
| mover_hold_ticks | 50 | 0 | the hold survives this many ticks of misses (`:278-281`) |
| mover_not_target_m | 0.25 | 0.0 | a new mover must be this far from the seek target (`:275-277`) |

**Defaults the config relies on (not set):**
- `walk_reset_m` 1.0 (`CloudMap.hpp:277`; `.cpp:162-164`)
- `walk_things` **true** (`hpp:282`; `.cpp:174-178, 423`): the things reduction also runs on walking clouds
- `move_ticks` 25 (`hpp:342`; `.cpp:103-109`)
- `max_voxels` 400000 (`hpp:341`)
- `view_half_fov` 64°, `view_range` 4.0 (`hpp:340`)
- `things_shape` false, so the full 8-dim descriptor is used (`.cpp:415`)
- `things_age_dim` false (`.cpp:447`)
- `things_isolated` false (`.cpp:446`)
- `mover_window_ticks` 25, i.e. 0.5 s (`hpp:284`)
- `mover_age_k` 0.06 (`hpp:284`)
- `mover_weighted` true (`hpp:284`): age is hit-weighted
- `mover_hold_gate` 0.4 m (`hpp:290`)
- `mover_hold_min_v` 0, so off (`.cpp:439`)
- `mover_vacated` 0 and `vacate_window_ticks` 0, so vacated-voxel ray traversal is **off** (`.cpp:424-427`). The probe log's `vacated: 0` agrees.
- `iso_height` 0.25, `iso_radius` 0.25 (`hpp:327`)
- `target_iso_radius` 0.35 (`hpp:312`)
- `line_close_k` 0.0982 rad (`hpp:360`)

---

### [CloudMap 3] WHAT IT COMPUTES EACH TICK

The entry point is `CloudMap::tick`, `CloudMap.cpp:666-771`.

#### [CloudMap 3.1] Opening and closing the cloud (walk_cloud = true branch, `CloudMap.cpp:680-689`)

Two run counters track the stillness flag: `still_run_` and `move_run_` (`:680`). With `walk_cloud` on, a cloud is **always open**. There are two kinds:

- **Stop cloud → walking cloud.** When a stop cloud is open and `move_run_ ≥ move_ticks` (25 ticks = 0.5 s of not-still), the stop cloud is filed and a walking cloud opens (`:685`).
- **Walking cloud → stop cloud.** When a walking cloud is open and `still_run_ ≥ still_ticks` (25), the walking cloud is filed and a stop cloud opens (`:686`).
- **Walking cloud → new walking cloud.** When a walking cloud's odometry displacement from its anchor exceeds `walk_reset_m` (1.0 m, straight-line from the anchor), it is filed and a new walking cloud is anchored at the current pose (`:687`).
- **No cloud open** (first tick only): a cloud opens, walking if `still_run_ < still_ticks` (`:688`).
- **Every tick, still or not, the cast is added** (`:689`).
  - The non-walk branch skips casts on moving ticks (`:696`), but that branch is not used in LA1.

**Opening** (`open_cloud`, `:455-479`) clears the voxels, the free-column record, the vacated list and the place histogram. It stores the anchor (yaw, x, y) and the open tick. It resets `points_`, `break_vox_`, `new_frac_` and `revisit_change_` = -1. A followed mover's point and step are carried into the new anchor frame (`:459-467`; see 3.6).

#### [CloudMap 3.2] Accumulation (`add_cast`, `CloudMap.cpp:481-594`)

1. **Rotate.** Each finite point (x, y, z) is rotated by R(+d), where d = yaw − anchor_yaw, wrapped (`:483-488, 502-503`). This de-rotation sign is pinned by the header (`hpp:21-30`).
2. **Translate (walking cloud only).** The odometry displacement from the anchor, turned into the anchor frame, is added (`:490-495`).
   - On a stop cloud, translation is ignored by design (`hpp:31-32`).
3. **Range cut.** Points with horizontal distance from the **anchor** above `max_range` (2.5 m) are dropped (`:504`).
4. **Voxelise.** Index = `floor(coord / voxel_m)` (`:506-508`). Each voxel keeps:
   - `hits`
   - `first` tick and `last` tick
   - `zsum`, the sum of point heights; mean height = zsum / hits
   - (`hpp:229`; `.cpp:509-516`)
5. **Classify by mean height.** A voxel is classified everywhere by the mean height of its points, not by its centre (`hpp:225-228`).
   - Exception: the diagnostic `break_vox_` is counted once, on a voxel's first hit, by that point's height (`:510-513`).
6. **Hard cap.** Accumulation stops at 400 000 voxels (`:482`).

**Free rays (`free_rays` true, `CloudMap.cpp:559-592`).** Every ray of the cast is walked from the sensor origin in half-voxel steps, in the same de-rotated and translated frame.
- A returning ray (Hit/Floor point) stops 1.5 voxels short of its return (`:571-572`).
- An empty ray (from the appended free-ray block) runs toward its 4 m end (`:573-574`). Each walk is cut at `max_range` horizontal distance from the origin (`:585`), and stops when it dips below `break_lo` into the floor (`:586`).
- For every (ix, iy) column the walk crosses, `free_col_` keeps **the highest free sample height** (`:587-589`).
- Free rays add no occupancy: a ray passing through a voxel does not clear that voxel.
  - Rays only mark free space per column. The separate "vacated" mechanism that tracks rays passing through occupied voxels is off (`vacate_window_ticks` 0, `:523`).
- `free_col_` is cleared only when a cloud opens (`:458`).

#### [CloudMap 3.3] Place histogram, filing and the place-keyed cache

**Place histogram.**
- Every tick a cloud is open, `map_epm`'s `winner_id` (if ≥ 0) is counted into `winner_hist_` (`:700-702`).
- The **cache key is the modal winner** while the cloud was open (`:602-604`). The rationale is that the map flickers between nodes (`.cpp:65-68`).

**Filing** (`file_cloud`, `:596-664`):
- Stores the viewer payload (`:606-613`) and the final clusters (`filed_things_`, `:615`).
- Clears the things and the attended thing (`:616-618`).
- **Only stop clouds are cached.** A cloud is cached only if `cache_size > 0`, the key is ≥ 0, and it is **not a walking cloud** (`:626, 650`).
- **Revisit change.** If a stop cloud's key is already cached, `revisit_change` = (voxels of the new cloud absent from the cached cloud of that place) / (voxels of the new cloud), where each new voxel centre is carried through the rigid transform between the two dead-reckoned anchors (`:626-648`). `revisit_dist` is the distance between the two anchors (`:630`).
- **Storage.** The cloud is then stored with its anchor pose and filing tick, evicting the least-recently-filed entry when 8 are held (`:650-661`).
- **The cache stores and compares only.** It is never read back into the live cloud.

#### [CloudMap 3.4] Change within the sweep (`:704-715`)

`new_fraction` = (voxels last touched in the last 50 ticks whose **first** sighting was also within those 50 ticks) / (voxels last touched in the last 50 ticks).

#### [CloudMap 3.5] THINGS: the stack rule

The stack rule is `cluster_things(since_tick)` (`:776-952`). `things_on_` is true because a things topic is set (`:420`). `walk_things` is true, so it runs on stop clouds and walking clouds alike (`:718-724`). The full recompute `update_things` runs on ticks with `tick_id % 4 == 0` (`:723`). On the ticks between, only the attended thing's bearing is re-aimed for yaw and body displacement (`:724, 970-974`).

**Clustering** (frozen geometry, not learned):

1. **Columns.**
   - Every voxel with mean height ≥ `break_lo` (0.02 m) is placed into its (ix, iy) column (`:781-789`).
   - A column is a **seed** if it holds any voxel with mean height < `break_hi` (0.20 m) (`:788`).
   - Floor voxels (mean height below 0.02 m) are excluded.
2. **Clusters.** Seed columns are joined into clusters by **8-connected flood fill** (`:829-846`).
3. **Per-cluster quantities:**
   - centroid `cx, cy`: mean of the column centres, cloud frame (`:850-862`)
   - `rng` = |(cx, cy)|, range from the **anchor** (`:888`)
   - `ext` / `ext_min`: larger and smaller bounding-box span, plus one voxel (`:889-890`)
   - `lo`: lowest mean height in the break band (`:858`)
   - `hits`: sum over all heights in the footprint (`:857`)
   - `ncols` (`:861`)
   - `age` / `age_w`: mean and hit-weighted mean of (last − first) ticks (`:864-869`)
   - `tall_near`: voxels of the whole open cloud with mean height ≥ `iso_height` 0.25 m within `iso_radius` 0.25 m of the centroid. It is computed because `mover_isolated` is true (`:875-887`).
4. **Stack top.**
   - gap = max(gap_min 0.10, gap_k 0.12 × rng) (`:892`).
   - Collect every voxel height over the 3×3-**dilated** footprint (`:893-906`), sort the heights, and climb from `lo` while each next height is within the gap of the current top (`:907-913`).
   - `chain` = round((top − lo) / voxel) + 1 (`:914`).
5. **Small** (`:915`): `top < small_top (0.16)` AND `ext ≤ small_ext (0.20)` AND `ext ≥ small_ext_min (0.08)`.
6. **Top seen** (`small_needs_top`, `:916-922`).
   - `seen_above` is the highest free-ray sample over the cluster's own (undilated) columns. A cluster stays small only if `seen_above ≥ top + one voxel` (0.04 m). In plain words, a ray must have passed through the cluster's columns at least one voxel above its top.
   - The default when no ray has passed is -1 (`hpp:113`).
7. **The line** (`line_tol_k` 0.049 > 0; computed only for clusters still small, `:923-947`).
   - **Tall columns:** columns of the open cloud holding a voxel with mean height ≥ `small_top` (0.16 m). Note that the threshold is `small_top`, not `iso_height` (`:808-815`).
   - **Closing:** the tall columns are morphologically closed (dilate, then an erosion test) with a disc of radius max(1 voxel, 0.0982 × column range / voxel_m) (`:805-827`).
   - `on_line` = the share of cluster columns that have a closed column within a disc of radius max(1, 0.049 × rng / 0.04) voxels (`:925-933`).
   - If `2·on ≥ ncols`, the cluster is **not small**: it is a fragment of structure (`:946`).
   - `near_tall` = exp(−d / max(voxel_m, 0.0982 × rng)), where d is the distance in metres from the cluster's columns to the nearest tall column. It is 0 if there are no tall columns (`:934-945`).
8. **Sort.** Clusters are sorted by anchor range (`:950`).

**Attention** (`update_things`, `:1168-1186`). The attended thing is the cluster with the **smallest body-relative range** (`body_rel`, `:992-1007`) within reach (`things_range` 0 → `max_range` 2.5 m) that passes all of these:
- it is `small`;
- it is not young: with `things_skip_movers`, a cluster is skipped if its `age_w < mover_age_k (0.06) × the oldest cluster's age_w` in the whole cloud (`:1179-1180`).

`things_isolated` is off, so the attended thing need not be isolated.

**Runtime check (observed, not from code).** In the probe log, a thing was attended on 435 of the 1,499 logged cast ticks, 294 of them on walking clouds (`thg` records).

#### [CloudMap 3.6] MOVERS (`update_movers`, `:1047-1105`; recomputed every 4 ticks, `:746`)

**Recency window.** Clusters are formed by the same stack-rule clustering, run only over voxels last seen in the last `mover_window_ticks` = 25 ticks (0.5 s) (`cluster_recent`, `hpp:204-206`). Size and small/large status do not matter here.

**Preconditions.**
- **Vouching period.** For the first 2 × 25 = 50 ticks after a cloud opens, no new mover is accepted (`:1054-1055, 1061`). Only a followed mover passes, when `follow` is true (`:1053`).
- **Cluster count.** At least 2 clusters are needed, or 1 when following (`:1057`).

**Candidate test.** Let `oldest` = the maximum `age_w` among the window's clusters (`:1058-1059`). A cluster is a candidate if all of these hold:
- **Range:** body range ≤ `mover_range` (1.2 m), unless it is *held* (see below) (`:1076`).
- **Young:** `age_w < 0.06 × oldest` (`:1061, 1076`).
  - The threshold scales with how long the cloud has watched.
  - It is **waived** for a held cluster, because `mover_hold_any_age` is true and `mover_hold_min_v` = 0 makes `moving_hold` always true (`:1075-1076`).
- **Not the seek target:** unless held, it is more than `mover_not_target_m` 0.25 m from the seek target placed in the cloud (`:1078`).
  - The seek target is placed by `target_in_cloud` (`:1021-1045`) from `percept.seek_bearing` and `seek_range`.
  - It is ignored if the seek token's `value` ≤ 0 or its 4th value (`chasing`) > 0.5 (`:1028-1030`).
  - "The thing walked to is not a mover": the static target reads young as it comes into view (`hpp:301-304`).
- **Size:** `ext ≤ mover_ext_max` (0.35 m) (`:1079`).
- **Isolated:** `tall_near == 0` (`:1080`).

**Held cluster.** A cluster is *held* (`:1074`) when all of these are true:
- `mover_range_hold` > 0 (it is 2.5);
- a follow is live (a mover was published at the last recompute, or `hold_left_ > 0`);
- its body range ≤ 2.5 m;
- its centroid is within `mover_hold_gate` 0.4 m of the predicted point.

The predicted point is the last mover position plus (1 + misses) × its last per-recompute displacement (`:1065-1066`).

**Choice and follow-up.**
- The **nearest** candidate by body range wins (`:1082`).
- On a hit, the per-recompute displacement and position are updated, and `hold_left_` is set to 50 ticks (`:1084-1091`).
- On a miss, `hold_left_` drops by 4 per recompute while the predicted point coasts forward (`:1096-1101`). 50 ticks is about 12 recomputes, roughly 1 s.
- The follow survives a cloud re-anchor: its point and step are transformed through the world into the new anchor frame (`:459-467`).

**Diagnostic counter.** `mover_cands_` is a **cumulative count of candidate clusters over all recomputes**, not a count of distinct movers (`:1081`). The probe log shows 271 over 120 s.

**Not a learned or EPM prediction.** The "prediction" here is constant-velocity extrapolation of the centroid. CloudMap does not decide that two sightings are one mover; that is `seek`'s job (`hpp:192`).

#### [CloudMap 3.7] The target's surroundings (`publish_target_tall`, every tick, `:1107-1148`)

If a cloud is open and the seek token has value > 0, a non-zero bearing and range > 0, the target is placed in the cloud frame:
- body frame: x_fwd = cy/|c| · range, y_left = −cx/|c| · range;
- then turned by +(yaw − anchor) and shifted by the body's displacement (`:1114-1128`).

The module then counts:
- `target_tall_`: open-cloud voxels with mean height ≥ `iso_height` 0.25 m within `target_iso_radius` 0.35 m of the target (`:1129-1137`).
- `target_small_`: small things within max(0.12, 0.175) = 0.175 m (`:1138-1139`).

Unlike `target_in_cloud`, this does **not** skip while seek is chasing.

---

### [CloudMap 4] OUTPUTS

All outputs are ProprioTokens published by `cloud`. Consumers were found by grepping the three LA1 configs and `mj_host/src`. The head and stop configs reference none of these topics.

| Topic | When | Layout and normalisation | Consumers |
|---|---|---|---|
| `reality.proprio.cloud` | every tick once input exists (`:752-760`) | 36-dim profile, see 4.1 | `object_epm` (EPM, jl_state, proprio_state_dims 36, projection_dim 104; `a1v2_t11_not_target.json:274-283`). Nothing in the configs or `mj_host/src` was found to subscribe to `object_epm`'s output `reality.cognitive.object` |
| `percept.cloud_change` | every tick (`:761-770`) | `[new_fraction ∈ [0,1], revisit_change ∈ [0,1] or -1]` | **none found** in the three configs or `mj_host/src` |
| `reality.proprio.thing` | every tick **while a thing is attended** (`:725-732`) | 8 dims, see 4.2 | `thing_epm` (`:291-297`), `thing_kind_epm` (`:328-334`). The probe log reports proprio_state_dims 8 for both |
| `reality.proprio.thing_context` | every tick while attended (`:733-741`) | `[on_line, near_tall]`, each clamped to [0,1] | `thing_context_epm` (`:365-371`) |
| `percept.thing_bearing` | every tick (`:743, 978-990`) | 4 dims, see 4.3 | `seek` `bearing_topic` (`:393`), `outcome` `bearing_topic` (`:498`) |
| `percept.mover_bearing` | every tick (`:745-749, 1150-1166`) | 6 dims, see 4.4 | `seek` `mover_topic` (`:407`) |
| `percept.target_tall` | every tick (`:750, 1141-1147`) | `[tall_count (raw voxel count), range (the seek range read in, m)]`; zeros when no target or no cloud | `seek` `yield_topic` (`:413`) with `chase_yield_tall: 1`, so one tall voxel at a chased target makes the chase yield. `static_yield_tall` is unset and defaults to 0, which is off (`BearingSeekLoop.hpp:191`; `.cpp:425`) |

**Host-side readers (not bus topics; the host calls module accessors through `IntentAdapter.cpp:563-632`):**
- `--map-view cloud`: while a stop cloud is open, `view()` replaces the map EPM's 8 column inputs (`main.cpp:2857-2864`).
- `--stop-cloud-end 0.45`: a stop ends when voxel growth over a 2 s window stays below 0.45 × the stop's peak for 2 s. This reads `voxels()` (`main.cpp:2440-2454`; `kCloudWin` `:1962`).
- Stuck escape and the impeded look use `view()`, `target_tall()` and `target_small()` (`main.cpp:2466-2492`).
- Logging only: `cld`, `thg`, `cloudv`, `things`, `mv*` records.

#### [CloudMap 4.1] `reality.proprio.cloud`: the 36-dim floor-break profile (`profile()`, `CloudMap.cpp:1207-1245`)

**Which voxels count.** Only voxels with mean height in [0.02, 0.20) m, horizontal range in (0, 2.5] m from the anchor, and azimuth |az| ≤ 40° in the anchor's de-rotated frame (`:1220-1225`).

**Sectors.** 8 sectors of 10° each. Sector = `floor((az + 40) / 80 · 8)`. Sector 0 is the **right** edge (az = −40°, with y to the left) (`:1226`).

| Index | Value | Empty sector |
|---|---|---|
| 0-7 | nearest break voxel's range / max_range (2.5) | 1 |
| 8-15 | that nearest voxel's mean height / break_hi (0.20), clamped to [0,1] | 0 |
| 16-23 | (highest − lowest mean height in the sector) / break_hi, clamped | 0 |
| 24-31 | voxel count in the sector / 60, clamped | 0 |
| 32 | total break voxels / 400, clamped | |
| 33 | sectors hit / 8 | |
| 34 | mean break-voxel height / break_hi, clamped | |
| 35 | total voxels in the cloud (all heights, including floor) / 4000, clamped | |

Rows 0-31 are at `:1232-1239`; rows 32-35 at `:1240-1243`. The whole vector is zero when the cloud is empty (`:1209`).

#### [CloudMap 4.2] `reality.proprio.thing`: the descriptor (`thing_descriptor`, `:954-968`)

There is no bearing in it: it describes the thing, not the pose. Each element is clamped to [0,1].

| Index | Value |
|---|---|
| 0 | top / break_hi |
| 1 | ext / small_ext |
| 2 | aspect = ext_min / ext |
| 3 | ncols / 25 |
| 4 | (hits / ncols) / 20, i.e. hits per column |
| 5 | chain / 5 |
| 6 | rng / max_range: the cluster's range from the **anchor**, not the body |
| 7 | lo / break_hi, the lowest height |

`things_shape` is false, so this is the 8-dim form. `things_age_dim` is false, so there is no 9th dim (`:961-966`).

#### [CloudMap 4.3] `percept.thing_bearing` (`:978-990`, `bearing_of` `:1009-1019`)

The layout is `[vx = +right, vy = +forward, proximity, walking]`.
- (vx, vy) is the **unit** direction to the attended thing in the body frame: vx = −by/rng and vy = bx/rng, from `body_rel` (cloud frame → body, with the walking displacement removed and turned by −(yaw − anchor_yaw)).
- proximity = clamp(1 − rng_body / 2.5, 0, 1).
- `walking` = 1 if the cloud is a walking cloud (`:988`).
- All bearing values are 0 when nothing is attended.

This is VisualBearing's 3-value shape plus a 4th value. The header comment still says 3 values (`hpp:182-183`).

#### [CloudMap 4.4] `percept.mover_bearing` (`:1150-1166`)

The layout is `[vx = +right, vy = +forward, proximity, age, oldest, recompute_tick]`.
- `age` is the mover's age_w and `oldest` is the oldest cluster's age_w. Both are in **ticks** (`:1162`), although the header names them `age_s` / `oldest_s` (`hpp:191, 332`).
- `recompute_tick` is the tick of the last recompute (`:1163`). Consumers must count sightings by this value, because the bearing is re-aimed every tick (`:1155-1158`).
- proximity uses the same reach as the thing bearing (2.5 m, `:1014`).
- All zeros when there is no mover.

---

### [CloudMap 5] LEARNING

**None.** CloudMap has no weights, no error signal and no rate. It accumulates voxels, applies fixed geometric rules and keeps an LRU cache of filed stop clouds. The cache is used only for `revisit_change`.

- It does not override `snapshot_state` / `restore_state`; the base defaults are at `cpp_core/src/ogma/Module.cpp:13-24`.
- In LA1 only `motor_epm_intent` is restored from the checkpoint (`main.cpp:1500, 1713`), so the cloud and its cache start empty every run.
- It has no `on_reset` override (header `hpp:127-138`), so it is unaffected by the host's `brain.on_reset()` after a rescue.
- The only adaptive quantity is the mover age threshold, 0.06 × the oldest cluster's age. It is relative to the cloud's own watching time and is not learned.
- The vocabularies over CloudMap's outputs are learned downstream by EPMs: `object_epm`, `thing_epm`, `thing_kind_epm` and `thing_context_epm`.

---

### [CloudMap 6] DIAG FIELDS (`diag_lite`, `CloudMap.cpp:1247-1259`)

| Field | Meaning |
|---|---|
| `open` | a cloud is open; always true after the first tick with walk_cloud on |
| `voxels` | voxels in the open cloud |
| `break` | voxels whose **first** point fell in [break_lo, break_hi) (`:510-513`) |
| `newfrac` | `new_fraction` (3.4) |
| `revisit` | `revisit_change_`; effectively always -1 in LA1 (§7, item 2) |
| `cached` | places in the cache (≤ 8) |
| `filed` | clouds filed so far, stop and walking |
| `place` | `last_key_`: the modal map winner of the **last filed** cloud, walking ones included (`:604`); -1 if none |
| `overlap` | voxels compared at the last revisit judgement. It is reset to 0 at each file (`:624`) but not on open |
| `revisit_dist` | dead-reckoned distance between the two anchors in the last revisit (m); -1 if none |
| `things` | number of clusters at the last recompute |
| `walking` | the open cloud is a walking cloud |
| `small` | number of clusters with `small` true |
| `attended` | the attended thing's **anchor-frame** range `rng` (m); -1 if none |
| `mover` | the mover cluster's anchor-frame range; -1 if none |
| `mover_cands` | cumulative candidate-cluster count (3.6) |
| `vacated` | cumulative vacated-voxel marks; 0 here because the mechanism is off |

`diag_snapshot` (`:1261-1305`) adds `voxel_m`, `anchor_yaw`, `points`, the full voxel list, the cache list, `things_rows`, `attended_i`, `mover_row`, `cfg`, `free_cols` and `body`.

**End-of-run diag (probe log, observed).** `voxels` 5040, `break` 669, `walking` true, `cached` 4, `filed` 15, `place` 9, `things` 6, `small` 0, `mover_cands` 271.
- The stderr summary reports "15 clouds filed, mean 3402 voxels; 4 cached by place" (`la_probe.err:105, 117`).
- Of the 15 filed clouds, 5 were stop clouds (places 1, 3, 5, 5, 9) and 10 were walking clouds (`cloudv` records).

---

## 2.6 `object_epm` — EPM over the cloud's floor-break profile


**Config:** `a1v2_t11_not_target.json:274-288`
- group cognitive, name object, encoder **jl_state**;
- input `reality.proprio.cloud`, `proprio_state_dims` 36, `projection_dim` 104 (explicit);
- `baking_threshold` 20, `min_insertion_error` 0.06, `max_nodes` 128, `master_seed` 4646 (no effect; the JL seed is the hash of
  "object");
- `process_every_n_ticks` 1 (default); `health_death_spares_baked` false (default).

**Input.** The producer is CloudMap `cloud` (`output_topic` `reality.proprio.cloud`, `a1v2_t11_not_target.json:232`),
which publishes `profile()` **every tick** (`CloudMap.cpp:752-760`). The profile has `kProfile = 8·4 + 4 = 36` values
(`CloudMap.hpp:68-69`; `CloudMap.cpp:1207-1243`). It covers voxels whose mean point height h satisfies
`break_lo ≤ h < break_hi` (0.02-0.2 m), within `max_range` 2.5 m and ±`half_fov` 40°, split into 8 sectors
(config `a1v2_t11_not_target.json:235-238`):

| slots | per sector s = 0..7 |
|---|---|
| 0-7 | nearest range / max_range (1.0 if the sector is empty) |
| 8-15 | the nearest voxel's height / break_hi (0 if empty) |
| 16-23 | (zmax − zmin) / break_hi, the vertical extent (0 if empty) |
| 24-31 | voxel count / 60, clipped to 1 (mass) |

| slot | global value |
|---|---|
| 32 | total break voxels / 400 |
| 33 | share of sectors hit |
| 34 | mean break height / break_hi |
| 35 | total voxels / 4000 |

- **Range and conditioning.** All values lie in [0,1] by construction. There is no mean removal. jl_state L2-normalises
  the vector, so only its **direction** matters (`encoder_jl.cpp:213-215`).
- **⚠ Empty cloud.** An empty voxel set returns all zeros (`CloudMap.cpp:1209`), which encodes to the zero latent. A
  quiet or empty cloud is therefore one fixed point in latent space.
- When the voxels come from: a stop's cloud, or the walking cloud (`walk_cloud` true, `a1v2_t11_not_target.json:252`). The CloudMap section covers
  which applies when.

**Computes.** 36-d → L2-normalise → 36×104 Gaussian projection → L2-normalise → GNG every tick. It is never frozen (A.8).
- **Probe:** 18 nodes, all 18 baked. ema_tle 0.080, `tle_norm` 1.0 (`la_probe.err:118`).
- The config comment cites an earlier measurement of 16 nodes all baked (`a1v2_t11_not_target.json:287`).

**Output.** `reality.cognitive.object`. **No consumer.** No module param in the three LA1 graphs names it, the only
pattern subscriber reads `reality.loop.` (`voter_loops`, `a1v2_t11_not_target.json:463`), and nothing in `mj_host/src`
reads it (grep for `cognitive.object` / `object_epm` is empty). It is computed, logged by the inspector diag, and never used.

## 2.7 `thing_epm` — EPM, the fine vocabulary of attended small things


**Config:** `a1v2_t11_not_target.json:291-325`
- group cognitive, name thing, rbf;
- input `reality.proprio.thing`, 8 dims, `dim_min` all 0 and `dim_max` all 1 (an identity map into [0,1]);
- `max_nodes` 64, `master_seed` 4747, `baking_threshold` 20, `min_insertion_error` 0.06, `health_death_spares_baked` true.

**Input.** The producer is CloudMap. It publishes on `things_topic` **only on ticks where a thing is attended**
(`attended_ ≥ 0`, `CloudMap.cpp:725-732`). The values are `thing_descriptor()`, the FULL form, because `things_shape` and
`things_age_dim` default to false (`CloudMap.hpp:268`, `328`; `CloudMap.cpp:415`, `447`). Each value is clipped to [0,1]
(`CloudMap.cpp:954-968`):

| slot | content |
|---|---|
| 0 | top / break_hi |
| 1 | ext / small_ext |
| 2 | ext_min / ext (aspect) |
| 3 | ncols / 25 |
| 4 | hits per column / 20 |
| 5 | chain / 5 |
| 6 | range / max_range |
| 7 | lowest break height / break_hi |

Field meanings are at `CloudMap.hpp:85-93`.
- **⚠ Repeated inputs.** The things are recomputed only every `things_every` = 4 ticks (`CloudMap.cpp:723`; config `a1v2_t11_not_target.json:250`), but the
  descriptor is published every tick from the cached `things_` vector. So about 3 of every 4 GNG steps see an identical input.
  Visit counts, and therefore baking at 20 visits, accumulate about 4× faster than distinct observations. This is
  consistent with `qe_lag1` 1.0 in the probe (`la_probe.err:119`).
- On ticks without an attended thing, the EPM receives no input, does not step, and publishes `winner_id −1` (A.6).

**Computes.** RBF 8 → 64 (Halton) → GNG on every tick with input. It is never frozen.
- **Probe:** 15 nodes, 14 baked (`la_probe.err:119`). `tepm` was logged on 1499 ticks, with winners −1 and 0..14 (`la_probe.jsonl`).

**Output.** `reality.cognitive.thing`. Its **only consumer is the host's log.** IntentAdapter reads `winner_id`, `tle` and
`node_count` when the token's `tick_id` matches (`IntentAdapter.cpp:231-234`). Those accessors (`IntentAdapter.hpp:335-337`)
are used only to print `tepm` in the JSON log (`main.cpp:3022`). No module subscribes. The config comment's plan
("its TLE at the attended thing is the pull T2 will use", `:324`) is not wired in LA1.

## 2.8 `thing_kind_epm` — EPM, the kind of thing


**Config:** `a1v2_t11_not_target.json:328-362`
- the same input and encoder as `thing_epm` (`reality.proprio.thing`, 8 dims, [0,1] ranges, Halton 64);
- **`max_nodes` 4**, **`min_insertion_error` 0.25** (a squared distance, so about 0.5 in distance);
- `baking_threshold` 20, `master_seed` 4748, `health_death_spares_baked` true.

**Computes.** The same per-tick behaviour as C.3.
- Insertion stops at 4 nodes (`gng.cpp:212`). Once all 4 are baked, the prototypes never move again (`gng.cpp:135`), so
  **the kind partition is fixed by the first things seen in a run** (the EPM is not restored, so this is fresh each run).
- **Probe:** 4/4 baked by 120 s (`la_probe.err:120`). `tkind` winners 0..3 (`la_probe.jsonl`).

**Output.** `reality.cognitive.thing_kind`. Consumers:

| consumer | reads | anchor |
|---|---|---|
| SkillOutcomeLoop `outcome` | `winner_id` only, as the kind key of the outcome table | `thing_topic`, `a1v2_t11_not_target.json:499`; `cpp_core/src/ogma/modules/SkillOutcomeLoop.cpp:162` |
| IntentAdapter (host) | `winner_id`, `tle`, `node_count`, for the `tkind` log only | `IntentAdapter.cpp:235-238`; `main.cpp:3023` |

In SkillOutcomeLoop the key is `kind × context_n + ctx` (`SkillOutcomeLoop.cpp:167`). `winner_id −1` (no attended thing)
means no node.

## 2.9 `thing_context_epm` — EPM, the attended thing's surroundings


**Config:** `a1v2_t11_not_target.json:365-387`
- rbf, input `reality.proprio.thing_context`, 2 dims, ranges [0,1];
- **`max_nodes` 3**, `min_insertion_error` 0.25, `baking_threshold` 20, `master_seed` 4757, spares baked true.

**Input.** The producer is CloudMap (`context_topic`, `a1v2_t11_not_target.json:264`). It publishes in the same block, and
under the same `attended_ ≥ 0` condition, as the thing descriptor (`CloudMap.cpp:733-741`). The values are:
- `[on_line, near_tall]`, each clamped to [0,1];
- `on_line` = the share of the cluster's columns lying on the closed tall-structure footprint;
- `near_tall` = `exp(−d / max(voxel, line_close_k·range))` to the nearest tall column, or 0 if there are none;
- both are computed only for clusters judged small (`CloudMap.cpp:923-947`).

**Computes.** RBF 2 → 48 on a 7×7 grid (A.2) → GNG. Full at 3 nodes.
- **Probe:** 3/3 baked, ema_tle 0.013 (`la_probe.err:121`).

**Output.** `reality.cognitive.thing_context`. Consumer: **SkillOutcomeLoop**, which reads `winner_id` only
(`context_topic` `a1v2_t11_not_target.json:518`, `context_n` 3; `SkillOutcomeLoop.cpp:164-167`).
- The winner is **clamped** to [0, 2] and used directly as a table index.
- Node IDs are GNG IDs, not indices, and are never reused. This is correct only while the context EPM's nodes keep IDs 0-2.
  A node removed before baking (isolated removal, A.3) followed by a re-insertion would give an ID ≥ 3, which aliases onto context 2.

## 2.10 `seek` — BearingSeekLoop


**Role:** go to a small thing seen only at stops. Fixes the attended thing's position in the odometry frame by dead reckoning, then homes to it while walking (re-aiming as the body turns), drops it on arrival / forgetting; and chases a moving thing ("mover") by predicting its position.

### Inputs
| Topic | Producer | Layout | Anchor |
|---|---|---|---|
| `percept.thing_bearing` | `cloud` (CloudMap) | `[vx=+right, vy=+forward, proximity, walking]`; proximity = 1 − range/2.5; `walking` = 1 if seen from a walking cloud (CloudMap.cpp:977-989; config `walk_cloud true`) | BearingSeekLoop.cpp:286-293 |
| `reality.proprio.odom` | host IntentAdapter: `{2·pose[0], 2·pose[1], heading_}` where pose[0..1] = odometry x/2, y/2 (main.cpp:2821) → **[x m, y m, unwrapped yaw rad]** | IntentAdapter.cpp:208-210 | BearingSeekLoop.cpp:281-285 |
| `percept.mover_bearing` | `cloud` | `[vx, vy, proximity, age, oldest, recompute_tick]`; a sighting is "fresh" only when `values[5]` changes (CloudMap recomputes clusters every `things_every 4` ticks) | CloudMap.cpp:1150-1165; BearingSeekLoop.cpp:464-473 |
| `reality.cognitive.outcome_need` (renew_topic) | `outcome` | `[need, x, y]` | :326-336 |
| `reality.cognitive.outcome_pull` (pull_topic) | `outcome` | `[pull]` | :372-374 |
| `percept.target_tall` (yield_topic) | `cloud` | `[tall voxel count around seek's held target, range]`; computed only while the cloud is open, from `seek_bearing`/`seek_range` of the previous tick | CloudMap.cpp:1107-1147; BearingSeekLoop.cpp:455-461 |

### Key parameters (config → defaults)
`proximity_range 2.5`, `min_conf 0.02`, `arrive_m 0.15` (default 0.25), `forget_ticks 3000`, `floor 0.05`, `renew_min 0.25`, `renew_range 2.0`, `chase_min_v 0.1`, `chase_v_max 0.6` (default 1.0), `chase_stop_v 0.05`, `chase_memory_ticks 250`, `chase_memory_holds true`, `chase_yield_tall 1`, `progress_walk_m 0.5`, `progress_m 0.05`, `chase_permanence_ticks 500`, `publish_chase_flag true`.
Defaults relied on (BearingSeekLoop.hpp): `walk_refix_m 0`, `walk_take_range 0` (:121, 126), `chase_gate_m 0.35`, `chase_lead_s 0.3` (:154), `chase_confirm 2`, `chase_confirm_ticks 25`, `chase_forget_ticks 50` (:261), `chase_v_window_s 0.8` (:231), `chase_pull_decay 1.0` (:217, so `pull_` stays 1), `static_yield_tall 0` (static yield off, :191), `chase_yield_look false` (:189), `lead_options` empty (learned lead off, fixed 0.3 s lead).

### What it computes each tick (BearingSeekLoop::tick, :280-450)
1. **Pose & walked distance** from odom (:281-285).
2. **Sighting:** `seen = prox > 0.02 && |v| > 0` (:293). With `walk_refix_m 0` and `walk_take_range 0`, a bearing flagged `walking` is discarded (`seen = false`) (:296-311). → **Static targets are fixed only from stop (non-walking) clouds.**
3. **Renewal** (the "linger"): with no target held and nothing seen, if `outcome_need[0] > 0.25` and its position is between 1.5·arrive_m = 0.225 m and 2.0 m away, the target is re-armed at that position with confidence = need (:326-336). With three intents, need ∈ {0, ⅓, ⅔, 1}, so any open intent (≥ ⅓) renews.
4. **Chase** (`chase_tick`, :452-559), see below; then pull recovery (no-op with decay 1), memory/yield expiry (:339-344).
5. **Target update:**
   - *Chasing or coasting:* target = candidate position + velocity·(time since last sighting + 0.3 s lead); need = `pull_` (= 1), multiplied while coasting by `1 − (ticks since coast start)/500` (linear decay over 10 s); range and bearing recomputed from pose. **No arrival check** (a mover is never "reached") (:346-357).
   - *Else if seen (static fix)* — and not while a lost mover's memory is live (`chase_memory_holds`, :358): range = (1 − prox)·2.5; position = pose + rotation(yaw)·(range along the bearing) (:362-367); confidence = `outcome_pull` (falls back to 1 only if no pull token) (:371-374); output bearing = the live bearing normalised (:376).
   - *Else if a target is held (homing):* range = distance pose→target; if range < 0.15 m → **arrival**: drop, `arrivals++` (:383-385); else confidence ×(1 − 1/3000) per tick, drop below 0.05 (`forgets++`), else re-aim `cx = −by/r, cy = bx/r` in the body frame (:377-390).
6. **Progress forget:** with a static target held, every 0.5 m walked the best range must have improved by ≥ 0.05 m, else the target is dropped (`progress_forgets++`); the place is not refused (renewal may re-arm it) (:401-414).
7. Static yield (:415-431) is inactive (`static_yield_tall 0`).
8. `value = have_target ? conf : 0` (:432); publish.

**The chase** (`chase_tick`):
- *Yield:* while chasing/coasting, if `target_tall[0] ≥ 1` the chase is dropped and the place remembered as not-a-mover for 250 ticks (`yield_to_structure`, :455-461, 589-606).
- *Sighting fix* in odometry frame (:474-478). Sightings within 0.35 m of a live yielded place are dropped (:479-480).
- *Candidate confirmation:* predicted position = candidate + v·dt; `ok` if miss ≤ 0.35 m and windowed speed (over the last 0.8 s of sightings, needs ≥ 0.2 s) ≤ 0.6 m/s (:481-498). A confirmation updates velocity/position and **resumes a coasting chase** (:499-506); a failed one replaces an unchased candidate (:507-512).
- *New candidate;* if a lost mover's memory (≤ 250 ticks = 5 s old) predicts this sighting within 0.35 m, the chase is re-acquired at once (:514-526).
- *Promotion to chase:* ≥ 2 sightings over ≥ 25 ticks and velocity ≥ 0.1 m/s and displacement/time watched ≥ 0.1 m/s; else counted `cand_still` (:528-536).
- *End of sightings:* no confirmation for 50 ticks (1 s): if speed < 0.05 m/s → becomes an ordinary static target at its last position (conf 1) (:543-548); else, with `chase_permanence_ticks 500`, **coasting** (pursuit of the predicted position for 10 s with linearly decaying need) (:549-551). When coasting runs out → `lose()`: drop, `lost_now_`, memory of extrapolated position/velocity for 250 ticks (:538-541, 575-587). (`--stop-on-lost` is not in the LA1 args, so no look is started; the config's `_comment_pursuit` says so.)

### Outputs
| Topic | Layout | Consumers |
|---|---|---|
| `percept.seek_bearing` | `[cx (+right), cy (+forward), value, chase_flag]` — slot 2 is the **need**, not 0 as the header/schema say; slot 3 = 1 while chasing or coasting (:434-439) | IntentAdapter: heading reference when `arbiter.gain.vision > 0.5` (steer code 3, IntentAdapter.cpp:250-256, 284-307), seek gate on the ToF slot of the target's sector (`--seek-gate`, IntentAdapter.cpp:145-151); CloudMap `target_topic` (uses slot 2 > 0 as "target valid", slot 3 to exclude a chased target from `mover_not_target_m`, CloudMap.cpp:1021-1030, 1069, 1107-1112) |
| `reality.cognitive.seek_value` | `[value]` = confidence/need ∈ [0,1] (:440-443) | arbiter (`hunger_topic` and `vision_value_topic`: G_seek = 1·seek_value·trust_seek, G_play = (1−seek_value)·trust_play, EFEArbiter.cpp:312-313); `outcome` (arrival); IntentAdapter (`seek_arrived` = value falls to 0 with range < 0.3 → `--stop-on-arrive`, IntentAdapter.cpp:261-262, 276; impeded detection main.cpp:2176) |
| `reality.cognitive.seek_range` | `[range_left m]` (last value persists when no target) (:444-449) | `comp_seek` (LoopCompetence `objective_field "value"` → `values[0]`, sign −1, gain topic `arbiter.gain.vision`; LoopCompetence.cpp:135-136) → `reality.loop.seek`; `outcome`; CloudMap `target_range_topic`; IntentAdapter (range sense slot IntentAdapter.cpp:162-165; impeded look main.cpp:2178; arrival gaze centre main.cpp:2227) |

Host coupling via direct module access (`find_seek`, IntentAdapter.cpp:486-524): `--chase-vx 0.35` raises the forward command to ≥ 0.35 m/s while chasing/coasting (main.cpp:2097-2098); `--stop-on-chase` ends a stop when a chase is confirmed (main.cpp:2193-2197); the impeded look's wall verdict calls `forget_target()` (main.cpp:2483; BearingSeekLoop.hpp:203); `chase_gaze_ego()` drives the head toward the chased target (main.cpp:2648).

### Learning
None in this config: the only learned table (the chase lead per situation, :608-671) is off (`lead_options` empty). Everything else is state (held target, candidate, memory). Observed (probe log `la_probe.err:98, 122`): in 120 s, 3 chases, 93 candidates rejected as still, 2 chases yielded near tall structure; `arrivals 4, renewals 4, forgets 0`.

---

## 2.11–2.14 The arbitration layer: shared facts


Two navigation loops compete for the heading reference: **seek** (BearingSeekLoop, homes on a
remembered thing) and **play** (PlayLoop, climbs toward the most novel map node). Each loop gets
a **LoopCompetence** grader that asks, every 30 consecutive ticks the loop has been in control,
"did my objective move the way I claim while I drove?" (seek: did the range to the target fall;
play: did the map EPM's TLE rise), and keeps a Beta posterior over that success rate, publishing
its optimistic upper bound (mean + 1 sd) as a 1-D RealityToken with `tle = 1 − competence`. A
level-1 **LateralVoter** turns those two errors into normalised trust weights `∝ 1/(err + 0.05)`.
The **EFEArbiter** in `precision` mode scores `G_seek = seek_need × trust_seek` and
`G_play = (1 − seek_need) × trust_play`, keeps the incumbent unless a challenger leads by an
adaptive margin (1 × running std of the top-two gap), and publishes hard one-hot 0/1 gains. The
host reads the gains and takes the winning loop's bearing as the heading reference. Nothing here
is restored from the checkpoint; all of it starts fresh each run and adapts throughout.

### Tick-order and timing facts (apply to all four modules)

- Modules tick in config order (§1.4): `seek` → `comp_seek` → `comp_play` → `voter_loops` → `arbiter` → `outcome`.
- Bus delivery of `Direct` subscriptions is synchronous at publish time (`cpp_core/src/ogma/InProcessBus.cpp:25-34`); a subscription pattern ending in `.` is a prefix match (`InProcessBus.cpp:14-17`, `:42`).
- Hence, within one tick: seek's `seek_value`/`seek_range` → comp_seek (same tick); comps' tokens → voter (same tick); voter's `consensus.1` → arbiter (same tick). But the **arbiter's gain reaches the LoopCompetence modules one tick late** (comp_* tick before arbiter, so they read the previous tick's gain).
- The intent brain ticks every host tick, including during stops and rescues (`mj_host/src/main.cpp:2040`, the call at `main.cpp:2080` → `IntentAdapter.cpp:223`). Brain rate 50 Hz (`mj_host/src/DuckBody.hpp:45`), so 30 ticks = 0.6 s.
- Restore: `--load-brain` restores only `motor_epm_intent` by default (`main.cpp:1500`, filter `main.cpp:1717-1722`); comp_seek, comp_play, voter_loops and arbiter all start from their constructor state in every LA1 run.
- Host freezing (`IntentAdapter::set_learning`, `IntentAdapter.cpp:408-436`) touches only `MotorEPM`/`MotorEPMv2` modules (`IntentAdapter.cpp:418`); map freezing touches only the map EPM (`IntentAdapter.cpp:438-455`). **Nothing in this layer is ever frozen.**
- Keys beginning with `_` (the `_comment` entries) are skipped at load (`cpp_core/src/ogma/OgmaInstance.cpp:66`); unknown keys print a warning (`OgmaInstance.cpp:69-72`). The probe log (`la_probe.err`) contains no such warning, so every non-comment key below is a schema key.

---

## 2.11 `comp_seek` — LoopCompetence


**Role:** grades the seek loop by whether the distance to its target actually shrinks while seek holds the heading.

### Inputs
| Topic | Producer | Payload / layout | How used |
|---|---|---|---|
| `reality.cognitive.seek_range` (config :427) | `seek` (BearingSeekLoop), published every tick (`BearingSeekLoop.cpp:444-448`) | ProprioToken, 1 value = `range_left_` (metres to the held target, dead-reckoned) | `objective_field "value"`, `objective_index 0` (default) → `obj_ = values[0]` (`LoopCompetence.cpp:135-136`) |
| `arbiter.gain.vision` (config :428) | `arbiter` | ProprioToken, 1 value ∈ {0,1} | `gain_ = values[0]` (`LoopCompetence.cpp:138-141`); the loop "drives" when `gain_ > 0.5` (`:145`) |

### Key parameters (config :426-437; schema/defaults `LoopCompetence.cpp:55-78`, members `LoopCompetence.hpp:64-84`)
| Param | Value | Effect |
|---|---|---|
| `sign` | −1.0 | prediction is "objective FALLS" (range shrinks) |
| `horizon_ticks` | 30 | one check per 30 consecutive driving ticks |
| `estimator` | `"beta"` | published competence comes from the Beta posterior, not the EMA |
| `optimism` | 1.0 | published = mean + 1·sd |
| `forget` | 0.001 | per-tick relaxation of the pseudo-counts toward 1 while not driving |
| `alpha` | 0.05 | EMA rate of `c_` — **has no effect on the published value under `estimator beta`** (see below) |
| `modality_group` / `modality_name` | `loop` / `seek` | output topic `reality.loop.seek` (`LoopCompetence.cpp:110`) |
| defaults relied on | `prior_ = 0.5` (hard-coded member, not a param, `LoopCompetence.hpp:75`); `objective_index 0` | |

### What it computes each tick (`LoopCompetence.cpp:144-182`)
1. `drive_now = gain_ > 0.5 && have_obj_` (`:145`).
2. If driving: on the first driving tick of a run, `run_ticks_ = 0` and `o_start_ = obj_` (`:147`); then `++run_ticks_`. When `run_ticks_ >= 30`, one check: `improved = sign·(obj_ − o_start_) > 0`, i.e. for seek **`range_now < range_30_ticks_ago`** (strictly) (`:151`). Then `checks_++`, `improvements_ += improved`, EMA `c_ += 0.05·(improved − c_)` (`:153`), and the Beta counts: `a_ += 1` on success, else `b_ += 1` (`:154`); the window restarts from the current objective (`:155`). The window is reset whenever driving is interrupted (a partial window is discarded — no check).
3. If not driving: `c_ += 0.001·(0.5 − c_)`, `a_ += 0.001·(1 − a_)`, `b_ += 0.001·(1 − b_)` (`:158-160`) — the counts are forgotten toward the flat Beta(1,1) prior with time constant 1/0.001 = 1000 ticks = 20 s.
4. Published competence (beta): `n = a+b`, `mean = a/n`, `sd = sqrt(a·b / (n²·(n+1)))`, `pub_ = clamp(mean + 1.0·sd, 0, 1)` (`:163-167`). At the fresh start (a = b = 1): mean 0.5, sd = sqrt(1/12) = 0.289, **pub = 0.789**.
5. Success definition: a strict decrease of range over the 30-tick window. No threshold, no scale (`:151`).

### Output
`reality.loop.seek`, a RealityToken published **every tick** (`LoopCompetence.cpp:172-181`):
| Field | Value |
|---|---|
| `winner_id` | 0 |
| `latent`, `winner_prototype` | `[pub_]` (1-D) |
| `quant_error` = `expected_error` = `tle` | `err = clamp(1 − pub_, 0, 1)` |
| `transition_surp` | 0 |
| `node_count` / `baked_count` | 1 / 3 ("informative by construction") |
| `producer_id` | module id `comp_seek` |

Consumer: `voter_loops` (prefix `reality.loop.`). Diagnostics (`diag_snapshot`, `:196-199`): `competence` (= EMA `c_`), `published`, `driving`, `checks`, `improvements`, `objective`, `gain`, `beta_a`, `beta_b`.

### Learning
The Beta pseudo-counts (a, b) are the learned state: +1 per 30-tick driving window, forgetting at 0.001/tick toward 1 while idle. On all run, never frozen, starts from a = b = 1 every run (not restored).

---

## 2.12 `comp_play` — LoopCompetence


**Role:** grades the play loop by whether the map's surprise (novelty) actually rises while play holds the heading.

### Inputs
| Topic | Producer | Payload | How used |
|---|---|---|---|
| `reality.proprio.place` (config :445) | `map_epm` (EPM, `modality_group proprio`, `modality_name place`) | RealityToken | `objective_field "tle"` (config :449) → `obj_ = rt->tle` (`LoopCompetence.cpp:131-133`); subscribed as RealityToken type (`:43`) |
| `arbiter.gain.play` (config :446) | `arbiter` | ProprioToken 1 value ∈ {0,1} | drives when > 0.5 |

### Key parameters (config :444-455)
`sign +1.0` (objective RISES), `horizon_ticks 30`, `alpha 0.05` (inert under beta), `forget 0.001`, `estimator beta`, `optimism 1.0`, group/name `loop`/`play` → output `reality.loop.play`. No `_comment`.

### What it computes / output / learning
Identical algorithm to §1 with `sign = +1`: a check succeeds when **map-EPM TLE now > TLE 30 ticks ago** (`LoopCompetence.cpp:151`). Output `reality.loop.play`, same token layout as §1. Same learning and the same always-on, fresh-each-run status.

---

## 2.13 `voter_loops` — LateralVoter (level 1)


**Role:** turns the two loops' competence errors into precision (trust) weights that sum to 1.

### Inputs
| Topic | Producer | Use |
|---|---|---|
| prefix `reality.loop.` (config :463) | comp_seek → `reality.loop.seek`, comp_play → `reality.loop.play` (the only `reality.loop.*` publishers in this config — `grep reality.loop` finds only lines 463, 478, 479 of the config besides the comps' group/name) | buffered per topic in `pending_` (`LateralVoter.cpp:206-225`); a token is "active" if `winner_id >= 0` and latent non-empty (`:281-284`) — both comps always qualify |
| `neuro.state` (`Topics.hpp:682`, subscribed at `LateralVoter.cpp:165-166`) | none in this graph | `dopamine_` stays 0 (probe: `"dopamine":0.0`, `la_probe.err:123`) |

### Key parameters (config :461-470; schema `LateralVoter.cpp:67-108`)
| Param | Value | Effect |
|---|---|---|
| `level` | 1 | output topic `consensus.1` (`LateralVoter.cpp:160`) |
| `input_pattern` | `reality.loop.` | prefix subscription |
| `trust_source` | `expected` | error = `|expected_error|` (`:388`) = 1 − pub for a LoopCompetence token |
| `trust_power` | 1.0 | `precision = 1/(err + ε)` (`:394`) |
| `group_balance` | false | plain L1 normalisation (`:472-478`) |
| `informativeness_gain` | 0.0 | the informativeness path is off (`:399`), legacy branch `:414-417` (still honouring `trust_source`) |
| `activity_gain` | 0.0 | activity term off (`:428`) |
| `master_seed` | 3 | read (`:158`) and stored; **not used in any computation** in `LateralVoter.cpp` |
| defaults relied on | `trust_epsilon 0.05` (`:72`, member `LateralVoter.hpp:110`), `softmax_temperature 1.0` (`:74`), `surprise_gain 0` (`:83`), `association_enabled false` (`:76`), `priority_group "proprio"` (`:75`) | |

### What it computes each tick (`LateralVoter.cpp:277-643`)
1. Collect active inputs; if none, republish the previous token (`:286-301`) — does not occur here since both comps publish every tick.
2. Raw trust per loop: `raw_i = 1 / (|expected_error_i| + 0.05)` = `1 / (1 − pub_i + 0.05)` (`:387-396`, `:414-417`). Range: 0.95 (pub 0) … 20 (pub 1).
3. Normalise: `trust_i = raw_i / Σ raw` (`:473-477`).
4. Temperature: `T = max(0.01, 1.0·(1 + 0.5·dopamine))` = 1 → no reshaping (`:483-484`).
5. Fusion: `fused_embedding = Σ trust_i · [pub_i]` (1-D), `fused_tle = Σ trust_i · tle_i` (`:497-508`).
6. Active modality: both inputs are group `loop`; the winning group is `loop`, and within it the input with the lowest `quant_error` (= the more competent loop) is the active member (`:510-544`), giving `active_modality = "loop.seek"` or `"loop.play"`.
7. At fresh start both pubs are 0.789, so trust = 0.5 / 0.5.

### Output: `consensus.1` (ConsensusToken, every tick, `:546-621`)
| Field | Content |
|---|---|
| `trust_weights` | `{ "reality.loop.seek": t_s, "reality.loop.play": t_p }`, keyed by **full input topic**, t_s + t_p = 1 (`:547-549`, `:562`) |
| `fused_embedding` | 1-D: trust-weighted mean published competence |
| `fused_tle` | trust-weighted mean error |
| `level` | 1 |
| `active_modality`, `active_winner_id` | `"loop.<more competent>"`, 0 |
| `winner_ids_by_modality` | both 0 (`:599-604`) |
| `surprise_ema` | empty (surprise_gain 0) |

Consumer: `arbiter` only (no other `consensus.1` reader in `mj_host/src` or `cpp_core/src` except EFEArbiter's subscription). `diag_lite` (`:656-675`): `dopamine`, `fused_tle`, `has_token`, `trust` map, `surprise` map.

**Observed (probe log, end of the 120 s run, `la_probe.err:123`):** `trust {reality.loop.play: 0.272, reality.loop.seek: 0.728}`, `fused_tle 0.204`, `dopamine 0.0`. By the formula above this means `(1 − pub_play + 0.05) ≈ 2.68 × (1 − pub_seek + 0.05)`, i.e. the seek loop's published competence is clearly the higher at run end (individual pubs not in the summary).

### Learning
None. The voter is stateless with these settings apart from `prev_token_` (and the unused surprise/assoc structures). Trust is recomputed every tick from the comps' current errors.

---

## 2.14 `arbiter` — EFEArbiter, scoring_mode "precision"


**Role:** each tick picks ONE loop to own the heading (seek or play) by need × trust, with an adaptive hysteresis, and publishes one-hot gains.

### Inputs (subscriptions `EFEArbiter.cpp:206-232`)
| Topic | Param | Producer | Used in precision scoring? |
|---|---|---|---|
| `reality.cognitive.seek_value` | `hunger_topic` (config :480) | `seek`, every tick (`BearingSeekLoop.cpp:440-443`): `value_ = have_target ? conf_ : 0` (`:433`); `conf_` ∈ [0,1] at every assignment (`:332, 352, 371, 374`; `pull_ ≤ 1` at `:339`) | **YES** — `hunger_` = "seek need" (`EFEArbiter.cpp:241-245`) |
| `consensus.1` | `trust_consensus_topic` (:477) | `voter_loops` | **YES** — `trust_ = ct->trust_weights` (`:276-279`) |
| `reality.cognitive.seek_value` | `vision_value_topic` (:481) | `seek` | **NO** — stored in `vision_value_` (`:281-285`) but read only in the `efe` branch (`:345`); in precision mode it only switches the vision channel on (`vision_active_`, `:238`) |
| `reality.cognitive.play_value` | `play_value_topic` (:483) | `play` (PlayLoop, `PlayLoop.cpp:477-485`) | **NO** — `play_value_` is read only in the `efe` branch (`:396`); in precision mode it only switches the play channel on (`play_active_`, `:236`) |
| `reality.proprio.scent_max`, `reality.cognitive.plan_value`, `percept.klino_confidence`, `reality.cognitive.plan_novelty`, `reality.cognitive.plan_precision` | defaults (`EFEArbiter.hpp:187-191`) | **no publisher in this graph** (Cell-era topics) | subscribed (`:209-223`) but irrelevant; `raw_klino_ = hunger·scent = 0`, `raw_planner_ = 0` (`:290-291`) |

### Key parameters (config :475-490; schema `EFEArbiter.cpp:65-102`)
| Param | Value | Effect |
|---|---|---|
| `scoring_mode` | `precision` | branch `EFEArbiter.cpp:297-316` |
| `trust_key_vision` | `reality.loop.seek` | the vision channel = the seek loop |
| `trust_key_play` | `reality.loop.play` | |
| `trust_key_klino` / `trust_key_planner` | defaults `reality.loop.klino` / `reality.loop.planner` (`EFEArbiter.hpp:242-243`) | **absent from the trust map** → trust 0 |
| `vision_weight` / `play_weight` | 1.0 / 1.0 | multiply G; >0 activates the channels (`:236-238`) |
| `precision_sign` | 1.0 | score by trust (−1 would be the wrong-sign control, `:307`) |
| gain topics | `arbiter.gain.vision`, `.play`, `.klino`, `.planner` | outputs |
| `master_seed` | 11 | used only by `force_policy "shuffle"` (`:523-529`) and `rng_.seed` (`:204`; `rng_` is not otherwise used) — **inert here** |
| defaults relied on | `hysteresis_k 1.0` (`EFEArbiter.hpp:255`), `gap_std_alpha 0.02` (`:256`), `play_hunger_weight false` (`:214`), `force_policy ""` (`:257`) | |

### What it computes each tick (`EFEArbiter.cpp:287-581`)
1. **Trust lookup** (`:304-310`): `T(key) = clamp(trust_weights[key], 0, 1)` or 0 if the key is absent; with `precision_sign ≥ 0` it is used as is. So `trust_vision = t_seek`, `trust_play = t_play`, `trust_klino = trust_planner = 0`.
2. **Scores** (`:311-316`), with `h = seek_value`:
   - `G_klino = h · 0 = 0`, `G_planner = h · 0 = 0`
   - `G_play = play_weight · (1 − h) · trust_play = (1 − h) · t_play` (`:313-314`; `play_pref = 1 − h` since `play_hunger_weight` is false)
   - `G_vision = vision_weight · h · trust_vision = h · t_seek` (`:315`)
   This **verifies the config comment** (config :491): `G_seek = seek need × seek trust`, `G_play = (1 − seek need) × play trust`. Note there is no epistemic term, no play value and no seek value other than h in the precision branch; play's own frontier value does not enter the score.
3. **Adaptive margin** (`:489-511`): vision is active, so the 4-way branch: `gap = top1 − top2` over `{G_klino, G_planner, G_play, G_vision}` (`:490-495`). With the two dead channels at 0 and the live ones ≥ 0, `gap = |G_play − G_vision|`. On the first tick `gap_mean = gap, gap_var = 0` (`:504`); afterwards EMA mean and variance at rate 0.02 (`:506-508`). `margin = 1.0 · sqrt(gap_var)` (`:510-511`).
4. **Selection** (`:530-540`): keep the incumbent `winner_` unless the best challenger's score exceeds the incumbent's by **strictly more than** `margin`. This is a deterministic argmax with hysteresis, not a softmax and not sampling; ties keep the incumbent. Indices: 0 klino, 1 planner, 2 play, 3 vision (`EFEArbiter.hpp:299`).
5. **Gains** (`:563-580`): one-hot hard gains, `1.0` to the winner's topic, `0.0` to every other, all four topics published every tick as 1-value ProprioTokens (`sensor "arbiter_gain"`, `producer_id` = module id).
6. **Horizon:** one step. Every term is an instantaneous scalar of the current tick; nothing is rolled out (matches the recipe's statement "Horizon: one step, greedy", `docs/plans-and-designs/loop_and_arbitration_recipe.md:111-114`). The only temporal smoothing is the margin's EMA.

**Start-up and the dead channels.** `winner_` is initialised to 0 (klino, `EFEArbiter.hpp:299-300`) and EFEArbiter has no snapshot/restore override, so each run starts with klino as incumbent. On tick 0 the margin is 0 (`gap_var = 0`), so any live channel with a positive score takes over immediately: seek publishes `value 0` at start, so `G_play = 1 × 0.5 > 0` and play wins tick 0 (probe tick 0: `steer 1`). After that **klino and planner can never win**: their score is 0 and a challenger must lead the incumbent (score ≥ 0) by more than a margin ≥ 0. Consequently `arbiter.gain.klino` and `arbiter.gain.planner` are 0 for the whole run (probe stderr: "6000 ticks steered by a loop's bearing (0 by avoidance)", `la_probe.err:113`). Seek can only win while `h > 0`, i.e. while the seek loop holds a target.

**Implied switching rule (derived, not a code constant).** Ignoring the margin, seek wins when `h · t_seek > (1 − h) · t_play`, i.e. `h > t_play / (t_play + t_seek) = t_play` (since the trusts sum to 1). With the end-of-run trusts (t_play 0.27) any seek need above ≈ 0.27 (plus the margin) takes the heading.

### Outputs and consumers
| Topic | Layout | Consumers |
|---|---|---|
| `arbiter.gain.vision` | ProprioToken `[0 or 1]` | `comp_seek` (next tick); host `IntentAdapter.cpp:250` → `g_seek` |
| `arbiter.gain.play` | `[0 or 1]` | `comp_play` (next tick); host `IntentAdapter.cpp:250` → `g_play` |
| `arbiter.gain.klino` | `[0 or 1]` (0 after tick 0) | host `IntentAdapter.cpp:250` → `g_avoid` (no avoidance loop exists in this config; `percept.avoid_bearing` has no publisher) |
| `arbiter.gain.planner` | `[0 or 1]` (always 0 here) | no consumer found (`grep arbiter.gain` in `mj_host/src`, `cpp_core/src`) |

**Host use (`IntentAdapter.cpp:244-310`).** After `instance_->tick()` the adapter reads the last value of each gain (`:244-250`); if any gain topic exists, the bearing source is chosen by priority `klino > vision > play` at `> 0.5` (`:254-258`), steer codes 2 / 3 / 1. That loop's bearing `(cx, cy)` becomes the heading reference `heading_ref_ = heading_ − atan2(cx, cy)` (`:301`), subject to a free-space gate on the ToF sectors (`:289-299`; the `--seek-gate` lever exempts the seek target's own sector, `:295`) and an escape hold that overrides all loops (`:278-283`, steer 4). A winner with a zero-length bearing releases the reference (`:308`). The resulting `last_steer()` (0 none, 1 play, 2 avoidance, 3 seek; `IntentAdapter.hpp:121`) is used by the host for further behaviour, e.g. stop-on-arrive (`main.cpp:2176`) and the head's gaze offset while seek steers (`main.cpp:2598`, `:2608`, `:2643`).

**Neither loop is muted.** No module in `cpp_core/src` other than LoopCompetence subscribes to `arbiter.gain.*`; seek and play compute their bearings every tick regardless, and the arbitration acts only through which bearing the host uses and through which loop's competence is checked.

### Diagnostics (`diag_snapshot`, `EFEArbiter.cpp:583-624`)
Relevant in precision mode: `scoring_mode`, `trust_play`, `trust_vision`, `trust_klino`, `trust_planner`, `precision_sign`, `g_epist_play` / `v_play` (= G_play), `g_prag_vision` / `v_vision` (= G_vision), `G_klino`, `G_planner`, `gain_*`, `winner`, `margin`, `hunger` (= seek_value), `play_active`, `vision_active`. `vision_value` and `play_value` are reported but do not enter the precision score. The comments on several fields say "efe mode" (e.g. `:599-600`, `:606`, `:612`); in precision mode the `g_*`/`v_*` fields carry the precision scores above.

### Learning
None in the scoring itself. The only adaptive state is the running mean/variance of the score gap (rate 0.02/tick) that sets the hysteresis margin, plus the incumbent. Always on; fresh each run.

---

### Arbitration: runtime evidence (probe log, observed)


From `la_probe.jsonl` (6000 rows, ticks 0–5999, field `steer` = `last_steer()`; `seek` = `[seek_value, seek_range, gated]`):

| Observation | Value |
|---|---|
| Ticks steered by play (code 1) / seek (code 3) / avoidance (2) / none (0) | 3624 / 2376 / 0 / 0 |
| Heading-owner switches | 12 (13 runs: play 220, seek 801, play 172, seek 526, play 969, seek 1, play 460, seek 416, play 1, seek 510, play 1502, seek 122, play 300 ticks) |
| Ticks with `seek_value > 0` | 2376 — **every one of them** steered by seek, and seek never steered with `seek_value = 0` |
| Seek-steered ticks during a stop phase (`stop` 2 = Brain, 3 = Walker; enum `main.cpp:1881`) | 539 of 2376 |
| End-of-run voter trust (`la_probe.err:123`) | play 0.272, seek 0.728; fused_tle 0.204 |

So in this 120 s probe the arbitration behaved as "seek whenever it holds a target, play otherwise"; the trust weights never made play override a seek need, and the hysteresis produced two one-tick blips (ticks 2688 and 3565).

---

## 2.15 `outcome` — SkillOutcomeLoop


**Role:** learn what each pre-built skill (kick / peck / push) does to a thing of a given kind in a given context. At an arrival it asks for the least-known skill; it measures the outcome as the thing's displacement at its next sighting; it publishes the surprise, a "need" (what is still unknown about the last thing) and a "pull" (expected chance the thing answers).

### Inputs
| Topic | Producer | Use | Anchor |
|---|---|---|---|
| `percept.thing_bearing` | cloud | `[vx, vy, prox]`; **the walking flag (slot 3) is not read** — walking-cloud sightings are fixed too | SkillOutcomeLoop.cpp:158-160, 173 |
| `reality.cognitive.thing_kind` (thing_topic) | `thing_kind_epm` (EPM, `max_nodes 4`) | `winner_id` = kind | :161-162 |
| `reality.cognitive.thing_context` (context_topic) | `thing_context_epm` (EPM, `max_nodes 3`) | `winner_id` clamped to [0, 2] = context | :163-166 |
| `reality.cognitive.seek_value`, `seek_range` | seek (same tick) | arrival detection | :168-170 |
| `reality.proprio.odom` | host | `[x, y, yaw]` | :156-157 |

**Cell key:** `cell = kind·context_n + ctx` (context_n 3 → 12 cells) (:167); stats key = `cell·4 + intent` with intent 0 = kick, 1 = peck, 2 = push (SkillOutcomeLoop.hpp:71-72).

### Key parameters
Config: `proximity_range 2.5`, `arrive_range 0.2`, `match_radius 0.6`, `min_samples 2`, `observe_ticks 1500` (30 s at 50 Hz), `min_conf_ticks 5`, `explore_gain 1.0`, `skill_left 0`, `skill_right 1`, `peck_id 3`, `push_id 4`, `context_n 3`, `context_pool_min 2`, `answer_m 0.08`, need/pull topics set.
Defaults relied on: `reach_m 0` → **request on the arrival tick** (the "fire at what you see" arming path is off) (SkillOutcomeLoop.hpp:116; .cpp:252-253); `reach_short_m 0.35` (irrelevant with range_seen 0).

### What it computes each tick (SkillOutcomeLoop::tick, :155-320)
1. **Fix:** after the bearing has been live ≥ 5 consecutive ticks, the thing's position is fixed in the odometry frame (same dead reckoning as seek, range = (1−prox)·2.5) and `node_`/`ctx_` are latched (:173-182). `seen_` is set true and **never reset** (it means "has ever fixed a thing").
2. **Outcome in flight:** `wait_++`; the first fixed sighting with distance < 0.6 m from the kick-time position and `wait_ > 25` ticks (0.5 s) is the outcome: `disp` = that distance; `surprise = |disp − mean| / (sd + 0.02)` (pred = 0 with no samples); Welford update of the (cell, intent) mean/variance; `ans++` if disp > 0.08 m; context pool `ctx_stats[kctx].n++` (and `.ans++`) (:186-197). If `wait_ > 1500` → `unknown++`, nothing learned ("unseen ≠ unmoved") (:198).
3. **Uncertainty rule:** a (cell, intent) is uncertain if it has < 2 outcomes, or its sd exceeds `explore_gain`·(mean sd over all keys with ≥ 2 outcomes) (:203-211).
4. **Arrival:** `seek_value` was > 0 last tick, is 0 now, `seek_range < 0.2`, `seen_`, nothing pending (:251).
5. **Ask** (:215-249): candidate intents {kick, peck, push} (all, since range_seen = 0 ≤ 0.35); choose the one with fewest outcomes for this cell, then largest sd, then the next after the last asked in the cycle (so the first ask is the kick). If that intent is uncertain → request: id = 3 (peck), 4 (push), or the kick side by the sign of the thing's lateral offset in the body frame (left ≥ 0 → `skill_left 0`, else `skill_right 1`); mark pending with the fixed position, cell, context.
6. **need** = (number of uncertain intents at the last cell) / 3, or 0 while pending or before any fix (:280-286).
7. **pull** = 1 unless something has been fixed; if no intent is uncertain at the cell → Laplace answered share `(ans+1)/(n+2)` summed over the cell's intents; else if the cell's context has ≥ 2 pooled outcomes → the context's Laplace share; else 1 (:288-313).

### Outputs
| Topic | Layout | Consumers |
|---|---|---|
| `intent.skill` | `[skill id, request (1 on the asking tick, else 0)]` (:268-271) | IntentAdapter reads a fresh request (`tick_id` == this tick, value > 0.5) (IntentAdapter.cpp:266-268); main.cpp executes `kSkills[id]` = {0 kick_left, 1 kick_right, 2 roulade, 3 peck (alpha_ground_pick.onnx, 2.8 s), 4 push (walker forward 0.25 for 1.2 s)} (main.cpp:1450-1456). With `--stop-on-arrive` and no `--skill-now`, a request while walking is deferred and fires from standing at the next stop's hand-back (main.cpp:2140-2145, 2246); a request during the brain phase of a stop fires at once (main.cpp:2253-2258); during settle / a skill / its unwind it is deferred (main.cpp:2270-2275) |
| `reality.cognitive.outcome` | 6 values on the outcome tick, zeros otherwise: `[cell, predicted, observed disp (m), surprise, n, intent]` (:272-276) — the schema text lists 5 fields | IntentAdapter copies it for logging (`outc`) (IntentAdapter.cpp:272-273); no graph module subscribes (grep of the config) |
| `reality.cognitive.outcome_need` | `[need, x, y]` (x, y = last fixed position) (:314-319) | seek renewal; IntentAdapter computes `thing_rng_`/`thing_ego_` from it (IntentAdapter.cpp:212-219) for the host's gaze-at-thing / unwind aim |
| `reality.cognitive.outcome_pull` | `[pull]` (:309-312) | seek: confidence of a freshly sighted static target (BearingSeekLoop.cpp:372-374) — a cell known not to answer is seen but carries low need |

### Learning
Per (cell, intent): count, running mean and variance (Welford) of displacement, answered count; per context: pooled count and answered count. Error signal: the observed displacement at re-sighting; surprise is published but not used to update anything except as telemetry. Live during the run; starts empty (not restored). Observed (probe log `la_probe.jsonl` / `la_probe.err:124`): 4 requests, 2 observed, 1 unknown, 1 pending; outcome rows `[10, 0.0, 0.087, 4.34, 1, 0]` (kick) and `[10, 0.0, 0.108, 5.39, 1, 1]` (peck) on cell 10 = kind 3 × context 1 — surprise = 0.087/0.02 ≈ 4.35 and 0.108/0.02 = 5.4 as the formula gives for a first sample; `nodes_known 0` at 120 s.

---

---

# §3 The HEAD brain (`mj_host/configs/head3o_h2_gaze_w10.json`)

**Modules in tick order:** `head_bridge` (JointSensorimotorBridge) → `motor_epm_head` (MotorEPMv2). No EPM.
Restored whole from `mj_host/checkpoints/head3o_gaze_h1_s2_nohold.json` (`main.cpp:1788-1794`). The later `--head-graph`
and `--load-head` win (plain assignments, `main.cpp:3368-3369`, `3561-3562`).

**In one paragraph.** The head brain keeps the head level in pitch and roll and turns the head yaw to drive a host-computed
gaze error to zero. Its self-model A was identified in an earlier babble run and is frozen in LA1 (model_lr 0,
state_model_lr 0); the only learning is the state-prior descent on the controller (C on pitch/roll feedback, h on all three
priors) at 0.02, on walks only. The gaze prior has C-weight 0, so the yaw's response to the gaze error is carried by the
tonic h, an integrator of `0.02·e·A(20,j)·G_j/anorm` with anti-windup (`MEPM:4704`, `4725-4748`). With `--look-around 0.9 2.0`
the gaze target, when nothing else holds it, is the stalest 15° sector within ±0.9 rad of the body axis if unseen for
≥ 2 s (`main.cpp:2599-2620`). The head brain's outputs become the head joint targets directly (`--head-joints`). At stops a
host sweep overrides its yaw and pitch and its learning is frozen.

**Gaze-error target, verified priority** (`main.cpp:2595-2625`): `want = gm_want` if a mover-glance source is active
(`gm_src > 0`, only with `--gaze-to-motion`, which LA1 does not set, so `gm_src = 0`, `main.cpp:2582-2590`); else
`−seek_ego` (the seek target's body bearing, + = left) while seek holds the heading reference (`last_steer()==3`); else
the look-around target if one qualifies (also requires no stop, no active or coasting chase, `main.cpp:2608`); else 0.
Error `e = wrap(want − head_yaw_joint)`, fed as sense slot 11 (`/1.4`, clamped) via `feed_gaze_error`.

## 3.1 Head adapter: what the host publishes into and reads from the head brain


Graph `mj_host/configs/head3o_h2_gaze_w10.json` (the later `--head-graph` wins), restored from
`checkpoints/head3o_gaze_h1_s2_nohold.json` (`main.cpp:1773-1798`). Seeds rewritten as in the intent adapter
(`HeadAdapter.cpp:162-169`). Because the graph publishes `action.head_yaw`, the yaw mask is off: the brain owns yaw
(`HeadAdapter.cpp:170-178`). Inspector at 7402/7403 (`HeadAdapter.cpp:187`).
Head command ranges `kHeadRange = {neck_pitch 1.10, head_pitch 1.10, head_yaw 1.40, head_roll 0.31}` rad ("the walker's
trained head-command ranges") — `HeadAdapter.hpp:26-28`.

### [Adapters 3.1] Topics PUBLISHED (every tick, 50 Hz)

| Topic | Dims | Layout | Anchor |
|---|---|---|---|
| `reality.proprio.head` | 4 | [neck_pitch, head_pitch, head_yaw, head_roll] joint positions minus HOME, each / kHeadRange, `unit()` | `HeadAdapter.cpp:213-215`; head_q from `main.cpp:2566-2567` |
| `reality.proprio.imu` | 6 | trunk gravity (3), trunk gyro (3) | `HeadAdapter.cpp:216` |
| `reality.proprio.head_sense` | 12 | see below | `HeadAdapter.cpp:226-228` |

`head_sense` slots (frame: head body, x DOWN when level, so a level head reads gravity (−1, 0, 0) — `HeadAdapter.cpp:217-225`):

| Slot | Content | Normalisation |
|---|---|---|
| 0 | head-frame gravity y (roll error) | raw |
| 1 | head-frame gravity z (pitch error) | raw |
| 2 | head-frame gravity x + 1 (the "down" shortfall; 0 when level) | raw |
| 3–5 | head gyro x, y, z × 0.3 | `unit()` (head yaw rate is gyro x, the down axis — `HeadAdapter.hpp:61-63`) |
| 6–7 | trunk gravity x, y | raw |
| 8–10 | trunk gyro x, y, z × 0.3 | `unit()` |
| 11 | **gaze error** / 1.4 rad (`--head-gaze-sense`, on); 0 when off | `unit()` |

The gaze error (`main.cpp:2595-2625`): `want` = the seek target's body bearing negated (`−seek_ego`, so + = left) when the
seek loop held the reference this tick (`last_steer == 3`), else 0; `--look-around 0.9 2.0`: 24 sectors of 15° in the
odometry frame are marked "seen" when within ±22.5° of the current view direction (odometry yaw + head yaw); when nothing else
holds the gaze (no seek steering, no stop, no chase/coast) the target is the bearing (relative to the body, ±0.9 rad max) of
the stalest sector if it is ≥ 2.0 s old. `e = wrap(want − head_yaw_from_home)`, fed via `feed_gaze_error` before the head tick.

Consumers in the head graph: `head_bridge` reads `reality.proprio.head` at `proprio_indices [1, 3, 2]` = head_pitch,
head_roll, head_yaw (neck pitch not bridged), action topics `[action.head_pitch, action.head_roll, action.head_yaw]`,
`load_topic reality.proprio.head_sense`, `load_slots 12` (`head3o_h2_gaze_w10.json:15-31`) → state 3 × 3 + 12 = 21.
`motor_epm_head` priors `state_prior_indices [9, 10, −1]`, targets 0 (`:80-85`) = head roll error, head pitch error, gaze
error → 0.

### [Adapters 3.2] Topics READ and what the host does with them
- `action.neck_pitch`, `action.head_pitch`, `action.head_yaw`, `action.head_roll`: `cmd_i = kHeadRange_i × clamp(accel)`
  rad from HOME (`HeadAdapter.cpp:233-237`). The LA1 head graph publishes no `action.neck_pitch`, so `last_cmd_[0]` stays at
  0 (reset value, `HeadAdapter.cpp:343`).
- Overrides (host scaffolds at stops): `set_yaw_override` / `set_pitch_override` replace the yaw / pitch command with a
  target, moved at ≤ `slew_` rad/s (step = slew/50 per tick), primed from the current command — `HeadAdapter.cpp:244-259`.
  Used by the stop's gaze SWEEP (`--stop-gaze 0.35 0.08 1.0 6 6 --stop-gaze-sweep 0.6 0.7 --stop-gaze-sweep-slow 1`:
  toward the least-visited of 14 × 3 gaze cells, ±0.7 rad yaw, at 0.6 rad/s — `main.cpp:2283-2318`, probe line "gaze SWEEP
  at stops…") and by the impeded look (`--look-up-when-impeded 3 -0.3 1.5 0.6`: head pitch target −0.3 rad, yaw 0, for 1.5 s
  at the impeded stop — `main.cpp:2574-2578`, `:3590-3592`). VOR, rate loop, phase feed-forward and release slew: off.
- Return value: the four commands (rad from HOME) — `HeadAdapter.cpp:331-333`.

### [Adapters 3.3] Learning gates
`set_learning` zeroes/restores the same seven MotorEPM rates (`HeadAdapter.cpp:348-374`). With `--stop-freeze-head` the head
brain is frozen through every stop (`main.cpp:2209`) and re-armed (`on_reset` + learning on) when the stop ends
(`main.cpp:2497`); also frozen during rescues (`main.cpp:2022`, `:2036`).

### [Adapters 3.4] How the head commands reach the joints (`--head-joints`, Track A at the head)
When the brain drives: `head_targets_i = HOME_{5+i} + cmd_i` (`main.cpp:2632-2634`); `--head-forward` 0; seek/chase gaze off;
then the translation `−fore` on neck and head pitch (§2.8). `command.head` sent to the policy is zeroed except the translation
(`main.cpp:2652-2661`). The final joint targets for indices 5–8 bypass the policy's head outputs and the policy low-pass:
`ctrl[5+i] = head_targets_i` on the walk (`main.cpp:2773-2774`) and during a stop with `--stop-keep-head`
(`main.cpp:2761-2762`). Probe: "head joints: the head brain writes the four head joint targets (Track A at the head)".

---

## 3.2 `head_bridge` — JointSensorimotorBridge


Params: `proprio_input_topic reality.proprio.head`, `proprio_indices [1,3,2]`, `group_size 3`,
`action_topics [action.head_pitch, action.head_roll, action.head_yaw]`, output `reality.motor_limb.head`,
`load_topic reality.proprio.head_sense`, `load_slots 12`. Width 3·3 + 12 = **21** (matches the checkpoint's `n: 21`).

Inputs (`HeadAdapter::tick`, `HA:52-90`):
- `reality.proprio.head` = 4 values `[neck_pitch, head_pitch, head_yaw, head_roll]` as (joint − HOME) / kHeadRange,
  clamped; kHeadRange = {1.10, 1.10, 1.40, 0.31} rad (`HA:72-75`, `mj_host/src/HeadAdapter.hpp:28`, joint offsets from
  `MAIN:2567-2568`). Indices [1,3,2] pick head_pitch, head_roll, head_yaw, matching the action order.
- `reality.proprio.head_sense` = 12 values (`HA:86-88`).

**Resolved state vector of the head brain's `motor_epm_head` (21):**

| idx | from end | content |
|---|---|---|
| 0,1,2 | | head_pitch position, own act, Δ |
| 3,4,5 | | head_roll position, act, Δ |
| 6,7,8 | | head_yaw position, act, Δ |
| 9 | −12 | head-frame gravity y = **roll error** (0 when level) |
| 10 | −11 | head-frame gravity z = **pitch error** |
| 11 | −10 | head-frame gravity x + 1 ("down" shortfall) |
| 12,13,14 | | head gyro x, y, z ·0.3 |
| 15,16 | | trunk gravity x, y |
| 17,18,19 | | trunk gyro x, y, z ·0.3 |
| 20 | **−1** | **gaze error** / 1.40 rad (`--head-gaze-sense`) |

Gaze error (`MAIN:2595-2624`): `want − head_yaw_joint`, wrapped to ±π, where `want` is (in priority) a mover glance
target, else the seek target's body bearing while seek steers (`−seek_ego`), else (with `--look-around 0.9 2.0`) the
bearing of the least-recently-seen 15° sector within ±0.9 rad if older than 2.0 s, else 0 (straight ahead).

## 3.3 `motor_epm_head` — MotorEPMv2 (mechanism in §2.2 [MotorEPM 2.x])


**Role:** the head's attitude and gaze controller: keeps the head level in pitch and roll and turns the head yaw toward the
gaze target, using a self-model identified in an earlier babble run.

**Key params:** motor_dim 3 (head_pitch, head_roll, head_yaw), model_lr 0, state_model_lr 0, model_trace 0.05, ctrl_lr 0,
bias_lr 0, sat_lr 0, babble_ticks 0, explore_noise 0, motor_gain 1, state_prior_gain 1, state_prior_lr 0.02
(h_lr follows), no step (state_prior_step_gain default 0, `MEPM:342-360`), no gates, no calm.

**Priors resolved on the 21-element state:**

| k | index | element | target | motors (`state_prior_motors`) | C weight (`state_prior_c_weights`) |
|---|---|---|---|---|---|
| 0 | 9 | head roll error (head-frame g_y) | 0 | first 2: head_pitch, head_roll | 1 |
| 1 | 10 | head pitch error (head-frame g_z) | 0 | first 2 | 1 |
| 2 | −1 → 20 | gaze error / 1.40 rad | 0 | all 3 | **0**: no C update, h only ("pure reach") |

**What it computes each tick:** `ξ` against a frozen model `x̂ = A·ỹ + b` (Bx unused because state_model_lr 0,
`MEPM:4330`); the descent on C (indices 9, 10, through pitch and roll motors) and on h (all three priors) at 0.02;
command `y = clamp(tanh(C·x + h), ±1)`. Because the gaze prior has C weight 0, the yaw's response to the gaze error is
carried by h, an integrator of `0.02·e·A(20,j)·G_j/anorm` (anti-windup at |tanh| > 0.95, `MEPM:4725-4748`).
The restored controller is near-identity: C has ≈1.0 on head_pitch←pos(head_pitch) and head_roll←pos(head_roll) and
≈0 on yaw ("the yaw's identity hold released"), h = 0 (checkpoint, §6). So the level and gaze responses beyond the
identity hold are built by the descent within each run, and only on walks.

**Outputs and consumers:** `action.head_pitch`, `action.head_roll`, `action.head_yaw`. `HeadAdapter::tick` reads them
(plus `action.neck_pitch`, which this graph never publishes) and scales by kHeadRange (`HA:93-97`); the yaw is not masked
because the graph owns `action.head_yaw` (`HA:30-37`, `HA:103`). Stop-time yaw/pitch overrides (sweep, look-around glance,
impeded look) replace the commands after the tick (`HA:104-120`; `MAIN:2281-2325`, `MAIN:2572-2593`). With `--head-joints`
the result becomes the four head joint targets (`MAIN:2631-2634`), with the bird's-neck translation added
(`MAIN:2655-2662`). `head_bridge` reads the three actions back as act slots.

**Learning during the run:** only `state_prior_lr 0.02` is nonzero. `HeadAdapter::set_learning` freezes the same seven
rates (`HA:208-233`). With `--stop-freeze-head` it is frozen at every stop start (`MAIN:2209`, also `MAIN:2514`,
`MAIN:2526`, `MAIN:2556`) and re-enabled at the stop's end (`MAIN:2497`); also frozen during rescues (`MAIN:2022`,
`MAIN:2036`). So the head brain learns (C and h only) on walks.

---

---

# §4 The STOP brain (`mj_host/configs/a1v2_r19_settle_each.json`)

**Modules in tick order:** `legs_bridge` (JointSensorimotorBridge) → `head_bridge` (JointSensorimotorBridge) →
`motor_epm_legs` (MotorEPMv2, 2 legs × 5 joints) → `motor_epm_head` (MotorEPMv2, 4 head joints) → `regime_epm` (EPM).
Restored whole from `mj_host/checkpoints/duck_r19_s2.json` (`main.cpp:1863-1868`).

**In one paragraph.** At a stop the walker first holds a zero twist (Settle, ≤ 2 s, until still); then the stander takes
the 10 leg joints (`ctrl = stander->act(body)`) with targets `origin + 0.35·u`, where the origin is the `alpha_stand`
scaffold's measured standing equilibrium (`main.cpp:1185-1221`, `OBA:214-240`). Its priors pull trunk pitch/roll, their
rates and the five joint positions of each leg to 0 (upright, at the calibrated stand pose). A regime EPM over 12 IMU/head
sense values keys up to 6 banks of the self-model. It learns during the stand, gated to near-upright, and is frozen otherwise.
The walker takes the legs back when trunk lean exceeds 6.5° and is rising for 3 ticks, or at the stop's end
(`main.cpp:2259-2269`, `2463`). Its own head motor module is frozen for the whole run and its head commands are replaced by the
head brain's (`--stop-keep-head`, `main.cpp:1870`, `2761-2762`). The stop schedule and end rules are host mechanisms (§5).

## 4.1 OgmaBrainAdapter: what the host publishes into and reads from the stop brain


Graph `mj_host/configs/a1v2_r19_settle_each.json`, restored from `checkpoints/duck_r19_s2.json`, constructed with amplitude
0.35 rad and the calibrated stand origin (`main.cpp:1854-1868`). `HOME` here = the scaffold's measured equilibrium, not the
keyframe (`OgmaBrainAdapter.hpp:62-71`; probe: "max |delta| from keyframe 0.1045 rad; head CoM +0.0307 −0.0002 m").
`--stop-keep-head` → `freeze_module("motor_epm_head")` for the whole run (`main.cpp:1870`, `OgmaBrainAdapter.cpp:270-288`;
probe "freeze: motor_epm_head for the rest of the run"). Inspector 7404/7405 (`main.cpp:1860`).

### [Adapters 4.1] Topics PUBLISHED (each tick it is ticked)

| Topic | Dims | Layout | Anchor |
|---|---|---|---|
| `reality.proprio.joints` | 14 | (q_i − HOME_i)/0.35, clamped [−1,1], policy-joint order | `OgmaBrainAdapter.cpp:85-91` |
| `reality.proprio.imu` | 6 | trunk gravity, trunk gyro | `:95-98` |
| `reality.proprio.lean` | 2 | [g_x, g_x] | `:121` |
| `reality.proprio.lean2` | 4 | [g_x, g_y, g_x, g_y] | `:133` |
| `reality.proprio.lean4` | 8 | [g_x, g_y, 0.3·w_y, 0.3·w_x] × 2 (rates clamped) | `:147-150` |
| `reality.proprio.sense` | 24 | the 12-slot vector below, repeated twice (group-major) | `:177-182` |
| `reality.proprio.sense1` | 12 | the 12-slot vector once (for the regime EPM) | `:186` |

The 12-slot sense (`OgmaBrainAdapter.cpp:152-178`): 0 g_x, 1 g_y, 2 pitch rate ×0.3, 3 roll rate ×0.3, 4 yaw rate ×0.3,
5–7 accel x/y/z ÷ 20, 8 head-frame gravity x (labelled "head-frame pitch"), 9 head-frame gravity y ("roll"), 10–11 head-subtree
CoM offset from its standing value (trunk x, y) ÷ 0.1 m — all clamped to [−1, 1] except 0–1.

Consumers: `legs_bridge` (`reality.proprio.joints` indices 0–4, 9–13; two output groups of 5; load `sense`, 12 slots →
group 0 gets slots 0–11, group 1 slots 12–23, identical) and `head_bridge` (indices 5–8, load 12) —
`a1v2_r19_settle_each.json:23-80`; `regime_epm` on `reality.proprio.sense1` (12 dims, `:273`). Probe state widths: legs 27
(5×3 + 12), head 24 (4×3 + 12); regime EPM projection_dim 96.

### [Adapters 4.2] Topics READ and what the host does with them
- `action.<joint>` for all 14 policy joints (`OgmaBrainAdapter.cpp:54`, `:200-205`): `target_i = clamp(HOME_i + 0.35 ×
  clamp(u_i, −1, 1), joint range from the MJCF)`; no servo low-pass (`--servo-filter` off) — `:214-240`. In LA1 the head four
  are then overwritten by the head brain's targets (`main.cpp:2761-2762`), so only the 10 leg targets are applied.
- `reality.proprio.regime` (RealityToken): winner and TLE, for diagnostics — `:206-211`.

### [Adapters 4.3] When it runs and learns
Stop phases (`main.cpp:2194-2271`): stops every 80 s from 0 s for up to 60 s, plus arrival / stuck / impeded stops; twist
zeroed; during `Settle` (≤ 2 s, until `g_z < −0.999 && max|w| < 0.15`) the stander is ticked frozen to observe
(`main.cpp:2217`); then the hand-back (`att_gate` 0 = no gate) → `on_reset()` + `set_learning(true)` (`main.cpp:2245-2247`);
in `Brain` it drives the legs each tick with the regime-learning gate `g_z < −0.90` (`main.cpp:2758-2760`); the walker takes
the legs back when lean > 6.5° and rising for 3 ticks, or at the stop's end (cloud-growth end `--stop-cloud-end 0.45`), and the
stander is frozen again (`main.cpp:2255-2271`, `:2463`, `:1996-2001`). Freezing zeroes the seven MotorEPM rates and reads them
back (`OgmaBrainAdapter.cpp:290-353`). Probe: "stops: 4 started, 4 hand-backs … the joint brain stood 7.7 s".

---

## 4.2 `legs_bridge` and `head_bridge` — JointSensorimotorBridge


Inputs (`OgmaBrainAdapter::publish_sensors`, `OBA:67-187`):
- `reality.proprio.joints` = 14 policy joints as (q − origin)/0.35 rad, clamped (`OBA:84-91`; amplitude 0.35 from
  `MAIN:1859`; origin = the scaffold's measured stand equilibrium, probe log line 47). Order
  (`mj_host/src/DuckBody.cpp:10-14`): 0–4 left hip_yaw, hip_roll, hip_pitch, knee, ankle; 5–8 neck_pitch, head_pitch,
  head_yaw, head_roll; 9–13 the right leg.
- `reality.proprio.sense` = the 12-slot IMU sense **duplicated** to 24 values (`OBA:162-182`):
  0 g_x (pitch), 1 g_y (roll), 2 pitch rate·0.3, 3 roll rate·0.3, 4 yaw rate·0.3, 5–7 accel/20,
  8 head-frame pitch, 9 head-frame roll, 10–11 head-CoM offset / 0.1 m.

`legs_bridge`: indices [0..4, 9..13], `group_size 5`, two outputs (`reality.motor_limb.left/right`), `load_slots 12`:
each leg's state = 5 × [pos, act, delta] (0–14) + the 12 sense slots (15–26) = **27** (probe log `state_dim=27`, line 109).
Leg 0 reads sense[0..11], leg 1 sense[12..23] (the duplicate), so both legs see the same 12 values.
`head_bridge` (stop): indices [5,6,7,8], `group_size 4`, `load_slots 12`: 4 × 3 + 12 = **24** (probe log `state_dim=24`).

---

## 4.3 `motor_epm_legs` and `motor_epm_head` — MotorEPMv2 (mechanism in §2.2 [MotorEPM 2.x], banks and consolidation in [MotorEPM 2.6])


**Role:** stands the legs during stops. `motor_epm_head` here is present in the graph but its commands are discarded
(`--stop-keep-head`).

**Key params (both):** model_lr 0.02, ctrl_lr 0.1, bias_lr 0, sat_lr 0, state_model_lr 0.05, model_trace 0.15,
reg_eps 0.01, max_dctrl 0.05, babble_ticks 3000 / scale 0.25 / isolate 1 / hold 6 / owns_a 1, explore_noise 0.05,
motor_gain 1, state_prior_gain 1, state_prior_lr 0.1, state_prior_h_lr 0 (C only), state_prior_calm 1 with
calm_indices [−12, −11] and calm_fixed 0.3, regime_topic `reality.proprio.regime`, regime_banks 6, consolidation as §2.6.
`motor_epm_legs`: n_legs 2, motor_dim 5; `motor_epm_head`: n_legs 1, motor_dim 4.

**Priors resolved:**
- `motor_epm_legs` (27 per leg): −12 → 15 g_x (pitch), −11 → 16 g_y (roll), −10 → 17 pitch rate, −9 → 18 roll rate,
  then 0, 3, 6, 9, 12 = the five joint positions (hip_yaw, hip_roll, hip_pitch, knee, ankle) — all targets 0, i.e. upright
  and at the calibrated stand origin. `consolidate_n 4`: the gate reads the first four (attitude).
- `motor_epm_head` (24): −12 → 12 g_x, −11 → 13 g_y, −10 → 14, −9 → 15 the rates, then 0, 3, 6, 9 = neck_pitch,
  head_pitch, head_yaw, head_roll positions — targets 0.

**What it computes each tick (legs):** model update (b and Bx only, since `babble_owns_a 1` with `babble_isolate 1` keeps
LMS off A permanently, `MEPM:4350-4351`) in the active regime bank; HK update `dC = 2·lr_scale·ctrl_lr·(AG)ᵀq·(qᵀL)`,
`q = (LLᵀ + εI)⁻¹ξ̃`, clamped to ‖dC‖ ≤ 0.05 (`MEPM:4391-4528`, clamp `MEPM:4525-4527`); prior descent on C at 0.1 (not annealed); command
`y = clamp(tanh(z_att + 0.3·(C·x + h − z_att)) + N(0, 0.015²), ±1)` (§2.6). The `OgmaBrainAdapter` turns it into joint
targets `home + 0.35·u`, clamped to the joint range (`OBA:211-227`; `--servo-filter` is not set). With
`--head-joints --stop-keep-head` the head joint targets are overwritten by the head brain's (`MAIN:2761-2762`).

**Learning during the run (code path, which the probe log's freeze lines do not show in full):**
- After the restore, `stander->set_learning(false)` freezes all MotorEPMv2 rates (`MAIN:1869`; probe log lines 49-62), and
  `freeze_module("motor_epm_head")` freezes the stop-brain head for good (`MAIN:1870`, `OBA:270-288`; probe log line 63).
- At each stop's hand-back to the joint brain, `stander->on_reset(); stander->set_learning(true)` (`MAIN:2247`):
  `motor_epm_legs` gets its saved rates back (`OBA:322-327`); `motor_epm_head` stays at 0 (`module_frozen_`, `OBA:323`).
- While it stands, a second gate freezes it unless near-upright: `set_regime_learning(g[2] < −0.90)` (`MAIN:2758`,
  `OBA:265-268`, combined at `OBA:291`).
- When the stand ends (handoff to the walker, survival to the end, or a rescue), `end_stop_drive` freezes it again
  (`MAIN:1996-2000`, called at `MAIN:2027`, `MAIN:2267`, `MAIN:2463`).
So `motor_epm_legs` learns during stands only (model b/Bx at 0.02/0.05, HK at 0.1·(1 − c), prior at 0.1). The probe's end
summary shows `state_prior_lr=0.00` because the stander was frozen at the time of printing (line 109). Probe JSONL: the joint
brain stood on 419 of 6000 ticks (`stop == 2`), the probe summary says 7.7 s (line 96).

**Outputs:** the 10 leg `action.*` topics (and 4 head ones from the stop-brain head), read by `OgmaBrainAdapter::act`
after the tick (`OBA:197-205`).

---

## 4.4 `regime_epm` — EPM (algorithm in §2.3 [EPM A.x])


**Config:** `mj_host/configs/a1v2_r19_settle_each.json:262-278`
- group proprio, name regime, rbf, input `reality.proprio.sense1`, 12 dims;
- **no `dim_min`/`dim_max`**, so the default is [-1,1] per dim;
- `max_nodes` 64, `master_seed` 4242, `process_every_n_ticks` 5, `mitosis_error_threshold` 0.1 (inert);
- `health_death_spares_baked` true, `min_insertion_error` 0.06;
- **`baking_threshold` omitted, so 100** (`gng.hpp:89`; the checkpoint confirms `baking_threshold: 100`).

**Restored** from `mj_host/checkpoints/duck_r19_s2.json`, `graph.modules.regime_epm`:
- 3 nodes, all baked, 2 edges, dim 96, GNG `step` 70787;
- see A.8 for the settings `from_json` drops (`max_nodes` reverts to 2000, `health_death_spares_baked` to false).

**Input.** The producer is OgmaBrainAdapter `publish_sensors` (`mj_host/src/OgmaBrainAdapter.cpp:152-186`). It publishes
`reality.proprio.sense1` (prefix `OgmaBrainAdapter.cpp:71-79`) before the stop graph ticks (`OgmaBrainAdapter.cpp:191-193`).
There are 12 slots, each clamped to [-1,1]:

| slot | content |
|---|---|
| 0, 1 | trunk projected-gravity x and y (pitch, roll) |
| 2, 3, 4 | pitch, roll and yaw rate × 0.3 |
| 5, 6, 7 | accelerometer x, y, z / 20 |
| 8, 9 | head-frame gravity x and y |
| 10, 11 | head-CoM offset Δx, Δy / 0.1 m from its calibrated reference |

- Gravity and gyro come from `body.gravity()` and `body.gyro()` (`OgmaBrainAdapter.cpp:93-94`).
- There is no mean removal (for example, `az/20` sits near −0.49 at rest). With the default ranges, each slot maps to [0,1] as `(x + 1)/2`.

**Computes.** Every 5th stop-brain tick: RBF 12 → 96 (Halton) → GNG.
- The stop brain ticks only during stops, in the Settle and Brain phases (A.8).
- It is never frozen, but all 3 restored nodes are baked, so their prototypes are fixed. Only an insertion (`q.ema_error ≥ 0.06`)
  could change the vocabulary.
- **Probe:** 3 nodes, 3 baked, ema_tle 0.084, final QE 0.052, `qe_lag1` 0.61 (`la_probe.err:111`).

**Output.** `reality.proprio.regime`. Consumers:

| consumer | reads | anchor |
|---|---|---|
| `motor_epm_legs` (MotorEPMv2) | `winner_id`, through the R1 regime socket, keying up to **`regime_banks` 6** self-model banks | `regime_topic` `a1v2_r19_settle_each.json:165-166`; `cpp_core/src/ogma/modules/MotorEPMv2.cpp:1752-1763`, `4187-4195` |
| `motor_epm_head` (MotorEPMv2) | the same as above | `a1v2_r19_settle_each.json:250-251` |
| OgmaBrainAdapter (host) | `winner_id`, `tle` → `regime_id()` / `regime_tle()` | `OgmaBrainAdapter.cpp:206-211` |

- `regime_epm` is **last** in tick order, and the MotorEPMv2 subscription is Direct, which is delivered synchronously on publish
  (`cpp_core/include/ogma/Bus.hpp:114`). The motors' tick therefore uses the **previous** tick's regime winner.
- `motor_epm_head` is frozen and its commands are not applied (`--stop-keep-head`; `main.cpp:1870`). Its bank switching
  still runs.
- **Probe:** the stander diag shows `rbsw=28` bank switches and 3 populated banks for the legs (`la_probe.err:109`).

---

---

# §5 Host mechanisms (not brain modules): scaffolds, reflexes, schedules, sensors, the twist path

## 5.1 Every flag of the LA1 argv, what it does, where

Scope: everything `mj_host` does around the three brains: the run loop, the scaffold networks (Pollen's ONNX
policies), reflexes, schedules, the scene and the clock. All anchors are relative to `/home/xaqmusic/xaq-ai`.
"probe log" = `scratchpad/la_probe.err` (a 120 s LA1 run; **its train line says `(seed 5)`**, so the probe was run with
`--seed 5`, not the preset's `--seed 1`; the scene manifest's "seed 1" is the scene GENERATOR's seed, see §E).

### [Host 0] The full argv and the mode

The ★ LA1 preset is `tools/duck_launcher/presets.json:27-48` (mode `level2`, config `a1v2_t11_not_target.json`,
`secs 600`, `seed 1`, `noise 0.05`, `start checkpoint` = `duck_forebody_s1.brain.json`, scene
`scene_playroom_train.xml`). `tools/duck_launcher/launcher.py:201-265` builds:
`--level2 --graph mj_host/configs/a1v2_t11_not_target.json --secs 600 --seed 1 --noise 0.05 --load-brain
mj_host/checkpoints/duck_forebody_s1.brain.json <host_args verbatim> mj_host/models/microduck/scene_playroom_train.xml`
(`--noise` added for level2 at launcher.py:228-233; `--load-brain` at 255-259; host_args shlex-split at 262-263; the
scene path last at 264-265). In "watch" output the viewer runs the host in live mode, which passes `--realtime`
(pacing only; `main.cpp:3280-3281`, `TickPacer` `main.cpp:122-136`).

`main()` dispatches `--level2` to `cmd_level2(scene, graph, seconds, seed, emit=true, pushes, nullptr, noise)`
(`mj_host/src/main.cpp:3640-3641`). `cmd_level2` is `main.cpp:1610-3190`; its tick loop is `main.cpp:2013-3083`.

### [Host A] Every flag of the LA1 argv

Notation: "parse" = the argv branch in `main()`; "impl" = where it acts. All in `mj_host/src/main.cpp` unless a
file is named.

#### [Host A.1] Mode, session, start state

| Flag + value | What it does (plain words) | Parse | Impl |
|---|---|---|---|
| `--level2` | Selects the level-2 harness: Pollen's walking network (`alpha_walking.onnx`) drives the joints every tick; the intent brain commands its twist; the head brain commands the head; the stop brain stands the legs at stops; the standing network rescues falls. | 3313-3315 | 3640-3641 → `cmd_level2` 1610 |
| `--graph configs/a1v2_t11_not_target.json` | The intent brain's graph. | 3610-3611 | `IntentAdapter brain(graph, seed)` 1617; ctor `IntentAdapter.cpp:22-95` |
| `--secs 600` | Run length: `ticks = int(seconds*50)` = 30 000 ticks. Default 3.0 s. | 3316-3317 (default 3294) | 1844 |
| `--seed 1` | Seeds (1) every module's `master_seed`/`seed` param in all three brains via `namespace_seed(seed, module id)` when seed≠0 (`IntentAdapter.cpp:24-31`, `HeadAdapter.cpp:22-29`, `OgmaBrainAdapter.cpp:37-42`); (2) the reset-noise RNG (`DuckBody.cpp:123`, `mt19937_64(seed)`); (3) the stop gaze sweep's RNG `mt19937(seed*7919+17)` (1921); (4) the train's schedule phase and start point along its track, `seed % 6` (1760-1761). It does NOT change the scene layout (a fixed XML file, §E). `calibrate_stand_home(probe, seed, …)` resets with noise 0 (1191), so the seed has no effect there. UNCLEAR: whether `restore_state` from the checkpoints overwrites the reseeded module RNG state (module code, not host). | 3320-3321 | as listed |
| `--noise 0.05` | Reset noise: at the start every ROBOT joint coordinate (free joint excluded; scene objects excluded via `qpos_is_robot_`) gets an independent N(0, 0.05 rad) offset from the STAND keyframe. | 3318-3319 | `body.reset("STAND", reset_noise, seed)` 1704; `DuckBody.cpp:115-129` (robot-only mask `DuckBody.cpp:83-92`) |
| `--load-brain checkpoints/duck_forebody_s1.brain.json` | Restores the intent brain from a saved run, **only module `motor_epm_intent`** by default (`g_load_brain_modules = "motor_epm_intent"`, 1500); every other intent module (map, cloud, things, play, seek, outcome…) starts fresh. The body is NOT restored (stays at its reset). No babble. Prints the walker's authority table after restore. | 3338-3339 | 1708-1729 (module filter 1717-1722) |
| `<scene>.xml` (positional) | The world: `mj_host/models/microduck/scene_playroom_train.xml` (§E). | 3617-3618 | `DuckBody body(scene)` 1613 |

#### [Host A.2] The head brain and the head joints

| Flag + value | What it does | Parse | Impl |
|---|---|---|---|
| `--head-graph configs/head2_h2_level_slow.json` … `--head-graph configs/head3o_h2_gaze_w10.json` | Head brain graph. Given twice; plain assignment, **the second wins** (head3o). The head2 file is never loaded. | 3368-3369 | `HeadAdapter` built 1774-1775 |
| `--load-head checkpoints/head2j_h1_s2.json` … `--load-head checkpoints/head3o_gaze_h1_s2_nohold.json` | Head brain state, whole graph. Second wins. | 3561-3562 | 1788-1794 |
| `--head-joints` | "Track A at the head": the head brain's four commands become the head JOINT TARGETS (HOME + command), written over the walker's head outputs; the walker is told a zero head command (except the translate below). | 3435-3436 | 2632-2662 (targets), 2774-2775 (walk), 2738-2739 (push window), 2761-2762 (stop stand) |
| `--head-gaze-sense` | The head brain's 12th sense slot (index 11 of `reality.proprio.head_sense`) carries the gaze error / 1.4 rad: (target bearing, + = left) − (head yaw joint from HOME). Target = the seek loop's bearing (negated `seek_ego`) while seek steers (`last_steer()==3`), else the look-around target (below), else 0 (straight ahead). | 3491-3492 | `head->set_gaze_sense` 1776; error computed 2595-2625; slot `HeadAdapter.cpp:121-123` |
| `--look-around 0.9 2.0` | On the walk, when nothing else holds the gaze (no seek steering, no stop, no chase active or coasting, no mover attention), the gaze-error target is the **stalest** of 24 sectors (15° each, odometry frame) within ±0.9 rad of the body axis, if it has not been inside the sensor's ±22.5° cone for ≥ 2.0 s. Sectors inside the cone (odom yaw + head yaw) are refreshed every tick. It is a target for the head brain's LEARNED gaze, not a commanded sweep. | 3584-3585 | 2599-2620 (refresh 2601-2606; choice 2608-2618) |
| `--intent-head-translate 0.6` | The intent brain's 4th motor `action.head_fore` slides the head fore-aft: target = 0.6 × 1.10 rad × clamp(action) (`IntentAdapter.cpp:332-334`); rate-limited at 1.0 rad/s (default `g_translate_rate`; the optional RATE was not given), **centred (0) at stops**; subtracted from BOTH the neck-pitch and head-pitch targets (the view stays level) and the walker's command head[0..1] = −fore ("the policy is told it"). Requires `--head-joints`. | 3509-3511 | 1651-1655; 2653-2661 |
| `--intent-fore-sense` | Where the head sits fore-aft = −½((q5−home5)+(q6−home6)), divided by (0.6×1.10), is inserted as the FIRST slot of the intent brain's `reality.proprio.sense`. | 3512-3513 | 2075-2079; `IntentAdapter.cpp:161` |

#### [Host A.3] The stop schedule and who drives at a stop

| Flag + value | What it does | Parse | Impl |
|---|---|---|---|
| `--stop-from 0 --stop-every 80 --stop-secs 60` | Scheduled stops: a stop STARTS on any walker-driven tick with no stop in progress where `(t − 0) % 4000 == 0` (t = 0, 80 s, 160 s, …), and also on the error triggers below. **Every** stop (any trigger) needs `(ticks − t) > 3000`, i.e. no stop starts in the last 60 s of the run. A timer tick that falls inside another stop is skipped, not deferred. `--stop-secs 60` is the CAP on a stop's length (`stop_left = 3000`); stops normally end earlier by the cloud rule. "The timer stays as the floor" = error-triggered stops are added to, not instead of, the schedule. | 3372-3377 | start condition 2198-2210; cap 2212, 2456; banner 1626 |
| `--stop-brain configs/a1v2_r19_settle_each.json` `--stop-load checkpoints/duck_r19_s2.json` | The stop brain ("stander"): built on a probe body after a 3 s alpha_stand stand calibration (`calibrate_stand_home` 1185-1221: command origin = the scaffold's mean pose over its last second), amplitude 0.35, restored from the checkpoint, **learning off** at construction. | 3378-3381 | 1854-1880 |
| (who drives during a stop) | Phase **Settle**: the walker holds with zero twist (the `else` walker branch 2765-2776, twist zeroed 2500); the stander ticks frozen to keep its attitude error fresh (2217). Settle ends when still (`g_z < −0.999` and max\|ω\| < 0.15 rad/s) or after 2 s (`settle_s` 1528). Then, with no attitude gate (`--stop-att` absent → 0), phase **Brain**: the stander takes the legs, `on_reset()` + `set_learning(true)` (2244-2247); `ctrl = stander->act(body)` with the head joints overwritten by the head brain's targets (2755-2764). **Hand-back to the walker** ("the walker takes them back past 6.5° of lean"): trunk lean `atan2(√(gx²+gy²), −gz)` above 6.5° (`handoff_lean` default 1527) AND rising for 3 consecutive ticks (`confirm_ticks` 1529) → `end_stop_drive(true)` (stander frozen, walker re-seeded from the current pose) and phase **Walker** for the rest of the stop (2259-2269). At the stop's end, a stander still in Brain phase is likewise handed back (2463). | — | 1881-1885, 2211-2270, 1996-2001 |
| `--stop-keep-head` | At stops the head stays the head brain's: the stander's `motor_epm_head` is frozen for the whole run (`freeze_module`, `OgmaBrainAdapter.cpp:270-288`) and its head outputs are replaced by the head brain's targets. | 3390-3391 | 1870; 2761-2762 |
| `--stop-freeze-head` | The head brain's learning is OFF through every stop (start → end), even though it keeps the head; reason: at stops the gaze override replaces its yaw/pitch commands. | 3392-3393 | off: 2209, 2514, 2526; on (+`on_reset`): 2497 |
| `--map-on-stop` | The place map (`map_epm`) learns only at stops: at startup and at each stop's end its `min_insertion_error=1e9, epsilon_b=0, epsilon_n=0, stale_prune_enabled=false`; restored to the graph's values when a stop's Settle completes. Baking is NOT gated (probe log: "baked 3 at stops / 4 on walks"). | 3421-3422 | `set_map_learning` `IntentAdapter.cpp:438-455`; calls 1903 (off), 2241 (on), 2461 (off), 2026 (off on rescue) |
| `--stop-on-arrive` | A stop starts when the seek loop ARRIVES: the adapter flags arrival when `seek_value` drops from >0 to 0 with `seek_range < 0.3 m` (`IntentAdapter.cpp:276`). Also the look stop after an unwind is counted as an arrival stop. And with this flag a graph skill request is deferred to the next stop's hand-back (A.5). | 3474-3475 | 2171-2172, 2198, 2204; 2144 |
| `--stop-on-chase` | A stop ENDS on the tick the seek loop's chase becomes active (rising edge), while a stop is in progress. | 3472-3473 | 2193-2197 |
| `--stop-on-stuck 8` | A stop starts on a forward STALL longer than 8 × the body's own running stall length: stalled = forward command > 0.75 of its 0.4 m/s range AND sensed forward speed < 0.25 of range; fires once per stall when `stall_run > 8 × stall_med` and ≥ 50 ticks; `stall_med` (init 12.5 ticks) tracks completed stall lengths by EMA 0.05. | 3532-3533 | `brain.set_stuck` 1664; `IntentAdapter.cpp:344-369`; 2188, 2198 |
| `--stuck-escape 6` | After a STUCK stop ends, the heading reference is held for 6 s at the freest sector of the stop's cloud view (8 sectors over ±64°; ties broken toward straight ahead); the loops' bearings are ignored meanwhile (`last_steer_=4`) and the stall detector re-arms at its end. Also the escape length for an impeded "wall" verdict. | 3466-3467 | 2467-2478; `set_ref_hold` `IntentAdapter.hpp:207`; `IntentAdapter.cpp:278-283`; 2489 |
| `--stop-is-still` | During a stop's standing phases (Brain or Walker) the cast handed to CloudMap is flagged STILL regardless of the gyro (efference: the body commanded the stop). Otherwise still = `g_z < −0.999` and max\|ω\| < 0.15. | 3588-3589 | 2827-2828 |
| `--stop-cloud-end 0.45` | A stop ENDS when the stop's own (non-walking) open cloud has grown, over a 2 s window, by less than 0.45 × the highest 2 s growth seen this stop, for 2 s in a row (100 ticks). Scale-free per stop. Replaces the gaze's quiet rule (the QUIET count no longer ends stops). A cloud filed mid-stop restarts the judgement. | 3450-3451 | 1956-1963; 2440-2455; quiet rule disabled at 2321/2419 (`!cloud_end_on`) |

#### [Host A.4] What the head does at a stop (the gaze)

| Flag + value | What it does | Parse | Impl |
|---|---|---|---|
| `--stop-gaze 0.35 0.08 1.0 6 6` | YAW_SD 0.35, PITCH_SD 0.08, HOLD 1.0 s, MAX 6.0 s, QUIET 6. With the sweep on (below) the random steps are NOT used; what survives: the pitch band = `[gaze_down − 0.7×0.08, gaze_down + 2×0.08]` = **−0.056 … +0.160 rad (+ = down)** (probe log confirms), the HOLD-long (50-tick) novelty windows and the MAX (300-tick) cap on a novel window. QUIET is inert because the cloud-end rule is on. | 3415-3418 | 1910-1925, 1935-1938 |
| `--stop-gaze-sweep 0.6 0.7` | The gaze never holds: a yaw × pitch grid of 0.1 rad cells (yaw ±0.7 → 14 cells; pitch band → 3 cells; "14 x 3" in the probe log) is counted by where the slewed head command is; on arrival at a target (within 1e-3 rad) the next target is drawn at random among the LEAST-looked-at cells this stop; the head yaw (and pitch) override slews toward it at 0.6 rad/s. The override replaces the head brain's yaw/pitch commands (`HeadAdapter.cpp:133-150`). | 3405-3406 | 1931-1953; 2283-2302; counts reset per stop 2224-2225 |
| `--stop-gaze-sweep-slow 1` | Speed while the window's view is novel = 1.0 × 0.6 rad/s, i.e. **no slow-down**. | 3452-3453 | 2316-2317 |
| `--stop-gaze-residual 1.0` | The novelty test's arrival threshold K: a window's view is "novel on arrival" if the map's `quant_error` at tick 11 of the window exceeds 1.0 × the map's `expected_error`. | 3409-3410 | 2307-2309 |
| `--stop-gaze-learn 0.5` | Then the window counts as novel while `quant_error > 0.5 × its arrival value` (learning progress). | 3401-3402 | 2307-2309 |
| (net effect of the three above in LA1) | With slow = 1 and the quiet rule disabled, the novelty verdict changes **only counters** (`saccades`, `novel_holds`, quiet run), not the head's motion or the stop's length: `set_override_slew` gets 0.6 either way (2317) and `stop_left=0` from quiet is gated off (2321). See open question Q3. | — | 2306-2322 |
| `--stop-gaze-at-thing` | At an ARRIVAL stop (incl. the look stop after an unwind, which sets `stop_is_arrive=true`, 2512) the sweep's pitch band is re-centred on `c = clamp(atan2(0.2, max(0.05, seek_range)+0.15), 0.2, 0.55)` ± 0.12 rad (down) and its yaw centre on `clamp(seek_ego, ±0.5)`. **Sign: `seek_ego` is + = right (`IntentAdapter.cpp:292, 306`) but the head-yaw override is + = left** (cf. 2598, 2643) — see Q1. | 3476-3477 | 2226-2228 |
| `--cloud` | Turns on publication of every ToF cast (stillness flag, odom yaw, odom z, odom x, y, 64 levelled returns, the ray origin, and with `--tof-free-rays` 64 empty-ray ends) on `reality.proprio.tof_points` for the graph's CloudMap. The voxel size argument is optional and **unused**: `g_cloud_voxel` (0.04 when no number follows) is only an on/off switch; the module's `voxel_m` comes from the graph. | 3443-3445 | 1686-1693; payload 2825-2854; publish `IntentAdapter.cpp:202-203` |
| `--map-view cloud` | The place map's 8 view slots carry the stop's CLOUD view (`CloudMap::view`, nearest off-floor return per sector over ±64° / 4 m) instead of the live ToF columns; held through the walk (the last stop cloud's view); the head-yaw slot reads 0. | 3446-3449 | 1966-1970; 2857-2864 |

#### [Host A.5] Skills, unwind, impeded look

| Flag + value | What it does | Parse | Impl |
|---|---|---|---|
| (graph requests) | A skill is requested by the graph on `intent.skill` = [id, request>0.5] (fresh this tick) (`IntentAdapter.cpp:265-268`); id indexes `kSkills` (1450-1456): 0 kick_left, 1 kick_right, 2 roulade, 3 peck, 4 push. | — | 2140-2146, 2255-2258, 2271-2276 |
| `--skill-unwind 0.3 1.5` | After a skill fired at a stop (stander phase), the stop ends and the body backs off at −0.3 m/s for 1.5 s (intent learning off each tick), then a "stop:look" stop starts. | 3550-3551 | 2137; 2502-2516 |
| `--skill-unwind-aim 1.0` | During the unwind, yaw = clamp(−1.0 × thing_ego, ±1) keeps the nose on the remembered thing (`thing_ego` from the outcome loop's `reality.cognitive.outcome_need` position vs odometry, `IntentAdapter.cpp:211-219`). It would also centre the look stop's sweep on the thing (2233-2236), but that branch is **shadowed** by `--stop-gaze-at-thing` (Q2). | 3548-3549 | 2506-2507; 2233-2236 |
| `--look-up-when-impeded 3 -0.3 1.5 0.6` | IMPEDED = while seek steers the reference (`last_steer()==3`, `seek_value>0`) and nothing else runs, the seek range has not closed by 5 cm in 3 s (a range jump of >0.2 m = a new target resets it). Then: back off at −0.3 m/s (the unwind speed) for 0.6 s, start a stop; for the first 1.5 s of that stop the head pitch override is −0.3 rad (UP) and yaw 0 (this override is written after the sweep's in the same tick, so it wins). At that stop's end: CloudMap's `target_tall()>0 && target_small()==0` → WALL: the seek target is forgotten and the reference held on the freest cloud-view sector for 6 s (`--stuck-escape`); else THING: kept. | 3590-3592 | detection 2173-2187; back-off + stop 2517-2528; look 2576-2580; verdict 2479-2495 |

#### [Host A.6] Steering reflexes, twist shaping, chase

| Flag + value | What it does | Parse | Impl |
|---|---|---|---|
| `--heading-reflex 1.0 0.3 1.0` | A host reflex on own yaw, inside the intent adapter: on every tick where a loop's bearing (or an escape hold) set the heading reference, `reflex = clamp(−err/1.0 s − 0.3 × sensed yaw rate, ±1.0 rad/s)` (err = unwrapped heading − reference); share `= clamp(1 − near/1.0, 0, 1)` with near = max ToF proximity left/ahead/right (seek's own sector zeroed under `--seek-gate`); `vyaw = share × reflex + (1−share) × brain's vyaw`. | 3555-3556 | `set_heading_reflex` 1676; `IntentAdapter.cpp:370-391` |
| `--seek-gate` | While the seek loop won the reference last tick, the walker's ToF slot of the target's sector (left/ahead/right by `seek_ego` ±0.3 rad) reads 0 (the thing walked to is not an obstacle); the heading reflex's proximity uses the same zeroing. | 3460-3461 | 1625; `IntentAdapter.cpp:145-151, 385-386` (also 294-295, inert: `--ref-free` not set) |
| `--chase-vx 0.35` | While the seek loop chases or coasts after a mover, no stop, walker driving: forward command raised to ≥ 0.35 m/s (walker range 0.4). | 3429-3430 | 2097-2098 |
| `--yaw-linearize-below 0.15` | When \|forward command\| < 0.15 m/s, the yaw command is treated as a DESIRED yaw rate and mapped through the inverse of a measured open-loop table of the walking policy's yaw response (it does not turn in place below ~1.25 rad/s commanded) — a deadband compensation; above 0.15 m/s the yaw passes through. | 3493-3494 | 2100-2101; table + inverse 51-83 |
| `--intent-range-sense` | `clamp(seek_range/2 m, 0, 1)` (2.0 → 1 when no target) inserted after the fore-aft slot in the intent's sense. | 3495-3496 | 1642; `IntentAdapter.cpp:162-165` |
| `--tof-body 0.5` | The walker's three ToF proximity slots are rebuilt in the BODY frame: every Hit return of the last 0.5 s, carried by odometry into the current body frame, binned by body azimuth from the sensor into left (5.6–22.5°), ahead (±5.6°), right; proximity `1 − r/1 m`. A head turned sideways no longer points "ahead" sideways. The too-close slot stays the sensor's. | 3503-3504 | 2790-2817 |
| `--tof-free-rays` | Each EMPTY zone's ray end at 4 m (levelled frame) is appended to the cloud cast so CloudMap can mark free space a ray passed through (a ball's top "seen"). | 3574-3575 | 2847-2853 |

#### [Host A.7] The train

| Flag + value | What it does | Parse | Impl |
|---|---|---|---|
| `--train 0.2 8 8` | Drives the scene's free body `mov_train0` KINEMATICALLY (pose and velocity written every tick, `DuckBody::place_free_body`, `DuckBody.cpp:244-255`) around the closed track stored in the scene (`<numeric name="train_path" data="0 0 1.525 0.8 1.5708">`: an ellipse centred (0,0), semi-axes 1.525 m and 0.80 m, rotated 90°, perimeter 7.48 m; `scene_playroom_train.xml:6-7`), walked by arc length (`TrackPath` 1343-1373): 0.2 m/s for 8 s, still for 8 s, repeating (cycle 16 s). Start phase `seed%6 × 16/6 s` and start position `seed%6 × perimeter/6` (1760-1761). It does not yield to contact. Body: red box 0.18×0.10×0.10 m + cab, 0.33 kg (`scene_playroom_train.xml:214-217`). Its truth goes only to the log field `"train"` (2922). | 3568-3569 | 1749-1768, 2057-2070 |

#### [Host A.8] Logging-only flags (confirmed: no behavioural effect)

All four are read only inside `if (emit)` (2883-3082) and call `const` accessors that only read state:

| Flag | Adds to the JSONL | Parse | Impl |
|---|---|---|---|
| `--log-cloud-profile` | `"cldp"`: the CloudMap's 36-dim break profile per cast (`CloudMap::profile() const`, `cpp_core/include/ogma/modules/CloudMap.hpp:159`) | 3454-3455 | 2999-3004 |
| `--log-com` | `"com"`: whole-body CoM over the feet, truth (`DuckBody::com_over_feet() const`, `DuckBody.cpp:270-279`, reads `subtree_com`) | 3413-3414 | 2921 |
| `--log-movers 0.5` | `"mvc"/"mva"/"mvw"`: clusters of voxels seen in the last 0.5 s (`CloudMap::cluster_recent() const`, `CloudMap.hpp:204`) + the cloud anchor's world pose | 3593-3594 | 1771, 2942-2950 |
| `--log-cloud-live` | `"cldo"/"cldn"/"cldx"` and `"walking"` in `"cloudv"` | 3572-3573 | 2930-2940, 3034 |

The world-pose latch on a cloud's open edge (2082-2094) feeds only these log records.

## 5.2 Host sensors: IMU, head IMU, odometry, body velocity, ToF


### [Adapters 1.1] IMU (trunk)
- `gravity()` = world (0,0,-1) rotated into the IMU site frame from the `orientation` framequat sensor; unit vector, upright ≈
  (0,0,-1) — `DuckBody.cpp:319-321`, sensor resolved by name `DuckBody.cpp:76`.
- `gyro()` = the `angular-velocity` gyro sensor on the `imu` site, rad/s, trunk frame — `DuckBody.cpp:354-356`, `:77`.
- `accel()` = `imu_accel` accelerometer, m/s², gravity included — `DuckBody.cpp:323-326`, `DuckBody.hpp:138`.
- `imu_quat()` = the raw framequat (w,x,y,z) — `DuckBody.cpp:161-164`.
- The MJCF declares `noise="0.001"` (framequat) and `noise="0.005"` (gyro) — `models/microduck/robot_overlay_playroom.xml:63-64`.
  UNCLEAR whether any noise is applied: nothing in the host adds noise, and (external knowledge, not verified in-repo)
  MuJoCo ≥ 3.1.4 no longer applies the `noise` attribute. Probe evidence that it is effectively exact: at t = 60 s the
  odometry yaw is 1.8272 rad and the simulator's trunk quaternion (`qpos[3:7]` in the probe JSONL line for tick 2999) gives
  1.8268 rad.

### [Adapters 1.2] Head IMU (derived + one sensor)
- `head_gravity()` = world gravity in the frame of the body carrying the `head_imu` site, read from MuJoCo's body `xquat` —
  documented as a transparent shortcut for trunk IMU ∘ FK(neck/head joints) — `DuckBody.cpp:328-338`.
  Measured frame: with head joints at zero the head-frame gravity is (−1, 0, 0) (x axis points DOWN when level), so roll is
  the y component and pitch the z component — `HeadAdapter.cpp:217-225`.
- `head_gyro()` = the `head_gyro` sensor on the `head_imu` site ("the ToF board's IMU"); zeros if the scene lacks it —
  `DuckBody.cpp:98-108`, `models/microduck/robot_overlay_playroom.xml:68`. Probe: "head gyro sensor: present".

### [Adapters 1.3] ODOMETRY — what it integrates (`mj_host/src/Odometry.{hpp,cpp}`)
- A line-for-line port of Pollen's `odometry` crate: **leg-contact (kinematic) odometry**. One sole corner of one foot is the
  anchor (world z = 0 on flat ground, x/y fixed when it became the anchor); the trunk position follows by forward kinematics
  of the foot sites in the trunk frame, rotated by the IMU orientation — `Odometry.hpp:2-17`, `Odometry.cpp:131-135`.
- Anchor switch: when another sole corner (sole half-extents 0.027 × 0.0206 m) drops more than 1 cm below the current
  anchor for 2 consecutive ticks, the anchor moves there at that corner's current world x/y (no jump) —
  `Odometry.hpp:43-45`, `Odometry.cpp:107-127`, `:137-156`.
- Heading = yaw of the IMU quaternion (`atan2` of the quaternion), "no magnetometer, the frame is wherever the robot was
  looking at boot" — `Odometry.cpp:128`, `Odometry.hpp:11-12`.
- Inputs per tick: the two foot sites' poses in the trunk frame (`site_pose_trunk("left_foot"/"right_foot")`, computed
  as R_trunkᵀ(p_site − p_trunk) from MuJoCo's kinematics, i.e. FK of the joint angles) and `body.imu_quat()` —
  `main.cpp:2780-2783`, `DuckBody.cpp:300-317`.
- **Verdict: egocentric / dead-reckoned in construction** (joint angles + body geometry + IMU orientation; no simulator world
  pose is read — `Odometry.hpp:14-17`). It does NOT integrate the gyro or the commanded twist. Caveat for the article: in
  simulation the "IMU yaw" is MuJoCo's exact site orientation, so the heading does not drift the way a real
  gyro-integrated, magnetometer-free yaw would; position drift comes only from foot slip/anchor geometry (probe at 60 s:
  odom (−0.228, −0.745) vs true trunk (−0.336, −0.825), i.e. ≈ 0.13 m drift).
- Called once per tick after the physics step (`main.cpp:2783`); `position()` = (x, y, z) with z = trunk height above the
  anchor (−contact z) — `Odometry.cpp:134`.

### [Adapters 1.4] Body velocity estimate (`vel_body`, the intent brain's "joint positions")
- vx, vy: the odometry position differenced tick-to-tick (×50), rotated into the body frame by the odometry yaw, then an EMA
  with α = 0.1 (τ ≈ 10 ticks = 0.2 s; "contact odometry steps at anchor switches") — `main.cpp:2005-2009`, `:2871-2876`.
- yaw rate: EMA (α = 0.1) of the trunk gyro z — `main.cpp:2878`.
- Units m/s, m/s, rad/s; body frame +x forward, +y left (from the rotation `vy_b = -s*vx_w + c*vy_w`, `main.cpp:2873-2874`).

### [Adapters 1.5] ToF (`mj_host/src/Tof.{hpp,cpp}`)
| Property | Value | Anchor |
|---|---|---|
| Sensor modelled | VL53L8CX 8×8 depth matrix | `Tof.hpp:2` |
| Zones | 8 rows × 8 cols = 64; row 0 = top, col 0 = left | `Tof.hpp:35`, `Tof.cpp:126-139` |
| Field of view | 45° (both axes); zone centres spaced 5.625°, half-zone inset (outermost centres at ±19.69°) | `Tof.hpp:36`, `Tof.cpp:132-137` |
| Max range | 4.0 m (beyond = Empty) | `Tof.hpp:36`, `Tof.cpp:202` |
| Min range (TooClose) | horizontal range < 0.10 m | `Tof.hpp:37`, `Tof.cpp:230-234` |
| Floor class | downward beam whose vertical drop reaches ≥ 85 % of the sensor's height above the floor | `Tof.hpp:37`, `Tof.cpp:166-167`, `:226-229` |
| Classes | Empty / TooClose / Floor / Hit (port of Pollen's `kinematics::tof::Reprojector`) | `Tof.hpp:4-9`, `:17` |
| Ray cast | `mj_ray` from the `tof` site's world pose, against world geometry group 0 only (robot invisible to itself) | `Tof.cpp:169`, `:201` |
| Noise | none added (deterministic ray cast) | `Tof.cpp:194-237` (no noise term) |
| Timing realism (`--tof-real`) | off in LA1 (instantaneous cast) | `Tof.hpp:55-64`, `main.cpp:1680-1683` |
| Mounting | site `tof` on body `jaw_soft`, the distal head body (inside `neck` → `neck_pitch` → `yaw_roll_motion`), so it moves with all four head joints | `models/microduck/robot_overlay_playroom.xml:185-263` |
| Height reference | sensor height above floor = sensor z in the gravity-levelled trunk frame + odometry trunk height | `Tof.cpp:162-166`, called with `p[2]` at `main.cpp:2788` |
| Cast rate | every 4th brain tick (12.5 Hz) | `main.cpp:2787` |

Per zone the cast stores slant range, horizontal range, the return point in the trunk frame (`point`) and in the
**gravity-levelled trunk frame** (`point_level`: origin at the trunk, +z up by measured gravity, x/y turning with the body),
and for an Empty zone `far_level` = the point 4 m along the beam in the same frame — `Tof.hpp:16-31`, `Tof.cpp:203-225`.
`origin_level()` = the sensor position in the levelled frame — `Tof.hpp:47`, `Tof.cpp:165`.

Reductions:
- `column_hit()`: per sensor column the nearest **Hit** horizontal range (m), 4.0 if none — `Tof.cpp:241-247`.
- `summary()` (4 floats): proximity left = cols 0-2, ahead = cols 3-4, right = cols 5-7, each `clamp(1 − r/1 m, 0, 1)`
  (0 = nothing within 1 m), and the TooClose fraction (TooClose zones / 64) — `Tof.cpp:255-263`.
- **`--tof-body 0.5` (on in LA1) replaces the first three summary slots** with body-frame sectors: every Hit return of each
  cast is stored in the odometry frame; returns older than 0.5 s (25 ticks) are dropped; at each cast the remembered points
  are re-expressed in the current body frame relative to the current sensor origin, filtered to |body azimuth| ≤ 22.5°, and
  binned left (az > 5.625°), ahead (|az| ≤ 5.625°), right (az < −5.625°); slot = `clamp(1 − r_min/1 m, 0, 1)` with r = planar
  distance from the sensor. The fourth slot (TooClose share) stays the sensor's — `main.cpp:2790-2817`. Probe: "tof body: the
  walker's left / ahead / right ToF slots by BODY azimuth from the last 0.50 s of returns, carried by the odometry".

---

## 5.3 Intent adapter: every topic the host computes for the intent brain, and what it reads back


Graph `mj_host/configs/a1v2_t11_not_target.json`; constructed at `main.cpp:1617`. The constructor rewrites every
`master_seed`/`seed` param from the run seed (`IntentAdapter.cpp:24-31`), and chooses the **place form** from the map EPM's
declared `proprio_state_dims` on `reality.proprio.place_in`: 13 → `ColumnsGaze` (`IntentAdapter.cpp:52-92`;
`a1v2_t11_not_target.json:168`). Probe: "place vector: 13 dims: x, y, cos, sin, head yaw / 1.4, the 8 ToF column ranges / 4 m
(a view)". Inspector surface on port 7400/7401 (`IntentAdapter.cpp:94`).

Units constants: `kTwistRangeVx = 0.4 m/s`, `kTwistRangeVy = 0.3 m/s`, `kTwistRangeVyaw = 1.0 rad/s` ("the walker's trained
command ranges") — `IntentAdapter.hpp:27-29`. `yaw_range_` = 1.0 in LA1 (`--twist-yaw-range` absent; `IntentAdapter.hpp:390`).
`unit(v)` = `clamp(v, −1, 1)` — `IntentAdapter.cpp:116`.

### [Adapters 2.1] Topics the intent adapter PUBLISHES (all every tick, 50 Hz, before the graph ticks)

| Topic | Dims | Layout and computation | Anchor | Consumer in LA1 graph |
|---|---|---|---|---|
| `reality.proprio.intent` | 3 | [vx/0.4, vy/0.3, vyaw/1.0], each `unit()`; from `vel_body` (§1.4). The level-2 "joint positions" | `IntentAdapter.cpp:138-140` | `twist_bridge` (proprio_input_topic, indices 0,1,2) — config `:31` |
| `reality.proprio.imu` | 6 | trunk gravity (x,y,z), trunk gyro (x,y,z) rad/s, raw | `IntentAdapter.cpp:141` | `motor_epm_intent` imu_topic — config `:83` |
| `reality.proprio.sense` | 18 in LA1 | see §2.2 | `IntentAdapter.cpp:144-176` | `twist_bridge` load_topic, `load_slots: 18` — config `:39` |
| `reality.proprio.place_in` | 13 | see §2.3 | `IntentAdapter.cpp:177-201` | `map_epm` (RBF encoder, dim_min/max) |
| `reality.proprio.tof_points` | 392 | see §2.4 (only when `--cloud` is on: `place.tof_points_valid = cloud_on`) | `IntentAdapter.cpp:202-203`, `main.cpp:2825` | `cloud` (CloudMap input_topic) |
| `reality.proprio.heading` | 1 | the unwrapped odometry yaw `heading_` (rad, CCW +), accumulated from wrapped increments | `IntentAdapter.cpp:121-129`, `:207` | `play` (heading_topic; PlayLoop multiplies by `heading_sign: -1`, config `:223`, `PlayLoop.cpp:252`) |
| `reality.proprio.odom` | 3 | [x, y, heading_]: x = 2·pose[0], y = 2·pose[1] (odometry metres, boot frame), heading_ unwrapped rad | `IntentAdapter.cpp:210` | `seek` and `outcome` pose_topic — config `:394`, `:502` |
| `reality.proprio.vel_ego` | 2 | [vy/0.3, vx/0.4] each `unit()` = [lateral (+left), forward] in command units | `IntentAdapter.cpp:220` | `play` vel_topic (PlayLoop path integral `PlayLoop.cpp:253-258`) |
| `reality.proprio.tof` | 4 | the ToF summary **as passed in** (body-frame slots under `--tof-body`, not seek-gated) | `IntentAdapter.cpp:221` | none in this graph (grep of the config finds no consumer) |
| `reality.proprio.depth_in` | — | not published (only the `Stacked` form) | `IntentAdapter.cpp:183-192` | — |
| `events.reset` | EnvEvent | on rescue hand-off/hand-back etc. (`on_reset`) | `IntentAdapter.cpp:398-406` | MotorEPM reset handling |

The adapter publishes no `reality.cognitive.*` topic; it only reads them (§2.5).

### [Adapters 2.2] `reality.proprio.sense` — slot by slot, as configured in LA1 (18 slots)

Base 16 slots are built at `IntentAdapter.cpp:153-157`; `--intent-fore-sense` inserts one slot at the front
(`:161`), `--intent-range-sense` inserts one slot after it (`:162-165`). Head sense and mover sense are off. The bridge appends
these 18 values after its 3 × [pos, act, delta] triplets (`JointSensorimotorBridge.cpp:347-401`), so the walker MotorEPM's
state is 9 + 18 = 27; "state index" below = 9 + sense index.

| Sense idx | State idx | Content | Units / normalisation | Anchor |
|---|---|---|---|---|
| 0 | 9 | head fore-aft position: measured synergy `fore = −0.5·((q_neck_pitch − home) + (q_head_pitch − home))`, divided by the translate span `0.6 × 1.10 rad = 0.66 rad`; + = head forward | `unit()` | `main.cpp:2075-2079`, `IntentAdapter.cpp:161` |
| 1 | 10 | distance to the seek loop's held target / 2 m; = 1 (i.e. 2 m) when seek holds no target (`seek_present_ && seek_value_ > 0 && seek_range_ > 0` false) | clamp [0,1]; uses the previous tick's `reality.cognitive.seek_value/seek_range` | `IntentAdapter.cpp:162-165`, `:261-264` |
| 2 | 11 | trunk gravity x (fore/aft tilt) | raw unit-vector component | `IntentAdapter.cpp:153` |
| 3 | 12 | trunk gravity y (lateral tilt) | raw | `:153` |
| 4 | 13 | gyro y (pitch rate) × 0.3 | `unit()`, ±3.3 rad/s spans ±1 | `:153` |
| 5 | 14 | gyro x (roll rate) × 0.3 | `unit()` | `:153` |
| 6 | 15 | gyro z (yaw rate) × 0.3 | `unit()` | `:153` |
| 7 | 16 | accel x / 20 | `unit()`, m/s² incl. gravity | `:154` |
| 8 | 17 | accel y / 20 | `unit()` | `:154` |
| 9 | 18 | accel z / 20 | `unit()` (gravity-dominated common mode, magnitude ≈ 0.49 at rest) | `:154` |
| 10 | 19 | sensed vx / 0.4 (again) | `unit()` | `:155` |
| 11 | 20 | sensed vy / 0.3 (again) | `unit()` | `:155` |
| 12 | 21 (= −6) | heading error `(heading_ − heading_ref_)/π`; + = body points LEFT of the reference; NOT wrapped here (clamped) | `unit()` | `:155` |
| 13 | 22 (= −5) | map EPM TLE (`reality.proprio.place` token's `tle`, read after the previous tick) — novelty | `unit()` | `:156`, `:225-230` |
| 14 | 23 (= −4) | ToF proximity LEFT (body frame under `--tof-body`); seek-gated (§2.6) | [0,1] | `:144-151`, `:157` |
| 15 | 24 (= −3) | ToF proximity AHEAD (body frame); seek-gated | [0,1] | same |
| 16 | 25 (= −2) | ToF proximity RIGHT (body frame); seek-gated | [0,1] | same |
| 17 | 26 (= −1) | ToF TooClose fraction (sensor's own, zones < 10 cm / 64); seek-gated only with `--seek-gate-contact` (off) | [0,1] | `:150`, `:157` |

The "grown 26 → 27, inserted at 10": the checkpoint `duck_forebody_s1` was identified with fore-sense only (9 + 17 = 26);
`state_grow_at: 10` (config `:147`) makes MotorEPMv2 insert one zero, unidentified state element at index 10 on restore
(`MotorEPMv2.cpp:3181-3206`) — that is exactly the range slot (sense idx 1). Probe: "MotorEPMv2 motor_epm_intent: the state
grew 26 -> 27, 1 unidentified element(s) inserted at 10".

The walker's prior indices in the config, mapped through this table (for cross-reference with the MotorEPM section):
`state_prior_indices [0, −6, −4, −3, −2, −1, 9]`, targets `[0.75, 0, 0, 0, 0, 0, 0]` (config `:89-98`) = sensed-vx position
(state 0, the bridge's vx "pos") → 0.75 (= 0.3 m/s); heading error → 0; ToF left/ahead/right/too-close → 0; head fore-aft → 0.
`state_prior_target_gate_reach: [10, …]` gates the speed target by state index 10 = the range slot.

### [Adapters 2.3] `reality.proprio.place_in` (13 dims, `ColumnsGaze` form)

Built on the host every tick after the step (`main.cpp:2820-2869`) and published by the adapter (`IntentAdapter.cpp:177-201`):

| Slot | Content | Units/normalisation | Anchor |
|---|---|---|---|
| 0 | odometry x / 2 | metres / 2, unclamped | `main.cpp:2821` |
| 1 | odometry y / 2 | metres / 2, unclamped | `main.cpp:2821` |
| 2 | cos(odometry yaw) | | `main.cpp:2821` |
| 3 | sin(odometry yaw) | | `main.cpp:2821` |
| 4 | head yaw joint from HOME / 1.4 — **overwritten to 0 by `--map-view cloud`** | | `main.cpp:2822`, `:2862` |
| 5–12 | without `--map-view cloud`: the 8 sensor columns' nearest Hit horizontal range / 4 m (col 0 = left). **With `--map-view cloud` (LA1):** `map_view_held`, the CloudMap's `view()` copied on every tick while a STOP cloud is open (`cloud_open() && !cloud_walking()`), held unchanged through the walk; initial value all 1.0 | [0,1] | `main.cpp:2855-2864`, `:1989` |

`CloudMap::view()`: 8 azimuth sectors over ±64° (`view_half_fov` default 64), each = min over the cloud's voxels whose mean
point height ≥ `break_lo` (0.02 m in this config) of planar range / `view_range` (4 m), clamped to [0,1]; 1 = nothing.
Sector index = `(az + 64°)/128° × 8` with az = atan2(y, x) in the cloud's ANCHOR frame (+y left), so **sector 0 is the
rightmost** (−64°…−48°) — `cpp_core/src/ogma/modules/CloudMap.cpp:1188-1205`, defaults `CloudMap.cpp:95-100`,
`CloudMap.hpp:340`; `break_lo` = config `:235`. Probe: "map view: the stop's CLOUD (nearest off-floor return per sector over
+-64 deg / 4 m), held through the walk; head yaw reads 0". (The host's stuck-escape code also treats "sector 0 on the right",
`main.cpp:2466-2474`.)

Consumer: `map_epm` with `encoder_kind: rbf`, `dim_min [−0.6,−0.6,−1,−1,−1,0×8]`, `dim_max [0.6,0.6,1,1,1,1×8]` (so x, y
are expected within ±1.2 m), `process_every_n_ticks: 5` (config `:160-201`, printed above). Probe: projection_dim
auto-derived = 104 for `reality.proprio.place`.

### [Adapters 2.4] `reality.proprio.tof_points` (392 floats) — format and frame

Header `IntentAdapter.hpp:38-51`; filled at `main.cpp:2825-2853`:

| Index | Content |
|---|---|
| 0 | `still` flag: with `--stop-is-still`, 1 whenever the stop phase is `Brain` or `Walker` (after the settle), else `g_z < −0.999 && max|gyro| < 0.15 rad/s` — `main.cpp:2827-2829` |
| 1 | odometry yaw (rad, wrapped IMU yaw) |
| 2 | odometry trunk height p[2] (m) |
| 3, 4 | odometry x, y (m, boot frame) |
| 5 … 196 | 64 × (x, y, z) of each zone's return in the gravity-levelled trunk frame, z shifted by +p[2] so it is height above the floor; only Hit and Floor zones (TooClose only with `--contact-cloud`, off); NaN otherwise |
| 197 … 199 | the sensor's ray origin (x, y, z) in the same frame, z above floor (`tof.origin_level()`) |
| 200 … 391 | `--tof-free-rays` (on): for each **Empty** zone, the point 4 m along its beam (`far_level`) in the same frame, z above floor; NaN for returning zones |

Frame: "gravity-levelled body frame": origin at the trunk, +z up by measured gravity, x/y rotate with the body (the yaw is NOT
removed; CloudMap de-rotates with value 1) — `Tof.hpp:22-26`, consumer `CloudMap.cpp:666-680`, `:481-500`.
Rate: see inconsistency (a) below — the block is rebuilt and republished on every tick, but the zones only change on the
ToF's cast ticks.

### [Adapters 2.5] Topics and module state the intent adapter READS, and what the host does with them

After `instance_->tick()` (`IntentAdapter.cpp:223`):

| Topic | Read as | Host use | Anchor |
|---|---|---|---|
| `action.vx`, `action.vy`, `action.vyaw` | `accel` clamped [−1,1] × (0.4 m/s, 0.3 m/s, 1.0 rad/s) | the twist (`last_twist_`) | `IntentAdapter.cpp:326-331` |
| `action.head_fore` | `fore_target_ = 0.6 × 1.10 × clamp(accel)` rad (+ = head forward) | the "bird's neck" translation (§2.8) | `IntentAdapter.cpp:332-334` |
| `action.neck_pitch/head_pitch/head_yaw/head_roll` | only with `--intent-head` (off) | — | `:335-341` |
| `reality.proprio.place` (RealityToken) | tle, is_novel, winner, just_baked, node/baked counts, quant_error, expected_error, transition, pruned ids | sense slot 13 (next tick); stop-gaze novelty rules and logs in `main.cpp` | `:225-230` |
| `reality.cognitive.thing`, `.thing_kind` | winner/tle/nodes if fresh this tick | logging only | `:231-238` |
| `arbiter.gain.klino`, `.play`, `.vision` | `values[0]`, −1 if absent | which loop's bearing sets the heading reference (§2.6) | `:244-259` |
| `percept.play_bearing` / `percept.seek_bearing` / `percept.avoid_bearing` | [cx (+right), cy (+forward)] | heading reference | `:270-271`, `:284-310` |
| `reality.cognitive.seek_value`, `.seek_range` | need, range (m) | range sense slot; seek gate; arrival detection (`need` → 0 with range < 0.3 m → `seek_arrived`, used by `--stop-on-arrive`) | `:260-264`, `:276-277` |
| `intent.skill` | [id, request]; fresh this tick and request > 0.5 | the host fires Pollen's skill network (0 kick_left, 1 kick_right, 2 roulade, 3 peck, 4 push) | `:265-268`, `main.cpp:1446-1452`, `:2140-2150` |
| `reality.cognitive.outcome` | 5-vector if fresh and samples > 0.5 | logging | `:272-273` |
| `reality.cognitive.outcome_need` | [need, x, y] (odometry frame) | converted to body bearing `thing_ego_` (+ = right) and range: aims the unwind and the look stop's sweep (`--skill-unwind-aim`) | `:211-219`, `main.cpp:2505-2507` |

The host also reads module internals directly (not via the bus), by `dynamic_cast` on the instance's modules
(`IntentAdapter.cpp:475-635`): CloudMap `view()`, `is_open()`, `is_walking_cloud()`, `voxels()`, `target_tall()`,
`target_small()`; BearingSeekLoop `chasing()`, `coasting()`, `chase_vx/vy()`, `chase_gaze_ego()`, `target_x/y()`. And it
writes into the graph through: `forget_seek_target()` (impeded-look verdict "wall", `main.cpp:2484`), `set_ref_hold()` (the
stuck escape / impeded escape, `main.cpp:2474`, `:2490`), `set_learning()` (MotorEPM rates, §2.9), `set_map_learning()`
(map EPM params, §2.9).

## 5.4 The heading reference, the twist path to the walking policy, the head translation, the intent learning gates

### [Adapters 2.6] The heading reference (`IntentAdapter.cpp:117-135`, `:239-310`; bare anchors `:N` here are `IntentAdapter.cpp`)

1. `heading_` = unwrapped odometry yaw (wrapped increments accumulated) — `:121-129`.
2. Default reference: slow running average `heading_ref_ += (1/3000)(heading_ − heading_ref_)` (τ = 3000 ticks = 60 s) —
   `:135`. It is overwritten below whenever a loop wins (in the probe every tick: "loop-heading: 6000 ticks steered by a loop's
   bearing (0 by avoidance)" over 120 s × 50 Hz).
3. Winner by the arbiter's gains (thresholds 0.5): `klino > 0.5` → `percept.avoid_bearing` (code 2); else `vision > 0.5` →
   `percept.seek_bearing` (code 3); else `play > 0.5` → `percept.play_bearing` (code 1); else none. If no gain topic exists,
   play's bearing alone — `:250-259`. The LA1 graph has no avoidance loop (klino never wins; config arbiter comment), so in
   practice **seek wins over play** whenever the arbiter's vision gain is > 0.5.
4. Escape hold (`--stuck-escape 6`): for 6 s after a stuck stop (or an impeded "wall" verdict) the reference is held at
   `heading_ − bearing_rel` where the bearing is the centre of the freest of the cloud view's 8 sectors, ties broken toward
   straight ahead; loops ignored; steer code 4 — `:278-283`, `IntentAdapter.hpp:207`, `main.cpp:2466-2475`, `:2482-2491`.
5. Bearing → reference: `ego = atan2(cx, cy)` (+ = right); `heading_ref_ = heading_ − ego` (a target to the right lowers the
   reference: clockwise, since the odometry yaw is right-handed CCW-positive) — `:300-307`. `--ref-unwrap` off: no continuity
   correction. A winner whose bearing is ~zero releases the reference (`heading_ref_ = heading_`) — `:308`. Free-space gate
   `--ref-free` off. When seek wins, `seek_ego_ = ego` is latched — `:306`.
6. Sign conventions summary: loops' bearings [cx = +right, cy = +forward] (`IntentAdapter.hpp:114-116`;
   `BearingSeekLoop.cpp:57-63`); `seek_ego`, `thing_ego` + = right; odometry yaw / `heading_` + = CCW (left); heading error
   in the sense = `heading_ − heading_ref_`, + = body left of reference; head yaw joint + = left (`main.cpp:2595-2598`).
   PlayLoop's internal frame is a reflection of the odometry's, corrected by `heading_sign: −1` in the config
   (`PlayLoop.cpp:93`, config `:223`).
7. THE SEEK GATE (`--seek-gate`, on): if the seek loop won the previous tick (`last_steer_ == 3`) and seek is present, the
   ToF sense slot of the sector the seek bearing lies in (ego < −0.3 rad → left slot, > 0.3 → right, else ahead) reads 0 —
   so the walker's ToF priors do not push it off the thing it walks to — `:144-151`. The same rule zeroes that sector in the
   heading reflex's proximity — `:385-386`.

### [Adapters 2.7] How the twist reaches Pollen's walking policy

Inside `IntentAdapter::tick` (after the graph tick):
1. Twist from `action.vx/vy/vyaw` (§2.5): ±0.4 m/s, ±0.3 m/s, ±1.0 rad/s — `:326-331`.
2. **Stuck detector** (`--stop-on-stuck 8`): a stall = brain's vx command > 0.75 of range and sensed vx < 0.25 of range;
   `stuck_now` fires once when the stall run exceeds 8 × a running median-like tracker of past stall lengths (init 12.5 ticks,
   EMA 0.05) and ≥ 50 ticks; judged on the brain's own command before any host override — `:342-369`.
3. **Heading reflex** (`--heading-reflex 1.0 0.3 1.0`), only while a loop holds the reference (`last_steer_ != 0`):
   `err = wrap(heading_ − heading_ref_)`; `reflex = clamp(−err/1.0 − 0.3 × vel_body_yawrate, ±1.0)` rad/s;
   share `= clamp(1 − max(prox_left, prox_ahead, prox_right)/1.0, 0, 1)` (seek-gated sector zeroed), i.e. the nearest body-frame
   ToF range in metres within 1 m; `vyaw = share × reflex + (1 − share) × brain_vyaw` — `:370-391`.
4. `--no-backing` off; open-loop override off — `:393-395`.

Then in `main.cpp`:
5. `command.twist = twist` when the driver is the brain — `main.cpp:2095`.
6. `--chase-vx 0.35`: while the seek loop chases or coasts after a mover and no stop is in progress,
   `vx = max(vx, 0.35)` m/s — `main.cpp:2097-2098`.
7. `--yaw-linearize-below 0.15`: if |vx command| < 0.15 m/s, the yaw command is treated as a DESIRED yaw rate and mapped
   through `yaw_calibrated(want, vx)`, the inverse of a measured open-loop table (rows vx ∈ {−0.2, 0, 0.1, 0.2, 0.4}, columns
   command ∈ {0 … 4.0} rad/s, made monotone; inside the standing deadband the command jumps to the first value that moves the
   body); output can be up to ±4.0 — `main.cpp:2099-2101`, table `main.cpp:51-81`.
8. Stop / skill overrides: during any stop phase the twist is zeroed (`main.cpp:2500`); the unwind backs off at −0.3 m/s for
   1.5 s with yaw `clamp(−1.0 × thing_ego, ±1)` (`--skill-unwind 0.3 1.5`, `--skill-unwind-aim 1.0`, `main.cpp:2502-2507`);
   the impeded back-off is −0.3 m/s (`main.cpp:2517-2519`); a rescue keeps the last command but the scaffold drives.
9. The policy observation (61-D): gyro(3), gravity(3), joint pos − HOME (14), joint vel (14), previous raw action (14),
   command [vx, vy, vyaw, neck_pitch, head_pitch, head_yaw, head_roll, 0, 0, body_z, body_roll, body_pitch, 0] —
   `mj_host/src/Observation.hpp:13-27`, `Observation.cpp:77-117`. In LA1 the head command block is `[−fore, −fore, 0, 0]`
   (§2.8), body block zero. No host clipping of the twist beyond steps 1, 3, 6, 7.
10. Policy output → joint targets: `HOME + 0.9 × action`, low-passed with blend 0.7 (legs) / 0.5 (head), the values Pollen's
    `robotd` uses; then the head joints are overwritten by the head brain's targets (§3.4) — `main.cpp:95-97`, `:2765-2774`.

### [Adapters 2.8] The fore-aft head translation (`--intent-head-translate 0.6`, rate default 1.0 rad/s)
`fore_target_` (§2.5) is slewed at ≤ 1.0 rad/s toward the target on the walk and toward 0 during stops; it is SUBTRACTED from
both the neck-pitch and head-pitch joint targets (moving them together leaves the view's attitude unchanged and slides the head
fore-aft) and the policy is told `command.head = [−fore, −fore, 0, 0]` — `main.cpp:2653-2661`, `IntentAdapter.hpp:139-148`.
The sensed counterpart is sense slot 0 (§2.2). Probe: "0.60 of the 1.10 rad range, at most 1.00 rad/s, centred at stops; the
policy is told it". (Argument parse: `main.cpp:3509-3511` — the optional RATE is not given because the next arg begins with `-`.)

### [Adapters 2.9] Learning gates the host applies to the intent brain
- `set_learning(false/true)` zeroes / restores the MotorEPM(v2) rates `model_lr, ctrl_lr, bias_lr, sat_lr, state_prior_lr,
  state_prior_h_lr, state_model_lr` through hot params — `IntentAdapter.cpp:408-436`. Off during: rescues
  (`main.cpp:2018`), every stop (`main.cpp:2207`), unwinds and impeded back-offs (`main.cpp:2508`, `:2520`); back on at the end
  of a stop / hand-back (`main.cpp:2033`, `:2496`).
- `--map-on-stop`: `set_map_learning(false)` on the walk sets the map EPM's `min_insertion_error = 1e9`, `epsilon_b = 0`,
  `epsilon_n = 0`, `stale_prune_enabled = false`; at a stop (after settling) the configured values are restored
  (`IntentAdapter.cpp:438-455`, `main.cpp:1903`, `:2241`, `:2461`). Probe: "map growth: 0 nodes on walks, 10 at stops".

---

## 5.5 Skills, falls, the walker, the scene, the clock, every learning switch

### [Host B] How SKILLS are executed

* **Networks** (Pollen's, named scaffolds; `mj_host/models/microduck/scaffolds/README.md`): `ball_kick_left.onnx`,
  `ball_kick_right.onnx` (0.5 s window), `roulade.onnx` (1.0 s), `alpha_ground_pick.onnx` = "peck" (2.8 s, phase-driven,
  period 4 s); "push" has no network: the walker with vx 0.25 m/s for 1.2 s (`kSkills`, main.cpp:1449-1456). All six
  `.onnx` files in the repo are in `mj_host/models/microduck/scaffolds/` (`alpha_stand`, `alpha_walking`,
  `alpha_ground_pick`, `ball_kick_left`, `ball_kick_right`, `roulade`); not tracked in git, fetched by
  `mj_host/scripts/fetch_scaffolds.sh` (README). A skill's network is loaded on first use (1830).
* **When**: the graph requests by id on `intent.skill`. With `--stop-on-arrive` (and no `--skill-now`), a request on
  the walk is **deferred** (`skill_pending`) and fired at the next stop's hand-back to the stander ("skill:stand",
  2246); a request during the stander phase fires at once (2255-2258); a request during settle / walker-hold / a skill
  window / an unwind is deferred likewise (2271-2276). Probe log: "skills: 2 fired (4 requested by the graph), 1
  unwinds" (requests at ticks 4076, 5700 never fired: no stop could start in the run's last 60 s).
* **Execution** (2742-2754): the skill network drives ALL 14 joints at standing tuning: `ctrl = HOME + 1.0 × action`,
  no low-pass, observation = the same 61-D vector with an all-zero command, except the peck whose twist slots carry
  `[cos 2πφ, sin 2πφ, 0]`, φ = elapsed/4 s (2746-2749). The head brain's targets are NOT applied in this branch. The
  push (2718-2741) runs the walker (0.9 scale, 0.7/0.5 low-pass) with `vx = 0.25`; the unwind-aim yaw applies to it only
  with `--push-reach` or for "approach" (2721), neither on in LA1.
* **Hand-back**: at the window's end the stander is `on_reset()` and resumes from where the body was left (2741, 2754);
  in LA1 `--skill-unwind` then sets `stop_left = 0` (2137), the stop ends on that tick (stander handed back to the walker
  via `end_stop_drive(true)` 2463), the 1.5 s back-off runs, then the "stop:look" stop. The intent brain is frozen
  throughout (stop + unwind); the stander does not tick during the window (the skill branch precedes it).

### [Host C] FALL detection and recovery

* `Recovery` (`mj_host/src/Recovery.hpp:95-180`, `Recovery.cpp:9-77`), updated every tick from projected gravity and
  the gyro only (2015). FALLEN: `g_z > −0.5` (≈60° over) for 0.2 s (Recovery.hpp:98-99); or STUCK: `g_z > −0.966` (≈15°) for 5 s
  (Recovery.hpp:131-135). Hand-back: `g_z < −0.999` (≈2.5°) AND max\|ω\| < 0.15 rad/s held 0.5 s (still
  criterion, Recovery.hpp:115-117), or give-up after 8 s (Recovery.hpp:121).
* While rescued, **`alpha_stand.onnx`** (`kStandScaffold`, main.cpp:91) drives all joints, zero command, scale 1.0, no
  low-pass (2713-2717).
* Hand-off edge (2017-2029): intent `set_learning(false)` + `on_reset()` (publishes `events.reset`,
  `IntentAdapter.cpp:398-406`); twist zeroed; head `set_learning(false)` + `on_reset()`; a stop in progress is aborted
  (map learning off, stander `end_stop_drive(false)`), counted "stop:rescued". Hand-back edge (2030-2037): intent
  `on_reset()` + `set_learning(true)`, the walker's feedback and targets re-seeded from the current pose; head
  `on_reset()` + `set_learning(true)`.
* Both brains keep ticking during the rescue (intent 2080, head 2626) but their commands are not applied (2095, 2628,
  2632). Probe log: "level-2 120 s — 1 rescues, 96% of the run walker-driven; learning frozen 4%" (that "frozen" share
  counts only rescue ticks, 2038-2039, not stops).
* `alpha_walking` is itself described as "walking … and fall recovery in one network" (scaffolds README; main.cpp:92-96),
  but the host's rescue is `alpha_stand`.

### [Host D] The walker: `alpha_walking.onnx`

* File `mj_host/models/microduck/scaffolds/alpha_walking.onnx` (`kWalkScaffold`, main.cpp:96), Pollen's network,
  loaded once (1615). Inference once per brain tick, i.e. **50 Hz** (2766).
* Observation, 61-D (`mj_host/src/Observation.hpp:11-27`, `Observation.cpp:7-48`): trunk gyro (3), projected gravity
  (3), 14 joint positions − HOME, 14 joint velocities, previous raw action (14), command (13): `vx, vy, vyaw`,
  `neck_pitch, head_pitch, head_yaw, head_roll`, body x, y (0), z, roll, pitch, yaw (0).
* Command in LA1: twist = intent brain's `action.vx/vy/vyaw` × ranges (0.4 m/s, 0.3 m/s, 1.0 rad/s;
  `IntentAdapter.hpp:29`, `IntentAdapter.cpp:326-331`), then heading-reflex mix, chase floor, yaw linearization (A.6),
  zeroed at stops, −0.3 m/s in unwinds/back-offs. Head command = 0 except head[0], head[1] = −fore (translate). Body
  z/roll/pitch = 0 (`--body-pitch` off).
* Output: `target = HOME + 0.9 × action`, low-passed per tick with blend 0.7 (legs) / 0.5 (head) (main.cpp:97-98,
  2768-2773); then the head joints are overwritten by the head brain's targets (2774-2775).

### [Host E] The scene

* `mj_host/models/microduck/scene_playroom_train.xml`, GENERATED by `mj_host/tools/playroom_gen.py --seed 1 --half 2.0
  --balls 2 --blocks 2 --chairs 2 --train` (xml:2-3); manifest beside it
  (`scene_playroom_train.manifest.json`). The host echoes it: probe log "scene manifest: seed 1 sha 170c5230fc1f52a7
  objects 15 half 2.00 m" (printed at main.cpp:1731-1746). **The scene is a fixed file; `--seed` does not regenerate it.**
* Room: 4 m × 4 m, walls at x, y = ±2.0 m, 1.0 m tall, 5 cm thick (xml:58-61). 15 manifest objects: 4 walls; a rug
  (extent 0.5 at the centre); table (1.26, −1.36); chair0 (−1.34, −1.55); chair1 (−1.25, 0.40); shelf on the +x wall
  (1.85, −0.80); a wall clock on +y (its hand hinge `clock_hand` spun at 2π/20 rad/s every tick, main.cpp:1374, 2056);
  balls r 0.039 (−0.16, −0.64) and r 0.047 (−1.69, −0.40); blocks half 0.038 (1.41, 0.04) and 0.042 (−1.70, −1.25); the
  train (§A.7). The robot starts at the origin (STAND keyframe), i.e. **inside the train's elliptical track**.
* MuJoCo timestep: no `<option>` in the scene or overlay → default 0.002 s (`mj_host/models/microduck/README.md:36`).
* **God's-eye audit (host → brains)**. Inputs the host hands to brains: IMU projected gravity, gyro, accelerometer
  (sensors on the `imu` site), head-frame gravity (from the head body's simulated orientation `xquat`,
  `DuckBody.cpp:336-338`) and head gyro, joint positions, contact odometry (FK from the feet + IMU quaternion,
  `Odometry.hpp:1-17`), the simulated 8×8 ToF (`mj_ray` against world geometry group 0, `Tof.cpp:85, 117`; levelled by
  measured gravity; floor test by odometry height), the cloud payload (ToF returns + odometry pose), and host-side
  efference (stop phase → stillness). The train's truth, the trunk's world pose, `tilt_deg`, `touching_wall/object`,
  `com_over_feet` and the cloud anchor's world pose go ONLY to the log. **No direct god's-eye quantity enters a brain.**
  Two idealisations to name: (1) the odometry's HEADING is the yaw of the simulated `framequat` sensor
  (`Odometry.cpp:67`, `DuckBody.cpp:161-164`), i.e. drift-free world yaw in simulation, where a real IMU's yaw
  integrates and drifts; the sensor declares `noise="0.001"` (`robot_overlay_playroom.xml:63`) — UNCLEAR whether
  MuJoCo 3.12 applies sensor noise during `mj_step` (I believe it does not; not verified here). (2) The head-frame
  gravity is the exact orientation of the head body (no sensor noise model).

### [Host F] The session clock

| Quantity | Value | Anchor |
|---|---|---|
| Brain tick (all three brains, walker, scaffolds, recovery) | 50 Hz (20 ms) | `mj_host/src/DuckBody.hpp:45` |
| Physics timestep | 0.002 s → 10 substeps per tick (must be integral, gate G3) | README.md:36; `DuckBody.cpp:59-66`, step 146 |
| Walker / stand / skill policy rate | one inference per tick, 50 Hz | 2714, 2730, 2750, 2766 |
| ToF cast | every 4 ticks = 12.5 Hz; cloud payload only on cast ticks' data | 1679, 2787-2819 |
| Intent brain tick | every host tick (also during rescue) | 2080; `IntentAdapter.cpp:223` |
| Head brain tick | every host tick | 2626 |
| Stander tick | only in a stop's Settle (observing, frozen) and Brain phases | 2217, 2760 |
| Body velocity estimate | odometry differenced, EMA 0.1 (≈10 ticks); yaw rate from gyro | 2004-2009, 2871-2880 |
| Run length | default 3.0 s; LA1 600 s = 30 000 ticks; probe 120 s | 3294, 1844 |
| Pacing | unpaced unless `--realtime` (watch mode) | 115-136 |

### [Host G] Every learning switch in `cmd_level2`, per brain

`set_learning` on all three adapters zeroes (and later restores) the MotorEPM/MotorEPMv2 rates `model_lr, ctrl_lr,
bias_lr, sat_lr, state_prior_lr, state_prior_h_lr, state_model_lr` only (`IntentAdapter.cpp:408-436`,
`HeadAdapter.cpp:208-230`, `OgmaBrainAdapter.cpp:260-336`). **No host call freezes the EPMs, CloudMap, loops or
outcome module of the intent graph**, except the map gate (`--map-on-stop`).

**Intent brain (`motor_epm_intent`)** — learns on the walk; frozen:
| Off | On | When |
|---|---|---|
| 2018 | 2033 | rescue hand-off / hand-back |
| 2207 | 2496 | every stop's start / end (timer, arrive, stuck; impeded and look stops via 2522/2511 + the same end) |
| 2508 (each tick) | — (next stop end or hand-back) | the unwind back-off; Q4 |
| 2520 (each tick) | 2496 (the impeded stop's end) | the impeded back-off |
| 2530 / 2550 | | orient reflex (off in LA1) |
Map (`map_epm` insertion/adaptation/prune): off 1903 (start), on 2241 (stop settled), off 2461 (stop end), off 2026
(rescue in a stop).

**Head brain (`motor_epm_head`)** — learns on the walk; frozen: rescue (off 2022 / on 2036, each with `on_reset`);
every stop (off 2209, 2514, 2526; on + `on_reset` 2497).

**Stop brain** — `set_learning(false)` at construction (1869); its `motor_epm_head` frozen for the whole run (1870,
probe log "freeze: motor_epm_head for the rest of the run"); legs LEARN only in a stop's Brain phase (on 2247) and only
near-upright (`set_regime_learning(g_z < −0.90)` each tick, 2758; frozen iff either axis says so,
`OgmaBrainAdapter.cpp:290-293`); off again via `end_stop_drive` (1999) at the lean hand-back (2267), the stop's end
(2463) or a rescue (2027). The probe log's "freeze: motor_epm_legs model_lr 0.0200 -> 0 …" lines are the one-time
announcement of the construction-time freeze (`OgmaBrainAdapter.cpp:318-321`). Its `regime_epm` (an EPM) is not
covered by `set_learning` and learns whenever the stander ticks (module behaviour, not host).

## 5.6 What the three checkpoints contain and restore


Restore path: `--load-brain` → `IntentAdapter::restore_brain_state` on `snap["graph"]`, after filtering
`graph.modules` to `--load-brain-modules` (default `motor_epm_intent`, `MAIN:1500`, `MAIN:1713-1720`, `IA:566`);
`--load-head` → `HeadAdapter::restore_brain_state(snap["graph"])` (`MAIN:1788-1794`, `HA:254-257`); `--stop-load` →
`OgmaBrainAdapter::restore_brain_state(snap["graph"])` (`MAIN:1863-1868`, `OBA:612-613`). Each calls
`OgmaInstance::restore_state`, which calls each module's `restore_state`. Config params are not restored (they come
from the graph JSON); `MotorEPMv2::restore_state` (`MEPM:7013-7245`) restores per leg: `n`, `have_prev`, `steps_seen`,
EMAs, `babble_rng`, `rest_pos`, **A, C, Cphi, Cvel, Bx, Cp (if present), banks (A, Bx, b, samples, tle, err), active_bank,
b, h, hr, ytrace, calm_state/peak, last_mult, prev_x, prev_y**; and module-level counters including
`consolidate_c`, `state_prior_err`, `gate_ema`, `sp_err_long`, `reset_count`, `ticks_since_reset`, `bank_of_winner`.
The pace and target gates are not in the snapshot.

| File | Top-level keys | Modules in `graph.modules` (restored in LA1) | MotorEPMv2 leg contents |
|---|---|---|---|
| `mj_host/checkpoints/duck_forebody_s1.brain.json` (231 KB) | `graph`, `qpos` [21], `qvel` [20] (body state: not restored, "the body at its reset", `la_probe.err:38`) | 14 modules saved (map_epm, object_epm, thing_epm, thing_kind_epm, comp_*, seek, voter_loops, outcome, …); only `motor_epm_intent` restored | 1 leg: n 26, A 26×4, C 4×26, Bx 26×26, b 26, h 4, hr 4 (zero), ytrace 4, steps_seen 31000, tle_ema 0.463; `module` dict of 222 keys (mostly zero/empty diagnostics; consolidate_c 0, explore_mult 1, ticks_since_reset 6358) |
| `mj_host/checkpoints/head3o_gaze_h1_s2_nohold.json` (33 KB) | `graph`, `head_graph` = `configs/head3o_h1_babble.json` | `head_bridge` (empty), `motor_epm_head` | 1 leg: n 21, A 21×3, C 3×21, Bx 21×21, b 21, h = 0, hr = 0, steps_seen 35000, tle_ema 0.013 |
| `mj_host/checkpoints/duck_r19_s2.json` (284 KB) | `graph`, `qpos`, `qvel` | `head_bridge`, `legs_bridge` (empty), `motor_epm_head`, `motor_epm_legs`, `regime_epm` (EPM) | legs: 2 legs, n 27, A 27×5, C 5×27, Bx 27×27, 6 banks (samples 320491 / 1855 / 28595 / 0 / 0 / 0; A, Bx, b only), active_bank 0, h = 0, steps_seen 353941; head: n 24, A 24×4, same banks; module: consolidate_c 0.99999, bank_of_winner [0,1,2], reset_count 1004 |

Contents worth knowing (computed from the JSON; matrices stored column-major):
- Intent C row norms ≈ 2.4–2.8 per motor; h = [0.43, 0.09, −0.93, 0.69]. The largest C entries (checkpoint 26-layout,
  where index 20 = heading error, 22–25 = ToF left/ahead/right/contact): vx row −1.93 on the heading error; vyaw row
  +1.45 on ToF right and −1.37 on its own sensed yaw rate (index 6); head_fore row −1.45 on the heading error. Bx diagonal
  max 0.95, ‖Bx‖_F 3.99.
- Head C: motor 0 (head_pitch) ≈ 1.01 on state 0, motor 1 (head_roll) ≈ 1.00 on state 3, motor 2 (head_yaw) row norm 0.04;
  h = 0. The babble graph `head3o_h1_babble.json` has the same action order and proprio indices as the LA1 head graph, so
  columns align.
- The head checkpoint's source graph had `babble_ticks 30000` and priors [9, 10]; the LA1 graph has babble 0 and adds the
  gaze prior.

The intent checkpoint's `have_prev: true` and its `prev_x`/`prev_y` are restored; no reset event is published between the
restore and the first tick in the code read (`MAIN:1703-1729`), so the first model update pairs the saved run's last command
with the new body's first state. UNCLEAR whether something else invalidates that pairing (one sample; minor).

---

---

# §6 Data-flow edge list

Notation: `producer → topic → consumer(s)`. "host" = the brain's host adapter (published before the graph ticks, read after).
"(t−1)" = the consumer ticks before the producer in the same brain and reads the previous tick's value (§1.4). Anchors for
every edge are in the module subsections cited.

## 6.1 Intent brain (diagram-ready)

**Sensory edges from the host**

| # | producer | topic (dims) | consumer(s) | ref |
|---|---|---|---|---|
| E1 | host | `reality.proprio.intent` (3: sensed vx, vy, yaw rate, normalised) | `twist_bridge` | §5.3, §2.1 |
| E2 | host | `reality.proprio.sense` (18 slots) | `twist_bridge` | §5.3 sense table |
| E3 | host | `reality.proprio.imu` (6) | `motor_epm_intent` (diagnostic only, misread with a picrawler layout) | §2.2 |
| E4 | host | `reality.proprio.place_in` (13: pose + held cloud view) | `map_epm` | §2.3 |
| E5 | host | `reality.proprio.heading` (1: unwrapped odometry yaw) | `play` | §2.4 |
| E6 | host | `reality.proprio.vel_ego` (2) | `play` | §2.4 |
| E7 | host | `reality.proprio.tof_points` (392) | `cloud` | §2.5 |
| E8 | host | `reality.proprio.odom` (3: x m, y m, unwrapped yaw) | `seek`, `outcome` | §2.10, §2.15 |
| E9 | host | `reality.proprio.tof` (4) | **none** | §5.3 |
| E10 | host | `events.reset` | MotorEPMv2 (reset handling, e.g. `MEPM:1809-1811`) | §5.3 |

**Internal edges, in producer tick order**

| # | producer | topic (dims) | consumer(s) |
|---|---|---|---|
| E11 | `twist_bridge` | `reality.motor_limb.twist` (27) | `motor_epm_intent` |
| E12 | `motor_epm_intent` | `action.vx`, `action.vy`, `action.vyaw` | `twist_bridge` (act echo, Direct → used next tick); host (twist) |
| E13 | `motor_epm_intent` | `action.head_fore` | host (head fore-aft slide) |
| E14 | `map_epm` | `reality.proprio.place` (RealityToken) | `play` (winner_id, tle); `cloud` (winner_id = cache key); `comp_play` (tle); host (tle → sense slot; quant/expected error → stop-gaze counters; logs) |
| E15 | `play` | `percept.play_bearing` (3) | host (heading reference when `arbiter.gain.play` wins) |
| E16 | `play` | `reality.cognitive.play_value` (1) | `arbiter` (switches the play channel on; **not used in the precision score**) |
| E17 | `cloud` | `reality.proprio.cloud` (36) | `object_epm` |
| E18 | `cloud` | `percept.cloud_change` (2) | **none** |
| E19 | `cloud` | `reality.proprio.thing` (8; only while a thing is attended) | `thing_epm`, `thing_kind_epm` |
| E20 | `cloud` | `reality.proprio.thing_context` (2; only while attended) | `thing_context_epm` |
| E21 | `cloud` | `percept.thing_bearing` (4) | `seek`, `outcome` |
| E22 | `cloud` | `percept.mover_bearing` (6) | `seek` |
| E23 | `cloud` | `percept.target_tall` (2) | `seek` (chase yield) |
| E24 | `object_epm` | `reality.cognitive.object` | **none** |
| E25 | `thing_epm` | `reality.cognitive.thing` | host log only |
| E26 | `thing_kind_epm` | `reality.cognitive.thing_kind` | `outcome` (winner_id = kind); host log |
| E27 | `thing_context_epm` | `reality.cognitive.thing_context` | `outcome` (winner_id = context, clamped 0..2) |
| E28 | `seek` | `percept.seek_bearing` (4: cx, cy, need, chase flag) | `cloud` (t−1); host (heading reference when `arbiter.gain.vision` wins; seek gate; gaze target) |
| E29 | `seek` | `reality.cognitive.seek_value` (1) | `arbiter` (= "hunger", the seek need); `outcome` (arrival); host (arrival → `--stop-on-arrive`; range-slot validity; impeded look) |
| E30 | `seek` | `reality.cognitive.seek_range` (1, m) | `cloud` (t−1); `comp_seek`; `outcome`; host (range sense slot; arrival; impeded look; arrival gaze centre) |
| E31 | `comp_seek` | `reality.loop.seek` (RealityToken, tle = 1 − competence) | `voter_loops` |
| E32 | `comp_play` | `reality.loop.play` | `voter_loops` |
| E33 | `voter_loops` | `consensus.1` (trust_weights by topic) | `arbiter` |
| E34 | `arbiter` | `arbiter.gain.vision` (0/1) | `comp_seek` (t−1); host (seek bearing → heading reference) |
| E35 | `arbiter` | `arbiter.gain.play` (0/1) | `comp_play` (t−1); host (play bearing → heading reference) |
| E36 | `arbiter` | `arbiter.gain.klino` (0 after tick 0) | host (avoidance slot; no avoidance loop exists) |
| E37 | `arbiter` | `arbiter.gain.planner` (always 0) | **none** |
| E38 | `outcome` | `intent.skill` (2: id, request) | host (fires a Pollen skill network at a stop's hand-back) |
| E39 | `outcome` | `reality.cognitive.outcome` (6) | host log only |
| E40 | `outcome` | `reality.cognitive.outcome_need` (3: need, x, y) | `seek` (t−1; renewal); host (`thing_ego` for unwind aim) |
| E41 | `outcome` | `reality.cognitive.outcome_pull` (1) | `seek` (t−1; confidence of a fresh static target) |

**Host side-channels into the intent graph (not bus topics)** — the host calls module methods directly
(`IA:475-635`): CloudMap `view()`, `is_open()`, `is_walking_cloud()`, `voxels()`, `target_tall()`, `target_small()`;
BearingSeekLoop `chasing()`, `coasting()`, `chase_vx/vy()`, `target_x/y()`, `forget_target()`; and it writes
`set_ref_hold()` (escape), `set_learning()` (MotorEPM rates), `set_map_learning()` (map EPM params) (§5.3–§5.4).

**From the intent brain to the joints (host chain, §5.4)**

`action.vx/vy/vyaw` × (0.4 m/s, 0.3 m/s, 1.0 rad/s) → heading reflex mixes vyaw while a loop holds the reference →
`--chase-vx` floor 0.35 m/s while chasing → `--yaw-linearize-below` remap when |vx| < 0.15 → zeroed at stops / −0.3 m/s in
unwinds and back-offs → command slots of the 61-D observation of `alpha_walking.onnx` → `HOME + 0.9·action`, low-passed →
leg joints (head joints overwritten by the head brain). The heading reference itself is set from the winning loop's bearing:
`heading_ref = heading − atan2(cx, cy)` (`IA:300-307`), and the heading error re-enters the brain as sense slot 12 (state
element 21) on the next tick.

## 6.2 Head brain

| producer | topic | consumer |
|---|---|---|
| host | `reality.proprio.head` (4) | `head_bridge` (indices 1, 3, 2) |
| host | `reality.proprio.head_sense` (12; slot 11 = gaze error) | `head_bridge` |
| host | `reality.proprio.imu` (6) | `motor_epm_head` (diagnostic) |
| `head_bridge` | `reality.motor_limb.head` (21) | `motor_epm_head` |
| `motor_epm_head` | `action.head_pitch`, `action.head_roll`, `action.head_yaw` | `head_bridge` (act echo); host → head joint targets (`--head-joints`), overridden at stops by the sweep |

Cross-brain coupling goes only through the host: the intent brain's seek bearing (`seek_ego`) and the look-around sectors
set the head's gaze error; the intent brain's `action.head_fore` slides neck and head pitch together on top of the head
brain's targets.

## 6.3 Stop brain

| producer | topic | consumer |
|---|---|---|
| host | `reality.proprio.joints` (14) | `legs_bridge` (indices 0–4, 9–13), `head_bridge` (5–8) |
| host | `reality.proprio.sense` (24 = 12 × 2) | `legs_bridge`, `head_bridge` |
| host | `reality.proprio.sense1` (12) | `regime_epm` |
| host | `reality.proprio.imu`, `.lean`, `.lean2`, `.lean4` | `imu` is the MotorEPMv2s' `imu_topic`; `lean`, `lean2`, `lean4` appear in no module param of this graph (config grep), so no consumer |
| `legs_bridge` | `reality.motor_limb.left`, `.right` (27 each) | `motor_epm_legs` |
| `head_bridge` | `reality.motor_limb.head` (24) | `motor_epm_head` |
| `regime_epm` | `reality.proprio.regime` | `motor_epm_legs`, `motor_epm_head` (bank key; read one tick late since it ticks last); host (diagnostics) |
| `motor_epm_legs` | 10 leg `action.*` | `legs_bridge` (echo); host → leg joint targets during the stand |
| `motor_epm_head` | 4 head `action.*` | `head_bridge` (echo); host discards them (`--stop-keep-head`) |

---

# §7 Open questions and inconsistencies

§7.1 is a consolidated, ranked list. §7.2 keeps each drafting section's full list with its anchors. Items marked
**verified** were re-checked against the code while assembling this sheet. The rest come from the section drafts, each
with an anchor.

## 7.1 Consolidated

### A. Likely defects that affect behaviour (check before the article describes the behaviour)

1. **The arrival-stop sweep may be centred on the wrong side (verified in code; one runtime observation).**
   `sweep_yc = clamp(brain.seek_ego(), ±0.5)` (`main.cpp:2228`; also `lost_ego` at `2232` and `thing_ego` at `2236`) is
   used as a head-yaw centre (`main.cpp:2287`, `2296`). `seek_ego` is "+ = right" (`IntentAdapter.hpp:248`, `410`), while
   head yaw is + = left: the `head_yaw` joint axis is `0 0 1` (`mj_host/models/microduck/robot_overlay_playroom.xml:209`),
   and the host negates `seek_ego` everywhere else it feeds the head (`main.cpp:2598`). Probe (seed 5): at the tick-1719
   arrival the thing was 0.37 rad right, and the stop's head yaw ranged −0.28…+1.02 rad (centre ≈ +0.33, left).
2. **The stuck-escape and impeded-wall escape bearing appears mirrored (verified in code).** `CloudMap::view()` puts sector 0
   at az = −64° (`atan2(y, x)` with +y left, so the right side) (`cpp_core/src/ogma/modules/CloudMap.cpp:1195-1201`); the host
   comment agrees ("sector 0 on the right", `main.cpp:2468-2469`). But it converts sector k to a bearing as
   `(−64 + (k+0.5)·16)°`, "+ = right" (`main.cpp:2474`, `2488`), which sends sector 0 to −56°, that is, left. Unless the
   cloud frame's +y is right (it is the levelled trunk frame, `mj_host/src/Tof.hpp:22-26`), the escape heads toward the
   mirror image of the freest sector.
3. **Each ToF cast enters the cloud about 4 times (verified in code).** The sensor casts on `t % 4 == 0`
   (`main.cpp:2786-2788`), but `tof_points` is rebuilt from the last cast on every tick (`main.cpp:2820-2853`), published
   every tick (`IA:202-203`), and added every tick (`CloudMap.cpp:689`). Consequences: voxel hit counts ×4 (observed peaks
   of 4 and 8); the voxel age floor rises by about 3 ticks (this affects the mover-youth test); repeats in a walking cloud are
   re-placed with later poses (smear). Similarly the thing EPMs see each 4-tick descriptor 4 times, so `baking_threshold`
   20 ≈ 5 distinct observations (§2.7). Whether this is intended is UNCLEAR.
4. **`--map-on-stop` does not fully freeze the map.** On walks the bake gate compares against the 1e9 insertion floor, so
   any node reaching 20 visits bakes without the consistency check. Probe: 4 of 7 baked ids were baked on walks. Isolated-node
   removal, edge ageing and health decay also continue (§2.3 [EPM A.8]).
5. **comp_seek likely scores stop-time windows as failures.** The intent brain keeps ticking during stops, and seek held the
   heading on 539 stop ticks in the probe. With the legs standing the range cannot fall, so those 30-tick windows count as
   failures. A re-target inside a window can also count as a failure (§2.11–2.14, Arb open questions 5–6). This is inferred
   from the code; the check counts were not logged.
6. **The intent self-model learns from the brain's own command, not the applied one.** The act echo and the command trace
   are the MotorEPMv2 output (`JSB:333`, `MEPM:5889`). The heading reflex, the chase floor and the yaw remap change what the
   walker actually receives (§2.2).
7. **Restoring `regime_epm` silently replaces config settings with GNG defaults:** `max_nodes` 2000 (config 64),
   `health_death_spares_baked` false (config true), among others (`cpp_core/src/v3/gng.cpp:734-750`). This is inert at
   3 nodes. Its `baking_threshold` is 100, not the schema's 50, because the key is omitted (**verified**: `v3/gng.hpp:89`
   vs `EPM.cpp:108`).
8. **The map's pose inputs saturate beyond ±1.2 m** (x/2, y/2 against `dim_min/max` ±0.6), in a room of half-width 2 m. This
   held on 61 % of probe ticks. Under `--map-view cloud` the view slots are constant on walks and head yaw reads 0, so on walks
   the place varies only through pose (§2.3).
9. **The cloud view can only read ≤ 0.625 or exactly 1.** Points beyond 2.5 m are never accumulated, but `view()` divides by
   4 m (§2.5 [CloudMap open question 6]).
10. **Smaller behavioural mismatches:**
    - The seek gate's ±0.3 rad sector boundaries do not match the ±5.6° body-frame "ahead" sector (§5.3/§5.4).
    - `--yaw-linearize-below` can command up to ±4 rad/s, against a stated trained range of ±1 (§5.4).
    - `thing_context_epm`'s `winner_id` is used directly as a table index, clamped to 0..2 (§2.9).
    - The outcome loop accepts walking-cloud sightings that seek refuses, and "arrival" is any drop of seek's value at short
      range (§2.15).
    - The unwind freezes intent learning with no matching re-enable when it ends (Host Q4).
    - The exploration-noise random generator is restored from the checkpoint, so its draws are seed-independent (§2.2).

### B. Mechanisms present but inert in LA1 (worth stating plainly in an article)

- **Outputs with no consumer:**
  - `object_epm`'s `reality.cognitive.object`: computed and never read.
  - `thing_epm` is read only by the host log; the "pull" its config comment promises is not wired.
  - `percept.cloud_change`; its `revisit_change` slot is always −1 with `walk_cloud` (§2.5).
  - `arbiter.gain.planner`.
  - `reality.proprio.tof` on the intent bus.
  - The stop brain's `lean*` topics.
- **No descending predictor and no neuro scaling.** No `prediction.*` and no `neuro.state` publisher exists, so every EPM
  clusters the raw encoded input. CLAUDE.md §0's "prediction subtracted before the GNG" and "LateralVoter fuses
  1/(tle+ε) of EPMs" do not describe LA1: the only voter fuses LoopCompetence tokens (§2.3 [EPM A.1], §2.13).
- **Mitosis is off everywhere** (`mitosis_gatekeeper` default false). `mitosis_error_threshold` on `map_epm` and
  `regime_epm` is dead config.
- **The arbiter's play and seek values do not enter the score.** PlayLoop's `play_value` and `vision_value_topic` only
  switch channels on in precision mode. The klino and planner channels are dead after tick 0. Observed rule: seek owns the
  heading whenever it holds a target (§2.14).
- **Other dead or no-effect settings:**
  - LoopCompetence `alpha`, inert under the beta estimator.
  - `master_seed` on the EPMs, voter and arbiter.
  - The `--cloud` voxel-size value.
  - The stop-gaze novelty rule (`--stop-gaze-residual/-learn`, HOLD/MAX/QUIET), which only moves counters.
  - `--skill-unwind-aim`'s sweep centring, which is shadowed by `--stop-gaze-at-thing` (§5.1).
- **`motor_epm_intent`'s `imu_topic` is read with a picrawler layout.** It is diagnostic only (`fwd_v` reads ≈ −1)
  (§2.2).

### C. Labels, comments and metrics that disagree with the code

- **Mislabelled tables in the probe log:**
  - The walker authority table's "tilt fwd/side" rows are really head fore-aft and range (`IA:550-551`).
  - The head readback's columns use the wrong motor order (`HA:282-284`, `425`).
  - The stop brain's head sense slots 8–9 are labelled pitch/roll but carry head-frame gravity x/y.
- **"learning frozen 4%"** counts rescue ticks only. Stops, when intent and head learning are also off, were ~24 % of probe
  ticks (`main.cpp:2038-2039`).
- **Stale comments:**
  - seek: "arrival (0.25 m)" and "~60 s"; the code uses 0.15 m and an exponential forget with τ 60 s to a 0.05 floor,
    about 180 s.
  - Bearing layouts: `seek_bearing` slot 2 is the need; `thing_bearing` has 4 values.
  - Mover ages are in ticks, not seconds.
  - The config's "nothing consumes the bearing yet".
  - `IntentAdapter.hpp:62-67` sense layout; `IA:142-143` "12-slot sense".
  - EFEArbiter and PlayLoop headers are Cell-era.
- **The stderr line "insertion, adaptation and pruning off on the walk"** overstates the map freeze (item A4).

### D. UNCLEAR (not settled from code)

- Whether MuJoCo 3.12 applies the MJCF sensor `noise` attributes. If not, the odometry heading is drift-free; the probe shows
  odom yaw within 0.0004 rad of truth at 60 s (§5.2).
- Whether the 4× cast re-add (A3) is intended.
- The `vel_ego` lateral sign versus PlayLoop's reflected frame (§2.4, §5.3).
- Why `tepm` was logged on only 1499 of 6000 probe ticks (§2.7).
- Whether module `restore_state` overwrites the per-seed reseeding beyond `babble_rng` (§5.1).
- The first model update after restore pairs the saved run's last command with the new body's first state; one sample
  (§5.6).

### E. Caveats on the runtime evidence

- The probe log is a single 120 s run at seed 5 (not the preset's seed 1). Counts in it (node counts, steer split
  3624 play / 2376 seek, 4 arrivals, 2 skills fired) are illustrative, not statistics.
- The scene is a fixed generated file (`scene_playroom_train.xml`, generator seed 1). `--seed` does not change the layout;
  it changes the train's phase and start point, reset noise, module seeds and the sweep RNG (§5.1).

## 7.2 Per-section detail (as drafted, with anchors)

### MotorEPMv2 and bridges


1. **Authority table labels are stale (diagnostic only).** `print_authority_table` labels state rows 9 and 10 as
   "tilt fwd" / "tilt side" (`IA:550-551`); with `--intent-fore-sense` and `--intent-range-sense` row 9 is the head
   fore-aft position and row 10 the range (after the restore, at n = 26, row 10 is g_x). The probe log's "tilt fwd" /
   "tilt side" rows (`la_probe.err:31-32`, `89-90`) should be read that way.
2. **Head readback column labels are wrong for this graph.** `HeadAdapter::readback` names three-motor columns
   head_pitch / head_yaw / head_roll (`HA:282-284`), but this graph's order is head_pitch / head_roll / head_yaw; the probe's
   "pos head_yaw" row is head_roll's position and the "head_yaw" column is the head_roll command (`la_probe.err:72-81`).
   Relabelled, the position diagonal is dominant (+0.078, +0.060, +0.064).
3. **The model is trained on the brain's own command, not the applied one.** The act echo and ỹ use the MotorEPMv2's
   command, while the host mixes the yaw with the heading reflex, raises vx during chases, remaps yaw below 0.15 m/s, and
   overrides head yaw/pitch at stops (§3, §4). A(·, vyaw) is therefore identified against a command the body did not
   receive whenever the reflex share is > 0. Whether this biases the identified authority was not measured here.
4. **Stale gates while frozen.** `update_prior_gates` runs only inside the active descent (`MEPM:4550`); during stops
   the step and command use the last gate values. The commands are not applied during stops, so the visible effect is
   at most the first tick after a stop (when `have_prev` is false after `on_reset`, the gates also skip that tick).
5. **Exploration noise is seed-independent after a restore.** The noise is drawn from `babble_rng`, which is restored from
   the checkpoint (`MEPM:5439`, `MEPM:7051-7052`); the per-seed `seed` rewrite in the adapters affects only the init and
   babble seeding (`MEPM:3116`). Every seed restoring the same checkpoint draws the same noise sequence (the sequence is
   consumed one draw per motor per non-warmup tick, independent of state). Code reading only; not checked at runtime.
6. **The stop brain is not frozen during stands.** The probe log's freeze lines (49-62) are the setup freeze; the code
   re-enables `motor_epm_legs` learning at every hand-back (`MAIN:2247`), gated to near-upright (`MAIN:2758`).
7. **"learning frozen 4%" in the probe summary** counts only rescue ticks (`MAIN:2038-2039`), while the intent brain is
   also frozen at every stop (~24 % of the probe's ticks).
8. **Load-slot width depends on the host flags.** `load_slots 18` matches the sense only with exactly fore-sense and
   range-sense on (16 + 2). With a different flag set, slots would be zero-padded or truncated (`JSB:282`, `JSB:397-401`)
   and every negative prior index (heading, ToF) would point at a different element.
9. **The step ignores `state_prior_motors`.** Not relevant in LA1 (the intent sets no mask; the head has no step), but the
   mask documented for the descent is not applied in `MEPM:5161-5194`.
10. **Comment drift:** `IA:142-143` still describes the sense as "12-slot ... two spare"; the HK description in the header
    (`MEPMh:31-50`) describes the module as homeokinetic, while in the intent and head brains the HK controller update is
    off and C changes only through the state-prior descent.

### EPM instances


1. **`object_epm` has no consumer** (C.2), and **`thing_epm` feeds only the host's log** (C.3). Both learn every tick, but neither
   affects behaviour in LA1.
2. **`--map-on-stop` does not fully freeze the map** (A.8):
   - On walks, nodes still bake, with the consistency check bypassed because the gate is 1e9. Probe: 4 of 7 baked ids were baked on walks.
   - Isolated-node removal, edge ageing and health decay continue.
   - The stderr message "insertion, adaptation and pruning off on the walk" overstates it.
3. **Restoring `regime_epm` silently resets settings to GNG defaults**: `max_nodes` 2000 instead of 64,
   `health_death_spares_baked` false instead of true, plus ε_n, α, β and `mitosis_enabled` (`gng.cpp:734-750`). This is inert at
   3 nodes, but the config values do not run.
4. **`regime_epm`'s `baking_threshold` is 100**, not the schema's advertised 50. The config omits it (A.1).
5. **`mitosis_error_threshold` on `map_epm` and `regime_epm` is dead config**, because `mitosis_gatekeeper` is off (A.4).
6. **`master_seed` has no effect** on any of these EPMs (A.2).
7. **The map's pose dims clamp beyond ±1.2 m** from the odometry origin. That happened on 61% of probe ticks (C.1). Two
   places with the same 8-sector view and both coordinates past the clamp then differ only by heading.
8. **Under `--map-view cloud`, the map's view slots are constant on the walk** (the last stop's cloud view) and the head-yaw
   slot is always 0. The "place" therefore varies on walks through odometry pose only (C.1).
9. **Thing EPMs see each descriptor about 4 times** (C.3), so `baking_threshold` 20 corresponds to about 5 distinct observations.
10. **`thing_context_epm`'s winner_id is used as an index, clamped to 0..2** (C.5). This is correct only while its node IDs stay 0-2.
11. **`remove_isolated` kills are not reported** in `pruned_ids`. This affects the host's prune counters and `transition_counts`
    cleanup (`gng.cpp:335-338` vs `EPM.cpp:874-887`).
12. **CLAUDE.md §0's "the prediction is subtracted before the GNG"** does not apply in LA1: no descending predictor is wired.
    **"1/(tle+ε) is what the LateralVoter fuses on"** also does not apply: the only voter (`voter_loops`) reads `reality.loop.*`,
    and no EPM token reaches a LateralVoter in these brains.
13. **UNCLEAR:** `tepm` was logged on only 1499 of 6000 probe ticks, although the EPM publishes a token every tick. The host
    logs only when `rt->tick_id == tick_id_` (`IntentAdapter.cpp:233`). Whether the remaining ticks are skipped by that check
    or by the JSON line's own gating was not traced.

### CloudMap


1. **Each ToF cast is accumulated four times.** The sensor casts only on `t % 4 == 0` (`main.cpp:2787`), but `tof_points` is rebuilt from the same `tof.zones()` and published every tick (`main.cpp:2822-2854`; `IntentAdapter.cpp:202-203`), and `add_cast` runs every tick under walk_cloud (`CloudMap.cpp:689`).
   - **Effect at stops:** hits are inflated ×4. **Observed:** stop-cloud voxel hit counts peak at 4 and 8 (`cloudv` at tick 444: 618 voxels with hits = 4, 163 with hits = 8). New voxels appear only on ticks 1 mod 4 (`cld` at ticks 2001/2005/2013).
   - **Effect on age:** a voxel seen in a single cast already has `last − first` ≈ 3 ticks, which raises the floor of the mover-age measure.
   - **Effect on walking clouds:** the stale body-frame cast is re-placed with each newer odometry pose and yaw, so it smears by up to 3 ticks of motion.
   - Whether this is intended is UNCLEAR. The `things_every` doc says "the sensor casts every 4" (`CloudMap.cpp:166`), and `publish_mover` notes four identical tokens per recompute (`:1155-1158`), but nothing says the cast itself is re-added.
2. **`revisit_change` is never observable with walk_cloud on.** `file_cloud` computes it (`:626-648`), but the walk branch calls `open_cloud` in the same tick (`:685-687`), and that resets `revisit_change_ = -1` (`:477`). `revisit_dist_` and `revisit_overlap_` are not reset.
   - **Observed:** the stop cloud filed at tick 2350 under place 5, a revisit of tick 1566's place 5, logs `revisit_dist: 0.1933` but `revisit: -1.0`.
   - So `percept.cloud_change[1]` is always -1 in LA1. That topic has no consumer anyway.
3. **No consumer found** for `percept.cloud_change`, or for `object_epm`'s output (`reality.cognitive.object`), in the LA1 configs or `mj_host/src`. The 36-dim profile therefore feeds a vocabulary nothing reads. Other code paths outside the grep scope (for example the inspector) were not checked.
4. **Anchor-relative versus body-relative range on walking clouds.**
   - Body-relative: attention, the thing bearing, the mover range test and proximity (`body_rel`).
   - Anchor-relative: the descriptor's range dim (`t.rng`, `:963`), the sort order (`:950`), the gap's range scaling (`:892`), the line tolerance (`:926`), the diag `attended` and `mover` values, and the `max_range` point cut (`:504`).
   - On a walking cloud (up to 1 m from its anchor) these differ.
5. **`on_line` is always below 0.5 for the attended thing.** The line rule refuses clusters with `2·on ≥ ncols` (`:946`), and only small clusters can be attended. So `thing_context[0]` lies in [0, 0.5).
6. **`view()` is capped by `max_range`.** Points beyond 2.5 m from the anchor are never accumulated (`:504`), while `view()` divides by 4 m (`:1202`). A view sector therefore reads either ≤ 0.625 or exactly 1 (empty), never in between. This matters because `--map-view cloud` feeds this view to the map EPM, whose winner in turn keys this cloud's cache, a host-mediated loop (`main.cpp:2857-2864`).
7. **Possible mirror in the host's use of `view()` (host code, outside this module).** `view()` and `profile()` place sector 0 on the **right** (az = −64°, y to the left; `CloudMap.cpp:1199-1201`; `hpp:161-162`). The host's stuck-escape and impeded-wall bearing is `(-64 + (k+0.5)·16)°`, labelled "+ = right" (`main.cpp:2472, 2487`), and `set_ref_hold` treats + as right (`IntentAdapter.hpp:207`). That maps sector 0 (right) to −56°, i.e. left. UNCLEAR whether this is intended; the host-section author should check it.
8. **Unit naming mismatches in comments.** The mover token's age values are in ticks, though the header calls them `age_s`/`oldest_s` (`hpp:191, 332` vs `.cpp:1162`). The thing bearing has 4 values, not the 3 the header says (`hpp:182` vs `.cpp:986`). The config `_comment` says "Nothing consumes the bearing yet: T1 is passive" (`a1v2_t11_not_target.json:242`), which is stale: `seek` and `outcome` consume it.
9. **`break` diag versus mean-height classification.** `break_vox_` classifies a voxel by its first point's height (`:512`), while every other reduction uses the mean height. This is a diagnostic-only inconsistency.
10. **Stop clouds also accept moving ticks under walk_cloud.** With walk_cloud on, casts are added on every tick, including non-still ticks inside a stop cloud (`:689`). The non-walk path skips them (`:696`). The stop cloud is filed only after 25 consecutive moving ticks.

### Loops (play, seek, outcome)


1. **Stale config comment (seek):** says "drops it on arrival (0.25 m) or after ~60 s"; code uses `arrive_m 0.15`, and forgetting is exponential (τ = 3000 ticks = 60 s) to the floor 0.05: from conf 1 that is ≈ 3000·ln 20 ≈ 9000 ticks ≈ 180 s (shorter from a renewal or pull conf < 1); the progress forget usually ends it first.
2. **Stale comments/docs:** BearingSeekLoop header/schema say the output is `[cx, cy, 0]` (BearingSeekLoop.hpp:16-17; .cpp:63) but slot 2 = need, slot 3 = chase flag (.cpp:436-438). SkillOutcomeLoop schema lists 5 outcome fields; code writes 6. SkillOutcomeLoop header says the key is "the thing EPM's winner"; the config wires `thing_kind` × `thing_context`. Outcome `_comment` speaks of "a kick" only; three intents are live. PlayLoop header says "Lives only in the_cell_arbiter*.json" and calls `grew` "baked"; `grew` is the first appearance of a winner id (PlayLoop.cpp:281-283).
3. **`play_value` does not affect arbitration in precision mode** (EFEArbiter.cpp:312-313 vs 396). Play's share is set by (1 − seek_value) × play trust.
4. **Outcome ignores the walking flag**: it fixes thing positions/cells (and can confirm outcomes) from walking-cloud sightings, which seek refuses (BearingSeekLoop.cpp:296-311 vs SkillOutcomeLoop.cpp:158-182). The outcome match is "any attended thing within 0.6 m" — not verified to be the same object.
5. **What "arrival" means to consumers:** outcome (`seek_value` → 0 with range < 0.2) and the host (`< 0.3`) treat any drop of seek's value with a small last range as an arrival — this also happens on a chase loss/yield, `forget_target()`, or a forget/progress-forget at short range, not only the `arrive_m` branch. UNCLEAR how often in practice.
6. **Self-referential uncertainty rule:** a cell with sd above the mean sd of known cells stays "uncertain", so roughly the above-average half of known cells never fully habituate (SkillOutcomeLoop.cpp:203-211). Behavioural effect not measured here.
7. **One-tick lags:** seek reads outcome's need/pull from the previous tick (outcome runs last). CloudMap's `target_tall` is computed from seek's previous-tick bearing (cloud runs before seek). PlayLoop's TLE-EMA is fed the same republished place token on 4 of 5 ticks (`process_every_n_ticks 5`), so the effective EMA weight per fresh EPM step is higher than 0.1.
8. **PlayLoop odometry units:** node positions live in command-units·ticks (integrated clamped `vel_ego`), not metres; only bearings are used, so the scale does not matter while `lookahead` is off. Whether map_epm reuses winner ids after pruning (PlayLoop never removes nodes) is UNCLEAR, not checked.
9. Whether the intent graph (and so these loops) ticks during stops was not checked in this section; the probe's `steer` field is present on all 6000 ticks (play 3624 / seek 2376), and steer is computed in the adapter every tick it runs.

### Arbitration (comp_seek, comp_play, voter, arbiter)


1. **`alpha` is inert.** With `estimator "beta"` the published competence comes only from (a, b) (`LoopCompetence.cpp:163-167`); the EMA `c_` updated with `alpha` (`:153`) is reported in diag as `competence` but never published. The config sets `alpha 0.05` on both comps (config :434, :451) to no behavioural effect.
2. **`vision_value_topic` and `play_value_topic` do not enter the precision score.** They are required only to switch the channels on (`EFEArbiter.cpp:236-238`); the stored values are used only in the `efe` branch (`:345`, `:396`). In particular, PlayLoop's frontier value plays no part in LA1's arbitration; play's score is `(1 − seek_value) × trust_play`.
3. **The klino and planner channels are dead but still published.** Their trust keys are absent from `consensus.1`, so their score is 0; the arbiter starts with klino as incumbent (`EFEArbiter.hpp:299-300`) and hands over on tick 0. If the config ever used `precision_sign −1`, an absent key would read `1 − 0 = 1` (`EFEArbiter.cpp:306-307`), giving the dead klino/planner channels full trust: the wrong-sign control is not clean when keys are missing.
4. **One-tick gain lag** into LoopCompetence (comp_* tick before arbiter): a competence window can start or end one tick after the actual switch. Minor.
5. **Range resets inside a window.** comp_seek compares the range now with the range 30 ticks ago without checking that the target is the same; a renewal or chase re-target with a farther point inside a window counts as a failure, and an arrival drops the target and sets `seek_value` to 0 (the window is then cut by the next tick's gain). UNCLEAR how often this happens; the diag `checks`/`improvements` are not in the probe summary.
6. **Stops.** The intent brain and the comps keep ticking during stops (`main.cpp:2040`); in the probe seek held the heading for 539 ticks inside stop phases. While the legs stand, range cannot fall, so those windows would likely score as failures for comp_seek. This is an inference from the code; the per-module `checks`/`improvements` were not logged.
7. **comp_play's objective is the map EPM's TLE, and the map inserts nodes only at stops in this run** (probe: "inserted 10 at stops / 0 on walks, baked 3 at stops / 4 on walks", `la_probe.err:103-104`; the host's map-freeze mechanism is `IntentAdapter.cpp:438-455`). So while play drives on a walk, "novelty rises" is judged on a map that is not growing. Whether that is the intended reading is for the map section to settle; not a bug in this layer.
8. **`master_seed`** on voter_loops (3) and arbiter (11) is read but not used in any live computation here.
9. **Header comments are Cell-era.** `EFEArbiter.hpp:1-94` describes klino/planner/hunger/scent; on the duck "hunger" is the seek loop's need and "vision" is the seek loop. The config's `_comment` (:491) is the accurate description of what the code does in this configuration.

### Host adapters (item m is settled: the launcher passes `scene_playroom_train.xml`, `launcher.py:264-265`)


a. **`tof_points` republishes a stale cast on 3 of every 4 ticks.** The ToF casts only on `t % 4 == 0` (`main.cpp:2787`), but
   the `tof_points` block is rebuilt outside that branch every tick from `tof.zones()` of the last cast, with the CURRENT
   stillness, yaw and odometry x/y (`main.cpp:2820-2853`), and the adapter publishes it every tick (`IntentAdapter.cpp:202-203`).
   CloudMap adds every token it reads (`CloudMap.cpp:666-684`, `add_cast` `:481-525` has no duplicate check), so each cast is
   added ~4 times; while still this only multiplies voxel hit counts, but in a walking cloud (`walk_cloud: true`, config
   `:252`) the 3 repeats are de-rotated / translated by the pose of later ticks (smear ∝ turn rate × 60 ms). Analysis from
   code, not measured. (The mover publisher already documents "four identical tokens per recompute", `CloudMap.cpp:1157`.)
b. **Stale doc comment**: `IntentAdapter.hpp:62-67` says sense slot 10 = heading deviation, slot 11 = cosine of raw yaw, ToF in
   12–15; in the code slot 11 is the map TLE and, with the LA1 flags, the heading error is sense slot 12 (state 21).
c. **Authority-table row labels are off in LA1**: `print_authority_table` labels state rows 9 and 10 "tilt fwd" / "tilt side"
   (`IntentAdapter.cpp:550-551`), but with `--intent-fore-sense` and `--intent-range-sense` those rows are the fore-aft slot and
   the range slot; trunk gravity x/y are rows 11/12. The probe's "tilt fwd / tilt side" rows are therefore mislabelled.
d. **Head readback column/row labels**: `HeadAdapter::readback` assumes a 3-motor graph is [head_pitch, head_yaw, head_roll]
   (`HeadAdapter.cpp:425`), but this graph's bridge order is [head_pitch, head_roll, head_yaw] (`head3o_h2_gaze_w10.json:15-31`).
   The probe's "pos head_yaw" row is really head roll, and the "head_yaw"/"head_roll" columns are swapped.
e. **Stop-brain head slots 8–9 are labelled pitch/roll but carry head-frame gravity x and y** (`OgmaBrainAdapter.cpp:157`,
   `:173-174`); per `HeadAdapter.cpp:217-225` x points down when the head is level (≈ −1 common mode) and head pitch is the z
   component, which the stop brain does not receive. UNCLEAR whether this was intended; not covered by any stop-brain prior.
f. **Seek-gate sector vs `--tof-body` sector boundaries**: the gate picks the slot with ±0.3 rad (±17°) thresholds
   (`IntentAdapter.cpp:148`, `:386`), while the body-frame "ahead" sector is only ±5.625° (`main.cpp:2806-2812`); a target at
   6–17° off the nose zeroes the "ahead" slot although its returns sit in the left/right slot.
g. **Yaw commands beyond the "trained range"**: `kTwistRangeVyaw = 1.0` is called the walker's trained range
   (`IntentAdapter.hpp:27-29`), but `--yaw-linearize-below` can send commands up to ±4.0 rad/s to the policy at |vx| < 0.15
   (`main.cpp:51-81`, `:2100-2101`); the header itself notes the policy turns in place only above ~1.25 (`IntentAdapter.hpp:168-171`).
h. **Odometry heading is effectively exact in simulation** (§1.3) — the IMU framequat is MuJoCo's site orientation, and sensor
   noise attributes are probably not applied by MuJoCo 3.12 (UNCLEAR, external knowledge). A real gyro-integrated yaw would drift.
i. **Map normalisation range vs arena**: `place_in` slots 0–1 are x/2, y/2 unclamped, while `map_epm` declares
   `dim_min/max ±0.6` (±1.2 m) (config `:169-199`); the probe's scene manifest is "half 2.00 m". How the RBF encoder treats
   out-of-range inputs belongs to the EPM section.
j. **Cloud-view sector order is the reverse of the frame-column order**: under `--map-view cloud` place slots 5–12 run
   right→left (CloudMap sector 0 = −64°), whereas the frame-column form runs left→right (Tof col 0 = left). Harmless for a
   map trained only on the cloud form, but the slot semantics in `place_form_desc()` ("the 8 ToF column ranges / 4 m",
   `IntentAdapter.cpp:460`) no longer describe the content.
k. **`vel_ego` lateral sign**: published as +left (`IntentAdapter.cpp:220`); PlayLoop's frame is a reflection corrected for the
   heading by `heading_sign: −1`, but UNCLEAR whether the lateral component is sign-consistent with it (`PlayLoop.cpp:253-258`).
   Lateral speed is small on this walker, so the effect is likely minor.
l. **One-tick lags by construction**: the map-TLE sense slot, the range slot and the seek gate all use tokens read after the
   previous graph tick (`IntentAdapter.cpp:145`, `:156`, `:163`, `:225-230`, `:261-264`). The bridge's "act" channel likewise
   pairs with the previous tick's action (Direct subscription, `JointSensorimotorBridge.cpp:287-289`). The bridge echoes only
   vx/vy/vyaw actions; the fourth motor (`action.head_fore`) has no action echo in the state (config `:23-49`).
m. **Scene**: the LA1 argument list given in the preamble contains no scene path (it is a positional argument,
   `main.cpp:3617-3618`); the probe's manifest line ("objects 15, half 2.00 m", train track 1.52 × 0.80 m) does not match
   `scene_train_room.xml` (half 1.25, track 0.875 × 0.8, `models/microduck/scene_train_room.xml:2-8`) but does match
   `models/microduck/scene_playroom_train.manifest.json` (seed 1, half 2.0, 15 objects). Most likely LA1 ran on
   `scene_playroom_train.xml`; not confirmed from the command line.

### Host mechanisms


* **Q1 — probable sign inversion of the arrival-stop sweep centre.** `sweep_yc = clamp(seek_ego, ±0.5)` (2228; also
  `lost_ego` 2232, `thing_ego` 2236) uses a + = right bearing as a head-yaw (+ = left) centre; elsewhere the host
  negates it (2598, 2643, 2507). Probe log, arrival at tick 1719: last seek bearing +0.368 rad (right, from `hdg`), the
  following stop's head-yaw commands ranged −0.28 … +1.02 (centre ≈ +0.33, LEFT). One observation, seed 5; worth a
  check before the article describes "looking at the thing".
* **Q2 — `--skill-unwind-aim`'s sweep centring is shadowed.** The look stop after an unwind sets `stop_is_arrive=true`
  (2512), so `--stop-gaze-at-thing`'s branch (2226) wins over the unwind-aim branch (2233). Probe log: "unwind aim: 0
  look stops had the sweep centred on the kicked thing's bearing" with 1 unwind. The unwind's yaw aim (2506) is live.
* **Q3 — the gaze novelty rule is behaviourally inert in LA1** (`--stop-gaze-sweep-slow 1` + cloud-end on): residual /
  learn / HOLD / MAX / QUIET only move counters. The probe log line "4 of 4 stops ended by a quiet round" is the
  `stops_bored` counter, which counts every stop that ended before its 60 s cap (2464), not a quiet round; the events
  are logged as "stop:bored" for the same reason (2466).
* **Q4** — the unwind freezes the intent each tick (2508) with no matching re-enable at its end; it is restored only
  by the following look stop's end (2496) or a rescue hand-back. If the look stop cannot start (last 60 s of the run,
  2509) the walker stays frozen. Minor.
* **Q5** — the JSONL `"learning"` field is `driver == Brain` (2038, 2898): it reads `true` during stops even though
  the intent and head MotorEPMs are frozen there; likewise the summary's "learning frozen 4%".
* **Q6** — the `--cloud [VOXEL_M]` value never reaches the module (only on/off).
* **Q7** — the probe log is seed 5 (train line), not the preset's seed 1.
* UNCLEAR: whether MuJoCo applies the declared sensor `noise` attributes (see §E).

