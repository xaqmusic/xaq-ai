# Bench protocol v1 — `ogma_benchd` ⇄ the Godot bench dashboard

The contract between the daemon on the Pi (`pi_host/`, C++ over `ServoDriver`) and the
dashboard on the laptop (`godot_host/`, `BenchClient` + `the_bench.tscn`). Design authority:
[`docs/plans-and-designs/picrawler_sim2real_port.md`](../docs/plans-and-designs/picrawler_sim2real_port.md)
SPEC §1.1 (structural boundary), §4 (safety semantics), §5 (dashboard).
**The verb socket carries the calibration/validation verb set only.** It has no verb that
starts the brain, and never will — `ogma_host` is a different program. Since 2026-10-03 the
daemon can also take the brain's servo commands, on a **separate, loopback-only** command
socket, and only in a brain run mode that is set on a **second loopback-only** control
socket (see [Run modes](#run-modes-and-the-brains-command-path-spec-41-42)). Both are off unless
`--cmd-port` / `--ctl-port` are given; without them the daemon is exactly the bench daemon.

## Transport

| socket | endpoint | pattern | payload |
|---|---|---|---|
| verbs | `tcp://<pi>:5590` | ZMQ **REQ/REP** | one JSON object each way |
| telemetry | `tcp://<pi>:5591` | ZMQ **PUB/SUB**, topic `bench` | **one single-part message per frame: the bytes `bench ` followed by the JSON**, at 10 Hz. Subscribers set `ZMQ_CONFLATE` (newest frame, never a backlog) — which is *why* it is single-part: CONFLATE does not support multi-part messages, and SUB filtering is a prefix match so the in-band topic still filters |
| state feed (`--state-pub`) | `tcp://<pi>:5592` | PUB, topic `state` | 50 Hz, for `ogma_host`: `{seq, t, us[12], out[12] (pulse on the line), i_a (servo-branch amps, INA219 ~68 ms average; null if absent), ina_window_ms, armed, mode, stopped, fsr[4], fsr_ok, tof_m, tof_valid, tof_ms}`. Read-only: it carries no verbs |
| brain commands (`--cmd-port`) | **`tcp://127.0.0.1:5594`** | SUB (benchd binds, CONFLATE), topic `cmd` | from `ogma_host --actuate`: `cmd {"seq", "tick", "us": [12 by HAT channel]}`, one per brain tick. **Loopback only, by bind address** |
| control (`--ctl-port`) | **`tcp://127.0.0.1:5593`** | REQ/REP | `mode.get`, `mode.set {mode}`, `stop`, `resume`, `pose.recall {name}`, `recover.pose {name}`, `hat.reset {confirm}`, `status`, `ping`. **Loopback only**; `pi_host/tools/ogma_ctl.py` and `picrawler_dash` speak it. `pose.recall` moves a STOPPED brain-mode robot to a saved pose (staggered, gentle) and leaves it stopped — the reset; refused unless stopped and in a brain mode |

Request: `{"verb": "<name>", ...args}`. Reply: `{"ok": true, ...}` or `{"ok": false, "error": "<why>"}`.
The client MUST use `ZMQ_RCVTIMEO` (≈500 ms) and recreate the REQ socket on timeout — a REQ
socket that missed a reply is stuck by design. The daemon is stateless about clients: it does
not know or care that a viewer came and went.

## Run modes and the brain's command path (SPEC §4.1, §4.2)

Three modes, set **on the robot** — `--mode` at start, or `mode.set` on the loopback control
socket. The calibration channel can read the mode (`mode`) and cannot change it.

| mode | who drives | link loss (dashboard gone) | brain stream goes quiet | calibration commanding verbs |
|---|---|---|---|---|
| **`bench`** (default) | the calibration channel | **deadman → rescue pose** | ignored — brain commands are counted (`brain.frames`) and never applied | allowed |
| **`dev`** | the brain | ignored | **after 200 ms: STOP** (freeze where it is, latched) — keep the evidence | refused |
| **`autonomous`** | the brain | ignored | after 200 ms: hold where it is; **after 5 s: rescue pose** (latched stop) | refused |

- **Entering `dev` or `autonomous` latches STOP.** The brain gets the servos only when someone
  resumes (the dashboards' SPACE, or `ogma_ctl.py resume`), watching. Leaving to `bench` freezes
  the body and starts the deadman's clock.
- **Resume is refused in a brain mode while any channel is unarmed.** An unknown pulse would
  take the brain's first command at full speed, and an unarmed channel reads 0 µs on the state
  feed, so `ogma_host` withholds every tick. Set a pose (e.g. `rescue`) in `bench` first.
- **Silence before the first command is not a loss.** Switching to `dev` before `ogma_host`
  starts does not trip anything; loss is counted only after a command has been applied.
- **Blocked ticks.** A brain command is applied only when nothing else owns the servos: not
  STOPPED, no rescue or pose move running, no widened channel, battery OK, no rail back-off.
  Commands that arrive while blocked are counted (`brain.blocked`), never queued.
- **The envelope and the slew are the driver's.** A brain command goes through the same
  `ServoDriver::command` clamp (each channel's calibrated `min_us`/`max_us`) and the same normal
  slew (`--normal-slew`, 40 µs/tick = 3.67 rad/s) as everything else. Telemetry
  `brain.clamped_mask` says which channels the envelope clamped on the last applied frame, and
  `brain.last_us` is what the brain asked for before the clamp.
- **Optional servo output lag (`--servo-lag-alpha`, default 0 = off).** In a brain mode only,
  the HAT gets `out += α·(current − out)` each tick instead of the slewed pulse itself, so the
  hobby servo moves like the sim's joint (unloaded sim joints fit α 0.22–0.28; the brain's own
  servo model is 0.2). `servos[].current_us` and the state feed stay the slewed command (the
  brain's efference copy); `servos[].out_us` is the pulse on the line; `servo_lag_alpha` echoes
  the setting. STOP and a stream hold freeze at `out_us`, so a stop never coasts.
- **A brain-rate stream on the calibration channel is refused** (SPEC §1.1): more than 30
  `servo.set` + `pose.set` in any second → `ok:false`, counted in `cal_stream_refused`. The
  dashboard throttles slider drags to 20 Hz on one channel; a brain is 600 commands/s.
- **`ogma_host` pauses the brain while STOPPED** (the state feed carries `stopped`): the graph
  does not tick, so it neither learns that its actions do nothing nor resets (SPEC §4.2.2).

### HAT outages and recovery (2026-10-03)

Servo current above ~2.4 A browns out the HAT's own microcontroller, and switching the HAT off is
an outage too. A reset unprograms the servo timers, so writing pulses into them drives the servos
to their stops. benchd therefore treats an outage as a fault:

- **Detection:** the post-reset ADC signature (A4 reads > 9 V), or benchd's own MCU reset after
  persistent bus errors (at most one per 5 s). One outage is one event, however long it lasts; it
  ends once the HAT has answered sanely for 500 ms. Frame: `hat_resets`, and `hat {outage, healthy,
  recovering, recover_resume, saved_channels, outages_60s, recoveries, auto_recover,
  last_reset_age_ms}`. Records: `hat_reset` (with `injected`), `hat_back`, `hat_recover_start`,
  `hat_recovered`, `hat_recover_cancelled`.
- **Response:** remember the pulse each armed servo was last sent, forget the timers, **disarm
  every channel** (nothing is written: the servos go unpowered, not driven), and in a brain mode
  latch STOP (which pauses ogma_host).
- **Recovery:** re-arm **one channel at a time** (the pose stagger — never twelve at once, the
  spike that caused it), then resume the brain with its learning intact. With a recovery pose set
  (control socket `recover.pose {name}`; the dash sets the run's start pose; cleared on return to
  bench), the servos RAMP from their last pulses to that pose at the pose slew. Without one, they
  return to the last pulses. The pose is the default for dash runs: restoring the last pulses
  brought back a hip1 the brain had pinned at its limit, and the robot circled.
  - `autonomous`, the stop came from the reset itself, ≤ 3 outages in 60 s: automatic, once the HAT
    is healthy.
  - Otherwise on `resume` (SPACE): the reply is `ok` with `stopped: true, recovering: true`, and
    the brain resumes when the re-arm lands. While the HAT is still out, resume is refused.
  - `pose.recall` (R) instead returns to a saved pose and discards the remembered pulses.
  - `stop` during a recovery cancels it.
- **Operator workflow:** SPACE (pause) → HAT off → move the robot → HAT on → SPACE.
- **Fault injection:** control-socket `hat.reset {confirm: true}` resets the MCU through its GPIO
  line, the same event a brownout causes, recorded `injected: true`.

### STOP and resume (the dashboards' SPACE)

`stop` **freezes**: every armed channel's target becomes the pulse it is at now, any pose or
rescue move is abandoned, and `servo.set` / `pose.set` / `cal.begin` are refused until `resume`.
It moves nothing — a pose move is itself motion and can catch a leg on whatever caused the stop,
so the rescue pose stays a separate, deliberate act (`limp`, which still works while stopped).
Both verbs work in every mode and every state, on the calibration channel and on the control
channel. `resume` only lifts a stop; it cannot change the mode, so it cannot start a brain that
is not already running. Telemetry: `stopped`, `stop_why`, `stopped_ms`, `stops`. In `bench`, the
deadman still applies while stopped.

⚠ **Holding is the quiet-cook case §4.1 warns about**: a frozen servo straining against a
load keeps drawing current. STOP is for the next few seconds; watch `servos[].at_limit_s`, and
use the rescue pose to leave a stop that is going to last.

### `bench` mode details

A bench daemon *is* the calibration channel, and the deadman belongs to the calibration
channel only. In `bench`:

- **Deadman.** While any servo is armed the client must send *some* verb (`ping` will do) at
  least every **1000 ms**. If it does not, the daemon commands the **rescue pose** once and
  keeps feeding the driver until it lands. Telemetry carries `deadman_ms_left`.
- **One servo at a time — while widened.** `cal.begin` limps every other channel first, and
  while a channel is widened `servo.set` / `pose.set` on any other channel is refused. Outside
  the widened mode channels hold independently, so a pose can be recalled and one joint
  adjusted against it (2026-08-29; before this every `servo.set` limped the others).
- **Envelope.** Every channel has *operating* limits (default **900–2100 µs** until calibration
  narrows them). `cal.begin` is the *audited widened mode*: it opens ONE channel to the full
  500–2500 µs for at most **120 s**, logs entry and exit, and `cal.end` (or the timeout, or any
  deadman trip) restores the operating limits. Calibration exercises the real driver — clamp,
  slew, watchdog — never a bypass (SPEC §4.4).
- **Low battery (SPEC §4.6).** Below **6.4 V** on A4, **sustained for `--vbat-sustain-ms`
  (default 1000 ms; 0 = the old instant trip)**, the daemon limps everything and refuses
  `servo.set` / `cal.begin` until the pack reads above **6.7 V** again (`low_battery` in
  telemetry). A dip that recovers inside the window is recorded as `vbat_dip` (min volts,
  duration) rather than tripping. Sustained since 2026-10-03: the Pi now has its own BEC, so
  servo inrush sags only the HAT rail, and an instant trip on a ~100 ms inrush dip to 6.21 V
  threw a standing robot into rescue mid-move. `pi_throttled` echoes `vcgencmd get_throttled` (bit 0 = under-voltage now,
  bit 16 = has occurred since boot) so a servo-transient brownout of the Pi is visible.
- **`limp` is a POSE, because this HAT cannot de-energise a servo (measured 2026-08-29).** Pulse
  count 0, 1 and ARR are ignored; a stopped timer is ignored; the MCU held in reset for 30 s
  leaves the servo powered; the 5 V/3 A DC-DC that feeds the servos also feeds the Pi. So the
  safe action is the saved pose named **`rescue`**: `limp`, the deadman and low battery all
  command it (slewed, every channel). Telemetry carries `rescue_pose` (name or `null` if none
  is saved) and `rescue_active`. `pose.save` a pose called `rescue` first; until then the
  daemon says so at start and in the banner.
- `stop` (above) holds the last pulse — a different wire action from the rescue pose, and
  not the safe resting state (SPEC §4.1); it is the "do no more harm this instant" key.

## Verbs

| verb | args | reply extras | notes |
|---|---|---|---|
| `ping` | — | `t_mono_ms` | feeds the deadman. ⚠ Read-only verbs (`status`, `pose.get`, `pose.list`, `pose.save`, `pose.delete`, `mark`, `adc.rate`) do NOT (since 2026-10-03; before that every verb did, so an open dashboard polling `status` kept armed servos alive with no controlling client). A client that holds a pose must `ping` |
| `status` | — | the full telemetry frame + `map` | |
| `limp` | — | `rescue_pose` | command the `rescue` pose on all 12 (see above); ends any widened state. Works while stopped and in every mode; in a brain mode it also latches STOP |
| `stop` | — | `stopped`, `mode` | **freeze every servo where it is** and latch (see STOP above). Every mode, every state |
| `resume` | — | `stopped`, `mode` | lift a stop. Refused on low battery, during a rail back-off, and in a brain mode while any channel is unarmed |
| `servo.set` | `ch` 0–11, `us` | `clamped_us` | arms `ch`; clamped to its current limits; slewed by the driver |
| `servo.limits` | `ch`, `min_us`, `max_us` | — | sets the OPERATING limits (persisted by `cal.save`) |
| `tof.stall` | `confirm` (must be `true`) | — | **FAULT INJECTION.** Stops the VL53L0X ranging so the stall-recovery path can be exercised on demand. The part stays addressable and simply stops producing measurements — the observed failure exactly. Needed because the natural rate is ~1 in 600 pose moves, so verifying recovery by waiting costs hours to test a few register writes. Refused without `confirm` |
| `mark` | `text` (1–200 chars) | `text` | Writes a labelled `mark` record into the local record. For sweeps whose arms are **physical** — a resistor swapped, a cap fitted, the surface changed — which leave no other trace, so the segments would have to be reconstructed from wall-clock notes afterwards. `adc_fast_report.py` splits on these |
| `adc.rate` | `ms` — **0 = off (default)**, else 20–60000 | `ms`, `effective_hz`, `channels`, `record_kind` | **Bench instrument.** Samples **A0–A3** from the 50 Hz tick and writes each sample to the local record as `adc_fast` (`{a: [4 counts], us: <cost of the four reads>}`). Exists because `frame()`'s ADC read is 10 Hz, which cannot characterise a tick-rate channel whose only anti-alias filter is outside the robot — foot-FSR bench order step E0. **⚠ the 20 ms floor is the tick period, not the cost of the read**; a faster rate is refused rather than silently rounded. Off by default, so a daemon that never gets this verb behaves exactly as before |
| `cal.begin` | `ch` | `until_ms` | widen `ch` to 500–2500 for ≤120 s; refused if another channel is widened |
| `cal.end` | — | — | restore operating limits on the widened channel |
| `cal.map` | `ch`, `physical` (`FL`/`FR`/`RL`/`RR`), `joint` (`hip1`/`hip2`/`knee`), `sign` ±1, `origin_us`, optional `min_us`/`max_us` | — | record one channel's anatomy + calibration; `sim_leg` is derived (below) |
| `cal.save` | optional `path` | `path` | write the map JSON (default `pi_host/calib/servo_map.json` in the Pi checkout) |
| `cal.load` | optional `path` | `path`, `map` | |
| `mode` | optional `mode` | `mode` | reports the run mode. Setting it here is refused — the mode is set on the robot (`--mode` or the loopback control socket) |
| `pose.set` | `us` (array of 12 µs; `null`/negative = leave that channel) | `us`, `staggered`, `eta_ms` | **staggered and gentle**: channels start one at a time (100 ms apart, shortest travel first) at 600 µs/s — twelve servos starting together on the 5 V/3 A rail the Pi shares browned the Pi out (reproduced 2026-08-29). Telemetry: `pose_move_active`, `pose_queue`. Refused while a channel is widened |
| `pose.save` | `name`, `us` (12) | `count` | store a named pose in `pi_host/calib/poses.json` (raw µs per channel, independent of the map) |
| `pose.list` / `pose.get` / `pose.delete` | — / `name` / `name` | `poses` / `us`,`saved_at` / `count` | |

## Telemetry frame (topic `bench`, 10 Hz)

```json
{"seq": 1234, "t_mono_ms": 812345, "uptime_s": 81.2, "mode": "bench", "body": "measured",
 "stopped": false, "stop_why": null, "stopped_ms": 0, "stops": 0, "cal_stream_refused": 0,
 "brain": null,
 "vbat": 7.63, "adc": [3209, 3367, 3487, 3575, 3137],
 "armed_ch": 0, "cal_ch": -1, "cal_ms_left": 0, "deadman_ms_left": 640,
 "watchdog_trips": 0, "tick_hz": 49.98, "overruns": 0, "bus_errors": 0,
 "low_battery": false, "pi_throttled": "0x0",
 "servos": [{"ch": 0, "target_us": 1500, "current_us": 1500, "armed": true,
             "at_limit_s": 0.0, "min_us": 900, "max_us": 2100}, "... x12"]}
```

`brain` is `null` without `--cmd-port`; with it: `{frames, applied, blocked, bad, seq_gaps,
last_tick, age_ms, have_stream, holding, losses, regains, clamped_mask, last_us[12],
hold_after_ms, rescue_after_ms}`. `deadman_ms_left` is `null` in a brain mode (there is none).

`tick_hz` is the daemon's measured driver-tick rate; the HAT's own servo frame is **49.95 Hz**
(PSC 352 × ARR 4095 at 72 MHz) and is a different clock. `body` is what the daemon was told
via `--body` and is echoed so the dashboard can refuse calibration on `cad`.

## The servo map — one file, both sides (SPEC §5.2)

Sim leg names are anatomically mirrored (port doc, "the leg-naming mirror"): sim `fl` is the
physical **front-right**. The map records **both**, derived by the daemon from `physical`:

| physical | sim_leg |
|---|---|
| FL | `fr` |
| FR | `fl` |
| RL | `rr` |
| RR | `rl` |

```json
{"version": 1, "body": "measured", "saved_at": "2026-08-28T21:40:00Z",
 "servos": [
   {"ch": 0, "physical": "RL", "joint": "knee", "sim_leg": "rr", "sign": 1,
    "origin_us": 1500, "min_us": 900, "max_us": 2100}
 ]}
```

`sign`/`origin_us`/limits are what the sim's `export_servo_calibration` calls `sign`/`origin`/
limits, in µs instead of rad. Conversion the dashboard uses to render the *commanded* pose:
`angle_rad = sign · (us − origin_us) / 636.6` (500–2500 µs ≙ ±π/2), labelled **"commanded —
not measured"** because hobby servos report nothing back.

## The dashboard's MIDI controller and the clouds of touches (2026-11-01)

Two bench instruments for the palpation work (the design conversation of 2026-11-01: palpation first, then the
stop-and-look, then places). Both live in the dashboard and change nothing on the robot's side of the protocol.

**MIDI → servos** (`godot_host/project/scripts/midi_servo_map.gd`). A controller's faders and knobs on the twelve
servo channels, P0..P11, the dashboard's rows. The map is its own local file, not a brain config:
`godot_host/project/local/midi_map.json` (git-ignored, written by LEARN) over `godot_host/project/midi_map.example.json`
(the AKAI MIDI Mix defaults: the eight faders → P0..P7, the third knob row of strips 1–4 → P8..P11, SOLO → STOP). Each
binding is `{"ch": n, "invert": bool}`; a bare integer reads as not inverted. The per-row LIMP buttons are gone (this HAT
cannot limp a servo; the top bar's RESCUE POSE remains the deliberate act).

- **LEARN**: a row's `LEARN` button, then touch the fader or knob that should drive it; the binding shows beside
  the button (`learn…` while waiting) and the map file is rewritten at once. `learn STOP button` binds a controller
  button to STOP (STOP only, never resume: a panic button).
- **The servo map loads on connect** (operator, 2026-11-01), so the rows are named and their limits set before a
  slider or a fader moves; LOAD MAP remains for a re-read.
- **No ARM step** (operator, 2026-11-01): a slider IS the servo's command, dragged by the mouse or driven by a MIDI
  control — the HAT holds a servo's last pulse anyway. A slider's value is sent when connected, in `bench` mode, not
  stopped, on a `measured` body; the daemon refuses the rest regardless (SPEC §1.1), so a controller can never fight
  a brain run. On the first fresh telemetry after a connect (and after a rescue) every slider takes its servo's
  current pulse, so the first drag starts from where the servo is, not from 1500 µs.
- **inv** (per row): flips that servo's control for a joint that runs the other way from the knob (hip1 here); it is
  stored with the binding in the map, survives a re-LEARN of that servo, and the control re-picks up in its new sense.
- **Pickup**: a MIDI control takes hold of its servo only once it has crossed, or come within 4 % of, the servo's
  current pulse (telemetry's `current_us`), so a controller whose faders sit anywhere never yanks twelve servos.
- **Rate**: the daemon admits `CAL_STREAM_MAX` = 30 `servo.set`/s in all, so the dashboard's sends go through a token
  bucket (24/s, burst 4): one dragged slider sends as before; twelve moving faders share the budget in turn.

**The scene** (2026-11-01). The arena the body script builds is hidden on the bench; the only geometry is two
translucent grids (`bench_grids.gd`, lines every 2 cm, heavier every 10 cm): **blue, translucent** at the chassis bottom, yaw-levelled
with the twin, and a **translucent brown plane** where the belly ToF says the real floor is — the chassis plane dropped by the clearance
(`tof.m`, raw − mount offset), with a yellow ring where the boom's beam lands (70 mm aft) and the beam drawn to it. The plane
hides when the ToF is not reading rather than freezing at a stale height.

**The twin leans as the robot leans** (2026-11-01). The daemon's fused up vector, low-pass filtered (τ 0.15 s; the
10 Hz telemetry would otherwise step the body every frame), rotates the whole FK-written twin about its chassis origin
(`calib_tilt` on the body; identity changes nothing in the sim). The servo angles the twin is posed from are filtered
the same way (τ 0.12 s). The voxels draw in the LEVEL frame at the chassis origin, so the body tilts around them and
they stay where they were measured; the blue belly grid rides the chassis, the brown floor plane stays level through
the point where the boom's beam meets the floor, and the body casts its shadow on it.

**MARK** (the field and button in the MIDI row, or `[M]`): sends the daemon's `mark` verb with the label, a running count
and the twin's touch count, so a bench session's conditions label themselves in `benchd_*.jsonl`.

**The clouds of touches** (`godot_host/project/scripts/touch_cloud.gd`, `cpp_core` `TouchMemory`). Every rising
foot-contact edge is one claim about where a surface is: the toe's commanded-FK position at that moment, carrying the
kinematic error the sag study measured (~32 mm standing). A voxel is believable in proportion to the agreement it
collects (hits, feet, time), so each keeps a hit count, the feet that touched it, the load and its age. The dashboard
draws two clouds in the twin's chassis frame and `[X]` hides them:

- **purple — the twin's**: at each FSR onset (≥ 200 counts), the twin's own FK toe (lower-leg node × construction toe
  offset), measured as the module measures it: height along the daemon's fused up vector (`imu.up_fused`), not the
  chassis axis, and relative to the support plane of the feet already planted — so a tap that rocks or lifts the
  chassis does not move the voxel (operator, 2026-11-01: a box's voxel had landed below the box). The first telemetry
  frame is a baseline, not four touches (a standing robot has four loaded feet); only
  changes after it count. `clear touches` or `[C]` empties it (the brain's cloud is the module's own state and stays). No robot-side change; it exists whenever the dashboard does.
- **lavender — the brain's**: `TouchMemory`'s diag, read from ogma_host's control port (`list_modules` once, then
  `module_diag` at ~3 Hz on the same worker thread as `host_sensors`), present when the running config carries the
  module (`_cs_pf_sl0` does). Its heights are relative to its support plane; the diag's `plane_y` places them. Since
  2026-11-02 its toe height is `reality.proprio.toe_y` — the toe TIP along the fused gravity with the floor calibration
  applied in ogma_host (`--toe-y [--toe-calib pi_host/calib/toe_height_calib.json]`, which the launcher passes from
  `body_env OGMA_PICRAWLER_TOE_Y=1`). The gait's `feet_y_gravity_cmd_imu` is NOT a toe height: it is the sim-parity
  foot (shank centre minus half a shank), up to 43 mm below the toe tip at the palpation knee, and the brain's cloud
  sat that much low until the channel was split. The same host publishes nothing new without the flag.

Both apply the per-leg floor calibration (`pi_host/calib/toe_height_calib.json`, fitted by `toe_floor_calib.py` from a
`palpate.py --cond floor --pose toes_up` pass over all four legs): a constant, a knee term and a hip1 term per leg that put
the floor at one plane, anchored where the rear pair found it below the body origin (−29 mm, 2026-11-02); the dashboard's
belly plane follows that anchor. The twin applies it in GDScript on its own FK; the brain gets it inside `toe_y` from the
host, so the two are the same correction on the same toe through two FKs.

**The encounter EPM** (`EncounterFeatures` → `EPM` touch/encounter → `BeliefFilter`, in `_cs_pf_sl0`): one ten-feature
token per contact onset on `sense.touch_encounter` (height over the floor, forward/lateral position, sole peak load,
the bus current's rise before and after onset, belly rise, lean rise, sonar range or −1, the vertical fraction of the
pre-onset travel), a RealityToken per touch on `reality.touch.encounter`. Offline, `cpp_core/bench/encounter_replay`
rebuilds the same inputs from a benchd record (adc_fast `pulse` + telemetry + marks) and writes one JSON line per
encounter with the features, the winner, the TLE and the belief; the ledger's 2026-11-02 entry is its first reading. The two clouds are the same idea through two FKs on one body: where they disagree is the
servo sag made visible. The manual
toe test is: the robot in front of a wall or a box, move one leg (slider or fader) until the toe touches, read the voxel.
In the sim the same purple cloud is always drawn (body-locked, carried by the body's own odometry, so the footprints
slide as it drifts), with the brain's in lavender; `touch_vox` / `touch_n` / `touch_brain` are in the body's JSONL.

## The forward 8x8 time-of-flight field (2026-11-04)

`sense.tof8`, ProprioToken float32[64]: the slant range per zone in metres, row-major, row 0 = the TOP of the field,
column 0 = the LEFT as the robot sees it, −1 = no return; 15 Hz; a leg in the beam is a return at the leg. The part is
a VL53L5CX / VL53L8CX class 8×8 (45° square field, zone pitch 5.6°, 4 m). The sim publishes it from `scripts/tof8.gd`
(`OGMA_PICRAWLER_TOF8=1`, mount `OGMA_PICRAWLER_TOF8_HEIGHT_MM` above the chassis bottom on the front face, pitched
down `OGMA_PICRAWLER_TOF8_PITCH_DEG`; `[7]`/`[8]`/`[9]`/`[0]` move it live, `[Y]` draws the 64 rays coloured by the
body's own classification: floor where a beam meets the ground the belly clearance and gravity estimate predict,
a thing otherwise, a leg when the body's own leg is the return) so the mount can be chosen before it is drilled;
the body log carries `tof8` (cm) and its summary every row. The field's returns within `OGMA_PICRAWLER_TOF8_CLOUD_M` (default 4 m,
the sensor's reach) also feed a body-locked cloud of their own (3 cm voxels, green where the body read floor, yellow where it read a thing), beside the purple
touch cloud; **every cloud's voxel lives 30 s** (operator, 2026-11-04: the odometry drifts, so an older claim is about
a place the body can no longer locate) — `TouchMemory max_age_ticks` 1500 at 50 Hz, the bench twin 300 at its 10 Hz,
0 = forever. The robot's host publishes the same topic from the
part when it is fitted (not yet: the driver is the next piece). **What reads it** (2026-11-05): `Tof8Range` reduces
the field to the nearest FEATURE ahead on `sense.range_tof` as ProprioToken [distance_m, valid], the sonar's own
contract — the floor is a plane fitted to the field's bottom rows (no IMU; the robot's reading is the sim's), a return
2 cm above it is a thing, 2 cm below it a drop, either a feature; no feature = [max_range, 0]; a second token
`reality.proprio.tof8_cols` [8] is the nearest feature per column. `StuckSense` reads `sense.range_tof` on its
closing-rate channel in the terrain configs, so the stall sense works through soft, dark and distant things.

## Local record (SPEC §3)

`pi_host/log/benchd_<YYYYMMDD_HHMMSS>.jsonl` on the Pi: every verb (with reply) and every
telemetry frame, `CLOCK_MONOTONIC` timestamps. The wifi stream is the view; this file is the
evidence.
