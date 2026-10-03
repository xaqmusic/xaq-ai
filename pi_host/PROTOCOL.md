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
| state feed (`--state-pub`) | `tcp://<pi>:5592` | PUB, topic `state` | 50 Hz, for `ogma_host`: `{seq, t, us[12], out[12] (pulse on the line), armed, mode, stopped, fsr[4], fsr_ok, tof_m, tof_valid, tof_ms}`. Read-only: it carries no verbs |
| brain commands (`--cmd-port`) | **`tcp://127.0.0.1:5594`** | SUB (benchd binds, CONFLATE), topic `cmd` | from `ogma_host --actuate`: `cmd {"seq", "tick", "us": [12 by HAT channel]}`, one per brain tick. **Loopback only, by bind address** |
| control (`--ctl-port`) | **`tcp://127.0.0.1:5593`** | REQ/REP | `mode.get`, `mode.set {mode}`, `stop`, `resume`, `status`, `ping`. **Loopback only**; `pi_host/tools/ogma_ctl.py` speaks it |

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

## Local record (SPEC §3)

`pi_host/log/benchd_<YYYYMMDD_HHMMSS>.jsonl` on the Pi: every verb (with reply) and every
telemetry frame, `CLOCK_MONOTONIC` timestamps. The wifi stream is the view; this file is the
evidence.
