# Bench protocol v1 — `ogma_benchd` ⇄ the Godot bench dashboard

The contract between the daemon on the Pi (`pi_host/`, C++ over `ServoDriver`) and the
dashboard on the laptop (`godot_host/`, `BenchClient` + `the_bench.tscn`). Design authority:
[`docs/plans-and-designs/picrawler_sim2real_port.md`](../docs/plans-and-designs/picrawler_sim2real_port.md)
SPEC §1.1 (structural boundary), §4 (safety semantics), §5 (dashboard).
**This daemon carries the calibration/validation verb set only.** It has no verb that starts
the brain, and never will — `ogma_host` is a different program with a different socket.

## Transport

| socket | endpoint | pattern | payload |
|---|---|---|---|
| verbs | `tcp://<pi>:5590` | ZMQ **REQ/REP** | one JSON object each way |
| telemetry | `tcp://<pi>:5591` | ZMQ **PUB/SUB**, topic `bench` | **one single-part message per frame: the bytes `bench ` followed by the JSON**, at 10 Hz. Subscribers set `ZMQ_CONFLATE` (newest frame, never a backlog) — which is *why* it is single-part: CONFLATE does not support multi-part messages, and SUB filtering is a prefix match so the in-band topic still filters |

Request: `{"verb": "<name>", ...args}`. Reply: `{"ok": true, ...}` or `{"ok": false, "error": "<why>"}`.
The client MUST use `ZMQ_RCVTIMEO` (≈500 ms) and recreate the REQ socket on timeout — a REQ
socket that missed a reply is stuck by design. The daemon is stateless about clients: it does
not know or care that a viewer came and went.

## Run mode and the deadman (SPEC §4.2)

`ogma_benchd` runs in mode **`bench`** and nothing else — a bench daemon *is* the calibration
channel, and the deadman belongs to the calibration channel only. In `bench`:

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
- **Low battery (SPEC §4.6).** The HAT powers the Pi as well as the servos, so a dying pack
  takes the whole robot down. Below **6.4 V** on A4 the daemon limps everything and refuses
  `servo.set` / `cal.begin` until the pack reads above **6.7 V** again (`low_battery` in
  telemetry). `pi_throttled` echoes `vcgencmd get_throttled` (bit 0 = under-voltage now,
  bit 16 = has occurred since boot) so a servo-transient brownout of the Pi is visible.
- **`limp` is a POSE, because this HAT cannot de-energise a servo (measured 2026-08-29).** Pulse
  count 0, 1 and ARR are ignored; a stopped timer is ignored; the MCU held in reset for 30 s
  leaves the servo powered; the 5 V/3 A DC-DC that feeds the servos also feeds the Pi. So the
  safe action is the saved pose named **`rescue`**: `limp`, the deadman and low battery all
  command it (slewed, every channel). Telemetry carries `rescue_pose` (name or `null` if none
  is saved) and `rescue_active`. `pose.save` a pose called `rescue` first; until then the
  daemon says so at start and in the banner.
- There is no "pause" verb: holding the last pulse is a different wire
  action and is never the safe one (SPEC §4.1).

## Verbs

| verb | args | reply extras | notes |
|---|---|---|---|
| `ping` | — | `t_mono_ms` | feeds the deadman |
| `status` | — | the full telemetry frame + `map` | |
| `limp` | — | `rescue_pose` | command the `rescue` pose on all 12 (see above); ends any widened state |
| `servo.set` | `ch` 0–11, `us` | `clamped_us` | arms `ch`; clamped to its current limits; slewed by the driver |
| `servo.limits` | `ch`, `min_us`, `max_us` | — | sets the OPERATING limits (persisted by `cal.save`) |
| `cal.begin` | `ch` | `until_ms` | widen `ch` to 500–2500 for ≤120 s; refused if another channel is widened |
| `cal.end` | — | — | restore operating limits on the widened channel |
| `cal.map` | `ch`, `physical` (`FL`/`FR`/`RL`/`RR`), `joint` (`hip1`/`hip2`/`knee`), `sign` ±1, `origin_us`, optional `min_us`/`max_us` | — | record one channel's anatomy + calibration; `sim_leg` is derived (below) |
| `cal.save` | optional `path` | `path` | write the map JSON (default `pi_host/calib/servo_map.json` in the Pi checkout) |
| `cal.load` | optional `path` | `path`, `map` | |
| `mode` | `mode` | — | only `bench` is accepted; anything else → `ok:false` (the brain's modes live in `ogma_host`) |
| `pose.set` | `us` (array of 12 µs; `null`/negative = leave that channel) | `clamped_us` | arms every listed channel at once (clamped, slewed); refused while a channel is widened |
| `pose.save` | `name`, `us` (12) | `count` | store a named pose in `pi_host/calib/poses.json` (raw µs per channel, independent of the map) |
| `pose.list` / `pose.get` / `pose.delete` | — / `name` / `name` | `poses` / `us`,`saved_at` / `count` | |

## Telemetry frame (topic `bench`, 10 Hz)

```json
{"seq": 1234, "t_mono_ms": 812345, "uptime_s": 81.2, "mode": "bench", "body": "measured",
 "vbat": 7.63, "adc": [3209, 3367, 3487, 3575, 3137],
 "armed_ch": 0, "cal_ch": -1, "cal_ms_left": 0, "deadman_ms_left": 640,
 "watchdog_trips": 0, "tick_hz": 49.98, "overruns": 0, "bus_errors": 0,
 "low_battery": false, "pi_throttled": "0x0",
 "servos": [{"ch": 0, "target_us": 1500, "current_us": 1500, "armed": true,
             "at_limit_s": 0.0, "min_us": 900, "max_us": 2100}, "... x12"]}
```

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
