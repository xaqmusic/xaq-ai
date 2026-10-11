# pi_host — the PiCrawler hardware host

The Raspberry Pi 5 side of the sim2real port
([design](../docs/plans-and-designs/picrawler_sim2real_port.md), Phase 2 / Phase 4 / SPEC).
Builds **natively on the Pi** (`cmake -S pi_host -B pi_host/build && cmake --build pi_host/build -j2`);
no Godot, and — at this layer — no `ogma_core` either, so the driver rebuilds in seconds.

| piece | what it is |
|---|---|
| `I2cBus` / `LinuxI2cBus` | the seam to `/dev/i2c-1`; the protocol is tested against a fake bus |
| `RobotHat` | SunFounder Robot HAT V4 **wire protocol** (from `robot_hat` 2.5.5 — the protocol, not the library) |
| `ServoDriver` | the safety envelope below the brain: clamp · slew · watchdog → pulse 0 · time-at-limit |
| `hat_tool` | bench CLI over the driver (never a bypass): `vbat` · `adc` · `limp` · `pulse` · `sweep` |
| `test_hw` | byte-level protocol tests + envelope tests, no hardware needed |
| `tools/foot_cal_sweep.py` | the robot probes its own foot sensors with its own weight — knee stepped up and back down on each `<foot>_down` pose. **Counts, not grams**; it answers the cross-foot and hysteresis questions without a length constant. ⚠ runs a deadman keepalive |
| `tools/adc_fast_report.py` | reads the `adc_fast` records `adc.rate` writes and reports level, noise and **wander** per channel. Foot-FSR bench steps E2/E2b. ⚠ reads the record, never the bus |
| **`ogma_benchd`** | the bench daemon — [`PROTOCOL.md`](PROTOCOL.md): ZMQ REP verbs (`:5590`) + PUB telemetry (`:5591`) over the driver; 50 Hz tick thread, 10 Hz telemetry, local JSONL record in `log/`, servo map in `calib/servo_map.json`. **Bench mode only, deadman on the calibration channel only, one servo at a time, `cal.begin` is the audited widened envelope.** No verb starts a brain |

**Wire facts** (bench-verified 2026-08-28): MCU `0x14`; every register write is `[reg, hi, lo]`;
servo timer = `channel/4`, `PSC+t = 351`, `ARR+t = 4095` ⇒ **49.95 Hz** frames (not 50.00);
pulse count `trunc(µs/20000·4095)`, `0` = limp; ADC select `(7−ch)|0x10` then two 1-byte reads;
`Vbat = A4·3.3/4095·3`.

**Channel → anatomy map** (grows with calibration; sim names are mirrored — see the port doc):

| channel | physical | first seen |
|---|---|---|
| P0 | rear-left knee | 2026-08-28, `hat_servo_smoke.py 0` |

Robot on a stand for any servo verb.

**The daemon is a systemd service** (`/etc/systemd/system/ogma-benchd.service`, installed
2026-08-29): starts on boot, restarts 2 s after any exit, `WorkingDirectory=~/xaq-ai` so the
relative `pi_host/log` and `pi_host/calib` paths hold, stdout/stderr appended to
`pi_host/log/benchd.stdout`. It auto-loads `pi_host/calib/servo_map.json` at start.
```sh
systemctl status ogma-benchd            # is it up, what did it print
sudo systemctl restart ogma-benchd      # after rebuilding pi_host/build/ogma_benchd
sudo systemctl stop ogma-benchd         # before hat_tool — both open /dev/i2c-1
```
I²C is retried 3× per transaction in `LinuxI2cBus`; a NACK that survives the retries is
counted (`bus_errors` in telemetry, logged every 50th) and never fatal — the first daemon died
on one such error mid-calibration and took the session with it.

## Running a brain on the robot (2026-10-03)

**On battery only.** On the bench supply the Pi reset the moment the brain took the servos.

benchd's service starts it with the brain command path (`--state-pub 5592 --cmd-port 5594
--ctl-port 5593 --servo-lag-alpha 0.2`). Nothing changes in `bench` mode, which it always starts in.
The command and control sockets are bound to 127.0.0.1, so only a process on the Pi can drive the
servos or set the run mode (port doc SPEC §1.1, amended 2026-10-03).

**From the robot's console:** `picrawler-dash`, then **C** — pick a config (robot-faithful ones
first; P-e·h0 is the validated one), confirm, and after a **10 s countdown** (any key aborts,
nothing is sent before it ends) the robot stands and the brain gets the servos in `autonomous`
mode.

| key | does |
|---|---|
| SPACE | STOP (freeze every servo where it is, brain paused) / resume |
| R | stop, return to the start pose, stay stopped (brain paused, not reset) |
| E | end the run: brain off, `bench`, rescue pose |
| Ctrl-C | ends a live run before the dash exits |

**HAT resets** (servo current above ~2.4 A browns out the HAT's MCU) are recovered automatically:
- benchd disarms all servos and pauses the brain;
- once the HAT answers again, it re-arms one servo at a time and ramps to the start pose;
- the brain resumes. Expect about 3 s, with a ⚠ warning on the dash; three in 60 s and it waits.

To move the robot by hand mid-run: **SPACE → HAT off → move it → HAT on → SPACE**.

Every run leaves `pi_host/log/dashrun_<stamp>.events` (each step) and `dashrun_<stamp>_<cfg>.log`
(ogma_host, with input dumps). benchd's record has `hat_reset` / `hat_recovered` / `stop` /
`resume`.

- `pi_host/tools/ogma_ctl.py` speaks the control socket by hand.
- `pi_host/tools/brainrun/arm.sh` is the scripted harness behind the 2026-10-03 ledger numbers.

**Bench-verified 2026-08-28** (pyzmq client from the laptop): telemetry under `ZMQ_CONFLATE`
(which is why frames are single-part), `servo.set` clamp + slew, one-at-a-time, deadman trip
after 1 s of silence, widen/restore audit, `cal.map`/`save`/`load` with the sim-name mirror.
