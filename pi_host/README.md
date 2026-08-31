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

**Bench-verified 2026-08-28** (pyzmq client from the laptop): telemetry under `ZMQ_CONFLATE`
(which is why frames are single-part), `servo.set` clamp + slew, one-at-a-time, deadman trip
after 1 s of silence, widen/restore audit, `cal.map`/`save`/`load` with the sim-name mirror.
