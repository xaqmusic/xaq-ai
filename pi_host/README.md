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

**Wire facts** (bench-verified 2026-08-28): MCU `0x14`; every register write is `[reg, hi, lo]`;
servo timer = `channel/4`, `PSC+t = 351`, `ARR+t = 4095` ⇒ **49.95 Hz** frames (not 50.00);
pulse count `trunc(µs/20000·4095)`, `0` = limp; ADC select `(7−ch)|0x10` then two 1-byte reads;
`Vbat = A4·3.3/4095·3`.

**Channel → anatomy map** (grows with calibration; sim names are mirrored — see the port doc):

| channel | physical | first seen |
|---|---|---|
| P0 | rear-left knee | 2026-08-28, `hat_servo_smoke.py 0` |

Robot on a stand for any servo verb.
