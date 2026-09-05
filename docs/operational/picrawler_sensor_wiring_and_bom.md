# PiCrawler sensor wiring and BOM

> **The bench reference for the 2026-08-27 sensor addition.** Parts and wiring only. The
> *rationale* for each sensor — what criterion term it recovers, what it is allowed to be used
> for — lives in [`../plans-and-designs/picrawler_sim2real_port.md`](../plans-and-designs/picrawler_sim2real_port.md)
> Phase 4 and the FSR spec below it. Update this file in place as parts arrive.

**Platform:** Raspberry Pi 5 + SunFounder **Robot HAT V4**, 2S LiPo (6.0–8.4 V) via XH2.54.
HAT pinout from [SunFounder's hardware introduction](https://docs.sunfounder.com/projects/robot-hat-v4/en/latest/robot_hat_v4/hardware_introduction.html).

---

## 1. What the HAT already provides

| Interface | Detail | What we hang on it |
|---|---|---|
| I²C | GPIO2/3, **10 K pull-ups on-board**; two connectors — 4-pin P2.54 **and SH1.0 QWIIC** | INA219, VL53L0X |
| **SPI** | 7-pin P2.54: `BSY(GPIO6) · CS(CE0/GPIO8) · SCK(GPIO11) · MI(GPIO9) · MO(GPIO10) · 3V3 · GND` | **ICM-20948** |
| ADC | **A0–A3** user, 3-pin P2.54, **12-bit, 3.3 V reference**; A4 = battery via 20K/10K | 4 × FSR |
| Servo PWM | **12 channels P0–P11**, 3-pin P2.54, **5 V rail** | the 12 MG90S (existing) |
| Digital | D0→GPIO17, D1→GPIO4, D2→GPIO27, D3→GPIO22 | **ultrasonic trig = D2, echo = D3** — ✅ **MEASURED 2026-08-30** (§7); **D0/D1 are the free pair**, not D2/D3 |
| Power in | 6.0–8.4 V, XH2.54 3-pin | the INA219 goes **here** (§3) |

⚠ **Almost every GPIO is consumed by the HAT.** Only GPIO7 (CE1) and GPIO20 (NC) are unlisted,
and neither is broken out. **Everything added must go through the existing I²C / SPI / ADC
connectors** — which, as it happens, it all does.

### Address map — no conflicts

| addr | device | bus |
|---|---|---|
| `0x14` | HAT MCU — servos + ADC | I²C |
| `0x29` | VL53L0X belly ToF | I²C |
| `0x40` | INA219 | I²C |
| — | ICM-20948 | **SPI CE0** (off the I²C bus by design) |

---

## 2. BOM

### Sensors

| # | item | qty | spec that matters | note |
|---|---|---|---|---|
| 1 | **ICM-20948** breakout | 1 | must expose **CS / SCK / SDI / SDO** for SPI | ✅ **CONFIRMED 2026-08-30** — the received board breaks out `NCS` and `ADO`, so SPI is available (§4) |
| 2 | **INA219** breakout | 1 | I²C `0x40`, 26 V bus max ✓ | **shunt must be changed — see #3** |
| 3 | **0.01 Ω shunt resistor** | 1 | 2512, ≥ 1 W, 1 % | replaces the stock 0.1 Ω (§3) |
| 4 | **VL53L0X / VL53L1X** ToF | 1 | I²C `0x29` | belly clearance |
| 5 | **Circular FSR, 20 g – 2 kg** | 4 | active dia ~14–20 mm | feet |

### Passives and conditioning

| # | item | qty | note |
|---|---|---|---|
| 6 | FSR divider resistor `R_g` | 4 | **value is set by measurement, not chosen** — §5. Buy an assortment (1 kΩ–100 kΩ, 1 % metal film) and fit after measuring |
| 7 | 0.1 µF ceramic | 4 | one across each ADC input to ground, anti-alias / noise |

### Connectors and cable

| # | item | qty | note |
|---|---|---|---|
| 8 | XH2.54 3-pin M+F pigtail pair | 1 | to break the battery line for the INA219 without cutting the pack — **or one M-to-F extension cable, cut in half** (§3.1) |
| 9 | 3-pin P2.54 leads (servo-style) | 4 | FSR → A0–A3 |
| 10 | SH1.0 4-pin QWIIC cable | 2 | I²C chain: INA219 + ToF |
| 11 | 7-pin P2.54 cable / header | 1 | SPI → IMU |
| 12 | 30 AWG silicone hookup wire | — | FSR tails; flexible enough to survive full swing |
| 12b | **22 AWG silicone hookup wire** | — | the INA219 power path only — 26 AWG stock cable is marginal at 2–3 A (§3.1) |
| 13 | Heat-shrink, Kapton tape | — | tail strain relief |

### Mechanical — the foot stack (§5)

| # | item | qty | note |
|---|---|---|---|
| 14 | Rigid puck disc | 4 | **slightly SMALLER than the FSR active area.** 3D-print or Delrin. This is the toe cap |
| 15 | PTFE shim / low-friction slip layer | 4 | **shear isolation — the main failure mode** |
| 16 | Compliant backing pad | 4 | spreads point contact across the puck |

### Bench / calibration

| # | item | qty | note |
|---|---|---|---|
| 17 | Known masses: 50 / 100 / 175 / 300 / 590 g | 1 set | FSR calibration curve |
| 18 | Inline DC current meter | 1 | size the shunt before committing; sanity-check the INA219 |
| 19 | Multimeter | 1 | measure `R_fsr` in place at 175 g |

---

## 3. Power path and the INA219

**Decision (2026-08-27): the INA219 goes inline on the battery input, not on the servo rail.**

The 5 V servo rail is **internal to the HAT** — the regulator feeds P0–P11 directly and there is
no exposed break point. The alternatives were cutting the regulator's output trace (permanent
board modification) or feeding the servos from an external BEC (a rebuild). Neither is worth it,
because the ledger's own argument for this sensor was **bus total, not per-joint** — *"which for
an energy term is the more honest quantity anyway."*

```
  2S LiPo                INA219                        Robot HAT V4
  6.0–8.4 V           (0.01 Ω shunt)
     ┌───┐   XH2.54    ┌──────────┐      XH2.54       ┌──────────────┐
     │ + ├────────────►│ Vin+     │──────────────────►│ PWR IN  +    │
     │   │             │      Vin−│                   │              │
     │ − ├─────────────┴──────────┴──────────────────►│ PWR IN  −    │
     └───┘                   │                        │              │
                             │ I²C 0x40               │              │
                             └───────────────────────►│ QWIIC / I²C  │
                                                      └──────────────┘
  measures: Pi 5 + 5 V regulator + all 12 servos (whole-robot current)
```

⚠ **Change the shunt to 0.01 Ω before installing.** A stock breakout ships **0.1 Ω**, which
drops **300 mV at 3 A**. The pack already sags toward the HAT's **6.0 V minimum** under servo
transients, and adding 300 mV of series drop right before that threshold is a brownout risk.
0.01 Ω costs resolution and buys headroom; take the trade.

- **A4 stays on battery voltage.** It is the independent brownout check and the
  one-servo-at-a-time stall detector during calibration (~100 mV sag ≈ 40 counts).
- **The Pi's own draw is common-mode**, roughly constant across a gait. Subtract an idle
  baseline before the energy term consumes it.

### 3.1 Building the inline module non-destructively

The pack is broken with connectors, never by cutting it. **The connector is JST XH, 2.54 mm
pitch, 3-pin.** The HAT carries the male header (pins); the battery ends in a female housing
(crimp sockets). So the module needs a **female pigtail toward the HAT** and a **male pigtail
toward the pack**. The cheapest correct part is a **JST XH 2.54 3-pin male-to-female extension
cable cut in half** — both halves are then guaranteed to mate.

⚠ **JST XH contacts are rated 3 A each, and that is almost certainly why a 2-wire battery gets
a 3-pin connector: the pins are paralleled to share current.** Steady draw is ~2.3 A with
transients plausibly 4–7 A, at or past a single contact. **Two consequences.** The pin order is
load-bearing — **meter it, never assume it**: with the pack unplugged, identify `+`/`−` on the
battery's own connector and check which pins are commoned, and check continuity between pins on
the HAT side. Reverse polarity destroys the HAT, and mis-reading the paralleling quietly puts
the full current through one 3 A contact. And the module carries **three** conductors, not two:
the paralleled polarity passes straight through, and only the single sensed leg goes through
`Vin+`/`Vin−`.

⚠ **Wire gauge.** Stock XH extension cables are usually 26 AWG, marginal at 2–3 A continuous and
poor on the transients. XH crimp terminals accept 22–28 AWG — **use 22 AWG silicone**, the
thickest the terminal takes. The INA219 breakout's own screw terminals and traces are in this
path too (§2 #2): check their rating before they become the weakest link.

---

## 4. IMU — ICM-20948 on SPI

**Decision (2026-08-27): SPI, not I²C.** The HAT shares its I²C bus with all 12 servo writes,
and host-side jitter integrates directly into dead-reckoned yaw. SPI removes that at the source
rather than filtering it afterward, and the 7-pin header carries 3V3 and GND, so it is one cable.

**The board on the bench (confirmed 2026-08-30)** breaks out
`VCC · GND · SCL · SDA · NCS · ADO · INT · FSY · ACL · ADA`. `NCS` is the proof SPI is
available — that pin exists only for SPI. Wire it by the silkscreen:

| HAT SPI header | RPi | → breakout pin | note |
|---|---|---|---|
| `3V3` | — | `VCC` | powers VDD + VDDIO |
| `GND` | — | `GND` | |
| `SCK` | GPIO11 | `SCL` | I²C name on a pin that is SCK in SPI mode |
| `MO` | GPIO10 (MOSI) | `SDA` | likewise SDI in SPI mode |
| `MI` | GPIO9 (MISO) | **`ADO`** | ⚠ **there is no pin labelled `SDO`** |
| `CS` | GPIO8 / CE0 | `NCS` | |
| `BSY` | GPIO6 | `INT` | optional |

⚠ **MISO goes to `ADO`.** In I²C mode that pin is the address-select bit; in SPI mode it is the
data output. Looking for an `SDO` pin is where a first wiring attempt stalls.

**Leave `FSY`, `ACL` and `ADA` unconnected.** `ACL`/`ADA` are the auxiliary I²C master bus, and
the decision below is not to enable the internal I²C master — which is what leaves the AK09916
magnetometer dark. Accepted trade: dead-reckoned yaw comes from the gyro.

**Enable in `/boot/firmware/config.txt`:**
```
dtparam=spi=on
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```
The 400 kHz setting still matters — the INA219 and ToF remain on I²C alongside the servo traffic.

**Firmware notes carried from the port doc** (full detail there):
`WHO_AM_I` (`0x00`) = **`0xEA`** · the register map is **banked**, four banks via `REG_BANK_SEL`
(`0x7F`) — unlike the flat MPU-6050/9150 map · **FIFO reads regardless of bus** · **never enable
the internal I²C master**, which keeps the AK09916 magnetometer dark · **do not use the DMP.**

---

## 5. FSRs — foot wiring

Full conditioning, mounting and calibration spec is in the port doc
(`## SPEC — foot FSRs`). The wiring half:

```
        3.3 V  ◄── ⚠ see hazard note below
          │
        [ FSR ]        (in the foot)
          │
          ├──────────────► A_n  signal   (ADC, 12-bit, 3.3 V ref)
          │
        [ R_g ]  ── measured, not chosen
          │
         GND
                        ┌── 0.1 µF ──┐
              A_n ──────┴────────────┴────── GND
```

⚠ **Confirm the 3-pin connector's pin ORDER and its VCC rail against the board silkscreen before
powering anything.** Two hazards:

1. **Pin order is not documented** in the vendor hardware page — do not assume signal/VCC/GND.
2. **If the ADC connector's VCC pin is 5 V, do NOT use it as the divider's top rail.** The
   divider would present up to 5 V to a **3.3 V-referenced** ADC input. Take 3.3 V from the SPI
   header or an I²C connector instead.

**`R_g` is set by measurement:** assemble one foot completely, rest **175 g** on it (the
mid-stance operating point), measure `R_fsr` in place, set `R_g` to that value, and use the same
value on all four channels so per-foot variation shows up in calibration rather than in hardware.

| foot | ADC | leg (⚠ **by anatomy, not by sim name** — see the port doc's leg-naming mirror) |
|---|---|---|
| front-left | A0 | drives sim `fr_*` |
| front-right | A1 | drives sim `fl_*` |
| rear-left | A2 | drives sim `rr_*` |
| rear-right | A3 | drives sim `rl_*` |

---

## 6. Bring-up order — one device at a time, each with its own check

**Never add two at once.** Every step has a pass condition; if it fails, stop there.

| # | step | pass condition |
|---|---|---|
| 1 | Baseline, nothing added | `i2cdetect -y 1` shows **`0x14`** only — ✅ **PASS 2026-08-28** (needs `i2c-dev` in `/etc/modules` besides the overlay) |
| 2 | INA219 inline on the battery | `0x40` appears; idle current is plausible; its bus voltage **agrees with A4's** reading |
| 3 | ICM-20948 on SPI | `ls /dev/spidev*` shows `spidev0.0`; `WHO_AM_I` = **`0xEA`**; at rest one accel axis reads ≈ 1 g and the other two ≈ 0 |
| 4 | VL53L0X on I²C | `0x29` appears; distance tracks a tape measure |
| 5 | FSRs, **one foot at a time** | counts rise monotonically with the known-mass series; fit and store per foot |

**After all four FSRs:** command the standing pose on a flat floor. The four `foot_load` values
should sum to ≈ 1.0 (one body weight) and be roughly equal. **If they sum well below 1.0 the
calibration or the puck geometry is wrong, not the robot.**

---

## 7. The ultrasonic rangefinder — already on the robot

The physical PiCrawler carries an ultrasonic module. It is **not** in the BOM above because it is
already fitted, but it needs wiring decisions and one measurement before it is trusted.

**✓ Role settled (operator, 2026-08-28): the ultrasonic points FORWARD and is for obstacle
avoidance. The VL53L0X points DOWN and is the belly-clearance channel.** The two are separate
sensors with separate jobs — **the ToF is not redundant, fit it as planned.**

⚠ **Never route the ultrasonic into `gc_raw`.** That channel is a *downward belly* sensor
(`picrawler_body.gd:2854`, range 0.3 m, standing 0.06 m) and the deployed height homeostat rides
it; feeding it a forward reading would corrupt a promoted lever silently. Rationale in the port
doc's §7.7.

| concern | detail |
|---|---|
| pins | ✅ **MEASURED 2026-08-30: trigger = D2 (GPIO27), echo = D3 (GPIO22).** This entry previously read D0/D1 and was wrong — see the measurement below. **D0/D1 are the free pair.** |
| ⚠ level | HC-SR04-class modules drive echo at **5 V**; the Pi is 3.3 V-tolerant only. **Confirm SunFounder's module is already shifted for the HAT** before connecting — if not, add a divider or a shifter |
| rate | **~10–20 Hz, off the tick thread.** Ping flight time bounds it; it cannot be a 50 Hz channel |
| accuracy | echo is a userspace **pulse width** at ~58 µs/cm — 100 µs of scheduling jitter ≈ 1.7 cm |
| blind spots | absorbed by carpet, reflects away past ~30° off-normal, ~15° cone reports the nearest thing in a fat lobe |
| unmeasured | whether 12 servos couple acoustic noise into a 40 kHz receiver. Ten minutes on the bench |

**How the pins were established, since the documented pair was wrong.** Holding every D pin
as an *input* and reading it twice — once with an internal pull-up, once with a pull-down —
separates "connected to a device output" from "floating" without driving anything, so there
is no risk of contending with a module that is itself driving. GPIO22 read low under **both**
pulls (a device output idling low); GPIO4, GPIO17 and GPIO27 all followed the pull, i.e.
nothing is attached. Triggering GPIO27 then produced a clean echo on GPIO22 on every ping.
(The same sweep also showed **GPIO20 driven low** — the speaker amplifier enable, §"speaker".)

⚠ **Echo level (§8.7) is resolved in practice, not in theory.** The module drives GPIO22
directly and the pin has behaved across thousands of pings, so SunFounder's HAT is doing
whatever shifting is needed. We did not meter the echo's high level with a scope; if that
matters later, measure it rather than inferring it from "it works".

**Timing is kernel-side, not userspace.** The BOM's original worry — "echo is a userspace
pulse width at ~58 µs/cm, so 100 µs of scheduling jitter is 1.7 cm" — applies to a poll loop.
The driver uses libgpiod v2 **edge events**, whose timestamps are taken in the interrupt path,
so the width is the difference of two kernel timestamps and userspace scheduling only affects
*when we learn* the answer. Measured spread on a static target: **866 µs ± 3 µs**, i.e. about
±0.5 mm.

**Expose it as an instrument first.** Put it on the dashboard beside the belly channel and watch
it across existing gaits. It earns a lever only once someone can name the prediction error it
reduces — and only after the authority check.

---

## 8. Open — must be resolved at the bench, not from documentation

1. **3-pin connector pin order and VCC rail voltage** (ADC / digital / servo) — not in the vendor
   hardware page. Read the silkscreen. §5 hazard 2 depends on this.
2. **5 V regulator current rating** — undocumented. Relevant only if the servo rail is ever
   revisited; the battery-input placement sidesteps it.
3. **Actual gait current draw** — measure with the inline meter (#18) before trusting the 0.01 Ω
   choice. If peak gait current is well under 3 A the stock 0.1 Ω would give better resolution,
   but the brownout margin argues against it either way.
4. ~~**Whether the chosen ICM-20948 breakout exposes SPI**~~ — ✅ **RESOLVED 2026-08-30**: the
   received board exposes `NCS` and `ADO`, so SPI is available. Pin map in §4.
5. **FSR creep** — hold 175 g for 60 s and record the drift **before** the graded `unloaded`
   criterion term is trusted. If it is large, that term wants the threshold, not the magnitude.
6. **Ultrasonic mounting height and pitch** — the role is settled (forward, obstacle avoidance),
   but height and downward pitch set what it can see of the floor ahead. Record both.
7. ~~**Whether the ultrasonic module's echo is already level-shifted**~~ — ⚠ **partially resolved
   2026-08-30**: the module drives GPIO22 (D3) directly and thousands of pings have been read
   without incident, so the HAT handles it. The high level was never metered with a scope;
   treat as "works", not as "characterised".
