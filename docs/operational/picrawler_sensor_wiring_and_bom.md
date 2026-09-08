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
| Power in | 6.0–8.4 V, XH2.54 3-pin — **`−` / mid tap / `+`**, ✅ MEASURED 2026-09-05 (§3.2) | the INA219 goes **here**, in the `+` leg only (§3) |

⚠ **Almost every GPIO is consumed by the HAT.** Only GPIO7 (CE1) and GPIO20 (NC) are unlisted,
and neither is broken out. **Everything added must go through the existing I²C / SPI / ADC
connectors** — which, as it happens, it all does.

### Address map — no conflicts

| addr | device | bus |
|---|---|---|
| `0x14` | HAT MCU — servos + ADC | I²C |
| `0x29` | VL53L0X belly ToF | I²C — ✅ **PRESENT 2026-09-07**, model ID `0xEE` (§9) |
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
| 4 | **VL53L0X / VL53L1X** ToF | 1 | I²C `0x29` | belly clearance — ✅ **FITTED AND CALIBRATED 2026-09-07 (§9)** |
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

**The 3-pin connector is a 2S centre tap, not a paralleled pair — ✅ MEASURED 2026-09-05 (§3.2).**
Only the `+` leg is broken by the shunt; `−` and the mid tap pass straight through.

```
  2S pack, 3-wire            INA219                      Robot HAT V4
  6.0–8.4 V              (0.01 Ω shunt)
   ┌──────────┐  XH2.54   ┌──────────┐     XH2.54      ┌──────────────┐
   │ +   8.4V ├──22AWG───►│ Vin+     │                 │              │
   │          │           │      Vin−├──22AWG─────────►│ PWR IN  +    │
   │ mid 4.2V ├──26AWG────── straight through ────────►│ PWR IN  mid  │
   │ −   0.0V ├──22AWG────── straight through ────────►│ PWR IN  −    │
   └──────────┘                │                       │              │
                               │ I²C 0x40 + 3V3 + GND  │              │
                               └──────────────────────►│ QWIIC / I²C  │
                                                       └──────────────┘
  measures: Pi 5 + 5 V regulator + all 12 servos (whole-robot current)
```

⚠ **The shunt goes in the `+` leg, never the return.** A shunt in `−` lifts the HAT's ground
above the Pi's I²C ground by the drop across it.

⚠ **The mid tap touches nothing on the INA219** — not `Vin+`, not `Vin−`, not `GND`. It carries
no load current. The breakout's `VCC` comes from the HAT's **3.3 V**, never from the pack.

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

⚠ **Wire gauge, and the contact that carries everything.** The third pin does **not** share the
load (§3.2), so the whole draw — ~2.3 A steady, plausibly 4–7 A on servo transients — goes
through **one 3 A-rated XH contact**, at or past its rating on the peaks. Nothing in the module
can fix that, but two things stop it getting worse: stock XH extension cable is usually 26 AWG,
marginal at 2–3 A continuous and poor on transients, so **crimp 22 AWG silicone** on `+` and `−`
(XH terminals accept 22–28 AWG; the mid tap carries only balance current, so stock 26 AWG is
fine there). And the INA219 breakout's own screw terminals and traces are in this path too
(§2 #2) — check their rating before they become the weakest link. **Watch the connector for
heating during the first full-servo load test.**

### 3.2 The battery connector — ✅ MEASURED 2026-09-05

The 3-pin XH is the pack's **2S centre tap**, the standard `B− / cell1+ / pack+` balance
layout — *not* two paralleled `+` pins sharing current, which is what this section previously
assumed. Pack unplugged, DC volts:

| pair | reading | reads as |
|---|---|---|
| `−` → `+` | **8.4 V** | full pack |
| `−` → middle | **~4.2 V** | cell 1 |
| middle → `+` | **4.2 V** | cell 2 — this is the one that settles it |

Two paralleled `+` pins would both read 8.4 V and 0 V between them. Half the pack voltage on the
middle wire can only be a cell junction.

**The HAT side**, everything unplugged, on ohms:

| pair | reading | reads as |
|---|---|---|
| middle → `+`, middle → `−` | **0.5 MΩ** both | floating pin, read through board leakage — 8 µA at 4.2 V |
| `+` → `−` | **45 kΩ** | bleed / divider path (the A4 sense divider is 20K/10K = 30K), **not a short** |

So the mid tap can pass straight through: the HAT does nothing with it, and it is not commoned
to either rail.

⚠ **This was the hazard worth metering.** Had the HAT's middle pin been commoned to `+` or `−`,
plugging in a centre-tapped pack would short one cell through two 3 A contacts. **Meter the HAT
side before mating a new pack**, and match the plug by its housing key, never by wire colour —
with a mid tap present, a reversed plug lands +8.4 V on the HAT's `−` pin.

### 3.3 Calibrating for the 0.01 Ω shunt

**The driver already does this — `ogma::hw::Ina219` (`pi_host/src/Ina219.cpp`).** What matters
on the bench is the two things it cannot know:

**`r_shunt` is calibration data, not 0.010.** At 10 mΩ, trace and solder resistance are a large
fraction of the part, and a multimeter cannot measure it — probe leads alone are ~200 mΩ. Fit it
against a known current and store it in the calib JSON. The authoritative current is derived
host-side from raw shunt microvolts precisely so a later re-fit re-derives every recorded
sample; the chip's own `CURRENT` register is programmed as a cross-check only.

**PGA stays at /8 (±320 mV).** The tempting move is to narrow it to /2 for "more resolution" —
there is none to gain. `SHUNT_V`'s LSB is **10 µV on every range**; the PGA sets full scale
alone. At 0.01 Ω that is **1 mA per count** regardless, 0.04 % of the ~2.3 A steady draw. What
narrowing does buy is a clipped inrush that looks like a real number. `hat_tool` flags it
(`! PGA CLIPPED`) rather than letting it pass, but the right default is not to clip.

| setting | value | why |
|---|---|---|
| bus range | 16 V | pack maxes at 8.4 V |
| PGA | **/8, ±320 mV** | resolution is fixed at 10 µV; width is free, clipping is not |
| BADC/SADC (telemetry) | 128-sample avg, ~68 ms | averages servo PWM ripple out of a human-read number |
| SADC (inrush capture) | 12-bit single, 532 µs | ~1.9 kHz; bus channel dropped — pack voltage is not what browns out |

**Cross-check before trusting a reading:** `hat_tool ina probe` compares the INA219's bus voltage
against **A4** and exits non-zero if they disagree by more than 150 mV — two independent paths to
one number. Then verify current against the inline DC meter (§2 #17–19).

### 3.4 Bring-up — ✅ MEASURED 2026-09-05

Module built per §3, I²C on `SDA`/`SCL` + 3V3 + GND to the HAT. `i2cdetect -y 1` shows `0x14`
and `0x40`. First `hat_tool ina probe`, servos undriven:

```
INA219 0x40   r_shunt 0.01000 ohm   I_lsb 1000.0 uA   cal 4096
  bus     7.968 V     shunt +4.690 mV (raw +469)
  current +0.469 A    (chip reg +0.469 A, power 3.70 W)
  A4      7.891 V     delta +0.077 V  -> AGREE (BOM 6.2 pass)
```

| check | result |
|---|---|
| shunt sign | **positive** — `Vin+`/`Vin−` orientation correct |
| INA219 bus ⟷ A4 | **+77 mV**, inside the 150 mV gate — ~1 % on the 20K/10K divider, ordinary tolerance |
| PGA clip / OVF | neither |
| series drop from the R010 | **4.69 mV** at idle, vs ~47 mV the stock 0.1 Ω would have cost at this current |
| idle draw | **0.469 A / 3.70 W** — Pi 5 + HAT regulator, servos undriven. This is the common-mode baseline to subtract before the energy term consumes it |
| stability | 5 samples over 5 s: **463–471 raw** (±4 mA on 467 mA, < 1 %), bus flat at 7.964 V |

**Still open:** `r_shunt` remains the 0.010 placeholder — the bench fit against a known current
has not been done, so absolute current is uncalibrated (the *relative* record is already sound,
since current is derived host-side from raw microvolts and a later re-fit re-derives every
recorded sample). And the inrush capture the driver was written for — `hat_tool ina capture`
across a pose recall — has not been run.

⚠ **Unresolved: pack sag.** Open-circuit the pack read **8.4 V** (§3.2); under a 0.47 A load it
reads **7.964 V**. If the pack is simply partly discharged that is nothing — 3.98 V/cell is a
normal resting point. If it is genuinely 436 mV of sag at 0.47 A, that is ~0.9 Ω of source
impedance, which at a 4 A transient would be 3.6 V and straight through the HAT's 6.0 V minimum.
**RESOLVED same day:** the 8.4 V reading was taken *while the charger was connected* — the
charger holding its CV endpoint, not an open-circuit pack. Unloaded resting voltage is ~7.96 V
(`benchd` logged `vbat 8.32 V` charging at 18:29, `7.98 V` off charge at 20:27). There is no
anomalous sag and no source-impedance problem.

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
| 4 | VL53L0X on I²C | `0x29` appears; distance tracks a tape measure — ✅ **PASS 2026-09-07** (§9): `0x29` present with model ID `0xEE`; 259 readings at a bench target measured 121.9 mm ± 1.51 mm, 0 invalid; then validated on the robot at two points, belly-down and standing (§9.3) |
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

### 3.5 Inrush across a pose recall — ✅ MEASURED 2026-09-05

The measurement the driver was written for (ledger 2026-08-29: a pose recall browns the Pi out
on the shared 5 V/3 A rail, and A4 is blind to the transient). `hat_tool ina capture 12` at
**1880 Hz**, trigger at t = 2.0 s: `pose.set` → `stand`. **Robot on the floor, carrying its own
weight.** `ogma_host` is senses-only and `benchd` never touches the INA219, so the capture tool
owned the part; the pose went through `benchd`, so there was only ever one writer to the servos.

### ⚠ 3.5.1 First, the trap: `benchd`'s deadman truncates any unattended pose move

**`DEADMAN_MS = 1000`, refreshed by *any* verb** (`benchd.cpp:37`, `:371`). While a channel is
armed, a client that falls silent for one second triggers a `rescue` — a slew back to the
`rescue` pose. **A script that fires `pose.set` and then waits is not measuring a pose recall.**
It measures ~1 s of the move, then the rescue moving the other way, superimposed.

This produced two invalid captures before it was caught. The tell was in `benchd`'s own log:

```
1197228 pose.set     {"us":[1455,...]}      <- the trigger
1198248 deadman      {"trips":35}           <- 1.02 s later
1198248 rescue       {"pose":"rescue","why":"deadman"}
1201128 pose.landed                          <- the RESCUE landed, not `stand`
```

35 `deadman` / 35 `rescue` / 35 `pose.landed` against **2** `pose.set`. The trap is that the
truncated capture looks perfectly plausible — a smooth hump, no clipping, a sane peak.

**Two rules follow.** Keep the deadman fresh — ping at ≤ 200 ms for the whole move. And
**verify the landing**: read `servos[].current_us` back and compare it to the target. Both
captures were disproved by that one check.

### 3.5.2 The measurement

Keepalive at 200 ms; `current_us` verified equal to the target on all 12 channels; no new
`watchdog_trips`.

| | value |
|---|---|
| baseline (rescue pose, on the floor) | **0.654 A** (0.615–0.959) |
| **peak** | **2.481 A at +1.305 s** after the trigger |
| peak as fraction of PGA /8 range | **2481 of 32000 counts — 7.8 %** |
| move complete | ~+1.75 s |
| **holding `stand`** | **0.625 A** — at or below baseline |
| cost of standing up | **0.726 A·s** above baseline ≈ 5.7 J at 7.8 V |
| `vcgencmd get_throttled` | **`0x0`** — no undervoltage, not even the sticky bits |

**The stagger works, and this shows how.** The peak is not a spike at t = 0 but a rise cresting
at **+1.305 s** — the end of the 100 ms × 12 launch window, where every channel is slewing at
once. The mitigation was written blind ("staggered + gentle: protects the Pi's rail"); this is
the first evidence of what it does.

**Holding costs nothing.** A standing quadruped rests on its gearboxes: with no position error
the servos draw no more than idle. Holding (0.625 A) is *below* the splayed rescue pose
(0.654 A), where legs sit near their limits. **The energy term lives in the transitions, not in
the postures** — worth knowing before an energy cost is wired to anything.

**PGA /8 is confirmed free.** The worst loaded transient reached 7.8 % of full scale. Narrowing
the range would have bought no resolution (§3.3: 10 µV LSB on every range) and risked clipping.

⚠ **Two limits on this number.** The **5 V rail is the constrained one and this measures the
battery side** — 2.481 A at ~7.8 V is what the pack delivers, not what the 3 A regulator sees,
and the INA219 cannot separate the Pi's share from the servos'. And the capture loop polls I²C
at 1.9 kHz, raising the Pi's own draw ~120 mA: **compare capture-to-capture, never against
§3.4's 0.469 A telemetry baseline.**

**Not yet measured:** the same recall *unloaded* on a stand, for a load-vs-no-load comparison.
The stand attempt on 2026-09-05 was one of the two deadman-truncated captures and was discarded.

---

## 3.6 Servo duty budget — concurrency sweep, ✅ MEASURED 2026-09-05

**Question:** the 2026-08-29 brownout happened when all 12 servos moved at once. Where is the
boundary, so a duty budget can be set?

**Design.** Lever = **K**, the number of servos commanded inside one 20 ms tick, issued as K
back-to-back `servo.set` verbs (which arm channels independently and bypass the pose stagger
entirely; measured send spread 0.3–3.6 ms, so they start on the same tick or the next). Held
constant: +400 µs travel, the driver's default 40 µs/tick slew, the rescue pose as home
(splayed and low — the robot cannot topple as K climbs), direction, and channel order.
**Channel order is knees first, one per leg** (`0,3,6,9` → hip2 `1,4,7,10` → hip1 `2,5,8,11`),
so every added servo is a different leg. **4 repeats per K, interleaved round-robin**, so
battery drain spreads across every K instead of aliasing onto the trend.

**Metrics.** Baseline, raw peak, p99, and the peak of a **10 ms moving average**. The last is
the one to read: a rail sags on sustained draw, and a single 532 µs sample is not that. A first
attempt reporting raw peak from **one** trial per K was discarded — its repeated K=4 control
came back 1.064 A against 1.419 A, a spread as large as the whole K=1→K=10 trend.

**Tripwire:** `vcgencmd get_throttled` after every trial, aborting on the first non-zero.

| K | peak A (mean ± sd) | **10 ms A (mean ± sd)** | over baseline | throttled |
|---|---|---|---|---|
| 1 | 0.844 ± 0.052 | **0.773 ± 0.039** | +0.170 | `0x0` |
| 2 | 0.894 ± 0.032 | **0.823 ± 0.020** | +0.218 | `0x0` |
| 4 | 1.082 ± 0.092 | **1.017 ± 0.079** | +0.412 | `0x0` |
| 6 | 1.289 ± 0.075 | **1.247 ± 0.064** | +0.650 | `0x0` |
| 8 | 1.630 ± 0.078 | **1.565 ± 0.068** | +0.974 | `0x0` |
| 10 | 2.024 ± 0.056 | **1.947 ± 0.046** | +1.356 | `0x0` |
| 12 | 2.272 ± 0.141 | **2.191 ± 0.138** | +1.601 | `0x0` |

Baseline ~0.60 A throughout; pack 7.63–7.80 V, n=28 trials.

**≈ 0.135 A per concurrent servo** (least-squares over K=1–12). The relation is mildly convex —
marginal cost per added servo rises from ~0.05 A at K=2 to ~0.19 A at K=10 — so a linear budget
is slightly optimistic at the top end.

### 3.6.1 The stagger is defeated by its own gentleness

The deployed pose move is **not** one leg at a time. Concurrency during a staggered move is

```
  K_concurrent  ≈  min(12,  travel_time_ms / stagger_ms)
  travel_time_ms = (travel_us / slew_us_per_tick) × 20 ms
```

With the pose defaults (`slew 12 us/tick`, `stagger 100 ms`), the `rescue`→`stand` recall
travels up to 940 µs on a channel = **1567 ms**, against a launch window of 12 × 100 ms =
**1200 ms**. The first channel is still moving when the last one starts: **every channel ends up
in motion together.** The stagger delays full concurrency, it does not prevent it.

This is corroborated by the two independent measurements agreeing: the loaded `stand` recall
peaked at **2.481 A** (§3.5), and this sweep puts **K=12 at 2.19–2.27 A** unloaded. Same regime.
The recall's peak at **+1.305 s** is exactly where the launch window closes and all 12 overlap.

**The knob is the ratio, not the stagger alone.** To hold a real one-leg-at-a-time K=3 at the
current pose slew, the stagger would need to be ~520 ms, not 100 ms.

### 3.6.2 What this does NOT establish

⚠ **The boundary was not reached.** K=12 produced no throttle event — not one non-zero
`get_throttled` in 28 trials. The 2026-08-29 brownout involved something harsher than a 400 µs
step from a splayed pose: more travel, real body load, or a stall. **A duty budget cannot be set
from this alone**; what the sweep gives is the *slope*, not the limit.

⚠ **This measures the battery side; the brownout is on the HAT's 5 V rail.** The INA219 cannot
see that rail and cannot separate the Pi's share from the servos'. `get_throttled` remains the
only direct evidence of the rail failing, and it is binary and sticky.

⚠ **Slew is the unmeasured second axis**, and it cuts both ways: a faster slew raises per-servo
current but shortens the overlap window. This sweep held slew fixed at 40 µs/tick. The budget is
two-dimensional and only one dimension has been measured.

**To actually find the boundary,** in increasing order of cost: sweep travel and slew upward at
K=12 until `get_throttled` moves; or put a sense wire on the 5 V rail and stop inferring it.

---

## 3.7 Where the cliff is — source impedance, ✅ MEASURED 2026-09-05

The 2026-08-29 event was a **hard Pi shutdown**, not a throttle. That rules out
`get_throttled` as the instrument: an unclean shutdown takes the reading with it, and the
approach risks the SD card. **Pack sag is the leading indicator** — the HAT's regulator drops
out below 6.0 V in — so `hat_tool ina sag` was added (shunt AND bus, both 12-bit, ~940 Hz).

Fitting pack volts against current across a K=12 transient (a load already run 4× safely):

```
n=4700 samples   I 0.598-2.568 A   V 7.012-7.792 V
fit:  V = 8.025 - 0.3943 * I        R_source = 394 mohm
```

| | value |
|---|---|
| open-circuit intercept `V0` | **8.025 V** (~4.01 V/cell, roughly half charge) |
| **source impedance** | **394 mΩ** — pack + wiring + XH contacts + shunt |
| sag already present at the K=12 peak | **1.01 V** (2.568 A → 7.012 V) |
| **current at which the pack reaches 6.0 V** | **≈ 5.13 A** |
| headroom from the K=12 peak | **2.57 A** |

The fit predicts the observed minimum to 1 mV (8.025 − 0.394 × 2.568 = 7.013 vs 7.012 measured).

**394 mΩ is high, and that is the real finding.** Cells account for maybe 100–160 mΩ of it
(2S 18650 at 50–80 mΩ each). **The remaining ~250 mΩ is wiring, connector and contacts** — the
single 3 A XH contact and cable gauge flagged in §3.1. Every milliohm removed there is bought
back directly as brownout headroom, and it is the cheapest fix available.

⚠ **Scope of the 5.13 A figure.** It is a **2× extrapolation** from 2.57 A; Li-ion series
resistance is near-constant over this range, so it is defensible, but it is not measured. It is
also **state-of-charge specific** — at a lower SoC `V0` falls and `R` rises, so the cliff moves
closer. And 6.0 V is the HAT's *stated* input minimum; the actual regulator dropout may sit
either side of it.

### 3.7.1 The budget this implies — and the lever it is NOT

**At the driver's default 40 µs/tick slew, concurrency is not the constraint.** §3.6 measured
all 12 servos at once as 2.19–2.57 A, which is **half** the 5.13 A cliff. The historical
brownout happened at **full speed**, and the slew limiter is exactly what "full speed" bypasses:
40 µs/tick caps a servo at 2000 µs/s, well under an MG90S's own maximum velocity.

**So the duty budget is a slew cap, not a concurrency cap.** §3.6's concurrency sweep found no
boundary because it held the variable that matters fixed.

A first cut, to be replaced by measurement:

| quantity | value |
|---|---|
| hard limit (pack → 6.0 V, at this SoC) | 5.13 A |
| working budget with margin | **≤ 3.5 A peak** (pack ≥ 6.65 V) |
| cost per concurrent servo at slew 40 | ~0.135 A (§3.6) |
| measured worst case so far, K=12 at slew 40 | 2.57 A ✅ inside budget |

**Not yet measured: current vs slew rate.** That curve can be mapped entirely *below* the cliff
— escalate slew at fixed K=12 and stop at 3.5 A — which yields the budget without reproducing
the shutdown. **A stall hypothesis was raised and withdrawn:** the X pose's out-of-range 2500 µs
entries are UI slider artifacts, and the servo map clamps to a calibrated in-range limit, so
nothing drives into a mechanical stop.

---

## 3.8 Slew is the lever — and the 5 V rail is the real cliff, ✅ MEASURED 2026-09-05

Sweeping **pose slew** at fixed motion (`rescue`↔`X`, `--pose-stagger-ms 0` so all 12 channels
start on the same tick), 3 reps each, `hat_tool ina sag` at 940 Hz.

| slew µs/tick | peak A (mean of 3) | min pack V | note |
|---|---|---|---|
| 12 | *1.64* | *7.37* | ⚠ **invalid** — 2000 µs at 12 µs/tick takes 3.3 s, longer than the 2.0 s window, so the servos reversed mid-flight |
| 20 | 1.16 | 7.55 | |
| 32 | 1.38 | 7.47 | |
| 50 | **1.90** | 7.24 | |
| 80 | 2.61 | 6.88 | |
| 125 | 2.76 | 6.89 | |
| 200 | 2.90 | 6.86 | **saturated** (vinyl) |
| 320 | 2.86 | 6.84 | |
| 500 | 2.96 | 6.81 | |
| 800 | 2.93 | 6.83 | ⚠ **leather couch** from here |
| 1300 | 3.04 | 6.70 | ⚠ leather couch |
| 2000 | — | — | ⚠ leather couch · **💀 hard Pi shutdown, unclean reboot** |

Within-point repeatability was excellent (e.g. 1.639 / 1.642 / 1.652 A), far tighter than §3.6.

**Current saturates near 3.0 A above ~200 µs/tick.** Past that the limiter stops limiting: the
servos are already at their own maximum velocity, and raising the slew number changes nothing.
So "full speed" is reached at slew ≈ 200, not at 2000.

### 3.8.1 The cliff is the 5 V regulator's current limit, not pack sag

§3.7 extrapolated a pack-sag cliff at **5.13 A** (pack → 6.0 V). **The collapse happened at a
pack voltage never observed below 6.70 V**, and at a measured current of ~3.0 A. The pack-sag
model does not explain it — it was answering the wrong question, exactly as §3.7's own caveat
warned.

The arithmetic that does fit (**inference, not measurement** — the 5 V rail is not instrumented):

```
  slew 1300:  3.04 A x ~6.8 V  =  20.7 W drawn from the pack
  less Pi + regulator idle     =  -4.6 W
  servo share                  =  ~16 W  ->  3.2 A at 5 V even at 100% efficiency
  plus the Pi's own 5 V draw   ->  ~3.4 A on a rail rated 3 A
```

**We were over the HAT's 3 A rating from slew ≈ 200 onward** and got away with it until a
single-tick full-scale step on all 12 channels at once. That also explains 2026-08-29: the
failure is a **regulator current limit**, which trips fast and hard, rather than a voltage sag,
which would have shown as a droop first.

### 3.8.2 The duty budget

| slew µs/tick | peak battery A | verdict |
|---|---|---|
| ≤ 50 | ≤ 1.90 | ✅ **the budget** |
| 80 | 2.61 | ⚠ at the edge |
| ≥ 200 | ~3.0 | ❌ over the 5 V rail's rating |

**The deployed defaults are already inside it** — pose slew 12, `NORMAL_SLEW_US` 40. The danger
is any path that bypasses slew limiting entirely. **The budget is a slew cap; concurrency (§3.6,
2.19–2.57 A for all 12 at slew 40) never was the binding constraint.**

⚠ **Every number in that budget was measured on low-friction vinyl (§3.8.3) and is therefore
optimistic.** Re-verify slew 50 and 80 on a high-friction surface before relying on them.

### 3.8.3 The surface changed, and it splits the sweep in two

**Slew 20–500 ran on a low-friction vinyl floor; slew 800, 1300 and 2000 ran on a leather
couch.** The operator moved the robot mid-sweep because the X pose at high slew was slamming the
chassis into the ground, and reported visibly higher draw afterward.

**Why friction changes the electrical load.** On slippery vinyl the feet slide when a leg pushes:
the body does not rise, and the servo turns against little torque. On grippy leather the foot
holds, so the servo must actually lift the chassis. **Grip converts free motion into work**, and
work is current. The couch was also compliant and slightly taller, which changes leg geometry
on top of the friction.

**What survives.** The saturation finding rests on slew 200/320/500 (2.90 / 2.86 / 2.96 A), all
**vinyl** — one surface, so the plateau is real. Curiously the couch points (800: 2.93, 1300:
3.04) sit right on that same plateau, which suggests the surface effect is small *once the
servos are velocity-saturated*: at max velocity the current is dominated by acceleration rather
than by steady load.

⚠ **What does NOT survive: the 1300 → 2000 comparison.** Two things differ between the last
surviving point and the collapse — a slew-limited ramp becomes a **single-tick full-scale step**
on all 12 channels at once, **and** the robot is on a grippy compliant surface. Two variables,
one outcome: **the collapse cannot be attributed to either.** The step is the better suspect (a
simultaneous full-scale error draws startup current from 12 stalled rotors at the same instant,
plausibly faster than 940 Hz sampling can even see), but that is a hypothesis, not a result.

**To separate them, without approaching collapse:** re-run slew 320 / 500 / 800 on the grippy
surface and compare against the vinyl values at the same slews. That isolates friction at
constant slew and stays on the measured plateau.

⚠ **Mechanical, not electrical:** at high slew the X pose slams the chassis into the ground.
That is a hardware risk independent of the rail, and a reason to cap slew that has nothing to do
with current.

⚠ **Still the cheapest headroom:** the ~250 mΩ of wiring and connector impedance (§3.7).

⚠ **Write captures OUTSIDE `/tmp`.** It is tmpfs. The brownout rebooted the Pi and destroyed
every raw capture in this sweep, leaving only the printed summary — so the surface-change
confound could not be tested afterward. **A brownout experiment must persist its evidence
outside the thing it is trying to break.**

---

## 3.9 Friction isolation — the slew cap only works on a slippery floor, ✅ MEASURED 2026-09-05

Same slews, same motion (`rescue`↔`X`, stagger 0 → K=12), same window, 3 reps — **surface as the
only lever**, vinyl → leather couch.

| slew | vinyl peak A | **couch peak A (mean ± sd)** | Δ | couch min V |
|---|---|---|---|---|
| 50 | 1.90 | **2.615 ± 0.113** | **+0.72 (+38 %)** | 6.82 |
| 80 | 2.61 | **2.667 ± 0.112** | +0.06 (+2 %) | 6.80 |
| 320 | 2.86 | **2.907 ± 0.090** | +0.05 (+2 %) | 6.73 |
| 500 | 2.96 | **2.829 ± 0.026** | −0.13 (−4 %) | 6.78 |

Baseline (0.59–0.63 A) and holding current (0.55–0.73 A) are unchanged by surface. **The entire
effect is in the transition.**

### 3.9.1 What this means: the lever depends on the floor

**On the couch, current is flat at ~2.6–2.9 A from slew 50 all the way to 500.** Slew stops
being a lever entirely. The mechanism is friction: on vinyl the feet slide, the body never
rises, and a slow move costs almost nothing — so on that surface current tracks velocity. On
leather the foot grips, so *any* move must lift the chassis, and load torque sets the current
regardless of how slowly it is done.

**This overturns §3.8.2's budget.** `slew ≤ 50` was the most surface-optimistic point in the
whole sweep: 1.90 A on vinyl, **2.62 A on the couch — a 38 % rise, landing it at the same level
as slew 320.** There is no slew setting that makes the `rescue`↔`X` transition cheap on a grippy
surface; it costs ~2.9 A and sags the pack to 6.73 V however it is commanded.

**The lever that survives both surfaces is the STAGGER.** Every measurement in §3.8 and §3.9 ran
at `--pose-stagger-ms 0` — full concurrency. Staggering divides the peak by spreading channel
starts in time, and unlike slew that works whatever the load per servo is. But §3.6.1 showed the
deployed stagger is **defeated by its own gentleness**: at pose slew 12, travel time per channel
(~1.5 s) far exceeds the 1.2 s launch window, so every channel ends up moving together anyway.

| surface | slew cap | stagger |
|---|---|---|
| low friction (vinyl) | ✅ effective | ✅ effective |
| high friction (leather) | ❌ **no effect** | ✅ effective |

**Set the budget on stagger, sized against travel time (§3.6.1), not on slew.**

### 3.9.2 The second shutdown was a STALL, not a budget overrun

Slew 800 on the couch took the Pi down. **The operator observed a leg catching on a seam at the
moment of the reset.** A snagged servo is a locked rotor: it draws several times its running
current, indefinitely, and no duty budget prevents it because the trigger is terrain, not a
command.

⚠ **This is a distinct failure path and it deserves its own mitigation** — stall detection and
release. The machinery is already present but unused for this: `ServoDriver` tracks
`time_at_limit_s` per channel, and the INA219 now resolves a current step at 940 Hz (§3.7).
A stalled channel held against an obstruction is exactly what those two together can see.

⚠ **A stall also re-opens the mechanism §3.5 dismissed.** The earlier stall hypothesis was
correctly withdrawn — the servo map clamps to in-range limits, so commands never drive into a
stop. **Terrain does what commands cannot.**

⚠ **`fsync` or lose it.** Captures were moved out of `/tmp` after the first brownout, but the
slew-800 files are still **0 bytes** and the run log lost its final lines: `python3 -u` and
`fprintf` are unbuffered *at the application level only*, and the page cache dies with the
power. **Evidence from a brownout experiment must be fsync'd per record, or streamed off-box.**

---

## 3.10 Current as a telemetry channel — ✅ SHIPPED 2026-09-05

`ogma_benchd` now owns the INA219 and publishes it in every 10 Hz frame. **Instrument only:
nothing in the daemon or the brain consumes it.** With the part absent the frame carries
`"ina": null` and every other behaviour is unchanged.

```json
"ina": {"ok":true,"i_a":-0.413,"v":8.084,"i_ema":-0.463,"i_peak":0.0,"i_max":0.0,
        "charge_as":-87.9,"energy_j":-710.0,"r_shunt":0.01,"charging":true,"resync":false,"errors":0}
```

| field | what it is |
|---|---|
| `i_a`, `v` | instantaneous current and the INA219's **own** bus voltage — a second path to a number A4 also reports |
| `i_ema` | **the slow metric**: τ = 30 s. Instantaneous current says nothing about duty |
| `i_peak` | decaying peak-hold, τ = 60 s — the *recent* worst, not a number stuck on one old spike |
| `i_max` | worst since daemon start |
| `charge_as`, `energy_j` | what the robot has actually spent — the budget itself |
| `charging` | see below |
| `resync` | another process had reprogrammed `CONFIG`; this frame re-applied it |

Time constants are in **seconds, converted per-sample** (`1 − exp(−dt/τ)`), so the numbers keep
their meaning if the telemetry rate ever changes.

**Consumers.** `picrawler-dash` gains a `power` line and a `slow` line. The Godot bench
dashboard gains a `power` row and a **60 s scrolling graph** (`scripts/current_graph.gd`) sitting
directly under `vbat` — the pair is the diagnostic, since sag without draw is a tired pack and
draw without sag is a healthy one. The graph draws each column as the **min–max** of its samples
(a per-column mean would smooth away the spike it exists to show), always keeps **zero and the
3 A rail line** on screen, and runs a **pose-move activity band** along the bottom. That band is
the point: a current trace with no record of what the body was doing is just a wiggle.

⚠ **The 3 A line is the HAT's regulator rating — a datasheet fact, not the duty budget.** §3.9
measured the budget to be surface-dependent (1.90 A on vinyl, 2.62 A on leather for the same
move), so drawing a fixed budget line in an instrument people trust would be a lie.

### 3.10.1 Charging runs backwards through the shunt

First live reading was **−0.354 A** with `vbat` climbing: the charger feeds the pack **through
the HAT input we instrumented**, so charge current crosses the shunt in reverse.

- **Useful:** the sign is a plugged-in detector, and a real interoceptive state the robot could
  eventually sense for itself.
- **Dangerous:** while it is true, **every energy number is confounded** — the accumulators run
  negative and any "what did that movement cost" reading is measuring the charger. Hence the
  explicit `charging` flag and the banner on both dashboards.
- `i_peak` / `i_max` floor at zero, because peak means *worst draw*; a signed peak-hold would
  quietly report the charge rate instead.

### 3.10.2 ⚠ Stop `benchd` before any high-rate capture

benchd re-asserts `CONFIG` whenever it finds it changed (`Ina219::ensure_configured`), which
keeps its own telemetry honest — **verified: 8 resync frames during a 2 s `hat_tool ina sag`,
and zero frames with implausible bus voltage.** But the two processes then fight over the
register, so **`hat_tool ina capture` / `sag` run against a live benchd is mutually corrupting.**

**`sudo systemctl stop ogma-benchd` before a capture**, as the §3.8 slew sweep already did. Every
capture in §3.5–§3.9 predates benchd owning the part and is unaffected.

---

## 9. The VL53L0X belly rangefinder — ✅ FITTED AND CALIBRATED 2026-09-07

The downward belly-clearance channel: the `gc_raw` / `gc_norm` signal the promoted height
homeostat rides. **Not** the forward ultrasonic (§7) — separate sensors, separate jobs, and
crossing them would corrupt a promoted lever silently.

Driver: `pi_host/src/Vl53l0x.cpp`. Bench tool: `hat_tool tof probe|watch|log`.

### 9.1 Mounting — as built

Mounted on a **solid boom off one of the HAT's own mounting screws**, on a standoff that puts
the module about **4 mm below the top of the HAT**, looking down.

| concern | as built |
|---|---|
| which face points down | the one carrying the **two shiny apertures** — VCSEL emitter and SPAD receiver, ~2.8 mm apart. On this breakout the chip is on the component side, so the **board mounts component-side-down**, connector facing up |
| cover glass | **none.** Open path to the floor. Any window over the aperture couples emitter light straight into the receiver, and fixing that needs an air gap, an opaque barrier between the apertures, and a crosstalk calibration |
| protective film | **removed.** Left on it does not block the reading — it gives a plausible wrong one |
| XSHUT | **not wired.** The breakout pulls it up; `0x29` answering is the proof |
| rigidity | boom + standoff, no tape. Measured sd is **1.51 mm, identical to the same sensor's sd on a static bench target** — so the mount contributes no measurable noise, and the spread is all sensor |

⚠ **The cone is ~25° full angle**, so the spot is `0.44 × distance` across: ~29 mm at the
belly-down standoff, ~51 mm at the sensor's own height above the floor. **Anything that enters
that cone reads as floor.** A leg segment swinging through it returns a short distance and the
homeostat reads that as *belly grounded* — see §9.5.

### 9.2 The mount offset — ✅ FITTED 2026-09-07

⚠ **`mount_offset_mm` is calibration data, on the same contract as the INA219's `r_shunt`
(§3.3).** The published record is `raw_mm` as the chip reported it; clearance is derived
host-side, so a later re-fit re-derives every stored sample instead of stranding the record.

**Method: one point, at the end that matters.** With the **belly flat on the floor** the true
clearance is 0 by definition, so the offset is simply the raw reading there. 243 samples off
the live telemetry channel, robot still:

```
raw_mm   mean 64.84   median 65   sd 1.51   min 61   max 70
drift    first half 65.02 mm -> second half 64.66 mm   (-0.37 mm over 25 s)
signal   23.21 Mcps   ambient 0.030 Mcps      invalid 0/243
```

Unimodal, no settling. **`mount_offset_mm = 64.8`**, and re-reading belly-down after applying
it gives **0.4 mm ± 0.69** — inside the sensor's own noise of zero.

⚠ **The fit is only as repeatable as the belly-down pose itself.** Re-measuring the same
resting baseline later, after the robot had been through a stand and a return to `rescue`, gave
**66.5–66.6 mm** rather than 64.84 — a real +1.8 mm, several times the standard error, from the
body settling differently on sprawled legs. So **the offset carries roughly ±2 mm of pose
uncertainty**, which is larger than the sensor's own 1.5 mm noise and is the dominant error term
in this channel. Re-fit with the belly deliberately flat, and treat sub-2 mm clearance readings
as "down" rather than as a number.

**Anchoring at belly-down is the deliberate choice, not a convenience.** It folds the sensor's
own bias into the offset (§9.7: the part reads 8–11 mm long against a physical ruler), and
in exchange it puts the calibration exactly at the dangerous end of the channel. A belly
sensor that is honest at 0 mm and slightly optimistic at 50 mm is the right trade; the reverse
is not.

⚠ **The value currently lives in the systemd unit**, as `--tof-offset 64.8` on
`ExecStart`. That is where `benchd` can read it today and it is the wrong home — a fitted
constant invisible to anyone reading this file. `r_shunt` has the identical problem (§3.3 says
it belongs in calib JSON and it is also a flag). **Both should move to `pi_host/calib/` together.**

### 9.3 Two-point validation — the standing pose, ✅ MEASURED 2026-09-07

A one-point fit calibrates but cannot be wrong-checked. The second point is the saved `stand`
pose, in which **the upper leg is horizontal to the ground** — which is the geometry doc's own
reference stance, so the belly height is predictable rather than merely comparable.

| | belly clearance | sd | invalid |
|---|---|---|---|
| belly down | **0.4 mm** | 0.69 | 0/28 |
| standing (`stand` pose) | **52.1 mm** | 1.27 | 0/58 |
| **through the move itself** | — | — | **0/107** |

Landed pulse matched the target on all 12 channels (the §3.5.1 deadman trap was fed
throughout). Peak **1.731 A**, minimum pack **7.72 V** — comfortable against the 3 A rail.

**Reconciling 52.1 mm with CAD's 56.3 mm.** With the upper leg horizontal the knee axis sits
at hip2 height, so belly = `L3·cos(shin angle) − 19 mm`. Using the **measured** `L3` = 76.5 mm
(geometry §"measured vs CAD"):

| shin angle from vertical | predicted belly |
|---|---|
| 0° | 57.5 mm |
| **10°** | **56.3 mm** — reproduces the figure the geometry doc quotes |
| 21.7° | **52.1 mm** — what was measured |

So the observed clearance implies a shin about **12° further from vertical** than the CAD
reference — real splay under load, plus a hand-saved `stand` pose that is not the CAD nominal.
The trace supports it: the belly hit **54.2 mm** the instant the servos reached position and
then relaxed to 51–53 mm, which is ~2 mm of the body settling onto its own legs.

**This is a stance result, not a sensor error.** Sensor bias runs the other way — the part
reads long, which would push standing *higher*.

⚠ **Standing repeats worse than belly-down.** Three separate recalls of the same `stand`
pose measured **52.1 / 48.7 / 50.8 mm** — a 3.4 mm spread, against ±2 mm for belly-down (§9.2)
and ±1 mm for the sensor itself (§9.7). The legs do not land identically. **So the channel's
absolute accuracy in use is set by how repeatably the body settles, not by the sensor**, and
any threshold on standing clearance needs to carry that 3 mm, not the sensor's 1 mm.

⚠ **Consequence for sim2real.** `picrawler_body.gd` normalizes on `GROUND_CLEARANCE_STAND =
0.06`. The real robot standing is 52.1 mm, so real **`gc_norm` at stand is 0.87, not 1.0**.
Anything tuned against a sim whose belly channel saturates at 1.0 meets a real one that never
reaches it.

### 9.4 What the channel publishes, and why it is more than millimetres

Every measurement carries its range status, signal rate, ambient rate and effective SPAD count,
and `benchd` republishes all of it beside the distance.

**A ToF reading that failed the part's own checks is not a large number or a small one — it is
an arbitrary one, and at a consumer it is indistinguishable from a good reading.** The part
computes the status for free. The slow metrics follow from the same argument:

| field | what it is for |
|---|---|
| `m_ema` | 30 s mean — how high the body is riding |
| `m_min` | 60 s decaying **min**-hold. A *min*, because on this channel the dangerous end is LOW; the peak-hold that serves current (§3.10) would faithfully report the safe extreme |
| `bad_frac` | 30 s EMA of the rate at which the part rejects its own readings — the channel's honesty meter. A ToF wedged at a plausible number with a 90 % invalid rate reads as a healthy belly on distance alone |
| `age_ms` | time since a measurement actually landed. Without it a part that stops ranging shows up as a very steady number |

An invalid reading is published as **`max_range_m`, not zero**, matching `Ultrasonic` (§7):
nothing came back, so the far limit is the honest floor. Zero would map "saw nothing" onto
"something against the belly" — the opposite extreme, and the one the homeostat reacts hardest to.

⚠ **`Valid` here is weaker than ST's full API's valid.** The driver decodes the branches of
ST's status mapping that depend on the device code alone; the full API also raises a sigma
failure from an estimate this driver does not carry. Seen once on the bench: device code 11
("valid") on a 0 mm reading with signal 0.64 Mcps against 4.32 ambient. **This is why the rates
are published** — a consumer can be stricter than the device code, and a collapsing
signal-to-ambient ratio moves before the status flips.

### 9.5 What this does NOT establish

1. **The cone is clear of the legs — partially proven.** §9.1 predicts that a leg entering the
   spot returns as floor and reads as belly-down. ✅ **MEASURED 2026-09-07, negative result:**
   with the belly grounded (so the body cannot fall and the sensor still sees the floor at its
   full standoff), every hip1 and every hip2 was swept across its **full operating range**, one
   at a time, on all four legs — 72 positions in total.

   | axis | worst deviation from the resting baseline | invalid | signal |
   |---|---|---|---|
   | hip1 (horizontal swing) | **−0.6 to +2.5 mm** — inside the sensor's own noise | 0/98 | 24.1–25.2 Mcps |
   | hip2 (femur lift) | **+3 to +9 mm, all POSITIVE** | 0/97 | 23.7–25.0 Mcps |

   **The sign is the finding.** An object entering the beam can only shorten the reading, and
   nothing shortened it. The hip2 excursions are all *longer*, which is the body being levered
   up as a leg pushes on the floor — the ToF incidentally working as a tilt detector. So the
   boom's placement clears the legs on both axes.

   ✅ **Extended to the STANDING pose 2026-09-07, also negative.** The operator's concern was
   that a leg could pass under the sensor once the body is up — which the geometry makes
   plausible: standing, the femur sits ~37 mm below the sensor, where the cone has narrowed to
   only ~16 mm across, so clearance there is decided by boom placement rather than by margin.
   Run in two stages, safest first:

   | configuration | sweep | worst deviation | invalid |
   |---|---|---|---|
   | standing, **feet planted** | coxa ±300 µs, 4 legs | −0.5 to +3.7 mm | 0/180 |
   | standing, **leg lifted** (knee tucked +350 µs) | coxa **full range**, 4 legs | −1.5 to +3.2 mm | 0/220 |

   The lift is what makes the second row meaningful: unloading the foot allows the full coxa
   range without stalling a planted leg against the floor, and it is the gait-like case. The
   tuck lowered the belly by 2.5–4.8 mm on three of the four legs, which is that corner
   unloading and the body settling — confirmation the foot actually left the floor. Current
   stayed 0.57–0.89 A throughout with no stall, pack flat at 7.83–7.91 V.

   **An intrusion here would have been unmissable rather than subtle**: a femur crossing the
   beam sits ~37 mm below the sensor, so clearance would collapse toward zero, not drift.
   Nothing of the kind appeared at any of 44 coxa positions.

   **So there is nothing to map around** — no servo range needs restricting, which was the
   option worth avoiding anyway (see §9.5.1).

   ⚠ **What is still untested is COMBINATION, not range.** Every sweep moved one joint at a
   time. A gait swings coxa, hip2 and knee together, and that space is not covered by the
   union of single-axis sweeps. The evidence is strong that the boom is simply not over the
   swing arc, but it is evidence from 4 configurations, not a proof over all of them.

### 9.5.1 If a leg ever does occlude the beam, restricting its travel is the LAST option

Recorded now, while it is cheap, because the instinct when an instrument gets occluded is to
constrain the body around it — and that is backwards here (CLAUDE.md §1: imposed constraints
fight the loop they ride on). In order:

1. **Move the boom.** The occluding volume is a ~16 mm-wide cone at femur height. A centimetre
   of relocation likely clears it, and costs nothing at the control layer.
2. **Publish the confound rather than prevent it.** `benchd` already holds every joint angle,
   so it can flag "leg in cone" beside the reading exactly as it already flags `bad_frac` and
   `age_ms`, and a consumer discounts those samples. This keeps the leg's full range and makes
   the channel honest about when it cannot see — the pattern the rest of this sensor follows.
3. **Restrict the servo range** — only if the boom cannot move *and* the occlusion is wide
   enough that flagging it would blind the channel too often. This buys instrument cleanliness
   with permanent body capability, which is the wrong direction to trade.
2. ~~One surface only.~~ ⚠ **PARTIALLY RESOLVED 2026-09-08 — see §9.8, and the answer was
   not the one expected.** The optical worry was unfounded on the surfaces tried; the surface
   that actually took the robot down did so *mechanically*. Still only two surfaces, both easy.
3. ~~Two points, both static.~~ ✅ **RESOLVED 2026-09-07 — see §9.7.** Linearity is now
   characterized across 1.5–90.5 mm, past the top of the operating band, at ±1 mm.
4. **Not fed to anything.** Instrument only. Nothing in `benchd` or the brain consumes belly
   clearance yet; `gc_raw` in the sim is still a raycast.

### 9.6 Bring-up notes worth keeping

- **The part boots unable to range.** Reference-SPAD selection out of the die's own NVM, ~80
  tuning register writes ST publishes only as an opaque blob, and two reference calibrations.
- ⚠ **`init()` must soft-reset first.** The stop variable at `0x91` is per-die and only valid
  as read after a fresh boot, and `stop_continuous()` writes **zero** to it. So a second
  `init()` reads `0x00`, replays it, and ranges wrongly — the part returned a correct 124 mm on
  its first-ever init and then nothing but out-of-range on every init after, aimed at the same
  target throughout. Metered on this die: **`0x3c` after a reset, `0x00` after a stop.**
- **Timing budget** 33 000 µs requested reads back **32 908 µs** — the mclks encoding's own
  quantization, and a useful sign the budget arithmetic round-trips against real registers.
  Continuous back-to-back gives **~32 Hz** against `benchd`'s 10 Hz poll, so a fresh sample is
  always waiting.
- **Cost to the servo loop**, A/B against the pre-ToF binary on the same idle robot, 120
  telemetry frames each, 20 ms tick budget: worst tick per 25-tick window went from a **3.5 %**
  median (20.2 % tail) to **4.5–5.2 %** (14.8–26.7 % across two runs) — roughly +0.2–0.35 ms on
  the one tick per telemetry frame that queues behind the bus mutex. **Zero overruns in every
  run, tick flat at 50.00 Hz.** The tail is too noisy to call from single runs.
- ⚠ **`hat_tool tof` re-inits the part**, which soft-resets it underneath a running `benchd`.
  Read the daemon's telemetry instead of running the bench tool while it is up — the same
  caution as §3.10.2.

### 9.7 Linearity across the working range — ✅ MEASURED 2026-09-07

**Method.** The channel operates at raw **65–121 mm** (belly-down to standing), so the way to
sweep it is to *raise* the robot, not to put objects under the sensor — belly-down is already
the closest the part ever gets, and sliding shims beneath it tests a range the channel never
uses. The robot sat in the `rescue` pose throughout, on blocks used as platforms, with the beam
clearing the block edge to the surface below. One rolling capture off the live telemetry, then
plateaus segmented from the step changes.

Truth = block thickness + **1.5 mm**, the gap `rescue` holds between the belly and whatever it
rests on (operator, measured).

| point | truth | measured | predicted | residual |
|---|---|---|---|---|
| bare surface (start) | 1.5 mm | 0.50 | 1.53 | **−1.03** |
| 59 mm block | 60.5 mm | 61.84 | 61.15 | **+0.69** |
| 89 mm block | 90.5 mm | 91.01 | 91.47 | **−0.46** |
| bare surface (end) | 1.5 mm | 2.32 | 1.53 | **+0.79** |
| ~~19 mm block~~ | ~~20.5 mm~~ | ~~25.51~~ | — | **dropped, §9.7.1** |

```
belly = 1.0106 · h + 0.01 mm        scale error +1.06 %
```

**0 invalid in 1315 samples** spanning raw 64 → 156 mm, and **sd flat at 1.47–1.64 mm at every
height** — noise does not grow with distance across this band. Maximum error over the 0–52 mm
operating band: **0.56 mm**.

Two results fall out of this that were not what it was measuring:

- **The intercept is +0.01 mm, which independently confirms the mount offset.** An error in the
  64.8 mm fit of §9.2 would appear here precisely as a nonzero intercept, and this data never
  touched that fit.
- **The two bare-surface plateaus share a truth and differ by 1.82 mm** — an independent
  reproduction of the ±2 mm pose repeatability of §9.2, from a different measurement entirely.

⚠ **Do NOT apply a scale correction.** +1.06 % is at most 0.56 mm across the working band,
which is smaller than the ±2 mm pose term that already dominates. A second calibration constant
dominated by a larger uncorrected one buys nothing but false precision.

### 9.7.1 ⚠ The part reads 8–11 mm LONG against a ruler — and that is not a fault

Two independent comparisons against a physical measurement, at very different distances:

| ruler | sensor | delta |
|---|---|---|
| ~114 mm (bench target) | 121.9 mm | **+7.6 mm** |
| 70 mm (desk to sensor) | 80.6 mm | **+10.5 mm** |

This is the VL53L0X's **inherent ranging offset**: the distance it reports is referenced to an
internal plane, not to the visible front face of the module. It is what ST's offset-calibration
procedure exists to remove, and it is why it stayed invisible until now — **the block sweep
measures DIFFERENCES, and a constant offset cancels in a slope.** Hence the +1.06 % scale with a
zero intercept while an absolute ruler check is out by ~9 mm.

Two consequences, both of which will otherwise be read as broken hardware:

1. **`mount_offset_mm = 64.8` is a READING, not a physical distance.** The true standoff from
   the optical face to the belly plane is ~9 mm less, about **56 mm**. Anyone who calipers the
   boom will measure ~56 mm and conclude the calibration is wrong. It is not — anchoring at
   belly-down absorbs the inherent offset by construction, which is the whole reason for
   anchoring there instead of deriving the standoff from CAD.
2. **A tape-measure check will always read ~9 mm long.** That is the documented expectation.

### 9.7.2 The 19 mm point, and why it was dropped

Recorded rather than silently omitted, because a discarded datapoint deserves the same
treatment as a refuted lever: **it was dropped for a reason found independently of the fit, not
for disagreeing with it.**

It came in **+4.78 mm** high while every other point sat inside ±1 mm. Two placement faults,
both identified by the operator at the bench, not inferred from the residual:

- **The small block does not span the chassis bottom screws** the way the larger blocks do, so
  it contacts the belly plate directly rather than the screw heads — the resting height it
  produces is not `1.5 + 19`.
- **During the sweep the sensor sat near the desk edge**, where the beam cleared the surface
  entirely. Re-measuring the same nominal setup gave **127.4 mm** — a confident, zero-invalid
  reading of the floor beyond the edge rather than the desk.

Re-placed centred and not leaning it read **80.6 mm**, 10 mm from the sweep's 90.3 mm for the
nominally identical configuration. **The placement moved, not the sensor.** A point whose truth
value cannot be stated is not evidence either way, so it is excluded — and the linearity result
rests on the three whose geometry was unambiguous.

⚠ **The general lesson is about the beam, not the block.** A ToF near an edge returns a
confident, valid, low-noise reading of whatever is beyond it. Nothing in the status, the signal
rate or the invalid count flagged the 127 mm reading as wrong, because *it was not wrong* — it
was an honest answer about a different surface. **Only knowing where the beam lands makes the
number mean anything**, which is the same reason §9.5 still wants a leg sweep at the standing pose.

---

## 9.8 Surface dependence — ✅ MEASURED 2026-09-08, and the threat was mechanical

**Method (operator's).** Make the robot do **pushups** — a joint-space lerp between `rescue`
and `stand`, 12 steps each way — so the same sweep of belly heights is replayed on each
surface and readings compare at *matched poses*. No IK needed.

⚠ **Read the OPTICS, not the height.** A surface changes both the return *and* the mechanics
(compressibility, foot grip), so belly height at a matched pose is confounded and was not used
as the comparison. `signal`, `ambient` and the effective SPAD count are surface properties at a
given distance and are not.

### 9.8.1 The finding that matters: signal rate is the WRONG metric

| alpha | bare vinyl: belly / sig / spads | black cloth: belly / sig / spads |
|---|---|---|
| 0.00 | 1.4 mm · 24.3 · **13.0** | 0.0 mm · 26.7 · **6.0** |
| 0.25 | 8.6 mm · 24.9 · **17.0** | 0.0 mm · 27.2 · **8.0** |
| 0.50 | 17.0 mm · 23.9 · **21.6** | 11.0 mm · 24.5 · **13.8** |
| 0.75 | 32.4 mm · 24.0 · **35.0** | 29.0 mm · 24.2 · **23.0** |
| 1.00 | 50.5 mm · 23.4 · **49.8** | 46.6 mm · 24.1 · **36.8** |
| median | sig 24.18 · spads 21.7 · 0/128 invalid | sig 24.48 · spads 13.4 · 0/125 invalid |

**Signal rate is flat at ~24 Mcps on both surfaces at every distance — because the part
REGULATES it.** It holds the return constant by recruiting more SPADs, so `signal_mcps` looks
identical on a good surface and a bad one and reports almost nothing about either. The obvious
thing to watch is the wrong thing to watch.

**The effective SPAD count is the surface metric.** It rises with distance (13 → 50 on vinyl,
6 → 37 on cloth) and is consistently **~40 % lower on black cloth**, which means the cloth is
the *better* 940 nm reflector of the two. Which is the second lesson:

⚠ **"Black" to the eye says nothing about 940 nm.** Most fabric dyes absorb across the visible
band and reflect near-IR perfectly well. The intuitive worst-case test surface turned out to be
the easier one, and a genuinely hostile surface would be one with low reflectance *at 940 nm* —
not one that looks dark.

**Headroom, and how to read it:** SPAD count at the TOP of the range is the number to watch.
Bare vinyl needs ~50 at 50 mm of clearance. A surface roughly twice as poor would need ~100,
and that is where the regulation runs out and `signal` finally starts to fall. **So the early
warning is a rising SPAD count, and it moves long before `bad_frac` does.**

### 9.8.2 ⚠ The surface that broke the robot broke it MECHANICALLY

The first attempt ran on **black rubber** and **took the machine down**: a foot caught on the
surface, stalled, and the Pi went. Operator's diagnosis, and it is §3.9's mechanism exactly —
*"grip converts free motion into work and load torque then sets the current regardless of
speed"* — plus §3.9.2, where a stall is what no duty budget prevents. The same pushups on vinyl
peaked at **0.73–0.77 A**.

**So the answer to "is this channel surface-dependent" is: less than feared optically, more
than feared mechanically.** The optical worry that motivated the test found nothing on either
surface; the surface property that mattered was friction, and it cost a shutdown.

Two process failures came with it, both already predicted by this document and both repeated
anyway:

- **The capture was written to `/tmp`**, which is tmpfs, and the reboot cleared it (§3.10).
  The baseline had to be re-run.
- **The crash destroyed its own evidence again.** The last surviving `benchd` record was from
  *before* the run — 0.52 A, 7.95 V, belly ~0 mm — and the final line is nulls. The measurement
  that would have shown the stall current died with the machine that made it. What the session
  did keep: peak 1.879 A over 99 minutes and pack never below 7.43 V, so this was **not** pack
  sag — consistent with §3.8.1, where the cliff is the 5 V regulator and the INA219 cannot see it.

### 9.8.3 Why the record now lives off-board — `tools/tele_record.py`

The obvious fix was to `fsync()` `benchd`'s record. **It was tried and measured, and it is not
available:**

| | worst tick / window (median) | worst tick (max) | overruns / 12 s | tick_hz |
|---|---|---|---|---|
| without | 3.4–5.2 % | 12.9–26.7 % | 0 | 50.00 flat |
| **fsync at 1 Hz** | **7.7 %** | **418.9 %** (~84 ms) | **54** | **min 35.8** |

`record()` runs under the mutex the 50 Hz servo tick needs, and an SD fsync costs ~80 ms.
**Durability bought with the control loop is not a trade this daemon may make**, and a longer
cadence only makes the stall rarer, not smaller. Reverted, with the numbers left in the source
so it is not retried.

**The operator's answer was better than the one being built: record on another machine.**
`benchd` already publishes every frame at 10 Hz, so a subscriber on the PC writes to a disk
that cannot share the fate of the thing that browns out — no fsync, no page cache, no SD card,
no extra thread on the robot, and zero cost to the loop.

```sh
python3 pi_host/tools/tele_record.py --host picrawler.local --out . --label <run>
```

⚠ **What it still cannot catch, stated so nobody over-trusts it.** It records what *arrived*.
A hard power cut still loses whatever sat in the Pi's TCP buffer (`benchd` sets ZMQ `SNDHWM` 4),
so expect to lose the last *frames* rather than the last *seconds*. And **a network drop is not
the robot going quiet**: `seq` is monotonic from the daemon, so every gap is detected and
written as an explicit `gap` record rather than left as a silent hole a reader would mistake
for a still robot. The bare-vinyl run recorded 655 frames with 0 gaps.

### 9.8.4 What this does NOT establish

1. **Two surfaces, both easy.** Vinyl and black cloth. Untested: carpet (scatters *and*
   compresses), gloss at a tilt (specular return steered away from the receiver, the case most
   likely to produce genuine invalids), and anything genuinely low-reflectance at 940 nm.
2. **Neither surface produced a single invalid reading**, so `bad_frac` has still never been
   exercised in anger. The channel's honesty meter remains untested against a surface that
   actually defeats it.
3. **Rubber was never measured optically** — the run died before producing data, and the reason
   was friction, not optics.
