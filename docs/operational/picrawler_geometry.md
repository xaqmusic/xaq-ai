> **Ported from the pre-split `ami-ogma` repo, 2026-07-25.** This is **physical ground truth**
> about the PiCrawler body (CAD/STEP-derived) and does not go stale with the software era —
> it is the reference for sim-to-real fidelity. The `ami_ogma`/`ogma` naming throughout means
> **xaq** (see [`../../AGENTS.md`](../../AGENTS.md)). Companion: [`../servo_dynamics.md`](../servo_dynamics.md).

# PiCrawler geometry — extracted from STEP + Blender measurements

Source: the PiCrawler kit's `picrawler_3d model.STEP` CAD export (~78MB,
SolidWorks 2023, 293 solids; not included in this repo). Parsed via
`scripts/extract_picrawler_solids.py` (pythonocc-core). STEP CAF reader
crashed on the file (likely CJK part names); fallback to the basic
STEPControl reader gave geometry but no assembly tree.

## Coordinate system

- **Z = up** (vertical)
- X = side-to-side (chassis width direction)
- Y = front-to-back
- Model is in **standing pose**: hip servos at neutral (legs splayed
  horizontally), knees angled down so lower leg drops vertically to floor.
- Units: millimetres.

## Confirmed from STEP (auto-extracted)

### Chassis

- **Bottom plate**: ≈ 98 × 98 × 25 mm at centroid (+23.8, −34.0, +95.5)
- **Top plate / battery**: ≈ 93 × 99 × 31 mm at centroid (+23.9, −34.0, +123.4)
- Total chassis envelope (top to bottom of structural plates):
  Z = 96 → 138 mm (height ≈ 42 mm of structural body + battery on top)
- Chassis-center XY: ≈ (+23.8, −34.0)

### Hip-attach pattern (4-fold confirmed)

Four hip-servo bodies at the chassis corners, all at the same Z:

| Leg label (proposed) | X    | Y    | Z   |
|----------------------|------|------|-----|
| front-left  (FL)     | −14.5| −1.0 | 109 |
| rear-left   (RL)     | −14.5| −67.0| 109 |
| front-right (FR)     | +62.1| −1.0 | 109 |
| rear-right  (RR)     | +62.1| −67.0| 109 |

- Rectangle: **76 mm (X span) × 66 mm (Y span)**, centered on (+23.8, −34.0).
- Hip servo body bbox: 12 × 31 × 33 mm (matches 9g hobby servo dimensions).
- The 31 / 33 dims are the body case; the 12 dim is the case thickness.

### Reach (one leg, observed in storage-flat verts)

- Hip corner (RR) at (+62, −67); a foot-region candidate at (+106, −149).
- Straight-line distance ≈ 93 mm.  This sits between
  `UPPER_LEG_LEN + LOWER_LEG_LEN` and the actual standing reach
  (which depends on knee angle).

## Kinematic chain (3 servos per leg, confirmed by user)

Top-down chain for one leg:

```
chassis
  └── hip1 servo  (mounted in chassis corner; rotation axis = WORLD Z)
       └── coxa segment, length L1
            └── hip2 servo  (rotation axis = leg's LOCAL LATERAL,
                             perpendicular to leg outward dir, horizontal)
                 └── upper-leg segment, length L2
                      └── knee servo  (rotation axis = same as hip2,
                                       leg's LOCAL LATERAL)
                           └── lower-leg segment, length L3
                                └── foot tip (toe contact point)
```

Sizes — measured directly in Blender, axis-to-axis (overrides any STEP
centroid-derived estimates):

| Symbol | Description | Value |
|--------|-------------|-------|
| `L1` | hip1 axis → hip2 axis (coxa segment) | **37.26 mm** |
| `L2` | hip2 axis → knee axis (upper leg) | **53.6 mm** |
| `L3` | knee axis → toe tip (lower leg + foot) | **75.5 mm** |
| `HIP1_AXIS` | hip1 servo rotation axis | **world +Z** |
| `HIP2_AXIS` | hip2 servo rotation axis | **leg-local lateral** |
| `KNEE_AXIS` | knee servo rotation axis | **leg-local lateral** |

Total straight-line leg reach: L1 + L2 + L3 = 166.4 mm
(actual standing height is less because hip2 + knee are bent).

## Standing pose (joint angles for the rest pose in Godot)

| Joint | Angle | Effect |
|-------|-------|--------|
| `hip1` (per leg) | 0 (neutral) — leg splayed at corner-outward direction | Legs form an X shape, splayed horizontally |
| `hip2` (per leg) | 0 (neutral) — upper leg PARALLEL TO GROUND | Upper leg sticks out horizontally |
| `knee` (per leg) | −80° (bent down) — lower leg nearly vertical | Toe points down, ~10° from vertical |

Standing height calculation:
- Chassis at Z=109 mm (from STEP)
- hip2 axis Z=102 mm (7 mm coxa drop)
- knee axis Z=102 mm (upper leg horizontal)
- Toe Z = 102 − 75.5·cos(10°) = 102 − 74.4 = 27.6 mm
- Floor at Z=27 mm (per STEP minimum vertex)
- → Chassis-bottom-to-floor: ~82 mm.  Standing PiCrawler is ~10 cm tall.

### Hip pivot convention

The "hip pivot" is the geometric center of the hip servo's output shaft —
the axis the upper leg rotates around when the hip servo moves.  Pick the
center of the 31 × 33 face that has the shaft sticking out.

### Knee pivot convention

Same idea — center of the knee servo's output shaft.

## What to do with this once filled

Replace the spider-mode placeholders in
`godot_host/project/scripts/quadruped_body.gd`:

```gdscript
const UPPER_LEG_LEN: float = ...  # from UPPER_LEG_LEN_MM / 1000
const LOWER_LEG_LEN: float = ...  # from LOWER_LEG_LEN_MM / 1000
const CHASSIS_LENGTH: float = 0.098  # from STEP bottom plate
const CHASSIS_WIDTH:  float = 0.076  # from hip rectangle (X span / 1000)
# Hip anchors per leg, in chassis-local space:
const HIP_OFFSETS = [
    Vector3(-0.038, +0.033, 0.0),  # FL
    Vector3(-0.038, -0.033, 0.0),  # RL
    Vector3(+0.038, +0.033, 0.0),  # FR
    Vector3(+0.038, -0.033, 0.0),  # RR
]
```

Joint orientations come from `HIP_AXIS`, `KNEE_AXIS`, `COXA_AXIS`.

Mass values — **superseded, see §"Measured from the built robot" below.** Original estimate:

- 9g servo: 9 g each, 12 servos = 108 g
- Chassis (Pi 5 + battery + base plate): ≈ 400 g (estimated)
- Leg segments: thin metal, ≈ 5–10 g per segment
- Total expected: ≈ 500–600 g

---

# Measured from the built robot — 2026-08-10

> **These are tape-measure/scale readings from the assembled physical PiCrawler now sitting on
> the operator's bench.** Where they conflict with the STEP/CAD numbers above, **these win** —
> CAD describes the kit as designed, this describes the robot as built. Recorded by the
> operator; two items still need disambiguation (§"Open questions" below).

## Link lengths

| Quantity | Const | CAD / sim (current) | **Measured** | Δ |
|---|---|---|---|---|
| toe tip → knee axis | `L3` | 75.5 mm | **76.5 mm** | +1.0 mm (+1.3 %) |
| knee axis → hip2 axis | `L2` | 53.6 mm | **48.0 mm** | **−5.6 mm (−10.4 %)** |
| hip2 axis → hip1 axis | `L1` | 37.26 mm | **32.0 mm** | **−5.26 mm (−14.1 %)** |
| total straight-line reach | `L1+L2+L3` | 166.36 mm | **156.5 mm** | **−9.86 mm (−5.9 %)** |

**The two proximal segments are both ~5.5 mm shorter than CAD; only the distal segment matches.**
The error is not a uniform scale factor — it is concentrated in the coxa and upper leg.

## Hip mount pattern

| Quantity | Const | CAD / sim | **Measured** | Δ |
|---|---|---|---|---|
| hip1 lateral span | `HIP_X_SPAN` | 76.6 mm | **76 mm** | −0.6 mm |
| hip1 fore-aft span | `HIP_Z_SPAN` | 66.0 mm | **76 mm** | **+10 mm (+15.2 %)** |
| pattern shape | — | rectangle 76.6 × 66 | **square 76 × 76** | — |

Hip1 anchors in chassis-local coordinates therefore sit at **(±38, ±38) mm**, not (±38, ±33).
**This widens the fore-aft support polygon by 15 %** — directly relevant to any stance/support
work, since the polygon is what a support criterion is computed over.

## Vertical stack-up and mass

| Quantity | Const | CAD / sim | **Measured** | Note |
|---|---|---|---|---|
| chassis below hip2 | — | *(none — CAD puts hip1 7 mm **above** hip2)* | **19 mm below** | **new; see open question 2** |
| CoG above hip2 | — | *(emergent)* | **≈ 15 mm**, chassis centre | **new — validation target** |
| total mass | `_TOTAL_MASS` | 600 g (published spec) | **590 g** | −10 g (−1.7 %) |
| per-leg mass | `COXA+UPPER+LOWER` | 75 g (25+25+25) | **62 g** | **−13 g (−17.3 %)** |
| body/chassis mass | `CHASSIS_MASS` | 300 g | **342 g** (derived: 590 − 4×62) | **+42 g (+14 %)** |
| leg mass fraction | — | 50 % | **42 %** | −8 points |

Two consequences worth carrying:

1. **The legs are still 42 % of the robot.** Leg inertia is not a perturbation on this body;
   any "massless leg" simplification is wrong by a wide margin.
2. **The "chassis is 50 % of body mass" comments are now stale** — `picrawler_body.gd:1605`
   and `:8714` both state it, and the real split is 58/42. The `body_cog` metric's reasoning
   depends on that ratio.

**The measured CoG is the most valuable number here**, because it is the only one that is a
*check* rather than an input: `_body_cog()` (`picrawler_body.gd:8713`) already computes a
mass-weighted CoG from the sim's own segment masses and positions. Feed it the corrected
masses and geometry, and it must land ≈15 mm above hip2 at the standing pose. If it doesn't,
the mass distribution is wrong somewhere the tape measure can't see.

## ⚠ Updating these constants is a re-baseline, not a bug fix

`L1`/`L2`/`L3` are inputs to the FK chain that produces **`feet_y_gravity_cmd`** — the promoted
swing-gate input (see `sensor_legitimacy_and_the_feet_y_oracle.md` §5.6). Changing them moves
that signal, which moves the gait, which moves **every number currently in the ledger**.

Treat the geometry correction as its own lever under the §3 protocol: change geometry alone,
seed-average, and record the new baseline explicitly. Do **not** fold it in alongside another
change — and do not read a post-correction regression as a verdict on any earlier lever.

## Open questions — RESOLVED 2026-08-10 (operator)

1. **76 mm is the SIDE of the hip1 square**, not the diagonal — confirmed, and consistent with
   CAD's independently-derived 76.6 mm X span. Hip1 anchors are at **(±38, ±38) mm**.
2. **The 19 mm references the BOTTOM PLATE underside** — the surface that contacts the floor
   when the robot bottoms out. No conflict with CAD's `COXA_Z_DROP`: hip1's *mounting face* is
   still above hip2, while the plate *underside* is 19 mm below it. **This surface is the belly**,
   and its collision must be accurate (see [[picrawler-ghost-chassis]] — this is precisely the
   surface that was passing through the floor).

## Chassis collision model — operator-specified 2026-08-10

Total chassis height **103 mm**, measured with wires plugged into the HAT, **no protective
shell**. Modelled as two stacked boxes:

| Element | Footprint | Thickness | Span (rel. hip2 axis) |
|---|---|---|---|
| Base structure (plate + hip1 servos) | 98 × 98 mm¹ | **41 mm** | **−19 → +22 mm** |
| Electronics stack (Pi + HAT + wiring) | **90 × 56 mm** | 62 mm² | **+22 → +84 mm** |

¹ carried over from CAD's ≈98 × 98 mm bottom plate — the only element here not directly
  measured; worth a tape check since it sets the belly contact area.
² derived: 103 − 41. The 90 × 56 mm footprint matches a Raspberry Pi (85 × 56 mm) plus margin.

**Reference plane: the hip2 axis, y = 0.** Belly (collision surface) at **−19 mm**; top of
electronics at **+84 mm**; total 103 mm. ✓

### Mass distribution — solved from the measured CoG

**The +15 mm is the CHASSIS ASSEMBLY's CoG** — settled by the operator's layer description,
which is direct observation and outranks any back-solve.

Chassis layers, bottom to top: **bottom plate · top plate · RPi · Robot HAT**, with the
**battery between the plates**, velcro'd to the underside of the top plate. That puts the single
heaviest component (~95 g) **low**, inside the base box — not in the electronics stack. A
layer-by-layer estimate reproduces the measurement independently:

| item | mass | y rel hip2 |
|---|---|---|
| bottom plate | ~40 g | −17.5 |
| 4 × hip1 servos | ~52 g | ~−10 |
| **battery** | ~95 g | ~+12 |
| top plate | ~40 g | +22 |
| RPi | 45 g | ~+30 |
| Robot HAT | 30 g | ~+45 |
| wiring / screws | ~40 g | ~+40 |
| **total** | **342 g** | **CoG +14.9 mm** |

→ box split **base ≈ 252 g, electronics ≈ 90 g** (Pi 45 + HAT 30 + wiring ≈ 90 g for the top box).

**⚠ Two different CoGs — do not conflate them.** The measured +15 mm is the *chassis*. The
**whole-robot** CoG is a *derived* **+3.9 mm** rel hip2, because 248 g of legs hang at ≈ −11.4 mm
in the standing pose. Conflating the two already produced one wrong mass split (a superseded
reading forced the battery into the top stack at ~217 g). `_report_geometry()` now prints **both**
every run so it cannot recur.

**✓ G1 VERIFIED 2026-08-10** — receipt reads `chassis CoG = +15.0 mm rel hip2`,
`whole-body = +3.9 mm`.

**This replaces the flat `CHASSIS_MASS = 0.300` single rigid body** with a two-part mass
distribution that reproduces the measured CoG by construction rather than by assumption.

## Servo envelope vs commanded range vs what the gait actually uses

A 500–2500 µs hobby servo has **~180° of travel, full stop.** Three different spans matter, and
conflating them gives the wrong answer:

| Joint | Sim joint limits | Brain's commanded span | **Measured occupancy**¹ | Fits 180°? |
|---|---|---|---|---|
| `hip1` | ±1.40 rad | 160° | — | ✓ |
| `hip2` | ±1.40 rad | 160° | **29°** (−23…+6) | ✓ easily |
| `knee` | −2.50 → +1.70 rad | **232°** (FOLD 3.20 / HYPEREXT 0.85) | **113°** (−98…+15) | ✓ **in practice** |

¹ arena, n=3, 1032 leg-frames post-warmup — ledger "The sprawl, quantified".

**The commanded knee range (232°) exceeds what any hobby servo can deliver, but the deployed
gait only ever occupies 113° of it** — comfortably inside the servo envelope. So the knee
widening is not a porting blocker, and `knee_widening_enabled = false` (the existing gain-0
toggle restoring a 160° symmetric span) is available but not required. The *limits* are
fictional; the *behaviour* is hardware-legal. Verify occupancy again on the corridor gym before
relying on this — the number above is from the arena.

## ⚠⚠ The real blocker: the gait plants its feet beyond the real leg's reach

The same measurement table records **planted foot radius = 170 mm (range 136–179)** against a
CAD total leg reach of 166 mm — the ledger already flags this as *"the feet plant at the limb's
full reach — straight-legged, maximum moment arm, minimum mechanical advantage."*

**With the measured geometry, total reach is 156.5 mm, not 166.4 mm.**

| | CAD | **Measured** |
|---|---|---|
| total leg reach `L1+L2+L3` | 166.4 mm | **156.5 mm** |
| gait's mean planted foot radius | 170 mm | 170 mm |
| **overshoot** | +3.6 mm | **+13.5 mm** |

The deployed gait stands at full extension of a limb that is now **10 mm shorter**. Those foot
placements are not merely inefficient on the real robot — a large share of them are
**geometrically unreachable**: the leg arrives straight and still short, and the foot lands
early or not at all.

This is the single most consequential item on this page for the port. It is also *not* a new
problem the correction created — the sprawl finding already identified full-extension planting
as a pathology, with `scrub` 0.100 against `fwd_v` 0.050 (the body slides sideways twice as fast
as it advances). **The geometry correction makes an already-diagnosed defect materially worse,
which means fixing the sprawl and porting to hardware are the same work item, not two.**
