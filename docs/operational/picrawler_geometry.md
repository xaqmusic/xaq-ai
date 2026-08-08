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

Mass values (from real PiCrawler hardware specs, to fill once we have them):

- 9g servo: 9 g each, 12 servos = 108 g
- Chassis (Pi 5 + battery + base plate): ≈ 400 g (estimated)
- Leg segments: thin metal, ≈ 5–10 g per segment
- Total expected: ≈ 500–600 g
