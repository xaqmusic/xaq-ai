# Microduck: what the ToF sensor can tell the duck about objects

Status: measured · Date: 2026-09-12 · Branch: `duck-l2` · Simulation only

*Three studies the operator asked for after the walk-stop-look line's verdicts: an EPM on the
sensor's full output with its PCA visible, several EPMs in different roles off the same sensor, and
the point cloud a head babble builds. Run before any of the proposed behaviours, on the principle
that saved the speed question — measure that the signal exists before designing what rides it.*

**Report and figures:** <https://claude.ai/code/artifact/468b1fff-ed27-483b-86a6-ca87d7c5459a>
**Verdicts:** [`microduck/rung2_regime_design.md`](rung2_regime_design.md) §17.28.
**Owning register rows:** O36, O38, O40, O41.
**Corrections (2026-09-13, §17.30):** this study's de-rotation was applied with the wrong sign — its
"+7 % distinct voxels" was smear and the de-rotated cloud's 45 % detection is withdrawn (41 %, the same
as without de-rotation); and the break profile classified voxels by their centre, so 82 % of what it
called floor breaks was bare ground. `CloudMap` is now an ogma module with both fixed, a unit test that
pins the sign, and a cache keyed by place.
**Follow-through:** design doc §17.29–17.30 — the cloud moved into the host (`CloudMap`), aiming the gaze came out `NULL` (and closed O36 by measurement), an EPM over the cloud gave the best vocabulary yet, and the body predictor gave a real contact channel.

---

## 1. The instrument, and what we were using of it

A VL53L8CX on the head: 8×8 zones, 45° field, cast every 4 ticks (12.5 Hz), ported in
`mj_host/src/Tof.{hpp,cpp}` with MuJoCo ray queries against world geometry only. Zone *i* is row
`i/8` (row 0 the **top**, +19.6875°, 5.625° per row) and column `i%8` (column 0 the **left**).

What the brain saw before these studies: four proximity numbers (`Tof::summary`) and, for the place
map, eight column minima (`Tof::column_hit`). Everything else the sensor returns — vertical
structure, the floor plane it already classifies, per-zone return points — was computed and dropped.

**Two host additions, both instruments, both gated so every earlier log stays byte-comparable:**

| flag | field | why |
|---|---|---|
| `--log-motor-tle` | `mtle` | the twist brain's own forward-model surprise per tick — the only body-error channel live while the walker drives |
| `--log-tof-cloud` | `tofp` | `[[zone, x, y, z], …]` for every Hit/Floor zone on a cast tick |

And one correction to the sensor itself: `TofZone::point_level` — the return in the
**gravity-levelled** trunk frame. `point` was in the raw trunk frame, where three degrees of body
tilt is ten centimetres of apparent height at two metres, against a four-centimetre block. The
levelling moved ~5 % of returns out of the "low object" height band, so it was not cosmetic.

## 2. How to re-run

```sh
# 1. a dataset (add --roll-past 6 0.8 for the labelled-change run)
mj_host/build/ogma_mjhost --level2 models/microduck/scene_playroom.xml \
  --graph configs/a1v2_r43_w3c_learn.json --secs 1500 --seed 6 --noise 0.05 \
  <the W3c stop/gaze args> --log-motor-tle --log-tof-cloud | python3 mj_host/tools/tof_reduce.py out.jsonl

# 2. arrays + the god's-eye labels (instrumentation only)
python3 mj_host/tools/tof_prep.py out.jsonl <scene>.manifest.json out.npz

# 3. the go/no-go checks: can it see them, is a stumble visible
python3 mj_host/tools/tof_m12.py out.npz

# 4. the REAL EPM over the recorded frames, several views at once
cpp_core/build/epm_tof_study --frames out.jsonl --out epm.jsonl \
  --epm cols8:cols8:rbf --epm full64_jl:full64:jl_state --epm full64_dm:full64_dm:jl_state \
  --epm heights8:heights8:jl_state --epm geom_shape:geom_shape:jl_state
python3 mj_host/tools/tof_studies.py out.npz epm.jsonl s12.json

# 5. the point cloud: what a sweep buys, and change detection
python3 mj_host/tools/tof_cloud.py out.npz
```

`epm_tof_study` drives the **shipped EPM module** (CLAUDE.md §0 rule 1 — no stand-in clusterer), one
topic per configured arm, and dedupes consecutive identical frames so the GNG sees the sensor's own
12.5 Hz rather than every frame four times. Its views are the study's design surface:

| view | dims | what it is |
|---|---|---|
| `full64` | 64 | the zone ranges / 4 m, Empty → 1.0 — the sensor as it is |
| `cols8` | 8 | the nearest Hit per column — today's map input |
| `full64_dm` | 64 | the frame's own mean removed (§0 rule 2's prescription) |
| `full64_norm` | 64 | each frame on its own maximum |
| `heights8` | 8 | per column, the height above the floor of its nearest non-floor return |
| `geom` | 20 | heights + ranges + floor-break fraction, extent, edge count, mean height |
| `geom_shape` | 12 | `geom` with every absolute range dropped — distance-free |

## 3. What the sensor throws away, for the next person who needs more

Measured or directly computable from what `Tof` already has, and unused by any live config:

1. **Vertical structure.** Column minima collapse eight rows to one number. A block breaks the floor
   in the lower rows while the upper rows still see the far wall; a wall occupies every row at one
   range.
2. **The floor-break map.** `Tof::sense` already computes the expected floor distance for its `Floor`
   class. Actual minus expected, per zone, is where something stands on the ground. Arithmetic.
3. **Height above the floor** of every return, from `point_level` plus the trunk's own height.
4. **Silhouette and edges** — the column-wise range gradient: two edges is a thing, none is a wall,
   and the span between them is its width. (R44's two-column rule is a crude version of this.)
5. **Per-zone range derivative** with the head's own yaw rate subtracted: what moved that I did not
   move.
6. **Parallax from the saccade the duck already performs** — a near object's angular position shifts
   much faster than a far wall's. Free near/far segmentation from a motion it already makes.

All six are frozen geometric reduction — legitimate encoder, in §0's sense. The vocabulary over
them stays the GNG's to earn.

## 4. Open follow-ups this study names

All four were taken up immediately; §17.29 has the results. In short:

- **Done** — the EPM over cloud-derived features: `CloudMap::break_profile` on
  `reality.proprio.cloud_in`, and the best vocabulary this duck has had (16 nodes, all baked, 7
  winner switches a minute). Its *semantics* remain open: the cloud only exists while standing, so
  one run gives one pose per object.
- **Done** — de-rotation is in the host (`--cloud`), gain-0 guarded.
- **Wrong** — the gaze was not the constraint. Widening the pitch babble and biasing it downward both
  come out `NULL`, and the geometry says why: at sd 0.2 the gaze already reaches 23° down, which
  looks at the floor by the duck's feet. O36 closed by measurement on the way (65/65 stands at the
  amplitude R42 measured as a regression).
- **Done, partly** — the joint brain now predicts through the walk (`--body-predicts`), giving a
  contact channel 2.7× the twist brain's. The instantaneous residual is still the open fix.
- **Still open, and now the only thing in the way** — travel. Three independent measurements reduce
  to it.
