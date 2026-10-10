# duck_viewer — watching the Microduck host

**The viewer does not simulate anything.** `ogma_mjhost` writes the full generalized
position on every tick; this reads it, assigns it, and calls `mj_forward` to place the
geometry. Every pose on screen was computed by the host.

That is the whole design decision. A viewer with its own copy of the dynamics is a second
thing to keep in step with the first, and the day they disagree the screen is the convincing
one — which is exactly backwards.

## Setup, once

```sh
tools/duck_viewer/setup.sh          # local venv: mujoco 3.12.0 + imageio
```

Its own venv rather than the repo's, for two reasons: nothing else here needs `mujoco`, and a
viewer should never be able to break a build. The MuJoCo version is pinned to match the host's
C++ pin — the same model has to load the same way on both sides.

## Use

Everything is reachable through [`mj_host/run.sh`](../../mj_host/run.sh), which is the
front door:

```sh
./mj_host/run.sh watch --secs 30                 # live window, run saved to mj_host/log/
./mj_host/run.sh watch --secs 10 --noise 0.05    # perturbed start
./mj_host/run.sh record /tmp/duck.mp4 --secs 8   # video, no window needed
```

Directly, if you want a saved run back:

```sh
tools/duck_viewer/.venv/bin/python tools/duck_viewer/view.py replay mj_host/log/watch-*.jsonl
tools/duck_viewer/.venv/bin/python tools/duck_viewer/view.py replay RUN.jsonl --fast
```

`live` and `replay` open MuJoCo's own passive viewer: drag to orbit, scroll to zoom, space to
pause, and all its usual keys. `live` paces to the wall clock, because the host runs far faster
than real time and a run that flashes past is not an observation.

Three keys are ours, listed in the HUD under the brain-camera image (with the status line):

| key | what | default |
|---|---|---|
| `V` | the ToF's 64 beams from the head, coloured by class | off — they hide the head |
| `C` | the **brain-camera window**: the head camera's frame at the brain's resolution (`--cam-res`, default `64x48`) and rate (12.5 Hz), scaled up without smoothing so each of the brain's pixels is a block; the status line and the keys are its HUD | on |
| `H` | the HUD text | on |
| `W` | **fade what hides the duck** — an outer wall drops to 15 % opacity while the camera is on its far side; a table, chair or shelf drops to 15 % while a bundle of rays from the camera to the duck's trunk hits it first (the table top when looking down through it, a chair in the line of sight); a fade holds half a second past its last hit so a grazing ray does not flicker | on |

**Watching a level-2 run: the first 600 s are the babble.** The duck pulses its twist in place
to identify its own velocity model, and the tour starts at 600 s. `--fast-until 600` on the
viewer (and, in `live` mode, on the host — the launcher sets both) fast-forwards that part: no
pacing, one frame in 25 drawn, the HUD says `[fast-forward]`, and the wall clock starts where the
pacing does. The run is the same run tick for tick — the host's flag only touches its pacer, and
a run watched with `--realtime` is byte-identical to the headless one (checked 2026-09-10).

The HUD is in the camera window rather than drawn into the MuJoCo window on purpose: label
geoms placed in the free camera's frame lag the mouse between syncs and flash on every zoom.

The camera window renders the head camera from the same `qpos` the viewer draws (an offscreen
EGL renderer in the viewer process, handed to Tk as a PPM), so it shows exactly the frame the
host will publish once it renders (playroom plan C1). Until then it is the preview of that
frame, not a copy of it. Scenes without a `head_camera` (the vendored default) get no window.

## Every run is kept

`run.sh watch` and `run.sh hold` write `mj_host/log/<mode>-<timestamp>.jsonl`, one JSON object
per tick. The directory is gitignored. A saved run replays identically forever, which means an
interesting three seconds can be watched again rather than described.

| field | what it is |
|---|---|
| `t`, `tick` | simulated seconds, and the brain tick index |
| `q` | the 14 policy joints, radians, in `JOINT_NAMES`-minus-mouth order |
| `qpos` | full generalized position (21), including the trunk's free joint — what the viewer draws |
| `grav` | projected gravity in the trunk frame; upright is about `[0, 0, -1]` |
| `x`, `y`, `z`, `tilt` | **instrumentation only** — world-frame, and no brain ever subscribes to them |

## Known rough edge

On this Wayland/XWayland setup the live window prints
`WARNING: OpenGL error 0x502 in or before mjr_makeContext` on startup and a GLFW teardown
warning on exit. Frames stream, telemetry prints and the run saves correctly either way. If the
window turns out to be unusable on your driver, `record` renders through EGL offscreen and is
verified working — reach for that and say so, and this note gets replaced by a fix.

## Who is driving

`live` and `replay` draw a status overlay in the window, where the eye already is:

- a **ball** above the duck — **green** while the brain drives, **red** while the recovery
  scaffold has the body, grey for a run with no driver (`--hold`);
- a **blue bar** beside it whose length is the earned consolidation `c` (the smaller of
  the two MotorEPMs);
- an **orange arrow** along the force while a shove is being applied, longer with the newtons.

Hand-offs are also printed as their own line in the terminal (`t=… -> scaffold`), and the
status line carries the driver, `c`, the active push and any harness event.

## The sweep clouds (2026-09-13)

A run whose graph declares `CloudMap` and whose host ran with `--cloud` writes one `cloudv` record
each time a cloud is filed: the stop's voxels as `[ix, iy, iz, hits, mean_height_mm]` in the cloud's
own body-anchored frame, plus the **world** pose it was anchored on. That pose is instrumentation for
this viewer — it is how a body-anchored cloud gets drawn beside the furniture it describes — and no
brain reads it.

- `P` toggles the clouds; `N` steps through them one place at a time, then back to all of them.
- Voxels are coloured by the **mean height of the points in them**, not the voxel centre: grey is the
  floor, orange is the 2–20 cm band where something stands on the floor, blue is furniture height,
  pale is wall tops. Logs written before 2026-09-13 carry 4-tuples and fall back to the centre, which
  draws the whole floor orange.
- Replay accumulates clouds as the run goes, so the room fills in as the duck visits it — **on a log
  without the live events below.** That union is not what the duck perceives (see next).

## The cloud as the duck holds it (2026-09-27)

The operator, watching the chase phase: "I can see the smear of the moving objects and the accumulated
errors. Is there a way to improve the voxel view so it is more similar to what the robot is perceiving? If
the robot is forgetting places then let's work that into the UI." `ogma::CloudMap` holds **one open
cloud** (every voxel since its anchor, each with the tick it was last seen), **forgets** a walking cloud the
moment it files it (never cached), and **remembers** a stop's cloud by place in a cache of eight, least
recently filed evicted. The union above kept every filed cloud on screen forever.

With the host's `--log-cloud-live` (on every chase-phase preset from R84) the record carries the module's
own events — `cldo` a cloud opens (its anchor's world pose, walking or not), `cldn` the voxels each cast
touched, `cloudv` a cloud filed (now with `walking`), `cldx` a place the cache evicted — and the view
follows them:

- the **open cloud** is drawn bright where a voxel was seen in the last 0.5 s (the movers' recency window)
  and fading with each voxel's age over ~6 s to a floor, so the smear of a mover reads as a tail that dies
  and a static thing settles to a dim, steady block; `A` toggles the fading;
- a **walking cloud vanishes** when the module files it (every metre of travel, or when the body stands);
- a **remembered place** (a stop's cloud) stays, drawn at a third of the strength, until the cache lets it
  go; `N` steps through the remembered places one at a time;
- the **chase** (`M`): a pale ring on the floor is the candidate the seek loop holds; a **yellow arrow** starts
  at the target it is chasing and points along the velocity it believes, longer the faster (the red ball was
  the play area's own, so it went); both are the loop's own odometry-frame positions put into the world through
  the body's true pose, exactly as the scorer labels them;
- the camera window's line reads `cloud: walking 16136 vox, oldest 99.9s  remembered 3` — the module's
  own count, and how long the oldest voxel has been held.

The scene holds 10 000 geoms; a long walking cloud can exceed it (a duck that babbles within a metre keeps
one cloud open for minutes), so the freshest voxels are drawn first and the oldest are what the cap drops.

## Recording a long run

`record` streams frames to the encoder one at a time, so its memory is flat in the run's length (about
0.7 GB). It used to keep every frame and write at the end — fine for an 8 s clip, fatal for a 1500 s
playroom run: 75 000 frames at 2 MB each, and on 2026-09-13 the kernel killed it at 23.9 GB. For a long
run, render a window:

```sh
.venv/bin/python view.py --scene scene_playroom.xml record RUN.jsonl OUT.mp4 --from 600 --to 1500 --every 10
```

`--from`/`--to` are run seconds; `--every N` keeps every Nth frame, so that example is an 86 s video.
Filed clouds are carried through skipped frames, so a window that opens after a stop still shows them.

