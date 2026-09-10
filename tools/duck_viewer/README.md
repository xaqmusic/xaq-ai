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
| `W` | **walls fade** — an outer wall drops to 15 % opacity while the camera is on its far side, so a camera parked outside the room for a screen recording never has a 1 m wall between it and the duck | on |

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
