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
