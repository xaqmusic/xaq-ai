#!/usr/bin/env python3
"""Watch the Microduck host.

**This viewer does not simulate anything.** It reads the `qpos` the host wrote,
assigns it, and calls `mj_forward` to place the geometry. Every pose on screen was
computed by `ogma_mjhost`, so what you are watching is the run rather than a
re-derivation of it. A second copy of the dynamics would be a second thing to keep
in step, and the first time they disagreed the screen would be the convincing one.

Three ways in:

    view.py live   [-- host args...]   spawn the host and watch it as it runs
    view.py replay RUN.jsonl           watch a saved run
    view.py record RUN.jsonl OUT.mp4   render one to video, no window needed

`live` and `replay` open an interactive MuJoCo window: drag to orbit, scroll to
zoom, space to pause, and every one of MuJoCo's own viewer keys works.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SCENE = REPO / "mj_host/models/microduck/scene.xml"
HOST = REPO / "mj_host/build/ogma_mjhost"
BRAIN_HZ = 50.0


def need(module, why):
    try:
        return __import__(module)
    except ImportError:
        sys.exit(f"{module} is not installed — needed for {why}.\n"
                 f"  {REPO}/tools/duck_viewer/setup.sh   installs it into a local venv")


def frames_from(stream):
    """Yield parsed frames, skipping anything that is not one of ours.

    The host writes its summary to stderr and its data to stdout, so in practice
    nothing else appears here — but a stray line must not take the viewer down
    mid-run, because the run is the thing being observed."""
    for line in stream:
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            frame = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "qpos" in frame:
            yield frame


def host_command(extra, mode="--hold"):
    if not HOST.exists():
        sys.exit(f"host not built: {HOST}\n  cmake --build mj_host/build -j8")
    return [str(HOST), mode, *extra]


# Who is driving, drawn where the eye already is.  A ball above the duck: GREEN
# while the brain drives, RED while the recovery scaffold has the body, grey when
# the run has no driver field (a --hold run).  Beside it a BLUE bar whose length
# is the earned consolidation c (the smaller of the two MotorEPMs), and an ORANGE
# arrow along the force while a shove is being applied.  These are user-scene
# geoms; nothing here touches the model or the data.
STATUS_LEGEND = ("  ball: green = brain driving, yellow = step hand-off, red = scaffold rescue, grey = no driver"
                 "  |  blue bar: consolidation c  |  orange arrow: a shove")
DRIVE_RGBA = {"brain": (0.10, 0.90, 0.20, 1.0), "scaffold": (0.95, 0.15, 0.10, 1.0),
              "step": (1.0, 0.85, 0.10, 1.0)}     # yellow: the walker taking a step for the brain


def draw_status(scn, frame, mujoco, np, reset=True):
    """Append the status geoms to a scene: the viewer's user scene (reset it — ours are
    the only geoms there) or the renderer's own scene after update_scene (append)."""
    if reset:
        scn.ngeom = 0
    x, y, z = frame["x"], frame["y"], frame["z"]
    eye = np.eye(3).flatten()

    def geom():
        if scn.ngeom >= scn.maxgeom:
            return None
        g = scn.geoms[scn.ngeom]
        scn.ngeom += 1
        return g

    drive = frame.get("drive")
    rgba = DRIVE_RGBA.get(drive, (0.55, 0.55, 0.55, 1.0))
    g = geom()
    if g is not None:
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.02, 0.0, 0.0]),
                            np.array([x, y, z + 0.20]), eye, np.array(rgba, dtype=np.float32))
    cons = frame.get("cons") or []
    if cons:
        c = max(0.0, min(1.0, min(cons)))
        g = geom()
        if g is not None:
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), eye,
                                np.array((0.25, 0.55, 1.0, 1.0), dtype=np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.006,
                                 np.array([x, y, z + 0.235]),
                                 np.array([x, y, z + 0.235 + 0.08 * max(c, 0.02)]))
    push = frame.get("push") or [0, 0, 0]
    if any(push):
        f = np.array(push, dtype=float)
        mag = float(np.linalg.norm(f))
        d = f / mag
        g = geom()
        if g is not None:
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), eye,
                                np.array((1.0, 0.6, 0.0, 1.0), dtype=np.float32))
            # The arrow points INTO the trunk along the force, its length with the newtons.
            tip = np.array([x, y, z])
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.012,
                                 tip - d * (0.08 + 0.02 * mag), tip)


def status_line(frame):
    line = f"  t={frame['t']:6.2f}s  tilt={frame['tilt']:6.2f}°  z={frame['z']:.4f}m"
    if "drive" in frame:
        line += f"  {frame['drive']:8s}"
    if frame.get("cons"):
        line += f"  c={min(frame['cons']):.2f}"
    push = frame.get("push") or [0, 0, 0]
    if any(push):
        line += f"  push {push[0]:+.1f},{push[1]:+.1f} N"
    if frame.get("event"):
        line += f"  {frame['event']}"
    return line


def watch(frames, realtime=True, title_every=25):
    """Drive the interactive viewer from a stream of frames."""
    mujoco = need("mujoco", "the viewer")
    np = need("numpy", "the status overlay")
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    print(STATUS_LEGEND)
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False,
                                      show_right_ui=False) as viewer:
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.9, -12, 130
        started = time.perf_counter()
        n = 0
        last_drive = None
        for frame in frames:
            if not viewer.is_running():
                break
            data.qpos[:] = frame["qpos"]
            mujoco.mj_forward(model, data)
            viewer.cam.lookat[:] = (frame["x"], frame["y"], frame["z"])
            draw_status(viewer.user_scn, frame, mujoco, np)
            viewer.sync()
            n += 1
            drive = frame.get("drive")
            if drive != last_drive and last_drive is not None:
                # A hand-off is the event worth a line of its own, not a title flicker.
                print(f"\n  t={frame['t']:6.2f}s  -> {drive}")
            last_drive = drive
            if n % title_every == 0:
                print("\r" + status_line(frame), end="", flush=True)
            if realtime:
                # Pace to wall clock. The host runs far faster than real time, and
                # a run that flashes past is not an observation.
                due = started + (n / BRAIN_HZ)
                slack = due - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
        print()
    return n


def record(frames, out_path, width=960, height=720):
    os.environ.setdefault("MUJOCO_GL", "egl")
    mujoco = need("mujoco", "rendering")
    imageio = need("imageio", "writing video")
    import imageio.v2 as iio

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    # The offscreen buffer is sized by the model, and the vendored scene does not
    # ask for one this large. Set it on the LOADED model rather than editing the
    # XML: gate G1 is that Pollen's files stay untouched.
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height, width)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.distance, camera.elevation, camera.azimuth = 0.9, -12, 130

    np = need("numpy", "the status overlay")
    images = []
    for frame in frames:
        data.qpos[:] = frame["qpos"]
        mujoco.mj_forward(model, data)
        camera.lookat[:] = (frame["x"], frame["y"], frame["z"])
        renderer.update_scene(data, camera)
        draw_status(renderer.scene, frame, mujoco, np, reset=False)   # the same overlay as live
        images.append(renderer.render().copy())
    if not images:
        sys.exit("no frames — did the host write anything?")
    iio.mimwrite(out_path, images, fps=int(BRAIN_HZ), quality=8, macro_block_size=1)
    print(f"wrote {out_path}  ({len(images)} frames, {len(images)/BRAIN_HZ:.1f} s)")
    return len(images)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    live = sub.add_parser("live", help="spawn the host and watch it run")
    live.add_argument("--save", metavar="RUN.jsonl", help="also keep the run")
    live.add_argument("--host-mode", default="--hold",
                      help="which host mode to drive: --hold (default) or --stub")
    live.add_argument("host_args", nargs="*", help="passed through, e.g. --secs 30 --noise 0.05")

    rep = sub.add_parser("replay", help="watch a saved run")
    rep.add_argument("run")
    rep.add_argument("--fast", action="store_true", help="as fast as it draws")

    rec = sub.add_parser("record", help="render a run to video")
    rec.add_argument("run", help="a saved run, or - to read stdin")
    rec.add_argument("out")

    a = p.parse_args()

    if a.mode == "live":
        cmd = host_command(a.host_args, a.host_mode)
        print(f"  {' '.join(cmd)}")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
        saved = open(a.save, "w") if a.save else None

        def stream():
            for line in proc.stdout:
                if saved:
                    saved.write(line)
                yield line

        try:
            n = watch(frames_from(stream()))
        finally:
            proc.terminate()
            proc.wait(timeout=5)
            if saved:
                saved.close()
                print(f"  kept {a.save}")
        print(f"  {n} frames")

    elif a.mode == "replay":
        with open(a.run) as f:
            watch(frames_from(f), realtime=not a.fast)

    elif a.mode == "record":
        stream = sys.stdin if a.run == "-" else open(a.run)
        record(frames_from(stream), a.out)


if __name__ == "__main__":
    main()
