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


def watch(frames, realtime=True, title_every=25):
    """Drive the interactive viewer from a stream of frames."""
    mujoco = need("mujoco", "the viewer")
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data, show_left_ui=False,
                                      show_right_ui=False) as viewer:
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.9, -12, 130
        started = time.perf_counter()
        n = 0
        for frame in frames:
            if not viewer.is_running():
                break
            data.qpos[:] = frame["qpos"]
            mujoco.mj_forward(model, data)
            viewer.cam.lookat[:] = (frame["x"], frame["y"], frame["z"])
            viewer.sync()
            n += 1
            if n % title_every == 0:
                print(f"\r  t={frame['t']:6.2f}s  tilt={frame['tilt']:6.2f}°"
                      f"  z={frame['z']:.4f}m", end="", flush=True)
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

    images = []
    for frame in frames:
        data.qpos[:] = frame["qpos"]
        mujoco.mj_forward(model, data)
        camera.lookat[:] = (frame["x"], frame["y"], frame["z"])
        renderer.update_scene(data, camera)
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
