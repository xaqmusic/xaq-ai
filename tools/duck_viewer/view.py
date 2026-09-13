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
zoom, space to pause, and every one of MuJoCo's own viewer keys works. Ours:

    V   the ToF beams (off by default: 64 lines from the head hide the head)
    C   the brain-camera window: what the head camera hands the brain, at the
        brain's resolution (--cam-res, default 64x48) and rate (12.5 Hz), scaled up
        without smoothing so the coarse frame is what you see; the status line and
        these keys are its HUD, under the image
    H   that HUD text
    W   what stands between the camera and the duck fades to 15 % (on by default):
        an outer wall while the camera is on its far side, a table or chair while
        it blocks the line of sight — a parked camera no longer loses the moment

The camera window renders the head camera from the same qpos the viewer draws, so it
shows exactly the frame the host will publish once it renders (playroom plan C1); until
then it is the preview of that frame, not a copy of it.
"""

import argparse
import json
import math
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
    # The host paces itself to the wall clock.  Pacing it through this pipe instead
    # makes it run in 8 KB bursts, which every tick-time observer (the inspector's
    # diag stream) sees as a jerk; with --realtime the frames arrive 20 ms apart and
    # the pacing in watch() has nothing to do.
    return [str(HOST), mode, "--realtime", *extra]


# Who is driving, drawn where the eye already is.  A ball above the duck: GREEN
# while the brain drives, RED while the recovery scaffold has the body, grey when
# the run has no driver field (a --hold run).  Beside it a BLUE bar whose length
# is the earned consolidation c (the smaller of the two MotorEPMs), and an ORANGE
# arrow along the force while a shove is being applied.  These are user-scene
# geoms; nothing here touches the model or the data.
STATUS_LEGEND = ("  ball: green = brain driving, yellow = step hand-off, red = scaffold rescue, grey = no driver"
                 "  |  blue bar: consolidation c  |  orange arrow: a shove")
DRIVE_RGBA = {"brain": (0.10, 0.90, 0.20, 1.0), "scaffold": (0.95, 0.15, 0.10, 1.0),
              "step": (1.0, 0.85, 0.10, 1.0),     # yellow: the walker taking a step for the brain
              "walk": (0.20, 0.75, 1.0, 1.0),     # blue: the walker walking on an intent
              "stand": (0.10, 0.90, 0.20, 1.0)}   # green, as "brain": the joint brain standing at a stop (W1)


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


# The ToF's 64 beams in the tof site frame (forward +x, left +y, up +z), as Tof.cpp lays
# them out: row 0 top, column 0 left, 45° FOV with a half-zone inset.
def _tof_beams(np):
    half = math.radians(45.0 / 2 - 45.0 / 8 / 2)
    step = 2 * half / 7
    out = []
    for i in range(64):
        el = half - (i // 8) * step
        az = half - (i % 8) * step
        out.append((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    return np.array(out)


TOF_RGBA = {"1": (1.0, 0.2, 0.2, 0.9), "2": (0.6, 0.6, 0.7, 0.35), "3": (1.0, 0.55, 0.1, 0.9)}


def draw_tof(scn, frame, model, data, mujoco, np, beams):
    """The beams that returned, from the head, coloured by class (Hit orange, Floor grey,
    TooClose red).  Empty beams are not drawn.  Uses the site pose the viewer already
    placed with mj_forward — the same geometry the host cast from."""
    z = frame.get("tofz")
    r = frame.get("tofr")
    if not z or not r:
        return
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tof")
    if sid < 0:
        return
    pos = data.site_xpos[sid]
    R = data.site_xmat[sid].reshape(3, 3)
    for i in range(64):
        cls = z[i]
        if cls == "0" or r[i] < 0 or scn.ngeom >= scn.maxgeom:
            continue
        tip = pos + R @ beams[i] * r[i]
        g = scn.geoms[scn.ngeom]
        scn.ngeom += 1
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                            np.array(TOF_RGBA[cls], dtype=np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, 1.5, pos, tip)


# ---- the sweep clouds ---------------------------------------------------------------
# A cloud is what the duck accumulated while standing still at one place: the ToF's returns,
# gravity-levelled, de-rotated by its own odometry yaw, voxelised at 4 cm.  The host writes one
# `cloudv` record per cloud when it files it (ogma::CloudMap), carrying the voxels in the cloud's
# own body-anchored frame plus the WORLD pose it was anchored on.  That pose is instrumentation:
# it exists so the cloud can be drawn beside the furniture it describes, which is the only way to
# see at a glance whether the cloud is right.  The duck never reads it.
#
# Height colours the voxels, because height is the feature that tells the kinds apart (design doc
# §17.28).  The 2-20 cm band an object on the floor occupies is drawn opaque and everything else
# faint, so a small thing standing on the floor is where the eye lands.
CLOUD_BANDS = (                      # (height below, rgba)
    (0.02, (0.42, 0.47, 0.55, 0.25)),          # the floor itself: faint grey-blue
    (0.20, (0.92, 0.41, 0.20, 0.95)),          # THE BREAK BAND: something stands here
    (0.45, (0.16, 0.47, 0.84, 0.55)),          # chair/table height
    (9.99, (0.40, 0.62, 0.80, 0.30)),          # wall tops, the shelf, the clock
)


def cloud_rgba(h):
    for below, rgba in CLOUD_BANDS:
        if h < below:
            return rgba
    return CLOUD_BANDS[-1][1]


def draw_clouds(scn, clouds, mujoco, np, only=None):
    """Append every carried cloud as voxel boxes at the world pose it was anchored on.
    `only` = draw just that place id.  Returns the number of voxels drawn."""
    eye = np.eye(3).flatten()
    drawn = 0
    for rec in clouds:
        if only is not None and rec["place"] != only:
            continue
        v = rec["voxel_m"]
        ax, ay, ayaw = rec["anchor"]
        ca, sa = math.cos(ayaw), math.sin(ayaw)
        half = np.array([v * 0.5, v * 0.5, v * 0.5])
        for vox in rec["vox"]:
            if scn.ngeom >= scn.maxgeom:
                return drawn
            ix, iy, iz = vox[0], vox[1], vox[2]
            px, py = (ix + 0.5) * v, (iy + 0.5) * v
            wz = (iz + 0.5) * v
            # colour by the MEAN height of the points in the voxel (the 5th value, mm), not the centre:
            # the ground layer's centre is exactly 2 cm, and a centre test drew the whole floor in the
            # break colour.  Logs from before 2026-09-13 carry 4-tuples and fall back to the centre.
            h = vox[4] / 1000.0 if len(vox) > 4 else wz
            g = scn.geoms[scn.ngeom]
            scn.ngeom += 1
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, half,
                                np.array([ca * px - sa * py + ax, sa * px + ca * py + ay, wz]), eye,
                                np.array(cloud_rgba(h), dtype=np.float32))
            drawn += 1
    return drawn


# ---- the operator's keys ------------------------------------------------------------
# State the key callback flips; read by the watch loop each frame. Defaults: the beams
# off (they hide the head), the camera window on, its HUD text on.
#
# The HUD lives in the camera window, not the MuJoCo window: label geoms placed in the
# free camera's frame lagged the mouse between syncs and flashed on every zoom (2026-09-10).
UI = {"tof": False, "help": True, "cam": True, "fade": True, "cloud": True, "solo": None}
HOTKEYS = ("V  ToF beams     C  this window     H  this text     W  fade what hides the duck\n"
           "P  sweep clouds  N  next cloud alone / all\n"
           "space  pause     drag  orbit     scroll  zoom     right-drag  pan")


def key_callback(keycode):
    if keycode == ord("V"):
        UI["tof"] = not UI["tof"]
    elif keycode == ord("C"):
        UI["cam"] = not UI["cam"]
    elif keycode == ord("H"):
        UI["help"] = not UI["help"]
    elif keycode == ord("W"):
        UI["fade"] = not UI["fade"]
    elif keycode == ord("P"):
        UI["cloud"] = not UI["cloud"]
    elif keycode == ord("N"):
        UI["solo"] = "advance"          # the watch loop resolves this against the clouds it has


def free_camera_position(cam, np):
    """Where MuJoCo's free camera is, from its lookat/distance/azimuth/elevation (the same
    formula the renderer uses; checked against mjvGLCamera on 2026-09-10)."""
    az, el = math.radians(cam.azimuth), math.radians(cam.elevation)
    fwd = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    return np.array(cam.lookat) - cam.distance * fwd


class WallFader:
    """What stands between the camera and the duck fades to 15 % opacity.

    A long run is recorded with the camera parked at an angle, and a 1 m wall between the
    camera and the duck hides the one moment worth seeing (the table-leg wedge, 2026-09-10,
    was lost that way). Two tests, both against the loaded model only — the viewer simulates
    nothing, so nothing else can change:

    * an outer wall (a `wall_*` box on the world body) fades while the camera is beyond its
      plane on the side away from the room;
    * a piece of furniture (a static `furn_*` body) fades while any of a small bundle of
      rays from the camera to the duck's trunk hits one of its geoms before reaching the
      duck — the table top when the camera looks down through it, a chair between them.
      A fade holds for half a second after the last hit, so a grazing ray does not flicker.
    """

    ALPHA = 0.15
    HOLD = 25                                             # frames a fade outlives its last hit

    def __init__(self, model, mujoco, np):
        self.model, self.mujoco, self.np = model, mujoco, np
        self.walls = []                                   # (geom id, centre, outward unit normal, half-thickness)
        self.furniture = {}                               # body id -> [geom ids]
        for g in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            body = model.geom_bodyid[g]
            if name.startswith("wall") and body == 0 and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                size, pos = model.geom_size[g], model.geom_pos[g]
                axis = int(np.argmin(size[:2]))           # the thin horizontal axis
                outward = np.zeros(3)
                outward[axis] = 1.0 if pos[axis] >= 0 else -1.0
                self.walls.append((g, pos.copy(), outward, float(size[axis])))
                continue
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
            if bname.startswith("furn_") and model.body_weldid[body] == 0:
                self.furniture.setdefault(body, []).append(g)
        self.hold = {}                                    # geom id -> frames left faded
        self.faded = set()

    def _ray_hits(self, data, origin, target):
        """Furniture geoms that any ray of the bundle meets before the duck."""
        mujoco, np = self.mujoco, self.np
        hits = set()
        for off in ((0, 0, 0), (0.12, 0, 0), (-0.12, 0, 0), (0, 0.12, 0), (0, -0.12, 0), (0, 0, 0.1)):
            vec = (target + np.array(off, dtype=float)) - origin
            reach = float(np.linalg.norm(vec))
            if reach < 1e-6:
                continue
            for body, geoms in self.furniture.items():
                for g in geoms:
                    dist = mujoco.mju_rayGeom(data.geom_xpos[g], data.geom_xmat[g], self.model.geom_size[g],
                                             origin, vec, int(self.model.geom_type[g]))
                    if 0.0 <= dist < 1.0:             # mju_rayGeom returns the distance in units of |vec|
                        hits.update(geoms)
                        break
        return hits

    def update(self, cam, data=None, target=None, on=True):
        pos = free_camera_position(cam, self.np) if on else None
        want = set()
        if on:
            for g, centre, outward, half in self.walls:
                if float(self.np.dot(pos - centre, outward)) > half:
                    want.add(g)
            if data is not None and target is not None and self.furniture:
                want |= self._ray_hits(data, pos, self.np.asarray(target, dtype=float))
        for g in want:
            self.hold[g] = self.HOLD
        for g in list(self.hold):
            self.hold[g] -= 1
            if self.hold[g] <= 0 or not on:
                del self.hold[g]
        self.faded = set(self.hold)
        for g in [g for g, *_ in self.walls] + [g for gs in self.furniture.values() for g in gs]:
            alpha = self.ALPHA if g in self.faded else 1.0
            if self.model.geom_rgba[g][3] != alpha:
                self.model.geom_rgba[g][3] = alpha


class CamWindow:
    """A second window showing the head camera's frame at the brain's resolution.

    Rendered here with an offscreen EGL renderer from the same model and qpos the viewer
    draws, then handed to Tk as a PPM (no image library needed) and zoomed without
    smoothing, so every one of the brain's pixels is a visible block."""

    def __init__(self, model, mujoco, np, width, height, zoom=8):
        import tkinter as tk
        self.mujoco, self.np, self.tk = mujoco, np, tk
        self.w, self.h, self.zoom = width, height, zoom
        self.cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
        self.renderer = mujoco.Renderer(model, height, width)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam.fixedcamid = self.cam_id
        self.root = tk.Tk()
        self.root.title(f"head camera -> brain   {width}x{height} @ 12.5 Hz   (C hides)")
        self.root.resizable(False, False)
        self.label = tk.Label(self.root, bd=0)
        self.label.pack()
        self.hud = tk.Label(self.root, bd=0, anchor="w", justify="left", font=("TkFixedFont", 10),
                            bg="#181818", fg="#e8e8e8", padx=8, pady=6)
        self.hud.pack(fill="x")
        self.photo = None
        self.shown = True
        self.hud_shown = True

    def set_hud(self, status, on):
        """The status line and the hotkeys under the image; H hides them."""
        if on != self.hud_shown:
            (self.hud.pack(fill="x") if on else self.hud.pack_forget())
            self.hud_shown = on
        if on:
            self.hud.configure(text=status.strip() + "\n" + HOTKEYS)

    def update(self, data):
        self.renderer.update_scene(data, self.cam)
        img = self.renderer.render()
        ppm = b"P6 %d %d 255\n" % (self.w, self.h) + self.np.ascontiguousarray(img).tobytes()
        photo = self.tk.PhotoImage(data=ppm).zoom(self.zoom, self.zoom)
        self.label.configure(image=photo)
        self.photo = photo                                   # keep a reference or Tk drops it
        self.root.update()

    def show(self, on):
        if on and not self.shown:
            self.root.deiconify()
        elif not on and self.shown:
            self.root.withdraw()
        self.shown = on
        self.root.update()

    def close(self):
        try:
            self.root.destroy()
        except Exception:
            pass


def status_line(frame):
    line = f"  t={frame['t']:6.2f}s  tilt={frame['tilt']:5.2f} deg  z={frame['z']:.4f}m"
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


def watch(frames, realtime=True, title_every=25, cam_res=(64, 48), fast_until=0.0):
    # the sweep clouds the run has filed so far, newest per place; and which one is soloed
    clouds, solo_at = [], None
    """Drive the interactive viewer from a stream of frames.

    `fast_until`: frames before this many run-seconds are fast-forwarded — no pacing, one
    frame in 25 drawn — and the wall clock re-bases where the pacing starts. With the host's
    own --fast-until this skips a level-2 run's 600 s babble without changing the run."""
    _prefer_x11_window()
    # The brain-camera window renders offscreen through EGL, which needs no display and
    # leaves the viewer's own GLFW window alone. Set before mujoco is imported.
    os.environ.setdefault("MUJOCO_GL", "egl")
    mujoco = need("mujoco", "the viewer")
    np = need("numpy", "the status overlay")
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    print(STATUS_LEGEND)
    print("  keys: V ToF beams (off)   C brain-camera window   H its HUD text   W fade what hides the duck (on)")
    beams = _tof_beams(np)
    fader = WallFader(model, mujoco, np)
    camwin = None
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera") >= 0:
        try:
            camwin = CamWindow(model, mujoco, np, *cam_res)
        except Exception as e:                               # no Tk, no EGL: the viewer still runs
            print(f"  brain-camera window unavailable: {e}")
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False,
                                      show_right_ui=False, key_callback=key_callback) as viewer:
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.9, -12, 130
        started = time.perf_counter()
        n = 0
        last_drive = None
        fast = fast_until > 0
        for frame in frames:
            if not viewer.is_running():
                break
            if fast and frame["t"] >= fast_until:
                fast = False
                started = time.perf_counter() - n / BRAIN_HZ          # the clock starts here
                print(f"\n  t={frame['t']:6.2f}s  -> real time")
            if fast:
                n += 1
                if n % 25:
                    continue                                          # fast-forward: draw 2 fps of it
            data.qpos[:] = frame["qpos"]
            mujoco.mj_forward(model, data)
            viewer.cam.lookat[:] = (frame["x"], frame["y"], frame["z"])
            fader.update(viewer.cam, data, (frame["x"], frame["y"], frame["z"]), UI["fade"])
            # a cloud the host filed on this tick joins the set we carry (replay accumulates
            # them as the run goes, so the room fills in as the duck visits it)
            if "cloudv" in frame:
                rec = frame["cloudv"]
                clouds = [c for c in clouds if c["place"] != rec["place"]] + [rec]
            if UI["solo"] == "advance":
                places = sorted({c["place"] for c in clouds})
                if not places:
                    UI["solo"] = None
                elif solo_at is None:
                    solo_at = 0
                elif solo_at + 1 >= len(places):
                    solo_at = None
                else:
                    solo_at += 1
                UI["solo"] = None
            draw_status(viewer.user_scn, frame, mujoco, np)
            if UI["cloud"] and clouds:
                places = sorted({c["place"] for c in clouds})
                only = places[solo_at] if solo_at is not None and solo_at < len(places) else None
                draw_clouds(viewer.user_scn, clouds, mujoco, np, only=only)
            if UI["tof"]:
                draw_tof(viewer.user_scn, frame, model, data, mujoco, np, beams)
            viewer.sync()
            if camwin is not None:
                camwin.show(UI["cam"])
                if UI["cam"]:
                    camwin.set_hud(status_line(frame) + ("   [fast-forward]" if fast else ""), UI["help"])
                    if n % 4 == 0 or fast:                   # the ToF's rate, 12.5 Hz: the brain's frame rate
                        camwin.update(data)
            if fast:
                continue
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
    if camwin is not None:
        camwin.close()
    return n


def record(frames, out_path, width=960, height=720, t_from=0.0, t_to=None, every=1):
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
    beams = _tof_beams(np)
    fader = WallFader(model, mujoco, np)
    rec_clouds = []                      # the sweep clouds filed so far, newest per place
    written = 0
    # STREAMED to the encoder, one frame at a time.  This used to collect every frame in a list and
    # write at the end — fine for the 8 s clips it was built for, fatal for a long run: 960x720x3 is
    # 2.07 MB a frame, a 1500 s playroom run is 75 000 frames, and on 2026-09-13 the kernel killed it
    # at 23.9 GB, eleven thousand frames in.  Memory is now flat in the run's length.
    writer = iio.get_writer(out_path, fps=int(BRAIN_HZ), quality=8, macro_block_size=1)
    try:
      for n, frame in enumerate(frames):
        # a filed cloud is an EVENT: carry it even through frames we skip, or a window that opens
        # after a stop draws the room empty
        if "cloudv" in frame:
            rec = frame["cloudv"]
            rec_clouds[:] = [c for c in rec_clouds if c["place"] != rec["place"]] + [rec]
        t = frame.get("t", 0.0)
        if t_to is not None and t > t_to:
            break
        if t < t_from or n % every:
            continue
        data.qpos[:] = frame["qpos"]
        mujoco.mj_forward(model, data)
        camera.lookat[:] = (frame["x"], frame["y"], frame["z"])
        fader.update(camera, data, (frame["x"], frame["y"], frame["z"]))
        renderer.update_scene(data, camera)
        draw_status(renderer.scene, frame, mujoco, np, reset=False)   # the same overlay as live
        if rec_clouds:
            draw_clouds(renderer.scene, rec_clouds, mujoco, np)
        draw_tof(renderer.scene, frame, model, data, mujoco, np, beams)
        writer.append_data(renderer.render())
        written += 1
    finally:
        writer.close()
    if not written:
        sys.exit("no frames in that window — did the host write anything, and do --from/--to cover it?")
    print(f"wrote {out_path}  ({written} frames, {written/BRAIN_HZ:.1f} s of video)")
    return written


def _prefer_x11_window():
    """Ask GLFW for an X11 window under a Wayland session.

    MuJoCo's passive viewer opens a GLFW window.  Under Wayland GLFW draws its
    own (missing) decorations, so the window cannot be moved or resized from
    the desktop; as an XWayland client the compositor frames it like any other
    window.  Set DUCK_VIEWER_PLATFORM=wayland to keep the native path.
    """
    if os.environ.get("XDG_SESSION_TYPE") != "wayland" or not os.environ.get("DISPLAY"):
        return
    if os.environ.get("DUCK_VIEWER_PLATFORM", "x11") != "x11":
        return
    # The glfw wheel ships an X11 and a Wayland build and picks by session
    # type unless told otherwise; this must be set before glfw is imported.
    os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "x11")
    try:
        import glfw
        glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)
    except (ImportError, AttributeError):
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    live = sub.add_parser("live", help="spawn the host and watch it run")
    live.add_argument("--save", metavar="RUN.jsonl", help="also keep the run")
    live.add_argument("--host-mode", default="--hold",
                      help="which host mode to drive: --hold (default) or --stub")
    live.add_argument("host_args", nargs="*", help="passed through, e.g. --secs 30 --noise 0.05")

    p.add_argument("--scene", default=None, help="the MJCF the run used (default: scene.xml); the arena runs need scene_arena.xml")
    p.add_argument("--cam-res", default="64x48", help="the brain-camera window's render size WxH (default 64x48, the playroom plan's C1 start)")
    p.add_argument("--fast-until", type=float, default=0.0, metavar="S",
                   help="fast-forward the run's first S seconds (no pacing, 1 frame in 25 drawn); pair with the host's --fast-until S in live mode")
    rep = sub.add_parser("replay", help="watch a saved run")
    rep.add_argument("run")
    rep.add_argument("--fast", action="store_true", help="as fast as it draws")

    rec = sub.add_parser("record", help="render a run to video")
    rec.add_argument("run", help="a saved run, or - to read stdin")
    rec.add_argument("out")
    rec.add_argument("--from", dest="t_from", type=float, default=0.0, metavar="S", help="start at this run second")
    rec.add_argument("--to", dest="t_to", type=float, default=None, metavar="S", help="stop at this run second")
    rec.add_argument("--every", type=int, default=1, metavar="N",
                     help="keep every Nth frame (a 1500 s run at --every 10 is a 150 s video)")

    a = p.parse_args()
    try:
        cam_res = tuple(int(v) for v in a.cam_res.lower().split("x"))
        assert len(cam_res) == 2 and min(cam_res) >= 8
    except (ValueError, AssertionError):
        sys.exit(f"--cam-res wants WxH, got {a.cam_res!r}")
    global SCENE
    if a.scene:
        SCENE = Path(a.scene) if Path(a.scene).is_absolute() else REPO / "mj_host/models/microduck" / a.scene

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
            n = watch(frames_from(stream()), cam_res=cam_res, fast_until=a.fast_until)
        finally:
            proc.terminate()
            proc.wait(timeout=5)
            if saved:
                saved.close()
                print(f"  kept {a.save}")
        print(f"  {n} frames")

    elif a.mode == "replay":
        with open(a.run) as f:
            watch(frames_from(f), realtime=not a.fast, cam_res=cam_res, fast_until=a.fast_until)

    elif a.mode == "record":
        stream = sys.stdin if a.run == "-" else open(a.run)
        record(frames_from(stream), a.out, t_from=a.t_from, t_to=a.t_to, every=max(1, a.every))


if __name__ == "__main__":
    main()
