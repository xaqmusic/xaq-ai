#!/usr/bin/env python3
"""playroom_gen — generate the duck's playroom scene from a seed, with a manifest.

    mj_host/tools/playroom_gen.py [--seed 1] [--half 2.0] [--out scene_playroom.xml]
                                  [--balls 2 --blocks 2 --chairs 2] [--check]

The playroom is the arena for the behaviour set (docs/plans-and-designs/microduck_playroom_plan.md
§5): a room the duck's own size sees as pillars, ceilings and things that answer when pushed.
Its objects are sorted by the only thing the brain can see — how they change:

  static, immovable   walls, a table, chairs, a shelf, a rug     (bake fast, go quiet)
  movable by the duck balls, blocks — free bodies with mass      (its own action changes them)
  self-changing       a wall clock whose hand the host turns     (novelty that never answers)
  operator-moved      any of the above, via the host's --move    (the (d) test)

Rules the generator keeps (plan §5.3, §5.5):
  * everything is placed from --seed and written to <out>.manifest.json; the host and the
    sweep print the manifest's seed and hash, so no two "varied" rooms can silently share a layout;
  * world geometry stays in collision group 0 (the ToF raycasts against group 0 only);
    the rug is group 0 but non-colliding, so the ToF reads it as floor;
  * static obstacles are named wall_*/furn_*: the host's `wall` instrument counts contact with
    any static world geom that is not floor or rug; movables are bodies named obj_*;
  * the keyframes are the arena's, with each free body's pose and the clock hinge appended,
    so `reset("STAND")` puts the room back exactly;
  * the encoder is fixed before the room is decorated; textures are MuJoCo builtins with
    fixed seeds and never adjusted to a result.

--check loads the result through the host (`ogma_mjhost --load-only`) when the host is built.
"""

import argparse
import hashlib
import json
import math
import random
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
MODEL_DIR = REPO / "mj_host/models/microduck"
ARENA = MODEL_DIR / "scene_arena.xml"
HOST = REPO / "mj_host/build/ogma_mjhost"

WALL_H = 0.30          # m, as the arena: the ToF sees it, the operator sees over it
WALL_T = 0.025
DUCK_KEEPOUT = 0.55    # m around the origin kept clear: the duck starts at (0, 0) facing +x


def arena_keyframes():
    """The robot's own keyframes, verbatim from scene_arena.xml (qpos 21, ctrl 14)."""
    text = ARENA.read_text()
    keys = []
    for m in re.finditer(r'<key name="(\w+)" qpos="([^"]+)" ctrl="([^"]+)"', text):
        name, qpos, ctrl = m.group(1), m.group(2).split(), m.group(3).split()
        keys.append((name, qpos, ctrl))
    if not keys:
        sys.exit(f"no keyframes found in {ARENA}")
    return keys


class Room:
    def __init__(self, seed, half, n_balls, n_blocks, n_chairs):
        self.seed, self.half = seed, half
        self.rng = random.Random(seed)
        self.n_balls, self.n_blocks, self.n_chairs = n_balls, n_blocks, n_chairs
        self.assets = []       # xml lines
        self.world = []        # xml lines
        self.free_bodies = []  # (name, x, y, z[, yaw]) in the order they appear
        self.hinges = []       # (name,) in order
        self.qpos_layout = []  # (name, kind) in worldbody order = MuJoCo's qpos order after the robot
        self.manifest = {"seed": seed, "half": half, "objects": []}
        self.placed = []       # (x, y, radius) for keep-out

    # ---- placement -------------------------------------------------------------------
    def place(self, radius, tries=200, margin=0.15):
        """A spot clear of the duck's start, the walls and everything placed so far."""
        lim = self.half - WALL_T - radius - margin
        for _ in range(tries):
            x, y = self.rng.uniform(-lim, lim), self.rng.uniform(-lim, lim)
            if math.hypot(x, y) < DUCK_KEEPOUT + radius:
                continue
            if all(math.hypot(x - px, y - py) > radius + pr + margin for px, py, pr in self.placed):
                self.placed.append((x, y, radius))
                return x, y
        sys.exit(f"could not place an object of radius {radius} (room too small for the count)")

    def record(self, name, cls, kind, x, y, z, extent, **extra):
        self.manifest["objects"].append(dict(name=name, cls=cls, kind=kind, x=round(x, 4), y=round(y, 4),
                                             z=round(z, 4), extent=round(extent, 4), **extra))

    # ---- assets ------------------------------------------------------------------------
    def textures(self):
        # Patterned, contrasting, fixed. A frozen random projection sees structure at every
        # scale in a pattern and none in a flat colour.
        T = self.assets.append
        T('<texture type="skybox" builtin="gradient" rgb1="0.85 0.9 1.0" rgb2="0.4 0.55 0.8" width="512" height="3072"/>')
        T('<texture type="2d" name="floor_tex" builtin="checker" mark="edge" rgb1="0.86 0.82 0.72" rgb2="0.80 0.76 0.66" markrgb="0.7 0.66 0.58" width="256" height="256"/>')
        T('<material name="floor_mat" texture="floor_tex" texuniform="true" texrepeat="8 8" reflectance="0.05"/>')
        T('<texture type="2d" name="rug_tex" builtin="checker" mark="cross" rgb1="0.75 0.15 0.15" rgb2="0.95 0.75 0.2" markrgb="0.1 0.1 0.4" width="128" height="128"/>')
        T('<material name="rug_mat" texture="rug_tex" texuniform="true" texrepeat="6 6"/>')
        walls = [("0.55 0.75 0.95", "0.35 0.55 0.85"), ("0.95 0.8 0.55", "0.85 0.6 0.35"),
                 ("0.6 0.85 0.6", "0.35 0.65 0.4"), ("0.9 0.6 0.75", "0.75 0.4 0.6")]
        for i, (a, b) in enumerate(walls):
            T(f'<texture type="2d" name="wall{i}_tex" builtin="checker" mark="random" random="0.08" rgb1="{a}" rgb2="{b}" markrgb="1 1 1" width="128" height="128"/>')
            T(f'<material name="wall{i}_mat" texture="wall{i}_tex" texuniform="true" texrepeat="12 2"/>')
        T('<texture type="2d" name="wood_tex" builtin="gradient" rgb1="0.55 0.35 0.2" rgb2="0.7 0.5 0.3" width="64" height="64"/>')
        T('<material name="wood_mat" texture="wood_tex" texuniform="true" texrepeat="3 3"/>')
        T('<texture type="2d" name="stripe_tex" builtin="checker" mark="none" rgb1="0.1 0.1 0.6" rgb2="0.95 0.95 0.95" width="64" height="64"/>')
        T('<material name="stripe_mat" texture="stripe_tex" texuniform="true" texrepeat="1 6"/>')
        T('<texture type="2d" name="dots_tex" builtin="flat" mark="random" random="0.25" rgb1="0.95 0.5 0.1" markrgb="0.1 0.1 0.1" width="64" height="64"/>')
        T('<material name="dots_mat" texture="dots_tex" texuniform="true" texrepeat="4 4"/>')
        T('<material name="ball0_mat" rgba="0.9 0.1 0.1 1"/>')
        T('<material name="ball1_mat" rgba="0.1 0.4 0.95 1"/>')
        T('<material name="ball2_mat" rgba="0.95 0.85 0.1 1"/>')
        T('<material name="block0_mat" rgba="0.2 0.8 0.3 1"/>')
        T('<material name="block1_mat" rgba="0.8 0.2 0.8 1"/>')
        T('<material name="block2_mat" rgba="0.1 0.8 0.85 1"/>')
        T('<material name="clock_face_mat" rgba="0.98 0.98 0.95 1"/>')
        T('<material name="clock_hand_mat" rgba="0.05 0.05 0.05 1"/>')

    # ---- the room ----------------------------------------------------------------------
    def walls(self):
        h, t, z = self.half, WALL_T, WALL_H / 2
        W = self.world.append
        W(f'<geom name="floor" size="0 0 0.05" pos="0 0 0" type="plane" material="floor_mat"/>')
        W(f'<geom name="wall_px" type="box" size="{t} {h} {z}" pos="{h} 0 {z}" material="wall0_mat"/>')
        W(f'<geom name="wall_nx" type="box" size="{t} {h} {z}" pos="{-h} 0 {z}" material="wall1_mat"/>')
        W(f'<geom name="wall_py" type="box" size="{h} {t} {z}" pos="0 {h} {z}" material="wall2_mat"/>')
        W(f'<geom name="wall_ny" type="box" size="{h} {t} {z}" pos="0 {-h} {z}" material="wall3_mat"/>')
        for n in ("wall_px", "wall_nx", "wall_py", "wall_ny"):
            self.record(n, "static", "wall", 0, 0, z, h)

    def rug(self):
        # Non-colliding, group 0: the camera sees a pattern on the floor, the ToF reads floor.
        x, y = self.place(0.5, margin=0.05)
        W = self.world.append
        W(f'<geom name="rug" type="box" size="0.5 0.35 0.002" pos="{x:.3f} {y:.3f} 0.002" material="rug_mat" contype="0" conaffinity="0"/>')
        self.record("rug", "static", "rug", x, y, 0.002, 0.5)

    def table(self):
        # A ceiling the duck can walk under: top at 0.36 m, legs 0.03 m radius.
        x, y = self.place(0.45)
        yaw = self.rng.uniform(0, math.pi)
        W = self.world.append
        W(f'<body name="furn_table" pos="{x:.3f} {y:.3f} 0" euler="0 0 {yaw:.3f}">')
        W('  <geom name="furn_table_top" type="box" size="0.40 0.28 0.015" pos="0 0 0.36" material="wood_mat"/>')
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            W(f'  <geom name="furn_table_leg{i}" type="cylinder" size="0.03 0.1725" pos="{0.35*sx:.3f} {0.23*sy:.3f} 0.1725" material="wood_mat"/>')
        W('</body>')
        self.record("furn_table", "static", "table", x, y, 0, 0.45, yaw=round(yaw, 3))

    def chair(self, i):
        x, y = self.place(0.22)
        yaw = self.rng.uniform(0, 2 * math.pi)
        mat = ("stripe_mat", "dots_mat")[i % 2]
        W = self.world.append
        W(f'<body name="furn_chair{i}" pos="{x:.3f} {y:.3f} 0" euler="0 0 {yaw:.3f}">')
        W(f'  <geom name="furn_chair{i}_seat" type="box" size="0.16 0.16 0.012" pos="0 0 0.22" material="{mat}"/>')
        W(f'  <geom name="furn_chair{i}_back" type="box" size="0.012 0.16 0.16" pos="-0.15 0 0.39" material="{mat}"/>')
        for j, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            W(f'  <geom name="furn_chair{i}_leg{j}" type="cylinder" size="0.018 0.105" pos="{0.13*sx:.3f} {0.13*sy:.3f} 0.105" material="{mat}"/>')
        W('</body>')
        self.record(f"furn_chair{i}", "static", "chair", x, y, 0, 0.22, yaw=round(yaw, 3))

    def shelf(self):
        # Against a wall: a bookcase with a row of coloured "books" — a landmark for the camera.
        side = self.rng.choice(("px", "nx", "py", "ny"))
        along = self.rng.uniform(-self.half * 0.6, self.half * 0.6)
        d = self.half - WALL_T - 0.13
        x, y, yaw = {"px": (d, along, math.pi), "nx": (-d, along, 0.0),
                     "py": (along, d, -math.pi / 2), "ny": (along, -d, math.pi / 2)}[side]
        self.placed.append((x, y, 0.45))
        W = self.world.append
        W(f'<body name="furn_shelf" pos="{x:.3f} {y:.3f} 0" euler="0 0 {yaw:.3f}">')
        W('  <geom name="furn_shelf_body" type="box" size="0.12 0.40 0.30" pos="0 0 0.30" material="wood_mat"/>')
        books = ["0.8 0.1 0.1 1", "0.1 0.5 0.9 1", "0.95 0.8 0.1 1", "0.2 0.7 0.3 1", "0.6 0.2 0.7 1", "0.9 0.5 0.1 1"]
        for k, rgba in enumerate(books):
            yy = -0.33 + k * 0.13
            W(f'  <geom name="furn_shelf_book{k}" type="box" size="0.10 0.05 0.14" pos="0.03 {yy:.3f} 0.74" rgba="{rgba}"/>')
        W('</body>')
        self.record("furn_shelf", "static", "shelf", x, y, 0, 0.45, wall=side)

    def clock(self):
        # On a wall at 0.45 m: a face and a hand on a free-spinning hinge the host turns
        # (`clock_hand`, one qpos entry). Out of the duck's reach; only the camera changes.
        side = self.rng.choice(("px", "nx", "py", "ny"))
        along = self.rng.uniform(-self.half * 0.5, self.half * 0.5)
        d = self.half - WALL_T - 0.02
        x, y, yaw = {"px": (d, along, math.pi), "nx": (-d, along, 0.0),
                     "py": (along, d, -math.pi / 2), "ny": (along, -d, math.pi / 2)}[side]
        W = self.world.append
        W(f'<body name="clock" pos="{x:.3f} {y:.3f} 0.45" euler="0 0 {yaw:.3f}">')
        W('  <geom name="clock_face" type="cylinder" size="0.10 0.008" euler="0 1.5708 0" material="clock_face_mat" contype="0" conaffinity="0"/>')
        # gravcomp: the hand is off its axis, so without it gravity makes a pendulum that the
        # host's constant-rate spin cannot hold (it hung at the bottom on the first run).
        W('  <body name="clock_hand" pos="0.012 0 0" gravcomp="1">')
        W('    <joint name="clock_hand" type="hinge" axis="1 0 0" damping="0" limited="false"/>')
        W('    <geom name="clock_hand_geom" type="box" size="0.004 0.006 0.045" pos="0 0 0.045" material="clock_hand_mat" contype="0" conaffinity="0" mass="0.005"/>')
        W('  </body>')
        W('</body>')
        self.hinges.append("clock_hand")
        self.qpos_layout.append(("clock_hand", "hinge"))
        self.record("clock", "self_changing", "clock", x, y, 0.45, 0.10, wall=side)

    def ball(self, i):
        r = self.rng.uniform(0.035, 0.06)
        x, y = self.place(r)
        W = self.world.append
        W(f'<body name="obj_ball{i}" pos="{x:.3f} {y:.3f} {r:.3f}">')
        W('  <freejoint/>')
        W(f'  <geom name="obj_ball{i}_geom" type="sphere" size="{r:.3f}" material="ball{i % 3}_mat" mass="0.04" friction="0.6 0.005 0.0001"/>')
        W('</body>')
        self.free_bodies.append((f"obj_ball{i}", x, y, r))
        self.qpos_layout.append((f"obj_ball{i}", "free"))
        self.record(f"obj_ball{i}", "movable", "ball", x, y, r, r)

    def block(self, i):
        s = self.rng.uniform(0.03, 0.05)
        x, y = self.place(s * 1.5)
        yaw = self.rng.uniform(0, math.pi / 2)
        W = self.world.append
        W(f'<body name="obj_block{i}" pos="{x:.3f} {y:.3f} {s:.3f}" euler="0 0 {yaw:.3f}">')
        W('  <freejoint/>')
        W(f'  <geom name="obj_block{i}_geom" type="box" size="{s:.3f} {s:.3f} {s:.3f}" material="block{i % 3}_mat" mass="0.08" friction="0.5"/>')
        W('</body>')
        # a freejoint's qpos is the body pose in the world: pos + the quaternion of `euler`
        self.free_bodies.append((f"obj_block{i}", x, y, s, yaw))
        self.qpos_layout.append((f"obj_block{i}", "free"))
        self.record(f"obj_block{i}", "movable", "block", x, y, s, s, yaw=round(yaw, 3))

    # ---- assembly ----------------------------------------------------------------------
    def build(self):
        self.textures()
        self.walls()
        self.rug()
        self.table()
        for i in range(self.n_chairs):
            self.chair(i)
        self.shelf()
        self.clock()
        for i in range(self.n_balls):
            self.ball(i)
        for i in range(self.n_blocks):
            self.block(i)

    def keyframes(self):
        lines = []
        for name, qpos, ctrl in arena_keyframes():
            extra = []
            # qpos order is worldbody order: the clock (a hinge, one entry) is defined before the
            # free bodies, so its entry comes first. Keep this in step with qpos_layout.
            extra += ["0"] * len(self.hinges)
            for fb in self.free_bodies:
                if len(fb) == 4:
                    _, x, y, z = fb
                    extra += [f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", "1", "0", "0", "0"]
                else:
                    _, x, y, z, yaw = fb
                    extra += [f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{math.cos(yaw/2):.6f}", "0", "0", f"{math.sin(yaw/2):.6f}"]
            lines.append(f'<key name="{name}" qpos="{" ".join(qpos + extra)}" ctrl="{" ".join(ctrl)}"/>')
        return lines

    def xml(self):
        nl = "\n        "
        return f'''<mujoco model="scene_playroom">
    <!-- GENERATED by mj_host/tools/playroom_gen.py --seed {self.seed} --half {self.half}
         --balls {self.n_balls} --blocks {self.n_blocks} --chairs {self.n_chairs}.
         Do not edit by hand: regenerate. The manifest is beside this file. -->
    <include file="robot_allcollisions.xml" />

    <visual>
        <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35" specular="0.1 0.1 0.1" />
        <rgba haze="0.9 0.92 0.95 1" />
        <global azimuth="160" elevation="-20" />
    </visual>

    <asset>
        {nl.join(self.assets)}
    </asset>

    <worldbody>
        <light pos="0 0 3.5" dir="0 0 -1" directional="true" />
        <light pos="{self.half:.1f} {-self.half:.1f} 2.5" dir="-0.5 0.5 -1" diffuse="0.3 0.3 0.3" directional="true" />
        {nl.join(self.world)}
    </worldbody>
    <keyframe>
        {nl.join(self.keyframes())}
    </keyframe>
</mujoco>
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--half", type=float, default=2.0, help="half-width of the square room (m)")
    ap.add_argument("--balls", type=int, default=2)
    ap.add_argument("--blocks", type=int, default=2)
    ap.add_argument("--chairs", type=int, default=2)
    ap.add_argument("--out", default="scene_playroom.xml", help="file name in mj_host/models/microduck (or a path)")
    ap.add_argument("--check", action="store_true", help="load the result through the host when it is built")
    a = ap.parse_args()

    room = Room(a.seed, a.half, a.balls, a.blocks, a.chairs)
    room.build()
    text = room.xml()
    out = Path(a.out) if Path(a.out).is_absolute() or "/" in a.out else MODEL_DIR / a.out
    out.write_text(text)
    room.manifest["xml_sha256"] = hashlib.sha256(text.encode()).hexdigest()[:16]
    room.manifest["free_bodies"] = [fb[0] for fb in room.free_bodies]
    room.manifest["hinges"] = room.hinges
    room.manifest["qpos_robot"] = 21
    # The qpos address of every non-robot entry, for the sweep's displacement metric and
    # anything that reads the JSONL's qpos: [name, adr, n] in order after the robot's 21.
    adr, layout = 21, []
    for name, kind in room.qpos_layout:
        n = 7 if kind == "free" else 1
        layout.append([name, adr, n]); adr += n
    room.manifest["qpos_layout"] = layout
    room.manifest["nq"] = adr
    man = out.with_suffix(".manifest.json")
    man.write_text(json.dumps(room.manifest, indent=1) + "\n")
    objs = room.manifest["objects"]
    print(f"playroom seed {a.seed}: {out.name}  half {a.half} m  {len(objs)} objects "
          f"({sum(o['cls']=='static' for o in objs)} static, {sum(o['cls']=='movable' for o in objs)} movable, "
          f"{sum(o['cls']=='self_changing' for o in objs)} self-changing)  sha {room.manifest['xml_sha256']}")
    for o in objs:
        if o["cls"] != "static" or o["kind"] not in ("wall",):
            print(f"  {o['name']:16s} {o['cls']:14s} at ({o['x']:+.2f}, {o['y']:+.2f})")
    if a.check:
        if not HOST.exists():
            sys.exit("--check: the host is not built (./mj_host/run.sh build)")
        r = subprocess.run([str(HOST), "--load-only", str(out)], capture_output=True, text=True, cwd=str(REPO / "mj_host"))
        tail = [l for l in r.stdout.splitlines() if l.strip()][-3:]
        print("\n".join(tail))
        if r.returncode != 0:
            sys.exit(r.stderr or "load failed")


if __name__ == "__main__":
    main()
