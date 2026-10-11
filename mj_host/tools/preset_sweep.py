#!/usr/bin/env python3
"""preset_sweep — an n-seed level-2 sweep of a launcher PRESET, with exactly the preset's host argv (2026-10-11).

The operator watches through launcher presets; every A/B arm should be the preset it will become.  The §17.116-17.118
sweeps ran from a session-local wrapper; this is that wrapper in the repo: it reads the preset from
tools/duck_launcher/presets.json (by name prefix, e.g. "★ MV"), turns its controls into l2_sweep.py's arguments
(the checkpoint as --load-brain, the host args with their mj_host/ prefix dropped -- l2_sweep runs the host from
mj_host -- the scene, the reset noise), and runs the sweep with --full-logs into mj_host/log/ten/NAME.

  preset_sweep.py "★ MV" NAME [--config PATH] [--extra "HOST ARGS"] [--seeds 24] [--secs S] [--jobs 7] [--dry-run]

--config swaps the brain config (an arm's edited copy, e.g. a placebo with play.explore_seed changed); --extra appends
the lever's host flags.  --secs defaults to the preset's.  --dry-run prints the l2_sweep command and stops.
Score the logs with fx_study.py / places.py / phantom_chase.py (CLOUD_OBJECTS_MANIFEST=models/microduck/
scene_playroom_train.manifest.json for the train room).
"""
import argparse
import json
import shlex
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PRESETS = REPO / "tools/duck_launcher/presets.json"


def strip_prefix(a):
    return a[len("mj_host/"):] if a.startswith("mj_host/") else a


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("preset", help="a preset's name, or the start of it")
    ap.add_argument("name", help="the arm's name: logs go to mj_host/log/ten/NAME")
    ap.add_argument("--config", default=None, help="a brain config to use instead of the preset's")
    ap.add_argument("--extra", default="", help="host args appended (the lever)")
    ap.add_argument("--seeds", type=int, default=24)
    ap.add_argument("--secs", type=int, default=None)
    ap.add_argument("--jobs", type=int, default=7)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    presets = json.load(open(PRESETS))
    hits = [p for p in presets if p["name"].startswith(a.preset)]
    if len(hits) != 1:
        sys.exit(f"{len(hits)} presets start with {a.preset!r}: " + "; ".join(p["name"][:40] for p in hits))
    st = hits[0]["state"]
    if st.get("mode") != "level2":
        sys.exit(f"preset mode {st.get('mode')!r}: this runs level-2 presets only")
    host = []
    if st.get("start") == "checkpoint" and st.get("checkpoint"):
        host += ["--load-brain", f"checkpoints/{st['checkpoint']}"]
    host += [strip_prefix(x) for x in shlex.split(st.get("host_args", ""))]
    host += shlex.split(a.extra)
    cfg = Path(a.config).resolve() if a.config else REPO / "mj_host/configs" / st["config"]
    cmd = [sys.executable, str(REPO / "mj_host/tools/l2_sweep.py"), str(cfg),
           "--seeds", str(a.seeds), "--secs", str(a.secs or int(st["secs"])), "--control-from", "30",
           "--jobs", str(a.jobs), "--full-logs", "--logdir", f"log/ten/{a.name}",
           "--scene", f"models/microduck/{st['scene']}", "--noise", str(st.get("noise", 0.05)),
           "--host-args", " ".join(shlex.quote(x) for x in host)]
    print(f"preset: {hits[0]['name']}")
    print(" ".join(shlex.quote(x) for x in cmd))
    if a.dry_run:
        return
    sys.exit(subprocess.run(cmd, cwd=REPO / "mj_host").returncode)


if __name__ == "__main__":
    main()
