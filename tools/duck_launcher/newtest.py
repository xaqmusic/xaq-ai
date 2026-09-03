#!/usr/bin/env python3
"""newtest — mint the next test in the launcher's series.

    tools/duck_launcher/newtest.py --from a1v2_r13_tax001.json --slug trace05 \
        --title "attitude-row trace 0.5" \
        --set motor_epm_legs.model_trace=0.5 --set motor_epm_head.model_trace=0.5 \
        --why "temporal depth on the attitude rows so a torque becomes a lean inside the model" \
        [--preset-from "★ PIPELINE 1/3"] [--seed 2] [--secs 7200] [--dry-run]

Takes the next R number after every a1v2_r<nn>* config in mj_host/configs, writes
mj_host/configs/a1v2_r<nn>_<slug>.json as a copy of --from with the --set overrides
(module.param=value; value parsed as JSON, so 0.5, 1, true, [1,2] all work), names
it "R<nn> · <title>" at launcher rank 1000 + nn, and appends a preset of the same
name (the controls copied from --preset-from, the config swapped) with "series": nn.
The design doc's R numbering and the launcher's are then the same number.
"""

import argparse
import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO / "mj_host/configs"
PRESETS_FILE = Path(__file__).with_name("presets.json")
SERIES_RANK = 1000


def next_number():
    n = 0
    for p in CONFIG_DIR.glob("a1v2_r*.json"):
        m = re.match(r"a1v2_r(\d+)", p.name)
        if m:
            n = max(n, int(m.group(1)))
    return n + 1


def parse_value(text):
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="base", required=True, help="config filename to copy (in mj_host/configs)")
    ap.add_argument("--slug", required=True, help="short filename tag, e.g. trace05")
    ap.add_argument("--title", required=True, help="what it tests, for the launcher name")
    ap.add_argument("--set", action="append", default=[], metavar="MODULE.PARAM=VALUE",
                    help="override a module param (repeatable); MODULE may be '*' for every MotorEPMv2")
    ap.add_argument("--why", default="", help="the description: motivation and what it is judged on")
    ap.add_argument("--hint", default="", help="the preset's hint (defaults to --why)")
    ap.add_argument("--preset-from", default="★ PIPELINE 1/3", help="prefix of the preset whose controls to copy")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--secs", type=float)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    base_path = CONFIG_DIR / a.base
    if not base_path.exists():
        sys.exit(f"no such config: {base_path}")
    cfg = json.loads(base_path.read_text())
    nn = next_number()
    out_name = f"a1v2_r{nn}_{a.slug}.json"
    display = f"R{nn} · {a.title}"

    for item in a.set:
        if "=" not in item or "." not in item.split("=", 1)[0]:
            sys.exit(f"--set wants MODULE.PARAM=VALUE, got {item!r}")
        target, value = item.split("=", 1)
        mod, param = target.rsplit(".", 1)
        hit = False
        for m in cfg["modules"]:
            if m["id"] == mod or (mod == "*" and m["type"] == "MotorEPMv2"):
                m.setdefault("params", {})[param] = parse_value(value)
                hit = True
        if not hit:
            sys.exit(f"no module {mod!r} in {a.base}")

    meta = cfg.setdefault("metadata", {})
    meta["name"] = display
    meta["launcher_rank"] = SERIES_RANK + nn
    if a.why:
        cfg["description"] = f"R{nn} ({a.title}), from {a.base}: {a.why}"

    presets = json.loads(PRESETS_FILE.read_text()) if PRESETS_FILE.exists() else []
    src = next((p for p in presets if p["name"].startswith(a.preset_from)), None)
    state = dict(src["state"]) if src else {"mode": "brain", "start": "scratch", "ident_every": 12,
                                            "ident_until": 3000, "secs": 7200, "output": "headless",
                                            "save_brain": True, "battery": False}
    state["config"] = out_name
    if a.seed is not None:
        state["seed"] = a.seed
    if a.secs is not None:
        state["secs"] = a.secs
    preset = {"name": display, "series": nn, "hint": a.hint or a.why, "state": state}

    print(f"config  {CONFIG_DIR / out_name}\n  name  {display}\n  rank  {SERIES_RANK + nn}")
    for item in a.set:
        print(f"  set   {item}")
    print(f"preset  {display}  (controls from {src['name'] if src else 'defaults'})")
    if a.dry_run:
        return
    (CONFIG_DIR / out_name).write_text(json.dumps(cfg, indent=2, ensure_ascii=True) + "\n")
    presets = [p for p in presets if p["name"] != display] + [preset]
    PRESETS_FILE.write_text(json.dumps(presets, indent=2, ensure_ascii=False) + "\n")
    print("written")


if __name__ == "__main__":
    main()
