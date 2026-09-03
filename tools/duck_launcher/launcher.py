#!/usr/bin/env python3
"""duck_launcher — the Microduck experiment launcher.

The mj_host counterpart of the Godot launcher: pick a config, a seed, a duration,
where to start from (from scratch with identification episodes, or a saved brain),
a shove schedule, and whether to watch it live or run it headless; press Launch.
The exact host command is always on screen, so a run can be repeated from a shell
verbatim — every run is deterministic, and a live window of the same command is
the same run.

    ./mj_host/run.sh launcher              # the window
    launcher.py --selftest                 # build every preset's command, no window
    launcher.py --print                    # the command for the saved selections

Configs appear under their `metadata.name` when they carry `metadata.launcher_rank`
(lower ranks first); "show all" lists the rest by filename.  Three classes, told apart
by the name's prefix and the rank band:

    ★ STACK / ★ PIPELINE n/3   milestones, rank < 100      — the promoted state
    PROBE · …                  instruments, rank 100–999   — push test, envelope, guard
    R<nn> · …                  the current series, rank 1000 + nn — tests in the order
                               they were made; the newest is marked ◀ latest

Mint the next test with newtest.py (it takes the next R number, writes the config and
its preset); at a milestone prune the refuted ones out of the whitelist (drop their
rank; the files stay) and promote the winner to a ★ name.  Presets (presets.json
beside this file) carry the same prefixes and an optional "series" number.

Nothing in this file simulates or decides anything: it builds argv for
mj_host/build/ogma_mjhost and, for watching, hands it to tools/duck_viewer/view.py.
"""

import argparse
import json
import os
import random
import shlex
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HOST = REPO / "mj_host/build/ogma_mjhost"
CONFIG_DIR = REPO / "mj_host/configs"
CKPT_DIR = REPO / "mj_host/checkpoints"
MODEL_DIR = REPO / "mj_host/models/microduck"
LOG_DIR = REPO / "mj_host/log"                 # gitignored, on the main disk
RUNS_DIR = LOG_DIR / "launcher"
STATE_FILE = LOG_DIR / "launcher_state.json"
VIEWER = REPO / "tools/duck_viewer/view.py"
VIEWER_PY = REPO / "tools/duck_viewer/.venv/bin/python"
REPORT = REPO / "mj_host/tools/push_report.py"
PRESETS_FILE = Path(__file__).with_name("presets.json")
GATES = REPO / "mj_host/run.sh"

MB_PER_SIM_HOUR = 190        # measured 2026-09-02; the reason logs live on the main disk
HOST_AMP_DEFAULT = 0.35
LATEST = "(latest saved brain)"
NONE = "(none)"

DEFAULTS = dict(
    preset="", config="a1v2_r12c_whole.json", show_all=False,
    mode="brain", seed=2, seed_random=False, battery=False, seeds="1-6",
    secs=600.0, scene="scene.xml", output="watch",
    start="checkpoint", ident_every=12, ident_until=3000,
    checkpoint="duck_pipeline_s2.json", save_brain=False,
    push=0.0, push_every=60.0, push_hold=0.1, push_from=30.0,
    step_lean=0.0, step_twist=0.0, step_twist_s=0.5, step_trigger="att", step_att=0.06,
    walk_from=-1.0, walk_secs=4.0, walk_vx=0.3, walk_vy=0.0, walk_vyaw=0.0,
    amp=HOST_AMP_DEFAULT, freeze_after=0.0, no_tilt_gate=False, servo_filter=False,
    noise=0.0, stub_amp=0.25, stub_drift=0.08,
)


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def scan_configs():
    """Every config in mj_host/configs with its display name, description and rank."""
    out = []
    for p in sorted(CONFIG_DIR.glob("*.json")):
        try:
            c = json.loads(p.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        meta = c.get("metadata", {}) if isinstance(c, dict) else {}
        desc = c.get("description") or meta.get("description") or ""
        rank = meta.get("launcher_rank")
        out.append(dict(file=p.name, name=meta.get("name", p.stem),
                        desc=desc, rank=rank if isinstance(rank, (int, float)) else None))
    return out


SERIES_RANK = 1000     # ranks at or above this are the current test series (1000 + R number)
LATEST = " ◀ latest"


def config_label(entry, latest=False):
    if entry["rank"] is not None:
        return entry["name"] + (LATEST if latest else "")
    head = entry["desc"].split(".")[0][:70]
    return f"{entry['file']}  — {head}" if head else entry["file"]


def latest_series(entries):
    """The highest-ranked series entry (rank >= SERIES_RANK), or None."""
    tests = [e for e in entries if e.get("rank") is not None and e["rank"] >= SERIES_RANK]
    return max(tests, key=lambda e: e["rank"]) if tests else None


def preset_key(p):
    """★ milestones, then PROBE instruments, then the series in order."""
    name = p.get("name", "")
    if name.startswith("★"):
        return (0, 0, name)
    if name.startswith("PROBE"):
        return (1, 0, name)
    if name.startswith("PIPELINE"):
        return (1, 1, name)
    return (2, p.get("series", 0), name)


def preset_label(p, latest=False):
    return p["name"] + (LATEST if latest else "")


def saved_brains():
    """Newest first: every *.brain.json the launcher (or a shell) left under mj_host/log."""
    found = [p for p in LOG_DIR.rglob("*.brain.json")] if LOG_DIR.exists() else []
    found.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return found


def scan_checkpoints():
    """[(label, path-or-None)] — none, latest, the repo checkpoints, then recent saved brains."""
    items = [(NONE, None), (LATEST, LATEST)]
    for p in sorted(CKPT_DIR.glob("*.json")):
        items.append((p.name, str(p)))
    for p in saved_brains()[:25]:
        items.append((f"log: {p.relative_to(LOG_DIR)}", str(p)))
    return items


def resolve_checkpoint(label_or_path):
    if label_or_path in (None, "", NONE):
        return None
    if label_or_path == LATEST:
        brains = saved_brains()
        return str(brains[0]) if brains else None
    p = Path(label_or_path)
    if p.is_absolute() and p.exists():
        return str(p)
    if (CKPT_DIR / label_or_path).exists():
        return str(CKPT_DIR / label_or_path)
    if label_or_path.startswith("log: ") and (LOG_DIR / label_or_path[5:]).exists():
        return str(LOG_DIR / label_or_path[5:])
    return None


# ---------------------------------------------------------------------------
# The command — pure functions, testable without a window
# ---------------------------------------------------------------------------

def parse_seeds(text):
    """'1-6' -> [1..6]; '1,2,5' -> [1,2,5]; mixtures allowed."""
    seeds = []
    for part in str(text).replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            seeds.extend(range(int(a), int(b) + 1))
        else:
            seeds.append(int(part))
    return seeds


def fmt(x):
    """Numbers as a person would type them on the command line."""
    if isinstance(x, float) and x.is_integer():
        return str(int(x))
    return str(x)


def run_name(s, seed):
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    stem = Path(s["config"]).stem if s["mode"] in ("brain", "level2") else s["mode"]
    name = f"{stamp}_{stem}_s{seed}"
    if float(s["push"]) > 0:
        name += f"_push{fmt(float(s['push']))}"
    return name


def host_args(s, seed, save_brain_path=None):
    """argv after the binary: mode flag first, the scene last."""
    mode = s["mode"]
    a = [f"--{mode}"]
    if mode in ("brain", "level2"):
        a += ["--graph", str(CONFIG_DIR / s["config"])]
    a += ["--secs", fmt(float(s["secs"])), "--seed", str(int(seed))]
    if mode == "brain":
        ckpt = resolve_checkpoint(s["checkpoint"]) if s["start"] == "checkpoint" else None
        if s["start"] == "scratch" and int(s["ident_every"]) > 0:
            a += ["--ident-every", str(int(s["ident_every"])),
                  "--ident-until", str(int(s["ident_until"]))]
        if ckpt:
            a += ["--load-brain", ckpt]
        if save_brain_path:
            a += ["--save-brain", str(save_brain_path)]
        if abs(float(s["amp"]) - HOST_AMP_DEFAULT) > 1e-9:
            a += ["--amp", fmt(float(s["amp"]))]
        if float(s["freeze_after"]) > 0:
            a += ["--freeze-after", fmt(float(s["freeze_after"]))]
        if s["no_tilt_gate"]:
            a += ["--no-tilt-gate"]
        if s["servo_filter"]:
            a += ["--servo-filter"]
    elif mode == "hold":
        if float(s["noise"]) > 0:
            a += ["--noise", fmt(float(s["noise"]))]
    elif mode == "stub":
        a += ["--stub-amp", fmt(float(s["stub_amp"])), "--stub-drift", fmt(float(s["stub_drift"]))]
    if float(s["push"]) > 0 and mode in ("brain", "hold"):
        a += ["--push", fmt(float(s["push"])), "--push-every", fmt(float(s["push_every"])),
              "--push-hold", fmt(float(s["push_hold"]))]
        if float(s["push_from"]) > 0:
            a += ["--push-from", fmt(float(s["push_from"]))]
    if mode == "brain" and float(s["step_lean"]) > 0:
        if s.get("step_trigger", "att") == "att":
            a += ["--step-att", fmt(float(s["step_att"]))]
        else:
            a += ["--step-lean", fmt(float(s["step_lean"]))]
        a += ["--step-twist", fmt(float(s["step_twist"])), "--step-twist-secs", fmt(float(s["step_twist_s"]))]
    if mode == "brain" and float(s["walk_from"]) >= 0:
        a += ["--walk-from", fmt(float(s["walk_from"])), "--walk-secs", fmt(float(s["walk_secs"])),
              "--walk-vx", fmt(float(s["walk_vx"])), "--walk-vy", fmt(float(s["walk_vy"])), "--walk-vyaw", fmt(float(s["walk_vyaw"]))]
    if mode == "level2" and save_brain_path:
        a += ["--save-brain", str(save_brain_path)]
    if s["scene"] and s["scene"] != "scene.xml":
        a += [str(MODEL_DIR / s["scene"])]
    return a


def plan(s, seed, name=None):
    """Everything one launch needs: the host argv, the viewer argv (watch mode), file paths."""
    name = name or run_name(s, seed)
    jsonl = RUNS_DIR / f"{name}.jsonl"
    err = RUNS_DIR / f"{name}.err"
    brain = RUNS_DIR / f"{name}.brain.json" if (s["mode"] in ("brain", "level2") and s["save_brain"]) else None
    args = host_args(s, seed, brain)
    host_cmd = [str(HOST)] + args
    watch_cmd = [str(VIEWER_PY), str(VIEWER), "live", "--save", str(jsonl),
                 f"--host-mode={args[0]}", "--"] + args[1:]
    return dict(name=name, jsonl=jsonl, err=err, brain=brain,
                host_cmd=host_cmd, watch_cmd=watch_cmd, watch=(s["output"] == "watch"))


def shell_line(p):
    """What to paste into a shell to reproduce the run."""
    rel = lambda x: os.path.relpath(x, REPO)
    host = " ".join(shlex.quote(rel(x) if x.startswith(str(REPO)) else x) for x in p["host_cmd"])
    if p["watch"]:
        sub = {"--brain": "brain", "--hold": "watch", "--stub": "stub", "--level2": "level2"}[p["host_cmd"][1]]
        return f"./mj_host/run.sh {sub} {host.split(' ', 2)[2]}"
    return f"{host} 2> {rel(p['err'])} | grep '^{{' > {rel(p['jsonl'])}"


def estimate_mb(s):
    return float(s["secs"]) / 3600.0 * MB_PER_SIM_HOUR


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

def load_state():
    s = dict(DEFAULTS)
    try:
        s.update({k: v for k, v in json.loads(STATE_FILE.read_text()).items() if k in DEFAULTS})
    except (OSError, json.JSONDecodeError):
        pass
    return s


def save_state(s):
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps(s, indent=1))
    except OSError:
        pass


def load_presets():
    try:
        presets = json.loads(PRESETS_FILE.read_text())
    except (OSError, json.JSONDecodeError):
        return []
    return sorted(presets, key=preset_key)


def preset_labels(presets):
    series = [p for p in presets if p.get("series")]
    newest = max(series, key=lambda p: p["series"]) if series else None
    return [preset_label(p, p is newest) for p in presets]


# ---------------------------------------------------------------------------
# Launching (shared by the window and --print/--selftest)
# ---------------------------------------------------------------------------

class Run:
    def __init__(self, p, proc, seed):
        self.p, self.proc, self.seed = p, proc, seed
        self.started = time.time()
        self.ended = None          # wall clock at exit; the elapsed column stops there
        self.stopped = False       # we terminated it (a Stop is not a failure)
        self.rc = None

    @property
    def status(self):
        if self.rc is None:
            return "running"
        if self.stopped:
            return "stopped"
        return "done" if self.rc == 0 else f"exit {self.rc}"

    @property
    def elapsed(self):
        return (self.ended or time.time()) - self.started

    def poll(self):
        if self.rc is None:
            self.rc = self.proc.poll()
            if self.rc is not None:
                self.ended = time.time()
        return self.rc

    def stop(self):
        if self.poll() is None:
            self.stopped = True
            self.proc.terminate()


def launch(p):
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    if not HOST.exists():
        raise RuntimeError(f"host not built: {HOST}\n  ./mj_host/run.sh build")
    if p["watch"]:
        if not VIEWER_PY.exists():
            raise RuntimeError(f"viewer venv missing: {VIEWER_PY}\n  tools/duck_viewer/setup.sh")
        err = open(p["err"], "w")
        proc = subprocess.Popen(p["watch_cmd"], cwd=str(REPO), stdout=err, stderr=subprocess.STDOUT)
        return proc
    # Headless: JSONL to disk, the host's summary to .err.  The grep the shell version
    # uses is only there to drop stray lines; the host writes nothing else to stdout.
    out = open(p["jsonl"], "w")
    err = open(p["err"], "w")
    return subprocess.Popen(p["host_cmd"], cwd=str(REPO), stdout=out, stderr=err)


def tail(path, n=40):
    try:
        lines = Path(path).read_text(errors="replace").splitlines()
    except OSError:
        return ""
    return "\n".join(lines[-n:])


# ---------------------------------------------------------------------------
# The window
# ---------------------------------------------------------------------------

def build_window():
    import tkinter as tk
    from tkinter import ttk, messagebox

    root = tk.Tk()
    root.title("Microduck launcher")
    root.minsize(980, 760)
    try:
        ttk.Style().theme_use("clam")
    except tk.TclError:
        pass

    state = load_state()
    presets = load_presets()
    configs = scan_configs()
    runs = []

    # -- variables ---------------------------------------------------------
    V = {}
    kinds = dict(show_all=tk.BooleanVar, seed_random=tk.BooleanVar, battery=tk.BooleanVar,
                 save_brain=tk.BooleanVar, no_tilt_gate=tk.BooleanVar, servo_filter=tk.BooleanVar)
    for k, v in state.items():
        V[k] = kinds.get(k, tk.StringVar)(value=fmt(v) if isinstance(v, float) else v)

    def S():
        """The selections as plain values (what the pure functions take)."""
        s = {}
        for k in DEFAULTS:
            v = V[k].get()
            d = DEFAULTS[k]
            if isinstance(d, bool):
                s[k] = bool(v)
            elif isinstance(d, int):
                try: s[k] = int(float(v))
                except (TypeError, ValueError): s[k] = d
            elif isinstance(d, float):
                try: s[k] = float(v)
                except (TypeError, ValueError): s[k] = d
            else:
                s[k] = str(v)
        return s

    # -- layout helpers ------------------------------------------------------
    main = ttk.Frame(root, padding=8)
    main.pack(fill="both", expand=True)
    main.columnconfigure(0, weight=1)
    main.columnconfigure(1, weight=1)

    def frame(title, col, row, colspan=1, sticky="nsew"):
        f = ttk.LabelFrame(main, text=title, padding=6)
        f.grid(column=col, row=row, columnspan=colspan, sticky=sticky, padx=4, pady=3)
        return f

    def spin(parent, var, lo, hi, inc, width=8):
        return ttk.Spinbox(parent, textvariable=var, from_=lo, to=hi, increment=inc, width=width)

    # -- experiment ----------------------------------------------------------
    fx = frame("Experiment", 0, 0, colspan=2)
    fx.columnconfigure(1, weight=1)
    ttk.Label(fx, text="Preset").grid(column=0, row=0, sticky="w")
    preset_box = ttk.Combobox(fx, values=preset_labels(presets), state="readonly", width=70)
    preset_box.grid(column=1, row=0, sticky="ew", padx=4)
    preset_hint = ttk.Label(fx, text="", wraplength=880, foreground="#555")
    preset_hint.grid(column=0, row=1, columnspan=3, sticky="w", pady=(0, 4))

    ttk.Label(fx, text="Config").grid(column=0, row=2, sticky="w")
    config_box = ttk.Combobox(fx, state="readonly", width=70)
    config_box.grid(column=1, row=2, sticky="ew", padx=4)
    ttk.Checkbutton(fx, text="show all", variable=V["show_all"]).grid(column=2, row=2, sticky="w")
    desc = tk.Text(fx, height=5, wrap="word", font=("TkDefaultFont", 9))
    desc.grid(column=0, row=3, columnspan=3, sticky="ew", pady=(4, 0))
    desc.configure(state="disabled")

    config_entries = []   # the entries currently shown, in order

    def refresh_configs(*_):
        nonlocal config_entries
        curated = [c for c in configs if c["rank"] is not None]
        curated.sort(key=lambda c: c["rank"])
        rest = [c for c in configs if c["rank"] is None]
        config_entries = curated + (rest if V["show_all"].get() else [])
        newest = latest_series(curated)
        config_box["values"] = [config_label(c, c is newest) for c in config_entries]
        cur = V["config"].get()
        idx = next((i for i, c in enumerate(config_entries) if c["file"] == cur), None)
        if idx is None and config_entries:
            idx = 0
            V["config"].set(config_entries[0]["file"])
        if idx is not None:
            config_box.current(idx)
        show_desc()

    def show_desc(*_):
        cur = V["config"].get()
        e = next((c for c in configs if c["file"] == cur), None)
        desc.configure(state="normal")
        desc.delete("1.0", "end")
        if e:
            desc.insert("1.0", f"{e['file']}\n{e['desc']}")
        desc.configure(state="disabled")

    def on_config_pick(_):
        i = config_box.current()
        if 0 <= i < len(config_entries):
            V["config"].set(config_entries[i]["file"])
            V["preset"].set("")
            preset_box.set("")
            preset_hint.configure(text="")

    config_box.bind("<<ComboboxSelected>>", on_config_pick)
    V["show_all"].trace_add("write", refresh_configs)

    def apply_preset(_=None):
        i = preset_box.current()
        if not (0 <= i < len(presets)):
            return
        p = presets[i]
        for k, v in p.get("state", {}).items():
            if k in V:
                V[k].set(v)
        V["preset"].set(p["name"])
        preset_hint.configure(text=p.get("hint", ""))
        refresh_configs()
        refresh_checkpoints()

    preset_box.bind("<<ComboboxSelected>>", apply_preset)

    # -- run -----------------------------------------------------------------
    fr = frame("Run", 0, 1)
    ttk.Label(fr, text="Mode").grid(column=0, row=0, sticky="w")
    mrow = ttk.Frame(fr); mrow.grid(column=1, row=0, columnspan=3, sticky="w")
    for txt, val in (("brain (ogma)", "brain"), ("level-2 (intent)", "level2"), ("scaffold hold", "hold"), ("stub", "stub")):
        ttk.Radiobutton(mrow, text=txt, value=val, variable=V["mode"]).pack(side="left", padx=(0, 8))

    ttk.Label(fr, text="Seed").grid(column=0, row=1, sticky="w")
    seed_spin = spin(fr, V["seed"], 0, 2**31 - 1, 1, width=12)
    seed_spin.grid(column=1, row=1, sticky="w")
    ttk.Checkbutton(fr, text="random", variable=V["seed_random"]).grid(column=2, row=1, sticky="w")
    seed_note = ttk.Label(fr, text="", foreground="#a50")
    seed_note.grid(column=1, row=2, columnspan=3, sticky="w")

    ttk.Checkbutton(fr, text="battery — seeds", variable=V["battery"]).grid(column=0, row=3, sticky="w")
    ttk.Entry(fr, textvariable=V["seeds"], width=12).grid(column=1, row=3, sticky="w")
    ttk.Label(fr, text="(headless, in parallel)").grid(column=2, row=3, columnspan=2, sticky="w")

    ttk.Label(fr, text="Duration (s)").grid(column=0, row=4, sticky="w")
    spin(fr, V["secs"], 1, 86400, 60, width=12).grid(column=1, row=4, sticky="w")
    size_note = ttk.Label(fr, text="")
    size_note.grid(column=2, row=4, columnspan=2, sticky="w")

    ttk.Label(fr, text="Scene").grid(column=0, row=5, sticky="w")
    ttk.Combobox(fr, textvariable=V["scene"], values=["scene.xml", "scene_walk.xml"],
                 state="readonly", width=14).grid(column=1, row=5, sticky="w")

    ttk.Label(fr, text="Output").grid(column=0, row=6, sticky="w")
    orow = ttk.Frame(fr); orow.grid(column=1, row=6, columnspan=3, sticky="w")
    ttk.Radiobutton(orow, text="watch live (real time)", value="watch", variable=V["output"]).pack(side="left")
    ttk.Radiobutton(orow, text="headless (≈80× real time)", value="headless", variable=V["output"]).pack(side="left", padx=8)

    # -- start from ------------------------------------------------------------
    fs = frame("Start from", 1, 1)
    fs.columnconfigure(1, weight=1)
    ttk.Radiobutton(fs, text="from scratch, identification episodes:", value="scratch",
                    variable=V["start"]).grid(column=0, row=0, columnspan=2, sticky="w")
    irow = ttk.Frame(fs); irow.grid(column=0, row=1, columnspan=2, sticky="w", padx=(20, 0))
    ttk.Label(irow, text="every").pack(side="left")
    spin(irow, V["ident_every"], 0, 1000, 1, width=6).pack(side="left", padx=(2, 8))
    ttk.Label(irow, text="brain ticks, until").pack(side="left")
    spin(irow, V["ident_until"], 0, 100000, 100, width=8).pack(side="left", padx=2)
    ttk.Label(irow, text="(0 = none)").pack(side="left", padx=(6, 0))
    ttk.Radiobutton(fs, text="a saved brain:", value="checkpoint",
                    variable=V["start"]).grid(column=0, row=2, sticky="w", pady=(6, 0))
    ckpt_box = ttk.Combobox(fs, state="readonly", width=44)
    ckpt_box.grid(column=0, row=3, columnspan=2, sticky="ew", padx=(20, 0))
    ttk.Radiobutton(fs, text="nothing (a fresh brain, no episodes)", value="none",
                    variable=V["start"]).grid(column=0, row=4, columnspan=2, sticky="w", pady=(6, 0))
    ttk.Checkbutton(fs, text="save the brain at the end (.brain.json beside the log)",
                    variable=V["save_brain"]).grid(column=0, row=5, columnspan=2, sticky="w", pady=(8, 0))

    ckpt_items = []

    def refresh_checkpoints(*_):
        nonlocal ckpt_items
        ckpt_items = scan_checkpoints()
        ckpt_box["values"] = [lbl for lbl, _ in ckpt_items]
        cur = V["checkpoint"].get()
        idx = next((i for i, (lbl, path) in enumerate(ckpt_items)
                    if lbl == cur or (path and path.endswith(cur))), None)
        if idx is None:
            idx = 0 if not cur else next((i for i, (lbl, _) in enumerate(ckpt_items) if lbl == LATEST), 0)
        ckpt_box.current(idx)

    def on_ckpt_pick(_):
        i = ckpt_box.current()
        if 0 <= i < len(ckpt_items):
            V["checkpoint"].set(ckpt_items[i][0])
            V["start"].set("checkpoint")

    ckpt_box.bind("<<ComboboxSelected>>", on_ckpt_pick)

    # -- perturbation ------------------------------------------------------------
    fp = frame("Perturbation — shoves on the trunk, rotating heading", 0, 2)
    for i, (lbl, key, lo, hi, inc) in enumerate((
            ("force (N)   0 = off", "push", 0, 100, 0.5),
            ("every (s)", "push_every", 0.5, 3600, 1),
            ("hold (s)", "push_hold", 0.02, 5, 0.1),
            ("from (s)", "push_from", 0, 86400, 10))):
        ttk.Label(fp, text=lbl).grid(column=0, row=i, sticky="w")
        spin(fp, V[key], lo, hi, inc).grid(column=1, row=i, sticky="w", padx=4)
    ttk.Label(fp, text="brain: lands only on brain-driven ticks; reported caught / rescued",
              foreground="#555", wraplength=380).grid(column=0, row=4, columnspan=2, sticky="w", pady=(4, 0))
    ttk.Separator(fp).grid(column=0, row=5, columnspan=2, sticky="ew", pady=6)
    ttk.Label(fp, text="Step hand-off (brain): lean (°)   0 = off").grid(column=0, row=6, sticky="w")
    spin(fp, V["step_lean"], 0, 30, 0.5).grid(column=1, row=6, sticky="w", padx=4)
    trow = ttk.Frame(fp); trow.grid(column=0, row=10, columnspan=2, sticky="w", pady=(4, 0))
    ttk.Label(trow, text="trigger:").pack(side="left")
    ttk.Radiobutton(trow, text="the brain's attitude error >", value="att", variable=V["step_trigger"]).pack(side="left", padx=(4, 0))
    spin(trow, V["step_att"], 0.01, 1, 0.01, width=6).pack(side="left", padx=(2, 8))
    ttk.Radiobutton(trow, text="the harness's lean (above)", value="lean", variable=V["step_trigger"]).pack(side="left")
    ttk.Separator(fp).grid(column=0, row=11, columnspan=2, sticky="ew", pady=6)
    ttk.Label(fp, text="Walk on request (brain): at (s)   −1 = none").grid(column=0, row=12, sticky="w")
    spin(fp, V["walk_from"], -1, 86400, 5).grid(column=1, row=12, sticky="w", padx=4)
    wrow = ttk.Frame(fp); wrow.grid(column=0, row=13, columnspan=2, sticky="w")
    for lbl, key, lo, hi in (("for (s)", "walk_secs", 0.5, 60), ("vx", "walk_vx", -0.4, 0.4), ("vy", "walk_vy", -0.3, 0.3), ("vyaw", "walk_vyaw", -1.5, 1.5)):
        ttk.Label(wrow, text=lbl).pack(side="left", padx=(0, 2)); spin(wrow, V[key], lo, hi, 0.05, width=6).pack(side="left", padx=(0, 8))
    ttk.Label(fp, text="the walker (alpha_walking, trained ±0.4 / ±0.3 / ±1.0) takes the joints for the walk, then hands back (blue ball); below ~0.25 m/s it stands",
              foreground="#555", wraplength=380).grid(column=0, row=14, columnspan=2, sticky="w", pady=(4, 0))
    ttk.Label(fp, text="twist (m/s along the lean; 0 = the walker's own stagger, measured best)").grid(column=0, row=7, sticky="w")
    spin(fp, V["step_twist"], -1, 1, 0.05).grid(column=1, row=7, sticky="w", padx=4)
    ttk.Label(fp, text="twist for (s)").grid(column=0, row=8, sticky="w")
    spin(fp, V["step_twist_s"], 0.1, 3, 0.1).grid(column=1, row=8, sticky="w", padx=4)
    ttk.Label(fp, text="past 6.5° and rising the walker takes one step for the brain, then hands back (yellow ball)",
              foreground="#555", wraplength=380).grid(column=0, row=9, columnspan=2, sticky="w", pady=(4, 0))

    # -- extras --------------------------------------------------------------------
    fe = frame("Extras", 1, 2)
    ttk.Label(fe, text="amp").grid(column=0, row=0, sticky="w")
    spin(fe, V["amp"], 0.01, 2, 0.05).grid(column=1, row=0, sticky="w", padx=4)
    ttk.Label(fe, text="freeze learning after (s, 0 = never)").grid(column=0, row=1, sticky="w")
    spin(fe, V["freeze_after"], 0, 86400, 60).grid(column=1, row=1, sticky="w", padx=4)
    ttk.Checkbutton(fe, text="--no-tilt-gate (learn at every tilt)", variable=V["no_tilt_gate"]).grid(column=0, row=2, columnspan=2, sticky="w")
    ttk.Checkbutton(fe, text="--servo-filter (robotd's deployed lag)", variable=V["servo_filter"]).grid(column=0, row=3, columnspan=2, sticky="w")
    ttk.Label(fe, text="hold: reset noise").grid(column=0, row=4, sticky="w", pady=(6, 0))
    spin(fe, V["noise"], 0, 1, 0.01).grid(column=1, row=4, sticky="w", padx=4, pady=(6, 0))
    ttk.Label(fe, text="stub: amp / drift").grid(column=0, row=5, sticky="w")
    srow = ttk.Frame(fe); srow.grid(column=1, row=5, sticky="w", padx=4)
    spin(srow, V["stub_amp"], 0, 2, 0.05, width=6).pack(side="left")
    spin(srow, V["stub_drift"], 0, 2, 0.02, width=6).pack(side="left", padx=(4, 0))

    # -- command -------------------------------------------------------------------
    fc = frame("Command (this is the run; paste it into a shell to repeat it)", 0, 3, colspan=2)
    fc.columnconfigure(0, weight=1)
    cmd_text = tk.Text(fc, height=4, wrap="word", font=("TkFixedFont", 9))
    cmd_text.grid(column=0, row=0, sticky="ew")

    def refresh_command(*_):
        s = S()
        seed = int(s["seed"])
        p = plan(s, seed, name=f"<stamp>_{Path(s['config']).stem if s['mode']=='brain' else s['mode']}_s{seed}")
        line = shell_line(p)
        if s["battery"]:
            try:
                line += f"\n… × seeds {parse_seeds(s['seeds'])}  (headless)"
            except ValueError:
                line += "\n… seeds: could not parse"
        cmd_text.configure(state="normal")
        cmd_text.delete("1.0", "end")
        cmd_text.insert("1.0", line)
        cmd_text.configure(state="disabled")
        size_note.configure(text=f"≈ {estimate_mb(s):.0f} MB of JSONL")
        resuming = s["mode"] == "brain" and s["start"] == "checkpoint" and resolve_checkpoint(s["checkpoint"])
        seed_note.configure(text="seed has no effect when resuming a saved brain (its RNG is restored)"
                            if resuming else "")
        save_state(s)

    for k in DEFAULTS:
        if k != "preset":
            V[k].trace_add("write", refresh_command)

    # -- runs ------------------------------------------------------------------------
    fl = frame("Runs", 0, 4, colspan=2)
    fl.columnconfigure(0, weight=1)
    brow = ttk.Frame(fl); brow.grid(column=0, row=0, sticky="ew")
    tree = ttk.Treeview(fl, columns=("status", "elapsed", "where"), show="headings", height=5)
    for c, w in (("status", 90), ("elapsed", 80), ("where", 700)):
        tree.heading(c, text=c); tree.column(c, width=w, anchor="w")
    tree.grid(column=0, row=1, sticky="ew", pady=4)
    out = tk.Text(fl, height=12, wrap="none", font=("TkFixedFont", 9))
    out.grid(column=0, row=2, sticky="nsew")
    fl.rowconfigure(2, weight=1)
    main.rowconfigure(4, weight=1)

    def show(text):
        out.configure(state="normal"); out.delete("1.0", "end"); out.insert("1.0", text); out.configure(state="disabled")

    def selected_run():
        sel = tree.selection()
        if not sel:
            return None
        i = tree.index(sel[0])
        return runs[i] if 0 <= i < len(runs) else None

    def add_run(r):
        runs.append(r)
        tree.insert("", "end", iid=str(len(runs) - 1), values=(r.status, "0 s", r.p["name"]))
        tree.selection_set(str(len(runs) - 1))

    def do_launch():
        s = S()
        if s["seed_random"]:
            V["seed"].set(random.randint(0, 2**31 - 1))
            s = S()
        seeds = [int(s["seed"])]
        if s["battery"]:
            try:
                seeds = parse_seeds(s["seeds"])
            except ValueError:
                messagebox.showerror("seeds", f"could not parse seeds: {s['seeds']!r}")
                return
            s["output"] = "headless"
        if s["mode"] == "brain" and s["start"] == "checkpoint" and not resolve_checkpoint(s["checkpoint"]):
            messagebox.showerror("start from", "no saved brain to resume — pick one, or start from scratch")
            return
        for seed in seeds:
            p = plan(s, seed)
            try:
                proc = launch(p)
            except (RuntimeError, OSError) as e:
                messagebox.showerror("launch", str(e))
                return
            add_run(Run(p, proc, seed))
            time.sleep(0.05)      # distinct timestamps in the names
        refresh_checkpoints()
        show(shell_line(plan(s, seeds[-1])) + "\n\n(running — the host's summary appears here when it ends)")

    def do_stop():
        r = selected_run()
        if r:
            r.stop()

    def replay(fast):
        r = selected_run()
        if not r:
            return
        if not r.p["jsonl"].exists():
            messagebox.showinfo("replay", "no JSONL for this run yet")
            return
        cmd = [str(VIEWER_PY), str(VIEWER), "replay", str(r.p["jsonl"])] + (["--fast"] if fast else [])
        subprocess.Popen(cmd, cwd=str(REPO))

    def do_replay():
        replay(fast=True)

    def do_replay_realtime():
        replay(fast=False)

    def do_report():
        r = selected_run()
        if not r or not r.p["jsonl"].exists():
            return
        try:
            res = subprocess.run([sys.executable, str(REPORT), str(r.p["jsonl"])],
                                 capture_output=True, text=True, timeout=600)
            show(res.stdout + res.stderr)
        except subprocess.TimeoutExpired:
            show("report timed out")

    def do_gates():
        show("running ./mj_host/run.sh gates …")
        root.update_idletasks()
        res = subprocess.run([str(GATES), "gates"], capture_output=True, text=True, cwd=str(REPO))
        show(res.stdout + res.stderr)

    def do_copy():
        root.clipboard_clear()
        root.clipboard_append(cmd_text.get("1.0", "end").strip())

    def do_open():
        RUNS_DIR.mkdir(parents=True, exist_ok=True)
        subprocess.Popen(["xdg-open", str(RUNS_DIR)])

    for txt, fn in (("Launch", do_launch), ("Stop", do_stop), ("Replay (fast)", do_replay),
                    ("Replay (real time)", do_replay_realtime), ("Report", do_report),
                    ("Copy command", do_copy), ("Open log dir", do_open), ("Health gates", do_gates)):
        ttk.Button(brow, text=txt, command=fn).pack(side="left", padx=(0, 6))
    ttk.Button(brow, text="Quit", command=root.destroy).pack(side="right")

    def on_select(_):
        r = selected_run()
        if r:
            show(shell_line(r.p) + "\n\n" + tail(r.p["err"]))

    tree.bind("<<TreeviewSelect>>", on_select)

    def poll():
        sel = selected_run()
        for i, r in enumerate(runs):
            was = r.rc
            r.poll()
            tree.item(str(i), values=(r.status, f"{r.elapsed:.0f} s", r.p["name"]))
            if was is None and r.rc is not None and r is sel:
                on_select(None)
                refresh_checkpoints()
        root.after(500, poll)

    # -- go --------------------------------------------------------------------------
    refresh_configs()
    refresh_checkpoints()
    if V["preset"].get():
        names = [p["name"] for p in presets]
        if V["preset"].get() in names:
            preset_box.current(names.index(V["preset"].get()))
            preset_hint.configure(text=presets[names.index(V["preset"].get())].get("hint", ""))
    refresh_command()
    poll()
    root.protocol("WM_DELETE_WINDOW", lambda: (save_state(S()), root.destroy()))
    autoclose = os.environ.get("DUCK_LAUNCHER_AUTOCLOSE_MS")
    if autoclose:
        root.after(int(autoclose), root.destroy)
    return root


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def selftest():
    presets = load_presets()
    if not presets:
        print("no presets"); return 1
    bad = 0
    for p in presets:
        s = dict(DEFAULTS); s.update(p.get("state", {}))
        try:
            pl = plan(s, int(s["seed"]), name=f"selftest_{Path(s['config']).stem}_s{s['seed']}")
            print(f"[{p['name']}]\n  {shell_line(pl)}")
            if s["mode"] == "brain" and not (CONFIG_DIR / s["config"]).exists():
                print(f"  !! config missing: {s['config']}"); bad += 1
        except Exception as e:  # noqa: BLE001 — a broken preset must be named, not hidden
            print(f"[{p['name']}] !! {e}"); bad += 1
    print(f"{len(presets)} presets, {bad} broken")
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true", help="build every preset's command; no window")
    ap.add_argument("--print", action="store_true", help="print the command for the saved selections")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    if a.print:
        s = load_state()
        print(shell_line(plan(s, int(s["seed"]), name="<stamp>")))
        return
    build_window().mainloop()


if __name__ == "__main__":
    main()
