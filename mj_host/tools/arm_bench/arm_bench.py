#!/usr/bin/env python3
"""arm_bench -- the MicroDuck brain's cost on an ARM board, from a tape of a real run (2026-10-08).

The replay benchmark (design doc §17.111) drives the three brains -- intent, head, stop -- from a tape of everything the
MuJoCo host did to them, with no MuJoCo and no ONNX, and checks tick by tick that it reproduces the live run.  This tool
takes it to a board over ssh and back:

    arm_bench.py record                   record the tape here (★ F5, seed 1, 600 s, the launcher's own argv)
    arm_bench.py stage HOST               copy the sources, the pinned Eigen/json and the tape to HOST:/tmp/ogma_arm_bench
    arm_bench.py reference                the same tape replayed here twice, per tick: the work's x86 signature
    arm_bench.py build HOST               build ogma_brain_replay there (-mcpu=cortex-a55, otherwise the compiler's defaults)
    arm_bench.py run HOST                 timing, per-tick hardware counters, all-cores contention; results come back here
    arm_bench.py report RESULTS_DIR       the summary and the projection to Pollen's Radxa Zero 3W (RK3566, 4x A55)
    arm_bench.py all HOST                 every step (record only if the tape is missing)

HOST is anything ssh reaches (`picrawler`, the Pi 5; later a Radxa Zero 3W, where the projection becomes a measurement).
Results land in mj_host/log/arm_bench/<host>_<stamp>/ (gitignored).  The board needs cmake, g++, libzmq-dev, rsync,
taskset; nothing runs as root unless --sudo-counters is given (a kernel with perf_event_paranoid > 2).

Why a projection at all: the Pi 5's Cortex-A76 is out-of-order with 64 kB L1D and 512 kB private L2; the RK3566's A55
is in-order with 32 kB L1D and NO private L2 -- one 512 kB L3 shared by all four cores.  The Pi's microseconds do not
transfer.  What does transfer is the work: the binary is built for the A55, so it retires the same instructions on
both cores, and the A76's L2 is the size of the RK3566's whole L3, so what misses the Pi's L2 is a fair stand-in for
what the Radxa sends to DRAM.  README.md has the model and its assumptions.
"""
import argparse
import csv
import datetime as dt
import json
import math
import re
import shlex
import statistics
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
LOG = REPO / "mj_host/log/arm_bench"
DEFAULT_TAPE = LOG / "f5_s1.tape"
REMOTE = "/tmp/ogma_arm_bench"          # tmpfs on the Pi: nothing touches the SD card
NAMES = ("intent", "head", "stop")
TICK_US = 20000.0                       # 50 Hz


def sh(cmd, **kw):
    print("$ " + (cmd if isinstance(cmd, str) else shlex.join(map(str, cmd))), flush=True)
    return subprocess.run(cmd, check=True, shell=isinstance(cmd, str), **kw)


def ssh(host, script, **kw):
    return sh(["ssh", host, "bash -lc " + shlex.quote(script)], **kw)


# ---------------------------------------------------------------------------------------------------------------------
# record
# ---------------------------------------------------------------------------------------------------------------------

def cmd_record(a):
    sys.path.insert(0, str(REPO / "tools/duck_launcher"))
    import launcher as L  # the preset's argv is the launcher's, never a copy of it
    presets = [p for p in L.load_presets() if p["name"].startswith(a.preset)]
    if len(presets) != 1:
        sys.exit(f"--preset {a.preset!r} matches {len(presets)} presets: {[p['name'] for p in presets]}")
    s = dict(L.DEFAULTS); s.update(presets[0]["state"]); s["output"] = "headless"
    if a.secs:
        s["secs"] = a.secs
    seed = a.seed if a.seed is not None else int(s["seed"])
    tape = Path(a.tape)
    tape.parent.mkdir(parents=True, exist_ok=True)
    if not L.HOST.exists():
        sys.exit(f"{L.HOST} is not built: cmake -S mj_host -B mj_host/build -DMJ_HOST_WITH_BRAIN=ON && cmake --build mj_host/build")
    argv = [str(L.HOST)] + L.host_args(s, seed) + ["--record-brains", str(tape)]
    stem = tape.with_suffix("")
    Path(f"{stem}.argv").write_text(shlex.join(argv) + "\n")
    Path(f"{stem}.preset").write_text(presets[0]["name"] + "\n")
    with open(f"{stem}.err", "w") as err, open(f"{stem}.jsonl", "w") as out:
        p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=err, text=True, cwd=REPO)
        for line in p.stdout:
            if line.startswith("{"):
                out.write(line)
        if p.wait():
            sys.exit(f"host exited {p.returncode}; see {stem}.err")
    print(f"tape: {tape} ({tape.stat().st_size / 1e6:.1f} MB), preset {presets[0]['name'][:40]}…, seed {seed}")


# ---------------------------------------------------------------------------------------------------------------------
# reference: the work's x86 signature
# ---------------------------------------------------------------------------------------------------------------------

def ref_paths(tape):
    stem = Path(tape).with_suffix("")
    return Path(f"{stem}.ref_a.csv"), Path(f"{stem}.ref_b.csv")


def cmd_reference(a):
    """The tape replayed twice here, a ticks CSV each.  On x86 the replay reproduces the live run, so this is the run's
    cost tick by tick; the two copies differ only by timing noise, which sets the ceiling a board's replay can reach."""
    binary = REPO / "mj_host/build/ogma_brain_replay"
    if not binary.exists():
        sys.exit(f"{binary} is not built")
    for p in ref_paths(a.tape):
        sh(f"ulimit -v 2000000; {shlex.quote(str(binary))} {shlex.quote(a.tape)} --ticks-csv {shlex.quote(str(p))} "
           f"| grep -E 'ticks|fidelity'")


# ---------------------------------------------------------------------------------------------------------------------
# stage / build / run
# ---------------------------------------------------------------------------------------------------------------------

def find_deps():
    """The Eigen and nlohmann/json sources cpp_core pins (3.4.0, v3.11.2), from any local cpp_core build."""
    for root in (REPO, REPO.parents[2] if len(REPO.parents) > 2 else REPO, Path.home() / "xaq-ai"):
        for d in (root / "cpp_core/build/_deps", root / "mj_host/build/cpp_core_build/_deps", root / "mj_host/build/_deps"):
            if (d / "eigen-src/Eigen").is_dir() and (d / "json-src/include").is_dir():
                return d
    sys.exit("no fetched Eigen/json found: build cpp_core or mj_host once here first")


def cmd_stage(a):
    r = a.remote
    deps = find_deps()
    rev = subprocess.run(["git", "-C", str(REPO), "describe", "--always", "--dirty"], capture_output=True, text=True).stdout.strip()
    ssh(a.host, f"mkdir -p {r}/src/mj_host/src {r}/src/mj_host/tools/arm_bench {r}/deps {r}/tapes && "
                f"df -h {r} | tail -1")
    rs = ["rsync", "-a", "--delete"]
    sh(rs + ["--exclude", "build/", "--exclude", ".git", f"{REPO}/cpp_core/", f"{a.host}:{r}/src/cpp_core/"])
    sh(["rsync", "-a", f"{REPO}/mj_host/src/BrainTape.cpp", f"{REPO}/mj_host/src/BrainTape.hpp", f"{a.host}:{r}/src/mj_host/src/"])
    sh(["rsync", "-a", f"{REPO}/mj_host/tools/brain_replay.cpp", f"{a.host}:{r}/src/mj_host/tools/"])
    sh(["rsync", "-a", f"{HERE}/CMakeLists.txt", f"{HERE}/remote_run.sh", f"{a.host}:{r}/src/mj_host/tools/arm_bench/"])
    sh(rs + ["--exclude", ".git", f"{deps}/eigen-src/", f"{a.host}:{r}/deps/eigen-src/"])
    sh(rs + ["--exclude", ".git", "--exclude", "tests/", "--exclude", "docs/", f"{deps}/json-src/", f"{a.host}:{r}/deps/json-src/"])
    tape = Path(a.tape)
    if not tape.exists():
        sys.exit(f"no tape {tape}: run `arm_bench.py record` first")
    sh(["rsync", "-a", "--progress", str(tape), f"{a.host}:{r}/tapes/{tape.name}"])
    ssh(a.host, f"echo {shlex.quote(rev)} > {r}/src/REVISION && du -sh {r} && df -h {r} | tail -1")


def build_dir(a):
    return "build"


def cmd_build(a):
    r, b = a.remote, build_dir(a)
    flags = ""
    ssh(a.host, f"""set -e
cd {r}
cmake -S src/mj_host/tools/arm_bench -B {b} -DCMAKE_BUILD_TYPE=Release \\
  -DOGMA_ARM_CPU={shlex.quote(a.cpu)} \\
  -DFETCHCONTENT_SOURCE_DIR_EIGEN={r}/deps/eigen-src -DFETCHCONTENT_SOURCE_DIR_JSON={r}/deps/json-src \\
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON > {b}.cmake.log
time cmake --build {b} -j{a.jobs} --target ogma_brain_replay > {b}.log 2>&1 || {{ tail -30 {b}.log; exit 1; }}
g++ --version | head -1 > {b}/compiler.txt
echo "-mcpu={a.cpu} $(grep CMAKE_CXX_FLAGS_RELEASE: {b}/CMakeCache.txt | cut -d= -f2) rev $(cat src/REVISION)" > {b}/flags.txt
cat {b}/compiler.txt {b}/flags.txt; ls -la {b}/ogma_brain_replay""")


def cmd_run(a):
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    label = f"{a.host}_{a.label}_{stamp}" if a.label else f"{a.host}_{stamp}"
    rout = f"{a.remote}/results/{label}"
    tape = f"{a.remote}/tapes/{Path(a.tape).name}"
    extra = ["--build", build_dir(a), "--core", str(a.core), "--repeat", str(a.repeat), "--vmem-kb", str(a.vmem_kb)]
    if a.no_contention:
        extra.append("--no-contention")
    if a.sudo_counters:
        extra.append("--sudo-counters")
    ssh(a.host, shlex.join(["bash", f"{a.remote}/src/mj_host/tools/arm_bench/remote_run.sh", a.remote, tape, rout] + extra))
    local = LOG / label
    local.parent.mkdir(parents=True, exist_ok=True)
    sh(["rsync", "-a", f"{a.host}:{rout}/", f"{local}/"])
    (local / "tape.txt").write_text(f"{Path(a.tape).resolve()}\n")
    for f in ("argv", "preset"):
        src = Path(a.tape).with_suffix("." + f)
        if src.exists():
            (local / f"tape.{f}").write_text(src.read_text())
    if not all(p.exists() for p in ref_paths(a.tape)):
        cmd_reference(a)
    for p, name in zip(ref_paths(a.tape), ("ref_a.csv", "ref_b.csv")):
        (local / name).write_bytes(p.read_bytes())
    print(f"results: {local}")
    a.results = str(local)
    cmd_report(a)


# ---------------------------------------------------------------------------------------------------------------------
# report and projection
# ---------------------------------------------------------------------------------------------------------------------

# The projection's parameters, each a (low, high) bracket -- "low" is the kinder board.  Sources in README.md.
# The clock: 1.8 GHz is Pollen's own figure for the duck's board ("408 MHz of 1800", their robotd design doc, on thermal
# throttling); some retail listings give the Zero 3W 1.6 GHz.  `report --ghz` sets it.
def target(ghz=1.8):
    return {
    "name": f"Radxa Zero 3W (RK3566, 4x Cortex-A55 @ {ghz:g} GHz, 32 kB L1D, no L2, 512 kB shared L3)",
    "ghz": ghz,
    # A: whole-CPU benchmark ratio.  Geekbench 6 single-core, Pi 5 ~764-800 at 2.4 GHz vs RK3566 ~203 (boards at 1.8 GHz),
    #    rescaled to the target clock: time on the Zero 3W / time on the Pi 5 at 2.4 GHz.
    "gb6_ratio": (764 / 203 * 1.8 / ghz, 800 / 203 * 1.8 / ghz),
    # B: the in-order core's cycles per instruction when the data is in L1, for branchy scalar/small-vector C++
    "a55_cpi_core": (1.3, 2.0),
    # an L1D miss that hits the shared L3 (no L2 between them on the RK3566), cycles
    "a55_l3_hit_cycles": (25, 45),
    # how many more L1D misses the A55's 32 kB L1D takes than the A76's 64 kB
    "l1d_miss_scale": (1.0, 2.0),
    # a miss to LPDDR4 (an A76 L2 refill stands for one), ns; the in-order core waits most of it out
    "dram_ns": (110, 170),
    }


def load_json(p):
    try:
        return json.loads(Path(p).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def parse_facts(d):
    facts = {}
    p = d / "facts.txt"
    if p.exists():
        for line in p.read_text().splitlines():
            if ": " in line and not line.startswith(" "):
                k, v = line.split(": ", 1)
                facts.setdefault(k, v)
    return facts


def parse_samples(d):
    """[(t, max temp °C, mean freq MHz of the bench core, throttled, per-core busy share since the previous sample)]."""
    rows, prev = [], None
    p = d / "samples.txt"
    if not p.exists():
        return rows
    core = int(parse_facts(d).get("core", "2"))
    for line in p.read_text().splitlines():
        f = line.split(";")
        if len(f) < 5:
            continue
        t = float(f[0])
        temps = [int(x) / 1000 for x in f[1].split(",") if x.strip().lstrip("-").isdigit()]
        freqs = [int(x) / 1000 for x in f[2].split(",") if x.strip().isdigit()]
        stat = {}
        for cl in f[4].split("|"):
            parts = cl.split()
            if parts and parts[0].startswith("cpu"):
                v = list(map(int, parts[1:]))
                idle = v[3] + (v[4] if len(v) > 4 else 0)
                stat[parts[0]] = (sum(v), idle)
        busy = {}
        if prev:
            for c, (tot, idle) in stat.items():
                if c in prev and tot > prev[c][0]:
                    busy[c] = 1.0 - (idle - prev[c][1]) / (tot - prev[c][0])
        prev = stat
        rows.append((t, max(temps) if temps else float("nan"), freqs[core] if core < len(freqs) else float("nan"), f[3], busy))
    return rows


def phases(d):
    out = {}
    p = d / "phases.txt"
    if p.exists():
        for line in p.read_text().splitlines():
            t, name = line.split()
            out[name] = float(t)
    return out


def window(samples, ph, name):
    t0, t1 = ph.get(f"{name}_start"), ph.get(f"{name}_end")
    if t0 is None or t1 is None:
        return []
    return [s for s in samples if t0 <= s[0] <= t1]


def project_ticks(rows, tgt):
    """Per tick, the model's A55 time in us, low and high: (I*CPI + L1miss*L3 + L2miss*DRAM) / f."""
    f_hz = tgt["ghz"] * 1e9
    lo, hi = [], []
    for r in rows:
        ins, l1, l2 = r["instructions"], r.get("l1d_refill", 0.0), r.get("l2d_refill", 0.0)
        for out, k in ((lo, 0), (hi, 1)):
            cyc = (ins * tgt["a55_cpi_core"][k]
                   + l1 * tgt["l1d_miss_scale"][k] * tgt["a55_l3_hit_cycles"][k]
                   + l2 * tgt["dram_ns"][k] * 1e-9 * f_hz)
            out.append(cyc / f_hz * 1e6)
    return lo, hi


def q(vals, f):
    if not vals:
        return float("nan")
    w = sorted(vals)
    return w[min(len(w) - 1, int(f * len(w)))]


def read_us(path, pass_=0):
    """{brain name: [us per tick]} for one pass of a ticks CSV."""
    out = {n: [] for n in NAMES}
    with open(path) as f:
        for r in csv.DictReader(f):
            b = int(r["brain"])
            if int(r["pass"]) == pass_ and 0 <= b < 3:
                out[NAMES[b]].append(float(r["us"]))
    return out


def pearson(x, y):
    n = min(len(x), len(y))
    if n < 3:
        return float("nan")
    x, y = x[:n], y[:n]
    mx, my = statistics.fmean(x), statistics.fmean(y)
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    sxx = sum((a - mx) ** 2 for a in x); syy = sum((b - my) ** 2 for b in y)
    return sxy / math.sqrt(sxx * syy) if sxx > 0 and syy > 0 else float("nan")


def top_overlap(x, y, frac=0.01):
    """Share of the most expensive 1 % of ticks that are the same ticks in both runs."""
    n = min(len(x), len(y))
    k = max(1, int(frac * n))
    tx = set(sorted(range(n), key=lambda i: x[i])[-k:]); ty = set(sorted(range(n), key=lambda i: y[i])[-k:])
    return len(tx & ty) / k


def same_work(d):
    """Rows for the fidelity section: does the board's replay do the x86 replay's work at the same ticks?"""
    ra, rb, board = d / "ref_a.csv", d / "ref_b.csv", d / "ticks.csv"
    if not (ra.exists() and rb.exists() and board.exists()):
        return []
    A, B, X = read_us(ra), read_us(rb), read_us(board)
    rows = []
    for n in NAMES:
        if A[n] and X[n]:
            ratio = sorted(x / a for x, a in zip(X[n], A[n]) if a > 0)
            rows.append((n, len(X[n]), len(A[n]), pearson(A[n], B[n]), pearson(A[n], X[n]),
                         top_overlap(A[n], B[n]), top_overlap(A[n], X[n]), ratio[len(ratio) // 2] if ratio else float("nan")))
    return rows


def cmd_report(a):
    d = Path(a.results)
    facts = parse_facts(d)
    timing, counters = load_json(d / "timing.json"), load_json(d / "counters.json")
    if not timing:
        sys.exit(f"no timing.json in {d}")
    samples, ph = parse_samples(d), phases(d)
    tgt = target(getattr(a, "ghz", 1.8))
    L = []
    P = L.append
    P(f"# MicroDuck brain on ARM -- {facts.get('model', '?')}")
    P("")
    P(f"- results `{d.relative_to(REPO) if d.is_relative_to(REPO) else d}`, {facts.get('date', '?')}")
    P(f"- tape `{Path((d / 'tape.txt').read_text().strip()).name if (d / 'tape.txt').exists() else '?'}`"
      f" -- {(d / 'tape.preset').read_text().strip()[:60] if (d / 'tape.preset').exists() else '?'}")
    P(f"- build: {facts.get('compiler', '?')}; {facts.get('build_flags', '?')}")
    P(f"- kernel page size {facts.get('page_size', '?')} B; governor {facts.get('cpu%s_scaling_governor' % facts.get('core', '?'), '?')}"
      f" ({facts.get('cpu%s_scaling_min_freq' % facts.get('core', '?'), '?')}-{facts.get('cpu%s_scaling_max_freq' % facts.get('core', '?'), '?')} kHz);"
      f" pinned to core {facts.get('core', '?')}; perf_event_paranoid {facts.get('perf_event_paranoid', '?')}")
    P("")

    # ---- what ran beside it, and the board's state -------------------------------------------------------------------
    P("## Conditions (the confounds, recorded)")
    P("")
    P("| phase | s | max temp °C | bench-core MHz (min-max) | busy share of the OTHER cores | throttled |")
    P("|---|---|---|---|---|---|")
    core = f"cpu{facts.get('core', '2')}"
    for name in ("idle", "timing", "counters", "profile", "contention", "cooldown"):
        w = window(samples, ph, name)
        if not w:
            continue
        others = [v for s in w for c, v in s[4].items() if c != core]
        fr = [s[2] for s in w if not math.isnan(s[2])]
        thr = sorted({s[3] for s in w if s[3]})
        P(f"| {name} | {ph[name + '_end'] - ph[name + '_start']:.0f} | {max(s[1] for s in w):.1f} | "
          f"{min(fr):.0f}-{max(fr):.0f} | {100 * statistics.fmean(others):.1f} % | {','.join(thr) or 'n/a'} |"
          if fr and others else f"| {name} | | | | | |")
    P("")
    P(f"Throttle flags before/after: {facts.get('throttled_before', '?')} / {facts.get('throttled_after', '?')}. "
      "A busy share above a few % on the other cores during *timing* or *counters* means something else was running "
      "(it shares the L3 and DRAM); `facts.txt` names the busiest processes.")
    P("")

    # ---- measured ----------------------------------------------------------------------------------------------------
    P("## Measured on this board")
    P("")
    P("| brain | ticks | mean µs | p50 | p99 | p99.9 | max | % of a core at 50 Hz | ticks > 20 ms | reproduces the live run |")
    P("|---|---|---|---|---|---|---|---|---|---|")
    for name in NAMES:
        b = timing["brains"].get(name)
        if not b:
            continue
        c, fid = b["cpu_us"], b["fidelity"]
        ok = fid["compared"] and not fid["actions_differ"] and not fid["outputs_differ"]
        P(f"| {name} | {b['ticks']} | {c['mean']:.0f} | {c['p50']:.0f} | {c['p99']:.0f} | {c['p999']:.0f} | {c['max']:.0f} | "
          f"{100 * c['mean'] / TICK_US:.2f} | {b['over_20ms']} | "
          f"{'yes' if ok else 'NO -- outputs differ on %d ticks, first at %d' % (fid['outputs_differ'], fid['first_diff_tick'])} |")
    tot = sum(timing["brains"][n]["cpu_us"]["sum"] for n in NAMES if n in timing["brains"])
    nt = timing["brains"]["intent"]["ticks"] if "intent" in timing["brains"] else 1
    P("")
    P(f"All three brains together: **{tot / nt:.0f} µs per 50 Hz tick on average = {100 * tot / nt / TICK_US:.2f} % of one core**"
      f" (thread CPU time, {timing['repeat']} passes).")
    pr = timing["process"]
    heap = pr.get("heap_end_of_tape_kb", 0) + pr.get("heap_end_of_tape_mmapped_kb", 0)
    trace = pr.get("heap_kb_each_second", [])
    P("")
    P(f"Memory: the replay process peaks at **{pr['peak_rss_kb'] / 1024:.1f} MB resident** (page size {pr['page_size']} B; "
      f"a 16 kB-page kernel rounds every mapping up, so a 4 kB-page board resides less); the brains' own heap at the end of "
      f"the tape is **{heap / 1024:.1f} MB**" + (f", from {trace[0] / 1024:.1f} MB at 1 s (max {max(trace) / 1024:.1f} MB)." if trace else "."))
    P("")

    # ---- the same work? -------------------------------------------------------------------------------------------------
    rows = same_work(d)
    if rows:
        P("## Fidelity: the same work at the same ticks?")
        P("")
        P("An aarch64 build cannot reproduce the x86 tape bit for bit (Eigen's aarch64 matrix products sum in another order; "
          "README.md \"Fidelity\"), so \"reproduces the live run: NO\" above is expected. The working check: the board's "
          "per-tick cost against the x86 replay's (which does reproduce the run), with two x86 replays as the noise ceiling.")
        P("")
        P("| brain | ticks board / x86 | r, x86 vs x86 (ceiling) | r, board vs x86 | top-1 % ticks shared, x86/x86 | top-1 % shared, board/x86 | median per-tick time ratio board / x86 |")
        P("|---|---|---|---|---|---|---|")
        for n, nx, na, raa, rax, oaa, oax, med in rows:
            P(f"| {n} | {nx} / {na} | {raa:.3f} | {rax:.3f} | {100 * oaa:.0f} % | {100 * oax:.0f} % | ×{med:.2f} |")
        P("")
        P("Same tick count and a correlation near the ceiling: the board did the recorded run's work, tick for tick, "
          "and its timings are that run's cost. A correlation well below the ceiling means the brain took another path.")
        P("")

    # ---- where the time goes ------------------------------------------------------------------------------------------
    prof = (d / "profile.txt").read_text().splitlines() if (d / "profile.txt").exists() else []
    if prof:
        P("## Where the time goes (OGMA_PROFILE=1, one pass, thread CPU per module)")
        P("")
        P("```")
        for line in prof:
            body = line.replace("OGMA_PROFILE ", "", 1)
            m = re.search(r"share\s+([\d.]+) %", body)
            if not m or float(m.group(1)) >= 2.0:            # a module under 2 % of its brain is left out
                P(body)
        P("```")
        P("")

    # ---- contention ----------------------------------------------------------------------------------------------------
    cont = [load_json(p) for p in sorted(d.glob("contention_*.json"))]
    cont = [c for c in cont if c]
    if cont:
        P("## Under contention (one replay on every core at once)")
        P("")
        P("| brain | alone: mean / p99 µs | every core busy: mean / p99 µs (median of the copies) | slowdown of the mean |")
        P("|---|---|---|---|")
        for name in NAMES:
            if name not in timing["brains"]:
                continue
            al = timing["brains"][name]["cpu_us"]
            means = [c["brains"][name]["cpu_us"]["mean"] for c in cont if name in c["brains"]]
            p99s = [c["brains"][name]["cpu_us"]["p99"] for c in cont if name in c["brains"]]
            P(f"| {name} | {al['mean']:.0f} / {al['p99']:.0f} | {statistics.median(means):.0f} / {statistics.median(p99s):.0f} | "
              f"×{statistics.median(means) / al['mean']:.2f} |")
        P("")
        P("The shared L3 and DRAM under load. The RK3566's L3 is a quarter of the Pi 5's (512 kB against 2 MB), "
          "so contention there should be read as a lower bound for the Radxa.")
        P("")

    # ---- counters and the projection -----------------------------------------------------------------------------------
    rows_by = {n: [] for n in NAMES}
    if counters and "names" in counters.get("counters", {}) and (d / "ticks.csv").exists():
        with open(d / "ticks.csv") as f:
            for r in csv.DictReader(f):
                b = int(r["brain"])
                if 0 <= b < 3:
                    rows_by[NAMES[b]].append({k: float(v) for k, v in r.items() if k not in ("pass", "brain")})
    if any(rows_by.values()):
        mux = counters["counters"].get("multiplexed_reads", 0)
        P("## The work, per tick (hardware counters, user space, around tick() only)")
        P("")
        P("| brain | instructions mean / p99 / max | cycles mean / p99 | IPC | L1D MPKI | L2 MPKI | L3 MPKI | L2 refills/tick mean / p99 | backend-stalled |")
        P("|---|---|---|---|---|---|---|---|---|")
        for name in NAMES:
            R = rows_by[name]
            if not R:
                continue
            S = lambda k: sum(r.get(k, 0.0) for r in R)
            ins = S("instructions")
            col = lambda k: [r.get(k, 0.0) for r in R]
            P(f"| {name} | {statistics.fmean(col('instructions')):,.0f} / {q(col('instructions'), .99):,.0f} / {max(col('instructions')):,.0f} | "
              f"{statistics.fmean(col('cycles')):,.0f} / {q(col('cycles'), .99):,.0f} | {ins / max(S('cycles'), 1):.2f} | "
              f"{1000 * S('l1d_refill') / ins:.1f} | {1000 * S('l2d_refill') / ins:.2f} | {1000 * S('l3d_refill') / ins:.2f} | "
              f"{statistics.fmean(col('l2d_refill')):.0f} / {q(col('l2d_refill'), .99):.0f} | {100 * S('stall_backend') / max(S('cycles'), 1):.0f} % |")
        P("")
        if mux:
            P(f"**WARNING: the kernel multiplexed the counter group on {mux} reads -- the counts are scaled estimates.**")
            P("")
        # does the counters pass agree with the plain timing pass?  (the instrument must not be what was measured)
        ct = counters["brains"].get("intent", {}).get("cpu_us", {}).get("mean")
        tt = timing["brains"].get("intent", {}).get("cpu_us", {}).get("mean")
        if ct and tt:
            P(f"Instrument check: the intent brain's mean is {ct:.0f} µs with counters, {tt:.0f} µs without (×{ct / tt:.2f}).")
            P("")

        P(f"## Projection to the {tgt['name']}")
        P("")
        P("Two independent estimates; each is a bracket, low = the kinder board. Parameters and sources in README.md.")
        P("")
        P(f"- **A, whole-CPU ratio:** the time here × {tgt['gb6_ratio'][0]:.1f}-{tgt['gb6_ratio'][1]:.1f} "
          f"(Geekbench 6 single-core, Pi 5 at 2.4 GHz against RK3566, rescaled to {tgt['ghz']:g} GHz). Valid only if this board ran "
          "at its 2.4 GHz top clock -- see Conditions.")
        P(f"- **B, the work on an in-order core:** per tick, (instructions × CPI {tgt['a55_cpi_core'][0]}-{tgt['a55_cpi_core'][1]}"
          f" + L1D misses × {tgt['l1d_miss_scale'][0]}-{tgt['l1d_miss_scale'][1]} × {tgt['a55_l3_hit_cycles'][0]}-{tgt['a55_l3_hit_cycles'][1]}"
          f" cycles to the L3 + L2 misses × {tgt['dram_ns'][0]}-{tgt['dram_ns'][1]} ns to DRAM) / {tgt['ghz']} GHz, "
          "computed tick by tick so the worst tick keeps its own instructions and misses.")
        P("")
        P("| brain | A: mean µs | A: p99 µs | B: mean µs | B: p99 µs | B: max µs | B: ticks > 20 ms | % of an A55 at 50 Hz (A / B) |")
        P("|---|---|---|---|---|---|---|---|")
        proj = {}
        tot_a, tot_b = [0.0, 0.0], [0.0, 0.0]
        for name in NAMES:
            R = rows_by[name]
            if not R or name not in timing["brains"]:
                continue
            c = timing["brains"][name]["cpu_us"]
            ra = tgt["gb6_ratio"]
            lo, hi = project_ticks(R, tgt)
            over = (sum(v > TICK_US for v in lo), sum(v > TICK_US for v in hi))
            bm = (statistics.fmean(lo), statistics.fmean(hi))
            proj[name] = dict(A_mean=[c["mean"] * ra[0], c["mean"] * ra[1]], A_p99=[c["p99"] * ra[0], c["p99"] * ra[1]],
                              B_mean=list(bm), B_p99=[q(lo, .99), q(hi, .99)], B_max=[max(lo), max(hi)], B_over_20ms=list(over),
                              ticks=len(R))
            for k in (0, 1):
                tot_a[k] += c["sum"] * ra[k]                     # every timing pass
                tot_b[k] += sum(lo if k == 0 else hi)
            P(f"| {name} | {c['mean'] * ra[0]:.0f}-{c['mean'] * ra[1]:.0f} | {c['p99'] * ra[0]:.0f}-{c['p99'] * ra[1]:.0f} | "
              f"{bm[0]:.0f}-{bm[1]:.0f} | {q(lo, .99):.0f}-{q(hi, .99):.0f} | {max(lo):.0f}-{max(hi):.0f} | {over[0]}-{over[1]} | "
              f"{100 * c['mean'] * ra[0] / TICK_US:.1f}-{100 * c['mean'] * ra[1] / TICK_US:.1f} / {100 * bm[0] / TICK_US:.1f}-{100 * bm[1] / TICK_US:.1f} |")
        ri = rows_by["intent"]
        if ri:
            # the floor no cache model touches: the instructions alone, at IPC 1.0 (the A55 at its best on this code)
            floor = sorted(r["instructions"] / (tgt["ghz"] * 1e3) for r in ri)       # us
            P("")
            P(f"Floor, the intent brain's instructions alone at IPC 1.0 (no cache misses at all): mean {statistics.fmean(floor):.0f} µs, "
              f"p99 {q(floor, .99):.0f} µs, max {floor[-1]:.0f} µs; {sum(v > TICK_US for v in floor)} ticks over 20 ms.")
        n_a = timing["brains"].get("intent", {}).get("ticks", 0) or 1     # 50 Hz ticks, every timing pass
        n_b = len(rows_by["intent"]) or 1                                # 50 Hz ticks, the counters pass
        share = {"A": [t / n_a / TICK_US for t in tot_a], "B": [t / n_b / TICK_US for t in tot_b]}
        P("")
        P(f"All three brains together on one A55: **A {100 * share['A'][0]:.1f}-{100 * share['A'][1]:.1f} %, "
          f"B {100 * share['B'][0]:.1f}-{100 * share['B'][1]:.1f} % of one core** at 50 Hz, of four.")
        (d / "projection.json").write_text(json.dumps({"target": tgt, "brains": proj, "all_brains_core_share": share}, indent=1))
    elif counters:
        P("## Counters")
        P("")
        why = counters.get("counters", {}).get("unavailable") or "no ticks.csv beside counters.json"
        P(f"Unavailable: {why} -- no projection; rerun (with --sudo-counters if perf_event_paranoid > 2).")
    text = "\n".join(L) + "\n"
    (d / "report.md").write_text(text)
    print(text)
    print(f"written: {d / 'report.md'}")


# ---------------------------------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p, host=True):
        if host:
            p.add_argument("host")
        p.add_argument("--remote", default=REMOTE, help=f"bench directory on the board (default {REMOTE}, tmpfs on the Pi)")
        p.add_argument("--tape", default=str(DEFAULT_TAPE))

    p = sub.add_parser("record"); common(p, host=False)
    p.add_argument("--preset", default="★ F5", help="launcher preset name prefix (default ★ F5)")
    p.add_argument("--seed", type=int); p.add_argument("--secs", type=float)
    p = sub.add_parser("reference"); common(p, host=False)
    p = sub.add_parser("stage"); common(p)
    for name in ("build", "all"):
        p = sub.add_parser(name); common(p)
        p.add_argument("--cpu", default="cortex-a55", help="-mcpu= ('' for the compiler's default)")
        p.add_argument("-j", "--jobs", type=int, default=2, help="parallel compiles on the board (2 GB Pi 5: 2)")
    for name in ("run", "all"):
        p = sub.choices[name] if name in sub.choices else sub.add_parser(name)
        if name == "run":
            common(p)
        p.add_argument("--core", type=int, default=2)
        p.add_argument("--repeat", type=int, default=3)
        p.add_argument("--vmem-kb", type=int, default=1048576, help="ulimit -v for every replay (OOM guard)")
        p.add_argument("--no-contention", action="store_true")
        p.add_argument("--sudo-counters", action="store_true", help="run the counters pass with sudo -n")
        p.add_argument("--label", default="")
    p = sub.add_parser("report"); p.add_argument("results")
    p.add_argument("--ghz", type=float, default=1.8, help="the target A55's clock (1.8: Pollen's figure for the duck's board)")
    a = ap.parse_args()
    if a.cmd == "all":
        if not Path(a.tape).exists():
            a.preset, a.seed, a.secs = "★ F5", None, None
            cmd_record(a)
        if not all(p.exists() for p in ref_paths(a.tape)):
            cmd_reference(a)
        cmd_stage(a); cmd_build(a); cmd_run(a)
    else:
        {"record": cmd_record, "reference": cmd_reference, "stage": cmd_stage, "build": cmd_build, "run": cmd_run, "report": cmd_report}[a.cmd](a)


if __name__ == "__main__":
    main()
