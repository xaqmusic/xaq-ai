#!/usr/bin/env python3
"""Human-readable HTML report from seedavg body logs — the shared-analysis tool.

Built 2026-08-03 because the operator and the assistant were looking at different
things: the assistant read per-tick JSONL and window aggregates, the operator watched
the UI, and several disagreements (activity that peaks then decays, a "frozen" arm that
was actually inchworming, a coherence metric reading a structural zero) came from having
no common view of the same data.

WHY TIME SERIES AND NOT JUST MEANS.  Two of this campaign's biggest errors were invisible
in whole-run aggregates:
  * an arm whose behaviour FORMS at ~10k ticks and DECAYS after 20k reads as a flat mean;
  * seed spread that exceeds the between-arm difference reads as a confident number.
So every metric is drawn per-seed (thin) with the seed-mean over it (thick).  If the thin
lines fan out, the mean is not a result — that is the point of the chart.

OUTPUT LOCATION: write these to docs/reports/run_summaries/ (NOT /tmp) using the
convention YYYY-MM-DD_<slug>.html, and cite the file from the ledger entry it supports.
The scratch logs are session-scoped and vanish; the chart is what survives to be re-read
when a later result contradicts an earlier one.  See that directory's README for the
naming and pruning convention.

Usage:
  gaitreport.py <out.html> <label>=<log_glob> [<label>=<log_glob> ...]

  gaitreport.py docs/reports/run_summaries/2026-08-03_pure-hk_vs_deployed.html \
      "deployed=/tmp/xaq_seedavg/sa_the_picrawler*steplock_off_s*.log" \
      "pure-HK lr0.01=/tmp/.../sa_*stance__c025_s*.log" \
      "pure-HK lr0.10=/tmp/.../sa_*stance__c025__lr10_s*.log"

Self-contained output: no CDN, no external fonts, renders offline and can be emailed.
"""
import glob, html, json, math, os, statistics, sys

# --- palette (dataviz reference instance; validated light+dark, adjacent pairlist) -----
LIGHT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
DARK  = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300"]

# metric key -> (display label, one-line "what it means", higher_is_better|None)
METRICS = [
    ("_disp",     "Distance travelled (m)",      "Straight-line displacement from spawn. The headline, and the noisiest.", True),
    ("_steprate", "Step rate (lifts / 1k ticks)","How busy the legs are. Rises when damping falls; not the same as progress.", True),
    ("coh",       "Inter-leg coherence (0-1)",   "Kuramoto phase-lock BETWEEN legs. 'Are they working together?' Deployed gait sits ~0.42.", True),
    ("hk_agree",  "hip2 + knee agreement (0-1)", "Do the two lift joints push the SAME way? 0.5 = chance. Emergent intra-leg IK.", True),
    ("y",         "Chassis height (m)",          "How tall it stands. Deployed ~0.058; pure-HK sinks unless something holds it up.", True),
    ("tq_mag",    "Ground force (norm.)",        "Mean |servo torque| vs the 0.15 Nm limit. How hard the legs push.", None),
    ("tq_sat",    "Torque saturation (frac)",    "Fraction of servo-ticks pinned at the limit. High = out of authority.", None),
    ("clip_duty", "Command clipping (frac)",     "Fraction of leg-ticks the requested command exceeded +-1 and was flattened.", False),
    ("motor_tle", "Self-model error",            "Forward-model surprise. 0.0000 exactly usually means a frozen body, not a good model.", None),
    ("tilt",      "Body tilt (rad)",             "Instantaneous lean. Its spread over time is the wobble.", False),
]

BIN = 500          # ticks per plotted point; keeps files small and lines readable
WARMUP = 900       # skip the spawn transient


def load(path):
    """-> {tick: {metric: value}} for one seed, binned."""
    rows = []
    for line in open(path, errors="replace"):
        line = line.strip()
        if not line.startswith("{") or '"x":' not in line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        if "t" not in d or d["t"] < WARMUP:
            continue
        rows.append(d)
    if len(rows) < 3:
        return None
    out, prev = [], None
    for r in rows:
        b = (r["t"] // BIN) * BIN
        if prev is None or b != prev["b"]:
            if prev is not None:
                out.append(prev)
            prev = {"b": b, "rows": [], "first": r, "last": r}
        prev["rows"].append(r)
        prev["last"] = r
    if prev is not None:
        out.append(prev)

    x0, z0 = rows[0].get("x", 0.0), rows[0].get("z", 0.0)
    series = []
    for w in out:
        rs, f, l = w["rows"], w["first"], w["last"]
        pt = {"t": w["b"]}
        pt["_disp"] = math.hypot(l.get("x", 0) - x0, l.get("z", 0) - z0)
        dt = max(1, l.get("t", 0) - f.get("t", 0))
        lf = f.get("leg_lifted_counts") or [0, 0, 0, 0]
        ll = l.get("leg_lifted_counts") or [0, 0, 0, 0]
        pt["_steprate"] = sum(b - a for a, b in zip(lf, ll)) / dt * 1000.0
        for k, *_ in METRICS:
            if k.startswith("_"):
                continue
            vals = [r[k] for r in rs if k in r]
            if vals:
                pt[k] = statistics.mean(vals)
        series.append(pt)
    return series


def collect(spec):
    label, _, pattern = spec.partition("=")
    paths = sorted(glob.glob(pattern))
    seeds = [s for s in (load(p) for p in paths) if s]
    return {"label": label or os.path.basename(pattern), "seeds": seeds,
            "n": len(seeds), "files": len(paths)}


def svg_chart(arms, key, title, blurb, idx):
    W, H = 620, 190
    ML, MR, MT, MB = 52, 14, 10, 26
    pts = [(p["t"], p[key]) for a in arms for s in a["seeds"] for p in s if key in p]
    if not pts:
        return ""
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    if y1 - y0 < 1e-9:
        y0, y1 = y0 - 0.5, y1 + 0.5
    pad = (y1 - y0) * 0.08
    y0 -= pad; y1 += pad
    sx = lambda v: ML + (v - x0) / max(1e-9, x1 - x0) * (W - ML - MR)
    sy = lambda v: MT + (1 - (v - y0) / max(1e-9, y1 - y0)) * (H - MT - MB)

    o = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{html.escape(title)}">']
    for f in range(5):                                     # recessive grid + y labels
        yy = y0 + (y1 - y0) * f / 4
        o.append(f'<line class="grid" x1="{ML}" x2="{W-MR}" y1="{sy(yy):.1f}" y2="{sy(yy):.1f}"/>')
        o.append(f'<text class="ax" x="{ML-6}" y="{sy(yy)+3.5:.1f}" text-anchor="end">{yy:.3g}</text>')
    for f in range(5):                                     # x labels (ticks -> k)
        xx = x0 + (x1 - x0) * f / 4
        o.append(f'<text class="ax" x="{sx(xx):.1f}" y="{H-8}" text-anchor="middle">{xx/1000:.0f}k</text>')

    for ai, a in enumerate(arms):
        col = f"var(--s{ai+1})"
        for s in a["seeds"]:                               # per-seed spaghetti = the variance
            d = " ".join(f"{'M' if i==0 else 'L'}{sx(p['t']):.1f},{sy(p[key]):.1f}"
                         for i, p in enumerate([q for q in s if key in q]))
            if d:
                o.append(f'<path class="seed" d="{d}" stroke="{col}"/>')
        buckets = {}
        for s in a["seeds"]:
            for p in s:
                if key in p:
                    buckets.setdefault(p["t"], []).append(p[key])
        mean = sorted((t, statistics.mean(v)) for t, v in buckets.items())
        d = " ".join(f"{'M' if i==0 else 'L'}{sx(t):.1f},{sy(v):.1f}"
                     for i, (t, v) in enumerate(mean))
        if d:
            o.append(f'<path class="mean" d="{d}" stroke="{col}"/>')
            lt, lv = mean[-1]
            o.append(f'<circle cx="{sx(lt):.1f}" cy="{sy(lv):.1f}" r="4.5" fill="{col}" '
                     f'stroke="var(--surface-1)" stroke-width="2"/>')
    o.append("</svg>")
    return (f'<figure class="chart"><figcaption><h3>{html.escape(title)}</h3>'
            f'<p>{html.escape(blurb)}</p></figcaption>{"".join(o)}</figure>')


def main(argv):
    if len(argv) < 2:
        print(__doc__); return 2
    out_path, specs = argv[0], argv[1:]
    arms = [collect(s) for s in specs]
    arms = [a for a in arms if a["n"]]
    if not arms:
        print("no parsable logs matched"); return 1

    legend = "".join(
        f'<span class="key"><i style="background:var(--s{i+1})"></i>'
        f'{html.escape(a["label"])} <em>n={a["n"]}</em></span>'
        for i, a in enumerate(arms))

    charts = "".join(svg_chart(arms, k, t, b, i)
                     for i, (k, t, b, _) in enumerate(METRICS))

    # table view — the accessibility fallback AND the numbers to quote
    head = "".join(f"<th>{html.escape(t)}</th>" for _, t, _, _ in METRICS)
    body = ""
    for a in arms:
        cells = ""
        for k, *_ in METRICS:
            fin = [ [p[k] for p in s if k in p][-1] for s in a["seeds"]
                    if any(k in p for p in s) ]
            if fin:
                m = statistics.mean(fin)
                sd = statistics.stdev(fin) if len(fin) > 1 else 0.0
                cells += f"<td>{m:.4g} <span class='sd'>± {sd:.2g}</span></td>"
            else:
                cells += "<td class='sd'>—</td>"
        body += f"<tr><th scope='row'>{html.escape(a['label'])}</th>{cells}</tr>"

    css_light = "".join(f"--s{i+1}:{c};" for i, c in enumerate(LIGHT))
    css_dark = "".join(f"--s{i+1}:{c};" for i, c in enumerate(DARK))
    doc = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Picrawler gait telemetry</title><style>
:root{{--surface-1:#fcfcfb;--surface-2:#f4f4f1;--text-primary:#0b0b0b;--text-secondary:#52514e;--muted:#8a8983;--rule:#e2e1dc;{css_light}}}
@media (prefers-color-scheme:dark){{:root:where(:not([data-theme=light])){{--surface-1:#1a1a19;--surface-2:#232322;--text-primary:#fff;--text-secondary:#c3c2b7;--muted:#8e8d85;--rule:#343432;{css_dark}}}}}
:root[data-theme=dark]{{--surface-1:#1a1a19;--surface-2:#232322;--text-primary:#fff;--text-secondary:#c3c2b7;--muted:#8e8d85;--rule:#343432;{css_dark}}}
*{{box-sizing:border-box}}
body{{margin:0;padding:28px 22px 60px;background:var(--surface-1);color:var(--text-primary);
font:15px/1.55 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}}
.wrap{{max-width:1320px;margin:0 auto}}
h1{{font-size:23px;margin:0 0 4px}}
.sub{{color:var(--text-secondary);margin:0 0 18px;max-width:74ch}}
.legend{{display:flex;flex-wrap:wrap;gap:16px;padding:10px 12px;background:var(--surface-2);
border:1px solid var(--rule);border-radius:8px;margin-bottom:20px}}
.key{{display:flex;align-items:center;gap:7px;font-size:13px}}
.key i{{width:22px;height:3px;border-radius:2px;display:inline-block}}
.key em{{color:var(--muted);font-style:normal}}
.grid-charts{{display:grid;grid-template-columns:repeat(auto-fit,minmax(430px,1fr));gap:20px}}
.chart{{margin:0;background:var(--surface-2);border:1px solid var(--rule);border-radius:8px;padding:12px 12px 4px;min-width:0}}
.chart h3{{font-size:14px;margin:0 0 2px}}
.chart p{{font-size:12.5px;color:var(--text-secondary);margin:0 0 6px}}
.chart svg{{width:100%;height:auto;display:block;overflow:visible}}
.grid{{stroke:var(--rule);stroke-width:1}}
.ax{{fill:var(--muted);font-size:10px}}
.seed{{fill:none;stroke-width:1;opacity:.28}}
.mean{{fill:none;stroke-width:2.5;stroke-linejoin:round;stroke-linecap:round}}
.tablewrap{{overflow-x:auto;margin-top:26px;border:1px solid var(--rule);border-radius:8px}}
table{{border-collapse:collapse;font-size:12.5px;width:100%;min-width:900px}}
th,td{{padding:7px 10px;text-align:right;border-bottom:1px solid var(--rule);white-space:nowrap}}
thead th{{text-align:right;color:var(--text-secondary);font-weight:600;background:var(--surface-2)}}
tbody th{{text-align:left;font-weight:600}}
.sd{{color:var(--muted)}}
.note{{margin-top:22px;padding:12px 14px;background:var(--surface-2);border:1px solid var(--rule);
border-radius:8px;color:var(--text-secondary);font-size:13px;max-width:88ch}}
</style></head><body><div class="wrap">
<h1>Picrawler gait telemetry</h1>
<p class="sub">Each metric over the run. <strong>Thin lines are individual seeds; the thick line is
their mean.</strong> When the thin lines fan apart, the mean is not a result — read the spread
first. Final-value means ± sd are in the table below.</p>
<div class="legend">{legend}</div>
<div class="grid-charts">{charts}</div>
<div class="tablewrap"><table><caption class="sr-only">Final values per arm</caption>
<thead><tr><th scope="col">Arm</th>{head}</tr></thead><tbody>{body}</tbody></table></div>
<p class="note"><strong>Reading notes.</strong> Values are binned every {BIN} ticks and the first
{WARMUP} are dropped (spawn transient). <em>Distance</em> is straight-line from spawn, so a robot
walking in circles shows a flat line while its step rate stays high. An <em>exactly round</em>
value (0.0000, 0.5000) is usually structural rather than measured — a frozen body or a metric
whose input was gated off — not a clean result.</p>
</div></body></html>"""
    open(out_path, "w").write(doc)
    print(f"wrote {out_path}")
    for a in arms:
        print(f"  {a['label']:<28} n={a['n']} (from {a['files']} logs)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
