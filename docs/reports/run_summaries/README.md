# Run summaries — data visualizations that live in the repo

**Rendered, human-readable views of measurement runs.** These exist so the operator and the
assistant look at the *same* data, and so a finding can be forwarded to someone outside the
project (or re-read months later) without re-running anything.

## Why these are in the repo and not `/tmp`

Every seed log written by `seedavg.py` lands in a scratch directory that is session-scoped and
disappears. That was fine while the numbers were the deliverable. It stopped being fine once the
campaign started producing results whose *shape over time* was the finding:

- behaviour that **forms at ~10 k ticks and decays after 20 k** reads as a flat mean;
- **seed spread that exceeds the between-arm difference** reads as a confident number;
- a metric reading **exactly 0.000** because its input was gated off reads as a measurement.

All three of those were missed at least once from aggregates alone. A chart makes them obvious,
and a chart that no longer exists cannot be re-read when a later result contradicts an earlier
one — which has happened repeatedly.

## Required format

**Every run summary carries the same three things, in this order.** A chart without them is
uninterpretable a month later, and unusable by anyone outside the project.

1. **What is under test** — 2–4 sentences: the question, what each arm is, and what they differ
   by. **This is the only part that cannot be scraped**, so it is a required `--concept`
   argument; the generator warns if it is missing.
2. **Provenance — what was actually run.** Auto-scraped from each run's own stdout, so it
   describes the run that happened rather than the one intended:
   - **config** — the `res://` path the brain actually loaded, its `metadata.name`, and its module list
   - **differs by** — the headline params that differ across arms, rendered *on* (`key=value`)
     or *off* (struck through), plus a count of any further differences **explicitly stated,
     never silently truncated** (a hidden difference is a silent confound)
   - **overlays** — any `⚠` body-side env override (gravity scale, damping scale, sensor
     noise). These live in no config file, so a summary that omits them can describe the wrong
     body entirely
   - **body & environment** — gym + difficulty, joint backend, build line, reset mode
   - **ticks · n seeds · seed numbers**
3. **The charts**, per-seed thin lines under the seed-mean, then the final-value table.

### Generate

```sh
python3 godot_host/project/scripts_tools/gaitreport.py \
    docs/reports/run_summaries/YYYY-MM-DD_<slug>.html \
    "label=/path/sa_*_s*.log" "label2=..." \
    --title   "One line naming the comparison" \
    --concept "2-4 sentences: the question, the arms, what they differ by."
```

- **Name** `YYYY-MM-DD_<short-slug>.html` — sortable, and the date ties it to the ledger entry.
- **Self-contained**: no CDN, no external fonts, no network. Opens offline, survives being
  emailed, and renders in light or dark.
- **Cite it** from the ledger entry it supports, so the numbers and the picture stay joined.

## Pruning

These are ~75–100 KB each and they are *artifacts of a moment*, not source. Prune freely:

- **Keep** a summary that a ledger entry cites, or that shows a result later work contradicts —
  those are the ones worth being able to re-read.
- **Drop** duplicates, superseded arms, and anything from a run whose configuration was later
  found to be defective (unless the defect itself is the finding).
- When in doubt, keep the one with the **widest seed spread** — it is the honest record of how
  noisy the regime was, and that is the thing this project has most often forgotten.

## Index

| file | what it shows |
|---|---|
| `2026-08-03_pure-hk_vs_deployed_coordination.html` | Deployed gait vs pure-HK at `ctrl_lr` 0.01 and 0.10, 10 metrics. The run where inter-leg coherence became measurable for the first time: pure-HK at 0.01 (**0.484**) out-coordinates the deployed gait (**0.419**), and raising `ctrl_lr` trades coordination for power. Supports the 2026-08-03 ledger entries. |
