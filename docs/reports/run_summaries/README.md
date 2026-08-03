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

## Convention

- **Generate with** [`../../../godot_host/project/scripts_tools/gaitreport.py`](../../../godot_host/project/scripts_tools/gaitreport.py):
  ```sh
  python3 gaitreport.py docs/reports/run_summaries/<date>_<what>.html \
      "label=/path/sa_*_s*.log" "label2=..."
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
