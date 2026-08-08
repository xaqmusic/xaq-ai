# xaq inspector — sidecar UI for the live Ogma brain

Sidecar PyQt6 application that attaches to a running Godot host's
`OgmaBrain` and renders deep per-module state.  Decoupled from Godot
(runs whether the editor / project is up or not) — connects via the
brain's two inspector surfaces:

- **Control socket**: TCP 7400, newline-delimited JSON.  Verbs:
  - `list_modules`
  - `module_snapshot {id}`
  - `module_subscribe_diag {id, topic, hz}` → returns
    `{sub_id, diag_port, topic_prefix}`
  - `unsubscribe {sub_id}`
- **Diag stream**: ZMQ PUB on TCP 7401.  Per-subscription topic prefix
  `diag.<sub_id>.` carries serialized `Module::snapshot_state()` as
  JSON, fanned out at the requested Hz from the host's tick thread.

## Run

```bash
pip install -r requirements.txt
tools/run_inspector.sh
```

Run `tools/run_inspector.sh` from the repo root (or anywhere — it locates
itself). **Don't run `python -m xaq_inspector` directly unless your CWD is
`tools/`** (the parent of this package) — `xaq_inspector` is a plain
directory package, not pip-installed, so `python -m xaq_inspector` only
resolves when `tools/` is on `sys.path`. The wrapper script sets
`PYTHONPATH` for you; running it from inside `tools/xaq_inspector/` itself
(a natural first read of this doc) fails with `No module named
xaq_inspector`.

Defaults to `tcp://127.0.0.1:7400` for control and `tcp://127.0.0.1:7401`
for diag.  Use `--control-port` / `--diag-host` to override (passed through
to `run_inspector.sh`).

## Module dispatch

Each module type registers a widget class in
`xaq_inspector/widgets/__init__.py`:

| type            | widget                              |
|-----------------|-------------------------------------|
| `EPM`           | `EpmInspector` — GNG canvas, encoder strip, TLE plot, lifecycle. |
| `NeurochemState` | `NeuroInspector` — DA / 5-HT scrolling time-series. |
| `HomeostaticDrive` | `DriveInspector` — per-channel urgency + setpoint streams. |
| `FaderController` | `FaderInspector` — fader alpha components. |
| `LateralVoter`  | `VoterInspector` — trust shares, consensus dynamics. |
| `Premotor`      | `PremotorInspector` — intent distribution, policy outputs, W heatmap. |
| `SequenceGNG`   | `SeqGNGInspector` — cluster-growth + match scalars + winner-window + per-node visits + transition matrix.  Use to gut-check whether SeqGNG is finding meaningful clusters or noise crystals. |
| reflexes / detectors | `ReflexInspector` — auto-fields. |
| (anything else) | `RawPayloadView` — pretty-printed JSON of the live snapshot. |

New widgets land by adding a row to the dispatch table.

## Reuse from v3

`tools/xaq_inspector/widgets/epm_canvas.py` is a focused port of
`src/native/widgets/graph_canvas.py` (v3 PyQtGraph GNG canvas).  The
v3 widget assumed a pre-projected (`x`,`y`) per node; here we project
each node's `prototype` vector to 2-D via its first two components
(adequate for the "is it changing?" feedback the inspector needs;
PCA can be re-added later).
