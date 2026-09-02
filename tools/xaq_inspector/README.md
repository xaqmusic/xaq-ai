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
  - Topic `"lite"` (2026-08-28): the publisher serves `Module::diag_lite()` instead — a
    handful of scalars (for an EPM: `last_tle`, `ema_tle`, `last_quant_error`,
    `novelty_threshold_now`, `nodes`, `baked`, `mitosis_count`, `baked_now`; for
    `MotorEPMv2`: `motor_tle`), ~280 bytes against ~50 KB for the full snapshot.
    High-rate subscribers that want the error signal, not the state, must use it —
    `tools/xaq_voice` does. The inspector keeps the full snapshot (topic `""`).
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

**The host fields under the module list are the normal way to connect** — type
`picrawler.local` (or `host:port`) and press Enter, or hit **Connect / Refresh**.
Both endpoints are **remembered between runs**, so the usual launch takes no
arguments at all:

```sh
tools/run_inspector.sh          # reconnects to whatever you used last
```

Control and diag are separate fields because they can legitimately differ — an
ssh tunnel forwarding only one, say — and each accepts a bare host or `host:port`
so the ports stay overridable without four widgets. A field never raises: an
unparseable port falls back to the default, and an unreachable host becomes a
status-bar line rather than a dialog in the middle of debugging.

`--control-host` / `--control-port` / `--diag-host` / `--diag-port` still work and
win for that run, then become the remembered value. Defaults are
`127.0.0.1:7400` (control) and `127.0.0.1:7401` (diag).

Retargeting keeps the live subscription: the SUB socket is disconnected and
reconnected rather than rebuilt, and ZMQ subscriptions belong to the socket, not
the connection.

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
