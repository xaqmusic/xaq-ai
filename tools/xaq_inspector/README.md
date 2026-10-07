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

To reuse the transport or widgets from another project, install it as a package:

```bash
pip install -e tools/xaq_inspector   # provides the `xaq_inspector` package and an `xaq-inspector` command
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

## The duck's three brains (2026-10-02)

The MuJoCo host runs up to three brains, and each serves its own pair of ports from the base
(`OGMA_INSPECTOR_PORT`, default 7400; diag = control + 1):

| brain | control · diag | what is in it |
|---|---|---|
| intent / walker | 7400 · 7401 | the twist bridge, the walker's MotorEPMv2, the map, play, the cloud, the thing EPMs, seek, the competences, the voter, the arbiter, the outcome loop |
| head | 7402 · 7403 | the head bridge and the head's MotorEPMv2 (the gaze) |
| stand | 7404 · 7405 | the stop's stand brain (it ticks only during a stop) |

The **brain** selector under the host fields re-points both fields and reconnects; the module list's header names
the brain that answered. The host's stderr prints one `inspector (… brain): control … diag …` line per brain. Before
2026-10-02 every brain asked for 7400 and the first constructed won.

## Static voxel viewer

A standalone tool, no brain needed: the duck's **sweep clouds** in an interactive 3D view. It reads
the `cloudv` records `ogma::CloudMap` files into a host run's JSONL — a run whose graph declares a
`CloudMap` and whose host ran with `--cloud` — or a saved CloudMap snapshot (a JSON object with `vox`).

```sh
tools/run_voxel_viewer.sh RUN.jsonl                      # every filed cloud, laid out in the room
tools/run_voxel_viewer.sh RUN.jsonl --place 8            # one place alone
tools/run_voxel_viewer.sh RUN.jsonl --frame body --colour hits
tools/run_voxel_viewer.sh RUN.jsonl --screenshot out.png # render once, print the readout, exit
```

- **Orbit** with the left mouse button, **pan** with the middle, **zoom** with the wheel. The list
  shows every filed cloud by place and time, with its revisit judgement where one was made.
- **World** lays clouds out at the world pose they were anchored on — instrumentation, the only way
  to see a cloud beside the furniture it describes; no brain reads it. **Body** shows one cloud in its
  own frame, which is what the duck actually has. A short line marks where the duck stood and which
  way it faced (in the body frame, the origin facing +x). With every cloud shown, the selected one is
  drawn at full size and colour and the rest recede as smaller, darker cubes: clouds of one wall
  share voxel cells, and cubes of equal size would flicker into each other. Picking another cloud in
  that view keeps the camera where you left it.
- **Height bands** colour each voxel by the mean height of the points in it: grey floor (below 2 cm,
  hidden by default), orange for something standing on the floor (2–20 cm), blue at furniture height
  (20–45 cm), pale above. **Hits** colours by how many returns landed in a voxel, on a log scale.
  `Min hits` drops thinly-supported voxels.
- Logs written before 2026-09-13 carry no mean height; the viewer falls back to the voxel centre and
  says so, and on those logs the floor reads as break (design doc §17.30).
- `--screenshot` needs a display: this GL stack draws nothing on Qt's offscreen platform.
- Needs **PyOpenGL** in the venv, which `requirements.txt` now lists.

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
| `MotorEPMv2`    | `MotorEpmV2Inspector` — **self-model & priors** tab (the identified A as a heatmap, the state against each prior's target and precision, the output, motor TLE and the prior's error) + the gait dashboard tab (a legged body opens on it). |
| `CloudMap`      | `CloudMapInspector` — plan view of the duck's ToF cloud (voxels by height band; clusters: small, *top unseen*, structure; the attended thing; the mover; toggle the free-space "top seen" layer) + counts over time. |
| `BearingSeekLoop` | `BearingSeekInspector` — body-frame plan of the held target (by source), the chase candidate and its velocity, the remembered mover; every way a target starts and ends. |
| `SkillOutcomeLoop` | `SkillOutcomeInspector` — the outcome table, kind × intent (n · mean ± spread; known / tried / never asked), need and surprise. |
| `LoopCompetence` | `LoopCompetenceInspector` — the Beta belief over a loop's success rate, competence, what is published, the gain it earns. |
| `JointSensorimotorBridge` | `JointBridgeInspector` — per motor position / command / change, and the sense slots now and as a rolling heat strip: exactly what a motor brain gets to feel. |
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
