# duck_launcher — the Microduck experiment launcher

The mj_host counterpart of the Godot launcher. One window: pick a preset or a
config, a seed (or a battery of seeds), a duration, where to start from, a shove
schedule, and whether to watch live or run headless. The exact host command is
always on screen and can be pasted into a shell to repeat the run verbatim —
every run is deterministic, so a live window of the same command *is* the run.

```sh
./mj_host/run.sh launcher                          # the window
tools/duck_launcher/launcher.py --selftest         # every preset's command, no window
tools/duck_launcher/launcher.py --print            # the command for the saved selections
```

It runs on the viewer's venv (`tools/duck_viewer/setup.sh`) and needs nothing
beyond it: tkinter is in the standard library, and the window never simulates —
it builds argv for `mj_host/build/ogma_mjhost` and, for watching, hands it to
`tools/duck_viewer/view.py live`.

## What is on the window

| section | controls | host flags |
|---|---|---|
| Experiment | preset (fills everything below), config, "show all", the config's description | `--graph` |
| Run | mode (brain / scaffold hold / stub), seed + random, battery of seeds (`1-6`, `1,2,5`), duration, scene, watch or headless | `--brain/--hold/--stub --seed --secs`, scene path |
| Start from | from scratch with identification episodes, a saved brain (repo checkpoints + every `.brain.json` under `mj_host/log`, or "latest"), nothing; save the brain at the end | `--ident-every --ident-until --load-brain --save-brain` |
| Perturbation | force, period, hold, start | `--push --push-every --push-hold --push-from` |
| Extras | amp, freeze-after, no-tilt-gate, servo-filter; hold noise; stub amp/drift | `--amp --freeze-after --no-tilt-gate --servo-filter --noise --stub-amp --stub-drift` |
| Command | the run, as a shell line | |
| Runs | Launch · Stop · Replay (fast) · Replay (real time) · Report (`push_report.py`) · Copy command · Open log dir · Health gates · Inspector (attaches `tools/xaq_inspector` to the live run's brain); each run's status and elapsed (frozen at exit; a Stop reads "stopped") and the selected run's summary | |

Selections persist in `mj_host/log/launcher_state.json`. Runs go to
`mj_host/log/launcher/<stamp>_<config>_s<seed>[_pushN].{jsonl,err,brain.json}` —
on the main disk on purpose: a run writes about 190 MB of JSONL per sim-hour,
and `/tmp` is a quota'd tmpfs that takes the shell down when it fills. The
window shows the estimate next to the duration.

**Every experiment run headless has a preset**, and a preset mirrors the harness's argv exactly
(mode, scene, seed, duration, reset noise, arena shift, every host flag). Picking a **config** by name
loads that config's own preset when one names it, so the window shows the same setup the headless run
used; a config no preset names is only the graph, with the controls left as they were (2026-09-27: the
train room lived in the R84 preset's scene and host flags, and the config pick had shown the plain room).
The scene list is every `scene_*.xml` in `mj_host/models/microduck`.

## Naming: milestones, probes, and the series

The two lists are read top to bottom, and each entry says what it is by its prefix:

| class | prefix | rank band | meaning |
|---|---|---|---|
| milestone | `★ STACK`, `★ HEAD`, `★ CLOUD`, `★ THINGS` | < 100 | the promoted state: the stand, the head brain, the glance, the things cycle |
| probe | `PROBE · …` | 100–999 | an instrument, not a hypothesis: the push test, the envelope, the byte-identity guard, the scaffold, the stub |
| series | `R<nn> · …` | 1000 + nn | the tests of the current campaign, in the order they were made; the newest carries **◀ latest** |

The R number is the design doc's: the config, its preset, and the doc section agree.
Mint a test with

```sh
tools/duck_launcher/newtest.py --from a1v2_r13_tax001.json --slug trace05 \
    --title "attitude-row trace 0.5" --set "*.model_trace=0.5" \
    --why "temporal depth on the attitude rows so a torque becomes a lean inside the model"
```

which takes the next number, writes the config with the overrides and the name
`R<nn> · attitude-row trace 0.5` at rank 1000 + nn, and appends a preset of the same
name whose controls are copied from `★ PIPELINE 1/3` (from scratch, 2 h, saves the
brain). At a milestone, prune: drop `launcher_rank` from the refuted tests (the files
stay, their names keep the verdict), rename the winner to a ★ role and give it a low
rank, and move its series preset to `presets_archive.json` (not read by the launcher; the exact
argv of every measured arm is kept there and in the design doc). `--selftest` checks every preset still builds.

## Exposing a config

The dropdown shows configs that carry `metadata.launcher_rank` (lower first),
under their `metadata.name`; "show all" lists the rest by filename. To surface a
config you have designed, give it a rank and a name in the Godot launcher's
convention — **ROLE — mechanism · what you'll see** — for example

```
"name": "★ PIPELINE 1/3 — R13 find · from scratch WITH ident 12/3000: seed 2 stands by ~30 min",
"launcher_rank": 20
```

Metadata is not read by the host, so this never changes a run. Presets live in
`presets.json` beside the launcher: a name, a hint, and the control values to
set. Add an experiment there, not in the code. At a milestone the list is pruned to the `★` stacks,
the guard, and the arms still waiting for the operator's eye; the retired presets go to
`presets_archive.json` beside it (last pruned 2026-10-02, 29 → 20, at ★ TURN; 2026-10-01, 93 → 18, at ★ BIRD; 2026-09-27, 82 → 13).

## Watching a level-2 run

The first 600 s of every level-2 run are the identification babble: the duck pulses its twist
in place and does not tour until 600 s. The `level-2 watch: fast-forward until` control (the
R26–R30 presets set it to 600) passes `--fast-until` to the host's pacer and to the viewer, so
the babble goes by in seconds and the tour is watched at real time. The run is tick-for-tick
the harness run: `--realtime` and `--fast-until` only touch the pacer (checked 2026-09-10).

Presets minted by `newtest.py` while the launcher is open appear after **↻ presets**. Picking a
config from the config list clears the preset and keeps whatever seed and controls were set —
check the seed line before trusting a watched run against a harness number.
