# Brain Builder

A desktop app for wiring a brain from scratch on a graph surface: every
registered module on a palette, the body's sources and sinks as fixed nodes,
topic wiring by dragging.  It edits the GraphConfig JSON that `mj_host` and the
Godot host run unchanged; it never runs a brain against a body itself.  The
inspector (`tools/xaq_inspector`) stays the real-time observation app.

Dear ImGui + imgui-node-editor + GLFW, linking `ogma_core` directly: the module
catalogue is the registry, the param schemas are `params_schema()`, and the
pins come from a trial `on_setup` on a scratch bus, exactly as the scheduler
validates a hot patch.

```sh
./brain_builder/run.sh build                # first run fetches ImGui, GLFW, node-editor; builds ogma_core
./brain_builder/run.sh open mj_host/configs/a1v2_r19_settle_each.json
./brain_builder/run.sh open                 # a new graph on the duck at the joints
./brain_builder/run.sh tests                # unit tests
brain_builder/tests/roundtrip.sh            # the four reference configs load → save unchanged
brain_builder/tests/dryrun.sh               # r19/r25 construct, tick, and drive every sink
```

## The surface

| panel | what it does |
|---|---|
| **Palette** | the 70 registered types by category, searchable; drag one onto the canvas (or right-click the canvas) |
| **Canvas** | nodes tinted by category, pins coloured by payload type, links by topic family. Drag output → input to wire. Delete a link to clear the param. Hover a pin for its topic and the manifest text; hover a red node for its setup error |
| **Properties** | the selected module's params from its schema: bool / int / float / enum / string / list editors, a topic picker on socket params, filters (required / set / sockets / hot), reset-to-default, unknown keys flagged |
| **Execution Order** | the modules array, which is the tick order; drag to reorder; "Sort topologically" previews a suggestion and names any cycle it kept |
| **Validation** | setup errors, unresolved required inputs, unread outputs, actions the body will not act on, payload mismatches; **Dry run** constructs the brain for real, feeds synthetic body input for N ticks, and reports which outputs published and which body sinks were driven |
| **File ▸ Publish** | writes the config where a launcher finds it: `mj_host/configs/a1v2_r<nn>_<slug>.json` numbered and ranked the way `tools/duck_launcher/newtest.py` does, or the Godot config dir with the metadata `launcher.gd` needs (its allowlist stays a one-line GDScript edit) |

Keys: Ctrl+N/O/S/P, Ctrl+Z/Y, F fits the view, Delete removes the selection.

## How wiring works

Modules subscribe by topic string, so a link is a *match*: an output pin and an
input pin with the same topic (or a trailing-dot prefix that covers it).
Dragging a link is therefore a **param edit**: the producer's topic is written
into the consumer's socket param — a scalar, a list entry, or a composed
split such as `reality.<group>.<name>` into `modality_group` + `modality_name`.
Everything is undoable.

Three kinds of pin:

- **live** — a topic the module declared in its trial setup (filled circle);
- **placeholder** — a required socket whose param is unset (hollow circle);
  optional inputs hide behind one **+** pin: drop a producer on it and pick the
  socket;
- **polled** — a topic the module names in a param but never declares as a port
  because it reads it with `last_value` (grey ring). MotorEPMv2's
  `regime_topic` is one; the Godot panel never drew that link.

**Execution order is behaviour**: the scheduler ticks the array top to bottom
in one level. Nothing reorders implicitly; new modules append, and the
topological sort is a suggestion you apply.

## Files

| path | role |
|---|---|
| `palette.json` | the catalogue's hand-authored half: category, layer, purpose, id prefix per type — and the generated half: **sockets** (which params name topics, verified by probing), fixed topics, probed kinds, and the schema-required params the constructing baseline tolerates missing. Regenerate the generated half with `./brain_builder/run.sh gen-palette`; review the diff like code |
| `bodies/*.json` | one manifest per body, hand-authored from the host's own publish/poll code: sources (topic, dims, description), sinks, host reads, events |
| `src/Catalogue` | registry → param schemas + palette |
| `src/PaletteGen` | the probe: set a probe value on each string/list/level param, re-run trial setup, diff the ports |
| `src/Graph` | the document: ordered JSON, only `modules[]` and `metadata.builder` are the builder's; undo by snapshot |
| `src/Wiring` | pins, links, diagnostics, connect/disconnect |
| `src/DryRun`, `src/Order`, `src/Publish` | what the names say |
| `src/ui/*` | one file per panel |

CLI modes for scripts and checks: `--dump-catalogue`, `--list-types`,
`--gen-palette [--merge]`, `--ports config`, `--validate config` (exit 1 on
errors), `--dry-run config [--ticks N]`, `--roundtrip in out`,
`--publish-dry config --title T`.

## Conventions the builder keeps

- Node positions live in the config under `metadata.builder.layout`, so a
  layout travels with the file and diffs in git. Launchers and the core ignore
  the key; `tests/roundtrip.sh` judges identity without it.
- Configs written by `newtest.py` use ASCII escapes; the builder reproduces
  the style of the file it loaded, so an unchanged r19 saves byte-identical.
- The `purpose` one-liners in `palette.json` are first drafts written from the
  module names and the campaign history; correct them where they are wrong.
- Wayland: GLFW is built for X11 here (no Wayland dev headers on this machine)
  and runs through XWayland with the compositor's own window frame. Build with
  `-DBRAIN_BUILDER_WAYLAND=ON` once `libwayland-dev wayland-protocols
  libxkbcommon-dev` are installed.

## Live: editing a brain while it runs

```sh
./brain_builder/run.sh live                 # connect to 127.0.0.1:7400 (mj_host or the Godot host)
./brain_builder/run.sh live 127.0.0.1:7500  # another port (a battery host, a robot)
```

File ▸ Connect pulls the running graph over the same control socket the
inspector uses, opens the host's source file for its metadata and layout,
and marks the document **LIVE** in the menu bar. From then on every edit
becomes a hot patch, sent about a third of a second after you stop editing:

| edit | what the host gets |
|---|---|
| drop a module, wire it, set its params | one `add_node` with the params, so the topics are set at construction |
| delete a module | `remove_node` |
| change a hot-mutable param | `set_param` |
| change a construction-only param (a topic, a dimension, a seed) on a running module | nothing yet: a prompt offers **Recreate** (remove + add, which loses that module's learned state) or **Revert** |
| reorder | nothing: the host ticks in its own order; save the file and it applies at the next restart |

The host validates every added module by trial construction *before* taking
its tick mutex, so a real robot's control loop never waits on a large module's
setup. A refused patch is logged with the host's reason and the link shows
**OUT OF SYNC** until File ▸ Resync from host.

**Several clients at once.** The control server gives each client its own
connection and serializes requests. The inspector keeps its own diagnostic
subscriptions (the builder never touches them), and it polls the host's
`graph_version` every two seconds, so a module you add appears in its list
and a module you remove drops out of it. The builder polls the same version
every second: when the Godot panel, another builder, or the inspector's own
`set_param` changes the graph, it pulls the new graph and redraws. Nothing
here alters the brain's computation: with no client connected the host runs
exactly as before.

Protocol, for other clients (newline-delimited JSON on the control port):
`{"verb":"get_graph"}` → the live config, edges, `source_path`, `graph_version`;
`{"verb":"apply_patch","ops":[...]}` with the Godot panel's op shapes
(`add_node`, `remove_node`, `connect`, `disconnect`, `set_param`) → `batch_id`
and the new version, or the errors; `{"verb":"graph_version"}`. The logic
lives in `cpp_core/include/ogma/LiveGraph.hpp` and both hosts call it.
`brain_builder --connect host:port [--pull out.json] [--patch ops.json]` speaks
it from a script.

## Not yet

Overlaying live TLE on nodes; explicit `connect` edges from the canvas (live
wiring today is by construction params, as the configs are); `--check-bodies`
against a running host. See `docs/plans-and-designs/brain_builder_plan.md`.
