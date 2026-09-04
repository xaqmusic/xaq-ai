# The brain builder — design and status

**Status (2026-09-04): M1–M5 shipped on branch `microduck-lean-prior`** (commits
`da86017`, `319822c`, `3981a22`, `e33ce73`, and the docs commit). M6, the live
link, is deferred until the microduck science resumes. Operating notes are in
[`brain_builder/README.md`](../../brain_builder/README.md).

## Why

The goal has always been a graph surface where a human connects a brain from
scratch: the full selection of modules on a palette, the body's pre-configured
sources and sinks as fixed nodes, wiring by dragging. The Godot graph panel got
part way and stalled: it is bound to the Godot render loop, its type table
knows 11 of the 70 registered types, and adding a module means typing raw
JSON. The inspector is the right place to *watch* a brain, not to build one.
So the builder is a separate Dear ImGui desktop app that links the core
directly; the inspector stays the observation app.

## Decisions

| decision | why |
|---|---|
| C++ Dear ImGui app linking `ogma_core` | the registry, the param schemas and trial setup are C++; a Python binding would be more work than the app |
| v1 is offline: load / build / validate / dry-run / save / publish | the config is the contract; a live link is a later milestone |
| body surfaces are hand-authored JSON manifests | the hosts register sources in GDScript and C++ with no shared surface to read; a manifest per body, checked in, written from the host's own code |
| the document is JSON-first, never a `GraphConfig` | `GraphConfig::to_json()` drops `metadata`, `description` and `_comment`; the builder must keep what it did not write |
| socket discovery is a one-off generator, not a startup cost | `--gen-palette` probes every string / list / level param and diffs the ports; the reviewed result lives in `palette.json` |
| node positions live in `metadata.builder.layout` | a layout travels with the file and diffs in git; launchers and the core ignore the key |
| no core, host, or Godot code changes | the builder is a consumer of the registry exactly like the hosts |

## What probing found (and could not)

The generator sets a probe value on each candidate param, re-runs trial setup,
and reads the port that appears. This finds exact sockets (`{input_topic}`),
composed ones (`reality.{modality_group}.{modality_name}`), prefix
subscriptions (LateralVoter's `input_pattern`), per-entry list sockets
(`action_topics`), and `consensus.{level}` from an int. Across the registry:
355 sockets and 56 fixed topics.

Two things the probe cannot see, both handled by rule:

- **Polled topics.** A module may name a topic in a param and read it with
  `last_value` without ever declaring it as a port. MotorEPMv2's
  `regime_topic` is one. Any `*_topic(s)` param the probe did not see becomes
  a *polled* socket, drawn with a grey ring. (72 across the registry.)
- **Tolerated required params.** MotorEPMv2 declares 24 params with no
  default that it constructs fine without (it has internal fallbacks). The
  generator records the schema-required params the constructing baseline omits
  as `tolerated_missing`, and the builder stops demanding them.

## Verification

- `brain_builder/tests/roundtrip.sh`: r19 and r25 round-trip byte-identical;
  the two picrawler reference configs are semantically identical (every key,
  module order) with whitespace differences from the tooling that wrote them.
- `brain_builder/tests/dryrun.sh`: r19 and r25 construct, tick 50 times and
  drive all 14 / 3 body sinks; the publish plan for r19 names `a1v2_r26_…`.
- unit tests: the catalogue covers the registry; r19 wiring (body → bridges →
  MotorEPMv2 → sinks, plus the feedback path); connect / disconnect / undo on
  an empty graph; topological order keeps the bridge before its MotorEPMv2 and
  names the cycle.
- the cell (`the_cell_cognitive.json`, 16 modules) and the picrawler rung0
  config (26 modules) validate with zero errors.

## M6 — later

Live link over the inspector control socket: `list_modules` and
`module_snapshot` to overlay TLE on nodes, `set_param` for hot patches;
`--check-bodies` to compare a manifest with what a running host reports; a
read-only graph tab in the inspector if still wanted. Live add / remove /
connect would need new verbs on the socket (the Godot in-process `apply_patch`
path is not exposed).
