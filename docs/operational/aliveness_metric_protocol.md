> **Ported from the pre-split `ami-ogma` repo, 2026-07-25.** ⚠️ Written in the RL era, and its
> named tooling/metrics (`max_distance_from_origin`, the aliveness panel) are old-repo artifacts.
> **The question it raises is still OPEN and still live:** distance-style metrics can reward dead
> drift and select *against* closed-loop adaptation. The current metric set (`net_z`, `net_disp`,
> `straight`) is distance-flavoured in exactly the way this doc warns about. Tracked as an open
> question in [`../reports/picrawler_lever_ledger.md`](../reports/picrawler_lever_ledger.md) §7;
> the partial answers adopted so far are the blind-metric discipline
> ([`../../CLAUDE.md`](../../CLAUDE.md) §3 rule 4) and the operator's three aliveness phenomena
> (§3.3). `ami_ogma`/`ogma` == xaq.

# Picrawler Aliveness Metric Protocol

**Status:** first operational draft, created after Phase 8 closed the open-loop
action-vocabulary bet.
**Companion findings:** `docs/findings/phase8_findings.md`,
`docs/findings/phase7_20_findings.md`, `docs/findings/mechanism_registry.md`.

## Why this exists

Phase 8 showed that `max_distance_from_origin` is an incomplete north-star
metric. It can reward dead drift, circles, and random-walk amplitude while
underrating the behaviour the project actually values: closed-loop adaptive
reorganisation. The Phase 7 Cruse lineage often looked worse on distance but
more alive to the operator; the Phase 8 posture/gait-option lineage sometimes
matched distance while looking like rote replay.

This protocol makes that distinction explicit. New picrawler work should report
legacy navigation metrics, but it should not be promoted on distance alone.

## Metric tiers

### Tier 0 -- retrospective proxy

Use existing summary JSONs to compute an `aliveness_proxy` without rerunning the
sim:

```bash
python scripts/aliveness_score.py results/phase7_hier_epm/longrun.json \
  --baseline-label A_baseline_60min
```

The proxy uses fields already present in historical summaries:

| Component | Existing fields | Interpretation |
|---|---|---|
| movement | `total_path_length / duration_s` | Body is doing work in the world. |
| stability | `pct_upright`, `pct_tipover`, `pct_below_fail_height` | Motion remains embodied rather than collapse-only. |
| learning | `pre_w_growth` | Premotor weights are being reshaped by experience. |
| motor variation | `joints_var_late` or `da_mean` fallback | Behaviour is not frozen to one pose. |
| recovery | `longest_upright_physics_ticks` | Long unbroken embodied stretches survive. |
| directionality report | `end_distance / path`, `max_distance / path` | Flags circles/wander; not treated as aliveness by itself. |
| penalties | resets, tipover, low path under high DA | Flags jitter traps and collapse-heavy runs. |

Tier 0 is useful for ranking old arms and detecting obvious metric inversions.
It is **not** enough for a new claim because it cannot observe perturbation
response directly.

### Tier 1 -- perturbation/obstacle assay

Future A/Bs should include an assay where the body must reorganise under world
change. Minimum design:

1. Stand/walk warmup under the normal curriculum.
2. Inject a perturbation: temporary obstacle, blocked path, shifted target,
   terrain step, or controlled body push.
3. Continue after perturbation release.

Required per-run summary fields:

| Field | Meaning |
|---|---|
| `perturb_tick` / `release_tick` | When the challenge starts and ends. |
| `pre_perturb_path_rate` | Movement rate before perturbation. |
| `during_perturb_path_rate` | Whether the body keeps acting or freezes. |
| `post_release_path_rate` | Whether motion recovers after release. |
| `prediction_error_recovery_s` | Time for fused TLE / surprise to return near pre-perturb baseline. |
| `policy_entropy_shift` | Whether the policy changes after perturbation. |
| `sensorimotor_coupling_delta` | Whether action changes predict sensor changes better after recovery. |
| `stuck_escape_count` | Count of meaningful escape attempts, not just jitter. |
| `target_progress_after_block` | Target-approach progress after the perturbation. |

Tier 1 success means the variant does more than move: it changes behaviour when
the world changes and recovers into embodied action.

## Decision rule

A future mechanism should be promoted only if one of these is true:

1. It improves legacy navigation **and** aliveness/adaptation at adequate power.
2. It trades distance for a pre-registered aliveness gain, with operator-visible
   adaptive behaviour and a Tier 1 perturbation signal.
3. It is explicitly mechanism-only infrastructure, in which case the finding
   must say so and must not claim locomotion progress.

Reject or archive mechanisms that improve only `da_mean`, `max_distance`, or
single-seed visual impressions while failing aliveness checks. Phase 7.20,
Phase 7.21, and Phase 8 are the reference failure patterns.

### Emergence instrument -- standing→gait latency (2026-05-29)

The Tier 0 proxy ranks *marginal* differences between arms; that is the wrong
instrument for the gait-ignition reframe, where real efficacy must be
non-ambiguous in a single ~20-min run (the way standing is). The operational
emergence instrument is **standing→gait latency**: time from standing-achieved to
the first sustained gait bout within one run.

```bash
python scripts/gait_ignition.py results/<run_dir>/<arm>_seed<N>.json
```

`scripts/gait_ignition.py` reads per-seed trajectory dumps and detects:
`t_stand` (chassis height held), then a gait bout (upright held + directed net
displacement + diagonal `feet_y` stepping over a window). It reports
`standing_to_gait_latency_s`, or ∞ (no ignition).

Calibration finding (2026-05-29): **no run on disk ignites** — every artifact,
including the most "mobile" 60-min runs (path 144 m), reads ∞, because they
accumulate distance by slow shuffle (mean ~0.13 m/s, feet barely alternating),
not stepping. ∞-everywhere is the honest baseline a real ignition mechanism must
break. The detector's positive path is validated against a synthetic trot.

Decision rule for this instrument: a mechanism is interesting only if it produces
a finite latency / operator-visible sustained bout in a single run across a couple
of seeds. Marginal 45-min aggregate deltas are auto-rejected without an n=10
delta-hunt. The bout definition is deliberately strict so jitter-in-place reads as
no bout (guards Pattern E).

### Aliveness panel — the operator's three signals (2026-05-29)

`scripts/aliveness_panel.py` scores ignition latency plus the three phenomena the
operator watches for: (1) heading regulation (faces radial-out + re-corrects against
noise), (2) proto-gait stumbling steps (`phase_contrast` + displacement), (3) obstacle
adaptation (per-EPM TLE spike on pyramid contact + behavior change). It is
measurement-forward: it reports distributions + provisional events so thresholds are set
from a real run.

Requires enriched diag fields (added to `picrawler_body.gd._emit_jsonl`, GDScript-only):
`heading_yaw`, `radial_compass[2]`, `nearest_pyramid_dist`, `epm_tle{}` (per-EPM TLE, read
from `get_module_metrics` key `tle`). `OGMA_PICRAWLER_PYRAMID_MIN_R/_MAX_R` move the pyramid
ring in.

Status: signals 1+2 validated and measurable; **signal 3 is downstream of translation**
(a 20-min baseline reaches only ~0.67 m, so a 2–3 m pyramid is unreachable) — treat it as a
post-ignition confirmation, not a pre-condition.

## Current next use

Use standing→gait latency as the gate for the gait-ignition bet
(`docs/plans-and-designs/gait_ignition_theory.md`): wire the existing
`HomeokineticExploration` drive into the Cruse v2 + perceptual-CPG stack as a
standing-basin destabiliser and check whether a bout ignites within a 20-min run.
The Tier 1 perturbation assay remains the follow-on once any mechanism produces a
real bout to perturb.
