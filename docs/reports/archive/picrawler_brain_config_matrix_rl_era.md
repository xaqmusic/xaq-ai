> **ARCHIVE — ported from the pre-split `ami-ogma` repo, 2026-07-25.** ⚠️ **This documents the
> reward-shaped RL era of the picrawler**, which [`../../the-picrawler-detour.md`](../../the-picrawler-detour.md)
> disowns as the cautionary origin story. **Its individual mechanism verdicts do NOT transfer** to
> the current reward-free active-inference stack — different substrate, different objective,
> different baseline. It is kept as an honest record and because its **failure shapes and
> measurement lessons are permanently valuable** (those are distilled into
> [`../picrawler_lever_ledger.md`](../picrawler_lever_ledger.md) §7 and
> [`../../../CLAUDE.md`](../../../CLAUDE.md) §3.2). For the **current** verdicts, read the ledger.
> `ami_ogma`/`ogma`/`AMI-Ogma` == xaq.

# PiCrawler Stage B — Brain-Config Test Matrix Report

**Date:** 2026-05-19
**Plan:** `~/.claude/plans/i-ve-observed-the-system-proud-lynx.md`
**Configs:** all stage variants live under `godot_host/project/addons/ami_ogma/configs/the_picrawler_stand_target*.json`
**Per-seed data:** `results/picrawler_B0_baseline_n20.json`, `results/picrawler_B1_hebbian.json`, `results/picrawler_B3_lr_symmetry.json`

## Setup

All A/Bs at n=20 paired seeds (42–61), duration 1800s sim (30 min), `OGMA_PICRAWLER_MC_PERIOD=1500`, `--turbo --parallel 4`. Curriculum disabled to avoid the teleport-on-stage-change confound. Both arms ran identical `auto_reset_on_inversion=false`. Self-check Δ=0 verified before each run.

Anchor (B0 baseline, n=20): chassis_y_max μ=0.129m, n_fall_events μ=57.2, pct_below_fail μ=0.66, longest_upright μ=29,295 ticks (8.1 min). Matches the live 30-min UI snapshot within sanity bands.

## Results

### B1 — Hebbian on/off (NULL)

| Metric | Δμ | σ | p | 95% CI |
|---|---|---|---|---|
| All 9 paired metrics | **+0** | **0** | **1.00** | **[0, 0]** |

Bit-identical trajectories across all 20 paired seeds. The Hebbian matrix populates as observed in the 30s smoke (nnz=36, |w|≈823) but **nothing downstream reads it** — `LateralVoter.cpp:499–511` updates `assoc_` and never consumes it. Confirms the Phase 1 stub claim in `LateralVoter.hpp:18–20`. Trust shares come purely from inverse-TLE + group_balance; Hebbian on/off changes neither.

**Action:** abandon as a path. Re-enabling Hebbian requires building a downstream consumer (chunk-key lookup, trust prior, or fused-embedding modulator) — that's a Phase-3 design exercise, not a knob.

### B3 — LR-symmetric Premotor sharing (STRONG WIN)

| Metric | indep_legs μ | lr_symmetric μ | Δμ | p | CI |
|---|---|---|---|---|---|
| pct_below_fail_height | 0.657 | **0.268** | −0.389 | <0.0001 | [−0.54, −0.22] |
| pct_high_tilt | 0.703 | **0.280** | −0.423 | <0.0001 | [−0.58, −0.26] |
| tilt_mean (rad) | 2.135 | **0.833** | −1.302 | <0.0001 | [−1.77, −0.80] |
| n_fall_events | 57.2 | **17.7** | −39.5 | 0.005 | [−69, −16] |
| longest_upright_ticks | 29,295 | **73,914** | +44,619 (+152%) | — | — |
| max_distance_from_origin | 4.73 | **6.25** | +1.52 (+32%) | — | — |
| pre_w_growth | 28.1 | **44.4** | +16.3 | <0.0001 | [+10, +23] |
| da_mean | 0.161 | 0.108 | −0.053 | <0.0001 | [−0.076, −0.030] |
| chassis_y_max | 0.129 | 0.129 | −0.0002 | 0.96 | null |
| first_sustained_fall_tick | 10,839 | 18,456 | +7,617 | 0.24 | trending |

**Mechanism confirmed:** `leg_symmetry_sync_count` increments per `mc_episode_period` boundary; per-pair `pre_W` divergence collapses to <0.01 within one sync (was 4.3→8.9 range in baseline). Front-vs-rear asymmetry preserved (front Premotors converge to one shared policy, rear to a different shared policy).

**Interpretation:** The user's UI observation was right — independent legs were diverging into four uncoordinated policies. Forcing left/right mirror symmetry every 25s of sim:
- Cuts the effective Premotor parameter count in half, so MC REINFORCE credit is shared per pair instead of split across 4 separate weight matrices.
- Lets the brain learn a coherent quadruped gait instead of 4 leg-specific reflexes.
- DA suppression is the expected concomitant of fewer `miss` events (fewer falls → less DA fluctuation).
- The +32% walking distance was unexpected — symmetry was hypothesised to help only standing. Coherent gait helps locomotion too.

**Caveat — convergence is slower in the early-learning phase.** The user's UI observation ("standing fine but slower than baseline") matches: the brain has to find a shared policy that works for both legs of a pair, which is harder than independently fitting each leg. The 1800s headless horizon was long enough for the lr_symmetric arm to overtake; shorter runs would not show the win.

### B4 — Combined (skipped)

Gate required ≥2 of {B1, B3} to pass. Only B3 passed → B4 not run. Even if Hebbian had not been inert, combining it with LR-symmetric would not have changed anything since Hebbian has no downstream consumer.

## Recommendation

**Promote LR-symmetric to default.** Two paths to ship:

1. **Cheap & immediate:** ship the LR-symmetric variant config (`the_picrawler_stand_target_lr_symmetric.json`) as the new default in the launcher dropdown and any "recommended" preset. Body-side hook (post-hoc weight averaging at mc_period boundary) already lives behind `OGMA_PICRAWLER_LEG_SYMMETRY` env var.

2. **Proper:** add `shared_with` aliasing to the Premotor C++ class so weight matrices are physically shared (not periodically averaged) — eliminates the 25s sync overhead and the destructive-averaging concern entirely. ~40 lines in Premotor.hpp + restore-state plumbing. Requires C++ rebuild.

Path 1 is justified given the magnitude of the win (−69% falls, +152% continuous stand, +32% walking). Path 2 is the architectural cleanup once the result has been confirmed in the live UI for a few sessions.

Per memory `feedback_no_tuning`: this win came from adding an **adaptive mechanism** (sync at MC boundaries, derived from the system's own episode boundaries) rather than from tuning a static parameter. The next architecture step (`shared_with` in Premotor) follows the same principle — bake the constraint into the structure rather than enforcing it via periodic post-hoc patches.

## Anti-goals satisfied
- Curriculum disabled during A/Bs (no teleport contamination)
- n=20 paired seeds for both A/Bs (above the v9 retraction threshold)
- Both arms shared identical `auto_reset_on_inversion=false`
- Self-check Δ=0 verified before each variant run
- No Premotor or LateralVoter behaviour changes in Stage B — only observability counters added

## Investigated post-hoc

- **`chassis_y_mean_late` anomaly — RESOLVED.** Values in the −155 to −640 range were not an aggregator bug per se; they were a metric-encoding bug poisoning the mean with off-world free-fall events. Root cause: floor was 20×20m, and with B3 variant's +32% walking distance some seeds wandered past `x=±10` or `z=±10` and accelerated downward in gravity (no terminal velocity). The mean of `[0.08, 0.08, …, −3, −8, −14, −20, …, −3500]` is dominated by the free-fall tail.

  **Fixes landed:**
  1. `picrawler_run.py` — `chassis_y_mean_late` now uses `max(0.0, y)` per tick before averaging (off-world ticks contribute 0, not their meaningless negative value). New column `n_offworld_ticks` makes the contamination visible.
  2. `picrawler_body.gd::_build_terrain()` — 45° outward-sloping wedges at all four floor edges (containment + slide-down challenge) + 12 random low pyramids in the r∈[3.5, 9] m donut (terrain to walk around / over). Layout uses a fixed terrain RNG seed so all body seeds see the same world — preserves A/B paired-seed comparability.

  Re-aggregating B0's per-seed data with the fix: `y_mean_late` μ = +0.025m (sensible) vs raw μ = −155m (poisoned). 2/20 baseline seeds were off-world. The headline B3 metrics (`pct_below_fail_height`, `n_fall_events`, `longest_upright_physics_ticks`, `tilt_mean`) are robust to off-world events and were never affected — the win stands.

- **lr_symmetric promoted to launcher default.** `the_picrawler_stand_target_lr_symmetric.json` now has `derived_from_manifest: true` and renamed to "PiCrawler — LR-symmetric Premotors (current default)". The unmodified target.json remains in the dropdown for A/B comparison.
