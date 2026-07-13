# Archived Configs — Registered Baselines Index

**Purpose:** This dir holds picrawler configs paired down from the active launcher
on 2026-05-30 (commit `713aa5f`). Most are dead experiment arms. A few are the
substrate basis for *registered findings* in `docs/findings/mechanism_registry.md`
or named in `memory/MEMORY.md`. Restoring or referencing those for replication
should be cheap — this index makes them locatable without digging through 30+ files.

**Convention:** before archiving any config, check this index. If it's named here
as a registered baseline, leave a forwarding note in the new dir or update this
index with the new path. Do NOT silently delete configs that underpin registry
findings — we lose the ability to A/B against our own history.

---

## Registered baseline configs

### B0 baseline — "first brain-driven evidence" milestone (2026-05-19)

- **`the_picrawler_stand_target.json`** (11k)
- **`the_picrawler_stand_target_lr_symmetric.json`** (11k)
- Memory: [[v6-picrawler-first-evidence]], [[v6-stage-b-lr-symmetric]]
- Result: B0 91% upright > A5 84% > B1 28%; LR-symmetric Premotor wins n=20
  (-69% falls, +152% continuous stand, +32% walking distance)
- **Audit status:** these runs pre-date the `reward_cum` telemetry — cannot be
  retroactively audited for dominance. **Replication under current code required**
  to confirm the milestone holds. See `mechanism_registry.md` §3p.
- Trajectory data: `results/picrawler_B0_baseline_n20/` (20 seeds)

### Phase 7.5.R — base config (RECOVERED 2026-05-31 → ACTIVE BASELINE)

- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot.json`** (17k)
- **2026-05-31:** restored from archive to active configs dir after S1.1
  replication test confirmed body stands (chassis_y=0.096 vs handtuned_v1's 0.033)
  and reward is balanced (no channel >60%). This is now the **active substrate
  baseline** — visible in the launcher, ready for UI runs.
- Original archive entry: `results/phase7_5_long/seed44_long.json` (1-hour run)
- Historical result: gated 45.4% + standing 42.8% = the **one and only** non-dominant
  reward attribution among non-trivial pre-S1.1 runs. The `gated_walk_bonus`
  multiplicative gate design here was the load-bearing mechanism that broke
  standing-channel dominance. See registry §3p ("the 4 balanced exceptions")
  + §3q (S1.1 replication).
- Used with curriculum: `picrawler_stand_walk_gated_trot.json` (already in active
  `curricula/` dir)
- Memory: [[v6-phase7-5-r-recovered]]

### Phase 7.10 — synergy + value-head + entropy-anneal (2026-05-25)

- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_anneal.json`** (25k)
- Used by `results/phase7_21_progress_stack/seq/A_anneal_*` (10 seeds)
- Result: gated 44-46% + standing 42-44% across all 10 seeds — the other balanced
  run family. Confirms Phase 7.5.R recipe is reproducible at n=10.
- Memory: registry §3i ("Phase 7.20-7.21 full locomotion stack + progress reward")

### Phase 7.13 Cruse coordination lineage (2026-05-26 to -27)

- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse.json`** — Cruse v1
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v3.json`** — Cruse v3
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v4.json`** — Cruse v4
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v4_warmup.json`** — Cruse v4 + warmup
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v4_body_gated.json`** — Cruse v4 body-state gate
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v5.json`** — Cruse v5
- Memory: [[v6-phase7-13-cruse-consolidation]]
- Result: registry §3a Pattern A — all 12+ Cruse coordination mechanisms NULL at n=10+.
  **Important per audit §3p:** these were tested against standing-dominant baselines.
  Re-running any against a balanced baseline (e.g. Phase 7.5.R recipe) may reverse
  the null. cruse_v2 itself remains in the active configs dir.

### Phase-viscosity P1 dwell variants (2026-05-29)

- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v2_dwell4.json`**
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v2_dwell8.json`**
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v2_dwell12.json`**
- Memory: registry §3k. Pattern A regression: path collapsed from 115m → 1.46m
- Result: dwell8 path 1.46m, dwell4 path 7.46m vs baseline 115.64m

### Phase-viscosity P2 phase-bin commitment (2026-05-29)

- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v2_p2_phasebin8_penalty010.json`**
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v2_p2_phasebin8_penalty020.json`**
- Memory: registry §3m. Single-seed +0.061 aliveness on distance / 310 falls on tilt80.
  Retired before n=10 powering.

### Cruse ablation set (2026-05-26)

- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v2_off.json`** — Cruse off
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v2_no_rule3.json`** — minus Rule 3
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_cruse_v2_invert.json`** — inverted rules
- Reference for ablation A/Bs only; no standalone registered findings.

### Action vocabulary / Phase 8 (2026-05-28)

Phase 8 action vocabulary configs (A1 postures, A2 GaitSelector) are in this
archive but tested as full Phase 8 archive (`archive/phase8-action-vocabulary` git tag).
See registry §3j for the Pattern F finding (open-loop primitive without differential
outcome). Memory: [[v6-phase8-action-vocabulary]].

### Eligibility-trace + REINFORCE + MC-gamma variants (2026-05-24)

- **`*_perceptual_cpg_trot_elig_05.json`** — λ=0.5 eligibility traces
- **`*_perceptual_cpg_trot_elig_95.json`** — λ=0.95
- **`*_perceptual_cpg_trot_mcg_995.json`** / **`*_mcg_999.json`** — MC gamma sweep
- **`*_perceptual_cpg_trot_reinforce.json`** — textbook REINFORCE
- Reference: registry §3l (Phase 6.5.x prior-trail). Most null'd as Pattern C
  ("dead code under MC mode") — see memory [[v4-phase6-5-29-td-premotor]] etc.

### Other notable archived configs

- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_adv_norm.json`** — advantage
  normalization probe (registry Pattern D — already on by default, tautology)
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_synergy.json`** /
  **`*_synergy_value.json`** — Phase 7.9 synergy + value-head probes
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_value_head.json`** — Phase 7.8
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_per_leg.json`** — Phase 7.7
  per-leg credit decomposition
- **`the_picrawler_stand_target_per_servo_perceptual_cpg_trot_diag_voters.json`** — Phase 7.6
  diagonal-pair voters (Pattern A null)
- **`the_picrawler_stand_target_per_servo_72epm_chunks_compass_cpg.json`** etc. —
  hierarchical-perception Phase 7.2-EPM probes (Pattern A/B)

---

## Curriculum archive

`godot_host/project/curricula/archive/` holds the matching curriculum files.
Notably:

- **`picrawler_stand_walk_gated_per_leg*.json`** — per-leg gated variants (Phase 7.7)
- **`picrawler_progress_full_stack.json`** / **`picrawler_progress_to_target.json`** —
  the progress-PB reward design (registry §3h Goodharted, §3i Pattern E)

The base `picrawler_stand_walk_gated_trot.json` (Phase 7.5.R curriculum) remains
in the active `curricula/` dir.

---

## How to use an archived config for headless replication

No need to restore. `picrawler_run.py` passes `--config` through verbatim:

```bash
conda run -n ami-ogma python scripts/picrawler_run.py \
  --seed 50 --duration 2700 \
  --config res://addons/ami_ogma/configs/archive/the_picrawler_stand_target_per_servo_perceptual_cpg_trot.json \
  --curriculum res://curricula/picrawler_stand_walk_gated_trot.json \
  --output-dir results/<label> \
  --label <label>
```

Active launcher UI stays pared. Only restore to parent dir if a config
becomes a long-term reference (e.g. new substrate baseline after S1.1).
