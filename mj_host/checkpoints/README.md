# Brain checkpoints

Full-brain snapshots (`--save-brain` / `--load-brain` on `ogma_mjhost`): every
module's working state — including earned consolidation — plus the bus cache
and the exact physics state (qpos/qvel).

- **`duck_tall_brain_s2.json`** (2026-09-01): THE TALL STANDER.  R8 stack
  (`a1v2_r8_tall.json`), seed 2, saved at t=7200 s after 80 straight minutes
  of zero-fall consolidated tall standing (upright 1.00, tilt ~1°,
  pose-distance 0.07 from the calibrated stand pose, cons 1.00 both modules).
  Resume gate passed: 300 s restored → 0 falls, cons intact.

  Resume with:
  ```
  mj_host/build/ogma_mjhost --brain --graph mj_host/configs/a1v2_r8_tall.json \
      --secs <N> --seed 2 --load-brain mj_host/checkpoints/duck_tall_brain_s2.json
  ```
  (No `--ident` flags on resume — the identification is long done.)

- **`duck_controlled_brain_s2.json`** (2026-09-01): THE CONTROLLED STAND.  R12b
  stack (`a1v2_r12b_controlled.json`), evolved from `duck_tall_brain_s2` through
  the gain hunt (C norms 40-60 → ~4-10) and full rest at earned stillness
  (`consolidate_rests_act` 1 + prior resting).  One hour verified: 0.12-0.43
  mrad/tick joint motion (the pre-fix tall stance ran 12.5), upright 1.00,
  tall pose 0.07, |u| 0.30, one self-recovered wobble.  Resume with the R12b
  config; no --ident flags.

- **`duck_pipeline_s2.json`** (2026-09-02): THE FROM-SCRATCH STANDER.  No
  old-era lineage: find (`a1v2_r13_tax001.json`, 2 h, seed 2, ident flags)
  → hunt (`a1v2_r11_hunt.json`, 30 min) → rest (`a1v2_r12c_whole.json`,
  30 min), zero falls after the find, 0.1 mrad/tick, cons 1.00 (design doc
  §12.6).  Resume with the R12c config; no --ident flags.

- **`duck_r19_s2.json`** (2026-09-03): THE STANDER THAT CATCHES.  R19
  (`a1v2_r19_settle_each.json`: R13 with a settle before EVERY pulse window,
  `--ident-every 6 --ident-until 3000`, head hold 6), seed 2, 2 h from
  scratch — and the same on all six seeds: c 1.00 inside 15 min, 2-3 rescues
  in two hours, tilt 0.35°, pose 0.036, |u| 0.18, joint motion at the noise
  floor.  Catches 2 N shoves (35/36 across seeds, peak ~2.7°, back in 0.6 s);
  3 N is 13/36.  Resume with the R19 config; no --ident flags.
