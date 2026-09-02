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
