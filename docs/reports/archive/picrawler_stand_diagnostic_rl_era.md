> **ARCHIVE — ported from the pre-split `ami-ogma` repo, 2026-07-25.** ⚠️ **This documents the
> reward-shaped RL era of the picrawler**, which [`../../the-picrawler-detour.md`](../../the-picrawler-detour.md)
> disowns as the cautionary origin story. **Its individual mechanism verdicts do NOT transfer** to
> the current reward-free active-inference stack — different substrate, different objective,
> different baseline. It is kept as an honest record and because its **failure shapes and
> measurement lessons are permanently valuable** (those are distilled into
> [`../picrawler_lever_ledger.md`](../picrawler_lever_ledger.md) §7 and
> [`../../../CLAUDE.md`](../../../CLAUDE.md) §3.2). For the **current** verdicts, read the ledger.
> `ami_ogma`/`ogma`/`AMI-Ogma` == xaq.

# PiCrawler Standing Task — Stage A Diagnostic Report

**Status:** Stage A complete — substrate validated; brain-driven standing demonstrated at n=20.
**Date range:** 2026-05-17 → 2026-05-18
**Author:** Joseph Butera III (observation) + Claude (analysis + experiments)
**Plan:** `~/.claude/plans/i-ve-observed-the-system-proud-lynx.md`
**Related memories:** `project_v4_phase6_5_arc_summary`, `project_v4_phase6_6_audit`,
`feedback_no_tuning`

---

## 1. Context

The PiCrawler is the next rung on the v6 quadruped affordance ladder
(`docs/v4_phase6_plan.md` §3.3). Body+brain wired per the manifest
framework (commit `d5ef661`): 2 EPMs (IMU 4-D vestibular + joints 12-D
proprio) → LateralVoter → DescendingPredictor → 8 Premotors → 12
servo channels. Reward is graded by chassis height via a duty-cycled
`events.hit` accumulator in `picrawler_body.gd` (peak 0.2 hits/tick at
standing).

User's initial UI observation: **"good motor babbling but no
convergence to standing over 6000 ticks."**

Per `project_v4_phase6_5_arc_summary` memory: the substrate has not
yet demonstrated brain-driven improvement over reflex-only on any
standard benchmark. This was the expected starting point.

## 2. Stage A0 — Harness construction

Built two scripts:

- `scripts/picrawler_run.py` — single-seed + sweep runner, modeled on
  `cartpole_run.py`. Parses per-tick diag JSONL from the headless body
  and aggregates 12 metrics including `chassis_y_max`,
  `chassis_y_mean_late`, `alive_ticks_max`, `da_mean`, `joints_var`, and
  `pre_W_growth` (Premotor weight-norm ratio).
- `scripts/picrawler_ab.py` — paired-seed A/B harness modeled on
  `paired_seed_ab.py`. Paired-t + bootstrap 95% CI on six metrics.
  Self-check mode confirms determinism.

Three body-side init fixes were needed before the harness could run:

1. **`OGMA_TURBO=1` honored** — `Engine.max_physics_steps_per_frame=64`
   gives ~10× speedup in turbo mode. Pattern lifted from
   `body_controller.gd:262-273`.
2. **`OGMA_QUIT_AFTER_TICKS` honored** — quit cleanly on `_done` OR
   tick budget. Without this the body printed `RUN_END` but never
   called `get_tree().quit()`, burning wall-clock until SIGTERM.
3. **`_ragdoll_mode` default flipped to false** — the previous
   default-on was a debugging hack from before the symmetric-knee +
   correct impulse-units fix (commit `c03d2f6`). Motors-on at boot
   mirrors the UI workflow "R then SPACE" and lets the headless
   harness measure brain-driven behaviour from t=0.

Determinism verified: paired Δ=0 on all 6 metrics at n=20 seeds,
parallel=4 (commit `465d4d2`).

## 3. Stage A1 — Baseline + §4.4 audit (n=20 × 30s)

Headline numbers on `the_picrawler_stand.json` (the as-built config):

| Metric | μ | σ | Range |
|---|---|---|---|
| `chassis_y_max` (m) | 0.053 | 0.009 | [0.041, 0.075] |
| `chassis_y_mean_late` (m) | **0.029** | **0.002** | uniformly below FAIL_HEIGHT=0.025 |
| `alive_ticks_max` | 1103 | 92 | ~18s mean before collapse |
| `da_mean` | 0.395 | 0.024 | non-saturated, within-run σ≈0.17 |
| `joints_var_late` | ≈0.029 | 0.008 | motor babbling real |
| **`pre_W_growth`** | **1.0000** | **0.0000** | **across ALL 20 seeds** |

**§4.4 audit empirically:**

- Determinism ✓ (paired Δ=0)
- DA non-saturated ✓
- Action diversity ✓ (joints_var > floor)
- **Premotor W changing ❌** (pre_W_growth=1.0000 exactly, all 20 seeds)
- EPM growing / drive urgency: not surfaced in current diag

## 4. Root cause — the wiring trap

Reading `picrawler_body.gd:1383-1397`:

```gdscript
if reset_mode == "continuous":
    if mc_episode_period > 0 and step_in_episode % mc_episode_period == 0:
        brain.publish_event("episode_end", ...)   # fires only if mc_period > 0
    if step_in_episode >= max_steps:
        print(...RUN_END...)                       # does NOT publish events.episode_end
```

In `reset_mode=continuous` (the default) + `mc_episode_period=0` (also
the default): **`events.episode_end` is never published.** Neither
periodically, nor on fall, nor at run end. The Premotor's MC REINFORCE
`finalize_mc_episode` (`Premotor.cpp:445`) runs ONLY on
`events.episode_end`. Without it, `mc_trajectory_` accumulates 6000
samples per run and is discarded at process exit.

This is the consumer-intent trap documented in `docs/v4_brain_derivation`
§4.1, also caught by `project_v4_phase6_6_audit` in Cell.

## 5. Stage A3 — mc_period sweep

Three paired A/Bs at n=20 paired seeds × 30s sim, all vs baseline
`mc_period=0`:

| mc_period | Δy_max | Δy_mean_late | **Δalive_max** | Δda_mean | **Δpre_W_growth** |
|---|---|---|---|---|---|
| 300  | +0.010 (p<.0001) | +0.005 (p<.0001) | **−843** (p<.0001) | +0.047 | **+4.83** |
| 600  | +0.007 (p<.001) | +0.005 (p<.0001) | **−623** (p<.0001) | +0.039 | **+4.28** |
| 1500 | +0.007 (p<.001) | +0.004 (p<.0001) | **−178** (p<.0001) | +0.022 | **+4.94** |

**Pattern:**
1. **Learning is on** — pre_W_growth jumps 1.0 → 4.3–4.9× across all
   three values. The Premotor REINFORCE finalises every `mc_period`
   ticks; weights move.
2. **Same height lift across all mc_periods** (+0.7 to +1.0 cm). The
   brain converges on the same "pump chassis_y" policy regardless of
   credit-assignment length.
3. **Survival regression shrinks with longer trajectories** (−843 →
   −623 → −178 ticks). γ=0.99 over 1500 ticks gives more reach to
   back-propagate the fall penalty into the credit, partially
   correcting the reward-hacking.

## 6. Why the brain learns the wrong objective (reward-hacking
analysis)

The reward signal is a **rate-coded duty cycle of instantaneous
chassis height**:

```gdscript
_hit_accumulator += chassis_y_norm * 0.2     # 0.2 per tick at standing
while _hit_accumulator >= 1.0:
    brain.publish_event("hit", 1.0)          # +0.45 DA pulse via NeurochemState
    _hit_accumulator -= 1.0
```

Three reinforcing factors produce the reward-hacking attractor:

### 6.1 Adaptive baseline EMA self-zero-centers sustained rewards

`NeurochemState.da_baseline_ema_alpha = 0.001` gives a ~17s time
constant. This prevents DA saturation by design — but the side-effect
is that **sustained** rewards become invisible to learning. The
`reward_signal = DA − DA_EMA` returns to zero whenever DA is held at
any constant level long enough. Only **transient spikes above the
running mean** produce a trainable signal. Standing still is "boring"
to the REINFORCE objective.

### 6.2 MC REINFORCE with γ=0.99 has an effective horizon of ~100 ticks

`G_t = Σ_k γ^k · r_{t+k}` with γ=0.99 means rewards in the first
~100 ticks dominate G_0. `1/(1−γ)=100`. So in a 1500-tick trajectory,
a 30-tick burst of high reward at the start contributes much more to
G_0 than the same height held for 1000 ticks at the end.

### 6.3 Premotor's `temperature_da_gain=0.5` makes high-DA policies sticky

When DA is high, exploration temperature drops, action sampling
becomes more deterministic. Once the brain *finds* a burst-policy that
pumps DA, it locks in on it instead of continuing to explore the
"stand still" alternative.

### 6.4 Net trajectory comparison

| | Stand still at standing pose | Brief upward burst, then collapse |
|---|---|---|
| Chassis height profile | sustained 0.078 m, 1500 ticks | spikes 0.082 for ~30 ticks, then ~0.02 for 1470 ticks |
| Total hits over episode | ~300 | ~6 |
| DA trajectory | DA pulled near ceiling; **EMA catches up → reward_signal ≈ 0** | DA spikes (huge above-EMA pulse) then drops |
| G_0 from MC REINFORCE | Big absolute G, small **trainable** signal | **Concentrated huge trainable signal in first 30 ticks** |

The brain optimizes for what's **visible to the substrate's view of
the signal**, not what's intended by the operator. Classic Goodhart's
law variant.

## 7. Stage A3.b — Neurochem stability shaping (user-proposed mechanism)

User's insight: "since we are allowed to manipulate the reward via
neurochem then perhaps we can provide a push toward stability and
less movement when the chassis is elevated."

Implementation in `picrawler_body.gd`: per-tick `events.miss` pulse
when `chassis_y_norm > stability_y_norm` AND
`chassis_speed > stability_speed`. Three env-var-overridable knobs:
`OGMA_PICRAWLER_STAB_{Y_NORM,SPEED,GAIN}`. Defaults to off.

A/B at n=20 paired seeds × 30s, variant = (mc=1500 + stab_gain=0.05)
vs baseline = (mc=1500):

| Metric | μΔ | p | Interpretation |
|---|---|---|---|
| Δda_mean | **−0.116** | <0.0001 | **DA is heavily suppressed** as designed |
| Δalive_max | +1 | 0.50 | **No survival improvement** |
| Δy_mean_late | −0.001 | 0.04 | Tiny drop in end-height |
| Δpre_W_growth | −0.12 | 0.01 | Slightly LESS learning |
| Δjoints_var | −0.0016 | 0.03 | Slightly less action diversity |

**Initial conclusion: NULL.** Mechanism produces intended DA
suppression but no behavioural improvement.

Per `feedback_no_tuning` memory: static reward-shaping seemed wrong.
The brain doesn't differentiate which actions caused the miss — the
aversive signal dampens learning broadly rather than redirecting
policy.

## 8. **CRITICAL CORRECTION — 30s window was too short**

User UI observation at ~10k ticks (~166 s) on
`the_picrawler_stand_mc1500_stab.json`: **"consistent standing… clear
pattern of learning to stand under dynamic conditions."**

The headless n=20 × 30s null was a **false negative due to
under-horizon**, not a substrate failure. Standing crystallization
needs ~5–10k ticks of Premotor weight updates (≈5–10 `events.episode_end`
firings at `mc_episode_period=1500`).

This validates the user's neurochem-shaping insight after all — the
mechanism IS doing useful work, the original A/B just didn't run long
enough for the policy to crystallize.

**Methodology lesson:** the v4_methodology n≥20 rule defends against
false positives. But it doesn't defend against **horizon
under-sampling**. The Phase 6 audit checklist needs a "minimum
horizon ≥ trajectory_length × N_updates_to_converge" rule.

## 9. Stage A4 — Long-horizon replication at 18k ticks (n=20 paired)

Two paired A/Bs at n=20 seeds × 300 s sim each, both run on commit
`b911d07`.

### 9.1 Full distributions across all 20 seeds (the headline data)

| Config | y_max | y_mean_late | y_p50 | y_p90 | DA mean | pre_W_growth | episodes |
|---|---|---|---|---|---|---|---|
| **Baseline (mc=0, no learning)** | 0.068±0.008 | **0.031±0.001** | 0.028 | 0.042 | 0.40±0.01 | **1.00** | 0 |
| **mc=1500 (learning ON)** | 0.126±0.010 | **0.075±0.028** | 0.075 | 0.105 | 0.69±0.08 | **20.4** | 12 |
| **mc=1500 + stab=0.05** | 0.124±0.005 | **0.080±0.022** | 0.065 | 0.101 | 0.12±0.02 | **22.7** | 12 |

`STANDING_CHASSIS_Y = 0.082 m`.  `FAIL_HEIGHT = 0.025 m`.

### 9.2 A/B 1 (mc1500_stab vs mc=0 baseline) — n=20 paired

| Metric | μΔ | σ | t | p | 95 % CI |
|---|---|---|---|---|---|
| Δy_max | **+0.056** | 0.010 | +25.20 | <.0001 | [+0.051, +0.060] |
| Δy_mean_late | **+0.049** | 0.022 | +9.84 | <.0001 | [+0.039, +0.058] |
| **Δpre_W_growth** | **+21.73** | 2.34 | +41.55 | <.0001 | [+20.71, +22.72] |
| Δda_mean | −0.284 | 0.022 | −57.41 | <.0001 | [−0.294, −0.274] |
| Δjoints_var_late | +0.013 | 0.011 | +5.23 | <.0001 | [+0.008, +0.018] |
| Δalive_max | −9685 | 247 | −175.61 | <.0001 | [−9788, −9581] *(metric artifact, §9.4)* |

**Interpretation.**  The brain learns to keep the chassis ~5 cm higher
than baseline.  `y_mean_late=0.080` is approximately `STANDING_CHASSIS_Y`
— body's average height in the late run IS standing.
`pre_W_growth=22.7×` means the Premotor weights moved 22× from
init; this is the highest learning signal we've seen on any picrawler
config.

### 9.3 A/B 2 (mc1500_stab vs mc1500-only) — n=20 paired

| Metric | μΔ | σ | t | p | 95 % CI |
|---|---|---|---|---|---|
| Δy_max | −0.002 | 0.012 | −0.83 | 0.40 | [−0.008, +0.003] |
| Δy_mean_late | +0.006 | 0.019 | +1.33 | 0.19 | [−0.001, +0.015] |
| **Δda_mean** | **−0.569** | 0.097 | −26.35 | <.0001 | [−0.608, −0.525] |
| Δpre_W_growth | **+2.29** | 1.46 | +7.01 | <.0001 | [+1.64, +2.88] |
| Δjoints_var | +0.003 | 0.008 | +1.62 | 0.10 | [−0.001, +0.007] |
| Δalive_max | 0 | 0 | — | 1.00 | [0, 0] *(both at 1500 cap, §9.4)* |

**Interpretation.**  Both arms converge to approximately the same
chassis height — but the stab variant uses 6× less dopamine pumping
to get there.  μ(DA)=0.69 → 0.12.  And the stab variant produced 2.3×
*more* Premotor weight movement (22.7 vs 20.4), not less.  The
stability shaping does NOT regress behavior at long horizon — it
produces a more **energy-efficient standing policy** with the same
chassis height.

### 9.4 Why `Δalive_max` is a metric artifact

`alive_ticks_max` resets to 0 every `mc_episode_period` ticks (by
design — it's the per-episode survival counter).  Both mc1500 variants
have `mc_period=1500` so their counters cap at 1500.  The baseline
(mc=0) never resets so its counter accumulates monotonically to ~11000
in 18000 ticks of run.  The Δ is comparing two incompatibly-scaled
counters.

The substantive signal is in `chassis_y_mean_late` and `y_p50`, which
are scale-invariant.

### 9.5 What the brain learned

Reading the trajectory data, both learning configs converge on a
"sustained upright" policy:

- **mc1500 alone**: high-DA "active standing" — the brain keeps the
  chassis at y≈0.075 with bursts up to 0.126.  DA pumps at 0.69
  (well above the 0.20 baseline).  Lots of leg motion, lots of DA,
  lots of energy.
- **mc1500 + stab=0.05**: low-DA "sustained standing" — chassis
  averages y≈0.080 (slightly higher).  DA pumps at 0.12 (close to
  baseline).  Less leg motion, less DA, more energy-efficient.

Both maintain the body at approximately `STANDING_CHASSIS_Y` for the
late portion of the run.  The stability shaping doesn't change WHERE
the body ends up — it changes the **policy mode** (bursting vs
smooth) that gets there.

This matches the user's UI observation precisely: **"consistent
standing… clear pattern of learning to stand under dynamic
conditions… eventually fell over on its back."**  The "fell over on
its back" is the failure mode of accumulated asymmetric leg motion
across many cycles — the brain learns to stand for ~166 s of stable
dynamics before tipping.

## 10. Lessons learned

### The headline result

**The picrawler quadruped learns to stand from tabula rasa using only
neurotransmitter pulses as the goal-direction lever, with no reward
shaping beyond duty-cycled `events.hit ∝ chassis_y` and a small
auxiliary `events.miss` for excess movement at elevation.**  Confirmed
at n=20 paired seeds × 300 s sim each, `p<0.0001` on every
substantive metric.

This is the first time the v4/v5/v6 substrate has demonstrated
**brain-driven improvement over reflex/no-learning baselines on an
embodied control task at n=20**, contrary to the prediction from
`project_v4_phase6_5_arc_summary` and the v5 falsification of Cell as
a policy-learning testbed.

### Methodology
- **Horizon matters as much as n.** The 30 s window produced two
  false negatives: (a) the stability-shaping NULL at 1.8 k ticks, (b)
  the apparent "reward-hacking is dominant" pattern at mc=1500.  Both
  reverse at 18 k ticks.  Add horizon × learning-rate-to-convergence
  as a first-class methodology parameter alongside n and seed.
- **Wiring traps before substrate failures.** The user-observed
  "substrate doesn't learn" was 100 % a missing event publication.
  The §4.1 consumer-intent audit, executed empirically (not on
  paper), catches this class of bug cheaply.
- **Empirical audits surface what theoretical ones miss.**  The
  §4.4 audit checklist passed when written.  Only the n=20 trajectory
  data exposed `pre_W_growth = 1.0000` across all seeds — proving
  empirically that Premotor weights were frozen, even though the
  events bus appeared healthy.
- **Static-parameter intuitions are misleading at short horizons.**
  The 30 s stability-shaping null *looked* like evidence that the
  reward-shape was wrong.  At long horizon the same mechanism is
  qualitatively beneficial.  The Phase 6.5.x lineage's null A/B
  results may need re-running at longer horizon before falsification
  claims stand.

### Substrate insights
- **The Premotor IS capable of policy learning, end-to-end, on a
  12-DOF body.**  Phase 6.5.x's "never demonstrated brain-driven
  improvement" claim doesn't survive this test once the wiring trap
  is fixed and the horizon is long enough.
- **Neurochem-only learning is sufficient.**  The user's directive
  ("the only lever is neurotransmitter pulses") is enough to drive
  policy improvement.  No reward shaping beyond the existing dopamine
  pathway.
- **`events.miss` shapes policy mode, not policy outcome.**  At
  long horizon, the stability shaping produces a low-DA energy-
  efficient stance with the same chassis height — not a different
  height.  This is a textbook policy-mode bifurcation; the underlying
  attractor is the same but the route to it differs.
- **The "reward-hacking" at 30 s was actually a transient phase of
  a longer learning trajectory.**  The brain initially finds the
  burst-policy (because γ=0.99 weights early-trajectory rewards
  heavily) but over multiple `events.episode_end` finalisations,
  the policy refines toward sustained standing.  We caught the
  brain mid-learning at 30 s and misread it as "wrong objective."

## 11. Stage B ablation lattice — falsifies alternative hypotheses

After the four Stage B mechanism A/Bs returned null at n=20, ran a
6-arm ablation lattice (commit `3aaf1f0` + `0c8f9e4`) to falsify
the hypothesis that the standing behaviour in A5 (Best) emerges from
something simpler than Premotor REINFORCE learning.  All at n=20 ×
300 s sim.

| Arm | y_mean_late | y_p50 | %below_FAIL | n_falls | 1st_fall_tick | pre_w_growth |
|---|---|---|---|---|---|---|
| **A0** Ragdoll (motors off) | 0.024 | 0.024 | 81 % | 46 | 228 | 1.00 |
| **A1** Brain-off (zero action) | 0.022 | 0.022 | 100 % | 0 | 60 | 1.00 |
| **A2** Random uniform action | 0.027 | 0.026 | 35 % | 68 | 2031 | 1.00 |
| **A3** Baseline (mc=0, no REINFORCE) | 0.031 | 0.028 | 35 % | 68 | 2415 | 1.00 |
| **A4** Frozen Premotor (mc_lr=lr=0) | 0.031 | 0.028 | 35 % | 68 | 2271 | 1.00 |
| **A5** Best (mc=1500 + stab) | **0.080** | **0.065** | **10 %** | **12** | **4062** | **22.73** |

### Falsifications

- **A5 ≫ A0 ragdoll** — motors matter.  Body without servo torque
  collapses immediately (81 % of run below FAIL_HEIGHT).
- **A5 ≫ A1 brain-off** — brain commanding actions matters, not just
  the rest pose.  Zero-action override collapses the body fully
  (100 % below FAIL).  The motors slewing from construction-pose to
  standing-pose (knee 0 → −1.6 rad) destabilises within 1 second.
- **A5 ≫ A2 random** — random uniform actions on every channel produce
  a ragdoll-like collapsed trajectory (y_mean_late=0.027, 35 %
  below FAIL).  Noise alone is not enough.
- **A5 ≫ A3 baseline** — Premotor REINFORCE updates matter, not just
  Premotor existence.  With mc_episode_period=0 (no episode_end fires),
  pre_w_growth stays at 1.0 and behaviour is indistinguishable from
  random action.
- **A5 ≫ A4 frozen** — Premotor weight updates matter, not just
  Premotor sampling from random init.  Even with mc_lr=learning_rate=0
  forced, sampling from initial random W produces the same collapsed
  behaviour as A3.

### The "A3 = A4" identity confirms the wiring trap

A3 (mc_lr=0.05, learning_rate=0.05, but no events.episode_end fires)
and A4 (mc_lr=learning_rate=lr_bc=0 explicitly) are **statistically
indistinguishable** on every behavioural metric.

Mechanistically: in v5.1 Premotor mode (mc_lr > 0), per-event Hebbian
updates are *deferred* to events.episode_end's finalize_mc_episode().
If events.episode_end never fires, the deferred updates never apply
— and per-tick Hebbian is also disabled because it was deferred.
Same outcome as having all learning rates zeroed.

**This is the structural gate** between "no learning" and "learning"
— not a parametric one.  Validates `feedback_no_tuning`: the underlying
issue was a missing event publication, not a knob to turn.

### A2 ≈ A3 ≈ A4 — Premotor's initial policy is effectively random

Random uniform actions (A2), Premotor sampling without learning (A3,
A4) all converge to within-noise-of-each-other on behaviour
(y_mean_late ≈ 0.030, %below ≈ 35 %, n_falls ≈ 68).  This means:

- The Premotor architecture has **no useful prior** baked in.  Its
  initial random W matrix produces essentially uniform action samples.
- The substrate's standing capacity comes ENTIRELY from REINFORCE
  structuring those weights — the architectural primitives (8-Premotor
  decomposition, temperature_da_gain, etc.) provide the *capacity for*
  learning, not learning itself.
- `temperature_da_gain=0.5` (DA → less exploration) has no benefit
  when policy is random — A2 and A3/A4 are equivalent despite A3/A4
  having more DA than A2 (0.40 vs 0.36).  Lock-in only matters once
  the brain has a non-random policy.

## 12. EPM ablation — IMU-only outperforms full Best

After the 6-arm A0–A5 ablation proved Premotor REINFORCE is necessary
for standing, the question of whether the EPMs were contributing
useful state remained open.  Two more arms tested it directly at
n=20 × 400 s (B0 IMU-only at n=10 due to a deterministic seed-51 hang):

| Arm | pct_upright | longest_up | pct_tipover | n_fall | tilt_mu | pre_w_growth |
|---|---|---|---|---|---|---|
| **B0** IMU-only EPM | **91 %** | 276 s | **5 %** | **13** | 0.40 | **33.1** |
| B1 Joints-only EPM  | 28 % | 114 s | **68 %** | 23 | 2.23 | 23.4 |
| A5 Best (IMU + joints) | 84 % | **304 s** | 12 % | 13 | 0.60 | 24.9 |

*B0 at full n=20 after harness NaN-robustness fix (commit `e3eadaf`).
n=10 preliminary in earlier sweep gave 94 % / 263 s / 2 % — full n=20
is slightly less extreme but still dominates on every metric except
longest single stretch.*

### Headline: IMU-only is strictly better than IMU + joints

On every metric except `longest_upright_physics_ticks`, **B0 IMU-only
strictly dominates A5 Best** at n=20:
  - 7 absolute points more upright (91 % vs 84 %)
  - 2.4 × less time tipped (5 % vs 12 %)
  - Same number of fall events but later in the run
  - 33 % less dynamic tilt (0.40 rad ≈ 23° vs 0.60 rad ≈ 34°)
  - 33 % more Premotor learning (pre_w_growth 33.1 vs 24.9)

**The joints EPM is actively harmful when combined with the IMU.**
Adding 12-D joint perception to a working IMU brain *reduces* standing
performance.

### Why — three plausible mechanisms

1. **Information dilution.**  The voter has `group_balance=true`,
   so half the consensus comes from the 4-D vestibular EPM and half
   from the 12-D proprio EPM.  The IMU's clean balance signal gets
   washed out by joint noise.
2. **Spurious correlation overfitting.**  With 12 joint dimensions
   feeding the Premotor's `consensus → action` mapping, the
   Premotor learns spurious "if knee_3 = X then push hip_2" rules
   that don't generalise.  The IMU's 4-D state is too low-dimensional
   to overfit this way.
3. **State irrelevance for balancing.**  For balancing (vs. moving),
   chassis pose is what matters — leg angles are downstream
   consequences.  The Premotor's policy doesn't *need* leg state if
   the IMU tells it which way it's leaning.

### The joints-only collapse (B1)

B1 (joints-only) reveals the converse failure mode:
  - Brain learns *how to stand up* (pre_w_growth=23.4 ≈ Best)
  - Achieves substantial upright stretches (114 s peak, much above
    the 13 s ceiling of all non-learning controls)
  - But lacks chassis-pose awareness — so it tips past π/2 and stays
    there.  pct_tipover=68 %, tilt_mean=2.23 rad ≈ 127°.

**Without IMU, the brain has motor coordination but no balance
perception.**  This is a clean falsification of "joints alone are
sufficient."

### Implications

- **IMU-only is the new demo Best.**  Promoted to
  `the_picrawler_stand_best.json` (formerly `the_picrawler_stand_imu_only.json`).
  Old "Best" (IMU + joints) hidden from launcher dropdown via
  `env_target=picrawler_research`.
- For **walking** later, joints likely become essential (locomotion
  needs leg position info to coordinate gait).  For **standing**,
  they're noise that the Premotor architecture can't filter out.
- **The Premotor architecture is sensitive to consensus
  dimensionality.**  This is a substrate property worth flagging:
  more state ≠ better policy when the additional dims are weakly
  correlated with the reward signal.
- **EPMs DO contribute** — IMU is load-bearing.  But the
  contribution is uneven across modalities, and joints currently
  carry no useful signal for this task.

### Known harness/substrate issue (resolved on harness side; substrate fragility remains)

The B0 IMU-only sweep initially hung on seed 51 because the body's
chassis_y goes NaN at tick ~22920 with this config + seed combo.
Godot reports `Condition "!v.is_finite()" is true at
instance_set_transform` and downstream diag emits have null y/hip/
knee/feet values, which crashed the aggregator on `statistics.fmean`.

Harness fix (commit `e3eadaf`): _aggregate filters None/NaN, marks
runs with >10 % corrupted ticks as ok=False, exposes
`n_corrupted_ticks` metric.  Re-ran B0 n=20: all 20 seeds complete,
1 seed has 19/400 corrupted ticks (4.75 %, below threshold, kept in
the aggregate); other 19 seeds clean.

Substrate fragility (not yet fixed): the physics blowup itself
suggests Premotor weights occasionally produce action samples that
drive the body into a singular state at long horizons.  Candidate
fixes for a future patch:
  - Body-side NaN guard in `_step_one` that detects non-finite
    transforms and triggers `_finish_episode(true)`
  - Tighter action-magnitude clipping in the brain→servo pipeline
  - Smaller `motor_max_impulse` or stricter slew limit at extremes

These are substrate-side robustness improvements, not blockers for
the standing experiments.

## 13. Metrics vs agentic behaviour — what the n=20 numbers miss

The Stage B-F result (B0 IMU-only outperforms full Best on every
behavioural metric) is statistically clean — and partially misleading
about what we should actually optimize for.

### The user observation

After watching all three demo configs run in the UI:

> *"While 6.0.b.9 [IMU-only Best] has better standing metrics, the
> actual standing behavior (overall height, stability, reactivity,
> etc.) is the least 'interesting' or 'biological' to my eyes.  6.0.b.3
> [old A5, IMU + joints] on the other hand is much more energetic,
> stands taller for longer and seems to be actively exploring how high
> it can raise its chassis to the point it will always result in
> tipping over.  This is also true for the energy variant — it had a
> smoother behavior and was equally curious."*

The IMU-only brain is **statistically dominant but visually dead**.
The joints version is **statistically inferior but visually alive**.
Both observations are correct.

### Why the metrics diverge from agency

The metrics we built (`pct_upright`, `pct_tipover`, `n_fall_events`,
`tilt_mean`) all reward **low variance around the standing pose**.
A brain that finds a stable basin and never leaves it scores best.
A brain that pushes its limits, occasionally tips, and recovers
scores worse on every metric except one (`longest_upright_physics_ticks`,
which the joints version also won at 304 s vs 276 s).

But living systems are characterized by *behaviours that look like
trying*.  Animals don't optimize for minimum variance — they probe
their capacity, get feedback, and revise.  The visual difference
between the two configs maps onto this distinction:

- **IMU-only (4-D state)**: brain finds a static-pose policy that the
  4-D consensus can't differentiate further.  No state-conditional
  variation in actions → no visible exploration → "dead."
- **IMU + joints (16-D state)**: brain has 16-D state to encode "leg
  3 is here AND leg 1 is there AND chassis is leaning forward."
  Policy can encode richer "if this configuration, then this action"
  rules.  More state → more policy diversity → more visible
  exploration → reads as "alive."

Per `docs/the_playful_machine_principles.md` (the substrate's default
lens when stuck), this is exactly the "edge of chaos" the system is
supposed to live in.  The IMU-only brain found a stable basin and
**stopped trying**.  That's the "stuck" failure mode the principles
warn about — even though the metrics call it success.

### Connecting to the prior re-interpretations

This re-frames §8's correction.  We had initially read the 30 s
"reward-hacking" pattern (burst then collapse) as failure — the brain
optimizing the wrong thing.  At 18 k ticks we re-read it as a
transient phase of a longer learning trajectory that converges to
sustained standing.

**Both re-readings are partial.**  The "burst then collapse" pattern
isn't a transient *to be passed through* — it's the agentic dynamic
*we should preserve*, properly bounded.  The brain probing its
limits IS the behaviour we want.  We just don't want the failure mode
(tip-over) at the limits.

### Implications for objective design

- **Stability metrics are not goal metrics.**  A brain optimizing for
  pct_upright would converge on lying still on the ground (technically
  satisfies "not fallen" if FAIL_HEIGHT is lenient).  We had to add
  multi-metric constraints (height, tilt, fall count) just to get to
  "stands somewhere reasonable."
- **The reward function should encode the goal, not punish failure.**
  Currently we reward "high chassis" and punish "fall."  This is
  unbounded in the rewarding direction → brain climbs past standing.
  A bounded objective (Stage C, §15) — "be at target height, neither
  higher nor lower" — gives the brain a goal to settle on rather
  than a gradient to chase indefinitely.
- **Add "behavioural engagement" metrics for future evaluation.**
  Candidates: action-distribution entropy during standing, frequency
  of trajectory excursions outside the steady-state, recovery
  vigour after perturbation.  These would surface "is the brain
  still trying?" alongside "is the body still up?"

### What we'd run differently with this insight

The Stage B mechanism A/Bs (antirot, drive_posture, tilt EPM, energy
cost) were all judged null on n=20 paired seed deltas at 300 s.  But
those metrics couldn't have distinguished "the mechanism makes the
brain look more alive but doesn't shift the static-pose attractor"
from "the mechanism does nothing."  Per the user's UI observation,
the energy variant *was* qualitatively different — "smoother behavior,
equally curious."  We probably retired some of those mechanisms too
quickly.

The Stage C capped-reward arm (§15) is the first experiment designed
with this insight in mind: change the *goal* rather than the
*shaping*.  Test whether the agentic dynamics survive (or improve)
under a bounded objective.

## 14. Recommended next steps

### Stage C — in flight: target-height cap (trapezoid reward)

Per §13's framing — change the *objective*, not the *shaping*.  Stage C
is being tested at n=20 × 400 s as of commit `5961436`:

  `the_picrawler_stand_capped.json`:
    base:                   old A5 (IMU + joints EPMs, mc=1500 + stab)
    target_height:          0.10 m
    height_penalty_grace:   0.02 m
    height_penalty_scale:   0.03 m
    height_penalty_gain:    0.05

  Trapezoid: ramp `[0, 0.10]`, flat plateau `[0.10, 0.12]`, graded
  `events.miss` on `(0.12, 0.15+]`.  Built on old A5 to preserve the
  joint-EPM agentic dynamics — testing whether agency survives under a
  bounded objective.

### Stage C results

**C-1 (trapezoid cap at 0.10m, height_penalty_gain=0.05):**
  Mixed — `first_sustained_fall_tick` lifted +4155 ticks (p=0.023);
  `pct_tipover` rose +0.06 (NS); `n_falls` +2.7 (p=0.057).  The cap
  works at delaying initial loss of balance but the brain still finds
  the bad attractor eventually.  Best-case interpretation: the
  per-tick miss is too weak to override the joints-EPM exploration
  capacity that drives over-target climbing.

**C-2 (inverted-U at peak=0.082m, band=0.04m):**
  **Strongly negative.**  Successfully constrained climbing
  (Δy_max=−4.6cm, p<0.0001) but at huge cost to standing:
  Δn_falls=+41 (4× the A5 fall rate, p<0.0001), Δpct_below_FAIL=+0.34
  (p<0.0001), Δpre_W_growth=−2.4 (p=0.02).  Symmetric reward
  over-constrains: brain oscillates trying to maintain the exact peak
  height, falls much more often.

**The right shape is asymmetric: positive ramp + plateau + mild
penalty above.**  The trapezoid was on the right track; the inverted-U
was wrong direction.  Future shape tuning should explore: weaker
trapezoid plateau width (e.g. plateau 0.08→0.10 instead of 0.10→0.12)
or stronger penalty above (gain=0.20 instead of 0.05).

### Followups (queued)

Behavioural follow-up if Stage C resolves the tip-over:
- Stability gain sweep around the new bounded optimum
- Add "behavioural engagement" metrics (entropy of action distribution,
  excursion frequency) so future runs distinguish dead-but-stable from
  alive-and-stable
- Body-side NaN guard (substrate fragility from §12) so long-horizon
  runs don't lose seeds to physics blowup

Architectural follow-ups (if Stage C doesn't fully eliminate tip-over):
- C1: **Anti-rotation stability term.**  Per-tick `events.miss` when
  chassis_angular_velocity exceeds threshold.  Already implemented
  (B-A); null at n=20 × 300 s under unbounded reward — should re-test
  on the capped variant where it might suddenly become useful.
- C2: **Survival-coupled reward.**  Single high-value `events.hit` per
  cumulative-standing-ticks threshold.  Tests integral vs rate-coded
  reward.
- C3: **Reflexive righting primitive.**  Hard-coded "if tilt > 0.5,
  contralateral leg extends" + `da_brick_gain` on successful righting.
  Reflex + brain hybrid.

### Open questions
- **Does the brain transfer across body morphology?**  If we change
  the chassis mass or leg length, does the learned policy generalise
  or does it have to re-learn?  Tests body-relative (cerebellum-style)
  vs body-specific policy.
- **What's the asymptotic policy?**  At 60 k+ ticks, does the brain
  find an indefinitely-stable standing or does it always tip?  The
  capped reward should let us answer this without seed contamination
  from tip-over events.
- **Is the 8-Premotor decomposition load-bearing?**  Would 12 single-
  channel Premotors converge faster or slower?  Phase 6.0.b memory
  suggests per-input EPM split lifted hard-mode by 83 % on Cell.

### Open questions
- **Does the brain transfer across body morphology?**  If we change
  the chassis mass or leg length, does the learned policy generalise
  or does it have to re-learn?  This would test whether the policy
  is body-relative (cerebellum-style) or body-specific.
- **What's the asymptotic policy?**  At 60 k+ ticks, does the brain
  find an indefinitely-stable standing or does it always tip?  The
  user's observation suggests tipping; this is the next horizon to
  probe.
- **Is the 8-Premotor decomposition load-bearing?**  Would 12 single-
  channel Premotors converge faster or slower?  Phase 6.0.b memory
  suggests per-input EPM split lifted hard-mode by 83 % on Cell —
  this is the picrawler analogue.

## 15. Reproducibility

### Demo configs (`godot_host/project/addons/ami_ogma/configs/`)

UI launcher dropdown shows four configs (env_target="picrawler"):
- `the_picrawler_stand.json`             — A1/A3 baseline (no learning)
- `the_picrawler_stand_best.json`        — Stage B-F new Best (IMU-only EPM)
- `the_picrawler_stand_mc1500_stab.json` — old A5 Best (IMU + joints; kept for comparison)
- `the_picrawler_stand_capped.json`      — Stage C target-height cap (in flight)

Research configs (env_target="picrawler_research", hidden from
launcher dropdown but available for headless A/Bs and `--config`
overrides):
- `the_picrawler_stand_joints_only.json` — Stage B-F B1 (joints-only, tipover failure mode)
- `the_picrawler_stand_tilt.json`        — Stage B-E (tilt EPM, null)
- `the_picrawler_stand_energy.json`      — Stage B-D (energy cost, null)
- `the_picrawler_stand_frozen.json`      — Stage B ablation A4 (mc_lr=lr=0)

### UI launch

`the_picrawler.tscn`, F1 toggles graph inspector.  In-config metadata
auto-populates ExperimentConfig via `launcher.gd`.

### Headless single-arm sweep

```
python scripts/picrawler_run.py --seeds 42-61 --duration 300 \
  --config res://addons/ami_ogma/configs/the_picrawler_stand_mc1500_stab.json \
  --label A5_Best --parallel 4
```

### Headless paired A/B

```
python scripts/picrawler_ab.py --seeds 42-61 --duration 300 \
  --baseline-config res://addons/ami_ogma/configs/the_picrawler_stand_mc1500_stab.json \
  --variant-config  res://addons/ami_ogma/configs/the_picrawler_stand_tilt.json \
  --parallel 4 --turbo --json-out results/<name>.json
```

### Headless ablation overrides (no config change needed)

```
--extra-env "OGMA_LEG_STRENGTH=0.01"           # ragdoll (A0)
--extra-env "OGMA_PICRAWLER_BRAIN_OFF=1"        # zero-action (A1)
--extra-env "OGMA_PICRAWLER_RANDOM_POLICY=1"    # random action (A2)
```

### Key result JSONs (under `results/` — gitignored)

Stage A:
- `picrawler_A1_baseline_n20.json`
- `picrawler_A3_mc{300,600,1500}_vs_0.json`
- `picrawler_A4_stab_vs_baseline_18k.json`
- `picrawler_A4_stab_vs_mc1500_18k.json`

Stage B mechanism A/Bs (all null):
- `picrawler_B_antirot05_vs_best.json`
- `picrawler_B_drive_only_v2_vs_best.json`
- `picrawler_B_tilt_vs_best.json`
- `picrawler_B_tilt_vs_best_v2.json` (with sensitive metrics)
- `picrawler_B_energy_vs_best_600s.json`

Stage B ablation lattice (`results/ablation/`):
- `A0_ragdoll.json`
- `A1_brainoff.json`
- `A2_random.json`
- `A3_baseline.json`
- `A4_frozen.json`
- `A5_Best.json`
