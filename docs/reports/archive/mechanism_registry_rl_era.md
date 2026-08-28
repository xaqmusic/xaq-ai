> **ARCHIVE — ported from the pre-split `ami-ogma` repo, 2026-07-25.** ⚠️ **This documents the
> reward-shaped RL era of the picrawler**, which [`../../the-picrawler-detour.md`](../../the-picrawler-detour.md)
> disowns as the cautionary origin story. **Its individual mechanism verdicts do NOT transfer** to
> the current reward-free active-inference stack — different substrate, different objective,
> different baseline. It is kept as an honest record and because its **failure shapes and
> measurement lessons are permanently valuable** (those are distilled into
> [`../picrawler_lever_ledger.md`](../picrawler_lever_ledger.md) §7 and
> [`../../../CLAUDE.md`](../../../CLAUDE.md) §3.2). For the **current** verdicts, read the ledger.
> `ami_ogma`/`ogma`/`AMI-Ogma` == xaq.

# AMI-Ogma Mechanism Registry

**Purpose**: single canonical ledger of every brain-substrate or reward-shape
mechanism we've shipped, the outcome at the most rigorous power we measured
it, and the mechanistic reason for the result.  Before proposing a new
mechanism, grep this doc.  When archived configs come up, this doc is the
annotation.

**Status legend**:
- `BASELINE` — kept in the picrawler baseline stack, active in current configs
- `WORKING` — measurable positive effect, kept but not in default baseline
- `PARTIAL` — measurable effect on secondary metrics, null/regression on primary (max_distance)
- `NULL` — no measurable effect at adequate power
- `REGRESSION` — measurable negative effect at adequate power
- `TAUTOLOGY` — variant was bit-identical to baseline (mechanism already on by default)
- `DEAD_CODE` — param had no effect because code path wasn't live in current config
- `ABLATED` — actively removed because of negative consequences
- `IN_FLIGHT` — currently being tested
- `DEFERRED` — built but never tested at adequate power

**Legacy primary metric**: `max_distance_from_origin` over 25-min sim time,
n=5 paired seeds at minimum (Stage 2 of fast-fail protocol).  Phase 8 showed
this metric is not sufficient as a north star: it can reward dead drift,
circling, or random-walk amplitude while underrating closed-loop adaptive
behaviour.  New picrawler mechanism proposals must pre-register both the
legacy navigation metric and an aliveness/adaptation metric (see
`docs/operational/aliveness_metric_protocol.md`).

---

## Section 1 — Currently in the picrawler baseline stack (ACTIVE)

These compose into every current picrawler experiment config and the v6
frozen baseline.

### Section 1.A — Substrate-level wins (substantiated by data, 2026-06-02)

These are the three "wins" that hold up under the post-confound re-tests. They constitute the substrate-claim foundation going into the walking-paradigm work.

**Win 1 — Dynamic standing without programmed physics.** The same brain wiring + reward shape learns dynamic balance from random init across all 5 seeds tested. No hardcoded standing pose, no PID setpoint, no scripted controller. Baseline_noCruse run on the corrected harness: chassis_y_late = 0.095 ± 0.002 (target = 0.085 — body learns to stand SLIGHTLY TALLER than asked, emergent overshoot), longest_upright = 33,648 ticks (9.3 min continuous, seed 42 hit 14.5 min), pct_below_fail_height = 1.7%. preW_growth = 30.58× confirms convergence from random init. Artifact: `results/s2_A_cruse_v2_retest_advance1200/`.

**Win 2 — Same brain solves 5+ motor variants in minutes.** Brain wiring is invariant; the motor stack is the variable. Confirmed working across: discrete PD (baseline), asymmetric KNEE_RANGE_EXTEND/BEND, motor_authority_scale (torque cap), motor_damping_factor (PD Kd multiplier), motor_freeplay_rad (mechanical slop), Bernoulli-impulse actuation (bri T9). Each variant verified to learn standing within the 20-min sim window with preW_growth ≥ 25×. Brain treats motor variation as just another perceptual feature. **Strong evidence the brain is body-agnostic in the range tested.** Next natural step: different morphologies (2-leg, 6-leg, different segment lengths) to test true cross-morphology generalization.

**Win 3 — Radial-outward drift (partial, NOT goal-orientation).** Bodies accumulate outward drift in baseline_noCruse + stage 2 walking reward (max_dist 1.4-4.2m per seed). Per-seed drift direction is non-random within a seed but IS random across seeds. NO clean evidence the brain learns to target a SPECIFIC pyramid. The §3q UI demo where the body "approached a pyramid" was random direction during pre-convergence high-entropy sampling, not learned target-seeking. Goal-orientation remains UNPROVEN; needs walk_to_target reward + target-switching test (Phase 5 of walking_paradigm_redesign).



| Mechanism | Phase | Status | Primary effect | Mechanistic reason | Files |
|---|---|---|---|---|---|
| 12-Premotor per-servo topology | v6.18 | BASELINE | +92% over flat single-voter | atomic lift/press kinematically asymmetric; intermediate stance vocabulary essential | `..._stand_target_per_servo.json` |
| LR-symmetric weight sharing | v6 Stage B | BASELINE | −69% falls, +152% continuous stand, +32% walking dist | LR pairs share W matrix; per-Premotor SAME weights × MIRRORED inputs = mirrored outputs | `Premotor::lr_symmetric_mode_` |
| MC actor-critic (mc_reinforce=true) | v5.1 | BASELINE | required for symmetry-breaking | `(indicator − p_i) × latent` gradient; without it, Hebbian path stays uniform | `Premotor.cpp finalize_mc_episode` |
| advantage_normalization=true | v5.1 | BASELINE | bounds Hebbian growth | running-mean/std subtraction prevents weight divergence under sustained reward | `Premotor.cpp` |
| HIP1_SPLAY_OUT_SIGN | v6.18 | BASELINE | enables coordinated forward motion | per-leg sign-convention for hip1 yaw; brain learns through it via REINFORCE | `picrawler_body.gd:462` |
| Adaptive standing reward (trapezoid + tilt_norm) | v6.0.b.11 / 6.16 | BASELINE | enables tabula-rasa standing | reward saturates at target_height; tilt multiplier requires upright | `picrawler_body.gd` reward block |
| Perceptual CPG (4-channel) | 7.5 | BASELINE | rhythm signal at 6-8% trust each | CPG publishes `[cos(φ), sin(φ)]` per leg; brain consumes via 4 rhythm EPMs | `CPGOscillator.cpp` perceptual mode |
| 4 rhythm EPMs | 7.5 | BASELINE | discriminative phase clustering | per-leg phase clusters as additional voter inputs | configs `epm_rhythm_<leg>` |
| Radial-compass sensor | 7.5.R+ | BASELINE | 19% voter trust, egocentric direction | body publishes 2-D body-frame outward unit vector; new EPM clusters | `picrawler_body.gd:radial_compass` |
| Multiplicative reward gating | 7.5.R | BASELINE | 47% of cumulative reward becomes gated bonus | `chassis_y_norm × v_radial × gain` only fires when upright AND moving outward | `picrawler_body.gd:gated_walk_bonus_rate` |
| Per-source reward attribution telemetry | 7.5+ | BASELINE | diagnostic only | tracks per-tick added rate from {standing, walking, gated} | `picrawler_body.gd:_hit_delta_*` |
| Feet-Y proprio publishing | 7.9 | BASELINE | enables touchdown detection | per-leg foot Y position published as ProprioToken | `picrawler_body.gd:feet_y_arr` |

---

## Section 2 — Working mechanisms NOT in default baseline (opt-in via config)

| Mechanism | Phase | Status | Primary effect | Notes | Files / configs |
|---|---|---|---|---|---|
| **CruseCoordinator v2–v5.1 (full Phase 7.13 stack)** | **7.13** | **WORKING (stability) / NULL (navigation)** | **Two Stage 3 n=10 tests vs anneal both nulled on max_distance.  Stage 3 consolidation: path_length +0.67σ (8/10+), end_distance +0.48σ (8/10+) trend positive but fail formal gate.  Stability gains (~2× fewer falls) survive in some variants.** | Stage 2.C n=5 lift (+1.97σ vs v1) didn't survive Stage 3.  See `docs/findings/phase7_13_findings.md` for full variant ladder and the v5.1≡v4.2 finding. | `CruseCoordinator.{hpp,cpp}` v5.1 / `..._trot_cruse_v5.json` |
| Predictive value head | 7.8 | WORKING (stability) | +1.40σ y_late, −0.60σ falls at n=3; null on max_dist | per-(intent, latent) reward prediction added to softmax logits | `Premotor::value_head_gain` / `..._trot_value_head.json` |
| SynergyTimer (touchdown-driven bias) | 7.9 | WORKING (composition) | C−B in 4-arm: +0.58σ on max_dist (rescues anneal) | per-leg phase from touchdowns; reward-gated Hebbian bias table | `SynergyTimer.cpp` / `..._trot_synergy.json` |
| Entropy anneal | 7.10 | WORKING (standing) | "fast/responsive standing" UI-confirmed, falls halved | when entropy_ema high, softmax T multiplied down → commit | `Premotor::entropy_anneal_gain` / `..._trot_anneal.json` |
| Phase contrast on gated bonus | 7.10b | PARTIAL | rescues anneal (+0.58σ in combo); −0.38σ alone | gated bonus × (1 − gain + gain × inter_diag_contrast) | `picrawler_body.gd:phase_contrast_gain` / `gated_trot.json` curr. |
| Lowstance pose plateau | 7.10.s | NULL on primary | UI-observed diagonal symmetry; −0.30σ on max_dist | `target_height=0.075` + no height penalty → reward plateau | `gated_trot_lowstance.json` curriculum |

---

## Section 3 — Tested null / regression mechanisms (ARCHIVED but documented)

### 3a. Phase 7.6 substrate sweeps (all closed null/regress at n=5 × 25min)

| Mechanism | Phase | Status | Primary effect | Reason for null |
|---|---|---|---|---|
| Eligibility λ sweep (0.5, 0.95) | 7.6.E | DEAD_CODE | bit-identical to λ=0 | `apply_reward` bypassed under MC mode (`mc_lr > 0`); eligibility-trace path is dead code in picrawler config |
| mc_gamma sweep (0.995, 0.999) | 7.6.G | REGRESSION | γ=0.999: −1.76σ path_length, −4.92σ y_late, +2.31σ falls | longer credit window dilutes credit beyond burst duration; γ=0.99 was already optimal |
| Voter `priority_group=spatial` | 7.6.V | TAUTOLOGY | bit-identical to baseline | priority_group is tiebreaker; trust shares not floor-clamped, so priority irrelevant |
| Voter `group_balance=false` | 7.6.V | NULL | +1.89σ path, −0.30σ max_dist | dynamic trust evolution produced more wander, no directional lift |
| Diagonal-pair sub-voters | 7.6.D | NULL | −0.32σ max_dist, +0.63σ path_len | rhythm channels got dedicated trust per diagonal but action-space remained per-Premotor independent |
| advantage_normalization=true (variant) | 7.6.N | TAUTOLOGY | bit-identical to baseline | already enabled in v6 frozen baseline; variant was redundant |
| mc_reinforce=true (variant) | 7.6.R | TAUTOLOGY | bit-identical to baseline | already enabled in v6 frozen baseline; variant was redundant |

### 3b. Phase 7.7 — Per-leg reward decomposition

| Mechanism | Phase | Status | Primary effect | Reason for null |
|---|---|---|---|---|
| Per-leg `events.hit_leg_<leg>` + `aligned_event_name` | 7.7 | REGRESSION | gain=0.50: −0.27σ max_dist, −16.5σ da_mean; gain=0.25: −0.55σ; gain=0.10: −0.35σ | per-leg credit rewards "be planted while body moves outward" → brain over-commits to stance, dampens exploration that drives gait emergence |

### 3c. Phase 7.3 — Hebbian co-activation

| Mechanism | Phase | Status | Primary effect | Reason for null |
|---|---|---|---|---|
| CoactivationMatrix v1 (gain sweep 0.1, 0.5, 1.0) | 7.3 | REGRESSION | all gains collapsed to ~0.1m max_dist, 0 falls | cross-Premotor coupling reinforced reward-co-firings indiscriminately → standing attractor lock |
| CoactivationMatrix v2 (Playful-Machine compliant: surprise-gated + boredom-decay) | 7.3 | REGRESSION | bit-identical to v1 distance | mechanism design corrected per principles, but per-Premotor REINFORCE Hebbian alone Goodharts on dense pose-based reward regardless of update rule |

### 3d. Phase 7.4 — Active inference

| Mechanism | Phase | Status | Primary effect | Reason for null |
|---|---|---|---|---|
| Premotor `epistemic_gain=1.0` + `output_noise_amplitude=0.4` | 7.4 | NULL/REGRESSION | −0.39σ max_dist, +1.11σ path (wandering); +31% falls | adds entropy to action selection without direction; brain wobbles + meanders, not goal-directed |

### 3e. Phase 7.2-EPM — Hierarchical perception

| Mechanism | Phase | Status | Primary effect | Reason for null |
|---|---|---|---|---|
| Per-joint EPM stack (12 EPMs + 4 sub-voters + JointSensorimotorBridge replacing monolithic joints EPM) | 7.2-EPM | REGRESSION (60-min) | tied at 25 min (Stage 2/3); −69% path_length at 60 min (n=8, t=−7.73) | stability without exploration; hierarchy damped inter-channel motor noise that drives translation discovery |

### 3f. Phase 7.x — Motor CPG (ablated)

| Mechanism | Phase | Status | Primary effect | Reason for ablation |
|---|---|---|---|---|
| CPGOscillator motor-side (sinusoidal accel injection + standing-bias DC) | 7.x | ABLATED | F1_inert (CPG amps zeroed) won 4-arm ablation | open-loop oscillator + dense standing reward → standing-bias suppressed Premotor exploration; gait could not emerge under hard motor override |

### 3g. Phase 7.11 — Structural inter-leg prior (CruseCoordinator, **weakened slice — superseded by 7.13**)

| Mechanism | Phase | Status | Primary effect | Reason for null |
|---|---|---|---|---|
| CruseCoordinator v1 (Rule 1 only, adaptive magnitude, logit-bias authority) | 7.11 | NULL | F−A: −0.47σ max_dist (2/5 pos), +1.11σ path_len, −0.64σ upright_t | this implementation was **1/6 of Walknet** — only Rule 1 (no-swing-overlap) — with `adaptive_magnitude` that faded the bias whenever gross compliance held, and softmax-logit authority that the brain's W-weights could override.  Rule 1 alone is purely PREVENTATIVE (don't lift when anterior swings); without Rule 2 (release/constructive lift), rear legs only ever got the stay-in-stance push.  Empirically observed in launcher: rear-leg stiffness, joints not exercising full range within first 3 min.  **Correction to prior framing**: this null falsifies "weakened 1-of-6 priors at this scale"; it does NOT falsify Walknet itself.  Hector is a published existence proof that the full 6-rule formulation walks.  Faithful re-test completed in Phase 7.13; it improved some stability/path secondaries but did not survive as a navigation win at adequate power. |

### 3h. Phase 7.12 v1 -- Progress-PB reward (Goodharted, fixed in v2)

| Mechanism | Phase | Status | Primary effect | Reason for regression |
|---|---|---|---|---|
| Progress-PB reward, upright gate=chassis_y_norm>0.5, no multiplicative scaling | 7.12 v1 | REGRESSION (Goodhart, fixed in v2) | B-A: -7.56sigma chassis_y_mean_late, -8.28sigma pre_w_growth, -8.50sigma da_mean, +25.2 falls/run | gate at 0.5 chassis_y_norm = 4.3 cm out of 8.5 cm target. Half-collapsed body tilted forward still earned PB-progress reward. Combined with reduced standing_baseline_factor=0.25, the brain found fall-forward exploit: collapse -> distance closes -> fire reward. Strong-negative effect proves substrate is reward-shape-sensitive (rules out "substrate ceiling is reward-deaf"); strong-negative is also more informative than null. v2 raised the gate to 0.9 and multiplied intensity by chassis_y_norm, but the later 7.21 full-stack progress arm still regressed catastrophically. |

### 3i. Phase 7.20-7.21 -- Full locomotion stack + progress reward

| Mechanism | Phase | Status | Primary effect | Reason for regression |
|---|---|---|---|---|
| Full locomotion stack (Cruse v5.1 + tilt80 + level-chassis + closed-loop rhythm + relaxed auto-reset + smooth fallback) | 7.20 | REGRESSION on navigation / WORKING on activity | `da_mean` +5.31sigma, 10/10 sign-consistent; `max_distance_from_origin` -0.96sigma, 9/10 worse | The stack produced much more motion without directional control. Energy increased, falls increased, learning velocity fell, and the added activity did not become travel. See `docs/findings/phase7_20_findings.md`. |
| Progress-PB reward on full stack (tight upright gate + multiplicative scaling) | 7.21 | REGRESSION | `max_distance` -2.00sigma, `total_path_length` -5.07sigma, `chassis_y_mean_late` -3.29sigma, `pre_w_growth` -4.20sigma; `da_mean` +8.78sigma | Degenerate jitter-in-place attractor. The body vibrates harder while covering far less ground; progress reward did not create target-reaching. See `docs/findings/phase7_20_findings.md` final section. |

### 3j. Phase 8 -- Action vocabulary / motor primitive options

| Mechanism | Phase | Status | Primary effect | Reason for null/regression |
|---|---|---|---|---|
| Whole-body posture vocabulary (single 12-channel Premotor over static postures) | 8.A1 | NULL on directed locomotion / WORKING on standing | n=5: `max_distance` +0.31sigma noise, `total_path_length` -14.6sigma, 0 falls | Re-anchored postures can stand, but static per-tick posture selection has a flat reward landscape. The policy stays high-entropy, reshuffles poses, and random-walks instead of forming temporal gait. |
| Temporal gait options (`GaitSelector`) | 8.A2 | NULL on directed locomotion | learned selector max reach 1.85m, directionality 0.05; force-trot/force-bound circle or cancel | Mechanism works and learns mechanically, but no primitive in the library translates. Without one differential outcome, the selector has no advantage signal. This falsifies open-loop action vocabulary as the next route, not the existence of closed-loop locomotion. |

### 3k. Phase-viscosity P1 -- Premotor intent dwell

| Mechanism | Phase | Status | Primary effect | Reason for regression |
|---|---|---|---|---|
| Premotor intent dwell (`intent_dwell_ticks` 4/8, `intent_dwell_break_bias=0.15`) | P1 | REGRESSION on locomotion / WORKING on quiet stance | seed-50 45min distance: dwell8 path 1.46m, max 0.18m, late joint var 0.000; dwell4 path 7.46m, max 0.39m, both far below Cruse v2 baseline path 115.64m, max 4.34m | Mechanism reduced tick noise by preserving held intents, but without phase structure it mostly locked stance. Dwell counters proved the mechanism engaged; movement collapse falsifies blind duration-only dwell as the next route. Keep only as a possible weak support (`break_bias=0.0-0.05`) after phase-aware switching is tested. |

### 3m. Phase-viscosity P2 -- Phase-bin commitment

| Mechanism | Phase | Status | Primary effect | Reason for null/regression |
|---|---|---|---|---|
| Phase-bin commitment (`phase_commit_mode=switch_penalty`, `phase_switch_penalty=0.20`, 8 bins on per-leg CPG phase) | P2 | RETIRED 2026-05-29 as lead | seed-50 distance curriculum: aliveness 0.530 vs baseline 0.469 (+0.061), max_distance 4.80m vs 4.34m. **Paired tilt80**: aliveness 0.329 vs 0.519, **310 falls vs 40**. | Pattern A signature: single-seed marginal lift on the friendly curriculum, hard regression on the stability-test curriculum. Same shape as the entire Phase 7→8 graveyard. Retired before n=10 powering per the unambiguous-emergence bar (`feedback_unambiguous_emergence_bar`). |

### 3n. Gait-cycle reward (2026-05-30/31)

Discrete `events.hit` pulse fired when all 4 feet complete a touchdown cycle within a window AND net signed displacement in the active `walk_reward_mode` direction clears a threshold AND backward excursion stays under a guard. Direction-aware (anti-Goodhart). Tested across 4 variants:

| Variant | Date | Config | Result | Reason for null/regression |
|---|---|---|---|---|
| v1 — strict static thresholds | 2026-05-30 | `min_progress=5mm`, `max_backward=2mm`, `consecutive=1` | NULL on reward firing | 0 good cycles in 45 min; 1536 wobble aborts; bars 3× tighter than actual motion scale. Bit-identical to control (seed=50 determinism). |
| v2 — relaxed static thresholds | 2026-05-30 | `min_progress=2mm`, `max_backward=10mm`, `consecutive=1` | **STRONG REGRESSION**, Pattern E | 17 good cycles **fired**; n_fall_events 21→97 (4.6×), n_tipover_events 14→176 (12×), longest_upright 25.6min→3.8min, max_distance 1.58m→0.77m. Brain learned to **lurch forward** to grab the pulse. ht_mean 0.54→0.08 (distressed system). |
| v3 — adaptive thresholds | 2026-05-30 | EMAs of cycle progress/wobble, K=(0.5, 2.0), warmup=20 | NULL → DECAY | Moderate regression (n_fall 21→56). Trajectory shows two-phase collapse: standing-trap mins 6-19 (chassis_y 0.075, tilt 0.6 rad, weights FROZEN at preW=28.39 for 11 consecutive minutes); break at min 20; post-collapse wandering. 7 good cycles total, ALL in first 20 min. |
| v4 — consecutive-cycle gate (N=2) | 2026-05-31 | adaptive + require 2 consecutive good cycles before pulse | NULL, mechanism falsified | **0 pulses** in 45 min; `consecutive_good_max=1` across 1043 cycle attempts. Of 3 good cycles total, not even 2 were consecutive. **Bit-identical to control on every behavioural metric** (chassis_y, falls, pct_upright, preW all match to 3 decimals). Substrate cannot chain 2 forward-displacing 4-foot cycles. N=3 would also be 0. Decisive falsification of the consecutive-cycle gate as a continuity test. |

**Net 2026-05-31:** gait-cycle reward family **falsified as the next route**. Mechanism mechanically works (events fire, telemetry visible). Behaviour is Pattern E: reward provoked motion without producing forward progress, body found Goodhart-degenerate solutions (lurch, standing-trap, eventual policy collapse). **Predicted by `gait_ignition_theory.md` §7** ("does not add a reward bonus on the standing fixed point — that is the Goodhart path"). Worth keeping that as a record so the next "let's add a reward channel" impulse has a visible cost.

### 3o. level_chassis ungated (2026-05-31 belly-flop bug)

| Mechanism | Phase | Status | Primary effect | Reason for failure |
|---|---|---|---|---|
| `level_chassis_rate=0.05` reward channel WITHOUT `reward_min_height` gate | handtuned_v1 | BUG, FIXED 2026-05-31 | Overnight UI run: `level_chassis` accumulated 20,730 hits (**86% of all reward**), body settled at `chassis_y≈0.035` (belly position) with twitching legs, DA saturated 1.0, preW kept growing (77→113). Lasted 3 sim hours undetected. | Reward-channel interaction bug. Every other height-conditional channel (standing, gated_walk) was gated by `reward_min_height=0.06`; `level_chassis` had no such gate. When chassis crashed below the floor, every channel went dark **except** level_chassis, which kept paying via its `chassis_y/target_height` height factor (linear, never reaches 0 unless y=0). Brain found: lie at y=0.035, keep tilt low → 0.02 hits/tick forever. **Fix** (`picrawler_body.gd:2664-2674`): force `level_height_factor=0` when `chassis_y < reward_min_height`. Same gate as other channels. |

### 3p. Systemic reward dominance audit (2026-05-31)

Built `scripts/audit_reward_attribution.py` against the full trajectory archive (216 sources scanned, 125 with reward attribution data). Output is reproducible — re-run after any reward design change.

**Result: 84% of analyzable historical runs trip the 60% dominance threshold.**

| Dominant channel | n runs flagged | avg cum share | max | Notable runs affected |
|---|---:|---:|---:|---|
| `standing` | **85** | **92%** | 100% | Phase 6/7/8 baselines, `ignition_baseline_cruse_v2` (89%), `phase8_a1/stage2/A_baseline` (90-91% all seeds), `phase8_a1/stage2/B_postures` (91-93%), bucket V2/V3/V4 (89-93%), most picrawler UI runs from May 25-28 |
| `progress` | 10 | 100% | 100% | All 10 seeds of `phase7_21_progress_stack/B_progress` (the §3i/§3h Goodharted progress-PB arm) |
| `level_chassis` | 7 | 77% | 87% | handtuned_v1 family + 2 UI logs (May 30-31). Closed by §3o fix. |
| `walking` | 3 | 79% | 94% | `cruse_sign_sweep/h2p_knp_R2off`, `bucket_v9_r2off` — short-duration fast-wander attractors |

**The only balanced exceptions in the entire archive:**

| Run | Top channel | Share | What made it balanced |
|---|---|---:|---|
| `handtuned_v1_lvlfix` (post-§3o fix) | walking | 36.7% | First clean baseline in weeks |
| `phase7_5_long/seed44` (1-hr) | gated | 45.4% | **Phase 7.5.R multiplicative gated_walk_bonus design** |
| `phase7_21_progress_stack/A_anneal` (10 seeds) | gated | 44-46% | Same Phase 7.5.R design |
| `phase7_5_long/seed44` | gated | 45.4% | Same Phase 7.5.R design |

**The Phase 7.5.R `gated_walk_bonus` multiplicative-gate design is the only mechanism in the entire archive that successfully diversified reward without creating a new dominance attractor.** Subsequent designs (V8 bucket, V10 Cruse, handtuned_v1) drifted *away* from it, reintroducing standing dominance.

**Critical implication for prior Pattern A/E/F nulls:** the substrate A/B comparisons in registry §3a (Phase 7.6), §3i (Phase 7.20-7.21), §3j (Phase 8 action vocab), and §3k (Phase-viscosity P1) were nearly all tested against standing-dominant controls (the controls themselves trip the 60% threshold). When the control is at a Goodhart attractor, "the mechanism didn't lift mean" tells us about the *baseline*, not the mechanism. **Some unknown number of those nulls may actually be valid mechanisms tested against a degenerate fixed point.** We cannot retroactively distinguish without re-running against a balanced baseline.

**91 runs SKIPPED** — pre-date the `reward_cum` dict in body telemetry (mostly May 19-22 runs). Includes `picrawler_B0_baseline_n20` — the canonical "first brain-driven evidence" milestone (`project_v6_picrawler_first_evidence` memory) **cannot be retroactively audited**. Replication under attribution-clean reward is required to confirm the finding.

### 3q. Phase 7.5.R replication test (S1.1, 2026-05-31 evening)

| Run | Config | Curriculum | Result | Interpretation |
|---|---|---|---|---|
| S1.1 | `archive/the_picrawler_stand_target_per_servo_perceptual_cpg_trot.json` | `picrawler_stand_walk_gated_trot.json` | **PASS** | Body standing tall (chassis_y_late=0.096 = 113% of target), balanced reward (standing 59.6%, gated 23.8%, walking 16.5% — all <60% bar), 41.5× pre_w_growth, da_mean=0.389 |

**Headline:** Phase 7.5.R recipe REPLICATES on current substrate (current code, current EPM + Premotor topology, with the §3o level_chassis fix applied). Body achieves standing height for the first time in any recent run.

**Empirically confirms §3p audit thesis:** the "we can't make the body stand" problem of the past 2 weeks was a **reward-design problem**, not a substrate problem. The substrate works. The handtuned_v1 → gait_cycle → bucket lineage moved AWAY from the working Phase 7.5.R baseline, and every iteration since then was tuning on a broken recipe.

**One concerning detail:** standing-channel share crept from historical 42.8% (`phase7_5_long/seed44`) to 59.6% (S1.1) — *just* under the 60% dominance bar. Something between May 24 and today shifted reward dynamics. Worth a follow-up code diff but doesn't block forward progress.

**Impact on prior Pattern A claims:** every §3 mechanism tested between May 24 and May 31 (Cruse v2/v3/v4/v5, P1 dwell, P2 phase-bin, V8/V9/V10 buckets, gait-cycle v1-v4) was evaluated against a *standing-dominant or body-failed* baseline. The Pattern A "tightens variance without lifting mean" interpretation may need revisiting for each. S1.3 (Cruse v2 on the recovered Phase 7.5.R baseline) is the immediate next test.

**Phase 7.5.R is the new substrate baseline going forward.** Memory: [[v6-phase7-5-r-recovered]].

### 3r. B0 first-evidence replication (S1.2, 2026-05-31 evening)

| Run | Config | Curriculum | Result | Interpretation |
|---|---|---|---|---|
| S1.2 | `archive/the_picrawler_stand_target.json` (B0's actual config) | none | **PASS** | chassis_y_late=0.099 (117% of target), chassis_y_max=0.141 (165%), pre_w_growth=60.9× (highest ever recorded), wall time 13.5 min (B0 has only 14 modules vs Phase 7.5.R's 25) |

**Headline:** The [[v6-picrawler-first-evidence]] standing milestone **REPLICATES** on current code. Body stands above target sustained, with the most aggressive weight growth in the archive. The original May 19 n=20 finding (91% upright, LR-symmetric wins) holds.

**But the reward attribution reveals the catch:** B0 fires **100% standing channel** (no other channels active in the config — single-channel design by intent). Brain optimizes the only reward signal available. This is *correctness*, not dominance-bug: the recipe is single-channel by design, so 100% is the maximum possible share.

**This explains why B0 has never generalized beyond standing.** There is no walking, gated, or progress channel to optimize toward. The brain learns standing because standing IS the entire reward landscape — and a multi-channel substrate has been the open problem ever since.

**Implication:** [[v6-picrawler-first-evidence]] is valid as a standing-capability finding. It does not — and cannot — say anything about whether the substrate generalizes to walking. The "brain helps" claim is bounded to the standing domain.

**Memory:** updated [[v6-picrawler-first-evidence]] with this audit annotation.

### 3s. Cruse v2 on Phase 7.5.R baseline — PATTERN A PARTIALLY FALSIFIED (S1.3, 2026-05-31 evening)

| Run | Config | Curriculum | Result |
|---|---|---|---|
| S1.3 | `..._cruse_v2.json` | `picrawler_stand_walk_gated_trot.json` | Mixed: locomotion LIFTED, stability degraded |

**S1.3 vs S1.1 baseline (single seed=50, 45 min each):**

| Dimension | Δ |
|---|---|
| max_distance_from_origin | **+42%** (2.72 → 3.86 m) |
| chassis_y_max | **+14%** (0.127 → 0.145) |
| total_path_length | +10% (119 → 132 m) |
| gated reward share | +5pp (23.8% → 28.7%) |
| pct_tipover | **+308%** (0.5% → 2.0%) |
| pct_high_tilt | **+1085%** (1.5% → 18%) |
| n_tipover_events | **+308%** (13 → 53) |
| longest_upright_physics_ticks | **−52%** (43800 → 21060) |
| auto_reset_count | +186% (22 → 63) |

**Headline:** This is **NOT Pattern A.** Pattern A signature is "tightens variance without lifting mean." S1.3 shows the *opposite*: variance MASSIVELY increased (3× tipovers) AND mean LIFTED (+42% on the goal metric).

**Cruse v2 has real coordination mechanism — invisible against the broken baseline.** Against handtuned_v1 (chassis_y=0.033, body in crouch), Cruse v2 had nothing meaningful to coordinate; the body was already failed. Against Phase 7.5.R (chassis_y=0.096, body standing), Cruse v2 actively engages the gated_walk_bonus channel and produces more locomotion — at the cost of stability.

**The registry-claimed Cruse v2 null (Pattern A signature, 12+ mechanisms tested at n=10+, §3a) was at least partially a baseline-bias artifact.** Re-evaluating other Pattern A mechanisms against the recovered Phase 7.5.R baseline is now a priority (Stage 2).

**Caveat:** single seed. Stage 2.A (n=5 paired-seed confirmation) is required before treating this as a structural lift rather than a seed-50 fluke. Cross-seed determinism + variance check unblocks any structural decision.

**Possible new pattern (named only after n=5 confirmation):** **Pattern G — "trades stability for reach"** — mechanism lifts the goal metric while degrading stability proportionally. Different from Pattern A (constrainers) and Pattern E (activity without aliveness). Engineerable target: damp the coordination authority to reduce instability without losing the lift.

**Memory:** [[v6-s1-3-cruse-v2-lift]].

### 3s.UPDATE — n=5 paired comparison (S2.A, 2026-05-31 late evening) — S1.3 lift DOES NOT REPLICATE

Paired n=5 (seeds 42-46) cruse_v2 vs Phase 7.5.R baseline:

| Metric | mean Δ | t (n=5) | Verdict |
|---|---:|---:|---|
| chassis_y_max | +6.3% | +3.43 | ✓ significant — small peak-height lift |
| chassis_y_mean_late | **−4.8%** | −2.57 | ✓ significant — Cruse v2 LOWERS sustained height |
| pct_high_tilt | **+298%** | **+4.58** | ✓ strongest signal — Cruse v2 reliably destabilizes |
| max_distance_from_origin | +62% nominal | +1.02 | ✗ NOT significant — driven entirely by seed 43 outlier (+9.5 m) |
| n_tipover_events | +199% nominal | +1.39 | ✗ NOT significant — driven by seed 42 (+179) |
| pre_w_growth | +9.4% | +0.78 | wash |
| da_mean | +0.2% | +0.12 | wash |

Per-seed max_distance Δ: −1.57, +9.50, −0.99, +2.07, +0.15 (m). Without seed 43, mean Δ ≈ 0.

**S1.3's "+42% max_distance" was a single-seed outlier within natural variance.** The n=5 paired test is decisive: there is NO structural max_distance lift. What IS structural and significant: Cruse v2 reliably destabilizes the body (pct_high_tilt +298% with t=+4.58) without compensating gain on locomotion.

**Pattern G ("trades stability for reach") is PARTIALLY REJECTED.** The stability cost is real and strong; the reach gain is not. Cruse v2 returns to Pattern A territory — variance up, mean not up — but with the wrinkle that the variance cost is much larger than typical Pattern A signatures (typical Pattern A is variance-tightening, not variance-amplifying without payoff).

**Revised takeaway:** the §3a Cruse v2 Pattern A null **holds even against the recovered Phase 7.5.R baseline**. The dominance-audit thesis (some nulls are baseline-bias artifacts) is still valid in general but does NOT apply to Cruse v2 specifically. S2.C (P2 phase-bin retest) is now the independent test of whether ANY prior null is recoverable.

**Methodological note:** without n=5 paired discipline we would have adopted Cruse v2 + Phase 7.5.R as the new substrate baseline based on S1.3's single-seed result. The paired comparison saved us from building Stage 3 on a chimera. The `feedback_fast_fail_iteration_protocol` 4-stage discipline (Pilot → Signal → Direction → Powered) is precisely calibrated to catch this class of error; S1.3 was Stage 1, S2.A was Stage 2, and Stage 2 caught it.

**Memory:** [[v6-s1-3-cruse-v2-lift]] updated with retraction. Skipping S2.B (damped Cruse v2 — no lift to damp). S2.C and S2.D remain valid independent tests.

### 3t. P2 phase-bin retest on Phase 7.5.R baseline — NULL HOLDS (S2.C, 2026-06-01)

Paired n=5 (seeds 42-46) P2 phase-bin (`cruse_v2_p2_phasebin8_penalty020.json` archive config) vs Phase 7.5.R baseline:

| Metric | mean Δ | t (n=5) | Verdict |
|---|---:|---:|---|
| chassis_y_max | +14.0% | **+4.94** | ✓ significant — body reaches higher peak |
| total_path_length | +10.6% | +2.13 | ✓ marginally significant |
| pct_high_tilt | **+430%** | **+2.29** | ✓ significant — strong destabilization |
| max_distance_from_origin | +13.8% nominal | +0.36 | ✗ NOT significant (σ=2.50m swamps mean) |
| chassis_y_mean_late | −7.8% | −1.84 | marginal LOWER |
| n_fall_events | +253% nominal | +1.21 | ✗ driven by seed 42 outlier (+327) |
| n_tipover_events | +584% nominal | +1.35 | ✗ same seed 42 outlier |
| pre_w_growth, da_mean | wash | | |

Per-seed max_distance Δ: −2.56, −1.13, −0.53, +4.62, +1.63 (m). Multimodal — some seeds worse, some better. Not a clean lift.

**3-way mean comparison (n=5 each):** baseline / +cruse_v2 / +P2:
- max_distance: 2.95 → 4.79 → 3.36 (neither mechanism reliably lifts mean)
- chassis_y_max: 0.127 → 0.135 → 0.145 (both lift peak slightly)
- pct_high_tilt: 4.3% → 17% → **22.6%** (P2 destabilizes MORE than Cruse v2)
- pct_tipover: 0.8% → 2.5% → 7.0%
- n_tipover_events: 22 → 66 → **151**

**P2 destabilizes even more than Cruse v2 with similar (non-significant) max_distance variation.** Both coordination-layer mechanisms exhibit the same null-signature against the clean baseline.

**Implications for the dominance-audit thesis (§3p):**

The audit suggested prior Pattern A nulls were "tested against standing-dominant controls — may be valid mechanisms tested against Goodhart fixed points." Two coordination-layer mechanisms have now been re-tested at n=5 paired against the recovered Phase 7.5.R baseline; **both nulls held**. The thesis is at least **partially refuted** for coordination-layer mechanisms — the nulls are robust, not baseline-bias artifacts.

What this DOES NOT refute:
- Other categories of Pattern A nulls (bucket scheme, action vocabulary, MotorFader) haven't been retested
- The audit's other findings (standing-channel dominance is systemic, Phase 7.5.R is the recoverable baseline, B0 is single-channel by design) are independent of Cruse v2 / P2 outcomes

**The provisional Pattern G ("trades stability for reach") is firmly dead.** Two mechanisms tested at n=5; neither shows a structural reach gain proportional to the stability cost. Pattern A is the correct classification for both — variance UP, mean NOT UP. The wrinkle is that variance is amplified (not constrained) by coordination-layer mechanisms on this substrate, which is distinct from typical Pattern A signatures of constrainers.

**Forward direction:** Stage 2.D (HK destabiliser, gait_ignition §6) launching next as the independent test. After 2.D, Stage 3 (load proprio + LIF actuation + joint compliance bundle) is the next substrate-addition move regardless of S2.D outcome — the multi-channel sensorimotor gap is what motivates Stage 3, not specific coordination-layer claims.

### 3u. HK destabiliser on Cruse v2 + Phase 7.5.R stack — STRUCTURAL FALSIFICATION (S2.D, 2026-06-01)

The gait_ignition_theory §6 named mechanism: wire `HomeokineticExploration` into the Cruse v2 + perceptual-CPG stack as standing-basin destabiliser. Implemented as `archive/` config sibling `the_picrawler_..._cruse_v2_hk.json` (12 Premotors wired to `exploration.directive`, HK module appended with `entropy_collapse_fraction=0.5`).

n=5 paired vs S2.A cruse_v2 (no HK), seeds 42-46:

| Per-seed metric | S2.A cruse_v2 | S2.D cruse_v2 + HK | Δ |
|---|---:|---:|---:|
| seed 42 | y_max=0.140, y_late=0.081, preW=57.4 | identical | 0 |
| seed 43 | y_max=0.132, y_late=0.093, preW=39.2 | identical | 0 |
| seed 44 | y_max=0.135, y_late=0.086, preW=34.5 | identical | 0 |
| seed 45 | y_max=0.127, y_late=0.095, preW=33.7 | identical | 0 |
| seed 46 | y_max=0.140, y_late=0.095, preW=40.6 | identical | 0 |

**Bit-identical across all 5 seeds.** Determinism-preserving same-seed test confirms HK published no events that altered the brain trajectory.

**HK telemetry across all 5 seeds:**
- `hk_active` = **False** at all ticks
- `hk_episodes_armed` = **0** at end (HK never fired an episode)
- `hk_sample_count` = ~162,000 (module loaded and sampling every tick — wiring is correct)
- `hk_long_change_ema` = **0.0** at end (the brain's change-rate baseline collapsed to zero — never accumulated)
- `hk_saturation_streak` = 0 (saturation gate never activated)

**Diagnosis:** the HK gates are tuned for cell-env chemotaxis-stuck dynamics (the v4 phase 6.5 era where HK was developed). On picrawler with the Phase 7.5.R reward stack, brain state is stable and standing — the kind of state HK *should* detect as stuck, but the gate criteria don't read it that way:
- `anomaly_factor` gate (current change-rate < 0.5 × median history) never triggers because the median baseline itself collapses to 0
- `entropy_collapse_fraction=0.5` needs a Premotor's `entropy_ema` to drop below half of its historical peak, AND that peak must exceed `entropy_min_peak=1.0` — neither happened
- `saturation_clamp` gate is off by default (`saturation_clamp=0.0`)

**Per gait_ignition_theory §6 falsifier:** "Fold back to null if hk_active fires but produces only jitter/wander with no bout." We got something **more decisive** — hk_active *never* fires. The §6 prediction is structurally falsified at the trigger layer, not the behavioral layer.

**Three interpretations:**

1. **HK parameters need picrawler-specific tuning.** The trigger thresholds were calibrated for a different env-class. With different thresholds (lower `entropy_min_peak`, lower `anomaly_factor`, enable `saturation_clamp`), HK might fire and produce *something*. Worth one parameter sweep before declaring HK as a mechanism non-functional on picrawler.

2. **HK's trigger model is wrong for picrawler.** The "change-rate falls below typical" gate fits an exploration-when-stuck context with active urgency signals. Picrawler's HomeostaticDrive may not produce the urgency signal HK reads. Needs investigation of `kDriveErrors` topic content under Phase 7.5.R reward.

3. **The standing-basin-as-stuck framing may be wrong for picrawler.** Phase 7.5.R produces a STANDING body that earns gated_walk_bonus when it moves. From the brain's perspective, standing is not "stuck" — it's the active strategy that produces reward. The gait_ignition theory's "fixed point → limit cycle" framing assumes standing is locally optimal but globally suboptimal; on the recovered substrate, standing may actually BE the locally appropriate behavior given current reward.

**Forward direction:** the HK retest (interpretation 1) is the cheapest next move IF the gait_ignition direction stays active. But the broader picture is that all three Stage 2 mechanisms (Cruse v2, P2, HK) failed to lift mean against the recovered baseline — coordination-layer + destabilization-layer additions are exhausted. **Stage 3 (Stage 3.A load proprio + 3.D LIF actuation + 3.E joint compliance) is the next substrate-addition move.** The motor-control / sensorimotor gap is the unexplored architectural axis.

**Memory:** [[v6-stage2-wrapup]].

### 3v. Stage 3 motor-stack shipping (2026-06-01)

The substrate-addition push named at the close of Stage 2 (§3u "Forward direction"). Six knobs / two backends shipped to `picrawler_body.gd`; one falsified-and-reverted (§3w).

**Shipped and validated (kept in default-off, no behavioral change):**

| Knob / mechanism | Commit | Purpose | Default |
|---|---|---|---|
| **3.A** `joint_torque` proprio publishing | `3c3765c` | per-servo torque measurement on the joints proprio channel | always-on; non-default consumer |
| **3.D** `bernoulli_impulse` actuation backend (`actuation_backend=bernoulli_impulse` + `bri_*` params) | `401e4f8` | body-as-integrator: brain emits `u`, body samples Bernoulli spikes, integrates target offset with friction decay | `actuation_backend=discrete` (default = pre-3.D discrete PD) |
| **3.E** `motor_authority_scale` | `372c643` | motor torque cap multiplier (servo-saver torque limit analog) | 1.0 (= unchanged) |
| **3.E++** `motor_damping_factor` | `f9ab634` | motor PD `Kd` multiplier (motor-internal damping tuner) | 1.0 (= unchanged) |
| **3.E++** `motor_freeplay_rad` | `ed2a642` + fix `4370dc7` | mechanical-slop zone: motor disengages (vel=0, max_impulse=0) inside `|error|<rad` | 0.0 (= unchanged) |
| Asymmetric knee range (`KNEE_RANGE_EXTEND/BEND` + widened `KNEE_LIMIT_LOW=-2.50`) | `f9ab634` | brain commands now cover the full physical knee range (was 86° of 194° prior) | symmetric default mapping preserved when not configured |
| G-mode unification through any actuation backend | `bf61bd6` | calibration mode (G key) writes inverse-mapped commands so the same UI slider works under both discrete and bri backends | always-on |

**Quantitative result so far:** bri T9 single-seed sweep (NO CPG + `bri_impulse_per_spike=0.10` + `bri_friction_per_tick=0.030`) identified as best bri configuration; n=5 paired against §3q Phase 7.5.R baseline showed **Pattern A** — variance tightens, no mean lift on `max_distance` (μ stay ≈ 2.95 m), `auto_resets` worse (+44 vs baseline). The bri backend works as a different actuation model but does not by itself unlock locomotion. The remaining knobs (`motor_authority_scale`, `motor_damping_factor`, `motor_freeplay_rad`, asymmetric knee) ship as **infrastructure** for the future G6DOFJoint3D migration (§3w) and aliveness-gated curriculum (§4 diagnostic_calibration); none have been n=5 validated independently yet.

**Operator demo, 2026-06-01.** UI run with Cruse v2 + Phase 7.5.R curriculum, seed 50, 12.5 min:

- **Best continuous stand 9 min 25 s** — new substrate best for any recovered baseline (vs prior 3–5 min).
- `pct_upright = 97.8%`, 8 falls + 6 tipovers across the full run.
- Reward share: **standing 100% / walking 0% / gated 0%** — brain never earned a walking reward.
- During the first ~3 min (pre-convergence, Premotor entropy still high ≈ 1.5), the body **incidentally shuffled forward and reached the first pyramid (~1 m)** before falling, auto-resetting to center, and converging to standing.

**What the demo shows:**

1. The body + Cruse v2 + current motor stack can produce **incidentally translating motion**. Locomotion is *not* blocked by morphology, perception, or motor control.
2. Once Premotor entropy drops and the brain locks onto standing (the rewarded attractor), the shuffle stops. The body returns to origin and stays.
3. This is **Pattern E** ([[feedback_aliveness_over_distance_metric]]) — activity without aliveness, random sampling that incidentally translated. Direction was not learned.
4. **The blocker is the reward landscape**, not the substrate. Confirms §3p dominance audit thesis playing out live: standing reward dominates the gated walk bonus once found.

**Forward direction:** the Stage 3 motor-stack infrastructure is in place; the bottleneck is on the reward / drive side (gait ignition, §4). The next mechanism bet must *push the brain off the standing fixed point* — not add coordination, and not add reward shape *on* the fixed point (which §3n already established produces Pattern E). The remaining Stage 3 knobs become tunable surface area *for the G6DOF migration and ignition-mechanism A/Bs to use*, not standalone mechanism candidates.

**Memory:** [[v6-phase7-5-r-seed50-standing]], [[v6-stage2-wrapup]], [[v6-diagnostic-calibration]].

### 3w. apply_torque passive joint spring — FALSIFIED, REVERTED (2026-06-01)

Stage 3.E++ named-but-deferred companion to `motor_freeplay_rad`: provide a centering force inside the deadband (servo-saver analog) via a per-tick `apply_torque` spring + damper on each HingeJoint3D's two bodies (`τ = −K·(angle−target) − D·ω`).

**Shipped 3f2bcf1 → ed2a642 → f53bcdc; reverted 383a2ef.**

**Why it failed (intrinsic, not parameter tuning):**

- Explicit-Euler integration of `τ = −K·(angle−target) − D·ω` via `apply_torque` is stable only if `K·dt²/I < 1` AND `D·dt/I < 2`.
- Picrawler leg segments: `I ≈ 5e-6 kg·m²` (25 g rod, L ≈ 5 cm).
- At `dt = 1/60 s`: `K_max ≈ 0.018` Nm/rad, `D_max ≈ 0.0006` Nm·s/rad.
- To suspend the chassis (gravity load ≈ 0.05 Nm), K needs to be ≥ 0.5 — **30× the stability limit.**

**Operator G-mode test 2026-06-01** confirmed the prediction: freeplay alone caused hip2-axis limit-cycle bouncing between deadband boundaries; adding damping made bouncing *faster* (sign-flip of ω each tick); large damping caused physics explosion. Clamping the per-tick spring+damping torque to ±0.30 Nm (`SPRING_TORQUE_CAP`) only papered over the symptom — the limit-cycle and the sign-flip persisted. Semi-implicit reformulation `τ_d = −D·ω / (1 + D·dt/I)` would have been stable but produced asymptotic damping that doesn't dissipate visibly. Either way, the K stability gap (30× short) remains unsolvable by re-parameterisation.

**The intrinsic fix (deferred, not lost):** `Generic6DOFJoint3D` exposes native angular spring + damping (`PARAM_ANGULAR_SPRING_STIFFNESS/DAMPING` with `FLAG_ENABLE_ANGULAR_SPRING`). Under the hood: Bullet's `btGeneric6DofSpring2Constraint` — constraint-level LCP-solver integration, unconditionally stable at any K/D. Motor params (`PARAM_ANGULAR_MOTOR_*`) live on the same joint, so `motor_authority_scale` and `motor_freeplay_rad` port cleanly. Migration cost: 12-joint refactor (lock 5 of 6 DOFs to mimic hinge, port motor params, verify bit-identical-modulo-solver at defaults). Scoped as a separate PR.

**Rule of thumb (per [[v6-apply-torque-spring-falsified]]):** when someone (including future-me) proposes "add a passive spring/damper to picrawler joints," the answer is **"yes, but through `Generic6DOFJoint3D`'s native params, not `apply_torque`."**

What was kept from the broader Stage 3.E++ work: `motor_freeplay_rad` (still useful as a slop zone *without* centering force — joint floats in zone), `motor_authority_scale`, `motor_damping_factor`, asymmetric knee range, bri backend.

**Memory:** [[v6-apply-torque-spring-falsified]].

### 3x. Three silent confounds reversal (2026-06-02)

**Major retroactive correction.** Detailed analysis in
`docs/findings/2026_06_02_three_confounds_lessons.md` and
`docs/findings/2026_06_01_silent_confound_audit.md`.

Three confounds were silently active in every multiarm A/B on this branch
from ~2026-05-15 onward:

1. **Knee limit too narrow** (`KNEE_LIMIT_LOW = −1.70`) — only 5.7° of bend
   past rest; V9-family spider rewards had peaks outside the kinematic
   manifold.  Fixed `f9ab634`.
2. **Curriculum advance threshold** — initially diagnosed as a bug, turned
   out to be a red herring; `cumulative_alive_ticks` only resets on falls,
   not at mc_period.  Threshold 1800 is reachable.
3. **`run_one()` never set `OGMA_PICRAWLER_CURRICULUM_AUTO_ADVANCE=1`** —
   the actual blocker.  Every multiarm A/B from `picrawler_multiarm.py` /
   `picrawler_ab.py` loaded a curriculum but kept auto_advance=False, so
   stage 2 (walking reward, V9 stage-2-only params, etc.) **never loaded**.
   Fixed `a4ec955`.  UI path was unaffected — only headless A/Bs.

**Validation re-tests after fixes:**

- **§3s.UPDATE (Cruse v2 n=5 paired)**: original verdict "NULL." Re-test
  result: **Cruse v2 is a −1.82σ STRONG NEGATIVE** vs the no-Cruse-v2
  baseline.  Cuts longest_upright 74%, reduces brain learning 24%, causes
  44% more falls.  Original null was a stage-1-only data artifact.
  See [[v6-cruse-v2-destabilizes]] memory.
- **Walking reward as designed** ([[v6-walking-reward-destabilizes]]):
  +2.3σ paired effect.  Body trying to walk under current `chassis_y ×
  v_norm × phase × bonus_rate` falls 73% more, max_dist −50% vs walking
  reward disabled.  Reward design pushes brain into instability.
- **V9 spider-stance** ([[v6-v9-spider-unblocked]]): all 3 V9 family
  pilots PROMOTE.  V9b spider has 0 auto-resets per run vs baseline 8.8
  AR.  Spider stance IS more stable.  But V9b n=5 vs no-Cruse baseline
  shows max_dist −53% — spider is too low-CoG to drift far.  Stable
  attractor, not a walking primitive.

**Knock-on effects on registry §3:** every entry that was tested against
a Cruse v2 baseline OR that involved stage-2 reward parameters is now
under suspicion.  Specifically:

- §3i Phase 7.20-7.21 locomotion stack — full stack used Cruse v2; the
  −0.96σ max_distance result was vs broken baseline.  Re-test deferred
  per "focus on walking" decision.
- §3t P2 phase-bin — vs Cruse v2 baseline; null verdict suspect.
- §3u HK destabiliser — structural-trigger failure was a separate issue,
  but Cruse v2 baseline confound also applied.
- Cruse v3/v4/v5, bucket, phase-viscosity variants — all built on top
  of Cruse v2.  Foundation itself was wrong primitive.

**Status of these entries:** marked-suspect in this footnote; not
retroactively re-tested at scale.  The walking-paradigm redesign
(`docs/plans-and-designs/walking_paradigm_redesign.md`) takes priority
over re-litigating individual §3 nulls.  Specific §3 entries will be
revisited if-and-only-if relevant to a future mechanism design.

**New best baseline (2026-06-02 onward):**
- Config: `the_picrawler_stand_target_per_servo_perceptual_cpg_trot.json`
  (NO Cruse v2)
- Curriculum: `picrawler_stand_walk_gated_trot_advance_1200.json` (works
  with the auto-advance fix; equivalent to original 1800 threshold)
- Reference numbers: chassis_y_late ≈ 0.095, longest_upright ≈ 33k
  ticks, max_dist ≈ 2.7m, preW ≈ 30×

### 3y. Stage 2 walking-paradigm-redesign step_quality channel — FALSIFIED across v1/v2/v3 (2026-06-02)

Walking paradigm redesign per `docs/plans-and-designs/walking_paradigm_redesign.md`
proposed a Stage-2 reward channel that rewards per-leg lift-and-plant
cycles in-place, reachable without velocity or destabilization.
Three formulations were built, smoked, and A/B'd at n=2.

| Variant | Reward shape | Share | Δ longest_upright vs gain=0 | Verdict |
|---|---|--:|--:|---|
| v1 boolean lift_state @ threshold 0.08 | (foot_y > 0.08) AND (foot_y < 0.04) | 0% | n/a (channel silent — pilot data showed max foot_y = 0.05-0.07) | unreachable from natural standing |
| v2 continuous lift × plant disjoint bands | clamp(1 − y/stance) × clamp((y − stance)/scale) | 0.3% | −27% | below noise floor + adds slight stream noise that fragiles policy |
| v3 CPG-phase contrast | clamp(2 × foot_dev/scale × sin(φ + leg_offset)) | 4.8% | −39% (bimodal: one seed −68%, one +13%) | signal IS loud enough to shape policy; resulting policy update destabilises (more foot lift → no inter-leg coord → falls) |

**4-arm Cruse × step_q matrix** (n=2 × 20 min, `results/stage2_cruse_matrix/`):

| arm | longest_upright μ | Δ vs A | pct_high_tilt | tipovers |
|---|--:|--:|--:|--:|
| A noCruse_noStepQ | 50,220 | — | 0.030 | 0.002 |
| B Cruse_noStepQ   | 10,800 | **−78%** | 0.234 | 0.025 |
| C noCruse_StepQ   | 30,660 | −39% | 0.020 | 0.007 |
| D Cruse_StepQ     | 18,510 | −63% | 0.098 | 0.019 |

**Every perturbation hurts longest_upright.**  Only the inert control
(A = standing-only reward, no Cruse) preserves the substrate's
standing competence.  Adding either Cruse or step_q (or both) shifts
the body toward fragility without producing measurable coordination
benefit at n=2.

The "Cruse v2 + standing-only = alive shuffle" hypothesis (Joseph UI
observation, [[v6-cruse-v2-destabilizes]] follow-up #2) was
**falsified by arm B's −78% longest_upright + seed 43 spending 40%
of run in high-tilt state**.

**Methodological lesson:** this is the SECOND time reward-shape
iteration at this layer has failed (after Phase 7-8 reward graveyard).
Per [[project_walking_not_emergent_from_perception]]: "Bottleneck is
multi-joint coordination on action side, not perception
representation.  Next substrate moves should add architectural bias
toward coordination (Hebbian synergies, mitosis, active inference),
NOT more EPMs/voters/reward shaping."  Confirmed again here.

Curricula committed: `picrawler_stand_step_quality_in_place.json` and
`..._ctrl.json` retained (Stage 2 reference + control).  v3 reward
shape committed at `61005e8` (active in body code at gain=0; default
preserves byte-identical legacy behaviour).

### 3l. v3/v4/v5/v6 chemotaxis/CartPole/MountainCar prior trail (referenced for context)

These are not picrawler-tested but inform our priors.  See linked memory files.

| Mechanism | Phase | Picrawler status | Source memory |
|---|---|---|---|
| TD-Premotor / value baseline | v5.1 / 6.5.36 | inherited as `baseline_lr` (off in picrawler MC mode) | [[project_v4_phase6_5_29_td_premotor]] |
| Drive-coupled Premotor | 6.5.30 | not enabled | [[project_v4_phase6_5_30_drive_coupled]] |
| Epistemic novelty bonus | 6.5.31 | tested as 7.4 active inference — null | [[project_v4_phase6_5_31_efe_premotor]] |
| MountainCar lambda=0.95 | 6.5.34 | env-class-specific; picrawler is not MountainCar-class | [[project_v4_phase6_5_34_lambda_tuning]] |
| CartPole REINFORCE (Phase 6.5.33 → 6.5.37 retraction) | 6.5.33 | methodology lesson: verify brain→body action path | [[project_v4_phase6_5_37_cartpole_artifact]] |
| Hebbian co-activation lesson (Phase 6.5.33) | 6.5.33 | textbook REINFORCE regresses on chemotaxis | [[project_v4_phase6_5_33_hebbian_lesson]] |
| Whisker bonus reward shape | 7.x archive | ABLATED (Goodhart attractor) | [[project_whisker_bonus_ablation]] |
| Saliency reward shape | v3 | known Goodhart trap (wandering) | per `feedback_no_tuning` |

---

## Section 4 — In flight

- **Diagnostic-first calibration push (2026-05-31 reframe — current active lead).**
  Triggered by the 2026-05-31 belly-flop discovery (see §3o): individually-correct reward
  channels combined into a 3-hour failure mode hidden from manifest summaries. Active
  push = **A + C from `reward_landscape_calibration_strategy.md`**: (A) per-channel
  reward attribution + dominance warnings in `summarize_logs.py`; (C) aliveness-gated
  curriculum replacing tick-count stage advancement with capability gates (heading
  regulation, proto-gait, obstacle-triggered TLE). Goal: end the "add mechanism → blind
  n=1 → infer from manifest" loop that produced 7 days of rich data with no breakthrough.
  Either outcome (substrate works / substrate doesn't) becomes legible. See
  `docs/plans-and-designs/picrawler_diagnostic_calibration_plan.md` — ⚠ **that plan did not
  survive the repo split and is not recoverable from this repo's history.** Its surviving
  content: the servo-saver / `motor_authority_scale` model in `docs/servo_dynamics.md` §4(d),
  and Bernoulli-Impulse Actuation in `docs/glossary.md`.
- **Gait ignition (2026-05-29 framing, ACTIVE as theory / DEFERRED as build).** Standing
  is a solved *fixed point*; gait is a *limit cycle* the body must ignite out of the
  standing basin; real efficacy must be non-ambiguous in a single ~20-min run, gated by
  **standing→gait latency** (`scripts/gait_ignition.py`), not seed-averaged deltas.
  **Empirical finding:** no run on disk shows a limit cycle — even the most "mobile"
  60-min runs are slow shuffles (mean ~0.13 m/s); latency = ∞ everywhere. **Named next
  bet:** wire the dormant `HomeokineticExploration` drive into Cruse v2 + perceptual-CPG
  as a standing-basin destabiliser. **Status 2026-05-31:** still un-built. The
  2026-05-30 work pivoted to the gait-cycle reward (§3n) — explicit deviation from
  `gait_ignition_theory.md` §7's "no reward bonus on the fixed point" guidance — and
  landed exactly in the predicted Pattern E destination (see §3n results). HK
  destabiliser becomes the natural follow-up *after* the diagnostic_calibration push
  gives us instrumentation to read its result. **2026-06-01 demo evidence (§3v):** body
  + Cruse v2 can incidentally translate ~1 m to a pyramid during the pre-convergence
  high-entropy window before the brain locks to standing. Locomotion is not blocked by
  morphology / perception / motor control; the blocker is reward landscape. Strengthens
  the case for a drive/reward-side ignition mechanism over any further coordination
  iteration. See `docs/plans-and-designs/gait_ignition_theory.md`.
- **Generic6DOFJoint3D migration — LANDED 2026-06-03.** 12 leg joints now have a
  backend-selectable construction path: `joint_backend=hinge` (default, preserves all
  historical baselines) or `joint_backend=g6dof` (Bullet's btGeneric6DofConstraint with
  per-axis angular spring + damping). Launcher dropdown writes
  `ExperimentConfig.picrawler_joint_backend`; env var `OGMA_PICRAWLER_JOINT_BACKEND`
  for headless. Per-joint-type spring stiffness/damping (hip1, hip2, knee) live-tunable
  via the reward_panel. Joseph hand-tuned compliant-stand preset saved as
  `picrawler_g6dof_compliant_stand_handtuned.json` + applied as the G6DOF default-on-
  selection so the body stands cleanly under passive compliance the moment the operator
  picks G6DOF in the launcher. Key implementation footnotes:
    - HingeJoint3D uses local +Z as hinge axis; G6DOF follows Bullet's default where +X
      is the TWIST axis (full ±π range) and Y/Z are SWING axes (mathematically capped at
      ±π/2). The G6DOF construction path remaps the basis so the hinge axis lands on +X.
      All motor/spring/limit setters on G6DOF target the X axis; locked Y/Z stay at 0.
    - Asymmetric knee mapping (`KNEE_RANGE_FOLD=3.20` / `KNEE_RANGE_HYPEREXT=0.85`,
      renamed and value-swapped 2026-06-03) — original constants were labelled wrong
      relative to actual geometry; brain `u_knee=+1` now correctly maps to deep tuck
      (spider stance reachable), `u=0` is straight (= REST), `u=-1` is hyperextension
      past straight. Affects the brain's interpretation of u_knee sign — REINFORCE
      re-learns the sign convention organically.
    - G-mode slider inversion no longer clamps u to ±1 (the clamp is correct for brain
      inputs but ate operator-slider intent past those u values).
  Operator observation: G6DOF body takes longer to converge to standing than hinge
  (more complex balance solve due to springy joint chain), but stance pose and stability
  are comparable once converged.
- **Walking-paradigm step_quality redesign — FALSIFIED 2026-06-02 (§3y).** All three
  formulations (v1 boolean, v2 lift_ema × plant_ema disjoint bands, v3 CPG-phase
  contrast) and the 4-arm Cruse × step_q matrix returned net-negative or null. Every
  perturbation hurt longest_upright; only the inert control preserved standing
  competence. Reward-shape iteration at this layer is exhausted — per
  [[project_walking_not_emergent_from_perception]], next moves must add architectural
  coordination bias, not more reward shapes.
- **NEXT ACTIVE LEAD (queued 2026-06-03): Premotor active-inference upgrade on the G6DOF
  substrate.** Per Joseph's 2026-06-02 decision and his "resonance discovery" framing:
  the new compliant body has natural frequencies the brain can DISCOVER via
  prediction-error minimization rather than have to compute. Replace per-Premotor
  REINFORCE-only learning with a predict-next-foot/motor-state + EFE-style action
  selection (the project's CLAUDE.md namesake mechanism). Brain's preferred prior over
  motor state can be the CPG-phase distribution so gait shape emerges from "match the
  rhythm I expect" without explicit reward shaping. The Cruse rules stay as the
  inter-leg coordinator (avoiding re-inventing the Hector wheel). Picrawler stays on
  G6DOF with the hand-tuned compliant defaults as the new substrate baseline.
- **Substrate-claim foundation (2026-06-02 Section 1.A wins).** Three substantiated
  wins (dynamic standing without programming, body-agnostic brain across 5+ motor
  variants, radial drift) are documented in §1.A and serve as the substrate-level
  claims foundation for all future picrawler experiments. Win 2 in particular suggests
  the next cross-morphology bet (2-leg or 6-leg variant of the same brain wiring) is
  worth setting up once walking is solved on the 4-leg picrawler.

## Section 5 — Built but not yet measured at adequate power (DEFERRED)

| Mechanism | Phase | Reason deferred |
|---|---|---|
| HomeokineticExploration | 6.5.x | shipped, off-by-default in picrawler; AMOS-style stuck-escape primitive |
| MotorFader / FaderController | 6.6.F/G | shipped, frozen-agent test passed but no recent Cell/picrawler use |
| EmbeddingRegistry | 6.6.E | shipped, no consumers yet |
| Predictive chunk dispatch | (action_side §5) | blocked on `seqgng_body.baked_count=0` under tabula-rasa picrawler |
| Inverse motor model | (action_side §6) | not started; substantial new substrate class |
| Learned (Matsuoka) CPG | (action_side §4) | not started; Pleurobot-style |
| Action chunking / open-loop gait primitives | Phase 8 | tested and falsified as the next route; only revisit if a known-good translating primitive is used as a seed inside a closed-loop/abortable controller |

---

## Section 6 — Cross-reference index

### Mechanisms by category

**Reward shape** (affects events.hit / events.miss firing):
- Multiplicative gating ✓ BASELINE
- Standing reward (trapezoid + tilt_norm) ✓ BASELINE
- Walking reward (radial / radial_penalize_inward / to_target) ✓ BASELINE
- Phase contrast (gated bonus multiplier) — PARTIAL (rescue only)
- Per-leg reward decomposition — REGRESSION
- Whisker bonus — ABLATED
- Saliency reward — known Goodhart

**Premotor action selection** (logit-space modifiers pre-softmax):
- Epistemic gain (`(1 − visit_ema) × gain`) — Phase 7.4 NULL
- Pathway temperature gain — DEFERRED in picrawler
- Output noise amplitude — Phase 7.4 NULL
- Value head bias — Phase 7.8 WORKING (stability)
- Rhythm bias topic — Phase 7.9 / 7.11 (consumer slot, single producer at a time)
- Entropy anneal (T multiplier) — Phase 7.10 WORKING (standing)

**Premotor learning rule modifiers**:
- mc_lr / mc_gamma — BASELINE (sweep tested 7.6.G)
- mc_reinforce — BASELINE (variant tautologous 7.6.R)
- advantage_normalization — BASELINE (variant tautologous 7.6.N)
- eligibility_lambda — DEAD_CODE under MC mode (7.6.E)
- baseline_lr / value_w — DEAD_CODE under MC mode
- mc_episode_topic — BASELINE

**Voter mechanics**:
- Group balance — BASELINE on, false-variant null (7.6.V)
- Priority group — `vestibular` BASELINE, variant tautologous (7.6.V)
- Trust epsilon — BASELINE at 0.05
- Voter topology (diagonal split) — Phase 7.6.D NULL

**Sensor primitives**:
- Compass (world heading) — BASELINE
- Radial compass (body-frame outward) — BASELINE
- Feet Y publishing — BASELINE
- Tilt — opt-in
- IMU — BASELINE
- Joints (12-D monolithic) — BASELINE (per-joint variant regressed at 60 min, see 7.2-EPM)

**Inter-leg coordination**:
- Diagonal-pair voters — NULL
- CoactivationMatrix (Hebbian cross-Premotor) — REGRESSION
- SynergyTimer (per-leg phase bias) — WORKING in composition
- Phase contrast (reward-side) — PARTIAL
- CruseCoordinator (structural prior) — WORKING on stability / NULL on legacy navigation
- GaitSelector / action vocabulary — NULL on directed locomotion (Phase 8)

**Capacity / scaffolding** (perception-side, untested in picrawler):
- EPM mitosis — DEFERRED
- Hierarchical EPM per-leg — REGRESSION at 60 min

### Mechanisms by failure pattern

**Pattern A — "tightens variance without lifting mean"** (substrate-ceiling signature, REAFFIRMED across coordination paradigms):
- Value head, SynergyTimer alone, Entropy anneal alone, Phase contrast alone, Lowstance, CruseCoordinator v1, **CruseCoordinator v2 / v4.2 / v5.1 (all 12+ mechanisms tested at n=10+)**
- **Phase-viscosity P1 dwell** (2026-05-28): mechanism reduced tick noise but collapsed translation (path 1.46m vs baseline 115.64m).
- **Phase-viscosity P2 phase-bin** (2026-05-29): single-seed +0.061 aliveness on distance / 310 falls on tilt80. Retired before n=10 powering.
- → spans LEARNED mechanisms (value head, synergy, anneal) AND faithful structural mechanisms (full Walknet 3/6 rules + constant gain + hard authority + phase-aware symmetric coupling + Rule 6 step-duration memory + body-state gating + rhythm-coupled response windows)
- → **Phase 7.13 final**: every coordination-layer mechanism tested hits this signature.  Two independent Stage 3 n=10 tests on Cruse v2 vs anneal both nulled on max_distance.  v9-retraction pattern confirmed.
- → **Phase 8 update**: action vocabulary/gait options were tested next and did
  not solve directed locomotion. Static posture vocabularies produced stable
  shuffle; temporal gait options produced rote replay, circling, or cancellation.
- → **Revised takeaway**: the legacy navigation objective is itself suspect.
  `max_distance_from_origin` can select for dead drift, while the Cruse lineage
  produced lower-scoring but more adaptive closed-loop behaviour. The next
  architectural bet must be preceded by an aliveness/adaptation metric.

**Pattern B — "stability gain costs exploration"** (Phase 7.2-EPM, per-leg credit, motor CPG):
- Damping motor noise / over-committing to stance reduces translation discovery
- → coordination must come from STRUCTURAL prior, not reward incentive

**Pattern C — "dead code under MC mode"**:
- eligibility_lambda, baseline_lr value-baseline path
- → Premotor has TWO reward paths (apply_reward vs finalize_mc_episode); picrawler uses MC only
- → mechanisms in the apply_reward path don't fire

**Pattern D — "tautology because already on"**:
- advantage_normalization, mc_reinforce
- → check current_params() before designing A/B against a "new" knob

**Pattern E — "activity without aliveness"**:
- Full locomotion stack and progress-PB reward (Phase 7.20-7.21)
- **Gait-cycle reward v2 / v3** (2026-05-30): direct reward-pulse on 4-foot cycle completion. v2 (relaxed static thresholds) fired 17 pulses → 4.6× falls, 12× tipovers, longest-upright cut 25.6→3.8 min, ht_mean 0.54→0.08 (distressed). v3 (adaptive thresholds) showed two-phase collapse: standing-trap mins 6-19 with weights FROZEN at preW=28.39, then policy collapse at min 20. Brain learned to lurch for the pulse or freeze into a high-stance posture earning the pulse intermittently. **Predicted by `gait_ignition_theory.md` §7.**
- **level_chassis ungated belly-flop** (2026-05-31): `level_chassis_rate=0.05` without `reward_min_height` gate accumulated 86% of all reward across a 3-hour UI run. Body settled at chassis_y=0.035 (belly position), twitched in place, DA saturated 1.0, weights kept growing. Hidden for days behind individually-correct knobs. Fixed `picrawler_body.gd:2664-2674`.
- → `da_mean` can climb monotonically while path length, height, and learning
  collapse. Activity is not agency; a metric must distinguish closed-loop
  adaptive reorganisation from jitter, circling, and rote replay.
- → **Critical 2026-05-31 lesson:** Pattern E can arise from **channel interactions** (gates that are correct individually but combine into a dominant attractor), not just from a single bad reward. Detect via per-channel reward attribution (`reward_landscape_calibration_strategy.md` A) — any channel exceeding ~60% of total reward should trigger suspicion.
- → **2026-05-31 systemic audit (§3p):** standing-channel dominance is the *default state* of our reward stack across Phase 6/7/8 — 85 of 125 analyzable historical runs at 92% avg. Phase E was not just a few bad runs; it has been the background condition for months. The Phase 7.5.R `gated_walk_bonus` multiplicative-gate design is the only mechanism in the archive that ever broke this — and we drifted away from it. Read §3p before interpreting any prior Pattern A/E/F null.

**Pattern F — "open-loop primitive without differential outcome"**:
- posture vocabulary and `GaitSelector`
- → if no primitive actually changes the world in a useful direction, the
  selector has no advantage signal. Open-loop primitive libraries are not a
  substitute for a closed-loop control substrate.

### Decision rules for future mechanism proposals

Before proposing a new mechanism, check:

1. **Same path as a known null?** If the proposed mechanism modulates the same Premotor or reward path that another mechanism null'd, expect a similar null absent a different context.
2. **Dead-code check.** Is the affected code path live under the current config's mc_lr / mc_reinforce / etc. settings?
3. **Tautology check.** Read current_params() of the running brain.  Is the "new" param already at the proposed value?
4. **Goodhart check.** Does the proposed reward shape have a degenerate solution that doesn't require the desired behavior?
5. **Variance-mean check.** Does the proposed mechanism *constrain* the policy (likely lowers mean) or *unlock new policy space* (could lift mean)?  Most of our nulls were variance-constrainers.
6. **Structural-vs-learned check.** Is the inductive bias the mechanism provides *learnable from reward* (we've tried 10+ variants) or *axiomatic from architecture* (the direction CruseCoordinator pursues)?
7. **Aliveness check.** Would the mechanism still look good if scored on
   perturbation response, prediction-error recovery, sensorimotor coupling,
   and target-reaching under obstacle constraints rather than max distance?
   If not, it is probably optimizing drift or jitter.

---

## Section 7 — When old mechanisms might apply again

| Mechanism | Original outcome | Future re-use context |
|---|---|---|
| eligibility_lambda | Dead code | If we switch picrawler off MC mode (mc_lr=0), apply_reward is active — λ becomes alive |
| Per-leg reward decomp | Goodhart on stance | If reward is purely walking-velocity-based (no standing reward to Goodhart against), per-leg credit might cleanly attribute |
| CoactivationMatrix | Hebbian indiscriminate | Could compose with CruseCoordinator: structural prior shapes WHICH co-firings get reinforced |
| Diagonal-pair voters | Null at action level | Could compose with closed-loop chunks only after a translating primitive and outcome abort exist |
| Hierarchical EPM | Stability cost | If action-side problem is solved (e.g. via CruseCoordinator), hierarchy might re-engage productively as the lateral inhibition layer |
| Active inference (epistemic + noise) | Wandering | If we add a forward model that converts wandering into navigation evaluation, the epistemic gain might find good states |
| Motor CPG (rhythmic accel injection) | Suppressed exploration under dense standing reward | If reward is sparse (only walking, no standing) and CPG amplitude is small, might provide useful rhythm without dominating |
| GaitSelector / action vocabulary | No directed locomotion | Only as an executor for a known-good translating primitive with closed-loop abort/outcome gating; not as the next research bet |
| Saliency / whisker reward shapes | Goodhart | Always at risk; only use when a structural prior bounds the policy space |

---

## Section 8 — Bookkeeping

**Last updated**: 2026-06-17 (adversarial-audit response — claim-vocabulary correction: the canonical Motor-EPM line is **"no external reward" + internal agency-fitness search**, NOT "reward-free"; see `docs/2026_06_17_adversarial_audit_report.md` F1 and the claim vocabulary in `docs/findings/motor_epm_results.md`. NOTE: this registry still trails the June MotorEPM/agency-reward content and should be promoted to a per-claim ledger — audit F4 / §"claim ledger". ENFORCEMENT NOW LIVE: `scripts/audit_claim_metadata.py` linter + `scripts/picrawler_run.py --claim-mode` gate + scaffold-explicit config tiers (`scripts/make_motor_epm_tiers.py`) + MotorEPM C++ regression suite (`cpp_core/tests/ogma/test_motor_epm.cpp`). See `docs/position-papers/embodied_scaffolding_principles.md`.)

_Prior entry — 2026-05-29 (gait-ignition reframe: P2 retired as lead; standing→gait latency instrument; HomeokineticExploration named as next bet)._

**Authors / contributors**: Joseph Butera III + Claude Opus 4.7

**Linked findings docs**:
- `docs/phase7_5_long_run_findings.md` — narrative for Phase 7.5-7.10
- `docs/phase7_arc_findings.md` — Phase 7 arc closure (pre-7.5)
- `docs/phase7_2_epm_findings.md` — hierarchical EPM
- `docs/phase7_cpg_status.md` — motor CPG ablation
- `docs/action_side_plan.md` — planned-but-not-tested queue
- `docs/synergy_timer_spec.md` — module spec
- `docs/findings/phase7_20_findings.md` — full-stack/progress regression
- `docs/findings/phase8_findings.md` — action-vocabulary falsification
- `docs/operational/aliveness_metric_protocol.md` — replacement metric protocol

**Linked memory files**: see Section 3l for the v3/v4/v5/v6 trail and Section 5 cross-reference index.

**Update rule**: amend this registry when:
- A new mechanism is committed (add entry)
- A Stage 2/3 result lands (update Status + Primary effect)
- An ABLATED or NULL mechanism gets revisited successfully (move to WORKING/BASELINE)
- A mechanism's failure mode is re-diagnosed (update Reason)
