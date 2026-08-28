# Picrawler lever ledger — what is promoted, what is refuted

*The per-lever verdict record for the picrawler active-inference gait (branch
`picrawler-dev`). **Read this before proposing a lever.** Most plausible ideas here have
already been built and falsified; re-proposing a dead one is the single most common way to
waste a session.*

*Companions: [`picrawler_gait_loop_findings.md`](picrawler_gait_loop_findings.md) (the
narrative + architecture), [`../plans-and-designs/picrawler_active_inference_plan.md`](../plans-and-designs/picrawler_active_inference_plan.md)
(the plan), [`../brain_building_doctrine.md`](../brain_building_doctrine.md) (the method),
[`../../CLAUDE.md`](../../CLAUDE.md) (the A/B protocol this ledger's verdicts were produced under).*

**Last updated: 2026-07-27.** Status as of `picrawler-dev` ~`ea9265b` + the stroke-to-step
lock work below.

> ⚠️ **Every corridor number recorded before 2026-07-27 is from a DIFFERENT GYM.** The
> corridor's back wall was a vertical seal a robot could park against, and its far end
> dropped off the world; both are now 30° self-centering ramps. Re-measured deployed
> baseline (corridor, n=4, 6000 ticks): net_z **4.58 ± 0.27**, straight 0.73, flat_v 0.05,
> step_bal 0.48, tilt_sd 0.065, planted 3.69, 0 falls. See the harness-defects box in §2.

---

## How to read this ledger

> **Nothing in this ledger is dead.** Every negative entry means *refuted in the context, at
> the power, and against the baseline stated* — never "this idea does not work." A refuted
> lever whose re-use context arrives is a lever to try again, and several entries below exist
> precisely because a lever was re-tested in a second regime. See `CLAUDE.md` §3.1–3.2 for
> the verdict vocabulary and the checks that must pass before any negative verdict is trusted.

- **A verdict is scoped to its SCENARIO.** "Refuted" means *refuted in the regime tested*.
  A lever refuted on flat ground has **not** been refuted on an incline — re-test it there
  before treating it as settled (and record the second verdict here). Conversely, do not cite
  a flat-ground refutation of a lever whose whole premise is terrain.
- **A verdict is also scoped to its BASELINE.** A null measured against a degenerate control
  is a fact about the control. This project has already had to reopen an entire era of nulls
  for exactly this reason (§6). Before believing a negative entry — including one you wrote —
  run the seven checks in `CLAUDE.md` §3.2.
- **Results before 2026-07-23 are single-seed.** The seed override was broken (it rewrote
  0 modules, so `OGMA_SEED` did nothing and every run was byte-identical). Pre-fix results
  are **byte-perfect isolations but not seed-averaged** — generality untested. Re-measure
  before building on them.
- **Every lever below shipped gain-0-guarded** — at gain 0 the build is byte-identical.
  Refuted levers whose infrastructure is harmless are **kept, default-off**, not ripped out.

### Verdict vocabulary

Every entry carries one of these **plus the scenario, the power, and the baseline** it was
decided against, and — if negative — a re-use context (§6).

| Verdict | Meaning |
|---|---|
| `BASELINE` | in the deployed stack |
| `WORKING` | real positive effect, kept but not in the default stack |
| `PARTIAL` | effect on secondary metrics, null/regression on the primary |
| `NULL` | no measurable effect **at the power and against the baseline stated** |
| `REGRESSION` | measurable negative effect |
| `TAUTOLOGY` | the variant was byte-identical — the mechanism was already on |
| `DEAD_CODE` | no effect because the code path wasn't live in that config |
| `ABLATED` | actively removed for negative consequences |
| `DEFERRED` | built but never tested at adequate power |
| `IN_FLIGHT` | under test now |

**`TAUTOLOGY`, `DEAD_CODE`, and a `NULL` against a broken baseline are measurement outcomes,
not verdicts on the idea.** Before recording any negative entry, run the seven checks in
`CLAUDE.md` §3.2.

---

## ★ THE OPEN PROBLEM — start here (2026-08-09, rewritten same day after the crash finding)

**The stack walks. It always did. The day's "shuffle attractor" was a process crash
scoring artifact** — see the ★★★★ §4 entry: a hardened-toolchain assert killed every
run at the moment progress-commit saturated (i.e. when the body committed to walking),
and `seedavg` scored the corpses. With the crash fixed, the v2base canonical walks
**6/6 screened seeds at net_z ~6.5, ~110 real steps** — consistent with the Aug-7
baselines, whose "non-reproduction" is now fully explained (the old binary predated
the hardening).

What is genuinely open, unchanged from before this detour and now measurable on clean
instruments:

1. **Gait REGULARITY — the operator's stated goal.** `step_cv` ≈ 1.0 (memoryless
   inter-step intervals) even on walking runs, and `td_plv` stays 0.04–0.21: nothing
   in the timing chain references contact. The audit's structural findings all stand
   (retrograde `L.phase`, three unsynchronized clocks, press-fights-lift). P1's clean
   scoring (post-fix, full walking runs) says the incumbent state-observation phase
   beats every smooth candidate on body-coupling — the substrate question is settled
   for now; the ENTRAINMENT question (P4: lock the stroke to touchdowns it can now
   feel) is the live thread.
2. **The recovery-shear toll** — real trace physics, unaffected by the crash: every
   leg pays −0.004…−0.015 g/tick braking during the 7–9-tick pressed window after its
   commanded stroke reverses. The stance release's CONVERSION story is retracted
   (crash-confounded), but its mechanism target survives; clean n=20 dose re-measure
   in flight.
3. **Flat speed** — `fwd_v` ~0.04–0.05 on walking runs; the old pinned-flat-speed
   thread returns as the transport question once rhythm exists.
4. **The criterion question is REOPENED** (its refutation was measured on crashed
   runs) — re-run `actsweep.py` on the fixed build before any verdict.

---

## 1. The deployed stack (promoted)

Base config: `the_picrawler_motor_epm_embed_corridor_bearinghold.json`, built on the
milestone gait `the_picrawler_motor_epm_embed.json` (emergent CPG-embedding + firm stance;
walks, and improvises a novel limb movement to free itself when stuck).

| Lever | Setting | Verdict & evidence |
|---|---|---|
| **CPG-as-embedding** (`cpg_embed`) — controller learns a phase-conditioned feed-forward from the keyframe error | on | **★ The milestone.** Phase as *context*, not as drive → emergent, self-rescuing gait |
| **Firm stance** (`postural_gain` 0.3→0.7) | 0.7 | Defends the stance → 0 falls, no collision, sharper phase map |
| **KeyframeGait** phase-indexed map + self-precision gate | on | Learns coordination; drives on consistency |
| **BodyRhythmTracker** PLL + CPG entrainment | on | CPG tracks the body — fixes the washout |
| **Heading bearing-hold (P)** — PD on dead-reckoned own-yaw through the *authoritative* skid-steer channel | `heading_bearing_hold_gain=7.0` | **Heading SOLVED.** Straightness 0.05→0.53 **and variance collapses** (net_z std 0.92→0.07). P-sweep {5,7,10,14}: 7 is the sweet spot (10 ties but re-adds a turn outlier; 14 overdrives→0.44). **POSITIVE = go straight; negative = catastrophic spin** |
| **Yaw-rate damping (D)** | `heading_hold_gain=0.3` | Pairs with the P term above |
| **Progress→commit** — sustained forward progress damps exploration σ + adds thrust | `progress_commit_gain=1.0` | **Marginal keeper.** net_z +2.55→+2.62; variance holds; falls unchanged. Seizes a found push, kills the circle-then-go dither. (0.5 too weak, 2.0 over-commits) |
| **Belly ToF rangefinder as the height observation** | `height_topic=reality.proprio.ground_clearance`, `height_k=0.30` | **Hump SOLVED** + Markov-compliant, *the same move*. Replaces the god's-eye `chassis_y_norm` (see §3) |
| **Height-homeostat windup fix** — defense fades with forward progress (`height_rest_frac`) | on | Height is a **standing** reflex: full defense at rest (Gate 0 preserved), →0 while walking. A hip2 lift bias on an incline hoists the legs off the slope = traction loss. Flat net_z +2.62→+3.48, straight 0.67, **falls 0**; teleport-to-hump final_z **+4.11, all seeds clear** (was ~2.6 stall) |
| **Stance-lift knee tuck** — constant knee-tuck bias on **planted legs only** (Cruse foot-height gate; swing legs untouched) | `stance_lift_gain=0.5` | **Belly-up SOLVED.** Clearance 0.015→0.030 (min 0.003→0.008 — off the ground), net_z +1.46→+1.75 (**~20 % faster**), 0 falls, **still clears the hump belly-up** = unified. The stance-gating is the whole trick; a blind DC knee bias kills the gait. Currently in `..._stancelift.json`, pending bake into the base config. **2026-07-25 — gain sweep 0/0.5/0.8 (n=4): 0.5 confirmed as the optimum, and its cost is now understood.** Belly clearance rises monotonically with gain (0.016→0.026→0.033) but so does damage: 0.8 gives net_z 1.87, straight 0.28, tilt_sd 0.178, 0.75 falls. The cause was **not** the bias — it was the swing detector it gates on (see §4), and the reason "the stance-gating is the whole trick" is subtler than recorded: what makes it rhythm-safe is that the bias is *modulated*, but it was being modulated by a self-referential signal beating against the gait clock |
| **★ Reward-free coordination fitness** — the (1+1) phase search keeps running, but ranks probes by `coherence · activity / (1 + tle)` instead of by forward velocity | `coord_fitness_mode=1` (0 = legacy fwd_v, byte-identical) | **Resolves the §4 reward problem at no cost.** n=4 vs the fwd_v-reward arm: net_z 3.52→**3.76**, straight 0.67→**0.70**, tilt_sd 0.069→**0.067**, 0 falls; hump gate **identical** (final_z 5.21, gain_z 2.59); recovery gate **better** — coordination returns 0.30→**0.48** after perturbation, which is the structural point: *a thrash can no longer become the incumbent, because a thrash has high `tle` and low coherence.* For contrast, deleting the search outright costs net_z 3.52→2.36 and step_bal 0.26→**0.03** ⇒ **the capability was the SEARCH, not the reward.** **Why all three factors are needed** (each was measured, not assumed): coherence alone is *maximal on a frozen body* at all-equal offsets — R = \|mean e^{−iP_j}\| = 1 — and the search can reach those offsets; `1/(1+tle)` alone also favours freezing (a still body is trivially predictable — the same reason Cphi was deliberately not trained on HK surprise); **the activity term is the homeokinetic normalisation that kills both**, since it → 0 when the body stops. Predicted failure mode was coherent marching-in-place; it did not occur. **Honest caveat: post-perturbation progress is LOWER (1.35→0.78)** — the fwd_v reward was directly buying that, and removing it costs some of it |
| **Adaptive coordination** — a leaky first-order tracker pulling the Kuramoto target offsets toward the body's OWN measured per-leg phase pattern | `coord_adapt_rate=0.001` (0.005 over-drives: net_z 2.84, +falls) | n=4: **ties the baseline on every progress metric** (net_z 3.52 vs 3.62, flat speed 0.04, straight 0.67 exactly) while reducing wobble on **4/4 seeds** (tilt_sd 0.078→0.069). **Passes the obstacle gate and improves it** — teleport-to-hump final_z 4.57→**5.21**, better on 4/4 seeds, 0 falls. ⚠️ **The RATIONALE first recorded here was wrong** — it was written as "replaces an imposed topology with a learned one," but `coord_reward_drive=0.3` was already reward-searching that topology (see §4), and it **overwrites `gait_phase` wholesale every 240 ticks**, discarding this tracker's nudges unless a probe wins. The measured A/B stands (the ratchet was equally on in both arms); the *explanation* does not. **Safety property that does hold, and it is the important one:** a leaky tracker has **no stored winner and no fitness**, so unlike the ratchet it has nothing to lock in — it always tracks what the body is doing *now* (τ ≈ 17 s). **Pending UI observation** |

---

## 2. Refuted levers — **do not re-propose without a new scenario**

| Lever | Scenario tested | Why it failed |
|---|---|---|
| **hk_value as a gait-quality criterion** (`value = responsiveness/(motor_tle+ε)` as the quantity a selector would optimize) ⚠ **CONTAMINATED (2026-08-09, see §4 crash entry): the sweep ran on the armed-assert build — its runs were crash-truncated. The verdict below is RETRACTED PENDING RE-MEASURE on the fixed build; do not cite the +0.05 number** | corridor diff 0.3, solid chassis, 12 000 ticks, **13 arms × n=6 across SIX knob axes** (postural, stroke, coupling, stance_lift, stance_lift_hip2, amp) — `actsweep.py`, 2026-08-09 | **Refuted as a transport proxy: pooled corr(hk_value, net_z) = +0.05 over 78 runs; within-arm mean +0.15 with the sign flipping arm to arm.** The +0.996 of 2026-08-07 was four arm-means along ONE axis — and that axis still reproduces (hk 4.46/4.49/3.75 as amp rises 0.4/0.55/0.7, net_z collapsing to −0.04) while every other axis decouples: `coupling_gain` moves the criterion hard (corr −0.95) with no transport consequence (+0.12); `stroke_gain` moves transport (−0.86 — SHORTER strokes carry further, again) with no criterion movement (+0.20). A selector attached to hk_value today would crank coupling down for nothing. ⚠ Instrument scale differs from the 2026-08-07 read (raw \|dx\|/\|du\| time-mean ~1.25 vs the per-bin EMA's ~0.42), but the amp-axis ordering matches, which is the behavior the reference itself demonstrated. **Re-use context: per-axis validity is real** — the criterion may still gate the one knob whose axis validated it; and the generalization bar for ANY intrinsic criterion is now written: score it pooled across ≥5 knob axes, never along the axis that motivated it |
| **Complete-the-lift** (`height_lift_knee` — drive the knee with the same sign as the height homeostat's hip2 lift, as the panic pathway already does) | corridor, n=6, `CHASSIS_COLLIDE=1`, on top of the belly arm; fracs 0.5 and 1.0 | **`NULL` at 0.5, `REGRESSION` at 1.0 — with a diagnosis that names the flaw in the design, not the idea.** Premise was sound and measured: **hip2 and the knee agree on sign only 50.8 % ± 1.1 % of ticks — chance** — and the panic pathway's own comment records that opposite signs mean the knee un-tucks and *fights* the hip2 lift ("same sign = a coherent anti-gravity push"). At 0.5: everything neutral (net_z t=+1.11, straight t=−0.25, tilt_sd t=−0.70, swing_frac t=−0.05) and **`hk_agree` moves only 0.522 → 0.529**. At 1.0 the pre-existing verdict reappears — tilt_sd 0.076 → **0.194** (2.5×), straight 0.72 → 0.61 ± 0.27, one seed collapsing to net_z 0.27. ★ **WHY IT CANNOT WORK: the height-lift path is multiplied by `height_rest_frac`, which is ZERO while cruising — so it extends a pathway that is switched off exactly when the disagreement occurs.** `hk_agree` is dominated by locomotion ticks the lift never touches. **Re-use context: the target (hip2/knee sign coherence during LOCOMOTION) is still live and still unaddressed; it needs a carrier that is active while walking — `stance_lift_gain`'s stance-gated path is the obvious candidate, not the rest-gated height path.** Note the prior in-code verdict ("a DC bias there clamps the swing and kills the gait") is *not* what was reproduced: `swing_frac` barely moved (0.493 → 0.486); the 1.0 failure is destabilisation, not swing-clamping |
| **Load in the motor input** (`load_topic` on the bridge → per-leg `foot_load` appended as a 10th element of each motor proprio vector; motor EPMs 9→10 dims) | corridor, n=6, `CHASSIS_COLLIDE=1`, on the stancehip2+supportEPM arm | **`NULL` on behaviour, `REGRESSION` on the self-model — and the diagnosis is structural, not a tuning miss.** Behaviour is untouched: net_z **t = −0.02** (4.56 → 4.55), %<10mm t = +0.02, legs+ t = 0.00; straight (+1.15) and tilt_sd (−0.70) drift the right way but not significantly. **But `motor_tle` rises 0.263 → 0.320, t = +2.75** — the forward model got *worse at predicting itself* after being handed the observation. ★ **WHY: load is a BODY-LEVEL quantity and `MotorEPMv2`'s model is PER-LEG.** What a foot carries depends on all four legs and where the CoG is, not on that leg's own commands — so a per-leg model asked to predict it can only accumulate irreducible error. The consumer-fired check passed (10-D encode live, \|last_x\| 8.08, model running at n=10), so this is not a plumbing null. **Re-use context: load belongs to a BODY-LEVEL consumer — the support EPM sees all four at once and DID find structure (a self-limiting ~150-node vocabulary) — not appended to each leg's private state.** Retry per-leg only if the model is ever made body-level, or if a leg-local load derivative (rate of unloading) turns out to be leg-predictable where the level is not |
| **Homeokinetic support selector** (`support_select_gain` — value the current support state as responsiveness/(motor_tle+ε) and modulate `explore_mult` by it; gains 1.0 / 2.0 / 4.0) | corridor, n=6, `CHASSIS_COLLIDE=1`, control has contact wired instrument-only so the ONLY difference is the gain | **`NULL` — and the diagnosis is that the ACTUATOR HAS NO AUTHORITY OVER THE TARGET.** The criterion is sound and measured: responsiveness (egocentric \|Δx\|/\|Δu\|) is 0.470 at 2 feet planted vs 0.367 at 4 (+28 %) with \|Δu\| flat, so preferring responsive support states prefers 2-leg support **without ever being told forward progress is good** — homeokinesis, not reward shaping on progress (§5.1). Consumer verified firing: `support_mult` non-unity on 418/500 samples, correctly signed (4 feet down → 1.033 = explore more; 2 feet down → 0.901 = commit). **But nothing moved**: net_z t = −1.87 / +0.10 / −1.17 (non-monotonic noise), mean planted 3.21 → 3.17, %≤2-down 15.1 % → 17.3 % (t = +1.06…+1.69, right direction, not significant). ★ **WHY: `corr(explore_mult, feet planted)` = −0.06…+0.03 now, and −0.14…−0.17 at 20–60 ticks ahead.** Exploration does not move the support state, so **no gain on this actuator could have worked.** Same shape as `gait_phase` having no authority over footfall timing, and as `height_lift_knee` extending a path that is faded to zero while cruising. **Re-use context: the criterion is worth keeping and re-aiming at an actuator with measured authority over stance overlap — which nothing yet has. Find the actuator FIRST.** |
| **Stance-gated hip2 lift** (`stance_lift_hip2`, ships **0.25**, on the belly arm) | corridor, n=6, `CHASSIS_COLLIDE=1` | **`PARTIAL` — mechanism confirmed, transport not.** `stance_lift` biased the KNEE ONLY on planted legs ("no hip2 → no foot-lift traction loss"), but that reasoning covers hip2 *minus* (foot up); on a **planted** foot hip2 *plus* presses down and levers the chassis up — which Rule 5 and the panic pathway both already assert. So `stance_lift` was the one-joint version of a two-joint raise, in the one carrier that IS live during locomotion (the height path is faded to 0 while cruising, which is why `height_lift_knee` was NULL). **hip2/knee sign agreement 0.522 → 0.601, t = +13.5**, and **`fr` is RECRUITED as a propulsor (2/6 → 5/6 seeds), `rr` strengthens (5/6 → 6/6)**. net_z +1.04, straight/tilt_sd/swing_frac all ns — the old "clamps the swing" objection does NOT appear (swing_frac t = −0.17). ⚠ **But `fl` brakes 2.5× harder (−0.00165 → −0.00407, 0/6 positive), so NET forward force FALLS (+0.00168 → +0.00114): `legs+` is a blind count that tallies propulsors and ignores the brake.** Higher fracs are worse (0.5 → tilt_sd 0.225; 1.0 → straight t = −2.01). **Next target is `fl`** — the consistent brake, which this lever deepens |
| **Belly-grounding height setpoint** (`height_ground_gain`, swept 0.005–0.08; ships at **0.01**) | corridor, n=6, 6000 ticks, **`CHASSIS_COLLIDE=1`** — the honest body | **`WORKING` (signal).** The height homeostat's target (`height_k` 0.3 × discovered max) sits **below** where the body rides (`h_ema` 0.39–0.44), so `height_bias` integrates **negative** (−0.30…−0.47) and commands hip2 **down** while the belly is simultaneously grounding (54 % of the first 500 ticks under 10 mm). hip2 — the chassis elevator — is consequently **4.5× under-driven** (`u_hip2` ~±0.20 of full scale vs `u_hip1` ~±0.9) and the **knee carries the support from a permanently flexed, low-leverage posture** (occupies −1.36…−0.44 against `KNEE_REST = −1.6` straight). Making the setpoint fraction *rise while the belly grounds and decay otherwise* takes it from asserted to discovered. At 0.01: **%<10 mm 11.4 → 5.3 (t = −3.95)**, gc p1 +46 % (t = +1.91), **net_z unchanged (t = −0.14)**, **tilt_sd unchanged (0.076)**. Operator UI (at 0.02): *"higher stance definitely looks better … more confident … the rangefinder is actually doing work now, and the chassis is raised up above the rumble strips."* ⚠ **`tilt_sd` is the metric that picks the gain** — 0.02 nearly doubles it (0.136) and 0.08 trebles it (0.260) while `straight` degrades to t = −2.86; buying clearance by wobbling is the degenerate win here. ⚠ **Deliberately does not touch `height_rest_frac`** (the fade to 0 while cruising), which is measured and whose removal is refuted. Does **not** cost the single-engine gait: net-propelling legs 2.2 → 2.0–2.7, rr's forward-GRF share 42 % → 28–47 %, no systematic trend |
| **EPM input commissioning on the motor layer** (`dim_autocal_ticks=1200`, k=4 — measure each input dim's range, then freeze) | corridor, seed 1, n=1, 6000–12000 ticks, `max_nodes` 120 **and** 400; graded on **baked fraction**, not node count | **`REGRESSION` — and it is the cause of a false diagnosis, so read the shape.** Commissioning rescales the three velocity (`delta`) channels from ~5–12 % of the default span to 100 %. Those are per-tick joint differences at 60 Hz: small **and** noise-dominated. Giving the noise equal footing with position raised quantisation error everywhere, so nothing reached consistency and everything looked novel. Measured: **baked 43 % → 5 %**, nodes 58 → 154 and still climbing, `ema_tle` 0.214 → 0.265. ⚠ **A channel's RANGE says nothing about whether that range is signal or noise, and commissioning is blind to the difference by construction** — the same shape as "a more accurate sensor is not automatically a better input", wearing a new costume: *a more VISIBLE channel is not automatically a better input*. **Re-use context: the mechanism is sound and unit-proven (0.698 → 1.000 winner purity on a small signal that is REAL) — it is for SMART SENSORS whose channels genuinely need commissioning, and the operator intends to revisit it there. It is refuted for the picrawler motor stream specifically, where the small channel is mostly noise.** Would become re-testable here if the velocity channel is first shown to carry stride phase, or is denoised/averaged upstream |
| **GNG insertion-gate self-tuning** (`insertion_autotune`, gate = max(configured floor, 30th pct of own squared-TLE)) | corridor, seed 1, n=1, 6000–12000 ticks, `max_nodes` 120 and 400, **both with and without commissioning** | **`NULL` on the picrawler motor signal.** With commissioning: 5 % baked either way. Without: 40 % vs 43 % baked — no difference. The gate *was* live and *was* lifting (0.0234 vs the 0.020 floor), so this is not a dead-code or tautology null; a ~1.4× lift is simply not enough on this signal, where the unit test's 5.6× lift took baking 0 % → 96 %. **The mechanism is real and was genuinely lost in the v3→C++ port** (the Python reference self-tuned; only its frozen debug branch survived, while EPM.md went on documenting the auto-tuning as present) — restoring it is correct on its own merits and it is shipped opt-in, default-off. **Re-use context: it applies wherever the configured gate is mis-scaled against the signal's own error distribution — like commissioning, that is the smart-sensor case, to be revisited there.** ⚠ It was built to fix a problem that turned out to be the commissioning regression above, i.e. it was aimed at a manufactured target; see §4 for the missing-control error that produced it |
| **CPG-phase drive** (open-loop per-joint rhythm) | flat | Servo sequencer; chassis slams the ground ("flopping fish"). *Injected a rhythm instead of a prediction* |
| **Keyframe tween** (smooth the staircase) | flat | Low-passes out the adaptive slack → stiff |
| **hip2 stroke** (imposed femur lift) | flat | Energetic but chaotic; no stable gait, frequent falls |
| **hip2 tuck** (femur rest override, alone) | flat | Didn't crouch (weak reflex) + destabilized |
| **Learned hip2** (per-joint postural *profile* loosening the femur spring) | flat, long 60k A/B | No gait/traversal/disengagement gain, **+instability** (firm ~3× faster net traversal, 2× less stuck, 0 tipovers vs loose's 4). The self-rescue is an HK+embedding property on a *stable* base — hip2 was never its source |
| **Phase-indexed velocity objective** (`Cvel` propulsive pump on v*−ẋ) | flat | Marginal steady gain (Cvel self-limits: v* *is* the body's own velocity → error ~0 in steady state), then **amplified a rear-leg asymmetry into circling** (embed 6.1 m straight / 0.47 turns vs velobj ~1.89 turns). Not wrong in principle — incompatible with an *asymmetric* base gait |
| **LR velocity-symmetry prior** (the fix for the above) | flat | **Backfired** — circles worse (~3.19 turns). Magnitude-equalization over-drove the weak leg; yaw is signed/phased, not per-joint RMS. Fixed the wrong layer |
| **Gait symmetry** — amp-homeostat, exploration/coord-adapt, per-leg controller coupling (`ctrl_symmetry_gain`), walk-phasing, balance, heading-hold knobs | flat, **~35 isolated A/Bs** | **Every symmetry-forcing lever → circling.** The RR-under-plant is a stable emergent **tripod-skid** and that asymmetry is **load-bearing for straightness**. One variant hit 3 % knee-amp imbalance *while spinning −19 turns* ⇒ **amplitude symmetry ≠ functional symmetry** |
| **Cruse Rule 3 (contralateral load), isolated with Cruse LIVE** (`cruse_gain=0.5`, `cruse_rule3_weight` 0.5 → 0.0) | corridor + coordadapt base, n=3 seed-avg — **and confirmed by operator UI observation the same day** | **Rule 3 is the harmful part.** Turning it off while leaving Rule 1/2 live: net_z 1.95→**2.30**, straight 0.28→0.40, tilt_sd 0.217→**0.085**, falls 0.67→0.33, parasitic steps 167→129. Operator independently: "a few trot-like steps emerge, but it fights the natural motion; setting it to 0 traverses obstacles without regression." Consistent with the standing Cruse verdict — a second coordination controller fighting the emergent gait — and now localized to the *contralateral* rule specifically. **Caveat: this is MotorEPM's Rule 3, not CruseCoordinator's — see §4** |
| **Cruse / Walknet contact-load reflex** (`cruse_gain`, `cruse_rule5_gain`) | flat **and** re-tested on the incline | A second coordination controller firing **out of phase** — its own foot-height detector ≠ the emergent gait phase. Flat: worse everywhere, shatters the variance collapse, +falls. Incline (its supposed home): correct-signed `cruse_gain` final_z 2.54, Rule 5 2.05, both 1.87 — **all worse than the plain 4.13**. **Grip/lift is the wrong instinct: the belly must ride LOW to climb** (or use stance-lift). *This is the model refutation — killed in the regime where its premise applied* |
| **Forward-flow homeostat** (`forward_flow_gain`, amplify stroke ∝ magnitude·predictability) | flat | No distance; falls climb 0.75→1.38→2.75. The predictability term is oscillation-dominated → just raw destabilizing thrust |
| **Proprioceptive balance / `propbal`** | corridor, seed-avg | Refuted as a straightener |
| **★ TRUE foot-contact sensor as the swing gate** (`contact_topic=reality.proprio.foot_contact` — the physics touch flag, wired in place of the foot-height inference) | corridor + reward-free base, n=4, **plus a compensating `stance_lift` sweep {0.3, 0.35, 0.4, 0.5}** | **REFUTED — and it is the most counter-intuitive result in this ledger: the ACCURATE sensor produces a WORSE gait.** net_z 3.76→**2.37**, straight 0.70→**0.46**, tilt_sd 0.067→0.088 (belly clearance did improve, 0.027→0.030). First hypothesis was a confound rather than a defeat — true contact reports stance 77 % of the time vs the proxy's 59 %, so every consumer's *effective gain* rises — but the compensating sweep **kills that explanation**: 0.3→2.53, 0.35→2.10, 0.4→1.95, none anywhere near 3.76. **Diagnosis: the height-EMA "detector" was never measuring contact — it was measuring the foot's PHASE within its own cycle (above/below its moving average), and a phase-locked gate is what `stance_lift` actually needs.** "Push down on this foot" wants *is this foot in the propulsive part of its cycle*, not *is this foot touching*. The inaccurate proxy was accidentally the right signal. This is the same lesson the Cruse re-use context already predicted from the other direction — that its foot-height detector should be replaced by *the emergent gait's own phase*. **Kept default-off; its real value is as an INSTRUMENT** (ground truth for duty factor — it is what showed the world-height proxy under-reports swing by 16×) **and as the input a genuine load/Cruse consumer would need.** Re-use: for a consumer that truly needs contact (load distribution, step-over foot placement), not for anything that wants gait phase |
| **Phase readout moved to the stride joint** (`phase_joint=0`, hip1 instead of the legacy knee) | corridor + stance-lift base, n=4 × a **4-point `stroke_phase` sweep** covering the full circle {−2.85, −1.28, +0.29, +1.86} | **`REGRESSION`, and structurally so — not a tuning miss.** Locomotion collapses at *every* phase offset: net_z 3.62 → {0.83, 0.25, −0.14, 0.55}, flat speed → ~0.00, falls 0 → {2.5, 0.75, 2.0, 1.0}, tilt_sd 0.078 → {0.34, 0.18, 0.23, 0.13}. **Cause: the stroke and the phase readout cannot share a joint.** `L.phase = atan2(hip1 velocity, hip1 deviation)` while the stroke drives `y[0] += amp·sin(L.phase + stroke_phase)` on that same hip1 → a self-excited oscillator. Sweeping the full circle proves the feedback is fatal rather than mis-offset. **Note what it DID do: step-balance rose 0.30 → 0.41–0.58 and feet-planted rose** — locking coordination to the stride joint really does even out the legs, so the *premise* is intact; only this wiring of it is refuted. See §6 for the re-use context |
| **stuck→explore** | flat | Refuted — but note this was arguably **the wrong scenario for it**; a stuck-detector has no content on open flat ground. Its inverse twin (progress→commit) was promoted |
| **Lateral-sequence walk phasing** (`gait_phase=[π/2,3π/2,0,π]`) | flat — **but only ever measured inside the rejected open-loop `cpgwalk` config** | **Its +65 % distance is the SEQUENCER's speed, not a gait win — see the correction below.** The only config carrying this phasing is `the_picrawler_motor_epm_cpgwalk.json`, which also carries `rhythm_gains=[1.8,1.0,1.6]` and **no `cpg_embed`** — i.e. the open-loop CPG servo sequencer that was **REJECTED on UI observation** (chassis collision 15.6 % vs 3.3 %; "flopping fish"). It also relocated the asymmetry (RL skids), cost 2 falls, and was fragile to every knob |
| **Lateral-sequence phasing, now isolated on the `embed` base** (`gait_phase=[0,π,3π/2,π/2]` on the `[FL,FR,RL,RR]` order — the vector above, rotated to make leg 0 the reference) | corridor + stance-lift base, n=4 seed-avg — **the clean isolation the correction below asked for** | `NULL`/slight `REGRESSION`. The *mechanical* prediction held exactly — de-clustering the swing targets raised feet-planted **3.30 → 3.63** — but it bought nothing behavioral: net_z 3.62→**3.16** (and variance doubled, std 0.37→0.94), straight 0.67→**0.55**, belly 0.026→0.024, 0→**0.25** falls, flat speed **unchanged**. **Diagnosis: the static-stability argument was right and irrelevant.** Trot is dynamically stable and *should* be wobbly at this body's crawl speed — but the body was already averaging 3.3 of 4 feet planted, so there was almost no support-polygon headroom to win back. The phasing question is now **closed on this base**, not deferred |

| **★ LOAD-GATED POWER STROKE** (`stroke_load_gain` — scale each leg's propulsion by its share of measured hip1 load, so a leg pushes in proportion to the ground it actually has) | corridor + the deployed `..._imufused` base, n=4 seed-avg × a **5-point gain sweep {0.5, 1, 2, 4, 6}** at the standard 6 000 ticks | **`NULL` on progress, `PARTIAL` on leg participation. Not promoted.** No gain beats the baseline on distance or straightness: net_z 4.75 → {4.74, 4.33, 4.52, 4.24, 4.47}, straight 0.74 → {0.74, 0.71, 0.72, 0.69, 0.70}, and **`flat_v` stays pinned at 0.04–0.05 — the ninth lever to leave it there** (§5). 0 falls in every arm. The one real movement is **`step_bal` 0.44 ± 0.16 → 0.54 ± 0.08 at gain 2** (the legs do share the gait more evenly, and the variance halves) — but it costs `steps` 53 → 43.5 and buys no progress. **The consumer demonstrably fired**: gate spread scales monotonically 0.28 → 1.87 across the 6.6× gain range while `stroke_gate_mean` holds at 1.00 (the gate is mean-normalized, so it redistributes thrust rather than attenuating it). **Why the null is believable rather than a measurement failure: it is what the mechanism predicts.** A purchase gate removes thrust spent in the air, but *during stance* it scales the push and drag halves of the stroke equally — and the boxed finding above measured that balance at 50/50. A magnitude gate cannot fix a timing problem. Built on the honest signal the ledger asked for (§6 named `joint_torque` as the load observation Walknet never had) and on the joint that measurement — not assumption — picked |

> ### ⚠️ Correction (2026-07-25) — the "+65 % walk phasing" number does not mean what it looks like
>
> Earlier versions of this ledger, and the closing line of
> [`picrawler_gait_loop_findings.md`](picrawler_gait_loop_findings.md) §"Gait symmetry", framed
> lateral-sequence walk phasing as a **banked speed lever awaiting stability**. That is wrong, and the
> operator caught it from memory: the gait it produced *looked sequenced and paddle-like, not alive.*
>
> **What actually happened.** In that report, "walk" means `the_picrawler_motor_epm_cpgwalk.json`
> throughout (§"Final tuned gait" names it). That config drives **every joint open-loop from the CPG
> clock**, ignoring ground contact. It was **rejected on UI observation** as a servo sequencer that slams
> the chassis into the ground every step. The +65 % distance was measured on *that* body. The symmetry
> sweep, written a day after the rejection, carried the number forward without re-flagging its source.
>
> **This is a textbook instance of two doctrine failures at once:** a **blind metric** (distance certified
> a degenerate paddling behavior — exactly what
> [`../operational/aliveness_metric_protocol.md`](../operational/aliveness_metric_protocol.md) warns
> distance metrics do), and **fast ≠ walking** (doctrine §8; the collision was invisible until
> `chassis_h` was added). The speed was real. It was the speed of the wrong thing.
>
> **What is genuinely open.** `gait_phase=[π/2,3π/2,0,π]` has **never been tested on the emergent
> `embed` base** — no config combines the two, so phasing has never been isolated from the open-loop
> drive it shipped with. That is a clean, cheap experiment and a legitimate open question. It is *not*
> a +65 % result waiting to be unlocked; any retry starts from zero and must be judged on chassis
> height, belly clearance, and adaptation — not distance.

---

### ✓ 2026-07-25 — THE GOD'S-EYE SWING SIGNAL IS GONE, replaced by something the real robot can build

**RESOLVED.** `feet_y` (absolute world-Y) is no longer the swing input. The replacement is
**`feet_y_gravity_cmd` = forward kinematics from the COMMANDED servo angles, projected onto
the accelerometer's gravity-up axis** — IK ⊕ IMU, and buildable on the physical PiCrawler,
whose hobby servos report no position at all.

| swing input | legal? | on hardware? | net_z | straight | hump final_z |
|---|---|---|---|---|---|
| `feet_y` — world-Y **oracle** | ✗ | ✗ | 3.76 ± 0.40 | 0.70 | 5.21 ± 0.67 |
| `feet_y_body` — chassis-frame FK | ✓ | ✓ | 2.52 ± 1.34 | 0.50 | — |
| `feet_y_ground` — FK + belly ToF | ✓ | ✓ | 2.26 ± 1.65 | 0.46 | — |
| `foot_contact` — true touch flag | ✓ | **✗ (no foot switches)** | 2.37 ± 0.86 | 0.46 | — |
| `feet_y_gravity` — FK from **achieved** pose | ✓ | ✗ (needs encoders) | 3.95 ± 0.56 | 0.62 | 5.35 ± 1.15 |
| **`feet_y_gravity_cmd`** — FK from **commanded** angles | ✓ | **✓** | **4.36 ± 0.28** | **0.74 ± 0.01** | **6.10 ± 0.46** |

**The property that mattered was a GRAVITY REFERENCE.** Three legal candidates lost badly and
two phase hypotheses died at chance agreement (0.49–0.56); all of them lacked it, and an
accelerometer supplies it directly. **And the hardware's poorer information proved BETTER**:
servo deflection (measured 22 mm mean / 38 mm max at the foot; FK chain validated to 1.1 mm)
is *noise* to the gate, so the commanded angles — the clean intended trajectory — give tighter
straightness (± 0.01 across seeds) and halved distance variance. Full analysis:
[`sensor_legitimacy_and_the_feet_y_oracle.md`](../plans-and-designs/sensor_legitimacy_and_the_feet_y_oracle.md).

**2026-07-26 — the attitude gap is closed too, and the result improved again.**
`feet_y_gravity_cmd_imu` uses a MODELLED accelerometer + gyro complementary filter (no exact
simulator attitude anywhere), sampled and filtered at the **240 Hz physics rate** rather than
the 50 Hz brain tick — real IMUs run 1–8 kHz, and the physics was already oversampled to stop
foot tunnelling, so that bandwidth was free. Measured attitude error **3.17° mean / 9.2° max**
(accel-only: 21.3°). n=4 vs the oracle: net_z 3.76→**4.75**, flat_v 0.04→0.05, straight
0.70→0.74, **tilt_sd 0.067→0.068 (wobble gap CLOSED)**, hump 5.21→**6.09**, 0 falls. It beats
even the exact-attitude arm (4.36) — the third time a hardware-honest model beat its idealized
twin, each of which happens to include smoothing.

**ROBUSTNESS MATRIX (`robustavg.py`, 8 sensors × 2 actuation backends): every
gravity-referenced signal emerges in BOTH backends, 8/8 — and the god's-eye oracle does NOT
(it falls on g6dof).** Overall emergence 10/16, net_z spread 2.4×. **A gravity reference is
the invariant; the substrate is robust in kind, sensitive in degree.** Full matrix in the
design note §6.

<details><summary>Historical: the entry as written when this was still open</summary>

#### ⚠️ THE DEPLOYED GAIT DEPENDS ON A GOD'S-EYE SIGNAL, and no legal substitute recovers it

**`reality.proprio.feet_y` is absolute WORLD-Y** (`_lowers[i].global_transform.origin.y −
L3*0.5`, picrawler_body.gd) — a quantity no physical picrawler can sense, and **the same
violation that retired `chassis_y_norm`** (§3). It is the input to MotorEPM's swing detector,
which gates `stance_lift` and every Cruse rule. It has been there the whole time and was not
caught until the operator asked where the brain thinks its feet are.

The detector partially launders it — `foot_y > foot_y_ema` is a *difference* of world-Y
values ~50 ticks apart, which on flat ground with a steady chassis ≈ the foot's motion
relative to the body (legitimate proprioception). But the difference still contains the
body's own vertical drift, so it is contaminated exactly where terrain matters. And
`feet_y < stance_y_threshold` (the body's "planted" test) is **doubly** god's-eye: absolute
world height against a fixed constant.

**Three Markov-compliant replacements were built and seed-averaged (n=4). All lose, by a
similar margin, with 2–4× the variance:**

| swing-detector input | legal? | net_z | straight | tilt_sd |
|---|---|---|---|---|
| `feet_y` — absolute world-Y | **✗** | **3.76 ± 0.40** | **0.70** | **0.067** |
| `feet_y_body` — foot pose vs chassis (encoder FK) | ✓ | 2.52 ± 1.34 | 0.50 | 0.095 |
| `feet_y_ground` — FK + belly ToF (terrain-relative) | ✓ | 2.26 ± 1.65 | 0.46 | 0.101 |
| `foot_contact` — true physics touch flag | ✓ | 2.37 ± 0.86 | 0.46 | 0.088 |

`feet_y_body` is the cleanest experiment: **identical formula, identical toe approximation,
only the reference frame changed** — so the delta isolates the god's-eye component and
nothing else. **It is load-bearing.**

**Best available reading (NOT verified):** world-Y and body-relative differ by exactly the
chassis's own vertical motion, so what the illegal signal carries is the **body's bounce** —
a whole-body vertical phase reference — and gating a knee push on that syncs it with when
the legs are actually being loaded. The `feet_y_ground` arm was built to reconstruct that
legally (belly ToF supplies body height) and **did not recover it**, which either falsifies
the reading or reflects the ToF being a noisier, single-point, short-range substitute for
true chassis height. Untried and the most promising legal source of body-bounce phase: the
**IMU's vertical acceleration**, which is a real sensor and needs no differencing of a range
finder.

</details>

**Status when written: an OPEN LEGITIMACY PROBLEM (now resolved above).** Options, all requiring an operator
call: (a) accept the dependency and state it in every claim; (b) keep hunting a legal
reconstruction (IMU vertical phase is next); (c) re-tune the consumers *on* a legal signal —
note every arm above ran `stance_lift=0.5`, tuned for the god's-eye signal, and the high
variance suggests instability rather than a simple offset. **Until resolved, any "the gait
works" claim should carry the caveat that its swing gate is not physically realizable.**

---

### ★ 2026-07-26 — CATASTROPHIC FORGETTING AFTER INVERSION WAS AN INTEGRATOR-WINDUP BUG

**Operator observation:** placed on angled/complex surfaces the robot flips onto its back,
lies tucked, then homeokinesis breaks the lock and it **self-rights** (~27 s measured, ~1 min
observed) — a large emergent win. But afterwards the walk never recovers: exaggerated
movements, wrong phasing, no forward traversal.

**Measured** (`forgetavg.py`, deterministic flip via the pre-existing
`OGMA_PICRAWLER_TELEPORT_FLIP=1`, `AUTO_RESET_INVERSION=0`): after self-righting, forward
progress stayed **NEGATIVE for 7200+ ticks** against a `+0.892` pre-flip baseline, with **zero
recovery trend** across six 1200-tick bins.

**Per-variable attribution — and it overturned the expected diagnosis:**

| variable | pre-flip | after righting | verdict |
|---|---|---|---|
| `chassis_h_max` | 0.999 | 0.999→1.000 | **not the mechanism** — the height signal is clamped to [0,1] so the "monotonic max" is already saturated and cannot ratchet |
| **`height_bias`** | −0.191 | **−0.500, all six bins** | **LATCHED at its clamp** |
| **`amp_gain`** | 0.100 | **2.9–4.3, never returns** | **LATCHED 30–40× high** ⇒ "movements are exaggerated" |
| `coord_best_fitness` | 0.336 | decays normally | not implicated |
| `motor_tle` | 0.278 | returns to band | **HK self-model re-adapts fine** |

**The learned structures recover; only the two homeostat INTEGRATORS latch.** So this is not
the "monolithic weights get overwritten, we need capacity allocation" problem it was first
framed as — it is two unbounded integrators accumulating in a regime where their setpoint is
meaningless, then being unable to unwind. Far smaller and entirely local.

**The general shape, worth keeping:** *an integrator must not accumulate where its error
signal is invalid, and it must be able to unwind anywhere it can wind.* `height_bias` violated
both — it integrates while inverted, and `height_rest_frac` (the incline windup fix) then
freezes it while the robot walks, so **the more it tried to walk the longer it stayed broken.**

> #### ⚠️ CORRECTION (same day) — the gate is NOT promoted: it may disable the escape
>
> Follow-up at n=8 on an angled surface: **the gated arm self-righted 0/8, the ungated arm
> 1/8.** 1-vs-0 is not significant alone, but the mechanism is specific and strong:
> **`amp_gain` winding 0.100 → 4.0 is a 40× amplitude escalation, and that violent flailing
> is WHAT RIGHTS THE ROBOT.** Freezing the integrator removes the escape drive. The poison
> and the capability are the same mechanism — so the gate protects the walk by lesioning a
> working loop, exactly what CLAUDE.md §5.4 forbids.
>
> **Reverted:** the launcher CURRENT is back to `..._imufused.json` (the forgetting flaw
> documented and unfixed) and `..._uprightgate.json` is parked as an UNRESOLVED TRADE.
> `homeo_upright_gate` stays in the code default-off.
>
> **The redesign the evidence points to — intervene on the TRANSITION, not the state:**
> *snapshot the homeostat integrators when uprightness is lost and restore them when it is
> regained*, making the inverted excursion scratch space. Full wind allowed (escape
> untouched), posture restored exactly rather than slowly unwound. That is "plastic" in the
> operator's sense; a freeze is the opposite.
>
> **Also — a power fix for any retest.** Self-righting runs ~10–30 %, so n=8 buys one event.
> `_teleport_to` applies a RELATIVE 180° rotation, so with `TELEPORT_EVERY` a second flip
> turns an inverted robot upright again — making recovery deterministic and giving full n on
> the post-recovery walk. That separates "does the fix restore the walk" (fully powered) from
> "does the fix preserve self-righting" (stochastic, needs its own larger run).
>
> **What the ungated n=8 curve added:** the latch does eventually release — `height_bias` sat
> at −0.500 for six bins then crept to −0.256 by 12 000 ticks — and progress partially
> recovers (+31 % of baseline at ~7 000 ticks) before collapsing to −63 % and drifting back
> to +14 %. So it is a ~200 s erratic degradation, not a permanent flatline.

**Mechanism fix (measured, but see the correction above) — `homeo_upright_gate=0.5` on `reality.proprio.upright`:** Normal walking
is byte-identical (net_z 4.75, flat_v 0.05, straight 0.74, tilt_sd 0.068, 0 falls) because
`upright` stays ~1.0 while walking so the gate never engages; inverted, `height_bias` holds
(−0.093 vs +1.497) and `amp_gain` **does not move at all** (0.100 → 0.100).

**Two process notes.** (1) The gate was initially wired to `tilt_topic` and was **silent dead
code** — the body's `publish_tilt` defaults FALSE so tilt never arrives headless and `upright_`
sat at its 1.0 init. Caught only by checking that the consumer fired (`amp_gain` wound
*identically* gate-on and gate-off). §3.2 rule 5, again. (2) `height_unwind_free` (asymmetric
windup fade, so a railed bias can unwind while moving) was also built and is **NOT promoted**:
it addresses recovery from a wind the gate prevents, and it cost ~8 % net_z and ~12 % hump.
Kept default-off as infrastructure.

**Also learned:** inverted on **flat** ground the robot never self-rights (0/3 seeds, 16 000
ticks) and `motor_tle` **falls** 0.24→0.08 — inverted-on-flat is a *low-surprise attractor*, so
homeokinesis has no pressure to escape. The self-righting win depends on the terrain's ongoing
disturbance keeping TLE high. **Still unmeasured at power:** the end-to-end post-righting
recovery curve, since self-righting is stochastic (1 of 3 seeds even on an angled surface).

---

### 2026-07-26 — the PLASTICITY / FORGETTING family (nothing promoted; `homeo_leak_cycles=0`)

The forgetting-after-inversion failure is an **integrator-windup** bug, not a learned-weights
one (see the entry above). Four mechanisms were built and measured. **None is promoted** — at
`homeo_leak_cycles=0` the stack performs as well as it ever has, and the operator's read is
that recovery does happen after a stuck period, just slower than wanted.

| Lever | Verdict |
|---|---|
| **`homeo_upright_gate`** — freeze the integrators while not upright | **`REGRESSION`.** Stops the latch (amp_gain 2.53→0.100, height_bias +1.50→−0.09) and normal walking is byte-identical — but self-righted **0/8 vs 1/8** ungated. **`amp_gain` winding 40× IS the flail amplitude that rights the robot**, so freezing it lesions the escape (§5.4). Do not use |
| **`height_unwind_free`** — asymmetric windup fade so a railed bias can unwind while moving | **`NULL`/`REGRESSION`.** Addresses recovery from a wind the leak/gate prevents anyway, and cost ~8 % net_z and ~12 % hump. Default-off infra |
| **`homeo_leak_cycles`** — leaky homeostats, rate in stride cycles off the body's own ω | **`PARTIAL`.** At 5 cycles FREE on flat (net_z 4.78 ± 0.16 vs 4.75; tilt_sd 0.068 identical) and bounds the amp_gain excursion by half (2.53→1.18). **But a CONSTANT leak blocks escape entirely: 0/4 off the operator's wall vs 2/4 with the leak off.** 2 cycles is too aggressive. Target matters — leaking toward *unity* (semantically neutral for a gain) cost net_z 4.75→3.6 because normal walking pins the integrator at its FLOOR; retargeted to minimum authority. **Semantically neutral ≠ behaviourally neutral** |
| **`homeo_leak_upright_only`** — posture-gate the leak: forget while upright, accumulate while not | **Best variant, still not promoted.** Escape restored (**2/4 @ 72 s** vs baseline 2/4 @ 96 s), free on flat (4.78, tilt_sd 0.068, 0 falls), hump 5.68 vs 6.09 (−7 %). The **inverse** of `homeo_upright_gate` — which is exactly why it works: accumulate while inverted (escape), forget once upright (fast recovery) |
| **`homeo_leak_progress_gate`** — also stop forgetting while stalled | **`REGRESSION`, two attempts.** Second operator observation: upright but BLOCKED at a slanted wall, the robot "no longer learns how to climb, it stays in the same gait it used on the flat" — a posture gate cannot catch that. Gating *both* integrators on stall cost hump 6.09→4.84 (stalling on a hill readmits the height windup). Gating *only* amp_gain: net_z 4.33, **falls 0.25**, hump 4.90, escape 1/3 @ 228 s. **Cause: `height_rest_frac` is INSTANTANEOUS progress, so it fires on ordinary gait pauses.** The signal wanted is a SUSTAINED stall — `stuck_ticks_`/`stuck_boost_` (~5 s) — which must first be decoupled from `stuck_explore_gain` |

**Two principles earned here.** (1) *Forgetting is a luxury of success; escalation is the
response to failure* — the wind-up is not only damage, it is how the body gets out of trouble,
so any mechanism that bounds it must not bound it while the body is failing. (2) **The leak had
to become per-integrator**, because `amp_gain` is EFFORT (may escalate when failing) and
`height_bias` is POSTURE BIAS (must never accumulate on a slope — that is the refuted windup).
*Which state may escalate depends on what that state means.*

**Open:** the upright-but-blocked case. Two failed attempts; needs the sustained-stall signal.

---

## 3. Substrate decisions (retired, with reasons)

- **God's-eye `chassis_y_norm` → RETIRED** as the height observation. Absolute world-Y reads
  ~1.0 on a raised hump, i.e. **blind to belly grounding**, so it cannot represent terrain
  at all (fixed god's-eye incline final_z 2.84, stalls). It is fine on flat and *only* on
  flat. Replaced by the belly ToF rangefinder — **Markov-blanket compliance and the hump fix
  turned out to be the same move.**
- **Active-balance reflex** — was **inert by default headless** (`publish_tilt` @export
  defaults false and the brain config doesn't set the body export → tilt=0 at MotorEPM), and
  *destabilizing* when enabled (`OGMA_PICRAWLER_PUBLISH_TILT=1` maps tilt→hip2 = pitch/roll
  leveling, not yaw; both signs circle). **Known measurement gap: headless ≠ UI here.**
- **`heading_gain` anti-yaw** — erratic (circled `embed` even at 0.4). Superseded by the
  bearing-hold P+D through the authoritative channel.

---

## 4. Bugs found (fixed — keep in mind as failure shapes)

| Bug | Shape |
|---|---|
| **★ `seedavg.py` does not set `OGMA_PICRAWLER_CHASSIS_COLLIDE` (2026-08-06)** | It inherits `os.environ` (`seedavg.py:29`) but never sets the flag, and `chassis_collides` defaults **false** — so **every `seedavg` A/B runs on the GHOST chassis**, a body whose belly cannot touch the ground. A belly-clearance lever measured this way showed net_z 4.31→3.44, a seed-3 collapse and falls 0→0.33; re-measured on a **solid** chassis at the same gain it is net_z 4.32→4.50 with no collapse. **The 2026-08-04 ghost finding warned that every belly CLAIM was measured on a ghost; this is the same trap entered from the other side — a belly lever REFUTED on one.** Rule: any lever whose mechanism involves belly/chassis contact must export `OGMA_PICRAWLER_CHASSIS_COLLIDE=1`, and a run summary that omits it describes a different body |
| **★★★★ THE CRASH THAT MANUFACTURED THE BASIN LOTTERY — every "shuffler" today was a corpse (2026-08-09, supersedes the entry below)** | The coordination probe constructs `normal_distribution(0, σ)` with σ = `coord_reward_drive × explore_mult`, and full progress-commit drives `explore_mult` to EXACTLY 0 (explore_floor 0) → σ = 0 → **Ubuntu's hardened libstdc++ asserts and ABORTS THE PROCESS**. Sustained progress saturates commit, so **runs were killed at precisely the moment the body committed to walking** — the better the walk, the more certain the death. The assert armed TODAY: the first rebuild of `MotorEPMv2.o` since the toolchain hardening (the Aug-7 binary predated it; unhardened `normal(0,0)` draws 0 = "propose the incumbent", benign). `seedavg` then scored every corpse as a completed run — **19/20 tie-run seeds died at t=960–8400 with the assertion in their logs; "net_z 1.45" was where the body stood when it died.** Verified three independent ways: per-seed log truncation + assert lines; pre-fix rebuild reproduces the 19/20 crash pattern; post-fix build walks 6/6 at net_z ~6.5 ≈ the Aug-7 baselines. **CONSEQUENTLY RETRACTED: the basin-lottery entry below (kept for the record); the walk-fraction crisis (2–5/20 — those were crashes); the criterion multi-axis refutation (§2 — the 78-run sweep ran on the armed build, re-measure before trusting); the release dose-curve conversion story (2/20→11/20 — confounded by crash timing; clean re-measure in flight); the "logging cadence changes trajectories" claim (both compared runs were crashes; determinism holds).** STILL STANDING: every code-structural audit finding; the recovery-shear trace physics; P0's hygiene (both tie arms crashed identically — the tie holds); P1's phase scoring (collected post-fix on full walking runs). **Rules, each now enforced in tooling: (1) a parser must verify run COMPLETION before scoring (seedavg now excludes truncated runs loudly); (2) after ANY build-state detour, rebuild before the next measurement — the .so on disk is part of the arm; (3) when a toolchain hardens, the first rebuild is a new context: re-baseline before comparing to history** |
| **★★★ THE BASELINE DID NOT SURVIVE THE ENVIRONMENT — the walk was a basin lottery (2026-08-09)** ⚠ **SUPERSEDED by the entry above — the "basin" was crash truncation** | The stancehip2+supportepm reference (net_z 4.56, n=6) reads **1.45 ± 0.54** at the exact recorded protocol, and the cause is BOUNDED, not found: code/configs/Godot 4.6.2/machine/boot/seeds all verified identical (full-tree git diff to the session-close commit is empty; same uptime since 08-06; runs are bit-reproducible today, seed-for-seed identical across two harnesses), and the new criterion instrument was isolated by a same-build-dir revert A/B — seed 1 posts net_z 1.09 on both builds, behaviorally byte-equal. What cannot be reconstructed is the 08-07 working tree's uncommitted state and its exact .so (untracked, since overwritten). Difficulty is NOT the cause (5/6 seeds identical at diff 0.0 — the stall is terrain-independent); chassis mode is not (ghost: 1.85 ± 0.94). **The lesson with teeth: an n=6 fixed-seed walking number records which BASIN six seeds landed in, and basin assignment did not survive an FP-level perturbation of the environment. Seeds ARE FP-level perturbations, so the corridor baselines were basin measurements wearing capability clothes.** Rules: (1) a walking-stack claim now requires the walk FRACTION at n≥20 (2026-08-09 truth: **4–5/20** on `..._imufused`, walkers at 48–112 real steps, shufflers at 0–12); (2) every ledger entry must record steps/difficulty/chassis/n — several reference entries recorded none of these, which is why this non-reproduction cannot be assigned |
| **★★★ "FEWER FEET PLANTED" WAS A SYMPTOM, NOT A CAUSE (2026-08-07)** | The correlational read was loud: 2 feet planted ↔ fwd_v +0.0996, 4 feet ↔ +0.0116, an 8× spread, and the body sits at 3–4 planted 85 % of the time. I proposed stance overlap as **the** target. **A causal test refutes it.** `amp_target` HAS authority over stance overlap (0.40 → 0.70 drives mean planted 3.23 → 2.14, %≤2-down 15 % → 63 %, t = −4.87) — **and transport collapses with it: net_z 4.67 → 0.61 (t = −14.17), straight 0.71 → 0.21, tilt_sd 0.089 → 0.901 (10×).** The body has two feet down **when it is striding well**; it does not stride well *because* two feet are down. **Rule: an actuator with authority is necessary but not sufficient — a correlational target can be a symptom, and the causal test is the only thing that separates them.** ⇒ The stance-overlap target is retired; the responsiveness measurement that motivated it stands, but its interpretation does not |
| **★★ THE HOMEOKINETIC CRITERION SURVIVES A TEST IT NEVER SAW (2026-08-07)** | Same amplitude sweep, scored by `value = responsiveness/(motor_tle+ε)` — a purely egocentric quantity that never observes forward progress: amp 0.30 → 1.46, **0.40 → 1.47**, 0.55 → 1.34, **0.70 → 1.24**, against net_z 4.53 / 4.67 / 2.01 / 0.61. **`corr(value, net_z) = +0.996`.** It ranks the fewest-feet-down arm LOWEST — correctly — and it is *not* a restatement of "fewer feet is better", which is what makes it more than the target it replaced. Mechanism: responsiveness itself saturates at high amplitude (0.4166 → 0.3663) because the commanded change grows faster than the sensory change. **This is the strongest evidence in the campaign that an intrinsic, §5.1-legal criterion can rank gaits the way transport does, without being told about transport.** Still unattached to any actuator with authority — that remains the open problem |
| **★★ THREE LEVERS IN ONE SESSION AIMED AT AN ACTUATOR WITH NO AUTHORITY (2026-08-07)** | `gait_phase` → footfall timing (commanded diagonal offset spans 24 ticks, realized spans 4 — the command varies **6× more** than the timing it sets); `height_lift_knee` → hip2/knee coherence during locomotion (the height path is multiplied by `height_rest_frac`, which is **zero while cruising**); `support_select_gain` → stance overlap (`corr(explore_mult, feet planted)` ≈ **0**). All three were correctly built, correctly signed, and **verified firing** — and all three were incapable of moving their target for reasons measurable in ten minutes beforehand. **RULE: before building a lever, measure whether the actuator has AUTHORITY over the target variable — `corr(actuator, target)` on existing traces. A consumer-fired check proves the code runs; an authority check proves it can matter.** The two are different, and this session needed both |
| **★ A gate read taken only on the CHANGED arm (2026-08-06)** — the missing-control error, in its most expensive form | The motor-layer GNG was measured as "grows unbounded, ~5 % baked, flat TLE" and diagnosed as a **broken insertion gate**. A whole mechanism was then designed, built and tested against that diagnosis. The gate read had **no un-commissioned control** — and when one was finally run, the pre-commissioning EPM baked **43 %** and sat at ~58 nodes. *The pathology was the commissioning change itself*, so the diagnosis described a self-inflicted regression and the fix was aimed at a manufactured target. **Three compounding causes worth separating: (1) no control arm on a diagnostic read — CLAUDE.md §3.2 check 4 applies to DIAGNOSES, not only to A/Bs; (2) the probe could not see `baked` at the time, so the one metric that would have exposed it was unavailable — and "the instrument cannot see it" silently became "it is not happening"; (3) node count was trusted three separate times in one session and was blind every time** (blind to §0 rule-2 collapse in the acceptance test, blind here because baked nodes are frozen and immune to pruning, so the REGRESSION raised node count). **Rule: before diagnosing the substrate, re-measure the last thing YOU changed with it turned off** |
| **Cruse `cruse_gain` sign** | Backward at +, forward at − (the emergent gait's foot-height swing is **anti-phase** with the canonical Cruse assumption). Seed-avg: +0.5 → net_z +0.26 (backs up); −0.5 → +2.57 clean forward. Fixed for hygiene; gains remain off |
| **Execution guards `> 0.0`** | Silently disabled **negative** gains — widening a parameter's bound is not enough if the guard rejects the sign. Changed to `!= 0.0` (3 sites) |
| **`postural_gain_joints` as absolute override** | Made the global UI scalar a **silent no-op** — and the robot's behavior improved right afterward by natural evolution, which *looked* causal. **Hand-tuning manufactures false causation; isolate before promoting.** Fixed to a multiplier + `postural_eff` diag so a knob-turn can never silently do nothing |
| **`master_seed` override rewrote 0 modules** | `OGMA_SEED` did nothing; every run byte-identical → **no A/B could be seed-averaged**. Fixed 2026-07-23 (the override now rewrites `seed` too) |
| **★ `coord_reward_drive=0.3` was live in the ENTIRE config lineage — including the `embed` milestone** (found 2026-07-25 by `mkarm.py`'s tautology guard refusing to write a "no-op" arm) | It is a **(1+1) hill-climb that KEEPS the highest-fitness probe and reverts to it** (`coord_best_phase_`), fitness = `fwd_v` minus wobble/lateral penalties, incumbent score decaying only `0.99` per 240-tick window (**τ ≈ 400 s**). Three consequences. **(1) It is a reward.** Probe → score by forward velocity → keep the winner. That sits in direct tension with §5.1 ("no reward shaping, ever — intrinsic homeostatic valence only"), and it has been on since the milestone, so *every* result in this ledger was measured with it running. **(2) It is structurally the destructive-crystallization shape** the operator identified from experience: a violent escape thrash that momentarily scores high `fwd_v` becomes the incumbent, and subsequent normal probes score lower and are reverted *back* to the thrash. **(3) It overwrites `gait_phase` wholesale every 240 ticks** from `coord_best_phase_` + noise — so `coord_adapt_rate`'s slow nudges are discarded each window unless a probe happens to win. **This invalidates the stated RATIONALE for the coord_adapt promotion** (it was never replacing an "imposed" topology — the topology was already being reward-searched); the A/B itself stands, since the ratchet was equally on in every arm. **Ratchet-OFF A/B (n=4):** it is *load-bearing for distance* — stancelift net_z 3.62→**2.90**, step_bal 0.30→**0.11**; coordadapt net_z 3.52→**2.36**, step_bal 0.25→**0.03** (a leg stops participating) — while *improving* wobble (tilt_sd 0.078→0.066). So a reward-driven search is producing a real share of the forward progress and nearly all the leg participation in a stack that is supposed to be reward-free. **Unresolved: this needs an operator decision, not a unilateral fix** |
| **TWO different Rule-3 parameters, in two different modules** (2026-07-25) | `MotorEPM::cruse_rule3_weight` is a **sub-weight inside the `cruse_gain` block** (single use site, MotorEPM.cpp:1503) — **inert whenever `cruse_gain == 0`, which is its default** — yet its own default is **0.5**, so the MOTOR-EPM panel *displays it as enabled while it does nothing*. Separately, `CruseCoordinator` has its own `rule3_weight`, gated by `cruse_bias_gain` whose default is **1.0 = ON**. The MOTOR-EPM panel slider writes the FORMER (`motor_epm_panel.gd:208` → `set_param` on `motor_epm`), so **zeroing MotorEPM's `cruse_gain` says nothing whatsoever about CruseCoordinator's Rule 3.** Verified by measurement, not inference: with `cruse_rule3_weight` at its **max (2.0)** and `cruse_gain=0`, MotorEPM's contribution is **exactly 0.00000** and `swing_frac` is bit-identical to the default arm; at `cruse_gain=0.3` it is 0.358. Instrumented with a `cruse_bias` diag (mean \|MotorEPM's own Cruse contribution\|, in diag + snapshot + the body's stdout JSON) so this is answerable by a number. **Shape: the same rule name in two modules with opposite default gating is the `postural_gain_joints` silent-no-op trap wearing a second module.** Consequence for the record: **any historical Cruse verdict must name WHICH module it tested** |
| **Height setpoint slammed the integrator** | `height_k=0.65` (a flat-ground memory) at the tucked-spawn low clearance → 3 startup flips. Lowered to 0.30 |
| **Reset artifact** | Auto-reset teleports fired no bus event, and MotorEPM's leg-phase/EMA survived fall+respawn → **any coherence/TLE trend across a reset was fake**. Fixed by publishing `events.reset` + reset-masking (Gate 0) |
| **★ THE CORRIDOR GYM RUNS OUT AT ~9.5 m, AND A LONG RUN CHARGES THE FASTEST ARM A `fall` FOR IT** (2026-07-27) | `_build_corridor()` lays a **9.5 m** curriculum on a **20×20** floor (`picrawler_body.gd:2945`, `:2665`), so the walkable strip ends near z=9.5 and the world ends at z=10. At **12 000** ticks a fast arm reaches it: in the load-stroke gain-2 sweep, seed 1 posted **the best distance of the entire campaign (net_z 10.04, max_z 10.06) while its mean `chassis_y` was −39.29** — it walked off the floor and kept falling, and `falls` and `chassis_y` both recorded that as a gait failure. **The bias runs the wrong way: it hits the FASTEST arm first, i.e. exactly the arm a propulsion lever exists to demonstrate, so a long run systematically penalizes levers that work.** Re-run clean at 6 000 ticks, the same arm shows 0 falls and no distance gain at all — so the 12 000-tick reading would have recorded BOTH a false positive (net_z +10 %) and a false negative (0.5 falls) on the same lever. This is §3.2 rule 7 (silent confound) and also explains why the standard protocol is 6 000 ticks — that is the run length that fits the gym, not an arbitrary choice. **Fixed in the harness:** `seedavg.py` now prints a loud `GYM-BOUNDARY WARNING` naming every seed whose `max_z` passes `SEEDAVG_SAFE_Z` (default 8.5) and states that `falls`/`chassis_y` are untrustworthy for those seeds |
| **Swing detector inferred contact from height instead of using the contact sensor that already existed** (2026-07-25; magnitude CORRECTED below) | `bool sw = foot_y_[i] > foot_y_ema_[i]` — no deadband. **Measured against the TRUE physics foot-contact sensor on identical runs: the detector reads 0.408 swing vs a true 0.229 — over-reporting by ~1.8×.** ⚠️ This row first claimed "40.3 % vs 0.7 %, wrong by ~50×"; that 0.7 % came from a **world-height** proxy which itself under-reports swing by ~16×, so the 50× figure was an artifact of comparing one bad proxy against another. **The real fix was not a deadband at all: `reality.proprio.foot_contact` (a physics touch flag, and the sensor a real picrawler has) was already published every tick and simply never wired into MotorEPM** — now available as `contact_topic`. Historical detail retained: | It then closes a feedback loop with any consumer that moves the foot (`stance_lift`, Cruse): bias lifts the foot above its EMA → declared swing → bias removed → foot drops → declared stance → bias returns. Cost scales with the consumer's gain, which is the measured `stance_lift` sweep: steps 50→84→**147** across gain 0→0.5→0.8 with **no** speed gain, tilt_sd 0.066→0.078→**0.178**, falls 0→0→**0.75**. Mitigated behind `swing_hyst_frac` (MAD-scaled deadband, default 0 = legacy; guard verified byte-identical to the per-seed digit). At gain 0.8 a 0.5 band halves the parasitic lifts (147→73), halves the wobble (0.178→0.089) and removes the falls. **⚠️ CORRECTED after unit testing — this is CONDITIONAL, and the condition is the absence of stepping.** Given a *real* duty cycle (80 planted + 20 lift) the legacy detector is essentially correct (0.18 measured vs 0.20 true): the lift excursions pull the EMA up, so the stance phase sits decisively below it and there is nothing to chatter on. And a band of 1.0·MAD in that regime is too wide — the stance deviation never clears −band, so the detector **latches** (0.58), which is the failure mode behind the live `frac=2.0` arm degrading after 0.5 helped. **So the detector is an AMPLIFIER of the no-stepping problem (§5), not an independent root cause.** Two shapes to remember: **a self-referential threshold is not a sensor, and any bias that moves what it measures will ring it** — and **a scale-invariant deadband cannot separate jitter from a step by amplitude** (a sinusoid has peak/MAD ≈ 1.57 at any size), so answering "is this foot loaded" needs a load observation, which the bus does not have |

| **★★ TWO LATENT TEST/RUNTIME DEFECTS FOUND WHILE GATING THE GAINEVOLVER (2026-08-17) — both PRE-EXISTING, attributed on a clean worktree** | A full `ctest` during the PART IV gate showed **20 failures**. Attribution was done properly instead of assumed: a detached worktree at `HEAD` (8b2fb2f, **none** of the GainEvolver code present) reproduces **both clusters identically**, so none belong to this build (`test_gain_evolver` 12/12 and `test_clone_shipping_configs` 4-passed/4-skipped are green). What the two clusters actually are: **(1) `DescendingPredictor` (9 tests, "Subprocess aborted") is THE SAME σ=0 `normal_distribution` PROCESS ABORT recorded above for the coordination probe** — `DescendingPredictor.cpp:197` and `:224` construct `normal_distribution(0.0f, init_noise_scale_)`, which on hardened libstdc++ **aborts the process** when the scale is 0. The default is 0.01 (safe), so this is armed only when a config or test sets it to 0 — **and `DescendingPredictor` is the `pc_predictor` of the pocketed B thread (`…__pc*.json`)**, whose recorded next knob is a freeze/decay experiment on exactly that predictor. If B resumes and that scale reaches 0, runs die mid-episode and `seedavg` scores corpses — the precise 2026-08-09 failure shape, in a second module. **The 2026-08-09 fix was applied at the crash site, not to the pattern; the pattern is "any `normal_distribution` whose σ is a live parameter".** Fix shape (as used in `GainEvolver::mutate_candidate`): draw from a **fixed unit normal and scale after**, so σ never parameterizes the distribution and σ=0 is unreachable by construction. **(2) `HotPatch`/`HotPatchConnect` (10 tests) are STALE, not broken code:** they `EXPECT_THROW` on a rejected patch, but commit `7db087a` ("per-batch patch isolation") deliberately changed the semantics to *log* `hot-patch batch N REJECTED … — remaining batches still applied* and continue. The live behavior is visible in every picrawler run today (a body-script patch aimed at the absent `cruse_coordinator` is rejected per-batch each run). The tests were never updated with the semantics change. **Rule this reinforces: a red suite that predates your branch still has to be attributed before it is dismissed — and "attributed" means run on a clean tree, not reasoned about.** |

---

### ★★ 2026-08-17 — PART IV: THE GAINEVOLVER IS BUILT AND GATE 1 PASSES (`IN_FLIGHT`)

**Verdict: `IN_FLIGHT`.** Byte-identity smoke PASSED; the search loop is verified live
end-to-end; **the convergence gate, the (d)-test, and job #1 are NOT yet run, so there is
no capability claim here at all.** Full build record + parameters:
[`../plans-and-designs/adaptive_gains_substrate_plan.md`](../plans-and-designs/adaptive_gains_substrate_plan.md) §6.

**What it is.** A lifetime (1+1)-ES over the 8 high-value gains (`rear_land_gain`,
`rear_knee_plant`, `rear_push_ext`, `amp_target`, `height_homeo_gain`, `postural_gain`,
`coupling_gain`, `plan_gain`), seeded at the operator's baked rear-sequence point, scored
by an intrinsic error-form criterion and shipped at `mutation_sigma = 0`.

**Three design decisions worth carrying forward, each answering a recorded burn:**

1. **A separate module + a gain topic, not a search inside MotorEPMv2** (the precedent's
   own shape). The payoff is not tidiness: because `GainEvolver` can only see
   `reality.proprio.*`, **the evaluator is stationary by construction** — it is
   structurally incapable of scoring a quantity that the gains it mutates regulate
   through MotorEPMv2's internals. The ratchet/self-referential-threshold family of burns
   is closed by architecture rather than by discipline.
2. **Interleaved incumbent re-evaluation replaces the stored winner.** The precedent keeps
   a best score decaying `×0.99`; this ledger already flags "stores a winner" as the
   residual ratchet shape. Re-measuring the incumbent in its own window every generation
   means a lucky thrash must **keep re-earning** its score, and after a perturbation the
   incumbent degrades within one window — so the (d)-test becomes structural rather than
   dependent on a decay constant.
3. **The viability guard is SEPARATE from the criterion, and its leg term is a MINIMUM.**
   `accept ⇔ G1(falls no-regression) ∧ G2(per-leg loaded-contact minimum) ∧ J_cand < J_inc`.
   G2 is the stance-capture lesson in enforcement form; a group mean would accept a
   candidate that kills one leg while three improve. Both are unit-tested by
   constructing exactly those candidates (`ViabilityRejectsTargetWinsBodyPays`,
   `PerLegMinimaGuard`) — a candidate with a *better* J is REVERTED in both.

**Gate 1 (byte-identity) — PASS.** Promoted config vs `__gainevo` at σ=0, seed 7,
corridor, 12 000 ticks: 251 body-log lines each, **byte-identical** after dropping the
`ge_`/`ga_` instrument keys. Critically this is byte-identity **with the evaluator
running** — the same run shows `ge_ji` 0.4578 and a populated per-term breakdown at
`ge_pub` 0 — so it is not the trivial identity of an inert module.

**Live-search smoke (pipe integrity only).** σ=0.08 via `SETPARAM_AT`, 30 k ticks: 3
generations, 1 accept / 2 reverts, vector migrated off the seed. **The §3.2 two-sided
consumer check passes exactly: `ga_app` 64 = `ge_pub` 8 × 8 keys, `ga_rej` 0.** `rlt`/`rpt`
kept climbing, so the rear-landing consumers stayed live under evolution. **Scale of this
claim: the pipe works. Nothing else.** 3 generations on one seed is far below signal
grade, and J moved *up* across them (0.281 → 0.336), which at this power is noise — quoted
here only so the number is on the record rather than quietly dropped.

**Two §3.2 traps closed during the build, both of which would have produced silent
false verdicts:**
- **The applied-counter had to be READ-BACK verified.** MotorEPMv2's `on_param_change`
  chain has **no terminal else** — an unknown key is silently ignored. A naive counter
  would report 8 landings for 8 sends regardless. The socket now re-reads
  `current_params()` and counts a landing only on an actual value match, so a typo'd key
  increments `gains_rejected` instead of vanishing (unit-tested).
- **Restore had to REPLAY the evolved gains.** They live in param members the instance
  snapshot does not round-trip (params come from the GraphConfig), so without replay a
  restored clone silently reverts to config gains — and the clone-determinism test would
  have blamed the evolver for a divergence it did not cause.

**Carried caveat, stated up front so a later reading is not mistaken for a result:**
`w_distress` consumes the body's **known-contaminated** distress signal as-is (world-frame
stall half + an apparent 50 Hz-vs-240 Hz normalisation error that makes it under-fire).
Fixing that sensor moves the deployed panic and plan-distress-cut, so it is a separate
lever. **If `ge_dis` reads ~0 through the gates, that weight is dead — which is a
measurement about the sensor, not a verdict on the criterion.**

### ★★★ 2026-08-17 — GATE 2: THE SEARCH RUNS, AND ITS CRITERION IS 81 % NOISE FROM ONE TERM

**Verdict: `PARTIAL`.** Arm `…__gainevo_factory.json` (factory seeds, σ 0.08), **arena,
n=4, 256 k ticks, solid chassis**, 31 generations/seed. Mechanism passes; **convergence
is NOT demonstrated**; the reason was measured rather than guessed.

**Passes.** Consumer lockstep on all 4 seeds — `ga_app` **512** = `ge_pub` 64 × 8 keys,
`ga_rej` **0**. Accepts > 0 on 4/4 (14/14/10/17 accepts vs 17/17/21/14 reverts), so the
tautology check clears and the search discriminates. Direction is sensible on the
structural gains: **`coupling_gain` 0 → 2.06 (hand 1.55) — switched ON from a factory
zero in all four seeds**, i.e. the search rediscovered from egocentric error alone that
the legs must be coupled; `height_homeo_gain` 0 → **0.039** against a hand-found 0.040;
`rear_push_ext` 0 → 0.63 (hand 0.5); `postural_gain` 0.3 → 0.57 (hand 0.7). 3/4 seeds
ended closer to the hand point.

**The failure, and the number that explains it.** Half-run J deltas were +0.30 / −0.09 /
+0.70 / −0.62 against per-seed incumbent-window sds of 0.54 / 0.88 / 2.08 / 1.21 —
**every delta is inside its own noise.** Pooled over 128 incumbent windows J = 1.009 ±
1.418, and decomposing that variance by weighted term is decisive:

| weighted term | share of J variance |
|---|---|
| `w_falls·falls` | **80.9 %** |
| `w_tilt·var(upright)` | 18.2 % |
| flow + distress + unloaded, combined | **0.9 %** |

**A rare discrete count owns four-fifths of the criterion, and the three gait-quality
terms the criterion exists to select on own under one percent.** Falls is counted over a
2 000-tick measured half-window, where its relative variance is enormous.

**★ The second finding is a design law, not a tuning note: the 1/5th-success anneal is
UNSAFE on a stochastic objective.** Acceptance ran 32–55 % against `target_accept` 0.2,
so σ was driven **up to the `sigma_max` ceiling on 3 of 4 seeds**. That is the signature
of a coin flip — a noisy criterion produces ~50 % acceptance, the rule reads it as
success, and the search *widens* exactly when it should be settling. The classic rule
assumes a deterministic objective; every self-annealing search over a noisy intrinsic
criterion in this repo inherits this trap.

**Re-use context (what would make gate 2 answerable):** (1) `eval_window_ticks` far
above 4 000 — the charter's "≥4 000–6 000" is now *measured* to be too short at noise
sd ≈ 1.4 vs signal ≈ 0.3; (2) cap the falls term per window or convert it to a rate over
a long horizon so a Poisson count cannot dominate; (3) accept on `J_cand < J_inc − k·sd`
rather than a bare inequality, and anneal on improvement magnitude rather than raw accept
rate; (4) common random numbers across the incumbent/candidate pair so shared noise
cancels. **Per §3.3 the response is to fix the instrument, not to power a
sub-noise effect with more seeds.**

**Context that bounds the claim:** over 256 k ticks all four seeds reach the arena floor
edge (`max_z` 9.9–10.1) and its containment ramps, so an unknown share of `falls` are
boundary events rather than gait failures — inflating precisely the dominant term. The
factory body is also genuinely unstable by construction (tilt_sd 0.339, `bellyc_min`
0.000 — the belly reaches the ground), which is the *hard* version of the question.
⚠ Note this run used `OGMA_PICRAWLER_CHASSIS_COLLIDE=1`, which `seedavg` does **not**
set: on the ghost default the belly cannot touch, and with `height_homeo_gain` under
evolution and no belly term in the criterion the search would have been free to sag for
nothing. Ghost-chassis history is therefore not directly comparable to these numbers.

### ★★★ 2026-08-17 — GATE 2 RE-RUN: THE NOISE FIX WORKS, AND IT COST US THE ONE STRONG RESULT

**Verdict: `PARTIAL` (instrument `WORKING`, convergence still not demonstrated).** Same
arm and protocol as the entry above, now with the repaired criterion (falls → guard-only,
var → sd, `w_distress` 0, window 4 000 → 12 000, noise-aware acceptance) plus
`auto_reset_on_outer_wall=1`. n=4, arena, solid chassis, **500 k ticks**, 20 generations.

**The instrument fix is confirmed, on every axis it was designed for:**

| | gate 2 | gate 2 re-run |
|---|---|---|
| pooled measurement noise (sd) | 0.8765 | **0.1913** (4.6× lower) |
| σ pinned at the 0.5 ceiling | **3/4 seeds** | **0/4 seeds** |
| `falls` share of J variance | 80.9 % | **0 %** (guard-only) |
| accepts / 20 generations | 14 / 14 / 10 / 17 | 6 / 8 / 7 / 6 |
| falls per 100 k ticks | 11.43 | **9.50** |
| tilt_sd | 0.339 | **0.239** |

Reverts now outnumber accepts 2:1 — the margin is biting, which is exactly what a
noise-aware acceptance rule is supposed to do. Lockstep OK and accepts > 0 on 4/4.

**The gate still does not pass.** Judged against each seed's OWN measured noise:
seed 3 **J fell −0.348 against noise 0.143** (2.4× — a real improvement, the first one
this campaign has been able to *claim*); seed 4 rose +0.232 against 0.136; seeds 1 and 2
sat inside their noise (−0.104 vs 0.290; +0.005 vs 0.113). One clear win, one clear loss,
two nulls is not convergence.

**★ THE FINDING THAT MATTERS, AND IT IS A WARNING ABOUT NOISE-CLEANING:** gate 2's single
strongest result was `coupling_gain` 0 → 2.06 with **all four seeds** switching it on —
the search rediscovering that the legs must be coupled. After the fix it is **0.61 ± 0.57
with two of four seeds leaving it at zero.** The term we deleted for being 81 % noise was
also carrying the selection pressure that found coupling: with `falls` in J, an
uncoordinated body was punished hard enough that turning coupling on paid; with it gone,
`sd(upright)` does not reward coupling nearly as strongly. **A term can be the noisiest
thing in a criterion AND the only thing carrying a particular signal. Removing it is not
free, and "the noise went down" is not by itself evidence the criterion got better.**

**Re-use context / next design, in order:**
1. **Restore the falls signal in a low-variance FORM rather than as a count.** `sd(upright)`
   measures wobble about the mean; it does not specifically measure *going toward
   inverted*. A continuous **near-inversion dwell** — mean over the window of
   `max(0, thresh − upright)` — is sampled every tick, is the actual pre-fall regime, and
   should recover the coupling pressure without the Poisson variance.
2. **Anneal on improvement MAGNITUDE relative to σ̂, not on accept count.** Even with the
   margin, acceptance held 30–40 % (above the 1/5th target of 0.2) and σ still climbed to
   0.13–0.40. The classic 20 % target is calibrated for a converged local search; during a
   genuine improvement phase from a bad start, 40 % acceptance is legitimate. Shrink σ when
   wins stop being large relative to σ̂ — that is the real signature of convergence.
3. **More generations.** The search is now progress-limited, not noise-limited: 6–8 accepts
   in 20 generations is too few to converge an 8-D vector.

⚠ **`net_disp` / `straight` are meaningless in this arm** — outer-wall recentering
teleports the body to the origin, so displacement no longer accumulates (straight reads
0.01). Falls and tilt above are per-tick normalised across the two run lengths (256 k vs
500 k); the raw counts are not comparable.

### ★★★ 2026-08-18 — ENERGY EARNS ITS PLACE: two seeds converge, σ anneals to the FLOOR

**Verdict: `PARTIAL`, and the first run whose direction is unambiguous.** Same factory
arm/protocol; criterion now carries **energy** (mean |joint torque| = servo current, w 8.0),
the operator's **leaky fall alarm** (guard-only, trip 8.0), dwell instrumented at 0, and the
operator's timing (warmup 10 000, window 4 000, settle 2 000 → **61 generations**).

**The campaign's convergence signal across three runs of the same arm:**

| | gate 2 | gate 2 re-run | **gate 2d** |
|---|---|---|---|
| seeds whose J fell beyond their OWN noise | 0/4 | 1/4 | **2/4** |
| seeds whose J *rose* | 1/4 | 1/4 | **0/4** |
| σ at the 0.5 ceiling (runaway) | 3/4 | 0/4 | **0/4** |
| σ annealed to the 0.01 FLOOR (settled) | 0/4 | 0/4 | **2/4** |
| moved closer to the hand point | 3/4 | 3/4 | **4/4** |
| pooled measurement noise | 0.8765 | 0.1913 | 0.2439 |

★ **Seeds 3 and 4 both fell (−0.208 vs noise 0.198; −0.303 vs 0.248) AND annealed σ down to
0.010/0.015 — the 1/5th rule reaching its floor is what a converged search looks like**, and
it is the exact opposite of gate 2's runaway to the ceiling. Accepts drop 21/14/10/8 across
the seeds as reverts take over, which is settling rather than starvation. Noise is slightly
above the re-run's because the window is 3× shorter — the expected, accepted trade.

**Energy is safe and it is doing the work.** Variance shares are now **energy 56.4 %**, flow
34.4 %, upright-sd 8.7 %, unloaded 0.5 %. ⚠ At w 8.0 energy holds the *majority* vote, which
is more than the design intended (the weight was set to equalise its signal with
sd(upright)'s) — worth dropping toward 4–5 if we want a genuine three-way balance.
**The freeze trap did NOT materialise**: at completion every seed either improved flow or
raised energy, and `amp_min` 0.419 → **0.442** with `step_bal` 0.41 → **0.44** says no leg was
sacrificed to save current. ⚠ *A mid-run reading of seed 4 tripped my own freeze detector and
I reported it as live; by completion it had resolved (energy −0.021 with flow_term −0.057,
i.e. cheaper AND better). Mid-run halves of a 61-generation search are not verdicts.*

**The fall alarm behaves exactly as specified.** Duty by seed: 27.4 % / 0 % / 0 % / 21.0 %,
tripping only on the two seeds that were actually falling (alarm peaks 21.6 and 15.0 vs 5.6
and 4.8 on the healthy pair). Threshold 8.0 sits cleanly between the two populations, so the
adventurous body is never punished and the persistent faller is.

**★ DWELL, measured at last on full per-tick data: `noise/mean 2.93` vs sd(upright)'s 1.93 —
STAYS AT WEIGHT 0.** Note the honest correction: the 60-tick-sampled proxy had predicted
5.45, so shipping it inert-but-instrumented was the right call *and* the proxy overstated the
penalty by ~1.9× (I predicted ~1.4×). It remains worse than the term it would join, so the
verdict holds — but on measurement, not on the estimate that motivated it.

**The open problem is unchanged: `coupling_gain` is still not reliably found** — 1.458 /
0.000 / 0.558 / 0.274 (mean 0.573), and notably the two *converged* seeds settled at LOW
coupling. Energy did not restore the pressure that the old noisy falls term supplied.
`plan_gain` did finally move off zero (0.037 vs hand 0.05), and `height_homeo` 0.036 vs hand
0.040 remains the closest match in the vector.

**Re-use context:** drop `w_energy` to 4–5 for a three-way variance balance; the coupling
question now needs its own answer rather than a better general criterion — the candidate is
a lexicographic viability-then-quality ordering, since a threshold comparison survives the
variance that ruins these signals as gradients.

### 2026-08-17 — POST-PLANT SLIP: `DEFERRED` from the GainEvolver's v1 criterion

The charter lists post-plant slip (foot drift while planted) as a criterion ingredient.
**It was not built, because no egocentric slip signal exists anywhere in the codebase** —
and the two nearest candidates are both illegal for a criterion that argues its own
sim2real legitimacy: `lateral_v` is a **soft oracle** (`_chassis.linear_velocity`
projected on body-right — the audit's own classification), and `_grf_fwd` is declared
**permanently god's-eye** in the body script. Building the estimator *and* the evolver in
one lever would also have violated one-lever-at-a-time, with the unvalidated estimator
inside the very criterion meant to judge everything else.

**Re-use context (what would justify building it):** the body publishing a legal
planted-foot drift signal — FK foot position in the chassis frame (`foot_xz`, already
logged) differenced across ticks while `foot_contact` holds, cross-checked between
simultaneously-planted legs to cancel body motion; or a servo-current slip proxy. Note
the confound that makes it a real build rather than a one-liner: body-frame foot motion is
body motion plus foot motion, so a *single* leg's drift cannot distinguish slipping from
walking — the differential between planted legs is the honest form. **When it exists it
slots into the GainEvolver as one more `w_*` term with zero loop changes** — the criterion
was written as a weighted sum of independently-accumulated terms for exactly this reason.

---

### ★ 2026-07-26 — THE POWER STROKE IS NOT PHASE-LOCKED TO GROUND CONTACT

**This is the measured cause of the pinned-flat-speed entry in §5, and it re-frames the
whole inter-leg coordination family.** It overturns no prior verdict; it supplies the *why*
that eight nulls were missing.

**Instrument.** `gait_align_diag` in MotorEPM (diagnostic only — the block is skipped
entirely at its default 0). Config `..._embed_corridor_alignprobe.json` subscribes the true
`foot_contact` flag with the new `contact_instrument_only=1`, so ground truth is READ
without being wired to the stance gate (that swap is separately refuted, §2), plus
`joint_torque`. Collector `scripts_tools/gaitalign.py`.

**Gain-0 guard verified by measurement, not argument:** the probe arm and the deployed
`..._imufused.json` produce identical `seedavg` output on all 17 metrics, per seed, at both
6 000 and 12 000 ticks.

#### The result (n=4, corridor, diff 0.3)

| | 12 000 ticks | 6 000 ticks |
|---|---|---|
| `pos_stance` — frac of STANCE in the stroke's positive half | **0.509 ± 0.010** | **0.512 ± 0.014** |
| `pos_swing` — same over SWING | **0.505 ± 0.010** | **0.513 ± 0.010** |
| `td_plv` — stroke phase-lock at true touchdown | 0.228 ± 0.022 | 0.200 ± 0.003 |
| `contact_duty` — true stance duty | 0.752 ± 0.027 | 0.800 ± 0.010 |

**The push direction is statistically independent of whether the foot is on the ground.**
Stance and swing split the stroke waveform identically, to ±0.01 on every seed. Half the
power stroke is spent in the air; half the return swing scrubs while planted.

#### Why: three clocks, none locked to each other

| clock | period (ticks) | what it drives |
|---|---|---|
| hip1 — the stride | 32.3 ± 1.4 / 30.7 ± 3.1 | — |
| **knee — `L.phase`** | **23.6 ± 2.7 / 22.2 ± 2.2** | **the power stroke**, `y[0] += amp·sin(L.phase + stroke_phase)` |
| **contact — the real step** | **29.9 ± 3.3 / 26.0 ± 0.6** | what the leg actually does |
| foot-height — the incumbent detector | 15.3 ± 4.5 / 12.6 ± 1.2 | `stance_lift` + every Cruse rule |

`phase_joint` defaults to −1 = the knee, so the stroke is timed by a ~22–24 tick clock while
the leg steps every ~26–30. **They beat with a period of ~2–2.5 s**, which is the operator's
report — *"occasionally the three planted legs are in a good position and the fourth steps
forward and moves the body, but this synchronization is often lost"* — as a number.

Separately: the foot-height detector runs at roughly **half** the true contact period, i.e.
it fires about twice per real step. That is the relaxation oscillator documented at
`MotorEPM.hpp:314-328`, measured directly for the first time. It is chatter, not stepping.

#### Consequence for the coordination family

All eight refuted timing levers adjusted phase **between** legs while the thrust↔support
relation **within** a leg was uncorrelated. Re-phasing legs whose own thrust is random with
respect to their own footfall cannot buy anything — and did not. **Re-use context for the
whole family: do not retry an inter-leg phasing lever until thrust and support are locked
within a leg.**

#### Two follow-on questions, answered by the same run

- **Load is real, and hip1 is the signal — which overturns the obvious guess.** Stance/swing
  torque ratio: **hip1 1.368 ± 0.053**, hip2 1.124 ± 0.018, **knee 1.011 ± 0.031 (nothing)**.
  hip2 and the knee hold a near-static posture in *both* phases; hip1's torque is the ground
  reaction to the sweep itself, measured on the very joint the stroke acts on.
- **A precision-gated coordination probe is NOT a tautology.** `explore_mult` = 0.87 (12 k) /
  0.73 (6 k), so progress→commit leaves the probe σ near full most of the time.

#### Also established

- **`gait_phase` has left the imposed trot entirely**, to a different place per seed —
  `[0,−2.13,2.43,−0.70]`, `[0,−2.66,1.87,−2.12]`, `[0,2.15,−3.07,−1.02]`,
  `[0,2.58,2.15,−2.35]` vs the configured `[0,π,π,0]`. The coordination target is a random
  walk, not an imposed topology.
- **The §4 "which Rule 3 is live" ambiguity resolves to NEITHER** for the deployed stack:
  `cruse_bias` is exactly 0.0000 *and* `CruseCoordinator` is not instantiated (the graph is 5
  modules: bridge, MotorEPM, CPG, KeyframeGait, BodyRhythmTracker).
- **`planted` = 3.69–3.79**, higher than the 3.30 previously recorded. The support constraint
  is over-satisfied, independently re-confirming that the phasing question is closed (§2).
- **Baseline figures are run-length-dependent and must be quoted with their tick count.**
  `..._imufused.json` scores net_z **4.75 ± 0.29 / straight 0.74** at **6 000** ticks (the
  standard protocol, reproduced exactly) and net_z **6.55 ± 0.56 / straight 0.61** at 12 000.
  §8's "a number outlives the body it was measured on" applies to run length too.

**New instruments, all default-off:** `gait_align_diag`, `contact_instrument_only`,
`torque_topic`, `stroke_load_gain`, `scripts_tools/gaitalign.py`, and the body-stdout fields
`td_plv`/`sd_plv`/`pos_stance`/`pos_swing`/`contact_duty`/`tq_agree`/`tq_sep_j`/
`per_hip1|knee|foot|con`/`explore_mult`/`sgate`/`sgate_spr`.

---

### ★ 2026-07-27 — THE ROBOT WALKS STRAIGHT-LEGGED, AND THE CORRIDOR WAS HIDING IT

Operator UI observation (arena gym): *"hip1 is appropriately doing the work to swing the leg
forward, but hip2 and knee stay fairly horizontal to the chassis."* Measured, and it is worse
than it looks.

#### The sprawl, quantified (arena, n=3, 1032 leg-frames, post-warmup)

| | measured | design rest pose |
|---|---|---|
| `hip2` angle | **−3.6° ± 4.4** (range −23…+6) | 0° = femur PARALLEL TO GROUND |
| `knee` angle | −49.3° ± 15.8 (range −98…+15) | −80° |
| **tibia off vertical** | **37.5° ± 15.3** (range 0…**101°**) | **10°** |
| **planted foot radius** | **170 mm** (range 136…179) | total leg reach **166 mm** |

**The femur never leaves neutral for an entire run**, so hip2 contributes nothing but a fixed
53.6 mm horizontal offset; the knee carries the whole gait alone and sweeps the shank to
**~4× the design rest angle**; and the feet plant **at the limb's full reach** — straight-legged,
maximum moment arm, minimum mechanical advantage. Corroborating waste: **`scrub` 0.100 against
`fwd_v` 0.050 — the body slides sideways twice as fast as it advances**, which is what sprawled
legs do (the lateral components largely cancel left-to-right and that work is thrown away).

#### ⚠️ THE CORRIDOR MASKS THIS FAMILY — measure heading effects in the ARENA

`_build_corridor()` places the curriculum *"inside self-centering 30° walls"*
(`picrawler_body.gd:2684`). That geometry **actively re-centers the body**, so a yaw excursion
is corrected by the wall before it reaches `straight` or `turns`. Every tool in
`scripts_tools/` hardcoded `OGMA_PICRAWLER_GYM="corridor"`, so this whole family was being
scored in the one gym where its target failure is suppressed. New `scripts_tools/arenaavg.py`
(open floor; `net_disp`/`straight` since `net_z` is meaningless off-axis; boundary guard;
`tib_off` and `foot_r` posture columns).

**The two gyms disagree materially, so a corridor verdict does not transfer.** Baseline
`steps` 53 → **25** and `step_bal` 0.44 → **0.07** in the arena: the tripod-skid is far more
severe on open ground. The load-gated stroke likewise reads differently (corridor: `step_bal`
0.44→0.54; arena: `tilt_sd` 0.088→**0.063** and `planted` 3.87→**3.99**, but `steps` 25→14).

#### The proposed MECHANISM was refuted while the proposed ACTION worked

The operator's hypothesis was that sweeping an extended limb dumps yaw into the chassis. A new
instrument (`|Δyaw rate|` split by support state, per limb) says **no**: `yawd_swing_excess` is
**negative in every arm** (−0.035 baseline) and grows *more* negative as lifting rises. The
chassis takes **less** yaw impulse while a foot is airborne than during full support — the
larger source appears to be the skid-steer heading controller pushing through four planted legs
that fight each other. *(A first version of this instrument used mean |yaw rate| and was blind:
intentional steering acts through planted feet and swamped the reaction torque. A reaction
torque is an impulse; differencing separates them.)*

**The likelier reading is foot CLEARANCE, not angular momentum:** the arena baseline takes 25
steps in 85 s with `step_bal` 0.07 — a limb that is not clearing the ground to complete a step.

#### ★ KINEMATIC CONFLICT — tibia-vertical and belly-up cannot both be won by femur angle

`hip2_tuck_target` sweep (arena, n=3) splits the two goals exactly as the CAD predicts, because
the femur angle sets ride height and shank angle in *opposite* directions:

| | base | −0.2 (femur UP) | +0.2 (femur DOWN) |
|---|---|---|---|
| tibia off vertical | 37.5° | **32.9°** | 43.7° |
| belly clearance | 0.0221 | **0.0134** ⚠️ | **0.0335** |
| foot radius (mm) | 170.3 | 170.8 | **167.9** |
| net_disp | 4.85 | **6.16 (+27 %)** | 4.56 |
| straight | 0.71 | **0.80** | 0.65 |
| falls | 0 | 0 | **0.33** ⚠️ |

Femur up plumbs the shank and buys **+27 % distance and +13 % straightness** — while dropping
belly clearance **39 %**, a regression on a promoted invariant. Femur down raises the belly
**52 %** and shortens the moment arm, while making the shank *more* oblique, slower, and adding
falls. **Neither is promotable, and the trade is kinematic, not a tuning miss.** Note also that
the +27 % arm did NOT shorten the moment arm (`foot_r` 170.3→170.8), so its gain is not the
gear ratio — it is shank verticality plus a lower CoG.

**Re-use context stands and is now sharper:** the original refutation was *"didn't crouch (weak
reflex)"* at `postural_gain=0.3`; at the promoted 0.7 the parameter now bites hard in both
directions. The failure is no longer authority, it is the conflict above.

#### ★★ `tibia_plumb_gain` — the operator's INVERSE-KINEMATICS framing, as an error

hip2 and the knee are a planar 2-link arm. With hip2 pinned at its horizontal rest the KNEE
ALONE must set both the foot's height and its fore-aft position, so the foot is forced along a
circular arc about the knee axis and the shank has to sweep through a large angle to translate
the foot at all. The reflex gives hip2 an objective — null the shank's deviation from vertical —
so the knee's gait drive TRANSLATES the foot instead of arcing it. Nothing about timing is
specified, so the rhythm stays emergent (§5.7).

`θ_tibia = 1.40·x[hip2] + x[knee] − 0.0292` rad, from the body's own kinematic constants
(`HIP2_LIMIT=1.40`, `KNEE_REST=−1.6`), validated against the CAD rest pose and cross-checked
controller-side against the raw joint angles (51.5° vs 52.5°).

**Distinct from the refuted "learned hip2"**, which merely LOOSENED hip2's postural spring and
hoped the HK controller would discover the coordination. An unconstrained joint is a wobble
dimension, not an IK solver; this one is given an objective.

| gain | net_disp | straight | tilt_sd | tib_off | bellyc | falls |
|---|---|---|---|---|---|---|
| base | 4.85 ± 0.50 | 0.71 ± 0.02 | 0.0877 | 37.5° | 0.0221 | 0 |
| −0.3 | 5.17 | 0.65 | 0.1228 | 50.8° | 0.0480 | 0.33 |
| −0.15 | 5.48 | 0.70 | 0.0834 | 43.6° | 0.0319 | 0.33 |
| **+0.15** | **6.38 ± 0.61 (+32 %)** | **0.82 ± 0.00** | **0.0689** | 34.3° | 0.0156 ⚠️ | **0** |
| +0.3 | 5.44 ± 1.09 | 0.75 | 0.266 ⚠️ | 33.9° | 0.0103 ⚠️ | 0 |

**+0.15 is the largest single effect measured this session: +32 % distance and `straight` 0.82
with a standard deviation of 0.00 across three seeds** — the variance-collapse signature the
promoted heading-hold produced. It beats the static `hip2_tuck_target` shift (6.16 / 0.80) while
moving the tibia *less* (34.3° vs 32.9°), i.e. the reflex buys more per degree than a mean
posture shift — consistent with correcting the shank *through the stride* rather than on
average. **Not promoted: belly clearance falls 0.0221 → 0.0156 (−29 %),** and belly-up is a
promoted invariant. `IN_FLIGHT` — needs the corridor, hump, recovery and inversion gates.

#### ★★ THE KINEMATIC DEAD END — no joint angle buys both a vertical shank and a high chassis

The escape route was to plumb with hip2 and pay the ride height back with the knee (the joint
`stance_lift` proved pushes the body up off PLANTED feet). It **closes, and informatively**:

| | base | hip2 −0.2 | hip2 −0.2 + knee_tuck 0.85 |
|---|---|---|---|
| belly clearance | 0.0221 | 0.0134 ⚠️ | **0.0215** ✓ restored |
| tibia off vertical | 37.5° | **32.9°** | **44.1°** ✗ worse than baseline |

**The knee buys height by FOLDING the shank, which is by definition the opposite of plumbing
it.** hip2 plumbs and drops the body; the knee raises the body and un-plumbs. Those are the same
2-link constraint seen twice, and the reason is geometric: the feet plant at a **170 mm** radius
against a **166 mm** total leg reach, so the shank *must* angle out simply to reach the ground.
At that foot placement the leg is not long enough for both.

**⇒ And the last escape — "shorter steps bring the feet closer in" — is REFUTED on kinematics.**
`stroke_gain` swept 1.65 → 1.4 → 1.2 → 0.9 (a 1.8× range) leaves **`foot_r` invariant at
170.3–171.2 mm and `tib_off` invariant at 37.4–38.2°.** The reason is structural:
**`HIP1_AXIS` is world +Z** (`picrawler_geometry.md`) — hip1 is a *yaw* joint that sweeps the
foot fore-aft along an arc at **CONSTANT RADIUS**. Stride length and foot radius are
kinematically independent on this body; only hip2 + knee set the radius, and every way they
reduce it increases shank obliquity (the `knee_tuck` column above: 170→164→162→145 mm bought
`tib_off` 37.5→44.1→48.8→**64.5°** and collapsed the gait to net_disp 1.33).

**So the "get the tibia vertical" family is closed on this geometry.** Verticality trades against
ride height (hip2), against feet-in (knee), and is untouchable by stride. It is not a tuning
failure — it is that the leg is not long enough for the foot placement the gait uses. Re-use
context: a body with a longer tibia or a shorter femur, or a lever that moves the foot placement
by some route other than these three.

#### ★★ BUT THE SWEEP FOUND A REAL LEVER: the body is OVER-STRIDING

The same runs answer a different question — the one §5 named and nobody had tried
(*"propulsion amplitude / stroke operating point (never swept)"*):

| `stroke_gain` | net_disp | straight | tilt_sd | bellyc | falls |
|---|---|---|---|---|---|
| **1.65 (deployed)** | 4.85 ± 0.50 | 0.71 ± 0.02 | 0.0877 | 0.0221 | 0 |
| 1.4 | 5.17 ± 0.43 | 0.75 ± 0.03 | **0.0703** | 0.0232 | 0 |
| **1.2** | **5.45 ± 0.51 (+12 %)** | **0.78 ± 0.03** | 0.0778 | 0.0228 | 0 |
| 0.9 | 4.56 ± 0.11 | **0.79 ± 0.01** | 0.0739 | 0.0223 | 0 |

**A SHORTER stroke walks FURTHER**, peaking near 1.2: +12 % distance, +10 % straightness, less
wobble, **belly clearance preserved**, 0 falls throughout. The deployed 1.65 is past the
optimum. This validates the operator's *conclusion* ("shorter steps move the robot faster")
while refuting the *mechanism* they proposed for it (feet closer in — see above): the gain is
that a long stroke wastes effort, not that it changes where the foot lands.

**GATES (all run).** Corridor cross-check n=4 vs the deployed baseline at `stroke_gain=1.2`:
net_z 4.75→4.70, straight 0.74→0.75, flat_v 0.05→0.05, belly 0.023→0.024, 0 falls — **a tie on
progress** — but `steps` 53→**64** and `step_bal` 0.44→**0.52**. (1.4 is worse in the corridor:
net_z 4.24.) Hump gate n=4: final_z 4.74→**4.67**, gain_z 2.13→2.03, 0 falls — **holds, with the
variance halved** (std 0.51→0.25).

**Verdict `PARTIAL`, not promoted.** No regression anywhere, belly preserved, hump intact, 0
falls in every arm — but the +12 % is arena-only and the corridor is a tie, so by §3.3 (*a real
capability is LOUD*) this is not loud enough to promote on metrics alone. **What appears in BOTH
gyms is more stepping and better leg participation.** Next step is operator UI observation (§3
rule 5) before any bake-in; the mechanism (over-striding) is worth pursuing further because it
is the first time this campaign has moved distance at all without paying in belly or falls.

#### Swing-phase leg fold (`swing_tuck_hip2` / `swing_tuck_knee`) — the KNEE half carries it

The mirror of the promoted `stance_lift` (which biases the knee of PLANTED legs), gated on
**true contact** rather than the foot-height detector — that detector fires ~2× per real step,
and a tuck on a chattering gate would retract the limb mid-stance, i.e. lift a loaded foot.
Wiring true contact as a swing gate is refuted (§2) but for a consumer that wanted gait *phase*;
this one wants "is the foot off the ground", which the re-use context names ("step-over foot
placement"). This is also `hip2_tuck_target` **with the gate it was missing** — doctrine §5:
*ask what state should have gated a failed bias before calling the idea dead.*

Sign probe (n=3, arena): **−hip2 LIFTS** the limb (airborne fraction 0.30–0.47 vs a 0.20
baseline), **+hip2 SUPPRESSES** lifting (0.11–0.17, `steps` collapse 25→4). Magnitude sweep:

| arm | net_disp | straight | tilt_sd | step_bal | steps | bellyc |
|---|---|---|---|---|---|---|
| base | 4.85 | 0.71 | 0.0877 | **0.07** | 25 | 0.0221 |
| h−0.1 / k+0.1 | **5.26** | **0.75** | 0.0713 | 0.28 | 26 | 0.0226 |
| h−0.2 / k+0.2 | 4.96 | 0.68 | 0.0710 | 0.47 | 35 | — |
| h−0.4 / k+0.4 | 4.20 | 0.65 | 0.0804 | 0.18 | 91 | 0.0237 |
| h−0.2 only | 4.68 | 0.71 | 0.0868 | 0.20 | 47 | 0.0221 |
| **k+0.2 only** | **5.08** | 0.72 | **0.0731** | **0.49** | 27 | **0.0229** |

**`step_bal` 0.07 → 0.49 (7×) on the KNEE HALF ALONE**, with distance +5 %, wobble −17 %, belly
preserved, 0 falls. Folding the shank during swing carries the effect; lifting the femur does
not.

**CORRIDOR CROSS-CHECK (n=4, 6000 ticks) — the headline does NOT transfer.** Against a matched
control (identical instruments, gain 0): net_z 4.75→4.80, straight 0.74→0.73, **step_bal
0.44→0.43**, belly 0.023→0.023, 0 falls, `steps` 53→35, and the one real move is **tilt_sd
0.068→0.059 (−13 %)**. A tie plus a modest wobble improvement. **Diagnosis: the arena's 7× gain
was HEADROOM, not transferable mechanism** — the corridor baseline already sits at `step_bal`
0.44 while the arena sits at 0.07. Verdict `PARTIAL`, scenario-scoped to open ground; the wobble
reduction is the part that appears in both gyms. Not promoted.

---

### ★★★ 2026-08-09 — THE BRAKE IS A PHASE, NOT A LEG: recovery-while-planted shear, measured down to its mechanism

The stance-hip2 verdict left "fl brakes 2.5× harder" as a leg-shaped mystery. Four
measurements on the actuator-sweep traces (solid chassis, per-tick GRF ⊕ true contact ⊕
commanded targets; instrument `flbrake.py` over the traces `actsweep.py` banks) close it:

1. **Liftoff lag is BODY-WIDE, not fl's.** Every leg stays planted ~10–12 ticks past its
   own stroke reversal (fl 11.1 / fr 11.8 / rl 9.8 / rr 10.2) — and the **commanded**
   stroke reverses **7–9 ticks before liftoff** with only 2–3 ticks of servo slew. The
   brain itself sweeps planted legs backward; the lift channel is late, not the tracking.
2. **Load does not discriminate.** fl's recovery-window normal force is ordinary
   (rec/prop 0.92) and the second-LOWEST of the four legs in absolute terms.
3. **Shear discriminates.** During recovery-in-stance fl converts **−0.48** of its normal
   load into backward force vs fr −0.32, rr −0.29, rl −0.15 — saturated friction. fl
   SLIPS where rl pivots.
4. **Posture is the correlate.** fl holds the most-flexed knee (−0.95 rad) and the
   most-tilted tibia (−0.95 rad from vertical) during recovery, and tibia tilt tracks the
   shear gradient across legs — the §2 plumb story's cost, localized to one leg and one
   phase.

**The knobs REDISTRIBUTE this cost; none remove it.** `stance_lift_hip2` 0 → 0.25 → 0.5
deepens fl's net brake monotonically (−0.0016 → −0.0021 → −0.0027 g/tick);
`stance_lift_gain` 0.8 flips fl to a propulsor (+0.0009, its propulsive force ×1.7) and
hands the brake to BOTH rear legs. Conservation, not cure: some leg always pays the
recovery-shear toll, because every leg is commanded to sweep while planted and pressed.

⚠ The first-pass numbers were computed on traces silently TRUNCATED at 1.9–4 MB of ~12 MB
(FileAccess buffers; the quit path never closes the file). Flush fix landed same day; the
hip2/stance_lift comparisons above are from full-length traces. An instrument that loses
its tail reads as "early-run behavior" without saying so.

⇒ **The lever this names** (§5 top): the stroke-direction-aware stance release. Predictions
it must satisfy: g_rec shrinks body-wide (flbrake), act_lag drops toward the 2–3-tick slew
floor, and — if the pressed-shuffle reading of the basin problem holds — shuffle seeds
convert to walk seeds at n=20.

**SAME-DAY RESULT — the walk-fraction prediction held, loudly.** `stance_release_frac=1.0`
on the supportepm base, n=20, 12 000 ticks, solid chassis, same seeds both arms:
**walkers (steps>30) 2/20 → 11/20**; steps 17.8 → 62.3 (3.5×); net_z 1.95 ± 1.31 →
2.97 ± 2.30; step_bal 0.12 → 0.23; **plv 0.09 → 0.13** — the legs phase-lock more when
their feet are allowed off the ground. Consumer check `sr_duty` = 0.75 (the release
latches for most of stance once the stroke reverses, as designed). **Costs, named:**
belly clearance 0.031 → 0.024 with `bellyc_min` 0.014 → **0.003** (the tuck IS the
belly-up mechanism and the release fades it — the body walks but rides the deck),
falls 0.15 → 0.30, straight 0.69 → 0.56, one destabilized seed (tilt_sd 1.09). At 0.5
the n=6 screen keeps control-level stability (tilt_sd 0.058, unstable 0.01) at an
intermediate belly cost — the dose sweep is the open edge, plus a shaped variant
(release the KNEE only, keep the hip2 press) if the belly cost proves inherent.
**Not promoted: pending operator UI observation (§3 rule 5) and the dose question.**

**DOSE MEASURED (same day, n=20 each): the gradient is monotonic on BOTH axes.**
Walkers 2/20 → **6/20** → **11/20** across frac 0 → 0.5 → 1.0; net_z 1.95 → 2.50 → 2.97;
plv 0.09 → 0.12 → 0.13 — and the costs ride the same gradient: bellyc_min 0.014 → 0.008
→ 0.003, tilt_sd 0.062 → 0.073 → 0.148, falls 0.15 → **0.15** → 0.30, straight 0.69 →
**0.69** → 0.56. **At 0.5 the lever buys 3× walkers at ZERO falls/straightness cost and
a moderate belly cost.** A monotonic dose–response on a mechanism-predicted target is
what "authority + cause" means — this knob has causal authority over the walk/shuffle
basin, which is the thing the actuator search was looking for and the criterion could
not name. Recommended operator read: watch 0.5 and 1.0 in the UI; if the 1.0 wobble is
the deck-riding, the knee-only shaped release is the next refinement.

**ARENA CROSS-CHECK (same day, operator-prompted; 4 arms × n=20, 12 000 ticks, diff 0,
solid chassis, `arenaavg.py`) — the costs were corridor artifacts; the conversion is
corridor-assisted.** The dose gradient on progress TRANSFERS: net_disp 1.99 → 2.40 →
**2.74** (+38 %), steps 8.6 → 11.4 → 13.8, both monotonic. The corridor's costs do NOT
transfer: straight is FLAT (0.70/0.69/0.69) and falls *drop* (0.15 → 0.10/0.10) — the
frac-1.0 wobble/veering was hump-and-wall interaction, not intrinsic yaw damage. **But
the walk-fraction conversion shrinks to 1/20 → 2/20 → 3/20** (vs 2 → 6 → 11 in the
corridor), and even the deployed `imufused` base walks only 2/20 on open flat ground
(net_disp 1.92 ± 1.53 — the honest arena reference, protocol fully recorded). ⇒ Layered
reading: the release removes the SUPPRESSOR (pressed feet), while the corridor's rumble
strips supply the PERTURBATIONS that recruit stepping — flat ground leaves the shuffle
unchallenged. Consistent with obstacle-triggered adaptation being one of §3.3's three
real-capability signatures. Directly testable: corridor diff 0 vs 0.3 on the srel arm
(does the conversion need the rumble?). ⚠ `yaw_swing_excess` read exactly 0.0000 in all
80 runs — the instrument did not fire in these configs (contact wiring absent), so the
yaw question is UNMEASURED here, not null.

> ⚠⚠ **CORRECTION (same day, later): EVERY walk-fraction number in this entry and the
> two blocks above — the 2/20→11/20 conversion, the dose curve, the arena and
> flat-corridor comparisons — was measured on the armed-assert build (§4 ★★★★ entry):
> the "shufflers" were runs that CRASHED when commit saturated, and the release changes
> commit dynamics, hence crash timing. The conversion story is RETRACTED.**
>
> **CLEAN VERDICT (n=20 per arm, fixed build, zero crashes, 20/20 walkers everywhere):
> `NULL`-to-`REGRESSION`.** control net_z 6.43 ± 1.58 / tilt_sd 0.099 / falls 0.30 /
> steps 113; srel 0.5 → 5.56 / **0.232** / 0.55 / 114; srel 1.0 → 5.69 / **0.255** /
> 0.55 / 115. Steps FLAT, transport slightly down, wobble 2.4×, falls +83 %. On a body
> that survives its own success the release adds instability and buys nothing.
> `stance_release_frac` stays 0. **Re-use context: the recovery-shear window is real
> trace physics (−0.004…−0.015 g/tick, 7–9 ticks, body-wide) — the toll exists but
> un-pressing mid-stance is evidently the wrong way to collect it. Candidates that
> honor the measurement: the knee-only shaped release, or timing the LIFT earlier
> (P4 entrainment) rather than weakening the stance.**

**TESTED (same day): the discriminator is the GYM, not the texture.** srel 1.0 at n=20,
three environments: corridor diff 0.3 → **11/20** walkers, steps 62; corridor diff 0.0
(flat) → **7–8/20**, steps 34; arena diff 0.0 → **3/20**, steps 14. The conversion
mostly SURVIVES the flat corridor and dies in the arena, so the rumble is a step-count
amplifier (halves without it) but the walker conversion tracks corridor-vs-arena
geometry. Candidate mechanisms for the gap, unmeasured: the corridor's self-centering
walls as a perturbation source, and the nav/bearing drive differing between gyms
(`target_compass` has a corridor goal to pull toward). Also new at flat corridor:
three walkers CIRCLE (straight 0.06–0.12) — the rumble may be helping the heading hold
as well. ⇒ Before attributing any of this, wire the contact instrumentation so
`yaw_swing_excess` fires, and check what `target_compass` publishes in each gym.

---

### ★★★ 2026-08-05 — COMMIT IS A PRECISION, AND THREE HAND-PICKED CROSSOVER POINTS LOST TO THE ORIGINAL

**Operator, watching at 2× with commit ON vs OFF — the observation the metric could not make:**
*"Without commit it takes longer to move forward at all, the pauses are longer, and during the
pause the robot is simply SHAKING — it doesn't look like it's taking any steps. With stroke12 I
see it at least TRYING to move forward during the pauses, with much better steps once it moves.
So commit is definitely promoted but needs refinement."*

⇒ **Commit's job is not preventing stalls — it is SUPPRESSING UNDIRECTED NOISE once directed
motion exists.** With `progress_commit_gain = 0`, `explore_mult` pins at 1.00 and the body gets
full exploration noise permanently: that is the shaking. `progress_commit_gain` moves from
"marginal keeper" (+2.55→+2.62 net_z) to **promoted on behaviour**.

#### Three crossover points, measured (arena, seed 6, `stroke12` base)

| arm | stalled-seconds | disp/s | metres |
|---|---|---|---|
| **default 180/240/90** | **14 %** | 0.0734 | 29 |
| commit OFF | 20 % | 0.0666 | 28 |
| inverted (engage 0.5 s, release 6 s) | **22 %** | 0.0647 | 27 |

**The hand-tuned original beat both directions**, including my own "the asymmetry is backwards"
reasoning — which was plausible (bursts last 1–2 s while commit needs 3 s to engage and 4 s to
ramp, so it arrives after the burst it protects) and **refuted**. The fourth mechanism this
session to die on a cheap test.

> #### ⇒ THE CONSTANT SHOULD BE SET BY THE MECHANISM THAT OUGHT TO SET IT
> Commit answers *"how much do I trust my current motion vs keep searching"* — a **precision**.
> Doctrine §2.3: precision is a CONTROLLED variable, and *"a designer picking the crossover
> point is the anti-pattern"*. Three designers have now picked one and the original won, which
> is precisely the signature of a knob that wants a mechanism rather than a better value.
>
> **`commit_prec_gain` (NEW, default 0 = byte-identical):** the commit window, ramp and release
> scale with how well the body predicts **itself** — the forward-model residual's shortfall
> against its **own running mean**, so it is scale-free (doctrine §6) and nothing is tuned to
> `tle`'s magnitude. Predicting better than usual ⇒ shorter window, faster ramp, slower release.
> **★ It can engage INSIDE a 1–2 s burst if that burst is genuinely predictable — which a fixed
> 180-tick (3 s) window structurally cannot, and that timescale mismatch is the operator's whole
> complaint.**
>
> n=1, seed 6: gain 2.5 → **stalled 11 %**, the lowest measured; disp/s and metres flat, so it
> buys **continuity, not speed**. ⚠ **NON-MONOTONE — gain 1.0 was WORSE (19 %)**, so 2.5 may be
> luck. A {1, 1.5, 2, 2.5, 3} sweep at n≥4 is owed before any claim. `IN_FLIGHT`, whitelisted
> for UI observation first, because the 3-point stall metric has already been blind to the
> shaking-vs-stepping distinction twice today.

**Operator's architectural call, recorded because it is the right one:** commit's dynamics
belong to the higher active-inference loop as a lever, not to a constant. The internal law above
is half of that; the other half is a `commit_topic` socket mirroring `goal_bearing_topic`, so the
EFE arbiter can command "commit hard, I know where I'm going" vs "release and search". Plan §1.1
already says the arbiter's currency is *a motor objective, not a heading* — this is one.

---

### ★★★ 2026-08-05 — `step_cv` NEVER MEANT WHAT WE READ IT TO MEAN (operator-driven)

**Operator observation, from UI runs on seed 6 with the path trail on:** *"the robot will pause
and fumble for about a second, then take two or three steps, up to four but never more, then
resume fumbling — even with the most confident configs. When there is good synchronization the
traversal speed is very good; it is the pauses that slow it down."*

**Measured at 5-tick resolution, this is confirmed and it overturns a load-bearing reading.**

| | value |
|---|---|
| step-rate CV, 5-tick bins | **5.6** |
| CV of a memoryless (Poisson) process | **1.0** |
| pooled `step_cv` as reported for the whole campaign | ~1.0 |

⇒ **Pooled `step_cv` ≈ 1.0 does NOT mean "the intervals are memoryless".** A two-state process —
tight clusters of steps separated by long silences — returns CV ≈ 1 *even when the rhythm inside
each cluster is regular*. Averaging collapses the two regimes into a number indistinguishable
from randomness. **The claim "THE BODY HAS NO STEP PERIOD", which invalidated the premise of all
NINE timing levers, is an artefact of the statistic.** Those levers may still be refuted; the
*reason* recorded against them ("they presuppose a rhythm the contact signal lacks") is not.

#### What the stall is, and what it is not

Per-tick capture (arena, seed 6, `stroke12`, ticks 3101–4000), using **true physics contact**,
not the swing detector:

- 70 swing episodes in 900 ticks, durations **3–21 ticks**, mean **3.3 feet planted**, and they
  mostly overlap. The legs do NOT stop for 800 ticks.
- The operator's ~1 s pauses are real and measurable as the **long** gaps: **62, 44, 30, 25,
  20 ticks** — the 62 is almost exactly one second.
- **The stall is genuinely stepping STOPPING, not stepping without purchase.** Across 1-second
  windows sorted by displacement: displacement varies **5.0×** between stalled and moving, and
  swing activity varies **2.0×** with it (0.48 → 0.96 feet airborne). Stepping-in-place would
  have held swing activity flat. It does not. ⇒ **the "half the power stroke is spent in the
  air" mechanism is NOT the cause of the pauses.**
- **Nothing body-side predicts a stall.** `y`, `gc_raw`, `tilt`, `planted` all peak at lag 0 and
  decay to noise within ±10 ticks; `amp_gain`, `swing_frac`, `clip_duty`, `motor_tle`, `scrub`
  differ <10% between states. ⇒ the gate is controller-internal, which is what justifies putting
  the remaining candidates (swing-gate threshold, `amp_gain`, the coordination overwrite) into
  the per-tick ring.

> #### ⚠ TWO OF MY OWN READINGS WERE WRONG, BOTH THE SAME SHAPE
> 1. **"The pause is a crouch."** Retracted. `planted` −0.658, `y` +0.339, `gc_raw` +0.358 all
>    peak at **lag 0** — they describe the mechanics of a footfall (a leg lifts, one fewer foot
>    is down, the body rises slightly), not a postural cycle that ends a burst. A lag-0
>    correlation with a state variable derived from the same event is circular.
> 2. **The height homeostat as the cause.** Refuted before it cost anything:
>    `corr(|fwd_v|, h_bias)` = −0.057 / +0.021 / −0.015 across arms, flat at every lag to 200
>    ticks. `height_rest_frac` fading the postural defence out while walking was a clean story
>    and it is not what happens.
>
> **And the trigger that produced the first analysis was slicing on the wrong events:**
> `_leg_lifted_count` is the SWING DETECTOR's event, not contact, and it fired ~8× too often
> (95-tick median "gap" against the operator's observed ~800). Any burst statistic keyed on it
> is measuring the detector.

#### Operator UI ranking (arena, seed 6, path trail, all `substrate: hinge`)

`ik_plumb` (5.66 m walked) and `stroke12` (5.65 m) are the two cleanest directed paths and the
operator's top two independently of the metrics. `nolearn2` (1.72 m) and `pairinit` (2.72 m)
walk closed loops. **`trim3` is a wide ARC — confirmed by eye**, which is exactly the failure
mode predicted for it: `|turns|` fell 3.3× while `straight` did not move, and a curve that nets
near zero produces that signature. **`heading_trim_rate` 3e-4 is refuted; 1e-4 stands.**

#### New instruments (all default-off)

- **Per-metre timestamped waypoints** (operator's design) — the same object the red `[P]` trail
  draws, in the log, so the analysis and the picture the operator trusts are one measurement.
  Reveals path speed is steady (1.8–1.9× spread/metre, median ~287 ticks/m) while the robot
  walks **29 m of path to gain 5 m of ground** — direction, not speed, is the distance cost.
- **`OGMA_PICRAWLER_CLIP_WINDOW="a,b"`** — per-tick dump of a named interval. Makes no claim
  about what a step is; hands over every tick and lets the analysis find the structure. The
  primary burst instrument.
- **`OGMA_PICRAWLER_BURST_PROBE=1`** — ring + auto-trigger saving ONSET/GAP clips for
  `clipdiff.py`. Kept, but see the detector caveat above.
- **Per-leg hip1 saturation** (`clip_h1_leg`, `pre_h1_leg`) — the pooled figure had been hiding a
  3× per-side split for the whole campaign.
- ⚠ **`[F1]` was DOUBLE-BOUND** — the graph panel and the clip recorder both claimed it, so
  "save a GOOD clip" silently opened the brain graph and the clip machinery was unreachable by
  its own documented key. F1 now belongs to clips; `` ` `` opens the graph.

---

### ★★ 2026-08-04 — L1 NAV STAGE 1: THE LOOP IS CLOSED AND VERIFIED, AND IT IS NOT YET DOING TAXIS

**Built and verified.** `beacon (honest colour) → RunTumbleNavV2 → percept.klino_heading →
MotorEPM.goal_bearing_topic → the existing heading PD → the gait (untouched)`.

- **The hook.** The PD's P term was `gain·(−heading_bearing_)` — setpoint implicitly zero, "hold
  the spawn bearing". The nav module emits an **egocentric** unit vector, so `atan2(vx,vy)` *is*
  the bearing error and replaces that term. The D term is untouched, so the yaw damping that
  produced the variance collapse now applies to the **new** setpoint: **the PD becomes the nav
  layer's actuator rather than its competitor.**
- ⚠ **Deliberately NOT via `nav_topic`** — `nav_on` gates the entire heading PD off (P *and* D)
  plus the forward facing gate, which is how the oracle path discards the very thing the PD is
  for.
- **`RunTumbleNavV2`, not `Klinotaxis`** — the latter has no behavioural measurement anywhere and
  no confidence output. The former is what the Cell deploys (n=20, 1.9 eats vs a 0.9 random-walk
  floor) and ships its own control arms. Its `eat_topic` calibrates confidence only; the policy
  is reward-free (§5.1 safe).
- **Guards, all by measurement:** unit test `GoalBearingSocketSteersAndSilentIsByteIdentical`
  (A/Z/N — a configured-but-silent socket must be byte-identical to no socket, a live one must
  diverge, plus the consumer check); 36/36 `test_motor_epm` pass; the deployed baseline
  reproduces **4.58 ± 0.27 per-seed** with the nav layer absent.
- **Consumer verified live:** `gb_msgs = 5999` over a 6 000-tick run and `gb_err` varies
  (−0.92 … −0.27) — the socket is receiving and the setpoint is moving, not latched.

> #### ⚠ BUT: `ablation:"shuffle"` PRODUCES A BYTE-IDENTICAL TRAJECTORY
> The shuffle arm is the module's own **random-walk floor**. Full taxis and shuffle give
> identical `gb_err` traces, identical trajectories, and identical `gb_msgs` on every seed.
> The module is instantiated and publishing, so this is not dead code — **it means the full
> taxis is currently operating AT its kinesis floor**, i.e. it has established no directional
> belief (R≈0) and is therefore doing exactly what the gradient-blind arm does.
>
> **That is what Stage 0 predicted.** The beacon is in frame ~50 % of the time, SNR only clears
> 1 at run lengths ≥2 m, and with `pyramid_max_r = 9.5` plus an **antipodal** target router the
> robot **starts at 10–19 m, outside the informative envelope entirely** — so there is no
> gradient to build a belief from. The loop is wired correctly and is being fed a flat field.
>
> **Verdict: `DEFERRED`, not `NULL`.** Nothing here has been tested at power, and a null against
> a field the sensor cannot resolve would be a fact about the arena, not the mechanism.
> **Next, in order:** (1) shrink `pyramid_max_r` (or spawn nearer) so the robot begins inside the
> ~8 m envelope; (2) confirm the module's own `R` / tumble-rate diagnostics distinguish the arms
> once it has a gradient — right now nothing surfaces RunTumbleNavV2's internal state to the
> JSONL, which is the next instrument to add; (3) only then run the (c) controls at power.

**Also honest:** `gb_err` is one-signed for the whole run. That is consistent with "holding a run
direction while the PD chases it" (the expected run-and-tumble shape) but is *also* what a
wrong-sign hook would look like, and the two cannot be separated on a flat field. **Sign must be
verified against behaviour once there is a gradient to climb** (plan trap 11) — it is not yet
established.

**New topics:** `reality.proprio.ego_heading` (dead-reckoned from the **modelled body-frame
gyro**, so it drifts exactly as hardware dead reckoning does — honest, and RunTumbleNavV2 only
needs the frame self-consistent), `reality.proprio.vel_ego` (⚠ **soft oracle**, world velocity
projected — used ONLY by the module's stuck check, never by the gradient, and named rather than
hidden), and `events.beacon_reached`.

---

### ★★★ 2026-08-04 — THE CAMERA WAS AN ORACLE, AND THE OPTICS WERE ARBITRARY (L1 nav, Stage 0)

**Two defects in the vision path, found while scoping the nav loop. Neither errored.**

**1. The camera never read the purple material.** `_capture_vision()` decided a pixel was purple
by comparing the raycast hit's **collider node identity** against
`_pyramid_meshes[walk_target_idx].get_parent()`. It never sampled `albedo_color`. So the
scene-graph purple was a **human-visible HUD affordance only**, and `host.video.color` was an
**oracle-segmented image** — the ground-truth answer to "which pyramid is the target" painted
into the raw pixels, upstream of `epm_color` and of everything downstream. `vision_compass` was
laundered twice: an oracle image, and a readout regressed onto `target_compass`
(`bearing_r = 0.440` — a number that was never real). This is `plan.md` trap 6 exactly.
**Fixed:** classification now keys on the surface's **own albedo** (a colour→bool map rebuilt
whenever the beacon is recoloured). The pyramid reads purple *because it is purple* — which any
camera can do — and it degrades honestly: paint two and both show; move the beacon and the
purple moves.

**2. The optics were never justified against hardware.** `VISION_RES = 32` / 90° square carried
no citation; the path *"originated as a HUD debug panel that was promoted to a sensor"*. Compare
`docs/servo_dynamics.md` — a cited reference, a conformance table, four argued deviations.
**Now modelled on the real sensor** (operator): SunFounder **OV5647**, quoted 65°, which is the
**diagonal** — its published 53.5° × 41.4° computes to a 64.4° diagonal, confirming the reading.
The grid is now **32×24, true 4:3 with square pixels**, and a **var** (`OGMA_PICRAWLER_VIS_W/H`,
the Cell's `OGMA_VIS_RES` precedent) rather than a const.

> **★ Faithful optics were BETTER AND CHEAPER.** 768 rays against the old 1024 — **25 % fewer** —
> for ~3× the beacon area at range, because a narrower FOV over the same budget resolves finer.
> The arbitrary 90° was actively costing us.

Also: **`LOOM_RAY_LEN` was 8.0 m**, justified as "arena ~5 m diag", against an arena of radius
9.5 whose target router deliberately picks the **antipodal** hemisphere ⇒ **93 % of target legs
began beyond the ray length**, so loom *and* camera were identically zero for most of every leg.
Raised to 20 m (past the diagonal); cost is per-ray distance, not ray count.

#### Stage 0 — the sensing envelope, measured before any loop was built

New honest scalar `reality.proprio.beacon` = the fraction of the frame that is beacon-coloured,
computed at **full ray resolution** and **deliberately not routed through an EPM**: a GNG
coarse-grains into a discrete vocabulary, which is right for "what kind of place is this" and
wrong for a gradient — node-quantising it destroys the signal run-and-tumble climbs. (It is also
computed *before* the JL encoder's fixed `{24,24,3}` resize, `encoder_jl.cpp:26`, so raising the
grid actually buys the nav loop range; it would not if this went via the encoder.)

Measured, arena, 9 000 ticks, conditioned on the beacon being in frame:

| range | beacon (32×24) | in px | σ | SNR @1 m run | **SNR @2 m** | **SNR @3 m** |
|---|---|---|---|---|---|---|
| 1–2 m | 0.0784 | 60.2 | 0.049 | 0.61 | 1.22 | 1.83 |
| 2–3 m | 0.0484 | 37.2 | 0.032 | 0.59 | 1.18 | 1.76 |
| 3–4 m | 0.0294 | 22.6 | 0.013 | 0.89 | 1.77 | 2.66 |
| 5–6 m | 0.0151 | 11.6 | 0.005 | 1.18 | 2.36 | 3.54 |
| 6–8 m | 0.0069 | 5.3 | 0.002 | 0.76 | 1.53 | 2.29 |

**★ SNR is set by RUN LENGTH, not by resolution.** At 48×36 (2.25× the rays) the signal is
2.25× larger *in pixels* and σ scales with it — **SNR is unchanged at ~0.5–1.1**. So the noise is
pose/aspect variation as the body walks, **not quantisation**, and buying pixels cannot fix it.
What resolution *does* buy is **visibility: 50 % → 65 %** of samples with the beacon in frame,
for **+8 % wall-clock**. **Gate: PASSES provided runs are ≥2 m** — which is the regime
run-and-tumble naturally operates in (it runs while the signal rises).

**Honest caveats.** The beacon is in frame only ~50 % of the time — correct, and precisely why
direction must be *acted out* rather than sensed. Beyond ~8 m the blob is ~1.6 px and the loop is
effectively blind, so with antipodal targets at 10–19 m the robot **starts blind and must
search** before it can climb; either shrink `pyramid_max_r` or accept a long search phase. And
the `8–12 m` row reads `d_run = 0` only because it is the last band and has no neighbour to
difference against — an artefact of the analysis, not a measurement.

**Instruments:** `scripts_tools/beaconprobe.py` (the gate above; conditions on visibility,
because pooling in-frame and out-of-frame mixes two populations and σ then exceeds the mean).
`beacon` / `vis_wh` / `tgt_range` in the JSONL — ⚠ `tgt_range` is **god's-eye, diagnostic only**,
and exists solely so the sensor can be characterised.

**Verified byte-identical with the camera off** (`publish_vision` default false): net_z
4.58 ± 0.27 per-seed, tilt_sd 0.065, 0 falls — three times across the change set.

⚠ **A silent confound caught mid-measurement, worth recording as a shape:** the first
resolution sweep returned **byte-identical numbers for 32×24 and 48×36** because
`OGMA_PICRAWLER_VIS_W/H` had been added as `@export` vars but never wired to the env parser. The
only reason it was caught is that two arms that *must* differ did not. §3.2 rule 7 again.

---

### ★★★ 2026-08-04 — RETRACTION: "NOTHING HERE IS COORDINATED" WAS MEASURED AGAINST AN UNREACHABLE CEILING

**A scripted, open-loop, perfectly periodic TROT — phase offsets hard-coded to [0, π, π, 0],
every leg driven from one shared 60-tick clock, coordinated BY CONSTRUCTION — scores
`plv_w` = 0.153. The emergent deployed gait scores 0.195.** The designed gait is *less*
phase-locked, on this instrument, than the one we have.

| arm | `plv_w` | what it is |
|---|---|---|
| random-phase null | 0.090 ± 0.066 | 40 sims, independent legs |
| **scripted TROT (imposed, periodic)** | **0.153 ± 0.016** | coordinated by definition |
| scripted WALK (lateral-sequence) | 0.185 ± 0.007 | coordinated by definition |
| **deployed emergent gait** | **0.195 ± 0.008** | — |
| perfect lock | 0.984 | simulation; **unreachable on this body** |

⇒ **`plv_w` ≈ 0.2 is at or near the practical CEILING for this body, not near the floor.**
The whole scale was wrong: both endpoints were established by *simulation*, the top one was
assumed to be attainable, and the middle was never filled in. **The 2026-08-03 finding — "No
configuration tested has ANY inter-leg coordination… it is absent, and has never been
present" — is not supported by this measurement and is RETRACTED.** So is every downstream
reading of six mechanisms "returning the random-phase null": they were being scored against a
ceiling nothing can reach, including a gait whose phases are literally hard-coded.

**Mechanism — the phase ESTIMATOR is the bottleneck, not the gait.** `L.phase =
atan2(knee velocity · scale, knee deviation)` is a noisy readout of knee *state*, while the
scripted gait imposes its rhythm on the *command*; measured knee motion is dominated by ground
reaction, the postural reflex and load. PLV computed over a noisy phase estimate is bounded
well below 1 however coordinated the legs actually are. **Any future coordination claim must be
calibrated against the scripted-gait line, not against 1.0.**

> #### ★★ AND THERE IS REAL STRUCTURE — THE DIAGONALS, WHICH IS THE TROT SIGNATURE
> Per-pair `plv_w` on the deployed gait (new per-pair instrument, since the pooled mean cannot
> express this):
>
> | FL-FR | FL-RL | **FL-RR** | **FR-RL** | FR-RR | RL-RR |
> |---|---|---|---|---|---|
> | 0.126 | 0.151 | **0.314** | **0.284** | 0.168 | 0.130 |
>
> **The two DIAGONAL pairs are 2.3× the two LATERAL pairs.** That is a trot signature, and it
> is the first quantitative evidence of gait *structure* this project has produced. It
> corroborates the operator's UI observation of "diagonal symmetry" — which the pooled metric
> had been averaging into invisibility for the whole campaign. It also survives a lesion
> (FL-RR stays highest at 0.268 with the RL servos dead).

#### The tripod — the operator saw something real, but it is the good tail, not the mean

Operator observation: with one rear leg ablated the tripod "seems faster than the quadruped
gait". Measured, deployed base, n=4, leg RL:

| arm | m/s | net_z | straight | steps | survivor `plv_w` | support |
|---|---|---|---|---|---|---|
| intact | **0.051** | 4.58 ± 0.27 | 0.73 | 50.0 | 0.203 | 1.000 |
| RL servos dead | 0.023 | 2.32 ± 1.32 | 0.48 | 39.2 | 0.174 | 0.999 |
| RL limb removed | 0.010 | 0.95 ± 0.85 | 0.13 | 26.5 | 0.174 | 0.501 |

**On average the tripod is 2–5× SLOWER, not faster.** But it is strongly **bimodal**: with the
servos dead, seed 2 reached net_z **4.36 at straight 0.79** — matching the intact gait on three
legs — while three other seeds sat at 1.1–2.6. **A competent three-legged gait exists in this
substrate's repertoire and is found unreliably (~1 seed in 4).** That is a more interesting
result than either "faster" or "slower", and it is a far better target than another timing
lever: the capability is already there, and what is missing is *reliably finding it*.

**Instruments:** per-pair windowed PLV (`plv_win_pairs` / `plv_win_pair_sup`), because with one
leg dead its three pairs decay toward 0 and halve the pooled mean — visible above as
`rear_gone` pooled 0.087 (support 0.501) against survivor-only 0.174. Tool:
`scripts_tools/plvladder.py`.

⚠ **`step_cv` reads 0.000 in every arm of this ladder and that is NOT a measurement** — the
deployed `imufused` config leaves `gait_align_diag` unset, so the whole block is skipped. The
exactly-round-null rule catching itself again; any `step_cv` comparison must use a config that
sets it (`nolearn2` does).

---

### ★★★ 2026-08-04 — THE CHASSIS HAS NEVER COLLIDED WITH ANYTHING (operator-found)

**Operator observation, from the new ablation bench: "when I lesion both rear legs the entire
back of the robot sinks into the ground — the hip1 joints collide, but the chassis itself does
not."** Correct. `_LAYER_CHASSIS` appears in **exactly two places** in `picrawler_body.gd` — the
constant and the assignment. **Nothing masks it, in either direction:** the chassis carries a
real `BoxShape3D` and passes straight through the floor, the hump, the rumble strips and the
pyramids. Only the leg segments (`_LAYER_BODY`) ever touch the world.

**It was deliberate, not an oversight** — the constant's own comment says it "prevents the
box-on-floor rolling instability that flips the body upside-down even when `leg_strength=0`."
A physics workaround that became a permanent property of the substrate.

**Measured, the operator's exact scenario** (deployed base, both rear legs detached at hip1):

| | chassis_y after | min | belly `gc_raw` |
|---|---|---|---|
| **ghost (every historical run)** | **−0.020** | **−0.028** | 0.000 |
| solid | +0.043 | +0.025 | — |

**The body sits 28 mm BELOW the floor plane.** It does not rest on the ground; it descends
through it until the legs catch.

#### What it costs the record

**Intact flat walking is shifted, not overturned** (deployed, n=4, corridor): net_z 4.58 → 5.13,
steps 50 → 60, tilt_sd 0.065 → 0.074, straight and belly tie, **0 falls in both — the rolling
instability the workaround existed for did not reappear.** So lever *rankings* on healthy
locomotion probably survive. Three things do not get off that lightly:

1. **Every belly / clearance claim.** `bellyc_min` has sat at **0.000–0.004 across the entire
   campaign** and was read as "just barely clearing". On a ghost chassis it means the belly is
   **at or through the floor**. "Belly-up SOLVED" and the `stance_lift = 0.5` promotion were
   measured on a body that *cannot experience belly contact*, so the invariant they defend was
   never tested. The ToF rangefinder was honest throughout — nothing ever stopped the gap it
   reported from reaching zero.
2. **Hump traversal.** §5 already suspected this and named it "may be a sim exploit
   (frictionless belly drag)". The real mechanism is worse: there is no belly contact at all.
3. **Anything measured while the body was fallen, inverted or damaged** — exactly the regime
   where the chassis is the part touching the ground.

#### Shipped as a LEVER, default OFF

`chassis_collides` (export · `OGMA_PICRAWLER_CHASSIS_COLLIDE=1` · **`[J]`** live), default
**false** so every historical number stays reproducible — promoting it is an evidence decision,
not a silent re-basing of the campaign. Gain-0 guard verified by measurement: OFF reproduces the
deployed baseline 4.58 ± 0.27 per-seed. Both directions are wired (chassis masks `_LAYER_WORLD`
*and* world bodies mask `_LAYER_CHASSIS`) rather than relying on one-sided matching, and the HUD
always shows `[J] chassis: SOLID/ghost` — which mode a run used is a physical fact about the body
that appears in **no config file**, the same class of silent overlay `gaitreport.py` scrapes for.

**Re-run under `chassis_collides=1` before trusting, in priority order:** the belly-up /
`stance_lift` promotion · the hump gate · the inversion + recovery gates · anything in §6 whose
re-use context mentions belly clearance (notably `tibia_plumb_gain`, held back ONLY by belly).

---

### ★★★ 2026-08-04 — INFERENTIAL COUPLING (`couple_prec_gain`): `NULL` on the healthy gait, `WORKING` on the (d) test — and the two together are the result

**The lever.** The Kuramoto term averaged the other three legs **uniformly** (`c / (n_legs−1)`,
`MotorEPM.cpp:2910`). That divisor is itself a hand-set precision — every leg trusted equally,
forever — which is what makes an otherwise legitimate *innate* reflex a **script** rather than
inference (doctrine §2.3: precision is a CONTROLLED variable; "a designer picking the crossover
point is the anti-pattern"). `couple_prec_gain` (`k`) replaces it with a precision-weighted mean,
`w_j = (amp_j/(tle_j+ε))^k`, L1-normalised — the LateralVoter's own idiom one layer down.
**Scale-free by construction:** `prec^k / Σprec^k` is invariant to a common factor, so nothing is
tuned to the residual's magnitude; `k` is a sharpness exponent. **Gain-0 guard exact and verified
by measurement** (`pow(x,0)=1`, and the divisor falls back to the legacy `n_legs−1`): per-seed
byte-identical.

**Base:** `..._nolearn2.json` — all controller learning off, the n=20 ablation *winner*. Chosen so
the precision weighting is the **only inferential thing left in the controller**; if the (d) test
moves, nothing else could have done it. (`pure_hk` is not available for this lever: `coupling_gain`
and `stroke_gain` are both 0 there, so modulating them is a §3.2 tautology.)

#### 1. Unperturbed, n=6, corridor, 6000 ticks — `NULL`, trending mildly negative

| `k` | net_z | straight | `plv` | `plv_w` | `cw_spr` | steps | `step_bal` |
|---|---|---|---|---|---|---|---|
| **0** (control) | 4.83 ± 0.41 | 0.73 | 0.15 | 0.20 | **0.00** | 45.0 | 0.31 |
| −1 (wrong-sign) | 4.39 ± 0.86 | 0.69 | 0.14 | 0.19 | 0.28 | 60.8 | 0.32 |
| +0.5 | 4.95 ± 0.53 | 0.72 | 0.15 | 0.20 | 0.14 | 41.8 | 0.31 |
| +1 | 5.10 ± 0.15 | 0.72 | 0.14 | 0.18 | 0.28 | 46.3 | **0.44** |
| +2 | 4.93 ± 1.07 | 0.68 | 0.12 | 0.18 | 0.60 | 70.0 | **0.44** |

**Consumer verified** (§3.2 rule 5): `cw_mean` = 1.00 on every `k≠0` arm and 0.00 on the control,
and `cw_spr` scales monotonically **0.00 → 0.14 → 0.28 → 0.60** with |k|. This is not a
measurement outcome. On a healthy body the lever does nothing good: coordination trends *down*
with |k| and distance ties. The one positive, `step_bal` 0.31 → 0.44, is **the same
participation-without-phase signature whole-body `C` produced** — the fourth mechanism to land on
that exact split.

#### 2. The (d) test — leg 0 killed at tick 2500 — `WORKING`, dose-dependent, and the wrong-sign control holds

Commanded motion of leg 0 set to 0 and left there. The world is unchanged and nothing tells the
brain, so it can only notice through its own prediction error. The lesion bit identically in every
arm (`amp_cut` 0.71 → 0.02, `tle_cut` 0.24 → 0.07), so the perturbation is matched. n=6.

| arm | `plv_w` recovered | support `plv_wn` | `plv_w`/support | `dz_rate` recovered | t vs control |
|---|---|---|---|---|---|
| **k=0 control** | 0.127 ± 0.023 | 0.669 | 0.190 | 0.011 | — |
| **k=−1 wrong-sign** | 0.121 ± 0.025 | 0.768 | **0.158** | 0.009 | **−0.43** |
| k=+1 | 0.167 ± 0.014 | 0.850 | 0.196 | 0.022 | **+3.64** |
| k=+2 | **0.190 ± 0.013** | **0.914** | 0.208 | 0.023 | **+5.84** |

**The control loses a third of its coordination and keeps falling (0.203 → 0.127); k=+2 ends
ABOVE where it started (0.169 → 0.190) and holds ~2× the forward progress.** Monotone in `k`.

**★ The wrong-sign arm is what makes this defensible.** `k=−1` carries a *comparable weight
spread* (`cw_spr` 0.92 vs k=+2's 0.98 in the recovered window) and performs like the control
(t = −0.43), with the worst locking-given-oscillation of any arm (0.158). ⇒ **The effect is not
"unequal weights"; it is weights in the right direction.** That is the (c) loop-isolation control
passing, and it is the check that separates this from a stiffness artifact.

> #### ⚠️ THE HONEST DECOMPOSITION — most of the gap is SUPPORT, not tighter phase-locking
> `plv_w ≤ plv_wn` by construction, and the arms differ in support, so the raw gap is partly
> mechanical. Split out:
>
> | | k=+2 vs control |
> |---|---|
> | raw `plv_w` gap | **+50 %** |
> | support (`plv_wn`) gap | **+37 %** |
> | locking GIVEN oscillation | **+10 %** |
>
> **So the claim is resilience, not coordination quality.** After a leg dies, precision-weighted
> coupling keeps the surviving legs oscillating and the body moving, where the uniform average
> lets the whole gait wind down. Phase-locking *given* that the legs are moving improves only
> slightly. **Mechanism, and it is the predicted one:** under a uniform average the dead leg's
> frozen, meaningless phase still commands 1/3 of every survivor's coupling authority and drags
> them down with it; the activity term collapses its weight and the survivors couple to each other.
> Saying "coordination improved 50 %" here would be the same error as the retracted coherence
> claim, one level subtler.

#### 3. ★ IT REPLICATES ON THE FIXED SUBSTRATE — re-run with `chassis_collides=1`

The (d) result above was measured on the **ghost chassis** (entry immediately below), in the one
regime where the ghost matters most: a body that collapses after a leg dies was partly sinking
*through* the floor rather than resting *on* it. That is a textbook §3.2 rule-7 exposure, so the
whole test was re-run with the chassis solid, n=6, same arms.

| arm | ghost `plv_w` recovered | **solid `plv_w` recovered** | solid support | solid t vs control |
|---|---|---|---|---|
| k=0 control | 0.127 ± 0.023 | 0.139 ± 0.018 | 0.768 | — |
| k=−1 wrong-sign | 0.121 ± 0.025 | **0.121 ± 0.015** | 0.837 | **−1.88** |
| k=+2 | 0.190 ± 0.013 | **0.187 ± 0.028** | 0.909 | **+3.53** |

**Every conclusion holds**, and the decomposition gets *better*: raw gap +50 % → +35 %, support
gap +37 % → **+18 %**, locking-given-oscillation +10 % → **+14 %**. So on the honest substrate a
larger share of the effect is genuine phase-locking and less of it is bookkeeping. The wrong-sign
arm sits slightly *below* the control (t = −1.88), which is a cleaner control result than the
ghost run gave. **No retraction required — recorded because a result that survives a substrate
fix is worth more than one that was never tested against it.**

#### 4. On the DEPLOYED base it is worse, and that is consistent

`..._imufused__cp20.json` (the CURRENT launcher entry + `couple_prec_gain=2`), n=4 unperturbed:
net_z 4.72 ± 1.01 (ties, **4× the variance**), straight 0.69 vs 0.73, `plv_w` **0.16 vs 0.20**,
tilt_sd 0.065 → **0.116** (one seed 0.210), steps 50 → 85, 0 falls, `cw_spr` 0.63. **Strengthening
the learned path on the deployed base makes things worse** — the same direction as the `ctrl_lr`
result (0.05/0.10 on the deployed base destabilised) and the same explanation: the scaffold gains
were hand-tuned around a *weak* learned signal, so both amplifying and removing it move away from
that tuning. The arm is allowlisted so the lever can be ablated on the robot that actually ships,
not because it is a better walk.

**Verdict: `NULL` unperturbed (mild `REGRESSION` on `plv` at high `k`, clearly worse on the
deployed base), `WORKING` under perturbation on BOTH substrates, at n=6 = a SIGNAL, not a
finding.** The effect is loud (t = 5.84) and dose-dependent
with a clean wrong-sign control, which is the §3.3 bar for "promote the direction". **Not promoted
into the deployed stack**, because on the healthy gait it costs a little coordination and buys
nothing — this is a *damage-response* lever, and the honest place for it is default-off behind the
case that motivates it.

**To make it a finding:** n≥20 with varied world seeds; the arena as well as the corridor (the
corridor's self-centering walls mechanically assist a hobbled body); and a second lesioned leg to
show it is not leg-0-specific. **Also worth testing next:** the same weighting on a *recoverable*
perturbation, once one exists — see the binary-degradation box below, which is why this had to be
tested against a dead leg rather than a weak one.

---

### ★★★ 2026-08-04 — A DAMAGED LIMB IS *MORE* PREDICTABLE, WHICH INVERTS THE PREMISE OF PRECISION-WEIGHTED REFLEXES

**This entry is about the substrate, not the lever, and it is the more important half of the
day.** It was found while building the (d) test for `couple_prec_gain` (below) and it
constrains every future proposal in the inferential-gain family.

**The direction assumed** that a leg in trouble produces *more* prediction error, so
weighting by `1/(tle+ε)` would down-weight it. **Measured, on the `nolearn2` base, cutting
leg 0 mid-episode (n=3, tick 2500):**

| lesion | `tle_cut` pre → acute → recovered | `amp_cut` pre → acute → recovered |
|---|---|---|
| torque ×0.2 | 0.236 → 0.232 → 0.255 | 0.708 → 0.672 → 0.719 |
| torque ×0.0 | 0.235 → 0.217 → **0.165** | 0.725 → 0.317 → 0.189 |
| action ×0.3 | 0.235 → 0.191 → 0.181 | 0.725 → 0.717 → 0.724 |
| action ×0.0 | 0.235 → 0.106 → **0.061** | 0.725 → 0.124 → 0.016 |

**The cut leg's own forward-model residual goes DOWN in every condition, monotonically with
severity — to 26 % of baseline when the leg is dead.** A limb that moves less is easier to
predict. So a precision weighting driven by prediction error alone would have *increased*
its trust in the damaged leg: the exact opposite of the intent.

⇒ **On a motor system, prediction error is not a proxy for competence — activity is.** The
`amp_ema` numerator in `w = (amp/(tle+ε))^k` was added as a guard against the LateralVoter's
documented flat-channel trap (`LateralVoter.cpp:80`) and against the coord-fitness lesson
("the activity term is the homeokinetic normalisation that kills both"). It turns out not to
be a guard against an edge case: **it is doing the entire job.** At action ×0.0 the weight
falls 0.725/0.235 = 3.09 → 0.016/0.061 = 0.26 — a 12× down-weight, produced wholly by the
activity term while the error term pushed the other way. **Any future `1/(tle+ε)` reflex
weighting on this body must carry an activity term, and the burden is on the proposal to say
what its activity term is.**

> #### ⚠️ AND THE BODY HAS NO GRADED DEGRADATION REGIME — every lesion channel is binary
>
> Two independent channels were tried and both are all-or-nothing, for two *different*
> reasons. This is a harness finding that any future (d) test has to design around.
>
> 1. **Torque ceiling — the body is not torque-limited.** At ×0.05 the cut leg's amplitude is
>    0.720 against 0.725 untouched: unchanged. `tq_sat` = **0.009**, i.e. the servos are
>    saturated under 1 % of the time, so the ceiling is not the binding constraint and
>    scaling it removes headroom the body never used. Only ×0.0 does anything.
>    **Corroborates two existing entries from a third direction:** "authority was never the
>    binding constraint" (§4) and the gravity-scaffold `NULL` ("servos have ~4× headroom").
> 2. **Commanded action — an existing homeostat silently absorbs it.** At ×0.3 the leg's
>    commanded motion is cut 70 % and its *measured* amplitude does not move (0.725 → 0.717
>    → 0.724), because `amp_homeo_gain = 0.01` is live on `nolearn2` (it is not an `*_lr`
>    param, so the "all learning off" ablation never touched it) and drives
>    `L.amp_gain += g·(amp_target − L.amp_ema)` until the amplitude returns to target.
>    `kAmpGainMax = 5.0` gives it ample range to cancel a 3.3× deficit. Forward progress
>    *does* halve (`dz_rate` 0.058 → 0.031), so the compensation is real but not free.
>
> **So the only severity that perturbs coordination is a dead leg, and a dead leg makes the
> body collapse rather than adapt** (`dz_rate` 0.058 → 0.012, and `plv_win` support falls to
> 0.90 acute / 0.66 recovered — the instrument itself starts going out from under the
> measurement, which is the 2026-08-03 rule arriving yet again).
>
> **★ The positive finding hiding in (2): the body ALREADY re-organises around a degraded
> limb, and it does it with the amplitude homeostat, not with coordination.** That is an
> adaptive response to an un-signalled perturbation — a (d)-shaped result — produced by a
> mechanism nobody proposed for it. It is worth measuring deliberately.

**New instruments this required** (all report-only, all verified against a byte-identical
base): per-leg `tle_leg[4]` / `amp_leg[4]` (the body-level `motor_tle` collapses exactly the
question this direction turns on), `panic_eff`, and `interleg_plv_win` — a trailing-window
PLV (EMA phasor, τ ≈ 500 ticks). **`interleg_plv` is a whole-run accumulator and therefore
structurally cannot express a before/after**, so until now no perturbation could be scored on
coordination at all. Its null was **measured, not derived** (40 sims): **0.090 ± 0.066
random-phase, 0.984 locked, 0.200 ± 0.010 on the deployed/`nolearn2` body.** And
`seedavg.py` now parses `plv`/`plv_n`/`step_cv`/`coh`/`motor_tle` — they have been in the
JSONL since 2026-08-03 and the parser never read them, so **every A/B since has scored
distance and steps while the operator's actual complaint went unmeasured.** A metric that
exists but is unparsed is exactly as invisible as one that was never emitted.

Also settled, and it removes a suspected confound rather than adding one: **`panic_duty` =
0.00 on every seed.** Panic never fires on this base, so the `(1 − pe)` factor that already
scales the coupling, stroke and rhythm terms is inert here and is not silently doing the work.

---

### ★★★ 2026-08-03 — THE SUBSTRATE WAS NEVER WHAT EITHER OF US THOUGHT (four defects, one chain)

**Operator-driven, from a single observation: "with stiffness and damping at zero the legs
slowly drift down then twitch back up — that is not expected."** Chasing it exposed four
compounding defects in the body, every one of which reported no error.

**1. Two different robots.** `resolve_picrawler_joint_backend()` returns the launcher's persisted
choice when launched from the UI, but falls through to the `@export` default `"hinge"` headless.
**Every headless measurement in this campaign ran `hinge`; every operator UI session ran
`g6dof`.** The same config gives a different body depending on how it is started.

**2. `joint_angular_damping` is g6dof-ONLY** (reaches the solver solely via
`Generic6DOFJoint3D.PARAM_ANGULAR_DAMPING`) and sits at **1.5** in the preset. So the operator
observed every homeokinetic result through a heavy joint low-pass that the headless numbers never
had. Operator: dropping it to 0.5 "allows the robot to move much, much faster" — **the largest
behavioural lever found on this body**, and it had no env override, so it had never been swept.

**3. `_apply_g6dof_default_preset()` silently clobbered EVERY env override.** Its comment claimed
"env vars still win because they run AFTER this baseline"; the env whitelist actually runs
**before** the preset inside `_resolve_env()`. So all thirteen preset keys — joint damping, motor
force scale, authority, erp, softness, all six spring params, freeplay — were overwritten with no
warning. ⇒ **Any past headless sweep over a preset key on g6dof measured NOTHING and would have
read as a clean null.** Caught only because a joint-damping sweep produced four identical arms.
Fixed: the preset now yields to env and prints which keys it yielded on.

**4. ★ THE JOINT SPRINGS HAVE NEVER EXISTED.** Measured, not inferred: `knee_spring_stiffness`
0 vs 20 with everything else fixed gave **byte-identical trajectories** (first −1.1800, last
0.9200, min −1.1900, max 1.5300 in both). The June note recorded that an `apply_torque` spring was
reverted in favour of *"`Generic6DOFJoint3D` with built-in angular spring (constraint-level
integration in **Bullet**)"* — but **Godot 4 replaced Bullet with Godot Physics 3D, which does not
implement angular springs.** The params and `FLAG_ENABLE_ANGULAR_SPRING` are set, nothing errors,
and the solver ignores all of it.

⇒ **Retract "the springs were fighting the gait"** — my explanation for g6dof measuring worse than
hinge (net_z 4.49 → 3.51, PLV 0.138 → 0.083). There were no spring forces. That difference comes
from the other preset parameters, most likely the 1.5 joint damping. **`spring_follows_target` was
also a no-op** — there was no spring to re-centre.

**Fixes shipped.** *Backlash:* the motor drove from the FULL error and merely zeroed inside the
deadband, so a drifting joint left the band, was driven back to TARGET, overshot in, and drifted
again — a limit cycle, exactly the operator's "drift then twitch". Now responds only to the error
*beyond* the band, so the joint settles AT the slop edge as real slop does. *Springs:* implemented
by scaling the MOTOR's velocity command inside the deadband — a velocity-target motor is solved at
constraint level, so it is stable where the reverted `apply_torque` was not. Verified working:
stiffness 0 → knee drifts to 1.41; stiffness 20 → held at 0.00, where before both read identically.
*Tooling:* brain now PAUSES in `[C]`/`[G]` (it was fighting the sliders **and** training its
forward model on motion it did not cause); ganged hip2/knee drive with PULSE (ring-down → damping
ratio) and SHAKE (frequency sweep → resonance) plus a measured peak-to-peak readout; joint and
body damping exposed alongside the spring sliders, since the panel had been showing one of **three**
damping sources.

**RESOLVED 2026-08-03 — `hinge` IS CANONICAL** (operator decision, after re-observing the
configurations on the fixed substrate). Three reasons, in increasing order of weight:

1. **It measures better on everything.** PLV **0.138** vs g6dof's best **0.097**; net_z 4.49 vs
   2.79; straight 0.70 vs 0.48; 0 falls vs 0.17. No g6dof damping value reaches hinge's
   coordination — the opposite of the prediction that compliance would help.
2. **It is the closer model of the hardware.** A real hobby servo is a stiff position tracker on
   a rigid gear train; its compliance is *backlash*, which hinge + `motor_freeplay_rad` models.
   A 6DOF joint with angular springs models a series-elastic actuator the PiCrawler does not have.
3. **Nothing needs re-running.** Every result in this ledger was measured on hinge, so the whole
   record stands as written — the n=20 ablation, the coordination series, the `ctrl_lr` sweep.

**Guard shipped:** the substrate is now ANNOUNCED at startup — `substrate = hinge (canonical)`, or
a `push_warning` + stdout line saying results are NOT comparable to the ledger. The resolver order
is unchanged, so g6dof stays one flag away; what changed is that it can no longer be selected
*silently*, which is what produced the two-robots split.

**g6dof is DEFERRED, not refuted** — and its verdict is now weaker than it looks, because every
g6dof number in this ledger was measured while (a) the joint springs did nothing, (b) the preset
clobbered env overrides, and (c) the backlash model had a limit-cycle bug. All three are fixed and
none of the g6dof arms have been re-run since. **Re-use context:** the compliance line becomes
live again if the physical build gains a genuinely compliant element (a springy foot, a
series-elastic joint), or if a resonance hunt on the fixed springs finds a stiffness that changes
the picture. Note that hinge has **no** joint-damping parameter and **no** springs, so the entire
compliance/resonance line is inert on the canonical substrate — testing it means deliberately
leaving canonical.

---

### ★★ 2026-08-03 — I2 (the real ∂G term) is BLOCKED BEHIND F3, not mis-dosed

`sense` implemented faithfully from `sos_avggrad.cpp` (`epsrel = diag(C·Q·A) ⊙ g'·2·sense`,
subtract `(epsrel ⊙ y)·xᵀ`), plus `ctrl_damping` — PM's `damping`, mandatory because `sat_lr`
is the only brake on the bias integrator. Both gain-0 guarded, tests pass.

| 40 k, n=6 | total steps | steps/1k early → late |
|---|---|---|
| `sense = 0` | **289** | 13.43 → 4.12 |
| `sense = 0.1` | **0** | 0.54 → 0.00 |
| `sense = 0.3` | 3.2 | 0.77 → 0.00 |
| `sense = 0.6` | 4.3 | 0.82 → 0.00 |
| `sense = 1.5` (PM's hexapod value) | 7 | 0.24 → **0.28** |

**`REGRESSION` at every dose — and at 1/15 of PM's value it still causes total paralysis, which
says the term is MIS-SCALED, not mis-dosed.** Diagnosis: the confinement term inherits the ε⁻²
blow-up from **F3**. PM's `epsrel` uses `Q = (L Lᵀ L)⁺`; ours uses `q qᵀ` with
`q = (LLᵀ+εI)⁻¹ξ`, and `Lp` is 9×9 of **rank ≤ 3**, so `P` is dominated by `1/ε = 100` and
`q qᵀ` carries that squared (~10⁴). ⇒ **F3 is not a theoretical wart: the degenerate metric makes
the principled confinement term unusable.** I2 is blocked on squaring the loop (or normalising
the term), which promotes F3 from "cheapest honest test" to a prerequisite.

**One thing worth keeping:** at `sense = 1.5` the decay *does* stop (0.24 → 0.15 → 0.28) where
the control keeps falling. The mechanism is real; only its scale is wrong.

---

### ★★ 2026-08-03 — DEP: `REGRESSION` — it amplifies the fall

DEP (Der & Martius 2015) ⚠️ **postdates our 2012 sources — reconstructed from the published
principle, not read from their code.** Replaces the HK update of `C` with the correlation of
motor derivatives against the sensor derivatives they caused, row-normalised so `dep_gain` is the
per-motor loop gain. Wired into the whole-body `C` (per-leg DEP has no cross-leg entries to
write). Gain-0 guarded; 35/35 tests.

| `dep_gain`, 40 k n=6 | steps | steps/1k early → late | PLV | **PLV support** | tilt |
|---|---|---|---|---|---|
| 0 (HK control) | 242 | 16.18 → 2.04 | 0.140 | **159 819** | **0.013–0.081** |
| 0.5 | 74.7 ± 105.6 | 8.13 → 0.00 | 0.553 | 14 018 | 0.63–2.04 |
| 1.0 | 5.5 | 0.96 → 0.00 | 0.736 | 5 310 | — |
| 2.0 | 1.2 | 0.35 → 0.00 | 0.750 | 4 011 | — |

**The PLV rise is the frozen-body artifact on a FALLEN body.** Per-seed at `dep 0.5`: four seeds
take 2–3 steps at tilt **0.63–1.16 rad (36–66°)**, two take ~235 steps at tilt **0.88 and 2.04
rad** — seed 6 is *inverted*. The HK control sits at tilt 0.013–0.081, upright. **Every DEP seed
has fallen over**; the high PLV is measured on 3 000 ticks of a tipped body against the control's
160 000.

**Why this is coherent rather than surprising: DEP amplifies whatever the body is already doing,
and our body falls.** On a body that cannot hold itself up, the rule's premise makes it worse —
the bootstrap problem arriving for the fourth time from a new direction. `dep_gain` 0.1/0.25 in
flight to bracket the low end before the verdict is final.

**Instrument note that paid for itself:** `plv_support` made this readable. Support 3 588 vs
159 820 is a 45× tell in the same table as the PLV, so the artifact announced itself instead of
being reported as a result. That is the fix for the failure two entries below.

---

### ★★★ 2026-08-03 (later) — RETRACTION: `gait_coherence` MEASURES NOTHING, AND NOTHING IS COORDINATED

**The operator caught this by eye** — "the difference between locked and unlocked seeds is
actually rather subtle… in both, the legs are trying to pull the robot in four different
directions." They were right and the metric was wrong.

**The defect.** `gait_coherence()` is the Kuramoto order parameter of the four leg phases **at an
instant**. I sampled it once, at the end of each run. For four *independent* phases that
statistic has **mean 0.450, sd 0.219** — so single readings scatter across [0, 1].

| | measured | random-phase null |
|---|---|---|
| time-mean coherence, 12 seeds | **0.453 ± 0.035** (range 0.396–0.505) | **0.450 ± 0.219** |
| fraction of readings > 0.7 | 4/12 = 0.33 | 0.147 (n=12: not significant) |

**Time-averaged, every seed sits exactly on the null.** Seed 2's `coh = 0.999` was one lucky
instant, not a locked gait.

**Retracted in full:** "coherence is bimodal / some seeds phase-lock" (that was the sampling
distribution of a random variable); "locked seeds are low and flat, tilt 0.071 vs 0.212" (that
compared seeds sorted by a random number); "pure-HK out-coordinates the deployed gait, 0.484 vs
0.419" (**both are the null**); and the locked-seed list written into the launcher label.
⚠️ **§5 of this ledger had already warned that coherence is maximal on a frozen body.** I used it
anyway without ever checking its null — the fifth instrument failure of the campaign and the only
one where the instrument reported *confidently* rather than reporting zero.

**Replacement instrument: `interleg_plv`** — per leg-pair `|mean_t e^{i(φᵢ−φⱼ)}|`, averaged.
It asks whether a pair holds a **constant relative phase over time** (1 for a trot at relative
phase π) rather than whether phases coincide at one instant, and unlike the order parameter **its
null falls toward 0 as the run lengthens**. Same formulation `clipdiff.py` already used.

> ### ★★★ THE SURVIVING FINDING, AND IT IS STARKER THAN WHAT IT REPLACES
> **No configuration tested has ANY inter-leg coordination.** Time-mean coherence, n=12 each,
> against a null of 0.450:
>
> | arm | time-mean coherence |
> |---|---|
> | stance · per-leg `C` | 0.436 ± 0.012 |
> | stance · whole-body `C` | 0.453 ± 0.035 |
> | belly-crawl · per-leg `C` | 0.435 ± 0.024 |
> | belly-crawl · whole-body `C` | 0.439 ± 0.038 |
>
> Every arm is the null. **The deployed scaffold stack, pure HK, whole-body `C`, and crawling all
> produce legs at random relative phase** — exactly what the operator sees. Inter-leg coordination
> is not "weak" in this project; it is **absent, and has never been present**.

---

### 2026-08-03 — BELLY CRAWL (`postural_gain = 0`): `NULL` on coordination — the support hypothesis is refuted

**Hypothesis:** PM's one clearly coherent repetitive gait is the humanoid *crawling*, where the
support problem is free (torso on the ground) and the controller works purely on propulsion. Our
picrawler must solve support and propulsion at once. So: drop the standing requirement
(`postural_gain` 0.7 → 0, the single lever that makes it try to stand) and see if coordination
appears. **Prediction: the lock rate rises.**

**Refuted.** n=12, 20 k, `c_init` 0.25 / `ctrl_lr` 0.10. Time-mean coherence **0.435 ± 0.024**
(per-leg) and **0.439 ± 0.038** (whole-body) — the null, unchanged from the standing arms.
Removing the standing requirement makes the body *more active and less stable* (`steps` 163 → 213,
falls 1.75 → 2.33, tilt_sd 0.279 → 0.366) and no more coordinated. **The missing ingredient is
not support**, and the humanoid-crawl story does not transfer.

**★ What DID survive, powered:** whole-body `C` reliably improves **leg participation** —
`step_bal` **0.253 ± 0.134 → 0.444 ± 0.131, t = +3.52** at n=12 on the crawl base (and 0.29 → 0.48
on the stance base). **The legs share the work far more evenly; they just do not time it.**
Participation and phase-locking are different quantities and whole-body `C` buys exactly one of
them. Verdict `PARTIAL`: kept for `step_bal`, does nothing for coordination, and costs stability
at `ctrl_lr` 0.10 (falls 2.33 → 5.50 on the crawl base).

---

### ★★★ 2026-08-03 — COORDINATION, MEASURED AT LAST: pure-HK OUT-COORDINATES THE DEPLOYED GAIT
> **⚠️ THIS ENTRY IS RETRACTED — see the retraction immediately above.** Its coherence numbers are
> single-instant samples of a statistic whose random-phase null is 0.450 ± 0.219. Kept for the
> audit trail; the `hip2_knee_agree` result in it stands (that metric has a real 0.5 chance line
> and was measured against it).

With the phase estimator ungated (below), inter-leg coherence and a new intra-leg
`hip2_knee_agree` were measured on the same arms. **Both confirm operator UI observations that
the metrics had been unable to see, and one reverses a conclusion.** Final values, n=4:

| arm | **inter-leg coherence** | **hip2/knee agreement** | steps | tilt_sd | falls |
|---|---|---|---|---|---|
| deployed (all scaffolds) | 0.419 ± 0.089 | **0.510 ± 0.009 — CHANCE** | 50 | 0.065 | 0 |
| pure-HK `ctrl_lr` 0.01 | **0.484 ± 0.077** | 0.465 ± 0.026 | 41 | 0.099 | 1.00 |
| pure-HK `ctrl_lr` 0.10 | 0.408 ± 0.091 | **0.555 ± 0.015** | 163 | 0.279 | 1.75 |

**1. The pure homeokinetic controller is MORE inter-leg coherent than the deployed gait**
(0.484 vs 0.419) — with `coupling_gain = 0`, no `gait_phase` in play, and no scaffolds. The legs
coordinate *through the body*, and they do it better than the Kuramoto term that was built to
make them. **"Lobotomized" describes the FAST-LEARNING arm specifically, not HK.**

**2. `ctrl_lr` trades inter-leg coordination for per-leg power.** 0.01 → 0.10 moves coherence
**0.484 → 0.408** while intra-leg agreement moves **0.465 → 0.555** and `steps` 41 → 163.
Mechanism: `C` has no cross-leg terms, so inter-leg coupling can only occur *mechanically,
through the body*, which is slow. When each leg's own loop converges faster than the body can
couple them, every leg settles into its own local solution first. **Exactly the operator's UI
report** — diagonal symmetry at 0.01, powerful but uncoordinated legs at 0.10. ⇒ **`ctrl_lr` has
NO single optimum: 0.10 for power, 0.01 for coordination. Map the trade rather than pick.**

**3. The deployed config's hip2/knee agreement is at CHANCE (0.510 ± 0.009).** Its two
foot-height joints act independently. Pure-HK at `ctrl_lr` 0.10 reaches **0.555 ± 0.015**
(t ≈ 5.2). **The synergy is an HK product the scaffolds never produced** — and the Cruse v2 rule
had to be explicitly hand-coded to drive both joints "because either alone is too weak"
(measured `corr(foot_y, joint) ≈ 0.27` each). **HK discovered unaided what the scaffold had to be
told.**

⇒ **This is the empirical case for whole-body `C`.** HK coordinates joints that share a `C`
(hip2+knee, demonstrated); the coordination it cannot do is precisely the coordination `C` cannot
represent (cross-leg). Same mechanism, wider matrix. Moves I7 from theoretically motivated to
empirically demanded.

**Shared-analysis tooling:** `scripts_tools/gaitreport.py` — self-contained HTML (no CDN, opens
offline, emailable), per-seed thin lines under the seed-mean across 10 metrics. **Rendered
summary for this entry:
[`run_summaries/2026-08-03_pure-hk_vs_deployed_coordination.html`](run_summaries/2026-08-03_pure-hk_vs_deployed_coordination.html)**
(see that directory's README for the naming/pruning convention — visualizations live in the repo,
not in scratch, so they can be re-read when a later result contradicts an earlier one). Built because two
of this campaign's largest errors were invisible in whole-run aggregates: behaviour that forms at
~10 k and decays after 20 k reads as a flat mean, and seed spread exceeding the between-arm
difference reads as a confident number.

---

### ★★★ 2026-08-03 — AN INSTRUMENT MUST NOT DEPEND ON THE THING BEING ABLATED

**Four instrument defects surfaced in two days, all the same shape**, and the fourth is the one
that names the rule:

| # | instrument | defect | consequence |
|---|---|---|---|
| 1 | `clip_duty` · `hk_share` · `echo_a` · `c_mass_*` | did not exist | saturation, the echo channel and HK's share were unmeasurable; **hip1 turned out to be clipped 56 % of the time** |
| 2 | `obj_active` · `obj_weight` | `diag_snapshot()` only | "is the objective driving?" read 0.0 in every headless run — **the socket's authority (w ≈ 0.10, and FALLING when the additive term was removed) was invisible** |
| 3 | `step_cv` family (prior session) | added to `diag_snapshot()` only | read 0.0 headless, indistinguishable from "never fired" |
| 4 | **`gait_coherence`** | `diag_snapshot()` only **AND gated on a scaffold consumer being active** | ★ see below |

**The fourth in detail, because it is the worst.** The per-leg phase pre-pass runs only when
`coupling_gain > 0 \|\| stroke_gain > 0 \|\| steer != 0 \|\| amp_homeo_gain > 0 \|\|
amp_seek_rate > 0 \|\| heading_gain != 0 \|\| nav_gain != 0`. **On the `pure_hk` tier every one of
those is zero**, so `L.phase` never updated and stayed at 0 — and because `gait_phase = [0, π, π, 0]`
makes the four unit vectors cancel *exactly*, `gait_coherence()` returned **precisely 0.000 on
every seed**.

That reads as "the legs are perfectly uncoordinated." Nothing was measured. **Stripping the
scaffolds silently stripped the instrument for the operator's central question — "do the legs
work together?" — and the resulting null was indistinguishable from a real measurement.**
Ungated 2026-08-03; the pre-pass writes only `L.phase` / `L.knee_ema` (the amplitude homeostat
inside stays separately gated), so it is behaviourally inert — **verified by measurement on both
the deployed and pure_hk arms, not argued.**

> ### THE RULE
> **When a config is stripped to isolate a mechanism, verify the INSTRUMENTS survive the
> stripping.** An instrument gated on a consumer, a scaffold, or a topic that the ablation
> removes will report a confident zero. Add to the §3.2 checks: *before trusting a null, confirm
> the instrument was live in THAT arm* — a non-zero reading somewhere in the run, or a
> deliberately perturbed control.
>
> **Corollary — beware the exactly-round null.** `0.000`, `0.5`, `1.0` to full precision are
> more often structural than measured. `gait_coherence = 0.000` was an algebraic cancellation;
> `motor_tle = 0.0000` on the windup arm was a frozen body. Both looked like data.

**Reference values now that coordination is measurable at all** (deployed base, where the phase
estimator was always live): `gait_coherence` **0.286 – 0.473** across seeds — the deployed gait is
*partially* phase-locked, not strongly. Ground force: `tq_mag` **0.376** deployed vs **0.400**
pure-HK at `ctrl_lr` 0.10 (only ~6 % more mean torque), but `tq_sat` **0.005 → 0.038**, an **8×
higher saturation duty**. ⇒ **Authority was never the binding constraint** (3.8 % saturated leaves
ample headroom); the deployed controller simply never used what it had. This corroborates the
operator's UI observation that pure-HK legs push far harder than the deployed gait ever did, and
localises the difference to *direction of force*, not magnitude — see the intra-leg entry below.

---

### ★★ 2026-08-02 — "MOVE THE SCAFFOLDS ONTO THE OBJECTIVE SOCKET": `REGRESSION`, and the reason matters

**The proposal** (from the competition finding below): scaffolds compete with the learned layer
because they are *additive terms on the same output*, which is a rewrite-rule violation —
`postural_gain·(x − x*)` added to the command **is a behavior**. MotorEPM already has the correct
path, the L-1b objective socket (`ξ̃ = (1−w)·ξ + w·(x − x*)` blended into the HK descent), and
posture is **double-implemented**: through the socket *and* additively at `postural_gain=0.7`.
So delete the additive copy.

**Instrument fix first (§3.2 rule 5):** `obj_active`/`obj_weight` lived **only in
`diag_snapshot()`** — the documented trap — so they read 0.0 in every headless run and "the
objective is driving" had never been verifiable from a seedavg arm. Now mirrored into the `mod`
dict and the JSONL.

| corridor n=6, 6 000 | net_z | straight | flat_v | chassis_y | tilt_sd | falls | steps | step_bal | **obj_w** |
|---|---|---|---|---|---|---|---|---|---|
| **CONTROL** postural 0.7 additive | **4.49 ± 0.53** | **0.70** | **0.044** | **0.058** | **0.065** | **0** | 54.8 | **0.52** | 0.10 |
| **A** postural 0, obj gain 0.3 | 1.51 ± 1.17 | 0.34 | 0.016 | 0.049 | 0.124 | 0.50 | **99.2** | 0.13 | **0.06** |
| **B** postural 0, obj gain 0.7 | 1.93 ± 1.14 | 0.41 | 0.019 | 0.047 | 0.088 | 0.17 | 97.0 | 0.11 | 0.13 |

**`REGRESSION` — two-thirds of the distance, half the straightness, a third of the flat speed,
and falls appear.** The proposal is refuted in this form.

**★ WHY, and this is a design fact about the socket, not just a failed lever.** It fired in every
arm (`obj_active`=1, 4 legs, every seed) so it is not `DEAD_CODE`. The *weight* is the problem:
`w = gain · self_precision(bin)`, and **at gain 0.3 delivered `w` is 0.10; at gain 0.7 it is only
0.13** — the objective path is **~5× weaker than the additive term it replaces** even at more
than double the policy gain. Worse: **in arm A the weight FELL when the additive term was
removed, 0.10 → 0.06** (self-precision ≈0.33 → ≈0.20).

> **The objective's authority is precision-gated, and its precision is downstream of the very
> stability it is supposed to produce.** Remove the additive term holding the body steady →
> posture becomes inconsistent → self-precision falls → the objective weakens *exactly when it is
> needed most*. **A precision-weighted objective can REFINE a posture something else is holding;
> it cannot BOOTSTRAP one.**

**Secondary, and it points somewhere:** removing the additive postural term **nearly doubled the
step count** (54.8 → 99.2) while `step_bal` collapsed (0.52 → 0.13). **`postural_gain` is doing
two jobs — holding the body up AND heavily damping the gait.** Congruent with the competition
finding: it is both the thing keeping the robot upright and the thing suppressing its gait.

**⇒ Architecture consequence.** The scaffolds that are load-bearing for *stability* cannot be
learned in from a body that is not yet stable — a bootstrapping problem. **PM solves that
bootstrap with an external anti-fall operator (a temporary prop, removable). We rejected the prop
and solved the same bootstrap with a PERMANENT INTERNAL control term** — which is arguably worse,
because it can never be de-scaffolded. **The one PM-legitimate scaffold class untried is
MORPHOLOGY:** a body whose *passive* equilibrium is standing needs no postural term to bootstrap
from. The kinematic dead-end entry already points there (feet plant at 170 mm against 166 mm
total leg reach). That is a body change — permanent rather than propped — and the only route
measured so far that **removes** the bootstrap instead of hiding it.

**Re-use context:** retry the objective route with a weight that is NOT gated by the precision of
the thing it is trying to establish (e.g. a floor on `w`, or precision measured against the
target rather than the body's own consistency) — or after morphology removes the bootstrap need.

---

### ★★★ 2026-08-02 — THE CONTROLLER LEARNING RATE WAS 5–10× BELOW THE PLAYFUL MACHINE'S, AND IT MATTERS

**The operator called this one** — *"we may see more interesting results with a steeper learning
rate (if that applies here)"* — after watching the pure-HK arm slowly grow its amplitude until
it escaped an obstacle. It applies. The source comparison recorded the gap on day one and nothing
tested it until they asked.

| | PM hexapod | PM dog | **ours** |
|---|---|---|---|
| controller rate | `epsC` **0.1** | `epsC` **0.05** | `ctrl_lr` **0.01** |
| model rate | `epsA` 0.05 | `epsA` 0.01 | `model_lr` 0.02 |

Sweep on the pure-HK self-excite base (`c_init=0.25`), 20 k ticks, n=4. **Sweep is not flat, so
`max_dctrl=0.05` is not binding and the mechanism engaged.**

| `ctrl_lr` | net_disp | **steps** | step_bal | tilt_sd | falls | turns |
|---|---|---|---|---|---|---|
| **0.01** (ours) | 0.56 | 41 | 0.16 | 0.099 | 1.00 | — |
| **0.05** (PM dog) | 0.68 | **141** | **0.42** | 0.225 | 0.75 | — |
| **0.10** (PM hexapod) | **1.79 ± 1.53** | **163** | 0.29 | 0.279 | 1.75 | — |
| **0.20** (past PM) | **2.17 ± 1.65** | **226** | 0.35 | 0.335 | 1.75 | **+3.58** |

**Steps ×4–5, displacement ×3–4. One seed at net_z 4.36 and another at 4.78 — inside the deployed
config's range (4.58) — from a controller with NO stroke, NO coupling, NO gait phase, NO heading
and NO nav.** `WORKING`, on the pure-HK base, at n=4 (a signal: net_z std 1.5–1.7 exceeds the
mean). The cost is stability (falls 1.00 → 1.75, tilt_sd 0.099 → 0.335) and, at 0.20, spin.

⚠️ **Retraction within the same day:** the 20 k windows showed `ctrl_lr=0.10` as the only arm
*improving late* (displacement rate 0.1375 → 0.1189 → **0.1524**), and I framed that as a
continuing trend. **At 40 k it does not hold** — net_disp falls to 1.46 and the windows decay
monotonically (steps/1000: 12.0 → 5.2 → **2.95**; disp: 0.107 → 0.080 → **0.042**). **It is a
PEAK around 14–20 k, not a trajectory.** The surviving claim is narrower: *PM's rate raises and
delays the activity peak by a large factor; it does not prevent the eventual decay.*

**New tool: `windowavg.py`** — early/mid/late per-tick RATES within one run. Built because the
operator watched a behaviour form at ~10 k while every whole-run aggregate said "no locomotion".
**It is also the shape the ADAPTATION question needs** (a scripted walker shows no early-vs-late
difference), which is the open instrument job from the obstacle-gate entry below.

**★ PROTOCOL CHANGE: 6 000 ticks is the DEPLOYED config's horizon and is wrong for a stripped
controller.** The pure-HK campaign's first 16 seed-runs measured the transient. Use ≥20 k for any
arm that must *find* its behaviour, and read it with `windowavg.py`, not the aggregate alone.

---

### 2026-08-02 — I4 colored proprioceptive noise: `REGRESSION` at PM's dose

PM wires every legged controller through `ColorUniformNoise(0.1)` on every sensor; our proprio
channel was noiseless (only white, motor-side, post-controller `explore_noise`). Built as a
first-order colored filter on `reality.proprio.joints` (`sensor_noise_sigma`/`_tau`, +
`OGMA_PICRAWLER_SENSOR_NOISE`, seeded off `OGMA_SEED`); σ=0 byte-identical, guard verified.

**σ=0.1 on the best pure-HK arm, 40 k, n=4: net_disp 1.46 → 0.45**, steps 240 → 185. Calmer
(tilt_sd 0.208 → 0.165, falls 1.75 → 1.25) but that is what a weaker gait looks like.
**★★ But σ=0.03 is `WORKING` ON POSTURE, and the dose response is NON-MONOTONIC.** Transport falls
monotonically with σ (net_disp 1.46 → 0.88 → 0.45), so the transport `REGRESSION` stands. The
window split shows what the aggregate hid:

| chassis_y per window | early 0–10k | mid 13–26k | late 27–40k |
|---|---|---|---|
| σ = 0 | 0.0424 | 0.0374 | **0.0341** ↓ |
| **σ = 0.03** | 0.0453 | 0.0550 | **0.0706** ↑ |
| σ = 0.10 | 0.0443 | 0.0346 | **0.0327** ↓ |

**At σ=0.03 the body progressively STANDS UP over the run** — monotone 0.045 → 0.071, ending
**above the deployed baseline's 0.058** — while every other arm sinks, and it does so with
`height_homeo_gain = 0` and no stance-lift (the pure-HK tier has no height mechanism at all). It
also holds the late step rate σ=0.10 destroys (2.99 vs 1.60) on the least path per unit
displacement of the three. **Aggregate metrics called this `chassis_y 0.059 ± 0.026` and it read
as noise; only `windowavg.py` showed it was a monotone rise.** σ=0.03 helping where σ=0.10 hurts
is the stochastic-resonance shape: an optimum exists and **PM's value is past it for our body** —
consistent with our `[pos, action, delta]` state doubling the same σ into the velocity channel.

⚠️ **The position-only test then RAN and REFUTED that reasoning — backwards.** Built as
`pos_noise_sigma` in `JointSensorimotorBridge` (injected *after* `delta` is computed from clean
positions, so the dose lands on `pos` alone), σ ∈ {0.01, 0.03, 0.05}, 40 k, n=4:

| chassis_y per window | early | mid | late | |
|---|---|---|---|---|
| BODY-side σ=0.03 (both channels) | 0.0453 | 0.0550 | **0.0706** | **RISES** |
| POS-only σ=0.01 | 0.0437 | 0.0366 | 0.0391 | sinks |
| POS-only σ=0.03 | 0.0420 | 0.0439 | 0.0331 | sinks |
| POS-only σ=0.05 | 0.0434 | 0.0401 | 0.0438 | flat |

**Position-only reproduces the posture rise at no σ**; transport still declines monotonically
(net_disp 1.46 → 1.13 → 0.87 → 0.71) and spin worsens (`turns` ±15–20 vs ±12). **The prediction
recorded above was that the velocity channel was where the damage was; the measurement says the
posture rise REQUIRES the velocity channel.** In hindsight that is the sensible mechanism —
velocity is the channel the HK gradient weights most (44 % of `|C|` mass), so a persistent
perturbation there is what it works hardest to become responsive to, and stiffening that loop is
what lifts the body. Position-only perturbs a channel the controller largely ignores.

**Revised verdict:** `REGRESSION` on transport at every σ and both injection sites; **`WORKING`
on posture only for BOTH-channel noise at σ ≈ 0.03**, active ingredient = the velocity component.
**Re-use / next test: velocity-channel-ONLY noise**, the exact complement, deliverable through
the same bridge hook — it would confirm the mechanism rather than infer it.

---

### ★★★ 2026-08-02 — OPERATOR OBSERVATION OVERTURNS PART OF THE SAME DAY'S ANALYSIS

Both pure-HK arms watched in the UI, seed 6000. **Four corrections, three of them to claims made
earlier the same day. Operator/UI diagnosis is first-class (`CLAUDE.md` §4) and here it caught
what 20 seed-runs did not.**

**1. Coordination DOES emerge, through the body — with `coupling_gain=0`.** Observed: *"a hint of
diagonal symmetry with front-left and rear-right working together"* by ~10k ticks, with no
coupling term and no gait phase in play. The claim that four independent 3×3 per-leg blocks
"structurally cannot coordinate" was **too strong**: `C` cannot *hold* a cross-leg term, but the
controllers are coupled **through the body**, and that suffices for a trot diagonal unaided.
**This is PM's own `armband_split` result reproduced** (independent per-channel controllers
cooperating through the physical medium). ⇒ whole-body `C` drops from "the leading candidate" to
one candidate, and the question becomes *"does representing coordination in `C` make it faster or
more stable than letting the body do it?"* — not *"is it required?"*

**2. ★ 6 000 TICKS IS TOO SHORT FOR A STRIPPED CONTROLLER.** Observed: convulsions *"morph over
time into sort of a crab walk with the chassis off the floor occasionally by 10k ticks."* The
entire pure-HK campaign — the `c_init` sweep, gravity arms, stance arms, 16 seed-runs — ran at
6 000. **We measured the transient and reported it as the behaviour.** 6 000 is the *deployed*
config's protocol (it walks from ~2 400); a controller that must FIND its behaviour needs a
longer horizon. **Every pure-HK `NULL` on locomotion is provisional until re-measured at ≥20 k.**

**3. ★★ `steps` DOES NOT SEE AN INCHWORM — new blind metric.** The control arm reports `steps`
**0.5 / 6 000 ticks**; the operator watched *"the front right leg forming an inchworm motion that
drags the body forward."* `steps` counts foot-lift events above a height threshold, so a leg that
flexes and drags registers nothing. **The arm the metrics called frozen was locomoting by a
mechanism the instrument cannot see.** Add to the blind-metric list beside `turns`, chassis
height and `fwd_v`.

**4. ★★★ "THE ABLATION COSTS NOTHING" WAS MEASURED ON A BLIND METRIC.** That headline (below)
rests on flat corridor metrics only. Observed on pure-HK: *"the robot seems a bit lobotomized
compared to our earlier work that had good emergent climbing skills. Placing this config on
obstacles after ~20k ticks does not exhibit the escape behaviors that marked progress."*
**The operator's three aliveness signals are heading regulation, proto-gait steps, and
obstacle-triggered adaptation — and the ablation was never tested against the third.** Flat
distance can be preserved while the adaptive capability is gone; this ledger has recorded that
failure twice already.

**The gate was then run, and the null SURVIVED it** (`humpavg.py`, teleport onto the crest at
tick 3 000, n=4): baseline final_z **4.52 ± 0.45** / gain_z 1.91 / belly 0.021 / 0 falls vs
all-learning-off final_z **4.50 ± 0.19** / gain_z 1.88 / belly 0.020 / 0 falls. **Identical, at
lower variance.** I expected this to overturn the headline; it did not.

⚠️ **But that makes the gate the finding.** §5 already records that hump clearance "works by
letting the belly ride low, which may be a sim exploit" — the robot may be *bulldozing* rather
than negotiating. **A gate a fully-scripted walker passes identically is not a test of
adaptation.** ⇒ **Our obstacle gates cannot distinguish a learned adaptive system from a scripted
one**, which is evidence about the gates as much as about the ablation. The operator's criterion
is sharper than either: *error spikes on contact, the body feels around and sometimes traverses,
late in a run.* A scripted walker shows no exploratory variation on contact and no early-vs-late
difference. **Building that comparison is the open instrument job**; `recoveravg.py` is the
closest existing tool. **Scoping note:** the "lobotomized" observation was made on the *pure-HK*
arms, which have no panic reflex, no stuck→explore, no height homeostat and no amplitude
homeostat — so it does not bear directly on the ablation arm, and the escape behaviour may live
in those reflexes rather than in the learned controller. This gate result is consistent with that.

---

### ★★ 2026-08-02 — PLAYFUL MACHINE IMPORTS: `c_init` `WORKING` on activity, gravity scaffold `NULL`

Source analysis: [`playful_machine_source_analysis.md`](playful_machine_source_analysis.md).
All arms n=4, corridor, 6 000 ticks, diff 0.3. **Measured on the `pure_hk` tier**, per the
context finding below — the deployed base cannot express a homeokinetic lever.

**`c_init` (import I1)** — PM's Sox `cInit`: adds a positive own-joint position feedback to
`C(j,3j)` so the loop starts self-exciting instead of at a dead fixed point. **Added to**, not
replacing, the per-leg random init (that randomness is this module's inter-leg symmetry breaker).
Effective loop gain is `motor_gain · c_init`, so PM's cInit 0.7–1.2 ⇒ `c_init` ≈ **0.23–0.40**
at our `motor_gain=3.0`. 0 = off, byte-identical (verified by measurement + unit test).

| `c_init` on `pure_hk` | net_z | chassis_y | belly | steps | step_bal | tilt_sd | falls |
|---|---|---|---|---|---|---|---|
| 0 (control) | −0.03 | 0.026 | 0.005 | 9.75 | **0.00** | 0.064 | 0.25 |
| 0.25 | −0.07 | 0.038 | 0.036 | 20.0 | **0.32** | 0.567 | 2.00 |
| 0.5 | +0.10 | 0.033 | 0.017 | 18.0 | 0.19 | 0.270 | 0.25 |
| 1.0 | +0.11 | 0.038 | 0.030 | **32.8** | 0.22 | 0.395 | 1.00 |

**`WORKING` on activity, `NULL` on locomotion, `REGRESSION` on stability.** Every non-zero value
converts a folded, inert body into an active one — chassis up, belly clearance ×3–7, steps ×2–3,
leg participation from literally zero to 0.19–0.32 — and none produces forward progress
(net_z ≤ 0.11). Activity rises with `c_init`; the stability cost is non-monotonic (0.25 wobbliest,
0.5 calmest — higher `C` pushes `z` further into `tanh` where `g'` shrinks and gain self-limits).
**The mechanism is confirmed; activity is not locomotion.**

**`scaffold_gravity_scale` (import I6)** — PM runs *every* legged experiment at gravity −6 vs
−9.81 (snake −4), on rubber, with compliant passive distal joints. New body knob (export +
`OGMA_PICRAWLER_GRAVITY_SCALE`), 1.0 = off, byte-identical (deployed baseline reproduces
4.58 ± 0.27 seed-for-seed). PM equivalent 0.61.

**`NULL`.** At 0.61 g the un-excited `pure_hk` body collapses to **exactly the same chassis height
(0.026) and belly clearance (0.005)** as at 1.0 g; `turns` gets *worse* (±1.1–5.6). With
`c_init=0.25` it buys mildly less wobble (tilt_sd 0.567→0.387, falls 2.0→1.5) and no progress.
**Diagnosis: gravity was never the binding constraint on standing** — the servos have ~4×
headroom over the static hip2 load already. PM's gravity reduction buys *dynamic* margin, not
the ability to hold a pose. **Re-use context: retry as a dynamics scaffold (fast/unstable gaits),
not as an uprightness scaffold; and the honest test it points to instead is `pure_hk` + the
stance reflex** (`postural_gain`), since that is the one parameter separating a `pure_hk` body
that collapses from an all-learning-ablated deployed body that stands fine (chassis 0.058).

**`cmd_squash` (import I3)** — see the saturation entry below. `NULL` on the deployed base, and
that null is a *passed prediction*, not a failed lever.

**★ What the pure-HK campaign converged on: HK produces MOTION but not DIRECTED motion.** Across
every pure-HK arm (1.0 g and 0.61 g, with and without the stance reflex, `c_init` 0 → 1.0) the
legs move and the body does not travel — best net_z **+0.23** against a deployed 4.58. The thing
that converts leg motion into translation in the deployed config is the hip1 stroke with its
hand-specified per-leg `stroke_signs` `{+1,−1,+1,−1}`. **This is structural, not a tuning gap:
MotorEPM is four independent 3×3 per-leg controllers, so `C` has no cross-leg terms and cannot
represent an inter-leg phase relationship at all** — four uncoupled oscillators at arbitrary
relative phase sum to no thrust. PM's dog, hexapod and humanoid all use **one Sox across every
joint**, where inter-leg coordination lives in `C`'s off-diagonal blocks and is *learned*.
⇒ **Whole-body `C` is promoted to the leading candidate** (the only untested import that could
supply direction as a learned quantity rather than as `stroke_signs`); **colored sensor noise** is
second (cheap, unbuilt, and PM's loop amplifies correlated sensory noise into motion while ours
has none).

**Also established, and it is a BODY fact:** `pure_hk` + `postural_gain=0.7` + `knee_tuck_target=0.7`
— identical to the deployed values — holds the robot at chassis **0.029** taking **0.5 steps per
6 000 ticks**, where the deployed config stands at **0.058**. The difference is entirely
`height_homeo_gain` + `stance_lift_gain`. **The picrawler's standing posture is actively
constructed by the height homeostat, not held by postural tone** — which connects to the
kinematic dead-end entry (feet plant at 170 mm against 166 mm total leg reach). PM's robots have
no analogue: their servos' neutral pose *is* a standing pose. That is a large part of why this
project needed a scaffold stack and theirs did not.

---

### ★★★ 2026-08-02 — THE DEPLOYED GAIT SURVIVES A TOTAL ABLATION OF THE LEARNED CONTROLLER

**Context:** a deep dive on the Playful Machine sources (Der/Martius, `~/Documents/PlayfulMachine`)
prompted three cheap Phase-0 measurements on our own stack. Full analysis, source diff and import
list: [`playful_machine_source_analysis.md`](playful_machine_source_analysis.md).
**Protocol:** corridor, n=4 fixed seeds, 6 000 ticks, diff 0.3; every arm one `mkarm.py` config
off the deployed `..._imufused__steplock_off.json` base with the diff printed.

| arm | net_z | straight | steps | step_bal | tilt_sd | planted | falls | verdict |
|---|---|---|---|---|---|---|---|---|
| **baseline** (all learning on) | 4.58 ± 0.27 | 0.73 | 50.0 | **0.48** | 0.065 | 3.69 | 0 | `BASELINE` |
| `ctrl_lr=0` (HK gradient off) | 3.78 ± 0.92 | 0.68 | 57.3 | 0.26 | 0.073 | 3.63 | 0 | see caveat |
| `embed_lr=0` (CPG-embedding off) | 4.23 ± 0.50 | 0.71 | 41.8 | 0.29 | 0.062 | 3.60 | 0 | — |
| **ALL controller learning off** (`ctrl_lr`+`embed_lr`+`sat_lr`+`bias_lr` = 0) | **4.75 ± 0.48** | 0.72 | 44.0 | 0.40 | 0.067 | 3.66 | 0 | **`NULL` — the ablation costs nothing** |

**Turning off every learned component in MotorEPM does not reduce distance, straightness,
stability, belly clearance or falls.** The learned layer buys `steps` 44→50 and `step_bal`
0.40→0.48 and nothing else. ⇒ **In the deployed config, locomotion is produced by the hand-built
scaffold stack** (stroke · Kuramoto-toward-a-specified-trot · postural · height · heading PD ·
stance-lift), not by the homeokinetic controller or the CPG-embedding.

**Verified, not assumed** (`CLAUDE.md` §3.2, all seven): `hk_share` 0.111 → **0.006** and `|C|`
mass reverts to the flat random-init profile (consumer fired); `motor_tle` **0.2495 in both arms**
— the forward MODEL still learns, only the controller was ablated (not dead code); baseline
reproduces 4.58 ± 0.27 exactly; `mkarm` printed four real diffs (no tautology, no silent
confound); this IS the faithful version — the 3-param attempt was the weakened slice (below).

**Both partial ablations are worse than either extreme** (3.78 and 4.23 vs 4.58 on and 4.75 off).
The learned components mutually compensate and the scaffold gains were hand-tuned with them
running — the signature of a learned layer the surrounding controller was fitted **around**,
rather than one the gait is built **on**.

> ### ★★★ POWERED TO n=20 (2026-08-02) — AND IT IS NOT A TIE, IT IS A WIN FOR THE ABLATION
>
> | metric | baseline n=20 | **ALL learning off n=20** | Δ | t |
> |---|---|---|---|---|
> | **net_z** | 4.322 ± 0.672 | **4.915 ± 0.459** | **+0.593** | **+3.26** |
> | **flat_v** | 0.044 ± 0.012 | **0.059 ± 0.011** | **+0.014** | **+3.80** |
> | **straight** | 0.698 ± 0.057 | **0.737 ± 0.031** | +0.039 | +2.68 |
> | steps | 55.45 ± 11.83 | 46.30 ± 7.55 | −9.15 | −2.92 |
> | tilt_sd / planted / belly / falls | 0.072 / 3.60 / 0.023 / 0 | 0.066 / 3.69 / 0.022 / 0 | ties | — |
>
> **All four \|t\| > 2.6; net_z and flat_v exceed 3.2.** At n=4 this read as "costs nothing." At
> n=20 it is **an improvement**: removing every learned component makes the deployed gait travel
> further, straighter and faster, **with lower variance on all three**. The learned layer buys
> `steps` (+20 %) and a little `step_bal`, and pays for it in distance, straightness and
> consistency.
>
> **★ Note what moved `flat_v`.** It has been pinned at 0.03–0.05 across *nine* isolated timing
> levers (§5) and a tenth (the load-gated stroke). The first thing to move it — 0.044 → 0.059,
> t = 3.8 — is **deleting the learned controller.** The eleventh lever is a subtraction.
>
> ⇒ The honest verdict is no longer `NULL` but a mild **`REGRESSION` for the learned layer on
> this base**: on the deployed config the homeokinetic controller and the CPG-embedding are not
> inert, they are slightly harmful to locomotion. **This does not generalise off this base** —
> the same controller on the `pure_hk` tier is the *only* thing producing motion at all.

**Corroborated by a second, independent arm.** Raising `ctrl_lr` **on the deployed base** (n=6)
does nothing good and destabilises: 0.05 → net_z 4.20 ± 0.95, straight 0.64; 0.10 → net_z 4.39 ±
0.77 but **tilt_sd 0.070 → 0.312 ± 0.504** (one seed diverged) and belly 0.023 → 0.045 ± 0.046.
**More learning on the deployed base is worse, in the same direction the ablation points.**
Consistent story: the scaffold stack was hand-tuned around a *weak* learned signal, and both
strengthening it and removing it move away from that tuning — removal favourably, amplification
not.

**Original n=4 framing, kept for the audit trail:** n=4 is a signal, and this was a null, which
is the claim type that most needs powering. It was recorded then because it was congruent with
`hk_share`=0.11, `step_cv` identical (0.94–1.06) in every arm ever measured, and the nine-lever
flat-`flat_v` record. Powering it strengthened rather than overturned it.

**What it does NOT say.** It is not a verdict on the Motor-EPM. The same run shows the HK gradient
learns something real and correct: `|C|` mass on the **velocity** columns is **0.445 with HK on
and 0.042 with it off** — homeokinesis learning a velocity-feedback law, exactly as it should.
It is a verdict on the **context**: a base whose scaffolds are loud enough that no homeokinetic
lever can be read. **⇒ Every mechanism lever from here is measured on the `pure_hk` tier first;
the deployed base stays the reference for "does this help the robot we have."**

> #### ⚠️ The first attempt at this ablation was INVALID, and the new instruments caught it
> `ctrl_lr=0`+`embed_lr=0`+`sat_lr=0` (without `bias_lr=0`) gave net_z **0.01**, `steps` **0**,
> `tilt_sd` **0.000** — a frozen statue. Not an ablation: an **integrator windup**. `h += bias_lr·μ`
> (`MotorEPM.cpp:2432`) is **not gated by `ctrl_lr`**, and `h -= sat_lr·z·tanh²(z)` is its only
> restoring term. Instruments named it on sight: clip duty **0.99 on every joint**, mean pre-clamp
> command **12.68** (baseline 1.40), `motor_tle` **0.0000** (a body pinned against its stops is
> trivially predictable). §3.2 rule 7 — the arm you think you ran is not the arm that ran.

---

### ★★ 2026-08-02 — `sat_lr` IS THE ONLY BRAKE ON THE BIAS INTEGRATOR (`REGRESSION`, mechanism proven)

Single parameter, `sat_lr` 0.02 → 0, deployed base, n=4 corridor:

| | baseline | `sat_lr=0` |
|---|---|---|
| net_z | 4.58 ± 0.27 | **0.12 ± 0.41** |
| steps | 50.0 | **20.5 ± 35.5** — `[82, 0, 0, 0]`, three seeds **zero** |
| tilt_sd | 0.065 | **0.380 ± 0.426** (one seed 1.117 — convulsed) |
| mean pre-clamp \|cmd\| hip1 | 1.40 | **14.27 ± 11.16** — seeds 4.2 / 9.3 / 13.6 / **30.0** |

Unbounded, still-growing, seed-divergent magnitude = an integrator with no brake. **The mechanism
is proven by the pairing with the entry above: with `bias_lr` ALSO zero, `sat_lr=0` is completely
harmless** (net_z 4.75, the best arm measured). So the failure is **bias windup**, not saturation.

**`sat_lr` is documented in-source as "anti-saturation — surrogate for the dropped ∂G term."
Nothing recorded that it is load-bearing for stability.** ⚠️ **Live trap:** the obvious
improvement — replacing `sat_lr` with the principled confinement term from PM's `sos_avggrad.cpp`
— removes this brake. Any such change **must** supply an explicit bound on `h`; PM's Sox carries a
separate `damping` parameter (dog 0.0001, humanoid 0.0001–0.0003) that we have no analogue of,
and that is very likely what it is for. **Third unbounded-integrator failure in this campaign**
(cf. 2026-07-26) — it is this codebase's characteristic failure shape.

---

### ★★ 2026-08-02 — THE STRIDE JOINT IS SATURATED 56 % OF THE TIME (new instrument)

New Phase-0 instruments, report-only, in **both** `snapshot_state()`'s `mod` dict and
`diag_snapshot()`: `clip_duty` / `clip_duty_j` (fraction of post-warmup leg-ticks the assembled
command exceeded ±1), `pre_mag_j` / `pre_max_j`, `hk_share`, `echo_a_gain`, `c_mass_{pos,act,del}`.

| deployed baseline | clip duty | mean pre-clamp \|cmd\| |
|---|---|---|
| **hip1 (stride)** | **0.559 ± 0.044** | **1.404** |
| knee | 0.179 | 0.592 |
| hip2 | 0.010 | 0.091 |

`MotorEPM.cpp` computes `y = motor_gain·tanh(z)` with `motor_gain=3.0`, then adds ~7 further
terms, then applies the **only** clamp 400 lines later. **The nonlinearity the body applies is a
hard discontinuity; the HK loop-Jacobian `G = diag(1−tanh²)` assumes a smooth one**, so `L`
overstates loop gain wherever the command is railed. `mean|1.65·sin|` = 1.05, so the power stroke
alone is past the rail 42 % of every cycle: **`stroke_gain` above ~1.0 is a duty-cycle control,
not an amplitude control**, and the heading PD (gain 7.0) has full authority on one side of the
stroke and none on the other.

**Retrodiction, tested the same day — directionally right, magnitude insufficient.**
`stroke_gain` 1.65→1.2 (the ledger's one uncosted win) drops hip1 mean request 1.404 → **1.038**
and clip duty 0.559 → **0.476**, and buys **`steps` 50 → 63.8 (+27 %)** with `step_bal` 0.48 →
0.52 — but net_z is a tie (4.70 ± 0.58) and the joint is *still* railed 48 % of the time. **1.2 is
a less-saturated operating point, not a de-saturated one**, so "shorter stride" is not simply a
saturation result in disguise; the part it moved is step count.

**`cmd_squash`** (new lever: tanh squash of the assembled command instead of the hard clamp,
default 0 = byte-identical) on the deployed base: `NULL`/slight `REGRESSION` — net_z 4.58 → 4.17 ±
0.50, `planted` 3.69 → 3.54, `steps` 50 → 58.5 ± 16.0, tilt_sd tied. **Exactly what the ablation
entry predicts:** on a base where the learned controller contributes nothing, making the actuator
honest *for the learning rule* buys nothing and costs ~11 % of the scripted stroke's peak. **Re-use
context: it is a prerequisite for reading the HK Jacobian honestly and must be re-measured on the
`pure_hk` tier, where it is a mechanism question rather than a stroke-amplitude question.**

**Also measured — the echo channel (`[pos, action, delta]` state layout).** `echo_a_gain` =
**0.945**: the forward model HAS latched the channel that is a copy of its own command, spending
one of three output directions on it. But `|C|` mass on those columns is **0.159 against a
uniform 0.333** — the controller is *not* exploiting the free-win subspace. Real, wasteful of
model rank, **not the dominant failure**; fix when the state vector is next touched.

⚠️ **`hk_share` > 1.0 is not a bug.** It is `Σ|HK branch| / Σ|assembled command|`, and the additive
terms partially *cancel* the HK branch (the postural reflex pulls against it), so the denominator
can be the smaller number. Values above 1 mean **HK is being opposed**, not "HK is more than
everything". Seen at 1.07 on the windup arm and 1.20 on pure-HK + stance.

⚠️ **Instrument caveat, learned the hard way: `hk_share` is a BLIND metric for importance.**
It measures how loud a term is, not how much it matters — the scaffolds carry ~89 % of the
magnitude and (per the windup arm) produce zero locomotion without the right modulation. **Read
`hk_share` only alongside an ablation.**

---

### ★★★ 2026-07-27 — THE BODY HAS NO STEP PERIOD, AND THAT INVALIDATES THE PREMISE OF THE WHOLE TIMING FAMILY

**The single most consequential measurement of the session, and it was never taken before.**
New instrument `step_cv` — the cycle-to-cycle coefficient of variation of the TRUE
inter-touchdown interval, computed in the diagnostic block so it reports on the **deployed
baseline**, not only on lever arms.

| deployed baseline, corridor, 6000 ticks | |
|---|---|
| `step_cv` — all touchdowns | **0.984** |
| `step_cv_real` — micro-lifts (<4-tick swings) filtered out | **0.882** |
| real-step period | **61.9 ticks** (~1 s per leg) |
| `stance_bout` / `swing_bout` | 21.3 / **5.58** ticks |
| `short_bouts` — bouts under 4 ticks | **0.399** |
| `contact_duty` | 0.793 |

**A CV near 1.0 means the interval's standard deviation equals its mean — an essentially
memoryless process.** Filtering out the micro-lifts barely helps (0.98 → 0.88), so this is
not a sensor artifact and there is no rhythm hiding underneath. The picture: feet planted
~79 % of the time, ~80 % of lifts lasting only 1–3 ticks, and a genuine step roughly once
per second per leg at irregular intervals.

**⇒ There is no step phase to lock anything to.** Every timing lever in this ledger — nine of
them now — presupposes a rhythm the contact signal does not have. That is a far better
explanation for their uniform failure than any of the per-lever diagnoses recorded against
them, and it should be checked before another one is proposed.

**It also reframes the 2026-07-26 "three clocks beating at ~2.5 s" finding.** There are not
three clocks. There is one oscillator (`L.phase`, the knee) and an **aperiodic contact
process**; `td_plv ≈ 0.2` was reported as "the stroke is mistimed relative to the step" when
it also reads as "there is no step timing to be mistimed against."

**And it is the operator's stated goal, as a number.** The target has been "a good,
repetitive, efficient stepping gait" — `step_cv` measures exactly the *repetitive* part, and
it currently says the gait is not repetitive at all. **Drive `step_cv` down first; only then
does locking a stroke to it mean anything.** That is a concrete, cheap, and previously
unmeasured objective.

---

### ★★ 2026-07-27 — LOCKING THE STROKE TO THE STEP: the CLOCK NEVER ENTRAINED, so the lever is `DEFERRED`, not refuted

> #### ⚠️ READ THIS FIRST — the verdict below was written as `REGRESSION` and is CORRECTED
>
> The behavioural numbers are real and reproducible: driving the stroke from this step clock
> costs net_z **4.58 → 0.20** at the deployed offset, one parameter, n=4. **But the mechanism
> instruments — once their own plumbing was fixed — say the clock was never phase-locked to
> anything.** `td_plv` sits at **0.04–0.10** (near-uniform: touchdown lands at a random stroke
> phase) at every loop gain from 0.1 to 0.7, and the true entrainment error is ≈**1.55 rad**
> throughout. The clock RAN (`step_lock` 1.0, all four legs, only 4–15 lock/unlock flips per
> 3 000 ticks) — it just free-ran near the right frequency while a weak pull failed to close a
> persistent phase error.
>
> **So the stroke rode a phase that was neither the leg's own state nor locked to the step —
> worst of both worlds.** That is `CLAUDE.md` §3.2 rule 6 (*faithfulness: did you build the
> mechanism or a weakened slice of it?*), and it means **the lever as specified has not been
> tested.** `DEFERRED`, with the strong caveat that what WAS tested is decisively bad.
>
> **The tell was in the sweep and I misread it.** The src=1 row is FLAT across all 8 offsets
> (0.20–0.69) while the src=0 control row varies enormously (−0.13 → 4.62). **A sweep that is
> flat where the control's sweep is structured means the swept parameter has no effect** — an
> offset does nothing when the phase it offsets is not locked. I read that flatness as "no good
> region exists"; it was the signature of a mechanism that never engaged.
>
> **Two instrument failures made this hard to see, both mine, both now fixed:**
> 1. `update_gait_align_diag` hard-coded `L.phase`, so on a step-clock arm the whole alignment
>    diagnostic reported the LEGACY phase's alignment. *Not "did the consumer fire?" but "is my
>    verification instrument watching the thing I changed?"*
> 2. `step_lock`/`mv_stance`/`torque_agree_hip1` were added to `diag_snapshot()` only. The body's
>    stdout reads `get_module_metrics()`, fed from the `mod` dict in `snapshot_state()` — so they
>    read **0.0 in every headless run**, which is indistinguishable from "the mechanism never
>    fired." **Add a new instrument in BOTH places.**
> 3. `step_td_err` is sampled AFTER the phase pull, so it understates the true error by
>    (1 − gain) and *looked* like it improved monotonically with loop gain (1.39 → 0.47) when
>    dividing the corrector out shows a flat ≈1.55 rad. **An instrument that measures your own
>    corrector is not a measure of lock.** `td_plv` is the honest read.
>
> **What to do next, concretely:** make the clock actually entrain before judging the idea —
> the frequency estimate, not the phase pull, is the suspect (`step_period` reads 23.6 ticks
> against a true contact period of 26–28). Then re-run this sweep. Everything below stands as
> a record of what was measured; only the *verdict* changes.

**§5's headline lever, built and refuted.** The 2026-07-26 finding measured thrust and support
unlocked *within* a leg (`pos_stance` 0.512 vs `pos_swing` 0.513, three clocks beating at
~2.5 s) and named the fix: give the stroke a phase it does not itself drive. Built as
`stroke_phase_src` — a per-leg touchdown-referenced step clock, φ=0 AT touchdown, consumed at
the stroke site ONLY (coupling, amplitude homeostat and prop-credit keep the legacy
`L.phase`, so this is one lever on one consumer — the isolation `phase_joint=0` lacked).

#### The clean result: ONE parameter, 96 % of locomotion

| corridor, n=4, 6000 ticks, ψ = −2.85 (the deployed offset) | net_z | straight | falls | planted |
|---|---|---|---|---|
| **src=0** — legacy `L.phase` (= the deployed config) | **4.58 ± 0.27** | 0.73 | 0.00 | 3.69 |
| **src=1** — contact-referenced step clock | **0.20 ± 0.33** | 0.05 | 1.00 | 3.90 |

Identical in every other respect. The consumer demonstrably fired — it destroyed the gait.

#### It is not an offset miss: the FULL CIRCLE was swept, with a matched control row

The first sweep was **uninterpretable and is retracted**: each arm changed *two* parameters
(`stroke_phase_src` **and** `stroke_phase`, because −2.85 was tuned against the knee-derived
phase and does not transfer). Adding the src=0 control row at the same offsets is what made
the result readable, and it changed the reading completely.

| `stroke_phase` | src=0 (legacy `L.phase`) | src=1 (step clock) |
|---|---|---|
| 0 | 0.28 | 0.38 |
| π/2 | −0.13 | 0.33 |
| **π** | **4.37 ± 0.30** | 0.40 |
| **−π/2** | **4.62 ± 1.96** | 0.69 |
| **−2.85 (deployed)** | **4.58 ± 0.27** | **0.20** |

**Legacy has a BROAD good window** (π → −π/2, ~90° wide, containing −2.85); **the step clock
never exceeds 0.69 at any offset tested.** The control row is load-bearing: at ψ=0 and ψ=π/2
*both* sources collapse, so the cardinal-offset arms alone would have "refuted" a lever that
had not been tested.

#### Why: a hidden virtue in the thing being "fixed"

`planted` rises 3.58–3.69 → **3.85–3.91** in every step-clock arm while progress collapses:
the legs keep stepping (`steps` ≈ 48–54, as the baseline) and push, and the body does not
move. Best reading: **the unlocked stroke's virtue was COHERENCE ACROSS LEGS.** All four
strokes previously shared one reference — each leg's knee oscillation, mutually coupled by
Kuramoto on `L.phase`. Locking each leg's stroke to *its own* footfall makes the four thrusts
independent, and independent thrusts on a body whose `gait_phase` is a measured random walk
cancel. The within-leg misalignment was real; removing it cost a between-leg alignment nobody
had measured.

**`L.phase` is not a clock — it is a STATE OBSERVATION.** `atan2(joint velocity, joint
deviation)` is where the leg *is* in its own oscillation, re-derived every tick, so the stroke
riding it is a resonant drive reinforcing what the leg already does. Any external timebase,
however well locked to footfall, trades feedback for feedforward — *LEARNED cooperates,
IMPOSED fights* (CLAUDE.md §1).

#### A separate, generalizable lesson: HOW the phase is corrected, not just where it comes from

The first implementation SNAPPED `step_phase = 0` at touchdown. It did not merely
underperform — it convulsed the body (repeated inversion, `tilt_sd` 0.065 → 0.34). Two
compounding causes: **(1)** the stroke can *cause* touchdowns, so resetting phase *on*
touchdown is positive feedback — a push bounces the foot, the bounce re-triggers the reset,
the period estimate runs to its rail. Closing a loop "through the world" is only safe when the
stroke cannot trigger the phase-setting **event**. **(2)** `sin(φ + stroke_phase)` is a
*continuous motor command*, so snapping φ steps the command at every off-schedule footfall.

The repo already held the fix, in the module §6's re-use context had named: BodyRhythmTracker
does `φ += ω` with a **0.10 proportional pull** at each crossing. SynergyTimer (which this was
ported from) *does* snap — correctly, because its phase only INDEXES a discrete bin.

> **A phase that DRIVES a continuous command needs a soft pull; a phase that is only READ to
> index a discrete bin can take a reset.**

`step_phase_lock` = 0.10 default, **1.0 reproduces the snap** so that refutation stays
reproducible rather than becoming folklore. All numbers above are the *corrected* PLL form —
the snap form is worse everywhere.

#### Instruments added (all default-off, gain-0 guard verified byte-identical by measurement)

`stroke_phase_src` · `step_phase_lock` · `step_phase_debounce` · `step_period_alpha/_min/_max`
· `gait_raster_diag` (512-tick footfall ring → live Hildebrand plot in `xaq_inspector`, the
picture that makes stroke-vs-step visible while the robot walks) · `mv_stance`/`mv_swing`
(the stance/swing split on **achieved** hip1 motion — the non-tautological read, since a
touchdown-referenced phase satisfies `td_plv`/`pos_stance` *by construction*) ·
`step_td_err` · `torque_agree_hip1`. Unit tests 16/16 → **24/24**.

**Re-use context.** Retry when the phase is contact-referenced **AND shared across legs** — a
body-level step phase with per-leg offsets, i.e. `BodyRhythmTracker`'s `rhythm.body.gait`,
which §6 already named as *"built for exactly this"* and which this campaign ranked last. The
objection to it (its collective coordinate is built from hip1, the joint the stroke drives)
now looks much weaker than the inter-leg coherence it would preserve. Also still open: leave
the stroke on `L.phase` and instead **entrain `L.phase` itself** toward the contact rhythm
with a weak pull, so the leg *and every consumer of its phase* lock to footfall together —
giving the oscillator a prediction to fulfil rather than imposing a clock on one consumer
(§5.7).

---

### ⚠️ 2026-07-27 — THREE HARNESS/GYM DEFECTS THAT WERE SILENTLY CORRUPTING MEASUREMENTS

Found while running the above. Each is fixed, and each invalidated real numbers.

| Defect | What it did | Fix |
|---|---|---|
| **Corridor back wall was a VERTICAL seal** | a robot that turned round parked against it with no escape, *while still accumulating `fwd_v`* — a trapped body reading as "walking" (the blind-metric shape, §3 rule 4) | 30° self-centering ramp, as the side walls; containment verified by teleport at both ends |
| **Corridor +Z end simply DROPPED OFF** | fast arms walked off the world and were charged a `fall` (already recorded in §4) | 30° ramp; the far end is now contained rather than open |
| **★ `auto_reset` zeroes `step_in_episode`, so continuous-mode runs never reached `max_steps`** | an arm that keeps flipping **restarts its own countdown forever**. Measured on the snap-form p0 arm: seeds ended at ticks **13 407 and 72 043** against a 6 000-tick protocol, while the healthy baseline ended at exactly 6 000. Every COUNT (`falls`, `steps`) becomes counts-per-unequal-duration, so arms are comparable neither to the baseline nor to each other | terminate on the monotonic `tick_counter`; **byte-identical for any arm that never auto-resets**, verified on the baseline (all 17 metrics, `step_in_episode == tick == 6000`) |

**The third one's bias is the dangerous kind**: it costs wall-clock *in proportion to how badly
an arm fails*, so the worse a lever is the longer it takes to find out — which quietly
discourages running the sweeps that would refute things. It was caught only because it
corrupted a result of ours that we then checked.

**Also fixed: `mkarm.py` parsed its own `--allow-noop` FLAG as a `key=value`** and wrote a bogus
`"--allow-noop"` param into the arm config. `MotorEPM::apply_param` ignores unknown keys, so
that arm would have run silently — *a silent-confound generator inside the one tool whose job
is preventing silent confounds*. Caught only because `mkarm` prints its diff, which is what
that printing is for.

**⚠️ Every corridor number in this ledger predates the wall fix and is from a different gym.**
The deployed baseline re-measured in the fixed corridor (n=4, 6000): net_z **4.58 ± 0.27**
(was 4.75 ± 0.29), straight 0.73, flat_v 0.05, step_bal 0.48, tilt_sd 0.065, planted 3.69,
0 falls. The change is within ~0.6 σ because a 4.6 m walker never reaches either end wall —
the fix protects **fast** arms, i.e. exactly the ones a propulsion lever exists to produce.

---

### ⚠️ 2026-08-07 — `apply_patch()` never trial-validates `SetParamOp`: a startup `ERROR` + one dropped tick on every config without `cruse_coordinator` (found during a from-scratch environment setup; **NOT FIXED — logged only, left for an operator decision**)

Found running a plain headless smoke test (`motor_epm_pure_hk.json`, corridor, 200 ticks) while
setting up this repo on a new machine — not a lever, not a measurement. Root-caused across three
layers; reproduces on any machine on this source tree, not a setup artifact.

**Symptom:** one line on startup —
`ERROR: OgmaBrain::tick: hot-patch or module-tick threw: hot_patch: SetParamOp.target_id 'cruse_coordinator' not found` —
then the run proceeds and completes cleanly (`RUN_END` at the configured tick count, no further
errors).

**The chain:**
1. `cpp_core`'s `Scheduler::apply_batch()` (`cpp_core/src/ogma/Scheduler.cpp:309-313`)
   deliberately throws `std::invalid_argument` when a `SetParamOp` targets an unknown module
   id — this is intentional, tested behavior
   (`cpp_core/tests/ogma/test_hot_patch.cpp:194-203`, `SetParamOpUnknownTargetRejected`).
2. The Godot bridge, `OgmaBrain::apply_patch()` (`godot_host/src/OgmaBrain.cpp:1943`), promises
   GDScript a synchronous `{success, error}` verdict, and does trial-construct every
   `AddNodeOp` before enqueueing to keep that promise (lines 1989-2008, with a comment
   explaining exactly why — to avoid a failure "surfacing only as a Godot console error").
   **That same trial-validation was never extended to `SetParamOp`** — a `set_param` op is
   parsed and enqueued unconditionally (line 2010), so `apply_patch()` always returns
   `success: true` for it even when the target module doesn't exist. The real failure shows up
   one tick later, inside `OgmaBrain::tick()`, as a `push_error`
   (`OgmaBrain.cpp:319-325`) — not through the Dictionary the caller is checking. (The
   ControlServer's own TCP `set_param` verb, `OgmaBrain.cpp:267-273`, *does* check
   `instance_->module(id) == nullptr` first — so the guard exists in the codebase, just not on
   this call path.)
3. `picrawler_body.gd` unconditionally pushes 5 `cruse_bias_gain*`/saturation-zone params into
   a `cruse_coordinator` module right after `brain.setup()`, for every config
   (`picrawler_body.gd:2397-2405`), with an explicit comment: *"Safe no-op when... the brain
   has no cruse_coordinator module."* It does check the returned error for
   `"unknown"`/`"not found"` to swallow it gracefully (`:7113-7116`) — correct in intent, but
   dead code on this path, since `apply_patch()` never gives it an error to check.

**Why it is very likely cosmetic, not a functional bug:** `Scheduler::process_pending_patches()`
swaps the whole pending queue into a local vector and applies each batch in order
(`Scheduler.cpp:115-121`); the first throw unwinds out of `tick()` before the remaining queued
batches are applied, so they are silently dropped rather than retried. Since none of the 5
queued ops had a real target either, nothing of value is lost. Net effect: one skipped tick of
brain processing at the very start of a run (no action computed, no diag published that tick),
then normal operation for the rest of the run — consistent with what was observed (one `ERROR`
line, then a clean run to `RUN_END`).

**Why this matters for the deployed stack specifically, not just the config tested:** per
§4's 2026-07-26 entry above, the deployed graph is 5 modules — bridge, MotorEPM, CPG,
KeyframeGait, BodyRhythmTracker — and **`CruseCoordinator` is not instantiated**. So this fires
on every run of the currently-deployed stack, not just the `motor_epm_pure_hk*` family tested
here. `cruse_coordinator` only appears in the archived `*_trot_cruse_v2*`/`*_walk_cruse_v2*`
config lineage (repo-wide grep) — a different machine that mostly ran that lineage, or that
wasn't watching raw stdout (vs. `seedavg.py`'s parsed summary, which this doesn't affect),
would have no reason to have noticed it.

**Not fixed. If it gets picked up:** extend `apply_patch()`'s pre-enqueue trial-validation
(`OgmaBrain.cpp:1989-2008`) to also check `instance_->module(op.target_id) != nullptr` for a
`SetParamOp`, mirroring both the existing `AddNodeOp` trial-construct path and the
ControlServer's own `set_param` verb. A regression test parallel to
`SetParamOpUnknownTargetRejected` but through `OgmaBrain::apply_patch()` (asserting
`success == false` synchronously, no `push_error`) would close the gap the tests currently
have at the Godot-bridge layer.

---
### ★★★ 2026-08-06 — THE PATH-LENGTH RETRACTION: a t=5.60, 6/6-seed, out-of-sample-replicated result that was BACKWARDS

`phase_vel_smooth=2` (low-pass the velocity arm of the atan2 producing `L.phase`) was
promoted on **+21.7 % "disp/s", t=+5.60, 6/6 seeds, then replicated on unseen seeds 7–12 at
+17.0 %**. Every statistical safeguard this project uses passed.

**It was a `REGRESSION`.** `disp/s` had been computed as `sum(hypot(dx,dz))/dt` — **PATH
LENGTH**, which counts backward motion as progress.

| | path/s | **net_disp** | straight | mean fwd_v |
|---|---|---|---|---|
| baseline | 0.1693 | **4.51 m** | 0.356 | +0.0503 |
| smooth 2 | 0.2059 | **1.91 m** | 0.121 | +0.0225 |

Net displacement **−57.6 %**; straightness a third of baseline; forward velocity halved.
Caught by the **operator's eye**, not by any metric: *"there is as much backward motion as
there is forward motion … it looks like an alternating current signal."*

**The lesson is not "seed-average harder".** Seed-averaging and out-of-sample replication
are filters against *sampling* error and cannot detect a metric that scores the wrong
quantity. CLAUDE.md §3 rule 4 names this exact trap (`fwd_v` mean is oscillation-dominated —
read it with `net_disp`) and it was used in the blind form anyway, for a whole session.
**Ask "what degenerate behavior scores well here?" BEFORE running the arm, not after.**

Fixed in tooling: `pulsereport.py` now reports `net_m`, `straight` (= net/path, which exposes
thrash directly) and mean `fwd_v`; the path-length number is deleted.

**Every "disp/s" figure from 2026-08-05/06 was re-read on net displacement.** The corrected
ordering (n=6, arena):

| arm | net_m | straight | fwd_v | verdict |
|---|---|---|---|---|
| **`intent_yaw_gain=0` (ground-only intent)** | **5.11** | **0.362** | **+0.0534** | ★ session best, `PARTIAL` |
| intent + yaw | 4.57 | 0.358 | +0.0510 | ties |
| baseline `stroke12` | 4.92 / 4.51 | 0.351 | +0.0497 | `BASELINE` |
| `intent_rhythm_gain=2.0` | 4.44 | 0.350 | +0.0499 | `NULL` |
| `fwd_resonance_gain=0.5` | 3.79 | 0.302 | +0.0406 | `NULL` (failed wrong-sign) |
| `coupling_gain=0.5` | 3.04 | 0.333 | +0.0327 | `REGRESSION` |
| `phase_sym_smooth=2` | 2.99 | 0.165 | +0.0330 | `REGRESSION` |
| `phase_vel_smooth=1` | 2.34 | 0.167 | +0.0244 | `REGRESSION` |
| `coupling_gain=3.0` | 1.86 | 0.153 | +0.0180 | `REGRESSION` |
| `phase_vel_smooth=2` | 1.91 | 0.121 | +0.0225 | `REGRESSION` (the retraction) |

### ★★ 2026-08-06 — `L.phase` TIMES THE POWER STROKE, so it cannot be filtered at all

New instrument: `couple_R` (Kuramoto order parameter), `phase_retro`, `res_period`,
`res_amp`, `res_lock` — live in the inspector and JSONL.

**Measured:** `L.phase` changes direction on **31 % of ticks** (directional consistency
`max(retro, 1−retro)` = 0.680; `couple_R` = 0.453 swinging 0.02–0.98). The y-arm of
`atan2(vel, pos)` is a **raw per-tick joint delta — a high-pass filter** — whose noise
exceeds the position arm near the zero crossings of a ~50-tick cycle.

Filtering it improves consistency (31 % → 18 % wrong-way) **and destroys locomotion**, both
asymmetric (`phase_vel_smooth`) and symmetric (`phase_sym_smooth`, built specifically to
test whether ellipse distortion was the cause — **refuted**, same failure). The signature is
identical in both: pulse amplitude **up** (0.243 → 0.278), net travel **down**.

⇒ **The lag is the defect and it is structural.** `L.phase` does not merely describe the
leg, it *times the power stroke*; any delay fires the stroke late so the leg drives backward
through part of every stride. **No filter of any shape can fix a signal whose value must be
timely.** `phase_joint=hip1` is not the fix either (37 % wrong-way, net −28 %).

**Re-use context:** the lag is *computable* — an EMA of weight `a` has group delay
τ ≈ (1−a)/a, and `phase_freq` is already measured — so advancing the filtered phase by
`phase_freq·τ` restores timing with both terms taken from the body's own dynamics. Superseded
in priority by MotorEPMv2 §rung 1, which fixes *all* loop delay with one mechanism.

### ★★ 2026-08-05 — commit precision has NO AUTHORITY over stride timing (a family verdict)

Nine variants of the intent/commit-precision error (yaw / ground-only, three exponents,
three rhythm gains) drove `commit_prec` from 1.00 to 2.75 and its clamp saturation from 0 %
to 37.5 %, with the consumer verified live each time. **`pulse_cv` never moved: 0.58–0.62
throughout.** `coupling_gain` *does* move it (0.755 at gain 3.0), so the metric is not
insensitive — the precision pathway simply cannot reach stride timing. `NULL` for the family,
not for any member.

Sub-results worth keeping: the intent error was **98 % yaw variance** in its first build
(`corr(intent_err, fwd_progress_ema) = −0.002`), fixed by per-term spread normalization then
by the operator's argument to drop chassis yaw entirely (**−0.833** correlation, and the
session's best arm). The commit exponent is **derived, not tuned**: `z` is bounded above at
exactly 1, so `k = ln(clamp_hi)` maps its ceiling onto the output ceiling — saturation fell
57 % → 6 % and displacement rose monotonically as it did.

### ⚠️ 2026-08-05 — n=4 fixed-seed CANNOT resolve a ~5 % effect on any commit_prec arm

Same exponent, same four seeds, same gym, measured twice: means **0.1770 vs 0.1617 (9.5 %
apart)** off a **0.01 %** parameter change. The baseline's own seed-sd is 0.0019 — an order
of magnitude tighter. Once the lever perturbs the trajectory, pinning the seed no longer
pins the outcome. A `+5.7 %` claimed at n=4 did not replicate. **Treat sub-10 % deltas on
these arms as unmeasured.**

## 5. Open frontier

- **★★★ THE MOTOR-SYSTEM AUDIT (2026-08-09) — read before proposing any lever:**
  [`motor_system_audit_2026-08-09.md`](motor_system_audit_2026-08-09.md) (+ verbatim
  appendix). Full topic graph, the 148-param inventory (11 live mechanisms; 3 configured
  params silently dead — `balance_gain`, `coord_stab_penalty`, `nav_gain` in the
  corridor; panic live via invisible header defaults), all five EPMs confirmed pure
  observers with unfed prediction sockets, three no-leak ratchets, and the timing
  finding that reframes the campaign: **`L.phase` is retrograde 2/3 of ticks, three
  clocks beat unsynchronized, and nothing references contact** — the structural
  explanation for every failed timing lever and the level-ground shuffle. Its §4.3
  dependency order (phase substrate → honest stance signal → stroke↔contact loop →
  anti-freeze floor → body-level predictor) is the audit's proposed roadmap, pending
  operator review.
- **★★★ THE STROKE-DIRECTION-AWARE STANCE RELEASE (2026-08-09) — BUILT AND MEASURED SAME
  DAY: walk fraction 2/20 → 11/20.** `stance_release_frac` (0 = byte-identical, verified):
  on a mid-stance sign flip of the leg's own commanded hip1 delta, fade that leg's stance
  biases until liftoff. At 1.0, n=20: walkers 2/20 → **11/20**, steps ×3.5, plv 0.09 →
  0.13 — the loudest single-lever move on the books under the new protocol, and it
  converts the shuffle attractor directly, which is what it was designed to do. **Costs:
  belly clearance (bellyc_min 0.014 → 0.003 — the released tuck rides the deck), falls
  0.15 → 0.30, straight −0.13.** See the boxed entry for the full record. **Open edges,
  in order: (1) dose — 0.5 keeps control-level stability at n=6, its n=20 walk fraction
  is unmeasured; (2) shape — release the KNEE only and keep the hip2 press if the belly
  cost is inherent; (3) operator UI observation before any promotion (§3 rule 5).**
- **★ Walk-fraction protocol (2026-08-09).** Until the shuffle attractor is solved, every
  A/B on the corridor stack reports walkers/n at n≥20 alongside the metric groups —
  seed-mean net_z on a bimodal distribution rewards lottery variance, not gait quality.
- **★★★ MAKE THE REFLEX GAINS INFERENTIAL — FIRST LEVER MEASURED 2026-08-04, see the boxed
  entry at the top of §2.** `couple_prec_gain` is `NULL` on the healthy gait and `WORKING` on the
  (d) test (coordination after a leg dies: control 0.127, k=+2 **0.190**, t = 5.84, wrong-sign
  control flat at 0.121) — but the honest decomposition says the effect is **resilience** (the
  survivors keep oscillating) rather than tighter phase-locking. **Two things the measurement
  changed about the direction itself, both of which any next lever in this family must absorb:**
  (i) a damaged limb's prediction error goes **DOWN**, not up, so `1/(tle+ε)` *alone* would
  up-weight the broken leg — the activity term is doing the work, not the error term;
  (ii) there is no `RealityToken.tle` at the motor layer at all (no `EPM`, no `LateralVoter` in
  any of the 139 modern picrawler configs), so this family runs on MotorEPM's own homeokinetic
  forward-model residual and must be *reported* as that, not as an EPM dual-TLE.
  **Still open in the family:** `postural_gain` and `stroke_gain` (note `stroke_gain` drives hip1,
  railed 56 % of leg-ticks, so its modulation is partly eaten by the clamp), and the
  scalar-magnitude form `g·(1 + k·tanh z)` rather than the neighbour-weight form.
  Run summaries: [sweep](run_summaries/2026-08-04_inferential-coupling-gain-sweep.html) ·
  [(d) test](run_summaries/2026-08-04_inferential-coupling-d-test-lesion.html).
- **★★★ MAKE THE REFLEX GAINS INFERENTIAL — the original statement of the direction (2026-08-03).** The deployed
  rules are legitimate as an **innate** layer (a lamprey's swimming rhythm is a spinal CPG;
  animals do not derive locomotion from nothing). What makes them a *script* rather than
  *inference* is that their gains are CONSTANTS. The line that matters is not hand-written vs
  learned — it is **does the agent's own error modulate it?**
  ⇒ Precision-weight the reflex gains by the agent's own TLE: `g_eff = g · f(1/(tle+ε))` —
  **couple harder when the model is uncertain, relax when it predicts well.** Same
  precision-weighting the LateralVoter already uses, applied to `coupling_gain`, `stroke_gain`,
  `postural_gain`. That converts the CPG from a fixed schedule into **a prior whose confidence is
  inferred**, which is what the framework actually asks for.
  **Why this and not a seventh mechanism:** it is the only proposal that gives the learned layer a
  job it can measurably do — and the n=20 ablation says it currently has none. **Judge it on the
  (d) perturbation test**, not on distance: perturb mid-episode and show the precision-modulated
  version re-organises where fixed-gain does not. If it does, the learned layer has earned its
  place on evidence. If it does not, that is a real result about this body.
- **★★ SCALE THE CLAIMS TO THE MEASUREMENT.** Not supported: "the picrawler walks via active
  inference." Supported: "the picrawler walks via an innate pattern generator and reflexes; the
  homeokinetic layer contributes leg participation and step count." §7 already requires this; the
  headline has been ahead of the evidence.
- **`nav_gain=5.0` reads `target_compass`, which the plan names as its declared disqualifier.**
  On the books as a known oracle in the deployed config — not a framing question.

- **★★★ THE MEASUREMENT CONTEXT IS THE BLOCKER (2026-08-02).** Ablating *every* learned component
  of MotorEPM costs the deployed gait nothing (4.75 ± 0.48 vs 4.58 ± 0.27, n=4). Locomotion here
  is the scaffold stack. **Consequence for everything below: a lever tested on the deployed base
  is being measured as a perturbation of a script.** That is the §3.1 lesson arriving from the
  other side — we have been generating verdicts in a context that cannot express the mechanism.
  **New protocol: every mechanism lever is measured on the `pure_hk` tier FIRST**
  (`motor_epm_pure_hk__inst.json`, instrumented); the deployed base remains the reference for
  "does this help the robot we have", not the place mechanisms are judged.
  **First powering job: take the ablation null to n≥20 with varied world seeds** — it is a null
  at n=4, which is the claim type that most needs power.
- **★★ NOTHING HAS EVER MOVED `step_cv`.** Across every arm measured to date — deployed baseline,
  HK-off, embed-off, full ablation, shorter stride, squash — `step_cv` sits between **0.94 and
  1.06**. Neither the learned controller nor the hand-built scaffolds affect footfall regularity.
  The operator's goal is untouched by the entire lever space tried so far, which argues the cause
  is structural (actuation, body, or the absence of any mechanism that *predicts* touchdown)
  rather than a gain anywhere in the current stack.
- **★ The Playful Machine import list** —
  [`playful_machine_source_analysis.md`](playful_machine_source_analysis.md) §5. Built and
  gain-0-guarded so far: `c_init` (self-exciting controller init, PM's Sox `cInit`; PM-equivalent
  ≈ 0.23–0.40 at our `motor_gain=3.0`) and `cmd_squash` (actuator honesty). Not yet built:
  colored **sensor** noise (PM puts `ColorUniformNoise(0.1)` on every sensor; our proprio is
  noiseless and our only noise is white and motor-side), the principled ∂G/`sense` confinement
  term (**must carry an `h` bound — see the `sat_lr` entry**), `Logarithmic` scale-free error,
  and `SERVO_KI` (PM's `ForceBoostWiring` booster 0.05 is on in every legged experiment; ours
  is 0).
- **Fast flat traversal, belly-up** — still the active thread. ~~bake `stance_lift=0.5` into
  the base~~ **DONE — `stance_lift_gain=0.5` + `feet_topic` are in the deployed
  `..._imufused.json`.** An alternative framing offered but never tested: express stance-lift
  as a **postural-target shift on stance legs** rather than a separate additive bias.
  **2026-07-27 status: distance CAN now be moved without paying in belly or falls** — a
  shorter stroke gives +12 % on open ground (below) — but `flat_v` specifically is still
  pinned at 0.05 in the corridor.
- **Genuine step-over obstacle negotiation** — deferred. Current hump clearance works by
  letting the belly ride low, which **may be a sim exploit** (frictionless belly drag); a
  real chassis could not do it. Obstacle nav is "good enough" for now.
- **★ FLAT SPEED IS PINNED ACROSS EVERY TIMING LEVER TRIED — ~~cause still unknown~~
  CAUSE MEASURED 2026-07-26, see the boxed finding below**
  (2026-07-25). Eight isolated levers were seed-averaged against the stance-lift base —
  stance-lift gain (3 values), swing-detector deadband (3), lateral-sequence phasing,
  stride-joint phase readout (4 offsets), adaptive coordination (2). **`flat_v` was
  0.03–0.04 in every arm that stayed upright and 0.00 in the ones that did not.** Nothing in
  the coordination, phasing, or phase-bookkeeping layer moved it. *That observation stands*
  — and a ninth lever (the load-gated stroke, §2) has since joined it at 0.04–0.05.

  > ### ⚠️ CORRECTION (same day) — the explanation first recorded here was WRONG
  >
  > This entry originally read "**the binding constraint is that the body does not step**",
  > citing "the feet are on the ground 99.3 % of the time". That number came from
  > `feet_y < stance_y_threshold` — a **world-height** test, which is neither
  > terrain-relative nor chassis-relative. Measured against the **true physics
  > foot-contact sensor** on identical runs, that proxy **under-reports swing by ~16×**:
  > true swing is **0.229**, the proxy says 0.014. **The robot IS stepping**, at roughly a
  > 77 % stance duty — an ordinary walking duty factor. The "shuffling / nothing swings"
  > story is retracted, along with the claim that this explains the old gait-symmetry era.
  >
  > The same correction shrinks the swing-detector bug in §4: the legacy detector reads
  > 0.408 against a true 0.229 — **over-reporting by ~1.8×, not the "50×" recorded.**
  > Still wrong, still worth replacing, but a different order of magnitude.
  >
  > **The methodological lesson is the sharper one, and it is on me:** I used an absolute
  > world-height threshold as *ground truth* while, in the very same change, arguing
  > correctly that an absolute world-height threshold is unsuitable for exactly this job.
  > A proxy cannot be the control for the proxy it is replacing. **Three different measures
  > of "is the foot down" disagreed by 16×, and the campaign ran without ever checking them
  > against each other.** The body publishes the real sensor (`reality.proprio.foot_contact`,
  > a physics touch flag) every tick and always did.

  **What is genuinely open:** why flat speed does not move. It is *not* explained by a
  missing duty cycle. Candidates now: propulsion amplitude/stroke operating point (never
  swept), ground friction / foot geometry, or the stroke and stance phases being
  mistimed relative to actual contact — the last of which is newly testable now that true
  contact is wired. **And note the sensor surface is far richer than anything yet consumed:
  `joint_torque` (a real per-joint load signal) is published and unused, and the IMU token
  carries no vertical acceleration and no pitch/roll rates at all.** See
  [`sensor_legitimacy_and_the_feet_y_oracle.md`](../plans-and-designs/sensor_legitimacy_and_the_feet_y_oracle.md).
- **~~OPERATOR DECISION PENDING — is `coord_reward_drive` allowed to stay?~~ RESOLVED
  2026-07-25 via option (c): the fitness was replaced, not the search.** `coord_fitness_mode=1`
  ranks probes by `coherence · activity / (1 + tle)` — no position, distance or velocity term
  — and ties-or-beats the reward it replaced on every metric while recovering better from
  perturbation (§1). **The generalizable result: the capability lived in the SEARCH, not in
  the reward.** Deleting the search cost net_z 3.52→2.36 and step_bal 0.26→0.03; deleting
  only the *reward* cost nothing. Worth trying the same substitution wherever a fitness has
  crept in. Remaining thread: `coord_reward_drive` still stores a winner, so the ratchet
  *shape* remains — it is now much harder to poison (a thrash scores badly on this fitness),
  but "stores a winner" is still the risky shape, and the inverted-stuck case below is
  still untested.
- **Does the ratchet lock in a destructive pattern after the robot gets stuck?** Partly
  answered. **Recovery gate (new tool `recoveravg.py`, n=3, 4 hump perturbations per run):
  no lock-in in either arm** — every seed kept advancing afterwards, 0 falls, and
  ratchet-ON actually recovered *more* (offsets returned 0.30 vs 0.11) and progressed more
  after the hits (+1.35 vs +0.76). This matches the operator's UI observation that the
  current config relearns to walk after getting stuck. **BUT the scenario tested is not the
  one that originally burned us:** the operator's failure was the robot stuck **on its
  back**, and in these runs the body never inverts (0 falls). So the inverted-stuck case is
  **UNTESTED**, not cleared. Note the likely reason it no longer bites:
  `auto_reset_on_inversion` defaults **true** (`picrawler_body.gd:490`), so an inverted robot
  is reset rather than left to thrash — meaning **the protection may be the auto-reset, not
  the fitness penalties.** If so it is a fragile dependency: disabling auto-reset could bring
  the old failure straight back. Worth testing directly before trusting the ratchet.
- **★★ GAIT REGULARITY IS THE BLOCKING PREREQUISITE — `step_cv` = 0.98 on the deployed
  baseline (0.88 with micro-lifts filtered out).** The body has NO step period: real steps
  arrive about once per second per leg at near-memoryless intervals. Every timing lever here
  presupposes a rhythm the contact signal does not have, which is a better explanation for
  their uniform failure than any per-lever diagnosis recorded against them. **This is also
  the operator's goal as a number** ("repetitive stepping" = low `step_cv`) and it had never
  been measured. Drive it down before proposing another phase lever. See the boxed entry in §2.
- **★ LOCK THE STROKE TO THE STEP — BUILT 2026-07-27; the build is `DEFERRED`, the IDEA is
  still open.** Driving the stroke from a per-leg touchdown-referenced clock costs net_z
  **4.58 → 0.20** (one parameter, full circle vs a matched control row) — **but the clock was
  measured never to entrain** (`td_plv` 0.04–0.10 at every loop gain; true phase error ≈1.55
  rad), so that number refutes *this build*, not the lever. See the boxed entry in §2 for the
  full record and the three instrument failures that hid it.
  **Concrete next steps, in order:** (1) fix the FREQUENCY estimate so the clock actually
  entrains (`step_period` reads 23.6 against a true contact period of 26–28) and re-run the
  sweep — only then is the lever tested; (2) if it still fails, try a phase that is
  contact-referenced *and shared across legs* (`rhythm.body.gait` + `gait_phase` offsets),
  since a per-leg clock also removes the inter-leg coherence the four strokes had via
  Kuramoto on `L.phase` (`planted` rose 3.69 → 3.90 while progress collapsed — effort up,
  output down); (3) or leave the stroke on `L.phase` and **entrain `L.phase` itself** toward
  the contact rhythm, so every consumer locks together. The original entry follows.
- **★ LOCK THE STROKE TO THE STEP — the sharpest lever the record now points at.** The
  boxed finding above measured the stroke riding a ~23-tick knee-derived clock while the leg
  steps every ~29 ticks, beating at ~2.5 s. The fix is to derive the phase the stroke rides
  from a signal the stroke does **not** drive — which is exactly the re-use context already
  recorded in §6 for `phase_joint=0`, whose refutation was a *wiring* refutation (a
  self-excited oscillator, because the stroke drove the same hip1 it read its phase from)
  and whose premise measured **positive**: step-balance rose 0.30 → 0.41–0.58 and
  feet-planted rose. Candidate phase sources, none of them hip1: the per-leg **load cycle**
  from `joint_torque` (now subscribed and measured to carry the signal on hip1 at ratio
  1.368), or `BodyRhythmTracker`'s `rhythm.body.gait`, already live in the config and built
  for exactly this. Budget a full `stroke_phase` re-sweep — the previous attempt swept the
  whole circle and still collapsed, but from self-excitation, which these sources do not
  have. **Judge it on `td_plv` / `pos_stance` vs `pos_swing` directly** (the instruments now
  exist), not only on the behavioural metrics — a lever that raises phase-lock without
  raising speed is still the informative result.
- **A working closed-loop attitude controller** — the balance reflex is a redesign, not a
  knob. ~~Prerequisite for revisiting lateral-sequence walk phasing~~ — that phasing question
  is now closed on this base (§2), so this is no longer blocking it.
- **Deferred nav layers** — L1 nav loops, L2 EFE arbiter, L3 keystone, per the plan's build
  order. The nav layer is still the plan's declared *disqualifier* (`target_compass` is an
  oracle); nothing in this ledger addresses it yet.

---

## 6. Re-use contexts — when a refuted lever should be tried again

*A complete verdict names the conditions that would justify a retry. These are live
proposals, not dead entries.*

| Lever | Refuted in | Try again when |
|---|---|---|
| **Lateral-sequence walk phasing** | ~~only ever measured inside the rejected sequencer~~ — **DONE 2026-07-25: isolated on the `embed`+stance-lift base, n=4. `NULL`/slight regression** (see §2). The support-polygon prediction held (planted 3.30→3.63) and bought nothing | **Only if the body starts genuinely stepping.** The isolation showed the win was already spent: the gait sits at 3.3/4 feet planted, so there is no support-polygon headroom for a phasing change to recover. Retry if a future lever produces real swing phases (a duty factor well under 1), which is when a swing *schedule* starts to matter at all |
| **Phase readout on the stride joint** (`phase_joint=0`) | corridor, n=4 × full-circle `stroke_phase` sweep — locomotion collapsed at every offset because the stroke drives the same joint the phase is read from (self-excited oscillator) | ~~When the stroke is moved off hip1, or the phase is derived from a joint the stroke does not drive~~ — **DONE 2026-07-27 and it also failed** (`stroke_phase_src`, a per-leg contact-referenced step clock: net_z 4.58 → 0.20 at the deployed offset, refuted across the full circle against a matched control row). Both refutations now point the same way: **the problem is not which signal the phase comes from, it is that any per-leg external timebase destroys the coherence the four strokes shared.** Retry only with a phase that is contact-referenced **and shared across legs** (`rhythm.body.gait` + per-leg offsets), or by entraining `L.phase` itself rather than replacing it |
| **★ Stroke-to-step lock, per-leg** (`stroke_phase_src=1`, touchdown-referenced step clock driving the stroke) | corridor, n=4, 6000 ticks, deployed instrumented base — **full circle swept (8 offsets) with a matched src=0 control row** | **`DEFERRED`, not refuted — THE CLOCK NEVER ENTRAINED** (`td_plv` 0.04–0.10 at every loop gain 0.1–0.7; true phase error ≈1.55 rad throughout). What was measured is decisively bad (net_z 4.58 → 0.20) but it is a *weakened slice*: the stroke rode a phase that was neither the leg's own state nor locked to the step. Fix the FREQUENCY estimate (`step_period` 23.6 vs a true contact period of 26–28) so the clock entrains, then re-run. Retry when the phase is **shared across legs**, not per-leg: `rhythm.body.gait` + `gait_phase` offsets, which preserves the inter-leg coherence this lever destroyed. The *within-leg* premise (thrust ⊥ support, measured) remains true and unaddressed — it just cannot be fixed by retargeting one consumer's phase. **Also carries a lesson independent of the idea:** the first build SNAPPED the phase at touchdown and convulsed the body; a phase that DRIVES a continuous command needs a soft proportional pull (`step_phase_lock=0.10`, BodyRhythmTracker's form), while a phase that is only READ to index a discrete bin can take a reset (SynergyTimer's form). `step_phase_lock=1.0` reproduces the snap |
| **stuck→explore** | flat ground, where a stuck-detector has no content — *the wrong scenario for it* | A regime where the body genuinely gets stuck: terrain, corridor corners, obstacle contact. Its inverse twin (progress→commit) was promoted, so the family is not dead |
| **Phase-indexed velocity (`Cvel`)** | on the *asymmetric* tripod-skid gait, which it amplified into circling | The base gait becomes symmetric, or the pump is gated by heading error so it cannot amplify yaw asymmetry. Explicitly "not wrong in principle" |
| **Cruse / Walknet contact-load reflex** | flat **and** incline; out-of-phase with the emergent gait — **and now known to have been gating on a GOD'S-EYE foot-height signal, never on load** | **This verdict is weak and should be re-opened.** Every Cruse rule gates on `in_swing_`, derived from `feet_y` = **absolute world-Y** (see the §2 oracle box), via a detector that over-reports swing ~1.8× vs true contact. So "out of phase with the emergent gait" was substantially a measurement of the *detector*, not of Walknet — a §7 *weakened-slice* shape. **The deeper point: Walknet's rules are LOAD rules, and they have never once had a load signal.** ⚠️ Correction to an earlier version of this row, which claimed there is "no load observation on the bus" — **there is: `reality.proprio.joint_torque`** (servo current sensing, 12 floats, hip1/hip2/knee × 4) is published every tick and **nothing in MotorEPM has ever consumed it.** That is the honest retry: give the load rules a real load observation. Historical warning still applies — an earlier Walknet null rested on a 1-of-6-rule slice, so scope any future claim carefully |
| **Load-gated power stroke** (`stroke_load_gain`) | corridor, n=4 × a 5-point gain sweep, on the deployed base — `NULL` on progress; the gate fired monotonically and the gait did not care | **When the stroke's TIMING is fixed first.** A magnitude gate cannot repair a push/drag balance measured at 50/50 (see the boxed finding) — it removes thrust spent in the air but scales push and drag equally during stance. Retry it *after* a lever locks thrust to support, where a purchase gate becomes a refinement of a correct stroke rather than a patch on a random one. Keep the `step_bal` 0.44→0.54 result: the load share genuinely does even out which legs take steps, so it is a candidate ingredient rather than a dead idea. The infra (`torque_topic`, `leg_load()`) stays default-off and is the load observation any Walknet retry needs |
| **`swing_tuck` (swing-phase leg fold)** | arena n=3 = a real 7× `step_bal` gain on the KNEE half alone; **corridor n=4 = a TIE** (0.44→0.43) | **It is not refuted, it is SCENARIO-SCOPED.** The arena gain was HEADROOM — the corridor baseline already sits at `step_bal` 0.44 while open ground sits at 0.07. Retry it wherever leg participation is genuinely poor (open ground, terrain), and expect nothing where it is already good. The −13 % wobble is the part that appeared in BOTH gyms and is the piece worth chasing |
| **The whole "get the tibia vertical" family** (`hip2_tuck_target`, `tibia_plumb_gain`, knee compensation, shorter stride) | arena n=3, all four routes, each closing differently | **Closed on LIMB GEOMETRY, not control** — the feet plant at 170 mm against a 166 mm total leg reach, so the shank must angle out just to touch the ground. Retry on **a body with a longer tibia or a shorter femur**, or behind a lever that moves foot placement by some route other than hip2 / knee / stride. Note `tibia_plumb_gain=0.15` is the largest single effect ever measured here (+32 % distance, `straight` 0.82 ± 0.00) and is held back ONLY by belly clearance — if belly-up is ever solved by another mechanism, this is the first thing to re-run |
| **Shorter stride** (`stroke_gain` 1.2 vs the deployed 1.65) | arena +12 % / corridor tie, n=3–4; hump gate holds; belly preserved; 0 falls | **`PARTIAL` awaiting operator UI observation, not refuted.** It is the first lever in this campaign to move distance without paying in belly clearance or falls. If the UI reads it as purposeful rather than merely busier, sweep 1.1–1.3 at n≥6 and re-run the recovery + inversion gates before baking |
| **Learned hip2** | flat, against a stable base | A regime where the femur must do real work — steep terrain, step-over. It was refuted as a *gait* lever, not as a terrain lever |
| **Gait symmetry (all forms)** | flat, ~35 A/Bs; the asymmetry is load-bearing for straightness | A different base gait exists whose straightness does not depend on the tripod-skid. Amplitude symmetry ≠ functional symmetry — any retry must target functional symmetry |
| **Active-balance reflex** | headless (inert — `publish_tilt` off) and UI (destabilizing — maps tilt→hip2, i.e. pitch/roll, not yaw) | It is redesigned as a real closed-loop attitude controller. The existing verdict is mostly a **DEAD_CODE + wrong-target** finding, not a verdict on balance |
| **★ `couple_prec_gain` on the HEALTHY gait** | corridor, n=6, `nolearn2` base — `NULL`, and mildly negative on `plv` at high `k` | **It is not refuted, it is SCENARIO-SCOPED, and the scope is the point: it is a DAMAGE-RESPONSE lever.** On an intact body there is nothing for a precision weighting to buy, and it costs a little coordination; on the (d) test it is worth t = 5.84. Retry as a *deployed* lever only if a regime exists where the body is routinely partly-broken (rough terrain, a real chassis with a weak servo). Meanwhile it belongs default-off, and the (d) result is the case for it |
| **Torque-ceiling lesion** (`LESION_MODE=torque`) | as a (d) perturbation — measured not to perturb (`tq_sat` = 0.009; ×0.05 leaves amplitude at 0.720 vs 0.725) | **Refuted as a PERTURBATION, not as infrastructure** — kept default-off. Its null is itself a finding (the body is not torque-limited). Retry on a body whose servos actually saturate, or use it to model a *real* servo-saver rather than to inject damage |

---

## 7. Inherited failure patterns (from the pre-split RL era)

*These come from `ami-ogma/docs/findings/mechanism_registry.md`, this ledger's ancestor. That
registry covers the **reward-shaped RL era** the doctrine now disowns, so its individual
mechanism verdicts do **not** transfer. The **failure shapes** and **measurement lessons**
do — they were paid for at enormous cost and several have already recurred on the current
reward-free stack.*

| Pattern | Signature | Recognize it by |
|---|---|---|
| **A — constrains without unlocking** | Variance tightens, mean does not lift | The mechanism *restricts* the policy rather than opening new policy space. Most nulls in this project's history were variance-constrainers |
| **B — stability bought with exploration** | Stability up, translation/discovery down | Damping motor noise or over-committing to stance suppresses the very exploration that finds the behavior |
| **C — dead code** | Byte-identical to control | The affected path isn't live under this config. **Not a verdict on the idea** |
| **D — tautology** | Byte-identical to control | The knob was already at that value. **Not a verdict on the idea** |
| **E — activity without aliveness** | Motion/energy metrics climb while goal metrics collapse | A degenerate attractor found by the agent: jitter, lurch, circling, lying down. Often arises from *channel interactions* that are each individually correct |
| **F — open-loop primitive with no differential outcome** | The selector learns mechanically but nothing translates | If no primitive changes the world usefully, the selector has no advantage signal. An open-loop primitive library is not a substitute for closed-loop control |
| **G — trades stability for reach** | Proposed, then **withdrawn** | The apparent reach gain was a single-seed outlier; only the stability cost was real. A cautionary tale about naming a pattern before powering it |

**The measurement lessons that came with them** — all now in `CLAUDE.md` §3.2 as
pre-verdict checks: baseline validity (a null against a degenerate control is not a null);
silent confounds (a harness flag that never got set invalidated an entire A/B family);
faithfulness (a 1-of-6-rule slice is not the mechanism); and consumer verification (a
published topic nobody acts on reads exactly like a null).

**Two inherited *open* questions**, neither ever settled: whether a metric should score
**aliveness** (closed-loop adaptive reorganization) rather than distance — the old repo
concluded distance metrics "select against aliveness"; and the **unambiguous-emergence
bar** (real capability should be *loud in a single run*, the way standing was) versus
seed-averaging small deltas. See `CLAUDE.md` §3.3 for how these two reconcile.

---

*Append every new verdict here — **with its scenario, its power, and its baseline** — as soon
as it is decided, plus the re-use context that would justify a retry (§6). When a verdict
generalizes into a reusable principle, fold that principle into
[`../brain_building_doctrine.md`](../brain_building_doctrine.md) and leave the specific
result here.*

### ★★★ 2026-08-20 — STEP 0: the criterion SEES coupling; the ACCEPTANCE MARGIN cannot

**Verdict: `WORKING` as a diagnosis — it refuted the fix we were about to build.** Evolver
held in observer mode (σ=0, scores without searching or publishing) while `SETPARAM_AT`
stepped `motor_epm.coupling_gain` through {0, 0.4, 0.8, 1.2, 1.6, 2.0}, 3 scored windows
per level, **ascending on 2 seeds and descending on 2** so hysteresis would show. Consumer
check: both arms reported `-> OK` and started at 0.0 / 2.0 respectively.

| coupling | J | upright sd | unloaded | flow | energy | falls |
|---|---|---|---|---|---|---|
| 0.0 | 3.6805 | 0.0027 | 0.0048 | **0.7555** | 0.3647 | 0.40 |
| 0.4 | 3.7476 | 0.0095 | 0.0022 | 0.7329 | 0.3754 | 0.17 |
| 0.8 | 3.6119 | 0.0782 | 0.0036 | 0.5245 | 0.3757 | 0.58 |
| 1.2 | 3.4045 | 0.0232 | 0.0031 | 0.4262 | 0.3690 | 0.50 |
| **1.6** | **3.3895** | 0.0163 | 0.0067 | **0.3940** | 0.3716 | 0.50 |
| 2.0 | 3.5881 | 0.0169 | 0.0071 | 0.4929 | 0.3839 | 0.70 |

**Authority:** `flow` **r = −0.518 (STRONG)**, J r = −0.260, falls r = +0.220, everything
else ~0.

**1. The criterion is NOT blind to coordination.** `flow_term` nearly halves from 0.756 to
0.394 as coupling rises — a large, monotone, strongly-correlated response. Hypothesis (a)
is refuted. There is also a real optimum near **1.2–1.6**, which vindicates the operator's
hand-found 1.55, and 2.0 is measurably worse.

**2. ★ The standing fix would NOT have worked.** Coupling's benefit is *not* catastrophe
avoidance — falls actually trend slightly UP with coupling (r = +0.220). The lexicographic
viability-then-quality ordering was the recorded next build; **this measurement killed it
before it was written.** That is the entire value of an authority check.

**3. ★★ The real blocker is a SCALE MISMATCH between the acceptance margin and the
mutation step**, and it is systemic rather than coupling-specific:

```
dJ/d(coupling) = −0.182 per unit           (measured, full-range swing −0.546)
σ 0.08  → step 0.240 → ΔJ 0.0436   vs margin 0.244  →  5.6× TOO SMALL
σ 0.04  → step 0.120 → ΔJ 0.0218   vs margin 0.244  → 11.2× TOO SMALL
σ 0.015 → step 0.045 → ΔJ 0.0082   vs margin 0.244  → 29.8× TOO SMALL
```

A single mutation's *true* improvement is far below `accept_k·σ̂`, so a real gradient is
rejected as indistinguishable from noise. **The margin that fixed the coin-flip problem now
blocks the gradient it was protecting.** This also explains why the ORIGINAL noisy criterion
found coupling (0→2.06 in 4/4): with no margin, a strong flow gradient could bias a random
walk upward; adding the margin removed the walk without supplying a detectable step.

**4. ⚠ THIS REFRAMES GATE 2d's HEADLINE — a correction to my own reporting.** I recorded
"2/4 seeds annealed σ to the FLOOR = settled", reading it as convergence. At the σ those
seeds reached (0.010–0.015) **nothing could clear the margin at all** (30× short), so the
anneal is at least as consistent with being FROZEN OUT as with converging: fewer accepts →
σ shrinks → smaller steps → fewer accepts. Self-reinforcing. I cannot settle it from the
gate-2d logs because `/tmp` is a tmpfs and the reboot cleared them, so this is a strong
hypothesis, not a fact. **The next run must log the accept timeline against σ to decide it.**

**Re-use context / the fix, derived from the measurement:** match the margin to the step by
construction. Requiring `|dJ/dg|·σ·range ≈ accept_k·σ̂` with the measured slope gives
**σ ≈ 0.22 at accept_k 0.5, or σ ≈ 0.11 at accept_k 0.25.** Present `sigma_min` is 0.01 —
10–20× below anything useful. Highest-value single change: **raise `sigma_min` to ~0.05–0.1
and lower `accept_k` to ~0.3**, so the search cannot anneal itself below the resolution its
own criterion can measure. Alternatives: longer windows (shrinks σ̂, costs generations) or
per-gain `sigma_scale` (coupling's range is the widest at 3.0).

⚠ Also fix before any coupling verdict: the evolver searches coupling to **3.0** while
MotorEPMv2's schema documents max **2.0**, and `set_param` does not enforce schema ranges.
The measurement above says >2.0 is worse anyway.

⚠ **OPERATOR CORRECTION (2026-08-22) — the chassis-collide distinction is smaller than I
claimed.** I set `OGMA_PICRAWLER_CHASSIS_COLLIDE=1` from gate 2 onward and wrote that
ghost-chassis history "is NOT comparable", reasoning that `height_homeo_gain` is under
evolution and a ghost belly would hide what sagging costs. The operator reports from the UI
that **the hip1 joints ground against the floor at much the same height as a solid chassis
does**, so the belly rarely becomes the limiting contact either way. The flag is therefore
worth keeping for honesty, but **"not comparable" overstates it** — ghost and solid runs are
closer than that phrasing implies, and no A/B here has actually measured the difference.
Treat cross-protocol comparisons as mildly caveated rather than void.

### ★★★ 2026-08-20 — GATE 2 FINAL: 3/4 seeds converge; the freeze question is ANSWERED

**Verdict: `PARTIAL`, best of the campaign.** Factory arm, arena, solid chassis, 500 k
ticks, 61 generations, with the step-0 calibration (`accept_k` 1.0→0.25,
`sigma_min` 0.01→0.08, `w_energy` 8→4, coupling max 3.0→2.0).

| | gate 2 | 2b | 2d | **2e** |
|---|---|---|---|---|
| J fell beyond own noise | 0/4 | 1/4 | 2/4 | **3/4** |
| J rose | 1/4 | 1/4 | 0/4 | **0/4** |
| closer to hand point | 3/4 | 3/4 | 4/4 | **4/4** |
| accepts (of 61 gens) | — | — | 21/14/10/8 | **16/18/24/26** |

Falls: −0.226 vs noise 0.139 (s1), −0.246 vs 0.221 (s3), −0.452 vs 0.195 (s4). s2 inside
noise. `amp_min` 0.483 — best recorded. Lockstep OK on all seeds.

**★ THE FREEZE QUESTION IS SETTLED, and my gate-2d correction was right.** The σ-bucketed
discriminator shows seeds 1 and 2 ending at σ 0.111/0.080 while **still accepting 43–50 %
at the σ 0.08 floor** (6/14, 7/14). Acceptance survives at the floor once the floor is set
above the criterion's resolution — so gate 2d's collapse to σ 0.010–0.015 with accepts
dying was **freeze-out, not convergence**. The headline I originally reported for gate 2d
("2/4 settled") is formally retracted; it was the margin starving the search.

**★★ THE OPPOSITE FAILURE NOW APPEARS, and it exposes a flaw in the anneal itself.** σ is
**at the 0.5 ceiling on 2/4 seeds** (3 and 4), which spent most of the run at maximum step
while still accepting 38–40 %. The cause is not the margin size per se — it is that
**`target_accept` has always been below the noise floor.** If a candidate differs from the
incumbent by noise alone, the difference of two window scores has sd √2·σ̂, so the
chance-level acceptance rate is **Φ(−k/√2)**:

| accept_k | acceptance from PURE NOISE | observed |
|---|---|---|
| 0.25 (gate 2e) | **43.0 %** | 26 / 30 / 39 / 43 % |
| 1.00 (gate 2d) | **24.0 %** | 13 / 23 / 16 / 34 % |

**Every run to date has had `target_accept` 0.2 sitting BELOW its own noise floor.** The
1/5th rule therefore reads a chance-level accept rate as success and inflates σ — which is
why gate 2 pinned the ceiling at k=1.0 and gate 2e pins it again at k=0.25. Note also that
observed acceptance never exceeds the noise floor in any run, so **acceptance rate alone
has never been evidence of signal**; the J-falls-beyond-noise result is, and it is
independent of it.

**Re-use context / next fix (derived, not guessed):** set **`target_accept = Φ(−accept_k/√2)`**
— the anneal then grows σ only when acceptance genuinely BEATS chance, and shrinks it
otherwise, which is the property the 1/5th rule was always meant to have. For k=0.25 that
is ≈0.43. This is a two-line change and it should be made before any further gate run.
Keep `accept_k` 0.25 (k≥1.19 would be needed to push the noise floor under a 0.2 target,
and step 0 measured that as blocking real gradients — the two constraints are only
satisfiable by fixing the target, not the margin).

Coupling improved but is still not reliably found: 0.322 / 0.568 / 0.227 / **2.000** (mean
0.779 ± 0.716, up from 0.573) — seed 4 drove it to the new cap. `plan_gain` 0.054 vs hand
0.05 and `postural` 0.651 vs 0.700 are now close; `amp_target` moved the wrong way
(0.177 vs hand 0.400) on every seed and is worth its own look.

### ★★★ 2026-08-20 — GATE 2f: THE COUPLING PROBLEM IS SOLVED (monotone on 4/4 seeds)

**Verdict: `WORKING` on the campaign's headline open problem; `PARTIAL` on the gate.**
Same arm/protocol, with `target_accept` now AUTO — derived as the noise floor
Φ(−accept_k/√2) instead of a fixed 0.2.

**★ `coupling_gain` now climbs MONOTONICALLY on every seed**, by generation block:

| seed | b1 | b2 | b3 | b4 | b5 | final |
|---|---|---|---|---|---|---|
| 1 | 0.18 | 0.35 | 0.52 | 0.64 | 0.61 | 0.641 |
| 2 | 0.02 | 0.57 | 0.50 | 1.07 | **1.56** | 1.464 |
| 3 | 0.25 | 0.68 | 0.70 | 0.74 | 0.98 | 0.815 |
| 4 | 0.10 | 0.38 | 0.51 | 1.09 | 1.28 | 1.419 |

Mean **1.085 ± 0.362** against a hand-found 1.55, and every seed non-zero — versus
gate 2e's erratic 0.32 / 0.57 / 0.23 / 2.00 (0.779 ± 0.716) and gate 2d's two seeds stuck
at zero. **Seed 2 reached 1.56, the hand point, on its own.** This is the search
rediscovering that the legs must be coupled, from egocentric error alone, reproducibly.
The whole chain — authority check → margin/step calibration → noise-floor anneal target —
was aimed at this, and it landed.

Also: **4/4 much closer to the hand point** (mean L2 distance **0.887**, vs 1.406 in
gate 2e), `amp_min` **0.523** — best recorded — and σ at the ceiling on only 1/4.

**★ A MIS-SPECIFIED TEST IN MY OWN ANALYSER, corrected.** The first-half-vs-last-half J
test reported only 1/4 seeds falling. That test is wrong for a *converging* search: gains
are FRONT-LOADED (early-block deltas −0.231 / −0.237 / −0.420 against totals −0.100 /
−0.224 / −0.372), so it averages the plateau against itself. A regression slope over all
generations has no such blind spot and gives **2/4 significant falls** (seed 3 t=−3.00,
seed 4 t=−8.73) with the other two trending down but not significantly. The analyser now
reports the slope test alongside; **earlier gate verdicts were scored on the weaker test
and are understated to that extent.**

**⚠ A WATCH ITEM, NOT A FINDING — falls read 54.75 → 66.50, which is NOT significant.**
I first recorded this as a cost of the coupling result; that overstates it. At n=4 the
spread swamps the difference: 54.75 ± 13.30 vs 66.50 ± 20.01, **t = 0.98, df 6, p > 0.3.**
It is a direction to watch, nothing more, and it would be exactly the §3.3 error to build
a lever against it at this power.

What IS specific and real is **one struggling seed**: seed 1 posts 88 falls, trips the
alarm on 25.8 % of generations (peak 32.97 against a threshold of 8), and is the only seed
whose J regressed in its final block (2.071 → 2.264). That is the ledger's recurring "one
bad seed" pattern, not a population shift.

There is a mechanism that would explain a real rise if one exists — step 0 measured
`corr(coupling, falls) = +0.220`, so the coordination the search now finds may carry
slightly more falling, and nothing in J penalises it (`w_falls` 0 by design; G1 blocks
per-comparison regression, not slow drift across many accepted steps). **But that
mechanism is a hypothesis awaiting power, not a diagnosis.**

**Re-use context:** settle it at power rather than patching it — job #1 runs at n≥20 and
measures falls directly, so it answers this question as a side effect of delivering the
phase's actual goal. Only if n≥20 confirms a rise should a falls channel be built, and
then as a small non-zero `w_falls` or an alarm-gated ratchet — **never a return to
`w_falls` 1.0**, which is what the entire noise campaign removed.

### ★★★ 2026-08-22 — JOB #1 AT POWER: the evolver's own point erases the falls tail

**Verdict: `WORKING` — the first gain vector this project did not choose by hand, and it
beats the hand point at n=20.** Four arms differing ONLY in the eight gains (evolver
present at σ=0 in all, so instruments live and nothing publishes), arena, **n=20**, 12 000
ticks, solid chassis, recentering off. The hand point was **re-run as a fresh control**
rather than compared to the recorded 3/20 tail, which was measured on a ghost chassis —
and `height_homeo_gain` is one of the evolved gains, so a ghost belly would hide what it
costs.

| metric | hand (control) | j1s1 | **j1s4 (WINNER)** | j1mean |
|---|---|---|---|---|
| falls | 0.25 ± 0.55 (**4/20**) | 0.00 (0/20) | **0.00 (0/20)** | 0.25 (4/20) |
| net_disp | 10.40 ± 2.10 | 9.87 ± 1.57 | **10.55 ± 1.20** | 10.45 ± 1.31 |
| straight | 0.658 ± 0.10 | 0.712 | **0.702 ± 0.04** | 0.679 |
| tilt_sd | 0.127 ± 0.11 | 0.085 | **0.080 ± 0.04** | 0.151 ± 0.13 |
| amp_min | 0.580 ± 0.11 | 0.563 | **0.625 ± 0.02** | 0.600 |
| step_bal | 0.137 | 0.106 | 0.095 | **0.203** |
| **rlt** (rear land) | 3847 | **2194 (−43 %)** | **3605 (−6 %)** | 3296 (−14 %) |
| **rpt** (rear push) | 12154 | 11934 | **12298 (+1 %)** | 11923 |

**j1s4 meets job #1's bar exactly: the falls tail is gone (4/20 → 0/20) AND the loaded rear
touchdowns hold** (rlt −6 %, rpt +1 %). The rival j1s1 also zeroed falls but gave back 43 %
of the rear-landing consumer — the precise failure job #1 forbade — so the acceptance
criterion did its job in discriminating them.

**★ THE LOUD RESULT IS CONSISTENCY, NOT THE MEAN.** Paired per-seed tests (same 20 seeds)
show every metric improving in direction but only marginally: falls t=−2.03, amp_min
t=+1.92, tilt_sd t=−1.78, straight t=+1.80 — all p≈0.06–0.09, **none significant**, and
falls alone is Fisher p=0.106. What IS loud is the collapse in across-seed spread:

| | sd hand | sd j1s4 | F (crit 2.53) |
|---|---|---|---|
| amp_min | 0.111 | **0.018** | **37.8** |
| tilt_sd | 0.107 | 0.036 | 9.1 |
| straight | 0.103 | 0.045 | 5.3 |
| net_disp | 2.10 | 1.20 | 3.1 |

All significant. **The hand point works well on most seeds and fails on a few; the evolved
point works on all of them.** That is precisely what PART IV was launched to fix — the
charter's opening argument was "a fixed gain vector cannot be right for every seed's
attractor", and the answer turns out to be that a *searched* fixed vector can be, where a
hand-found one was not. ⚠ It is a ROBUSTNESS win, not a speed win: means are statistically
marginal and `step_bal` is slightly worse (0.137 → 0.095).

**★ NEGATIVE WORTH KEEPING — averaging multi-modal optima FAILS.** The four seeds settled
in genuinely different basins (one abandoned coupling entirely at 0.14 while others held
1.0–1.9). `j1mean`, their arithmetic mean, is **no better than the hand point on falls
(4/20) and WORSE on tilt_sd (0.151 vs 0.127)** — it lands between basins rather than in
one. Do not average evolved vectors; pick a seed and test it.

**Promoted to the launcher** as `…__j1s4.json` with `gym_mode: arena` (metadata outranks
the env var under the UI, so a corridor default would silently give the wrong world) and
σ=0 so it cannot drift while being watched. **The operator's eye remains the promotion
gate** — the numbers say steadier, and whether the rear landing still *looks* loaded is a
question for the UI, not the table.

### ★★ 2026-08-22 — ROBUSTNESS IN THE REAL SCENARIO: no evidence of long-term damage (underpowered)

**Verdict: `NULL` — and the null is the wanted answer, but it is a weak one.** The operator's
question: does a LIVE evolver damage the gait over long horizons in the real use case, with
obstacles and perturbation? Design: **the same config file in both arms, differing ONLY in
`mutation_sigma` set at tick 1**, so "the evolver did it" cannot be confused with "the body
did it". Corridor (hump → rumble → pyramids) with outer-wall recentering ON, so the body
traverses the obstacle curriculum **repeatedly**. 400 k ticks, n=4, paired seeds.

| whole-run mean | live | frozen | Δ | t (df 3) |
|---|---|---|---|---|
| falls | 68.75 | 65.75 | +3.00 | **0.18** |
| \|tilt\| | 0.659 | 0.432 | +0.227 | 1.07 |
| fwd_v | 0.083 | 0.096 | −0.013 | −1.46 |
| energy | 0.384 | 0.390 | −0.007 | −1.28 |
| criterion J | 2.603 | 2.514 | +0.089 | 0.89 |
| within-run tilt slope | +0.243 | −0.120 | +0.362 | 0.75 |

**Nothing is significant** (all |t| ≤ 1.46 against a 3.18 threshold). Most directly: **falls
are indistinguishable** (68.75 vs 65.75, t=0.18), and falls is where gait damage would show
first. The rear-landing consumer did not collapse either — live ran *above* frozen on
rear-land events throughout.

**⚠ THE POWER, STATED HONESTLY.** Paired fall differences were −41 / +40 / +6 / +7 (sd 38),
so the **minimum detectable difference at n=4 is ~60 falls — a ~90 % change.** "No significant
damage" here means **"no LARGE damage"**; this design cannot rule out a modest one, and the
mildly unfavourable direction on tilt, fwd_v and J (all ns) is exactly what a modest effect
would look like if it were real. Settling it needs n≥12, roughly 6 h of compute.

**A METHOD NOTE worth keeping: the first cut of this analysis was wrong.** Reporting Q4−Q1
per quarter made the live arm look clearly damaged (falls +20.25 vs frozen −3.25). But both
arms swing violently quarter to quarter — live falls ran 15 / 14 / 5 / **35**, frozen ran
11 / **32** / 17 / 7 — so Q4−Q1 was reporting *which quarter happened to spike*, not a trend.
Whole-run paired means killed the artefact. **On a metric this bursty, an endpoint difference
is not a trend; fit or pool instead.**

**Context:** the corridor is genuinely harsher than the arena — ~16–17 falls per 100 k ticks
against the arena's ~9.5 — so the stress test did stress. Also note the frozen arm drifts on
its own (its falls ranged 29–108 across seeds with no search running at all), which is why
the control was indispensable: without it, live's tilt rise would have read as evolver damage.

### ★★ 2026-08-23 — n=12 CORRIDOR CONFIRMATION: the live search neither helps nor harms

**Verdict: `NULL` at usable power.** The powered replacement for the 2026-08-22 n=4 test.
Same config in both arms, differing only in whether the search runs; corridor with
obstacles, recentering ON so the obstacle curriculum is traversed repeatedly; 300 k ticks,
**n=12 paired seeds**.

| metric | live | frozen | Δ | t (df 11) | MDE |
|---|---|---|---|---|---|
| falls | 46.75 | 40.75 | +6.00 | 0.54 | 24.6 |
| **burst onsets** | 450.8 | 494.0 | −43.2 | −0.52 | 183.7 |
| **burst duty** | 6.68 % | 7.48 % | −0.80 pp | −0.58 | 3.1 pp |
| **longest burst** | 15.5 s | 15.9 s | −0.37 s | −0.16 | 5.2 s |
| mean \|tilt\| | 0.641 | 0.814 | −0.173 | −0.90 | 0.420 |
| amp_min | 0.372 | 0.385 | −0.014 | −0.26 | 0.119 |
| step_bal | 0.533 | 0.549 | −0.017 | −0.24 | 0.150 |
| net_disp | 4.93 | 4.01 | +0.92 | 0.86 | 2.34 |

**Nothing reaches significance on any metric.** Power roughly doubled against the n=4
attempt — the minimum detectable difference in falls fell from ~53 (≈81 %) to **24.6
(≈60 %)** — so this rules out a large effect in either direction, and still cannot resolve
a modest one.

**★ THE SUSTAINED-BURST METRIC (operator's, new here) EARNS ITS PLACE.** Defined as a
3-second rolling mean of forward velocity at or above 0.06 m/s — roughly 18 cm of travel
in three seconds. Its value shows in the contrast with the mean: **mean forward velocity
reads 0.01 m/s, which alone would say the body barely moves**, while the burst measure
shows 6.7–7.5 % of the run spent in genuine sustained travel, with individual runs reaching
**25 s**. A body that lurches and stalls posts the same mean as one that periodically
covers ground; only the burst view separates them. Spread across seeds is wide (sd 247 on
a mean of 451), so it has the range to move under a future A/B.

**Scope note that matters for interpretation.** This does NOT contradict job #1. The two
tests ask different questions: job #1 compared a FIXED evolved operating point against the
hand point (arena, 12 k ticks) and the evolved point won on falls and on across-seed
consistency; this test asks whether running the SEARCH LIVE in the corridor changes
anything over 300 k ticks, and the answer is that it does not, in either direction. The
evolved *destination* is better; the live *journey* is neutral in this scenario.

**Re-use context:** the corridor at 300 k ticks is a hostile measurement environment — falls
run 16–17 per 100 k ticks against the arena's ~9.5, and the frozen arm alone spans 29–108
falls across seeds. Resolving a modest live-search effect here needs either many more seeds
or a less variable scenario. The burst metric is the most promising discriminator to build
the next comparison on.

### ★★★ 2026-08-23 — THE CRITERION SEES 3 OF 8 GAINS: per-gain landscapes measured

**Verdict: `WORKING` as a diagnosis, and it reframes the search.** Each gain swept across
its full declared range while the other seven were held at the n=20-validated operating
point, search in observer mode, 6 levels × 3 windows, 3 seeds, ascending on odd seeds and
descending on even. Authority = landscape span ÷ within-level scatter.

| gain | best J at | span/noise | authority |
|---|---|---|---|
| **amp_target** | 0.15 | **3.60** | STRONG |
| **coupling_gain** | 1.6 | **3.00** | STRONG |
| **postural_gain** | 1.5 | **2.34** | STRONG |
| height_homeo_gain | 0.1 | 1.39 | weak |
| plan_gain | 0.12 | 1.11 | weak ⚠ order-dominated |
| rear_push_ext | 1.2 | 1.04 | weak |
| rear_land_gain | 0.3 | 0.93 | **FLAT** ⚠ order-dominated |
| rear_knee_plant | 0 | 0.84 | **FLAT** |

**1. Three gains carry the search.** `amp_target` is cleanly monotone (J 1.865 → 2.843 as
amplitude rises 0.15 → 0.8); `coupling_gain` is a clean bowl with its floor at 1.6.

**2. `coupling_gain`'s optimum REPLICATES.** The 2026-08-20 step-0 sweep put it at 1.2–1.6
from a different operating point; this puts it at 1.6. It also lands on the operator's
hand-found 1.55. Independent measurement, same answer.

**3. ★ THE REAR-LANDING GAINS ARE INVISIBLE TO THE CRITERION.** `rear_land_gain` and
`rear_knee_plant` — the pair PART III promoted and the operator hand-tuned — have
landscapes below their own scatter. This retro-explains behaviour recorded as puzzling:
gate 2f's rear gains scattered across seeds (rear_push ended 0.006 / 0.79 / 0.34 / 0.03)
not because the search failed but because there is no slope to climb. **A gain the
criterion cannot sense is one the search can only wander in**, and eight dimensions of
which five are weak-or-flat is a large part of why independent seeds land in different
basins.

**4. ★ TWO EARLIER "ANOMALIES" WERE THE SEARCH BEING RIGHT.** `amp_target` drifting *away*
from the hand point (0.177 against 0.400) and `postural_gain` climbing were both recorded
as odd. The landscapes show the criterion prefers amplitude 0.15 and postural 1.5; the
search was tracking its criterion faithfully. Whether the criterion is *right* to prefer
those is a separate question the operator's eye should settle.

**⚠ 5. RETRACTED SAME DAY — "two optima sit on their range boundaries, so the search is
being truncated."** `amp_target`'s argmin is at its range minimum and `postural_gain`'s at
its range maximum, and the first reading was that the true optima lie outside the declared
bounds. The resolvability check below kills that: both argmins sit *inside a flat band*,
so they are the argmin of noise, not evidence of a wall. **Widening the bounds is not
required.** The error was reading an argmin off a landscape without first asking what the
landscape can resolve — the same mistake in a new costume as gate 2d's "σ annealed to the
floor, therefore converged."

**★ 6. THE CRITERION RESOLVES BANDS, NOT POINTS — and this is what makes the
displacement test designable.** Scoring every level against one within-level noise width:

| gain | argmin | GOOD band (indistinguishable from best) | BAD band |
|---|---|---|---|
| amp_target | 0.15 | **0.15 – 0.41** | 0.67 – 0.8 |
| coupling_gain | 1.6 | **1.2 – 2.0** | 0 – 0.8 |
| postural_gain | 1.5 | **0.66 – 1.5** | 0.1 |

So the criterion cleanly separates a good region from a bad one on each of the three, and
cannot pick a point inside the good region. **The recovery test is therefore BAND RE-ENTRY,
never return-to-argmin** — demanding a point the criterion cannot resolve would fail a
working search and produce a false negative. `gainevo_landscape.py` now prints bands and
refuses to nominate an "ideal", so the tool cannot reproduce the over-read.

**★ 7. THIS RE-EXPLAINS THE `amp_target` ANOMALY A SECOND TIME, better.** The hand point
0.400 and the evolved 0.177 are BOTH inside the good band. The search was not preferring
0.177 over 0.400; it was drifting freely inside a flat basin, which is what a search does
where its criterion is blind. Finding 3 above ("the search was being right") overstates
it — the accurate version is that the search was neither right nor wrong there.

**★ 8. EACH STRONG GAIN IS SEEN THROUGH A DIFFERENT TERM.** Decomposing |dJ| across each
landscape: `coupling_gain` is **105% flow** (coupling buys phase predictability; the other
terms slightly oppose), `postural_gain` is **65% energy** (higher tone costs the servos
less than actively fighting gravity does), `amp_target` is **45% tilt-variance + 41% flow**
(high amplitude thrashes the body — tilt sd runs 0.015 → 0.456 across the range). No single
term would have seen all three, which is the multi-term criterion earning its shape.

**⚠ 9. The flow term does not oppose a quiet body, and was assumed to.** It was included as
the complement that stops the search from minimizing everything by standing still. It does
not: flow quality is magnitude × predictability, and at low amplitude the predictability
gain outweighs the magnitude loss, so flow votes WITH tilt, energy and unloaded for a
quieter gait. Measured `|fwd_v|` is lowest (0.0495) at the amplitude the criterion likes
best. The degenerate attractor did not bite here — the good band's upper half still moves
(0.154 at amp 0.28) and nothing drove amplitude to zero — but **the guard against it is
not present**, and the criterion is currently free to prefer slow. Re-use context: if a
future run creeps, this is the first place to look, and the fix is a flow term whose
magnitude factor cannot be traded away for predictability.

**⚠ 6. The hysteresis check earned its place**: `rear_land_gain` and `plan_gain` both show
an ascending/descending gap LARGER than their landscape span, so their apparent optima are
sweep-order artefacts, not landscape.

**Scope:** measured at ONE operating point with seven gains fixed, so interactions are
invisible here — a gain flat at this point could matter elsewhere. The random-init basin
study addresses exactly that.

### 2026-08-23 — flow term: MIN form, so magnitude cannot be traded for predictability

**Verdict: `NULL` on its stated purpose — and ★ THE PREMISE WAS A MISREAD, retracted below.**
Built, unit-pinned, gain-0 guarded, A/B'd the same day. It ships and stays OFF.

The flow term is the criterion's only counterweight against minimizing every other
term by standing still, and the landscape entry above measured it not doing that job.
`flow_quality = magnitude × predictability`, a quiet body is a predictable one, and the
product let the predictability a slower gait gains pay for the travel it loses.

**The fix is a MINIMUM, which is this repo's own idiom for "cannot be traded"** — the
same shape, for the same reason, as the per-leg loaded-contact guard using per-leg minima
after a mean let one leg die while the others paid for it. Quality is capped by whichever
factor is worse, so stillness-bought predictability buys nothing. Magnitude also becomes
`v/(v+ref)` instead of `clamp(v,0,ref)/ref`: monotone in speed with no ceiling to sit
against, so more travel always scores better and the factor never drops out of a
comparison.

`flow_min_form` — **0 = legacy product, and it is the default**, so every existing config
is byte-identical and this is one isolated lever.

**The instrument ships with it.** `flow_mag` / `flow_pred` are accumulated and logged
apart (`ge_fmag` / `ge_fpred`), so a window reports WHICH factor limited it rather than
only their product — the split that would have made the original defect visible on sight.
`ge_fmf` echoes the live rule into every log line so no arm can be judged without knowing
which criterion it ran under.

**Five unit tests pin the truth table, including the defect itself** (`test_gain_evolver`,
29/29): under form 0 a 0.06 m/s crawl beats 0.20 m/s travel and a *tenfold* speed range
scores identically (the saturation that removed magnitude from the landscape); under
form 1 the ordering reverses and the same range separates monotonically. Stillness scores
worst under both — the fix must not accidentally reward the thing it is guarding against.
Pinning the OLD behaviour as a test is deliberate: it proves the A/B has a real contrast
rather than comparing a fix against nothing.

**A/B armed** (`__j1s4_flowmin.json`, chained behind the basin study): the identical
landscape sweep — same config, seeds, schedules, ticks — on the three gains with
authority. Judged on whether flow still prefers the amplitude the body travels least at.
The runner greps `ge_fmf:1` from every log before reporting, per §3.2 #7.

**Not touched:** the forward-only convention (`clamp` at 0 means steady *backward* travel
scores zero magnitude) is unchanged in both forms. It is a real directional preference
inside a criterion that is supposed to have none, and it is a separate lever.

### ★★★ 2026-08-23 — AUDIT: the GainEvolver criterion's inputs against Markov-blanket discipline

**Verdict: five of six inputs are clean; `fwd_v` is an oracle, and today's flow fix
INCREASED the criterion's dependence on it.** Every criterion input traced to its
publisher in `picrawler_body.gd` rather than to the plan doc's description of it.

| input | where it comes from | verdict |
|---|---|---|
| `upright` (basis.y.y) | `_chassis…basis.y.y` = body-up · world-up | **LEGAL** — literally what an accelerometer reads |
| `joint_torque` → energy | `_prev_torque_*/MAX_SERVO_TORQUE`, 1 tick delayed | **LEGAL** — servo current |
| `foot_load` → unloaded | `_foot_load_ema / static weight` | **LEGAL** — FSR / servo-current analog |
| `foot_contact` | physics `get_colliding_bodies()` | **SIM-ONLY, not an oracle** — a foot switch is physically realisable and cheap; a fidelity gap, not a god's-eye one |
| `distress` | `perch × stall`, stall = **world-XZ chassis displacement** | **CONTAMINATED** — but `w_distress = 0.0` in every shipped config, so it is INERT. Not a live violation |
| `imu[2]` = `fwd_v` → flow | `_chassis.linear_velocity` projected on `basis.get_euler().y` (`picrawler_body.gd:5916`) | **ORACLE** |

**1. `fwd_v` is not IMU-derived. It is the rigid body's true world-frame velocity,**
projected onto its true world yaw — god's-eye twice over. No accelerometer yields it; a
real legged robot has no odometry, and integrating acceleration to velocity drifts without
bound. The codebase already says so in three separate comments ("unlike `fwd_v` this is a
LAWFUL brain input"), and the legitimacy audit flags it as "not free — worth its own pass."
The GainEvolver reads only `values[2]`, so the absolute-yaw components are not consumed
directly — but they are baked into the projection.

**2. ★ TODAY'S FLOW FIX INCREASED THE EXPOSURE, and that must be recorded as a cost of it.**
Under the legacy product the magnitude factor was a hard clamp that SATURATED at 0.05 m/s,
so across the measured range the oracle's magnitude contributed a constant and only its
volatility varied. The min form deliberately removed that ceiling — which is what makes the
term work — so the criterion now responds **monotonically, across its whole range, to a
quantity the robot cannot sense.** The fix is right by the criterion's own logic and worse
by this principle at the same time. Both are true and neither cancels the other.

**3. ★ `fwd_v` CANNOT DISTINGUISH FAST CIRCLING FROM FAST TRAVEL** — it is body-frame
forward speed, so a body carving a tight circle reads as flowing beautifully. This is the
ledger's own recorded blind metric ("high fwd_v + low net_disp = fast circling"), and the
flow term has no complement for it. Today's change made this term stronger without adding
that complement. **The legal complement is available and unused: `imu[3]` = `ang_v` is a
GYROSCOPE reading — fully egocentric.** Penalizing sustained |ang_v| would close the
circling hole without adding a gram of oracle.

**4. Scale of the exposure.** Flow is 1 of 7 weight units (~14%), and energy at 4.0 —
the dominant term — is legal. But weight share understates it: on `coupling_gain`'s
landscape the flow term accounted for **105% of the total movement in J**. The single most
convincing result in PART IV, coupling's replicated optimum, rests almost entirely on the
oracle-fed term.

**5. The legal replacement is a SENSOR, not a smarter term** (doctrine §1 step 2:
"if no egocentric observation carries the signal, the fix is a new sensor"). The standard
legged-robot answer is **planted-foot kinematic odometry** — while a foot is planted, joint
angles say how the body moved relative to it, needing only encoders and contact, both of
which this body already publishes. This is the same build the ledger already carries as
DEFERRED under post-plant slip ("planted-foot FK drift"), which makes it one build serving
two deferred needs. Optical flow from the existing forward camera is the second candidate.

**Not a reason to stop.** The charter sanctioned one speed-flavored term knowingly, and it
is named in the config, the code, and the logs rather than hidden. But "sanctioned" was
priced against a term whose magnitude was saturated out; it is a bigger loan now, and PART
IV's headline results should not be reported as oracle-free.

### 2026-08-23 — anti-circling: closing the flow term's blind spot with a LAWFUL signal

**Verdict: `IN_FLIGHT`** — built, unit-pinned, gain-0 guarded, queued behind the flow-min
verdict (never two levers in one comparison).

The audit above found the flow term blind to circling: `fwd_v` is **body-frame** forward
speed, so a body carving a tight circle reads as flowing beautifully. The ledger already
records this as a blind metric ("high fwd_v + low net_disp = fast circling") and the flow
term had no complement for it — while the min form made that term stronger.

**A new SENSOR, not a cleverer metric** (doctrine §1 step 2). `imu[3]` was the obvious
input and is **not clean**: it carries `_chassis.angular_velocity.y`, the WORLD vertical
component, which diverges from what a chassis-mounted gyro reads exactly when the body
tilts. So the body now publishes `reality.proprio.gyro` — world angular velocity projected
onto each **body** axis, `[roll, YAW ABOUT OWN UP, pitch]`, normalized by π. A real MEMS
gyro measures precisely this. Additive and default-no-consumer, so it is byte-identical
until a module names it. Using the almost-legal signal to fix a legality complaint would
have defeated the point.

**★ THE DESIGN IS THE TIMESCALE AND THE SIGN, not the magnitude.** Heading regulation is
one of the three behaviours this project reads as genuine capability (§3.3), so a term
punishing turning *per se* would punish the thing we most want to see. The turn bias is a
**signed** EMA at **~5 s** (`flow_turn_alpha` 0.004, deliberately ~5× slower than
`flow_alpha`): a body correcting left-then-right averages toward zero, a body orbiting
holds a bias. Built on mean |yaw rate| it would have penalized exactly the wrong thing,
and a test pins that distinction rather than trusting it.

`straight = 1/(1 + flow_turn_k·|turn bias|)` joins whichever combining rule is in force —
the min under form 1 (so circling cannot be bought off with speed and smoothness either),
the product under form 0. **`flow_turn_k` defaults to 0**, which makes the factor exactly
1.0 and cannot perturb either rule.

**Four tests, 33/33 total.** A sustained turn registers (factor < 0.85) and scores worse
than going straight at identical speed and steadiness; **the same |yaw rate| alternating
every 0.8 s stays above 0.95** — the test that fails if the term is built the naive way; a
fast, glassy-smooth circle has the turn factor bind under the min form; and at k=0 a turn
is invisible under BOTH forms.

**Byproduct — 40,000 spurious ERROR lines per run, fixed.** `_emit_jsonl` probes five
per-leg EPM ids from an earlier stack generation; absent modules return `""`, and parsing
`""` is a Godot ERROR. A 300k-tick run emitted ~40k of them. The parse result was already
discarded on failure so the guard is behaviour-identical — what it buys back is a run log
where `grep -i error` means something. **This nearly cost a false alarm today**: a routine
liveness check on the basin study reported 27,096 "errors" and read as a crash.

### ★★★ 2026-08-23 — BASIN STUDY: the 8-D search diffuses; only what the criterion sees resists

**Verdict: `NULL` on convergence, `WORKING` as a diagnosis — and it names the fix.**
12 random start vectors × 2 bodies (`cad`, `measured`), arena, 300k ticks each (35
generations), body seed pinned so the only difference between runs is where the search
started. All distances in normalized gain units (0 = `gain_min`, 1 = `gain_max`).

**1. The search does NOT converge. It DIVERGES.** Mean pairwise spread between runs went
**1.070 → 1.206** (cad) and **1.070 → 1.159** (measured). Independent starts end further
apart than they began.

*(Figures corrected the same day: the first pass normalized against a hand-copied bounds
table that had drifted from the shipped config on two of eight gains. `gainevo_basin.py`
now READS the bounds from the basin configs, so the table cannot drift again. Every
verdict below is unchanged — per-gain sd ratios are scale-invariant, which is exactly why
the authority cross-check, the finding this entry rests on, was untouched.)*

**2. ★ BUT THE MEASURED AUTHORITY PREDICTS THE ORDERING — replicated on both bodies.**
End/start spread ratio, averaged by the landscape sweeps' verdict:

| authority | cad | measured |
|---|---|---|
| STRONG | **0.95** | **0.96** |
| weak | 1.11 | 1.11 |
| FLAT | **1.70** | **1.41** |

Monotone, twice, on data the sweeps never touched. **A gain the criterion can see resists
being scattered; a gain it cannot see scatters freely.** This is the sweeps' sharpest
independent confirmation, and it arrives inside a negative headline.

**3. ★ THE MEANS DO NOT TRAVEL — so this is DIFFUSION, not selection.** Reporting spread
alone would have hidden it. Across both bodies the STRONG gains' means move by ≤0.04
(`amp_target` −0.021/+0.024, `postural_gain` −0.018/+0.024), while the largest movement
belongs to `rear_push_ext` (+0.24/+0.27) — a gain with no landscape at all, i.e. pure
drift. **`coupling_gain` moved in OPPOSITE directions on the two bodies** (+0.185 cad,
−0.039 measured), which kills any claim of a consistent pull. The second body earned its
place by producing that disagreement.

**4. The mechanism, stated plainly: five of eight dimensions are noise to the criterion,
and they inject that noise into every single comparison.** That is why even the three
gains with real landscapes only manage a 0.82–0.96 ratio instead of collapsing. The
searched space is mostly dimensions the search cannot navigate, and each one taxes the
ones it can.

**5. ⚠ THE TWO-BODY COMPARISON IS UNINFORMATIVE, and the script now says so itself.**
Centroid separation 0.267 against within-body spread 1.182 reads as "both bodies agree",
but that is satisfied trivially when neither body converged — the within-body cloud
swamps any body difference. It is a statement about the SEARCH, not about the two bodies.
`gainevo_basin.py` refuses the verdict when spread exceeds 2× separation rather than
letting a future reader take the null at face value. **The sim2real question is not yet
asked, let alone answered.**

**6. ⚠ Scope: 35 generations.** Each generation costs two 4000-tick windows, so 300k ticks
buys 35. This is a real bound on the claim — "does not converge in 35 generations from
random in 8-D" is what was measured, not "cannot converge."

**→ THE FIX THIS POINTS AT: search 3-D, not 8-D.** Restrict the searched vector to
`amp_target`, `coupling_gain`, `postural_gain` — the three with demonstrated authority —
and hand-set the rest. Two independent measurements now support it: the landscapes say
the other five carry no gradient, and the basin study says they scatter while taxing the
three that do. This is the next lever, and it is cheap.


### ★★★ 2026-08-23 — RETRACTION + the finding underneath it: the flow term is an AMPLITUDE proxy

**The A/B killed my own diagnosis, which is what an A/B is for.** Identical sweep, same
config/seeds/schedules/ticks, `ge_fmf:1` verified in all nine logs.

**1. ★ THE PREMISE WAS FALSE.** I wrote — into the ledger, the code comments, the config
metadata, three commit messages, and a report to the operator — that the flow term "scored
BEST at the gait amplitude whose mean travel was LOWEST (0.0495)." **It did not.** Flow's
optimum sat at amp 0.28, where `|fwd_v|` = 0.1537 — the FASTEST level in the low band. The
slowest level scored *worse* on flow (0.4086 vs 0.3733).

The mistake: `|fwd_v|` is **not monotone in amplitude** (0.050, 0.154, 0.093, 0.069, 0.110,
0.145). I saw flow preferring low amplitude, silently read that as preferring low travel,
and never checked the column sitting next to it in my own table.

**2. ★ WHAT FLOW ACTUALLY MEASURES — and this is the real finding.**

| | corr with `|fwd_v|` | corr with amplitude |
|---|---|---|
| legacy product | **+0.256** | **+0.967** |
| min form | +0.249 | +0.970 |

**The flow term is a gait-amplitude proxy, not a travel term.** Mechanically it is
`1 − 1/(1+k·vol)` in disguise: with magnitude saturated, flow reduces to the *volatility*
of `fwd_v`, and velocity volatility tracks amplitude almost perfectly. **The criterion's
one sanctioned speed-flavored term does not measure speed.**

**3. The saturation half was real.** `ge_fmag` — the instrument shipped with the fix —
reads median 0.534, i.e. `flow_ema` ≈ 0.057, above the 0.05 ceiling: magnitude was pinned
at 1.0 most of the time, exactly as the unit test asserts. **A true defect whose repair
changed nothing measurable**, because the term's variance never lived in magnitude.

**4. Verdict on `flow_min_form`: NULL, ships OFF.** Span/noise 3.60 → 3.66, bands
identical, correlation structure unmoved. It is defensible on first principles (a tenfold
speed range scoring identically is indefensible) and it is free to keep as dead-by-default
code — but principle is not evidence, and it does not get enabled on this. **Re-use
context: it becomes live the moment a LEGAL travel signal replaces `fwd_v`**, because only
then does magnitude carry information worth un-saturating.

**5. This SHARPENS the audit rather than softening it.** I told the operator the min form
"increased our exposure to the oracle." Weaker than stated: the term's response is
dominated by `fwd_v`'s volatility in both forms, so removing the ceiling did not make it
track the oracle's *magnitude* any harder. The input is still illegal and the exposure is
still real — but the bigger problem is now visible, and it is not exposure. **The criterion
has NO working travel term at all**, so nothing in it opposes a body that stops moving
except by proxy through amplitude. That is a much stronger argument for planted-foot
kinematic odometry than the one I made this morning: it is not only the legal replacement,
it is the only way this term starts measuring what its name claims.

**6. The anti-circling factor is untouched by this.** Its motivation — `fwd_v` is
body-frame, so a circle reads as travel — is independent of the misread, and a term that
tracks amplitude rather than travel cannot see circling either. It remains queued.

**Process note (§3.2 #6, self-inflicted): I built, tested, documented and shipped a fix on
an unverified reading of my own table.** The unit tests all passed because they pinned the
*mechanism* I believed in, not the *claim* about the robot. Mechanism tests cannot catch a
misdiagnosis; only the A/B could, and it did — within hours, because it was armed at build
time. The lesson is not "test more", it is **run the A/B before writing the justification
into five places.**

### ★★★ 2026-08-24 — LEG ODOMETRY ON THE REAL ROBOT: correcting "the body already publishes it"

**Verdict: `DEFERRED`, with a corrected design and a MEASUREMENT that must come first.**

**★ 1. RETRACTION.** Yesterday's audit said planted-foot odometry needs "only encoders and
contact, **both of which this body already publishes**." True in **sim only**. The physical
picrawler (`picrawler_sim2real_port.md`, robot on the bench since 2026-08-10) has:

| what the design assumed | what the robot has |
|---|---|
| joint encoders | ❌ **hobby servos accept an angle and report nothing** |
| foot contact switches | ❌ none |
| servo current (torque surrogate) | ❌ **none** — Robot HAT V4's A4 is battery *voltage*; needs an INA219/INA3221 |
| belly ToF rangefinder | ❌ not yet acquired (VL53L0X planned) |
| magnetometer | ❌ unusable (12 servos ≈ 7 µT each vs 50 µT earth) |

The legality bar is `{ commanded joint angles, IMU 6-axis, known link geometry }` — and
**servo current has no hardware behind it**, so it is thinner than the legitimacy doc's §5
table implies.

**★ 2. CONTACT SENSING IS NOT THE BLOCKER — a robust consensus makes it optional.** A
planted foot's body-frame velocity *is* the negative of the body's. A swinging foot's is
not, and differs grossly. So take the **median across all four legs**, never the mean: in a
crawl gait roughly three feet are down, the median is carried by the planted majority, and
swing legs fall out as outliers **without any contact input at all**. Foot switches would
sharpen this and are cheap — they are an optimization, not a prerequisite.

This is the same differential the DEFERRED post-plant-slip entry required, read the other
way round: **the consensus is body motion, and each leg's residual from it is that leg's
slip.** One build, both deferred needs, as previously noted — but now with a form that
survives the missing sensors.

**★ 3. ACCELEROMETER SPIKES SOLVE A DIFFERENT PROBLEM THAN THE ONE ASKED.** A touchdown
impulse is genuinely detectable, but one body-mounted IMU sees the **sum** of all four legs:
it gives *timing*, not *identity*, and attributing a spike to a leg needs the gait phase,
which is efference and is wrong exactly when a foot misses. Liftoff produces no impulse at
all, so spikes give at best half of stance. Their real value is elsewhere and worth keeping:
detecting a **full flight phase**, where no foot is planted and the kinematic estimate is
undefined, so IMU integration must bridge it. Rare in a crawl gait, so low priority.

**★ 4. THE DOMINANT ERROR IS THE COMMANDED-FK LOAD BIAS — AND THE NUMBER WE HAVE IS THE
WRONG AXIS.** Commanded-angle FK is blind to servo deflection: a planted leg sags while its
command still says "I am here." Measured `fk_cmd_err` = **22 mm mean / 38 mm max** — but
that is `mean |commanded − achieved| foot HEIGHT` (`picrawler_body.gd:_dbg_fk_cmd_err`).
**Odometry needs the HORIZONTAL component, which has never been measured.**

Two things cut in opposite directions and neither is yet quantified:
- *In our favour:* velocity is a **difference across ticks**, so any constant deflection
  cancels. Only the deflection's **change within a stance phase** (load 0 → peak → 0)
  corrupts the estimate.
- *Against us:* leg reach is only 156.5 mm, so a stride is tens of mm. If the horizontal
  deflection swing is of that order, it dominates.

**→ THE FIRST BUILD IS A MEASUREMENT, NOT AN ESTIMATOR.** Both FK variants (`_fk` from
achieved angles, `_cmd` from commanded) already run every tick through the same chain, with
the miswiring control already in place (`_dbg_fk_valid_err` = 1.1 mm, 20× smaller than the
deflection, which is what licenses the comparison). Extend that comparison to report
**horizontal** error and, specifically, its **within-stance change**. Cheap, no new
mechanism, and it decides whether this estimator is worth building before any of it is
written. Ship the estimator only if the within-stance horizontal drift is small against
stride length.

**★ 5. THE INSERTION POINT IS ONE TOPIC.** `vel_ego` (`picrawler_body.gd:6071-6082`) is the
sole consumer path — `PlaceGraphPlanner`, `PlayLoop` and `PlaceNav` all read it via
`vel_topic`, and each already documents tolerating drift as common-mode. Replacing its
publisher converts the nav stack **and** the GainEvolver's flow term from soft-oracle to
legal in one swap, with no consumer changes.

**★ 6. Whatever gets built must keep a GRAVITY REFERENCE.** The legitimacy doc's §6 is the
one empirically-established porting invariant — gravity-referenced signals emerged in
**8/8 cells across both actuation backends**, and no other family did. Horizontal travel
must be separated from vertical bounce through fused accel+gyro attitude, not assumed.

### ★★★ 2026-08-24 — THE SEARCH ACCEPTS AT CHANCE: every run used a window the project had already refuted

**Verdict: `NULL` on the 3-D lever, and ★ the basin study's DIAGNOSIS is retracted — the
observation stands, the cause was wrong.**

**1. 3-D did not fix it.** Same protocol as the 8-D basin study, same 12 start points
projected onto the three gains with authority, other five pinned at j1s4 so generation 0 is
the same body. Spread 0.747 → **0.777** — still no convergence. The means appear to travel
toward the good bands (coupling +0.259, postural +0.209) but **paired tests say no**:
t = +0.88 / +1.09 / +0.40, sign counts 6/12, 8/12, 7/12. Coin flips. *Reporting those mean
moves as a partial success was one sentence away and would have been wrong.*

**2. ★ THE CAUSE: acceptance sits AT THE NOISE FLOOR.**

| arm | accept rate | noise floor (k=0.25) |
|---|---|---|
| 8-D cad | 0.383 ± 0.074 | 0.430 |
| 8-D measured | 0.290 ± 0.101 | 0.430 |
| 3-D cad | 0.373 ± 0.072 | 0.430 |

**The search is accepting by coin flip.** That is diffusion with extra steps, it is
independent of dimensionality, and it is exactly why dropping 8-D to 3-D changed nothing.

**3. ★ AND THE REASON IS A CONFIG THAT DRIFTED FROM ITS OWN MODULE DEFAULT.** Every config
— `basin_*`, `basin3d_*`, `__j1s4`, `__gainevo_live` — runs `eval_window_ticks: 4000`.
The module's default is **12000**, and its own parameter documentation says why:

> *"Gate 2 MEASURED 4000 to be far too short (noise sd 1.4 vs signal 0.3); with the
> post-gate-2 criterion ~11k suffices, so the default is 12000."*

**The window was raised in the code after gate 2 measured it inadequate, and the configs
were never updated.** So the basin study, the 3-D test and the deployed operating point all
ran a search at the one window length this project had already refuted in writing. Short
windows inflate σ̂, an inflated σ̂ makes the margin unfilterable, and the search accepts noise.

This is §3.2 #7 — *did the arm you think you ran actually load?* — in its least visible
form. Nothing failed, no guard fired, the parameter was legal and in range. **A default
changed and its consumers did not, and a stale value is invisible precisely because it
runs perfectly.**

**4. What survives.** The authority cross-check from the basin study is **untouched** and
remains the campaign's best independent confirmation: gains the criterion can see resisted
scattering (STRONG 0.95/0.96) while gains it cannot see scattered freely (FLAT 1.70/1.41),
replicated on two bodies. That effect appeared *despite* chance-level acceptance, which
makes it more impressive, not less. What is retracted is the diagnosis built on top of it —
"five of eight dimensions are noise and tax the three that work" — which the 3-D test
falsified within hours of my writing it.

**→ NEXT, AND CHEAP FIRST: verify the mechanism before repeating the study.** 6 runs at
`eval_window_ticks: 12000`, 400k ticks each (~16 generations), measuring one number: does
acceptance drop below the 0.430 floor? A basin study at 12000 costs 2.8× the ticks per
generation, so it is worth ~9 hours only once the acceptance claim is confirmed for ~2.

### ★★★ 2026-08-24 — HARDWARE AUDIT: 1 of 7 criterion weight units is buildable on the real robot

**Verdict: `WORKING` as a porting diagnosis. Two findings — one about what the criterion
measures in SIM, one about what survives the port.**

**★ 1. THE DOMINANT TERM IS NOT MEASURING ENERGY.** `w_energy` is **4.0**, more than every
other term combined, and it reads `reality.proprio.joint_torque`. The body's own
registration string for that topic says:

> *"⚠ **NOT applied torque and NOT a load proxy despite its original description**: it sets
> the motor's max-impulse cap, and with Kp=20/Kd=8 it is **DOMINATED BY THE −Kd·omega
> damping term** (measured corr with joint motion −0.46..−0.56), i.e. **it is mostly a
> negated velocity copy**. Use JointLoad for load."

So the criterion's heaviest term is largely **a measure of how fast the joints are moving**.
Combined with the flow term turning out to be a gait-amplitude proxy (r = +0.97), **two of
the criterion's three live terms are motion-magnitude proxies wearing other names.** The
term was introduced and weighted on a signal/noise argument that was never wrong about the
*statistics* — only about what the signal was.

The honest load signal already exists and is unused here: `joint_load` (velocity-tracking
deficit) — though it was measured NOT to discriminate stance from swing (0.383 vs 0.257),
so it is not a drop-in. This needs its own lever, not a quiet re-point.

**★ 2. PORT AUDIT — what the criterion can actually sense on the physical robot.**

| term | weight | topic | on hardware today |
|---|---|---|---|
| tilt sd | 1.0 | `upright` | ✅ accelerometer |
| **energy** | **4.0** | `joint_torque` | ❌ a sim PD-controller internal; **no hardware analog exists** |
| unloaded | 1.0 | `foot_load` | ❌ needs foot FSRs |
| flow | 1.0 | `imu[2]` = `fwd_v` | ❌ oracle even in sim |
| G1 falls guard | — | `upright` | ✅ |
| G2 per-leg minima guard | — | `foot_load` | ❌ needs foot FSRs |

**One of seven weight units, and only one of two guards, survives the port as-is.**

**★ 3. THREE LOW-COMPLEXITY ADDITIONS RECOVER SIX OF SEVEN.** All are I²C or the existing
ADC header — no new buses, and the IMU wiring is already committed.

| add | cost | recovers |
|---|---|---|
| **INA219** on the servo rail | ~$5, I²C | the **energy** term (4.0) as *real* current — bus total, not per-joint, which for an energy term is the more honest quantity anyway |
| **4× FSR on A0–A3** | ~$10, existing ADC | `foot_load` (1.0) **+ the G2 guard** + stance ground truth for odometry bring-up |
| **VL53L0X** belly ToF | ~$5, I²C `0x29` | the belly-clearance channel the deployed height homeostat rides on — CLAUDE.md's "hump breakthrough" |

FSRs **supersede foot microswitches** — an FSR above threshold *is* a contact switch and
also carries the load magnitude, so do not spend GPIO on both. Keep A4 on battery voltage;
it is the brownout/stall detector.

**★ 4. `vel_ego` FROM ACCELEROMETER ALONE: NO — and tolerating noise does not help, because
the error is BIAS.** The accelerometer cannot separate tilt from acceleration, so an
attitude error θ leaks `g·sin(θ)` into the horizontal axis:

| attitude error | leak | drift after 1 s | after 5 s |
|---|---|---|---|
| 0.5° | 0.086 m/s² | 0.086 m/s | 0.43 m/s |
| 1.0° | 0.171 m/s² | 0.171 m/s | 0.86 m/s |

Measured body speed is **0.05–0.15 m/s**, so a **1° attitude error produces false velocity
exceeding the true signal within one second**. Noise averages down as 1/√N; a gravity leak
integrates **linearly**, and on a walking robot it is **gait-synchronous**, so it does not
even average out over a stride. Gyro fusion bounds attitude but the picrawler's untrimmed
ZRO is ±20 °/s and the accelerometer's own correction is corrupted by body acceleration —
which is the classic legged-robot version of this problem.

**★ 5. THE FIX NEEDS NO NEW SENSOR — it is a fusion of two things already on the parts
list.** Integration drift is bounded by a velocity *observation*, and leg FK supplies one
every tick from commanded angles. The two error profiles are exactly complementary:

- **Leg FK** is bias-free over short intervals but **over-reports when feet slip** (a foot
  sliding backward in body frame is indistinguishable from the body moving forward).
- **The IMU** catches that slip but drifts.

**Their disagreement IS the slip signal** — which is precisely the DEFERRED post-plant-slip
term. One fusion delivers a legal `vel_ego`, the flow term's missing travel magnitude, and
the deferred slip term, from commanded angles plus the IMU already being wired.

### ★★★ 2026-08-24 — THE WINDOW FIX FAILED, AND THE REASON RETIRES ACCEPTANCE AS A DIAGNOSTIC

**Verdict: `NULL` on the window lever. ★ And the three diagnoses before it were all reading
a controlled variable.**

**1. Longer windows changed nothing.** 6 runs at `eval_window_ticks` 12000 vs 12 at 4000,
same config otherwise:

| arm | accepts/gens | rate | σ̂ | margin |
|---|---|---|---|---|
| 3-D, window 4000 | 161/432 | 0.373 ± 0.023 | 0.168 | 0.0420 |
| 3-D, window 12000 | 38/96 | 0.396 ± 0.050 | 0.125 | 0.0313 |

Difference −0.023 ± 0.055, **z = −0.42**. The window did exactly what it should — σ̂ fell
0.168 → 0.125 — and acceptance did not move.

**2. ★ IT CANNOT MOVE. THE ACCEPTANCE RATE IS A CONTROLLED VARIABLE WHOSE SETPOINT IS
CHANCE.** Two lines of the module, read together:

```cpp
double target = (target_accept_ >= 0.0) ? target_accept_ : noise_floor_rate();
sigma_ = clamp(sigma_ * (rate > target ? anneal_up_ : anneal_down_), sigma_min_, sigma_max_);
```
```cpp
double GainEvolver::noise_floor_rate() const { return 0.5 * std::erfc(accept_k_ * 0.5); }
```

Every config runs `target_accept: -1.0` = AUTO. **So the anneal servos σ until the accept
rate equals the noise floor — the rate defined as what a search reaches while learning
nothing.** It is a controller whose setpoint is "learn nothing," and it holds that setpoint
regardless of window length, dimensionality, or body. That is why all four arms read
0.29–0.40 against a 0.430 floor: **I was measuring the setpoint of a controller and
reporting it as evidence about the search.**

Doctrine §8 already names this — *"an instrument that measures your own corrector is not a
measure of the thing"* — and it caught me anyway, because the corrector was three files
away from the number. The gate-2f fix moved `target_accept` from a fixed 0.2 (below the
floor, so the rule read chance as success) to AUTO (**at** the floor). Right direction,
stopped one step short: at the floor the equilibrium *is* chance.

**3. ★ THE UNCONFOUNDED MEASUREMENT, WHICH NOBODY HAD RUN: DOES J ACTUALLY FALL?**
Incumbent J per generation, first third vs last third, on data already collected:

| arm | n | first J | last J | slope/gen | runs improving | t |
|---|---|---|---|---|---|---|
| 8-D cad, 4000 | 12 | 2.206 | 2.142 | −0.0024 | 7/12 | −0.75 |
| 8-D measured, 4000 | 12 | 2.591 | 2.469 | −0.0048 | 7/12 | −1.22 |
| 3-D cad, 4000 | 12 | 2.125 | 2.176 | +0.0023 | 6/12 | +0.67 |
| 3-D cad, 12000 | 6 | 2.313 | 2.196 | −0.0092 | 4/6 | −1.51 |

**The search does not improve its own criterion from random starts in ANY configuration
tested.** Every slope is within noise and every improving-run count is a coin flip. The
12000 arm has the largest negative slope and the fewest runs — a hint, not a result.

**4. ⚠ THIS PUTS A QUESTION MARK ON JOB #1'S ATTRIBUTION — not on its result.** The j1s4
vector is genuinely better and that stands: n=20, falls tail 4/20 → 0/20, `amp_min` sd
0.111 → 0.018, F=37.8. What is now in doubt is **what found it.** The protocol was: run the
search from the hand point, **operator selects a settled vector from the panel**, bake, A/B.
If the search diffuses, that is a *random search with human selection of the endpoint* —
still a legitimate way to find a better vector, and it did, but it is not the charter's
claim of a vector that **settles from its own criterion**. The A/B of the resulting vector
is untouched; the mechanism credited for producing it is not.

**→ WHAT THIS LEAVES STANDING.** The criterion demonstrably carries real information: three
gains have landscapes above noise, `coupling_gain`'s optimum replicated from two independent
operating points and matched the hand-found value, and authority predicted resistance to
scattering on both bodies. **The criterion works; the search on top of it does not yet.**
That is a far better position than the reverse, and it says the next work is in the
accept/anneal loop, not in the criterion.

**→ NEXT.** `target_accept` must sit **above** the floor for acceptance to indicate real
improvement — but raising it invites the gate-2d/2e freeze-out, so the honest move is to
anneal on something that is not noise-dominated: realized ΔJ over a block of generations,
or the fraction of accepts that survive re-evaluation. Either way the first step is a
sweep of `target_accept` against measured ΔJ, because ΔJ is the only signal in this loop
that the anneal does not already control.

### ★★ 2026-08-24 — target_accept IS A STEP-SIZE SELECTOR, and a self-inflicted seed confound

**Verdict: the `target_accept` axis is `TAUTOLOGY` — it does nothing the σ bounds do not
already do. The ΔJ question is re-running with the control it needed.**

**★ 1. HARNESS TRAP, self-inflicted: `OGMA_SEED` overrides the module's own `seed` param.**
The sweep pinned `OGMA_SEED=1234` for every run (to hold the body fixed) and tried to vary
the search RNG through each config's `seed`. `OGMA_SEED` is the **master override** and
rewrites it — the GainEvolver's schema says so in its own doc string. Result: all three
replicates of every arm were **byte-identical**. n=3 was n=1, and ~5 hours of compute bought
replication of a single trajectory.

The tell was an analyzer crash — `stdev` of three identical numbers is zero. **A divide-by-
zero was the only thing standing between this and three "seeds" reported as agreement.**
`gainevo_tsweep.py` now names the condition instead of dying on it. To vary the search, vary
`OGMA_SEED`; the module `seed` param is not an independent knob.

**★ 2. `target_accept` CONFIRMED AS A σ SELECTOR — the axis is redundant.** Predicted from
the anneal's structure, then measured: with acceptance pinned near the noise floor,
`rate > target` has a fixed answer, so the anneal drives σ to one bound.

| arm | target | σ settled | ΔJ | coupling end |
|---|---|---|---|---|
| `tgt070` | 0.70 (above floor) | 0.080 = σ_min | −0.175 | 0.769 |
| `fix008` | — (σ pinned 0.08) | 0.080 | **−0.175** | **0.769** |
| `tgt020` | 0.20 (below floor) | 0.180 → σ_max | −0.070 | 1.054 |

`tgt070` and `fix008` are **byte-identical**. `target_accept` is a step-size knob wearing a
statistician's name, so the tgt arms are dropped and the honest axis is σ.

**★ 3. THE TWO HALVES DISAGREE — and that is the interesting part.** Single seed, so a
direction only. Starting displaced (coupling 0.30, inside its BAD band):

| arm | σ | ΔJ | coupling end | re-entered 1.2–2.0 |
|---|---|---|---|---|
| auto | 0.08 | **−0.132** | 0.793 | no |
| fix003 | 0.03 | −0.122 | 0.838 | no |
| fix008 | 0.08 | **−0.175** | 0.769 | no |
| **fix020** | 0.20 | **+0.122** | **1.614** | **YES** |
| fix045 | 0.45 | +0.111 | 0.937 | no |

**J falls in every arm that leaves coupling displaced, and rises in the one arm that
recovers it.** The single-gain landscape said coupling's good band is 1.2–2.0 with 0–0.8
bad; here the search finds lower J *without* leaving the bad band — and `fix020`, which did
recover coupling, paid for it by driving `amp_target` 0.385 → 0.575, outside amp's own good
band. **The single-gain landscapes do not compose**, which is the interaction question the
basin study was built for and could not answer while diffusing.

**⚠ 4. THE MISSING CONTROL, and why the ΔJ numbers above cannot yet be read.** There was no
**σ=0 observer** arm. J falling over 28 generations is equally consistent with the body
simply settling after the displaced start — no search required. Five of seven arms showing
ΔJ < 0 looks like a search working and may be nothing but relaxation. **This is the same
shape as the acceptance-rate error: a number that moves for a reason other than the one
being credited.** The re-run adds `sigma0` and it is the arm the others are measured
against.

### ★★★ 2026-08-24 — n=6 KILLS THE n=3 SIGNAL: the search does not beat not-searching

**Verdict: `NULL` on the search, measured against the control it needed. The criterion
stands; the (1+1)-ES on top of it does not.**

All arms start displaced — `coupling_gain` at 0.30, inside its measured BAD band — with
`amp_target` and `postural_gain` in theirs. ΔJ is the last third of scored windows minus
the first third; negative means the criterion improved.

| arm | σ | n | ΔJ | sd | vs control |
|---|---|---|---|---|---|
| **`sigma0` — NO SEARCH** | 0 | 6 | **+0.033** | 0.117 | — |
| `auto` — what ships | 0.08 | 6 | **−0.033** | 0.210 | t = **−0.67**, ns |
| `fix008` — σ pinned | 0.08 | 6 | +0.100 | 0.336 | t = +0.46, ns |

**★ 1. THE n=3 RESULT WAS NOISE, and this is the clearest demonstration of §3 rule 3 the
project has.** At n=3 `auto` gave ΔJ **−0.181** against a −0.001 control, t = −2.48 —
close enough to 2.8 to be tempting, and it was reported as "real-looking, underpowered, not
yet a claim." Three more seeds moved it to **−0.033**, t = −0.67, with the sd rising 0.088
→ 0.210. One of the new seeds drove coupling to 2.00 and carried its own ΔJ far off the
others. **The effect did not shrink because the estimate got better; it was never there.**

**★ 2. THE CONTROL WORKED, AND THAT IS WHAT MAKES THIS READABLE.** `sigma0` holds coupling
at 0.30 in all six runs and gives ΔJ +0.033 — J does **not** drift on its own after a
displaced start. So the ΔJ numbers are attributable, and the answer is that searching and
not searching are indistinguishable. Without this arm the earlier five-of-seven-arms-improve
pattern would have read as a working search.

**★ 3. BAND RE-ENTRY: 1 of 12 searching runs.** The displaced gain stayed displaced. Both
halves of the pass criterion fail together, which at least is consistent — the earlier
disagreement (J falling while coupling stayed put) was itself an artifact of the noise.

**★ 4. The anneal contributes nothing measurable.** `auto` vs `fix008` — same σ, anneal live
vs neutralized — differ by 0.133 at t = −0.82. Consistent with the anneal servoing to a
setpoint that is by construction the chance-acceptance rate.

**WHAT THIS DOES AND DOES NOT SAY.** It does not say the criterion is blind: three gains
have landscapes above noise, `coupling_gain`'s optimum replicated across two independent
operating points onto the operator's hand-found 1.55, and measured authority predicted which
gains resist scattering on two different bodies. **The signal is there and the search cannot
climb it.** Scope: arena, 240k ticks (~28 generations), one displaced start, 3-D. Longer
runs, a different accept rule, or a criterion whose terms are not two motion-magnitude
proxies are all untested.

### ★★★ 2026-08-24 — THE PRE-REGISTERED DECISION EXPERIMENT: a controlled null, and the criterion turns out to have TWO descents

**Verdict: `NULL` on displacement recovery — now against the control it always needed, at
the step size the accept arithmetic favored, with the verdict rule committed before the
data existed** (`picrawler_part4_audit.md` §6, commit 1a52b82). And ★ the decomposition
underneath the null upgrades the diagnosis: the search descends its criterion; the
criterion does not point where the single-gain landscape said.

**Protocol.** Arena, difficulty 0.3, 600k ticks (49 scored windows at the now-default
12000 window; searching arms reach generation 24), `OGMA_SEED` 1–6, three arms from the
same displaced start — coupling 0.30 (BAD band), amp/postural in their good bands —
**written into the MotorEPM params as well as the evolver seed**, because the audit found
the previous σ=0 control never published and its body silently ran the j1s4 point
(coupling 1.655). `ge_vec` reported the module's never-published intent; the "sigma0
holds coupling at 0.30" line in the earlier n=6 entry was that instrument artifact.
First-third J levels now certify arm equivalence: 2.33 / 2.40 / 2.42. Logs:
`~/xaq_runs/tsw2_20260824/` (the configs regenerate from `gainevo_make_arms.py tsweep`).

**1. The pre-registered primary outcome — every arm fails both halves.**

| arm | n | dJ (last−first third) | vs control (Welch t) | band re-entry |
|---|---|---|---|---|
| `sigma0` control | 6 | **+0.029 ± 0.055** | — | 0/6 (cannot move) |
| `fix008` (σ=0.08) | 6 | −0.028 ± 0.086 | −1.34, ns | 0/6 |
| `fix020` (σ=0.20) | 6 | −0.073 ± 0.172 | −1.41, ns | **1/6** |

The registered rule said fix020 re-entry ≤ 1/6 ⇒ the null is defensible, and that is the
row that came back. **"The (1+1)-ES does not recover a displaced gain on this criterion"
now stands at n=6, both step sizes, 24 generations, the validated window, against a true
displaced control.** The prediction of ≥3/6 re-entry at σ=0.20 was WRONG, and §3 below is
the diagnosis of why.

**2. ★ THE DECOMPOSITION: coupling→0 is SELECTED, and energy pays for it.** Per-run
weighted term movement (first→last third):

- **7 of 12 searching runs drove coupling to ≤0.08** — a boundary, coherently, not
  diffusion around 0.30. In every one of them the **energy term (weight 4) improved while
  the flow term (weight 1) worsened** (worst case `fix008_s6`: energy −0.185, flow
  +0.244). Quieting the body is a descent direction, and the 4:1 weight ratio makes it
  the wider one.
- The **two runs that moved coupling UP toward the band** (`fix020_s2` → 0.90,
  `fix020_s5` → 1.31, the sole re-entry) posted the **two best dJ of the experiment**
  (−0.284, −0.224), with flow AND energy improving together.

**3. ★ WHY THE PREDICTION FAILED — the arithmetic modeled the wrong thing.** The audit's
accept arithmetic (selection differential ~0.24 at σ=0.20) was about magnitudes, and the
magnitudes were fine: the search demonstrably descends J. What it assumed silently is that
the band is the ONLY descent from 0.30. It is not: coupling 0 is **0.3 units away** with
the heaviest term paying for the trip, while the band is **0.9 units away**. A working
gradient-follower goes down the nearest slope. **The displacement test was set, unknowingly,
between two attractors with the wrong one closer** — so its null measures the one-gain
landscape's failure to compose (already seen once at n=1: "the single-gain landscapes do
not compose"), not the search's failure to descend. The criterion's motion-proxy
composition (flow = amplitude proxy r=+0.97; energy = damping copy, corr −0.46..−0.56,
weight 4) is exactly the mechanism that digs the coupling-0 basin.

**4. Exploratory, stated as such (not pre-registered):** pooling both searching arms
(n=12, dJ −0.051 ± 0.137) against the control (+0.029): Welch t ≈ −1.75, p ≈ 0.10.
A *signal* that searching descends the criterion where not-searching does not — promote-
or-kill grade, never a finding, and it is the thing a criterion-first retry should power.

**→ CONSEQUENCES.**
- **Per the registered rule: no module change.** `sigma_min` stays 0.08; the
  raise-to-0.2 lever was contingent on the success branch. σ=0.20's faster descent and
  sole re-entry is recorded as its re-use context, to be retried *after* the criterion fix.
- **The next lever is the CRITERION, not the loop.** Rebalance or repair the two
  motion-magnitude proxies (legal travel term via leg-FK+IMU; energy re-pointed at
  `joint_load` or de-weighted) before spending another tick on search mechanics. The
  displacement test should then be re-armed with the same protocol — control included.
- **Harness traps, recorded:** (a) a σ=0 "control" does not publish — its body runs the
  config's own params, and `ge_vec` will happily report a vector the body never had;
  (b) `ge_gen` never increments at σ=0, so any generation-indexed analyzer silently drops
  the control arm (this bit `gainevo_tsweep.py` the same day it was fixed for (a));
  (c) `noise_min_n` was silently ignored at construction (missing from `on_setup`'s parse
  list) — caught by the de-vacuized margin test, inert in practice because every config
  set the default. All three fixed 2026-08-24.

### ★★ 2026-08-25 — THE PHASE DEMO, REHEARSED — and non-composition measured from a SECOND displaced start

**Verdict: demo config shipped (`__gainevo_demo`), rehearsed before its blurb was allowed
to promise anything.** Arena, window 6000 (observation-cadence compromise; DEMO SCAFFOLD,
never evidence), σ pinned 0.2, 3-D vector, displaced start written into BOTH the evolver
seed and the MotorEPM params. The inspector's gain rack now draws the landscape's measured
GOOD bands, so "did it converge on the ideal" is answerable by eye.

**1. ★ Rev 1 (amp 0.70 + postural 0.30 displaced jointly, n=3): the joint descent ESCAPES
the band corner 2/3.** J fell in all three seeds (−0.20..−0.37) — but via **tilt_sd**
(−0.24..−0.32), not the band route: two seeds parked postural at 0.10 (its single-gain BAD
point) with amp at/near the ceiling and coupling risen to 2.0, and scored well anyway.
From a *second* displaced start, with a *different* term paying (tilt here, energy in the
coupling case), the criterion's joint landscape again fails to match the single-gain
bands. Non-composition is generic, not a coupling quirk — PART V's premise, strengthened
at n=3.

**2. Rev 2 (amp-only displacement, n=4): 3/4 enter the band by generation ~3–6, 2/4 hold
through 300k.** Seed map recorded in the launcher blurb: seeds 2/3 clean, seed 1 finds a
stable high-amp posture instead (coupling ~1.96, tilt improving, amp pinned at the
ceiling — the lesson live), seed 4 converges then wanders out late (bands are flat;
nothing pins a point inside one, by design of the criterion's resolution).

**3. ⚠ Estimator note: last-third-minus-first-third dJ is BLIND to the demo's own
recovery.** Amp recovers by window ~6–12 of 49, i.e. *inside* the first third, so the
whole-run dJ carries late flat-band drift, not the descent (signs were mixed across
converged seeds). Fine for the tsweep (its recovery target was ~24 generations out);
wrong lens for fast transients. Watch windows 0–15, or the inspector, for the event.

Rehearsal logs: `demo_rehearsal*/` in the session scratch (regenerable: the demo config +
seeds 1–4). The `gainevo_live` launcher entry rotated out of the dropdown (file stays):
σ 0.08 is now measured to show ~nothing over an observation session.

**Promotion (2026-08-25): the operator's eye — the standing gate — passed.** A long live
run on `__gainevo_demo` in the arena showed the expected parameter behavior in the
inspector (the rack's band overlays doing their job). The demo ships as the phase's
observation instrument, and PART IV closes with the phase-boundary PR to `picrawler-dev`;
PART V (`stride_odometry_and_criterion_repair_plan.md`) starts in fresh context.

### ★★★ 2026-08-25 — GATE A: stance-FK odometry CARRIES travel — through a servo forward model, and only through one

**Verdict: `PARTIAL`, promoted to stage B.** The pre-registered gate
(`stride_odometry_and_criterion_repair_plan.md` §3-A) asked two things of the efference-copy
half of `stride_v`: r ≥ ~0.8 against true forward velocity, and a *structured* slip
residual. Measured (27 runs, n=6/arm fixed seeds + n=3 stepping-in-place, 12k ticks,
j1s4 at difficulty 0/0.3/0.6 + v3base at 0.3): **r = 0.74–0.79 at the criterion timescale
(1 s windows) under the best simple stance rule — a hair under the bar — and the residual
is cleanly gait-phase-locked (6–10× a circular-shift null, seed-consistent).** The fail
branch the gate was built to catch — unstructured slip, no signal — did not happen; every
loss mechanism found is the thing the stage-B IMU fusion and slip percept are *for*.
Signal-grade power (§3.3): promote-or-kill only, and it promotes. Sweep + analyzer:
`scripts_tools/strido_gateA.py`; logs `~/xaq_runs/stridoA_20260825/`.

**★ 1. THE NAIVE EFFERENCE COPY IS DEAD ON THIS BODY.** Raw commanded-angle FK velocity is
near-uncorrelated with truth (r_tick ≈ 0.03; r_w50 = −0.08..+0.07 across arms). The servos
low-pass the command too hard for the command to *be* the trajectory (the 22 mm foot-height
tracking error, now measured on the axis that matters). **A first-order forward model of
the servo rescues it**: LP(effective target) matches measured angles at r 0.93–0.99 per
joint (α ≈ 0.2 at 50 Hz ≈ 90 ms; knees slightly faster than hips), and FK on the filtered
commands reaches r_w50 0.74–0.79 — within 0.05 of the measured-angle ceiling. Efference
copy works *only through a plant model* — biologically unembarrassing, and sim2real-clean:
α is discoverable inside the blanket by fitting the FK/IMU disagreement.

**★ 2. THE STANCE RULE IS A MEASURED PARAMETER: fload ≥ ~0.2, a plateau.** G2's ghost-touch
threshold (0.05) costs ~0.15 of r (0.58–0.69); the optimum plateaus over 0.15–0.30 at
0.74–0.79. Hysteresis (0.2/0.1) and edge-trimming both score *below* the plain rule —
r 0.67–0.74 — so the touchdown/liftoff transients are apparently carrying signal, not just
noise. The published `foot_load` EMA lags liftoff by ~1–2 ticks (α 0.15 at 240 Hz), which
is exactly when a swing-forward foot pollutes the mean; the higher threshold trims that
lag. Guard and sensor want *different* thresholds — G2 keeps 0.05.

**★ 3. THE MEDIAN-OF-LEGS CONSENSUS IS REFUTED IN THIS CONTEXT** (the 2026-08-24 entry's
★2, built and scored offline): r_w50 = 0.29–0.36, slope 0.14–0.19. These gaits simply do
not keep 3+ feet loaded (stance count mode 3 with heavy 2s), so the median regularly
includes swing legs. *Re-use context: a true crawl with duty factor ≥ 0.75, or as the
fallback estimator if load sensing dies on hardware.*

**★ 4. THE WITHIN-STANCE DRIFT DECIDER (2026-08-24 ★4) RETURNED "LARGE" — and the verdict
it licenses is about the RAW estimator, which is indeed dead.** Forward-model-vs-measured
FK drift per stance episode: mean ~10 mm, p90 ~22 mm on a ~26 mm stance sweep. Not small.
The estimator survives at the criterion timescale because the drift is near-zero-mean
across episodes and legs; what does NOT survive it is the unmodelled command (★1), exactly
as the registered rule predicted.

**★ 5. SCALE: FK travel integrates to a STABLE 73–78% of true travel** — flat across
difficulty 0→0.6 and both gaits. A calibratable constant, not a terrain-varying bias; the
missing quarter is candidate rolling-contact displacement (a capsule foot's contact point
migrates without any joint moving) plus residual slip — i.e. the slip percept's raw
material. Regression slope at W=50 is lower (~0.42–0.46): the estimator under-tracks
*fast* velocity swings specifically, which is the band the stage-B accelerometer term is
complementary in.

**★ 6. BLIND-METRIC IMMUNITY, DEMONSTRATED NOT ASSUMED.** The `j1s4_c0` arm (coupling → 0
in both MotorEPM and the frozen evolver seed = PART IV's shuffling basin as a config)
steps at the same amplitude as the walkers (h1 sd 0.26 vs 0.26–0.32) while FK odometry
reads 0.73 ± 0.36 m of travel vs the walkers' ~5.4 m — a 7× separation on the axis where
the flow term's amplitude proxy reads *equal*. This is the property the criterion repair
buys. Bonus consistency check: slip structure weakens when barely translating (phase-lock
0.008 vs 0.013–0.024 walking) — the residual is translation-linked re-afference mismatch,
not sensor noise.

**Stage-B design, as measured:** publish `stride_v` = stance-mean forward FK velocity on
LP(effective targets), stance = `foot_load ≥ 0.2` both-ends-of-tick, gyro ω×p rotation
removal (tick-mean gyro); fuse complementary with the IMU per the plan; publish the
FK/IMU disagreement as `slip`. Instrument stays: `sv_*`/`svl_*` trace fields, the
stdout `strido_*` EMAs, and the HUD fk-vs-true line (operator-requested).

### ★★ 2026-08-25 — STAGE B SHIPPED: `stride_v` ⊕ `slip` on the bus, and the fusion's β was decided by the criterion's own timescale

**Verdict: `WORKING` (default-no-consumer; behavior-identity MEASURED).**
`reality.proprio.stride_v` (`[v_right, v_forward]`, vel_ego-shaped for the one-swap
insertion; lateral flagged UNVALIDATED) and `reality.proprio.slip` now publish every tick
from `picrawler_body.gd`. Constants as gate A measured them: stance `foot_load ≥ 0.2`,
servo forward model α 0.2, tick-mean-gyro rotation removal. Uncalibrated by design (the
~0.75 scale is the consumer's to adapt — prohibition 5).

**★ 1. THE NAIVE COMPLEMENTARY FILTER CRUSHED THE SIGNAL, AND THE FIX IS A PI BIAS
ESTIMATOR.** P-only fusion (β 0.1) collapsed the mean — integrated travel 0.65 of 3.57 m
— because the accelerometer's attitude-leak bias enters as `b·TAU/β` ≈ the whole signal.
Letting the FK innovation *teach* the bias (`ki` 0.1) restores it to ~0.85 of truth,
better than FK-alone's 0.80. The offline filter re-simulation (`alin` trace field) is the
instrument that found this; one recorded run tunes every (β, ki) for free.

**★ 2. β = 1.0, BECAUSE THE CRITERION CONSUMES ~1 s EMAs.** Measured trade-off, walk/c0
validation runs: β 0.3 wins per-tick r (0.75/0.81 vs 0.57/0.55) but LOSES the criterion
band (r_w50 0.62/0.72); β 1.0 ties the best there (0.71/0.79) with the accel kept for
swing/flight coasting and the slip innovation. *Re-use context for β ≈ 0.3, ki ≈ 0.05: a
fast-band consumer (reflexes, footfall-scale prediction).* The published sensor matches
the offline simulation exactly — the in-body filter is faithful.

**★ 3. GATE B PASSED ON ALL THREE CHECKS.** Publisher meter in the stdout diag
(`stride_v`/`stride_vx`/`stride_slip`; consumer half arms when stage C subscribes);
stepping-in-place reads 3.7× less than walking at equal step amplitude on the validation
pair (7× on the gate-A n=9); and the no-consumer guard was measured, not argued —
**7900/7900 ticks byte-identical trajectory** vs the pre-sensor build on the same seed.

### ★★ 2026-08-25 — C1: THE TERM SWAP IS IN, and the min-form split already reads "travel-limited"

**Verdict: `WORKING` (plumbing verified end to end; the science waits for D1).**
`travel_topic` (ConstructionOnly) on the GainEvolver: `""` (default) keeps flow on
`imu values[2]` — **measured byte-identical, 5900/5900 ticks, same seed, vs the pre-C1
build**; set to `reality.proprio.stride_v` it feeds the flow term the legal estimate and
gates the imu write off. The two-sided meter closed the loop the same day it opened:
`ge_trx` counts exactly 1 message/tick and `ge_fin == stride_v` to the digit.

The `j1s4_c1` arm config turns on, unchanged, the two OFF-shipped PART IV pieces whose
recorded re-use context was exactly this moment: `flow_min_form = 1` (magnitude can no
longer be traded for stillness-bought predictability) and `flow_turn_k = 4` (the lawful
anti-circling factor). Short-window smoke: `flow = 0.775` with **fmag 0.225 < fpred
0.686** — the min-form split reads *travel is the binding constraint*, which is
precisely the pressure the PART IV criterion lacked. D1 (pre-registered in the plan doc
§3-D before its runs) decides whether that pressure flattens the coupling→0 basin.

### ★★★ 2026-08-25 — D1: THE BASIN SURVIVES THE TRAVEL TERM — and the C2 budget says why, and what fixes it

**Verdict: D1 `NULL` on its registered rule (P1 held, P2 failed) — and the registered
fail-branch immediately produced the C2 lever.** 12 runs (2 arms × 6 seeds, 228k ticks,
coupling stepped 0→2 by SETPARAM_AT, σ=0 observer, behavior-identical bodies so the J
difference is the criterion alone). Logs `~/xaq_runs/stridoD1_20260825/`.

**★ 1. P1 HELD — the control replicates the wrong-way basin** (old criterion: J best at
coupling 0.0–0.4, band worse; zero wins 4/6 seeds), so the control is valid and C1 gets a
real verdict. **P2 FAILED — under C1 the zero basin still wins 4/6 seeds** (J(0.0) 2.443
vs band 2.458–2.472), *the same seeds* as the control.

**★ 2. THE FLOW REPAIR ITSELF WORKS.** C1's flow penalty gap at coupling 0 vs the band
widened ~50% over the old term (0.18 vs 0.12 of flow units) — travel now pays. It is
simply OUTVOTED: energy (w=4) rises 0.375→0.414 across the sweep, worth 4×0.039 ≈ 0.156
of J against the band, more than flow's improvement buys back. **A criterion repair is
two problems: the term (fixed by C1) and the BALANCE (C2's, exactly as the plan's
fail-branch registered).**

**★ 3. THE C2 BUDGET, measured on the D1 corpus** (variance share = cov(w·term, J)/var(J);
noise from same-level consecutive-window pairs — the gate-analyze estimator on a cleaner
corpus than gate 2 ever had): under C1, **energy carries 55% of the decision variance at
s/n 0.76** (and PART IV showed its content is a damping copy), **the repaired flow is the
cleanest term in the criterion (s/n 1.11) but holds only 8.7%**, tilt injects noise
(s/n 0.47, 35%), unloaded is negligible in variance (0.9%) but is a guard-flavored term.

**★ 4. THE RE-BALANCE, chosen by offline recomputation on the same 108 windows** (no new
runs — the windows carry every term):

| weights (flow, energy) | argmin | band beats zero | margin |
|---|---|---|---|
| c1 as-run (1, 4) | 0.4 | 2/6 | −0.022 |
| energy→1 (1, 1) | 0.8 | 4/6 | +0.046 |
| **flow→2, energy→1** | **0.8** | **5/6** | **+0.206, and coupling 0 becomes the WORST point on the landscape** |

**C2 ships `w_energy 4→1` + `w_flow 1→2`** — each justified by content (energy is a
damping copy; flow is the honest travel carrier with the best s/n), the pair by the
measured flip. This is an in-sample read of the D1 corpus; **D2 (displacement, tsw2
shape) is the out-of-sample behavioral test and the phase's headline gate.**

**★ 5. SLIP STAYS OUT OF J.** The candidate measured s/n 1.10 — term-grade — but its
level profile RISES with coupling (0.124 → 0.144): slip is travel-linked, so a positive
weight would re-create a move-less pressure. It ships as a percept, not a criterion
term. *Re-use context: normalized per unit travel (slip/|stride_v|), or as a guard
input.*

**★ 6. BUG (fixed): window-term attribution in the landscape readers was off by one.**
A window's ge_* terms appear on the FIRST diag line AFTER the `ge_wt` reset that closes
it; `coupling_authority.py` took the last line before, attributing each window the
previous window's terms. With 3 windows per level this dilutes level contrast but
preserves ordering, so PART IV's authority ORDERING reads survive; exact per-level
values shift. Found by asking the D1 data where the J-step lands relative to the
coupling-step (§3.2: measure the instrument before trusting it).

### ★★★ 2026-08-26 — GATE D: THE ZERO BASIN IS GONE — the search now climbs toward the band, and what remains is the flat band, not the criterion's direction

**Verdict: `PARTIAL` on the registered endpoint rule — and the phase's position sentence
is half-falsified in the direction the phase wanted.** D2 = the tsw2 protocol under the
C2 criterion (18 runs × 600k ticks, arena 0.3, `gainevo_make_arms.py d2` configs,
displaced start amp 0.385 / coupling 0.30 / postural 1.092 in BOTH evolver seed and
MotorEPM params; logs `~/xaq_runs/stridoD2_20260825/`, analyzer `gainevo_tsweep.py`).
Consumer meter: `ge_trx` = 600000 in all 12 searching runs — every scored window ran on
the legal travel signal.

**★ 1. THE REGISTERED RULE, EXACTLY:** control valid (σ0: coupling 0.30 in 6/6, dJ
+0.005 ± 0.029). fix020: **ΔJ −0.117, Welch t −2.9 vs control — the J half passes
decisively — and endpoint re-entry into 1.2–2.0 is 3/6: one run short of the registered
majority.** fix008: +0.019 dJ, 2/6 — σ 0.08 still drifts too slowly, exactly as PART IV's
drift arithmetic said (its σ-floor re-use context stands).

**★ 2. THE WRONG-WAY BASIN IS ELIMINATED.** Under the old criterion, 7/12 searching runs
descended coupling to ≈0 and stayed. Under C2: **0/12 endpoints at zero; the two runs
that visited the zero basin mid-run (s3 to 0.00, s4 to 0.00) both ESCAPED it and ended
IN the band** (1.17, 1.22). Escape from the old attractor is the single sharpest
behavioral signature a re-pointed criterion could produce — closer to a (d)-test event
than an endpoint statistic.

**★ 3. BY TRAJECTORY, 6/6 fix020 runs ENTERED the band at least once**; the endpoint
misses are late flat-band wander (s1 held ~1.04 for ~24 windows then dropped to 0.41 at
run end; s6 reached 1.68 then wandered to 0.76). This is the ALREADY-LEDGERED flat-band
property ("bands are flat; nothing pins a point inside one — by design of the
criterion's resolution"), now the binding residual — a criterion-RESOLUTION question,
explicitly out of this phase's scope (charter §6), not a direction question. The
endpoint rule was registered before the data and its 3/6 stands as recorded; the
trajectory read is reported beside it, not instead of it.

**★ 4. NO PATHOLOGY SWAP-IN:** amp endpoints 0.15–0.52 (no ceiling-pinning), postural
0.60–1.50 (no 0.10 parking) — the demo's tilt-led escape pattern did not reappear under
the re-balance. Falls concentrated in the two zero-basin-visiting runs (40, 81), i.e.
exactly where coupling was low — consistent with coupling's catastrophe-avoidance role.

**The phase sentence, rewritten by measurement:** *the search follows its criterion, and
the criterion now points at locomotion; what it cannot yet do is PIN a point inside a
flat good band.* n=6/arm fixed seeds: signal-grade (§3.3) — promote-or-kill, and it
promotes the C2 criterion as the reference criterion for the next phase's work. The
deployed j1s4 stack is UNTOUCHED (σ=0; the criterion change affects only searching
configs). Band-pinning (resolution / anneal-on-realized-ΔJ / σ floor) is the successor
phase's natural charter, with σ=0.20's re-use context now doubly earned.

### ★★★ 2026-08-26 — STAGE E, THE TWO-BODY STUDY: the criterion reads the body; the search transmits the landscape it is given; and searching on the WRONG body's operating point is dangerous

**The operator-directed embodiment capstone** (same C2 criterion, same seed streams,
same coupling displacement; `cad` = the body as designed, every prior result; `measured`
= the body as built). Pre-registrations in the plan doc §3-E; tool `strido_twobody.py`;
logs `~/xaq_runs/twobodyE1_20260826/` and `twobodyE2_20260826/` (cad's search arm = the
D2 corpus reused — deterministic replay would be byte-identical, registered as such).

**★ 1. E1 — THE CRITERION DISTINGUISHES THE BODIES (the study's clean positive).**
Per-body coupling landscapes, arena, gate PASS at 4/6 levels beyond 2× window noise —
and the difference is SHAPE, not offset: cad has a sculpted basin descending to argmin
1.6 (relief 0.41); the measured body is FLAT across 0.4–1.2 (relief 0.11 ≈ noise) and
turns worse exactly at cad's optimum (+0.44, the largest gap). Same gains, same
evaluator, different mechanics ⇒ different preferred region. This is an embodiment
property, measured.

**★ 2. E2 — THE SEARCH TRANSMITS ITS BODY'S LANDSCAPE, in both directions.** On cad's
basin it climbs (D2: ΔJ −0.117, t −2.9, endpoints mean 1.23). On measured's flat
conditional landscape it DIFFUSES: ΔJ +0.056 (t −0.04 vs its own valid control — P1b
FAILS), endpoints bimodal [0.11, 0.24, 0.25, 0.93, 1.75, 2.00]. Registered reads: P1 ✓
(controls hold 0.30, 6/6); P2 partial (mean endpoint 0.88 < cad's 1.23 ✓ but no
concentration in the flat region — flatness concentrates nothing); P3 ✓ directionally
(one-sided, underpowered at t ≈ 0.8); **P4 ✗ as registered, and the metric itself is
the lesson**: excess-J is relief-scale-dependent, so a flat cross-landscape makes ANY
endpoint look good on it. A relief-normalized version favors own-landscape on the
measured side only; recorded as post-hoc, not evidence.

**★ 3. THE STRONG CLAIM — "each body settles into its own basin" — is NOT demonstrated
in this context**, and the mechanism is precise: at the cad-tuned operating point the
measured body HAS no coupling basin to settle into; a search cannot converge to a
feature that does not exist. E1 predicted E2's measured-body outcome before it ran.
*Re-use contexts for the vivid form:* (a) let each body settle its OWN operating point
first (native run from the j1s4 start), then displace both from their own points; (b)
an amplified third body; (c) the injured-body (ablation) variant, where the landscape
difference is guaranteed large.

**★ 4. ★ THE UNREGISTERED LOUD FINDING — VIABILITY DOES NOT TRANSFER.** Falls per 600k
ticks, medians: cad σ0 **2** / cad searching **17** / measured σ0 **42** / measured
searching **210** (peak 831, in a run that pinned amp at its ceiling — the known
pathology, reappearing on the new body). Two independent effects: the cad-tuned
operating point itself is ~20× less viable on the built body, and SEARCHING multiplies
falls ~5–8× on both bodies — the guard set (falls out of J by design; guards tuned on
cad) does not contain exploration on the measured body. **Sim2real warning of record:
porting the evolver to hardware requires re-earning the viability guards on the
physical body BEFORE the search is allowed to move** — a fall on the bench is not an
auto-reset. Confound noted honestly: the measured body carries chassis collision boxes
(cad's chassis is a ghost), which may contribute to the falls RATIO; it cannot explain
the search-multiplier within either body.

**Net: the method's embodiment story stands on E1 + the climb/diffuse contrast; the
settling demo needs per-body operating points; and the port plan gains a hard
requirement it did not have yesterday.**

### ★★★ 2026-08-27 — E3: DISPLACE FROM YOUR OWN POINT AND THE SEARCH COMES HOME — on BOTH bodies, over-recovering, and the "home" is a region, not a point

**Verdict: every registered E3b prediction PASSES on both bodies** (protocol +
registrations in the plan doc §3-E3; logs `~/xaq_runs/twobodyE3a_20260826/` and
`twobodyE3b_20260826/`; 24 runs, all body/config receipts confirmed).

**★ 1. THE (d)-TEST, PER BODY.** Validity held (displacement costs J on both bodies:
cad 1.994 → 2.120, measured 2.198 → 2.331); displaced-σ0 controls held 0.30 in 6/6.
fix020 re-climbed coupling ≥ 0.6 in **6/6 (cad)** and **5/6 (measured)** with arm ΔJ
beating the control on both. **On the measured body — where the common-start E2 search
could not descend J at all — displacement from its OWN operating point recovers
decisively.** The E2 null was about the operating point, not the body or the method.

**★ 2. OVER-RECOVERY: recovery fractions −0.24 (cad) and −0.68 (measured)** — the
searchers ended BELOW the native benchmarks' J. 600k ticks of E3a settling had not
converged; the criterion still has downhill around both natives. Best finds (endpoint
vectors, J3 while still searching): cad [0.150, 1.250, 0.316] J3 1.837; measured
[0.150, 1.509, 1.078] J3 2.026 at **50 falls** — against the measured native's frozen
**849–940 falls/600k** (★4). Frozen n=3 validation of both candidates is running; the
benchmark-upgrade rule, registered before it: promote only if the candidate beats the
incumbent on BOTH last-third J and falls, n=3 frozen means.

**★ 3. "HOME" IS A REGION, NOT A POINT — reported as the descriptive read it was
registered as.** Recovered endpoints scatter within-body (cad coupling 0.63–2.00;
measured 0.74–2.00 plus one 0.16 miss) and do NOT return to the exact native vector —
measured's recovered postural sits high (1.03–1.50, 6/6) where its E3a native was
0.85. The criterion's per-body optimum is a broad region with multiple modes (cad's
second mode at coupling ~2.0 appeared in both E3a and E3b), and single settled points
are samples of it. The demo claim that survives: **displaced, each body's search
returns to ITS OWN viable region and recovers its own J level** — not "returns to the
same coordinates."

**★ 4. THE MEASURED NATIVE BENCHMARK IS FALL-HEAVY FROZEN: 849–940 falls/600k**
(the E3a selection could only choose among what its arm offered; its falls filter was
relative, not absolute). Its blurb carries this until the candidate validation lands.
cad's native is clean frozen (1–5 falls/600k, J3 1.994). The falls asymmetry between
bodies persists at every stage — the port-plan guard-re-earning requirement stands.

**★ 5. THE FROZEN VALIDATION (2026-08-27) SPLIT THE UPGRADE — and caught both
candidates over-promising.** Registered rule: promote only if better on BOTH frozen
J3 and falls, n=3. **cad: KEEP** — the challenger's J3 is better (1.90 vs 1.99) but it
falls 9× more (26 vs 3). **measured: PROMOTE** — [0.150, 1.509, 1.078] beats the
incumbent on both (J3 2.114 vs 2.198; falls 709 vs 879); `native_measured` now carries
it, provenance chained in its metadata. The method lesson, recorded for every future
selection: **a searching run's J and falls are trajectory-mixed** — the measured
candidate's 50-fall search run froze to ~710 falls at its endpoint (the low falls were
accrued at OTHER vectors along its path), and cad's 1.84-J search run froze to 1.90 at
9× the falls. Never promote a point from its searching-run statistics; freeze it first.
The measured body remains fall-heavy at every point yet measured (~1.2/1000 ticks) —
its benchmark says so on its face.

### ★★ 2026-08-28 — G2/G3 RUN, AND THE REACH GATE WAS MEASURING THE WRONG QUANTITY

**Verdict: measurement outcomes, not lever verdicts.** G2 `WORKING` (measured body reproduces
the operator's crouch), G3 `REGRESSION`-class on *limits* (2.8° short of a 180° fold), and the
"feet plant beyond reach" claim `TAUTOLOGY`-adjacent — it compared two different quantities.
Context: frozen native operating points (`native_measured` / `native_cad`), arena, chassis
colliding, n=3 × 6000 ticks. Port-doc Phase 1a table and geometry doc updated in place.

**★ 1. G2 PASSES ON THE BUILT BODY, AND ONLY ON IT.** `_report_reach_gates()` (new, print-only,
in every geometry receipt) evaluates the operator's crouch — femur straight up, 180° fold —
through `_fk_leg`: measured **toe −28.5 mm rel hip2, belly clearance 9.5 mm** against the
measured −28 / 9; cad −21.9 / 7.9. The limit box clips the pose (needs hip2 −90° vs ±80°,
knee +100° vs +97°) but the admissible corner still gives −28.6 mm — the shortfalls cancel.

**★ 2. G3 FAILS BY 2.8°, AS A LIMIT, AND CANNOT BE TESTED AS A COLLISION.** Max fold under is
177.2° at `KNEE_LIMIT_HIGH` +1.70 rad; 180° needs +1.745. Leg segments mask `_LAYER_WORLD`
only, so leg-on-leg interpenetration is not modelled anywhere — G3 is geometric by
construction. Sim also admits 63° of fold *above* the femur line, which hardware presumably
lacks; Phase 2's hard-stop sweep is where both get settled.

**★ 3. `foot_r` IS CHASSIS-CENTRE → TIBIA MIDPOINT.** `arenaavg.py` reads `foot_xz`, the lower
capsule's origin in chassis frame; reach is hip1 → toe. "170 mm vs 166 mm reach" therefore
never tested reach (the sim cannot exceed it). `scripts_tools/reach_check.py` computes the
gate's own quantity, `ext = |hip1→toe| / reach`, from the achieved hinge angles with a planar
model validated to 0.1 mm against the FK spot table on both bodies:

| body | ext mean | p95 | planted > 0.95 | fold | tibia off vertical | chassis y | legacy foot_r |
|---|---|---|---|---|---|---|---|
| measured | 0.935 ± 0.006 | 0.993 | 52 % | 40.8° | 47.6° | 60.7 mm | 157.5 mm |
| cad | 0.955 ± 0.000 | 0.983 | 68 % | 37.2° | 53.5° | 54.2 mm | 169.4 mm |

The straight-legged finding **stands** — restated as "plants at 93–96 % of reach, half to two
thirds of stance past 95 %, the knee at its singularity where its torque has no radial
authority." Proposed hardware gate: p95 `ext` ≤ 0.90.

**★ 4. HINGE ZERO IS THE CONSTRUCTION POSE, NOT A STRAIGHT LEG.** Every segment is built with
an identity basis and `_relative_angle_world_axis` reads rotation from that pose, so knee 0 =
tibia already dropped 80°; negative knee straightens, positive folds under; negative hip2
raises the femur. Two consequences: `arenaavg`'s `tib_off = |hip2 + knee + π/2|` assumes
0 = straight and **under-reads the sprawl by ~30°** (kept unchanged for continuity, annotated
in place); and **`KNEE_REST = −1.6` is a nearly straight leg** (fold −12°, toe at hip height)
— the brain's u = 0 posture is the sprawl attractor, and every standing posture is a held
offset from it. To verify in the `[C]` calibration panel before anything is built on it.
Re-use context: the sprawl family (Phase 3) should start from this, not from a bias lever.

**★ 5. SILENT CONFOUND (§3.2 #7), CAUGHT BY THE RECEIPT.** `metadata.body` is honoured only from
the UI launcher; a headless run of `native_measured` loaded **`cad`** until
`OGMA_PICRAWLER_BODY=measured` was set. `strido_twobody.py` already passes it per arm and its
logs carry the receipt, so the E-stage results are unaffected — but any ad-hoc headless
"measured" number without a receipt line saying `'measured'` is a cad number.
