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

### ★★ 2026-08-28 — HARDWARE BRING-UP DAY 1: the HAT answers, a knee moves, and cross-arch FP is not free

**Verdict: `WORKING` (H0 + the servo driver layer), on the STOCK robot — no new sensors yet.**
Full log in the port doc's Phase 4 "Bring-up log"; this entry keeps what generalises.

**★ 1. THE PROTOCOL TRANSFERS, THE LIBRARY DOES NOT — confirmed by measurement.** SunFounder's
`robot_hat` 2.5.5 source yielded a five-line wire protocol; raw `smbus2` then the C++
`ogma_hw` driver read **Vbat 7.65 V** and moved **P0 = the rear-left knee** without the
library present. The servo frame rate the HAT actually produces is **49.95 Hz** (PSC 352 ×
ARR 4095 at 72 MHz), not 50.00 — the 50 Hz brain tick and the servo frame are *different
clocks* and the host must not assume they coincide.

**★ 2. FIRST NATIVE BUILD FOUND A PORTABILITY DEFECT, NOT A LOGIC ONE.** `cpp_core` had never
been built without X11 headers; one `USE_X11` gate (`d30ffdd`) and 657/658 tests pass on
aarch64.

**★ 3. SAME CODE ≠ SAME BITS ACROSS ARCHITECTURES.** One stochastic 4000-step nav test flips
hemisphere on the Pi with an identical RNG stream. FMA contraction (default on aarch64, absent
on baseline x86-64) and per-arch `libm` are the candidates; **`-ffp-contract=off` measured:
still fails, μ moves −2.74 → +2.94** — contraction matters but `libm` differences remain, so no
flag restores bit-parity. The test itself asserts a hemisphere from ONE 4000-step seed and is
seed-fragile; make it seed-robust rather than chase bits. Rule for the port: **parity claims are about attributability of derivations, not
bit-identity of trajectories** — golden replays are per-architecture unless the build pins
FP contraction on both sides. Re-use context: any cross-machine replay or A/B that assumes
bit-identity.

**★ 4. Two Trixie-specific traps recorded for the next Pi:** the I²C overlay does not create
`/dev/i2c-1` without `i2c-dev` in `/etc/modules`; the Imager no longer grants NOPASSWD sudo to
a custom user.

### ★★★ 2026-08-28 — THE DEPLOYED CONFIG HAS NEVER RUN ON ITS LEGAL INPUTS

**Verdict: audit finding (`WORKING` as a diagnosis), from scoping the C++ derivation port.**
Of the twelve `reality.proprio.*` topics `native_measured` consumes, **five are oracle-fed in
sim**: `imu` = `[sin yaw, cos yaw, fwd_v, ang_v]` from world attitude and world velocity
(entirely god's-eye despite its name; consumed by MotorEPMv2 AND GainEvolver); `upright` and
`tilt` publish the exact basis, not the attitude filter the file already runs; `joints` are
achieved hinge angles hobby servos cannot report; `distress` integrates world position and
exact tilt (and has a units bug — window ÷ physics rate — that saturates it at 0.04 m instead
of 0.192 m); `target_compass` is a god's-eye bearing. `joint_torque` (weight 4.0) has no
hardware analog at all. Only `gyro`, `foot_contact/load`, `ground_clearance`, `stride_v`,
`feet_y_gravity_cmd_imu` are hardware-shaped as published. **Every legal substitute already
exists in the file** (`_ego_heading`, `stride_v`, `gyro[1]`, `_up_est_body`, the `_strido_lp`
servo forward model, `vision_compass`), so this is a config-and-publisher swap with a §5.4
honesty A/B — but it must happen BEFORE any hardware result is attributed to the gait. Re-use
context: every sim result on this config is a result on oracle inputs until that A/B runs.
Scope and order recorded in the port doc (§H3).

### ★★ 2026-08-29 — THE ROBOT HAT V4 CANNOT LIMP A SERVO: the safe state is a pose

**Verdict: hardware fact (`WORKING` as a diagnosis); a SPEC §4.1 assumption falsified.** Bench
sequence with the operator's hand on the rear-left knee (P0): armed at 1500/1800 µs it holds;
then pulse count **0, 1, ARR** (8 s each) — holds; timer period **0** and **1**, prescaler
**65535** — holds; **MCU held in reset for 5 s, then 30 s** — holds, and re-arms afterwards. The
V4's servo 5 V is the same 5 V/3 A DC-DC that powers the Pi (SunFounder: "provides a stable
power source for the Raspberry Pi and other devices"), so there is no rail to cut. SunFounder's
own robots never go limp. **Consequence:** `limp`, the bench deadman and the low-battery
auto-safe all command a saved **`rescue` pose** instead; "ragdoll" in the sim has no
hardware counterpart, and the (d) test's "drop a sensor"-style perturbations must be designed
around poses, not power. Re-use context: a separate servo BEC with its own switch would bring
limp back (a rebuild the wiring doc declined; the reason is now measured, not assumed).

---

### 2026-08-29 — `diag_lite()` on eight more modules: diagnostic surface, structurally gain-0

**Verdict: not a lever, and no verdict applies.** Recorded because it touches `cpp_core`,
which is the bar for appearing here at all — not because anything about the brain changed.

`xaq_voice` could only sonify what the brain publishes at rate, and that was **nine
scalars**: eight from `EPM` and exactly one (`motor_tle`) from `MotorEPMv2`. Everything
else lives in `diag_snapshot()`, which for an EPM is the whole GNG — ~50 KB serialised **on
the tick thread**, per frame, per subscription — and is therefore closed to a 60 Hz
subscriber. `diag_lite()` overrides were added to `MotorEPMv2` (1 → 16 keys),
`NeurochemState`, `LateralVoter`, `HomeostaticDrive`, `GainEvolver`, `GradientEPM`,
`SequenceGNG` and `MotorEPM` v1. About **60 signals** now reach a subscriber on the
picrawler's corridor config.

**★ 1. THE GAIN-0 GUARD IS STRUCTURAL, NOT A FLAG.** `diag_lite()` is called only by
`DiagPublisher`, only when a subscription's topic is `"lite"`, and nothing in the brain
reads it. With no `lite` subscriber the build is byte-identical — there is no knob to set
to zero because there is no path into the tick. Worth naming as a shape: a diagnostic that
is *unreachable from the control loop by construction* needs no A/B, and claiming one would
be theatre.

**★ 2. A TYPE FILTER SELECTED THREE MODULES THAT PUBLISHED NOTHING.** `xaq_voice` chose
voices by matching `"EPM"` in the type name plus `SequenceGNG`. `GradientEPM`, `MotorEPM`
v1 and `SequenceGNG` all matched and all returned `{}` — a **silent oscillator with no
diagnostic**, which is strictly worse than an error because nothing reports it. This is the
same failure the ledger records under "dead code" (§3.2 rule 2) wearing different clothes:
the consumer was live, the producer was empty, and the only symptom was absence. The tool
now names any subscribed module that publishes nothing, and every type its filter can
select publishes something.

**★ 3. AN ALPHABETICAL CONTAINER SILENTLY REORDERED A TUNED THING.** The voice registry
stored modules in a `std::map` keyed by id, so the octave ladder was handed out by *name*
rather than by the brain's `list_modules` order. `motor_epm` is documented as the lowest
voice — the anchor the ear separates the mix against — and alphabetically it is fourth, so
it landed on C6 and reached C8 on a spike. Nothing failed; it just sounded wrong, and only
against a live sim. **A container's ordering is a decision even when it is not written down
as one.**

**★ 4. THE TEST CAUGHT TWO DEFECTS BEFORE THE EAR DID.** `test_diag_lite.cpp` pins flat
scalars, names the unbounded containers and asserts them absent, and checks the base class
still opts out with `{}` rather than falling back the way `diag_snapshot()` does. It
immediately caught `GainEvolver::diag_lite()` shipping the gain **vectors** (returning
`metrics()` wholesale), which is exactly the unbounded payload the topic exists to avoid —
now filtered to scalars rather than key-listed, so a scalar added to `metrics()` reaches a
subscriber for free and an array cannot make the payload unbounded.

Also fixed en route: `MotorEPMv2::gait_coherence()` bounded its loop by `n_legs_` while
indexing `legs_`. The two agree after `on_setup()`, but `diag_lite()` is reachable from the
diag thread before then, where `n_legs_` carries its config default and `legs_` is empty.
Latent out-of-bounds, never triggered by the existing callers.

Re-use context: wanting a new signal sonified is a one-line addition to that module's
`diag_lite()`, and it must stay O(1) and additive. That is the only place `tools/xaq_voice`
may touch brain code.

### ★★ 2026-08-29/30 — A POSE RECALL BROWNED THE PI OUT: the shared 5 V rail, measured the hard way

**Verdict: hardware fact + fix (`WORKING`).** Recalling the `rescue` pose from the X pose —
eight servos starting together toward their 500 µs extremes at 2000 µs/s — took the Pi down
within 0.5 s (telemetry stopped, the daemon's record ends mid-line at Vbat 8.08 V, the HAT
stayed latched off until power-cycled). **A4 battery voltage never saw it**: the sag is on the
5 V/3 A DC-DC output that the Pi and all twelve servos share (see the previous entry), and
10 Hz telemetry cannot resolve a 50 ms dip. Reproduced on demand. **Fix:** `pose.set` and the
rescue path start channels one every 100 ms, shortest travel first, at 600 µs/s; the same
X → rescue recall then lands in 4.2 s with the throttle flags clean. Two instrument notes:
`vcgencmd pmic_read_adc` takes ~0.7 s per read, useless for rail transients (the INA219 is the
instrument) — ⚠ **corrected 2026-09-13: that is the cost of reading the WHOLE ADC set. Naming
one value, `pmic_read_adc EXT5V_V`, costs 85 ms median (8.2 Hz), and it is the only direct
view of the 5 V rail that actually fails — see bom §3.8.8.4**; and a first arm after boot cannot slew (no known position) — the stagger alone
carried that case. Re-use context: the servo-BEC rebuild now has two measured reasons (no
limp, shared-rail brownout); the INA219 quantifies the inrush margin before deciding.

---

### 2026-09-05 — PER-NODE KALMAN GAIN ON THE BODY-POSE EPMs: THE SCHEDULE'S TAIL HELPS, ITS HEAD CHURNS THE VOCABULARY (dose response, n=6)

*Charter with the bench and the derivation:
[`../plans-and-designs/epm_kalman_lessons_plan.md`](../plans-and-designs/epm_kalman_lessons_plan.md)
(Kalman-lessons campaign, Stage 1). Lever: `gain_kind=kalman` on the `EPM` (`body_pose`,
`body_pose_t`), whose `winner_id` feeds `MotorPlanner`. Gain-0 guard: `gain_kind=linear`
default, byte-identical (120/120 bench seed files). Base: `…__j1s4.json`, corridor, 12 k
ticks, difficulty 0.3, n=6 paired `OGMA_SEED` 1–6. Arms made with `mkarm.py --module`.*

**What the lever is.** The GNG winner update `w += g(x−w)` has the form of the Kalman filter
for a constant; the legacy anneal `g = ε_b(1 − 0.9·visits/N)` is not its gain. Measured on the
synthetic bench (n=20): a baked prototype keeps 24 % of its weight on its birth point and
carries 2× the MSE of the mean of the samples it saw. Kalman mode gives each node `p`, a
prior-variance ratio: `K = min(cap, p/(p+1))`, `p *= 1−K`, i.e. exactly `1/(n+1)` at cap 1.
On the bench it is WORKING everywhere (seed weight 0.275 → 0.034; 3-cluster vocabulary 10.7 →
7.05 nodes at purity 1.0; with `kalman_q=0.01` a drifting mean is tracked within 6 % of the
Kalman optimum by 2 nodes instead of 19).

| Arm | falls (6 seeds) | tilt_sd | net_z | flat_v | t_flat | brt_err | whole-run winner ids | Verdict |
|---|---|---|---|---|---|---|---|---|
| base `linear` | 1 | 0.104 | 7.39 ± 1.35 | 0.047 | 2416 | 1.545 | 46 | `BASELINE` |
| `kalman`, cap 1.0 | **6** (one seed 4) | 0.279 | 6.44 ± 3.00 (one collapse) | 0.055 | 2127 | 1.557 | 53 | `REGRESSION` |
| `kalman`, cap 0.2 | **5** (one seed 4) | 0.339 | 6.54 (one collapse) | 0.050 | 2318 | 1.503 | 53 | `REGRESSION` |
| **`kalman`, cap 0.05** | **0** | 0.123 | 8.52 ± 0.94 | **0.057** (paired +0.010 ± 0.009, t 2.7) | **2000** (−416 ± 507) | **1.438** (−0.107 ± 0.084, t −3.3) | 43 | **`PARTIAL`** — promote to n≥20 + arena + operator's eye |

**§3.2 checks.** Not a tautology (the base has no `gain_kind`); live (`MotorPlanner` reads
the winner ids and every paired seed shows a different id set); healthy baseline (6/6
walkers, 1 fall); the arm loaded (per-seed id sets differ; the whole-run id count moves with
the cap). One seed reached the far wall in every arm (the harness's gym-boundary warning):
distance is under-reported there, which only strengthens cap 0.05's `net_z` trend.

**Diagnosis.** The extra winner ids under cap 1.0 / 0.2 are **turnover, not size**: distinct
ids per run-quarter match the base (21–27) while the whole-run count rises. A gain of 0.5 on
a node's first win is right for a static cluster and wrong for a gait: on a moving ring of
poses the young node chases the input, its region gets re-inserted, the vocabulary churns,
and the planner reading it loses the walk. Capped at 0.05 (never above the legacy starting
gain) the body keeps only the Kalman *tail* (`1/(n+1)` instead of decaying to `0.1·ε_b`),
which is where the estimate improves. `unstable` (ticks with < 3 feet planted) doubles under
cap 0.05, but a faster gait raises it by construction; falls and tilt are the honest reads.

**Re-use context.** Uncapped / 0.2: a *stationary* vocabulary (a Level-N EPM over settled
consensus, a place map) — retry there. Cap 0.05: the promotion run. `kalman_q > 0`: its own
lever, bench-loud, on a body only with the cap. Bench gap: no scenario expressed the churn;
a rotating-ring scenario (`S6`) goes in before Stage 2's creature A/Bs.

---

### ★★★ 2026-09-11 — THE SIM-HONESTY A/B: the joint oracle was COSTING us, and one input is invisible at 6 000 ticks

**Verdict: `WORKING` (signal) on `honest_joints`; `PARTIAL` (mild cost) on `honest_imu`;
`NULL` — behaviorally FREE, consumer verified — on `honest_upright`.** Port doc Phase 4
step (c). The switches ship gain-0 (`c09148f`),
byte-identical off on a 1 200-tick continuous gate and a 4 999-tick `instant_pause` gate
carrying 14/15 hard resets. Every arm prints a startup receipt naming its ON/off state,
and all four were confirmed loaded before any number was read.

**★ 1. THE 2026-08-28 CONSUMER MAP IS WRONG IN TWO PLACES.** Read against the config
rather than against the audit's own summary:

- **`tilt` is not in `native_measured` at all**, so the tilt half of the attitude
  substitution has no consumer here (`publish_tilt` also defaults FALSE headless). Inert.
- **`imu` reaches MotorEPMv2 ONLY.** GainEvolver subscribes, but `handle_imu` discards
  every value while `travel_topic` is non-empty — and the config sets it to
  `reality.proprio.stride_v`. **The flow term is therefore ALREADY legal**, which retires
  the largest single exposure the legality audit recorded (flow accounted for 105 % of the
  movement in J on `coupling_gain`'s landscape). Of `imu`'s four values MotorEPMv2 reads
  only `[2]` and `[3]`; `sin/cos yaw` have no consumer in this config.

**★★ 2. `honest_upright` IS INVISIBLE AT THE STANDARD PROTOCOL, and the reason
generalizes beyond this lever.** All 46 metrics identical to baseline — not approximately,
exactly. `upright`'s only consumer is GainEvolver, and GainEvolver's first generation was
**measured at tick 34 020** (`warmup_ticks` 10 000 + `eval_window_ticks` 12 000, then one
per ~24 000: 34 020 / 58 080 / 82 080 / 106 080 / 130 080). At 6 000 ticks `ge_tilt`,
`ge_energy` and `ge_unl` all read **exactly 0.0** — the module buffers and scores nothing.
**A 6 000-tick A/B cannot say anything about any GainEvolver-only input, and a `NULL`
recorded there would have been a false verdict** (§3.2 checks 2 and 5). Re-use context:
measure at >= 34 000 ticks, ideally >= 58 000 for two generations. ⚠ The same caveat
applies retroactively to any claim about a GainEvolver-mediated quantity measured on the
6 000-tick standard.

**★★★ 3. `honest_joints` IS THE LOUD ONE, AND IT BUYS RHYTHM.** Feeding the Bridge and
both body-pose EPMs the **servo forward model** instead of achieved hinge angles — the
only joint signal an encoder-less robot can ever have — produces a markedly more rhythmic,
more actively stepping gait. n=6 × 6 000, corridor, diff 0.3, Welch t:

| metric | base | honest_joints | t |
|---|---|---|---|
| `plv_w` | 0.133 | **0.200** | **+14.2** |
| `plv` | 0.072 | **0.152** | **+9.7** |
| `swing_bout` | 5.70 | 7.13 | +8.5 |
| `contact_duty` | 0.773 | 0.731 | −7.2 |
| `steps` | 74.2 | **119.3** | **+6.7** |
| `amp_min` | 0.626 | 0.506 | −6.3 |
| `hk_value` | 5.52 | 5.04 | −5.8 |
| `tle_spr` | 0.391 | 0.499 | +5.4 |
| `step_cv_real` | 0.836 | **0.673** | −5.3 |
| `td_plv` | 0.319 | 0.246 | −4.4 |
| `scrub` | 0.094 | 0.115 | +4.0 |
| `coh` | 0.489 | 0.536 | +3.6 |
| `fwd_v` | 0.039 | 0.054 | +2.4 |
| `tilt_sd` | 0.069 | 0.085 | +2.2 |
| `straight` | 0.786 | 0.743 | −2.1 |
| `net_z` | 6.01 ± 0.46 | **6.70 ± 0.62** | +2.0 |
| `falls` | 0 | 0 | — |

**Phase-locking doubles, stepping regularity improves by a third, and it covers ~11 % more
ground** — while `flat_v` is unmoved (0.079 → 0.076), the **tenth** lever to leave that
pinned (§5). It is not free: it walks 19 % further in path for 11 % more net displacement,
with more scrub and tilt. Livelier and less directed, not simply better.

**★ The self-model gets WORSE while the gait gets better** — `motor_tle` 0.229 → 0.263
(t = +24.3), `tle_spr` +5.4. Worth recording because the naive prediction is the opposite:
a lag-filtered input is *smoother*, so it should be easier to predict. The reason is in the
same table — the body is taking 61 % more steps. **The residual rose because the behaviour
got livelier, not because the signal got noisier**, a distinction `motor_tle` alone cannot
make and `steps`/`plv` beside it can. ⚠ This is exactly the shape that would read as a
REGRESSION if the self-model metric were judged on its own.

**Direction check against §5.6:** the second time the hardware-POORER signal has won here,
for the same structural reason — `feet_y_gravity_cmd` beat its achieved-pose twin because
load deflection is noise to the consumer. Both cases are *commanded-vs-achieved*, so the
honest reading is **removing servo deflection helps a consumer that wants intent**, not
anything broader about impoverished sensing.

**4. `honest_imu` costs little and destabilises slightly.** MotorEPMv2's `fwd_v_` becomes
`stride_v` forward (~75 % of true scale, one tick lagged, NOT rescaled — prohibition 5)
and its yaw rate becomes body-frame rather than world-vertical.

| metric | base | honest_imu | t |
|---|---|---|---|
| `unstable` | 0.101 | 0.165 | **+2.6** |
| `turns` | 0.001 ± 0.074 | −0.060 ± 0.021 | −1.8 |
| `brt_plv` | 0.081 | 0.039 | −1.7 |
| `net_z` | 6.01 ± 0.46 | 5.47 ± 0.61 | −1.6 |
| `motor_tle` | 0.2290 | 0.2285 | ~0 |
| `falls` | 0 | 0 | — |

**No significant distance cost at n=6** and the self-model residual is untouched — but
`unstable` rises 63 % and the turn bias becomes small, consistent and low-variance, the
expected signature of a dead-reckoned heading that drifts. The scale-free parts of
MotorEPMv2 absorbed the 25 % gain change by design (the `fwd_v` resonance divides by its
own running spread); `coord_fit_accum_` does not, and is where the residual cost most
plausibly sits.

**★★ 5. `honest_upright` RE-MEASURED AT 60 000 TICKS: behaviorally FREE, and the
consumer demonstrably fired.** n=4 × 60 000, both arms reaching **generation 2**
(`ge_gen` 0 → 1 → 2), which is the horizon §2 above says is required.

| | base | honest_upright |
|---|---|---|
| `ge_tilt` per seed | 0.0731 / 0.0263 / 0.0338 / 0.0408 | **0.0749 / 0.0290 / 0.0363 / 0.0431** |
| `ge_acc` (accepts) | 2 / 1 / 1 / 1 | 2 / 1 / 1 / 1 |
| `ge_vec` (evolved gains) | `[0.1794,1.9569,1.1386]` … | **bit-identical, all 4 seeds** |
| all 45 seedavg metrics | — | **identical** |

**The criterion sees a different world and takes the same path.** `ge_tilt` — the
`w_tilt_sd` term, i.e. sd(upright) — is **higher in every seed** (+2.6 / +10.2 / +7.6 /
+5.5 %, mean +5.4 %), which is exactly what §5.4 predicts: the fused estimate carries
accelerometer contamination the exact basis does not, so its variance is larger. But the
perturbation never flipped an accept/reject decision, so the evolved gains came out
bit-identical and behaviour with them.

**This is a NULL that survives §3.2 rather than one that fails it.** Not a tautology (the
values provably differ), not dead code (`ge_gen` = 2, `ge_acc` = 5 accepts across the four
seeds), and the consumer was verified by telemetry rather than assumed. **Read it as: the
attitude gap costs ~5 % of one criterion term and nothing at all of behaviour** — the
cheapest of the three substitutions, and the one safest to take to hardware.

⚠ **Scoped to the power, and the power here is a discrete-event count, not a seed count.**
Two generations gave only **5 accept decisions across 4 seeds**; a 5 % perturbation of one
of five weighted terms flipping none of them is weak evidence that it flips none *ever*.
Re-use context for re-testing: more generations, a larger `w_tilt_sd`, or terrain where
body acceleration — and therefore the accelerometer's contamination — is larger than a
flat corridor's.

**Scope and power.** n=6 × 6 000 fixed-seed is a **signal**, promote-or-kill only — not a
finding. `honest_joints`'s `net_z` at t = 2.0 is precisely the marginal case §3.3 says to
confirm rather than excavate; its *loud* claims are `plv_w`, `plv`, `steps` and
`step_cv_real`, all t > 5. **Not yet observed in the UI** (§3 rule 5) and nothing is
promoted.

---

### ★★★ 2026-09-13 — THE MEASURED BODY'S CHASSIS LURCH IS A RATCHET, AND `height_k` CANNOT REACH IT

> ⚠ **CORRECTED 2026-10-02 — the ratchet is real code, but on the measured body it was
> triggered by a sensor bug.** The belly ray the brain read here (`ground_clearance`
> with the boom off) subtracted `CHASSIS_Y/2` and read **25.5 mm short** on the measured body,
> so it reported "grounded" while the belly was ~25 mm up, and `height_ground_gain` ratcheted on
> that. With the ray fixed (n=6, arena, 6000 ticks): `height_k_eff` ends at **0.33** instead of
> 0.83, peak chassis height is **79 mm** instead of 132, and **falls go from 7 to 0**. The
> measured body does not "tip over more" once its belly sensor is right. See the
> 2026-10-02 entry at the end of this ledger.

**Verdict: diagnosis (`WORKING` as a diagnosis) + `PARTIAL` signal on the ablation.**
Operator observation: the measured body sits higher than cad and tips over more, and the
real robot has no roll cage. **The observation is right and the proposed lever is the
wrong one** — which is worth recording, because the lever has no authority over the thing
being observed (the §412 rule, applied before building rather than after).

**★ 1. THE COMMANDED HEIGHT IS NOT THE DIFFERENCE.** `standing_y` is 0.0820 (cad) vs
0.0823 (measured) — 0.3 mm. And the *settled* ride height is the same in both bodies
(0.054–0.064 m). What differs is a **transient**.

**★★ 2. THE MECHANISM IS A ONE-WAY RATCHET IN THE ADAPTIVE SETPOINT.** The height target
is `height_k_eff × chassis_h_max`, and `height_k_eff` is adapted by `height_ground_gain`:

```
if (grounded) height_k_eff += gain * (kHeightKMax - height_k_eff);        // fast, toward 0.95
else          height_k_eff -= gain * 0.05 * (height_k_eff - height_k);    // 20x slower
```

The asymmetry is deliberate and sound *when grounding is intermittent* — grounding is
strong evidence the target is too low, not-grounding is weak evidence it is too high. **On
the measured body the belly is on the floor at spawn** (`gc_raw` 0.000 at tick 60), so the
ratchet starts immediately and the 20× decay never recovers it. Measured, arena, 3 seeds:

| | `height_k_eff` | `height_bias` | `gc_raw` | **chassis_y PEAK** | settled |
|---|---|---|---|---|---|
| cad | **0.300 → 0.300** (never moves) | −0.500 | 33 mm | 0.080 | 0.054 |
| measured s7 | 0.561 → **0.917** | +0.075 | 2.9 mm | **0.118** | 0.054 |
| measured s13 | 0.553 → **0.891** | +0.890 | 13.0 mm | **0.122** | 0.064 |
| measured s21 | 0.565 → **0.909** | **+1.500 (railed)** | 5.3 mm | **0.148** | 0.056 |

**The body lurches to 2.2–2.7× its settled height.** `chassis_h_max` is itself set by that
lurch — and `picrawler_body.gd:11532` already suspected the shape: *"h_max above is the
worst: a monotonic max with no decay and no reset, and it sets the height setpoint."* The
loop closes on itself: a windup excursion records a high ceiling, the high ceiling sustains
the demand, the demand sustains the windup. cad never enters it because its belly never
grounds at spawn.

**★★★ 3. `height_k` HAS NO AUTHORITY OVER THIS.** The obvious fix — lower the target — cannot
work, because `height_k` is only the **floor** of the adapted fraction
(`clamp(height_k_eff, height_k, kHeightKMax)`). With `height_k_eff` ratcheted to 0.9,
lowering the floor from 0.30 changes nothing. Ten minutes of reading, and it saves the
campaign that §412 was written about.

**4. ABLATION — `height_ground_gain = 0` on the measured body**, arena, n=3 × 3000:

| | peak chassis_y | settled | `gc_raw` | tilt mean |
|---|---|---|---|---|
| base (gg 0.01) | 0.118 / 0.122 / 0.148 | 0.054–0.064 | 2.9–13.0 mm | 0.083–0.101 |
| ablated (gg 0) | **0.076 / 0.084 / 0.069** | 0.056–0.066 | **5.0–14.7 mm** | 0.079–0.101 |

**Peak height down 38–53 %, 3/3 seeds, no overlap — and it costs nothing measured**: ride
height unchanged, belly clearance the same or slightly *better*, tilt unchanged,
`height_bias` sits near neutral instead of railing.

⚠ **THIS IS NOT A PROMOTION, AND `height_ground_gain` IS NOT REFUTED.** It was measured
`WORKING` on the **cad-era** body (2026-08-27, corridor n=6, `%<10 mm` 11.4 → 5.3) and its
re-use context never covered a body whose belly grounds at spawn. Refuted *in a context*
(§3.1), and ablating it globally would give back what it bought on cad. The candidates
worth an A/B are (a) per-body gain, (b) **anti-windup**: grounding while `height_k_eff` is
already high is evidence the target is UNREACHABLE, not that it is too low — only ratchet
up if raising it actually improved clearance; (c) decay or reset `chassis_h_max`, which the
code comment already flags as a monotonic ratchet with no reset.

⚠ **THE OUTCOME WAS NOT MEASURED, ONLY THE MECHANISM.** **Zero falls and zero auto-resets
in every arm** at 3000 ticks — so the tipovers are the operator's UI observation, and the
link from "lower peak excursion" to "fewer tipovers" is **inference, not measurement**. A
promote-or-kill run needs an arm long enough to produce falls, and the honest complement
to peak height is the fall count itself.

### ★★★ 2026-09-13 — THE AS-BUILT BELLY ToF CANNOT SEE THE BELLY, and the anti-windup fixed the wrong thing

> ⚠ **RETRACTED 2026-10-02 — item 2 below is a harness artifact (`TAUTOLOGY`-class: it
> measured the instrument).** The "belly truth" ray it scored the boom against read **25.5 mm
> short** on the measured body (`CHASSIS_Y/2` where the belly is 26.0 mm below the origin, not
> 51.5). The "+25.4 to +25.9 mm over-report" is that offset, and the "missed 21 of 21
> groundings" were moments the belly was ~25 mm up. Against belly height from chassis geometry
> the boom reads within **~1 mm**, compensated or not. The boom was telling the truth, and that is
> why `height_k_eff` stopped climbing when it was switched on. Item 1's `NULL` on
> `height_windup_guard` stands as measured, but against a baseline driven by the same bug, so
> it is a `NULL`-against-a-broken-baseline. Re-use context: re-test only if a ratchet recurs on a
> correctly-read belly.

**Verdicts: `NULL` on `height_windup_guard` (mechanism fired, target unmoved);
`WORKING` as a hardware finding on the boom ToF model.** Two levers, measured separately,
both shipping gain-0 and OFF. Follows the ratchet diagnosis above.

**1. `height_windup_guard` — the ratchet stops, and the lurch does not.** Gate: only let
`height_ground_gain` raise `height_k_eff` while `height_bias` is not already positive
(grounded-while-already-lifting is evidence the target is UNREACHABLE, not too low).
Arena, measured body, n=3 × 3000:

| arm | `height_k_eff` end | `height_bias` | **chassis_y peak** | tilt |
|---|---|---|---|---|
| base | 0.917 / 0.891 / 0.909 | +0.08 / +0.89 / +1.50 | **0.118 / 0.122 / 0.148** | 0.083–0.101 |
| guard v1 (hold) | **0.479 / 0.503 / 0.495** | +0.97 / +0.91 / +0.49 | 0.118 / 0.113 / 0.128 | 0.083–0.102 |
| guard v2 (decay) | **0.405 / 0.420 / 0.410** | +0.95 / +0.07 / +0.42 | 0.118 / 0.113 / 0.116 | 0.089 / **0.174** / **0.178** |
| `height_ground_gain=0` | 0.300 (flat) | −0.20 / +0.17 / −0.20 | **0.076 / 0.084 / 0.069** | 0.079–0.101 |

**The gate demonstrably fires** — the ratchet is halted in both forms — **and the chassis
lurch does not move.** v2 also nearly doubles tilt on 2/3 seeds. ★ **WHY: the damage is
done before the gate can act.** `height_k_eff` is already **0.561 at the first diag
(t = 60)**, ratcheted from 0.300 during the spawn second while the belly is flat on the
floor and `height_bias` is still negative — precisely the state the gate is designed to
*permit*. The runaway to 0.9 is cosmetic; the lurch is set by the first sixty ticks.
Re-use context: a fix must act on the SPAWN transient (initial pose, or an initial
setpoint that does not treat "lying on the floor at t=0" as evidence), not on the
steady-state ratchet.

**2. ★★★ THE BOOM ToF IS BLIND TO BELLY GROUNDING — a hardware finding, not a sim one.**
The as-built sensor is on a boom **70 mm aft** of centre at **+77 mm** (top of the HAT),
not under the belly. Modelled faithfully (ray along body-down from the boom tip, with the
lever arm and the tilt resolved out using the *fused* attitude estimate). Both readings
computed in the same run, boom vs a belly-centre truth ray:

| | boom reads | belly truth | over-report | corr | belly grounded | **boom missed** |
|---|---|---|---|---|---|---|
| boom+comp | 31.6 mm | 5.7 mm | **+25.9 mm** | +0.80 | 21/50 samples | **21 of 21 (100 %)** |
| boom raw | 32.1 mm | 7.0 mm | **+25.0 mm** | +0.78 | 20/50 samples | **20 of 20 (100 %)** |

**It tracks well (r ≈ 0.8) and is offset high by ~26 mm, and it missed every single
belly-grounding event.** ⚠ **And that is why it appeared to "fix" the ratchet**: with the
boom model on, `height_k_eff` stays at 0.32–0.35 instead of climbing to 0.92 and the peak
drops to 0.071–0.084 — **not because the loop got healthier but because the sensor stopped
reporting the grounding that drives it.** A channel that removes a pathology by not
observing it is the worst possible pass.

⚠ **Tilt compensation cannot fix this, and it is important to say why.** The correction
resolves the sensor's own height change out of the reading — but the boom is measuring a
*different patch of ground*, 70 mm behind the belly centre, and under pitch the belly's
LEADING edge is what strikes while the boom is furthest from it. The residual is
geometric, not trigonometric. Re-use context for the hardware: either the offset is
characterised and subtracted as calibration (it is stable — r ≈ 0.8), or belly contact
needs an observation that is actually at the belly.

**Both ship gain-0 and OFF**: `height_windup_guard` (MotorEPMv2 param) and
`tof_boom` / `tof_tilt_comp` (body exports, with a `tof[...]` startup receipt). Byte
identity re-verified after every step. `gc_belly` is published as a diagnostic whenever
the boom model is on, and is `-1` otherwise so "not modelled" cannot be read as zero.

⚠ **Still not measured: tipovers.** At difficulty 0.8 over 8000 ticks the arena produced
**3 auto-resets on one seed of three** — roughly one event per 8000 ticks. The outcome the
operator cares about is too rare here to A/B at reasonable cost, so every number above is
a MECHANISM measurement and the link to tipping remains inference.

**★★★ ADDENDUM 2026-09-13 — every number above was taken on a GHOST CHASSIS, and the
default is now solid.** `chassis_collides` defaulted **false**, `native_measured` declares
an empty `body_env`, and **`seedavg.py` does not set it either** — so for a body that
cannot touch the ground with its belly, "belly grounding" meant the belly
**interpenetrating the floor**. That is a different physics, not a conservative
approximation, and it sits underneath the one channel the promoted height homeostat rides.
Other configs carry a prose warning about exactly this; the benchmark did not.

**The boom-blindness result was re-measured with a solid belly and is UNCHANGED:**

| | boom | belly truth | over-report | r | missed |
|---|---|---|---|---|---|
| ghost chassis | 30.7 mm | 5.2 mm | +25.5 mm | +0.836 | 22/22 (100 %) |
| **solid chassis** | 30.7 mm | 5.2 mm | **+25.4 mm** | +0.790 | **23/23 (100 %)** |

So the finding stands on its own and did not depend on the ghost body.

**What the solid chassis does change** (arena, measured body, belly-centre sensor, n=3):
peak chassis height falls (0.090 / 0.127 / 0.107 against a ghost 0.148 — the belly can no
longer sink through the floor), and **tilt becomes erratic rather than worse**: 0.213 /
0.102 / 0.071 against a ghost 0.092, with the single auto-reset of the whole comparison on
the same seed that hit 0.213. n=3 and one event: a hint that the solid belly adds variance,
nothing more.

⚠ **CONSEQUENCE FOR THE RECORD:** figures taken before 2026-09-13 are not directly
comparable to figures after it unless the arm set `OGMA_PICRAWLER_CHASSIS_COLLIDE`
explicitly. Reproducing a historical number now requires setting it to **0** and saying so.
The startup receipt prints the state unconditionally — ON or off, default or override — so
no run is ambiguous about it again.

### ★★★ 2026-09-13 — THE SIM HAS BEEN WALKING AT A SERVO SPEED THE ROBOT'S CURRENT BUDGET FORBIDS

**Verdict: sim2real gap (`WORKING` as a diagnosis).** Asked to study slew limiting and
round-robin as brownout mitigations. **Round-robin needs no study — it is already
refuted on hardware**, and the slew question turned out to be a live gap rather than an
open design choice.

**★ 1. CONCURRENCY WAS NEVER THE BINDING CONSTRAINT.** BOM §3.6 swept K = 1…12 servos per
tick at slew 40: **all twelve at once is 2.19–2.57 A**, with **zero throttle events in 28
trials**, against a ≤ 3.5 A working budget. §3.8.2 states it outright: *"the duty budget is
a slew cap, not a concurrency cap; §3.6's concurrency sweep found no boundary because it
held the variable that matters fixed."* §3.6.1 adds that the deployed 100 ms stagger is
**defeated by its own gentleness** — travel time (1567 ms) exceeds the launch window
(1200 ms), so every channel ends up moving together anyway; a true K=3 would need a
~520 ms stagger. **A priority round-robin would be regulating a quantity that has already
been measured not to be the limit.**

**★★★ 2. THE SIM RUNS AT ~76 µs/tick EQUIVALENT, WHICH IS THE "AT THE EDGE" ROW.**
`MAX_SERVO_SPEED = 6.0 rad/s`; at the standard hobby-servo scale (500–2500 µs = 180°,
636.6 µs/rad) that is **76 µs/tick** at the 50 Hz tick. Against §3.8.2's measured budget:

| slew | peak battery A | verdict | rad/s |
|---|---|---|---|
| ≤ 50 µs/tick | ≤ 1.90 | ✅ the budget | 3.93 |
| **76 µs/tick** | — | **⚠ the sim sits here** | **6.00** |
| 80 µs/tick | 2.61 | ⚠ at the edge | 6.28 |
| ≥ 200 µs/tick | ~3.00 | ❌ over the 5 V rail's 3 A rating | 15.7 |

**The deployed `ServoDriver` default is 40 µs/tick = 3.14 rad/s — half the sim's speed.**

**★★ 3. THE SCALE WAS MEASURED ON THE BENCH, and the standard was 14 % wrong.**
`us_per_rad` = **545.2**, not the 636.6 the datasheet default implies — so the servo
sweeps more angle per microsecond than assumed. Method: command a known µs step, mark the
toe on paper, measure the swept chord against the known tibia radius `L3` = 76.5 mm
(`angle = 2·asin(chord / 2L3)`). Seven marks at 100 µs steps.

Three checks agree: the six-gap average gives **549.0** and the single 700→1300 chord gives
**545.2** (0.7 % apart, different error structure); chord/arc = 0.958 against a predicted
0.950, confirming the marks lie on a true 76.5 mm arc; and — the convincing one, because
nobody measured it for this — at 545 µs/rad the *already-calibrated* knee envelope
600–2300 µs spans **178.6°**, a hobby servo's full mechanical travel. At 636.6 it would
have been 153°, leaving 27° unexplained.

⚠ **A DEADMAN NEARLY CORRUPTED IT — and "limp" is the wrong word for what it does.**
`ServoDriver`'s own 25-tick watchdog writes pulse 0, **which this HAT ignores**: the V4
cannot de-energise a servo from software at all (measured 2026-08-29 — pulse 0/1/ARR
ignored, stopped timer ignored, MCU held in reset 30 s and the servo still powered). The
real safe action is one layer up: **benchd's deadman (`DEADMAN_MS` = 1000) commands the
saved `rescue` POSE**, and `benchd::limp_all()` is literally `rescue(why)`. Nothing goes
slack; the robot *moves to rescue*.

**That is what makes it dangerous to a measurement.** A rescue excursion of 100–200 µs is
a centimetre or two at the toe and reads as nothing happening — which is exactly why the
first marks looked fine. It only announced itself at 900 µs, where the excursion was
~31 mm and the operator saw the leg go and come back. Every mark used was re-taken with a
**5 Hz keepalive** holding the client fresh (benchd then re-commands every armed channel
each tick, so the driver watchdog never fires either). The `600→700` gap was dropped for
the same reason — it starts from a position the deadman had already pulled to rescue — and
it was the clear outlier at 449 µs/rad against 509–588.

★ **The arc check is what licenses trusting the rest.** If the femur had wandered between
marks the toe positions could not lie on a clean 76.5 mm arc, and they do (chord/arc 0.958
against a predicted 0.950). A geometric consistency check earned more here than any amount
of care at the bench could have.

**★★ 4. WHAT THE BUDGET COSTS THE GAIT** — corridor, measured body, n=3 × 6000, with the
sim's speed cap set to each *measured-scale* hardware slew:

| `MAX_SERVO_SPEED` | = slew | net_z | fwd_v | steps | tilt_sd | contact |
|---|---|---|---|---|---|---|
| 6.00 rad/s | 65 µs/tick (**sim today, over budget**) | **6.31 ± 0.66** | 0.110 | 34 | 0.0565 | 0.779 |
| **4.59 rad/s** | **50 µs/tick (budget ceiling, ≤ 1.90 A)** | **4.63 ± 0.19** | 0.083 | **35** | **0.0440** | 0.835 |
| 3.67 rad/s | 40 µs/tick (**deployed default**) | 3.69 ± 0.46 | 0.066 | **13** | 0.0523 | 0.855 |
| 1.83 rad/s | 20 µs/tick | 0.68 ± 0.18 | 0.028 | **0** | 0.1035 | 0.897 |

**★★★ THE BUDGET CEILING IS NOT A COMPROMISE — IT IS THE BEST-BEHAVED POINT MEASURED.**
At 50 µs/tick the gait keeps **all** its stepping (35 against the over-budget 34), runs at
the **lowest tilt_sd of the whole set** (0.0440), and has by far the **tightest seed
spread** (± 0.19 against ± 0.66). It costs 27 % of distance against a setting that is
outside the current budget anyway. The deployed 40 µs/tick is materially worse than 50 on
every axis: distance −20 %, stepping **13 against 35**, tilt worse, variance worse.

**RECOMMENDATION FOR FIRST POWER-ON: raise `ServoDriver::slew_us_per_tick` from 40 to 50.**
It is inside §3.8.2's measured ≤ 1.90 A budget, it is the most stable setting measured, and
it recovers the stepping that 40 loses. The safest available setting and the best-behaving
one are the same setting, which is not the trade-off this study expected to find.

⚠ **Slower is still not safer.** At 20 µs/tick the gait collapses — zero steps and tilt_sd
doubles. Below ~2 rad/s the body cannot keep up with its own postural demands.

⚠ **Scope.** n=3 is a signal, not a finding; falls were not captured; the scale is one
channel (RL knee), and part-to-part variation is unchecked. §3.8.2's current figures are
vinyl-measured and flagged optimistic — grip converts free motion into work, and the same
move cost 1.90 A on vinyl against 2.62 A on leather — so **re-verify 50 µs/tick on the
actual test surface before relying on it.**

**Shipped:** `MAX_SERVO_SPEED` is `const` → `var` with an `OGMA_PICRAWLER_MAX_SERVO_SPEED`
override and a startup receipt naming the µs/tick equivalent and the budget. Default
unchanged, gain-0 verified against a stashed baseline (1200 ticks × seeds 7/13, identical).

### ★★ 2026-09-12 — THE ROBOT AND THE SIM DID NOT AGREE, AND ONE FLAG FIXED IT

**Verdict: defect + fix (`WORKING`), measured on the robot.** Recorded because it touches
`cpp_core`, and because it **corrects a conclusion already in this project's record.**

The shared `ogma::body` helpers exist so sim and robot produce the same NUMBERS. They did
not. Built the three parity checkers on the Pi and replayed the x86-generated oracles:

| build | `leg_kinematics` | `imu_attitude` | `stride_odometry` |
|---|---|---|---|
| default | **400/400 mismatch** (worst 3.1e-07) | **796/800 mismatch** (worst 6.3e-07) | **900/900 mismatch** |
| `-ffp-contract=off` | 400/400 **exact** | 800/800 **exact** | 900/900 **exact** |

aarch64 GCC fuses `a*b+c` into FMA by default; baseline x86-64 has none and cannot.

**★ This corrects the 2026-08-30 reading that "no compiler flag buys cross-arch
bit-parity: golden replays are per-architecture."** That was measured on
`RunTumbleNavV2` — 4 000 stochastic steps of DOUBLE precision leaning on `libm` — where
per-arch `libm` genuinely does survive contraction being disabled. The body helpers are
float32 and their trig agreed across the pair the moment FMA was off, so **for
`ogma::body` contraction was the whole difference.** The earlier verdict stands for what
it measured; it was the generalization that was wrong. Ledger §3.1: a refutation is about
a mechanism IN A CONTEXT.

**★ The diagnosis came free from an instrument the checker already had.**
`imu_attitude_parity_check` counts `up_accel` — sqrt and divide, **no trig** —
separately: 14/800 mismatched against 796/800 overall. Arithmetic-only paths nearly
clean, trig-bearing ones not, which points at contraction rather than at the maths.
**Distinguishing "rounding" from "a bug" is the difference between a flag and a rewrite**,
and a bare pass/fail could not have done it.

**Why 3e-7 was not "small enough to ignore."** On metre-scale links that is 0.3 µm, far
below any physical error. But `stride_v` **accumulates** — `est`/`bias`/`slip` carry
forward every tick into MotorEPMv2 and GainEvolver — and an accumulating estimator does
not get to ignore 1 ULP per step.

**Safe on the sim, verified rather than assumed:** GCC emits no FMA on baseline x86-64,
so the flag is a no-op there; the 1200-tick byte-identity gate was re-run after adding it
to `cpp_core`, `pi_host` AND `godot_host` and stayed IDENTICAL on both seeds.

⚠ **Live consequence:** the robot's `Icm20948` already holds a shared
`ogma::body::ImuAttitude`, so until its build carries the flag it runs the same filter as
the sim and produces different bits. Re-use context: any future `ogma::body` helper is
covered by the flag, but a NEW cross-arch claim needs its own oracle replay — this one is
evidence about float32 arithmetic, not about `libm` in general.


### ★★★ 2026-10-02 — THE BELLY "TRUTH" RAY READ 25.5 mm SHORT ON THE MEASURED BODY: the lurch, the tipping and P-d's boom-blindness were one sensor bug

**Verdict: harness defect, fixed; it retracts the 2026-09-13 boom finding and re-reads the
ratchet entry.** Arena, difficulty 0.3, n=6 × 6000 ticks, measured body. The arms differ only
in the belly ray.

**The bug.** `_compute_ground_clearance_centre` subtracted `CHASSIS_Y/2 + 0.02` as the
sensor-to-belly offset. That is right only for a single box centred on the origin (cad). The
measured body's origin is not its centre: the belly is **26.0 mm** below it (`_chassis_bottom_local`),
not 51.5. Every reading was 25.5 mm short and floored at 0 while the belly was still ~25 mm up.
The boom function's own comment already said "NOT CHASSIS_Y/2"; the centre ray was never fixed
to match. **Found by checking `gc_belly` against chassis `y`**: it tracked the `CHASSIS_Y/2`
formula to ~3 mm, while the boom tracked `y − 26 mm` to ~1 mm.

**What it reached.** With the boom off, this ray IS the brain's `ground_clearance`, so the
promoted height homeostat on every measured-body config (`native_measured`, P-c honest joints)
saw a belly ~25 mm lower than it was. With the boom on, it was the `gc_belly` truth proxy.

| `native_measured`, measured body | legacy ray | **fixed ray** |
|---|---|---|
| falls | **1.17 ± 1.07** (7 in 6 seeds) | **0** |
| tilt_sd | 0.277 ± 0.26 | 0.063 ± 0.010 |
| straight | 0.60 ± 0.28 | 0.81 ± 0.06 |
| net_disp | 5.70 ± 2.59 | 6.86 ± 1.38 |
| `height_k_eff` at end | 0.83 | 0.33 |
| peak chassis y | 132 mm | 79 mm |
| steps / step_bal | 20 / 0.15 | 23 / 0.10 |

**Gates.** The fixed form equals the old one exactly on cad (`bottom = −CHASSIS_Y/2`): a
1500-tick cad run is identical in every diag field. On P-e (boom on) only `gc_belly` moves, by
exactly +25.5 mm. `OGMA_PICRAWLER_BELLY_RAY_LEGACY=1` restores the old ray and reproduces the
pre-fix `native_measured` run in every field. The body prints `belly_ray[...]` at startup.

**Re-read of earlier entries.** 2026-09-13's boom-blindness finding is retracted (banner
there). The ratchet mechanism is real, but on the measured body it was fed false "grounded"
readings (banner there). ⚠ **Every measured-body number taken with the boom off before this date
ran on the short ray**, and it was the brain's input, so those runs measured a different
closed loop. That includes the `native_measured` benchmark and P-c. Re-measure before building
on them.

**A second confound, in the UI only: `body_env` leaked between launcher selections.** The
launcher applies a config's `body_env` into the process when the config is *selected*, and
"a pre-existing env var wins". It counted its own leftovers as the operator's, so selecting
P-d and then P-e ran P-e with tilt compensation ON. Selecting `native_measured` after any boom
config ran it **with the boom ToF and solid chassis switched on**. The launcher auto-selects its
remembered config at startup, so this fired with no visible cause. Fixed: the launcher records
what it set, overwrites only that, and unsets keys the new config does not declare. A
command-line export still wins. Tested by driving the real selection handler
P-d → P-e → `native_measured` → P-e, on the old code (leaks in both directions) and the new.
Headless harnesses never applied `body_env` at all, so seed-averaged numbers are unaffected.
**Any UI observation of a boom/non-boom config made after selecting the other kind may have shown
the wrong arm.**

### ★★★ 2026-10-01/02 — THE FSR LEG, THE RAW BOOM, AND THE ROBOT'S SERVO SPEED: at its real speed the robot does not lift its feet

**Verdicts: FSR leg `WORKING` (signal); raw boom ties-or-beats the compensated one on flat
ground (signal); the robot's servo speed is a `REGRESSION` to a shuffle (signal, loud).**
Arena, difficulty 0.3, n=6 × 6000 ticks, `native_measured__tofboom` gains, same build and
seeds throughout. Body and env passed explicitly per arm, and every log carries the receipt.

**1. The FSR toe modules lengthen the lower leg to 87 mm (knee axis to toe tip, operator).**
`body/measured_fsr.json`: L3 76.5 → 87 mm, `standing_y` 82.3 → 92.7 mm (`coxa_z_drop + L3·sin 80°`),
+2 g per foot. `stand_m` is deliberately unchanged, because it must match the robot. Against the
76.5 mm leg (compensated boom): net_disp 6.37 ± 0.94 → 7.84 ± 1.63 (edge-censored), straight and
tilt tie, 0 falls in both, belly ~+5 mm higher. The gait survives the leg with no retuning.

**2. The robot publishes the UNCOMPENSATED boom** (`ogma_host`: raw − 64.8 mm; the compensated
arm exists only in benchd telemetry). On the FSR leg, uncompensated vs compensated: net_disp
8.61 ± 0.57 vs 7.84 ± 1.63, straight 0.87 vs 0.82, tilt_sd 0.066 vs 0.083, 0 falls. The raw boom
reads within −1.1 ± 1.0 mm of belly height from chassis geometry. On flat ground pitch does not
swamp it. Not a terrain result: on a slope the compensated arm is the wrong one (BOM §9.10.3).

**3. ★ At the robot's servo speed the gait stops lifting its feet.** The operator kept the robot
at its deployed 40 µs/tick, which is **3.668 rad/s at the MEASURED 545.2 µs/rad** (not the
3.14 that the 636.6 hobby-servo standard gives). The sim's 6.0 rad/s default had never been
matched to it.

| P-e (FSR leg, raw boom) | 6.0 rad/s (sim default) | **3.668 rad/s (robot)** |
|---|---|---|
| steps | 28.2 ± 9.9 | **0 on all 6 seeds** |
| net_disp | 8.61 ± 0.57 | 4.03 ± 0.32 |
| straight | 0.87 | 0.81 |
| tilt_sd | 0.066 | 0.052 |
| scrub | 0.098 | 0.058 |
| td_plv | 0.34 | 0.25 |
| feet_y p99 / max | 50 / 70 mm | **−4 / 24 mm** |
| falls | 0 | 0 |

**Checked before recording it** (§3.2): the arm loaded (`max_servo_speed = 3.668` receipt, body
`measured_fsr`, `tof[boom=ON tilt_comp=off]`). `steps` counts swing-detector lifts past a
fixed `feet_y` threshold, identical in both arms. At 3.668 the toes never reach it, because
99 % of samples sit below −4 mm. **The body still travels 4 m, so this is a shuffle: toes
dragged rather than lifted.** That is the degenerate behaviour a distance metric rewards
(CLAUDE.md §3.3). Same direction as 2026-09-13's sweep (13 steps vs 35 at this speed, on the
measured body with the short belly ray), and stronger.

**Why, most likely, and NOT verified:** these gains are the `native_measured` operating point,
which the E3b searcher found at 6.0 rad/s. A body+gains pair is a unit (stage E3), and slowing
the servos moves the body out from under the gains it was tuned with. Re-use contexts, any of
which reopens it:
- a native re-settle of the gains at 3.668 rad/s;
- the robot at 50 µs/tick (4.59 rad/s), the ledger's best-behaved speed at no measured
  current cost (BOM §3.8.4/§3.8.6);
- a run long enough for the GainEvolver to re-adapt (6000 ticks may be too short).

**Shipped:** P-e now declares `OGMA_PICRAWLER_MAX_SERVO_SPEED=3.668` in `body_env`, so it shows
the robot as it will actually run. The sim's global default stays 6.0 so historical configs
reproduce. The body's startup receipt and the bench dashboard now convert at the measured
545.2 µs/rad. At 636.6, the dashboard's mirror of the robot's pose read ~14 % small.

### ★★★ 2026-10-02 — WHY THE GAIT SHUFFLES AT THE ROBOT'S SERVO SPEED: nothing times the swing, four imposed terms jitter at full slew, and the "commanded" swing detector never sees a command

**Verdict: diagnosis (attribution, single seed; not a lever verdict) + two harness defects.**
P-e (FSR leg, raw boom), arena difficulty 0.3, seed 1, per-tick traces (`OGMA_PICRAWLER_TRACE`),
ticks 900–3000/4000. Every arm used identical instrument settings.

**1. Nothing sets swing length.** Swing (true contact) is about 6 ticks at both speeds, as
2026-10-01/02 found, but chatter (<4 ticks) is 20 % of lift-offs at 6.0 rad/s and 51 % at
3.668. Counting only real swings (≥4 ticks), the median is 7 vs 6 ticks. The difference is
the count: 323 vs 73 in ~3100 ticks × 4 legs. Front and rear legs behave the same, so the
rear legs' self-referential descent timer (`rear_land_gain`, half of an EMA of the leg's own
past swings) is not the main cause.

**2. The joint command reverses every 1–2 ticks, at full slew, at both speeds.** The knee's
slew-limited target moves at the `MAX_SERVO_SPEED·TAU` cap on 95–97 % of ticks, hip1 on
77–88 %. It reverses direction on 38–45 % (knee) and 53–55 % (hip2) of moving ticks, and the
median same-direction run is 1–2 ticks. The achieved joints reverse on 21–39 % of ticks. A
"swing" is a stretch where this jitter happens to drift one way long enough to unload a foot.
At 3.668 rad/s the same drift covers about half the distance (knee excursion within a swing
0.53 → 0.27 rad target, 0.41 → 0.24 achieved), so the toe stops clearing.
⚠ **On hardware this is servo thrash from the first tick** — reversal at full slew every
20–40 ms. benchd's 40 µs/tick limit caps speed, not reversals, so it passes it through.

**3. Attribution: all command terms off, then one back at a time** (`SETPARAM_AT` at tick 1,
16 MotorEPMv2 params; every patch confirmed by effect):

| only this term on | knee rev | knee at cap | hip1 at cap | real swings |
|---|---|---|---|---|
| none | 0 % | 0 % | 0 % | 0 |
| coupling (1.509) | 59 % | **92 %** | 0 % | 91 |
| stroke (1.2 + heading) | 0 % | 0 % | **88 %** | 0 |
| `stance_lift` (0.5 / 0.25) | 46 % | **100 %** | 0 % | 0 |
| explore noise (0.05) | 60 % | **52 %** | 41 % | 0 |
| postural (1.078) | 86 % | 0 % (small ringing) | 0 % | 0 |
| learned HK (`motor_gain` 3) | 21 % | 3 % | 0 % | 73 |
| swing tuck + rear land | 0 % | 0 % | 0 % | 0 |
| height homeostat | 0 % | 0 % | 0 % | 10 |

**Four imposed terms are each sufficient for full-slew jitter:**
- **Coupling and stroke** are driven by `L.phase = atan2(15·Δknee, knee − ema)`, computed
  from a one-tick velocity with no smoothing (`MotorEPMv2.cpp:2707-2733`). It flips ~180°
  whenever the knee reverses, and the coupling closes the loop on its own input: a relay
  limit cycle.
- **`stance_lift`** toggles on a swing detector with no hysteresis (see 4).
- **Explore noise** is white, fresh every tick, with a step equal to the slew cap.

The **learned HK term is the only smooth one, but alone it does not walk**: its 73 "swings"
are legs paddling with the belly on the floor (chassis y 24 mm, displacement 0.00 m). The
imposed terms supply posture and propulsion, and the jitter along with them. Removing any
single term from the full stack leaves knee reversals at 41–52 %. Learned cooperates,
imposed fights, now with the mechanism in the trace.

**4. Harness defect: the promoted swing detector never sees a command.**
`feet_y_gravity_cmd_imu` / `_cmd_acc` are FK of `servo_targets[]`
(`picrawler_body.gd` ~6940), which is written only in `_cpg_drive_calibrate`, the gang
drive, and the operator's panels. **Verified live:** in a brain run the knee target swept
−1.78 → +0.79 rad while `servo_targets` stayed at 0.0000. So the detector reads the
construction pose rotated by the fused attitude estimate: it is an **attitude detector**, and
`stance_lift` switches on body rocking. That is a legal signal, but not the one its name
claims. It also re-reads 2026-07-25's "`feet_y_gravity` beats the oracle", which the oracle
note already explained as body bounce. The ledger's `fk_cmd_err` is "construction pose vs
achieved", not "commanded vs achieved". ⚠ A robot port computing a TRUE commanded-FK would
not behave like the sim.

**5. Harness defect: `SETPARAM_AT` reports "OK" for keys MotorEPMv2 cannot set live.**
`phase_sym_smooth` is in the config-time `apply_param` set but not in the live `set_param`
chain. The patch printed `-> OK` and the trace was byte-identical to the baseline. Any
`SETPARAM_AT` arm must show an effect before it is believed.

**What this means for the operator's plan** (servo speed and gait adapt to the robot's own
confidence, slow and gentle first): the jitter comes from imposed terms running at full gain
from tick 0, not from the servo speed. The levers, one at a time:
- **(b)** smooth the phase that coupling and stroke ride on. ⚠ That family was refuted
  2026-08-09 (repair plan P1 / P4 arms 1–2: "the phase is a STATE OBSERVATION and every
  consumer needs it raw"). This is a re-test in a new context: robot servo speed, FSR body,
  jitter as a hardware cost.
- **(a)** temporally correlated exploration noise, scaled by learned confidence.
- **(c)** feed the detector the real command, and give it hysteresis. That is a re-baseline
  the robot port must match.

### ★★ 2026-10-02 — THREE JITTER LEVERS AT THE ROBOT'S SERVO SPEED: the swing can lengthen, the thrash is reduced but not removed

**Protocol for all three:** P-e (FSR leg, raw boom), arena difficulty 0.3, n=6 × 6000 ticks,
seeds 1–6, at **3.668 rad/s (robot)** and **6.0 rad/s (sim default)**. Per-seed traces in every
arm (identical instrument settings). Standard metrics from `arenaavg.parse`; belly from
chassis y − 26 mm. Jitter metrics from the trace: reversal fraction of the slew-limited target,
knee ticks at the slew cap, real swings (true contact ≥ 4 ticks) per 1000 leg-ticks, and median
real-swing length. Controls: 3.668 — net_disp 4.03 ± 0.32, steps 0, knee reversals 43 %,
knee at cap 97 %, real swings 7.5, swing median 6.2, tilt_sd 0.052, straight 0.81. 6.0 — net_disp
8.61 ± 0.57, steps 28, knee reversals 41 %, real swings 21.6, swing median 6.7, tilt_sd 0.066,
straight 0.87. 0 falls in both.

**(b) `phase_sym_smooth` — smooth the per-leg phase that coupling and stroke ride on.**
A re-test of the family refuted 2026-08-09 (repair plan P1 / P4), in a new context: the robot's
servo speed, the FSR body, jitter judged as a hardware cost. Config-time (it is not
live-patchable; see the previous entry). Consumer check: `pretro` 0.61 → 0.86 (the filtered
readout reads more retrograde, as P1 recorded), so the lever acted.

| | 3.668 s=2 | 3.668 s=5 | 6.0 s=2 |
|---|---|---|---|
| net_disp | **5.59 ± 0.58** | 3.29 ± 1.98 | 5.22 ± 0.56 |
| steps | 8.2 (5/6 seeds) | 23 | 82 |
| real swings / swing median | 13.0 / **8** | 14.1 / 9 | 27.8 / 7.5 |
| knee / hip1 reversals | **28 % / 23 %** | 27 % / 26 % | 30 % / 30 % |
| knee at cap | 96 % | 96 % | 94 % |
| tilt_sd / straight | 0.082 / 0.78 | 0.317 / 0.45 | **0.153 / 0.56** |
| td_plv / falls | 0.10 / 0 | 0.12 / **1.7** | 0.19 / 0.17 |

**Verdicts:**
- **`PARTIAL` at 3.668, s=2.** The first lever to lengthen the swing toward a slower servo
  (6 → 8 ticks). Real swings nearly double, distance rises 39 %, and reversals drop by a third.
  It costs wobble (+58 %) and contact locking (td_plv 0.25 → 0.10, P1's prediction).
- **`REGRESSION` at 6.0** (straight 0.87 → 0.56, distance −39 %, wobble ×2.3). Same direction as
  the 08-09 refutation.
- **s=5 is unstable** (2/6 seeds collapse).
- **It does not remove the thrash:** the knee still sits at the slew cap 96 % of the time.

Not promoted. Re-use context: combined with a lever that removes the `stance_lift` and noise
jitter, so its contact-locking cost is judged against a smoother command; or wherever wobble is
cheap.

**(a1) `explore_noise_tau` — exploration noise as a per-joint Ornstein-Uhlenbeck process**
(same stationary σ). New MotorEPMv2 param, default 0. **Byte-identical at 0, verified:**
6000/6000 trace lines and 110/110 diag lines matched a pre-build control. Mechanism check
(noise as the ONLY term, single seed): knee-at-cap 52 % → **15 %**, reversals 60 % → 49 %.
**`NULL` in the full stack at both speeds**: τ=10 / 25 at 3.668 give net_disp 3.88 / 4.42,
reversals 43 %, at-cap 97 %; τ=10 at 6.0 gives 8.13 ± 1.66, reversals 39 %. Two reasons, both
measured:
- Noise is not the dominant jitter source once coupling, stroke and `stance_lift` are on.
- **⚠ A first-order OU is a weakened slice of "smooth noise" (§3.2 #6).** It shrinks each tick's
  step, but its VELOCITY is still white, so it still reverses on ~half the ticks.

Kept, default off. Re-use context: a second-order (cascaded) filter, so the noise is
differentiable; and its confidence-scaled form (a2), once noise is a material share of the
remaining jitter. a2 was **not built**, because it would scale a term that is not the thrash.

**(c) `cmd_fk_source=1` — the promoted swing detector runs FK on the slew-limited targets.**
It no longer reads `servo_targets`, which stays 0 in brain mode (previous entry). New body
export / `OGMA_PICRAWLER_CMD_FK_SOURCE`, default 0. **Byte-identical at 0, verified** (6000/6000
trace, 110/110 diag). Consumer check: `fk_cmd_err` moved (6.0: 18.6 → 14.1 mm).

| | 3.668 | 6.0 |
|---|---|---|
| steps | **0 → 13.3** (6/6 seeds lift) | 28 → **143** |
| real swings / swing median | 7.5 → 14.7 / 6.2 → **8.5** | 21.6 → 28.6 / 6.7 → **9.8** |
| knee reversals | 43 % → **27 %** | 41 % → **22 %** |
| net_disp | 4.03 → **5.14** | 8.61 → 7.02 |
| tilt_sd / straight | 0.052 → **0.105** / 0.81 → 0.77 | 0.066 → **0.120** / 0.87 → **0.68** |
| knee at cap / falls | 97 % / 0 | 95 % / 0 |

**`PARTIAL`, a re-baseline candidate.** Stepping is the loud signal (CLAUDE.md §3.3
"proto-gait steps"): feet lift on every seed at the robot's speed, 5× the steps at 6.0, longer
swings, and fewer reversals. The costs are wobble (~2×) and straightness, with distance −18 % at
6.0. The current gains were tuned against the attitude-driven detector, so this is the body
without its matching gains. That is a reason to watch it and re-settle, not to judge it final.
It is the same shape as P-c honest joints ("more rhythm, less straight"). Registered in the
launcher as **P-e·c** for observation. A robot port must match whichever source is promoted.

**Across all three:** the knee runs at the slew cap 94–97 % of ticks in every arm. The thrash is
reduced (reversals 43 % → 27 % at best) but not removed, because coupling, stroke and
`stance_lift` all still command far past what the servo can reach every tick. Next candidates:
- `swing_hyst_frac` on the corrected detector. The 08-09 P2 deadband `NULL` was measured on the
  attitude detector, so its context has changed.
- b + c stacked.
- An amplitude change: the command's distance from the slewed target, which is what puts the
  knee at the cap.

### ★★★ 2026-10-02 — THE FSR-LEG BODY LIFTS ITSELF UNTIL IT TIPS: the height ratchet is fed false "grounded" readings by the raw boom (and lever c's "0 falls" was a 6000-tick blind spot)

> ⚠ **CORRECTED SAME DAY — the "false grounding" mechanism below is WRONG; the ablation
> result stands.** The claim rested on comparing the boom's MINIMUM reading with the belly's
> MEAN height, which shows nothing about the moment the boom reads zero. Checked properly
> against the correct belly-centre ray (`gc_belly`, post-`2072f56`), on the same P-e logs:
> - The boom reads < 3 mm on 1.9 % of samples, and at those moments the true belly-centre is
>   **median 7 mm** (p10 0.8, p90 16).
> - Across all samples the boom minus the true belly is **median −2.8 mm** (p05 −12, p95 +4).
> - In P-e·c, most boom-zero samples come AFTER a tip (tilt median 1.5 rad), as a consequence.
>
> **The uncorrected boom, with its fitted offset, tracks the belly to a few millimetres, as
> the operator said it should.** What stands: the height homeostat lifts the body until it
> tips, and `height_ground_gain`=0 removes the tipping (0/8 seeds). WHY the controller over-lifts
> on this configuration is open. A one-factor-at-a-time test (servo speed, FSR leg, tilt
> compensation, each against P-e at 24 000 ticks) is running.
>
> **The proposed "grounding confirmed by foot load" lever is REJECTED by the operator, for
> reasons that hold:**
> - On terrain (the pyramid case) the belly can rest on an obstacle while the feet stay loaded.
> - In splayed poses foot load redistributes without any belly contact.
>
> Total foot load is not a belly-contact observation. Re-use context: none proposed.

**Verdict: diagnosis, confirmed by ablation (signal, n=4 × 24 000 ticks, every seed agrees).**
Operator observation in the UI: *the robot elevates its body too far off the ground and tips
over.* P-e and P-e·c, 3.668 rad/s, arena difficulty 0.3, seeds 1–4, 24 000 ticks (~8 min).
Traces off, identically in every arm.

| | P-e | P-e, `height_ground_gain`=0 | P-e·c | P-e·c, `height_ground_gain`=0 |
|---|---|---|---|---|
| resets (seeds tipped) | 2 (1/4) | **0 (0/4)** | **10 (3/4)** | **0 (0/4)** |
| `height_k_eff` max | 0.69 | 0.30 | 0.83 (0.95 on 2) | 0.30 |
| `height_bias` max | 1.21 | 0.52 | **1.50 (railed) on 4/4** | 0.10 |
| tilt max, t ≥ 12 000 (rad) | 1.00 | 0.41 | 2.04 | 0.51 |
| belly mean (y − 26 mm) | 37 mm | 24 mm | 37 mm | 25 mm |
| peak belly before a tip | 89–109 mm | — | 102–121 mm | — |
| displacement | 10.1 m | **10.3 m** | 6.4 m | 5.2 m |

**Mechanism** (MotorEPMv2 ~2603–2660; constants in the header):
- The target is `height_k_eff × chassis_h_max`.
- **`chassis_h_max` is a never-decaying record, and it reads 1.00 (the 60 mm clamp) in every
  window of every seed.** The FSR leg spawns the belly at 66.7 mm, above `stand_m` = 60 mm, and
  the raw boom spikes high whenever the nose pitches down. This is the `stand_m` problem
  `picrawler_foot_fsr_mod.md` §5.3 predicted, now with a consequence.
- **"Grounded" is a raw reading below 0.05 (3 mm).** The UNCORRECTED boom reads **0 mm**
  routinely while belly height from chassis geometry averages 25–45 mm: a nose-up pitch swings
  the aft boom toward the floor.
- Each false "grounded" raises `height_k_eff` by 1 % of its gap to 0.95. It decays 20× more
  slowly. So the target climbs from 18 mm toward 51 mm, `height_bias` rails at +1.5, the chassis
  lifts past 100 mm, and the body tips.
- It is the 2026-09-13 ratchet again, with a different source of false grounding (the belly-ray
  bug then, the boom's pitch error now).
- P-e·c is worse because `stance_lift` now engages on genuinely planted legs, adding knee lift
  to the same climb.
- The GainEvolver is not the cause: its first mutation cannot land before ~t = 22 000, and the
  climb starts earlier.

**⚠ Correction to the lever-c entry above:** its "0 falls" was measured at 6000 ticks, and
every tip here comes after t ≈ 12 000. Over a long horizon lever c raises tipping (3/4 seeds
vs 1/4) through this ratchet. The stepping gain stands; the safety reading does not. **Any
safety claim needs runs longer than the slowest integrator. 6000 ticks is shorter than this
one's climb.**

**This is not a refutation of `height_ground_gain`.** It is promoted `WORKING` (line 136: it
made the setpoint discovered rather than asserted, and fixed belly grounding on the cad body).
The ablation is a causal TEST. The lever is sound; its evidence is false. Two levers follow,
one at a time:
- **Grounding from an observation that sees it.** When the belly truly grounds, weight leaves
  the feet. The foot-load channel (FSRs on the robot, contact normal force in the sim) measures
  exactly that. Require low clearance **and** low total foot load, relative to its own running
  level and not a fitted constant, before the ratchet climbs. The confound rides in the channel.
- **An adaptive ceiling:** `chassis_h_max` / `stand_m` must not be pinned by the spawn pose.

`height_windup_guard` (`NULL` on 2026-09-13, against a broken baseline) is a third candidate;
its recorded re-use condition ("a ratchet recurs") is now met.

**For the hardware:** P-e IS the robot's configuration (raw boom, this homeostat). As shipped,
it should be expected to do this after several minutes. Fix before the first hardware run.

### ★★ 2026-10-02 — WHAT MAKES THE FSR-LEG BODY TIP: one factor at a time, and the operator's two observations

**Verdict: the tipping the operator saw is lever c (P-e·c). P-e is operator-approved for its
behaviour: "looks good, the robot is moving conservatively and there are no dangerous falls."**
Supersedes the open question in the corrected entry above.

**One factor at a time against P-e** (3.668 rad/s, FSR leg, raw boom), 4 seeds × 24 000 ticks,
arena difficulty 0.3. Every arm's receipt was checked (servo speed, body, `tof[...]`).

| arm | resets (seeds) | `height_bias` max | ticks railed (>1.4) | boom − true belly | tilt max |
|---|---|---|---|---|---|
| P-e | 2 (1/4) | 1.21 | 1.2 % | −2.8 mm | 1.14 |
| servo 6.0 rad/s | 0 (0/4) | 0.61 | 0 % | −2.2 mm | 0.75 |
| old 76.5 mm leg | 0 (0/4) | 0.73 | 0 % | −2.8 mm | 0.30 |
| tilt compensation ON | 4 (2/4) | 0.75 | 0 % | +0.4 mm | 1.60 |
| `height_ground_gain`=0 | 0 (0/4) | 0.52 | 0 % | −2.5 mm | 0.48 |
| *P-e·c (lever c)* | *10 (3/4)* | *1.50* | *every seed* | *−2.3 mm* | *2.04* |

**Findings:**
- **The ToF is not the cause.** The raw boom reads within ~3 mm of the true belly in every arm.
  Tilt compensation makes it more accurate (+0.4 mm) and does not reduce tipping.
- **The mechanism is integrator windup in the height homeostat.**
  - `height_k_eff` ratchets 0.3 → ~0.6 in every ratchet-on arm, on GENUINE near-grounding
    (belly-centre median 7 mm when the boom reads < 3 mm).
  - `chassis_h_max` sits at its 60 mm clamp, so the target is ≈ 36 mm.
  - When the body cannot hold that, `height_bias` winds toward +1.5, the legs over-lift, and the
    body tips.
  - Railing is the readable precursor: every P-e·c seed, briefly in P-e, never elsewhere.
- **Lever c is the operator-observed cause.** Its knee tuck now engages on genuinely planted
  legs and stacks with the height bias. Stays NOT PROMOTED. Re-use context: with an anti-windup
  on the height integrator, or with `stance_lift` re-tuned against the corrected detector.
- **P-e at 3.668 with the FSR leg winds up occasionally** (1 seed). One tip-over across 4 seeds
  cannot rank the arms against each other (a rare discrete count, as GainEvolver's own `w_falls`
  note says), so the factor ranking is a lead, not a finding.

**Operator UI observation of P-e** (rule 5): conservative movement, no dangerous falls, judged
good. This sits beside the measurements, which still stand:
- 0 lifts past the fixed `feet_y` step threshold.
- True-contact swings at a quarter of the 6.0 rate.
- 1 tip-over in 4 × 24 000 ticks.

The conservative, slow gait at the robot's servo speed is what the operator wants first on
hardware (servo safety before speed). This is the first configuration both measured and watched
as the robot will run it.

Open, in order:
- (a) anti-windup on the height integrator (`height_windup_guard` re-test, judged on railing and
  tilt at n ≥ 8, long runs);
- (b) the adaptive height ceiling (`stand_m` / `chassis_h_max` pinned by the spawn pose);
- (c) the remaining full-slew command jitter.

### ★★★ 2026-10-02 — P-e IS NOT PORTABLE AS-IS: legacy reward events reset the motor state on ≥39 % of ticks, plus a windup-guard NULL

> ⚠ **CORRECTED SAME DAY — the events fire in the UI ONLY, not in headless runs.**
> `stability_gain` and `height_penalty_gain` default to 0 in the body. P-e's 0.05 is metadata,
> which `ExperimentConfig.resolve_picrawler_stab_gain()` honours only when `launched` (the UI);
> headless falls through to the env var, then the 0 default.
> - **Every headless P-e number in this ledger ran with NO shaping events**: the shuffle, the
>   jitter levers, the tipping, the windup tests.
> - **The operator's UI observation of P-e ran WITH them**: two controllers under one config
>   name.
> - The "≥ 39 %" below is how often the firing CONDITION holds in the headless traces, i.e.
>   what the UI would emit.
>
> The body printed no receipt for these gains, which is how this went unseen. That is the same
> UI/headless parity class as `metadata.body` (2026-08-28) and the 2026-08-05 joint backend.
> **The operator's decision (2026-10-02): turn the events off in the robot-faithful arm.**
> Headless P-e already matches that; the UI must be made to match, and the operator must watch
> the events-off version.

**1. `height_windup_guard` = 1 on P-e — `NULL`, leaning `REGRESSION` (n=8 × 24 000 ticks,
3.668 rad/s, arena difficulty 0.3).**

| | control | guard |
|---|---|---|
| resets (seeds) | 2 (1/8) | 3 (2/8) |
| bias railed | 0.6 % | 4.7 % |
| `height_k_eff` max | 0.70 | 0.65 |
| tilt mean | 0.11 | 0.22 |
| displacement | 10.2 ± 0.3 | 9.4 ± 3.6 (one seed collapsed, 0.61 m) |

The guard lowers the ratchet slightly, but the integrator still rails, more often than the
control. It does not address windup on this body. Re-use context: an anti-windup on the
integrator itself (`height_bias`), not on the setpoint ratchet. P-e's own tip rate is ~1 in
8 seeds per 24 000 ticks.

**2. ★ Hidden dependency: legacy reward-shaping events drive MotorEPMv2's reset mask.**
- P-e's metadata sets `stability_gain` 0.05 (also `height_penalty_gain` 0.05,
  `target_height` 0.085).
- With `stability_gain` > 0 the body publishes `events.miss` on **every tick** where
  `chassis_y / STANDING_CHASSIS_Y > 0.5` and world 3-D speed > 0.05 m/s
  (`picrawler_body.gd` ~8045).
- MotorEPMv2 treats ANY `miss` or `reset` as a respawn (`MotorEPMv2.cpp:1206-1218`). It zeroes
  `heading_bearing_` (the gain-7.0 bearing hold), `fwd_progress_ema_` (commit, and the height
  fade), commit, flow, the stuck state, and every per-leg step clock.
- Measured lower bound from the traces (forward speed only, which understates 3-D speed):
  **≥ 39 % of ticks at 3.668 rad/s, ≥ 69 % at 6.0. The median gap between resets is 0 ticks.**
- So in the sim those states almost never accumulate while the body walks.
- It is an RL-era reward event (CLAUDE.md §5.1 prohibits reward shaping), computed from world
  height and world speed. **The robot cannot emit it.** On hardware the heading hold, commit
  and height fade would accumulate freely: a different controller than the one validated.
- The same reset path also takes the height-miss and fall-miss events (world height, exact
  tilt).

**3. Other sim-truth inputs in P-e** (full per-topic contract in the port doc, "Brain input
contract"):
- `joints` = achieved hinge angles (`honest_joints` off).
- `imu` = world yaw, world forward velocity and world yaw rate (`honest_imu` off).
  MotorEPMv2's commit (0.030 m/s) and height fade (0.025 m/s) thresholds are on the
  true-velocity scale.
- `upright` = exact basis (`honest_upright` off).
- `joint_torque` = a PD model of achieved angle and velocity.
- `distress` = world XZ displacement × exact tilt.
- `foot_contact` = whole-shank physics contact.
- `foot_load` = physics normal impulse.
- Also: `tilt` is published by `ogma_host` but not by the P-e sim, and MotorEPMv2 subscribes
  by DEFAULT, so on the robot it would switch on a coord-fitness wobble penalty the sim never
  had.

**Consequence:** the configuration the operator approved in the UI has been validated only
with inputs the robot cannot produce. The gate before hardware is a sim arm in which every
input is one the robot can actually publish ("P-e honest"), A/B'd against P-e and watched in
the UI. The decisions it needs are listed in the port doc.

### ★★★★ 2026-10-02 — ON THE INPUTS THE ROBOT CAN PUBLISH, P-e FLIPS OVER — and one input does it: joints from the servo forward model

**Verdict: the hardware gate FAILED, and the cause is isolated (signal, every seed agrees).**
3.668 rad/s, FSR leg, raw boom, arena difficulty 0.3, shaping events off (headless).
Every arm's receipt was checked.

**P-e·h — all robot-faithful inputs together** (`honest_joints`, `honest_imu`, `honest_upright`,
`honest_distress`, `joint_torque_zero`, `tilt_topic` ""), n=8 × 24 000:

| | P-e | P-e·h |
|---|---|---|
| resets (seeds tipped) | 2 (1/8) | **64 (8/8)** |
| tilt max | 0.78 rad | **3.03 rad, inverted** |
| height bias railed | 0.6 % | **54 %** |
| `height_k_eff` max | 0.70 | 0.94 |
| straight | 0.59 | 0.18 |
| steps | 22 | 100 |

**Each change added alone to P-e**, n=4 × 12 000, against P-e's first 12 000 ticks (0/8
tipped, railed 0 %):

| added alone | tipped | resets | railed | `height_k_eff` | displacement |
|---|---|---|---|---|---|
| **`honest_joints`** | **3/4** | **10** | **53 %** | **0.95** | **1.4 m** |
| `honest_imu` | 0/4 | 0 | 0 % | 0.53 | 7.9 m |
| `honest_upright` | 0/4 | 0 | 0 % | 0.61 | 7.3 m |
| `honest_distress` | 1/4 | 1 | 0.5 % | 0.61 | 7.2 m |
| `joint_torque_zero` / `tilt_topic` "" | 0/4 | 0 | 0 % | 0.61 | 7.3 m |

Torque, tilt and upright reproduce the control exactly. Their consumers do not act inside
12 000 ticks (GainEvolver's first mutation lands at ~22 000), or the sim never published the
signal. They are inert here; that is not a claim about longer runs.

**So the approved P-e behaviour depends on ACHIEVED joint angles,** which hobby servos cannot
report. The brain needs a joint sense the robot does not have.

**Mechanism — inferred, NOT verified:** with `joints` = the forward model of its own command,
the brain cannot see its legs give under load. MotorEPMv2's postural term (−1.078·(x − rest))
and the HK loop then act on the command they issued, a self-referential loop with no
information about sag. The body settles lower than intended, the height homeostat sees the
belly low, `height_bias` winds to its rail, the legs over-extend, and the body flips.

**Re-reads P-c** (2026-09-11 "honest joints: WORKING, more rhythm, less straight"). That was
n=6 × 6000 in the corridor on the 76.5 mm leg at 6.0 rad/s, boom off, so on the short belly ray
(2026-10-02). On the robot's body and speed, over a long horizon, the same input is a
**REGRESSION to repeated inversion**. Same lever, different context: §3.1.

**Re-use contexts / what would fix it** (CLAUDE.md §1 rule 2: when the needed signal is not
in the channel, the fix is a sensor, not a smarter policy):
- (a) A real joint-angle observation: tap each servo's internal potentiometer into an
  external ADC (the HAT's ADC is taken by the FSRs and the battery).
- (b) An observation of the deflection the forward model misses. Body height and attitude
  (ToF, IMU) and foot load (FSRs) constrain it; a learned correction graded by its own error.
- (c) Re-settle the gains on the forward-model joints (the E3b searcher), with anti-windup on
  `height_bias` itself.

**The robot must not run P-e·h.**

### ★★★ 2026-10-02 — OPTION C, STEP 1: on the robot's real inputs, the height RATCHET is what flips the body

**Verdict: diagnosis by intervention (signal, n=4 × 12 000 per arm).** Base is P-e·h: every
input is one the robot can publish, shaping events off, 3.668 rad/s, arena difficulty 0.3.
The operator's goal (2026-10-02): *see what is possible with the robot's current hardware*,
i.e. joints from the servo forward model, with no encoders.

| P-e·h + one change | tipped | bias railed | tilt mean | belly | displacement | straight |
|---|---|---|---|---|---|---|
| none | 8/8 (8 seeds) | 54 % | 0.56 | 28 mm | 3.9 m | 0.18 |
| **A `height_ground_gain` 0** (ratchet off) | **1/4** (1 reset) | **0 %** | 0.18 | 21 mm | 8.3 m | 0.62 |
| B `height_unwind_free` 1 | 4/4 | 38 % | 0.42 | 25 mm | 5.0 m | 0.38 |
| C `homeo_leak_cycles` 4 | 3/4 | 11 % | 0.38 | 25 mm | 4.5 m | 0.34 |
| D `homeo_upright_gate` 0.8 | 4/4 | 49 % | 0.44 | 27 mm | 4.4 m | 0.34 |
| **E `height_homeo_gain` 0** (homeostat off, a lesion TEST) | **0/4** | — | 0.14 | 23 mm | **10.3 m** | **0.78** |

**Reading:**
- The windup is the ratchet. With `height_k_eff` pinned at `height_k` (0.30), the homeostat
  defends a fixed fraction of its ceiling and the body stops flipping.
- Treating the integrator alone (B, C, D: faster unwinding, finite memory, freeze-when-tipped)
  does not stop it. The setpoint keeps climbing past what the body can hold.
- With the homeostat off entirely the body is steadiest and moves farthest. The belly still
  rides 21–23 mm off the floor in every arm.
- C and D re-test refuted members of the 2026-07-26 plasticity family. Refuted again here, in
  the robot-faithful context.

**Next (option c, step 2, running):** a GainEvolver re-settle from A, with
`height_homeo_gain` added to the evolver's keys (range 0–0.15). The search, not a designer,
then decides how much height control this body wants, from off to twice today's value,
alongside `amp_target`, `coupling_gain` and `postural_gain`. Frozen-evolver control. 4 seeds
× 200 000 ticks per arm.

### ★★ 2026-10-02 — OPTION C, STEP 2: the live gain search does not beat frozen gains on the robot's inputs (truncated run)

> ⚠ **CORRECTED SAME DAY by the completed confirmation (entry below): the tips are NOT confined
> to the first 40k ticks.** The 4-seed truncated read below was too short and too small to see it.

**Protocol:** P-e·h + `height_ground_gain` 0. Search arm: GainEvolver live, with
`height_homeo_gain` added to its keys (seed 0.077, range 0–0.15). Control arm: same, evolver
frozen (`mutation_sigma` 0). 4 seeds each. All 4 searched gains confirmed landing
(`ga_app` 4, `ga_rej` 0).

⚠ **HARNESS DEFECT: every seed was cut at 87–95k of the planned 200k ticks.** The runner's
per-seed timeout (1500 s) was sized for short runs. Fixed (timeout now scales with steps).
The data to ~90k is valid; the protocol is half-run. Also, per-segment displacement after the
first 20k is the arena edge, not the gait.

| | frozen gains | evolver searching |
|---|---|---|
| resets 0–40k ticks | 2 (both seed 2) | 2 |
| resets 40–90k ticks | **0** | **6** (seeds 1, 2) |
| tilt mean over time | 0.163 → 0.099 (settling) | 0.16–0.21 |
| `height_bias` railed | ~0 % | 0–1.8 % |

**Endpoints scatter:** [0.15, 1.85, 1.40, 0.06], [0.15, 1.20, 1.22, 0.10],
[0.32, 1.50, 1.50, 0.03], [0.27, 0.70, 0.26, 0.12] (amp_target, coupling, postural,
height_homeo_gain). 1–2 accepts per seed, so not converged, and "home is a region" again
(2026-08-27).

**Verdicts:**
- **Frozen P-e·hr is the current-hardware candidate.** Its tips fall in the first ~40k ticks,
  while the brain is still learning, and stop afterwards. Tilt keeps settling through online
  learning alone.
- **The live search is `NULL` leaning `REGRESSION` within 90k.** Every candidate window runs
  an untested gain set on the body, and some tip it.

**For hardware:** running the evolver live on the robot means exploring gains on a real body,
and in the sim that costs falls. The early-learning tips are the operator's stated concern:
slow and gentle while the brain knows little.

Registered as **P-e·hr** in the launcher. Confirmation running: frozen, n=8 × 100 000,
completed runs this time.

### ★★ 2026-10-02 — P-e·hr CONFIRMATION (n=8 × 100 000, completed): ~1 tip per 80k ticks, spread through the run

**Verdict: `PARTIAL` — a ~27× lower tip rate than P-e·h, not yet safe enough to call cleared.**
P-e·hr, gains frozen, 3.668 rad/s, arena difficulty 0.3. Every seed ran all 100 000 ticks.
Receipts checked.
- **Resets: 10 on 5/8 seeds** (0, 2, 0, 0, 1, 5, 1, 1), at ticks 540, 8 580, 27 540, 29 580,
  41 040, 41 400, 52 740, 64 860, 71 580, 93 960. **Spread through the run.** That retracts
  step 2's "only early" read.
- **Rate:** ≈ 1 per 80k ticks (~27 min of robot time at 50 Hz). P-e·h: 64 per 192k (1 per 3k).
  P-e headless: 2 per 192k (1 per 96k). Same order as the configuration the operator approved,
  but P-e has not been run to 100k, so that comparison is a lead, not a finding.
- **3/8 seeds clean throughout,** tilt mean settling to ~0.09.
- **Seed 6 ends tipped and STUCK:** over the last 20k ticks tilt mean 1.47 rad, `height_bias`
  railed 70 %, `height_k_eff` still 0.30. The INTEGRATOR winds up on a body that cannot reach
  its target, even with the ratchet off. Anti-windup on `height_bias` itself is still open;
  B, C and D did not do it.
- **Belly** median 22 mm (p05 3.8 mm); 3.5 % of samples under 3 mm.

**Next candidate:** P-e·h + `height_homeo_gain` 0 (homeostat off). It tipped 0/4 at 12 000
ticks, but has never had a long run.

### ★★★★ 2026-10-02 — OVER 100k TICKS THE HEIGHT HOMEOSTAT IS WHAT TIPS THIS BODY — including in P-e itself; off, the robot-faithful brain is the safest config measured

**Verdict: signal, n=8 × 100 000 per arm (every seed completed), gains frozen, 3.668 rad/s,
arena difficulty 0.3.** Receipts checked (P-e: `honest[...]` off; the others on).

| | resets (seeds) | per 100k | bias railed | stuck tipped | tilt mean | belly median | belly < 3 mm |
|---|---|---|---|---|---|---|---|
| **P-e** (sim-only inputs) | **67 (7/8)** | **8.4** | 13.7 % | 2 | 0.40 | 32 mm | 0.7 % |
| P-e·hr (robot inputs, ratchet off) | 10 (5/8) | 1.25 | 2.4 % | 1 | 0.22 | 22 mm | 3.5 % |
| **P-e·h0** (robot inputs, homeostat off) | **2 (2/8)** | **0.25** | 0 % | 0 | 0.16 | 26 mm | 0.5 % |

**Findings:**
- **P-e is not safe over a long horizon.** The 24 000-tick runs (1/8 tipped) were shorter than
  the windup, and so was the operator's UI watch, which also ran a different controller (the
  UI-only shaping events, entry above). A safety claim needs a run longer than the slowest integrator: 100k ticks shows
  what 24k could not.
- **On the robot's real inputs, switching the height homeostat off gives the lowest tip rate
  measured:** 33× below P-e, 5× below P-e·hr. No seed got stuck. The belly rides HIGHER (26 mm)
  with fewer near-ground samples than with the homeostat on.
- **So the height homeostat, with or without its ratchet, is the destabiliser on this body
  over long runs.** Its promotion (2026-08-27, `WORKING` signal) was cad body, corridor, 6000
  ticks. Refuted in THIS context: FSR leg, robot servo speed, robot-faithful inputs, 100k ticks,
  arena difficulty 0.3.
- **Re-use context:** terrain. The homeostat exists to get the body over the hump (CLAUDE.md
  §1, the belly ToF), and no terrain run has been made here. Also re-test it with an
  anti-windup on `height_bias` itself.

**Registered as P-e·h0.** Not promoted and not cleared for hardware. It needs the operator's
watch, a terrain test, and the remaining safety items (servo jitter, the robot-side input port
and the actuation path).

### ★★ 2026-10-03 — OPERATOR UI OBSERVATION: P-e·h0 is the better configuration; P-e·hr binds up on terrain

The operator watched P-e·h0 and P-e·hr in the UI (rule 5): **"P-e·h0 is the better
configuration. hr causes the robot to get bound up on terrain in some cases."** This sits
beside the 100k-tick measurement (P-e·h0 0.25 tips per 100k vs P-e·hr 1.25) and points the
same way. It is also the first observation on terrain, the height homeostat's stated re-use
context. With the homeostat on (P-e·hr), the body binds on terrain; with it off (P-e·h0), it
does not, in what the operator saw. A measured terrain A/B follows.

### ★★ 2026-10-03 — TERRAIN A/B: dropping the height homeostat costs no hump traversal (P-e·h0 vs P-e·hr)

**Verdict: P-e·h0 TIES P-e·hr on terrain and clears the hump farther (signal, n=6).** FSR leg,
robot-faithful inputs, shaping events off, 3.668 rad/s, difficulty 0.3. Receipts checked.
This answers the open re-use context from the 100k entry: the homeostat exists to get the body
over the hump, and on this body it is not needed for that.

| | P-e·h0 (homeostat off) | P-e·hr (ratchet off) |
|---|---|---|
| hump gate (`humpavg`, teleport onto the crest): cleared | **6/6** | 6/6 |
| hump: final_z / gain past the crest | **6.29 / 3.68 ± 0.60** | 5.70 / 3.09 ± 0.33 |
| hump: falls | 1/6 | 1/6 |
| corridor 24 000 ticks: net_z | 7.09 ± 2.51 | 7.44 ± 2.33 |
| corridor: straight | 0.38 | 0.44 |
| corridor: unstable | **0.13** | 0.24 |
| corridor: falls | 7 (4 on seed 2) | 3 |

⚠ **Corridor falls are confounded by the gym edge.** The harness flagged 3/6 (h0) and 2/6 (hr)
seeds past z = 8.5, where falls and chassis_y are not trustworthy. h0 seed 2 reached
z = 10.05 and left the world (chassis_y −21.9 m). On the seeds that stayed inside, falls
tie (h0 2, hr 3).

**Together with the operator's UI read ("hr gets bound up on terrain in some cases") and the
100k safety run** (h0 0.25 tips per 100k vs hr 1.25), P-e·h0 is the operator's configuration
on every axis measured. The height homeostat's re-use context stays OPEN for steeper terrain
than difficulty 0.3.

### ★ 2026-10-03 — FIRST BRAIN-DRIVEN RUN ON THE ROBOT: the path works; the thrash the sim predicted is visible and real

**What ran.** P-e·h0 on the Pi (`ogma_host --imu --brain-inputs --actuate`), benchd in `dev`,
30 s of brain control, robot loose in its safety box on a smooth floor, battery. Started from
the rescue pose. Ended with STOP, then `bench` → deadman → rescue pose. Off-board telemetry
record: `tele_20261003_124358_brainrun2.jsonl` (laptop).

**Path verdict: `WORKING` (signal, n=1).**
- 1495 commands applied in 30 s, 0 overruns, 0 bad frames, 4 sequence gaps.
- STOP froze all 12 channels, and `ogma_host` paused the graph for 367 ticks (held, not reset).
- The deadman sent the rescue pose after leaving `dev`.
- Servo-branch current: mean 0.80 A, peak 2.49 A (10 Hz samples). A4 minimum 7.10 V; battery 7.78 V at rest.

**The first attempt reset the Pi** the moment the brain took the servos. The robot was on the
bench supply, which the operator says is only good for desk builds. The cause is NOT measured:
benchd's record of that run is 0 bytes, because the page cache was lost in the reset. The retry
on battery, with the off-board recorder running, did not reset. Re-use context: any power source
change, and the soft-engage question below.

**The thrash (operator: "faster than our sim, with rather jerky movements").** In the 10 Hz record:
- knee and hip1 channels sit at full slew in 20–35 % of 100 ms windows;
- every channel reverses 3–5 times a second (an undercount at 10 Hz).

This is the 2026-10-02 finding arriving on hardware: in the sim the slew-limited knee target sits
at the cap on 97 % of ticks and reverses on 43 %. The sim's joint follows it through PD, a 30 ms
torque lag and inertia, so the achieved joints reverse less (21–39 %). The hobby servo tracks the
50 Hz staircase almost at once, so the robot shows the raw command. Lowering the slew caps speed,
not reversals, and in the sim 1.83 rad/s (20 µs/tick) stopped stepping (0 steps, tilt_sd doubled;
2026-09-13 table).

**Two instrument failures, both found after the run:**
1. **My tilt guard was blind.** benchd failed its IMU probe at start (WHO_AM_I 0x00), so the frame
   carried `imu: null`, and the logger's fallback read "upright" on every sample. The robot stayed
   upright, but the guard proved nothing. The next run's guard must refuse to start without
   attitude.
2. **`ogma_host`'s attitude filter ended diverged.** Exit summary: disagree 116.7°, upright −0.45,
   with the robot resting upright in the rescue pose and |a| 1.003 g. The brain's upright and
   distress inputs may have been wrong for part of the run. When it diverged is unknown: inputs
   were not dumped. The next run needs `--dump-inputs`. The held-pose shadow run ended at upright
   0.9994, so this is new with motion.

### ★★ 2026-10-03 — SERVO OUTPUT LAG α 0.2 ON THE ROBOT: the thrash on the line drops ~4×, current halves (`WORKING`, hardware signal)

**Lever.** `benchd --servo-lag-alpha 0.2` puts a first-order lag between the slewed command and
the HAT, in brain modes only (gain-0: alpha 0 is byte-identical). Rationale: in the sim the joint
follows the slew-limited target through PD, torque lag and inertia. Unloaded it fits α 0.22–0.28
per tick (P-e·h0 trace; stance is load-dominated and fits no lag). The hobby servo has almost
none of that lag, so it passed the brain's raw thrash straight to the legs. α 0.2 is also the
brain's own servo forward model, so its joints input becomes the pulse the servo actually receives.

**Protocol.** Robot loose in the safety box, battery, ABBA (A = off, B = 0.2), 30 s each from the
rescue pose, fresh brain each run (P-e·h0). Measured on benchd's 50 Hz state feed (`out` = pulse on
the line), 5 Hz telemetry, and ogma_host input dumps every 10 ticks.

| arm | reversals/s on the line | ticks at full slew | line speed (rad/s) | command speed | jerk (µs/tick²) | servo I mean / max (A) | A4 min (V) | tilt max |
|---|---|---|---|---|---|---|---|---|
| A1 (off) | 17.8 | 0.80 | 3.16 | 3.16 | 27.9 | 0.79 / 2.91 | 6.92 | 28° |
| B1 (0.2) | 12.7 | 0.01 | 0.94 | 3.26 | 6.9 | 0.37 / 1.06 | 7.37 | 26° |
| B2 (0.2) | 8.8 | 0.02 | 1.15 | 3.17 | 6.3 | 0.44 / 1.64 | 7.16 | 24° |
| A2 (off) | 21.7 | 0.82 | 3.21 | 3.21 | 34.3 | 0.64 / 1.84 | 7.13 | 33° |

- **Thrash.** Jerk on the line is ~4.5× lower, full-slew ticks fall from ~80 % to ~1–2 %, and
  reversals roughly halve.
- **Load.** Servo current about halves, and so does its peak.
- **Brain.** The brain's own command is unchanged (3.2 rad/s, reversals as before): the lever
  filters the plant, not the policy.
- **Operator, watching:** "the lag/smoothing run looked good. definitely reduces the jitter."

⚠ **Slightly MORE damped than the sim.** The robot passes 0.30–0.36 of command speed to the line;
the sim's achieved joints move at 0.40–0.56 of their target (hip1 0.46, hip2 0.40, knee 0.56). α
0.25–0.3 would match better. Not tested. Also n=2 per arm at 30 s with no distance sensor, so
behavioural effects are unmeasured: a signal for the hardware cost, not a gait finding.

### ★★ 2026-10-03 — ogma_host's IMU on a 225 Hz thread: the attitude divergence is gone (`WORKING`, signal)

On the first brain-driven run, ogma_host's attitude filter ended 117° from the accelerometer with
the robot resting upright. It sampled the IMU once per 50 Hz tick, and the servos step at exactly
that rate; benchd's header already warns that a slow sample rate aliases motion. Moved to its own
225 Hz thread; the tick takes the newest attitude and the mean gyro since the last tick. In all
four ABBA runs, including the two no-lag arms that reproduce the earlier motion, the filter ended
within 0.35° of the accelerometer (upright 0.9999), with 0 read errors and ~12 860 samples per
run. Mid-run upright minimum 0.81–0.91, consistent with the 24–33° tilts benchd's own IMU saw.
Not proven causal (the old path was not re-run), but the motion that diverged it no longer does.

### ★ 2026-10-03 — LOWER SLEW (30 µs/tick = 2.751 rad/s) for the robot's jerk: `REGRESSION` in the sim, not run on hardware

**Why it was tested.** The operator saw jerky, faster-than-sim motion on the first brain-driven
run and suggested dialling in the slew. Run in the sim first, because the brain was tuned at
3.668 rad/s.

**Protocol.** P-e·h0 (robot-faithful inputs, homeostat off), arena difficulty 0.3, n=6 × 12000
ticks, seeds 1–6, gains frozen, identical instruments (`score.py`). Speed is the only difference.

| | 3.668 rad/s (40 µs/tick, control) | 2.751 rad/s (30 µs/tick) |
|---|---|---|
| net_disp | 9.99 ± 0.83 | 5.17 ± 0.87 |
| steps | 13.8 ± 4.5 | 4.2 ± 5.7 (3/6 seeds: 0) |
| real swings / 1000 leg-ticks | 11.3 ± 0.6 | 6.1 ± 1.4 |
| belly (mm) | 23.7 ± 2.0 | **15.9 ± 1.6** |
| tilt_sd | 0.090 | 0.080 |
| falls | 0.17 (1 in 6) | 0 |
| knee reversal fraction | 0.34 | 0.37 |
| knee ticks at the slew cap | 0.97 | 0.98 |

**Verdict: `REGRESSION` (signal, n=6).** It halves the gait (distance, swings) and lowers the belly
by 8 mm. It does **not** touch the thrash: reversals and cap time are unchanged, because the slew
caps speed, not reversals (the 2026-10-02 finding again). The only gain is one fewer fall in six,
inside noise. It was NOT run on the robot. The servo output lag (above) already addresses what
the operator saw, at the plant instead of the speed cap.

**Re-use context:** a brain retuned at the lower speed (frozen gains were settled at 3.668), or the
operator's confidence-gated speed (slow while the brain knows little, rising with confidence),
which is a different mechanism from a fixed lower cap.

### ★★ 2026-10-03 — 60 s BRAIN RUN ON THE ROBOT (lag 0.2): the belly comes up briefly, then settles on the floor; the sim never does

**Run.** P-e·h0, `--servo-lag-alpha 0.2`, fresh brain, 60 s, safety box, battery, started from the
**rescue pose**. Belly from the boom ToF at 50 Hz (100 % valid). Clean run: the IMU ended within
0.28° of the accelerometer, 4196 commands sent, servo current 0.37 A mean / 2.06 A peak, A4 ≥ 7.12 V.

| window | belly median (mm) | p90 | >10 mm | foot_load sum | contact | hip2 on the line (rad) |
|---|---|---|---|---|---|---|
| 0–10 s | 14.2 | 30.2 | 62 % | 1.76 | — | (from rescue, −1.8) |
| 10–20 s | 5.2 | 20.2 | 32 % | 0.35 | — | |
| 20–40 s | 3.2 | ~7 | 1–3 % | 0.05–0.08 | 0.4 / 4 | −0.67 |
| 40–60 s | ~3 | 7–13 | 5–18 % | 0.16–0.94 | 1.2 / 4 | −0.70 |

**The sim, same config, its first 60 s (6 seeds):** belly median 14–26 mm in EVERY 10 s window, hip2
target +0.04 / achieved −0.08 rad, knee −0.71, foot_load sum 0.96, contact 3.5 / 4.

**Where the robot departs:** the knees match the sim (−0.68 vs −0.71 rad); **hip2 does not**. The
robot holds hip2 near −0.65 rad (legs pitched up, feet mostly off the floor, belly down) where the
sim sits near 0. Nothing was at the envelope (0 % at min/max on every joint type in the last 40 s),
so this is the brain's command, not a clamp.

**Leading hypothesis — the start state, not the plant.** The sim spawns STANDING (belly ~26 mm,
hip2 ≈ 0) and auto-resets to standing after a fall, so this brain has never had to raise its belly
from the floor. The robot started in the rescue pose (hip2 −1.8 rad, folded). The brain brought hip2
two-thirds of the way back and settled there. **Decisive test:** the same run started from the
`stand` pose. If it holds the belly up from stand, the gap is stand-up-from-floor (a capability
the sim never trained). If it sinks from stand too, it is a sim-to-real gap in holding posture.
`IN_FLIGHT`.

Note: `feet_y` reads a constant −0.093 m on the robot, as designed — with `cmd_fk_source = 0` it is
the zero-pose foot height rotated by tilt, not the commanded FK (see the feet_y oracle doc).

### ★★ 2026-10-03 — START POSE IS THE ROBOT'S hip2 CENTRE: from `stand` the belly stays up (24–51 mm) — then it tips sideways at 34 s

**Run.** Identical to the 60 s rescue-start run (P-e·h0, lag 0.2, fresh brain, same 12 s bench
pre-roll) except the start pose: the operator's `stand` (hip2 at centre, first frame hip2 −0.02…+0.01
rad, belly 63 mm).

| window | belly median (mm) | >10 mm | hip2 median (rad) | knee median | tilt max | contact |
|---|---|---|---|---|---|---|
| 0–5 s | 48 | 96 % | −0.13 | −0.39 | 29° | 4.0 |
| 5–10 s | 51 | 88 % | +0.06 | −0.65 | 17° | 4.0 |
| 10–20 s | 24–36 | 83–98 % | +0.02 | −0.70…−0.78 | 15–17° | 2.6–2.8 |
| 20–30 s | 29–51 | 99–100 % | +0.03…+0.05 | −0.66…−0.73 | 14–34° | 2.8–3.0 |
| 30–34.7 s | 32 | 100 % | +0.03 | −0.77 | **73° → tilt guard STOP** | 3.0 |

- **Hypothesis confirmed (n=1 against n=1).** hip2 held at +0.02…+0.06 rad from `stand`, against
  −0.65…−0.74 from rescue: the brain keeps hip2 near where the body starts. The sim's centre is
  +0.04. The belly stayed off the floor 83–100 % of the time.
- **New failure: it rolled.** At 34 s the body rolled toward +x (up.x 0.87) and the tilt guard
  froze it at up.y 0.30 — the guard's first real trigger, and it worked. P-e·h0 tips about 0.25
  times per 100k ticks in the sim; this tipped at ~1700 ticks.
- **The robot stands TALLER than the sim at the same joint angles.** The belly sat at 24–51 mm, the
  sim's at 14–26 mm, with matching hip2 (≈ 0) and knee (−0.7) commands. The sim's stance joints
  sit off target under load (soft PD; its commanded-FK foot error is ~27 mm). Stiff hobby servos
  do not sag, so the same command stands the robot ~15–25 mm higher, and its centre of mass is
  higher. A plausible contributor to the roll; not measured.

Open: is the roll repeatable (n=1)? Is the sim's stance compliance what the brain's posture was
tuned on? The 12 s bench pre-roll (the brain ticks while its commands are not applied) is still in
every run's protocol.

### ★★★ 2026-10-03 — 60 s FROM `stand`, TILT GUARD AT 80°: THE BELLY STAYS UP THE WHOLE RUN, AND THE POSTURE STEADIES (`WORKING`, hardware signal)

**Run.** The previous stand-start run repeated unchanged, except the harness's tilt guard moved from
60° to 80° (operator: "the robot was not in danger of flipping over"). Full 60 s; the guard never
tripped.

| window | belly median (mm) | p10 | p90 | >10 mm | hip2 (rad) | knee | tilt median | tilt max | contact | servo I (A) |
|---|---|---|---|---|---|---|---|---|---|---|
| 0–10 s | 37 | 6 | 84 | 88 % | +0.00 | −0.75 | 10.8° | 30.0° | 3.8 | 1.24 |
| 10–20 s | 28 | 15 | 47 | 97 % | +0.02 | −0.74 | 7.0° | 21.9° | 2.6 | 1.57 |
| 20–30 s | 28 | 19 | 53 | 100 % | +0.02 | −0.69 | 5.1° | 19.0° | 2.9 | 1.42 |
| 30–40 s | 29 | 20 | 43 | 99 % | +0.03 | −0.75 | 4.8° | 14.1° | 3.2 | 1.44 |
| 40–50 s | 30 | 21 | 41 | 100 % | +0.03 | −0.73 | 4.3° | 12.1° | 2.9 | 1.45 |
| 50–60 s | 37 | 26 | 54 | 100 % | +0.04 | −0.65 | 6.7° | 20.8° | 2.9 | 1.52 |

- **Belly up:** median 31 mm over the run, above 10 mm 97 % of the time and above 20 mm 88 %. The sim
  holds 14–26 mm.
- **Adaptation within the run:** the floor of the belly distribution (p10) rises 6 → 26 mm, and
  median tilt falls from 10.8° to 4–5°. Per CLAUDE.md §3.3, regulation that tightens with experience
  is the kind of result that reads as real; it is still n=1 at 60 s.
- **The previous stand-start run's 73° roll did not repeat** (1 of 2 stand starts rolled).
- **Cost:** holding the belly up draws 1.2–1.6 A mean on the servo branch, against ~0.4 A belly-down.
  A4 dipped to 6.58 V at peaks, from 7.39 V at rest after ~11 runs on one charge: close to the
  6.4 V limp line. The battery is the next run's limit.
- IMU ended 0.28° from the accelerometer; 4198 commands, 0 dropped.

### ★★★ 2026-10-03 — SERVO CURRENT ABOVE ~2.4 A RESETS THE HAT's MCU, AND THE DRIVER THEN DROVE THE SERVOS TO THEIR END STOPS (fixed in software; the power ceiling is open)

**What the operator saw.** The first dash-launched run (P-e·h0, lag 0.2, from `stand`) went into a
"bad pose" that STOP could only freeze. Later, after the run was ended, "something is throwing the
hip1 joints to their rightmost extremes"; the operator cut HAT power.

**What the record shows** (benchd_20261003_214658.jsonl):
- **+234.6 s, 4 s after resume:** servo current jumped from ~1.3 A to 2.4–2.9 A during large, fast
  moves. The battery channel read 8.23 / 8.91 V, ADC garbage, the post-reset signature: the HAT's
  microcontroller had rebooted. There were no I²C errors.
- **+513.9 s:** a 3.02 A spike, INA bus voltage reading 9.39 V (garbage), then I²C NACKs. benchd's
  recovery reset the MCU every 20 bus errors, about 20 times in 4 s.
- **Mechanism:** a HAT MCU reset unprograms its servo timers. `ServoDriver` programmed a timer only
  when a channel was first ARMED, so every still-armed channel kept getting its pulse written into
  an unprogrammed timer. Garbage PWM drove the servos to their stops. The brain ran on that body
  from +234 s, the "bad pose".

**Fixed (6f7254b):** timers are re-programmed before any pulse whenever not ready (a test proves the
old driver failed this); a detected HAT reset disarms every channel (nothing written: the servos go
unpowered instead of being driven) and latches STOP in brain modes; benchd's MCU reset is limited
to one per 5 s; the dash announces it.

**Open, and not a software problem:** the brain's moves, from `stand` with the belly up, draw
2.4–3.0 A peaks, against the ~1.9 A budget measured 2026-09-13 for the HAT's 5 V / 3 A regulator.
The earlier 60 s belly-up run drew 1.2–1.6 A mean without a reset, so the margin is small and
depends on what the brain does. Options are hardware (a servo supply that is not the HAT's
regulator), a current-aware reflex in benchd (a safety envelope like the low-battery limp, not a
behaviour), or both.

### ★★★★ 2026-10-04 — MILESTONE: THE SIM-TRAINED BRAIN WALKS THE REAL PICRAWLER (operator: "we have validated sim2real")

**What runs.** P-e·h0, frozen gains, unchanged from the sim, on the Pi
(`ogma_host --imu --brain-inputs --actuate`). It is fed only what the robot can publish:
- joints from the servo forward model of the commanded pulse;
- heading, stride and distress from the IMU and odometry;
- foot contact and load from the FSRs;
- belly clearance from the boom ToF.

It drives 12 servos through benchd with the servo output lag at 0.2, and it is launched from
`picrawler-dash`.

**Operator verdict (2026-10-04, after watching P-e·h0 runs):** "P-e·h0 is working well … the gait is
working well with our FSRs and looks very much like the sim. Overall this is a big milestone and
we have validated sim2real." Earlier: "the lag/smoothing run looked good"; the 60 s stand-start run
kept the belly up the whole time.

**Measured parity (each a hardware signal, n=1–4 short runs; there is no distance sensor on the
robot, so gait distance and straightness are not measured):**

| quantity | sim (P-e·h0) | robot | entry |
|---|---|---|---|
| belly clearance, median | 14–26 mm | 31 mm (60 s, 97 % > 10 mm) | 2026-10-03 "60 s FROM stand" |
| hip2 centre | +0.04 rad | +0.02…+0.04 rad (from `stand`) | "START POSE IS THE ROBOT'S hip2 CENTRE" |
| knee centre | −0.71 rad | −0.65…−0.78 rad | same |
| joint speed vs command | 0.40–0.56 | 0.30–0.36 at lag 0.2 | "SERVO OUTPUT LAG α 0.2" |
| attitude filter | — | within 0.35° of accel after every run (225 Hz) | "ogma_host's IMU on a 225 Hz thread" |
| posture over a run | steady | tilt median 10.8° → 4–5° within 60 s | "60 s FROM stand" |

**Known gaps, each with its entry:**
- the robot stands 15–25 mm taller than the sim at the same angles (stiff servos against the sim's
  soft stance PD);
- it cannot yet stand up from the floor: from the rescue pose hip2 parks at −0.7 rad, because the
  sim always spawns standing;
- the brain sometimes pins a hip1 at its limit, and the robot circles.

**The blocker for agility is power, not the brain.** Agile moves draw 2.4–3.0 A peaks on the servo
branch, against a ~1.9 A budget on the HAT's 5 V / 3 A regulator, and the HAT's MCU resets: about 1
every 10–40 s in the runs of 2026-10-03. That is now fail-safe and recovered:
- servos disarm instead of being driven to their stops;
- they re-arm one at a time and ramp to the start pose;
- the brain resumes with its learning intact, in about 3 s.

But every reset is a 3 s interruption. Options:
- (1) a dedicated servo supply, which removes the cause;
- (2) a current envelope in benchd, a safety limit with A/B;
- (3) servo current as an interoceptive brain input — the doctrine's "reflex plus a learned layer
  that keeps it quiet", a real lever for later.

**Scale of the claim.** "Sim2real validated" here is the operator's direct observation of behaviour
plus the parity table — a strong signal at this stage, not a seed-averaged finding. The finding-level
version needs a distance/heading measure on the robot and n ≥ 20 runs, which power currently caps.

### ★★ 2026-10-04 — POWER BUDGET S0: SERVO CURRENT FOLLOWS LOAD, NOT MOTION (instrument built; analysis of the 2026-10-03 runs)

**Built (instrument only, consumed by nothing):** benchd puts servo-branch current (`i_a`, INA219,
~68 ms average) on its 50 Hz state feed; ogma_host publishes it as `sense.servo_current` and dumps
it with the brain inputs. It is under sense.*, not reality.proprio.*, because NeurochemState and
WhiskerAversionReflex subscribe to that whole prefix.

**Analysis** (`brainrun/current_analysis.py`, `current_by_foot.py`; 7116 brain-driven 10 Hz frames,
12 benchd records, a mix of configs and lag settings):

| correlate of current | r |
|---|---|
| feet in contact (FSR > 200) | **+0.40** |
| FSR sum | **+0.39** |
| per foot: phys FR / RL / FL / RR | +0.38 / +0.33 / +0.25 / +0.03 |
| channels moving ≥ 60 µs per frame | +0.12 |
| total slew | +0.09 |
| commanded past the range limit | +0.07 |
| "stalled" (at limit, not moving) | −0.07 |

| feet loaded | current mean | p95 |
|---|---|---|
| 0–1 (belly down) | 0.61–0.67 A | 1.4–1.9 A |
| **2–3** | **1.44–1.48 A** | **2.87–2.88 A** |
| 4 | 1.24 A | 2.37 A |

**Reading.** The brownout-zone current is servos HOLDING THE BODY on two or three legs, a stall
against the ground, not legs moving fast or pressing past their calibrated range. Four feet
sharing the load draw less. Per-leg attribution is available without per-servo current: the FSRs
say which feet carry the weight. Consequences for the plan:
- (S1) the sim's current model should be torque/load-driven, and the table above is its parity
  target;
- (S2) a reflex that reverses motion would not touch load-driven current. What lowers it is more
  feet sharing the load or a lower stance;
- (S3) "one leg at a time" keeps three feet loaded, which this table puts in the high band, so the
  budget prior should be allowed to find load sharing (four-foot support, lower stance), not one
  particular gait.

**Caveats:**
- 10 Hz frames with a 68 ms current average: reversal thrash at 50 Hz is invisible here;
- correlations are modest (load explains ~16 % of variance);
- the RR FSR reads a high, near-constant count (creep), which flattens its correlation;
- runs mixed configs and lag on/off.

The 50 Hz feed fixes the first caveat for new runs.

**Also found:** the ICM-20948 IMU stopped answering on SPI (WHO_AM_I and PWR_MGMT_1 read 0x00 in
both SPI modes, even after a reset command). It answered at the 13:39 boot and intermittently
failed restarts on 2026-10-03, so a loose connector or cable on the HAT's 7-pin SPI header is the
lead. benchd now retries the probe 4 times; that does not help a silent chip. Without the IMU,
dash runs are refused (tilt guard blind) and ogma_host cannot run brain inputs.

### ★ 2026-10-04 — STALL PROBE #1 (FR leg, on the stand): CURRENT SEPARATES A STALL CLEANLY; AUDIO DOES NOT, YET

> **⚠ CORRECTED the same day — the audio half of this entry measured the analysis, not the robot.**
> Its alignment came from one broadband onset (+85 ms, 1 of 9 moves). That put the labels of short
> moves AFTER the moves (a 100 µs move lasts ~50 ms), so "free_motion" was largely mislabelled.
> The corrected numbers are in the entry below. The current half stands.

**Probe:** `stall_probe.py --leg FR`: 27 free single-joint moves (100/200/400 µs) and 6 pushes into
the operator's block (≤ 1 s), 124 s of audio on the same clock as benchd's 50 Hz feed. Analysis:
`brainrun/stall_audio_analysis.py`, whose audio features are an exact twin of FrozenSTFTEncoder
over the 1024-sample windows ogma_host hands the EPM.

| class | level | servo current mean / peak | energy <1k / 1–4k / 4–8k kHz |
|---|---|---|---|
| silence | −45.6 dBFS | 0.27 / 1.24 A | 0.76 / 0.15 / 0.08 |
| free hold | −41.9 | 0.10 / 0.16 | 0.51 / 0.25 / 0.22 |
| free motion | −37.2 | 0.10 / 0.18 | 0.43 / 0.14 / **0.40** |
| blocked push | −42.3 | **0.68 / 1.30** | 0.62 / 0.22 / 0.15 |

- **Current is the clean stall witness:** a blocked push draws 7× the free-motion mean.
- **Audio, as the operator predicted:** a moving servo whines in 4–8 kHz, and a stalled one is
  quieter (hip1/hip2 motion −35 dB, blocked −41/−45 dB). But the knee's free motion was nearly
  inaudible (−49 dB).
- **Through the brain's encoder the classes barely separate:** free-motion vs blocked centroid
  cosine 0.971; nearest-centroid recall 0.35 / 0.18 (chance 0.25), whole events held out.

**Why this is not a verdict on audio (§3.1):**
- the moves were 50–200 ms single-joint taps on an unloaded leg, unlike the brain's continuous
  12-servo motion under load (operator: "the short movements may not be sufficient for the type
  of movements caused by the brain");
- only one move passed the onset detector, so the +85 ms alignment is weak;
- the "silence" class picked up handling noise around the blocked trials (its 1.24 A peak);
- mic placement and capture gain were not set.

Re-use context: audio recorded during real brain motion.


### ★★ 2026-10-04 — STALL PROBE #1 RE-ANALYSED: A MOVING SERVO WHINES IN 4–8 kHz, A STALLED ONE SOUNDS LIKE SILENCE

**Alignment, measured properly:** a moving servo raises the 4–8 kHz band from −76 dB (the room) to
about −35 dB, ~40 dB, for exactly the move's duration. Onset +20 ms after the commanded start
(IQR 20–30 ms), from the 18 moves that start from quiet.

| class | level | 4–8 kHz share | current mean / peak |
|---|---|---|---|
| silence | −44.2 dBFS | 0.24 | 0.27 / 1.24 A |
| free hold | −43.3 | 0.07 | 0.10 / 0.16 |
| free motion | −36.0 | **0.46** | 0.10 / 0.18 |
| blocked push | −42.9 | **0.08** | **0.68 / 1.30** |

- **The operator's prediction holds for hip1 and hip2.** Moving: −33 / −35 dB with the whine.
  Blocked: −42 / −46 dB without it.
- **The knee breaks it:** −48.5 dB moving, near the floor, so a quiet knee is not a stalled knee.
  Its mechanical noise barely reaches the mic.
- **Through the brain's encoder:** a blocked push is most often classified as SILENCE (190 / 318).
  On audio alone a stall is indistinguishable from rest; only the motor intent separates them. Given
  a commanded move, moving vs stalled scores 0.66 per window (blocked 0.86, free motion 0.48: the
  quiet knee and move tails). At event level with window means it is chance (31/60).

**So:** audio is a real but partial stall witness. It works where the joint is audible, it needs the
intent (the sensorimotor contingency, as proposed), and it is weakest exactly where current is
also weakest (the knee: 0.27 A blocked). Current remains the clean witness.

**Open:**
- these were single-joint taps on the stand; run recordings now capture brain motion;
- mic placement relative to the knees;
- the silence class still includes handling noise around the blocked trials.

### ★★ 2026-10-04 — TWO RECORDED BRAIN RUNS: IN REAL GAIT THE SERVOS NEVER GO QUIET, SO "STALLED = SILENT" HAS NOTHING TO CONTRAST WITH

**Runs:** P-e·h0 from picrawler-dash, battery, 317 s and 321 s of brain control, audio and the 50 Hz
feed recorded (`brainrun/run_audio_analysis.py`).

**Alignment works because it was measured.** arecord's first sample lands ~383 ms after the request,
which an assumed latency would have missed by ~0.4 s. The six sync taps rise a median 26–30 dB in
4–8 kHz at +20 ms.

**Audio in gait:**
- commanded motion is non-zero in 99.98 % of brain-driven frames (mean ~170 µs per 20 ms frame
  across the 12 channels: the brain's command thrash);
- the 4–8 kHz whine sits at −31…−34 dB the whole time, against −76 dB in the quiet room (probe #1);
- commanded motion predicts it at r = +0.04…+0.06 (best lag +90…+130 ms), with no joint type
  standing out (|r| ≤ 0.06);
- the whine barely changes between "some" and "a lot" of commanded motion (−32.8 vs −31.8 dB).

A stalled servo among eleven whining ones cannot be heard as silence. The single-servo probe cue
does not survive brain gait in this form. This is a statement about THIS gait, which moves almost
every servo almost every frame; a calmer gait (smaller, fewer simultaneous moves, which the power
budget wants anyway) would restore contrast. Untested: per-servo pitch differences within the
128-band spectrum.

**Current vs feet loaded at 50 Hz confirms the 10 Hz table:**

| feet loaded | run 1 mean / p95 | run 2 mean / p95 |
|---|---|---|
| 0 | 0.52 / 1.23 A | 0.40 / 0.71 A |
| 1 | 0.93 / 2.37 | 1.00 / 2.24 |
| 2 | **1.56 / 2.88** | **1.72 / 2.98** |
| 3 | 1.53 / 2.82 | 1.64 / 2.82 |
| 4 | 1.33 / 2.37 | 1.45 / 2.58 |

**Events:**
- three HAT resets, each auto-recovered to `stand` in 2.2–2.5 s. The second before each: current
  peak 2.4–3.0 A, 2.1–3.4 feet loaded, 155–180 µs/frame of commanded motion;
- run 1's roll past 80° (the tilt guard read 141°) followed the most vigorous second of either run:
  current mean 2.07 A (peak 3.04), 2.2 feet loaded, 203 µs/frame.

**Reading:** the brain's continuous all-servo motion is one root behind three of today's problems:
- it drives the 2–3-feet-loaded current peaks;
- it drowns the audio stall cue;
- the roll came after its most vigorous second.

The power-budget prior (S3) and the jitter levers (2026-10-02) target the same thing from two sides.

### ★★★ 2026-10-05 — FAST CURRENT DURING A 19-MIN RUN: THE HAT RESETS ARE NOT TRIGGERED BY CURRENT — AND SOME MAY NOT BE RESETS

> **⚠ CORRECTED the same day (next entry):** the "garbage" battery reads were the HAT's 3.3 V rail
> drooping, not corrupted transactions. The resets are real brownouts. The current finding stands:
> battery-side current does not predict them. The "some may not be resets" reading does not.

**Run:** P-e·h0 from picrawler-dash, 1154 s of brain control, including the operator's textured
mat. The sag capture (current + pack voltage, ~940 Hz) covered the first 900 s; the cap is now an
hour. 8 HAT resets in the record, 4 inside the capture. Lifted periods excluded by belly ToF
> 100 mm (4.3 % of frames). Analysis: `brainrun/fast_current_analysis.py`.

**Current is not the trigger at ~1 kHz:**
- in the 500 ms before each captured reset the current peaked at 3.03 / 1.65 / 3.34 / 3.36 A;
  before #2 it never passed 1.65 A, and before #1 it fell to 0.7–1 A for the last 100 ms;
- in 806 s of gait away from resets: 939 excursions above 3 A (70 per minute, median 40 ms, max
  380 ms) with no reset; the current never reached 4 A (max 3.58);
- the pack never sagged: 10 ms minimum 6.84 V (p0.1 6.89);
- the operator's "catch-up" after each recovery, servos slewing at the cap toward the brain's
  targets, peaked at 3.04–3.45 A and spent up to 415 ms above 3 A without a reset.

**The capture did not break at the resets.** The INA219 kept sampling through all four, so whatever
reset (or seemed to reset) the HAT's MCU did not take the INA219 down. Around #3 and #4 the INA219's
bus-voltage readings went IMPLAUSIBLE (8.7–9.5 V from a ~7.1 V pack) for 80–420 ms. That is
two different chips on the same I²C bus returning garbage at the same time.

**What triggered each "reset":**
- 5 of 8 (#1, #2, #3, #6, #8): a single implausible battery read (ADC "garbage", 9.07–11.19 V). For
  #1, #2 and #6 there was no I²C error anywhere in the 2 s before.
- 3 of 8 (#4, #5, #7): benchd's OWN MCU reset after bus errors. ⚠ That rule fired at every 20th
  CUMULATIVE bus error (rate-limited to 1 per 5 s). The run accumulated ~600 NACKs, so sporadic,
  survivable errors added up to deliberate resets. That is self-inflicted.

**Reading:** the evidence points at I²C read corruption (electrical noise or a shared
supply/ground disturbance during servo activity), with benchd's detector and its recovery turning
glitches into 3 s interruptions. It does not point at current overload. It is not proven that no
real MCU reset happened today; on 2026-10-03 real ones drove servos to their stops. The
discriminating fixes:
- confirm a "reset" before disarming (garbage persisting across reads);
- reset the MCU only on a burst of errors with no successful transaction;

then A/B resets per minute.

**For servo speed:** this run does not rule out a faster slew on power grounds (no current-related
reset; catch-up at the slew cap was fine). The sim-first rule and the earlier speed table
(2026-09-13: 4.59 rad/s was the best-behaved setting on an older config) still apply.


### ★★★ 2026-10-05 (evening) — THE HAT's 3.3 V RAIL IS BROWNING OUT; BATTERY-SIDE CURRENT IS CAPPED, SO IT CANNOT SEE WHY

**Run:** P-e·h0, 418 s of brain control, with reset confirmation and the burst-only MCU reset
deployed (0d452653), the fast capture covering the whole run. **9 resets (1.15 / min, against 0.4 / min
in the 19-min run)**: 6 confirmed by re-reads, 3 by benchd's burst rule (57–64 errors in 1 s, nothing
through for ≥ 500 ms). 4 garbage reads were unconfirmed and logged as `hat_glitch`; before the fix,
each would have cost a ~3 s recovery.

**The "garbage" is a measurement.** The HAT's ADC measures the battery divider against its own 3.3 V
rail. The garbage values climb smoothly as that reference falls, rather than scattering:
- confirmed resets: first read 9.59, re-reads 9.07 / 9.11 / 9.22; first read 9.10, re-reads
  9.00 / 9.13 / 9.31;
- pinned at the top of the range: 11.19 ×3;
- no answer at all: None ×3;
- unconfirmed glitches: 8.2–9.5 V recovering to 7.7.

A true pack of ~7.4 V read as 9.2 V implies a 3.3 V rail near 2.65 V, where an MCU browns out. In two
resets (#5, #7) the INA219, powered from the same rail, read 0 V: it died too. The 10 Hz telemetry
estimate of the rail (3.3 × INA pack V ÷ A4 reading) sits at 3.31 V median, with p1 3.12 and p0.1
2.79. It droops, briefly, mostly too fast for 10 Hz, and benchd's frame replaces any reading above
9 V with the last good one, so the deepest dips are filtered out of that record.

**Why battery current cannot see it.** Battery-side current hits a hard ceiling in every run:
- maximum 3.53–3.58 A, never above 4 A;
- above 3 A 70–75 times a minute (median 40 ms, up to 1.4 s) with no reset;
- before resets, a peak of only 2.6–3.5 A.

A flat ceiling is what a current-limited supply produces, so this looks like the HAT's 5 V regulator
at its limit. Demand above it shows up as 5 V rail sag, not as more input current, and a deep enough
sag drops the 3.3 V rail out. Hypothesis, consistent with every number so far.

**Consequences:**
- no battery-side current threshold can predict a reset;
- the 3.3 V rail itself is the precursor worth instrumenting (added the same day, next entry);
- faster servos would likely add demand above the limit (hold);
- the real fix is to move the servo load off the HAT regulator. The PCB cannot be modified, so the
  proposal is a servo power-injection harness: an extension per servo with its V+ cut on the HAT
  side and fed from a separate high-current BEC, ground common, signal untouched.

### ★★★ 2026-10-05 (night) — THE 3.3 V RAIL IS THE GRADED PRECURSOR: 444 DIPS, 15 BELOW 3.0 V, ONE FATAL AT 2.51 V

**Run:** P-e·h0, 599 s of brain control, with the rail estimate (3.3 × INA pack V ÷ A4) in the fast
capture at 232 Hz. **1 HAT reset (0.1 / min, against 1.15 / min the run before and 0.4 / min the run
before that)**, 1 hat_glitch; ended by the tilt guard at 85°. The reset rate varies by an order of
magnitude between runs; nothing here says why (battery, surface or brain behaviour), so treat it as
a property of the run, not of the build.

**The rail, in gait:** median 3.31 V, p1 3.23, p0.1 3.05; below 3.0 V 0.06 % of 126 628 samples.

| dip minimum | count in 10 min | outcome |
|---|---|---|
| < 3.15 V | 444 | all recovered |
| < 3.0 V | 15 | all recovered |
| < 2.8 V | 6 | 5 recovered (to 2.65–2.68 V, 22–91 ms); 1 = the hat_glitch (2.65 V, 85 ms) |
| **2.51 V**, 68 ms | 1 | **RESET** |

- **The cliff is fast:** before the reset, the rail sat at 3.28–3.30 V until ~60 ms before benchd
  detected it, then went 3.24 → 3.18 → 2.72 → 2.58 V (20 ms minima). Dips last 20–90 ms.
- **The MCU survives ~2.65 V and dies by ~2.5 V.**
- **Every deep dip came with battery current at its ceiling** (3.04–3.42 A within ±50 ms). That
  supports the regulator-limit reading: being at the current cap is necessary for a dip, the depth
  is what varies, and the INA219 cannot see the depth. The rail can.

**Why this matters:**
- **For the brain (S3):** the rail's dip depth is a GRADED, egocentric signal that rises toward the
  fatal event: 444 small dips for every fatal one, the gradient the memory "fatal events have no
  gradient" asks for. It is a better power-budget term than current, which saturates at the cap.
- **For a reflex (S2):** a dip below ~2.9 V is a candidate trigger to unload briefly, but the window
  is 20–60 ms against a 20 ms tick. Untested.
- **For the injection harness:** a clean before/after. With the servo load off the HAT regulator, the
  dips should vanish.

### ★★ 2026-10-05 (night) — SURFACE A/B: CONCRETE DRIVES THE RAIL DEEPER AND RESETS THE HAT ~13× MORE OFTEN THAN CARPET

**Runs:** the same build and brain (P-e·h0, autonomous, lag 0.2), on smooth carpet (599 s) and on
concrete (283 s). Fast capture with the rail estimate in both. n = 1 per surface: a signal.

| | carpet | concrete |
|---|---|---|
| HAT resets | 1 (0.10 / min) | 6 (1.27 / min) |
| battery current above 3 A | 87 / min | 150 / min |
| 10 ms peak current, p50 / p90 | 1.56 / 2.76 A | 1.79 / 3.00 A |
| rail dips < 3.0 V | 1.5 / min | 6.1 / min |
| rail dips < 2.8 V | 0.6 / min | 4.5 / min |
| rail p0.1 in gait | 3.05 V | 2.88 V |
| current ceiling (max) | 3.46 A | 3.50 A |

- **Operator's reading:** concrete "seemed to increase inertial stress on servos holding position".
  The data agree. On a hard, unyielding surface the same gait spends nearly twice as long at the
  current ceiling, and the rail dips deeper and far more often. Carpet's compliance absorbs what the
  servos otherwise have to fight.
- **The reset mechanism is unchanged:** every concrete reset follows the same cliff, flat at ~3.28 V,
  then down to 2.45–2.61 V within 20–60 ms. The deepest "dips" (2.0 V for ~1.2 s) are the outages
  themselves. One reset (#5) was preceded by survivable dips to 2.56 / 2.51 / 2.88 V in the 200 ms
  before, the only sign of a run-up in any reset so far.
- **The reset rate is a function of surface.** That explains most of the earlier spread (0.10–1.27 / min).
  The 1.15 / min run of the same evening was not labelled; if it was on a hard floor, it fits.

**Consequences:**
- any hardware A/B of resets per minute must hold the surface fixed;
- concrete is the stress test for the injection harness;
- a brain-side power term could learn to stand and step more softly on hard ground: the rail dips
  give it the gradient.

### ★★ 2026-10-05 (night) — SIM POWER MODEL S1, FIRST PASS: the sim servo has a 9.6 N m ceiling; an honest datasheet cap breaks the gait, and the robot steps like the strong servo

> **⚠ CONTEXT CORRECTION (2026-10-06): every sim run in this entry ran on the DEFAULT body, not
> P-e·h0's.** `arenaavg.py` (and the direct launches) never applied the config's `metadata.body_env`;
> only the launcher did. So these runs had servo speed 6.0 (not 3.668), a ghost chassis, and raw
> joints, IMU, upright and distress. The arms are consistent with each other, so the comparisons stand
> as comparisons, but on a different body. Re-measured on the faithful body: entry "BODY_ENV
> CONFOUND" below.

**Instrument (`WORKING`, byte-identical).** `scripts/servo_power_model.gd` recovers each servo's
motor torque from Newton-Euler on the segments beyond it: angular-momentum change about the moving
anchor, minus gravity, contacts on every leg segment and body damping, projected on the joint axis.
Godot Physics 3D does not expose a joint motor's impulse, so this is the only route.
`OGMA_PICRAWLER_POWER_LOG=<csv>` writes it at 240 Hz. A 3000-tick trace is byte-identical with the
instrument on and off. Static stand reads hip2 0.12–0.15 N m and knee 0.07 N m, which matches
statics (body weight ÷ 4 × the horizontal lever).

**Finding: the "0.15 N m" servo has really had a 9.6 N m ceiling.** Godot 4.6's
`GodotHingeJoint3D::solve()` clamps the motor impulse on every solver iteration and keeps no
accumulated total, and the space runs `solver_iterations` = 64 (read back from the server). The
instrument confirms it: P-e·h0 stance hip2 carries 0.18 N m on average, joints sit past an MG90S's
datasheet stall (0.18 N m) ~25 % of the time, and the tail runs smoothly to 2 N m with no ceiling.

**Lever: `OGMA_PICRAWLER_HONEST_TORQUE_CAP=<N m>`.** It divides the per-call impulse by the iteration
count, so one physics step can never exceed the cap (hinge backend; gain-0 verified byte-identical).
P-e·h0, arena 0.3, n=6 × 12000, seeds 1–6, `arenaavg.py`:

| cap | net_disp | steps | contact duty | belly (mm) | belly min | tilt_sd | falls |
|---|---|---|---|---|---|---|---|
| legacy (9.6) | 10.42 ± 1.07 | 84.5 ± 24.4 | 0.80 | 39.2 | 14.5 | 0.091 | 0 |
| 0.35 | 10.02 ± 2.01 | 32.3 ± 21.3 | 0.81 | 35.8 | 10.6 | 0.108 | 0.17 |
| 0.25 | 10.21 ± 1.66 | 12.0 ± 6.5 | 0.85 | 32.9 | 2.8 | 0.067 | 0 |
| 0.18 (MG90S datasheet) | 8.20 ± 2.03 | 6.8 ± 8.4 | 0.92 | 27.8 | 3.1 | 0.072 | 0 |

**Verdict: `REGRESSION` (signal, n=6), monotonic in the cap.** A weaker servo plants its feet
(duty 0.80 → 0.92), stops stepping, and slides with the belly scraping (min 3 mm). It still drifts
8 m. The context matters: the brain's frozen gains were settled on the 9.6 N m body.

**The robot says the strong servo is closer to the truth.** From its FSRs (> 200, brain-driven, on
the floor), the robot runs at contact duty 0.70, 200–270 swings ≥ 4 ticks per minute, and swing
median 10–13 ticks. The legacy sim runs at 0.80, ~270 per minute and 6 ticks. The robot steps at
least as much as the uncapped sim, and every cap moves the sim away from it.

**Current model (`pi_host/tools/brainrun/power_calib.py`).** Per servo, I = idle + running +
min(I_stall, K · g · |τ|). g is 1 when the servo does work and H_HOLD when holding or back-driven
(gear friction carries part of the load). τ comes from the instrument. With datasheet K
(0.75 A ÷ 0.18 N m) and any H, the sim predicts 2.3–2.9 A battery-side against the robot's 1.55
(carpet). Fitting K alone to the robot's mean gives 2.1–2.5 A/N m. At the stall-probe current
(0.75 A), that is an effective stall torque of ~0.3–0.36 N m. The cap dose curve independently
needs > 0.35 N m to step. **Two estimates agree that the robot's servos act about twice as strong as
the MG90S datasheet, or that its stance loads are about half the sim's.** The robot stands taller
than the sim, which would point at the second.

**What does not fit yet: the shape.** With the mean matched, the sim's 10 ms peaks are too narrow:

| | p90 | p99 | above 3 A |
|---|---|---|---|
| sim | 2.0 A | 2.4 A | ~0 / min |
| robot | 2.76 A | 3.26 A | 87 / min |

So the sim never reaches the regulator limit, and the rail model cannot be fit. The robot's extra
variance is at gait timescale: 85 % of its current spectrum is below 5 Hz, and there is no 50 Hz
line, so a synchronized servo drive pulse is refuted as the cause. Two ideas remain:
- the robot's longer swings concentrate the load on fewer legs;
- a hobby servo's deadband: near-free inside it, a stall current once a load pushes it out.

**Also reverted the same session:** mirroring the robot's servo output lag (α 0.2) into the sim.
That lag was chosen to give the robot the plant lag the sim ALREADY has (entry "SERVO OUTPUT LAG
α 0.2 ON THE ROBOT"), so adding it to the sim would have lagged it twice.

**Re-use context for the cap:**
- a MEASURED stall torque: one MG90S at 5 V with a weight on a known arm (operator bench test);
- geometry and stand-height parity first, if the loads are what differ;
- a brain retuned on a capped body.

The cap stays default-off. The power model's next step is the shape, not the rail.

### ★★ 2026-10-05 (late) — SCALE PROBE: an MG90S holds a load almost for free, drives into one at ~1 A, and cuts its own drive after ~3.3 s

**Probe:** `scale_probe.py --leg FL` (ch 4), robot on its stand, operator poses `toes_up` →
`torque_check` (leg straight, knee ~2 mm above a kitchen scale). hip2 pushes in 40 µs steps past
that point; each push is held ~7 s, then the leg rests 2 s. Current is benchd's 50 Hz INA219 (whole
HAT, battery side, base ~0.10 A). Operator: the scale reading **peaked, then settled**, and the
SETTLED value was entered. Separately, back-driving a held servo by hand took ~1.2 kg on the same
arm (~0.63 N m).

| push past contact | settled reading | settled torque (53.6 mm arm) | current while driving (0.25–3.3 s) | current settled (4.5–7 s) |
|---|---|---|---|---|
| 40 µs | 130 g | 0.07 N m | 0.11 A (= base) | 0.11 A |
| 80 µs | 228 g | 0.12 N m | 0.10 A (= base) | 0.10 A |
| 120 µs | 320 g | 0.17 N m | 0.83–0.93 A | 0.21 A |
| 160 µs | 387 g | 0.20 N m | 0.79–0.88 A | 0.21 A |
| 200 µs | 475 g | 0.25 N m | 0.95–1.08 A | 0.21 A |
| 240–440 µs | 475–480 g | 0.25 N m | 0.73–0.84 A | 0.20–0.21 A |

The commanded pulse was constant through every hold (benchd's `out` = target, all other channels
unchanged), so everything below happened inside the servo.

- **Holding is nearly free.** At 40–80 µs the leg reached its target (stand and scale compliance
  absorbed the few mm) and held 0.07–0.12 N m with no current above base. After the drive phase, it
  held 0.25 N m at 0.11 A. Gear friction carries a static load: the back-drive force is ~0.63 N m.
- **Driving into a load is a step to ~0.7–1.0 A** (battery side, one servo) once the target is out of
  reach. That level barely depends on how far out of reach (120–440 µs): the servo saturates.
- **The servo cuts its own drive after ~3.3 s.** At every push from 120 µs on, the current falls from
  ~0.8–1.0 A to 0.21 A between 3.3 and 4.5 s, on the same timing every time, with the pulse
  unchanged. A thermal fold-back would not reset in the 2 s rest, so this looks like the servo IC's
  stall protection. The scale's "peak then settle" is the force following that current.
- **Not measured: the drive torque.** The peak force during the drive phase was not entered, so
  amps per N m while driving is still open. The settled 475 g plateau over 200–440 µs may be a
  holding limit, or the robot lifting on its stand; which one is unknown.

**What it changes:**
- **The sim's current model (S1).** A linear current-per-torque model is the wrong shape. A real
  servo is close to bimodal: ~0 A while holding, even at 0.25 N m, and ~1 A while driving against a
  load. That is exactly why the robot's current swings at gait timescale and its mean sits below
  the linear model's: the sim's stance torques are mostly holding. The next model is two-state
  (holding at H_HOLD ≈ 0, driving at a saturated level), with the 3.3 s stall cut-off as a third.
- **The honest torque cap.** A symmetric 0.18 N m cap is wrong in both directions. The real servo
  resists back-driving up to ~0.63 N m and drives at ≥ 0.25 N m. A cap that gives way at 0.18 under a
  stance load models a servo that does not exist, which explains part of the dose-curve `REGRESSION`.
  The faithful cap is asymmetric: drive ≥ 0.25 N m, hold ~0.6 N m.
- **The robot's stuck leg.** A snagged leg draws stall current for ~3.3 s and then limits itself.
  Rail dips from a stall are bounded in time by the servo itself.

**Re-use / next:**
- re-run with the scale display FILMED, aligned on the 12 pushes, for the peak (drive) torque per step;
- one more servo, to see whether the 3.3 s cut-off is this unit or the model;
- note whether the robot rises off its stand at the plateau.

**Cross-check against SunFounder's published sibling servos (2026-10-05).** No datasheet exists for the
PiCrawler's own servo (servodatabase lists "PiCrawel", A0000284, with empty fields). The closest
published siblings are the same 13.5 g SunFounder micro servo. Their specs match the scale probe:

| | SF006PRO (AI Lab kit doc) | SF006FM (PiDog) | scale probe (FL hip2) |
|---|---|---|---|
| drive / dynamic load | ≥ 2.2 kgf·cm (0.22 N m) at 5 V | ≥ 1.3–1.4 kgf·cm max | 475 g plateau = 0.25 N m |
| static stall (holding) | ≥ 5 kgf·cm (0.49 N m) | — | back-drive ~1.2 kg = ~0.63 N m |
| stall current | ≤ 1.2 A at 5 V | ≤ 0.85 A (4.8 V) / 1.0 A (6 V) | ~0.9 A battery side ≈ 1.1–1.2 A at 5 V |
| stall cut-off | drops to ≤ 250 mA after 5 s | "power failure protection after 5 s" | drops to ~0.1 A after 3.3 s |
| deadband | ≤ 6 µs | ≤ 3 µs | — |
| no-load speed | ≤ 0.17 s/60° (6.2 rad/s) | ≤ 0.18 s/60° | — |
| clutch | — | slips under external force | operator saw the servo "shifting against its own hinge" |

So the sim's servo should drive at ~0.22–0.25 N m and hold to ~0.5–0.6 N m, draw ~1.2 A at 5 V when
driving into a load, and limit itself to ≤ 0.25 A after a few seconds of stall.

### ★ 2026-10-05 (late) — ASYMMETRIC TORQUE CAP (drive 0.25 / hold 0.6 N m): ties legacy on distance, still a quarter of the steps (`PARTIAL`, signal n=6)

> **⚠ CONTEXT CORRECTION (2026-10-06): every sim run in this entry ran on the DEFAULT body, not
> P-e·h0's.** `arenaavg.py` (and the direct launches) never applied the config's `metadata.body_env`;
> only the launcher did. So these runs had servo speed 6.0 (not 3.668), a ghost chassis, and raw
> joints, IMU, upright and distress. The arms are consistent with each other, so the comparisons stand
> as comparisons, but on a different body. Re-measured on the faithful body: entry "BODY_ENV
> CONFOUND" below.

**Lever:** `OGMA_PICRAWLER_HONEST_HOLD_CAP=<N m>`, on top of `HONEST_TORQUE_CAP`. Gear friction is modelled
as a passive brake. Every physics step, a joint pushed back against its command (or told to hold
still) gets target velocity 0 at the hold cap. A braked joint drives again only when the torque it
is holding against, in the commanded direction (read from the power instrument), is below the
drive cap. Values from the scale probe and SunFounder's SF006PRO (dynamic ≥ 0.22, static
≥ 0.49 N m). Gain-0: legacy and the symmetric cap are both byte-identical with it at 0. The brake
engaged on 13.5–14.1 % of joint-steps in every seed.

**First build: `REGRESSION`, a faithfulness bug, not a verdict.** It released the brake every step.
Under a stance load between the caps, each drive step slipped, and the brake stopped the joint but
never recovered the lost ground. The stance crept down: belly 19.8 mm, net_disp 8.61. Real static
friction holds without creep, which is why the release now needs the load test.

P-e·h0, arena 0.3, n=6 × 12000, seeds 1–6, `arenaavg.py`:

| | legacy (9.6 N m) | symmetric 0.25 | **asymmetric 0.25 / 0.6** | symmetric 0.18 |
|---|---|---|---|---|
| net_disp | 10.42 ± 1.07 | 10.21 ± 1.66 | **9.94 ± 0.07** | 8.20 ± 2.03 |
| straight | 0.69 | 0.67 | 0.69 | 0.71 |
| steps | 84.5 ± 24.4 | 12.0 ± 6.5 | **20.7 ± 13.3** | 6.8 ± 8.4 |
| step_bal | 0.22 | 0.00 | 0.13 | 0.04 |
| contact duty | 0.80 | 0.85 | 0.87 | 0.92 |
| belly (mm) | 39.2 | 32.9 | 33.0 | 27.8 |
| belly min (mm) | 14.5 | 2.8 | 4.2 | 3.1 |
| tilt_sd | 0.091 | 0.067 | 0.078 | 0.072 |
| falls | 0 | 0 | 0 | 0 |

**Verdict: `PARTIAL` (signal, n=6).**
- **Against legacy:** distance and heading tie (9.94 vs 10.42, straight 0.69 both), with no falls.
  But steps fall by three quarters and the belly drops 6 mm: the measured servo still plants its
  feet more than the 9.6 N m one.
- **Against the symmetric 0.25 cap:** the hold side adds little in this context. Belly is equal, the
  steps difference is inside noise (21 ± 13 vs 12 ± 6), and net_disp is much tighter across seeds
  (± 0.07 vs ± 1.66).
- **Context:** the brain's gains were frozen on the 9.6 N m body.

**The robot still steps more than any capped sim** (FSR duty 0.70, swings 10–13 ticks). So either the
drive cap is low, or the robot's stance loads are lower than the sim's. On the cap: 0.25 N m is the
SETTLED scale reading; the operator saw a higher peak that was not recorded, and the SF006PRO's
2.2 kgf·cm is a minimum. On the loads: the robot stands taller.

**Re-use / next:**
- the drive cap from the recorded PEAK (film the scale), then re-run this arm;
- stance-height parity, if the loads differ;
- this body for the S1 current fit (its torques are the faithful ones).

Default off.

### ★★ 2026-10-06 — BODY_ENV CONFOUND: the headless harness never loaded P-e·h0's body; re-measured, the asymmetric cap is a `REGRESSION` on the faithful body

**The confound (§3.2 #7, "did the arm you think you ran actually load?").** P-e·h0 declares its
robot-faithful body in `metadata.body_env`: solid chassis, honest joints / IMU / upright / distress,
the boom ToF, 3.668 rad/s, joint_torque_zero. Only `launcher.gd` applied it. `arenaavg.py`,
`seedavg.py` and direct `OGMA_PICRAWLER_CONFIG` launches did not. Every sim run of 2026-10-05
(power instrument, cap dose curve, asymmetric cap, the power fit) therefore ran on the default body:
6.0 rad/s, ghost chassis, raw joints. The run logs said so (`max_servo_speed = 6.000`), and nothing
read them. **Fixed:** both harnesses now apply `body_env` as the launcher does (an exported variable
or a `K=V` argument still wins), and `arenaavg.py` prints a body_env receipt per arm. **Worth
checking:** any earlier seedavg/arenaavg result for a config that declares `body_env`.

**Re-measured on the faithful body** (P-e·h0, arena 0.3, n=6 × 12000, seeds 1–6, receipt
printed on both arms):

| | legacy servo (9.6 N m) | asymmetric 0.25 / 0.6 N m |
|---|---|---|
| net_disp | 9.40 ± 0.91 | 7.38 ± 1.98 |
| straight | 0.74 | 0.59 ± 0.22 |
| steps | 26.3 ± 14.0 | 21.8 ± 17.2 |
| contact duty | 0.83 | 0.88 |
| belly (mm) | 29.9 | 22.4 |
| belly min (mm) | 3.9 | 0.2 |
| tilt_sd | 0.082 | 0.141 |
| falls | 0 | 0.33 |

**Verdict: `REGRESSION` (signal, n=6), on the faithful body.** The faithful body already steps far
less than the default one (26 vs 84.5), so the cap's step loss is small here (inside noise). What it
costs instead is posture and stability: the belly drops 8 mm and scrapes (min 0.2 mm), tilt nearly
doubles, straightness falls, and falls appear (0.33). Context: the brain's gains were settled on the
9.6 N m servo. **Re-use:** the drive cap from the recorded PEAK torque (the 0.25 used here is the
settled reading); a brain whose gains are settled on the capped body; stance-height parity.

**Power model refit on this body** (`power_calib.py`, the same constants in the sim): K 2.39 → 2.49
A/N m (the default-body fit transferred within 4 %). Carpet mean 1.55 A matched; shape still narrow
(p90 2.22 vs 2.76 A, p99 2.81 vs 3.26, > 3 A 8 vs 87 per minute). The sim port reproduces the Python
model to 1.4e-4 A over 9602 steps (`power_parity.py`). The rail fold-back branch is not exercised by
that check, because the model never reached the regulator limit.

**HUD:** windowed sim runs now show a POWER panel (bottom right; `[T]` hides it). It holds the bench
dashboard's `current_graph.gd` fed at the INA219's 10 Hz with its 68 ms averaging, the
picrawler-dash power and slow lines, the modelled pack and 3.3 V rail, and a joint line speed (10 Hz,
mean |Δangle|/s over 12 joints). picrawler-dash gained the same line speed (from benchd's 50 Hz
`out` pulses, over exact 100 ms spans) and the rail estimate. **Launch through the launcher**, so
P-e·h0's body_env applies; a direct scene launch gets the default body.

### ★★ 2026-10-10 — SIM vs ROBOT, SAME NUMBERS: the legs move at the same speed; current matches on the mean and is too calm in the sim; the robot steps more

**Runs:** the robot, P-e·h0 from picrawler-dash (autonomous, lag 0.2, carpet), 384 s brain-driven on
the floor, 1 HAT reset. The sim: the same config on its faithful body (body_env applied), arena 0.3,
3 seeds × 12000 per arm, with the capped servo (0.25 / 0.6 N m, what the operator watched) and the
legacy servo. `brainrun/sim_robot_compare.py` measures both sides the same way (robot: the pulse on
the line; sim: the joint itself; 100 ms spans).

| | robot | sim, capped | sim, legacy servo |
|---|---|---|---|
| joint line speed, mean (rad/s) | 1.12 | 1.31 | 1.25 |
| … p90 | 2.75 | 3.12 | 3.08 |
| … hip1 / hip2 / knee | 1.25 / **0.47** / 1.63 | 1.26 / 0.87 / 1.80 | 1.36 / 0.68 / 1.72 |
| current mean (A) | 1.55 | 1.58 | 1.59 |
| 10 ms peaks p90 / p99 (A) | 2.81 / 3.26 | 2.22 / 2.79 | 2.30 / 2.93 |
| above 3 A per minute | 67 | 8 | 15 |
| rail dips < 3.0 V per minute | 2.35 | 0 | 0 |
| contact duty | 0.67 | 0.89 | 0.85 |
| swings ≥ 4 ticks per minute | 231 | 129 | 156 |

- **Speed: the robot is no longer faster than the sim.** With the servo lag on, its legs move ~10–15 %
  SLOWER than the sim's on average (hip1 and knee within 10 %). The robot number is the pulse on the
  line, so its real joints can only be slower still. The earlier "faster than the sim" was the
  pre-lag robot (2026-10-03).
- **hip2 is the exception: the robot moves it about half as much** (0.47 vs 0.68–0.87). This is the
  brain's command, not the servo, so it is the brain behaving differently on the real body, and hip2
  is the joint that carries the stance.
- **Current: the mean holds on a run the model never saw** (1.55 robot vs 1.58–1.59 sim; K was
  fitted on 2026-10-05's carpet run). The shape does not: the robot's peaks are higher and it crosses
  3 A 4–8× as often. The sim never reaches the regulator limit, so its rail never dips.
- **Stepping: the robot lifts its feet far more** (duty 0.67 vs 0.85–0.89, swings 231 vs 129–156 per
  minute). FSR thresholds and creep make the robot side noisier, but the gap is large.

**Reading:** the sim matches the robot on how fast the legs move and on average power. It differs in
what the legs do: the robot steps more and works hip2 less. Its current spikes are bigger, which may
be the same fact (more lift-offs mean more moments with fewer feet carrying the load). The current
model's missing variance probably sits in the stepping gap, not in the servo model.

### ★ 2026-10-10 — P-e·h0 AT 4.59 AND 6.0 rad/s (sim): the gait steps 3× and 6× as often and the belly rises; heading and current pay (`PARTIAL`, signal n=6)

**Lever:** servo speed only (`OGMA_PICRAWLER_MAX_SERVO_SPEED`), same brain and gains. New configs
`…__nohomeo__spd459` (4.585 rad/s = 50 µs/tick on the robot) and `…__nohomeo__spd600` (6.0 = 65 µs/tick),
now in the pruned launcher/dash menu beside P-e·h0. On the robot, `dash_run` sets benchd's brain slew
from the config's declared speed for the run and restores it after. Arena 0.3, n=6 × 12000, seeds
1–6, faithful body (receipts: `max_servo_speed` 3.668 / 4.585 / 6.000 on every seed).

| | 3.668 (P-e·h0) | 4.585 | 6.0 |
|---|---|---|---|
| steps | 26.3 ± 14.0 | 75.3 ± 10.9 | 159.7 ± 14.9 |
| contact duty | 0.83 | 0.79 | 0.74 |
| belly (mm) | 29.9 | 33.6 | 37.9 |
| net_disp | 9.40 ± 0.91 | 9.97 ± 0.40 | 10.67 ± 0.92 |
| path_len | 12.6 | 14.8 | 19.3 |
| straight | 0.74 | 0.67 | 0.56 |
| scrub | 0.075 | 0.093 | 0.115 |
| tilt_sd | 0.082 | 0.080 | 0.100 |
| falls | 0 | 0.17 (1 / 6) | 0.17 (1 / 6) |
| modelled current, mean (A) | 1.62 | 1.66 | 1.82 |
| modelled 10 ms peaks above 3 A per min | 15 | 33 | 91 |

(Current: the S1 electrical model, 2 seeds per arm. It under-reads the robot's peaks ~4× at
3.668 — robot 67 / min — so read the column as a ratio: ~2× and ~6× the time near the ceiling.)

**Verdict: `PARTIAL` (signal, n=6).** Faster servos give the gait what 3.668 lost (2026-10-01): it
steps 3× and 6× as often, lifts the belly 4 and 8 mm, and moves its contact duty toward the robot's
(0.67 measured on carpet, 2026-10-10). It costs heading (straight 0.74 → 0.56: more path for the same
displacement), scrub, and one fall in six at each faster speed (12000-tick horizon only; see the
slowest-loop rule before calling either safe). The current model puts ~2× (4.59) and ~6× (6.0) as
much time near the HAT's current ceiling, which is where the rail dips and the resets come from.
**Operator's call:** try it on smooth floor / carpet. 4.59 is the smaller step; watch the dash's rail
line and the reset count against P-e·h0's 0.10–0.15 per minute on carpet.

---

### 2026-10-10 — S1 OF THE MICRODUCK PORT, SIM HALF: THE FORWARD ULTRASONIC MODEL AND THE ROOM; P-e·h0 WALKS INTO THE WALL AND STAYS THERE (n=6)

*Plan: [`../plans-and-designs/picrawler_microduck_port_plan.md`](../plans-and-designs/picrawler_microduck_port_plan.md)
§4 S1. Lever: `OGMA_PICRAWLER_ULTRASONIC=1` (a sensor model, instrument-only) and gym `room`
(`OGMA_PICRAWLER_GYM=room`, hotkey `[5]`, `OGMA_PICRAWLER_ROOM_SIZE` default 3.0). Gain-0 guard:
both off, the three deployed configs are byte-identical on body JSON at seed 1 × 6000 ticks
(the same gate that covered the master merge and the eyes). Harness:
`scripts_tools/roomavg.py`. Verdict: `BASELINE` — the scenario S3 will be measured against.*

**What was built.** The robot's forward HC-SR04-class module, in the sim, on the robot's own
topic and format: `sense.range` = `[distance_m, valid]`, 20 Hz off the 50 Hz tick, max 1.5 m,
commissioned floor 0.023 m; a no-echo ping reads 1.5 with `valid` 0, never 0. The transducer
pair's midpoint is 11 mm above the belly plane on the front face, level (operator: no downward
pitch in the mount). The ping is a 5 × 3 fan of rays inside the ~15° lobe, nearest echo wins,
and an echo returns only inside a 30° glancing limit (BOM §7: past ~30° off-normal the real
echo reflects away). A new `OgmaBrain.publish_token` lets the sim publish on an arbitrary
topic; `publish_proprio` hardwires the `reality.proprio.` prefix, and a channel meant to run
the robot's `picrawler_senses.json` unchanged must land on the robot's name. The room: a
3 × 3 m flat floor inside four vertical 0.3 m walls with three boxes, fixed layout, spawn at
the origin facing +Z. God's-eye instruments, never published: `us_true` (the axis range at
any incidence), `wall_contact_ticks/episodes` (chassis or a lower leg against a wall or box),
`room_cov` (0.25 m cells visited). HUD: a `range [bar] m valid/no echo · axis · pings` line.

**The baseline, P-e·h0 in the room, n=6 × 6000 ticks (seed 1–6):**

| metric | mean ± sd | per seed |
|---|---|---|
| first contact (tick) | 1210 ± 223 | 1200 1080 1560 900 1320 1200 |
| wall_frac (ticks against a wall / box) | 0.531 ± 0.096 | 0.59 0.53 0.35 0.60 0.53 0.58 |
| contact episodes | 126 ± 21 | 111 136 96 122 151 142 |
| coverage (0.25 m cells) | 12.7 ± 1.8 | 12 12 16 12 13 11 |
| net_disp / path / straight | 1.55 / 3.66 / 0.42 | — |
| falls | 0 | — |
| us_valid (rows with an echo) | 0.94 ± 0.08 | 0.99 0.93 0.78 1.00 0.97 0.99 |
| us_err, \|us_r − us_true\| on valid rows | 0.035 ± 0.029 m | 0.010 … 0.092 |
| confound rows (axis sees a surface, no echo) | 5.2 ± 8.1 | 1 6 21 0 3 0 |

**What it says.** Every seed does the same thing: the heading hold (the robot's loud
capability, ledger §1) drives it straight into the +Z wall at 1.5 m, first contact at tick
~1200, and then **holds it there for the rest of the run** — half of all ticks in contact,
~125 contact episodes as the gait bounces off and re-presses, coverage a dozen cells. Nothing
in the stack sees the wall before contact or leaves it after. This is the re-use context the
`stuck→explore` lever was refuted into ("terrain, corridor corners, obstacle contact",
ledger §1) — it exists now. The sensor saw the wall the whole way in: range tracks the axis
truth to 3.5 cm (the lobe's nearest return is shorter than the axis, as it should be), and
reads ~0.09 m when the body is pressed on the wall (the front legs reach ahead of the
chassis). Seed 3 drifted and turned along the wall: its echo fraction fell to 0.78 and it
logged 21 confound rows — the glancing rule firing as designed, a wall at an angle reading as
open floor. That confound is what S1's separation study must show the vocabulary can carry
(with `valid` in the vector), before any consumer is wired.

**Not modelled, each a later switch:** absorption (carpet), ping noise, servo-whine coupling
into the 40 kHz receiver (BOM §7 "unmeasured"). **Still open in S1:** the camera luma at the
robot's 32×32, `odom`/`vel_ego` as a shared body helper, the robot publishing range and camera
during a brain run, the separation study, and the three actuator measurements (turn table,
stall signature, reverse).

---

### 2026-10-10 — S1 SIM HALF COMPLETE: THE ROBOT'S SENSES ASSEMBLED IN THE ROOM, INSTRUMENT-ONLY, FOR THE OPERATOR'S EYE

*Plan §4 S1. Preset: `…__nohomeo__room_senses.json`, in the launcher dropdown. Verdict:
`BASELINE` (an assembly; nothing tuned, no consumer). Gain-0: every switch off, the three
deployed configs are byte-identical on body JSON (seed 1 × 6000 ticks).*

**What was built, beside the ultrasonic model and the room (entry above):**

- **The camera at the robot's format** (`OGMA_PICRAWLER_CAMERA_ROBOT=1`): the sim's raycast
  camera rendered as pi_host's plane — the 4:3 frame centre-cropped square, so a 41.4° field
  both ways, 32 × 32 luma (Rec. 601 weights), on `sense.camera`, captured every 3 ticks and
  the cached frame published every tick. `picrawler_senses.json`'s `epm_vision` runs on it
  unchanged. The mount is the sim's loom eye (chassis + 0.08 m); the robot's camera position
  is not measured yet.
- **Dead-reckoned odometry** (`OGMA_PICRAWLER_ODOM=1`): `ogma::body::DeadReckon`, a shared
  header the robot host will run unchanged (six unit tests pin the frame), integrating
  `stride_v` under `ego_heading`. `reality.proprio.odom` = [x, y, unwrapped yaw] in the body
  frame at the last reset; `reality.proprio.place_in` = [x/L, y/L, cos ψ, sin ψ], the duck's
  place-map input. Egocentric. Measured against the truth in the odom frame (instrument,
  never published): before the wall, forward reads 0.88 of the truth (the stride sensor's
  known 75–90 %), yaw matches, and the lateral channel has the right sign but reads ~0.4
  (the stride sensor's own note: "lateral channel UNVALIDATED"). Pressed on the wall the legs
  keep stepping and the integrator keeps walking — 14 m of "stride" for 1.4 m of ground over
  the run, odom_err 0.5 m at the end. That is the slip the stride fusion already names, and
  the odom carries it as dead reckoning must.
- **Honest `vel_ego`** (`OGMA_PICRAWLER_HONEST_VEL_EGO=1`): `[stride_v.x, stride_v.y]` in
  place of the world-velocity soft oracle — the ledger's one-topic swap that makes PlayLoop
  / PlaceNav legal.
- **`OgmaBrain.publish_image`**: a RawImageFrame on an arbitrary topic (as `publish_token`
  for ProprioTokens), so sim channels land on the robot's topic names.
- **Three instrument-only EPMs** in the preset, none read by anything: `epm_range` (rbf over
  `sense.range`, the senses config's), `epm_vision` (jl retinal 32 over `sense.camera`),
  `epm_place` (rbf over `place_in`, ±1.1, 128 nodes, every 5 ticks, the duck's map
  parameters). The JSONL carries their node / baked / TLE so a headless run can show each
  channel being coarse-grained.

**The run (seed 1, 6000 ticks):** the body behaves as the room baseline (into the +Z wall at
tick ~1200, held there). The range vocabulary grows to 22 nodes on the approach and settles at
16 / 13 baked once the wall holds it; the camera's at 10 / 10 (mean luma 62 → 54 as the wall
fills the frame); the place map at 4 / 4 for a 1.3 m walk. Every channel moves, every
vocabulary bakes, nothing consumes.

**What S1 still owes:** the robot half (`ogma_host --range --camera` during a brain run,
`ego_heading` / `odom` published from `BrainInputBuilder` via the same header), the camera's
mount position, the separation study on the recorded senses, and the three actuator
measurements (turn table, stall signature, reverse). Then S2, the speed socket.

---

### 2026-10-11 — S1 ROBOT HALF, FIRST PASS: THE ROOM PRESET'S SENSES RUN ON THE PI; ODOMETRY WAITS FOR ARMED SERVOS

*Plan §4 S1, robot. Build: the merged tree (`7fcf4fc`) built natively on the Pi (`ogma_core`
with the MicroDuck modules, ~45 min at `-j2`, no errors; `test_hw` passes). Host: `ogma_host
--odom [--odom-scale] --vel-ego` (ogma::body::DeadReckon, the sim's integrator, on `stride_v`
under the gyro's unwrapped yaw); the dash passes `--range --camera --odom --vel-ego` from a
config's `body_env`, as the sim reads the same keys. Verdict: `IN_FLIGHT` — three of four
channels verified live; odometry built, not yet observed on the robot.*

**The read-only check** (`ogma_host --config …__room_senses.json --imu --brain-inputs --range
--camera --odom --vel-ego`, 1500 ticks, no `--actuate`; benchd in bench mode; the senses
service stopped for the run and restarted after). Nothing moved.

| channel | on the robot |
|---|---|
| `sense.range` | up, 99 pings per 250 ticks ≈ 20 Hz, 0 timeouts |
| `sense.camera` | up, 63 frames per 250 ticks ≈ 12.6 fps (15 target), mean luma 99 |
| `epm_range` / `epm_vision` / `epm_place` | in the graph; the range vocabulary holds nodes within 150 ticks (probe on the control port) |
| `odom`, `place_in`, `ego_heading`, `vel_ego` | **not observed**: they ride the brain-input path, and the builder withheld all 1500 ticks because every servo read 0 µs (benchd in bench mode, no pose set). The 0-µs guard is deliberate ("its angle is unknown"); the dash's run flow sets a pose first |
| host tick cost, all senses on | wall p50 0.15–0.4 % of the 20 ms budget, p95 < 1 % (`host_cost`); 0 overruns in three of four runs. One run showed a 57-overrun burst between ticks 500 and 750 with p95 still < 1 %: a stall, not load — suspect the camera pipe's start. Re-measure in the dash flow |

**What verifies the odometry:** any run with commanded servos — a held pose on the bench, or
the room preset picked in the dash on the floor (which also exercises the integrator
walking). The first-tick odom must read 0 and `place_in` [0, 0, 1, 0]; a walk's `odom.x`
should grow at ~0.75–0.9 of the ground covered (the sim's read). Both are the operator's call
because either arms the servos.

**Also noted:** 45 sequence gaps in 1234 state-feed frames over 1500 ticks (189 ticks stale)
with nothing else running — the feed's own cadence, to read beside the robot's known step rate
before trusting any per-tick odometry number.

**Addendum, 2026-10-11 (the operator's call: the stand pose held on the bench).** With
`pose_hold.py stand` feeding the deadman (12 servos armed, 0.22 A at 8.1 V on the bench
supply), the same read-only run published **1500/1500 ticks, 0 withheld, 0 overruns**, 11 seq
gaps in 1471 state-feed frames. The odometry at rest: `odom` stayed at |x|, |y| ≤ 0.1 mm and
yaw drifted 0.0005 rad over 30 s (the gyro's converged bias); `vel_ego` equalled `stride_v`
sample for sample (≤ 1 mm/s of noise); `upright` 0.9997. All three instrument EPMs baked
their first nodes (range 2/2, camera 6 nodes 1 baked as the bench scene drifted, place 2/2).
Verdict on the four channels: **`WORKING` as instruments on the robot.** What remains for
the odometry is the walking read (the room preset from the dash, on battery): `odom.x` at
~0.75–0.9 of ground covered, as in the sim. Noted in passing: in `stand` on the bench only
`fl` and `rr` read FSR contact (≥ 200 counts) — the FSR threshold against this pose, to read
before any consumer leans on four-foot contact.

---

### 2026-10-11 — S2: THE SPEED SOCKET OPENS THE INTENT BOUNDARY; THE SPEED TABLE AND THE TURN TABLE (n=3)

*Plan §4 S2 and S1's actuator measurements. Lever: MotorEPMv2 `speed_topic` (ProprioToken [s],
s ∈ [−1, 1], scaling the hip1 stroke's propulsion term only; the steer differential and the
heading hold untouched) and `goal_bearing_rezero_hold` (a goal-bearing message re-zeroes the
hold's integrator, so a later silence holds the body's heading, not the spawn's; trap T4).
Gain-0: no message = the 1.0f literal; `rezero` default 0. The three deployed configs are
byte-identical on body JSON (seed 1 × 6000). Unit tests (`test_state_prior`, the MotorEPMv2
fixture): silent-equals-absent, half speed and reverse diverge, the consumer check, the
re-zero off/on. Config `…__room_senses__s2.json` names both sockets; S2 scaffolds
`OGMA_PICRAWLER_SPEED_CMD` / `OGMA_PICRAWLER_GOAL_BEARING_DEG` publish constants (named as
scaffolds, standing in for S3's loops). Harnesses: `speedtable.py`, `turntable.py`. Verdict:
`WORKING` on the channel (both sockets measured live and monotone); two signals for S3 below.*

**The speed table** (room, 3000 ticks, n=3, displacement along the spawn heading):

| speed | along (m) | path (m) | belly (m) | wall contact | falls |
|---|---|---|---|---|---|
| +1.0 | +1.10 ± 0.42 | 2.18 | 0.027 | 0.27 | 0 |
| +0.5 | +0.78 ± 0.21 | 1.92 | 0.027 | 0.10 | 0 |
| 0.0 | **+0.28 ± 0.06** | **1.58** | 0.027 | 0.00 | **1** |
| −0.5 | −0.47 ± 0.42 | 1.98 | 0.027 | 0.00 | 0 |
| −1.0 | −1.28 ± 0.05 | 2.49 | 0.028 | 0.06 | 0 |

Monotone and nearly linear in the command, with an offset: **speed 0 is not standing still.**
The propulsion term is gone but the gait's other terms (coupling, posture objectives, stance
lift, explore noise) keep the legs cycling, and the body drifts 0.28 m forward on 1.6 m of
path, with one fall in three seeds. **Reverse works**: the stroke runs the other way and the
body backs up at the same rate it advances, belly up, 0 falls. That is the exit the duck
measured as the only one (`--no-backing` REGRESSION), now available. *Signal for S3:* a "stop"
is not speed 0 — it is a stance, and the stuck-stop needs a posture objective, not a zero.

**The turn table** (arena, 2000 ticks, n=3, a constant body-relative bearing = keep turning;
+ = toward body +X):

| error | yaw rate (°/s) | fwd_v | falls |
|---|---|---|---|
| 0° | +0.6 ± 2.4 | 0.041 | 0 |
| +15° | +10.8 ± 7.5 | 0.029 | 0 |
| +45° | +11.4 ± 4.7 | 0.032 | 0 |
| +90° | +17.3 ± 8.0 | 0.001 | 0 |
| +180° | +21.7 ± 3.0 | −0.017 | **3** |
| −15° | −4.0 ± 2.3 | 0.032 | 0 |
| −45° | −12.4 ± 2.9 | 0.009 | **2** |
| −90° | −12.7 ± 3.6 | 0.014 | **1** |

The yaw actuator saturates early, as the rectified differential predicted (hip1 at 56 % clip
duty; with P = 7 in π-units a 25° error already rails the steer): **≈ 11–13 °/s from 15° up,
≈ 17–22 °/s only at 90–180°, where the body is turning in place and falling.** Forward speed
drains to zero past 45°. There is a handedness: +15° turns at 11 °/s, −15° at 4 °/s. *What
this gives S3:* the arbiter's reach constant from ω ≈ 0.2 rad/s (the duck's `reach_k = 2 m·ω
/ (π·v)` with v ≈ 0.05 m/s gives k ≈ 2.5 — a target 1 m away at 90° cannot be turned to in
the time it takes to arrive), and a rule: **never hold a large bearing error for long** — the
loops must command speed down while turning, which is exactly the duck's ★ TURN speed gate.

**Six falls in 24 runs at |error| ≥ 45°** against 0 for P-e·h0 in the room: the loud number of
this entry. Sustained turning in place is where this body tips. S3's avoidance must turn by
small bearings and back off rather than pivot.

**Open:** the stall signature (robot, walking into a wall), the reverse on the robot, the
"stop as a stance" objective.

---

### ★★★ 2026-10-11 — S3a: THE AVOID LOOP DRIVES THE GAIT — WALL CONTACT 0.53 → 0.008, COVERAGE ×2, NO FALLS (n=6)

*Plan §4 S3, form A, lever 1. Three loop modules on the S2 assembly, nothing learned:
**ProxMemory** (new, generic): every valid ping of the single ultrasonic beam becomes a point in
the odom frame, kept 3 s, re-expressed in the body frame each tick and binned by azimuth into
the duck's four slots `[−X, ahead, +X, too_close]` plus an 8-sector view; **TofAvoidLoop** (the
duck's, R28 — `REGRESSION` there, on a walker with a yaw deadband) bears away from the nearer
side, silent below proximity 0.10, with a new `side_min` tie rule (a flat wall head-on fills
the slots equally and the legacy rule normalised that into "go ahead"; default 0 keeps the
duck's configs byte-identical); **BearingToIntent** (new, generic): the loop's bearing to
`goal_bearing_topic`, a HELD heading reference while the loop is silent (the duck's
IntentAdapter rule), and the speed to `speed_topic` shed as cos e with a 0.2 floor (the turn
table's lesson: never pivot). Config `…__room_senses__s3a.json`, in the launcher. Unit tests:
ProxMemory 6, BearingToIntent 6, TofAvoidLoop 5. Gain-0: the three deployed configs are
byte-identical on body JSON (seed 1 × 6000). Verdict: **`WORKING`, LOUD** at n=6.*

| metric (room, 6000 ticks, n=6) | baseline P-e·h0 | S3a | per seed (S3a) |
|---|---|---|---|
| first contact (tick) | 1210 ± 223 | three seeds never; 900 / 4260 / 4980 | −1 4980 −1 900 4260 −1 |
| **wall_frac** | **0.531 ± 0.096** | **0.008 ± 0.013** | 0 .015 0 0 .031 0 |
| contact episodes | 126 ± 21 | 3.7 ± 5.6 | 0 6 0 2 14 0 |
| **coverage** (0.25 m cells) | **12.7 ± 1.8** | **27.2 ± 2.5** | 25 27 24 30 27 30 |
| path (m) | 3.66 | 6.20 ± 0.37 | — |
| net_disp / straight | 1.55 / 0.42 | 0.74 / 0.12 | — |
| falls | 0 | **0** | — |
| us_valid / confound rows | 0.94 / 5 | 0.47 / 26 | — |

**What the body does.** It walks at the far wall as before; at proximity ≈ 0.5 (≈ 0.5 m) the
loop bears away and the speed sheds to ≈ 0.7; it turns (yaw 1.2 rad in seed 1), holds the new
heading when the loop goes quiet, and keeps walking. Over 6000 ticks it covers twice the
floor the baseline did, on 1.7× the path, and touches a wall in three of six seeds for a few
dozen ticks in all. Straightness falls because it turns — in a 3 m room that is the job, not a
cost; the blind-metric pair (coverage up, path up, contacts down, 0 falls) says it is not a
spinner.

**§3.2 checks.** Not a tautology (the baseline has none of the three modules); live
(`gb_msgs` = 6000 of 6000 ticks, the intent active in 38–53 % of diag rows, speed shed to
0.35–0.57 at the turns, 30–40 remembered points on average); healthy baseline (walks, hits the wall, holds); faithful (this is the
duck's reflex form as built, with the heading reference it had host-side); the arm loaded
(the three receipts; `side_min` 0.1). The confound is now frequent — half the rows have no
echo, because the body spends its time at angles to walls — and the loop still works
because the memory carries the pings it got square-on. That is what the memory is for.

**The duck's verdict does not transfer, in the direction the plan predicted.** R28's
`REGRESSION` was a bearing loop on a walker whose yaw actuator had a deadband; on a body whose
heading PD is loud, the same loop is loud. Recorded as the re-use context of R28.

**What is left of S3.** Lever 2 (the stuck detector + escape by backing) now addresses the
residual: the seeds that touched (6 and 14 episodes) did so where the beam had no echo (the
glancing confound), and nothing yet reads "I am commanding forward and not moving". Form B
(the learned priors on an intent-level MotorEPMv2) is **deferred by the plan's own gate** —
"if A alone is loud, B is deferred with that context recorded": the context is a loud reflex
form at n=6 in a 3 m room with fixed boxes; B's re-use context is a scenario where the
reflex's fixed geometry fails (moving obstacles, the robot's real echo statistics, or the
(d) test) or where avoidance must be shaped by what the body learns of its own turning.
The (d) test (move the boxes mid-run) is owed; with a 3 s memory it tests the loop, not a
map, and belongs with S4.

---

### 2026-10-11 — S3 LEVER 2: THE STUCK SENSE WORKS; THE ESCAPE BY BACKING IS NULL ON THE STACK IN THE ROOM AND A REGRESSION ON TERRAIN (n=6)

*Plan §4 S3, form A, lever 2, on S3a. Two modules: **StuckSense** (new, generic), the stall as
the duck defines it, body-scaled — the stride forward's 2 s mean against a decaying peak-hold
of itself (the body's own recent best speed), while commanding forward and not turning (the
NET yaw over 3 s, whole strides); plus a range-deficit channel for the slip case. **StuckEscape**
(new, generic), the duck's rule: a confirmed stall → back 1.5 s → hold a heading toward the
freest remembered sector (or an alternating 90°) for 2 s → release, 3 s refractory, through
BearingToIntent's new `override_topic`. Config `…__room_senses__s3b.json`; the instrument arm
`…__s3b_escape_only.json` silences the avoid loop so the escape must act alone. Unit tests:
StuckSense 8, StuckEscape 4, BearingToIntent 7. Gain-0: the three deployed configs byte-identical
(seed 1 × 6000) at every build. Verdicts below, one per thing measured.*

**The stuck sense: `WORKING` as a percept.** Pressed on a wall this body keeps stepping: its
honest `distress` stays at 0.000 and its `slip` reads 0.066 against 0.069 free — the stuck
state was a HIDDEN state with no observation (doctrine §2.1, confirmed a third time). Its
stride forward estimate oscillates −0.35…+0.46 m/s around a mean of 0.007 against ~0.03
walking free, so the per-tick stride is useless and its 2 s mean is the signal. Three
instrument passes found three ways the sense can be silenced, each now a parameter with a
reason: (1) the range-deficit channel ABSTAINS on the wall, because it only speaks when the
stride claims walking and the stride's mean there is ~0 — it is the slip channel, not the
stall channel; (2) gating "not turning" on the COMMANDED bearing silenced it for good once a
pinned body was commanded to turn away and could not; (3) gating on a short yaw-rate EMA
silenced it every stride, because a quadruped yaws every stride by construction — the net yaw
over 3 s is what a turn is. On the escape-only arm (loop silenced, 3 seeds, 6000 ticks) the
sense fires and the escape acts: 3–5 escapes a seed, **wall contact 0.27–0.35 against 0.45–0.54
for the same body with only its heading hold**, coverage 14–21 against 13, 0 falls.

**The escape on the stack (S3b vs S3a, room, n=6): `NULL`.**

| metric | S3a | S3b |
|---|---|---|
| wall_frac | 0.008 ± 0.013 | 0.016 ± 0.027 |
| episodes | 3.7 | 8.5 |
| coverage | 27.2 ± 2.5 | 27.0 ± 2.2 |
| falls | 0 | 0 |
| escapes / seed | — | 0 1 1 2 2 1 |

The avoid loop already keeps the body off the walls; what the escape adds is three episodes
fired while the body was dawdling in open floor (the gait's own pauses: it had not moved for
2–4 s while commanding forward — the sense told the truth, the escape had nothing to fix),
and the one seed that did touch (seed 5, the glancing slide) touched longer. Nothing in the
room needs it.

**The escape on terrain (corridor, difficulty 0.5, 12000 ticks, n=6): `REGRESSION`.** S3a walks
3.3–4.5 m down the corridor and over the hump in every seed; S3b escapes 4–9 times a seed and
ends four of six runs **behind the spawn** (z −0.4), one fall. The stall fires on the hump —
truthfully: a leg snagged on terrain is a stall — and backing-and-turning is the wrong answer
to it: it retreats from the thing the body was climbing. This is the ledger's own
`homeo_leak_progress_gate` lesson in a new coat ("blocked at a slanted wall and no longer
learns how to climb"): **a reflex that retreats from resistance kills climbing.** The duck's
world had no terrain, so its escape never met this.

**Re-use context, and what the result points at.** The sense stays (an instrument of the
stall, the brain-side observation the power problem needs: "pushing and not moving" is
where the servos draw their peaks). The escape as a REFLEX is refuted on this body in both
scenarios it was tried; it is not dead. What distinguishes a wall-snag from a hump-snag is
not in the sense — both are "commanding forward, not moving" — it is whether *trying
differently* restores progress, which is a question for a loop with COMPETENCE under the
arbiter (S4: LoopCompetence grades "did my escape restore progress", and the arbiter learns
when to hand the body to it), or for the climbing homeostat. That is the learned form of
this lever, and the gate that B was deferred by now names it: the reflex's fixed geometry
fails on terrain. Lever 2's scripted escape is `DEFERRED` to that. The stack remains S3a.

**On the way:** `BearingToIntent.override_topic` (an escape or any loop can own the intent
outright, with its own speed, negative for backing) and `sense.servo_current` from the sim's
power model (`OGMA_PICRAWLER_PUBLISH_CURRENT`, opt-in) are built and gain-0, for the
current channel of the sense on the robot.

---

### 2026-10-12 — S3b DEPLOYED TO THE ROBOT: THE REFLEX GRAPH LOADS AND RUNS ON THE PI, VERIFIED AT REST; THE FLOOR RUN IS THE OPERATOR'S

*The operator's rule: the hardware keeps pace with the sim at each phase. The Pi is at `6cb9aca`
(fast-forward, 8 objects rebuilt, no errors). Read-only check with the stand pose held on the
bench (`pose_hold.py stand`, 0.21 A at 8.0 V), the S3b config, every sense flag the dash passes,
no `--actuate`, 1500 ticks. Verdict: `IN_FLIGHT` — the graph is live on the robot; behaviour is
the floor run.*

| what | on the robot |
|---|---|
| the graph | all 17 modules load: P-e·h0's nine, the three instrument EPMs, ProxMemory, TofAvoidLoop, BearingToIntent, StuckSense, StuckEscape |
| brain inputs | 1468/1500 ticks published, 0 overruns |
| the proximity memory | 537 pings in 30 s, points held from the live beam (a bench object at 0.17–0.42 m) |
| the avoid loop | ahead 0.83, sides 0 — it sees the object |
| the intent bridge | 1422 bearings consumed, reference set (the body cannot turn to it: held) |
| the stuck sense | 0 measurements, score 0 — a standing body has never walked, so it has no stall (the floor gate working as designed) |
| the escape | 0 episodes |
| host tick cost | wall p50 1.5 % of the 20 ms budget, p95 2.2 % (was 0.3 % with the senses alone) |
| the dash | lists S3b and the escape-only arm as robot-faithful, with `--range --camera --odom --odom-scale 2.0 --vel-ego` |

**Watch item for the floor:** the state feed showed **447 sequence gaps in 1013 frames** (32
ticks withheld as stale) against 11 in 1471 in the stand run of 2026-10-11 with the same
sensors on. Nothing in benchd's journal. Either the heavier graph plus the camera process
contend on the Pi's four cores, or the probe's snapshot calls held the host's lock; the floor
run has no probe and the dash records the feed, so it will say. If the gaps recur, the odom
and the stall channel read them as pauses.

**How it is run:** `picrawler-dash`, pick "S3b · the avoid loop + the stuck sense and the
escape", battery, the 10 s countdown, SPACE. A boxed-off floor with a wall the beam can see
square-on. What to watch, in order: the body turning away before contact (S3a's behaviour,
measured in the sim); whether the stall ever confirms on carpet and what the escape does
when it does; falls, and the rail. The escape-only arm is the one to watch the escape itself.

**Also:** the vision EPM's stale `encoder_res` parameter (ignored with a warning at load) is
removed from the six configs that carried it; no behaviour change.

---

### 2026-10-12 — S3b ON THE ROBOT, TWO FLOOR RUNS: THE ESCAPE BACKS THE BODY UP EVERY 7–10 s ON OPEN FLOOR — THE STRIDE ESTIMATE IS NOISE AT CARPET SPEEDS (REGRESSION; S3a IS THE ROBOT STACK)

*The operator ran S3b from the dash twice (carpet, "various environments"; battery; 416 s and
290 s of brain time; 1 and 3 HAT resets, auto-recovered) and reported a tendency to walk
backward, strongest in the second run with open floor ahead, and asked whether the
rangefinder saw the floor on a nose-down pitch. Logs: `~/xaq_robot_runs/dashrun_20261012_0137*`
and `_0145*` (feed, events, meta, the host log; audio left on the Pi).*

**It was the escape, not the range trigger.** Nothing in S3b but the escape's BACK state can
reverse the stroke: the avoid loop sheds speed to a floor of 0.2 while turning and never
below it, and a pitched beam seeing the floor would read as "something ahead" — a turn, not
a retreat. The commanded pulses in the 50 Hz feed say it directly. Decoding each leg's hip1
sweep direction during its knee-lifted phase (the servo map, 1 s windows, three legs voting):

| run | brain time | reverse episodes | reverse time | spacing |
|---|---|---|---|---|
| 1 (013732) | 416 s | 40+ (1–8 s each) | 91 s (22 %) | every 4–12 s |
| 2 (014553) | 290 s | 31 (1–5 s each) | 38 s (13 %) | every 5–10 s |

That spacing is the escape's own cycle — confirm 1 s + back 1.5 s + turn 2 s + refractory 3 s
≈ 7.5 s — so the stall was confirming again almost as soon as it was allowed to.

**Why the stall confirmed on open floor.** The sense is the stride forward estimate's 2 s mean
against its own recent peak. On the robot the per-sample stride swings **−0.6 … +1.0 m/s
around a mean of 0.023 m/s** (174 of 406 samples negative in run 1, 116 of 273 in run 2):
the stance-leg kinematics read the body's rocking, carpet compliance and FSR threshold
flicker as metres per second. A 2 s EMA of that has a ripple comparable to its mean, so it
spends much of its time below a quarter of its own peak, and a body walking normally reads
as stalled. In the sim the same channel separated 0.007 on the wall from 0.03 free; the
robot's stride has no such separation at these speeds. **The stride estimate is not a
progress observation on carpet**, and S3b's verdict on the robot is `REGRESSION`, for a
reason the sim could not show: the sim's stride is too clean.

**Not seen in these logs:** the range itself. The host dumps the brain inputs, not
`sense.range`, and nothing on the robot recorded the loop modules' state (the avoid value,
the intent's mode, the stall score, the escape's state). That gap is why the diagnosis had to
come from the pulses, and it is closed below.

**What changes.**
1. **S3a is the robot's reflex stack.** Deployed as `…__room_senses__s3a_stuckinst.json`:
   S3a with StuckSense present and NO escape — the stall score is an instrument on the robot
   until it has a progress observation it can trust (the servo current on a snag, the range
   rate when a wall is square ahead, or optical flow: each its own lever, measured first).
2. **The dash records the loop modules' state** (`brain_diag.jsonl`, 5 Hz: proximity slots,
   the avoid bearing and value, the intent's mode/error/speed, the stall score and its
   internals, the escape's state) and the host's own range and stride samples, so the next
   run is read from the brain's side, not reconstructed from the servos.
3. The range-timeout rate on carpet is a number now: run 1, **1226 of 6973 pings had no
   echo (18 %)**; run 2, see the host log. The confound is live on the robot at that rate.

**On the operator's hypothesis** (the beam seeing the floor nose-down): not refuted — the
level mount's −5° lobe reaches the floor at ~1 m, and a nose-down pitch of ~25° brings it
nearer — but it cannot produce backing, and nothing in these runs recorded the beam. The
next run does.

---

### 2026-10-12 — RUN 3, READ FROM THE BRAIN'S SIDE: EVERY ESCAPE FIRED ON A STRIDE MEAN ≤ 0 WHILE WALKING; THE AVOID LOOP NEVER RESTS IN A REAL ROOM; THE 0.6 m SCALE IS A REGRESSION IN THE SIM

*The operator's third floor run (S3b, 240 s, 2 HAT resets) is the first with `brain_diag.jsonl`
(1150 rows at 5 Hz) and the machine-cost fields. Logs: `~/xaq_robot_runs/dashrun_20261012_1659*`.*

**The escape, directly observed.** 16 episodes in 230 s, every ~14 s; the body in BACK or TURN
26 % of the time, reversing 10 % of the time (the pulse decoder agrees: 29 reverse windows,
59 s). Before every one of the 16 triggers the stride's 2 s mean was **≤ 0** (−0.004 … −0.104
m/s) against a peak-hold of 0.013–0.118, with the body walking. Range at the trigger: 1.5
with no echo in 5 cases, 0.4–1.2 m valid in the rest, never under 0.3 m — not a floor return
(`upright` p05 0.964 = 15°; past 25° in <2 % of samples). **The floor hypothesis is closed for
the backing. The stride estimate is the cause.**

**The stride estimate on carpet is not a speed sensor.** Per-sample forward −0.92 … +0.80 m/s,
sd 0.19, **mean 0.004** at 5 Hz (−0.019 in the host's 1 Hz dumps, 118 of 224 negative); the 2 s
EMA has mean 0.001 and sd 0.039. The body walked the whole run. The integrated odometry
credits it with 1.75 m forward and −1.5 turns over 234 s, with 2.8 feet reading FSR contact on
average (the threshold admits dragging feet into the stance set). Run 2 on another floor had
a mean of 0.023. A stance-FK stride whose mean is 0–0.02 m/s with a 0.19 m/s sample spread
cannot carry a stall sense, a speed sense or an odometry on this robot, whatever the sim says
(there its spread is a tenth and its mean holds). This is the open problem under everything
that reads progress on the robot: StuckSense, DeadReckon, place_in, and the honest `vel_ego`.
Candidates, each a lever: a stance set from the FSR *shape* rather than one threshold; the
range rate against a square wall as a speed calibration; optical flow from the camera; the
IMU's own integration over a stride with a ZUPT at each planted foot.

**The avoid loop never rests in a real room.** Active 100 % of the samples, median value 0.48,
44 remembered points on average, the intent's error over 45° in 26 % of samples, mean speed
0.60: at a 1 m proximity scale the robot's room always has something within reach, so the
body never holds a heading and always walks slowed. That is the trigger range the operator
asked about. **A/B in the sim room, S3a with `range_m` 0.6 and `floor` 0.15 (`…__s3a_near.json`,
n=6): `REGRESSION`** — wall_frac 0.116 ± 0.074 against 0.008, contact episodes 56 against 3.7,
coverage 27.7 (tie), 0 falls. A 0.5 m warning is not enough for a body that turns at 12 °/s.
Re-use context: a scale set by the body's own turning authority (the arbiter's reach
constant, S4) rather than a constant; or a room with nothing within a metre.

**Fixed on the way:** every escape turned −52° (sector 0) — the tie-break took the first free
sector; it now takes the free sector nearest straight ahead (test added). The escape stays
deferred.

**The machine's side, now recorded:** host process 6–10 % of one core at full load, 1-min load
0.6–1.0 on four cores, RSS 12.6 MB, 36 → 42 °C over four minutes, tick p50 2–7 % of budget,
0 overruns; **112 sequence gaps in 11558 frames (1 %)** — the 447 of the probe run were the
probe's snapshot calls holding the host's lock, not contention. Servo current median 1.47 A,
p95 2.98 A, max 3.40 A: the known regime, two HAT resets.

**The robot stack stands:** `…__s3a_stuckinst.json` — S3a, the stuck sense as an instrument.

---

### 2026-10-12 — THE TIBIA-PLUMB LEVER, RE-RUN ON THE ROBOT'S BODY IN THE SIM: DISTANCE TIES, THE BELLY PAYS 9 mm, THE SHANK MOVES 4° — THE GEOMETRY CLOSURE HOLDS; THE ROBOT IS THE TEST

*The operator's diagnosis (2026-10-12): on carpet the gait "swims" — the lower leg lands at an
angle, slides, and the FSR stance set admits dragging feet; good steps are the ones with the
shank perpendicular to the floor. That is the ledger's "get the tibia vertical" family
(§ 2026-08, `tibia_plumb_gain`: +32 % distance on the cad body, held back only by belly
clearance; the family closed on limb geometry — feet at 170 mm against a 166 mm reach). One
lever, re-run on the body the robot actually is: `tibia_plumb_gain` 0.15 on P-e·h0
(`…__nohomeo__plumb015.json`), arena, 12000 ticks, n=6; and on the robot stack in the room
(`…__s3a_stuckinst__plumb015.json`, 6000 ticks, n=6). Both arms reached the arena's ramps
(r ≈ 10 m) so `falls` there is a floor, not a count. Verdict in the sim: `NULL` on distance,
`REGRESSION` on belly and on wall contact; the robot's context differs, see below.*

| arena, P-e·h0 (measured_fsr body) | base | plumb 0.15 |
|---|---|---|
| net_disp (m) | 9.40 ± 0.91 | 9.73 ± 0.42 (tie) |
| straight | 0.74 ± 0.05 | 0.77 ± 0.03 |
| **belly clearance** (m) | **0.0299** | **0.0213** (−8.6 mm) |
| belly minimum (m) | 0.0039 | 0.0015 |
| **tib_off** (shank from vertical) | **47.0°** | **42.9°** (−4°: the lever acts) |
| foot_r (mm, reach 166) | 167.4 | 168.6 |
| planted / contact_duty | 3.89 / 0.83 | 3.88 / 0.85 |
| falls | 0 | 0 |

| room, robot stack | s3a_stuckinst | + plumb 0.15 |
|---|---|---|
| wall_frac | 0.008 ± 0.013 | **0.075 ± 0.030** |
| contact episodes | 3.7 | 31.5 |
| net_disp / straight | 0.74 / 0.12 | 1.05 / 0.18 |
| coverage | 27.2 | 23.0 |
| falls | 0 | 0 |

**What it says.** On this body the +32 % does not reproduce (+3.5 %, inside the seed spread)
and the belly cost does (−8.6 mm, the minimum down to 1.5 mm): the shank gets only 4° more
vertical because the feet still plant at 168 mm on a 166 mm reach — the geometry closure of
2026-08 stands on the FSR leg too. In the room the plumb gait is faster and straighter (net
+42 %, straight +50 %) and **hits the walls nine times as often**: the avoid loop's 1 m
warning was fit to the slower gait, and a straighter body reaches the wall before the loop
has turned it — the same lesson as the 0.6 m scale, from the other side: the loop's lead must
follow the body's speed and turning authority (S4).

**Why the robot is still the test, and what to read.** The sim's stance SAGS (the robot stands
15–25 mm taller on its stiff servos, ledger 2026-10-04), which is why the sim's feet sit at
the reach limit and the belly has no margin to give. On the robot the margin exists and the
feet sit nearer, so a vertical shank is geometrically reachable there and the belly cost may
not bind. The robot run reads, in order: the **feet-in-contact count** (run 3: 2.8 of 4; a
clean stance set reads ~2–2.5 while walking), the **stride mean** (0.004 m/s; it should
become a speed), the **belly ToF** in the feed (`tof_m`, against the 20–30 mm it holds now),
falls, and the current. Not promotable from the sim; a robot-only re-use context, measured.
Config: `…__room_senses__s3a_stuckinst__plumb015.json`, in the dash. Gain 0.15 only — the
sim found 0.3 unstable on the cad body.

---

### 2026-10-12 — THE PLUMB LEVER ON THE ROBOT: THE SHANK MOVES THE SAME 4.5° IT MOVED IN THE SIM, THE BELLY PAYS 7 mm, THE STANCE SET AND THE STRIDE DO NOT MOVE — THE KNEE TUCK IS WHAT PINS THE SHANK

*The operator's run of `…__s3a_stuckinst__plumb015.json` (carpet, 215 s, 0 HAT resets; a first
134 s run ended by hand). Operator's eye: "the tibia was slightly more vertical but not all the
way; occasionally a few good steps, but the run is dominated by noisy movements"; and the
question whether the chassis must be allowed to stand higher. Against run 3 (the same gait
without the lever, same floor):*

| | run 3 | plumb 0.15 |
|---|---|---|
| shank off-vertical (from the commanded joints, the plumb formula) | 47.1° | **42.6°** |
| hip2 (proprio units, mean) | +0.019 | −0.035 (the reflex pulls it) |
| knee (proprio units, mean) | 0.825 | 0.822 (unchanged: the tuck target holds it) |
| feet in contact (FSR ≥ 200) | 2.80 | 2.73 |
| stride forward, mean / sd | −0.019 / 0.19 | +0.011 / 0.19 |
| odometry, 215 s | 1.75 m | 1.36 m |
| **belly ToF, median / p05** | **27 / 9 mm** | **20 / 5 mm** |
| servo current, median | 1.50 A | 1.71 A |
| avoid loop active / speed mean | 1.00 / 0.60 | 0.98 / 0.79 |

**Verdict: `NULL` on the stance set and the stride, `REGRESSION` on belly (−7 mm, p05 5 mm)
and current (+0.2 A).** The sim predicted all three numbers: −4° on the shank, −8.6 mm on the
belly, a distance tie. The robot's taller stance did not buy the margin hoped for.

**Why the shank stops at 43°, and the operator is right about height.** The plumb reflex acts
through hip2 only. The knee is held at `knee_tuck_target` 0.85 of its range by the postural
spring (`postural_gain` 1.078) — the "spider stance, knees tucked, chassis suspended below"
promoted for belly-up on the cad body — so the foot's radius is set by the knee and the hip2
can only tilt the shank a few degrees before it runs out of geometry (feet at 168 mm on a
166 mm reach, the closure of 2026-08). A vertical shank needs the feet NEARER the hips, which
is LESS knee tuck, which stands the chassis HIGHER — exactly the operator's "allowed to stand
higher". There is no height limit as such in P-e·h0 (the height homeostat is off, "h0"); the
height is CONSTRUCTED, not held (§2284: without an active lift term this body sinks to 29 mm and
stops stepping) — in P-e·h0 by `stance_lift_gain` 0.5 on the planted legs, against the knee
tuck 0.85 and the postural spring. Standing higher is a dose on that pair. So the lever that reaches the
operator's stance is the pair `knee_tuck_target` (down from 0.85) with the plumb gain, read
against belly, falls, stride and current — a body fact already in the ledger (§2284: at tuck
0.7 and `postural_gain` 0.7 the body stands differently) and the re-use context the geometry
closure asked for ("a lever that moves foot placement by some route other than hip2, knee or
stride" — or the knee itself, deliberately). Next lever, sim first.

**The noise the operator sees** is the known command thrash (ledger 2026-10-02: four imposed
terms each drive the joint command to full slew with reversals every 1–2 ticks). It cannot be
read from the feed's `out` pulses, which are benchd's slew-limited output (0.000 reversals ≥
20 µs per servo-tick in both runs); it is the brain's raw command, before the slew. That is
the gait's own open problem, separate from the stance geometry, and the ledger already names
its four sources.

---

### 2026-10-12 — THE KNEE-TUCK DOSE (WITH PLUMB 0.15): LESS TUCK IS A FLATTER, WIDER SPRAWL — THE CHASSIS SINKS 45 → 27 mm AND THE FEET GO OUT; STANDING HIGHER IS THE FEMUR'S JOB (REGRESSION, MONOTONE)

*The operator's hypothesis after the plumb run: the chassis must be allowed to stand higher, and
"true chassis height is an IK problem — hip2 and the knee must work together". Dose:
`knee_tuck_target` 0.85 (P-e·h0) → 0.75 → 0.65 → 0.55 with `tibia_plumb_gain` 0.15, arena,
6000 ticks, n=6 (`…__plumb015__tuck0xx.json`). Verdict: `REGRESSION`, monotone in the dose.*

| knee tuck (+ plumb) | chassis_y (mm) | belly (mm) | belly min | foot_r (mm, reach 166) | shank formula | net_disp (m) | steps / 6000 |
|---|---|---|---|---|---|---|---|
| base 0.85, no plumb | **44.9** | 28.8 | 6.0 | 167.7 | 46° | 5.07 | 8.8 |
| 0.85 | 37.8 | 20.2 | 2.2 | 168.7 | 43° | 5.30 | 5.0 |
| 0.75 | 32.3 | 13.5 | 0.8 | 171.3 | 37° | 4.03 | 1.5 |
| 0.65 | 28.9 | 9.0 | 0.2 | 172.7 | 32° | 3.71 | 0.5 |
| 0.55 | **26.7** | 6.1 | 0.0 | 173.3 | 28° | 3.59 | 0.5 |

Falls 0 and 4.0 feet planted throughout: the body does not fall, it lies down.

**The direction was wrong, and the geometry says why.** Opening the knee extends the shank
OUTWARD and down from a femur that rests near horizontal: the feet move out (168 → 173 mm,
further past the 166 mm reach), the chassis descends toward the floor, the belly goes to zero,
and the gait stops stepping. The controller's "shank off-vertical" formula reads more and more
plumb as this happens, so that number is measuring the formula, not the posture — it is the
sprawl flattening, not the leg standing up. The ledger's closure ("feet plant at 170 mm against
a 166 mm reach") is a consequence of the body sitting LOW and WIDE, not a limit of the limb.

**The operator's IK point, made concrete by the sign convention in the code** (MotorEPMv2 Rule 5:
"+hip2 = press foot down"; `stance_lift_hip2`: "on a planted foot hip2 PLUS presses the foot
down and raises the chassis"): standing higher with the feet nearer the hips is the FEMUR
pressing down with the knee tucked, hip2 positive. The plumb reflex pulled hip2 NEGATIVE on the
robot (−0.035) to satisfy its formula, which is exactly the belly cost it paid in both the sim
and the robot. The next dose is therefore `hip2_tuck_target` (the femur's rest, off in
P-e·h0 = the spawn pose) run both ways, −0.3 / +0.3 / +0.6, plumb off, to settle the sign on
the body and read chassis_y, foot_r, belly, steps and falls. `stance_lift_hip2` (0.25 in
P-e·h0) is the same press, gated to planted legs, and is the second knob of the pair.

---

### ★★ 2026-10-12 — THE FEMUR DOSE: hip2 +0.3 STANDS THE BODY 19 mm HIGHER WITH THE FEET 7 mm INSIDE THE REACH, BELLY +23 mm, 0 FALLS, DISTANCE TIES (n=6) — THE OPERATOR'S STANCE, FOUND

*`hip2_tuck_target` on P-e·h0 (default −99 = the spawn pose), plumb off, −0.3 / +0.3 / +0.6, arena,
6000 ticks, n=6 (`…__hip2m030 / p030 / p060.json`). Sign per the code: "+hip2 = press foot
down". Verdict: **+0.3 `WORKING` on posture** (loud), distance tie, 0 falls — the promote-or-kill
candidate for the robot; +0.6 `REGRESSION` (falls, half the distance); −0.3 `REGRESSION` (the
body lies down — the direction the plumb reflex pushed).*

| hip2 rest | chassis_y (mm) | belly (mm) | belly min | foot_r (mm, reach 166) | planted | net_disp (m) | straight | steps | falls |
|---|---|---|---|---|---|---|---|---|---|
| −0.3 | 30.4 | 10.4 | 0.0 | 169.4 | 3.86 | 5.20 | 0.81 | 22.5 | 0 |
| base (spawn) | 44.9 | 28.8 | 6.0 | 167.7 | 3.97 | 5.07 | 0.78 | 8.8 | 0 |
| **+0.3** | **64.1** | **52.2** | **21.6** | **160.8** | 3.99 | 4.65 ± 0.43 | 0.74 | 2.8 | **0** |
| +0.6 | 80.8 | 74.8 | 37.9 | 148.5 | 3.98 | 2.93 ± 1.36 | 0.48 | 4.2 | 2 (one seed) |

**What it says.** Pressing the femur down with the knee tucked is the stance the operator
described: the chassis rises (45 → 64 mm), the feet come in from the reach limit (168 → 161
mm), the belly has margin for the first time (minimum 22 mm against 6), four feet stay
planted, nothing falls, and the body still travels (4.65 against 5.07, inside the seed spread).
At +0.6 it stands 81 mm high on 148 mm feet and starts to tip (one seed twice) and crawl. The
dose is monotone in height and the sweet spot is between the two; +0.3 is the arm that goes to
the robot. −0.3 is the plumb reflex's direction, and it lies the body down the same way.

**Two closures corrected.** (1) The "feet at 170 mm on a 166 mm reach" closure of 2026-08 was a
posture, not a limb limit: at hip2 +0.3 the feet plant at 161 mm. (2) The plumb formula's
"shank off-vertical" rises to 60° and 76° as the body stands up — the formula reads the knee's
fold under a raised femur as sprawl. It measures its own encoding, not the posture, which is
why the whole plumb family optimised toward a flatter body. Retire it as a posture metric;
chassis_y, belly and foot_r are the honest three.

**For the robot:** `…__s3a_stuckinst__hip2p030.json` (the reflex stack + hip2 +0.3), in the
dash. Read, against run 3 and the plumb run: the feet-in-contact count (2.8 / 2.73), the
stride mean (≈ 0), the belly ToF (27 / 20 mm median), the servo current (1.50 / 1.71 A — the
femur press holds the body up; the robot already stands 15–25 mm above the sim, so watch
the rail), falls, and the operator's eye on the shank and the steps. The room A/B of the
same stack with the taller stance is below.

**Addendum — the taller stance on the reflex stack in the room (`…__s3a_stuckinst__hip2p030`,
n=6 × 6000):** wall_frac **0.056 ± 0.050** against 0.008 for the stack at the spawn stance;
contact episodes 30 against 3.7; coverage 22.2 against 27.2; net_disp 1.04 against 0.74;
straight 0.18 against 0.12; **0 falls**; two seeds never touched a wall. The same shape as
the plumb arm in the room and the 0.6 m scale: a faster, straighter body reaches the wall
before the avoid loop, whose 1 m warning was fit to the sprawling gait, has turned it. The
stance is the gait lever; the loop's lead must follow the body's speed and turning authority
(S4's reach constant). For the robot run this means more wall touches than S3a had, with the
belly, the feet and the current as the numbers that decide the stance.

---

### ★★ 2026-10-12 — hip2 +0.3 ON THE ROBOT: THE STANCE REPRODUCES (BELLY 27 → 51 mm, CURRENT DOWN), THE STEPPING DOES NOT CHANGE — PROMOTED AS THE ROBOT'S POSTURE; THE NOISY STEP IS THE NEXT PROBLEM

*The operator's run of `…__s3a_stuckinst__hip2p030.json` (carpet, 225 s, 2 HAT resets).
Operator's eye: "the height is better but steps are still noisy; clean strides are still rare."
Against run 3 (the spawn stance) and the plumb run, same floor:*

| | run 3 | plumb 0.15 | **hip2 +0.3** |
|---|---|---|---|
| **belly ToF median / p05** | 27 / 9 mm | 20 / 5 mm | **51 / 32 mm** |
| servo current median / p95 | 1.50 / 2.98 A | 1.71 / 2.94 A | **1.38** / 3.06 A |
| hip2 (proprio, mean) | +0.019 | −0.035 | +0.183 |
| feet in contact (FSR ≥ 200) | 2.80 | 2.73 | 2.77 |
| stride forward mean / sd | −0.019 / 0.19 | +0.011 / 0.19 | −0.001 / 0.23 |
| **clean strides** (a ≥ 0.25 s swing then a ≥ 0.25 s stance, all legs) | **24 %** (302/1246) | 26 % | **20 %** (212/1077) |
| FSR contact transitions per minute per leg | 128–221 | 117–189 | 106–181 |
| upright p05 | 0.964 | 0.962 | 0.973 |
| HAT resets | 2 | 0 | 2 |

**Verdict: `WORKING` on posture, `NULL` on the step — promoted as the robot's posture.** The sim
predicted +23 mm of belly and the robot gave +24; the taller, less sprawled stance draws LESS
current at the median (1.50 → 1.38 A: less torque holding a narrower base), tilts less, and
costs nothing in falls. That is the operator's stance, measured, and it goes into the robot
stack. But the stepping is untouched: the same 2.8 feet in the FSR stance set, the same
zero-mean stride, the same one-in-four clean strides, the same FSR flicker — 100–200 contact
transitions a minute per leg against the ~40 a clean 1.4 s gait would make, i.e. **each foot
touches and lifts three to four times per stride.** That flicker is the "swimming" the operator
sees, and it is what the stride estimate, the stance set and the odometry are all reading.

**What the noisy step is.** The ledger's 2026-10-02 finding stands and is now the front: four
imposed terms in MotorEPMv2 (coupling, stroke, `stance_lift`, explore noise) each drive the
joint command to full slew with reversals every 1–2 ticks; benchd's slew (40 µs/tick) and the
servo lag (α 0.2) smooth what reaches the servos, so the feed's `out` shows no reversals while
the feet still skitter. Nothing on the robot records the brain's RAW action targets, so the
thrash cannot yet be attributed per term there. **Next, in order:** (1) an instrument — the host
logs its raw action targets (the values it sends benchd, before the slew) beside the applied
pulses, and a thrash metric on them (reversals per stride, per joint); (2) in the sim, the four
terms ablated one at a time as TESTS (never the operating mode) to attribute the reversals;
(3) the lever that follows from (2) — most likely a learned, rate-aware form of whichever term
owns the reversals, never a new imposed smoother. The clean-stride and flicker numbers above are
the metrics it must move.

**Robot stack now:** `…__room_senses__s3a_stuckinst__hip2p030.json`.

---

### 2026-10-12 — THE NOISY STEP, INSTRUMENTED AND ABLATED: THE BRAIN'S RAW TARGETS REVERSE EVERY 2–5 TICKS, NO SINGLE TERM OWNS IT, THE EXPLORE NOISE IS THE CHEAP FIRST LEVER — AND THE SIM'S GAIT IS A SHUFFLE, NOT THE ROBOT'S FLICKER

*Two instruments and one lesion study, after the operator's eye on the hip2 +0.3 run ("height
better, steps still noisy, clean strides rare").*

**The instrument on the robot.** `ogma_host --dump-actions` prints the brain's raw action
targets every tick, before benchd's slew (the dash passes it from now on; `thrash.py` reads it
beside the feed). Verified on the bench with the stand pose held, benchd in bench mode so the
commands were not applied (open loop): **29 reversals per 70-tick stride per servo, 30 % of
servo-ticks at the rail.** One reversal every 2.4 ticks — the 2026-10-02 finding, now a number
on the robot. The closed-loop figure comes with the next floor run.

**The lesion study in the sim** (P-e·h0 + hip2 +0.3, arena, 6000 ticks, n=6, one diagnostic
row per tick, `stepnoise.py`; each term set to 0 as a TEST, never an operating mode):

| arm | net_disp (m) | steps | falls | rev/stride hip1 | hip2 | knee | contact flicker /min | clean strides |
|---|---|---|---|---|---|---|---|---|
| base (hip2 +0.3) | 4.65 | 2.8 | 0 | 11.1 | 15.9 | 13.8 | 1.7 | 0 % |
| coupling 0 (was 1.509) | 2.33 | 1.2 | 1 | 11.1 | 19.6 | 21.4 | 0.5 | — |
| stroke 0 (was 1.2) | 1.07 | 1.0 | 0 | 14.1 | 16.4 | 15.0 | 1.2 | — |
| stance_lift 0 (was 0.5) | 2.57 | 1.8 | 0 | 10.7 | 18.7 | 25.2 | 0.9 | — |
| **explore noise 0 (was 0.05)** | **4.94** | **6.2** | 1 | **8.7** | **13.1** | 13.7 | 3.4 | 14 % |

**What it says.** (1) The commanded targets reverse 11–16 times per stride per joint in the
sim's base gait — every 4–5 ticks — and no lesion removes it: coupling, stroke and stance_lift
each raise the reversals when removed (the remaining terms thrash more without them) while
halving progress, so the thrash is the controller's output as a whole, not one term's. (2) The
explore noise is the one term whose removal LOWERS the reversals (hip1 −22 %, hip2 −18 %) at no
cost — net 4.94 against 4.65, twice the steps — with one fall in six (a signal to read on the
robot, not a verdict). That makes `explore_noise 0` the cheap first lever:
`…__hip2p030__explore0.json`, in the dash. (3) **The sim's gait is not the robot's.** In the sim
the feet almost never leave the ground (swing bouts of 0.07 s, contact flicker 1–3 a minute,
0 % clean strides): the body advances by scrubbing, and it still covers 4.65 m. On the robot
the feet flicker 100–200 times a minute and the body goes nowhere. The thrash is common to
both; what differs is the foot–ground contact — the sim's floor lets a scrubbing foot push,
the carpet does not. That is the sim2real gap under the whole gait question, and no lever in
the controller closes it from the sim side; it is a body-model fact (friction, compliance,
the FSR's threshold as a stance test) to be measured and put into the sim before the sim can
be trusted on stepping. The sim's shank-angle formula was retired this morning for the same
reason: it measured its encoding.

**Also:** per-tick diagnostics (`OGMA_PICRAWLER_DIAG_INTERVAL=1`) are an instrument that may
cost determinism — the base arm's chassis_y read 74 mm at interval 1 against 64 mm at
interval 60 on the same config and seeds. Checked on seed 1: the body's (x, z, yaw) are identical at
the start, the middle and the last tick of both runs (100 of 115 common rows bit-equal, the rest
transient), so the physics is the same and the 10 mm is in how the harness averages chassis_y
over rows of different density — a harness fact to fix, not a body fact.

### ★★★ 2026-10-12 — THE LOAD-HELD PER-LEG PHASE (LoadPhaseGait, Owaki's Tegotae rule): STEPS ×17, CLEAN STRIDES 0 → 38 %, HIP REVERSALS ÷2.5, DISTANCE +35 %; THE FREE-CLOCK CONTROL COLLAPSES; THE COST IS WOBBLE AND ~1 FALL PER RUN (`WORKING`, signal, n=6)

*The lever the gait mechanism audit proposed
([`picrawler_gait_mechanism_audit_2026-10-12.md`](picrawler_gait_mechanism_audit_2026-10-12.md)),
built and measured the same day. Operator direction: "proceed autonomous until you have a
signal of efficacy."*

**The mechanism.** A new module, `LoadPhaseGait` (`cpp_core/src/ogma/modules/LoadPhaseGait.cpp`),
owns one phase per leg: `dφ_i = ω (1 − σ_r · N_i · cos φ_i)`, ω = 2π/70 ticks, N_i = foot i's
SHARE of the published foot load (`reality.proprio.foot_load`; the contact flags when no load is
published). Swing is φ ∈ (0, π), touchdown at π, stance (π, 2π). A loaded leg is pulled toward
mid-stance and does not lift; an unloaded leg runs free. No inter-leg phase is prescribed. It
publishes `rhythm.leg.<leg>` = [cos φ, sin φ, swing, N]. MotorEPMv2 gained three sockets, each
behind its own gain-0 guard: `leg_phase_stroke` (the hip stroke rides the external phase,
offset `leg_phase_offset` −π/2, so the hip target is −cos φ: forward at touchdown), `leg_phase_swing`
(the stance/swing state every gated term reads is the phase's half-cycle, not the attitude
detector) and `leg_phase_lift_hip2/knee` (y += gain · max(0, sin φ): a smooth swing lift).
`coupling_gain` 0 in the lever arms. Unit tests: `cpp_core/tests/ogma/test_load_phase_gait.cpp` (7).

**Protocol.** P-e·h0 + hip2 +0.3 (the lesion study's base), arena, difficulty 0.3, n=6 × 6000 ticks,
seeds 1–6, `OGMA_PICRAWLER_DIAG_INTERVAL=1` in every arm, `arenaavg.py` + `stepnoise.py` + the
module's own instruments in the log (`lp_*`). Gain-0 guard: the SHADOW arm (module present and
publishing, every socket 0) is **byte-identical to the base on all 6 seeds, all 6000 rows** (x, z,
yaw, the three joint arrays). Consumer check: `lp_on` 1, `lp_stroke` = 4 legs × every post-warmup
tick, `lp_swing` every tick, `lp_lift` mean 0.26. Structure control: `lpg_free` = the same with
σ_r = 0 (a free clock, no load coupling — the open-loop sequencer of 2026-07-20).

| arm | net_disp (m) | straight | steps | clean strides | rev/stride hip1 / hip2 / knee | swing / stance (s) | step_cv_real | tilt_sd | falls | bellyc / min (mm) |
|---|---|---|---|---|---|---|---|---|---|---|
| base (hip2 +0.3) | 4.65 ± 0.43 | 0.28 | 2.8 | 0 % | 11.1 / 15.9 / 13.8 | 0.07 / 40.6 | 0.75 | 0.057 | **0** | 52 / 13 |
| shadow | 4.65 ± 0.43 | 0.28 | 2.8 | 0 % | identical | identical | 0.75 | 0.057 | 0 | identical |
| **lpg** (σ_r 2, lift −0.3 / +0.2) | **6.26 ± 1.69** | 0.31 | **47.7** | **38 %** | **4.4 / 9.7 / 11.4** | **0.24 / 1.5** | **0.64** | 0.242 | 1.17 | 45 / 3 |
| lpg_free (σ_r 0, control) | 0.64 ± 0.67 | 0.03 | 35.3 | 22 % | 4.7 / 15.6 / 20.4 | 0.13 / 2.3 | 0.73 | 0.503 | **6.5** | 53 / 2 |
| lpg_nolift | 4.96 ± 2.05 | 0.24 | 26.5 | 29 % | 4.4 / 11.5 / 12.3 | 0.20 / 2.5 | 0.67 | 0.251 | 1.5 | 52 / 2 |

Dose arms (same protocol):

| arm | net_disp | steps | clean | rev hip1 / hip2 / knee | swing (s) | falls | lp_il | lp_retro | lp_w |
|---|---|---|---|---|---|---|---|---|---|
| lpg (σ_r 2) | 6.26 | 47.7 | 38 % | 4.4 / 9.7 / 11.4 | 0.24 | 1.17 | 0.63 | 0.009 | 0.79 |
| σ_r 1.5 | 4.21 ± 1.98 | 40.7 | 31 % | 4.3 / 9.4 / 10.5 | 0.18 | 1.33 | 0.84 | 0.000 | 0.87 |
| σ_r 3 | 5.14 ± 1.48 | 24.8 | 35 % | 9.8 / 15.9 / 22.2 | 0.20 | 1.33 | 0.40 | **0.102** | 0.59 |
| lift ×0.5 (−0.15 / +0.1) | 5.80 ± 1.36 | 38.2 | 29 % | 4.3 / 10.6 / 11.9 | 0.21 | **0.50** | 0.73 | 0.005 | 0.82 |
| lift ×2 (−0.6 / +0.4) | 6.08 ± 2.40 | 67.5 | **58 %** | 6.0 / 10.6 / 14.8 | **0.32** | 1.33 | 0.68 | 0.006 | 0.79 |

**What it says.**
1. **The step is loud** (CLAUDE.md §3.3: proto-gait steps). Real swings go from 0.07 s bouts on a
   body that sat in stance for 40 s at a stretch to 0.24 s swings every 1.5 s of stance; strides
   per run 6 → 70; clean strides from none to 38 % (58 % at double lift); the stride period
   settles at 89 ticks (= 70 / `lp_w` 0.79: the load slows the free clock by a fifth). The
   inter-touchdown CV falls 0.75 → 0.64, the second lever ever to move it and the first to do so
   while distance rises.
2. **The thrash halves at its source.** Hip1 reversals per stride 11.1 → 4.4 and hip2 15.9 → 9.7
   with the stroke on a monotone phase (`lp_retro` 0.9 % of leg-ticks against the knee readout's
   65 %). The knee's 13.8 → 11.4 is the smaller move because the posture spring, the stance
   press and the HK term still write to it every tick; that is the next place to look.
3. **The load coupling is the mechanism, not the clock.** The free clock (σ_r 0) steps as often
   (35/run) and goes nowhere (0.64 m, straight 0.03, 6.5 falls, tilt_sd 0.50): the 2026-07-20
   flopping fish reproduced as a control. Holding a loaded leg in stance is what turns the same
   stepping into transport and support.
4. **Over-holding over-reaches.** At σ_r 3 the phase runs backwards 10 % of leg-ticks, the
   stride slows to 0.59 of the free rate, steps halve and the reversals return to the base's;
   σ_r 1.5–2 is the band. The lift dose trades clean strides (29 → 38 → 58 %) against stability
   (falls 0.5 → 1.2 → 1.3) and chassis height (76 → 75 → 66 mm).
5. **Inter-leg coordination is now measurable and non-null.** `lp_il` (pairs holding a constant
   relative phase, EMA over ~100 ticks) reads 0.63–0.84 in the lever arms against an EMA null of
   ~0.1; the free clock's 1.00 is four identical clocks, trivially locked. Which gait the four
   phases settle into (walk order vs diagonal pairs) is unread: the per-pair relative phases are
   in `diag_snapshot` but not yet mirrored to the log.

**Costs, stated.** Falls 0 → 1.2 per 6000 ticks (0.5 at half lift), tilt_sd ×4, belly minimum to
zero on 4/6 seeds (brief touches during the wobble; the mean clearance holds at 45 mm vs 52), and
`straight` is unmoved at 0.3. Two of the six lever seeds crossed the arena's 8 m safe radius, so
their `falls` are not trustworthy; the four inside read 1, 2, 0, 2. A body that now swings a leg
every 1.8 s is being stabilised by a posture spring, a stance press and a heading hold that were
tuned on a body that never did; the wobble is the base's gains, not yet a verdict on the lever.

**Verdict: `WORKING` (signal, n=6, one body, one gym, 6000 ticks).** Loud enough to promote the
direction, not a finding. Handed to the operator for UI observation (§3 rule 5) before any
promotion: configs `…__hip2p030__lpg.json` (sim arm) and
`…__room_senses__s3a_stuckinst__hip2p030__lpg.json` (the robot stack + lever, in the launcher and
the dash), with the dose arms beside them.

**Next, in order.** (a) The operator's eye on the sim arm and the robot stack arm. (b) Re-settle
the base's stabilisers on the stepping body: `postural_gain`, `stance_lift_gain`, the heading
hold's D term — one at a time, judged on falls and tilt with the step metrics held. (c) Mirror the
per-pair relative phases and read which gait the legs choose, and whether it changes with
`period_ticks` (Owaki's walk→trot transition is the operator's "trot or one leg at a time"
question, as a measurement). (d) The knee's remaining reversals: attribute them with the same
lesion method (posture spring, press, HK) now that the phase is clean. (e) The robot: the FSR
publishes `foot_load` already; `thrash.py` is the instrument; the first floor run is the
operator's. (f) Long runs (≥ 100 k ticks) before any stability claim (the 2026-10-02 lesson).

**Files.** `cpp_core/{include/ogma/modules,src/ogma/modules}/LoadPhaseGait.{hpp,cpp}`,
`cpp_core/tests/ogma/test_load_phase_gait.cpp`, `cpp_core/CMakeLists.txt`,
`cpp_core/src/ogma/ModuleRegistry.cpp`, `MotorEPMv2.{hpp,cpp}` (the three sockets + diag),
`godot_host/src/OgmaBrain.cpp` (diag mirror), `picrawler_body.gd` (`lp_*` log fields),
`launcher.gd` (two entries), eight configs under `configs/…__hip2p030__lpg*.json` and the robot
stack `…__s3a_stuckinst__hip2p030__lpg.json`.

### ★★★ 2026-10-13 — ON THE LOAD-HELD PHASE THE FALLS ARE THE HIP SWEEP: THE STROKE GATED BY THE LEVEL ERROR (`WORKING`, signal) AND A SHORTER STROKE (`WORKING`, signal) REMOVE THEM; THE CORNER-ALTITUDE REFLEX ON PLANTED LEGS IS `NULL`

*Operator UI observation on the lever (2026-10-13): "the hip one range of motion can cause the
chassis to tilt and tip overs to happen. However, the stride is confident and the leg movement
is much smoother. The robot should be reducing the error of keeping its chassis level." Then:
"all planted legs will be responsible for maintaining the altitude of the chassis corner where
they connect to keep level."*

**Protocol.** Base = `…__hip2p030__lpg` (the 2026-10-12 lever: LoadPhaseGait σ_r 2, stroke and
stance gate on the load-held phase, lift −0.3/+0.2, coupling 0). Arena, difficulty 0.3, n=6,
seeds 1–6, per-tick diag. First batch 6000 ticks; the rest **4500 ticks**, because the stroke
arms reached the arena's 8 m safe radius at 6000 (two lever seeds already had). The tilt token
(`reality.proprio.tilt`, honest: the modelled accelerometer's up vector, `PUBLISH_TILT=1`) is
subscribed in every attitude arm; **publishing it with every gain 0 reproduces the lever to the
digit** (`lpg_tilt0` = `lpg` on all metrics), so the token is a clean control.

**1. The fall anatomy (new instrument, `falls.py`: the half second before each tilt ramp).**
Across 10 falls on the lever: **none has two legs airborne** (one has one leg up); each is a
65–120-tick tilt ramp (one stride) with the feet nominally down, a **1.3–1.9 rad hip1 sweep on
one planted leg** in the preceding half second (the others 0.5–1.0), sideways velocity
0.2–0.3 m/s, chassis 59–72 mm. The monotone phase lets the stroke complete the full ±1.2 (clipped
±1) sweep every stride, which the retrograde knee readout never did; a sprawled planted leg's
full sweep shoves the chassis sideways and over. The operator's eye, as a trace.

**2. Corner-altitude reflex on planted legs (`level_gain`, `level_knee_gain`).**
`y[hip2] += g·e`, `y[knee] += g·e`, `e = front·sin pitch + left·sin roll` (front = FL,FR; left =
FL,RL), planted legs only (the load-held phase's stance half). Consumer verified (`lv_ticks` ≈
10 000/run, mean |e| 0.13 → 0.08 under the reflex). The femur-only form first, then the whole leg:

| arm (4500 unless noted) | net_disp | straight | falls | tilt_sd | steps | rev hip1 |
|---|---|---|---|---|---|---|
| lpg (6000) | 6.26 | 0.31 | 1.17 | 0.242 | 47.7 | 4.4 |
| hip2 only +0.5 (6000) | 4.49 | 0.23 | 1.17 | 0.221 | 43.7 | 4.6 |
| hip2 only +1.0 (6000) | 5.23 | 0.26 | 1.50 | 0.405 | 54.8 | 5.0 |
| hip2 only −0.5 (6000, wrong sign) | 4.85 | 0.24 | 1.33 | 0.287 | 60.2 | 5.2 |
| **lpg (4500)** | 4.94 ± 1.26 | 0.33 | 1.00 | 0.242 | 37.2 | 4.1 |
| hip2+knee 0.5 | 5.27 | 0.33 | 1.00 | 0.207 | 38.8 | 4.1 |
| hip2+knee 0.75 | 4.61 | 0.29 | 0.83 | 0.201 | 37.7 | 3.8 |
| hip2+knee **1.0** | 5.10 | 0.33 | **0.33** | 0.161 | 40.5 | 3.7 |
| hip2+knee 1.5 | 4.76 | 0.30 | 1.00 | 0.222 | 36.5 | 4.4 |
| hip2+knee 2.0 | 4.66 | 0.28 | 1.33 | 0.279 | 40.0 | 4.1 |
| hip2+knee −1.0 (wrong sign) | 3.21 | **0.18** | 0.83 | 0.203 | 29.7 | **7.2** |
| stroke 0.7 + hip2+knee 1.0 | 5.89 | 0.43 | 0.50 | 0.190 | 38.3 | 4.6 |
| stroke 0.9 + hip2+knee 1.0 | 6.92 | 0.47 | 0.67 | 0.160 | 39.2 | 4.6 |

**Verdict `NULL` (n=6).** The dose is non-monotone (0.75 → 0.83 falls, 1.0 → 0.33, 1.5 → 1.00),
so the one good point is not a mechanism's; stacked on the shorter strokes it RAISES their falls
(0.17 → 0.50, 0.33 → 0.67); and the wrong sign degrades straightness, reversals and step
regularity, which proves the term has authority — it can make the body worse, and the right sign
does not make it better. Mechanism, from the anatomy: the leg whose sweep is tipping the body is
itself planted, so pressing the low corners harder presses THAT leg harder too; the reflex acts
on the effect while the cause continues. Re-use context: a corner-altitude error that enters the
STANCE PRESS (redistributing a fixed press by corner, so a leg that is shoving cannot be pressed
up more) or that reaches the leg's length through the learned model (the duck's state-prior
pattern, which needs attitude in the leg's state vector) — both untested.

**3. The hip range, as a dose (`stroke_gain`, the operator's observation):**

| stroke_gain (4500) | net_disp | straight | falls | tilt_sd | steps | clean strides | swing s |
|---|---|---|---|---|---|---|---|
| 1.2 (lpg) | 4.94 ± 1.26 | 0.33 | 1.00 | 0.242 | 37.2 | 40 % | 0.24 |
| 1.05 | 5.89 | 0.40 | 0.33 | 0.215 | 44.3 | 40 % | 0.23 |
| 0.9 | 7.48 ± 1.00 | 0.51 | 0.33 | 0.135 | 50.0 | 44 % | 0.23 |
| **0.7** | **7.60 ± 1.68** | **0.55** | **0.17** | **0.101** | 40.7 | 38 % | 0.24 |
| 0.5 | 6.27 | 0.55 | **0.00** | 0.068 | 23.3 | 21 % | 0.17 |

**`WORKING` (signal).** Monotone on tilt and falls down to 0.5; distance and straightness peak
at 0.7–0.9 (the full stroke was costing transport, not buying it — at 1.2 the hip sits at the
clamp 42 % of its cycle); at 0.5 the stepping thins (steps 23, clean 21 %). The "shorter stride"
row of §6 (2026-07) had this as PARTIAL on the old phase; on a monotone phase it is loud.

**4. The stroke gated by the level error (`stroke_level_gain`): amp × max(floor, 1 − g·|tilt|),
|tilt| = √(sin²p + sin²r), propulsion term only (the heading-hold steer keeps full authority).**
The planted legs withhold their push while the chassis is off level and push fully when it is.

| arm (4500) | net_disp | straight | falls | tilt_sd | steps | clean | chassis | mean gate |
|---|---|---|---|---|---|---|---|---|
| lpg | 4.94 | 0.33 | 1.00 | 0.242 | 37.2 | 40 % | 74 mm | 1.00 |
| gain 1 | 6.87 | 0.45 | 0.50 | 0.169 | 41.3 | 39 % | 73 | — |
| gain 2 | 5.85 | 0.43 | 0.50 | 0.137 | 45.8 | 47 % | 75 | 0.31 (s1) |
| **gain 4** | **6.79 ± 0.63** | **0.50** | **0.17** | **0.079** | **47.7** | 34 % | **78** | 0.38 (s1) |
| gain 2 + stroke 0.9 | 7.20 | 0.49 | **0.00** | 0.101 | 41.5 | 31 % | 79 | — |
| **stroke 0.5 (the matched-mean control)** | 6.27 | 0.55 | 0.00 | 0.068 | **23.3** | **21 %** | 74 | const |

**`WORKING` (signal), and the control says what the gate adds.** At gain 4 the mean gate is
~0.4, i.e. on average the same amplitude as a constant stroke of 0.5; against that matched
constant it keeps **twice the steps (48 vs 23) and the clean strides (34 vs 21 %)** at equal
falls and tilt, with the chassis 4 mm higher. The gate withholds the push only when the chassis
is off level and gives the full stroke back when it is, which a constant cannot do. This is the
rewrite rule's shape: the error (attitude vs level) gates the term that produces it, on the
state that makes the gate valid. It is the arm offered for the operator's next observation,
as the sim arm `…__hip2p030__lpg_sl4.json` and the robot stack
`…__room_senses__s3a_stuckinst__hip2p030__lpg_sl4.json`; the shorter constant stroke (0.7) is
the plain alternative, and both can stack (gain 2 + 0.9: 0 falls, 7.2 m).

**Caveats.** 4500-tick runs, one body, one gym; falls on a stepping body are a 100 k-tick
question (2026-10-02). The step metrics under the gate are read at ~0.4 mean amplitude, so a
later re-settle of the lift and the hold (σ_r) on the gated stroke is expected. `lp_il` (pairs
at constant relative phase) sits 0.6–0.75 in every lever arm; which gait that is remains
unread (the per-leg phases are now in the log as `lp_ph`).

**Files.** `MotorEPMv2.{hpp,cpp}`: `level_gain`, `level_knee_gain`, `stroke_level_gain`,
`stroke_level_floor` (+ `lv_*`, `sl_mult`, `tilt_p/r`, `lp_ph` in the body log; the generic
loop-module diag mirror in `OgmaBrain.cpp` now passes per-leg arrays). Configs
`…__hip2p030__lpg_{tilt0,lv05,lvm05,lv1,lvk05,lvk075,lvk1,lvk15,lvk2,lvkm1,sl1,sl2,sl4,sl2_st09,st05,st07,st09,st105,st07_lvk1,st09_lvk1}.json`;
the robot stack `…__s3a_stuckinst__hip2p030__lpg_sl4.json`. Instrument `~/xaq_runs/lpg_20261013/falls.py`.

### ★★★ 2026-10-13 — IN THE ROOM THE LEVER FALLS TEN TIMES PER RUN, AND THE CAUSE IS THE TURN: the avoid loop's steer rides the hip stroke; gating push AND steer by the level error, with a 0.7 stroke, brings it to 0.33 (`WORKING`, signal) — the robot-stack candidate

**Protocol.** The robot stack (`…__room_senses__s3a_stuckinst__hip2p030`: ProxMemory → TofAvoidLoop
→ BearingToIntent on the speed and goal-bearing sockets, StuckSense as an instrument), room 3 m,
n=6 × 12 000 ticks, `roomavg.py`; one arm re-run at per-tick diag for the anatomy.

| arm (room, 12 000) | falls | coverage (cells) | wall_frac | episodes | net_disp |
|---|---|---|---|---|---|
| ROBOT STACK (benchmark) | **0.00** | 37.7 ± 7.4 | 0.047 | 51 | 1.11 |
| + LPG | **10.17 ± 2.3** | 46.0 | 0.036 | 24 | 0.39 |
| + LPG + level gate 4 on the push only | 9.17 ± 5.6 | 37.8 | 0.053 | 29 | 0.46 |
| + LPG + stroke 0.7, no gate | 4.67 ± 2.3 | 41.2 | **0.218** | 71 | 0.53 |
| + LPG + level gate 4 on push AND steer | 2.50 ± 2.5 | **51.0 ± 3.2** | **0.023** | 18 | 0.66 |
| **+ LPG + gate on both + stroke 0.7** | **0.33 ± 0.8** (5/6 seeds 0) | 42.5 ± 3.3 | 0.043 | 25 | 0.63 |

**The anatomy (per-tick, 59 falls).** Not at the walls (0/61 within 0.3 m of one), not late (spread
over the quarters), ramp median 76 ticks as in the arena. **Before a fall the bearing error averages
0.33 against 0.15 overall; 41 of 59 falls happen above 0.2, where only a quarter of all ticks
sit.** The heading hold (gain 7 on the bearing error in units of π) folds a steer of ~1 into the hip
stroke as a left/right differential, so during a commanded turn one side's hip sweeps the full
clipped range every stride — the arena's fall anatomy, now produced on demand by the avoid loop.
In the arena the hold has no goal and the error is near zero, which is why 2026-10-12 never saw it.
The push-only gate spared the steer by design (so the promoted heading hold kept its authority),
and the room measured that choice: it does nothing there (9.2). A turn is a push.

**Verdicts.** `stroke_level_steer` = 1 (the gate scales the steer differential too): `WORKING`
(signal) — falls ÷4, the best coverage of any arm and the least wall contact. Stacked with the
0.7 stroke: falls 0.33, coverage above the benchmark's, wall contact at the benchmark's, five seeds
of six without a fall over 12 000 ticks. Stroke 0.7 ALONE in the room is a `REGRESSION` on walls
(0.22): the shorter stroke without the gate turns too little to clear them. The candidate for the
operator's eye and the robot is `…__s3a_stuckinst__hip2p030__lpg_sl4s_st07.json` ("ROBOT STACK +
LPG + gate (both) + stroke 0.7" in the launcher), with `…__lpg_sl4s.json` as the gate-only form.

**Open.** The remaining 0.33 is not yet the benchmark's 0 — but the benchmark does not step (2.8
steps per 6000 in the arena); the candidate steps ~40 times per 4500 with the room's turns. The
steer's authority is now shared with the level gate, so turning while tilted is withheld; whether
the avoid loop's lead (1 m) still fits this gait is S4's reach-constant question. Long runs and the
robot floor run next; the hold strength σ_r and the lift were tuned before the gate and may
re-settle.

### ★★★ 2026-10-13 — TERRAIN AND THE LOW CORNER: the per-leg corner integrator levels the roll and halves the pitch (`WORKING`, signal); the absolute level gate stalls on a slope (diagnosis); the gait's lean is physics the brain cannot see, not a sign — four geometry checks, two refuted levers, and the sensor the corner rule needs

*Operator (sim, corridor): "it was not able to adapt to that difference in elevation between the
left side and the right side"; "the right rear leg holds its corner of the chassis lower than the
rest … reproduced on flat ground"; "every leg is equal so asymmetry should not be a normal outcome."*

**Protocol.** Base = the candidate's sim twin `…__hip2p030__lpg_cand` (LPG, gate 4 on push and
steer, stroke 0.7). Arena n=6 × 4500 (flat ground, per-leg read: `perleg.py`), corridor n=6 ×
6000 at difficulty 0.3 (ramp and 30° walls; `seedavg.py`), per-tick diag throughout; one traced
run (`OGMA_PICRAWLER_TRACE`) and one no-brain hold (`babble_scale 0`, every servo at the same
command for the whole run).

**1. What the new module learns: nothing.** `LoadPhaseGait` has three constants (period, hold
strength, load floor) and four phase states; every socket in the motor layer is a constant. The
learned parts underneath are unchanged (per-leg forward model and HK controller, the keyframe
map, the amplitude homeostat, the heading trim). Two ratchets explained the slope: the level
gate's reference is absolute gravity, so a steady lean withholds the push for good (at gain 4
and floor 0 a 14° lean stops the stroke); and the posture spring pins all four legs to one rest
pose, so no leg can hold its corner. Both are mine.

**2. The gate referenced to the running attitude (`stroke_level_ref_tau` 300): `REGRESSION`
on flat ground** — steps 18 → 45 but tilt_sd 0.044 → 0.131 and net 6.0 → 5.5. It revealed a
confound in the candidate: the steady 7° standing pitch had been shortening the stroke through
the absolute gate (mean gate ~0.7 while "level"), so part of the candidate's calm is a shorter
stroke. Kept off; re-use: with the lean itself removed (below).

**3. The per-leg rest-length integrator on planted legs (`level_int_gain`, the operator's
rule): `WORKING` (signal).** Each planted leg integrates its corner error (front/left projection
of pitch and roll, absolute) into its own hip2 + knee offset, bounded ±0.3, frozen when not
upright, zeroed on reset; `level_int_planted_only` 1 applies it only while planted (with it
applied in swing too the right rear's +0.13 offset cancelled its swing lift).

| arm | sin pitch | sin roll | offsets FL/FR/RL/RR | arena net / falls | corridor net_z / straight / falls |
|---|---|---|---|---|---|
| candidate | −0.068 | −0.028 | — | 6.02 / 0 | 5.16 ± 0.47 / 0.35 / 0 |
| + integrator 0.002 | −0.041 | −0.001 | −0.06 / −0.02 / +0.03 / +0.09 | 6.34 / 0 | 5.73 ± 0.36 / 0.37 / 0 |
| + integrator 0.005 | −0.034 | +0.003 | −0.07 / −0.04 / +0.03 / +0.13 | 6.21 / 0 | — |
| + 0.005 planted-only | −0.041 | −0.005 | −0.08 / −0.03 / +0.03 / +0.12 | 6.38 / 0 | **5.72 ± 0.65 / 0.38 / 0** |
| benchmark (old gait) | n/a | n/a | — | — | 4.23 ± 0.32 / 0.26 / 0 |

The roll is levelled and the pitch halved, at no cost on flat ground, and the corridor gains
11 % over the candidate (35 % over the benchmark) with zero falls. The largest positive offset
lands on the right rear, where the operator saw the low corner. The pitch is only halved
because the rear legs sit near full reach (feet at 160 mm of 166) and can only be levelled by
shortening the front, which the integrator does slowly.

**4. The asymmetry audit: no sign or geometry mistake found.** (i) Hinge axes are built per
leg from that leg's heading (`_build_leg`: hip2 axis = up × heading, hip1 axis = up), so "+"
presses every femur down and yaws every hip the same way; the four headings are exact ±45°
mirrors. (ii) Stroke direction, measured: every foot moves backward while loaded (−1.3 to
−2.0 mm/tick) and forward in swing (+1.7 to +2.7), with the left/right mirror the sign array
encodes. (iii) Hip means, sign-corrected: all four trail neutral by 0.04–0.09 rad, identical
with the learned controller off. (iv) The body file is one value per link. (v) The no-brain
hold stands within 0.7° of level with a diagonal 60/40 load split (FR+RL heavier) and the
right rear foot 3 mm deepest. The gait's lean (pitch −0.05 to −0.09 in every seed) is present
from the first quarter and never grows: not a ratchet.

**5. Where the lean comes from, and why the rear feet do not rise.** Trace, per leg: all four
femurs are commanded to the same lift bottom and reach it (achieved 0.06); all four feet
UNLOAD in their swing (contact 0.02 vs 0.35 planted); the front feet rise 28/16 mm, the rear
9/1 mm. The difference is the stance femur angle: front 0.23/0.16 rad, rear 0.135/0.134 under
near-identical commands — the rear femurs sag ~0.04 rad more under load (the sim's weak
position motor), the nose comes up, and from a lower hip the same absolute lift bottom clears
nothing: the rear feet scrub forward on the floor, unloaded. Lesions (learning off, lift ×2,
posture spring ×0.5) leave the lean and the non-rising rear unchanged: structural.

**6. Two levers on that mechanism, both refuted in context.** (a) `stride_center_gain` 0.002
(per-leg homeostat driving the hip's time-average to neutral): it acted (offsets +0.12 to
+0.19, all forward) and the pitch did not move; distance fell (arena 6.38 → 5.40, corridor
5.72 → 4.68). The "trailing stroke loads the rear" hypothesis is refuted. `REGRESSION`.
(b) `lift_relative_gain` (lift the femur by its measured stance sag): the measured sag was
0.00–0.02 — **under honest joints the brain's joint state is the servo model's output, not the
physics angle**, because the real servos report nothing. The brain cannot see sag, in the sim
or on the robot. `DEAD_CODE` on measurement, not on the idea.

**7. The sensor the corner rule needs.** The corner altitude is unobservable from joints
under honest inputs. It is observable from the commanded forward kinematics rotated by the
IMU (`feet_y_gravity_cmd_imu` with `OGMA_PICRAWLER_CMD_FK_SOURCE=1`; the zero-pose form is
what the robot host publishes today): each foot's height under gravity from the command and
the attitude, both of which the robot has. With it, a planted leg's corner error becomes that
foot's commanded depth relative to the others' (the integrator's input), and a swing leg's
clearance becomes readable, so a lift can be lengthened until the commanded foot clears — the
robot form of lever 6(b). That is the next build; the pitch/roll corner map stays as its
fallback.

**Files.** `MotorEPMv2`: `stroke_level_ref_tau`, `level_int_gain/_max/_planted_only`,
`stride_center_gain/_tau/_max`, `lift_relative_gain/_tau` (+ `lv_off`, `lv_sc_off`, `lv_sag`,
`lp_sh`, `tilt_pref` in the log). Configs `…__hip2p030__lpg_cand*.json`, `…__hold0.json`, the
robot stack `…__lpg_sl4s_st07_int.json`. Instruments: `~/xaq_runs/lpg_20261013/perleg.py`,
the hold-and-trace protocol. Launcher: the integrator candidate added beside the candidate.

> **AMENDMENT (2026-10-13, later) to §5 and §7 above — the "rear feet do not rise" reading was
> the proxy, and the ledger had already recorded this trap (2026-07-25).** The per-leg foot height
> used for it (`feet_y`) is the shank centre minus half the shank: a shank-angle approximation,
> and the rear shanks sit more sprawled. The physics contact flag, per leg (the attribution trace,
> seed 1, the candidate): airborne FL 33 %, FR 19 %, RL 31 %, RR 11 %, lift-offs 31/38/57/25 in
> 3000 ticks. **The rear legs lift.** The true asymmetry is right/left — the right side airborne
> about half as often as the left, the right rear least — which is the roll the attitude integrator
> removes, not a rear-leg lift deficit. §5's mechanism for "the rear foot scrubs" is therefore
> withdrawn as stated; the stance-femur sag difference it measured is real but small. The
> swing-clearance socket (`swing_clear_*`, the commanded-FK swing foot vs the planted plane) was
> built against the proxy's picture; its first read (clearance alone, arena n=6: net 5.87, 0
> falls, steps 24; corridor net_z 5.08, a tie with the candidate) is recorded as `NULL` pending a
> re-read on the true per-leg contact, which the body log now carries (`con`). The commanded-FK
> source for the PLANTED corner error (`level_int_src` 1) is a `REGRESSION` for a measured reason:
> among planted feet the commanded depth cannot differ in reality, so a deeper commanded foot is
> that leg's sag, not a higher corner, and the integrator ran the right rear the wrong way (pitch
> −0.07 → −0.12, corridor net_z 5.7 → 1.9). The attitude map stays the planted corner's source.
> Publishing the commanded FK with nothing consuming it is byte-identical to the candidate (`fk0`).

### ★★ 2026-10-13 — THE COMMANDED-FK SWING CLEARANCE on the attitude integrator: corridor net_z 5.16 → 6.30 (`WORKING`, signal, sim-only); and the flat-ground validation BY TRUE CONTACT — every leg lifts more under the lever than under the benchmark, the right rear least

**Protocol.** Candidate's sim twin + the attitude corner integrator (planted-only, 0.005) +
`swing_clear_m` 0.025 / 0.040, `swing_clear_gain` 0.05, `swing_clear_max` 3, body
`CMD_FK_SOURCE=1`. Corridor n=6 × 6000 (difficulty 0.3), arena n=6 × 4500. The body log now
carries the per-leg physics contact (`con`); every per-leg airborne figure below is from it.

| corridor arm | net_z | straight | falls | tilt_sd | true airborne FL/FR/RL/RR |
|---|---|---|---|---|---|
| benchmark (old gait) | 4.23 ± 0.32 | 0.26 | 0 | 0.062 | (proxy only) |
| candidate | 5.16 ± 0.47 | 0.35 | 0 | 0.053 | (proxy only) |
| + attitude integrator | 5.72 ± 0.65 | 0.38 | 0 | 0.048 | (proxy only) |
| + integrator + clearance 25 mm | 5.71 ± 0.94 | 0.39 | 0 | 0.052 | 0.26 / 0.27 / 0.17 / 0.15 |
| **+ integrator + clearance 40 mm** | **6.30 ± 0.70** | **0.43** | 0 | 0.052 | — |
| clearance alone (no integrator) | 5.08 ± 0.49 | 0.36 | 0 | 0.058 | — |

**`WORKING` (signal).** The clearance only pays on the levelled body (alone it ties the
candidate); on the integrator it adds 10 % at 40 mm, with straightness up and no falls. The
lever reads the commanded foot's height under gravity against the planted feet's commanded
plane near mid-swing and scales that leg's phase lift toward the target; both inputs are the
command and the attitude, so the robot can produce them — **but the robot host publishes only
the zero-pose FK today** (`feet_y_gravity_zero_pose`, BrainInputs.hpp), so this arm is
sim-only until the host computes the commanded FK. Sim arm `…__lpg_cand_intclr40.json` in the
launcher as "EXP · LPG candidate + integrator + swing clearance (sim)".

**Flat ground, by the physics contact (arena n=6 × 4500).**

| arm | net_disp | falls | true airborne FL/FR/RL/RR | lift-offs/run | median swing (s) |
|---|---|---|---|---|---|
| benchmark | 3.52 | 0 | 0.23 / 0.12 / 0.26 / 0.21 | 129 / 107 / 111 / 64 | 0.12 / 0.07 / 0.14 / 0.20 |
| candidate | 6.02 | 0 | 0.31 / 0.22 / 0.32 / 0.14 | 53 / 74 / 91 / 50 | **0.53** / 0.24 / 0.18 / 0.17 |
| + integrator | 6.38 | 0 | 0.28 / 0.23 / 0.24 / 0.15 | 59 / 73 / 92 / 52 | 0.39 / 0.27 / 0.16 / 0.19 |
| + integrator + clearance 40 | 6.38 | 0 | 0.29 / 0.23 / 0.26 / 0.17 | 58 / 71 / 90 / 52 | 0.46 / 0.27 / 0.19 / 0.22 |

**What it says, and two instrument corrections.** (1) Under the lever every leg is airborne
MORE than under the benchmark (0.14–0.32 vs 0.12–0.26), with fewer and longer swings (the
benchmark's 64–129 lift-offs of 0.07–0.20 s are its flicker). (2) The earlier reading that
the candidate "barely steps on flat ground" (steps 18 → 5–9) was `arenaavg`'s `steps`, which
counts the feet_y proxy past a threshold; by true contact the stepping is intact. Both `steps`
and the per-leg `air`/`rise` from feet_y are now known proxies: read `con`. (3) The residual
asymmetry is the right rear: airborne 0.14–0.17 against 0.23–0.32 elsewhere, the shortest
swings, and the lowest femur in every arm; the integrator and the clearance each move it a
point or two. Its cause is open; the body/sign audit above cleared the geometry.

**Next.** The robot host's commanded FK (so the clearance can run on hardware and the
corner sensor is the same object in both); the right rear's short swing as its own question
(its load share, its phase dwell, the diagonal 60/40 split at the hold); long runs; the floor
run with `thrash.py` and the FSR flicker as the honest step count on the robot.

### 2026-10-13 — THE COMMANDED FK ON THE ROBOT HOST: `ogma_host --cmd-fk` publishes `feet_y_gravity_cmd_imu` from the commanded pose, the sim's CMD_FK_SOURCE=1, so the corner sensor is one object on both sides (built, unit-tested, NOT yet run on the Pi)

**What changed.** `pi_host/include/ogma/hw/BrainInputs.hpp`: `feet_y_gravity_cmd_fk(body, cmd, up)`
= the shared `fk_leg` on the commanded hinge angles (from the pulse on the line, which benchd has
already slewed — the sim's `_eff_target`), `rest_inv` of the lower segment's origin, then the
shared `feet_y_gravity` with its half-shank offset along the fused up: line for line the sim's
`feet_y_gravity_cmd_imu` branch at `cmd_fk_source == 1`. `BrainInputBuilder::set_cmd_fk`; off =
the zero-pose form, byte-identical to P-e as validated. `ogma_host --cmd-fk`; `dash_run.py` maps
body_env `OGMA_PICRAWLER_CMD_FK_SOURCE=1` to it, as it maps the S1 senses. Test
(`test_brain_inputs.cpp`): at all-zero angles the commanded form equals the zero-pose form to
1e-6 on every leg; a femur lifted as the swing lift commands it (hip2 −0.3) raises that foot's
reading by more than 5 mm and no other's; pitched nose-down the front still reads above the rear.
The whole host suite passes on the x86 build; the Pi build and the floor run are the operator's.

**Why it matters.** Until now the corner sensor existed only in the sim; every lever on it was
sim-only. With this the integrator's swing-clearance arm can run on hardware from the same config
(`…__s3a_stuckinst__hip2p030__lpg_sl4s_st07_intclr.json`, in the launcher and the dash).

**The operator's concern, recorded as the next question for the robot (2026-10-13): "I am
concerned about the robot's lack of balance and tendency to fall on obstacles. The previous gait
was noisy and inefficient but it handled uneven terrain well."** The sim says the lever's
terrain transport is higher than the benchmark's (corridor 6.30 vs 4.23 m, 0 falls both) — but
the sim's foot is the open sim2real gap (2026-10-12), the robot's falls on obstacles have not
been counted under either gait, and the benchmark's terrain tolerance is the bar. The honest
test is the same obstacle course under both configs on the robot, falls and FSR flicker counted,
before any promotion. ⚠ The reflex stack's protections the old gait leaned on (the stance press,
the swing tuck, the rear landing sequence) are all still live under the lever; what changed is
the timing substrate and the gate on the push.

### 2026-10-13 — THE OPERATOR'S TERRAIN OBSERVATION (sim, corridor), recorded as the design brief for the next lever: reach to contact before the push, push only what the chassis needs, and a learned swing length

*Operator: "when the robot encounters an object in the corridor such as a speed hump or pyramid,
the first leg to encounter a surface higher than the previous flat surface will press down the
same amount as the previous steps thus raising that corner (usually a front corner) so high, the
opposite leg cannot touch the ground. the robot attempts to stride forward but the hip1 movements
can pitch the robot over. if the robot learns to keep its chassis level even on rough terrain,
then each leg can reach down before the stroke, make contact with the ground, then push and swing
only the amount needed to move the robot forward and keep the chassis level as possible. also,
shorter swings could help with the instability (also should be a learned param)."*

**Read against the mechanism as built.** The load-held phase ends a swing by TIME (touchdown
at phase π) and the stance press is a constant: a foot that lands early on a hump is pressed by
the same amount from a higher floor, and a foot over a dip is pushed in the air. Nothing seeks
the ground. The planted-leg integrator corrects the corner afterwards, slowly, through the
attitude; the operator's brief asks for the correction BEFORE the push, per leg, from contact:

1. **Reach to contact.** The swing ends when the FOOT SENSOR reports contact, not when the
   phase says so: the leg descends (femur down, shank out) through the end of swing until the
   FSR loads, and that leg length becomes its stance length. The load-held phase already has
   the sensor; what changes is that touchdown becomes an EVENT the phase waits for (hold at
   phase π until contact, or advance the descent until contact) rather than a time.
2. **Push only what the chassis needs.** The stance press on a leg whose corner is already
   high (its commanded-FK depth below the planted plane, or its corner error negative) is
   withheld, and the stroke on a leg whose corner is low is shortened — the corner error
   gating the per-leg press and push, the way the level error already gates the whole stroke.
3. **A learned swing length.** The hip stroke amplitude per leg adapted from the body's own
   outcome (tilt during that leg's stroke, or the corner error it produced), bounded, not a
   constant: shorter where the body rocks, longer where it does not.

Each is one lever with a gain-0 guard and its own control; (1) first, because it supplies the
state (2) and (3) gate on. The sim's corridor humps are the test; the robot's obstacle course is
the bar (the operator's concern, previous entry). Deferred behind the robot run of the current
stack, by the operator's call ("it may be a good time to test the current config on the real
robot").

### ★★★ 2026-10-13 — FIRST ROBOT RUNS OF THE LEVER: one pair works, the body circles, few forward steps — read from the feed: the FSR shares were RL 0.5 / RR 0.05 under one curve (the rule held one leg and freed another), the avoid loop steered 95 % of the run at ~1.0 m, and the raw hip command reversed 30× a stride against the sim's 13

*Operator, four floor runs (`…__lpg_sl4s_st07` ×2, `…_int`, `…_intclr`, 134–295 s each): "a
tendency for one side or one pair of the legs to do more work than the other pair … front left
and front right, and the robot would turn in circles … in other cases front right and rear left.
I did not see the robot taking steps in a forward direction often. The range to the closest
obstacle was ~150 cm so there should be no reason for the robot to turn away."*

**1. The load shares the rule saw** (feed FSR counts → the one shared curve → shares, sim leg
order; the host did not log the module's own shares — instrument gap, closed below):

| run | FSR mean counts FL/FR/RL/RR | contact frac (≥200) | **load share** FL/FR/RL/RR |
|---|---|---|---|
| gate + 0.7 (295 s) | 1569 / 1324 / 2205 / 835 | 0.79 / 0.71 / 0.78 / 0.64 | 0.27 / 0.12 / **0.51** / **0.07** |
| gate + 0.7 (148 s) | 1050 / 818 / 1363 / 565 | 0.51 / 0.47 / 0.52 / 0.42 | 0.20 / 0.08 / **0.31** / **0.05** |
| + integrator | 1826 / 1558 / 2408 / 847 | 0.92 / 0.83 / 0.88 / 0.70 | 0.23 / 0.15 / **0.55** / **0.04** |
| + integrator + clearance | 1867 / 1881 / 2429 / 526 | 0.94 / 0.91 / 0.89 / 0.49 | 0.23 / 0.20 / **0.52** / **0.03** |

The rear-left FSR reads ~3× the rear-right's at the same stance (2200–2400 vs 530–850 counts)
and one counts-to-grams curve serves all four, so the rule read rear-left as carrying half the
body and rear-right as carrying nothing: a share above ½ is HELD at its fixed point (never
lifts), a share near 0 runs free. One leg pinned, its diagonal partner flapping — the operator's
"one pair does the work". In the sim the physics shares read 0.25 each, which is why it never
showed. The rear-right also fails the 200-count contact threshold half the time at 526 counts.
**Lever: per-foot self-normalisation** (`load_norm_tau` 500 on LoadPhaseGait: each foot's load
÷ its own running reference while touching), the §5.5 form — adapt to the sensor rather than
calibrate it. Unit-tested: a 3× stiffer sensor reads the same share once warm, and a real
doubling still reads.

**2. The avoid loop was steering the whole run.** `avoid.value` mean 0.47–0.62, active 93–97 %
of diag rows, |intent err| 0.63–0.82, speed cut to 0.63–0.77, with the ultrasonic's last range
0.97–1.11 m — inside ProxMemory's `range_m` 1.0, so a wall at a metre populated the proximity
slots continuously and the heading hold turned the body all run. Not the gait. **Test condition:
`range_m` 0.3** (the operator: "the robot is able to turn in place well so the reflex can kick
in at 30 cm"), on the lever arms AND the benchmark, so forward walking is measured clean.

**3. The raw hip command reverses ~30× a stride on the robot (all four legs, 26–34) against
13 in the sim under the same config** (the attribution trace's raw actions; the earlier sim
figure of 4.4 was the ACHIEVED joint, smoothed by the servo — not like with like). The sim
lever halves the raw thrash from the old ~29; the robot does not. Robot-side suspect: the level
gate multiplies the stroke by the fused tilt every tick, and that signal is noisy on hardware.
**Lever: `stroke_level_smooth`** 0.1 (the gate on a ~10-tick EMA of |tilt|), one arm.

**Also seen:** stroke sd and hip2 means are symmetric across legs in the command, so the
asymmetry entered through the shares, not the command. Belly 20–50 mm, 0 HAT resets, current
in range (the dash's meta).

**Arms for the next floor run** (launcher + dash): `…__lpgn_r03_sl4s_st07` (normalised shares
+ avoid 0.3 m), `…__lpgn_r03_sl4s_sm_st07` (+ the smoothed gate), `…__lpgn_r03_sl4s_st07_int`,
and the benchmark at the same avoid range `…__r03`. Instrument: the host's per-tick diag now
mirrors LoadPhaseGait (phases, shares, references, lock) and MotorEPMv2's leg-phase / gate /
integrator fields, so the next run's shares are read from the module, not reconstructed.

### ★★★ 2026-10-13 (evening) — SECOND ROBOT RUNS: the normalised shares even out (0.20–0.30), the avoid range at 30 cm gives straight walking, and the SMOOTHED GATE is the operator's clear winner — "deliberate foot placement and stepping"; what remains is the servo roughness, and the raw command reverses 30–45× a stride on every joint in every config, the benchmark included

*Operator, four runs in the suggested order: "config 1 (BENCH avoid 30 cm): the robot attempting
to walk straight, no turning behavior away from walls. 2 (norm + gate): better stepping but a
lot of small servo reversals making the movements noisy. 3 (norm + SMOOTHED gate): much improved
and deliberate foot placement and stepping; servos still not smooth like the sim. 4 (+ integrator):
noisier than 3 but ended up walking straight after some flailing. The smoothed gate is a clear
winner if we can smooth out the servo motion."*

**Read from the module (the dash's per-tick diag now carries it):** shares FL/FR/RL/RR 0.24–0.29 /
0.23–0.29 / 0.21–0.28 / 0.19–0.30 in every lever arm (the raw read was 0.5 / 0.05 the run before);
the per-foot references the normalisation found: RL 0.6–1.1, RR 0.04–0.06 — a 15–20× spread in
sensor sensitivity under one curve. The gate sits at 0.55–0.75 with sd 0.04–0.08 (steady), the
tilt |pitch| 0.04–0.12, phase swing fraction 0.39–0.43, lock at touchdown 0.32–0.50, inter-leg
0.26–0.49, the phase running backwards 5–10 % of leg-ticks (load spikes). `WORKING` (operator's
eye + the module's own read) for `load_norm_tau` 500 and `stroke_level_smooth` 0.1; the avoid
range 0.3 m is the test condition and stays for forward measurements.

**The remaining roughness, measured (raw action dump, reversals per 70-tick stride per joint):**

| arm | hip1 FL/FR/RL/RR | hip2 | knee |
|---|---|---|---|
| benchmark (old gait, avoid 30 cm) | 25 / 28 / 31 / 28 | 42 / 46 / 39 / 46 | 33 / 35 / 36 / 34 |
| norm + gate | 31 / 35 / 34 / 31 | 40 / 39 / 38 / 41 | 38 / 45 / 43 / 41 |
| norm + smoothed gate (the winner, two runs) | 36–43 | 37–47 | 37–44 |
| + integrator | 35 / 40 / 40 / 35 | 42 / 43 / 42 / 45 | 38 / 44 / 46 / 43 |
| **the sim's candidate, same read** | **13 / 13 / 13 / 14** | **6 / 9 / 14 / 9** | **9 / 12 / 20 / 18** |

Three times the sim, on every joint, in every config including the benchmark: the source is
robot-wide, in the inputs the robot shares, not in any lever. Two robot-only inputs reach the
command every tick: the FOOT-CONTACT FLAG (the swing tuck and the rear landing gate on it raw,
and it transitions 70–216 times a minute per foot at the 200-count threshold — roughly half the
gait's own touchdowns and lift-offs, half flicker; hysteresis 300/120 with a 3-frame debounce
halves it) and the GYRO in the heading hold's damping term. The sim's contact flag is clean and
its gyro is noiseless.

**Arms for the next run** (launcher + dash, on the winner): `…__lpgn_r03_sm_pg` — the tuck and
the rear landing gated on the phase's swing state (`swing_gate_src` 1), so no flickering flag
reaches the command; `…_pg_e0` — plus the explore noise off (the ledger's cheap reversal lever);
`…_pg_d0` — a TEST lesion of the heading hold's yaw-rate damping, to read the gyro's share of
the hip1 reversals. The honest sensor-side form (contact hysteresis at the host) follows if the
phase gate alone does not close the gap.

### ★★★ 2026-10-15 — THIRD ROBOT RUNS: THE THRASH WAS THE EXPLORATION NOISE — with it off three legs drop to the sim's 3–9 reversals a stride and the HAT resets stop; the contact flag and the gyro were not it; the front-right leg keeps reversing (12–37) with the noise off; the heading-damping lesion was the fastest, best-turning run; and "no navigation away from walls" is the 30 cm memory meeting glancing walls

*Operator, five runs: (1) phase-gated tuck: "stepping is slow and I am still seeing jitter; 82 s to
the end of the carpet; 2 HAT resets". (2) + noise off, twice: "looks more smooth; a few instances
where the supporting legs would drop the chassis a bit but the belly did not impact the floor;
navigation away from walls was not as good; 72 s". (3) + heading D off (the lesion), twice: "62 s;
more jittery than 2 but more confident steps; turned away from the wall at 30 cm; the gait became
more confident toward the end; the tipover was the robot hung up on a curtain, unable to back up,
the thrash caused it".*

**Raw command reversals per 70-tick stride (the brain's targets, before the slew):**

| arm | hip1 FL/FR/RL/RR | hip2 | knee | HAT resets | shares |
|---|---|---|---|---|---|
| winner (13th, smoothed gate) | 36 / 40 / 41 / 39 | 37 / 40 / 39 / 43 | 37 / 41 / 43 / 39 | 0 | 0.27 / 0.28 / 0.24 / 0.20 |
| + phase-gated tuck (`swing_gate_src` 1) | 39 / 46 / 44 / 44 | 43 / 39 / 42 / 45 | 40 / 43 / 46 / 43 | 2 | 0.30 / 0.25 / 0.24 / 0.21 |
| **+ noise off (`explore_noise` 0), run 1** | **7 / 29 / 8 / 7** | **4 / 12 / 8 / 10** | **9 / 37 / 9 / 9** | 0 | 0.28 / 0.24 / 0.25 / 0.22 |
| + noise off, run 2 | 6 / 12 / 4 / 5 | 4 / 9 / 4 / 7 | 5 / 19 / 4 / 5 | 0 | 0.32 / 0.24 / 0.23 / 0.22 |
| + heading D off (lesion), run 1 | 38 / 45 / 44 / 40 | 43 / 38 / 40 / 43 | 36 / 42 / 44 / 41 | 0 | 0.28 / 0.25 / 0.25 / 0.22 |
| + heading D off, run 2 (431 s) | 36 / 43 / 41 / 40 | 42 / 42 / 40 / 44 | 34 / 38 / 39 / 36 | 0 | 0.23 / 0.21 / 0.26 / 0.21 |
| the sim's candidate, same read | 13 / 13 / 13 / 14 | 6 / 9 / 14 / 9 | 9 / 12 / 20 / 18 | | 0.25 each |

**Verdicts.** `swing_gate_src` 1: `NULL` on the reversals (the flag was not the source; kept, it is
the cleaner gate). The gyro: `NULL` on the reversals. **`explore_noise` 0: `WORKING` (loud):** three
legs go to the sim's level or below, the HAT resets stop (the thrash was the current), the
operator sees it as smoother. The white noise at σ 0.05, redrawn every tick with a step near the
slew cap, was three quarters of the robot's command reversals; on the robot it costs more than in
the sim because the servo passes it through. Its role for the gait on the robot is not
exploration; it is noise.

**The front-right leg.** With the noise off, FR still reverses 12–37 a stride on hip1 and knee
while the other three sit at 3–9. Not the shares (0.24), not the stroke sign (RR shares it and
is quiet). The per-leg learned controller is the suspect: the homeokinetic rule drives each
leg's loop gain toward the edge, and without the noise to mask it FR's loop may be self-
exciting at the tick rate. Lesion arm `…_e0_nl` (noise off + the learned controller off); the
dash's diag now carries the per-leg loop state (`hk_share`, `clip_*`, `pre_*`) to read it.

**The heading-damping lesion.** The D term (`heading_hold_gain` 0.3 on the gyro's yaw rate)
changed nothing in the command and its removal gave the fastest run (62 s vs 72 and 82), the
turn at 30 cm, and "more confident steps". It was a promoted term on the old gait (the variance
collapse of 2026-08-04); on this gait, with the stroke and the steer gated and the body slower,
the damping reads as stiffness. Combined arm `…_e0d0` (noise off + D off).

**"Not as good at navigating away from walls" with the noise off — the ultrasonic, not the
gait.** On the two noise-off runs the beam saw a surface inside 0.35 m on 25 and 4 readings of
~1 100 (brief dips to 0.19–0.30 m) and the avoid loop fired on 2 of them; on the long D-off run,
366 readings and 329 firings. A body that walks straight meets a wall at a glancing angle, sees
it late, and the 0.3 m memory leaves no lead; the noisy gait wandered and met walls head-on.
Arm `…_r05_…_e0d0`: the memory at 0.5 m, under the ~1.0 m floor return that steered the first
runs. The 30 cm setting remains the clean forward-measurement condition.

**Also:** the tipover was a curtain the robot could not back out of; the stuck escape was
deferred on 2026-10-12 (the stride estimate is noise on carpet) and stays deferred. The speed
to the carpet's end (82 / 72 / 62 s) is the operator's own stride instrument until the odometry is
read back from the host (it is published; the dash does not yet log it).

### ★★ 2026-10-15 (afternoon) — FOURTH ROBOT RUNS: the noise-off + damping-off arm walks "slow and purposeful"; the 50 cm memory turns the body before it scrapes; FR's residual thrash IS its homeokinetic loop (3 a stride with the controller off); the gate sits at 0.55 on the robot's STANDING lean, which halves every stroke — and the crab-walk was the floor, not the sunbeam

*Operator: (1) noise off + D off: "68 s to the edge of the carpet; gait begins slow and
purposeful and low noise; turn away from the wall was successful". (2) + avoid 50 cm: "59 s; the
longer reflex range helped the robot turn away from the wall earlier and did not result in
scraping legs against the wall; in both runs the legs moved faster and more 'confident' when
turning or when other reflexes are engaged — why? In the last 30 s the robot began to crab-walk
sideways; it had entered a sunbeam and the vision EPM grew a lot of nodes." (3) noise off + the
learned controller off: "70 s; overall performance is lackluster".*

| arm | raw reversals/stride hip1 FL/FR/RL/RR | knee | gate mean | HK share of the command | time to the carpet's edge |
|---|---|---|---|---|---|
| noise off + D off | 8 / 9 / 9 / 6 | 12 / 21 / 14 / 12 | 0.55 | 0.31 | 68 s |
| + avoid 0.5 m | 7 / **16** / 9 / 7 | 9 / **21** / 11 / 9 | 0.55 | 0.29 | 59 s |
| + learned controller off (lesion) | 3 / 3 / 3 / 3 | 6 / 6 / 8 / 8 | 0.52 | 0.02 | 70 s |

**Four readings.**
1. **FR's residual is the learned loop.** With the controller off every joint of every leg sits
   at 3–8 reversals a stride; with it on, FR's hip and knee run at 16–21 while the other three
   sit at 7–12. The homeokinetic share of the command is 0.3 on this gait (0.11 on the old one):
   the layer is louder here, and FR's loop is at its edge. Removing the layer costs the gait its
   participation ("lackluster", 70 s). Lever: lower the loop gain, not the layer —
   `motor_gain` 3 → 2 (`…_mg2`, a TEST arm).
2. **The gate sits at 0.55 while walking straight.** |pitch| 0.07–0.09 and |roll| 0.08 are the
   robot's STANDING lean (the same nose-up stance the sim has), and the gate reads absolute
   gravity, so the stroke is permanently withheld to about half: the slow gait. Turning adds the
   steer differential to the amplitude, which is why the legs look "confident" whenever a reflex
   engages — the body is briefly allowed a full stroke. Lever: the gate on the deviation from a
   ~6 s running attitude (`stroke_level_ref_tau` 300, `…_ref300`), so only transients withhold
   the push. The sim read it as more steps and more wobble; the robot is the test.
3. **The crab-walk was the floor.** Odometry (the host logs it every 50 ticks): over the run the
   body made +0.033 m/s forward with 0.001 lateral; in the last 30 s, −0.031 forward (backward)
   and −0.041 lateral, with the strokes unchanged and the raw load shares shifting FL 0.16 →
   0.11, FR 0.23 → 0.30. The vision EPM has no consumer and the host's CPU stayed under 5 %
   (p50 2.7 → 4.6 %, tick budget 20 ms untouched). A sunbeam on the floor is the end of the
   carpet: on a slick surface the loaded feet slide, so the net push comes from whichever feet
   grip — the sim2real foot problem (2026-10-12), now seen on the robot as a lateral drift. The
   grippy sole is the hardware lever; the stance press on a sliding foot is the brain's.
4. **The 50 cm memory** turns the body before contact and costs nothing to the forward
   measurement; it stays. The floor return at ~1 m that caused the first circling is still out.

**Instrument note.** The dash already passes `--dump-inputs 50`, so odometry, body-frame
velocity, stride estimate and raw loads are in every host log at 1 Hz — the speed and the net
displacement no longer need a stopwatch. The raw FSR shares in those rows (RL 0.56–0.58, RR
0.03) show the per-foot spread the normalisation absorbs; RR's reference of 0.05 is
noise-dominated and that foot's share is the least trustworthy of the four.

**Arms on the dash:** `…_r05_e0d0_ref300` (the gate on the running attitude) and
`…_r05_e0d0_mg2` (loop gain 3 → 2, TEST), both on the 50 cm noise-off D-off base.

### ★★★ 2026-10-18 — THE RUNNING-ATTITUDE GATE ON THE ROBOT: +40 % forward speed and FR's thrash halved; the run ended with two HAT brownouts at 3 A peaks and the Pi dropping out on a stall — the stall protection the gait lacked, and a battery lower than on the 13th

*Operator: "I did a run on the new running attitude config but the robot got stuck. After that
brownout I cannot reach the robot via ssh." The Pi had rebooted; its journal is volatile, so
the previous boot left no record. Everything below is from the run's own feed, the brain's
diag and action dump, and the INA capture, all of which the host kept writing to the end.*

**The gait, before it died** (`…_r05_e0d0_ref300`, `stroke_level_ref_tau` 300, 184 s):

| | gate mean | stroke sd (hip) | forward speed (odom) | net over run | raw reversals/stride FR hip1 / hip2 / knee | servo current mean / p95 |
|---|---|---|---|---|---|---|
| previous winner (noise off, D off, 50 cm) | 0.55 | 0.17 | 0.026 m/s | 3.16 m / 253 s | 16 / 10 / 21 | 1.34 / 2.54 A |
| **the gate on the running attitude** | 0.64 | 0.22 | **0.037 m/s** | 2.81 m / 184 s | **9 / 6 / 9** | 1.10 / 2.39 A |

`WORKING` (signal, one run): a larger stroke, 40 % more forward speed at LESS mean current, and
the front-right's residual thrash halved, so part of what read as FR's learned loop was the
absolute gate multiplying the standing lean tick by tick on that leg. The gate still sits at
0.64, not 1: the reference adapts over ~6 s and the body rocks within that.

**What killed it.** Three events, two kinds.
1. **Two HAT brownouts, 22 s and 84 s:** 62 and 108 rail readings under 3.1 V in ten seconds,
   the rail to 2.00 and 1.98 V, servo current 2.7–3.0 A at the dips, the HAT's MCU reset and
   disarmed (`armed` 0), then re-armed (the 2026-10-03 fix held: the servos did not run to
   their stops). The 2026-10-05 finding again: the 3.3 V rail browns out under servo current;
   peaks above ~2.4 A are the known reset line, and this run spent 2–8 % of its samples there.
2. **The Pi dropped out at 190 s** with the rail at 3.06–3.3 V and the fast current trace
   showing 1.3 A and a 7.4 V bus in the last tenth of a second — no rail or current signature
   at all. Two seconds earlier the stuck sense had fired (stall 1.0, forward stride −0.30 m/s,
   intent error −1.06, speed cut to 0.49): the body was pushing on something it could not
   back out of. The Pi sits on its own BEC upstream of the shunt, so a dropout there is
   invisible to both instruments; a battery lower than on the 13th (bus 6.6–7.7 V, mean 7.4;
   the HAT's battery sense ~3 045 counts against ~3 150) leaves the BEC less margin under
   the stall's load. The journal cannot confirm an under-voltage reboot because it is not
   persistent.

**Levers, in order.**
- **A stall gate on the push** (`stuck_gate_gain`, MotorEPMv2, reading StuckSense's score):
  the stroke scaled by 1 − gain·stuck, so a body the stuck sense calls stalled stops driving
  into what it is stalled on. The escape by backing was deferred (the stride estimate is noise
  on carpet); this is the half that costs nothing when wrong — a stalled body that stops
  pushing is merely still. "Fatal events have no gradient": the reflex keeps the learner alive.
- **The persistent journal** on the Pi (`/var/log/journal`), so the next reboot carries its
  cause; a one-line system change, the operator's call.
- **Battery state before a run**: the bus mean and the HAT's battery sense are in every feed;
  read them before trusting a brownout to the gait.

### ★★ 2026-10-18 (later) — THE STALL GATE ON THE STUCK SENSE: `REGRESSION`, for the reason the ledger already held — the stall channel is stride noise on carpet; replaced by a CURRENT gate on the push

*Operator: "the gate may be too sensitive. The robot only took a few steps and spent most of the
time struggling."* Run 104 s: the stuck score above 0.8 for 44 % of the run, all of it the STALL
channel (the range-deficit channel 0 %, never measured: it speaks only when the stride claims
walking); the stride estimate sat at ±0.02 m/s around zero with its reference at 0.02–0.17; the
push was withheld, the body made 0.36 m in 99 s (0.011 m/s). The loop the 2026-10-12 deferral
predicted: withhold the push, the stride reads lower, the score rises, withhold more. My miss —
the deferral was on the books. Verdict `REGRESSION`; the arm stays in the launcher as a TEST
for the record; `stuck_gate_gain` 0 everywhere.

**The right sensor is the one that kills the robot.** The HAT's rail browns out and its MCU
resets above ~2.4 A (2026-10-05, 2026-10-18), a stalled servo draws ~1 A each (2026-10-05 scale
probe), and the servo-branch current is on the bus on both sides (`sense.servo_current`: the
INA on the robot, the power model in the sim). **Lever: `current_gate_lo/hi`** — the
propulsion term fades linearly from full at 1.8 A to zero at 2.4 A of a 5-tick EMA of the
current; steer untouched. The effort variable, read where the regulation hides it (the memory
of 2026-10-05). Arm `…_r05_e0d0_ref300_cg`. What it should show: the current's time above
2.4 A (2–8 % of samples on the 18th) going to ~0 with the stride intact on open floor; on a
stall, the push fading while the steer turns the body away. What would refute it: a walking
gait whose normal peaks (3 A on the 18th) are themselves above the line — then the gate
withholds the push mid-stride and the test is the stride, not the stall.

### ★★ 2026-10-18 (evening) — THE CURRENT GATE ON THE PUSH: `REGRESSION` — the servo current is the stride's own rhythm, not a stall; and a stall on this body does not show as current at all

*Operator: run completed on the current-gate config; "let's see the data."* 292 s, 0 HAT resets.

| | time > 2.4 A | time > 1.8 A | rail min / dips < 3.1 V | forward (odom) | knee reversals / stride |
|---|---|---|---|---|---|
| running attitude (the brownout run) | 4.9 % | 14.3 % | 1.98 V / 188 | **0.037 m/s** | 5–8 |
| + current gate 1.8 → 2.4 A | 4.2 % | 14.5 % | 2.79 V / 57 | 0.010 m/s | **16–20** |

The gate saw its input above 1.8 A on 12 % of ticks and modulated the push at the current's
own rhythm: the servo current swings 0.3–3 A inside every stride as the legs drive, so a 5-tick
average of it is the gait, not a stall. The stride fell to a third, the knee thrash tripled, and
the rail's time above the reset line barely moved, because those dips ARE the stride's peaks.
This is the refutation case named when the arm was built. **And a stall does not show as
current here:** in the two seconds before the Pi dropped out on the 18th the fast trace read
1.3–2.3 A, because slew-limited servos that have reached their targets against an obstacle
draw little. Verdict `REGRESSION`; the arm stays as a TEST for the record; `current_gate_hi` 0.

**What the record now says about the stall.** Three signals tried, three verdicts: the stride
estimate (noise on carpet — the stall gate withheld a walking body), the current (the stride's
own rhythm, and silent at a real stall), and the range deficit (the honest one — "I command
forward and the wall does not get closer" — but it only speaks when the STRIDE claims walking,
which at the fatal stall it did not). **Lever: the range deficit expected from the COMMANDED
speed** (`expect_from_cmd` on StuckSense: expected = speed command × a nominal forward speed ×
the window), so the sense speaks whenever the brain commands forward with a surface ahead,
and the push gate reads THAT channel (`stuck_gate_src` 1) through a slow EMA. It will not
fire with nothing ahead within the beam's 1.4 m; the curtain had a valid echo at 0.5–0.8 m.

**The rail dips themselves are hardware** (the 2026-10-05 finding): 3 A stride peaks against a
3.3 V regulator with no reserve. A bulk capacitor on the HAT's rail, or a separate servo
supply, is the lever for those; no gate on the command should be asked to trim a healthy
stride's peaks.

### 2026-10-18 (night) — THE SERVO CURRENT'S SHAPE DOES NOT SEPARATE A STALL FROM WALKING EITHER: the stalled body kept cycling; refuted on the logs before a gate was built on it

*Operator: "Does this new method account for normal slippage? Both no progress and high sustained
current?" — the hypothesis offered in reply: a stalled body's legs are pinned so its current goes
flat, a walking or slipping body's current pulses at the stride; so deficit × (1 − pulsation)
would separate stall from slip. Checked on the recorded feeds (50 Hz servo-branch current,
pulsation = sd / mean over the segment, the strongest autocorrelation lag as the rhythm):*

| segment | mean A | pulsation | strongest period (autocorr) |
|---|---|---|---|
| walking, running-attitude run, 100–180 s | 1.22 | 0.49 | 1.28 s (+0.09) |
| **the curtain stall, last 3 s before the Pi died** | 1.22 | **0.47** | 1.28 s (+0.14) |
| the same, last 8 s | 1.19 | 0.47 | 1.20 s (+0.39) |
| walking, 2026-10-15 run, 60–200 s | 1.47 | 0.43 | 1.12 s (+0.19) |
| the slick-floor crab-walk, last 30 s | 1.07 | **0.66** | 0.60 s (+0.42) |
| the stall-gate run, push withheld, 20–100 s | 1.09 | 0.60 | 1.64 s (+0.22) |

**Refuted.** The stalled body pulsed exactly like the walking one, with the same stride rhythm:
against the curtain the gait kept cycling (the load rule only slows a loaded leg; the strokes
went on, the feet scrubbed), so nothing was pinned and the current never went flat. Slip did
not read lower either — it read HIGHER (0.66, with a faster rhythm), the opposite of the
expectation. The current's level was refuted the same day; its shape is refuted now. Not
built, by the measure-first rule.

**What stands.** No-progress-toward-a-surface (the range deficit from the commanded speed) is
the one honest stall signal on this body, and it cannot tell a stall from slippage with a
surface ahead; with none ahead it is blind to both. The slip signature that the logs do show is
in the odometry's direction (sideways and backward drift at 0.03–0.04 m/s with the strokes
unchanged, 2026-10-15), which is a slow read and noise on carpet. Slip versus stall stays an
open question; the conservative gate (fade the push when the surface ahead does not approach)
is what runs next, and its cost on a slippery floor is a body that stops pushing.

### 2026-10-18 (night) — THE RANGE-DEFICIT GATE READ THE SONAR'S JITTER, NOT A STALL: a 1 s window cannot resolve the 3 cm a walking body closes — the window goes to 10 s with median ends, and the expectation to the body's measured 0.02 m/s

*The run of "running attitude + range-deficit stall gate" (`…_ref300_sg2`, 189 s, 0 HAT resets,
rail never below 3.0 V, the operator: "the robot did not encounter any stuck situations").
Verdict on the gate as built: NULL on progress at n=1 and a MEASUREMENT DEFECT in its input.*

| run (2026-10-18) | net m/s | path m/s | along-heading (5 s windows) | straight | reversals/s | range_score > 0.5 | rail min |
|---|---|---|---|---|---|---|---|
| running attitude (`ref300`, baseline) | 0.015 | 0.054 | 0.020 | 0.27 | 136 | 6 % (gate not wired) | 1.98 V (the Pi died) |
| + range-deficit gate (`sg2`) | 0.018 | 0.039 | 0.020 | 0.46 | 150 | **15 %** (34 % > 0.2) | 3.04 V |

**What the channel was reading.** While the odometry said the body was progressing along its
heading (> 1.5 cm/s), the channel's deficit was **35 % exactly 1.0 and 38 % exactly 0.0** (mean
0.50): a coin flip. The sonar's valid reading changes by 0.040 m (median) to 0.17 m (90th
percentile) over one second, and a third of its readings are invalid, while the body closes
0.03 m in that second; two single end samples one second apart cannot see 3 cm through that.
The gate's smoothed input therefore sat at 0.3–0.95 whenever a surface was within 1.4 m and
the push was withheld a fifth of the time on average (`stuck_gate_gain` 1 × a mean
`range_score` of 0.19), peaking at 95 % during wall approaches at t = 56, 78, 104 s where the
odometry shows the body still moving at 1–3 cm/s. A second bias on top: the expectation
(`expect_from_cmd` 0.03) is above what this stack's body measurably does along its heading on
carpet (0.020 mean in both runs), so a healthy walk already reads a third of a deficit.

**Why it did not hurt more.** The gate only attenuates the stroke; the gait kept cycling, net
progress matched the baseline exactly (0.020 along-heading in both), and the straightness
doubled — read with care, n=1 and the baseline run ended in the brownout. The run was also the
first of the day with no rail excursion below 3.0 V, which the gate may have bought by
trimming the stroke near walls; that is not a stall protection, it is a random 20 % push
attenuator near walls, and it is recorded as such.

**Lever (re-instrumentation, not a new idea).** StuckSense `end_median_ticks`: the range at
each end of the window is the MEDIAN of the valid samples in the window's first and last N
ticks (an end needs a quarter of them valid). Default 0 = legacy single samples, byte-identical.
Test: under the recorded jitter pattern (±0.12 m, one in three invalid) a 1 s window with single
ends scores > 0.3 on a walking body; a 10 s window with 2 s median ends scores under 0.3 and
under half of that, and still scores > 0.8 on a wall that does not approach. The residual
(~0.2) is the jitter's phase at the two ends on a 0.3 m expectation; a walk reads a fifth of a
deficit, not a coin flip. **Arm `…_ref300_sg3`** ("EXP · ROBOT · running attitude + 10 s range
gate"): `window_ticks` 500, `end_median_ticks` 100, `min_expected_m` 0.10, `expect_from_cmd`
0.02 (the body's measured along-heading speed under this stack — a constant, named as such;
the adaptive form, the body's own recent best closing rate held like the stall channel's
`v_ref`, is the next lever if this one holds). The cost: a stall takes several seconds to
register; this gate protects the rail from a long stall, not a stumble. The dash now logs the
gate's actual input (`stuck_gate_in`), which the sg2 run did not (its `stuck_score` row is
the combined score, not the channel the gate read).

**Re-use context for the 1 s window:** a rangefinder with jitter well under the per-window
closing (a ToF at 1 cm sigma would resolve 3 cm per second), or a body walking ten times
faster.

### 2026-10-18 (late) — THE PUSH GATES ARE SUSPENDED (DEFERRED) UNTIL THE SERVO RAIL HAS ITS HARDWARE FIX; the 10 s range gate worked on the approach and then LATCHED

*The sg3 run (82 s, cut short on a low battery: 6.7 V under load with dips to 6.0 V from the first
stride). On the approach to a surface the gate stayed at 0 while the body walked at 3–5 cm/s and
the range closed at the expected rate (the coin flip of sg2 resolved); when the range stopped
closing at 0.57 m the gate rose over six seconds, as designed, and the stride channel agreed.
Then it latched: push withheld to 2–30 %, so no closing, so the deficit stayed, until the surface
left the beam 20 s later. Its onset was partly geometry (the beam swept the wall at a yaw rate
just under the turning threshold). And the throttling did not throttle: the current during the
latched phase (1.29 A mean, 3.3 A peaks) matched walking exactly — the gait kept lifting and
swinging; only the push amplitude shrank. A gate on the push cannot protect the rail or the
battery on this body.*

**Verdict: DEFERRED, all three push gates** (stride stall REGRESSION, current REGRESSION, range
PARTIAL-then-latch). Operator's call: suspended until the rail has its hardware fix (bulk
capacitor on the HAT's 3.3 V rail or a separate servo supply, the 2026-10-05 finding), and the
Pi's own supply holds on a low pack. Their configs stay on disk and out of the launcher.
**Re-use context:** with the rail fixed, if a stall protection is still wanted, the range gate
needs its release — the expectation taken from the push actually applied (the motor module
publishing its effective stroke), so a withheld push expects no closing, the deficit falls, and
the body probes again: push, test, fade, retry at the window's timescale. Not built.

**Promoted and in the launcher:** the three benchmarks (the original mechanism) and the
LoadPhaseGait stack's best in each world — sim `…__lpg_sl4s_st07_int` ("PROMOTED · LPG stack
(sim)"), robot `…__lpgn_r05_e0d0_ref300` ("PROMOTED · ROBOT · LPG + running-attitude gate").

### 2026-10-18 (evening) — TEGOTAE REPLICATION: the bare load rule TROTS from random starts (signal); Owaki's walk regime is closed on this body; THE SWING-LOAD PUSH GATE is a `REGRESSION`, and it refutes the "the push pins the rear foot" diagnosis in its within-stride form

*Operator: set up experiments to see what it takes to reach Owaki & Ishiguro's results on our quadruped.
Plan and full tables: [`tegotae_replication_plan.md`](../plans-and-designs/tegotae_replication_plan.md).
Harness `scripts_tools/tegotae_sweep.py`. Runs `~/xaq_runs/tegotae_20261018/`. Arena, n = 6 × 4500,
stripped mode = LoadPhaseGait + posture spring + phase stroke and lift only (every other joint writer
0), GainEvolver removed (operator).*

**Measurement defects found first (each would have produced a false verdict):** (1) LoadPhaseGait
read its seed through a double, so every OGMA_SEED started the legs from the SAME phases — every LPG
seed-average in this ledger since 2026-10-12 shares one start condition (fixed, regression test);
(2) `babble_scale` 0 divides by zero in MotorEPMv2's pulse identification and freezes the joints
all run; (3) the GainEvolver re-publishes `coupling_gain` 1.509 at tick 10 000 in every LPG config
(separate task); (4) headless harnesses load the `cad` body, the launcher `measured_fsr`.

**Results (signal, CAD body unless noted).** The stripped rule trots: diagonal touchdowns locked
(R ≈ 1) from 6/6 random starts, 59 % of strides on the trot template at a slew-matched stroke
(0.525), 4.35 m. Free clock and contact-count shares: no gait (R2 and the tonic-receptor claim
reproduced). Owaki's walk (σN/ω ≈ 5) is out of reach: from σ_r ≈ 3 every leg parks at the stance
fixed point, because the sprawled body's foot load PEAKS in late stance (`sigma_crit` = 1/max(N cos φ)
≈ 3.0, measured). The ω sweep at fixed σ keeps a trot at every period 30–100 (R3 not reproduced:
long periods park, short ones slew-clip the lift). `leg_phase_lift_lead` (late-stance unload):
`PARTIAL` (frees σ_r 3 only; breaks the σ_r 2 trot). On `measured_fsr` the same arm's rear feet
barely lift (yellow = code `rr`: 0 touchdowns), and the trot does not reach the feet.

**The yellow leg, ruled out:** servo (joints overshoot the lift), attitude (corner integrator
levels pitch/roll to 0: `NULL`), static COM/feet (centred), COM height (8 mm lower: `NULL`), more
lift (`REGRESSION`). Freed by: COM 15 mm forward (yellow 46 touchdowns, 77 % trot) or a constant
stroke 0.35 (38, 77 %).

**Lever: swing-load push gate** (`MotorEPMv2.stroke_swingload_gain`, push × max(floor, 1 − g·e),
e = max over legs of max(0, sin φ)·N, N = the phase token's absolute load). Gain-0 arm
byte-identical on x, z, hip1, knee per tick, all six seeds. Consumer: gate mean 0.70 / 0.47 / 0.35.

| arm (measured body) | net (m) | straight | touchdowns FL/FR/RL/RR | trot strides | yellow mid-swing load |
|---|---|---|---|---|---|
| base | 2.98 ± 0.43 | 0.40 | 47 / 51 / 12 / 0 | 0 % | 0.070 |
| gate 4 | 3.11 ± 0.04 | 0.38 | 46 / 46 / 54 / **0** | 0 % | 0.069 |
| gate 8 | 0.77 ± 0.04 | 0.11 | 47 / 14 / 47 / 0 | 0 % | 0.047 |
| gate 15 | 1.18 ± 0.16 | 0.18 | 48 / 39 / 42 / 0 | 0 % | **0.032** |
| constant stroke 0.35 (matched to gate 4's mean push) | 2.82 ± 0.01 | — | 48 / 48 / 50 / **38** | **77 %** | 0.046 |
| CAD base / CAD gate 8 | 4.35 / 3.44 | 0.55 / 0.34 | 46/49/51/48 → 46/86/46/46 | 59 % → 13 % | 0.037 / 0.023 |

**Verdict: `REGRESSION`** (measured and CAD bodies, stripped mode, arena, n = 6). At gate 15 yellow's
swing load is below the CAD body's (0.032 vs 0.037) and it still never lifts, and the constant
stroke with gate 4's mean push frees it where the gate does not. So the within-stride form of the
diagnosis — "the push loads the swinging rear foot and pins it" — is refuted. Load is not what
holds the foot down. What frees it changes the foot path or the standing load (stroke amplitude,
COM). The next instrument is a true foot clearance (the lower segment's tip height, god's-eye,
instrument only); the record has only the shank-angle proxy. **Re-use context:** a body whose
swing feet are pinned by load, shown by a clearance instrument rather than inferred from contact.

### 2026-10-18 (evening) — THE LATE-STANCE PRESS PROBE: a leg's pre-liftoff press DOES control its corner (signed per corner it levels them), but a level corner does not free the yellow foot; the true clearance instrument finds a BODY-SIDE ASYMMETRY AT LEG 3 (`rr`), present with the brain off

*Operator proposal: the push force each leg applies should be learned, to reduce the dip of the corner
it is responsible for (terrain, stuck legs). Step 1, a probe of the actuation before building a learner.
Full table: [`tegotae_replication_plan.md`](../plans-and-designs/tegotae_replication_plan.md) §7.8.*

`MotorEPMv2.leg_phase_late_press` (+ `_hip2`, `_legs`): a fixed press over the second half of stance;
gain 0 byte-identical per tick, six seeds. Measured body, Tegotae trot config, n = 6 × 4500.
A uniform press trades stance dip for swing dip (yellow +177/+56 → +133/+214 at 0.4). A press SIGNED
per corner (multipliers from each leg's measured swing dip) levels them (yellow +96 swing / −10 stance
at ×0.2), so the proposed error is controllable by the proposed actuator. **But yellow still never
lifts** (0 touchdowns in every press arm; blue stops too; net 2.98 → 1.22 m). Verdict on the press
as a stuck-leg fix: `REGRESSION`. As levelling actuation: established (signal), and the learned per-leg
press stays well-founded for terrain.

**New instrument, new finding.** `ftip`/`fknee` (true toe and knee-end height of the lower capsule,
god's-eye, logged under `OGMA_PICRAWLER_LOAD_LOG`): yellow's toe sits ~5 mm deeper than the other toes
from spawn, before any gait (−10 vs −6 mm), and with the brain OFF its joints settle at different
angles from the other three (hip2 +0.025 vs ~+0.012, knee −1.590 vs −1.600), on both bodies. Leg 3 is
built as an exact mirror and the body files have no per-leg offsets. **A body-side asymmetry at leg 3**
is the likeliest source of the record's "right rear lowest corner" (code names, since 2026-10-13) and
of yellow dragging; not yet located. Every per-leg verdict on this body carries it until found.

### 2026-10-18 (night) — THE LEG-3 ASYMMETRY IS THE PHYSICS ENGINE: Godot Physics lets contacts sink 10 mm and solves in creation order, so the LAST-BUILT LEG sinks ~6 mm deeper; a 2 mm allowance evens the feet and the yellow leg steps

*Plan §7.9 has the tables.* **Cause.** `SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION` defaults to
10 mm and the sequential solver favours creation order. Legs are built 0–3, so leg 3 (`rr`, yellow)
settles deepest (stance toe −13 mm vs −6 to −8). **Proof by build order** (diagnostic
`OGMA_PICRAWLER_LEG_BUILD_ORDER`): reversed, yellow is normal and steps (0 → 20 touchdowns), and
leg 0, now last, goes deep. With the brain off the last-built leg's joints settle off the
others. **Fix candidate** `OGMA_PICRAWLER_CONTACT_PENETRATION=0.002` (diagnostic env, default
unchanged): feet within 0.4 mm. Measured body: yellow 0 → 39 touchdowns, trot strides 0 → 31 %,
corner dips halved, tilt_sd −30 %, net 2.98 → 1.93 m (the base's distance was front paddling with
rears sliding). CAD: ties on net and straightness (4.35 → 4.45, 0.55 → 0.56), trot 59 → 55 %,
tilt_sd −40 %. **This resolves the 2026-10-13 "right rear lowest" corner**, recorded then as rear-femur
sag physics; the sag is real but the asymmetry was the solver. **Every sim verdict in this ledger
carries the bias.** Making 2 mm the default, and re-baselining the promoted configs, is the
operator's call. Aside: `OGMA_SOLVER_ITERATIONS` does not reach the physics space at runtime (still
64 after setting 256). Recorded, not chased. Verdicts in the 2026-10-18 evening entries on the yellow
leg (swing-load gate, press probe) are measurement outcomes of this artifact, not verdicts on
those mechanisms; their re-use context is "rerun at 2 mm".

**Addendum (same night) — 2 mm IS THE SIM DEFAULT** (operator: "not unlike the travel on the real FSR
toes"; sim-to-real is a rough translation anyway). `picrawler_body.gd` `contact_max_penetration` =
0.002, override `OGMA_PICRAWLER_CONTACT_PENETRATION`. **Every sim result before 2026-10-18 ran at
10 mm**; set `…=0.01` to reproduce one. Deployed stacks tie (arena, n = 6 × 4500, 2 vs 10 mm):
promoted LPG stack (sim) 5.73 vs 5.80 m, benchmark (hip2 +0.3) 2.80 vs 2.90 m, 0 falls in all four.

### 2026-10-18 (night) — WHY BOTH REAR FEET DRAG: the load-held phase lifts every leg while it still carries ~a quarter of the body, and the forward push keeps the rear pair loaded; the missing mechanism is a WEIGHT SHIFT before liftoff

*Operator: both back legs drag in the corridor, the fronts step. Plan §7.11 has the tables.* At 2 mm,
measured body: the rears drag in the ARENA too (in contact ~90 % of mid-swing, toes never above
−4 mm; the touchdown counts were flickers). Ruled out: pitch (levelled: `NULL`), more lift
(`REGRESSION`), friction (lower = worse; note: a per-foot material cannot lower foot–floor friction
in Godot, because two `rough` materials combine to the HIGHER friction, so the diagnostic is
`OGMA_PICRAWLER_GROUND_FRICTION`), imposed trot or walk clocks (rears pinned in both), build order
(only matters when marching in place). **Cause:** at the phase's liftoff every foot carries 0.20–0.23
BW (σ_r·N ≈ 0.4 < 1, so the load never stops the phase). Ten ticks later the fronts are down to
0.02–0.03 and the rears still carry 0.08–0.11. The COM sweep moves the drag continuously from
rear to front (+30 mm: rear mid-swing contact 0.94/0.89 → 0.61/0.67, trot strides 33 → 90 %), so
the push keeps the rear pair loaded. A stronger hold would wait for the unload but deadlocks,
because nothing unloads a waiting leg on this body. **Next mechanism:** a weight shift. The error
is "my phase says swing and my foot carries weight", the sensor is the FSR, and the actuators are
the other legs. This is the operator's learned per-leg press, extended to the neighbour's unload.
Correction: the measured-body "trot" (plan §7.5, §7.9) is a touchdown-timing pattern whose rear
feet do not clear.

### 2026-10-18 (night) — THE SIM FORCE SENSORS ARE CORRECT; THE ROBOT'S STANCE PRESS IS THE WEIGHT SHIFT; THE LEARNED WEIGHT-SHIFT PRESS (`wshift_rate`) IS A `REGRESSION` — it winds up to its bound on every leg

*Plan §7.12.* Operator: the robot trots with the diagonal legs pushing together and no dragging rear
feet. **Sensor check (sim, measured body, 2 mm):** feet + belly = 0.999 BW; LPG's N = published
`foot_load` on the same tick; 0.0007 BW when the contact flag is 0; no lag at liftoff. **The robot's
config in sim** (`…lpgn_r05_e0d0_ref300`, corridor): rear toes clear 6–19 mm. With its stance press
(0.5) removed, every foot drags (contact ~0.9, net 2.47 → 0.65 m). The stripped Tegotae entries
dragged because they removed exactly this press. **Lever:** each standing leg's press learned from the
swing-weighted load on the other feet (gain 0 byte-identical). On the robot config the amplitude
saturates at its bound on every leg (rate 0.02/0.05, bound 1: net 3.23 → 0.34–0.39 m, falls 0.5 →
1.2–2.7, tilt_sd ×7–10). Bounded at 0.1/0.2 it is a constant extra press (net 2.31/2.12, rear contact
down), the same trade as a fixed 0.7 press (1.77 m). Verdict `REGRESSION`: the integrator assumes more
press clears the swing foot; past ~0.3 the body rocks and the residual swing load rises. A uniform press
also cannot move weight between legs. **Re-use context:** a differential press (per leg and per
neighbour) with an estimated sensitivity or a deadbanded target, judged against the fixed 0.5 press.

### 2026-10-18 (night) — THE DRAGGING REAR FEET ARE AN UNCONVERGED SOLVER: 64 iterations let the loaded leg joints give way; `OGMA_SOLVER_ITERATIONS` was DEAD (physics configured before the env is read); at 256 the rear feet lift on every config — and the promoted stacks lose ~20 % transport (operator's call)

*Plan §7.13.* After the stance press, which rear foot dragged depended on the build order. The
iterations knob had never worked: `_ready()` configures physics before `_resolve_env()` runs. Same bug
for `OGMA_PHYSICS_HZ` (normalisations change, engine rate does not): recorded. Iterations now applied
to the space after the env. **At 256** (corridor, measured, 2 mm, n = 6): robot config rear mid-swing
contact 0.74/0.87 → 0.13/0.17, rear toe peaks 21/20 → 55/47 mm, net 3.23 → 3.70, falls 0.5 → 0.2;
order-independent. Stripped rule without the press: 0.99 → 3.18 m, rears lift. **2×2:** 256 iterations
at the 64-iteration motor authority still lift the rears; 64 iterations at 4× authority are identical
to base, so it is convergence, not servo strength. This is likely the 2026-10-13 "rear femurs sag
under load". **But** arena, promoted LPG stack (sim) 5.73 → 4.52 m, straight 0.54 → 0.38, and benchmark
2.80 → 2.31 m (0 falls, rear feet clear, twice the touchdowns): the stacks were tuned on the sagging
body. **Default kept at 64** (verified byte-identical per tick); 256 is the operator's call.
Measurement outcome for this session's 64-iteration verdicts on the rear legs (swing-load gate, press
probe, learned weight-shift press): rerun at 256.

**Addendum 2026-10-19 — 256 SOLVER ITERATIONS IS THE SIM DEFAULT** (operator: "that slower speed sounds
like what I see on the real robot"). `picrawler_body.gd` `solver_iterations` = 256, applied to the space
after the env is read; `OGMA_SOLVER_ITERATIONS=64` reproduces earlier runs. With the 2 mm contact
default (2026-10-18), **every sim result before 2026-10-18 ran at 10 mm / 64 iterations**. Note the
hinge motor's per-iteration impulse cap: configs without `HONEST_TORQUE_CAP` now have 4× the legacy
servo authority (38.4 N·m); the 2×2 of 2026-10-18 showed authority is not binding. Re-baseline of every
launcher config (paired 256 vs 64, arena / corridor / room, n = 6) is in
`~/xaq_runs/rebase_20261019/`; tables in the plan §7.14 and the next entry.

### 2026-10-19 — RE-BASELINE AT 2 mm / 256 ITERATIONS: every config steps more and stands higher; the promoted stacks tie in the corridor, travel 10–30 % less in the arena, and FALL MORE IN THE ROOM (promoted robot 0.33 → 2.33 per 12 000 ticks); the bare Tegotae rule on the measured body goes 1.73 → 6.24 m

*Plan §7.14 has the three tables (arena 4500, corridor 6000, room 12 000; n = 6; each config paired
256 vs 64 at 2 mm; standard harnesses; `~/xaq_runs/rebase_20261019/`).* Highlights, 256 / 64: promoted
LPG stack (sim) arena 4.99 / 6.47 m, corridor 4.08 / 4.03, room falls 1.00 / 0.33; promoted ROBOT
arena 5.14 / 5.63, corridor 4.08 / 4.02 (std 0.11 / 0.56), room coverage 44 / 50, falls 2.33 / 0.33;
benchmarks arena −15 to −42 %, corridor −0.5 to −1.5 m. Stride-proxy steps 2–8×, belly clearance
+12–20 mm everywhere. The room falls at 256 come mostly before the GainEvolver's tick-10 000 switch-on
(promoted robot 11 of 14), so they belong to the body, not the evolver. **These are the baselines from
2026-10-19.** Next for the stacks: the room falls on the stiffer body (the turn-ride-the-stroke anatomy of
2026-10-13 is the first suspect), then any re-tuning.

### 2026-10-20 — TERRAIN ADAPTATION: the LPG stacks are HIGH-CENTRED on a tall pyramid (0/6 escapes vs the original's 5/6); STANCE REACH frees them (5/6, faster than the original) but winds up after a fall (`PARTIAL`); NO STACK climbs from the floor onto the 30° wall in 60 s — the LPG robot stalls pressed against the wall's foot

*Plan §7.15.* Operator: the original adapts to terrain and unsticks itself anywhere; the LPG walker
"looks like a wind-up toy". Escape test (dropped at tick 1200, 3000 ticks, measured body, 2 mm / 256,
no GainEvolver, n = 6): pyramid i2 original 5/6, LPG robot 0/6, LPG sim 0/6 (belly 0.66–0.77 BW, the legs
cycling in the air). Walls: LPG robot 4/6 per side with 0 tip-overs, original 1/6 and 3/6 with 10
tip-overs. The robot config lacks the planted-leg corner integrator (the sim stack levels on the hump;
the robot config pitches beyond the slope). **Lever** `reach_rate` (+ `reach_load_topic` and friends):
leg extension grows while the phase says stance and the self-normalised foot load is below half.
Gain 0 byte-identical by construction. At 0.01: pyramid i2 0/6 → 5/6 (median 1232 ticks vs the
original's 2071), other placements roughly held (pyr0 5/6, left wall 3/6 with 4 falls); flat arena
4.46 → 4.05 m, falls 0 → 0.67, all on one seed that wound up after a fall. At 0.03: `REGRESSION`.
Verdict `PARTIAL` (signal); fix the windup (reset on the reset event, leak floor) before re-testing.
**Wall transition** (`OGMA_PICRAWLER_TELEPORT_YAW_DEG`, new): 0/6 for every stack. The LPG robot reaches the
foot and stays there; the original does not reach the foot in 60 s. Needs a longer window, and the
LPG failure is stepping up onto the slope.

### 2026-10-20 (later) — ROUND 2: the stance-reach wind-up fix does not remove its flat falls (they were not wind-up); the corner integrator on the robot LPG config frees the pyramid 3/6 but tips over on the wall (0/6); NO STACK climbs onto the wall in 5 minutes, at 256 or 64 iterations — the original never passes the wall's foot

*Plan §7.16.* Stance reach with a reset on the reset event and a leak floor: pyramid i2 4/6, left wall
1/6 (7 tip-overs), flat 3.82 m with 0.83 falls (seed 6: 5; extension before the fall only 0.07–0.13).
Corner integrator on the robot config: pyramid i2 3/6, left wall 0/6 (6), flat 3.33 m. Both `PARTIAL`, not
promotable. Five-minute wall test: 0/6 for original, LPG, and both levers, also at 64 iterations; the
original reaches max |x| 0.74 (never past the foot), LPG 0.86. At the foot the LPG robot is pitched
~31° with its front feet on the face and its rear pair loaded and jammed in the corner (rear toes ~0 mm):
the weight-shift problem again. The operator's observation of the original's transition needs its
placement to reproduce. Both levers are in the launcher as EXP entries (GainEvolver removed) for observation.

### 2026-10-20 (evening) — THE PRE-LIMITS REFERENCE RESTORED (P-e at 6.0 rad/s): it reproduces its 8.61 m record to the digit, and it is the ONLY stack that climbs from the floor onto the wall (3/6 head-on, "eventually": median 4.3 min) — the operator's "original" was this servo speed

*Plan §7.17.* Operator: restore the best config from before the robot limits, for performance on future
or modified robots. `…native_measured__tofboom__fsrleg__ref_servo6.json` (MAX_SERVO_SPEED 6.0, GainEvolver
removed), first in the launcher. `arenaavg` at 10 mm / 64: 8.61 ± 0.57 m, straight 0.87, steps 28.17, 0
falls, exactly the 2026-10-01/02 record. At 2 mm / 256: 7.06 m, straight 0.75, 81 steps, 0.17 falls. Against
the 3.668 rad/s stacks (measured body, 2 mm / 256, n = 6): wall transition 3/6 vs 0/6 (rises to 188 mm),
left-wall escape 6/6 vs 1/6 (original) and 4/6 (LPG), pyramid i2 5/6 vs 5/6 and 0/6, hump escape twice as
fast, reaches the rumble strips 6/6. Costs: belly lower (5th percentile 9 mm on the rumble strips), more
tip-overs. The LPG stacks have never been measured at 6.0 rad/s; that is the comparison for future robots.

### 2026-10-20 (night) — THE LPG STACKS AT 6.0 rad/s: on flat ground the servo speed was most of the gap (LPG + stance reach 6.63 m vs the reference's 7.06, straighter, as few falls); stance reach matches the reference on the tall pyramid with no tip-overs; the FLOOR-TO-WALL TRANSITION stays the reference's alone (3/6 vs 0–1/6) — that part is mechanism

*Plan §7.18.* Same battery as the reference, `MAX_SERVO_SPEED` 6.0, measured body, 2 mm / 256, n = 6. Flat
arena (`arenaavg` 6000): reference 7.06 / straight 0.75 / 0.17 falls; LPG robot 6.44 / 0.83 / 0.33; LPG +
stance reach 6.63 / 0.84 / 0.17; LPG sim 5.96 / 0.74 / 0.33. Pyramid i2: 5/6 / 0/6 / **5/6 (0 tip-overs)** /
3/6. Left-wall escape 6/6 / 5/6 / 4/6 / 0/6. Wall transition head-on 3/6 / 1/6 / 0/6 / 0/6. Posture change on
the hump: reference 0.037–0.047, LPG robot 0.016–0.018. Stance reach at 6.0 rad/s: `PARTIAL → WORKING
candidate` for flat + pyramid (its 3.668 flat cost is gone); not for walls. Harness fix: `--drop-module`
on an already-absent module now warns instead of exiting (it had silently dropped one arm's runs, caught by
the n = 0 rows).

### 2026-10-20 (night) — THE CADENCE: LoadPhaseGait's `period_ticks` is a clock the servo speed never moved (stride 1.84 s at 3.668 AND 6.0 rad/s); at 6.0 rad/s and period 30 (0.82 s), LPG + stance reach covers 8.95 m vs the reference's 6.14 with 0 falls and escapes pyramid and wall without tip-overs — the floor-to-wall transition and hump posture adaptation stay the reference's

*Plan §7.19.* Sweep 70 / 50 / 40 / 30 (executed stride 92 / 66 / 53 / 41 ticks, stroke 0.86–0.90 rad, no slew
clipping). Flat (6000, same harness): 6.19 / 6.88 / 7.47 / **8.95 m**, 0 falls throughout; reference 6.14, 0.17
falls. Pyramid i2 5–6/6, left wall up to 6/6 with 0 tip-overs at 30, hump escape 254 ticks (reference 246).
Wall transition 0/6 at every period (rise ≤ 90 mm; reference 3/6, 188 mm). Hump posture change 0.021–0.025 vs
the reference's 0.037–0.047. In the launcher: "EXP · LPG + stance reach at 6.0 rad/s, period 30 (sim)". Scorer
fix (drop tick read from the teleport line); the only rescored arm (stance reach 0.03) keeps its `REGRESSION`.

### 2026-10-20 (night) — THE FEMUR LEVEL (`hip2_tuck_target` 0.3 → 0.0): a lower, spider-like stance lets the LPG walker climb from the floor onto the wall 5/6 head-on (the reference 3/6), with zero tip-overs on every escape and the reference's posture adaptation — `WORKING` (signal, sim)

*Plan §7.20.* Operator: the chassis rides too high; with the longer FSR leg the femur should sit near horizontal, as on the
real robot. On the latest config (LPG robot + stance reach, 6.0 rad/s, period 30), measured body, 2 mm / 256, n = 6. Tuck 0.3
/ 0.15 / 0.0 / −0.1: stance femur +0.25 / +0.14 / +0.03 / −0.03, chassis 101 / 92 / 89 / 78 mm. Flat 8.95 / 9.50 / 9.26 / 7.93 m
(falls 0 / 0 / 0 / 0.33). Pyramid i2 5/6 / 5/6 / **6/6** / 3/6. **Floor → wall head-on 0/6 / 1/6 / 5/6 (median 4952 ticks) / 4/6.**
Hump posture change 0.021–0.025 / … / **0.034–0.050** (the reference 0.037–0.047). Belly 5th percentile 60 / 47 / 30 / 23 mm,
belly load ≈ 0 except at −0.1. 45° wall approach: 0/6 throughout. In the launcher: "EXP · LPG + stance reach, 6.0 rad/s, period
30, femur level (sim)". Re-use context on the robot: its LPG config carries tuck 0.3 while the operator reports the real femur level.

### 2026-10-22 — SELF-RIGHTING: the robot-faithful distress NEVER FIRED upside down (stride odometry coasted to a phantom 0.76 m/s with no foot on the ground); gated on planted feet, the LPG walker panics and rights itself 9/30 vs the reference's 5/30, walking byte-identical — `WORKING` (sensor fix)

*Plan §7.21.* Inverted drops (`TELEPORT_FLIP=1`, inversion auto-reset off, 15 000 ticks, measured body, 2 mm / 256,
n = 6) on the channel floor, pyramids i2 and i0, both wall slopes. Reference 5/30 righted, LPG latest 1/30 with
distress 0.00 and panic 0 % while inverted (legs walking, sd 0.39 rad). Same brain with the sim's world-position
distress: 8/30. **Cause**: `HONEST_DISTRESS`'s stall score reads stride odometry, which coasts on the accelerometer
when no foot is planted; inverted it reported 0.76 m/s (true 0.01). **Fix**: `DistressAccumulator::step(..., n_planted)`
— zero displacement on ticks with no planted foot (−1 = old arithmetic, robot host unchanged); sim passes the FSR
stance count (`OGMA_PICRAWLER_DISTRESS_PLANTED=0` restores). Tests `test_distress` 3/3. Result: 9/30 righted, panic
70–85 % while inverted; flat and corridor byte-identical, walking panic 0 %. Open: the tall pyramid inverted (0/6 vs
the reference's 4/6); the robot host must pass its FSR count; the confidence handover is next.

### 2026-10-22 — CONFIDENCE HANDOVER: the LPG terms scaled by support × upright, + white search noise — self-righting 9/30 → 2/30 (3/30 with noise); walking ties — `REGRESSION` (signal) on self-righting

*Plan §7.22.* `conf_gain` 1 scales stance press, stance reach, leg-phase lift and phase stroke by
`ramp(support EMA) × ramp(upright)`; `conf_noise` 0.2 adds `(1−conf)·0.2` to the babble sigma. conf_gain 0
byte-identical (6 seeds). Withdrawing the LPG terms at low confidence REDUCED leg motion while inverted (sd 0.65 →
0.46 rad): the walking terms writing into an inverted body were part of the thrash. Only the tall pyramid improved
(0 → 2/6). Flat 9.53 m vs 9.26, wall climb 4/6 all arms, escapes and corridor within a seed. **Later found**: the
upright factor was `DEAD_CODE` here — the LPG lineage never subscribes motor_epm to `reality.proprio.upright` and
the honest tilt estimate reads BOTH pitch and roll as 180° on its back (`pitch_roll_from_up`, full-range atan2), so the motor's tilt fallback `upright_ = cos(pitch)·cos(roll)` reads +1 inverted (corrected 2026-10-23; first written as `publish_tilt` false — tilt is published); confidence fell through the support term alone (§7.23). Re-use
context: the handover is the right frame once the search it hands over to is shaped (below).

### 2026-10-22 — SHAPED SEARCH NOISE (OU tau 15, knee fold side ×0.27, escalation ×1→×3) on the handover: self-righting 3/30 → 9/30, both pyramids 4/6; flat ties, corridor slower — `PARTIAL` (signal). Inversion-triggered panic: `DEAD_CODE` (no upright subscription)

*Plan §7.23.* Operator: the noise ramps too slowly, pulls the knees inward, and rarely reaches hip2-up + knee-extend.
Causes: white noise in command units (the servo slew averages it away) and the knee command's asymmetric map (fold
×3.20 rad, extend ×0.85) making symmetric command noise a 3.8× inward bias in radians. `conf_noise_shaped` 1: per
joint OU process, knee fold-side scaled 0.27, amplitude ×(1 + low-confidence ticks/50) capped at 3. Inverted: tall
pyramid 4/6 (median 170 ticks, = reference), pyramid i0 4/6 (reference 0/6), floor 0/6, walls 1/6 + 0/6; total 9/30
vs 3/30 white, 9/30 gated base, 5/30 reference. Flat 9.69 ± 1.34 m, 0 falls. Corridor: flat zone 817 vs 666 ticks,
hump 780 vs 381, rough zone reached 4/6 vs 5/6 with one fall — short confidence dips on rough ground now draw
stronger noise. `conf_panic_upright` −0.2 matched handover+noise and shaped to the digit: the upright input never
arrived (see above). Re-use context for the corridor cost: an onset delay on the noise so short dips do not draw it.

### 2026-10-22 — UPRIGHT INPUT + INVERSION PANIC on the shaped handover (round 2): panic latches 94–100 % of inverted ticks, righting 7/30 and 6/30 vs 9/30 — `NULL` (signal)

*Plan §7.23.* `upright_topic: reality.proprio.upright` makes the handover's upright factor and `conf_panic_upright`
live (round 1's were `DEAD_CODE`). Confidence 0 on every inverted tick; panic engaged; no gain in righting. Flat
byte-identical; corridor 630 / 376 ticks, 5/6, 0 falls (shaped 817 / 780, 4/6, 1 fall; base 666 / 381, 5/6, 0) — the
shaped arm's corridor cost did not reproduce here, cause not traced. Re-use context: the inversion trigger is the
right gate for a panic that has something directed to do.

### 2026-10-22 — PANIC-SUSPENDED HANDOVER (`conf_panic_suspend` 1): the learned layer's full-range wind-up restored (hip1 clipped 8 % → 60–63 %), self-righting 9/30 = shaped 9/30 = gated base 9/30 — `NULL` (signal), mechanism verified

*Plan §7.24.* Diagnosis first: the original's thrash is HK winding up on the LPG terms (pre-clamp |u| hip2 7.4 at
10 000 ticks inverted, HK share 0.98); the handover starved it. Suspending the handover in panic restores the
wind-up exactly, but not the floor (0/6 vs the base's 3/6, p ≈ 0.18 at n = 6) and not the walls. Gain 0 byte-
identical. Walking ties except the pyramid escape (faster, 6 tip-overs vs 2). Lesson: amplitude is not the gap;
undirected thrash rights the robot by chance. Re-use context: behind a directed exploration.

### 2026-10-22 — HONEST LEG–CHASSIS COLLISION (`OGMA_PICRAWLER_LEG_SELF_COLLIDE=1`): self-righting 9/30 → 27/30 is a PHYSICS ARTIFACT — the depenetration of a toe sunk into the chassis flings the body over (tilt 0.9–2.2 rad in one tick) — `TAUTOLOGY`-class measurement outcome, not a verdict

*Plan §7.25.* The lower capsules mask the world only, so a folded toe passes through the chassis (geometric probe:
5.5 % of inverted leg-ticks on the shaped config). With the lower legs also masking `_LAYER_CHASSIS`, every floor
and wall righting followed toe–chassis contacts with a single-tick tilt change of 0.9–2.2 rad; without the
collision no run, righted or not, exceeds 0.18 rad/tick, and the pyramid rightings (no chassis contact) roll at
0.08–0.14. At 6.0 rad/s the toe moves ~10 mm a tick, the capsule radius, so it sinks deep into the 41 mm base box
before the contact resolves. Batch stopped. Re-use context: the collision is usable for a verdict only after a
fidelity fix (continuous collision detection on the lower legs, or a gentler contact bias) passes a tilt-rate gate
(max single-tick tilt change under ~0.2 rad, as without it). Instrument kept: `OGMA_PICRAWLER_SELFCOL_PROBE=1`
(`sci` = legs overlapping the chassis, geometry only; agrees with the physics contacts 39/51 with collision on).

### 2026-10-23 — SELF-COLLISION KEEP-OUT (`selfcol_knee_max`, knee fold capped against hip2 from the body's own FK map): toe-in-chassis while inverted 5.28 % → 0.14 %, walking ties, self-righting 9/30 → 6/30 within n = 6 — `WORKING` (signal) as a body limit, PROMOTED by the operator

*Plan §7.25.* Clean A/B without the sim collision (parked, see the entry above), geometric probe on both arms; the
control reproduced the shaped arm to the digit. Only 1 of the control's 9 rightings used a toe-in-chassis pose, so
the lower count is not a removed cheat. Residual intrusions: the cap reads the commanded hip2, which leads the joint.
Re-use context: the curve is per body; re-map for cad (no electronics block) and for any leg change.

### 2026-10-23 — ROBOT: the sim's femur level (`hip2_tuck_target` 0.0) splays the real robot's legs and puts the chassis near the floor; the robot candidate goes back to 0.3 — a sim–robot posture mismatch, recorded, not yet explained

*Operator, on the robot with `…__robot_cand.json`:* "the legs splayed out … in the previous LPG config on the robot
the femur was more perpendicular with the ground … the chassis very close to the floor." The candidate had inherited
the sim's 0.0 (plan 7.20: chassis 101 → 89 mm in the sim, floor→wall 5/6). Robot candidate set back to the robot
config's 0.3; the sim configs keep 0.0. Open: why the same command sits lower on the robot (load sag in the real
servos, or a hip2 zero offset) — measure the real chassis height and femur angle at 0.0 and 0.3 against the sim's
before treating any sim posture result as transferable.

### 2026-10-23 — LEARNED STANCE HEIGHT (`tuck_learn_*`, plan 7.28) and its LEAK (7.28b): no fall cost in the sim, but as a pure integrator it ran the femur into its bounds; the leak keeps it off them — `PARTIAL` (signal) for the learner, `WORKING` (mechanism) for the leak

*Robot config `…__robot_cand_tl` at 4.22 rad/s, period 50, measured body, n = 6 × 9000.* Learner on vs off: flat
8.64 ± 1.96 vs 8.48 ± 1.30 m, corridor 3.62 vs 3.66 m, resets 1 vs 1 and 1 vs 0; 100k ticks (8 seeds, cut at ~90k by a
machine crash) 6 vs 11 resets. But the unleaked integrator pinned the femur at −0.1 (near level, the splayed stance
the operator rejected on the robot) on 3 of 6 flat seeds and at 0.5 over 100k ticks: the femur angle alone cannot
always close the belly error. Narrowed range 0.15–0.45: ties, still at its bound on 3 flat seeds. **Range + leak
0.002 toward 0.3: never at a bound, femur 0.21–0.32**, belly 71–77 mm (it tracks the 65 mm target less), flat
7.46 ± 1.43 m with tilt_sd 0.093 vs 0.073 and 2 resets vs 1 (within n = 6), corridor 4.23 vs 3.62. Gain 0 per-tick
identical (6 seeds). The robot config takes range + leak. Robot run 2026-10-23 21:02 (operator: "thrashing rather
than stepping") ran at 3.12 rad/s and period 30 carried over from earlier tuning, target 30–40 mm: 9 HAT resets, belly
p10 7 mm — the cadence outran the servos and the low target lowered the stance; not a test of the learner. Re-use
context: belly height wants more than one actuator (hip2 alone); a learner over the stance press or the knee would
share the load.

### 2026-10-24 — ROBOT PROMOTED (operator): `…__robot_p1.json` — the promoted sim stack + learned stance height, 4.22 rad/s, period 50, swing lift 0.40, belly target 60 mm, as tuned live on the bench

Bench run 2026-10-24 00:06 (5.7 min): with swing lift 0.35–0.50, the swinging foot still touches during 33–69 % of
swing samples per 15 s window (best 33 %, worst 69 %), and the belly falls over the run (63 → 34–38 mm) while the femur sits
at its 0.45 limit from ~150 s: the 60 mm target is not reached on this robot; the body sags (servo give or pack
voltage, not measured here). Dragging tracks the low belly. Rail min 2.8–3.3 V, current p95 2.1–2.8 A, 0 HAT resets.
The previous robot promoted config is relabelled BASE. Next: make the swing lift and stance height adapt
(discussion with the operator).

### 2026-10-24 — LEARNED SWING LIFT, per leg (`swing_learn_*`, plan 7.29): the sim's swing contacts are early landings, the robot's are a foot that never leaves — counting only the rising lift, `PARTIAL` (signal, sim) and safe; robot bench trial pending

*Robot `…__robot_p1` in the sim (4.22 rad/s, period 50, lift 0.40), n = 6 × 9000.* A leg's swing-lift multiplier
rises while its foot touches mid-swing (debounce 3 ticks), leaks back to 1, max 2. Whole mid-swing window (sin φ > 0.3):
touching flat 28 → 23 %, corridor 32 → 24 %, but multipliers wound to 1.4–1.8, sway 0.075 → 0.097 (corridor), resets
0 → 1–2, pyramid escape tip-overs 5 → 8: `PARTIAL`, not promotable. Diagnosis: in the sim the touching toe sits in the
SECOND half of swing (after the lift peak; 100 % toe, never the shank) — an early landing more lift cannot fix. On the
robot (2026-10-24 00:06) the foot touches 85 % of samples just after liftoff and 44–54 % through mid-swing: it does not
leave. `swing_learn_rising` 1 counts only the first half of swing: touching while the lift rises flat 22 → 16 %,
corridor 29 → 25 %, multipliers 1.14–1.58, resets 0 and 0, sway unchanged. Gain 0 per-tick identical (2 seeds, two
configs). Bench trial config `…__robot_p1_swl` with the rate live on picrawler-dash (k / l, 0 = off). Re-use context:
the sim rarely has the robot's failure; the robot is the test.

### 2026-10-24 — ADAPTIVE PERIOD (`period_adapt_*`, MotorEPMv2 `track_topic`, plan 7.30): the lag instrument does not see servo speed — `NULL` (the instrument, not the idea)

The motor published |previous command − servo-model position| over the command's stroke; the gait lengthened its
period when that ratio exceeded the lag at the operator's 4.22 rad/s / period 50 (1.04). With the period fixed, the
ratio reads 1.00 at 3.668 rad/s AND 1.00 at 5.0 rad/s: it measures the command's tick-to-tick jitter, not the slew
limit. Fed it, the period ran to its bounds (30 or 90). Publishing the ratio with the adaptation off is per-tick
identical (6 seeds). Code kept, off. Re-use context: an instrument that does move with speed — the achieved joint
excursion over a stride against the commanded excursion. Side note: fixed period 50 walks 8.11 ± 2.30 m at 5.0 rad/s
vs 5.44 ± 3.33 m at 3.668.

### 2026-10-24 — ROBOT BENCH TRIAL of the learned swing lift (`…__robot_p1_swl`, 407 s, learner on throughout): `NULL` on the robot (n = 1 run) — and the belly sag is NOT the pack

**Sag:** pack voltage (new 1 Hz record) 7.39 → 7.30 V mean per minute (peaks 7.88 → 7.93) over 7 min, while the belly
fell 48 → 41 mm in the first ~2 min and then held, with the learned femur at its 0.45 limit from ~80 s. The body settles
lower under load with the battery steady: the servos give (or the stance geometry cannot hold more), not the pack.
**Learner:** per-leg multipliers climbed to 1.6–1.95 (cap 2.0) within 80 s, yet touching in the first half of swing
went 67/53/72/67 → 62/51/63/65 % (FL/FR/RL/RR) and by swing sixth 85/54/44/45/53/64 → 81/49/42/44/49/53 %, against
the 00:06 run (no learner, lift 0.40–0.50 by hand). Commanded hip2 range per second rose 285 → 322 µs while the slew
delivered ~70 % of it both times (196 → 226 µs). Mid-swing contacts are light (median 0–2 g): the toe skims at floor
level. More lift command does not become clearance. Body: when the FL+RR diagonal swings the chassis tips roll −0.061 /
pitch −0.071 (sin), vs +0.003 / +0.012 for FR+RL — ~4° toward one swinging diagonal, ~5–7 mm at a foot, the order of
the clearance. Re-use context: a lift that the slew can complete (longer period or faster servo), or a within-stride
weight shift off the swinging diagonal; the learner itself works as built (it raised the commands).

### 2026-10-24 — LEARNED STANCE PRESS (`press_learn_*`, plan 7.31): over the gait phase it does not stop the diagonal tip and rocks the body; as ONE CONSTANT PER LEG it holds the belly at its target (66 vs 65 mm, where the femur learner overshoots to 77–79) with no stability cost — `WORKING` (signal, sim, full-strength servos) as the height owner

*Operator: the push from the planted legs is what should be learned to hold the chassis up.* On a planted leg
P(φ) = a + b cos φ + c sin φ is added to hip2 and the knee; a, b, c step by rate × deficit × (1, cos φ, sin φ), deficit =
target − belly + 0.08 m × (front·pitch + left·roll) (the corner integrator's validated sign). Robot `…__robot_p1` in
the sim, n = 6 × 9000. **Round 1, target 80 mm, phase on:** belly 79 (press) vs 75 (femur) mm, but the diagonal tip
did not fall (0.160/0.139 vs 0.130/0.144) and sway rose (corridor 0.273 vs 0.123, resets 9 vs 6); press + femur at full
rate fought (corridor sway 0.636). The 80 mm target itself destabilised the femur arm too (5 resets vs 0 at 60).
Why the phase terms fail: two diagonal feet are a hinge; legs on the support line cannot resist rotation about it, so
the tip is a centre-of-mass placement problem (§7.32), not a press. **Round 2, target 65 mm, `press_phase` 0:** belly
66 mm (press) vs 77→79 (femur), flat 10.40 ± 1.20 vs 7.01 ± 2.01 m, straight 0.50 vs 0.32, sway 0.072 vs 0.083,
resets 1 vs 1; corridor 4.88 vs 3.52 m; escapes tie. Press + femur at 1/10 rate cooperates (9.61 m, corridor 5.18,
resets 0). The walking gain is mostly the lower stance (11 mm), which also doubles early-swing touching (9 → 19 %).
Gain 0 per-tick identical (2 seeds). Re-use context: the robot's sag is not in the full-strength sim — the decisive test
is at a servo strength calibrated to the robot (queued), then the bench.

### 2026-10-24 — SIM SERVO SAG CANNOT BE EMULATED WITH THE EXISTING KNOBS: weaker servos are a cliff, and an error-proportional force cap has no effect — `NULL` (fidelity, not a lever); the joint model must be understood first

*Goal: reproduce the robot's belly settling (~41 mm with the learned femur at its 0.45 limit) in the sim, to test the
learned stance press fairly.* robot_p1 at 4.22 rad/s, period 50, femur learner at 60 mm, n = 6 × 9000. `leg_strength`
1.0 / 0.6 / 0.4 / 0.25: late belly 68 / 76 / 78 / 77 mm, femur never pinned. New `OGMA_PICRAWLER_SERVO_COMPLY` (the
motor's impulse cap scaled by |error| / c, so a loaded joint should give until the error carries the load): c 0 / 0.05 /
0.1 / 0.2 / 0.3 rad → 68 / 76 / 74 / 69 / 76 mm; single seeds at c 1 / 3 / 10 / 30 → 70 / 52 / 68 / 66 mm. Strength
0.03 / 0.01 / 0.003 (single seeds) → 71 / 63 / 26 mm (0.003 collapses, no forward motion). c 0 is per-tick identical
to the strength 1.0 arm (2 seeds). Reading: the servo is ~100× stronger than standing needs, and something other than
the motor impulse holds the joints (the 6-DOF joint's springs / limits / damping, cf. the motor_force_scale units
comment in picrawler_body.gd). Re-use context: a sag model built on the joint backend itself, validated by a static
load test (body weight on a planted leg → measured deflection vs the robot's), before any press A/B depends on it.

### 2026-10-24 — LEG-LESION BATTERY: the original walker at 6 rad/s keeps 74 % / 43 % of its forward progress with a rear / front leg frozen; the LPG walker keeps 10-26 %, and still 20-23 % at the same 6 rad/s — the lesion weakness is the LPG mechanism, not the servo speed — `BASELINE` (signal)

*Operator: the original walker recovered from total leg ablation; LPG struggles.* One leg's commands ×0 from tick 3000
(`OGMA_PICRAWLER_LESION_*`: the leg holds its neutral pose), flat arena, n = 6 × 9000; net progress per 1000 ticks over
ticks 4500-9000, as a share of the same config unlesioned. Original walker at 6.0 rad/s (`ref_servo6`): rear-left 74 %,
front-left 43 %, straightness after 0.25 / 0.15. Original gait at 3.67 rad/s (BENCH robot stack): 14 % / 21 %. LPG
promoted robot config at 4.22 rad/s: 10 % / 26 %. LPG at 6.0 rad/s, period 30 (`…__sim_s60_p30`): 20 % / 23 %, while it
is the fastest walker unlesioned (2.35 m/1k before the lesion vs the original's 1.08). After a lesion every config's
legs keep moving (path 2-3 m/1k); only the original at 6 rad/s still translates. Reading: the original's lesion recovery
needs both its mechanism (its learned layer re-adapts to the body it has) and servo speed; LPG's load-held phase gives
the frozen leg no load, so its phase runs free and the others do not reorganise. Re-use context: the per-situation
operating points of generator_and_belief_filter.md §3.3 (a lesion as a situation with its own settings), or a LPG
coupling that redistributes when a leg's load share stays zero.

### 2026-10-24 — THE GENERATOR AND THE BELIEF FILTER (generator_and_belief_filter.md G1-G3, instruments): the generator converges to the exact floor on the bench and predicts footfalls half a stride ahead on the body; the belief filter beats the EPM's single winner 2-3x and a best-tuned single-hypothesis Kalman filter 1.6-6x; a gait-context vocabulary separates a frozen leg (NMI 0.56 vs 0.23) but terrain zones only weakly — G1 `WORKING`, G2 `WORKING` (bench), G3 `PARTIAL` (signal)

**G1 (Generator, an EPM's decoder).** Bench S9 (K 4, p_stay 0.95, n = 20): horizon 0, second half of 24 000 ticks, 0.9 ±
1.8 % over the exact floor (range −2.5..+4.0 %); horizon 5 / 20 within 6 / 8 %; NIS 0.91-1.24. Body (robot_p1, body_pose
EPM, 12-14 nodes): foot contact half a stride ahead +0.79..+0.81 vs persistence, +0.44..+0.51 vs the mean, NIS 1.2-1.6
(after a relative variance floor; one channel had read 45-90); belly height ~unpredictable from pose (+0.15 / −0.10): a
slow variable. **G2 (BeliefFilter).** Bench S10 (K 4 overlapping, p_stay 0.97, 60-tick silences every 400, n = 20, MSE of
the estimate vs the true mean): noise 0.4 — raw 0.020, EPM winner 0.0047, filter 0.0015, best Kalman filter 0.0089,
true-model optimum ~0; silent: stale winner 0.104, filter 0.080, Kalman 0.088, optimum 0.067. Noise 0.8 — 0.080 / 0.024 /
0.012 / 0.020 / 0.0008; silent 0.120 / 0.083 / 0.087 / 0.067. The gap to the optimum with readings is the vocabulary
(222-244 nodes for 4 states; insertion_autotune does not change it). **G3 (FeatureWindow → slow EPM → Generator).**
robot_p1, 6 seeds: winner vs label NMI, lesion before/after 0.56 ± 0.10 vs the unlesioned control's 0.23 ± 0.07 (chance
0.20 / 0.12); corridor zones 0.31 ± 0.06 vs chance 0.22. Its generator (100 ticks ahead) +0.06..+0.12 vs the mean
unlesioned, +0.49 lesioned, NIS 16-36 (few samples per node, a drifting summary). Bugs found and fixed on the way: a
horizon counted in queued readings (sub-rate target), an EPM sub-rate gate that dropped every off-grid input, a
no-input token clearing the generator's situation. Re-use context: G3 terrain needs features that differ between
zones at the window's scale (or a shorter window); G4 (choosing settings) needs per-node outcome models and an
exploration schedule — a behaviour change, for the operator.
**G3 conditioning check:** the same instrument with the EPM commissioning its own per-channel ranges
(`dim_autocal_ticks` 40 windows) instead of hand-set ones — corridor zones NMI 0.28 ± 0.04 vs chance 0.20, lesion
0.49 ± 0.20 vs the control's 0.22 (6 seeds): unchanged. The commissioning flags 4 of 26 channels degenerate (the
StuckSense score and deficit, constant in these worlds). Terrain separation is limited by the features at the
two-stride scale, not by conditioning (CLAUDE.md §0 rule 2 checked).

### 2026-10-24 — ROBOT SAG FROM THE LOGS (`pi_host/tools/sag_from_log.cpp`, runs robot_20261024 / 24b, 355 s / 416 s): with the robot standing, the commanded-kinematics height and the belly ToF differ by a near-constant ~32 mm at both command heights; walking adds ~20 mm, growing over the first 2 min and with servo current, and not with the number of planted feet — `BASELINE` (instrument, n = 2 runs)

*Operator: investigate servo sag on the real robot before simulating it.* The servos report no position, so the tool
runs the host's own kinematics (`feet_y_gravity_cmd_fk`, ServoMapping, BodyCalib) on every commanded pulse in the
50 Hz feed: each FSR-planted foot's height below the body if the servos tracked exactly, against the belly ToF.
**Standing still** (4 planted, commands unchanged for 0.5 s): gap 33.8 mm at a commanded 93 mm and 29.9-33.9 mm at
45 mm — a near-constant offset across command heights: the body origin vs the belly, the foot model and the ToF's
3.2 mm permissive offset, not load sag. **Walking** (median, both runs): 54.4 / 52.1 mm, i.e. ~20 mm more; by time
0-30 s 36-39 mm, 30-60 s 45 mm, 60-120 s 50-52 mm, then 52-57 mm held. Current quintile 0.5-0.6 A → 2.3 A: 43-47 →
57-59 mm. Planted feet 2 / 3 / 4: 53-56 / 50-53 / 56 mm (no dependence). Tilt cannot explain it: |pitch| and |roll|
during walking have medians ~0.05 rad and p90 ~0.12, a few mm at the boom. One 0.2 s stand at the end of run 1 (before
the robot was lifted) read 33.9 mm, the start's value: weak evidence that the excess exists only while moving
(tracking lag or give under dynamic load) and is not heating. Caveats: assumes level floor; the slew delivers ~70 % of
the commanded hip2 range (2026-10-24 swl entry), so lag and give are not separated here. Re-use context: a protocol
run — stand 20 s at the working pose, walk 3 min, stand 20 s at the same pose, then static loads on the chassis — to
split lag, give and drift before any sim sag model is built (the SIM SERVO SAG entry needs exactly this number).

### 2026-10-24 — G3 TERRAIN FEATURES: leg posture (femur and knee mean and spread over the window) raises the context vocabulary's terrain separation above chance from 0.08 to 0.14 NMI; conditioning variants change nothing; one shared EPM dilutes strong channels with weak ones — G3 terrain still `PARTIAL` (signal)

*Operator: if separation is weak the encoder may be to blame — analyse the input and its PCA.* **Input** (6 corridor
runs, 100-tick windows every 25, 1598 windows): PC1 (27 % of variance; foot load and contact means) separates the rumble
strip, flat and hump overlap; 2 channels are constant (StuckSense deficit mean and sd). Supervised kNN, leave-one-run-
out, balanced accuracy: the 26 deployed channels 0.70 (recalls hump 0.57), + posture 0.82 (0.81 / 0.81 / 0.84),
posture alone 0.72; accelerometer, stride, step-count and forward-range features add nothing. **Conditioning** (the
recorded windows through the real EPM, new `cpp_core/bench/epm_replay`): hand-set ranges squeeze the data into ~12 % of
each range (latent PC1 zone separation 0.23, NMI 0.20 vs chance 0.16), self-commissioned 0.79 (0.43 vs 0.29), running
z-score 0.38, whole-run z-score 0.39; λ 5 / 2 give 41 / 98 nodes with no gain over chance. Live (6 seeds): k 1.5, and k
1.5 with min_insertion_error 0.005 / 0.001 — 0.30 / 0.29 / 0.29 vs chance 0.22-0.23 (the k 4 baseline 0.28 vs 0.20).
**Posture live** (`…__robot_p1_ctx_pos`, + `reality.proprio.joints:4-11`, 42 channels): corridor 0.39 ± 0.05 vs chance
0.25; lesion 0.57 vs control 0.30. Posture only (`…__robot_p1_ctx_posonly`, 16 channels): corridor 0.44 ± 0.07 vs
chance 0.34; lesion 0.57 vs 0.31. Offline the posture-only EPM looked best (0.57 vs 0.25); live, its chance rises
because posture drifts with the slow learners and the corridor's zones come in a fixed order, so anything that drifts
reads as zone. The same drift gives the unlesioned control 0.30 for a before/after split. Reading: the features
limit separation, not conditioning (§0 rule 2 checked twice); an unsupervised vocabulary of ~10 nodes spends them on
the largest-variance axes, so the channels that go into ONE EPM matter (kNN gains from combining, the EPM barely). Re-use
context: one EPM per sense group fused above them (fusion_notes.md), and a terrain test whose zones repeat in varied
order so time drift cannot pass for terrain.

### 2026-10-25 — TERRAIN ARENA CAPABILITY MAP (gym "terrain", terrain seed = run seed 1-6, 24 000 ticks): both walkers stall at 38-50 mm VERTICAL edges (steps, bars); the bowl is barely reached; the forward ultrasonic's avoid turn costs a third of the reach — `BASELINE` (signal)

Promoted robot config (`…__robot_p1`, 4.22 rad/s) and the ideal-servo reference (`…__sim_s60_p30`, 6 rad/s),
deployed stack vs ultrasonic OFF (`OGMA_PICRAWLER_ULTRASONIC=0`; operator call, memory
ultrasonic-off-for-walking-tests). Farthest radius: robot_p1 7.8 ± 2.5 m with the sensor vs 12.1 ± 2.2 m
without; sim_s60 10.4 ± 5.0 vs 12.0 ± 1.5. Bands entered (of 5): robot 4/1/2/3/4/4 vs 5/5/4/4/4/5. With the
sensor on, 3 of 6 robot runs spend their last 6000 ticks on flat ground (turned away); off, the failure
points are 38 mm steps (4 runs), 50 mm steps or bars, and the runs that stall sit on that edge for most of
the remaining time (steps 41 % of robot ticks, bars 44 % of sim ticks). Bowl: one run per arm entered it
(robot 16.7 m at the 96 mm pyramid scale; sim 17.5 m at 116 mm, sensor on). 6 rad/s buys path (82-89 m vs
63-66 m) but not terrain reach (12.0 vs 12.1 m off). Rough sectors (pyramids ≤ band height) and the slope
humps did not stop either walker: sloped faces are climbable, vertical edges are the limit — a swing-
clearance problem, the setting a per-terrain choice (G4) would act on. Caveats: the walkers do not choose
sectors (whatever lies along their heading), so a failure point is "the first impassable feature met";
the belly-strike count from gc_raw is not yet a validated metric and is not reported. Re-use context: the
baseline for any per-terrain setting (swing lift on 38-50 mm edges first), and for an ultrasonic gate
(O72): a gate should recover the sensor-off reach while keeping the walls.

### 2026-10-25 — G3 ON THE TERRAIN ARENA (20 varied worlds per arm, 16 000 ticks, ultrasonic off): the gait-context vocabulary does NOT separate edge terrain from flat — `NULL` (n = 20, varied worlds) for this input set; and the rear legs bind at edges — the operator's observation, reproduced at 6 rad/s

Arms: `…__robot_p1_ctx_ac` (26 channels, self-commissioned), `_ctx_pos` (+ femur/knee posture, 42), `_ctx_posonly`
(16). Winner-vs-label NMI against a shifted-label chance: terrain type 0.27-0.28 vs 0.23-0.28; steps/bars vs flat
0.23 vs 0.19-0.21; with an edge-proximity label (within 0.15 m of a step edge, or among the bars — a steps sector is
mostly flat plateau top) 0.23-0.24 vs 0.19-0.21; ≥ 25 mm edges 0.18-0.20 vs 0.13-0.16. Posture adds nothing here;
posture alone sits at chance. The corridor's 0.14 margin (ledger 2026-10-24) was partly its fixed zone order. Reading:
two-stride summaries of contact, load, belly, tilt and posture do not carry "an edge is under me" at this scale;
the design note's own input list names what is missing — early-swing contact and the progress deficit, i.e. the gait's
prediction errors. **Rear binding** (capability logs, swing-phase contact = touching while sin(phase) > 0): at 6 rad/s,
on 25 mm+ edges rear 57 % vs front 31 % (flat 34 / 26); at the robot's 4.22 rad/s both pairs rise alike (43 / 37 vs
35 / 28). New handle `leg_phase_lift_rear_mult` (MotorEPMv2, hot, default 1 = per-tick identical, seed 3) and dash keys
n/m. Re-use context: a context input built from the gait's own errors per leg pair (the situations G4 needs to tell
apart are the ones where the errors differ); the camera's anticipatory channel; G4 with one global context first.

### 2026-10-26 — ROBOT BENCH RUN of robot_p1 with the live rear-lift knob (416 s, autonomous): the stance-height learner pins at its 0.45 limit within ~2 min while the belly sinks 57 → 32 mm, so raising its target (60 → 70 mm) cannot act; swing contact stays 40-55 % on both pairs; four brownout HAT resets cluster late, at the higher lift and target — `BASELINE` (n = 1 run)

Operator: the rear lift seemed to help but steps stayed unreliable; robot_p1 (hand-tuned) is the only LPG config that
walks reliably, the learned variants (swl, press) cause falls. Measured (brain_diag phase joined to the 50 Hz FSR
feed, swing = sin φ > 0, touching = FSR ≥ 200): swing-phase contact front 55 / rear 41 % at the start, 37-47 / 41-52 %
later; belly (ToF median) 57 → 55 → 51 → 41 → 36 → 32 mm across the run; `tuck_learned` 0.34 at 20 s, 0.45 (its limit)
from 13:40:34 to the end. Current mean 1.1-1.5 A, p95 2.1-3.2 A; the rail fell to 0 at each of the four disarms
(13:39:38, 13:43:41, 13:44:27, 13:44:46); a tilt-guard stop (141°) at 13:44:17; v_ema turned negative after
13:42:37. The rear-lift values are unknown: the dash rounded the multiplier to a whole number ("× 1 (was 1)" for every
press) and the brain does not record it — the display is fixed (0.1 steps shown) and the value will be logged.
Reading: on this robot a height learner asked for an unreachable target pins at its bound and becomes a constant
maximum tuck; the belly still sinks (the servos give under load, ledger 2026-10-24 sag entry), so every swing starts
low. A learner at its bound is the effort variable saying the target is out of reach
([[regulated-quantities-hide-their-own-stress]]). Re-use context: a reachable belly target (~45-50 mm here), short
runs before the sink, and the servo-rail hardware fix before any higher-demand setting.

### 2026-10-26 — G4a (LiftDescent) NEVER STEPPED, and the swing-lift dose response says why: at the robot's 4.22 rad/s, more commanded lift does not lower swing contact in the sim and costs stability — G4a `NULL` (the selection never acted: 52 valid trials per run, 0 steps; 8 worlds per arm before an unrelated OOM); dose response `BASELINE` (n = 20 paired worlds)

G4a on the terrain arena (robot_p1 + LiftDescent, ±δ dithers of 0.2-0.25 held 400 ticks): the fitted slopes never
cleared 2 standard errors, so the operating point stayed at 1.00 / 1.00 in every run — a measurement of the dither, not
of the descent. Fixed-setting dose response (robot_p1, terrain arena, 20 varied worlds, 24 000 ticks, ultrasonic off,
paired by world): rear ×1.5 — nothing moves (reach +0.3 m, rear swing contact flat +1 pt); rear ×2.0 — rear swing
contact on flat +4.8 pts (t 2.1), resets +0.7 (t 2.0), reach −1.1 m; all legs ×1.5 — rear flat contact +4.1 (t 2.1),
resets +0.65 (t 2.2), reach −1.8 m (t −1.7), edge contact −7 to −8 pts on 8 worlds (noisy). Reading: the servos cannot
complete a larger lift inside the stride at this speed (the robot's 2026-10-24 entry: the slew delivers ~70 % of the
commanded hip2 range; "more lift command does not become clearance"), so a larger command adds sway, not clearance; the
slope G4a looked for is small and partly adverse, and the terrain swing of the errors swamps it. The 2026-10-25 rear
binding at 6 rad/s (rear 57 % vs front 31 % on edges) is a fast-servo phenomenon. Re-use context: G4a with a handle that
DOES move the errors at this servo speed (stride period, or lift with a longer period that lets the slew finish), or
per-situation trials (G4b) that remove the terrain variance; LiftDescent's machinery passed its unit tests and stays.

### 2026-10-26 — G4b INPUT TEST: a context vocabulary built from the gait's own errors (GaitErrors: swing-phase contact per leg + stall) does not separate edge terrain from flat either — `NULL` (n = 20 varied worlds, terrain arena, robot_p1 at 4.22 rad/s)

`…__robot_p1_ctx_err` (errors only, mean + sd, 10 channels) and `_ctx_errplus` (+ the 26 summary channels). Winner vs
label NMI against a shifted-label chance: errors only — type 0.24 vs 0.21, edge-zone vs flat 0.20 vs 0.18, ≥ 25 mm
0.17 vs 0.13; errors + summary — 0.29 vs 0.25, 0.24 vs 0.19, 0.19 vs 0.14. Margins of 0.03-0.05, as the summary-channel
vocabulary (2026-10-25). Reading, with the dose response the same day: at this servo speed the gait's errors do not
differ much between edges and flat (swing contact 35-45 % on both), so no vocabulary of them can tell the two apart —
the walker is not terrain-sensitive at the two-stride scale. Re-use context: the faster servo (rear binding is loud at
6 rad/s, 57 vs 31 %), event-scale features (the contact error AT an edge rather than over two strides), or the camera's
look-ahead; on the robot the height/press and power problems come first.

### 2026-10-26 — TERRAIN ARENA REBUILT FROM SOLIDS: the heightmap ground let a fast foot through (the operator's front-leg fall-through and physics explosion at the bowl's edge); thick convex solids hold — fidelity fix, not a lever

Godot Physics' HeightMapShape3D is a zero-thickness surface. Leg-sized capsules (r 12 mm, 75 mm, 0.15 kg) launched down
at 3 m/s from 15 mm above flat heightmap ground went through 6 / 300 times; a 0.1 m box floor stopped all of them up to
5 m/s; gentle footfalls held on both (0 / 1600 on the heightmap). (A first drop test pressed a 25 g capsule with 3 N —
12 g — and "showed" every shape leaking; a test that exceeds the foot's real speeds measures tunnelling, not the
ground.) The arena is now a box floor, every pyramid a convex solid (base 1 cm under the ground beneath it), the bowl and
slope humps thick convex cells (0.5 / 0.25 m), a box rim, in 66 static bodies; the same random draws keep every seed's
layout (17 294 pyramids at seed 0); build 0.5 s; drawn from the same solids. Drops: 0 deep failures on flat ground, the
inner rings and the inner bowl at 0 / 3 / 5 m/s; on the steep outer bowl 1 / 1800 capsules wedged into the seam between
two cells down to the floor box (a seam between adjacent convex cells, not a thin surface). Robot walk-outs from the
bowl's edge (5 × 8000 ticks, press at 80 mm, to r 21.3 m): worst leg tip 13 mm below the surface, 0 resets. Capability
and G3 numbers on the terrain arena before this date were measured on the heightmap ground. Re-use context: larger bowl
cells (fewer seams) if the outer-bowl seam ever shows in a run.

### 2026-10-26 — SPEED MAP at the robot's servo speed (robot_p1, 4.22 rad/s, flat arena, LPG period 30-90 × stroke gain 0.4-1.4, 4 seeds × 9000 ticks): with the stroke at 1.0, the period alone sweeps forward speed 4.9 → 7.0 cm/s without a fall; the deployed stroke 0.7 is under-stroked — `BASELINE` (signal, n = 4)

Forward speed (net displacement / time, cm/s), stroke 0.7 / 1.0 / 1.4: period 90 — 3.9 / 4.9 / 5.4 (1 fall); 70 — 3.0 /
5.1 / 5.5 (2 falls); 50 — 5.2 (deployed) / 5.9 / 5.3 (3 falls, tilt_sd 0.154); 40 — 5.4 / 6.0 / 5.8; 30 — 5.9 / 7.0 (tilt_sd
0.128) / 6.9. Hip1 swing actually achieved (p95-p5): ~0.73-0.83 rad at stroke 0.7, 0.92-1.07 at 1.0; at stroke 1.4 it
reaches 1.25-1.40 at long periods but only 1.03 at period 30 — there the slew limit binds. Distance per stride per radian
of hip swing falls with the period (~8 cm/rad at period 90, ~6 at 50, ~4.4 at 30): short strides slip more, which is the
"walking in place" the operator saw when the period shortened alone. Reading: speed is not the period alone but the pair;
a single speed intent can map onto the period at stroke ~1.0, from 90 (slow, efficient) to 30 (fastest), and stroke 1.4
buys no speed and costs falls. Caveats: sim servos do not sag, and faster gaits draw more current (the robot browned out
at 4.5 rad/s, 2026-10-19). Re-use context: a speed meta-control (one intent → period at stroke 1.0), then G4a with the
period as its handle (it moves the progress error, unlike swing lift); robot check of stroke 1.0 at period 50 first.

### 2026-10-26 — ROBOT RUN of robot_p1_presskin (308 s, carpet onto vinyl): the kinematics-aware press and the per-leg corner integrator CANCEL each other — every planted leg reads itself as the high corner, the integrator runs all four legs to −0.30 while the press runs to +0.30, and the belly stays at ~50 mm whatever its target (60 → 80 mm); fix `level_int_zero_mean` built, sim tie — measurement `BASELINE` (n = 1 run, the pattern in 5 of 5 press runs); the fix `IN_FLIGHT`

**What was seen.** The operator raised the belly target 60 → 65 → 70 → 75 → 80 mm late in the run to get the legs
steeper on the slippery floor. The belly ToF did not move (median 44–51 mm in every 10 s bin from 140 s to the end).
`press_a` climbed steadily from 0 to its 0.30 cap on all four legs (cap reached ~255 s). `press_def`, the height deficit,
grew from 5 to 32 mm. Over the same time `level_off`, the per-leg rest-length integrator, fell to its −0.30 bound on
all four legs, almost a mirror image of the press. Both add to hip2 and the knee with the same sign, so their sum
stayed near zero. Measured femur tilt barely moved (FL 79° → 74°), and the shin stayed 25–40° from vertical in stance
(10° at the stand pose).

**Why.** The integrator's error is the corner projection of pitch and roll, integrated only while the leg is
planted. In a trot the swinging diagonal's corners sag, so a planted leg is, on average, the HIGH corner: mean
`corner_err` while planted was −0.069 / −0.038 / −0.029 / −0.005 rad (FL/FR/RL/RR, 76 / 67 / 55 / 48 % of samples
negative). The error is zero-sum over all four legs but not over the planted pair, so the integrator gains a common
mode: it shortens every leg, which is a height command, and height belongs to the press. The same pair of saturated
integrators shows in all five press runs that day (press2 ×2, press3 ×2, presskin). In the sim the common mode is small
(level_off mean ±0.05) because the sim's servos sag less under the swinging diagonal. The sim therefore never showed
the fight.

**The fix (one lever).** `level_int_zero_mean` 1 (MotorEPMv2, hot, default 0 = byte-identical) subtracts the
offsets' mean after every update. The integrator can then tilt the body but never raise or lower it. Sim
(robot_p1_presskin vs `_lz`, corridor, n = 4 × 9000): net_z 6.87 ± 1.16 vs 6.58 ± 0.78, straight 0.71 / 0.71, tilt_sd
0.091 / 0.086, 0 falls in both — **a tie**. The lever landed: the level_off mean is 0.000 in every `_lz` seed and
−0.05 to +0.03 without it. Config `…robot_p1_presskin_lz.json` is in the launcher.

**Not settled by this run.** Where the carpet ends and the vinyl starts (no external odometry; `stride_v` is
kinematic and cannot see slip). Whether a steeper stance helps on vinyl can only be tested once the height command
actually reaches the legs, so this fix comes first. The leg tool `pi_host/tools/leg_jac_from_log.cpp` now prints the
femur and shin tilt per leg.

**Re-use context.** If `_lz` still cannot raise the belly, the next suspect is the press's own reach. A planted leg
whose shin is 30–40° off vertical turns more of its push into splay on a low-friction floor. That would make stance
reach the handle, solved by IK (the open proposal).

> **Addendum (same day).** The operator puts the carpet → vinyl change at ~100 s of run time. The belly ToF agrees.
> Over 50–100 s: median ~52 mm, 10th percentile ~38 mm, 4 % of frames below 30 mm. Over 100–300 s: median ~47 mm,
> 10th percentile ~31 mm, 10 % below 30 mm, with 10 s bins at 16–20 %. The upward excursions also stop (90th percentile
> 62–69 → 53–61 mm). So on vinyl the chassis drops more often and never rises, which fits feet sliding out under a
> splayed stance. The commanded geometry cannot show a slide (no joint feedback); the drops can.
> **benchd, same deploy:** once off the lock, the 10 Hz `vcgencmd get_throttled` that follows each 1 Hz `pmic_read_adc`
> takes 104–105 ms, every time, at idle. With the poll under S.m that was a ~5-tick servo freeze every second, plus the
> catch-up burst after it. New benchd: 0 stalls, 0 overruns, 0 dropped ticks in the first minute.

### 2026-10-26 — ROBOT: `level_int_zero_mean` lets the press hold the belly — median 48–52 → 61–63 mm (target 60), stance shins 32° → 22° from vertical, belly drops below 30 mm on vinyl 9.2 → 1.4 % — `WORKING` (signal, n = 2 runs against 1); and the remaining pauses are ogma_host's own stalls, not benchd's

**Robot (robot_p1_presskin_lz, two runs, 193 s and ~226 s, carpet then vinyl).** The two integrators no longer fight.
`level_off` stays differential (sum ~0, |offset| ≤ 0.27), `press_a` stays within ±0.11, and the press deficit sits
near zero. Belly ToF median: 62.2 / 61.2 mm over the first 100 s and 61.2 / 63.2 mm after (the presskin run: 52.2 /
48.2). Stance shin tilt, median over planted legs: 21–26° (presskin 32°). On vinyl in the second run the belly fell
below 30 mm in 1.4 % of frames, with the 10th percentile at 44 mm (presskin: 9.2 %, 30 mm). The operator saw clean
steps on the vinyl by 134 s. The first run's vinyl section is less clean (7.8 % below 30 mm). Two runs is a signal,
not a finding.

**Pauses.** benchd recorded 0 stalls, 0 overruns and 0 dropped ticks in both runs; its 50 Hz feed has no gap over 100
ms. ogma_host itself froze 0.7–3.6 s eight times (brain_diag: one 3.58 s gap with a single tick). After each freeze it
ran the missed ticks at roughly twice the normal rate, the same replay benchd had. Its stdout, one JSON line per tick,
goes to a file on the SD card. Fix built (`AsyncStdout`: stdout through a pipe and a writer thread; a stall witness; missed
ticks dropped), deployed 2026-10-26 23:04 (Pi smoke test: 250 ticks, 0 stalls, every line delivered); the next run's summary line reads it.

**Power-off.** The second run's files were cut short: meta.json 0 bytes, events and log truncated. The operator
powered off ~4 s after the end, inside Linux's 30 s writeback window. picrawler-dash now calls `sync` at the end of
every run and shows SAFE TO POWER OFF.

### 2026-10-26 — SIM: a slippery floor could not exist — Godot gives a contact the `rough` material's friction, and the feet were rough at 1.5; the SLIP SHEET (terrain gym, `OGMA_PICRAWLER_TERRAIN_SLIP`) — fidelity fix and instrument, not a lever

**The rule, measured** (`scripts_tools/friction_slide/`, a box launched at 2 m/s, mu = decel / g). When either material
is rough, the contact takes the rough one's friction (the higher, if both are rough). Only two smooth materials take
the lower. smooth 0.3 on rough 1.5 → 1.52; rough 1.5 on smooth 0.3 → 1.52; rough 1.5 on rough 0.3 → 1.52; smooth 1.5
on smooth 0.3 → 0.31. **Two older instruments were inert because of it.** The chassis material (smooth, 0.20) drags at
the floor's 1.5. The per-foot slick ablation (smooth 0.05 on one lower leg) left that foot gripping at 1.5. Treat any
verdict that leaned on either as `DEAD_CODE` until re-measured (a separate task is open for the chassis).

**The sheet.** A 2 mm rough sheet at mu over r 2.0–4.5 m (default) replaces ring 1 and its buffer; `_R="r0,r1"` moves
it. With it on, every leg segment (sphere and capsule) is smooth at 1.5, so each contact takes the floor's friction:
1.5 on all the old ground, mu on the sheet. Inert when off and identical when on but unused: arena run with the env
set vs unset, 32 body-log lines byte-identical. **First build was a harness defect:** only the sphere segments got the
smooth material, the lower-leg capsules kept the rough one, and the sheet at mu 0.05 and 0.3 gave identical runs to the
last digit. The capsule fix made it act.

**What the sim gait does on it** (robot_p1_presskin_lz, spawn on a wide sheet r 2–8 m, 2 seeds × 3000 ticks, speed on
the sheet): mu 0.25 → 9.9 cm/s, 0.20 → 8.9, 0.16 → 7.8, 0.12 → 7.4, 0.08 → 5.5, 0.05 → barely moves (1.5 m in a run
that covers 5–6 m at mu 0.3). The belly does not drop at any mu (67–70 mm): the sim's legs do not splay the way the
robot's do, so the sim reproduces the loss of traction but not the robot's sinking. `BASELINE` (n = 2 per mu).
Config `…robot_p1_presskin_lz_slipsim.json` (mu 0.1) is in the launcher; press [6] for the terrain gym.

**Re-use context.** A study of slip adaptation needs a percept of slip first (CLAUDE.md §1 step 2): the yaw the
planted feet predict against the gyro, or the stride odometry against vision. A sim belly that sinks on low friction
would need the robot's splay mechanism (servo compliance under a sprawled stance) in the servo model.

### 2026-10-26 — ROBOT (presskin_lz, 262 s, carpet then vinyl): belly dips follow lift-offs made with the CoG outside the remaining support — 56 % of lift-offs beyond 30 mm outside are followed by a dip, 14 % within 15 mm; the front-right leg is the worst; and the SD card WAS the stall — the writer thread absorbed a 3.2 s write — `BASELINE` (n = 1 run)

**Stalls.** First run on the new ogma_host. 8 stdout writes took over 80 ms (worst 3208 ms, pipe backlog peak 58 KB),
and the tick loop saw none of them: 0 late wakes, 2 ticks dropped, 0 stale-feed holds (the previous two runs had 3
holds each). The SD card blocking a write for seconds is confirmed as the cause of the brain's pauses.

**The operator's reading:** dips come partly from slipping and partly from lifting and swinging with the CoG in the
wrong place. Measured from the 50 Hz feed (`leg_jac_from_log` now prints each toe's position in the chassis frame
and each foot's grams). Support comes from the FSR contacts (≥ 200 counts) and the commanded toe positions; the CoG
is taken at the chassis centre. A dip is the belly ToF falling ≥ 12 mm below its trailing 1 s median: 210 onsets,
48 per minute. At each of 1125 lift-offs, the signed distance from the CoG to the feet still down (the support
polygon; for two feet, the line between them):

| margin at lift-off | lift-offs | dip within 0.4 s |
|---|---|---|
| > 30 mm outside | 203 | 56 % |
| 15–30 mm outside | 338 | 28 % |
| 0–15 mm outside | 408 | 14 % |
| inside | 176 | 18–25 % |

Per leg: a dip follows FR lift-offs 35 % of the time, FL 24 %, RL 25 %, RR 21 %. Before FR's dipping lift-offs the
CoG sits a median 62 mm toward FR from the centre of the remaining feet (51 mm before its clean ones). RR differs most
(+41 mm vs −6 mm). Shifting the assumed CoG 10 mm in any direction changes the margins by a few mm and leaves the
ordering intact. Caveats: the toes are the COMMANDED positions (a slipping or sagging foot is elsewhere), and the CoG
is assumed, not measured. The inside bins still dip 18–25 %, and those are the slips.

**The FSRs cannot locate the CoG yet.** Grams while planted, median: FL 46, FR 56, RL 680, RR 30. The four sum to
705 g when all are down (body 598 g), and RR reads 0 at the stand pose. The counts → grams curve is the predicted
one, and the feet differ by more than 10×. LoadPhaseGait is unaffected (`load_norm_tau` 500 divides each foot by its
own stance reference), but any CoG or load-magnitude estimate needs a per-foot calibration first.

**Re-use context.** The error to give the gait is the one the dips show: a lift-off that drops the corner. The
geometric support margin (commanded FK + contacts, both egocentric) predicts it. The open proposal: gate each leg's
lift-off by its margin, with the threshold learned from the dips that follow, gain-0 guarded, sim first.

### 2026-10-26 — SIM: SUPPORT-GATED LIFT-OFF (LoadPhaseGait `support_gate`) — belly dips per lift-off 29 → 18 % in the corridor, lower in all 6 paired seeds, tilt_sd 0.091 → 0.079, distance a tie — `WORKING` (signal, n = 6); on a mu 0.1 slip sheet 16 → 13 %, 3 of 4 seeds lower — `PARTIAL` (n = 4)

**The lever** (from the robot run above: lift-offs with the CoG far outside the remaining feet are followed by dips).
LoadPhaseGait holds a leg whose phase is about to enter swing while the CoG lies more than that leg's allowance
θ_i outside the feet that stay down: the polygon of the other touching feet, or the line between two. The hold lasts
at most `support_hold_max` ticks (25), so a trot whose support is a line cannot deadlock. θ_i is learned per leg:
after each lift-off the belly is watched for 20 ticks, a drop of 12 mm or more is a dip, and θ_i += rate × (0.1 −
dip). It settles where 10 % of that leg's lift-offs dip. Inputs: the new `reality.proprio.feet_xz` (each toe's [x, z]
in the chassis frame from the COMMANDED pose: sim body `OGMA_PICRAWLER_FEET_XZ=1`, robot `ogma_host --feet-xz`, the
same `toe_body` FK), the contact flags, and `belly_m`. The CoG is fixed at the chassis centre (`support_cog`). Gain-0:
gate off is byte-identical (unit test). The control `_sg0` runs the gate with `support_hold_max` 0: it never holds but
counts everything, and its run is byte-identical to `_lz` (2000-tick arena, body logs equal).

**Corridor** (n = 6 × 9000, difficulty 0.3, paired seeds; dips counted over the last two thirds of each run):

| arm | dips / lift-off | net_z | straight | tilt_sd | falls | belly | lift-offs |
|---|---|---|---|---|---|---|---|
| `_sg0` (watching) | 29.2 % (39, 22, 25, 33, 32, 25) | 6.92 ± 0.85 | 0.71 | 0.091 ± 0.010 | 0 | 61 mm | ~370 |
| `_sg` (gate) | **18.1 %** (23, 13, 20, 21, 22, 10) | 6.96 ± 1.05 | 0.71 | **0.079 ± 0.004** | 1 (seed 5) | 64 mm | ~340 |

Holds: ~170 per run, ~10 ticks each. Lift-offs released at the cap: 13–46 per run, against 181–244 lift-offs the control
would have gated. Learned θ: 0.005–0.04 m, front legs looser than rear in most seeds. The 10 % target is not reached
(18 %): the remaining dips are not about support (bumps, slips). One fall in six seeds against none, not yet read.

**Slip sheet** (mu 0.1, r 2–9 m, spawn on it, n = 4 × 6000): dips 16.1 / 21.6 / 13.7 / 12.2 → 8.6 / 26.2 / 7.5 / 9.6 %,
speed a tie, 0 resets both arms, tilt lower in 3 of 4. The sim does not sink on low friction (belly 63 mm both), so few
of its dips there come from support.

**Next.** Observe in the UI (launcher `…_sg`), read seed 5's fall, then the robot: ogma_host `--feet-xz` and the dash's
FEET_XZ mapping are built and tested (host tests 96/96), not yet deployed. Re-use context, if the robot ties: the
assumed CoG. With the FSRs calibrated per foot, the CoG can be measured at stand and set in `support_cog`.

### 2026-10-27 — ROBOT (presskin_lz_sg, 170 s): one rear leg (sim index RL) folded under again — knee command pinned at +0.58 rad (the self-collision cap) from 90 s, knee lift gain negative, toe pulled in from (−106, −106) to (−47, −49) mm, and the leg's three commands chattering between saturated poses every few ticks; two stance-length terms are blind to the leg's geometry — `KINEMATIC ROUTE` (`kin_route`) built; sim: distance 6.96 → 7.34, falls 1 → 2 of 6, dips 18 → 21 % — `IN_FLIGHT` (the sim does not tuck, so it cannot test the fix)

**What the logs show.** From ~50 s the RL leg's commands (u, leg-major indices 6–8) run the full −1..+1 range, flipping
every 2–10 ticks between (hip1 −1, hip2 +1, knee +0.68) and (hip1 +1, hip2 −1, knee −1). The other legs stay inside
±0.5. +0.68 is the last entry of `selfcol_knee_max` (hip2 at +1): the knee asks for more fold and is capped. Benchd's
output knee for that leg sits at +0.58 rad from 90 s on, and the leg's oscillation amplitude (`amp_leg`) is ~0 from 70
s. Its corner integrator reached +0.29 at 99–108 s, then fell back to +0.05 while the knee stayed pinned. Most of the
command comes from the homeokinesis branch (`hk_share` 0.7–0.9), and that leg has the highest forward-model error
(`tle_leg` 0.2–0.3).

**Two kinematics-blind terms.** The stance reach (`reach_rate` 0.01, `reach_max` 0.6): while a stance foot's load is
under half its own running reference, knee += e and hip2 += 0.5e. The corner integrator: hip2 and knee += off. On a leg
folded under, more knee fold RAISES the foot, so the foot feels less load, the corner reads low, and both terms add
more. This leg's FSR reads in the over-range part of the predicted curve (680 g median planted), so an ordinary stance
can read as light against its inflated reference. **Not confirmed:** `reach_e` was not recorded (the dash's diag
filter dropped `reach_`; now kept, with `hk_` and `kin_`). The chatter itself is the HK branch saturating, which this
fix does not touch.

**The lever.** `kin_route` 1 (needs `press_kin` and `jac_topic`): the reach and the integrator's positive offset go
along the leg's own lift direction, the direction the kinematics-aware press uses (on a folded-under leg it UNFOLDS the
knee). The integrator's negative offset goes through the femur only, and the reach stops growing while the lift gain is
below `press_kin_min`. 0 = byte-identical (`_lz` body log equal on the new build).

**Sim, corridor n = 6 × 9000, `_sg` → `_sgk`:** net_z 6.96 ± 1.05 → 7.34 ± 0.35, straight 0.71 → 0.74, tilt_sd 0.079 →
0.092, falls 1 → 2, dips per lift-off 18.1 → 21.0 %. Leg-samples with knee > 0.3 rad were 0–2 % in both arms: the sim
does not reproduce the tuck. **All three falls in the gate configs are flips at tick 300–360**, just after warm-up,
with knees jumping to ~+1.2 rad on three legs first. The watching control (`_sg0`) and `_lz` never flipped. That points
at the gate's start, to be read before the gate is trusted.

**Also fixed:** AsyncStdout wrote its stdout_slow note at a chunk edge and split one JSON line in run 020712. Notes now
go in only at a line break (stress test: a slow FIFO reader, 20 001 lines, 1 note, 0 corrupt).

### 2026-10-27 — ROBOT (presskin_lz_sgk, 286 s, walking toward a wall): the ultrasonic's false close readings — 16 single-sample drops below the slow approach in the 5 Hz record, 7 with a front toe in front of the sensor (44 % against an 10.9 % base rate) — `BASELINE` (n = 1 run); per-ping range logging added

**What was seen.** The operator watched the camera and the range together: the slow descent of the range (1.4 → ~0.5 m
over the run) is the walk toward the wall; short spikes inside it are errors. The only range record in the run is
prox_memory's `last_d`, sampled at ~5 Hz by brain_diag (the sensor pings at 18.7/s; 65 % of samples valid). A spike is a
valid reading more than 0.15 m below its 2 s running median, under 0.5 m: 16 of them. Three read 0.077 m (218, 219,
255 s), just outside the 0.075 m deadzone. Others read 0.11–0.17 m, and a cluster at 234–239 s reads 0.35–0.49 m against
a 0.6–0.97 m trend.

**Self-occlusion.** The front toes' commanded positions (`leg_jac_from_log`): in the 0.15 s before 7 of the 16 spikes,
a front toe was within 20 mm of the centreline and more than 90 mm ahead of the chassis centre (in front of the
sensor), mostly FR at |x| 5–19 mm. That happens in 10.9 % of all frames (binomial p ≈ 0.002 for 7 of 16). The
0.075 m deadzone was set for both legs occluding (operator, 2026-10-25); one foot swinging across the beam reads
further out. The 234–239 s cluster does not fit occlusion (toes clear of the centreline).

**Instrument.** ogma_host now logs every new ping (`{"kind":"range",...}`, raw metres, validity, deadzoned) into the
run log when the dash runs it. Built, not deployed.

**Proposed** (not built): the body knows where its toes are (`feet_xz`, commanded FK), so a foot in the beam is a
confound it can publish in-band (memory: confounds ride in the channel). Mark a reading no-echo, as the deadzone does,
while a front toe is inside the beam's cone at a distance matching the reading. That replaces the fixed deadzone with
the leg's own geometry; it needs the sensor's mount position and beam width.

### 2026-10-27 — ProxMemory CORROBORATION (`support_n`, `too_close_pings`): a lone false ping is never the obstacle — built, unit-tested, sim: avoidance still acts on real walls — `IN_FLIGHT` (the robot's blips are the test; the sim's sensor has none)

The operator's proposal: the range need not be fast, so let distances accumulate instead of reacting to blips.
ProxMemory already accumulated (every ping a point in the odom frame for `memory_ticks` 150), but each bin took its
NEAREST point. One false 0.077 m ping was therefore the ahead obstacle for the whole 3 s memory, and `too_close` fired
on one live ping. `support_n` k: a bin's range is its k-th nearest point, and a bin with fewer than k reads nothing.
A wall (~19 pings/s on the same spot) is believed within k pings. A toe moves with the body, so its blips scatter in
the odom frame and do not stack. `too_close_pings` k: contact needs k consecutive valid close pings. Both 1 =
byte-identical. Unit tests: a lone 0.077 m hit among 0.6 m wall pings dominates at k = 1 and is ignored at k = 3; a
real obstacle is believed on its 3rd ping; a far ping resets the contact streak. Sim (`_sgkc` = `_sgk` + 3 / 2): arena,
no obstacle reached, 3 seeds identical to `_sgk`. Room gym 3 × 3 m, 4 seeds: walls still read ahead under 0.25 m in
every seed (1–8 % of samples vs 2–5 %), avoidance active 5–37 % vs 16–21 %, resets 3 vs 4. The sim's beam has no
blips, so this shows the reflex still works, not that it is better. **Instrument:** `cpp_core/bench/prox_replay`
replays a robot run log's pings and odometry through the real module under several settings. It counts ahead-near
ticks while the 2 s ping median is beyond 0.40 m (false) and the first real near-wall tick (latency), so the robot's
setting comes from its own data.

> **Replay of the first robot run with per-ping logging** (run 182543, first 274 s, 5491 pings, 84 % valid, through
> `prox_replay`). Close pings come in runs: 6 single pings under 0.15 m and runs of 2, 3 and 4 (e.g. four at 0.09–0.11 m
> between 0.93 m readings: a toe resting in the beam for ~0.2 s, its points stacked within ~1 cm). False = the module
> says ahead < 0.25 m (or too_close) while the 2 s ping median is beyond 0.40 m:
>
> | support_n / too_close_pings | ahead-near ticks (false) | too_close ticks (false) | first real near-wall tick |
> |---|---|---|---|
> | 1 / 1 (deployed) | 815 (314) | 40 (34) | 10119 |
> | 3 / 3 | 336 (73) | 8 (8) | +10 |
> | 5 / 5 | 188 (21) | 0 | +13 |
> | **6 / 6** | **140 (0)** | **0** | **+15 (0.3 s)** |
> | 8 / 8 | 76 (0) | 0 | +18 |
>
> 6 / 6 removes every false trigger at 0.3 s of first-detection delay. It also shortens the time a real wall reads as
> near ahead (501 → 140 ticks), because six points must fall in the ±5.6° ahead bin. `_sgkc` set to 6 / 6. Chosen
> in-sample: the next run checks it.

### 2026-10-27 — ROBOT (presskin_lz_sgk, 386 s): the robot ended up moving BACKWARD because one leg went into saturated chatter — FR's hip1 clipped in 6 → 43 → 72 → 82 % of ticks from 120 s, its hip2 in 91 %, its stance stroke reversed (toe +5–6 cm/s forward while planted from 260 s); the HK controller's learning is the driver — sim 24 000 ticks: learning on, one leg slides to 17 % clip and stays; off, 0 % — `HK learning off` (`_sgk_hk0`) `IN_FLIGHT`

**Robot.** Dead-reckoned progress per 20 s is positive to ~300 s (up to 96 cm), then negative (−43, −13, −42 cm). The
toes' commanded fore-aft speed while planted (negative = propulsive), summed over the legs, is −4 to −11 cm/s to 240 s
and ~0 to +5 after. FR alone reverses (stance +5 to +6.4 cm/s, swing −10 to −15), while FL and RL shrink to ~0 (their
hip1 commands within ±0.1 late in the run) and RR keeps pushing. FR is not folded under (knee −0.6, lift gains
positive) and its phase agrees with its contact as often as the other legs do. Its commands are the previous run's
RL failure: hip1 saturated in 6 % of ticks (80 s), then 43 % (120 s), 72, 78, 82 %; hip2 20 → 91 %. Over the run,
`clip_duty` rose 0.010 → 0.104 and `hk_share` 0.41 → 0.69. The avoid reflex was off for most of it. `kin_route` held:
no leg folded under (the reach stayed ≤ 0.27), but it does not touch this.

**Sim, the slowest-integrator horizon** (24 000 ticks, 4 seeds, corridor 0.3, ultrasonic off; memory: windups
appeared after 12 000 ticks). `_sgk`: seed 3's FL hip1 clip per 3000-tick window goes 1 → 10 → 17 % from ~9000 ticks
and stays (hk_share 0.45, clip_duty 0.045 over the second half); seed 2's RR 2–4 %; 3 resets. `_sgk_hk0` (`ctrl_lr` 0,
`bias_lr` 0: the HK controller frozen at its start, output still present, hk_share ~0.10): every leg ≤ 1 % in every
window, clip_duty 0.000, 0 resets. net_z ties (9.3–10.0 vs 8.3–10.1), but the corridor caps it near 10 m, so distance
cannot separate the arms. This agrees with 2026-08's NULL ("all controller learning off costs nothing").

**Instrument.** MotorEPMv2 diag `hk_cnorm` / `hk_hnorm` (per-leg |C|, |h|; the dash keeps `hk_`) to see on the robot
whether a leg's controller grows before it saturates. **Next:** the robot with `_sgk_hk0`, one change from the last run.
The range corroboration (6 / 6, `_sgkc`) is evaluated offline on any run's logged pings, so it can wait.

### 2026-10-27 — ROBOT (presskin_lz_sgk_hk0, 336 s, one operator pause): HK controller learning OFF — no leg saturated (0 % of ticks on every leg, 5–7 % on all four only during one avoid turn), every leg pushes in stance, `hk_share` 0.13–0.15 flat — `WORKING` (n = 1 run; operator: HK off is the right choice for the real robot now); the remaining problem is PACE — path speed steady, net progress not

**Saturation.** Per 2000-tick window, % of ticks with any joint of a leg at |u| ≥ 0.99: 0 % on all legs except ticks
8000–10000 (5 / 7 / 6 / 6 %, all four legs together, while the avoid reflex turned at 166–189 s). The `_sgk` run
before it reached 82 % on FR. hip1 ranges stay ±0.2–0.4 on every leg. `clip_duty` ≤ 0.004, `hk_share` 0.13–0.15
for the whole run (0.41 → 0.69 with learning on).

**Propulsion.** Stance toe fore-aft speed, summed over the legs, −5 to −10 cm/s (propulsive) in every 30 s window
(the `_sgk` run reversed after 260 s). RR and RL push hardest (−2 to −5.5 each), FL −1 to −2.7. FR is weakest
(−1.2 to +1.0; +8.5 in the last window).

**Pace — the open problem.** Dead-reckoned path per 30 s is fairly steady (144–227 cm, 362 in the last window), but
net forward progress along the heading swings from −25 to +182 cm, because the heading wanders (summed |Δyaw|
60–306° per 30 s). The per-sample stride velocity is oscillation-dominated (sd ~11 cm/s around a mean of −2 to +3
cm/s): a blind metric for pace. Coordination is loose: touchdown-phase PLV 0.15–0.35, inter-leg PLV 0.25–0.45. The
support gate holds ~65 % of lift-offs (70–107 holds of 114–155 lift-offs per minute), and 26–51 % of lift-offs are
followed by a belly dip. The LPG period stays at 50 and the level-gated stroke at 0.4–0.65.

**Range corroboration out of sample** (this run's 6636 pings, `prox_replay`): 6 / 6 cut false near-ahead ticks 198
→ 56 (not 0) and false too_close 6 → 0. But the first near-ahead on a real approach moved from tick 8552 to 9225
(13 s), possibly a missed approach. The ping-median "truth" is crude. `_sgkc` stays OFF the robot: a better
criterion (the camera, or the toes' geometry) is needed before choosing a setting.

**Housekeeping.** The 210–248 s pause was the operator (SPACE: a foot about to stick). The events files of the last
two runs end in zero bytes: the dash wrote its "SAFE TO POWER OFF" line after the flush, so that one line was lost at
power-off (fixed: a second sync after the message; not deployed). The per-leg `hk_cnorm` / `hk_hnorm` went into the
full diag, not the snapshot brain_diag records (fixed; not deployed). After a boot the Pi answers on 10.0.0.114 before
`picrawler.local` resolves.

### 2026-10-27 — ROBOT INSTRUMENTS for the stride question: the operator's mark key (`g` good / `w` swim into events.jsonl on the feed's clock) and the raw IMU per tick in the host log; FIRST READINGS: the stride odometry over-reads 2–3× (sonar), the heading hold works but its REFERENCE walks on one-ping avoid episodes, and the support gate's allowance winds to its floor — `BASELINE` (n = 4 runs of the 27th)

**The heading reference.** With the avoid loop silent the hold keeps the heading within 2° over 30 s (run 185039, first
window). The wander is the reference: every activation of the avoid loop sets it to "yaw + away-bearing", and when the
loop falls silent 0.2–2 s later the hold turns the body to it. Per run, the episodes under 2.4 s moved the reference
134 / 338 / 354 / 318° (runs 020712, 164309, 182531, 185039); about half of such single pings are a front toe in the beam
(entry of 2026-10-27, the range false readings). Lever A built the next day (`ref_attack_s`, below).
**The odometry.** Where a surface was ahead and the heading steady, the stride odometry read 3–7 cm/s while the sonar
closed at 1.2–2.4 cm/s: the stance feet slip and the brain's progress sense cannot see it (it assumes a loaded foot
holds). "Path speed steady" in the resume note of the 27th was this blind metric. The commit boost (`progress_commit_gain`
1.0, +30 % thrust after 3.6 s above 3 cm/s) sat above its threshold about half of every run on phantom progress.
**The support gate.** In all four runs the learned allowance fell 27 → 5 mm (the floor) while dips per lift-off stayed
0.30–0.38 against a 0.10 target, holds rose to 0.66–0.77 of lift-offs at 12–13 ticks each. The regulated quantity did not
respond; the gate was not controlling the robot's dips (resolved by the contact shares, 2026-10-28 below).
**Marks (runs 215328, 215940; 60 marks).** Nothing the brain records separates "swim" from "good": inter-leg PLV, phase
rate, phase–contact agreement, holds, belly, feet planted, current, contact flicker, IMU variance, sonar closing — all
overlap within noise. The diagonal-pair phase resultant is ~0.4 at all times (the sim's 0.88), and the per-leg touchdown
rates differ by 30–45 % (physical FL 0.77–0.79/s vs RR 0.52–0.55/s): the gait beats between legs with different periods,
and "a few good steps" are the beat's in-phase moments. Sim, room, 6000 ticks, n = 1 each: support gate off lowers the
diagonal resultant 0.88 → 0.58; contact-only shares raise it to 0.95 (PLV 0.68 → 0.76).

### 2026-10-28 — ROBOT: CONTACT-ONLY LOAD SHARES (`_cs`, LoadPhaseGait `load_topic` empty) — diagonal phase lock 0.28–0.40 → 0.51 and 0.89 (sim 0.88), touchdown-interval CV 0.49–0.61 → 0.38, the support gate's dips at target (0.14) with the allowance stable at 25–29 mm, saturation ≤ 0.4 %, 2.0 cm/s toward a wall by sonar (base 0.6–1.4), 95 "good" marks and no "swim" — `WORKING` (n = 2 runs: 163757, 223518); and the PROGRESS→COMMIT thrust off (`_pc0`) — `NULL` on coordination (n = 1, run 164225; the odometry over-read fell to 1.2× from 2–3×, kept for the slip question)

Why contact beats force here: the force shares pass through one shared FSR curve on four feet whose curves differ (one
reads 4× the others and sits beyond the curve's last point 65–74 % of the time; weight conservation across support
patterns: RR planted → 820–890 g, RR lifted → 140–190 g against a 598 g body). The contact flag is the channel the FSR
work validated (one miss in 128 taps). Shares take the values {0, 1/4, 1/3, 1/2} and still coordinate better, as in the
sim. Per-leg rates still differ (0.61–0.85/s) but now as a pair structure. Leg names in brain_diag are the SIM's order,
which is mirrored: sim `fr` = physical FL (ADC A0). Both morning runs of the 28th (113159 `_pc0`, 113459 `robot_p1`) had
the physical FR FSR (A1) severed: 0–4 counts for the whole run. They are one-foot-blind lesion references, not A/Bs
(the gait was visibly broken; the operator repaired the wire and repeated both). `robot_p1` repeated (222733, 342 s):
PLV 0.52 but diagonal lock 0.33, swing feet touching 0.32 of samples (the highest of any run), one front foot cycling
at a third of the other's rate, a tilt-guard stop at 65 s after the body pitched ~43°, 1.4 cm/s toward the wall.
Re-use context for the force shares: per-foot calibrated curves (weight conservation on the logs is the method), or a
contact belief (the generator-and-belief-filter note, G2) in place of the raw flag.

### 2026-10-28 — ROBOT `_cs` CIRCLES RIGHT at 4–6°/s (both runs): the heading REFERENCE, not the legs — the avoid loop read an obstacle on one side in 84–94 % of its active ticks and steered away, the hold finished each turn while silent (−2.4 to −3.8°/s idle, error −0.1 to −0.24 rad), and the left-side stance push 40–55 % larger than the right IS the hold's steer differential (`_sgk_hk0` and `robot_p1` push evenly with the same hold) — mechanism `BASELINE`; LEVER A built: BearingToIntent `ref_attack_s` (`_cs_ra`)

The loop that sustains it is geometric: proximity memory keeps points in the odometry frame, a right turn rotates every
remembered point to the body's left, "obstacle left" commands right again. Whether the first point was a wall or a toe
blip the video will say; the sonar's valid-ping rate in run 223518 was the lowest recorded (48 %). Why `_cs` and not the
base: the better-coordinated gait turns decisively when told to, so the feedback closes. Lever A: while a loop is active
the reference moves toward yaw + bearing with a time constant (2 s) instead of jumping, so a one-ping blip moves it a few
degrees and a wall episode moves it fully; the bearing published during the episode is unchanged. Gain-0: `_cs` and
`_cs` with `ref_attack_s` 0 explicit are byte-identical over 3000 ticks in the room with the sonar on (deterministic, 0
differing lines). Unit tests: a one-tick blip moves the reference 1 %, 400 ticks move it 97 %; 0 = the jump.
