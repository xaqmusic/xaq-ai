# Launcher arms archived 2026-08-07

The launcher dropdown was trimmed to three entries (arena control, arena IK-plumb,
and the current stroke12 stack). **Nothing was deleted from the repo** — the
declaration's own note still holds: *"every config stays loadable by path for
scripts + A/B runs"*, so each JSON below is still runnable via
`OGMA_PICRAWLER_CONFIG=res://addons/ami_ogma/configs/<name>`.

What is preserved here is the **annotation**, which in several cases is the only
prose record of what an arm tested and what to look for. Verdicts themselves live in
[`../picrawler_lever_ledger.md`](../picrawler_lever_ledger.md); this file exists so
the *watch-for-this* guidance is not lost with the dropdown entry.

## `the_picrawler_motor_epm_embed_corridor_stancelift.json`

*2026-07-25 — DEMO-CLEAN: narrowed to the current config + one baseline, so the*

*dropdown is presentable and an A/B is unambiguous.  Everything else stays*

*loadable by path (scripts, seedavg/humpavg/recoveravg arms) and in git —*

*nothing is deleted.  To restore one to the dropdown, uncomment it.*

── BASELINE ── the deployed stack: heading P+7 · progress-commit · Markov belly-rangefinder height · stance_lift 0.5 (belly-up). Clears the hump; net_z 3.62, straight 0.67, tilt_sd 0.078, 0 falls.

---

## `the_picrawler_motor_epm_embed_corridor_imufused.json`

── CURRENT ── the fully hardware-honest stack (commanded-angle IK ⊕ gyro-fused IMU attitude). net_z 4.75, flat_v 0.05, straight 0.74, tilt_sd 0.068, hump 6.09, 0 falls. KNOWN FLAW, documented and unfixed: an inverted episode latches height_bias and amp_gain, and the walk stays degraded for ~200 s after self-righting (ledger §2).

---

## `the_picrawler_motor_epm_embed_corridor_uprightgate.json`

"the_picrawler_motor_epm_embed_corridor_uprightgate.json",  # UNRESOLVED TRADE — the upright gate stops the latch (amp_gain 2.53→0.100, height_bias +1.50→−0.09, normal walking byte-identical) BUT self-righted 0/8 vs ungated 1/8. amp_gain winding 40× IS the flail amplitude that rights the robot, so freezing it may disable the escape. Do not promote until resolved.

---

## `the_picrawler_motor_epm_embed_corridor_gravcmd.json`

"the_picrawler_motor_epm_embed_corridor_gravcmd.json",  # same but EXACT sim attitude — uncomment to A/B what the honest IMU model costs (it gains). HARDWARE-HONEST: feet_topic = FK from COMMANDED servo angles ⊕ accelerometer gravity-up. Buildable on the real PiCrawler (hobby servos report no position, so commanded angles are all it has). Servo deflection measured at 22mm mean/38mm max; FK validated to 1.1mm. THE HARDWARE'S POORER INFORMATION IS THE BETTER SIGNAL: net_z 3.76 oracle → 3.95 achieved → 4.36 commanded (±0.28); straight 0.70 → 0.62 → 0.74 (±0.01); hump final_z 5.21 → 5.35 → 6.10; 0 falls. Deflection is noise to the gate. STILL UNMODELLED: attitude uses the exact sim transform, not gyro/accel fusion.

---

## `the_picrawler_motor_epm_embed_corridor_gravfeet.json`

"the_picrawler_motor_epm_embed_corridor_gravfeet.json",  # same but FK from ACHIEVED pose (sim-privileged) — uncomment to A/B the sim-to-real gap. ORACLE REMOVED. feet_topic = feet_y_gravity (encoder-FK foot position · accelerometer gravity-up = IK ⊕ IMU) replacing feet_y, which was absolute WORLD-Y and unsensable by a real robot. n=4: net_z 3.76→3.95 and flat_v 0.04→0.05 — the FIRST flat-speed movement across 10+ levers; step_bal 0.25→0.42, steps 74→113. Hump gate improves (5.21→5.35, 0 falls). Costs straightness (0.70→0.62) and wobble (0.067→0.095) — livelier, less controlled. Three other legal signals lost badly and two phase hypotheses died at chance; gravity reference was the missing property.

---

## `the_picrawler_motor_epm_embed_corridor_rewardfree.json`

"the_picrawler_motor_epm_embed_corridor_rewardfree.json",  # the ORACLE arm — identical but feet_topic=feet_y (absolute world-Y). Uncomment to A/B the legitimacy question directly. baseline + coord_adapt 0.001 + the coordination search made REWARD-FREE (coord_fitness_mode=1): probes ranked by coherence·activity/(1+tle) instead of forward velocity. No distance/velocity term anywhere. vs the fwd_v-reward arm: net_z 3.52→3.76, straight 0.67→0.70, tilt_sd 0.069→0.067, hump gate IDENTICAL (5.21), and coordination RECOVERS BETTER after perturbation (0.30→0.48) — a thrash can't win, it has high tle and low coherence. Removing the reward cost nothing; removing the search entirely costs net_z→2.36.

---

## `the_picrawler_motor_epm_arena_short_stride.json`

── ARENA · SHORTER STRIDE ── stroke_gain 1.2 (deployed 1.65) — the "propulsion amplitude / stroke operating point" the ledger named and nobody had swept. A SHORTER stroke walks FURTHER: net_disp 4.85→5.45 (+12%), straight 0.71→0.78, BELLY PRESERVED, 0 falls; sweep peaks at 1.2. Corridor is a TIE on progress but step_bal 0.44→0.52; hump gate HOLDS. PARTIAL, not promoted — not LOUD enough on metrics alone, hence the eyeball. NOTE it does NOT move the feet inboard (that mechanism is refuted: hip1's axis is world +Z, a YAW joint that sweeps at CONSTANT radius, so foot_r is invariant across a 1.8× sweep). Live slider `stroke_gain` on [M].

---

## `motor_epm_pure_hk__inst__stance.json`

*--- 2026-08-02 PURE-HK OBSERVATION PAIR (corridor gym; [1]/[2] swap gyms live) ---*

*THE CONTEXT, which is why these two are worth eyeballing at all: ablating EVERY*

*learned component of MotorEPM costs the deployed gait nothing (net_z 4.75 ± 0.48*

*vs baseline 4.58 ± 0.27, n=4).  Locomotion in the CURRENT entry above is the*

*hand-built scaffold stack, not the brain — so every lever this campaign has judged*

*was measured in a context where the mechanism has no authority.  These two arms*

*strip the scaffolds so the homeokinetic controller is the ONLY thing driving the*

*legs.  Load the CONTROL first.  Full record: docs/reports/playful_machine_source_analysis.md*

── PURE-HK CONTROL ── homeokinetic core + the postural stance reflex ONLY. No stroke, no coupling, no gait phase, no heading, no height homeostat, no nav. n=4: net_z +0.13, chassis_y 0.029, steps 0.5, tilt_sd 0.044. Expect a low crouch that barely moves — that is the point of the comparison.

---

## `motor_epm_pure_hk__inst__stance__c025.json`

── PURE-HK · SELF-EXCITE ── the control + ONE parameter: c_init=0.25, the Playful Machine's self-exciting controller init (Sox cInit; ≈ their 0.75 at our motor_gain=3.0). Starts the loop at the edge of instability so HK SHAPES an oscillation instead of manufacturing one. n=4: steps 0.5→21, chassis 0.029→0.040, tilt_sd 0.119, planted 3.89, unstable 0.00, net_z +0.23 (deployed 4.58). ★ THE QUESTION METRICS CANNOT ANSWER: is this a body TRYING TO WALK or twitching in place? Every leg motion here is the learned controller — nothing scripts a gait. §3 rule 5 promote-or-kill gate.

---

## `motor_epm_pure_hk__inst__stance__c025__lr10.json`

── PURE-HK · SELF-EXCITE + PM LEARNING RATE ── THE ARM UNDER TEST (2026-08-03). Same stripped base as the pair above plus PM's hexapod learning rate: c_init=0.25 AND ctrl_lr=0.10 (we had been running 0.01, 5-10× below every legged experiment of theirs). n=4 @20k: steps 41→163, net_disp 0.56→1.79, one seed net_z 4.36 (deployed 4.58) with NO stroke/coupling/gait-phase/heading/nav. ★ WATCH OVER MINUTES, NOT SECONDS: activity peaks ~14-20k ticks then DECAYS (steps/1000: 12.0→5.2→2.95 by 40k). Expect a crab walk that travels but does not hold a heading (turns ±12) — direction is the ingredient this arm lacks. `ctrl_lr` is live on [M] (ceiling raised 0.10→0.30; the old max sat at the BOTTOM of the useful band); `c_init` is construction-only, restart to change. `[`/`]` sweep body damping live — a DIAGNOSTIC only, see the note at the hotkey.

---

## `motor_epm_pure_hk__inst__stance__c025__lr10__wb.json`

── PURE-HK · WHOLE-BODY C ── ONE controller across all 12 joints (whole_body_c=1) instead of four per-leg blocks, so inter-leg coordination is a learnable entry of C. Still no stroke/coupling/gait-phase/heading/nav. ★ WHAT IT BUYS, powered at n=12: LEG PARTICIPATION — step_bal 0.25→0.44 (t=+3.52). The legs share the work far more evenly. ★ WHAT IT DOES NOT BUY: phase coordination. Time-mean inter-leg coherence is 0.453 against a random-phase null of 0.450 — i.e. the legs are at RANDOM relative phase, in this and every other arm tested including the deployed stack. An earlier label here listed "locked seeds"; that was RETRACTED — it sampled an instantaneous statistic whose null is 0.450 ± 0.219, and the operator caught it by eye. Expect vigorous, powerful, uncoordinated legs. Costs stability: ~4 falls per 20k.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__steplock_off__nolearn2.json`

*--- 2026-08-04 INFERENTIAL-COUPLING OBSERVATION SET (corridor gym) ---*

*Precision-weighting the Kuramoto neighbour average.  The deployed term averages the*

*other three legs UNIFORMLY (c/(n-1)) — a hand-set precision: every leg trusted equally,*

*forever, which is what makes an otherwise legitimate INNATE reflex a script.  These arms*

*replace it with w_j = (amp_j/(tle_j+eps))^k, the LateralVoter's idiom one layer down.*

*★ THESE ARE THE ARMS TO ABLATE FROM THE [K] PANEL: the whole result is a DAMAGE*

*response — on an intact body the lever is NULL, so nothing here is visible until you*

*break a leg.  Kill leg 0's knee or detach a segment and compare the CONTROL against k=+2.*

*Base is `nolearn2` (all controller learning off — the n=20 ablation WINNER), so the*

*precision weighting is the only inferential thing left in the controller.*

── (d) CONTROL ── couple_prec_gain=0, the legacy UNIFORM neighbour mean. Byte-identical to the nolearn2 base, verified per-seed. Healthy: net_z 4.83 ± 0.41, plv_w 0.20. ★ AFTER A LEG DIES it loses a third of its coordination and keeps falling (plv_w 0.203 → 0.127) and forward progress collapses to 0.011 — the dead leg's frozen, meaningless phase still commands 1/3 of every survivor's coupling authority and drags them down with it. LOAD THIS FIRST.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__steplock_off__nolearn2__cp20.json`

── (d) ARM · k=+2 ── same body, ONE param: couple_prec_gain=2. Each leg now pulls toward its neighbours in proportion to how well each of THEM predicts itself. Healthy: TIES on distance, slightly WORSE coordination (plv_w 0.18) — the lever costs a little on an intact body. ★ AFTER A LEG DIES coordination ends ABOVE where it started (0.169 → 0.190, t = 5.84 vs control) and progress holds at ~2×. ⚠ READ IT HONESTLY: most of that gap is SUPPORT (the survivors keep oscillating, +37%), only +10% is tighter phase-locking. The claim is resilience, not coordination.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__steplock_off__nolearn2__cpm10.json`

── (d) WRONG-SIGN CONTROL ── couple_prec_gain=−1: trust the leg that predicts itself WORST. THE ARM THAT MAKES THE RESULT DEFENSIBLE — it carries a COMPARABLE weight spread to k=+2 (cw_spr 0.92 vs 0.98) yet performs like the plain control after a lesion (0.121, t = −0.43) with the worst locking-given-oscillation of any arm. So the effect is weights in the right DIRECTION, not merely unequal weights stiffening the system. Expect it to look like the control, not like k=+2.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__cp20.json`

── (d) ARM ON THE DEPLOYED ROBOT ── the CURRENT entry above + couple_prec_gain=2, so the lever can be seen on the stack that actually ships rather than only on the stripped research base. ⚠ IT BEHAVES WORSE HERE, and honestly so: n=4 unperturbed vs the CURRENT baseline — net_z 4.72 ± 1.01 (ties, but 4× the variance), straight 0.69 vs 0.73, plv_w 0.16 vs 0.20 (WORSE coordination), tilt_sd 0.065 → 0.116 with one seed at 0.210, steps 50 → 85. 0 falls. Consistent with the standing result that the scaffold stack was hand-tuned around a weak learned signal, so anything that strengthens the learned path moves away from that tuning. Load it to ABLATE from [K], not to admire the walk.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12.json`

*--- 2026-08-05 STRAIGHT-GAIT OBSERVATION SET (corridor; [P] shows the path trail) ---*

*★ THE FINDING THESE EXIST TO TEST: this body has NO fixed handedness. The right/left*

*hip1 demand ratio across 4 seeds is 3.07 / 4.35 / 0.69 / 0.97 — SEED 3 IS LEFT-HEAVY —*

*and it still flips with every controller learning rate at zero. So the skid direction*

*comes from the 0.01*random init of C, locked in by the gait as a stable attractor, and*

*the heading PD fights it all run. That standing effort is what pins one side near its*

*clamp (right legs request 2.10 against a +-1 limit, 77% clip duty, vs 0.70 left) and is*

*why turn authority is DIRECTION-DEPENDENT. Load the CURRENT entry first as the control.*

*⚠ ALL FOUR improved the asymmetry and NONE improved `straight` — which is the open*

*question these need eyes on. n=4, so signal only.*

_(no annotation)_

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__gng__bellyset.json`

── ★ BELLY-GROUNDING HEIGHT SETPOINT (gain 0.01, swept) ── MEASURED 2026-08-06: the height homeostat target (`height_k` 0.3 x discovered max) sits BELOW where the body rides (h_ema 0.39-0.44), so `height_bias` integrates NEGATIVE (-0.30..-0.47) and commands hip2 DOWN while the belly is grounding (54% of the first 500 ticks under 10mm). hip2 — the chassis elevator — is 4.5x under-driven as a result and the KNEE carries support from a permanently flexed low-leverage posture. This lets the setpoint RISE while the belly grounds and decay back otherwise: discovered from contact, not asserted. CORRIDOR n=6, SOLID chassis: %<10mm 11.4->5.3 (t=-3.95), gc p1 +46% (t=+1.91), net_z UNCHANGED (t=-0.14), tilt_sd unchanged (0.076). ⚠⚠ EVALUATE ONLY WITH `OGMA_PICRAWLER_CHASSIS_COLLIDE=1` — seedavg.py does NOT set it, and judging a belly lever on a ghost chassis (which cannot touch the ground with its belly) is meaningless; doing so is what made this arm first look like a REGRESSION. ★ WHAT TO READ: belly-clearance PERCENTILES (never the mean, which reads "fine" at 22mm while 60% of early ticks are under 10mm), net_z, AND tilt_sd — gain 0.02 nearly doubles tilt_sd and 0.08 trebles it, which is how you buy clearance by wobbling.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__gng.json`

── ★ RUNG 2 STEP 1 — THE MOTOR LAYER FINALLY HAS A REAL EPM ── four EPM modules (one per leg, RBF encoder, dual TLE α=0.7/β=0.3, mitosis on) on `reality.motor_leg.*`. Until now the motor layer had NO GNG at all: no node vocabulary, no edges, no transition surprise, and a SYNTHETIC gng payload built purely to feed the inspector widget (MotorEPM.cpp:4564, `edges` always empty). ⚠ NOTHING CONSUMES THE TOKENS YET — deliberately. This arm exists ONLY to answer the §0 rule-2 conditioning question BEFORE any behavioural claim: does the GNG EARN a vocabulary, or does the insertion gate collapse the whole stride manifold onto one node while the encoder still shows the structure? ★ WHAT TO READ: the EPM panels for motor_gng_fl/fr/rl/rr — `nodes`, `baked`, `tle`, mitosis. Never growing, never baking, or growing unbounded is a CONDITIONING diagnosis, not a verdict on the idea. If node count says "one thing" and the PCA scatter says "several", believe the scatter. Rung 1 (act on the prediction) was NULL because a one-step linear model is too shallow to act on; this is the prerequisite that null exposed.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__sm2.json`

── ⚠ RETRACTED — A REGRESSION, KEPT AS THE CAUTIONARY ARM ── phase_vel_smooth=2. It WAS promoted on +21.7 % "disp/s" (t=+5.60, 6/6 seeds, replicated out-of-sample at +17.0 %). That metric was PATH LENGTH per second, which counts backward motion as progress. On net displacement it is a 57 % LOSS (4.51 m → 1.91 m), straightness collapses 0.356 → 0.121 and mean fwd_v halves (+0.050 → +0.023). The operator caught it by eye: "there is as much backward motion as there is forward motion ... it looks like an alternating current signal". Load it to SEE the failure mode — bigger, more energetic leg pulses that cancel out. The real defect it exposes: an EMA on only the velocity arm of atan2 distorts the phase non-uniformly, and lag on the reference that TIMES THE POWER STROKE makes the stroke push backward as often as forward.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__cprec25.json`

── STROKE 1.20 · COMMIT AS EARNED PRECISION ── commit_prec_gain=2.5. Commit is a PRECISION ("how much do I trust my current motion vs keep searching"), and doctrine §2.3 says precision is a CONTROLLED variable — "a designer picking the crossover point is the anti-pattern". Three hand-picked crossover points were measured first: the original 180/240/90 = 14 % stalled, commit OFF = 20 %, inverted fast-engage/slow-release = 22 %. The hand-tuned original won, which is exactly when the constant should be replaced by the mechanism that ought to set it. Now the window/ramp/release scale with how well the body predicts ITSELF (the forward-model residual against its OWN running mean — scale-free, nothing tuned to tle's magnitude). ★ Commit can now engage INSIDE a 1–2 s burst if that burst is genuinely predictable; a fixed 180-tick (3 s) window structurally cannot, and that timescale mismatch is the operator's complaint. n=1 seed 6: stalled 14 % → 11 % (the lowest measured), disp/s and metres flat — it buys CONTINUITY, not speed. ⚠ NON-MONOTONE: gain 1.0 was WORSE (19 %), so 2.5 may be luck; a proper sweep is owed. THE QUESTION FOR THE EYE: during a pause, does it look like stroke12 ("trying to move forward, better steps once moving") or like nocommit ("simply shaking")? The 3-point stall metric has been blind to that distinction twice today.

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__nocommit.json`

_(no annotation)_

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__intent.json`

_(no annotation)_

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__intentd.json`

_(no annotation)_

---

## `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__intentg.json`

── STROKE 1.20 · INTENT · PROGRESS OVER GROUND ONLY ── intent_yaw_gain 1.0 → 0. The operator's separation-of-concerns argument, 2026-08-05: "chassis yaw should not be part of this function since the chassis oscillates constantly while stepping. If heading needs to be adjusted that must come from some other mechanism that affects the bilateral stride symmetry, and not the direction the chassis happens to be facing. The progress over ground relative to the CoG is all that matters." A quadruped yaws every stride BY CONSTRUCTION, so penalising instantaneous chassis yaw asks the body to stop doing the thing that moves it — gait mechanics leaking into a goal-achievement scalar, which is also why the error spiked on VALID steps. Heading is not abandoned: it keeps its own correct channel (goal_bearing_topic → heading PD → steer_eff → the bilateral stroke-amplitude differential), which IS the bilateral-symmetry mechanism named above and is already built. ★ FOR THE EYE: the spikes that coincide with good steps should be the ones that disappear. ⚠ Read behaviour, not the delta — same-exponent replication moved every seed 6–13%, so n=4 cannot resolve ~5% on any arm that engages commit_prec.  # ── STROKE 1.20 · INTENT · DERIVED EXPONENT ── the operator's "this seems like a lot of tuning", answered by deleting the constant rather than picking a better one. `z = (err_run - e)/err_run` is bounded ABOVE at exactly 1 (e >= 0), so the exponent is NOT free: z's own ceiling must map onto the output ceiling, which fixes k = ln(clamp_hi) = 1.609. The clamps are reciprocal (5.0 / 0.2), so the map is symmetric in log space — z=+1 -> 5.0, z=0 -> 1.0 (byte-identical), z=-1 -> 0.2. Nothing left to choose. THE DEFECT IT REPAIRS: at the old hand-picked 2.5 the 5.0 clamp was reached at z = 0.64, collapsing the top 36 % of the signal's range onto one value — measured 22 % of ticks pinned at cp = 5.0. Saturation is not a stronger signal, it is a DESTROYED one: inside the rail the mechanism cannot tell "just achieving" from "achieving spectacularly". ★ THE QUESTION FOR THE EYE: __intent (2.5) should read as OVER-committed — locking into a stride and not releasing when it should. This arm should hold the same commitment but let go on time. If both look identical, the exponent was never the binding constraint and the railing is a symptom of something upstream.  # ── STROKE 1.20 · INTENT-RELATIVE COMMIT ── the operator's ask: make confidence depend on what we WANT the body doing now, not on a self-prediction residual (which measures +0.129 against displacement — a moving body is LESS predictable, so the residual anti-indicates competence). commit_prec now descends the error between (fwd_progress_ema, yaw_rate_ema) and a declared (v*, w*) intent, giving a higher loop a real lever. ⚠ THE FIRST BUILD WAS BROKEN AND THE FIX IS WHAT YOU ARE LOADING: summing the two error terms raw made intent_err 98 % YAW variance (|ew| 0.151 vs |ev| 0.022 — the forward term is bounded by v*, yaw is not), so corr(intent_err, fwd_progress_ema) = -0.002: the scalar carried NO information about forward progress, and since skid-steer locomotion IS yaw the error rose exactly when the body walked. Each term is now normalized by its own running spread first (doctrine §5). n=4 arena: disp/s 0.1685 vs 0.1680, stalled 0.3 % vs 0.4 % — it TIES. Honest, cheap, buys nothing yet. ★ THE QUESTION FOR THE EYE, which the logs cannot answer: through a pause, does `fwd-prog-ema` (thick green, MOTOR-EPM panel) drop BEFORE `commit-prec` rises, or after? Progress-first => commit is a symptom. Commit-first => it is causal, and the repair is different. corr(commit_prec, displacement) is ~0 in every build tried, so the term you can plainly see moving with the pause/step cycle is not tracking displacement — that contradiction is the open question.  # ── STROKE 1.20 · COMMIT OFF ── progress_commit_gain 1.0 → 0. THE INTERVENTION THAT REFUTED A GOOD STORY: the operator spotted that exploration amplitude swings in antiphase with movement, and it does — explore_mult oscillates 0.00↔1.00 with displacement leading it by ~4 s (r = −0.34, consistent across arms), a textbook delayed-negative-feedback loop. Setting commit to 0 pins explore_mult at 1.00 and removes the oscillator cleanly. ★ AND STALLING GOT WORSE: stalled-seconds 14 % → 20 %, disp/s 0.073 → 0.067, 29 → 28 m. So the loop is REAL but is NOT what gates the steps, and permanent full exploration noise stalls MORE. Load it beside stroke12 and ask whether it is more stalling or DIFFERENT stalling — the metric cannot tell those apart. Incidentally this is also the best evidence progress_commit has ever had (the ledger has it as a "marginal keeper" on +2.55→+2.62 net_z).

---

## `the_picrawler_motor_epm_embed_corridor_imufused__pairinit.json`

── PAIRED L/R INIT ── acts on the CAUSE: left/right partner legs (0&1, 2&3) get the SAME initial control law, so no side starts with an advantage. asymmetry 1.44→0.78, |turns| 0.138→0.077. ⚠ THE RISK IS VISUAL, NOT NUMERIC: the MotorEPM class header warns the per-leg random init is the "inter-leg symmetry breaker" guarding against v6-premotor-bilateral-mirror-collapse. WATCH FOR: left and right legs moving in LOCKSTEP (collapse — antiphase lost) versus mirrored-but-independent (fine). Metrics cannot see this; eyes can.

---

## `the_picrawler_motor_epm_embed_corridor_coordadapt.json`

*--- everything below is parked, not deleted ---*

"the_picrawler_motor_epm_embed_corridor_coordadapt.json",  # the fwd_v-REWARD comparison arm for the above (coord_fitness_mode=0). Uncomment to A/B the reward question directly in the UI.

---

## `the_picrawler_motor_epm_minimal.json`

"the_picrawler_motor_epm_minimal.json",   # L-1a Gate 0 — reward-free upright prior + HK loop (additive baseline)

---

## `the_picrawler_motor_epm_objposture.json`

"the_picrawler_motor_epm_objposture.json",# L-1b — postural via the objective.posture socket (PosturalPrior), postural_gain=0

---

## `the_picrawler_motor_epm_keyframe.json`

"the_picrawler_motor_epm_keyframe.json",  # L-1b — KeyframeGait phase-indexed keyframe on the socket + dialed-down postural (0.3)

---

## `the_picrawler_motor_epm_embed.json`

"the_picrawler_motor_epm_embed.json",     # L-1b — the embed MILESTONE gait (arena): keyframe + CPG-EMBED

---

## `the_picrawler_motor_epm_embed_corridor.json`

"the_picrawler_motor_epm_embed_corridor.json",  # embed milestone in the CORRIDOR trench gym

---

## `the_picrawler_motor_epm_embed_corridor_propbal.json`

"the_picrawler_motor_epm_embed_corridor_propbal.json",  # SEED-REFUTED (2026-07-23): noise + anti-composes

---

## `the_picrawler_motor_epm_embed_corridor_bearinghold.json`

"the_picrawler_motor_epm_embed_corridor_bearinghold.json",  # the belly-DRAG predecessor to stance-lift (clearance dips to 0.003 = scraping)

---

## `the_picrawler_motor_epm_embed_corridor_explore.json`

"the_picrawler_motor_epm_embed_corridor_explore.json",  # heading-hold + stuck→explore 2.0 (seed-refuted as net-neutral)

---

## `the_picrawler_motor_epm_velobj.json`

"the_picrawler_motor_epm_velobj.json",  # REJECTED: phase-indexed VELOCITY objective (Cvel pump) — amplified a gait asymmetry into circling

---

## `the_picrawler_motor_epm_velsym.json`

"the_picrawler_motor_epm_velsym.json",  # REJECTED: velobj + LR-symmetry prior — the fix BACKFIRED (worse circling)

---

## `the_picrawler_motor_epm_hip2learn.json`

"the_picrawler_motor_epm_hip2learn.json",  # REJECTED: learned hip2 — no gain + more instability

---

## `the_picrawler_motor_epm_cpgwalk.json`

"the_picrawler_motor_epm_cpgwalk.json",  # REJECTED: coherent but an open-loop sequencer — chassis slams the ground (flopping fish)

---

## `the_picrawler_motor_epm_energetic.json`

"the_picrawler_motor_epm_energetic.json", # EXPERIMENTAL high-energy variant

---

## `the_picrawler_motor_epm_rung0.json`

"the_picrawler_motor_epm_rung0.json",

---

## `the_picrawler_motor_epm_vision.json`

"the_picrawler_motor_epm_vision.json",   # rung0 + epm_color (camera→brain, V1 vision)

---

## `the_picrawler_motor_epm_vision_steer.json`

"the_picrawler_motor_epm_vision_steer.json",  # V2: nav steers on VISION (vision_compass)

---

