# Motor-system audit 2026-08-09 — appendix: verbatim reader reports

Three independent extraction passes over the deployed stack, preserved unedited.
Synthesis and dispositions live in motor_system_audit_2026-08-09.md.


---

# Reader A — topic pub/sub graph

All sources read. Complete graph follows.

# PICRAWLER MOTOR SYSTEM — TOPIC PUB/SUB GRAPH
Config: `godot_host/project/addons/ami_ogma/configs/the_picrawler_motor_epm_embed_corridor_imufused__stroke12__gng__bellyset__stancehip2__supportepm.json`
Sources: config JSON; `cpp_core/src/ogma/modules/{JointSensorimotorBridge,MotorEPMv2,CPGOscillator,KeyframeGait,BodyRhythmTracker,EPM}.cpp`; `cpp_core/include/ogma/modules/MotorEPMv2.hpp`; `godot_host/project/scripts/picrawler_body.gd`; `godot_host/src/OgmaBrain.cpp`; `docs/plans-and-designs/sensor_legitimacy_and_the_feet_y_oracle.md`.

Host mapping (OgmaBrain.cpp): `publish_proprio(v, "<name>")` → topic `reality.proprio.<name>` (ProprioToken, producer "host"); `publish_event("<name>", i)` → `events.<name>` (EnvEvent); `publish_video(..., "<m>")` → `host.video.<m>`. Action channels: host `register_action_channel` polls bus last-value of each `action.*` topic every tick → servo PD targets.

## 1. PUBLISHERS

### 1a. Body (picrawler_body.gd, all producer "host")
| Topic | Payload | Condition |
|---|---|---|
| reality.proprio.imu | f32[4] [sin(yaw), cos(yaw), fwd_v/1.0, ang_v/π] — world yaw, world-projected velocities | every tick |
| reality.proprio.joints | f32[12] normalized joint angles, layout hip1[fl,fr,rl,rr], hip2[×4], knee[×4] | every tick |
| reality.proprio.ego_heading | f32[1] dead-reckoned heading (integrated modelled gyro, drifts) | every tick |
| reality.proprio.vel_ego | f32[2] [v_right, v_forward] body frame (world velocity projected — soft oracle) | every tick |
| reality.proprio.lateral_v | f32[1] signed sideways-slip velocity | every tick |
| reality.proprio.compass | f32[2] [sin(yaw), cos(yaw)] world-frame | every tick |
| reality.proprio.radial_compass | f32[2] body-frame outward direction | every tick |
| reality.proprio.target_compass | f32[2] body-frame unit vector to active pyramid target; (0,0) if no target | every tick |
| reality.proprio.target_loom | f32[1] fraction of forward FOV hitting target | every tick |
| reality.proprio.distress | f32[1] wedge severity [0,1] = perch(tilt EMA) × stall(2 s net world-XZ displacement deficit) | every tick (0 during 10 s warmup) |
| reality.proprio.motor_intent | f32[2] [v_fwd*, yaw*] scaffold intent | only if `intent_fwd >= 0`; default −1 → **NOT published** |
| reality.proprio.feet_y | f32[4] per-leg absolute WORLD-Y foot height | every tick |
| reality.proprio.feet_y_body | f32[4] foot height vs chassis (encoder FK) | every tick |
| reality.proprio.feet_y_gravity | f32[4] FK(measured angles)·accelerometer-up | every tick |
| reality.proprio.feet_y_gravity_fk | f32[4] analytic FK validation twin | every tick once `_chassis_rest_xform` captured |
| reality.proprio.feet_y_gravity_cmd | f32[4] FK from COMMANDED angles, exact attitude | same block |
| reality.proprio.imu6 | f32[6] body-frame accel[3]+gyro[3] | same block |
| reality.proprio.feet_y_gravity_cmd_acc | f32[4] commanded FK on accel-only gravity estimate | same block |
| reality.proprio.feet_y_gravity_cmd_imu | f32[4] commanded FK on gyro-fused (complementary-filter) gravity estimate — hardware-realizable | same block |
| reality.proprio.foot_contact | f32[4] true physics touch flag per leg | every tick |
| reality.proprio.ground_clearance | f32[1] belly ToF normalized [0,1] | every tick |
| reality.proprio.feet_y_ground | f32[4] FK + belly ToF, terrain-relative | every tick |
| reality.proprio.upright | f32[1] chassis up·gravity (+1..−1) | every tick |
| reality.proprio.bucket_{fl,fr,rl,rr} | f32[1] stance/swing bucket per leg | every tick |
| reality.proprio.leg_id_{fl,fr,rl,rr} | f32[1] constants 0–3 | every tick |
| reality.proprio.tgt_align_{fl,fr,rl,rr} | f32[1] target-alignment bucket 0–3 | every tick |
| reality.proprio.joint_torque | f32[12] PD command budget (≈ negated velocity copy, see its own caveat) | every tick |
| reality.proprio.joint_load | f32[12] velocity-tracking deficit | every tick |
| reality.proprio.foot_load | f32[4] foot normal force / body weight (FSR analog) | every tick |
| reality.proprio.chassis_y_norm | f32[1] clamp(chassis world-Y / target_height) | every tick |
| reality.proprio.tilt | f32[4] [sin p, cos p, sin r, cos r] | only `publish_tilt` — default false, metadata `publish_tilt:false`, no env override ⇒ **NOT published in this config** |
| reality.proprio.beacon, reality.proprio.vision_compass, host.video.color | vision stream | only `publish_vision`/`vision_steer` — default false ⇒ **NOT published** |
| events.hit / events.miss / events.alive / events.episode_end / events.reset / events.beacon_reached / events.leg_planted_<leg> / events.leg_lifted_<leg> / events.leg_event / events.hit_leg_<leg> | EnvEvent intensity | reward/reset machinery conditions (hit: height-reward duty cycle; miss: tip/sink; reset: teleport/respawn incl. outer-wall auto-reset) |

### 1b. Brain modules
| Topic | Module | Payload | Condition |
|---|---|---|---|
| action.{fl,fr,rl,rr}_{hip1,hip2,knee} (12) | motor_epm (MotorEPMv2) | ActionOut accel per joint | every tick (babble noise first 200 ticks, then control law) |
| reality.motor_leg.{fl,fr,rl,rr} | motor_proprio_bridge (JointSensorimotorBridge) | ProprioToken 9-D per leg: [pos, action(efference copy), delta] × {hip1,hip2,knee}; no load slot (load_topic unset) | every tick on joints arrival; `range_probe_ticks=2000` → BRIDGE_RANGE stdout instrument only |
| rhythm.cpg.body | cpg_clock (CPGOscillator) | ProprioToken [cos φ, sin φ], sensor "cpg_phase" | every tick (motor `output_topics` empty ⇒ no motor emission; amplitude=0 irrelevant to perceptual token) |
| rhythm.body.gait | body_rhythm_tracker (BodyRhythmTracker) | ProprioToken [cos φ_body, sin φ_body, ω rad/tick] | every tick |
| objective.posture.{fl,fr,rl,rr} | keyframe_gait (KeyframeGait) | PredictionToken: 3-D keyframe posture per leg, confidence = gain(0.3) × self-precision | every tick while `publish` (default true); velocity socket empty ⇒ objective.velocity.* NOT published |
| reality.motorgng.{fl,fr,rl,rr} | motor_gng_{fl,fr,rl,rr} (EPM) | RealityToken {winner_id, latent, tle, is_novel} | every tick (bootstrap token pre-bake) |
| reality.support.body | support_epm (EPM) | RealityToken over 4-D foot_load | every tick |

## 2. SUBSCRIBERS
| Topic | Subscriber | Used for | Gate |
|---|---|---|---|
| reality.proprio.joints | JointSensorimotorBridge | source positions, reordered per `proprio_indices` into per-leg groups | always |
| action.* (12) | JointSensorimotorBridge | efference-copy channel of the 9-D output | always |
| action.* (12) | body (host action-channel poll) | servo PD targets | always |
| reality.motor_leg.{fl..rr} | MotorEPMv2 | HK state x per leg | always |
| reality.motor_leg.{fl..rr} | KeyframeGait | posture accumulation into phase bins | always |
| reality.motor_leg.{fl..rr} | BodyRhythmTracker | measured gait phase/ω extraction | always |
| reality.motor_leg.{fl..rr} | motor_gng_{fl..rr} EPMs | GNG input (RBF, 9-D) | always |
| reality.proprio.tilt | MotorEPMv2 (`tilt_topic`, set in config) | active balance reflex | balance_gain=−0.5 (nonzero) — **but topic never published: silent dead code** |
| reality.proprio.imu | MotorEPMv2 (`imu_topic`) | imu[3] ang_v → dead-reckoned bearing integration + heading PD D-term; imu[2] fwd_v → progress-commit / stuck detection | heading_bearing_hold_gain=7.0 (P), heading_hold_gain=0.3 (D) — LIVE; heading_gain=0.0 (yaw-rate regulator OFF); progress_commit_gain=1.0 — LIVE. Heading PD gated off while nav steers |
| reality.proprio.target_compass | MotorEPMv2 (`nav_topic`) | steer target dead-ahead | nav_gain=5.0 — **LIVE ORACLE** (declared scaffold, tier "nav_oracle") |
| reality.proprio.ground_clearance | MotorEPMv2 (`height_topic`, overrides default chassis_y_norm) | height homeostat + grounded-stiffening | height_homeo_gain=0.04, height_k=0.3, height_ground_gain=0.01 — LIVE |
| reality.proprio.distress | MotorEPMv2 (`distress_topic` **header default**, not in config JSON) | panic pathway (decoupled flail escape) | panic_on default 0.5, panic_motor_mult=4.0 — LIVE |
| reality.proprio.lateral_v | MotorEPMv2 (`lateral_topic` **header default**) | anti-crab coord fitness penalty | coord_lat_penalty default 0.0 — subscribed but INERT |
| reality.proprio.feet_y_gravity_cmd_imu | MotorEPMv2 (`feet_topic`, overrides header default `reality.proprio.feet_y`) | stance/swing inference (foot height vs own EMA) → stance-gated knee tuck + hip2, step clock touchdowns | stance_lift_gain=0.5, stance_lift_hip2=0.25 — LIVE |
| rhythm.cpg.body | MotorEPMv2 (`cpg_phase_topic`) | clean entrained phase; Cphi phase-embedding feed-forward | cpg_embed=true, coupling_gain=1.55, stroke_gain=1.2 — LIVE |
| objective.posture.{fl..rr} | MotorEPMv2 (Feedback) | soft posture targets; Cphi trained on keyframe error | needs cpg_embed + keyframe confidence>0 — LIVE |
| events.* (prefix sub) | MotorEPMv2 | only `events.miss` and `events.reset` acted on (reset-mask: bearing origin, stall/commit clocks, step clocks); all other event names received and ignored | always |
| rhythm.cpg.body | KeyframeGait (`cpg_topic`) | phase bin index (16 bins) | always |
| rhythm.body.gait | CPGOscillator (`entrain_topic`) | entrain CPG freq/phase to body's measured rhythm | entrain_freq_gain=0.02, entrain_phase_gain=0.05 — LIVE |
| neuro.state | CPGOscillator + all 5 EPMs | neuromodulation scaling | unconditional subscription — **UNFED** (no NeurochemState module) |
| consensus.0 | CPGOscillator | TLE competence gate | unconditional — **UNFED** (no LateralVoter) |
| reality.proprio.foot_load | support_epm (EPM) | GNG input (RBF, 4-D, dim range [0,0.8]) | always |
| prediction.motorgng.{fl..rr}, prediction.support.body | respective EPMs (Feedback) | descending-prediction subtraction (`subtract_descending_prediction` defaults **true**) | **UNFED** (no DescendingPredictor) — GNG sees raw encoding |
| KeyframeGait bake gates (bake_upright/contact/reset) | — | not subscribed: all three default empty ⇒ bake gate OFF | — |

Not subscribed by MotorEPMv2 in this config (params default empty): upright_topic, contact_topic, torque_topic, rhythm_topic, intent_topic, goal_bearing_topic, cog_steer/cog_thrust, boredom/interest/hunger, velocity_objective_topics.

## 3. UNCONSUMED OUTPUTS (published, zero subscribers)
- **reality.motorgng.fl / .fr / .rl / .rr** — all 4 motor-GNG EPM token streams (no LateralVoter/consumer; diagnostics-only via inspector).
- **reality.support.body** — by design (config metadata: "NOTHING CONSUMES THE TOKENS").
- Body topics with no module subscriber: **reality.proprio.**{compass, radial_compass, target_loom, ego_heading, vel_ego, feet_y, feet_y_body, feet_y_gravity, feet_y_gravity_fk, feet_y_gravity_cmd, feet_y_gravity_cmd_acc, imu6, foot_contact, feet_y_ground, upright, chassis_y_norm, joint_torque, joint_load, bucket_fl/fr/rl/rr, leg_id_fl/fr/rl/rr, tgt_align_fl/fr/rl/rr}.
- Events received only by MotorEPMv2's prefix sub but ignored by its handler: events.hit, events.alive, events.episode_end, events.beacon_reached, events.leg_planted_*, events.leg_lifted_*, events.leg_event, events.hit_leg_* (only miss/reset are acted on).

## 4. UNFED INPUTS (subscribed, nothing publishes)
- **reality.proprio.tilt** — MotorEPMv2, with balance_gain=−0.5 set in config. `publish_tilt` is false (body default + metadata + no env). The balance reflex is silent dead code; the exact failure mode called out in MotorEPMv2.cpp:395's own param doc.
- **neuro.state** — CPGOscillator + all 5 EPMs (only NeurochemState publishes it; absent).
- **consensus.0** — CPGOscillator competence gate never engages (only a consensus EPM/LateralVoter would publish; absent).
- **prediction.motorgng.{fl,fr,rl,rr}**, **prediction.support.body** — EPM descending-prediction Feedback subs (default-on); no DescendingPredictor in config. Benign: nothing subtracted.
- **reality.proprio.motor_intent** — not subscribed here (intent_topic empty) AND not published (intent_fwd=−1): absent on both sides.

## 5. GOD'S-EYE / ORACLE FLAGS
| Topic | Status | In this config |
|---|---|---|
| reality.proprio.target_compass | **ORACLE** — ground-truth bearing to target (CLAUDE.md §5.2 disqualifier) | **CONSUMED LIVE**, nav_gain=5.0. Declared scaffold: metadata tier "nav_oracle", scaffolds_active includes "nav_oracle" |
| reality.proprio.feet_y | **GOD'S-EYE** — absolute world-Y (the sensor-legitimacy doc's headline violation) | Published but NOT consumed: `feet_topic` is set to `reality.proprio.feet_y_gravity_cmd_imu` (gyro-fused commanded-FK — the hardware-realizable legal twin). The doc's "LIVE god's-eye dependency in the deployed gait" is repaired in this arm |
| reality.proprio.chassis_y_norm | **GOD'S-EYE** — absolute chassis world-Y | Published but NOT consumed: `height_topic` overridden to `reality.proprio.ground_clearance` (legal belly ToF) |
| reality.proprio.imu | Mixed: sin/cos(yaw) is exact world-frame yaw; fwd_v/ang_v are world-velocity projections (doc §"fwd_v, lateral_v": soft oracle) | CONSUMED. Heading PD only integrates ang_v (accepted as dead-reckoned own-yaw); progress_commit (gain 1.0) reads fwd_v — **live soft-oracle dependency** |
| reality.proprio.vel_ego | soft oracle (world velocity projected; flagged in its own registration string) | published, unconsumed |
| reality.proprio.lateral_v | soft oracle (same world-frame velocity family) | subscribed by default but inert (coord_lat_penalty=0) |
| reality.proprio.compass | world-frame yaw | published, unconsumed |
| reality.proprio.distress | derived from world-XZ position history (net displacement deficit) × tilt — not directly sensable; approximable on hardware via odometry only | **CONSUMED LIVE** by panic pathway (panic_motor_mult=4.0) |
| reality.proprio.foot_contact | egocentric in sim, but doc §hardware: real picrawler has no foot switches | unconsumed here |
| events.hit/miss | reward machinery keyed to god's-eye chassis height | only miss consumed, and only as a reset-mask (disruption marker), not a gradient |
| reality.proprio.ground_clearance, foot_load, joint_load, feet_y_body, feet_y_ground, feet_y_gravity_cmd_imu | legal egocentric (per doc) | ground_clearance, foot_load, feet_y_gravity_cmd_imu consumed; rest unconsumed |

Key file paths: `godot_host/project/addons/ami_ogma/configs/the_picrawler_motor_epm_embed_corridor_imufused__stroke12__gng__bellyset__stancehip2__supportepm.json`, `cpp_core/src/ogma/modules/MotorEPMv2.cpp` (subs at 925–1045, action publish 3862, event handler 1068), `cpp_core/include/ogma/modules/MotorEPMv2.hpp` (non-empty topic defaults at 441, 935, 954, 978, 1048, 1163–1164 — the reason distress/lateral are subscribed without appearing in the config), `cpp_core/src/ogma/modules/EPM.cpp` (output topic build 203–206, prediction sub 332–337), `godot_host/project/scripts/picrawler_body.gd` (register block 2576–2725, publish block 5468–6203), `godot_host/src/OgmaBrain.cpp` (topic mapping 440–484).

Notable for the diagram: the closed motor loop is body joints → `reality.proprio.joints` → bridge → `reality.motor_leg.*` → {MotorEPMv2, KeyframeGait, BodyRhythmTracker, 4× EPM} → MotorEPMv2 → `action.*` → body servos (+ efference copy back into the bridge). The rhythm loop is `reality.motor_leg.*` → BodyRhythmTracker → `rhythm.body.gait` → CPG (entrain) → `rhythm.cpg.body` → {KeyframeGait bins, MotorEPMv2 phase embedding} → `objective.posture.*` → MotorEPMv2. Both EPM groups are pure observers with zero downstream consumers.

---

# Reader B — MotorEPMv2 mechanism inventory (148 params)

I have everything needed. Here is the structured report.

---

# MotorEPMv2 MECHANISM INVENTORY — deployed config `the_picrawler_motor_epm_embed_corridor_imufused__stroke12__gng__bellyset__stancehip2__supportepm.json`

Sources: `cpp_core/src/ogma/modules/MotorEPMv2.cpp` (param table lines 111–578, tick lines 2265–3910), `cpp_core/include/ogma/modules/MotorEPMv2.hpp` (member defaults/constants), config lines 86–173.

**Environment facts that drive ON/OFF verdicts** (verified in `godot_host/project/scripts/picrawler_body.gd`):
- `reality.proprio.tilt` is published ONLY if `publish_tilt` (@export default false, line 2116; config metadata `publish_tilt: false`) → **tilt never arrives**; `tilt_pitch_`/`tilt_roll_` stay 0.
- `reality.proprio.target_compass` publishes **(0,0)** unless `walk_target_idx >= 0`, which requires `target_mode="random_pyramid"` from a curriculum (default `target_mode="off"`, line 1797; no curriculum in this config) → nav gate `|tc_x|+|tc_y|>0.05` never passes.
- `reality.proprio.distress`, `.imu`, `.lateral_v`, `.ground_clearance`, `.feet_y_gravity_cmd_imu` are published every tick → those inputs are live.
- `contact_topic`, `torque_topic`, `upright_topic`, `intent_topic`, `goal_bearing_topic`, `velocity_objective_topics`, `rhythm_topic` are all EMPTY in this config → their consumers are unwired.

## Per-param table (schema order)

| # | param | deployed | status | mechanism (≤15 words) | reads | writes |
|---|---|---|---|---|---|---|
| 1 | proprio_topics | 4× reality.motor_leg.* | ON | per-leg 9-D state input [pos,act,delta]×3 | bus | L.x |
| 2 | action_topics | 12× action.<leg>_<joint> | ON | command outputs, one per servo | — | all 12 joints |
| 3 | objective_topics | 4× objective.posture.* | ON | KeyframeGait soft posture target; retargets HK error + trains Cphi | PredictionToken (w=0.3·self_precision) | obj_target_/obj_weight_ |
| 4 | velocity_objective_topics | DEFAULT empty | OFF | phase-indexed velocity target trains Cvel | — | Cvel (stays 0) |
| 5 | n_legs | 4 | ON | leg count | — | — |
| 6 | motor_dim | 3 | ON | joints/leg (hip1,hip2,knee) | — | — |
| 7 | model_lr | 0.02 | ON | forward-model A,b descent on ξ | ξ = x − (A·prev_y+b) | L.A, L.b |
| 8 | ctrl_lr | 0.01 | ON | homeokinetic C descent on metric error ξ̃ | ξ̃, A, G | L.C |
| 9 | bias_lr | 0.005 | ON | controller bias descent | G·Aᵀ·q | L.h |
| 10 | reg_eps | 0.01 | ON | (LLᵀ+εI)⁻¹ regularizer | — | HK gradient |
| 11 | max_dctrl | 0.05 | ON | per-tick ‖ΔC‖ clamp | dC norm | dC |
| 12 | init_scale | 0.01 | ON | random init magnitude of A, C | RNG (init) | A, C init |
| 13 | cmd_squash | DEFAULT 0 | OFF | tanh squash instead of hard ±1 clamp | — | final y |
| 14 | dep_gain | DEFAULT 0 | OFF | DEP derivative-correlation replaces HK's C update | Δx, Δy | L.C, L.Cdep |
| 15 | dep_alpha | DEFAULT 0.05 | INERT (sub of dep_gain=0) | DEP correlation EMA rate | — | — |
| 16 | sense | DEFAULT 0 | OFF | PM's confining ∂G term (epsrel) | C,q,A,G | dC |
| 17 | ctrl_damping | DEFAULT 0 | OFF | L2 decay on C and h | — | C, h |
| 18 | whole_body_c | DEFAULT 0 | OFF | one controller over all 12 joints | — | Zw_/Cw_ |
| 19 | c_init | DEFAULT 0 | OFF | self-exciting diagonal C init (Sox cInit) | — | C init |
| 20 | seed | 1234 | ON | base RNG seed; per-leg = base^leg | — | all RNGs |
| 21 | babble_ticks | 200 | ON | warmup: random commands, model-only learning | steps_seen | warmup flag |
| 22 | babble_scale | 0.3 | ON (ticks ≤200) | uniform babble amplitude | L.babble_rng | y (warmup) |
| 23 | sat_lr | 0.02 | ON | anti-saturation push out of tanh rails | z=Cx+h | C, h |
| 24 | postural_gain | 0.7 | ON | PD pull of all 3 joints toward rest pose | x[3j], rest_pos | y[0..2] |
| 25 | postural_gain_joints | DEFAULT empty | OFF (uniform 1.0) | per-joint tone profile multiplier | — | — |
| 26 | explore_noise | 0.05 | ON, conditionally zeroed | per-tick Gaussian noise σ; ×explore_mult (commit can zero it, explore_floor=0) | commit state | y all joints |
| 27 | knee_tuck_target | 0.7 | ON | overrides knee rest → spider tuck stance | first proprio frame | rest_pos[2] |
| 28 | hip2_tuck_target | DEFAULT −99 | OFF | overrides hip2 rest (crouch femur) | — | — |
| 29 | motor_gain | 3.0 | ON | multiplier on tanh(HK) output | — | y all joints |
| 30 | coupling_gain | 1.55 | ON | Kuramoto pull of leg phase toward gait_phase offsets | L.phase all legs, gait_phase | y[knee] |
| 31 | intent_topic | DEFAULT "" | OFF | (v*,w*) intent → commit-precision meaning | — | — |
| 32 | explore_floor | DEFAULT 0 | OFF | lower bound on explore_mult (never abolish search) | — | — (mult can hit 0) |
| 33 | phase_sym_smooth | DEFAULT 0 | OFF | symmetric 2-arm smoothing of L.phase atan2 | — | — |
| 34 | phase_vel_smooth | DEFAULT 0 | OFF | velocity-arm-only phase smoothing (refuted) | — | — |
| 35 | fwd_resonance_gain | DEFAULT 0 | OFF (estimator still runs as instrument) | Hopf resonator entrained by fwd_v couples legs | fwd_v_, res state | y[knee] (gated off) |
| 36 | intent_rhythm_gain | DEFAULT 0 | OFF (also needs intent) | stride-shape deviation term in intent error | — | fwd_profile_ (not running) |
| 37 | lookahead_gain | DEFAULT 0 | OFF | controller acts on predicted state x̂ | — | — |
| 38 | lookahead_mode | DEFAULT 0 | INERT (sub of 37) | fixed-point vs prev-action x̂ | — | — |
| 39 | lookahead_null | DEFAULT 0 | INERT (sub of 37) | control arm: x̂:=b | — | — |
| 40 | intent_yaw_gain | DEFAULT 1.0 | INERT (inside intent block, no intent) | yaw-term weight in intent error; non-zero default displays enabled | — | — |
| 41 | commit_prec_gain | DEFAULT 0 | OFF | commit windows scaled by earned prediction precision (cp=1) | — | — |
| 42 | commit_window_ticks | DEFAULT 180 | ON (commit live via #68) | ticks of progress before commit engages | fwd_progress_ema_ | commit_ticks_ |
| 43 | commit_rise_ticks | DEFAULT 240 | ON | commit ramp-to-full time | — | commit_boost_ |
| 44 | commit_decay_ticks | DEFAULT 90 | ON | commit release time | — | commit_boost_ |
| 45 | heading_trim_rate | DEFAULT 0 | OFF | heading I-term learns DC steer effort | — | heading_trim_ (0) |
| 46 | heading_trim_leak | DEFAULT 0.001 | INERT (sub of 45) | trim integrator leak | — | — |
| 47 | c_pair_init | DEFAULT 0 | OFF | L/R partner legs share init seed | — | — |
| 48 | goal_bearing_topic | DEFAULT "" | OFF | L1 nav setpoint replaces spawn bearing | — | — (bearing_err = −heading_bearing_) |
| 49 | couple_prec_gain | DEFAULT 0 | OFF | precision-weighted Kuramoto neighbour mean | — | — (uniform mean) |
| 50 | phase_joint | DEFAULT −1 → knee | ON | joint whose pos/delta derive L.phase | x[3pj], x[3pj+2] | L.phase |
| 51 | rhythm_gains | DEFAULT empty | OFF | per-joint coherent sin drive at leg phase | — | — |
| 52 | rhythm_offsets | DEFAULT empty | INERT (sub of 51) | per-joint rhythm phase offsets | — | — |
| 53 | cpg_phase_topic | rhythm.cpg.body | ON | global CPG phase context for embed | CPGOscillator token | cpg_phase_, cpg_seen_ |
| 54 | cpg_embed | true | ON | phase-conditioned feed-forward Cphi·[cosφ,sinφ] added pre-tanh | cpg_phase_ | y (pre-tanh), z in learning |
| 55 | embed_lr | DEFAULT 0.02 | ON | Cphi trained on keyframe error (x*−x) at command phase | obj_target_, x | L.Cphi |
| 56 | embed_decay | DEFAULT 0.001 | ON | L2 bound on Cphi (and Cvel) | — | L.Cphi |
| 57 | ctrl_symmetry_gain | DEFAULT 0 | OFF | pull per-leg C,h,Cphi toward group mean | — | — |
| 58 | symmetry_group_of | DEFAULT empty | INERT (sub of 57) | leg grouping for symmetry coupling | — | — |
| 59–60 | rhythm_fade_start/end | DEFAULT −1 | OFF | scheduled linear fade of rhythm scaffold | tick_id | rhythm_scale_ (stays 1) |
| 61–62 | coupling_fade_start/end | DEFAULT −1 | OFF | scheduled linear fade of Kuramoto coupling | tick_id | coupling_eff_ (stays 1.55) |
| 63 | gait_phase | [0,π,π,0] trot | ON, MUTATED live | per-leg target phase offsets; modified by #64 and #66 every tick/probe | — | coupling + coord state |
| 64 | coord_adapt_rate | 0.001 | ON | gait_phase crystallises toward emergent leg phases (Hebbian) | L.phase rel. leg0 | gait_phase_[1..3]; draws coord_rng_ every tick |
| 65 | coord_explore | DEFAULT 0 | OFF (RNG still drawn ×3/tick, multiplied by 0) | persistent phase-offset noise | coord_rng_ | — |
| 66 | coord_reward_drive | 0.3 | ON | (1+1) hill-climb probes on phase offsets, keep/revert per 240-tick window | fitness (see #79) | gait_phase_, coord_best_phase_/fitness_ |
| 67 | stuck_explore_gain | DEFAULT 0 | OFF | stall-triggered exploration amplification | — | stuck_boost_ (pinned 0) |
| 68 | progress_commit_gain | 1.0 | ON | sustained progress damps exploration (explore_mult→0) + adds stroke thrust (+30% max) | fwd_progress_ema_ vs 0.030 | explore_mult, lever_stroke_mult |
| 69 | forward_flow_gain | DEFAULT 0 | OFF | stroke amp ∝ flow magnitude·predictability | — | — |
| 70 | stance_lift_gain | 0.5 | ON | knee bias +0.5 on PLANTED legs only (belly-up) | in_swing_ (foot-height detector) | y[knee] planted legs |
| 71 | stance_release_frac | DEFAULT 0 | OFF | fade stance press after commanded stroke reversal | — | press stays 1 |
| 72 | swing_hyst_frac | DEFAULT 0 | OFF | MAD deadband on swing detector; legacy foot_y>EMA (chatters ~2×/step) | — | in_swing_ (legacy path) |
| 73 | homeo_leak_upright_only | DEFAULT 0 | OFF | stop homeostat forgetting when tilted | — | — |
| 74 | homeo_leak_progress_gate | DEFAULT 0 | OFF | scale effort leak by progress | — | — |
| 75 | homeo_leak_cycles | DEFAULT 0 | OFF | stride-cycle leak on height_bias/amp_gain — **both integrators leak-free here** | — | leak_amp_=leak_h_=0 |
| 76 | rhythm_topic | DEFAULT "" | OFF | body omega for leak timescale | — | body_omega_ unused |
| 77 | homeo_upright_gate | DEFAULT 0 | OFF | freeze integrators while inverted | — | — |
| 78 | height_unwind_free | DEFAULT 0 | OFF | asymmetric windup fade — legacy: railed height_bias cannot unwind while walking | — | — |
| 79 | coord_fitness_mode | 1 | ON | reward-free probe fitness: coherence·activity/(1+tle) − 0.3·wob | gait_coherence(), amp_ema_mean(), tle_ema_mean(), tilt | coord_fit_accum_ |
| 80 | coord_probe_ticks | 240 | ON | probe window; fitness over back half | tick counter | — |
| 81 | coord_stab_penalty | 0.3 | **CONDITIONALLY DEAD** — tilt unpublished → wob≡0, penalty always 0 | wobble penalty in probe fitness | tilt_pitch_/roll_ (=0) | — |
| 82 | coord_lat_penalty | DEFAULT 0 | OFF (mode 1 drops it anyway) | anti-crab probe penalty | lateral_v_ | — |
| 83 | coord_intent_nav | DEFAULT 0 | OFF | probe reward = velocity toward target | — | — |
| 84 | cruse_gain | DEFAULT 0 | OFF | Cruse rules 1/2/3 hip2+knee lift sequencing | in_swing_, ticks_since_plant_ | — |
| 85 | cruse_rule3_weight | DEFAULT 0.5 | INERT (sub of cruse_gain=0; displays enabled) | contralateral hold weight | — | — |
| 86 | cruse_rule2_window | DEFAULT 15 | INERT (sub of cruse_gain=0) | post-touchdown swing-release window | — | — |
| 87 | cruse_rule5_gain | DEFAULT 0 | OFF | stance press ∝ number of swinging legs | — | — |
| 88 | upright_topic | DEFAULT "" | OFF | uprightness input; upright_ pinned at init 1.0 | — | — |
| 89 | contact_topic | DEFAULT "" | OFF | true foot-contact for stance gate; falls back to foot-height inference | — | have_contact_=false |
| 90 | feet_topic | reality.proprio.feet_y_gravity_cmd_imu | ON | 4-D foot heights (IMU-fused legal variant) feeding swing detector | body topic | foot_y_ → in_swing_ |
| 91 | stroke_gain | 1.2 | ON | hip1 power stroke sin(L.phase+stroke_phase), ×lever_stroke_mult | L.phase, commit | y[hip1] |
| 92 | stroke_phase | −2.85 | ON | stroke timing offset vs knee phase | — | stroke waveform |
| 93 | stroke_phase_src | DEFAULT 0 | OFF | touchdown-referenced step clock for the stroke | — | L.step_phase (never updates) |
| 94–98 | step_phase_debounce/period_alpha/min/max/lock | DEFAULTS 2/0.2/8/200/0.10 | INERT (sub of #93=0) | step-clock plumbing | — | — |
| 99 | gait_raster_diag | DEFAULT 0 | OFF | Hildebrand footfall ring for inspector | — | — |
| 100 | steer | 0.0 | OFF | manual L/R stroke differential | — | steer_eff term=0 |
| 101 | stroke_signs | [1,−1,1,−1] | ON | per-leg hip1 push direction | — | stroke sign |
| 102 | propulsion_balance_gain | DEFAULT 0 | OFF | boost below-mean propulsive-credit legs | — | prop_credit still not computed |
| 103 | balance_gain | −0.5 | **CONDITIONALLY DEAD** — tilt topic never published → term adds exactly 0 | tilt-levelling hip2 differential | tilt_pitch_/roll_ (=0) | y[hip2] (+0) |
| 104 | tilt_topic | reality.proprio.tilt | WIRED-BUT-SILENT | 4-D tilt input; body publish_tilt=false | — | tilt_* stay 0 |
| 105 | amp_homeo_gain | 0.01 | ON | per-leg integral gain drives amp_ema→amp_target; multiplies HK output | L.amp_ema | L.amp_gain (clamp 0.1–5, **no leak**) |
| 106 | amp_target | 0.4 | ON | amplitude setpoint | — | — |
| 107 | amp_seek_rate | DEFAULT 0 | OFF | (1+1) CoT search on amp_target | — | — |
| 108 | amp_seek_ticks | DEFAULT 900 | INERT (sub of 107) | CoT probe window | — | — |
| 109 | heading_gain | 0.0 (explicit) | OFF | raw yaw-rate → steer damping | — | — |
| 110 | heading_hold_gain | 0.3 | ON | D term: −0.3·yaw_rate_ema into skid-steer | yaw_rate_ema_ (imu) | steer_eff → y[hip1] |
| 111 | heading_bearing_hold_gain | 7.0 | ON | P term: 7·(−dead-reckoned bearing) into skid-steer; the promoted go-straight | heading_bearing_ (integrated yaw, reset on respawn) | steer_eff → y[hip1] |
| 112 | imu_topic | reality.proprio.imu | ON | [sin/cos yaw, fwd_v, ang_v]; feeds fwd_v_, yaw_rate_, bearing integrator | body topic | fwd_v_, yaw_rate_, heading_bearing_ |
| 113 | nav_gain | 5.0 | **CONDITIONALLY DEAD** — target_compass ≡ (0,0) (no target in standard corridor run); nav_on gate never passes | steer to zero target bearing + facing-gated thrust | tc_x_, tc_y_ | steer_eff, fwd (both untouched) |
| 114 | nav_topic | reality.proprio.target_compass | WIRED, carries (0,0) | egocentric target bearing input | body topic | tc_x_, tc_y_ |
| 115–118 | cog_steer_gain/topic, cog_thrust_gain/topic | DEFAULT 0/"" | OFF (cell-only, gated n_legs==1) | cognitive steer/thrust for the cell | — | — |
| 119–123 | boredom_noise_gain, boredom/interest/hunger_topic, boredom_escalation_rate | DEFAULT 0/"" | OFF (cell-only) | run-and-tumble boredom escape | — | — |
| 124 | height_homeo_gain | 0.04 | ON (rest/stand-up only) | integral hip2 lift toward height_k_eff·chassis_h_max; **output AND integration ×height_rest_frac which →0 at fwd_progress≥0.025** | chassis_h_ema_, chassis_h_max_, fwd_progress_ema_ | height_bias_ → y[hip2] |
| 125 | support_select_gain | DEFAULT 0 | OFF (also needs contact_topic) | explore_mult ∝ support-state responsiveness value | — | — |
| 126 | stance_lift_hip2 | 0.25 | ON | fraction of stance lift also on hip2: +0.125 on planted legs | in_swing_ | y[hip2] planted legs |
| 127 | height_lift_knee | DEFAULT 0 | OFF; **by design CONDITIONALLY DEAD even when set** (path ×height_rest_frac →0 while walking) | extend height lift to knee | — | — |
| 128 | height_ground_gain | 0.01 | ON | height_k_eff rises on belly grounding (<0.05 clearance), slow-decays toward height_k; clamp [0.3, 0.95] | chassis_h_ | height_k_eff_ |
| 129 | height_k | 0.3 | ON | floor of the adapted setpoint fraction | — | — |
| 130 | height_topic | reality.proprio.ground_clearance | ON | belly rangefinder (replaced god's-eye chassis_y_norm) | body topic | chassis_h_, ema, **chassis_h_max_ ratchet** |
| 131 | panic_on | DEFAULT 0.5 | ON (armed) | distress hysteresis engage threshold | distress_ | panic_latched_ |
| 132 | panic_off | DEFAULT 0.25 | ON (armed) | hysteresis release threshold | distress_ | panic_latched_ |
| 133 | panic_strength | DEFAULT 1.0 | ON (armed) | overall panic scale pe | panic_ | pe |
| 134 | panic_noise | DEFAULT 0.4 | ON when panicking | added exploration σ at full panic | pe | noise_sigma |
| 135 | panic_motor_mult | 4.0 | ON when panicking | motor_gain ×4 at full panic | pe | mg |
| 136 | panic_push_amp | DEFAULT 1.2 | ON when panicking | staggered rectified hip2+knee anti-gravity pump | panic_phase_ | y[hip2], y[knee] |
| 137 | panic_push_hz | DEFAULT 0.8 | ON when panicking | pump frequency | — | panic_phase_ |
| 138 | distress_topic | DEFAULT reality.proprio.distress | ON | wedge severity input (published every tick) | body topic | distress_ |
| 139 | lateral_topic | DEFAULT reality.proprio.lateral_v | ON as input; **instrument only** (consumers #82/#83 off) | sideways-slip velocity | body topic | lateral_v_ (diag only) |
| 140 | torque_topic | DEFAULT "" | OFF | 12-D servo load input | — | have_torque_=false |
| 141 | contact_instrument_only | DEFAULT 0 | INERT (no contact topic) | split contact subscription from stance gate | — | — |
| 142 | stroke_load_gain | DEFAULT 0 | OFF | scale propulsion by hip1 load share | — | stroke_gate_ ≡ 1 |
| 143 | tibia_plumb_gain | DEFAULT 0 | OFF | hip2 nulls shank deviation from vertical | — | — |
| 144–145 | tibia_plumb_scale/offset | DEFAULTS 1.40 / −0.0292 | INERT (kinematic constants, sub of 143) | proprio-units→radians conversion | — | — |
| 146–147 | swing_tuck_hip2/knee | DEFAULT 0 | OFF (also needs contact_topic) | fold limb inboard on lifted legs | — | — |
| 148 | gait_align_diag | DEFAULT 0 | OFF | stroke-vs-touchdown phase-lock measurement | — | — |

## 1. LIVE MECHANISMS (order of application to the command, per leg, post-warmup)

1. **HK core** (`model_lr`/`ctrl_lr`/`bias_lr`/`reg_eps`/`max_dctrl`/`sat_lr`): y = C·x + h; C descends the metric error with the **objective retarget** blending in the KeyframeGait posture target (w = 0.3·self_precision) on position components; Cphi trains on keyframe error.
2. **CPG embed** (`cpg_embed` + `cpg_phase_topic` + `embed_lr`/`embed_decay`): y += Cphi·[cosφ,sinφ] pre-tanh (Cvel stays 0, socket unwired).
3. **Gain stage**: y = motor_gain(3.0, ×4 under panic) · amp_gain(per-leg amplitude homeostat, `amp_homeo_gain` 0.01 → target 0.4) · tanh(y).
4. **Postural reflex** (`postural_gain` 0.7 + `knee_tuck_target` 0.7): PD toward spawn rest on all three joints, knee rest overridden to spider tuck.
5. **Height homeostat** (`height_homeo_gain` 0.04, `height_k` 0.3, `height_ground_gain` 0.01, topic = belly rangefinder): y[hip2] += height_bias·height_rest_frac — **fades to zero at cruise** (fwd_progress ≥ 0.025); setpoint fraction self-raises on belly grounding.
6. **Stance lift** (`stance_lift_gain` 0.5 + `stance_lift_hip2` 0.25): planted legs (per foot-height detector on `feet_y_gravity_cmd_imu`, legacy no-deadband foot_y>EMA) get knee +0.5 and hip2 +0.125.
7. **Exploration noise** (`explore_noise` 0.05): Gaussian on all joints, σ ×explore_mult — **commit can drive σ to exactly 0** (explore_floor 0).
8. **Kuramoto coupling** (`coupling_gain` 1.55): knee bias pulling leg phase toward `gait_phase` offsets — which are themselves live-mutated by **coord_adapt_rate 0.001** (crystallise toward emergent) and the **(1+1) coordination search** (`coord_reward_drive` 0.3, `coord_probe_ticks` 240, `coord_fitness_mode` 1 = coherence·activity/(1+tle); its tilt penalty is dead). Panic decouples (×(1−pe)).
9. **Power stroke + steering** on hip1 (`stroke_gain` 1.2, `stroke_phase` −2.85, `stroke_signs` [1,−1,1,−1]): amp = sgn·(1.2·lever_stroke_mult + side·steer_eff)·sin(L.phase−2.85), where steer_eff = **heading_bearing_hold_gain 7.0**·(−dead-reckoned bearing) + **heading_hold_gain 0.3**·(−yaw_rate_ema) — the promoted go-straight PD — and lever_stroke_mult = 1 + 0.3·commit (`progress_commit_gain` 1.0). Panic kills it (×(1−pe)).
10. **Panic pathway** (armed, fires when distress > 0.5): hysteresis latch, smooth ramp; ×4 motor gain, +0.4 noise, decouples coupling/stroke, staggered rectified hip2+knee pump at 0.8 Hz amp 1.2.
11. **Hard clamp** to ±1 (cmd_squash 0), publish 12 ActionOut.

Module-level machinery feeding the above: commit detector (fwd_progress_ema vs 0.030, windows 180/240/90), phase pre-pass (knee-derived L.phase via `phase_joint` −1), amp homeostat integration, height integrator, coord adaptation+search, respawn/reset handling (zeroes bearing, stall/commit clocks).

## 2. DEAD/INERT in this config

**Explicitly configured but dead (the traps):**
- `balance_gain = −0.5` — dead: `reality.proprio.tilt` never published (publish_tilt false). Listed in metadata `scaffolds_active` as "balance" but contributes exactly 0.
- `coord_stab_penalty = 0.3` — dead for the same reason (wob ≡ 0); the mode-1 coordination fitness runs unpenalised.
- `nav_gain = 5.0` — dead in the standard corridor run: target_compass ≡ (0,0) (target_mode "off", no curriculum); nav_on gate never passes. Metadata "nav_oracle" tier notwithstanding. Would go live if a pyramid target were activated.
- `steer = 0.0`, `heading_gain = 0.0` — explicit zeros.

**Default-off gains (gain-0-guarded)**: cmd_squash, dep_gain(+dep_alpha), sense, ctrl_damping, whole_body_c, c_init, c_pair_init, postural_gain_joints, hip2_tuck_target, explore_floor, phase_sym_smooth, phase_vel_smooth, fwd_resonance_gain, intent_rhythm_gain, lookahead_gain(+mode,+null), commit_prec_gain, heading_trim_rate(+leak), couple_prec_gain, rhythm_gains(+offsets), ctrl_symmetry_gain(+groups), rhythm/coupling fades, coord_explore, stuck_explore_gain, forward_flow_gain, stance_release_frac, swing_hyst_frac, all homeo_leak/upright gates, height_unwind_free, coord_lat_penalty, coord_intent_nav, cruse_gain(+rule3_weight 0.5, rule2_window 15 — non-zero defaults that display enabled), cruse_rule5_gain, stroke_phase_src(+5 step-clock subs), propulsion_balance_gain, amp_seek_rate(+ticks), support_select_gain, height_lift_knee, stroke_load_gain, tibia_plumb_gain(+scale/offset), swing_tuck_hip2/knee, all cell params (cog_*, boredom_*, interest, hunger).

**Unwired sockets**: intent_topic, goal_bearing_topic, upright_topic, contact_topic, torque_topic, rhythm_topic, velocity_objective_topics — their dependent mechanisms cannot fire regardless of gains.

**Conditionally dead by construction (flag for any future enabling)**:
- `height_lift_knee`: its path is `height_lift_knee · height_bias · height_rest_frac` — rest_frac →0 while walking, so it can never act during gait even if set.
- Similarly the entire height homeostat output is rest-gated; `height_bias` **cannot unwind while walking** (height_unwind_free 0, no leak) — railed-latch hazard documented in code.
- `explore_noise` is conditionally zeroed at full commit (explore_floor 0) — the "frozen-but-confident" state named in the explore_floor docstring.
- `intent_yaw_gain` (1.0) and `cruse_rule3_weight` (0.5) are non-zero sub-params of off mechanisms — panel displays them as enabled.

## 3. INSTRUMENTS vs AUTHORITY

**Pure instruments (no command authority, always computed)**: phase pre-pass diagnostics (phase_retro/phase_freq), inter-leg PLV (run + windowed, amp-floor-gated), couple_R, swing_frac_ema, phase_agree/legphase_agree, hk_agree (hip2/knee sign agreement), saturation instruments (sat_pre/clip per joint and per leg), support responsiveness diag (support_resp; per-bin EMAs only with the selector on), resonator state (res_freq/res_amp/res_lock — estimator runs unconditionally, output gated off at gain 0), la_dev/rhythm_dev/commit_prec/explore_mult/flow_quality diags, reset-rate EMA + ticks_since_reset (Gate 0 bookkeeping), cruse_bias_mean, swing_tuck_frac, stroke_gate diagnostics, cw_spr. Off-by-default diag blocks: gait_align_diag, gait_raster_diag. Full diag key list is in `diag_snapshot()` (lines 4418–4812).
- **lateral_v** is subscribed but instrument-only here (its two consumers are off).
- **Caveat — instruments with indirect authority in THIS config**: `gait_coherence()`, `amp_ema_mean()`, `tle_ema_mean()` are the mode-1 coordination-search fitness, so these "diagnostics" rank probes and therefore steer `gait_phase`. Likewise `fwd_v_`/`fwd_progress_ema_` have authority through commit (explore_mult, stroke mult) and the height fade; `L.amp_ema` gates PLV *and* feeds the amp homeostat and fitness.

## RNG usage (determinism)

Per-mechanism draws in the deployed config:
- **Init (deterministic from seed 1234)**: per-leg `mt19937(seed^const)` for A/C init; `L.babble_rng` seeded per leg; `coord_rng_` (seeded in ctor/setup).
- **Warmup babble** (ticks ≤ 200): 3 uniform draws/leg/tick from `L.babble_rng`.
- **Exploration noise**: 3 normal draws/leg/tick from `L.babble_rng` whenever noise_sigma > 0 — **stream length is behavior-dependent**: full commit zeroes σ and skips the draws; panic re-enables them.
- **coord_adapt block**: `nz(coord_rng_)` drawn for each initialized leg 1–3 **every tick** even though coord_explore = 0 multiplies it by zero — RNG consumed regardless (coord_adapt_rate 0.001 > 0 enables the block).
- **Coordination probe proposal**: 3 normal draws from `coord_rng_` every 240 ticks (coord_reward_drive > 0).
- Not drawing in this config: amp_seek, boredom escape, wb_init. Both `babble_rng` and `coord_rng_` are serialized in snapshot/restore (lines 4031, 4051).

## Persistent winners / ratchets (no or slow decay)

- **`chassis_h_max_`** (handle_height, line 1247): pure max-ratchet on the smoothed belly clearance, **no decay ever**, persisted in snapshots; the height homeostat's setpoint is height_k_eff × this.
- **`coord_best_phase_` / `coord_best_fitness_`** (coord search): persistent incumbent phase offsets; fitness decays ×0.99 per 240-tick window (τ ≈ 100 windows ≈ 8 min) so the incumbent is displaceable but sticky; both serialized.
- **`height_bias_`**: integrator clamped [−0.5, +1.5]; **no leak in this config** (homeo_leak_cycles 0) and legacy fade means it cannot unwind while walking — the documented latch.
- **`L.amp_gain`** (per leg): integrator clamped [0.1, 5.0]; **no leak in this config**; rectifying (only unwinds when amp_ema exceeds target) — the documented inversion-windup shape, all guards for it (leak/upright gate) off here.
- **`height_k_eff_`**: rises fast on grounding, decays slowly (0.01×0.05/tick) toward height_k — self-decaying, clamp [0.3, 0.95].
- **`heading_bearing_`**: dead-reckoned yaw integral, clamp ±4.0, reset to 0 on miss/reset events — persistent within a bout, not a ratchet.
- **`step_per_ema`** is deliberately kept across respawns (step clock unused here); `fwd_profile_` count-annealed bins exist but never run (intent off).

---

# Reader C — control-law assembly, phase system, lift pathway

All reading done. Here is the structured audit report.

---

# MotorEPMv2 per-tick control-law audit (deployed embed/corridor stack)

**Files:** `cpp_core/src/ogma/modules/MotorEPMv2.cpp` (cpp), `cpp_core/include/ogma/modules/MotorEPMv2.hpp` (hpp).
**Config matched:** `godot_host/project/addons/ami_ogma/configs/the_picrawler_motor_epm_embed_corridor_imufused__stroke12__gng__bellyset__stancehip2.json` (exact match to the given params; `__srel05/10` variants differ only by `stance_release_frac`). Relevant non-listed facts from that config: `contact_topic`/`torque_topic`/`intent_topic`/`rhythm_topic` **unset**, `objective_topics=objective.posture.{fl,fr,rl,rr}` (KeyframeGait, gain 0.3), `velocity_objective_topics` **unset** (Cvel never trains), `cpg_phase_topic=rhythm.cpg.body`, `stroke_phase_src` unset (=0 → stroke rides `L.phase`), `phase_joint` unset (=−1 → knee), `swing_hyst_frac` unset (=0 → legacy detector), `height_topic=reality.proprio.ground_clearance` (belly rangefinder), `stroke_signs=[1,−1,1,−1]`, `gait_phase init=[0,π,π,0]`.

---

## 1. THE COMMAND ASSEMBLY — one leg, y = [hip1(0), hip2(1), knee(2)], in execution order

Per-leg loop: cpp:3059–3873. `m=3`. `warmup = steps_seen ≤ 200` (cpp:3065). Module-level pre-passes run first (see §2). During warmup y = uniform babble ±0.3 (cpp:3208–3212) and only steps 4, 17, 20, 21 below apply (each additive term is `!warmup`-gated except postural, height, balance — see notes).

Learning happens **before** the command, on last tick's outcome (cpp:3072–3204): model `A += 0.02·ξ·yᵀ, b += 0.02·ξ` (cpp:3076–3077); HK controller `dC = 2·0.01·(AG)ᵀq(qᵀL)` (cpp:3135), **`max_dctrl=0.05` rate-limits ‖dC‖_F here — it clamps LEARNING, never the output** (cpp:3151–3153); the **KeyframeGait objective** enters as an error-retarget `ξ̃[3j] = (1−w)·ξ[3j] + w·(x[3j]−x*_j)` with w = PredictionToken.confidence = 0.3·self_precision(bin) from KeyframeGait (cpp:3108–3117; handle_objective cpp:1104–1111; KeyframeGait.cpp:104) — **an objective-change in learning, not an additive output term**; `h += 0.005·G·Aᵀq` (cpp:3155–3156); `Cphi` trained on the keyframe error `(x*−x)` at command phase (cpp:3161–3168) — **ON** (cpg_embed && obj_seen); `Cvel` training **OFF** (socket unwired, cpp:3175–3182); anti-saturation `sat_lr=0.02` pushes C,h back from the tanh rails (cpp:3187–3194); `sense`/`ctrl_damping`/DEP/whole-body/lookahead all 0 → **OFF**.

Post-warmup command assembly, in order:

| # | Term | Formula (short) | Gate | Deployed |
|---|---|---|---|---|
| 1 | **HK operating point** | `y = C·x + h` (cpp:3237–3238) | always (wb off, lookahead off) | **ON** |
| 2 | **CPG-embed feed-forward (pre-tanh)** | `y += Cphi·[cosφ_cpg, sinφ_cpg] + Cvel·ctx` (cpp:3255–3258) | `cpg_embed && cpg_seen_` | **ON** (Cphi learned; Cvel ≡ 0) |
| 3 | **HK squash + gains** | `y[j] = mg·ag·tanh(y[j])`, `mg = 3.0·(1+pe·(4.0−1))`, `ag = amp_gain ∈ [0.1,5]` (cpp:3215–3217, 3260) | always | **ON** (panic_motor_mult=4 only bites when pe>0) |
| 4 | **Postural reflex** | `y[j] −= 0.7·(x[3j] − rest_pos[j])`, all three joints; knee rest overridden to **+0.7 tuck** (cpp:3279–3292; rest capture + knee_tuck_target cpp:2249–2260) | `postural_gain>0 && rest_captured` — NOT warmup-gated | **ON** |
| 5 | **Height homeostat lift (hip2)** | `y[1] += (+1)·height_bias_·height_rest_frac_` (cpp:3309–3315); bias integrates `0.04·(height_k_eff·h_max − h_ema)` (cpp:2371–2407), `height_k_eff` adapts 0.3→0.95 on belly grounding at rate 0.01 (cpp:2374–2384); clamp [−0.5,1.5] | `!warmup && height_homeo_gain>0`; **multiplied by `height_rest_frac_ = clamp(1−fwd_ema/0.025)` → ~0 while cruising** (cpp:2315) | **ON but self-faded to ~0 during locomotion**; `height_lift_knee=0` → hip2 only |
| 6 | Cruse rules 1/2/3 (hip2±, knee∓) (cpp:3324–3339) | `cruse_gain≠0` | **OFF** (0) |
| 7 | Cruse rule 5 load press (cpp:3343–3350) | `cruse_rule5_gain≠0` | **OFF** (0) |
| 8 | **Stance-lift press** | `y[2] += press·0.5`; `y[1] += press·0.25·0.5 = 0.125` (cpp:3355–3389) | `!warmup && !in_swing_[leg]` (detector §3); `press ≡ 1` because **stance_release_frac=0** (release branch cpp:3364–3384 inert) | **ON** — knee+ = tuck-press body up on planted foot; withdrawn when leg declared swinging |
| 9 | Tibia-plumb hip2 (cpp:3397–3401) | `tibia_plumb_gain≠0` | **OFF** (0) |
| 10 | Swing tuck hip2/knee (cpp:3407–3415) | gains ≠0 && `have_contact_` | **OFF** (0, and contact unwired) |
| 11 | **Exploration noise** | `y[j] += N(0, σ)`, `σ = 0.05·(1+0)·explore_mult + pe·0.4` (cpp:3421–3426); `explore_mult = max(0, 1 − commit_amt)`, `commit_amt = 1.0·commit_boost` (cpp:2774, 2797) | `!warmup && σ>0` | **ON**, but **driven to 0 at full progress-commit** (explore_floor default 0, hpp:1396) |
| 12 | **Kuramoto coupling (knee)** | `y[2] += (1−pe)·1.55·Σ_{j≠i} sin((φ_j−φ_i)−(P_j−P_i))/3` (cpp:3484–3518); uniform weights (couple_prec 0) | `!warmup && coupling_eff>0` (no fade configured) | **ON** |
| 13 | fwd-resonance knee bias (cpp:3526–3538) | `fwd_resonance_gain≠0` | **OFF** (0) |
| 14 | **Power stroke + steering (hip1)** | `y[0] += (1−pe)·amp·sin(L.phase + (−2.85))` with `amp = sgn·(1.2·lever_stroke_mult·fwd·load_gate + side·steer_eff)` (cpp:3544–3625). `steer_eff = 0(steer) + nav_term − 0(head_term) + hold_steer`; **`hold_steer = 7.0·(−heading_bearing_) + 0.3·(−yaw_rate_ema_) + 0(trim)`** (cpp:3584–3588); `heading_bearing_` = dead-reckoned yaw integral, clamp ±4 (cpp:1160–1161); `lever_stroke_mult = 1 + 0.30·commit_amt` (cpp:2861–2862, kCommitStrokeFrac hpp:1418); `fwd=1`, `load_gate=1` (stroke_load 0); `nav_on` requires `|tc_x|+|tc_y|>0.05` (cpp:3558) — corridor publishes no target ⇒ **nav_gain=5.0 is latent, PD-hold active** | gains ≠ 0 | **ON** — the single imposed rhythmic drive, hip1 only |
| 15 | Propulsion-balance hip1 boost (cpp:3635–3643) | gain>0 | **OFF** (0) |
| 16 | rhythm_gains coherent drive (cpp:3651–3665) | array non-empty | **OFF** (empty) |
| 17 | **Balance reflex (hip2)** | `y[1] += (−0.5)·(front·tilt_pitch + left·tilt_roll)` (cpp:3797–3801) | `balance_gain≠0` — **NOT warmup-gated** | **ON** |
| 18 | Cell-only blocks (nav/cog/boredom) (cpp:3674–3793) | `n_legs==1` | **OFF** (n_legs=4) |
| 19 | **Panic push (hip2+, knee+)** | `y[1] += drive; y[2] += drive`, `drive = pe·1.2·(0.5+0.5·sin(panic_phase+leg·π/2))` (cpp:3809–3820) | `pe>0.001` (distress hysteresis 0.5/0.25, cpp:2413–2421) | **armed, normally 0** |
| 20 | **Final clamp** | `y[j] = clamp(y[j], −1, 1)` (cpp:3853; cmd_squash=0 so hard clip, not tanh — cpp:3850) | always | **ON — the ONLY output clamp**; the actuator sees a hard clip while the HK Jacobian assumes tanh (hpp:840–847); hip1 pre-clamp historically ~56% clip duty (hpp:284–292) |
| 21 | Publish `ActionOut.accel = y[j]` per joint (cpp:3856–3863); bookkeeping `prev_x/prev_phi_ctx/prev_prev_y/prev_y` (cpp:3868–3872) | always | **ON** |
| 22 | Controller symmetry pull (cpp:3882–3909) | gain>0 | **OFF** (0) |

Module-level pre-loop machinery live this config: progress→commit boost (cpp:2596–2757; window 180 / rise 240 / decay 90 ticks, threshold fwd_ema>0.030; commit_prec inert — gain 0 and no intent topic), adaptive gait_phase crystallization at 0.001 (cpp:2870–2887), **(1+1) coordination search** `coord_reward_drive=0.3, mode 1` = reward-free fitness `coherence·activity/(1+tle) − 0.3·wobble` every 240 ticks, probe σ scaled by `explore_mult` (cpp:2895–2981), stroke-load gate self-guarded to 1 (cpp:3027). stuck-explore, forward-flow, amp-seek, support-select, homeostat leak: all **OFF** (0).

---

## 2. THE PHASE SYSTEM

**L.phase update (every tick, ungated pre-pass, cpp:2429–2536):** `pj = m−1 = knee` (phase_joint −1, cpp:2449). `kp = x[3·pj]` (knee pos), `kd = x[3·pj+2]` (raw per-tick knee delta). `knee_ema` EMA α=0.01 (cpp:2452). **`L.phase = atan2(kd·15, kp − knee_ema)`** (cpp:2461, 2475, 2492; kPhaseVelScale hpp:1254). It is a *direct readout*, not an integrator — **no frequency state, no adaptation**. `phase_vel_smooth`/`phase_sym_smooth` both 0 deployed → raw arms (cpp:2456–2474 inert).

**Backwards running:** `phase_retro_diag_` EMA of (Δφ<0) computed cpp:2482–2491. Documented measurement: **phase_retro = 0.666 — L.phase runs backwards two ticks in three** because the atan2 y-arm is a raw per-tick delta (a high-pass) whose noise exceeds the x-arm near zero crossings (hpp:1255–1265). The one-arm filter fix was RETRACTED (net disp −57%, hpp:1268–1275); both repair knobs are off here. So the Kuramoto term and the stroke both ride jitter, per the code's own comment "the coupling is chasing noise" (cpp:2476–2481).

**Kuramoto coupling toward gait_phase:** it never writes L.phase — it biases the **knee command** by `1.55·mean_j sin((φ_j−φ_i)−(P_j−P_i))` (cpp:3484–3518); entrainment closes **through the body** (knee command → joint moves → next tick's atan2). Offsets `P` start trot `[0,π,π,0]`, are crystallized toward the measured emergent offsets at 0.001/tick with leg 0 pinned (cpp:2870–2887), and are probed/reverted by the (1+1) search (cpp:2959–2980).

**CPG entrainment chain (all live):**
`reality.motor_leg.* → BodyRhythmTracker` (hip1-based, leg_signs [1,−1,−1,1], init period 70; φ_body is a pure integrator softly pulled to 0 at up-crossings, ω low-passed — BodyRhythmTracker.cpp:256–284) → publishes **`rhythm.body.gait` = [cosφ, sinφ, ω rad/tick]** → `CPGOscillator` (period 70, entrained: freq gain 0.02, phase gain 0.05, period clamped 48–120, amplitude 0 = pure clock) → publishes **`rhythm.cpg.body` = [cosφ, sinφ]** → MotorEPMv2 `handle_cpg_phase` sets `cpg_phase_` (cpp:1060–1066) → **cpg_embed** context `[cosφ,sinφ]` enters pre-tanh via Cphi/Cvel (cpp:3255–3258) and at learn time via `prev_phi_ctx` (cpp:3084–3085, 3869). KeyframeGait consumes the same `rhythm.cpg.body` to bin postures (16 bins) and publish `objective.posture.*` back into the learning retarget (§1). Note: MotorEPM's `rhythm_topic` (body ω for the homeostat leak) is NOT wired in this config; leak is off anyway.

**Every consumer of L.phase, deployed status:**
1. **Power stroke** `sin(L.phase − 2.85)` (cpp:3623–3625) — ON (stroke_phase_src=0 ⇒ `step_phase` branch never selected; `update_step_phase` cpp:1345–1510 never even runs, gate cpp:3020).
2. **Kuramoto coupling** (cpp:3493–3506) — ON.
3. **Amplitude homeostat**: amp = |(vx,vy)| of the same phase vector → `amp_ema`, `amp_gain` integrator toward amp_target 0.4 at 0.01 (cpp:2504–2532) — ON.
4. **Adaptive-coordination crystallization**: emergent offsets `legs[i].phase − legs[0].phase` (cpp:2876) — ON.
5. **Coordination-search fitness** via `gait_coherence()` = Kuramoto order parameter over `(L.phase − gait_phase)` (cpp:2938, 3956–3967) — ON.
6. Diagnostics: `couple_R_diag` (cpp:3477–3482), inter-leg PLV (cpp:3438–3466), `phase_freq/retro` (cpp:2483–2490), `legphase_agree` `cos(L.phase)>0` vs in_swing_ (cpp:1938–1949) — ON, report-only.
7. Prop-credit `sin(L.phase+stroke_phase)` (cpp:3637) — OFF (gain 0). intent_rhythm bin (cpp:2695) — OFF. Resonance feedback (cpp:3528) — OFF. rhythm_gains fallback (cpp:3656) — OFF. gait_align diag — OFF.

**Stroke timing vs touchdown: nothing sets it.** `stroke_phase = −2.85` offsets the stroke from the knee's own flexion cycle, not from ground contact. The code's own record: three unforced clocks — stroke/knee 22–24 ticks, true step 26–30, foot-height detector 12–15 — beating at ~2–2.5 s, with the stroke's positive half occupying 0.512 of stance vs 0.513 of swing, i.e. **push direction statistically independent of contact** (hpp:663–678, 626–632). The touchdown-referenced step clock (`stroke_phase_src`, soft-PLL, cpp:1451–1494) exists exactly for this and is OFF here.

---

## 3. THE STANCE/SWING DETECTOR (`in_swing_`)

**Exact deployed test** (update_cruse_state, cpp:1854–1957; runs because stance_lift_gain≠0, cpp:3015): `contact_topic` unset ⇒ true-contact path (cpp:1868–1886) never taken. `swing_hyst_frac`=0 ⇒ legacy branch:

```
foot_y_ema[i] = 0.98·foot_y_ema[i] + 0.02·foot_y[i]        (cpp:1890, α = kFootYEmaAlpha ~50 ticks)
in_swing_[i]  = (foot_y[i] > foot_y_ema[i])                 (cpp:1903 — no deadband)
```

**Critical signal fact:** `feet_topic = reality.proprio.feet_y_gravity_cmd_imu` = per-leg foot height by **FK from the COMMANDED servo angles** on a gyro-fused gravity estimate (picrawler_body.gd:2640–2643, 5867–5878). The detector therefore thresholds **the brain's own commanded foot height against its own moving average** — hardware-legal, but a command-phase gate, not a contact sensor; it flips to "swing" only once the *commanded* foot has already risen above its ~50-tick mean.

**Known failure modes (per the code's own comments):**
- No deadband ⇒ splits ~50/50 by construction — "a PHASE test, not a contact test" (hpp:449–456).
- **Positive feedback with any consumer that moves the foot** (stance_lift here): bias raises foot → above EMA → declared swing → bias removed → foot drops → declared stance → a relaxation oscillator at the EMA's ~50-tick timescale competing with the ~70-tick stride (hpp:452–456).
- Measured chatter ~2× per real step: 12–15 ticks vs true 26–30-tick step (hpp:668–670, 793–795).
- Vs ground truth: measured **40.3% "swinging" while the feet were genuinely down 99.3%** of the time (cpp:1273–1278).
- Wiring true contact instead was separately refuted (net_z 3.76→2.37 — "the consumer wanted gait PHASE, not contact", hpp:499–504).

**Consumers of in_swing_:** (a) **stance_lift gate** `!in_swing_[leg]` (cpp:3355–3356) + its swing-branch release-arming (cpp:3390–3392) — ON; (b) Cruse rules 1/2/3 (cpp:3324–3339) — OFF; (c) Rule 5 (cpp:3343–3350) — OFF; (d) `ticks_since_plant_` (Rule 2 input, cpp:1909) — maintained, unconsumed; (e) diagnostics: `swing_frac_ema` (cpp:1917), `phase_agree`/`legphase_agree` (cpp:1923–1949), gait raster bits 8–11 and align-diag bouts (both OFF).

---

## 4. THE LIFT PATHWAY — what actually lifts a foot, and why it lags the stroke reversal

**Vertical authority (hip2/knee) in this config comes from exactly these live terms:**
1. **The learned HK oscillation with CPG-phase conditioning** — `mg·ag·tanh(C·x + Cphi·[cosφ_cpg,sinφ_cpg] + h)` (cpp:3237–3260). `Cphi` is the only *phase-timed* vertical drive in the whole stack: it is trained to pull each joint toward KeyframeGait's 16-bin average posture at the current CPG phase (cpp:3161–3168). There is **no imposed swing-lift oscillator** (rhythm_gains empty, Cruse 0, swing_tuck 0).
2. **Withdrawal of the stance-lift press** — while declared planted the knee carries +0.5 and hip2 +0.125 of press-down/tuck-press (cpp:3385–3389); when the detector flips to swing these are removed, releasing the leg upward.
3. Secondary: postural spring toward knee-tuck rest 0.7 (cpp:3288–3291), Kuramoto knee bias (phase-corrective, →0 when locked, cpp:3518), balance hip2 (cpp:3800), height-homeostat hip2 (faded ~0 while moving, cpp:3310–3311), noise (cpp:3425).

**The stroke, by contrast, is direct:** `y[0] += amp·sin(L.phase − 2.85)` reverses the instant the knee-derived phase crosses the zero of the sinusoid — a pure function of the *measured* phase, ungated by contact or load (stroke_phase_src=0, stroke_load_gain=0).

**Why liftoff lags the commanded stroke reversal by 7–9 ticks (the measured pathology, hpp:1083–1094):**
- **The lift is causally downstream through the body; the reversal is not.** Stroke reversal is command-instant; liftoff requires the Cphi/C·x knee–hip2 cycle to physically unload the foot through servo slew (measured 2–3 ticks) plus weight transfer to the other legs. Nothing synchronizes the two: the stroke and the step were measured as independent clocks (hpp:663–678).
- **The stance press actively fights the first part of every lift, and its own gate guarantees the delay.** The press (+0.5 knee, +0.125 hip2) stays applied until `in_swing_` flips — and `in_swing_` flips only after the **commanded** foot height has risen above its own ~50-tick EMA (§3). So the lift command must already be well underway, against the press, before the press is removed. This is the exact "pressed window" flbrake.py measured: commanded hip1 delta flips sign 7–9 ticks before liftoff while the leg pays −0.004…−0.015 g/tick braking shear the whole time (hpp:1083–1094). The purpose-built countermeasure — `stance_release_frac`, which fades the press from the tick the leg's own commanded hip1 delta reverses (cpp:3357–3384) — **is 0 in this config**, so `press ≡ 1` for the entire stance bout.
- **Opposing static biases raise the lift threshold:** the postural spring (0.7 toward tuck) and the press mean the Cphi feed-forward — which is inside the tanh and bounded by mg·ag — must overcome a standing offset before the foot moves; hip2 and knee were separately measured to agree in sign only 50.8% of the time (hpp:1055–1059), so roughly half of lift attempts are partially self-cancelling.
- **Aggravators:** hip1 pre-clamp requests run well past ±1 (stroke 1.2 with HK on top of steer 7·bearing; ~56% historical clip duty, hpp:284–292), so the *executed* stroke reversal also lags the commanded sinusoid; and L.phase itself is retrograde 2/3 of ticks (hpp:1255–1261), jittering the reversal instant the whole chain is referenced to.

**Bottom line:** liftoff is produced by the learned Cphi/HK knee–hip2 cycle plus release of the stance press; both are gated or lagged by signals derived from the command stream itself (the CPG phase via the keyframe average, and the commanded-FK foot height via a 50-tick EMA), while the hip1 stroke reversal is a direct, contact-blind function of the knee's instantaneous phase. The 7–9-tick lead of stroke reversal over liftoff is the structural gap between those two chains, and the only two levers built to close it — `stroke_phase_src` (touchdown-referenced stroke clock) and `stance_release_frac` (reversal-triggered press fade) — are both at 0 in this configuration.