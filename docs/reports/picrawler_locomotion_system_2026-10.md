# A quadruped that times its legs by what its feet feel

The PiCrawler's locomotion system as of 2026-10-13: what it is, why it is built that way, what
it does in simulation and on the robot, and where it stops. Written for someone who wants to
propose an improvement or a loop on top, and needs the foundation first.

Status: the system described here is a working signal, not a finding. Every number carries its
sample size; the section "Claims we are not making" says what has not been shown. Companion
records: the gait audit that motivated it
([picrawler_gait_mechanism_audit_2026-10-12.md](picrawler_gait_mechanism_audit_2026-10-12.md)),
and the lever ledger's entries dated 2026-10-12 and 2026-10-13
([picrawler_lever_ledger.md](picrawler_lever_ledger.md)), which hold every measurement cited.

## Executive summary

Can a small four-legged robot, built from hobby servos and driven by a brain that is never
told how to walk, produce a clean, repeating stride? For most of this project the answer was
no. The robot walked, but every joint reversed direction several times a second, its feet
touched and lifted a hundred times a minute, and one stride in five was clean. An audit found
why: the gait was five hand-written terms added together and timed by a clock read off the
very knee those terms were driving, so the clock ran backwards two ticks in three, and nothing
in the controller knew whether a foot was on the ground.

The system described here replaces that clock with something a walking animal has and the old
gait did not: each leg keeps its own sense of where it is in its step, and that sense is held
back by the weight the foot is carrying. A loaded foot stays down and keeps pushing; an
unloaded foot swings. No leg is told what the others are doing. They coordinate through the
ground, because when one foot lifts, the others feel the extra weight. This is a rule published
by Owaki and Ishiguro in 2013, and it does on this robot what it did on theirs: it turns a
shuffle into steps.

Around that rule sit a few reflexes, each of which answers one measured failure. The body
withholds its push, and its turn, while it is off level, because every fall turned out to be
one planted leg sweeping its hip through its full range. Each planted leg slowly adjusts its
own length to hold its corner of the body level, which also absorbs the fact that the four
feet's force sensors differ fifteen-fold in sensitivity. And a learned layer underneath, which
predicts what each leg will feel next and acts to keep its own body both lively and
predictable, is left in place and unchanged.

In simulation, on the same body and the same floor, the number of steps per run rose from
three to about fifty, clean strides from none to two in five, hip reversals per stride fell by
two and a half times, and transport on a course with a ramp and angled walls rose by half over
the old gait with no falls, in six seeds each. On the robot, two sim-to-real facts had to be
met before the gait appeared: the force sensors had to be normalised per foot, and the wall
reflex had to be pulled in to 30 cm so it would stop steering. After that the operator's
judgement of the best configuration was "deliberate foot placement and stepping". What remains
on the robot is roughness in the servos, and it is commanded: the brain's raw targets reverse
about forty times a stride, three times the simulation's, and the two inputs the robot has that
the simulation does not, the foot-contact flag and the gyro, are the suspects now under test.

## 1. The robot, the premise, and the words used below

**The robot.** A SunFounder PiCrawler: four legs, each with three servos (a hip that yaws the
leg forward and back, a femur joint that raises and lowers it, a knee), a Raspberry Pi, a
nine-axis inertial unit, a forward ultrasonic rangefinder, a belly rangefinder, and a force
sensor (an FSR) under each foot. The servos accept a position and report nothing back. A
simulated twin in Godot carries the measured geometry and masses; most of the measurements
below were made there, at the robot's own servo speed, and then on the robot.

**The premise.** This project's brains have no reward signal. A module holds predictions about
its own body and acts to make them true. A behaviour is never written as a trajectory; it is
written as the error it reduces, and the gate that says when that error applies. The one
through-line of the project's record is that terms which let the body discover a motion get
promoted, and terms which dictate one get refuted.

**Words.**
- *Phase*: where a leg is in its step, a number that runs around a circle once per stride.
  The first half is *swing* (the foot in the air, moving forward), the second half is *stance*
  (the foot down, pushing back).
- *Load share*: the fraction of the body's weight one foot is carrying, from its force sensor,
  relative to the other feet.
- *Gate*: a state that switches a term on, off, or down. The gate is the design; the
  magnitude is tuning.
- *Reflex*: a fixed-gain term that descends a measured error (level, height, posture).
- *Integrator*: a term that accumulates an error slowly into an offset, so a persistent
  condition (a slope, a weak sensor) becomes a persistent correction.
- *The learned layer*: the per-leg forward model and controller that are trained online.
- *Commanded forward kinematics (FK)*: where the feet would be if the servos were exactly
  where they were told to be, computed from the commands. Used because the servos cannot
  report where they are.

## 2. The system, layer by layer

Think of it as five layers, from the ground up. Each one reads sensors the robot has and feeds
the layer above through a small, named socket.

```
   navigation loops  (proximity memory -> avoid -> intent)        : a bearing and a speed
          |  heading hold, stuck sense                            : steer, go/stop
   ------ the motor layer ----------------------------------------------------------
   stabilisers   level gate on push and steer | per-leg corner integrator | posture spring
   the phase     one phase per leg, held by that foot's load      : stroke, stance, swing
   learned       per-leg forward model + controller (homeokinesis): participation
   ------ the body -------------------------------------------------------------------
   sensors       foot force (4) | attitude, yaw rate (IMU) | commanded joints | ranges
   actuators     12 position servos, slew-limited, lag-filtered
```

### 2.1 The phase: one clock per leg, held by the foot

Each leg owns a phase that advances at a fixed rate and is slowed in proportion to the load
its foot carries:

    rate of phase = ω · (1 − 2 · N · cos φ)

where φ is the phase, ω the free rate (one stride per 70 ticks, about 1.4 s at 50 Hz), and N
that foot's share of the body's load. The product N·cos φ is largest at the end of stance, so
a loaded foot is held there and does not lift; an unloaded foot runs free. A foot carrying more
than half the body's weight stops at the end of its push and waits. Nothing couples the four
phases directly. When one foot lifts, its weight moves to the others, which slows them, which
is the coordination: through the ground, not through a prescribed pattern.

Two details matter on the robot. The load share is formed after each foot's force is divided
by its own running reference (its force while touching, averaged over about ten seconds), so a
stiff sensor and a weak one carry equal weight; without this the robot's rear-left sensor,
which reads fifteen to twenty times the rear-right's at the same stance, made the rule pin one
leg and free its diagonal partner. And the contact flag that defines "touching" is the
sensor's own threshold, which flickers; the phase, not the flag, is what the motor layer's
gated terms read.

The module publishes, per leg, the phase and the share. It learns nothing; its three constants
are the free rate, the hold strength, and a noise floor.

### 2.2 What the phase drives

- **The hip stroke.** The hip is driven by a sinusoid of the phase: fully forward at
  touchdown, sweeping back through stance, returning through swing. Amplitude 0.7 of the
  servo's range (the full range was measured to tip the body, see §4.2). Left and right legs
  mirror.
- **Stance and swing.** The phase's half-cycle is the stance/swing state every gated term
  reads. Planted legs receive a constant press on the knee and femur that holds the body up;
  swing legs receive a smooth lift shaped by the phase (zero at lift-off and touchdown, peak
  mid-swing) and a fold of the femur and shank.
- **Steering.** The navigation layer's bearing error, through a proportional-derivative hold,
  becomes a left/right differential on the stroke amplitude, so the body turns by pushing
  harder on one side.

### 2.3 The stabilisers

- **The level gate.** The stroke amplitude, both its push and its steer differential, is
  multiplied by 1 − 4·|tilt|, where |tilt| is the body's lean from level (the fused attitude
  from the inertial unit, smoothed over ten ticks on the robot). A body that is off level
  withholds its push and its turn and lets the posture spring and the planted legs recover it;
  level, the stroke is untouched. Against a constant shorter stroke that averages to the same
  amplitude, the gate keeps twice the steps.
- **The per-leg corner integrator.** Each planted leg integrates its corner's error, the
  front/left projection of pitch and roll, into its own offset on femur and knee: a low corner
  presses more, a high corner less. Bounded, frozen when the body is not upright, zeroed after
  a fall, applied only while the leg is planted so the swing lifts from the shared rest. On a
  side slope the uphill legs shorten and the downhill legs lengthen; on flat ground it
  removes the body's roll entirely and halves a standing pitch that the physics creates.
- **The posture spring and the stance press** are the previous gait's, unchanged.
- **The swing clearance** (simulation so far). A swing foot's commanded height under gravity
  is compared with the planted feet's plane near mid-swing; a leg whose command would not
  clear by 40 mm scales its lift up next time. It needs the commanded FK, which the robot host
  now publishes but has not yet run with.

### 2.4 The learned layer

Under all of this, each leg still has its forward model (what the next sensed state will be,
given the command) and a controller trained by the homeokinetic rule, which keeps the body
lively but predictable, plus a phase-conditioned bias trained toward a learned posture map. On
the old gait, removing all of this cost nothing and even helped; on this gait it supplies leg
participation and costs some steps on flat ground. It is kept because a closed loop that works
is foundation, and because the terrain levers planned next (§6) are the first that give it a
job: discovering how each leg's length moves its corner.

### 2.5 Above: the navigation loops

Unchanged from the room work of 2026-10-11: a proximity memory built from the ultrasonic and
odometry, an avoid loop that turns a bearing away from the nearest surface, an intent module
that turns the bearing into the heading hold's goal and a speed, and a stuck sense. One
parameter changed for the forward-walking measurements: the proximity memory's range, from
1.0 m to 0.3 m, because at 1.0 m the ultrasonic's floor return kept the robot turning. The
robot turns in place well, so the reflex has time at 30 cm.

### 2.6 What each layer reads, and what it cannot

| signal | source | read by |
|---|---|---|
| foot force, foot contact | FSR under each toe | the phase (shares); the clearance; the stuck sense |
| attitude (pitch, roll), uprightness | inertial unit, fused | level gate; corner integrator; homeostat gates |
| yaw rate | gyro | heading hold (damping) |
| joints | the servo model of the command (no feedback exists) | learned layer; posture spring |
| commanded foot height under gravity | FK of the command, rotated by the attitude | clearance (sim); the old swing detector |
| ranges | ultrasonic (forward), ToF (belly) | navigation loops; height homeostat (off) |

The servos report nothing, so the brain never sees how far a joint sagged under load. That
limit shaped two results in §4.

## 3. Results in simulation

All arena runs: the measured body with the FSR feet, the robot's servo speed, difficulty 0.3,
six seeds, 4 500 or 6 000 ticks, per-tick logs. "Base" is the previous gait at its promoted
posture. Reversals are of the achieved joint per 70-tick stride unless stated.

### 3.1 The phase alone, and its control

| arm (6 000 ticks) | net distance | steps / run | clean strides | hip reversals / stride | falls |
|---|---|---|---|---|---|
| base | 4.65 ± 0.43 m | 2.8 | 0 % | 11.1 | 0 |
| the phase (with lift, trot coupling off) | 6.26 ± 1.69 m | 47.7 | 38 % | 4.4 | 1.17 |
| the same with the load coupling removed (a free clock) | 0.64 ± 0.67 m | 35 | 22 % | 4.7 | 6.5 |

The free clock steps as often and goes nowhere; holding a loaded leg is the mechanism. The
swing bouts went from 0.07 s to 0.24 s and stance from 40 s at a stretch to 1.5 s. The cost was
about one fall per run and four times the tilt variance.

### 3.2 Removing the falls

Every fall on the phase was one planted leg sweeping its hip 1.3 to 1.9 rad while the body
slid sideways; none had two feet in the air. Three levers on that, 4 500 ticks:

| arm | net distance | straightness | falls | tilt sd | steps |
|---|---|---|---|---|---|
| the phase, stroke 1.2 | 4.94 | 0.33 | 1.00 | 0.242 | 37 |
| stroke 0.7 | 7.60 | 0.55 | 0.17 | 0.101 | 41 |
| level gate, gain 4 (push only) | 6.79 | 0.50 | 0.17 | 0.079 | 48 |
| the constant stroke that matches the gate's mean (0.5) | 6.27 | 0.55 | 0.00 | 0.068 | 23 |
| corner reflex, proportional, planted legs | null at every dose; wrong sign degrades | | | | |

In the walled room, with the avoid loop commanding turns, the gate on the push alone did
nothing (falls 9 per 12 000 ticks) because the turn rides the same hip sweep; gating push and
steer together brought it to 2.5, and with the 0.7 stroke to 0.33, five seeds of six at zero,
with coverage above the old gait's.

### 3.3 Terrain

Corridor with a ramp and 30° walls, 6 000 ticks, six seeds, zero falls in every arm:

| arm | transport (net z) | straightness |
|---|---|---|
| base | 4.23 ± 0.32 m | 0.26 |
| the candidate (phase, gate on both, stroke 0.7) | 5.16 ± 0.47 m | 0.35 |
| + corner integrator | 5.72 ± 0.65 m | 0.38 |
| + swing clearance 40 mm | 6.30 ± 0.70 m | 0.43 |

### 3.4 By the true contact sensor

The per-leg foot height used through most of the campaign is the body's shank-angle
approximation, and the harness's step count uses it too; both under-report airborne time, and
they misled this work for a day. By the physics contact flag, now in the log per leg, the
candidate's legs are airborne 31 / 22 / 32 / 14 % of the time (front-left, front-right,
rear-left, rear-right) against the base's 23 / 12 / 26 / 21, with fewer and longer swings
(0.2 to 0.5 s against the base's 0.07 to 0.20). The right rear lifts least in every arm; its
cause is open, and the geometry has been cleared.

## 4. Results on the robot

Four floor runs on 2026-10-13 afternoon, then four more that evening, each two to five
minutes, read from the dash's feed, the brain's per-tick action dump and, for the evening
runs, the modules' own state.

### 4.1 The first runs: one pair works

The operator saw one pair of legs doing the work and the body circling. The feed explained
both. The force shares the rule saw were rear-left 0.51 to 0.55 and rear-right 0.03 to 0.07 in
every run, because the four sensors share one counts-to-grams curve and the rear-left reads
about three times the rear-right's counts at the same stance; a share above a half is held,
a share near zero runs free. And the avoid loop was steering 93 to 97 % of the time: the
ultrasonic read about 1.0 m, inside its 1.0 m range.

### 4.2 The second runs: the winner

With per-foot normalisation and the 0.3 m reflex range, the shares read 0.20 to 0.30 on all
four feet, and the operator's judgements were, in order: the old gait "attempting to walk
straight, no turning behaviour away from walls"; the normalised phase "better stepping but a
lot of small servo reversals"; the same with the level gate smoothed over ten ticks "much
improved and deliberate foot placement and stepping"; with the corner integrator "noisier than
3 but ended up walking straight after some flailing". The smoothed gate is the configuration
carried forward.

### 4.3 What remains: a commanded roughness

| | raw target reversals / stride | pulse on the line, reversals / stride | median step on the line |
|---|---|---|---|
| the winner (robot) | 43 (40 to 49 across servos) | 16 (12 to 20) | 7 µs |
| the old gait (robot) | 35 (26 to 48) | 13 (10 to 20) | 10 µs |
| the candidate (simulation, same read) | 13 (6 to 20 by joint) | | |

The brain asks for about forty reversals a stride per servo, three times the simulation's
under the same configuration and regardless of configuration; the slew and lag in the servo
driver reduce what reaches the servo to about sixteen sub-degree dithers a stride, and the
gear train's backlash turns those into the trembling the operator sees. The roughness is
commanded, so the fix belongs in the command. The two inputs the robot has and the simulation
lacks are the foot-contact flag, which gated the swing fold directly and transitions 70 to
216 times a minute per foot, and the gyro in the heading damping. Arms that remove each are
built and waiting for the next run.

## 5. Why it behaves as it does

**Why the load rule works where the clock did not.** The old timing signal was read off the
knee the terms drove, so each correction fed the next. The phase here is an integrator that
only the foot's load slows; it cannot run backwards unless a foot is suddenly loaded, and the
stroke, the press and the lift are all smooth functions of it. The free-clock control shows
the other half: timing alone produces stepping without transport; the coupling through the
ground is what turns stepping into walking.

**Why the falls were the sweep, and why a corner reflex could not catch them.** A sprawled
leg's full hip sweep pushes the body sideways; on the old jittery clock the sweep never
completed, on the monotone phase it completed every stride. A proportional press on the low
corners acts on the effect while the cause continues, and the leg doing the shoving is itself
planted, so it was pressed harder too: null at every dose. Withholding the push while off
level removes the cause; the integrator, slow and bounded, handles what persists.

**Why the brain cannot see sag.** Under honest inputs the joint state is the servo model of
the command, because the real servos report nothing. Two levers built on seeing sag were
therefore dead on measurement, and the commanded FK, which does not see sag either, measures
exactly that sag when used as a planted-leg corner error, and ran the integrator the wrong way.
The attitude is the right corner source for planted legs; the commanded FK is the right source
for a swing leg's clearance.

**Why the simulation did not show the robot's two problems.** Its force shares come from
physics and read 0.25 each; its walls are mostly beyond a metre; its gyro is noiseless and its
contact flag clean. Each of the robot's problems was a sensor fact, and each fix was to adapt
to the sensor rather than calibrate it.

## 6. Limits, and what comes next

Not yet shown: anything at power (every number is six fixed seeds, one body, one gym at a
time), stability past 12 000 ticks, the robot on an obstacle course under either gait, the
swing clearance on hardware, and which gait the four phases settle into (the per-leg phases
are logged; the pairwise read is not yet done). The right rear's shorter swing is unexplained.
The learned layer's role is smaller than the project's premise asks.

Next, in order:
1. The three arms on the robot's winner that remove the contact flag and the gyro from the
   command, read by reversals per joint against the simulation's figures. Then, if needed,
   contact hysteresis at the host.
2. The robot on the same obstacle course under the old gait and the new, falls and contact
   flicker counted. The operator's concern is that the old gait, for all its noise, handled
   uneven terrain; that is the bar.
3. The terrain brief from the operator's simulation observations: a leg that reaches down to
   contact before it pushes, a press and a push gated by the corner's own error, and a swing
   length the body adapts rather than a constant. The first supplies the state the other two
   gate on.
4. Long runs, varied worlds, and the perturbation test (drop a foot sensor mid-run, watch
   the other three reorganise), which this design makes natural and has not yet been run.

## 7. Claims we are not making

- That the robot walks "by active inference". The timing rule and the reflexes are
  hand-written, with constants; the learned layer rides along. What is true is that every term
  descends a measured error of the body's own, gated by the state that makes it valid.
- That the gait is efficient. Steps are clean and strides repeat; current and effort on the
  robot have not been compared against the old gait.
- That six seeds are a finding. They are enough to choose a direction.
- That the simulation's terrain result transfers. The sim's foot contact is the open
  sim-to-real gap, and the obstacle course on the robot has not been run.
- That the roughness is solved. It is located and measured; the levers are in test.

## Appendix: where things live

For the reader who will build on this. Names are the repository's.

| what | where |
|---|---|
| the phase module | `cpp_core/src/ogma/modules/LoadPhaseGait.cpp` (parameters: `period_ticks`, `sigma_ratio`, `load_norm_tau`) |
| the motor layer's sockets | `MotorEPMv2`: `leg_phase_*` (stroke, stance gate, lift), `stroke_level_gain/_steer/_smooth` (the level gate), `level_int_gain/_planted_only` (the corner integrator), `swing_clear_*` (the clearance), `swing_gate_src` (the contact-gated terms on the phase) |
| the robot host's commanded FK | `pi_host/include/ogma/hw/BrainInputs.hpp`, `ogma_host --cmd-fk`, mapped by the dash from the config's `OGMA_PICRAWLER_CMD_FK_SOURCE` |
| the configurations | `godot_host/project/addons/ami_ogma/configs/…hip2p030__lpg*.json` (sim) and `…s3a_stuckinst__hip2p030__lpg*.json` (robot stack); the launcher shows short names, the long form is each file's description |
| the instruments | `scripts_tools/arenaavg.py`, `seedavg.py`, `roomavg.py` (harnesses); `stepnoise.py`, `thrash.py` (reversals); the per-leg physics contact `con` and the modules' state in the body log and the dash's brain diag |
| the record | the ledger's 2026-10-12 and 2026-10-13 entries; the audit of 2026-10-12 |

## Addendum, 2026-10-31 — from stepping to walking on the robot

The design above stands. Four levers, each one measured against the configuration before it on the robot, took the
gait from the 2026-10-13 winner to a steady diagonal pattern that holds for six minutes and turns away from obstacles
placed in its path. Each was a sensor fact rather than a tuning; the ledger's entries from 2026-10-26 to 2026-10-31
carry the evidence, and the Claude Docs page "PiCrawler locomotion system" carries the readable account.

| Lever (config suffix) | What the robot did | What it was | Verdict |
|---|---|---|---|
| learned-controller learning off (`_hk0`) | one leg at a time slid into saturated chatter after minutes; the body walked backward | the per-leg controller winding up on a servo that reports nothing | `WORKING`, 1 run; sim 24 000 ticks reproduces it |
| contact-only load shares (`_cs`) | the legs never locked (diagonal lock ~0.3 vs the sim's 0.88), cycling 30–45 % apart | force shares through one FSR curve shared by four sensors whose curves differ 4× | `WORKING`, 2 runs; sim PLV 0.68 → 0.76 |
| pinged-range floor on the proximity memory (`_pf`) | a right-hand circle at 4–6°/s | the stride odometry over-reads 2–3× on slip and dragged far sonar points into the body (replay: 1343 → 0 avoid ticks) | `WORKING`, 1 run |
| support-gate allowance fixed at 30 mm (`_sl0`) | swimming late in every run over 3 min | the gate's learner collapsed its allowance at obstacle contact into an absorbing state | `WORKING`, 1 run of 376 s with placed obstacles; sim corridor ties |

The gait as the instruments read it: diagonal phase lock 0.28–0.40 (2026-10-27) → 0.86, 0.94 in the last minute;
touchdown-interval CV 0.49–0.61 → 0.38; saturation per leg 0.3–3 % → ≤ 0.6 %; about 3 cm/s toward a wall by sonar.
Two cautions for a reader of the logs: brain_diag's leg names are the simulation's, and the simulation is mirrored
(its front-left is the physical front-right); and every odometry-derived speed over-reads on slip by 2–3×.

Not yet shown: the six-minute result at power (one run), a progress sense that survives slip, whether the late decay
has a thermal component as well as the learned one, and a simulation that carries the robot's disturbances. The
operator's reading: the robot is near the end of what it can do without a heavier redesign (servos without feedback,
one sonar beam, a pack that sags to 6.5 V in six minutes).
