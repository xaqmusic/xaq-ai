# The picrawler's gait, audited: why the step is noisy, and what to try instead

Status: audit, 2026-10-12. Scope: the motor layer the robot runs today (the `MotorEPMv2`
module under the deployed room config), its documentation, the comments inside it, and the
ledger record that produced it. Companion documents are cited inline. Nothing was run for
this audit; every number below is taken from the ledger entry that measured it.

## Executive summary

The question: the robot walks, but every step is a flicker. Its joint targets reverse
direction every two to five ticks, its feet touch and lift a hundred times a minute on
carpet, and one stride in five is clean. Months of levers have not produced the purposeful,
repeating leg movement the operator wants. Is something about the mechanism itself wrong,
and is there a different approach to try?

The answer from the record is yes on both counts.

**The gait is not learned, and the noise is what the mechanism is made of.** The walking is
produced by five hand-written terms added together: a spring toward a rest pose, a pull
toward a prescribed trot, a sinusoidal push on the hip, a press on whichever legs a detector
calls planted, and fresh random noise every tick. These are summed and then cut off at the
servo's limit. The learned part of the controller, the homeokinetic layer that is the whole
point of the module, was switched off in a twenty-seed test and the robot walked further and
straighter without it. Measured again this month by removing the terms one at a time, every
one of them adds reversals, and removing any one makes the others reverse more. The thrash is
not a term to tune. It is the sum.

Three construction facts make the sum thrash. First, the timing signal every rhythmic term
rides on is read from the knee the terms themselves drive, from its position and its
one-tick velocity, and it runs backwards two ticks in three; the trot coupling therefore
closes a loop on its own output, which is the recipe for a relay oscillation at the servo's
slew limit. Second, the detector that decides which legs are "planted", and so where the
press and the tuck go, does not look at the feet at all. In the simulator it reads the
construction pose rotated by the body's tilt, and the robot host reproduces that exactly, so
on hardware the stance press switches on body rocking. Third, nothing in the stack references
ground contact, although the robot carries foot force sensors and the simulator publishes a
contact flag and a load value every tick. A step is a contact event, and the controller is
blind to it.

This diagnosis is not new. The Playful Machine source analysis (2026-08-02), the motor-system
audit (2026-08-09) and this month's lesion study each reached it, and each was followed by
another lever on the same stack. The module is now 7 300 lines carrying about sixty
gain-guarded levers, most refuted, several load-bearing, and the operator's goal has not
moved: the inter-step interval still has a coefficient of variation near one, which is a
memoryless process, in every configuration ever measured.

**The approach to try is a different timing substrate, built as a separate small module
rather than as lever sixty-one.** Each leg owns a phase that advances at its own rate and is
slowed by the load that leg feels: when the foot is bearing weight the leg stays in its push
and does not lift; when the load leaves, the leg swings. No inter-leg phase is prescribed.
The legs coordinate through the body, because lifting one foot loads the others. This is the
rule Owaki and Ishiguro published in 2013 and 2017 ("Tegotae", roughly "the feel of the
ground answering"), and on a real quadruped it produced walking at low drive and trotting at
high drive with no gait ever specified, which is the operator's stated target. In this
project's language it is the rewrite rule applied cleanly: the error each leg descends is
"I intend to be pushing, and I feel no ground" (or the reverse); the sensory channel that
carries it is the foot force sensor, which the robot already has; the gate is the load. The
learned homeokinetic layer, the posture spring and the reflexes stay underneath, unchanged.

The prerequisite is the simulator's foot. This month's lesion study found that the sim's gait
is a shuffle whose feet barely leave the ground while the robot's flicker, and that the
difference is the foot-ground contact. A timing rule whose coupling runs through contact
cannot be evaluated on a floor that lets a scrubbing foot push. Measuring the robot's foot
friction and compliance and putting them into the body model is the first job, and a cheap
hardware change (a grippy, slightly compliant sole) is the Playful Machine's lesson that the
easy body makes the simple brain work.

## 1. What runs today

The deployed robot stack is the room config with the taller stance (the ledger's 2026-10-12
entries; `the_picrawler_motor_epm_embed_corridor_v3base__…__s3a_stuckinst__hip2p030.json`).
Its motor layer is one module instance driving twelve servos, three per leg, with the
following live terms, in the order the code adds them each tick. Values are the config's.

| order | term | what it does | timed or gated by |
|---|---|---|---|
| 1 | learned controller | a per-leg linear map from the leg's sensed state through a squashing function, gain 3.0, learned by the homeokinetic rule; plus a phase-conditioned bias learned from the keyframe posture error | the central clock's phase |
| 2 | posture spring | pulls every joint toward a rest pose (knee tucked 0.85, femur pressed +0.3), gain 1.08 | always on |
| 3 | stance press | adds knee +0.5 and femur +0.125 on legs the detector calls planted | the swing detector (see below) |
| 4 | swing tuck and rear landing | folds the femur and shank of legs the detector calls swinging; the rear legs get a descent timer | the swing detector |
| 5 | exploration noise | white noise, standard deviation 0.05, redrawn every tick on every joint | always on; damped by progress-commit |
| 6 | trot coupling | pulls each knee toward a phase offset from its neighbours, gain 1.51, offsets [0, π, π, 0] | the per-leg phase (see below) |
| 7 | power stroke | a sinusoid on the hip, amplitude 1.2 times the speed command, carrying the heading hold as a left/right differential | the per-leg phase |
| 8 | clamp | the sum is cut to ±1 | |

Two further learned mechanisms are present with little authority: the motor planner enters
through the objective socket at a weight of 0.025, and the gain evolver holds three gains at
the seeds the 2026-08-27 settle produced.

**The per-leg phase.** Every rhythmic term (6 and 7, the amplitude homeostat, the
coordination fitness) reads a phase computed per leg as the angle of the vector (knee
position minus its running mean, fifteen times the knee's one-tick velocity). The module's
own comments record that this phase runs backwards two ticks in three ("retrograde 0.666"),
and the 2026-10-02 ledger entry traces why it thrashes: the knee reverses, the phase flips
half a turn, the coupling term reverses the knee again. The two smoothing repairs the file
carries are both off, both having regressed distance.

**The swing detector.** The stance press and the tuck gate on a topic named as the foot's
commanded height under gravity. In the simulator that topic is forward kinematics of a servo
target array that brain mode never writes, so it is the construction pose rotated by the
fused attitude: an attitude detector. The 2026-10-02 entry measured it live (the knee target
swept 2.5 rad while the array stayed at zero). The robot host computes the same quantity the
same way, by design ("exactly as P-e computes it: FK of the ZERO pose"), so the hardware
reproduces the defect faithfully. The corrected source (forward kinematics of the slewed
target, the `cmdfk` arm) exists, lifted the feet on every seed at the robot's servo speed,
and is not in the deployed stack. The physics contact flag is subscribed as an instrument
only.

**The central clock.** A 70-tick oscillator entrained to a body-rhythm tracker that has never
locked (phase-locking value 0.02 to 0.10 on every run scored). Its phase conditions the
learned bias and the keyframe map; it does not drive a joint.

## 2. What the record says

The evidence is spread over the ledger (`picrawler_lever_ledger.md`), the gait findings
report, the Playful Machine analysis, the motor-system audit and the repair plan. Gathered:

| finding | measurement | where |
|---|---|---|
| The learned layer has no job on flat ground | removing all controller learning: distance 4.32 → 4.92 m, straightness 0.70 → 0.74, both t > 2.6, n = 20 | ledger 2026-08-02 |
| The thrash is the sum of the imposed terms | base 11–16 reversals per stride per joint (sim); each term alone saturates the slew; removing coupling, stroke or press raises the others' reversals; only the noise lowers them when removed, by a fifth | ledger 2026-10-02, 2026-10-12 |
| On the robot, open loop | 29 reversals per 70-tick stride per servo, 30 % of servo-ticks at the rail | ledger 2026-10-12 |
| The timing signal is jitter | per-leg phase retrograde on 65 % of ticks; smoothing it (three forms), locking it to touchdown, or offsetting it each cost 40–97 % of distance | repair plan P1, P4 arms 1–2 |
| There is no step rhythm | inter-touchdown coefficient of variation 0.94–1.06 in every arm ever measured; one lever moved it to 0.82 at the cost of all transport | ledger 2026-07-27, repair plan P4 |
| There is no inter-leg coordination | time-mean coherence 0.44–0.45 against a random-phase null of 0.45, n = 12 per arm, every arm | ledger 2026-08-03 |
| The hip is clipped half the time | stride-joint clip duty 0.56; the stroke alone exceeds the rail 42 % of its cycle | ledger 2026-08-02 |
| The stance press switches on body rocking | commanded-FK topic reads the zero pose; the detector chatters twice per real step | ledger 2026-10-02; motor audit §4.2 |
| Contact is referenced by nothing | the physics contact flag and load are published every tick and consumed by no control path | motor audit §2.3 |
| The planner has no material | the body-pose vocabulary's transition model equals persistence once warm; masking damages the cone | repair plan M0–M0.c |
| The predictor is reactive | pose-transition surprise leads touchdowns by +0.02σ | repair plan P5 |
| At the robot's servo speed the feet do not lift | 0 steps on 6 of 6 seeds at 3.67 rad/s against 28 at 6.0 | ledger 2026-10-01 |
| Filtering the plant works where no brain lever has | servo lag α 0.2 on the robot: jerk ÷4.5, full-slew ticks 80 % → 1–2 %, current halved; the brain's own command unchanged | ledger 2026-10-03 |
| The sim's gait and the robot's differ at the foot | sim: swing bouts 0.07 s, contact flicker 1–3/min, 0 % clean strides, still 4.65 m; robot: flicker 100–200/min, no progress | ledger 2026-10-12 |

Two earlier results frame these. The Playful Machine's hexapod walks on five settings and no
prescribed phase relationship, with its help in the body (reduced gravity, grippy ground,
springy feet). And the one gait in this project's history that was fully coherent, the
open-loop clock-driven walk of 2026-07-20, was rejected on sight because the chassis slammed
the ground every step: a timed command that cannot feel the ground is a servo sequencer.

## 3. Why the thrash is structural

Put the facts in §1 and §2 together and the noise is overdetermined.

**The phase is its own consumer.** The knee's phase is the angle of (position, velocity). The
coupling term is a knee command computed from that angle. A knee command changes the knee's
velocity next tick, which rotates the angle, which changes the command. With the velocity
arm unfiltered and scaled by fifteen, a one-tick reversal moves the angle by half a turn. The
loop is a relay: it drives to the slew limit, reverses, drives to the limit. The 2026-10-02
attribution table shows the coupling alone putting the knee at the slew cap 92 % of ticks.
Every repair that smoothed the phase was refuted because the stroke rides the same phase and
a smoothed phase fires the stroke late; the family's own closing verdict was "the phase is a
state observation, every consumer needs it raw". That verdict is correct about this phase and
is also the reason this phase cannot be a clock.

**The actuator is clipped, so timing changes do not register.** With the stroke alone past
the rail 42 % of its cycle and the sum clipped 56 % of the time on the hip, adjusting the
timing of any one term changes the output little. The Playful Machine analysis called this
the mixing desk with every channel in distortion. It predicts, correctly, the nine timing
levers that moved nothing.

**The press is gated on the wrong state.** A press on planted legs is the doctrine's own
worked example of a correct gate. Gated on attitude instead of on the foot, it presses
whichever legs the body happens to be rocking onto, and withdraws as the body rocks back,
at the detector's 50-tick relaxation timescale, against the 70-tick stride. The corrected
source lifted the feet on every seed and was parked for re-tuning.

**The learned layer cannot rescue it.** Three reasons, all measured. The homeokinetic rule's
gradient assumes a smooth actuator and is wrong wherever the command is railed. A slow,
predictable shuffle is low prediction error, so the rule is content with it, and
progress-commit damps the exploration that would leave it (the audit's "rewarded fixed
point"). And the rule has no cross-leg terms, so inter-leg coordination can only arrive
through the body, which the imposed terms drown.

**Nothing predicts a touchdown, so nothing can be early.** Every term reacts to the present
state. The one predictor built (the body-pose transition surprise) led touchdowns by nothing.
The 2026-08-06 analysis that the layer "predicts in order to learn, never in order to act"
stands, and its rung 1 (act on the predicted state) is in the file, off.

The through-line the project already names, that imposed fights and learned cooperates, is
being read here one level up: not that one term is imposed, but that the whole timing
substrate is, and the learned part is riding on it rather than the other way round.

## 4. What should not be tried again on this stack

The ledger's verdicts, with their re-use contexts, say these are closed in this context:

- Smoothing, locking or offsetting the per-leg phase (P1, P4 arms 1–2): each costs most of
  the transport. Retry only with a phase that is not read from the joint the stroke drives.
- A per-leg touchdown clock driving the stroke while the coupling still reads the old phase
  (`stroke_phase_src`): two clocks on one leg, coherence lost both times.
- Symmetry of any form (~35 arms), the velocity objective, learned hip 2, lateral-sequence
  phasing on this base, coupling precision on an intact body, the deadband on the attitude
  detector, first-order smoothed noise.
- Selecting rhythm through the coordination search (P4 arms 3, 3b): rhythm is not in that
  search space.
- The motor planner and the anticipatory predictor on the current vocabulary: both killed
  at the instrument stage for lack of material. Their re-use context is a body that steps
  regularly, which is this audit's goal, not its means.
- More gain search on the same terms: the evolver found its operating point and the
  2026-10-02 run showed live search does not beat frozen gains on the robot's inputs.

## 5. The approach to try: a load-modulated per-leg phase

### 5.1 The rule

Each leg i keeps a phase φ_i that advances at an intrinsic rate ω and is slowed in proportion
to the load N_i the foot feels:

    dφ_i/dt = ω − σ · N_i · cos φ_i

with the convention that the leg is in stance (pushing back) for one half of the cycle and in
swing (lifted, recovering forward) for the other. The product N_i · cos φ_i is positive when
the leg is loaded during the part of stance where it should keep supporting, which slows the
phase and holds the push; it is negative when a loaded leg has reached the point where it
would lift, which also keeps it from lifting while it still carries weight. The foot's
target position is a fixed function of φ_i (back and down through stance, up and forward
through swing). No inter-leg term exists. When one leg lifts, its share of the body's weight
moves to the others, their load rises, their phases slow, and they hold stance; the legs
coordinate through the ground. Owaki and colleagues showed walking, trotting and a gallop
emerging from this on a real quadruped as ω was raised (J. R. Soc. Interface 2013;
Scientific Reports 2017), and the same rule drives a hexapod and a centipede in their later
work.

### 5.2 Why it answers each defect in §3

- The phase is an integrator, so it is monotone by construction. Nothing reads an angle off
  the joint it drives. The relay loop is gone.
- The push knows whether the foot is on the ground. That is the stroke-to-contact loop the
  motor audit put third in its dependency order, achieved by the oscillator's own dynamics
  rather than by a separate frequency estimator (the failure of the touchdown clock).
- The planted/swinging state is the phase, and the phase is coupled to the load. The
  attitude detector is retired, and with it the relaxation oscillation.
- The per-tick command is a smooth function of a slowly advancing phase. At the robot's servo
  speed the servo sees a trajectory, not a reversal every 2.4 ticks.
- No trot is prescribed. Gait selection becomes a property of the drive level, which is what
  the operator asked for: trot or one-leg-at-a-time walking, arrived at rather than specified.

### 5.3 Fit to the doctrine, and the one judgment call

The rewrite rule asks for the error, the channel, the owning module and the gate.

- The error: each leg predicts that in stance it feels load and in swing it does not; the
  phase moves to make that prediction true. It is a one-dimensional active-inference
  oscillator per leg.
- The channel: the foot force sensor. It exists on the robot (threshold-grade, which is
  enough: with N_i ∈ {0, 1} the rule still holds) and in the simulator (the physics contact
  flag and the load value). This is the sensor the ledger's Walknet retry context asked for
  ("give the load rules a real load observation").
- The owning module: a new, small one. Not a sixty-first parameter in the 7 300-line file;
  the v2 plan's own argument for building v2 as a separate module applies with more force
  now. Its output replaces terms 6 and 7 of §1 and the detector behind 3 and 4. Terms 1, 2
  and the reflexes stay, which honours the rule never to disable a working loop.
- The gate: the load.

The judgment call is §5.7 of the hard prohibitions, "don't inject a rhythm". The rule carries
an intrinsic rate ω per leg. The ledger's 2026-08-03 position is the precedent: an innate
rhythm is legitimate (the lamprey's spinal generator); what makes a term a script is a gain
no error modulates. Here the rate is modulated every tick by the leg's own load error, and
no coordination topology is injected at all, which is the half of §5.7 the current stack
violates. The operator decides whether that reading holds. The strictly rhythm-free
alternative is in §7.

### 5.4 The risk the record names

Per-leg clocks lost inter-leg coherence twice (repair plan P4 arms 1 and 2). Both times a
second clock still pulled the knee from the old phase, and the reference was touchdown
timing, not load. The Tegotae rule has one clock per leg and no other, and its coupling is
physical. The hypothesis this build falsifies or confirms is that four load-modulated legs
on this body coordinate through the ground. Two things could make it fail here and not on
Owaki's robot: a sprawled stance whose belly carries weight, so lifting a foot moves no load
to the others (the hip 2 +0.3 stance, belly 51 mm, is the body to test on); and a foot that
cannot grip, so stance carries no reaction force worth reading (§6).

### 5.5 Staging and gates

Build it gain-0-guarded as a separate module, shadow first, then authority, then hardware.

1. **Shadow phase, zero authority, sim.** Run the four oscillators on the contact flag beside
   the current stack and score them as the P1 shadows were scored: monotonicity, and
   phase-locking value against true touchdown. A phase that does not lock to contact better
   than the incumbent's 0.21 while walking is dead before it drives anything.
2. **Authority in the sim, n ≥ 6, full metric set.** Replace the coupling, stroke and
   detector with the oscillator's trajectory; keep the learned layer, the posture spring and
   the reflexes. The gates are the operator's goal as numbers: reversals per stride per joint
   under 3 (today 11–16), inter-touchdown coefficient of variation under 0.5 (today ≈ 1.0),
   inter-leg phase-locking above its null, belly clearance held, zero falls, and distance that
   at least ties. A real capability is loud (§3.3); if it needs n = 20 to see, kill it.
3. **Robot, through the dash,** with the existing thrash instrument reading reversals per
   stride and contact flicker. Judge on the servo line and on the operator's eye.
4. **The (d) test:** lesion one foot sensor mid-run and watch the three others reorganize.

## 6. The parallel lever: the foot

Every mechanism in §5 couples through contact, and the simulator's contact is the open
problem the lesion study named: the sim's foot pushes while scrubbing, the robot's does not.
Before step 2 above can be trusted, measure the robot's foot on carpet and on concrete
(friction coefficient from a drag test, compliance from a load test, the force sensor's
threshold against actual stance) and put those into the measured body model. In parallel, a
grippy, slightly compliant sole on the four toe modules is a cheap hardware change in the
direction the Playful Machine took: springy feet and rubber ground were in every one of
their walking experiments. That change is measurable on its own, on the current stack, by the
contact flicker count.

## 7. Secondary candidates, ranked

- **Walknet with real load** (the ledger's own retry context). Six hand-written inter-leg
  rules gated on load. More imposed structure than §5, and the project has already been
  burned by a one-of-six-rule slice. Try it if §5 coordinates poorly through the body.
- **Pure homeokinesis with load-gated posture terms and no clock at all.** The strictest
  reading of §5.7. The record says pure homeokinesis paddles with the belly on the floor;
  it would need the taller stance and the foot fix first, and it has never produced a step
  rhythm anywhere in this project or in the Playful Machine's quadruped (their coherent gaits
  are the hexapod and the crawl).
- **Command-shaping only:** a second-order noise filter, a slew-aware command, the servo lag
  as the plant filter. These treat the symptom; the lag result shows how much the hardware
  gains from it and is worth keeping regardless.
- **Act on the predicted state** (rung 1, in the file, off). Cheap to run and unmeasured; it
  cannot fix a clock that is its own consumer, but it is the right first experiment on
  whatever substrate §5 produces.

## 8. Claims we are not making

- That the Tegotae rule will coordinate this body. It is a hypothesis with a precedent on a
  different quadruped, and §5.4 names the two ways this body can defeat it.
- That the learned layer is useless. On the pure-homeokinesis tier it is the only thing that
  moves the body, and it learns a correct velocity-feedback law on the deployed stack. It has
  no job on this stack because the imposed terms leave it none.
- That the simulator can evaluate stepping today. It cannot, until the foot is measured.
- That anything above was run. This audit reads the record; every number has its ledger
  entry and its sample size there.

## 9. Next work, in order

1. Measure the robot's foot (friction, compliance, sensor threshold at stance) and update the
   measured body model; count contact flicker before and after a grippy sole.
2. Build the load-modulated oscillator as a separate module with a gain-0 guard; run it as a
   shadow and score it against true touchdown.
3. Give it authority in the sim on the hip 2 +0.3 body, n ≥ 6, judged on reversals per stride,
   interval regularity, inter-leg locking, belly clearance, falls and distance.
4. Put it on the robot through the dash with the thrash instrument, and watch.
5. Then, and only on a body that steps regularly, re-audition the planner and the predictor,
   whose re-use contexts both name exactly that body.

## Postscript: the first measurement, the same day

The load-modulated phase was built as a separate module (`LoadPhaseGait`) with three gain-guarded
sockets in the motor layer and run on the lesion study's own protocol (arena, six seeds, 6 000
ticks). With the module present and every socket off, the trajectory is byte-identical to the
base on all six seeds. With the stroke and the stance gate on the load-held phase and the trot
coupling off: steps per run 2.8 → 47.7, clean strides 0 → 38 %, hip reversals per stride 11.1 →
4.4, swings 0.07 → 0.24 s, distance 4.65 → 6.26 m, inter-touchdown variation 0.75 → 0.64. The
same phase with the load coupling removed (a free clock) steps as often and goes nowhere, with
six falls per run: the coupling through the ground is the mechanism. The cost is stability, about
one fall per run and four times the tilt variance, on a body whose stabilisers were tuned for a
shuffle. The full record, the dose arms and the next steps are in the ledger's 2026-10-12 entry.

## Appendix: anchors for the claims above

- Per-leg phase: `cpp_core/src/ogma/modules/MotorEPMv2.cpp:3478-3540` (the pre-pass; the
  retrograde diagnostic is computed beside it).
- Trot coupling on the knee: `MotorEPMv2.cpp:5617`. Power stroke on the hip:
  `MotorEPMv2.cpp:5716-5726`. Stance press: `MotorEPMv2.cpp:5409-5413`. Noise:
  `MotorEPMv2.cpp:5506`. Clamp: `MotorEPMv2.cpp:5966-5970`.
- Swing detector, attitude source: `godot_host/project/scripts/picrawler_body.gd` (~6940,
  per the 2026-10-02 ledger entry); robot reproduction:
  `pi_host/include/ogma/hw/BrainInputs.hpp:204-211`.
- Contact and load published and unconsumed: `pi_host/src/ogma_host.cpp:916-918`;
  `contact_instrument_only: 1` in the deployed config.
- The corrected detector source, not deployed:
  `…__native_measured__tofboom__fsrleg__cmdfk.json` (`OGMA_PICRAWLER_CMD_FK_SOURCE=1`).
- Evidence: `docs/reports/picrawler_lever_ledger.md` entries dated 2026-07-27, 2026-08-02,
  2026-08-03, 2026-10-01/02, 2026-10-03, 2026-10-12;
  `docs/plans-and-designs/locomotion_substrate_repair_plan.md` campaign logs P0–P5 and
  M0–M0.c; `docs/reports/motor_system_audit_2026-08-09.md` §2.2, §2.3, §4;
  `docs/reports/playful_machine_source_analysis.md` §0, §7;
  `docs/plans-and-designs/motor_layer_is_reactive.md`.
- Literature: Owaki, Kano, Nagasawa, Tero, Ishiguro, "Simple robot suggests physical interlimb
  communication is essential for quadruped walking," J. R. Soc. Interface 10:20120669 (2013);
  Owaki, Ishiguro, "A quadruped robot exhibiting spontaneous gait transitions from walking to
  trotting to galloping," Scientific Reports 7:277 (2017).
