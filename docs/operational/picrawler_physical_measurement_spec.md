# Physical PiCrawler — measurement spec for the sim rebuild

**Purpose.** The operator is building the physical PiCrawler to measure geometry, mass and joint
behaviour precisely. This is the list of what to measure, **in priority order**, and what each
number is load-bearing for. It exists because the 2026-08-02 campaign concluded that the
project's remaining bottleneck is **morphology, not control** — see
[`../reports/playful_machine_source_analysis.md`](../reports/playful_machine_source_analysis.md)
§8k and the ledger entry of the same date.

**Why morphology.** The scaffolds keeping the robot upright (`postural_gain`,
`height_homeo_gain`, `stance_lift_gain`) are load-bearing for *stability*, and stability cannot
be learned in from a body that is not yet stable — a bootstrap. The Playful Machine solves that
bootstrap with an external anti-fall operator (a removable prop, which we reject); we solved it
with a permanent internal control term (which can never be de-scaffolded). **A body whose
PASSIVE equilibrium is standing needs no bootstrap at all.** That is the target, and it is a
mechanical question.

**Current sim values are CAD-derived, not measured.** Anything below marked ⚠ is a value the sim
currently asserts and has never been checked against hardware.

---

## Priority 1 — the numbers that decide the morphology question

### 1.1 Segment lengths (axis-to-axis, not case-to-case)

| symbol | definition | sim value ⚠ | how to measure |
|---|---|---|---|
| `L1` | hip1 axis → hip2 axis (coxa) | 37.26 mm | pin both axes, measure centre-to-centre with the leg straight |
| `L2` | hip2 axis → knee axis (upper leg) | 53.6 mm | same |
| `L3` | knee axis → **toe tip** (lower leg + foot) | 75.5 mm | include whatever actually contacts the floor |

**Why this is priority 1.** The ledger's kinematic dead-end entry measured feet planting at
**170 mm** against a total straight-line reach of `L1+L2+L3 = 166.4 mm`. The limb is *at or past
its geometric limit just to touch the ground*, which forces the shank to splay outward — and that
splay is what makes the belly ride low and the postural term necessary. **If the real L3 differs
from 75.5 mm the whole dead-end conclusion changes.**

### 1.2 Standing stance geometry (the number that actually matters)

With the robot powered and standing in its natural rest pose, measure:

- **Foot spread**: horizontal distance from each hip1 axis to its toe contact point.
- **Belly height**: floor to lowest chassis point.
- **Shank angle**: angle of the lower leg from vertical (the sim measures 37.5° against a
  design-rest of 10°; ⚠ never verified).
- **Hip2 and knee angles** in that pose.

**This is the bootstrap question stated physically:** *does the robot stand at a useful height
with the servos unpowered or at neutral, or only when actively held?* Try both — power off, and
power on at neutral commands. If it collapses unpowered, the postural term is compensating for
morphology and no controller change will remove it.

### 1.3 Lower-leg extension feasibility

You mentioned extending the lower leg is feasible. What the sim needs to evaluate it:

- **Range achievable** for `L3` (min/max you can practically build).
- **Added mass** per unit extension, and whether it is distal (worst for inertia) or shifts the
  segment CoM.
- Whether the knee servo still has torque authority at the longer moment arm — this is the
  trade: a longer shank reaches the ground without splay, but multiplies the gravitational
  moment at the knee.

**A sweep of `L3` in sim is cheap and I can run it as soon as we know the feasible range and the
mass penalty.** That is the single most direct test of the morphology hypothesis.

---

## Priority 2 — mass and inertia

| | sim value ⚠ | note |
|---|---|---|
| chassis | 300 g | "Pi 5 + battery + base plate + 4 chassis-mounted hip1 servos" |
| coxa | 25 g | hip2 servo + bracket + connectors |
| upper leg | 25 g | knee servo + bracket + connectors |
| lower leg | 25 g | |
| **total** | **600 g** | |

Measure: **total assembled mass**, then each segment separately if you can disassemble
(otherwise: one leg complete, and the chassis with legs removed). Also worth having:

- **Chassis CoM** location relative to the hip1 rectangle (sim assumes centred; a real Pi +
  battery is not).
- Whether battery position can be moved — CoM height and fore/aft position are free design
  variables that affect stability more than most control gains.

**Why it matters:** the sim's mass ratios set every torque margin. The servo model currently
assumes a static hip2 load of ~0.037 Nm against a 0.15 Nm limit (≈4× headroom). If real mass is
higher or more distal, that headroom shrinks and several "the controller should fix this"
conclusions become "the actuator cannot."

---

## Priority 3 — servo behaviour (the model with the least evidence behind it)

The sim's servo is a velocity-command model: `target_velocity = Kp_vel · position_error`, capped
at `MAX_SERVO_SPEED = 6 rad/s`, torque-limited to `MAX_SERVO_TORQUE = 0.15 Nm`, with a 30 ms
first-order torque rise and a 0.001 rad deadband. ⚠ **All of these are estimates.**

Worth measuring, roughly, on one servo:

1. **No-load slew rate** — command a full ±80° swing, time it. Gives `MAX_SERVO_SPEED` directly.
2. **Stall / holding torque** — hang a known mass at a known moment arm, find where it gives up.
   The sim uses 0.15 Nm against a spec sheet claiming 1.5–3.5 Nm; the comment in the code argues
   the low value is right for these small legs, but it has never been checked.
3. **Backdrive resistance, unpowered** — how much torque to move a powered-off joint. The sim
   models 0.10 Nm stiction + 0.02 Nm·s/rad viscous.
4. **Does it report position at all?** The sensor-legitimacy work assumed hobby servos report
   nothing, which is why the deployed gait uses FK from *commanded* angles. **If your build has
   any position feedback, that changes what is legally sensable** and re-opens several entries.

---

## Priority 4 — foot and ground contact

- **Foot material and contact patch shape.** Sim uses friction 0.5 ("small plastic foot on smooth
  surface"), a value chosen by argument, never measured.
- **Is there any passive compliance in the foot or ankle?** PM's legged robots all have a
  compliant passive distal segment (`tarsus`), and every one of their emergence results includes
  it. We have none. **A compliant foot is a morphological change that is doctrine-legal
  permanently**, unlike an anti-fall operator, and it is the closest legitimate analogue to what
  PM's scaffold buys them.

---

## What I will do with each number

| measurement | what it unblocks |
|---|---|
| `L3` + feasible extension range + mass penalty | **an `L3` sweep in sim** — the direct test of whether morphology removes the bootstrap |
| standing stance, powered vs unpowered | whether `postural_gain` is compensating for geometry (if yes, it is a body bug, not a control one) |
| true masses + chassis CoM | re-derive every torque margin; several "controller" conclusions may be actuator limits |
| stall torque + slew rate | whether `MAX_SERVO_TORQUE = 0.15` is honest; it gates how hard any gait can push |
| position feedback yes/no | re-opens the sensor-legitimacy ledger entries |
| foot compliance | the one PM-legitimate scaffold class we have never tried |

**Send them in any order — each one is independently useful, and the `L3` group is the one that
unblocks the most.**
