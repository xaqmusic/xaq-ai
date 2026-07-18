# Servo dynamics — hobby-servo model for the picrawler

> **Scope.** Sections 1–3 are the *external reference spec* — ballpark values for a
> metal-gear hobby servo (~MG90S class), drawn from academic robotics papers,
> MATLAB/Simulink docs, and mechanical models [1]. Section 4 documents the
> **as-built implementation** in `godot_host/project/scripts/picrawler_body.gd`
> and how/why it deviates. This model backs the **`hinge`** joint backend (the
> default); the **`g6dof`** backend instead models passive angular elasticity
> (see `joint_backend` in `picrawler_body.gd`), not this servo model.

While exact engineering datasheets detailing internal back-EMF and gear-train
friction for hobby servos are rarely published, standardized "ballpark"
approximations are widely available. The baseline values below can be used
directly in a Godot simulation. [1]

***

## 1. Unpowered state (passive resistance)

When the servo is turned off, the motor acts as a generator (back-EMF) and the
dense metal gear train resists rotation. In Godot these values apply a
counter-torque opposing the joint's current angular velocity (`ω`).

* **Static/Coulomb friction `T_f`: 0.05–0.15 Nm** — the "stiction" / backdrive
  torque. An unpowered metal-gear servo needs a solid push before it will budge;
  if external torque is below this value the joint stays put.
* **Viscous + back-EMF `c_v + c_emf`: 0.01–0.03 Nm·s/rad** — the thick "greased
  damping" feel; the faster an external force rotates the unpowered joint, the
  harder it resists. [1]

***

## 2. Powered state (PWM & control loop)

When powered, the virtual PWM signal maps to a target angle. A real servo reads a
pulse of 1000–2000 µs (or 500–2500 µs for extended range) and scales it to an
angle. [2]

* **Virtual PWM mapping:** 1000 µs = −90°, 1500 µs = 0° (center), 2000 µs = +90°.
* **Deadband: 0.0003–0.0015 rad** (≈1–5 µs of PWM width) — if |target − current|
  is below this, output 0 torque, to stop rest-twitching.
* **PID gains** (metal-gear servos use stiff, aggressive loops):
  * `Kp` (proportional): **150–300** — high holding torque, snaps to target.
  * `Ki` (integral): **0.01–0.05** — very low or omitted; hobby servos prioritize
    speed over zero-error perfection.
  * `Kd` (derivative): **5–15** — critical for heavy gears; damps overshoot. [2, 3]

***

## 3. Saturation & performance limits

* **Stall torque `T_stall`: 1.5–3.5 Nm** (≈200–500 oz-in) — the max torque the PID
  loop may output; caps when the joint is blocked.
* **Max no-load speed `ω_max`: 6.0–10.0 rad/s** (≈0.10–0.17 s per 60°).

### Implementation tip (Godot `_physics_process`)

1. **Unpowered:** read the joint's angular velocity, compute passive torque from
   the unpowered values, apply opposite to motion.
2. **Powered:** read the target angle, compute error vs current angle; if it
   exceeds the deadband, compute PID torque; apply a linear torque–speed dropoff
   (scale available torque down as speed → `ω_max`); pass the torque limit to
   Godot's joint motor. [4, 5]

***

## 4. As-built in xaq-ai — verified 2026-07-18

The sim drives joints with **motor target-velocity control** (not `apply_torque()`),
so some reference gains are reinterpreted. Constants: `picrawler_body.gd:123-165`
(powered) and `:182-185` (unpowered); helpers `_set_motor_vf` (`:3171`),
`_powered_torque`, first-order lag (`:~5172`), unpowered back-drive (`:~5284`).

| Reference (§1–3) | Range | As-built constant | Value | Status |
|---|---|---|---|---|
| Static/Coulomb friction | 0.05–0.15 Nm | `UNPOWERED_STATIC_FRICTION` | 0.10 Nm | ✅ in range |
| Viscous + back-EMF | 0.01–0.03 Nm·s/rad | `UNPOWERED_VISCOUS_FRICTION` | 0.02 | ✅ in range |
| Deadband | 0.0003–0.0015 rad | `SERVO_DEADBAND` | 0.001 rad | ✅ in range |
| Kd | 5–15 | `SERVO_KD` | 8.0 | ✅ in range |
| Max no-load speed | 6–10 rad/s | `MAX_SERVO_SPEED` | 6.0 rad/s | ✅ (low end) |
| Kp | 150–300 (position-PD) | `SERVO_KP` | 20.0 | ⚠️ reinterpreted — (a) |
| Ki | 0.01–0.05 | `SERVO_KI` | 0.0 | ⚠️ omitted (doc allows) |
| Stall-torque cap | 1.5–3.5 Nm | `MAX_SERVO_TORQUE` | 0.15 Nm | ⚠️ no-load class — (b) |
| PWM travel | ±90° | `HIP_TARGET_RANGE` | ±80° (1.40 rad) | ⚠️ ±80°; knee asymmetric |
| — | — | `SERVO_TORQUE_RISE_TAU` | 0.030 s | ➕ addition — (c) |
| — | — | `motor_authority_scale` | 1.0 | ➕ addition — servo-saver (d) |

**Deviations & additions (all deliberate):**

**(a) Kp = 20, not 150–300.** The reference Kp is a *position-PD torque* gain. The
sim commands *motor target velocity* (`HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY`
via `_set_motor_vf`), so `SERVO_KP` is a velocity-command gain (rad/s per rad of
error): 0.4 rad error → 8 rad/s (saturates at max). A gentler gain gives smoother
motion under velocity control.

**(b) MAX_SERVO_TORQUE = 0.15 Nm, not 1.5–3.5 Nm stall.** 0.15 Nm is the motor's
*no-load* torque class, which dominates on the picrawler's tiny legs (static
gravity load at hip2 ≈ 0.037 Nm → ~4× headroom). The datasheet stall figure only
matters under heavy load; feeding 2 Nm produces ~150,000 rad/s² accelerations that
exceed Euler stability at 240 Hz and read as flailing.

**(c) First-order torque-rise lag τ = 30 ms** (`SERVO_TORQUE_RISE_TAU`). Not in the
generic reference; emulates gear inertia + motor electrical time constant + 50 Hz
PWM. Per 20 ms brain tick, α = 1 − exp(−0.020/0.030) ≈ 0.49. Prevents step-impulse
torque "twitching" that stiff gains would otherwise cause at 240 Hz integration.

**(d) Servo-saver compliance** via `motor_authority_scale` (default 1.0 = rigid).
Values <1.0 cap transmitted torque so the joint deflects under overload — modeling
the finite spring rate of a real picrawler servo-saver. Sim2real:
`motor_authority_scale = saver_max_transmissible_torque / MAX_SERVO_TORQUE`.

**Also implemented per the reference's Implementation Tip:** the torque–speed
dropoff (output tapers to 0 as ω → `MAX_SERVO_SPEED`) and unpowered back-drive
resistance (static + viscous friction applied opposite to ω when a joint is
unpowered).

**Verdict.** The directly-comparable reference values (friction, viscous, deadband,
Kd, speed) are all within spec as-built. The three deviations (Kp *meaning*, torque
cap, travel) and two additions (rise-lag, servo-saver) are intentional and
explained above — the doc now matches the current implementation.

***

[1] MathWorks — https://www.mathworks.com/help/sps/ref/rcservo.html
[2] Instructables — https://www.instructables.com/Servo-Deadband-Correction/
[3] Parker Motion — https://www.parkermotion.com/whitepages/servofundamentals.pdf
[4] Godot Engine — https://docs.godotengine.org/en/stable/classes/class_joint3d.html
[5] ESI Motion — https://esimotion.com/blogs/news/the-servo-controller-and-motor-relationship-explained
