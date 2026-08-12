# quat_sitl

Pure-Python Software-In-The-Loop reproduction of:

> Emil Fresk & George Nikolakopoulos, **"Full Quaternion Based Attitude Control for a
> Quadrotor,"** European Control Conference (ECC) 2013.

The paper's claim: a quadrotor's attitude plant and a nonlinear P² controller can both be
formulated **entirely in quaternion space**, with no Euler-angle or DCM transformations
anywhere in the dynamics or control loop — avoiding gimbal lock and the trigonometric cost
of Euler-angle-based control, while remaining derivative-free and low-complexity. This
project implements that plant and controller exactly, with Euler angles used only for
plotting/logging, and reproduces the paper's three simulation scenarios (step, sine
tracking, 360° flip).

## Install & run

```bash
pip install -e .
pytest tests/ -v

python -m quat_sitl.simulator --scenario step --duration 15 --seed 0 --noise 0.1
python scenarios/step_response.py
python scenarios/sine_tracking.py
python scenarios/flip_360.py
```

Each scenario script writes a timestamped CSV + PNG figures to `results/`.

## Equation → function map

| Paper eq. | Description | Location |
|---|---|---|
| (1)-(2) | Quaternion representation, Kronecker product, `Q`/`Q̄` matrices | `quaternion.mul`, `quaternion.Q`, `quaternion.Q_bar` |
| (3) | Norm | `quaternion.norm` |
| (4) | Conjugate | `quaternion.conj` |
| (5) | Inverse | `quaternion.inv` |
| (6) | q̇, fixed-frame ω | `quaternion.qdot_fixed` |
| (7) | q̇, body-frame ω′ | `quaternion.qdot_body` |
| (8) | Vector rotation via q ⊗ v ⊗ q* | `quaternion.rotate` |
| (9)-(12) | DCM columns Rx, Ry, Rz | `quaternion.to_dcm` |
| (13) | DCM transpose (frame rotation) | `quaternion.to_dcm(q).T` |
| (14) | Axis-angle → quaternion | `quaternion.from_axis_angle` |
| (15) | Euler → quaternion | `quaternion.from_euler` |
| (16) | Quaternion → Euler (plotting only) | `quaternion.to_euler` |
| (17)-(18) | Rigid-body plant | `dynamics.state_derivative`, `dynamics.InertiaParams` |
| (19)-(21) | Nonlinear P² controller | `controller.NonlinearP2Controller.compute_torque` |

## Results vs paper

### What matches well

- **Step response** (`scenarios/step_response.py`): staggered 1 rad steps on x/y/z converge
  smoothly with essentially no overshoot, settling in ~2s — matches the paper's Fig. 3/4
  qualitative description ("very small overshoot ... errors go quickly to zero").
- **Sine tracking** (`scenarios/sine_tracking.py`): 0.5 rad / 1 rad/s tracking shows a phase
  lag of ~0.4-0.5s, matching the paper's stated "~0.5 second" lag almost exactly.
- **360° flip** (`scenarios/flip_360.py`): with `shortest_path=False`, the controller
  commits to the full rotation. The raw `q0`/`q1` plot is smooth throughout with no
  singularity, exactly reproducing the point of the paper's Fig. 7/8 pair (the wrapped
  `φ = atan2(...)` plot jumps at ±π, which is an inherent property of `atan2`'s range, not
  a controller defect — that's precisely why the paper plots `q0`/`q1` alongside it).

### Genuine paper ambiguities, resolved with a documented default

1. **DCM convention (eq. 12 vs 13).** Eq. 12 stacks `Rx, Ry, Rz` as columns (rotates a
   point from body frame to fixed frame); eq. 13 is its transpose (rotates the frame
   itself). `to_dcm()` implements eq. 12; test 4 (`rotate(q,v) == to_dcm(q) @ v`) pins this
   down concretely. Use `to_dcm(q).T` for the eq. 13 sense.
2. **Noise distribution.** The paper says "additive... amplitude 0.1" without stating
   uniform vs. Gaussian. Implemented as uniform `U(-0.1, 0.1)` (an "amplitude" implies a
   hard bound; a Gaussian has no such bound, only a σ).
3. **Step stagger timing.** Paper says only "at different time instants." Chosen: x@1s,
   y@5s, z@9s over a 15s run (`references.DEFAULT_STEP_AXIS_TIMES`) — evenly spaced with
   settling margin.
4. **Sine phase shift.** Paper requires torques "not in phase" but gives no numeric value.
   Chosen: `phase_y=0, phase_z=π/2` (`references.sine_reference`).
5. **Flip ramp duration.** Paper doesn't state how much of the ~5s window the 0→2π ramp
   takes. Chosen: 2s (`references.flip_reference`).

### The one deviation from the literal spec, and why

**`dynamics.state_derivative` preserves eq. 18's sign exactly, as required.** Eq. 18 reads
`q̇ = -½[0,ω]⊗q`, the *opposite* sign from eq. 7's general body-frame derivative
`q̇ = +½[0,ω′]⊗q`. This was verified directly against the PDF text (not a transcription
slip) and is kept literally in `dynamics.py`, deliberately *not* delegated to
`quaternion.qdot_body` so the deviation stays visible rather than being silently "fixed."
Test 7 (`test_sign_consistency_...`) guards this: it checks that one closed-loop RK4 step
reduces attitude error, which would fail if either eq. 18's or eq. 21's sign were wrong. A
side effect worth noting: because of this sign, a *positive* commanded ω about an axis
*reduces* an existing positive attitude error on that axis in this formulation — the
opposite of the usual textbook `q̇ = +½[0,ω]⊗q` intuition. This is a real, reproducible
property of the paper's own equations, not a bug.

**Control-tick rate: deviated from the literal "200 Hz" spec, and this one is load-bearing.**
The original SITL design called for the controller to run at 200 Hz (ZOH-held torque) with
the plant integrated via RK4 at 1 kHz. Implementing that literally and testing the
closed-loop step response revealed the loop **never converges** — it saturates into a
sustained oscillation for a 1 rad step, and in fact for a step of *any* size, however small
(even 0.001 rad). This was tracked down rigorously, not assumed:

- Linearizing the closed loop (plant eq. 18 + controller eq. 21) around identity attitude
  gives a 2nd-order system `I·ë + Pw·ė + (Pq/2)·e = 0` with the paper's own gains
  (`Pq=20, Pw=4`) and inertia (`Ixx=Iyy=6.5e-4`). This has two *real* poles: a slow one at
  ≈ -2.5 rad/s (time constant ≈0.4s — matches the paper's few-second settling) and a fast
  one at ≈ -Pw/Ixx ≈ -6150 rad/s (time constant ≈0.16 ms). The continuous-time system is
  **not oscillatory** — it's heavily overdamped (ζ≈25) and well-behaved.
- The problem is entirely about *sampling* that fast pole. Computing the exact
  zero-order-hold discrete-time closed-loop transition matrix at the spec's rates gives a
  spectral radius of **~29.9 at 200 Hz** and **~5.16 at 1 kHz** — both far outside the unit
  circle (provably unstable), confirmed against the raw simulation (which shows the
  controller torque pinned at the ±4 N·m saturation limit indefinitely, chattering between
  extremes at the fast-pole timescale). The instability only clears somewhere around
  3-5 kHz.
- This is **not a plant-integration accuracy problem** — RK4 at `dt=1e-3` for a *fixed*
  torque is already fine; sub-stepping the ODE solver alone does not fix it, because the
  issue is the control *sample* rate itself being far too slow (by roughly an order of
  magnitude) relative to the closed loop's fastest natural mode.

**Resolution:** `dynamics.stable_control_rate_hz(Pw, inertia)` computes a safe control-tick
rate from the actual gains/inertia (`Pw / I_min / margin`, margin=0.5 → ≈12.3 kHz for the
paper's defaults — comfortably inside the stable region with headroom). `simulator.
run_simulation` uses this as the default control rate instead of a literal 200 Hz, while
state is still **logged at exactly 1 kHz** as specified, and the plant equations,
controller equations, and gains (`Pq=20, Pw=4`) are all completely unchanged from the
paper. Pass `control_rate_hz=200.0` explicitly to `run_simulation` to reproduce the literal
spec rate and observe the instability directly. This is the only place this project departs
from a literal reading of the SITL architecture spec, and it was forced by the paper's own
numbers, not chosen for convenience — see `dynamics.stable_control_rate_hz`'s docstring for
the full derivation.

### Minor observed effect

The sine-tracking scenario briefly saturates for a single logged sample at `t=0.001s`
(1 row out of 15000). This is a startup transient: the `phase_z=π/2` sine reference starts
at `ψ_ref(0)=0.5` rad rather than 0, so the very first instant looks like a small step to
the controller. It has no visible effect on tracking quality and clears immediately: the
paper's "no saturation" claim is read as describing steady-state tracking, not this
unavoidable single-sample initialization edge.

## Assumptions / simplifications carried from the spec

- Control-signal → torque relation is identity (`dynamics.IdentityMotorModel`), per the
  paper's own stated simplification. `dynamics.FirstOrderLagMotorModel` is included as an
  unwired stub proving the interface is swappable, but no scenario uses it.
- The plant (eq. 17-18) has no aerodynamic drag/damping terms — the paper's own simulation
  used a separate, more detailed nonlinear model (ref. [10] in the paper) for its actual
  numeric results, which likely included such damping; that model's equations aren't given
  in the paper text, so this project implements exactly the simplified eq. 17-18 the spec
  asked for (rigid body, gravity-bias neglected, only differential propeller torque).

## Future work

The plant's `MotorModel` interface (`dynamics.py`) and the controller's independence from
plant internals were both kept swappable intentionally. A natural next step (not built
here) is wrapping the plant in a MAVLink-style interface (`pymavlink`/`mavsdk`) so the same
`NonlinearP2Controller`, unmodified, could drive an ArduPilot/PX4 SITL instance instead of
the internal plant.
