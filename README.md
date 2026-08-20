<p align="center">
  <img src="https://github.com/user-attachments/assets/060f7774-a73f-4132-9413-36887ed09cfa" alt="Amrita Vishwa Vidyapeetham" width="430">
</p>

<h1 align="center">Full Quaternion-Based Attitude Control for a Quadrotor</h1>

<p align="center">
  <b>A Python Software-In-The-Loop Reproduction, Stability Analysis, and 6-DOF Extension</b><br>
  Group 12 · School of Artificial Intelligence, Amrita Vishwa Vidyapeetham
</p>

<p align="center">
  <img alt="Python" src="https://img.shields.io/badge/python-3.10%2B-3776AB?logo=python&logoColor=white">
  <img alt="Tests" src="https://img.shields.io/badge/tests-27%2F27%20passing-brightgreen">
  <img alt="Stack" src="https://img.shields.io/badge/stack-numpy%20%7C%20scipy%20%7C%20matplotlib%20%7C%20pandas-9C27B0">
  <img alt="Base paper" src="https://img.shields.io/badge/base%20paper-ECC%202013-00599C">
</p>

---

## Overview

This project is a complete, tested, open-source reproduction of Fresk and Nikolakopoulos's
nonlinear **P² quaternion attitude controller** for a quadrotor, implemented **entirely in
quaternion space** — no Euler-angle or direction-cosine-matrix (DCM) computation anywhere in
the plant or control loop. Every equation of the paper, (1)–(21), is implemented
function-for-function and verified by a 27-test suite, including a literal preservation of
the paper's own sign convention on the rotational kinematics (eq. 18), which differs from
the general body-frame quaternion derivative (eq. 7).

Beyond reproduction, this work makes two original contributions:

1. **A discrete-time stability analysis the paper does not perform.** The paper's own tuned
   gains ($P_q=20$, $P_\omega=4$) combined with its published inertia produce a fast
   closed-loop pole (≈ −6150 rad/s) that a naively chosen digital control rate (200 Hz–1 kHz)
   samples **unstably**. The minimum stable rate (≈ 12.3 kHz) is *derived* from the exact
   zero-order-hold discretization — not tuned by trial — and confirmed against simulation.
2. **A 6-DOF PX4-style extension** (`pysitl/`) that flies the *same, unmodified* attitude
   controller on a real rigid-body vehicle — rotors, mixer, gravity, ground contact, a
   uORB-style message bus, multi-rate scheduler, arming/failsafe logic, and a browser-based
   ground station — with vehicle mass and geometry *derived algebraically* from the paper's
   own published inertia rather than assumed.

All three of the paper's benchmark scenarios — **step**, **sinusoidal tracking**, and
**360° flip** — are reproduced quantitatively and qualitatively, including the absence of
any gimbal-lock artifact through the full-rotation flip, matching the paper's central claim.

### Base paper

> E. Fresk and G. Nikolakopoulos, *"Full Quaternion Based Attitude Control for a Quadrotor,"*
> in **2013 European Control Conference (ECC)**, Zürich, Switzerland, Jul. 2013, pp. 3864–3869.
> DOI: [10.23919/ECC.2013.6669617](https://doi.org/10.23919/ECC.2013.6669617) ·
> [Open-access full text (DiVA portal)](https://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf)

## Team

| Name | Roll No. | Email |
|---|---|---|
| Manohar Paturi | CB.SC.U4AIE24339 | cb.sc.u4aie24339@cb.students.amrita.edu |
| K Pushpak | CB.SC.U4AIE24328 | cb.sc.u4aie24328@cb.students.amrita.edu |
| Sai Krishna | CB.SC.U4AIE24308 | cb.sc.u4aie24308@cb.students.amrita.edu |
| B Sainath | CB.SC.U4AIE24309 | cb.sc.u4aie24309@cb.students.amrita.edu |
| Vishal | CB.SC.U4AIE24363 | cb.sc.u4aie24363@cb.students.amrita.edu |

## What's in this repository

| Path | Contents |
|---|---|
| [`quat_sitl/`](quat_sitl/) | **The paper reproduction** — quaternion algebra (eqs. 1–16), attitude plant (eqs. 17–18), nonlinear P² controller (eqs. 19–21), sensor noise, the three benchmark scenarios, plotting, and an animated 3D replay viewer. Attitude-only, exactly like the paper. |
| [`pysitl/`](pysitl/) | **The 6-DOF PX4-style extension** — real rotors + mixer + gravity + ground contact, uORB-style message bus, multi-rate scheduler, arming/failsafe commander, altitude hold, autopilot modes, CSV logging, analysis plots, and a browser ground station. Flies the **unmodified** `quat_sitl` controller. |
| [`scenarios/`](scenarios/) | Runnable benchmark scripts (step / sine / flip). |
| [`tests/`](tests/) | 27 tests: 7 for the paper reproduction, 20 for the 6-DOF extension. |
| [`docs/figures/`](docs/figures/) | Generated result figures used in the report and below. |
| [`REPORT.md`](REPORT.md) | Full technical report: methodology, stability analysis, results, discussion, references. |
| [`report1.md`](report1.md) | Project report #1: team info, base-paper discussion, methodology, eqs. 1–21. |
| [`file_structure.md`](file_structure.md) | Complete file-by-file map, with every equation cited to the exact function that implements it. |
| [`fresk_nikolakopoulos_precise_attitude_simulator_xyz_labels(1).html`](fresk_nikolakopoulos_precise_attitude_simulator_xyz_labels%281%29.html) | Standalone zero-dependency browser simulator of eqs. 1–21 (single self-contained HTML file). |

## Quickstart

```bash
pip install -e .
pytest tests/ -v                                    # 27/27 pass

# Paper reproduction (quat_sitl) — writes timestamped CSV + PNGs to results/
python -m quat_sitl.simulator --scenario step --duration 15 --seed 0 --noise 0.1
python scenarios/step_response.py
python scenarios/sine_tracking.py
python scenarios/flip_360.py
python -m quat_sitl.visualize3d                     # animated 3D replay of a logged run

# 6-DOF extension (pysitl)
python -m pysitl.run --gcs                          # interactive browser ground station → http://127.0.0.1:8765
python -m pysitl.run --mode auto_step --duration 15 --log   # headless paper scenarios
python -m pysitl.run --mode auto_sine --duration 15 --log
python -m pysitl.run --mode auto_flip --duration 5 --log
```

In the ground station: **Arm**, pick a flight mode, and fly with `W/S` (pitch), `A/D`
(roll), `Q/E` (yaw rate), `↑/↓` (throttle). Full reference: [`pysitl/UI_GUIDE.md`](pysitl/UI_GUIDE.md).
A convenience launcher that picks a Python interpreter with the dependencies installed is
provided: `bash run_gcs.sh [port]`.

## Results vs. paper

All numeric parameters stated in the paper are reproduced exactly:

| Quantity | Paper value | Implemented | Match |
|---|---|---|---|
| $I_{xx}=I_{yy}$ | $6.5\times10^{-4}\ \mathrm{kg\,m^2}$ | $6.5\times10^{-4}$ | Exact |
| $I_{zz}$ | $1.2\times10^{-3}\ \mathrm{kg\,m^2}$ | $1.2\times10^{-3}$ | Exact |
| $P_q$ / $P_\omega$ | 20 / 4 | 20 / 4 | Exact |
| Torque saturation | ±4 N·m | ±4 N·m | Exact |
| Sine amplitude / frequency | 0.5 rad, 1 rad/s | 0.5 rad, 1 rad/s | Exact |
| Flip range | 0 → 2π rad | 0 → 2π rad | Exact |
| Noise amplitude | 0.1 | 0.1 | Exact |

### Step response

A 1 rad step is commanded on $\phi$, $\theta$, $\psi$ in turn, staggered at
$t = 1, 5, 9$ s over a 15 s run. Essentially no overshoot; each axis settles within
≈ 2 s — matching the paper's Fig. 3/4 ("very small overshoot … errors go quickly to zero").

<p align="center">
  <img src="docs/figures/step_attitude.png" width="620" alt="Step response: reference (dashed) vs. output (solid) for roll, pitch, yaw">
</p>
<p align="center">
  <img src="docs/figures/step_torque.png" width="620" alt="Step response commanded torque, briefly saturating at each step onset">
</p>

### Sinusoidal tracking

A 0.5 rad, 1 rad/s sine on all three axes, phase-shifted so torques are never in phase.
Measured phase lag ≈ 0.4–0.5 s, matching the paper's stated "about 0.5 second" almost
exactly; torque stays in the linear region throughout steady-state tracking (paper Fig. 5–6).

<p align="center">
  <img src="docs/figures/sine_attitude.png" width="620" alt="Sine tracking: reference vs. output">
</p>
<p align="center">
  <img src="docs/figures/sine_torque.png" width="620" alt="Sine tracking torque, unsaturated in steady state">
</p>

### 360° flip

With the shortest-path correction deliberately disabled, the reference ramps 0 → 2π rad
about the body $x$-axis over 2 s. The raw quaternion components $q_0, q_1$ remain smooth
throughout the full rotation with **no discontinuity** — the paper's central claim — even
though the *plotted* Euler angle $\phi$ necessarily wraps at ±π, an inherent property of
that display transform, not of the controller (paper Fig. 7–8).

<p align="center">
  <img src="docs/figures/flip_attitude.png" width="620" alt="360° flip: wrapped Euler roll alongside the smooth raw q0, q1">
</p>
<p align="center">
  <img src="docs/figures/flip_animation.gif" width="480" alt="Animated 3D playback of the 360° flip">
</p>

## Key finding: the control-rate stability bound

The paper reports continuous-time-style results without specifying a digital control rate.
Implementing a plausible embedded design point (200 Hz control / 1 kHz plant) exposes a
genuine problem: the loop **never converges for any step size**. Linearizing the closed
loop about identity attitude gives

$$I\,\ddot e + P_\omega\,\dot e + \tfrac{P_q}{2}\,e = 0
\qquad\Rightarrow\qquad
\lambda_{1,2} = \frac{-P_\omega \pm \sqrt{P_\omega^2 - 2 I P_q}}{2I}$$

with the paper's own numbers: $\lambda_1 \approx -2.5\ \mathrm{rad/s}$ (matches the paper's
few-second settling) and $\lambda_2 \approx -6150\ \mathrm{rad/s}$. The continuous-time loop
is heavily overdamped ($\zeta \approx 25$) and well-behaved — the instability is purely a
**sampling artifact of the fast pole**. The exact zero-order-hold discrete closed-loop
spectral radius $\rho(A_{cl})$ at candidate rates:

| Control rate | $\rho(A_{cl})$ | Stable? |
|---|---|---|
| 200 Hz | 29.9 | No |
| 1 kHz | 5.16 | No |
| 10 kHz | 0.9997 | Marginal |
| 100 kHz | 0.99997 | Yes |

The minimum stable rate is derived (not tuned) as $f_{control} \ge P_\omega / (m \cdot I_{\min})$;
with margin $m = 0.5$ this is **≈ 12.3 kHz** for the paper's defaults.
`quat_sitl/dynamics.py: stable_control_rate_hz` computes it from whatever gains and inertia
are configured, the simulator uses it as the default control rate (state still logged at
exactly 1 kHz, all gains unchanged), and the 6-DOF scheduler *refuses to run* below it.
Pass `control_rate_hz=200.0` to `run_simulation` to reproduce the instability directly.
Full derivation: [`REPORT.md` §2.5](REPORT.md).

## Equation → function map

| Paper eq. | Description | Location |
|---|---|---|
| (1)–(2) | Quaternion representation, Kronecker product, `Q`/`Q̄` matrices | `quaternion.mul`, `quaternion.Q`, `quaternion.Q_bar` |
| (3) | Norm | `quaternion.norm` |
| (4) | Conjugate | `quaternion.conj` |
| (5) | Inverse | `quaternion.inv` |
| (6) | q̇, fixed-frame ω | `quaternion.qdot_fixed` |
| (7) | q̇, body-frame ω′ | `quaternion.qdot_body` |
| (8) | Vector rotation via q ⊗ v ⊗ q* | `quaternion.rotate` |
| (9)–(12) | DCM columns Rx, Ry, Rz | `quaternion.to_dcm` |
| (13) | DCM transpose (frame rotation) | `quaternion.to_dcm(q).T` |
| (14) | Axis-angle → quaternion | `quaternion.from_axis_angle` |
| (15) | Euler → quaternion | `quaternion.from_euler` |
| (16) | Quaternion → Euler (plotting only) | `quaternion.to_euler` |
| (17)–(18) | Rigid-body plant | `dynamics.state_derivative`, `dynamics.InertiaParams` |
| (19)–(21) | Nonlinear P² controller | `controller.NonlinearP2Controller.compute_torque` |

Equation (16) is used **only** for plotting and logging — the entire closed loop operates
on $q$ directly.

## Fidelity notes

- **Eq. (18)'s sign, preserved deliberately.** Eq. (18) reads $\dot q = -\tfrac12[0,\omega]^T \otimes q$ —
  the opposite sign from the general body-frame form in eq. (7). Verified against the
  published PDF and kept literally in `dynamics.py` (not silently "fixed"), with a dedicated
  test that would fail loudly if either eq. (18)'s or eq. (21)'s sign were wrong. See
  [`REPORT.md` §4.4](REPORT.md) for the full discussion.
- **Paper ambiguities, resolved with documented defaults.** Noise distribution (uniform
  $\mathcal{U}(-0.1, 0.1)$), step stagger timing (x@1 s, y@5 s, z@9 s), sine phase offsets
  ($\phi_y = 0$, $\phi_z = \pi/2$), flip ramp duration (2 s), and the DCM convention
  (eq. 12, column form, pinned by test). Every resolution with its rationale:
  [`REPORT.md` §3.3](REPORT.md).
- **Control-signal → torque is the identity**, per the paper's own stated simplification
  (`dynamics.IdentityMotorModel`).

## The 6-DOF extension (`pysitl`)

The paper is attitude-only and never needs mass or geometry — so both are *derived* from the
paper's own inertia: $I_{zz}/I_{xx} = 1.846$ is close to the exactly-2 of a planar-X
point-mass quadrotor, giving arm length $L = 0.15$ m, rotor mass ≈ 12.2 g, total mass
≈ 200 g (with an assumed 150 g central body), hover thrust ≈ 1.95 N.

The unmodified controller — same gains, same noise model — flies this real rigid-body
vehicle through a PX4-shaped architecture: uORB-style typed pub/sub bus, deterministic
multi-rate lockstep scheduler (16 kHz base: physics + attitude + mixer; 1 kHz sensors;
250 Hz altitude; 50 Hz commander; 200 Hz logger), arming/flight-mode commander with
failsafes, a mixer with thrust-preserving desaturation, ULog-style CSV logging, and a
stdlib browser ground station with live SSE telemetry. Under the paper's full 0.1 noise, a
commanded 0.3 rad roll converges to 0.309 rad while holding altitude to within centimeters.

Architecture, design decisions, and the derivation of the vehicle from two published
numbers: [`pysitl/README.md`](pysitl/README.md) · dashboard manual: [`pysitl/UI_GUIDE.md`](pysitl/UI_GUIDE.md).

## Testing

| Suite | Tests | Coverage |
|---|---|---|
| `tests/test_quaternion.py` | 7 | Non-commutativity, identity, DCM orthonormality/round-trip, rotation consistency, 10 s norm-drift bound, zero-input fixed point, closed-loop sign-consistency |
| `tests/test_sitl.py` | 20 | Body↔world frame convention, eq. 18 numerical parity, hover equilibrium, ground contact, mixer round-trip, thrust-preserving desaturation, noise bounds, closed-loop tracking under full paper noise, scheduler rate-rejection and determinism, arming/failsafe, message-bus semantics, full-rotation flip completion |

The frame-convention and desaturation tests are direct regressions for issues discovered
and fixed during development. Run everything with `pytest tests/ -v`.

## References

[1] E. Fresk and G. Nikolakopoulos, "Full Quaternion Based Attitude Control for a Quadrotor,"
in *2013 European Control Conference (ECC)*, Zürich, Switzerland, Jul. 2013, pp. 3864–3869,
doi: [10.23919/ECC.2013.6669617](https://doi.org/10.23919/ECC.2013.6669617).

[2] J. B. Kuipers, *Quaternions and Rotation Sequences*. Princeton University Press, 1998.

[3] J. Diebel, "Representing Attitude: Euler Angles, Unit Quaternions, and Rotation
Vectors," Stanford University, 2006.

[4] S. Bouabdallah and R. Siegwart, "Full Control of a Quadrotor," in *2007 IEEE/RSJ
International Conference on Intelligent Robots and Systems*, 2007, pp. 153–158.

[5] A. Tayebi and S. McGilvray, "Attitude Stabilization of a VTOL Quadrotor Aircraft,"
*IEEE Transactions on Control Systems Technology*, vol. 14, no. 3, pp. 562–571, 2006.

[6] PX4 Development Team, *PX4 Autopilot User Guide* — architecture reference for the
multi-rate scheduler, uORB message-bus pattern, and commander/arming model:
[docs.px4.io](https://docs.px4.io/).

[7] This repository: implementation, test suite, and all figures — `quat_sitl/` (paper
reproduction) and `pysitl/` (6-DOF PX4-style extension), with an equation-to-function map
and additional implementation notes in [`file_structure.md`](file_structure.md),
[`REPORT.md`](REPORT.md), and [`pysitl/README.md`](pysitl/README.md).
