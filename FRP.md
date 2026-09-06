# FRP — Final Project Proposal

**Course:** Introduction to Data-Driven Control of Drones
**Project:** Full Quaternion-Based Attitude Control for a Quadrotor
**Team:** Group 12 — Manohar Paturi, K Pushpak, Sai Krishna, B Sainath, Vishal
**School of Artificial Intelligence, Amrita Vishwa Vidyapeetham**

---

## 1. Base paper

E. Fresk and G. Nikolakopoulos, *"Full Quaternion Based Attitude Control for a
Quadrotor,"* 2013 European Control Conference (ECC), Zürich, pp. 3864–3869.
[DOI: 10.23919/ECC.2013.6669617](https://doi.org/10.23919/ECC.2013.6669617)

The paper designs a quadrotor attitude controller that lives **entirely in
quaternion space** — no Euler angles and no DCM anywhere in the control loop —
avoiding gimbal lock and the trigonometric cost of Euler-based laws. Attitude
error is the vector part of the quaternion product `q_ref ⊗ q_m*` (eq. 19), and
a derivative-free nonlinear P² law (eq. 21) maps it plus the measured body rate
directly to torque:

```
τ = −P_q · Axis_err(q_err) − P_ω · ω_m        (saturated at ±4 N·m)
```

## 2. Work completed so far

1. **Exact paper reproduction** (`quat_sitl/`): every equation (1)–(21)
   implemented function-for-function under the paper's own gains (P_q = 20,
   P_ω = 4), inertia (I_xx = I_yy = 6.5×10⁻⁴, I_zz = 1.2×10⁻³ kg·m²), torque
   bounds, and uniform ±0.1 measurement noise. All three benchmark scenarios
   (1 rad step, 0.5 rad / 1 rad/s sine, 2π flip) reproduce the paper's figures,
   and a 27-test suite guards every fidelity decision.
2. **Discrete-time stability analysis** (original contribution): a derivation
   showing the paper's gains are unstable at plausible digital control rates
   and that the minimum safe rate is ≈ 12.3 kHz, computed exactly from the
   closed-loop poles rather than by tuning.
3. **6-DOF PX4-style Python SITL** (`pysitl/`): the *unmodified* controller
   flying a full rigid-body vehicle — rotors, mixer, gravity, ground contact,
   uORB-style message bus, lockstep scheduler, arming/failsafes, altitude hold,
   and a browser ground station — with mass and geometry *derived* from the
   paper's published inertia.

## 3. Proposed project work

The controller currently flies only our own Python plant. The proposed work is
to **deploy the same unmodified quaternion controller on four industry- and
research-standard simulation stacks** and benchmark it against the paper's
scenarios on each:

| # | Simulator | Role in the evaluation | Nature |
|---|-----------|------------------------|--------|
| 1 | **gym-pybullet-drones** | RL-grade PyBullet physics; custom environment wrapping our controller | Python, runs natively |
| 2 | **MuJoCo** | High-accuracy contact dynamics; quadrotor MJCF model + torque/thrust actuators | Python, runs natively |
| 3 | **Gazebo** | The robotics-community standard; controller as an external Gazebo node speaking its transport | WSL2 / Linux |
| 4 | **ArduPilot SITL** | Real production flight-stack firmware; controller drives it over MAVLink, exactly as it would drive hardware | WSL2 / Linux |

### Why this matters

A control law is only credible when it survives contact with physics it was
not designed around. The paper's plant is an idealized double integrator with
a torque input; the four targets above add, progressively: realistic rotor
models and aerodynamic drag (PyBullet/MuJoCo), a full external simulation loop
with sensor transport (Gazebo), and production firmware with its own EKF,
motor mixing, and timing (ArduPilot). Demonstrating eq. (21) unchanged across
all four is strong evidence the quaternion law is fundamental, not an artifact
of a friendly plant.

### Architecture

One shared controller package, four thin adapters:

```
sim/
├── common/          # controller bridge: quat_sitl.NonlinearP2Controller + mixer,
│                    #   identical gains/scenarios/metrics everywhere
├── gym_pybullet/    # custom Aviary environment + QuatControl(BaseControl)
├── mujoco/          # quadrotor MJCF model + actuator bridge
├── gazebo/          # world + model + external controller node
└── ardupilot/       # SITL instance + pymavlink bridge (offboard attitude setpoints)
```

The controller gains, inertia, torque saturation, and the three benchmark
references are frozen at the paper's values in `sim/common/` and imported by
all four adapters — no per-simulator retuning is permitted. That constraint is
itself an experiment: if performance degrades on a richer simulator, the
degradation is signal about the law, not about our tuning.

### Per-simulator plan

**1. gym-pybullet-drones** — Install via pip; subclass `BaseControl` so the
aviary steps rotors while our law computes body torques from PyBullet's
quaternion state; run the step / sine / flip references; log attitude error,
torque, and settling time with the same CSV schema as `quat_sitl`.

**2. MuJoCo** — Author a quadrotor MJCF model (rigid body, 4 thrust/torque
actuators, ground contact); drive it from Python with the same controller at
the derived ≥ 12.3 kHz-safe control rate scaled to MuJoCo's timestep; replay
the three scenarios; compare tracking against the ideal plant.

**3. Gazebo** — Under WSL2: a quadrotor model (SDF) + empty world; the
controller runs as an external node subscribing to IMU/pose and publishing
body-torque commands, matching how a real companion computer would sit on a
vehicle; verify the same scenarios headless with logging.

**4. ArduPilot SITL** — Under WSL2: run ArduCopter SITL; a `pymavlink` bridge
sends attitude setpoints (and monitors motor outputs/EKF states) so the
unmodified `NonlinearP2Controller` replaces the stock attitude loop's
reference; demonstrate stable hover plus the three paper maneuvers; this is
the closest step to hardware without hardware.

### Cross-simulator benchmark (final evaluation deliverable)

For each scenario × simulator: attitude tracking plots (reference vs.
measured), torque time series with saturation marked, settling time, RMS
error, and the 360° flip gimbal-lock check (quaternion components stay
smooth). Everything lands in one comparison table and figure set in the final
report — the paper's claims re-verified on four independent physics engines.

## 4. Methodology

1. Freeze the shared controller/mixer/scenario package (`sim/common/`).
2. Implement each adapter behind a uniform runner CLI
   (`python -m sim.<stack> --scenario step|sine|flip --log`).
3. Continuous integration: existing 27 tests + new adapter smoke tests.
4. Results collected with one CSV/plot schema → automated cross-comparison.
5. Branching: work happens on per-member feature branches, merged into
   `main` only when tests pass; `main` always demo-ready.

## 5. Milestones

| Milestone | Content | Status |
|-----------|---------|--------|
| M0 | FRP.md pushed; repo baseline (reproduction + 6-DOF SITL) | done |
| M1 (update 1) | `sim/common/` bridge + gym-pybullet-drones and MuJoCo adapters running all three scenarios natively | proposed |
| M2 (update 2) | Gazebo and ArduPilot SITL adapters running under WSL2; unified logs | proposed |
| M3 (final) | Cross-simulator comparison, final report, demo videos | proposed |

## 6. Team division (indicative)

| Member | Ownership |
|--------|-----------|
| Manohar Paturi | `sim/common/` controller bridge + metrics harness |
| K Pushpak | gym-pybullet-drones adapter |
| Sai Krishna | MuJoCo adapter + MJCF model |
| B Sainath | Gazebo adapter (WSL2, SDF, transport) |
| Vishal | ArduPilot SITL + pymavlink bridge |

## 7. Expected outcome

The paper's quaternion P² law, with its published gains untouched, stabilizes
and flies the three benchmark maneuvers on all four stacks. Anticipated
findings: (i) tracking degrades gracefully as plant realism increases;
(ii) the derived control-rate bound remains the binding constraint on digital
targets; (iii) on ArduPilot the controller behaves as an outer-loop attitude
reference generator, exposing exactly what a hardware port would need.

## 8. References

[1] E. Fresk, G. Nikolakopoulos, "Full Quaternion Based Attitude Control for a
Quadrotor," *ECC 2013*, pp. 3864–3869.
[2] J. Panerati et al., "Learning to Fly — a Gym Environment with PyBullet
Physics for Reinforcement Learning of Multi-Agent Quadrotor Control," *ICRA
2021*.
[3] T. Erez, Y. Tassa, E. Todorov, "Simulation tools for model-based
robotics: verification of physical controllers," *Mechatronics 2015* (MuJoCo).
[4] N. Koenig, A. Howard, "Design and use paradigms for Gazebo, an open-source
multi-robot simulator," *IROS 2004*.
[5] ArduPilot Dev Team, *SITL (Software In The Loop)*, ardupilot.org.
