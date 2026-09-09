# File Structure — Full Quaternion-Based Attitude Control for a Quadrotor

Complete map of this repository: every coding file from root to leaves, what each
contains, the results the project produces, the three UIs, and — at the end — every
equation used in `report1.md` (Eqs. 1–21) cited to the exact file (and function) that
implements it, plus the extension equations (22–30) that appear only in `REPORT.md`.

The repository is a Python software-in-the-loop (SITL) reproduction and extension of
Fresk & Nikolakopoulos, *"Full Quaternion Based Attitude Control for a Quadrotor"*
(ECC 2013). It contains **two Python packages**:

| Package | Role |
|---|---|
| `quat_sitl/` | The paper reproduction: quaternion algebra (Eqs. 1–16), attitude plant (Eqs. 17–18), nonlinear P² controller (Eqs. 19–21), sensor noise, the three benchmark scenarios, plotting, and a 3D replay viewer. Attitude-only, exactly like the paper. |
| `pysitl/` | A PX4-style 6-DOF extension: real rotors + mixer + gravity + ground contact, a uORB-style message bus, multi-rate scheduler, arming/failsafe commander, altitude hold, autopilot modes, CSV logging, analysis plots, and a browser ground station. Flies the **unmodified** `quat_sitl` controller. |
| `sim/` | The four-simulator deployment (FRP.md): the same unmodified controller flying gym-pybullet-drones, MuJoCo, Gazebo (WSL2), and ArduPilot SITL (WSL2) through one shared bridge (`q_paper = conj(q_sim)`), one telemetry schema, and one cross-simulator benchmark (`python -m sim.compare`). |

Plus `scenarios/` (runnable benchmark scripts), `tests/` (41 tests), `docs/figures/`
(generated result figures), and one standalone browser simulator HTML file.

---

## 1. Full file tree (root → leaves)

Coding files are the tree's content; generated/doc artifacts are marked ⚙ (output) or
📄 (documentation). `results/` does not exist in the repo — it is created at runtime by
the scenario scripts and is git-ignored.

```text
CD_12_Full-Quaternion-Based-Attitude-Control-for-a-Quadrotor-main/
│
├── pyproject.toml                          # package build config (pip install -e .)
├── requirements.txt                        # pip dependencies
├── run_gcs.sh                              # launcher: picks a Python that has the deps,
│                                           #   starts `pysitl.run --gcs`, opens the browser
├── .gitignore                              # ignores caches + generated results/
├── README.md                               📄 quat_sitl docs: run instructions, eq→function map,
│                                           #   results-vs-paper discussion, stability analysis
├── REPORT.md                               📄 full technical report (extends report1.md with
│                                           #   Eqs. 22–30, test tables, discussion, references)
├── report1.md                              📄 project report #1 (team info, methodology,
│                                           #   Eqs. 1–21, expected/measured results)
├── file_structure.md                       📄 this file
├── fresk_nikolakopoulos_precise_attitude_simulator_xyz_labels(1).html
│                                           # standalone zero-dependency browser simulator of
│                                           #   Eqs. 1–21 (single self-contained HTML+JS file)
│
├── quat_sitl/                              # ── PACKAGE 1: paper reproduction ──────────────
│   ├── __init__.py                         #   version string only
│   ├── quaternion.py                       #   quaternion algebra, Eqs. 1–16 (numpy, batched)
│   ├── dynamics.py                         #   rigid-body plant Eqs. 17–18, torque saturation,
│   │                                       #   motor models, control-rate stability helpers
│   ├── controller.py                       #   nonlinear P² controller, Eqs. 19–21
│   ├── sensors.py                          #   paper's noise model (uniform ±0.1)
│   ├── references.py                       #   step / sine / flip reference generators
│   ├── integrator.py                       #   generic RK4 stepper
│   ├── simulator.py                        #   main SITL loop + CLI, CSV logging at 1 kHz
│   ├── plotting.py                         #   paper-figure-style matplotlib plots
│   └── visualize3d.py                      #   animated 3D quadrotor replay (matplotlib)
│
├── scenarios/                              # ── runnable benchmark scripts ─────────────────
│   ├── step_response.py                    #   paper Figs. 3–4: staggered 1 rad steps, 15 s
│   ├── sine_tracking.py                    #   paper Figs. 5–6: 0.5 rad / 1 rad/s sines, 15 s
│   └── flip_360.py                         #   paper Figs. 7–8: 0→2π flip about x, 5 s
│
├── pysitl/                                 # ── PACKAGE 2: PX4-style 6-DOF extension ────────
│   ├── __init__.py                         #   package docstring + version
│   ├── params.py                           #   vehicle derivation from paper inertia, gains, rates
│   ├── plant.py                            #   13-state 6-DOF rigid body (scalar hot path)
│   ├── airframe.py                         #   rotor geometry, mixer, desaturation, motor lag
│   ├── sensors.py                          #   scalar/seeded copy of the paper's noise model
│   ├── bus.py                              #   uORB-style typed pub/sub message bus
│   ├── topics.py                           #   message types + topic-name constants
│   ├── scheduler.py                        #   multi-rate deterministic lockstep scheduler
│   ├── modes.py                            #   flight modes + arming/preflight/failsafe commander
│   ├── sim.py                              #   Simulation orchestrator (wires everything)
│   ├── recorder.py                         #   ULog-style flight logger → CSV
│   ├── analysis.py                         #   post-flight plots (attitude/altitude/torque/track)
│   ├── run.py                              #   CLI entry point (--gcs or headless --mode)
│   ├── README.md                           📄 pysitl architecture docs
│   ├── UI_GUIDE.md                         📄 ground-station user guide
│   ├── control/
│   │   ├── __init__.py                     #   re-exports
│   │   ├── attitude.py                     #   thin adapter over the UNMODIFIED paper controller
│   │   ├── altitude.py                     #   altitude PID + tilt compensation (new, not paper)
│   │   ├── manual.py                       #   sticks → attitude reference / thrust
│   │   └── autopilot.py                    #   paper's step/sine/flip as AUTO flight modes
│   └── gcs/
│       ├── __init__.py                     #   (empty)
│       ├── server.py                       #   stdlib HTTP server: SSE telemetry + POST controls
│       └── static/
│           └── index.html                  #   browser ground station (single-file HTML/JS/CSS)
│
├── tests/
│   ├── test_quaternion.py                  # 7 tests: algebra + closed-loop sign guardrail
│   ├── test_sitl.py                        # 20 tests: pysitl frame/mixer/scheduler/safety/...
│   ├── test_sim_bridge.py                  # 8 tests: sim/ bridge (conjugate-convention proof,
│   │                                       #   priority mixer, altitude hold, rpm conversion)
│   └── test_sim_adapters.py                # 4 tests: MuJoCo + gym-pybullet adapter smoke,
│                                           #   PyBullet actuation calibration (stack-skipping)
│
├── sim/                                    # ── PACKAGE 3: four-simulator deployment (FRP) ───
│   ├── README.md                           📄 architecture, runbook, findings, conventions
│   ├── compare.py                          #   cross-simulator table → results/comparison.md
│   ├── run_all.sh                          #   one-command benchmark reproduction (native stacks)
│   ├── common/
│   │   ├── __init__.py                     #   re-exports
│   │   ├── bridge.py                       #   frozen paper controller + q_paper=conj(q_sim)
│   │   │                                   #     convention, z-up priority mixer, altitude hold
│   │   ├── scenarios.py                    #   paper scenarios re-export + 6-DOF flip-ramp note
│   │   ├── telemetry.py                    #   one CSV schema + uniform metrics for all stacks
│   │   └── plots.py                        #   standard attitude/torque figure pair per run
│   ├── mujoco/
│   │   ├── quadrotor.xml                   #   paper vehicle MJCF (20 kHz, contact floor)
│   │   └── run.py                          #   adapter: sensors→conj→controller→mixer→wrench
│   ├── gym_pybullet/
│   │   ├── paper_quad.urdf                 #   paper vehicle as a gym-pybullet-drones asset
│   │   └── run.py                          #   adapter: CtrlAviary at 24 kHz, rotor-order +
│   │                                       #     yaw-pairing mapping to the package's _physics
│   ├── gazebo/
│   │   ├── README.md                       📄 WSL2 runbook (gz Harmonic + ardupilot_gazebo)
│   │   ├── run_gazebo.sh                   #   one-shot world+SITL+bridge launch
│   │   └── worlds/paper_attitude.world     #   gz world spawning the ardupilot_gazebo iris
│   └── ardupilot/
│       ├── bridge_node.py                  #   pymavlink bridge: SET_ATTITUDE_TARGET @ 50 Hz,
│       │                                   #     q_cmd = conj(q_ref), ω_des = −(Pq/Pω)·axis_err
│       └── requirements.txt                #   pymavlink & co
│
├── docs/figures/                           ⚙ generated result figures (embedded in reports)
│   ├── step_attitude.png                   #   step: φ/θ/ψ vs reference (paper Fig. 3)
│   ├── step_torque.png                     #   step: Mx/My/Mz with ±4 N·m lines (paper Fig. 4)
│   ├── sine_attitude.png                   #   sine tracking attitude (paper Fig. 5)
│   ├── sine_torque.png                     #   sine tracking torque (paper Fig. 6)
│   ├── flip_attitude.png                   #   flip: wrapped φ + raw q0/q1 (paper Figs. 7–8)
│   └── flip_animation.gif                  #   animated 3D playback of the flip
│
└── results/                                ⚙ created at runtime, git-ignored:
                                            #   {step,sine,flip}_<timestamp>.csv,
                                            #   *_latest.csv, *_euler.png, *_torque.png,
                                            #   pysitl_<mode>_<timestamp>.csv + analysis PNGs
```

---

## 2. Root-level files

### `pyproject.toml`
Setuptools build config for package `quat-sitl` v0.1.0, Python ≥ 3.10. Declares the
dependencies `numpy>=1.26`, `scipy>=1.11`, `matplotlib>=3.8`, `pandas>=2.1`, and
restricts `setuptools.packages.find` to `quat_sitl*` (so `pip install -e .` installs
only the reproduction package; `pysitl` runs from the repo). Note: `scipy` is declared
but never actually imported by any module — a vestigial dependency.

### `requirements.txt`
Flat pip list mirroring `pyproject.toml` plus `pytest>=7.4` for the test suite.

### `.gitignore`
Ignores Python caches (`__pycache__/`, `*.pyc`, `*.egg-info/`, `.pytest_cache/`), build
dirs, and everything generated under `results/` (CSV/PNG/GIF/MP4), keeping the repo to
source + committed figures only.

### `README.md` 📄
The `quat_sitl` package documentation. Contains: install/run commands; the canonical
**equation → function map** (Eqs. 1–21 → `quaternion.py`/`dynamics.py`/`controller.py`
functions); results-vs-paper discussion for all three scenarios; the five documented
paper ambiguities and their chosen defaults (noise distribution, step stagger, sine
phase, flip ramp, DCM convention); the eq. (18) sign-convention discussion; and the full
discrete-time **control-rate stability analysis** (why 200 Hz / 1 kHz control ticks are
provably unstable for the paper's gains, and why `stable_control_rate_hz` ≈ 12.3 kHz is
used instead, with state still logged at 1 kHz).

### `REPORT.md` 📄
The full technical report: *"…A Python Software-In-The-Loop Reproduction, Stability
Analysis, and 6-DOF Extension."* Restates Eqs. 1–21 with full derivations and **adds
equations not in the paper** — Eq. 22 (noise model), Eqs. 23–24 (linearized closed loop
and its poles), Eq. 25 (exact ZOH discretization + spectral-radius table), Eq. 26
(minimum stable control rate), Eqs. 27–28 (vehicle mass/geometry derivation from the
paper's inertia), Eq. 29 (forward mixer), Eq. 30 (thrust-preserving desaturation) —
plus results tables, 27-test summary, discussion sections, and 7 references.

### `report1.md` 📄
Project report #1 (the one this document cross-references): group-12 team table,
abstract, base-paper metadata/discussion, problem statement, objectives, control-loop
block diagram, methodology walk-through of Eqs. 1–21, expected results, measured
results vs. the paper (parameter table, step/sine/flip comparison tables with figures),
and conclusion. **All equations cited in Section 8 below come from this file.**

### `fresk_nikolakopoulos_precise_attitude_simulator_xyz_labels(1).html`
A completely self-contained (no server, no dependencies) **browser simulator** of the
paper, in one HTML file (~29 KB): 3D attitude view with X/Y/Z axis labels, live
attitude + torque strip charts, benchmark buttons (Step / Sine / Flip / Manual), manual
φ/θ/ψ reference sliders, live controller sliders (Pq, Pω, τ_max), noise
amplitude/seed, speed control, pause/reset, CSV export, auto-loop, and an "equation
audit" panel. Its JavaScript reimplements, from scratch: quaternion multiply /
conjugate / norm / inverse (Eqs. 1–5), the DCM (Eqs. 9–13), axis-angle and Euler
conversions (Eqs. 14–16), the eq. (18) plant **with the literal leading-minus sign**
(`deriv()`), RK4 integration, the Eqs. 19–21 controller (`torque()`), a seeded
deterministic uniform noise generator, and an adaptive physics step `stableDt(Pw)`
recomputed from the current Pω — the browser analogue of
`quat_sitl.dynamics.stable_control_rate_hz`.

---

## 3. `quat_sitl/` — the paper reproduction package

### `quat_sitl/__init__.py`
One line: `__version__ = "0.1.0"`.

### `quat_sitl/quaternion.py` — *Eqs. 1–16, the entire quaternion algebra*
Numpy implementation, scalar-first convention `q = [q0, q1, q2, q3]`, left-hand
notation. Every function transparently accepts a single quaternion or a batch
`(N,4)`. Functions (line numbers):

| Function | Line | Implements |
|---|---|---|
| `mul(p, q)` | 29 | Eq. 2 Hamilton/Kronecker product (non-commutative) |
| `Q(p)` | 47 | Eq. 2 left-multiplication matrix `Q(p) @ q == mul(p,q)` |
| `Q_bar(q)` | 60 | Eq. 2 right-multiplication matrix `Q̄(q) @ p == mul(p,q)` |
| `norm(q)` | 72 | Eq. 3 |
| `conj(q)` | 79 | Eq. 4 (negate vector part) |
| `inv(q)` | 86 | Eq. 5 `q*/‖q‖²` |
| `normalize(q)` | 96 | renormalization after integration/noise (support) |
| `qdot_fixed(q, ω)` | 106 | Eq. 6 `q̇ = ½ q⊗[0,ω]`, world-frame ω |
| `qdot_body(q, ω′)` | 116 | Eq. 7 `q̇ = ½ [0,ω′]⊗q`, body-frame ω (positive sign) |
| `rotate(q, v)` | 131 | Eq. 8 sandwich product `q⊗[0,v]⊗q*` |
| `to_dcm(q)` | 141 | Eqs. 9–12 DCM, columns Rx/Ry/Rz (eq. 13 = `.T`) |
| `from_axis_angle(axis, α)` | 162 | Eq. 14 `cos(α/2) + u·sin(α/2)` |
| `from_euler(φ, θ, ψ)` | 177 | Eq. 15 ZYX aerospace Euler → quaternion |
| `to_euler(q)` | 194 | Eq. 16 quaternion → Euler — **plotting/logging only** |

`to_euler`'s docstring explicitly forbids use in `dynamics.py`/`controller.py` — the
whole point of the paper is that the closed loop never leaves quaternion space.

### `quat_sitl/dynamics.py` — *Eqs. 17–18, the plant*
- `InertiaParams` (line 27): frozen dataclass with the paper's CAD-derived diagonal
  inertia `Ixx=Iyy=6.5e-4`, `Izz=1.2e-3 kg·m²`.
- `MotorModel` / `IdentityMotorModel` / `FirstOrderLagMotorModel` (38–71): the
  control-signal→torque relation. The paper simplifies it to identity; the first-order
  lag is an unwired, swappable stub.
- `torque_saturate(τ, limit=4.0)` (74): elementwise ±4 N·m clip (paper Section V).
- `stable_rk4_substeps(Pw, inertia, dt)` (81): how many RK4 sub-steps keep the fast
  rate-feedback eigenvalue (≈ Pw/I_min ≈ 6150 rad/s) inside explicit RK4's stability
  region at a given dt.
- `stable_control_rate_hz(Pw, inertia, margin=0.5)` (103): the **minimum stable
  digital control-tick rate** — Eq. 26 — ≈ 12.3 kHz for the paper's defaults. Docstring
  carries the full ZOH spectral-radius derivation (29.9 at 200 Hz, 5.16 at 1 kHz → both
  unstable).
- `state_derivative(state, τ, inertia)` (124): **Eq. 18 itself** —
  `q̇ = −½[0,ω]⊗q` (the paper's literal leading **minus** sign, kept visible here and
  deliberately *not* delegated to `qdot_body`) and `ω̇ = I⁻¹τ − I⁻¹[ω×(Iω)]` (the
  rotational half of Eq. 17).

### `quat_sitl/controller.py` — *Eqs. 19–21, the nonlinear P² controller*
- `ControllerGains` (17): `Pq=20`, `Pw=4` (paper Section V).
- `NonlinearP2Controller.compute_torque(q_ref, q_m, ω_m)` (33): Eq. 19 error quaternion
  `q_err = q_ref ⊗ q_m*` (line 36); Eq. 20 vector part `Axis_err = q_err[1:4]` (37)
  with the shortest-path sign flip when `q_err[0] < 0` (39–40, disabled for the flip);
  Eq. 21 `τ = −Pq·Axis_err − Pω·ω_m` (43). Returns **unsaturated** torque; saturation
  is applied downstream by `dynamics.torque_saturate`. Derivative-free, no integrator.

### `quat_sitl/sensors.py` — *the paper's noise model (Eq. 22 of REPORT.md)*
`SensorModel.measure` (24): adds zero-mean **uniform** U(−0.1, 0.1) noise to the true
quaternion (then renormalizes) and to the body rates. The uniform choice is a
documented assumption ("amplitude" implies a hard bound; a Gaussian has none).

### `quat_sitl/references.py` — scenario reference generators
Each maps scalar t → unit quaternion, always built via `from_euler`/`from_axis_angle`:
- `step_reference` (21): 1 rad steps on φ/θ/ψ staggered x@1 s, y@5 s, z@9 s
  (`DEFAULT_STEP_AXIS_TIMES`) — paper says only "different time instants".
- `sine_reference` (33): 0.5 rad, 1 rad/s sines on all axes with `phase_z = π/2` so
  torques are not in phase — phase values are documented assumptions.
- `flip_reference` (49): ramps 0→2π rad about the body x-axis over 2 s (assumed
  duration), then holds — built from `from_axis_angle` (Eq. 14).

### `quat_sitl/integrator.py` — generic RK4
`rk4_step` (13): classic 4-stage Runge–Kutta, physics-agnostic. `rk4_integrate` (23):
advances by dt using n internal sub-steps (for numerical stability of the fast
rate-feedback pole).

### `quat_sitl/simulator.py` — the main SITL loop + CLI
`run_simulation` (44): the closed loop. Controller runs at
`stable_control_rate_hz` (≈12.3 kHz; pass `control_rate_hz=200` explicitly to watch the
spec-literal rate go unstable) while **state is logged once per 1 ms** (1 kHz).
Per control sub-tick: sample reference → measure (noisy) → compute torque → motor
model → saturate → RK4 plant step → renormalize. Logs one wide row per ms with
q_ref/q_meas/q_true, ω, ω_meas, τ, saturation flags, and Euler angles (via Eq. 16,
logging only). `save_run` (126) writes a timestamped CSV + `*_latest.csv` to
`results/`. `_build_cli_config` (135) maps `--scenario step|sine|flip` to the right
reference/controller (flip uses `shortest_path=False`). `main` (153) is the argparse
CLI (`python -m quat_sitl.simulator`).

### `quat_sitl/plotting.py` — paper-style figures
`plot_scenario` (12): two stacked 3-row figures (φ/θ/ψ dashed-reference vs
solid-output; Mx/My/Mz with ±4 N·m guides) matching paper Figs. 3–4 / 5–6.
`plot_flip` (46): 2-row figure matching paper Figs. 7–8 — wrapped Euler φ on top,
**raw q0/q1 below** to show singularity-free quaternions through the full rotation.

### `quat_sitl/visualize3d.py` — interactive 3D replay
`animate_flight` (41): matplotlib `FuncAnimation` replaying a logged CSV (or live
DataFrame) as a quadrotor — X-configuration arms from `_ARM_TIPS_BODY`, RGB body-axis
triad — attitude applied via `quat.to_dcm` (Eqs. 9–12). CLI (`main`, 103):
`python -m quat_sitl.visualize3d --scenario flip|--csv path [--fps --speed --save]`.
This is what rendered `docs/figures/flip_animation.gif`.

---

## 4. `scenarios/` — one runnable script per paper benchmark

Each script wires the exact paper configuration (seed 0, noise 0.1, identity motor,
paper inertia/gains), runs `run_simulation`, saves CSV + figures to `results/`:

| Script | Scenario | Duration | Controller | Output |
|---|---|---|---|---|
| `step_response.py` | staggered 1 rad steps (Figs. 3–4) | 15 s | `NonlinearP2Controller()` | step CSV + `_euler.png` + `_torque.png` |
| `sine_tracking.py` | 0.5 rad / 1 rad/s sines (Figs. 5–6) | 15 s | `NonlinearP2Controller()` | sine CSV + figures; warns if saturation occurred |
| `flip_360.py` | 0→2π flip about x (Figs. 7–8) | 5 s | `NonlinearP2Controller(shortest_path=False)` | flip CSV + `_flip.png` |

---

## 5. `pysitl/` — the PX4-style 6-DOF extension

### `pysitl/__init__.py`
Package docstring (paper-faithful content, PX4-faithful architecture) + version.

### `pysitl/params.py` — vehicle derivation + all parameters
Module docstring derives mass/geometry **from the paper's inertia** (Eqs. 27–28):
`Izz/Ixx = 1.846 ≈ 2` ⇒ planar-X point-mass layout; with L = 0.15 m ⇒ rotor mass
12.2 g, and with an assumed 150 g central body ⇒ total mass ≈ 200 g, hover thrust
≈ 1.95 N. Contains:
- `VehicleParams` (43): paper inertia verbatim (`PAPER_IXX/IYY/IZZ`,
  `PAPER_TORQUE_LIMIT=4.0`), arm length, body mass, derived properties
  (`rotor_mass`, `mass`, `hover_thrust_total`, `rotor_thrust_max`, `rotor_arm =
  L/√2`), drag/friction/motor-lag coefficients, and `verify_inertia_match` (83).
- `GainParams` (98): paper `Pq=20`, `Pw=4`, `torque_limit=4`, plus new altitude PID
  gains (kp=6, kd=4, ki=1.5 with anti-windup clamp).
- `SimParams` (112): scheduler rates — base 16 kHz (clears the 12.3 kHz stability
  floor), sensors 1 kHz, altitude 250 Hz, commander 50 Hz, logger 200 Hz; noise 0.1;
  seed 0.

### `pysitl/plant.py` — 13-state 6-DOF rigid body (scalar hot path)
State = (x, y, z, q0..q3, vx..vz, wx..wz), NED position, scalar-first quaternion.
- `_qmul`/`_qconj` (34–46): tuple-native Eq. 2 / Eq. 4 for speed.
- `rotate_body_to_world(q, v)` (49): `rotate(conj(q), v)` — the body→world direction
  dictated by eq. 18's sign convention (equivalently `to_dcm(q).T @ v`).
- `state_derivative` (57): full 6-DOF derivative — **eq. 18 verbatim in scalar
  arithmetic including the leading minus** (lines 69–71), thrust rotated body→world,
  linear drag, gravity, and the gyroscopic ω×(Iω) coupling (Eq. 17's rotational half).
- `rk4_step` (91): scalar RK4 (5.6 µs/step vs 65.5 µs numpy — needed for 14.5×
  real-time at 16 kHz).
- `normalize_quat` (109), `apply_ground_contact` (119, non-bouncy floor + friction),
  `IDENTITY_STATE` (135), and the `Plant` wrapper (138) with `.step/.reset` and
  position/altitude/quaternion/velocity/omega properties.

### `pysitl/airframe.py` — rotors, mixer, desaturation (Eqs. 29–30)
X-configuration, rotor order matching `quat_sitl`'s arm tips (front-right,
rear-left, front-left, rear-right; alternating spin directions).
- `rotor_positions` (41): (±d, ±d, 0) tips, d = L/√2.
- `thrusts_to_wrench` (46): **forward mix Eq. 29** — Fz = −ΣTᵢ, τx/τy from arm
  moments, τz from the yaw reaction coefficient c.
- `mix` (59): **inverse mix with thrust-preserving desaturation Eq. 30** — holds
  common-mode thrust `Tc/4` fixed, scales only the zero-sum torque deltas by the
  largest feasible s ∈ [0,1] (naive per-rotor clipping measurably doubled thrust and
  blew altitude to 154 m). Returns per-rotor thrusts + desat scale for telemetry.
- `Airframe` (99): per-rotor first-order actuator lag (τ = 30 ms) — PX4-like
  spin-up/down dynamics.

### `pysitl/sensors.py`
Scalar, stdlib-seeded twin of `quat_sitl/sensors.py` — same uniform ±amplitude noise
on quaternion (renormalized) and rates (Eq. 22), off the numpy path so the 1 kHz
sensor task stays fast.

### `pysitl/bus.py` — uORB-style message bus
`Sample` (25, timestamp + value + sequence number), `MessageBus` (31, thread-safe
latest-value pub/sub with `publish/get/get_value/snapshot`), `Subscriber` (57, PX4's
`updated()` idiom). All inter-module communication goes through this bus, never by
direct call — the defining PX4 architectural trait reproduced here.

### `pysitl/topics.py` — typed messages + topic names
Dataclasses shaped after PX4 uORB messages: `VehicleAttitude` (truth), 
`VehicleLocalPosition` (NED truth), `SensorCombined` (the paper's noisy q + ω — what
the controller actually reads; there is no estimator, matching the paper),
`ManualControlSetpoint`, `AttitudeSetpoint` (q_ref + shortest_path),
`VehicleTorqueSetpoint`, `ThrustSetpoint`, `ActuatorControls` (rotor cmds +
desat), `ActuatorOutputs` (post-lag), `VehicleStatus` (armed/mode/failsafe); plus the
`TOPIC_*` name constants.

### `pysitl/scheduler.py` — multi-rate lockstep scheduler
`Scheduler` (34): refuses construction below `stable_control_rate_hz` for the given
Pω/inertia (raises `ValueError`); tasks register at rates that must evenly divide the
base rate; `tick()` (60) decimates one base tick into each task's cadence;
`run_for(seconds, realtime)` (67) optionally paces to wall-clock for interactive
flying. Fully deterministic headless.

### `pysitl/modes.py` — flight modes + commander
`FlightMode` enum (12): STABILIZED, ALTITUDE, AUTO_STEP, AUTO_SINE, AUTO_FLIP.
`Commander` (31): preflight check (must be on ground), `try_arm`/`disarm`,
`set_mode`, and the altitude-envelope `check_failsafe` (auto-disarm outside
−1…50 m).

### `pysitl/sim.py` — the orchestrator
`Simulation` (38) wires plant + airframe + controllers + bus + scheduler + commander.
Scheduler tasks: `_task_sensors` (1 kHz, ZOH-held noise), `_task_altitude` (250 Hz
altitude hold / climb-rate), `_task_commander` (50 Hz failsafe + status publish), and
`_task_control_and_physics` (16 kHz: attitude setpoint → **the unmodified paper
controller** → mixer → actuator lag → wrench → plant step). Disarmed ⇒ zero
thrust/torque regardless of input. `set_flight_mode` (77) auto-wires AUTO_* modes to
the paper's reference functions (resetting to t = 0 on entry); `set_autopilot`,
`reset`, `euler()`, `run_for`.

### `pysitl/recorder.py` — ULog-style flight logger
`Recorder` (24): registers a 200 Hz scheduler task that snapshots every bus topic into
one wide row (position, q, ω, q_meas, q_ref, Euler via Eq. 16, τ, per-rotor thrusts,
armed/mode). `to_dataframe()` / `save_csv()` write `results/pysitl_<mode>_<ts>.csv`
(+ `_latest.csv`), column-compatible with `quat_sitl`'s CSVs.

### `pysitl/analysis.py` — post-flight plots
`plot_flight` (18): attitude-vs-reference (dashed/solid, same convention as
`quat_sitl.plotting`), altitude + torque (with saturation limit), and a ground-track
(East vs North) plot — Flight-Review style.

### `pysitl/run.py` — CLI
`main` (31): `--gcs` starts the web dashboard + real-time physics loop
(`_physics_loop`, 18) with the vehicle armed in ALTITUDE mode; `--mode auto_step|
auto_sine|auto_flip|... --duration --seed --log` runs headless, prints a summary
line, and (with `--log`) attaches the Recorder and writes CSV + analysis plots.

### `pysitl/control/` — control-layer modules
- `__init__.py`: re-exports.
- `attitude.py` (23): `compute_torque_scalar` — the **only** bridge to the paper's
  controller: converts tuples ↔ numpy, calls the unmodified
  `NonlinearP2Controller.compute_torque` (Eqs. 19–21), clips to ±4 N·m. Documented
  tradeoff: ~1.85× real-time with the real numpy controller vs a faster but
  "hopefully-equivalent" copy.
- `altitude.py` (19): `AltitudeController` — PD+I on altitude error → acceleration
  command → tilt-compensated collective thrust with anti-windup. **New, not from the
  paper** (the paper has no translation).
- `manual.py` (23–55): `ManualInput` (normalized sticks) and `StabilizedMapper` —
  roll/pitch → bounded tilt (max 0.6 rad) via `quat.from_euler` (Eq. 15), yaw stick
  integrates a heading target, throttle → direct thrust (STABILIZED) or climb rate
  (ALTITUDE).
- `autopilot.py` (17–35): maps AUTO_STEP/AUTO_SINE/AUTO_FLIP to the paper's own
  `quat_sitl.references` functions; flip disables shortest-path, exactly like
  `scenarios/flip_360.py`.

### `pysitl/gcs/` — the browser ground station
- `server.py` (171 lines): stdlib-only `ThreadingHTTPServer`. `GET /` serves the
  dashboard; `GET /events` streams ~50 Hz **SSE** JSON snapshots (`_snapshot`, 34:
  q, ω, position, altitude, armed/mode/failsafe, q_ref, τ, rotor thrusts, gains,
  control rate); `POST /stick` sets manual input; `POST /command` handles
  arm/disarm/set_mode/set_gains/reset. HTTP/1.1 keep-alive so clicks never queue
  behind stick posts.
- `static/index.html` (466 lines): the single-file dashboard — see §7 (UI).

---

## 6. `tests/`

### `tests/test_quaternion.py` — 7 tests (a fixed contract for the paper reproduction)
1. `test_mul_noncommutative_and_matrix_forms` — Eq. 2 non-commutativity; `Q(p)@q ==
   Q̄(q)@p == mul(p,q)`.
2. `test_q_times_conj_q_is_identity` — Eq. 4: q⊗q* = identity both ways.
3. `test_to_dcm_orthonormal_and_round_trips_euler` — Eqs. 9–16: RᵀR = I, det = +1,
   Euler round-trip over a 5³ angle grid.
4. `test_rotate_matches_dcm` — Eq. 8 vs Eq. 12: `rotate(q,v) == to_dcm(q) @ v` (pins
   the DCM convention).
5. `test_quaternion_norm_drift_under_10s_free_integration` — 10 s of unrenormalized
   RK4 on Eq. 7 stays on the unit sphere within 1e-6.
6. `test_zero_torque_zero_rate_state_unchanged` — Eq. 18 fixed point.
7. `test_sign_consistency_negative_feedback_and_closed_loop_convergence` — the
   eq. 18/eq. 21 **sign guardrail**: one closed-loop step must reduce attitude error
   (fails loudly if either sign is wrong), plus a 2 s convergence smoke test at
   `stable_control_rate_hz`.

### `tests/test_sitl.py` — 20 tests (pysitl; independent of the 7-test contract)
Vehicle inertia match; body→world frame convention (spins at a known rate, checks
where body-x lands in world); `rotate_body_to_world == to_dcm(q).T @ v`; **eq. 18
term-for-term parity** between the scalar pysitl plant and the numpy quat_sitl plant;
hover equilibrium; ground contact; zero-input fixed point; mixer round-trip;
thrust-preserving desaturation regression (the 154 m runaway bug) + randomized rotor
bounds; sensor noise bounded/renormalized; **closed-loop 0.3 rad roll tracking under
the paper's full 0.1 noise** through the real controller + mixer + 6-DOF plant;
scheduler rejection of unstable base rates, decimation counts, and seed determinism;
commander arming-on-ground, altitude-breach failsafe, disarmed ⇒ zero thrust; bus
latest-value/sequence semantics; AUTO_FLIP completing the full 2π (lands on
q ≈ [−1,0,0,0], identity's double cover).

**Core total: 27/27 tests pass** — plus 14 sim-deployment tests (`test_sim_bridge.py`: 8, `test_sim_adapters.py`: 5, `test_bridge_mavlink.py`: 1; adapter tests stack-gated) = **41 total**.

---

## 7. UI — three user interfaces

### 7.1 Browser ground station (`pysitl/gcs/`) — the interactive flyable UI
Launch: `python -m pysitl.run --gcs` → `http://127.0.0.1:8765`. Boots armed, hovering,
in ALTITUDE mode. Layout (documented in `pysitl/UI_GUIDE.md`):

```
┌─────────────┬───────────────────────────────┬─────────────┐
│  LEFT RAIL  │        3D VIEWPORT             │ RIGHT RAIL  │
│  Arm/Disarm │  (drag orbit, scroll zoom)     │ Attitude    │
│  Flight mode│   solid vehicle + dashed       │ strip chart │
│  Pq/Pω      │   reference ghost, spinning    │ Torque      │
│   sliders   │   props, HUD, stick pads,      │ strip chart │
│  Throttle + │   saturation state             │ Rotor thrust│
│  WASD/QE/↑↓ │                                │   meters    │
└─────────────┴───────────────────────────────┴─────────────┘
```

- **Left rail**: Arm/Disarm (preflight-checked), mode buttons (Stabilized, Altitude,
  Auto Step / Sine / Flip, Reset), live **Pq / Pω sliders wired directly into the real
  controller**, throttle slider, keyboard help.
- **Viewport** (`index.html` JS, hand-rolled canvas 3D projection with NED-aware
  camera): ground grid, solid drone with colored front/rear motors, legs, nose
  chevron, spinning props (rate ∝ actual rotor thrust), dashed **amber reference
  ghost** — the gap between ghost and vehicle *is* the live tracking error — plus a
  body-axis triad and an altitude leg to the ground. HUD shows mode, control-tick
  rate, altitude/target, sim time; on-screen dual stick pads mirror input.
- **Right rail**: scrolling 12 s attitude (φ/θ/ψ) and torque strip charts (torque axis
  auto-scaled to ±τ_max) and four per-rotor thrust meters.
- **Controls**: buttons/gains POST to `/command`; keyboard W/S, A/D, Q/E, ↑/↓
  sampled at 25 Hz and POSTed to `/stick` only on change; telemetry arrives as 50 Hz
  SSE on `/events`.
- **Safety**: arming only on the ground; altitude-envelope failsafe (auto-disarm +
  red banner); Disarm instantly zeros thrust/torque.

### 7.2 3D replay viewer (`quat_sitl/visualize3d.py`) — non-interactive replay UI
Matplotlib window replaying any logged CSV as an animated quadrotor with arms and RGB
body axes (attitude from `to_dcm`), next to attitude/torque plots matching paper
Figs. 3–8; options for fps, playback speed, and GIF/MP4 export. Produced
`docs/figures/flip_animation.gif`.

### 7.3 Standalone browser simulator (`fresk_..._(1).html`) — zero-install UI
A single self-contained HTML file (see §2) implementing the entire paper in
JavaScript: 3D view with labeled X/Y/Z axes, live attitude/torque charts, Step/Sine/
Flip/Manual benchmark buttons, manual φ/θ/ψ sliders, Pq/Pω/τ_max sliders, noise
amplitude + seed, 0.5×/1×/2× speed, pause/reset/CSV export, auto-loop, and an
"equation audit" panel. Useful for demonstrating the controller with no Python
installed.

---

## 8. Results produced by the code

### 8.1 Paper-parameter fidelity (from `report1.md` §Results, verified in code)

| Quantity | Paper | Implemented where | Match |
|---|---|---|---|
| Ixx = Iyy | 6.5e-4 kg·m² | `quat_sitl/dynamics.py:31-32`, `pysitl/params.py:36` | exact |
| Izz | 1.2e-3 kg·m² | `dynamics.py:33`, `params.py:38` | exact |
| Pq / Pω | 20 / 4 | `controller.py:20-21`, `params.py:102-103` | exact |
| Torque saturation | ±4 N·m | `dynamics.py:74`, `control/attitude.py:32` | exact |
| Sine | 0.5 rad, 1 rad/s | `references.py:34-35` | exact |
| Flip | 0→2π rad | `references.py:55` | exact |
| Noise | 0.1 | `sensors.py` (both) | exact |

### 8.2 Scenario results (`scenarios/*.py` → `results/` + `docs/figures/`)

- **Step (Figs. 3–4)**: staggered 1 rad steps converge with overshoot ≤ 0.009 rad,
  5 %-band settling ≈ 1.2 s/axis; torque saturates briefly at each onset then returns
  to the linear region. Matches the paper.
- **Sine (Figs. 5–6)**: 0.5 rad / 1 rad/s tracked at 0.46–0.47 rad amplitude with
  phase lag 0.37–0.39 s (paper says "about 0.5 s"; linear theory predicts 0.3805 s);
  torque stays linear except one initialization sample. Matches.
- **360° flip (Figs. 7–8)**: full 2π completes; final attitude 0.1° from identity
  (q0 ≈ −1, the double cover); max per-ms quaternion jump ≈ 0.0016 — smooth
  throughout; the plotted Euler φ wraps at ±π purely from `atan2` (Eq. 16), not from
  any controller singularity. Matches the paper's central claim (no gimbal lock).

### 8.3 Original findings produced by this codebase

- **Discrete-time control-rate stability** (`dynamics.py:103`, `REPORT.md` §2.5):
  the paper's own gains/inertia give a fast real pole ≈ −Pω/Ixx ≈ −6150 rad/s; exact
  ZOH discretization gives closed-loop spectral radius ≈ 29.9 at 200 Hz and ≈ 5.16 at
  1 kHz (both unstable); the implemented control rate is Eq. 26's ≈ 12.3 kHz (16 kHz in
  pysitl), with 1 kHz state logging preserved.
- **6-DOF validation** (`test_sitl.py`, `REPORT.md` §3.5): the *unmodified* P²
  controller flies the derived 200 g vehicle; a 0.3 rad roll command converges to
  0.309 rad under full paper noise while holding altitude to centimeters.
- **Mixer regression**: thrust-preserving desaturation (`airframe.py:59`) fixes a
  measured altitude runaway to 154 m that naive per-rotor clipping caused.
- **Test suite**: 7 (`test_quaternion.py`) + 20 (`test_sitl.py`) = 27/27 core; + 14 sim (`test_sim_bridge.py`, `test_sim_adapters.py`, `test_bridge_mavlink.py`) = 41 total.

---

## 9. All equations used in `report1.md`, cited across the files

`report1.md` uses the paper's equations **(1)–(21)**. The table below lists each
equation, what it is, and every file that implements/uses it (line numbers point at
the defining function; JS line numbers are within the named HTML file).

| Eq. | Content (as in `report1.md`) | Implemented / used in |
|---|---|---|
| **(1)** | Quaternion hyper-complex / vector representation, scalar-first `[q0,q1,q2,q3]ᵀ` | `quat_sitl/quaternion.py:1` (module convention); `pysitl/plant.py:6` (13-state layout); `fresk…html` (array convention throughout) |
| **(2)** | Multiplication (Kronecker product) `p⊗q` + left/right matrices `Q(p)`, `Q̄(q)` | `quat_sitl/quaternion.py:29` (`mul`), `:47` (`Q`), `:60` (`Q_bar`); scalar twin `pysitl/plant.py:34` (`_qmul`); JS `pysitl/gcs/static/index.html:153` and `fresk…html:105` (`qmul`); tested in `tests/test_quaternion.py:27` |
| **(3)** | Norm `‖q‖ = √(q0²+q1²+q2²+q3²)` | `quaternion.py:72` (`norm`); `fresk…html:112` (`qnorm`); used by `normalize` (`quaternion.py:96`), `simulator.py:98` (drift assert), `pysitl/plant.py:111` |
| **(4)** | Conjugate `q* = [q0,−q1,−q2,−q3]ᵀ` | `quaternion.py:79` (`conj`); `pysitl/plant.py:45` (`_qconj`); `index.html:156`, `fresk…html:111` (`qconj`); tested in `test_quaternion.py:39` |
| **(5)** | Inverse `q⁻¹ = q*/‖q‖²` | `quaternion.py:86` (`inv`); `fresk…html:114` (`qinv`) |
| **(6)** | q̇ with fixed-frame ω: `½ q⊗[0,ω]` | `quaternion.py:106` (`qdot_fixed`) |
| **(7)** | q̇ with body-frame ω′: `½ [0,ω′]⊗q` (positive sign) | `quaternion.py:116` (`qdot_body`); used in the norm-drift test `test_quaternion.py:67`; contrasted against Eq. 18 in `dynamics.py:124` |
| **(8)** | Vector rotation `w = q⊗[0,v]⊗q*` | `quaternion.py:131` (`rotate`); consistency with DCM tested in `test_quaternion.py:59`; body→world counterpart `rotate(conj(q),v)` in `pysitl/plant.py:49` |
| **(9)–(11)** | DCM columns `Rx(q), Ry(q), Rz(q)` | `quaternion.py:146-157` (inside `to_dcm`) |
| **(12)** | DCM `R(q) = [Rx Ry Rz]` (column form) | `quaternion.py:141` (`to_dcm`); used for rendering in `quat_sitl/visualize3d.py:77`; JS `fresk…html:117` (`q2R`); orthonormality tested in `test_quaternion.py:46` |
| **(13)** | DCM transpose (frame rotation) | `to_dcm(q).T` — used as `pysitl/plant.py:49` (`rotate_body_to_world`); pinned by `tests/test_sitl.py:68` |
| **(14)** | Axis-angle → quaternion `cos(α/2)+u sin(α/2)` | `quaternion.py:162` (`from_axis_angle`); `fresk…html:129` (`axang2q`); used by `references.py:56` (flip reference) and tests |
| **(15)** | Euler (ZYX) → quaternion | `quaternion.py:177` (`from_euler`); `fresk…html:132` (`euler2q`); used by `references.py:30,46`, `pysitl/control/manual.py:49`, `tests/test_sitl.py:210` |
| **(16)** | Quaternion → Euler (**plotting/logging only**) | `quaternion.py:194` (`to_euler`); `fresk…html:138` and `index.html:205` (`q2euler`/`eulerOf`); used only in `quat_sitl/simulator.py:102-103`, `plotting.py`, `pysitl/sim.py:174`, `pysitl/recorder.py:49`, and the dashboards — never in the control loop |
| **(17)** | Newton–Euler rigid-body equations (rotational half feeds Eq. 18) | `quat_sitl/dynamics.py:146` (`ω̇` incl. gyroscopic `ω×(Iω)`); full 6-DOF translational+rotational use in `pysitl/plant.py:57-88`; `fresk…html:158-165` |
| **(18)** | **The plant**: `q̇ = −½[0,ω]⊗q` (paper's literal leading minus), `ω̇ = I⁻¹τ − I⁻¹[ω×(Iω)]` | `quat_sitl/dynamics.py:124` (`state_derivative`, sign discussed at 129-137); scalar twin `pysitl/plant.py:57` (parity test `tests/test_sitl.py:85`); JS `fresk…html:158` (`deriv`); guarded by `test_quaternion.py:94` |
| **(19)** | Error quaternion `q_err = q_ref ⊗ q_m*` | `quat_sitl/controller.py:36`; `fresk…html:173`; both dashboards display q_ref |
| **(20)** | `Axis_err = [q_err,1..3]ᵀ` (+ shortest-path flip when q_err,0 < 0, disabled for the flip) | `quat_sitl/controller.py:37-40` (`shortest_path` flag); `fresk…html:174`; flag routed via `references`/`autopilot.py:20`, `sim.py:140-141` |
| **(21)** | **Control law** `τ = −Pq·Axis_err − Pω·ω_m`, gains 20/4, ±4 N·m saturation | `quat_sitl/controller.py:43` (unsaturated) + `quat_sitl/dynamics.py:74` (`torque_saturate`); pysitl path `pysitl/control/attitude.py:23` (`compute_torque_scalar`, clips at :32); JS `fresk…html:172-176`; gains from `controller.py:20-21` / `pysitl/params.py:102-104`; closed-loop tested in `test_quaternion.py:94` and `test_sitl.py:199` |

### Extension equations (appear in `REPORT.md` only, not in `report1.md`)

For completeness — these are this project's own contributions, also citable in code:

| Eq. | Content | Implemented / used in |
|---|---|---|
| **(22)** | Sensor noise model: `q_m = (q+n_q)/‖·‖`, `ω_m = ω+n_ω`, `U(−0.1,0.1)` | `quat_sitl/sensors.py:24`; `pysitl/sensors.py:22`; JS `fresk…html:178-182`; tested in `test_sitl.py:183` |
| **(23)** | Linearized closed loop `I ë + Pω ė + (Pq/2) e = 0` | derivation in `README.md` ("Results vs Paper"); motivates `dynamics.py:103` |
| **(24)** | Characteristic roots λ₁ ≈ −2.5, λ₂ ≈ −6150 rad/s | `dynamics.py:88-99` docstrings; `simulator.py:4-12` module docstring |
| **(25)** | Exact ZOH discretization Φ, Γ and spectral radius (29.9 @200 Hz, 5.16 @1 kHz) | `dynamics.py:103-121` docstring; `simulator.py:4-22`; `test_sitl.py:232` (scheduler rejects unstable rates) |
| **(26)** | Minimum stable control rate `f ≥ Pω/(m·I_min)` ≈ 12.3 kHz (m = 0.5) | `quat_sitl/dynamics.py:103` (`stable_control_rate_hz`); consumed by `simulator.py:64`, `pysitl/scheduler.py:36`, `SimParams.base_hz = 16 kHz` (`params.py:117`); JS analogue `fresk…html:153` (`stableDt`) |
| **(27)** | Rotor inertia contributions `Ixx=Iyy=4m_r d²`, `Izz=8m_r d²`, `d = L/√2` | `pysitl/params.py:83-94` (`verify_inertia_match`); tested in `test_sitl.py:40` |
| **(28)** | Derived vehicle: `m_r = (Izz−Ixx)/(2L²)`, `I_b = 2Ixx−Izz` | `pysitl/params.py:56-59` (`rotor_mass`), docstring derivation at 9-25 |
| **(29)** | Forward mixer (rotor thrusts → wrench) | `pysitl/airframe.py:46` (`thrusts_to_wrench`); round-trip tested in `test_sitl.py:142` |
| **(30)** | Thrust-preserving desaturation `Tᵢ = Tc/4 + s·Dᵢ`, `ΣDᵢ = 0` | `pysitl/airframe.py:59-96` (`mix`); regression-tested in `test_sitl.py:153,167` |
| **(31)** | Simulator convention `q_paper = conj(q_sim)` (eq. 18's literal minus sign integrates the conjugate of a standard body→world quaternion) | derived + proven in `sim/common/bridge.py` (module docstring); guarded by `tests/test_sim_bridge.py::test_conjugate_convention_drives_standard_plant` |
| **(32)** | Priority desaturation: fit roll/pitch deltas with max `s₁∈[0,1]`, then yaw into remaining headroom (PX4-style) — yaw authority `c·T ≈ 0.03 N·m` vs ±4 N·m commands | `sim/common/bridge.py:mix_zup`; regression-tested in `test_sim_bridge.py::test_mixer_priority_protects_roll_from_yaw_noise` |
| **(33)** | P² pursuit rate equilibrium `ω = (Pq/Pω)·sin(e/2)` — a 2 s flip ramp (π rad/s) is trackable only at lag e ≈ 1.36 rad; flip completion is engine-dependent at that knife-edge | derivation + measurements in `sim/README.md` (finding 4); scenario knob `sim/common/scenarios.py:FLIP_RAMP_6DOF` |

---

*Generated from a full read of all 51 repository files. Cross-reference the canonical
equation→function table in `README.md` and the extended derivations in `REPORT.md`.*
