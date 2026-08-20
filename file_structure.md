# File Structure — Full Quaternion-Based Attitude Control for a Quadrotor

Complete map of this repository: every coding file from root to leaves, what each
contains, the results the project produces, the UIs, and — at the end — every
equation used in `report1.md` (Eqs. 1–21) cited to the exact file (and function) that
implements it, plus the extension equations (22–26) that appear only in `REPORT.md`.

The repository is a Python software-in-the-loop (SITL) reproduction of
Fresk & Nikolakopoulos, *"Full Quaternion Based Attitude Control for a Quadrotor"*
(ECC 2013). It contains **one Python package** plus runnable benchmark scripts, the
test suite, committed result figures, and one standalone browser simulator:

| Component | Role |
|---|---|
| `quat_sitl/` | The paper reproduction: quaternion algebra (Eqs. 1–16), attitude plant (Eqs. 17–18), nonlinear P² controller (Eqs. 19–21), sensor noise, the three benchmark scenarios, plotting, and a 3D replay viewer. Attitude-only, exactly like the paper — no mass, no rotors, no translation. |
| `fresk…(1).html` | A zero-dependency browser simulator of the same paper model (Eqs. 1–21) in one self-contained HTML+JS file. |

Plus `scenarios/` (runnable benchmark scripts), `tests/` (7 tests), and
`docs/figures/` (generated result figures).

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
├── .gitignore                              # ignores caches + generated results/
├── README.md                               📄 docs: run instructions, eq→function map,
│                                           #   results-vs-paper discussion, stability analysis
├── REPORT.md                               📄 full technical report (extends report1.md with
│                                           #   Eqs. 22–26, test tables, discussion, references)
├── report1.md                              📄 project report #1 (team info, methodology,
│                                           #   Eqs. 1–21, expected/measured results)
├── file_structure.md                       📄 this file
├── fresk_nikolakopoulos_precise_attitude_simulator_xyz_labels(1).html
│                                           # standalone zero-dependency browser simulator of
│                                           #   Eqs. 1–21 (single self-contained HTML+JS file)
│
├── quat_sitl/                              # ── the paper reproduction package ─────────────
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
├── tests/
│   └── test_quaternion.py                  # 7 tests: algebra + closed-loop sign guardrail
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
                                            #   *_flip.png
```

---

## 2. Root-level files

### `pyproject.toml`
Setuptools build config for package `quat-sitl` v0.1.0, Python ≥ 3.10. Declares the
dependencies `numpy>=1.26`, `scipy>=1.11`, `matplotlib>=3.8`, `pandas>=2.1`, and
restricts `setuptools.packages.find` to `quat_sitl*`. Note: `scipy` is declared
but never actually imported by any module — a vestigial dependency.

### `requirements.txt`
Flat pip list mirroring `pyproject.toml` plus `pytest>=7.4` for the test suite.

### `.gitignore`
Ignores Python caches (`__pycache__/`, `*.pyc`, `*.egg-info/`, `.pytest_cache/`), build
dirs, and everything generated under `results/` (CSV/PNG/GIF/MP4), keeping the repo to
source + committed figures only.

### `README.md` 📄
The package documentation. Contains: install/run commands; the canonical
**equation → function map** (Eqs. 1–21 → `quaternion.py`/`dynamics.py`/`controller.py`
functions); results-vs-paper discussion for all three scenarios; the five documented
paper ambiguities and their chosen defaults (noise distribution, step stagger, sine
phase, flip ramp, DCM convention); the eq. (18) sign-convention discussion; and the full
discrete-time **control-rate stability analysis** (why 200 Hz / 1 kHz control ticks are
provably unstable for the paper's gains, and why `stable_control_rate_hz` ≈ 12.3 kHz is
used instead, with state still logged at 1 kHz).

### `REPORT.md` 📄
The full technical report: *"…A Python Software-In-The-Loop Reproduction and Stability
Analysis."* Restates Eqs. 1–21 with full derivations and **adds equations not in the
paper** — Eq. 22 (noise model), Eqs. 23–24 (linearized closed loop and its poles),
Eq. 25 (exact ZOH discretization + spectral-radius table), Eq. 26 (minimum stable
control rate) — plus results tables, the 7-test summary, discussion sections, and 6
references.

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

## 5. `tests/`

### `tests/test_quaternion.py` — 7 tests (the paper-reproduction contract)
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

**Total: 7/7 tests pass.**

---

## 6. UI — two user interfaces

### 6.1 Standalone browser simulator (`fresk_..._(1).html`) — zero-install UI
A single self-contained HTML file (see §2) implementing the entire paper in
JavaScript: 3D view with labeled X/Y/Z axes, live attitude/torque charts, Step/Sine/
Flip/Manual benchmark buttons, manual φ/θ/ψ sliders, Pq/Pω/τ_max sliders, noise
amplitude + seed, 0.5×/1×/2× speed, pause/reset/CSV export, auto-loop, and an
"equation audit" panel. Attitude-only — exactly the paper's model, nothing more.

### 6.2 3D replay viewer (`quat_sitl/visualize3d.py`) — replay UI
Matplotlib window replaying any logged CSV as an animated quadrotor with arms and RGB
body axes (attitude from `to_dcm`), next to attitude/torque plots matching paper
Figs. 3–8; options for fps, playback speed, and GIF/MP4 export. Produced
`docs/figures/flip_animation.gif`.

---

## 7. Results produced by the code

### 7.1 Paper-parameter fidelity (from `report1.md` §Results, verified in code)

| Quantity | Paper | Implemented where | Match |
|---|---|---|---|
| Ixx = Iyy | 6.5e-4 kg·m² | `quat_sitl/dynamics.py:31-32` | exact |
| Izz | 1.2e-3 kg·m² | `dynamics.py:33` | exact |
| Pq / Pω | 20 / 4 | `controller.py:20-21` | exact |
| Torque saturation | ±4 N·m | `dynamics.py:74` | exact |
| Sine | 0.5 rad, 1 rad/s | `references.py:34-35` | exact |
| Flip | 0→2π rad | `references.py:55` | exact |
| Noise | 0.1 | `sensors.py` | exact |

### 7.2 Scenario results (`scenarios/*.py` → `results/` + `docs/figures/`)

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

### 7.3 Original finding produced by this codebase

- **Discrete-time control-rate stability** (`dynamics.py:103`, `REPORT.md` §2.5):
  the paper's own gains/inertia give a fast real pole ≈ −Pω/Ixx ≈ −6150 rad/s; exact
  ZOH discretization gives closed-loop spectral radius ≈ 29.9 at 200 Hz and ≈ 5.16 at
  1 kHz (both unstable); the implemented control rate is Eq. 26's ≈ 12.3 kHz, with
  1 kHz state logging preserved.
- **Test suite**: 7/7 pass (`tests/test_quaternion.py`).

---

## 8. All equations used in `report1.md`, cited across the files

`report1.md` uses the paper's equations **(1)–(21)**. The table below lists each
equation, what it is, and every file that implements/uses it (line numbers point at
the defining function; JS line numbers are within the named HTML file).

| Eq. | Content (as in `report1.md`) | Implemented / used in |
|---|---|---|
| **(1)** | Quaternion hyper-complex / vector representation, scalar-first `[q0,q1,q2,q3]ᵀ` | `quat_sitl/quaternion.py:1` (module convention); `fresk…html` (array convention throughout) |
| **(2)** | Multiplication (Kronecker product) `p⊗q` + left/right matrices `Q(p)`, `Q̄(q)` | `quat_sitl/quaternion.py:29` (`mul`), `:47` (`Q`), `:60` (`Q_bar`); JS `fresk…html:105` (`qmul`); tested in `tests/test_quaternion.py:27` |
| **(3)** | Norm `‖q‖ = √(q0²+q1²+q2²+q3²)` | `quaternion.py:72` (`norm`); `fresk…html:112` (`qnorm`); used by `normalize` (`quaternion.py:96`), `simulator.py:98` (drift assert) |
| **(4)** | Conjugate `q* = [q0,−q1,−q2,−q3]ᵀ` | `quaternion.py:79` (`conj`); `fresk…html:111` (`qconj`); tested in `test_quaternion.py:39` |
| **(5)** | Inverse `q⁻¹ = q*/‖q‖²` | `quaternion.py:86` (`inv`); `fresk…html:114` (`qinv`) |
| **(6)** | q̇ with fixed-frame ω: `½ q⊗[0,ω]` | `quaternion.py:106` (`qdot_fixed`) |
| **(7)** | q̇ with body-frame ω′: `½ [0,ω′]⊗q` (positive sign) | `quaternion.py:116` (`qdot_body`); used in the norm-drift test `test_quaternion.py:67`; contrasted against Eq. 18 in `dynamics.py:124` |
| **(8)** | Vector rotation `w = q⊗[0,v]⊗q*` | `quaternion.py:131` (`rotate`); consistency with DCM tested in `test_quaternion.py:59` |
| **(9)–(11)** | DCM columns `Rx(q), Ry(q), Rz(q)` | `quaternion.py:146-157` (inside `to_dcm`) |
| **(12)** | DCM `R(q) = [Rx Ry Rz]` (column form) | `quaternion.py:141` (`to_dcm`); used for rendering in `quat_sitl/visualize3d.py:77`; JS `fresk…html:117` (`q2R`); orthonormality tested in `test_quaternion.py:46` |
| **(13)** | DCM transpose (frame rotation) | `to_dcm(q).T` — the eq.-13 sense of `to_dcm`; noted in `README.md` "Results vs Paper" (ambiguity 1) |
| **(14)** | Axis-angle → quaternion `cos(α/2)+u sin(α/2)` | `quaternion.py:162` (`from_axis_angle`); `fresk…html:129` (`axang2q`); used by `references.py:56` (flip reference) and tests |
| **(15)** | Euler (ZYX) → quaternion | `quaternion.py:177` (`from_euler`); `fresk…html:132` (`euler2q`); used by `references.py:30,46` |
| **(16)** | Quaternion → Euler (**plotting/logging only**) | `quaternion.py:194` (`to_euler`); `fresk…html:138` (`q2euler`); used only in `quat_sitl/simulator.py:102-103`, `plotting.py`, and the browser charts — never in the control loop |
| **(17)** | Newton–Euler rigid-body equations (rotational half feeds Eq. 18) | `quat_sitl/dynamics.py:146` (`ω̇` incl. gyroscopic `ω×(Iω)`); `fresk…html:158-165` |
| **(18)** | **The plant**: `q̇ = −½[0,ω]⊗q` (paper's literal leading minus), `ω̇ = I⁻¹τ − I⁻¹[ω×(Iω)]` | `quat_sitl/dynamics.py:124` (`state_derivative`, sign discussed at 129-137); JS `fresk…html:158` (`deriv`); guarded by `test_quaternion.py:94` |
| **(19)** | Error quaternion `q_err = q_ref ⊗ q_m*` | `quat_sitl/controller.py:36`; `fresk…html:173` |
| **(20)** | `Axis_err = [q_err,1..3]ᵀ` (+ shortest-path flip when q_err,0 < 0, disabled for the flip) | `quat_sitl/controller.py:37-40` (`shortest_path` flag); `fresk…html:174` |
| **(21)** | **Control law** `τ = −Pq·Axis_err − Pω·ω_m`, gains 20/4, ±4 N·m saturation | `quat_sitl/controller.py:43` (unsaturated) + `quat_sitl/dynamics.py:74` (`torque_saturate`); gains from `controller.py:20-21`; JS `fresk…html:172-176`; closed-loop tested in `test_quaternion.py:94` |

### Extension equations (appear in `REPORT.md` only, not in `report1.md`)

For completeness — these are this project's own contributions, also citable in code:

| Eq. | Content | Implemented / used in |
|---|---|---|
| **(22)** | Sensor noise model: `q_m = (q+n_q)/‖·‖`, `ω_m = ω+n_ω`, `U(−0.1,0.1)` | `quat_sitl/sensors.py:24`; JS `fresk…html:178-182` |
| **(23)** | Linearized closed loop `I ë + Pω ė + (Pq/2) e = 0` | derivation in `README.md` ("Results vs Paper"); motivates `dynamics.py:103` |
| **(24)** | Characteristic roots λ₁ ≈ −2.5, λ₂ ≈ −6150 rad/s | `dynamics.py:88-99` docstrings; `simulator.py:4-12` module docstring |
| **(25)** | Exact ZOH discretization Φ, Γ and spectral radius (29.9 @200 Hz, 5.16 @1 kHz) | `dynamics.py:103-121` docstring; `simulator.py:4-22` |
| **(26)** | Minimum stable control rate `f ≥ Pω/(m·I_min)` ≈ 12.3 kHz (m = 0.5) | `quat_sitl/dynamics.py:103` (`stable_control_rate_hz`); consumed by `simulator.py:64`; JS analogue `fresk…html:153` (`stableDt`) |

---

*Generated from a full read of the repository. Cross-reference the canonical
equation→function table in `README.md` and the extended derivations in `REPORT.md`.*
