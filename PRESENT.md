# Presenting the project — the full explanation

A speaking guide, layered: 30 seconds → 3 minutes → deep dive → likely questions.
Everything here is true and traceable to a committed CSV, figure, or test.

---

## 1. The 30-second version

> "Our base paper replaces Euler angles with quaternions for quadrotor attitude
> control — no gimbal lock, no trigonometry in the loop. We reproduced the
> paper exactly, derived things the paper left out, and then flew the SAME
> controller — gains untouched — on four simulators: MuJoCo,
> gym-pybullet-drones, Gazebo, and real ArduPilot firmware we compiled
> ourselves. Where physics allows it, all four agree to within 3 degrees. Where
> they don't, we can prove why — with derivations and experiments."

## 2. The 3-minute version (the five stages)

**Stage 1 — Reproduce the paper exactly** (`quat_sitl/`).
Every equation (1)–(21), every parameter — gains Pq=20, Pω=4, ±4 N·m torque
limit, the paper's inertia, its ±0.1 sensor noise. All three benchmark
scenarios (step / sine / 360° flip) match the paper's figures. 27 tests guard
the fidelity.

**Stage 2 — Derive what the paper didn't** (Eqs. 23–26).
The paper never says how fast the digital loop must run. We linearized the
closed loop, found the fast pole (~6150 rad/s), and proved the paper's own
gains are UNSTABLE at 200 Hz–1 kHz and need ≥ ~12.3 kHz. That number is
derived, not tuned.

**Stage 3 — Make it fly like a real vehicle** (`pysitl/`).
The paper's plant is an ideal torque-input double integrator. We derived a
real 200 g vehicle from the paper's inertia and flew the UNMODIFIED controller
through rotors, a mixer, gravity, ground contact, a PX4-style scheduler.

**Stage 4 — The four simulators** (`sim/`, the FRP deliverable).
One shared bridge: the same frozen controller, the same scenarios, the same
noise, the same logging — on MuJoCo, gym-pybullet-drones, Gazebo (real gz
physics + the ardupilot_gazebo plugin), and ArduPilot SITL (firmware compiled
from source, flown over MAVLink).

**Stage 5 — Results + findings.** Sine agrees within 3° across all four
stacks (the firmware stack is the BEST: 14.7°). Flip completes on firmware
(2.89 s) and MuJoCo (2.93 s). Seven documented findings — including the
biggest: ArduPilot silently IGNORES attitude targets unless GUID_OPTIONS is
set; we caught it, fixed it with one bit, and sine went 37.7° → 14.7°.

## 3. The deep dive — each piece explained simply

### 3.1 Why quaternions at all?
Euler angles (roll/pitch/yaw) have gimbal lock — at 90° pitch, two axes align
and a degree of freedom vanishes; flips pass exactly through it. Rotation
matrices (DCM) carry 9 numbers with 6 constraints. A quaternion is 4 numbers,
no singularities, one clean multiply to compose rotations. The paper's point:
keep the ENTIRE control loop in quaternion space — Euler appears only for
plotting.

### 3.2 The controller (eqs. 19–21) — it is just two terms
```
q_err  = q_ref ⊗ q_measured*        (eq. 19 — quaternion error)
axis   = vector part of q_err       (eq. 20 — which way + how far to rotate)
τ      = −Pq·axis − Pω·ω            (eq. 21 — torque = stiffness + damping)
```
Pq=20 is the stiffness (pulls attitude toward the reference), Pω=4 is the
damping (fights rotation rate). No integral term, no derivative computation —
because attitude→torque is a double integrator, the error decays on its own.

### 3.3 The bridge — the one clever bit (Eq. 31)
Every simulator stores orientation as a body→world quaternion. The paper's eq.
(18) has a literal minus sign, which makes the paper's quaternion the
CONJUGATE of the simulator's:
```
q_paper = conj(q_sim)
```
We PROVED this (a test drives a standard plant through this exact mapping and
converges). One bridge therefore works on every engine: read q_sim, conjugate,
hand to the unmodified paper controller, apply the returned body torque.

### 3.4 The mixer — priority desaturation (Eq. 32)
The paper assumes control-signal == torque. Real rotors can produce only
~0.4 N·m (not ±4), and the yaw channel is ~50× weaker than roll/pitch. The
paper's own sensor noise commands ±2–3 N·m of yaw chatter, which under naive
mixing collapses ALL axes. Our mixer desaturates roll/pitch first, yaw gets
the leftovers — the same trick PX4's mixer uses. Measured necessity.

### 3.5 The four stacks — what each one adds
- **MuJoCo** — high-accuracy contact physics, 20 kHz, our paper-vehicle MJCF.
- **gym-pybullet-drones** — the RL community's standard env, 24 kHz, custom
  paper-vehicle URDF; exercises a different integrator and damping model.
- **Gazebo** — the robotics standard: gz-harmonic physics + ardupilot_gazebo
  plugin + the firmware. The full chain a real lab would run.
- **ArduPilot SITL** — production flight-stack firmware, built from source
  with waf, flown over MAVLink (GUIDED, arm, takeoff, 50 Hz setpoints) — the
  closest thing to hardware without hardware.

### 3.6 The results (know these numbers)
- **Sine RMS:** ArduPilot **14.7°±0.2** · Gazebo **17.3°±0.0** · PyBullet
  17.52° · MuJoCo 17.61° — all four within 3°, firmware best. The shared ±0.1
  quaternion noise sets an ~11.5° floor — so tracking is only ~3–6° above the
  sensor noise itself.
- **Step (1 rad):** natives 17.68/17.63 (0.3% apart); firmware tracks with
  liveliness (47.4°); Gazebo tracks the onset (φ→1.03) then the paper's gains
  excite sustained rolls through full rigid-body physics — clamp RULED OUT by
  experiment (ANGLE_MAX 80° changed nothing).
- **Flip:** settles in 2.89 s on firmware, 2.93 s on MuJoCo (both complete,
  near-identical); PyBullet/Gazebo sit in the knife-edge regime — derived as
  the P² pursuit equilibrium ω = 5·sin(e/2) (Eq. 33): a 2 s ramp is trackable
  only at lag ≈1.36 rad, right at the edge.

### 3.7 The seven findings (one line each)
1. The controller transfers across engines (0.3% agreement natives; 3° all four).
2. ±4 N·m is not rotor-realizable (~0.4 physical) → priority mixer.
3. Flips need acro-style thrust handling (idle when inverted) or they deadlock at 180°.
4. The flip is knife-edge on rotor plants — Eq. 33.
5. The 12.3 kHz control-rate bound carries over (adapters assert it at startup).
6. Integration gotchas: GRAVITY-is-weight, PyBullet damping, MJCF massless sites, spawn altitude.
7. **GUID_OPTIONS**: ArduPilot ACKs-and-ignores SET_ATTITUDE_TARGET by default — caught by trajectory inspection (φ pinned at 0), fixed with one bit, sine 37.7°→14.7°.

## 4. Likely questions — and the answers

**Q: Why is your firmware sine BETTER than your ideal sims?**
The firmware's inner rate/attitude loops act as extra filtering of the ±0.1
sensor noise our law consumes — the noise floor effectively drops. The
controller is identical; the stack conditions the signal.

**Q: Why does the flip fail on two stacks? Is that a controller flaw?**
It's the paper's own dynamics: the rate-equilibrium analysis (Eq. 33) shows a
2 s ramp is trackable only with lag ≈1.36 rad — marginally. MuJoCo and the
firmware close it; PyBullet's integrator details land on the other side. On
the ideal plant (the paper's world) there is no such edge — that's exactly the
kind of thing porting to real physics reveals.

**Q: Why is Gazebo's step row large?**
Measured boundary: it tracks the onset, then the paper's Pq=20 through full
rigid-body physics excites sustained rolls. We ruled out the tilt clamp by
raising ANGLE_MAX to 80° — unchanged (121° vs 123°). Physics, not a bug.

**Q: Did you change the controller per simulator?**
Never. One import, frozen gains, everywhere — that's the experiment's premise.
The adapters only translate conventions (the conjugate bridge).

**Q: What would you do differently?**
Unify the vehicle across all four stacks (paper vehicle on the two natives,
reference iris on the firmware pair — documented), and multi-seed the
diagnostic runs. Both are noted as future work in the README.

**Q: Show me proof of a number.**
`results/<stack>/<scenario>_seed<k>.csv` — every row of the table regenerates
from these via `python -m sim.compare`. Nothing hand-typed.

## 5. The demo flow (if asked to run)
```bash
git checkout sainath/sim-deployment
python -m sim.mujoco.run --scenario flip          # airborne 360°, ~30 s
python -m sim.gym_pybullet.run --scenario step
pytest tests/ -q                                  # 39 pass
```
Gazebo/ArduPilot: show the committed CSVs/figures — the WSL2 environment is
reproducible from `sim/wsl/` + the runbook in one script.
