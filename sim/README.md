# Cross-simulator deployment (`sim/`)

The paper's quaternion attitude law (Fresk & Nikolakopoulos, ECC 2013), with
its published gains **untouched** (Pq = 20, Pω = 4, ±4 N·m saturation), flying
on four independent simulation stacks — the course's final-evaluation
requirement. Proposal and rationale: [`FRP.md`](../FRP.md).

| Stack | Package | Runs | Vehicle |
|---|---|---|---|
| gym-pybullet-drones | `sim/gym_pybullet/` | natively (Windows/Linux) | custom URDF: paper vehicle (0.2 kg, paper inertia, 0.15 m arms) |
| MuJoCo | `sim/mujoco/` | natively (Windows/Linux) | custom MJCF: same paper vehicle |
| Gazebo | `sim/gazebo/` | WSL2/Ubuntu (runbook in its README) | ardupilot_gazebo `iris`, physics in Gazebo |
| ArduPilot SITL | `sim/ardupilot/` | WSL2/Ubuntu or native Linux | ArduCopter SITL frame |

## The one invariant

`quat_sitl.controller.NonlinearP2Controller` is imported **unmodified** by
every adapter. What each adapter owns is only the plumbing: state conversion,
actuation path, logging. `sim/common/` is that shared plumbing:

- `bridge.py` — the convention proof (why `q_paper = conj(q_sim)`, derived
  from eq. 18's literal minus sign and guarded by
  `tests/test_sim_bridge.py::test_conjugate_convention_drives_standard_plant`),
  the z-up thrust-preserving mixer (twin of `pysitl/airframe.py`), and the
  altitude-hold scaffolding that keeps 6-DOF vehicles airborne.
- `scenarios.py` — the paper's three benchmarks, imported directly from
  `quat_sitl.references` (identical references on every stack).
- `telemetry.py` — one CSV schema and one metric set for all stacks.
- `plots.py` — standard attitude/torque figure pair per run.

## Quickstart

```bash
# MuJoCo (native; results → results/sim_mujoco/)
python -m sim.mujoco.run --scenario step --duration 15 --seed 0 --noise 0.1
python -m sim.mujoco.run --scenario sine
python -m sim.mujoco.run --scenario flip          # completes 2π, no gimbal-lock artifact
python -m sim.mujoco.run --scenario step --gui    # passive viewer

# gym-pybullet-drones (native)
python -m sim.gym_pybullet.run --scenario step
python -m sim.gym_pybullet.run --scenario flip

# ArduPilot SITL (Linux/WSL2, SITL already running)
python -m sim.ardupilot.bridge_node --scenario step --connection tcp:127.0.0.1:5760

# Gazebo (WSL2) — see sim/gazebo/README.md for the full runbook
bash sim/gazebo/run_gazebo.sh step

# Cross-simulator table from all collected runs
python -m sim.compare
```

Dependencies: MuJoCo adapter needs `mujoco`, `numpy`, `matplotlib`; the
gym-pybullet adapter needs `gym-pybullet-drones` (from GitHub, Python ≥ 3.12)
and `pybullet`; the ArduPilot bridge needs `pymavlink`
(`pip install -r sim/ardupilot/requirements.txt`).

## Findings so far (logged as they appeared)

1. **The ideal-plant ±4 N·m bound is not rotor-realizable.** On the paper
   vehicle the mixer can produce at most ≈ 0.4 N·m of roll/pitch torque; the
   adapters command through the same desaturation as a real ESC mixer, and the
   scenarios still track (the inertia is tiny) — but the paper's saturation
   plot is a property of its idealized control-signal-to-torque identity.
2. **A slow 2 s ramp flip needs acro-style thrust handling.** Tilt-compensated
   altitude hold keeps pushing collective while the vehicle is inverted,
   accelerating the fall and (through mixer desaturation) starving attitude
   authority — measured: flip deadlocks at exactly 180°. The flip scenario
   therefore starts at 5 m, idles collective when `tilt_cos < 0.3`, and caps
   collective while torque demand is high, exactly as acro-mode firmware does.
   Step/sine must NOT use those rules: holding 1 rad on two axes gives
   `tilt_cos = 0.29`, and idling/capping there sinks the vehicle.
3. **The 12.3 kHz control-rate bound carries over.** The MuJoCo adapter steps
   physics at 20 kHz with the controller in the loop every step and asserts
   the derived minimum at startup, same as the pysitl scheduler.

## Conventions (for anyone adding a fifth stack)

Read `sim/common/bridge.py`'s module docstring first. Summary:

- simulator quaternion `q_sim` = scalar-first **body→world** Hamilton;
  controller sees `q_m = conj(q_sim)`;
- controller torque is **body-frame** — apply in the sim's local frame
  (PyBullet `LOCAL`, MuJoCo `xfrc_applied` rotated to world, Gazebo via the
  flight stack);
- collective thrust is **body +z, up**, mixed per-rotor by `mix_zup`;
- paper sensor noise (uniform ±0.1 on q and ω) is injected at 1 kHz, the
  altitude loop runs at 250 Hz, logging decimates to 1 kHz.
