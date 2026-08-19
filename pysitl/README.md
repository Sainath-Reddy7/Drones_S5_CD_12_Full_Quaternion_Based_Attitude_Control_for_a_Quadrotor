# pysitl — a PX4/Gazebo-style SITL, in pure Python

`pysitl` wraps the attitude-only paper reproduction in `../quat_sitl/` in a
full 6-DOF quadrotor flight simulation with a PX4-shaped *architecture*: a
uORB-style pub/sub bus, a multi-rate lockstep scheduler, an arming/flight-mode
commander, control allocation (a mixer), structured logging, and a browser
ground station. No PX4, Gazebo, ROS 2, or MAVSDK anywhere — stdlib + numpy +
matplotlib + pandas, same as the rest of this repo.

**The content stays paper-faithful; the architecture is PX4-faithful.**
`quat_sitl.controller.NonlinearP2Controller` — Pq=20, Pω=4, eq. 19-21 — flies
the vehicle completely unmodified. Measurements are the paper's own model:
uniform ±0.1 noise on the quaternion and body rates, nothing else (no state
estimator — the paper has none, so neither does this). The three paper
scenarios (step / sine / flip) are selectable flight modes.

## Run it

```bash
python -m pysitl.run --gcs                                  # interactive: opens http://127.0.0.1:8765
python -m pysitl.run --mode auto_step --duration 15 --log    # headless: paper step scenario, CSV + plots
python -m pysitl.run --mode auto_sine --duration 15 --log
python -m pysitl.run --mode auto_flip --duration 5 --log
pytest tests/test_sitl.py -v
```

In the dashboard: **Arm**, pick a flight mode, and fly with `W/S` (pitch),
`A/D` (roll), `Q/E` (yaw rate), `↑/↓` (throttle). STABILIZED throttle is
direct thrust; ALTITUDE throttle is a climb-rate command around a held
target. AUTO_STEP/SINE/FLIP run the paper's own scenarios. Full control
reference, layout, and troubleshooting: [`UI_GUIDE.md`](UI_GUIDE.md).

## Why mass and geometry, when the paper only gives inertia

The paper models control→torque as the identity and never introduces rotors,
so it never needed mass or arm length. Flying a real vehicle needs both, so
they're *derived* from the paper's own inertia rather than picked freely:
`Izz/Ixx = 1.846` is extremely close to the `2.0` a planar-X point-mass quad
predicts exactly, so that's the assumed layout. Solving the two rotor-plus-
body inertia equations for both published values gives `L = 0.15 m`,
`m_rotor ≈ 12.2 g`, and (with an assumed 150 g central body) **total mass
≈ 200 g, hover thrust ≈ 1.95 N**. See `params.py`'s docstring for the full
derivation and `test_vehicle_inertia_matches_paper` for the check.

## Three things that would have quietly broken this, found before they did

**1. Frame convention.** `quat_sitl`'s eq. 18 has a leading minus sign the
paper itself uses (see the main README's "Results vs Paper"). Verified by
integrating a known body rate and checking where the body x-axis lands in
world frame: `to_dcm(q)` maps world→body, and `to_dcm(q).T` — equivalently
`quat.rotate(quat.conj(q), v)` — maps body→world. Backwards, and the vehicle
translates mirrored. `plant.rotate_body_to_world` is the one place this
matters; `test_body_to_world_convention` pins it down.

**2. The hot loop has to be scalar Python, not numpy.** The paper's gains
force a ~12.3 kHz inner loop (`quat_sitl.dynamics.stable_control_rate_hz`).
Measured: a 13-state 6-DOF RK4 step costs 65.5 µs in numpy (1.24× real-time —
not usable for interactive flying) vs. 5.6 µs in plain Python floats/tuples
(14.5× real-time). `plant.py`'s derivative and integrator are scalar for
this reason; numpy stays for setup, logging, and plotting. The controller
call itself still uses numpy (see `control/attitude.py`'s docstring for that
tradeoff — it costs real-time margin but means the vehicle is flown by the
user's actual unmodified algorithm).

**3. Naive per-rotor torque clipping breaks total thrust.** When a torque
command saturates, clipping each rotor independently sends two to max and
two to zero — total thrust changes (measured: it can double, sending
altitude to 154 m on a single 0.3 rad roll command). `airframe.mix` instead
holds the common-mode thrust fixed and scales only the zero-sum torque-driven
deltas, so total thrust is exactly preserved regardless of how hard the
torque saturates. `test_desaturation_preserves_total_thrust_under_saturation`
is the regression test for this.

## Architecture

```
pysitl/
  params.py     VehicleParams (derived, see above), GainParams, SimParams
  plant.py      13-state 6-DOF rigid body, scalar hot path, eq.18 + ground contact
  airframe.py   rotor geometry, mixer, thrust-preserving desaturation, motor lag
  sensors.py    the paper's own noise model, scalar/seeded
  bus.py        uORB-style typed pub/sub (latest-value + sequence numbers)
  topics.py     the message types modules publish/subscribe
  scheduler.py  multi-rate deterministic lockstep scheduler
  modes.py      flight modes + arming/preflight/failsafe commander
  sim.py        orchestrator: wires everything above into one Simulation
  control/
    attitude.py   thin adapter over the UNMODIFIED quat_sitl controller
    altitude.py   altitude PID + tilt compensation (new -- the paper has no translation)
    manual.py     stick input -> attitude reference / thrust
    autopilot.py  the paper's step/sine/flip as AUTO flight modes
  recorder.py   ULog-style structured logging -> CSV
  analysis.py   post-flight plots (attitude, altitude, torque, ground track)
  gcs/          stdlib http.server dashboard: SSE telemetry, POST controls
  run.py        CLI
```

Every module talks through `bus.py`, never by direct call, even in-process —
that's deliberately the single biggest thing that makes this feel like PX4
rather than a monolithic script (see `bus.py`'s docstring).

**Scheduler rates:** 16 kHz base (physics + attitude loop + mixer), 1 kHz
sensors (paper noise, zero-order hold between samples — not redrawn every
base tick, which would inject far more high-frequency energy than a real
sensor), 250 Hz altitude, 50 Hz commander, 200 Hz logger. `Scheduler` refuses
to construct below the numerically-required stable rate for the configured
gains/inertia rather than trusting the caller.

## Testing

`tests/test_sitl.py` is separate from `tests/test_quaternion.py`, whose
7-test count is a prior contract for the paper reproduction — this file adds
frame-convention, hover, mixer/desaturation, eq.18 parity, closed-loop
tracking under noise, scheduler determinism, commander/failsafe, and bus
tests without touching that contract. `pytest tests/ -v` runs both; both
pass independently, since `pysitl` only imports from `quat_sitl` and never
modifies it.
