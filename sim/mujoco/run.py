"""MuJoCo adapter: the paper's unmodified controller flying the paper vehicle
in MuJoCo contact physics.

Physics plan (FRP.md section 3):
- MJCF model sim/mujoco/quadrotor.xml: paper inertia + derived mass, z-up,
  freejoint, floor contact; timestep 5e-5 s (20 kHz) clears the paper-derived
  ~12.3 kHz minimum control rate.
- Every step: read body->world quaternion + LOCAL angular velocity (both
  unambiguous via mj_objectVelocity), conjugate into the paper convention,
  compute torque via sim.common.QuatBridge, mix with the altitude-hold
  collective thrust, and apply the RESULTING wrench (not the command -- the
  vehicle feels what the mixer produces) via data.xfrc_applied in world frame.
- Paper measurement noise (uniform +/-0.1) is injected on q_m/omega_m at
  1 kHz, matching quat_sitl's sensor model; altitude hold runs at 250 Hz;
  logging decimated to 1 kHz.

Usage:
    python -m sim.mujoco.run --scenario step --duration 15 --seed 0 --noise 0.1
    python -m sim.mujoco.run --scenario flip --gui
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from quat_sitl import quaternion as quat
from sim.common import (
    AltitudeHold,
    PAPER_VEHICLE,
    QuatBridge,
    RunLog,
    summarize,
    thrusts_to_wrench_zup,
)
from sim.common.plots import plot_run
from sim.common.scenarios import (
    FLIP_RAMP_6DOF,
    default_duration,
    reference_quat,
    shortest_path,
)

XML_PATH = Path(__file__).with_name("quadrotor.xml")
VEHICLE_MASS = 0.2  # kg -- same pysitl derivation the MJCF inertial encodes

SENSOR_HZ = 1_000.0
ALTITUDE_HZ = 250.0
LOG_HZ = 1_000.0
Z_REF = 0.5  # m, hover altitude for step/sine
Z_REF_FLIP = 60.0  # m: the flip's ~2.9 s of near-idle collective free-falls the
# vehicle ~19.5 m (measured, seeds 0-2: dip to 40.5-40.6 m) before the
# attitude loop recovers and altitude hold climbs back; 60 m keeps the
# maneuver fully airborne with ~40 m of clearance.
IDLE_FRACTION = 0.1  # collective = 10% of hover while tilt_cos < 0.3 (acro flip)
AGGRO_CAP_FACTOR = 2.2  # collective cap (x hover) while torque demand is large,
# preserving rotor headroom for attitude -- see bridge.AltitudeHold docstring


def run(
    scenario: str,
    duration: float | None = None,
    seed: int = 0,
    noise: float = 0.1,
    gui: bool = False,
    out_root: Path | None = None,
) -> dict:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")

    dt = float(model.opt.timestep)
    control_hz = 1.0 / dt
    # Same guard as the pysitl scheduler: refuse rates the derivation forbids.
    from quat_sitl.dynamics import InertiaParams, stable_control_rate_hz

    need = stable_control_rate_hz(4.0, InertiaParams())
    assert control_hz >= need, f"MuJoCo timestep {dt}s ({control_hz:.0f} Hz) < {need:.0f} Hz minimum"

    rng = np.random.default_rng(seed)
    bridge = QuatBridge(geometry=PAPER_VEHICLE, shortest_path=shortest_path(scenario))
    altitude = AltitudeHold()
    z_ref = Z_REF_FLIP if scenario == "flip" else Z_REF
    acro = scenario == "flip"  # acro thrust handling is flip-only; step/sine
    # hold combined 1-rad tilts where tilt_cos = 0.29 -- capping or idling
    # collective there sinks the vehicle (measured), and the mixer already
    # delivers ample attitude authority at those thrusts
    duration = duration if duration is not None else default_duration(scenario)

    log = RunLog(simulator="mujoco", scenario=scenario, noise=noise)
    vel = np.zeros(6)
    q_meas = np.array([1.0, 0.0, 0.0, 0.0])
    omega_meas = np.zeros(3)

    # the MJCF's static spawn pose is 0.5 m; hover at the scenario's altitude
    # (the flip starts high -- its ~2.9 s of near-idle collective free-falls
    # ~43 m, so 60 m clears the floor with margin and the flip stays airborne)
    data.qpos[2] = z_ref

    viewer = None
    if gui:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(model, data)

    n_steps = int(round(duration / dt))
    sensor_every = max(1, int(round(control_hz / SENSOR_HZ)))
    alt_every = max(1, int(round(control_hz / ALTITUDE_HZ)))
    log_every = max(1, int(round(control_hz / LOG_HZ)))

    for k in range(n_steps):
        t = k * dt

        # --- sensors (paper noise at 1 kHz) --------------------------------
        if k % sensor_every == 0:
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, vel, 1)
            q_sim = np.array(data.qpos[3:7])  # MuJoCo freejoint quat: w-first, body->world
            omega = vel[0:3].copy()  # LOCAL angular velocity [rad/s]
            q_meas = quat.normalize(q_sim + rng.uniform(-noise, noise, 4))
            omega_meas = omega + rng.uniform(-noise, noise, 3)

        # --- paper controller, every step -----------------------------------
        q_ref = reference_quat(scenario, t, flip_ramp=FLIP_RAMP_6DOF if scenario == "flip" else None)
        q_m = quat.conj(q_meas)  # bridge convention: paper q = conj(body->world)
        tau, sat = bridge.torque(q_ref, q_m, omega_meas)

        # --- collective thrust (250 Hz), acro-aware ------------------------
        # Order matters: torque demand gates the thrust cap, mirroring real
        # acro firmware -- attitude authority first, altitude second. The
        # near-idle branch is FLIP-ONLY: a step holding 1 rad on BOTH roll and
        # pitch gives tilt_cos = cos(phi)cos(theta) = 0.29, and idling there
        # would drop a vehicle that is perfectly able to hold itself up.
        if k % alt_every == 0:
            z = float(data.qpos[2])
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, vel, 0)
            vz = float(vel[5])  # linear z, WORLD frame (up-positive)
            R_bw = quat.to_dcm(np.array(data.qpos[3:7]))
            tilt_cos = float(np.clip(R_bw[2, 2], -1.0, 1.0))
            hover = VEHICLE_MASS * 9.81
            if acro and tilt_cos < 0.3:
                # inverted/near-inverted: near-idle collective, full rotor
                # headroom for torque (thrust along body +z points DOWN now;
                # pushing it would only accelerate the fall)
                thrust = IDLE_FRACTION * hover
            else:
                thrust = altitude.thrust(
                    z_ref, z, vz, tilt_cos, VEHICLE_MASS,
                    PAPER_VEHICLE.thrust_max_total, dt * alt_every,
                )
                if acro and np.max(np.abs(tau)) > 0.3:
                    thrust = min(thrust, AGGRO_CAP_FACTOR * hover)

        thrusts, desat = bridge.mix(thrust, tau)
        fz, tau_actual = thrusts_to_wrench_zup(thrusts, PAPER_VEHICLE)

        R_bw = quat.to_dcm(np.array(data.qpos[3:7]))
        force_world = R_bw @ np.array([0.0, 0.0, fz])
        torque_world = R_bw @ tau_actual
        data.xfrc_applied[body_id, 0:3] = force_world
        data.xfrc_applied[body_id, 3:6] = torque_world

        mujoco.mj_step(model, data)

        if viewer is not None and k % 40 == 0:
            viewer.sync()

        if k % log_every == 0:
            log.add(
                t, q_ref, q_m, omega_meas, tau, sat.astype(float),
                thrust, desat, np.array(data.qpos[0:3]),
                bridge.attitude_error_angle(q_ref, q_m),
            )

    if viewer is not None:
        viewer.close()

    out_dir = (out_root or (Path.cwd() / "results" / "sim_mujoco"))
    stamp = f"{scenario}_seed{seed}"
    log.write(out_dir / f"{stamp}.csv")
    plot_run(log, out_dir)
    return summarize(log)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", choices=["step", "sine", "flip"], default="step")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.1)
    ap.add_argument("--gui", action="store_true")
    args = ap.parse_args()
    print(run(args.scenario, args.duration, args.seed, args.noise, args.gui))
