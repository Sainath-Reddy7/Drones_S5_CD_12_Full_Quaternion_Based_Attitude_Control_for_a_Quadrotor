"""gym-pybullet-drones adapter: the paper's unmodified controller flying the
paper vehicle inside the utiasDSL gym-pybullet-drones Aviary.

How it plugs in (all public/semantics of the package, no monkeypatching):

- `paper_quad.urdf` (in this folder) is the paper vehicle -- same derived
  airframe as the MuJoCo MJCF and pysitl -- structured exactly like the
  package's cf2x.urdf because BaseAviary parses URDFs positionally and its
  `_physics` applies per-rotor forces to child links 0-3 and the yaw torque
  to link 4. It is copied idempotently into the package's assets folder and
  selected via a foreign enum member whose `.value` is the file stem.
- Control runs through the standard env API: action = 4 per-rotor RPMs,
  `CtrlAviary.step()` applies `force_i = KF * rpm_i^2` at each rotor link
  (LINK_FRAME) and `z_torque = KM * (-,+,-,+) rpm_i^2` at the center link.
- Rates: pyb_freq = ctrl_freq = 24 kHz (one physics step per action, ZOH),
  clearing the paper-derived ~12.3 kHz bound, asserted at startup like the
  other adapters. Sensors/noise at 1 kHz, altitude at 250 Hz, logging 1 kHz.

Rotor-order bridge (derived against cf2x.urdf's layout):
    sim prop0 (+d,-d) FR   my rotor 3   (mix_zup order 1..4)
    sim prop1 (-d,-d) RR   my rotor 2
    sim prop2 (-d,+d) RL   my rotor 4
    sim prop3 (+d,+d) FL   my rotor 1
and the sim's yaw-reaction pairing (props 0,2 negative; 1,3 positive) is the
MIRROR of mix_zup's, so the yaw term is negated before mixing. Position
torques are unaffected by the reorder.

Usage:
    python -m sim.gym_pybullet.run --scenario step --duration 15 --seed 0 --noise 0.1
    python -m sim.gym_pybullet.run --scenario flip
    python -m sim.gym_pybullet.run --scenario sine --gui   # slow: renders at pyb rate
"""
from __future__ import annotations

import argparse
import shutil
from enum import Enum
from pathlib import Path

import numpy as np

from quat_sitl import quaternion as quat
from quat_sitl.dynamics import InertiaParams, stable_control_rate_hz
from sim.common import AltitudeHold, PAPER_VEHICLE, QuatBridge, RotorGeometry, RunLog, summarize
from sim.common.plots import plot_run
from sim.common.scenarios import (
    FLIP_RAMP_6DOF,
    default_duration,
    reference_quat,
    shortest_path,
)

CTRL_HZ = 24_000
SENSOR_HZ = 1_000.0
ALTITUDE_HZ = 250.0
LOG_HZ = 1_000.0
Z_REF = 0.5
Z_REF_FLIP = 60.0  # same free-fall margin analysis as sim/mujoco/run.py
IDLE_FRACTION = 0.1
AGGRO_CAP_FACTOR = 2.2
VEHICLE_MASS = 0.2  # kg, pysitl-derived paper vehicle
KF = 3.0e-9  # N per rpm^2 -- must match paper_quad.urdf

# sim prop i flies my-mix rotor MY_TO_SIM[i] (see module docstring)
MY_TO_SIM = (2, 1, 3, 0)
GEOMETRY = RotorGeometry(kf=KF)


def _install_urdf() -> None:
    """Copy paper_quad.urdf into the package assets (idempotent)."""
    import gym_pybullet_drones

    assets = Path(gym_pybullet_drones.__file__).parent / "assets"
    src = Path(__file__).with_name("paper_quad.urdf")
    dest = assets / src.name
    if not dest.exists():
        shutil.copy(src, dest)


def _paper_drone_enum():
    """Foreign enum member: BaseAviary only uses .value (+ inequality checks
    against its own members, which all come out False -> the generic X-config
    branches are taken, which is what this vehicle is)."""
    return Enum("PaperDrone", {"PAPER_QUAD": "paper_quad"}).PAPER_QUAD


def run(
    scenario: str,
    duration: float | None = None,
    seed: int = 0,
    noise: float = 0.1,
    gui: bool = False,
    out_root: Path | None = None,
) -> dict:
    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

    need = stable_control_rate_hz(4.0, InertiaParams())
    assert CTRL_HZ >= need, f"ctrl rate {CTRL_HZ} Hz < derived minimum {need:.0f} Hz"

    _install_urdf()
    z_ref = Z_REF_FLIP if scenario == "flip" else Z_REF
    acro = scenario == "flip"  # same reasoning as sim/mujoco/run.py: step/sine
    # hold combined 1-rad tilts (tilt_cos = 0.29) where idling/capping sinks

    env = CtrlAviary(
        drone_model=_paper_drone_enum(),
        num_drones=1,
        initial_xyzs=np.array([[0.0, 0.0, z_ref]]),
        initial_rpys=np.zeros((1, 3)),
        pyb_freq=CTRL_HZ,
        ctrl_freq=CTRL_HZ,
        gui=gui,
        user_debug_gui=False,
        output_folder="results",
    )

    rng = np.random.default_rng(seed)
    bridge = QuatBridge(geometry=GEOMETRY, shortest_path=shortest_path(scenario))
    altitude = AltitudeHold()
    duration = duration if duration is not None else default_duration(scenario)
    # BaseAviary.GRAVITY is the vehicle's WEIGHT (M*g), not g -- the gravity
    # acceleration actually set in PyBullet is BaseAviary.G
    gravity = float(getattr(env, "G", 9.8))

    log = RunLog(simulator="gym_pybullet", scenario=scenario, noise=noise)
    q_meas = np.array([1.0, 0.0, 0.0, 0.0])
    omega_meas = np.zeros(3)
    thrust = VEHICLE_MASS * gravity

    sensor_every = max(1, int(round(CTRL_HZ / SENSOR_HZ)))
    alt_every = max(1, int(round(CTRL_HZ / ALTITUDE_HZ)))
    log_every = max(1, int(round(CTRL_HZ / LOG_HZ)))

    obs, _ = env.reset()
    # PyBullet's default damping (0.04 linear/angular) is aerodynamically
    # wrong for a quadrotor airframe and, at flip rotation rates, eats the
    # angular momentum the flip coasts on -- gym-pybullet-drones ships the
    # same removal commented out in BaseAviary. Applied explicitly here.
    import pybullet as p

    for i in env.DRONE_IDS:
        p.changeDynamics(int(i), -1, linearDamping=0.0, angularDamping=0.0,
                         physicsClientId=env.CLIENT)
    dt = 1.0 / CTRL_HZ
    n_steps = int(round(duration / dt))

    for k in range(n_steps):
        t = k * dt
        pos = obs[0, 0:3]
        quat_xyzw = obs[0, 3:7]           # PyBullet convention
        vel_world = obs[0, 10:13]
        ang_v_world = obs[0, 13:16]

        q_sim = np.array(
            [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
        )  # -> scalar-first, body->world
        R_bw = quat.to_dcm(q_sim)
        omega_body = R_bw.T @ ang_v_world

        # --- sensors (paper noise at 1 kHz) --------------------------------
        if k % sensor_every == 0:
            q_meas = quat.normalize(q_sim + rng.uniform(-noise, noise, 4))
            omega_meas = omega_body + rng.uniform(-noise, noise, 3)

        # --- paper controller, every step -----------------------------------
        q_ref = reference_quat(scenario, t, flip_ramp=FLIP_RAMP_6DOF if scenario == "flip" else None)
        q_m = quat.conj(q_meas)
        tau, sat = bridge.torque(q_ref, q_m, omega_meas)

        # --- collective thrust (250 Hz), acro-aware (flip only) -------------
        if k % alt_every == 0:
            tilt_cos = float(np.clip(R_bw[2, 2], -1.0, 1.0))
            hover = VEHICLE_MASS * gravity
            if acro and tilt_cos < 0.3:
                thrust = IDLE_FRACTION * hover
            else:
                thrust = altitude.thrust(
                    z_ref, float(pos[2]), float(vel_world[2]), tilt_cos, VEHICLE_MASS,
                    PAPER_VEHICLE.thrust_max_total, dt * alt_every,
                )
                if acro and np.max(np.abs(tau)) > 0.3:
                    thrust = min(thrust, AGGRO_CAP_FACTOR * hover)

        # --- mix, sim rotor order + mirrored yaw pairing --------------------
        tau_mixed = tau * np.array([1.0, 1.0, -1.0])
        thrusts, desat = bridge.mix(thrust, tau_mixed)
        thrusts_sim = [thrusts[j] for j in MY_TO_SIM]
        rpms = bridge.rpms(thrusts_sim)

        obs, _, _, _, _ = env.step(np.array(rpms)[None, :])

        if k % log_every == 0:
            log.add(
                t, q_ref, q_m, omega_meas, tau, sat.astype(float),
                thrust, desat, np.array(pos),
                bridge.attitude_error_angle(q_ref, q_m),
            )

    env.close()

    out_dir = out_root or (Path.cwd() / "results" / "sim_gym_pybullet")
    log.write(out_dir / f"{scenario}_seed{seed}.csv")
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
