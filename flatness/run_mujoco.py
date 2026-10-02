"""The flatness + double-loop LQR (Choutri & Lagha 2017) flying in MuJoCo.

Same controller stack as flatness/run_paper.py (outer position LQR ->
flatness map -> inner attitude LQR, 200 Hz ZOH as the paper states), but the
plant is the repo's MuJoCo contact physics (sim/mujoco/quadrotor.xml, 20 kHz,
z-up, freejoint, floor contact) instead of the paper's ideal Eq. (8)
integrator. Integration mirrors sim/mujoco/run.py: read body->world quaternion
and LOCAL angular velocity via mj_objectVelocity, mix (T, tau) through the
same priority-desaturating Eq. (9) mixer, and apply the RESULTING wrench (the
vehicle feels what the mixer produces, not the command) via xfrc_applied in
world frame.

Usage:
    python -m flatness.run_mujoco [--cycles 2] [--speed 0.5] [--noise 0.0] [--gui]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from flatness.flatness import attitude_error, flatness_reference
from flatness.lqr import AttitudeLQR, PositionLQR
from flatness.model import QuadrotorModel, Vehicle
from flatness.trajectories import CircleTrajectory
from flatness.run_paper import _plots

CTRL_HZ = 200.0  # the paper's controller rate (Section V)
XML_PATH = Path(__file__).resolve().parent.parent / "sim" / "mujoco" / "quadrotor.xml"
SPAWN_Z = 0.1  # small floor clearance (the reference's z starts at 0 and ramps up)


class ActuatorLag:
    """First-order ESC/motor lag on the commanded wrench — the same
    actuator model as flatness/model.py (paper Section V "limitations over
    the actuators"). Without it the raw 200 Hz ZOH torque commands are
    marginally stable in contact physics: the inner-loop bandwidth rides
    the ZOH delay margin and the lateral mode diverges (measured: without
    the filter the ideal Eq. (8) model diverges identically, 1.7 m)."""

    def __init__(self, tau: float, dt: float):
        self.alpha = 1.0 - np.exp(-dt / tau) if tau > 0 else 1.0
        self.T = 0.0
        self.tau = np.zeros(3)

    def update(self, T: float, tau: np.ndarray) -> tuple[float, np.ndarray]:
        self.T += self.alpha * (T - self.T)
        self.tau += self.alpha * (tau - self.tau)
        return self.T, self.tau


def run(speed: float = 0.5, cycles: float = 2.0, seed: int = 0, noise: float = 0.0,
        out_root: Path | None = None, show: bool = False,
        frame_cb=None, xml_path: Path | None = None) -> dict:
    """`frame_cb(t, data)` (if given) is called at every 200 Hz control tick —
    the offscreen recorder (flatness/record_mujoco.py) uses it to capture
    in-engine frames without duplicating the control loop. `xml_path` swaps
    the plant scene (the recorder uses flatness/circle_scene.xml, which adds
    only visual-only markers)."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(xml_path or XML_PATH))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    dt = float(model.opt.timestep)
    ctrl_every = max(1, int(round((1.0 / dt) / CTRL_HZ)))

    traj = CircleTrajectory(radius=1.0, alt=1.0, speed=speed)
    vehicle = Vehicle()
    mixer = QuadrotorModel(vehicle)      # Eq. (9) mixer only (priority desat)
    outer = PositionLQR()
    inner = AttitudeLQR()
    rng = np.random.default_rng(seed)

    # the paper's start point, slightly above the floor for contact clearance
    data.qpos[0:3] = (-1.0, 0.0, SPAWN_Z)

    duration = traj.climb_time + cycles * 2.0 * np.pi / traj.omega
    n_steps = int(round(duration / dt))

    log = {k: [] for k in ("t", "p", "p_ref", "q", "q_d", "T", "tau", "w")}
    vel = np.zeros(6)
    T = vehicle.mass * 9.81
    tau = np.zeros(3)
    q_d = np.array([1.0, 0.0, 0.0, 0.0])
    act = ActuatorLag(vehicle.esc_tau, dt)
    act.T = T  # initialize the filter at hover

    viewer = None
    if show:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(model, data)

    for k in range(n_steps):
        t = k * dt

        # --- controller at 200 Hz, zero-order hold ------------------------
        if k % ctrl_every == 0:
            q_sim = np.array(data.qpos[3:7])        # w-first, body->world
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                                     body_id, vel, 1)
            omega = vel[0:3].copy()                  # LOCAL angular velocity
            p = np.array(data.qpos[0:3])
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                                     body_id, vel, 0)
            v = vel[3:6].copy()                      # world linear velocity
            if noise > 0.0:                           # optional sensor model
                q_sim = q_sim + rng.uniform(-noise, noise, 4)
                omega = omega + rng.uniform(-noise, noise, 3)

            ref = traj.evaluate(t)
            a_cmd = outer.acceleration(p, v, ref.p, ref.v, ref.a)
            T, q_d = flatness_reference(ref, vehicle, a_cmd=a_cmd)
            q_vec_err, _ = attitude_error(q_d, q_sim)
            tau = inner.torque(q_vec_err, omega)

            log["t"].append(t)
            log["p"].append(p.copy())
            log["p_ref"].append(ref.p.copy())
            log["q"].append(np.array(data.qpos[3:7]))
            log["q_d"].append(q_d.copy())
            log["T"].append(T)
            log["tau"].append(tau.copy())

        # --- apply the ACTUAL mixed wrench (Eq. 9 output through the ESC
        # lag), world frame -----------------------------------------------
        T_lag, tau_lag = act.update(T, tau)
        w = mixer.mix(T_lag, tau_lag)
        T_eff, tau_eff = mixer.wrench(w)
        if k % ctrl_every == 0:
            log["w"].append(w.copy())
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, np.array(data.qpos[3:7]))
        R = R.reshape(3, 3)
        data.xfrc_applied[body_id, 0:3] = R @ np.array([0.0, 0.0, T_eff])
        data.xfrc_applied[body_id, 3:6] = R @ tau_eff

        mujoco.mj_step(model, data)

        if viewer is not None and k % 40 == 0:
            viewer.sync()
        if frame_cb is not None and k % ctrl_every == 0 and k > 0:
            frame_cb(t, data)

    if viewer is not None:
        viewer.close()

    for key in log:
        log[key] = np.asarray(log[key])

    mask = log["t"] >= traj.climb_time
    err = log["p"][mask] - log["p_ref"][mask]
    summary = {
        "simulator": "flatness_lqr_mujoco",
        "scenario": "circle_r1_z1",
        "duration_s": float(log["t"][-1]),
        "rms_x_m": float(np.sqrt(np.mean(err[:, 0] ** 2))),
        "rms_y_m": float(np.sqrt(np.mean(err[:, 1] ** 2))),
        "rms_z_m": float(np.sqrt(np.mean(err[:, 2] ** 2))),
        "rms_radial_m": float(np.sqrt(np.mean(
            (np.linalg.norm(log["p"][mask][:, :2], axis=1) - 1.0) ** 2))),
        "max_rotor_thrust_n": float(np.max(vehicle.kf * log["w"])),
    }

    out_dir = out_root or (Path.cwd() / "results" / "flatness")
    out_dir.mkdir(parents=True, exist_ok=True)
    np.savez(out_dir / "mujoco_circle_log.npz", **log)  # recordings source
    _plots(log, vehicle, out_dir, show, label=" (MuJoCo contact physics)",
           prefix="mujoco_")
    return summary


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--cycles", type=float, default=2.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--gui", action="store_true")
    args = ap.parse_args()
    print(run(speed=args.speed, cycles=args.cycles, seed=args.seed,
             noise=args.noise, show=args.gui))
