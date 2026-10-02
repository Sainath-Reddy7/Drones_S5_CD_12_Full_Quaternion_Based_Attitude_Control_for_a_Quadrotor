"""Reproduce the Choutri & Lagha 2017 study case (Section V).

Circular path, radius 1 m at altitude 1 m, from p0 = (-1, 0, 0); double-loop
LQR at 200 Hz (the paper's stated control rate) on the full nonlinear
quaternion model of Eq. (8); actuator limits + ESC lag active. Produces the
paper's result figures:

    Fig. 4-6: X / Y / Z responses vs reference
    Fig. 7:   full 3D trajectory
    Fig. 8:   motor (PWM-equivalent) signals
    Fig. 9:   quaternion time histories

Usage:
    python -m flatness.run_paper [--speed 0.5] [--cycles 2] [--gui]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from flatness.flatness import attitude_error, flatness_reference
from flatness.lqr import AttitudeLQR, PositionLQR
from flatness.model import QuadrotorModel, Vehicle
from flatness.trajectories import CircleTrajectory

PHYS_HZ = 1000.0     # model integration rate
CTRL_HZ = 200.0      # the paper's controller rate


def run(speed: float = 0.5, cycles: float = 2.0, seed: int = 0,
        out_root: Path | None = None, show: bool = False) -> dict:
    traj = CircleTrajectory(radius=1.0, alt=1.0, speed=speed)
    vehicle = Vehicle()
    model = QuadrotorModel(vehicle, seed=seed)
    model.reset(p0=(-1.0, 0.0, 0.0))   # the paper's start point

    outer = PositionLQR()
    inner = AttitudeLQR()

    dt = 1.0 / PHYS_HZ
    ctrl_every = int(round(PHYS_HZ / CTRL_HZ))
    duration = traj.climb_time + cycles * 2.0 * np.pi / traj.omega
    n_steps = int(round(duration / dt))

    log = {k: [] for k in ("t", "p", "p_ref", "q", "q_d", "T", "tau", "w")}
    T = vehicle.mass * 9.81
    tau = np.zeros(3)

    for k in range(n_steps):
        t = k * dt
        if k % ctrl_every == 0:  # 200 Hz, zero-order hold (paper Section V)
            ref = traj.evaluate(t)
            a_cmd = outer.acceleration(model.p, model.vv, ref.p, ref.v, ref.a)
            T, q_d = flatness_reference(ref, vehicle, a_cmd=a_cmd)
            q_vec_err, _ = attitude_error(q_d, model.q)
            tau = inner.torque(q_vec_err, model.omg)
        model.step(dt, T, tau)

        if k % ctrl_every == 0:  # log at control rate
            ref = traj.evaluate(t)
            log["t"].append(t)
            log["p"].append(model.p.copy())
            log["p_ref"].append(ref.p.copy())
            log["q"].append(model.q.copy())
            log["q_d"].append(q_d.copy())
            log["T"].append(T)
            log["tau"].append(tau.copy())
            log["w"].append(model.w_cmd.copy())

    for key in log:
        log[key] = np.asarray(log[key])

    settle_t = traj.climb_time
    mask = log["t"] >= settle_t
    err = log["p"][mask] - log["p_ref"][mask]
    summary = {
        "simulator": "flatness_lqr",
        "scenario": "circle_r1_z1",
        "duration_s": float(log["t"][-1]),
        "rms_x_m": float(np.sqrt(np.mean(err[:, 0] ** 2))),
        "rms_y_m": float(np.sqrt(np.mean(err[:, 1] ** 2))),
        "rms_z_m": float(np.sqrt(np.mean(err[:, 2] ** 2))),
        "rms_radial_m": float(np.sqrt(np.mean(
            (np.linalg.norm(log["p"][mask][:, :2], axis=1) - 1.0) ** 2))),
        "max_rotor_thrust_n": float(np.max(vehicle.kf * np.asarray(log["w"]))),
    }

    out_dir = out_root or (Path.cwd() / "results" / "flatness")
    _plots(log, vehicle, out_dir, show)
    return summary


def _plots(log: dict, vehicle: Vehicle, out_dir: Path, show: bool,
           label: str = "", prefix: str = "") -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    t = log["t"]

    # Figs. 4-6: axis responses
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for i, (ax, name) in enumerate(zip(axes, "XYZ")):
        ax.plot(t, log["p"][:, i], label="actual")
        ax.plot(t, log["p_ref"][:, i], "--", label="reference")
        ax.set_ylabel(f"{name} [m]")
        ax.grid(True)
        ax.legend(loc="upper right")
    axes[0].set_title(f"Circular path tracking (paper Figs. 4-6){label}")
    axes[-1].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(out_dir / f"{prefix}axis_responses.png", dpi=150)

    # Fig. 7: full trajectory
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(log["p"][:, 0], log["p"][:, 1], log["p"][:, 2], label="flown")
    ax.plot(log["p_ref"][:, 0], log["p_ref"][:, 1], log["p_ref"][:, 2],
            "--", label="reference")
    ax.scatter([-1.0], [0.0], [0.0], marker="*", s=80, color="k", label="start")
    ax.set_zlim(0, 1.6)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
    ax.set_title(f"Full trajectory (paper Fig. 7){label}")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / f"{prefix}trajectory_3d.png", dpi=150)

    # Fig. 8: motor signals (PWM-equivalent: per-rotor thrust)
    thrusts = vehicle.kf * log["w"]
    fig, ax = plt.subplots(figsize=(8, 4))
    for i in range(4):
        ax.plot(t, thrusts[:, i], label=f"motor {i + 1}")
    ax.set_xlabel("t [s]"); ax.set_ylabel("rotor thrust [N]")
    ax.set_title(f"Motor commands (paper Fig. 8, PWM-equivalent){label}")
    ax.grid(True); ax.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(out_dir / f"{prefix}motor_signals.png", dpi=150)

    # Fig. 9: quaternions
    fig, ax = plt.subplots(figsize=(8, 4))
    for i, name in enumerate(("q0", "q1", "q2", "q3")):
        ax.plot(t, log["q"][:, i], label=name)
    ax.set_xlabel("t [s]"); ax.set_ylabel("quaternion component")
    ax.set_title(f"Attitude quaternions (paper Fig. 9){label}")
    ax.grid(True); ax.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(out_dir / f"{prefix}quaternions.png", dpi=150)

    if show:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--speed", type=float, default=0.5, help="tangential speed [m/s]")
    ap.add_argument("--cycles", type=float, default=2.0)
    ap.add_argument("--gui", action="store_true")
    args = ap.parse_args()
    print(run(speed=args.speed, cycles=args.cycles, show=args.gui))
