"""Animated GIF recordings of every simulation result in the repo.

- The paper's 3 attitude tests (STEP / SINE / FLIP) on gym-pybullet-drones
  and MuJoCo, animated from the benchmark CSV logs: the color-coded drone
  frame (RED front arms, BLUE rear — same paint as the interactive sims)
  plus an RGB triad for the paper's REFERENCE attitude, with live time and
  quaternion tracking error.
- The flatness path-planning circle (Choutri & Lagha 2017) on the ideal
  Eq. (8) model and on MuJoCo contact physics, animated from the npz logs:
  the 3D path drawing itself under the drone, with live radial/altitude
  error.

Quaternion convention (verified against the CSV euler columns): both q_ref
and q_m in the benchmark logs are body->world, scalar-first — drawable
directly with quat_sitl.to_dcm.

Usage:  python make_recordings.py    (numpy + matplotlib only, no sim envs)
Output: results/recordings/*.gif
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Line3D

from quat_sitl import quaternion as quat

REPO = Path(__file__).resolve().parent
OUT = REPO / "results" / "recordings"
N_FRAMES = 120
FPS = 14

# arm tips in body frame (front = +x), same layout/paint as the URDF
_TIPS = np.array([[1, -1, 0], [1, 1, 0], [-1, 1, 0], [-1, -1, 0]], float)
_ARM_COLORS = ("red", "red", "blue", "blue")   # FR, FL front; RL, RR rear
_TRIAD_COLORS = ("red", "green", "blue")


def _drone(ax):
    """Pre-create the drone's 4 arm lines (unit layout scaled on update)."""
    return [ax.plot([], [], [], color=c, lw=4, solid_capstyle="round")[0]
            for c in _ARM_COLORS]


def _triad(ax):
    return [ax.plot([], [], [], color=c, lw=2.5, ls="--")[0]
            for c in _TRIAD_COLORS]


def _set_arm(ln: Line3D, pos: np.ndarray, R: np.ndarray, scale: float,
             tip: np.ndarray) -> None:
    end = pos + scale * (R @ tip)
    ln.set_data_3d([pos[0], end[0]], [pos[1], end[1]], [pos[2], end[2]])


def _save(anim: FuncAnimation, path: Path) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    anim.save(str(path), writer=PillowWriter(fps=FPS), dpi=85)
    print(f"wrote {path.name} ({path.stat().st_size // 1024} KB)")


def record_attitude(csv_path: Path, title: str, out_name: str,
                    arm_scale: float = 1.0) -> None:
    """Animate one benchmark attitude test: drone frame + reference triad."""
    d = np.genfromtxt(csv_path, delimiter=",", names=True, skip_header=1)
    q_ref = np.stack([d[f"q_ref{i}"] for i in range(4)], axis=1)
    q_m = np.stack([d[f"q_m{i}"] for i in range(4)], axis=1)
    t = d["t"]
    err_deg = np.degrees(d["alpha_err"])

    idx = np.linspace(0, len(t) - 1, N_FRAMES).astype(int)

    fig = plt.figure(figsize=(5.4, 4.4))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-1.7, 1.7); ax.set_ylim(-1.7, 1.7); ax.set_zlim(-1.7, 1.7)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=18, azim=-55)
    ax.set_axis_off()
    arms = _drone(ax)
    tri = _triad(ax)
    title_obj = ax.set_title("", fontsize=10)

    origin = np.zeros(3)

    def frame(i: int):
        j = idx[i]
        R = quat.to_dcm(quat.normalize(q_m[j]))       # body->world
        R_ref = quat.to_dcm(quat.normalize(q_ref[j]))
        for ln, tip in zip(arms, _TIPS):
            _set_arm(ln, origin, R, arm_scale, tip)
        for k, ln in enumerate(tri):
            end = arm_scale * 1.45 * (R_ref.T @ np.eye(3)[k])
            ln.set_data_3d([0, end[0]], [0, end[1]], [0, end[2]])
        title_obj.set_text(f"{title}\nt = {t[j]:5.2f} s   err = {err_deg[j]:5.1f} deg")
        return arms + tri

    _save(FuncAnimation(fig, frame, frames=N_FRAMES, blit=False),
          OUT / out_name)
    plt.close(fig)


def record_circle(npz_path: Path, title: str, out_name: str) -> None:
    """Animate the flatness circle: 3D path drawn under the drone frame."""
    log = np.load(npz_path)
    t, p, q = log["t"], log["p"], log["q"]
    idx = np.linspace(0, len(t) - 1, N_FRAMES).astype(int)

    fig = plt.figure(figsize=(5.6, 4.6))
    ax = fig.add_subplot(111, projection="3d")
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(-np.cos(th), np.sin(th), np.ones_like(th), "k--", lw=1, alpha=0.5)
    ax.plot(-np.cos(th), np.sin(th), np.zeros_like(th), "k:", lw=0.8, alpha=0.3)
    ax.set_xlim(-1.6, 1.6); ax.set_ylim(-1.6, 1.6); ax.set_zlim(-0.1, 1.6)
    ax.set_box_aspect((1, 1, 0.8))
    ax.view_init(elev=22, azim=-55)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    (path_ln,) = ax.plot([], [], [], color="crimson", lw=2)
    arms = _drone(ax)
    title_obj = ax.set_title("", fontsize=10)

    def frame(i: int):
        j = idx[i]
        R = quat.to_dcm(quat.normalize(q[j]))
        for ln, tip in zip(arms, _TIPS):
            _set_arm(ln, p[j], R, 0.16, tip)
        path_ln.set_data_3d(p[: j + 1, 0], p[: j + 1, 1], p[: j + 1, 2])
        radial = np.linalg.norm(p[j, :2])
        title_obj.set_text(
            f"{title}\nt = {t[j]:5.1f} s   radial = {radial:5.3f} m (ref 1.000)"
            f"   z = {p[j, 2]:5.3f} m (ref 1.000)")
        return [path_ln] + arms

    _save(FuncAnimation(fig, frame, frames=N_FRAMES, blit=False),
          OUT / out_name)
    plt.close(fig)


def main() -> None:
    tests = [("step", "STEP test"), ("sine", "SINE test"), ("flip", "FLIP test")]
    for sim, pretty in (("sim_gym_pybullet", "gym-pybullet-drones (24 kHz)"),
                        ("sim_mujoco", "MuJoCo contact physics (20 kHz)")):
        for scen, label in tests:
            csv = REPO / "results" / sim / f"{scen}_seed0.csv"
            if csv.exists():
                record_attitude(csv, f"{label} — {pretty}",
                                f"{sim.replace('sim_', '')}_{scen}.gif")

    jobs = [
        (REPO / "results" / "flatness" / "circle_log.npz",
         "Flatness + LQR circle — paper Eq. (8) model",
         "flatness_ideal_circle.gif"),
        (REPO / "results" / "flatness" / "mujoco_circle_log.npz",
         "Flatness + LQR circle — MuJoCo contact physics",
         "flatness_mujoco_circle.gif"),
    ]
    for npz, title, name in jobs:
        if npz.exists():
            record_circle(npz, title, name)


if __name__ == "__main__":
    main()
