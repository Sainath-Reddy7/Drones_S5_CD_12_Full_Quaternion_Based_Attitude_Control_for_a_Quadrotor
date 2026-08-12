"""Real-time-ish 3D attitude playback: replays a logged scenario CSV (or a live
DataFrame from simulator.run_simulation) as a spinning quadrotor frame in a
matplotlib 3D window. Attitude-only (no translation is simulated by this
project) — the drone spins in place so step/sine/flip maneuvers are visible.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Line3D

from . import quaternion as quat

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"

# Body-frame quadrotor geometry: X configuration, 4 arm tips + center.
ARM_LENGTH = 0.15
_ARM_TIPS_BODY = np.array(
    [
        [ARM_LENGTH, ARM_LENGTH, 0.0],
        [-ARM_LENGTH, -ARM_LENGTH, 0.0],
        [ARM_LENGTH, -ARM_LENGTH, 0.0],
        [-ARM_LENGTH, ARM_LENGTH, 0.0],
    ]
)
_AXIS_LEN = 0.22


def _decimate(df: pd.DataFrame, fps: int, playback_speed: float) -> pd.DataFrame:
    duration = df["time"].iloc[-1]
    n_frames = max(2, int(duration / playback_speed * fps))
    idx = np.linspace(0, len(df) - 1, n_frames).astype(int)
    return df.iloc[idx].reset_index(drop=True)


def animate_flight(
    df: pd.DataFrame,
    fps: int = 30,
    playback_speed: float = 1.0,
    save_path: str | Path | None = None,
    title: str = "Attitude playback",
) -> FuncAnimation:
    frames = _decimate(df, fps, playback_speed)

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection="3d")
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(-0.3, 0.3)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    arm1: Line3D = ax.plot([], [], [], "o-", color="tab:blue", linewidth=3, markersize=6)[0]
    arm2: Line3D = ax.plot([], [], [], "o-", color="tab:orange", linewidth=3, markersize=6)[0]
    axis_x: Line3D = ax.plot([], [], [], "-", color="red", linewidth=2)[0]
    axis_y: Line3D = ax.plot([], [], [], "-", color="green", linewidth=2)[0]
    axis_z: Line3D = ax.plot([], [], [], "-", color="blue", linewidth=2)[0]
    time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

    def init():
        for line in (arm1, arm2, axis_x, axis_y, axis_z):
            line.set_data([], [])
            line.set_3d_properties([])
        time_text.set_text("")
        return arm1, arm2, axis_x, axis_y, axis_z, time_text

    def update(i: int):
        row = frames.iloc[i]
        q = np.array([row.q_true_0, row.q_true_1, row.q_true_2, row.q_true_3])
        R = quat.to_dcm(q)
        tips = (R @ _ARM_TIPS_BODY.T).T

        arm1.set_data([tips[0, 0], tips[1, 0]], [tips[0, 1], tips[1, 1]])
        arm1.set_3d_properties([tips[0, 2], tips[1, 2]])
        arm2.set_data([tips[2, 0], tips[3, 0]], [tips[2, 1], tips[3, 1]])
        arm2.set_3d_properties([tips[2, 2], tips[3, 2]])

        for line, body_axis in ((axis_x, [1, 0, 0]), (axis_y, [0, 1, 0]), (axis_z, [0, 0, 1])):
            tip = R @ (np.array(body_axis) * _AXIS_LEN)
            line.set_data([0, tip[0]], [0, tip[1]])
            line.set_3d_properties([0, tip[2]])

        time_text.set_text(f"t = {row.time:.2f}s")
        return arm1, arm2, axis_x, axis_y, axis_z, time_text

    ax.set_title(title)
    anim = FuncAnimation(
        fig, update, frames=len(frames), init_func=init, interval=1000 / fps, blit=False
    )

    if save_path is not None:
        anim.save(str(save_path), fps=fps)
    return anim


def main() -> None:
    parser = argparse.ArgumentParser(description="3D attitude playback")
    parser.add_argument("--scenario", choices=["step", "sine", "flip"])
    parser.add_argument("--csv", type=str)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    parser.add_argument("--save", type=str, default=None, help="save to .mp4/.gif instead of showing")
    args = parser.parse_args()

    if args.csv:
        csv_path = Path(args.csv)
    elif args.scenario:
        csv_path = RESULTS_DIR / f"{args.scenario}_latest.csv"
    else:
        parser.error("pass --scenario or --csv")

    df = pd.read_csv(csv_path)
    anim = animate_flight(df, fps=args.fps, playback_speed=args.speed, save_path=args.save, title=csv_path.stem)
    if args.save is None:
        plt.show()
    else:
        print(f"wrote {args.save}")


if __name__ == "__main__":
    main()
