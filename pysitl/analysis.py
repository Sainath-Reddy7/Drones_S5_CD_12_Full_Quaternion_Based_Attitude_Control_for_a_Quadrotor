"""Post-flight analysis plots, Flight-Review-style: attitude vs reference,
torque vs the paper's saturation limit, altitude, and ground track. Mirrors
quat_sitl.plotting's dashed=reference/solid=output convention so figures from
both packages read consistently.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

REF_COLOR = "#ffab3d"
OUT_COLOR = "#42c9ff"
AXIS_COLORS = ("#42c9ff", "#3ddc84", "#ffab3d")


def plot_flight(df: pd.DataFrame, title: str, save_path: str | Path, dpi: int = 150) -> None:
    save_path = Path(save_path)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    for ax, out, ref, label in zip(
        axes, ["phi", "theta", "psi"], ["phi_ref", "theta_ref", "psi_ref"], ["Phi [rad]", "Theta [rad]", "Psi [rad]"]
    ):
        ax.plot(df["time"], df[ref], "--", color=REF_COLOR, label="reference")
        ax.plot(df["time"], df[out], "-", color=OUT_COLOR, label="output")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"{title} — attitude")
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(save_path.with_name(save_path.stem + "_attitude.png"), dpi=dpi)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    axes[0].plot(df["time"], df["altitude"], color=OUT_COLOR)
    axes[0].set_ylabel("Altitude [m]")
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f"{title} — altitude & torque")
    for col, color, label in zip(["tau_x", "tau_y", "tau_z"], AXIS_COLORS, ["Mx", "My", "Mz"]):
        axes[1].plot(df["time"], df[col], color=color, label=label)
    axes[1].set_ylabel("Torque [N·m]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(save_path.with_name(save_path.stem + "_flight.png"), dpi=dpi)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(df["y"], df["x"], color=OUT_COLOR)
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title(f"{title} — ground track")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(save_path.with_name(save_path.stem + "_track.png"), dpi=dpi)
    plt.close(fig)
