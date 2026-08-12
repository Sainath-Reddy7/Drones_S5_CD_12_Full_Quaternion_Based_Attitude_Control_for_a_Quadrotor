"""Figure reproduction matching the paper's presentation: dashed=reference,
solid=output, radians, shared time axis, 150 dpi.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def plot_scenario(df: pd.DataFrame, title: str, save_path: str | Path, dpi: int = 150) -> None:
    """Two 3-row stacked-subplot figures (Euler angles, torques) matching paper
    Figs 3-4 / 5-6. save_path is used as a stem: "{stem}_euler.png", "{stem}_torque.png"."""
    save_path = Path(save_path)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    for ax, out_col, ref_col, label in zip(
        axes, ["phi", "theta", "psi"], ["phi_ref", "theta_ref", "psi_ref"], ["Phi [rad]", "Theta [rad]", "Psi [rad]"]
    ):
        ax.plot(df["time"], df[ref_col], "--", label="reference")
        ax.plot(df["time"], df[out_col], "-", label="output")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"{title} — attitude")
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(save_path.with_name(save_path.stem + "_euler.png"), dpi=dpi)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    for ax, col, label in zip(axes, ["tau_x", "tau_y", "tau_z"], ["Mx [Nm]", "My [Nm]", "Mz [Nm]"]):
        ax.plot(df["time"], df[col], "-")
        ax.axhline(4.0, linestyle="--", color="gray", alpha=0.6)
        ax.axhline(-4.0, linestyle="--", color="gray", alpha=0.6)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[0].set_title(f"{title} — control effort")
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(save_path.with_name(save_path.stem + "_torque.png"), dpi=dpi)
    plt.close(fig)


def plot_flip(df: pd.DataFrame, title: str, save_path: str | Path, dpi: int = 150) -> None:
    """2-row figure matching paper Figs 7-8: Euler phi, and raw q0/q1 (shows no
    singularity through the full 360-degree rotation)."""
    save_path = Path(save_path)

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    axes[0].plot(df["time"], df["phi_ref"], "--", label="reference")
    axes[0].plot(df["time"], df["phi"], "-", label="output")
    axes[0].set_ylabel("Phi [rad]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f"{title}")

    axes[1].plot(df["time"], df["q_true_0"], "-", label="q0")
    axes[1].plot(df["time"], df["q_true_1"], "-", label="q1")
    axes[1].set_ylabel("Quaternion")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(save_path.with_name(save_path.stem + "_flip.png"), dpi=dpi)
    plt.close(fig)
