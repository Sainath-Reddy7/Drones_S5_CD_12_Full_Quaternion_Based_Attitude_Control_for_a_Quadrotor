"""Shared result figures for every adapter: attitude tracking (reference
dashed vs. measured solid, Euler for interpretability -- the loop itself stays
in quaternion space) and torque with the paper's +/-4 N*m saturation band."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _data(log) -> np.ndarray:
    return np.array(log.rows, dtype=np.float64)


def plot_attitude(log, path: Path) -> Path:
    d = _data(log)
    t = d[:, 0]
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for i, (name, ref_col, meas_col) in enumerate(
        (("roll $\\phi$", 10, 13), ("pitch $\\theta$", 11, 14), ("yaw $\\psi$", 12, 15))
    ):
        ax = axes[i]
        ax.plot(t, np.degrees(d[:, ref_col]), "k--", lw=1.0, label="reference")
        ax.plot(t, np.degrees(d[:, meas_col]), "b-", lw=1.0, label="measured")
        ax.set_ylabel(f"{name} [deg]")
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"{log.simulator} -- {log.scenario}: attitude tracking")
    axes[-1].set_xlabel("t [s]")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def plot_torque(log, path: Path, limit: float = 4.0) -> Path:
    d = _data(log)
    t = d[:, 0]
    fig, ax = plt.subplots(figsize=(9, 4))
    for i, name in enumerate(("$\\tau_x$", "$\\tau_y$", "$\\tau_z$")):
        ax.plot(t, d[:, 19 + i], lw=1.0, label=name)
    ax.axhline(limit, color="r", ls=":", lw=0.8)
    ax.axhline(-limit, color="r", ls=":", lw=0.8)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("body torque [N$\\cdot$m]")
    ax.set_title(f"{log.simulator} -- {log.scenario}: torque (dotted = paper saturation)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def plot_run(log, out_dir: Path) -> list:
    """Write the standard figure pair for a run; returns the written paths."""
    return [
        plot_attitude(log, out_dir / f"{log.scenario}_attitude.png"),
        plot_torque(log, out_dir / f"{log.scenario}_torque.png"),
    ]
