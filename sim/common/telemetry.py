"""Uniform logging + metrics for the cross-simulator benchmark.

Every adapter writes the SAME CSV schema (below), so sim/compare.py can build
the final evaluation table without caring which stack produced a run.

Schema (one row per control tick, or decimated to <=1 kHz for file size):
    t, alpha_err, q_ref0..3, q_m0..3, phi_ref, theta_ref, psi_ref,
    phi, theta, psi, p, q, r, tau_x, tau_y, tau_z, sat_x, sat_y, sat_z,
    thrust, desat, x, y, z

sat_* are 0/1 per-axis torque saturation flags; desat is the mixer's surviving
torque fraction in [0,1]; alpha_err is the geodesic attitude error [rad].
"""
from __future__ import annotations

import csv
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from quat_sitl import quaternion as quat

FloatArr = NDArray[np.float64]

COLUMNS = [
    "t", "alpha_err",
    "q_ref0", "q_ref1", "q_ref2", "q_ref3",
    "q_m0", "q_m1", "q_m2", "q_m3",
    "phi_ref", "theta_ref", "psi_ref",
    "phi", "theta", "psi",
    "p", "q", "r",
    "tau_x", "tau_y", "tau_z",
    "sat_x", "sat_y", "sat_z",
    "thrust", "desat",
    "x", "y", "z",
]

CONTROL_RATE_HINT = 50.0  # assumed log rate [Hz] for the settle-time smoother


@dataclass
class RunLog:
    """Accumulates one adapter run; write() flushes to CSV."""
    simulator: str
    scenario: str
    noise: float = 0.1  # paper sensor noise amplitude used in the run
    rows: list = field(default_factory=list)
    started: float = field(default_factory=time.time)

    def add(
        self,
        t: float,
        q_ref: FloatArr,
        q_m: FloatArr,
        omega: FloatArr,
        tau: FloatArr,
        sat: FloatArr,
        thrust: float,
        desat: float,
        position: FloatArr,
        alpha_err: float,
    ) -> None:
        phi_r, theta_r, psi_r = quat.to_euler(np.asarray(q_ref))
        phi, theta, psi = quat.to_euler(np.asarray(q_m))
        self.rows.append(
            [
                t, alpha_err,
                *q_ref, *q_m,
                phi_r, theta_r, psi_r,
                phi, theta, psi,
                *omega,
                *tau,
                *sat,
                thrust, desat,
                *position,
            ]
        )

    def write(self, path: Path) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([f"# simulator={self.simulator}", f"# scenario={self.scenario}"])
            w.writerow(COLUMNS)
            w.writerows(self.rows)
        return path


def summarize(log: RunLog) -> dict:
    """Cross-simulator metrics, computed identically for every stack."""
    data = np.array(log.rows, dtype=np.float64)
    return summarize_data(data, log.simulator, log.scenario, log.noise)


def summarize_data(data: np.ndarray, simulator: str, scenario: str, noise: float = 0.1) -> dict:
    """Metric core, shared by live runs (RunLog) and CSV replays (sim.compare).

    - rms_alpha_deg: RMS geodesic attitude error over the whole run
    - settle_{phi,theta,psi}_s: settling time of each Euler axis after its
      reference's last discontinuity/ramp onset (noise-aware band, 0.5 s
      smoothed); None if the axis never settles
    - sat_fraction: fraction of ticks with any torque axis saturated
    - final_pos_err_m: |position drift| at run end (attitude demos drift by
      design -- the paper controls attitude only -- but it is reported)
    """
    t = data[:, 0]
    alpha = data[:, 1]
    # COLUMNS layout (0-based): 0 t, 1 alpha_err, 2-5 q_ref, 6-9 q_m,
    # 10-12 euler_ref, 13-15 euler_meas, 16-18 omega, 19-21 tau,
    # 22-24 sat flags, 25 thrust, 26 desat, 27-29 position
    out: dict = {
        "simulator": simulator,
        "scenario": scenario,
        "duration_s": float(t[-1]),
        "rms_alpha_deg": float(np.degrees(np.sqrt(np.mean(alpha**2)))),
        "max_alpha_deg": float(np.degrees(np.max(alpha))),
        "sat_fraction": float(np.mean(np.any(data[:, 22:25] > 0.5, axis=1))),
        "final_pos_err_m": float(np.linalg.norm(data[-1, 27:30])),
    }

    # Settling per Euler axis: find the last time the tracking error leaves a
    # 5%-of-scale band, measured from the reference's last change onset.
    onsets = {"phi": 1.0, "theta": 5.0, "psi": 9.0}  # step staggering (refs.py)
    if scenario == "flip":
        onsets = {"phi": 0.0, "theta": 0.0, "psi": 0.0}
    for i, axis in enumerate(("phi", "theta", "psi")):
        ref = data[:, 10 + i]
        meas = data[:, 13 + i]
        err = np.abs(ref - meas)
        # wrap-aware error on [-pi, pi]
        err = np.minimum(err, 2 * np.pi - err)
        # 0.5 s boxcar smoothing so isolated sensor-noise spikes don't count
        # as unsettled; sustained tracking error still does
        w = max(1, int(0.5 * CONTROL_RATE_HINT))
        kernel = np.ones(w) / w
        err_s = np.convolve(err, kernel, mode="full")[: len(err)]
        amp = max(np.max(np.abs(ref)), 1e-6)
        # noise-aware band: quaternion-component noise of amplitude `noise`
        # maps to a geodesic-angle floor of roughly 2*noise, so demanding a
        # 5% band inside that floor would report "never settles" under the
        # paper's own noise model
        band = max(0.05 * amp, 2.0 * noise)
        onset = onsets[axis]
        mask = t >= onset
        t_a, e_a = t[mask], err_s[mask]
        outside = e_a > band
        if np.any(outside):
            settle = t_a[np.where(outside)[0][-1]] - onset
            out[f"settle_{axis}_s"] = float(max(settle, 0.0))
        else:
            out[f"settle_{axis}_s"] = 0.0
    return out


def results_dir(repo_root: Path, simulator: str) -> Path:
    return repo_root / "results" / f"sim_{simulator}"
