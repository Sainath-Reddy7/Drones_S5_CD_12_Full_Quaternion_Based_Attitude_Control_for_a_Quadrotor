#!/usr/bin/env python3
"""Build a self-contained 3D attitude-simulation HTML viewer.

Runs the three paper scenarios (step / sine / flip) through the SITL simulator,
decimates the logged trajectories to ~60 fps, and injects them into an HTML
template containing a pure-Canvas 3D quadrotor renderer + tracking plots.

Output: quadrotor_sim.html  (open in any browser; no server or dependencies)

Usage:  python3 build_ui.py [--fps 60] [--duration 15]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE / "extracted" / "CD_12_Full-Quaternion-Based-Attitude-Control-for-a-Quadrotor-main"
sys.path.insert(0, str(PKG))

from quat_sitl import controller, dynamics, references, simulator  # noqa: E402


def run_scenario(name, duration, ref_fn, ctrl, fps):
    print(f"  running '{name}' ({duration}s) ...", end=" ", flush=True)
    t0 = time.time()
    df = simulator.run_simulation(
        scenario=name,
        duration=duration,
        seed=0,
        noise_amplitude=0.1,
        reference_fn=ref_fn,
        controller=ctrl,
        inertia=dynamics.InertiaParams(),
        motor_model=dynamics.IdentityMotorModel(),
    )
    dt_sim = time.time() - t0

    # Decimate to target fps (sim logs at 1 kHz)
    n_frames = max(2, int(duration * fps))
    idx = np.linspace(0, len(df) - 1, n_frames).astype(int)
    d = df.iloc[idx]

    out = {
        "duration": round(float(duration), 2),
        "times": np.round(d["time"].values, 4).tolist(),
        "q": np.round(d[["q_true_0", "q_true_1", "q_true_2", "q_true_3"]].values, 6).tolist(),
        "euler": np.round(d[["phi", "theta", "psi"]].values, 4).tolist(),
        "eref": np.round(d[["phi_ref", "theta_ref", "psi_ref"]].values, 4).tolist(),
        "tau": np.round(d[["tau_x", "tau_y", "tau_z"]].values, 4).tolist(),
    }
    print(f"{dt_sim:.1f}s  ({len(d)} frames)")
    return out


def main():
    ap = argparse.ArgumentParser(description="Build quadrotor 3D simulation HTML")
    ap.add_argument("--fps", type=int, default=60, help="animation data fps (default 60)")
    ap.add_argument("--duration", type=float, default=15.0, help="step/sine duration (default 15)")
    ap.add_argument("--flip-duration", type=float, default=5.0, help="flip duration (default 5)")
    args = ap.parse_args()

    tmpl_path = HERE / "viewer_template.html"
    out_path = HERE / "quadrotor_sim.html"

    print("Running SITL scenarios (eq. 17-21, Pq=20, Pw=4) ...")
    data = {
        "step": run_scenario(
            "step", args.duration, references.step_reference,
            controller.NonlinearP2Controller(), args.fps),
        "sine": run_scenario(
            "sine", args.duration, references.sine_reference,
            controller.NonlinearP2Controller(), args.fps),
        "flip": run_scenario(
            "flip", args.flip_duration, references.flip_reference,
            controller.NonlinearP2Controller(shortest_path=False), args.fps),
    }

    json_str = json.dumps(data, separators=(",", ":"))
    print(f"JSON payload: {len(json_str) / 1024:.0f} KB")

    template = tmpl_path.read_text(encoding="utf-8")
    marker = "__SIM_DATA__"
    if marker not in template:
        sys.exit(f"ERROR: marker {marker!r} not found in template")
    html = template.replace(marker, json_str)

    out_path.write_text(html, encoding="utf-8")
    size_kb = os.path.getsize(out_path) / 1024
    print(f"\nWrote {out_path}  ({size_kb:.0f} KB)")
    print(f"Open: file://{out_path}")


if __name__ == "__main__":
    main()
