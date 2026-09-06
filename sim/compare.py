"""Cross-simulator comparison table (final-evaluation deliverable, FRP.md
section 3): replays every results/sim_*/  CSV, recomputes the uniform metrics,
and writes results/comparison.md.

    python -m sim.compare            # scan results/, write + print table
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from sim.common.telemetry import COLUMNS, summarize_data

METRIC_KEYS = [
    "rms_alpha_deg", "max_alpha_deg",
    "settle_phi_s", "settle_theta_s", "settle_psi_s",
    "sat_fraction", "final_pos_err_m", "duration_s",
]
HEADERS = {
    "rms_alpha_deg": "RMS att. err [deg]",
    "max_alpha_deg": "max att. err [deg]",
    "settle_phi_s": "settle phi [s]",
    "settle_theta_s": "settle theta [s]",
    "settle_psi_s": "settle psi [s]",
    "sat_fraction": "torque sat. [-]",
    "final_pos_err_m": "drift [m]",
    "duration_s": "duration [s]",
}


def load_run(path: Path):
    """Read one adapter CSV -> (data, simulator, scenario, noise)."""
    simulator = scenario = ""
    noise = 0.1
    rows = []
    with open(path, newline="") as f:
        for r in csv.reader(f):
            if not r:
                continue
            if r[0].startswith("#"):
                for item in (c.strip() for c in r):
                    if item.startswith("simulator="):
                        simulator = item.split("=", 1)[1]
                    elif item.startswith("scenario="):
                        scenario = item.split("=", 1)[1]
                continue
            if r[0] == "t":  # column header
                continue
            rows.append([float(x) for x in r])
    return np.array(rows, dtype=np.float64), simulator, scenario, noise


def collect(results_root: Path):
    runs = []
    for csv_path in sorted(results_root.glob("sim_*/*.csv")):
        data, simulator, scenario, noise = load_run(csv_path)
        if len(data) == 0:
            continue
        runs.append((csv_path, summarize_data(data, simulator, scenario, noise)))
    return runs


def write_table(runs, out_path: Path) -> str:
    lines = [
        "# Cross-simulator comparison -- paper scenarios on all stacks",
        "",
        "Same frozen controller (Pq=20, Pw=4, +/-4 N*m), same references, same",
        "noise model everywhere; see FRP.md section 3. Lower is better except",
        "duration. settle_* use the noise-aware band (max(5%, 2*noise)) on the",
        "0.5 s-smoothed wrap-aware Euler error.",
        "",
        "| simulator | scenario | " + " | ".join(HEADERS[k] for k in METRIC_KEYS) + " |",
        "|---|---|" + "---|" * len(METRIC_KEYS),
    ]
    for path, m in runs:
        vals = []
        for k in METRIC_KEYS:
            v = m.get(k)
            vals.append("--" if v is None else f"{v:.3g}")
        lines.append(f"| {m['simulator']} | {m['scenario']} | " + " | ".join(vals) + " |")
    table = "\n".join(lines)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(table + "\n", encoding="utf-8")
    return table


if __name__ == "__main__":
    results_root = Path.cwd() / "results"
    runs = collect(results_root)
    if not runs:
        raise SystemExit(f"no runs found under {results_root} -- run adapters first")
    table = write_table(runs, results_root / "comparison.md")
    print(table)
    print(f"\nwrote {results_root / 'comparison.md'}")
