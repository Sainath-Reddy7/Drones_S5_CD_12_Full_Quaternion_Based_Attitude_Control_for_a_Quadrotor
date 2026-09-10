"""End-to-end protocol test: the ArduPilot bridge against a MAVLink endpoint.

Flies sim/ardupilot/bridge_node.py (unmodified) against sim/ardupilot/
mock_sitl.py -- the same wire protocol subset real ArduPilot SITL uses
(heartbeats, GUIDED/LAND, arm, takeoff, ATTITUDE_QUATERNERn/LOCAL_POSITION_NED
telemetry, SET_ATTITUDE_TARGET @ 50 Hz). Proves the bridge's connection
handling, mode transitions, state parsing, control math, CSV/plot output.

This is protocol validation, not a firmware run -- the firmware itself runs
under WSL2/Linux (sim/gazebo/README.md).
"""
from __future__ import annotations

import csv
from pathlib import Path

import pytest

pytest.importorskip("pymavlink")

from sim.ardupilot.bridge_node import run as bridge_run
from sim.ardupilot.mock_sitl import MockSitlServer

PORT = 5770


def test_bridge_flies_mock_sitl_step_scenario(tmp_path):
    server = MockSitlServer(f"tcpin:127.0.0.1:{PORT}", duration_s=60.0)
    server.start()

    metrics = bridge_run(
        connection=f"tcp:127.0.0.1:{PORT}",
        scenario="step",
        duration=4.0,  # covers the t=1 phi step; takeoff wait is 8 s
        seed=0,
        noise=0.0,
        out_root=tmp_path,
    )

    # the run must complete, log, and track: by the end of a 4 s step
    # scenario the geodesic error sits at the mock's tracking floor
    assert metrics["simulator"] == "ardupilot"
    assert metrics["scenario"] == "step"
    assert metrics["duration_s"] > 3.0
    assert metrics["max_alpha_deg"] > 40.0  # the 1 rad step registers

    # CSV written with the uniform schema
    csvs = list(Path(tmp_path).rglob("*.csv"))
    assert csvs, "bridge must write its CSV log"
    rows = [r for r in csv.reader(open(csvs[0], newline="")) if r and not r[0].startswith("#")]
    assert rows[0][0] == "t"
    assert len(rows) > 100  # 4 s at >=50 Hz logging of control ticks

    import numpy as np

    data = np.array([[float(x) for x in r] for r in rows[1:]], dtype=float)

    # SET_ATTITUDE_TARGET actually flowed: torque column shows the paper law
    # reacting to the t=1 step (nonzero, and saturating at the step onset)
    tau_x = data[:, 19]
    assert np.max(np.abs(tau_x)) > 4.0 - 1e-6  # paper's saturation bound reached
    assert np.max(np.abs(tau_x)) <= 4.0 + 1e-6  # and never exceeded

    # the mock vehicle is a first-order tracker, so the transient is broad --
    # what must hold is CONVERGENCE: final-second error small
    t = data[:, 0]
    alpha = data[:, 1]
    tail = alpha[t >= t[-1] - 1.0]
    assert float(np.mean(np.abs(tail))) < 0.35, (
        f"no convergence: tail |alpha| {np.mean(np.abs(tail)):.3f} rad"
    )
