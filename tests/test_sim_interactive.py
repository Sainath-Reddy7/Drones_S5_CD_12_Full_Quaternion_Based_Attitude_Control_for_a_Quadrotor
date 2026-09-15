"""Headless validation of the interactive simulator (no viewer/keyboard).

Covers: model + environment inventory, hover stability on the paper law,
collision detection with named objects, safe mode handoff, path tracking,
paper tests, wind, noise, traffic, safety, and telemetry logging.
"""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco")

from sim.interactive.app import SPAWN, DroneSim


def _drive(sim: DroneSim, seconds: float, keys: set = frozenset()) -> dict:
    n = int(seconds * sim.rate)
    for k in range(n):
        if k % 100 == 0:
            if sim.mode == "MANUAL":
                sim.pilot_input(set(keys))
            else:
                sim.guidance()
        if k % 10 == 0:
            sim.control_tick()
        sim.step()
        sim.check_collisions(k * sim.dt)
    return sim.read_state()


def _geom_names(sim):
    import mujoco

    return {mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            for i in range(sim.model.ngeom)}


def test_model_and_environment_inventory():
    sim = DroneSim()
    m = sim.model
    assert m.body_mass[sim.bid] == 0.2
    assert np.allclose(m.body_inertia[sim.bid], [6.5e-4, 6.5e-4, 1.2e-3])
    names = _geom_names(sim)
    assert len([n for n in names if n and n.startswith("bldg_") and not n.endswith("r")]) >= 5
    assert len([n for n in names if n and n.startswith("car_") and not n.endswith("c")]) >= 3
    assert len([n for n in names if n and n.endswith("trailer")]) >= 3  # 2 static + 1 moving
    assert len([n for n in names if n and n.endswith("_trunk")]) >= 6
    assert len([n for n in names if n and n.startswith("mk_")]) == 121  # path marker pool


def test_manual_hover_is_stable():
    sim = DroneSim()
    s = _drive(sim, 3.0)
    assert abs(s["pos"][2] - SPAWN[2]) < 0.3
    from quat_sitl import quaternion as quat
    alpha = 2 * np.arccos(np.clip(abs(float(np.dot(quat.from_euler(0, 0, 0), s["q"]))), 0, 1))
    assert np.degrees(alpha) < 20


def test_collision_detection_and_logging():
    sim = DroneSim()
    sim.set_drone([9.0, 20.0, 5.0])  # inside bldg_2
    _drive(sim, 0.5)
    assert sim.collisions and any(c["object"].startswith("bldg") for c in sim.collisions)


def test_mode_switch_initializes_from_current_state():
    sim = DroneSim()
    _drive(sim, 1.0)
    sim.refs["z"] = 2.0
    _drive(sim, 1.0)
    at_switch = sim.read_state()
    sim.switch_mode()
    assert sim.mode == "OUR CONTROL"
    assert sim.refs["z"] == pytest.approx(at_switch["pos"][2], abs=0.3)
    s = _drive(sim, 1.5)
    assert abs(s["pos"][2] - at_switch["pos"][2]) < 0.8
    assert np.linalg.norm(s["vel"]) < 2.0


def test_circle_path_tracking_converges():
    sim = DroneSim()
    sim.switch_mode()                      # -> OUR CONTROL
    sim.start_track("circle")
    _drive(sim, 12.0)                      # takeoff + ~2 laps at omega=0.5
    assert sim.mission in ("TRACK",)
    assert sim.xtrack < 2.5, f"cross-track {sim.xtrack:.2f} m after 12 s"
    assert len(sim.track_log) > 50         # tracking telemetry recorded


def test_paper_step_test_runs_live():
    sim = DroneSim()
    sim.switch_mode()
    sim.start_paper_test("step")
    _drive(sim, 5.0)                       # past the t=1 phi step
    assert sim.mission in ("PAPER", "HOLD")
    assert sim.telemetry["err_deg"] < 60   # tracking, not diverged


def test_wind_physically_pushes_drone():
    sim = DroneSim()
    sim.wind = "HIGH"
    s = _drive(sim, 4.0)
    assert abs(s["pos"][0]) > 0.8, f"wind produced only {s['pos'][0]:.2f} m drift"


def test_noise_modes_change_sensor_output():
    sim = DroneSim()
    sim.noise = "PERFECT"
    clean = sim.read_state(noisy=True)["q"]
    assert np.allclose(clean, sim.read_state()["q"], atol=1e-9)
    sim.noise = "HIGH"
    dirty = [sim.read_state(noisy=True)["q"] for _ in range(5)]
    assert any(not np.allclose(d, clean, atol=1e-3) for d in dirty)


def test_traffic_moves():
    sim = DroneSim()
    import mujoco

    bid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "car_m1")
    qadr = sim.model.jnt_dofadr[sim.model.body_jntadr[bid]]
    y0 = sim.data.qpos[qadr + 1]
    _drive(sim, 2.0)
    y1 = sim.data.qpos[qadr + 1]
    assert abs(y1 - y0) > 1.0              # car physically moved along its lane


def test_safety_and_marker_pool():
    from sim.interactive import config as cfg

    sim = DroneSim()
    sim.refs["z"] = 45.0                   # above max altitude
    assert "altitude" in sim._safety_check({"pos": np.array([0, 0, 45.0]),
                                             "vel": np.zeros(3), "rpy": (0, 0, 0)})
    sim.start_track("circle")
    assert sim._mk["shown"] > 50           # path dots placed in the world
