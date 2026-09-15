"""Headless validation of the interactive simulator (no viewer/keyboard).

Covers: both environments, hover stability, collision detection, safe mode
handoff, circle path tracking, live paper tests, wind, noise, traffic,
safety, marker pool, and the INPUT PIPELINE (InputManager -> refs -> motors
-> physics movement) with synthetic key events through the identical code
path a real keypress takes.
"""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco")

from sim.interactive.app import ENV_LIST, SPAWN, DroneSim
from sim.interactive.input_manager import InputManager


def _drive(sim, seconds, keys=frozenset()):
    for k in range(int(seconds * sim.rate)):
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


def _names(sim):
    import mujoco

    return {mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(sim.model.ngeom)}


@pytest.mark.parametrize("env", ENV_LIST)
def test_both_environments_load_with_inventory(env):
    sim = DroneSim(env=env)
    m = sim.model
    assert m.body_mass[sim.bid] == 0.2
    assert np.allclose(m.body_inertia[sim.bid], [6.5e-4, 6.5e-4, 1.2e-3])
    names = _names(sim)
    assert sum(1 for n in names if n and n.startswith("mk_")) == 169
    wheels = sum(1 for n in names if n and "_w" in n and ("car_" in n or "truck_" in n))
    assert wheels >= 24, f"wheels={wheels}"          # real wheeled vehicles
    assert any(n and n.endswith("_cabin") for n in names)
    assert any(n and n.endswith("_glass") for n in names)
    if env == "urban_v1":
        assert sum(1 for n in names if n and n.startswith("bldg_") and not n.endswith(("r", "win", "roof", "ant"))) >= 5


def test_manual_hover_stable():
    sim = DroneSim()
    s = _drive(sim, 3.0)
    assert abs(s["pos"][2] - SPAWN[2]) < 0.3
    from quat_sitl import quaternion as quat
    a = 2 * np.arccos(np.clip(abs(float(np.dot(quat.from_euler(0, 0, 0), s["q"]))), 0, 1))
    assert np.degrees(a) < 20


def test_collision_detection():
    sim = DroneSim()
    sim.set_drone([9.0, 20.0, 5.0])  # inside bldg_2 (urban_v1)
    _drive(sim, 0.5)
    assert any(c["object"].startswith("bldg") for c in sim.collisions)


def test_mode_switch_safe():
    sim = DroneSim()
    _drive(sim, 1.0)
    sim.refs["z"] = 2.0
    _drive(sim, 1.0)
    at = sim.read_state()
    sim.switch_mode()
    assert sim.mode == "OUR CONTROL"
    assert sim.refs["z"] == pytest.approx(at["pos"][2], abs=0.3)
    s = _drive(sim, 1.5)
    assert abs(s["pos"][2] - at["pos"][2]) < 0.8 and np.linalg.norm(s["vel"]) < 2.0


def test_circle_tracking_converges():
    sim = DroneSim()
    sim.switch_mode()
    sim.start_track("circle")
    _drive(sim, 12.0)
    assert sim.mission == "TRACK"
    assert sim.xtrack < 2.5, f"xtrack {sim.xtrack:.2f}"


def test_paper_step_test_live():
    sim = DroneSim()
    sim.switch_mode()
    sim.start_paper_test("step")
    _drive(sim, 5.0)
    assert sim.mission in ("PAPER", "HOLD")
    assert sim.telemetry["err_deg"] < 60


def test_wind_and_noise_levels():
    sim = DroneSim()
    sim.lock_position = False  # wind needs translation to show drift
    sim.wind = "HIGH"
    s = _drive(sim, 4.0)
    assert abs(s["pos"][0]) > 0.8
    sim2 = DroneSim()
    sim2.noise = "PERFECT"
    clean = sim2.read_state(noisy=True)["q"]
    assert np.allclose(clean, sim2.read_state()["q"], atol=1e-9)
    sim2.noise = "HIGH"
    assert any(not np.allclose(sim2.read_state(noisy=True)["q"], clean, atol=1e-3) for _ in range(5))


def test_traffic_moves():
    import mujoco

    sim = DroneSim()
    b = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "car_m1")
    a = sim.model.jnt_dofadr[sim.model.body_jntadr[b]]
    y0 = sim.data.qpos[a + 1]
    _drive(sim, 2.0)
    assert abs(sim.data.qpos[a + 1] - y0) > 1.0


def test_environment_switch_isolation():
    # both env files exist and switching never modifies them
    from pathlib import Path

    base = Path("sim/interactive/envs")
    h1 = (base / "urban_v1" / "world.xml").read_bytes()
    h2 = (base / "open_field_v1" / "world.xml").read_bytes()
    DroneSim(env="urban_v1")
    DroneSim(env="open_field_v1")
    assert (base / "urban_v1" / "world.xml").read_bytes() == h1
    assert (base / "open_field_v1" / "world.xml").read_bytes() == h2


# ---------------- INPUT PIPELINE: synthetic keys -> refs -> motors -> motion ----

def test_input_manager_events():
    inp = InputManager(use_hook=False)
    inp.inject("w", "down")
    assert inp.is_pressed("w")
    held, taps = inp.poll()
    assert "w" in held and "w" in taps        # held state + one-shot tap
    inp.inject("w", "up")
    assert not inp.is_pressed("w")


def test_pipeline_key_to_motor_commands():
    """W pressed through InputManager -> pilot_input -> control_tick changes
    the per-rotor thrusts (the controller is really reacting)."""
    inp = InputManager(use_hook=False)
    sim = DroneSim()
    sim.noise = "PERFECT"
    for k in range(int(1.0 * sim.rate)):  # settle hover first
        if k % 10 == 0:
            sim.control_tick()
        sim.step()
    base = sim.motors.copy()
    inp.inject("w", "down")
    for k in range(3000):  # 0.3 s with W held
        if k % 100 == 0:
            held, _ = inp.poll()
            sim.pilot_input(held)
        if k % 10 == 0:
            sim.control_tick()
        sim.step()
    assert sim.refs["pitch"] > 0.05          # desired attitude changed
    assert not np.allclose(sim.motors, base, atol=1e-3)  # motors changed
    inp.inject("w", "up")
    for k in range(5000):  # release: command decays to neutral
        if k % 100 == 0:
            held, _ = inp.poll()
            sim.pilot_input(held)
        if k % 10 == 0:
            sim.control_tick()
        sim.step()
    assert abs(sim.refs["pitch"]) < 0.02


def test_pipeline_key_to_physics_movement():
    """Holding W moves the drone forward (+x body) through real physics
    (position unlocked — tests the full 6-DOF pipeline)."""
    sim = DroneSim()
    sim.noise = "PERFECT"
    sim.lock_position = False
    x0 = sim.read_state()["pos"][0]
    for k in range(int(2.5 * sim.rate)):
        keys = {"w"} if k < int(2.0 * sim.rate) else frozenset()
        if k % 100 == 0:
            sim.pilot_input(keys)
        if k % 10 == 0:
            sim.control_tick()
        sim.step()
    x1 = sim.read_state()["pos"][0]
    assert x1 - x0 > 2.0, f"W produced only {x1 - x0:.2f} m of travel"


def test_pipeline_yaw_and_altitude():
    sim = DroneSim()
    sim.noise = "PERFECT"
    sim.lock_position = False
    z0 = sim.read_state()["pos"][2]
    for k in range(int(3.0 * sim.rate)):
        keys = {"q", "r"}  # yaw left + climb
        if k % 100 == 0:
            sim.pilot_input(keys)
        if k % 10 == 0:
            sim.control_tick()
        sim.step()
    s = sim.read_state()
    assert s["pos"][2] - z0 > 1.5, "R did not climb"
    assert abs(s["rpy_sim"][2]) > 0.5, "Q did not yaw"


def test_position_locked_attitude_only():
    """THE PAPER'S PLANT: attitude-only, zero translation.

    In manual + paper-test mode the drone is pinned to its hover point —
    it can rotate freely (that's the paper's contribution) but must not
    translate even a millimeter. This test drives aggressive attitude
    commands and verifies position stays EXACTLY at the spawn point.
    """
    sim = DroneSim()
    assert sim.lock_position is True, "lock should be the default"
    spawn = sim.read_state()["pos"].copy()

    # aggressive attitude: full forward + yaw for 3 seconds
    for k in range(int(3.0 * sim.rate)):
        keys = {"w", "q"} if k % 100 == 0 else None
        if keys:
            sim.pilot_input(keys)
        if k % 10 == 0:
            sim.control_tick()
        sim.step()

    s = sim.read_state()
    assert np.allclose(s["pos"], spawn, atol=1e-10), \
        f"position drifted: {np.linalg.norm(s['pos'] - spawn):.2e} m"
    assert abs(s["rpy_sim"][2]) > 0.3, "drone should have yawed (rotation works)"

    # also verify paper-test mode stays locked
    sim.reset()
    sim.switch_mode()
    sim.start_paper_test("step")
    for k in range(int(2.0 * sim.rate)):
        if k % 100 == 0:
            sim.guidance()
        if k % 10 == 0:
            sim.control_tick()
        sim.step()
    s2 = sim.read_state()
    assert np.allclose(s2["pos"], SPAWN, atol=1e-10), "paper test drifted"

    # and path tracking UNLOCKS (needs translation)
    sim.reset()
    sim.switch_mode()
    sim.start_track("circle")
    assert sim.lock_position is False, "path tracking should unlock position"
    for k in range(int(3.0 * sim.rate)):
        if k % 100 == 0:
            sim.guidance()
        if k % 10 == 0:
            sim.control_tick()
        sim.step()
    s3 = sim.read_state()
    assert s3["pos"][2] > SPAWN[2] + 0.5, "path tracking should climb (free movement)"
