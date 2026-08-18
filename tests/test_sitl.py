"""Tests for pysitl -- the PX4-style 6-DOF SITL wrapping the paper's
attitude controller. Separate from tests/test_quaternion.py, whose 7-test
count is a prior contract for the quat_sitl paper reproduction; this file has
no such constraint.

Covers exactly the risks the plan called out before any of this was built:
frame convention, hover equilibrium, mixer correctness (including the
thrust-preserving-desaturation regression), eq.18 sign parity with
quat_sitl.dynamics, attitude tracking under the paper's own noise, scheduler
determinism, and ground contact.
"""
from __future__ import annotations

import math
import random

import numpy as np
import pytest

from pysitl import airframe, plant, topics
from pysitl.bus import MessageBus
from pysitl.control.attitude import compute_torque_scalar
from pysitl.control.attitude import NonlinearP2Controller
from pysitl.modes import Commander, FlightMode
from pysitl.params import GainParams, SimParams, VehicleParams
from pysitl.plant import Plant
from pysitl.scheduler import Scheduler
from pysitl.sensors import SensorModel
from pysitl.sim import Simulation

from quat_sitl import dynamics as qd
from quat_sitl import quaternion as quat


# --------------------------------------------------------------------------
# Vehicle derivation
# --------------------------------------------------------------------------


def test_vehicle_inertia_matches_paper():
    v = VehicleParams()
    assert v.verify_inertia_match()
    assert v.inertia.Ixx == pytest.approx(6.5e-4)
    assert v.inertia.Izz == pytest.approx(1.2e-3)


# --------------------------------------------------------------------------
# Frame convention -- the highest-risk item in the plan
# --------------------------------------------------------------------------


def test_body_to_world_convention():
    """Spin at a known body rate about +z, then check where the body x-axis
    lands in the world frame. If body->world were backwards, this would
    land at -sin instead of +sin -- a mirrored vehicle."""
    v = VehicleParams()
    p = Plant(vehicle=v)
    p.state = (0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1.0)
    dt = 1e-4
    for _ in range(5000):  # 0.5 s
        p.step(0.0, (0.0, 0.0, 0.0), dt)

    bx_world = plant.rotate_body_to_world(p.quaternion, (1.0, 0.0, 0.0))
    expected = (math.cos(0.5), math.sin(0.5), 0.0)
    assert bx_world == pytest.approx(expected, abs=1e-3)


def test_rotate_body_to_world_matches_quat_sitl_dcm_transpose():
    """rotate_body_to_world(q, v) must equal to_dcm(q).T @ v -- the exact
    relationship the plan derived from quat_sitl's own eq.18 convention."""
    rng = np.random.default_rng(0)
    q = rng.normal(size=4)
    q = q / np.linalg.norm(q)
    v = rng.normal(size=3)
    expected = quat.to_dcm(q).T @ v
    got = plant.rotate_body_to_world(tuple(q), tuple(v))
    assert np.allclose(got, expected, atol=1e-10)


# --------------------------------------------------------------------------
# eq.18 sign parity vs the paper reproduction
# --------------------------------------------------------------------------


def test_eq18_parity_with_quat_sitl_dynamics():
    """pysitl's scalar rotational dynamics must match quat_sitl.dynamics'
    numpy eq.18 implementation term-for-term (zero thrust so only rotation
    is compared)."""
    inertia = qd.InertiaParams()
    v = VehicleParams(inertia=inertia)
    state13 = (0.0, 0.0, 0.0, 0.9689, 0.0, 0.0, -0.2474, 0.0, 0.0, 0.0, 0.3, -0.2, 0.5)
    tau = (0.01, -0.02, 0.03)

    d13 = plant.state_derivative(state13, 0.0, tau, v)
    qdot_scalar = d13[3:7]
    wdot_scalar = d13[10:13]

    state7 = np.array([state13[3], state13[4], state13[5], state13[6], state13[10], state13[11], state13[12]])
    d7 = qd.state_derivative(state7, np.array(tau), inertia)

    assert qdot_scalar == pytest.approx(tuple(d7[0:4]), abs=1e-12)
    assert wdot_scalar == pytest.approx(tuple(d7[4:7]), abs=1e-12)


# --------------------------------------------------------------------------
# Plant: hover equilibrium, ground contact
# --------------------------------------------------------------------------


def test_hover_equilibrium_holds_altitude():
    """Thrust = weight, level attitude -> zero net vertical acceleration."""
    v = VehicleParams()
    p = Plant(vehicle=v)
    thrust = v.hover_thrust_total
    for _ in range(20000):  # 2 s at dt=1e-4
        p.step(thrust, (0.0, 0.0, 0.0), 1e-4)
    assert p.altitude == pytest.approx(0.0, abs=1e-6)
    assert p.velocity[2] == pytest.approx(0.0, abs=1e-6)


def test_ground_contact_prevents_falling_through_floor():
    v = VehicleParams()
    p = Plant(vehicle=v)  # starts on the ground, zero thrust -> should not sink below z=0
    for _ in range(10000):  # 1 s
        p.step(0.0, (0.0, 0.0, 0.0), 1e-4)
    assert p.altitude == pytest.approx(0.0, abs=1e-9)
    assert p.state[2] <= 1e-9  # NED z (down) never goes positive (below ground)


def test_zero_torque_zero_rate_state_unchanged():
    v = VehicleParams()
    p = Plant(vehicle=v)
    p.step(0.0, (0.0, 0.0, 0.0), 1e-4)
    assert p.state == pytest.approx(plant.IDENTITY_STATE, abs=1e-12)


# --------------------------------------------------------------------------
# Mixer: forward/inverse round trip + the desaturation regression
# --------------------------------------------------------------------------


def test_mixer_round_trip_no_saturation():
    v = VehicleParams()
    thrust_cmd = v.hover_thrust_total
    torque_cmd = (0.05, -0.03, 0.02)
    thrusts, scale = airframe.mix(thrust_cmd, torque_cmd, v)
    assert scale == pytest.approx(1.0)
    Fz, tx, ty, tz = airframe.thrusts_to_wrench(thrusts, v)
    assert -Fz == pytest.approx(thrust_cmd, rel=1e-9)
    assert (tx, ty, tz) == pytest.approx(torque_cmd, abs=1e-9)


def test_desaturation_preserves_total_thrust_under_saturation():
    """The regression this plan was built around: naive per-rotor clipping
    changes total thrust when torque saturates (measured altitude runaway to
    154 m in the prototype). Thrust-preserving desaturation must not."""
    v = VehicleParams()
    thrust_cmd = v.hover_thrust_total
    huge_torque = (10.0, 0.0, 0.0)  # deliberately unachievable
    thrusts, scale = airframe.mix(thrust_cmd, huge_torque, v)
    assert scale < 1.0  # confirms saturation actually occurred
    assert sum(thrusts) == pytest.approx(thrust_cmd, rel=1e-9)
    for t in thrusts:
        assert -1e-9 <= t <= v.rotor_thrust_max + 1e-9


def test_desaturation_respects_rotor_bounds_across_random_commands():
    v = VehicleParams()
    rng = random.Random(0)
    for _ in range(200):
        thrust_cmd = rng.uniform(0, 4 * v.rotor_thrust_max)
        torque_cmd = tuple(rng.uniform(-2, 2) for _ in range(3))
        thrusts, _ = airframe.mix(thrust_cmd, torque_cmd, v)
        for t in thrusts:
            assert -1e-9 <= t <= v.rotor_thrust_max + 1e-9


# --------------------------------------------------------------------------
# Sensors
# --------------------------------------------------------------------------


def test_sensor_noise_bounded_and_renormalized():
    model = SensorModel(noise_amplitude=0.1, rng=random.Random(0))
    q_true = (1.0, 0.0, 0.0, 0.0)
    w_true = (0.1, -0.2, 0.3)
    for _ in range(200):
        q_meas, w_meas = model.measure(q_true, w_true)
        assert abs(sum(c * c for c in q_meas) - 1.0) < 1e-9
        for wm, wt in zip(w_meas, w_true):
            assert abs(wm - wt) <= 0.1 + 1e-9


# --------------------------------------------------------------------------
# Closed-loop: attitude tracking under the paper's own noise
# --------------------------------------------------------------------------


def test_closed_loop_tracks_commanded_roll_with_paper_noise():
    """End-to-end through the real unmodified controller + real mixer +
    real 6-DOF plant: a 0.3 rad roll command should be tracked closely even
    with the paper's full 0.1 measurement noise active."""
    v = VehicleParams()
    g = GainParams()
    p = Plant(vehicle=v)
    af = airframe.Airframe(vehicle=v)
    ctrl = NonlinearP2Controller()
    model = SensorModel(noise_amplitude=0.1, rng=random.Random(0))

    q_ref = tuple(quat.from_euler(0.3, 0.0, 0.0))
    dt = 1.0 / 16000

    for _ in range(int(3.0 / dt)):
        q_meas, w_meas = model.measure(p.quaternion, p.omega)
        tau = compute_torque_scalar(ctrl, q_ref, q_meas, w_meas, g.torque_limit)
        rotor_cmd, _ = airframe.mix(v.hover_thrust_total, tau, v)
        rotor_actual = af.actuate(rotor_cmd, dt)
        Fz, tx, ty, tz = airframe.thrusts_to_wrench(rotor_actual, v)
        p.step(-Fz, (tx, ty, tz), dt)

    from quat_sitl.quaternion import to_euler

    roll = to_euler(np.array(p.quaternion))[0]
    assert roll == pytest.approx(0.3, abs=0.03)


# --------------------------------------------------------------------------
# Scheduler: rate decimation + determinism
# --------------------------------------------------------------------------


def test_scheduler_rejects_unstable_base_rate():
    """base_hz below dynamics.stable_control_rate_hz must be refused, not
    silently accepted into a chattering loop (plan finding 2)."""
    from quat_sitl.dynamics import InertiaParams

    with pytest.raises(ValueError):
        Scheduler(base_hz=1000.0, Pw=4.0, inertia=InertiaParams())


def test_scheduler_decimation_rates():
    calls = {"fast": 0, "slow": 0}
    sched = Scheduler(base_hz=16000.0, Pw=4.0, inertia=VehicleParams().inertia)
    sched.register("fast", 16000.0, lambda t, dt: calls.__setitem__("fast", calls["fast"] + 1))
    sched.register("slow", 1000.0, lambda t, dt: calls.__setitem__("slow", calls["slow"] + 1))
    for _ in range(16000):  # exactly 1 simulated second
        sched.tick()
    assert calls["fast"] == 16000
    assert calls["slow"] == 1000


def test_simulation_is_deterministic_given_seed():
    """Two identical headless runs with the same seed must land on
    (numerically) identical state -- no hidden wall-clock or unseeded
    randomness in the hot path."""

    def run():
        sim = Simulation(sim_params=SimParams(seed=42))
        sim.commander.try_arm(sim.plant.altitude)
        sim.set_flight_mode(FlightMode.AUTO_STEP)
        sim.run_for(2.0)
        return sim.plant.state

    s1 = run()
    s2 = run()
    assert s1 == pytest.approx(s2, abs=1e-12)


# --------------------------------------------------------------------------
# Commander: arming, mode switching, failsafe
# --------------------------------------------------------------------------


def test_commander_arm_requires_on_ground():
    c = Commander()
    assert c.try_arm(altitude=0.0) is True
    assert c.armed is True

    c2 = Commander()
    assert c2.try_arm(altitude=5.0) is False
    assert c2.armed is False


def test_commander_failsafe_disarms_on_altitude_breach():
    c = Commander()
    c.armed = True
    c.check_failsafe(altitude=60.0)
    assert c.armed is False
    assert c.failsafe_reason is not None


def test_disarmed_vehicle_gets_zero_thrust():
    """sim.py's control task must zero thrust/torque when disarmed,
    regardless of mode or stick input -- the actual safety property, not
    just the commander flag."""
    sim = Simulation()
    sim.commander.armed = False
    sim.set_flight_mode(FlightMode.ALTITUDE)
    sim.run_for(0.5)
    outputs = sim.bus.get_value(topics.TOPIC_ACTUATOR_OUTPUTS)
    assert outputs is not None
    assert all(t == pytest.approx(0.0, abs=1e-6) for t in outputs.rotor_thrust)


# --------------------------------------------------------------------------
# Bus
# --------------------------------------------------------------------------


def test_bus_latest_value_and_sequence():
    bus = MessageBus()
    assert bus.get("x") is None
    bus.publish("x", 1, timestamp=0.0)
    bus.publish("x", 2, timestamp=0.1)
    s = bus.get("x")
    assert s.value == 2
    assert s.seq == 1
    assert bus.get_value("missing", default="fallback") == "fallback"


# --------------------------------------------------------------------------
# AUTO scenarios: flip actually completes the full rotation
# --------------------------------------------------------------------------


def test_auto_flip_completes_full_rotation_without_shortest_path():
    sim = Simulation()
    sim.commander.try_arm(sim.plant.altitude)
    sim.set_flight_mode(FlightMode.AUTO_FLIP)
    assert sim.autopilot_shortest_path is False
    sim.run_for(5.0)
    q = sim.plant.quaternion
    # a full 2*pi rotation about x lands back on identity's double cover, [-1,0,0,0]
    assert q[0] == pytest.approx(-1.0, abs=0.05)
    assert q[1] == pytest.approx(0.0, abs=0.05)
