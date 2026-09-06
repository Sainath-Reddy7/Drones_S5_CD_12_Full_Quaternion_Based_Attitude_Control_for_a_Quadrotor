"""Guardrails for the sim/common bridge.

The decisive test is test_conjugate_convention_drives_standard_plant: it
integrates a STANDARD Hamilton body->world quaternion plant (qdot = +1/2 q (x)
[0,w] -- what PyBullet/MuJoCo/Gazebo/ArduPilot actually integrate) driven
through the bridge's q_m = conj(q_sim) mapping. If the paper controller
stabilizes that plant, the adapter convention is right for all four stacks.
"""
from __future__ import annotations

import numpy as np
import pytest

from quat_sitl import quaternion as quat
from quat_sitl.dynamics import InertiaParams, stable_control_rate_hz
from sim.common import (
    AltitudeHold,
    PAPER_VEHICLE,
    QuatBridge,
    RotorGeometry,
    mix_zup,
    reference_quat,
    thrusts_to_wrench_zup,
)


def test_mixer_round_trip_unsaturated():
    rng = np.random.default_rng(0)
    for _ in range(50):
        thrust = float(rng.uniform(0.5, 1.5))
        # within combined-axis authority at the LOWEST thrust draw: per-rotor
        # delta budget is thrust/4, consumed as (|tx|/d + |ty|/d + |tz|/c)/4,
        # i.e. |tx|+|ty| <= d*thrust and |tz| <= c*thrust with headroom
        tau = rng.uniform(-0.01, 0.01, 2).tolist() + [float(rng.uniform(-0.002, 0.002))]
        thrusts, scale = mix_zup(thrust, np.array(tau), PAPER_VEHICLE)
        assert scale == 1.0
        assert all(t >= 0.0 for t in thrusts)
        fz, tau_back = thrusts_to_wrench_zup(thrusts, PAPER_VEHICLE)
        assert fz == pytest.approx(thrust, abs=1e-12)
        assert tau_back == pytest.approx(tau, abs=1e-9)


def test_mixer_desaturation_preserves_thrust():
    thrust = 1.0
    tau = np.array([3.0, 0.0, 0.0])  # big roll demand
    thrusts, scale = mix_zup(thrust, tau, PAPER_VEHICLE)
    assert scale < 1.0
    assert sum(thrusts) == pytest.approx(thrust, abs=1e-12)  # common mode held
    assert all(0.0 <= t <= PAPER_VEHICLE.rotor_thrust_max for t in thrusts)


def test_mixer_roll_sign():
    """Positive tau_x must come mostly from the +y-arm rotors (1 at (d,d),
    4 at (-d,d)) -- the z-up forward mix's own sign convention."""
    thrusts, _ = mix_zup(1.0, np.array([0.5, 0.0, 0.0]), PAPER_VEHICLE)
    t1, t2, t3, t4 = thrusts
    assert t1 > t2 and t4 > t3


def test_conjugate_convention_drives_standard_plant():
    """THE adapter-convention proof: paper controller + q_m = conj(q_sim) on a
    standard body->world Hamilton plant converges to the reference."""
    bridge = QuatBridge(shortest_path=True)
    inertia = InertiaParams()
    q_sim = np.array([1.0, 0.0, 0.0, 0.0])  # scalar-first body->world
    omega = np.zeros(3)
    q_target = quat.from_euler(0.4, -0.3, 0.7)

    dt = 1.0 / stable_control_rate_hz(4.0, inertia)  # paper Pw, ~12.3 kHz
    for _ in range(int(8.0 / dt)):
        q_m = quat.conj(q_sim)
        tau, _ = bridge.torque(q_target, q_m, omega)
        tau = np.clip(tau, -4.0, 4.0)
        # standard plant: qdot = +1/2 q (x) [0,w]; Euler equations on omega
        zero_w = np.concatenate([[0.0], omega])
        q_sim = quat.normalize(q_sim + dt * (0.5 * quat.mul(q_sim, zero_w)))
        I = inertia.as_matrix()
        omega = omega + dt * (np.linalg.solve(I, tau - np.cross(omega, I @ omega)))

    q_m = quat.conj(q_sim)
    assert quat.norm(quat.mul(q_target, quat.conj(q_m))) == pytest.approx(1.0, abs=1e-3)
    phi, theta, psi = quat.to_euler(q_m)
    assert phi == pytest.approx(0.4, abs=1e-2)
    assert theta == pytest.approx(-0.3, abs=1e-2)
    assert psi == pytest.approx(0.7, abs=1e-2)


def test_altitude_hold_bounded_and_positive():
    alt = AltitudeHold()
    thrust = alt.thrust(
        z_ref=1.0, z=1.0, vz=0.0, tilt_cos=1.0,
        mass=0.2, thrust_max_total=PAPER_VEHICLE.thrust_max_total, dt=1e-3,
    )
    assert thrust == pytest.approx(0.2 * 9.81, rel=1e-6)  # exactly hover
    falling = alt.thrust(
        z_ref=1.0, z=0.5, vz=0.0, tilt_cos=1.0,
        mass=0.2, thrust_max_total=PAPER_VEHICLE.thrust_max_total, dt=1e-3,
    )
    assert falling > thrust  # below reference -> more than hover


def test_rpm_conversion():
    geo = RotorGeometry(kf=3.16e-10)
    b = QuatBridge(geometry=geo)
    rpms = b.rpms((1e-4, 0.0, 0.0, 0.0))  # 1e-4 N with cf2-like kf
    assert rpms[0] == pytest.approx(np.sqrt(1e-4 / 3.16e-10), rel=1e-9)
    assert rpms[1] == 0.0


def test_reference_registry_matches_quat_sitl():
    from quat_sitl import references

    for t in (0.0, 3.3, 11.7):
        assert np.allclose(reference_quat("step", t), references.step_reference(t))
        assert np.allclose(reference_quat("sine", t), references.sine_reference(t))
        assert np.allclose(reference_quat("flip", t), references.flip_reference(t))
