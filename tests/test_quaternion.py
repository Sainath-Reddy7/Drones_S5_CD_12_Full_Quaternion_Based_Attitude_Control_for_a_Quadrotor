"""Test suite for quat_sitl. Exactly 7 tests per project spec (added incrementally
per build-order stage — stage 1 adds tests 1-4, stage 2 adds 5-6, stage 3 adds 7).

1. mul non-commutative + Q/Q_bar cross-check
2. q (x) conj(q) == identity
3. to_dcm orthonormal, det=+1, round-trips from_euler/to_euler
4. rotate(q, v) == to_dcm(q) @ v
5. quaternion norm drift < 1e-6 over 10s of unrenormalized RK4 integration
6. zero-torque, zero-rate plant state unchanged
7. sign-consistency: eq.18/eq.21 together produce negative feedback (fails loudly),
   plus a folded-in closed-loop convergence smoke check
"""
from __future__ import annotations

import numpy as np

from quat_sitl import quaternion as quat
from quat_sitl import dynamics, integrator, controller  # simulator added in stage 4


def _random_unit_quaternions(n: int, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    q = rng.normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def test_mul_noncommutative_and_matrix_forms():
    p = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.7)
    q = quat.from_axis_angle(np.array([0.0, 1.0, 0.0]), 1.1)

    pq = quat.mul(p, q)
    qp = quat.mul(q, p)
    assert not np.allclose(pq, qp)

    assert np.allclose(pq, quat.Q(p) @ q, atol=1e-12)
    assert np.allclose(pq, quat.Q_bar(q) @ p, atol=1e-12)


def test_q_times_conj_q_is_identity():
    qs = _random_unit_quaternions(20, seed=1)
    for q in qs:
        assert np.allclose(quat.mul(q, quat.conj(q)), [1.0, 0.0, 0.0, 0.0], atol=1e-10)
        assert np.allclose(quat.mul(quat.conj(q), q), [1.0, 0.0, 0.0, 0.0], atol=1e-10)


def test_to_dcm_orthonormal_and_round_trips_euler():
    angles = np.linspace(-1.2, 1.2, 5)
    for phi in angles:
        for theta in angles:
            for psi in angles:
                q = quat.from_euler(phi, theta, psi)
                R = quat.to_dcm(q)
                assert np.allclose(R.T @ R, np.eye(3), atol=1e-10)
                assert np.isclose(np.linalg.det(R), 1.0, atol=1e-10)
                phi2, theta2, psi2 = quat.to_euler(q)
                assert np.allclose([phi, theta, psi], [phi2, theta2, psi2], atol=1e-8)


def test_rotate_matches_dcm():
    rng = np.random.default_rng(2)
    qs = _random_unit_quaternions(10, seed=3)
    vs = rng.normal(size=(10, 3))
    for q, v in zip(qs, vs):
        assert np.allclose(quat.rotate(q, v), quat.to_dcm(q) @ v, atol=1e-10)


def test_quaternion_norm_drift_under_10s_free_integration():
    dt = 1e-3
    n_steps = round(10.0 / dt)
    omega = np.array([0.3, -0.2, 0.5])
    q = np.array([1.0, 0.0, 0.0, 0.0])

    def f(qs: np.ndarray) -> np.ndarray:
        return quat.qdot_body(qs, omega)

    for _ in range(n_steps):
        q = integrator.rk4_step(f, q, dt)  # no renormalization on purpose

    assert abs(quat.norm(q) - 1.0) < 1e-6


def test_zero_torque_zero_rate_state_unchanged():
    state = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    tau = np.zeros(3)
    inertia = dynamics.InertiaParams()

    def f(s: np.ndarray) -> np.ndarray:
        return dynamics.state_derivative(s, tau, inertia)

    state_next = integrator.rk4_step(f, state, 1e-3)
    assert np.allclose(state_next, state, atol=1e-12)


def test_sign_consistency_negative_feedback_and_closed_loop_convergence():
    # --- targeted sign-consistency check (fails loudly on a single-equation sign bug) ---
    # eq.18's plant has a leading minus sign not present in eq.7 (see dynamics.py), which
    # flips the naive omega-vs-attitude-change intuition: NOTE this means a positive x-axis
    # attitude error is corrected by a *positive* commanded omega_x here, not negative, since
    # the paper's qdot = -1/2*[0,omega]⊗q inverts that mapping relative to the textbook +1/2
    # form. Rather than hardcode that (easy to get backwards), assert the only thing that
    # actually matters: the attitude error must shrink after one closed-loop step. A sign bug
    # in either eq.18 or eq.21 makes the error grow instead, so this still fails loudly.
    inertia = dynamics.InertiaParams()
    ctrl = controller.NonlinearP2Controller()

    q_true = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.1)
    q_ref = np.array([1.0, 0.0, 0.0, 0.0])
    omega = np.array([0.0, 0.0, 0.0])

    def angle_err(q: np.ndarray) -> float:
        qe = quat.mul(q_ref, quat.conj(q))
        return 2.0 * np.arccos(np.clip(abs(qe[0]), -1.0, 1.0))

    tau = ctrl.compute_torque(q_ref, q_true, omega)
    state = np.concatenate([q_true, omega])

    def f(s: np.ndarray) -> np.ndarray:
        return dynamics.state_derivative(s, tau, inertia)

    state_next = integrator.rk4_step(f, state, 1e-3)
    err_before = angle_err(q_true)
    err_after = angle_err(state_next[0:4])
    assert err_after < err_before, "closed-loop step must reduce attitude error (negative feedback)"

    # --- folded-in closed-loop convergence smoke check (noise-free, ZOH control loop) ---
    # Runs the controller at dynamics.stable_control_rate_hz rather than a naive 1 kHz/
    # 200 Hz sample rate: the -Pw*omega feedback samples a fast real closed-loop pole
    # (~Pw/Ixx) that is provably unstable as a DISCRETE (sampled/ZOH) control loop at
    # either of those rates (verified via exact ZOH discretization — see
    # dynamics.stable_control_rate_hz docstring and README "Results vs Paper"). This is
    # not a plant-integration accuracy issue (RK4 is already stable at 1e-3 for a FIXED
    # torque); it is the control SAMPLE rate itself that must be fast enough.
    inertia = dynamics.InertiaParams()
    ctrl = controller.NonlinearP2Controller()
    q_ref = quat.from_euler(1.0, 0.0, 0.0)
    state = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    control_hz = dynamics.stable_control_rate_hz(ctrl.gains.Pw, inertia)
    dt = 1.0 / control_hz
    for _ in range(round(2.0 / dt)):
        q_m, omega_m = state[0:4], state[4:7]
        tau = ctrl.compute_torque(q_ref, q_m, omega_m)
        tau, _ = dynamics.torque_saturate(tau)

        def f(s: np.ndarray, tau=tau) -> np.ndarray:
            return dynamics.state_derivative(s, tau, inertia)

        state = integrator.rk4_step(f, state, dt)
        state[0:4] = quat.normalize(state[0:4])

    q_err = quat.mul(q_ref, quat.conj(state[0:4]))
    angle_err = 2 * np.arccos(np.clip(abs(q_err[0]), -1.0, 1.0))
    assert angle_err < 0.01
