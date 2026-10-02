"""Double-loop LQR — Choutri & Lagha 2017, Section IV (Eqs. 17-18, 22-23).

Outer loop (position, Eq. 18): per-axis double integrator
    X = [p, p_dot],  X_dot = [[0,1],[0,0]] X + [0,1] a
optimal state feedback a_cmd = a_ref - K (X - X_ref) with K from the
algebraic Riccati equation (Eq. 23). The resulting (a_x, a_y, a_z) feed the
flatness map: T_d and the desired quaternion q_d (Eqs. 19-21).

Inner loop (attitude, Eq. 17): the paper's linearized hover model
    X = [q_vec(3), omega(3)],
    A = [[0, I/2], [0, 0]],  B = [[0], [J^-1]]
gives the torque command tau = -K x_err with x_err from the quaternion
error. Gains from the same continuous Riccati solve.

The paper states the gains were "tuned to be adaptive with the full
non-linear model" but gives no numbers; the defaults here are tuned on the
repo vehicle (see README) with the time-scale separation the paper's
figures show (~1 s position settling, attitude an order of magnitude
faster).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are

from flatness.model import Vehicle


def lqr_gain(A, B, Q, R) -> np.ndarray:
    """K = R^-1 B^T P with P the stabilizing solution of the continuous
    algebraic Riccati equation (paper Eq. 23)."""
    A, B, Q, R = map(np.asarray, (A, B, Q, R))
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


@dataclass
class PositionLQR:
    """Outer loop, Eq. (18): per-axis LQR on the double integrator."""

    q_pos: float = 49.0   # state weight on position error [1/m^2]
    q_vel: float = 42.0   # state weight on velocity error
    r_acc: float = 1.0    # input weight on acceleration
    acc_max_xy: float = 3.0   # m/s^2, lateral acceleration clamp (actuator-aware)
    acc_max_z: float = 5.0    # m/s^2, vertical acceleration clamp (both signs)

    def __post_init__(self):
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0], [1.0]])
        self.K = lqr_gain(A, B, np.diag([self.q_pos, self.q_vel]),
                          np.array([[self.r_acc]]))

    def acceleration(self, p, v, p_ref, v_ref, a_ref) -> np.ndarray:
        """a_cmd = a_ref - K [p - p_ref, v - v_ref] per axis (world frame),
        clamped to the acceleration envelope the rotors can realize at
        hover tilt (the paper's actuator limitations, Section V)."""
        e = np.stack([np.asarray(p, float) - np.asarray(p_ref, float),
                      np.asarray(v, float) - np.asarray(v_ref, float)])
        a = np.asarray(a_ref, float) - (self.K @ e).ravel()
        xy = np.linalg.norm(a[:2])
        if xy > self.acc_max_xy:
            a[:2] *= self.acc_max_xy / xy
        a[2] = float(np.clip(a[2], -self.acc_max_z, self.acc_max_z))
        return a


@dataclass
class AttitudeLQR:
    """Inner loop, Eq. (17): LQR on [q_vec, omega] of the linearized model."""

    q_quat: float = 100.0  # weight on quaternion-vector error
    q_omega: float = 20.0  # weight on body rate
    r_tau: float = 200.0   # input weight on torque (keeps commands inside the
                           # mixer's ~0.2 N*m hover headroom before saturation)
    tau_limit: float = 4.0  # N m — the repo airframe's paper bound

    def __post_init__(self):
        vehicle = Vehicle()
        J_inv = np.linalg.inv(np.diag(vehicle.inertia))
        self.A = np.zeros((6, 6))
        self.A[0:3, 3:6] = 0.5 * np.eye(3)      # q_vec_dot = omega / 2
        self.B = np.zeros((6, 3))
        self.B[3:6, :] = J_inv                  # omega_dot = J^-1 tau
        Q = np.diag([self.q_quat] * 3 + [self.q_omega] * 3)
        R = self.r_tau * np.eye(3)
        self.K = lqr_gain(self.A, self.B, Q, R)

    def torque(self, q_vec_err: np.ndarray, omega: np.ndarray) -> np.ndarray:
        """tau = -K [q_vec_err, omega] (Eq. 17 + Eq. 22), saturated."""
        x = np.concatenate([np.asarray(q_vec_err, float),
                            np.asarray(omega, float)])
        tau = -(self.K @ x)
        return np.clip(tau, -self.tau_limit, self.tau_limit)
