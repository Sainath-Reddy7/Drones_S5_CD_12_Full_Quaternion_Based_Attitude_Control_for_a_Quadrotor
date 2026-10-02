"""Nonlinear quaternion quadrotor model — Choutri & Lagha 2017, Eq. (8) & (9).

The paper's Newton-Euler model with quaternions, adapted to the repo's Z-UP
convention and the repo's paper vehicle (the Fresk/Nikolakopoulos airframe:
0.2 kg, J = diag(6.5e-4, 6.5e-4, 1.2e-3), kf = 3.0e-9 N/(rad/s)^2,
km = 0.016*kf, tip-to-center arm 0.15 m), so the two papers share one
airframe:

    p_dot = v
    v_dot = R(q) [0,0,T/m] - [0,0,g]          (Eq. 8, row 1)
    q_dot = 1/2 q * [0, omega]                (Eq. 6/8, body rates)
    J omega_dot = -omega x (J omega) + tau    (Eq. 8, row 3)

Rotor mixer (Eq. 9, X-config, z-up): rotors at (+d,-d) FR, (+d,+d) FL,
(-d,+d) RL, (-d,-d) RR with d = arm/sqrt(2); FR & RL spin CW (negative yaw
reaction), FL & RR CCW:

    [T, tau_x, tau_y, tau_z]^T = M [w_FR, w_FL, w_RL, w_RR]^T,
    w_i = omega_i^2 (squared rotor speed)

Actuation limits and a first-order ESC lag model the paper's "limitations
over the actuators and battery modeling" (Section V; no numeric values are
given in the paper, so defaults are documented assumptions).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from quat_sitl import quaternion as quat

G = 9.81


@dataclass(frozen=True)
class Vehicle:
    """The repo's paper vehicle (identical to quat_sitl / pysitl / sims)."""

    mass: float = 0.2                       # kg
    inertia: tuple = (6.5e-4, 6.5e-4, 1.2e-3)  # kg m^2 (Ixx, Iyy, Izz)
    kf: float = 3.0e-9                      # N per (rad/s)^2
    km: float = 0.016 * 3.0e-9              # N m per (rad/s)^2
    arm: float = 0.15                       # tip-to-center [m]
    rotor_thrust_max: float = 1.95          # N per rotor (T/W = 4)
    esc_tau: float = 0.02                   # s, first-order ESC lag (assumed)

    @property
    def d(self) -> float:
        """Perpendicular rotor offset (arm / sqrt 2)."""
        return self.arm / np.sqrt(2.0)

    @property
    def w_max(self) -> float:
        """Max squared rotor speed (thrust cap per rotor)."""
        return self.rotor_thrust_max / self.kf


# Eq. (9): [T, tau] = M @ w  with w = (w_FR, w_FL, w_RL, w_RR) — z-up X-config
def _mixer_matrix(v: Vehicle) -> np.ndarray:
    kf, km, d = v.kf, v.km, v.d
    return np.array([
        [kf,     kf,     kf,     kf],     # total thrust
        [kf * d, kf * d, -kf * d, -kf * d],   # tau_x: front - rear
        [-kf * d, kf * d, kf * d, -kf * d],   # tau_y: left - right
        [-km,    km,     -km,    km],     # tau_z: yaw reaction (FR,RL CW)
    ])


class QuadrotorModel:
    """Continuous Eq. (8) dynamics + Eq. (9) mixer with actuator limits."""

    def __init__(self, vehicle: Vehicle | None = None, seed: int = 0):
        self.v = vehicle or Vehicle()
        self.M = _mixer_matrix(self.v)
        self.M_inv = np.linalg.inv(self.M)
        self.J = np.diag(self.v.inertia)
        self.J_inv = np.linalg.inv(self.J)
        self.rng = np.random.default_rng(seed)
        # state: p(3), v(3), q(4), omega(3), w_rotor(4, commanded squared)
        self.p = np.zeros(3)
        self.vv = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.omg = np.zeros(3)
        self.w_cmd = np.zeros(4)  # ESC-filtered squared speeds

    def reset(self, p0, v0=(0, 0, 0), q0=(1, 0, 0, 0), omg0=(0, 0, 0)):
        self.p = np.asarray(p0, float).copy()
        self.vv = np.asarray(v0, float).copy()
        self.q = quat.normalize(np.asarray(q0, float))
        self.omg = np.asarray(omg0, float).copy()
        self.w_cmd[:] = self.v.mass * G / (4 * self.v.kf)

    # -- Eq. (9) inverse: (T, tau) -> feasible squared rotor speeds -------
    def mix(self, T: float, tau: np.ndarray) -> np.ndarray:
        """Priority mixing (actuator-aware, the paper's Section V limits):
        thrust T is always delivered exactly; when the demanded torque would
        drive a rotor below 0 or above its max, the torque is uniformly
        scaled until feasible. Naive clipping instead corrupts the total
        thrust one-sidedly (clipped rotor loses thrust, unclipped keep it),
        which feeds back into the altitude loop and runs away."""
        tau = np.asarray(tau, float)

        def w_of(s: float) -> np.ndarray:
            return self.M_inv @ np.concatenate(([T], s * tau))

        w = w_of(1.0)
        lo, hi = 1.0, 1.0
        if w.min() < 0.0 or w.max() > self.v.w_max:
            lo, hi = 0.0, 1.0
            for _ in range(24):  # bisection: largest feasible torque scale
                mid = 0.5 * (lo + hi)
                ww = w_of(mid)
                if ww.min() >= 0.0 and ww.max() <= self.v.w_max:
                    lo = mid
                else:
                    hi = mid
            w = w_of(lo)
        return w

    def wrench(self, w: np.ndarray) -> tuple[float, np.ndarray]:
        """Forward Eq. (9): squared speeds -> (thrust, torques)."""
        out = self.M @ np.asarray(w, float)
        return float(out[0]), out[1:4]

    # -- Eq. (8) derivative ------------------------------------------------
    def deriv(self, state, T, tau):
        p, vv, q, omg = state
        R = quat.to_dcm(q)   # columns = body axes in world (v_world = R v_body)
        thrust_world = R @ np.array([0.0, 0.0, T / self.v.mass])
        v_dot = thrust_world - np.array([0.0, 0.0, G])
        q_dot = quat.qdot_body(q, omg)
        gyro = np.cross(omg, self.J @ omg)
        omg_dot = self.J_inv @ (-gyro + tau)
        return (vv, v_dot, q_dot, omg_dot)

    def state(self):
        return (self.p.copy(), self.vv.copy(), self.q.copy(), self.omg.copy())

    def step(self, dt: float, T: float, tau: np.ndarray) -> None:
        """One RK4 step at constant (T, tau) — zero-order hold."""
        T, tau = float(T), np.asarray(tau, float)
        # ESC lag on squared rotor speeds (actuator/battery model, assumed)
        w_target = self.mix(T, tau)
        alpha = 1.0 - np.exp(-dt / self.v.esc_tau) if self.v.esc_tau > 0 else 1.0
        self.w_cmd += alpha * (w_target - self.w_cmd)
        T_eff, tau_eff = self.wrench(self.w_cmd)

        s = self.state()
        k1 = self.deriv(s, T_eff, tau_eff)
        k2 = self.deriv(self._add(s, k1, dt / 2), T_eff, tau_eff)
        k3 = self.deriv(self._add(s, k2, dt / 2), T_eff, tau_eff)
        k4 = self.deriv(self._add(s, k3, dt), T_eff, tau_eff)
        dp = (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6
        dv = (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6
        dq = (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6
        do = (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) / 6
        self.p += dt * dp
        self.vv += dt * dv
        self.q = quat.normalize(self.q + dt * dq)
        self.omg += dt * do

    @staticmethod
    def _add(state, k, h):
        from quat_sitl import quaternion as _q
        p, v, qq, o = state
        return (p + h * k[0], v + h * k[1], _q.normalize(qq + h * k[2]), o + h * k[3])
