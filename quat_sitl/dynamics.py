"""Rigid-body quadrotor attitude plant, eq. (17)-(18).

Assumptions (paper Section III): rigid symmetric structure, center of gravity
coincides with the body-frame origin, propellers rigid, gravity-bias throttle
neglected, only differential propeller forces affect rotation. Control-signal to
torque relation simplified to identity (see MotorModel).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil

import numpy as np
from numpy.typing import NDArray

from . import quaternion as quat

FloatArr = NDArray[np.float64]

RK4_STABILITY_MARGIN = 1.0
"""Target |lambda * dt_micro| ceiling for the fast rate-feedback eigenvalue, kept
comfortably under explicit RK4's real-axis stability bound (~2.785) for accuracy,
not just marginal stability. See stable_rk4_substeps."""


@dataclass(frozen=True)
class InertiaParams:
    """Diagonal inertia, paper Section V CAD-model values."""

    Ixx: float = 6.5e-4
    Iyy: float = 6.5e-4
    Izz: float = 1.2e-3

    def as_matrix(self) -> FloatArr:
        return np.diag([self.Ixx, self.Iyy, self.Izz])


class MotorModel:
    """Control-signal -> torque mapping. Base interface; paper simplifies this to
    identity, but keeps the relation swappable (e.g. for a first-order rotor lag)."""

    def step(self, cmd: FloatArr, dt: float) -> FloatArr:
        raise NotImplementedError

    def reset(self) -> None:
        pass


class IdentityMotorModel(MotorModel):
    """tau_actual = tau_cmd. The paper's simplification (control-to-torque = identity)."""

    def step(self, cmd: FloatArr, dt: float) -> FloatArr:
        return np.asarray(cmd, dtype=np.float64)


@dataclass
class FirstOrderLagMotorModel(MotorModel):
    """tau_dot = (cmd - tau) / tau_motor, forward-Euler. Unwired stub proving the
    control-to-torque relation is swappable per the paper's Section III note; not
    used by any scenario in this reproduction."""

    tau_motor: float = 0.02
    _state: FloatArr = field(default_factory=lambda: np.zeros(3), repr=False)

    def step(self, cmd: FloatArr, dt: float) -> FloatArr:
        cmd = np.asarray(cmd, dtype=np.float64)
        self._state = self._state + dt * (cmd - self._state) / self.tau_motor
        return self._state.copy()

    def reset(self) -> None:
        self._state = np.zeros(3)


def torque_saturate(tau: FloatArr, limit: float = 4.0) -> tuple[FloatArr, NDArray[np.bool_]]:
    """Elementwise clip to +/- limit N*m per axis (paper Section V bound)."""
    tau = np.asarray(tau, dtype=np.float64)
    saturated = np.abs(tau) > limit
    return np.clip(tau, -limit, limit), saturated


def stable_rk4_substeps(
    Pw: float, inertia: InertiaParams, dt: float, margin: float = RK4_STABILITY_MARGIN
) -> int:
    """Number of internal RK4 sub-steps needed to keep the rate-feedback loop
    numerically stable at plant step `dt`.

    The closed-loop rate-feedback term -Pw*omega, combined with eq.18's
    omegadot = Icm^-1 tau, produces a fast real eigenvalue of magnitude
    ~Pw/I_min (I_min = smallest principal inertia). For the paper's own values
    (Pw=4, Ixx=Iyy=6.5e-4) this is ~6150 rad/s: at dt=1e-3 that is far outside
    explicit RK4's real-axis stability region (|lambda*dt| <~ 2.785), causing
    divergent oscillation (amplification ~35x/step) that is a pure numerical
    artifact, NOT a sign or tuning defect — the underlying continuous-time
    system is heavily overdamped and well-behaved (see README "Results vs
    Paper"). This computes how many `dt`-subdividing RK4 sub-steps keep
    |lambda * dt/n| <= margin.
    """
    I_min = min(inertia.Ixx, inertia.Iyy, inertia.Izz)
    fast_eigenvalue = Pw / I_min
    return max(1, ceil(fast_eigenvalue * dt / margin))


def stable_control_rate_hz(Pw: float, inertia: InertiaParams, margin: float = 0.5) -> float:
    """Minimum control-tick (ZOH sample) rate needed for the DISCRETE closed loop
    to be stable, not just the plant's own ODE integration.

    This is a distinct, deeper issue than stable_rk4_substeps: even with an exact
    plant integrator, sampling the -Pw*omega rate-feedback at too slow a rate is
    itself an unstable discretization of the fast eigenvalue (~Pw/I_min), verified
    via the exact ZOH discrete-time closed-loop transition matrix. For the paper's
    own values (Pw=4, Ixx=6.5e-4) the fast pole is ~6150 rad/s (~980 Hz); sampling
    at the paper-SITL-realistic 200 Hz (or even 1 kHz) gives a discrete closed-loop
    spectral radius > 1 (unstable oscillation), confirmed numerically. margin=0.5
    keeps dt_control*fast_eigenvalue <= 0.5, comfortably inside the stable region
    (empirically the discrete closed loop is stable for dt*fast_eigenvalue below
    roughly 1.6-3.2 depending on gains; 0.5 leaves ample headroom). See README
    "Results vs Paper" for the full derivation and the resolution this forced.
    """
    I_min = min(inertia.Ixx, inertia.Iyy, inertia.Izz)
    fast_eigenvalue = Pw / I_min
    return fast_eigenvalue / margin


def state_derivative(state: FloatArr, tau: FloatArr, inertia: InertiaParams) -> FloatArr:
    """Eq. (18), THE PLANT:
        qdot   = -1/2 * [0, omega] (x) q      <- note the leading minus sign
        omegadot = Icm^-1 tau - Icm^-1 [omega x (Icm omega)]

    The minus sign is the paper's own (verified from the PDF text) and is the
    opposite sign of eq. (7)'s general body-frame derivative +1/2*[0,w'](x)q. The
    paper notes eqs. 6-7 are left-hand notation and that converting to right-hand
    notation requires conjugating the omega-quaternion, which flips this sign. This
    is preserved literally and NOT delegated to quaternion.qdot_body, so the
    deviation stays visible here rather than being silently "corrected" later. See
    README "Results vs Paper" for the full discussion, and test 7 in
    tests/test_quaternion.py for the guardrail.
    """
    q = state[0:4]
    omega = state[4:7]

    omega_quat = np.concatenate([[0.0], omega])
    qdot = -0.5 * quat.mul(omega_quat, q)

    Icm = inertia.as_matrix()
    Icm_inv = np.diag(1.0 / np.diag(Icm))
    omegadot = Icm_inv @ tau - Icm_inv @ np.cross(omega, Icm @ omega)

    return np.concatenate([qdot, omegadot])
