"""Generic RK4 integrator, decoupled from any specific state's physical meaning."""
from __future__ import annotations

from typing import Callable

import numpy as np
from numpy.typing import NDArray

FloatArr = NDArray[np.float64]
DerivFn = Callable[[FloatArr], FloatArr]


def rk4_step(f: DerivFn, x: FloatArr, dt: float) -> FloatArr:
    """Classic 4-stage Runge-Kutta step. f(state) -> state_dot only; callers close
    over any additional fixed arguments (e.g. a zero-order-held torque) themselves."""
    k1 = f(x)
    k2 = f(x + 0.5 * dt * k1)
    k3 = f(x + 0.5 * dt * k2)
    k4 = f(x + dt * k3)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def rk4_integrate(f: DerivFn, x: FloatArr, dt: float, n_substeps: int = 1) -> FloatArr:
    """Advance state by `dt` using `n_substeps` internal rk4_step calls of size
    dt/n_substeps. n_substeps=1 (default) is exactly rk4_step and preserves the
    project's literal fixed dt=1e-3 spec for any single-step test/usage.

    Needed because the closed-loop rate-feedback term (controller Pw gain over the
    plant's rotational inertia) produces a fast, real, physically-overdamped
    eigenvalue whose magnitude can exceed explicit RK4's real-axis stability bound
    (|lambda * dt| < ~2.785) at dt=1e-3 for the paper's own Pw/Ixx values — an
    RK4 numerical-stability artifact, not a design or sign defect (the underlying
    continuous-time system is not oscillatory; see README "Results vs Paper").
    simulator.py computes a safe n_substeps from the actual gains/inertia so the
    spec's 1 kHz plant-step / 200 Hz control-tick cadence stays intact as the
    logging/ZOH boundary while the true numerical integration stays stable.
    """
    if n_substeps <= 1:
        return rk4_step(f, x, dt)
    h = dt / n_substeps
    for _ in range(n_substeps):
        x = rk4_step(f, x, h)
    return x
