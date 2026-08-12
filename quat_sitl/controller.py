"""Nonlinear P^2 attitude controller, eq. (19)-(21). Derivative-free, no integrator —
the paper's double-integrator plant dynamics drive error to zero without one.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from . import quaternion as quat

FloatArr = NDArray[np.float64]


@dataclass
class ControllerGains:
    """Paper Section V tuned values."""

    Pq: float = 20.0
    Pw: float = 4.0


@dataclass
class NonlinearP2Controller:
    """Outer loop Pq (attitude), inner loop Pw (rate), combined per Fig. 2 block
    diagram. `shortest_path` must be False only for the 360-degree flip scenario,
    where the controller must commit to the full rotation rather than the short way."""

    gains: ControllerGains = field(default_factory=ControllerGains)
    shortest_path: bool = True

    def compute_torque(self, q_ref: FloatArr, q_m: FloatArr, omega_m: FloatArr) -> FloatArr:
        """Eq. (19)-(21). Returns UNSATURATED torque command (3,); saturation is
        applied downstream by dynamics.torque_saturate, called from simulator.py."""
        q_err = quat.mul(q_ref, quat.conj(q_m))  # eq. 19
        axis_err = np.array(q_err[1:4])  # eq. 20

        if self.shortest_path and q_err[0] < 0.0:
            axis_err = -axis_err

        omega_m = np.asarray(omega_m, dtype=np.float64)
        return -self.gains.Pq * axis_err - self.gains.Pw * omega_m  # eq. 21
