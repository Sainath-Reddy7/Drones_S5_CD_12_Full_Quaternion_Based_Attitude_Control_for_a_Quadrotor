"""Sensor noise injection: additive zero-mean uniform noise, amplitude 0.1, on the
measured quaternion and angular rates (paper Section V). "Amplitude" is read as a
uniform bound U(-a, a) rather than a Gaussian sigma, since a Gaussian has no hard
amplitude; documented as an assumption in README (paper text doesn't specify the
distribution shape).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from . import quaternion as quat

FloatArr = NDArray[np.float64]


@dataclass
class SensorModel:
    noise_amplitude: float = 0.1
    rng: np.random.Generator = field(default_factory=np.random.default_rng)

    def measure(self, q_true: FloatArr, omega_true: FloatArr) -> tuple[FloatArr, FloatArr]:
        """Returns (q_meas, omega_meas). q_meas is renormalized after noise injection."""
        q_noisy = q_true + self.rng.uniform(-self.noise_amplitude, self.noise_amplitude, size=4)
        q_meas = quat.normalize(q_noisy)
        omega_meas = omega_true + self.rng.uniform(-self.noise_amplitude, self.noise_amplitude, size=3)
        return q_meas, omega_meas
