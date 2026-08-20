"""The paper's own measurement model, scalar/seeded for the hot loop:
additive zero-mean uniform noise of the configured amplitude on the measured
quaternion and body rates (Fresk & Nikolakopoulos Section V), renormalizing
the noisy quaternion. No estimator, no bias, no accel/gyro split -- the paper
has none of those, and the user asked to keep only what the paper has.

This is the scalar-tuple counterpart of quat_sitl.sensors.SensorModel (which
is numpy-based and used by the attitude-only reproduction); sim.py uses this
one so the 1 kHz sensor task stays off the numpy path.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field


@dataclass
class SensorModel:
    noise_amplitude: float = 0.1
    rng: random.Random = field(default_factory=random.Random)

    def measure(self, q_true: tuple, omega_true: tuple) -> tuple:
        """Returns (q_meas, omega_meas) as plain tuples; q_meas is
        renormalized after noise injection."""
        na = self.noise_amplitude
        q_noisy = tuple(c + self.rng.uniform(-na, na) for c in q_true)
        n = sum(c * c for c in q_noisy) ** 0.5
        q_meas = tuple(c / n for c in q_noisy) if n > 1e-12 else (1.0, 0.0, 0.0, 0.0)
        omega_meas = tuple(c + self.rng.uniform(-na, na) for c in omega_true)
        return q_meas, omega_meas
