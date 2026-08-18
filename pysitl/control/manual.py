"""Manual stick input -> attitude reference (+ throttle/climb-rate demand).

Sticks are normalized to [-1, 1] (roll, pitch, yaw_rate, throttle), matching
a real RC transmitter. STABILIZED mode maps roll/pitch directly to a bounded
target attitude via quat_sitl's own from_euler (eq. 15) and throttle directly
to a fraction of max thrust; yaw stick integrates a target heading, since
there's no "yaw position" stick on a real transmitter either -- only a rate.
ALTITUDE mode instead treats throttle as a climb-rate command around a held
target altitude (see run.py for how the two modes select which of
thrust_from_throttle / target_altitude gets used).
"""
from __future__ import annotations

from dataclasses import dataclass

from quat_sitl.quaternion import from_euler

MAX_TILT = 0.6  # rad (~34 deg): generous but bounded manual authority
MAX_YAW_RATE = 1.5  # rad/s


@dataclass
class ManualInput:
    roll: float = 0.0  # [-1, 1]
    pitch: float = 0.0  # [-1, 1]
    yaw_rate: float = 0.0  # [-1, 1]
    throttle: float = 0.0  # [0, 1], 0.5 ~ hover in STABILIZED

    def clamped(self) -> "ManualInput":
        c = lambda v, lo, hi: max(lo, min(hi, v))
        return ManualInput(c(self.roll, -1, 1), c(self.pitch, -1, 1), c(self.yaw_rate, -1, 1), c(self.throttle, 0, 1))


@dataclass
class StabilizedMapper:
    """Holds the integrated yaw target between calls (yaw stick is a rate,
    not a position)."""

    yaw_target: float = 0.0

    def reset(self, yaw: float = 0.0) -> None:
        self.yaw_target = yaw

    def attitude_reference(self, stick: ManualInput, dt: float) -> tuple:
        s = stick.clamped()
        self.yaw_target += s.yaw_rate * MAX_YAW_RATE * dt
        phi = s.roll * MAX_TILT
        theta = s.pitch * MAX_TILT
        return tuple(from_euler(phi, theta, self.yaw_target))

    @staticmethod
    def thrust_from_throttle(stick: ManualInput, rotor_thrust_max: float) -> float:
        """Throttle in [0,1] -> collective thrust in [0, 4*rotor_thrust_max],
        linear -- stick at 1.0 gives full available thrust."""
        return stick.clamped().throttle * 4.0 * rotor_thrust_max
