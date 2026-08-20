"""Altitude hold: PD+I on altitude error, converted to a collective thrust
command via tilt compensation (thrust must increase as the vehicle tilts,
since only the vertical component of body thrust fights gravity).

Not from the paper -- the paper has no translational control at all -- but
needed for the manual interactive altitude control the user asked for.
Validated by direct prototyping before this was written (plan finding 7):
holds 1.05 m stably, including with the paper's full 0.1 measurement noise.
"""
from __future__ import annotations

from dataclasses import dataclass

from ..params import G, GainParams, VehicleParams
from ..plant import rotate_body_to_world


@dataclass
class AltitudeController:
    gains: GainParams
    vehicle: VehicleParams
    integral: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def compute_thrust(self, z_ref: float, altitude: float, vz: float, q: tuple, dt: float) -> float:
        """z_ref/altitude in meters (up-positive), vz in m/s (up-positive).
        Returns a collective thrust magnitude, NOT yet mixed to rotors."""
        err = z_ref - altitude
        accel_cmd = self.gains.alt_kp * err - self.gains.alt_kd * vz + self.gains.alt_ki * self.integral
        accel_cmd = max(-0.9 * G, min(3.0 * G, accel_cmd))
        thrust_world_up = self.vehicle.mass * (G + accel_cmd)

        # cos(tilt) = world-z component of the body +z axis (down-positive
        # NED, so "level" means this is +1); guard near-90-degree tilt.
        tilt_cos = max(0.3, rotate_body_to_world(q, (0.0, 0.0, 1.0))[2])
        thrust = thrust_world_up / tilt_cos

        max_thrust = 4.0 * self.vehicle.rotor_thrust_max
        thrust_clamped = max(0.0, min(max_thrust, thrust))

        # Anti-windup: only integrate while the command is not saturated.
        if 0.0 < thrust < max_thrust:
            bound = self.gains.alt_ki_max
            self.integral = max(-bound, min(bound, self.integral + err * dt))

        return thrust_clamped
