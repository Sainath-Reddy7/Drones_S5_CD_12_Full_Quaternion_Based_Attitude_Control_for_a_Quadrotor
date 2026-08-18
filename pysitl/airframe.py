"""Rotor model and control allocation (mixer).

X configuration, rotor order [front-right, rear-left, front-left, rear-right]
matching quat_sitl's own ARM_TIPS convention ([L,L,0], [-L,-L,0], [L,-L,0],
[-L,L,0]) so the 3D viewers agree:

    rotor 1: ( d,  d, 0)   spins CCW (+z reaction on airframe)
    rotor 2: (-d, -d, 0)   spins CCW
    rotor 3: ( d, -d, 0)   spins CW  (-z reaction on airframe)
    rotor 4: (-d,  d, 0)   spins CW

Forward mix (rotor thrusts -> body wrench), d = arm_length/sqrt(2), c = yaw
reaction coefficient:
    Fz  = -(T1+T2+T3+T4)                    (thrust acts along body -z)
    tx  = -d*(T1 - T2 - T3 + T4)
    ty  =  d*(T1 - T2 + T3 - T4)
    tz  =  c*(T1 + T2 - T3 - T4)

Inverse mix (desired collective thrust + torque -> 4 rotor thrusts) is a
straightforward 4x4 solve, but naively CLIPPING each rotor to
[0, rotor_thrust_max] after that solve is wrong: when torque demand
saturates, two rotors clip to max and two to zero, and total thrust changes
(measured: it can double, causing altitude runaway -- see plan finding 5).
The fix is thrust-preserving desaturation: hold the common-mode thrust
`base = Tc/4` fixed, and scale the *torque-driven* per-rotor deltas (which
sum to zero by construction) by the largest factor s in [0,1] that keeps
every rotor within [0, rotor_thrust_max]. Since the deltas are zero-sum,
total thrust is exactly preserved regardless of s.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .params import VehicleParams

# Body-frame rotor tip positions, X config -- matches quat_sitl's ARM_TIPS
# layout exactly (visualize3d.py, the JS consoles) so every viewer agrees.
_TIP_SIGNS = ((1, 1), (-1, -1), (1, -1), (-1, 1))


def rotor_positions(vehicle: VehicleParams) -> tuple:
    d = vehicle.rotor_arm
    return tuple((sx * d, sy * d, 0.0) for sx, sy in _TIP_SIGNS)


def thrusts_to_wrench(thrusts: tuple, vehicle: VehicleParams) -> tuple:
    """Forward mix: 4 rotor thrusts -> (Fz_body, taux, tauy, tauz). Used for
    telemetry and the mixer round-trip test."""
    T1, T2, T3, T4 = thrusts
    d = vehicle.rotor_arm
    c = vehicle.yaw_torque_coeff
    Fz = -(T1 + T2 + T3 + T4)
    tx = -d * (T1 - T2 - T3 + T4)
    ty = d * (T1 - T2 + T3 - T4)
    tz = c * (T1 + T2 - T3 - T4)
    return (Fz, tx, ty, tz)


def mix(
    thrust_cmd: float,
    torque_cmd: tuple,
    vehicle: VehicleParams,
) -> tuple:
    """Inverse mix with thrust-preserving desaturation.

    Returns (rotor_thrusts[4], desat_scale) where desat_scale in [0,1] is how
    much of the commanded torque actually made it through (1.0 = no
    saturation) -- exposed for telemetry so saturation is visible, matching
    the paper's own sat_x/y/z logging.
    """
    d = vehicle.rotor_arm
    c = vehicle.yaw_torque_coeff
    Tmax = vehicle.rotor_thrust_max
    tx, ty, tz = torque_cmd

    base = thrust_cmd / 4.0
    base = max(0.0, min(Tmax, base))

    # Zero-sum per-rotor torque deltas (D1+D2+D3+D4 == 0 by construction).
    deltas = (
        -tx / (4 * d) + ty / (4 * d) + tz / (4 * c),
        tx / (4 * d) - ty / (4 * d) + tz / (4 * c),
        tx / (4 * d) + ty / (4 * d) - tz / (4 * c),
        -tx / (4 * d) - ty / (4 * d) - tz / (4 * c),
    )

    scale = 1.0
    for delta in deltas:
        if delta > 1e-12:
            scale = min(scale, (Tmax - base) / delta)
        elif delta < -1e-12:
            scale = min(scale, (0.0 - base) / delta)
    scale = max(0.0, min(1.0, scale))

    thrusts = tuple(base + scale * delta for delta in deltas)
    return thrusts, scale


@dataclass
class Airframe:
    """Per-rotor first-order actuator lag (ESC + motor spin-up/down response,
    time constant vehicle.motor_tau) applied on top of the mixer's commanded
    thrusts -- this is what gives the vehicle PX4-like actuator dynamics
    instead of instantaneous thrust."""

    vehicle: VehicleParams
    _actual: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])

    def reset(self) -> None:
        self._actual = [0.0, 0.0, 0.0, 0.0]

    def actuate(self, rotor_cmds: tuple, dt: float) -> tuple:
        tau = self.vehicle.motor_tau
        for i in range(4):
            self._actual[i] += dt * (rotor_cmds[i] - self._actual[i]) / tau
        return tuple(self._actual)
