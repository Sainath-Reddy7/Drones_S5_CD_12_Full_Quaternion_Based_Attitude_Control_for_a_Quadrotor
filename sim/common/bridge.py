"""Frozen paper controller + generic-vehicle bridge shared by all four sim adapters.

The invariant: quat_sitl.controller.NonlinearP2Controller is imported UNMODIFIED with
the paper's own gains (Pq=20, Pw=4) and torque bound (+/-4 N*m). Every adapter
converts its simulator's state into the paper's quaternion convention, calls the
same compute_torque, and applies the result. No per-simulator retuning is permitted
(see FRP.md) -- if performance differs across stacks, that difference is signal.

Convention bridge (why q_paper = conj(q_sim)):

The paper's eq. (18) integrates  qdot = -1/2 [0,w] (x) q  with the STANDARD Euler
equations  wdot = I^-1 (tau - w x (I w))  for the same body rates w. Differentiating
conj(q) of a standard body->world Hamilton quaternion q_sim (whose kinematics are
qdot_sim = +1/2 q_sim (x) [0,w]) gives exactly eq. (18)'s literal expression:

    d/dt conj(q_sim) = -conj(qdot_sim) = -1/2 [0,w] (x) conj(q_sim)

pysitl's plant.py verified this numerically (to_dcm(q_paper) maps world->body, and
rotate(conj(q_paper), v) maps body->world). Therefore each adapter computes

    q_m      = conj(q_sim)     # q_sim = simulator's scalar-first body->world quaternion
    omega_m  = sim body rates  # unchanged -- eq. 18's omega IS the standard body rate
    tau      = controller(...) # apply directly as the simulator's BODY-frame torque

Thrust convention: unlike pysitl's NED z-down plant, this bridge is z-UP -- the
collective thrust Tc >= 0 acts along the simulator's body +z (up) axis, which is
what PyBullet / MuJoCo / Gazebo / ArduPilot models naturally provide. The mixer
below is the z-up twin of pysitl/airframe.py's X-config mixer with the same
thrust-preserving desaturation: hold the common mode, scale only the zero-sum
torque deltas.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from quat_sitl import quaternion as quat
from quat_sitl.controller import ControllerGains, NonlinearP2Controller
from quat_sitl.dynamics import torque_saturate

FloatArr = NDArray[np.float64]

TORQUE_LIMIT = 4.0  # N*m, paper Section V


@dataclass(frozen=True)
class RotorGeometry:
    """Actuator geometry for the generic z-up X-config mixer. Defaults are the
    pysitl-derived paper vehicle (FRP.md section 2); adapters with their own
    vehicle (e.g. the Crazyflie 2.1 in gym-pybullet-drones) override these."""

    arm: float = 0.15 / np.sqrt(2.0)  # d = L/sqrt(2), perpendicular rotor offset [m]
    yaw_coeff: float = 0.016  # c = km/kf yaw reaction coefficient [m]
    rotor_thrust_max: float = 1.95  # per-rotor upper bound [N]
    kf: float | None = None  # N per rpm^2, for rpm-based sims (None = thrust-based)

    @property
    def thrust_max_total(self) -> float:
        return 4.0 * self.rotor_thrust_max


PAPER_VEHICLE = RotorGeometry()


@dataclass
class AltitudeHold:
    """PD+I altitude hold, structurally identical to pysitl/control/altitude.py
    (same gains, tilt compensation, anti-windup) but z-up and vehicle-agnostic:
    gains act in the acceleration domain so they carry over with vehicle mass.

    Not from the paper (it has no translational control) -- scaffolding that
    keeps the vehicle airborne while the paper's attitude scenarios run."""

    kp: float = 6.0
    kd: float = 4.0
    ki: float = 1.5
    ki_max: float = 2.0
    g: float = 9.81
    integral: float = 0.0
    # Collective-thrust ceiling as a multiple of hover thrust. None disables it
    # (pure altitude priority). Adapters for rotor-level actuated simulators
    # set ~2.2: tilt compensation that eats the whole rotor range leaves the
    # mixer zero headroom for torque (measured: a 360-degree flip deadlocks
    # exactly at 180 degrees, inverted, with desat_scale = 0). Capping thrust
    # is what real acro-mode firmware does during flips -- attitude first,
    # altitude later.
    max_thrust_factor: float | None = None

    def reset(self) -> None:
        self.integral = 0.0

    def thrust(
        self,
        z_ref: float,
        z: float,
        vz: float,
        tilt_cos: float,
        mass: float,
        thrust_max_total: float,
        dt: float,
    ) -> float:
        """z/vz up-positive [m, m/s]; tilt_cos = world-up component of the body
        +z axis (1 = level). Returns collective thrust [N] along body +z."""
        effective_max = thrust_max_total
        if self.max_thrust_factor is not None:
            effective_max = min(effective_max, self.max_thrust_factor * mass * self.g)

        err = z_ref - z
        accel_cmd = self.kp * err - self.kd * vz + self.ki * self.integral
        accel_cmd = max(-0.9 * self.g, min(3.0 * self.g, accel_cmd))
        thrust_world_up = mass * (self.g + accel_cmd)

        tilt_cos = max(0.3, tilt_cos)  # guard near-90-degree (or inverted) tilt
        thrust = thrust_world_up / tilt_cos

        thrust_clamped = max(0.0, min(effective_max, thrust))

        # Anti-windup: integrate only while unsaturated.
        if 0.0 < thrust < effective_max:
            self.integral = max(-self.ki_max, min(self.ki_max, self.integral + err * dt))

        return thrust_clamped


def mix_zup(
    thrust_cmd: float,
    torque_cmd: FloatArr,
    geometry: RotorGeometry,
) -> tuple[tuple[float, float, float, float], float]:
    """Inverse mix (collective thrust + body torque -> 4 rotor thrusts) with
    thrust-preserving desaturation, z-up twin of pysitl/airframe.py:mix.

    Rotor order and arm positions match pysitl's ARM_TIPS layout
    [(d,d), (-d,-d), (d,-d), (-d,d)]; rotors 1,2 spin CCW (reaction -z on the
    airframe), rotors 3,4 CW (+z). Forward mix:
        Fz = T1+T2+T3+T4
        tx = d*( T1 - T2 - T3 + T4)
        ty = d*(-T1 + T2 - T3 + T4)
        tz = c*(-T1 - T2 + T3 + T4)
    Returns (rotor_thrusts, desat_scale) with desat_scale in [0,1] = fraction of
    the commanded torque that survived saturation (1.0 = unsaturated).
    """
    d = geometry.arm
    c = geometry.yaw_coeff
    tmax = geometry.rotor_thrust_max
    tx, ty, tz = (float(torque_cmd[0]), float(torque_cmd[1]), float(torque_cmd[2]))

    base = max(0.0, min(tmax, thrust_cmd / 4.0))

    a, b, g = tx / d, ty / d, tz / c
    deltas = ((a - b - g) / 4.0, (-a + b - g) / 4.0, (-a - b + g) / 4.0, (a + b + g) / 4.0)

    scale = 1.0
    for delta in deltas:
        if delta > 1e-12:
            scale = min(scale, (tmax - base) / delta)
        elif delta < -1e-12:
            scale = min(scale, (0.0 - base) / delta)
    scale = max(0.0, min(1.0, scale))

    thrusts = tuple(base + scale * delta for delta in deltas)
    return thrusts, scale  # type: ignore[return-value]


def thrusts_to_wrench_zup(
    thrusts: tuple[float, float, float, float], geometry: RotorGeometry
) -> tuple[float, FloatArr]:
    """Forward mix: 4 rotor thrusts -> (Fz_body_up, body torque). Round-trip
    partner of mix_zup; used by telemetry and the mixer test."""
    t1, t2, t3, t4 = thrusts
    d = geometry.arm
    c = geometry.yaw_coeff
    fz = t1 + t2 + t3 + t4
    tau = np.array(
        [d * (t1 - t2 - t3 + t4), d * (-t1 + t2 - t3 + t4), c * (-t1 - t2 + t3 + t4)]
    )
    return fz, tau


@dataclass
class QuatBridge:
    """One frozen paper controller bound to one vehicle geometry. This is the
    ONLY object the four adapters talk to -- identical gains, saturation, mixer
    semantics, and scenario references everywhere."""

    geometry: RotorGeometry = field(default_factory=RotorGeometry)
    shortest_path: bool = True
    gains: ControllerGains = field(default_factory=ControllerGains)

    def __post_init__(self) -> None:
        self.controller = NonlinearP2Controller(gains=self.gains, shortest_path=self.shortest_path)

    def torque(self, q_ref: FloatArr, q_m: FloatArr, omega_m: FloatArr) -> tuple[FloatArr, FloatArr]:
        """Paper eqs. (19)-(21) with the paper's +/-4 N*m clip. q_ref/q_m in the
        paper's scalar-first convention (adapter-side conjugated), omega_m body."""
        tau = self.controller.compute_torque(
            np.asarray(q_ref, dtype=np.float64),
            np.asarray(q_m, dtype=np.float64),
            np.asarray(omega_m, dtype=np.float64),
        )
        return torque_saturate(tau, TORQUE_LIMIT)

    def mix(self, thrust_cmd: float, tau_cmd: FloatArr) -> tuple[tuple[float, ...], float]:
        return mix_zup(thrust_cmd, tau_cmd, self.geometry)

    def rpms(self, thrusts: tuple[float, ...]) -> tuple[float, ...]:
        """Per-rotor RPM for rpm-based sims (thrust = kf * rpm^2)."""
        if self.geometry.kf is None:
            raise ValueError("geometry.kf not set -- simulator is thrust-based, not rpm-based")
        return tuple(float(np.sqrt(max(t, 0.0) / self.geometry.kf)) for t in thrusts)

    def attitude_error_angle(self, q_ref: FloatArr, q_m: FloatArr) -> float:
        """Geodesic angle between reference and measured attitude [rad]:
        alpha = 2*acos(|<q_ref, q_m>|). Frame-free tracking metric for the
        cross-simulator comparison."""
        dot = abs(float(np.dot(q_ref, q_m)))
        return 2.0 * float(np.arccos(np.clip(dot, -1.0, 1.0)))
