"""6-DOF rigid-body quadrotor plant. Scalar Python in the hot path (finding 2
in the plan: numpy costs 65.5us/step, plain floats cost 5.6us/step, and the
paper's gains force ~12.3 kHz -- numpy would only run at 1.24x real-time,
too slow for interactive flying; scalars give ~14.5x).

State is a 13-tuple (x, y, z, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz):
  - position/velocity in NED (north, east, down), PX4's own convention
  - quaternion scalar-first, same convention as quat_sitl.quaternion
  - body rates in the body frame

Rotational dynamics are eq. 18 from quat_sitl.dynamics.state_derivative,
reimplemented here as flat scalar arithmetic for speed but kept
term-for-term identical (see tests/test_sitl.py::test_eq18_parity), INCLUDING
the paper's leading-minus sign.

Translational dynamics need one fact not in the attitude-only paper: which
way rotation maps thrust into the world frame. Verified numerically (see plan
finding 1) by integrating a known body rate and checking where the body
x-axis lands in world frame:
    to_dcm(q)      maps world -> body
    to_dcm(q).T    maps body  -> world   (== quat.rotate(quat.conj(q), v))
So world-frame thrust = rotate_body_to_world(q, [0, 0, -thrust]).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .params import G, VehicleParams

State = tuple  # 13 floats; kept as a plain tuple for hot-path speed


def _qmul(p, q):
    p0, p1, p2, p3 = p
    q0, q1, q2, q3 = q
    return (
        p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
        p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
        p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
        p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0,
    )


def _qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def rotate_body_to_world(q, v):
    """quat.rotate(quat.conj(q), v), inlined scalar -- see module docstring
    for why conj(q) (not q) is the body->world direction under eq.18's sign."""
    qc = _qconj(q)
    r = _qmul(_qmul(qc, (0.0, v[0], v[1], v[2])), q)
    return (r[1], r[2], r[3])


def state_derivative(
    state: State,
    thrust: float,
    torque: tuple,
    vehicle: VehicleParams,
) -> State:
    """d(state)/dt. thrust is the collective magnitude along body -z (a
    positive thrust lifts when level). torque is (taux, tauy, tauz) in the
    body frame, already past the mixer/desaturation."""
    x, y, z, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz = state
    q = (q0, q1, q2, q3)

    # eq.18, first half: qdot = -1/2 * [0,w] (x) q -- literal paper sign.
    qd = _qmul((0.0, wx, wy, wz), q)
    qd = (-0.5 * qd[0], -0.5 * qd[1], -0.5 * qd[2], -0.5 * qd[3])

    fwx, fwy, fwz = rotate_body_to_world(q, (0.0, 0.0, -thrust))
    k = vehicle.linear_drag_coeff
    ax = fwx / vehicle.mass - k * vx / vehicle.mass
    ay = fwy / vehicle.mass - k * vy / vehicle.mass
    az = fwz / vehicle.mass - k * vz / vehicle.mass + G

    Ixx, Iyy, Izz = vehicle.inertia.Ixx, vehicle.inertia.Iyy, vehicle.inertia.Izz
    tx, ty, tz = torque
    cx = wy * Izz * wz - wz * Iyy * wy
    cy = wz * Ixx * wx - wx * Izz * wz
    cz = wx * Iyy * wy - wy * Ixx * wx
    wdx = (tx - cx) / Ixx
    wdy = (ty - cy) / Iyy
    wdz = (tz - cz) / Izz

    return (vx, vy, vz, qd[0], qd[1], qd[2], qd[3], ax, ay, az, wdx, wdy, wdz)


def rk4_step(f: Callable[[State], State], s: State, dt: float, *args) -> State:
    """Scalar RK4, generic over the derivative function's extra args (thrust,
    torque, vehicle), mirroring quat_sitl.integrator.rk4_step's shape but
    without numpy so it stays on the fast scalar path."""

    def add(a, b, h):
        return tuple(ai + h * bi for ai, bi in zip(a, b))

    k1 = f(s, *args)
    k2 = f(add(s, k1, dt / 2), *args)
    k3 = f(add(s, k2, dt / 2), *args)
    k4 = f(add(s, k3, dt), *args)
    return tuple(
        si + (dt / 6.0) * (a + 2 * b + 2 * c + d)
        for si, a, b, c, d in zip(s, k1, k2, k3, k4)
    )


def normalize_quat(state: State) -> State:
    x, y, z, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz = state
    n = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) ** 0.5
    if n < 1e-12:
        q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
    else:
        q0, q1, q2, q3 = q0 / n, q1 / n, q2 / n, q3 / n
    return (x, y, z, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz)


def apply_ground_contact(state: State, vehicle: VehicleParams, dt: float) -> State:
    """Simple non-bouncy ground model: down (NED z) can't go below 0
    (altitude can't go below 0); a vehicle resting on the ground gets its
    sinking velocity zeroed and horizontal velocity damped by friction, but
    is free to lift off (negative vz / climbing) the instant thrust allows."""
    x, y, z, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz = state
    if z >= 0.0:
        z = 0.0
        if vz > 0.0:
            vz = 0.0
        damp = max(0.0, 1.0 - vehicle.ground_friction * dt)
        vx *= damp
        vy *= damp
    return (x, y, z, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz)


IDENTITY_STATE: State = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class Plant:
    """Thin stateful wrapper: holds the current 13-state and vehicle params,
    advances one physics step (RK4 + ground contact + renormalize)."""

    vehicle: VehicleParams
    state: State = IDENTITY_STATE

    def step(self, thrust: float, torque: tuple, dt: float) -> None:
        self.state = rk4_step(state_derivative, self.state, dt, thrust, torque, self.vehicle)
        self.state = normalize_quat(self.state)
        self.state = apply_ground_contact(self.state, self.vehicle, dt)

    def reset(self, state: State = IDENTITY_STATE) -> None:
        self.state = state

    @property
    def position(self) -> tuple:
        return self.state[0:3]

    @property
    def altitude(self) -> float:
        return -self.state[2]

    @property
    def quaternion(self) -> tuple:
        return self.state[3:7]

    @property
    def velocity(self) -> tuple:
        return self.state[7:10]

    @property
    def omega(self) -> tuple:
        return self.state[10:13]
