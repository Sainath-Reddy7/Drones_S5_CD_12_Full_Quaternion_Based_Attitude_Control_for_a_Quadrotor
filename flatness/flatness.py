"""Differential-flatness map — Choutri & Lagha 2017, Eqs. (15), (19)-(21).

Given the flat outputs and derivatives sigma = (x, y, z, psi), the map
returns everything the double-loop controller needs:

    thrust vector:  f = m (a_ref + g ẑ)                (Eq. 15, row 3)
    total thrust:   T_d = ||f||                        (altitude output)
    tilt target:    q_pd — the shortest-arc rotation taking ẑ onto f/||f||
                    (horizontal axis ⟹ zero z component, the paper's Eq. 19
                    "4th element equal to zero" in scalar-first order)
    yaw target:     q_zd = rotation about ẑ by psi_d   (Eq. 20)
    attitude ref:   q_d = q_pd ⊗ q_zd                  (Eq. 21, literal)

q_d is constructed exactly as the paper composes it, so both properties hold
exactly (body z along the thrust vector, tilt free of yaw content) for any
non-inverted thrust direction — no Euler singularities, which is the paper's
reason for the quaternion formulation. The only degeneracy is thrust pointing
exactly along -ẑ (alpha = pi), handled with a 180-deg tilt about the heading
axis. (An earlier Gram-Schmidt version differed from this object by a twist
about the thrust axis growing quadratically with tilt — found and replaced by
independent multi-model validation.)
"""
from __future__ import annotations

import numpy as np

from quat_sitl import quaternion as quat

from flatness.model import G, Vehicle
from flatness.trajectories import FlatRef


def dcm_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix (rows = body axes in world) -> scalar-first quaternion
    via Shepperd's method — numerically stable at any rotation."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return quat.normalize(np.array([w, x, y, z]))


def flatness_reference(ref: FlatRef, vehicle: Vehicle,
                       a_cmd: np.ndarray | None = None) -> tuple[float, np.ndarray]:
    """(T_d, q_d) from flat outputs. `a_cmd` (outer-loop LQR acceleration,
    world frame) replaces ref.a when given — the flatness feedforward plus
    LQR feedback of the paper's Fig. 3.

    LITERAL Eqs. (19)-(21): the tilt quaternion q_pd is the shortest-arc
    rotation taking ẑ onto the thrust direction (axis ẑ×f, horizontal by
    construction, so its z component — the paper's "4th element" in
    scalar-first order — is exactly zero, Eq. 19), q_zd is the yaw
    quaternion from ψ_d (2nd/3rd elements zero, Eq. 20), and q_d = q_pd ⊗
    q_zd (Eq. 21). The only degeneracy is thrust along −ẑ (α = π, axis
    undefined), which falls back to a tilt about the heading axis."""
    a = np.asarray(ref.a if a_cmd is None else a_cmd, float)
    f = vehicle.mass * (a + np.array([0.0, 0.0, G]))   # thrust vector, Eq. 15
    T_d = float(np.linalg.norm(f))
    if T_d < 1e-9:
        return 0.0, np.array([1.0, 0.0, 0.0, 0.0])
    d = f / T_d                                       # desired body-z (world)

    # Eq. (19): shortest-arc tilt, zero z component by construction
    axis = np.cross(np.array([0.0, 0.0, 1.0]), d)
    s = float(np.linalg.norm(axis))
    if s < 1e-9:
        if d[2] > 0.0:                                 # level: no tilt
            q_pd = np.array([1.0, 0.0, 0.0, 0.0])
        else:                                          # thrust = -ẑ: the one
            # true degeneracy — tilt 180 deg about the heading axis
            psi0 = ref.psi
            q_pd = quat.from_axis_angle(
                np.array([np.cos(psi0), np.sin(psi0), 0.0]), np.pi)
    else:
        alpha = float(np.arctan2(s, d[2]))
        q_pd = np.concatenate(([np.cos(alpha / 2.0)],
                               np.sin(alpha / 2.0) * axis / s))

    # Eq. (20): pure yaw quaternion (2nd/3rd elements zero)
    half = 0.5 * ref.psi
    q_zd = np.array([np.cos(half), 0.0, 0.0, np.sin(half)])

    # Eq. (21): composition
    return T_d, quat.normalize(quat.mul(q_pd, q_zd))


def attitude_error(q_d: np.ndarray, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Inner-loop error state (paper Eq. 17 state vector): vector part of the
    quaternion error q_e = q_d^-1 ⊗ q (approx. half the rotation vector) and
    the body angular rate omega (paper uses omega_ref = 0 on smooth paths).
    The vector part is shortest-path sign-corrected (q and -q are the same
    rotation; the paper's hover-linearized model implicitly assumes small
    errors)."""
    q_e = quat.mul(quat.conj(np.asarray(q_d, float)), np.asarray(q, float))
    q_vec = q_e[1:4].copy()
    if q_e[0] < 0.0:
        q_vec = -q_vec
    return q_vec, q_e
