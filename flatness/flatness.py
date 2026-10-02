"""Differential-flatness map — Choutri & Lagha 2017, Eqs. (15), (19)-(21).

Given the flat outputs and derivatives sigma = (x, y, z, psi), the map
returns everything the double-loop controller needs:

    thrust vector:  f = m (a_ref + g ẑ)                (Eq. 15, row 3)
    total thrust:   T_d = ||f||                        (altitude output)
    tilt target:    q_pd — rotation taking ẑ onto f/||f|| with zero yaw
                    content (paper Eq. 19: the position quaternion, whose
                    yaw ("4th") component is zero in the paper's notation)
    yaw target:     q_zd = rotation about ẑ by psi_d   (Eq. 20)
    attitude ref:   q_d = q_pd ⊗ q_zd                  (Eq. 21)

q_d is constructed so that BOTH properties hold exactly (body z along the
thrust vector, yaw equal to psi_d) for any non-inverted thrust direction —
no Euler singularities, which is the paper's reason for the quaternion
formulation. The only degeneracy is thrust pointing exactly along -ẑ
(upside-down hover), guarded with a fallback tilt axis.
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
    LQR feedback of the paper's Fig. 3."""
    a = np.asarray(ref.a if a_cmd is None else a_cmd, float)
    f = vehicle.mass * (a + np.array([0.0, 0.0, G]))   # thrust vector, Eq. 15
    T_d = float(np.linalg.norm(f))
    if T_d < 1e-9:
        return 0.0, np.array([1.0, 0.0, 0.0, 0.0])
    z_d = f / T_d                                       # desired body-z (world)

    # Gram-Schmidt: complete z_d to a right-handed frame whose x-axis keeps
    # the desired yaw (equivalent to q_pd ⊗ q_zd, Eqs. 19-21)
    psi = ref.psi
    x_c = np.array([np.cos(psi), np.sin(psi), 0.0])  # desired heading (body-x)
    if abs(float(np.dot(z_d, x_c))) > 0.999:            # degenerate: near-inverted
        x_c = np.array([1.0, 0.0, 0.0])
    y_d = np.cross(z_d, x_c)                            # z x x = y (right-handed)
    y_d /= np.linalg.norm(y_d)
    x_d = np.cross(y_d, z_d)
    R_d = np.column_stack([x_d, y_d, z_d])   # columns = body axes in world
    return T_d, dcm_to_quat(R_d)


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
