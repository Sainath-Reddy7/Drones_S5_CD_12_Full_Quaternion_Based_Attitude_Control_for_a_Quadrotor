"""Quaternion algebra, scalar-first convention q = [q0, q1, q2, q3], left-hand notation.

Implements Fresk & Nikolakopoulos (ECC 2013), eq. (1)-(16). Every function accepts a
single quaternion/vector or a batch (N,4)/(N,3) and returns the same leading shape.
"""
from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

FloatArr = NDArray[np.float64]


def _as_batch(x: FloatArr, width: int) -> tuple[FloatArr, bool]:
    x = np.asarray(x, dtype=np.float64)
    if x.ndim == 1:
        if x.shape[-1] != width:
            raise ValueError(f"expected last dim {width}, got shape {x.shape}")
        return x[None, :], True
    if x.shape[-1] != width:
        raise ValueError(f"expected last dim {width}, got shape {x.shape}")
    return x, False


def _unbatch(x: FloatArr, was_1d: bool) -> FloatArr:
    return x[0] if was_1d else x


def mul(p: FloatArr, q: FloatArr) -> FloatArr:
    """Hamilton/Kronecker product p (x) q, eq. (2). Non-commutative."""
    p2, p1d = _as_batch(p, 4)
    q2, q1d = _as_batch(q, 4)
    p0, p1, p2_, p3 = p2[..., 0], p2[..., 1], p2[..., 2], p2[..., 3]
    q0, q1, q2_, q3 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
    out = np.stack(
        [
            p0 * q0 - p1 * q1 - p2_ * q2_ - p3 * q3,
            p0 * q1 + p1 * q0 + p2_ * q3 - p3 * q2_,
            p0 * q2_ - p1 * q3 + p2_ * q0 + p3 * q1,
            p0 * q3 + p1 * q2_ - p2_ * q1 + p3 * q0,
        ],
        axis=-1,
    )
    return _unbatch(out, p1d and q1d)


def Q(p: FloatArr) -> FloatArr:
    """Left-multiplication matrix: Q(p) @ q == mul(p, q). Eq. (2)."""
    p2, was_1d = _as_batch(p, 4)
    p0, p1, p2_, p3 = p2[..., 0], p2[..., 1], p2[..., 2], p2[..., 3]
    z = np.zeros_like(p0)
    row0 = np.stack([p0, -p1, -p2_, -p3], axis=-1)
    row1 = np.stack([p1, p0, -p3, p2_], axis=-1)
    row2 = np.stack([p2_, p3, p0, -p1], axis=-1)
    row3 = np.stack([p3, -p2_, p1, p0], axis=-1)
    out = np.stack([row0, row1, row2, row3], axis=-2)
    return out[0] if was_1d else out


def Q_bar(q: FloatArr) -> FloatArr:
    """Right-multiplication matrix: Q_bar(q) @ p == mul(p, q). Eq. (2)."""
    q2, was_1d = _as_batch(q, 4)
    q0, q1, q2_, q3 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
    row0 = np.stack([q0, -q1, -q2_, -q3], axis=-1)
    row1 = np.stack([q1, q0, q3, -q2_], axis=-1)
    row2 = np.stack([q2_, -q3, q0, q1], axis=-1)
    row3 = np.stack([q3, q2_, -q1, q0], axis=-1)
    out = np.stack([row0, row1, row2, row3], axis=-2)
    return out[0] if was_1d else out


def norm(q: FloatArr) -> FloatArr:
    """Eq. (3)."""
    q2, was_1d = _as_batch(q, 4)
    n = np.sqrt(np.sum(q2 * q2, axis=-1))
    return float(n[0]) if was_1d else n


def conj(q: FloatArr) -> FloatArr:
    """Eq. (4): negate vector part."""
    q2, was_1d = _as_batch(q, 4)
    out = q2 * np.array([1.0, -1.0, -1.0, -1.0])
    return _unbatch(out, was_1d)


def inv(q: FloatArr) -> FloatArr:
    """Eq. (5): q* / ||q||^2."""
    q2, was_1d = _as_batch(q, 4)
    n2 = np.sum(q2 * q2, axis=-1, keepdims=True)
    if np.any(n2 < 1e-24):
        raise ValueError("cannot invert near-zero-norm quaternion")
    out = (q2 * np.array([1.0, -1.0, -1.0, -1.0])) / n2
    return _unbatch(out, was_1d)


def normalize(q: FloatArr) -> FloatArr:
    """q / ||q||. Required for renormalization after integration/noise injection."""
    q2, was_1d = _as_batch(q, 4)
    n = np.sqrt(np.sum(q2 * q2, axis=-1, keepdims=True))
    if np.any(n < 1e-12):
        raise ValueError("cannot normalize near-zero-norm quaternion")
    out = q2 / n
    return _unbatch(out, was_1d)


def qdot_fixed(q: FloatArr, omega: FloatArr) -> FloatArr:
    """Eq. (6): qdot = 1/2 * q (x) [0, omega], omega in fixed frame."""
    q2, q1d = _as_batch(q, 4)
    w2, w1d = _as_batch(omega, 3)
    zero = np.zeros(w2.shape[:-1] + (1,))
    omega_quat = np.concatenate([zero, w2], axis=-1)
    out = 0.5 * mul(q2, omega_quat)
    return _unbatch(out, q1d and w1d)


def qdot_body(q: FloatArr, omega_body: FloatArr) -> FloatArr:
    """Eq. (7): qdot = 1/2 * [0, omega'] (x) q, omega' in body frame.

    Positive sign as written in the paper. Used for eq.7-citing checks only —
    dynamics.state_derivative reimplements eq. (18)'s literal negative-sign
    variant directly rather than calling this function; see dynamics.py.
    """
    q2, q1d = _as_batch(q, 4)
    w2, w1d = _as_batch(omega_body, 3)
    zero = np.zeros(w2.shape[:-1] + (1,))
    omega_quat = np.concatenate([zero, w2], axis=-1)
    out = 0.5 * mul(omega_quat, q2)
    return _unbatch(out, q1d and w1d)


def rotate(q: FloatArr, v: FloatArr) -> FloatArr:
    """Eq. (8): w = q (x) [0, v] (x) q*."""
    q2, q1d = _as_batch(q, 4)
    v2, v1d = _as_batch(v, 3)
    zero = np.zeros(v2.shape[:-1] + (1,))
    v_quat = np.concatenate([zero, v2], axis=-1)
    out = mul(mul(q2, v_quat), conj(q2))
    return _unbatch(out[..., 1:], q1d and v1d)


def to_dcm(q: FloatArr) -> FloatArr:
    """Eq. (9)-(12): rotation matrix R with columns Rx(q), Ry(q), Rz(q), s.t.
    rotate(q, v) == R @ v. Transpose (eq. 13) rotates the frame instead of a point."""
    q2, was_1d = _as_batch(q, 4)
    q0, q1, q2_, q3 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
    rx = np.stack(
        [q0**2 + q1**2 - q2_**2 - q3**2, 2 * (q1 * q2_ + q0 * q3), 2 * (q1 * q3 - q0 * q2_)],
        axis=-1,
    )
    ry = np.stack(
        [2 * (q1 * q2_ - q0 * q3), q0**2 - q1**2 + q2_**2 - q3**2, 2 * (q2_ * q3 + q0 * q1)],
        axis=-1,
    )
    rz = np.stack(
        [2 * (q1 * q3 + q0 * q2_), 2 * (q2_ * q3 - q0 * q1), q0**2 - q1**2 - q2_**2 + q3**2],
        axis=-1,
    )
    out = np.stack([rx, ry, rz], axis=-1)  # columns = Rx, Ry, Rz
    return out[0] if was_1d else out


def from_axis_angle(axis: FloatArr, angle: float | FloatArr) -> FloatArr:
    """Eq. (14): q = cos(alpha/2) + u*sin(alpha/2), u = normalized axis."""
    ax2, ax1d = _as_batch(axis, 3)
    ax_norm = np.linalg.norm(ax2, axis=-1, keepdims=True)
    if np.any(ax_norm < 1e-12):
        raise ValueError("axis must be nonzero")
    u = ax2 / ax_norm
    ang = np.atleast_1d(np.asarray(angle, dtype=np.float64))
    half = ang / 2.0
    q0 = np.cos(half)
    vec = u * np.sin(half)[..., None]
    out = np.concatenate([q0[..., None], vec], axis=-1)
    return _unbatch(out, ax1d and np.isscalar(angle))


def from_euler(phi: float | FloatArr, theta: float | FloatArr, psi: float | FloatArr) -> FloatArr:
    """Eq. (15): quaternion from Euler angles (aerospace ZYX: roll phi, pitch theta, yaw psi)."""
    was_scalar = np.ndim(phi) == 0 and np.ndim(theta) == 0 and np.ndim(psi) == 0
    phi = np.atleast_1d(np.asarray(phi, dtype=np.float64))
    theta = np.atleast_1d(np.asarray(theta, dtype=np.float64))
    psi = np.atleast_1d(np.asarray(psi, dtype=np.float64))
    cph, sph = np.cos(phi / 2), np.sin(phi / 2)
    cth, sth = np.cos(theta / 2), np.sin(theta / 2)
    cps, sps = np.cos(psi / 2), np.sin(psi / 2)
    q0 = cph * cth * cps + sph * sth * sps
    q1 = sph * cth * cps - cph * sth * sps
    q2_ = cph * sth * cps + sph * cth * sps
    q3 = cph * cth * sps - sph * sth * cps
    out = np.stack([q0, q1, q2_, q3], axis=-1)
    return out[0] if was_scalar else out


def to_euler(q: FloatArr) -> tuple[FloatArr, FloatArr, FloatArr]:
    """Eq. (16): Euler angles (phi, theta, psi) from quaternion. PLOTTING/LOGGING ONLY —
    never call from dynamics.py or controller.py."""
    q2, was_1d = _as_batch(q, 4)
    q0, q1, q2_, q3 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
    phi = np.arctan2(2 * (q0 * q1 + q2_ * q3), q0**2 - q1**2 - q2_**2 + q3**2)
    theta = np.arcsin(np.clip(2 * (q0 * q2_ - q3 * q1), -1.0, 1.0))
    psi = np.arctan2(2 * (q0 * q3 + q1 * q2_), q0**2 + q1**2 - q2_**2 - q3**2)
    if was_1d:
        return float(phi[0]), float(theta[0]), float(psi[0])
    return phi, theta, psi
