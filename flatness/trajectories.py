"""Flat-output trajectories — Choutri & Lagha 2017, Section V study case.

Flat outputs (Eq. 13): sigma = (x, y, z, psi) plus derivatives up to the
second order, which is all the flatness map of Eqs. (15)/(19)-(21) needs.

The paper's study case: a circular path at fixed altitude 1 m starting from
p0 = (-1, 0, 0) — i.e. center (0, 0, 1), radius 1 m (the start point is on
the circle, which fixes the geometry). The paper does not give the circle
period; the default tangential speed is 0.5 m/s. The X/Y responses in the
paper's Figs. 4-5 lag about 1 s "due to the altitude settling time", i.e.
the vehicle starts on the ground and climbs onto the circle: a smooth
(minimum-jerk) altitude ramp 0 -> 1 m reproduces this.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FlatRef:
    """Flat outputs and derivatives at one time instant."""

    p: np.ndarray      # (3,) position [m]
    v: np.ndarray      # (3,) velocity [m/s]
    a: np.ndarray      # (3,) acceleration [m/s^2]
    psi: float         # yaw [rad]
    psi_dot: float     # yaw rate [rad/s]


def _min_jerk(t: float, T: float, total: float) -> tuple[float, float, float]:
    """Minimum-jerk scalar profile s(t), s_dot, s_ddot over [0, T] in [0, 1]."""
    if t >= T:
        return total, 0.0, 0.0
    x = t / T
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    sd = (30 * x**2 - 60 * x**3 + 30 * x**4) / T
    sdd = (60 * x - 180 * x**2 + 120 * x**3) / T**2
    return total * s, total * sd, total * sdd


class CircleTrajectory:
    """The paper's circular reference: radius 1 m at altitude 1 m from
    (-1, 0, 0), climbed onto with a smooth altitude ramp. The angular rate
    also ramps 0 -> omega with the same profile, so the reference starts
    AT REST at the start point (the paper's Figs. 4-5 show X/Y following
    with ~1 s delay from rest; a full-speed reference at t=0 would demand
    an instantaneous tilt beyond the mixer's hover headroom)."""

    def __init__(self, radius: float = 1.0, alt: float = 1.0,
                 speed: float = 0.5, climb_time: float = 2.0,
                 yaw_follow: bool = False):
        self.r = float(radius)
        self.alt = float(alt)
        self.omega = float(speed) / float(radius)   # rad/s around +z
        self.climb_time = float(climb_time)
        self.yaw_follow = yaw_follow  # heading along the path (default: hold 0)

    def _theta(self, t: float) -> tuple[float, float, float]:
        """Angle, rate, acceleration on the circle: rate ramps 0 -> omega
        with the min-jerk profile over the climb, then holds omega."""
        T, w = self.climb_time, self.omega
        if t >= T:
            return w * (t - T / 2.0), w, 0.0
        x = t / T
        theta = w * T * (2.5 * x**4 - 3.0 * x**5 + x**6)   # integral of min-jerk
        theta_dot = w * (10 * x**3 - 15 * x**4 + 6 * x**5)
        theta_ddot = w * (30 * x**2 - 60 * x**3 + 30 * x**4) / T
        return theta, theta_dot, theta_ddot

    def evaluate(self, t: float) -> FlatRef:
        """sigma(t) = (x, y, z, psi) with derivatives, starting at (-1, 0, 0):
        p(t) = (-r cos(th), r sin(th), z(t)) — counter-clockwise."""
        th, th_d, th_dd = self._theta(t)
        p = np.array([-self.r * np.cos(th), self.r * np.sin(th), 0.0])
        v = self.r * np.array([th_d * np.sin(th), th_d * np.cos(th), 0.0])
        a = self.r * np.array(
            [th_dd * np.sin(th) + th_d**2 * np.cos(th),
             th_dd * np.cos(th) - th_d**2 * np.sin(th), 0.0])
        z, z_dot, z_ddot = _min_jerk(t, self.climb_time, self.alt)
        p[2], v[2], a[2] = z, z_dot, z_ddot
        psi = 0.0
        psi_dot = 0.0
        if self.yaw_follow:
            psi = float(np.arctan2(v[1], v[0]))  # heading along the tangent
            psi_dot = th_d
        return FlatRef(p=p, v=v, a=a, psi=float(psi), psi_dot=float(psi_dot))

    @property
    def duration(self) -> float:
        """Two full circles — enough to show steady tracking (paper Figs. 4-7)."""
        return 2.0 * 2.0 * np.pi / self.omega + self.climb_time
