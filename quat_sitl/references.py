"""Reference generators for the three scenarios. Each is scalar-t-in, single
quaternion-out, composed via quaternion.from_euler/from_axis_angle so q_ref is
always a valid unit quaternion (never built via raw quaternion addition).

The paper describes these scenarios qualitatively without literal numeric timing/
phase values; the specific choices below are documented assumptions (see README
"Results vs Paper").
"""
from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from . import quaternion as quat

FloatArr = NDArray[np.float64]

DEFAULT_STEP_AXIS_TIMES = {"x": 1.0, "y": 5.0, "z": 9.0}


def step_reference(
    t: float, axis_times: dict[str, float] = DEFAULT_STEP_AXIS_TIMES, amplitude: float = 1.0
) -> FloatArr:
    """1 rad step around each axis, staggered at different time instants (paper
    Figs 3-4). Default staggering x@1s, y@5s, z@9s over a 15s run — not given
    literally by the paper, chosen for >=4s settling margin per step."""
    phi = amplitude if t >= axis_times["x"] else 0.0
    theta = amplitude if t >= axis_times["y"] else 0.0
    psi = amplitude if t >= axis_times["z"] else 0.0
    return quat.from_euler(phi, theta, psi)


def sine_reference(
    t: float,
    amplitude: float = 0.5,
    freq_rad_s: float = 1.0,
    phase_y: float = 0.0,
    phase_z: float = np.pi / 2,
) -> FloatArr:
    """0.5 rad amplitude, 1 rad/s sine tracking (paper Figs 5-6), phase-shifted on
    z so the torques are not all in phase. phase_z=pi/2 is a documented assumption
    (paper gives no numeric phase value, only the qualitative requirement)."""
    phi = amplitude * np.sin(freq_rad_s * t)
    theta = amplitude * np.sin(freq_rad_s * t + phase_y)
    psi = amplitude * np.sin(freq_rad_s * t + phase_z)
    return quat.from_euler(phi, theta, psi)


def flip_reference(t: float, ramp_duration: float = 2.0, axis: FloatArr | None = None) -> FloatArr:
    """Ramp 0 -> 2*pi rad about `axis` (default x) over ramp_duration seconds
    (paper Figs 7-8), then holds at 2*pi. ramp_duration=2.0s is a documented
    assumption (paper gives no literal ramp duration)."""
    if axis is None:
        axis = np.array([1.0, 0.0, 0.0])
    angle = min(t / ramp_duration, 1.0) * 2.0 * np.pi
    return quat.from_axis_angle(axis, angle)
