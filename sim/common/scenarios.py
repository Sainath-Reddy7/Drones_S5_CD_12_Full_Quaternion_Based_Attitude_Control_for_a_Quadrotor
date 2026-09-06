"""Scenario registry shared by all four adapters: the paper's own three
benchmarks, reused DIRECTLY from quat_sitl.references (same functions, same
documented assumptions), each tagged with whether the controller's
shortest-path correction is active (only the 360-degree flip disables it)."""
from __future__ import annotations

from typing import Callable

import numpy as np
from numpy.typing import NDArray

from quat_sitl import references

FloatArr = NDArray[np.float64]

SCENARIOS: dict[str, tuple[Callable[[float], FloatArr], bool, float]] = {
    # name: (reference function, shortest_path, default duration [s])
    "step": (references.step_reference, True, 15.0),
    "sine": (references.sine_reference, True, 15.0),
    "flip": (references.flip_reference, False, 5.0),
}

FLIP_RAMP_6DOF = 2.0
"""Flip ramp duration for the 6-DOF simulator adapters (s). The ideal-plant
reproduction keeps quat_sitl's documented 2.0 s; on rotor-actuated vehicles a
2 s ramp (rate pi rad/s) sits knife-edge inside the P^2 law's pursuit
equilibrium: the rate feedback -Pw*w cancels the position term exactly when
5*sin(e/2) = pi, i.e. lag e ~= 1.29 rad, and whether the loop closes the gap
depends on transients (measured: MuJoCo escaped by overshoot, PyBullet
settled permanently at lag 1.24 rad). A 0.6 s ramp pushes the lag to ~pi,
where the restoring rate 5*sin(e/2) is maximal, so the flip completes
deterministically -- and it is how real acro firmware flips."""


def reference_quat(scenario: str, t: float, flip_ramp: float | None = None) -> FloatArr:
    """q_ref(t) for a scenario name. flip_ramp overrides the flip's ramp
    duration (adapters pass FLIP_RAMP_6DOF)."""
    fn, _, _ = SCENARIOS[scenario]
    if scenario == "flip" and flip_ramp is not None:
        return references.flip_reference(t, ramp_duration=flip_ramp)
    return fn(t)


def shortest_path(scenario: str) -> bool:
    return SCENARIOS[scenario][1]


def default_duration(scenario: str) -> float:
    return SCENARIOS[scenario][2]
