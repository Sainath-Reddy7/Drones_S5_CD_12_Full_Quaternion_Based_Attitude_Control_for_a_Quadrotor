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
"""Flip ramp duration for the 6-DOF simulator adapters (s) -- the same 2.0 s
documented assumption quat_sitl's ideal-plant reproduction uses. Why it stays
2.0 and is not "tuned per engine": on rotor-actuated vehicles the P^2 law's
pursuit equilibrium (rate feedback -Pw*w balancing the position term, i.e.
vehicle rate 5*sin(e/2) at lag e) means a pi rad/s ramp is trackable only at
lag e* = 2*asin(pi/5) ~= 1.36 rad -- the vehicle crosses 2 pi just after the
reference, so completion is structurally marginal (knife-edge) rather than a
tuning matter. Measured: MuJoCo completes it deterministically airborne
(seeds 0-2: phi settles 2.87-2.97 s, dip to ~40 m of a 60 m start); PyBullet
enters a limit cycle near phi ~= 2.1 rad and unwinds -- documented as finding
4 in sim/README.md. Shortening the ramp does not help: the reference then
reaches identity (2 pi = q = [1,0,0,0]) while the vehicle still lags, and the
quaternion pulls it back the SHORT way, unwinding the flip."""


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
