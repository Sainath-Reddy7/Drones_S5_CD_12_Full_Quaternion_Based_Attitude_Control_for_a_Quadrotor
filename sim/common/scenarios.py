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


def reference_quat(scenario: str, t: float) -> FloatArr:
    """q_ref(t) for a scenario name."""
    fn, _, _ = SCENARIOS[scenario]
    return fn(t)


def shortest_path(scenario: str) -> bool:
    return SCENARIOS[scenario][1]


def default_duration(scenario: str) -> float:
    return SCENARIOS[scenario][2]
