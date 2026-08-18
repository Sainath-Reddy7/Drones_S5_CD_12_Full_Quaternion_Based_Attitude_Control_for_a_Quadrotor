"""The paper's own three scenarios (step / sine / flip), reused directly from
quat_sitl.references -- same functions, same documented assumptions
(staggered step times, sine phase offsets, flip ramp duration; see that
module and the quat_sitl README's "Results vs Paper" section) -- exposed here
as the AUTO_STEP/AUTO_SINE/AUTO_FLIP flight modes.
"""
from __future__ import annotations

from typing import Callable

from quat_sitl import references

from ..modes import FlightMode

# (reference function, shortest_path). AUTO_FLIP disables shortest_path so
# the controller commits to the full rotation, exactly as flip_360.py does.
_SCENARIOS: dict = {
    FlightMode.AUTO_STEP: (references.step_reference, True),
    FlightMode.AUTO_SINE: (references.sine_reference, True),
    FlightMode.AUTO_FLIP: (references.flip_reference, False),
}


def reference_fn_for(mode: FlightMode) -> Callable[[float], tuple]:
    fn, _ = _SCENARIOS[mode]
    return lambda t: tuple(fn(t))


def shortest_path_for(mode: FlightMode) -> bool:
    _, shortest_path = _SCENARIOS[mode]
    return shortest_path


def is_autopilot_mode(mode: FlightMode) -> bool:
    return mode in _SCENARIOS
