"""Shared bridge package for the four simulator adapters (FRP.md section 3).

Exports the frozen paper controller bridge, the scenario registry, and the
uniform telemetry/metrics used by the cross-simulator benchmark."""
from .bridge import (
    AltitudeHold,
    PAPER_VEHICLE,
    QuatBridge,
    RotorGeometry,
    TORQUE_LIMIT,
    mix_zup,
    thrusts_to_wrench_zup,
)
from .scenarios import SCENARIOS, default_duration, reference_quat, shortest_path
from .telemetry import COLUMNS, RunLog, summarize

__all__ = [
    "AltitudeHold",
    "PAPER_VEHICLE",
    "QuatBridge",
    "RotorGeometry",
    "TORQUE_LIMIT",
    "mix_zup",
    "thrusts_to_wrench_zup",
    "SCENARIOS",
    "reference_quat",
    "shortest_path",
    "default_duration",
    "COLUMNS",
    "RunLog",
    "summarize",
]
